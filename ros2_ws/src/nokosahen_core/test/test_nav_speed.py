"""自律走行(Nav2)の最高速度を1つの設定にまとめた機能(2026-10-07)のテスト。

ROS 2が無いPCでも動くよう、rclpy系モジュールをスタブしてdashboard_commonを読み込み、
実際にHTTPサーバーを立てて /api/speed_settings・/api/nav_speed を叩く(test_voice_settings.pyと
同じ方式)。実機のNav2・Jetson・ロボットには一切触れない(パラメータクライアントはフェイク)。
確認すること:
- 変換(純粋関数): v=0.22で元のyamlと完全一致/0.5/1.0/1.1、加減速の上限、先読み>=停止距離+余裕、
  後退・旋回は不変、配列の形、他のキーは全て保持、入力dictは非破壊、起動時と実行時の値が一致
- 記憶JSON: 検証(古い/範囲外/NaN/壊れ)・launchの優先順位
- launchファイル: nav_max_speed引数(既定0.5)があり、nav2_bringupへは書き換え後のparamsを渡す
- コントローラ: 反映順序(上げ=衝突監視→追従→smoother)、部分失敗、タイムアウト、Nav2未起動(409+記憶)
- HTTP: 正常/NaN 400/guest 403/丸めメッセージ/0.6超はconfirm_high必須/Nav2未起動409、
  ダッシュボードの速度変更はcmd_velを一切publishしない・ソフト緊急停止の状態を変えない

実行: python ros2_ws/src/nokosahen_core/test/test_nav_speed.py
"""

import ast
import copy
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import types
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest import mock

TEST_DIR = Path(__file__).resolve().parent
PKG = TEST_DIR.parent
REPO = PKG.parents[2]
BRINGUP = REPO / "ros2_ws" / "src" / "nokosahen_bringup"
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(TEST_DIR))


def _install_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    class Msg:
        def __init__(self, *a, **k):
            self.data = None

    mod("geometry_msgs")
    mod("geometry_msgs.msg", PoseStamped=Msg, Twist=Msg)
    mod("nav_msgs")
    mod("nav_msgs.msg", Odometry=Msg)
    mod("std_msgs")
    mod("std_msgs.msg", Bool=Msg, Float32MultiArray=Msg, String=Msg, Empty=Msg)


try:
    import geometry_msgs.msg  # noqa: F401
    import nav_msgs.msg  # noqa: F401
    import std_msgs.msg as _std  # noqa: F401
except Exception:  # noqa: BLE001
    _install_stubs()

import yaml  # noqa: E402

from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import nav_speed as ns  # noqa: E402

NAV2_YAML = BRINGUP / "config" / "nav2_params_jetson.yaml"
GRID = [0.1, 0.15, 0.22, 0.3, 0.5, 0.6, 0.75, 0.9, 1.0, 1.1]


def original_params():
    return yaml.safe_load(NAV2_YAML.read_text(encoding="utf-8"))


def flatten(d, prefix=""):
    out = {}
    for k, v in d.items():
        key = f"{prefix}{k}"
        if isinstance(v, dict):
            out.update(flatten(v, key + "."))
        else:
            out[key] = v
    return out


class TransformTests(unittest.TestCase):
    def test_normalize(self):
        self.assertEqual(ns.normalize_nav_speed(0.5), 0.5)
        self.assertEqual(ns.normalize_nav_speed(5), 1.1)
        self.assertEqual(ns.normalize_nav_speed(0), 0.1)
        self.assertEqual(ns.normalize_nav_speed(-3), 0.1)
        self.assertEqual(ns.normalize_nav_speed("0.4"), 0.4)
        for bad in (float("nan"), float("inf"), float("-inf"), "abc", None, True, [1], {}):
            with self.assertRaises(ValueError, msg=repr(bad)):
                ns.normalize_nav_speed(bad)

    def test_v022_reproduces_original_file_exactly(self):
        orig = original_params()
        self.assertEqual(ns.apply_nav_speed_to_params(orig, 0.22), orig)

    def test_derived_values(self):
        d = ns.derive_nav_speed_values(0.22)
        self.assertEqual((d["accel"], d["decel"], d["sim_time"], d["vx_samples"], d["time_before_collision"], d["costmap_size"]),
                         (0.45, 0.45, 1.4, 16, 1.2, 3))
        d = ns.derive_nav_speed_values(0.5)
        self.assertEqual(d["accel"], 0.8)  # 手動ランプ(0.8)で頭打ち
        self.assertAlmostEqual(d["decel"], 1.023, places=3)
        self.assertEqual(d["time_before_collision"], 1.3)
        for v in (1.0, 1.1):
            d = ns.derive_nav_speed_values(v)
            self.assertEqual((d["accel"], d["decel"]), (0.8, 2.0))
            self.assertEqual(d["time_before_collision"], round(0.8 + v, 2))
            self.assertGreaterEqual(d["costmap_size"], 4)

    def test_invariants_over_grid(self):
        for v in GRID:
            d = ns.derive_nav_speed_values(v)
            self.assertLessEqual(d["accel"], 0.8, v)
            self.assertLessEqual(d["decel"], 2.0, v)
            self.assertGreaterEqual(d["time_before_collision"], 1.2, v)
            self.assertLessEqual(d["sim_time"], ns.SIM_TIME_MAX)
            self.assertGreaterEqual(d["sim_time"], 1.4)
            # 先読み距離 >= 停止距離 + 余裕
            self.assertGreaterEqual(v * d["sim_time"] + 1e-9, d["stop_distance"] + ns.LOOKAHEAD_MARGIN_M - 0.005, v)
            # ローカルコストマップ >= 2*(先読み+ロボット半径)
            self.assertGreaterEqual(d["costmap_size"], 2 * (v * d["sim_time"] + 0.28) - 1e-9, v)
            self.assertLessEqual(d["vx_samples"], 24)

    def test_hard_limits(self):
        self.assertEqual(ns.derive_nav_speed_values(99)["speed"], 1.1)
        self.assertEqual(ns.derive_nav_speed_values(0.0)["speed"], 0.1)
        with self.assertRaises(ValueError):
            ns.derive_nav_speed_values(float("nan"))

    def test_rewrite_05_and_preserves_everything_else(self):
        orig = original_params()
        snap = copy.deepcopy(orig)
        new = ns.apply_nav_speed_to_params(orig, 0.5)
        self.assertEqual(orig, snap)  # 入力は非破壊
        fp = new["controller_server"]["ros__parameters"]["FollowPath"]
        self.assertEqual((fp["max_vel_x"], fp["max_speed_xy"], fp["acc_lim_x"], fp["vx_samples"]), (0.5, 0.5, 0.8, 19))
        self.assertAlmostEqual(fp["decel_lim_x"], -1.023, places=3)
        self.assertEqual(fp["max_vel_theta"], 1.8)  # 【2026-10-10】旋回を速く(0.85 -> 1.8)。直進速度vとは独立
        self.assertEqual(fp["acc_lim_theta"], 3.0)
        sm = new["velocity_smoother"]["ros__parameters"]
        self.assertEqual(sm["max_velocity"], [0.5, 0.0, 1.8])
        self.assertEqual(sm["min_velocity"], [ns.REVERSE_SPEED, 0.0, -1.8])  # 後退はBackUpの速さ(通常の走行と同じ)。速度vでは変えない
        self.assertEqual(sm["max_accel"], [0.8, 0.0, 3.0])
        self.assertAlmostEqual(sm["max_decel"][0], -1.023, places=3)
        self.assertEqual(sm["max_decel"][1:], [0.0, -4.0])
        for k in ("max_velocity", "min_velocity", "max_accel", "max_decel"):
            self.assertEqual(len(sm[k]), 3)
        self.assertEqual(new["collision_monitor"]["ros__parameters"]["FootprintApproach"]["time_before_collision"], 1.3)
        # 変わったキーは速度関連だけ
        a, b = flatten(orig), flatten(new)
        self.assertEqual(set(a), set(b))
        changed = {k for k in a if a[k] != b[k]}
        allowed = {
            "controller_server.ros__parameters.FollowPath." + k
            for k in ("max_vel_x", "max_speed_xy", "acc_lim_x", "decel_lim_x", "vx_samples")
        } | {"velocity_smoother.ros__parameters." + k for k in ("max_velocity", "max_accel", "max_decel")} | {
            "collision_monitor.ros__parameters.FootprintApproach.time_before_collision"}
        self.assertTrue(changed <= allowed, changed - allowed)

    def test_rewrite_high_speeds(self):
        for v, tbc, size in ((1.0, 1.8, 4), (1.1, 1.9, 4)):
            new = ns.apply_nav_speed_to_params(original_params(), v)
            fp = new["controller_server"]["ros__parameters"]["FollowPath"]
            self.assertEqual((fp["max_vel_x"], fp["max_speed_xy"], fp["acc_lim_x"], fp["decel_lim_x"]), (v, v, 0.8, -2.0))
            self.assertEqual(new["velocity_smoother"]["ros__parameters"]["max_velocity"][0], v)
            self.assertEqual(new["velocity_smoother"]["ros__parameters"]["min_velocity"][0], ns.REVERSE_SPEED)
            self.assertEqual(new["collision_monitor"]["ros__parameters"]["FootprintApproach"]["time_before_collision"], tbc)
            lc = new["local_costmap"]["local_costmap"]["ros__parameters"]
            self.assertEqual((lc["width"], lc["height"]), (size, size))
        # 上限超えは1.1へ
        self.assertEqual(ns.apply_nav_speed_to_params(original_params(), 7.0)["controller_server"]["ros__parameters"]["FollowPath"]["max_vel_x"], 1.1)
        with self.assertRaises(ValueError):
            ns.apply_nav_speed_to_params(original_params(), float("nan"))

    def test_other_yaml_untouched_sections_kept(self):
        # 速度に無関係な節(amcl・planner・docking等)はそのまま
        orig = original_params()
        new = ns.apply_nav_speed_to_params(orig, 1.0)
        for node in ("amcl", "planner_server", "docking_server", "behavior_server", "global_costmap", "bt_navigator"):
            self.assertEqual(new[node], orig[node], node)

    def test_runtime_values_match_launch_time_values(self):
        for v in GRID:
            new = ns.apply_nav_speed_to_params(original_params(), v)
            arrays = {k: new["velocity_smoother"]["ros__parameters"][k] for k in ("max_velocity", "max_accel", "max_decel")}
            live = ns.live_parameters(v, arrays)
            for node, values in live.items():
                flat = flatten(new[node]["ros__parameters"])
                for name, value in values.items():
                    self.assertEqual(flat[name], value, (v, node, name))

    def test_live_arrays_keep_current_theta(self):
        live = ns.live_parameters(0.8, {"max_velocity": [0.1, 0.0, 0.5], "max_accel": [0.1, 0.0, 0.9], "max_decel": [-0.1, 0.0, -0.9]})
        sm = live[ns.NODE_SMOOTHER]
        self.assertEqual(sm["max_velocity"], [0.8, 0.0, 0.5])
        self.assertEqual(sm["max_accel"][1:], [0.0, 0.9])
        self.assertEqual(sm["max_decel"][1:], [0.0, -0.9])
        self.assertNotIn("min_velocity", sm)  # 後退は触らない

    def test_write_rewritten_params_roundtrip(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = ns.write_rewritten_params(str(NAV2_YAML), 0.5, os.path.join(tmp, "n.yaml"))
            loaded = yaml.safe_load(Path(out).read_text(encoding="utf-8"))
            # 起動時の書き換え = 速度 + 目的地精度 + 機体端からの距離 + BTの絶対パス(yamlは同値を静的に持つ)
            expected = ns.apply_bt_to_params(ns.apply_nav_speed_to_params(original_params(), 0.5), str(NAV2_YAML.parent))
            self.assertEqual(loaded, expected)
            self.assertTrue(os.path.isabs(loaded["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"]))
            self.assertNotIn(b"\r", Path(out).read_bytes())


class SavedSpeedTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.tmp.name, "s.json")

    def tearDown(self):
        self.tmp.cleanup()

    def test_roundtrip_and_validation(self):
        self.assertIsNone(ns.load_saved_nav_speed(self.path))
        self.assertTrue(ns.save_nav_speed(0.8, self.path, now=1000.0))
        self.assertEqual(ns.load_saved_nav_speed(self.path, now=1100.0), 0.8)
        self.assertIsNone(ns.load_saved_nav_speed(self.path, now=1000.0 + ns.SAVED_MAX_AGE_SEC + 1))  # 古い
        self.assertIsNone(ns.load_saved_nav_speed(self.path, now=500.0))  # 未来の日付
        for text in ("{bad", "[1]", '{"nav_max_speed_mps": NaN, "saved_at": 1000}',
                     '{"nav_max_speed_mps": 3.0, "saved_at": 1000}', '{"nav_max_speed_mps": 0.01, "saved_at": 1000}',
                     '{"nav_max_speed_mps": true, "saved_at": 1000}', '{"nav_max_speed_mps": 0.5}', '{"nav_max_speed_mps": "x", "saved_at": 1}'):
            Path(self.path).write_text(text, encoding="utf-8")
            self.assertIsNone(ns.load_saved_nav_speed(self.path, now=1000.0), text)
        ns.clear_saved_nav_speed(self.path)
        ns.clear_saved_nav_speed(self.path)  # 無くても例外にならない

    def test_resolve_priority(self):
        self.assertEqual(ns.resolve_launch_nav_speed("0.5", True, self.path), (0.5, "arg"))
        ns.save_nav_speed(0.9, self.path)
        self.assertEqual(ns.resolve_launch_nav_speed("0.5", True, self.path), (0.9, "saved"))
        self.assertEqual(ns.resolve_launch_nav_speed("0.5", False, self.path), (0.5, "arg"))
        self.assertEqual(ns.resolve_launch_nav_speed("9", False, self.path), (1.1, "arg"))
        self.assertEqual(ns.resolve_launch_nav_speed("abc", False, self.path), (0.7, "default"))
        self.assertEqual(ns.resolve_launch_nav_speed("nan", False, self.path), (0.7, "default"))


class LaunchFileTests(unittest.TestCase):
    def test_yaml_default_is_seven_tenths_meter_per_second(self):
        # 【2026-10-10】0.5 -> 0.7
        self.assertEqual(ns.read_yaml_default(str(BRINGUP / "config" / "nokosahen.yaml")), 0.7)
        self.assertEqual(ns.read_yaml_default("/no/such/file.yaml"), 0.7)
        data = yaml.safe_load((BRINGUP / "config" / "nokosahen.yaml").read_text(encoding="utf-8"))
        self.assertEqual(data["nokosahen_launch"]["ros__parameters"]["nav_max_speed"], 0.7)
        self.assertEqual(ns.NAV_SPEED_DEFAULT, 0.7)

    def test_launch_files_declare_arg_and_use_rewritten_params(self):
        for name in ("mapping_mode.launch.py", "localization_mode.launch.py"):
            text = (BRINGUP / "launch" / name).read_text(encoding="utf-8")
            ast.parse(text)
            self.assertRegex(text, r'DeclareLaunchArgument\(\s*"nav_max_speed"')
            self.assertIn("read_yaml_default", text)  # 既定はnokosahen.yaml(0.7)
            self.assertIn("write_rewritten_params", text)
            self.assertIn("OpaqueFunction", text)
            self.assertIn('"use_saved_nav_speed"', text)
            # nav2_bringupへ渡すparams_fileは書き換え後のコピー(元のnav2_params_fileを直接渡さない)
            self.assertIn('"params_file": rewritten', text)
            self.assertNotIn('"params_file": nav2_params_file', text)
            self.assertNotIn("cmd_vel", text)

    def test_launch_helper_runs_without_ros_for_default(self):
        speed, source = ns.resolve_launch_nav_speed(str(ns.read_yaml_default(str(BRINGUP / "config" / "nokosahen.yaml"))), False)
        self.assertEqual((speed, source), (0.7, "arg"))


def make_initial_store():
    store = {n: dict(v) for n, v in ns.live_parameters(0.22).items()}
    store[ns.NODE_LOCAL_COSTMAP] = {"width": 3, "height": 3}
    return store


class FakeClient:
    """RclpyParamClientと同じ形のフェイク。呼び出しを記録し、cmd_vel等には触れない。"""

    def __init__(self, ready=True):
        self.ready = ready
        self.store = make_initial_store()
        self.calls = []
        self.timeout_nodes = set()
        self.reject_nodes = set()
        self.reject_only_name = None

    def service_ready(self, node, timeout):
        return self.ready

    def set_parameters(self, node, values, timeout):
        self.calls.append(("set", node, dict(values), timeout))
        if node in self.timeout_nodes:
            raise TimeoutError("3秒以内に応答がありません")
        res = {}
        for n, v in values.items():
            if node in self.reject_nodes or n == self.reject_only_name:
                res[n] = (False, "rejected by fake")
            else:
                self.store[node][n] = v
                res[n] = (True, "")
        return res

    def get_parameters(self, node, names, timeout):
        self.calls.append(("get", node, list(names), timeout))
        if node in self.timeout_nodes:
            raise TimeoutError("timeout")
        return {n: self.store[node][n] for n in names if n in self.store.get(node, {})}

    def set_order(self):
        return [c[1] for c in self.calls if c[0] == "set"]


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.tmp.name, "s.json")
        self.client = FakeClient()
        self.ctrl = ns.NavSpeedController(client=self.client, save_path=self.path)

    def tearDown(self):
        self.tmp.cleanup()

    def test_default_status(self):
        st = self.ctrl.status()
        self.assertEqual((st["nav_max_speed_mps"], st["nav_max_speed_source"]), (0.7, "default"))
        self.assertEqual((st["nav_speed_min"], st["nav_speed_max"]), (0.1, 1.1))

    def test_apply_success_raising_order_and_readback(self):
        code, r = self.ctrl.apply(0.5)
        self.assertEqual(code, 200)
        self.assertTrue(r["ok"])
        self.assertEqual(self.client.set_order(), [ns.NODE_COLLISION, ns.NODE_CONTROLLER, ns.NODE_SMOOTHER])
        self.assertEqual(self.client.store[ns.NODE_CONTROLLER]["FollowPath.max_vel_x"], 0.5)
        self.assertEqual(self.client.store[ns.NODE_SMOOTHER]["max_velocity"], [0.5, 0.0, 1.8])
        self.assertEqual(r["read_back"][ns.NODE_CONTROLLER]["FollowPath.max_speed_xy"], 0.5)
        self.assertTrue(all(n["ok"] for n in r["nodes"]))
        self.assertTrue(all(c[3] == 3.0 for c in self.client.calls))  # 全呼び出し3秒タイムアウト
        self.assertEqual(self.ctrl.status()["nav_max_speed_source"], "live")
        self.assertIn("0.5", r["message"])

    def test_lowering_order(self):
        self.ctrl.apply(1.0, confirm_high=True)
        self.client.calls.clear()
        code, r = self.ctrl.apply(0.4)
        self.assertEqual(code, 200)
        self.assertEqual(self.client.set_order(), [ns.NODE_SMOOTHER, ns.NODE_CONTROLLER, ns.NODE_COLLISION])
        self.assertEqual(self.client.store[ns.NODE_SMOOTHER]["max_velocity"][0], 0.4)
        self.assertEqual(self.client.store[ns.NODE_SMOOTHER]["min_velocity"] if "min_velocity" in self.client.store[ns.NODE_SMOOTHER] else None, None)

    def test_confirm_required_above_06_and_nothing_sent(self):
        for v in (0.61, 1.0, 1.1, 50):
            code, r = self.ctrl.apply(v)
            self.assertEqual(code, 400, v)
            self.assertTrue(r["confirm_required"])
        self.assertEqual(self.client.calls, [])
        self.assertEqual(self.ctrl.apply(0.6)[0], 200)  # 0.6ちょうどは確認不要
        self.assertEqual(self.ctrl.apply(0.9, confirm_high="yes")[0], 400)  # trueそのものだけ有効

    def test_clamp_message(self):
        code, r = self.ctrl.apply(0.01)
        self.assertEqual(code, 200)
        self.assertTrue(r["clamped"])
        self.assertEqual(r["nav_max_speed_mps"], 0.1)
        self.assertIn("丸めました", r["message"])
        code, r = self.ctrl.apply(9.0, confirm_high=True)
        self.assertEqual(r["nav_max_speed_mps"], 1.1)
        self.assertIn("丸めました", r["message"])

    def test_nonfinite_is_400(self):
        for bad in (float("nan"), float("inf"), "x", None, True):
            self.assertEqual(self.ctrl.apply(bad)[0], 400, repr(bad))
        self.assertEqual(self.client.calls, [])

    def test_not_running_is_409_and_remembered(self):
        self.client.ready = False
        code, r = self.ctrl.apply(0.5)
        self.assertEqual(code, 409)
        self.assertEqual(r["error"], "nav2_not_running")
        self.assertIn("起動していません", r["message"])
        self.assertTrue(r["saved_for_next_launch"])
        self.assertEqual(ns.load_saved_nav_speed(self.path), 0.5)
        self.assertEqual(self.ctrl.status()["nav_max_speed_source"], "saved")
        self.assertEqual(self.client.set_order(), [])
        # 確認が必要な速度は、未起動でも確認なしでは記憶しない
        os.remove(self.path)
        self.assertEqual(self.ctrl.apply(0.9)[0], 400)
        self.assertFalse(os.path.exists(self.path))

    def test_success_clears_saved_value(self):
        ns.save_nav_speed(0.9, self.path)
        self.assertEqual(self.ctrl.apply(0.5)[0], 200)
        self.assertFalse(os.path.exists(self.path))  # 高速が次回起動へ持ち越されない

    def test_partial_failure_reported_per_node(self):
        self.client.timeout_nodes.add(ns.NODE_COLLISION)
        code, r = self.ctrl.apply(0.5)
        self.assertEqual(code, 200)
        self.assertFalse(r["ok"])
        by = {n["node"]: n for n in r["nodes"]}
        self.assertFalse(by[ns.NODE_COLLISION]["ok"])
        self.assertIn("TimeoutError", by[ns.NODE_COLLISION]["detail"])
        self.assertTrue(by[ns.NODE_CONTROLLER]["ok"] and by[ns.NODE_SMOOTHER]["ok"])
        self.assertIn("一部のノード", r["message"])
        self.assertIn("衝突監視", r["message"])
        self.assertEqual(self.ctrl.status()["nav_max_speed_source"], "default")  # 全成功でないので「反映済み」にしない

    def test_rejected_parameter_reported(self):
        self.client.reject_only_name = "FollowPath.sim_time"
        code, r = self.ctrl.apply(0.5)
        by = {n["node"]: n for n in r["nodes"]}
        self.assertFalse(by[ns.NODE_CONTROLLER]["ok"])
        self.assertIn("FollowPath.sim_time", by[ns.NODE_CONTROLLER]["detail"])

    def test_smoother_unreadable_is_not_overwritten_blindly(self):
        self.client.timeout_nodes.add(ns.NODE_SMOOTHER)
        code, r = self.ctrl.apply(0.5)
        by = {n["node"]: n for n in r["nodes"]}
        self.assertFalse(by[ns.NODE_SMOOTHER]["ok"])
        self.assertNotIn(ns.NODE_SMOOTHER, self.client.set_order())

    def test_costmap_warning(self):
        code, r = self.ctrl.apply(1.0, confirm_high=True)
        self.assertIn("ローカルコストマップ", r["message"])  # 実行中の3mは1.0 m/sには小さい(次回起動で広がる)
        code, r = self.ctrl.apply(0.5)
        self.assertNotIn("ローカルコストマップ", r["message"])

    def test_read_back_not_running(self):
        self.client.ready = False
        self.assertEqual(self.ctrl.read_back(), {"running": False, "nodes": {}})
        self.client.ready = True
        rb = self.ctrl.read_back()
        self.assertTrue(rb["running"])
        self.assertEqual(rb["nodes"][ns.NODE_CONTROLLER]["FollowPath.max_vel_x"], 0.22)


class FakeNode:
    def create_publisher(self, typ, topic, depth):
        pub = mock.Mock(name=f"pub{topic}")
        self.pubs = getattr(self, "pubs", {})
        self.pubs[topic] = pub
        return pub

    def create_subscription(self, *a, **k):
        return mock.Mock()

    def create_timer(self, *a, **k):
        return mock.Mock()

    def get_logger(self):
        return mock.Mock()


class ApiBase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.tmp.name, "s.json")
        self.client = FakeClient()
        self.nav = ns.NavSpeedController(client=self.client, save_path=self.path)
        self.node = FakeNode()
        self.teleop = dc.TeleopController(self.node)
        self.auto = dc.AutoSpeedController(self.node)
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, self.teleop,
            auto_speed=self.auto, nav_speed=self.nav,
            auth_username="TL", auth_password="pw", enforce_single_controller=False,
        )
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        self.port = self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.tl = self.login("TL", "pw")
        self.guest = self.login("guest", "")

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.tmp.cleanup()

    def login(self, user, password):
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.port}/api/login",
            data=json.dumps({"username": user, "password": password}).encode(),
            method="POST", headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.headers.get("Set-Cookie", "").split(";")[0]

    def call(self, method, path, body=None, raw=None, cookie=None):
        cookie = self.tl if cookie is None else cookie
        data = raw if raw is not None else (None if body is None else json.dumps(body).encode())
        headers = {"Content-Type": "application/json"}
        if cookie:
            headers["Cookie"] = cookie
        req = urllib.request.Request(f"http://127.0.0.1:{self.port}{path}", data=data, method=method, headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=30) as r:
                return r.status, json.loads(r.read().decode("utf-8"))
        except urllib.error.HTTPError as e:
            try:
                return e.code, json.loads(e.read().decode("utf-8"))
            finally:
                e.close()

    def post(self, body, **kw):
        return self.call("POST", "/api/speed_settings", body, **kw)

    def cmd_vel_publishes(self):
        pub = self.node.pubs["/cmd_vel"]
        return pub.publish.call_count


class ApiTests(ApiBase):
    def test_get_shows_nav_speed_and_director_speed(self):
        st, d = self.call("GET", "/api/speed_settings")
        self.assertEqual(st, 200)
        self.assertEqual(d["nav_max_speed_mps"], 0.7)
        self.assertEqual(d["nav_speed_max"], 1.1)
        self.assertIn("auto_linear_mps", d)  # directorの速度は別のつまみとして残る
        self.assertEqual(d["auto_linear_mps"], 0.75)

    def test_valid_change_applies_and_reads_back(self):
        st, d = self.post({"nav_max_speed_mps": 0.5})
        self.assertEqual(st, 200, d)
        self.assertTrue(d["nav"]["ok"])
        self.assertEqual(d["nav_max_speed_mps"], 0.5)
        self.assertEqual(d["nav"]["read_back"][ns.NODE_CONTROLLER]["FollowPath.max_vel_x"], 0.5)
        st, d = self.call("GET", "/api/nav_speed")
        self.assertEqual(st, 200)
        self.assertTrue(d["read_back"]["running"])
        self.assertEqual(d["read_back"]["nodes"][ns.NODE_SMOOTHER]["max_velocity"][0], 0.5)
        self.assertEqual(d["nav_max_speed_mps"], 0.5)

    def test_nan_is_400_and_applies_nothing(self):
        before = self.teleop.status()["manual_linear_mps"]
        st, d = self.post(None, raw=b'{"nav_max_speed_mps": NaN, "manual_linear_mps": 0.3}')
        self.assertEqual(st, 400)
        self.assertEqual(self.client.calls, [])
        self.assertEqual(self.teleop.status()["manual_linear_mps"], before)  # 他の項目も保存しない
        for raw in (b'{"nav_max_speed_mps": Infinity}', b'{"nav_max_speed_mps": "fast"}', b'{"nav_max_speed_mps": true}'):
            self.assertEqual(self.post(None, raw=raw)[0], 400, raw)
        self.assertEqual(self.client.calls, [])

    def test_guest_is_403(self):
        st, d = self.post({"nav_max_speed_mps": 0.5}, cookie=self.guest)
        self.assertEqual(st, 403)
        self.assertEqual(self.client.calls, [])
        self.assertEqual(self.post({"nav_max_speed_mps": 0.5}, cookie="")[0], 401)

    def test_clamp_message(self):
        st, d = self.post({"nav_max_speed_mps": 0.01})
        self.assertEqual(st, 200)
        self.assertEqual(d["nav_max_speed_mps"], 0.1)
        self.assertTrue(d["nav"]["clamped"])
        self.assertIn("丸めました", d["message"])

    def test_confirm_high_required_above_06(self):
        st, d = self.post({"nav_max_speed_mps": 0.9})
        self.assertEqual(st, 400)
        self.assertTrue(d["confirm_required"])
        self.assertEqual(self.client.calls, [])
        st, d = self.post({"nav_max_speed_mps": 0.9, "manual_linear_mps": 0.3})
        self.assertEqual(st, 400)
        self.assertNotAlmostEqual(self.teleop.status()["manual_linear_mps"], 0.3)  # 確認前は他も保存しない
        st, d = self.post({"nav_max_speed_mps": 0.9, "confirm_high": True})
        self.assertEqual(st, 200)
        self.assertEqual(self.client.store[ns.NODE_CONTROLLER]["FollowPath.max_vel_x"], 0.9)
        st, d = self.post({"nav_max_speed_mps": 0.6})
        self.assertEqual(st, 200)

    def test_nav2_not_running_is_409(self):
        self.client.ready = False
        st, d = self.post({"nav_max_speed_mps": 0.5})
        self.assertEqual(st, 409)
        self.assertIn("起動していません", d["message"])
        self.assertTrue(d["nav"]["saved_for_next_launch"])
        self.assertEqual(ns.load_saved_nav_speed(self.path), 0.5)
        st, d = self.call("GET", "/api/nav_speed")
        self.assertFalse(d["read_back"]["running"])
        self.assertEqual(d["nav_max_speed_mps"], 0.5)
        self.assertEqual(d["nav_max_speed_source"], "saved")

    def test_other_speed_fields_still_work_without_nav(self):
        st, d = self.post({"manual_linear_mps": 0.6})
        self.assertEqual(st, 200)
        self.assertAlmostEqual(d["manual_linear_mps"], 0.6)
        self.assertEqual(self.client.calls, [])

    def test_speed_change_never_publishes_cmd_vel_nor_touches_estop(self):
        self.teleop.handle_soft_estop(True)
        engaged_before = self.teleop.soft_estop_engaged
        base = self.cmd_vel_publishes()
        self.assertEqual(self.post({"nav_max_speed_mps": 0.5})[0], 200)  # パラメータ設定は動作ではない(従来どおり許可)
        self.assertEqual(self.post({"nav_max_speed_mps": 1.0, "confirm_high": True})[0], 200)
        self.assertEqual(self.post({"nav_max_speed_mps": 0.3})[0], 200)
        self.client.ready = False
        self.assertEqual(self.post({"nav_max_speed_mps": 0.5})[0], 409)
        self.call("GET", "/api/nav_speed")
        self.assertEqual(self.cmd_vel_publishes(), base)
        self.assertEqual(self.teleop.soft_estop_engaged, engaged_before)
        # 新モジュールはcmd_vel/ゴール/estopを一切扱わない
        src = (PKG / "nokosahen_core" / "nav_speed.py").read_text(encoding="utf-8")
        code_only = re.sub(r'""".*?"""', "", src, flags=re.S)
        for word in ("Twist", "PoseStamped", "NavigateToPose", "cancel_goal", "soft_estop", "create_publisher"):
            self.assertNotIn(word, code_only)

    def test_mobile_handler_rejects_nav_field(self):
        status, body = dc._speed_settings_post({"nav_max_speed_mps": 0.5}, self.teleop, self.auto, None)
        self.assertEqual(status, 404)


class UnifiedSpeedTests(ApiBase):
    def test_all_three_set_to_same_value(self):
        st, d = self.post({"all_linear_mps": 0.5})
        self.assertEqual(st, 200, d)
        u = d["unified"]
        self.assertEqual([t["status"] for t in u["targets"]], ["成功", "成功", "成功"])
        self.assertEqual(d["manual_linear_mps"], 0.5)
        self.assertEqual(d["auto_linear_mps"], 0.5)
        self.assertEqual(self.client.store[ns.NODE_CONTROLLER]["FollowPath.max_vel_x"], 0.5)
        self.assertFalse(u["clamped"])
        # 旋回は変わらない
        self.assertEqual(d["manual_angular_rps"], self.teleop.status()["manual_angular_rps"])
        self.assertEqual(d["auto_angular_rps"], 0.75)

    def test_confirm_required_above_06_and_nothing_applied(self):
        manual = self.teleop.status()["manual_linear_mps"]
        st, d = self.post({"all_linear_mps": 0.9})
        self.assertEqual(st, 400)
        self.assertTrue(d["confirm_required"])
        self.assertEqual(self.client.calls, [])
        self.assertEqual(self.teleop.status()["manual_linear_mps"], manual)
        self.assertEqual(self.auto.status()["auto_linear_mps"], 0.75)
        st, d = self.post({"all_linear_mps": 0.9, "confirm_high": True})
        self.assertEqual(st, 200)
        self.assertEqual(self.auto.status()["auto_linear_mps"], 0.9)

    def test_clamp_to_11(self):
        st, d = self.post({"all_linear_mps": 5, "confirm_high": True})
        self.assertEqual(st, 200)
        self.assertTrue(d["unified"]["clamped"])
        self.assertEqual(d["unified"]["applied"], 1.1)
        self.assertEqual(d["manual_linear_mps"], 1.1)
        self.assertEqual(d["auto_linear_mps"], 1.1)
        self.assertIn("丸めました", d["message"])
        st, d = self.post({"all_linear_mps": 0.0})
        self.assertEqual(d["unified"]["applied"], 0.1)

    def test_nonfinite_400_guest_403(self):
        manual = self.teleop.status()["manual_linear_mps"]
        for raw in (b'{"all_linear_mps": NaN}', b'{"all_linear_mps": Infinity}', b'{"all_linear_mps": "x"}', b'{"all_linear_mps": true}'):
            self.assertEqual(self.post(None, raw=raw)[0], 400, raw)
        self.assertEqual(self.post({"all_linear_mps": 0.5}, cookie=self.guest)[0], 403)
        self.assertEqual(self.teleop.status()["manual_linear_mps"], manual)
        self.assertEqual(self.client.calls, [])

    def test_nav2_not_running_still_applies_manual_and_director(self):
        self.client.ready = False
        st, d = self.post({"all_linear_mps": 0.5})
        self.assertEqual(st, 409)
        by = {t["key"]: t["status"] for t in d["unified"]["targets"]}
        self.assertEqual(by, {"manual_linear_mps": "成功", "auto_linear_mps": "成功", "nav_max_speed_mps": "Nav2未起動"})
        self.assertEqual(self.teleop.status()["manual_linear_mps"], 0.5)
        self.assertEqual(self.auto.status()["auto_linear_mps"], 0.5)
        self.assertEqual(ns.load_saved_nav_speed(self.path), 0.5)
        self.assertIn("Nav2未起動", d["message"])

    def test_nav2_partial_failure_reported(self):
        self.client.timeout_nodes.add(ns.NODE_COLLISION)
        st, d = self.post({"all_linear_mps": 0.4})
        self.assertEqual(st, 200)
        by = {t["key"]: t["status"] for t in d["unified"]["targets"]}
        self.assertEqual(by["nav_max_speed_mps"], "失敗")
        self.assertEqual(by["manual_linear_mps"], "成功")

    def test_no_cmd_vel_published(self):
        base = self.cmd_vel_publishes()
        self.post({"all_linear_mps": 1.0, "confirm_high": True})
        self.client.ready = False
        self.post({"all_linear_mps": 0.3})
        self.assertEqual(self.cmd_vel_publishes(), base)

    def test_without_nav_controller_still_sets_manual_and_director(self):
        st, d = dc._speed_settings_post({"all_linear_mps": 0.5}, self.teleop, self.auto, None)
        self.assertEqual(st, 404)
        by = {t["key"]: t["status"] for t in d["unified"]["targets"]}
        self.assertEqual((by["manual_linear_mps"], by["auto_linear_mps"]), ("成功", "成功"))
        self.assertEqual(by["nav_max_speed_mps"], "失敗")


class ManualLimitTests(ApiBase):
    """【2026-10-10】手動走行の上限引き上げ(直進1.2 / 旋回4.0)。既定値は不変、自律側の上限は不変。"""

    def test_constants_and_wheel_clamp(self):
        self.assertEqual((ns.MANUAL_LINEAR_MAX_MPS, ns.MANUAL_ANGULAR_MAX_RADPS), (1.2, 4.0))
        # 旧上限より大きく、ファームの車輪速クランプ(1.23)の範囲内(直進単独 / その場旋回単独)
        self.assertGreater(ns.MANUAL_LINEAR_MAX_MPS, 1.1)
        self.assertGreater(ns.MANUAL_ANGULAR_MAX_RADPS, 3.0)
        self.assertLessEqual(ns.peak_wheel_speed(ns.MANUAL_LINEAR_MAX_MPS, 0), ns.FIRMWARE_WHEEL_CLAMP_MPS)
        self.assertLessEqual(ns.peak_wheel_speed(0, ns.MANUAL_ANGULAR_MAX_RADPS), ns.FIRMWARE_WHEEL_CLAMP_MPS)
        # 自律側(Nav2 / director)の上限は不変
        self.assertEqual(ns.NAV_SPEED_MAX, 1.1)
        self.assertEqual((ns.AUTO_LINEAR_MAX_MPS, ns.AUTO_ANGULAR_MAX_RADPS), (1.1, 3.0))
        self.assertEqual(dc.MAX_DRIVE_LINEAR_MPS, 1.1)

    def test_defaults_unchanged(self):
        # コントローラの既定値(引数既定)・nokosahen.yamlの手動既定(直進1.0 / 旋回1.5)は変えていない
        self.assertEqual(self.teleop.status()["manual_linear_mps"], 0.15)
        self.assertEqual(self.teleop.status()["manual_angular_rps"], 0.6)
        yaml_text = (BRINGUP / "config" / "nokosahen.yaml").read_text(encoding="utf-8")
        self.assertEqual(len(re.findall(r"linear_speed_mps: 1\.0\b", yaml_text)), 3)
        self.assertEqual(len(re.findall(r"angular_speed_rps: 1\.5\b", yaml_text)), 3)
        # 起動値が上限を超えるときだけ丸める(旋回も)
        t = dc.TeleopController(FakeNode(), linear_speed=9.0, angular_speed=9.0)
        self.assertEqual((t.linear_speed, t.angular_speed), (1.2, 4.0))
        t = dc.TeleopController(FakeNode(), linear_speed=1.0, angular_speed=1.5)
        self.assertEqual((t.linear_speed, t.angular_speed), (1.0, 1.5))

    def test_get_limits_from_constants(self):
        st, d = self.call("GET", "/api/speed_settings")
        self.assertEqual(st, 200)
        lim = d["limits"]
        self.assertEqual(lim, ns.manual_speed_limits())
        self.assertEqual((lim["manual_linear_max"], lim["manual_angular_max"]), (ns.MANUAL_LINEAR_MAX_MPS, ns.MANUAL_ANGULAR_MAX_RADPS))
        self.assertEqual((lim["auto_linear_max"], lim["auto_angular_max"]), (1.1, 3.0))
        self.assertEqual(d["max_linear_mps"], 1.1)  # 従来キーは自律側の値のまま
        self.assertEqual(d["nav_speed_max"], 1.1)

    def test_manual_high_needs_confirm_and_nothing_applied(self):
        before = self.teleop.status()
        for body in ({"manual_linear_mps": 1.2}, {"manual_angular_rps": 4.0}, {"manual_linear_mps": 1.15, "auto_linear_mps": 0.5},
                     {"manual_linear_mps": 99}, {"manual_angular_rps": "3.5"}):
            st, d = self.post(body)
            self.assertEqual(st, 400, body)
            self.assertTrue(d["confirm_required"], body)
            self.assertTrue(d["manual_high"], body)
        self.assertEqual(self.teleop.status(), before)
        self.assertEqual(self.auto.status()["auto_linear_mps"], 0.75)  # 他の項目も保存しない
        st, d = self.post({"manual_linear_mps": 1.2, "manual_angular_rps": 4.0, "confirm_high": True})
        self.assertEqual(st, 200, d)
        self.assertEqual((d["manual_linear_mps"], d["manual_angular_rps"]), (1.2, 4.0))
        self.assertEqual(d["clamped"], [])

    def test_manual_clamped_to_new_max(self):
        st, d = self.post({"manual_linear_mps": 9, "manual_angular_rps": 99, "confirm_high": True})
        self.assertEqual(st, 200)
        self.assertEqual((d["manual_linear_mps"], d["manual_angular_rps"]), (1.2, 4.0))
        self.assertEqual({c["key"] for c in d["clamped"]}, {"manual_linear_mps", "manual_angular_rps"})
        st, d = self.post({"manual_linear_mps": -5, "manual_angular_rps": 0})
        self.assertEqual((d["manual_linear_mps"], d["manual_angular_rps"]), (0.01, 0.01))

    def test_old_range_needs_no_confirm(self):
        st, d = self.post({"manual_linear_mps": 1.1, "manual_angular_rps": 3.0})
        self.assertEqual(st, 200, d)
        st, d = self.post({"manual_linear_mps": 1.0, "manual_angular_rps": 1.5, "auto_linear_mps": 0.9, "auto_angular_rps": 2.5})
        self.assertEqual(st, 200, d)

    def test_unchanged_high_value_not_reconfirmed(self):
        self.post({"manual_linear_mps": 1.2, "manual_angular_rps": 4.0, "confirm_high": True})
        st, d = self.post({"manual_linear_mps": 1.2, "manual_angular_rps": 4.0, "auto_linear_mps": 0.6})
        self.assertEqual(st, 200, d)
        self.assertEqual(self.auto.status()["auto_linear_mps"], 0.6)

    def test_auto_limits_unchanged(self):
        st, d = self.post({"auto_linear_mps": 5, "auto_angular_rps": 9})
        self.assertEqual(st, 200)
        self.assertEqual((d["auto_linear_mps"], d["auto_angular_rps"]), (1.1, 3.0))

    def test_nonfinite_still_rejected(self):
        before = self.teleop.status()
        for raw in (b'{"manual_linear_mps": NaN}', b'{"manual_angular_rps": Infinity}', b'{"manual_linear_mps": "x"}'):
            self.assertEqual(self.post(None, raw=raw)[0], 400, raw)
        self.assertEqual(self.teleop.status(), before)

    def test_all_same_speed_rounds_to_nav_cap_not_manual_cap(self):
        st, d = self.post({"all_linear_mps": 1.2, "confirm_high": True})
        self.assertEqual(st, 200, d)
        self.assertEqual(d["unified"]["applied"], 1.1)
        self.assertEqual((d["manual_linear_mps"], d["auto_linear_mps"]), (1.1, 1.1))
        self.assertEqual(self.client.store[ns.NODE_CONTROLLER]["FollowPath.max_vel_x"], 1.1)
        # nav_max_speed_mps単独も1.1止まり(手動の上限は効かない)
        st, d = self.post({"nav_max_speed_mps": 1.2, "confirm_high": True})
        self.assertEqual(d["nav_max_speed_mps"], 1.1)

    def test_drive_twist_never_exceeds_manual_max(self):
        class FakeTwist:  # 共有スタブのTwistはlinear/angularを持たないため、ここだけ差し替える
            def __init__(self):
                self.linear = types.SimpleNamespace(x=0.0)
                self.angular = types.SimpleNamespace(z=0.0)

        t = self.teleop
        t.linear_speed, t.angular_speed = 50.0, 50.0  # 万一の内部不整合でもハード上限で丸まる
        with mock.patch.object(dc, "Twist", FakeTwist):
            t.handle_drive(1.0, -1.0)
            tw = self.node.pubs["/cmd_vel"].publish.call_args[0][0]
            self.assertEqual((tw.linear.x, tw.angular.z), (1.2, -4.0))
            t.handle_drive(5.0, 5.0)  # 入力も±1に丸める
            tw = self.node.pubs["/cmd_vel"].publish.call_args[0][0]
            self.assertEqual((tw.linear.x, tw.angular.z), (1.2, 4.0))

    def test_ui_max_attributes_match_constants(self):
        import test_dashboard_layout as tdl

        pages = tdl.pages()
        for name in ("HTML", "MOBILE_HTML"):
            html = pages[name]
            for el, want in (("manualLinear", ns.MANUAL_LINEAR_MAX_MPS), ("manualAngular", ns.MANUAL_ANGULAR_MAX_RADPS),
                             ("autoLinear", ns.AUTO_LINEAR_MAX_MPS), ("autoAngular", ns.AUTO_ANGULAR_MAX_RADPS)):
                m = re.search(r'id="%s"[^>]*\bmax="([0-9.]+)"' % el, html)
                self.assertIsNotNone(m, (name, el))
                self.assertEqual(float(m.group(1)), want, (name, el))
                self.assertIn('id="speedConfirmBar"', html)
            for needle in ("speedConfirmYes", "speedConfirmNo", "confirm_high", "電源を切"):
                self.assertIn(needle, html, (name, needle))
            # 注意書きの数値も定数と同じ(1.2 / 4 / 1.1 / 3)
            self.assertIn("直進1.2m/s・旋回4rad/s", html)
        # 全て同じ速度にする欄のmaxはNav2の上限のまま
        self.assertRegex(pages["HTML"], r'id="allSpeed"[^>]*max="1\.1"')
        self.assertRegex(pages["HTML"], r'id="navSpeed"[^>]*max="1\.1"')


class DashboardSourceTests(unittest.TestCase):
    def test_estop_and_cancel_paths_untouched_and_line_endings(self):
        # 編集したCRLFファイルがCRLFのまま(混在していない)
        for name in ("dashboard_common.py", "full_dashboard_node.py"):
            b = (PKG / "nokosahen_core" / name).read_bytes()
            self.assertEqual(b.count(b"\r\n"), b.count(b"\n"), name)
        for name in ("nav_speed.py",):
            self.assertNotIn(b"\r", (PKG / "nokosahen_core" / name).read_bytes())

    def test_ui_elements_and_scripts(self):
        import test_dashboard_layout as tdl

        html = tdl.pages()["HTML"]
        for needle in ('id="navSpeed"', 'id="navSpeedBtn"', 'id="navConfirmBar"', 'id="navConfirmYes"', 'id="navConfirmNo"',
                       "自動走行(Nav2)の最高速度 [m/s]", 'min="0.1" max="1.1"', 'step="0.05"',
                       "物理の緊急停止は未配線です。広い場所で、電源を切る手段を手元に置いて確認してください",
                       "上げる", "やめる", "confirm_high", "/api/nav_speed", "director自動"):
            self.assertIn(needle, html, needle)
        markup, js, scripts = tdl.split(html)
        for i in ("navSpeed", "navSpeedBtn", "navConfirmBar", "navConfirmText", "navConfirmYes", "navConfirmNo", "navSpeedHint", "navSpeedInfo"):
            self.assertEqual(markup.count(f'id="{i}"'), 1, i)
        block = js[js.index("Nav2(自律走行)の最高速度"):js.index("// --- BGM(voice_bridge")]
        self.assertNotIn("confirm(", block.replace("navShowConfirm(", "").replace("navHideConfirm(", ""))  # ブラウザダイアログ無し
        self.assertNotIn("alert(", block)
        for needle in ('id="allSpeed"', 'id="allSpeedBtn"', "全て同じ速度にする [m/s]", "all_linear_mps", 'id="allSpeedResult"'):
            self.assertIn(needle, html, needle)
        self.assertNotIn("prompt(", block)
        node = shutil.which("node")
        if node:
            for n, script in enumerate(scripts):
                with tempfile.TemporaryDirectory() as tmp:
                    f = Path(tmp) / f"s{n}.js"
                    f.write_text(script, encoding="utf-8")
                    r = subprocess.run([node, "--check", str(f)], capture_output=True, text=True)
                    self.assertEqual(r.returncode, 0, r.stderr)


if __name__ == "__main__":
    unittest.main()
