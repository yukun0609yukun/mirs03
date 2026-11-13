"""自律走行の高速化・旋回の高速化・人/障害物から機体端まで50cm・立ちふさがり時の後退(2026-10-10)のテスト。
ROS 2もロボットも不要。設定値の整合(yaml <-> nav_speed.derive_clearance_values <-> BT XML <-> ファームの限界)を守る。

- 機体の外形(footprint)と四隅の柱の実測、インフレーションの半径/コスト
- collision_monitorの停止領域VelocityStop: 機体端から50cm、速度帯ごとの停止距離、後退用の後方、速度帯に抜けがない
- 旋回(1.8 rad/s)がFollowPath/velocity_smoother/behavior_serverで一貫、車輪速クランプとの関係
- BT XML(nokosahen_nav_to_pose_bt.xml): 標準ノードだけ、ポート名、順序(条件 -> Wait -> BackUp -> コストマップ消去 / TF_ERROR等はWait -> 消去だけ)、legacy版、環境変数切替
- 「動けない」検出(6秒) + Wait(2秒)の合計、起動時の書き換え(footprint/インフレ/停止領域/BTの絶対パス)

実行: python ros2_ws/src/nokosahen_core/test/test_nav2_clearance.py
"""
import copy
import json
import math
import os
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]
BRINGUP = PKG.parent / "nokosahen_bringup"
sys.path.insert(0, str(PKG))

from nokosahen_core import nav_speed as ns  # noqa: E402
from nokosahen_core import scan_self_filter as ssf  # noqa: E402

YAML_PATH = BRINGUP / "config" / "nav2_params_jetson.yaml"
BT_PATH = BRINGUP / "config" / "nokosahen_nav_to_pose_bt.xml"
BT_LEGACY_PATH = BRINGUP / "config" / "nokosahen_nav_to_pose_bt.legacy.xml"

# Nav2 Jazzy標準のBTノード(bt_navigatorが組み込みで読み込む)と、このBTで使うポート。
# 保守的に標準ノードだけを使う。Nav2 Jazzy標準navigate_to_pose_w_replanning_and_recovery.xmlと
# リポジトリ内のbehavior_trees/cafeteria_promoter_nav2.xmlに出てくる名前・ポートが根拠。
BT_ALLOWED = {
    "RecoveryNode": {"number_of_retries", "name"},
    "PipelineSequence": {"name"},
    "RateController": {"hz"},
    "ComputePathToPose": {"goal", "path", "planner_id", "start", "use_start", "error_code_id"},
    "FollowPath": {"path", "controller_id", "goal_checker_id", "progress_checker_id", "error_code_id"},
    "ReactiveFallback": {"name"},
    "Sequence": {"name"},
    "Fallback": {"name"},
    "WouldAControllerRecoveryHelp": {"error_code"},
    "ForceSuccess": set(),
    "GoalUpdated": set(),
    "Wait": {"wait_duration", "server_name", "server_timeout", "error_code_id"},
    "RoundRobin": {"name"},
    "Spin": {"spin_dist", "time_allowance", "server_name", "server_timeout", "error_code_id"},
    "BackUp": {"backup_dist", "backup_speed", "time_allowance", "server_name", "server_timeout", "error_code_id"},
    "ClearEntireCostmap": {"name", "service_name", "server_timeout"},
}
BT_STRUCTURE = {"BehaviorTree", "root"}


def load_yaml():
    return yaml.safe_load(YAML_PATH.read_text(encoding="utf-8"))


def pts(s):
    return json.loads(s)


def depth_x(points):
    xs = [p[0] for p in points]
    return min(xs), max(xs)


class TestFootprintAndInflation(unittest.TestCase):
    def setUp(self):
        self.d = load_yaml()
        self.want = ns.derive_clearance_values()

    def test_footprint_covers_measured_corners(self):
        # 四隅の柱の実測: LiDARから 0.33〜0.37 m。外接半径がそれ以上(安全側)
        circ = ns.circumscribed_radius()
        self.assertGreaterEqual(circ, 0.365)
        self.assertLessEqual(circ, 0.40)
        # scan_self_filterの4窓の向きは機体の対角(約±45度、±135度)付近 = 正方形の四隅
        for c in ssf.DEFAULT_CENTERS_DEG:
            self.assertLess(abs(abs(c) % 90.0 - 45.0), 8.0, c)  # 取り付けの回転(yaw -1.75度)と柱の幅のぶんずれる

    def test_yaml_footprint_matches_derivation_in_both_costmaps(self):
        for n in ("local_costmap", "global_costmap"):
            rp = self.d[n][n]["ros__parameters"]
            self.assertNotIn("robot_radius", rp, n)  # 円(0.28)と併記しない
            # 【2026-10-22】経路計画(global)は小さめ(PLAN_BODY_HALF_M)、追従(local)は実寸(BODY_HALF_M)
            want = self.want["plan_footprint"] if n == "global_costmap" else self.want["footprint"]
            half = ns.PLAN_BODY_HALF_M if n == "global_costmap" else ns.BODY_HALF_M
            self.assertEqual(pts(rp["footprint"]), pts(want), n)
            xs = sorted({abs(p[0]) for p in pts(rp["footprint"])} | {abs(p[1]) for p in pts(rp["footprint"])})
            self.assertEqual(xs, [half])  # 正方形(LiDAR中心)
        self.assertLess(ns.PLAN_BODY_HALF_M, ns.BODY_HALF_M)
        self.assertGreaterEqual(ns.PLAN_BODY_HALF_M, 0.18)  # 小さくしすぎない(狭い所を通る経路が出すぎる)

    def test_inflation_equal_in_both_costmaps_and_matches_derivation(self):
        for n in ("local_costmap", "global_costmap"):
            infl = self.d[n][n]["ros__parameters"]["inflation_layer"]
            self.assertEqual(infl["inflation_radius"], self.want["inflation_radius"], n)
            self.assertEqual(infl["cost_scaling_factor"], self.want["cost_scaling_factor"], n)

    def test_inflation_reaches_clearance_from_the_corner(self):
        r = self.want["inflation_radius"]
        self.assertGreaterEqual(r, ns.circumscribed_radius() + ns.CLEARANCE_M - 1e-9)
        self.assertGreaterEqual(r, 0.55)

    def test_cost_curve(self):
        k = self.want["cost_scaling_factor"]
        cost = lambda edge: 252.0 * math.exp(-k * edge)  # 端からの距離edge[m]のコスト(side-on)
        # 【2026-10-22】端から20cmあれば低コスト。それより内側(寄りすぎ)は急に高コスト=舵を強く切って離れる
        self.assertLess(cost(0.20), 25.0)
        self.assertGreater(cost(0.10), 60.0)
        self.assertGreater(cost(0.05), 120.0)
        self.assertGreater(cost(0.10) / cost(0.20), 3.0)  # 20cmの所と比べて3倍以上嫌われる
        self.assertLess(cost(0.50), 1.0)  # 遠くはほぼ無関係
        # インフレーション半径の端(外接半径+20cm)では十分小さい
        self.assertLess(252.0 * math.exp(-k * (r_edge := self.want["inflation_radius"] - ns.BODY_HALF_M)), 12.0, r_edge)

    def test_local_costmap_window_covers_inflation(self):
        lc = self.d["local_costmap"]["local_costmap"]["ros__parameters"]
        self.assertGreaterEqual(min(lc["width"], lc["height"]) / 2.0, self.want["inflation_radius"] + 0.5)
        # 脚(細いLiDARクラスタ)は障害物として入る
        obs = lc["obstacle_layer"]["scan"]
        self.assertTrue(obs["marking"] and obs["clearing"])
        self.assertEqual(obs["data_type"], "LaserScan")
        self.assertLessEqual(obs["obstacle_min_range"], 0.1)
        self.assertGreaterEqual(obs["obstacle_max_range"], 2.0)
        self.assertIn("obstacle_layer", lc["plugins"])
        self.assertIn("obstacle_layer", self.d["global_costmap"]["global_costmap"]["ros__parameters"]["plugins"])

    def test_clearance_is_a_single_knob(self):
        # インフレーションは clearance_m 1つで決まる。前方の停止領域は停止距離のため、clearance_mとは独立(50cm固定)
        a = ns.derive_clearance_values(0.2)
        b = ns.derive_clearance_values(0.5)
        self.assertGreater(b["inflation_radius"], a["inflation_radius"])
        self.assertLess(b["cost_scaling_factor"], a["cost_scaling_factor"])
        fa = pts(a["stop_polygon"]["forward"]["points"])
        fb = pts(b["stop_polygon"]["forward"]["points"])
        self.assertAlmostEqual(depth_x(fb)[1], depth_x(fa)[1], places=3)
        self.assertAlmostEqual(depth_x(fa)[1], ns.BODY_HALF_M + ns.FRONT_STOP_CLEARANCE_M, places=3)
        for bad in (0.0, 5.0, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                ns.derive_clearance_values(bad)

    def test_apply_clearance_is_identity_on_the_yaml_and_non_destructive(self):
        d = load_yaml()
        snap = copy.deepcopy(d)
        self.assertEqual(ns.apply_clearance_to_params(d), d)  # yamlは導出値と同値(静的な値がずれていない)
        self.assertEqual(d, snap)
        # 古い値(robot_radius・小さいインフレ・停止領域なし)から導出値になる
        old = copy.deepcopy(d)
        for n in ("local_costmap", "global_costmap"):
            rp = old[n][n]["ros__parameters"]
            rp.pop("footprint")
            rp["robot_radius"] = 0.28
            rp["inflation_layer"]["inflation_radius"] = 0.45
        old["collision_monitor"]["ros__parameters"]["polygons"] = ["FootprintApproach"]
        old["collision_monitor"]["ros__parameters"].pop("VelocityStop")
        old["collision_monitor"]["ros__parameters"].pop("RearStop")
        self.assertEqual(ns.apply_clearance_to_params(old), d)
        # 存在しないノードは作らない
        self.assertEqual(ns.apply_clearance_to_params({"amcl": {}}), {"amcl": {}})


class TestStopZone(unittest.TestCase):
    def setUp(self):
        self.d = load_yaml()
        self.cm = self.d["collision_monitor"]["ros__parameters"]
        self.vs = self.cm["VelocityStop"]
        self.rs = self.cm["RearStop"]  # 【2026-10-24】後退用の停止領域は、person_clear_gateが外さない別のポリゴン
        self.h = ns.BODY_HALF_M

    def test_yaml_matches_derivation(self):
        self.assertEqual(self.vs, ns.derive_clearance_values()["stop_polygon"])
        self.assertEqual(self.rs, ns.derive_clearance_values()["rear_polygon"])
        self.assertEqual(self.cm["polygons"], ["FootprintApproach", "VelocityStop", "RearStop"])

    def test_rear_stop_is_separate_and_always_enabled(self):
        # H1: VelocityStop(ゲートが無効にする対象)に後退の領域を入れない。後退はRearStopだけが見る(常に有効)
        self.assertNotIn("backward", self.vs)
        self.assertNotIn("backward", self.vs["velocity_polygons"])
        self.assertEqual(sorted(self.vs["velocity_polygons"]),
                         ["backward_pass", "forward", "forward_fast", "forward_mid", "stopped_rotating"])
        self.assertEqual(self.rs["type"], "velocity_polygon")
        self.assertEqual(self.rs["action_type"], "stop")
        self.assertIs(self.rs["enabled"], True)
        self.assertEqual(self.rs["velocity_polygons"], ["backward", "not_backward"])
        self.assertEqual(sorted(k for k, v in self.rs.items() if isinstance(v, dict)), ["backward", "not_backward"])
        self.assertLessEqual(self.rs["min_points"], 3)
        # ポリゴン名はこの3つだけ(未知の名前が増えたら、ゲートの対象かどうかを確認させる)
        self.assertEqual(set(self.cm["polygons"]), {"FootprintApproach", "VelocityStop", "RearStop"})
        # 前進・旋回の速度では、RearStopのどの領域にも合わない = 何も止めない
        sub = self.rs["backward"]
        for v in (0.0, 0.02, 0.5, 1.1):
            self.assertFalse(sub["linear_min"] <= v <= sub["linear_max"], v)

    def test_pass_polygons_are_tiny_and_stop_nothing(self):
        # 2026-10-26: 速度が合う領域が無いと「Velocity is not covered」の警告が毎回出るので、相手の担当の速度を受け持つ極小の領域を持つ。
        # 極小(1 cm角)で、機体の中心にだけあり、LiDARの点(機体の外)が入らない = 何も止めない
        for sub in (self.vs["backward_pass"], self.rs["not_backward"]):
            p = pts(sub["points"])
            self.assertLessEqual(max(abs(x) for x, _ in p), 0.01)
            self.assertLessEqual(max(abs(y) for _, y in p), 0.01)
        # 互いの担当の速度を埋める: 後退はVelocityStopのbackward_pass、それ以外はRearStopのnot_backward
        for v in (-0.7, -0.03):
            self.assertTrue(self.vs["backward_pass"]["linear_min"] <= v <= self.vs["backward_pass"]["linear_max"], v)
        for v in (-0.02, 0.0, 0.7, 1.1):
            self.assertTrue(self.rs["not_backward"]["linear_min"] <= v <= self.rs["not_backward"]["linear_max"], v)

    def test_existing_monitor_settings_kept(self):
        self.assertEqual(self.cm["transform_tolerance"], 0.2)  # 安全層は古い姿勢で判定しない(据え置き)
        self.assertEqual(self.cm["source_timeout"], 1.0)
        self.assertEqual(self.cm["cmd_vel_in_topic"], "cmd_vel_smoothed")
        self.assertEqual(self.cm["cmd_vel_out_topic"], "cmd_vel")
        self.assertEqual(self.cm["observation_sources"], ["scan"])
        self.assertEqual(self.cm["scan"]["topic"], "scan")
        self.assertEqual(self.cm["FootprintApproach"]["type"], "polygon")

    def test_velocity_polygon_is_a_stop_polygon_that_sees_legs(self):
        self.assertEqual(self.vs["type"], "velocity_polygon")
        self.assertEqual(self.vs["action_type"], "stop")
        self.assertIs(self.vs["holonomic"], False)
        self.assertIs(self.vs["enabled"], True)
        # 脚は細くLiDARで数点: 1〜2点の外れ値で止まらない最小値(3)以下
        self.assertGreaterEqual(self.vs["min_points"], 2)
        self.assertLessEqual(self.vs["min_points"], 3)
        self.assertEqual(sorted(self.vs["velocity_polygons"]),
                         sorted(k for k, v in self.vs.items() if isinstance(v, dict)))

    def test_every_velocity_is_covered(self):
        subs = [self.vs[n] for n in self.vs["velocity_polygons"]] + [self.rs[n] for n in self.rs["velocity_polygons"]]
        # 直進速度の範囲(-1.1〜1.1)と旋回(±1.8、余裕を見て±4)に抜けがない(0.001刻みで確認)
        for i in range(-1100, 1101):
            v = i / 1000.0
            for w in (-3.0, -1.8, -0.5, 0.0, 0.5, 1.8, 3.0):
                hit = [s for s in subs
                       if s["linear_min"] <= v <= s["linear_max"] and s["theta_min"] <= w <= s["theta_max"]]
                self.assertTrue(hit, (v, w))
        self.assertGreaterEqual(ns.NAV_SPEED_MAX, 1.1)
        self.assertLessEqual(ns.NAV_SPEED_MAX, max(s["linear_max"] for s in subs))
        self.assertGreaterEqual(4.0, ns.BASE_THETA_VEL)

    def test_forward_zone_reaches_clearance_beyond_the_front_edge(self):
        front_edge = self.h
        for name in ("forward", "forward_mid", "forward_fast"):
            lo, hi = depth_x(pts(self.vs[name]["points"]))
            self.assertGreaterEqual(hi - front_edge, ns.CLEARANCE_M - 1e-9, name)  # 機体の端から50cm以上
            self.assertGreaterEqual(lo, front_edge)  # 機体・柱の写り込みを数えない
            ys = [p[1] for p in pts(self.vs[name]["points"])]
            self.assertGreaterEqual(max(ys), self.h + ns.SIDE_MARGIN_M - 1e-9)  # 機体の幅より外側まで

    def test_stopping_distance_fits_in_each_zone(self):
        # 衝突監視が0を出してから止まるまで(遅れ + ファームの減速ランプ + 尾)が、領域の奥行きに収まる = 脚に届かない
        for name in ("forward", "forward_mid", "forward_fast"):
            sub = self.vs[name]
            _lo, hi = depth_x(pts(sub["points"]))
            depth = hi - self.h
            vmax = min(sub["linear_max"], ns.NAV_SPEED_MAX)
            self.assertLessEqual(ns.stop_distance_estimate(vmax), depth - 0.05, (name, vmax))
        # 既定速度(0.7)も収まる
        d = ns.derive_nav_speed_values(ns.NAV_SPEED_DEFAULT)
        band = next(n for n in ("forward", "forward_mid", "forward_fast")
                    if self.vs[n]["linear_min"] <= d["speed"] <= self.vs[n]["linear_max"])
        _lo, hi = depth_x(pts(self.vs[band]["points"]))
        self.assertLessEqual(ns.stop_distance_estimate(d["speed"]), hi - self.h)
        self.assertLessEqual(d["decel"], ns.MAX_DECEL)

    def test_backward_zone_is_behind_only_and_covers_backup(self):
        sub = self.rs["backward"]
        p = pts(sub["points"])
        self.assertTrue(all(x < 0 for x, _ in p))
        lo, hi = depth_x(p)
        self.assertGreaterEqual(-lo - self.h, ns.REAR_MARGIN_M - 1e-9)
        # 後退(BackUp)は通常の走行と同じ速さ(2026-10-23)。後方の停止領域の深さは、その停止距離+余裕を覆う
        depth = max(abs(x) for x, _ in p) - ns.BODY_HALF_M  # 機体の端からの深さ
        self.assertGreaterEqual(depth, ns.stop_distance_estimate(ns.BACKUP_SPEED_MPS) + ns.BACKUP_STOP_EXTRA_M - 2e-3)  # 座標は小数3桁に丸める
        self.assertLessEqual(sub["linear_min"], -ns.BACKUP_SPEED_MPS)  # 後退速度(BackUp)を含む
        self.assertLessEqual(sub["linear_max"], -0.02)
        # 前方の人が後退を妨げない: 後退の領域は前方(x>0)の点を含まない
        self.assertFalse(any(x > 0 for x, _ in p))

    def test_rotation_zone_surrounds_the_body(self):
        sub = self.vs["stopped_rotating"]
        p = pts(sub["points"])
        half = max(abs(x) for x, _ in p)
        self.assertGreaterEqual(half, ns.circumscribed_radius() + 0.01)  # 旋回で掃く円(外接半径0.368)より外側(2026-10-22: +5cm -> +1cm)
        self.assertLessEqual(half, self.h + ns.CLEARANCE_M)  # 壁際の旋回を50cmで禁止しない
        self.assertLessEqual(sub["linear_min"], 0.0 - 0.0)
        self.assertGreaterEqual(sub["linear_max"], 0.0)
        self.assertGreaterEqual(sub["theta_max"], ns.BASE_THETA_VEL)

    def test_side_margin_smaller_than_front_to_avoid_stopping_at_walls(self):
        self.assertLess(ns.SIDE_MARGIN_M, ns.CLEARANCE_M)
        # 幅1.1 mの通路の中央を通っても側壁(中心から0.55 m)は停止領域(±0.46)に入らない
        _lo, _hi = depth_x(pts(self.vs["forward"]["points"]))
        ys = max(abs(p[1]) for p in pts(self.vs["forward"]["points"]))
        self.assertLess(ys, 0.55)


class TestRotationConsistency(unittest.TestCase):
    def setUp(self):
        self.d = load_yaml()
        self.f = self.d["controller_server"]["ros__parameters"]["FollowPath"]
        self.sm = self.d["velocity_smoother"]["ros__parameters"]
        self.bs = self.d["behavior_server"]["ros__parameters"]

    def test_angular_limits_same_everywhere(self):
        w = ns.BASE_THETA_VEL
        self.assertGreaterEqual(w, 1.6)
        self.assertLessEqual(w, 2.0)
        self.assertEqual(self.f["max_vel_theta"], w)
        self.assertEqual(self.sm["max_velocity"][2], w)
        self.assertEqual(self.sm["min_velocity"][2], -w)
        self.assertEqual(self.bs["max_rotational_vel"], w)  # spin(先回し旋回/BT)
        self.assertEqual(self.f["acc_lim_theta"], ns.BASE_THETA_ACC)
        self.assertEqual(self.sm["max_accel"][2], ns.BASE_THETA_ACC)
        self.assertEqual(self.bs["rotational_acc_lim"], ns.BASE_THETA_ACC)
        self.assertEqual(self.f["decel_lim_theta"], -ns.BASE_THETA_DEC)
        self.assertEqual(self.sm["max_decel"][2], -ns.BASE_THETA_DEC)
        self.assertGreaterEqual(ns.BASE_THETA_ACC, 2.0)
        self.assertGreater(ns.BASE_THETA_VEL, 0.85 * 1.9)  # 従来(0.85)の約2倍

    def test_spin_profile_and_dead_band(self):
        self.assertLess(self.bs["min_rotational_vel"], 0.5)
        self.assertGreaterEqual(self.bs["min_rotational_vel"], 0.2)  # 従来値以上(終端で粘らない)
        self.assertLess(self.bs["min_rotational_vel"], self.bs["max_rotational_vel"])
        self.assertEqual(self.f["min_speed_xy"], 0.0)  # min_speed_thetaの停止禁止を有効にしない

    def test_180_degree_turn_is_fast(self):
        # 台形の見積り: 加速 w/a、180度 = pi。従来(0.85, 1.2)は約4.9秒
        def seconds(w, a):
            t_ramp = w / a
            ramp_angle = w * t_ramp  # 加速+減速で w^2/a
            return (math.pi - ramp_angle) / w + 2 * t_ramp if ramp_angle < math.pi else 2 * math.sqrt(math.pi / a)
        self.assertLess(seconds(ns.BASE_THETA_VEL, ns.BASE_THETA_ACC), 3.0)
        self.assertGreater(seconds(0.85, 1.2), 4.0)
        # ファームの角加速ランプ(1.6)が実効上限でも5秒以内
        self.assertLess(seconds(ns.BASE_THETA_VEL, 1.6), 4.0)

    def test_wheel_clamp_at_default_speed(self):
        # 外側の車輪速 = |v| + |w|*輪距/2 がファームのクランプ(約1.23 m/s)に収まる(既定0.7 m/s + 最大旋回)
        self.assertLessEqual(ns.peak_wheel_speed(ns.NAV_SPEED_DEFAULT, ns.BASE_THETA_VEL), ns.FIRMWARE_WHEEL_CLAMP_MPS)
        self.assertLessEqual(ns.peak_wheel_speed(0.0, ns.BASE_THETA_VEL), 0.4)  # その場旋回は車輪0.35 m/s
        # 上限1.1 m/sでの同時最大旋回はクランプを超える(文書化: 弧が緩くなるだけで走行は安全側)
        self.assertGreater(ns.peak_wheel_speed(ns.NAV_SPEED_MAX, ns.BASE_THETA_VEL), ns.FIRMWARE_WHEEL_CLAMP_MPS)

    def test_rotation_independent_of_linear_speed(self):
        for v in (0.2, 0.7, 1.1):
            new = ns.apply_nav_speed_to_params(self.d, v)
            sm = new["velocity_smoother"]["ros__parameters"]
            self.assertEqual(sm["max_velocity"][2], ns.BASE_THETA_VEL)
            self.assertEqual(sm["max_accel"][2], ns.BASE_THETA_ACC)
            self.assertEqual(sm["max_decel"][2], -ns.BASE_THETA_DEC)
            self.assertEqual(sm["min_velocity"], self.sm["min_velocity"])

    def test_default_linear_speed_derivation(self):
        d = ns.derive_nav_speed_values(0.7)
        self.assertEqual((d["accel"], d["time_before_collision"], d["costmap_size"], d["sim_time"]), (0.8, 1.5, 3, 1.4))
        self.assertAlmostEqual(d["decel"], 1.43, places=2)
        self.assertLessEqual(d["speed"], ns.NAV_SPEED_MAX)
        self.assertEqual(ns.NAV_SPEED_DEFAULT, 0.7)
        # 先読み(DWB sim_time)は停止距離より長い
        self.assertGreater(d["speed"] * d["sim_time"], d["stop_distance"] + ns.LOOKAHEAD_MARGIN_M - 0.005)
        # ローカルコストマップは先読み + インフレーション + 機体の外接半径をカバー
        self.assertGreaterEqual(d["costmap_size"] / 2.0, d["speed"] * d["sim_time"] + ns.circumscribed_radius() - 0.05)

    def test_reverse_speed_allows_backup(self):
        self.assertEqual(ns.REVERSE_SPEED, self.sm["min_velocity"][0])
        for v in (0.1, 0.7, 1.1):
            live = ns.live_parameters(v)
            self.assertNotIn("min_velocity", live[ns.NODE_SMOOTHER])


class TestBehaviorTree(unittest.TestCase):
    def setUp(self):
        self.d = load_yaml()
        self.root = ET.parse(BT_PATH).getroot()
        self.tree = {t.get("ID"): t for t in self.root.findall("BehaviorTree")}

    def nodes(self):
        return [e for t in self.tree.values() for e in t.iter() if e is not t]

    def test_yaml_points_at_the_file(self):
        bn = self.d["bt_navigator"]["ros__parameters"]
        self.assertEqual(bn["default_nav_to_pose_bt_xml"], BT_PATH.name)  # ファイル名だけ(起動時に絶対パスへ)
        self.assertTrue(BT_PATH.is_file())
        self.assertEqual(bn["transform_tolerance"], 0.6)

    def test_root_and_main_tree(self):
        self.assertEqual(self.root.tag, "root")
        self.assertEqual(self.root.get("BTCPP_format"), "4")
        self.assertIn(self.root.get("main_tree_to_execute"), self.tree)
        self.assertEqual(len(self.tree), 1)
        self.assertEqual(len(self.tree["MainTree"]), 1)  # 本体は1ノード

    def test_error_code_ports_wired(self):
        by = {}
        for e in self.nodes():
            by.setdefault(e.tag, []).append(e)
        self.assertEqual([e.get("error_code_id") for e in by["FollowPath"]], ["{follow_path_error_code}"])
        self.assertEqual([e.get("error_code_id") for e in by["ComputePathToPose"]], ["{compute_path_error_code}"])
        # 追従の失敗理由で後退するか決める。計画側のWouldAPlannerRecoveryHelpは使わない
        self.assertEqual([e.get("error_code") for e in by["WouldAControllerRecoveryHelp"]], ["{follow_path_error_code}"])
        self.assertNotIn("WouldAPlannerRecoveryHelp", by)

    def test_xml_wellformed_and_text_is_balanced(self):
        for path in (BT_PATH, BT_LEGACY_PATH):
            text = path.read_text(encoding="utf-8")
            root = ET.fromstring(text)  # 全ノードが閉じていないと例外
            self.assertEqual(root.get("BTCPP_format"), "4")
            self.assertTrue(text.lstrip().startswith("<?xml"))
            self.assertEqual(text.count("<root"), 1)

    def test_legacy_bt_is_the_previous_tree(self):
        legacy = ET.parse(BT_LEGACY_PATH).getroot()
        main = legacy.find("BehaviorTree")[0]
        rec = main[1]
        self.assertEqual(rec.tag, "ReactiveFallback")
        seq = rec[1]
        self.assertEqual([k.tag for k in seq], ["Wait", "ForceSuccess", "ClearEntireCostmap", "ClearEntireCostmap"])
        self.assertEqual(seq[0].get("wait_duration"), "2.0")
        self.assertEqual(seq[1][0].attrib, {"backup_dist": "0.35", "backup_speed": "0.7", "time_allowance": "8.0"})
        tags = {e.tag for e in legacy.iter()}
        self.assertNotIn("WouldAControllerRecoveryHelp", tags)
        self.assertFalse(any("error_code_id" in e.attrib for e in legacy.iter()))

    def test_only_standard_nodes_with_valid_ports(self):
        for e in self.nodes():
            self.assertIn(e.tag, BT_ALLOWED, e.tag)
            extra = set(e.attrib) - BT_ALLOWED[e.tag]
            self.assertFalse(extra, (e.tag, extra))

    def test_ports_values(self):
        by = {}
        for e in self.nodes():
            by.setdefault(e.tag, []).append(e)
        # 計画/追従のプラグイン名がyamlの宣言と一致
        c = self.d["controller_server"]["ros__parameters"]
        p = self.d["planner_server"]["ros__parameters"]
        for e in by["FollowPath"]:
            self.assertIn(e.get("controller_id"), c["controller_plugins"])
            self.assertEqual(e.get("path"), "{path}")
        for e in by["ComputePathToPose"]:
            self.assertIn(e.get("planner_id"), p["planner_plugins"])
            self.assertEqual((e.get("goal"), e.get("path")), ("{goal}", "{path}"))
        # ClearEntireCostmapのサービス名は標準
        services = {e.get("service_name") for e in by["ClearEntireCostmap"]}
        self.assertEqual(services, {"global_costmap/clear_entirely_global_costmap",
                                    "local_costmap/clear_entirely_local_costmap"})
        # 再計画は1〜2 Hz
        rates = [float(e.get("hz")) for e in by["RateController"]]
        self.assertTrue(all(1.0 <= hz <= 2.0 for hz in rates), rates)
        # リトライ数
        retries = {e.get("name"): int(e.get("number_of_retries")) for e in by["RecoveryNode"]}
        self.assertEqual(retries["NavigateRecovery"], 2, retries)  # 2回回復してだめなら中止して、ルーターの迂回処理へ(docs/detour.md)
        # 数値の形式
        for tag, key in (("Wait", "wait_duration"), ("BackUp", "backup_dist"), ("BackUp", "backup_speed"),
                         ("BackUp", "time_allowance")):
            for e in by[tag]:
                self.assertGreater(float(e.get(key)), 0.0)

    def test_recovery_order_roundrobin_backup_spin_spin(self):
        main = self.tree["MainTree"][0]
        self.assertEqual((main.tag, main.get("name")), ("RecoveryNode", "NavigateRecovery"))
        kids = list(main)
        self.assertEqual(len(kids), 2)
        navigate, recovery = kids
        self.assertEqual(navigate.tag, "PipelineSequence")
        # 経路作成(RateController)の次が追従(FollowPath)
        self.assertEqual([k.tag for k in navigate], ["RateController", "FollowPath"])
        self.assertEqual(recovery.tag, "ReactiveFallback")
        self.assertEqual(recovery[0].tag, "GoalUpdated")  # 新しい目的地が来たら回復を打ち切る
        choice = recovery[1]
        self.assertEqual(choice.tag, "Fallback")
        self.assertEqual(len(choice), 2)
        helpful, replan_only = choice
        # 回復が役立つ失敗(進展なし等)だけ: 条件 -> RoundRobin(回復のたびに違う動き)
        self.assertEqual(helpful.tag, "Sequence")
        self.assertEqual([k.tag for k in helpful], ["WouldAControllerRecoveryHelp", "RoundRobin"])
        rr = helpful[1]
        self.assertEqual(len(rr), 3)
        back, left, right = rr
        # 1回目: Wait 2 -> BackUp -> コストマップ消去 x2 -> Wait 0.8(LiDARが障害物を書き込み直すのを待つ)
        self.assertEqual([k.tag for k in back], ["Wait", "ForceSuccess", "ClearEntireCostmap", "ClearEntireCostmap", "Wait"])
        self.assertEqual(back[1][0].tag, "BackUp")
        self.assertEqual(float(back[0].get("wait_duration")), 2.0)
        self.assertEqual(float(back[4].get("wait_duration")), 0.8)
        # 2回目: 左へ+90度 / 3回目: 右へ-90度(拒否されてもForceSuccess)。消去はしない(障害物をコストマップに残して迂回させる)
        for seq, sign in ((left, 1.0), (right, -1.0)):
            self.assertEqual([k.tag for k in seq], ["ForceSuccess", "Wait"])
            self.assertEqual(seq[0][0].tag, "Spin")
            self.assertAlmostEqual(float(seq[0][0].get("spin_dist")), sign * 1.57, places=2)
            self.assertEqual(float(seq[1].get("wait_duration")), 0.8)
            self.assertNotIn("ClearEntireCostmap", {e.tag for e in seq.iter()})
        # それ以外(TF_ERROR等): Wait -> コストマップ消去 -> 待ち(後退しない)
        self.assertEqual([k.tag for k in replan_only], ["Wait", "ClearEntireCostmap", "ClearEntireCostmap", "Wait"])
        self.assertNotIn("BackUp", {e.tag for e in replan_only.iter()})
        self.assertEqual(float(replan_only[0].get("wait_duration")), 1.0)
        self.assertEqual(float(replan_only[3].get("wait_duration")), 0.8)
        self.assertEqual(len([e for e in self.nodes() if e.tag == "BackUp"]), 1)  # 後退は1つだけ(毎回同じ後退を繰り返さない)

    def test_every_clear_costmap_is_followed_by_wait(self):
        # コストマップを消した直後に再計画すると、障害物が消えた経路が出てまた後退する。消去の列の直後は必ずWait
        for seq in (e for e in self.nodes() if e.tag == "Sequence"):
            kids = list(seq)
            for i, k in enumerate(kids):
                if k.tag == "ClearEntireCostmap" and (i + 1 == len(kids) or kids[i + 1].tag != "ClearEntireCostmap"):
                    self.assertTrue(i + 1 < len(kids) and kids[i + 1].tag == "Wait", seq.get("name"))
                    self.assertGreaterEqual(float(kids[i + 1].get("wait_duration")), 0.8)

    def test_backup_parameters(self):
        bu = next(e for e in self.nodes() if e.tag == "BackUp")
        dist, speed, allow = float(bu.get("backup_dist")), float(bu.get("backup_speed")), float(bu.get("time_allowance"))
        self.assertTrue(0.30 <= dist <= 0.40)
        self.assertEqual(speed, ns.BACKUP_SPEED_MPS)  # 通常の走行と同じ速さ
        self.assertGreaterEqual(allow, dist / speed + 2.0)  # 立ち上がりの余裕
        self.assertLessEqual(allow, 10.0)
        sm = self.d["velocity_smoother"]["ros__parameters"]
        self.assertLessEqual(sm["min_velocity"][0], -speed)  # velocity_smootherが後退を削らない
        # 後退の停止領域が後方を見る(後方30cm以上)
        rs = self.d["collision_monitor"]["ros__parameters"]["RearStop"]
        self.assertLessEqual(rs["backward"]["linear_min"], -speed)
        # behavior_serverにbackup/waitが登録されている
        bs = self.d["behavior_server"]["ros__parameters"]
        for name in ("backup", "wait", "spin"):
            self.assertIn(name, bs["behavior_plugins"])
        self.assertEqual(bs["backup"]["plugin"], "nav2_behaviors::BackUp")
        self.assertEqual(bs["wait"]["plugin"], "nav2_behaviors::Wait")

    def test_standstill_to_backup_seconds(self):
        allowance = self.d["controller_server"]["ros__parameters"]["progress_checker"]["movement_time_allowance"]
        wait = float(self.tree["MainTree"][0][1][1][0][1][0][0].get("wait_duration"))  # 後退する側(RoundRobin1つ目)のWait
        total = allowance + wait
        self.assertTrue(5.0 <= total <= 10.0, total)  # 「動けない」を検出してから後退を始めるまで 5〜10秒
        self.assertEqual(allowance, ns.GOAL_PRECISION[ns.NODE_CONTROLLER]["progress_checker.movement_time_allowance"])
        self.assertTrue(2.0 <= wait <= 3.0)
        # 6秒で足りる動き: 0.5 radの旋回(1.8 rad/sなら1秒未満)・0.15 mの前進(0.1 m/sでも1.5秒)
        self.assertGreater(allowance, 0.5 / 0.2 + 1.0)  # 最小旋回速度0.2 rad/sでも0.5 radに2.5秒 + 余裕
        pc = self.d["controller_server"]["ros__parameters"]["progress_checker"]
        self.assertEqual(pc["plugin"], "nav2_controller::PoseProgressChecker")
        self.assertGreater(allowance, pc["required_movement_radius"] / 0.05)  # 0.05 m/sでも進展と数えられる

    def test_cafeteria_tree_unchanged_and_uses_same_vocabulary(self):
        # 同じ語彙(既に動かした実績のあるBT)から外れていないことの参照
        other = ET.parse(BRINGUP / "behavior_trees" / "cafeteria_promoter_nav2.xml").getroot()
        used = {e.tag for e in other.iter()}
        ours = {e.tag for e in self.nodes()}
        extra = ours - used - {"BackUp", "ForceSuccess", "Fallback", "WouldAControllerRecoveryHelp", "RoundRobin", "Spin"}  # 追加したのはBackUp/ForceSuccess(Nav2標準)だけ
        self.assertFalse(extra, extra)


class TestLaunchRewrite(unittest.TestCase):
    def test_apply_bt_resolves_relative_name_beside_params(self):
        d = load_yaml()
        out = ns.apply_bt_to_params(d, str(BRINGUP / "config"))
        path = out["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"]
        self.assertTrue(os.path.isabs(path))
        self.assertTrue(os.path.isfile(path))
        self.assertEqual(d["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"], BT_PATH.name)  # 入力は不変

    def test_env_switch_selects_legacy_bt(self):
        d = load_yaml()
        cfg = str(BRINGUP / "config")
        key = "default_nav_to_pose_bt_xml"
        for env in ({}, {"NOKOSAHEN_BT": ""}, {"NOKOSAHEN_BT": "new"}):
            got = ns.apply_bt_to_params(d, cfg, env)["bt_navigator"]["ros__parameters"][key]
            self.assertEqual(os.path.basename(got), BT_PATH.name, env)  # 既定は新しいBT
        for v in ("legacy", " Legacy "):
            got = ns.apply_bt_to_params(d, cfg, {"NOKOSAHEN_BT": v})["bt_navigator"]["ros__parameters"][key]
            self.assertEqual(os.path.basename(got), BT_LEGACY_PATH.name)
            self.assertTrue(os.path.isfile(got))
        # 環境変数そのものを読む
        old = os.environ.get("NOKOSAHEN_BT")
        os.environ["NOKOSAHEN_BT"] = "legacy"
        try:
            got = ns.apply_bt_to_params(d, cfg)["bt_navigator"]["ros__parameters"][key]
        finally:
            if old is None:
                os.environ.pop("NOKOSAHEN_BT", None)
            else:
                os.environ["NOKOSAHEN_BT"] = old
        self.assertEqual(os.path.basename(got), BT_LEGACY_PATH.name)
        # legacyファイルが無ければ新しいBTのまま
        with tempfile.TemporaryDirectory() as tmp:
            Path(tmp, BT_PATH.name).write_text("<root/>", encoding="utf-8")
            got = ns.apply_bt_to_params(d, tmp, {"NOKOSAHEN_BT": "legacy"})["bt_navigator"]["ros__parameters"][key]
        self.assertEqual(os.path.basename(got), BT_PATH.name)

    def test_legacy_bt_installed(self):
        cm = (BRINGUP / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("config", cm)  # configフォルダごとインストール(legacyも同梱)
        self.assertTrue(BT_LEGACY_PATH.is_file())

    def test_missing_file_falls_back_to_standard_bt(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = ns.apply_bt_to_params(load_yaml(), tmp)
        self.assertNotIn("default_nav_to_pose_bt_xml", out["bt_navigator"]["ros__parameters"])

    def test_absolute_path_left_alone_and_no_bt_node_ok(self):
        d = {"bt_navigator": {"ros__parameters": {"default_nav_to_pose_bt_xml": os.path.abspath("x.xml")}}}
        self.assertEqual(ns.apply_bt_to_params(d, "/nowhere"), d)
        self.assertEqual(ns.apply_bt_to_params({"amcl": {}}, "/nowhere"), {"amcl": {}})

    def test_written_params_contain_everything(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = ns.write_rewritten_params(str(YAML_PATH), 0.7, os.path.join(tmp, "n.yaml"))
            got = yaml.safe_load(Path(out).read_text(encoding="utf-8"))
        want = ns.derive_clearance_values()
        c = got["controller_server"]["ros__parameters"]
        self.assertEqual(c["FollowPath"]["max_vel_x"], 0.7)
        self.assertEqual(c["FollowPath"]["max_vel_theta"], ns.BASE_THETA_VEL)
        self.assertEqual(c["progress_checker"]["movement_time_allowance"], 6.0)
        self.assertEqual(got["collision_monitor"]["ros__parameters"]["VelocityStop"], want["stop_polygon"])
        self.assertEqual(got["collision_monitor"]["ros__parameters"]["RearStop"], want["rear_polygon"])
        self.assertEqual(got["collision_monitor"]["ros__parameters"]["polygons"], ["FootprintApproach", "VelocityStop", "RearStop"])
        self.assertEqual(got["collision_monitor"]["ros__parameters"]["FootprintApproach"]["time_before_collision"], 1.5)
        self.assertEqual(got["local_costmap"]["local_costmap"]["ros__parameters"]["inflation_layer"]["inflation_radius"],
                         want["inflation_radius"])
        self.assertTrue(os.path.isabs(got["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"]))
        self.assertEqual(got["velocity_smoother"]["ros__parameters"]["max_velocity"], [0.7, 0.0, ns.BASE_THETA_VEL])

    def test_goal_router_pre_rotate_uses_behavior_server_spin(self):
        text = (PKG / "nokosahen_core" / "nav2_goal_router.py").read_text(encoding="utf-8")
        self.assertIn('ActionClient(self, Spin, "spin")', text)  # spinはbehavior_serverの上限(1.8 rad/s)に従う
        # 20秒の許容時間は旋回(180度で約3秒)に十分
        self.assertIn('declare_parameter("pre_rotate_time_allowance", 20.0)', text)


if __name__ == "__main__":
    unittest.main()
