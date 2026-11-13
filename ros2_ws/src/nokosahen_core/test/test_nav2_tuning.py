"""Nav2自律走行の堅牢化(2026-10-08)のテスト。ROS 2もロボットも不要(rclpy系はスタブ)。

- nav2_params_jetson.yaml: PoseProgressChecker、transform_tolerance 0.6、failure_tolerance
- nav_speed.GOAL_PRECISION(起動時の書き換え)がyamlと整合し、速度の書き換えを壊さない
- slam_toolbox_online_async.yamlのCPU軽減設定
- MapGoalController.map_json()のキャッシュ(同一出力・新しい地図/clearで無効化・map_idの増加)
- frontier_explorer: 無効の間は/mapも/tfも購読せず、_tickは重い処理をしない

実行: python ros2_ws/src/nokosahen_core/test/test_nav2_tuning.py
"""
import base64
import sys
import types
import unittest
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parents[1]
BRINGUP = PKG.parent / "nokosahen_bringup"
sys.path.insert(0, str(PKG))


def _mod(name):
    m = sys.modules.get(name)
    if m is None:
        m = types.ModuleType(name)
        sys.modules[name] = m
    return m


class _Msg:
    def __init__(self, *a, **k):
        self.data = None


class StubNode:
    def __init__(self, name="n"):
        self._params = {}
        self.subs = {}
        self.destroyed = []

    def declare_parameter(self, name, value):
        self._params[name] = value

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self._params[name])

    def create_subscription(self, typ, topic, cb, qos):
        sub = types.SimpleNamespace(topic=topic, cb=cb)
        self.subs[topic] = sub
        return sub

    def destroy_subscription(self, sub):
        self.destroyed.append(sub.topic)
        self.subs.pop(sub.topic, None)

    def create_publisher(self, typ, topic, qos):
        return types.SimpleNamespace(publish=lambda m: None)

    def create_timer(self, period, cb):
        return None

    def get_logger(self):
        lg = types.SimpleNamespace()
        lg.info = lg.warn = lg.error = lg.debug = lambda *a, **k: None
        return lg


def _install_stubs():
    _mod("geometry_msgs")
    _mod("geometry_msgs.msg").__dict__.update(PoseStamped=_Msg, Twist=_Msg)
    _mod("nav_msgs")
    _mod("nav_msgs.msg").__dict__.update(Odometry=_Msg, OccupancyGrid=_Msg)
    _mod("std_msgs")
    _mod("std_msgs.msg").__dict__.update(Bool=_Msg, String=_Msg, Float32MultiArray=_Msg)
    _mod("rclpy")
    _mod("rclpy.node").__dict__["Node"] = StubNode
    _mod("rclpy.time").__dict__["Time"] = lambda: None


_install_stubs()

from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import nav_speed as ns  # noqa: E402


def _load(name):
    return yaml.safe_load((BRINGUP / "config" / name).read_text(encoding="utf-8"))


class TestNav2Yaml(unittest.TestCase):
    def setUp(self):
        self.d = _load("nav2_params_jetson.yaml")
        self.c = self.d["controller_server"]["ros__parameters"]

    def test_pose_progress_checker(self):
        pc = self.c["progress_checker"]
        self.assertEqual(pc["plugin"], "nav2_controller::PoseProgressChecker")
        self.assertEqual(pc["required_movement_radius"], 0.15)
        self.assertEqual(pc["required_movement_angle"], 0.5)
        # 【2026-10-10】30 -> 6秒(旋回が速くなり、立ちふさがりを早く検出してBTの後退へ進める)
        self.assertEqual(pc["movement_time_allowance"], 6.0)
        self.assertEqual(self.c["progress_checker_plugin"], "progress_checker")

    def test_transform_tolerance_raised(self):
        self.assertEqual(self.c["FollowPath"]["transform_tolerance"], 1.2)  # 2026-10-23: 0.6→1.2(負荷でmap→odomが遅れても追従を即失敗にしない)
        for n in ("local_costmap", "global_costmap"):
            self.assertEqual(self.d[n][n]["ros__parameters"]["transform_tolerance"], 0.6, n)
        self.assertEqual(self.d["behavior_server"]["ros__parameters"]["transform_tolerance"], 0.6)
        self.assertEqual(self.d["bt_navigator"]["ros__parameters"]["transform_tolerance"], 0.6)
        # 安全層のcollision_monitorは古い姿勢で判定しないよう据え置き
        self.assertEqual(self.d["collision_monitor"]["ros__parameters"]["transform_tolerance"], 0.2)

    def test_failure_tolerance_small(self):
        ft = self.c["failure_tolerance"]
        self.assertTrue(0.0 < ft <= 1.0)

    def test_rotation_not_forbidden(self):
        f = self.c["FollowPath"]
        self.assertEqual(f["min_vel_x"], 0.0)  # vx=0(その場旋回)の候補が残る
        self.assertGreaterEqual(f["vtheta_samples"], 10)
        # 【2026-10-10】角加速は3.0へ(ファームのRAMP_ACCEL_ANGULAR_RPS2=1.6が実効の上限になる。ファーム側を上げるまで
        # 実機の立ち上がりは1.6のまま。減速はファームのRAMP_DECEL_ANGULAR_RPS2=4.0以下)
        self.assertGreaterEqual(f["acc_lim_theta"], 1.6)
        self.assertLessEqual(-f["decel_lim_theta"], 4.0)
        self.assertEqual(f["min_speed_xy"], 0.0)  # min_speed_thetaによる「停止禁止」を有効にしない

    def test_slam_cpu_settings(self):
        s = _load("slam_toolbox_online_async.yaml")["slam_toolbox"]["ros__parameters"]
        self.assertEqual(s["transform_publish_period"], 0.05)
        self.assertEqual(s["map_update_interval"], 3.0)
        # マッピング品質に効く値は不変
        self.assertEqual(s["minimum_travel_distance"], 0.08)
        self.assertEqual(s["minimum_travel_heading"], 0.08)
        self.assertEqual(s["minimum_time_interval"], 0.2)
        self.assertEqual(s["resolution"], 0.05)

    def test_inference_divisor_in_yaml(self):
        d = _load("nokosahen.yaml")["person_target_detector"]["ros__parameters"]
        self.assertEqual(d["mapping_inference_divisor"], 4)


class TestGoalPrecisionRewrite(unittest.TestCase):
    def test_rewrite_consistent_with_yaml(self):
        d = _load("nav2_params_jetson.yaml")
        out = ns.apply_goal_precision_to_params(ns.apply_nav_speed_to_params(d, 0.5))
        c = out["controller_server"]["ros__parameters"]
        pc = c["progress_checker"]
        yc = d["controller_server"]["ros__parameters"]["progress_checker"]
        self.assertEqual(pc["movement_time_allowance"], yc["movement_time_allowance"])
        self.assertEqual(pc["plugin"], "nav2_controller::PoseProgressChecker")  # 書き換えでプラグインを戻さない
        self.assertEqual(pc["required_movement_angle"], 0.5)
        # transform_toleranceは速度/精度どちらの書き換えも触らない
        self.assertEqual(c["FollowPath"]["transform_tolerance"], 1.2)  # 2026-10-23: 0.6→1.2

    def test_raspi4_still_gets_allowance_only(self):
        out = ns.apply_goal_precision_to_params(_load("nav2_params_raspi4.yaml"))
        pc = out["controller_server"]["ros__parameters"]["progress_checker"]
        self.assertEqual(pc["movement_time_allowance"], 6.0)
        self.assertEqual(pc["plugin"], "nav2_controller::SimpleProgressChecker")


class FakeInfo:
    def __init__(self, w, h):
        self.width, self.height, self.resolution = w, h, 0.05
        self.origin = types.SimpleNamespace(position=types.SimpleNamespace(x=-1.0, y=2.0))


class FakeMap:
    def __init__(self, w, h, data):
        self.info = FakeInfo(w, h)
        self.data = data


class CountingList(list):
    """map_jsonがdataを何回走査したかを数える。"""
    iterations = 0

    def __iter__(self):
        CountingList.iterations += 1
        return super().__iter__()


class TestMapJsonCache(unittest.TestCase):
    def setUp(self):
        self.saved = (dc.PoseStamped, dc.Bool)
        dc.PoseStamped = dc.Bool = _Msg
        self.mg = dc.MapGoalController(StubNode())
        CountingList.iterations = 0

    def tearDown(self):
        dc.PoseStamped, dc.Bool = self.saved

    @staticmethod
    def _uncached(m):
        raw = bytes((v + 1) & 0xFF for v in m.data)
        return base64.b64encode(raw).decode("ascii")

    def test_no_map(self):
        self.assertIsNone(self.mg.map_json())

    def test_same_output_and_cached(self):
        m = FakeMap(3, 2, CountingList([-1, 0, 100, 50, -1, 0]))
        self.mg._on_map(m)
        a = self.mg.map_json()
        b = self.mg.map_json()
        self.assertEqual(CountingList.iterations, 1)  # 2回目は再計算しない
        self.assertIs(a, b)
        self.assertEqual(a["data_base64"], self._uncached(m))
        for k, v in (("width", 3), ("height", 2), ("resolution", 0.05), ("origin_x", -1.0), ("origin_y", 2.0)):
            self.assertEqual(a[k], v)  # 従来のキーは不変
        self.assertIn("map_id", a)

    def test_invalidated_on_new_map(self):
        self.mg._on_map(FakeMap(2, 1, CountingList([0, 100])))
        a = self.mg.map_json()
        m2 = FakeMap(2, 1, CountingList([100, -1]))
        self.mg._on_map(m2)
        b = self.mg.map_json()
        self.assertNotEqual(a["data_base64"], b["data_base64"])
        self.assertEqual(b["data_base64"], self._uncached(m2))
        self.assertGreater(b["map_id"], a["map_id"])

    def test_same_content_new_message_gets_new_id(self):
        self.mg._on_map(FakeMap(2, 1, [0, 100]))
        a = self.mg.map_json()
        self.mg._on_map(FakeMap(2, 1, [0, 100]))
        self.assertGreater(self.mg.map_json()["map_id"], a["map_id"])

    def test_invalidated_on_clear(self):
        self.mg._on_map(FakeMap(2, 1, [0, 100]))
        self.assertIsNotNone(self.mg.map_json())
        self.mg.clear()
        self.assertIsNone(self.mg.map_json())
        m = FakeMap(2, 1, [0, 100])
        self.mg._on_map(m)
        self.assertEqual(self.mg.map_json()["data_base64"], self._uncached(m))

    def test_direct_latest_map_assignment_still_works(self):
        # 既存テストはlatest_mapを直接代入する。その場合もキャッシュが古い地図を返さない。
        self.mg.latest_map = FakeMap(2, 1, [0, 100])
        self.mg.map_json()
        m2 = FakeMap(2, 1, [100, 100])
        self.mg.latest_map = m2
        self.assertEqual(self.mg.map_json()["data_base64"], self._uncached(m2))


class FakeTfListener:
    instances = []

    def __init__(self, buf, node):
        self.unregistered = False
        FakeTfListener.instances.append(self)

    def unregister(self):
        self.unregistered = True


class TestFrontierIdle(unittest.TestCase):
    def setUp(self):
        tf = _mod("tf2_ros")
        tf.Buffer = lambda: types.SimpleNamespace()
        tf.TransformListener = FakeTfListener
        FakeTfListener.instances = []
        from nokosahen_core import frontier_explorer as fe
        fe.Buffer, fe.TransformListener = tf.Buffer, tf.TransformListener
        self.n = fe.FrontierExplorer()
        self.heavy = 0
        self.n._publish_cancel_goal = lambda: None
        self.status = []
        self.n._publish_status = lambda extra=None: self.status.append(extra)

        def heavy(*a, **k):
            self.heavy += 1
            return []
        self.n._find_frontiers = heavy
        self.n._get_pose = heavy

    def test_idle_does_not_subscribe_map_or_tf(self):
        self.assertNotIn("/map", self.n.subs)
        self.assertEqual(FakeTfListener.instances, [])
        self.assertIsNone(self.n._map_sub)

    def test_idle_tick_is_cheap(self):
        for _ in range(50):
            self.n._tick()
        self.assertEqual(self.heavy, 0)  # 地図処理も姿勢取得もしない
        self.assertEqual(len(self.status), 50)  # 状態配信(DDS discovery対策)は従来どおり

    def test_enable_subscribes_then_disable_releases(self):
        on = types.SimpleNamespace(data=True)
        self.n._on_enabled(on)
        self.assertIn("/map", self.n.subs)
        self.assertEqual(len(FakeTfListener.instances), 1)
        self.n._on_enabled(on)  # 二重のenableでも購読を増やさない
        self.assertEqual(len(FakeTfListener.instances), 1)
        self.n._latest_map = object()
        self.n._on_enabled(types.SimpleNamespace(data=False))
        self.assertNotIn("/map", self.n.subs)
        self.assertTrue(FakeTfListener.instances[0].unregistered)
        self.assertIsNone(self.n._latest_map)
        self.assertEqual(self.n._status, "idle")
        self.n._on_enabled(on)  # 再度有効化できる(新しいlistener)
        self.assertEqual(len(FakeTfListener.instances), 2)

    def test_enabled_without_map_waits(self):
        self.n._on_enabled(types.SimpleNamespace(data=True))
        self.n._tick()
        self.assertEqual(self.status[-1], {"reason": "waiting_for_map"})
        self.assertEqual(self.heavy, 0)


if __name__ == "__main__":
    unittest.main()
