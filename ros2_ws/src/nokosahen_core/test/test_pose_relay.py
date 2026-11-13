"""pose_relay(2026-10-26)の純粋な部分と、3ノードが /tf を直接購読しなくなったことの確認。ROSは使わない。
実行: python3 -I test/test_pose_relay.py   (nokosahen_coreディレクトリで)
"""
import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core import pose_relay as pr  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
CORE = PKG / "nokosahen_core"
BRINGUP = PKG.parent / "nokosahen_bringup"
ROOT = PKG.parent.parent.parent


def src(name):
    return (CORE / name).read_text(encoding="utf-8")


class BuildMessageTest(unittest.TestCase):
    def test_contains_tf_stamp_and_pose(self):
        d = json.loads(pr.build_pose_message((100.5, 1.0, 2.0, 0.5), 99.0))
        self.assertEqual(d["base"], {"stamp": 100.5, "x": 1.0, "y": 2.0, "yaw": 0.5})
        self.assertEqual(d["map_odom"], {"stamp": 99.0})

    def test_missing_transform_is_not_published(self):
        self.assertIsNone(pr.build_pose_message(None, None))
        d = json.loads(pr.build_pose_message(None, 5.0))
        self.assertNotIn("base", d)
        d = json.loads(pr.build_pose_message((1.0, 0.0, 0.0, 0.0), None))
        self.assertNotIn("map_odom", d)

    def test_non_finite_values_are_dropped(self):
        self.assertIsNone(pr.build_pose_message((1.0, float("nan"), 0.0, 0.0), float("inf")))

    def test_yaw_and_stamp(self):
        self.assertAlmostEqual(pr.yaw_from_quaternion(0, 0, math.sin(0.4), math.cos(0.4)), 0.8)
        self.assertAlmostEqual(pr.stamp_to_sec(10, 500_000_000), 10.5)


class PoseCacheTest(unittest.TestCase):
    def test_never_received_is_unknown(self):
        c = pr.PoseCache()
        self.assertIsNone(c.base(10.0))
        self.assertIsNone(c.map_odom_age(10.0))

    def test_age_is_computed_from_tf_stamp_not_from_arrival(self):
        c = pr.PoseCache()
        self.assertTrue(c.update(pr.build_pose_message((100.0, 1.0, 2.0, 0.3), 100.0)))
        x, y, yaw, age = c.base(100.4)
        self.assertEqual((x, y, yaw), (1.0, 2.0, 0.3))
        self.assertAlmostEqual(age, 0.4)
        # 中継ノードが止まる/TFが途絶えると、同じ値のまま古さだけが増える(消費側の「古ければ判断しない」に落ちる)
        self.assertAlmostEqual(c.base(103.0)[3], 3.0)
        self.assertAlmostEqual(c.map_odom_age(103.0), 3.0)

    def test_transform_lost_in_latest_message_clears_value(self):
        c = pr.PoseCache()
        c.update(pr.build_pose_message((100.0, 1.0, 2.0, 0.3), 100.0))
        c.update(pr.build_pose_message(None, 101.0))   # map→base_linkが取れなくなった
        self.assertIsNone(c.base(101.0))
        self.assertAlmostEqual(c.map_odom_age(101.5), 0.5)

    def test_garbage_is_ignored_and_keeps_previous(self):
        c = pr.PoseCache()
        c.update(pr.build_pose_message((100.0, 1.0, 2.0, 0.3), None))
        for bad in ("not json", "[]", '{"base": {"stamp": 1}}', '{"base": {"stamp": "a", "x": 0, "y": 0, "yaw": 0}}'):
            self.assertFalse(c.update(bad), bad)
        self.assertEqual(c.base(100.0)[:3], (1.0, 2.0, 0.3))


class WiringTest(unittest.TestCase):
    NODES = ("nav2_goal_router.py", "wall_escape.py", "motion_arbiter.py", "nav2_detour.py")

    def test_consumers_do_not_listen_to_tf(self):
        for n in self.NODES:
            s = src(n)
            self.assertNotIn("TransformListener", s, n)
            self.assertNotIn("lookup_transform", s, n)
            self.assertNotIn("tf2_ros", s, n)

    def test_consumers_use_the_relay_topic(self):
        for n in ("nav2_goal_router.py", "wall_escape.py", "motion_arbiter.py"):
            s = src(n)
            self.assertIn("ROBOT_POSE_TOPIC", s, n)
            self.assertIn("PoseCache", s, n)

    def test_staleness_checks_are_kept(self):
        r = src("nav2_goal_router.py")
        self.assertIn("age > 1.5", r)                      # 先回し旋回の判断
        self.assertIn("bp[3] > 1.5", r)                    # 回頭の向き誤差
        self.assertIn("bp[3] > 1.5", src("nav2_detour.py"))
        self.assertIn("bp[3] > 1.5", src("wall_escape.py"))
        self.assertIn("map_odom_age", src("motion_arbiter.py"))
        self.assertIn("map→odom 取得不可", src("motion_arbiter.py"))

    def test_only_relay_listens_to_tf(self):
        self.assertIn("TransformListener(self._buf, self)", src("pose_relay.py"))

    def test_registered(self):
        self.assertIn("pose_relay = nokosahen_core.pose_relay:main", (PKG / "setup.py").read_text(encoding="utf-8"))
        for f in ("localization_mode", "mapping_mode", "localization_robot", "raspi4_robot"):
            t = (BRINGUP / "launch" / f"{f}.launch.py").read_text(encoding="utf-8")
            self.assertIn('executable="pose_relay"', t, f)
            self.assertLess(t.index('executable="pose_relay"'), t.index('executable="nav2_goal_router"'), f)
        self.assertIn('"pose_relay"', (ROOT / "scripts" / "stop_robot.sh").read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main(verbosity=1)
