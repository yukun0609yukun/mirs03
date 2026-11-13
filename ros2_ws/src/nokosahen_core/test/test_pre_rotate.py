"""目的地が背後寄りのとき先にその場で旋回する判定(pre_rotation_angle)の単体テスト。

rclpy不要(未インストール時はスタブ注入)。ロボット・ROSには触れない。
実行: python ros2_ws/src/nokosahen_core/test/test_pre_rotate.py
"""
import math
import os
import sys
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

for _name, _attrs in (
    ("rclpy", ()), ("rclpy.action", ("ActionClient",)), ("rclpy.duration", ("Duration",)),
    ("rclpy.node", ("Node",)), ("geometry_msgs", ()), ("geometry_msgs.msg", ("PoseStamped", "Twist")),
    ("std_msgs", ()), ("std_msgs.msg", ("Bool", "String")),
):
    try:
        __import__(_name)
    except Exception:  # noqa: BLE001
        _m = types.ModuleType(_name)
        for _a in _attrs:
            setattr(_m, _a, object)
        sys.modules[_name] = _m

from nokosahen_core.nav2_goal_router import pre_rotation_angle, wrap_angle  # noqa: E402


class PreRotationTests(unittest.TestCase):
    def test_goal_ahead_no_rotation(self):
        self.assertIsNone(pre_rotation_angle(0, 0, 0.0, 2.0, 0.0))

    def test_goal_to_the_side_no_rotation(self):
        # 真横(90°)はしきい値100°未満なので、Nav2に任せる
        self.assertIsNone(pre_rotation_angle(0, 0, 0.0, 0.0, 2.0))

    def test_goal_behind_rotates_about_180(self):
        d = pre_rotation_angle(0, 0, 0.0, -2.0, 0.0)
        self.assertIsNotNone(d)
        self.assertAlmostEqual(abs(d), math.pi, places=6)

    def test_sign_turns_toward_goal(self):
        # 左後ろ(+y側)なら正、右後ろなら負の相対角
        self.assertGreater(pre_rotation_angle(0, 0, 0.0, -2.0, 1.0), 0)
        self.assertLess(pre_rotation_angle(0, 0, 0.0, -2.0, -1.0), 0)

    def test_robot_yaw_is_respected(self):
        # 機体が+y向き(90°)なら、-y側の目的地が真後ろ
        d = pre_rotation_angle(0, 0, math.pi / 2, 0.0, -2.0)
        self.assertAlmostEqual(abs(d), math.pi, places=6)
        self.assertIsNone(pre_rotation_angle(0, 0, math.pi / 2, 0.0, 2.0))

    def test_near_goal_no_rotation(self):
        self.assertIsNone(pre_rotation_angle(0, 0, 0.0, -0.4, 0.0))

    def test_threshold_boundary(self):
        # 99°は回さず、101°は回す
        for deg, expect in ((99.0, False), (101.0, True)):
            a = math.radians(deg)
            r = pre_rotation_angle(0, 0, 0.0, 2 * math.cos(a), 2 * math.sin(a))
            self.assertEqual(r is not None, expect, deg)

    def test_non_finite_is_safe(self):
        self.assertIsNone(pre_rotation_angle(0, 0, float("nan"), -2.0, 0.0))
        self.assertIsNone(pre_rotation_angle(0, 0, 0.0, float("inf"), 0.0))

    def test_wrap(self):
        self.assertAlmostEqual(wrap_angle(3 * math.pi), math.pi, places=6)


if __name__ == "__main__":
    unittest.main()
