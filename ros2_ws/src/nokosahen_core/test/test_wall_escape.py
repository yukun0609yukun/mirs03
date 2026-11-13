"""壁際からの脱出(wall_escape.py)の、純粋な関数のテスト(2026-10-22)。ROSは使わない。
実行: python ros2_ws/src/nokosahen_core/test/test_wall_escape.py
"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core import wall_escape as we  # noqa: E402

DEG = math.pi / 180.0


class ScanPointsTests(unittest.TestCase):
    def test_filters_invalid_and_far(self):
        pts = we.scan_points([float("inf"), float("nan"), 0.02, 0.3, 0.9, 2.0], 0.0, 0.1, 0.0, 10.0, 0.7)
        self.assertEqual([round(r, 2) for r, _ in pts], [0.3])

    def test_angles(self):
        pts = we.scan_points([0.3, 0.3], -math.pi, math.pi, 0.0, 10.0, 0.7)
        self.assertAlmostEqual(pts[1][1], 0.0)

    def test_min_range(self):
        self.assertIsNone(we.min_range([]))
        self.assertEqual(we.min_range([(0.5, 0.0), (0.3, 1.0)]), 0.3)


class FleeTests(unittest.TestCase):
    def test_wall_behind_flees_forward(self):
        pts = [(0.35, math.pi), (0.36, math.pi - 0.1), (0.36, -math.pi + 0.1)]
        a = we.flee_angle(pts, 0.7)
        self.assertLess(abs(a), 10 * DEG)

    def test_wall_left_flees_right(self):
        pts = [(0.35, 90 * DEG), (0.36, 80 * DEG), (0.36, 100 * DEG)]
        a = we.flee_angle(pts, 0.7)
        self.assertLess(a, -80 * DEG)
        self.assertGreater(a, -100 * DEG)

    def test_wall_front_flees_backward(self):
        a = we.flee_angle([(0.3, 0.0), (0.3, 0.1), (0.3, -0.1)], 0.7)
        self.assertGreater(abs(a), 170 * DEG)

    def test_far_points_ignored(self):
        self.assertIsNone(we.flee_angle([(0.8, 0.0)], 0.7))
        self.assertIsNone(we.flee_angle([], 0.7))

    def test_nearer_points_dominate(self):
        a = we.flee_angle([(0.2, 90 * DEG), (0.65, -90 * DEG)], 0.7)
        self.assertLess(a, -60 * DEG)


class CommandTests(unittest.TestCase):
    def test_forward(self):
        lin, ang = we.escape_command(0.0, 0.1, 0.6)
        self.assertEqual((lin, ang), (0.1, 0.0))

    def test_turn_is_clamped_and_signed(self):
        lin, ang = we.escape_command(50 * DEG, 0.1, 0.6)
        self.assertEqual(lin, 0.1)
        self.assertAlmostEqual(ang, 0.6)
        lin, ang = we.escape_command(-50 * DEG, 0.1, 0.6)
        self.assertAlmostEqual(ang, -0.6)

    def test_sideways_turns_mostly(self):
        lin, ang = we.escape_command(-90 * DEG, 0.1, 0.6)
        self.assertAlmostEqual(lin, 0.05)
        self.assertAlmostEqual(ang, -0.6)

    def test_backward_reverses_at_the_same_speed(self):
        lin, ang = we.escape_command(math.pi, 0.1, 0.6)
        self.assertAlmostEqual(lin, -0.1)  # 後退も前進と同じ速さ
        self.assertEqual(ang, 0.0)

    def test_speed_never_exceeds_limit(self):
        for deg in range(-180, 181, 10):
            lin, ang = we.escape_command(deg * DEG, 0.1, 0.6)
            self.assertLessEqual(abs(lin), 0.1 + 1e-9)
            self.assertLessEqual(abs(ang), 0.6 + 1e-9)


class WiringTests(unittest.TestCase):
    ROOT = Path(__file__).resolve().parents[2]

    def test_entry_point_and_launch(self):
        self.assertIn("wall_escape = nokosahen_core.wall_escape:main", (self.ROOT / "nokosahen_core" / "setup.py").read_text(encoding="utf-8"))
        for f in ("mapping_mode", "localization_mode"):
            src = (self.ROOT / "nokosahen_bringup" / "launch" / f"{f}.launch.py").read_text(encoding="utf-8")
            self.assertIn('executable="wall_escape"', src, f)

    def test_commands_go_through_the_safety_chain(self):
        src = (Path(we.__file__)).read_text(encoding="utf-8")
        self.assertIn('"/cmd_vel_nav"', src)
        self.assertNotIn('create_publisher(Twist, "/cmd_vel",', src)


class RouterFallbackTests(unittest.TestCase):
    """背後の目的地に走れない件(2026-10-22)の対策が、ルーターに入っていることの静的な確認。"""

    SRC = (Path(__file__).resolve().parents[1] / "nokosahen_core" / "nav2_goal_router.py").read_text(encoding="utf-8")

    def test_fallback_rotation_uses_the_safety_chain(self):
        self.assertIn('create_publisher(Twist, "/cmd_vel_nav"', self.SRC)
        self.assertIn("_begin_fallback_rotation", self.SRC)

    def test_tf_age_limit_relaxed(self):
        self.assertIn("age > 1.5", self.SRC)
        self.assertNotIn("age > 0.5", self.SRC)

    def test_active_goal_cancelled_before_pre_rotation(self):
        i = self.SRC.index("delta = self._pre_rotation_for(pose)")
        self.assertIn("cancel_goal_async", self.SRC[i:i + 900])


if __name__ == "__main__":
    unittest.main(verbosity=1)
