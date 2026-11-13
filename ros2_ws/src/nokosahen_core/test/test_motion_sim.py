"""先読みの衝突シミュレーション(motion_sim.py)のテスト(2026-10-22)。ROS不要。
実行: python ros2_ws/src/nokosahen_core/test/test_motion_sim.py
"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core import motion_sim as ms  # noqa: E402

H = ms.BODY_HALF_M
CIRC = math.hypot(H, H)  # 外接半径 0.368


def wall(x0, y0, x1, y1, n=60):
    return [(x0 + (x1 - x0) * i / (n - 1), y0 + (y1 - y0) * i / (n - 1)) for i in range(n)]


class GeometryTests(unittest.TestCase):
    def test_pose_straight_and_arc(self):
        self.assertEqual(ms.pose_at(0.2, 0.0, 2.0), (0.4, 0.0, 0.0))
        x, y, th = ms.pose_at(0.2, 0.5, math.pi)  # 半径0.4の円を、旋回量 0.5*pi = 90°だけ進む(四分円)
        self.assertAlmostEqual(x, 0.4, places=6)
        self.assertAlmostEqual(y, 0.4, places=6)
        self.assertAlmostEqual(th, math.pi * 0.5, places=6)

    def test_inside_body(self):
        pose = (0.0, 0.0, 0.0)
        self.assertTrue(ms.inside_body(0.25, 0.25, pose, H))
        self.assertFalse(ms.inside_body(0.27, 0.0, pose, H))
        # 45°回すと、前の中央の点0.30 mは外形の外、角の方向の点(0.30,0.0を45°回した位置)は中
        pose45 = (0.0, 0.0, math.pi / 4)
        self.assertFalse(ms.inside_body(0.30, 0.0, pose45, H) is False)  # 45°回ると角(0.368)がx軸上に来るので、0.30は中
        self.assertTrue(ms.inside_body(0.36, 0.0, pose45, H))  # 角(0.368)が来る向き
        self.assertFalse(ms.inside_body(0.38, 0.0, pose45, H))

    def test_min_clearance(self):
        self.assertAlmostEqual(ms.min_clearance([(0.5, 0.0)], (0.0, 0.0, 0.0)), 0.5 - H)
        self.assertEqual(ms.min_clearance([(0.1, 0.0)], (0.0, 0.0, 0.0)), 0.0)


class CollisionTests(unittest.TestCase):
    def test_no_points_no_collision(self):
        self.assertIsNone(ms.first_collision_time([], 0.2, 0.5))

    def test_straight_into_wall(self):
        pts = wall(0.6, -0.5, 0.6, 0.5)
        t = ms.first_collision_time(pts, 0.2, 0.0, horizon=3.0)
        self.assertIsNotNone(t)
        self.assertAlmostEqual(t, (0.6 - H - 0.02) / 0.2, delta=0.1)
        self.assertIsNone(ms.first_collision_time(pts, -0.1, 0.0, horizon=3.0))  # 後退なら当たらない

    def test_rotation_in_place_hits_when_corner_reaches(self):
        # 壁が中心から0.34 mの左側: 角(0.368)が回ると当たる。0.40 mなら当たらない
        near = wall(-0.5, 0.34, 0.5, 0.34)
        far = wall(-0.5, 0.40, 0.5, 0.40)
        self.assertIsNotNone(ms.first_collision_time(near, 0.0, 0.6, horizon=2.0))
        self.assertIsNone(ms.first_collision_time(far, 0.0, 0.6, horizon=2.0))

    def test_already_inside_returns_zero(self):
        self.assertEqual(ms.first_collision_time([(0.1, 0.1)], 0.1, 0.0), 0.0)


class PickMotionTests(unittest.TestCase):
    def test_wall_behind_picks_forward(self):
        pts = wall(-0.35, -0.6, -0.35, 0.6)
        m = ms.pick_motion(pts, flee=0.0)
        self.assertIsNotNone(m)
        self.assertGreater(m[0], 0.0)

    def test_wall_left_and_behind_never_collides(self):
        pts = wall(-0.36, -0.6, -0.36, 0.6) + wall(-0.5, 0.36, 0.6, 0.36)
        m = ms.pick_motion(pts, flee=math.radians(-45))
        self.assertIsNotNone(m)
        self.assertIsNone(ms.first_collision_time(pts, m[0], m[1], horizon=1.5, margin=0.02))

    def test_boxed_in_returns_none(self):
        # 四方0.30 m: 動けば必ず当たる
        pts = wall(-0.3, -0.3, 0.3, -0.3) + wall(-0.3, 0.3, 0.3, 0.3) + wall(0.3, -0.3, 0.3, 0.3) + wall(-0.3, -0.3, -0.3, 0.3)
        self.assertIsNone(ms.pick_motion(pts, flee=None))

    def test_open_space_returns_a_motion(self):
        self.assertIsNotNone(ms.pick_motion([], flee=0.0))

    def test_every_candidate_choice_is_collision_free(self):
        import random
        random.seed(1)
        for _ in range(200):
            pts = [(random.uniform(-0.9, 0.9), random.uniform(-0.9, 0.9)) for _ in range(40)]
            pts = [p for p in pts if max(abs(p[0]), abs(p[1])) > H + 0.03]  # 出発時に外形にめり込む点は除く
            m = ms.pick_motion(pts, flee=random.uniform(-math.pi, math.pi))
            if m is not None:
                self.assertIsNone(ms.first_collision_time(pts, m[0], m[1], horizon=1.5, margin=0.02))


class WiringTests(unittest.TestCase):
    ROOT = Path(__file__).resolve().parents[1] / "nokosahen_core"

    def test_wall_escape_uses_the_simulation(self):
        src = (self.ROOT / "wall_escape.py").read_text(encoding="utf-8")
        self.assertIn("pick_motion", src)

    def test_router_checks_rotation_before_fallback_turn(self):
        src = (self.ROOT / "nav2_goal_router.py").read_text(encoding="utf-8")
        self.assertIn("_rotation_would_collide", src)
        self.assertIn("first_collision_time", src)

    def test_router_skips_pre_rotation_when_corner_would_hit_wall(self):
        # 2026-10-26: 壁際ではSpinが進まず20秒待たされたので、先回し旋回の判断の段階で壁判定を使って省く
        src = (self.ROOT / "nav2_goal_router.py").read_text(encoding="utf-8")
        body = src[src.index("def _pre_rotation_for"):src.index("def _on_spin_response")]
        self.assertIn("_rotation_would_collide(math.copysign(BASE_THETA_VEL, delta))", body)
        self.assertIn("return None", body[body.index("_rotation_would_collide"):])


if __name__ == "__main__":
    unittest.main(verbosity=1)
