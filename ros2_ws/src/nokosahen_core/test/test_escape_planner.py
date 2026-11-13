"""壁を避ける中継点(escape_planner.py)と、壁沿いにすべる動き(motion_sim.pick_motion)のテスト(2026-10-22)。ROS不要。
実行: python3 -I ros2_ws/src/nokosahen_core/test/test_escape_planner.py
"""
import math
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core import escape_planner as ep  # noqa: E402
from nokosahen_core import motion_sim as ms  # noqa: E402

RES = 0.05


def make_grid(w, h, fill=0):
    return [[fill] * w for _ in range(h)]


def to_info(rows):
    h, w = len(rows), len(rows[0])
    flat = [c for r in rows for c in r]
    return ep.GridInfo(flat, w, h, RES, 0.0, 0.0)


def put_wall(rows, x0, y0, x1, y1):
    """セル範囲[x0,x1]x[y0,y1]を致死(100)に。"""
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            rows[y][x] = 100


def wall_dist(grid, x, y):
    best = 1e9
    for cy in range(grid.height):
        for cx in range(grid.width):
            if ep._is_obstacle(grid.cost(cx, cy)):
                px, py = grid.center_of(cx, cy)
                best = min(best, math.hypot(px - x, py - y))
    return best


class WaypointTests(unittest.TestCase):
    def setUp(self):
        # 6m x 6m。下側(y小)の壁際。壁はy<=1セル(0.1m)まで、機体はy=0.4m。
        self.rows = make_grid(120, 120)
        put_wall(self.rows, 0, 0, 119, 1)
        self.grid = to_info(self.rows)
        self.robot = (3.0, 0.4)

    def test_found_and_far_from_wall(self):
        wp = ep.find_escape_waypoint(self.grid, self.robot, (3.0, 5.0))
        self.assertIsNotNone(wp)
        self.assertLessEqual(math.hypot(wp[0] - self.robot[0], wp[1] - self.robot[1]), 1.0 + 1e-9)
        # 壁(y<0.1m)から、機体端で25cm以上 => 中心は0.1+0.26+0.25=0.61m以上
        self.assertGreaterEqual(wall_dist(self.grid, *wp), ep.BODY_HALF_M + 0.25 - 1e-6)

    def test_bias_toward_goal(self):
        left = ep.find_escape_waypoint(self.grid, self.robot, (0.5, 3.0))
        right = ep.find_escape_waypoint(self.grid, self.robot, (5.5, 3.0))
        up = ep.find_escape_waypoint(self.grid, self.robot, (3.0, 5.5))
        self.assertLess(left[0], self.robot[0])
        self.assertGreater(right[0], self.robot[0])
        self.assertLess(left[0], up[0])
        self.assertLess(up[0], right[0])

    def test_never_backwards_from_goal(self):
        for gx, gy in ((0.5, 3.0), (5.5, 3.0), (3.0, 5.5)):
            wp = ep.find_escape_waypoint(self.grid, self.robot, (gx, gy))
            dx, dy = gx - self.robot[0], gy - self.robot[1]
            self.assertGreaterEqual((wp[0] - self.robot[0]) * dx + (wp[1] - self.robot[1]) * dy, 0.0)

    def test_without_goal_nearest(self):
        wp = ep.find_escape_waypoint(self.grid, self.robot, None)
        self.assertIsNotNone(wp)
        self.assertLessEqual(math.hypot(wp[0] - self.robot[0], wp[1] - self.robot[1]), 0.8)

    def test_high_cost_cells_excluded(self):
        rows = make_grid(120, 120)
        put_wall(rows, 0, 0, 119, 1)
        for y in range(2, 120):
            for x in range(120):
                rows[y][x] = 60  # 全面が高コスト(>50)
        self.assertIsNone(ep.find_escape_waypoint(to_info(rows), self.robot, (3.0, 5.0)))

    def test_u_shape_pocket(self):
        # コの字(幅0.7m、奥行き深い)の中の機体。出口側(上)が開いている。中継点は壁から25cm以上、かつ見つかるなら出口方向。
        rows = make_grid(120, 120)
        put_wall(rows, 50, 20, 51, 100)   # 左の壁 x=2.5..2.6
        put_wall(rows, 66, 20, 67, 100)   # 右の壁 x=3.3..3.4  (内幅 0.7m)
        put_wall(rows, 50, 20, 67, 21)    # 底
        grid = to_info(rows)
        wp = ep.find_escape_waypoint(grid, (2.95, 1.3), (2.95, 5.0))
        # 幅0.7mでは壁から機体端25cm(=中心から0.51m)を満たせない
        self.assertIsNone(wp)

    def test_u_shape_wide(self):
        rows = make_grid(120, 120)
        put_wall(rows, 40, 20, 41, 100)   # x=2.0..2.1
        put_wall(rows, 80, 20, 81, 100)   # x=4.0..4.1 (内幅 1.9m)
        put_wall(rows, 40, 20, 81, 21)
        grid = to_info(rows)
        robot = (3.05, 1.3)
        wp = ep.find_escape_waypoint(grid, robot, (3.05, 5.0))
        self.assertIsNotNone(wp)
        self.assertGreaterEqual(wall_dist(grid, *wp), ep.BODY_HALF_M + 0.25 - 1e-6)
        self.assertGreater(wp[1], robot[1] - 1e-9)

    def test_no_free_space(self):
        rows = make_grid(120, 120, 100)
        self.assertIsNone(ep.find_escape_waypoint(to_info(rows), (3.0, 3.0), (3.0, 5.0)))

    def test_unknown_cells_block(self):
        rows = make_grid(120, 120, -1)
        self.assertIsNone(ep.find_escape_waypoint(to_info(rows), (3.0, 3.0), (3.0, 5.0)))

    def test_wall_between_blocks_line(self):
        # 機体と空きの間に細い壁(目的地側)。壁越しのマスは選ばれない
        rows = make_grid(120, 120)
        put_wall(rows, 0, 0, 119, 1)
        put_wall(rows, 0, 30, 119, 31)  # y=1.5..1.6 の水平な壁
        grid = to_info(rows)
        wp = ep.find_escape_waypoint(grid, (3.0, 0.4), (3.0, 5.0))
        if wp is not None:
            self.assertLess(wp[1], 1.5)


def wall_pts(rng):
    """機体座標のランダムな壁(線分1〜3本+散らばった点)。"""
    pts = []
    for _ in range(rng.randint(1, 3)):
        a = rng.uniform(-math.pi, math.pi)
        d = rng.uniform(0.28, 0.9)
        cx, cy = d * math.cos(a), d * math.sin(a)
        th = rng.uniform(0, math.pi)
        L = rng.uniform(0.3, 2.0)
        n = 40
        for i in range(n):
            t = (i / (n - 1) - 0.5) * L
            pts.append((cx + t * math.cos(th), cy + t * math.sin(th)))
    for _ in range(rng.randint(0, 10)):
        pts.append((rng.uniform(-1, 1), rng.uniform(-1, 1)))
    return pts


class SlideMotionTests(unittest.TestCase):
    def test_new_candidates_present_and_old_first(self):
        old = [(0.0, 0.0)]
        for v in (0.1, -0.1):  # 後退も前進と同じ速さ(2026-10-23)
            for w in (-0.6, -0.3, 0.0, 0.3, 0.6):
                old.append((v, w))
        old += [(0.0, -0.6), (0.0, 0.6)]
        new = ms.default_candidates(0.1, 0.6)
        self.assertEqual(new[:len(old)], old)
        self.assertGreater(len(new), len(old))
        # 小さな旋回の弧(前進・後退)がある
        self.assertTrue(any(v > 0 and 0 < abs(w) < 0.3 * 0.6 + 1e-9 for v, w in new))
        self.assertTrue(any(v < 0 and 0 < abs(w) < 0.3 * 0.6 + 1e-9 for v, w in new))

    def test_picked_never_collides_200_cases(self):
        rng = random.Random(20261022)
        picked_count = 0
        for i in range(300):
            pts = wall_pts(rng)
            flee = rng.choice([None, rng.uniform(-math.pi, math.pi)])
            kw = {}
            if rng.random() < 0.7:
                kw["goal_dir"] = rng.uniform(-math.pi, math.pi)
            if rng.random() < 0.5:
                kw["waypoint_dir"] = rng.uniform(-math.pi, math.pi)
            m = ms.pick_motion(pts, flee, 0.10, 0.6, **kw)
            if m is None:
                continue
            picked_count += 1
            self.assertIsNone(ms.first_collision_time(pts, m[0], m[1], 1.5, 0.05), (i, m))
            # 細かい刻みでも当たらない(刻み0.05で見落とす角のかすめを確認)
            self.assertIsNone(ms.first_collision_time(pts, m[0], m[1], 1.5, 0.01, margin=0.0), (i, m))
        self.assertGreater(picked_count, 50)  # 残りは、最初から外形がめり込む等で「動かない」が正しいケース

    def test_goal_dir_adds_progress(self):
        # 壁なし(遠い点だけ): 目的地の向きへ進む動きが選ばれる
        far = [(3.0, 0.0)]
        fwd = ms.pick_motion(far, None, 0.10, 0.6, goal_dir=0.0)
        back = ms.pick_motion(far, None, 0.10, 0.6, goal_dir=math.pi)
        self.assertGreater(fwd[0], 0)
        self.assertLess(back[0], 0)

    def test_slide_along_wall_toward_goal(self):
        # 左側(y=+0.3)に壁。目的地は前方(+x)。壁に平行に進む前進が選ばれる(後退ではない)
        pts = [(x / 20.0, 0.30 + 0.0) for x in range(-40, 41)]
        m = ms.pick_motion(pts, -math.pi / 2, 0.10, 0.6, goal_dir=0.0)
        self.assertIsNotNone(m)
        self.assertGreater(m[0], 0)
        self.assertIsNone(ms.first_collision_time(pts, m[0], m[1], 1.5, 0.05))

    def test_backward_compatible_signature(self):
        pts = [(0.4, 0.0)]
        self.assertEqual(ms.pick_motion(pts, math.pi, 0.1, 0.6), ms.pick_motion(pts, math.pi, 0.1, 0.6, goal_dir=None))


if __name__ == "__main__":
    unittest.main()
