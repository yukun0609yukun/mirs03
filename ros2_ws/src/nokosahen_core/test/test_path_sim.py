"""経路のシミュレーション(path_sim.py)のテスト(2026-10-25)。ROS不要。
実行: python3 -I ros2_ws/src/nokosahen_core/test/test_path_sim.py
"""
import math
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core import path_sim as ps  # noqa: E402

H = ps.BODY_HALF_M
NAN = float("nan")
INF = float("inf")


def line(x0, y0, x1, y1, step=0.1):
    n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]


def box(cx, cy, w, d, step=0.03):
    pts = []
    x = cx - w / 2
    while x <= cx + w / 2 + 1e-9:
        pts += [(x, cy - d / 2), (x, cy + d / 2)]
        x += step
    y = cy - d / 2
    while y <= cy + d / 2 + 1e-9:
        pts += [(cx - w / 2, y), (cx + w / 2, y)]
        y += step
    return pts


POSE = (0.0, 0.0, 0.0)


class FirstCollisionTests(unittest.TestCase):
    def test_straight_corridor_is_clear(self):
        # 幅1.2 mの通路(壁は y=+-0.6)
        walls = line(-1, 0.6, 6, 0.6, 0.03) + line(-1, -0.6, 6, -0.6, 0.03)
        self.assertIsNone(ps.first_collision_on_path(line(0, 0, 5, 0), POSE, walls, 0.2))

    def test_box_on_path_collides_at_the_right_distance(self):
        pts = box(2.0, 0.0, 0.4, 0.4)
        d = ps.first_collision_on_path(line(0, 0, 5, 0), POSE, pts, 0.2)
        self.assertIsNotNone(d)
        # 箱の手前面は x=1.8。機体の前面(中心+0.26+margin)が着く中心位置は 1.52 付近
        self.assertAlmostEqual(d, 1.8 - H - 0.02, delta=0.07)

    def test_box_beside_the_path(self):
        self.assertIsNone(ps.first_collision_on_path(line(0, 0, 5, 0), POSE, box(2.0, 1.0, 0.4, 0.4), 0.2))
        # 箱の端が経路から0.3 m(機体側面0.26+0.02より外) -> 通れる
        self.assertIsNone(ps.first_collision_on_path(line(0, 0, 5, 0), POSE, box(2.0, 0.5, 0.4, 0.4), 0.2))
        # 箱の端が0.25 m(< 0.28) -> 当たる
        self.assertIsNotNone(ps.first_collision_on_path(line(0, 0, 5, 0), POSE, box(2.0, 0.45, 0.4, 0.4), 0.2))

    def test_box_beyond_lookahead_is_ignored(self):
        pts = box(4.6, 0.0, 0.4, 0.4)
        self.assertIsNone(ps.first_collision_on_path(line(0, 0, 6, 0), POSE, pts, 0.2, lookahead_m=4.0))
        self.assertIsNotNone(ps.first_collision_on_path(line(0, 0, 6, 0), POSE, pts, 0.2, lookahead_m=5.0))

    def test_initial_turn_hits_wall_so_path_is_rejected(self):
        # 前0.30 mに壁。真後ろへ向かう経路は、先頭の180度旋回で角が壁に当たる -> 拒否(0.0)
        wall = line(0.30, -0.6, 0.30, 0.6, 0.03)
        self.assertEqual(ps.first_collision_on_path(line(0, 0, -2, 0), POSE, wall, 0.2), 0.0)

    def test_turn_in_open_space_or_far_wall_is_ok(self):
        path = line(0, 0, -2, 0)
        self.assertIsNone(ps.first_collision_on_path(path, POSE, [], 0.2))
        self.assertIsNone(ps.first_collision_on_path(path, POSE, line(1.0, -0.6, 1.0, 0.6, 0.03), 0.2))

    def test_path_corner_checks_in_place_turn(self):
        # L字の経路。角(2,0)の外側(左折の内側前方)に点があると、角でのその場旋回で外形が当たる
        path = line(0, 0, 2, 0) + line(2, 0, 2, 2)[1:]
        self.assertIsNone(ps.first_collision_on_path(path, POSE, [], 0.2))
        d = ps.first_collision_on_path(path, POSE, [(2.2, 0.31)], 0.2)
        self.assertIsNotNone(d)

    def test_robot_not_at_path_start_is_connected(self):
        path = line(1, 0, 3, 0)
        self.assertIsNotNone(ps.first_collision_on_path(path, POSE, box(0.5, 0.0, 0.2, 0.2), 0.2))

    def test_empty_and_invalid_inputs(self):
        self.assertIsNone(ps.first_collision_on_path(line(0, 0, 2, 0), POSE, [], 0.2))
        self.assertEqual(ps.first_collision_on_path([], POSE, [], 0.2), 0.0)       # 経路が無い -> 越せない扱い
        self.assertEqual(ps.first_collision_on_path(line(0, 0, 2, 0), (NAN, 0, 0), [], 0.2), 0.0)
        self.assertEqual(ps.first_collision_on_path(line(0, 0, 2, 0), (0, INF, 0), [], 0.2), 0.0)
        self.assertEqual(ps.first_collision_on_path(line(0, 0, 2, 0), None, [], 0.2), 0.0)
        # 点のNaN/Infは無視(他の点は効く)
        pts = [(NAN, 0.0), (INF, 1.0), (1.0, -INF)] + box(2.0, 0.0, 0.4, 0.4)
        self.assertIsNotNone(ps.first_collision_on_path(line(0, 0, 5, 0), POSE, pts, 0.2))
        self.assertIsNone(ps.first_collision_on_path(line(0, 0, 5, 0), POSE, [(NAN, NAN), (INF, INF)], 0.2))
        # 経路のNaNは捨てる
        self.assertIsNone(ps.first_collision_on_path([(0, 0), (NAN, NAN), (2, 0)], POSE, [], 0.2))
        # ゼロ長の経路
        self.assertIsNone(ps.first_collision_on_path([(0, 0)], POSE, [], 0.2))
        self.assertEqual(ps.first_collision_on_path([(0, 0)], POSE, [(0.1, 0.0)], 0.2), 0.0)

    def test_point_already_inside_body_is_collision(self):
        self.assertEqual(ps.first_collision_on_path(line(0, 0, 2, 0), POSE, [(0.1, 0.1)], 0.2), 0.0)

    def test_random_verified_paths_do_not_collide_on_independent_recheck(self):
        rnd = random.Random(7)
        ok = 0
        for _ in range(150):
            pts = [(rnd.uniform(0.3, 4.5), rnd.uniform(-1.2, 1.2)) for _ in range(rnd.randint(0, 25))]
            gx, gy = rnd.uniform(1.0, 4.0), rnd.uniform(-1.0, 1.0)
            hd = math.atan2(gy, gx)
            path = line(0, 0, gx, gy, 0.1)
            pose = (0.0, 0.0, hd)   # 最初から経路の向き(旋回は無し)。旋回側は別のテスト
            if ps.first_collision_on_path(path, pose, pts, 0.2, lookahead_m=10.0) is not None:
                continue
            ok += 1
            # 独立な再確認: 直線を0.01 m刻みで、進行方向の向きの正方形(同じ余白)に点が入らない
            c, s = math.cos(hd), math.sin(hd)
            n = int(math.hypot(gx, gy) / 0.01)
            for i in range(n + 1):
                x, y = gx * i / n, gy * i / n
                for px, py in pts:
                    dx, dy = px - x, py - y
                    if abs(c * dx + s * dy) <= H + 0.02 and abs(-s * dx + c * dy) <= H + 0.02:
                        self.fail((pts, (x, y)))
            # 同じ入力を再実行しても同じ結果
            self.assertIsNone(ps.first_collision_on_path(path, pose, pts, 0.2, lookahead_m=10.0))
        self.assertGreater(ok, 10)

    def test_random_collisions_are_real(self):
        # 衝突を返したとき、その位置の外形に実際に点が入っている(誤検出なし)
        rnd = random.Random(11)
        hits = 0
        for _ in range(100):
            pts = [(rnd.uniform(0.3, 4.0), rnd.uniform(-1.0, 1.0)) for _ in range(rnd.randint(1, 15))]
            path = line(0, 0, 4, 0, 0.1)
            d = ps.first_collision_on_path(path, POSE, pts, 0.2)
            if d is None:
                continue
            hits += 1
            self.assertTrue(any(abs(px - d) <= H + 0.02 + 1e-6 and abs(py) <= H + 0.02 + 1e-6 for px, py in pts), (d, pts))
        self.assertGreater(hits, 10)


class InitialRotationTests(unittest.TestCase):
    def test_open_space_ok(self):
        self.assertTrue(ps.initial_rotation_ok(POSE, math.pi, []))

    def test_corner_hits_wall_when_rotating(self):
        wall = line(0.30, -0.6, 0.30, 0.6, 0.03)    # 前0.30 m: 外接半径0.368 > 0.30。45度回ると角が壁にめり込む
        self.assertFalse(ps.initial_rotation_ok(POSE, math.radians(45), wall))

    def test_wall_outside_circumscribed_circle_ok(self):
        self.assertTrue(ps.initial_rotation_ok(POSE, math.pi, line(0.45, -0.6, 0.45, 0.6, 0.03)))

    def test_invalid(self):
        self.assertFalse(ps.initial_rotation_ok((NAN, 0, 0), 1.0, []))
        self.assertFalse(ps.initial_rotation_ok(POSE, INF, []))
        self.assertFalse(ps.initial_rotation_ok(None, 1.0, []))
        self.assertTrue(ps.initial_rotation_ok(POSE, 1.0, [(NAN, NAN)]))


class DetourTests(unittest.TestCase):
    def setUp(self):
        self.obs = box(1.5, 0.0, 0.5, 0.5)

    def test_candidates_outside_obstacle_and_both_sides(self):
        c = ps.detour_waypoints(self.obs, POSE, (4.0, 0.0))
        self.assertEqual(len(c), 6)
        self.assertEqual((len([p for p in c if p[1] > 0]), len([p for p in c if p[1] < 0])), (3, 3))
        for x, y in c:
            self.assertAlmostEqual(x, 1.5, delta=0.05)
            self.assertGreaterEqual(abs(y), 0.25 + H + 0.02 + 0.7 - 1e-6)   # 端 + 半幅 + 余白 + 最小offset

    def test_each_candidate_clears_the_box_in_simulation(self):
        for p in ps.detour_waypoints(self.obs, POSE, (4.0, 0.0)):
            path = line(0, 0, p[0], p[1]) + line(p[0], p[1], 4.0, 0.0)[1:]
            self.assertIsNone(ps.first_collision_on_path(path, POSE, self.obs, 0.2), p)
        # 比較: 箱を通る直線は当たる
        self.assertIsNotNone(ps.first_collision_on_path(line(0, 0, 4, 0), POSE, self.obs, 0.2))

    def test_order_is_nearest_first(self):
        c = ps.detour_waypoints(self.obs, POSE, (4.0, 0.0))
        costs = [math.hypot(x, y) + math.hypot(4.0 - x, y) for x, y in c]
        self.assertEqual(costs, sorted(costs))
        self.assertAlmostEqual(abs(c[0][1]), abs(c[1][1]), places=6)   # 最小offsetの左右が先頭2つ
        self.assertGreater(c[0][1], 0)                                  # 同距離なら左が先

    def test_asymmetric_obstacle_prefers_short_side(self):
        obs = box(1.5, 0.4, 0.5, 1.0)   # y: -0.1..0.9 -> 右へ避けるほうが短い
        c = ps.detour_waypoints(obs, POSE, (4.0, 0.0))
        self.assertLess(c[0][1], 0)

    def test_goal_direction_is_front_even_if_robot_faces_away(self):
        self.assertEqual(len(ps.detour_waypoints(self.obs, (0.0, 0.0, math.pi), (4.0, 0.0))), 6)

    def test_no_blocking_cluster_or_bad_input(self):
        self.assertEqual(ps.detour_waypoints([], POSE, (4.0, 0.0)), [])
        self.assertEqual(ps.detour_waypoints(box(1.5, 1.5, 0.4, 0.4), POSE, (4.0, 0.0)), [])   # 進路の外
        self.assertEqual(ps.detour_waypoints(box(-1.5, 0.0, 0.4, 0.4), POSE, (4.0, 0.0)), [])  # 後ろ
        self.assertEqual(ps.detour_waypoints([(NAN, 0.0), (INF, INF)], POSE, (4.0, 0.0)), [])
        self.assertEqual(ps.detour_waypoints(self.obs, (NAN, 0, 0), (4.0, 0.0)), [])
        self.assertEqual(ps.detour_waypoints(self.obs, POSE, (NAN, 0.0)), [])
        self.assertEqual(ps.detour_waypoints(self.obs, None, (4.0, 0.0)), [])

    def test_two_close_objects_are_one_cluster(self):
        obs = box(1.5, 0.4, 0.3, 0.3) + box(1.5, -0.4, 0.3, 0.3)   # 隙間0.5 m(クラスタの間隔0.35 mでは別)
        c = ps.detour_waypoints(obs, POSE, (4.0, 0.0), offsets=(0.7,))
        self.assertEqual(len(c), 2)
        near = box(1.5, 0.3, 0.3, 0.3) + box(1.5, -0.3, 0.3, 0.3)   # 隙間0.3 m -> 1つの塊
        c2 = ps.detour_waypoints(near, POSE, (4.0, 0.0), offsets=(0.7,))
        self.assertGreater(max(abs(p[1]) for p in c2), 0.45 + H)


class MiscTests(unittest.TestCase):
    def test_decimate_limits_count_and_keeps_far_point(self):
        pts = [(i * 0.01, 0.0) for i in range(1000)] + [(3.0, 0.5)]
        out = ps.decimate_points(pts, 150)
        self.assertLessEqual(len(out), 150)
        self.assertTrue(any(abs(x - 3.0) < 0.3 and abs(y - 0.5) < 0.3 for x, y in out))
        self.assertEqual(ps.decimate_points([(1, 1), (NAN, 0)], 150), [(1.0, 1.0)])

    def test_path_heading(self):
        self.assertAlmostEqual(ps.path_heading(line(0, 0, 0, 3), POSE), math.pi / 2, places=3)
        self.assertIsNone(ps.path_heading([], POSE))
        self.assertIsNone(ps.path_heading([(0.0, 0.0)], POSE))


if __name__ == "__main__":
    unittest.main(verbosity=1)
