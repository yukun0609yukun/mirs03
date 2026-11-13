"""patrol_planner(壁から機体端で余白を取る可走領域・巡回経路・目的地のスナップ)のテスト。ROS不要。

実行: python3 -I ros2_ws/src/nokosahen_core/test/test_patrol_planner.py
"""

import math
import sys
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR.parent))

from nokosahen_core import patrol_planner as pp  # noqa: E402

RES = 0.05
HALF = pp.ROBOT_HALF_WIDTH_M
MARGIN = 0.30
REQ = HALF + MARGIN   # 0.56


def make_grid(w, h, walls=None, free=None, origin=(0.0, 0.0)):
    """w x h セル。free(c, r)がTrueなら自由、そうでなければ占有。wallsは追加の占有矩形[(c0,r0,c1,r1)]。"""
    data = [0] * (w * h)
    for r in range(h):
        for c in range(w):
            if free is not None and not free(c, r):
                data[r * w + c] = 100
    for (c0, r0, c1, r1) in walls or []:
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                data[r * w + c] = 100
    return pp.Grid(w, h, RES, origin[0], origin[1], data)


def room(w=100, h=80, thick=2):
    """外周に壁がある部屋(5m x 4m)。"""
    return make_grid(w, h, free=lambda c, r: thick <= c < w - thick and thick <= r < h - thick)


def dist_to_wall(grid, x, y):
    """点から最寄りの占有セル中心までの距離(総当たり、テスト用)。"""
    best = 1e9
    for r in range(grid.height):
        for c in range(grid.width):
            if grid.data[r * grid.width + c] >= pp.OCCUPIED_MIN:
                px, py = grid.center_of(c, r)
                best = min(best, math.hypot(px - x, py - y))
    return best


def edge_clear(grid, x, y):
    """機体端(中心-半幅)から壁表面までの距離(壁表面=占有セル中心から半セル手前)。"""
    return dist_to_wall(grid, x, y) - RES / 2.0 - HALF


def rect_of(cx, cy, w, h, yaw=0.0):
    return {"center_x": cx, "center_y": cy, "width": w, "height": h, "yaw": yaw}


def coverage(area, path, radius):
    """可走セルのうち、経路(点列を結ぶ線分)から radius 以内にあるものの割合。"""
    cells = area.cells()
    segs = list(zip(path, path[1:])) if len(path) > 1 else [(path[0], path[0])]

    def d(px, py, a, b):
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
        return math.hypot(px - (ax + t * dx), py - (ay + t * dy))

    hit = 0
    for c, r in cells:
        px, py = area.grid.center_of(c, r)
        if any(d(px, py, a, b) <= radius for a, b in segs):
            hit += 1
    return hit / max(1, len(cells))


class DrivableAreaTest(unittest.TestCase):
    def test_cells_keep_margin_from_wall(self):
        g = room()
        area = pp.compute_drivable_area(g, None)
        self.assertGreater(area.count(), 0)
        for c, r in area.cells()[::7]:
            x, y = g.center_of(c, r)
            self.assertGreaterEqual(edge_clear(g, x, y), MARGIN - 1e-9)

    def test_cells_just_inside_are_excluded(self):
        g = room()
        area = pp.compute_drivable_area(g, None)
        # 壁(内側の面は x=0.10 m)から機体端で0.2 mの場所 = 中心 x≈0.10+0.26+0.20=0.56 手前
        c, r = g.cell_of(0.10 + HALF + 0.20, 2.0)
        self.assertFalse(area.cell_ok(c, r))
        c, r = g.cell_of(0.10 + HALF + 0.40, 2.0)
        self.assertTrue(area.cell_ok(c, r))

    def test_unknown_and_outside_are_not_drivable(self):
        g = room()
        for r in range(30, 50):
            for c in range(40, 60):
                g.data[r * g.width + c] = -1
        area = pp.compute_drivable_area(g, None)
        self.assertFalse(area.cell_ok(50, 40))
        self.assertFalse(area.cell_ok(-1, 5))
        self.assertFalse(area.cell_ok(9999, 5))

    def test_rect_limits_area(self):
        g = room()
        rect = rect_of(2.5, 2.0, 2.0, 1.0)
        area = pp.compute_drivable_area(g, rect)
        for c, r in area.cells():
            x, y = g.center_of(c, r)
            self.assertTrue(abs(x - 2.5) <= 1.0 + 1e-6 and abs(y - 2.0) <= 0.5 + 1e-6)
        self.assertGreater(area.count(), 0)

    def test_narrow_corridor_excluded(self):
        # 幅 0.9 m (18セル)の廊下: 必要な幅は 2*0.56=1.12 m 以上 -> 可走セルなし
        g = make_grid(120, 60, free=lambda c, r: 20 <= r < 38 and 2 <= c < 118)
        area = pp.compute_drivable_area(g, None)
        self.assertEqual(area.count(), 0)
        # 幅 1.5 m (30セル)なら中央だけ可走
        g = make_grid(120, 60, free=lambda c, r: 15 <= r < 45 and 2 <= c < 118)
        area = pp.compute_drivable_area(g, None)
        self.assertGreater(area.count(), 0)
        rows = {r for _, r in area.cells()}
        self.assertTrue(all(15 + 11 <= r <= 44 - 11 for r in rows))

    def test_bad_grid(self):
        self.assertIsNone(pp.compute_drivable_area(None))
        self.assertIsNone(pp.compute_drivable_area(pp.Grid(0, 0, 0.05, 0, 0, [])))
        self.assertIsNone(pp.compute_drivable_area(pp.Grid(10, 10, 0.05, 0, 0, [0] * 5)))


class PatrolPathTest(unittest.TestCase):
    def check_path(self, g, rect, spacing=0.6, min_cov=0.97):
        area = pp.compute_drivable_area(g, rect)
        path = pp.plan_patrol_path(g, rect, MARGIN, HALF, spacing)
        self.assertTrue(path)
        for x, y in path:
            self.assertTrue(area.point_ok(x, y), (x, y))
            self.assertGreaterEqual(edge_clear(g, x, y), MARGIN - 1e-9, (x, y))
            if rect is not None:
                self.assertTrue(pp._in_rect(x, y, rect), (x, y))
        # 区間の中も壁から離れている(区間は可走セルだけを連ねる)
        segs = pp.plan_patrol_segments(area, spacing)
        for s in segs:
            n = max(1, int(s.length() / 0.05))
            for i in range(n + 1):
                t = i / n
                x = s.start[0] + (s.end[0] - s.start[0]) * t
                y = s.start[1] + (s.end[1] - s.start[1]) * t
                self.assertGreaterEqual(edge_clear(g, x, y), MARGIN - 0.03, (x, y))
        cov = coverage(area, path, spacing / 2.0 + RES)
        self.assertGreaterEqual(cov, min_cov, cov)
        return area, path, segs

    def test_room(self):
        g = room()
        area, path, segs = self.check_path(g, rect_of(2.5, 2.0, 5.0, 4.0))
        # 往復: 隣のレーンは逆向き
        lanes = {}
        for s in segs:
            lanes.setdefault(s.lane, s)
        ks = sorted(lanes)
        self.assertGreaterEqual(len(ks), 3)
        d0 = lanes[ks[0]].end[0] - lanes[ks[0]].start[0]
        d1 = lanes[ks[1]].end[0] - lanes[ks[1]].start[0]
        self.assertLess(d0 * d1, 0)

    def test_room_without_rect(self):
        self.check_path(make_grid(60, 50, free=lambda c, r: 2 <= c < 58 and 2 <= r < 48), None)

    def test_lanes_inside_rect(self):
        g = room()
        rect = rect_of(2.5, 2.0, 3.0, 1.6)
        area, path, _ = self.check_path(g, rect)
        for x, y in path:
            self.assertTrue(abs(x - 2.5) <= 1.5 + 1e-6 and abs(y - 2.0) <= 0.8 + 1e-6)

    def test_rotated_rect(self):
        g = room()
        self.check_path(g, rect_of(2.5, 2.0, 3.0, 1.6, math.radians(30)), min_cov=0.95)

    def test_u_shape(self):
        # コの字: 外周の壁 + 中央に下から突き出す壁(上が開いている)
        g = room(120, 90)
        for r in range(2, 60):
            for c in range(40, 80):
                g.data[r * g.width + c] = 100
        self.check_path(g, rect_of(3.0, 2.25, 5.8, 4.3))

    def test_l_shape(self):
        free = lambda c, r: (2 <= c < 118 and 2 <= r < 40) or (2 <= c < 50 and 2 <= r < 88)  # noqa: E731
        g = make_grid(120, 90, free=free)
        self.check_path(g, rect_of(3.0, 2.25, 5.8, 4.3))

    def test_box_obstacle(self):
        g = room(120, 90, 2)
        for r in range(30, 50):
            for c in range(50, 70):
                g.data[r * g.width + c] = 100
        area, path, _ = self.check_path(g, rect_of(3.0, 2.25, 5.8, 4.3))
        # 箱のすぐ脇(機体端で30 cm未満)は可走でも経路でもない
        for x, y in path:
            self.assertGreaterEqual(edge_clear(g, x, y), MARGIN - 1e-9)

    def test_gap_between_box_and_wall_too_narrow(self):
        # 箱と右の壁の間が 0.9 m: 通れない(必要 1.12 m)ので経路に入らない
        g = room(120, 90, 2)
        for r in range(20, 70):
            for c in range(60, 112):
                g.data[r * g.width + c] = 100
        area, path, _ = self.check_path(g, rect_of(3.0, 2.25, 5.8, 4.3))
        for x, y in path:
            self.assertFalse(5.6 <= x <= 5.9 and 1.0 <= y <= 3.5)

    def test_too_small_room_has_no_path(self):
        g = make_grid(24, 24, free=lambda c, r: 2 <= c < 22 and 2 <= r < 22)   # 1.0 m 四方
        self.assertEqual(pp.plan_patrol_path(g, None), [])

    def test_no_grid_or_disabled(self):
        self.assertEqual(pp.plan_patrol_path(None, None), [])
        self.assertEqual(pp.plan_patrol_path(room(), None, margin_m=0.0), [])


class SnapTest(unittest.TestCase):
    def test_ok_stays(self):
        g = room()
        r = pp.snap_to_drivable(g, 2.5, 2.0)
        self.assertEqual(r["status"], "ok")
        self.assertEqual((r["x"], r["y"]), (2.5, 2.0))

    def test_snaps_away_from_wall(self):
        g = room()
        r = pp.snap_to_drivable(g, 0.3, 2.0)   # 壁の面(x=0.10)から機体端で-0.06 m(食い込み)
        self.assertEqual(r["status"], "snapped")
        self.assertGreater(r["shift_m"], 0.0)
        self.assertGreaterEqual(edge_clear(g, r["x"], r["y"]), MARGIN - 1e-9)

    def test_snap_onto_wall_cell(self):
        g = room()
        r = pp.snap_to_drivable(g, 0.05, 2.0)   # 壁の上
        self.assertEqual(r["status"], "snapped")
        self.assertGreaterEqual(edge_clear(g, r["x"], r["y"]), MARGIN - 1e-9)

    def test_snap_inside_rect(self):
        g = room()
        rect = rect_of(3.5, 2.0, 3.0, 3.0)   # x in [2.0, 5.0]
        r = pp.snap_to_drivable(g, 1.5, 2.0, rect=rect)   # 部屋の中だが範囲の外
        self.assertEqual(r["status"], "snapped")
        self.assertTrue(pp._in_rect(r["x"], r["y"], rect))
        self.assertGreaterEqual(edge_clear(g, r["x"], r["y"]), MARGIN - 1e-9)

    def test_no_drivable_nearby(self):
        # 幅 0.9 m の廊下の中: 近くに可走セルなし
        g = make_grid(100, 80, free=lambda c, r: 31 <= r < 49 and 2 <= c < 98)
        r = pp.snap_to_drivable(g, 2.5, 2.0)
        self.assertEqual(r["status"], "no_drivable")
        self.assertEqual((r["x"], r["y"]), (2.5, 2.0))

    def test_without_grid_or_margin_unchanged(self):
        for r in (pp.snap_to_drivable(None, 1.0, 2.0),
                  pp.snap_to_drivable(room(), 0.3, 2.0, margin_m=0.0),
                  pp.snap_to_drivable(room(), 0.3, 2.0, margin_m=-1.0),
                  pp.snap_to_drivable(pp.Grid(0, 0, 0.05, 0, 0, []), 0.3, 2.0),
                  pp.snap_to_drivable(room(), float("nan"), 2.0)):
            self.assertEqual(r["status"], "no_map")
        r = pp.snap_to_drivable(room(), 0.3, 2.0, margin_m=0.0)
        self.assertEqual((r["x"], r["y"], r["shift_m"]), (0.3, 2.0, 0.0))

    def test_grid_from_msg(self):
        from types import SimpleNamespace as NS
        msg = NS(info=NS(width=4, height=3, resolution=0.05, origin=NS(position=NS(x=1.0, y=2.0))), data=[0] * 12)
        g = pp.grid_from_msg(msg)
        self.assertEqual((g.width, g.height, g.origin_x, g.origin_y), (4, 3, 1.0, 2.0))
        self.assertIsNone(pp.grid_from_msg(NS()))
        msg.data = [0] * 3
        self.assertIsNone(pp.grid_from_msg(msg))


def fake_map_msg(g):
    from types import SimpleNamespace as NS
    return NS(info=NS(width=g.width, height=g.height, resolution=g.resolution,
                      origin=NS(position=NS(x=g.origin_x, y=g.origin_y))), data=g.data)


class IntegrationTest(unittest.TestCase):
    """MapGoalController.check_goal(クリック目的地)への組み込み(frontier_explorer側はtest_explore_approach.py)。"""

    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(TEST_DIR))
        import test_map_pins as tmp   # ROSスタブの注入とdashboard_commonのimportを流用
        cls.tmp = tmp
        cls.dc = tmp.dc

    def make_ctl(self, margin=None):
        ctl = self.dc.MapGoalController(self.tmp.FakeNode(), robot_pose=None)
        if margin is not None:
            ctl._goal_wall_margin_cache = margin
        return ctl

    def test_click_goal_snapped_away_from_wall(self):
        g = room()
        ctl = self.make_ctl()
        ctl.latest_map = fake_map_msg(g)
        # 壁の面から機体端で約10 cm: 従来のsnap(機体端4 cm)ではokだが、30 cm未満なので寄せる
        x, y = 0.10 + HALF + 0.10, 2.0
        r = ctl.check_goal(x, y)
        self.assertEqual(r["status"], "snapped")
        self.assertGreaterEqual(edge_clear(g, r["x"], r["y"]), MARGIN - 1e-9)
        self.assertAlmostEqual(r["shift_m"], math.hypot(r["x"] - x, r["y"] - y))

    def test_click_goal_disabled_by_param(self):
        g = room()
        ctl = self.make_ctl(0.0)
        ctl.latest_map = fake_map_msg(g)
        r = ctl.check_goal(0.10 + HALF + 0.10, 2.0)
        self.assertEqual(r["status"], "ok")   # 従来どおり
        ctl2 = self.make_ctl()
        ctl2.latest_map = fake_map_msg(g)
        r = ctl2.check_goal(0.10 + HALF + 0.10, 2.0, wall_margin=False)
        self.assertEqual(r["status"], "ok")

    def test_click_goal_no_map_unchanged(self):
        ctl = self.make_ctl()
        self.assertEqual(ctl.check_goal(1.0, 1.0)["status"], "no_map")

    def test_click_goal_center_unchanged_and_rect(self):
        g = room()
        ctl = self.make_ctl()
        ctl.latest_map = fake_map_msg(g)
        self.assertEqual(ctl.check_goal(2.5, 2.0)["status"], "ok")
        rect = rect_of(3.5, 2.0, 3.0, 3.0)
        r = ctl.check_goal(1.9, 2.0, rect=rect)   # 範囲の外 -> 範囲の中へ
        self.assertEqual(r["status"], "snapped")
        self.assertTrue(pp._in_rect(r["x"], r["y"], rect))


if __name__ == "__main__":
    unittest.main()
