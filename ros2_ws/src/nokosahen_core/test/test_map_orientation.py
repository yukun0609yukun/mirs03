"""map_orientation.py(回転矩形の走行可能範囲・壁方向推定・表示角の永続化)の単体テスト。

走行可能範囲は安全に関わるため、点の内外判定(yaw 0/30/90/-45、内側/外側/境界)、
旧形式(min/max)の変換、NaN/不正入力の拒否を網羅する。rclpy不要。

実行: python ros2_ws/src/nokosahen_core/test/test_map_orientation.py
"""

import json
import math
import random
import sys
import tempfile
import unittest
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
from nokosahen_core import map_orientation as mo  # noqa: E402


def rect(cx=0.0, cy=0.0, w=4.0, h=2.0, yaw_deg=0.0):
    return mo.make_rect(cx, cy, w, h, math.radians(yaw_deg))


def world(rect_, lx, ly):
    """矩形ローカル座標(lx,ly)の世界座標。"""
    c, s = math.cos(rect_["yaw"]), math.sin(rect_["yaw"])
    return rect_["center_x"] + lx * c - ly * s, rect_["center_y"] + lx * s + ly * c


class PointInRect(unittest.TestCase):
    def test_inside_outside_edge_for_various_yaws(self):
        for yaw in (0.0, 30.0, 90.0, -45.0, 135.0, -170.0):
            r = rect(cx=1.5, cy=-2.0, w=4.0, h=2.0, yaw_deg=yaw)
            with self.subTest(yaw=yaw):
                self.assertTrue(mo.point_in_rect(*world(r, 0, 0), r))
                self.assertTrue(mo.point_in_rect(*world(r, 1.9, 0.9), r))
                self.assertTrue(mo.point_in_rect(*world(r, -1.9, -0.9), r))
                # 辺上・頂点上は内側(以前の<=判定と同じ)
                self.assertTrue(mo.point_in_rect(*world(r, 2.0, 0.0), r))
                self.assertTrue(mo.point_in_rect(*world(r, 2.0, 1.0), r))
                self.assertTrue(mo.point_in_rect(*world(r, -2.0, -1.0), r))
                # すぐ外側
                self.assertFalse(mo.point_in_rect(*world(r, 2.01, 0.0), r))
                self.assertFalse(mo.point_in_rect(*world(r, 0.0, 1.01), r))
                self.assertFalse(mo.point_in_rect(*world(r, -2.01, 0.5), r))
                self.assertFalse(mo.point_in_rect(*world(r, 0.0, -1.01), r))
                self.assertFalse(mo.point_in_rect(*world(r, 10.0, 10.0), r))

    def test_rotated_rect_excludes_bbox_corner(self):
        """30°回転した矩形では、外接矩形の隅は範囲外(軸平行判定との違い)。"""
        r = rect(w=6.0, h=2.0, yaw_deg=30.0)
        min_x, max_x, min_y, max_y = mo.rect_bbox(r)
        px, py = max_x - 0.01, max_y - 0.01  # bboxの右上隅付近
        self.assertTrue(min_x <= px <= max_x and min_y <= py <= max_y)
        self.assertFalse(mo.point_in_rect(px, py, r))

    def test_yaw0_matches_old_axis_aligned_test(self):
        r = mo.rect_from_minmax(-1.0, 3.0, 2.0, 5.0)
        rng = random.Random(1)
        for _ in range(500):
            x, y = rng.uniform(-3, 5), rng.uniform(0, 7)
            old = -1.0 <= x <= 3.0 and 2.0 <= y <= 5.0
            if abs(x + 1) < 1e-6 or abs(x - 3) < 1e-6 or abs(y - 2) < 1e-6 or abs(y - 5) < 1e-6:
                continue
            self.assertEqual(mo.point_in_rect(x, y, r), old)

    def test_non_finite_point_is_outside(self):
        r = rect()
        for p in ((math.nan, 0), (0, math.nan), (math.inf, 0), (0, -math.inf), ("a", 0), (None, 0)):
            self.assertFalse(mo.point_in_rect(p[0], p[1], r))

    def test_margin(self):
        r = rect(w=4.0, h=2.0, yaw_deg=30.0)
        p = world(r, 2.2, 0.0)
        self.assertFalse(mo.point_in_rect(*p, r))
        self.assertTrue(mo.point_in_rect(*p, r, margin=0.3))
        p2 = world(r, 1.9, 0.0)
        self.assertFalse(mo.point_in_rect(*p2, r, margin=-0.2))

    def test_corners_are_on_boundary_and_bbox_contains_them(self):
        r = rect(cx=2, cy=3, w=5, h=3, yaw_deg=-45)
        x0, x1, y0, y1 = mo.rect_bbox(r)
        for (x, y) in mo.rect_corners(r):
            self.assertTrue(mo.point_in_rect(x, y, r))
            self.assertTrue(x0 - 1e-9 <= x <= x1 + 1e-9 and y0 - 1e-9 <= y <= y1 + 1e-9)
        # -45°: bboxの幅 = (w+h)/sqrt2
        self.assertAlmostEqual(x1 - x0, (5 + 3) / math.sqrt(2), places=6)


class OldFormConversion(unittest.TestCase):
    def test_minmax_to_rect(self):
        r = mo.rect_from_minmax(-1.0, 3.0, 2.0, 6.0)
        self.assertEqual((r["center_x"], r["center_y"], r["width"], r["height"], r["yaw"]),
                         (1.0, 4.0, 4.0, 4.0, 0.0))

    def test_old_post_body_accepted(self):
        r = mo.parse_boundary_request({"min_x": 0, "max_x": 2, "min_y": -1, "max_y": 1})
        self.assertEqual(r["yaw"], 0.0)
        self.assertEqual(r["width"], 2.0)

    def test_new_post_body_yaw_rad_and_deg(self):
        a = mo.parse_boundary_request({"center_x": 1, "center_y": 2, "width": 3, "height": 4, "yaw": math.pi / 6})
        b = mo.parse_boundary_request({"center_x": 1, "center_y": 2, "width": 3, "height": 4, "yaw_deg": 30})
        self.assertAlmostEqual(a["yaw"], b["yaw"], places=12)
        c = mo.parse_boundary_request({"center_x": 1, "center_y": 2, "width": 3, "height": 4})
        self.assertEqual(c["yaw"], 0.0)

    def test_state_has_both_forms(self):
        r = rect(cx=1, cy=1, w=4, h=2, yaw_deg=30)
        st = mo.boundary_state(r)
        x0, x1, y0, y1 = mo.rect_bbox(r)
        self.assertEqual((st["min_x"], st["max_x"], st["min_y"], st["max_y"]), (x0, x1, y0, y1))
        self.assertTrue(st["enabled"])
        self.assertAlmostEqual(st["yaw_deg"], 30.0)
        # 状態から矩形を復元できる
        back = mo.rect_from_state(st)
        self.assertAlmostEqual(back["yaw"], r["yaw"])
        self.assertAlmostEqual(back["width"], 4.0)

    def test_old_state_dict_loads_as_yaw0(self):
        st = mo.load_boundary_state({"enabled": True, "min_x": -2, "max_x": 2, "min_y": 0, "max_y": 3})
        self.assertTrue(st["enabled"])
        self.assertEqual(st["yaw"], 0.0)
        self.assertEqual((st["center_x"], st["center_y"], st["width"], st["height"]), (0.0, 1.5, 4.0, 3.0))
        self.assertEqual((st["min_x"], st["max_x"], st["min_y"], st["max_y"]), (-2.0, 2.0, 0.0, 3.0))

    def test_disabled_and_absent_mean_no_restriction(self):
        self.assertFalse(mo.load_boundary_state({})["enabled"])
        self.assertFalse(mo.load_boundary_state(None)["enabled"])
        self.assertFalse(mo.load_boundary_state({"enabled": False, "min_x": 0, "max_x": 1, "min_y": 0, "max_y": 1})["enabled"])
        self.assertTrue(mo.state_allows_point({"enabled": False}, 1e6, 1e6))
        self.assertTrue(mo.state_allows_point({}, 5, 5))
        self.assertTrue(mo.state_allows_point(None, 5, 5))

    def test_enabled_with_corrupt_file_falls_back_to_no_restriction(self):
        self.assertFalse(mo.load_boundary_state({"enabled": True, "center_x": "nan"})["enabled"])
        self.assertFalse(mo.load_boundary_state({"enabled": True, "min_x": 1, "max_x": 0, "min_y": 0, "max_y": 1})["enabled"])

    def test_state_allows_point_rotated(self):
        st = mo.boundary_state(rect(w=6, h=2, yaw_deg=30))
        self.assertTrue(mo.state_allows_point(st, 0, 0))
        x0, x1, y0, y1 = (st["min_x"], st["max_x"], st["min_y"], st["max_y"])
        self.assertFalse(mo.state_allows_point(st, x1 - 0.01, y1 - 0.01))
        # enabledなのに矩形が読めない状態は安全側(拒否)
        self.assertFalse(mo.state_allows_point({"enabled": True}, 0, 0))
        self.assertFalse(mo.state_allows_point(st, math.nan, 0))


class Validation(unittest.TestCase):
    def assertRejects(self, body, kind):
        with self.assertRaises(mo.BoundaryError) as cm:
            mo.parse_boundary_request(body)
        self.assertEqual(cm.exception.kind, kind, body)

    def test_nan_inf_rejected_as_malformed(self):
        nan, inf = float("nan"), float("inf")
        self.assertRejects({"min_x": nan, "max_x": 1, "min_y": 0, "max_y": 1}, "malformed")
        self.assertRejects({"min_x": 0, "max_x": inf, "min_y": 0, "max_y": 1}, "malformed")
        self.assertRejects({"center_x": nan, "center_y": 0, "width": 1, "height": 1}, "malformed")
        self.assertRejects({"center_x": 0, "center_y": 0, "width": inf, "height": 1}, "malformed")
        self.assertRejects({"center_x": 0, "center_y": 0, "width": 1, "height": 1, "yaw": nan}, "malformed")
        self.assertRejects({"center_x": 0, "center_y": 0, "width": 1, "height": 1, "yaw_deg": inf}, "malformed")
        self.assertRejects({"center_x": "NaN", "center_y": 0, "width": 1, "height": 1}, "malformed")

    def test_malformed_rejected(self):
        self.assertRejects(None, "malformed")
        self.assertRejects([1, 2], "malformed")
        self.assertRejects({}, "malformed")
        self.assertRejects({"min_x": 0, "max_x": 1}, "malformed")
        self.assertRejects({"center_x": 0, "center_y": 0, "width": 1}, "malformed")
        self.assertRejects({"center_x": "abc", "center_y": 0, "width": 1, "height": 1}, "malformed")
        self.assertRejects({"center_x": True, "center_y": 0, "width": 1, "height": 1}, "malformed")
        self.assertRejects({"center_x": None, "center_y": 0, "width": 1, "height": 1}, "malformed")
        self.assertRejects({"center_x": 0, "center_y": 0, "width": 1, "height": 1, "yaw": 0, "yaw_deg": 0}, "malformed")
        self.assertRejects({"center_x": 1e12, "center_y": 0, "width": 1, "height": 1}, "malformed")

    def test_geometry_rejected(self):
        self.assertRejects({"min_x": 1, "max_x": 1, "min_y": 0, "max_y": 1}, "geometry")
        self.assertRejects({"min_x": 2, "max_x": 1, "min_y": 0, "max_y": 1}, "geometry")
        self.assertRejects({"center_x": 0, "center_y": 0, "width": 0, "height": 1}, "geometry")
        self.assertRejects({"center_x": 0, "center_y": 0, "width": 1, "height": -1}, "geometry")
        self.assertRejects({"center_x": 0, "center_y": 0, "width": 5000, "height": 1}, "geometry")

    def test_json_nan_token_roundtrip_rejected(self):
        body = json.loads('{"center_x": NaN, "center_y": 0, "width": 1, "height": 1}')
        self.assertRejects(body, "malformed")


class WallAngle(unittest.TestCase):
    @staticmethod
    def make_room(angle_deg, W=240, H=240, half_w=60.0, half_h=40.0, thick=2.0, noise=0, seed=0):
        """中心(W/2,H/2)・half_w x half_h セルの矩形部屋(外壁の太さthick)をangle_degだけ
        回転した占有グリッド。壁の向きは angle_deg (mod 90)。"""
        rng = random.Random(seed)
        cells = [-1] * (W * H)
        a = math.radians(angle_deg)
        c, s = math.cos(a), math.sin(a)
        cx, cy = W / 2.0, H / 2.0
        for r in range(H):
            for col in range(W):
                dx, dy = col + 0.5 - cx, r + 0.5 - cy
                lx = dx * c + dy * s
                ly = -dx * s + dy * c
                inside = abs(lx) <= half_w and abs(ly) <= half_h
                wall = inside and (abs(lx) > half_w - thick or abs(ly) > half_h - thick)
                if wall:
                    cells[r * W + col] = 100
                elif inside:
                    cells[r * W + col] = 0
        for _ in range(noise):
            cells[rng.randrange(W * H)] = 100
        return W, H, cells

    @staticmethod
    def diff_mod90(a, b):
        return abs((a - b + 45.0) % 90.0 - 45.0)

    def test_synthetic_rotated_rectangles(self):
        for ang in (0.0, 10.0, 22.5, 33.0, 44.0, -17.0, -38.5, 61.0, 90.0, 123.0):
            W, H, cells = self.make_room(ang)
            est = mo.estimate_wall_angle_deg(W, H, cells)
            with self.subTest(angle=ang):
                self.assertIsNotNone(est)
                self.assertLess(self.diff_mod90(est["angle_deg"], ang), 1.0, est)
                self.assertGreater(est["confidence"], 0.4)
                self.assertTrue(-45.0 < est["angle_deg"] <= 45.0)

    def test_with_clutter_noise(self):
        W, H, cells = self.make_room(27.0, noise=300, seed=3)
        est = mo.estimate_wall_angle_deg(W, H, cells)
        self.assertLess(self.diff_mod90(est["angle_deg"], 27.0), 2.5, est)

    def test_view_angle_makes_wall_horizontal(self):
        W, H, cells = self.make_room(30.0)
        est = mo.estimate_wall_angle_deg(W, H, cells)
        view = mo.view_angle_from_wall_deg(est["angle_deg"])
        # 壁方向(30°)を表示で回すと水平(0°, mod 90)
        self.assertLess(self.diff_mod90(30.0 + view, 0.0), 2.0)

    def test_too_few_cells_returns_none(self):
        self.assertIsNone(mo.estimate_wall_angle_deg(50, 50, [-1] * 2500))
        self.assertIsNone(mo.estimate_wall_angle_deg(2, 2, [0] * 4))
        self.assertIsNone(mo.estimate_wall_angle_deg(10, 10, [0] * 5))

    def test_two_point_angle(self):
        self.assertAlmostEqual(mo.view_angle_from_two_points(0, 0, 1, 0), 0.0)
        self.assertAlmostEqual(mo.view_angle_from_two_points(0, 0, 1, 1), -45.0)
        self.assertAlmostEqual(mo.view_angle_from_two_points(1, 1, 0, 0), -45.0)  # 向きは無関係
        self.assertAlmostEqual(mo.view_angle_from_two_points(0, 0, 0, 1), 90.0)
        a = mo.view_angle_from_two_points(0, 0, math.cos(math.radians(20)), math.sin(math.radians(20)))
        self.assertAlmostEqual(a, -20.0)
        with self.assertRaises(ValueError):
            mo.view_angle_from_two_points(1, 1, 1, 1)
        with self.assertRaises(ValueError):
            mo.view_angle_from_two_points(math.nan, 0, 1, 1)


class ViewStore(unittest.TestCase):
    def test_session_and_per_map_roundtrip(self):
        with tempfile.TemporaryDirectory() as d:
            maps = Path(d) / "maps"
            maps.mkdir()
            sess = Path(d) / "sess.json"
            st = mo.MapViewStore(str(maps), str(sess))
            self.assertEqual(st.status()["angle_deg"], 0.0)
            self.assertAlmostEqual(st.set_session_angle(190.0), -170.0)
            self.assertAlmostEqual(st.save_for_map("room1"), -170.0)
            self.assertTrue((maps / "room1.view.json").exists())
            # セッションは別インスタンス(再起動)でも復元
            st2 = mo.MapViewStore(str(maps), str(sess))
            self.assertAlmostEqual(st2.status()["angle_deg"], -170.0)
            st2.set_session_angle(12.5)
            # 読み込みで保存済みの向きに復元
            self.assertAlmostEqual(st2.on_map_loaded("room1"), -170.0)
            self.assertAlmostEqual(st2.status()["angle_deg"], -170.0)
            self.assertEqual(st2.status()["loaded_map"], "room1")
            # 保存済み角度が無い地図は現在値を維持
            st2.set_session_angle(33.0)
            self.assertIsNone(st2.on_map_loaded("nope"))
            self.assertAlmostEqual(st2.status()["angle_deg"], 33.0)

    def test_rejects_bad_values_and_names(self):
        with tempfile.TemporaryDirectory() as d:
            st = mo.MapViewStore(d, str(Path(d) / "s.json"))
            for bad in (math.nan, math.inf, None, True, "x"):
                with self.assertRaises((ValueError, TypeError)):
                    st.set_session_angle(bad)
            for name in ("", "../evil", "a/b", "a b", "x" * 65):
                with self.assertRaises(ValueError):
                    st.save_for_map(name)
            self.assertIsNone(st.read_map_angle("../etc/passwd"))

    def test_corrupt_files_ignored(self):
        with tempfile.TemporaryDirectory() as d:
            sess = Path(d) / "s.json"
            sess.write_text("{broken", encoding="utf-8")
            (Path(d) / "m.view.json").write_text('{"angle_deg": NaN}', encoding="utf-8")
            st = mo.MapViewStore(d, str(sess))
            self.assertEqual(st.status()["angle_deg"], 0.0)
            self.assertIsNone(st.read_map_angle("m"))


if __name__ == "__main__":
    unittest.main()
