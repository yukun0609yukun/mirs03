"""カメラの障害物検知よりLiDARの距離を優先する判定と、向く用の全周の人候補(scan_geometry、rclpy不要)。
2026-11-04: 廊下の壁・扉をカメラが障害物と誤認識して止まり続けた/真横に立つと候補が出なかった、への対策。
実行: python -m pytest -q ros2_ws/src/nokosahen_core/test/test_camera_obstacle_lidar_priority.py
"""
import importlib.util
import math
import sys
import unittest
from pathlib import Path

import numpy as np

# nokosahen_core/__init__ がrclpy等を読む環境でも動くよう、ファイルを直接読み込む。
_PATH = Path(__file__).resolve().parents[1] / "nokosahen_core" / "scan_geometry.py"
_spec = importlib.util.spec_from_file_location("scan_geometry_under_test", _PATH)
sg = importlib.util.module_from_spec(_spec)
sys.modules["scan_geometry_under_test"] = sg
_spec.loader.exec_module(sg)

INC = math.radians(0.5)
ANGLES = -math.pi + np.arange(720) * INC
HALF = math.radians(20.0)


def scan_with(default, spots):
    """全方向 default[m]。spots = [(角度[deg], 距離)] の近傍(±2度)を上書き。"""
    r = np.full(ANGLES.shape, float(default))
    for deg, rng in spots:
        r[np.abs(((ANGLES - math.radians(deg) + math.pi) % (2 * math.pi)) - math.pi) <= math.radians(2)] = rng
    return r


def confirm(r, bearing_deg=0.0, confirm_m=1.0, age=0.1, stale=1.0, rmin=0.15, rmax=12.0, half=HALF):
    return sg.lidar_confirms_obstacle(ANGLES, r, rmin, rmax, math.radians(bearing_deg), confirm_m, half, age, stale)


class LidarPriorityTests(unittest.TestCase):
    def test_close_lidar_confirms(self):
        self.assertTrue(confirm(scan_with(5.0, [(0, 0.8)])))

    def test_far_lidar_ignores_camera_obstacle(self):
        self.assertFalse(confirm(scan_with(5.0, [(0, 3.0)])))

    def test_boundary_distance_is_inclusive(self):
        self.assertTrue(confirm(scan_with(5.0, [(0, 1.0)]), confirm_m=1.0))
        self.assertFalse(confirm(scan_with(5.0, [(0, 1.05)]), confirm_m=1.0))

    def test_unmeasured_direction_is_safe_side(self):
        r = np.full(ANGLES.shape, np.inf)  # その方向を測れていない
        self.assertTrue(confirm(r))
        self.assertTrue(confirm(np.zeros(ANGLES.shape)))  # range_min未満=無効値

    def test_stale_or_missing_scan_is_safe_side(self):
        r = scan_with(5.0, [(0, 3.0)])
        self.assertTrue(confirm(r, age=2.0, stale=1.0))
        self.assertTrue(sg.lidar_confirms_obstacle(None, None, 0.15, 12.0, 0.0, 1.0, HALF, None, 1.0))

    def test_disabled_is_safe_side(self):
        r = scan_with(5.0, [(0, 3.0)])
        self.assertTrue(confirm(r, confirm_m=0.0))
        self.assertTrue(confirm(r, confirm_m=-1.0))

    def test_only_near_the_camera_bearing_counts(self):
        r = scan_with(5.0, [(90, 0.5)])  # 真横は近いが、カメラが見た正面は遠い
        self.assertFalse(confirm(r, bearing_deg=0.0))
        self.assertTrue(confirm(r, bearing_deg=90.0))

    def test_half_width_boundary(self):
        r = scan_with(5.0, [(30, 0.5)])  # 30度ずれた所の近い物
        self.assertFalse(confirm(r, bearing_deg=0.0, half=math.radians(20)))
        self.assertTrue(confirm(r, bearing_deg=0.0, half=math.radians(35)))

    def test_angle_wraps_around_pi(self):
        r = scan_with(5.0, [(180, 0.6)])  # 真後ろ(±π)の近い物
        self.assertTrue(confirm(r, bearing_deg=179.0))
        self.assertTrue(confirm(r, bearing_deg=-179.0))
        self.assertFalse(confirm(r, bearing_deg=0.0))


class WideCandidateTests(unittest.TestCase):
    FRONT_HALF = math.radians(70.0)  # 前方140度
    WIDE = math.radians(360.0)

    def masks(self, ranges, background_excluded=None, max_range=3.0):
        front = (np.abs(ANGLES) <= self.FRONT_HALF) & (ranges >= 0.15) & (ranges <= max_range)
        wide = sg.wide_point_mask(front, ANGLES, ranges, 0.15, max_range, self.FRONT_HALF, self.WIDE, background_excluded)
        return front, wide

    @staticmethod
    def leg_at(deg, rng, width_deg=3.0):
        return [(d, rng) for d in np.arange(deg - width_deg / 2, deg + width_deg / 2 + 1e-9, 0.5)]

    def clusters(self, mask, ranges):
        idx = np.nonzero(mask)[0]
        pts = list(zip(ANGLES[idx].tolist(), ranges[idx].tolist()))
        return sg.cluster_leg_points(pts, 0.10, INC * 3.0, 0.04, 0.35)

    def test_side_leg_outside_front_is_a_candidate_but_not_a_front_cluster(self):
        r = scan_with(8.0, self.leg_at(90, 1.5))  # 真左に脚
        front, wide = self.masks(r)
        self.assertEqual(self.clusters(front, r), [])  # 前方140度だけでは出ない(近接停止・接近はこちら)
        wide_clusters = self.clusters(wide, r)
        self.assertEqual(len(wide_clusters), 1)
        self.assertAlmostEqual(math.degrees(wide_clusters[0][0]), 90.0, delta=2.0)

    def test_rear_leg_is_a_candidate(self):
        r = scan_with(8.0, self.leg_at(160, 1.2))  # 後ろ寄り(±πの継ぎ目をまたぐ位置は、点列が分かれるので別扱い)
        _, wide = self.masks(r)
        self.assertEqual(len(self.clusters(wide, r)), 1)

    def test_front_result_is_unchanged_by_wide_mask(self):
        r = scan_with(8.0, self.leg_at(30, 1.0) + self.leg_at(90, 1.5))
        front, wide = self.masks(r)
        self.assertTrue(np.all(wide[front]))  # 前方の点は全部含まれる
        self.assertEqual(len(self.clusters(front, r)), 1)
        self.assertEqual(len(self.clusters(wide, r)), 2)

    def test_candidate_angle_limits_the_search(self):
        r = scan_with(8.0, self.leg_at(180, 1.2))
        front = (np.abs(ANGLES) <= self.FRONT_HALF) & (r >= 0.15) & (r <= 3.0)
        wide = sg.wide_point_mask(front, ANGLES, r, 0.15, 3.0, self.FRONT_HALF, math.radians(200.0))
        self.assertEqual(self.clusters(wide, r), [])  # ±100度までしか探さない

    def test_background_wall_is_excluded_from_wide_candidates(self):
        r = scan_with(8.0, self.leg_at(90, 1.5))
        bg = np.zeros(ANGLES.shape, dtype=bool)
        bg[np.abs(ANGLES - math.radians(90)) <= math.radians(5)] = True  # 基準スキャンにある壁
        _, wide = self.masks(r, background_excluded=bg)
        self.assertEqual(self.clusters(wide, r), [])

    def test_range_limit_applies(self):
        r = scan_with(8.0, self.leg_at(90, 4.0))
        _, wide = self.masks(r, max_range=3.0)
        self.assertEqual(self.clusters(wide, r), [])


if __name__ == "__main__":
    unittest.main()


class NearestMPayloadTests(unittest.TestCase):
    """person_candidates の nearest_m(全周で一番近い物までの距離。近接停止の免除の確認用)。"""

    def build(self, **kw):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        try:
            from nokosahen_core.person_target_detector import build_person_candidates
        except Exception:  # rclpy無しの環境ではimportできない
            self.skipTest("person_target_detector import失敗")
        return build_person_candidates(1.0, "laser", [], [], False, 0.1, **kw)

    def test_included_when_given(self):
        self.assertEqual(self.build(nearest_m=0.62)["nearest_m"], 0.62)

    def test_absent_when_not_given_or_scan_stale(self):
        self.assertNotIn("nearest_m", self.build())
        self.assertNotIn("nearest_m", self.build(nearest_m=0.62, scan_stale=True))


class LidarPriorityScopeTests(unittest.TestCase):
    """【2026-11-04】LiDAR優先は、その場の旋回(向く)の間だけ。それ以外はカメラだけで障害物と判断する。"""

    def fn(self):
        spec = importlib.util.spec_from_file_location(
            "scan_geometry_scope", Path(__file__).resolve().parents[1] / "nokosahen_core" / "scan_geometry.py"
        )
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod.lidar_priority_active

    def test_only_during_in_place_turn(self):
        f = self.fn()
        self.assertTrue(f(0.2, True))      # 旋回中(通知が新しい): LiDAR優先
        self.assertTrue(f(1.0, True))      # 境界
        self.assertFalse(f(1.5, True))     # 通知が古い(旋回していない): カメラだけ
        self.assertFalse(f(None, True))    # 通知を受けたことが無い/falseを受けた: カメラだけ
        self.assertFalse(f(-0.1, True))    # 異常値は優先しない(安全側)

    def test_always_when_not_limited(self):
        f = self.fn()
        self.assertTrue(f(None, False))
        self.assertTrue(f(99.0, False))
