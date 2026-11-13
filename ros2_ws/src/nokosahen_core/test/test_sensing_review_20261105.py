"""【2026-11-05】Opusレビュー(2026-11-04)の中・低のうち、センシング・認識の修正のテスト(rclpy不要)。"""
import json
import math
import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def _install_stubs():
    try:
        import rclpy  # noqa: F401
        import sensor_msgs.msg  # noqa: F401
        return
    except ImportError:
        pass

    class _Msg:
        def __init__(self, *a, **k):
            self.data = None

    class _AutoMod(types.ModuleType):
        def __getattr__(self, item):
            if item.startswith("__"):
                raise AttributeError(item)
            return _Msg

    for name in list(sys.modules):
        if name.split(".")[0] in ("rclpy", "sensor_msgs", "std_msgs", "tf2_ros"):
            del sys.modules[name]
    for mod in ("rclpy", "rclpy.node", "rclpy.time", "sensor_msgs", "sensor_msgs.msg",
                "std_msgs", "std_msgs.msg", "tf2_ros"):
        sys.modules[mod] = _AutoMod(mod)


_install_stubs()

from nokosahen_core import scan_geometry as sg  # noqa: E402
from nokosahen_core.person_target_detector import (  # noqa: E402
    build_person_candidates, pose_from_cache,
)
from nokosahen_core.pose_relay import PoseCache  # noqa: E402

GAP = 0.10
ANG = math.radians(3.0)


def _arc(center_deg, width_m, rng, step_deg=1.0):
    """center_deg を中心に、幅 width_m ぶんの点列 (angle, range) を作る(角度は-π〜πに折り返す)。"""
    half_deg = math.degrees(width_m / 2.0 / rng)
    n = int(half_deg / step_deg)
    out = []
    for i in range(-n, n + 1):
        a = math.radians(center_deg + i * step_deg)
        out.append((math.atan2(math.sin(a), math.cos(a)), rng))
    return out


class NearestAgeTest(unittest.TestCase):
    def build(self, **kw):
        return build_person_candidates(1.0, "laser", [], [], False, 0.1, **kw)

    def test_age_with_nearest(self):
        out = self.build(nearest_m=1.2, nearest_age_s=0.12)
        self.assertEqual(out["nearest_m"], 1.2)
        self.assertAlmostEqual(out["nearest_age_s"], 0.12)

    def test_no_age_without_nearest(self):
        out = self.build(nearest_m=None, nearest_age_s=0.1)
        self.assertNotIn("nearest_m", out)
        self.assertNotIn("nearest_age_s", out)

    def test_stale_scan_has_neither(self):
        out = self.build(scan_stale=True, nearest_m=1.2, nearest_age_s=2.0)
        self.assertNotIn("nearest_m", out)
        self.assertNotIn("nearest_age_s", out)

    def test_backward_compatible_default(self):
        out = self.build(nearest_m=1.2)
        self.assertNotIn("nearest_age_s", out)


class ScanStaleTest(unittest.TestCase):
    def test_received_then_silent(self):
        self.assertFalse(sg.scan_is_stale(10.0, 10.5, 0.0, 1.0))
        self.assertTrue(sg.scan_is_stale(10.0, 11.5, 0.0, 1.0))

    def test_never_received_within_grace(self):
        self.assertFalse(sg.scan_is_stale(None, 2.0, 0.0, 1.0, True, 3.0))

    def test_never_received_after_grace_is_stale(self):
        self.assertTrue(sg.scan_is_stale(None, 3.5, 0.0, 1.0, True, 3.0))

    def test_require_scan_false_never_stale(self):
        self.assertFalse(sg.scan_is_stale(None, 100.0, 0.0, 1.0, False, 3.0))

    def test_candidates_empty_when_stale(self):
        out = build_person_candidates(1.0, "laser", [(0.0, 1.0, 0.0)], [], False, 0.1, True, 1.0, None, 0.5)
        self.assertEqual(out["lidar"], [])
        self.assertNotIn("nearest_m", out)


class ObstacleReissueTest(unittest.TestCase):
    def test_memory_fresh_and_old(self):
        m = sg.RawObstacleMemory()
        self.assertIsNone(m.fresh(0.0, 1.0))
        m.update([0.1], 10.0)
        self.assertEqual(m.fresh(10.5, 1.0), [0.1])
        self.assertIsNone(m.fresh(12.0, 1.0))
        m.update([], 20.0)
        self.assertEqual(m.fresh(20.1, 1.0), [])  # 空=障害物なし(Noneとは別)

    def test_turn_edge(self):
        self.assertTrue(sg.in_place_turn_ended(True, False))
        self.assertFalse(sg.in_place_turn_ended(False, False))
        self.assertFalse(sg.in_place_turn_ended(True, True))
        self.assertFalse(sg.in_place_turn_ended(False, True))

    def test_real_obstacle_reaches_after_turn(self):
        """旋回中はLiDARが近くに何も測らず(抑制=false)、旋回が終われば抑制なしで本物の障害物が出る。"""
        raw = [0.2]
        angles = [0.2]
        ranges = [3.0]
        during = [b for b in raw if sg.lidar_confirms_obstacle(angles, ranges, 0.05, 12.0, b, 1.0, 0.3, 0.1, 1.0)]
        self.assertEqual(during, [])
        self.assertFalse(sg.lidar_priority_active(None, True))
        m = sg.RawObstacleMemory()
        m.update(raw, 5.0)
        self.assertTrue(bool(m.fresh(5.1, 1.0)))


class PoseCacheTest(unittest.TestCase):
    def test_fresh_pose(self):
        c = PoseCache()
        c.update(json.dumps({"base": {"stamp": 100.0, "x": 1.0, "y": 2.0, "yaw": 0.5}}))
        self.assertEqual(pose_from_cache(c, 100.5), {"x": 1.0, "y": 2.0, "yaw": 0.5})

    def test_old_or_missing_is_none(self):
        c = PoseCache()
        self.assertIsNone(pose_from_cache(c, 1.0))
        self.assertIsNone(pose_from_cache(None, 1.0))
        c.update(json.dumps({"base": {"stamp": 100.0, "x": 1.0, "y": 2.0, "yaw": 0.5}}))
        self.assertIsNone(pose_from_cache(c, 105.0))
        c.update(json.dumps({"map_odom": {"stamp": 100.0}}))  # baseが含まれない=取れなかった
        self.assertIsNone(pose_from_cache(c, 100.1))


class FrontReuseTest(unittest.TestCase):
    def test_same_as_separate_clustering(self):
        """全周の1回のクラスタリングから切り出した前方の結果が、前方の点だけを別にクラスタリングした結果と同じ。"""
        half = math.radians(70.0)
        pts = []
        for center, rng in ((0, 1.5), (60, 1.2), (70, 2.0), (-69, 1.0), (-70, 1.0), (100, 1.0), (-150, 1.4), (180, 1.1)):
            pts += _arc(center, 0.2, rng)
        pts = sorted(set(pts))
        front_pts = [p for p in pts if abs(p[0]) <= half]
        expect_front = sg.cluster_leg_points(front_pts, GAP, ANG, 0.04, 0.35)
        expect_wide = sg.cluster_leg_points(pts, GAP, ANG, 0.04, 0.35)
        front, wide = sg.cluster_front_and_wide(pts, GAP, ANG, 0.04, 0.35, half)
        self.assertEqual(front, expect_front)
        self.assertEqual(wide, expect_wide)

    def test_boundary_group_trimmed_by_front(self):
        """前方の境界をまたぐ長い壁は、全周では幅超過で消え、前方側では切り取って幅を判定する。"""
        half = math.radians(70.0)
        pts = [(math.radians(a), 1.0) for a in range(60, 81)]
        front_pts = [p for p in pts if abs(p[0]) <= half]
        expect = sg.cluster_leg_points(front_pts, GAP, ANG, 0.04, 0.35)
        front, _wide = sg.cluster_front_and_wide(pts, GAP, ANG, 0.04, 0.35, half)
        self.assertEqual(front, expect)
        self.assertEqual(len(front), 1)


class SeamTest(unittest.TestCase):
    def _leg_across_seam(self):
        a = math.radians
        pts = [(a(-180 + i), 1.0) for i in range(0, 6)]  # -180..-175
        pts = pts + [(a(175 + i), 1.0) for i in range(0, 5)]  # 175..179
        return sorted(pts)

    def test_leg_behind_is_found_with_wrap(self):
        pts = self._leg_across_seam()
        # 継ぎ目で割れると片側は約0.09 m で、最小幅0.12 mに足りず消える
        self.assertEqual(sg.cluster_leg_points(pts, GAP, ANG, 0.12, 0.35), [])
        out = sg.cluster_leg_points(pts, GAP, ANG, 0.12, 0.35, wrap_full=True)
        self.assertEqual(len(out), 1)
        self.assertGreater(abs(out[0][0]), math.radians(170))

    def test_no_join_when_far_apart(self):
        a = math.radians
        pts = sorted([(a(-179), 1.0), (a(-178), 1.0), (a(178), 2.0), (a(179), 2.0)])
        out = sg.cluster_leg_points(pts, GAP, ANG, 0.0, 0.35, wrap_full=True)
        self.assertEqual(len(out), 2)  # 距離差が大きいのでつながない

    def test_no_join_when_seam_has_gap(self):
        a = math.radians
        pts = sorted([(a(-170), 1.0), (a(-169), 1.0), (a(169), 1.0), (a(170), 1.0)])
        out = sg.cluster_leg_points(pts, GAP, ANG, 0.0, 0.35, wrap_full=True)
        self.assertEqual(len(out), 2)  # 継ぎ目に点が無い(角度が離れている)のでつながない

    def test_wide_uses_wrap(self):
        pts = self._leg_across_seam()
        front, wide = sg.cluster_front_and_wide(pts, GAP, ANG, 0.12, 0.35, math.radians(70), wrap_full=True)
        self.assertEqual(front, [])
        self.assertEqual(len(wide), 1)


class BackgroundValidateTest(unittest.TestCase):
    def test_ok(self):
        d, _why = sg.validate_background(
            {"angle_min": -3.1, "angle_increment": 0.01, "ranges": [1.0, float("inf"), float("nan"), 2]})
        self.assertIsNotNone(d)

    def test_null_in_ranges_rejected(self):
        d, why = sg.validate_background({"ranges": [1.0, None, 2.0]})
        self.assertIsNone(d)
        self.assertTrue(why)

    def test_bad_shapes_rejected(self):
        for bad in (None, [], {"ranges": []}, {"ranges": "abc"}, {"x": 1},
                    {"ranges": [1.0], "angle_min": None}, {"ranges": [1.0], "angle_increment": float("nan")},
                    {"ranges": [True]}):
            self.assertIsNone(sg.validate_background(bad)[0], bad)


class RearCameraRetryTest(unittest.TestCase):
    def test_retry_timer(self):
        from nokosahen_core.rear_camera_persons import RetryTimer
        t = RetryTimer(10.0)
        self.assertTrue(t.due("a", 100.0))
        self.assertFalse(t.due("a", 105.0))
        self.assertTrue(t.due("b", 105.0))
        self.assertTrue(t.due("a", 110.0))
        self.assertFalse(t.due("a", 111.0))


if __name__ == "__main__":
    unittest.main()
