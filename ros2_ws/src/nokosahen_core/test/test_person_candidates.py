"""person_target_detector.build_person_candidates(/nokosahen/person_candidates)の単体テスト。

rclpy不要(未インストール時はスタブ注入)。実行: python test/test_person_candidates.py
"""

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

from nokosahen_core.person_target_detector import build_person_candidates  # noqa: E402
from nokosahen_core.json_tools import dumps  # noqa: E402

TOL = math.radians(15.0)


def cluster(bearing, rng):
    return (bearing, rng * math.cos(bearing), rng * math.sin(bearing))


def build(clusters, cams, active=True, stale=False):
    return build_person_candidates(100.0, "laser", clusters, cams, active, TOL, stale)


class PersonCandidatesTest(unittest.TestCase):
    def test_two_clusters_one_camera_match(self):
        out = build([cluster(0.35, 1.8), cluster(-0.9, 2.5)], [0.30])
        self.assertEqual(len(out["lidar"]), 2)
        self.assertTrue(out["lidar"][0]["confirmed"])
        self.assertFalse(out["lidar"][1]["confirmed"])
        self.assertAlmostEqual(out["lidar"][0]["range"], 1.8)
        self.assertAlmostEqual(out["lidar"][1]["range"], 2.5)
        self.assertEqual(out["camera"], [{"bearing": 0.30}])

    def test_camera_does_not_match(self):
        out = build([cluster(0.35, 1.8), cluster(-0.9, 2.5)], [1.5])
        self.assertFalse(any(c["confirmed"] for c in out["lidar"]))
        self.assertEqual(len(out["camera"]), 1)

    def test_degraded_without_camera(self):
        out = build([cluster(0.1, 1.0)], [0.1], active=False)
        self.assertEqual(out["camera"], [])
        self.assertFalse(out["lidar"][0]["confirmed"])
        self.assertEqual(len(out["lidar"]), 1)

    def test_stale_scan_empties_lidar(self):
        out = build([cluster(0.1, 1.0)], [0.1], stale=True)
        self.assertEqual(out["lidar"], [])
        self.assertEqual(len(out["camera"]), 1)

    def test_empty_lists(self):
        out = build([], [])
        self.assertEqual(out["lidar"], [])
        self.assertEqual(out["camera"], [])

    def test_left_is_positive_and_wrapped(self):
        c = (math.radians(30), 1.0, 0.5)  # y>0 = 左
        out = build([c], [])
        self.assertGreater(out["lidar"][0]["bearing"], 0.0)
        out = build([(-math.radians(40), 1.0, -0.8)], [-math.radians(40)])
        self.assertLess(out["lidar"][0]["bearing"], 0.0)
        self.assertLess(out["camera"][0]["bearing"], 0.0)
        out = build([], [3 * math.pi / 2])
        self.assertAlmostEqual(out["camera"][0]["bearing"], -math.pi / 2)

    def test_wraparound_match(self):
        out = build([cluster(math.pi - 0.05, 1.0)], [-math.pi + 0.05])
        self.assertTrue(out["lidar"][0]["confirmed"])

    def test_json_shape(self):
        out = build([cluster(0.35, 1.8)], [0.30])
        data = json.loads(dumps(out))
        self.assertEqual(set(data), {"stamp", "frame", "lidar", "camera"})
        self.assertEqual(data["frame"], "laser")
        self.assertEqual(set(data["lidar"][0]), {"bearing", "range", "confirmed"})
        self.assertEqual(set(data["camera"][0]), {"bearing"})
        self.assertIsInstance(data["lidar"][0]["confirmed"], bool)

    def test_rejected_flag_only_in_front_camera_fov(self):
        """【2026-11-04】カメラ有効で前方視野内の未確認クラスタだけ rejected。視野外・確認済みは付かない。"""
        half = math.radians(75)
        out = build_person_candidates(1.0, "laser", [cluster(0.0, 1.0), cluster(math.radians(120), 1.0), cluster(0.5, 1.0)],
                  [0.5], True, 0.1, False, None, half)
        l = out["lidar"]
        self.assertTrue(l[0].get("rejected"))
        self.assertNotIn("rejected", l[1])
        self.assertNotIn("rejected", l[2])
        out = build_person_candidates(1.0, "laser", [cluster(0.0, 1.0)], [], False, 0.1, False, None, half)
        self.assertNotIn("rejected", out["lidar"][0])


class NearestRangeTest(unittest.TestCase):
    def test_close_object_found_regardless_of_background(self):
        import numpy as np
        from nokosahen_core.scan_geometry import nearest_range
        r = np.array([5.0, 0.35, 2.0, float("inf"), float("nan"), 0.01])
        self.assertAlmostEqual(nearest_range(r, 0.05, 3.0), 0.35)

    def test_nothing_close_is_99(self):
        import numpy as np
        from nokosahen_core.scan_geometry import nearest_range
        self.assertEqual(nearest_range(np.array([5.0, float("inf")]), 0.05, 3.0), 99.0)


if __name__ == "__main__":
    unittest.main()
