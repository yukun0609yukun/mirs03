"""scan_self_filterの純粋関数の単体テスト(rclpy不要、numpyのみ)。

実行: python ros2_ws/src/nokosahen_core/test/test_scan_self_filter.py
"""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nokosahen_core.scan_self_filter import ScanMaskCache, apply_filter, build_window_mask  # noqa: E402

CENTERS = [-134.75, -43.42, 43.92, 141.24]
AMIN = math.radians(-179.0)
INC = math.radians(0.333)
N = 1080


def idx(deg):
    return int(round((math.radians(deg) - AMIN) / INC))


class T(unittest.TestCase):
    def setUp(self):
        self.mask = build_window_mask(AMIN, INC, N, CENTERS, 6.0)

    def test_near_in_window_becomes_inf(self):
        r = np.full(N, 5.0, dtype=np.float32)
        for c in CENTERS:
            r[idx(c)] = 0.35
        out = apply_filter(r, self.mask, 0.5)
        for c in CENTERS:
            self.assertTrue(math.isinf(out[idx(c)]))

    def test_far_in_window_kept(self):
        r = np.full(N, 5.0, dtype=np.float32)
        r[idx(43.92)] = 1.2
        out = apply_filter(r, self.mask, 0.5)
        self.assertAlmostEqual(float(out[idx(43.92)]), 1.2, places=5)

    def test_outside_window_untouched(self):
        r = np.full(N, 0.3, dtype=np.float32)
        out = apply_filter(r, self.mask, 0.5)
        outside = ~self.mask
        self.assertTrue(np.all(out[outside] == np.float32(0.3)))
        self.assertTrue(np.all(np.isinf(out[self.mask])))
        self.assertTrue(0 < self.mask.sum() < N)

    def test_existing_inf_nan_preserved(self):
        r = np.full(N, 0.3, dtype=np.float32)
        r[idx(43.92)] = np.nan
        out = apply_filter(r, self.mask, 0.5)
        self.assertTrue(math.isnan(out[idx(43.92)]))

    def test_input_not_modified(self):
        r = np.full(N, 0.3, dtype=np.float32)
        apply_filter(r, self.mask, 0.5)
        self.assertTrue(np.all(r == np.float32(0.3)))

    def test_angle_wrap(self):
        # 中心180度の窓は-180付近(配列の先頭)と+180付近(末尾)の両方に掛かる
        m = build_window_mask(AMIN, INC, N, [180.0], 3.0)
        self.assertTrue(m[0] and m[-1])
        self.assertFalse(m[N // 2])

    def test_window_width(self):
        m = build_window_mask(AMIN, INC, N, [43.92], 6.0)
        self.assertTrue(abs(int(m.sum()) - 12.0 / 0.333) <= 2)

    def test_cache_rebuild(self):
        c = ScanMaskCache()
        _, a = c.get(AMIN, INC, N, CENTERS, 6.0)
        _, b = c.get(AMIN, INC, N, CENTERS, 6.0)
        _, d = c.get(AMIN + 0.01, INC, N, CENTERS, 6.0)
        self.assertEqual((a, b, d), (True, False, True))


if __name__ == "__main__":
    unittest.main()
