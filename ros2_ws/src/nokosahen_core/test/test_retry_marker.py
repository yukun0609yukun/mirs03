"""wall_escape の出し直しの目印(retry_marker.py)のテスト(2026-10-23, B7)。ROS不要。
実行: python3 -I ros2_ws/src/nokosahen_core/test/test_retry_marker.py
"""
import math
import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core.retry_marker import RetryMarker, pose_key  # noqa: E402


def goal(x, y, yaw):
    pos = types.SimpleNamespace(x=x, y=y)
    ori = types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
    return types.SimpleNamespace(pose=types.SimpleNamespace(position=pos, orientation=ori))


class RetryMarkerTests(unittest.TestCase):
    def test_pose_key_yaw(self):
        x, y, yaw = pose_key(goal(1.0, 2.0, 0.7))
        self.assertAlmostEqual(yaw, 0.7)
        self.assertEqual((x, y), (1.0, 2.0))

    def test_no_mark_means_not_mine(self):
        m = RetryMarker()
        self.assertFalse(m.consume((1.0, 2.0, 0.0), 0.0))

    def test_matching_goal_is_mine_once(self):
        m = RetryMarker()
        m.mark((1.0, 2.0, 0.5), 10.0)
        self.assertTrue(m.consume((1.0, 2.0, 0.5), 10.1))
        self.assertFalse(m.consume((1.0, 2.0, 0.5), 10.2))  # 使い切り

    def test_other_source_goal_first_is_not_mistaken(self):
        m = RetryMarker()
        m.mark((1.0, 2.0, 0.5), 10.0)
        self.assertFalse(m.consume((5.0, 5.0, 0.0), 10.1))  # 別の出し元の目的地
        self.assertTrue(m.pending(10.2))                    # 目印は残る
        self.assertTrue(m.consume((1.0, 2.0, 0.5), 10.3))   # 自分の出し直しは、あとで正しく判定

    def test_marker_expires(self):
        m = RetryMarker(ttl_sec=2.0)
        m.mark((1.0, 2.0, 0.5), 10.0)
        self.assertFalse(m.pending(12.5))
        self.assertFalse(m.consume((1.0, 2.0, 0.5), 12.5))  # 時間切れ後は一致しても無視しない
        self.assertFalse(m.pending(12.6))                   # 捨てられている

    def test_tolerances_and_yaw_wrap(self):
        m = RetryMarker()
        m.mark((1.0, 2.0, math.pi - 1e-4), 0.0)
        self.assertTrue(m.consume((1.0004, 2.0, -math.pi + 1e-4), 0.1))  # yawの周回をまたいでも一致
        m.mark((1.0, 2.0, 0.0), 0.0)
        self.assertFalse(m.consume((1.01, 2.0, 0.0), 0.1))
        self.assertFalse(m.consume((1.0, 2.0, 0.05), 0.1))

    def test_legacy_mode_ignores_next_unconditionally(self):
        m = RetryMarker(match=False)
        m.mark((1.0, 2.0, 0.5), 0.0)
        self.assertTrue(m.pending(100.0))  # 失効しない(従来動作)
        self.assertTrue(m.consume((9.0, 9.0, 0.0), 100.0))
        self.assertFalse(m.consume((9.0, 9.0, 0.0), 100.1))

    def test_clear(self):
        m = RetryMarker()
        m.mark((1.0, 2.0, 0.5), 0.0)
        m.clear()
        self.assertFalse(m.consume((1.0, 2.0, 0.5), 0.1))


if __name__ == "__main__":
    unittest.main()
