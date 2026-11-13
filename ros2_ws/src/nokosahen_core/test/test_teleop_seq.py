"""TeleopController.handle_drive のseq/tab_id制御の単体テスト(rclpy不要、スタブ注入)。

実行: python ros2_ws/src/nokosahen_core/test/test_teleop_seq.py
"""

import sys
import types
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def _install_stubs():
    """dashboard_commonがimportするROS系モジュールを最小スタブへ差し替える(未インストール時のみ)。"""
    try:
        import rclpy  # noqa: F401
        import geometry_msgs.msg  # noqa: F401
        return
    except ImportError:
        pass

    class _Msg:
        def __init__(self, *a, **k):
            self.data = None
            self.linear = types.SimpleNamespace(x=0.0)
            self.angular = types.SimpleNamespace(z=0.0)

    for name in list(sys.modules):
        if name.split(".")[0] in ("rclpy", "geometry_msgs", "nav_msgs", "std_msgs", "sensor_msgs", "nav2_msgs", "action_msgs", "tf2_ros", "visualization_msgs"):
            del sys.modules[name]

    class _AutoMod(types.ModuleType):
        def __getattr__(self, item):
            if item.startswith("__"):
                raise AttributeError(item)
            return _Msg

    for mod in ("rclpy", "rclpy.node", "rclpy.action", "rclpy.qos", "rclpy.duration", "rclpy.time",
                "geometry_msgs", "geometry_msgs.msg", "nav_msgs", "nav_msgs.msg", "std_msgs", "std_msgs.msg",
                "sensor_msgs", "sensor_msgs.msg", "nav2_msgs", "nav2_msgs.action", "nav2_msgs.msg",
                "action_msgs", "action_msgs.msg", "tf2_ros", "visualization_msgs", "visualization_msgs.msg"):
        sys.modules[mod] = _AutoMod(mod)


_install_stubs()
try:
    from nokosahen_core import dashboard_common as dc  # noqa: E402
    _IMPORT_ERR = None
except Exception as exc:  # noqa: BLE001
    dc = None
    _IMPORT_ERR = exc


class _Node:
    def create_publisher(self, *a, **k):
        return mock.Mock()

    def create_timer(self, *a, **k):
        return mock.Mock()

    def create_subscription(self, *a, **k):
        return mock.Mock()

    def get_logger(self):
        return mock.Mock()


@unittest.skipIf(dc is None, f"dashboard_common import failed: {_IMPORT_ERR}")
class TeleopSeqTest(unittest.TestCase):
    def setUp(self):
        self.t = dc.TeleopController(_Node())
        self.t.publish_manual_override = mock.Mock()
        self.t._publish_wasd_active = mock.Mock()
        self.published = []
        self.t.cmd_vel_pub = types.SimpleNamespace(publish=lambda m: self.published.append(m))

    def test_late_forward_does_not_overtake_stop(self):
        self.assertTrue(self.t.handle_drive(1.0, 0.0, source="a@1#t1", seq=5))
        self.assertTrue(self.t.handle_drive(0.0, 0.0, source="a@1#t1", seq=7))  # stop
        n = len(self.published)
        self.assertTrue(self.t.handle_drive(1.0, 0.0, source="a@1#t1", seq=6))  # 遅着
        self.assertEqual(len(self.published), n)  # 破棄(何もpublishされない)
        self.assertEqual(self.t.last_cmd_time, 0.0)  # 停止状態のまま

    def test_without_seq_is_legacy(self):
        self.t.handle_drive(1.0, 0.0, source="a@1")
        self.t.handle_drive(1.0, 0.0, source="a@1")
        self.assertEqual(len(self.published), 2)

    def test_other_tab_idle_is_not_own_stop(self):
        self.t.handle_drive(1.0, 0.0, source="a@1#drive", seq=1)
        n = len(self.published)
        self.t.handle_drive(0.0, 0.0, source="a@1#idle", seq=1)
        self.assertEqual(len(self.published), n)  # 別タブのアイドルは無視
        self.assertNotEqual(self.t.last_cmd_time, 0.0)
        self.t.handle_drive(0.0, 0.0, source="a@1#drive", seq=2)  # 自タブの停止は効く
        self.assertEqual(self.t.last_cmd_time, 0.0)

    def test_seq_independent_per_source(self):
        self.t.handle_drive(1.0, 0.0, source="a@1#x", seq=100)
        n = len(self.published)
        self.t.handle_drive(1.0, 0.0, source="a@1#y", seq=1)
        self.assertEqual(len(self.published), n + 1)

    def test_equal_seq_not_dropped_and_tracking_bounded(self):
        self.t.handle_drive(1.0, 0.0, source="s", seq=3)
        n = len(self.published)
        self.t.handle_drive(1.0, 0.0, source="s", seq=3)
        self.assertEqual(len(self.published), n + 1)
        for i in range(dc._TELEOP_SEQ_TRACKED_MAX + 50):
            self.t.handle_drive(0.0, 0.0, source=f"x{i}", seq=1)
        self.assertLessEqual(len(self.t._drive_seq), dc._TELEOP_SEQ_TRACKED_MAX)

    def test_helpers(self):
        self.assertEqual(dc.build_teleop_source("a", "1.2.3.4"), "a@1.2.3.4")
        self.assertEqual(dc.build_teleop_source("a", "1.2.3.4", "ab_C-9"), "a@1.2.3.4#ab_C-9")
        self.assertEqual(dc.build_teleop_source("a", "i", "a b/../" + "z" * 40), "a@i#ab" + "z" * 14)
        self.assertEqual(dc.build_teleop_source("a", "i", "!!!"), "a@i")
        self.assertEqual(dc.build_teleop_source("a", "i", 5), "a@i")
        self.assertIsNone(dc.parse_teleop_seq(None))
        self.assertIsNone(dc.parse_teleop_seq(True))
        self.assertIsNone(dc.parse_teleop_seq(-1))
        self.assertIsNone(dc.parse_teleop_seq("3"))
        self.assertIsNone(dc.parse_teleop_seq(float("nan")))
        self.assertEqual(dc.parse_teleop_seq(7), 7)
        self.assertEqual(dc.parse_teleop_seq(7.0), 7)


if __name__ == "__main__":
    unittest.main()
