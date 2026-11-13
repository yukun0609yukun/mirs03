"""AutoSpeedController(自動運転の最高速度)の再publishの単体テスト。

director再起動で既定に戻らないよう、ユーザーが設定した後は5秒ごとに再publishする。
未設定の間は何も出さない(従来動作)。rclpy不要(スタブ注入)。
実行: python ros2_ws/src/nokosahen_core/test/test_auto_speed_republish.py
"""
import json
import os
import sys
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

def _install_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    class Msg:
        def __init__(self, *a, **k):
            self.data = None

    mod("geometry_msgs")
    mod("geometry_msgs.msg", PoseStamped=Msg, Twist=Msg)
    mod("nav_msgs")
    mod("nav_msgs.msg", Odometry=Msg)
    mod("std_msgs")
    mod("std_msgs.msg", Bool=Msg, Float32MultiArray=Msg, String=Msg, Empty=Msg)


try:  # test_nav_speed.py と同じ方式(ROSが無いPCではスタブ、有ればそのまま)
    import geometry_msgs.msg  # noqa: F401
    import nav_msgs.msg  # noqa: F401
    import std_msgs.msg  # noqa: F401
except Exception:  # noqa: BLE001
    _install_stubs()

try:
    from nokosahen_core.dashboard_common import AutoSpeedController
    _ERR = None
except Exception as exc:  # pragma: no cover
    AutoSpeedController = None
    _ERR = exc


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(json.loads(msg.data))


class _Node:
    def __init__(self):
        self.pub = _Pub()
        self.timers = []

    def create_publisher(self, *_a, **_k):
        return self.pub

    def create_timer(self, period, cb):
        self.timers.append((period, cb))


@unittest.skipIf(AutoSpeedController is None, f"dashboard_common import失敗: {_ERR}")
class AutoSpeedRepublishTests(unittest.TestCase):
    def test_no_publish_until_user_sets(self):
        n = _Node()
        AutoSpeedController(n)
        self.assertEqual(len(n.timers), 1)
        self.assertEqual(n.timers[0][0], 5.0)
        n.timers[0][1]()  # 未設定の間は何も出さない
        self.assertEqual(n.pub.sent, [])

    def test_republish_after_set(self):
        n = _Node()
        c = AutoSpeedController(n)
        c.set_max_speed(0.9, 1.2)
        self.assertEqual(len(n.pub.sent), 1)
        n.timers[0][1]()
        n.timers[0][1]()
        self.assertEqual(len(n.pub.sent), 3)
        self.assertEqual(n.pub.sent[-1], {"linear_mps": 0.9, "angular_rps": 1.2})

    def test_nan_is_ignored_but_counts_as_user_set(self):
        n = _Node()
        c = AutoSpeedController(n)
        c.set_max_speed(float("nan"), None)
        self.assertEqual(n.pub.sent[-1], {"linear_mps": 0.75, "angular_rps": 0.75})

    def test_node_without_timer_still_works(self):
        class NoTimer:
            def __init__(self):
                self.pub = _Pub()

            def create_publisher(self, *_a, **_k):
                return self.pub

        n = NoTimer()
        c = AutoSpeedController(n)
        c.set_max_speed(0.5, 0.5)
        self.assertEqual(len(n.pub.sent), 1)


if __name__ == "__main__":
    unittest.main()
