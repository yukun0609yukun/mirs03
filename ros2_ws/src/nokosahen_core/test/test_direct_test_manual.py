"""ダッシュボードの走行テスト(ステップ応答・距離指定、直接/cmd_vel)が、手動運転として宣言されることのテスト(2026-10-23、B3)。
開始時に /nokosahen/cancel_goal と wasd_active=true、終了時に wasd_active=false。WASDが同時に運転中なら、終了時に manual_override・wasd_active を消さない。
ROS・ロボット不要(test_nav2_tuningのスタブを流用。cmd_velなどはメモリ上に溜まるだけ)。
実行: python3 -I ros2_ws/src/nokosahen_core/test/test_direct_test_manual.py
"""
import sys
import time
import types
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(TEST_DIR.parent))

import test_nav2_tuning as tnt  # noqa: E402  (rclpyなどのスタブ注入とStubNodeを流用)

dc = tnt.dc


class _Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0)
        self.angular = types.SimpleNamespace(z=0.0)
        self.data = None


dc.Twist = _Twist  # スタブのTwistはlinear/angularを持たないため差し替える(このテストのプロセス内だけ)


class RecNode(tnt.StubNode):
    def __init__(self, params=None):
        super().__init__()
        self.events = []
        self._declared = {}
        self._given = params or {}

    def create_publisher(self, typ, topic, qos):
        return types.SimpleNamespace(publish=lambda m, t=topic: self.events.append((t, getattr(m, "data", None))))


class FakeTeleop:
    def __init__(self, events, driving=False):
        self.soft_estop_engaged = False
        self._events = events
        self.driving = driving

    def publish_manual_override(self, active):
        self._events.append(("manual_override", active))

    def publish_wasd_active(self, active):
        self._events.append(("wasd_active", active))

    def is_driving(self):
        return self.driving


def make(driving=False, claims=True):
    node = RecNode()
    if not claims:
        node.declare_parameter = lambda name, value: node._params.__setitem__(name, False)
    teleop = FakeTeleop(node.events, driving)
    ctrl = dc.PidGainsController(node, teleop)
    ctrl._begin_fast_odom = lambda *a, **k: None
    ctrl.node.events.clear()
    return ctrl, node, teleop


def wait_done(ctrl, timeout=5.0):
    end = time.monotonic() + timeout
    while ctrl.test_running() and time.monotonic() < end:
        time.sleep(0.02)
    assert not ctrl.test_running()


def seq(events, key):
    return [v for k, v in events if k == key]


class DriveTestClaimsManual(unittest.TestCase):
    def test_start_cancels_goal_and_claims_wasd_then_releases(self):
        ctrl, node, _ = make()
        self.assertTrue(ctrl.run_drive_test(0.1, 0.0, 0.3))
        wait_done(ctrl)
        ev = node.events
        self.assertEqual(seq(ev, "/nokosahen/cancel_goal")[:2], [True, True])
        w = seq(ev, "wasd_active")
        self.assertTrue(w[0] is True and w[-1] is False)
        self.assertEqual(seq(ev, "manual_override")[-1], False)
        # 取り消し・wasd_active=true が、最初の速度指令より前
        first_cmd = next(i for i, (k, _v) in enumerate(ev) if k == "/cmd_vel")
        self.assertLess(ev.index(("/nokosahen/cancel_goal", True)), first_cmd)
        self.assertLess(ev.index(("wasd_active", True)), first_cmd)
        self.assertEqual(ev[-1][0], "wasd_active")
        self.assertIs(ev[-1][1], False)

    def test_wasd_driving_at_the_same_time_is_not_cleared(self):
        ctrl, node, _ = make(driving=True)
        ctrl.run_drive_test(0.1, 0.0, 0.3)
        wait_done(ctrl)
        self.assertEqual(seq(node.events, "manual_override")[-1], True)
        self.assertEqual(seq(node.events, "wasd_active")[-1], True)
        zero = [v for k, v in node.events if k == "/cmd_vel"]
        self.assertTrue(zero)  # ゼロ速度は出す(WASDの有無に関係なく)

    def test_parameter_off_keeps_old_behaviour(self):
        ctrl, node, _ = make(claims=False)
        ctrl.run_drive_test(0.1, 0.0, 0.3)
        wait_done(ctrl)
        self.assertEqual(seq(node.events, "/nokosahen/cancel_goal"), [])
        self.assertEqual(seq(node.events, "wasd_active"), [])
        self.assertEqual(seq(node.events, "manual_override")[-1], False)


class StepResponseClaimsManual(unittest.TestCase):
    def run_step(self, ctrl, tmp):
        return ctrl.run_step_response_test("forward", 0.01, 1.0, 20.0, 0.1, 5.0, 5.0, str(tmp))

    def test_step_response_claims_and_releases(self):
        import tempfile
        ctrl, node, _ = make()
        with tempfile.TemporaryDirectory() as tmp:
            self.assertTrue(self.run_step(ctrl, tmp))
            wait_done(ctrl, 10.0)
        ev = node.events
        self.assertEqual(seq(ev, "/nokosahen/cancel_goal")[:2], [True, True])
        w = seq(ev, "wasd_active")
        self.assertTrue(w[0] is True and w[-1] is False)
        self.assertEqual(seq(ev, "manual_override")[-1], False)

    def test_step_response_with_wasd_driving_keeps_manual(self):
        import tempfile
        ctrl, node, _ = make(driving=True)
        with tempfile.TemporaryDirectory() as tmp:
            self.run_step(ctrl, tmp)
            wait_done(ctrl, 10.0)
        self.assertEqual(seq(node.events, "manual_override")[-1], True)
        self.assertEqual(seq(node.events, "wasd_active")[-1], True)


class TeleopHelpers(unittest.TestCase):
    def test_is_driving_follows_the_watchdog_window(self):
        t = object.__new__(dc.TeleopController)
        t.watchdog_sec = 0.5
        t.last_cmd_time = 0.0
        self.assertFalse(t.is_driving())
        t.last_cmd_time = time.monotonic()
        self.assertTrue(t.is_driving())
        t.last_cmd_time = time.monotonic() - 1.0
        self.assertFalse(t.is_driving())


if __name__ == "__main__":
    unittest.main(verbosity=1)
