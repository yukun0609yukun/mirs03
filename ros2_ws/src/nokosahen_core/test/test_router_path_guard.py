"""nav2_goal_router の /plan 監視(経路が走行可能範囲の外へ出るなら止める)のテスト。ROSは使わない。

実行: python3 -I ros2_ws/src/nokosahen_core/test/test_router_path_guard.py
"""
import os
import sys
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

for _name, _attrs in (
    ("rclpy", ()), ("rclpy.action", ("ActionClient",)), ("rclpy.duration", ("Duration",)),
    ("rclpy.node", ("Node",)), ("geometry_msgs", ()), ("geometry_msgs.msg", ("PoseStamped", "Twist")),
    ("std_msgs", ()), ("std_msgs.msg", ("Bool", "String")),
):
    try:
        __import__(_name)
    except Exception:  # noqa: BLE001
        _m = types.ModuleType(_name)
        for _a in _attrs:
            setattr(_m, _a, object)
        sys.modules[_name] = _m

from nokosahen_core import map_orientation as mo  # noqa: E402
from nokosahen_core import nav2_goal_router as r  # noqa: E402


class FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(("info", m))

    def warn(self, m):
        self.lines.append(("warn", m))

    def error(self, m):
        self.lines.append(("error", m))


def plan(points, frame="map"):
    return types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id=frame),
        poses=[types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y)))
               for x, y in points])


def line(x0, y0, x1, y1, n=40):
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]


GOOD = line(0, 0, 1.5, 0)
BAD = line(0, 0, 0, 3.0)  # 4 m角(縁は2.0)の外へ1 m


def goal(x, y):
    return types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y)))


class PathGuardTests(unittest.TestCase):
    def make(self):
        n = r.Nav2GoalRouter.__new__(r.Nav2GoalRouter)
        n._boundary_rect = mo.make_rect(0.0, 0.0, 4.0, 4.0, 0.0)
        n._goal_active = True
        n._current_goal_xy = (1.0, 1.0)
        n._plan_last_time = 0.0
        n._plan_violations = 0
        n._plan_blocked_xy = None
        n._plan_blocked_until = 0.0
        n._log = FakeLogger()
        n.get_logger = lambda: n._log
        n.cancels = []
        n.statuses = []
        n._cancel_active_goal = lambda reason="": n.cancels.append(reason)
        n._publish_goal_status = lambda event, status=0, target=None: n.statuses.append((event, target))
        return n

    def feed(self, n, msg):
        n._plan_last_time = 0.0  # 間引きを通す
        n._on_plan(msg)

    def test_two_consecutive_violations_cancel(self):
        n = self.make()
        self.feed(n, plan(BAD))
        self.assertEqual(n.cancels, [])
        self.feed(n, plan(BAD))
        self.assertEqual(n.cancels, ["経路が走行可能範囲の外へ出る"])
        self.assertEqual(n.statuses, [("path_outside_boundary", (1.0, 1.0))])
        self.assertEqual(sum(1 for k, _ in n._log.lines if k == "warn"), 1)

    def test_single_violation_does_not_cancel(self):
        n = self.make()
        self.feed(n, plan(BAD))
        self.feed(n, plan(GOOD))
        self.feed(n, plan(BAD))
        self.assertEqual(n.cancels, [])
        self.assertEqual(n._plan_violations, 1)

    def test_good_plans_never_cancel(self):
        n = self.make()
        for _ in range(10):
            self.feed(n, plan(GOOD))
        self.assertEqual(n.cancels, [])

    def test_noop_cases(self):
        n = self.make()
        n._boundary_rect = None
        self.feed(n, plan(BAD))
        self.feed(n, plan(BAD))
        self.assertEqual(n.cancels, [])
        n = self.make()
        n._goal_active = False
        self.feed(n, plan(BAD))
        self.feed(n, plan(BAD))
        self.assertEqual(n.cancels, [])
        n = self.make()
        self.feed(n, plan(BAD, frame="odom"))
        self.feed(n, plan(BAD, frame="odom"))
        self.assertEqual(n.cancels, [])
        n = self.make()
        self.feed(n, plan(BAD, frame=""))   # 空のframe_idは許可
        self.feed(n, plan(BAD, frame=""))
        self.assertEqual(len(n.cancels), 1)

    def test_noop_resets_counter(self):
        n = self.make()
        self.feed(n, plan(BAD))
        n._goal_active = False
        self.feed(n, plan(BAD))
        n._goal_active = True
        self.feed(n, plan(BAD))
        self.assertEqual(n.cancels, [])

    def test_throttle(self):
        n = self.make()
        n._on_plan(plan(BAD))
        n._on_plan(plan(BAD))  # 直後なので間引かれる
        self.assertEqual(n.cancels, [])
        self.assertEqual(n._plan_violations, 1)

    def test_blocked_goal_rejected_for_10s_then_accepted(self):
        n = self.make()
        self.feed(n, plan(BAD))
        self.feed(n, plan(BAD))
        self.assertGreater(n._plan_blocked_until, 0.0)
        n._navigate_action = object()
        n._hardware_estop = False
        n._soft_estop = False
        n._survey_active = False
        n._survey_active_time = None
        n._goal_outside_boundary = lambda pose: False
        n._client = types.SimpleNamespace(server_is_ready=lambda: False)  # 拒否されなければここ(server_unavailable)まで進む
        n.statuses.clear()
        n._on_goal_pose(goal(1.1, 1.05))
        self.assertEqual([e for e, _ in n.statuses], ["path_outside_boundary"])
        n.statuses.clear()
        n._on_goal_pose(goal(-1.0, -1.0))  # 離れた目的地は拒否しない
        self.assertEqual([e for e, _ in n.statuses], ["server_unavailable"])
        n._plan_blocked_until -= 11.0      # 10秒後は受理
        n.statuses.clear()
        n._on_goal_pose(goal(1.1, 1.05))
        self.assertEqual([e for e, _ in n.statuses], ["server_unavailable"])


if __name__ == "__main__":
    unittest.main()
