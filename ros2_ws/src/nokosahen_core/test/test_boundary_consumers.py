"""走行可能範囲(回転矩形)を、behavior_director と nav2_goal_router が真の矩形で扱うことの単体テスト。

rclpy不要(未インストール時はスタブ注入)。ロボット・ROSには一切触れない(cmd_vel/ゴールは
スタブのpublisherに溜まるだけ)。
実行: python ros2_ws/src/nokosahen_core/test/test_boundary_consumers.py
"""

import json
import math
import sys
import types
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(TEST_DIR.parent))

import test_behavior_rules as tbr  # noqa: E402  (rclpyスタブ注入とdirectorの組み立てを流用)

for _name, _attrs in (("rclpy.action", ("ActionClient",)), ("rclpy.duration", ("Duration",))):
    if _name not in sys.modules:
        try:
            __import__(_name)
        except Exception:  # noqa: BLE001
            _m = types.ModuleType(_name)
            for _a in _attrs:
                setattr(_m, _a, object)
            sys.modules[_name] = _m

from nokosahen_core import map_orientation as mo  # noqa: E402

bd = tbr.bd
try:
    from nokosahen_core import nav2_goal_router as router  # noqa: E402
    _ROUTER_ERR = None
except Exception as exc:  # pragma: no cover
    router = None
    _ROUTER_ERR = exc

YAW30 = math.radians(30.0)


def rotated_state(enabled=True, cx=0.0, cy=0.0, w=6.0, h=2.0, yaw=YAW30):
    return mo.boundary_state(mo.make_rect(cx, cy, w, h, yaw), enabled)


# 長辺方向(30度)に2.5m進んだ点: 回転矩形(6x2)の内側
INSIDE = (2.5 * math.cos(YAW30), 2.5 * math.sin(YAW30))
# ローカル座標(2.0, 1.3): 短辺方向に高さの半分(1.0)を超えるので真の矩形の外側だが、
# 軸平行の外接矩形の中には入ってしまう(旧方式だと「範囲内」と誤認される位置)
OUTSIDE = (
    2.0 * math.cos(YAW30) - 1.3 * math.sin(YAW30),
    2.0 * math.sin(YAW30) + 1.3 * math.cos(YAW30),
)


class _Msg:
    def __init__(self, payload):
        self.data = payload if isinstance(payload, str) else json.dumps(payload)


@unittest.skipIf(bd is None, f"behavior_director import失敗: {tbr._BD_ERR}")
class DirectorBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.helper = tbr.DirectorTickTests("run_ticks")
        self.helper.setUp()
        self.addCleanup(self.helper.tearDown)
        self.n = self.helper.make()

    def feed(self, payload):
        self.n._on_boundary(_Msg(payload))

    def at(self, x, y):
        self.n._get_pose = lambda: {"x": x, "y": y, "yaw": 0.0}

    def test_point_inside_outside_for_yaw_30(self):
        self.feed(rotated_state())
        self.at(*INSIDE)
        self.assertFalse(self.n._out_of_bounds)
        b = self.n._boundary
        # 旧方式(外接矩形の判定)なら、この点は「範囲内」に見えてしまう(回帰の意味づけ)
        self.assertTrue(b["min_x"] <= OUTSIDE[0] <= b["max_x"] and b["min_y"] <= OUTSIDE[1] <= b["max_y"])
        self.at(*OUTSIDE)
        self.assertTrue(self.n._out_of_bounds)

    def test_clamp_outside_entrance_lands_inside_rotated_rect(self):
        self.feed(rotated_state())
        rect = mo.rect_from_state(self.n._boundary)
        for ex, ey in [(10.0, 0.0), (-4.0, 5.0), (0.0, -7.0), (3.0, 3.0), OUTSIDE]:
            cx, cy = self.n._clamp_to_boundary(ex, ey)
            self.assertTrue(mo.point_in_rect(cx, cy, rect), (ex, ey, cx, cy))
        # すでに内側の点は動かさない
        self.assertEqual(self.n._clamp_to_boundary(0.3, 0.1), (0.3, 0.1))

    def test_entrance_goal_is_projected_into_rect(self):
        self.feed(rotated_state())
        self.n.entrance_x, self.n.entrance_y = OUTSIDE
        self.n._clock.t += 10
        self.assertTrue(self.n._publish_entrance_goal())
        goal = self.n._goal_pub.msgs[-1]
        rect = mo.rect_from_state(self.n._boundary)
        self.assertTrue(mo.point_in_rect(goal.pose.position.x, goal.pose.position.y, rect))

    def test_old_form_message_still_works(self):
        self.feed({"enabled": True, "min_x": 0, "max_x": 2, "min_y": 0, "max_y": 1})
        self.at(1.0, 0.5)
        self.assertFalse(self.n._out_of_bounds)
        self.at(2.5, 0.5)
        self.assertTrue(self.n._out_of_bounds)
        # 目的地は縁から GOAL_MARGIN_M 内側へ(2026-11-02)。範囲(2 x 1 m)は縦が狭いので、y は中心(0.5)から余りの分だけ。
        from nokosahen_core.boundary_motion_guard import GOAL_MARGIN_M as M
        x, y = self.n._clamp_to_boundary(5.0, 5.0)
        self.assertAlmostEqual(x, 2.0 - M, places=6)
        self.assertAlmostEqual(y, 0.5 + (0.5 - M), places=6)
        x, y = self.n._clamp_to_boundary(-1.0, 0.5)
        self.assertAlmostEqual(x, M, places=6)
        self.assertAlmostEqual(y, 0.5, places=6)

    def test_non_finite_and_malformed_messages_ignored(self):
        self.feed(rotated_state())
        before = dict(self.n._boundary)
        bad_messages = [
            "not json", "[1,2]", "null",
            {"enabled": True},
            {"enabled": True, "min_x": 0, "max_x": 1, "min_y": 0},
            {"enabled": True, "center_x": float("nan"), "center_y": 0, "width": 1, "height": 1, "yaw": 0},
            '{"enabled": true, "center_x": NaN, "center_y": 0, "width": 1, "height": 1}',
            {"enabled": True, "center_x": 0, "center_y": 0, "width": float("inf"), "height": 1},
            {"enabled": True, "center_x": 0, "center_y": 0, "width": -1, "height": 1},
            {"enabled": True, "min_x": 1, "max_x": 0, "min_y": 0, "max_y": 1},
            {"enabled": True, "min_x": "a", "max_x": 1, "min_y": 0, "max_y": 1},
        ]
        for m in bad_messages:
            self.feed(m)
            self.assertEqual(self.n._boundary, before, m)
        self.assertFalse(self.n._boundary_unreadable)

    def test_disabled_message_clears_and_unset_means_no_restriction(self):
        self.feed(rotated_state())
        self.feed(mo.disabled_state())
        self.at(1e3, 1e3)
        self.assertFalse(self.n._out_of_bounds)
        self.assertEqual(self.n._clamp_to_boundary(7.0, 8.0), (7.0, 8.0))

    def test_unreadable_enabled_state_fails_safe(self):
        # 万一enabledなのに読めない状態になっても、停止側に倒れてゴールは送らない
        self.n._boundary = {"enabled": True, "center_x": float("nan")}
        self.at(0.0, 0.0)
        self.assertTrue(self.n._out_of_bounds)
        self.n._clock.t += 10
        self.assertFalse(self.n._publish_entrance_goal())
        self.assertEqual(self.n._goal_pub.msgs, [])

    def test_out_of_bounds_hold_in_tick_uses_true_rect(self):
        self.feed(rotated_state())
        self.at(*OUTSIDE)
        modes = self.helper.run_ticks(self.n, 0.5)
        self.assertEqual(set(modes), {"OUT_OF_BOUNDS_HOLD"})
        self.assertFalse(self.helper.moved(self.n))


class _Future:
    def add_done_callback(self, cb):
        pass


class _Client:
    def __init__(self):
        self.sent = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        self.sent.append(goal)
        return _Future()


class _GoalAction:
    class Goal:
        def __init__(self):
            self.pose = None


def _pose(x, y, frame="map"):
    p = types.SimpleNamespace()
    p.header = types.SimpleNamespace(frame_id=frame)
    p.pose = types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y))
    return p


@unittest.skipIf(router is None, f"nav2_goal_router import失敗: {_ROUTER_ERR}")
class RouterBoundaryTests(unittest.TestCase):
    def make(self):
        r = object.__new__(router.Nav2GoalRouter)
        r._navigate_action = _GoalAction
        r._client = _Client()
        r._hardware_estop = False
        r._soft_estop = False
        r._boundary_rect = None
        r.statuses = []
        r.get_logger = lambda: types.SimpleNamespace(
            warn=lambda m: None, info=lambda m: None, error=lambda m: None
        )
        r._publish_goal_status = lambda event, status=0, target=None: r.statuses.append((event, target))
        return r

    def test_unset_boundary_allows_everything(self):
        r = self.make()
        r._on_goal_pose(_pose(1e3, 1e3))
        self.assertEqual(len(r._client.sent), 1)

    def test_goal_outside_rotated_rect_rejected_inside_accepted(self):
        r = self.make()
        r._on_boundary(_Msg(rotated_state()))
        r._on_goal_pose(_pose(*INSIDE))
        self.assertEqual(len(r._client.sent), 1)
        r._on_goal_pose(_pose(*OUTSIDE))
        self.assertEqual(len(r._client.sent), 1)  # 増えない
        self.assertEqual(r.statuses[-1][0], "outside_boundary")

    def test_non_map_frame_not_judged(self):
        r = self.make()
        r._on_boundary(_Msg(rotated_state()))
        r._on_goal_pose(_pose(50.0, 50.0, frame="laser"))
        self.assertEqual(len(r._client.sent), 1)

    def test_invalid_boundary_message_keeps_previous(self):
        r = self.make()
        r._on_boundary(_Msg(rotated_state()))
        r._on_boundary(_Msg('{"enabled": true, "center_x": NaN, "center_y": 0, "width": 1, "height": 1}'))
        r._on_boundary(_Msg({"enabled": True}))
        self.assertIsNotNone(r._boundary_rect)
        r._on_goal_pose(_pose(50.0, 50.0))
        self.assertEqual(r.statuses[-1][0], "outside_boundary")
        # 解除メッセージで制限なしへ
        r._on_boundary(_Msg(mo.disabled_state()))
        r._on_goal_pose(_pose(50.0, 50.0))
        self.assertEqual(len(r._client.sent), 1)

    def test_estop_latch_still_wins(self):
        r = self.make()
        r._hardware_estop = True
        r._on_goal_pose(_pose(0.0, 0.0))
        self.assertEqual(r.statuses[-1][0], "estop_blocked")
        self.assertEqual(r._client.sent, [])


if __name__ == "__main__":
    unittest.main()
