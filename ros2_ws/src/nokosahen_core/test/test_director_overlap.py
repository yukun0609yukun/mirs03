"""behavior_directorの、動きの重複対策(2026-10-23、B2/B3/B6)の挙動テスト。ROS不要(test_survey_holdと同じスタブ流用)。
実行: python3 -I ros2_ws/src/nokosahen_core/test/test_director_overlap.py
"""

import json
import sys
import types
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(TEST_DIR.parent))

import test_behavior_rules as tbr  # noqa: E402  (rclpyスタブ注入とdirectorの組み立てを流用)
import test_explore_approach as tea  # noqa: E402  (frontier_explorerのimportスタブを流用)

bd = tbr.bd
fe = tea.fe


class _BoolMsg:
    def __init__(self, data):
        self.data = data


def cancels(n):
    return sum(1 for m in n._cancel_goal_pub.msgs if m.data)


def nonzero(n):
    return [t for t in n._cmd_vel_pub.msgs if t.linear.x != 0.0 or t.angular.z != 0.0]


def zero_twists(n):
    return [t for t in n._cmd_vel_pub.msgs if t.linear.x == 0.0 and t.angular.z == 0.0]


@unittest.skipIf(bd is None, f"behavior_director import失敗: {tbr._BD_ERR}")
class DirectorOverlapTests(unittest.TestCase):
    def setUp(self):
        self.helper = tbr.DirectorTickTests("run_ticks")
        self.helper.setUp()
        self.addCleanup(self.helper.tearDown)

    def make(self, **kw):
        return self.helper.make(**kw)

    def run_ticks(self, n, seconds):
        return self.helper.run_ticks(n, seconds)

    def survey(self, n, seconds, value=True):
        """5Hzのsurvey_active配信を模して回す。"""
        for _ in range(int(round(seconds / 0.1))):
            if int(round(n._clock.t * 10)) % 2 == 0:
                n._on_survey_active(_BoolMsg(value))
            self.run_ticks(n, 0.1)

    # --- B2: HOLDのcancel_goalは最初の1回だけ、ゼロ速度は毎tick ---
    def test_hold_cancels_once_but_zero_velocity_every_tick(self):
        n = self.make(crowd="calm")
        n._clock.t += 10
        self.survey(n, 3.0)  # 30 tick
        self.assertEqual(cancels(n), 1)
        self.assertGreaterEqual(len(zero_twists(n)), 30)

    def test_hold_cancel_every_tick_when_disabled(self):
        n = self.make(crowd="calm")
        n._hold_edge = bd.HoldCancelEdge(False)
        n._clock.t += 10
        self.survey(n, 3.0)
        self.assertGreaterEqual(cancels(n), 30)

    def test_hold_cancels_again_on_next_entry(self):
        n = self.make(crowd="calm")
        n._clock.t += 10
        self.survey(n, 1.0)
        self.survey(n, 1.0, value=False)  # 解除
        self.survey(n, 1.0)
        self.assertEqual(cancels(n), 2)

    def test_near_person_hold_cancels_once(self):
        n = self.make(crowd="calm")
        n._clock.t += 10
        for _ in range(20):
            n._on_near_person_stop(_BoolMsg(True))
            self.run_ticks(n, 0.1)
        modes = json.loads(n._command_pub.msgs[-1].data)["mode"]
        self.assertEqual(modes, "NEAR_PERSON_HOLD")
        self.assertEqual(cancels(n), 1)
        self.assertGreaterEqual(len(zero_twists(n)), 20)

    # --- B6: ユーザー中止の後は、一定時間 目的地を出し直さない ---
    def approach_ready(self):
        n = self.make(crowd="calm")
        n._clock.t += 3  # 接近ゴールの再送間隔を越える
        return n

    def test_approach_goal_resent_without_user_cancel(self):
        n = self.approach_ready()
        self.run_ticks(n, 1.0)
        self.assertTrue(n._goal_pub.msgs)

    def test_user_cancel_stops_goals_for_hold_time_then_resumes(self):
        n = self.approach_ready()
        n._on_user_cancel(_BoolMsg(True))
        self.run_ticks(n, 9.0)
        self.assertEqual(n._goal_pub.msgs, [])  # 10秒は出し直さない(接近の2秒再送間隔・入口の5秒間隔を越えても)
        self.run_ticks(n, 2.0)
        self.assertTrue(n._goal_pub.msgs)  # 10秒を過ぎたら再開

    def test_user_cancel_false_is_ignored(self):
        n = self.approach_ready()
        n._on_user_cancel(_BoolMsg(False))
        self.run_ticks(n, 1.0)
        self.assertTrue(n._goal_pub.msgs)

    def test_user_cancel_hold_zero_disables(self):
        n = self.approach_ready()
        n._user_cancel = bd.UserCancelHold(0.0)
        n._on_user_cancel(_BoolMsg(True))
        self.run_ticks(n, 1.0)
        self.assertTrue(n._goal_pub.msgs)

    def test_own_cancel_goal_does_not_suppress_goals(self):
        n = self.approach_ready()
        n._publish_cancel_goal()  # 自分の取り消し(購読もしない=/nokosahen/user_cancelだけが止める)
        self.run_ticks(n, 1.0)
        self.assertTrue(n._goal_pub.msgs)

    def test_user_cancel_stops_entrance_goal_too(self):
        n = self.make(crowd="crowded", target=False)
        n._clock.t += 10
        n._on_user_cancel(_BoolMsg(True))
        self.run_ticks(n, 3.0)
        self.assertEqual(n._goal_pub.msgs, [])

    # --- B3: 直接/cmd_velは、motion_arbiterの許可があるときだけ ---
    def grant(self, n, allowed):
        n._on_motion_grant(types.SimpleNamespace(data=json.dumps({"owner": "idle", "allow": {"director_direct": allowed}})))

    def scanning(self):
        n = self.make(crowd="calm", target=False)
        n._clock.t += 3
        return n

    def test_no_grant_ever_received_moves_as_before(self):
        n = self.scanning()
        self.run_ticks(n, 1.0)
        self.assertTrue(nonzero(n))

    def test_scan_moves_with_grant_and_stops_without(self):
        n = self.scanning()
        self.grant(n, True)
        self.run_ticks(n, 0.5)
        self.assertTrue(nonzero(n))
        n._cmd_vel_pub.msgs.clear()
        for _ in range(5):
            self.grant(n, False)
            self.run_ticks(n, 0.1)
        self.assertEqual(nonzero(n), [])

    def test_grant_going_stale_stops_scan(self):
        n = self.scanning()
        self.grant(n, True)
        self.run_ticks(n, 0.5)
        self.run_ticks(n, 1.5)  # 受信が1秒を超えて途絶える
        n._cmd_vel_pub.msgs.clear()
        self.run_ticks(n, 1.0)
        self.assertEqual(nonzero(n), [])

    def test_wiggle_needs_grant(self):
        n = self.scanning()
        n._idle_wiggle_enabled = True
        self.grant(n, False)
        self.run_ticks(n, 0.5)
        self.assertEqual(nonzero(n), [])
        self.grant(n, True)
        self.run_ticks(n, 0.3)
        self.assertTrue(nonzero(n))

    def test_direct_approach_fallback_needs_grant(self):
        n = self.make(crowd="calm", target=False)
        n.enable_direct_cmd_vel = True
        self.assertEqual(n._approach_target(None), "idle")
        self.assertTrue(nonzero(n))
        n._cmd_vel_pub.msgs.clear()
        self.grant(n, False)
        n._approach_target(None)
        self.assertEqual(nonzero(n), [])

    def test_zero_velocity_still_published_without_grant(self):
        n = self.make(crowd="calm", target=False)
        n._clock.t += 3
        for _ in range(5):
            n._on_near_person_stop(_BoolMsg(True))
            self.grant(n, False)
            self.run_ticks(n, 0.1)
        self.assertGreaterEqual(len(zero_twists(n)), 5)  # 安全側のゼロ速度は許可と無関係

    def test_requires_grant_false_ignores_grant(self):
        n = self.scanning()
        n._direct_grant = bd.GrantTracker("director_direct", 1.0, required=False)
        self.grant(n, False)
        self.run_ticks(n, 0.5)
        self.assertTrue(nonzero(n))


@unittest.skipIf(fe is None, f"frontier_explorer import失敗: {tea._FE_ERR}")
class ExplorerUserCancelTests(unittest.TestCase):
    def make(self, hold_sec=10.0):
        orig = fe.PoseStamped
        fe.PoseStamped = tbr._Pose
        self.addCleanup(lambda: setattr(fe, "PoseStamped", orig))
        n = object.__new__(fe.FrontierExplorer)
        n._clock = tbr._Clock()
        n.get_clock = lambda: n._clock
        n.get_logger = lambda: tbr._Log()
        n._goal_pub = tbr._Pub()
        n._enabled = True
        n._hardware_estop = n._soft_estop = False
        n._manual_override_raw = False
        n._last_manual_override_time = None
        n._latest_map = object()
        n._get_pose = lambda: (1.0, 1.0)
        n._find_frontiers = lambda g, r: [(1.0, 4.0, 10)]
        n._boundary = {"enabled": False}
        n._boundary_rect = None
        n._keepout_zones = []
        n._blacklist = []
        n.blacklist_radius_m = 0.5
        n.blacklist_sec = 120.0
        n.person_avoid_radius_m = 1.0
        n.person_avoid_timeout_sec = 2.0
        n.goal_timeout_sec = 25.0
        n.goal_reached_tolerance_m = 0.4
        n._tracked_person = None
        n._tracked_person_receive_time = None
        n._current_goal = None
        n._goal_sent_time = None
        n._status = "exploring"
        n._published = []
        n._publish_status = lambda extra=None: n._published.append(extra)
        n._user_cancel = fe.UserCancelHold(hold_sec)
        return n

    def tick(self, n, seconds):
        for _ in range(int(seconds)):
            n._clock.t += 1.0
            n._tick()

    def test_goal_issued_normally(self):
        n = self.make()
        self.tick(n, 1)
        self.assertEqual(len(n._goal_pub.msgs), 1)

    def test_user_cancel_pauses_then_resumes(self):
        n = self.make()
        self.tick(n, 1)
        sent = len(n._goal_pub.msgs)
        n._on_user_cancel(_BoolMsg(True))
        self.assertIsNone(n._current_goal)
        self.tick(n, 9)
        self.assertEqual(len(n._goal_pub.msgs), sent)  # 9秒間は出し直さない
        self.assertEqual(n._status, "paused")
        self.assertEqual(n._published[-1], {"reason": "user_cancel"})
        self.tick(n, 2)
        self.assertGreater(len(n._goal_pub.msgs), sent)  # 10秒を過ぎたら再開
        self.assertEqual(n._status, "exploring")

    def test_hold_zero_disables(self):
        n = self.make(hold_sec=0.0)
        self.tick(n, 1)
        n._on_user_cancel(_BoolMsg(True))
        self.tick(n, 3)
        self.assertGreaterEqual(len(n._goal_pub.msgs), 2)


if __name__ == "__main__":
    unittest.main(verbosity=1)
