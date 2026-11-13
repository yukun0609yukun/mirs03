"""動きの重複対策の純粋な部品(motion_hold_logic.py)と、その配線の確認(2026-10-23、B2/B3/B6)。ROS不要。
実行: python3 -I ros2_ws/src/nokosahen_core/test/test_motion_hold_logic.py
"""
import json
import re
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core.motion_hold_logic import GrantTracker, HoldCancelEdge, UserCancelHold  # noqa: E402

CORE = Path(__file__).resolve().parents[1] / "nokosahen_core"


def src(name):
    return (CORE / name).read_text(encoding="utf-8").replace("\r\n", "\n")


class HoldCancelEdgeTests(unittest.TestCase):
    def test_only_first_tick_of_a_hold_cancels(self):
        e = HoldCancelEdge(once=True)
        self.assertEqual([e.should_cancel("NEAR_PERSON_HOLD") for _ in range(5)], [True, False, False, False, False])

    def test_switching_to_another_hold_cancels_once_more(self):
        e = HoldCancelEdge()
        self.assertTrue(e.should_cancel("SURVEY_ACTIVE_HOLD"))
        self.assertFalse(e.should_cancel("SURVEY_ACTIVE_HOLD"))
        self.assertTrue(e.should_cancel("NEAR_PERSON_HOLD"))
        self.assertFalse(e.should_cancel("NEAR_PERSON_HOLD"))

    def test_leaving_the_hold_rearms(self):
        e = HoldCancelEdge()
        self.assertTrue(e.should_cancel("OUT_OF_BOUNDS_HOLD"))
        e.left_hold()
        self.assertTrue(e.should_cancel("OUT_OF_BOUNDS_HOLD"))

    def test_disabled_keeps_old_every_tick_behaviour(self):
        e = HoldCancelEdge(once=False)
        self.assertEqual([e.should_cancel("CAMERA_OBSTACLE_HOLD") for _ in range(3)], [True, True, True])


class UserCancelHoldTests(unittest.TestCase):
    def test_holds_for_the_configured_time(self):
        h = UserCancelHold(10.0)
        self.assertFalse(h.active(0.0))
        h.on_user_cancel(100.0)
        self.assertTrue(h.active(100.0))
        self.assertTrue(h.active(109.9))
        self.assertFalse(h.active(110.0))

    def test_new_cancel_extends(self):
        h = UserCancelHold(10.0)
        h.on_user_cancel(0.0)
        h.on_user_cancel(8.0)
        self.assertTrue(h.active(17.0))
        self.assertFalse(h.active(18.0))

    def test_zero_disables(self):
        for sec in (0.0, -1.0):
            h = UserCancelHold(sec)
            h.on_user_cancel(0.0)
            self.assertFalse(h.active(0.0))

    def test_clear(self):
        h = UserCancelHold(10.0)
        h.on_user_cancel(0.0)
        h.clear()
        self.assertFalse(h.active(1.0))


def grant(allowed):
    return json.dumps({"owner": "idle", "allow": {"director_direct": allowed}, "stamp": 0})


class GrantTrackerTests(unittest.TestCase):
    def test_never_received_is_allowed(self):
        self.assertTrue(GrantTracker("director_direct").ok(1000.0))

    def test_follows_the_grant(self):
        g = GrantTracker("director_direct")
        g.on_message(grant(True), 10.0)
        self.assertTrue(g.ok(10.5))
        g.on_message(grant(False), 10.6)
        self.assertFalse(g.ok(10.7))

    def test_stale_after_one_second_is_denied(self):
        g = GrantTracker("director_direct", stale_sec=1.0)
        g.on_message(grant(True), 10.0)
        self.assertTrue(g.ok(11.0))
        self.assertFalse(g.ok(11.01))

    def test_missing_name_or_broken_message_is_denied(self):
        g = GrantTracker("director_direct")
        g.on_message(json.dumps({"allow": {"wall_escape": True}}), 1.0)
        self.assertFalse(g.ok(1.1))
        g.on_message("not json", 2.0)
        self.assertFalse(g.ok(2.1))

    def test_not_required_always_allowed(self):
        g = GrantTracker("director_direct", required=False)
        g.on_message(grant(False), 1.0)
        self.assertTrue(g.ok(1.1))
        self.assertTrue(g.ok(100.0))


class DirectorWiring(unittest.TestCase):
    D = src("behavior_director.py")

    def test_hold_modes_do_not_publish_cancel_every_tick(self):
        for mode in ("SURVEY_ACTIVE_HOLD", "NEAR_PERSON_HOLD", "OUT_OF_BOUNDS_HOLD", "KEEPOUT_ZONE_HOLD", "CAMERA_OBSTACLE_HOLD"):
            m = re.search(r'mode = "%s"\n(.*?)self\._reset_crowd_scan' % mode, self.D, re.S)
            self.assertIsNotNone(m, mode)
            body = m.group(1)
            # ゼロ速度は毎tick、取り消しは最初の1回(_hold_cancel)
            self.assertIn("self._publish_stop()", body, mode)
            self.assertIn("self._hold_cancel(mode)", body, mode)
            self.assertNotIn("self._publish_cancel_goal()", body, mode)

    def test_publish_cancel_goal_only_via_helpers(self):
        calls = [m.start() for m in re.finditer(r"self\._publish_cancel_goal\(\)", self.D)]
        self.assertEqual(len(calls), 2)  # _hold_cancel と _cancel_goal_once の中だけ

    def test_hold_cancel_param_and_rearm(self):
        self.assertIn('declare_parameter("hold_cancel_once", True)', self.D)
        self.assertIn("self._hold_edge.left_hold()", self.D)

    def test_user_cancel_subscription_and_goal_suppression(self):
        self.assertIn('"/nokosahen/user_cancel"', self.D)
        self.assertIn('declare_parameter("user_cancel_hold_sec", 10.0)', self.D)
        for fn in ("def _publish_entrance_goal", "def _publish_pin_goal", "def _approach_target"):
            i = self.D.index(fn)
            j = self.D.index("\n    def ", i + 10)
            self.assertIn("self._goals_suppressed()", self.D[i:j], fn)
    def test_direct_cmd_vel_goes_through_the_grant(self):
        self.assertIn('"/nokosahen/motion_grant"', self.D)
        self.assertIn('declare_parameter("direct_cmd_vel_requires_grant", True)', self.D)
        self.assertIn('"director_direct"', self.D)
        # _publish_stop(ゼロ速度)以外で、_cmd_vel_pub.publishを直接呼ぶのは_publish_directの中だけ
        direct = [m.start() for m in re.finditer(r"self\._cmd_vel_pub\.publish\(", self.D)]
        self.assertEqual(len(direct), 2)  # _publish_stop と _publish_direct
        i = self.D.index("def _publish_direct")
        self.assertTrue(any(i < p < i + 600 for p in direct))
        # 見回しの旋回・向き直り・直進、揺動、直接接近、向く動作(2026-11-03)、向くの見回し(2026-11-04)の7か所。いずれも許可(grant)を通る
        self.assertEqual(self.D.count("self._publish_direct(twist)"), 7)


class ExplorerWiring(unittest.TestCase):
    E = src("frontier_explorer.py")

    def test_explorer_pauses_on_user_cancel(self):
        self.assertIn('"/nokosahen/user_cancel"', self.E)
        self.assertIn('declare_parameter("user_cancel_hold_sec", 10.0)', self.E)
        i = self.E.index("def _tick")
        body = self.E[i:]
        self.assertIn("self._user_cancel.active(", body)
        self.assertLess(body.index("self._user_cancel.active("), body.index("self._goal_pub.publish"))


class DashboardWiring(unittest.TestCase):
    def test_dashboard_cancel_also_publishes_user_cancel(self):
        s = src("dashboard_common.py")
        self.assertIn('"/nokosahen/user_cancel"', s)
        i = s.index("def cancel(self) -> bool:")
        j = s.index("\nclass ", i)
        self.assertIn("user_cancel_pub.publish", s[i:j])


if __name__ == "__main__":
    unittest.main(verbosity=1)
