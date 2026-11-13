"""迂回の処理の状態機械(detour_machine.py)のテスト(2026-10-25)。ROS不要。
実行: python3 -I ros2_ws/src/nokosahen_core/test/test_detour_machine.py
"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core import detour_machine as dm  # noqa: E402

GOAL = (3.0, 1.0)


def kinds(acts):
    return [a[0] for a in acts]


class Base(unittest.TestCase):
    def setUp(self):
        self.m = dm.DetourMachine(max_attempts=2, settle_sec=1.0, plan_timeout_sec=5.0, total_timeout_sec=60.0)
        self.t = 0.0

    def start_and_settle(self):
        self.assertEqual(self.m.start(GOAL, self.t), [])
        self.assertTrue(self.m.active)
        self.assertEqual(self.m.tick(self.t + 0.5), [])      # まだ1秒待つ
        self.t += 1.0
        acts = self.m.tick(self.t)
        self.assertEqual(acts, [("plan", "direct", "goal", 0)])
        return acts

    def direct_clear(self):
        self.start_and_settle()
        self.assertEqual(self.m.plan_result(self.m.sid, "direct", True, self.t), [("check", "direct", 0)])
        return self.m.verdict(self.m.sid, True, self.t)

    def to_candidates(self, n=3):
        self.start_and_settle()
        self.assertEqual(self.m.plan_result(self.m.sid, "direct", True, self.t), [("check", "direct", 0)])
        self.assertEqual(self.m.verdict(self.m.sid, False, self.t, "collision"), [("candidates",)])
        return self.m.candidates(self.m.sid, n, self.t)


class NormalFlow(Base):
    def test_direct_path_clear_sends_original_goal_and_finishes(self):
        self.assertEqual(self.direct_clear(), [("send", "goal", 0)])
        self.assertEqual(self.m.attempts, 1)
        self.assertEqual(self.m.goal_done(self.m.sid, 4, self.t), [("done",)])
        self.assertFalse(self.m.active)

    def test_wait_before_planning(self):
        self.m.start(GOAL, 10.0)
        self.assertEqual(self.m.tick(10.99), [])
        self.assertEqual(self.m.tick(11.0)[0][0], "plan")

    def test_no_path_goes_to_candidates(self):
        self.start_and_settle()
        self.assertEqual(self.m.plan_result(self.m.sid, "direct", False, self.t), [("candidates",)])


class Waypoint(Base):
    def test_waypoint_adopted_and_then_goal_sent(self):
        self.assertEqual(self.to_candidates(), [("plan", "wp_a", "wp", 0)])
        sid = self.m.sid
        self.assertEqual(self.m.plan_result(sid, "wp_a", True, self.t), [("plan", "wp_b", "goal", 0)])
        self.assertEqual(self.m.plan_result(sid, "wp_b", True, self.t), [("check", "wp", 0)])
        self.assertEqual(self.m.verdict(sid, True, self.t), [("send", "wp", 0)])
        self.assertEqual(self.m.attempts, 1)
        # 中継点に着いたら元の目的地を送る(attemptsは増えない)
        self.assertEqual(self.m.goal_done(sid, 4, self.t), [("send", "goal", 0)])
        self.assertEqual(self.m.attempts, 1)
        self.assertEqual(self.m.goal_done(sid, 4, self.t), [("done",)])

    def test_first_candidate_blocked_second_adopted(self):
        self.to_candidates()
        sid = self.m.sid
        self.m.plan_result(sid, "wp_a", True, self.t)
        self.m.plan_result(sid, "wp_b", True, self.t)
        self.assertEqual(self.m.verdict(sid, False, self.t, "wp_collision"), [("plan", "wp_a", "wp", 1)])
        # 2つ目は中継点への経路が出ない -> 3つ目へ
        self.assertEqual(self.m.plan_result(sid, "wp_a", False, self.t), [("plan", "wp_a", "wp", 2)])
        self.m.plan_result(sid, "wp_a", True, self.t)
        self.m.plan_result(sid, "wp_b", True, self.t)
        self.assertEqual(self.m.verdict(sid, True, self.t), [("send", "wp", 2)])

    def test_wp_b_plan_failure_goes_to_next_candidate(self):
        self.to_candidates()
        sid = self.m.sid
        self.m.plan_result(sid, "wp_a", True, self.t)
        self.assertEqual(self.m.plan_result(sid, "wp_b", False, self.t), [("plan", "wp_a", "wp", 1)])


class Failure(Base):
    def test_all_candidates_blocked_fails_without_sending(self):
        self.to_candidates(2)
        sid = self.m.sid
        acts = []
        for _ in range(2):
            self.m.plan_result(sid, "wp_a", True, self.t)
            self.m.plan_result(sid, "wp_b", True, self.t)
            acts = self.m.verdict(sid, False, self.t)
        self.assertEqual(acts, [("fail", "no_clear_route")])
        self.assertFalse(self.m.active)
        self.assertEqual(self.m.attempts, 0)   # 一度も送っていない(=後退も走行もしていない)

    def test_no_candidates_fails(self):
        self.start_and_settle()
        self.m.plan_result(self.m.sid, "direct", False, self.t)
        self.assertEqual(self.m.candidates(self.m.sid, 0, self.t), [("fail", "no_candidates")])

    def test_plan_timeout_counts_as_failure(self):
        self.start_and_settle()
        self.assertEqual(self.m.tick(self.t + 4.9), [])
        self.assertEqual(self.m.tick(self.t + 5.1), [("candidates",)])
        self.assertEqual(self.m.tick(self.t + 11.0), [("fail", "no_candidates")])

    def test_total_timeout(self):
        self.m.start(GOAL, 0.0)
        self.assertEqual(self.m.tick(61.0), [("fail", "timeout")])

    def test_send_failed_gives_up(self):
        self.direct_clear()
        self.assertEqual(self.m.send_failed(self.m.sid, "rotate_not_granted"), [("fail", "rotate_not_granted")])
        self.assertFalse(self.m.active)


class Attempts(Base):
    def test_resent_goal_abort_retries_then_exhausts(self):
        self.direct_clear()
        sid = self.m.sid
        # 1回目の送信が中止 -> 迂回の処理は新しく始めず(sidそのまま)、最初からやり直す
        self.assertEqual(self.m.goal_done(sid, 6, self.t), [])
        self.assertEqual(self.m.sid, sid)
        self.assertEqual(self.m.state, dm.SETTLE)
        self.t += 1.0
        self.assertEqual(self.m.tick(self.t)[0][:2], ("plan", "direct"))
        self.m.plan_result(sid, "direct", True, self.t)
        self.assertEqual(self.m.verdict(sid, True, self.t), [("send", "goal", 0)])
        self.assertEqual(self.m.attempts, 2)
        # 2回目も中止 -> attemptsの上限
        self.assertEqual(self.m.goal_done(sid, 6, self.t), [("fail", "attempts_exhausted")])
        self.assertFalse(self.m.active)

    def test_waypoint_leg_abort_consumes_the_same_attempt(self):
        self.to_candidates()
        sid = self.m.sid
        self.m.plan_result(sid, "wp_a", True, self.t)
        self.m.plan_result(sid, "wp_b", True, self.t)
        self.m.verdict(sid, True, self.t)
        self.m.goal_done(sid, 4, self.t)             # 中継点に着いた
        self.assertEqual(self.m.goal_done(sid, 6, self.t), [])   # 目的地への最後の区間が中止 -> やり直し(attempts 1 < 2)
        self.assertEqual(self.m.state, dm.SETTLE)

    def test_max_attempts_one(self):
        m = dm.DetourMachine(max_attempts=1)
        m.start(GOAL, 0.0)
        m.tick(1.0)
        m.plan_result(m.sid, "direct", True, 1.0)
        m.verdict(m.sid, True, 1.0)
        self.assertEqual(m.goal_done(m.sid, 6, 2.0), [("fail", "attempts_exhausted")])


class CancelAndStale(Base):
    def test_cancel_in_every_state_discards(self):
        for steps in range(0, 4):
            m = dm.DetourMachine()
            m.start(GOAL, 0.0)
            if steps >= 1:
                m.tick(1.0)
            if steps >= 2:
                m.plan_result(m.sid, "direct", True, 1.0)
            if steps >= 3:
                m.verdict(m.sid, True, 1.0)       # 送信中
            self.assertEqual(m.cancel("estop"), [("cancelled", "estop")])
            self.assertFalse(m.active)
            self.assertEqual(m.tick(100.0), [])   # 取り消し後は何もしない

    def test_stale_events_after_cancel_are_ignored(self):
        self.start_and_settle()
        old = self.m.sid
        self.m.cancel("new_goal")
        self.assertEqual(self.m.plan_result(old, "direct", True, self.t), [])
        self.assertEqual(self.m.verdict(old, True, self.t), [])
        self.assertEqual(self.m.goal_done(old, 4, self.t), [])
        self.assertFalse(self.m.active)
        # 新しい処理を始めたあとに、古いsidの結果が来ても無視
        self.m.start(GOAL, self.t)
        self.m.tick(self.t + 1.0)
        self.assertEqual(self.m.plan_result(old, "direct", True, self.t), [])
        self.assertEqual(self.m.state, dm.PLAN_DIRECT)

    def test_wrong_tag_or_state_ignored(self):
        self.start_and_settle()
        self.assertEqual(self.m.plan_result(self.m.sid, "wp_a", True, self.t), [])
        self.assertEqual(self.m.verdict(self.m.sid, True, self.t), [])
        self.assertEqual(self.m.candidates(self.m.sid, 3, self.t), [])
        self.assertEqual(self.m.goal_done(self.m.sid, 4, self.t), [])

    def test_goal_canceled_status_is_not_a_failure(self):
        self.direct_clear()
        self.assertEqual(self.m.goal_done(self.m.sid, 5, self.t), [("cancelled", "goal_canceled")])
        self.assertFalse(self.m.active)

    def test_no_double_run(self):
        self.m.start(GOAL, 0.0)
        sid = self.m.sid
        self.assertEqual(self.m.start((9.0, 9.0), 0.1), [])
        self.assertEqual(self.m.sid, sid)
        self.assertEqual(self.m.goal_xy, GOAL)
        self.assertEqual(self.m.rejected, 1)

    def test_idle_machine_ignores_everything(self):
        m = dm.DetourMachine()
        self.assertEqual(m.tick(5.0), [])
        self.assertEqual(m.cancel(), [])
        self.assertEqual(m.goal_done(0, 6, 0.0), [])
        self.assertEqual(m.send_failed(0, "x"), [])


if __name__ == "__main__":
    unittest.main(verbosity=1)
