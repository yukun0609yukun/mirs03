"""文化祭前の確認(2026-11-06): 「人の方を向く」だけON・放置運用で、次の点が現在のコードで成り立つかの確認テスト。

1. 真正面(近接停止0.5 m以内)に来られたら止まる(旋回は免除の安全条件つきでだけ)
2. アンケート(iPad)の回答中は旋回も含めて完全に止まり、終わったら「向く」に戻る(近接停止の免除経路も迂回しない)
(3. 権限の確認は test_festival_roles_20261106.py)
ROS・ロボット・2台目には一切触れない(PC上のみ)。
実行: python ros2_ws/src/nokosahen_core/test/test_festival_checks_20261106.py
"""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import face_person_sim as fps  # noqa: E402
import test_survey_monitor  # noqa: E402,F401  (rclpyが無い環境用のスタブを入れる)

try:
    from nokosahen_core.survey_monitor import SurveyGate  # noqa: E402
    from nokosahen_core.display_hub import AnswerSession  # noqa: E402
except Exception:  # pragma: no cover
    SurveyGate = AnswerSession = None


class FestivalSim(fps.Sim):
    """近接停止フラグを人との距離から作り(単独クラスタ0.5 m / 脚ペア0.8 m)、アンケート回答中の通知(survey_active)を時刻で出し入れする。"""

    def __init__(self, near_thr=0.5, survey_from=None, survey_to=None, **kw):
        super().__init__(**kw)
        self.near_thr, self.survey_from, self.survey_to = near_thr, survey_from, survey_to

    def step(self):
        n = self.n
        near = min((math.hypot(p.x - self.robot["x"], p.y - self.robot["y"]) for p in self.people), default=99.0)
        n._near_person_stop_detected = near <= self.near_thr
        t = self.t + fps.DT  # この step で進んだ後の時刻
        if self.survey_from is not None and self.survey_from <= t and (self.survey_to is None or t < self.survey_to):
            n._survey_active = True
            n._last_survey_active_time = n._clock.now()  # survey_monitorが5 Hzで出し続けるのを模す
        elif self.survey_from is not None:
            n._survey_active = False
        super().step()

    def slice(self, t0, t1):
        return [r for r in self.log if t0 < r["t"] <= t1 + 1e-9]


def moved(rows):
    return any(abs(r["vx"]) > 1e-9 or abs(r["wz"]) > 1e-9 for r in rows)


class NearStopTable(unittest.TestCase):
    """確認1: (人の位置, 近接停止の判定距離) ごとに、止まる/向く/前進しない。"""

    def run_case(self, x, y, thr, sec=6):
        s = FestivalSim(near_thr=thr, people=[fps.Person(x, y)])
        s.run(sec)
        s.close()
        return s

    def test_front_within_half_meter_stops(self):
        for d in (0.30, 0.45, 0.50):
            for thr in (0.5, 0.8):
                s = self.run_case(d, 0.0, thr)
                self.assertEqual(set(s.modes()), {"NEAR_PERSON_HOLD"}, (d, thr, s.modes()))
                self.assertFalse(moved(s.log), (d, thr))

    def test_side_within_half_meter_stops(self):
        for thr in (0.5, 0.8):
            s = self.run_case(0.0, 0.40, thr)
            self.assertEqual(set(s.modes()), {"NEAR_PERSON_HOLD"})
            self.assertFalse(moved(s.log))

    def test_beyond_half_meter_never_drives_forward(self):
        for (x, y) in ((0.6, 0.0), (0.53, 0.53), (0.0, 0.6), (0.9, 0.0)):
            for thr in (0.5, 0.8):
                s = self.run_case(x, y, thr)
                self.assertEqual(s.max_linear(), 0.0, (x, y, thr))

    def test_front_0_6_with_leg_pair_is_exempt_but_does_not_move(self):
        # 0.55 m 以上なら免除して向く判定になるが、真正面なので回らない(静止)
        s = self.run_case(0.6, 0.0, 0.8)
        self.assertIn("FACE_PERSON", s.modes())
        self.assertFalse(moved(s.slice(1.0, 6.0)))

    def test_side_0_6_with_leg_pair_turns_in_place(self):
        # 横0.6 m(脚ペアで近接停止が出る距離): 免除して、その場で向く(前進しない)
        s = self.run_case(0.0, 0.6, 0.8, sec=8)
        self.assertGreater(s.modes().get("FACE_PERSON", 0), 0)
        self.assertEqual(s.max_linear(), 0.0)
        self.assertGreater(abs(math.degrees(s.robot["yaw"])), 40)


class SurveyStopTests(unittest.TestCase):
    """確認2: アンケート回答中は旋回も含めて完全に止まる。近接停止の免除経路(_face_only_step)も迂回しない。"""

    def test_turn_stops_when_survey_starts_and_resumes_after(self):
        p = fps.Person(2.0, 1.2)
        s = FestivalSim(people=[p], survey_from=2.0, survey_to=6.0)
        s.run(12)
        self.assertTrue(moved(s.slice(0.0, 1.9)), "開始前は旋回している")
        hold = s.slice(2.0, 6.0)
        self.assertEqual({r["mode"] for r in hold}, {"SURVEY_ACTIVE_HOLD"})
        self.assertFalse(moved(hold), "回答中は旋回も前進もゼロ")
        y0 = hold[0]["yaw"]
        self.assertTrue(all(abs(r["yaw"] - y0) < 1e-9 for r in hold))
        after = s.slice(6.0, 12.0)
        self.assertIn("FACE_PERSON", {r["mode"] for r in after}, "終わったら向くに戻る")
        err = abs(s.heading_error_deg(fps.world_bearing((0, 0), p)))
        self.assertLess(err, 8.0, "最終的に人の方を向く")
        s.close()

    def test_sweep_stops_during_survey(self):
        s = FestivalSim(people=[], survey_from=1.0, survey_to=5.0)
        s.run(9)
        self.assertEqual({r["mode"] for r in s.slice(1.0, 5.0)}, {"SURVEY_ACTIVE_HOLD"})
        self.assertFalse(moved(s.slice(1.0, 5.0)))
        self.assertIn("FACE_SWEEP", {r["mode"] for r in s.slice(5.0, 9.0)})
        s.close()

    def test_near_exempt_turn_path_does_not_bypass_survey(self):
        # 近接停止が出ていて(脚ペア0.8 m)、免除して旋回している最中にアンケートが始まる
        p = fps.Person(0.0, 0.7)
        s = FestivalSim(near_thr=0.8, people=[p], survey_from=2.0, survey_to=7.0)
        s.run(12)
        self.assertTrue(moved(s.slice(0.5, 1.9)), "免除で旋回している")
        hold = s.slice(2.0, 7.0)
        self.assertEqual({r["mode"] for r in hold}, {"SURVEY_ACTIVE_HOLD"}, {r["mode"] for r in hold})
        self.assertFalse(moved(hold))
        s.close()

    def test_survey_during_near_hold_stays_zero(self):
        s = FestivalSim(near_thr=0.5, people=[fps.Person(0.4, 0.0)], survey_from=1.0, survey_to=4.0)
        s.run(6)
        self.assertEqual({r["mode"] for r in s.slice(1.0, 3.9)}, {"SURVEY_ACTIVE_HOLD"})
        self.assertFalse(moved(s.log))
        s.close()

    def test_feed_loss_releases_after_3s(self):
        p = fps.Person(2.0, 1.2)
        s = FestivalSim(people=[p])
        n = s.n
        s.run(1.0)
        n._survey_active = True
        n._last_survey_active_time = n._clock.now()   # 以後は届かない(survey_monitorが落ちた)
        t_on = s.t
        s.run(5.0)
        rows = [r for r in s.log if r["t"] > t_on]
        held = [r for r in rows if r["mode"] == "SURVEY_ACTIVE_HOLD"]
        self.assertTrue(held)
        self.assertLessEqual(held[-1]["t"] - t_on, 3.2)
        self.assertEqual(rows[-1]["mode"], "FACE_PERSON")
        s.close()


@unittest.skipIf(SurveyGate is None, "survey_monitor/display_hub import失敗")
class SurveyChainTiming(unittest.TestCase):
    """iPad(AnswerSession)→ survey_monitor(SurveyGate) → director までの遅れと上限(秒)を実測する。"""

    DT = 0.02
    POLL = 1 / 1.5   # survey_monitor のポーリング周期
    PUB = 1 / 5.0    # /nokosahen/survey_active の配信周期
    TICK = 0.1       # director の tick(想定10 Hz)

    def drive(self, events, total):
        clk = [0.0]
        sess = AnswerSession(clock=lambda: clk[0])
        gate = SurveyGate()
        next_poll = next_pub = 0.0
        ts, out = [], []
        fired = set()
        state = False
        while clk[0] < total:
            for et, fn in sorted(events.items()):
                if et not in fired and clk[0] >= et:
                    fired.add(et)
                    fn(sess)
            if clk[0] >= next_poll:
                next_poll += self.POLL
                gate.feed({"ok": True, "mode": "survey", "active": sess.active(), "session_state": sess.state(),
                           "saved_seq": sess.saved_seq, "count": 0, "age": 0.0, "condition": None})
            if clk[0] >= next_pub:
                next_pub += self.PUB
                state = gate.tick(clk[0])
            ts.append(clk[0])
            out.append(bool(state))
            clk[0] += self.DT
        return ts, out

    @staticmethod
    def first(ts, seq, val, after=0.0):
        return next((t for t, v in zip(ts, seq) if t >= after and v == val), None)

    def test_start_delay(self):
        ts, out = self.drive({1.0: lambda s: s.start()}, 6.0)
        d = self.first(ts, out, True, 1.0) - 1.0
        print(f"\n[遅れ] タップ(start)→survey_active=true: {d:.2f}秒 (+director tick {self.TICK}秒)")
        self.assertLess(d, self.POLL + self.PUB + 0.1)

    def test_saved_release_delay(self):
        ts, out = self.drive({1.0: lambda s: s.start(), 4.0: lambda s: s.saved()}, 12.0)
        d = self.first(ts, out, False, 4.0) - 4.0
        print(f"[遅れ] 保存完了→survey_active=false: {d:.2f}秒 (+director tick)")
        self.assertGreater(d, 1.4)      # 保存後ホールド1.5秒
        self.assertLess(d, 1.5 + self.POLL + self.PUB + 0.2)

    def test_end_release_delay(self):
        ts, out = self.drive({1.0: lambda s: s.start(), 4.0: lambda s: s.end()}, 12.0)
        d = self.first(ts, out, False, 4.0) - 4.0
        print(f"[遅れ] end(30秒放置/離脱)→false: {d:.2f}秒")
        self.assertLess(d, 1.5 + self.POLL + self.PUB + 0.2)

    def test_beat_loss_abandon_120s(self):
        ts, out = self.drive({1.0: lambda s: s.start()}, 130.0)   # 以後 beat なし
        d = self.first(ts, out, False, 2.0) - 1.0
        print(f"[上限] 通知(beat)が途絶えた場合の自動解除: {d:.1f}秒")
        self.assertAlmostEqual(d, 120.0, delta=4.0)

    def test_max_300s_even_if_beats_continue(self):
        ev = {1.0: lambda s: s.start()}
        for k in range(1, 70):
            ev[1.0 + 5.0 * k] = lambda s: s.beat()
        ts, out = self.drive(ev, 320.0)
        d = self.first(ts, out, False, 2.0) - 1.0
        print(f"[上限] beatを送り続けても最長: {d:.1f}秒")
        self.assertAlmostEqual(d, 300.0, delta=4.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
