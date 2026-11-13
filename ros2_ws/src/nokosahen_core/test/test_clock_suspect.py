"""時計が合っていない起動直後(年が2026より前。Jetsonは再起動のたびに1970年などへ戻る)の、時間帯ルール・木のテスト(2026-10-21)。

- 時刻・曜日・日付の条件を持つルールは、時計が不正なら成立しない(昼に「22:00-06:00は黙って待機」などが成立しない)
- 時刻の条件を持たないルール(混雑・残数など)は、時計が不正でも、従来どおり
- behavior tree の time_between / weekday_in / date_in も同様
- 状態(RuleState.to_json_dict)に clock_suspect が出る。時計が合えば、成立する
実行: python ros2_ws/src/nokosahen_core/test/test_clock_suspect.py
"""
import datetime
import sys
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(TEST_DIR.parent))

import test_behavior_rules as tbr  # noqa: E402
from nokosahen_core import behavior_rules as br  # noqa: E402
from nokosahen_core import behavior_tree as bt  # noqa: E402

BAD = datetime.datetime(1970, 1, 1, 12, 0, 0)   # 時計が合っていない(昼の12時)
GOOD = datetime.datetime(2026, 10, 6, 12, 0, 0)  # 合っている(火曜の昼)
BAD_NIGHT = datetime.datetime(1970, 1, 1, 23, 0, 0)


class RuleTableClockTest(unittest.TestCase):
    def test_clock_suspect_threshold(self):
        self.assertTrue(br.clock_suspect(datetime.datetime(1970, 1, 1)))
        self.assertTrue(br.clock_suspect(datetime.datetime(2025, 12, 31, 23, 59)))
        self.assertFalse(br.clock_suspect(datetime.datetime(2026, 1, 1)))

    def test_time_conditions_do_not_match_with_a_bad_clock(self):
        night = tbr.ruleset(tbr.rule(when={"time_ranges": ["22:00-06:00"]})).rules[0]
        # 1970年の「23時」「12時」のどちらも、時刻の条件は成立しない(時計が不正)
        self.assertFalse(night.matches(BAD_NIGHT, br.Context()))
        self.assertFalse(night.matches(BAD, br.Context()))
        # 時計が合っていれば、従来どおり
        self.assertTrue(night.matches(datetime.datetime(2026, 10, 6, 23, 0), br.Context()))
        self.assertFalse(night.matches(GOOD, br.Context()))
        weekday = tbr.ruleset(tbr.rule(when={"weekdays": ["thu"]})).rules[0]  # 1970-01-01は木曜日
        self.assertFalse(weekday.matches(BAD, br.Context()))
        dated = tbr.ruleset(tbr.rule(when={"dates": ["1970-01-01"]})).rules[0]
        self.assertFalse(dated.matches(BAD, br.Context()))

    def test_rules_without_time_conditions_are_unaffected(self):
        crowd = tbr.ruleset(tbr.rule(when={"crowd_level": ["crowded"]})).rules[0]
        self.assertTrue(crowd.matches(BAD, br.Context(crowd_level="crowded")))
        always = tbr.ruleset(tbr.rule(when={})).rules[0]
        self.assertTrue(always.matches(BAD, br.Context()))

    def test_engine_state_reports_clock_suspect_and_recovers(self):
        rs = tbr.ruleset(
            tbr.rule(id="quiet_night", priority=10, when={"time_ranges": ["22:00-06:00"]}, then={"policy": "stay"}),
        )
        e, fc = tbr.engine_with(rs, start=BAD_NIGHT)
        st = e.evaluate(br.Context())
        self.assertIsNone(st.active_rule_id)  # 夜に見えても、時計が不正なので、成立しない
        self.assertTrue(st.to_json_dict()["clock_suspect"])
        fc.dt = datetime.datetime(2026, 10, 6, 23, 0)  # 時計合わせのあと
        fc.advance(100)
        st = e.evaluate(br.Context())
        self.assertEqual(st.active_rule_id, "quiet_night")
        self.assertFalse(st.to_json_dict()["clock_suspect"])


class TreeClockTest(unittest.TestCase):
    """条件ノードの check() を、環境(env.dt)だけを渡して直接確かめる。"""

    def cond(self, cls, **params):
        n = object.__new__(cls)
        n.params = params
        return n

    def env(self, dt):
        import types
        return types.SimpleNamespace(dt=dt)

    def test_time_conditions_fail_with_a_bad_clock_and_work_with_a_good_one(self):
        every_day = ["mon", "tue", "wed", "thu", "fri", "sat", "sun"]
        cases = (
            (bt.TimeBetween, {"range": "00:00-24:00"}),          # 1日中成立する条件
            (bt.WeekdayIn, {"days": every_day}),                  # 毎日成立する条件
        )
        for cls, params in cases:
            c = self.cond(cls, **params)
            self.assertFalse(c.check(self.env(BAD)), cls.__name__)   # 時計が不正 → 成立しない
            self.assertTrue(c.check(self.env(GOOD)), cls.__name__)   # 時計が合っている → 従来どおり
        d = self.cond(bt.DateIn, dates=["1970-01-01", "2026-10-06"])
        self.assertFalse(d.check(self.env(BAD)))   # 1970-01-01が指定されていても、時計が不正なら成立しない
        self.assertTrue(d.check(self.env(GOOD)))

    def test_static_source_checks(self):
        t = (TEST_DIR.parent / "nokosahen_core" / "behavior_tree.py").read_text(encoding="utf-8")
        self.assertEqual(t.count("br.clock_suspect(env.dt)"), 3)  # time_between / weekday_in / date_in


if __name__ == "__main__":
    unittest.main(verbosity=1)
