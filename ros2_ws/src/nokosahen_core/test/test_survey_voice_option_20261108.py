"""アンケート設問3「近づいてきたとき、どう感じましたか？」に、選択肢「声が聞こえた」を追加した(2026-11-08、ユーザーの依頼)。

- 現行版 r2 の選択肢に入っている(順序は「じゃまに感じた」の次、「近づいてこなかった」の前)
- 旧版 r1 は変えない(古い画面の回答を検証するためだけに残している)
- 「声が聞こえた」を含む回答が保存できる。最大2つ・重複・未知の語の拒否は、これまでどおり
- それまでの回答(選択肢が今の選択肢の部分集合)は、そのまま有効
"""
import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core import display_survey as dsv  # noqa: E402

NEW = "声が聞こえた"


def _q(defn, qid):
    return next(q for q in defn["questions"] if q["id"] == qid)


def good(**over):
    rec = {"response_id": "resp-voice-0001-abcdef", "submitted_at": "2026-11-08T01:02:03.000Z", "survey_version": "r2",
           "mode": "robot", "condition": "展示 (高専祭)", "location": "食堂入口", "duration_sec": 12,
           "answers": {"impression": 4, "noticed_by": "声が聞こえたから", "approach_feel": ["楽しい"],
                       "want_to_move": "やや思った", "visitor_type": "大人"}}
    rec.update(over)
    return rec


class VoiceOptionTests(unittest.TestCase):
    def test_r2_has_the_new_option_in_place(self):
        opts = _q(dsv.SURVEY_DEF, "approach_feel")["options"]
        self.assertEqual(opts, ["楽しい", "気にならない", "少しこわい", "じゃまに感じた", NEW, "近づいてこなかった"])
        self.assertEqual(_q(dsv.SURVEY_DEF, "approach_feel")["maxSelect"], 2)

    def test_r1_is_unchanged(self):
        self.assertEqual(_q(dsv.SURVEY_DEF_R1, "approach_feel")["options"],
                         ["楽しい", "気にならない", "少しこわい", "じゃまに感じた", "近づいてこなかった"])

    def test_version_name_kept(self):
        self.assertEqual(dsv.SURVEY_DEF["version"], "r2")

    def test_answer_with_new_option_is_accepted(self):
        for chosen in ([NEW], [NEW, "楽しい"], ["気にならない", NEW]):
            p = good()
            p["answers"] = dict(p["answers"], approach_feel=chosen)
            rec, err = dsv.validate_response(p)
            self.assertIsNotNone(rec, (chosen, err))
            # 保存される順序は選択肢の定義の順(送られた順ではない)。内容が同じであることを確かめる
            order = _q(dsv.SURVEY_DEF, "approach_feel")["options"]
            self.assertEqual(rec["answers"]["approach_feel"], sorted(chosen, key=order.index))

    def test_limits_still_apply(self):
        for bad in ([NEW, "楽しい", "気にならない"], [NEW, NEW], NEW, ["声がきこえた"], []):
            p = good()
            p["answers"] = dict(p["answers"], approach_feel=bad)
            rec, err = dsv.validate_response(p)
            self.assertIsNone(rec, bad)

    def test_old_answers_are_still_valid(self):
        for chosen in (["楽しい", "気にならない"], ["少しこわい"], ["じゃまに感じた", "楽しい"], ["近づいてこなかった"]):
            p = good()
            p["answers"] = dict(p["answers"], approach_feel=chosen)
            rec, err = dsv.validate_response(p)
            self.assertIsNotNone(rec, (chosen, err))

    def test_new_option_not_accepted_in_r1(self):
        p = copy.deepcopy(good())
        p["survey_version"] = "r1"
        p["answers"] = {"noticed_by": "声が聞こえたから", "impression": 3, "approach_feel": [NEW], "visitor_type": "大人"}
        rec, err = dsv.validate_response(p)
        self.assertIsNone(rec)


if __name__ == "__main__":
    unittest.main()
