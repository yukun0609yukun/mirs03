"""起動時の表示モード復元が、アンケート回答中は行われないこと(B10, 2026-10-23)のテスト。ROS不要。
実行: python3 -I ros2_ws/src/nokosahen_core/test/test_ephemeral_survey_guard.py
"""
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core import ephemeral_overrides as eo  # noqa: E402


class Survey:
    configured = True

    def __init__(self, state):
        self._state = state
        self.mode_calls = []
        self.fresh_args = []

    def get_state(self, fresh=False):
        self.fresh_args.append(fresh)
        return self._state

    def set_mode(self, d):
        self.mode_calls.append(d)
        return 200, {"ok": True}

    def set_ocr_mock(self, d):
        return 200, {"ok": True}


def ok_state(**kw):
    st = {"mode": "survey", "active": False, "session_state": "idle"}
    st.update(kw)
    return {"ok": True, "configured": True, "state": st}


class SurveyGuardTests(unittest.TestCase):
    def restore(self, survey, **kw):
        return eo.build_restorers(survey_admin=survey, **kw)[eo.RESTORE_SURVEY_MODE]("customer")

    def test_idle_restores(self):
        s = Survey(ok_state())
        self.assertTrue(self.restore(s))
        self.assertEqual(s.mode_calls, [{"mode": "customer"}])
        self.assertEqual(s.fresh_args, [True])  # キャッシュを使わず確認する

    def test_active_does_not_restore(self):
        s = Survey(ok_state(active=True))
        self.assertFalse(self.restore(s))
        self.assertEqual(s.mode_calls, [])

    def test_answering_session_does_not_restore(self):
        s = Survey(ok_state(session_state="answering"))
        self.assertFalse(self.restore(s))
        self.assertEqual(s.mode_calls, [])

    def test_saved_session_restores(self):
        self.assertTrue(self.restore(Survey(ok_state(session_state="saved"))))

    def test_unknown_state_does_not_restore(self):
        for st in ({"ok": False, "message": "届きません"}, {"ok": True}, None):
            s = Survey(st)
            self.assertFalse(self.restore(s))
            self.assertEqual(s.mode_calls, [])

    def test_guard_off_param_restores_even_when_answering(self):
        s = Survey(ok_state(active=True))
        self.assertTrue(self.restore(s, guard_survey_active=False))
        self.assertEqual(len(s.mode_calls), 1)

    def test_guard_off_env(self):
        old = os.environ.get(eo.SURVEY_GUARD_ENV)
        os.environ[eo.SURVEY_GUARD_ENV] = "0"
        try:
            s = Survey(ok_state(active=True))
            self.assertTrue(self.restore(s))
        finally:
            if old is None:
                os.environ.pop(eo.SURVEY_GUARD_ENV, None)
            else:
                os.environ[eo.SURVEY_GUARD_ENV] = old

    def test_journal_kept_until_answer_finished(self):
        # 回答中はジャーナルに残り、回答が終わったあとの再試行で戻る
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "j.json")
            e1 = eo.EphemeralOverrides(path)
            e1.record("display.survey_mode", "survey", "op", restore=eo.RESTORE_SURVEY_MODE, baseline="customer")
            e2 = eo.EphemeralOverrides(path)
            s = Survey(ok_state(active=True))
            rs = eo.build_restorers(survey_admin=s)
            self.assertEqual(e2.restore_pending(rs), (0, 1))
            self.assertEqual(s.mode_calls, [])
            s._state = ok_state()
            self.assertEqual(e2.restore_pending(rs), (1, 0))
            self.assertEqual(s.mode_calls, [{"mode": "customer"}])


if __name__ == "__main__":
    unittest.main()
