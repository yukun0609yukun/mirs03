"""Jetson2で班員が行った変更(高専祭用)の取り込み(2026-10-30)のテスト。docs/jetson2_team_changes.md

実機・VOICEVOX・Ollama・スピーカーには接続しない(すべてスタブ)。確認すること:
- 話者ID・声のパラメータの既定値(班員の値)と、環境変数で話者を上書きできること
- 先頭N文への短縮(limit_sentences): 2文/境界/1文/空/句点なし/改行/0=切らない
- _process_item: 短縮が text/context の両方のOllama経路で効き、固定文・フォールバック文には効かないこと。
  履歴記録・マスターOFF確認・speak_voicevox が全経路(text/generate/context/context固定)で行われること
  (班員版の _process_item はインデント崩れでこれらが一部経路だけになっていた)
- persona_prompt.txt が高専祭版で、旧版が persona_prompt_foodhall_backup.txt に残っていること
- 班員版の事故(誤入力の文字列・関数名の誤り・デバッグprint)がソースに無いこと
"""

import os
import subprocess
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_announce_loop as base  # noqa: E402  (環境変数とsounddevice等のスタブを先に整える)

vb = base.vb
VB_DIR = Path(vb.__file__).resolve().parent


def _speaker_id_in_fresh_process(extra_env: dict, drop: tuple = ()) -> str:
    """別プロセスでvoice_bridgeを読み込み直して SPEAKER_ID を得る(環境変数の効き方を確かめるため)。"""
    env = {k: v for k, v in os.environ.items() if k not in drop}
    env.update(extra_env)
    env["BGM_ENABLED"] = "0"
    code = ("import sys,types;"
            "sd=types.ModuleType('sounddevice');sd.PortAudioError=Exception;sys.modules['sounddevice']=sd;"
            "sf=types.ModuleType('soundfile');sys.modules['soundfile']=sf;"
            f"sys.path.insert(0,{str(VB_DIR)!r});import voice_bridge as v;print(v.SPEAKER_ID)")
    out = subprocess.run([sys.executable, "-c", code], env=env, capture_output=True, text=True, timeout=120)
    return out.stdout.strip().splitlines()[-1] if out.stdout.strip() else "ERR " + out.stderr[-500:]


class DefaultsTests(unittest.TestCase):
    def test_speaker_default_is_27(self):
        self.assertEqual(_speaker_id_in_fresh_process({}, drop=("VOICEVOX_SPEAKER_ID",)), "27")

    def test_speaker_env_override_still_works(self):
        self.assertEqual(_speaker_id_in_fresh_process({"VOICEVOX_SPEAKER_ID": "3"}), "3")

    def test_voice_query_tuning(self):
        t = vb.VOICE_QUERY_TUNING
        self.assertEqual((t["speedScale"], t["pitchScale"], t["intonationScale"]), (0.76, -0.09, 2))
        self.assertEqual((t["prePhonemeLength"], t["postPhonemeLength"]), (0.08, 0.12))

    def test_max_sentences_default(self):
        self.assertEqual(vb.MAX_SENTENCES, 2)


class LimitSentencesTests(unittest.TestCase):
    def test_cuts_to_two(self):
        self.assertEqual(vb.limit_sentences("あ。い。う。え。", 2), "あ。い。")

    def test_boundaries(self):
        self.assertEqual(vb.limit_sentences("あ。い。", 2), "あ。い。")  # ちょうど2文
        self.assertEqual(vb.limit_sentences("あ。い。う。", 3), "あ。い。う。")
        self.assertEqual(vb.limit_sentences("あ。い。う。", 1), "あ。")

    def test_single_sentence_and_empty(self):
        self.assertEqual(vb.limit_sentences("こんにちは。", 2), "こんにちは。")
        self.assertEqual(vb.limit_sentences("", 2), "")
        self.assertEqual(vb.limit_sentences("  \n ", 2), "")
        self.assertEqual(vb.limit_sentences(None, 2), "")

    def test_no_period(self):
        self.assertEqual(vb.limit_sentences("ねえちょっと待っておくれ", 2), "ねえちょっと待っておくれ")
        # 2文目が句点無しで終わる場合は、そのまま残る(文の途中では切らない)
        self.assertEqual(vb.limit_sentences("あ。い", 2), "あ。い")
        self.assertEqual(vb.limit_sentences("あ。い。う", 2), "あ。い。")

    def test_other_terminators_and_newlines(self):
        self.assertEqual(vb.limit_sentences("元気かい？よかった！またね。", 2), "元気かい？よかった！")
        self.assertEqual(vb.limit_sentences("あ。\nい。\nう。", 2), "あ。い。")
        self.assertEqual(vb.limit_sentences("あ\nい\nう", 2), "あい")

    def test_zero_or_negative_means_no_cut(self):
        self.assertEqual(vb.limit_sentences("あ。い。う。", 0), "あ。い。う。")
        self.assertEqual(vb.limit_sentences("あ。い。う。", -1), "あ。い。う。")


class ProcessItemTests(unittest.TestCase):
    """_process_item を、生成・記録・再生をスタブにして直接呼ぶ。"""

    class _Master:
        def __init__(self):
            self.enabled = True

    def setUp(self):
        self.calls = {"record": [], "update": [], "speak": []}
        self._orig = (vb.generate_with_ollama, vb.generate_utterance_from_context, vb.speak_voicevox,
                      vb._record_utterance, vb._update_utterance, vb.voice_master, vb.MAX_SENTENCES)
        self.master = self._Master()
        self.llm_text = "あ。い。う。え。"
        self.llm_fallback = None  # 生成のフォールバック理由(trace["fallback_reason"])

        def gen_text(text, trace=None, settings=None):
            if self.llm_fallback:
                trace["fallback_reason"] = self.llm_fallback
            return self.llm_text

        def gen_ctx(context, trace=None, settings=None):
            if self.llm_fallback:
                trace["fallback_reason"] = self.llm_fallback
            return self.llm_text

        def speak(text, trace=None):
            self.calls["speak"].append(text)
            if trace is not None:
                trace["speak_ok"] = True

        vb.generate_with_ollama = gen_text
        vb.generate_utterance_from_context = gen_ctx
        vb.speak_voicevox = speak
        vb._record_utterance = lambda text, meta: (self.calls["record"].append((text, meta)) or 1)
        vb._update_utterance = lambda row_id, after: self.calls["update"].append((row_id, after))
        vb.voice_master = self.master
        vb.MAX_SENTENCES = 2

    def tearDown(self):
        (vb.generate_with_ollama, vb.generate_utterance_from_context, vb.speak_voicevox,
         vb._record_utterance, vb._update_utterance, vb.voice_master, vb.MAX_SENTENCES) = self._orig

    def run_item(self, item):
        item = dict(item, enqueued_mono=0.0)
        vb.SpeechQueue._process_item(item)

    def assert_full_path(self, spoken, kind):
        self.assertEqual(self.calls["speak"], [spoken])
        self.assertEqual(len(self.calls["record"]), 1)
        self.assertEqual(self.calls["record"][0][0], spoken)
        self.assertEqual(self.calls["record"][0][1]["kind"], kind)
        self.assertEqual(len(self.calls["update"]), 1)  # 再生後の追記まで行われた

    def test_text_generate_is_trimmed_recorded_spoken(self):
        self.run_item({"text": "元", "generate": True})
        self.assert_full_path("あ。い。", "text")

    def test_context_generate_is_trimmed_recorded_spoken(self):
        self.run_item({"text": "", "generate": True, "context": {"phase": "initial", "menu_items": []}})
        self.assert_full_path("あ。い。", "context")

    def test_text_without_generate_is_not_trimmed(self):
        self.run_item({"text": "一。二。三。四。", "generate": False})
        self.assert_full_path("一。二。三。四。", "text")

    def test_context_without_generate_uses_fallback_text_untouched(self):
        self.run_item({"text": "x", "generate": False,
                       "context": {"phase": "initial", "fallback_text": "甲。乙。丙。"}})
        self.assert_full_path("甲。乙。丙。", "context")

    def test_fallback_reason_text_is_not_trimmed(self):
        self.llm_fallback = "timeout"
        self.llm_text = "代替一。代替二。代替三。"
        self.run_item({"text": "元", "generate": True})
        self.assert_full_path("代替一。代替二。代替三。", "text")

    def test_single_sentence_unchanged(self):
        self.llm_text = "こんにちは。"
        self.run_item({"text": "元", "generate": True})
        self.assert_full_path("こんにちは。", "text")

    def test_max_sentences_zero_disables_trim(self):
        vb.MAX_SENTENCES = 0
        self.run_item({"text": "元", "generate": True})
        self.assert_full_path("あ。い。う。え。", "text")

    def test_master_off_before_start_does_nothing(self):
        self.master.enabled = False
        self.run_item({"text": "元", "generate": True})
        self.assertEqual((self.calls["speak"], self.calls["record"]), ([], []))

    def test_master_off_during_generation_discards(self):
        def gen_then_off(text, trace=None, settings=None):
            self.master.enabled = False
            return "あ。い。う。"
        vb.generate_with_ollama = gen_then_off
        self.run_item({"text": "元", "generate": True})
        self.assertEqual((self.calls["speak"], self.calls["record"]), ([], []))

    def test_muted_during_speak_is_recorded_as_not_spoken(self):
        def speak_muted(text, trace=None):
            self.calls["speak"].append(text)
            trace["muted"] = True
        vb.speak_voicevox = speak_muted
        self.run_item({"text": "元", "generate": True})
        self.assertEqual(len(self.calls["update"]), 1)
        self.assertEqual(self.calls["update"][0][1]["speak_ok"], 0)


class PersonaTests(unittest.TestCase):
    def test_persona_is_kousensai_version(self):
        p = vb.load_persona_prompt()
        self.assertIn("のこさへん", p)
        self.assertIn("高専祭", p)
        self.assertNotIn("お疲れ様ー、今日も来てくれてありがとねぇ。", p)

    def test_old_persona_kept_as_backup(self):
        old = (VB_DIR / "persona_prompt_foodhall_backup.txt").read_text(encoding="utf-8")
        self.assertIn("食堂のおばちゃん", old)
        self.assertIn("お疲れ様ー、今日も来てくれてありがとねぇ。", old)

    def test_default_settings_use_persona_file(self):
        self.assertEqual(vb.default_voice_settings()["persona_prompt"], vb.load_persona_prompt())


class NoTeamMistakesTests(unittest.TestCase):
    """班員版(Jetson2)にあった事故がソースに入っていないこと。"""

    def setUp(self):
        self.src = (VB_DIR / "voice_bridge.py").read_text(encoding="utf-8")

    def test_no_typos(self):
        for bad in ("エラiー", "できませiん", "builds_context_prompt_ex"):
            self.assertNotIn(bad, self.src)
        self.assertTrue(callable(vb.build_context_prompt_ex))

    def test_no_debug_prints(self):
        for bad in ('print("sdplay")', 'print("ENTER SPEAK', 'print("WAV READ")', 'print("BEFORE play_audio")'):
            self.assertNotIn(bad, self.src)


if __name__ == "__main__":
    unittest.main()
