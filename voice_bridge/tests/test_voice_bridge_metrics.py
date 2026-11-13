"""voice_bridgeの「音声設定+発話計測」(2026-10-06追加)のテスト。

実機には一切触れない。Ollama/VOICEVOXは http.server のスタブで置き換え(待ち時間を決め打ち
できるので計測値が決定的に検証できる)、スピーカー再生(sounddevice)もスタブ。
確認すること:
- 設定の検証(型・範囲・NaN/Infinity・空/長すぎるプロンプト・未知のキー)と保存(原子的+.bak、
  優先順位 ファイル > 環境変数 > 既定値、保存後は再起動なしで次の発話に効く)
- プロンプト: メニュー名/残数をOFFにすると本当に文面から消える。材料が無ければ設定がONでも
  フラグはfalse。両方ONの文面は従来と同一
- 計測: llm_ms/tts_ms/play_ms/queue_wait_ms/total_ms/モデル/プロンプトが記録される。
  フォールバック理由(timeout/empty/placeholder/length_cut/no_menu)
- DBの列追加(冪等)、古い行(新列NULL)でも/utterancesと統計が動く、統計の計算
- /voice_settings /voice_metrics /voice_preview(話さず履歴にも残さない) /utterances の互換

実行: python voice_bridge/tests/test_voice_bridge_metrics.py
"""

import json
import math
import os
import sqlite3
import sys
import tempfile
import threading
import time
import types
import unittest
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

TMP = tempfile.mkdtemp(prefix="vb_test_")
os.environ["VOICE_UTTERANCE_LOG_DB"] = os.path.join(TMP, "utt.db")
os.environ["VOICE_SETTINGS_FILE"] = os.path.join(TMP, "voice_settings.json")
os.environ.setdefault("BGM_ENABLED", "0")

sd = types.ModuleType("sounddevice")
sd.PortAudioError = type("PortAudioError", (Exception,), {})
sd.play = lambda *a, **k: None
sd.wait = lambda: None
sf = types.ModuleType("soundfile")
sf.read = lambda path, dtype="float32": ([0.0] * 10, 24000)
sys.modules.setdefault("sounddevice", sd)
sys.modules.setdefault("soundfile", sf)
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import voice_bridge as vb  # noqa: E402


class FakeServices:
    """Ollama(/api/generate)とVOICEVOX(/audio_query, /synthesis)のスタブ(1つのhttp.server)。"""

    def __init__(self):
        self.ollama_delay = 0.2
        self.ollama_body = {"response": "今日はカレーがおすすめだよ。しっかり食べなさい。", "done_reason": "stop"}
        self.ollama_status = 200
        self.tts_delay = 0.15
        self.requests = []
        outer = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _reply(self, status, body, ctype="application/json"):
                data = body if isinstance(body, bytes) else json.dumps(body).encode()
                self.send_response(status)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_POST(self):
                n = int(self.headers.get("Content-Length", "0"))
                raw = self.rfile.read(n) if n else b""
                outer.requests.append((self.path, raw))
                if self.path.startswith("/api/generate"):
                    time.sleep(outer.ollama_delay)
                    self._reply(outer.ollama_status, outer.ollama_body)
                elif self.path.startswith("/audio_query"):
                    time.sleep(outer.tts_delay / 2)
                    self._reply(200, {"accent_phrases": []})
                elif self.path.startswith("/synthesis"):
                    time.sleep(outer.tts_delay / 2)
                    self._reply(200, b"RIFFxxxxWAVE", "audio/wav")
                else:
                    self._reply(404, {})

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), H)
        self.port = self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def count(self, prefix):
        return sum(1 for p, _ in self.requests if p.startswith(prefix))

    def last_ollama(self):
        for p, raw in reversed(self.requests):
            if p.startswith("/api/generate"):
                return json.loads(raw)
        return None

    def close(self):
        self.server.shutdown()
        self.server.server_close()


MENU = [{"name": "カレー", "merit": "スタミナ"}, {"name": "A定食", "merit": ""}]
REMAINING = [
    {"name": "カレー", "remaining": 3, "initial": 10},
    {"name": "A定食", "remaining": 8, "initial": 10},
]


def ctx(**kw):
    d = {"phase": "initial", "menu_items": MENU, "fallback_text": "FALLBACK文です。"}
    d.update(kw)
    return d


def induction_ctx(**kw):
    d = ctx(phase="induction", promote_item="A定食", remaining_status=REMAINING)
    d.update(kw)
    return d


class Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.svc = FakeServices()
        cls._orig = (vb.OLLAMA_URL, vb.VOICEVOX_URL, vb._play_audio, vb.OLLAMA_TIMEOUT_SEC,
                     vb.OLLAMA_NUM_PREDICT, vb.MIN_REPEAT_SEC)
        vb.OLLAMA_URL = f"http://127.0.0.1:{cls.svc.port}/api/generate"
        vb.VOICEVOX_URL = f"http://127.0.0.1:{cls.svc.port}"
        vb._play_audio = lambda data, sr: time.sleep(0.05)
        cls.httpd = ThreadingHTTPServer(("127.0.0.1", 0), vb.make_handler())
        cls.base = f"http://127.0.0.1:{cls.httpd.server_address[1]}"
        threading.Thread(target=cls.httpd.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.httpd.shutdown()
        cls.httpd.server_close()
        cls.svc.close()
        (vb.OLLAMA_URL, vb.VOICEVOX_URL, vb._play_audio, vb.OLLAMA_TIMEOUT_SEC,
         vb.OLLAMA_NUM_PREDICT, vb.MIN_REPEAT_SEC) = cls._orig

    def setUp(self):
        for p in (vb.settings_store.path, vb.settings_store.backup_path):
            if os.path.exists(p):
                os.remove(p)
        vb.settings_store._loaded = False
        vb.OLLAMA_TIMEOUT_SEC = 60.0
        vb.OLLAMA_NUM_PREDICT = 400
        conn = sqlite3.connect(vb.UTTERANCE_LOG_DB)
        conn.execute("DELETE FROM utterances")
        conn.commit()
        conn.close()
        self.svc.ollama_delay = 0.2
        self.svc.tts_delay = 0.15
        self.svc.ollama_status = 200
        self.svc.ollama_body = {"response": "今日はカレーがおすすめだよ。しっかり食べなさい。", "done_reason": "stop"}
        self.svc.requests.clear()
        self._wait_idle()

    def _wait_idle(self):
        for _ in range(100):
            st = vb.speech_queue.status()
            if not st["speaking"] and st["queue_length"] == 0:
                return
            time.sleep(0.05)

    def http(self, method, path, body=None, raw=None):
        data = raw if raw is not None else (None if body is None else json.dumps(body).encode())
        req = urllib.request.Request(self.base + path, data=data, method=method,
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=20) as r:
                return r.status, json.loads(r.read().decode())
        except urllib.error.HTTPError as e:
            try:
                return e.code, json.loads(e.read().decode())
            finally:
                e.close()

    def speak_and_wait(self, context, n_rows=1):
        """/speakで依頼し、再生完了(total_ms記録)まで待って、新しい行(最新)を返す。"""
        self.assertTrue(vb.speech_queue.enqueue("", True, context=context))
        for _ in range(200):
            rows = vb.fetch_metrics_rows(50)
            if len(rows) >= n_rows and rows[-1].get("total_ms") is not None:
                return rows[-1]
            time.sleep(0.05)
        self.fail("再生完了の記録を待ちきれませんでした")

    def set_settings(self, **kw):
        ok, errs = vb.settings_store.update(kw)
        self.assertTrue(ok, errs)


class ValidationTests(Base):
    def test_valid_values(self):
        up, err = vb.validate_settings_update(
            {"include_menu_names": False, "temperature": 1, "num_predict": 300.0, "max_chars": 0,
             "persona_prompt": "  あなたは  ", "ollama_timeout_sec": 30, "min_repeat_sec": 0})
        self.assertEqual(err, [])
        self.assertEqual(up["persona_prompt"], "あなたは")
        self.assertIsInstance(up["num_predict"], int)

    def test_rejections(self):
        bad = [
            {"include_menu_names": 1}, {"include_remaining": "true"},
            {"temperature": float("nan")}, {"temperature": float("inf")}, {"temperature": -0.1},
            {"temperature": 2.5}, {"temperature": "0.7"}, {"temperature": True},
            {"num_predict": 31}, {"num_predict": 2001}, {"num_predict": 100.5},
            {"max_chars": -1}, {"max_chars": 501}, {"ollama_timeout_sec": 1},
            {"min_repeat_sec": 601}, {"persona_prompt": ""}, {"persona_prompt": "   "},
            {"persona_prompt": "あ" * 4001}, {"persona_prompt": 5}, {"persona_prompt": "a\x00b"},
            {"unknown_key": 1},
        ]
        for b in bad:
            up, err = vb.validate_settings_update(b)
            self.assertTrue(err, b)
            self.assertEqual(up, {}, b)  # 不正な項目は1つも受け付けない(単独指定の場合)
        self.assertEqual(vb.validate_settings_update([])[1] != [], True)

    def test_prompt_length_boundary(self):
        self.assertEqual(vb.validate_settings_update({"persona_prompt": "あ" * 4000})[1], [])

    def test_null_means_reset(self):
        up, err = vb.validate_settings_update({"persona_prompt": None, "temperature": None})
        self.assertEqual(err, [])
        self.assertEqual(up, {"persona_prompt": None, "temperature": None})


class StoreTests(Base):
    def test_precedence_file_over_env_over_default(self):
        d = vb.get_voice_settings()
        self.assertEqual(d["num_predict"], 400)  # env/既定
        self.assertEqual(vb.settings_store.sources()["num_predict"], "default")
        vb.OLLAMA_NUM_PREDICT = 250
        self.assertEqual(vb.get_voice_settings()["num_predict"], 250)  # envより
        self.set_settings(num_predict=123)
        self.assertEqual(vb.get_voice_settings()["num_predict"], 123)  # ファイルが最優先
        self.assertEqual(vb.settings_store.sources()["num_predict"], "file")
        self.set_settings(num_predict=None)  # ファイルから外すと戻る
        self.assertEqual(vb.get_voice_settings()["num_predict"], 250)

    def test_save_is_atomic_with_backup_and_hot_reload(self):
        self.set_settings(temperature=0.3)
        self.assertFalse(os.path.exists(vb.settings_store.backup_path))
        first_mtime = os.stat(vb.settings_store.path).st_mtime_ns
        self.set_settings(temperature=0.4, persona_prompt="新しいペルソナ")
        self.assertTrue(os.path.exists(vb.settings_store.backup_path))
        old = json.loads(Path(vb.settings_store.backup_path).read_text(encoding="utf-8"))
        self.assertEqual(old["temperature"], 0.3)
        cur = vb.get_voice_settings()
        self.assertEqual((cur["temperature"], cur["persona_prompt"]), (0.4, "新しいペルソナ"))
        leftovers = [n for n in os.listdir(TMP) if n.endswith(".tmp")]
        self.assertEqual(leftovers, [])
        # 外部(手編集)の書き換えもmtime/サイズ変化で検知する
        Path(vb.settings_store.path).write_text(json.dumps({"version": 1, "temperature": 1.5}), encoding="utf-8")
        self.assertEqual(vb.get_voice_settings()["temperature"], 1.5)
        self.assertGreaterEqual(os.stat(vb.settings_store.path).st_mtime_ns, first_mtime)

    def test_broken_file_is_ignored_with_error(self):
        Path(vb.settings_store.path).write_text("{broken", encoding="utf-8")
        self.assertEqual(vb.get_voice_settings()["temperature"], vb.DEFAULT_TEMPERATURE)
        self.assertTrue(vb.settings_store.load_error)
        Path(vb.settings_store.path).write_text('{"temperature": NaN}', encoding="utf-8")
        self.assertEqual(vb.get_voice_settings()["temperature"], vb.DEFAULT_TEMPERATURE)
        self.assertIn("NaN", vb.settings_store.load_error)
        Path(vb.settings_store.path).write_text('{"temperature": 99}', encoding="utf-8")  # 範囲外
        self.assertEqual(vb.get_voice_settings()["temperature"], vb.DEFAULT_TEMPERATURE)

    def test_defaults_persona_comes_from_persona_file(self):
        self.assertEqual(vb.default_voice_settings()["persona_prompt"], vb.load_persona_prompt())


ORIGINAL_INITIAL_PROMPT_TAIL = (
    "<指示>\n以下のメニュー一覧を踏まえて、お客さんに親しみやすく声をかける一言を考えてください。"
    "特定のメニューを無理に推す必要はありません。メニュー名は必ず下の一覧にある名前をそのまま使い、"
    "「○○」「〇〇」という伏せ字のまま出力したり、一覧に無いメニューを作ったりしないでください。\n\n"
    "<メニュー一覧>\n- カレー: スタミナ\n- A定食: \n看板娘:"
)


class PromptTests(Base):
    def test_default_prompt_unchanged(self):
        p, flags = vb.build_context_prompt_ex(ctx())
        self.assertTrue(p.endswith(ORIGINAL_INITIAL_PROMPT_TAIL), p[-400:])
        self.assertEqual(vb._build_context_prompt(ctx()), p)  # 互換関数
        self.assertEqual((flags["used_menu_names"], flags["used_remaining"]), (True, False))

    def test_induction_with_both_on(self):
        p, flags = vb.build_context_prompt_ex(induction_ctx())
        self.assertIn("今は特に「A定食」の残数に余裕があります。以下のメニュー一覧と残数状況を踏まえて、", p)
        self.assertIn("<残数状況>\n- カレー: 残り3個(元は10個)", p)
        self.assertEqual((flags["used_menu_names"], flags["used_remaining"]), (True, True))

    def test_remaining_off_removes_remaining(self):
        s = vb.get_voice_settings()
        s["include_remaining"] = False
        p, flags = vb.build_context_prompt_ex(induction_ctx(), s)
        self.assertNotIn("<残数状況>", p)
        self.assertNotIn("残り3個", p)
        self.assertNotIn("残数", p.split("<指示>")[1])
        self.assertIn("<メニュー一覧>", p)
        self.assertEqual((flags["used_menu_names"], flags["used_remaining"]), (True, False))

    def test_menu_names_off_removes_names_everywhere(self):
        s = vb.get_voice_settings()
        s["include_menu_names"] = False
        for c in (ctx(), induction_ctx()):
            p, flags = vb.build_context_prompt_ex(c, s)
            instr = p.split("<指示>")[1]
            for name in ("カレー", "A定食"):
                self.assertNotIn(name, instr, c.get("phase"))
            self.assertNotIn("<メニュー一覧>", p)
            self.assertFalse(flags["used_menu_names"])
        # 残数ありだと名前を伏せた残数だけ渡す
        p, flags = vb.build_context_prompt_ex(induction_ctx(), s)
        self.assertIn("<残数状況>\n- メニュー1: 残り3個", p)
        self.assertTrue(flags["used_remaining"])

    def test_both_off(self):
        s = vb.get_voice_settings()
        s.update(include_menu_names=False, include_remaining=False)
        p, flags = vb.build_context_prompt_ex(induction_ctx(), s)
        self.assertNotIn("カレー", p.split("<指示>")[1])
        self.assertNotIn("残り3個", p)
        self.assertTrue(p.endswith("看板娘:"))
        self.assertEqual((flags["used_menu_names"], flags["used_remaining"]), (False, False))

    def test_flags_are_honest_when_context_lacks_material(self):
        # 設定は両方ONだが、残数情報が無い(initialフェーズ/remaining_status空)ならused_remainingはfalse
        _, f1 = vb.build_context_prompt_ex(ctx())
        self.assertFalse(f1["used_remaining"])
        _, f2 = vb.build_context_prompt_ex(ctx(phase="induction", promote_item="A定食", remaining_status=[]))
        self.assertFalse(f2["used_remaining"])
        # メニュー項目が空ならused_menu_names=false(generate側ではno_menuでフォールバック)
        trace = {}
        text = vb.generate_utterance_from_context(ctx(menu_items=[]), trace=trace)
        self.assertEqual(text, "FALLBACK文です。")
        self.assertEqual(trace["fallback_reason"], "no_menu")
        self.assertFalse(trace["used_menu_names"])
        self.assertEqual(self.svc.count("/api/generate"), 0)

    def test_persona_from_settings_is_used(self):
        self.set_settings(persona_prompt="テスト用ペルソナXYZ")
        p, _ = vb.build_context_prompt_ex(ctx())
        self.assertTrue(p.startswith("<設定>\nテスト用ペルソナXYZ\n\n"))


class InstrumentationTests(Base):
    def setUp(self):
        super().setUp()
        self.set_settings(min_repeat_sec=0)

    def test_timings_flags_model_prompt_are_recorded(self):
        row = self.speak_and_wait(induction_ctx())
        self.assertGreaterEqual(row["llm_ms"], 190)
        self.assertLess(row["llm_ms"], 1500)
        self.assertGreaterEqual(row["tts_ms"], 140)
        self.assertGreaterEqual(row["play_ms"], 45)
        self.assertGreaterEqual(row["queue_wait_ms"], 0)
        self.assertGreaterEqual(row["total_ms"], row["llm_ms"] + row["tts_ms"] + row["play_ms"] - 1)
        self.assertTrue(row["used_menu_names"])
        self.assertTrue(row["used_remaining"])
        self.assertIsNone(row["fallback_reason"])
        self.assertEqual(row["model"], vb.MODEL_NAME)
        self.assertEqual(row["kind"], "context")
        self.assertEqual(row["phase"], "induction")
        self.assertTrue(row["speak_ok"])
        self.assertEqual(row["temperature"], 0.7)
        self.assertEqual(row["num_predict"], 400)
        self.assertTrue(row["request_id"])
        self.assertEqual(row["text"], "今日はカレーがおすすめだよ。しっかり食べなさい。")
        detail = vb.fetch_metrics_rows(row_id=row["id"])[0]
        self.assertIn("残り3個", detail["prompt"])
        self.assertLessEqual(len(detail["prompt"]), vb.PROMPT_LOG_MAX + 20)
        # 一覧にはプロンプト(巨大)を含めない
        self.assertNotIn("prompt", vb.fetch_metrics_rows(10)[0])

    def test_queue_wait_is_measured(self):
        self.svc.ollama_delay = 0.6
        vb.speech_queue.enqueue("", True, context=ctx(menu_items=[{"name": "一件目", "merit": ""}]))
        time.sleep(0.15)  # 1件目の処理中に2件目を積む
        vb.speech_queue.enqueue("", True, context=ctx(menu_items=[{"name": "二件目", "merit": ""}]))
        for _ in range(200):
            rows = vb.fetch_metrics_rows(10)
            if len(rows) == 2 and rows[-1].get("total_ms") is not None:
                break
            time.sleep(0.05)
        second = rows[-1]
        self.assertGreaterEqual(second["queue_wait_ms"], 300)
        self.assertGreater(second["total_ms"], second["queue_wait_ms"] + second["llm_ms"])

    def test_settings_apply_to_next_utterance_without_restart(self):
        self.set_settings(temperature=1.25, num_predict=111, include_menu_names=False)
        row = self.speak_and_wait(ctx())
        sent = self.svc.last_ollama()
        self.assertEqual(sent["options"], {"num_predict": 111, "temperature": 1.25})
        self.assertNotIn("カレー", sent["prompt"].split("<指示>")[1])
        self.assertFalse(row["used_menu_names"])
        self.assertEqual((row["temperature"], row["num_predict"]), (1.25, 111))
        self.set_settings(include_menu_names=True, temperature=0.2)
        row2 = self.speak_and_wait(ctx(fallback_text="別"), n_rows=2)
        self.assertTrue(row2["used_menu_names"])
        self.assertIn("カレー", self.svc.last_ollama()["prompt"])
        self.assertEqual(self.svc.last_ollama()["options"]["temperature"], 0.2)

    def test_fallback_timeout(self):
        vb.OLLAMA_TIMEOUT_SEC = 0.3
        self.svc.ollama_delay = 1.0
        row = self.speak_and_wait(ctx())
        self.assertEqual(row["fallback_reason"], "timeout")
        self.assertEqual(row["text"], "FALLBACK文です。")
        self.assertGreaterEqual(row["llm_ms"], 250)
        self.assertLess(row["llm_ms"], 900)
        self.assertTrue(row["error"])
        self.assertIsNotNone(row["tts_ms"])  # フォールバック文でも合成・再生は行う

    def test_fallback_reasons(self):
        cases = [
            ({"response": "   ", "done_reason": "stop"}, "empty"),
            ({"response": "今日は○○定食だよ。", "done_reason": "stop"}, "placeholder"),
            ({"response": "うどんがおすすめでし", "done_reason": "length"}, "length_cut"),
        ]
        for i, (body, reason) in enumerate(cases):
            self.svc.ollama_body = body
            row = self.speak_and_wait(ctx(fallback_text=f"FB{i}"), n_rows=i + 1)
            self.assertEqual(row["fallback_reason"], reason, body)
            self.assertEqual(row["text"], f"FB{i}")
        self.svc.ollama_body = {"response": "x", "done_reason": "stop"}
        self.svc.ollama_status = 500
        row = self.speak_and_wait(ctx(fallback_text="FB-err"), n_rows=4)
        self.assertEqual(row["fallback_reason"], "error")

    def test_length_trim_and_max_chars(self):
        self.svc.ollama_body = {"response": "一つ目の文です。二つ目の文です。三つ目はとちゅ", "done_reason": "length"}
        row = self.speak_and_wait(ctx())
        self.assertIsNone(row["fallback_reason"])
        self.assertTrue(row["trimmed"])
        self.assertEqual(row["text"], "一つ目の文です。二つ目の文です。")
        self.set_settings(max_chars=10)
        self.svc.ollama_body = {"response": "一つ目の文です。二つ目の文です。", "done_reason": "stop"}
        row = self.speak_and_wait(ctx(fallback_text="2"), n_rows=2)
        self.assertEqual(row["text"], "一つ目の文です。")
        self.assertTrue(row["trimmed"])
        self.svc.ollama_body = {"response": "文末のない長い長い応答", "done_reason": "stop"}
        row = self.speak_and_wait(ctx(fallback_text="3"), n_rows=3)
        self.assertEqual(row["fallback_reason"], "length_cut")

    def test_no_menu_context_skips_llm_but_is_recorded(self):
        row = self.speak_and_wait(ctx(menu_items=[]))
        self.assertEqual(row["fallback_reason"], "no_menu")
        self.assertIsNone(row["llm_ms"])
        self.assertFalse(row["used_menu_names"])
        self.assertEqual(self.svc.count("/api/generate"), 0)
        self.assertIsNotNone(row["tts_ms"])

    def test_text_mode_without_generate(self):
        self.assertTrue(vb.speech_queue.enqueue("そのまま話す文", False))
        for _ in range(200):
            rows = vb.fetch_metrics_rows(5)
            if rows and rows[-1].get("total_ms") is not None:
                break
            time.sleep(0.05)
        r = rows[-1]
        self.assertEqual((r["kind"], r["text"], r["llm_ms"]), ("text", "そのまま話す文", None))
        self.assertFalse(r["used_menu_names"])

    def test_tts_failure_is_recorded(self):
        orig = vb.VOICEVOX_URL
        vb.VOICEVOX_URL = "http://127.0.0.1:9"  # 繋がらない
        try:
            row = self.speak_and_wait(ctx())
        finally:
            vb.VOICEVOX_URL = orig
        self.assertFalse(row["speak_ok"])
        self.assertIn("TTS", row["error"])
        self.assertIsNone(row["play_ms"])

    def test_dedupe_uses_setting(self):
        self.set_settings(min_repeat_sec=600)
        c = ctx(fallback_text="dup")
        self.assertTrue(vb.speech_queue.enqueue("", True, context=c))
        self._wait_idle()
        n = self.svc.count("/api/generate")
        self.assertTrue(vb.speech_queue.enqueue("", True, context=c))  # 受理はするが積まない
        time.sleep(0.3)
        self.assertEqual(self.svc.count("/api/generate"), n)


class DbTests(Base):
    def test_migration_is_idempotent_and_old_rows_work(self):
        path = os.path.join(TMP, "legacy.db")
        conn = sqlite3.connect(path)
        conn.execute("CREATE TABLE utterances (id INTEGER PRIMARY KEY AUTOINCREMENT, timestamp TEXT NOT NULL, text TEXT NOT NULL)")
        conn.execute("INSERT INTO utterances (timestamp, text) VALUES ('2026-10-01T10:00:00', '古い発話')")
        conn.commit()
        conn.close()
        orig = vb.UTTERANCE_LOG_DB
        vb.UTTERANCE_LOG_DB = path
        try:
            vb._init_utterance_log()
            vb._init_utterance_log()  # 2回目も落ちない
            cols = {r[1] for r in sqlite3.connect(path).execute("PRAGMA table_info(utterances)")}
            for name, _ in vb._METRIC_COLUMNS:
                self.assertIn(name, cols)
            rows = vb.fetch_utterances()
            self.assertEqual(rows[0]["text"], "古い発話")
            self.assertIsNone(rows[0]["llm_ms"])
            mrows = vb.fetch_metrics_rows(50)
            stats = vb.compute_metric_stats(mrows)
            self.assertEqual(stats["timings"]["llm_ms"]["count"], 0)
            self.assertIsNone(stats["timings"]["total_ms"]["mean"])
        finally:
            vb.UTTERANCE_LOG_DB = orig

    def test_stats_values(self):
        rows = [{"llm_ms": float(v), "tts_ms": None, "total_ms": float(v) * 2} for v in range(1, 21)]
        rows.append({"llm_ms": None, "total_ms": None, "fallback_reason": "timeout"})
        rows[0]["fallback_reason"] = "empty"
        rows[1]["used_menu_names"] = True
        s = vb.compute_metric_stats(rows)
        llm = s["timings"]["llm_ms"]
        self.assertEqual(llm["count"], 20)
        self.assertEqual(llm["mean"], 10.5)
        self.assertEqual(llm["median"], 10.5)
        self.assertEqual(llm["p95"], 19.0)  # ceil(0.95*20)=19番目
        self.assertEqual(llm["max"], 20.0)
        self.assertEqual(s["timings"]["tts_ms"]["count"], 0)
        self.assertEqual(s["fallback_count"], 2)
        self.assertEqual(s["fallback_reasons"], {"empty": 1, "timeout": 1})
        self.assertEqual(s["used_menu_names_count"], 1)
        self.assertEqual(s["rows"], 21)
        self.assertFalse(any(math.isnan(v) for t in s["timings"].values() for v in t.values() if v is not None))
        self.assertEqual(vb.compute_metric_stats([])["rows"], 0)


class HttpTests(Base):
    def test_get_and_post_settings(self):
        st, d = self.http("GET", "/voice_settings")
        self.assertEqual(st, 200)
        self.assertTrue(d["ok"])
        self.assertEqual(d["settings"]["temperature"], 0.7)
        self.assertEqual(d["defaults"]["include_menu_names"], True)
        self.assertEqual(d["limits"]["persona_prompt"]["max_chars"], 4000)
        self.assertEqual(d["model"], vb.MODEL_NAME)
        st, d = self.http("POST", "/voice_settings", {"temperature": 0.9, "include_remaining": False, "persona_prompt": "新ペルソナ"})
        self.assertEqual(st, 200, d)
        self.assertEqual(d["settings"]["temperature"], 0.9)
        self.assertEqual(d["sources"]["temperature"], "file")
        st, d = self.http("GET", "/voice_settings")
        self.assertEqual((d["settings"]["include_remaining"], d["settings"]["persona_prompt"]), (False, "新ペルソナ"))
        st, d = self.http("POST", "/voice_settings", {"persona_prompt": None})
        self.assertEqual(st, 200)
        self.assertEqual(d["settings"]["persona_prompt"], vb.load_persona_prompt())

    def test_post_invalid_is_400_and_writes_nothing(self):
        for body in ({"temperature": 5}, {"persona_prompt": ""}, {"nope": 1}, {}, {"num_predict": 12.5}):
            st, d = self.http("POST", "/voice_settings", body)
            self.assertEqual(st, 400, body)
            self.assertFalse(d["ok"])
            self.assertTrue(d["errors"])
        st, d = self.http("POST", "/voice_settings", raw=b'{"temperature": NaN}')
        self.assertEqual(st, 400)
        st, d = self.http("POST", "/voice_settings", raw=b'[1]')
        self.assertEqual(st, 400)
        self.assertFalse(os.path.exists(vb.settings_store.path))

    def test_metrics_endpoint(self):
        self.set_settings(min_repeat_sec=0)
        self.speak_and_wait(ctx())
        st, d = self.http("GET", "/voice_metrics?limit=50")
        self.assertEqual(st, 200)
        self.assertEqual(len(d["rows"]), 1)
        self.assertEqual(d["stats"]["timings"]["llm_ms"]["count"], 1)
        self.assertGreater(d["stats"]["timings"]["total_ms"]["max"], 300)
        rid = d["rows"][0]["id"]
        st, d = self.http("GET", f"/voice_metrics?id={rid}")
        self.assertEqual(st, 200)
        self.assertIn("<設定>", d["row"]["prompt"])
        self.assertEqual(self.http("GET", "/voice_metrics?id=99999")[0], 404)
        self.assertEqual(self.http("GET", "/voice_metrics?limit=abc")[0], 400)
        self.assertEqual(self.http("GET", "/voice_metrics?limit=0")[0], 400)

    def test_utterances_is_backward_compatible_and_additive(self):
        vb._record_utterance("メタ無しの発話")  # 旧形式の呼び出し(metaなし)
        st, d = self.http("GET", "/utterances")
        self.assertEqual(st, 200)
        u = d["utterances"][0]
        self.assertEqual((u["timestamp"][:2], u["text"]), ("20", "メタ無しの発話"))
        self.assertIn("llm_ms", u)
        self.assertIsNone(u["llm_ms"])
        self.assertNotIn("prompt", u)
        # ダッシュボードのUtteranceLogReaderと同じ読み方(timestamp/textだけ取る)で動く
        self.assertEqual([(x["timestamp"], x["text"]) for x in d["utterances"]][0][1], "メタ無しの発話")
        st, d = self.http("GET", "/status")
        self.assertIn("speaking", d)

    def test_preview_does_not_speak_or_log(self):
        before = vb.fetch_utterances()
        st, d = self.http("POST", "/voice_preview", {
            "context": induction_ctx(),
            "settings": {"include_remaining": False, "temperature": 1.1, "persona_prompt": "未保存ペルソナ"},
        })
        self.assertEqual(st, 200, d)
        self.assertEqual(d["text"], "今日はカレーがおすすめだよ。しっかり食べなさい。")
        self.assertGreaterEqual(d["llm_ms"], 190)
        self.assertFalse(d["used_remaining"])
        self.assertTrue(d["used_menu_names"])
        self.assertIn("未保存ペルソナ", d["prompt"])
        self.assertEqual(self.svc.last_ollama()["options"]["temperature"], 1.1)
        self.assertEqual(self.svc.count("/audio_query"), 0)  # 話していない
        self.assertEqual(vb.fetch_utterances(), before)  # 履歴に残していない
        self.assertFalse(os.path.exists(vb.settings_store.path))  # 保存もしない
        # 検証エラー
        self.assertEqual(self.http("POST", "/voice_preview", {"context": ctx(), "settings": {"temperature": 9}})[0], 400)
        self.assertEqual(self.http("POST", "/voice_preview", {"settings": {}})[0], 400)
        # フォールバック(Ollama失敗)でも文面と理由を返す
        self.svc.ollama_status = 500
        st, d = self.http("POST", "/voice_preview", {"context": ctx()})
        self.assertEqual((st, d["fallback_reason"], d["text"]), (200, "error", "FALLBACK文です。"))


if __name__ == "__main__":
    unittest.main()
