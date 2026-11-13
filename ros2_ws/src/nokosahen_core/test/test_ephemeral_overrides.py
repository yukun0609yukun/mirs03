"""operatorの「一時的な変更(再起動で消える)」の仕組みのテスト(2026-10-13追加、docs/accounts.md)。

1. EphemeralOverrides(台帳): 記録・有効値(永続値+一時上書き)・adminの保存で破棄・全て破棄・再起動相当で消える
2. ジャーナル: 外部に実値が書かれる項目(発話間隔・表示モード・模擬POS)のbaselineを記録し、起動時に元へ戻す
   (戻せなければ残して再試行、最初のbaselineを保持)
3. voice_bridge(2台目)側: persist:false は保存ファイルを更新しない(メモリのみ)、adminの保存で破棄、
   /ephemeral_reset で保存値へ戻る、保存ファイルにoperatorの値が混ざらない
4. 起動時リセット(start_startup_reset): 相手が起動していなくても後から成功するまで再試行

実行: python ros2_ws/src/nokosahen_core/test/test_ephemeral_overrides.py
(voice_bridgeの依存の requests/numpy が無い環境では3だけスキップ)
"""
import json
import os
import sys
import tempfile
import threading
import types
import unittest
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
REPO = PKG.parents[2]
sys.path.insert(0, str(PKG))

TMP = tempfile.mkdtemp(prefix="ephemeral_test_")
os.environ["VOICE_UTTERANCE_LOG_DB"] = os.path.join(TMP, "utt.db")
os.environ["VOICE_SETTINGS_FILE"] = os.path.join(TMP, "voice_settings.json")
os.environ["VOICE_MASTER_FILE"] = os.path.join(TMP, "master.json")
os.environ["VOICE_LEVELS_FILE"] = os.path.join(TMP, "levels_main.json")
os.environ["BGM_ENABLED"] = "0"
for _k in ("SPEECH_VOLUME", "BGM_VOLUME", "BGM_DUCK_VOLUME", "ANNOUNCE_VOLUME"):
    os.environ.pop(_k, None)
_sd = types.ModuleType("sounddevice")
_sd.PortAudioError = type("PortAudioError", (Exception,), {})
_sd.play = lambda *a, **k: None
_sd.wait = lambda: None
_sd.stop = lambda *a, **k: None
_sf = types.ModuleType("soundfile")
_sf.read = lambda path, dtype="float32": ([0.0] * 10, 24000)
sys.modules.setdefault("sounddevice", _sd)
sys.modules.setdefault("soundfile", _sf)
sys.path.insert(0, str(REPO / "voice_bridge"))
try:
    import numpy  # noqa: F401, E402
    import voice_bridge as vb  # noqa: E402
except ImportError:
    vb = None

from nokosahen_core import ephemeral_overrides as eo  # noqa: E402
from nokosahen_core.ephemeral_overrides import EphemeralOverrides  # noqa: E402


class LedgerTest(unittest.TestCase):
    def test_operator_value_is_listed_and_effective(self):
        e = EphemeralOverrides()
        self.assertEqual(e.effective("voice_levels.speech_volume", 2.5), (2.5, False))
        e.record("voice_levels.speech_volume", 4.0, "op1")
        self.assertEqual(e.effective("voice_levels.speech_volume", 2.5), (4.0, True))
        rows = e.items()
        self.assertEqual(len(rows), 1)
        self.assertEqual((rows[0]["item"], rows[0]["by"], rows[0]["value"]), ("voice_levels.speech_volume", "op1", 4.0))
        self.assertTrue(rows[0]["label"])
        self.assertEqual(e.count(), 1)

    def test_admin_save_discards_only_that_item(self):
        e = EphemeralOverrides()
        e.record("voice_levels.speech_volume", 4.0, "op1")
        e.record("voice_master.enabled", False, "op1")
        self.assertEqual(e.discard(["voice_levels.speech_volume"]), 1)
        self.assertEqual(e.effective("voice_levels.speech_volume", 2.5), (2.5, False))
        self.assertEqual(e.count(), 1)
        self.assertEqual(e.discard(["not.there"]), 0)

    def test_restart_equivalent_loses_memory_overrides(self):
        e = EphemeralOverrides()
        e.record("voice_levels.speech_volume", 4.0, "op1")
        e2 = EphemeralOverrides()  # 新しいプロセス相当
        self.assertEqual(e2.count(), 0)
        self.assertEqual(e2.effective("voice_levels.speech_volume", 2.5), (2.5, False))

    def test_items_for_mapping(self):
        f = eo.items_for
        self.assertEqual(f("/api/voice_levels", {"speech_volume": 3, "bgm_duck_volume": None}),
                         {"voice_levels.speech_volume": 3, "voice_levels.bgm_duck_volume": None})
        self.assertEqual(f("/api/bgm", {"volume": 1, "enabled": True}), {"voice_levels.bgm_volume": 1})
        self.assertEqual(f("/api/voice_master", {"enabled": False}), {"voice_master.enabled": False})
        self.assertEqual(
            f("/api/voice_settings/save", {"announce": {"announce_min_interval_seconds": 30},
                                           "voice": {"temperature": 0.5, "persona_prompt": "x" * 200}}),
            {"announce.min_interval": 30, "voice_settings.temperature": 0.5, "voice_settings.persona_prompt": "x" * 80 + "…"})
        self.assertEqual(f("/api/survey/mode", {"mode": "survey"}), {"display.survey_mode": "survey"})
        self.assertEqual(f("/api/survey/ocr_mock", {"action": "reset"}), {"display.ocr_mock": "reset"})
        self.assertEqual(f("/api/teleop", {}), {})
        self.assertEqual(f("/api/pins/goto", {"id": "a"}), {})
        self.assertEqual(f("/api/voice_levels", "garbage"), {})


class JournalTest(unittest.TestCase):
    def setUp(self):
        self.jp = os.path.join(TMP, "journal_%s.json" % self._testMethodName)
        if os.path.exists(self.jp):
            os.remove(self.jp)

    def test_journal_written_for_external_items_only(self):
        e = EphemeralOverrides(self.jp)
        e.record("voice_levels.speech_volume", 4.0, "op1")  # メモリ上書き: ジャーナル不要
        self.assertFalse(os.path.exists(self.jp))
        e.record("announce.min_interval", 20, "op1", restore=eo.RESTORE_ANNOUNCE_INTERVAL, baseline=60)
        doc = json.loads(open(self.jp, encoding="utf-8").read())
        self.assertEqual(doc["entries"]["announce.min_interval"], {"restore": "announce_interval", "baseline": 60})
        self.assertNotIn("voice_levels.speech_volume", doc["entries"])

    def test_first_baseline_is_kept_on_second_change(self):
        e = EphemeralOverrides(self.jp)
        e.record("announce.min_interval", 20, "op1", restore=eo.RESTORE_ANNOUNCE_INTERVAL, baseline=60)
        e.record("announce.min_interval", 15, "op1", restore=eo.RESTORE_ANNOUNCE_INTERVAL, baseline=20)
        doc = json.loads(open(self.jp, encoding="utf-8").read())
        self.assertEqual(doc["entries"]["announce.min_interval"]["baseline"], 60)

    def test_restart_restores_baseline_and_clears_journal(self):
        e = EphemeralOverrides(self.jp)
        e.record("announce.min_interval", 20, "op1", restore=eo.RESTORE_ANNOUNCE_INTERVAL, baseline=60)
        e.record("display.survey_mode", "survey", "op1", restore=eo.RESTORE_SURVEY_MODE, baseline="customer")
        e.record("display.ocr_mock", "set", "op1", restore=eo.RESTORE_OCR_MOCK, baseline=None)
        calls = []
        restorers = {
            eo.RESTORE_ANNOUNCE_INTERVAL: lambda b: calls.append(("interval", b)) or True,
            eo.RESTORE_SURVEY_MODE: lambda b: calls.append(("mode", b)) or True,
            eo.RESTORE_OCR_MOCK: lambda b: calls.append(("mock", b)) or True,
        }
        e2 = EphemeralOverrides(self.jp)  # 再起動
        self.assertEqual(e2.count(), 0)
        self.assertEqual(e2.pending_restore_count(), 3)
        self.assertEqual(e2.restore_pending(restorers), (3, 0))
        self.assertEqual(sorted(calls), [("interval", 60), ("mock", None), ("mode", "customer")])
        self.assertFalse(os.path.exists(self.jp))
        self.assertEqual(EphemeralOverrides(self.jp).pending_restore_count(), 0)

    def test_failed_restore_is_kept_for_retry(self):
        e = EphemeralOverrides(self.jp)
        e.record("display.survey_mode", "survey", "op1", restore=eo.RESTORE_SURVEY_MODE, baseline="customer")
        e2 = EphemeralOverrides(self.jp)
        self.assertEqual(e2.restore_pending({eo.RESTORE_SURVEY_MODE: lambda b: False}), (0, 1))
        self.assertTrue(os.path.exists(self.jp))
        self.assertEqual(e2.restore_pending({eo.RESTORE_SURVEY_MODE: lambda b: True}), (1, 0))
        self.assertFalse(os.path.exists(self.jp))

    def test_restorer_exception_is_failure_not_crash(self):
        e = EphemeralOverrides(self.jp)
        e.record("display.survey_mode", "survey", "op1", restore=eo.RESTORE_SURVEY_MODE, baseline="customer")
        e2 = EphemeralOverrides(self.jp)

        def boom(_b):
            raise RuntimeError("x")

        self.assertEqual(e2.restore_pending({eo.RESTORE_SURVEY_MODE: boom}), (0, 1))

    def test_admin_save_removes_journal_entry(self):
        e = EphemeralOverrides(self.jp)
        e.record("announce.min_interval", 20, "op1", restore=eo.RESTORE_ANNOUNCE_INTERVAL, baseline=60)
        e.discard(["announce.min_interval"])  # 管理者が同じ項目を保存 -> 管理者の値が正。戻さない
        self.assertFalse(os.path.exists(self.jp))
        self.assertEqual(EphemeralOverrides(self.jp).pending_restore_count(), 0)

    def test_clear_all_restores_external_and_reports_failures(self):
        e = EphemeralOverrides(self.jp)
        e.record("voice_levels.speech_volume", 4.0, "op1")
        e.record("announce.min_interval", 20, "op1", restore=eo.RESTORE_ANNOUNCE_INTERVAL, baseline=60)
        e.record("display.survey_mode", "survey", "op1", restore=eo.RESTORE_SURVEY_MODE, baseline="customer")
        seen = []
        res = e.clear_all({eo.RESTORE_ANNOUNCE_INTERVAL: lambda b: seen.append(b) or True,
                           eo.RESTORE_SURVEY_MODE: lambda b: False})
        self.assertEqual(res["cleared"], 3)
        self.assertEqual(res["restore_failed"], ["display.survey_mode"])
        self.assertEqual(seen, [60])
        self.assertEqual(e.count(), 0)
        self.assertEqual(e.pending_restore_count(), 1)  # 失敗分は次の起動/リトライまで残る

    def test_unreadable_journal_is_ignored(self):
        open(self.jp, "w").write("{ broken")
        e = EphemeralOverrides(self.jp)
        self.assertEqual(e.pending_restore_count(), 0)


class RestorersTest(unittest.TestCase):
    def test_announce_restorer_deletes_file_when_baseline_none_or_rewrites(self):
        p = os.path.join(TMP, "announce_restore.json")
        open(p, "w").write("{}")

        class Store:
            path = p
            saved = []

            def save_interval(self, v):
                self.saved.append(v)
                return {"ok": True}

        s = Store()
        r = eo.build_restorers(announce_store=s)[eo.RESTORE_ANNOUNCE_INTERVAL]
        self.assertTrue(r(45))
        self.assertEqual(s.saved, [45])
        self.assertTrue(r(None))
        self.assertFalse(os.path.exists(p))
        self.assertTrue(r(None))  # 既に無くてもOK

    def test_survey_restorers(self):
        class Survey:
            configured = True
            calls = []

            def set_mode(self, d):
                self.calls.append(("mode", d))
                return 200, {"ok": True}

            def set_ocr_mock(self, d):
                self.calls.append(("mock", d))
                return 200, {"ok": False, "message": "OCRテストモードではありません"}

        s = Survey()
        rs = eo.build_restorers(survey_admin=s)
        self.assertTrue(rs[eo.RESTORE_SURVEY_MODE]("survey"))
        self.assertTrue(rs[eo.RESTORE_SURVEY_MODE](None))  # 不明なら安全側のcustomer
        self.assertEqual(s.calls[1], ("mode", {"mode": "customer"}))
        self.assertTrue(rs[eo.RESTORE_OCR_MOCK](None))  # 戻すものが無い=完了
        s2 = Survey()
        s2.set_ocr_mock = lambda d: (200, {"ok": False, "message": "ラズパイに届きません: x"})
        self.assertFalse(eo.build_restorers(survey_admin=s2)[eo.RESTORE_OCR_MOCK](None))

    def test_unconfigured_survey_has_no_restorers(self):
        class Survey:
            configured = False

        self.assertNotIn(eo.RESTORE_SURVEY_MODE, eo.build_restorers(survey_admin=Survey()))


# ----------------------------------------------------------------------
# voice_bridge(2台目)側
# ----------------------------------------------------------------------
@unittest.skipIf(vb is None, "voice_bridgeを読み込めません(requests/numpy未導入?)")
class BridgeLevelsTest(unittest.TestCase):
    def setUp(self):
        self.path = os.path.join(TMP, "lv_%s.json" % self._testMethodName)
        if os.path.exists(self.path):
            os.remove(self.path)
        self.lv = vb.VoiceLevels(self.path, env={})
        self.lv.update({"speech_volume": 2.0})  # 管理者の保存済みの値

    def file(self):
        return json.loads(open(self.path, encoding="utf-8").read())

    def test_ephemeral_update_does_not_touch_file(self):
        before = open(self.path, "rb").read()
        out = self.lv.update({"speech_volume": 4.5}, persist=False)
        self.assertEqual(self.lv.speech_volume, 4.5)  # メモリでは反映
        self.assertEqual(open(self.path, "rb").read(), before)  # ファイルは不変
        self.assertEqual(out["ephemeral_keys"], ["speech_volume"])

    def test_admin_save_of_other_key_does_not_persist_operator_value(self):
        self.lv.update({"speech_volume": 4.5}, persist=False)
        self.lv.update({"bgm_volume": 1.0})  # 管理者が別の項目を保存
        self.assertEqual(self.file()["speech_volume"], 2.0)  # operatorの値はファイルに入らない
        self.assertEqual(self.file()["bgm_volume"], 1.0)
        self.assertEqual(self.lv.speech_volume, 4.5)  # 上書きはまだ有効

    def test_admin_save_of_same_key_discards_override(self):
        self.lv.update({"speech_volume": 4.5}, persist=False)
        self.lv.update({"speech_volume": 3.0})
        self.assertEqual(self.lv.speech_volume, 3.0)
        self.assertEqual(self.file()["speech_volume"], 3.0)
        self.assertEqual(self.lv.reset_ephemeral(), 0)
        self.assertEqual(self.lv.speech_volume, 3.0)

    def test_reset_restores_persisted_and_notifies_listener(self):
        got = []
        self.lv.add_listener(lambda snap: got.append(snap["speech_volume"]))
        self.lv.update({"speech_volume": 4.5, "bgm_duck_volume": 0.0}, persist=False)
        self.assertEqual(self.lv.reset_ephemeral(), 2)
        self.assertEqual(self.lv.speech_volume, 2.0)
        self.assertEqual(got[-1], 2.0)
        self.assertEqual(self.lv.status()["ephemeral_keys"], [])

    def test_restart_equivalent_loads_file_only(self):
        self.lv.update({"speech_volume": 4.5}, persist=False)
        lv2 = vb.VoiceLevels(self.path, env={})
        self.assertEqual(lv2.speech_volume, 2.0)

    def test_none_resets_to_default_ephemerally(self):
        self.lv.update({"speech_volume": None}, persist=False)
        self.assertEqual(self.lv.speech_volume, self.lv.defaults["speech_volume"])
        self.assertEqual(self.file()["speech_volume"], 2.0)

    def test_bgm_compat_persist_false(self):
        before = open(self.path, "rb").read()
        self.lv.set_bgm_compat(volume=2.0, duck_volume=0.5, persist=False)
        self.assertEqual(self.lv.bgm_volume, 2.0)
        self.assertEqual(open(self.path, "rb").read(), before)


@unittest.skipIf(vb is None, "voice_bridgeを読み込めません(requests/numpy未導入?)")
class BridgeMasterTest(unittest.TestCase):
    def setUp(self):
        self.path = os.path.join(TMP, "ms_%s.json" % self._testMethodName)
        if os.path.exists(self.path):
            os.remove(self.path)
        self.m = vb.VoiceMaster(self.path)
        self.m.set_enabled(True)

    def test_ephemeral_off_not_saved_and_reset(self):
        before = open(self.path, "rb").read()
        seen = []
        self.m.add_listener(seen.append)
        out = self.m.set_enabled(False, persist=False)
        self.assertFalse(self.m.enabled)
        self.assertTrue(out["ephemeral"])
        self.assertEqual(open(self.path, "rb").read(), before)
        self.assertTrue(self.m.reset_ephemeral())
        self.assertTrue(self.m.enabled)
        self.assertEqual(seen, [False, True])
        self.assertFalse(self.m.reset_ephemeral())

    def test_restart_equivalent_reads_file(self):
        self.m.set_enabled(False, persist=False)
        self.assertTrue(vb.VoiceMaster(self.path).enabled)

    def test_admin_save_clears_ephemeral(self):
        self.m.set_enabled(False, persist=False)
        out = self.m.set_enabled(False)
        self.assertFalse(out["ephemeral"])
        self.assertFalse(vb.VoiceMaster(self.path).enabled)


@unittest.skipIf(vb is None, "voice_bridgeを読み込めません(requests/numpy未導入?)")
class BridgeSettingsTest(unittest.TestCase):
    def setUp(self):
        self.path = os.path.join(TMP, "st_%s.json" % self._testMethodName)
        for p in (self.path, self.path + ".bak"):
            if os.path.exists(p):
                os.remove(p)
        self.s = vb.VoiceSettingsStore(self.path)
        ok, _ = self.s.update({"temperature": 0.3})
        self.assertTrue(ok)

    def test_overlay_not_saved(self):
        before = open(self.path, "rb").read()
        self.s.update({"temperature": 1.5}, persist=False)
        self.assertEqual(self.s.effective()["temperature"], 1.5)
        self.assertEqual(self.s.sources()["temperature"], "ephemeral")
        self.assertEqual(open(self.path, "rb").read(), before)

    def test_none_means_default(self):
        self.s.update({"temperature": None}, persist=False)
        self.assertEqual(self.s.effective()["temperature"], vb.default_voice_settings()["temperature"])

    def test_reset_and_admin_save(self):
        self.s.update({"temperature": 1.5}, persist=False)
        self.assertEqual(self.s.reset_ephemeral(), 1)
        self.assertEqual(self.s.effective()["temperature"], 0.3)
        self.s.update({"temperature": 1.5}, persist=False)
        self.s.update({"temperature": 0.9})  # 管理者の保存
        self.assertEqual(self.s.effective()["temperature"], 0.9)
        self.assertEqual(self.s.reset_ephemeral(), 0)

    def test_restart_equivalent(self):
        self.s.update({"temperature": 1.5}, persist=False)
        self.assertEqual(vb.VoiceSettingsStore(self.path).effective()["temperature"], 0.3)


@unittest.skipIf(vb is None, "voice_bridgeを読み込めません(requests/numpy未導入?)")
class BridgeHttpTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), vb.make_handler())
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()
        cls.url = f"http://127.0.0.1:{cls.srv.server_address[1]}"

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def post(self, path, body):
        req = urllib.request.Request(self.url + path, data=json.dumps(body).encode(), method="POST",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=10) as r:
                return r.status, json.loads(r.read().decode())
        except urllib.error.HTTPError as e:
            try:
                return e.code, json.loads(e.read().decode())
            finally:
                e.close()

    def setUp(self):
        self.post("/ephemeral_reset", {})
        self.post("/voice_levels", {"speech_volume": 2.0})
        self.post("/voice_master", {"enabled": True})

    def test_persist_false_levels_master_then_reset(self):
        code, d = self.post("/voice_levels", {"speech_volume": 4.0, "persist": False})
        self.assertEqual((code, d["speech_volume"]), (200, 4.0))
        saved = json.loads(open(vb.VOICE_LEVELS_FILE, encoding="utf-8").read())
        self.assertEqual(saved["speech_volume"], 2.0)
        code, d = self.post("/voice_master", {"enabled": False, "persist": False})
        self.assertFalse(d["enabled"])
        code, d = self.post("/ephemeral_reset", {})
        self.assertEqual((code, d["ok"], d["levels"], d["master"]), (200, True, 1, True))
        with urllib.request.urlopen(self.url + "/voice_levels", timeout=10) as r:
            self.assertEqual(json.loads(r.read().decode())["speech_volume"], 2.0)
        with urllib.request.urlopen(self.url + "/voice_master", timeout=10) as r:
            self.assertTrue(json.loads(r.read().decode())["enabled"])

    def test_persist_key_does_not_break_validation_and_is_not_an_unknown_item(self):
        code, d = self.post("/voice_levels", {"speech_volume": 3.0, "persist": True})
        self.assertEqual(code, 200)
        code, d = self.post("/voice_levels", {"bogus": 1, "persist": False})
        self.assertEqual(code, 400)

    def test_bgm_post_persist_false(self):
        code, d = self.post("/bgm", {"volume": 3.0, "persist": False})
        self.assertEqual(code, 200)
        saved = json.loads(open(vb.VOICE_LEVELS_FILE, encoding="utf-8").read())
        self.assertNotEqual(saved["bgm_volume"], 3.0)
        self.post("/ephemeral_reset", {})


# ----------------------------------------------------------------------
# 起動時リセット(再試行)
# ----------------------------------------------------------------------
class _Remote(BaseHTTPRequestHandler):
    hits = 0
    mode = "ok"

    def log_message(self, *a):
        pass

    def do_POST(self):
        type(self).hits += 1
        n = int(self.headers.get("Content-Length", "0"))
        self.rfile.read(n)
        if self.path != "/ephemeral_reset":
            self.send_response(404)
        elif type(self).mode == "fail":
            self.send_response(500)
        elif type(self).mode == "old":
            self.send_response(404)
        else:
            self.send_response(200)
        body = b"{}"
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class StartupResetTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), _Remote)
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()
        cls.url = f"http://127.0.0.1:{cls.srv.server_address[1]}"

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def test_post_results(self):
        _Remote.mode = "ok"
        self.assertEqual(eo.ephemeral_reset_post(self.url), "ok")
        _Remote.mode = "old"
        self.assertEqual(eo.ephemeral_reset_post(self.url), "unsupported")
        _Remote.mode = "fail"
        self.assertEqual(eo.ephemeral_reset_post(self.url), "error")
        self.assertEqual(eo.ephemeral_reset_post("http://127.0.0.1:9", 0.5), "error")
        self.assertEqual(eo.ephemeral_reset_post(""), "ok")

    def test_reset_remote_dedups_urls(self):
        _Remote.mode = "ok"
        _Remote.hits = 0
        ok, ng = eo.reset_remote([self.url, self.url + "/", "", self.url])
        self.assertEqual((len(ok), ng), (1, []))
        self.assertEqual(_Remote.hits, 1)

    def test_startup_retries_until_remote_and_journal_succeed(self):
        jp = os.path.join(TMP, "journal_startup.json")
        if os.path.exists(jp):
            os.remove(jp)
        e = EphemeralOverrides(jp)
        e.record("display.survey_mode", "survey", "op1", restore=eo.RESTORE_SURVEY_MODE, baseline="customer")
        e2 = EphemeralOverrides(jp)  # 再起動
        state = {"n": 0}

        def restorer(b):
            state["n"] += 1
            return state["n"] >= 3  # 3回目で成功(ラズパイの起動待ち)

        _Remote.mode = "fail"
        _Remote.hits = 0
        sleeps = []

        def fake_sleep(s):
            sleeps.append(s)
            if len(sleeps) == 2:
                _Remote.mode = "ok"  # 2台目が起動した

        th = eo.start_startup_reset(e2, {eo.RESTORE_SURVEY_MODE: restorer}, [self.url],
                                    attempts=10, interval_s=0.01, sleep=fake_sleep)
        th.join(timeout=20)
        self.assertFalse(th.is_alive())
        self.assertEqual(e2.pending_restore_count(), 0)
        self.assertFalse(os.path.exists(jp))
        self.assertGreaterEqual(len(sleeps), 2)
        self.assertGreaterEqual(_Remote.hits, 3)
        _Remote.mode = "ok"

    def test_startup_gives_up_but_keeps_journal(self):
        jp = os.path.join(TMP, "journal_giveup.json")
        if os.path.exists(jp):
            os.remove(jp)
        EphemeralOverrides(jp).record("display.survey_mode", "survey", "op1", restore=eo.RESTORE_SURVEY_MODE,
                                      baseline="customer")
        e2 = EphemeralOverrides(jp)
        logs = []
        th = eo.start_startup_reset(e2, {eo.RESTORE_SURVEY_MODE: lambda b: False}, [], attempts=3,
                                    interval_s=0, sleep=lambda s: None, log=logs.append)
        th.join(timeout=20)
        self.assertEqual(e2.pending_restore_count(), 1)
        self.assertTrue(os.path.exists(jp))
        self.assertTrue(any("完了できません" in m for m in logs))


if __name__ == "__main__":
    unittest.main(verbosity=1)
