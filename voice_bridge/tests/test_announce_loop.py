"""状況アナウンス(voice_bridge/announce_engine.py と voice_bridge.py の /announce* )のテスト。

実機・VOICEVOX・スピーカーには一切触れない(合成・再生・時計は差し替え)。
1. キャッシュ: 同じ文言は再合成しない、文言変更で再合成、ディスクに保存して再起動後も再利用、上限件数で古いものから削除
2. エンジン: ループ間隔、再生が重ならない、発話中はループを鳴らさない、keepaliveが途絶えたら停止、停止、マスターOFFは無音、
   1回再生の待ち・破棄、合成失敗時の再試行間隔、ループ数の上限
3. 入力検証(型違い・範囲外・未知項目・不正id)
4. HTTP: /announce_loop /announce /announce_state の検証(400)・音量(announce_volume)・マスターOFF・許可リスト(403)

実行: python voice_bridge/tests/test_announce_loop.py
"""

import json
import os
import sys
import tempfile
import threading
import types
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

TMP = tempfile.mkdtemp(prefix="vb_announce_test_")
os.environ["VOICE_UTTERANCE_LOG_DB"] = os.path.join(TMP, "utt.db")
os.environ["VOICE_SETTINGS_FILE"] = os.path.join(TMP, "voice_settings.json")
os.environ["VOICE_MASTER_FILE"] = os.path.join(TMP, "master.json")
os.environ["VOICE_LEVELS_FILE"] = os.path.join(TMP, "levels.json")
os.environ["VOICE_ANNOUNCE_CACHE_DIR"] = os.path.join(TMP, "cache_main")
os.environ["BGM_ENABLED"] = "0"
for _k in ("SPEECH_VOLUME", "BGM_VOLUME", "BGM_DUCK_VOLUME", "ANNOUNCE_VOLUME", "VOICE_BRIDGE_ALLOW"):
    os.environ.pop(_k, None)

sd = types.ModuleType("sounddevice")
sd.PortAudioError = type("PortAudioError", (Exception,), {})
sd.play = lambda *a, **k: None
sd.wait = lambda: None
sd.stop = lambda *a, **k: None
sf = types.ModuleType("soundfile")
sf.read = lambda path, dtype="float32": ([0.0] * 10, 24000)
sys.modules.setdefault("sounddevice", sd)
sys.modules.setdefault("soundfile", sf)
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import announce_engine as ae  # noqa: E402
import voice_bridge as vb  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def make_engine(directory=None, **kw):
    clock = Clock()
    synth_calls = []

    def synth(text):
        synth_calls.append(text)
        if kw.get("fail"):
            raise RuntimeError("synth down")
        return ("WAV:" + text).encode()

    cache = ae.AnnounceCache(directory or tempfile.mkdtemp(dir=TMP), synth, tag="t", max_items=kw.get("max_items", 64))
    state = {"muted": False, "busy": False, "plays": [], "duck": 0, "playing": 0, "overlap": False}

    def play(data, sr, vol):
        state["playing"] += 1
        if state["playing"] > 1:
            state["overlap"] = True
        state["plays"].append((data, vol))
        clock.t += 0.5  # 再生に0.5秒かかる
        state["playing"] -= 1

    eng = ae.AnnounceEngine(
        cache, decode=lambda b: (b, 24000), play=play, is_muted=lambda: state["muted"],
        speech_busy=lambda: state["busy"],
        duck_begin=lambda: state.__setitem__("duck", state["duck"] + 1),
        duck_end=lambda: state.__setitem__("duck", state["duck"] - 1),
        clock=clock, wall=lambda: 1.7e9 + clock.t, keepalive_sec=15.0)
    return eng, clock, state, synth_calls, cache


class CacheTest(unittest.TestCase):
    def test_synth_once_then_reuse(self):
        eng, clock, st, calls, cache = make_engine()
        eng.set_loop("exploring", True, "探索中です", 8, 1.0)
        for _ in range(3):
            eng.tick()
            clock.t += 9
            eng.set_loop("exploring", True, "探索中です", 8, 1.0)  # keepalive
        self.assertEqual(calls, ["探索中です"])
        self.assertEqual(len(st["plays"]), 3)

    def test_text_change_resynthesizes(self):
        eng, clock, st, calls, cache = make_engine()
        eng.set_loop("a", True, "探索中です", 8, 1.0)
        eng.tick()
        eng.set_loop("a", True, "探索しています", 8, 1.0)
        eng.tick()
        self.assertEqual(calls, ["探索中です", "探索しています"])

    def test_disk_reuse_after_restart(self):
        d = tempfile.mkdtemp(dir=TMP)
        eng, clock, st, calls, cache = make_engine(d)
        cache.get("こんにちは")
        eng2, _, _, calls2, cache2 = make_engine(d)
        blob, src = cache2.get("こんにちは")
        self.assertEqual(src, "disk")
        self.assertEqual(calls2, [])
        self.assertTrue(cache2.has("こんにちは"))

    def test_tag_change_is_a_different_key(self):
        d = tempfile.mkdtemp(dir=TMP)
        a = ae.AnnounceCache(d, lambda t: b"x", tag="speaker2")
        b = ae.AnnounceCache(d, lambda t: b"x", tag="speaker3")
        self.assertNotEqual(a.key("t"), b.key("t"))

    def test_eviction_oldest_first(self):
        d = tempfile.mkdtemp(dir=TMP)
        cache = ae.AnnounceCache(d, lambda t: t.encode(), max_items=3)
        for i in range(5):
            cache.get(f"文言{i}")
            os.utime(cache._path(cache.key(f"文言{i}")), (1000 + i, 1000 + i))
        self.assertEqual(cache.count(), 3)
        self.assertTrue(cache.has("文言4"))
        self.assertFalse(os.path.isfile(cache._path(cache.key("文言0"))))

    def test_disk_error_keeps_memory_cache(self):
        bad = os.path.join(TMP, "afile")
        open(bad, "w").close()
        cache = ae.AnnounceCache(os.path.join(bad, "sub"), lambda t: b"x")
        blob, src = cache.get("t")
        self.assertEqual(src, "synth")
        self.assertEqual(cache.get("t")[1], "memory")
        self.assertTrue(cache.disk_error)


class EngineTest(unittest.TestCase):
    def test_interval_and_no_overlap(self):
        eng, clock, st, calls, cache = make_engine()
        eng.set_loop("a", True, "A", 8, 1.0)
        eng.set_loop("b", True, "B", 8, 1.0)
        n = 0
        for _ in range(40):
            eng.tick()
            clock.t += 1
            eng.set_loop("a", True, "A", 8, 1.0)
            eng.set_loop("b", True, "B", 8, 1.0)
        self.assertFalse(st["overlap"])
        self.assertGreaterEqual(len(st["plays"]), 8)
        self.assertEqual(st["duck"], 0)
        s = eng.state()["loops"]["a"]
        self.assertGreaterEqual(s["plays"], 3)
        # 再生が終わってから8秒以上あける
        self.assertLessEqual(s["plays"], 6)

    def test_speech_busy_defers_loop(self):
        eng, clock, st, *_ = make_engine()
        eng.set_loop("a", True, "A", 8, 1.0)
        st["busy"] = True
        self.assertEqual(eng.tick(), "")
        self.assertEqual(st["plays"], [])
        self.assertEqual(eng.state()["loops"]["a"]["skipped_for_speech"], 1)
        st["busy"] = False
        clock.t += 1.5
        self.assertEqual(eng.tick(), "played")

    def test_keepalive_timeout_stops_loop(self):
        eng, clock, st, *_ = make_engine()
        eng.set_loop("a", True, "A", 3, 1.0)
        eng.tick()
        clock.t += 16
        eng.tick()
        s = eng.state()["loops"]["a"]
        self.assertFalse(s["active"])
        self.assertEqual(s["reason"], "expired")
        n = len(st["plays"])
        clock.t += 100
        eng.tick()
        self.assertEqual(len(st["plays"]), n)

    def test_keepalive_resend_keeps_alive(self):
        eng, clock, st, *_ = make_engine()
        for _ in range(10):
            eng.set_loop("a", True, "A", 30, 1.0)
            clock.t += 10
            eng.tick()
        self.assertTrue(eng.state()["loops"]["a"]["active"])

    def test_explicit_stop(self):
        eng, clock, st, *_ = make_engine()
        eng.set_loop("a", True, "A", 3, 1.0)
        eng.set_loop("a", False)
        self.assertEqual(eng.tick(), "")
        self.assertFalse(eng.state()["loops"]["a"]["active"])
        self.assertFalse(eng.set_loop("zzz", False)["known"])

    def test_master_off_is_silent_and_no_synth(self):
        eng, clock, st, calls, cache = make_engine()
        st["muted"] = True
        eng.set_loop("a", True, "A", 3, 1.0)
        eng.enqueue_once("o", "O", 1.0)
        for _ in range(5):
            clock.t += 1
            eng.set_loop("a", True, "A", 3, 1.0)
            eng.tick()
        self.assertEqual(st["plays"], [])
        self.assertEqual(calls, [])
        self.assertEqual(eng.state()["pending_once"], [])
        st["muted"] = False
        clock.t += 2
        self.assertEqual(eng.tick(), "played")  # ONに戻ればループは続く(溜まった分をまとめて鳴らさない)

    def test_once_plays_and_waits_for_speech(self):
        eng, clock, st, *_ = make_engine()
        eng.enqueue_once("o", "O", 2.0)
        st["busy"] = True
        self.assertEqual(eng.tick(), "")
        st["busy"] = False
        self.assertEqual(eng.tick(), "played")
        self.assertEqual(st["plays"][0][1], 2.0)
        self.assertEqual(eng.state()["once"]["o"]["plays"], 1)

    def test_once_dropped_after_max_wait(self):
        eng, clock, st, *_ = make_engine()
        eng.enqueue_once("o", "O", 1.0)
        st["busy"] = True
        clock.t += ae.ONCE_MAX_WAIT_SEC + 1
        eng.tick()
        self.assertEqual(eng.state()["once_dropped"], 1)
        st["busy"] = False
        self.assertEqual(eng.tick(), "")

    def test_urgent_once_ignores_speech_busy(self):
        eng, clock, st, *_ = make_engine()
        st["busy"] = True
        eng.enqueue_once("estop", "止まります", 1.0, urgent=True)
        self.assertEqual(eng.tick(), "played")

    def test_once_priority_over_loop(self):
        eng, clock, st, *_ = make_engine()
        eng.set_loop("a", True, "A", 8, 1.0)
        eng.enqueue_once("o", "O", 1.0)
        eng.tick()
        self.assertEqual(st["plays"][0][0], b"WAV:O")

    def test_synth_failure_backs_off(self):
        eng, clock, st, calls, cache = make_engine(fail=True)
        eng.set_loop("a", True, "A", 3, 1.0)
        self.assertEqual(eng.tick(), "synth_failed")
        self.assertEqual(eng.tick(), "")  # すぐには再試行しない
        self.assertIn("合成に失敗", eng.state()["loops"]["a"]["last_error"])
        clock.t += ae.SYNTH_RETRY_SEC + 1
        eng.set_loop("a", True, "A", 3, 1.0)
        self.assertEqual(eng.tick(), "synth_failed")
        self.assertEqual(len(calls), 2)

    def test_loop_count_limit(self):
        eng, *_ = make_engine()
        for i in range(ae.MAX_LOOPS):
            eng.set_loop(f"l{i}", True, "A", 8, 1.0)
        with self.assertRaises(ValueError):
            eng.set_loop("extra", True, "A", 8, 1.0)

    def test_interval_change_applies_sooner(self):
        eng, clock, st, *_ = make_engine()
        eng.set_loop("a", True, "A", 300, 1.0)
        eng.tick()
        clock.t += 20
        eng.set_loop("a", True, "A", 5, 1.0)
        self.assertEqual(eng.tick(), "played")


class ParseTest(unittest.TestCase):
    def test_loop_valid(self):
        r, e = ae.parse_loop_request({"id": "exploring", "text": " 探索中です ", "interval_sec": 8, "volume": 1.5})
        self.assertEqual(e, [])
        self.assertEqual((r["text"], r["interval_sec"], r["volume"], r["enabled"]), ("探索中です", 8.0, 1.5, True))

    def test_loop_stop_needs_only_id(self):
        r, e = ae.parse_loop_request({"id": "exploring", "enabled": False})
        self.assertEqual(e, [])

    def test_loop_invalid(self):
        base = {"id": "exploring", "text": "あ", "interval_sec": 8}
        bad = [
            {**base, "id": "Bad Id"}, {**base, "id": 5}, {**base, "id": ""}, {**base, "id": "a" * 33},
            {**base, "text": ""}, {**base, "text": "あ" * 81}, {**base, "text": 5}, {**base, "text": "a\x00b"},
            {**base, "interval_sec": 1.9}, {**base, "interval_sec": 301}, {**base, "interval_sec": "8"},
            {**base, "interval_sec": True}, {**base, "interval_sec": float("nan")},
            {**base, "volume": -0.1}, {**base, "volume": 5.1}, {**base, "volume": "1"}, {**base, "volume": None},
            {**base, "enabled": "yes"}, {**base, "extra": 1},
        ]
        for b in bad:
            r, e = ae.parse_loop_request(b)
            self.assertTrue(e, b)
        self.assertTrue(ae.parse_loop_request([1])[1])

    def test_once_invalid(self):
        self.assertTrue(ae.parse_once_request({"id": "x"})[1])
        self.assertTrue(ae.parse_once_request({"id": "x", "text": "a", "urgent": "1"})[1])
        self.assertTrue(ae.parse_once_request({"id": "x", "text": "a", "volume": 9})[1])
        self.assertFalse(ae.parse_once_request({"id": "x", "text": "a", "urgent": True})[1])


class AllowListTest(unittest.TestCase):
    def test_default(self):
        a = vb.parse_allow_list(vb.ALLOW_DEFAULT)
        self.assertTrue(vb.client_allowed("198.51.100.2", a))
        self.assertTrue(vb.client_allowed("127.0.0.1", a))
        self.assertTrue(vb.client_allowed("::1", a))
        self.assertTrue(vb.client_allowed("::ffff:198.51.100.2", a))
        for ip in ("198.51.100.4", "203.0.113.6", "198.51.100.22", "", "garbage", "8.8.8.8"):
            self.assertFalse(vb.client_allowed(ip, a), ip)

    def test_wildcard_cidr_and_junk(self):
        self.assertTrue(vb.client_allowed("8.8.8.8", vb.parse_allow_list("*")))
        a = vb.parse_allow_list("198.51.100.0/24, bad-entry ,")
        self.assertTrue(vb.client_allowed("198.51.100.77", a))
        self.assertFalse(vb.client_allowed("198.51.100.11", a))
        self.assertTrue(vb.client_allowed("127.0.0.1", a))  # ループバックは常に許可


class HttpTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), vb.make_handler())
        cls.url = f"http://127.0.0.1:{cls.srv.server_address[1]}"
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()
        # 合成・再生はスタブ。ワーカースレッドは起動しない(tickを手で回す)
        cls.plays = []
        vb.announce_cache.synth = lambda t: b"WAV:" + t.encode()
        vb.announce_engine.decode = lambda b: (b, 24000)
        vb.announce_engine.play = lambda d, sr, v: cls.plays.append((d, v))

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def setUp(self):
        self.plays.clear()
        vb.voice_master.set_enabled(True)
        vb.ALLOW = vb.parse_allow_list(vb.ALLOW_DEFAULT)
        with vb.announce_engine._lock:
            vb.announce_engine._loops.clear()
            vb.announce_engine._once.clear()
        vb.announce_engine.speech_busy = lambda: False

    def call(self, method, path, body=None):
        req = urllib.request.Request(self.url + path, method=method,
                                     data=None if body is None else json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read().decode())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read().decode())

    def test_loop_start_state_stop(self):
        code, r = self.call("POST", "/announce_loop", {"id": "exploring", "text": "探索中です", "interval_sec": 8})
        self.assertEqual(code, 200)
        self.assertTrue(r["loop"]["active"])
        vb.announce_engine.tick()
        code, st = self.call("GET", "/announce_state?text=" + urllib.request.quote("探索中です"))
        self.assertEqual(code, 200)
        self.assertEqual(st["loops"]["exploring"]["plays"], 1)
        self.assertTrue(st["cached"]["探索中です"])
        self.assertEqual(st["announce_volume"], vb.voice_levels.announce_volume)
        self.assertEqual(self.plays[0][0], b"WAV:" + "探索中です".encode())
        code, r = self.call("POST", "/announce_loop", {"id": "exploring", "enabled": False})
        self.assertFalse(r["loop"]["active"])

    def test_once(self):
        code, r = self.call("POST", "/announce", {"id": "test", "text": "試験です", "volume": 2})
        self.assertEqual(code, 202)
        vb.announce_engine.tick()
        self.assertEqual(self.plays[0][1], 2.0)

    def test_validation_400(self):
        for path, body in (("/announce_loop", {"id": "x", "text": "a", "interval_sec": 1}),
                           ("/announce_loop", {"id": "X", "text": "a", "interval_sec": 5}),
                           ("/announce_loop", {"id": "x", "text": "a", "interval_sec": "5"}),
                           ("/announce", {"id": "x", "text": "a", "volume": 6}),
                           ("/announce", {"id": "x"}),
                           ("/announce", {"id": "x", "text": "a", "bogus": 1})):
            code, r = self.call("POST", path, body)
            self.assertEqual(code, 400, (path, body))
            self.assertFalse(r["ok"])
        self.assertEqual(vb.announce_engine.state()["loops"], {})

    def test_master_off_is_muted_and_silent(self):
        vb.voice_master.set_enabled(False)
        code, r = self.call("POST", "/announce", {"id": "x", "text": "a"})
        self.assertEqual((code, r["muted"], r["accepted"]), (200, True, False))
        code, r = self.call("POST", "/announce_loop", {"id": "x", "text": "a", "interval_sec": 5})
        self.assertTrue(r["muted"])
        vb.announce_engine.tick()
        self.assertEqual(self.plays, [])
        code, st = self.call("GET", "/announce_state")
        self.assertTrue(st["muted"])

    def test_announce_volume_in_levels_and_gain(self):
        code, r = self.call("POST", "/voice_levels", {"announce_volume": 3})
        self.assertEqual(code, 200)
        self.assertEqual(r["announce_volume"], 3.0)
        self.assertEqual(self.call("POST", "/voice_levels", {"announce_volume": 5.1})[0], 400)
        self.assertEqual(self.call("POST", "/voice_levels", {"announce_volume": "3"})[0], 400)
        calls = []
        orig = vb._play_audio
        vb._play_audio = lambda d, sr: calls.append(d)
        try:
            import numpy as np
            vb._announce_play(np.full(2400, 0.1, dtype="float32"), 24000, 1.0)
            vb._announce_play(np.full(2400, 0.1, dtype="float32"), 24000, 5.0)  # 3*5=15 -> 5で頭打ち
        finally:
            vb._play_audio = orig
        self.assertAlmostEqual(float(calls[0][1200]), 0.3, places=3)
        self.assertLessEqual(float(abs(calls[1]).max()), vb.SPEECH_CEILING + 1e-6)
        self.call("POST", "/voice_levels", {"announce_volume": None})
        self.assertEqual(vb.voice_levels.announce_volume, 2.5)

    def test_forbidden_ip_403_everywhere(self):
        vb.ALLOW = vb.parse_allow_list("198.51.100.2")
        # テストはループバックから叩くので常に通る(ループバックは常に許可)ことを確認し、
        # 許可外判定そのものは client_allowed で確認する
        self.assertEqual(self.call("GET", "/status")[0], 200)
        h = vb.make_handler()
        sent = []

        class Fake(h):
            def __init__(self):
                self.client_address = ("203.0.113.50", 1234)
                self.path = "/voice_levels"

            def _send_json(self, status, payload):
                sent.append((int(status), payload))

        f = Fake()
        self.assertTrue(f._forbidden())
        self.assertEqual(sent[0][0], 403)
        for ip in ("198.51.100.2", "127.0.0.1"):
            f.client_address = (ip, 1)
            self.assertFalse(f._forbidden())


if __name__ == "__main__":
    unittest.main()
