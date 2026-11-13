"""文化祭の出店紹介(プレイリスト・事前合成)のテスト。announce_engine.py / voice_bridge.py の /announce_prepare・/announce_loop(parts)。

実機・VOICEVOX・スピーカーには触れない(合成・再生・時計は差し替え)。docs/announcements.md「文化祭の出店紹介」
1. 入力検証: parts / gap_sec / interval_sec / 文字数(400) / 未知項目 / prepare の texts・start・force / cached_only
2. 文章の分割と、wavの連結(標準ライブラリ)。長い文は文ごとに合成して1本にする。短い文は従来どおり1回
3. 事前合成(PrepareWorker): 1回だけ合成してディスクに保存、同じ文章は再合成しない、失敗の状態、再試行の間隔、force
4. プレイリストの再生: パート順・パート間(gap)・一巡後(interval)の間隔、再生中は合成しない(未作成は飛ばす)、発話中は鳴らさない、
   重ならない、停止、緊急停止(urgent)で中断、キープアライブ途絶で停止、内容が変わったら最初から
5. キャッシュの容量上限(バイト)、再起動後もディスクから再利用、書き込みの原子性(一時ファイル→置き換え)
6. HTTP: /announce_prepare の検証(400)・状態・マスターOFF中も受理、/announce_loop(parts)、/announce の cached_only

実行: python voice_bridge/tests/test_pr_playlist.py
"""

import io
import json
import os
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
import wave
from http.server import ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_announce_loop as base  # noqa: E402  (環境変数とsounddevice等のスタブを先に整える)

ae = base.ae
vb = base.vb
TMP = base.TMP


def wav_bytes(n_frames=2400, rate=24000, value=1000):
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(int(value).to_bytes(2, "little", signed=True) * n_frames)
    return buf.getvalue()


def make(parts_texts=("一つ目です。", "二つ目です。", "三つ目です。"), **kw):
    """プレイリスト用のエンジン(合成関数は呼ばれたら記録するスタブ)。"""
    eng, clock, st, calls, cache = base.make_engine(**kw)
    return eng, clock, st, calls, cache, [{"text": t, "volume": 1.0 + i} for i, t in enumerate(parts_texts)]


def prepare_all(cache, parts):
    for p in parts:
        cache.get(p["text"])


class ParseTest(unittest.TestCase):
    def test_playlist_ok(self):
        req, errs = ae.parse_loop_request({"id": "pr_intro", "parts": [{"text": "あ", "volume": 2}, {"text": "い"}],
                                           "gap_sec": 8, "interval_sec": 600})
        self.assertEqual(errs, [])
        self.assertTrue(req["playlist"])
        self.assertEqual([p["volume"] for p in req["parts"]], [2.0, 1.0])
        self.assertEqual((req["gap_sec"], req["interval_sec"]), (8.0, 600.0))

    def test_playlist_limits(self):
        ok = {"id": "pr_intro", "parts": [{"text": "あ"}], "gap_sec": 5, "interval_sec": 60}
        self.assertEqual(ae.parse_loop_request(ok)[1], [])
        bad = [
            {**ok, "parts": []},
            {**ok, "parts": [{"text": "あ"}] * (ae.PLAYLIST_MAX_PARTS + 1)},
            {**ok, "parts": [{"text": "あ" * (ae.PR_TEXT_MAX + 1)}]},
            {**ok, "parts": [{"text": ""}]},
            {**ok, "parts": [{"text": "あ", "volume": 6}]},
            {**ok, "parts": [{"text": "あ", "x": 1}]},
            {**ok, "parts": ["あ"]},
            {**ok, "gap_sec": 0.5}, {**ok, "gap_sec": ae.GAP_MAX + 1}, {**ok, "gap_sec": "5"},
            {**ok, "interval_sec": 1}, {**ok, "interval_sec": ae.PLAYLIST_INTERVAL_MAX + 1},
            {**ok, "text": "併用不可"},
            {**ok, "id": "PR"},
        ]
        for b in bad:
            self.assertNotEqual(ae.parse_loop_request(b)[1], [], b)
        self.assertEqual(ae.parse_loop_request({**ok, "parts": [{"text": "あ" * ae.PR_TEXT_MAX}]})[1], [])

    def test_playlist_stop_may_omit_parts(self):
        req, errs = ae.parse_loop_request({"id": "pr_intro", "parts": [{"text": "a"}], "enabled": False})
        self.assertEqual(errs, [])
        req, errs = ae.parse_loop_request({"id": "pr_intro", "gap_sec": 5, "enabled": False})
        self.assertEqual(errs, [])

    def test_normal_loop_text_limit_unchanged(self):
        self.assertNotEqual(ae.parse_loop_request({"id": "x", "text": "あ" * 81, "interval_sec": 5})[1], [])
        self.assertNotEqual(ae.parse_loop_request({"id": "x", "text": "a", "interval_sec": 301})[1], [])

    def test_once_cached_only_allows_long_text(self):
        long_text = "あ" * 300
        self.assertNotEqual(ae.parse_once_request({"id": "pr_test", "text": long_text})[1], [])
        req, errs = ae.parse_once_request({"id": "pr_test", "text": long_text, "cached_only": True})
        self.assertEqual(errs, [])
        self.assertTrue(req["cached_only"])
        self.assertNotEqual(ae.parse_once_request({"id": "x", "text": "a", "cached_only": "yes"})[1], [])
        self.assertNotEqual(ae.parse_once_request({"id": "x", "text": "あ" * 401, "cached_only": True})[1], [])

    def test_prepare_request(self):
        req, errs = ae.parse_prepare_request({"texts": ["あ", "い"]})
        self.assertEqual((errs, req["start"], req["force"]), ([], True, False))
        req, errs = ae.parse_prepare_request({"texts": ["あ"], "start": False, "force": True})
        self.assertEqual((req["start"], req["force"]), (False, True))
        for bad in ({}, {"texts": []}, {"texts": ["あ"] * 9}, {"texts": ["あ" * 401]}, {"texts": [""]}, {"texts": [1]},
                    {"texts": ["あ"], "start": "1"}, {"texts": ["あ"], "x": 1}, [1]):
            self.assertIsNone(ae.parse_prepare_request(bad)[0], bad)


class SplitJoinTest(unittest.TestCase):
    def test_split_keeps_text_and_respects_limit(self):
        text = "これは一つ目の文です。" * 5 + "二つ目は読点、を含む少し長い文章で、" * 3 + "最後の文！"
        chunks = ae.split_for_synthesis(text, 40)
        self.assertEqual("".join(chunks), text)
        self.assertTrue(all(len(c) <= 40 for c in chunks), [len(c) for c in chunks])
        self.assertGreater(len(chunks), 3)

    def test_split_long_sentence_without_punctuation(self):
        text = "あ" * 200
        chunks = ae.split_for_synthesis(text, 80)
        self.assertEqual("".join(chunks), text)
        self.assertTrue(all(len(c) <= 80 for c in chunks))

    def test_split_newlines(self):
        self.assertEqual("".join(ae.split_for_synthesis("一行目\n二行目", 80)), "一行目二行目")

    def test_join_wavs(self):
        a, b = wav_bytes(2400), wav_bytes(1200)
        joined = ae.join_wavs([a, b], pause_sec=0.25)
        with wave.open(io.BytesIO(joined), "rb") as r:
            self.assertEqual(r.getframerate(), 24000)
            self.assertEqual(r.getnframes(), 2400 + 1200 + 6000)
        self.assertEqual(ae.join_wavs([a]), a)
        with self.assertRaises(ValueError):
            ae.join_wavs([a, wav_bytes(100, rate=16000)])

    def test_synthesize_long_short_text_single_call(self):
        calls = []
        out = ae.synthesize_long("短い文です。", lambda t: calls.append(t) or wav_bytes(), max_chars=80)
        self.assertEqual(calls, ["短い文です。"])
        self.assertTrue(out)

    def test_synthesize_long_splits_and_joins(self):
        calls = []
        text = "これは長い文章の一部です。" * 20
        out = ae.synthesize_long(text, lambda t: calls.append(t) or wav_bytes(1000), max_chars=60)
        self.assertGreater(len(calls), 3)
        self.assertEqual("".join(calls), text)
        with wave.open(io.BytesIO(out), "rb") as r:
            self.assertGreater(r.getnframes(), 1000 * len(calls))

    def test_synthesize_long_deadline_aborts_without_partial(self):
        t = {"v": 0.0}

        def synth(c):
            t["v"] += 100
            return wav_bytes()

        with self.assertRaises(TimeoutError):
            ae.synthesize_long("文です。" * 100, synth, max_chars=40, deadline_sec=250, clock=lambda: t["v"])

    def test_synthesize_long_failure_propagates(self):
        def synth(c):
            raise RuntimeError("voicevox down")

        with self.assertRaises(RuntimeError):
            ae.synthesize_long("文です。" * 100, synth, max_chars=40)


class PrepareWorkerTest(unittest.TestCase):
    def make(self, fail=False, **kw):
        d = kw.pop("directory", None) or tempfile.mkdtemp(dir=TMP)
        calls = []
        clock = base.Clock()

        def synth(text):
            calls.append(text)
            if fail:
                raise RuntimeError("voicevox down")
            return ("WAV:" + text).encode()

        cache = ae.AnnounceCache(d, synth, tag="t", **kw)
        return ae.PrepareWorker(cache, clock=clock), cache, calls, clock

    def drain(self, w):
        while w.process_one():
            pass

    def test_synthesizes_once_and_saves_to_disk(self):
        w, cache, calls, _ = self.make()
        w.request(["あ。", "い。"])
        self.assertEqual([i["state"] for i in w.status(["あ。", "い。"])], ["queued", "queued"])
        self.drain(w)
        self.assertEqual(calls, ["あ。", "い。"])
        self.assertEqual([i["state"] for i in w.status(["あ。", "い。"])], ["ready", "ready"])
        self.assertTrue(os.path.isfile(cache._path(cache.key("あ。"))))
        # 同じ内容の要求は再合成しない
        w.request(["あ。", "い。"])
        self.assertFalse(w.process_one())
        self.assertEqual(len(calls), 2)

    def test_changed_text_only_is_resynthesized(self):
        w, cache, calls, _ = self.make()
        w.request(["一。", "二。"])
        self.drain(w)
        w.request(["一。", "二(変更)。"])
        self.drain(w)
        self.assertEqual(calls, ["一。", "二。", "二(変更)。"])

    def test_reuse_after_restart(self):
        d = tempfile.mkdtemp(dir=TMP)
        w, cache, calls, _ = self.make(directory=d)
        w.request(["再起動。"])
        self.drain(w)
        w2, cache2, calls2, _ = self.make(directory=d)
        self.assertEqual(w2.status(["再起動。"])[0]["state"], "ready")
        w2.request(["再起動。"])
        self.assertFalse(w2.process_one())
        self.assertEqual(calls2, [])

    def test_failure_state_retry_interval_and_force(self):
        w, cache, calls, clock = self.make(fail=True)
        w.request(["失敗。"])
        self.drain(w)
        st = w.status(["失敗。"])[0]
        self.assertEqual(st["state"], "failed")
        self.assertIn("voicevox down", st["error"])
        self.assertEqual(w.summary()["failed"], 1)
        w.request(["失敗。"])  # 30秒以内の自動再要求は無視
        self.assertFalse(w.process_one())
        clock.t += ae.PREPARE_RETRY_SEC + 1
        w.request(["失敗。"])
        self.assertTrue(w.process_one())
        clock.t += 1
        w.request(["失敗。"], force=True)  # 画面の「音声を作る」は即時
        self.assertTrue(w.process_one())
        self.assertEqual(len(calls), 3)

    def test_failure_never_leaves_partial_file(self):
        w, cache, calls, _ = self.make(fail=True)
        w.request(["失敗。"])
        self.drain(w)
        self.assertEqual(cache.count(), 0)
        self.assertEqual([n for n in os.listdir(cache.dir) if n.endswith(".tmp")], [])

    def test_background_thread_runs_and_stops(self):
        w, cache, calls, _ = self.make()
        w.start()
        try:
            w.request(["背景。"])
            for _ in range(100):
                if w.status(["背景。"])[0]["state"] == "ready":
                    break
                threading.Event().wait(0.05)
            self.assertEqual(w.status(["背景。"])[0]["state"], "ready")
        finally:
            w.stop()


class AtomicStoreTest(unittest.TestCase):
    def test_replace_failure_keeps_old_file_and_removes_tmp(self):
        d = tempfile.mkdtemp(dir=TMP)
        cache = ae.AnnounceCache(d, lambda t: b"NEW", tag="t")
        k = cache.key("文")
        cache._store(k, b"OLD")
        orig = os.replace

        def boom(a, b):
            raise OSError("disk full")

        os.replace = boom
        try:
            cache._store(k, b"NEW")
        finally:
            os.replace = orig
        with open(cache._path(k), "rb") as f:
            self.assertEqual(f.read(), b"OLD")
        self.assertEqual([n for n in os.listdir(d) if n.endswith(".tmp")], [])
        self.assertTrue(cache.disk_error)

    def test_byte_cap_evicts_oldest_but_keeps_newest(self):
        d = tempfile.mkdtemp(dir=TMP)
        cache = ae.AnnounceCache(d, lambda t: b"x" * 1000, max_items=64, max_bytes=2500)
        for i in range(4):
            cache.get(f"文{i}")
            os.utime(cache._path(cache.key(f"文{i}")), (1000 + i, 1000 + i))
        self.assertLessEqual(cache.total_bytes(), 2500)
        self.assertTrue(cache.has("文3"))
        self.assertFalse(cache.has("文0"))
        self.assertEqual(cache.stats()["max_bytes"], 2500)

    def test_single_item_larger_than_cap_is_kept(self):
        d = tempfile.mkdtemp(dir=TMP)
        cache = ae.AnnounceCache(d, lambda t: b"x" * 5000, max_bytes=100)
        cache.get("大きい")
        self.assertTrue(cache.has("大きい"))


class PlaylistEngineTest(unittest.TestCase):
    def run_until_idle(self, eng, clock, limit=50):
        """時刻を進めながら tick を回し、再生順を返す。"""
        played = []
        for _ in range(limit):
            r = eng.tick()
            if r == "played":
                played.append(len(played))
            clock.t += 0.1
        return played

    def test_order_gap_interval_and_no_synthesis_during_playback(self):
        eng, clock, st, calls, cache, parts = make()
        prepare_all(cache, parts)
        n_synth = len(calls)
        eng.set_loop("pr_intro", True, interval=100, gap=10, parts=parts)
        order = []
        times = []
        for _ in range(4000):
            if eng.tick() == "played":
                order.append(st["plays"][-1][0])
                times.append(round(clock.t, 1))
            clock.t += 0.1
            eng.set_loop("pr_intro", True, interval=100, gap=10, parts=parts)  # keepalive
            if len(order) >= 6:
                break
        texts = [p["text"] for p in parts]
        self.assertEqual(order, [("WAV:" + t).encode() for t in texts] * 2)
        self.assertEqual(len(calls), n_synth, "再生中に合成(推論)してはいけない")
        # 再生は0.5秒。パート間は再生が終わってから gap(10秒)、一巡後は interval(100秒)
        self.assertAlmostEqual(times[1] - times[0], 10.5, delta=0.3)
        self.assertAlmostEqual(times[2] - times[1], 10.5, delta=0.3)
        self.assertAlmostEqual(times[3] - times[2], 100.5, delta=0.3)
        self.assertFalse(st["overlap"])
        self.assertEqual(st["plays"][0][1], 1.0)  # パートごとの音量
        self.assertEqual(st["plays"][2][1], 3.0)

    def test_unprepared_parts_are_skipped_without_synthesis(self):
        eng, clock, st, calls, cache, parts = make()
        cache.get(parts[1]["text"])  # 2つ目だけ作成済み
        calls.clear()
        eng.set_loop("pr_intro", True, interval=100, gap=5, parts=parts)
        r1 = eng.tick()
        self.assertEqual(r1, "not_prepared")
        s = eng._loops["pr_intro"]
        self.assertEqual(s.idx, 1)
        self.assertIn("未作成", s.last_error)
        clock.t += 5.1
        eng.set_loop("pr_intro", True, interval=100, gap=5, parts=parts)
        self.assertEqual(eng.tick(), "played")
        self.assertEqual(st["plays"][0][0], ("WAV:" + parts[1]["text"]).encode())
        self.assertEqual(calls, [], "未作成でも再生側は合成しない")
        self.assertEqual(eng.state()["loops"]["pr_intro"]["skipped_unprepared"], 1)

    def test_all_unprepared_does_not_spin(self):
        eng, clock, st, calls, cache, parts = make()
        eng.set_loop("pr_intro", True, interval=100, gap=5, parts=parts)
        n = 0
        for _ in range(300):
            if eng.tick() == "not_prepared":
                n += 1
            clock.t += 0.1
            eng.set_loop("pr_intro", True, interval=100, gap=5, parts=parts)
        self.assertLessEqual(n, 4)  # 30秒で 5秒おき×2 + 100秒の間隔
        self.assertEqual(calls, [])

    def test_speech_busy_defers_and_never_overlaps(self):
        eng, clock, st, calls, cache, parts = make()
        prepare_all(cache, parts)
        eng.set_loop("pr_intro", True, interval=100, gap=5, parts=parts)
        st["busy"] = True
        for _ in range(5):
            self.assertEqual(eng.tick(), "")
            clock.t += 1.1
            eng.set_loop("pr_intro", True, interval=100, gap=5, parts=parts)
        self.assertEqual(st["plays"], [])
        st["busy"] = False
        clock.t += 1.1
        self.assertEqual(eng.tick(), "played")

    def test_master_off_is_silent(self):
        eng, clock, st, calls, cache, parts = make()
        prepare_all(cache, parts)
        st["muted"] = True
        eng.set_loop("pr_intro", True, interval=100, gap=5, parts=parts)
        for _ in range(5):
            self.assertEqual(eng.tick(), "")
            clock.t += 1
        self.assertEqual(st["plays"], [])

    def test_keepalive_expiry_stops(self):
        eng, clock, st, calls, cache, parts = make()
        prepare_all(cache, parts)
        eng.set_loop("pr_intro", True, interval=100, gap=5, parts=parts)
        clock.t += 16
        eng.tick()
        self.assertEqual(eng.state()["loops"]["pr_intro"]["reason"], "expired")
        self.assertEqual(st["plays"], [])

    def test_stop_then_resume_continues_from_current_part(self):
        eng, clock, st, calls, cache, parts = make()
        prepare_all(cache, parts)
        eng.set_loop("pr_intro", True, interval=100, gap=5, parts=parts)
        eng.tick()
        eng.set_loop("pr_intro", False, parts=[])
        clock.t += 6
        self.assertEqual(eng.tick(), "")
        eng.set_loop("pr_intro", True, interval=100, gap=5, parts=parts)
        self.assertEqual(eng.tick(), "played")
        self.assertEqual(st["plays"][1][0], ("WAV:" + parts[1]["text"]).encode())

    def test_changed_text_restarts_from_first_part(self):
        eng, clock, st, calls, cache, parts = make()
        prepare_all(cache, parts)
        eng.set_loop("pr_intro", True, interval=100, gap=5, parts=parts)
        eng.tick()
        new_parts = [dict(parts[0], text="新しい一つ目。")] + parts[1:]
        cache.get("新しい一つ目。")
        eng.set_loop("pr_intro", True, interval=100, gap=5, parts=new_parts)
        self.assertEqual(eng._loops["pr_intro"].idx, 0)
        self.assertEqual(eng.tick(), "played")
        self.assertEqual(st["plays"][-1][0], b"WAV:" + "新しい一つ目。".encode())

    def test_interval_change_applies_without_waiting_old_interval(self):
        eng, clock, st, calls, cache, parts = make(("一つだけ。",))
        prepare_all(cache, parts)
        eng.set_loop("pr_intro", True, interval=3000, gap=5, parts=parts)
        eng.tick()
        clock.t += 1
        eng.set_loop("pr_intro", True, interval=30, gap=5, parts=parts)
        clock.t += 31
        eng.set_loop("pr_intro", True, interval=30, gap=5, parts=parts)
        self.assertEqual(eng.tick(), "played")

    def test_urgent_once_interrupts_playlist_playback(self):
        eng, clock, st, calls, cache, parts = make()
        prepare_all(cache, parts)
        stops = []
        eng.stop_playback = lambda: stops.append(1)
        eng.set_loop("pr_intro", True, interval=100, gap=5, parts=parts)
        # 再生中を模擬: playが呼ばれている間に緊急のアナウンスが来る
        orig_play = eng.play

        def play(data, sr, vol):
            eng.enqueue_once("emergency_stop", "緊急停止しました", 1.5, urgent=True)
            orig_play(data, sr, vol)

        eng.play = play
        eng.tick()
        self.assertEqual(stops, [1])
        # 通常のループ・緊急でない1回再生では中断しない
        eng.play = orig_play
        eng.enqueue_once("x", "通常", 1.0, urgent=False)
        self.assertEqual(stops, [1])

    def test_stop_during_playback_interrupts(self):
        eng, clock, st, calls, cache, parts = make()
        prepare_all(cache, parts)
        stops = []
        eng.stop_playback = lambda: stops.append(1)
        eng.set_loop("pr_intro", True, interval=100, gap=5, parts=parts)
        orig_play = eng.play

        def play(data, sr, vol):
            eng.set_loop("pr_intro", False, parts=[])  # 移動中の注意などで紹介を止める
            orig_play(data, sr, vol)

        eng.play = play
        eng.tick()
        self.assertEqual(stops, [1])

    def test_cached_only_once(self):
        eng, clock, st, calls, cache, parts = make()
        eng.enqueue_once("pr_test", "未作成の長い文章です。", 1.0, cached_only=True)
        self.assertEqual(eng.tick(), "not_prepared")
        self.assertEqual(calls, [])
        self.assertEqual(eng.once_dropped, 1)
        cache.get("作成済み。")
        calls.clear()
        eng.enqueue_once("pr_test", "作成済み。", 1.0, cached_only=True)
        self.assertEqual(eng.tick(), "played")
        self.assertEqual(calls, [])

    def test_existing_loop_still_synthesizes_on_first_play(self):
        eng, clock, st, calls, cache, parts = make()
        eng.set_loop("exploring", True, "探索中です", 8, 1.0)
        self.assertEqual(eng.tick(), "played")
        self.assertEqual(calls, ["探索中です"])

    def test_state_reports_playlist(self):
        eng, clock, st, calls, cache, parts = make()
        prepare_all(cache, parts)
        eng.set_loop("pr_intro", True, interval=100, gap=5, parts=parts)
        s = eng.state()["loops"]["pr_intro"]
        self.assertTrue(s["playlist"])
        self.assertEqual((s["parts"], s["next_part"], s["gap_sec"]), (3, 0, 5.0))
        self.assertTrue(s["cached"])


class HttpTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), vb.make_handler())
        cls.url = f"http://127.0.0.1:{cls.srv.server_address[1]}"
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()
        cls.plays = []
        cls.synth_calls = []
        vb.announce_cache.synth = lambda t: cls.synth_calls.append(t) or b"WAV:" + t.encode()
        vb.announce_engine.decode = lambda b: (b, 24000)
        vb.announce_engine.play = lambda d, sr, v: cls.plays.append((d, v))

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def setUp(self):
        self.plays.clear()
        self.synth_calls.clear()
        vb.voice_master.set_enabled(True)
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

    def test_prepare_validation(self):
        for body in ({}, {"texts": []}, {"texts": ["あ" * 401]}, {"texts": ["あ"], "z": 1}, {"texts": "あ"}):
            code, r = self.call("POST", "/announce_prepare", body)
            self.assertEqual(code, 400, body)
            self.assertFalse(r["ok"])

    def test_prepare_status_only_does_not_synthesize(self):
        code, r = self.call("POST", "/announce_prepare", {"texts": ["状態だけ見る文章です。"], "start": False})
        self.assertEqual(code, 200)
        self.assertEqual(r["items"][0]["state"], "missing")
        self.assertEqual(self.synth_calls, [])

    def test_prepare_then_playlist_plays_saved_wav_only(self):
        texts = ["文化祭一つ目の文章です。", "文化祭二つ目の文章です。"]
        code, r = self.call("POST", "/announce_prepare", {"texts": texts})
        self.assertEqual(code, 200)
        self.assertEqual([i["state"] for i in r["items"]], ["queued", "queued"])  # すぐ返る(背景で合成)
        while vb.announce_prepare.process_one():
            pass
        code, r = self.call("POST", "/announce_prepare", {"texts": texts, "start": False})
        self.assertEqual([i["state"] for i in r["items"]], ["ready", "ready"])
        self.assertEqual(self.synth_calls, texts)
        # プレイリスト開始
        code, r = self.call("POST", "/announce_loop", {"id": "pr_intro", "parts": [{"text": t} for t in texts],
                                                      "gap_sec": 5, "interval_sec": 60})
        self.assertEqual(code, 200)
        self.assertTrue(r["loop"]["active"])
        self.assertTrue(r["loop"]["playlist"])
        vb.announce_engine.tick()
        self.assertEqual(self.plays[0][0], b"WAV:" + texts[0].encode())
        self.assertEqual(self.synth_calls, texts, "再生で再合成しない")
        code, st = self.call("GET", "/announce_state")
        self.assertEqual(st["loops"]["pr_intro"]["plays"], 1)
        self.assertIn("prepare", st)
        code, r = self.call("POST", "/announce_loop", {"id": "pr_intro", "enabled": False})
        self.assertFalse(r["loop"]["active"])

    def test_prepare_accepted_when_master_off(self):
        vb.voice_master.set_enabled(False)
        code, r = self.call("POST", "/announce_prepare", {"texts": ["マスターOFFでも作成だけはできる文章。"]})
        self.assertEqual(code, 200)
        self.assertTrue(r["ok"])
        while vb.announce_prepare.process_one():
            pass

    def test_playlist_validation_400(self):
        for body in ({"id": "pr_intro", "parts": [], "gap_sec": 5, "interval_sec": 60},
                     {"id": "pr_intro", "parts": [{"text": "あ" * 401}], "gap_sec": 5, "interval_sec": 60},
                     {"id": "pr_intro", "parts": [{"text": "あ"}], "gap_sec": 0, "interval_sec": 60},
                     {"id": "pr_intro", "parts": [{"text": "あ"}], "gap_sec": 5, "interval_sec": 4000}):
            code, r = self.call("POST", "/announce_loop", body)
            self.assertEqual(code, 400, body)
        self.assertEqual(vb.announce_engine.state()["loops"], {})

    def test_cached_only_once_via_http(self):
        long_text = "とても長い試し再生の文章です。" * 10
        code, r = self.call("POST", "/announce", {"id": "pr_test", "text": long_text, "cached_only": True})
        self.assertEqual(code, 202)
        self.assertEqual(vb.announce_engine.tick(), "not_prepared")
        self.assertEqual(self.synth_calls, [])
        self.assertEqual(self.call("POST", "/announce", {"id": "pr_test", "text": long_text})[0], 400)  # 80字超は従来どおり拒否


if __name__ == "__main__":
    unittest.main()
