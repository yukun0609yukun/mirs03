"""音声の「全体ON/OFF」(マスタースイッチ、2026-10-09追加、docs/voice_master.md)のテスト。

ロボット・スピーカー・Ollama・VOICEVOXには一切触れない(sounddevice/soundfileはスタブ、生成・合成は差し替え)。
1. voice_bridge 側の純ロジック: 永続化(原子的保存・再起動後も維持)、破損/想定外の値のファイルは既定ON、
   保存失敗でも切替は反映(persisted=false+警告)、リクエスト検証(boolのみ)、BGMとの関係(BGM個別設定は
   変えずにゲイン0・ONで復帰)、OFF中の /speak は生成も再生もせず200 muted、お試し生成は409、
   /status・他のAPIは生きている、キュー破棄、生成/合成中にOFFになったら再生しない(失敗カウンタ不変)
2. 1台目 voice_node: マスターOFFの間は発話依頼を送らない/muted応答を失敗扱いにしない
3. VoiceMasterProxy とダッシュボードの /api/voice_master(入力検証、権限: guestは閲覧のみ、
   未ログイン401、2台目不達はエラーJSON、未設定は非表示)
4. 画面(カード・ヘッダーのバッジ・緊急停止ドックが残っていること・JS構文)

実行: python ros2_ws/src/nokosahen_core/test/test_voice_master.py
(rclpy不要。voice_bridgeの依存の requests が入っていない環境ではその部分だけスキップ)
"""

import importlib.util
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import types
import unittest
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest import mock

PKG = Path(__file__).resolve().parents[1]
REPO = PKG.parents[2]
sys.path.insert(0, str(PKG))

TMP = tempfile.mkdtemp(prefix="voice_master_test_")
os.environ["VOICE_UTTERANCE_LOG_DB"] = os.path.join(TMP, "utt.db")
os.environ["VOICE_SETTINGS_FILE"] = os.path.join(TMP, "voice_settings.json")
os.environ["VOICE_MASTER_FILE"] = os.path.join(TMP, "master.json")
os.environ["BGM_ENABLED"] = "0"

# ---- voice_bridge(sounddevice/soundfileはスタブ) ----
SD_STOPS = []
_sd = types.ModuleType("sounddevice")
_sd.PortAudioError = type("PortAudioError", (Exception,), {})
_sd.play = lambda *a, **k: None
_sd.wait = lambda: None
_sd.stop = lambda *a, **k: SD_STOPS.append(1)
_sf = types.ModuleType("soundfile")
_sf.read = lambda path, dtype="float32": ([0.0] * 10, 24000)
sys.modules.setdefault("sounddevice", _sd)
sys.modules.setdefault("soundfile", _sf)
sys.path.insert(0, str(REPO / "voice_bridge"))
try:
    import voice_bridge as vb  # noqa: E402
except ImportError as _exc:  # requests等が無い環境
    vb = None
    _VB_ERR = str(_exc)

# ---- ダッシュボード(rclpy系はスタブ) ----


def _install_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m

    class Msg:
        def __init__(self, *a, **k):
            self.data = None

    mod("geometry_msgs")
    mod("geometry_msgs.msg", PoseStamped=Msg, Twist=Msg)
    mod("nav_msgs")
    mod("nav_msgs.msg", Odometry=Msg)
    mod("std_msgs")
    mod("std_msgs.msg", Bool=Msg, Float32MultiArray=Msg, String=Msg, Empty=Msg)


try:
    import geometry_msgs.msg  # noqa: F401
    import nav_msgs.msg  # noqa: F401
    import std_msgs.msg as _std  # noqa: F401
except Exception:  # noqa: BLE001
    _install_stubs()

from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core.voice_master_proxy import VoiceMasterProxy  # noqa: E402


def http(method, url, body=None, raw=None, headers=None, timeout=10):
    data = raw if raw is not None else (None if body is None else json.dumps(body).encode())
    req = urllib.request.Request(url, data=data, method=method, headers=headers or {})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read().decode("utf-8"))
        finally:
            e.close()


def serve(handler):
    srv = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, srv.server_address[1]


# ======================================================================
# 1. voice_bridge
# ======================================================================
@unittest.skipIf(vb is None, "voice_bridgeを読み込めません(requests未導入?)")
class MasterLogicTest(unittest.TestCase):
    def setUp(self):
        self.path = os.path.join(TMP, "m_logic.json")
        for p in (self.path,):
            if os.path.exists(p):
                os.remove(p)

    def test_default_is_on_without_file(self):
        m = vb.VoiceMaster(self.path)
        self.assertTrue(m.enabled)
        self.assertEqual(m.load_error, "")

    def test_persists_across_restart_and_atomic(self):
        m = vb.VoiceMaster(self.path)
        out = m.set_enabled(False)
        self.assertTrue(out["ok"] and out["changed"] and out["persisted"])
        self.assertFalse(out["enabled"])
        doc = json.loads(Path(self.path).read_text(encoding="utf-8"))
        self.assertEqual(doc["enabled"], False)
        self.assertNotIn("\r", Path(self.path).read_text(encoding="utf-8"))
        # 一時ファイルが残らない
        self.assertEqual([f for f in os.listdir(TMP) if f.startswith(".nokosahen_voice_master_")], [])
        self.assertFalse(vb.VoiceMaster(self.path).enabled)          # 再起動してもOFFのまま
        m.set_enabled(True)
        self.assertTrue(vb.VoiceMaster(self.path).enabled)           # ONに戻せば再起動後もON
        self.assertFalse(m.set_enabled(True)["changed"])             # 同じ値への切替はchanged=false

    def test_corrupt_or_unexpected_file_defaults_to_on(self):
        for content in ("{ broken", "", "[]", "null", '{"enabled": "false"}', '{"enabled": 0}', '{"x": 1}', "\x00\x01"):
            Path(self.path).write_text(content, encoding="utf-8")
            m = vb.VoiceMaster(self.path)
            self.assertTrue(m.enabled, content)
            self.assertTrue(m.load_error, content)
            self.assertEqual(m.status()["load_error"], m.load_error)
        Path(self.path).write_bytes(b"\xff\xfe\x00bad")
        self.assertTrue(vb.VoiceMaster(self.path).enabled)
        # 正常に保存し直すとエラー表示は消える
        m = vb.VoiceMaster(self.path)
        m.set_enabled(False)
        self.assertEqual(m.load_error, "")

    def test_save_failure_still_switches_and_warns(self):
        blocker = os.path.join(TMP, "blocker_file")
        Path(blocker).write_text("x", encoding="utf-8")
        m = vb.VoiceMaster(os.path.join(blocker, "sub", "m.json"))   # 親がファイル -> 書けない
        out = m.set_enabled(False)
        self.assertFalse(m.enabled)                                   # 無音にしたい要求は反映する
        self.assertFalse(out["persisted"])
        self.assertIn("保存できませんでした", out["warning"])

    def test_listener_called_and_failure_isolated(self):
        m = vb.VoiceMaster(self.path)
        seen = []
        m.add_listener(lambda e: (_ for _ in ()).throw(RuntimeError("boom")))
        m.add_listener(seen.append)
        m.set_enabled(False)
        self.assertEqual(seen, [False])                               # 先のリスナーが落ちても次が呼ばれる

    def test_non_bool_rejected_in_logic(self):
        m = vb.VoiceMaster(self.path)
        for bad in ("false", 0, 1, None, "off"):
            with self.assertRaises(TypeError):
                m.set_enabled(bad)
        self.assertTrue(m.enabled)

    def test_parse_master_request(self):
        self.assertEqual(vb.parse_master_request({"enabled": False}), (False, ""))
        self.assertEqual(vb.parse_master_request({"enabled": True}), (True, ""))
        for bad in (None, [], "x", {}, {"enabled": "false"}, {"enabled": 0}, {"enabled": None},
                    {"enabled": True, "x": 1}, {"x": 1}):
            en, err = vb.parse_master_request(bad)
            self.assertIsNone(en, bad)
            self.assertTrue(err, bad)


@unittest.skipIf(vb is None, "voice_bridgeを読み込めません(requests未導入?)")
class BgmRelationTest(unittest.TestCase):
    def test_master_off_silences_bgm_without_touching_its_own_setting(self):
        t = [0.0]
        g = vb.BgmGain(volume=0.3, duck_volume=0.1, clock=lambda: t[0], enabled=True)
        self.assertEqual(g.target(), 0.3)
        g.muted = True
        self.assertEqual(g.target(), 0.0)
        self.assertTrue(g.enabled)                  # 個別設定は変わらない
        self.assertFalse(g.active())
        g.muted = False
        self.assertEqual(g.target(), 0.3)           # ONに戻せば個別設定どおり復帰
        self.assertTrue(g.active())

    def test_master_off_beats_bgm_on_operation(self):
        g = vb.BgmGain(volume=0.3, duck_volume=0.1, enabled=False)
        g.muted = True
        g.enabled = True                            # マスターOFF中にBGM個別をONにしても
        self.assertEqual(g.target(), 0.0)           # 鳴らない
        g.muted = False
        self.assertEqual(g.target(), 0.3)           # マスターONで鳴り出す

    def test_individual_off_stays_off_after_master_on(self):
        g = vb.BgmGain(volume=0.3, duck_volume=0.1, enabled=False)
        g.muted = True
        g.muted = False
        self.assertEqual(g.target(), 0.0)           # 個別OFFだったものはマスターONでも鳴らない

    def test_fade_out_to_zero_when_muted(self):
        g = vb.BgmGain(volume=0.3, duck_volume=0.1, down_sec=0.3, enabled=True)
        g.gain = 0.3
        g.muted = True
        for _ in range(20):
            g.advance(0.1)
        self.assertEqual(g.gain, 0.0)

    def test_player_status_has_muted(self):
        vb.bgm.set_muted(True)
        try:
            st = vb.bgm.status()
            self.assertTrue(st["muted"])
        finally:
            vb.bgm.set_muted(False)


@unittest.skipIf(vb is None, "voice_bridgeを読み込めません(requests未導入?)")
class BridgeHttpTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv, cls.port = serve(vb.make_handler())
        cls.url = f"http://127.0.0.1:{cls.port}"

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def setUp(self):
        self.calls = {"process": 0}
        self._orig_process = vb.SpeechQueue.__dict__["_process_item"]

        def fake_process(item):
            self.calls["process"] += 1
        vb.SpeechQueue._process_item = staticmethod(fake_process)
        vb.voice_master.set_enabled(True)
        vb.voice_master.muted_requests = 0
        vb.speech_queue.clear()
        SD_STOPS.clear()
        # 【2026-11-05】sounddevice のスタブは、最初に voice_bridge を読んだテストファイルのものが sys.modules に残る
        # (setdefault)ので、他のテストと一緒に動かすと、このファイルの _sd.stop が呼ばれない。voice_bridge が実際に使っている
        # vb.sd の stop を、このテストの間だけ差し替える(終わったら元に戻す)。
        self._sd_patch = mock.patch.object(vb.sd, "stop", side_effect=lambda *a, **k: SD_STOPS.append(1))
        self._sd_patch.start()

    def tearDown(self):
        self._sd_patch.stop()
        vb.SpeechQueue._process_item = self._orig_process
        vb.voice_master.set_enabled(True)

    def post(self, path, body=None, raw=None):
        h = {"Content-Type": "application/json"}
        return http("POST", self.url + path, body=body, raw=raw, headers=h)

    def wait_processed(self, n, timeout=3.0):
        import time
        end = time.time() + timeout
        while time.time() < end and self.calls["process"] < n:
            time.sleep(0.02)

    def test_get_and_status(self):
        code, d = http("GET", self.url + "/voice_master")
        self.assertEqual((code, d["ok"], d["enabled"]), (200, True, True))
        self.assertTrue(http("GET", self.url + "/status")[1]["voice_enabled"])

    def test_off_on_roundtrip_and_status(self):
        code, d = self.post("/voice_master", {"enabled": False})
        self.assertEqual((code, d["enabled"], d["changed"]), (200, False, True))
        self.assertFalse(http("GET", self.url + "/voice_master")[1]["enabled"])
        self.assertFalse(http("GET", self.url + "/status")[1]["voice_enabled"])
        self.assertEqual(SD_STOPS, [1])                                   # 再生中の音を止める
        self.assertTrue(vb.bgm.gain.muted)                                # BGMも止まる
        code, d = self.post("/voice_master", {"enabled": True})
        self.assertEqual((code, d["enabled"]), (200, True))
        self.assertFalse(vb.bgm.gain.muted)

    def test_invalid_requests_are_400_and_change_nothing(self):
        for body in ({"enabled": "false"}, {"enabled": 0}, {"enabled": None}, {}, {"enabled": False, "x": 1}):
            code, d = self.post("/voice_master", body)
            self.assertEqual(code, 400, body)
            self.assertFalse(d["ok"])
        self.assertEqual(self.post("/voice_master", raw=b"{bad")[0], 400)
        self.assertEqual(self.post("/voice_master", raw=b"[true]")[0], 400)
        self.assertTrue(vb.voice_master.enabled)

    def test_speak_while_off_is_muted_and_does_nothing(self):
        self.post("/voice_master", {"enabled": False})
        ctx = {"phase": "initial", "menu_items": [{"name": "カレー", "merit": "x"}], "fallback_text": "いらっしゃいませ"}
        with mock.patch.object(vb, "generate_utterance_from_context") as gen, \
                mock.patch.object(vb, "speak_voicevox") as spk, \
                mock.patch.object(vb, "_call_ollama") as oll:
            for body in ({"context": ctx, "generate": True}, {"text": "こんにちは", "generate": False},
                         {"text": "こんにちは", "generate": True}):
                code, d = self.post("/speak", body)
                self.assertEqual(code, 200, body)
                self.assertTrue(d["ok"] and d["muted"])
                self.assertFalse(d["accepted"])
            self.assertEqual(gen.call_count + spk.call_count + oll.call_count, 0)
        self.assertEqual(vb.speech_queue.status()["queue_length"], 0)       # 溜めない
        self.assertEqual(self.calls["process"], 0)
        self.assertEqual(vb.voice_master.status()["muted_requests"], 3)
        self.assertEqual(http("GET", self.url + "/status")[1]["consecutive_speak_failures"],
                         vb.speech_queue.status()["consecutive_speak_failures"])

    def test_invalid_speak_still_400_while_off(self):
        self.post("/voice_master", {"enabled": False})
        self.assertEqual(self.post("/speak", {})[0], 400)

    def test_speak_after_on_works_again(self):
        self.post("/voice_master", {"enabled": False})
        self.post("/speak", {"text": "あ", "generate": False})
        self.post("/voice_master", {"enabled": True})
        code, d = self.post("/speak", {"text": "あ", "generate": False})   # OFF中の依頼で重複抑止されない
        self.assertEqual((code, d), (202, {"ok": True}))
        self.wait_processed(1)
        self.assertEqual(self.calls["process"], 1)

    def test_other_apis_alive_while_off(self):
        self.post("/voice_master", {"enabled": False})
        self.assertEqual(http("GET", self.url + "/status")[0], 200)
        self.assertEqual(http("GET", self.url + "/utterances")[0], 200)
        self.assertEqual(http("GET", self.url + "/voice_settings")[0], 200)
        self.assertEqual(http("GET", self.url + "/bgm")[0], 200)
        self.assertTrue(http("GET", self.url + "/bgm")[1]["muted"])

    def test_bgm_on_while_master_off_does_not_play(self):
        self.post("/voice_master", {"enabled": False})
        code, d = self.post("/bgm", {"enabled": True})
        self.assertEqual(code, 200)
        self.assertTrue(d["enabled"] and d["muted"])
        self.assertEqual(vb.bgm.gain.target(), 0.0)
        self.post("/voice_master", {"enabled": True})
        self.assertGreater(vb.bgm.gain.target(), 0.0)                       # ONに戻すと個別設定どおり復帰
        self.post("/bgm", {"enabled": False})

    def test_preview_is_blocked_while_off(self):
        self.post("/voice_master", {"enabled": False})
        with mock.patch.object(vb, "preview_utterance") as pv:
            code, d = self.post("/voice_preview", {"context": {"phase": "initial", "menu_items": []}})
            self.assertEqual(code, 409)
            self.assertTrue(d["muted"])
            self.assertEqual(pv.call_count, 0)

    def test_queue_cleared_on_off(self):
        gate = threading.Event()

        def slow(item):
            gate.wait(3)
        vb.SpeechQueue._process_item = staticmethod(slow)
        try:
            vb.speech_queue.enqueue("a1", False)          # ワーカーが掴んで待つ
            import time
            time.sleep(0.2)
            vb.speech_queue.enqueue("b2", False)          # キューに残る
            self.assertGreaterEqual(vb.speech_queue.status()["queue_length"], 1)
            self.post("/voice_master", {"enabled": False})
            self.assertEqual(vb.speech_queue.status()["queue_length"], 0)
        finally:
            gate.set()
            time.sleep(0.2)


@unittest.skipIf(vb is None, "voice_bridgeを読み込めません(requests未導入?)")
class MidFlightTest(unittest.TestCase):
    """生成中・合成中にOFFになった場合、再生せず、失敗カウンタも増やさない。"""

    def setUp(self):
        vb.voice_master.set_enabled(True)

    def tearDown(self):
        vb.voice_master.set_enabled(True)

    def test_off_during_generation_discards(self):
        def gen(context, trace=None, settings=None):
            vb.voice_master.set_enabled(False)            # 生成中にOFF
            return "生成された文面"
        with mock.patch.object(vb, "generate_utterance_from_context", side_effect=gen), \
                mock.patch.object(vb, "speak_voicevox") as spk, \
                mock.patch.object(vb, "_record_utterance") as rec:
            vb.SpeechQueue._process_item({"text": "", "generate": True, "context": {"phase": "initial"},
                                          "enqueued_mono": 0.0, "request_id": "t"})
            self.assertEqual(spk.call_count, 0)
            self.assertEqual(rec.call_count, 0)           # 履歴にも残さない

    def test_item_taken_after_off_does_nothing(self):
        vb.voice_master.set_enabled(False)
        with mock.patch.object(vb, "generate_utterance_from_context") as gen, \
                mock.patch.object(vb, "speak_voicevox") as spk:
            vb.SpeechQueue._process_item({"text": "x", "generate": True, "context": {"phase": "initial"}})
            self.assertEqual(gen.call_count + spk.call_count, 0)

    def test_off_during_synthesis_skips_playback_and_keeps_failure_counter(self):
        class Res:
            content = b"RIFF"

            def raise_for_status(self):
                pass

            def json(self):
                return {}

        def post(url, **kw):
            if "synthesis" in url:
                vb.voice_master.set_enabled(False)        # 合成中にOFF
            return Res()
        played = []
        before_fail = vb._consecutive_speak_failures
        with mock.patch.object(vb.requests, "post", side_effect=post), \
                mock.patch.object(vb, "_play_audio", side_effect=lambda *a: played.append(1)):
            trace = {}
            vb.speak_voicevox("テスト", trace)
        self.assertEqual(played, [])
        self.assertTrue(trace.get("muted"))
        self.assertNotIn("speak_error", trace)
        self.assertEqual(vb._consecutive_speak_failures, before_fail)


# ======================================================================
# 2. voice_node
# ======================================================================
def _load_voice_node():
    class Logger:
        def __init__(self):
            self.warns, self.infos, self.debugs = [], [], []

        def warn(self, m):
            self.warns.append(m)

        def info(self, m):
            self.infos.append(m)

        def debug(self, m):
            self.debugs.append(m)

    class Node:
        def __init__(self, name):
            self._log = Logger()
            self._p = {}

        def declare_parameter(self, k, v):
            self._p[k] = v

        def get_parameter(self, k):
            return types.SimpleNamespace(value=self._p[k])

        def create_publisher(self, *a, **k):
            return types.SimpleNamespace(publish=lambda m: None)

        def create_subscription(self, *a, **k):
            return None

        def create_timer(self, *a, **k):
            return None

        def get_logger(self):
            return self._log

    class Msg:
        def __init__(self, *a, **k):
            self.data = None

    saved = {k: sys.modules.get(k) for k in ("rclpy", "rclpy.node", "std_msgs", "std_msgs.msg", "requests")}
    sys.modules["rclpy"] = types.ModuleType("rclpy")
    sys.modules["rclpy.node"] = types.SimpleNamespace(Node=Node)
    if "std_msgs.msg" not in sys.modules:
        sys.modules["std_msgs"] = types.ModuleType("std_msgs")
        sys.modules["std_msgs.msg"] = types.SimpleNamespace(Bool=Msg, String=Msg)
    fake_requests = types.ModuleType("requests")
    fake_requests.post = lambda *a, **k: None
    fake_requests.get = lambda *a, **k: None
    sys.modules["requests"] = fake_requests
    try:
        path = REPO / "ros2_ws" / "src" / "nokosahen_voice" / "nokosahen_voice" / "voice_node.py"
        spec = importlib.util.spec_from_file_location("voice_node_under_test", str(path))
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
        return m, fake_requests
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v


class VoiceNodeTest(unittest.TestCase):
    def setUp(self):
        self.mod, self.req = _load_voice_node()
        self.node = self.mod.VoiceNode()
        self.posts = []
        self.status_body = {"speaking": False, "voice_enabled": True}

        class Res:
            def __init__(s, body):
                s.body = body

            def raise_for_status(s):
                pass

            def json(s):
                return s.body

        self.Res = Res
        self.req.post = lambda url, json=None, timeout=None: (self.posts.append(json), Res({"ok": True}))[1]
        self.req.get = lambda url, timeout=None: Res(self.status_body)

    def msg(self, text):
        return types.SimpleNamespace(data=json.dumps({"phase": "initial", "menu_items": [], "n": text}))

    def test_sends_when_on(self):
        self.node._on_announce_context(self.msg("a"))
        self.assertEqual(len(self.posts), 1)

    def test_does_not_send_when_off(self):
        self.status_body = {"speaking": False, "voice_enabled": False}
        self.node._poll_status()
        self.node._on_announce_context(self.msg("a"))
        self.assertEqual(self.posts, [])
        self.assertEqual(self.node.get_logger().warns, [])                  # ログを荒らさない
        self.status_body = {"speaking": False, "voice_enabled": True}
        self.node._poll_status()
        self.node._on_announce_context(self.msg("b"))
        self.assertEqual(len(self.posts), 1)                                # ONに戻れば再開

    def test_muted_response_is_not_a_failure(self):
        self.req.post = lambda url, json=None, timeout=None: self.Res({"ok": True, "muted": True})
        self.node._on_announce_context(self.msg("a"))
        self.assertEqual(self.node.get_logger().warns, [])

    def test_old_bridge_without_field_keeps_sending(self):
        self.status_body = {"speaking": False}
        self.node._poll_status()
        self.node._on_announce_context(self.msg("a"))
        self.assertEqual(len(self.posts), 1)

    def test_real_failure_still_warns(self):
        def boom(*a, **k):
            raise OSError("down")
        self.req.post = boom
        self.node._on_announce_context(self.msg("a"))
        self.assertEqual(len(self.node.get_logger().warns), 1)

    def test_unreachable_status_keeps_last_state(self):
        self.status_body = {"speaking": False, "voice_enabled": False}
        self.node._poll_status()

        def boom(*a, **k):
            raise OSError("down")
        self.req.get = boom
        self.node._poll_status()
        self.node._on_announce_context(self.msg("a"))
        self.assertEqual(self.posts, [])


# ======================================================================
# 3. proxy + dashboard
# ======================================================================
class _Bridge(BaseHTTPRequestHandler):
    state = {"ok": True, "enabled": True, "persisted": True, "muted_requests": 0, "load_error": ""}
    posts = []
    fail = 0
    garbage = False

    def log_message(self, *a):
        pass

    def _send(self, code, obj):
        body = obj if isinstance(obj, bytes) else json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.fail:
            self._send(self.fail, {})
        elif self.garbage:
            self._send(200, b"not json")
        elif self.path == "/voice_master":
            self._send(200, type(self).state)
        else:
            self._send(404, {})

    def do_POST(self):
        n = int(self.headers.get("Content-Length", "0"))
        data = json.loads(self.rfile.read(n))
        type(self).posts.append(data)
        if self.fail:
            self._send(self.fail, {})
            return
        type(self).state = dict(type(self).state, enabled=data["enabled"])
        self._send(200, type(self).state)


class ProxyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv, cls.port = serve(_Bridge)
        cls.url = f"http://127.0.0.1:{cls.port}"

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def setUp(self):
        _Bridge.state = {"ok": True, "enabled": True, "persisted": True, "muted_requests": 0, "load_error": ""}
        _Bridge.posts = []
        _Bridge.fail = 0
        _Bridge.garbage = False
        self.p = VoiceMasterProxy(self.url)

    def test_get_and_set(self):
        r = self.p.get_status()
        self.assertTrue(r["ok"] and r["configured"])
        self.assertTrue(r["status"]["enabled"])
        code, r = self.p.set({"enabled": False})
        self.assertEqual((code, r["ok"], r["status"]["enabled"]), (200, True, False))
        self.assertEqual(_Bridge.posts, [{"enabled": False}])

    def test_validation_never_forwards(self):
        for bad in (None, [], "x", 1, {}, {"enabled": "false"}, {"enabled": 1}, {"enabled": None},
                    {"enabled": True, "volume": 1}, {"volume": 1}):
            code, r = self.p.set(bad)
            self.assertEqual(code, 400, bad)
            self.assertFalse(r["ok"])
            self.assertTrue(r["message"])
        self.assertEqual(_Bridge.posts, [])

    def test_unreachable_is_error_json(self):
        dead = VoiceMasterProxy("http://127.0.0.1:1", timeout_s=0.5)
        r = dead.get_status()
        self.assertFalse(r["ok"])
        self.assertIn("接続できません", r["message"])
        code, r = dead.set({"enabled": False})
        self.assertEqual((code, r["ok"]), (200, False))

    def test_http_error_garbage_and_old_bridge(self):
        _Bridge.fail = 500
        self.assertIn("HTTP 500", self.p.get_status()["message"])
        _Bridge.fail = 404
        self.assertIn("未対応", self.p.get_status()["message"])
        _Bridge.fail = 0
        _Bridge.garbage = True
        self.assertFalse(self.p.get_status()["ok"])
        _Bridge.garbage = False
        _Bridge.state = {"ok": True}                       # enabledが無い応答(想定外)も不達扱い
        self.assertFalse(self.p.get_status()["ok"])

    def test_unconfigured(self):
        p = VoiceMasterProxy("")
        self.assertFalse(p.get_status()["configured"])
        code, r = p.set({"enabled": False})
        self.assertFalse(r["configured"])
        self.assertEqual(_Bridge.posts, [])

    def test_save_warning_passes_through(self):
        _Bridge.state = dict(_Bridge.state, warning="状態を保存できませんでした")
        code, r = self.p.set({"enabled": False})
        self.assertTrue(r["ok"])
        self.assertIn("保存できません", r["message"])


class FakeTeleop:
    soft_estop_engaged = False


class DashboardApiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge, cls.bport = serve(_Bridge)

    @classmethod
    def tearDownClass(cls):
        cls.bridge.shutdown()
        cls.bridge.server_close()

    def setUp(self):
        _Bridge.state = {"ok": True, "enabled": True, "persisted": True, "muted_requests": 0, "load_error": ""}
        _Bridge.posts = []
        _Bridge.fail = 0
        _Bridge.garbage = False
        self.proxy = VoiceMasterProxy(f"http://127.0.0.1:{self.bport}")
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, FakeTeleop(), voice_master=self.proxy,
            auth_username="TL", auth_password="pw", enforce_single_controller=False)
        self.srv, self.port = serve(handler)
        self.tl = self.login("TL", "pw")
        self.guest = self.login("guest", "")

    def tearDown(self):
        self.srv.shutdown()
        self.srv.server_close()

    def login(self, user, pw):
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.port}/api/login", data=json.dumps({"username": user, "password": pw}).encode(),
            method="POST", headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.headers.get("Set-Cookie", "").split(";")[0]

    def call(self, method, path, body=None, cookie=None):
        h = {"Content-Type": "application/json"}
        c = self.tl if cookie is None else cookie
        if c:
            h["Cookie"] = c
        return http(method, f"http://127.0.0.1:{self.port}{path}", body=body, headers=h)

    def test_requires_login(self):
        self.assertEqual(self.call("GET", "/api/voice_master", cookie="")[0], 401)
        self.assertEqual(self.call("POST", "/api/voice_master", {"enabled": False}, cookie="")[0], 401)
        self.assertEqual(_Bridge.posts, [])

    def test_guest_can_view_but_not_switch(self):
        for ck in (self.tl, self.guest):
            code, d = self.call("GET", "/api/voice_master", cookie=ck)
            self.assertEqual((code, d["ok"], d["status"]["enabled"]), (200, True, True))
        self.assertEqual(self.call("POST", "/api/voice_master", {"enabled": False}, cookie=self.guest)[0], 403)
        self.assertEqual(_Bridge.posts, [])

    def test_tl_switch_and_validation(self):
        code, d = self.call("POST", "/api/voice_master", {"enabled": False})
        self.assertEqual((code, d["ok"], d["status"]["enabled"]), (200, True, False))
        self.assertEqual(self.call("POST", "/api/voice_master", {"enabled": "false"})[0], 400)
        self.assertEqual(self.call("POST", "/api/voice_master", {})[0], 400)
        self.assertEqual(_Bridge.posts, [{"enabled": False}])
        self.assertFalse(self.call("GET", "/api/voice_master")[1]["status"]["enabled"])

    def test_bridge_down_is_error_json(self):
        self.proxy.base_url = "http://127.0.0.1:1"
        self.proxy.timeout_s = 0.5
        code, d = self.call("GET", "/api/voice_master")
        self.assertEqual((code, d["ok"]), (200, False))
        self.assertIn("接続できません", d["message"])
        code, d = self.call("POST", "/api/voice_master", {"enabled": False})
        self.assertEqual((code, d["ok"]), (200, False))

    def test_unconfigured_hides(self):
        self.proxy.base_url = ""
        self.assertFalse(self.call("GET", "/api/voice_master")[1]["configured"])


# ======================================================================
# 4. 画面
# ======================================================================
class PageTest(unittest.TestCase):
    def setUp(self):
        src = (PKG / "nokosahen_core" / "full_dashboard_node.py").read_text(encoding="utf-8")
        self.html = src[:src.index("MOBILE_HTML = ")]

    def test_card_in_voice_tab_before_dock(self):
        h = self.html
        i_tab = h.index('id="tab-voice"')
        i_card = h.index('id="voiceMasterBlock"')
        self.assertGreater(i_card, i_tab)
        self.assertLess(i_card, h.index('id="tab-logs"'))
        self.assertLess(i_card, h.index('id="estopDock"'))
        for el in ("voiceMasterState", "voiceMasterDetail", "voiceMasterBtn", "voiceMasterConfirmBar",
                   "voiceMasterYes", "voiceMasterNo", "voiceMasterHint"):
            self.assertIn(f'id="{el}"', h)
            self.assertEqual(h.count(f'id="{el}"'), 1)
        self.assertIn("/api/voice_master", h)
        self.assertIn('id="estopDock"', h)

    def test_header_chip_and_dock_untouched(self):
        nav = (PKG / "nokosahen_core" / "nav_layout.py").read_text(encoding="utf-8")
        self.assertIn('id="voiceOffChip"', nav)
        self.assertIn("#estopDock { position:fixed;", nav)
        self.assertIn("z-index:2000", nav)

    def test_crlf_preserved(self):
        for name in ("dashboard_common.py", "full_dashboard_node.py", "nav_layout.py"):
            b = (PKG / "nokosahen_core" / name).read_bytes()
            self.assertEqual(b.count(b"\r\n"), b.count(b"\n"), name)
        for name in ("voice_master_proxy.py",):
            b = (PKG / "nokosahen_core" / name).read_bytes()
            self.assertNotIn(b"\r", b, name)

    def test_js_syntax(self):
        node = shutil.which("node")
        if not node:
            self.skipTest("nodeが無い")
        scripts = []
        rest = self.html
        while "<script>" in rest:
            a = rest.index("<script>") + len("<script>")
            b = rest.index("</script>", a)
            scripts.append(rest[a:b])
            rest = rest[b:]
        js = [s for s in scripts if "loadVoiceMaster" in s]
        self.assertEqual(len(js), 1)
        p = Path(TMP) / "page.js"
        p.write_text(js[0].replace("__NAV_JS__", ""), encoding="utf-8")
        r = subprocess.run([node, "--check", str(p)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)


def tearDownModule():
    shutil.rmtree(TMP, ignore_errors=True)


if __name__ == "__main__":
    unittest.main(verbosity=1)
