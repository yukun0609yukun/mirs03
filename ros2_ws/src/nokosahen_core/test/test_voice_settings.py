"""呼びかけ音声設定ページ(/voice_settings)のダッシュボード側のテスト。

ROS 2が無いPCでも動くよう、rclpy系モジュールをスタブしてdashboard_commonを読み込み、
実際にHTTPサーバーを立てて /api/voice_settings* を叩く(test_behavior_rules_api.pyと同じ方式)。
2台目のvoice_bridgeは http.server のスタブで置き換える(本物のOllama/VOICEVOX/Jetsonには触れない)。
確認すること:
- 発話間隔ファイル(announce_settings): 検証(範囲/NaN/型)・原子的保存+.bak・directorのホットリロード
  (有効値の反映/不正なファイルは直前の値を維持/ファイル削除でパラメータ値へ戻る)
- プロキシ(voice_settings_proxy): 検証(型・範囲・NaN・空/長いプロンプト・未知キー)、
  voice_bridge.pyの範囲定義との一致、接続不可時の表示、HTTPエラーの中継
- ルート: guestは閲覧可・保存/お試し/試し発話は403、未ログイン401、NaN/範囲外は400で何も書かない、
  2台目が落ちていても発話間隔は保存できる、お試し生成は現在のメニュー状態を材料にする
ロボットには何も送らない(cmd_velもゴールも出さない)。

実行: python ros2_ws/src/nokosahen_core/test/test_voice_settings.py
"""

import importlib.util
import json
import os
import socket
import sys
import tempfile
import threading
import types
import unittest
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
PKG = TEST_DIR.parent
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(TEST_DIR))


def _install_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

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

from nokosahen_core import announce_settings as ann  # noqa: E402
from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import voice_settings_proxy as vsp  # noqa: E402
from nokosahen_core.voice_settings_controller import (  # noqa: E402
    VoiceSettingsController,
    build_preview_context,
)
from nokosahen_core.voice_settings_page import build_voice_settings_html  # noqa: E402

import test_behavior_rules as tbr  # noqa: E402  (directorの組み立てを流用)


def closed_port_url():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return f"http://127.0.0.1:{port}"


class FakeBridge:
    """voice_bridgeのスタブ(設定/計測/お試し/話す)。受け取ったリクエストを記録する。"""

    def __init__(self):
        self.calls = []
        self.settings = {
            "include_menu_names": True, "include_remaining": True, "temperature": 0.7,
            "num_predict": 400, "max_chars": 0, "ollama_timeout_sec": 60.0, "min_repeat_sec": 45.0,
            "persona_prompt": "標準のペルソナ",
        }
        self.post_status = 200
        self.post_body = None  # 上書き応答
        outer = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _reply(self, status, body):
                data = json.dumps(body).encode()
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):
                outer.calls.append(("GET", self.path, None))
                if self.path == "/voice_settings":
                    self._reply(200, {
                        "ok": True, "settings": outer.settings, "defaults": outer.settings,
                        "sources": {k: "default" for k in outer.settings},
                        "limits": {"temperature": {"min": 0.0, "max": 2.0}},
                        "model": "gemma2:2b", "settings_file": "/x/voice.json", "load_error": "",
                    })
                elif self.path.startswith("/voice_metrics"):
                    if "id=" in self.path:
                        self._reply(200, {"ok": True, "row": {"id": 7, "text": "t", "prompt": "<設定>"}})
                    else:
                        self._reply(200, {"ok": True, "limit": 50, "rows": [], "stats": {"rows": 0}})
                else:
                    self._reply(404, {})

            def do_POST(self):
                n = int(self.headers.get("Content-Length", "0"))
                body = json.loads(self.rfile.read(n).decode()) if n else None
                outer.calls.append(("POST", self.path, body))
                if outer.post_body is not None:
                    self._reply(outer.post_status, outer.post_body)
                    return
                if self.path == "/voice_preview":
                    self._reply(200, {"ok": True, "text": "生成文", "chars": 3, "llm_ms": 1234.5,
                                      "used_menu_names": True, "used_remaining": False,
                                      "fallback_reason": None, "model": "gemma2:2b", "prompt": "P"})
                elif self.path == "/speak":
                    self._reply(202, {"ok": True})
                else:
                    self._reply(outer.post_status, {"ok": True, "settings": outer.settings})

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), H)
        self.url = f"http://127.0.0.1:{self.server.server_address[1]}"
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def posts(self, path):
        return [b for m, p, b in self.calls if m == "POST" and p == path]

    def close(self):
        self.server.shutdown()
        self.server.server_close()


class FakeTeleop:
    soft_estop_engaged = False


class AnnounceSettingsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.tmp.name, "announce.json")

    def tearDown(self):
        self.tmp.cleanup()

    def test_parse_interval(self):
        for ok in (10, 60, 600, 90.5):
            self.assertEqual(ann.parse_interval(ok)[1], None, ok)
        for bad in (9.99, 601, -1, 0, float("nan"), float("inf"), "60", None, True, [60]):
            self.assertIsNotNone(ann.parse_interval(bad)[1], bad)

    def test_store_save_read_backup(self):
        store = ann.AnnounceSettingsStore(self.path)
        st = store.read()
        self.assertEqual((st["exists"], st["announce_min_interval_seconds"], st["valid"]), (False, None, True))
        self.assertTrue(store.save_interval(45)["ok"])
        self.assertEqual(store.read()["announce_min_interval_seconds"], 45)
        self.assertFalse(os.path.exists(store.backup_path))
        self.assertTrue(store.save_interval(120)["ok"])
        self.assertEqual(json.loads(Path(store.backup_path).read_text())["announce_min_interval_seconds"], 45)
        r = store.save_interval(5)
        self.assertFalse(r["ok"])
        self.assertEqual(store.read()["announce_min_interval_seconds"], 120)  # 不正なら何も変えない
        self.assertEqual([n for n in os.listdir(self.tmp.name) if n.endswith(".tmp")], [])

    def test_read_reports_broken_files(self):
        store = ann.AnnounceSettingsStore(self.path)
        for text in ("{bad", '{"announce_min_interval_seconds": NaN}', '{"announce_min_interval_seconds": 3}',
                     '{"unknown": 1}', "[1]"):
            Path(self.path).write_text(text, encoding="utf-8")
            st = store.read()
            self.assertFalse(st["valid"], text)
            self.assertTrue(st["errors"], text)

    def test_reader_hot_reload(self):
        reader = ann.AnnounceSettingsReader(self.path)
        self.assertEqual(reader.poll(), (None, "", False))  # ファイル無し=パラメータ値
        Path(self.path).write_text(json.dumps({"version": 1, "announce_min_interval_seconds": 90}), encoding="utf-8")
        v, msg, changed = reader.poll()
        self.assertEqual((v, changed), (90, True))
        self.assertIn("90", msg)
        self.assertEqual(reader.poll(), (90, "", False))  # 変化なしなら何もしない
        Path(self.path).write_text('{"announce_min_interval_seconds": 5}', encoding="utf-8")  # 範囲外
        v, msg, changed = reader.poll()
        self.assertEqual((v, changed), (90, False))  # 直前の有効値を維持
        self.assertTrue(msg and reader.load_error)
        self.assertEqual(reader.poll()[1], "")  # 同じ不正内容では二度は警告しない
        Path(self.path).write_text('{"announce_min_interval_seconds": 30}', encoding="utf-8")
        self.assertEqual(reader.poll()[0], 30)
        self.assertEqual(reader.load_error, "")
        os.remove(self.path)
        v, msg, changed = reader.poll()
        self.assertEqual((v, changed), (None, True))  # 消したらパラメータ値へ戻る


class DirectorHotReloadTests(unittest.TestCase):
    def setUp(self):
        self.helper = tbr.DirectorTickTests("run_ticks")
        self.helper.setUp()
        self.addCleanup(self.helper.tearDown)
        self.n = self.helper.make()
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = os.path.join(self.tmp.name, "announce.json")
        self.n._announce_param_interval = 60.0
        self.n._announce_reader = ann.AnnounceSettingsReader(self.path)

    def write(self, value):
        Path(self.path).write_text(json.dumps({"version": 1, "announce_min_interval_seconds": value}), encoding="utf-8")

    def test_interval_follows_file_without_restart(self):
        n = self.n
        n._refresh_announce_settings()
        self.assertEqual(n.announce_min_interval_seconds, 60.0)  # ファイル無し=パラメータ値(現行動作)
        self.write(120)
        n._refresh_announce_settings()
        self.assertEqual(n.announce_min_interval_seconds, 120.0)
        self.write(15)
        n._refresh_announce_settings()
        self.assertEqual(n.announce_min_interval_seconds, 15.0)
        # cooldown(8秒)より小さい値は取り得ない設計だが、下限10秒>8秒で常に満たす。念のためcooldown超過時もmax
        n.announce_cooldown_seconds = 20.0
        self.write(25)
        n._refresh_announce_settings()
        self.assertEqual(n.announce_min_interval_seconds, 25.0)

    def test_invalid_file_keeps_previous_and_warns(self):
        n = self.n
        self.write(100)
        n._refresh_announce_settings()
        Path(self.path).write_text("{broken", encoding="utf-8")
        n._refresh_announce_settings()
        self.assertEqual(n.announce_min_interval_seconds, 100.0)
        self.write(9999)  # 範囲外
        n._refresh_announce_settings()
        self.assertEqual(n.announce_min_interval_seconds, 100.0)
        self.assertTrue(any(k == "warn" for k, _m in n._log.lines))
        os.remove(self.path)
        n._refresh_announce_settings()
        self.assertEqual(n.announce_min_interval_seconds, 60.0)

    def test_new_interval_gates_announcements(self):
        """実際の発話依頼の間隔に効くこと(_publish_announce_if_needed)。"""
        n = self.n
        self.write(30)
        n._refresh_announce_settings()
        pub = n._announce_context_pub
        before = len(pub.msgs) if hasattr(pub, "msgs") else None
        n._publish_announce_if_needed("A")  # 起動後の最初の1回は即時
        n._clock.t += 29
        n._publish_announce_if_needed("A")
        n._clock.t += 2
        n._publish_announce_if_needed("A")
        if before is not None:
            self.assertEqual(len(pub.msgs) - before, 2)  # 最初と31秒後の2回だけ

    def test_no_reader_is_noop(self):
        n = self.helper.make()
        n._refresh_announce_settings()  # _announce_reader無し(既存テスト環境)でも落ちない
        self.assertEqual(n.announce_min_interval_seconds, 60.0)


class ProxyValidationTests(unittest.TestCase):
    def test_valid(self):
        p, e = vsp.validate_voice_settings({
            "include_menu_names": False, "temperature": 1, "num_predict": 300.0, "max_chars": 0,
            "persona_prompt": " 文 ", "ollama_timeout_sec": 30, "min_repeat_sec": 0, "include_remaining": True})
        self.assertEqual(e, [])
        self.assertEqual(p["persona_prompt"], "文")
        self.assertIsInstance(p["num_predict"], int)

    def test_invalid(self):
        bad = [
            {"include_menu_names": 1}, {"temperature": float("nan")}, {"temperature": float("inf")},
            {"temperature": -0.1}, {"temperature": 2.1}, {"temperature": "0.7"}, {"temperature": True},
            {"num_predict": 31}, {"num_predict": 100.5}, {"max_chars": 501}, {"ollama_timeout_sec": 4},
            {"min_repeat_sec": 601}, {"persona_prompt": ""}, {"persona_prompt": "  "},
            {"persona_prompt": "あ" * 4001}, {"persona_prompt": 3}, {"x": 1},
        ]
        for b in bad:
            p, e = vsp.validate_voice_settings(b)
            self.assertTrue(e, b)
            self.assertEqual(p, {}, b)
        self.assertTrue(vsp.validate_voice_settings([1])[1])
        self.assertEqual(vsp.validate_voice_settings({"persona_prompt": "あ" * 4000})[1], [])
        self.assertEqual(vsp.validate_voice_settings({"persona_prompt": None})[0], {"persona_prompt": None})

    def test_limits_match_voice_bridge(self):
        """プロキシの範囲定義がvoice_bridge.pyと一致していること(片方だけ直した事故を防ぐ)。"""
        vb_path = PKG.parents[2] / "voice_bridge" / "voice_bridge.py"
        if not vb_path.exists():
            self.skipTest("voice_bridge.pyが見つかりません")
        env_backup = {k: os.environ.get(k) for k in ("VOICE_UTTERANCE_LOG_DB", "VOICE_SETTINGS_FILE")}
        tmp = tempfile.mkdtemp(prefix="vsp_sync_")
        os.environ["VOICE_UTTERANCE_LOG_DB"] = os.path.join(tmp, "u.db")
        os.environ["VOICE_SETTINGS_FILE"] = os.path.join(tmp, "s.json")
        stubs = {}
        for name in ("sounddevice", "soundfile"):
            if name not in sys.modules:
                stubs[name] = sys.modules[name] = types.ModuleType(name)
        try:
            try:
                spec = importlib.util.spec_from_file_location("voice_bridge_for_sync_test", vb_path)
                vb = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(vb)
            except Exception as exc:  # noqa: BLE001  (requests未導入など)
                self.skipTest(f"voice_bridgeを読み込めません: {exc}")
        finally:
            for name in stubs:
                sys.modules.pop(name, None)
            for k, v in env_backup.items():
                if v is None:
                    os.environ.pop(k, None)
                else:
                    os.environ[k] = v
        self.assertEqual(dict(vb.NUMERIC_LIMITS), dict(vsp.NUMERIC_LIMITS))
        self.assertEqual(vb.PERSONA_MAX_CHARS, vsp.PERSONA_MAX_CHARS)
        self.assertEqual(tuple(vb.SETTING_KEYS), tuple(vsp.SETTING_KEYS))
        self.assertEqual(tuple(vb.BOOL_SETTINGS), tuple(vsp.BOOL_KEYS))


class ProxyTransportTests(unittest.TestCase):
    def test_unconfigured(self):
        px = vsp.VoiceSettingsProxy("")
        self.assertFalse(px.configured)
        r = px.get_settings()
        self.assertEqual((r["ok"], r["configured"]), (False, False))

    def test_unreachable_message(self):
        px = vsp.VoiceSettingsProxy(closed_port_url(), timeout_s=1.0, metrics_timeout_s=1.0)
        for r in (px.get_settings(), px.get_metrics(50), px.save_settings({"temperature": 1.0})):
            self.assertFalse(r["ok"])
            self.assertEqual(r["kind"], "unreachable")
            self.assertTrue(r["error"].startswith("2台目に接続できません"))

    def test_http_error_relays_japanese_errors(self):
        b = FakeBridge()
        self.addCleanup(b.close)
        b.post_status = 400
        b.post_body = {"ok": False, "errors": ["temperatureは0.0〜2.0の範囲で指定してください"]}
        r = vsp.VoiceSettingsProxy(b.url).save_settings({"temperature": 1.0})
        self.assertEqual((r["ok"], r["kind"]), (False, "http"))
        self.assertIn("temperature", r["errors"][0])

    def test_timeouts_are_as_specified(self):
        px = vsp.VoiceSettingsProxy("http://x")
        self.assertEqual((px.timeout_s, px.metrics_timeout_s), (3.0, 8.0))


class ApiBase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.announce_path = os.path.join(self.tmp.name, "announce.json")
        self.bridge = FakeBridge()
        self.menu_state = {
            "phase": "induction", "promote_item": "カレー",
            "items": [{"name": "カレー", "initial": 10, "remaining": 8}, {"name": "A定食", "initial": 10, "remaining": 2}],
        }
        self.start_server(self.bridge.url)

    def start_server(self, url, controller=True):
        self.ctrl = VoiceSettingsController(ann.AnnounceSettingsStore(self.announce_path), vsp.VoiceSettingsProxy(url))
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>",
            lambda: {"menu_state": self.menu_state},
            FakeTeleop(),
            voice_settings=self.ctrl if controller else None,
            voice_settings_html_getter=build_voice_settings_html if controller else None,
            auth_username="TL",
            auth_password="pw",
            enforce_single_controller=False,
        )
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        self.port = self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.tl = self.login("TL", "pw")
        self.guest = self.login("guest", "")

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.bridge.close()
        self.tmp.cleanup()

    def login(self, user, password):
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.port}/api/login",
            data=json.dumps({"username": user, "password": password}).encode(),
            method="POST", headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.headers.get("Set-Cookie", "").split(";")[0]

    def call(self, method, path, body=None, raw=None, cookie=None, text=False):
        cookie = self.tl if cookie is None else cookie
        data = raw if raw is not None else (None if body is None else json.dumps(body).encode())
        headers = {"Content-Type": "application/json"}
        if cookie:
            headers["Cookie"] = cookie
        req = urllib.request.Request(f"http://127.0.0.1:{self.port}{path}", data=data, method=method, headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=15) as r:
                payload = r.read().decode("utf-8")
                return r.status, (payload if text else json.loads(payload))
        except urllib.error.HTTPError as e:
            try:
                payload = e.read().decode("utf-8")
                return e.code, (payload if text else json.loads(payload))
            finally:
                e.close()

    def save(self, body, **kw):
        return self.call("POST", "/api/voice_settings/save", body, **kw)


class ReadTests(ApiBase):
    def test_page_served_with_dock_and_tabs(self):
        st, html = self.call("GET", "/voice_settings", text=True)
        self.assertEqual(st, 200)
        for needle in ('id="estopDock"', "呼びかけ音声設定", "基本設定", "プロンプト", "実績(生成時間)",
                       "標準に戻す", "現在のプロンプトで試す", "各時間の意味"):
            self.assertIn(needle, html)

    def test_unauthenticated_is_401(self):
        self.assertEqual(self.call("GET", "/api/voice_settings", cookie="")[0], 401)
        self.assertEqual(self.save({"announce": {"announce_min_interval_seconds": 60}}, cookie="")[0], 401)

    def test_get_payload_merges_both_machines(self):
        st, d = self.call("GET", "/api/voice_settings")
        self.assertEqual(st, 200)
        self.assertTrue(d["configured"])
        self.assertIsNone(d["announce"]["announce_min_interval_seconds"])
        self.assertTrue(d["voice"]["ok"])
        self.assertEqual(d["voice"]["settings"]["persona_prompt"], "標準のペルソナ")
        self.assertEqual(d["voice"]["model"], "gemma2:2b")

    def test_unreachable_bridge_still_returns_announce(self):
        self.server.shutdown()
        self.server.server_close()
        self.start_server(closed_port_url())
        st, d = self.call("GET", "/api/voice_settings")
        self.assertEqual(st, 200)
        self.assertFalse(d["voice"]["ok"])
        self.assertIn("2台目に接続できません", d["voice"]["error"])
        self.assertIn("announce", d)
        st, m = self.call("GET", "/api/voice_settings/metrics?limit=50")
        self.assertFalse(m["ok"])
        self.assertIn("2台目に接続できません", m["error"])
        # それでも発話間隔は保存できる
        st, r = self.save({"announce": {"announce_min_interval_seconds": 75}})
        self.assertEqual(st, 200, r)
        self.assertEqual(json.loads(Path(self.announce_path).read_text())["announce_min_interval_seconds"], 75)

    def test_metrics_proxy_and_validation(self):
        st, d = self.call("GET", "/api/voice_settings/metrics?limit=200")
        self.assertEqual((st, d["ok"]), (200, True))
        self.assertIn(("GET", "/voice_metrics?limit=200", None), self.bridge.calls)
        st, d = self.call("GET", "/api/voice_settings/metrics?id=7")
        self.assertEqual(d["row"]["prompt"], "<設定>")
        for q in ("limit=0", "limit=501", "limit=abc", "id=x"):
            self.assertEqual(self.call("GET", "/api/voice_settings/metrics?" + q)[0], 400, q)
        self.assertEqual(self.call("GET", "/api/voice_settings/metrics?limit=50", cookie=self.guest)[0], 200)

    def test_feature_hidden_when_controller_not_registered(self):
        self.server.shutdown()
        self.server.server_close()
        self.start_server("", controller=False)
        self.assertEqual(self.call("GET", "/api/voice_settings")[0], 404)
        self.assertEqual(self.save({"announce": {"announce_min_interval_seconds": 60}})[0], 404)

    def test_unconfigured_url_shows_configured_false(self):
        self.server.shutdown()
        self.server.server_close()
        self.start_server("")
        st, d = self.call("GET", "/api/voice_settings")
        self.assertFalse(d["configured"])
        self.assertIsNone(d["voice"])
        st, r = self.save({"voice": {"temperature": 1.0}})
        self.assertEqual(st, 400)


class GuestTests(ApiBase):
    def test_guest_can_read_but_not_write(self):
        self.assertEqual(self.call("GET", "/voice_settings", cookie=self.guest, text=True)[0], 200)
        self.assertEqual(self.call("GET", "/api/voice_settings", cookie=self.guest)[0], 200)
        st, _ = self.save({"announce": {"announce_min_interval_seconds": 60}, "voice": {"temperature": 1.0}}, cookie=self.guest)
        self.assertEqual(st, 403)
        self.assertFalse(os.path.exists(self.announce_path))
        self.assertEqual(self.bridge.posts("/voice_settings"), [])
        self.assertEqual(self.call("POST", "/api/voice_settings/preview", {}, cookie=self.guest)[0], 403)
        self.assertEqual(self.call("POST", "/api/voice_settings/speak", {"text": "x"}, cookie=self.guest)[0], 403)
        self.assertEqual(self.bridge.posts("/voice_preview"), [])
        self.assertEqual(self.bridge.posts("/speak"), [])


class SaveTests(ApiBase):
    def test_save_both(self):
        st, d = self.save({"announce": {"announce_min_interval_seconds": 90},
                           "voice": {"temperature": 0.9, "include_remaining": False, "persona_prompt": "新ペルソナ"}})
        self.assertEqual((st, d["ok"]), (200, True), d)
        self.assertEqual(json.loads(Path(self.announce_path).read_text())["announce_min_interval_seconds"], 90)
        self.assertEqual(self.bridge.posts("/voice_settings"),
                         [{"temperature": 0.9, "include_remaining": False, "persona_prompt": "新ペルソナ"}])
        self.assertIn("2台目", d["message"])
        # 保存後の読み直し(ページが行う)で新しい値が見える
        self.assertEqual(self.call("GET", "/api/voice_settings")[1]["announce"]["announce_min_interval_seconds"], 90)

    def test_validation_errors_write_nothing(self):
        bad_bodies = [
            {"announce": {"announce_min_interval_seconds": 5}},
            {"announce": {"announce_min_interval_seconds": 601}},
            {"announce": {"announce_min_interval_seconds": "60"}},
            {"announce": {"x": 1}},
            {"voice": {"temperature": 9}},
            {"voice": {"persona_prompt": ""}},
            {"voice": {"persona_prompt": "あ" * 4001}},
            {"voice": {"nope": 1}},
            {"voice": {}},
            {"voice": [1]},
            {},
            {"announce": {"announce_min_interval_seconds": 60}, "voice": {"temperature": 99}},  # 片方不正なら両方書かない
        ]
        for b in bad_bodies:
            st, d = self.save(b)
            self.assertEqual(st, 400, b)
            self.assertFalse(d["ok"], b)
            self.assertTrue(d["errors"], b)
        self.assertFalse(os.path.exists(self.announce_path))
        self.assertEqual(self.bridge.posts("/voice_settings"), [])

    def test_nan_and_infinity_are_400(self):
        for raw in (b'{"announce": {"announce_min_interval_seconds": NaN}}',
                    b'{"voice": {"temperature": NaN}}',
                    b'{"voice": {"temperature": Infinity}}',
                    b'{"announce": {"announce_min_interval_seconds": -Infinity}}'):
            st, d = self.call("POST", "/api/voice_settings/save", raw=raw)
            self.assertEqual(st, 400, raw)
        self.assertFalse(os.path.exists(self.announce_path))
        self.assertEqual(self.bridge.posts("/voice_settings"), [])

    def test_bridge_down_saves_interval_and_reports(self):
        self.server.shutdown()
        self.server.server_close()
        self.start_server(closed_port_url())
        st, d = self.save({"announce": {"announce_min_interval_seconds": 100}, "voice": {"temperature": 1.0}})
        self.assertEqual(st, 502)
        self.assertFalse(d["ok"])
        self.assertTrue(d["announce"]["ok"])  # 1台目の保存は有効
        self.assertFalse(d["voice"]["ok"])
        self.assertIn("2台目に接続できません", d["voice"]["error"])
        self.assertEqual(json.loads(Path(self.announce_path).read_text())["announce_min_interval_seconds"], 100)

    def test_bridge_http_error_is_relayed(self):
        self.bridge.post_status = 400
        self.bridge.post_body = {"ok": False, "errors": ["相手側の検証エラー"]}
        st, d = self.save({"voice": {"temperature": 1.0}})
        self.assertEqual(st, 400)
        self.assertIn("相手側の検証エラー", d["voice"]["errors"][0])


class PreviewAndSpeakTests(ApiBase):
    def test_preview_uses_live_menu_state_and_unsaved_settings(self):
        st, d = self.call("POST", "/api/voice_settings/preview", {"settings": {"persona_prompt": "未保存", "temperature": 1.2}})
        self.assertEqual(st, 200, d)
        self.assertEqual(d["text"], "生成文")
        self.assertEqual(d["context_source"], "live")
        self.assertEqual(d["context_menu"], ["カレー", "A定食"])
        sent = self.bridge.posts("/voice_preview")[0]
        self.assertEqual(sent["settings"], {"persona_prompt": "未保存", "temperature": 1.2})
        self.assertEqual(sent["context"]["promote_item"], "カレー")
        self.assertEqual(sent["context"]["remaining_status"][0], {"name": "カレー", "remaining": 8, "initial": 10})
        # 設定は保存していない
        self.assertEqual(self.bridge.posts("/voice_settings"), [])
        self.assertFalse(os.path.exists(self.announce_path))

    def test_preview_sample_when_no_menu(self):
        self.menu_state = {}
        st, d = self.call("POST", "/api/voice_settings/preview", {})
        self.assertEqual(d["context_source"], "sample")
        st, d = self.call("POST", "/api/voice_settings/preview", {"force_sample": True})
        self.assertEqual(d["context_source"], "sample")
        self.menu_state = {"phase": "initial", "items": [{"name": "X", "initial": 1, "remaining": 1}]}
        st, d = self.call("POST", "/api/voice_settings/preview", {})
        self.assertEqual((d["context_source"], d["context_phase"]), ("live", "initial"))
        self.assertNotIn("remaining_status", self.bridge.posts("/voice_preview")[-1]["context"])

    def test_preview_validation(self):
        st, d = self.call("POST", "/api/voice_settings/preview", {"settings": {"temperature": float("nan")}})
        self.assertEqual(st, 400)
        st, d = self.call("POST", "/api/voice_settings/preview", {"settings": {"num_predict": 1}})
        self.assertEqual(st, 400)
        self.assertEqual(self.bridge.posts("/voice_preview"), [])

    def test_preview_bridge_down(self):
        self.server.shutdown()
        self.server.server_close()
        self.start_server(closed_port_url())
        st, d = self.call("POST", "/api/voice_settings/preview", {})
        self.assertEqual(st, 502)
        self.assertIn("2台目に接続できません", d["error"])

    def test_speak_forwards_text_only(self):
        st, d = self.call("POST", "/api/voice_settings/speak", {"text": "  こんにちは  "})
        self.assertEqual((st, d["ok"]), (200, True))
        self.assertEqual(self.bridge.posts("/speak"), [{"text": "こんにちは", "generate": False}])
        for body in ({}, {"text": ""}, {"text": "   "}, {"text": 5}, {"text": "あ" * 401}):
            self.assertEqual(self.call("POST", "/api/voice_settings/speak", body)[0], 400, body)
        self.assertEqual(len(self.bridge.posts("/speak")), 1)

    def test_build_preview_context_rules(self):
        ctx, src = build_preview_context({"phase": "initial", "items": [{"name": "A", "initial": 5, "remaining": 5}]})
        self.assertEqual((src, ctx["phase"], ctx["menu_items"][0]["name"]), ("live", "initial", "A"))
        self.assertNotIn("remaining_status", ctx)
        ctx, src = build_preview_context(None)
        self.assertEqual(src, "sample")
        self.assertIn("remaining_status", ctx)


if __name__ == "__main__":
    unittest.main()
