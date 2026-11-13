"""状況アナウンス設定のAPI/中継/画面(announce_events_proxy.py、dashboard_commonの /api/announce_events*、
announce_settings_page.py)のテスト。docs/announcements.md

ロボット・2台目には触れない(voice_bridgeはhttp.serverのスタブ、rclpy系はスタブ)。
1. AnnounceEventsController: 保存(部分更新・検証・400で何も保存しない)、音量の中継、試し再生、読み戻し(payload)、
   2台目に届かなくても1台目の設定は編集・保存できる
2. ダッシュボードの権限: 未ログイン401、guestは閲覧のみ(POSTは403)、TLは保存・試し再生、検証エラーは400
3. 画面: JS構文(node --check)、入力欄がNKFieldで登録されている、確認バー、緊急停止ドックが残っている、
   ナビのリンク、既存の発話・音声ページのリンクが残っている

実行: python ros2_ws/src/nokosahen_core/test/test_announce_api.py
"""

import json
import re
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

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))


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
    import std_msgs.msg  # noqa: F401
except Exception:  # noqa: BLE001
    _install_stubs()

from nokosahen_core import announce_events as ae  # noqa: E402
from nokosahen_core import announce_events_proxy as ap  # noqa: E402
from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core.announce_settings_page import build_announce_settings_html  # noqa: E402
from nokosahen_core.nav_layout import LINK_GROUPS  # noqa: E402

TMP = tempfile.mkdtemp(prefix="announce_api_test_")


def http(method, url, body=None, headers=None, timeout=10):
    data = None if body is None else json.dumps(body).encode()
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


class _Bridge(BaseHTTPRequestHandler):
    announce_volume = 2.5
    muted = False
    posts = []
    queries = []

    def log_message(self, *a):
        pass

    def _reply(self, code, body):
        data = json.dumps(body, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        _Bridge.queries.append(self.path)
        if self.path.startswith("/announce_state"):
            self._reply(200, {"ok": True, "muted": _Bridge.muted, "loops": {}, "once": {}, "cached": {},
                              "cache": {"items": 2}, "announce_volume": _Bridge.announce_volume})
        else:
            self._reply(404, {"error": "not found"})

    def do_POST(self):
        n = int(self.headers.get("Content-Length", "0"))
        body = json.loads(self.rfile.read(n).decode())
        _Bridge.posts.append((self.path, body))
        if self.path == "/voice_levels":
            _Bridge.announce_volume = body["announce_volume"]
            self._reply(200, {"ok": True, "announce_volume": body["announce_volume"]})
        elif self.path == "/announce":
            self._reply(200, {"ok": True, "muted": True} if _Bridge.muted else {"ok": True})
        else:
            self._reply(404, {"error": "not found"})


def new_controller(url):
    path = tempfile.mkdtemp(dir=TMP) + "/ev.json"
    return ap.AnnounceEventsController(ae.AnnounceEventsStore(path), ap.AnnounceBridgeProxy(url))


class ControllerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge, cls.bport = serve(_Bridge)
        cls.url = f"http://127.0.0.1:{cls.bport}"

    @classmethod
    def tearDownClass(cls):
        cls.bridge.shutdown()
        cls.bridge.server_close()

    def setUp(self):
        _Bridge.posts = []
        _Bridge.queries = []
        _Bridge.muted = False
        _Bridge.announce_volume = 2.5
        self.c = new_controller(self.url)

    def test_payload_has_config_meta_and_bridge(self):
        p = self.c.payload()
        self.assertTrue(p["ok"])
        self.assertFalse(p["config"]["exists"])
        self.assertEqual(len(p["meta"]["events"]), len(ae.EVENT_DEFS))
        self.assertTrue(p["bridge"]["ok"])
        self.assertEqual(p["bridge"]["announce_volume"], 2.5)
        self.assertIn("text=", _Bridge.queries[0])  # キャッシュ済みか調べる文言を渡している

    def test_save_events_and_readback(self):
        code, r = self.c.save({"events": {"exploring": {"text": "探索中", "interval_sec": 5, "volume": 2}}})
        self.assertEqual((code, r["ok"]), (200, True))
        p = self.c.payload()
        e = p["config"]["events"]["exploring"]
        self.assertEqual((e["text"], e["interval_sec"], e["volume"], p["config"]["enabled"]), ("探索中", 5, 2, True))
        self.assertEqual(_Bridge.posts, [])  # 音量を変えていなければ2台目へは送らない

    def test_save_volume_goes_to_bridge(self):
        code, r = self.c.save({"announce_volume": 3.5})
        self.assertEqual((code, r["ok"]), (200, True))
        self.assertEqual(_Bridge.posts, [("/voice_levels", {"announce_volume": 3.5})])
        self.assertEqual(self.c.payload()["bridge"]["announce_volume"], 3.5)
        self.assertFalse(self.c.payload()["config"]["exists"])  # 音量だけならファイルは作らない

    def test_validation_400_saves_nothing(self):
        bad = [
            {"announce_volume": 5.1}, {"announce_volume": "3"}, {"announce_volume": True}, {"announce_volume": float("nan")},
            {"events": {"exploring": {"interval_sec": 1}}}, {"events": {"nope": {}}}, {"events": {"exploring": {"x": 1}}},
            {"enabled": "yes"}, {}, {"unknown": 1}, [1], "x",
        ]
        for b in bad:
            code, r = self.c.save(b)
            self.assertEqual(code, 400, b)
            self.assertFalse(r["ok"])
        self.assertFalse(self.c.payload()["config"]["exists"])
        self.assertEqual(_Bridge.posts, [])

    def test_bridge_down_still_edits_local(self):
        c = new_controller("http://127.0.0.1:1")
        c.bridge.timeout_s = 0.3
        p = c.payload()
        self.assertFalse(p["bridge"]["ok"])
        self.assertIn("接続できません", p["bridge"]["error"])
        code, r = c.save({"events": {"exploring": {"text": "あ"}}})
        self.assertEqual((code, r["ok"]), (200, True))
        code, r = c.save({"announce_volume": 2})
        self.assertFalse(r["ok"])
        code, r = c.test({"id": "exploring"})
        self.assertFalse(r["ok"])

    def test_unconfigured(self):
        c = new_controller("")
        self.assertFalse(c.payload()["configured"])
        self.assertFalse(c.test({"id": "exploring"})[1]["ok"])

    def test_test_play(self):
        code, r = self.c.test({"id": "exploring"})
        self.assertEqual((code, r["ok"], r["played"]), (200, True, True))
        self.assertEqual(_Bridge.posts[-1], ("/announce", {"id": "exploring", "text": "探索中です。ご注意ください", "volume": 1.0}))
        code, r = self.c.test({"id": "exploring", "text": "試験", "volume": 3})
        self.assertEqual(_Bridge.posts[-1], ("/announce", {"id": "exploring", "text": "試験", "volume": 3.0}))
        _Bridge.muted = True
        code, r = self.c.test({"id": "exploring"})
        self.assertEqual((r["ok"], r["played"], r["muted"]), (True, False, True))

    def test_test_play_validation(self):
        for b in ({"id": "nope"}, {"id": 3}, {"id": "exploring", "text": ""}, {"id": "exploring", "volume": 9},
                  {"id": "exploring", "x": 1}, [1]):
            n = len(_Bridge.posts)
            self.assertEqual(self.c.test(b)[0], 400, b)
            self.assertEqual(len(_Bridge.posts), n)


class FakeTeleop:
    soft_estop_engaged = False


class DashboardApiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge, cls.bport = serve(_Bridge)
        cls.url = f"http://127.0.0.1:{cls.bport}"

    @classmethod
    def tearDownClass(cls):
        cls.bridge.shutdown()
        cls.bridge.server_close()

    def setUp(self):
        _Bridge.posts = []
        _Bridge.muted = False
        _Bridge.announce_volume = 2.5
        self.ctrl = new_controller(self.url)
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, FakeTeleop(), announce_events=self.ctrl,
            announce_settings_html_getter=build_announce_settings_html,
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

    def page(self, cookie):
        req = urllib.request.Request(f"http://127.0.0.1:{self.port}/announce_settings",
                                     headers={"Cookie": cookie} if cookie else {})
        class NoRedirect(urllib.request.HTTPRedirectHandler):
            def redirect_request(self, *a, **k):
                return None
        try:
            with urllib.request.build_opener(NoRedirect).open(req, timeout=10) as r:
                return r.status, r.read().decode("utf-8")
        except urllib.error.HTTPError as e:
            e.close()
            return e.code, ""

    def test_requires_login(self):
        self.assertEqual(self.call("GET", "/api/announce_events", cookie="")[0], 401)
        self.assertEqual(self.call("POST", "/api/announce_events", {"announce_volume": 2}, cookie="")[0], 401)
        self.assertEqual(self.call("POST", "/api/announce_events/test", {"id": "exploring"}, cookie="")[0], 401)
        self.assertIn(self.page("")[0], (302, 303, 401))
        self.assertEqual(_Bridge.posts, [])

    def test_guest_views_but_cannot_write(self):
        for ck in (self.tl, self.guest):
            code, d = self.call("GET", "/api/announce_events", cookie=ck)
            self.assertEqual((code, d["ok"]), (200, True))
            code, html = self.page(ck)
            self.assertEqual(code, 200)
            self.assertIn("状況アナウンス", html)
        self.assertEqual(self.call("POST", "/api/announce_events", {"announce_volume": 2}, cookie=self.guest)[0], 403)
        self.assertEqual(self.call("POST", "/api/announce_events", {"events": {"exploring": {"text": "x"}}}, cookie=self.guest)[0], 403)
        self.assertEqual(self.call("POST", "/api/announce_events/test", {"id": "exploring"}, cookie=self.guest)[0], 403)
        self.assertEqual(_Bridge.posts, [])
        self.assertFalse(self.ctrl.store.read()["exists"])

    def test_tl_save_validation_and_readback(self):
        code, d = self.call("POST", "/api/announce_events", {"events": {"exploring": {"text": "探索中", "volume": 2.5}}, "announce_volume": 3})
        self.assertEqual((code, d["ok"]), (200, True))
        code, d = self.call("GET", "/api/announce_events")
        self.assertEqual(d["config"]["events"]["exploring"]["text"], "探索中")
        self.assertEqual(d["bridge"]["announce_volume"], 3)
        for bad in ({"events": {"exploring": {"volume": 6}}}, {"announce_volume": -1}, {"events": {"x": {}}}, {}):
            self.assertEqual(self.call("POST", "/api/announce_events", bad)[0], 400, bad)
        self.assertEqual(self.call("GET", "/api/announce_events")[1]["config"]["events"]["exploring"]["volume"], 2.5)

    def test_tl_test_play(self):
        code, d = self.call("POST", "/api/announce_events/test", {"id": "backing_up"})
        self.assertEqual((code, d["ok"]), (200, True))
        self.assertEqual(_Bridge.posts[-1][0], "/announce")
        self.assertEqual(self.call("POST", "/api/announce_events/test", {"id": "zzz"})[0], 400)

    def test_roles_classify_the_new_write_apis(self):
        # 認可の分類は roles.API_RULES(別担当が一元管理)。ここでは「表に載っている=デフォルト拒否で漏れない」ことだけ確認する
        from nokosahen_core import roles
        self.assertEqual(roles.required_capabilities("POST", "/api/announce_events"), ["announce.events"])
        self.assertEqual(roles.required_capabilities("POST", "/api/announce_events/test"), ["announce.test"])


class PageTest(unittest.TestCase):
    def setUp(self):
        self.html = build_announce_settings_html().decode("utf-8")

    def test_js_syntax(self):
        node = shutil.which("node")
        if not node:
            self.skipTest("nodeがありません")
        scripts = re.findall(r"<script>(.*?)</script>", self.html, re.S)
        self.assertEqual(len(scripts), 1)
        f = Path(tempfile.mkdtemp(dir=TMP)) / "page.js"
        f.write_text(scripts[0], encoding="utf-8")
        r = subprocess.run([node, "--check", str(f)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)

    def test_structure(self):
        h = self.html
        self.assertNotIn("__NK_FIELD_JS__", h)
        self.assertIn("window.NKField", h)
        for el in ("aeMaster", "aeVol", "aeVolNum", "saveBtn", "discardBtn", "confirmBar", "confirmYes", "confirmNo",
                   "estopDock", "estopBtn", "bridgeBadge", "events"):
            self.assertEqual(h.count(f'id="{el}"'), 1, el)
        self.assertIn("const CONFIRM_ABOVE = 2;", h)
        self.assertIn("NKField.begin(", h)
        self.assertIn("tx.verify(", h)
        self.assertIn("/api/announce_events/test", h)
        self.assertNotIn("setInterval(", h.split("window.NKField = ")[-1])  # 定期取得はNKField.pollだけ
        self.assertIn('max="5"', h)

    def test_all_dynamic_fields_are_guarded(self):
        h = self.html
        # 動的に作る欄(ev_<id>_<sfx>)は全てFIELDS表に載り、buildCardsで guard に渡される
        for sfx in ("enabled", "text", "mode", "interval", "volume", "cooldown", "minact", "quiet", "survey", "estop"):
            self.assertIn(f'sfx: "{sfx}"', h)
        self.assertIn("NKField.guard(guardIds)", h)
        self.assertIn('NKField.link(fid(ev.id, "volume"), fid(ev.id, "volumeNum"))', h)
        for tag in re.findall(r"<(?:input|select)\b[^>]*>", h.split("<script>")[0]):
            if 'type="checkbox"' in tag and "aeMaster" not in tag:
                self.fail("静的な欄はaeMasterとスライダー類だけのはず: " + tag)

    def test_links_and_existing_pages_remain(self):
        links = [item for _g, _t, items in LINK_GROUPS for item in items]
        self.assertIn("/announce_settings", [i[0] for i in links])
        self.assertIn("/voice_settings", [i[0] for i in links])
        self.assertIn("/utterance_log", [i[0] for i in links])


if __name__ == "__main__":
    unittest.main()
