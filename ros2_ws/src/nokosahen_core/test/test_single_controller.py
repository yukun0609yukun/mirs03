"""同時1端末制限(SingleControllerGate / enforce_single_controller)のテスト(2026-10-13、本番値trueに復帰)。

- 設定: nokosahen.yamlのfull_dashboardが本番値true、Stage1/2は常に無制限、ノードの既定true、
  開発用の上書き手順(環境変数NOKOSAHEN_ENFORCE_SINGLE_CONTROLLER)がコードとdocsに存在すること
- SingleControllerGate: 取得 / 他IPの拒否 / 同IPの再取得 / 時間切れでの引き継ぎ / 明示解放 / アカウント別 / 無効時は素通り
- _resolve_control_account: guestは対象外、未ログインは401相当、別端末は"occupied"
- 環境変数の上書き(resolve_enforce_single_controller)
- HTTP(ROSはスタブ): 2台目のTLは画面で案内ページ(409 HTML)・APIで409 in_use、guestは常に閲覧可、ログアウトで解放、
  時間切れで引き継ぎ、PC版(8090相当)とスマホ版(8095相当)は別枠、無効なら2台とも通る
  (別端末は送信元127.0.0.2で模擬。使えない環境ではHTTPの項目だけskip)
実行: python ros2_ws/src/nokosahen_core/test/test_single_controller.py
"""

import http.client
import json
import re
import sys
import threading
import time
import types
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest import mock

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
REPO = PKG.parent.parent.parent
YAML = REPO / "ros2_ws" / "src" / "nokosahen_bringup" / "config" / "nokosahen.yaml"


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
    mod("std_msgs.msg", Bool=Msg, Float32MultiArray=Msg, String=Msg)


try:
    import geometry_msgs.msg  # noqa: F401
    import nav_msgs.msg  # noqa: F401
except Exception:  # noqa: BLE001
    _install_stubs()

from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core.http_auth import SessionStore  # noqa: E402


# ------------------------------------------------------------------ 設定・コード
def _yaml_section(name):
    text = YAML.read_text(encoding="utf-8")
    m = re.search(rf"^{name}:\n(.*?)(?=^\S)", text + "\nEND:", re.M | re.S)
    assert m, name
    return m.group(1)


class ConfigTests(unittest.TestCase):
    def test_full_dashboard_is_production_true(self):
        sec = _yaml_section("full_dashboard")
        vals = re.findall(r"^\s*enforce_single_controller:\s*(\S+)", sec, re.M)
        self.assertEqual(vals, ["true"])

    def test_yaml_only_full_dashboard_sets_it(self):
        # 他のセクション(stage1/stage2等)に同名キーを置かない(Stage1/2は常に無制限)。
        text = YAML.read_text(encoding="utf-8")
        self.assertEqual(len(re.findall(r"^\s*enforce_single_controller:", text, re.M)), 1)

    def test_stage_dashboards_always_unrestricted(self):
        for name in ("stage1_dashboard_node.py", "stage2_dashboard_node.py"):
            src = (PKG / "nokosahen_core" / name).read_text(encoding="utf-8")
            self.assertIn("enforce_single_controller=False", src, name)
            self.assertNotIn("enforce_single_controller=True", src, name)

    def test_node_default_is_true_and_env_override_wired(self):
        src = (PKG / "nokosahen_core" / "full_dashboard_node.py").read_text(encoding="utf-8")
        self.assertIn('declare_parameter("enforce_single_controller", True)', src)
        self.assertEqual(src.count("resolve_enforce_single_controller("), 2)  # PC版とスマホ版の両方
        self.assertNotIn('enforce_single_controller=bool(self.get_parameter', src)

    def test_make_handlers_default_true(self):
        import inspect
        for fn in (dc.make_dashboard_handler, dc.make_mobile_handler):
            self.assertIs(inspect.signature(fn).parameters["enforce_single_controller"].default, True)
        self.assertTrue(dc.SingleControllerGate().try_acquire("1.1.1.1", "TL"))
        self.assertIs(inspect.signature(dc.SingleControllerGate).parameters["enforce"].default, True)

    def test_docs_have_single_place_for_override(self):
        doc = (REPO / "docs" / "open_items.md").read_text(encoding="utf-8")
        self.assertIn(dc.ENFORCE_SINGLE_CONTROLLER_ENV, doc)
        self.assertIn("enforce_single_controller", doc)
        self.assertIn("NOKOSAHEN_ENFORCE_SINGLE_CONTROLLER=0", doc)


# ------------------------------------------------------------------ 環境変数の上書き
class EnvOverrideTests(unittest.TestCase):
    R = staticmethod(dc.resolve_enforce_single_controller)
    K = dc.ENFORCE_SINGLE_CONTROLLER_ENV

    def test_unset_uses_param(self):
        self.assertTrue(self.R(True, {}))
        self.assertFalse(self.R(False, {}))
        self.assertTrue(self.R(True, {self.K: ""}))
        self.assertTrue(self.R(True, {self.K: "  "}))

    def test_falsy_values_disable(self):
        for v in ("0", "false", "FALSE", "off", "no", " 0 "):
            self.assertFalse(self.R(True, {self.K: v}), v)

    def test_truthy_values_enable(self):
        for v in ("1", "true", "On", "yes"):
            self.assertTrue(self.R(False, {self.K: v}), v)

    def test_garbage_ignored_with_warning(self):
        log = mock.Mock()
        self.assertTrue(self.R(True, {self.K: "maybe"}, log))
        log.warn.assert_called_once()

    def test_override_warns(self):
        log = mock.Mock()
        self.assertFalse(self.R(True, {self.K: "0"}, log))
        log.warn.assert_called_once()
        log2 = mock.Mock()
        self.assertTrue(self.R(True, {self.K: "1"}, log2))  # 同じ値なら黙る
        log2.warn.assert_not_called()

    def test_reads_os_environ_by_default(self):
        with mock.patch.dict("os.environ", {self.K: "0"}):
            self.assertFalse(dc.resolve_enforce_single_controller(True))
        with mock.patch.dict("os.environ", {}, clear=False):
            import os
            os.environ.pop(self.K, None)
            self.assertTrue(dc.resolve_enforce_single_controller(True))


# ------------------------------------------------------------------ ゲート本体
class GateTests(unittest.TestCase):
    def setUp(self):
        self.now = [100.0]
        p = mock.patch.object(dc.time, "monotonic", lambda: self.now[0])
        p.start()
        self.addCleanup(p.stop)
        self.gate = dc.SingleControllerGate(timeout_sec=8.0, enforce=True)

    def test_first_acquires_second_denied(self):
        self.assertTrue(self.gate.try_acquire("10.0.0.1", "TL"))
        self.assertFalse(self.gate.try_acquire("10.0.0.2", "TL"))

    def test_same_ip_reacquires_and_keeps_alive(self):
        self.assertTrue(self.gate.try_acquire("10.0.0.1", "TL"))
        for _ in range(5):
            self.now[0] += 6.0  # 8秒未満の間隔で来続ける限り保持
            self.assertTrue(self.gate.try_acquire("10.0.0.1", "TL"))
            self.assertFalse(self.gate.try_acquire("10.0.0.2", "TL"))

    def test_takeover_after_timeout(self):
        self.gate.try_acquire("10.0.0.1", "TL")
        self.now[0] += 8.0   # ちょうど8秒は保持(超えたら解放)
        self.assertFalse(self.gate.try_acquire("10.0.0.2", "TL"))
        self.now[0] += 0.01
        self.assertTrue(self.gate.try_acquire("10.0.0.2", "TL"))
        # 奪われた元の端末は、今度は拒否される(先着が保持)
        self.assertFalse(self.gate.try_acquire("10.0.0.1", "TL"))

    def test_denied_attempt_does_not_extend_holder(self):
        self.gate.try_acquire("10.0.0.1", "TL")
        for _ in range(3):
            self.now[0] += 3.0
            self.gate.try_acquire("10.0.0.2", "TL")   # 拒否されるが、保持側の時刻は更新しない
        self.assertTrue(self.gate.try_acquire("10.0.0.2", "TL"))  # 合計9秒>8秒で引き継げる

    def test_release_frees_immediately(self):
        self.gate.try_acquire("10.0.0.1", "TL")
        self.gate.release("TL")
        self.assertTrue(self.gate.try_acquire("10.0.0.2", "TL"))

    def test_release_of_unknown_account_is_noop(self):
        self.gate.release("nobody")

    def test_accounts_are_independent(self):
        self.assertTrue(self.gate.try_acquire("10.0.0.1", "TL"))
        self.assertTrue(self.gate.try_acquire("10.0.0.2", "restricted_account"))
        self.assertFalse(self.gate.try_acquire("10.0.0.3", "TL"))
        self.assertFalse(self.gate.try_acquire("10.0.0.4", "restricted_account"))

    def test_disabled_always_passes(self):
        g = dc.SingleControllerGate(enforce=False)
        self.assertTrue(g.try_acquire("10.0.0.1", "TL"))
        self.assertTrue(g.try_acquire("10.0.0.2", "TL"))

    def test_default_timeout_is_8_seconds(self):
        self.assertEqual(dc.SingleControllerGate()._timeout_sec, 8.0)


class _FakeHandler:
    def __init__(self, ip, token=None):
        self.client_address = (ip, 1234)
        self.headers = {"Cookie": f"nokosahen_session={token}"} if token else {}


class ResolveAccountTests(unittest.TestCase):
    def setUp(self):
        self.sessions = SessionStore()
        self.gate = dc.SingleControllerGate(enforce=True)
        self.tl_a = self.sessions.create("TL")
        self.tl_b = self.sessions.create("TL")
        self.guest = self.sessions.create("guest")

    def res(self, ip, token):
        with mock.patch.object(dc, "get_session_token", lambda h: token):
            return dc._resolve_control_account(_FakeHandler(ip), "TL", self.gate, self.sessions)

    def test_no_session(self):
        self.assertEqual(self.res("10.0.0.1", None), (None, "auth_required"))

    def test_second_device_occupied(self):
        self.assertEqual(self.res("10.0.0.1", self.tl_a), ("TL", None))
        self.assertEqual(self.res("10.0.0.2", self.tl_b), (None, "occupied"))

    def test_guest_never_gated_and_never_holds(self):
        self.assertEqual(self.res("10.0.0.2", self.guest), ("guest", None))
        self.assertEqual(self.res("10.0.0.3", self.guest), ("guest", None))
        self.assertEqual(self.res("10.0.0.1", self.tl_a), ("TL", None))  # guestがいてもTLは取れる

    def test_auth_disabled(self):
        with mock.patch.object(dc, "get_session_token", lambda h: None):
            self.assertEqual(dc._resolve_control_account(_FakeHandler("1.1.1.1"), "", self.gate, self.sessions), ("", None))


# ------------------------------------------------------------------ HTTP
class FakeTeleop:
    soft_estop_engaged = False


def _can_use_second_loopback():
    try:
        s = http.client.HTTPConnection  # noqa: F841
        import socket
        a = socket.socket()
        a.bind(("127.0.0.1", 0))
        a.listen(1)
        b = socket.socket()
        b.bind(("127.0.0.2", 0))
        b.settimeout(2)
        b.connect(a.getsockname())
        peer, addr = a.accept()
        ok = addr[0] == "127.0.0.2"
        for x in (peer, a, b):
            x.close()
        return ok
    except Exception:  # noqa: BLE001
        return False


HAVE_LOOPBACK2 = _can_use_second_loopback()
REAL_GATE = dc.SingleControllerGate


class Client:
    """送信元IPを選べる簡易クライアント(別端末の模擬)。"""

    def __init__(self, port, src):
        self.port, self.src, self.cookie = port, src, ""

    def req(self, method, path, body=None):
        c = http.client.HTTPConnection("127.0.0.1", self.port, timeout=10, source_address=(self.src, 0))
        try:
            headers = {"Content-Type": "application/json"}
            if self.cookie:
                headers["Cookie"] = self.cookie
            c.request(method, path, body=None if body is None else json.dumps(body).encode(), headers=headers)
            r = c.getresponse()
            data = r.read()
            if r.getheader("Set-Cookie"):
                self.cookie = r.getheader("Set-Cookie").split(";")[0]
            return r.status, r.getheader("Content-Type", ""), data
        finally:
            c.close()

    def login(self, user, pw):
        st, _, data = self.req("POST", "/api/login", {"username": user, "password": pw})
        assert st == 200, data
        return self

    def api(self, path="/api/state", method="GET", body=None):
        st, _, data = self.req(method, path, body)
        try:
            return st, json.loads(data.decode("utf-8"))
        except ValueError:
            return st, data


@unittest.skipUnless(HAVE_LOOPBACK2, "127.0.0.2から接続できない環境のためHTTPでの別端末模擬をskip")
class HttpTests(unittest.TestCase):
    TIMEOUT = 0.4

    def start(self, kind, enforce=True):
        orig = REAL_GATE
        t = self.TIMEOUT
        p = mock.patch.object(dc, "SingleControllerGate", lambda enforce: orig(timeout_sec=t, enforce=enforce))
        p.start()
        self.addCleanup(p.stop)
        if kind == "pc":
            handler = dc.make_dashboard_handler(
                lambda: b"<html>pc</html>", lambda: {"ok": 1}, FakeTeleop(),
                auth_username="TL", auth_password="pw", enforce_single_controller=enforce)
        else:
            handler = dc.make_mobile_handler(
                lambda: b"<html>mobile</html>", lambda: {"ok": 1}, FakeTeleop(),
                auth_username="TL", auth_password="pw", restricted_username="restricted_account",
                restricted_password="pw2", enforce_single_controller=enforce)
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        return server.server_address[1]

    def two(self, port):
        a = Client(port, "127.0.0.1").login("TL", "pw")
        b = Client(port, "127.0.0.2").login("TL", "pw")
        return a, b

    def test_second_device_blocked_page_and_api(self):
        port = self.start("pc")
        a, b = self.two(port)
        self.assertEqual(a.api()[0], 200)      # 先着の端末が操作権
        st, ctype, body = b.req("GET", "/")    # 画面を開くと案内ページ(生JSONではない)
        self.assertEqual(st, 409)
        self.assertIn("text/html", ctype)
        self.assertIn("他の端末が操作中です".encode("utf-8"), body)
        self.assertIn(b'http-equiv="refresh"', body)
        st, d = b.api()                        # APIは従来どおりJSON(error=in_use)
        self.assertEqual((st, d["error"]), (409, "in_use"))
        st, d = b.api("/api/teleop", "POST", {"forward": 1, "turn": 0})
        self.assertEqual(st, 409)
        self.assertEqual(a.api()[0], 200)      # 先着は影響を受けない

    def test_owner_page_opens(self):
        port = self.start("pc")
        a, _ = self.two(port)
        st, ctype, body = a.req("GET", "/")
        self.assertEqual((st, body), (200, b"<html>pc</html>"))

    def test_guest_always_viewable_and_does_not_block(self):
        port = self.start("pc")
        g1 = Client(port, "127.0.0.1").login("guest", "")
        g2 = Client(port, "127.0.0.2").login("guest", "")
        self.assertEqual(g1.api()[0], 200)
        self.assertEqual(g2.api()[0], 200)
        a = Client(port, "127.0.0.1").login("TL", "pw")
        self.assertEqual(a.api()[0], 200)       # guestがいてもTLは操作権を取れる
        st, d = g2.api("/api/soft_estop", "POST", {"engage": True})
        self.assertEqual(st, 403)               # guestは閲覧専用

    def test_logout_releases(self):
        port = self.start("pc")
        a, b = self.two(port)
        self.assertEqual(a.api()[0], 200)
        self.assertEqual(b.api()[0], 409)
        self.assertEqual(a.api("/api/logout", "POST", {})[0], 200)
        self.assertEqual(b.api()[0], 200)       # すぐ引き継げる

    def test_timeout_takeover(self):
        port = self.start("pc")
        a, b = self.two(port)
        self.assertEqual(a.api()[0], 200)
        time.sleep(self.TIMEOUT + 0.2)          # 先着が操作をやめて時間切れ
        self.assertEqual(b.api()[0], 200)
        self.assertEqual(a.api()[0], 409)       # 元の端末は今度は拒否される

    def test_mobile_and_pc_are_separate_slots(self):
        pc = self.start("pc")
        mobile = self.start("mobile")
        a = Client(pc, "127.0.0.1").login("TL", "pw")
        m = Client(mobile, "127.0.0.2").login("TL", "pw")
        self.assertEqual(a.api()[0], 200)
        self.assertEqual(m.api()[0], 200)       # PC(別端末)とスマホで1人ずつ

    def test_mobile_second_device_blocked(self):
        mobile = self.start("mobile")
        a, b = self.two(mobile)
        self.assertEqual(a.api()[0], 200)
        st, ctype, body = b.req("GET", "/")
        self.assertEqual(st, 409)
        self.assertIn("他の端末が操作中です".encode("utf-8"), body)
        self.assertEqual(b.api()[0], 409)

    def test_mobile_accounts_independent(self):
        mobile = self.start("mobile")
        tl = Client(mobile, "127.0.0.1").login("TL", "pw")
        sk = Client(mobile, "127.0.0.2").login("restricted_account", "pw2")
        self.assertEqual(tl.api()[0], 200)
        self.assertEqual(sk.api()[0], 200)      # TLとrestricted_accountは別枠

    def test_disabled_allows_both(self):
        port = self.start("pc", enforce=False)
        a, b = self.two(port)
        self.assertEqual(a.api()[0], 200)
        self.assertEqual(b.api()[0], 200)

    def test_same_device_two_tabs_share_slot(self):
        # 同じ端末(同じIP)の別タブ/別ブラウザは同じ枠(別セッションでも通る)。
        port = self.start("pc")
        a1 = Client(port, "127.0.0.1").login("TL", "pw")
        a2 = Client(port, "127.0.0.1").login("TL", "pw")
        self.assertEqual(a1.api()[0], 200)
        self.assertEqual(a2.api()[0], 200)


class BannerJsTests(unittest.TestCase):
    def test_in_use_banner_is_in_shared_js(self):
        from nokosahen_core.nav_layout import NK_FIELD_JS, NAV_JS
        self.assertIn("in_use", NK_FIELD_JS)
        self.assertIn("__nkInUseWatch", NAV_JS)
        self.assertEqual(NK_FIELD_JS.count("__nkInUseWatch = true"), 1)

    def test_occupied_html_wired(self):
        src = (PKG / "nokosahen_core" / "dashboard_common.py").read_text(encoding="utf-8")
        self.assertEqual(src.count("_send_occupied_response(self, redirect)"), 2)  # PC版・スマホ版
        self.assertIn(b"http-equiv", dc._OCCUPIED_HTML)


if __name__ == "__main__":
    unittest.main(verbosity=1)
