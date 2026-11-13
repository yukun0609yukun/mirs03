"""管理画面(PC版8090・スマホ版8095)の全POSTの Origin / Sec-Fetch-Site 検証(2026-10-30追加。docs/ipad_display.md・docs/accounts.md)。

別サイトのブラウザからのPOST(Origin が別ホスト・Sec-Fetch-Site が cross-site/same-site・Origin: null)は、ログイン済みでも403(error=cross_origin)で、
何も実行しない。同じオリジン(Host と同じ Origin、Sec-Fetch-Site: same-origin/none)と、どちらのヘッダも無い要求(curl・内部呼び出し・既存テスト)は従来どおり。

実行: python ros2_ws/src/nokosahen_core/test/test_admin_origin_guard.py
"""
import json
import sys
import types
import unittest
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_account_roles_http as T  # noqa: E402  (ROS無しの代替・偽サーバー・serve などを借りる)

from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import display_routes as dr  # noqa: E402
from nokosahen_core import roles  # noqa: E402
from nokosahen_core.bgm_proxy import BgmProxy  # noqa: E402


def post(url, body=None, cookie=None, headers=None):
    h = {"Content-Type": "application/json"}
    if cookie:
        h["Cookie"] = cookie
    h.update(headers or {})
    req = urllib.request.Request(url, data=json.dumps(body if body is not None else {}).encode(), method="POST", headers=h)
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            return r.status, json.loads(r.read().decode("utf-8") or "{}"), r.headers.get("Set-Cookie", "")
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read().decode("utf-8") or "{}"), ""
        finally:
            e.close()


CROSS = ({"Origin": "http://evil.example"}, {"Origin": "http://evil.example", "Sec-Fetch-Site": "cross-site"},
         {"Sec-Fetch-Site": "cross-site"}, {"Sec-Fetch-Site": "same-site"}, {"Origin": "null"},
         {"Origin": "http://127.0.0.1:1"}, {"Origin": "https://evil.example", "Sec-Fetch-Site": "same-origin"})   # 最後: Origin が別ならSec-Fetch-Siteが何でも断る


def posts_in_rules():
    return sorted({p for (m, p) in roles.API_RULES if m == "POST"})


class OriginOkUnitTest(unittest.TestCase):
    def ok(self, **kw):
        return dr.origin_ok(types.SimpleNamespace(get=lambda k, d=None: kw.get(k.replace("-", "_"), d)))

    def test_table(self):
        yes = [dict(Host="192.0.2.1:8090"),                                                  # ヘッダ無し(curl等)
               dict(Host="192.0.2.1:8090", Origin="http://192.0.2.1:8090"),
               dict(Host="Jetson.LOCAL:8090", Origin="http://jetson.local:8090"),             # 大文字小文字
               dict(Host="jetson.local", Origin="http://jetson.local"),                      # 既定ポート省略(80)
               dict(Host="jetson.local:80", Origin="http://jetson.local"),
               dict(Host="jetson.local", Origin="http://jetson.local:80"),
               dict(Host="jetson.local:443", Origin="https://jetson.local"),
               dict(Host="[::1]:8090", Origin="http://[::1]:8090"),
               dict(Host="192.0.2.1:8090", Sec_Fetch_Site="same-origin"),
               dict(Host="192.0.2.1:8090", Sec_Fetch_Site="none"),                            # アドレスバーから直接
               dict(Host="192.0.2.1:8090", Origin="http://192.0.2.1:8090", Sec_Fetch_Site="same-origin")]
        no = [dict(Host="192.0.2.1:8090", Origin="http://192.0.2.1:8095"),                   # ポート違い(8090と8095は別オリジン)
              dict(Host="192.0.2.1:8090", Origin="http://192.0.2.2:8090"),
              dict(Host="192.0.2.1:8090", Origin="http://evil.example"),
              dict(Host="192.0.2.1:8090", Origin="null"),
              dict(Host="192.0.2.1:8090", Origin="ftp://192.0.2.1:8090"),
              dict(Host="192.0.2.1:8090", Origin="http://192.0.2.1:8090.evil.example"),
              dict(Host="192.0.2.1:8090", Origin="http://192.0.2.1:8090x"),
              dict(Host="192.0.2.1:8090", Origin="http://"),
              dict(Origin="http://192.0.2.1:8090"),                                           # Host が無い
              dict(Host="jetson.local:8090", Origin="http://jetson.local"),                   # 80 と 8090 は別
              dict(Host="192.0.2.1:8090", Sec_Fetch_Site="cross-site"),
              dict(Host="192.0.2.1:8090", Sec_Fetch_Site="same-site"),
              dict(Host="192.0.2.1:8090", Sec_Fetch_Site="bogus"),
              dict(Host="192.0.2.1:8090", Origin="http://192.0.2.1:8090", Sec_Fetch_Site="cross-site")]
        for h in yes:
            self.assertTrue(self.ok(**h), h)
        for h in no:
            self.assertFalse(self.ok(**h), h)


class DesktopOriginGuardTest(T.Base):
    def setUp(self):
        self.start()
        self.tl = self.cookie("TL", "tl-pass")

    def test_every_post_api_is_refused_from_another_site(self):
        paths = posts_in_rules() + ["/api/login", "/api/logout", "/api/unknown_new_api", "/display/api/login"]
        self.assertGreater(len(paths), 30)
        for hdr in CROSS:
            for p in paths:
                code, d, _ = post(self.base + p, {"mode": "survey", "forward": 0.5, "turn": 0.0}, self.tl, hdr)
                self.assertEqual(code, 403, (p, hdr))
                if not p.startswith("/display/"):
                    self.assertEqual(d.get("error"), "cross_origin", (p, hdr))
        self.assertEqual(self.teleop.drives, [])             # ロボットには何も送られていない
        self.assertEqual(T.Pi.posts, [])
        self.assertEqual(T.Bridge.posts, [])
        # ログイン済みのセッションは、拒否で壊れていない
        self.assertEqual(T.http("GET", self.base + "/api/whoami", cookie=self.tl)[1]["role"], "admin")

    def test_login_from_another_site_is_refused_and_sets_no_cookie(self):
        for hdr in CROSS:
            code, d, ck = post(self.base + "/api/login", {"username": "TL", "password": "tl-pass"}, None, hdr)
            self.assertEqual((code, ck), (403, ""), hdr)
        self.assertEqual(post(self.base + "/api/login", {"username": "TL", "password": "tl-pass"}, None, {"Origin": self.base})[0], 200)

    def test_same_origin_and_headerless_posts_pass(self):
        goods = ({}, {"Origin": self.base}, {"Origin": self.base.replace("http", "HTTP", 1)},   # 大文字の方式名でも
                 {"Sec-Fetch-Site": "same-origin"}, {"Sec-Fetch-Site": "none"},
                 {"Origin": self.base, "Sec-Fetch-Site": "same-origin"})
        for hdr in goods:
            code, d, _ = post(self.base + "/api/teleop", {"forward": 0.0, "turn": 0.0}, self.tl, hdr)
            self.assertEqual(code, 200, hdr)
            code, d, ck = post(self.base + "/api/login", {"username": "TL", "password": "tl-pass"}, None, hdr)
            self.assertEqual(code, 200, hdr)
            code, d, _ = post(self.base + "/api/logout", {}, ck.split(";")[0], hdr)
            self.assertEqual(code, 200, hdr)
        self.assertEqual(len(self.teleop.drives), len(goods))

    def test_operator_flow_keeps_working_with_same_origin_headers(self):
        hdr = {"Origin": self.base, "Sec-Fetch-Site": "same-origin"}
        code, d, ck = post(self.base + "/api/login", {"username": "kari-op", "password": T.OP_PASSWORD}, None, hdr)
        self.assertEqual(code, 200)
        op = ck.split(";")[0]
        self.assertEqual(post(self.base + "/api/soft_estop", {"engaged": True}, op, hdr)[0], 200)
        code, d, _ = post(self.base + "/api/speed_settings", {"manual_linear_mps": 0.3}, op, hdr)   # 【2026-11-06】/api/teleopは弱いアカウントも可になったので、管理者専用の速度設定で確認
        self.assertEqual(code, 403)                                  # 権限の403は従来どおり(cross_originではない)
        self.assertNotEqual(d.get("error"), "cross_origin")


class MobileOriginGuardTest(T.Base):
    def test_mobile_handler(self):
        teleop = T.FakeTeleop()
        store = self.make_store(perms=["voice.levels"])
        handler = dc.make_mobile_handler(
            lambda: b"<html></html>", lambda: {}, teleop, auth_username="TL", auth_password="tl-pass",
            enforce_single_controller=False, bgm=BgmProxy(self.bridge_url), account_store=store)
        srv, port = T.serve(handler)
        self.addCleanup(lambda: (srv.shutdown(), srv.server_close()))
        base = f"http://127.0.0.1:{port}"
        for hdr in CROSS:
            code, d, ck = post(base + "/api/login", {"username": "TL", "password": "tl-pass"}, None, hdr)
            self.assertEqual((code, ck), (403, ""), hdr)
        ok = {"Origin": base, "Sec-Fetch-Site": "same-origin"}
        code, d, ck = post(base + "/api/login", {"username": "TL", "password": "tl-pass"}, None, ok)
        self.assertEqual(code, 200)
        tl = ck.split(";")[0]
        for hdr in CROSS:
            for p in posts_in_rules() + ["/api/unknown_new_api"]:
                code, d, _ = post(base + p, {"forward": 0.5, "turn": 0.0}, tl, hdr)
                self.assertEqual((code, d.get("error")), (403, "cross_origin"), (p, hdr))
        self.assertEqual(teleop.drives, [])
        # PC版(8090)のOriginを付けたスマホ版へのPOSTは別オリジンなので断る
        self.assertEqual(post(base + "/api/teleop", {"forward": 0.0, "turn": 0.0}, tl, {"Origin": "http://127.0.0.1:8090"})[0], 403)
        for hdr in ({}, ok, {"Sec-Fetch-Site": "none"}):
            self.assertEqual(post(base + "/api/teleop", {"forward": 0.0, "turn": 0.0}, tl, hdr)[0], 200, hdr)


if __name__ == "__main__":
    unittest.main(verbosity=1)
