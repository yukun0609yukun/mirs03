"""認証・権限・APIの強化(2026-11-04): TL等のログイン回数制限 / restricted_account(restricted)専用ロール / スイッチAPIのenabled必須 / 緊急停止engaged厳密化。

実行: python -m pytest -q -p no:cacheprovider test/test_auth_hardening_20261104.py
"""
import io
import json
import sys
import types
import unittest
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import test_account_roles_http as base  # noqa: E402  (スタブ導入・HTTPヘルパを再利用)

from nokosahen_core import accounts as acc  # noqa: E402
from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import roles  # noqa: E402
from nokosahen_core.http_auth import SessionStore  # noqa: E402

http = base.http
serve = base.serve


class FakeBC(base.FakeBehaviorControl):
    def set_idle_wiggle_enabled(self, enabled):
        self.calls.append(("idle_wiggle", enabled))
        return True, "ok"

    def set_recommend_enabled(self, enabled):
        self.calls.append(("recommend", enabled))
        return True, "ok"


# ---------------------------------------------------------------- roles
class RestrictedRoleTest(unittest.TestCase):
    def auth(self, role, path, data=None, perms=()):
        return roles.authorize(role, perms, "POST", path, data)

    def test_restricted_caps_are_minimal(self):
        self.assertEqual(roles.role_caps(roles.ROLE_RESTRICTED),
                         {"session.logout", "safe.estop_engage", "robot.manual_drive", "behavior.face_switch"})

    def test_restricted_allowed_and_denied(self):
        self.assertTrue(self.auth("restricted", "/api/teleop", {"forward": 0.1}).allowed)
        self.assertTrue(self.auth("restricted", "/api/logout", {}).allowed)
        self.assertTrue(self.auth("restricted", "/api/soft_estop", {"engaged": True}).allowed)
        self.assertTrue(self.auth("restricted", "/api/soft_estop", {}).allowed)
        self.assertFalse(self.auth("restricted", "/api/soft_estop", {"engaged": False}).allowed)
        # 【2026-11-06】face_enabled は restricted にも許可(behavior.face_switch)。他は従来どおり拒否
        self.assertTrue(self.auth("restricted", "/api/face_enabled", {"enabled": True}).allowed)
        for path in ("/api/person_interaction", "/api/scan_enabled", "/api/idle_wiggle", "/api/force_demo",
                     "/api/recommend_enabled", "/api/speed_settings", "/api/bgm", "/api/voice_levels",
                     "/api/goal", "/api/mapping/save", "/api/accounts/permissions", "/api/system/shutdown",
                     "/api/totally_new_api", "/api/goal/cancel"):
            self.assertFalse(self.auth("restricted", path, {"enabled": True}, list(roles.CAPABILITIES)).allowed, path)

    def test_restricted_never_exceeds_admin_or_gets_dangerous_caps(self):
        caps = roles.role_caps(roles.ROLE_RESTRICTED, list(roles.CAPABILITIES))
        self.assertTrue(caps <= roles.role_caps(roles.ROLE_ADMIN))
        for c in ("robot.estop_release", "behavior.control", "robot.speed", "accounts.manage", "system.shutdown"):
            self.assertNotIn(c, caps)

    def test_estop_engaged_strict(self):
        for body in ({}, {"engaged": None}, {"engaged": 0}, {"engaged": ""}, {"engaged": []}, {"engaged": "false"},
                     {"engaged": True}):
            self.assertEqual(roles.required_capabilities("POST", "/api/soft_estop", body), ["safe.estop_engage"], body)
        self.assertEqual(roles.required_capabilities("POST", "/api/soft_estop", {"engaged": False}),
                         ["robot.estop_release"])

    def test_release_admin_and_operator_only(self):
        # 【2026-11-06】operatorも解除可(ユーザーの明示の決定)。viewer・restricted は解除不可、admin(TL)は可
        body = {"engaged": False}
        self.assertTrue(self.auth("operator", "/api/soft_estop", body, []).allowed)
        for role in ("viewer", "restricted"):
            self.assertFalse(self.auth(role, "/api/soft_estop", body, list(roles.CAPABILITIES)).allowed, role)
        self.assertTrue(self.auth("admin", "/api/soft_estop", body).allowed)


class AccountRoleOfTest(unittest.TestCase):
    def test_restricted_name_is_restricted_not_admin(self):
        s = acc.AccountStore(accounts_path="", permissions_path=str(base.TMP / "perm_hard1.json"))
        s.set_reserved(["guest", "TL", "restricted_account"], restricted_names=["restricted_account"])
        self.assertEqual(s.role_of("restricted_account"), roles.ROLE_RESTRICTED)
        self.assertEqual(s.role_of("TL"), roles.ROLE_ADMIN)
        # PC版が後から(restrictedなし)登録しても、restrictedはadminに戻らない
        s.set_reserved(["guest", "TL"])
        self.assertEqual(s.role_of("restricted_account"), roles.ROLE_RESTRICTED)

    def test_listing_shows_restricted_role(self):
        s = acc.AccountStore(accounts_path="", permissions_path=str(base.TMP / "perm_hard2.json"))
        rows = s.listing(admin_names=["TL"], restricted_names=["restricted_account"])
        self.assertEqual([r["role"] for r in rows if r["username"] == "restricted_account"], ["restricted"])


# ---------------------------------------------------------------- login limit
class FakeLoginHandler:
    def __init__(self, ip):
        self.client_address = (ip, 1234)
        self.headers = {}
        self.wfile = io.BytesIO()
        self.status = None
        self.sent = {}

    def send_response(self, code, *a):
        self.status = int(code)

    def send_header(self, k, v):
        self.sent[k] = v

    def end_headers(self):
        pass

    def body(self):
        return json.loads(self.wfile.getvalue().decode("utf-8") or "{}")


class LoginLimitTest(unittest.TestCase):
    ACCOUNTS = {"TL": "tl-pass", "guest": "", "restricted_account": "sk-pass"}

    def setUp(self):
        self.sessions = SessionStore()
        self.limiter = acc.LoginLimiter()
        self.store = acc.AccountStore(accounts_path="", permissions_path=str(base.TMP / "perm_hard3.json"))

    def login(self, user, pw, ip="10.0.0.1"):
        h = FakeLoginHandler(ip)
        dc._handle_login_post(h, {"username": user, "password": pw}, self.ACCOUNTS, self.sessions,
                              self.store, self.limiter)
        return h

    def test_tl_locked_after_failures_even_with_correct_password(self):
        for _ in range(5):
            self.assertEqual(self.login("TL", "bad").status, 401)
        h = self.login("TL", "tl-pass")
        self.assertEqual(h.status, 429)
        self.assertEqual(h.body()["error"], "too_many_attempts")
        self.assertGreater(h.body()["retry_after_sec"], 0)
        self.assertIn("Retry-After", h.sent)

    def test_success_resets_counter(self):
        for _ in range(4):
            self.assertEqual(self.login("TL", "bad").status, 401)
        self.assertEqual(self.login("TL", "tl-pass").status, 200)
        for _ in range(4):
            self.assertEqual(self.login("TL", "bad").status, 401)
        self.assertEqual(self.login("TL", "tl-pass").status, 200)

    def test_restricted_account_limited_too_and_other_account_other_ip_unaffected(self):
        for _ in range(5):
            self.login("TL", "bad", ip="10.0.0.1")
        self.assertEqual(self.login("TL", "tl-pass", ip="10.0.0.1").status, 429)
        self.assertEqual(self.login("restricted_account", "sk-pass", ip="10.0.0.2").status, 200)
        for _ in range(5):
            self.login("restricted_account", "bad", ip="10.0.0.2")
        self.assertEqual(self.login("restricted_account", "sk-pass", ip="10.0.0.2").status, 429)

    def test_guest_never_limited(self):
        for _ in range(8):
            self.login("TL", "bad")
        for _ in range(8):
            self.assertEqual(self.login("guest", "").status, 200)

    def test_unknown_name_failures_are_counted_but_do_not_lock_others(self):
        for _ in range(6):
            self.assertEqual(self.login("nobody", "x").status in (401, 429), True)
        self.assertEqual(self.login("TL", "tl-pass").status, 200)


# ---------------------------------------------------------------- 実HTTP
class MobileRestrictedHttpTest(base.Base):
    def start_mobile(self, bc=None):
        self.teleop = base.FakeTeleop()
        self.bc = bc or FakeBC()
        store = acc.AccountStore(accounts_path="", permissions_path=str(base.TMP / f"perm_{self.id()}.json"))
        handler = dc.make_mobile_handler(
            lambda: b"<html></html>", lambda: {}, self.teleop, auth_username="TL", auth_password="tl-pass",
            restricted_username="restricted_account", restricted_password="sk-pass",
            enforce_single_controller=False, account_store=store, behavior_control=self.bc)
        srv, port = serve(handler)
        self.addCleanup(lambda: (srv.shutdown(), srv.server_close()))
        self.m = f"http://127.0.0.1:{port}"

    def ck(self, user, pw):
        code, d, c = http("POST", self.m + "/api/login", {"username": user, "password": pw})
        self.assertEqual(code, 200, user)
        return c.split(";")[0]

    def test_restricted_account_drive_and_engage_only(self):
        self.start_mobile()
        sk = self.ck("restricted_account", "sk-pass")
        self.assertEqual(http("GET", self.m + "/api/whoami", cookie=sk)[1]["role"], "restricted")
        self.assertEqual(http("POST", self.m + "/api/teleop", {"forward": 0.0, "turn": 0.0}, sk)[0], 200)
        self.assertEqual(http("POST", self.m + "/api/soft_estop", {"engaged": True}, sk)[0], 200)
        self.assertEqual(http("POST", self.m + "/api/soft_estop", {"engaged": False}, sk)[0], 403)
        self.assertEqual(self.teleop.estops, [True])
        self.assertTrue(self.teleop.soft_estop_engaged)
        for path in ("/api/person_interaction", "/api/scan_enabled", "/api/idle_wiggle"):
            self.assertEqual(http("POST", self.m + path, {"enabled": True}, sk)[0], 403, path)
        self.assertEqual(self.bc.calls, [])
        # 【2026-11-06】「人の方を向く」だけはrestricted_accountも切り替えられる
        self.assertEqual(http("POST", self.m + "/api/face_enabled", {"enabled": True}, sk)[0], 200)
        self.assertEqual(self.bc.calls, [("face", True)])
        self.bc.calls.clear()
        for path, body in (("/api/speed_settings", {"manual_linear_mps": 0.3}), ("/api/bgm", {"volume": 1})):
            self.assertEqual(http("POST", self.m + path, body, sk)[0], 403, path)

    def test_tl_can_release_estop(self):
        self.start_mobile()
        tl = self.ck("TL", "tl-pass")
        self.assertEqual(http("POST", self.m + "/api/soft_estop", {"engaged": True}, tl)[0], 200)
        self.assertEqual(http("POST", self.m + "/api/soft_estop", {"engaged": False}, tl)[0], 200)
        self.assertEqual(self.teleop.estops, [True, False])

    def test_estop_non_false_values_engage(self):
        self.start_mobile()
        tl = self.ck("TL", "tl-pass")
        for body in ({}, {"engaged": None}, {"engaged": 0}, {"engaged": ""}, {"engaged": []}, {"engaged": "false"}):
            self.teleop.estops.clear()
            self.assertEqual(http("POST", self.m + "/api/soft_estop", body, tl)[0], 200)
            self.assertEqual(self.teleop.estops, [True], body)

    def test_mobile_login_limit_for_tl_and_restricted_account(self):
        self.start_mobile()
        for _ in range(5):
            self.assertEqual(http("POST", self.m + "/api/login", {"username": "TL", "password": "bad"})[0], 401)
        code, d, _ = http("POST", self.m + "/api/login", {"username": "TL", "password": "tl-pass"})
        self.assertEqual((code, d["error"]), (429, "too_many_attempts"))
        self.assertEqual(http("POST", self.m + "/api/login", {"username": "guest", "password": ""})[0], 200)


SWITCHES = (
    ("/api/person_interaction", "person_interaction"),
    ("/api/scan_enabled", "scan"),
    ("/api/face_enabled", "face"),
    ("/api/idle_wiggle", "idle_wiggle"),
)


class SwitchRoutesTest(base.Base):
    def _check(self, url, tl, bc, routes):
        for path, name in routes:
            for bad in ({}, {"enabled": "true"}, {"enabled": "false"}, {"enabled": None}, {"enabled": 1}):
                self.assertEqual(http("POST", url + path, bad, tl)[0], 400, (path, bad))
            # 壊れたJSON・空本文も400(以前は{}→ON扱い)
            import urllib.request
            for raw in (b"{broken", b""):
                req = urllib.request.Request(url + path, data=raw, method="POST",
                                             headers={"Content-Type": "application/json", "Cookie": tl})
                try:
                    code = urllib.request.urlopen(req, timeout=10).status
                except urllib.error.HTTPError as e:
                    code = e.code
                    e.close()
                self.assertEqual(code, 400, (path, raw))
            self.assertEqual(bc.calls, [], path)
            self.assertEqual(http("POST", url + path, {"enabled": False}, tl)[0], 200, path)
            self.assertEqual(http("POST", url + path, {"enabled": True}, tl)[0], 200, path)
            self.assertEqual(bc.calls, [(name, False), (name, True)], path)
            bc.calls.clear()

    def test_pc_handler_five_routes(self):
        bc = FakeBC()
        handler = dc.make_dashboard_handler(
            lambda: b"x", lambda: {}, base.FakeTeleop(), auth_username="TL", auth_password="tl-pass",
            enforce_single_controller=False, behavior_control=bc,
            account_store=acc.AccountStore(accounts_path="", permissions_path=str(base.TMP / "perm_sw_pc.json")))
        srv, port = serve(handler)
        self.addCleanup(lambda: (srv.shutdown(), srv.server_close()))
        url = f"http://127.0.0.1:{port}"
        tl = http("POST", url + "/api/login", {"username": "TL", "password": "tl-pass"})[2].split(";")[0]
        self._check(url, tl, bc, SWITCHES + (("/api/recommend_enabled", "recommend"),))

    def test_mobile_handler_four_routes(self):
        bc = FakeBC()
        handler = dc.make_mobile_handler(
            lambda: b"x", lambda: {}, base.FakeTeleop(), auth_username="TL", auth_password="tl-pass",
            enforce_single_controller=False, behavior_control=bc,
            account_store=acc.AccountStore(accounts_path="", permissions_path=str(base.TMP / "perm_sw_m.json")))
        srv, port = serve(handler)
        self.addCleanup(lambda: (srv.shutdown(), srv.server_close()))
        url = f"http://127.0.0.1:{port}"
        tl = http("POST", url + "/api/login", {"username": "TL", "password": "tl-pass"})[2].split(";")[0]
        self._check(url, tl, bc, SWITCHES)


if __name__ == "__main__":
    unittest.main(verbosity=1)
