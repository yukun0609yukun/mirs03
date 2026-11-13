"""文化祭前の確認(2026-11-06): 「人の方を向く」スイッチ(/api/face_enabled)を、ロール別に切り替えられるか。
PC版(8090)とスマホ版(8095)の本物のHTTPハンドラを実際に起動して確認する。ロボット・ROSには触れない(スイッチ呼び出しは記録するだけの替え玉)。
ロール: admin(TL) / restricted(restricted_account、スマホ版のみ) / operator(弱いアカウント、全権限を許可しても) / viewer(guest) / 未ログイン。
実行: python ros2_ws/src/nokosahen_core/test/test_festival_roles_20261106.py
"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import test_account_roles_http as t  # noqa: E402  (スタブ注入・偽サーバー・http()を流用)

dc, roles = t.dc, t.roles
serve, http = t.serve, t.http


class FaceSwitchRoles(unittest.TestCase):
    PW = {"TL": "tl-pass", "restricted_account": "sho-pass", "kari-op": t.OP_PASSWORD}
    results = {}

    def setUp(self):
        self.bc = t.FakeBehaviorControl()
        self.teleop = t.FakeTeleop()
        tmp = t.TMP
        af = tmp / f"acc_{self.id()}.yaml"
        # operatorには「付与できる権限を全部」許可しておく(それでも根幹のスイッチは通らないはず)
        perms = sorted(roles.GRANTABLE)
        af.write_text('accounts:\n  - username: "kari-op"\n    role: operator\n'
                      f'    password_hash: "{t.OP_HASH}"\n    enabled: true\n\noperator_permissions: {t.json.dumps(perms)}\n',
                      encoding="utf-8")
        pf = tmp / f"perm_{self.id()}.json"
        if pf.exists():
            pf.unlink()
        store_pc = t.acc.AccountStore(accounts_path=str(af), permissions_path=str(pf))
        store_mb = t.acc.AccountStore(accounts_path=str(af), permissions_path=str(pf))
        pc = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, self.teleop, behavior_control=self.bc,
            auth_username="TL", auth_password=self.PW["TL"], enforce_single_controller=False,
            account_store=store_pc, ephemeral=t.eo.EphemeralOverrides(None))
        mb = dc.make_mobile_handler(
            lambda: b"<html></html>", lambda: {"face_enabled": True}, self.teleop,
            auth_username="TL", auth_password=self.PW["TL"], restricted_username="restricted_account",
            restricted_password=self.PW["restricted_account"], enforce_single_controller=False,
            behavior_control=self.bc, account_store=store_mb, ephemeral=t.eo.EphemeralOverrides(None))
        self.pc_srv, pcp = serve(pc)
        self.mb_srv, mbp = serve(mb)
        self.base = {"PC8090": f"http://127.0.0.1:{pcp}", "スマホ8095": f"http://127.0.0.1:{mbp}"}
        self.addCleanup(lambda: [(s.shutdown(), s.server_close()) for s in (self.pc_srv, self.mb_srv)])

    def cookie(self, base, user):
        pw = "" if user == "guest" else self.PW[user]
        code, d, ck = http("POST", base + "/api/login", {"username": user, "password": pw})
        return code, (ck.split(";")[0] if ck else "")

    def post(self, base, ck, enabled):
        return http("POST", base + "/api/face_enabled", {"enabled": enabled}, ck)[:2]

    def get_state(self, base, ck):
        return http("GET", base + "/api/state", None, ck)[:2]

    def check(self, name, user, expect_on, expect_off, expect_state=200):
        out = {}
        for label, base in self.base.items():
            if user == "restricted_account" and label.startswith("PC"):
                code, _ = self.cookie(base, user)
                out[label] = f"ログイン不可({code})"
                self.assertEqual(code, 401)
                continue
            if user is None:
                ck = None
            else:
                code, ck = self.cookie(base, user)
                self.assertEqual(code, 200, (label, user))
            c_on, _ = self.post(base, ck, True)
            c_off, _ = self.post(base, ck, False)
            c_st, _ = self.get_state(base, ck)
            self.assertEqual(c_on, expect_on, (label, user, "ON"))
            self.assertEqual(c_off, expect_off, (label, user, "OFF"))
            self.assertEqual(c_st, expect_state, (label, user, "state"))
            out[label] = f"ON={c_on} OFF={c_off} 状態取得={c_st}"
        print(f"\n[{name}] " + " / ".join(f"{k}: {v}" for k, v in out.items()))
        return out

    def test_admin_TL_can_toggle(self):
        self.check("admin(TL)", "TL", 200, 200)
        self.assertIn(("face", True), self.bc.calls)
        self.assertIn(("face", False), self.bc.calls)

    # 【2026-11-06 仕様変更】「人の方を向く」だけは弱いアカウント(restricted・operator)にも許可(behavior.face_switch)。
    # 以前は「403のはず」を検証していたが、文化祭で班員がこのスイッチだけをONにする運用にするため200に変えた
    # (「ロール別に挙動が決まっている」という検証の意図は維持。ほかのスイッチは403のまま下の表で確認する)。
    def test_restricted_restricted_account_can_toggle_face(self):
        self.check("restricted(restricted_account)", "restricted_account", 200, 200)
        self.assertEqual(set(self.bc.calls), {("face", True), ("face", False)})

    def test_operator_can_toggle_face_with_all_grants(self):
        self.check("operator(全権限許可)", "kari-op", 200, 200)
        self.assertEqual(set(self.bc.calls), {("face", True), ("face", False)})

    def test_operator_without_any_grant_can_toggle_face_only(self):
        af = t.TMP / f"acc_none_{self.id()}.yaml"
        af.write_text('accounts:\n  - username: "kari-op"\n    role: operator\n'
                      f'    password_hash: "{t.OP_HASH}"\n    enabled: true\n\noperator_permissions: []\n', encoding="utf-8")
        pf = t.TMP / f"perm_none_{self.id()}.json"
        if pf.exists():
            pf.unlink()
        store = t.acc.AccountStore(accounts_path=str(af), permissions_path=str(pf))
        pc = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, self.teleop, behavior_control=self.bc,
            auth_username="TL", auth_password=self.PW["TL"], enforce_single_controller=False,
            account_store=store, ephemeral=t.eo.EphemeralOverrides(None))
        srv, port = serve(pc)
        self.addCleanup(lambda: (srv.shutdown(), srv.server_close()))
        base = f"http://127.0.0.1:{port}"
        code, ck = self.cookie(base, "kari-op")
        self.assertEqual(code, 200)
        self.assertEqual(self.post(base, ck, True)[0], 200)
        self.assertEqual(http("POST", base + "/api/scan_enabled", {"enabled": True}, ck)[0], 403)
        self.assertIn("behavior.face_switch", http("GET", base + "/api/whoami", None, ck)[1]["permissions"])

    def test_guest_cannot(self):
        self.check("viewer(guest)", "guest", 403, 403)
        self.assertEqual(self.bc.calls, [])

    def test_not_logged_in(self):
        self.check("未ログイン", None, 401, 401, expect_state=401)
        self.assertEqual(self.bc.calls, [])

    # ---- 全ロール x (PC版, スマホ版) x (face ON/OFF, 近づく, 見回す, 揺動, 状態取得) の表 ----
    TABLE = {
        "TL": (200, 200, 200, 200, 200),
        "restricted_account": (200, 200, 403, 403, 403),
        "kari-op": (200, 200, 403, 403, 403),
        "guest": (403, 403, 403, 403, 403),
        None: (401, 401, 401, 401, 401),
    }

    def test_full_matrix(self):
        rows = []
        for user, exp in self.TABLE.items():
            for label, base in self.base.items():
                if user == "restricted_account" and label.startswith("PC"):
                    continue  # restricted_account はスマホ版だけにログインできる(PC版は401)
                ck = None if user is None else self.cookie(base, user)[1]
                got = (self.post(base, ck, True)[0], self.post(base, ck, False)[0],
                       http("POST", base + "/api/person_interaction", {"enabled": True}, ck)[0],
                       http("POST", base + "/api/scan_enabled", {"enabled": True}, ck)[0],
                       http("POST", base + "/api/idle_wiggle", {"enabled": True}, ck)[0])
                self.assertEqual(got, exp, (user, label))
                state = http("GET", base + "/api/state", None, ck)[0]
                self.assertEqual(state, 401 if user is None else 200, (user, label, "state"))
                rows.append((user, label, got, state))
        print("\n[ロール x 画面 の表: (face ON, face OFF, 近づく, 見回す, 揺動), 状態取得]")
        for r in rows:
            print("  ", r)

    def test_weak_accounts_cannot_do_anything_else(self):
        # 弱いアカウントは face 以外のスイッチ・デモ強制・推奨発話・緊急停止の解除・速度・アカウント管理ができない(スイッチも呼ばれない)
        # (operatorはこのテストでは付与可能な権限を全部持つが、behavior.control・解除・速度・アカウント管理は付与できないので403)
        for user in ("restricted_account", "kari-op"):
            for label, base in self.base.items():
                if user == "restricted_account" and label.startswith("PC"):
                    continue
                ck = self.cookie(base, user)[1]
                for path, b in (("/api/person_interaction", {"enabled": True}), ("/api/scan_enabled", {"enabled": True}),
                                ("/api/idle_wiggle", {"enabled": True}), ("/api/force_demo", {"enabled": True}),
                                ("/api/recommend_enabled", {"enabled": True}),
                                ("/api/speed_settings", {"manual_linear_mps": 0.3}),
                                ("/api/accounts/permissions", {"operator_permissions": ["behavior.control"]})):
                    self.assertEqual(http("POST", base + path, b, ck)[0], 403, (user, label, path))
        self.assertEqual(self.bc.calls, [])
        self.assertEqual(self.teleop.estops, [])

    def test_shutdown_not_granted_by_face_switch(self):
        # restricted は face_switch を持っていてもシャットダウンは不可(operator は別途の許可リストによる)
        r = roles
        self.assertFalse(r.authorize("restricted", [], "POST", "/api/system/shutdown", {}).allowed)
        self.assertFalse(r.authorize("restricted", [], "POST", "/api/force_demo", {}).allowed)
        self.assertFalse(r.authorize("operator", [], "POST", "/api/system/shutdown", {}).allowed)

    def test_required_capability_and_tables(self):
        self.assertEqual(roles.required_capabilities("POST", "/api/face_enabled", {"enabled": True}), ["behavior.face_switch"])
        for p in ("/api/person_interaction", "/api/scan_enabled", "/api/idle_wiggle", "/api/force_demo", "/api/recommend_enabled"):
            self.assertEqual(roles.required_capabilities("POST", p, {"enabled": True}), ["behavior.control"], p)
        self.assertIn("behavior.control", roles.ADMIN_ONLY)
        self.assertNotIn("behavior.control", roles.GRANTABLE)
        self.assertNotIn("behavior.control", roles.ROLE_BASE_CAPS[roles.ROLE_RESTRICTED])
        self.assertNotIn("behavior.control", roles.ROLE_BASE_CAPS[roles.ROLE_OPERATOR])
        self.assertIn("behavior.face_switch", roles.ROLE_BASE_CAPS[roles.ROLE_RESTRICTED])
        self.assertIn("behavior.face_switch", roles.ROLE_BASE_CAPS[roles.ROLE_OPERATOR])
        self.assertNotIn("behavior.face_switch", roles.ROLE_BASE_CAPS[roles.ROLE_VIEWER])
        self.assertIn("behavior.face_switch", roles.ROLE_BASE_CAPS[roles.ROLE_ADMIN])
        self.assertNotIn("behavior.face_switch", roles.GRANTABLE)  # 許可リストの選択肢には出ない(ロールの既定)


if __name__ == "__main__":
    unittest.main(verbosity=2)
