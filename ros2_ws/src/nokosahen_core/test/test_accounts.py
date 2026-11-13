"""operatorアカウントの枠(アカウント表・パスワードハッシュ・ログイン回数制限)のテスト(2026-10-13追加、docs/accounts.md)。

rclpy・ネットワーク不要。ハッシュは軽いパラメータでテストする(本番の既定パラメータは別テストで形式だけ確認)。
- ハッシュ: 検証、塩付き(同じパスワードでも別のハッシュ)、空パスワード不可、不正ハッシュ不可、平文を含まない
- 既定(ファイル無し)はプレースホルダー・無効でログイン不可
- enabled:false / プレースホルダー名 / ハッシュ空・不正 / 空パスワードのいずれでもログイン不可
- 既存アカウント名(管理者・guest)との衝突・重複・role:admin定義は弾く
- 許可リスト(ファイル値・永続ファイル、admin_only混入の扱い、原子的保存)
- レートリミット(ロックアウト・解除・成功でリセット)
- scripts/make_account_hash.py の出力がハッシュだけ(パスワードを出さない)

実行: python ros2_ws/src/nokosahen_core/test/test_accounts.py
"""
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
REPO = PKG.parents[2]
sys.path.insert(0, str(PKG))

from nokosahen_core import accounts as acc  # noqa: E402
from nokosahen_core import roles  # noqa: E402

TMP = Path(tempfile.mkdtemp(prefix="accounts_test_"))
FAST = dict(n=2 ** 10)  # テスト用の軽いscrypt


def H(pw):
    return acc.hash_password(pw, **FAST)


def write_yaml(name, text):
    p = TMP / name
    p.write_text(text, encoding="utf-8")
    return str(p)


class HashTest(unittest.TestCase):
    def test_roundtrip_and_wrong(self):
        h = H("correct horse")
        self.assertTrue(acc.verify_password("correct horse", h))
        self.assertFalse(acc.verify_password("correct horsE", h))
        self.assertFalse(acc.verify_password("", h))
        self.assertFalse(acc.verify_password(None, h))

    def test_salted(self):
        self.assertNotEqual(H("same"), H("same"))

    def test_no_plaintext_in_hash(self):
        self.assertNotIn("topsecret", H("topsecret"))

    def test_empty_password_cannot_be_hashed(self):
        with self.assertRaises(ValueError):
            acc.hash_password("")

    def test_bad_or_empty_hashes_never_verify(self):
        for bad in ("", None, "plain", "scrypt$x$y", "scrypt$1024$8$1$!!$!!", "pbkdf2_sha256$1$a$b", 123):
            self.assertFalse(acc.verify_password("anything", bad), bad)
            self.assertFalse(acc.verify_password("", bad), bad)

    def test_pbkdf2_fallback_roundtrip(self):
        h = acc.hash_password("pw-1234567", scheme="pbkdf2_sha256", iterations=2000)
        self.assertTrue(h.startswith("pbkdf2_sha256$"))
        self.assertTrue(acc.verify_password("pw-1234567", h))
        self.assertFalse(acc.verify_password("pw-1234568", h))

    def test_japanese_password(self):
        h = H("ぱすわーど123")
        self.assertTrue(acc.verify_password("ぱすわーど123", h))

    def test_default_parameters_form(self):
        h = acc.hash_password("x" * 10)
        self.assertEqual(h.split("$")[:4], ["scrypt", str(acc.SCRYPT_N), str(acc.SCRYPT_R), str(acc.SCRYPT_P)])


class StoreTest(unittest.TestCase):
    def store(self, text=None, perms_name="perm.json", reserved=("guest", "TL")):
        path = write_yaml("acc_%d.yaml" % id(self), text) if text is not None else ""
        perm = str(TMP / (perms_name + str(id(self))))
        if os.path.exists(perm):
            os.remove(perm)
        return acc.AccountStore(reserved_names=reserved, accounts_path=path, permissions_path=perm)

    def test_default_is_disabled_placeholder(self):
        s = self.store(None)
        self.assertTrue(s.is_operator_name("CHANGE_ME_OPERATOR"))
        self.assertFalse(s.verify_operator("CHANGE_ME_OPERATOR", ""))
        self.assertFalse(s.verify_operator("CHANGE_ME_OPERATOR", "CHANGE_ME_OPERATOR"))
        self.assertEqual(s.operator_permissions(), frozenset())
        rows = s.listing(admin_names=["TL"])
        self.assertEqual([r["username"] for r in rows], ["TL", "guest", "CHANGE_ME_OPERATOR"])
        self.assertFalse(rows[-1]["enabled"])
        self.assertTrue(rows[-1]["placeholder"])

    def test_enabled_account_logs_in_only_with_right_password(self):
        h = H("op-password-1")
        s = self.store(f'accounts:\n  - username: "kari-op"\n    role: operator\n    password_hash: "{h}"\n    enabled: true\n')
        self.assertTrue(s.verify_operator("kari-op", "op-password-1"))
        self.assertFalse(s.verify_operator("kari-op", "op-password-2"))
        self.assertFalse(s.verify_operator("kari-op", ""))
        self.assertFalse(s.verify_operator("other", "op-password-1"))
        self.assertEqual(s.role_of("kari-op"), roles.ROLE_OPERATOR)
        self.assertEqual(s.role_of("TL"), roles.ROLE_ADMIN)
        self.assertEqual(s.role_of("guest"), roles.ROLE_VIEWER)
        self.assertEqual(s.role_of(""), roles.ROLE_ADMIN)  # 認証無効(開発用)は従来どおり

    def test_enabled_false_cannot_login_even_with_correct_hash(self):
        h = H("op-password-1")
        s = self.store(f'accounts:\n  - username: "kari-op"\n    role: operator\n    password_hash: "{h}"\n    enabled: false\n')
        self.assertFalse(s.verify_operator("kari-op", "op-password-1"))
        self.assertIn("enabled:false", s.listing()[-1]["note"])

    def test_enabled_must_be_boolean_true(self):
        h = H("op-password-1")
        for val in ('"true"', "1", '"yes"'):
            s = self.store(f'accounts:\n  - username: "kari-op"\n    role: operator\n    password_hash: "{h}"\n    enabled: {val}\n')
            self.assertFalse(s.verify_operator("kari-op", "op-password-1"), val)

    def test_placeholder_name_never_logs_in(self):
        h = H("op-password-1")
        for name in ("CHANGE_ME_OPERATOR", "change_me_x", "   "):
            s = self.store(f'accounts:\n  - username: "{name}"\n    role: operator\n    password_hash: "{h}"\n    enabled: true\n')
            self.assertFalse(s.verify_operator(name, "op-password-1"), name)

    def test_empty_or_bad_hash_never_logs_in(self):
        for h in ("", "garbage", "scrypt$1$2$3$a$b"):
            s = self.store(f'accounts:\n  - username: "kari-op"\n    role: operator\n    password_hash: "{h}"\n    enabled: true\n')
            self.assertFalse(s.verify_operator("kari-op", ""), h)
            self.assertFalse(s.verify_operator("kari-op", "garbage"), h)
        s = self.store('accounts:\n  - username: "kari-op"\n    role: operator\n    enabled: true\n')
        self.assertFalse(s.verify_operator("kari-op", "x"))

    def test_reserved_names_duplicates_and_other_roles_rejected(self):
        h = H("op-password-1")
        text = (
            f'accounts:\n'
            f'  - {{username: "TL", role: operator, password_hash: "{h}", enabled: true}}\n'
            f'  - {{username: "guest", role: operator, password_hash: "{h}", enabled: true}}\n'
            f'  - {{username: "boss", role: admin, password_hash: "{h}", enabled: true}}\n'
            f'  - {{username: "dup", role: operator, password_hash: "{h}", enabled: true}}\n'
            f'  - {{username: "dup", role: operator, password_hash: "{h}", enabled: true}}\n'
        )
        s = self.store(text)
        self.assertFalse(s.is_operator_name("TL"))
        self.assertFalse(s.is_operator_name("guest"))
        self.assertFalse(s.is_operator_name("boss"))
        self.assertEqual(s.role_of("TL"), roles.ROLE_ADMIN)  # 既存アカウントはロールが変わらない
        # 【2026-10-21】定義ファイルでrole:adminと書いても昇格しない(operatorにもならない)。登録のない名前は閲覧のみ
        # (以前は「それ以外は全部admin」で、削除・無効化したoperatorのセッションがadminに昇格していた)。
        self.assertEqual(s.role_of("boss"), roles.ROLE_VIEWER)
        self.assertTrue(s.is_operator_name("dup"))
        self.assertGreaterEqual(len(s.problems), 4)

    def test_broken_file_disables_everything(self):
        s = self.store("accounts: [unclosed\n  - : :")
        self.assertFalse(s.verify_operator("kari-op", "x"))
        self.assertTrue(any("読めない" in p for p in s.problems))

    def test_listing_never_contains_hash_or_password(self):
        h = H("op-password-1")
        s = self.store(f'accounts:\n  - username: "kari-op"\n    role: operator\n    password_hash: "{h}"\n    enabled: true\n')
        dumped = json.dumps(s.listing(admin_names=["TL"]), ensure_ascii=False)
        self.assertNotIn(h, dumped)
        self.assertNotIn("password", dumped.replace("password_hash", "").lower().replace("パスワード", ""))
        s.note_login("kari-op")
        row = [r for r in s.listing() if r["username"] == "kari-op"][0]
        self.assertTrue(row["last_login"])

    def test_file_edit_is_picked_up(self):
        h = H("op-password-1")
        p = TMP / "acc_reload.yaml"
        p.write_text(f'accounts:\n  - username: "kari-op"\n    role: operator\n    password_hash: "{h}"\n    enabled: false\n', encoding="utf-8")
        s = acc.AccountStore(reserved_names=("guest",), accounts_path=str(p), permissions_path=str(TMP / "perm_reload.json"))
        self.assertFalse(s.verify_operator("kari-op", "op-password-1"))
        p.write_text(f'accounts:\n  - username: "kari-op"\n    role: operator\n    password_hash: "{h}"\n    enabled: true\n  - username: "x"\n', encoding="utf-8")
        self.assertTrue(s.verify_operator("kari-op", "op-password-1"))


class PermissionsTest(unittest.TestCase):
    def mk(self, yaml_text="", perm_exists=False):
        n = "perm_%d_%s" % (id(self), self._testMethodName)
        path = write_yaml(n + ".yaml", yaml_text)
        perm = TMP / (n + ".json")
        if perm.exists():
            perm.unlink()
        return acc.AccountStore(reserved_names=("guest",), accounts_path=path, permissions_path=str(perm)), perm

    def test_default_nothing_granted(self):
        s, _ = self.mk("")
        self.assertEqual(s.operator_permissions(), frozenset())

    def test_yaml_list_grants_and_admin_only_is_ignored_with_warning(self):
        s, _ = self.mk("operator_permissions: [voice.levels, robot.speed, accounts.manage, bogus]\n")
        self.assertEqual(s.operator_permissions(), {"voice.levels"})
        self.assertEqual(len([p for p in s.problems if "付与できません" in p or "未知" in p]), 3)

    def test_save_is_atomic_json_and_wins_over_yaml(self):
        s, perm = self.mk("operator_permissions: [voice.levels]\n")
        granted, _ = s.save_permissions(["voice.master", "display.survey_mode"], updated_by="TL")
        self.assertEqual(granted, {"voice.master", "display.survey_mode"})
        doc = json.loads(perm.read_text(encoding="utf-8"))
        self.assertEqual(doc["operator_permissions"], ["display.survey_mode", "voice.master"])
        self.assertEqual(doc["updated_by"], "TL")
        self.assertEqual(s.operator_permissions(), {"voice.master", "display.survey_mode"})  # yamlのvoice.levelsより優先
        self.assertEqual([p.name for p in perm.parent.glob(".nokosahen_operator_perm_*")], [])  # 一時ファイルが残らない
        s.save_permissions([], updated_by="TL")  # 全て外せる(yamlに戻らない)
        self.assertEqual(s.operator_permissions(), frozenset())

    def test_save_rejects_admin_only_deferred_unknown_and_writes_nothing(self):
        s, perm = self.mk("")
        for bad in (["robot.speed"], ["voice.levels", "accounts.manage"], ["voice.bgm_onoff"], ["nope"]):
            with self.assertRaises(roles.PermissionConfigError):
                s.save_permissions(bad)
        self.assertFalse(perm.exists())
        self.assertEqual(s.operator_permissions(), frozenset())

    def test_broken_permission_file_grants_nothing(self):
        s, perm = self.mk("operator_permissions: [voice.levels]\n")
        perm.write_text("{ not json", encoding="utf-8")
        self.assertEqual(s.operator_permissions(), frozenset())  # 壊れていたら何も許可しない(yamlにも戻さない)
        self.assertTrue(any("許可リスト" in p for p in s.problems))

    def test_hand_edited_permission_file_with_admin_only_is_sanitized(self):
        s, perm = self.mk("")
        perm.write_text(json.dumps({"operator_permissions": ["voice.levels", "robot.speed"]}), encoding="utf-8")
        self.assertEqual(s.operator_permissions(), {"voice.levels"})


class LimiterTest(unittest.TestCase):
    def test_lockout_and_release(self):
        now = [1000.0]
        lim = acc.LoginLimiter(max_failures=3, window_sec=600, lockout_sec=300, clock=lambda: now[0])
        keys = ("user:a", "ip:1.2.3.4")
        self.assertEqual(lim.retry_after(keys), 0)
        lim.record_failure(keys)
        lim.record_failure(keys)
        self.assertEqual(lim.retry_after(keys), 0)
        lim.record_failure(keys)
        self.assertGreater(lim.retry_after(keys), 290)
        now[0] += 299
        self.assertGreater(lim.retry_after(keys), 0)
        now[0] += 2
        self.assertEqual(lim.retry_after(keys), 0)  # 解除
        lim.record_failure(keys)
        self.assertEqual(lim.retry_after(keys), 0)  # 解除後は数え直し

    def test_success_resets(self):
        lim = acc.LoginLimiter(max_failures=2)
        keys = ("user:a",)
        lim.record_failure(keys)
        lim.record_success(keys)
        lim.record_failure(keys)
        self.assertEqual(lim.retry_after(keys), 0)

    def test_other_key_not_locked(self):
        lim = acc.LoginLimiter(max_failures=2)
        lim.record_failure(("user:a", "ip:x"))
        lim.record_failure(("user:a", "ip:x"))
        self.assertGreater(lim.retry_after(("user:a",)), 0)
        self.assertGreater(lim.retry_after(("ip:x",)), 0)
        self.assertEqual(lim.retry_after(("user:b", "ip:y")), 0)

    def test_old_failures_expire(self):
        now = [0.0]
        lim = acc.LoginLimiter(max_failures=3, window_sec=60, lockout_sec=300, clock=lambda: now[0])
        lim.record_failure(("k",))
        lim.record_failure(("k",))
        now[0] += 120
        lim.record_failure(("k",))
        self.assertEqual(lim.retry_after(("k",)), 0)


class HashScriptTest(unittest.TestCase):
    def test_script_prints_only_hash(self):
        # getpassを差し替えて対話入力を模擬し、標準出力が「ハッシュ1行だけ」であることを確認する
        spec = importlib.util.spec_from_file_location("mk_hash", str(REPO / "scripts" / "make_account_hash.py"))
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
        import contextlib
        import getpass
        import io
        from unittest import mock

        out, err = io.StringIO(), io.StringIO()
        with mock.patch.object(getpass, "getpass", side_effect=["pass-for-test-1", "pass-for-test-1"]), \
                contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            rc = m.main()
        self.assertEqual(rc, 0)
        lines = out.getvalue().splitlines()
        self.assertEqual(len(lines), 1)
        self.assertTrue(lines[0].startswith("scrypt$"))
        self.assertNotIn("pass-for-test-1", out.getvalue() + err.getvalue())
        self.assertTrue(acc.verify_password("pass-for-test-1", lines[0]))

    def test_script_rejects_empty_short_and_mismatch(self):
        spec = importlib.util.spec_from_file_location("mk_hash2", str(REPO / "scripts" / "make_account_hash.py"))
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
        import contextlib
        import getpass
        import io
        from unittest import mock

        for answers in (["", ""], ["short", "short"], ["longenough1", "different11"]):
            out = io.StringIO()
            with mock.patch.object(getpass, "getpass", side_effect=answers), \
                    contextlib.redirect_stdout(out), contextlib.redirect_stderr(io.StringIO()):
                rc = m.main()
            self.assertNotEqual(rc, 0, answers)
            self.assertEqual(out.getvalue(), "")  # 失敗時はハッシュを出さない


class RoleOfSafetyTest(unittest.TestCase):
    """【2026-10-21】登録のない名前・削除/無効化したoperator・定義ファイルが壊れた場合は、adminに昇格せず閲覧のみ(viewer)。"""

    def store(self, text, reserved=("guest", "TL", "restricted_account")):
        path = write_yaml("rof_%d_%d.yaml" % (id(self), len(text)), text) if text is not None else ""
        perm = str(TMP / ("perm_rof_%d_%d.json" % (id(self), len(str(text)))))
        return acc.AccountStore(reserved_names=reserved, accounts_path=path, permissions_path=perm)

    def operator_text(self, name="op1", enabled="true"):
        h = H("op-password-1")
        return f'accounts:\n  - {{username: "{name}", role: operator, password_hash: "{h}", enabled: {enabled}}}\n'

    def test_removed_operator_is_not_admin(self):
        s = self.store(self.operator_text())
        self.assertEqual(s.role_of("op1"), roles.ROLE_OPERATOR)
        # 定義ファイルから消えた(別のoperatorだけにした)あと、ログイン中だったop1のセッションはadminにならない
        p = Path(s.source_path)
        p.write_text(self.operator_text("op2"), encoding="utf-8")
        os.utime(p, (p.stat().st_atime + 5, p.stat().st_mtime + 5))
        self.assertEqual(s.role_of("op1"), roles.ROLE_VIEWER)
        self.assertEqual(s.role_of("op2"), roles.ROLE_OPERATOR)

    def test_disabled_operator_is_viewer_not_operator(self):
        s = self.store(self.operator_text(enabled="false"))
        self.assertEqual(s.role_of("op1"), roles.ROLE_VIEWER)

    def test_broken_file_never_promotes_to_admin(self):
        s = self.store(self.operator_text())
        p = Path(s.source_path)
        p.write_text("accounts: [ {this is: not valid yaml", encoding="utf-8")
        os.utime(p, (p.stat().st_atime + 5, p.stat().st_mtime + 5))
        self.assertNotEqual(s.role_of("op1"), roles.ROLE_ADMIN)
        self.assertEqual(s.role_of("op1"), roles.ROLE_VIEWER)

    def test_only_registered_names_are_admin(self):
        s = self.store(None)
        s.set_reserved(["guest", "TL", "restricted_account"], restricted_names=["restricted_account"])
        self.assertEqual(s.role_of("TL"), roles.ROLE_ADMIN)
        # 【2026-11-04変更】スマホ用の専用アカウントはadminではなくrestricted(運転と緊急停止をかけるまで)
        self.assertEqual(s.role_of("restricted_account"), roles.ROLE_RESTRICTED)
        self.assertEqual(s.role_of("anybody"), roles.ROLE_VIEWER)
        self.assertEqual(s.role_of("guest"), roles.ROLE_VIEWER)
        self.assertEqual(s.role_of(""), roles.ROLE_ADMIN)  # 認証無効(開発用)

    def test_set_reserved_accumulates_admin_names(self):
        # PC版(TLだけ)とスマホ版(TLと専用アカウント)が同じストアを共有しても、先に登録した名前が消えない
        s = acc.AccountStore(reserved_names=("guest",), accounts_path="", permissions_path=str(TMP / "perm_acc_sr.json"))
        s.set_reserved(["guest", "TL", "restricted_account"], restricted_names=["restricted_account"])
        s.set_reserved(["guest", "TL"])
        self.assertEqual(s.role_of("restricted_account"), roles.ROLE_RESTRICTED)  # 【2026-11-04変更】adminではなくrestricted
        self.assertEqual(s.role_of("TL"), roles.ROLE_ADMIN)


class SessionCapTest(unittest.TestCase):
    """【2026-10-21】guestの連続ログインで、TL・operatorのセッションが追い出されない。"""

    def test_guest_flood_does_not_evict_tl(self):
        from nokosahen_core import http_auth
        st = http_auth.SessionStore(max_sessions=256, max_guest_sessions=16)
        tl = st.create("TL")
        op = st.create("op1")
        for _ in range(1000):
            st.create("guest")
        self.assertEqual(st.resolve(tl), "TL")
        self.assertEqual(st.resolve(op), "op1")
        guests = [t for t, (a, _) in st._sessions.items() if a == "guest"]
        self.assertLessEqual(len(guests), 16)

    def test_global_cap_evicts_guests_first(self):
        from nokosahen_core import http_auth
        st = http_auth.SessionStore(max_sessions=4, max_guest_sessions=3)
        tl = st.create("TL")
        for _ in range(3):
            st.create("guest")
        op = st.create("op1")  # 全体が上限 → guestの古いものから捨てる
        self.assertEqual(st.resolve(tl), "TL")
        self.assertEqual(st.resolve(op), "op1")


class HandlerTimeoutTest(unittest.TestCase):
    """【2026-10-21】HTTPハンドラに読み取りタイムアウトがある(本文を送らない接続でスレッドが永久に残らない)。"""

    def test_handlers_define_timeout(self):
        import re
        base = PKG / "nokosahen_core"
        for name in ("dashboard_common.py", "web_ui_node.py"):
            t = (base / name).read_text(encoding="utf-8")
            n_handlers = len(re.findall(r"class Handler\(BaseHTTPRequestHandler\):", t))
            n_timeouts = len(re.findall(r"^\s+timeout = \d+\s*$", t, re.M))
            self.assertGreaterEqual(n_timeouts, n_handlers, name)


if __name__ == "__main__":
    unittest.main(verbosity=1)
