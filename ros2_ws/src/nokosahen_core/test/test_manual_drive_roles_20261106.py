"""手動運転(WASD・スマホのボタン)を、弱いアカウント(operator)にも許可した確認(2026-11-06)。
PC版(8090)とスマホ版(8095)の本物のHTTPハンドラを実際に起動して確認する。ロボット・ROSには触れない(替え玉)。
ロール: admin(TL) / restricted(restricted_account、スマホ版のみ) / operator(許可リストが空でも、全権限を許可しても) / viewer(guest) / 未ログイン。

- 表: 運転(前進/停止) x 緊急停止をかける x 緊急停止の解除 x 速度設定 x PID x 探索 x 近づく x 見回す
- 優先度(本物の TeleopController): TLの運転中は operator / restricted_account の指令が破棄される。TLが止めて保持時間が過ぎれば通る
- 緊急停止中は operator の運転も受け付けない / 速度は上限まで / 人避けONで人が近いと前進しない / 指令が途切れたら止まる(watchdog)

実行: python ros2_ws/src/nokosahen_core/test/test_manual_drive_roles_20261106.py
"""
import sys
import time
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import test_manual_person_guard as tmg  # noqa: E402  (rclpyスタブ・本物のTeleopControllerを組み立てる部品。先にimportする)
import test_account_roles_http as t  # noqa: E402  (偽サーバー・http()を流用)

dc, roles = t.dc, t.roles
serve, http = t.serve, t.http

PW = {"TL": "tl-pass", "restricted_account": "sho-pass", "kari-op": t.OP_PASSWORD}

# (名前, パス, 本文)。順番に意味がある: 緊急停止をかける -> 解除 は最後(adminは解除まで通る)
ACTIONS = [
    ("運転(前進)", "/api/teleop", {"forward": 1.0, "turn": 0.0}),
    ("運転(停止)", "/api/teleop", {"forward": 0.0, "turn": 0.0}),
    ("速度設定", "/api/speed_settings", {"manual_linear_mps": 0.3}),
    ("PID", "/api/pid_gains", {}),
    ("探索", "/api/exploration/start", {}),
    ("近づく", "/api/person_interaction", {"enabled": True}),
    ("見回す", "/api/scan_enabled", {"enabled": True}),
    ("緊急停止をかける", "/api/soft_estop", {"engaged": True}),
    ("緊急停止の解除", "/api/soft_estop", {"engaged": False}),
]
OK = 200
# ロールごとの期待(ACTIONSと同じ順)。adminは「403/401ではない」を確認(速度設定などは替え玉に実体が無く200とは限らない)
EXPECT = {
    "operator": (200, 200, 403, 403, 403, 403, 403, 200, 200),  # 2026-11-06: 解除も可
    "restricted": (200, 200, 403, 403, 403, 403, 403, 200, 403),
    "guest": (403,) * 9,
    None: (401,) * 9,
}


def make_stores(perms):
    tmp = t.TMP
    af = tmp / f"acc_md_{id(perms)}_{time.monotonic_ns()}.yaml"
    af.write_text('accounts:\n  - username: "kari-op"\n    role: operator\n'
                  f'    password_hash: "{t.OP_HASH}"\n    enabled: true\n\noperator_permissions: {t.json.dumps(perms)}\n',
                  encoding="utf-8")
    pf = tmp / f"perm_md_{time.monotonic_ns()}.json"
    return lambda: t.acc.AccountStore(accounts_path=str(af), permissions_path=str(pf))


class Env:
    """PC版・スマホ版のハンドラを起動する(teleopは渡されたもの)。"""

    def __init__(self, case, teleop, perms):
        mk = make_stores(perms)
        self.bc = t.FakeBehaviorControl()
        pc = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, teleop, behavior_control=self.bc,
            auth_username="TL", auth_password=PW["TL"], enforce_single_controller=False,
            account_store=mk(), ephemeral=t.eo.EphemeralOverrides(None))
        mb = dc.make_mobile_handler(
            lambda: b"<html></html>", lambda: {}, teleop,
            auth_username="TL", auth_password=PW["TL"], restricted_username="restricted_account",
            restricted_password=PW["restricted_account"], enforce_single_controller=False,
            behavior_control=self.bc, account_store=mk(), ephemeral=t.eo.EphemeralOverrides(None))
        self.pc_srv, pcp = serve(pc)
        self.mb_srv, mbp = serve(mb)
        self.base = {"PC8090": f"http://127.0.0.1:{pcp}", "スマホ8095": f"http://127.0.0.1:{mbp}"}
        case.addCleanup(lambda: [(s.shutdown(), s.server_close()) for s in (self.pc_srv, self.mb_srv)])

    def cookie(self, base, user):
        pw = "" if user == "guest" else PW[user]
        code, d, ck = http("POST", base + "/api/login", {"username": user, "password": pw})
        return code, (ck.split(";")[0] if ck else "")


class RoleTable(unittest.TestCase):
    def run_table(self, user, perms, expect):
        teleop = t.FakeTeleop()
        env = Env(self, teleop, perms)
        out = {}
        for label, base in env.base.items():
            if user == "restricted_account" and label.startswith("PC"):
                self.assertEqual(env.cookie(base, user)[0], 401)  # PC版にはrestricted_accountのログインは無い
                continue
            ck = None
            if user is not None:
                code, ck = env.cookie(base, user)
                self.assertEqual(code, 200, (label, user))
            teleop.soft_estop_engaged = False
            row = []
            for (name, path, body), want in zip(ACTIONS, expect):
                try:
                    code, _d, _ = http("POST", base + path, body, ck)
                except Exception:  # noqa: BLE001  adminの速度設定などは替え玉に実体が無く落ちることがある(=権限の関門は通過した)
                    self.assertEqual(want, "not403", (label, user, name))
                    row.append("通過(替え玉で実行時エラー)")
                    continue
                row.append(code)
                if want == "not403":
                    self.assertNotIn(code, (401, 403), (label, user, name, code))
                else:
                    self.assertEqual(code, want, (label, user, name, code))
            out[label] = row
        print(f"\n[{user}] " + " / ".join(f"{k}: {v}" for k, v in out.items()))
        return teleop

    def test_admin(self):
        tel = self.run_table("TL", [], ("not403",) * 9)
        self.assertTrue(tel.drives)
        self.assertEqual(tel.estops[0], True)  # かける -> 解除の順で通る

    def test_restricted_restricted_account(self):
        tel = self.run_table("restricted_account", [], EXPECT["restricted"])
        # 解除は通らない(かけた状態のまま)
        self.assertTrue(tel.soft_estop_engaged)
        self.assertEqual(set(tel.estops), {True})

    def test_operator_no_grants(self):
        tel = self.run_table("kari-op", [], EXPECT["operator"])
        self.assertFalse(tel.soft_estop_engaged)  # かける -> 解除
        self.assertEqual(tel.estops, [True, False, True, False])  # PC版・スマホ版で各1回

    def test_operator_with_all_grants_still_cannot_do_admin_things(self):
        # 付与できる権限を全部許可しても、速度設定・PID・探索・近づく・見回すは403のまま。運転・緊急停止の解除は可
        self.run_table("kari-op", sorted(roles.GRANTABLE), EXPECT["operator"])

    def test_guest(self):
        tel = self.run_table("guest", [], EXPECT["guest"])
        self.assertEqual(tel.drives, [])
        self.assertEqual(tel.estops, [])

    def test_not_logged_in(self):
        tel = self.run_table(None, [], EXPECT[None])
        self.assertEqual(tel.drives, [])

    def test_operator_whoami_has_manual_drive_and_not_admin_caps(self):
        env = Env(self, t.FakeTeleop(), [])
        for label, base in env.base.items():
            code, ck = env.cookie(base, "kari-op")
            who = http("GET", base + "/api/whoami", None, ck)[1]
            self.assertEqual(who["role"], "operator", label)
            self.assertIn("robot.manual_drive", who["permissions"], label)
            for cap in ("robot.speed", "robot.estop_config", "robot.pid_firmware", "robot.exploration", "behavior.control"):
                self.assertNotIn(cap, who["permissions"], (label, cap))
            code, ck = env.cookie(base, "guest")
            self.assertNotIn("robot.manual_drive", http("GET", base + "/api/whoami", None, ck)[1].get("permissions", []))


class Priority(unittest.TestCase):
    """本物の TeleopController(ROSは替え玉)で、TLの運転中は弱いアカウントの指令が破棄されること。"""

    def setUp(self):
        self.teleop = tmg.make_teleop()
        self.teleop._priority_hold_sec = 0.4
        self.env = Env(self, self.teleop, [])

    def drive(self, label, ck, forward=1.0):
        base = self.env.base[label]
        code, d, _ = http("POST", base + "/api/teleop", {"forward": forward, "turn": 0.0}, ck)
        return code, d

    def login(self, label, user):
        code, ck = self.env.cookie(self.env.base[label], user)
        self.assertEqual(code, 200)
        return ck

    def test_tl_blocks_operator_on_both_pages_and_allows_after_hold(self):
        for label in ("PC8090", "スマホ8095"):
            teleop = tmg.make_teleop()
            teleop._priority_hold_sec = 0.4
            env = Env(self, teleop, [])
            self.env = env
            tl = self.login(label, "TL")
            op = self.login(label, "kari-op")
            self.assertTrue(self.drive(label, tl)[1]['ok'])  # TLの運転
            code, d = self.drive(label, op)
            self.assertEqual(code, 200, label)
            self.assertFalse(d["ok"], label)  # TLの運転中なので破棄
            if label.startswith("スマホ"):
                self.assertEqual(d["reason"], "tl_active")
            # TLが止めて保持時間が過ぎたら、operatorの運転が通る
            time.sleep(0.5)
            code, d = self.drive(label, op)
            self.assertEqual((code, d["ok"]), (200, True), label)
            # operatorが運転中でも、TLが運転すればTLが優先(TLは破棄されない)
            self.assertTrue(self.drive(label, tl)[1]["ok"], label)

    def test_restricted_account_same_priority_as_operator(self):
        label = "スマホ8095"
        tl = self.login(label, "TL")
        sho = self.login(label, "restricted_account")
        op = self.login(label, "kari-op")
        self.assertTrue(self.drive(label, tl)[1]["ok"])
        for ck in (sho, op):
            d = self.drive(label, ck)[1]
            self.assertEqual((d["ok"], d["reason"]), (False, "tl_active"))

    def test_estop_blocks_operator_drive_and_release_denied(self):
        for label in ("PC8090", "スマホ8095"):
            op = self.login(label, "kari-op")
            base = self.env.base[label]
            self.assertEqual(http("POST", base + "/api/soft_estop", {"engaged": True}, op)[0], 200)
            self.teleop.published.clear()
            code, d = self.drive(label, op)
            if label.startswith("PC"):
                self.assertEqual((code, d["error"]), (409, "estop_engaged"))
            else:
                self.assertEqual((code, d["ok"], d["reason"]), (200, False, "estop_engaged"))
            self.assertEqual(self.teleop.published, [])  # 緊急停止中は動かない
            # 2026-11-06: operatorも解除でき、解除後は運転できる
            self.assertEqual(http("POST", base + "/api/soft_estop", {"engaged": False}, op)[0], 200)
            self.assertFalse(self.teleop.soft_estop_engaged)
            self.assertTrue(self.drive(label, op)[1]["ok"])

    def test_operator_speed_is_capped_by_configured_limit(self):
        label = "PC8090"
        op = self.login(label, "kari-op")
        self.teleop.linear_speed = 0.2
        self.teleop.angular_speed = 0.8
        self.assertTrue(self.drive(label, op, forward=100.0)[1]["ok"])  # 入力が1を超えても±1に丸める
        tw = self.teleop.published[-1]
        self.assertAlmostEqual(tw.linear.x, 0.2)
        self.assertLessEqual(abs(tw.linear.x), dc.MANUAL_LINEAR_MAX_MPS)
        # operatorは速度の設定自体を変えられない(403)
        self.assertEqual(http("POST", self.env.base[label] + "/api/speed_settings", {"manual_linear_mps": 1.2}, op)[0], 403)
        self.assertEqual(self.teleop.linear_speed, 0.2)

    def test_person_guard_applies_to_operator(self):
        for label in ("PC8090", "スマホ8095"):
            op = self.login(label, "kari-op")
            self.teleop.set_manual_person_guard(True, persist=False)
            tmg.near(self.teleop, True)
            self.teleop.published.clear()
            self.assertTrue(self.drive(label, op)[1]["ok"])
            self.assertEqual(self.teleop.published[-1].linear.x, 0.0, label)  # 人が近いので前進しない
            self.teleop.set_manual_person_guard(False, persist=False)

    def test_watchdog_stops_when_commands_stop(self):
        label = "PC8090"
        op = self.login(label, "kari-op")
        self.assertTrue(self.drive(label, op)[1]["ok"])
        self.assertGreater(self.teleop.published[-1].linear.x, 0.0)
        self.teleop.last_cmd_time = time.monotonic() - 10.0  # 指令が途切れた
        self.teleop._watchdog_tick()
        self.assertEqual(self.teleop.published[-1].linear.x, 0.0)

    def test_priority_function(self):
        self.assertEqual(dc.teleop_priority_for_role(roles.ROLE_ADMIN), dc.TELEOP_PRIORITY_FULL)
        for r in (roles.ROLE_OPERATOR, roles.ROLE_RESTRICTED):
            self.assertEqual(dc.teleop_priority_for_role(r), dc.TELEOP_PRIORITY_RESTRICTED)
            self.assertLess(dc.teleop_priority_for_role(r), dc.TELEOP_PRIORITY_FULL)


class UiMarkers(unittest.TestCase):
    def test_mobile_drive_section_is_marked_and_other_cards_are_not(self):
        import re
        from nokosahen_core import full_dashboard_node as fdn
        html = fdn.MOBILE_HTML
        m = re.search(r'<section data-cap="robot\.manual_drive">\s*<h2>手動運転</h2>', html)
        self.assertTrue(m, "スマホ版の手動運転カードに data-cap=robot.manual_drive がありません")
        # 速度・近づく・見回すのカードには運転の目印を付けない(operatorは無効のまま)
        self.assertEqual(html.count('data-cap="robot.manual_drive"'), 1)
        for cap in ('id="speedSection"', 'id="interactOnBtn"', 'id="scanOnBtn"'):
            i = html.index(cap)
            self.assertNotIn("robot.manual_drive", html[max(0, i - 120):i], cap)


if __name__ == "__main__":
    unittest.main(verbosity=2)
