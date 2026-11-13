"""operator(弱いアカウント)の実サーバー結合テスト(2026-10-13追加、docs/accounts.md)。

本物の make_dashboard_handler / make_mobile_handler を実際にHTTPで起動し、admin(TL)・operator・guest が
各APIを叩いた結果(許可 / 403 / 保存の有無)を確認する。2台目(voice_bridge)とラズパイは偽サーバー、
ロボットには何も送らない(teleopは記録するだけの替え玉)。

- ログイン: 同じ /api/login。operatorは無効(プレースホルダー・enabled:false)の間は入れない、回数制限(429)、
  既存アカウントの動作は不変
- 権限: 既定は何も許可しない(デフォルト拒否)。根幹のAPIは許可リストに入れても403、未分類のAPIも403
- 緊急停止: operatorもかけられる / 解除はadminとoperator(2026-11-06〜。restrictedは不可) / guestは従来どおり書き込み不可
- 一時的な変更: operatorの書き込みはpersist:falseで2台目へ、adminは通常保存。adminが同じ項目を保存すると破棄、
  「全て破棄」で2台目の/ephemeral_resetが呼ばれる。再起動相当(新しいハンドラの起動時リセット)でファイル系も戻る
- 管理画面のAPI: adminのみ、パスワード/ハッシュは返さない、admin_onlyの付与は400

実行: python ros2_ws/src/nokosahen_core/test/test_account_roles_http.py
"""
import json
import os
import sys
import tempfile
import threading
import time
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
    import std_msgs.msg as _std  # noqa: F401
except Exception:  # noqa: BLE001
    _install_stubs()

from nokosahen_core import accounts as acc  # noqa: E402
from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import ephemeral_overrides as eo  # noqa: E402
from nokosahen_core import roles  # noqa: E402
from nokosahen_core.announce_settings import AnnounceSettingsStore  # noqa: E402
from nokosahen_core.bgm_proxy import BgmProxy  # noqa: E402
from nokosahen_core.survey_admin_proxy import SurveyAdminProxy  # noqa: E402
from nokosahen_core.voice_levels_proxy import VoiceLevelsProxy  # noqa: E402
from nokosahen_core.voice_master_proxy import VoiceMasterProxy  # noqa: E402
from nokosahen_core.voice_settings_controller import VoiceSettingsController  # noqa: E402
from nokosahen_core.voice_settings_proxy import VoiceSettingsProxy  # noqa: E402

TMP = Path(tempfile.mkdtemp(prefix="account_roles_http_"))
OP_PASSWORD = "operator-pass-123"
OP_HASH = acc.hash_password(OP_PASSWORD, n=2 ** 10)


def serve(handler):
    srv = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, srv.server_address[1]


class Bridge(BaseHTTPRequestHandler):
    """偽の2台目(voice_bridge)。受け取ったPOSTを記録する。"""
    posts = []
    levels = {"speech_volume": 2.0, "bgm_volume": 1.0, "bgm_duck_volume": 0.15}

    def log_message(self, *a):
        pass

    def _send(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/voice_levels":
            self._send(200, dict(type(self).levels))
        elif self.path == "/voice_master":
            self._send(200, {"ok": True, "enabled": True})
        elif self.path == "/voice_settings":
            self._send(200, {"ok": True, "settings": {}})
        else:
            self._send(404, {})

    def do_POST(self):
        n = int(self.headers.get("Content-Length", "0"))
        data = json.loads(self.rfile.read(n) or b"{}")
        type(self).posts.append((self.path, data))
        if self.path == "/voice_levels":
            self._send(200, dict(type(self).levels))
        elif self.path == "/voice_master":
            self._send(200, {"ok": True, "enabled": bool(data.get("enabled"))})
        elif self.path == "/bgm":
            self._send(200, {"ok": True})
        elif self.path in ("/voice_settings", "/ephemeral_reset"):
            self._send(200, {"ok": True})
        else:
            self._send(404, {})


class Pi(BaseHTTPRequestHandler):
    """偽のラズパイ(kiosk_admin)。表示モードを覚える。"""
    mode = "customer"
    posts = []

    def log_message(self, *a):
        pass

    def _send(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        self._send(200, {"mode": type(self).mode, "count": 0})

    def do_POST(self):
        n = int(self.headers.get("Content-Length", "0"))
        data = json.loads(self.rfile.read(n) or b"{}")
        type(self).posts.append((self.path, data))
        if self.path == "/mode":
            type(self).mode = data["mode"]
        self._send(200, {"mode": type(self).mode, "ok": True})


class FakeBehaviorControl:
    """BehaviorDirectorControlControllerの代わり(スイッチ呼び出しを記録するだけ)。"""

    def __init__(self):
        self.calls = []

    def set_person_interaction_enabled(self, enabled):
        self.calls.append(("person_interaction", enabled))
        return True, "ok"

    def set_scan_enabled(self, enabled):
        self.calls.append(("scan", enabled))
        return True, "ok"

    def set_face_enabled(self, enabled):
        self.calls.append(("face", enabled))
        return True, "ok"

    def set_idle_wiggle_enabled(self, enabled):
        self.calls.append(("idle_wiggle", enabled))
        return True, "ok"


class FakeTeleop:
    soft_estop_engaged = False

    def __init__(self):
        self.drives = []
        self.estops = []
        self.cmd_vel_pub = types.SimpleNamespace(publish=lambda m: None)

    def handle_drive(self, *a, **k):
        self.drives.append((a, k))
        return True

    def handle_soft_estop(self, engaged):
        self.estops.append(engaged)
        self.soft_estop_engaged = engaged

    def status(self):
        return {}


def http(method, url, body=None, cookie=None):
    headers = {"Content-Type": "application/json"}
    if cookie:
        headers["Cookie"] = cookie
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method=method, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            return r.status, json.loads(r.read().decode("utf-8") or "{}"), r.headers.get("Set-Cookie", "")
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read().decode("utf-8") or "{}"), ""
        finally:
            e.close()


class Base(unittest.TestCase):
    ENABLED = True

    @classmethod
    def setUpClass(cls):
        cls.bridge_srv, cls.bridge_port = serve(Bridge)
        cls.pi_srv, cls.pi_port = serve(Pi)
        cls.bridge_url = f"http://127.0.0.1:{cls.bridge_port}"
        cls.pi_url = f"http://127.0.0.1:{cls.pi_port}"

    @classmethod
    def tearDownClass(cls):
        for s in (cls.bridge_srv, cls.pi_srv):
            s.shutdown()
            s.server_close()

    def make_store(self, enabled=True, perms=None):
        accounts_file = TMP / f"accounts_{self.id()}.yaml"
        accounts_file.write_text(
            'accounts:\n  - username: "kari-op"\n    role: operator\n'
            f'    password_hash: "{OP_HASH}"\n    enabled: {"true" if enabled else "false"}\n'
            f'    \noperator_permissions: {json.dumps(perms or [])}\n', encoding="utf-8")
        self.perm_file = TMP / f"perm_{self.id()}.json"
        if self.perm_file.exists():
            self.perm_file.unlink()
        return acc.AccountStore(accounts_path=str(accounts_file), permissions_path=str(self.perm_file))

    def start(self, enabled=True, perms=None, journal=True, startup_reset=False, store=None, eph=None):
        Bridge.posts = []
        Pi.posts = []
        if not startup_reset:
            Pi.mode = "customer"  # 起動時リセットのテストでは、前の起動で残った表示モードをそのまま使う
        self.teleop = FakeTeleop()
        self.store = store or self.make_store(enabled, perms)
        self.journal = TMP / f"journal_{self.id()}.json"
        if journal and not startup_reset and self.journal.exists():
            self.journal.unlink()
        self.eph = eph or eo.EphemeralOverrides(str(self.journal) if journal else None)
        self.ann = AnnounceSettingsStore(str(TMP / f"announce_{self.id()}.json"))
        self.survey = SurveyAdminProxy(self.pi_url)
        self.vs = VoiceSettingsController(self.ann, VoiceSettingsProxy(self.bridge_url))
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, self.teleop,
            bgm=BgmProxy(self.bridge_url), voice_master=VoiceMasterProxy(self.bridge_url),
            voice_levels=VoiceLevelsProxy(self.bridge_url), survey_admin=self.survey,
            voice_settings=self.vs, auth_username="TL", auth_password="tl-pass",
            enforce_single_controller=False, account_store=self.store, ephemeral=self.eph,
            startup_reset=startup_reset)
        self.srv, self.port = serve(handler)
        self.base = f"http://127.0.0.1:{self.port}"
        self.addCleanup(self._stop)

    def _stop(self):
        self.srv.shutdown()
        self.srv.server_close()

    def login(self, user, pw):
        code, d, ck = http("POST", self.base + "/api/login", {"username": user, "password": pw})
        return code, (ck.split(";")[0] if ck else "")

    def cookie(self, user, pw):
        code, ck = self.login(user, pw)
        self.assertEqual(code, 200, user)
        return ck

    def call(self, method, path, body=None, ck=None):
        code, d, _ = http(method, self.base + path, body, ck)
        return code, d


class LoginTest(Base):
    def test_existing_accounts_unchanged(self):
        self.start()
        self.assertEqual(self.login("TL", "tl-pass")[0], 200)
        self.assertEqual(self.login("TL", "wrong")[0], 401)
        self.assertEqual(self.login("guest", "")[0], 200)
        self.assertEqual(self.login("", "")[0], 401)
        # 【2026-11-04変更】管理者(TL)にも失敗回数の制限を掛けた(総当たり対策)。上の"wrong"1回+ここの3回=4回までの失敗なら、その後も入れて、成功でリセットされる
        # (5回失敗するとロックされる: test_auth_hardening_20261104.py)。guestは制限なし。
        for _ in range(3):
            self.assertEqual(self.login("TL", "x")[0], 401)
        self.assertEqual(self.login("TL", "tl-pass")[0], 200)
        for _ in range(8):
            self.assertEqual(self.login("guest", "")[0], 200)

    def test_operator_logs_in_from_same_endpoint(self):
        self.start()
        code, ck = self.login("kari-op", OP_PASSWORD)
        self.assertEqual(code, 200)
        c, who = self.call("GET", "/api/whoami", ck=ck)
        self.assertEqual((who["account"], who["role"]), ("kari-op", "operator"))
        self.assertEqual(set(who["permissions"]), set(roles.SAFE_FOR_ALL | roles.ROLE_DEFAULT))
        c, who = self.call("GET", "/api/whoami", ck=self.cookie("TL", "tl-pass"))
        self.assertEqual(who["role"], "admin")
        self.assertNotIn("permissions", who)
        c, who = self.call("GET", "/api/whoami", ck=self.cookie("guest", ""))
        self.assertEqual(who["role"], "viewer")

    def test_disabled_placeholder_and_empty_password_cannot_login(self):
        self.start(enabled=False)
        self.assertEqual(self.login("kari-op", OP_PASSWORD)[0], 401)
        self.assertEqual(self.login("CHANGE_ME_OPERATOR", "")[0], 401)
        self.assertEqual(self.login("CHANGE_ME_OPERATOR", "CHANGE_ME_OPERATOR")[0], 401)
        self.start(enabled=True)
        self.assertEqual(self.login("kari-op", "")[0], 401)
        self.assertEqual(self.login("kari-op", OP_PASSWORD)[0], 200)

    def test_rate_limit_locks_out_even_correct_password_then_admin_unaffected(self):
        self.start()
        for _ in range(5):
            self.assertEqual(self.login("kari-op", "bad")[0], 401)
        self.assertEqual(self.login("kari-op", OP_PASSWORD)[0], 429)
        self.assertEqual(self.login("TL", "tl-pass")[0], 200)  # 管理者は影響を受けない
        self.assertEqual(self.login("guest", "")[0], 200)

    def test_login_page_mentions_lockout(self):
        self.start()
        req = urllib.request.urlopen(self.base + "/login", timeout=10)
        html = req.read().decode("utf-8")
        self.assertIn("429", html)

    def test_session_role_cannot_be_forged(self):
        self.start()
        ck = self.cookie("kari-op", OP_PASSWORD)
        # クッキーを書き換えても、他人のロールにはなれない / 未知のトークンは401
        code, _ = self.call("POST", "/api/teleop", {"forward": 0.1}, ck="nokosahen_session=forged")
        self.assertEqual(code, 401)
        # ロールを名乗る本文・ヘッダは無視される
        code, d, _ = http("POST", self.base + "/api/speed_settings", {"manual_linear_mps": 0.3, "role": "admin"}, ck)
        self.assertEqual(code, 403)


class OperatorPermissionTest(Base):
    def test_default_deny_everything_blocked_and_nothing_reaches_robot_or_bridge(self):
        self.start()
        ck = self.cookie("kari-op", OP_PASSWORD)
        blocked = [
            ("/api/speed_settings", {"manual_linear_mps": 0.3}),  # 【2026-11-06】/api/teleop(手動運転)は弱いアカウントも可。別テストで確認
            
            ("/api/resume_autonomy", {}), ("/api/goal", {"x": 1, "y": 1}), ("/api/pins/goto", {"id": "a"}),
            ("/api/pins/add", {"name": "x"}), ("/api/global_localization", {}), ("/api/boundary/set", {}),  # 2026-11-07: initial_pose・mapping/loadはoperatorも可
            ("/api/keepout_zones/clear", {}), ("/api/mapping/reset", {}), ("/api/behavior_tree", {}),
            ("/api/behavior_rules/save", {}), ("/api/exploration/start", {}), ("/api/pid_gains", {}),
            ("/api/force_demo", {}), ("/api/crowd_background/calibrate", {}), ("/api/checkout_cam_assign", {}),
            ("/api/voice_levels", {"speech_volume": 2}), ("/api/voice_master", {"enabled": False}),
            ("/api/bgm", {"volume": 1}), ("/api/voice_settings/save", {"announce": {"announce_min_interval_seconds": 30}}),
            ("/api/voice_settings/speak", {"text": "x"}), ("/api/survey/mode", {"mode": "survey"}),
            ("/api/survey/ocr_mock", {"action": "reset"}), ("/api/ocr/profile", {}),
            ("/api/accounts/permissions", {"operator_permissions": ["voice.levels"]}), ("/api/ephemeral/clear", {}),
            ("/api/totally_new_api", {}),  # 緊急停止の解除は2026-11-06からoperatorも可(別テスト)
            ("/api/schedule", {}), ("/api/schedule/preview", {}), ("/api/schedule/run", {}),
        ]
        for path, body in blocked:
            code, d = self.call("POST", path, body, ck)
            self.assertEqual(code, 403, path)
            self.assertEqual(d["error"], "forbidden_capability", path)
            self.assertIn("message", d)
        self.assertEqual(self.teleop.drives, [])
        self.assertEqual(self.teleop.estops, [])
        self.assertEqual(Bridge.posts, [])
        self.assertEqual(Pi.posts, [])
        self.assertFalse(os.path.exists(self.ann.path))
        self.assertFalse(self.perm_file.exists())

    def test_admin_only_message_vs_not_granted_message(self):
        self.start()
        ck = self.cookie("kari-op", OP_PASSWORD)
        code, d = self.call("POST", "/api/speed_settings", {"manual_linear_mps": 0.3}, ck)
        self.assertTrue(d["admin_only"])
        self.assertIn("管理者のみ", d["message"])
        code, d = self.call("POST", "/api/voice_levels", {"speech_volume": 2}, ck)
        self.assertFalse(d["admin_only"])
        self.assertEqual(d["missing"], ["voice.levels"])

    def test_estop_engage_allowed_release_denied(self):
        self.start()
        ck = self.cookie("kari-op", OP_PASSWORD)
        self.assertEqual(self.call("POST", "/api/soft_estop", {"engaged": True}, ck)[0], 200)
        self.assertEqual(self.call("POST", "/api/soft_estop", {}, ck)[0], 200)
        # 【2026-11-06】厳密判定: engaged が false 以外(null・0・""・[]・"false"・無し)は「かける」であって解除ではない
        for bad in (None, 0, "", [], "false"):
            self.assertEqual(self.call("POST", "/api/soft_estop", {"engaged": bad}, ck)[0], 200)
        self.assertTrue(self.teleop.soft_estop_engaged)
        self.assertNotIn(False, self.teleop.estops)
        # operatorも解除できる(2026-11-06。監査ログにaccountが残る)
        with self.assertLogs("nokosahen.audit", level="INFO") as cm:
            self.assertEqual(self.call("POST", "/api/soft_estop", {"engaged": False}, ck)[0], 200)
        self.assertTrue(any("ALLOW account=kari-op role=operator" in m and "/api/soft_estop" in m and "robot.estop_release" in m for m in cm.output))
        self.assertFalse(self.teleop.soft_estop_engaged)
        self.assertEqual(self.teleop.estops[-1], False)
        # adminも解除できる(従来どおり)
        self.call("POST", "/api/soft_estop", {"engaged": True}, ck)
        self.assertEqual(self.call("POST", "/api/soft_estop", {"engaged": False}, self.cookie("TL", "tl-pass"))[0], 200)
        self.assertFalse(self.teleop.soft_estop_engaged)
        # 目的地キャンセル(止める方向)はoperatorも通る(map_goal無しなのでゲートの先で404)
        self.assertNotEqual(self.call("POST", "/api/goal/cancel", {}, ck)[0], 403)

    def test_guest_unchanged_read_only_including_estop(self):
        self.start()
        ck = self.cookie("guest", "")
        for path, body in (("/api/soft_estop", {"engaged": True}), ("/api/teleop", {}), ("/api/voice_levels", {"speech_volume": 2})):
            code, d = self.call("POST", path, body, ck)
            self.assertEqual((code, d["error"]), (403, "guest_read_only"), path)
        self.assertEqual(self.teleop.estops, [])
        self.assertEqual(self.call("GET", "/api/whoami", ck=ck)[0], 200)

    def test_admin_unchanged_everything_passes_gate(self):
        self.start()
        ck = self.cookie("TL", "tl-pass")
        for path, body in (("/api/teleop", {"forward": 0.0, "turn": 0.0}), ("/api/totally_new_api", {})):
            code, d = self.call("POST", path, body, ck)
            self.assertNotEqual(code, 403, path)
        self.assertEqual(len(self.teleop.drives), 1)
        self.assertEqual(self.call("POST", "/api/totally_new_api", {}, ck)[0], 404)  # 従来どおり未知は404

    def test_grant_one_at_a_time_via_admin_api(self):
        self.start()
        op = self.cookie("kari-op", OP_PASSWORD)
        tl = self.cookie("TL", "tl-pass")
        code, d = self.call("POST", "/api/accounts/permissions", {"operator_permissions": ["voice.levels"]}, tl)
        self.assertEqual((code, d["operator_permissions"]), (200, ["voice.levels"]))
        self.assertEqual(self.call("POST", "/api/voice_levels", {"speech_volume": 3}, op)[0], 200)
        self.assertEqual(self.call("POST", "/api/voice_master", {"enabled": False}, op)[0], 403)
        code, who = self.call("GET", "/api/whoami", ck=op)
        self.assertIn("voice.levels", who["permissions"])
        # 外すとすぐ効く
        self.call("POST", "/api/accounts/permissions", {"operator_permissions": []}, tl)
        self.assertEqual(self.call("POST", "/api/voice_levels", {"speech_volume": 3}, op)[0], 403)

    def test_yaml_configured_permission_works_and_admin_only_in_yaml_is_ignored(self):
        self.start(perms=["voice.master", "robot.speed", "accounts.manage"])
        op = self.cookie("kari-op", OP_PASSWORD)
        self.assertEqual(self.call("POST", "/api/voice_master", {"enabled": False}, op)[0], 200)
        self.assertEqual(self.call("POST", "/api/speed_settings", {"manual_linear_mps": 0.3}, op)[0], 403)
        self.assertEqual(self.call("POST", "/api/accounts/permissions", {"operator_permissions": []}, op)[0], 403)
        self.assertEqual(self.teleop.drives, [])

    def test_admin_only_permission_cannot_be_saved_via_api(self):
        self.start()
        tl = self.cookie("TL", "tl-pass")
        for bad in (["robot.speed"], ["voice.levels", "robot.pid_firmware"], ["voice.bgm_onoff"], ["bogus"]):
            code, d = self.call("POST", "/api/accounts/permissions", {"operator_permissions": bad}, tl)
            self.assertEqual((code, d["error"]), (400, "invalid_permissions"), bad)
        self.assertFalse(self.perm_file.exists())
        self.assertEqual(self.call("POST", "/api/accounts/permissions", {"operator_permissions": "voice.levels"}, tl)[0], 400)

    def test_pin_goto_grant_passes_gate_only_for_goto(self):
        self.start(perms=["nav.goto_pin"])
        op = self.cookie("kari-op", OP_PASSWORD)
        code, d = self.call("POST", "/api/pins/goto", {"id": "a"}, op)
        self.assertEqual(code, 404)  # ゲートは通った(pins未接続の404)
        self.assertEqual(self.call("POST", "/api/pins/delete", {"id": "a"}, op)[0], 403)
        self.assertEqual(self.call("POST", "/api/goal", {"x": 0, "y": 0}, op)[0], 403)

    def test_audit_log_records_denied_and_allowed(self):
        self.start(perms=["voice.levels"])
        op = self.cookie("kari-op", OP_PASSWORD)
        with self.assertLogs("nokosahen.audit", level="INFO") as cm:
            self.call("POST", "/api/speed_settings", {"manual_linear_mps": 0.3}, op)
            self.call("POST", "/api/voice_levels", {"speech_volume": 2}, op)
        text = "\n".join(cm.output)
        self.assertIn("DENY account=kari-op role=operator", text)
        self.assertIn("POST /api/speed_settings", text)
        self.assertIn("ALLOW account=kari-op", text)
        self.assertNotIn(OP_PASSWORD, text)
        self.assertNotIn(OP_HASH, text)

    def test_failed_login_is_audited_without_password(self):
        self.start()
        with self.assertLogs("nokosahen.audit", level="WARNING") as cm:
            self.login("kari-op", "my-wrong-password")
        text = "\n".join(cm.output)
        self.assertIn("LOGIN_FAILED account=kari-op", text)
        self.assertNotIn("my-wrong-password", text)


class EphemeralHttpTest(Base):
    def test_operator_levels_are_ephemeral_admin_are_persistent(self):
        self.start(perms=["voice.levels", "voice.master"])
        op = self.cookie("kari-op", OP_PASSWORD)
        tl = self.cookie("TL", "tl-pass")
        self.assertEqual(self.call("POST", "/api/voice_levels", {"speech_volume": 3.5}, op)[0], 200)
        self.assertEqual(Bridge.posts[-1], ("/voice_levels", {"speech_volume": 3.5, "persist": False}))
        self.assertEqual(self.call("POST", "/api/voice_master", {"enabled": False}, op)[0], 200)
        self.assertEqual(Bridge.posts[-1], ("/voice_master", {"enabled": False, "persist": False}))
        self.assertEqual(self.call("POST", "/api/bgm", {"volume": 2.0}, op)[0], 200)
        self.assertEqual(Bridge.posts[-1], ("/bgm", {"volume": 2.0, "persist": False}))
        self.assertEqual(self.eph.count(), 4 - 1)  # speech_volume / master / bgm_volume の3件
        # admin: persistキーなし(従来どおりの保存)
        self.assertEqual(self.call("POST", "/api/voice_levels", {"bgm_volume": 1.5}, tl)[0], 200)
        self.assertEqual(Bridge.posts[-1], ("/voice_levels", {"bgm_volume": 1.5}))
        self.assertNotIn("persist", Bridge.posts[-1][1])
        # adminが同じ項目を保存 -> その項目の一時上書きは破棄
        self.assertEqual(self.eph.count(), 2)  # bgm_volumeの分が破棄された
        self.call("POST", "/api/voice_levels", {"speech_volume": 2.0}, tl)
        self.assertEqual(self.eph.count(), 1)
        self.call("POST", "/api/voice_master", {"enabled": True}, tl)
        self.assertEqual(self.eph.count(), 0)

    def test_failed_forward_is_not_recorded(self):
        self.start(perms=["voice.levels"])
        op = self.cookie("kari-op", OP_PASSWORD)
        self.assertEqual(self.call("POST", "/api/voice_levels", {"speech_volume": 99}, op)[0], 400)
        self.assertEqual(self.eph.count(), 0)
        self.assertEqual(Bridge.posts, [])

    def test_admin_view_lists_overrides_and_clear_all_resets_bridge(self):
        self.start(perms=["voice.levels"])
        op = self.cookie("kari-op", OP_PASSWORD)
        tl = self.cookie("TL", "tl-pass")
        self.call("POST", "/api/voice_levels", {"speech_volume": 3.5}, op)
        code, d = self.call("GET", "/api/accounts", ck=tl)
        self.assertEqual(code, 200)
        self.assertEqual(d["ephemeral"]["count"], 1)
        row = d["ephemeral"]["items"][0]
        self.assertEqual((row["item"], row["by"], row["value"]), ("voice_levels.speech_volume", "kari-op", 3.5))
        self.assertEqual(self.call("GET", "/api/whoami", ck=tl)[1]["ephemeral_count"], 1)
        self.assertEqual(self.call("GET", "/api/whoami", ck=op)[1]["ephemeral_count"], 1)
        Bridge.posts = []
        code, d = self.call("POST", "/api/ephemeral/clear", {}, tl)
        self.assertEqual((code, d["ok"], d["cleared"]), (200, True, 1))
        self.assertIn(("/ephemeral_reset", {}), Bridge.posts)
        self.assertEqual(self.eph.count(), 0)

    def test_announce_interval_journaled_and_restored_on_restart(self):
        self.start(perms=["voice.announce_interval"])
        self.ann.save_interval(60)  # 管理者の保存済みの値
        op = self.cookie("kari-op", OP_PASSWORD)
        body = {"announce": {"announce_min_interval_seconds": 20}}
        code, d = self.call("POST", "/api/voice_settings/save", body, op)
        self.assertEqual(code, 200, d)
        self.assertEqual(self.ann.read()["announce_min_interval_seconds"], 20)  # directorが読むファイルには入る
        self.assertEqual(self.eph.count(), 1)
        journal = json.loads(self.journal.read_text(encoding="utf-8"))
        self.assertEqual(journal["entries"]["announce.min_interval"]["baseline"], 60)
        # 2回目の変更でも最初のbaselineを保つ
        self.call("POST", "/api/voice_settings/save", {"announce": {"announce_min_interval_seconds": 15}}, op)
        journal = json.loads(self.journal.read_text(encoding="utf-8"))
        self.assertEqual(journal["entries"]["announce.min_interval"]["baseline"], 60)
        # 2台目へは何も送っていない(voiceの項目を含まない)
        self.assertEqual([p for p in Bridge.posts if p[0] == "/voice_settings"], [])
        # --- 再起動相当: 同じジャーナルで新しいハンドラを起動 -> 起動時リセットで60秒に戻る ---
        eph2 = eo.EphemeralOverrides(str(self.journal))
        self.assertEqual(eph2.pending_restore_count(), 1)
        self.srv.shutdown()
        self.srv.server_close()
        self.start(perms=["voice.announce_interval"], startup_reset=True, store=self.store, eph=eph2)
        deadline = time.time() + 10
        while time.time() < deadline and eph2.pending_restore_count():
            time.sleep(0.05)
        self.assertEqual(eph2.pending_restore_count(), 0)
        self.assertEqual(AnnounceSettingsStore(self.ann.path).read()["announce_min_interval_seconds"], 60)
        self.assertFalse(self.journal.exists())

    def test_announce_interval_restored_to_param_when_no_file_before(self):
        self.start(perms=["voice.announce_interval"])
        op = self.cookie("kari-op", OP_PASSWORD)
        self.call("POST", "/api/voice_settings/save", {"announce": {"announce_min_interval_seconds": 25}}, op)
        self.assertTrue(os.path.exists(self.ann.path))
        eph2 = eo.EphemeralOverrides(str(self.journal))
        restorers = eo.build_restorers(self.ann, self.survey)
        self.assertEqual(eph2.restore_pending(restorers), (1, 0))
        self.assertFalse(os.path.exists(self.ann.path))  # 元々ファイルが無かった=ROSパラメータ値に戻る

    def test_admin_announce_save_discards_journal_entry(self):
        self.start(perms=["voice.announce_interval"])
        op = self.cookie("kari-op", OP_PASSWORD)
        tl = self.cookie("TL", "tl-pass")
        self.call("POST", "/api/voice_settings/save", {"announce": {"announce_min_interval_seconds": 25}}, op)
        self.assertEqual(self.eph.count(), 1)
        self.call("POST", "/api/voice_settings/save", {"announce": {"announce_min_interval_seconds": 90}}, tl)
        self.assertEqual(self.eph.count(), 0)
        self.assertFalse(self.journal.exists())  # 管理者の値(90)が正。再起動しても戻さない
        self.assertEqual(self.ann.read()["announce_min_interval_seconds"], 90)

    def test_voice_text_sent_to_bridge_without_persist(self):
        self.start(perms=["voice.announce_text"])
        op = self.cookie("kari-op", OP_PASSWORD)
        code, d = self.call("POST", "/api/voice_settings/save", {"voice": {"temperature": 0.5}}, op)
        self.assertEqual(code, 200, d)
        self.assertEqual(Bridge.posts[-1], ("/voice_settings", {"temperature": 0.5, "persist": False}))
        self.assertEqual(self.eph.items()[0]["item"], "voice_settings.temperature")
        # 発話間隔の権限が無いので、announce付きは丸ごと403(何も書かれない)
        code, d = self.call("POST", "/api/voice_settings/save",
                            {"announce": {"announce_min_interval_seconds": 30}, "voice": {"temperature": 0.6}}, op)
        self.assertEqual(code, 403)
        self.assertFalse(os.path.exists(self.ann.path))

    def test_survey_mode_journaled_and_restored(self):
        self.start(perms=["display.survey_mode"])
        op = self.cookie("kari-op", OP_PASSWORD)
        code, d = self.call("POST", "/api/survey/mode", {"mode": "survey"}, op)
        self.assertEqual(code, 200, d)
        self.assertEqual(Pi.mode, "survey")
        journal = json.loads(self.journal.read_text(encoding="utf-8"))
        self.assertEqual(journal["entries"]["display.survey_mode"]["baseline"], "customer")
        eph2 = eo.EphemeralOverrides(str(self.journal))
        self.srv.shutdown()
        self.srv.server_close()
        mode_before = Pi.mode
        self.assertEqual(mode_before, "survey")
        self.start(perms=["display.survey_mode"], startup_reset=True, store=self.store, eph=eph2)
        deadline = time.time() + 10
        while time.time() < deadline and (eph2.pending_restore_count() or Pi.mode != "customer"):
            time.sleep(0.05)
        self.assertEqual(Pi.mode, "customer")

    def test_admin_survey_switch_is_not_journaled(self):
        self.start(perms=["display.survey_mode"])
        tl = self.cookie("TL", "tl-pass")
        self.assertEqual(self.call("POST", "/api/survey/mode", {"mode": "survey"}, tl)[0], 200)
        self.assertFalse(self.journal.exists())
        self.assertEqual(self.eph.count(), 0)


class AccountsApiTest(Base):
    def test_accounts_api_admin_only_and_no_secrets(self):
        self.start(perms=["voice.levels"])
        tl = self.cookie("TL", "tl-pass")
        op = self.cookie("kari-op", OP_PASSWORD)
        gu = self.cookie("guest", "")
        self.assertEqual(self.call("GET", "/api/accounts", ck=op)[0], 403)
        self.assertEqual(self.call("GET", "/api/accounts", ck=gu)[0], 403)
        self.assertEqual(self.call("GET", "/api/accounts")[0], 401)
        code, d = self.call("GET", "/api/accounts", ck=tl)
        self.assertEqual(code, 200)
        names = [r["username"] for r in d["accounts"]]
        self.assertEqual(names[:2], ["TL", "guest"])
        self.assertIn("kari-op", names)
        raw = json.dumps(d, ensure_ascii=False)
        self.assertNotIn(OP_HASH, raw)
        self.assertNotIn(OP_PASSWORD, raw)
        self.assertNotIn("tl-pass", raw)
        self.assertNotIn("password_hash", raw)
        caps = {c["name"]: c for c in d["capabilities"]}
        self.assertEqual(caps["robot.speed"]["tier"], "admin_only")
        self.assertEqual(caps["voice.levels"]["tier"], "grantable")
        self.assertEqual(d["operator_permissions"], ["voice.levels"])
        # 最後のログインが記録される
        row = [r for r in d["accounts"] if r["username"] == "kari-op"][0]
        self.assertTrue(row["last_login"])
        # 拒否の履歴
        self.call("POST", "/api/speed_settings", {"manual_linear_mps": 0.3}, op)
        d = self.call("GET", "/api/accounts", ck=tl)[1]
        self.assertEqual(d["recent_denied"][-1]["path"], "/api/speed_settings")

    def test_placeholder_listed_as_disabled(self):
        store = acc.AccountStore(accounts_path="", permissions_path=str(TMP / "perm_placeholder.json"))
        self.start(store=store)
        d = self.call("GET", "/api/accounts", ck=self.cookie("TL", "tl-pass"))[1]
        row = [r for r in d["accounts"] if r["username"] == "CHANGE_ME_OPERATOR"][0]
        self.assertFalse(row["enabled"])
        self.assertTrue(row["placeholder"])


class MobileHandlerTest(Base):
    def test_mobile_operator_login_and_gate(self):
        teleop = FakeTeleop()
        store = self.make_store(perms=["voice.levels"])
        Bridge.posts = []
        eph = eo.EphemeralOverrides()
        bc = FakeBehaviorControl()
        handler = dc.make_mobile_handler(
            lambda: b"<html></html>", lambda: {}, teleop, auth_username="TL", auth_password="tl-pass",
            enforce_single_controller=False, bgm=BgmProxy(self.bridge_url), account_store=store, ephemeral=eph,
            behavior_control=bc)
        srv, port = serve(handler)
        self.addCleanup(lambda: (srv.shutdown(), srv.server_close()))
        base = f"http://127.0.0.1:{port}"
        code, d, ck = http("POST", base + "/api/login", {"username": "kari-op", "password": OP_PASSWORD})
        self.assertEqual(code, 200)
        ck = ck.split(";")[0]
        self.assertEqual(http("GET", base + "/api/whoami", cookie=ck)[1]["role"], "operator")
        # 【2026-11-06】手動運転は弱いアカウントも可(robot.manual_drive はロール既定)。速度設定は403のまま
        self.assertEqual(http("POST", base + "/api/teleop", {"forward": 0.1}, ck)[0], 200)
        self.assertEqual(len(teleop.drives), 1)
        self.assertEqual(http("POST", base + "/api/speed_settings", {"manual_linear_mps": 0.3}, ck)[0], 403)
        self.assertEqual(http("POST", base + "/api/idle_wiggle", {"enabled": False}, ck)[0], 403)
        self.assertEqual(http("POST", base + "/api/person_interaction", {"enabled": True}, ck)[0], 403)  # 人の方へ動くスイッチも、権限がなければ403
        # 【2026-11-03】見回す・人の方を向くスイッチも、権限がなければ403(スイッチは呼ばれない)
        self.assertEqual(http("POST", base + "/api/scan_enabled", {"enabled": True}, ck)[0], 403)
        self.assertEqual(bc.calls, [])
        # 【2026-11-06】「人の方を向く」だけは弱いアカウントも既定で切り替えられる(behavior.face_switch)。他のスイッチは上のとおり403のまま
        self.assertEqual(http("POST", base + "/api/face_enabled", {"enabled": True}, ck)[0], 200)
        self.assertEqual(bc.calls, [("face", True)])
        bc.calls.clear()
        self.assertEqual(http("POST", base + "/api/soft_estop", {"engaged": True}, ck)[0], 200)
        self.assertEqual(http("POST", base + "/api/soft_estop", {"engaged": False}, ck)[0], 200)  # 2026-11-06: operatorも解除可
        self.assertEqual(http("POST", base + "/api/bgm", {"volume": 1.0}, ck)[0], 200)
        self.assertEqual(Bridge.posts[-1], ("/bgm", {"volume": 1.0, "persist": False}))
        self.assertEqual(http("POST", base + "/api/bgm", {"enabled": False}, ck)[0], 403)  # BGM ON/OFFは付与不可
        # 管理者は従来どおり
        code, d, tl = http("POST", base + "/api/login", {"username": "TL", "password": "tl-pass"})
        tl = tl.split(";")[0]
        self.assertEqual(http("POST", base + "/api/teleop", {"forward": 0.0, "turn": 0.0}, tl)[0], 200)
        self.assertEqual(len(teleop.drives), 2)
        # 見回す・人の方を向く: 管理者は200、真偽以外は400
        self.assertEqual(http("POST", base + "/api/scan_enabled", {"enabled": True}, tl)[0], 200)
        self.assertEqual(http("POST", base + "/api/face_enabled", {"enabled": False}, tl)[0], 200)
        self.assertEqual(http("POST", base + "/api/scan_enabled", {"enabled": "false"}, tl)[0], 400)
        self.assertEqual(http("POST", base + "/api/face_enabled", {"enabled": 1}, tl)[0], 400)
        self.assertEqual(bc.calls, [("scan", True), ("face", False)])

    def test_mobile_operator_rate_limit(self):
        store = self.make_store()
        handler = dc.make_mobile_handler(
            lambda: b"x", lambda: {}, FakeTeleop(), auth_username="TL", auth_password="tl-pass",
            enforce_single_controller=False, account_store=store)
        srv, port = serve(handler)
        self.addCleanup(lambda: (srv.shutdown(), srv.server_close()))
        base = f"http://127.0.0.1:{port}"
        for _ in range(5):
            self.assertEqual(http("POST", base + "/api/login", {"username": "kari-op", "password": "bad"})[0], 401)
        self.assertEqual(http("POST", base + "/api/login", {"username": "kari-op", "password": OP_PASSWORD})[0], 429)


class NoAuthCompatTest(unittest.TestCase):
    """認証無効(開発用・auth_username空)の従来動作が変わらないこと。"""

    def test_auth_disabled_still_works(self):
        teleop = FakeTeleop()
        store = acc.AccountStore(accounts_path="", permissions_path=str(TMP / "perm_noauth.json"))
        handler = dc.make_dashboard_handler(lambda: b"x", lambda: {}, teleop, account_store=store,
                                            enforce_single_controller=False)
        srv, port = serve(handler)
        try:
            base = f"http://127.0.0.1:{port}"
            code, d, _ = http("POST", base + "/api/teleop", {"forward": 0.0, "turn": 0.0})
            self.assertEqual(code, 200)
            self.assertEqual(http("GET", base + "/api/whoami")[1]["role"], "admin")
        finally:
            srv.shutdown()
            srv.server_close()


if __name__ == "__main__":
    unittest.main(verbosity=1)
