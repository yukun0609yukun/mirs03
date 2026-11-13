"""手動運転の人避け(manual_person_guard、既定OFF)のテスト(2026-10-24、docs/manual_person_guard.md)。rclpy不要(スタブ)。

- 判定(純粋関数): OFFは従来どおり / ON+人が近い=前進ゼロ・後退は通す・旋回は半分 / ON+近くない=前進する /
  ON+near_person_stopが3秒以上途絶・未受信=前進を許し警告 / ソフト緊急停止中は従来どおり
- 状態ファイル: 保存・復元・壊れたファイルはOFF
- API: guest 403 / 権限なしoperator 403 / 権限ありoperator 200(保存しない) / admin 200(保存する) / 厳密なbool以外は400
- 画面: トグルのidがあり、JSがnode --checkを通る

実行: python3 -I ros2_ws/src/nokosahen_core/test/test_manual_person_guard.py
"""
import json
import shutil
import subprocess
import sys
import tempfile
import time
import types
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(TEST_DIR.parent))

import test_teleop_seq as tts  # noqa: E402  (rclpyスタブの注入と_Nodeを流用)

dc = tts.dc
NODE = shutil.which("node")


def make_teleop(path=None):
    t = dc.TeleopController(tts._Node(), manual_person_guard_state_file=path)
    t.publish_manual_override = lambda *_a, **_k: None
    t._publish_wasd_active = lambda *_a, **_k: None
    t.published = []
    t.cmd_vel_pub = types.SimpleNamespace(publish=lambda m: t.published.append(m))
    return t


def near(t, value, age=0.0):
    """near_person_stopをage秒前に受信した状態にする(age=None=未受信)。"""
    t.near_person_stop_active = bool(value)
    t._near_person_time = None if age is None else time.monotonic() - age


@unittest.skipIf(dc is None, f"dashboard_common import失敗: {tts._IMPORT_ERR}")
class DecisionTest(unittest.TestCase):
    def test_pure_decision(self):
        d = dc.person_guard_decision
        self.assertEqual(d(False, True, 0.1), {"detector_alive": True, "blocked": False, "warning": ""})
        self.assertTrue(d(True, True, 0.1)["blocked"])
        self.assertFalse(d(True, False, 0.1)["blocked"])
        for age in (None, 3.5, 100.0):
            r = d(True, True, age)
            self.assertFalse(r["blocked"], age)
            self.assertFalse(r["detector_alive"], age)
            self.assertEqual(r["warning"], dc.PERSON_GUARD_NO_DETECTOR_MSG)
        # OFFのときは警告も出さない
        self.assertEqual(d(False, True, None)["warning"], "")
        self.assertTrue(d(True, True, 3.0)["detector_alive"])  # ちょうど3秒は生きている扱い

    def test_apply_person_guard(self):
        self.assertEqual(dc.apply_person_guard(1.0, 0.5, False), (1.0, 0.5, False))
        self.assertEqual(dc.apply_person_guard(1.0, 0.5, True), (0.0, 0.25, True))
        self.assertEqual(dc.apply_person_guard(-1.0, 0.0, True), (-1.0, 0.0, False))
        self.assertEqual(dc.apply_person_guard(0.0, -1.0, True), (0.0, -0.5, False))


@unittest.skipIf(dc is None, f"dashboard_common import失敗: {tts._IMPORT_ERR}")
class DriveTest(unittest.TestCase):
    def test_default_is_off_and_unchanged(self):
        t = make_teleop()
        self.assertFalse(t.manual_person_guard)
        near(t, True)  # 人が近くても、OFFなら従来どおり前進する
        self.assertTrue(t.handle_drive(1.0, 0.5))
        self.assertAlmostEqual(t.published[-1].linear.x, t.linear_speed)
        self.assertAlmostEqual(t.published[-1].angular.z, 0.5 * t.angular_speed)
        self.assertFalse(t.last_guard_blocked)

    def test_on_and_person_near_blocks_forward_only(self):
        t = make_teleop()
        t.set_manual_person_guard(True)
        near(t, True)
        self.assertTrue(t.handle_drive(1.0, 0.0))
        self.assertEqual(t.published[-1].linear.x, 0.0)
        self.assertTrue(t.last_guard_blocked)
        # 後退は通る
        self.assertTrue(t.handle_drive(-1.0, 0.0))
        self.assertAlmostEqual(t.published[-1].linear.x, -t.linear_speed)
        self.assertFalse(t.last_guard_blocked)
        # その場旋回は通る(角速度は半分)
        self.assertTrue(t.handle_drive(0.0, 1.0))
        self.assertEqual(t.published[-1].linear.x, 0.0)
        self.assertAlmostEqual(t.published[-1].angular.z, 0.5 * t.angular_speed)
        self.assertFalse(t.last_guard_blocked)
        # 前進+旋回: 前進だけゼロ
        self.assertTrue(t.handle_drive(1.0, -1.0))
        self.assertEqual(t.published[-1].linear.x, 0.0)
        self.assertAlmostEqual(t.published[-1].angular.z, -0.5 * t.angular_speed)
        self.assertTrue(t.last_guard_blocked)
        st = t.status()
        self.assertTrue(st["manual_person_guard"])
        self.assertTrue(st["manual_person_guard_blocked"])

    def test_on_and_person_not_near_moves_normally(self):
        t = make_teleop()
        t.set_manual_person_guard(True)
        near(t, False)
        self.assertTrue(t.handle_drive(1.0, 1.0))
        self.assertAlmostEqual(t.published[-1].linear.x, t.linear_speed)
        self.assertAlmostEqual(t.published[-1].angular.z, t.angular_speed)
        self.assertFalse(t.last_guard_blocked)
        self.assertFalse(t.status()["manual_person_guard_blocked"])

    def test_on_and_detector_silent_allows_forward_with_warning(self):
        for age in (None, 3.5, 60.0):
            t = make_teleop()
            t.set_manual_person_guard(True)
            near(t, True, age=age)  # 最後の値がtrueのままでも、古い/未受信は「不明」
            self.assertTrue(t.handle_drive(1.0, 0.0), age)
            self.assertAlmostEqual(t.published[-1].linear.x, t.linear_speed, msg=str(age))
            self.assertFalse(t.last_guard_blocked)
            st = t.person_guard_state()
            self.assertFalse(st["detector_alive"])
            self.assertEqual(st["warning"], dc.PERSON_GUARD_NO_DETECTOR_MSG)
            self.assertFalse(st["near_person_stop"])

    def test_subscription_callback_updates_time(self):
        t = make_teleop()
        t.set_manual_person_guard(True)
        self.assertIsNone(t._near_person_time)
        t._on_near_person_stop(types.SimpleNamespace(data=True))
        self.assertTrue(t.person_guard_state()["blocked"])
        t._on_near_person_stop(types.SimpleNamespace(data=False))
        self.assertFalse(t.person_guard_state()["blocked"])
        self.assertTrue(t.person_guard_state()["detector_alive"])

    def test_soft_estop_unchanged(self):
        t = make_teleop()
        t.set_manual_person_guard(True)
        near(t, False)
        t.handle_soft_estop(True)
        n = len(t.published)
        self.assertFalse(t.handle_drive(1.0, 0.0))  # 従来どおり、指令を出さずFalse
        self.assertEqual(len(t.published), n)
        near(t, True)
        self.assertFalse(t.handle_drive(-1.0, 0.0))
        self.assertEqual(len(t.published), n)
        self.assertTrue(t.handle_drive(0.0, 0.0))  # アイドルは従来どおりTrue

    def test_toggle_off_restores_old_behaviour(self):
        t = make_teleop()
        t.set_manual_person_guard(True)
        near(t, True)
        t.handle_drive(1.0, 0.0)
        self.assertEqual(t.published[-1].linear.x, 0.0)
        t.set_manual_person_guard(False)
        t.handle_drive(1.0, 0.0)
        self.assertAlmostEqual(t.published[-1].linear.x, t.linear_speed)


@unittest.skipIf(dc is None, f"dashboard_common import失敗: {tts._IMPORT_ERR}")
class PersistTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = str(Path(self.tmp.name) / "mpg.json")

    def test_no_file_means_off(self):
        self.assertFalse(make_teleop(self.path).manual_person_guard)
        self.assertFalse(make_teleop(None).manual_person_guard)

    def test_saved_and_restored(self):
        t = make_teleop(self.path)
        t.set_manual_person_guard(True)
        self.assertIs(json.loads(Path(self.path).read_text(encoding="utf-8"))["enabled"], True)
        self.assertTrue(make_teleop(self.path).manual_person_guard)  # 再起動後も保つ
        t.set_manual_person_guard(False)
        self.assertFalse(make_teleop(self.path).manual_person_guard)

    def test_not_persisted_when_requested(self):
        t = make_teleop(self.path)
        t.set_manual_person_guard(True, persist=False)  # operatorの一時的な変更
        self.assertTrue(t.manual_person_guard)
        self.assertFalse(Path(self.path).exists())
        self.assertFalse(make_teleop(self.path).manual_person_guard)

    def test_broken_file_means_off(self):
        for content in ("", "not json", "[1]", '{"enabled": "yes"}', '{"enabled": 1}', '{"enabled": false}', '{}'):
            Path(self.path).write_text(content, encoding="utf-8")
            self.assertFalse(make_teleop(self.path).manual_person_guard, content)

    def test_unwritable_path_does_not_break_toggle(self):
        t = make_teleop(str(Path(self.tmp.name) / "no_such_dir" / "x.json"))
        t.set_manual_person_guard(True)
        self.assertTrue(t.manual_person_guard)

    def test_no_temp_file_left(self):
        make_teleop(self.path).set_manual_person_guard(True)
        self.assertEqual([p.name for p in Path(self.tmp.name).iterdir()], ["mpg.json"])

    def test_node_wiring(self):
        text = (TEST_DIR.parent / "nokosahen_core" / "full_dashboard_node.py").read_text(encoding="utf-8")
        self.assertIn('declare_parameter("manual_person_guard_state_file", "~/.nokosahen_manual_person_guard.json")', text)
        self.assertIn('manual_person_guard_state_file=str(self.get_parameter("manual_person_guard_state_file").value)', text)


# ---- API(実サーバー。test_account_roles_http の土台を流用し、teleopだけ本物のTeleopControllerにする) ----
try:
    import test_account_roles_http as tarh
except Exception as exc:  # noqa: BLE001
    tarh = None
    _TARH_ERR = exc


@unittest.skipIf(dc is None or tarh is None, "import失敗")
class ApiTest(tarh.Base if tarh is not None else unittest.TestCase):
    def start_guard(self, perms=None):
        self.state_path = str(Path(tarh.TMP) / f"mpg_{self.id()}.json")
        Path(self.state_path).unlink(missing_ok=True)
        real = make_teleop(self.state_path)
        orig = tarh.FakeTeleop
        tarh.FakeTeleop = lambda: real
        self.addCleanup(setattr, tarh, "FakeTeleop", orig)
        self.start(perms=perms)
        self.real = real
        return real

    def test_get_shape_and_guest_can_read(self):
        self.start_guard()
        near(self.real, True)
        code, d = self.call("GET", "/api/manual_person_guard", ck=self.cookie("guest", ""))
        self.assertEqual(code, 200)
        for k in ("enabled", "blocked", "near_person_stop", "detector_alive"):
            self.assertIn(k, d)
            self.assertIsInstance(d[k], bool)
        self.assertEqual((d["enabled"], d["blocked"], d["near_person_stop"], d["detector_alive"]), (False, False, True, True))
        self.assertFalse(d["can_change"])
        code, d = self.call("GET", "/api/manual_person_guard", ck=self.cookie("TL", "tl-pass"))
        self.assertTrue(d["can_change"])

    def test_guest_403_and_no_change(self):
        self.start_guard()
        code, _ = self.call("POST", "/api/manual_person_guard", {"enabled": True}, self.cookie("guest", ""))
        self.assertEqual(code, 403)
        self.assertFalse(self.real.manual_person_guard)

    def test_operator_without_permission_403(self):
        self.start_guard()
        code, d = self.call("POST", "/api/manual_person_guard", {"enabled": True}, self.cookie("kari-op", tarh.OP_PASSWORD))
        self.assertEqual(code, 403)
        self.assertIn("drive.person_guard", d["missing"])
        self.assertFalse(self.real.manual_person_guard)
        code, d = self.call("GET", "/api/manual_person_guard", ck=self.cookie("kari-op", tarh.OP_PASSWORD))
        self.assertFalse(d["can_change"])

    def test_operator_with_permission_200_but_not_persisted(self):
        self.start_guard(perms=["drive.person_guard"])
        ck = self.cookie("kari-op", tarh.OP_PASSWORD)
        code, d = self.call("POST", "/api/manual_person_guard", {"enabled": True}, ck)
        self.assertEqual(code, 200)
        self.assertTrue(d["enabled"])
        self.assertTrue(self.real.manual_person_guard)
        self.assertFalse(Path(self.state_path).exists())  # 一時的(再起動で戻る)
        self.assertTrue(self.call("GET", "/api/manual_person_guard", ck=ck)[1]["can_change"])

    def test_admin_200_and_persisted(self):
        self.start_guard()
        ck = self.cookie("TL", "tl-pass")
        code, d = self.call("POST", "/api/manual_person_guard", {"enabled": True}, ck)
        self.assertEqual(code, 200)
        self.assertTrue(d["enabled"])
        self.assertTrue(json.loads(Path(self.state_path).read_text(encoding="utf-8"))["enabled"])
        code, d = self.call("POST", "/api/manual_person_guard", {"enabled": False}, ck)
        self.assertEqual(code, 200)
        self.assertFalse(d["enabled"])
        self.assertFalse(json.loads(Path(self.state_path).read_text(encoding="utf-8"))["enabled"])

    def test_strict_bool_only(self):
        self.start_guard()
        ck = self.cookie("TL", "tl-pass")
        for body in ({"enabled": "true"}, {"enabled": 1}, {"enabled": 0}, {"enabled": None}, {"enabled": []}, {}, {"on": True}):
            code, _ = self.call("POST", "/api/manual_person_guard", body, ck)
            self.assertEqual(code, 400, body)
            self.assertFalse(self.real.manual_person_guard, body)

    def test_teleop_response_reports_block(self):
        self.start_guard()
        ck = self.cookie("TL", "tl-pass")
        self.real.set_manual_person_guard(True)
        near(self.real, True)
        code, d = self.call("POST", "/api/teleop", {"forward": 1.0, "turn": 0.0}, ck)
        self.assertEqual(code, 200)
        self.assertTrue(d["ok"])
        self.assertTrue(d["manual_person_guard_blocked"])
        self.assertEqual(self.real.published[-1].linear.x, 0.0)
        code, d = self.call("POST", "/api/teleop", {"forward": -1.0, "turn": 0.0}, ck)
        self.assertFalse(d["manual_person_guard_blocked"])
        code, d = self.call("POST", "/api/teleop", {"forward": 0.0, "turn": 0.0}, ck)
        self.assertFalse(d["manual_person_guard_blocked"])


class HtmlTest(unittest.TestCase):
    def test_toggle_ids_in_full_page_and_js_ok(self):
        import test_dashboard_layout as tdl
        html = tdl.pages()["HTML"]
        for i in ("mpgToggle", "mpgBadge", "mpgBlocked", "mpgWarn", "mpgNote", "mpgBlock"):
            self.assertEqual(html.count(f'id="{i}"'), 1, i)
        self.assertIn("手動運転で人が近いとき前進を止める", html)
        self.assertIn('data-cap="drive.person_guard"', html)
        self.assertIn("/api/manual_person_guard", html)
        # 既存の運転カードの要素が残っている
        for i in ("resumeBtn", "wallEscapeBtn", "driveBadge", "speedFold"):
            self.assertIn(f'id="{i}"', html, i)
        self.assertIsNotNone(NODE, "node が必要です")
        _markup, js, _scripts = tdl.split(html)
        with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False, encoding="utf-8") as f:
            f.write(js)
        r = subprocess.run([NODE, "--check", f.name], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=1)
