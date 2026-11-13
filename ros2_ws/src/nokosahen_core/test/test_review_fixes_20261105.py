"""Opusレビュー(2026-11-04)の「中・低」のうち、管理画面・API・表示・アンケートの修正のテスト(2026-11-05)。rclpy不要。

- FACE_PERSON/FACE_SWEEP は管理画面で赤い停止バッジにしない(POLICY_REASONS=黄色側)
- ソフト緊急停止: 停止と解除の同時呼び出しでも、メモリの状態と保存ファイルが一致する/保存の失敗はログに残す
- プログラム終了の監査ログは parse_request の結果をそのまま記録する
- voice_bridge のHTTPハンドラに読み取りタイムアウトがある
- アンケート: CSV・条件ファイルの原子的書き込み(一意の一時ファイル)
- iPad表示: 回答の保存が遅くても、状態取得が待たされない
実行: python -m pytest -q -p no:cacheprovider test/test_review_fixes_20261105.py
"""
import json
import os
import re
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
REPO = PKG.parents[2]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(PKG))

import test_teleop_seq as tts  # noqa: E402  (rclpyスタブの注入)
import test_system_shutdown as tss  # noqa: E402

dc = tts.dc
NC = PKG / "nokosahen_core"


def _js_block(text, name):
    """`const NAME = { ... };` の中身(最初の `};` まで)を全部返す。"""
    return [m.group(1) for m in re.finditer(r"const " + name + r" = \{(.*?)\n  \};", text, re.S)]


class FaceBadgeTest(unittest.TestCase):
    def test_face_modes_are_not_red_hold_reasons(self):
        for fn in ("full_dashboard_node.py", "stage2_dashboard_node.py"):
            t = (NC / fn).read_text(encoding="utf-8")
            holds, pols = _js_block(t, "HOLD_REASONS"), _js_block(t, "POLICY_REASONS")
            self.assertEqual(len(holds), 2, fn)
            self.assertEqual(len(pols), 2, fn)
            for h in holds:
                self.assertNotIn("FACE_PERSON", h, fn)
                self.assertNotIn("FACE_SWEEP", h, fn)
            for p in pols:
                self.assertIn('FACE_PERSON: "人の方を向いています(その場で旋回)"', p, fn)
                self.assertIn('FACE_SWEEP: "人を探して見回しています(その場で旋回)"', p, fn)


@unittest.skipIf(dc is None, "dashboard_common import失敗")
class SoftEstopConcurrencyTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = str(Path(self.tmp.name) / "estop.json")

    def make(self):
        return dc.TeleopController(tts._Node(), soft_estop_state_file=self.path)

    def test_concurrent_engage_release_keeps_memory_and_file_consistent(self):
        for _ in range(20):
            t = self.make()
            barrier = threading.Barrier(2)
            errs = []

            def run(val):
                try:
                    barrier.wait()
                    for _i in range(30):
                        t.handle_soft_estop(val)
                except Exception as e:  # noqa: BLE001
                    errs.append(e)
            th = [threading.Thread(target=run, args=(v,)) for v in (True, False)]
            [x.start() for x in th]
            [x.join() for x in th]
            self.assertEqual(errs, [])
            saved = json.loads(Path(self.path).read_text(encoding="utf-8"))["engaged"]
            self.assertEqual(saved, t.soft_estop_engaged)
            self.assertEqual([p for p in os.listdir(self.tmp.name) if p != "estop.json"], [], "一時ファイルが残らない")

    def test_handle_soft_estop_takes_drive_lock(self):
        t = self.make()
        done = threading.Event()
        t._drive_lock.acquire()
        try:
            th = threading.Thread(target=lambda: (t.handle_soft_estop(True), done.set()))
            th.start()
            self.assertFalse(done.wait(0.3), "手動運転が _drive_lock を持っている間は待つ")
        finally:
            t._drive_lock.release()
        self.assertTrue(done.wait(2.0))
        th.join()

    def test_save_failure_is_logged_not_swallowed(self):
        node = tts._Node()
        logger = mock.Mock()
        node.get_logger = lambda: logger
        t = dc.TeleopController(node, soft_estop_state_file=self.path)
        with mock.patch.object(dc.os, "replace", side_effect=OSError("disk")):
            t.handle_soft_estop(True)
        self.assertTrue(t.soft_estop_engaged, "保存に失敗してもメモリの停止は有効")
        self.assertTrue(logger.error.called, "失敗はログに残る")
        self.assertEqual(os.listdir(self.tmp.name), [], "一時ファイルを残さない")


@unittest.skipIf(dc is None, "dashboard_common import失敗")
class ShutdownAuditTest(unittest.TestCase):
    def _post(self, body):
        s = tss.HttpTest()
        s.start()
        self.addCleanup(lambda: (s.srv.shutdown(), s.srv.server_close()))
        ck = s.cookie("TL", "tl-pass")
        with self.assertLogs("nokosahen.audit", level="WARNING") as cm:
            code, _ = s.post(body, ck)
        self.assertEqual(code, 200)
        return [m for m in cm.output if "SYSTEM_SHUTDOWN " in m][0]

    def test_audit_logs_parse_request_result(self):
        # APIの既定は include_poweroff=False(明示したときだけ)。include_ocr のみ既定ON
        line = self._post({"confirm": "シャットダウン"})
        self.assertIn("options=include_ocr", line)
        self.assertNotIn("include_poweroff", line)
        line = self._post({"confirm": "シャットダウン", "include_poweroff": True})
        self.assertIn("include_poweroff", line)


class VoiceBridgeTimeoutTest(unittest.TestCase):
    def test_handler_has_read_timeout(self):
        t = (REPO / "voice_bridge" / "voice_bridge.py").read_text(encoding="utf-8")
        m = re.search(r"class Handler\(BaseHTTPRequestHandler\):\s*(?:#[^\n]*\n\s*)*timeout = (\d+)", t)
        self.assertIsNotNone(m)
        self.assertLessEqual(int(m.group(1)), 30)


class SurveyAtomicTest(unittest.TestCase):
    def setUp(self):
        from nokosahen_core import display_survey as dsv
        self.dsv = dsv
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.store = dsv.SurveyStore(self.tmp.name)

    def test_csv_temp_name_is_unique_and_not_left(self):
        names = []
        real = self.dsv.tempfile.mkstemp

        def spy(*a, **k):
            r = real(*a, **k)
            names.append(r[1])
            return r
        with mock.patch.object(self.dsv.tempfile, "mkstemp", side_effect=spy):
            self.store.csv_bytes()
            self.store.csv_bytes()
        self.assertEqual(len(set(names)), 2)
        self.assertFalse(any(os.path.exists(n) for n in names))
        self.assertTrue(os.path.exists(self.store.csv_path))

    def test_write_condition_is_atomic(self):
        self.store.write_condition("停止 + 音声")
        self.assertEqual(self.store.read_condition(), "停止 + 音声")
        # 置き換えに失敗しても、元の内容が壊れず、一時ファイルが残らない
        with mock.patch.object(self.dsv.os, "replace", side_effect=OSError("x")):
            self.store.write_condition("移動のみ")
        self.assertEqual(self.store.read_condition(), "停止 + 音声")
        self.assertEqual([p for p in os.listdir(self.tmp.name) if p.endswith(".tmp")], [])

    def test_received_at_clock_is_documented(self):
        t = (REPO / "docs" / "survey_questions.md").read_text(encoding="utf-8")
        self.assertRegex(t, r"received_at.{0,40}Jetson.{0,40}時計")
        self.assertRegex(t, r"submitted_at.{0,40}iPad.{0,40}時計")


class HubSlowSaveTest(unittest.TestCase):
    def test_public_state_is_not_blocked_by_slow_save(self):
        import test_integration_display as tid_
        from nokosahen_core import display_hub as dh
        from nokosahen_core import display_survey as dsv
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        store = dsv.SurveyStore(tmp.name)
        hub = dh.DisplayHub(mode_file=str(Path(tmp.name) / "mode"), survey_store=store)
        real_save = store.save
        started, release = threading.Event(), threading.Event()

        def slow_save(rec):
            started.set()
            release.wait(5)
            return real_save(rec)
        store.save = slow_save
        payload = tid_.valid_response("resp-slow-0001")
        res = {}
        th = threading.Thread(target=lambda: res.update(r=hub.submit_response(payload)))
        th.start()
        self.assertTrue(started.wait(2))
        t0 = time.monotonic()
        st = hub.public_state()
        self.assertLess(time.monotonic() - t0, 0.5, "保存中でも状態取得は待たされない")
        self.assertTrue(st["ok"])
        release.set()
        th.join(5)
        self.assertEqual(res["r"][0], 200)
        saved = [json.loads(x) for x in Path(store.jsonl).read_text(encoding="utf-8").splitlines()]
        self.assertEqual(len(saved), 1)
        self.assertEqual(saved[0]["response_id"], "resp-slow-0001")
        self.assertEqual(saved[0]["answers"], payload["answers"])


class SwitchApiPermissionTest(unittest.TestCase):
    """PC版・スマホ版のスイッチAPI(person_interaction/scan_enabled/face_enabled/idle_wiggle)の、権限と behavior_control=None の確認。
    入力検証と setter 呼び出しは test_auth_hardening_20261104.py が見ている。ここは、それに無かった3点。"""
    PATHS = ("/api/person_interaction", "/api/scan_enabled", "/api/face_enabled", "/api/idle_wiggle")

    class BC:
        def __init__(self):
            self.calls = []

        def __getattr__(self, name):
            if not name.startswith("set_"):
                raise AttributeError(name)
            return lambda enabled: (self.calls.append((name, enabled)) or (True, "ok"))

    def _serve(self, maker, bc, perms=None):
        import test_account_roles_http as T
        store = T.acc.AccountStore(accounts_path="", permissions_path=str(T.TMP / f"perm_sw_{id(self)}.json"))
        handler = maker(lambda: b"x", lambda: {}, T.FakeTeleop(), auth_username="TL", auth_password="tl-pass",
                        enforce_single_controller=False, behavior_control=bc, account_store=store)
        srv, port = T.serve(handler)
        self.addCleanup(lambda: (srv.shutdown(), srv.server_close()))
        return T, f"http://127.0.0.1:{port}"

    def _login(self, T, url, user, pw):
        code, _d, ck = T.http("POST", url + "/api/login", {"username": user, "password": pw})
        self.assertEqual(code, 200, user)
        return ck.split(";")[0]

    @unittest.skipIf(dc is None, "dashboard_common import失敗")
    def test_guest_cannot_switch_and_nothing_is_called(self):
        for maker in (dc.make_dashboard_handler, dc.make_mobile_handler):
            bc = self.BC()
            T, url = self._serve(maker, bc)
            guest = self._login(T, url, "guest", "")
            for p in self.PATHS:
                self.assertEqual(T.http("POST", url + p, {"enabled": True}, guest)[0], 403, p)
                self.assertEqual(T.http("POST", url + p, {"enabled": True}, None)[0], 401, p)
            self.assertEqual(bc.calls, [])

    @unittest.skipIf(dc is None, "dashboard_common import失敗")
    def test_behavior_control_none_gives_404_for_tl(self):
        for maker in (dc.make_dashboard_handler, dc.make_mobile_handler):
            T, url = self._serve(maker, None)
            tl = self._login(T, url, "TL", "tl-pass")
            for p in self.PATHS:
                self.assertEqual(T.http("POST", url + p, {"enabled": True}, tl)[0], 404, p)

    @unittest.skipIf(dc is None, "dashboard_common import失敗")
    def test_operator_without_permission_gets_403(self):
        import test_account_roles_http as T
        bc = self.BC()
        d = Path(tempfile.mkdtemp(prefix="sw_op_", dir=T.TMP))
        yaml_text = "\n".join([
            "accounts:", '  - username: "kari-op"', "    role: operator",
            f'    password_hash: "{T.OP_HASH}"', "    enabled: true", ""])
        (d / "accounts.yaml").write_text(yaml_text, encoding="utf-8")
        store = T.acc.AccountStore(accounts_path=str(d / "accounts.yaml"), permissions_path=str(d / "perm.json"))
        handler = dc.make_dashboard_handler(
            lambda: b"x", lambda: {}, T.FakeTeleop(), auth_username="TL", auth_password="tl-pass",
            enforce_single_controller=False, behavior_control=bc, account_store=store)
        srv, port = T.serve(handler)
        self.addCleanup(lambda: (srv.shutdown(), srv.server_close()))
        url = f"http://127.0.0.1:{port}"
        op = self._login(T, url, "kari-op", T.OP_PASSWORD)
        for p in self.PATHS:
            if p == "/api/face_enabled":
                continue  # 2026-11-06: 向くスイッチだけは弱いアカウントも既定で切り替えられる(下で確認)
            self.assertEqual(T.http("POST", url + p, {"enabled": True}, op)[0], 403, p)
        self.assertEqual(bc.calls, [])
        self.assertEqual(T.http("POST", url + "/api/face_enabled", {"enabled": True}, op)[0], 200)
        self.assertEqual(bc.calls, [("set_face_enabled", True)])


if __name__ == "__main__":
    unittest.main()
