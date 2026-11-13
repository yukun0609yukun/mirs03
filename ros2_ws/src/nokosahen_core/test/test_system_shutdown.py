"""管理画面の「プログラム終了(全停止)」のテスト(2026-10-13追加、docs/system_shutdown.md)。

**実際にはプロセスを止めない**: 停止スクリプトの起動は「引数を記録するだけの偽の実行器」に差し替える。ssh・ROS・配備なし。
(a) APIの権限(admin 200 / operator・guest 403 / 未ログイン 401)・確認文字列の完全一致・入力検証(型・許可キー以外は400)
(b) 事前チェック(地図未保存で409・discard_mapで通る・二重実行409・スクリプト不在・機能無効)。拒否のときは何も止めない
(c) 実行の順序(モーター停止 -> スクリプト起動)、引数は固定のリスト(シェル展開なし)、ユーザー入力が混ざらない、
    start_new_session で切り離される、スクリプトの中の停止順序(監視ループ -> ROS -> OCR -> 音声 -> 管理画面が最後)
(d) roles.API_RULES でadmin専用に分類されている
(e) 画面(Playwright/Edge): 確認文字列を入れるまで実行ボタンが無効 / 実行後のカウントダウンと通信停止 / operatorでは無効 / スクリーンショット1枚
(f) scripts/shutdown_all.sh の構文(bash -n)と、pkill -f / pgrep -f のパターン割り(自己巻き込み防止)の静的確認

実行: python ros2_ws/src/nokosahen_core/test/test_system_shutdown.py (普通のpythonで。-I は付けない)
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
REPO = PKG.parents[2]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(PKG))

try:
    from playwright.sync_api import sync_playwright
    HAVE_PW = True
except Exception:  # noqa: BLE001
    HAVE_PW = False

import test_account_roles_http as H  # noqa: E402  (偽サーバー・ログイン用の部品)
import test_dashboard_field_sync as FS  # noqa: E402  (rclpy無しでHTMLを組み立てる部品)

from nokosahen_core import accounts as acc  # noqa: E402
from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import roles  # noqa: E402
from nokosahen_core import system_shutdown as ss  # noqa: E402
from nokosahen_core import system_shutdown_ui as ssu  # noqa: E402

SCRIPT = REPO / "scripts" / "shutdown_all.sh"
SHOT = os.environ.get("SCREENSHOT") or os.path.join(tempfile.gettempdir(), "system_shutdown_card.png")
TMP = Path(tempfile.mkdtemp(prefix="system_shutdown_test_"))
OK = {"confirm": "シャットダウン"}


class Recorder:
    """偽の実行器。起動された引数と、モーター停止との順序を記録する(何も実行しない)。"""

    def __init__(self):
        self.events = []
        self.launched = []

    def stop_motors(self):
        self.events.append("motors")

    def launcher(self, argv):
        self.events.append("launch")
        self.launched.append(list(argv))


def make_controller(rec, script=None, enabled=True, map_state=None, nav_active=None, delay=3):
    if script is None:
        script = TMP / "fake_shutdown_all.sh"
        script.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
    return ss.SystemShutdownController(
        script_path=str(script), enabled=enabled, delay_sec=delay, log_path=str(TMP / "shutdown.log"),
        stop_motors=rec.stop_motors, map_state=map_state, nav_active=nav_active, launcher=rec.launcher)


# ---------------------------------------------------------------------------
class ParseAndPrecheckTest(unittest.TestCase):
    def test_confirm_must_match_exactly(self):
        for bad in ("", "シャットダウン ", " シャットダウン", "シャツトダウン", "shutdown", "ｼｬｯﾄﾀﾞｳﾝ", "シャットダウン\n"):
            rec = Recorder()
            code, body = make_controller(rec).request("TL", "1.2.3.4", {"confirm": bad})
            self.assertEqual(code, 400, repr(bad))
            self.assertEqual(rec.events, [], "拒否のときは何も止めない")
        self.assertEqual(ss.CONFIRM_TEXT, ssu.CONFIRM_TEXT)
        self.assertIn(ss.CONFIRM_TEXT, ssu.SHUTDOWN_CARD_JS)

    def test_validation_types_and_keys(self):
        cases = [
            None, [], "x", {}, {"confirm": 1}, {"confirm": None},
            {**OK, "include_voice": "true"}, {**OK, "include_ocr": 1}, {**OK, "discard_map": None},
            {**OK, "cmd": "rm -rf /"}, {**OK, "extra": True}, {**OK, "include_ocr": [True]},
        ]
        for data in cases:
            rec = Recorder()
            code, body = make_controller(rec).request("TL", "ip", data)
            self.assertEqual(code, 400, repr(data))
            self.assertFalse(body["ok"])
            self.assertEqual(rec.events, [])

    def test_defaults(self):
        rec = Recorder()
        code, body = make_controller(rec).request("TL", "ip", OK)
        self.assertEqual(code, 200)
        # 既定: OCRは止める、音声は止めない。OSの停止(電源断)は、明示したときだけ(2026-10-21: 項目を省いた呼び出しで電源が落ちない)
        self.assertNotIn("--no-ocr", rec.launched[0])
        self.assertNotIn("--voice", rec.launched[0])
        self.assertNotIn("--poweroff", rec.launched[0])
        self.assertEqual(body["will_stop_in_sec"], 3)
        self.assertTrue(body["ok"])
        self.assertTrue(body["steps"][0].startswith("モーター停止"))
        self.assertTrue(body["steps"][-1].startswith("管理画面"))

    def test_explicit_poweroff_true_adds_the_flag_and_step(self):
        rec = Recorder()
        code, body = make_controller(rec).request("TL", "ip", {**OK, "include_poweroff": True})
        self.assertEqual(code, 200)
        self.assertIn("--poweroff", rec.launched[0])
        self.assertTrue(body["steps"][-2].startswith("管理画面"))
        self.assertIn("電源を落とす", body["steps"][-1])

    def test_poweroff_off_keeps_os_running(self):
        rec = Recorder()
        code, body = make_controller(rec).request("TL", "ip", {**OK, "include_poweroff": False})
        self.assertEqual(code, 200)
        self.assertNotIn("--poweroff", rec.launched[0])
        self.assertTrue(body["steps"][-1].startswith("管理画面"))

    def test_poweroff_must_be_bool(self):
        for bad in ("true", 1, None):
            rec = Recorder()
            code, body = make_controller(rec).request("TL", "ip", {**OK, "include_poweroff": bad})
            self.assertEqual(code, 400, bad)
            self.assertEqual(rec.events, [], "不正な入力では何も止めない")

    def test_map_unsaved_rejected_then_discard_passes(self):
        st = {"mode": "mapping", "unsaved": True, "reason": "地図が未保存です。先に保存してください"}
        rec = Recorder()
        c = make_controller(rec, map_state=lambda: st)
        code, body = c.request("TL", "ip", OK)
        self.assertEqual(code, 409)
        self.assertEqual(body["error"], "map_unsaved")
        self.assertIn("先に保存してください", body["message"])
        self.assertEqual(rec.events, [], "未保存で拒否したときはモーターも止めない・起動もしない")
        code, body = c.request("TL", "ip", {**OK, "discard_map": True})
        self.assertEqual(code, 200)
        self.assertEqual(rec.events, ["motors", "launch"])

    def test_double_execution_rejected(self):
        rec = Recorder()
        c = make_controller(rec)
        self.assertEqual(c.request("TL", "ip", OK)[0], 200)
        code, body = c.request("TL", "ip", OK)
        self.assertEqual(code, 409)
        self.assertEqual(body["error"], "already_in_progress")
        self.assertEqual(rec.events, ["motors", "launch"])  # 2回目は何もしない
        self.assertTrue(c.state()["in_progress"])

    def test_script_missing_and_disabled(self):
        rec = Recorder()
        code, body = make_controller(rec, script=TMP / "no_such_dir" / "x.sh").request("TL", "ip", OK)
        self.assertEqual(code, 503)
        self.assertIn("停止スクリプトが見つかりません", body["message"])
        self.assertEqual(rec.events, [])
        code, body = make_controller(rec, enabled=False).request("TL", "ip", OK)
        self.assertEqual(code, 403)
        self.assertEqual(body["error"], "disabled")
        self.assertEqual(rec.events, [])

    def test_motor_stop_failure_aborts_everything(self):
        rec = Recorder()

        def boom():
            raise RuntimeError("publish失敗")
        c = make_controller(rec)
        c._stop_motors = boom
        code, body = c.request("TL", "ip", OK)
        self.assertEqual(code, 500)
        self.assertEqual(body["error"], "motor_stop_failed")
        self.assertEqual(rec.launched, [], "モーターを止められなければ、他のプログラムは止めない")
        self.assertFalse(c.state()["in_progress"])

    def test_nav_active_is_allowed_but_reported(self):
        rec = Recorder()
        c = make_controller(rec, nav_active=lambda: True)
        self.assertTrue(c.state()["nav_active"])
        code, body = c.request("TL", "ip", OK)
        self.assertEqual(code, 200)
        self.assertTrue(body["nav_was_active"])

    def test_map_state_builder(self):
        class LC:
            def __init__(self, mode, started):
                self.mode, self.started = mode, started

            def status(self):
                return {"mode": self.mode, "started_at": self.started}

        class MP:
            def __init__(self, last=None, busy=False):
                self.last, self.busy = last, busy

            def status(self):
                return {"busy": self.busy, "last_result": self.last or {}}
        self.assertFalse(ss.build_map_state(LC("localization", 0), MP())()["unsaved"])
        self.assertFalse(ss.build_map_state(LC("idle", 0), MP())()["unsaved"])
        self.assertTrue(ss.build_map_state(LC("mapping", 100), MP())()["unsaved"])
        self.assertTrue(ss.build_map_state(LC("mapping", 100), MP({"ok": True, "time": 50}))()["unsaved"])  # 開始前の保存
        self.assertTrue(ss.build_map_state(LC("mapping", 100), MP({"ok": False, "time": 200}))()["unsaved"])  # 失敗
        self.assertFalse(ss.build_map_state(LC("mapping", 100), MP({"ok": True, "time": 200}))()["unsaved"])
        busy = ss.build_map_state(LC("mapping", 100), MP({"ok": True, "time": 200}, busy=True))()
        self.assertTrue(busy["unsaved"])
        self.assertIn("保存処理中", busy["reason"])

    def test_external_mapping_process_counts_as_unsaved(self):
        """【2026-10-21】管理画面だけが再起動されて状態がidleに戻っても、外で地図作成が動き続けていれば、未保存として扱う。"""
        class LC:
            def __init__(self, mode, external):
                self.mode, self._ext = mode, external

            def status(self):
                return {"mode": self.mode, "started_at": 0}

            def external_mapping_running(self):
                if isinstance(self._ext, Exception):
                    raise self._ext
                return self._ext

        class MP:
            def status(self):
                return {"busy": False, "last_result": {}}
        for mode in ("idle", "localization"):
            st = ss.build_map_state(LC(mode, True), MP())()
            self.assertTrue(st["unsaved"], mode)
            self.assertIn("管理画面の外", st["reason"])
            self.assertFalse(ss.build_map_state(LC(mode, False), MP())()["unsaved"], mode)
        # 確認に失敗した(例外)ときは、従来どおり「動いていない」扱い(終了を妨げない)
        self.assertFalse(ss.build_map_state(LC("idle", RuntimeError("pgrep failed")), MP())()["unsaved"])
        # 外部確認のメソッドが無い古い構成でも、従来どおり
        class Old:
            def status(self):
                return {"mode": "idle"}
        self.assertFalse(ss.build_map_state(Old(), MP())()["unsaved"])

    def test_map_lifecycle_external_check_uses_pgrep_and_ignores_own_process(self):
        import types
        from unittest import mock
        c = object.__new__(dc.MapLifecycleController)
        c._proc = None
        c.remote_ssh_host = ""
        calls = []

        def fake_run(cmd, **kw):
            calls.append(cmd)
            return types.SimpleNamespace(stdout="1234\n", returncode=0)
        with mock.patch.object(subprocess, "run", fake_run):
            self.assertTrue(c.external_mapping_running())
        self.assertEqual(calls[0][:2], ["pgrep", "-f"])
        self.assertEqual(calls[0][2], "mapping_mode[.]launch[.]py")  # 文字を割って、pgrep自身に一致させない
        with mock.patch.object(subprocess, "run", lambda *a, **k: types.SimpleNamespace(stdout="", returncode=1)):
            self.assertFalse(c.external_mapping_running())
        with mock.patch.object(subprocess, "run", mock.Mock(side_effect=OSError("no pgrep"))):
            self.assertFalse(c.external_mapping_running())
        # このコントローラが起動した(把握している)プロセスが生きている間は、「外部」ではない
        c._proc = types.SimpleNamespace(poll=lambda: None)
        with mock.patch.object(subprocess, "run", fake_run):
            self.assertFalse(c.external_mapping_running())


class ExecutionTest(unittest.TestCase):
    def test_motor_stop_before_launch_and_estop_first(self):
        events = []

        class Teleop:
            def handle_soft_estop(self, engaged):
                events.append(("estop", engaged))

        class Goal:
            def cancel(self):
                events.append("cancel_goal")

        class Explorer:
            def set_enabled(self, v):
                events.append(("explore", v))
        rec = Recorder()
        rec.events = events
        c = ss.SystemShutdownController(
            script_path=str(TMP / "fake_shutdown_all.sh"), log_path=str(TMP / "shutdown.log"),
            stop_motors=ss.build_motor_stopper(Teleop(), Goal(), Explorer(), repeats=3, interval_sec=0),
            launcher=rec.launcher)
        (TMP / "fake_shutdown_all.sh").write_text("exit 0\n", encoding="utf-8")
        self.assertEqual(c.request("TL", "ip", OK)[0], 200)
        self.assertEqual(events[:3], [("estop", True)] * 3, "ソフト緊急停止が最初。他の何よりも先")
        self.assertIn("cancel_goal", events)
        self.assertIn(("explore", False), events)
        self.assertEqual(events[-1], "launch", "スクリプト起動(=他のプログラムを止める処理)は、モーター停止の後")
        self.assertLess(max(events.index("cancel_goal"), events.index(("explore", False))), events.index("launch"))

    def test_argv_is_fixed_list_without_shell_and_no_user_input(self):
        script = str(TMP / "fake_shutdown_all.sh")
        (TMP / "fake_shutdown_all.sh").write_text("exit 0\n", encoding="utf-8")
        expect = {
            (True, False, False): ["bash", script, "--delay", "3"],
            (False, False, False): ["bash", script, "--delay", "3", "--no-ocr"],
            (True, True, False): ["bash", script, "--delay", "3", "--voice"],
            (False, True, False): ["bash", script, "--delay", "3", "--no-ocr", "--voice"],
            (True, False, True): ["bash", script, "--delay", "3", "--poweroff"],
            (False, True, True): ["bash", script, "--delay", "3", "--no-ocr", "--voice", "--poweroff"],
        }
        for (ocr, voice, power), argv in expect.items():
            rec = Recorder()
            c = make_controller(rec, script=script)
            code, _ = c.request("名前; rm -rf /", "$(id)", {"confirm": ss.CONFIRM_TEXT, "include_ocr": ocr, "include_voice": voice,
                                                              "include_poweroff": power})
            self.assertEqual(code, 200)
            self.assertEqual(rec.launched, [argv])
            self.assertIsInstance(rec.launched[0], list)
            self.assertNotIn("名前", " ".join(rec.launched[0]))
            self.assertNotIn("$(id)", " ".join(rec.launched[0]))

    def test_default_launcher_is_detached_without_shell(self):
        calls = []
        real = ss.subprocess.Popen

        class FakePopen:
            def __init__(self, argv, **kw):
                calls.append((argv, kw))
        ss.subprocess.Popen = FakePopen
        try:
            script = TMP / "fake_shutdown_all.sh"
            script.write_text("exit 0\n", encoding="utf-8")
            c = ss.SystemShutdownController(script_path=str(script), log_path=str(TMP / "shutdown.log"),
                                            stop_motors=lambda: None)
            self.assertEqual(c.request("TL", "ip", OK)[0], 200)
        finally:
            ss.subprocess.Popen = real
        self.assertEqual(len(calls), 1)
        argv, kw = calls[0]
        self.assertIsInstance(argv, list)
        self.assertIs(kw.get("start_new_session"), True, "親(管理画面)が死んでも続くよう、別セッションで起動")
        self.assertIs(kw.get("shell"), False)
        self.assertEqual(kw.get("stdin"), subprocess.DEVNULL)
        self.assertEqual(argv[0], "bash")

    def test_log_records_who_and_options(self):
        rec = Recorder()
        c = make_controller(rec)
        log = TMP / "shutdown.log"
        if log.exists():
            log.unlink()
        c.request("TL", "10.0.0.5", {**OK, "include_voice": True})
        text = log.read_text(encoding="utf-8")
        self.assertIn("account=TL", text)
        self.assertIn("ip=10.0.0.5", text)
        self.assertIn("include_voice=True", text)
        self.assertIn("LAUNCHED", text)


class ScriptStaticTest(unittest.TestCase):
    src = SCRIPT.read_text(encoding="utf-8")

    def test_file_is_lf(self):
        self.assertNotIn(b"\r", SCRIPT.read_bytes())

    def test_bash_syntax(self):
        bash = shutil.which("bash")
        if not bash:
            self.skipTest("bash がありません")
        for f in (SCRIPT, REPO / "scripts" / "stop_robot.sh"):
            r = subprocess.run([bash, "-n", str(f)], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, f"{f.name}: {r.stderr}")

    def test_pkill_pgrep_patterns_are_split(self):
        # pkill -f / pgrep -f のパターンは 'recei[v]er' のように割る(自分のコマンド文字列に一致して自分を巻き込まない)
        found = re.findall(r"\b(?:pkill|pgrep)\b((?:\s+-\w+)*)\s+(?:-\w+\s+)*('[^']*'|\"[^\"]*\"|\S+)", self.src)
        lines = [ln for ln in self.src.splitlines() if re.search(r"\b(pkill|pgrep)\b", ln) and not ln.lstrip().startswith("#")]
        self.assertGreaterEqual(len(lines), 6)
        for ln in lines:
            for m in re.finditer(r"\b(?:pkill|pgrep)\s+((?:-\w+\s+)*)('([^']*)'|\"([^\"]*)\"|(\S+))", ln):
                flags, pat = m.group(1), (m.group(3) or m.group(4) or m.group(5))
                if "f" in flags.replace("-", ""):
                    self.assertRegex(pat, r"\[[^\]\^]\]", f"パターンが割られていません: {ln.strip()}")
                    self.assertNotRegex(pat, r"\$", "変数のパターンは使わない")
        self.assertTrue(found)

    def test_step_order(self):
        s = self.src
        marks = ["== 0)", "== 1) 監視ループ", "== 2) ロボット", "== 3)", "== 4)", "== 5) 管理画面自身"]
        pos = [s.index(m) for m in marks]
        self.assertEqual(pos, sorted(pos), "停止の順序: モーター -> 監視ループ -> ROS -> OCR -> 音声 -> 管理画面(最後)")
        # 監視ループの停止は、ROS(stop_robot)より前
        self.assertLess(s.index("run_persistent_base_superv[i]sed"), s.index('stop_robot.sh" --keep-dashboard'))
        # ROSの停止は管理画面を残す。管理画面(full_dashboard)を止めるのは最後の節だけ
        self.assertIn("stop_robot.sh\" --keep-dashboard", s)
        last = s[pos[-1]:]
        self.assertIn("full_dashboar[d]", last)
        self.assertNotIn("full_dashboar[d]' 2>/dev/null\n  sleep", s[:pos[-1]])
        before = s[:pos[-1]]
        self.assertNotIn("pkill -f 'full_dashboar", before)
        # OSの電源断は --poweroff のときだけ(2026-10-19〜)。再起動(reboot)はしない。実行は POWEROFF=1 のガードの中だけ
        code = "\n".join(ln for ln in s.splitlines() if not ln.lstrip().startswith("#"))
        self.assertNotRegex(code, r"\b(reboot|systemctl\s+(poweroff|reboot))\b")
        self.assertEqual(code.count("/usr/sbin/shutdown -h now"), 1, "shutdownの指定は1か所だけ(SHUTDOWN_CMD)")
        guard = code.index('if [ "$POWEROFF" = "1" ]')
        self.assertLess(guard, code.index('poweroff_remote pi "'), "ラズパイの停止は --poweroff のガードの中")
        self.assertLess(code.index("== 5) 管理画面自身"), code.index("== 6) 1台目のOSを停止"), "1台目のOS停止は最後")
        # sudo は、-n(パスワードを聞かない)だけ
        self.assertNotRegex(code, r"\bsudo\s+(?!-n\b)")

    def test_options_and_log(self):
        for opt in ("--delay", "--no-ocr", "--voice"):
            self.assertIn(opt, self.src)
        self.assertIn(".nokosahen_shutdown.log", self.src)
        code = "\n".join(l for l in self.src.splitlines() if not l.lstrip().startswith("#"))
        self.assertIn("--poweroff) POWEROFF=1", code)  # 2026-10-19: OSの停止(電源断)のオプション。既定は無効(画面側が既定で付ける)
        self.assertIn("POWEROFF=0", code)

    def test_stop_robot_keep_dashboard_default_unchanged(self):
        t = (REPO / "scripts" / "stop_robot.sh").read_text(encoding="utf-8")
        self.assertIn('KEEP_DASHBOARD=0', t)
        self.assertIn('"persistent_base.launch.py"', t)
        self.assertIn('"full_dashboard"', t)


class LauncherFailureTest(unittest.TestCase):
    """停止スクリプトがすぐ(0.4秒以内)に非0で終了したら、起動失敗として500を返し、再試行できる状態に戻す。"""

    class _Proc:
        def __init__(self, rc):
            self.rc = rc

        def poll(self):
            return self.rc

    def test_script_dying_immediately_is_reported_and_retry_allowed(self):
        rec = Recorder()
        procs = [self._Proc(127), self._Proc(None)]

        def launcher(argv):
            rec.events.append("launch")
            return procs.pop(0)

        ctl = make_controller(rec)
        ctl._launcher = launcher
        code, body = ctl.request("TL", "1.2.3.4", OK)
        self.assertEqual(code, 500, body)
        self.assertEqual(body["error"], "launch_failed")
        self.assertFalse(ctl.state()["in_progress"], "失敗したら『実行中』を解除する")
        code, body = ctl.request("TL", "1.2.3.4", OK)  # 2回目は(まだ動いている)プロセス → 成功
        self.assertEqual(code, 200, body)
        self.assertTrue(ctl.state()["in_progress"])

    def test_script_exiting_zero_or_running_is_success(self):
        for proc in (self._Proc(None), self._Proc(0), None):
            rec = Recorder()
            ctl = make_controller(rec)
            ctl._launcher = lambda argv, p=proc: p
            code, body = ctl.request("TL", "1.2.3.4", OK)
            self.assertEqual(code, 200, (proc, body))

    def test_response_does_not_expose_expanded_home_path(self):
        rec = Recorder()
        code, body = make_controller(rec).request("TL", "1.2.3.4", OK)
        self.assertEqual(code, 200)
        self.assertTrue(body["log_file"].startswith("~/"), body["log_file"])


class ScriptExitStatusTest(unittest.TestCase):
    """`コマンド | tail` で終了コードを失う不具合の回帰テスト(静的確認)と、start_all.sh の dry-run。"""

    def test_scripts_do_not_pipe_commands_into_tail_for_status(self):
        for name in ("shutdown_all.sh", "stop_everything.sh", "start_all.sh"):
            text = (REPO / "scripts" / name).read_text(encoding="utf-8")
            for ln, line in enumerate(text.splitlines(), 1):
                code = line.split("#", 1)[0]
                if "| tail" in code and ("ssh " in code or "docker " in code or "stop_robot" in code or "stop_everything" in code):
                    self.fail(f"{name}:{ln} に『{code.strip()}』: パイプ(| tail)は終了コードを失う")

    def test_every_ssh_has_batchmode(self):
        for name in ("shutdown_all.sh", "stop_everything.sh", "start_all.sh"):
            text = (REPO / "scripts" / name).read_text(encoding="utf-8")
            for ln, line in enumerate(text.splitlines(), 1):
                code = line.split("#", 1)[0]
                if " ssh " in code or code.strip().startswith("ssh "):
                    self.assertTrue("BatchMode=yes" in code or "$SSH_OPTS" in code, f"{name}:{ln}: {code.strip()}")

    def test_start_all_dry_run_has_no_side_effects_and_exit_zero(self):
        import shutil
        import subprocess
        bash = shutil.which("bash")
        if not bash or sys.platform.startswith("win"):
            self.skipTest("bash(Linux)が無い")
        for mode in ("manual", "nav_test"):
            r = subprocess.run([bash, str(REPO / "scripts" / "start_all.sh"), mode, "--dry-run", "--ocr", "--display", "customer"],
                               capture_output=True, text=True, timeout=60)
            self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
            self.assertIn("dry-run", r.stdout)
        r = subprocess.run([bash, str(REPO / "scripts" / "start_all.sh"), "full"], capture_output=True, text=True, timeout=60)
        self.assertEqual(r.returncode, 3, "fullは --yes が無ければ起動しない")


class RolesTest(unittest.TestCase):
    def test_grantable_to_operator_but_discard_map_is_admin_only(self):
        # 2026-10-16: 終了そのものは operator に許可リストで付与できる。「未保存の地図を捨てる」だけ admin 専用
        self.assertEqual(roles.required_capabilities("POST", "/api/system/shutdown", {}), ["system.shutdown"])
        self.assertEqual(roles.required_capabilities("POST", "/api/system/shutdown", {"discard_map": True}),
                         ["system.shutdown", "system.shutdown_discard_map"])
        self.assertIn("system.shutdown", roles.GRANTABLE)
        self.assertIn("system.shutdown_discard_map", roles.ADMIN_ONLY)
        # 許可なし → 拒否
        self.assertFalse(roles.authorize("operator", [], "POST", "/api/system/shutdown", {}).allowed)
        # 許可あり → 通る。ただし discard_map 付きは全権限を書いても拒否(admin_only)
        self.assertTrue(roles.authorize("operator", ["system.shutdown"], "POST", "/api/system/shutdown", {}).allowed)
        d = roles.authorize("operator", list(roles.CAPABILITIES), "POST", "/api/system/shutdown", {"discard_map": True})
        self.assertFalse(d.allowed)
        self.assertTrue(d.admin_only)
        self.assertTrue(roles.authorize("admin", [], "POST", "/api/system/shutdown", {"discard_map": True}).allowed)
        self.assertFalse(roles.authorize("viewer", [], "POST", "/api/system/shutdown", {}).allowed)

    def test_listed_in_do_post_of_pc_handler(self):
        self.assertIn('"/api/system/shutdown"', (PKG / "nokosahen_core" / "dashboard_common.py").read_text(encoding="utf-8"))


# ---------------------------------------------------------------------------
def make_store():
    d = Path(tempfile.mkdtemp(prefix="sd_acc_", dir=TMP))
    (d / "accounts.yaml").write_text(
        'accounts:\n  - username: "kari-op"\n    role: operator\n'
        f'    password_hash: "{H.OP_HASH}"\n    enabled: true\n', encoding="utf-8")
    return acc.AccountStore(accounts_path=str(d / "accounts.yaml"), permissions_path=str(d / "perm.json"))


class HttpTest(unittest.TestCase):
    def start(self, perms=None, **kw):
        self.rec = Recorder()
        self.ctl = make_controller(self.rec, **kw)
        store = make_store()
        if perms:
            store.save_permissions(perms, "setup")
        self.teleop = H.FakeTeleop()
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, self.teleop, auth_username="TL", auth_password="tl-pass",
            enforce_single_controller=False, account_store=store, system_shutdown=self.ctl)
        self.srv, port = H.serve(handler)
        self.base = f"http://127.0.0.1:{port}"
        self.addCleanup(lambda: (self.srv.shutdown(), self.srv.server_close()))

    def cookie(self, user, pw):
        code, d, ck = H.http("POST", self.base + "/api/login", {"username": user, "password": pw})
        self.assertEqual(code, 200)
        return ck.split(";")[0]

    def post(self, body, ck=None):
        code, d, _ = H.http("POST", self.base + "/api/system/shutdown", body, ck)
        return code, d

    def test_permissions(self):
        # 許可リストが空のoperatorは拒否。guest・未ログインも拒否
        self.start()
        self.assertEqual(self.post(OK)[0], 401)
        self.assertEqual(self.post(OK, self.cookie("guest", ""))[0], 403)
        code, d = self.post(OK, self.cookie("kari-op", H.OP_PASSWORD))
        self.assertEqual(code, 403)
        self.assertEqual(self.rec.events, [], "拒否のときは何も止めない")
        code, d = self.post(OK, self.cookie("TL", "tl-pass"))
        self.assertEqual(code, 200, d)
        self.assertEqual(d["will_stop_in_sec"], 3)
        self.assertEqual(self.rec.events, ["motors", "launch"])

    def test_operator_with_permission_can_shutdown_but_not_discard_map(self):
        self.start(perms=["system.shutdown"], map_state=lambda: {"unsaved": True, "reason": "地図が未保存です"})
        ck = self.cookie("kari-op", H.OP_PASSWORD)
        # 未保存の地図を捨てる指定は、権限があっても403(admin専用)。何も止めない
        code, d = self.post({**OK, "discard_map": True}, ck)
        self.assertEqual(code, 403)
        self.assertTrue(d.get("admin_only"))
        self.assertEqual(self.rec.events, [])
        # 捨てない指定で未保存の地図があるなら、409で止めない
        code, d = self.post(OK, ck)
        self.assertEqual(code, 409, d)
        self.assertEqual(self.rec.events, [])
        # state は見られるが、地図を捨てる権限は無い
        code, st, _ = H.http("GET", self.base + "/api/system/shutdown/state", None, ck)
        self.assertEqual(code, 200)
        self.assertFalse(st["can_discard_map"])

    def test_operator_with_permission_shuts_down_when_map_saved(self):
        self.start(perms=["system.shutdown"])
        code, d = self.post(OK, self.cookie("kari-op", H.OP_PASSWORD))
        self.assertEqual(code, 200, d)
        self.assertEqual(self.rec.events, ["motors", "launch"])

    def test_state_denied_without_permission(self):
        self.start()
        code, _, _ = H.http("GET", self.base + "/api/system/shutdown/state", None, self.cookie("kari-op", H.OP_PASSWORD))
        self.assertEqual(code, 403)

    def test_admin_validation_and_state(self):
        self.start()
        ck = self.cookie("TL", "tl-pass")
        self.assertEqual(self.post({}, ck)[0], 400)
        self.assertEqual(self.post({"confirm": "NG"}, ck)[0], 400)
        self.assertEqual(self.post({**OK, "include_ocr": "yes"}, ck)[0], 400)
        self.assertEqual(self.post({**OK, "shell": "x"}, ck)[0], 400)
        self.assertEqual(self.rec.events, [])
        code, st, _ = H.http("GET", self.base + "/api/system/shutdown/state", None, ck)
        self.assertEqual(code, 200)
        self.assertTrue(st["enabled"] and st["script_ok"])
        self.assertFalse(st["in_progress"])
        self.assertNotIn("script", " ".join(st.keys()).replace("script_ok", ""))
        self.assertEqual(self.post(OK, ck)[0], 200)
        self.assertEqual(self.post(OK, ck)[0], 409)
        # operator・guestは状態も見られない(403)
        self.assertEqual(H.http("GET", self.base + "/api/system/shutdown/state", None, self.cookie("guest", ""))[0], 403)

    def test_map_unsaved_over_http(self):
        self.start(map_state=lambda: {"mode": "mapping", "unsaved": True, "reason": "地図が未保存です。先に保存してください"})
        ck = self.cookie("TL", "tl-pass")
        code, d = self.post(OK, ck)
        self.assertEqual(code, 409)
        self.assertIn("先に保存してください", d["message"])
        self.assertEqual(self.rec.events, [])
        self.assertEqual(self.post({**OK, "discard_map": True}, ck)[0], 200)

    def test_handler_without_controller(self):
        self.start()
        store = make_store()
        handler = dc.make_dashboard_handler(lambda: b"x", lambda: {}, H.FakeTeleop(), auth_username="TL",
                                            auth_password="tl-pass", enforce_single_controller=False, account_store=store)
        srv, port = H.serve(handler)
        self.addCleanup(lambda: (srv.shutdown(), srv.server_close()))
        code, d, ck = H.http("POST", f"http://127.0.0.1:{port}/api/login", {"username": "TL", "password": "tl-pass"})
        code, d, _ = H.http("POST", f"http://127.0.0.1:{port}/api/system/shutdown", OK, ck.split(";")[0])
        self.assertEqual(code, 404)


# ---------------------------------------------------------------------------
@unittest.skipUnless(HAVE_PW, "playwright がありません")
class UiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pw = sync_playwright().start()
        cls.browser = cls.pw.chromium.launch(channel="msedge", headless=True)

    @classmethod
    def tearDownClass(cls):
        cls.browser.close()
        cls.pw.stop()

    def start(self, map_unsaved=False, nav_active=False):
        self.rec = Recorder()
        self.ctl = make_controller(
            self.rec,
            map_state=lambda: {"mode": "mapping" if map_unsaved else "idle", "unsaved": map_unsaved,
                               "reason": "地図が未保存です。先に保存してください"},
            nav_active=lambda: nav_active)
        self.store = make_store()
        handler = dc.make_dashboard_handler(
            lambda: FS.PAGES["full"].encode("utf-8"), lambda: {}, H.FakeTeleop(),
            auth_username="TL", auth_password="tl-pass", enforce_single_controller=False,
            account_store=self.store, system_shutdown=self.ctl)
        self.srv, port = H.serve(handler)
        self.base = f"http://127.0.0.1:{port}"
        self.addCleanup(lambda: (self.srv.shutdown(), self.srv.server_close()))

    def open(self, user, pw):
        ctx = self.browser.new_context(viewport={"width": 1100, "height": 1500})
        self.addCleanup(ctx.close)
        r = ctx.request.post(self.base + "/api/login", data=json.dumps({"username": user, "password": pw}),
                             headers={"Content-Type": "application/json"})
        self.assertEqual(r.status, 200)
        page = ctx.new_page()
        self.errors = []
        self.requests = []
        page.on("pageerror", lambda e: self.errors.append(str(e)))
        page.on("request", lambda rq: self.requests.append((time.time(), rq.method, rq.url)))
        page.goto(self.base + "/")
        page.evaluate("() => { document.querySelector('#tabs [data-tab=settings]').click(); }")
        page.wait_for_selector("#shutdownCard", state="visible", timeout=15000)
        page.wait_for_timeout(1200)
        return page

    def test_admin_flow_countdown_and_polling_stops(self):
        self.start(nav_active=True)
        page = self.open("TL", "tl-pass")
        card_text = page.inner_text("#shutdownCard")
        # 【2026-11-05】2026-10-19 に「OSも停止して電源を落とす」を既定ONにしたので、文言と既定値の期待を現行の仕様に合わせた
        for word in ("止まるもの", "チェックを外したとき", "再起動の方法", "start_all.sh", "SSH", "電源を抜いてよい状態"):
            self.assertIn(word, card_text)
        self.assertIn("走行中", page.inner_text("#shNavWarn"))
        self.assertFalse(page.is_visible("#shDiscardRow"))  # 地図が未保存でなければ出ない
        self.assertTrue(page.is_checked("#shOcr"))
        self.assertFalse(page.is_checked("#shVoice"))
        self.assertTrue(page.is_checked("#shPower"))  # OSも停止は既定ON
        # 確認文字列を入れるまで無効
        self.assertTrue(page.is_disabled("#shRunBtn"))
        page.fill("#shConfirm", "シャットダウ")
        self.assertTrue(page.is_disabled("#shRunBtn"))
        page.fill("#shConfirm", "シャットダウン ")
        self.assertTrue(page.is_disabled("#shRunBtn"))
        page.fill("#shConfirm", "シャットダウン")
        self.assertFalse(page.is_disabled("#shRunBtn"))
        # 緊急停止ドックは最前面のまま
        self.assertEqual(page.evaluate("() => { const d = document.getElementById('estopDock'); const s = getComputedStyle(d); return [s.position, s.zIndex]; }"), ["fixed", "2000"])
        page.click("#shRunBtn")
        self.assertTrue(page.is_visible("#shBar"))
        self.assertIn("本当に止めますか", page.inner_text("#shBarText"))
        self.assertIn("走行中", page.inner_text("#shBarText"))
        page.click("#shNo")
        self.assertFalse(page.is_visible("#shBar"))
        self.assertEqual(self.rec.events, [])
        page.screenshot(path=SHOT, full_page=False)
        page.click("#shRunBtn")
        page.click("#shYes")
        page.wait_for_function("() => document.getElementById('shStatus').textContent.indexOf('停止処理を開始しました') >= 0", timeout=10000)
        self.assertEqual(self.rec.events, ["motors", "launch"])
        self.assertEqual(self.rec.launched[0], ["bash", str(self.ctl.script_path), "--delay", "3", "--poweroff"])
        status = page.inner_text("#shStatus")
        self.assertIn("数秒後にこの画面は開けなくなります", status)
        self.assertIn("接続が切れます", status)
        self.assertIn("停止処理を開始しました", page.inner_text("#shutdownBanner"))
        self.assertTrue(page.is_disabled("#shRunBtn"))
        # カウントダウンが進み、最後に「接続が切れました」になる
        page.wait_for_function("() => document.getElementById('shStatus').textContent.indexOf('接続が切れました') >= 0", timeout=10000)
        # ポーリングが止まっている(以後の通信なし)
        t0 = time.time()
        page.wait_for_timeout(3500)
        after = [r for r in self.requests if r[0] > t0]
        self.assertEqual(after, [], f"停止後に通信が続いています: {after[:5]}")
        self.assertTrue(page.evaluate("() => NKField.frozen()"))
        self.assertEqual([e for e in self.errors if "Failed" in e or "NetworkError" in e], [])

    def test_options_are_sent(self):
        self.start()
        page = self.open("TL", "tl-pass")
        page.uncheck("#shOcr")
        page.check("#shVoice")
        page.fill("#shConfirm", "シャットダウン")
        page.click("#shRunBtn")
        page.click("#shYes")
        page.wait_for_function("() => document.getElementById('shStatus').textContent.indexOf('停止処理を開始しました') >= 0", timeout=10000)
        self.assertEqual(self.rec.launched[0][3:], ["3", "--no-ocr", "--voice", "--poweroff"])  # OSも停止は既定ON

    def test_discard_checkbox_only_when_map_unsaved(self):
        self.start(map_unsaved=True)
        page = self.open("TL", "tl-pass")
        self.assertTrue(page.is_visible("#shDiscardRow"))
        page.fill("#shConfirm", "シャットダウン")
        self.assertTrue(page.is_disabled("#shRunBtn"), "未保存の地図があるときは、捨てるにチェックするまで押せない")
        self.assertIn("地図が未保存", page.inner_text("#shMapWarn"))
        page.check("#shDiscard")
        self.assertFalse(page.is_disabled("#shRunBtn"))
        page.click("#shRunBtn")
        self.assertIn("未保存の地図は捨てられます", page.inner_text("#shBarText"))
        page.click("#shYes")
        page.wait_for_function("() => document.getElementById('shStatus').textContent.indexOf('停止処理を開始しました') >= 0", timeout=10000)
        self.assertEqual(self.rec.events, ["motors", "launch"])

    def test_server_rejection_is_shown_and_page_keeps_working(self):
        self.start()
        page = self.open("TL", "tl-pass")
        self.ctl.enabled = False
        page.fill("#shConfirm", "シャットダウン")
        page.evaluate("() => { document.getElementById('shRunBtn').disabled = false; }")
        page.click("#shRunBtn")
        page.click("#shYes")
        page.wait_for_function("() => document.getElementById('shStatus').textContent.indexOf('止めませんでした') >= 0", timeout=10000)
        self.assertEqual(self.rec.events, [])
        self.assertFalse(page.evaluate("() => NKField.frozen()"))

    def test_operator_card_is_disabled_with_reason(self):
        self.start()
        page = self.open("kari-op", H.OP_PASSWORD)
        page.wait_for_timeout(800)
        for sel in ("#shRunBtn", "#shConfirm", "#shOcr", "#shVoice"):
            self.assertTrue(page.is_disabled(sel), sel)
        self.assertIn("許可されていません", page.inner_text("#shutdownCard"))
        # 無効化を外して送っても、サーバーが403
        code = page.evaluate("""async () => (await fetch('/api/system/shutdown', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({confirm:'シャットダウン'})})).status""")
        self.assertEqual(code, 403)
        self.assertEqual(self.rec.events, [])
        self.assertEqual(self.errors, [])

    def test_guest_card_is_disabled(self):
        self.start()
        page = self.open("guest", "")
        page.wait_for_timeout(800)
        for sel in ("#shRunBtn", "#shConfirm", "#shOcr", "#shVoice"):
            self.assertTrue(page.is_disabled(sel), sel)
        self.assertIn("許可されていません", page.inner_text("#shLock"))


if __name__ == "__main__":
    unittest.main(verbosity=1)
