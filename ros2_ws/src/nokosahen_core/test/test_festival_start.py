"""文化祭モード(scripts/festival/)の静的検査と dry-run・退避ロジックのテスト(2026-11-06)。
ROS・実機・systemd・2台目は使わない(HOMEを一時フォルダにし、偽のcurlをPATHの先頭に置く)。

- bash構文、LF、sudo不使用、秘密を読まない、pkill -f / pgrep -f を使わない(自分を巻き込まない)
- 起動構成: 自律行動あり(監視ループの引数が空)、アンケート監視ON、近づく/見回す等を自動でONにしない
- 出店紹介の設定(pr_intro.festival.json): pr_introの検証を通る、出店→ロボットの順、30秒/30秒、ON、speech_mode=pr
- --dry-run は何も変えない
- アンケートの退避: 1回だけ・削除ではなく移動・condition.txtは残す・2回目は何もしない・指定なしでは動かさない
- BGMの見張り: enabled=true のときだけOFFを送る
- boot_start.sh: フラグファイルがあるときだけ festival_start.sh --boot、manual以外は従来どおり拒否
"""
import json
import os
import re
import shutil
import stat
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
FEST = REPO / "scripts" / "festival"
START = FEST / "festival_start.sh"
KEEPER = FEST / "festival_voice_keeper.sh"
MODE = FEST / "festival_mode.sh"
BOOT = REPO / "scripts" / "autostart" / "boot_start.sh"
STOP_ROBOT = REPO / "scripts" / "stop_robot.sh"
TEMPLATE = FEST / "pr_intro.festival.json"
PKG = REPO / "ros2_ws" / "src" / "nokosahen_core"
BASH = shutil.which("bash")
SCRIPTS = [START, KEEPER, MODE]

sys.path.insert(0, str(PKG))


def p(path) -> str:
    return Path(path).as_posix()


def run(script, args=(), home=None, env=None, timeout=60):
    e = dict(os.environ)
    e["HOME"] = p(home) if home else e.get("HOME", "")
    e["REPO"] = p(REPO)
    e.pop("NOKOSAHEN_FESTIVAL_FLAG", None)
    e.pop("SURVEY_DATA_DIR", None)
    e.update(env or {})
    return subprocess.run([BASH, p(script)] + list(args), capture_output=True, text=True, timeout=timeout, env=e,
                          stdin=subprocess.DEVNULL, encoding="utf-8")


class StaticTest(unittest.TestCase):
    def test_files_exist_lf(self):
        for s in SCRIPTS + [TEMPLATE, FEST / "README.md"]:
            self.assertTrue(s.is_file(), s)
        for s in SCRIPTS:
            self.assertNotIn(b"\r", s.read_bytes(), f"{s.name}: LFであること")

    @unittest.skipIf(BASH is None, "bashが無い")
    def test_bash_syntax(self):
        for s in SCRIPTS + [BOOT]:
            r = subprocess.run([BASH, "-n", p(s)], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, f"{s.name}: {r.stderr}")

    def code_of(self, path):
        return "\n".join(l for l in path.read_text(encoding="utf-8").splitlines() if not l.lstrip().startswith("#"))

    def test_no_sudo_no_secrets_no_pkill_f(self):
        for s in SCRIPTS:
            code = self.code_of(s)
            self.assertNotRegex(code, r"\bsudo\b", s.name)
            self.assertNotRegex(code, r"\bp(kill|grep)\s+(-\w*f|-f)", f"{s.name}: pkill -f / pgrep -f は自分を巻き込む")
            self.assertNotRegex(code, r"(?i)secrets|password|passwd|\.pem|\.key\b", s.name)
            self.assertNotRegex(code, r"\brm\s+-[a-z]*r", f"{s.name}: 再帰削除はしない(退避は移動)")

    def test_no_stop_robot_pattern_in_own_commandline(self):
        # stop_robot.sh のパターンが、festival_start.sh 自身のコマンド行(ファイル名)に一致すると、全停止で自分を止めてしまう
        text = STOP_ROBOT.read_text(encoding="utf-8")
        body = re.search(r"PATTERNS=\((.*?)\n\)", text, re.S).group(1)
        pats = re.findall(r'"([^"]+)"', body)
        for s in SCRIPTS:
            cmd = f"bash /home/robot/mirs2603_nokosahen/scripts/festival/{s.name} --boot --reset-survey-once"
            for pat in pats:
                rx = pat if pat.endswith(".launch.py") else r"(^|[ /])" + re.escape(pat)
                self.assertIsNone(re.search(rx, cmd), (s.name, pat))

    def test_launch_configuration(self):
        code = self.code_of(START)
        # 自律行動あり = 監視ループの引数に rc_only を付けない
        self.assertNotIn("rc_only", code)
        self.assertIn("use_survey_monitor:=true", code)
        self.assertIn("use_full_dashboard:=true", code)
        self.assertIn("use_stage2_dashboard:=false", code)
        # 近づく・見回す・向く・揺動・探索を、自動でONにする経路を持たない
        for bad in ("person_interaction_enabled", "scan_enabled", "face_enabled", "idle_wiggle_enabled", "frontier",
                    "/api/", "behavior_director/"):
            self.assertNotIn(bad, code, bad)
        # 既定: 後ろカメラOFF・人物検出ON
        self.assertIn("REAR=0; PERSON=1", code)

    def test_boot_start_hook(self):
        text = BOOT.read_text(encoding="utf-8")
        self.assertIn(".nokosahen_festival_mode", text)
        self.assertIn("festival_start.sh\" --boot", text)
        # 従来の拒否は変えていない
        self.assertIn('if [ "$MODE" != "manual" ]', text)
        self.assertLess(text.index('if [ "$MODE" != "manual" ]'), text.index("festival_start.sh"))
        unit = (REPO / "scripts" / "autostart" / "nokosahen-robot.service.in").read_text(encoding="utf-8")
        self.assertIn("boot_start.sh manual", unit, "systemdユニットは変えない(フラグで切り替える)")


class TemplateTest(unittest.TestCase):
    def setUp(self):
        self.doc = json.loads(TEMPLATE.read_text(encoding="utf-8"))
        self.kousensai = json.loads((REPO / "config" / "pr_intro.kousensai.json").read_text(encoding="utf-8"))

    def test_valid_for_pr_intro_and_30_second_alternation(self):
        from nokosahen_core import pr_intro
        clean, errors = pr_intro.validate_update(self.doc)
        self.assertEqual(errors, [])
        cfg = pr_intro.merged_config(clean, True)
        self.assertEqual(pr_intro.check_consistency(cfg), [])
        self.assertTrue(cfg["enabled"])
        self.assertEqual(cfg["speech_mode"], "pr")
        self.assertEqual((cfg["interval_sec"], cfg["gap_sec"]), (30, 30))
        self.assertFalse(cfg["quiet_when_person_near"])
        parts = pr_intro.enabled_parts(cfg)
        self.assertEqual(len(parts), 2)
        self.assertIn("出店", parts[0]["title"])        # 1つ目: 出店の説明
        self.assertIn("ロボット", parts[1]["title"])    # 2つ目: ロボットの説明
        # 文章は、本番の文章(config/pr_intro.kousensai.json)と同じ。仮の文を含まない
        self.assertEqual([x["text"] for x in parts], [x["text"] for x in self.kousensai["parts"]])
        for x in parts:
            self.assertFalse(pr_intro.is_placeholder(x["text"]))

    def test_playlist_sent_to_second_unit(self):
        from nokosahen_core import pr_intro
        clean, _ = pr_intro.validate_update(self.doc)
        mgr = pr_intro.PrIntroManager(pr_intro.merged_config(clean, True), settle_sec=0.0)
        ops = mgr.update({}, {}, 100.0)
        pl = [o for o in ops if o["op"] == "playlist"]
        self.assertEqual(len(pl), 1)
        self.assertEqual((pl[0]["interval_sec"], pl[0]["gap_sec"]), (30.0, 30.0))
        self.assertEqual(len(pl[0]["parts"]), 2)
        self.assertTrue(any(o["op"] == "prepare" for o in ops))   # 音声の事前作成も依頼する
        # 人の方を向く(その場の旋回)は、紹介を止める状況に含まれない
        self.assertNotIn("face", pr_intro.BLOCK_FLAGS)


@unittest.skipIf(BASH is None, "bashが無い")
class DryRunTest(unittest.TestCase):
    def setUp(self):
        self.home = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.home, True)

    def test_dry_run_changes_nothing_and_describes(self):
        survey = Path(self.home) / "survey_data"
        survey.mkdir()
        (survey / "responses.jsonl").write_text('{"a":1}\n', encoding="utf-8")
        r = run(START, ["--dry-run", "--reset-survey-once"], self.home)
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout
        for s in ("自律行動(behavior_director)=あり", "すべてOFF", "アンケート監視=ON", "BGM=OFF",
                  "use_survey_monitor:=true", "use_person_target_detector:=true", "use_rear_cameras:=false",
                  "間隔 30秒", "stop_everything.sh", "survey_data_backup_before_festival_", "festival_voice_keeper.sh",
                  "run_persistent_base_supervised.sh"):
            self.assertIn(s, out)
        self.assertNotIn("rc_only", out)
        # 何も作らない・動かさない
        self.assertEqual(sorted(x.name for x in Path(self.home).iterdir()), ["survey_data"])
        self.assertTrue((survey / "responses.jsonl").exists())

    def test_boot_dry_run_does_not_stop_everything(self):
        out = run(START, ["--boot", "--dry-run"], self.home).stdout
        self.assertNotIn("stop_everything", out)
        self.assertIn("exec bash", out)
        self.assertIn("アンケート退避=しない", out)

    def test_bad_args(self):
        self.assertEqual(run(START, ["--bogus"], self.home).returncode, 2)
        self.assertEqual(run(START, ["--interval", "abc"], self.home).returncode, 2)
        self.assertEqual(run(START, ["--interval", "5"], self.home).returncode, 2)

    def test_flag_file_options_only_with_boot(self):
        flag = Path(self.home) / ".nokosahen_festival_mode"
        flag.write_text("# コメント\nrear_cameras\nreset_survey_once  # 1回だけ\ninterval=45\nno_dds_wired\nnonsense\n", encoding="utf-8")
        out = run(START, ["--boot", "--dry-run"], self.home).stdout
        self.assertIn("use_rear_cameras:=true", out)
        self.assertIn("間隔 45秒", out)
        self.assertIn("ROS有線限定=false", out)
        self.assertIn("アンケート退避=1回だけ", out)
        # --boot でない手動実行では、フラグファイルを読まない
        out = run(START, ["--dry-run"], self.home).stdout
        self.assertIn("use_rear_cameras:=false", out)
        self.assertIn("間隔 30秒", out)

    def test_warns_when_policy_files_exist(self):
        (Path(self.home) / ".nokosahen_behavior_tree.json").write_text("{}", encoding="utf-8")
        self.assertIn("注意:", run(START, ["--dry-run"], self.home).stdout)


@unittest.skipIf(BASH is None, "bashが無い")
class SurveyResetTest(unittest.TestCase):
    def setUp(self):
        self.home = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.home, True)
        self.survey = self.home / "survey_data"
        self.survey.mkdir()
        (self.survey / "responses.jsonl").write_text('{"response_id":"a"}\n{"response_id":"b"}\n', encoding="utf-8")
        (self.survey / "responses.csv").write_text("x\n", encoding="utf-8")
        (self.survey / "condition.txt").write_text("B\n", encoding="utf-8")
        self.marker = self.home / ".nokosahen_festival_survey_reset_done"

    def go(self, *extra):
        return run(START, ["--prepare-only", "--no-pr-intro", *extra], self.home)

    def backups(self):
        return sorted(self.home.glob("survey_data_backup_before_festival_*"))

    def test_without_flag_nothing_moves(self):
        r = self.go()
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertTrue((self.survey / "responses.jsonl").exists())
        self.assertEqual(self.backups(), [])
        self.assertFalse(self.marker.exists())

    def test_reset_once_moves_not_deletes_and_second_run_keeps_new_data(self):
        r = self.go("--reset-survey-once")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        bk = self.backups()
        self.assertEqual(len(bk), 1)
        # 移動: 元から消え、退避先に中身がそのまま残る(削除していない)
        self.assertFalse((self.survey / "responses.jsonl").exists())
        self.assertFalse((self.survey / "responses.csv").exists())
        self.assertEqual((bk[0] / "responses.jsonl").read_text(encoding="utf-8"), '{"response_id":"a"}\n{"response_id":"b"}\n')
        self.assertTrue((bk[0] / "responses.csv").exists())
        # 実験条件(condition.txt)は元の場所に残し、退避先にもコピーする
        self.assertEqual((self.survey / "condition.txt").read_text(encoding="utf-8"), "B\n")
        self.assertTrue((bk[0] / "condition.txt").exists())
        self.assertTrue(self.marker.exists())
        self.assertIn(bk[0].name, self.marker.read_text(encoding="utf-8"))
        # 2回目(8日の起動など): 新しい回答があっても消さない・退避を増やさない
        (self.survey / "responses.jsonl").write_text('{"response_id":"new"}\n', encoding="utf-8")
        r = self.go("--reset-survey-once")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual((self.survey / "responses.jsonl").read_text(encoding="utf-8"), '{"response_id":"new"}\n')
        self.assertEqual(len(self.backups()), 1)
        self.assertIn("すでに退避済み", r.stdout)

    def test_reset_with_no_data_writes_marker_and_creates_no_backup(self):
        for f in self.survey.glob("responses.*"):
            f.unlink()
        self.assertEqual(self.go("--reset-survey-once").returncode, 0)
        self.assertEqual(self.backups(), [])
        self.assertTrue(self.marker.exists())

    def test_custom_survey_dir_env(self):
        other = self.home / "elsewhere"
        other.mkdir()
        (other / "responses.jsonl").write_text("{}\n", encoding="utf-8")
        r = run(START, ["--prepare-only", "--no-pr-intro", "--reset-survey-once"], self.home,
                env={"SURVEY_DATA_DIR": p(other)})
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertFalse((other / "responses.jsonl").exists())
        self.assertTrue((self.survey / "responses.jsonl").exists())   # 既定の場所は触らない


@unittest.skipIf(BASH is None, "bashが無い")
class PrIntroInstallTest(unittest.TestCase):
    def setUp(self):
        self.home = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.home, True)
        self.dst = self.home / ".nokosahen_pr_intro.json"

    def test_installs_valid_config_with_interval_and_backs_up_old(self):
        self.dst.write_text('{"version":1,"enabled":false}', encoding="utf-8")
        r = run(START, ["--prepare-only", "--interval", "45"], self.home)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        doc = json.loads(self.dst.read_text(encoding="utf-8"))
        self.assertEqual((doc["interval_sec"], doc["gap_sec"]), (45, 45))
        self.assertTrue(doc["enabled"])
        self.assertEqual(doc["speech_mode"], "pr")
        from nokosahen_core import pr_intro
        self.assertEqual(pr_intro.validate_update(doc)[1], [])
        baks = list(self.home.glob(".nokosahen_pr_intro.json.bak_before_festival_*"))
        self.assertEqual(len(baks), 1)
        self.assertEqual(baks[0].read_text(encoding="utf-8"), '{"version":1,"enabled":false}')
        # 同じ内容なら、もう一度置いても変えず、バックアップも増えない
        r = run(START, ["--prepare-only", "--interval", "45"], self.home)
        self.assertIn("変更なし", r.stdout)
        self.assertEqual(len(list(self.home.glob(".nokosahen_pr_intro.json.bak_before_festival_*"))), 1)

    def test_default_30(self):
        run(START, ["--prepare-only"], self.home)
        doc = json.loads(self.dst.read_text(encoding="utf-8"))
        self.assertEqual((doc["interval_sec"], doc["gap_sec"]), (30, 30))
        self.assertEqual(json.loads(TEMPLATE.read_text(encoding="utf-8"))["parts"], doc["parts"])

    def test_no_pr_intro_leaves_file(self):
        run(START, ["--prepare-only", "--no-pr-intro"], self.home)
        self.assertFalse(self.dst.exists())


@unittest.skipIf(BASH is None, "bashが無い")
class KeeperTest(unittest.TestCase):
    def fake_curl(self, home: Path, bgm_state: str):
        bindir = home / "bin"
        bindir.mkdir()
        c = bindir / "curl"
        c.write_text(
            "#!/usr/bin/env bash\n"
            f'echo "$@" >> "{p(home)}/curl_calls.txt"\n'
            'case "$*" in\n'
            '  *-X\\ POST*) echo \'{"enabled": false}\' ;;\n'
            f"  *) echo '{bgm_state}' ;;\n"
            "esac\n", encoding="utf-8", newline="\n")
        c.chmod(c.stat().st_mode | stat.S_IEXEC)
        return bindir

    def run_keeper(self, state):
        home = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, home, True)
        bindir = self.fake_curl(home, state)
        env = {"PATH": p(bindir) + os.pathsep + os.environ.get("PATH", "")}
        r = run(KEEPER, ["--once"], home, env=env)
        calls = (home / "curl_calls.txt").read_text(encoding="utf-8") if (home / "curl_calls.txt").exists() else ""
        return r, calls

    def test_turns_bgm_off_when_enabled(self):
        r, calls = self.run_keeper('{"enabled": true, "playing": "a.wav"}')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("-X POST", calls)
        self.assertIn('{"enabled": false}', calls)
        self.assertIn("198.51.100.3:8100/bgm", calls)
        self.assertNotIn("volume", calls)   # 触るのは enabled だけ

    def test_does_nothing_when_already_off(self):
        r, calls = self.run_keeper('{"enabled": false}')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertNotIn("-X POST", calls)

    def test_unreachable_is_not_an_error_loop_crash(self):
        r, calls = self.run_keeper("")
        self.assertEqual(r.returncode, 1)
        self.assertNotIn("-X POST", calls)


@unittest.skipIf(BASH is None, "bashが無い")
class ModeToggleTest(unittest.TestCase):
    def test_enable_disable_status(self):
        home = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, home, True)
        flag = home / ".nokosahen_festival_mode"
        self.assertIn("無効", run(MODE, ["status"], home).stdout)
        r = run(MODE, ["enable", "reset_survey_once", "interval=40"], home)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertTrue(flag.exists())
        lines = [l for l in flag.read_text(encoding="utf-8").splitlines() if not l.startswith("#")]
        self.assertEqual(lines, ["reset_survey_once", "interval=40"])
        self.assertIn("有効", run(MODE, ["status"], home).stdout)
        self.assertEqual(run(MODE, ["enable", "full"], home).returncode, 2)   # 未知の語(fullなど)は拒否
        self.assertEqual(run(MODE, ["disable"], home).returncode, 0)
        self.assertFalse(flag.exists())


if __name__ == "__main__":
    unittest.main(verbosity=1)
