"""scripts/pc/deploy_jetson1.sh(PCから1台目へ配置・ビルド・再起動)の静的・単体テスト(2026-10-24)。
ROS・実機・本物のssh/rsyncは使わない(届かない先を指定、または偽のssh/rsync/pingをPATHの先頭に置く)。

- bash構文、LF、使うsudoはdateだけ
- --dry-run: rsyncコマンドを表示し、除外に秘密・.git・作業日報が入る / 1台目へつながない
- 届かないとき終了コード2、引数の解釈(不正は64)
- 除外一覧が決めた運用と一致し、.gitignoreの秘密と矛盾しない
- 起動コマンドの文字列が、stop_robot.sh のpkillパターンに一致しない(自分を止めない)
- 偽ssh/rsyncで全工程を通し、結果の表・終了コード(ビルド失敗=3、確認NG=1)・再起動前の確認を見る
"""
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
SCRIPT = REPO / "scripts" / "pc" / "deploy_jetson1.sh"
STOP_ROBOT = REPO / "scripts" / "stop_robot.sh"
BASH = shutil.which("bash") if not sys.platform.startswith("win") else None

# 決めた運用(rsyncの除外)。変えるときは、スクリプトとこのテストとdocs/deploy_jetson1.mdを一緒に直す
EXPECTED_EXCLUDES = {
    ".git", "__pycache__", "*.pyc", "build/", "install/", "log/",
    "*.local.yaml", "*.local.json", "secrets.*",
    "作業日報.md", "work/", "outputs/", "poc_tests/", "docs/report/", "firmware/*/.pio",
    "*.keras", "digit_model", ".claude", "standalone_checkout_cam", ".agents", ".codex", ".vscode",
}


def parse_array(name: str) -> list:
    """スクリプトの bash 配列 NAME=( ... ) の要素を取り出す(クォートを外す)。"""
    text = SCRIPT.read_text(encoding="utf-8")
    m = re.search(r"^%s=\((.*?)^\)|^%s=\((.*?)\)\s*$" % (name, name), text, re.S | re.M)
    assert m, name
    body = m.group(1) or m.group(2)
    body = "\n".join(l.split("  #")[0] for l in body.splitlines())
    return [t.strip("'\"") for t in body.split()]


def run_script(args, env=None, timeout=60, stdin=subprocess.DEVNULL):
    e = dict(os.environ)
    e.update(env or {})
    return subprocess.run([BASH, str(SCRIPT)] + args, capture_output=True, text=True, timeout=timeout, env=e,
                          stdin=stdin, encoding="utf-8")


@unittest.skipIf(BASH is None, "bash(Linux)が無い")
class StaticTest(unittest.TestCase):
    def test_exists_lf_and_syntax(self):
        self.assertTrue(SCRIPT.is_file())
        self.assertNotIn(b"\r", SCRIPT.read_bytes(), "LFであること(.gitattributes: *.sh eol=lf)")
        r = subprocess.run([BASH, "-n", str(SCRIPT)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)

    def test_sudo_only_date(self):
        text = SCRIPT.read_text(encoding="utf-8")
        code = [l for l in text.splitlines() if not l.lstrip().startswith("#")]
        uses = [l for l in code if re.search(r"\bsudo\b", l) and not re.match(r"\s*(echo|row)\b", l)]
        self.assertTrue(uses)
        for l in uses:
            self.assertIn("sudo -n /usr/bin/date -s", l, l)
        self.assertNotRegex("\n".join(code), r"(?i)password|passwd|cat\s+[^|\n]*\.pem|/etc/shadow")

    def test_excludes_match_decided_list(self):
        self.assertEqual(set(parse_array("EXCLUDES")), EXPECTED_EXCLUDES)

    def test_secret_excludes_agree_with_gitignore(self):
        gi = (REPO / ".gitignore").read_text(encoding="utf-8").splitlines()
        gi = {l.strip() for l in gi}
        for p in ("*.local.yaml", "*.local.json", "secrets.local.yaml"):
            self.assertIn(p, gi)
        ex = set(parse_array("EXCLUDES")) | set(parse_array("EXCLUDES_EXTRA"))
        for p in ("*.local.yaml", "*.local.json", "secrets.*", ".env", "*.pem", "*.key", "id_rsa*", "credentials*.json"):
            self.assertIn(p, ex)

    def test_existing_scripts_unchanged_contract(self):
        # 既存の運用(start_all.shのオプション)が、いまも使える名前であること
        sa = (REPO / "scripts" / "start_all.sh").read_text(encoding="utf-8")
        for opt in ("--no-dds-wired", "--no-voice", "--no-clock", "--no-survey-monitor",
                    "--no-person-detector", "--no-rear-cameras", "--person-detector", "--rear-cameras", "--wait"):
            self.assertIn(opt, sa)


def stop_patterns():
    text = STOP_ROBOT.read_text(encoding="utf-8")
    body = re.search(r"PATTERNS=\((.*?)\n\)", text, re.S).group(1)
    pats = re.findall(r'"([^"]+)"', body)
    return [p if p.endswith(".launch.py") else r"(^|[ /])" + re.escape(p) for p in pats]


@unittest.skipIf(BASH is None, "bash(Linux)が無い")
class DryRunTest(unittest.TestCase):
    def test_dry_run_shows_rsync_and_excludes(self):
        r = run_script(["--dry-run"], env={"NOKOSAHEN_JETSON1": "x@nonexistent-host.invalid"})
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout
        self.assertIn("(dry-run) rsync ", out)
        for ex in ("--exclude=.git ", "'--exclude=*.local.yaml'", "'--exclude=*.local.json'", "'--exclude=secrets.*'",
                   "'--exclude=作業日報.md'", "--exclude=outputs/", "--exclude=poc_tests/", "--exclude=.claude"):
            self.assertIn(ex, out)
        self.assertIn("--backup-dir=", out)
        self.assertRegex(out, r"deploy_backup_\d{8}")
        self.assertIn("colcon build --symlink-install --packages-select nokosahen_core nokosahen_bringup", out)
        self.assertIn("lidar_src_build", out)
        self.assertIn("microros_ws", out)
        self.assertIn("/opt/ros/jazzy", out)
        self.assertIn("sudo -n /usr/bin/date -s", out)
        self.assertNotIn("接続できず", out)  # つないでいない

    def test_default_start_command(self):
        out = run_script(["--dry-run"]).stdout
        self.assertIn("start_all.sh manual --no-dds-wired --no-voice --no-clock --no-survey-monitor "
                      "--no-person-detector --no-rear-cameras", out)
        self.assertIn("start_all_single.log", out)
        self.assertNotIn("--restart", out)

    def test_cameras_option(self):
        out = run_script(["--dry-run", "--cameras"]).stdout
        self.assertIn("--person-detector --rear-cameras", out)
        self.assertNotIn("--no-person-detector", out)

    def test_no_restart(self):
        out = run_script(["--dry-run", "--no-restart"]).stdout
        self.assertNotIn("start_all.sh", out)
        self.assertIn("再起動: しない", out)
        self.assertIn("起動後の確認: しない", out)
        out = run_script(["--dry-run", "--no-restart", "--verify"]).stdout
        self.assertIn("起動後の確認(読み取りだけ)", out)

    def test_yes_and_confirmation_text(self):
        self.assertIn("実行前に確認します", run_script(["--dry-run"]).stdout)
        self.assertIn("確認は省略(--yes)", run_script(["--dry-run", "--yes"]).stdout)

    def test_restart_manual_accepted_and_others_rejected(self):
        self.assertEqual(run_script(["--dry-run", "--restart", "manual"]).returncode, 0)
        self.assertEqual(run_script(["--dry-run", "--restart", "full"]).returncode, 64)
        self.assertEqual(run_script(["--dry-run", "--restart"]).returncode, 64)

    def test_bad_args(self):
        r = run_script(["--bogus"])
        self.assertEqual(r.returncode, 64)
        self.assertIn("不明な引数", r.stdout)
        self.assertEqual(run_script(["--host"]).returncode, 64)

    def test_help(self):
        r = run_script(["-h"])
        self.assertEqual(r.returncode, 0)
        for s in ("--dry-run", "--no-restart", "--cameras", "--yes", "終了コード"):
            self.assertIn(s, r.stdout)

    def test_host_option_shown(self):
        out = run_script(["--dry-run", "--host", "abc@10.9.9.9", "--key", "/tmp/k"]).stdout
        self.assertIn("abc@10.9.9.9:mirs2603_nokosahen/", out)
        self.assertIn("/home/abc/deploy_backup_", out)

    def test_start_command_does_not_match_stop_patterns(self):
        for args in ([], ["--cameras"]):
            out = run_script(["--dry-run"] + args).stdout
            line = [l for l in out.splitlines() if "(dry-run) 起動:" in l][0].split("起動:", 1)[1]
            for pat in stop_patterns():
                self.assertIsNone(re.search(pat, line), f"{pat} が起動コマンドに一致: {line}")


@unittest.skipIf(BASH is None, "bash(Linux)が無い")
class UnreachableTest(unittest.TestCase):
    def test_unreachable_exits_2_with_wifi_hint(self):
        with tempfile.TemporaryDirectory() as d:
            r = run_script(["--yes"], env={"NOKOSAHEN_JETSON1": "robot@nonexistent-host.invalid",
                                           "NOKOSAHEN_DEPLOY_LOG_DIR": d, "NOKOSAHEN_SSH_KEY": d + "/none"})
            self.assertEqual(r.returncode, 2, r.stdout + r.stderr)
            self.assertIn("nokosahen", r.stdout)
            self.assertIn("Wi-Fi", r.stdout)
            logs = list(Path(d).glob("*.log"))
            self.assertEqual(len(logs), 1)  # PC側にもログが残る
            self.assertIn("届きません", logs[0].read_text(encoding="utf-8"))


FAKE_SSH = r'''#!/usr/bin/env bash
# 偽ssh: 本物には出ない。コマンド文字列を $FAKE_DIR/ssh_cmds に記録し、内容に応じて返す。
args=("$@"); tgt=""; i=0
while [ $i -lt ${#args[@]} ]; do
  case "${args[$i]}" in -o|-i) i=$((i+2)); continue ;; -*) i=$((i+1)); continue ;; esac
  tgt="${args[$i]}"; i=$((i+1)); break
done
cmd="${args[*]:$i}"
echo "$cmd" >> "$FAKE_DIR/ssh_cmds"
if [ "${cmd%% *}" = "bash" ]; then
  body=$(cat)
  echo "$body" > "$FAKE_DIR/last_stdin"
  case "$body" in
    *colcon\ build*) echo "Starting >>> nokosahen_core"; [ "${FAKE_BUILD_FAIL:-0}" = 1 ] && { echo "Failed <<< nokosahen_core"; exit 1; }
                     echo "Summary: 2 packages finished"; exit 0 ;;
    *lifecycle_status*) echo "lifecycle_status.code=200"; echo "lifecycle_status.body={\"mode\": \"${FAKE_MAPMODE:-idle}\", \"running\": ${FAKE_MAPRUN:-false}}"; exit 0 ;;
    *proc.*)
      echo "http=200"; echo "proc.full_dashboard=1"; echo "proc.micro_ros_agent=1"; echo "proc.rplidar_node=${FAKE_LIDAR:-1}"
      echo "proc.person_target_detector=0"; echo "proc.rear_camera_persons=0"
      echo "load=${FAKE_LOAD:-2.50}"; echo "cores=6"; echo "epoch=$(date +%s)"
      echo "scan_hz=10.01"; echo "ticks_hz=${FAKE_TICKS-49.9}"
      echo "topic./scan=1"; echo "topic./wheel_ticks=1"; echo "topic./estop_state=1"; echo "estop_val=data: false"
      exit 0 ;;
  esac
  exit 0
fi
case "$cmd" in
  *RSYNC=*) echo "HOME=/home/fake"; echo "RSYNC=1" ;;
  "date +%s") echo "${FAKE_REMOTE_EPOCH:-$(date +%s)}" ;;
  *"date -s"*) echo "date-set" >> "$FAKE_DIR/clock_set" ;;
  *start_all_single.log*grep*) echo DONE ;;
  *"deploy_logs"*) cat > "$FAKE_DIR/uploaded.log" ;;
esac
exit 0
'''
FAKE_RSYNC = r'''#!/usr/bin/env bash
echo "$@" > "$FAKE_DIR/rsync_args"
echo ">f++++++++++ scripts/pc/deploy_jetson1.sh"
echo ">f.st...... README.md"
exit "${FAKE_RSYNC_RC:-0}"
'''
FAKE_PING = "#!/usr/bin/env bash\nexit 0\n"


@unittest.skipIf(BASH is None, "bash(Linux)が無い")
class FakeFlowTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        d = Path(self.tmp.name)
        self.bin = d / "bin"
        self.bin.mkdir()
        for name, body in (("ssh", FAKE_SSH), ("rsync", FAKE_RSYNC), ("ping", FAKE_PING)):
            p = self.bin / name
            p.write_text(body, encoding="utf-8", newline="\n")
            p.chmod(p.stat().st_mode | stat.S_IEXEC)
        self.fake = d / "fake"
        self.fake.mkdir()
        self.logs = d / "logs"

    def tearDown(self):
        self.tmp.cleanup()

    def go(self, args, env=None, stdin=subprocess.DEVNULL):
        e = {"PATH": f"{self.bin}:{os.environ['PATH']}", "FAKE_DIR": str(self.fake), "DEPLOY_POLL_SEC": "0",
             "NOKOSAHEN_DEPLOY_LOG_DIR": str(self.logs), "NOKOSAHEN_SSH_KEY": str(self.fake / "nokey"),
             "NOKOSAHEN_JETSON1": "robot@192.0.2.1"}
        e.update(env or {})
        return run_script(args, env=e, stdin=stdin)

    def cmds(self):
        return (self.fake / "ssh_cmds").read_text(encoding="utf-8")

    def test_full_flow_ok(self):
        r = self.go(["--yes"])
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        out = r.stdout
        for s in ("[OK] 配置", "[OK] ビルド", "[OK] 再起動", "[OK] ダッシュボード8090", "[OK] /wheel_ticks",
                  "[OK] ESP32の接続", "[OK] /estop_state", "[OK] 時計", "[OK] 負荷"):
            self.assertIn(s, out)
        self.assertNotIn("プロセス person_target_detector", out)  # カメラなしでは見ない
        # 秘密はrsyncに渡さない設定
        args = (self.fake / "rsync_args").read_text(encoding="utf-8")
        self.assertIn("--exclude=secrets.*", args)
        self.assertIn("--exclude=.git", args)
        self.assertIn("robot@192.0.2.1:mirs2603_nokosahen/", args)
        self.assertIn("--backup-dir=/home/fake/deploy_backup_", args)
        # 実行したsshコマンドが、stop_robot.shのパターンに一致しない(自分を止めない)
        for line in self.cmds().splitlines():
            for pat in stop_patterns():
                self.assertIsNone(re.search(pat, line), f"{pat}: {line}")
        # ログ: PC側と1台目側
        self.assertEqual(len(list(self.logs.glob("*.log"))), 1)
        self.assertTrue((self.fake / "uploaded.log").is_file())
        self.assertIn("結果", (self.fake / "uploaded.log").read_text(encoding="utf-8"))

    def test_old_clock_is_set_from_pc(self):
        r = self.go(["--yes", "--no-restart"], env={"FAKE_REMOTE_EPOCH": "1000000"})
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertTrue((self.fake / "clock_set").is_file())
        self.assertIn("sudo -n /usr/bin/date -s @", self.cmds())
        self.assertIn("[OK] 時計合わせ", r.stdout)
        # 新しい時計なら触らない
        (self.fake / "clock_set").unlink()
        self.go(["--yes", "--no-restart"])
        self.assertFalse((self.fake / "clock_set").exists())

    def test_build_failure_exits_3_without_restart(self):
        r = self.go(["--yes"], env={"FAKE_BUILD_FAIL": "1"})
        self.assertEqual(r.returncode, 3, r.stdout)
        self.assertNotIn("start_all.sh", self.cmds())

    def test_rsync_failure_exits_6(self):
        r = self.go(["--yes"], env={"FAKE_RSYNC_RC": "23"})
        self.assertEqual(r.returncode, 6, r.stdout)

    def test_no_restart_skips_start_and_verify(self):
        r = self.go(["--no-restart"])
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertNotIn("start_all.sh", self.cmds())
        self.assertNotIn("ダッシュボード8090", r.stdout)

    def test_confirmation_required_without_tty(self):
        r = self.go([])  # --yes なし、端末なし
        self.assertEqual(r.returncode, 5, r.stdout)
        self.assertNotIn("start_all.sh", self.cmds())
        self.assertIn("--yes", r.stdout)

    def test_mapping_in_progress_is_shown(self):
        r = self.go(["--yes"], env={"FAKE_MAPMODE": "mapping", "FAKE_MAPRUN": "true"})
        self.assertIn("地図作成中です", r.stdout)

    def test_verify_ng_exit_1_and_warn(self):
        r = self.go(["--yes"], env={"FAKE_LIDAR": "0", "FAKE_TICKS": "", "FAKE_LOAD": "7.5"})
        self.assertEqual(r.returncode, 1, r.stdout)
        self.assertIn("[NG] プロセス rplidar_node", r.stdout)
        self.assertIn("[NG] ESP32の接続", r.stdout)
        self.assertIn("[警告] 負荷", r.stdout)

    def test_cameras_checks_camera_processes(self):
        r = self.go(["--yes", "--cameras"])
        self.assertIn("--person-detector --rear-cameras", self.cmds())
        self.assertIn("[NG] プロセス person_target_detector", r.stdout)
        self.assertIn("[NG] プロセス rear_camera_persons", r.stdout)


if __name__ == "__main__":
    unittest.main()
