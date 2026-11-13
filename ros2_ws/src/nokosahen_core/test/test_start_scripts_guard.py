"""start_persistent_base*.sh の二重起動ガード(overlap_audit_2026-10-23 の B5)のテスト。
偽の `ros2`(sleepするだけ)を使って、実際にスクリプトを動かして確かめる(ROS・ロボットは起動しない)。
- 1つ目が動いている間、2つ目は拒否される(終了コード1、動いているプロセスと止め方を表示)
- 1つ目が終われば、2つ目は起動できる
- NOKOSAHEN_ALLOW_MULTI_BASE=1 で回避できる
- 監視ループ(run_persistent_base_supervised.sh)が動いている間も拒否される(同じロックファイル)
- すべての start_persistent_base*.sh が同じガードを持つ(静的検査)
Linux(WSL)のbash・flockが必要。無ければskip(静的検査は常に実行)。
"""
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
SCRIPTS = REPO / "scripts"
START_SCRIPTS = sorted(SCRIPTS.glob("start_persistent_base*.sh"))
HAVE = (not sys.platform.startswith("win")) and all(shutil.which(c) for c in ("bash", "flock", "sleep"))


def wait_for(pred, timeout=15.0, step=0.1):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(step)
    return False


class StaticGuardTest(unittest.TestCase):
    def test_all_start_scripts_found(self):
        names = {p.name for p in START_SCRIPTS}
        self.assertEqual(names, {"start_persistent_base.sh", "start_persistent_base_app.sh",
                                 "start_persistent_base_control.sh", "start_persistent_base_rc_only.sh"})

    def test_every_start_script_has_the_same_guard_before_launch(self):
        for p in START_SCRIPTS:
            t = p.read_text(encoding="utf-8")
            self.assertIn("base_guard.sh", t, p.name)
            self.assertIn("nokosahen_base_guard", t, p.name)
            self.assertLess(t.index("nokosahen_base_guard"), t.index("ros2 launch"), f"{p.name}: ガードがlaunchより後ろ")

    def test_guard_uses_supervisor_lock_and_has_escape_and_stop_hints(self):
        t = (SCRIPTS / "base_guard.sh").read_text(encoding="utf-8")
        self.assertIn("NOKOSAHEN_SUPERVISOR_LOCK", t)
        self.assertIn("/tmp/nokosahen_supervisor.lock", t)  # 監視ループと同じ既定のロックファイル
        self.assertIn("flock -n 9", t)
        self.assertIn("NOKOSAHEN_ALLOW_MULTI_BASE", t)
        self.assertIn("stop_robot.sh", t)
        self.assertIn("stop_everything.sh", t)
        self.assertIn("exit 1", t)

    def test_start_all_does_not_call_start_scripts(self):
        # start_all.sh は先に全停止して監視ループを起動する。start_persistent_base*.sh を呼ぶと、ガードと矛盾し得る
        for rel in ("start_all.sh", "autostart/boot_start.sh", "start_all_with_clock.sh"):
            f = SCRIPTS / rel
            if f.exists():
                self.assertNotIn("start_persistent_base", f.read_text(encoding="utf-8"), rel)

    def test_scripts_syntax_ok(self):
        if not shutil.which("bash"):
            self.skipTest("bashが無い")
        for p in START_SCRIPTS + [SCRIPTS / "base_guard.sh"]:
            # CRLFでチェックアウトされた作業ツリーでも調べられるよう、改行をLFにしたものを渡す
            src = p.read_bytes().replace(b"\r\n", b"\n")
            r = subprocess.run(["bash", "-n"], input=src, capture_output=True)
            self.assertEqual(r.returncode, 0, p.name + r.stderr.decode("utf-8", "replace"))


@unittest.skipUnless(HAVE, "Linuxのbash/flockが無い")
class GuardBehaviorTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        d = Path(self.tmp.name)
        (d / "bin").mkdir()
        fake = d / "bin" / "ros2"
        fake.write_text("#!/bin/sh\nexec sleep 300\n", encoding="utf-8")  # ros2 launch の代わり(長く動き続ける)
        fake.chmod(0o755)
        # スクリプトのコピー(改行をLFにする。CRLFでチェックアウトされた作業ツリーでも動かすため)
        self.scripts = d / "mirs2603_nokosahen" / "scripts"
        self.scripts.mkdir(parents=True)
        # 必要なものだけコピーする(sync_clocks.sh等、ネットワークへ出るものは置かない)
        for src in START_SCRIPTS + [SCRIPTS / n for n in ("base_guard.sh", "dds_wired_env.sh",
                                                           "run_persistent_base_supervised.sh")]:
            (self.scripts / src.name).write_bytes(src.read_bytes().replace(b"\r\n", b"\n"))
        (d / "mirs2603_nokosahen" / "config").mkdir()
        for src in (REPO / "config").glob("*"):
            if src.is_file():
                (d / "mirs2603_nokosahen" / "config" / src.name).write_bytes(src.read_bytes().replace(b"\r\n", b"\n"))
        for rel in ("lidar_src_build/install", "microros_ws/install"):
            (d / rel).mkdir(parents=True)
            # 実環境にROSが入っていると、/opt/ros の setup.bash が本物の ros2 をPATHの先頭に置く。
            # 偽の ros2 が必ず勝つよう、source される側でPATHの先頭に置き直す(本物のlaunchは絶対に起動しない)
            (d / rel / "setup.bash").write_text(f'export PATH="{d / "bin"}:$PATH"\n', encoding="utf-8")
        self.home = d
        self.env = dict(os.environ, HOME=str(d), PATH=str(d / "bin") + os.pathsep + os.environ["PATH"],
                        NOKOSAHEN_SUPERVISOR_LOCK=str(d / "sup.lock"), NOKOSAHEN_DDS_WIRED="0")
        self.env.pop("NOKOSAHEN_ALLOW_MULTI_BASE", None)
        self.procs = []

    def tearDown(self):
        for p in self.procs:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except Exception:  # noqa: BLE001
                pass
            try:
                p.wait(timeout=5)
            except Exception:  # noqa: BLE001
                pass

    def run_script(self, name, env=None, timeout=30):
        return subprocess.run(["bash", str(self.scripts / name)], env=env or self.env, capture_output=True,
                              text=True, timeout=timeout, start_new_session=True)

    def sleepers(self):
        # このテストが起動した偽ros2(sleep 300)の数: 環境のHOMEを持つプロセスだけ数える
        n = 0
        for pid in os.listdir("/proc"):
            if not pid.isdigit():
                continue
            try:
                cmd = Path(f"/proc/{pid}/cmdline").read_bytes()
                env = Path(f"/proc/{pid}/environ").read_bytes()
            except Exception:  # noqa: BLE001
                continue
            if cmd.startswith(b"sleep\x00300") and f"HOME={self.home}".encode() in env:
                n += 1
        return n

    def kill_sleepers(self):
        for pid in os.listdir("/proc"):
            if not pid.isdigit():
                continue
            try:
                cmd = Path(f"/proc/{pid}/cmdline").read_bytes()
                env = Path(f"/proc/{pid}/environ").read_bytes()
                if cmd.startswith(b"sleep\x00300") and f"HOME={self.home}".encode() in env:
                    os.kill(int(pid), signal.SIGKILL)
            except Exception:  # noqa: BLE001
                continue

    def addCleanupSleepers(self):
        self.addCleanup(self.kill_sleepers)

    def test_second_start_is_refused_while_first_runs_then_allowed_after(self):
        self.addCleanupSleepers()
        r1 = self.run_script("start_persistent_base.sh")
        self.assertEqual(r1.returncode, 0, r1.stdout + r1.stderr)
        self.assertTrue(wait_for(lambda: self.sleepers() == 1), "1つ目(偽launch)が動かない")
        for name in [p.name for p in START_SCRIPTS]:
            r = self.run_script(name)
            out = r.stdout + r.stderr
            self.assertEqual(r.returncode, 1, f"{name}: {out}")
            self.assertIn("起動しません", out, name)
            self.assertIn("stop_robot.sh", out, name)
            self.assertIn("stop_everything.sh", out, name)
            self.assertIn("NOKOSAHEN_ALLOW_MULTI_BASE", out, name)
        self.assertEqual(self.sleepers(), 1, "拒否されたのに、さらに起動している")
        # 1つ目が終わったら、2つ目が起動できる
        self.kill_sleepers()
        self.assertTrue(wait_for(lambda: self.sleepers() == 0))
        r2 = self.run_script("start_persistent_base_rc_only.sh")
        self.assertEqual(r2.returncode, 0, r2.stdout + r2.stderr)
        self.assertTrue(wait_for(lambda: self.sleepers() == 1), "1つ目が終わっても2つ目が起動できない")

    def test_allow_multi_base_env_bypasses(self):
        self.addCleanupSleepers()
        self.run_script("start_persistent_base.sh")
        self.assertTrue(wait_for(lambda: self.sleepers() == 1))
        env = dict(self.env, NOKOSAHEN_ALLOW_MULTI_BASE="1")
        r = self.run_script("start_persistent_base_rc_only.sh", env=env)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertTrue(wait_for(lambda: self.sleepers() == 2), "回避したのに起動していない")
        self.assertIn("NOKOSAHEN_ALLOW_MULTI_BASE=1", r.stderr)

    def test_refused_while_supervisor_loop_holds_the_lock(self):
        self.addCleanupSleepers()
        sup = subprocess.Popen(["bash", str(self.scripts / "run_persistent_base_supervised.sh"), "rc_only"], env=self.env,
                               start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.procs.append(sup)
        log = self.home / ".nokosahen_supervisor.log"
        self.assertTrue(wait_for(lambda: log.exists() and "persistent_base起動" in log.read_text(encoding="utf-8")),
                        "監視ループが起動しない")
        r = self.run_script("start_persistent_base.sh")
        self.assertEqual(r.returncode, 1, r.stdout + r.stderr)
        self.assertIn("run_persistent_base_supervised.sh", r.stderr)  # 動いているプロセスとして表示される
        self.assertIn("stop_everything.sh", r.stderr)
        self.assertEqual(self.sleepers(), 1)

    def test_guard_does_not_leak_lock_to_nothing_when_refused(self):
        # ロックを持った状態(監視ループ相当)でも、拒否したスクリプト自身は何も起動しない
        self.addCleanupSleepers()
        holder = subprocess.Popen(["flock", str(self.home / "sup.lock"), "sleep", "60"], env=self.env,
                                  start_new_session=True)
        self.procs.append(holder)
        time.sleep(0.5)
        r = self.run_script("start_persistent_base_control.sh")
        self.assertEqual(r.returncode, 1, r.stdout + r.stderr)
        self.assertIn("排他ロック", r.stderr)
        self.assertEqual(self.sleepers(), 0)


if __name__ == "__main__":
    unittest.main(verbosity=1)
