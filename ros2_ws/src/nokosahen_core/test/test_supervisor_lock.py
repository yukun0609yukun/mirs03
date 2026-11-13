"""run_persistent_base_supervised.sh の二重起動防止(排他ロック)のテスト(2026-10-21、全体レビューの指摘)。
偽の `ros2`(sleepするだけ)を使って、実際にスクリプトを動かして確かめる(ROS・ロボットは使わない)。
- 2つ目の監視ループは、何もせず終了コード0で終わる(ROS一式が二重に起動しない)
- 監視ループが落ちたあと、残った子プロセスがロックを持ち続けず、新しい監視ループが起動できる
Linux(WSL)のbash・flockが必要。無ければskip。
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
SCRIPT = REPO / "scripts" / "run_persistent_base_supervised.sh"
HAVE = (not sys.platform.startswith("win")) and all(shutil.which(c) for c in ("bash", "flock", "sleep"))


def wait_for(pred, timeout=15.0, step=0.1):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(step)
    return False


@unittest.skipUnless(HAVE, "Linuxのbash/flockが無い")
class SupervisorLockTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        d = Path(self.tmp.name)
        (d / "bin").mkdir()
        fake = d / "bin" / "ros2"
        fake.write_text("#!/bin/sh\nexec sleep 300\n", encoding="utf-8")  # ros2 launch の代わり(長く動き続ける)
        fake.chmod(0o755)
        self.env = dict(os.environ, HOME=str(d), PATH=str(d / "bin") + os.pathsep + os.environ["PATH"],
                        NOKOSAHEN_SUPERVISOR_LOCK=str(d / "sup.lock"))
        self.log = d / ".nokosahen_supervisor.log"
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

    def start(self):
        p = subprocess.Popen(["bash", str(SCRIPT), "rc_only"], env=self.env, start_new_session=True,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.procs.append(p)
        return p

    def log_text(self):
        return self.log.read_text(encoding="utf-8") if self.log.exists() else ""

    def test_second_supervisor_does_not_start(self):
        a = self.start()
        self.assertTrue(wait_for(lambda: "persistent_base起動" in self.log_text()), "1つ目が起動しない")
        r = subprocess.run(["bash", str(SCRIPT), "rc_only"], env=self.env, capture_output=True, text=True, timeout=30)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn("すでに監視ループが動いています", r.stdout)
        self.assertIn("すでに監視ループが動いているため、起動しません", self.log_text())
        self.assertEqual(self.log_text().count("persistent_base起動"), 1, "ROS一式が二重に起動していない")
        self.assertIsNone(a.poll(), "1つ目は動き続ける")

    def test_new_supervisor_can_start_after_the_first_dies_even_if_children_remain(self):
        a = self.start()
        self.assertTrue(wait_for(lambda: self.log_text().count("persistent_base起動") == 1))
        os.kill(a.pid, signal.SIGKILL)  # 監視ループだけが落ちる(子のsleepは残る)
        a.wait(timeout=5)
        c = self.start()
        self.assertTrue(wait_for(lambda: self.log_text().count("persistent_base起動") == 2),
                        "子プロセスがロックを持ち続けて、新しい監視ループが起動できない")
        self.assertIsNone(c.poll())

    def test_script_syntax_and_lock_present(self):
        r = subprocess.run(["bash", "-n", str(SCRIPT)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)
        t = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("flock -n 9", t)
        self.assertIn("9>&-", t)


if __name__ == "__main__":
    unittest.main(verbosity=1)
