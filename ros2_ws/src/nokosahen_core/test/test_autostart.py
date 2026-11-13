"""自動起動(scripts/autostart/)の静的・単体テスト(2026-10-19)。実機・systemd・sudoは使わない。

- シェルスクリプトの構文(bash -n)
- boot_start.sh は manual 以外を拒否する(自動起動で自律行動 full を絶対に起動しない)
- systemdユニットの雛形: 置き換え記号、Restart=no(止めたあとに勝手に復活しない)、WantedBy
- install_autostart.sh は、root以外・不明な対象・不明な引数を拒否する
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
AUTO = REPO / "scripts" / "autostart"
BASH = shutil.which("bash") if not sys.platform.startswith("win") else None


def run(args, **kw):
    env = dict(os.environ)
    env.update(kw.pop("env", {}))
    return subprocess.run([BASH] + args, capture_output=True, text=True, timeout=30, env=env, **kw)


class FilesTest(unittest.TestCase):
    def test_files_exist(self):
        for n in ("boot_start.sh", "install_autostart.sh", "check_pi_autostart.sh",
                  "nokosahen-robot.service.in", "nokosahen-voice.service.in"):
            self.assertTrue((AUTO / n).is_file(), n)

    @unittest.skipIf(BASH is None, "bash(Linux)が無い")
    def test_bash_syntax(self):
        for n in ("boot_start.sh", "install_autostart.sh", "check_pi_autostart.sh"):
            r = run(["-n", str(AUTO / n)])
            self.assertEqual(r.returncode, 0, f"{n}: {r.stderr}")


class UnitTemplateTest(unittest.TestCase):
    def read(self, n):
        return (AUTO / n).read_text(encoding="utf-8")

    def test_robot_unit(self):
        t = self.read("nokosahen-robot.service.in")
        self.assertIn("@USER@", t)
        self.assertIn("@HOME@", t)
        self.assertIn("Restart=no", t, "止めたあとに、systemdが勝手に復活させない")
        self.assertIn("boot_start.sh manual", t, "自動起動は manual だけ")
        self.assertNotIn("full", t.split("ExecStart=", 1)[1].splitlines()[0])
        self.assertIn("WantedBy=multi-user.target", t)
        self.assertIn("SuccessExitStatus=143", t)
        # shutdown_all.sh は同じcgroupに入る。control-group だと、監視ループを止めた瞬間に自分自身も止められる
        self.assertIn("KillMode=process", t)
        active = [ln.strip() for ln in t.splitlines() if ln.strip() and not ln.strip().startswith("#")]
        self.assertNotIn("KillMode=control-group", active)

    def test_voice_unit(self):
        t = self.read("nokosahen-voice.service.in")
        self.assertIn("start_voice_stack.sh", t)
        self.assertIn("Type=oneshot", t)
        self.assertIn("RemainAfterExit=yes", t)
        self.assertIn("KillMode=process", t, "停止時に、切り離した常駐プロセスを道連れにしない")
        self.assertIn("WantedBy=multi-user.target", t)

    def test_rendering_replaces_all_placeholders(self):
        for n in ("nokosahen-robot.service.in", "nokosahen-voice.service.in"):
            out = self.read(n).replace("@USER@", "u").replace("@HOME@", "/home/u")
            self.assertIsNone(re.search(r"@[A-Z]+@", out), n)


@unittest.skipIf(BASH is None, "bash(Linux)が無い")
class ScriptBehaviorTest(unittest.TestCase):
    def test_boot_start_refuses_non_manual(self):
        with tempfile.TemporaryDirectory() as home:
            for mode in ("full", "nav_test", "rc_only", "FULL"):  # 引数なし/空は既定のmanualで起動するのでテストしない
                r = run([str(AUTO / "boot_start.sh"), mode], env={"HOME": home})
                self.assertEqual(r.returncode, 2, (mode, r.stdout, r.stderr))
                self.assertIn("manual", r.stderr)
            # 何も起動していない(ログも作られない)
            self.assertFalse((Path(home) / ".nokosahen_boot.log").exists())

    def test_install_refuses_bad_usage(self):
        if os.geteuid() == 0:
            self.skipTest("rootで実行されている")
        r = run([str(AUTO / "install_autostart.sh"), "jetson9"])
        self.assertEqual(r.returncode, 2, r.stderr)
        r = run([str(AUTO / "install_autostart.sh"), "jetson1", "--bogus"])
        self.assertEqual(r.returncode, 2, r.stderr)
        r = run([str(AUTO / "install_autostart.sh"), "jetson1"])
        self.assertEqual(r.returncode, 1, "root以外は拒否")
        self.assertIn("sudo", r.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=1)
