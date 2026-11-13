"""scripts/stop_robot.sh のテスト(2026-10-21、全体レビューの指摘)。ダミーのプロセスを作って実際に確かめる(ROS・ロボットは使わない)。

- 起動引数の文字(use_behavior_director:=false など)に一致して、管理画面を含む親プロセスを巻き込まない
- 実行ファイルのパス(.../lib/nokosahen_core/behavior_director)のものは止まる
- 止まりきらないものがあれば終了コード1(以前は常に0)
Linux(WSL)のbash・pkill・pgrepが必要。無ければskip。
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
SCRIPT = REPO / "scripts" / "stop_robot.sh"
HAVE = (not sys.platform.startswith("win")) and all(shutil.which(c) for c in ("bash", "pkill", "pgrep", "sleep"))


def spawn(name):
    """argv[0]を name に差し替えた sleep を起動する(pgrep -f / pkill -f は、この文字列に一致する)。"""
    return subprocess.Popen(["bash", "-c", f"exec -a '{name}' sleep 120"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def alive(p):
    return p.poll() is None


@unittest.skipUnless(HAVE, "Linuxのbash/pkill/pgrepが無い")
class StopRobotTest(unittest.TestCase):
    def setUp(self):
        self.procs = []

    def tearDown(self):
        for p in self.procs:
            try:
                p.kill()
                p.wait(timeout=5)
            except Exception:  # noqa: BLE001
                pass

    def run_script(self, *args, env=None):
        e = dict(os.environ)
        if env:
            e.update(env)
        return subprocess.run(["bash", str(SCRIPT), *args], capture_output=True, text=True, timeout=120, env=e)

    def test_launch_args_do_not_match_node_patterns(self):
        # persistent_base の起動コマンド(引数に use_behavior_director:= 等を含む)は、--keep-dashboard のとき残る
        launch = spawn("ros2 launch nokosahen_bringup persistent_base.launch.py use_behavior_director:=false "
                       "use_person_target_detector:=true use_voice_node:=true use_survey_monitor:=false use_nav2:=true")
        # 引数の文字だけが同じ別のプロセスは、通常の停止でも巻き込まない(直前が「_」なので一致しない)
        argonly = spawn("something use_behavior_director:=false use_person_target_detector:=true use_voice_node:=true")
        # 実行ファイルのパス(本物のノード)は止まる
        node = spawn("/home/robot/ros2_ws/install/nokosahen_core/lib/nokosahen_core/behavior_director")
        node2 = spawn("/opt/ros/jazzy/lib/nav2_controller/controller_server --ros-args -r __node:=nav2_x")
        self.procs += [launch, argonly, node, node2]
        time.sleep(1.0)
        self.assertTrue(all(alive(p) for p in self.procs))

        r = self.run_script("--keep-dashboard")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        time.sleep(0.5)
        self.assertTrue(alive(launch), "管理画面を含む起動の親プロセスは、--keep-dashboard で残る")
        self.assertTrue(alive(argonly), "起動引数の文字だけが同じプロセスは巻き込まない")
        self.assertFalse(alive(node), "本物のノード(実行ファイルのパス)は止まる")
        self.assertFalse(alive(node2), "nav2_ のノードは止まる")

    def test_without_keep_dashboard_launch_is_stopped_too(self):
        launch = spawn("ros2 launch nokosahen_bringup persistent_base.launch.py use_behavior_director:=false")
        self.procs.append(launch)
        time.sleep(0.5)
        r = self.run_script()
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        time.sleep(0.5)
        self.assertFalse(alive(launch))

    def test_exit_code_is_nonzero_when_something_remains(self):
        node = spawn("/x/lib/nokosahen_core/behavior_director")
        self.procs.append(node)
        time.sleep(0.5)
        # pkillを何もしない偽物に差し替える → 止まらず残る → 終了コード1と「残っています」
        with tempfile.TemporaryDirectory() as d:
            fake = Path(d) / "pkill"
            fake.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
            fake.chmod(0o755)
            r = self.run_script(env={"PATH": d + os.pathsep + os.environ["PATH"]})
        self.assertEqual(r.returncode, 1, r.stdout + r.stderr)
        self.assertIn("残っています: behavior_director", r.stdout)
        self.assertTrue(alive(node))

    def test_editor_or_tail_of_node_file_is_not_stopped(self):
        # 【2026-11-05】`vim .../full_dashboard_node.py` や `tail -f .../behavior_director.log` は止めない(直後が空白/行末のものだけ一致)
        vim = spawn("vim /home/x/nokosahen_core/full_dashboard_node.py")
        tail = spawn("tail -f /home/x/behavior_director.log")
        node = spawn("/home/x/lib/nokosahen_core/full_dashboard --ros-args")
        self.procs += [vim, tail, node]
        time.sleep(0.5)
        r = self.run_script()
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        time.sleep(0.5)
        self.assertTrue(alive(vim))
        self.assertTrue(alive(tail))
        self.assertFalse(alive(node), "本物のノードは止まる")

    def test_device_watchdog_stopped_first_and_pause_file(self):
        # 【2026-11-06】device_watchdog は最初に止める。一時停止ファイルは、通常の停止では最後に消し、--keep-dashboard では残す
        with tempfile.TemporaryDirectory() as d:
            pause = Path(d) / "pause"
            env = {"NOKOSAHEN_DW_PAUSE_FILE": str(pause)}
            wd = spawn("/ws/install/nokosahen_core/lib/nokosahen_core/device_watchdog --ros-args -r __node:=device_watchdog")
            other = spawn("vim /home/x/device_watchdog.log")      # 名前を含むだけのものは止めない
            launch = spawn("ros2 launch nokosahen_bringup persistent_base.launch.py use_device_watchdog:=true")
            self.procs += [wd, other, launch]
            time.sleep(0.5)
            r = self.run_script("--keep-dashboard", env=env)
            self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
            time.sleep(0.5)
            self.assertFalse(alive(wd))
            self.assertTrue(alive(other))
            self.assertTrue(alive(launch), "launchは残る(use_device_watchdog:=true の文字では巻き込まない)")
            self.assertTrue(pause.exists(), "launchが残るので、respawnした watchdog が再起動しないよう、ファイルを残す")
            r = self.run_script(env=env)
            self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
            self.assertFalse(pause.exists(), "全部止めたあとは消す")

    def test_script_has_no_unanchored_node_patterns(self):
        t = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("(^|[ /])", t)
        self.assertIn("exit \"$LEFT\"", t)
        for name in ("survey_monitor", "announce_node", "stage1_dashboard"):
            self.assertIn(f'"{name}"', t)


if __name__ == "__main__":
    unittest.main(verbosity=1)
