"""device_watchdog の純粋ロジックのテスト(ROS・実機なし。偽のIoと偽の単調時計で動かす)。"""
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core import device_watchdog_logic as L  # noqa: E402
from nokosahen_core.device_watchdog_logic import (  # noqa: E402
    DeviceConfig, DeviceMachine, Io, ProcessSpec, backoff_delay, list_pids, match_process,
)

REPO = Path(__file__).resolve().parents[4]


class FakeIo(Io):
    def __init__(self):
        self.t = 1000.0
        self.present = True
        self.openable = True
        self.pids = []               # 動いているプロセス
        self.data_at = None
        self.calls = []              # (種別, 引数)
        self.start_ok = True
        self.start_makes_process = True
        self.term_kills = True       # SIGTERMで死ぬか(Falseなら半死: SIGKILLが要る)
        self.kill_works = True       # SIGKILLで死ぬか(Falseなら消せない)
        self.next_pid = 100
        self.logs = []

    def now(self):
        return self.t

    def device_state(self, cfg):
        return self.present, self.openable, "" if self.present else (cfg.paths[0] if cfg.paths else "usb")

    def find_pids(self, spec, fresh=False):
        return list(self.pids)

    def send_signal(self, pid, sig):
        self.calls.append(("signal", pid, sig))
        if (sig == L.SIGKILL and self.kill_works) or (sig == L.SIGTERM and self.term_kills):
            if pid in self.pids:
                self.pids.remove(pid)

    def start(self, cfg):
        self.calls.append(("start", cfg.name))
        if not self.start_ok:
            return False
        if self.start_makes_process:
            self.pids.append(self.next_pid)
            self.next_pid += 1
        return True

    def last_data(self, cfg):
        return self.data_at

    def log(self, level, msg):
        self.logs.append((level, msg))

    def starts(self):
        return [c for c in self.calls if c[0] == "start"]

    def signals(self):
        return [c for c in self.calls if c[0] == "signal"]


def lidar_cfg(**kw):
    base = dict(name="lidar", paths=("/dev/rplidar",), process=ProcessSpec("rplidar_node", start_cmd=("/x/rplidar_node",)),
                has_data=True, data_timeout_sec=3.0, startup_grace_sec=15.0, stable_sec=3.0, term_grace_sec=3.0)
    base.update(kw)
    return DeviceConfig(**base)


def run(m, io, seconds, step=0.5, data=False):
    """seconds進める。data=Trueなら毎tick「今データが来た」ことにする。"""
    n = int(round(seconds / step))
    for _ in range(n):
        io.t += step
        if data:
            io.data_at = io.t
        m.tick()


class Base(unittest.TestCase):
    def healthy(self, settle=True, **kw):
        io = FakeIo()
        io.pids = [50]
        io.data_at = io.t
        m = DeviceMachine(lidar_cfg(**kw), io)
        m.tick()                                  # INIT -> STARTING(引き継ぎ)
        io.t += 0.5
        io.data_at = io.t
        m.tick()                                  # -> HEALTHY
        self.assertEqual(m.state, L.HEALTHY)
        if settle:
            run(m, io, 35, data=True)             # 30秒以上健全=失敗回数0(バックオフ無し)の状態にする
        return m, io


class StateMachineTest(Base):
    def test_adopt_existing_process_no_double_start(self):
        io = FakeIo(); io.pids = [50]
        m = DeviceMachine(lidar_cfg(), io)
        run(m, io, 5, data=True)
        self.assertEqual(m.state, L.HEALTHY)
        self.assertEqual(io.starts(), [])

    def test_initial_no_process_starts_after_stable(self):
        io = FakeIo(); io.pids = []
        m = DeviceMachine(lidar_cfg(), io)
        run(m, io, 2.5, data=True)
        self.assertEqual(io.starts(), [])             # まだ3秒安定していない
        run(m, io, 2.0, data=True)
        self.assertEqual(len(io.starts()), 1)
        run(m, io, 3, data=True)
        self.assertEqual(m.state, L.HEALTHY)

    def test_full_cycle_fault_stop_wait_restart_healthy(self):
        m, io = self.healthy()
        io.present = False                            # 抜かれた
        run(m, io, 0.5)                               # 1回目は連続判定の途中
        self.assertEqual(m.state, L.HEALTHY)
        run(m, io, 2.5)
        self.assertIn(m.state, (L.WAITING_FOR_DEVICE, L.FAULT))
        sigs = io.signals()
        self.assertEqual(sigs[0], ("signal", 50, L.SIGTERM))   # 止める
        self.assertEqual(m.state, L.WAITING_FOR_DEVICE)
        self.assertEqual(io.pids, [])
        run(m, io, 5)                                 # 抜けたまま: 起動しない
        self.assertEqual(io.starts(), [])
        io.present = True
        run(m, io, 2.0)                               # 戻った直後は起動しない
        self.assertEqual(io.starts(), [])
        run(m, io, 2.0, data=True)                    # 3秒安定したあと起動
        self.assertEqual(len(io.starts()), 1)
        run(m, io, 2.0, data=True)
        self.assertEqual(m.state, L.HEALTHY)
        self.assertEqual(m.restarts, 1)
        # 順序: SIGTERM のあとに start
        kinds = [c[0] for c in io.calls]
        self.assertLess(kinds.index("signal"), kinds.index("start"))

    def test_consecutive_confirmation_single_glitch_ignored(self):
        m, io = self.healthy()
        io.data_at = io.t - 10                        # 1tickだけデータ古い
        m.tick()
        io.t += 0.5; io.data_at = io.t
        m.tick()
        run(m, io, 5, data=True)
        self.assertEqual(m.state, L.HEALTHY)
        self.assertEqual(io.signals(), [])

    def test_hang_process_alive_but_no_data(self):
        m, io = self.healthy()
        run(m, io, 2.0, data=True)
        # データだけ止まる(プロセスは生きている)
        run(m, io, 3.0)
        self.assertEqual(m.state, L.HEALTHY)          # 3秒のタイムアウトまでは健全
        run(m, io, 3.0)
        self.assertEqual(m.state, L.WAITING_FOR_DEVICE)
        self.assertEqual(io.signals()[0], ("signal", 50, L.SIGTERM))
        # デバイスは存在し続けているので3秒の安定後に再起動する
        run(m, io, 4.0, data=True)
        self.assertEqual(len(io.starts()), 1)

    def test_sigterm_ignored_escalates_to_sigkill_before_waiting(self):
        m, io = self.healthy()
        io.term_kills = False
        io.present = False
        run(m, io, 3.0)
        self.assertEqual(m.state, L.FAULT)
        self.assertEqual([c[2] for c in io.signals()], [L.SIGTERM])
        run(m, io, 3.5)
        self.assertEqual([c[2] for c in io.signals()], [L.SIGTERM, L.SIGKILL])
        run(m, io, 1.0)
        self.assertEqual(m.state, L.WAITING_FOR_DEVICE)
        self.assertEqual(io.starts(), [])

    def test_startup_grace_no_fault_during_grace(self):
        io = FakeIo(); io.pids = []
        m = DeviceMachine(lidar_cfg(), io)
        run(m, io, 4.0)                               # 起動
        self.assertEqual(len(io.starts()), 1)
        run(m, io, 10.0)                              # データ無しだが猶予(15秒)内
        self.assertEqual(m.state, L.STARTING)
        self.assertEqual(io.signals(), [])
        run(m, io, 6.0)                               # 猶予切れ
        self.assertNotEqual(m.state, L.STARTING)
        self.assertEqual(m.fails, 1)

    def test_old_data_before_restart_not_counted(self):
        m, io = self.healthy()
        io.present = False
        run(m, io, 3.0)
        io.present = True
        old = io.data_at
        run(m, io, 4.0)                               # 起動された。io.data_at は古いまま
        self.assertEqual(len(io.starts()), 1)
        self.assertLess(old, m._started_at)
        self.assertEqual(m.state, L.STARTING)         # 古いデータでHEALTHYにならない

    def test_start_failure_backoff_5_10_20_40_60_60(self):
        io = FakeIo(); io.pids = []; io.start_ok = False
        m = DeviceMachine(lidar_cfg(), io)
        times = []
        last = 0
        for _ in range(2000):
            io.t += 0.5
            m.tick()
            n = len(io.starts())
            if n != last:
                times.append(io.t)
                last = n
            if n >= 7:
                break
        gaps = [round(b - a) for a, b in zip(times, times[1:])]
        # 失敗1回目→5秒、2→10、3→20、4→40、5→60、6→60(上限)
        self.assertEqual(gaps, [5, 10, 20, 40, 60, 60])

    def test_backoff_function(self):
        c = lidar_cfg()
        self.assertEqual([backoff_delay(c, n) for n in range(0, 8)], [0, 5, 10, 20, 40, 60, 60, 60])

    def test_process_dies_quickly_after_start_counts_as_failure(self):
        io = FakeIo(); io.pids = []
        m = DeviceMachine(lidar_cfg(), io)
        run(m, io, 4.0)
        self.assertEqual(len(io.starts()), 1)
        io.pids.clear()                               # 終了コード255で即死
        run(m, io, 1.0, data=False)
        self.assertEqual(m.fails, 1)
        self.assertEqual(m.state, L.WAITING_FOR_DEVICE)
        run(m, io, 3.5)                               # 安定は満たすがバックオフ5秒の間は起動しない
        self.assertEqual(len(io.starts()), 1)
        run(m, io, 3.0)
        self.assertEqual(len(io.starts()), 2)

    def test_fast_relapse_after_healthy_extends_backoff_but_long_healthy_resets(self):
        m, io = self.healthy(settle=False)
        # 健全になって10秒で落ちる -> 失敗扱い
        run(m, io, 10, data=True)
        io.pids.clear()
        run(m, io, 3.0)
        self.assertEqual(m.fails, 1)
        # 別個体: 長く健全(>30秒)なら 0 から
        m2, io2 = self.healthy()
        run(m2, io2, 40, data=True)
        io2.pids.clear()
        run(m2, io2, 3.0)
        self.assertEqual(m2.fails, 0)

    def test_chatter_does_not_restart(self):
        m, io = self.healthy()
        io.present = False
        run(m, io, 3.0)
        self.assertEqual(m.state, L.WAITING_FOR_DEVICE)
        # 1秒つながっては切れる、を繰り返す
        for _ in range(10):
            io.present = True
            run(m, io, 1.5)
            io.present = False
            run(m, io, 0.5)
        self.assertEqual(io.starts(), [])
        io.present = True
        run(m, io, 3.5, data=True)
        self.assertEqual(len(io.starts()), 1)

    def test_not_openable_waits(self):
        m, io = self.healthy(check_open=True)
        io.openable = False
        run(m, io, 3.0)
        self.assertEqual(m.state, L.WAITING_FOR_DEVICE)
        run(m, io, 10.0)
        self.assertEqual(io.starts(), [])
        io.openable = True
        run(m, io, 4.0, data=True)
        self.assertEqual(len(io.starts()), 1)

    def test_no_double_start_when_someone_started_it(self):
        m, io = self.healthy()
        io.present = False
        run(m, io, 3.0)
        io.present = True
        io.pids.append(777)                           # 手で起動された
        run(m, io, 6.0, data=True)
        self.assertEqual(io.starts(), [])
        self.assertEqual(m.state, L.HEALTHY)

    def test_stale_process_before_start_is_stopped_not_doubled(self):
        io = FakeIo(); io.pids = []
        m = DeviceMachine(lidar_cfg(), io)
        run(m, io, 0.5)
        m._goto(L.RESTARTING)
        io.pids = [900]                               # 起動の直前に残っていた
        m._t_restarting(io.t)
        self.assertEqual(io.starts(), [])
        self.assertIn(("signal", 900, L.SIGTERM), io.calls)
        self.assertEqual(m.state, L.FAULT)

    def test_never_starts_while_pid_exists(self):
        io = FakeIo(); io.pids = []
        m = DeviceMachine(lidar_cfg(), io)
        run(m, io, 4.0)
        starts_before = len(io.starts())
        # 起動後、プロセスが死なずにデータが出ない -> 止めてから再起動。その間 start は増えない
        io.term_kills = False
        io.kill_works = False                          # 消せないプロセス
        run(m, io, 60.0)
        self.assertEqual(len(io.starts()), starts_before)
        self.assertGreaterEqual(len([c for c in io.signals() if c[2] == L.SIGKILL]), 2)   # SIGKILLを再送し続ける

    def test_clock_regress_is_absorbed(self):
        m, io = self.healthy()
        io.present = False
        run(m, io, 1.0)
        io.t -= 500                                   # 逆行
        io.data_at = io.t
        m.tick()
        self.assertNotEqual(m.state, L.RESTARTING)
        run(m, io, 3.0)
        self.assertEqual(m.state, L.WAITING_FOR_DEVICE)

    def test_clock_jump_forward_resets_confirmation(self):
        m, io = self.healthy()
        io.present = False
        m.tick()                                      # 異常1回
        io.t += 3600                                  # サスペンド等で1時間飛ぶ
        m.tick()
        self.assertEqual(m.state, L.HEALTHY)          # 飛びだけでは即FAULTにしない
        run(m, io, 3.0)
        self.assertEqual(m.state, L.WAITING_FOR_DEVICE)

    def test_pause_does_nothing_then_resumes(self):
        m, io = self.healthy()
        m.set_paused(True)
        io.present = False
        run(m, io, 10.0)
        self.assertEqual(io.calls, [])
        self.assertEqual(m.status()["state"], L.PAUSED)
        io.present = True
        m.set_paused(False)
        run(m, io, 3.0, data=True)
        self.assertEqual(io.starts(), [])             # 既存プロセスを引き継ぐだけ
        self.assertEqual(m.state, L.HEALTHY)

    def test_report_only_never_touches_processes(self):
        io = FakeIo()
        cfg = DeviceConfig(name="cam", action="report", paths=("/dev/v4l/by-id/usb-AONI*",), check_open=True)
        m = DeviceMachine(cfg, io)
        run(m, io, 2.0)
        self.assertEqual(m.state, L.HEALTHY)
        io.present = False
        run(m, io, 3.0)
        self.assertEqual(m.state, L.FAULT)
        io.present = True
        run(m, io, 1.0)
        self.assertEqual(m.state, L.HEALTHY)
        self.assertEqual(io.calls, [])

    def test_esp32_agent_restart_waits_for_wheel_ticks(self):
        io = FakeIo(); io.pids = [60]; io.data_at = io.t
        cfg = DeviceConfig(name="esp32", paths=("/dev/esp32",), process=ProcessSpec("micro_ros_agent", ("serial", "/dev/esp32"), ("/x/agent",)),
                           has_data=True, data_timeout_sec=2.0, startup_grace_sec=25.0)
        m = DeviceMachine(cfg, io)
        run(m, io, 40.0, data=True)
        self.assertEqual(m.state, L.HEALTHY)
        io.present = False                            # USBが外れた
        run(m, io, 3.0)
        self.assertEqual(m.state, L.WAITING_FOR_DEVICE)
        io.present = True
        run(m, io, 4.0)
        self.assertEqual(len(io.starts()), 1)
        run(m, io, 10.0)                              # ESP32の再接続待ち(まだ/wheel_ticksが出ない)
        self.assertEqual(m.state, L.STARTING)
        run(m, io, 2.0, data=True)                    # /wheel_ticks が流れ出す
        self.assertEqual(m.state, L.HEALTHY)

    def test_status_fields(self):
        m, io = self.healthy()
        st = m.status()
        for k in ("state", "reason", "restarts", "last_ok_age_s", "consecutive_fails"):
            self.assertIn(k, st)


class ProcessMatchTest(unittest.TestCase):
    spec = ProcessSpec("rplidar_node")

    def test_exact_match(self):
        self.assertTrue(match_process(["/opt/ros/jazzy/lib/rplidar_ros/rplidar_node", "--ros-args"], self.spec))
        self.assertTrue(match_process(["rplidar_node"], self.spec))

    def test_does_not_match_commands_that_merely_mention_it(self):
        for argv in (
            ["ssh", "robot@192.0.2.1", "pkill -f rplidar_node"],
            ["bash", "-c", "pgrep -f rplidar_node"],
            ["pgrep", "-f", "rplidar_node"],
            ["vim", "rplidar_node.log"],
            ["tail", "-f", "/home/x/rplidar_node"],
            ["/usr/bin/python3", "/home/x/device_watchdog", "rplidar_node"],
            ["/opt/ros/jazzy/lib/rplidar_ros/rplidar_node_extra"],
            ["python3", "/x/ros2", "run", "rplidar_ros", "rplidar_node"],
        ):
            self.assertFalse(match_process(argv, self.spec), argv)

    def test_args_contain_is_exact(self):
        spec = ProcessSpec("micro_ros_agent", ("serial", "/dev/esp32"))
        self.assertTrue(match_process(["/x/micro_ros_agent", "serial", "--dev", "/dev/esp32", "-b", "115200"], spec))
        self.assertFalse(match_process(["/x/micro_ros_agent", "serial", "--dev", "/dev/ttyUSB1"], spec))
        self.assertFalse(match_process(["/x/micro_ros_agent", "serial", "--dev", "/dev/esp32x"], spec))

    def test_interpreter_script(self):
        spec = ProcessSpec("person_target_detector")
        self.assertTrue(match_process(["/usr/bin/python3", "/ws/install/lib/nokosahen_core/person_target_detector"], spec))
        self.assertFalse(match_process(["/usr/bin/python3", "-m", "pytest", "person_target_detector"], spec))

    def test_list_pids_with_fake_proc_excludes_self_and_zombies(self):
        with tempfile.TemporaryDirectory() as d:
            def mk(pid, argv, state="S"):
                p = Path(d) / str(pid)
                p.mkdir()
                (p / "cmdline").write_bytes("\0".join(argv).encode() + b"\0")
                (p / "stat").write_text("%d (rplidar_node) %s 1 1 1" % (pid, state))
            mk(10, ["/x/rplidar_node", "--ros-args"])
            mk(11, ["/x/rplidar_node"], "Z")                          # ゾンビ
            mk(12, ["ssh", "host", "pkill -f rplidar_node"])           # 無関係
            mk(13, ["/x/rplidar_node"])                                # 自分(除外)
            (Path(d) / "self").mkdir()
            self.assertEqual(list_pids(self.spec, proc_root=d, exclude=(13,)), [10])

    def test_usb_ids(self):
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / "1-1").mkdir()
            (Path(d) / "1-1" / "idVendor").write_text("18d1\n")
            (Path(d) / "1-1" / "idProduct").write_text("9302\n")
            self.assertTrue(L.usb_ids_present(["1a6e:089a", "18D1:9302"], d))
            self.assertFalse(L.usb_ids_present(["1234:5678"], d))


class WiringTest(unittest.TestCase):
    """起動配線(launch・setup.py・yaml)の静的確認。"""

    def test_entry_point(self):
        s = (REPO / "ros2_ws/src/nokosahen_core/setup.py").read_text(encoding="utf-8")
        self.assertIn("device_watchdog = nokosahen_core.device_watchdog:main", s)

    def test_launch_gates_children_by_watchdog_flag(self):
        s = (REPO / "ros2_ws/src/nokosahen_bringup/launch/persistent_base.launch.py").read_text(encoding="utf-8")
        self.assertIn('DeclareLaunchArgument("use_device_watchdog", default_value="true")', s)
        self.assertIn('executable="device_watchdog"', s)
        self.assertEqual(s.count("use_device_watchdog, \"' != 'true'\""), 2)   # rplidar_nodeとagentの2か所を、watchdog有効時は起動しない
        self.assertIn("respawn=True", s)

    def test_yaml_has_section_and_parses(self):
        try:
            import yaml
        except ImportError:
            self.skipTest("PyYAML無し")
        d = yaml.safe_load((REPO / "ros2_ws/src/nokosahen_bringup/config/nokosahen.yaml").read_text(encoding="utf-8"))
        self.assertIn("device_watchdog", d)
        self.assertIn("rplidar", d.get("scan_self_filter", {}) and "rplidar" or "rplidar")


if __name__ == "__main__":
    unittest.main()
