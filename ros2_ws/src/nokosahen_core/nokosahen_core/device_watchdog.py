"""機器監視ノード device_watchdog(docs/device_watchdog.md)。

ESP32(micro-ROS agent)・LiDAR(rplidar_node)・カメラ・Coral を監視し、
  デバイスの存在 / プロセスの存在 / データ(トピック)の流れ
のどれかが続けて異常なら、managed な機器(agent, rplidar_node)のプロセスを
SIGTERM→SIGKILL で止め、デバイスが「安定して戻った」ことを確認してから自動で起動し直す。
カメラ・Coral は見せるだけ(止めない、起動しない)。

車輪へ指令は出さない(cmd_vel を一切publishしない)。判定ロジックは device_watchdog_logic.py。
"""
from __future__ import annotations

import datetime
import json
import os
import shlex
import subprocess
import sys
import time
from typing import Dict, List, Optional, Tuple

from nokosahen_core.device_watchdog_logic import (
    DeviceConfig, DeviceMachine, Io, ProcessSpec, SIGKILL, expand_paths, list_pids, pid_matches, usb_ids_present,
)

# 機器ごとの既定値。nokosahen.yaml の device_watchdog セクション(<機器名>.<項目>)で上書きできる。
# 値の理由は docs/device_watchdog.md。リストは扱いやすいようカンマ区切りの文字列にしてある。
_FIELD_DEFAULTS = {
    "enabled": True, "action": "restart", "paths": "", "usb_ids": "", "check_open": False,
    "process": "", "process_args": "", "start": "", "baud": 0,
    "topic": "", "data_timeout_sec": 3.0, "startup_grace_sec": 15.0,
    "fault_confirm_count": 3, "fault_confirm_sec": 2.0, "stable_sec": 3.0,
}

_DEVICE_DEFAULTS: Dict[str, dict] = {
    # ESP32: /nokosahen/noise_guard_counts はファームが制御タイマー(50 Hz)の中で1秒に1回publishする診断トピック(main.cpp)。
    # /wheel_ticks(50 Hz)と同じ制御ループが出すので「agent経由の通信+ファームの制御ループが生きている」ことの証明になり、
    # 購読が1 Hzで済む(50 Hzの購読はPythonのコールバックだけで1コア近くを使うため)。5秒=4回連続で途絶。
    # agent再起動後はESP32が再接続してから流れ始める(session確立に数秒〜10秒台)ので猶予は25秒。
    # シリアルを開く確認(check_open)はDTR/RTSでESP32がリセットされ得るため、しない(存在のみ)。
    "esp32": dict(paths="/dev/esp32", process="micro_ros_agent", process_args="serial,/dev/esp32", baud=115200,
                  start="{prefix:micro_ros_agent}/lib/micro_ros_agent/micro_ros_agent serial --dev {path} -b {baud}",
                  topic="/nokosahen/noise_guard_counts=Int32MultiArray", data_timeout_sec=5.0, startup_grace_sec=25.0),
    # LiDAR(S1): /scan_raw は約10 Hz。3秒=約30フレーム途絶。モーター起動と初回スキャンに数秒かかるので猶予15秒。
    # 「プロセスは生きているがデータが出ない」ハングは、この data_timeout で検知する。
    "lidar": dict(paths="/dev/rplidar", process="rplidar_node", baud=256000,
                  start=("{prefix:rplidar_ros}/lib/rplidar_ros/rplidar_node --ros-args -r __node:=rplidar_node "
                         "-r scan:=scan_raw -p channel_type:=serial -p serial_port:={path} -p serial_baudrate:={baud} "
                         "-p frame_id:=laser -p inverted:=false -p angle_compensate:=true"),
                  topic="/scan_raw=LaserScan", data_timeout_sec=3.0, startup_grace_sec=15.0),
    # 以下は見せるだけ(止めない)。前方カメラは person_target_detector が起動時に1回だけ開く作り(再オープン無し)で、
    # LiDAR人物検出と近接停止も担うため、ノードの再起動は影響が大きい。理由は docs/device_watchdog.md。
    "front_camera": dict(action="report", paths="/dev/v4l/by-id/usb-AONI*-video-index0", check_open=True),
    "buffalo_camera": dict(action="report", paths="/dev/v4l/by-id/usb-KYE_Systems_Corp._BUFFALO*-video-index0", check_open=True),
    "coral": dict(action="report", usb_ids="1a6e:089a,18d1:9302"),
}

_TYPES = {}


def _msg_types():
    if not _TYPES:
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import Int32MultiArray, String
        from nav_msgs.msg import Odometry
        _TYPES.update(LaserScan=LaserScan, Int32MultiArray=Int32MultiArray, String=String, Odometry=Odometry)
    return _TYPES


def _split(s: str) -> Tuple[str, ...]:
    return tuple(x.strip() for x in str(s).split(",") if x.strip())


def _prefix_expand(cmd: str, path: str, baud: int) -> Optional[List[str]]:
    """起動コマンドの {prefix:pkg} {path} {baud} を展開。パッケージが見つからなければNone。"""
    import re
    from ament_index_python.packages import PackageNotFoundError, get_package_prefix

    def rep(m):
        return get_package_prefix(m.group(1))

    try:
        out = re.sub(r"\{prefix:([A-Za-z0-9_]+)\}", rep, cmd)
    except PackageNotFoundError:
        return None
    out = out.replace("{path}", path).replace("{baud}", str(baud))
    return shlex.split(out)


class RealIo(Io):
    """実機のI/O。プロセスは /proc を読んで完全一致で特定する(pkill -f / pgrep -f は使わない)。"""

    def __init__(self, node, log_dir: str):
        self.node = node
        self.log_dir = log_dir
        self.last_rx: Dict[str, float] = {}      # トピック名 -> 単調時刻
        self.children: Dict[str, subprocess.Popen] = {}
        self.cmds: Dict[str, List[str]] = {}
        self._me = {os.getpid(), os.getppid()}
        self._pid_cache: Dict[str, Tuple[float, List[int]]] = {}
        self.scan_interval_sec = 3.0

    def now(self) -> float:
        return time.monotonic()

    def device_state(self, cfg: DeviceConfig):
        ok, missing = expand_paths(cfg.paths)
        if ok and cfg.usb_ids and not usb_ids_present(cfg.usb_ids):
            ok, missing = False, "usb:" + ",".join(cfg.usb_ids)
        openable = ok
        if ok and cfg.check_open:
            import glob
            for p in cfg.paths:
                for real in (glob.glob(p) or [p]):
                    try:
                        fd = os.open(real, os.O_RDONLY | os.O_NONBLOCK)
                        os.close(fd)
                    except OSError:
                        openable = False
        return ok, openable, missing

    def find_pids(self, spec: ProcessSpec, fresh: bool = False) -> List[int]:
        """/proc の全走査は重い(数百プロセス×2ファイル)ので、間引く。
        見つかっているPIDは毎回 /proc/<pid> だけを確認(軽い)。見つかっていないとき(空)は scan_interval_sec に1回だけ全走査。
        起動直前・停止後の確認(fresh=True)は必ず全走査する。"""
        for p in list(self.children.values()):
            p.poll()                              # 自分の子は終了を回収(ゾンビを残さない)
        now = time.monotonic()
        ent = self._pid_cache.get(spec.name)
        if not fresh and ent is not None:
            t, pids = ent
            if pids:
                alive = [p for p in pids if pid_matches(p, spec)]
                self._pid_cache[spec.name] = (t, alive)
                if alive:
                    return alive
                # 全部消えた: 全走査(別のPIDで立ち上がり直しているかもしれない)
            elif now - t < self.scan_interval_sec:
                return []
        pids = list_pids(spec, exclude=tuple(self._me))
        self._pid_cache[spec.name] = (now, pids)
        return pids

    def send_signal(self, pid: int, sig: int) -> None:
        if pid in self._me:
            return
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass
        except OSError as exc:
            self.node.get_logger().error("signal %s -> %s 失敗: %s" % (sig, pid, exc))

    def start(self, cfg: DeviceConfig) -> bool:
        cmd = self.cmds.get(cfg.name)
        if not cmd:
            return False
        if not os.path.exists(cmd[0]):
            self.node.get_logger().error("%s: 実行ファイルが無い: %s" % (cfg.name, cmd[0]))
            return False
        self._pid_cache.pop(cfg.process.name if cfg.process else cfg.name, None)   # 起動したので、次の確認は全走査から
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            logf = open(os.path.join(self.log_dir, ".nokosahen_dw_%s.log" % cfg.name), "ab")
            old = self.children.get(cfg.name)
            if old is not None:
                old.poll()
            self.children[cfg.name] = subprocess.Popen(
                cmd, stdin=subprocess.DEVNULL, stdout=logf, stderr=subprocess.STDOUT,
                start_new_session=True, close_fds=True)
            logf.close()
            return True
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().error("%s: 起動に失敗: %s" % (cfg.name, exc))
            return False

    def last_data(self, cfg: DeviceConfig) -> Optional[float]:
        return self.last_rx.get(cfg.name)

    def log(self, level: str, msg: str) -> None:
        lg = self.node.get_logger()
        stamp = datetime.datetime.now().strftime("%H:%M:%S")
        getattr(lg, {"warn": "warn", "error": "error"}.get(level, "info"))("[%s] %s" % (stamp, msg))


def _lock_single_instance(path: str):
    """多重起動防止(watchdog自身)。ロックを持つファイルオブジェクトを返す(閉じない)。取れなければNone。"""
    try:
        import fcntl
        fh = open(path, "w")
        fcntl.flock(fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return fh
    except Exception:  # noqa: BLE001
        return None


def main(args=None) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    class DeviceWatchdog(Node):
        def __init__(self):
            super().__init__("device_watchdog")
            self.declare_parameter("tick_hz", 1.0)
            self.declare_parameter("status_topic", "/nokosahen/device_watchdog/status")
            self.declare_parameter("pause_file", "/tmp/nokosahen_device_watchdog.pause")
            self.declare_parameter("log_dir", os.path.expanduser("~"))
            self.declare_parameter("devices", "esp32,lidar,front_camera,buffalo_camera,coral")
            self.declare_parameter("backoff_base_sec", 5.0)
            self.declare_parameter("backoff_max_sec", 60.0)
            self.declare_parameter("term_grace_sec", 3.0)
            self.declare_parameter("healthy_reset_sec", 30.0)
            self.pause_file = str(self.get_parameter("pause_file").value)
            self.io = RealIo(self, str(self.get_parameter("log_dir").value))
            self.machines: List[DeviceMachine] = []
            names = _split(self.get_parameter("devices").value)
            sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
            for name in names:
                cfg = self._load_device(name)
                self.machines.append(DeviceMachine(cfg, self.io))
                if cfg.enabled and cfg.has_data:
                    topic, tname = self._topic_of(name)
                    self.create_subscription(
                        _msg_types()[tname], topic,
                        (lambda _m, n=name: self.io.last_rx.__setitem__(n, time.monotonic())),
                        sensor_qos, raw=True)              # rawで受信(デシリアライズしない)=50Hzでも軽い
            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
            self.pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), qos)
            self.create_timer(1.0 / max(0.2, float(self.get_parameter("tick_hz").value)), self._tick)
            self.create_timer(1.0, self._publish)
            self.get_logger().info("device_watchdog 開始: %s" % ", ".join(m.cfg.name for m in self.machines))

        def _p(self, name: str, field: str):
            full = "%s.%s" % (name, field)
            default = _DEVICE_DEFAULTS.get(name, {}).get(field, _FIELD_DEFAULTS[field])
            if not self.has_parameter(full):
                self.declare_parameter(full, default)
            return self.get_parameter(full).value

        def _topic_of(self, name: str) -> Tuple[str, str]:
            topic, _, tname = str(self._p(name, "topic")).partition("=")
            return topic.strip(), (tname.strip() or "String")

        def _load_device(self, name: str) -> DeviceConfig:
            g = lambda f: self._p(name, f)  # noqa: E731
            paths = _split(g("paths"))
            proc = None
            start_cmd: Tuple[str, ...] = ()
            if str(g("process")).strip():
                if str(g("action")) == "restart" and str(g("start")).strip():
                    exp = _prefix_expand(str(g("start")), paths[0] if paths else "", int(g("baud")))
                    if exp is None:
                        self.get_logger().error("%s: 起動コマンドのパッケージが見つからない。見せるだけにする" % name)
                    else:
                        start_cmd = tuple(exp)
                        self.io.cmds[name] = exp
                args_req = _split(g("process_args"))
                # デバイスのパスは launch 引数で変わるので、process_args に既定の /dev/xxx があれば実際の paths[0] に置き換える
                if paths:
                    args_req = tuple(paths[0] if a.startswith("/dev/") else a for a in args_req)
                proc = ProcessSpec(name=str(g("process")).strip(), args_contain=args_req, start_cmd=start_cmd)
            return DeviceConfig(
                name=name, enabled=bool(g("enabled")), action=str(g("action")), paths=paths,
                usb_ids=_split(g("usb_ids")), check_open=bool(g("check_open")), process=proc,
                has_data=bool(str(g("topic")).strip()), data_timeout_sec=float(g("data_timeout_sec")),
                startup_grace_sec=float(g("startup_grace_sec")), fault_confirm_count=int(g("fault_confirm_count")),
                fault_confirm_sec=float(g("fault_confirm_sec")), stable_sec=float(g("stable_sec")),
                term_grace_sec=float(self.get_parameter("term_grace_sec").value),
                backoff_base_sec=float(self.get_parameter("backoff_base_sec").value),
                backoff_max_sec=float(self.get_parameter("backoff_max_sec").value),
                healthy_reset_sec=float(self.get_parameter("healthy_reset_sec").value),
            )

        def _tick(self) -> None:
            paused = os.path.exists(self.pause_file)
            for m in self.machines:
                try:
                    m.set_paused(paused)
                    m.tick()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error("%s: tick例外 %s" % (m.cfg.name, exc))

        def _publish(self) -> None:
            body = {"stamp": time.time(), "paused": os.path.exists(self.pause_file),
                    "devices": {m.cfg.name: m.status() for m in self.machines if m.cfg.enabled}}
            msg = String()
            msg.data = json.dumps(body, ensure_ascii=False)
            self.pub.publish(msg)

        def stop_children(self) -> None:
            """終了時。managedなプロセスを残さない(SIGTERM→最大3秒待つ→SIGKILL)。"""
            managed = [m for m in self.machines if m.cfg.managed and m.cfg.enabled]
            for m in managed:
                m.shutdown()
            end = time.monotonic() + 3.0
            while time.monotonic() < end and any(self.io.find_pids(m.cfg.process) for m in managed):
                time.sleep(0.1)
            for m in managed:
                for pid in self.io.find_pids(m.cfg.process):
                    self.io.send_signal(pid, SIGKILL)
            for p in self.io.children.values():
                try:
                    p.wait(timeout=1.0)
                except Exception:  # noqa: BLE001
                    pass

    lock = _lock_single_instance(os.environ.get("NOKOSAHEN_DEVICE_WATCHDOG_LOCK", "/tmp/nokosahen_device_watchdog.lock"))
    if lock is None and os.name == "posix":
        print("device_watchdog: すでに動いているため起動しません", file=sys.stderr)
        return
    rclpy.init(args=args)
    node = DeviceWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_children()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
