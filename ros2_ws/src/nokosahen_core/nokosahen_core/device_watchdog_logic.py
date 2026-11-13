"""device_watchdog の純粋ロジック(ROS・実機・OSに依存しない)。

機器(ESP32 / LiDAR / カメラ / Coral 等)ごとの状態機械 `DeviceMachine` と、プロセス特定の関数群。
ハードウェアI/O(デバイスの存在確認・/proc・シグナル・プロセス起動・トピック受信時刻)は
`Io` インターフェースの後ろに隠してあり、テストでは偽物の Io と偽の単調時計で動かせる。

状態遷移(managed=プロセスを止めて再起動できる機器):

    INIT ──(プロセスが既にある)──────────────▶ STARTING(引き継ぎ)
      └──(無い)──────────────────────────▶ WAITING_FOR_DEVICE
    STARTING ──(データが流れた)────────────▶ HEALTHY
      ├──(猶予切れ/プロセス死亡/デバイス消失)▶ FAULT
    HEALTHY ──(異常が N回かつ fault_confirm_sec 連続)▶ FAULT
    FAULT ──(SIGTERM→待つ→SIGKILL で、プロセスが無くなった)▶ WAITING_FOR_DEVICE
    WAITING_FOR_DEVICE ──(デバイスが stable_sec 安定して存在し、開け、バックオフ時間が過ぎた)▶ RESTARTING
    RESTARTING ──(既存プロセスが無いことを確認して起動)▶ STARTING
      └──(起動失敗)▶ WAITING_FOR_DEVICE(バックオフを伸ばす)

report 専用(止める/起動する手段が無い機器): STARTING → HEALTHY ⇄ FAULT(止めない、見せるだけ)。
時計は単調時計だけを使う。逆行は時刻の基準をずらして吸収し、大きな飛びは判定タイマーをやり直す。
"""
from __future__ import annotations

import glob as _glob
import os
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

INIT = "INIT"
STARTING = "STARTING"
HEALTHY = "HEALTHY"
FAULT = "FAULT"
WAITING_FOR_DEVICE = "WAITING_FOR_DEVICE"
RESTARTING = "RESTARTING"
PAUSED = "PAUSED"
DISABLED = "DISABLED"

# 1 tick でこれ以上時計が進んだら「飛び(サスペンド等)」とみなして判定タイマーをやり直す[秒]
CLOCK_JUMP_SEC = 10.0

# Windows(開発PC)には signal.SIGKILL が無いので数値で持つ(実機はLinux: 15, 9)
SIGTERM = 15
SIGKILL = 9


# --------------------------------------------------------------------------- 設定
@dataclass
class ProcessSpec:
    """監視・停止・起動するプロセスの特定条件。完全一致のみ(部分一致のpkill -f 方式は使わない)。"""

    name: str                                   # 実行ファイル名(例: rplidar_node)。argvの先頭、またはインタプリタの次の引数のbasenameと完全一致
    args_contain: Tuple[str, ...] = ()          # argvにこれらが(完全一致で)全て含まれること(例: /dev/esp32)
    start_cmd: Tuple[str, ...] = ()              # 起動コマンド(空なら起動できない=report専用)


@dataclass
class DeviceConfig:
    name: str
    enabled: bool = True
    action: str = "restart"                      # "restart" | "report"
    paths: Tuple[str, ...] = ()                  # 全部が存在すること(globを許す)
    usb_ids: Tuple[str, ...] = ()                # "vid:pid"のどれかがUSBに見えること
    check_open: bool = False                     # 開けるか(シリアルは DTR でESP32をリセットし得るので既定false)
    process: Optional[ProcessSpec] = None
    has_data: bool = False                       # データ(トピック)の流れを見るか
    data_timeout_sec: float = 3.0
    startup_grace_sec: float = 15.0
    fault_confirm_count: int = 3
    fault_confirm_sec: float = 2.0
    stable_sec: float = 3.0                      # 再起動前に、デバイスが安定して存在すべき時間
    term_grace_sec: float = 3.0                  # SIGTERM後、SIGKILLまで待つ時間
    kill_wait_sec: float = 3.0                   # SIGKILL後、まだ残るなら再送するまで
    backoff_base_sec: float = 5.0
    backoff_max_sec: float = 60.0
    healthy_reset_sec: float = 30.0              # この時間ずっと健全なら失敗回数を0に戻す

    @property
    def managed(self) -> bool:
        return self.action == "restart" and self.process is not None and bool(self.process.start_cmd)


# --------------------------------------------------------------------------- プロセス特定(純粋)
def match_process(argv: Sequence[str], spec: ProcessSpec) -> bool:
    """argvが spec に完全一致するか。

    - argv[0] の basename == spec.name、またはインタプリタ(python*/bash/sh 以外は対象外)を介した
      スクリプト(argv[1] の basename == spec.name)。
    - ssh / bash -c "... rplidar_node ..." / vim rplidar_node.log / pgrep rplidar_node などは、
      argv[0] や argv[1] がそれぞれ ssh/bash/vim/pgrep なので一致しない。
    - args_contain の各要素は argv のどれかと完全一致していること。
    """
    if not argv or not spec.name:
        return False
    a0 = os.path.basename(argv[0])
    hit = a0 == spec.name
    if not hit and a0.startswith("python") and len(argv) >= 2:
        hit = os.path.basename(argv[1]) == spec.name
    if not hit:
        return False
    rest = list(argv[1:])
    return all(req in rest for req in spec.args_contain)


def _read_text(path: str) -> Optional[str]:
    try:
        with open(path, "rb") as fh:
            return fh.read().decode("utf-8", "replace")
    except OSError:
        return None


def pid_matches(pid: int, spec: ProcessSpec, proc_root: str = "/proc") -> bool:
    """1つのPIDがまだ spec に一致する生きたプロセスか(/proc/<pid> の2ファイルだけ読む=全走査より桁違いに軽い)。"""
    cmd = _read_text(os.path.join(proc_root, str(pid), "cmdline"))
    if not cmd:
        return False
    argv = [a for a in cmd.split("\0") if a != ""]
    if not match_process(argv, spec):
        return False
    stat = _read_text(os.path.join(proc_root, str(pid), "stat"))
    if stat is not None:
        tail = stat.rsplit(")", 1)[-1].split()
        if tail and tail[0] in ("Z", "X"):
            return False
    return True


def list_pids(spec: ProcessSpec, proc_root: str = "/proc", exclude: Sequence[int] = ()) -> List[int]:
    """/proc を読んで spec に一致するPIDを返す。ゾンビ・自分自身(exclude)は除く。"""
    out: List[int] = []
    try:
        entries = os.listdir(proc_root)
    except OSError:
        return out
    for ent in entries:
        if not ent.isdigit():
            continue
        pid = int(ent)
        if pid in exclude:
            continue
        cmd = _read_text(os.path.join(proc_root, ent, "cmdline"))
        if not cmd:
            continue
        argv = [a for a in cmd.split("\0") if a != ""]
        if not match_process(argv, spec):
            continue
        stat = _read_text(os.path.join(proc_root, ent, "stat"))
        if stat is not None:
            tail = stat.rsplit(")", 1)[-1].split()
            if tail and tail[0] in ("Z", "X"):
                continue
        out.append(pid)
    return sorted(out)


def usb_ids_present(ids: Sequence[str], sys_root: str = "/sys/bus/usb/devices") -> bool:
    """sysfs に vid:pid のどれかが見えるか(空の指定は True)。"""
    wanted = {i.strip().lower() for i in ids if i.strip()}
    if not wanted:
        return True
    try:
        entries = os.listdir(sys_root)
    except OSError:
        return False
    for ent in entries:
        vid = _read_text(os.path.join(sys_root, ent, "idVendor"))
        pid = _read_text(os.path.join(sys_root, ent, "idProduct"))
        if vid and pid and "%s:%s" % (vid.strip().lower(), pid.strip().lower()) in wanted:
            return True
    return False


def expand_paths(paths: Sequence[str]) -> Tuple[bool, str]:
    """全パス(globを許す)が1つ以上に展開できるか。(存在, 欠けているパス)。"""
    for p in paths:
        if not p:
            continue
        if any(c in p for c in "*?["):
            if not _glob.glob(p):
                return False, p
        elif not os.path.exists(p):
            return False, p
    return True, ""


def backoff_delay(cfg: DeviceConfig, fails: int) -> float:
    """失敗回数に応じた再試行までの待ち時間。0回=0秒、1回=base、2回=2*base、…、上限max。"""
    if fails <= 0:
        return 0.0
    return min(cfg.backoff_max_sec, cfg.backoff_base_sec * (2 ** (fails - 1)))


# --------------------------------------------------------------------------- I/O の窓口
class Io:
    """DeviceMachine が外界に触る唯一の窓口。実機版は device_watchdog.py、テストでは偽物を使う。"""

    def now(self) -> float:  # 単調時計[秒]
        raise NotImplementedError

    def device_state(self, cfg: DeviceConfig) -> Tuple[bool, bool, str]:  # (存在, 開ける, 欠けているもの)
        raise NotImplementedError

    def find_pids(self, spec: ProcessSpec, fresh: bool = False) -> List[int]:
        """一致するPID。fresh=True は必ず最新(起動直前の二重起動確認・停止後の確認用)。False は間引いた結果でよい。"""
        raise NotImplementedError

    def send_signal(self, pid: int, sig: int) -> None:
        raise NotImplementedError

    def start(self, cfg: DeviceConfig) -> bool:
        raise NotImplementedError

    def last_data(self, cfg: DeviceConfig) -> Optional[float]:  # 最後にデータを受けた単調時刻(未受信=None)
        raise NotImplementedError

    def log(self, level: str, msg: str) -> None:
        raise NotImplementedError


# --------------------------------------------------------------------------- 状態機械
class DeviceMachine:
    def __init__(self, cfg: DeviceConfig, io: Io):
        self.cfg = cfg
        self.io = io
        self.state = INIT if cfg.enabled else DISABLED
        self.reason = ""
        self.restarts = 0
        self.fails = 0
        self.faults = 0
        self.paused = False
        now = io.now()
        self._last_now = now
        self.state_since = now
        self.last_ok: Optional[float] = None
        self._started_at = now
        self._healthy_since: Optional[float] = None
        self._bad_count = 0
        self._bad_since: Optional[float] = None
        self._stable_since: Optional[float] = None
        self._next_attempt_at = now
        self._term_at: Optional[float] = None
        self._kill_at: Optional[float] = None
        self._warned_wait = False
        self.warning = ""

    # ---- 公開
    def set_paused(self, paused: bool) -> None:
        if paused == self.paused:
            return
        self.paused = paused
        self.io.log("info", "%s: %s" % (self.cfg.name, "一時停止(監視・再起動をしない)" if paused else "再開(状態を見直す)"))
        if not paused and self.state != DISABLED:
            self._goto(INIT, "resume")

    def status(self) -> Dict[str, object]:
        now = self.io.now()
        return {
            "state": PAUSED if self.paused else self.state,
            "reason": self.reason,
            "warning": self.warning,
            "restarts": self.restarts,
            "consecutive_fails": self.fails,
            "faults": self.faults,
            "managed": self.cfg.managed,
            "last_ok_age_s": None if self.last_ok is None else round(max(0.0, now - self.last_ok), 1),
            "state_age_s": round(max(0.0, now - self.state_since), 1),
            "next_attempt_in_s": round(max(0.0, self._next_attempt_at - now), 1) if self.state == WAITING_FOR_DEVICE else None,
        }

    def shutdown(self) -> None:
        """ノード終了時。managedな機器のプロセスを残さない(SIGTERMだけ。残りは呼び出し側が待って確認)。"""
        if self.cfg.managed and self.cfg.process:
            for pid in self.io.find_pids(self.cfg.process):
                self.io.send_signal(pid, SIGTERM)

    # ---- tick
    def tick(self) -> None:
        if not self.cfg.enabled or self.state == DISABLED:
            return
        now = self.io.now()
        if now < self._last_now:                    # 逆行: 基準時刻をずらして、経過時間を保つ
            self._shift(now - self._last_now)
            self.io.log("warn", "%s: 時計が逆行(%.1f秒)。基準時刻をずらして継続" % (self.cfg.name, self._last_now - now))
        elif now - self._last_now > CLOCK_JUMP_SEC:  # 飛び: 判定のやり直し
            self.io.log("warn", "%s: 時計が飛んだ(%.1f秒)。連続判定・安定待ちをやり直す" % (self.cfg.name, now - self._last_now))
            self._bad_count = 0
            self._bad_since = None
            self._stable_since = None
            self._started_at = now
            self._healthy_since = now if self.state == HEALTHY else self._healthy_since
        self._last_now = now
        if self.paused:
            return
        handler = {
            INIT: self._t_init, STARTING: self._t_starting, HEALTHY: self._t_healthy, FAULT: self._t_fault,
            WAITING_FOR_DEVICE: self._t_waiting, RESTARTING: self._t_restarting,
        }[self.state]
        handler(now)

    # ---- 内部
    def _shift(self, delta: float) -> None:
        for attr in ("state_since", "_started_at", "_healthy_since", "_bad_since", "_stable_since",
                     "_next_attempt_at", "_term_at", "_kill_at", "last_ok"):
            v = getattr(self, attr)
            if v is not None:
                setattr(self, attr, v + delta)
        self._last_now += delta

    def _goto(self, state: str, reason: str = "") -> None:
        if state != self.state:
            self.io.log("info", "%s: %s -> %s %s" % (self.cfg.name, self.state, state, ("(%s)" % reason) if reason else ""))
        self.state = state
        self.reason = reason
        self.state_since = self.io.now()

    def _check(self, now: float) -> Tuple[bool, str, bool]:
        """(健全か, 理由, 起動直後でも即異常と断定できるか)"""
        cfg = self.cfg
        present, openable, missing = self.io.device_state(cfg)
        if not present:
            return False, "device_missing:%s" % missing, True
        if cfg.check_open and not openable:
            return False, "device_not_openable", False
        if cfg.managed and not self.io.find_pids(cfg.process):
            return False, "process_missing", True
        if cfg.has_data:
            last = self.io.last_data(cfg)
            # 起動(または引き継ぎ)より前に受けたデータは、新しい個体のものとは認めない
            if last is None or last < self._started_at:
                return False, "no_data_since_start", False
            if now - last > cfg.data_timeout_sec:
                return False, "no_data(%.1fs>%.1fs)" % (now - last, cfg.data_timeout_sec), False
        return True, "", False

    def _t_init(self, now: float) -> None:
        cfg = self.cfg
        self._started_at = now
        if not cfg.managed:
            self._goto(STARTING, "report-only")
            return
        pids = self.io.find_pids(cfg.process)
        if pids:
            self.io.log("info", "%s: 既存プロセス %s を引き継いで監視する(二重起動しない)" % (cfg.name, pids))
            self._goto(STARTING, "adopt")
        else:
            self._goto(WAITING_FOR_DEVICE, "no_process")
            self._stable_since = None
            self._next_attempt_at = now

    def _t_starting(self, now: float) -> None:
        ok, reason, definite = self._check(now)
        if ok:
            self._become_healthy(now)
            return
        if definite or now - self._started_at > self.cfg.startup_grace_sec:
            self.fails += 1
            self._enter_fault(now, reason if definite else "startup_timeout:" + reason)

    def _become_healthy(self, now: float) -> None:
        self._goto(HEALTHY)
        self._healthy_since = now
        self.last_ok = now
        self._bad_count = 0
        self._bad_since = None

    def _t_healthy(self, now: float) -> None:
        cfg = self.cfg
        ok, reason, _ = self._check(now)
        if ok:
            self.last_ok = now
            self._bad_count = 0
            self._bad_since = None
            self.reason = ""
            if self._healthy_since is not None and now - self._healthy_since >= cfg.healthy_reset_sec:
                self.fails = 0
            return
        self.reason = reason
        if self._bad_since is None:
            self._bad_since = now
        self._bad_count += 1
        if self._bad_count >= cfg.fault_confirm_count and now - self._bad_since >= cfg.fault_confirm_sec:
            if self._healthy_since is not None and now - self._healthy_since < cfg.healthy_reset_sec:
                self.fails += 1          # 立ち上がってすぐまた落ちた: バックオフを伸ばす
            else:
                self.fails = 0
            self._enter_fault(now, reason)

    def _enter_fault(self, now: float, reason: str) -> None:
        self.faults += 1
        self._term_at = None
        self._kill_at = None
        self._stable_since = None
        self._bad_count = 0
        self._bad_since = None
        self._next_attempt_at = now + backoff_delay(self.cfg, self.fails)
        self.io.log("warn", "%s: 異常を確認(%s) 異常%d回目 連続失敗=%d" % (self.cfg.name, reason, self.faults, self.fails))
        self._goto(FAULT, reason)
        if self.cfg.managed:
            self._t_fault(now)

    def _t_fault(self, now: float) -> None:
        cfg = self.cfg
        if not cfg.managed:                          # 見せるだけの機器: 健全に戻ったら自動で戻る
            ok, _reason, _ = self._check(now)
            if ok:
                self.io.log("info", "%s: 回復を確認" % cfg.name)
                self._become_healthy(now)
            return
        pids = self.io.find_pids(cfg.process, fresh=True)
        if not pids:
            self._goto(WAITING_FOR_DEVICE, self.reason)
            self._stable_since = None
            self._warned_wait = False
            return
        if self._term_at is None:
            self.io.log("info", "%s: 停止(SIGTERM) pid=%s" % (cfg.name, pids))
            for p in pids:
                self.io.send_signal(p, SIGTERM)
            self._term_at = now
        elif self._kill_at is None:
            if now - self._term_at >= cfg.term_grace_sec:
                self.io.log("warn", "%s: SIGTERMで止まらないためSIGKILL pid=%s" % (cfg.name, pids))
                for p in pids:
                    self.io.send_signal(p, SIGKILL)
                self._kill_at = now
        elif now - self._kill_at >= cfg.kill_wait_sec:
            self.io.log("error", "%s: SIGKILL後も残っている pid=%s。再送する" % (cfg.name, pids))
            for p in pids:
                self.io.send_signal(p, SIGKILL)
            self._kill_at = now

    def _t_waiting(self, now: float) -> None:
        cfg = self.cfg
        pids = self.io.find_pids(cfg.process)
        if pids:
            # 誰かが(手動などで)先に起動した。二重起動を避けるため、こちらでは起動せず引き継ぐ
            self.io.log("info", "%s: 待機中に既存プロセス %s を検出。引き継ぐ" % (cfg.name, pids))
            self._started_at = now
            self._goto(STARTING, "adopt")
            return
        present, openable, missing = self.io.device_state(cfg)
        usable = present and (openable or not cfg.check_open)
        if not usable:
            if self._stable_since is not None:
                self.io.log("info", "%s: デバイスが途切れた(%s)。安定待ちをやり直す" % (cfg.name, missing or "not_openable"))
            self._stable_since = None
            if not self._warned_wait:
                self.io.log("info", "%s: デバイス待ち(%s)" % (cfg.name, missing or "not_openable"))
                self._warned_wait = True
            self.reason = "device_missing:%s" % missing if not present else "device_not_openable"
            return
        self._warned_wait = False
        if self._stable_since is None:
            self._stable_since = now
            self.io.log("info", "%s: デバイスを検出。%.1f秒の安定を待つ" % (cfg.name, cfg.stable_sec))
        if now - self._stable_since < cfg.stable_sec or now < self._next_attempt_at:
            return
        self._goto(RESTARTING, "device_stable")
        self._t_restarting(now)

    def _t_restarting(self, now: float) -> None:
        cfg = self.cfg
        pids = self.io.find_pids(cfg.process, fresh=True)
        if pids:                                    # 二重起動防止: 残っているなら起動せず停止からやり直す
            self.io.log("warn", "%s: 起動直前に既存プロセス %s を検出。起動せず停止からやり直す" % (cfg.name, pids))
            self._term_at = None
            self._kill_at = None
            self._goto(FAULT, "stale_process_before_start")
            self._t_fault(now)
            return
        self.restarts += 1
        self.io.log("info", "%s: 起動 %d回目" % (cfg.name, self.restarts))
        if self.io.start(cfg):
            self._started_at = now
            self._goto(STARTING, "restarted")
        else:
            self.fails += 1
            self._next_attempt_at = now + backoff_delay(cfg, self.fails)
            self._stable_since = None
            self.io.log("error", "%s: 起動に失敗。%.0f秒後に再試行" % (cfg.name, self._next_attempt_at - now))
            self._goto(WAITING_FOR_DEVICE, "start_failed")
