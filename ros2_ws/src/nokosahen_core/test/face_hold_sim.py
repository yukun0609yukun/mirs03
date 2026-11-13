"""「向く」旋回中の位置保持(position_hold.py + behavior_director の _hold_linear)のシミュレーション。ROS・実機は使わない。

face_person_sim.Sim を継承し、車体のモデルだけを差し替える(判断は本物の behavior_director)。

車体のモデル(実機 2026-11-06 の測定に合わせた)
- 指令(/cmd_vel)は1tick(0.1秒)遅れて届き、ファームの加速ランプ(角 加速1.6/減速4.0 rad/s^2、直進 加速0.8/減速2.0 m/s^2)と臨界減衰の2次フィルタ(ω_n=8)を通る。
- wz < 0.6 rad/s 相当の指令では回らない、という実機の下限(face_turn_min_rps=0.9 で回避済み)は、director側にあるのでここでは入れない。
- ずれ: 左回り(wz>0)のとき、実際の旋回量 1 rad あたり前方へ k_left=1.4 cm(ゆっくり±25%変動)、右回りは k_right=0.1 cm/rad。
  さらに位置のランダムな揺れ(0.003 m/s)。odom の linear.x は、実際の前後速度(ずれ+補正)に測定ノイズ(0.003 m/s)を足したもの。
- LiDAR(/scan)は、障害物の点から作った360本(障害物が無ければ全部 inf)。

使い方: python test/face_hold_sim.py   (結果の表を出力)
"""
from __future__ import annotations

import json
import math
import random
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from typing import List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import face_person_sim as fs  # noqa: E402
from face_person_sim import DT, Person, wrap  # noqa: E402

bd = fs.bd
K_LEFT = 0.014    # m/rad(左回りの旋回 1 rad あたり、前方へ)
K_RIGHT = 0.001   # m/rad(右回り)


def make_scan(robot: dict, obstacles: List[Tuple[float, float]], n: int = 360):
    ranges = [math.inf] * n
    inc = 2.0 * math.pi / n
    for ox, oy in obstacles:
        dx, dy = ox - robot["x"], oy - robot["y"]
        r = math.hypot(dx, dy)
        b = wrap(math.atan2(dy, dx) - robot["yaw"])
        i = int(round((b + math.pi) / inc)) % n
        for j in (i - 1, i, i + 1):  # 点を3本分の幅にする
            ranges[j % n] = min(ranges[j % n], r)
    return SimpleNamespace(ranges=ranges, angle_min=-math.pi, angle_increment=inc, range_min=0.15, range_max=12.0)


class DriftSim(fs.Sim):
    def __init__(self, hold: bool = False, k_left: float = K_LEFT, k_right: float = K_RIGHT, obstacles=(),
                 drift_seed: int = 7, hold_params=None, odom_ok: bool = True, scan_ok: bool = True,
                 pose_ok: bool = True, **kw):
        super().__init__(**kw)
        n = self.n
        self.rng = random.Random(drift_seed)
        self.k_left, self.k_right = k_left, k_right
        self.k_slow = 0.0                      # ゆっくり変動する係数の揺れ(-0.25〜+0.25)
        self.vx_a = 0.0                        # 実際の車体の前後速度(指令による分)
        self.wz_a = 0.0
        self.pending = (0.0, 0.0)              # 1tick遅れの指令
        self.obstacles = list(obstacles)
        self.odom_ok, self.scan_ok, self.pose_ok = odom_ok, scan_ok, pose_ok
        self.v_body = 0.0
        self.v_r = self.v1 = self.w_r = self.w1 = 0.0
        self.start = dict(self.robot)
        n.face_turn_rps = 1.5       # 実機の値(directorの既定。スタブ構成の既定は0.75)
        n.face_turn_min_rps = 0.9
        n._face_hold_enabled = bool(hold)
        if hold_params is not None:
            n._hold = bd._ph.PositionHold(hold_params, log=lambda lv, m: n._log.lines.append((lv, m)))
        n._hold_subs_made = True              # ROSの購読は作らない(入力は下で直接入れる)
        if not pose_ok:
            n._get_pose = lambda: None
        else:
            n._get_pose = lambda: dict(self.robot)
        self.hooks = []   # step() の最初に呼ぶ関数(self を渡す)。安全条件のテストで状態を差し込む

    @staticmethod
    def _ramp(cur: float, cmd: float, acc: float, dec: float, h: float) -> float:
        rate = acc if (abs(cmd) > abs(cur) and cmd * cur >= 0.0) else dec
        return cur + max(-rate * h, min(rate * h, cmd - cur))

    # 実機のずれ + 指令への応答
    def _plant(self, cmd_vx: float, cmd_wz: float):
        sub = 5
        h = DT / sub
        vx_prev, wz_prev = self.pending
        self.pending = (cmd_vx, cmd_wz)
        for _ in range(sub):
            # ファームの加速ランプ(docs/autonomous_speed.md: 角 加速1.6/減速4.0 rad/s^2、直進 加速0.8/減速2.0 m/s^2)
            # + 臨界減衰の2次フィルタ(ω_n=8 → 1次遅れ(τ=0.125秒)を2段)
            self.v_r = self._ramp(self.v_r, vx_prev, 0.8, 2.0, h)
            self.w_r = self._ramp(self.w_r, wz_prev, 1.6, 4.0, h)
            self.v1 += (self.v_r - self.v1) * h / 0.125
            self.vx_a += (self.v1 - self.vx_a) * h / 0.125
            self.w1 += (self.w_r - self.w1) * h / 0.125
            self.wz_a += (self.w1 - self.wz_a) * h / 0.125
            self.k_slow += (-self.k_slow * 0.05 + self.rng.gauss(0, 0.04)) * h * 2
            self.k_slow = max(-0.25, min(0.25, self.k_slow))
            k = (self.k_left if self.wz_a > 0 else self.k_right) * (1.0 + self.k_slow)
            drift = k * abs(self.wz_a)
            self.v_body = self.vx_a + drift + self.rng.gauss(0, 0.003)
            self.robot["yaw"] = wrap(self.robot["yaw"] + self.wz_a * h)
            self.robot["x"] += self.v_body * math.cos(self.robot["yaw"]) * h
            self.robot["y"] += self.v_body * math.sin(self.robot["yaw"]) * h

    def step(self):
        n = self.n
        n._clock.t += DT
        self.t += DT
        for hk in self.hooks:
            hk(self)
        for p in self.people:
            p.x += p.vx * DT
            p.y += p.vy * DT
        n._person_candidates = self.sensor.sense(self.robot, [p for p in self.people if self.t >= p.t_on], self.clutter)
        n._person_candidates_mono = time.monotonic()
        n._person_candidates_rx = n._clock.now().nanoseconds / 1e9
        if getattr(n, "_near_person_stop_detected", False):
            n._last_near_person_stop_receive_time = n._clock.now()
        now_s = n._clock.now().nanoseconds / 1e9
        if self.odom_ok:
            n._hold_odom = (now_s, self.v_body + self.rng.gauss(0, 0.001), self.robot["x"], self.robot["y"], self.robot["yaw"],
                           self.wz_a + self.rng.gauss(0, 0.02))
        if self.scan_ok:
            n._hold_scan = (now_s, make_scan(self.robot, self.obstacles))
        if int(round(self.t / DT)) % 10 == 0:
            n._rules_tick()
        before = len(n._cmd_vel_pub.msgs)
        n._tick()
        new = n._cmd_vel_pub.msgs[before:]
        vx = new[-1].linear.x if new else 0.0
        wz = new[-1].angular.z if new else 0.0
        self._plant(vx, wz)
        cmd = json.loads(n._command_pub.msgs[-1].data)
        if len(n._announce_pub.msgs) > self._announce_n:
            self._announce_n = len(n._announce_pub.msgs)
            self.announce_times.append(self.t)
        self.log.append({"t": self.t, "x": self.robot["x"], "y": self.robot["y"], "yaw": self.robot["yaw"],
                         "vx": vx, "wz": wz, "mode": cmd["mode"], "aligned": cmd.get("face_aligned"),
                         "face_bearing": cmd.get("face_bearing_deg"), "people": [(p.x, p.y) for p in self.people],
                         "hold": cmd.get("face_hold")})

    # ---- 評価 ----
    def displacement(self) -> float:
        return math.hypot(self.robot["x"] - self.start["x"], self.robot["y"] - self.start["y"])

    def max_displacement(self) -> float:
        return max(math.hypot(r["x"] - self.start["x"], r["y"] - self.start["y"]) for r in self.log)

    def forward_displacement(self) -> float:
        """開始時の向きに対する前後方向の変位(前が正)。"""
        dx, dy = self.robot["x"] - self.start["x"], self.robot["y"] - self.start["y"]
        return dx * math.cos(self.start["yaw"]) + dy * math.sin(self.start["yaw"])

    def total_rotation(self) -> float:
        yaws = [self.start["yaw"]] + [r["yaw"] for r in self.log]
        return sum(abs(wrap(b - a)) for a, b in zip(yaws, yaws[1:]))

    def linear_sign_reversals(self) -> int:
        signs = [math.copysign(1, r["vx"]) for r in self.log if abs(r["vx"]) > 0.004]
        return sum(1 for a, b in zip(signs, signs[1:]) if a != b)

    def linear_nonzero_ticks(self) -> int:
        return sum(1 for r in self.log if abs(r["vx"]) > 0.0)

    def max_abs_linear(self) -> float:
        return max((abs(r["vx"]) for r in self.log), default=0.0)

    def n_aligned_events(self) -> int:
        a = [bool(r["aligned"]) for r in self.log]
        return sum(1 for x, y in zip(a, a[1:]) if y and not x)


# ------------------------------------------------------------------ シナリオ
def alternating_people(period: float = 5.0, n_flips: int = 40) -> List[Person]:
    """左(+80度)・右(-80度)に、交互に人が現れる(約160度ずつ往復して向く)。period 秒ごとに入れ替わる。"""
    ps = []
    for i in range(n_flips):
        side = 1.0 if i % 2 == 0 else -1.0
        # 距離2 m、世界の方位 ±75度。i番目の人は [i*period, (i+1)*period) の間だけ見える(前の人は消える)
        ang = math.radians(75.0) * side
        p = Person(2.0 * math.cos(ang), 2.0 * math.sin(ang), t_on=i * period)
        ps.append(p)
    return ps


class _Windowed(DriftSim):
    """alternating_people 用: 各人は t_on から period 秒だけ見える。"""

    def __init__(self, period: float = 5.0, **kw):
        super().__init__(**kw)
        self.period = period
        self._all = list(self.people)

    def step(self):
        t_next = self.t + DT
        self.people = [p for p in self._all if p.t_on <= t_next < p.t_on + self.period]
        super().step()


def run_sweep(hold: bool, seconds: float = 120.0, seed: int = 1, **kw) -> DriftSim:
    """人がいない: 前180度を往復して見回す(FACE_SWEEP)。"""
    s = DriftSim(hold=hold, seed=seed, drift_seed=seed + 10, robot=(0.0, 0.0, 0.0), **kw)
    s.run(seconds)
    return s


def run_alternating(hold: bool, seconds: float = 120.0, seed: int = 1, period: float = 5.0, **kw) -> DriftSim:
    """左右に交互に人が現れ、約150度ずつ往復して向く(FACE_PERSON。向き終わると止まる=外側ループが働ける)。"""
    s = _Windowed(period=period, hold=hold, seed=seed, drift_seed=seed + 10, robot=(0.0, 0.0, 0.0),
                  people=alternating_people(period, int(seconds / period) + 2), **kw)
    s.run(seconds)
    return s


def run_single_turn(hold: bool, seed: int = 1, **kw):
    """人が1人(左+70度)。整列までの時間・精度を測る。"""
    p = Person(2.0 * math.cos(math.radians(70)), 2.0 * math.sin(math.radians(70)))
    s = DriftSim(hold=hold, seed=seed, drift_seed=seed + 10, robot=(0.0, 0.0, 0.0), people=[p], **kw)
    s.run(10.0)
    return s, math.atan2(p.y, p.x)


def summarize(name: str, s: DriftSim) -> dict:
    return {
        "name": name,
        "disp_cm": round(s.displacement() * 100, 1),
        "max_disp_cm": round(s.max_displacement() * 100, 1),
        "fwd_cm": round(s.forward_displacement() * 100, 1),
        "rot_rad": round(s.total_rotation(), 1),
        "max_lin_mps": round(s.max_abs_linear(), 3),
        "lin_reversals": s.linear_sign_reversals(),
        "aligned_events": s.n_aligned_events(),
    }


if __name__ == "__main__":
    rows = []
    for hold in (False, True):
        for nm, fn in (("sweep", run_sweep), ("alternating", run_alternating)):
            tot = []
            for seed in (1, 2, 3):
                s = fn(hold, seed=seed)
                s.close()
                tot.append(s)
                rows.append((("ON " if hold else "OFF"), nm, seed, summarize(nm, s)))
    for r in rows:
        print(r)
    for hold in (False, True):
        for seed in (1, 2, 3):
            s, tgt = run_single_turn(hold, seed)
            err = math.degrees(wrap(tgt - s.robot["yaw"]))
            print("single", "ON " if hold else "OFF", seed, "aligned_at", s.first_aligned_time(), "err_deg", round(err, 1),
                  "max_lin", round(s.max_abs_linear(), 3), "disp_cm", round(s.displacement() * 100, 1))
            s.close()


def run_offset(forward_m: float = 0.0, lateral_m: float = 0.0, seconds: float = 25.0, hold: bool = True,
               kick_at: float = 3.0, **kw) -> DriftSim:
    """人が真正面(整列して止まっている=旋回なし)。kick_at 秒に、車体を前方 forward_m・左 lateral_m だけ動かす(外から押された/ずれた想定)。
    外側ループが、前後方向の成分だけ戻すこと(横は戻さない)を見る。"""
    p = Person(2.5, 0.0)
    s = DriftSim(hold=hold, robot=(0.0, 0.0, 0.0), people=[p], k_left=0.0, k_right=0.0, **kw)
    s.run(kick_at)
    s.robot["x"] += forward_m
    s.robot["y"] += lateral_m
    s.run(seconds - kick_at)
    return s
