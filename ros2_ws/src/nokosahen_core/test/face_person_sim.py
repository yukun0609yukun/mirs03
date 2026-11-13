"""「その場で人の方を向く」機能のパソコン上のシミュレーション(閉ループのモデル)。ROS・実機は使わない。

モデルの構成
- 世界: 走行可能範囲(回転矩形)、人(静止・移動)、壁などの誤検出(LiDARだけに映る静止物)。
- ロボット: 2輪の運動モデル(前進速度・角速度の指令を積分)。指令は behavior_director が実際に出した /cmd_vel を使う。
- センサ: LiDARの足の候補(全周、方位・距離にノイズ、見逃しあり)と、前面カメラ(画角150度、方位にノイズ、見逃しあり)から、
  person_target_detector が出す /nokosahen/person_candidates と同じJSONを毎tick作る。
- 判断: 本物の behavior_director(test/test_behavior_rules.py のスタブ構成)。位置は、このモデルのロボット位置を返す。

使い方: python test/face_person_sim.py [出力フォルダ]   (matplotlibがあれば、各シナリオの軌跡と向きの誤差のグラフを保存)
"""
from __future__ import annotations

import json
import math
import random
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import test_behavior_rules as base  # noqa: E402
from nokosahen_core import behavior_rules as br  # noqa: E402
from nokosahen_core import boundary_motion_guard as bmg  # noqa: E402
from nokosahen_core import map_orientation as mo  # noqa: E402

bd = base.bd
DT = 0.1


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class Person:
    def __init__(self, x: float, y: float, vx: float = 0.0, vy: float = 0.0, t_on: float = 0.0):
        self.x, self.y, self.vx, self.vy = x, y, vx, vy
        self.t_on = t_on  # この時刻[秒]から現れる(それまでは、いない)


class SensorModel:
    """LiDAR(足の候補)とカメラ(YOLO)のモデル。person_target_detector が出す候補JSONと同じ形を作る。"""

    def __init__(self, seed=1, lidar_range=6.0, camera_range=5.0, fov_deg=150.0, bearing_sigma_deg=1.5,
                 lidar_miss=0.05, camera_miss=0.10, confirm_tol_deg=15.0):
        self.rng = random.Random(seed)
        self.lidar_range, self.camera_range = lidar_range, camera_range
        self.fov = math.radians(fov_deg)
        self.sigma = math.radians(bearing_sigma_deg)
        self.lidar_miss, self.camera_miss = lidar_miss, camera_miss
        self.tol = math.radians(confirm_tol_deg)

    def sense(self, robot, people: List[Person], clutter: List[Tuple[float, float]]) -> dict:
        lidar, camera = [], []
        for pt in [(p.x, p.y, True) for p in people] + [(x, y, False) for x, y in clutter]:
            dx, dy = pt[0] - robot["x"], pt[1] - robot["y"]
            rng = math.hypot(dx, dy)
            if rng < 0.2 or rng > self.lidar_range or self.rng.random() < self.lidar_miss:
                continue
            bearing = wrap(math.atan2(dy, dx) - robot["yaw"] + self.rng.gauss(0, self.sigma))
            lidar.append({"bearing": bearing, "range": rng + self.rng.gauss(0, 0.03), "confirmed": False})
            is_person = pt[2]
            if is_person and rng <= self.camera_range and abs(bearing) <= self.fov / 2 and self.rng.random() >= self.camera_miss:
                camera.append({"bearing": wrap(bearing + self.rng.gauss(0, self.sigma))})
        for c in lidar:  # カメラの人と方位が一致したLiDAR候補だけ confirmed
            c["confirmed"] = any(abs(wrap(c["bearing"] - k["bearing"])) <= self.tol for k in camera)
        # 全周で一番近い物までの距離(人も物も含む。LiDARのばらつきあり)。検出ノードの nearest_m と同じ意味
        near = [math.hypot(pt[0] - robot["x"], pt[1] - robot["y"]) for pt in [(p.x, p.y) for p in people] + list(clutter)]
        nearest = (min(near) + self.rng.gauss(0, 0.03)) if near else 99.0
        return {"stamp": 0.0, "frame": "laser", "lidar": lidar, "camera": camera, "nearest_m": nearest,
                "nearest_age_s": 0.0}  # 検出ノードが出す、nearest_m の元のスキャンの古さ(理想のセンサなので0)


class Sim:
    def __init__(self, robot=(0.0, 0.0, 0.0), people=(), clutter=(), boundary=None, face=True, approach=False,
                 scan=False, seed=1, near_person=False, announce_after_facing=False, sensor=None):
        self.h = base.DirectorTickTests("make")
        self.h.setUp()
        n = self.h.make(None, crowd="calm", target=False)
        n._require_interaction_opt_in = True
        n._person_interaction_enabled = approach
        n._scan_enabled = scan
        n._face_enabled = face
        n._clock.t += 10
        self.n = n
        self.robot = {"x": robot[0], "y": robot[1], "yaw": robot[2]}
        self.people = list(people)
        self.clutter = list(clutter)
        self.sensor = sensor or SensorModel(seed)
        self.t = 0.0
        self.rect = None
        if boundary is not None:
            n._boundary = mo.boundary_state(boundary, enabled=True)
            self.rect = boundary
        n._get_pose = lambda: dict(self.robot)
        if near_person:
            n._near_person_stop_detected = True
        if announce_after_facing:
            n._rule_effects = br.Effects(announce_explicit=True, announce_min_interval_seconds=10.0,
                                         announce_after_facing=True, face_explicit=True)
            n._require_announce_opt_in = True
            n._rules_tick = lambda: None  # ルール評価が上の効果を上書きしないように固定(木の announce の after_facing を模す)
        self.log: List[dict] = []
        self.announce_times: List[float] = []
        self._announce_n = 0

    def close(self):
        self.h.tearDown()

    def step(self):
        n = self.n
        n._clock.t += DT
        self.t += DT
        for p in self.people:
            p.x += p.vx * DT
            p.y += p.vy * DT
        n._person_candidates = self.sensor.sense(self.robot, [p for p in self.people if self.t >= p.t_on], self.clutter)
        n._person_candidates_mono = time.monotonic()
        n._person_candidates_rx = n._clock.now().nanoseconds / 1e9  # ROS時計の受信時刻(近接停止の免除の鮮度用)
        if near_flag := getattr(n, "_near_person_stop_detected", False):
            n._last_near_person_stop_receive_time = n._clock.now()
        if int(round(self.t / DT)) % 10 == 0:
            n._rules_tick()
        before = len(n._cmd_vel_pub.msgs)
        n._tick()
        new = n._cmd_vel_pub.msgs[before:]
        vx = new[-1].linear.x if new else 0.0
        wz = new[-1].angular.z if new else 0.0
        self.robot["yaw"] = wrap(self.robot["yaw"] + wz * DT)
        self.robot["x"] += vx * math.cos(self.robot["yaw"]) * DT
        self.robot["y"] += vx * math.sin(self.robot["yaw"]) * DT
        cmd = json.loads(n._command_pub.msgs[-1].data)
        if len(n._announce_pub.msgs) > self._announce_n:
            self._announce_n = len(n._announce_pub.msgs)
            self.announce_times.append(self.t)
        self.log.append({"t": self.t, "x": self.robot["x"], "y": self.robot["y"], "yaw": self.robot["yaw"],
                         "vx": vx, "wz": wz, "mode": cmd["mode"], "aligned": cmd.get("face_aligned"),
                         "face_bearing": cmd.get("face_bearing_deg"), "people": [(p.x, p.y) for p in self.people]})

    def run(self, seconds: float):
        for _ in range(int(round(seconds / DT))):
            self.step()
        return self

    # --- 評価の道具 ---
    def crowd_bearing_world(self, persons: Optional[List[Person]] = None) -> Optional[float]:
        ps = persons or self.people
        if not ps:
            return None
        sx = sum(math.atan2(p.y - self.robot["y"], p.x - self.robot["x"]) for p in ps)  # 簡易: 平均方位(近い扇形内で使う)
        return sx / len(ps)

    def heading_error_deg(self, target_world: float) -> float:
        return math.degrees(wrap(target_world - self.robot["yaw"]))

    def first_aligned_time(self) -> Optional[float]:
        return next((r["t"] for r in self.log if r["aligned"]), None)

    def max_linear(self) -> float:
        return max((abs(r["vx"]) for r in self.log), default=0.0)

    def reversals(self) -> int:
        signs = [math.copysign(1, r["wz"]) for r in self.log if abs(r["wz"]) > 1e-6]
        return sum(1 for a, b in zip(signs, signs[1:]) if a != b)

    def inside_always(self) -> bool:
        if self.rect is None:
            return True
        return all(bmg.footprint_inside(self.rect, r["x"], r["y"], r["yaw"], margin=0.0) for r in self.log)

    def modes(self) -> Dict[str, int]:
        out: Dict[str, int] = {}
        for r in self.log:
            out[r["mode"]] = out.get(r["mode"], 0) + 1
        return out


def world_bearing(robot_xy, p: Person) -> float:
    return math.atan2(p.y - robot_xy[1], p.x - robot_xy[0])


# ------------------------------------------------------------------ シナリオ(それぞれ Sim を返す)
def scenario_single_left() -> Tuple[Sim, float]:
    p = Person(2.0, 1.2)
    s = Sim(people=[p]).run(12)
    return s, world_bearing((0, 0), p)


def scenario_single_behind() -> Tuple[Sim, float]:
    """監視範囲(スイッチONの向きの前180度)の外(真後ろ)にいる人は、向く対象にしない。範囲の中を見回すだけ。"""
    p = Person(-2.0, -0.4)
    s = Sim(robot=(0, 0, 0.0), people=[p]).run(14)
    return s, world_bearing((0, 0), p)


def scenario_single_side() -> Tuple[Sim, float]:
    """範囲の端(ほぼ真横、80度)にいる人: 大きく旋回して向く(行き過ぎや往復をしない)。"""
    p = Person(0.4, 2.2)
    s = Sim(robot=(0, 0, 0.0), people=[p]).run(14)
    return s, world_bearing((0, 0), p)


def scenario_sweep_then_person() -> Tuple[Sim, float]:
    """人がいない間は前180度を往復して見回し、途中で範囲の中に人が現れたら、その時のLiDAR・カメラで向く(1周してから向くのではない)。"""
    p = Person(2.0, -1.5, t_on=5.0)
    s = Sim(robot=(0, 0, 0.0), people=[p]).run(14)
    return s, world_bearing((0, 0), p)


def scenario_crowd_majority() -> Tuple[Sim, float]:
    left = [Person(2.2, 1.3), Person(2.4, 1.0), Person(2.0, 1.7)]   # 左前に3人
    right = [Person(1.5, -2.2)]                                   # 右に1人
    s = Sim(people=left + right).run(14)
    mean = sum(world_bearing((0, 0), q) for q in left) / len(left)
    return s, mean


def scenario_moving_person() -> Tuple[Sim, float]:
    p = Person(2.0, -1.5, vx=0.0, vy=0.25)  # 右から左へ横切る
    s = Sim(people=[p]).run(14)
    return s, world_bearing((s.robot["x"], s.robot["y"]), p)


def scenario_wall_clutter() -> Tuple[Sim, float]:
    p = Person(2.0, 1.4)
    s = Sim(people=[p], clutter=[(-2.5, 0.5), (0.3, -2.8)]).run(12)  # 壁の誤検出(LiDARだけ)が2つ
    return s, world_bearing((0, 0), p)


def scenario_switch_off() -> Tuple[Sim, float]:
    p = Person(2.0, 1.2)
    s = Sim(people=[p], face=False).run(8)
    return s, world_bearing((0, 0), p)


def scenario_near_person_hold() -> Tuple[Sim, float]:
    # 機体が回って触れる範囲(外接円+余白 約0.42 m)の中にいる人: 近接停止のまま動かない
    p = Person(0.3, 0.2)
    s = Sim(people=[p], near_person=True).run(6)
    return s, world_bearing((0, 0), p)


def scenario_near_person_face_turn() -> Tuple[Sim, float]:
    # 【2026-11-04】触れる範囲の外(約0.72 m。免除を始める基準は0.55 m)でも近接停止が出ている人: その場の旋回だけなので、免除して向く
    p = Person(0.6, 0.4)
    s = Sim(people=[p], near_person=True).run(8)
    return s, world_bearing((0, 0), p)


def scenario_boundary_room() -> Tuple[Sim, float]:
    p = Person(1.5, 1.3)
    s = Sim(robot=(0.0, 0.0, 0.0), people=[p], boundary=mo.make_rect(0, 0, 4.0, 4.0, 0.0)).run(12)
    return s, world_bearing((0, 0), p)


def scenario_boundary_edge() -> Tuple[Sim, float]:
    p = Person(1.0, 1.0)
    # 縁まで0.35 m: 機体は範囲の内側(半幅0.26)だが、旋回に必要な0.42 mは無い → 旋回しない
    s = Sim(robot=(1.65, 0.0, 0.0), people=[p], boundary=mo.make_rect(0, 0, 4.0, 4.0, 0.0)).run(8)
    return s, world_bearing((1.65, 0.0), p)


def scenario_rotated_boundary_corner() -> Tuple[Sim, float]:
    p = Person(1.4, 1.5)
    rect = mo.make_rect(0, 0, 3.0, 2.0, math.radians(35))
    s = Sim(robot=(0.9, 0.2, 0.0), people=[p], boundary=rect).run(10)
    return s, world_bearing((0.9, 0.2), p)


def scenario_after_facing() -> Tuple[Sim, float]:
    p = Person(0.3, 1.8)  # 左の真横寄り(範囲の中): 大きく旋回が要る
    s = Sim(robot=(0, 0, 0.0), people=[p], announce_after_facing=True).run(14)
    return s, world_bearing((0, 0), p)


SCENARIOS = {
    "single_left": scenario_single_left,
    "single_behind": scenario_single_behind,
    "single_side": scenario_single_side,
    "sweep_then_person": scenario_sweep_then_person,
    "crowd_majority": scenario_crowd_majority,
    "moving_person": scenario_moving_person,
    "wall_clutter": scenario_wall_clutter,
    "switch_off": scenario_switch_off,
    "near_person_hold": scenario_near_person_hold,
    "near_person_face_turn": scenario_near_person_face_turn,
    "boundary_room": scenario_boundary_room,
    "boundary_edge": scenario_boundary_edge,
    "rotated_boundary_corner": scenario_rotated_boundary_corner,
    "after_facing": scenario_after_facing,
}


def summarize(name: str, s: Sim, target_world: float) -> dict:
    err = s.heading_error_deg(target_world)
    return {
        "name": name,
        "final_error_deg": round(err, 1),
        "aligned_at_s": s.first_aligned_time(),
        "max_linear": round(s.max_linear(), 3),
        "reversals": s.reversals(),
        "inside_boundary": s.inside_always(),
        "modes": s.modes(),
        "first_announce_s": s.announce_times[0] if s.announce_times else None,
    }


def plot_all(out_dir: Path) -> List[Path]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    for name, fn in SCENARIOS.items():
        s, target = fn()
        fig, (a, b) = plt.subplots(1, 2, figsize=(9.5, 4.2))
        xs = [r["x"] for r in s.log]
        ys = [r["y"] for r in s.log]
        a.plot(xs, ys, "-", color="#2a7")
        for r in s.log[:: max(1, len(s.log) // 12)]:
            a.arrow(r["x"], r["y"], 0.35 * math.cos(r["yaw"]), 0.35 * math.sin(r["yaw"]), head_width=0.08, color="#175")
        for k, (px, py) in enumerate(s.log[-1]["people"]):
            a.plot(px, py, "o", color="#d33")
        for cx, cy in s.clutter:
            a.plot(cx, cy, "s", color="#999")
        if s.rect is not None:
            pts = mo.rect_corners(s.rect)
            a.plot([p[0] for p in pts + [pts[0]]], [p[1] for p in pts + [pts[0]]], "--", color="#36c")
        a.set_aspect("equal")
        a.set_title(name)
        a.grid(alpha=.3)
        ts = [r["t"] for r in s.log]
        err = [math.degrees(wrap(target - r["yaw"])) for r in s.log]
        b.plot(ts, err)
        b.axhline(5, color="#aaa", ls=":")
        b.axhline(-5, color="#aaa", ls=":")
        b.set_xlabel("time [s]")
        b.set_ylabel("heading error [deg]")
        b.grid(alpha=.3)
        fig.tight_layout()
        p = out_dir / f"face_sim_{name}.png"
        fig.savefig(p, dpi=90)
        plt.close(fig)
        paths.append(p)
    return paths


if __name__ == "__main__":
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    for nm, fn in SCENARIOS.items():
        sim, tgt = fn()
        print(summarize(nm, sim, tgt))
        sim.close()
    if out is not None:
        print("グラフ:", len(plot_all(out)), "枚 →", out)
