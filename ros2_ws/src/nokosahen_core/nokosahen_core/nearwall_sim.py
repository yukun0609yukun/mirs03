"""壁際の出発点の模擬(2026-10-22)。ROS不要。実機でNavFnが「Failed to create plan」になった状況を机上で再現する。

- 2D格子の模擬コストマップ(0.05m/マス): 壁(LiDAR点)をNav2のインフレーションで膨らませる
  (致死254・内接253・それより外は 252*exp(-k*(距離-内接半径))、inflation_radiusの外は0)。
- NavFn相当の経路計画: コスト>=253のマスは通行不可。出発点のマスだけ空きにする(clearRobotCell)が、隣が全て
  通行不可なら失敗する。波面伝播(Dijkstra、8近傍)。
- シナリオ(壁・箱・コの字・通路)と、wall_escape の離脱(motion_sim.pick_motion を繰り返し適用して姿勢を積分)。

座標は世界座標(x前、y左、m)。機体の初期姿勢は原点・向き0。実機の値は nav_speed.derive_clearance_values() から取る。
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from . import nav_speed
from .motion_sim import min_clearance, pick_motion, pose_at, to_xy
from .wall_escape import flee_angle, min_range

Point = Tuple[float, float]
Pose = Tuple[float, float, float]

RES = 0.05
BODY_HALF = nav_speed.BODY_HALF_M            # 0.26
LETHAL, INSCRIBED = 254, 253
# wall_escape.py の既定値(ノードのパラメータと同じ)
TRIGGER_M, CLEAR_M, RADIUS_M, LIN, ANG, MAX_SEC = 0.45, 0.55, 0.7, 0.10, 0.6, 6.0


# ---------------- 壁(LiDAR点)の生成 ----------------
def seg(a: Point, b: Point, step: float = 0.02) -> List[Point]:
    n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n + 1)]


def box_points(x0: float, y0: float, x1: float, y1: float) -> List[Point]:
    return seg((x0, y0), (x1, y0)) + seg((x1, y0), (x1, y1)) + seg((x1, y1), (x0, y1)) + seg((x0, y1), (x0, y0))


# ---------------- 模擬コストマップ ----------------
@dataclass
class Costmap:
    cost: List[List[int]]
    x0: float
    y0: float
    res: float = RES

    @property
    def nx(self) -> int:
        return len(self.cost[0])

    @property
    def ny(self) -> int:
        return len(self.cost)

    def cell(self, x: float, y: float) -> Tuple[int, int]:
        return int(math.floor((x - self.x0) / self.res)), int(math.floor((y - self.y0) / self.res))

    def at(self, ix: int, iy: int) -> int:
        if 0 <= ix < self.nx and 0 <= iy < self.ny:
            return self.cost[iy][ix]
        return LETHAL  # 地図の外は通れない


@dataclass(frozen=True)
class Inflation:
    inscribed: float          # 内接半径(機体半幅)
    radius: float             # inflation_radius
    k: float                  # cost_scaling_factor

    def cost_at(self, d: float) -> int:
        if d <= 0.0:
            return LETHAL
        if d <= self.inscribed:
            return INSCRIBED
        if d > self.radius:
            return 0
        return int(252.0 * math.exp(-self.k * (d - self.inscribed)))


def inflation_for(body_half: float = BODY_HALF, clearance_m: float = nav_speed.CLEARANCE_M) -> Inflation:
    """nav_speed.derive_clearance_values() の値(inflation_radius・cost_scaling_factor)から作る。"""
    d = nav_speed.derive_clearance_values(clearance_m, body_half)
    return Inflation(body_half, float(d["inflation_radius"]), float(d["cost_scaling_factor"]))


def build_costmap(points: Sequence[Point], infl: Inflation, bounds: Tuple[float, float, float, float],
                  res: float = RES, bias_m: float = 0.0) -> Costmap:
    """障害物の点を致死マスにして、インフレーションする。bias_mは「コストマップ上で壁が実際よりbias_m近く見える」誤差
    (位置ずれ・障害物の太り・LiDAR取り付け位置の誤差)の模擬: 壁からの距離をbias_mだけ短く見積もってコストを付ける。"""
    x0, y0, x1, y1 = bounds
    nx, ny = int(math.ceil((x1 - x0) / res)), int(math.ceil((y1 - y0) / res))
    lethal = {(int(math.floor((px - x0) / res)), int(math.floor((py - y0) / res))) for px, py in points}
    r_cells = int(math.ceil((infl.radius + bias_m) / res)) + 1
    dist: Dict[Tuple[int, int], float] = {}
    for lx, ly in lethal:
        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                ix, iy = lx + dx, ly + dy
                if not (0 <= ix < nx and 0 <= iy < ny):
                    continue
                d = math.hypot(dx, dy) * res
                if d < dist.get((ix, iy), 1e9):
                    dist[(ix, iy)] = d
    cost = [[0] * nx for _ in range(ny)]
    for (ix, iy), d in dist.items():
        cost[iy][ix] = infl.cost_at(max(d - bias_m, 0.0) if d > 0 else 0.0)
    return Costmap(cost, x0, y0, res)


# ---------------- NavFn相当の経路計画 ----------------
@dataclass
class PlanResult:
    ok: bool
    reason: str = ""
    length_m: float = 0.0


_N8 = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]


def plan(cm: Costmap, start: Point, goal: Point) -> PlanResult:
    """出発点のマスだけ空き(clearRobotCell)にして、出発点から目的地へDijkstra。コスト>=253は通行不可。"""
    s, g = cm.cell(*start), cm.cell(*goal)

    def passable(c: Tuple[int, int]) -> bool:
        return c == s or cm.at(*c) < INSCRIBED

    if not passable(g):
        return PlanResult(False, "目的地が通行不可(壁のコスト帯の中)")
    if not any(passable((s[0] + dx, s[1] + dy)) for dx, dy in _N8):
        return PlanResult(False, "出発点の隣8マスが全て通行不可(出発点だけ空きにしても出られない=Failed to create plan)")
    best = {s: 0.0}
    heap = [(0.0, s)]
    while heap:
        d, c = heapq.heappop(heap)
        if c == g:
            return PlanResult(True, "", d * cm.res)
        if d > best.get(c, 1e18):
            continue
        for dx, dy in _N8:
            n = (c[0] + dx, c[1] + dy)
            if not passable(n):
                continue
            step = math.hypot(dx, dy) * (1.0 + cm.at(*n) / 100.0)  # 高コストを嫌う(NavFnも同様に重み付け)
            nd = d + step
            if nd < best.get(n, 1e18):
                best[n] = nd
                heapq.heappush(heap, (nd, n))
    return PlanResult(False, "経路が通行不可の帯で遮られている(隣は空いているが、目的地まで続かない)")


# ---------------- シナリオ ----------------
@dataclass
class Scenario:
    name: str
    kind: str                   # back/left/right/front/corner/ushape/corridor/real
    edge_m: float               # 機体端から壁まで(通路は片側)
    walls: List[Point]
    goal: Point
    bias_m: float = 0.0
    note: str = ""
    bounds: Tuple[float, float, float, float] = (-2.0, -2.5, 3.5, 2.5)


def _long(x0, y0, x1, y1):
    return seg((x0, y0), (x1, y1))


def single_wall(side: str, edge: float) -> Scenario:
    d = BODY_HALF + edge
    L = 2.5
    if side == "back":
        walls, goal = _long(-d, -L, -d, L), (1.5, 0.0)
    elif side == "left":
        walls, goal = _long(-L, d, L, d), (1.5, -0.3)    # 壁の反対側へ30cm(同じ線上だと目的地が壁のコスト帯に入りうる)
    elif side == "right":
        walls, goal = _long(-L, -d, L, -d), (1.5, 0.3)
    else:  # front: 前が壁なので、後ろ側(開いている方)へ
        walls, goal = _long(d, -L, d, L), (-1.5, 0.0)
    return Scenario(f"{side}壁 端{int(round(edge * 100))}cm", side, edge, walls, goal,
                    note="目的地は開いた側へ1.5m" if side == "front" else "前方1.5m")


def corner(edge: float) -> Scenario:
    d = BODY_HALF + edge
    walls = _long(-d, -2.5, -d, d) + _long(-d, d, 3.0, d)
    # 後ろ壁(x=-d)と左壁(y=d)の隅。目的地は前方1.5m
    return Scenario(f"隅(後+左) 端{int(round(edge * 100))}cm", "corner", edge, walls, (1.5, 0.0))


def ushape(edge: float) -> Scenario:
    d = BODY_HALF + edge
    walls = _long(-d, -d, -d, d) + _long(-d, d, 2.0, d) + _long(-d, -d, 2.0, -d)
    return Scenario(f"コの字の奥 端{int(round(edge * 100))}cm", "ushape", edge, walls, (3.0, 0.0),
                    note="幅は機体+両側の余白、奥行き2m")


def corridor(width_m: float) -> Scenario:
    h = width_m / 2.0
    walls = _long(-2.0, h, 3.5, h) + _long(-2.0, -h, 3.5, -h)
    return Scenario(f"通路 幅{int(round(width_m * 100))}cm", "corridor", (width_m - 2 * BODY_HALF) / 2.0, walls, (1.5, 0.0))


def all_scenarios() -> List[Scenario]:
    out: List[Scenario] = []
    for side in ("back", "left", "right", "front"):
        for e in (0.0, 0.05, 0.10, 0.15, 0.20, 0.30):
            out.append(single_wall(side, e))
    for e in (0.05, 0.10, 0.15, 0.20, 0.30):
        out.append(corner(e))
    for e in (0.05, 0.10, 0.15, 0.20, 0.30):
        out.append(ushape(e))
    for w in (0.60, 0.80, 1.00):
        out.append(corridor(w))
    out.extend(real_log_scenarios())
    return out


def real_log_scenarios() -> List[Scenario]:
    """実機のログ: 後ろの壁まで0.36m(LiDAR中心から)=機体端10cm。目的地は前方1.5m。
    実機では計画が失敗した。素直なモデル(bias 0)では成功するため、「コストマップ上で壁が実際より10cm近く見える」
    (位置ずれ・障害物の太り・LiDAR取り付け位置の誤差)を足した版を並べる(10cmでもまだ成功、20cmで失敗=原因の推定)。"""
    out = []
    for side, bias in (("back", 0.0), ("back", 0.10), ("back", 0.20), ("left", 0.20), ("right", 0.20), ("front", 0.20)):
        base = single_wall(side, 0.10)
        tag = "素直" if bias == 0 else f"壁{int(bias * 100)}cm近く見える"
        out.append(Scenario(f"実機ログ {side}0.36m({tag})", "real", 0.10, base.walls, base.goal, bias,
                            f"LiDAR中心から壁0.36m、コストマップ上は{0.36 - bias:.2f}m"))
    return out


def plan_scenario(sc: Scenario, infl: Inflation, start: Pose = (0.0, 0.0, 0.0)) -> PlanResult:
    cm = build_costmap(sc.walls, infl, sc.bounds, bias_m=sc.bias_m)
    return plan(cm, (start[0], start[1]), sc.goal)


# ---------------- 離脱(wall_escape)の模擬 ----------------
@dataclass
class EscapeResult:
    status: str                 # "not_needed" / "ok" / "failed"
    reason: str = ""
    t_clear: Optional[float] = None      # 壁から十分(CLEAR_M)離れるまでの秒
    t_plan_ok: Optional[float] = None    # 計画が成功する姿勢になるまでの秒
    collided: bool = False
    min_clearance_m: float = float("inf")  # 途中の、機体外形から壁までの最小距離
    final_pose: Pose = (0.0, 0.0, 0.0)
    steps: int = 0


def _to_robot(points: Sequence[Point], pose: Pose) -> List[Point]:
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return [(c * (px - x) + s * (py - y), -s * (px - x) + c * (py - y)) for px, py in points]


def _advance(pose: Pose, v: float, w: float, dt: float) -> Pose:
    dx, dy, dth = pose_at(v, w, dt)
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return x + c * dx - s * dy, y + s * dx + c * dy, th + dth


def run_escape(sc: Scenario, infl: Inflation, dt: float = 0.1, lidar_step: float = 0.05) -> EscapeResult:
    """wall_escape の動きを模擬: 近い点の逃げる向きflee→pick_motion(先読み)→(v,w)でdt秒進む、を繰り返す。
    min_range>=CLEAR_Mで終了(ノードと同じ)。MAX_SEC秒で時間切れ。毎ステップ、本当の壁に角が当たっていないか確認する。"""
    cm = build_costmap(sc.walls, infl, sc.bounds, bias_m=sc.bias_m)
    lidar = sc.walls[:: max(1, int(round(lidar_step / 0.02)))]
    pose: Pose = (0.0, 0.0, 0.0)
    res = EscapeResult("failed")
    t = 0.0
    if plan(cm, (pose[0], pose[1]), sc.goal).ok and min_range(_polar(_to_robot(lidar, pose), RADIUS_M)) is not None \
            and min_range(_polar(_to_robot(lidar, pose), RADIUS_M)) >= TRIGGER_M:
        return EscapeResult("not_needed", "壁が遠く、計画も成功")
    n = int(MAX_SEC / dt)
    for i in range(n + 1):
        pr = _polar(_to_robot(lidar, pose), RADIUS_M)
        m = min_range(pr)
        if res.t_plan_ok is None and plan(cm, (pose[0], pose[1]), sc.goal).ok:
            res.t_plan_ok = t
        if m is None or m >= CLEAR_M:
            res.t_clear = t
            res.status, res.final_pose, res.steps = "ok", pose, i
            if res.t_plan_ok is None:
                res.status, res.reason = "failed", "壁からは離れたが、その姿勢でも計画が成功しない"
            return res
        flee = flee_angle(pr, RADIUS_M)
        if flee is None:
            res.reason, res.final_pose, res.steps = "逃げる向きがありません", pose, i
            return res
        picked = pick_motion(to_xy(pr), flee, LIN, ANG)
        if picked is None:
            res.reason, res.final_pose, res.steps = "角が当たらずに動ける向きがありません", pose, i
            return res
        pose = _advance(pose, picked[0], picked[1], dt)
        t += dt
        c = min_clearance(_to_robot(sc.walls, pose), (0.0, 0.0, 0.0), BODY_HALF)
        res.min_clearance_m = min(res.min_clearance_m, c)
        if c <= 0.0:
            res.collided = True
            res.reason, res.final_pose, res.steps = f"{t:.1f}秒で角が壁に当たった", pose, i + 1
            return res
    res.reason, res.final_pose, res.steps = f"時間切れ({MAX_SEC:.0f}秒でも離れきれない)", pose, n
    return res


def _polar(points_xy: Sequence[Point], max_range: float) -> List[Tuple[float, float]]:
    out = []
    for x, y in points_xy:
        r = math.hypot(x, y)
        if 0.05 <= r <= max_range:
            out.append((r, math.atan2(y, x)))
    return out


# ---------------- 比較・表 ----------------
def compare_table(small_half: float = 0.21) -> List[dict]:
    """各シナリオで、現状(半幅0.26)と、経路計画だけ小さく見積もった場合(半幅small_half)の計画成否を返す。"""
    cur, small = inflation_for(BODY_HALF), inflation_for(small_half)
    rows = []
    for sc in all_scenarios():
        a, b = plan_scenario(sc, cur), plan_scenario(sc, small)
        rows.append({"sc": sc, "cur": a, "small": b})
    return rows


def format_table(rows: List[dict], escapes: Optional[Dict[str, EscapeResult]] = None) -> str:
    lines = [f"{'シナリオ':<32}{'現状0.26':>9}{'小0.21':>8}  離脱"]
    for r in rows:
        sc = r["sc"]
        e = (escapes or {}).get(sc.name)
        et = ""
        if e is not None:
            if e.status == "not_needed":
                et = "不要"
            elif e.status == "ok":
                et = f"出られる t_clear={e.t_clear:.1f}s 計画成功まで={e.t_plan_ok:.1f}s 最小余白={e.min_clearance_m * 100:.0f}cm"
            else:
                et = f"出られない: {e.reason}"
        elif not r["cur"].ok:
            et = f"(計画失敗の理由: {r['cur'].reason})"
        lines.append(f"{sc.name:<32}{'成功' if r['cur'].ok else '失敗':>9}{'成功' if r['small'].ok else '失敗':>8}  {et}")
    return "\n".join(lines)
