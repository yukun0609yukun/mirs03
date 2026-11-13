"""経路を実行する前に、機体の外形(52cm角)が障害物(LiDARの点)に当たらず越せるかをシミュレーションする(2026-10-25)。ROS不要の純粋な関数だけ。

背景: BTの回復で後退したあと、経路を作り直しても、その経路が障害物を越せるか確かめていなかったため、何度も下がっていた(docs/detour.md)。
ここでは「経路を実際にたどったら当たるか」を動かす前に判定し、越せると分かった経路だけを実行する。迂回の中継点の候補も作る。

座標はすべてmap座標(x, y [m]、向きyaw [rad])。障害物はLiDARの点(静止している前提)をmap座標に直したもの。
機体は「先頭で経路の向きへその場旋回 -> 経路を細かく(既定0.05 m刻み)たどる。向きが5度以上変わる所ではその場で旋回(5度刻み)」と動くと仮定する
(純追跡より保守的: 実機は滑らかに曲がるので、ここで当たらなければ実機も当たらない側)。

安全側の約束:
- 入力が検証できないとき(経路が空・機体の姿勢がNaN/Inf)は、衝突位置0.0を返す(=越せないと扱う)。点のNaN/Infは無視する(点が無ければ障害物なし=None)。
- 出発の時点で点が外形(+margin)の中にあるときも0.0。
"""
from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

from .motion_sim import BODY_HALF_M

Point = Tuple[float, float]
Pose = Tuple[float, float, float]

PATH_STEP_M = 0.05                    # 経路に沿った判定の刻み
ROT_STEP_RAD = math.radians(5.0)      # その場旋回の判定の刻み
LOOKAHEAD_M = 4.0
DEFAULT_MARGIN_M = 0.02
CIRCUMSCRIBED_M = BODY_HALF_M * math.sqrt(2.0)   # 外接半径 0.368 m


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def clean_points(points: Optional[Sequence[Point]]) -> List[Point]:
    """NaN/Infや形の悪い点を捨てる。"""
    out: List[Point] = []
    for p in points or ():
        try:
            x, y = float(p[0]), float(p[1])
        except (TypeError, ValueError, IndexError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            out.append((x, y))
    return out


def decimate_points(points: Sequence[Point], max_points: int = 150) -> List[Point]:
    """点を格子ごとに1点へ間引く(CPU節約)。格子の大きさをmax_points以下になるまで広げる。薄い障害物が丸ごと消えないよう、単純な等間隔間引きにはしない。"""
    pts = clean_points(points)
    if len(pts) <= max_points:
        return pts
    cell = 0.03
    while True:
        seen = {}
        for x, y in pts:
            seen.setdefault((int(math.floor(x / cell)), int(math.floor(y / cell))), (x, y))
        if len(seen) <= max_points or cell > 2.0:
            return list(seen.values())[:max_points]
        cell *= 1.3


def _resample(path: Sequence[Point], step: float) -> List[Tuple[float, float, float]]:
    """折れ線を、弧長step刻みの点(x, y, 弧長s)に直す。"""
    out: List[Tuple[float, float, float]] = [(path[0][0], path[0][1], 0.0)]
    s_total = 0.0
    carry = 0.0   # 直前の出力点から現在の線分の始点までの距離
    for i in range(1, len(path)):
        ax, ay = path[i - 1]
        bx, by = path[i]
        seg = math.hypot(bx - ax, by - ay)
        if seg < 1e-9:
            continue
        pos = step - carry
        while pos <= seg + 1e-9:
            f = pos / seg
            s_total_pt = s_total + pos
            out.append((ax + (bx - ax) * f, ay + (by - ay) * f, s_total_pt))
            pos += step
        carry = seg - (pos - step)
        s_total += seg
    last = path[-1]
    if math.hypot(out[-1][0] - last[0], out[-1][1] - last[1]) > 1e-6:
        out.append((last[0], last[1], s_total))
    return out


class _Body:
    """点群に対して、姿勢(x, y, yaw)の外形(半幅half+margin)が当たるかを調べる。"""

    def __init__(self, points: Sequence[Point], half: float, margin: float) -> None:
        self.points = points
        self.h = half + margin
        self.r2 = (self.h * math.sqrt(2.0)) ** 2

    def hit(self, x: float, y: float, yaw: float) -> bool:
        c, s = math.cos(yaw), math.sin(yaw)
        h, r2 = self.h, self.r2
        for px, py in self.points:
            dx, dy = px - x, py - y
            if dx * dx + dy * dy > r2:   # 外接円の外 -> 当たらない(安い判定)
                continue
            if abs(c * dx + s * dy) <= h and abs(-s * dx + c * dy) <= h:
                return True
        return False


def _rotation_hit(body: _Body, x: float, y: float, yaw_from: float, yaw_to: float) -> bool:
    """その場でyaw_fromからyaw_toへ(短い方へ)5度刻みで旋回して、途中でも当たるか。"""
    d = _wrap(yaw_to - yaw_from)
    n = max(1, int(math.ceil(abs(d) / ROT_STEP_RAD)))
    for i in range(n + 1):
        if body.hit(x, y, yaw_from + d * i / n):
            return True
    return False


def initial_rotation_ok(
    robot_pose: Pose, target_yaw: float, scan_points_map: Sequence[Point],
    half: float = BODY_HALF_M, margin: float = DEFAULT_MARGIN_M,
) -> bool:
    """出発の姿勢から、target_yawまで、その場で(短い方へ)旋回しても角が壁に当たらないか。
    外接半径(0.368 m)の外の点は調べず、内側の点は外形(正方形)を5度刻みで回して調べる。入力が不正ならFalse。"""
    try:
        x, y, yaw = (float(robot_pose[0]), float(robot_pose[1]), float(robot_pose[2]))
        tgt = float(target_yaw)
    except (TypeError, ValueError, IndexError):
        return False
    if not all(math.isfinite(v) for v in (x, y, yaw, tgt)):
        return False
    body = _Body(clean_points(scan_points_map), half, margin)
    return not _rotation_hit(body, x, y, yaw, tgt)


def _headings(samples: Sequence[Tuple[float, float, float]], chord: float = 0.25) -> List[float]:
    """各サンプルの進行方向。弧長chord先の点への向き(角で急に変わらないよう少し先を見る)。終端は直前の向きを保つ。"""
    n = len(samples)
    out: List[float] = [0.0] * n
    j = 0
    last = 0.0
    for i in range(n):
        j = max(j, i)
        while j < n - 1 and math.hypot(samples[j][0] - samples[i][0], samples[j][1] - samples[i][1]) < chord:
            j += 1
        dx, dy = samples[j][0] - samples[i][0], samples[j][1] - samples[i][1]
        if math.hypot(dx, dy) > 1e-6:
            last = math.atan2(dy, dx)
        out[i] = last
    return out


def first_collision_on_path(
    path_xy: Sequence[Point], robot_pose: Pose, scan_points_map: Sequence[Point], speed: float = 0.0,
    half: float = BODY_HALF_M, margin: float = DEFAULT_MARGIN_M, lookahead_m: float = LOOKAHEAD_M,
) -> Optional[float]:
    """経路に沿って動いたとき、機体の外形が最初にLiDARの点に当たる位置(経路の始点からの距離[m])。lookahead_mまでで当たらなければNone。

    動き: 先頭で経路の最初の向きへその場旋回(5度刻み) -> 経路を0.05 m刻みでたどる(向きが5度以上変わる所ではその場で旋回)。
    path_xyが空/機体の姿勢が不正なら0.0(検証できない=越せない扱い)。speedは将来の拡張用で、いまは使わない。
    出発の旋回で当たるときも0.0。"""
    try:
        rx, ry, ryaw = (float(robot_pose[0]), float(robot_pose[1]), float(robot_pose[2]))
    except (TypeError, ValueError, IndexError):
        return 0.0
    if not all(math.isfinite(v) for v in (rx, ry, ryaw)):
        return 0.0
    path = clean_points(path_xy)
    if not path:
        return 0.0
    if math.hypot(path[0][0] - rx, path[0][1] - ry) > 0.02:
        path.insert(0, (rx, ry))   # 経路の始点が機体と離れていれば、機体の位置から始める
    lookahead = max(0.0, float(lookahead_m)) if math.isfinite(lookahead_m) else LOOKAHEAD_M
    reach = lookahead + 1.0
    near = [p for p in clean_points(scan_points_map) if math.hypot(p[0] - rx, p[1] - ry) <= reach + CIRCUMSCRIBED_M]
    body = _Body(near, half, margin)
    all_samples = _resample(path, PATH_STEP_M)
    heads_all = _headings(all_samples)
    keep = [i for i, smp in enumerate(all_samples) if smp[2] <= lookahead + 1e-9]
    samples = [all_samples[i] for i in keep]
    heads = [heads_all[i] for i in keep]
    if len(samples) < 2:
        # 経路が実質ゼロ長 -> その場の姿勢だけ確かめる
        return 0.0 if body.hit(rx, ry, ryaw) else None
    # 出発: その場で最初の向きへ旋回
    if _rotation_hit(body, rx, ry, ryaw, heads[0]):
        return 0.0
    prev_yaw = heads[0]
    for (x, y, s), yaw in zip(samples, heads):
        if abs(_wrap(yaw - prev_yaw)) > ROT_STEP_RAD and _rotation_hit(body, x, y, prev_yaw, yaw):
            return s   # 角でその場旋回するとき(終端の姿勢も含む)
        if body.hit(x, y, yaw):
            return s
        prev_yaw = yaw
    return None


def path_heading(path_xy: Sequence[Point], robot_pose: Pose, chord: float = 0.5) -> Optional[float]:
    """経路の最初の進行方向(map座標のrad)。機体の位置からchord先の経路上の点への向き。経路が短い/不正ならNone。"""
    path = clean_points(path_xy)
    try:
        rx, ry = float(robot_pose[0]), float(robot_pose[1])
    except (TypeError, ValueError, IndexError):
        return None
    if not path or not math.isfinite(rx) or not math.isfinite(ry):
        return None
    for px, py in path:
        if math.hypot(px - rx, py - ry) >= chord:
            return math.atan2(py - ry, px - rx)
    px, py = path[-1]
    if math.hypot(px - rx, py - ry) < 0.05:
        return None
    return math.atan2(py - ry, px - rx)


def detour_waypoints(
    obstacle_points: Sequence[Point], robot_pose: Pose, goal_xy: Point,
    offsets: Sequence[float] = (0.7, 1.0, 1.4), half: float = BODY_HALF_M, margin: float = DEFAULT_MARGIN_M,
    cluster_gap: float = 0.35, front_range: float = 3.0, side_range: float = 2.0,
) -> List[Point]:
    """前方の障害物の塊を避ける、左右の中継点(map座標)の候補を、近い順(機体->中継点->目的地の直線距離の合計が小さい順)に返す。

    obstacle_points: map座標のLiDARの点。目的地の方向(目的地が0.3 m以内なら機体の向き)を「前」とする座標に直し、
    前方front_range・左右side_range以内の点を、cluster_gap以内でつながるものを1つの塊にまとめる(隙間が機体幅より狭ければ1つの塊)。
    進路(横幅 half+margin+0.1 の帯)に入る、いちばん近い塊の左端・右端から、機体半幅+余白+offsetだけ横へずらした点を、塊の前後の中央に置く。
    進路をふさぐ塊が無い/入力が不正なら空。"""
    pts = clean_points(obstacle_points)
    try:
        rx, ry, ryaw = (float(robot_pose[0]), float(robot_pose[1]), float(robot_pose[2]))
        gx, gy = float(goal_xy[0]), float(goal_xy[1])
    except (TypeError, ValueError, IndexError):
        return []
    if not all(math.isfinite(v) for v in (rx, ry, ryaw, gx, gy)):
        return []
    fwd = math.atan2(gy - ry, gx - rx) if math.hypot(gx - rx, gy - ry) > 0.3 else ryaw
    c, s = math.cos(fwd), math.sin(fwd)
    local = []   # 「前」を+xとした座標(原点=機体)
    for px, py in pts:
        dx, dy = px - rx, py - ry
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        if 0.0 < lx <= front_range and abs(ly) <= side_range:
            local.append((lx, ly))
    if not local:
        return []
    # 単連結でクラスタ化
    local.sort()
    parent = list(range(len(local)))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    g2 = cluster_gap * cluster_gap
    for i in range(len(local)):
        for j in range(i + 1, len(local)):
            if local[j][0] - local[i][0] > cluster_gap:
                break
            if (local[i][0] - local[j][0]) ** 2 + (local[i][1] - local[j][1]) ** 2 <= g2:
                parent[find(j)] = find(i)
    clusters = {}
    for i, p in enumerate(local):
        clusters.setdefault(find(i), []).append(p)
    lane = half + margin + 0.1
    blocking = [cl for cl in clusters.values() if any(abs(y) <= lane for _, y in cl)]
    if not blocking:
        return []
    cl = min(blocking, key=lambda q: min(x for x, _ in q))
    ys = [y for _, y in cl]
    xs = [x for x, _ in cl]
    xc = 0.5 * (min(xs) + max(xs))
    left_edge, right_edge = max(ys), min(ys)
    cands = []
    for off in offsets:
        if not math.isfinite(off) or off < 0:
            continue
        lat = half + margin + off
        cands.append((xc, left_edge + lat))     # 左(+y)
        cands.append((xc, right_edge - lat))    # 右(-y)
    out = []
    for lx, ly in cands:
        mx, my = rx + c * lx - s * ly, ry + s * lx + c * ly
        cost = math.hypot(mx - rx, my - ry) + math.hypot(gx - mx, gy - my)
        out.append((cost, len(out), (mx, my)))
    out.sort(key=lambda t: (t[0], t[1]))   # 同じ距離なら左が先(作った順)
    return [p for _, _, p in out]
