"""機体の外形(正方形)を使った、先読みの衝突シミュレーション(2026-10-22)。ROS不要の純粋な関数だけ。

「今どこにいて、壁(LiDARの点)までどれくらいで、この速さ・舵でこう動くと、機体が何度回って、どの辺に角が来るか → 当たるか」を、
動かす前に計算する。壁際で自分から動く処理(wall_escape・先回し旋回の代替の回頭)が、角を壁にぶつけないために使う。

座標は機体座標(原点=LiDAR=機体中心、x前・y左)。障害物はLiDARの点(静止している前提)。機体は等速の(直進v, 旋回w)で動くとして、
dt刻みで姿勢を積分し、各時刻で「点が機体の外形(半幅half+margin)に入っているか」を調べる。
"""
from __future__ import annotations

import math
from typing import Iterable, List, Optional, Sequence, Tuple

Point = Tuple[float, float]
BODY_HALF_M = 0.26  # 機体は52cm角(nav_speed.BODY_HALF_Mと同じ)


def to_xy(points_polar: Iterable[Tuple[float, float]]) -> List[Point]:
    """(距離, 角度rad)のリストを、機体座標(x, y)に。"""
    return [(r * math.cos(a), r * math.sin(a)) for r, a in points_polar]


def pose_at(v: float, w: float, t: float) -> Tuple[float, float, float]:
    """等速(v, w)でt秒動いた後の姿勢(x, y, th)。原点・向き0から。"""
    if abs(w) < 1e-6:
        return v * t, 0.0, 0.0
    th = w * t
    return v / w * math.sin(th), v / w * (1.0 - math.cos(th)), th


def inside_body(px: float, py: float, pose: Tuple[float, float, float], half: float) -> bool:
    """点(px, py)(出発時の機体座標)が、姿勢poseの機体の外形(半幅half)の中か。"""
    x, y, th = pose
    dx, dy = px - x, py - y
    c, s = math.cos(th), math.sin(th)
    lx = c * dx + s * dy     # 機体座標へ(-th回転)
    ly = -s * dx + c * dy
    return abs(lx) <= half and abs(ly) <= half


def first_collision_time(
    points: Sequence[Point], v: float, w: float, horizon: float = 1.5, dt: float = 0.05,
    half: float = BODY_HALF_M, margin: float = 0.02,
) -> Optional[float]:
    """動かすと、何秒後に最初に点が外形(+margin)に入るか。horizon内で当たらなければNone。
    出発時にすでに点が外形内にあるときは、0.0(そのまま動くとめり込んだまま)を返す。"""
    h = half + margin
    n = int(math.ceil(horizon / dt))
    for i in range(n + 1):
        t = i * dt
        x, y, th = pose_at(v, w, t)
        # 【2026-10-24】cos/sinはステップごとに1回だけ(点の数に関係なく)。点ごとにinside_bodyを呼ぶと、点数 x ステップ数回も計算していた。
        # 判定はinside_bodyと同じ式(結果は同じ)。
        c, s = math.cos(th), math.sin(th)
        for px, py in points:
            dx, dy = px - x, py - y
            if abs(c * dx + s * dy) <= h and abs(-s * dx + c * dy) <= h:
                return t
    return None


def min_clearance(points: Sequence[Point], pose: Tuple[float, float, float], half: float = BODY_HALF_M) -> float:
    """姿勢poseの機体の外形(正方形)から、最も近い点までの距離[m](めり込んでいれば0)。"""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    best = float("inf")
    for px, py in points:
        dx, dy = px - x, py - y
        lx, ly = abs(c * dx + s * dy), abs(-s * dx + c * dy)
        ox, oy = max(lx - half, 0.0), max(ly - half, 0.0)
        best = min(best, math.hypot(ox, oy))
    return best


def default_candidates(linear: float, angular: float) -> List[Tuple[float, float]]:
    """壁際から動くための候補(直進v, 旋回w)。前進・後退・その場旋回・弧。"""
    cands: List[Tuple[float, float]] = [(0.0, 0.0)]
    for v in (linear, -linear):  # 後退も前進と同じ速さ(2026-10-23)
        for w in (-angular, -0.5 * angular, 0.0, 0.5 * angular, angular):
            cands.append((v, w))
    for w in (-angular, angular):
        cands.append((0.0, w))
    # 【壁沿いにすべる動き】前進/後退しながらの小さな旋回(壁に平行に進む弧・斜めの動き)。従来の候補が先頭。
    for v in (linear, 0.5 * linear, -linear, -0.5 * linear):
        for w in (-0.25 * angular, -0.12 * angular, 0.12 * angular, 0.25 * angular):
            cands.append((v, w))
    return cands


def pick_motion(
    points: Sequence[Point], flee: Optional[float], linear: float = 0.10, angular: float = 0.6,
    horizon: float = 1.5, half: float = BODY_HALF_M, margin: float = 0.02,
    goal_dir: Optional[float] = None, goal_weight: float = 2.0,
    waypoint_dir: Optional[float] = None, waypoint_weight: float = 4.0,
) -> Optional[Tuple[float, float]]:
    """候補の動きのうち、horizon秒の間に外形が点に当たらず、壁から離れる向きに進めるものを選ぶ。なければNone(動かない)。

    評価: (1)当たらない(first_collision_time is None)ものだけ (2)horizon後の最小クリアランスが大きい (3)fleeの向きに近い
    (v>0なら前向きの動き、v<0なら後ろ向きの動きとして)。静止(0,0)は、他に安全な動きがないときだけ。
    省略可能: goal_dir(目的地の向き, 機体座標rad)/waypoint_dir(壁を避ける中継点の向き)があれば、その向きへ進む成分
    (horizon後の移動量[m]の、その向きへの射影)に重みを掛けて加点する。当たる動きの除外は従来どおり先に行う(加点で安全は崩れない)。"""
    best = None
    best_score = -1e9
    for v, w in default_candidates(linear, angular):
        if v == 0.0 and w == 0.0:
            continue
        if first_collision_time(points, v, w, horizon, 0.05, half, margin) is not None:
            continue
        end = pose_at(v, w, horizon)
        clear = min_clearance(points, end, half)
        # 中間も含めた最小クリアランス(角が壁をかすめる動きを避ける)
        mid = min(min_clearance(points, pose_at(v, w, horizon * f), half) for f in (0.25, 0.5, 0.75))
        score = 2.0 * min(clear, mid) + 1.0 * min(clear, 1.0)
        if flee is not None:
            heading = end[2] + (0.0 if v >= 0 else math.pi)
            d = abs(math.atan2(math.sin(heading - flee), math.cos(heading - flee)))
            score -= 0.5 * d
        for dirn, wt in ((goal_dir, goal_weight), (waypoint_dir, waypoint_weight)):
            if dirn is not None:
                score += wt * (end[0] * math.cos(dirn) + end[1] * math.sin(dirn))
        if score > best_score:
            best_score, best = score, (v, w)
    return best
