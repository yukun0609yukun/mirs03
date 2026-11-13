"""走行可能範囲(回転矩形)の中で、機体が外へ出ないことを「動く前に計算して」確かめる(純Python、ROS非依存)。

behavior_directorが直接/cmd_velへ出す動き(見回しの旋回・直進、待機の左右振動)は、Nav2の経路計画を通らないため、
範囲を予測して止める仕組みが無かった(範囲の外に出てから OUT_OF_BOUNDS_HOLD で止まる)。ここでは、機体の外形
(52 cm角、LiDAR中心)を使い、動く前に次を計算する。

- その場旋回: 旋回中の機体の最大の広がり(外接円、半径0.368 m)+余白が、範囲の内側に収まるか。
- 直進/後退: 先読みの距離(止まるまでの余裕を含む)まで、0.05 m刻みで機体の4隅が範囲の内側に収まるか。

位置が分からない・範囲が読めないときは、安全側(動かない)にする。docs/behavior_tree.md の「走行可能範囲との関係」。
"""
from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

from .map_orientation import point_in_rect

ROBOT_HALF_WIDTH_M = 0.26                                  # 52 cm角(docs/nav2_clearance_and_recovery.md)
ROBOT_CIRCUM_RADIUS_M = ROBOT_HALF_WIDTH_M * math.sqrt(2.0)  # 約0.368 m(その場で旋回したときの最大の広がり)
SAFETY_MARGIN_M = 0.05                                     # 位置の誤差・遅れの余白
ROTATE_MARGIN_M = ROBOT_CIRCUM_RADIUS_M + SAFETY_MARGIN_M  # 旋回してよい最小の縁からの距離(約0.42 m)
GOAL_MARGIN_M = ROTATE_MARGIN_M                            # 目的地は縁からこれだけ内側へ(到着時の旋回で出ない)
LOOKAHEAD_M = 0.35                                         # 直進の先読み(止まるまでの余裕。0.5 m/s・遅れ0.2秒+停止分)
STEP_M = 0.05

Pose = Dict[str, float]


def footprint_corners(x: float, y: float, yaw: float) -> Tuple[Tuple[float, float], ...]:
    """機体(52 cm角)の4隅の世界座標。"""
    c, s = math.cos(yaw), math.sin(yaw)
    h = ROBOT_HALF_WIDTH_M
    return tuple(
        (x + lx * c - ly * s, y + lx * s + ly * c)
        for lx, ly in ((h, h), (h, -h), (-h, -h), (-h, h))
    )


def footprint_inside(rect: Dict[str, float], x: float, y: float, yaw: float, margin: float = SAFETY_MARGIN_M) -> bool:
    """機体の4隅が、範囲を margin だけ狭めた内側に全部ある。"""
    return all(point_in_rect(px, py, rect, margin=-margin) for px, py in footprint_corners(x, y, yaw))


def can_rotate_in_place(rect: Optional[Dict[str, float]], pose: Optional[Pose]) -> bool:
    """その場で旋回しても外へ出ないか(外接円+余白が範囲の内側)。範囲が無ければ常にTrue。"""
    if rect is None:
        return True
    if not _pose_ok(pose):
        return False
    return point_in_rect(pose["x"], pose["y"], rect, margin=-ROTATE_MARGIN_M)


def safe_drive_distance(
    rect: Optional[Dict[str, float]], pose: Optional[Pose], max_dist: float, direction: float = 1.0
) -> float:
    """今の向きのまま直進(direction=1)/後退(-1)したとき、機体が範囲の内側に収まる最大の距離[m]。
    範囲が無ければ max_dist。位置が分からなければ0。今すでに収まっていなければ0。"""
    if rect is None:
        return max(0.0, float(max_dist))
    if not _pose_ok(pose):
        return 0.0
    d = 1.0 if direction >= 0 else -1.0
    x0, y0, yaw = pose["x"], pose["y"], pose["yaw"]
    if not footprint_inside(rect, x0, y0, yaw):
        return 0.0
    safe = 0.0
    n = int(math.floor(max(0.0, float(max_dist)) / STEP_M + 1e-9))
    for i in range(1, n + 1):
        dist = i * STEP_M
        if not footprint_inside(rect, x0 + d * dist * math.cos(yaw), y0 + d * dist * math.sin(yaw), yaw):
            return safe
        safe = dist
    return safe


def can_drive(rect: Optional[Dict[str, float]], pose: Optional[Pose], direction: float = 1.0) -> bool:
    """今の1歩(先読み LOOKAHEAD_M)を進んでも外へ出ないか。"""
    if rect is None:
        return True
    return safe_drive_distance(rect, pose, LOOKAHEAD_M, direction) >= LOOKAHEAD_M - 1e-9


def _pose_ok(pose: Optional[Pose]) -> bool:
    if not isinstance(pose, dict):
        return False
    try:
        return all(math.isfinite(float(pose[k])) for k in ("x", "y", "yaw"))
    except (KeyError, TypeError, ValueError):
        return False


# ---------------------------------------------------------------- Nav2の経路(/plan)が範囲の外へ出るか
PATH_EXIT_TOL_M = 0.05      # 範囲をこれだけ広げた矩形の外だけを「外」とする(計画の誤差・縁沿いの経路を止めない)
PATH_EXIT_MIN_RUN_M = 0.25  # 外の点が連続してこの長さ以上のときだけ「出る」(一瞬のはみ出しは止めない)


def path_exit_info(points, rect, tol: float = PATH_EXIT_TOL_M, min_run_m: float = PATH_EXIT_MIN_RUN_M) -> dict:
    """経路の中心線 points=[(x,y),...] が、範囲 rect の外へ出るか。誤停止を避けるため、迷うときは exits=False。

    - 範囲を tol だけ広げた矩形の外にある点を「外」とする。
    - 先頭から連続する外の点(ロボットがすでに範囲の外にいて、戻る区間)は数えない。
    - それ以降に、外の点が連続する区間の長さ(隣り合う点の距離の合計)が min_run_m 以上あれば exits=True。
    - rect が None、有効な点が2未満、非有限の点(その点は無視)は exits=False 側。
    戻り値: dict(exits, run_m(最長の外の区間), first_xy(その区間の最初の点。無ければ None))。
    """
    result = {"exits": False, "run_m": 0.0, "first_xy": None}
    if rect is None:
        return result
    pts = []
    for p in points or ():
        try:
            x, y = float(p[0]), float(p[1])
        except (TypeError, ValueError, IndexError, KeyError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            pts.append((x, y))
    if len(pts) < 2:
        return result
    inside = [point_in_rect(x, y, rect, margin=tol) for x, y in pts]
    if True not in inside:
        return result  # 全部外: 内側へ入る点が無い(位置が合っていない等)ので判断しない
    i = inside.index(True)  # 最初の内側の点より前は無視
    best_run, best_first = 0.0, None
    run, first = 0.0, None
    for k in range(i + 1, len(pts)):
        if inside[k]:
            run, first = 0.0, None
            continue
        if inside[k - 1]:
            run, first = 0.0, pts[k]
        else:
            run += math.hypot(pts[k][0] - pts[k - 1][0], pts[k][1] - pts[k - 1][1])
        if run > best_run or (best_first is None and first is not None and run >= best_run):
            best_run, best_first = run, first
    result["run_m"] = best_run
    result["first_xy"] = best_first
    result["exits"] = best_run >= min_run_m - 1e-9
    return result
