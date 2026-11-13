"""壁際からの脱出で使う「壁を避ける中継点」の探索(2026-10-22)。ROS不要の純粋な関数だけ。

グローバルコストマップ(nav_msgs/OccupancyGrid相当: data=行優先の0..100、-1=未知)と機体位置・目的地(いずれもmap座標)から、
「機体から半径radius_m以内で、コストが低く、周囲に致死・内接・未知が無く(壁から機体端でedge_clear_m以上)、目的地の方向に近い」
最寄りの空きマスを探す。
"""
from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

BODY_HALF_M = 0.26        # 機体は52cm角
OBSTACLE_COST = 99        # OccupancyGridの内接(99)・致死(100)以上は障害物扱い。未知(-1)も障害物扱い


class GridInfo:
    """OccupancyGridの必要最小限の情報。data は行優先(index = row*width + col)。"""

    def __init__(self, data: Sequence[int], width: int, height: int, resolution: float,
                 origin_x: float, origin_y: float) -> None:
        self.data = data
        self.width = int(width)
        self.height = int(height)
        self.res = float(resolution)
        self.ox = float(origin_x)
        self.oy = float(origin_y)

    def cell_of(self, x: float, y: float) -> Tuple[int, int]:
        return int(math.floor((x - self.ox) / self.res)), int(math.floor((y - self.oy) / self.res))

    def center_of(self, cx: int, cy: int) -> Tuple[float, float]:
        return self.ox + (cx + 0.5) * self.res, self.oy + (cy + 0.5) * self.res

    def cost(self, cx: int, cy: int) -> int:
        """範囲外は未知(-1)。"""
        if cx < 0 or cy < 0 or cx >= self.width or cy >= self.height:
            return -1
        return int(self.data[cy * self.width + cx])


def _is_obstacle(c: int) -> bool:
    return c < 0 or c >= OBSTACLE_COST


def find_escape_waypoint(
    grid: GridInfo, robot_xy: Tuple[float, float], goal_xy: Optional[Tuple[float, float]] = None,
    radius_m: float = 1.0, max_cost: int = 50, edge_clear_m: float = 0.25, half: float = BODY_HALF_M,
    min_dist_m: float = 0.15, goal_angle_weight: float = 0.5,
) -> Optional[Tuple[float, float]]:
    """条件を満たす最寄りの空きマスの中心(map座標)。なければNone。

    条件: (1)機体からradius_m以内(min_dist_m以上) (2)そのマスのコストが0..max_cost (3)マスの中心から半径(half+edge_clear_m)
    以内に障害物(コスト>=99、未知、範囲外)が無い=壁から機体端でedge_clear_m以上 (4)機体からそのマスまでの直線上に障害物が無い
    (機体の近く=half以内は、壁際の出発点なので見ない)。
    順位: 距離[m] + goal_angle_weight*(目的地の方向との角度差[rad])が小さいもの。目的地と逆向き(90度超)は候補から外す。
    goalがNoneなら最寄り。"""
    rx, ry = robot_xy
    res = grid.res
    clear_cells = int(math.ceil((half + edge_clear_m) / res))
    clear_r = half + edge_clear_m
    # 近傍オフセット(円盤)
    disc = [(dx, dy) for dx in range(-clear_cells, clear_cells + 1) for dy in range(-clear_cells, clear_cells + 1)
            if math.hypot(dx, dy) * res <= clear_r]
    # 外側から先に調べると壁に当たりやすく早く打ち切れる
    disc.sort(key=lambda d: -math.hypot(*d))
    goal_dir = None
    if goal_xy is not None and math.hypot(goal_xy[0] - rx, goal_xy[1] - ry) > 1e-6:
        goal_dir = math.atan2(goal_xy[1] - ry, goal_xy[0] - rx)

    r_cells = int(math.ceil(radius_m / res))
    c0x, c0y = grid.cell_of(rx, ry)
    best = None
    best_score = float("inf")
    for cy in range(c0y - r_cells, c0y + r_cells + 1):
        for cx in range(c0x - r_cells, c0x + r_cells + 1):
            x, y = grid.center_of(cx, cy)
            dist = math.hypot(x - rx, y - ry)
            if dist > radius_m or dist < min_dist_m:
                continue
            c = grid.cost(cx, cy)
            if c < 0 or c > max_cost:
                continue
            score = dist
            if goal_dir is not None:
                ang = abs(math.atan2(math.sin(math.atan2(y - ry, x - rx) - goal_dir),
                                     math.cos(math.atan2(y - ry, x - rx) - goal_dir)))
                if ang > math.pi / 2:
                    continue
                score += goal_angle_weight * ang
            if score >= best_score:
                continue
            if any(_is_obstacle(grid.cost(cx + dx, cy + dy)) for dx, dy in disc):
                continue
            if not _line_free(grid, rx, ry, x, y, half):
                continue
            best_score, best = score, (x, y)
    return best


def _line_free(grid: GridInfo, x0: float, y0: float, x1: float, y1: float, skip_near: float) -> bool:
    d = math.hypot(x1 - x0, y1 - y0)
    n = max(1, int(d / (grid.res * 0.5)))
    for i in range(1, n + 1):
        t = i / n
        if d * t <= skip_near:
            continue
        cx, cy = grid.cell_of(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
        if _is_obstacle(grid.cost(cx, cy)):
            return False
    return True
