"""範囲(travel_boundary)と壁との余白を一貫して扱う純粋関数(ROS不要、素のPython)。

「壁に近づきすぎると経路が作れず動けなくなる」問題への対策として、目的地・巡回点・探索の目標を
**壁(占有セル)から機体端で margin_m(既定0.30 m)以上**離れた場所に限る。

- compute_drivable_area(): 範囲の矩形 ∩ 「セル中心から最寄りの壁まで 機体の半幅+margin 以上離れた自由セル」
  を可走領域(DriveArea)として求める(正確なユークリッド距離変換)。
- plan_patrol_segments()/plan_patrol_path(): 可走領域全体を往復して覆う巡回経路
  (レーン間隔は既定0.6 m、各レーンは連続した可走区間に分割し、区間の端点を順に訪問する点列)。
- snap_to_drivable(): 目的地が可走領域外なら最寄りの可走セル中心へ寄せる(局所窓だけ計算するので軽い)。

占有格子はOccupancyGridと同じ並び(行0=origin_yの行、1要素=1セル、-1=未知、0=自由、100=占有)。
未知セルは「壁ではないが走ってはいけない」(可走にしない)。地図の外も不可。
壁までの距離は「セル中心〜占有セル中心」に1セル分(壁の面の半セル+点のずれ半セル)上乗せして測る(安全側)。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

OCCUPIED_MIN = 50          # この値以上は占有(map_pins.OCCUPIED_MINと同じ)
ROBOT_HALF_WIDTH_M = 0.26  # 機体(52 cm角)の半幅
WALL_MARGIN_M = 0.30       # 壁から機体端までの余白の既定
LANE_SPACING_M = 0.6       # 巡回のレーン間隔の既定(機体幅+余裕)
SNAP_MAX_SHIFT_M = 1.5     # 目的地を寄せる最大距離の既定

_INF = 1e12


@dataclass
class Grid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: Sequence[int]

    def valid(self) -> bool:
        try:
            return (
                self.width > 0 and self.height > 0 and self.resolution > 0
                and math.isfinite(self.resolution)
                and len(self.data) >= self.width * self.height
            )
        except (TypeError, ValueError):
            return False

    def cell_of(self, x: float, y: float) -> Tuple[int, int]:
        return (int(math.floor((x - self.origin_x) / self.resolution)),
                int(math.floor((y - self.origin_y) / self.resolution)))

    def center_of(self, c: int, r: int) -> Tuple[float, float]:
        return (self.origin_x + (c + 0.5) * self.resolution,
                self.origin_y + (r + 0.5) * self.resolution)


def grid_from_msg(msg: Any) -> Optional[Grid]:
    """nav_msgs/OccupancyGrid(またはそれに似たオブジェクト)からGridを作る。不正ならNone。"""
    try:
        g = Grid(int(msg.info.width), int(msg.info.height), float(msg.info.resolution),
                 float(msg.info.origin.position.x), float(msg.info.origin.position.y), msg.data)
    except Exception:  # noqa: BLE001
        return None
    return g if g.valid() else None


# ------------------------------------------------------------------ 距離変換
def _edt_1d(f: List[float], n: int) -> List[float]:
    """Felzenszwalb-Huttenlocherの1次元二乗距離変換。"""
    d = [0.0] * n
    v = [0] * n
    z = [0.0] * (n + 1)
    k = 0
    z[0], z[1] = -_INF, _INF
    for q in range(1, n):
        while True:
            p = v[k]
            s = ((f[q] + q * q) - (f[p] + p * p)) / (2.0 * q - 2.0 * p)
            if s <= z[k]:
                k -= 1   # z[0]=-INFなので k>=0 のまま
            else:
                break
        k += 1
        v[k] = q
        z[k] = s
        z[k + 1] = _INF
    k = 0
    for q in range(n):
        while z[k + 1] < q:
            k += 1
        p = v[k]
        d[q] = (q - p) * (q - p) + f[p]
    return d


def _edt_sq(occ: List[bytearray], w: int, h: int) -> List[List[float]]:
    """occ[r][c]==1 のセルまでの二乗距離[セル^2]。占有が1つも無ければ全て_INF。"""
    if not any(1 in row for row in occ):
        return [[_INF] * w for _ in range(h)]
    # 列方向 → 行方向
    tmp = [[0.0] * w for _ in range(h)]
    for c in range(w):
        col = [0.0 if occ[r][c] else _INF for r in range(h)]
        dc = _edt_1d(col, h)
        for r in range(h):
            tmp[r][c] = dc[r]
    return [_edt_1d(tmp[r], w) for r in range(h)]


# ------------------------------------------------------------------ 可走領域
@dataclass
class DriveArea:
    grid: Grid
    c0: int            # 窓の左下セル(grid内の絶対セル番号)
    r0: int
    w: int
    h: int
    mask: List[bytearray]   # mask[r-r0][c-c0] == 1 なら可走
    required_m: float       # 壁(占有セル)から必要な距離(半幅+margin)
    rect: Optional[Dict[str, float]] = None

    def cell_ok(self, c: int, r: int) -> bool:
        cc, rr = c - self.c0, r - self.r0
        return 0 <= cc < self.w and 0 <= rr < self.h and bool(self.mask[rr][cc])

    def point_ok(self, x: float, y: float) -> bool:
        c, r = self.grid.cell_of(x, y)
        if not self.cell_ok(c, r):
            return False
        return self.rect is None or _in_rect(x, y, self.rect)

    def cells(self) -> List[Tuple[int, int]]:
        return [(self.c0 + c, self.r0 + r) for r in range(self.h) for c in range(self.w) if self.mask[r][c]]

    def count(self) -> int:
        return sum(sum(row) for row in self.mask)


def _in_rect(x: float, y: float, rect: Dict[str, float]) -> bool:
    dx, dy = x - rect["center_x"], y - rect["center_y"]
    c, s = math.cos(rect.get("yaw", 0.0)), math.sin(rect.get("yaw", 0.0))
    lx, ly = dx * c + dy * s, -dx * s + dy * c
    tol = 1e-9 * max(1.0, rect["width"], rect["height"])
    return abs(lx) <= rect["width"] / 2.0 + tol and abs(ly) <= rect["height"] / 2.0 + tol


def _rect_bbox(rect: Dict[str, float]) -> Tuple[float, float, float, float]:
    hw, hh = rect["width"] / 2.0, rect["height"] / 2.0
    c, s = math.cos(rect.get("yaw", 0.0)), math.sin(rect.get("yaw", 0.0))
    xs, ys = [], []
    for lx, ly in ((-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)):
        xs.append(rect["center_x"] + lx * c - ly * s)
        ys.append(rect["center_y"] + lx * s + ly * c)
    return min(xs), max(xs), min(ys), max(ys)


def _window_area(grid: Grid, win: Tuple[int, int, int, int], required_m: float,
                 rect: Optional[Dict[str, float]]) -> DriveArea:
    """窓(c0, r0, c1, r1: 端を含む、grid内)の可走マスクを作る。壁は窓の外側 required分も見る。"""
    c0, r0, c1, r1 = win
    res = grid.resolution
    halo = int(math.ceil(required_m / res)) + 2
    ec0, er0 = max(0, c0 - halo), max(0, r0 - halo)
    ec1, er1 = min(grid.width - 1, c1 + halo), min(grid.height - 1, r1 + halo)
    ew, eh = ec1 - ec0 + 1, er1 - er0 + 1
    data, gw = grid.data, grid.width
    occ: List[bytearray] = []
    for r in range(er0, er1 + 1):
        base = r * gw
        row = bytearray(ew)
        for c in range(ec0, ec1 + 1):
            if data[base + c] >= OCCUPIED_MIN:
                row[c - ec0] = 1
        occ.append(row)
    d2 = _edt_sq(occ, ew, eh)
    # 壁の表面は占有セル中心より半セル手前、点はセル中心から最大で半セルずれる: 合わせて1セル分上乗せ(安全側)
    need_cells = required_m / res + 1.0
    need2 = need_cells * need_cells
    w, h = c1 - c0 + 1, r1 - r0 + 1
    mask: List[bytearray] = []
    for r in range(r0, r1 + 1):
        row = bytearray(w)
        base = r * gw
        drow = d2[r - er0]
        for c in range(c0, c1 + 1):
            v = data[base + c]
            if 0 <= v < OCCUPIED_MIN and drow[c - ec0] >= need2:
                if rect is None or _in_rect(*grid.center_of(c, r), rect):
                    row[c - c0] = 1
        mask.append(row)
    return DriveArea(grid, c0, r0, w, h, mask, required_m, rect)


def compute_drivable_area(
    grid: Grid,
    rect: Optional[Dict[str, float]] = None,
    margin_m: float = WALL_MARGIN_M,
    half_width_m: float = ROBOT_HALF_WIDTH_M,
) -> Optional[DriveArea]:
    """走行してよい領域: 範囲の矩形(Noneなら地図全体)の内側で、セル中心が壁から
    half_width_m+margin_m 以上離れた既知の自由セル。地図が不正ならNone。

    注意: 地図全体(rect=None)だと約40万セルで純Pythonでは遅い。巡回計画は範囲を指定して呼ぶこと。"""
    if grid is None or not grid.valid():
        return None
    required = max(0.0, half_width_m) + max(0.0, margin_m)
    if rect is None:
        win = (0, 0, grid.width - 1, grid.height - 1)
    else:
        x0, x1, y0, y1 = _rect_bbox(rect)
        c0, r0 = grid.cell_of(x0, y0)
        c1, r1 = grid.cell_of(x1, y1)
        win = (max(0, c0 - 1), max(0, r0 - 1), min(grid.width - 1, c1 + 1), min(grid.height - 1, r1 + 1))
        if win[0] > win[2] or win[1] > win[3]:
            return None
    return _window_area(grid, win, required, rect)


# ------------------------------------------------------------------ 目的地のスナップ
def snap_to_drivable(
    grid: Optional[Grid], x: float, y: float,
    margin_m: float = WALL_MARGIN_M,
    half_width_m: float = ROBOT_HALF_WIDTH_M,
    rect: Optional[Dict[str, float]] = None,
    max_shift_m: float = SNAP_MAX_SHIFT_M,
) -> Dict[str, Any]:
    """目的地(x, y)が可走(壁から機体端でmargin_m以上、rectがあればその内側)なら"ok"、
    そうでなければ max_shift_m 以内の最寄りの可走セル中心へ寄せる("snapped")。

    戻り値: {"status", "x", "y", "shift_m"}
      status: "ok" / "snapped" / "no_drivable"(max_shift_m以内に可走セルが無い。x,yは元のまま)
              / "no_map"(地図が無い/不正/margin_m<=0。x,yは元のまま=従来どおり)
    """
    out = {"status": "no_map", "x": x, "y": y, "shift_m": 0.0}
    try:
        if grid is None or not grid.valid() or margin_m is None or margin_m <= 0.0:
            return out
        if not (math.isfinite(x) and math.isfinite(y)):
            return out
        res = grid.resolution
        c, r = grid.cell_of(x, y)
        reach = int(math.ceil(max_shift_m / res)) + 1
        win = (max(0, c - reach), max(0, r - reach), min(grid.width - 1, c + reach), min(grid.height - 1, r + reach))
        if win[0] > win[2] or win[1] > win[3]:
            return out
        area = _window_area(grid, win, max(0.0, half_width_m) + margin_m, rect)
    except Exception:  # noqa: BLE001 - 判定の失敗は従来どおり(寄せない)
        return out
    if area.point_ok(x, y):
        out["status"] = "ok"
        return out
    best, best_d2 = None, None
    lim2 = max_shift_m * max_shift_m
    for rr in range(area.h):
        row = area.mask[rr]
        for cc in range(area.w):
            if not row[cc]:
                continue
            px, py = grid.center_of(area.c0 + cc, area.r0 + rr)
            d2 = (px - x) ** 2 + (py - y) ** 2
            if d2 <= lim2 and (best_d2 is None or d2 < best_d2):
                best, best_d2 = (px, py), d2
    if best is None:
        out["status"] = "no_drivable"
        return out
    out.update(status="snapped", x=best[0], y=best[1], shift_m=math.sqrt(best_d2))
    return out


# ------------------------------------------------------------------ 巡回経路
@dataclass
class Segment:
    lane: int
    start: Tuple[float, float]
    end: Tuple[float, float]

    def length(self) -> float:
        return math.hypot(self.end[0] - self.start[0], self.end[1] - self.start[1])


def plan_patrol_segments(
    area: Optional[DriveArea],
    lane_spacing_m: float = LANE_SPACING_M,
) -> List[Segment]:
    """可走領域全体を往復して覆うレーンの区間列(行き来する順)。

    レーンは矩形の長辺方向(rectが無ければx軸方向)。レーンの位置は可走セルの広がりの両端を含めて
    等間隔(間隔 <= lane_spacing_m)。各レーンは連続した可走セルの区間に分割し、隣のレーンと
    逆向きに辿る。レーンの隙間に残った可走セルは、足りない所だけ補助レーンを足して覆う。
    """
    if area is None or lane_spacing_m <= 0 or area.count() == 0:
        return []
    g = area.grid
    res = g.resolution
    yaw = 0.0
    if area.rect is not None:
        yaw = area.rect.get("yaw", 0.0)
        if area.rect["height"] > area.rect["width"]:
            yaw += math.pi / 2.0   # 長辺方向をレーン方向(u)にする
    cu, su = math.cos(yaw), math.sin(yaw)
    ox = area.rect["center_x"] if area.rect is not None else g.origin_x
    oy = area.rect["center_y"] if area.rect is not None else g.origin_y

    def to_uv(x: float, y: float) -> Tuple[float, float]:
        dx, dy = x - ox, y - oy
        return dx * cu + dy * su, -dx * su + dy * cu

    def to_xy(u: float, v: float) -> Tuple[float, float]:
        return ox + u * cu - v * su, oy + u * su + v * cu

    cells = area.cells()
    uv = {cell: to_uv(*g.center_of(*cell)) for cell in cells}
    vmin = min(p[1] for p in uv.values())
    vmax = max(p[1] for p in uv.values())
    span = vmax - vmin
    n = max(1, int(math.ceil(span / lane_spacing_m - 1e-9)) + 1) if span > 1e-9 else 1
    lane_vs = [vmin + (span * k / (n - 1) if n > 1 else 0.0) for k in range(n)]

    step = res * 0.5
    us_all = [p[0] for p in uv.values()]
    umin, umax = min(us_all) - res, max(us_all) + res
    covered = set()
    cover_r = lane_spacing_m / 2.0 + res
    cr_cells = int(math.ceil(cover_r / res))
    disk = [(dc, dr) for dr in range(-cr_cells, cr_cells + 1) for dc in range(-cr_cells, cr_cells + 1)
            if (dc * dc + dr * dr) * res * res <= cover_r * cover_r]

    def lane_runs(v: float) -> List[Tuple[float, float]]:
        """レーン(v一定)上の連続した可走区間 [(u_start, u_end)]。"""
        runs: List[Tuple[float, float]] = []
        cur: Optional[List[float]] = None
        u = umin
        while u <= umax + 1e-9:
            x, y = to_xy(u, v)
            ok = area.point_ok(x, y)
            if ok:
                if cur is None:
                    cur = [u, u]
                else:
                    cur[1] = u
            elif cur is not None:
                runs.append((cur[0], cur[1]))
                cur = None
            u += step
        if cur is not None:
            runs.append((cur[0], cur[1]))
        return runs

    def stamp(run: Tuple[float, float], v: float) -> None:
        u = run[0]
        while True:
            c, r = g.cell_of(*to_xy(u, v))
            for dc, dr in disk:
                covered.add((c + dc, r + dr))
            if u >= run[1]:
                break
            u = min(run[1], u + cover_r * 0.5)

    lanes: List[Tuple[float, List[Tuple[float, float]]]] = []
    for v in lane_vs:
        runs = lane_runs(v)
        for run in runs:
            stamp(run, v)
        lanes.append((v, runs))
    # 隙間に残った可走セルを覆う補助レーン
    for _ in range(500):
        left = [c for c in cells if c not in covered]
        if not left:
            break
        c = min(left, key=lambda cell: (uv[cell][1], uv[cell][0]))
        v = uv[c][1]
        runs = [rn for rn in lane_runs(v) if rn[0] - res <= uv[c][0] <= rn[1] + res]
        if not runs:
            runs = [(uv[c][0], uv[c][0])]
        for run in runs:
            stamp(run, v)
        lanes.append((v, runs))

    lanes.sort(key=lambda t: t[0])
    segments: List[Segment] = []
    forward = True
    for idx, (v, runs) in enumerate(lanes):
        if not runs:
            continue
        ordered = sorted(runs)
        if not forward:
            ordered = [(b, a) for a, b in reversed(ordered)]
        for a, b in ordered:
            segments.append(Segment(idx, to_xy(a, v), to_xy(b, v)))
        forward = not forward
    return segments


def plan_patrol_path(
    grid: Optional[Grid],
    rect: Optional[Dict[str, float]],
    margin_m: float = WALL_MARGIN_M,
    half_width_m: float = ROBOT_HALF_WIDTH_M,
    lane_spacing_m: float = LANE_SPACING_M,
) -> List[Tuple[float, float]]:
    """巡回の点列 [(x, y), ...](各区間の始点・終点を順に。長さ0の区間は1点)。
    占有格子が無い/不正、margin_m<=0 のときは空(呼び出し側は従来どおりにする)。"""
    if grid is None or margin_m is None or margin_m <= 0.0:
        return []
    area = compute_drivable_area(grid, rect, margin_m, half_width_m)
    pts: List[Tuple[float, float]] = []
    for seg in plan_patrol_segments(area, lane_spacing_m):
        pts.append(seg.start)
        if math.hypot(seg.end[0] - seg.start[0], seg.end[1] - seg.start[1]) > 1e-9:
            pts.append(seg.end)
    return pts
