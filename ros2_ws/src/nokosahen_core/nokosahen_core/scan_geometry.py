"""person_target_detector から切り出した、LiDARまわりの純粋関数(numpyのみ。rclpy不要でテストできる)。

- lidar_confirms_obstacle: カメラの障害物検知よりLiDARの距離を優先する判定(2026-11-04)。
- wide_point_mask: 向く用の人候補を探す点の選び方(前方より広い角度、既定は全周)。
- cluster_leg_points: 点列を脚幅のクラスタ(bearing, x, y)にまとめる。
detector本体の挙動は変えない(同じ式をここへ移しただけ)。
"""
from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np


def lidar_confirms_obstacle(
    angles,
    ranges,
    range_min: float,
    range_max: float,
    bearing: float,
    confirm_m: float,
    half_width_rad: float,
    age_s: Optional[float],
    stale_s: float,
) -> bool:
    """カメラが障害物と見た方位 bearing(機体基準、左が正)の近くで、LiDARが confirm_m 以内に何かを測っているか。
    近くに何も測っていなければ False(カメラの障害物は十分遠い/誤認識として無視する)。
    機能が無効(confirm_m<=0)・LiDARの結果が無い/古い(age_s が None か stale_s 超え)・その方向が測れていない
    ときは True(従来どおりカメラの障害物を採用=安全側)。"""
    if confirm_m <= 0.0:
        return True
    if angles is None or ranges is None or age_s is None or age_s > stale_s:
        return True
    angles = np.asarray(angles, dtype=np.float64)
    ranges = np.asarray(ranges, dtype=np.float64)
    diff = np.abs((angles - bearing + math.pi) % (2.0 * math.pi) - math.pi)
    sel = (diff <= half_width_rad) & np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    if not np.any(sel):
        return True
    return bool(np.min(ranges[sel]) <= confirm_m)


def wide_point_mask(
    in_range,
    angles,
    ranges,
    range_min: float,
    max_range: float,
    front_half_rad: float,
    candidate_angle_rad: float,
    background_excluded=None,
):
    """前方(±front_half_rad)の点 in_range に、前方の外で candidate_angle_rad の範囲に入る点を足したマスク。
    background_excluded(基準スキャンより明確に近くない点=壁・柱)があれば、足した点からも除く。"""
    angles = np.asarray(angles, dtype=np.float64)
    ranges = np.asarray(ranges, dtype=np.float64)
    wide = np.asarray(in_range, dtype=bool) | (
        (np.abs(angles) <= candidate_angle_rad / 2.0 + 1e-9)
        & np.isfinite(ranges)
        & (ranges >= range_min)
        & (ranges <= max_range)
        & ~((angles >= -front_half_rad) & (angles <= front_half_rad))
    )
    if background_excluded is not None:
        wide &= ~np.asarray(background_excluded, dtype=bool)
    return wide


def nearest_range(ranges, range_min: float, max_range: float) -> float:
    """【2026-11-04】全周の、range_min 以上 max_range 以下の有限な点の最短距離。無ければ 99.0。
    基準スキャンは使わない(機体が動いた先の近い物を見逃さないため)。"""
    r = np.asarray(ranges, dtype=np.float64)
    ok = np.isfinite(r) & (r >= range_min) & (r <= max_range)
    return float(np.min(r[ok])) if np.any(ok) else 99.0


def _group_width(group: Sequence[Tuple[float, float]]) -> float:
    if len(group) < 2:
        return 0.0
    a1, r1 = group[0]
    a2, r2 = group[-1]
    return math.hypot(r1 * math.cos(a1) - r2 * math.cos(a2), r1 * math.sin(a1) - r2 * math.sin(a2))


def _split_groups(
    points: Sequence[Tuple[float, float]],
    cluster_gap_m: float,
    angle_gap_max: float,
    wrap_full: bool = False,
) -> List[List[Tuple[float, float]]]:
    """点列を、距離差・角度差で区切ったグループにする。
    【2026-11-05】wrap_full が真(全周)のとき、最初と最後のグループが±π(真後ろ)の継ぎ目をまたいで続いていれば1つにつなぐ
    (継ぎ目で脚が2つに割れて、どちらも幅が足りず消えるのを防ぐ)。"""
    if not points:
        return []
    groups: List[List[Tuple[float, float]]] = []
    current: List[Tuple[float, float]] = [points[0]]
    for prev, curr in zip(points, points[1:]):
        if abs(curr[1] - prev[1]) > cluster_gap_m or abs(curr[0] - prev[0]) > angle_gap_max:
            groups.append(current)
            current = []
        current.append(curr)
    groups.append(current)
    if wrap_full and len(groups) > 1:
        last, first = groups[-1][-1], groups[0][0]
        seam_gap = (first[0] + 2.0 * math.pi) - last[0]  # 最後の点から最初の点まで、継ぎ目をまたぐ角度差
        if seam_gap <= angle_gap_max and abs(first[1] - last[1]) <= cluster_gap_m:
            groups[0] = groups[-1] + groups[0]
            groups.pop()
    return groups


def _group_to_cluster(
    group: Sequence[Tuple[float, float]], leg_min_width_m: float, leg_max_width_m: float
) -> Optional[Tuple[float, float, float]]:
    if not group or not (leg_min_width_m <= _group_width(group) <= leg_max_width_m):
        return None
    mid_angle, mid_range = group[len(group) // 2]
    return (mid_angle, mid_range * math.cos(mid_angle), mid_range * math.sin(mid_angle))


def cluster_leg_points(
    points: Sequence[Tuple[float, float]],
    cluster_gap_m: float,
    angle_gap_max: float,
    leg_min_width_m: float,
    leg_max_width_m: float,
    wrap_full: bool = False,
) -> List[Tuple[float, float, float]]:
    """(angle, range) の点列を、距離差・角度差で区切ってクラスタにし、脚幅のものを (bearing, x, y) で返す。
    wrap_full: 全周の点列のとき真(±πの継ぎ目をまたぐ脚をつなぐ)。"""
    clusters: List[Tuple[float, float, float]] = []
    for group in _split_groups(points, cluster_gap_m, angle_gap_max, wrap_full):
        c = _group_to_cluster(group, leg_min_width_m, leg_max_width_m)
        if c is not None:
            clusters.append(c)
    return clusters


def cluster_front_and_wide(
    points: Sequence[Tuple[float, float]],
    cluster_gap_m: float,
    angle_gap_max: float,
    leg_min_width_m: float,
    leg_max_width_m: float,
    front_half_rad: float,
    wrap_full: bool = False,
) -> Tuple[List[Tuple[float, float, float]], List[Tuple[float, float, float]]]:
    """【2026-11-05】全周(広い範囲)の点列を1回だけグループ分けし、前方(|角度|<=front_half_rad)のクラスタも同じ結果から出す。
    戻り値は (前方のクラスタ, 全周のクラスタ)。前方は、全周のグループを前方の範囲で切り取って幅を判定する
    (前方の点だけを別にクラスタリングした結果と同じ。前方の点は並びの中で連続しているため)。前方側は継ぎ目をつながない。"""
    groups = _split_groups(points, cluster_gap_m, angle_gap_max, wrap_full)
    front: List[Tuple[float, float, float]] = []
    wide: List[Tuple[float, float, float]] = []
    for group in groups:
        c = _group_to_cluster(group, leg_min_width_m, leg_max_width_m)
        if c is not None:
            wide.append(c)
        trimmed = [p for p in group if abs(p[0]) <= front_half_rad]
        if len(trimmed) != len(group):
            c = _group_to_cluster(trimmed, leg_min_width_m, leg_max_width_m)
        if c is not None:
            front.append(c)
    return front, wide


def scan_is_stale(
    last_scan_mono: Optional[float],
    now_mono: float,
    started_mono: float,
    stale_s: float,
    require_scan: bool = True,
    startup_grace_s: float = 3.0,
) -> bool:
    """【2026-11-05】/scan が途絶えているか。受信済みなら stale_s 超えで真。
    一度も受信していないときは、require_scan が真で、起動から startup_grace_s を超えたら真(=途絶と同じ扱い=安全側)。
    require_scan が偽(LiDARを使わない構成)なら、未受信は従来どおり偽。"""
    if last_scan_mono is not None:
        return (now_mono - last_scan_mono) > stale_s
    return bool(require_scan) and (now_mono - started_mono) > startup_grace_s


def validate_background(data) -> Tuple[Optional[dict], str]:
    """【2026-11-05】背景スキャン(~/.nokosahen_crowd_background.json)の中身を検証する。(使えるdata, 理由)。使えなければ (None, 理由)。
    rangesの要素に null・文字列があると、毎スキャン例外になるため、読み込み時に弾く(inf/nanは有効な値)。"""
    if not isinstance(data, dict) or "ranges" not in data:
        return None, "形式が違います(rangesがありません)"
    ranges = data.get("ranges")
    if not isinstance(ranges, list) or not ranges:
        return None, "rangesが空か、リストではありません"
    for r in ranges:
        if isinstance(r, bool) or not isinstance(r, (int, float)):
            return None, "rangesに数値でない値(null等)があります"
    for key in ("angle_min", "angle_increment"):
        if key in data:
            v = data[key]
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                return None, f"{key}が有限の数値ではありません"
    return data, ""


class RawObstacleMemory:
    """【2026-11-05】カメラの障害物判定の「LiDAR優先で抑制する前」の方位を覚えておく。
    その場の旋回が終わった瞬間に、抑制なしの判定を出し直すために使う(旋回中は抑制した結果(false)を出していたため、
    前進に移った直後は本物の障害物が次のカメラ判定まで伝わらなかった)。"""

    def __init__(self) -> None:
        self._bearings: List[float] = []
        self._stamp: Optional[float] = None

    def update(self, bearings: Sequence[float], now: float) -> None:
        self._bearings = list(bearings)
        self._stamp = now

    def fresh(self, now: float, max_age_s: float) -> Optional[List[float]]:
        """max_age_s 以内の生の方位(空リスト=障害物なし)。一度も判定していない/古いときは None(出し直さない)。"""
        if self._stamp is None or now - self._stamp > max_age_s:
            return None
        return list(self._bearings)


def in_place_turn_ended(was_turning: bool, now_turning: bool) -> bool:
    """旋回中(true)から旋回なし(false)に変わった瞬間だけ真。"""
    return bool(was_turning) and not bool(now_turning)


def lidar_priority_active(in_place_turn_age_s: Optional[float], only_in_place: bool, fresh_sec: float = 1.0) -> bool:
    """カメラの障害物判定で、LiDARの距離を優先してよいか。
    only_in_place が真のときは、directorが「向く(その場の旋回)」を出している間(通知 in_place_turn_age_s が fresh_sec 以内)だけ優先する。
    それ以外はカメラだけで障害物と判断する(机の天板・ガラス・段差などLiDARに映らない物を止めるため)。
    only_in_place が偽なら、常にLiDAR優先。"""
    if not only_in_place:
        return True
    return in_place_turn_age_s is not None and 0.0 <= in_place_turn_age_s <= fresh_sec
