"""人の多い方位を選ぶ純粋ロジック(ROS非依存)。

ロボット正面を0、左が正のラジアン。設計書 docs/face_person_design.md 3章/4章/7.7章。
"""
import math
from typing import Any, Dict, Iterable, List, Optional, Tuple

SECTOR_DEG = 30
LIDAR_ONLY_WEIGHT = 0.75   # 【2026-11-04】LiDARの足を優先(カメラの方位は魚眼で粗いので、カメラだけの候補は軽く扱う)
CAMERA_ONLY_WEIGHT = 0.25
REJECTED_WEIGHT = 0.1      # 【2026-11-04】前方カメラの視野内でカメラに人と認められなかったLiDAR候補(机・椅子の脚など)の重み
LEG_PAIR_MIN_GAP_M = 0.05  # 【2026-11-04】脚ペア(2クラスタで1人)とみなす間隔。person_target_detector の leg_pair_min/max_gap_m と同じ考え方
LEG_PAIR_MAX_GAP_M = 0.6
LEG_PAIR_FACTOR = 0.6      # 脚ペアの各脚の重みの倍率(2本で 0.75*0.6*2=0.9、単独脚1本0.75よりやや重い=約1人分)
MONITOR_HALF_DEG = 90      # 【2026-11-04】向く動作の監視範囲: スイッチをONにしたときの車体の向きを基準に、前の180度(左右90度)
MONITOR_STICKY_DEG = 20    # 【2026-11-05】今追っている人は、範囲の縁からこの角度まで外へ出ても対象に残す(縁で出入りして目標がちらつくのを防ぐ)
STICKY_NEAR_DEG = 30       # 「今追っている人」とみなす範囲: 直前に向いた世界の方位からこの角度以内
BODY_RANGE_M = 0.36        # 【2026-11-05】LiDARの中心からこの距離より近いLiDAR候補は機体の枠(四隅の支柱は0.37 m)で、人ではない
SWEEP_MARGIN_DEG = 5       # 見回しの折り返しは、範囲の端のこの角度手前
FUSE_TOL_DEG = 8
# 【2026-11-05】実機で、LiDARだけの足(重み合計0.9)が1秒見えるたびに、追っていたカメラの人(0.25)から切り替わって往復した。
# 今の人の重みを1.6倍(1.2から)にして、挑戦者が2.5秒(1.0秒から)続いたときだけ切り替える。今の人が消えたときの付き替え(CONTINUITY_DEG)は即時のまま。
HYSTERESIS_GAIN = 1.6
SWITCH_HOLD_SEC = 2.5
CONTINUITY_DEG = 45   # 今の扇形が空になったとき、直前の方位からこの角度以内の候補は「同じ人が動いた」とみなして、すぐ付いていく
ALIGN_DEG = 5
REENGAGE_DEG = 10

# 【2026-11-06】向く先を「多数派の扇形」から「一番近い人」に変えた(FaceBearingSelector)。理由と各値の根拠は docs/face_person_design.md 7.8章。
SAME_DIST_ABS_M = 0.25      # 同じ距離とみなす差の下限(近い人の距離の10%がこれより小さいときはこの値)
SAME_DIST_RATIO = 0.10      # 同じ距離とみなす差: 近い人の距離の10%。遠くほど測距のゆらぎが大きいので比例させる
SAME_DIST_STAY_GAIN = 1.5   # 「同じ距離の組」に入った人は、差がこの倍まで開くまで抜けない(境目の出入りで向く先が飛ぶのを防ぐ)
PAIR_MAX_SPAN_DEG = 90      # 同じ距離の人たちの方位の開きがこれ以内なら「間」を向く。超えたら間を向かず、追っている人を向く
PAIR_STAY_GAIN = 1.15
SWITCH_MARGIN_M = 0.3       # 追っている人より、この距離以上近い人が現れたら、切替の候補にする
SWITCH_RATIO = 1.3          # または追っている人の距離が、その人の距離の1.3倍以上のとき
NEAREST_HOLD_SEC = 2.5      # 上の条件が続いたときだけ追う人を替える(足の見え隠れ・ゆらぎ・通りすがりで振り回されない)
TRACK_MATCH_DEG = 12        # 前の測定と同じ人とみなす方位差(世界座標)
TRACK_MATCH_RANGE_M = 0.8   # 〃 距離差
TRACK_KEEP_SEC = 0.8        # 見えなくなった人を覚えておく時間(足が一瞬途切れても別人扱いにしない)
RANGE_WINDOW_SEC = 1.0      # 距離の平滑化: この時間内の最大 RANGE_WINDOW_N 点の中央値
RANGE_WINDOW_N = 5
CHALLENGER_FRESH_SEC = 0.35  # 切替の候補は、今(この間以内)見えている人だけ。見えなくなった人の記憶(TRACK_KEEP_SEC)で切替の時間が進まないように
SWITCH_KEEP_FACTOR = 0.7     # 切替の候補になった人は、条件(差・倍率)がこの割合まで緩んでも候補のままにする(境目の雑音でタイマーが戻らないように)
SETTLE_SEC = 1.0            # 最初に追う人を決めてからこの間は、近い人が見つかり次第すぐ替えてよい(最初の1回の取りこぼしで遠い人を選んでしまっても、2.5秒待たない)

_EPS = 1e-9
_MIN_SPEED_RATIO = 0.25
_FULL_SPEED_ERR = math.pi / 3
_N_SECTORS = int(round(360 / SECTOR_DEG))


def _num(v) -> Optional[float]:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    v = float(v)
    return v if math.isfinite(v) else None


def _wrap(a: float) -> float:
    """-π〜π に正規化。"""
    a = math.fmod(a + math.pi, 2 * math.pi)
    if a < 0:
        a += 2 * math.pi
    return a - math.pi


def _items(seq) -> Iterable[Any]:
    return seq if isinstance(seq, (list, tuple)) else []


def fuse_candidates(lidar, camera) -> List[Dict[str, Any]]:
    """LiDAR候補とカメラの人を融合する。戻り値は bearing/range/weight/source のリスト。"""
    lid = []
    for c in _items(lidar):
        if not isinstance(c, dict):
            continue
        b = _num(c.get("bearing"))
        if b is None:
            continue
        r = _num(c.get("range"))
        lid.append({"bearing": _wrap(b), "range": r,
                    "confirmed": bool(c.get("confirmed", False)),
                    "rejected": bool(c.get("rejected", False))})
    cam = []
    for c in _items(camera):
        if not isinstance(c, dict):
            continue
        b = _num(c.get("bearing"))
        if b is not None:
            cam.append(_wrap(b))

    tol = math.radians(FUSE_TOL_DEG)
    pairs = []
    for ci, cbr in enumerate(cam):
        for li, l in enumerate(lid):
            d = abs(_wrap(cbr - l["bearing"]))
            if d <= tol + _EPS:
                pairs.append((d, ci, li))
    pairs.sort()
    cam_used, lid_used = set(), set()
    for _d, ci, li in pairs:
        if ci in cam_used or li in lid_used:
            continue
        cam_used.add(ci)
        lid_used.add(li)

    # 【2026-11-04】LiDARだけの候補(カメラと一致せず、確認済みでもない)のうち、カメラに否定されていないものは、
    # 脚ペア(間隔 LEG_PAIR_MIN_GAP_M〜MAX)の2クラスタを約1人として数える(2本とも重みを LEG_PAIR_FACTOR 倍=合計で約1人分)。1本ずつ0.75で数えると、
    # 4本脚の机が人(脚2本)より重くなるため。
    lonely = [li for li, l in enumerate(lid)
              if li not in lid_used and not l["confirmed"] and not l["rejected"] and l["range"] is not None]
    # 脚幅の範囲に隣のクラスタがある脚は、全部「脚ペアの脚」として軽くする(4本脚の机も、全部の脚が軽くなる)。
    paired = set()
    for i, a in enumerate(lonely):
        for b2 in lonely[i + 1:]:
            la, lb = lid[a], lid[b2]
            ax, ay = la["range"] * math.cos(la["bearing"]), la["range"] * math.sin(la["bearing"])
            bx, by = lb["range"] * math.cos(lb["bearing"]), lb["range"] * math.sin(lb["bearing"])
            if LEG_PAIR_MIN_GAP_M <= math.hypot(ax - bx, ay - by) <= LEG_PAIR_MAX_GAP_M + 1e-6:
                paired.add(a)
                paired.add(b2)

    out: List[Dict[str, Any]] = []
    for li, l in enumerate(lid):
        if li in lid_used or l["confirmed"]:
            out.append({"bearing": l["bearing"], "range": l["range"],
                        "weight": 1.0, "source": "both"})
        elif l["rejected"]:
            out.append({"bearing": l["bearing"], "range": l["range"],
                        "weight": REJECTED_WEIGHT, "source": "lidar"})
        else:
            w = LIDAR_ONLY_WEIGHT * LEG_PAIR_FACTOR if li in paired else LIDAR_ONLY_WEIGHT
            out.append({"bearing": l["bearing"], "range": l["range"],
                        "weight": w, "source": "lidar"})
    for ci, cbr in enumerate(cam):
        if ci not in cam_used:
            out.append({"bearing": cbr, "range": None,
                        "weight": CAMERA_ONLY_WEIGHT, "source": "camera"})
    return out


def _sector_of(bearing: float) -> int:
    """正面中心の扇形番号(0..N-1)。扇形iは 30*i±15度。"""
    idx = int(math.floor(math.degrees(bearing) / SECTOR_DEG + 0.5))
    return idx % _N_SECTORS


class WeightFaceBearingSelector:
    """扇形ごとの重みで向く方位を選ぶ。ちらつき防止つき。
    【2026-11-06】距離が一人も分からないとき(カメラだけの候補しかないとき)の予備として FaceBearingSelector が使う。"""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._current: Optional[int] = None
        self._challenger: Optional[int] = None
        self._challenger_since = 0.0
        self._last_bearing: Optional[float] = None
        self._last_yaw: Optional[float] = None

    @staticmethod
    def _mean_bearing(items: List[Tuple[float, float]]) -> float:
        sx = sum(w * math.cos(b) for b, w in items)
        sy = sum(w * math.sin(b) for b, w in items)
        if abs(sx) < _EPS and abs(sy) < _EPS:
            return items[0][0]
        return math.atan2(sy, sx)

    def _follow_nearby(self, sectors) -> bool:
        """今の扇形が空のとき、直前の方位の近くにある候補の扇形へ、すぐ移る。移れたらTrue。"""
        if self._last_bearing is None:
            return False
        best_k, best_d = None, None
        for k, sec in sectors.items():
            for b, _w in sec["items"]:
                d = abs(_wrap(b - self._last_bearing))
                if best_d is None or d < best_d:
                    best_k, best_d = k, d
        if best_k is not None and best_d <= math.radians(CONTINUITY_DEG):
            self._current = best_k
            return True
        return False

    def update(self, now_sec: float, candidates, yaw: Optional[float] = None) -> Optional[float]:
        # 【2026-11-05】yaw(今の機体の向き・世界)が分かるときは、直前の方位を「世界の向き」で保持する。切替待ちの間に旋回しても、
        # 返す機体基準の方位が古いまま(旋回で行き過ぎる)にならない。yaw が無ければ従来どおり。
        y = _num(yaw)
        if y is not None and self._last_yaw is not None and self._last_bearing is not None:
            self._last_bearing = _wrap(self._last_bearing + self._last_yaw - y)
        self._last_yaw = y
        sectors: Dict[int, Dict[str, Any]] = {}
        for c in _items(candidates):
            if not isinstance(c, dict):
                continue
            b = _num(c.get("bearing"))
            w = _num(c.get("weight"))
            if b is None or w is None or w <= 0:
                continue
            b = _wrap(b)
            r = _num(c.get("range"))
            s = sectors.setdefault(_sector_of(b), {"w": 0.0, "items": [], "rmin": None})
            s["w"] += w
            s["items"].append((b, w))
            if r is not None and (s["rmin"] is None or r < s["rmin"]):
                s["rmin"] = r
        if not sectors:
            self.reset()
            return None

        def eff(k: int) -> float:
            w = sectors[k]["w"]
            return w * HYSTERESIS_GAIN if k == self._current else w

        top = max(eff(k) for k in sectors)
        tied = [k for k in sectors if eff(k) >= top - _EPS]
        if len(tied) == 1:
            best = tied[0]
        else:
            with_r = [k for k in tied if sectors[k]["rmin"] is not None]
            if self._current in tied and not with_r:
                best = self._current
            elif with_r:
                best = min(with_r, key=lambda k: sectors[k]["rmin"])
            else:
                best = tied[0]

        if self._current is None:
            self._current = best
            self._challenger = None
        elif self._current not in sectors and self._follow_nearby(sectors):
            # 現在の扇形に誰もいないが、直前の方位の近く(CONTINUITY_DEG以内)に候補がある: ロボットが旋回して、人が機体基準で
            # 隣の扇形へ動いただけ。待たず、すぐその人に付いていく(待つと古い方位を返し続けて旋回が行き過ぎる)。
            # 遠くの候補だけが残ったときは、見逃しの可能性があるので、従来どおり SWITCH_HOLD_SEC 待つ。
            self._challenger = None
        elif best == self._current:
            self._challenger = None
        else:
            if self._challenger != best:
                self._challenger = best
                self._challenger_since = now_sec
            if now_sec - self._challenger_since >= SWITCH_HOLD_SEC - _EPS:
                self._current = best
                self._challenger = None

        cur = sectors.get(self._current)
        if cur is not None:
            self._last_bearing = self._mean_bearing(cur["items"])
        elif self._last_bearing is None:
            # 現在の扇形が一時的に空(切替待ち)なら直前の方位を保つ
            self._last_bearing = self._mean_bearing(sectors[best]["items"])
        return self._last_bearing


class _Track:
    __slots__ = ("id", "world", "samples", "last_t")

    def __init__(self, tid: int, world: float, t: float, r: float) -> None:
        self.id, self.world, self.last_t = tid, world, t
        self.samples: List[Tuple[float, float]] = [(t, r)]

    def add(self, t: float, r: float) -> None:
        self.samples = [(a, b) for a, b in self.samples if t - a <= RANGE_WINDOW_SEC][-(RANGE_WINDOW_N - 1):]
        self.samples.append((t, r))
        self.last_t = t

    @property
    def last_range(self) -> float:
        return self.samples[-1][1]

    @property
    def range(self) -> float:
        v = sorted(r for _t, r in self.samples)
        n = len(v)
        return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])


class FaceBearingSelector:
    """向く方位を「一番近い人」で選ぶ。【2026-11-06】
    - 距離はLiDAR(range)。人ごとに世界座標の方位で追跡し、距離は短い窓の中央値で平滑化する。
    - 追っている人(anchor)は、十分近い別の人が NEAREST_HOLD_SEC 続いたときだけ替える。消えたら近く(CONTINUITY_DEG)の人にすぐ付く。
    - anchor と同じ距離(SAME_DIST_*)の人がいれば、その人たちの「間」を向く(方位の開きが PAIR_MAX_SPAN_DEG 以内のとき)。
    - 距離が分かる人が一人もいない(カメラだけ)ときは、従来の重みの多数決(WeightFaceBearingSelector)。
    update の入出力は従来と同じ(方位は機体基準・左が正)。"""

    def __init__(self) -> None:
        self.anchor_switches = 0
        self.reset()

    def reset(self) -> None:
        self._tracks: List[_Track] = []
        self._next_id = 1
        self._anchor: Optional[int] = None
        self._anchor_world: Optional[float] = None
        self._challenger: Optional[int] = None
        self._challenger_since = 0.0
        self._partners: set = set()
        self._lost_since: Optional[float] = None
        self._anchor_seen_t: Optional[float] = None
        self._settle_until = -1.0
        self._yaw_known: Optional[bool] = None
        self._legacy = WeightFaceBearingSelector()

    @property
    def anchor_id(self) -> Optional[int]:
        return self._anchor

    def _clear_tracks(self) -> None:
        self._tracks, self._anchor, self._challenger, self._partners = [], None, None, set()
        self._anchor_world, self._lost_since, self._anchor_seen_t = None, None, None
        self._legacy = WeightFaceBearingSelector()

    def _set_anchor(self, tid: Optional[int]) -> None:
        if tid != self._anchor and self._anchor is not None and tid is not None:
            self.anchor_switches += 1
        self._anchor = tid
        self._challenger = None
        self._lost_since = None
        if tid is None:
            self._partners = set()

    @staticmethod
    def _same_dist_tol(r_anchor: float) -> float:
        return max(SAME_DIST_ABS_M, SAME_DIST_RATIO * r_anchor)

    def update(self, now_sec: float, candidates, yaw: Optional[float] = None) -> Optional[float]:
        y = _num(yaw)
        yk = y is not None
        yy = y if yk else 0.0
        now = _num(now_sec)
        now = 0.0 if now is None else now
        if self._yaw_known is not None and yk != self._yaw_known:
            self._clear_tracks()
        self._yaw_known = yk

        parsed = []
        for c in _items(candidates):
            if not isinstance(c, dict):
                continue
            b, w = _num(c.get("bearing")), _num(c.get("weight"))
            if b is None or w is None or w <= 0:
                continue
            r = _num(c.get("range"))
            parsed.append((_wrap(b), w, r if (r is not None and r > 0) else None))
        if not parsed:
            self.reset()
            return None
        strong = [p for p in parsed if p[1] > REJECTED_WEIGHT + _EPS]
        use = strong or parsed       # 机の脚など(カメラに人と否定された)は、他に候補が無いときだけ使う
        ranged = [(_wrap(b + yy), r) for b, _w, r in use if r is not None]

        self._tracks = [t for t in self._tracks if not (now - t.last_t > TRACK_KEEP_SEC)]
        pairs = []
        for ti, t in enumerate(self._tracks):
            for ci, (cw, cr) in enumerate(ranged):
                d = abs(_wrap(cw - t.world))
                if d <= math.radians(TRACK_MATCH_DEG) + _EPS and abs(cr - t.last_range) <= TRACK_MATCH_RANGE_M:
                    pairs.append((d, ti, ci))
        pairs.sort()
        used_t, used_c = set(), set()
        for _d, ti, ci in pairs:
            if ti in used_t or ci in used_c:
                continue
            used_t.add(ti)
            used_c.add(ci)
            self._tracks[ti].world = ranged[ci][0]
            self._tracks[ti].add(now, ranged[ci][1])
        for ci, (cw, cr) in enumerate(ranged):
            if ci not in used_c:
                self._tracks.append(_Track(self._next_id, cw, now, cr))
                self._next_id += 1
        # (rangeなしのカメラだけの候補は追跡しない。同じ方位にLiDARの人が居れば同一人物としてその距離を引き継ぎ=別の人として数えず、
        #  居なければ距離不明で、距離が分かる人が一人でもいる間は無視する)

        if not self._tracks:
            # 距離が分かる人が一人もいない: 従来の重みの多数決
            b = self._legacy.update(now, candidates, yaw=y)
            if b is not None:
                self._anchor_world = _wrap(b + yy)
            return b
        self._legacy.reset()

        tracks = {t.id: t for t in self._tracks}
        nearest = min(self._tracks, key=lambda t: (round(t.range, 3), -_wrap(t.world - yy)))   # 同距離なら左(正)を先に

        if self._anchor not in tracks:
            self._anchor = None
            self._partners = set()
            if self._anchor_world is None:
                self._set_anchor(nearest.id)
                self._settle_until = now + SETTLE_SEC
            else:
                near = min(self._tracks, key=lambda t: abs(_wrap(t.world - self._anchor_world)))
                if abs(_wrap(near.world - self._anchor_world)) <= math.radians(CONTINUITY_DEG) + _EPS:
                    self._set_anchor(near.id)      # 追っていた人が動いただけ(または消えた): 近くの人にすぐ付く
                else:
                    if self._lost_since is None:
                        self._lost_since = min(now, self._anchor_seen_t if self._anchor_seen_t is not None else now)
                    if now - self._lost_since >= NEAREST_HOLD_SEC - _EPS:
                        self._set_anchor(nearest.id)
                    else:
                        return _wrap(self._anchor_world - yy)   # 遠くの人だけが残った: 見逃しかもしれないので少し待つ
        anchor = tracks[self._anchor]
        self._anchor_seen_t = anchor.last_t

        # 追う人を替える条件: 十分近い人が NEAREST_HOLD_SEC 続く
        k = SWITCH_KEEP_FACTOR if self._challenger == nearest.id else 1.0
        if nearest.id != anchor.id and now - nearest.last_t <= CHALLENGER_FRESH_SEC + _EPS and (
                anchor.range - nearest.range >= SWITCH_MARGIN_M * k - _EPS
                or anchor.range >= (1.0 + (SWITCH_RATIO - 1.0) * k) * nearest.range - _EPS):
            if self._challenger != nearest.id:
                self._challenger, self._challenger_since = nearest.id, now
            if now - self._challenger_since >= NEAREST_HOLD_SEC - _EPS or now <= self._settle_until:
                settling = now <= self._settle_until
                self._set_anchor(nearest.id)
                if settling:
                    self.anchor_switches -= 1     # 最初の見極め中の付け替えは、切替の回数に数えない
                    self._settle_until = now + SETTLE_SEC
                anchor = nearest
                self._anchor_seen_t = anchor.last_t
        else:
            self._challenger = None

        # 同じ距離の人たち: anchor からの方位差の近い順に、開きが PAIR_MAX_SPAN_DEG 以内なら加える
        tol = self._same_dist_tol(anchor.range)
        lo = hi = 0.0
        members = set()
        others = []
        for t in self._tracks:
            if t.id == anchor.id:
                continue
            stay = t.id in self._partners
            if abs(t.range - anchor.range) <= tol * (SAME_DIST_STAY_GAIN if stay else 1.0) + _EPS:
                others.append((abs(_wrap(t.world - anchor.world)), stay, t))
        for _ad, stay, t in sorted(others, key=lambda o: o[0]):
            d = _wrap(t.world - anchor.world)
            lim = math.radians(PAIR_MAX_SPAN_DEG) * (PAIR_STAY_GAIN if stay else 1.0)
            if max(hi, d) - min(lo, d) <= lim + _EPS:
                lo, hi = min(lo, d), max(hi, d)
                members.add(t.id)
        self._partners = members
        target_world = _wrap(anchor.world + 0.5 * (lo + hi))
        self._anchor_world = anchor.world
        return _wrap(target_world - yy)


def face_turn_command(error_rad: float, max_rps: float, aligned: bool, min_rps: float = 0.0) -> Tuple[float, bool]:
    """誤差角(目標-現在、左が正)から (angular_z, aligned_next) を返す。
    【2026-11-05】min_rps: 旋回の指令の下限[rad/s]。実機(2026-11-05)で、0.6 rad/s前後以下の指令では車体が回らない
    (オドメトリの yaw が動かず wz=0)ことが分かった。誤差が小さいと指令が比例して小さくなり、回りきれずに旋回指令を出し続けていたため、
    整列の判定(ALIGN_DEG)の外では、この下限より小さい指令を出さない。0(既定)なら従来どおり(最大の 25% が下限)。"""
    e = _num(error_rad)
    m = _num(max_rps)
    if e is None:
        return 0.0, bool(aligned)
    e = _wrap(e)
    ae = abs(e)
    if aligned:
        if ae <= math.radians(REENGAGE_DEG) + _EPS:
            return 0.0, True
    elif ae <= math.radians(ALIGN_DEG) + _EPS:
        return 0.0, True
    if m is None or m <= 0:
        return 0.0, False
    mag = m * min(1.0, ae / _FULL_SPEED_ERR)
    floor = max(_MIN_SPEED_RATIO * m, _num(min_rps) or 0.0)
    mag = min(m, max(floor, mag))
    return math.copysign(mag, e), False


def in_front_half(bearing_rel: float, half_rad: float = math.pi / 2) -> bool:
    """機体基準の方位が正面 ±half_rad(既定90度)に入っているか。位置(向き)が取れないときの絞り込み用。"""
    b = _num(bearing_rel)
    return b is not None and abs(_wrap(b)) <= half_rad + _EPS


def in_monitor_range(bearing_rel: float, yaw: float, ref_yaw: float, half_rad: float = math.radians(MONITOR_HALF_DEG)) -> bool:
    """機体基準の方位 bearing_rel の人が、基準の向き ref_yaw の前 ±half_rad の監視範囲に入っているか。
    yaw は今の機体の向き(世界)。world = yaw + bearing_rel を ref_yaw との差で見る。"""
    b, y, r = _num(bearing_rel), _num(yaw), _num(ref_yaw)
    if b is None or y is None or r is None:
        return False
    return abs(_wrap(y + b - r)) <= half_rad + _EPS


def is_body_candidate(cand, body_range_m: float = BODY_RANGE_M) -> bool:
    """LiDAR候補が機体の枠(中心から body_range_m 未満)か。カメラだけの候補(rangeなし)は False。"""
    if not isinstance(cand, dict):
        return False
    r = _num(cand.get("range"))
    return r is not None and r < body_range_m


def in_monitor_range_sticky(bearing_rel: float, yaw: float, ref_yaw: float, last_world: Optional[float]) -> bool:
    """in_monitor_range に、縁のヒステリシスをつけたもの。範囲の中なら True。範囲の外でも、直前に追っていた人
    (last_world: 世界の方位[rad])の近く(STICKY_NEAR_DEG以内)で、範囲の縁から MONITOR_STICKY_DEG 以内なら True。"""
    if in_monitor_range(bearing_rel, yaw, ref_yaw):
        return True
    lw = _num(last_world)
    b, y = _num(bearing_rel), _num(yaw)
    if lw is None or b is None or y is None:
        return False
    if abs(_wrap(y + b - lw)) > math.radians(STICKY_NEAR_DEG) + _EPS:
        return False
    return in_monitor_range(bearing_rel, yaw, ref_yaw, half_rad=math.radians(MONITOR_HALF_DEG + MONITOR_STICKY_DEG))


def sweep_direction(yaw: float, ref_yaw: float, direction: float,
                    half_rad: float = math.radians(MONITOR_HALF_DEG),
                    margin_rad: float = math.radians(SWEEP_MARGIN_DEG)) -> float:
    """範囲(基準の向きの前 ±half_rad)の中を往復して見回すときの、次の回転方向(+1=左、-1=右)。
    端の margin_rad 手前で折り返す。範囲の外に出ていたら、範囲の中へ戻る向きにする。"""
    y, r = _num(yaw), _num(ref_yaw)
    d = 1.0 if (_num(direction) or 1.0) >= 0 else -1.0
    if y is None or r is None:
        return d
    off = _wrap(y - r)
    if off > half_rad:
        return -1.0
    if off < -half_rad:
        return 1.0
    if off >= half_rad - margin_rad and d > 0:
        return -1.0
    if off <= -half_rad + margin_rad and d < 0:
        return 1.0
    return d
