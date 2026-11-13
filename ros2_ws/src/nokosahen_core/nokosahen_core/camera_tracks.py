"""カメラだけの人の検出(距離なし)を、世界座標の方位で追跡して安定化する純粋ロジック(ROS非依存)。【2026-11-06】

実機の記録で、カメラの検出が1フレームごとに数・方位が入れ替わり、距離が無いので「一番近い人」の規則が使えず、向く先がぶれた。
そこで、検出を「世界の方位(yaw + bearing)」で追跡し、
  (a) 同じ世界方位(match_deg 以内)の検出を同じ対象に結びつける
  (b) confirm_n 回以上(confirm_window_sec 以内)検出された対象だけを出す(1フレームだけの検出は出さない)
  (c) 見えなくなっても hold_sec は出し続ける(旋回で視野から出ても世界の方位は変わらない)
  (d) 方位は指数移動平均(smooth_alpha)でならす
して、確かな対象の方位だけを「カメラ検出」として融合(crowd_bearing.fuse_candidates)に渡す。設計と根拠は docs/face_person_design.md 7.9章。
方位はロボット正面を0・左が正のラジアン。位置(yaw)が取れないときは安定化せず、検出をそのまま返す。
"""
import math
from typing import List, Optional

# 既定値は docs/face_person_design.md 7.9章の感度分析で決めた値
MATCH_DEG = 12.0          # 同じ対象とみなす世界方位の差(記録の検出と最寄りの対象の差の90%が12度以内)
CONFIRM_N = 2             # 出力に使う検出回数
CONFIRM_WINDOW_SEC = 3.0  # その回数を数える時間窓(確定前の対象は最後の検出からこの時間で捨てる)
HOLD_SEC = 1.6            # 確定した対象を、見えなくなってから覚えておく時間(検出の間隔は0.8秒。1回の取りこぼし=1.6秒を越える)
SMOOTH_ALPHA = 0.3        # 方位の指数移動平均の係数(1で平滑化なし。新しい検出の重み)

_EPS = 1e-9
_HIST = 8                 # 検出時刻の履歴の最大数


def _num(v) -> Optional[float]:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    v = float(v)
    return v if math.isfinite(v) else None


def _wrap(a: float) -> float:
    a = math.fmod(a + math.pi, 2 * math.pi)
    if a < 0:
        a += 2 * math.pi
    return a - math.pi


class _CamTrack:
    __slots__ = ("id", "world", "hits", "first_t", "last_t", "confirmed")

    def __init__(self, tid: int, world: float, t: float) -> None:
        self.id, self.world, self.first_t, self.last_t = tid, world, t, t
        self.hits: List[float] = [t]
        self.confirmed = False

    def add(self, world: float, t: float, alpha: float, window: float, n: int) -> None:
        self.world = _wrap(self.world + alpha * _wrap(world - self.world))
        self.last_t = t
        self.hits = [h for h in self.hits if t - h <= window][-(_HIST - 1):] + [t]
        if len(self.hits) >= n:
            self.confirmed = True


class CameraTrackBank:
    """update(now, yaw, bearings) -> 安定化した検出の方位(機体基準・rad)のリスト。

    同じ内容の検出リストが続けて渡されたとき(カメラの周期はLiDARより遅く、同じ検出が何回も届く。directorは1 tickに何度も呼ぶ)は、
    新しい観測として数えない(検出回数を水増ししない)。また、その間に機体が旋回しても、検出の世界方位は最初に見た時の yaw で決めたまま
    (繰り返し届く古い機体基準の方位に、旋回した後の yaw を足してずれるのを防ぐ)。"""

    def __init__(self, match_deg: float = MATCH_DEG, confirm_n: int = CONFIRM_N,
                 confirm_window_sec: float = CONFIRM_WINDOW_SEC, hold_sec: float = HOLD_SEC,
                 smooth_alpha: float = SMOOTH_ALPHA) -> None:
        self.match_rad = math.radians(max(0.0, match_deg))
        self.confirm_n = max(1, int(confirm_n))
        self.confirm_window_sec = max(0.0, confirm_window_sec)
        self.hold_sec = max(0.0, hold_sec)
        self.alpha = min(1.0, max(0.01, smooth_alpha))
        self.reset()

    def reset(self) -> None:
        self._tracks: List[_CamTrack] = []
        self._next_id = 1
        self._last_key = None
        self._last_obs_world: List[float] = []
        self._yaw_known: Optional[bool] = None
        self.observations = 0

    @property
    def tracks(self) -> List[_CamTrack]:
        return list(self._tracks)

    def update(self, now_sec, yaw, bearings) -> List[float]:
        now = _num(now_sec)
        y = _num(yaw)
        raw = []
        for b in (bearings if isinstance(bearings, (list, tuple)) else []):
            v = _num(b)
            if v is not None:
                raw.append(_wrap(v))
        if y is None or now is None:
            # 位置が取れない: 世界座標に直せないので安定化しない(従来どおり)。追跡は捨てる(位置が戻ったとき古い対象を引きずらない)。
            if self._tracks or self._yaw_known:
                self.reset()
            self._yaw_known = False
            return raw
        self._yaw_known = True

        key = tuple(sorted(round(b, 6) for b in raw))
        if raw and key != self._last_key:
            self._observe(now, [_wrap(y + b) for b in raw])
        if key != self._last_key:
            self._last_key = key
        # 期限切れを捨てる
        self._tracks = [t for t in self._tracks
                        if now - t.last_t <= (self.hold_sec if t.confirmed else self.confirm_window_sec) + _EPS]
        if now < max((t.last_t for t in self._tracks), default=now) - 1.0:
            self.reset()    # 時計が大きく戻った(別の再生など): 古い追跡を捨てる
            return raw
        return [_wrap(t.world - y) for t in sorted(self._tracks, key=lambda t: t.id) if t.confirmed]

    def _observe(self, now: float, worlds: List[float]) -> None:
        self.observations += 1
        pairs = []
        for ti, t in enumerate(self._tracks):
            for di, w in enumerate(worlds):
                d = abs(_wrap(w - t.world))
                if d <= self.match_rad + _EPS:
                    pairs.append((d, ti, di))
        pairs.sort()
        used_t, used_d = set(), set()
        for _d, ti, di in pairs:
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            self._tracks[ti].add(worlds[di], now, self.alpha, self.confirm_window_sec, self.confirm_n)
        for di, w in enumerate(worlds):
            if di not in used_d:
                tr = _CamTrack(self._next_id, w, now)
                self._next_id += 1
                if self.confirm_n <= 1:
                    tr.confirmed = True
                self._tracks.append(tr)


TIER2_AFTER_SEC = 4.0     # 階層1(確かな人)が全く無くなってから、階層2(LiDARだけの物)を使い始めるまでの時間
TIER1_HOLD_SEC = 2.0      # 階層1の各候補を、見えなくなってから覚えておく時間(カメラ一致は断続的に出るので、1人ずつ覚える)
TIER1_MATCH_DEG = 10.0    # 同じ候補とみなす世界方位の差
TIER1_MATCH_RANGE_M = 0.7
STATIC_REJ_SEC = 1.5      # カメラに人と否定され続けた(rejected)LiDAR候補を「静止物」とみなす継続時間(カメラの検出1回の取りこぼしでは静止物にしない)
STATIC_MEMORY_SEC = 20.0  # 静止物とみなした位置(世界方位+距離)を覚えておく時間。視野の外へ回って「否定」が届かなくなっても、階層2で軽いままにする
STATIC_MATCH_DEG = 8.0
STATIC_MATCH_RANGE_M = 0.4
_REJ_WEIGHT = 0.1         # crowd_bearing.REJECTED_WEIGHT と同じ(selectorは重みがこれ以下の候補を、他に候補があるときは使わない)


class TierFilter:
    """候補の階層化。【2026-11-06】
    階層1 = カメラとLiDARが一致した人(source="both")と、安定したカメラトラック(source="camera")。
    階層2 = LiDARだけの候補(source="lidar"。確認なし・カメラに否定されたもの)。机・椅子・壁などの静止物を含む。
    階層1が1つでもあるあいだは階層1だけを向く対象にする(階層2は無視)。階層1の候補は、見えなくなっても tier1_hold_sec は世界方位で
    覚えて出し続け(カメラ一致の断続で、候補が入れ替わったり階層2へ飛んだりしない)、階層1が全く無くなって tier2_after_sec 過ぎて初めて
    階層2を使う(階層1が一度も見えていないうちは待たずに階層2)。カメラの視野の外(後ろ)の人はカメラで確認できず階層2になるので、階層1が無い間は階層2で向く(階層1が見えたあとに無くなったときは、この時間だけ遅れる)。
    update(now, yaw, cands) の cands は crowd_bearing.fuse_candidates の出力(bearing は機体基準rad)。yaw が無ければ何もしない。"""

    def __init__(self, tier2_after_sec: float = TIER2_AFTER_SEC, hold_sec: float = TIER1_HOLD_SEC,
                 static_memory_sec: float = STATIC_MEMORY_SEC) -> None:
        self.tier2_after_sec = max(0.0, tier2_after_sec)
        self.static_memory_sec = max(0.0, static_memory_sec)
        self.hold_sec = max(0.0, hold_sec)
        self.reset()

    def reset(self) -> None:
        self._held: List[dict] = []
        self._t1_seen: Optional[float] = None
        self._start: Optional[float] = None
        self._statics: List[dict] = []
        self.tier2_active = False

    def _near_static(self, world: float, rng_m) -> Optional[dict]:
        for e in self._statics:
            if abs(_wrap(world - e["world"])) <= math.radians(STATIC_MATCH_DEG) + _EPS and (
                    rng_m is None or abs(rng_m - e["range"]) <= STATIC_MATCH_RANGE_M):
                return e
        return None

    def _learn_statics(self, now: float, y: float, items: list) -> None:
        self._statics = [e for e in self._statics if now - e["last_t"] <= self.static_memory_sec + _EPS]
        for c in items:
            b, r = _num(c.get("bearing")), _num(c.get("range"))
            if b is None or r is None:
                continue
            w = _wrap(y + b)
            e = self._near_static(w, r)
            if c.get("source") == "lidar" and (_num(c.get("weight")) or 1.0) <= _REJ_WEIGHT + _EPS:
                if e is None:
                    self._statics.append({"world": w, "range": r, "first_t": now, "last_t": now})
                else:
                    e.update(world=w, range=r, last_t=now)
            elif e is not None and c.get("source") in ("both", "camera"):
                self._statics.remove(e)      # そこにカメラ確認済みの人が出た: 静止物ではない

    def update(self, now_sec, yaw, cands) -> list:
        now, y = _num(now_sec), _num(yaw)
        items = [c for c in (cands if isinstance(cands, (list, tuple)) else []) if isinstance(c, dict)]
        if now is None or y is None:
            self.reset()
            return items
        if self._start is None:
            self._start = now
        if self._t1_seen is not None and now < self._t1_seen - 1e-6:    # 時計が戻った
            self.reset()
            self._start = now
        self._learn_statics(now, y, items)
        t1 = [c for c in items if c.get("source") in ("both", "camera") and _num(c.get("bearing")) is not None]
        self._held = [h for h in self._held if now - h["last_t"] <= self.hold_sec + _EPS]
        cur = [(_wrap(y + _num(c["bearing"])), c) for c in t1]
        pairs = []
        for hi, h in enumerate(self._held):
            for ci, (w, c) in enumerate(cur):
                d = abs(_wrap(w - h["world"]))
                r, hr = _num(c.get("range")), h["c"].get("range")
                if d <= math.radians(TIER1_MATCH_DEG) + _EPS and (r is None or hr is None or abs(r - hr) <= TIER1_MATCH_RANGE_M):
                    pairs.append((d, hi, ci))
        pairs.sort()
        used_h, used_c = set(), set()
        for _d, hi, ci in pairs:
            if hi in used_h or ci in used_c:
                continue
            used_h.add(hi)
            used_c.add(ci)
            self._held[hi].update(world=cur[ci][0], c=cur[ci][1], last_t=now)
        for ci, (w, c) in enumerate(cur):
            if ci not in used_c:
                self._held.append({"world": w, "c": c, "last_t": now})
        if t1:
            self._t1_seen = now      # 実際に見えた時刻(覚えているだけの間は進めない)
        if self._held:
            self.tier2_active = False
            return [dict(h["c"]) if h["last_t"] == now else dict(h["c"], bearing=_wrap(h["world"] - y)) for h in self._held]
        # 階層1が一度も見えていない間(起動直後・LiDARだけの人しか居ない)は待たずに階層2を使う。待つのは、階層1が見えていたあとに無くなったときだけ
        last = self._t1_seen
        if last is not None and now - last < self.tier2_after_sec - _EPS:
            self.tier2_active = False
            return []
        self.tier2_active = True
        # 階層2: 静止物とみなした位置の候補は軽くする(他に候補があれば使われない)
        out = []
        for c in items:
            b, r = _num(c.get("bearing")), _num(c.get("range"))
            e = self._near_static(_wrap(y + b), r) if (b is not None and r is not None) else None
            if e is not None and e["last_t"] - e["first_t"] >= STATIC_REJ_SEC - _EPS:
                c = dict(c, weight=min(_num(c.get("weight")) or _REJ_WEIGHT, _REJ_WEIGHT))
            out.append(c)
        return out
