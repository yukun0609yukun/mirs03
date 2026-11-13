"""【2026-11-06】「向く」(その場の旋回)中の位置のずれを抑える、純粋ロジック(ROS非依存。numpyのみ)。

背景: 実機で、その場の旋回(linear.x=0)なのに、左回りのときだけ車体がじわじわ前へ進んだ(約1.4 cm/rad、24秒で9.5 cm)。
原因は左右の車輪の非対称(摩擦/PID)と推測される(PIDは別途、実機で調整)。ここでは制御側で打ち消す。
既定は無効(director の face_position_hold_enabled=false)。実機で試して有効にする。docs/face_person_design.md 7.10章。

2つのループ
- 内側(速度): 旋回中に、オドメトリの前後方向の速度 v_odom を測り、旋回の方向(左/右)ごとに「旋回1 rad あたりのずれ c [m/rad]」を
  少しずつ学習して、旋回の指令に足す linear.x = -c*|wz| を作る(上限 max_ff_mps)。立ち上がりの遅れ(ファームの加速ランプ)があるので、
  方向が変わってから settle_sec の間は学習しない・速度はEMAで平滑化・出力にスルーレート制限をかける。cは方向ごとに覚えるので、往復しても引き継ぐ。
- 外側(位置): アンカー(向く動作に入った地点)からの変位 D を測り、旋回が止まっている間だけ、車体の前後方向の成分 along が
  start_m を超えたら、ゆっくり(return_speed_mps)戻す(stop_m 未満で終了=ヒステリシス)。横方向の成分は戻せない(差動二輪)ので、
  戻さず、大きければ警告を出す。1回あたりの最大移動量・タイムアウト・停滞の検出あり。アンカーから max_total_m を超えたら、以後ぜんぶ補正しない。

安全: allow=False(director側の安全条件が1つでも外れた)・位置/オドメトリが古い・進む方向の障害物の距離が足りない/不明、のどれかで
その場で linear=0(スルーレートを待たず即座に)。出力は常に ±max_ff_mps と ±return_speed_mps の大きい方以内。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Dict, Optional, Tuple

import numpy as np

IDLE = "idle"
RETURN = "return"


@dataclass
class HoldParams:
    # --- 内側ループ ---
    max_ff_mps: float = 0.05        # 旋回中に足してよい linear.x の上限[m/s]
    ki: float = 0.8                 # 学習ゲイン[1/s](時定数 1/ki 秒。遅れ約0.6秒に対し、発振しない大きさ)
    v_filter_tau: float = 0.3       # odom 速度の平滑化の時定数[秒]
    settle_sec: float = 0.4         # 実際の旋回の方向が変わってから学習を始めるまで/旋回が止まってから戻しを始めるまでの待ち[秒]
    lead_sec: float = 0.3           # 指令がオドメトリの速度に現れるまでの遅れ(指令の遅れ+ランプ後のフィルタ)。この先の旋回速度を予測して補正を出す
    ramp_acc_rps2: float = 1.6      # ファームの角加速ランプ[rad/s^2](firmware config.h RAMP_ACCEL_ANGULAR_RPS2)
    ramp_dec_rps2: float = 4.0      # 同、減速(RAMP_DECEL_ANGULAR_RPS2)
    learn_steady_rps: float = 0.15  # 予測と測定の旋回速度の差がこれ以下(=安定した旋回)のときだけ学習する
    rot_min_rps: float = 0.15       # 実際の旋回速度(odom)がこれ未満なら、まだ回っていない(補正しない)
    slew_mps2: float = 0.15         # 出力の変化率の上限[m/s^2]
    turn_min_rps: float = 0.3       # これ以上の |wz| の指令を「旋回中」とみなす
    learn_min_rps: float = 0.5      # これ以上の |wz| のときだけ学習する
    c_max: float = 0.04             # 学習する c の上限[m/rad]
    deadband_mps: float = 0.002     # これ未満の出力は 0 とみなす
    seed_c_left: float = 0.0        # cの初期値(左回り)。0=未学習で始める
    seed_c_right: float = 0.0
    # --- 外側ループ ---
    start_m: float = 0.07           # 前後方向のずれがこれを超えたら戻し始める
    stop_m: float = 0.025           # これ未満になったら戻し終わる
    max_total_m: float = 0.25       # アンカーからの変位がこれを超えたら、以後ぜんぶ補正しない
    over_clear_ratio: float = 0.8   # 超過の解除(変位 < max_total_m * 比)
    reanchor_m: float = 1.0         # これ以上離れたら、別の場所に動かされたとみなしてアンカーを取り直す
    lateral_warn_m: float = 0.15    # 横方向のずれの警告
    return_speed_mps: float = 0.06  # 戻す速度の上限(0.08 以下)
    return_min_mps: float = 0.03    # 戻す速度の下限(これ未満だと車体が動かないため)
    return_gain: float = 1.0        # 戻す速度 = gain * 残り距離[m]  (1/s)
    max_episode_m: float = 0.20     # 1回の戻しで動いてよい最大距離
    episode_timeout_sec: float = 8.0
    stall_sec: float = 3.0          # この時間で stall_min_m 動かなければ打ち切る
    stall_min_m: float = 0.01
    cooldown_sec: float = 3.0       # 戻し終わってから次の戻しを始めるまで
    stall_cooldown_sec: float = 15.0
    # --- 共通 ---
    clear_fwd_m: float = 0.6        # 前進するときの、前方の障害物までの距離[m]の下限
    clear_back_m: float = 0.6       # 後退するときの、後方の障害物までの距離[m]の下限
    gap_reset_sec: float = 2.0      # 呼ばれない時間がこれを超えたら、新しい「向く」動作とみなしてアンカーを取り直す


def sector_min_range(
    ranges, angle_min: float, angle_increment: float, center: float, half_width: float,
    range_min: float, range_max: float, min_valid_frac: float = 0.6,
) -> Optional[float]:
    """LiDARの扇形(中心 center[rad]、半幅 half_width[rad]。機体基準で前=0、左が正)の中で一番近い距離[m]。
    扇形の点の min_valid_frac 未満しか有効(範囲内の距離 か +inf=何も無い)でない、扇形に点が無い、入力が不正なときは None(=不明)。
    全部 inf なら range_max(=十分遠い)を返す。NaN は無効。"""
    try:
        r = np.asarray(ranges, dtype=np.float64)
        n = int(r.size)
        inc = float(angle_increment)
        if n == 0 or not math.isfinite(inc) or inc == 0.0 or not math.isfinite(float(angle_min)):
            return None
        ang = float(angle_min) + inc * np.arange(n)
        diff = np.abs((ang - float(center) + math.pi) % (2.0 * math.pi) - math.pi)
        sel = diff <= float(half_width)
        if not np.any(sel):
            return None
        rs = r[sel]
        finite_ok = np.isfinite(rs) & (rs >= float(range_min)) & (rs <= float(range_max))
        valid = finite_ok | np.isposinf(rs)
        if float(np.mean(valid)) < float(min_valid_frac):
            return None
        if np.any(finite_ok):
            return float(np.min(rs[finite_ok]))
        return float(range_max)
    except Exception:  # noqa: BLE001
        return None


def predict_wz(wz_now: float, wz_cmd: float, lead_sec: float, acc: float, dec: float) -> float:
    """今の実際の旋回速度 wz_now から、指令 wz_cmd を出し続けたとき lead_sec 秒後の旋回速度を、ファームのランプ(加速 acc・減速/反転 dec)で予測する。"""
    w = float(wz_now)
    left = max(0.0, float(lead_sec))
    while left > 1e-9:
        h = min(0.05, left)
        rate = acc if (abs(wz_cmd) > abs(w) and wz_cmd * w >= 0.0) else dec
        w += max(-rate * h, min(rate * h, wz_cmd - w))
        left -= h
    return w


def _finite(*vals) -> bool:
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(float(v)) for v in vals)


class PositionHold:
    """位置保持の本体。step() を、向く動作の各tick(10 Hz程度)で呼ぶ。戻り値の linear.x [m/s] を旋回の指令に足す。"""

    def __init__(self, params: Optional[HoldParams] = None, log: Optional[Callable[[str, str], None]] = None) -> None:
        self.p = params or HoldParams()
        self._log = log or (lambda level, msg: None)
        self.c: Dict[int, float] = {1: self.p.seed_c_left, -1: self.p.seed_c_right}
        self._reset_all()

    # ---- 状態 ----
    def _reset_all(self) -> None:
        self._t: Optional[float] = None
        self.anchor: Optional[Tuple[float, float]] = None
        self.anchor_src: Optional[str] = None
        self._u = 0.0                 # 出力(スルーレート制限後)
        self._target = 0.0
        self._applied = False         # 前のtickの出力が目標に追いついていた(学習してよい)
        self._dir = 0
        self._dir_t = 0.0
        self._vf: Optional[float] = None
        self._last_turn_t = 0.0
        self.state = IDLE
        self._ep_t0 = 0.0
        self._ep_along0 = 0.0
        self._chk_t = 0.0
        self._chk_along = 0.0
        self._cool_until = 0.0
        self.over = False
        self._over_warned = False
        self._lat_warned = False
        self._inner_logged = False
        self._last_reason = ""
        self._last_reason_log_t = -1e9
        self.reason = ""
        self.drift_m: Optional[float] = None
        self.along_m: Optional[float] = None
        self.lateral_m: Optional[float] = None
        self.episodes = 0

    def reset(self, why: str = "") -> None:
        """動作の終わり・無効化: 出力と状態を捨てる(cは残す)。"""
        if self.state == RETURN:
            self._end_episode(f"中断({why})" if why else "中断", 0.0)
        c = dict(self.c)
        self._reset_all()
        self.c = c

    def status(self) -> dict:
        return {
            "state": self.state,
            "linear": round(self._u, 4),
            "drift_m": None if self.drift_m is None else round(self.drift_m, 4),
            "along_m": None if self.along_m is None else round(self.along_m, 4),
            "lateral_m": None if self.lateral_m is None else round(self.lateral_m, 4),
            "anchor": None if self.anchor is None else [round(self.anchor[0], 3), round(self.anchor[1], 3)],
            "anchor_src": self.anchor_src,
            "over_limit": bool(self.over),
            "reason": self.reason,
            "c_left": round(self.c[1], 5),
            "c_right": round(self.c[-1], 5),
            "episodes": self.episodes,
        }

    # ---- 内部 ----
    def _zero(self) -> None:
        self._u = 0.0
        self._target = 0.0
        self._applied = False

    def _end_episode(self, why: str, t: float) -> None:
        self._log("info", f"位置保持: 戻しを終了しました({why}、前後のずれ {0.0 if self.along_m is None else self.along_m * 100:.1f} cm)")
        self.state = IDLE
        self._cool_until = t + (self.p.stall_cooldown_sec if "停滞" in why else self.p.cooldown_sec)

    def _note_reason(self, t: float, reason: str) -> None:
        self.reason = reason
        if reason and reason != self._last_reason and (t - self._last_reason_log_t) >= 5.0 and self._u != 0.0:
            self._log("info", f"位置保持: 補正を止めました({reason})")
            self._last_reason_log_t = t
        self._last_reason = reason

    # ---- 本体 ----
    def step(
        self,
        t: float,
        wz: float,
        allow: bool = True,
        block_reason: str = "",
        pos: Optional[Tuple[float, float, float, str]] = None,   # (x, y, yaw, 座標系の名前)。古い/無いときは None
        v_odom: Optional[float] = None,                          # odom の linear.x [m/s]。古い/無いときは None
        wz_odom: Optional[float] = None,                         # odom の angular.z [rad/s](実際の旋回速度)。無ければ指令 wz を使う
        clear_fwd: Optional[float] = None,                       # 前方の障害物までの距離[m]。不明/古いときは None
        clear_back: Optional[float] = None,
    ) -> float:
        p = self.p
        if not _finite(t):
            self._zero()
            return 0.0
        dt = 0.0 if self._t is None else min(max(t - self._t, 0.0), 0.5)
        if self._t is not None and (t - self._t) > p.gap_reset_sec:
            self.reset("間が空いた")
            dt = 0.0
        if self._t is None:
            self._last_turn_t = t
        self._t = t
        wz = float(wz) if _finite(wz) else 0.0
        if not allow:
            if self.state == RETURN:
                self._end_episode(f"中断({block_reason})", t)
            self._note_reason(t, block_reason or "blocked")
            self._zero()
            self._vf = None
            return 0.0

        # --- 変位 ---
        along = lateral = None
        if pos is not None and len(pos) == 4 and _finite(pos[0], pos[1], pos[2]):
            x, y, yaw, src = float(pos[0]), float(pos[1]), float(pos[2]), str(pos[3])
            if self.anchor is None or self.anchor_src != src:
                self.anchor, self.anchor_src = (x, y), src
                self._over_warned = self._lat_warned = False
                self.over = False
                self._log("info", f"位置保持: アンカーを記録しました({src}座標 x={x:.3f} y={y:.3f})")
            dx, dy = x - self.anchor[0], y - self.anchor[1]
            if math.hypot(dx, dy) > p.reanchor_m:
                self._log("warn", "位置保持: アンカーから大きく離れました(動かされた?)。アンカーを取り直します")
                self.anchor = (x, y)
                dx = dy = 0.0
                self._over_warned = self._lat_warned = False
                self.over = False
            d = math.hypot(dx, dy)
            along = dx * math.cos(yaw) + dy * math.sin(yaw)
            lateral = -dx * math.sin(yaw) + dy * math.cos(yaw)
            self.drift_m, self.along_m, self.lateral_m = d, along, lateral
            if not self.over and d > p.max_total_m:
                self.over = True
                if not self._over_warned:
                    self._over_warned = True
                    self._log("warn", f"位置保持: アンカーから {d * 100:.0f} cm 離れました(上限 {p.max_total_m * 100:.0f} cm)。これ以上は補正しません")
            elif self.over and d < p.max_total_m * p.over_clear_ratio:
                self.over = False
            if abs(lateral) > p.lateral_warn_m and not self._lat_warned:
                self._lat_warned = True
                self._log("warn", f"位置保持: 横方向に {lateral * 100:.0f} cm ずれています(差動二輪なので戻せません。前後方向だけ補正します)")
        else:
            self.drift_m = self.along_m = self.lateral_m = None

        turning = abs(wz) >= p.turn_min_rps
        target = 0.0
        reason = ""
        if turning:
            self._last_turn_t = t
            if self.state == RETURN:
                self._end_episode("旋回が再開", t)
                self._zero()
            if self.over:
                reason = "over_limit"
            elif v_odom is None or not _finite(v_odom):
                reason = "odom_stale"
                self._vf = None
            elif pos is None:
                reason = "pos_stale"   # 位置が取れないときは出さない(アンカー・上限の判定ができない)
            else:
                # 実際の旋回速度(odom)に比例させる。指令が反転しても、車体が実際に回り始めるまで(加速ランプの間)は補正しない。
                wzm = float(wz_odom) if (wz_odom is not None and _finite(wz_odom)) else wz
                d_ = 0 if abs(wzm) < p.rot_min_rps else (1 if wzm > 0 else -1)
                if d_ != self._dir:
                    self._dir, self._dir_t, self._vf = d_, t, None
                if d_ != 0:
                    v = float(v_odom)
                    if self._vf is None:
                        self._vf = v
                    else:
                        a = dt / (p.v_filter_tau + dt) if dt > 0 else 0.0
                        self._vf += a * (v - self._vf)
                    # 学習は、旋回が安定している(予測と測定がほぼ同じ)ときだけ。
                    wzp = predict_wz(wzm, wz, p.lead_sec, p.ramp_acc_rps2, p.ramp_dec_rps2)
                    steady = abs(wzp - wzm) <= p.learn_steady_rps
                    if (t - self._dir_t) >= p.settle_sec and abs(wzm) >= p.learn_min_rps and dt > 0 and self._applied and steady:
                        c = self.c[d_] + p.ki * (self._vf / abs(wzm)) * dt
                        self.c[d_] = max(-p.c_max, min(p.c_max, c))
                else:
                    wzp = predict_wz(wzm, wz, p.lead_sec, p.ramp_acc_rps2, p.ramp_dec_rps2)
                # 出力は「補正が車体に届く頃(lead_sec 後)の旋回速度」の予測に比例させる(ファームのランプと遅れで、補正が旋回より遅れて届くのを打ち消す)。
                dp = 0 if abs(wzp) < p.rot_min_rps else (1 if wzp > 0 else -1)
                if dp != 0:
                    target = -self.c[dp] * abs(wzp)
                    target = max(-p.max_ff_mps, min(p.max_ff_mps, target))
                    if not self._inner_logged and abs(target) >= p.deadband_mps:
                        self._inner_logged = True
                        self._log("info", f"位置保持: 旋回中の補正を開始しました(c左={self.c[1] * 100:.2f} cm/rad c右={self.c[-1] * 100:.2f} cm/rad)")
        else:
            self._dir = 0
            self._vf = None
            if self.over:
                reason = "over_limit"
            elif pos is None or along is None:
                reason = "pos_stale"
                if self.state == RETURN:
                    self._end_episode("位置が取れない", t)
            else:
                idle_for = t - self._last_turn_t
                if self.state == IDLE:
                    still = wz_odom is None or not _finite(wz_odom) or abs(float(wz_odom)) < p.rot_min_rps
                    if abs(along) >= p.start_m and idle_for >= p.settle_sec and still and t >= self._cool_until:
                        self.state = RETURN
                        self.episodes += 1
                        self._ep_t0 = self._chk_t = t
                        self._ep_along0 = self._chk_along = along
                        self._log("info", f"位置保持: 戻しを開始しました(前後のずれ {along * 100:.1f} cm、変位 {self.drift_m * 100:.1f} cm)")
                if self.state == RETURN:
                    if abs(along) <= p.stop_m:
                        self._end_episode("戻った", t)
                        self._zero()
                    elif abs(along - self._ep_along0) >= p.max_episode_m:
                        self._end_episode("1回の最大移動量", t)
                        self._zero()
                    elif (t - self._ep_t0) >= p.episode_timeout_sec:
                        self._end_episode("タイムアウト", t)
                        self._zero()
                    else:
                        if (t - self._chk_t) >= p.stall_sec:
                            if abs(along - self._chk_along) < p.stall_min_m:
                                self._log("warn", "位置保持: 戻しているのに動きません(停滞)。打ち切ります")
                                self._end_episode("停滞", t)
                                self._zero()
                            else:
                                self._chk_t, self._chk_along = t, along
                        if self.state == RETURN:
                            sp = max(p.return_min_mps, min(p.return_speed_mps, p.return_gain * abs(along)))
                            target = -math.copysign(sp, along)

        # --- 進む方向の障害物(距離が足りない/不明なら 0 に戻す) ---
        if abs(target) >= p.deadband_mps and not reason:
            if target > 0 and not (clear_fwd is not None and _finite(clear_fwd) and clear_fwd >= p.clear_fwd_m):
                reason = "front_blocked"
            elif target < 0 and not (clear_back is not None and _finite(clear_back) and clear_back >= p.clear_back_m):
                reason = "back_blocked"
        if reason:
            if self.state == RETURN:
                self._end_episode(f"中断({reason})", t)
            self._note_reason(t, reason)
            self._zero()
            return 0.0
        self._note_reason(t, "")

        if abs(target) < p.deadband_mps:
            target = 0.0
        # --- スルーレート制限 ---
        step = p.slew_mps2 * dt
        if dt <= 0.0:
            u = self._u
        elif target > self._u:
            u = min(target, self._u + step)
        else:
            u = max(target, self._u - step)
        if abs(u) < p.deadband_mps:
            u = 0.0
        self._u = u
        self._target = target
        self._applied = abs(u - target) <= 0.004
        lim = max(p.max_ff_mps, p.return_speed_mps)
        return max(-lim, min(lim, u))
