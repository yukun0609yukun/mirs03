"""走行距離ステップ応答の指標計算と、--simモード用の疑似応答生成。

ロボット・ROS 2に一切依存しない純粋関数だけを置く(test_analysis.pyから
ロボット無しで単体テストできるようにするため)。CLI本体(step_response_test.py)
からも、ダッシュボード表示用のJSON生成からも、ここの関数を呼ぶだけにする。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence


@dataclass
class StepResponseMetrics:
    target_m: float
    d_max: float
    overshoot_m: float
    overshoot_pct: float
    peak_time_s: Optional[float]
    rise_time_s: Optional[float]
    t10_s: Optional[float]
    t90_s: Optional[float]
    settling_time_s: Optional[float]
    steady_state_error_m: Optional[float]
    judgement: str  # "OK" or "NG"
    os_threshold_pct: float
    tol_pct: float

    def to_dict(self) -> dict:
        return {
            "target_m": self.target_m,
            "d_max": self.d_max,
            "overshoot_m": self.overshoot_m,
            "overshoot_pct": self.overshoot_pct,
            "peak_time_s": self.peak_time_s,
            "rise_time_s": self.rise_time_s,
            "t10_s": self.t10_s,
            "t90_s": self.t90_s,
            "settling_time_s": self.settling_time_s,
            "steady_state_error_m": self.steady_state_error_m,
            "judgement": self.judgement,
            "os_threshold_pct": self.os_threshold_pct,
            "tol_pct": self.tol_pct,
        }


def _first_crossing_time(
    times: Sequence[float], distances: Sequence[float], level: float
) -> Optional[float]:
    """distances が(0から上昇していく前提で)level を初めて上回った時刻を返す。

    一度も届かない場合はNone。線形補間で、サンプル間のちょうどの時刻を推定する。
    """
    prev_t, prev_d = None, None
    for t, d in zip(times, distances):
        if prev_d is not None and prev_d < level <= d:
            if d == prev_d:
                return t
            frac = (level - prev_d) / (d - prev_d)
            return prev_t + frac * (t - prev_t)
        prev_t, prev_d = t, d
    return None


def compute_step_response_metrics(
    times: Sequence[float],
    distances: Sequence[float],
    target_m: float,
    os_threshold_pct: float = 5.0,
    tol_pct: float = 2.0,
    steady_state_window: int = 5,
) -> StepResponseMetrics:
    """時刻[s]・走行距離[m]の時系列から、ステップ応答の指標を計算する。

    target_m<=0の場合は距離ベースの%指標(オーバーシュート%等)は計算できない
    ため0扱いにする(ゼロ割回避)。
    """
    if len(times) != len(distances):
        raise ValueError("times and distances must be the same length")
    if not times:
        raise ValueError("times/distances must not be empty")

    d_max = max(distances)
    peak_idx = distances.index(d_max) if isinstance(distances, list) else list(distances).index(d_max)
    peak_time = times[peak_idx]

    overshoot_m = max(0.0, d_max - target_m)
    overshoot_pct = (overshoot_m / target_m * 100.0) if target_m > 0 else 0.0

    t10 = _first_crossing_time(times, distances, target_m * 0.10) if target_m > 0 else None
    t90 = _first_crossing_time(times, distances, target_m * 0.90) if target_m > 0 else None
    rise_time = (t90 - t10) if (t10 is not None and t90 is not None) else None

    # 整定時間: 許容帯(±tol_pct%)の外に最後にいた時刻の「次」の時刻。
    # 末尾まで帯の外にいたままならNone(整定できなかった)。
    settling_time: Optional[float] = None
    if target_m > 0:
        tol_abs = target_m * (tol_pct / 100.0)
        last_outside_idx: Optional[int] = None
        for i, d in enumerate(distances):
            if abs(d - target_m) > tol_abs:
                last_outside_idx = i
        if last_outside_idx is None:
            settling_time = times[0]
        elif last_outside_idx < len(times) - 1:
            settling_time = times[last_outside_idx + 1]
        else:
            settling_time = None

    window = distances[-steady_state_window:] if len(distances) >= 1 else []
    steady_state_error = (sum(window) / len(window) - target_m) if window else None

    judgement = "NG" if overshoot_pct > os_threshold_pct else "OK"

    return StepResponseMetrics(
        target_m=target_m,
        d_max=d_max,
        overshoot_m=overshoot_m,
        overshoot_pct=overshoot_pct,
        peak_time_s=peak_time,
        rise_time_s=rise_time,
        t10_s=t10,
        t90_s=t90,
        settling_time_s=settling_time,
        steady_state_error_m=steady_state_error,
        judgement=judgement,
        os_threshold_pct=os_threshold_pct,
        tol_pct=tol_pct,
    )


def simulate_step_response(
    target_m: float,
    duration_s: float,
    rate_hz: float,
    kind: str = "second_order",
    tau_s: float = 1.2,
    zeta: float = 0.5,
    wn: float = 2.5,
    noise_std_m: float = 0.0,
    seed: Optional[int] = 12345,
) -> "tuple[List[float], List[float]]":
    """--simモード用: ロボット無しで疑似の走行距離応答を生成する。

    kind="first_order": 1次遅れ(オーバーシュートしない、常にOK判定になる)。
    kind="second_order": 2次系(zeta<1で減衰振動=オーバーシュートが出る、
    ゲイン調整で狙うNG検出パスをsimでも試せるようにするための既定)。
    """
    if rate_hz <= 0:
        raise ValueError("rate_hz must be positive")
    n = max(1, int(round(duration_s * rate_hz)))
    times = [i / rate_hz for i in range(n + 1)]

    if kind == "first_order":
        distances = [target_m * (1.0 - math.exp(-t / max(1e-6, tau_s))) for t in times]
    elif kind == "second_order":
        if zeta < 1.0:
            wd = wn * math.sqrt(1.0 - zeta * zeta)
            distances = [
                target_m
                * (
                    1.0
                    - math.exp(-zeta * wn * t)
                    * (math.cos(wd * t) + (zeta * wn / wd) * math.sin(wd * t))
                )
                for t in times
            ]
        else:
            # 臨界減衰/過減衰(zeta>=1)はオーバーシュートしない解析解。
            distances = [
                target_m * (1.0 - math.exp(-wn * t) * (1.0 + wn * t)) for t in times
            ]
    else:
        raise ValueError(f"unknown kind: {kind}")

    if noise_std_m > 0:
        import random

        rng = random.Random(seed)
        distances = [d + rng.gauss(0.0, noise_std_m) for d in distances]

    return times, distances
