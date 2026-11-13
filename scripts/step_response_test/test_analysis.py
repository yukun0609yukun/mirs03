"""analysis.pyの単体テスト(ロボット無し・ネットワーク無しで実行できる)。

実行方法:
    cd scripts/step_response_test
    python3 -m pytest test_analysis.py -v
"""
from __future__ import annotations

import math

import pytest

from analysis import compute_step_response_metrics, simulate_step_response


def test_no_overshoot_first_order_is_ok():
    times, distances = simulate_step_response(
        target_m=1.0, duration_s=8.0, rate_hz=20.0, kind="first_order", tau_s=1.0
    )
    metrics = compute_step_response_metrics(times, distances, target_m=1.0, os_threshold_pct=5.0, tol_pct=2.0)
    assert metrics.judgement == "OK"
    assert metrics.overshoot_pct == pytest.approx(0.0, abs=1e-6)
    # 1次遅れは単調増加なのでd_maxは目標未満(タイムアウトで打ち切っているため)。
    assert metrics.d_max <= 1.0 + 1e-6


def test_underdamped_second_order_overshoots_and_is_ng():
    times, distances = simulate_step_response(
        target_m=1.0, duration_s=6.0, rate_hz=20.0, kind="second_order", zeta=0.3, wn=2.5
    )
    metrics = compute_step_response_metrics(times, distances, target_m=1.0, os_threshold_pct=5.0, tol_pct=2.0)
    assert metrics.overshoot_pct > 5.0
    assert metrics.judgement == "NG"
    assert metrics.peak_time_s is not None
    assert metrics.d_max > 1.0


def test_overdamped_second_order_no_overshoot():
    times, distances = simulate_step_response(
        target_m=2.0, duration_s=6.0, rate_hz=20.0, kind="second_order", zeta=1.5, wn=2.0
    )
    metrics = compute_step_response_metrics(times, distances, target_m=2.0, os_threshold_pct=5.0, tol_pct=2.0)
    assert metrics.overshoot_pct == pytest.approx(0.0, abs=1e-6)
    assert metrics.judgement == "OK"


def test_overshoot_exact_value_against_hand_computed_curve():
    # d_max=1.10, target=1.0 -> overshoot 10% ちょうど、しきい値5%超えでNG。
    times = [0.0, 1.0, 2.0, 3.0, 4.0]
    distances = [0.0, 0.5, 1.10, 1.02, 1.00]
    metrics = compute_step_response_metrics(times, distances, target_m=1.0, os_threshold_pct=5.0, tol_pct=2.0)
    assert metrics.d_max == pytest.approx(1.10)
    assert metrics.overshoot_m == pytest.approx(0.10)
    assert metrics.overshoot_pct == pytest.approx(10.0)
    assert metrics.judgement == "NG"
    assert metrics.peak_time_s == pytest.approx(2.0)


def test_rise_time_10_to_90_linear_ramp():
    # 0->1mへ0〜10秒の線形ランプ(0.1m/s)。10%=1s地点、90%=9s地点のはず。
    times = [float(i) for i in range(11)]
    distances = [t * 0.1 for t in times]
    metrics = compute_step_response_metrics(times, distances, target_m=1.0, os_threshold_pct=5.0, tol_pct=2.0)
    assert metrics.t10_s == pytest.approx(1.0, abs=1e-6)
    assert metrics.t90_s == pytest.approx(9.0, abs=1e-6)
    assert metrics.rise_time_s == pytest.approx(8.0, abs=1e-6)


def test_settling_time_detects_last_excursion_outside_tolerance():
    # target=1.0, tol=5% -> 許容帯は[0.95, 1.05]。
    # t=3で帯外(1.10)に戻ってから、以降ずっと帯内なのでsettling_time=4。
    times = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    distances = [0.0, 0.5, 1.02, 1.10, 0.99, 1.00]
    metrics = compute_step_response_metrics(times, distances, target_m=1.0, os_threshold_pct=20.0, tol_pct=5.0)
    assert metrics.settling_time_s == pytest.approx(4.0)


def test_never_settles_returns_none():
    times = [0.0, 1.0, 2.0]
    distances = [0.0, 1.5, 1.5]  # ずっと許容帯の外
    metrics = compute_step_response_metrics(times, distances, target_m=1.0, os_threshold_pct=100.0, tol_pct=2.0)
    assert metrics.settling_time_s is None


def test_steady_state_error_averages_tail_window():
    times = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    distances = [0.0, 0.5, 0.9, 1.02, 0.98, 1.02, 0.98]
    metrics = compute_step_response_metrics(
        times, distances, target_m=1.0, os_threshold_pct=5.0, tol_pct=2.0, steady_state_window=4
    )
    # 末尾4点: 1.02, 0.98, 1.02, 0.98 -> 平均1.00 -> 誤差0.0
    assert metrics.steady_state_error_m == pytest.approx(0.0, abs=1e-6)


def test_zero_target_does_not_divide_by_zero():
    times = [0.0, 1.0, 2.0]
    distances = [0.0, 0.1, 0.05]
    metrics = compute_step_response_metrics(times, distances, target_m=0.0, os_threshold_pct=5.0, tol_pct=2.0)
    assert metrics.overshoot_pct == 0.0
    assert metrics.t10_s is None
    assert metrics.t90_s is None
    assert metrics.settling_time_s is None


def test_mismatched_lengths_raise():
    with pytest.raises(ValueError):
        compute_step_response_metrics([0.0, 1.0], [0.0], target_m=1.0)


def test_empty_series_raise():
    with pytest.raises(ValueError):
        compute_step_response_metrics([], [], target_m=1.0)


def test_simulate_step_response_sample_count_matches_rate_and_duration():
    times, distances = simulate_step_response(target_m=1.0, duration_s=5.0, rate_hz=10.0, kind="first_order")
    assert len(times) == len(distances)
    assert times[0] == 0.0
    assert times[-1] == pytest.approx(5.0)
    assert len(times) == 51


def test_simulate_step_response_noise_is_reproducible_with_seed():
    t1, d1 = simulate_step_response(
        target_m=1.0, duration_s=2.0, rate_hz=10.0, kind="first_order", noise_std_m=0.02, seed=42
    )
    t2, d2 = simulate_step_response(
        target_m=1.0, duration_s=2.0, rate_hz=10.0, kind="first_order", noise_std_m=0.02, seed=42
    )
    assert d1 == d2


def test_simulate_step_response_rejects_invalid_rate():
    with pytest.raises(ValueError):
        simulate_step_response(target_m=1.0, duration_s=2.0, rate_hz=0.0)


def test_simulate_step_response_rejects_unknown_kind():
    with pytest.raises(ValueError):
        simulate_step_response(target_m=1.0, duration_s=2.0, rate_hz=10.0, kind="bogus")
