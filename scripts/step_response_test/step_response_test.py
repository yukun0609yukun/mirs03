#!/usr/bin/env python3
"""走行距離/回転角度のステップ応答テスト。

目標走行距離または目標回転角度(ステップ入力)を与え、実際の応答が時間と
ともにどう追従するかを記録・プロットし、オーバーシュートしていないかを
自動判定する。ゲイン調整(ESP32 PID・速度プロファイル等)の前後比較に使う。

【設計方針】
- ロボットを実際に動かす部分は、既存の操作盤ダッシュボード
  (stage1/stage2/full_dashboard_node.py、いずれもdashboard_common.pyの
  PidGainsController.run_drive_test()を使う)に既に実装済みの走行テスト
  機能(`/api/drive_test`、forward/backward/rotate_left/rotate_right)に
  そのまま乗せる。このスクリプト自身は新たに/cmd_velをpublishしたり
  ROS 2ノードを起動したりしない(既存の安全機構 ソフト緊急停止・速度/時間
  クランプ・同時実行防止・Nav2/behavior_directorとのcmd_vel競合回避を
  再利用するため)。
- 記録(時刻・走行距離または回転角度のサンプリング)は、このスクリプト側が
  `/api/state`を`--rate`Hzで自前でポーリングして行う(ダッシュボード側に
  新しい記録機構を追加していない=既存コードへの変更を最小限にする方針)。
  回転角度は`/api/state`のodom.yaw_deg(180°を超える回転も追跡できるよう
  毎サンプルの差分を積算)から求める。
- `--sim`はロボット・ネットワーク無しで動く。1次遅れ/2次系の疑似応答を
  ローカルで生成し、判定・プロット処理だけを試せる。`--direction=rotate_left/
  rotate_right`でも使える(疑似応答生成自体が単位に依存しない汎用関数のため。
  --targetを度として生成するだけで、ラベル表記だけ度になる)。
- 安全面: 目標距離に上限(既定3.0m、--max-targetで変更可)、目標回転角度に
  上限(既定360.0度、--max-target-degで変更可)を設けて超過をエラーにする。
  ダッシュボード側(PidGainsController.MAX_TEST_DISTANCE_M/
  MAX_TEST_ROTATE_DEG)にも同じ上限を実装済みなので、こちらは第一の防波堤。
  Ctrl-Cや例外発生時は、実走行中(POST /api/drive_test成功後)であれば必ず
  /api/soft_estopでソフト緊急停止をかけてから終了する(既存のsoft_estopは
  run_drive_test()のワーカーループが毎ループ確認しており、即座に停止する)。

使い方:
    # ロボット無しで判定・プロット処理だけ試す(距離)
    python3 step_response_test.py --sim --target 1.0

    # ロボット無しで判定・プロット処理だけ試す(回転角度)
    python3 step_response_test.py --sim --direction rotate_left --target 90.0

    # 実機(ダッシュボードがhttp://<host>:8090で動いている前提、距離)
    python3 step_response_test.py --target 1.0 --api-base http://169.254.135.10:8090 \\
        --username TL --password <パスワード(TL)>

    # 実機・回転(90度左回転)
    python3 step_response_test.py --direction rotate_left --target 90.0 \\
        --api-base http://169.254.135.10:8090 --username TL --password <パスワード(TL)>

過去の結果はダッシュボードの「ステップ応答テスト」ページ
(/step_response、stage1/stage2/full_dashboard_node.py共通)から一覧・比較できる。
"""
from __future__ import annotations

import argparse
import csv
import json
import logging
import math
import os
import sys
import time
from datetime import datetime, timezone
from typing import List, Optional, Tuple

from analysis import (
    StepResponseMetrics,
    compute_step_response_metrics,
    simulate_step_response,
)

logger = logging.getLogger("step_response_test")

DEFAULT_OUT_DIR = "~/.nokosahen_step_response_tests"
DEFAULT_MAX_TARGET_M = 3.0
# 回転(rotate_left/rotate_right)版の安全上限。ダッシュボード側
# (PidGainsController.MAX_TEST_ROTATE_DEG)と揃えてある。その場回転は1周
# (360°)しても部屋の外へ飛び出すような危険が無く、PID調整用のステップ応答
# テストとしても1周分の応答を見たいケースは正当なので、360°を上限とする。
DEFAULT_MAX_TARGET_DEG = 360.0
ROTATE_DIRECTIONS = ("rotate_left", "rotate_right")
# サーバー側(PidGainsController)のMAX_TEST_DURATION_SECと揃えておき、
# クランプ前にこちらで明らかに超過した値を送らないようにする。
SERVER_MAX_TEST_DURATION_SEC = 15.0


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="走行距離/回転角度のステップ応答テスト(ゲイン調整の前後比較用)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "--target", type=float, default=1.0,
        help="目標値(ステップ入力)。--directionがforward/backwardなら走行距離[m]、"
        "rotate_left/rotate_rightなら回転角度[度]として扱う。",
    )
    p.add_argument("--duration", type=float, default=8.0, help="記録時間[s](目標到達後の整定を見られる長さ)")
    p.add_argument("--rate", type=float, default=10.0, help="サンプリング周期[Hz]")
    p.add_argument("--os-threshold", type=float, default=5.0, help="オーバーシュート判定のしきい値[%%]")
    p.add_argument("--tol", type=float, default=2.0, help="整定判定の許容幅[%%](±tol%%)")
    p.add_argument("--out", type=str, default=DEFAULT_OUT_DIR, help="出力ディレクトリ")
    p.add_argument(
        "--sim", action="store_true",
        help="ロボットを動かさず、疑似応答で判定・プロット処理だけを試す。"
        "--direction=rotate_left/rotate_rightでも使える(疑似応答の生成自体は"
        "単位に依存しない汎用の1次/2次系カーブなので、--targetを度として"
        "生成するだけ。ラベル・保存されるCSV/JSONの単位表記のみ度になる)。",
    )
    p.add_argument(
        "--direction", choices=["forward", "backward", "rotate_left", "rotate_right"], default="forward",
        help="走行方向。rotate_left/rotate_rightの場合、--target/--max-target-degの単位は[度]になる(forward/backwardは[m])。",
    )
    p.add_argument("--speed", type=float, default=0.4, help="実走行時の目標速度[m/s](forward/backward用。=target/speedで走行時間を決める。--simでは無視)")
    p.add_argument(
        "--angular-speed", type=float, default=45.0,
        help="実走行時の目標角速度[度/s](rotate_left/rotate_right用。=target/angular_speedで回転時間を決める。--simでは無視)",
    )
    p.add_argument("--max-target", type=float, default=DEFAULT_MAX_TARGET_M, help="目標距離の安全上限[m](forward/backward用)。超過はエラーで終了")
    p.add_argument(
        "--max-target-deg", type=float, default=DEFAULT_MAX_TARGET_DEG,
        help="目標回転角度の安全上限[度](rotate_left/rotate_right用)。超過はエラーで終了",
    )
    p.add_argument("--api-base", type=str, default="http://127.0.0.1:8090", help="ダッシュボードのURL(実走行時のみ使用)")
    p.add_argument("--username", type=str, default="TL", help="ダッシュボードのBasic認証ユーザー名")
    p.add_argument("--password", type=str, default="", help="ダッシュボードのBasic認証パスワード")
    p.add_argument("--sim-kind", choices=["first_order", "second_order"], default="second_order", help="--sim時の疑似応答の種類")
    p.add_argument("--sim-tau", type=float, default=1.2, help="--sim --sim-kind first_order の時定数[s]")
    p.add_argument("--sim-zeta", type=float, default=0.5, help="--sim --sim-kind second_order の減衰係数(1未満でオーバーシュートする)")
    p.add_argument("--sim-wn", type=float, default=2.5, help="--sim --sim-kind second_order の固有角周波数[rad/s]")
    p.add_argument("--sim-noise", type=float, default=0.01, help="--sim時に加える観測ノイズの標準偏差[m](0で無し)")
    p.add_argument("--show", action="store_true", help="プロットを画面にも表示する(ヘッドレス環境では自動でスキップされる)")
    p.add_argument("-v", "--verbose", action="store_true", help="詳細ログを表示")
    return p


def _validate_target(direction: str, target: float, max_target_m: float, max_target_deg: float) -> None:
    is_rotate = direction in ROTATE_DIRECTIONS
    unit = "度" if is_rotate else "m"
    max_value = max_target_deg if is_rotate else max_target_m
    max_flag = "--max-target-deg" if is_rotate else "--max-target"
    label = "回転角度" if is_rotate else "距離"
    if target <= 0:
        raise SystemExit(f"エラー: --target は正の値にしてください(指定値: {target})")
    if abs(target) > max_value:
        raise SystemExit(
            f"エラー: 目標{label} {target}{unit} が安全上限 {max_value}{unit} を超えています。"
            f"意図的に上限を上げる場合は {max_flag} で指定してください。"
        )


def _login(session: "requests.Session", base_url: str, username: str, password: str, timeout: float = 10.0) -> None:
    """ダッシュボードにログインする。

    【注意】ダッシュボードの認証はHTTP Basic認証ではなく、`/api/login`への
    POSTでセッションCookieを受け取る独自方式(dashboard_common.py
    `_handle_login_post`参照)。`requests.get(..., auth=(user, pass))`の
    ようなBasic認証ヘッダーでは通らないので、必ずこの関数でログインしてから
    同じ`session`で以降のリクエストを送ること。
    """
    resp = session.post(
        f"{base_url}/api/login", json={"username": username, "password": password}, timeout=timeout
    )
    if resp.status_code != 200:
        try:
            detail = resp.json().get("error")
        except Exception:  # noqa: BLE001 - レスポンス本文がJSONでない場合のフォールバック表示のみ
            detail = resp.text
        raise RuntimeError(f"ログインに失敗しました(HTTP {resp.status_code}): {detail}")


def _fetch_state(session: "requests.Session", base_url: str, timeout: float = 5.0) -> dict:
    resp = session.get(f"{base_url}/api/state", timeout=timeout)
    resp.raise_for_status()
    return resp.json()


def _start_drive_test(
    session: "requests.Session", base_url: str, direction: str, target: float, duration_s: float, timeout: float = 10.0
) -> None:
    if direction in ROTATE_DIRECTIONS:
        payload = {"mode": direction, "rotate_deg": target, "duration_s": duration_s}
    else:
        payload = {"mode": direction, "distance_m": target, "duration_s": duration_s}
    resp = session.post(f"{base_url}/api/drive_test", json=payload, timeout=timeout)
    if resp.status_code != 200:
        try:
            detail = resp.json().get("message") or resp.json().get("error")
        except Exception:  # noqa: BLE001 - レスポンス本文がJSONでない場合のフォールバック表示のみ
            detail = resp.text
        raise RuntimeError(f"走行テストを開始できませんでした(HTTP {resp.status_code}): {detail}")


def _engage_soft_estop(session: "requests.Session", base_url: str) -> None:
    try:
        session.post(f"{base_url}/api/soft_estop", json={"engaged": True}, timeout=5.0)
        logger.warning("安全のためソフト緊急停止をかけました。解除はダッシュボードから行ってください。")
    except Exception as exc:  # noqa: BLE001 - 停止指令自体はベストエフォート、失敗してもここで握りつぶして終了処理を続ける
        logger.error("ソフト緊急停止の送信に失敗しました: %s", exc)


def run_real_test(args: argparse.Namespace) -> Tuple[List[float], List[float]]:
    import requests

    base_url = args.api_base.rstrip("/")
    session = requests.Session()
    is_rotate = args.direction in ROTATE_DIRECTIONS

    logger.info("ダッシュボード(%s)へログインします...", base_url)
    _login(session, base_url, args.username, args.password)

    logger.info("開始位置を取得します...")
    state0 = _fetch_state(session, base_url)
    odom0 = state0.get("odom")
    # 回転量の追跡状態(is_rotate時のみ使う)。直前サンプルとの差分(180°を
    # 大きく下回ることが分かっている)を積算していくことで、target(度)が
    # 180°を超えても(最大--max-target-deg=360°まで)正しく追跡できる。
    # 開始角度との単純な差分だとラップアラウンド(270°回転しても-90°に
    # 見える等)で壊れるため、この積算方式にしている
    # (dashboard_common.py PidGainsController.run_step_response_test()の
    # _make_rotation_tracker()と同じ考え方)。
    prev_yaw_deg: Optional[float] = None
    cum_deg = 0.0
    x0 = y0 = 0.0
    if is_rotate:
        if not odom0 or "yaw_deg" not in odom0:
            raise SystemExit(
                "エラー: /api/state に odom.yaw_deg がありません。"
                "wheel_odomノード(/odom)が動いていない可能性があります"
                "(またはダッシュボードが古く、yaw_degを未公開の可能性があります)。"
            )
        prev_yaw_deg = float(odom0["yaw_deg"])
    else:
        if not odom0 or "x" not in odom0 or "y" not in odom0:
            raise SystemExit(
                "エラー: /api/state に odom.x/odom.y がありません。"
                "wheel_odomノード(/odom)が動いていない可能性があります。"
            )
        x0, y0 = float(odom0["x"]), float(odom0["y"])

    if is_rotate:
        drive_duration_s = min(
            SERVER_MAX_TEST_DURATION_SEC - 0.5, max(0.5, args.target / max(0.01, args.angular_speed))
        )
    else:
        drive_duration_s = min(SERVER_MAX_TEST_DURATION_SEC - 0.5, max(0.5, args.target / max(0.01, args.speed)))
    if drive_duration_s + 1.0 > args.duration:
        logger.warning(
            "--duration(%.1fs)が走行想定時間(約%.1fs)に対して短いため、"
            "整定の様子が十分に記録できない可能性があります。",
            args.duration, drive_duration_s,
        )

    started = False
    try:
        logger.info(
            "走行テストを開始します: %s %.2f%s(目安速度%.2f%s、走行時間上限%.1fs)",
            args.direction, args.target, "度" if is_rotate else "m",
            args.angular_speed if is_rotate else args.speed, "度/s" if is_rotate else "m/s",
            drive_duration_s,
        )
        _start_drive_test(session, base_url, args.direction, args.target, drive_duration_s)
        started = True

        times: List[float] = []
        responses: List[float] = []
        t_start = time.monotonic()
        period = 1.0 / args.rate
        n_samples = max(1, int(round(args.duration * args.rate)))
        for i in range(n_samples + 1):
            target_wall_time = t_start + i * period
            now = time.monotonic()
            if target_wall_time > now:
                time.sleep(target_wall_time - now)
            try:
                s = _fetch_state(session, base_url, timeout=2.0)
                odom = s.get("odom") or {}
                if is_rotate:
                    yaw = float(odom.get("yaw_deg", prev_yaw_deg))
                    delta = (yaw - prev_yaw_deg + 180.0) % 360.0 - 180.0
                    cum_deg += delta
                    prev_yaw_deg = yaw
                    r = abs(cum_deg)
                else:
                    x, y = float(odom.get("x", x0)), float(odom.get("y", y0))
                    r = math.hypot(x - x0, y - y0)
            except Exception as exc:  # noqa: BLE001 - 1回の取得失敗で記録全体を止めない(欠測として前回値を維持)
                logger.warning("サンプル取得に失敗しました(%s回目): %s", i, exc)
                r = responses[-1] if responses else 0.0
            times.append(time.monotonic() - t_start)
            responses.append(r)
        return times, responses
    except KeyboardInterrupt:
        logger.warning("Ctrl-Cを検知しました。")
        if started:
            _engage_soft_estop(session, base_url)
        raise
    except Exception:
        if started:
            _engage_soft_estop(session, base_url)
        raise


def save_results(
    out_dir: str,
    args: argparse.Namespace,
    times: List[float],
    responses: List[float],
    metrics: StepResponseMetrics,
) -> str:
    is_rotate = args.direction in ROTATE_DIRECTIONS
    executed_at = datetime.now(timezone.utc).astimezone()
    run_name = "{ts}_{mode}_{target:.2f}{unit}".format(
        ts=executed_at.strftime("%Y%m%d_%H%M%S"),
        mode="sim" if args.sim else args.direction,
        target=args.target,
        unit="deg" if is_rotate else "m",
    )
    run_dir = os.path.join(os.path.expanduser(out_dir), run_name)
    os.makedirs(run_dir, exist_ok=True)

    csv_path = os.path.join(run_dir, "data.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["t_s", "rotation_deg" if is_rotate else "distance_m"])
        for t, d in zip(times, responses):
            writer.writerow([f"{t:.4f}", f"{d:.5f}"])

    params = {
        # 【注意】is_rotate(direction=rotate_left/rotate_right)の場合、
        # target_mという歴史的なフィールド名のまま角度[度]の値を入れている
        # (ダッシュボード側result.jsonとスキーマを揃えるため)。単位の判別は
        # 必ずdirectionを見て行うこと。
        "target_m": args.target,
        "duration_s": args.duration,
        "rate_hz": args.rate,
        "os_threshold_pct": args.os_threshold,
        "tol_pct": args.tol,
        "sim": bool(args.sim),
        "direction": args.direction,
    }
    if args.sim:
        params.update(
            {
                "sim_kind": args.sim_kind,
                "sim_tau": args.sim_tau,
                "sim_zeta": args.sim_zeta,
                "sim_wn": args.sim_wn,
                "sim_noise": args.sim_noise,
            }
        )
    elif is_rotate:
        params["angular_speed_dps"] = args.angular_speed
    else:
        params["speed_mps"] = args.speed

    result = {
        "executed_at": executed_at.isoformat(),
        "run_name": run_name,
        "params": params,
        "times_s": times,
        "distances_m": responses,
        "metrics": metrics.to_dict(),
    }
    json_path = os.path.join(run_dir, "result.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)

    png_path = os.path.join(run_dir, "plot.png")
    save_plot(png_path, args, times, responses, metrics, show=args.show)

    return run_dir


def save_plot(
    png_path: str,
    args: argparse.Namespace,
    times: List[float],
    responses: List[float],
    metrics: StepResponseMetrics,
    show: bool = False,
) -> None:
    import matplotlib

    used_interactive_backend = False
    if show:
        try:
            matplotlib.use("TkAgg")
            used_interactive_backend = True
        except Exception:  # noqa: BLE001 - 表示できる環境が無ければ黙ってheadless保存だけ行う
            matplotlib.use("Agg")
    else:
        matplotlib.use("Agg")

    import matplotlib.pyplot as plt

    # 日本語ラベルが文字化け(豆腐/警告)しないよう、見つかった範囲でCJK対応
    # フォントを使う。無ければDejaVu Sansにフォールバックし、警告が出るだけで
    # 処理自体は継続する(ヘッドレス環境で必ず動くことを優先する)。
    matplotlib.rcParams["font.sans-serif"] = [
        "Yu Gothic", "Meiryo", "MS Gothic",  # Windows
        "Noto Sans CJK JP", "IPAexGothic", "TakaoGothic", "VL Gothic",  # Linux
        "DejaVu Sans",
    ]
    matplotlib.rcParams["axes.unicode_minus"] = False

    is_rotate = args.direction in ROTATE_DIRECTIONS
    unit = "度" if is_rotate else "m"
    label = "回転角度" if is_rotate else "走行距離"

    target_m = metrics.target_m
    tol_abs = target_m * (metrics.tol_pct / 100.0)

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(times, responses, label=f"{label} d(t)", color="#1f6f4a", linewidth=1.6)
    ax.axhline(target_m, color="#333333", linestyle="--", linewidth=1.2, label=f"目標 {target_m:.2f} {unit}")
    if target_m > 0:
        ax.axhspan(target_m - tol_abs, target_m + tol_abs, color="#1f6f4a", alpha=0.12, label=f"許容帯 ±{metrics.tol_pct:.1f}%")
    if metrics.peak_time_s is not None:
        ax.plot([metrics.peak_time_s], [metrics.d_max], marker="o", color="#c0392b", zorder=5, label=f"ピーク {metrics.d_max:.3f} {unit}")

    title = f"判定: {metrics.judgement}  /  オーバーシュート {metrics.overshoot_pct:.1f}% ({metrics.overshoot_m:.3f} {unit})"
    ax.set_title(title, color=("#c0392b" if metrics.judgement == "NG" else "#1f6f4a"), fontsize=13)
    ax.set_xlabel("時間 [s]")
    ax.set_ylabel(f"{label} [{unit}]")
    ax.legend(loc="lower right", fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    if used_interactive_backend:
        try:
            plt.show()
        except Exception:  # noqa: BLE001 - 表示に失敗してもPNGは既に保存済みなので継続する
            pass
    plt.close(fig)


def print_report(direction: str, metrics: StepResponseMetrics) -> None:
    is_rotate = direction in ROTATE_DIRECTIONS
    unit = "度" if is_rotate else "m"
    label = "回転角度" if is_rotate else "距離"

    def fmt(v: Optional[float], unit: str = "s") -> str:
        return f"{v:.3f}{unit}" if v is not None else "N/A"

    print("=" * 52)
    print(f"判定: {metrics.judgement}  (しきい値 {metrics.os_threshold_pct:.1f}%)")
    print(f"目標{label}:       {metrics.target_m:.3f} {unit}")
    print(f"最大到達{label}:   {metrics.d_max:.3f} {unit}")
    print(f"オーバーシュート: {metrics.overshoot_m:.3f} {unit} ({metrics.overshoot_pct:.2f} %)")
    print(f"ピーク時刻:     {fmt(metrics.peak_time_s)}")
    print(f"立ち上がり時間(10-90%): {fmt(metrics.rise_time_s)}")
    print(f"整定時間(±{metrics.tol_pct:.1f}%): {fmt(metrics.settling_time_s)}")
    ss = metrics.steady_state_error_m
    print(f"定常偏差:       {f'{ss:.3f} {unit}' if ss is not None else 'N/A'}")
    print("=" * 52)


def main() -> int:
    # Windows端末(既定cp932)でも日本語の標準出力が文字化けしないようにする。
    # 非対話的にリダイレクトされている等でreconfigureが使えない場合は無視する。
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except Exception:  # noqa: BLE001 - 出力エンコーディング調整の失敗は致命的でないため継続する
            pass

    args = build_arg_parser().parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(levelname)s: %(message)s")

    _validate_target(args.direction, args.target, args.max_target, args.max_target_deg)
    if args.rate <= 0:
        raise SystemExit("エラー: --rate は正の値にしてください")
    if args.duration <= 0:
        raise SystemExit("エラー: --duration は正の値にしてください")

    if args.sim:
        logger.info("--sim モード: ロボットを動かさず疑似応答を生成します。")
        # simulate_step_response()/compute_step_response_metrics()は単位に
        # 依存しない汎用関数なので、--direction=rotate_left/rotate_rightでも
        # そのまま使える(target_mという引数名だがtargetを度として渡すだけ)。
        times, responses = simulate_step_response(
            target_m=args.target,
            duration_s=args.duration,
            rate_hz=args.rate,
            kind=args.sim_kind,
            tau_s=args.sim_tau,
            zeta=args.sim_zeta,
            wn=args.sim_wn,
            noise_std_m=args.sim_noise,
        )
    else:
        times, responses = run_real_test(args)

    metrics = compute_step_response_metrics(
        times, responses, target_m=args.target, os_threshold_pct=args.os_threshold, tol_pct=args.tol
    )
    print_report(args.direction, metrics)
    run_dir = save_results(args.out, args, times, responses, metrics)
    print(f"保存先: {run_dir}")

    return 0 if metrics.judgement == "OK" else 1


if __name__ == "__main__":
    sys.exit(main())
