"""「ロボットに現在位置を教える」(初期位置・位置合わせ)機能の純粋関数部分(ROS非依存)。

【2026-10-07追加】保存した地図を読み込むとlocalizationモード(map_server + AMCL)になるが、
AMCLは「今ロボットが地図上のどこにいるか」の初期値(/initialpose)が無いと位置が定まらない。
ダッシュボードの地図上でクリック+ドラッグして位置と向きを教える(RVizの「2D Pose Estimate」相当)。
間違った初期位置のままNav2の目的地を出すと見当違いの場所へ走るので、AMCLの共分散から
「位置の確かさ」を良好/要確認/未確定に分類して画面に出す(classify_localization_quality)。

ここにはROSに依存しない検証・変換だけを置き、テストしやすくしている。
実際の送信(/initialpose publish、/reinitialize_global_localization呼び出し、/amcl_pose購読)は
dashboard_common.LocalizationAssistController。
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Sequence, Tuple

# RVizの「2D Pose Estimate」既定の共分散(6x6行優先): x/y 0.25 m^2, yaw 0.0685 rad^2
INITIAL_POSE_VAR_XY = 0.25
INITIAL_POSE_VAR_YAW = 0.06853891945200942
POSE_ABS_LIMIT_M = 1000.0

# 位置の確かさのしきい値(AMCLのパーティクル分布の標準偏差)
QUALITY_GOOD_POS_STD_M = 0.3
QUALITY_GOOD_YAW_STD_RAD = 0.2
QUALITY_CHECK_POS_STD_M = 1.0
QUALITY_CHECK_YAW_STD_RAD = 0.7

QUALITY_LABELS = {
    "good": "良好",
    "check": "要確認",
    "unknown": "未確定",
    "waiting": "位置合わせ待ち",
    "inactive": "-",
}


class PoseRequestError(ValueError):
    """/api/initial_pose のリクエストが不正(HTTP 400)。"""


def initial_pose_covariance() -> list:
    cov = [0.0] * 36
    cov[0] = INITIAL_POSE_VAR_XY
    cov[7] = INITIAL_POSE_VAR_XY
    cov[35] = INITIAL_POSE_VAR_YAW
    return cov


def yaw_to_quaternion(yaw_rad: float) -> Tuple[float, float, float, float]:
    """平面内の回転(z軸まわり)を(x, y, z, w)のクォータニオンへ。"""
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _num(data: Dict[str, Any], key: str) -> float:
    if key not in data:
        raise PoseRequestError(f"{key} は必須の数値です")
    v = data[key]
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise PoseRequestError(f"{key} は数値で指定してください")
    v = float(v)
    if not math.isfinite(v):
        raise PoseRequestError(f"{key} に有限の数値を指定してください(NaN/無限大は不可)")
    return v


def parse_initial_pose_request(data: Any) -> Tuple[float, float, float]:
    """{x, y, yaw_deg}(地図座標系のm・度)を検証して(x, y, yaw_rad)を返す。不正なら PoseRequestError。"""
    if not isinstance(data, dict):
        raise PoseRequestError("リクエストが不正です(x, y, yaw_deg が必要)")
    x = _num(data, "x")
    y = _num(data, "y")
    yaw_deg = _num(data, "yaw_deg")
    if abs(x) >= POSE_ABS_LIMIT_M or abs(y) >= POSE_ABS_LIMIT_M:
        raise PoseRequestError(f"x/y が大きすぎます(絶対値 {POSE_ABS_LIMIT_M:.0f} m 未満)")
    if abs(yaw_deg) > 3600.0:
        raise PoseRequestError("yaw_deg が大きすぎます")
    # (-180, 180] へ正規化
    yaw_deg = math.fmod(yaw_deg, 360.0)
    if yaw_deg <= -180.0:
        yaw_deg += 360.0
    elif yaw_deg > 180.0:
        yaw_deg -= 360.0
    return x, y, math.radians(yaw_deg)


def point_in_map_bounds(info: Any, x: float, y: float) -> Optional[bool]:
    """点(x,y)が地図グリッドの範囲内か。地図情報が無い/不正ならNone(判定不能)。
    originの回転は無視(保存地図は通常yaw=0)。"""
    try:
        res = float(info.resolution)
        w = int(info.width)
        h = int(info.height)
        ox = float(info.origin.position.x)
        oy = float(info.origin.position.y)
    except Exception:  # noqa: BLE001
        return None
    if not (res > 0 and w > 0 and h > 0):
        return None
    return ox <= x <= ox + w * res and oy <= y <= oy + h * res


def classify_localization_quality(covariance: Optional[Sequence[float]]) -> Dict[str, Any]:
    """/amcl_poseの共分散(6x6行優先)から位置の確かさを分類する。

    pos_std = sqrt(var_x + var_y)[m]、yaw_std = sqrt(var_yaw)[rad]。
    良好: pos<0.3m かつ yaw<0.2rad / 要確認: pos<1.0m かつ yaw<0.7rad / それ以外・非有限: 未確定。
    covarianceがNone(まだ/amcl_poseが来ていない)は「位置合わせ待ち」。
    """
    if covariance is None:
        return {"state": "waiting", "label": QUALITY_LABELS["waiting"], "pos_std_m": None, "yaw_std_rad": None}
    try:
        vx, vy, vyaw = float(covariance[0]), float(covariance[7]), float(covariance[35])
    except Exception:  # noqa: BLE001
        return {"state": "unknown", "label": QUALITY_LABELS["unknown"], "pos_std_m": None, "yaw_std_rad": None}
    if not all(math.isfinite(v) for v in (vx, vy, vyaw)):
        return {"state": "unknown", "label": QUALITY_LABELS["unknown"], "pos_std_m": None, "yaw_std_rad": None}
    pos_std = math.sqrt(max(vx, 0.0) + max(vy, 0.0))
    yaw_std = math.sqrt(max(vyaw, 0.0))
    if pos_std < QUALITY_GOOD_POS_STD_M and yaw_std < QUALITY_GOOD_YAW_STD_RAD:
        state = "good"
    elif pos_std < QUALITY_CHECK_POS_STD_M and yaw_std < QUALITY_CHECK_YAW_STD_RAD:
        state = "check"
    else:
        state = "unknown"
    return {"state": state, "label": QUALITY_LABELS[state], "pos_std_m": pos_std, "yaw_std_rad": yaw_std}
