"""自律走行(Nav2)の最高直進速度を「1つの設定値」にまとめ、起動時にも実行時にも
同じ変換で全関連パラメータへ反映するための部品(2026-10-07追加)。

背景: 速度に関係するNav2パラメータ(DWBのmax_vel_x/max_speed_xy、velocity_smootherの
max_velocity、加減速、DWBのsim_time、collision_monitorのtime_before_collision、
ローカルコストマップの大きさ)は互いに整合していないと危険(例えば速度だけ上げて
先読みが足りないと、止まれずに障害物へ近付く)。そこで速度v[m/s]だけを入力とする
純粋関数`derive_nav_speed_values()`/`live_parameters()`/`apply_nav_speed_to_params()`に
全ての派生値を集約し、

- 起動時: mapping_mode/localization_mode.launch.pyがnav2_params_*.yamlを読み込み、
  `apply_nav_speed_to_params()`で書き換えた一時コピーをnav2_bringupへ渡す
- 実行時: ダッシュボードの`NavSpeedController.apply()`が、同じ値を実行中のノードへ
  パラメータサービス(SetParameters)で流し込む

の両方が必ず同じ値になるようにしてある。

安全上の注意: ここはパラメータを書くだけで、/cmd_velもNav2のゴールも一切publishしない。
緊急停止/ソフト緊急停止のキャンセル経路にも触れない。rclpyはRclpyParamClientの中で
遅延importするので、ROS無しのPCでもこのモジュールの純粋部分とテストは動く。

自動走行の速度は2つの別々のつまみがある点に注意:
- Nav2の最高速度(このモジュール): Nav2が出すゴール追従(探索マッピング・地図クリック
  移動・ルート巡回など)の速度。既定0.5 m/s、上限1.1 m/s。
- behavior_directorの自動速度(AutoSpeedController、auto_linear_mps既定0.75):
  CROWD_SCAN/接近時にdirector自身が出す速度。本モジュールとは無関係(変更しない)。
"""
from __future__ import annotations

import copy
import json
import math
import os
import tempfile
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

NAV_SPEED_MIN = 0.1
NAV_SPEED_MAX = 1.1  # dashboard_common.MAX_DRIVE_LINEAR_MPSと同じ(ファームのクランプ約1.23に余裕)
# 【2026-10-10】0.5 -> 0.7(実機の感想「指示を出す速度をもっと早くしてok」)。停止距離の検算はdocs/nav2_clearance_and_recovery.md。
NAV_SPEED_DEFAULT = 0.7
# これを超える速度へ上げるときは、POSTに {"confirm_high": true} が必要。
NAV_SPEED_CONFIRM_ABOVE = 0.6

# 手動運転のファームウェア側ランプ(加速0.8、減速2.0 m/s^2)を超える加減速は要求しない。
MAX_ACCEL = 0.8
MAX_DECEL = 2.0
MIN_ACCEL = 0.3
# v=0.22(従来値)で加速0.45になる比率(到達時間 約0.49秒を保つ)
ACCEL_PER_MPS = 0.45 / 0.22

# 【2026-10-10】後退 -0.08 -> -0.15(BTのBackUp backup_speed 0.15を通すため。それ以上は上げない。後方は衝突監視の後退用ポリゴンが見る)。
# 【2026-10-23】後退(障害物があるときの下がって再計算=BackUp)の速さを、通常の走行(NAV_SPEED_DEFAULT)と同じにした(実機の感想「下がる速さを通常時と同じでいい、もっと早くても」)。
# 後退は速くなると停止距離が伸びるので、衝突監視の後退用の停止領域(後方)を、停止距離から導いて広げる(derive_clearance_values参照)。
BACKUP_SPEED_MPS = NAV_SPEED_DEFAULT
REVERSE_SPEED = -BACKUP_SPEED_MPS  # velocity_smoother.min_velocity[0]。BTのBackUp backup_speedと同じ値
BACKUP_STOP_EXTRA_M = 0.10  # 後退の停止距離の見積りに足す余裕
# 【2026-10-10】旋回を速く(実機の感想「回るときにこんなにゆっくりじゃなくて」): 0.85 -> 1.8 rad/s、加速1.2 -> 3.0、減速 -> 4.0。
# 直進速度vとは独立(vを変えても旋回は変えない)。根拠・車輪クランプとの関係はdocs/nav2_clearance_and_recovery.md。
# 注意: ファームの角加速ランプ(RAMP_ACCEL_ANGULAR_RPS2=1.6、減速4.0)が実機の実効上限。3.0へ上げても実際の立ち上がりは1.6のまま
# (ファームを書き換えるまで)。減速4.0はファームの減速ランプと同じ値。
BASE_THETA_VEL = 1.8
BASE_THETA_ACC = 3.0
BASE_THETA_DEC = 4.0
# ファームの車輪速クランプ(約1.23 m/s)と輪距(WHEEL_BASE 0.3883 m): 外側の車輪速 = |v| + |w| * 輪距/2
FIRMWARE_WHEEL_CLAMP_MPS = 1.23
WHEEL_HALF_BASE_M = 0.3883 / 2.0

# 【2026-10-10】手動走行(WASD/D-pad)の設定上限を引き上げる(実機の感想「値は今のままでいいが、最大値をもう少し上げられるように。回転も」)。
# 既定値(nokosahen.yamlのlinear_speed_mps 1.0 / angular_speed_rps 1.5)・スライダー初期位置は変えない。上限だけを広げる。
# 手動の上限は自律側(NAV_SPEED_MAX=1.1、director自動、Nav2)とは別の値で、自律の安全設計(停止距離・衝突監視)は変えない。
# 上限の根拠(車輪速 = |v| + |w| * 輪距/2 がファームの頭打ち約1.23 m/sに収まる範囲。docs/autonomous_speed.md):
# - 直進 1.2 m/s: 旧1.1。クランプ1.23の少し下。
# - 旋回 4.0 rad/s: 旧3.0。その場旋回の外側車輪速は 4.0*0.194=0.78 m/s(クランプ1.23の約63%。理論上限は 1.23/0.194=6.3 rad/s)。
#   衝突監視の停止領域のtheta範囲(±4.0)ともそろう。直進と旋回を同時に最大にすると外側が頭打ちになり、弧が緩くなるだけ(危険ではない)。
# ファームの角加速ランプ(RAMP_ACCEL_ANGULAR_RPS2=1.6)が立ち上がりの実効上限で、設定上限を上げても立ち上がりは速くならない。
MANUAL_LINEAR_MIN_MPS = 0.01
MANUAL_ANGULAR_MIN_RADPS = 0.01
MANUAL_LINEAR_MAX_MPS = 1.2
MANUAL_ANGULAR_MAX_RADPS = 4.0
# 旧上限(自律側の上限=director自動・Nav2はこのまま。AutoSpeedControllerもこの値)
AUTO_LINEAR_MAX_MPS = 1.1
AUTO_ANGULAR_MAX_RADPS = 3.0
# 手動でこれを超える値へ「変える」ときは、POSTに {"confirm_high": true} が必要(新しく開いた高い領域だけ。従来の範囲の運用は確認なし)。
MANUAL_LINEAR_CONFIRM_ABOVE_MPS = 1.1
MANUAL_ANGULAR_CONFIRM_ABOVE_RADPS = 3.0


def manual_speed_limits() -> Dict[str, float]:
    """手動走行の上限・確認しきい値(画面とAPIの応答`limits`が使う、唯一の出所)。"""
    return {
        "manual_linear_min": MANUAL_LINEAR_MIN_MPS,
        "manual_linear_max": MANUAL_LINEAR_MAX_MPS,
        "manual_angular_min": MANUAL_ANGULAR_MIN_RADPS,
        "manual_angular_max": MANUAL_ANGULAR_MAX_RADPS,
        "manual_linear_confirm_above": MANUAL_LINEAR_CONFIRM_ABOVE_MPS,
        "manual_angular_confirm_above": MANUAL_ANGULAR_CONFIRM_ABOVE_RADPS,
        "auto_linear_max": AUTO_LINEAR_MAX_MPS,
        "auto_angular_max": AUTO_ANGULAR_MAX_RADPS,
    }


def manual_confirm_needed(linear: Optional[float], angular: Optional[float],
                          cur_linear: float, cur_angular: float) -> List[str]:
    """手動の要求値のうち、確認(confirm_high)が必要な項目名。現在値と同じ(変えない)項目は含めない。"""
    out: List[str] = []
    if linear is not None and linear > MANUAL_LINEAR_CONFIRM_ABOVE_MPS and abs(linear - cur_linear) > 1e-9:
        out.append("manual_linear_mps")
    if angular is not None and angular > MANUAL_ANGULAR_CONFIRM_ABOVE_RADPS and abs(angular - cur_angular) > 1e-9:
        out.append("manual_angular_rps")
    return out

SIM_TIME_MIN = 1.4
SIM_TIME_MAX = 3.0
LOOKAHEAD_MARGIN_M = 0.25  # 先読み距離 >= 停止距離 + この余裕
# 【2026-10-10】0.28(旧robot_radius) -> 0.37(機体外接半径。footprint 0.26角の対角の半分=0.368)。
ROBOT_RADIUS_M = 0.37

DEFAULT_SAVE_PATH = "~/.nokosahen_nav_speed.json"
SAVED_MAX_AGE_SEC = 12 * 3600  # 古い記憶値は無視する(高速が何日も残らないように)

NODE_CONTROLLER = "controller_server"
NODE_SMOOTHER = "velocity_smoother"
NODE_COLLISION = "collision_monitor"
NODE_LOCAL_COSTMAP = "local_costmap/local_costmap"

NODE_LABELS = {
    NODE_CONTROLLER: "経路追従(controller_server)",
    NODE_SMOOTHER: "速度平滑化(velocity_smoother)",
    NODE_COLLISION: "衝突監視(collision_monitor)",
}


# ---------------------------------------------------------------- 純粋関数

def normalize_nav_speed(value: Any) -> float:
    """数値化して[0.1, 1.1]へ丸める。NaN/Inf/数値でない値はValueError。"""
    if isinstance(value, bool):
        raise ValueError("nav_max_speed_mps")
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise ValueError("nav_max_speed_mps")
    if not math.isfinite(v):
        raise ValueError("nav_max_speed_mps")
    return max(NAV_SPEED_MIN, min(NAV_SPEED_MAX, v))


def derive_nav_speed_values(speed: float) -> Dict[str, Any]:
    """速度v[m/s]から、整合した派生値を計算する(丸めはここで1回だけ行う)。"""
    v = normalize_nav_speed(speed)
    accel = round(min(MAX_ACCEL, max(MIN_ACCEL, v * ACCEL_PER_MPS)), 3)
    decel = round(min(MAX_DECEL, max(MIN_ACCEL, v * ACCEL_PER_MPS)), 3)
    stop_dist = v * v / (2.0 * decel)
    sim_time = round(
        max(SIM_TIME_MIN, min(SIM_TIME_MAX, (stop_dist + LOOKAHEAD_MARGIN_M) / v)), 2
    )
    vx_samples = 16 + int(round(8 * max(0.0, (v - 0.22) / (NAV_SPEED_MAX - 0.22))))
    time_before_collision = round(max(1.2, 0.8 + v), 2)
    costmap_size = max(3, int(math.ceil(2.0 * (v * sim_time + ROBOT_RADIUS_M))))
    return {
        "speed": round(v, 3),
        "accel": accel,
        "decel": decel,
        "sim_time": sim_time,
        "vx_samples": vx_samples,
        "time_before_collision": time_before_collision,
        "costmap_size": costmap_size,
        "stop_distance": round(stop_dist, 3),
    }


def live_parameters(
    speed: float,
    current_arrays: Optional[Dict[str, List[float]]] = None,
) -> Dict[str, Dict[str, Any]]:
    """ノード名 -> {パラメータ名: 値} 。実行時のSetParametersにそのまま使える形。

    current_arraysは現在のvelocity_smootherの配列(max_velocity/max_accel/max_decel)。
    指定があれば、直進(要素0)だけを置き換え、旋回などの要素は現在値のまま残す。
    後退速度(min_velocity)は一切変更しない。
    """
    d = derive_nav_speed_values(speed)
    cur = current_arrays or {}

    def arr(key: str, first: float, default_rest: List[float]) -> List[float]:
        base = list(cur.get(key) or [first] + default_rest)
        while len(base) < 3:
            base.append(0.0)
        base[0] = first
        return [float(x) for x in base[:3]]

    return {
        NODE_CONTROLLER: {
            "FollowPath.max_vel_x": d["speed"],
            "FollowPath.max_speed_xy": d["speed"],
            "FollowPath.acc_lim_x": d["accel"],
            "FollowPath.decel_lim_x": -d["decel"],
            "FollowPath.sim_time": d["sim_time"],
            "FollowPath.vx_samples": d["vx_samples"],
        },
        NODE_SMOOTHER: {
            "max_velocity": arr("max_velocity", d["speed"], [0.0, BASE_THETA_VEL]),
            "max_accel": arr("max_accel", d["accel"], [0.0, BASE_THETA_ACC]),
            "max_decel": arr("max_decel", -d["decel"], [0.0, -BASE_THETA_DEC]),
        },
        NODE_COLLISION: {
            "FootprintApproach.time_before_collision": d["time_before_collision"],
        },
    }


def _set_dotted(section: Dict[str, Any], dotted: str, value: Any) -> None:
    cur = section
    parts = dotted.split(".")
    for p in parts[:-1]:
        nxt = cur.get(p)
        if not isinstance(nxt, dict):
            nxt = {}
            cur[p] = nxt
        cur = nxt
    cur[parts[-1]] = value


def apply_nav_speed_to_params(params: Dict[str, Any], speed: float) -> Dict[str, Any]:
    """パース済みnav2_params(dict)を深くコピーし、速度関連の値を一括で設定して返す。
    入力は変更しない。その他のキーは全て保持する。存在しないノードの節は作らない。"""
    out = copy.deepcopy(params)

    def ros_params(node: str) -> Optional[Dict[str, Any]]:
        sec = out.get(node)
        if isinstance(sec, dict) and isinstance(sec.get("ros__parameters"), dict):
            return sec["ros__parameters"]
        return None

    smoother = ros_params(NODE_SMOOTHER)
    current_arrays = {}
    if smoother is not None:
        for k in ("max_velocity", "max_accel", "max_decel"):
            if isinstance(smoother.get(k), list):
                current_arrays[k] = smoother[k]
    live = live_parameters(speed, current_arrays)
    for node, values in live.items():
        rp = ros_params(node)
        if rp is None:
            continue
        for name, value in values.items():
            _set_dotted(rp, name, value)

    # ローカルコストマップ(起動時のみ。ローリングウィンドウの大きさは実行中に変えない)
    lc = out.get("local_costmap")
    if isinstance(lc, dict) and isinstance(lc.get("local_costmap"), dict):
        crp = lc["local_costmap"].get("ros__parameters")
        if isinstance(crp, dict):
            size = derive_nav_speed_values(speed)["costmap_size"]
            for k in ("width", "height"):
                crp[k] = max(int(crp.get(k, 0) or 0), size)
    return out


# ---------------------------------------------------------------- 目的地の到着精度(2026-10-07)
# 速度(nav_max_speed)とは独立した「別ブロック」。起動時にだけ、速度の書き換えと同じ一時コピーへ反映する
# (ダッシュボードの実行時の速度変更=live_parameters()は、これらを一切触らない。nav_max_speedの意味は不変)。
#
# 背景: 地図クリックの目的地に「なんとなく近い所で止まる」問題への対策。従来の許容誤差は
# xy 0.16-0.18 m / yaw 0.22-0.25 rad と大きく、Navfnの tolerance 0.5 m は、目的地が占有・インフレーション内
# だと「最も近い有効点」へ最大50 cmも寄せていた。
#
# 値の考え方(実機で振動/停止しないことを優先して、極端に小さくはしない):
# - general_goal_checker(SimpleGoalChecker, stateful): xy 0.08 m / yaw 0.15 rad。stateful=trueなので
#   一度xy許容内に入ったら位置は再判定せず、向きだけ合わせる(行ったり来たりしにくい)。
# - FollowPath(DWB)のxy_goal_tolerance: RotateToGoal/GoalAlign批評が「近い」と見なす半径。goal checkerと
#   同じ0.08 mにそろえる(ずれていると、チェッカーは未達なのに批評だけ回転を始めて止まる/振動する)。
# - 減速して寄せる挙動(RotateToGoal.slowing_factor=5、GoalDist/GoalAlign)は元のまま。
# - progress_checker.movement_time_allowance: 目的地手前の微調整で進みが小さくなっても「進展なし」で
#   打ち切られないよう 10 -> 15 秒。
#   【2026-10-08】15 -> 30秒。nav2_params_jetson.yamlはPoseProgressCheckerへ切り替え(その場旋回も進展と数える、
#   required_movement_angle 0.5 rad)。背後のゴールへの180度旋回は位置が動かず、負荷で遅くなるとSimple版の
#   15秒では足りなかった。raspi4側はSimpleProgressCheckerのままだが、許容秒数だけ同じ値が効く。
#   【2026-10-10】30 -> 6秒。旋回が1.8 rad/sになり180度でも約3秒なので30秒は不要になり、逆に「人が立ちふさがって
#   動けない」を早く検出して後退(BT: Wait 2秒+BackUp)へ進めたい。6秒 + Wait 2秒 = 約8秒で後退を始める。
# - Navfn planner tolerance 0.5 -> 0.2 m: 目的地が通れないセルに乗っていても最大20 cmしか寄せない。
#   (サーバーが先に最も近い空きセルへ寄せる(map_pins.snap_goal_to_free)ので、ここは保険)。
# 【限界】位置の精度はSLAM/AMCLの自己位置推定の誤差(通常数cm〜十数cm)より細かくはならない(docs/map_pins.md)。
GOAL_PRECISION: Dict[str, Dict[str, Any]] = {
    NODE_CONTROLLER: {
        "general_goal_checker.xy_goal_tolerance": 0.08,
        "general_goal_checker.yaw_goal_tolerance": 0.15,
        "FollowPath.xy_goal_tolerance": 0.08,
        "progress_checker.movement_time_allowance": 6.0,
    },
    "planner_server": {
        "GridBased.tolerance": 0.2,
    },
}


def apply_goal_precision_to_params(params: Dict[str, Any]) -> Dict[str, Any]:
    """パース済みnav2_params(dict)を深くコピーし、GOAL_PRECISIONの値を設定して返す。
    入力は変更しない。存在しないノードの節は作らない。速度に関する値には触れない。"""
    out = copy.deepcopy(params)
    for node, values in GOAL_PRECISION.items():
        sec = out.get(node)
        if isinstance(sec, dict) and isinstance(sec.get("ros__parameters"), dict):
            for name, value in values.items():
                _set_dotted(sec["ros__parameters"], name, value)
    return out


# ---------------------------------------------------------------- 人・障害物との距離(2026-10-10)
# 「機体の一番端から50cm以上離れて通る」「50cm以内に急に入られたら止まる」を、1つのclearance_mから導く。
# 値の根拠・停止距離の検算・実機確認手順は docs/nav2_clearance_and_recovery.md。
# nav2_params_jetson.yamlには同じ値が静的に書いてある(直接使うlaunch用)。test_nav2_clearance.pyが
# derive_clearance_values(CLEARANCE_M)とyamlの一致を見張るので、値を変えたらyamlも同じ値に直すこと。
# 【2026-10-22】0.50 -> 0.20。壁際(機体端から10〜20cm)で経路が作れず動かなくなるため。「走るときは機体端から壁まで20cmあればよく、
# それより内側に入ったら舵を強く切って離れる」= インフレーションを狭く急にする(端から20cmでコスト20、10cmで約90、接触で252)。
CLEARANCE_M = 0.20  # 機体の外形端から人・障害物まで確保したい最低距離
# 機体は約52cm四方(LiDAR中心)。四隅の柱の実測 0.33〜0.37 m(外接半径 0.368)に余裕を持たせた値。
BODY_HALF_M = 0.26
# 【2026-10-22】0.20 -> 0.10。壁沿いを機体端から20cmで通っても、側方の停止領域に壁が入って止まらないようにする。
# 【2026-10-23】0.10 -> 0.03。何回か同じ所を行き来すると、急に後退(BTの回復)が増えた。側方の判定が厳しく、地図の壁の太り・ずれや側方の点で、
# 前進が止まり→追従が進展なしで失敗→後退、となっていた。走行中は前後のLiDARの結果を優先し、左右は機体の幅の外側ぎりぎり(3cm)だけにする。
SIDE_MARGIN_M = 0.03        # 停止領域の側方の余白(前方・後方より狭い: 壁際の通過で止まり続けないため)
# 【2026-10-22】前方の停止領域は、インフレーションの余白(CLEARANCE_M=0.20)とは切り離して50cmのまま。0.6m/sの停止距離の見積りが約0.36mで、
# 前方20cmでは止まりきれないため(test_stopping_distance_fits_in_each_zoneが見張る)。
FRONT_STOP_CLEARANCE_M = 0.50
# 旋回/停止中の停止領域の半幅。旋回で掃く円(外接半径0.368)より外側(+5cm)。側方の余白(SIDE_MARGIN_M)とは別。
# 【2026-10-22】0.42 -> 0.38。壁が中心から約37〜42cmにあると、先回し旋回が衝突監視で0にされて回れず、背後の目的地に走れなかった。
ROTATE_HALF_M = 0.38
# 【2026-10-22】経路計画(グローバルコストマップ)だけ、機体を小さく(半幅0.26→0.21)見積もる。壁際の出発点が「通れない」扱いになって
# 『Failed to create plan』で動けなくなる件の対策。追従(DWB・ローカルコストマップ)・衝突監視・先読みは実寸(0.26)のまま守る。
PLAN_BODY_HALF_M = 0.21
REAR_MARGIN_M = 0.30        # 後退用の停止領域の最小(後退が速いときは、停止距離から導いた値のほうが大きい。BACKUP_SPEED_MPS)
FRONT_EXTRA_MID_M = 0.20    # 前進 0.60〜0.90 m/s の停止領域を前へ延ばす分(停止距離が伸びるため)
FRONT_EXTRA_FAST_M = 0.50   # 前進 0.90 m/s 以上
BODY_GAP_M = 0.04           # 停止領域は機体端のすぐ外から始める(機体・柱の写り込みを数えない)
STOP_MIN_POINTS = 3         # 足(細い物体)はLiDARで3点前後。1〜2点の外れ値で止まらない最小値
COST_AT_CLEARANCE = 20.0    # 端からclearance_mの所でのインフレーションのコスト(0〜252)。cost_scaling_factorの根拠
# 停止距離の見積り(検算用): 遅れ + 減速ランプ + 臨界減衰フィルタの尾
STOP_LATENCY_S = 0.2        # 検知(scan 10Hz) + 衝突監視の周期(約20Hz) + 通信
FIRMWARE_RAMP_OMEGA_N = 8.0  # config.h RAMP_OMEGA_N_LINEAR


def circumscribed_radius(body_half: float = BODY_HALF_M) -> float:
    return math.hypot(body_half, body_half)


def stop_distance_estimate(
    speed: float, decel: float = MAX_DECEL, latency_s: float = STOP_LATENCY_S,
    omega_n: float = FIRMWARE_RAMP_OMEGA_N,
) -> float:
    """衝突監視が0を出してから止まるまでの見積り[m]: v*遅れ + v^2/(2*減速) + 2v/ω_n(臨界減衰の尾。保守側)。"""
    v = abs(float(speed))
    return v * latency_s + v * v / (2.0 * decel) + 2.0 * v / omega_n


def peak_wheel_speed(linear: float, angular: float) -> float:
    """外側の車輪速[m/s]。ファームは約1.23 m/sでクランプする(FIRMWARE_WHEEL_CLAMP_MPS)。"""
    return abs(float(linear)) + abs(float(angular)) * WHEEL_HALF_BASE_M


def _rect(x_near: float, x_far: float, half_y: float) -> str:
    pts = [[x_far, half_y], [x_far, -half_y], [x_near, -half_y], [x_near, half_y]]
    return json.dumps([[round(x, 3), round(y, 3)] for x, y in pts])


_TINY_POINTS = json.dumps([[0.005, 0.005], [0.005, -0.005], [-0.005, -0.005], [-0.005, 0.005]])  # 機体の中心の極小の領域(何も止めない)


def derive_clearance_values(
    clearance_m: float = CLEARANCE_M, body_half: float = BODY_HALF_M
) -> Dict[str, Any]:
    """clearance_m(機体端からの最低距離)から、コストマップのインフレーションと衝突監視の停止領域を導く。

    インフレーション: 半径 = 外接半径 + clearance(角で測っても端から確保できる)。cost_scaling_factorは
    コスト 252*exp(-k*端からの距離) が距離clearanceでCOST_AT_CLEARANCEになるk(端から50cm以上は低コスト、
    それより内側は急に高コスト=迂回を選びやすいが、通路が狭ければ通れる)。
    停止領域(velocity_polygon、座標はbase_link=LiDAR中心、x前、単位m)は速度の向きと速さで切り替える。"""
    c = float(clearance_m)
    if not (math.isfinite(c) and 0.1 <= c <= 1.5):
        raise ValueError("clearance_m")
    h = float(body_half)
    half_y = h + SIDE_MARGIN_M
    x_near = h + BODY_GAP_M
    front = h + FRONT_STOP_CLEARANCE_M
    footprint = json.dumps([[h, h], [h, -h], [-h, -h], [-h, h]])
    ph = min(PLAN_BODY_HALF_M, h)
    plan_footprint = json.dumps([[ph, ph], [ph, -ph], [-ph, -ph], [-ph, ph]])
    sub = {
        # 先頭ほど優先(境界は両側を含む)。止まっている/旋回中(直進ほぼ0)は機体の周りの正方形
        "stopped_rotating": {"points": _rect(-ROTATE_HALF_M, ROTATE_HALF_M, ROTATE_HALF_M),
                             "linear_min": -0.02, "linear_max": 0.02, "theta_min": -4.0, "theta_max": 4.0},
        "forward": {"points": _rect(x_near, front, half_y),
                    "linear_min": 0.02, "linear_max": 0.6, "theta_min": -4.0, "theta_max": 4.0},
        "forward_mid": {"points": _rect(x_near, front + FRONT_EXTRA_MID_M, half_y),
                        "linear_min": 0.6, "linear_max": 0.9, "theta_min": -4.0, "theta_max": 4.0},
        "forward_fast": {"points": _rect(x_near, front + FRONT_EXTRA_FAST_M, half_y),
                         "linear_min": 0.9, "linear_max": 1.5, "theta_min": -4.0, "theta_max": 4.0},
        # 【2026-10-26】指令の速度がどの領域にも合わないと、collision_monitorは毎回(約30Hz)「Velocity is not covered by any of the
        # velocity polygons」の警告を出す(RearStopに分けた直後、実機で前進のたびに大量に出てログとCPUを圧迫した)。そこで、相手
        # (RearStop)が担当する後退の速度を受け持つだけの、機体の中心の極小(1 cm角)の領域を足す。点は入らないので何も止めない。
        "backward_pass": {"points": _TINY_POINTS,
                          "linear_min": -1.5, "linear_max": -0.02, "theta_min": -4.0, "theta_max": 4.0},
    }
    stop = {
        "type": "velocity_polygon",
        "action_type": "stop",
        "holonomic": False,
        "min_points": STOP_MIN_POINTS,
        "visualize": False,
        "enabled": True,
        "velocity_polygons": list(sub),
    }
    stop.update(sub)
    # 【2026-10-24】後退用の停止領域は別のポリゴン(RearStop)にする。person_clear_gateが無効にするのはVelocityStopだけで、RearStopは常に有効。
    # (後ろ向きカメラは真後ろ約±30°が死角で、後退(BackUp 0.7 m/s・壁離脱)中の人を、カメラで「いない」とは言い切れないため。)
    # 後退(linear < -0.02)のときだけ合う。それ以外の速度ではどの領域にも合わず、何も止めない。
    rear = {
        "type": "velocity_polygon",
        "action_type": "stop",
        "holonomic": False,
        "min_points": STOP_MIN_POINTS,
        "visualize": False,
        "enabled": True,
        "velocity_polygons": ["backward", "not_backward"],
        "backward": {"points": _rect(-(h + max(REAR_MARGIN_M, stop_distance_estimate(BACKUP_SPEED_MPS) + BACKUP_STOP_EXTRA_M)), -x_near, half_y),
                     "linear_min": -1.5, "linear_max": -0.02, "theta_min": -4.0, "theta_max": 4.0},
        # 後退以外はVelocityStopの担当。警告を出さないための極小の領域(上の backward_pass と同じ理由)
        "not_backward": {"points": _TINY_POINTS,
                         "linear_min": -0.02, "linear_max": 1.5, "theta_min": -4.0, "theta_max": 4.0},
    }
    return {
        "clearance_m": c,
        "footprint": footprint,
        "plan_footprint": plan_footprint,
        "inflation_radius": round(circumscribed_radius(h) + c, 2),
        "cost_scaling_factor": round(math.log(252.0 / COST_AT_CLEARANCE) / c, 1),
        "stop_polygon": stop,
        "rear_polygon": rear,
    }


def apply_clearance_to_params(params: Dict[str, Any], clearance_m: float = CLEARANCE_M) -> Dict[str, Any]:
    """パース済みnav2_params(dict)を深くコピーし、footprint・インフレーション・停止領域を設定して返す。
    入力は変更しない。存在しないノードの節は作らない。速度・目的地精度の値には触れない。"""
    out = copy.deepcopy(params)
    d = derive_clearance_values(clearance_m)
    for name in ("local_costmap", "global_costmap"):
        sec = out.get(name)
        rp = sec.get(name, {}).get("ros__parameters") if isinstance(sec, dict) else None
        if not isinstance(rp, dict):
            continue
        rp.pop("robot_radius", None)  # footprintと併記しない(footprintが無い場合だけ円として使われる)
        # グローバル(経路計画)は小さめの外形、ローカル(追従)は実寸
        rp["footprint"] = d["plan_footprint"] if name == "global_costmap" else d["footprint"]
        infl = rp.get("inflation_layer")
        if not isinstance(infl, dict):
            infl = {"plugin": "nav2_costmap_2d::InflationLayer"}
            rp["inflation_layer"] = infl
        infl["inflation_radius"] = d["inflation_radius"]
        infl["cost_scaling_factor"] = d["cost_scaling_factor"]
    cm = out.get("collision_monitor")
    rp = cm.get("ros__parameters") if isinstance(cm, dict) else None
    if isinstance(rp, dict):
        polys = list(rp.get("polygons") or [])
        if "VelocityStop" not in polys:
            polys.append("VelocityStop")
        if "RearStop" not in polys:
            polys.append("RearStop")
        rp["polygons"] = polys
        rp["VelocityStop"] = copy.deepcopy(d["stop_polygon"])
        rp["RearStop"] = copy.deepcopy(d["rear_polygon"])
    return out


# ---------------------------------------------------------------- ビヘイビアツリー(2026-10-10)
# 立ちふさがられたとき「待つ -> 少し下がる -> 経路を作り直す」ためのBT。config/nokosahen_nav_to_pose_bt.xml。
# yamlのbt_navigator.default_nav_to_pose_bt_xmlはファイル名だけを書き、起動時の書き換えで同じ場所の絶対パスにする
# (インストール先がJetsonごとに違っても動かすため)。ファイルが無ければ項目を消し、Nav2標準のBTに戻す。
BT_KEY = "default_nav_to_pose_bt_xml"


BT_ENV = "NOKOSAHEN_BT"  # "legacy" なら変更前のBT(<名前>.legacy.xml)を使う。既定は新しいBT
BT_LEGACY_VALUE = "legacy"


def legacy_bt_name(name: str) -> str:
    root, ext = os.path.splitext(name)
    return f"{root}.legacy{ext}"


def apply_bt_to_params(params: Dict[str, Any], config_dir: str, env: Optional[Dict[str, str]] = None) -> Dict[str, Any]:
    out = copy.deepcopy(params)
    if env is None:
        env = os.environ
    sec = out.get("bt_navigator")
    rp = sec.get("ros__parameters") if isinstance(sec, dict) else None
    if not isinstance(rp, dict):
        return out
    name = rp.get(BT_KEY)
    if not isinstance(name, str) or not name or os.path.isabs(name):
        return out
    path = os.path.join(config_dir, name)
    if str(env.get(BT_ENV, "")).strip().lower() == BT_LEGACY_VALUE:
        legacy = os.path.join(config_dir, legacy_bt_name(name))
        if os.path.isfile(legacy):
            path = legacy
    if os.path.isfile(path):
        rp[BT_KEY] = os.path.abspath(path)
    else:
        rp.pop(BT_KEY, None)
    return out


def load_yaml_dict(path: str) -> Dict[str, Any]:
    import yaml

    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: トップレベルがマップではありません")
    return data


def write_rewritten_params(
    src_path: str, speed: float, out_path: Optional[str] = None, goal_precision: bool = True
) -> str:
    """src_pathのnav2 paramsを読み、速度(と、goal_precisionなら目的地の到着精度)、機体端からの距離
    (footprint・インフレーション・停止領域)、BTの絶対パスを反映した一時コピーを書いてパスを返す。"""
    import yaml

    rewritten = apply_nav_speed_to_params(load_yaml_dict(src_path), speed)
    if goal_precision:
        rewritten = apply_goal_precision_to_params(rewritten)
    rewritten = apply_clearance_to_params(rewritten)
    rewritten = apply_bt_to_params(rewritten, os.path.dirname(os.path.abspath(src_path)))
    if out_path is None:
        out_path = os.path.join(tempfile.gettempdir(), f"nokosahen_nav2_params_{os.getpid()}.yaml")
    with open(out_path, "w", encoding="utf-8", newline="\n") as f:
        f.write("# nav_speed.pyが起動時に生成した一時コピー(元: %s、Nav2最高速度 %.3f m/s)\n" % (src_path, speed))
        yaml.safe_dump(rewritten, f, allow_unicode=True, sort_keys=False, default_flow_style=None)
    return out_path


def precheck_nav_request(requested: Any, confirm_high: Any) -> Optional[Tuple[int, Dict[str, Any]]]:
    """副作用なしの事前検証。問題があれば(400, 応答)、無ければNone。
    ダッシュボードは他の速度設定を保存する前にこれを呼び、不正なら何も書かない。"""
    try:
        v = normalize_nav_speed(requested)
    except ValueError:
        return 400, {"error": "nav_max_speed_mps must be a finite number",
                     "message": "自動走行(Nav2)の最高速度は有限の数値で指定してください。"}
    if v > NAV_SPEED_CONFIRM_ABOVE and confirm_high is not True:
        return 400, {
            "error": "confirm_required", "confirm_required": True,
            "nav_max_speed_mps": v, "nav_max_speed_requested": float(requested),
            "message": f"{NAV_SPEED_CONFIRM_ABOVE:g} m/s を超える速度には確認が必要です(confirm_high)。",
        }
    return None


# ---------------------------------------------------------------- 記憶値(JSON)

def save_path_expanded(path: Optional[str] = None) -> str:
    return os.path.expanduser(path or DEFAULT_SAVE_PATH)


def load_saved_nav_speed(
    path: Optional[str] = None, now: Optional[float] = None
) -> Optional[float]:
    """記憶値を検証して返す。無い/壊れている/古い/範囲外/非有限なら None。"""
    p = save_path_expanded(path)
    try:
        with open(p, "r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            return None
        v = data.get("nav_max_speed_mps")
        saved_at = data.get("saved_at")
        if isinstance(v, bool) or isinstance(saved_at, bool):
            return None
        v = float(v)
        saved_at = float(saved_at)
        if not (math.isfinite(v) and math.isfinite(saved_at)):
            return None
        if not (NAV_SPEED_MIN <= v <= NAV_SPEED_MAX):
            return None
        t = time.time() if now is None else now
        if saved_at > t + 300 or t - saved_at > SAVED_MAX_AGE_SEC:
            return None
        return v
    except (OSError, ValueError, TypeError):
        return None


def save_nav_speed(speed: float, path: Optional[str] = None, now: Optional[float] = None) -> bool:
    p = save_path_expanded(path)
    try:
        v = normalize_nav_speed(speed)
        tmp = p + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump({"nav_max_speed_mps": v, "saved_at": time.time() if now is None else now}, f)
        os.replace(tmp, p)
        return True
    except (OSError, ValueError):
        return False


def clear_saved_nav_speed(path: Optional[str] = None) -> None:
    try:
        os.remove(save_path_expanded(path))
    except OSError:
        pass


def read_yaml_default(params_file: Optional[str]) -> float:
    """nokosahen.yamlの`nokosahen_launch.ros__parameters.nav_max_speed`(ユーザー向け既定値)。
    無い/不正なら NAV_SPEED_DEFAULT(0.5)。"""
    try:
        if params_file:
            data = load_yaml_dict(params_file)
            v = data["nokosahen_launch"]["ros__parameters"]["nav_max_speed"]
            return normalize_nav_speed(v)
    except Exception:  # noqa: BLE001
        pass
    return NAV_SPEED_DEFAULT


def resolve_launch_nav_speed(
    arg_value: Any, use_saved: bool = True, save_path: Optional[str] = None
) -> Tuple[float, str]:
    """起動時に使う速度と出所。優先順位: 記憶値(use_saved、新しく有効) > 引数 > 0.5。"""
    if use_saved:
        saved = load_saved_nav_speed(save_path)
        if saved is not None:
            return saved, "saved"
    try:
        return normalize_nav_speed(arg_value), "arg"
    except ValueError:
        return NAV_SPEED_DEFAULT, "default"


# ---------------------------------------------------------------- 実行時コントローラ

class RclpyParamClient:
    """rclpyのパラメータサービスを使うクライアント。ノードはexecutorで別スレッドが
    spinしている前提(完了コールバック+Eventで待つので、ここではspinしない)。"""

    def __init__(self, node, prefix: str = "/") -> None:
        self._node = node
        self._prefix = prefix
        self._set_clients: Dict[str, Any] = {}
        self._get_clients: Dict[str, Any] = {}
        self._lock = threading.Lock()

    def _client(self, kind: str, node_name: str):
        from rcl_interfaces.srv import GetParameters, SetParameters

        cache = self._set_clients if kind == "set" else self._get_clients
        with self._lock:
            c = cache.get(node_name)
            if c is None:
                srv = SetParameters if kind == "set" else GetParameters
                c = self._node.create_client(srv, f"{self._prefix}{node_name}/{kind}_parameters")
                cache[node_name] = c
            return c

    def service_ready(self, node_name: str, timeout: float) -> bool:
        return bool(self._client("set", node_name).wait_for_service(timeout_sec=timeout))

    def _call(self, client, request, timeout: float):
        done = threading.Event()
        fut = client.call_async(request)
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            try:
                client.remove_pending_request(fut)
            except Exception:  # noqa: BLE001
                pass
            raise TimeoutError(f"{timeout:g}秒以内に応答がありません")
        exc = fut.exception()
        if exc is not None:
            raise RuntimeError(str(exc))
        return fut.result()

    @staticmethod
    def _to_value(value):
        from rcl_interfaces.msg import ParameterType, ParameterValue

        pv = ParameterValue()
        if isinstance(value, (list, tuple)):
            pv.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            pv.double_array_value = [float(x) for x in value]
        elif isinstance(value, bool):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = value
        elif isinstance(value, int):
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = int(value)
        else:
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)
        return pv

    @staticmethod
    def _from_value(pv):
        from rcl_interfaces.msg import ParameterType

        t = pv.type
        if t == ParameterType.PARAMETER_DOUBLE:
            return pv.double_value
        if t == ParameterType.PARAMETER_INTEGER:
            return pv.integer_value
        if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(pv.double_array_value)
        if t == ParameterType.PARAMETER_INTEGER_ARRAY:
            return list(pv.integer_array_value)
        if t == ParameterType.PARAMETER_BOOL:
            return pv.bool_value
        if t == ParameterType.PARAMETER_STRING:
            return pv.string_value
        return None

    def set_parameters(
        self, node_name: str, values: Dict[str, Any], timeout: float
    ) -> Dict[str, Tuple[bool, str]]:
        from rcl_interfaces.msg import Parameter
        from rcl_interfaces.srv import SetParameters

        req = SetParameters.Request()
        names = list(values.keys())
        for n in names:
            p = Parameter()
            p.name = n
            p.value = self._to_value(values[n])
            req.parameters.append(p)
        resp = self._call(self._client("set", node_name), req, timeout)
        return {n: (bool(r.successful), str(r.reason)) for n, r in zip(names, resp.results)}

    def get_parameters(self, node_name: str, names: List[str], timeout: float) -> Dict[str, Any]:
        from rcl_interfaces.srv import GetParameters

        req = GetParameters.Request()
        req.names = list(names)
        resp = self._call(self._client("get", node_name), req, timeout)
        out = {}
        for n, pv in zip(names, resp.values):
            v = self._from_value(pv)
            if v is not None:
                out[n] = v
        return out


CALL_TIMEOUT_SEC = 3.0
READY_TIMEOUT_SEC = 1.0

# 読み戻し(表示)用に読むパラメータ
_READBACK = {
    NODE_CONTROLLER: [
        "FollowPath.max_vel_x", "FollowPath.max_speed_xy", "FollowPath.acc_lim_x",
        "FollowPath.decel_lim_x", "FollowPath.sim_time", "FollowPath.vx_samples",
    ],
    NODE_SMOOTHER: ["max_velocity", "max_accel", "max_decel"],
    NODE_COLLISION: ["FootprintApproach.time_before_collision"],
}


class NavSpeedController:
    """ダッシュボードから、実行中のNav2の最高速度を変更する(再起動不要)。

    パラメータを書くだけで、cmd_velもゴールも出さない。ソフト緊急停止中でも
    パラメータ設定自体は動作ではないので拒否しない(従来どおり)。"""

    def __init__(
        self,
        client=None,
        node=None,
        save_path: Optional[str] = None,
        now: Callable[[], float] = time.time,
        default_speed: float = NAV_SPEED_DEFAULT,
    ) -> None:
        if client is None and node is not None:
            client = RclpyParamClient(node)
        self._client = client
        self._save_path = save_path
        self._now = now
        self._default = default_speed
        self._lock = threading.Lock()
        self._applied: Optional[float] = None  # このダッシュボードが実行中Nav2へ最後に反映した値

    # --- 表示用
    def effective_speed(self) -> Tuple[float, str]:
        with self._lock:
            applied = self._applied
        if applied is not None:
            return applied, "live"
        saved = load_saved_nav_speed(self._save_path, self._now())
        if saved is not None:
            return saved, "saved"
        return self._default, "default"

    def status(self) -> Dict[str, Any]:
        v, source = self.effective_speed()
        return {
            "nav_max_speed_mps": v,
            "nav_max_speed_source": source,
            "nav_speed_min": NAV_SPEED_MIN,
            "nav_speed_max": NAV_SPEED_MAX,
            "nav_speed_default": NAV_SPEED_DEFAULT,
            "nav_speed_confirm_above": NAV_SPEED_CONFIRM_ABOVE,
        }

    def read_back(self) -> Dict[str, Any]:
        """実行中ノードから現在値を読む(3秒タイムアウト)。Nav2が無ければ running=False。"""
        out: Dict[str, Any] = {"running": False, "nodes": {}}
        if self._client is None:
            return out
        try:
            if not self._client.service_ready(NODE_CONTROLLER, READY_TIMEOUT_SEC):
                return out
        except Exception:  # noqa: BLE001
            return out
        out["running"] = True
        for node, names in _READBACK.items():
            try:
                out["nodes"][node] = self._client.get_parameters(node, names, CALL_TIMEOUT_SEC)
            except Exception as exc:  # noqa: BLE001
                out["nodes"][node] = {"error": str(exc)}
        return out

    # --- 変更
    def apply(self, requested: Any, confirm_high: bool = False) -> Tuple[int, Dict[str, Any]]:
        """(HTTPステータス, 応答)。400=不正値/要確認、409=Nav2未起動(値は記憶)、200=反映試行済み。"""
        pre = precheck_nav_request(requested, confirm_high)
        if pre is not None:
            return pre
        v = normalize_nav_speed(requested)
        raw = float(requested)
        clamped = abs(raw - v) > 1e-9
        notes = []
        if clamped:
            notes.append(f"指定値 {raw:g} が範囲外のため {v:g} m/s に丸めました(範囲 {NAV_SPEED_MIN:g}〜{NAV_SPEED_MAX:g})。")
        base = {"nav_max_speed_requested": raw, "clamped": clamped}
        if self._client is None:
            return 409, self._not_running(v, base, notes)

        result: Dict[str, Any] = {}

        def work():
            result["r"] = self._apply_worker(v)

        t = threading.Thread(target=work, daemon=True)
        t.start()
        t.join(timeout=60.0)
        if "r" not in result:
            return 504, dict(base, error="timeout", message="Nav2への反映が時間切れです。")
        running, node_results, readback = result["r"]
        if not running:
            return 409, self._not_running(v, base, notes)

        all_ok = all(n["ok"] for n in node_results)
        if all_ok:
            with self._lock:
                self._applied = v
            # 高速が次回起動へ勝手に持ち越されないよう、記憶値は消す(起動時は既定0.5に戻る)
            clear_saved_nav_speed(self._save_path)
            notes.append(f"自動走行(Nav2)の最高速度を {v:g} m/s に設定しました(全ノードに反映)。")
        else:
            failed = [NODE_LABELS.get(n["node"], n["node"]) for n in node_results if not n["ok"]]
            notes.append("一部のノードに反映できませんでした: " + "、".join(failed)
                         + "。ノード間で速度設定が揃っていません。再実行するか、Nav2を再起動してください。")
        warn = self._costmap_warning(readback.get("local_costmap"), v)
        if warn:
            notes.append(warn)
        payload = dict(base, **{
            "ok": all_ok,
            "nav_max_speed_mps": v,
            "nodes": node_results,
            "read_back": readback,
            "message": " ".join(notes),
        })
        return 200, payload

    def _not_running(self, v: float, base: Dict[str, Any], notes: List[str]) -> Dict[str, Any]:
        saved = save_nav_speed(v, self._save_path, self._now())
        msg = "Nav2が起動していません(地図作成/位置合わせの開始後に再度設定してください)。"
        if saved:
            msg += f" {v:g} m/s を記憶しました。次にNav2を起動するとき(12時間以内)の既定値になります。"
        return dict(base, **{
            "error": "nav2_not_running", "ok": False, "nav_max_speed_mps": v,
            "saved_for_next_launch": saved, "message": " ".join(notes + [msg]),
        })

    @staticmethod
    def _costmap_warning(costmap: Optional[Dict[str, Any]], v: float) -> Optional[str]:
        if not isinstance(costmap, dict):
            return None
        need = derive_nav_speed_values(v)["costmap_size"]
        try:
            have = min(float(costmap["width"]), float(costmap["height"]))
        except (KeyError, TypeError, ValueError):
            return None
        if have + 1e-9 < need:
            return (f"ローカルコストマップが {have:g}m 四方で、この速度に必要な {need} m 四方より小さいです"
                    "(起動時のみ反映されるため、次のNav2起動で自動的に広がります)。")
        return None

    def _apply_worker(self, v: float):
        client = self._client
        if not client.service_ready(NODE_CONTROLLER, READY_TIMEOUT_SEC):
            return False, [], {}
        # 現在の配列(旋回要素などを保つため)と、現在の速度(上げ/下げの順序決定)を読む
        current_arrays: Dict[str, List[float]] = {}
        smoother_err = None
        cur_speed = None
        try:
            got = client.get_parameters(NODE_SMOOTHER, ["max_velocity", "max_accel", "max_decel"], CALL_TIMEOUT_SEC)
            current_arrays = {k: x for k, x in got.items() if isinstance(x, (list, tuple))}
            if "max_velocity" in current_arrays:
                cur_speed = float(current_arrays["max_velocity"][0])
        except Exception as exc:  # noqa: BLE001
            smoother_err = str(exc)

        values = live_parameters(v, current_arrays)
        raising = cur_speed is None or v >= cur_speed
        # 上げるときは安全側(衝突監視→経路追従)を先に、最後に速度上限(smoother)。
        # 下げるときは逆順(速度上限を先に下げる)。
        order = [NODE_COLLISION, NODE_CONTROLLER, NODE_SMOOTHER] if raising else [NODE_SMOOTHER, NODE_CONTROLLER, NODE_COLLISION]
        results = []
        for node in order:
            entry = {"node": node, "label": NODE_LABELS[node], "ok": False, "detail": ""}
            if node == NODE_SMOOTHER and smoother_err is not None:
                entry["detail"] = "現在値を読めませんでした: " + smoother_err
                results.append(entry)
                continue
            try:
                res = client.set_parameters(node, values[node], CALL_TIMEOUT_SEC)
                bad = [f"{n}: {r[1] or '拒否'}" for n, r in res.items() if not r[0]]
                missing = [n for n in values[node] if n not in res]
                if bad or missing:
                    entry["detail"] = "失敗 " + ", ".join(bad + [f"{n}: 応答なし" for n in missing])
                else:
                    entry["ok"] = True
                    entry["detail"] = f"{len(res)}項目を設定"
            except Exception as exc:  # noqa: BLE001
                entry["detail"] = f"{type(exc).__name__}: {exc}"
            results.append(entry)

        readback: Dict[str, Any] = {}
        for node, names in _READBACK.items():
            try:
                readback[node] = client.get_parameters(node, names, CALL_TIMEOUT_SEC)
            except Exception as exc:  # noqa: BLE001
                readback[node] = {"error": str(exc)}
        try:
            readback["local_costmap"] = client.get_parameters(NODE_LOCAL_COSTMAP, ["width", "height"], CALL_TIMEOUT_SEC)
        except Exception:  # noqa: BLE001
            pass
        return True, sorted(results, key=lambda r: list(NODE_LABELS).index(r["node"])), readback
