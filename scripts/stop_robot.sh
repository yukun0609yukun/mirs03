#!/usr/bin/env bash
# Jetson上で動いているのこさへん関連のROS2プロセスを全て止める。
# SSH接続が不安定でも確実に効くよう、複数回ループしてpkillする。
#
# 使い方:
#   bash ~/mirs2603_nokosahen/scripts/stop_robot.sh
#   bash ~/mirs2603_nokosahen/scripts/stop_robot.sh --keep-dashboard   # 管理画面(persistent_base/full_dashboard)だけ残す
#     (2026-10-13追加。管理画面の「プログラム終了」が scripts/shutdown_all.sh から使う。管理画面は最後に別手順で止める)
#
# 終了コード(2026-10-21): 0=全部止まった / 1=SIGKILLを送っても、まだ残っているプロセスがある(残りは標準出力に出す)。
#   以前は最後がechoだけで常に0だったため、start_all.sh / shutdown_all.sh が「停止に失敗」を検知できなかった。
#
# 詳細: docs/field_ssh_operations.md
set -uo pipefail

# 実行ファイル名・ノード名のパターン。起動引数(use_behavior_director:=false など)の文字に一致して、管理画面を含む親プロセス
# (persistent_base.launch.py)まで止めてしまわないよう、*.launch.py 以外は「直前が空白かスラッシュ」のものだけに一致させる
# (パスの `.../lib/nokosahen_core/behavior_director` や `ros2 run nokosahen_core behavior_director` には一致し、
#  `use_behavior_director:=false` には一致しない)。
PATTERNS=(
    # 【2026-11-06】device_watchdog(機器監視)は、止めたrplidar_node/micro_ros_agentを数秒後に起動し直してしまうので、最初に止める
    "device_watchdog"
    "persistent_base.launch.py"
    "mapping_mode.launch.py"
    "localization_mode.launch.py"
    "jetson_robot.launch.py"
    "jetson_robot_full.launch.py"
    "rplidar_node"
    "scan_self_filter"
    "micro_ros_agent"
    "inventory_manager"
    "web_ui"
    "menu_schedule"
    "crowd_monitor"
    "behavior_director"
    "wheel_odom"
    "static_transform_publisher"
    "person_target_detector"
    "voice_node"
    "announce_node"
    "survey_monitor"
    "stage1_dashboard"
    "stage2_dashboard"
    "full_dashboard"
    "slam_toolbox"
    "nav2_"
    "opennav_docking"
    "frontier_explorer"
    # 【2026-10-23】今回足したノード。ここに無いと、全停止のあとも(特にlaunchの外で起動した/launchが先に落ちたとき)残って、カメラや調停を握り続ける
    "rear_camera_persons"
    "pose_relay"
    "wall_escape"
    "person_clear_gate"
    "motion_arbiter"
)

KEEP_DASHBOARD=0
[ "${1:-}" = "--keep-dashboard" ] && KEEP_DASHBOARD=1

regex_of() {  # regex_of <パターン>: *.launch.py はそのまま、それ以外は直前が空白かスラッシュのものだけ
    # 【2026-11-05】nav2_*/slam_toolbox/opennav_docking(ディレクトリ名で前方一致させるもの)以外は、直後も「空白か行末」に限る。
    # 以前は前方一致だったため、`vim .../full_dashboard_node.py` や `tail -f .../behavior_director.log` なども止めていた。
    # 本物のノードは `.../lib/nokosahen_core/full_dashboard` か `ros2 run nokosahen_core full_dashboard`(あとに引数)の形。
    case "$1" in
        *.launch.py) printf '%s' "$1" ;;
        nav2_|slam_toolbox|opennav_docking) printf '(^|[ /])%s' "$1" ;;
        *) printf '(^|[ /])%s($| )' "$1" ;;
    esac
}

kept() {  # kept <パターン>: --keep-dashboard のとき残す対象か
    [ "$KEEP_DASHBOARD" = "1" ] || return 1
    case "$1" in persistent_base.launch.py|stage1_dashboard|stage2_dashboard|full_dashboard) return 0 ;; esac
    return 1
}

# 【2026-11-06】device_watchdog を最初に止める。launchの respawn で戻ってきても何もしないよう、先に一時停止ファイルを置く
# (watchdog はこのファイルがある間、監視も再起動もしない)。パターンは regex_of 経由なので、自分のコマンド行は巻き込まない。
# 通常の停止(launchごと全部止める)では、全部止めたあとでファイルを消す。--keep-dashboard では、launch(とwatchdog)が
# 残るので、ファイルを残して再起動を防ぐ(解除は、通常の stop_robot.sh か、rm /tmp/nokosahen_device_watchdog.pause)。
DW_PAUSE="${NOKOSAHEN_DW_PAUSE_FILE:-/tmp/nokosahen_device_watchdog.pause}"
touch "$DW_PAUSE" 2>/dev/null
pkill -f "$(regex_of "device_watchdog")" 2>/dev/null
sleep 1

for round in 1 2 3 4 5; do
    for pat in "${PATTERNS[@]}"; do
        kept "$pat" && continue
        pkill -f "$(regex_of "$pat")" 2>/dev/null
    done
    sleep 1
done

# SIGTERMで終わらないものは、SIGKILLで止める。それでも残るものがあれば、終了コード1で知らせる
for pat in "${PATTERNS[@]}"; do
    kept "$pat" && continue
    re="$(regex_of "$pat")"
    [ "$(pgrep -fc "$re" 2>/dev/null)" != "0" ] && pkill -9 -f "$re" 2>/dev/null
done
sleep 1
LEFT=0
for pat in "${PATTERNS[@]}"; do
    kept "$pat" && continue
    n="$(pgrep -fc "$(regex_of "$pat")" 2>/dev/null)"
    if [ "${n:-0}" != "0" ]; then
        echo "  残っています: $pat ($n 件)"
        LEFT=1
    fi
done

if [ "$KEEP_DASHBOARD" = "0" ] && [ "$LEFT" = "0" ]; then
    rm -f "$DW_PAUSE" 2>/dev/null   # 全部止まったので、次の起動で watchdog が働けるようにする
fi

if [ "$LEFT" = "0" ]; then
    echo "停止コマンドを投入しました(必要なものはSIGKILLまで)。'ros2 node list' で何も出なければ停止完了です。"
else
    echo "停止しきれないプロセスがあります(上の一覧)。手動で確認してください。"
fi
exit "$LEFT"
