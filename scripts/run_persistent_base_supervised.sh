#!/usr/bin/env bash
# ノード監視・自動復旧スクリプト(2026-07-22追加)。
#
# persistent_base.launch.pyを起動し、何らかの理由でlaunchプロセス全体が
# 終了したら(1ノードのクラッシュでlaunchツリー全体が落ちるケースを含む)
# 自動的に再起動し続ける。無人稼働中の実証実験でも、SSHを切断していても
# 動き続けるようにする(nohup + disownして呼び出すこと、下記使い方参照)。
#
# 個別ノード単位の監視ではなく、persistent_base.launch.py全体を1つの単位と
# して監視する(このセッションで実際に使っている起動単位と同じ粒度にする
# ことで、監視ロジック自体をシンプルに保つ)。
#
# 使い方:
#   # 自律行動あり
#   nohup bash ~/mirs2603_nokosahen/scripts/run_persistent_base_supervised.sh \
#       >> ~/.nokosahen_supervisor_wrapper.log 2>&1 < /dev/null &
#   disown
#
#   # ラジコン運用オンリー(自律行動なし)
#   nohup bash ~/mirs2603_nokosahen/scripts/run_persistent_base_supervised.sh rc_only \
#       >> ~/.nokosahen_supervisor_wrapper.log 2>&1 < /dev/null &
#   disown
#
# 停止するには、通常のscripts/stop_robot.shに加えて、このスクリプト自身の
# プロセスも止める必要がある(自動的にすぐ再起動してしまうため):
#   pkill -f run_persistent_base_supervised.sh
#   bash ~/mirs2603_nokosahen/scripts/stop_robot.sh
set -uo pipefail

MODE="${1:-full}"

# 【2026-10-21】二重起動の防止(全体レビュー)。監視ループが2組あると、ROS一式が二重に起動して、/dev/esp32 と /dev/rplidar を奪い合う
# (systemctl restart nokosahen-robot は監視ループだけを止める・start_all.shのあとにsystemctl startする、など)。
# 排他ロックを取れなければ、何もせず終了する(終了コード0)。ロックは、このプロセスが終われば自動で解放される。
# 起動する子プロセス(ros2 launch)には、ロックを引き継がせない(下のループの 9>&-)。引き継ぐと、監視ループだけが落ちたときに、
# 残った子プロセスがロックを持ち続けて、新しい監視ループが起動できなくなる。
LOCK_FILE="${NOKOSAHEN_SUPERVISOR_LOCK:-/tmp/nokosahen_supervisor.lock}"
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
    echo "$(date): すでに監視ループが動いているため、起動しません(ロック: $LOCK_FILE)" >> "${HOME}/.nokosahen_supervisor.log"
    echo "すでに監視ループが動いています。止めるには: bash ~/mirs2603_nokosahen/scripts/stop_everything.sh"
    exit 0
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
# Fast DDSを有線NICのみに制限(無効化: NOKOSAHEN_DDS_WIRED=0)。詳細: config/fastdds_wired.xml
source "$HOME/mirs2603_nokosahen/scripts/dds_wired_env.sh"
ESP32_PORT="${ESP32_PORT:-/dev/esp32}"
LIDAR_PORT="${LIDAR_PORT:-/dev/rplidar}"
LOG_FILE="${HOME}/.nokosahen_persistent_base.log"
SUPERVISOR_LOG="${HOME}/.nokosahen_supervisor.log"

# 追加のlaunch引数(例: EXTRA_LAUNCH_ARGS="use_full_dashboard:=true use_stage2_dashboard:=false")。
# 2026-10-02追加: 総合ダッシュボード構成でも自動復旧を使えるようにするため。
USE_BEHAVIOR_DIRECTOR="true"
if [ "$MODE" = "rc_only" ]; then
    USE_BEHAVIOR_DIRECTOR="false"
fi

# ROSのsetup.bashは未定義変数を参照することがあるため、source中だけは
# nounset(set -u)を無効にしておく(set -euしていると誤爆して途中終了する)。
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
source ~/lidar_src_build/install/setup.bash
source ~/microros_ws/install/setup.bash
source ~/mirs2603_nokosahen/ros2_ws/install/setup.bash
set -u

echo "$(date): 監視ループ開始(mode=${MODE})" >> "$SUPERVISOR_LOG"

# 2台目・ラズパイの時計を1台目に揃える(2026-10-09追加)。非ブロッキング・失敗しても本体は止めない。
# 1台目自身の時計が古い(2026年より前)ときはsync_clocks.sh側が警告のみで同期しない。docs/clock_sync.md
if [ -f "$HOME/mirs2603_nokosahen/scripts/sync_clocks.sh" ]; then
    ( bash "$HOME/mirs2603_nokosahen/scripts/sync_clocks.sh" 2>&1         | while IFS= read -r l; do echo "$(date '+%F %T') $l"; done         >> "$HOME/.nokosahen_clock_sync.log" ) > /dev/null 2>&1 < /dev/null 9>&- &
fi


while true; do
    START_TIME=$(date +%s)
    echo "$(date): persistent_base起動" >> "$SUPERVISOR_LOG"
    ros2 launch nokosahen_bringup persistent_base.launch.py \
        esp32_port:="${ESP32_PORT}" lidar_port:="${LIDAR_PORT}" \
        use_behavior_director:="${USE_BEHAVIOR_DIRECTOR}" \
        ${EXTRA_LAUNCH_ARGS:-} \
        >> "$LOG_FILE" 2>&1 9>&-
    EXIT_CODE=$?
    END_TIME=$(date +%s)
    RUNTIME=$((END_TIME - START_TIME))
    echo "$(date): persistent_baseが終了しました(exit=${EXIT_CODE}、稼働時間=${RUNTIME}秒)。再起動します。" >> "$SUPERVISOR_LOG"
    if [ "$RUNTIME" -lt 10 ]; then
        # 起動直後に即終了を繰り返す(設定ミス等)場合、無限ループで
        # CPU・ログを食い潰さないよう待ち時間を長くする。
        sleep 10
    else
        sleep 2
    fi
done
