#!/usr/bin/env bash
# 「ラジコン運用のみ」の階層: ESP32モーター制御・LiDAR・ダッシュボードは
# 起動するが、自律行動(behavior_director、見回し・接近・案内)は起動しない。
# 自律行動ありで起動したい場合は scripts/start_persistent_base.sh を使う
# (こちらは既定のuse_behavior_director:=trueのまま)。
#
# 使い方:
#   bash ~/mirs2603_nokosahen/scripts/start_persistent_base_rc_only.sh
#
# 詳細: docs/quick_start_ssh.md, docs/known_issues_and_startup_checklist.md
set -uo pipefail

# 二重起動の防止(B5): すでにpersistent_base系が動いていたら拒否する。回避: NOKOSAHEN_ALLOW_MULTI_BASE=1
source "$(dirname "${BASH_SOURCE[0]}")/base_guard.sh"
nokosahen_base_guard start_persistent_base_rc_only.sh

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
ESP32_PORT="${ESP32_PORT:-/dev/esp32}"
LIDAR_PORT="${LIDAR_PORT:-/dev/rplidar}"

# ROSのsetup.bashは未定義変数を参照することがあるため、source中だけは
# nounset(set -u)を無効にしておく(set -euしていると誤爆して途中終了する)。
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
source ~/lidar_src_build/install/setup.bash
source ~/microros_ws/install/setup.bash
source ~/mirs2603_nokosahen/ros2_ws/install/setup.bash

nohup ros2 launch nokosahen_bringup persistent_base.launch.py \
    esp32_port:="${ESP32_PORT}" lidar_port:="${LIDAR_PORT}" \
    use_behavior_director:=false \
    > ~/.nokosahen_persistent_base.log 2>&1 < /dev/null &
disown

echo "起動コマンドを投入しました(ラジコン運用のみ、自律行動なし)。"
echo "ログ: ~/.nokosahen_persistent_base.log"
echo "5秒ほど待ってから 'ros2 node list' で確認してください(behavior_directorが出ないことが正常です)。"
