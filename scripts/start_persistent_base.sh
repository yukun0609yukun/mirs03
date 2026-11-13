#!/usr/bin/env bash
# Jetson上で「土台」(ESP32モーター制御・LiDAR・ダッシュボード・behavior_director等)を
# バックグラウンドで起動する。SSHが切れても動き続けるように nohup + disown する。
#
# 使い方:
#   bash ~/mirs2603_nokosahen/scripts/start_persistent_base.sh
#
# 詳細: docs/field_ssh_operations.md

# 二重起動の防止(B5): すでにpersistent_base系が動いていたら拒否する。回避: NOKOSAHEN_ALLOW_MULTI_BASE=1
source "$(dirname "${BASH_SOURCE[0]}")/base_guard.sh"
nokosahen_base_guard start_persistent_base.sh

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
# Fast DDSを有線NICのみに制限(無効化: NOKOSAHEN_DDS_WIRED=0)。詳細: config/fastdds_wired.xml
source "$HOME/mirs2603_nokosahen/scripts/dds_wired_env.sh"
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
    > ~/.nokosahen_persistent_base.log 2>&1 < /dev/null &
disown

echo "起動コマンドを投入しました。ログ: ~/.nokosahen_persistent_base.log"
echo "5秒ほど待ってから 'ros2 node list' で確認してください。"
