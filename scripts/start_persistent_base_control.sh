#!/usr/bin/env bash
# 【2026-08-05追加】Jetson 2台分散構成の「足回り・安全系(Jetson A)」を
# このJetson上で起動する。対になるアプリ系は別のJetson(B)で
# start_persistent_base_app.sh を実行すること。
#
# 前提: Jetson A・BはLANケーブルで接続し、同じROS_DOMAIN_ID(既定77)。
# 詳細: ros2_ws/src/nokosahen_bringup/launch/persistent_base_control.launch.py
# の冒頭コメント、docs/known_issues_and_startup_checklist.md 補足6。
#
# 使い方:
#   bash ~/mirs2603_nokosahen/scripts/start_persistent_base_control.sh
#   自律行動なし(ラジコン運用オンリー)にしたい場合:
#   USE_BEHAVIOR_DIRECTOR=false bash ~/mirs2603_nokosahen/scripts/start_persistent_base_control.sh
#
# 【未検証】2台目Jetsonのセットアップ前に用意した準備段階のスクリプトであり、
# 実機2台構成での動作確認はまだ行っていない。
set -uo pipefail

# 二重起動の防止(B5): すでにpersistent_base系が動いていたら拒否する。回避: NOKOSAHEN_ALLOW_MULTI_BASE=1
source "$(dirname "${BASH_SOURCE[0]}")/base_guard.sh"
nokosahen_base_guard start_persistent_base_control.sh

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
ESP32_PORT="${ESP32_PORT:-/dev/esp32}"
LIDAR_PORT="${LIDAR_PORT:-/dev/rplidar}"
USE_BEHAVIOR_DIRECTOR="${USE_BEHAVIOR_DIRECTOR:-true}"

# ROSのsetup.bashは未定義変数を参照することがあるため、source中だけは
# nounset(set -u)を無効にしておく(set -euしていると誤爆して途中終了する)。
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
source ~/lidar_src_build/install/setup.bash
source ~/microros_ws/install/setup.bash
source ~/mirs2603_nokosahen/ros2_ws/install/setup.bash

nohup ros2 launch nokosahen_bringup persistent_base_control.launch.py \
    esp32_port:="${ESP32_PORT}" lidar_port:="${LIDAR_PORT}" \
    use_behavior_director:="${USE_BEHAVIOR_DIRECTOR}" \
    > ~/.nokosahen_persistent_base_control.log 2>&1 < /dev/null &
disown

echo "起動コマンドを投入しました(Jetson A: 足回り・安全系)。"
echo "ログ: ~/.nokosahen_persistent_base_control.log"
echo "対になるJetson Bで start_persistent_base_app.sh も忘れずに起動してください。"
