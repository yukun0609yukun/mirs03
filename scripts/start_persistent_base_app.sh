#!/usr/bin/env bash
# 【2026-08-05追加】Jetson 2台分散構成の「アプリ・表示系(Jetson B)」を
# このJetson上で起動する。対になる足回り・安全系は別のJetson(A)で
# start_persistent_base_control.sh を実行すること。
#
# 前提: Jetson A・BはLANケーブルで接続し、同じROS_DOMAIN_ID(既定77)。
# LiDAR/microROS向けのソースビルド(lidar_src_build/microros_ws)は
# Jetson A側にしか要らないため、こちらではsourceしない。
# 詳細: ros2_ws/src/nokosahen_bringup/launch/persistent_base_app.launch.py
# の冒頭コメント、docs/known_issues_and_startup_checklist.md 補足6。
#
# 使い方:
#   bash ~/mirs2603_nokosahen/scripts/start_persistent_base_app.sh
#
# 【未検証】2台目Jetsonのセットアップ前に用意した準備段階のスクリプトであり、
# 実機2台構成での動作確認はまだ行っていない。
set -uo pipefail

# 二重起動の防止(B5): すでにpersistent_base系が動いていたら拒否する。回避: NOKOSAHEN_ALLOW_MULTI_BASE=1
source "$(dirname "${BASH_SOURCE[0]}")/base_guard.sh"
nokosahen_base_guard start_persistent_base_app.sh

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

# ROSのsetup.bashは未定義変数を参照することがあるため、source中だけは
# nounset(set -u)を無効にしておく(set -euしていると誤爆して途中終了する)。
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
source ~/mirs2603_nokosahen/ros2_ws/install/setup.bash

nohup ros2 launch nokosahen_bringup persistent_base_app.launch.py \
    > ~/.nokosahen_persistent_base_app.log 2>&1 < /dev/null &
disown

echo "起動コマンドを投入しました(Jetson B: アプリ・表示系)。"
echo "ログ: ~/.nokosahen_persistent_base_app.log"
echo "対になるJetson Aで start_persistent_base_control.sh も忘れずに起動してください。"
