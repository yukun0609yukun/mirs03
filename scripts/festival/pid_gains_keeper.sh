#!/usr/bin/env bash
# ESP32のPIDゲインを文化祭用の値に保つ見張り(2026-11-06)。festival_start.sh が裏で起動する。ログ ~/.nokosahen_pid_gains_keeper.log
# 中身は pid_gains_keeper.py(目標の値・理由・元に戻し方はそちらの説明)。ROS 2 の環境を読み込んでから実行する。
set -u
REPO="${REPO:-$HOME/mirs2603_nokosahen}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$REPO/ros2_ws/install/setup.bash" 2>/dev/null
set -u
exec python3 "$REPO/scripts/festival/pid_gains_keeper.py" "$@" >> "$HOME/.nokosahen_pid_gains_keeper.log" 2>&1
