#!/usr/bin/env bash
# rplidar_ros(LiDARドライバ)をSlamtec公式リポジトリからソースビルドする。
#
# 【なぜaptではなくソースビルドか】
# aptの`ros-<distro>-rplidar-ros`(2026-07時点、Jazzy向け2.1.0)が提供する
# 'rplidar_composition'実行ファイルは、RPLIDAR S1でstartMotor()呼び出し時に
# SEGV(セグメンテーションフォールト)でクラッシュするバグがあることを
# Jetson Orin Nano実機(JetPack Ubuntu 24.04 + ROS 2 Jazzy)で確認した
# (gdbのバックトレースでは legacy な rplidar_driver.cpp 側の
# RPlidarDriverImplCommon::startMotor() 内で不正なmutexポインタを踏んでいた)。
# Slamtec公式リポジトリの`ros2`ブランチ(新しいsl_lidar_driver SDK経由の
# 'rplidar_node'実行ファイル)をソースビルドすると、この問題を回避できる
# ことを同じ実機で確認済み。
#
# この問題がA1/A2/A3等の他モデルでも起きるかは未確認(S1でのみ確認)。
# 将来aptパッケージ側が修正された場合は、この手順は不要になる可能性がある。
#
# 使い方:
#   bash scripts/install_rplidar_from_source.sh [インストール先ディレクトリ(既定: ~/lidar_src_build)]
#
# 使う側(launch実行前)に必要なこと:
#   source /opt/ros/<distro>/setup.bash
#   source <インストール先>/install/setup.bash   # ← これを追加でsourceする
#   source ~/mirs2603_nokosahen/ros2_ws/install/setup.bash
#   ros2 launch nokosahen_bringup lidar_test.launch.py ...
set -euo pipefail

INSTALL_DIR="${1:-${HOME}/lidar_src_build}"
ROS_DISTRO="${ROS_DISTRO:-$(ls /opt/ros/ 2>/dev/null | head -n1)}"

if [[ -z "${ROS_DISTRO}" ]]; then
  echo "ROS_DISTROを検出できませんでした。手動で環境変数ROS_DISTROを指定してください。" >&2
  exit 1
fi

echo "=== rplidar_ros(Slamtec公式、ros2ブランチ)を ${INSTALL_DIR} にソースビルド ==="
mkdir -p "${INSTALL_DIR}/src"
cd "${INSTALL_DIR}/src"

if [[ -d rplidar_ros ]]; then
  echo "rplidar_rosは既に存在するため、最新化のみ行います。"
  git -C rplidar_ros fetch origin ros2 --depth 1
  git -C rplidar_ros reset --hard FETCH_HEAD
else
  git clone --branch ros2 --depth 1 https://github.com/Slamtec/rplidar_ros.git
fi

cd "${INSTALL_DIR}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
colcon build --symlink-install

echo
echo "=== 完了 ==="
echo "以後、nokosahen_bringupのlaunch実行前に以下もsourceしてください:"
echo "  source ${INSTALL_DIR}/install/setup.bash"
