#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${REPO_DIR}/ros2_ws"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
cd "${WS_DIR}"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

echo
echo "Build complete."
echo "Run: source ${WS_DIR}/install/setup.bash"

