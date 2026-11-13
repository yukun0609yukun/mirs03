#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
ESP32_PORT="${ESP32_PORT:-/dev/ttyUSB0}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${REPO_DIR}/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

ros2 launch nokosahen_bringup raspi4_robot.launch.py esp32_port:="${ESP32_PORT}"

