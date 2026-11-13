#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /opt/micro_ros_ws/install/setup.bash
if [ -f /workspace/ros2_ws/install/setup.bash ]; then
  source /workspace/ros2_ws/install/setup.bash
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

exec "$@"
