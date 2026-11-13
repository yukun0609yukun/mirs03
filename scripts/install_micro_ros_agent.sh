#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
MICROROS_WS="${HOME}/microros_ws"

# ROS 2のsetup.bashは内部で未設定の変数を参照することがあり、set -u(nounset)
# と組み合わせると「unbound variable」で落ちる。source中だけ一時的に無効化する。
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

mkdir -p "${MICROROS_WS}/src"
cd "${MICROROS_WS}"

if [[ ! -d src/micro_ros_setup ]]; then
  git clone -b "${ROS_DISTRO}" https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
fi

sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
set +u
source install/local_setup.bash
set -u

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh

grep -qxF "source ${MICROROS_WS}/install/local_setup.bash" "${HOME}/.bashrc" \
  || echo "source ${MICROROS_WS}/install/local_setup.bash" >> "${HOME}/.bashrc"

echo
echo "micro-ROS Agent installed."
echo "Run: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200"

