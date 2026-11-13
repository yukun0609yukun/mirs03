#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"

if [[ "${ROS_DISTRO}" != "humble" && "${ROS_DISTRO}" != "jazzy" ]]; then
  echo "ROS_DISTRO must be humble or jazzy" >&2
  exit 1
fi

sudo apt update
sudo apt install -y \
  curl \
  gnupg \
  lsb-release \
  software-properties-common

sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

sudo apt update
sudo apt install -y \
  "ros-${ROS_DISTRO}-ros-base" \
  "ros-${ROS_DISTRO}-navigation2" \
  "ros-${ROS_DISTRO}-nav2-bringup" \
  "ros-${ROS_DISTRO}-nav2-map-server" \
  "ros-${ROS_DISTRO}-slam-toolbox" \
  "ros-${ROS_DISTRO}-rplidar-ros" \
  "ros-${ROS_DISTRO}-tf2-ros" \
  "ros-${ROS_DISTRO}-robot-state-publisher" \
  "ros-${ROS_DISTRO}-xacro" \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pip \
  python3-opencv \
  build-essential \
  git \
  v4l-utils

if ! rosdep --version >/dev/null 2>&1; then
  echo "rosdep is not available" >&2
  exit 1
fi

sudo rosdep init 2>/dev/null || true
rosdep update

grep -qxF "source /opt/ros/${ROS_DISTRO}/setup.bash" "${HOME}/.bashrc" \
  || echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "${HOME}/.bashrc"
grep -qxF "export ROS_DOMAIN_ID=77" "${HOME}/.bashrc" \
  || echo "export ROS_DOMAIN_ID=77" >> "${HOME}/.bashrc"

sudo usermod -aG dialout "${USER}"

echo
echo "Installed ROS 2 ${ROS_DISTRO} base packages."
echo "Log out/in once so the dialout group is applied."

