#!/usr/bin/env bash
# Jetson #2 (Orin Nano Super, Ubuntu 24.04 / L4T R39.2.1 / JetPack 7) 用 root必須セットアップ。
# 使い方(Jetson #2上で1回だけ):   sudo bash setup_jetson2_sudo.sh
# 何度実行しても安全(冪等)。各ステップは "=== [STEP n/N] ..." で区切って表示する。
#
# オプション(環境変数):
#   SET_MAXN=1        nvpmodelをMAXN_SUPER(ID=2)にし、jetson_clocksも実行(既定は触らない、25Wのまま)
#   INSTALL_TRT=1     TensorRT(python3-libnvinfer等)とCUDA toolkit(nvcc)もaptで入れる(任意、後述)
#   WITH_NAV2=1       Nav2/slam_toolbox/rplidar_ros/xacro等も入れる(既定は入れる=1。不要なら0)
#   SKIP_DOCKER_GROUP=1  docker グループ追加を省略
#
# 【前提】ユーザー側(venv・PyTorch・ultralytics・YOLO)は sudo不要の範囲で導入済み(~/nokosahen_venv)。
# このスクリプトはそれ以外(ROS 2 Jazzy本体、colcon、swap、WiFi省電力OFF等)だけを担当する。
set -euo pipefail

if [[ $EUID -ne 0 ]]; then echo "sudo bash $0 で実行してください" >&2; exit 1; fi
TARGET_USER="${SUDO_USER:-mirs2603}"
TARGET_HOME="$(getent passwd "$TARGET_USER" | cut -d: -f6)"
N=9
step() { echo; echo "=== [STEP $1/$N] $2 ==="; }
export DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------------------
step 1 "ロケール(UTF-8)と基本パッケージ・universeリポジトリ"
apt-get update
apt-get install -y locales software-properties-common curl gnupg lsb-release ca-certificates
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
add-apt-repository -y universe

# ---------------------------------------------------------------------------
step 2 "ROS 2 apt source (ros2-apt-source パッケージ方式, 公式手順)"
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')"
  [[ -n "$ROS_APT_SOURCE_VERSION" ]] || { echo "ros-apt-sourceの最新バージョン取得に失敗(GitHub API制限?)。数分後に再実行" >&2; exit 1; }
  CODENAME="$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})"   # noble
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
  dpkg -i /tmp/ros2-apt-source.deb
else
  echo "ros2-apt-source は導入済み"
fi
apt-get update

# ---------------------------------------------------------------------------
step 3 "ROS 2 Jazzy ros-base + colcon + 開発ツール + Pythonのvenv/pip"
# python3-venv/python3-pip: 今回Jetson #2には無かった(ensurepip欠落)ため追加。
apt-get install -y ros-jazzy-ros-base ros-dev-tools \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  python3-venv python3-pip python3-opencv build-essential git v4l-utils rsync libportaudio2 alsa-utils

# ---------------------------------------------------------------------------
step 4 "プロジェクトが依存するROSパッケージ(package.xml / install_robot_base_ubuntu.sh と同等)"
# nokosahen_core: geometry_msgs nav_msgs nav2_msgs rclpy sensor_msgs std_msgs tf2_ros
# nokosahen_bringup(exec): nav2_bringup slam_toolbox rplidar_ros tf2_ros (micro_ros_agentはESP32直結の1号機側。2号機では不要)
# ros-baseに geometry/sensor/std/nav_msgs/tf2_ros/rclpy は含まれる。nav2_msgsは別。
apt-get install -y ros-jazzy-nav2-msgs ros-jazzy-cv-bridge ros-jazzy-image-transport \
  ros-jazzy-rmw-cyclonedds-cpp python3-requests
if [[ "${WITH_NAV2:-1}" == "1" ]]; then
  apt-get install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server \
    ros-jazzy-slam-toolbox ros-jazzy-rplidar-ros ros-jazzy-robot-state-publisher ros-jazzy-xacro
fi
# rosdep初期化(失敗しても致命的ではない)
[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || rosdep init || true
sudo -u "$TARGET_USER" rosdep update || true

# ---------------------------------------------------------------------------
step 5 "8GB swapファイル(RAM 7.3GB・swap無しのため。YOLO/colcon build中のOOM対策)"
if ! swapon --show | grep -q '/swapfile'; then
  if [[ ! -f /swapfile ]]; then
    fallocate -l 8G /swapfile || dd if=/dev/zero of=/swapfile bs=1M count=8192 status=progress
    chmod 600 /swapfile
    mkswap /swapfile
  fi
  swapon /swapfile
fi
grep -q '^/swapfile' /etc/fstab || echo '/swapfile none swap sw 0 0' >> /etc/fstab
# swapを使いすぎてGPU推論が遅くならないよう、控えめに
echo 'vm.swappiness=10' > /etc/sysctl.d/99-nokosahen-swap.conf
sysctl -p /etc/sysctl.d/99-nokosahen-swap.conf >/dev/null
swapon --show

# ---------------------------------------------------------------------------
step 6 "WiFi省電力(power save)をOFF(定期切断対策, wlP1p1s0)"
iw dev wlP1p1s0 set power_save off || true
# NetworkManagerの現在アクティブなWiFi接続に恒久設定(802-11-wireless.powersave 2 = disable)
WIFI_CON="$(nmcli -t -f NAME,TYPE con show --active | awk -F: '$2=="802-11-wireless"{print $1; exit}')"
if [[ -n "${WIFI_CON:-}" ]]; then
  nmcli con modify "$WIFI_CON" 802-11-wireless.powersave 2
  echo "接続 '$WIFI_CON' に powersave=disable を設定"
fi
# 他のWiFi接続にも効くよう全体既定も置く
mkdir -p /etc/NetworkManager/conf.d
printf '[connection]\nwifi.powersave = 2\n' > /etc/NetworkManager/conf.d/99-nokosahen-wifi-powersave-off.conf
iw dev wlP1p1s0 get power_save || true

# ---------------------------------------------------------------------------
step 7 "ユーザーグループ(docker / dialout)"
# dialout: ESP32/LiDARシリアル、docker: Dockerをsudo無しで(2号機をDocker運用する場合のみ必要)
usermod -aG dialout "$TARGET_USER"
if [[ "${SKIP_DOCKER_GROUP:-0}" != "1" ]]; then usermod -aG docker "$TARGET_USER"; fi
echo "(グループ反映には再ログインが必要)"

# ---------------------------------------------------------------------------
step 8 "~/.bashrc へ ROS環境変数(ROS_DOMAIN_ID=77, source)を追記"
BRC="$TARGET_HOME/.bashrc"
add_line() { grep -qxF "$1" "$BRC" || echo "$1" >> "$BRC"; }
add_line "source /opt/ros/jazzy/setup.bash"
add_line "export ROS_DOMAIN_ID=77"
# ※ RMWは1号機と必ず同じにすること(1号機が既定FastDDSなら下行は入れない)。入れていないのは意図的。
# add_line "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
chown "$TARGET_USER":"$TARGET_USER" "$BRC"

# ---------------------------------------------------------------------------
step 9 "任意: 電源モード(MAXN_SUPER)・TensorRT/CUDA toolkit"
if [[ "${SET_MAXN:-0}" == "1" ]]; then
  nvpmodel -m 2 || true      # 2 = MAXN_SUPER (0=15W,1=25W)
  jetson_clocks || true
  nvpmodel -q
else
  echo "SET_MAXN未指定: 電源モードは変更しません(現在: $(nvpmodel -q 2>/dev/null | head -1))"
fi
if [[ "${INSTALL_TRT:-0}" == "1" ]]; then
  # 現状のPyTorch(pip同梱CUDA13)だけでYOLOはGPUで動くので必須ではない。
  # TensorRT(.engine化)で更に速くしたい場合に入れる。容量大(数GB)。
  apt-get install -y nvidia-jetpack-runtime python3-libnvinfer libnvinfer-bin cuda-toolkit-13-2 || \
    apt-get install -y python3-libnvinfer libnvinfer-bin
else
  echo "INSTALL_TRT未指定: TensorRT/CUDA toolkitは入れません"
fi

echo
echo "=== 完了。再ログイン(または再起動)後、'source /opt/ros/jazzy/setup.bash; ros2 --help' で確認 ==="
