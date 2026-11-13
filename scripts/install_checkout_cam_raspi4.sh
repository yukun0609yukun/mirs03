#!/usr/bin/env bash
# 決済横カメラユニット(Webカメラ+Raspberry Pi 4、メニュー1つにつき1台)を
# セットアップするための一括インストールスクリプト。
#
# 設計方針: camera_id(自機の識別)はCPUシリアル番号から自動で決まり、
# 担当メニューも起動後にJetson側の完成版ダッシュボードから指定する仕組み
# なので、3台のRaspberry Piすべてに「全く同じリポジトリ・このスクリプトを
# そのまま1回流すだけ」でセットアップできる(個体ごとの書き換え不要)。
#
# 前提:
# - Raspberry Pi 4にUbuntu Server 22.04(64bit/arm64)を書き込み済みで、
#   このリポジトリを(例)~/mirs2603_nokosahen にクローン/コピー済みであること。
#   Raspberry Pi OS(Debian)ではなくUbuntuを使うのは、本体側の
#   install_robot_base_ubuntu.shと同じくROS 2 Humbleの公式aptパッケージが
#   Ubuntu向けだから。
# - このスクリプトの実行中だけは、人物検出モデルのダウンロードのため
#   インターネットに出られるネットワーク(自宅wifi等)に接続しておくこと。
#   本番で使う「決済カメラ専用のオフラインルーター」はインターネットに
#   出られない前提のため、モデル取得はこのセットアップ時に済ませておく。
#
# 使い方:
#   sudo bash scripts/install_checkout_cam_raspi4.sh
#
# 任意の環境変数(指定すると本番用オフラインルーターへの接続も自動設定する):
#   CHECKOUT_WIFI_SSID=nokosahen-checkout-net \
#   CHECKOUT_WIFI_PASSWORD=xxxxxxxx \
#   sudo -E bash scripts/install_checkout_cam_raspi4.sh
#
# 実行後にすること:
#   1. (Wi-Fi環境変数を指定しなかった場合)決済カメラ専用オフラインルーターへ
#      手動で接続を切り替える: sudo nmcli device wifi connect "<SSID>" password "<パスワード>"
#   2. sudo reboot
#   3. Jetson側の完成版ダッシュボード(http://<JetsonのIP>:8080)を開き、
#      「決済場所カメラ」欄にこのユニットが表示されたら担当メニューを選ぶ。
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "root権限が必要です。'sudo bash $0' で実行してください。" >&2
  exit 1
fi

ROS_DISTRO="${ROS_DISTRO:-humble}"
TARGET_USER="${SUDO_USER:-$(whoami)}"
TARGET_HOME="$(getent passwd "${TARGET_USER}" | cut -d: -f6)"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${REPO_DIR}/ros2_ws"
MODEL_DIR="${TARGET_HOME}/.nokosahen_models"

echo "=== 1. apt更新 ==="
apt-get update
apt-get upgrade -y

echo "=== 2. ROS 2 ${ROS_DISTRO} apt リポジトリ登録 ==="
apt-get install -y curl gnupg lsb-release software-properties-common
if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
  add-apt-repository -y universe
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    > /etc/apt/sources.list.d/ros2.list
  apt-get update
fi

echo "=== 3. パッケージ導入(ROS 2 base + カメラ/画像処理) ==="
apt-get install -y \
  "ros-${ROS_DISTRO}-ros-base" \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  python3-opencv \
  v4l-utils \
  network-manager \
  build-essential \
  git

if ! rosdep --version >/dev/null 2>&1; then
  echo "rosdep が見つかりません" >&2
  exit 1
fi
rosdep init 2>/dev/null || true
sudo -u "${TARGET_USER}" rosdep update

echo "=== 4. 人物検出モデル(YOLOv8)導入 ==="
echo "この手順はインターネット接続が必要です(オフラインルーターではなく一時的に別のネットワークへ接続しておいてください)。"
# Raspberry Pi(arm64)向けのtorchホイールが無い等でここが失敗しても、
# ultralytics未導入時はモーション差分の簡易フォールバックで動作を継続できる
# 設計(checkout_camera_node.py)のため、setup全体は止めずに警告のみ出す。
YOLO_READY=1
if ! sudo -u "${TARGET_USER}" python3 -m pip install --user --upgrade ultralytics; then
  echo "!! ultralyticsの導入に失敗しました(Raspberry Pi向けのtorchホイールが無い等)。" >&2
  echo "!! このユニットはYOLOなしのモーション差分フォールバックで動作します(精度は落ちます)。" >&2
  echo "!! 後で手動インストールする場合: python3 -m pip install --user ultralytics" >&2
  YOLO_READY=0
fi

sudo -u "${TARGET_USER}" mkdir -p "${MODEL_DIR}"
if [[ "${YOLO_READY}" -eq 1 && ! -f "${MODEL_DIR}/yolov8n.pt" ]]; then
  if ! sudo -u "${TARGET_USER}" python3 - "${MODEL_DIR}/yolov8n.pt" <<'PYEOF'
import shutil
import sys
from pathlib import Path

from ultralytics import YOLO

dst = Path(sys.argv[1])
YOLO("yolov8n.pt")  # カレントディレクトリへ初回自動ダウンロードさせる
src = Path("yolov8n.pt")
if src.exists():
    shutil.copy(src, dst)
    print(f"モデルを {dst} に保存しました")
else:
    print("!! yolov8n.ptのダウンロードに失敗した可能性があります", file=sys.stderr)
PYEOF
  then
    echo "!! YOLOv8モデルの取得に失敗しました。model_pathが見つからない場合は起動時にフォールバック動作へ切り替わります。" >&2
  fi
elif [[ "${YOLO_READY}" -eq 1 ]]; then
  echo "モデルは既に ${MODEL_DIR}/yolov8n.pt にあるためダウンロードをスキップ"
fi

echo "=== 5. ワークスペースのビルド(nokosahen_checkoutのみ) ==="
sudo -u "${TARGET_USER}" bash -lc "cd '${WS_DIR}' && source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --packages-up-to nokosahen_checkout --symlink-install"

echo "=== 6. 環境変数(.bashrc) ==="
BASHRC="${TARGET_HOME}/.bashrc"
grep -qxF "source /opt/ros/${ROS_DISTRO}/setup.bash" "${BASHRC}" || echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "${BASHRC}"
grep -qxF "source ${WS_DIR}/install/setup.bash" "${BASHRC}" || echo "source ${WS_DIR}/install/setup.bash" >> "${BASHRC}"
grep -qxF "export ROS_DOMAIN_ID=77" "${BASHRC}" || echo "export ROS_DOMAIN_ID=77" >> "${BASHRC}"

usermod -aG dialout "${TARGET_USER}"
usermod -aG video "${TARGET_USER}"

echo "=== 7. Wi-Fi(決済カメラ専用オフラインルーター)への接続設定 ==="
if [[ -n "${CHECKOUT_WIFI_SSID:-}" ]]; then
  if command -v nmcli >/dev/null 2>&1; then
    nmcli device wifi connect "${CHECKOUT_WIFI_SSID}" password "${CHECKOUT_WIFI_PASSWORD:-}" \
      || echo "!! Wi-Fi接続に失敗しました。後で手動で 'sudo nmcli device wifi connect \"${CHECKOUT_WIFI_SSID}\" password \"...\"' を実行してください。" >&2
  else
    echo "!! nmcliが見つかりません。手動でWi-Fi設定を行ってください。" >&2
  fi
else
  echo "CHECKOUT_WIFI_SSID未指定のためWi-Fi自動設定はスキップしました。"
  echo "後で手動で: sudo nmcli device wifi connect \"<SSID>\" password \"<パスワード>\""
fi

echo "=== 8. systemdサービス化(電源投入で自動起動) ==="
SERVICE_FILE="/etc/systemd/system/nokosahen-checkout-camera.service"
cat > "${SERVICE_FILE}" <<EOF
[Unit]
Description=Nokosahen checkout camera unit (per-menu Raspberry Pi 4)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${TARGET_USER}
Environment=ROS_DOMAIN_ID=77
ExecStart=/bin/bash -lc 'source /opt/ros/${ROS_DISTRO}/setup.bash && source ${WS_DIR}/install/setup.bash && ros2 launch nokosahen_checkout checkout_camera.launch.py'
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable nokosahen-checkout-camera.service

echo
echo "=== 完了 ==="
echo "モデルファイル: ${MODEL_DIR}/yolov8n.pt (config/checkout_cam.yamlのmodel_pathが指す既定の場所)"
echo "ログアウト/再起動後、'systemctl status nokosahen-checkout-camera' で自動起動を確認してください。"
echo "Webカメラは/dev/video*を自動探索するため設定不要、担当メニューはJetsonの完成版ダッシュボードから割り当ててください。"
