#!/usr/bin/env bash
# Jetson Orin Nano のホストOS(JetPack、Dockerの外側)に必要なソフトを
# 一括インストールするスクリプト。
#
# 対象: Docker Engine, NVIDIA Container Toolkit(GPUパススルー用),
#       Ollama, VOICEVOX Engine(aarch64), voice_bridgeのPython依存関係。
#
# 使い方:
#   sudo bash scripts/install_jetson_host.sh
#
# 注意:
# - JetPack本体の書き込み(SDK Manager等でのOSイメージ書き込み)はこの
#   スクリプトの対象外。先にJetPackを書き込み、起動できる状態にしてから使う。
# - VOICEVOX Engineのaarch64対応状況は本プロジェクト作成時点で未検証。
#   このスクリプトは最新リリースからlinux-arm64向けアセットを自動検出して
#   ダウンロードを試みるが、見つからない場合はその旨を表示して手動確認を促す。
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "root権限が必要です。'sudo bash $0' で実行してください。" >&2
  exit 1
fi

TARGET_USER="${SUDO_USER:-$(whoami)}"
INSTALL_DIR="/opt/voicevox_engine"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=== 1. apt更新 ==="
apt-get update
apt-get upgrade -y

echo "=== 2. Docker Engine ==="
if ! command -v docker >/dev/null 2>&1; then
  curl -fsSL https://get.docker.com | sh
else
  echo "docker は既にインストール済みのためスキップ"
fi
usermod -aG docker "${TARGET_USER}"
systemctl enable --now docker

echo "=== 3. NVIDIA Container Toolkit(GPUパススルー用) ==="
# JetPackのapt sourcesに含まれている前提。無い場合はNVIDIAのJetson向け
# 手順(SDK Manager経由の"Nvidia Container Runtime"コンポーネント)を先に導入する。
if apt-cache show nvidia-container-toolkit >/dev/null 2>&1; then
  apt-get install -y nvidia-container-toolkit
elif apt-cache show nvidia-container-runtime >/dev/null 2>&1; then
  apt-get install -y nvidia-container-runtime
else
  echo "!! nvidia-container-toolkit / nvidia-container-runtime がapt候補に無い" >&2
  echo "!! JetPack SDK Managerの'Nvidia Container Runtime'コンポーネントを先に導入してください" >&2
fi

DAEMON_JSON="/etc/docker/daemon.json"
if [[ ! -f "${DAEMON_JSON}" ]] || ! grep -q '"nvidia"' "${DAEMON_JSON}" 2>/dev/null; then
  mkdir -p /etc/docker
  cat > "${DAEMON_JSON}" <<'EOF'
{
  "runtimes": {
    "nvidia": {
      "path": "nvidia-container-runtime",
      "runtimeArgs": []
    }
  },
  "default-runtime": "nvidia"
}
EOF
  systemctl restart docker
fi

echo "docker info で nvidia ランタイムが見えるか確認してください: docker info | grep -i runtime"

echo "=== 4. Ollama ==="
if ! command -v ollama >/dev/null 2>&1; then
  curl -fsSL https://ollama.com/install.sh | sh
else
  echo "ollama は既にインストール済みのためスキップ"
fi
systemctl enable --now ollama || true
ollama pull gemma2:2b

echo "=== 5. VOICEVOX Engine(aarch64、要検証) ==="
mkdir -p "${INSTALL_DIR}"
RELEASE_JSON="$(curl -fsSL https://api.github.com/repos/VOICEVOX/voicevox_engine/releases/latest || true)"
ASSET_URL="$(echo "${RELEASE_JSON}" | grep -oE '"browser_download_url":\s*"[^"]*linux-arm64[^"]*"' | grep -oE 'https://[^"]+' | head -n1 || true)"

if [[ -n "${ASSET_URL}" ]]; then
  echo "検出したアセット: ${ASSET_URL}"
  TMP_ZIP="$(mktemp --suffix=.zip)"
  curl -fsSL -o "${TMP_ZIP}" "${ASSET_URL}"
  unzip -o "${TMP_ZIP}" -d "${INSTALL_DIR}"
  rm -f "${TMP_ZIP}"
  chown -R "${TARGET_USER}:${TARGET_USER}" "${INSTALL_DIR}"
  echo "VOICEVOX Engineを ${INSTALL_DIR} に展開しました。実行ファイル名を確認して起動してください。"
  echo "例: ${INSTALL_DIR}/run --host 127.0.0.1 --port 50021"
else
  echo "!! linux-arm64向けのVOICEVOX Engineリリースアセットが自動検出できませんでした。" >&2
  echo "!! https://github.com/VOICEVOX/voicevox_engine/releases を開いて、" >&2
  echo "!! Jetson(aarch64)向けのビルドが提供されているか手動で確認してください。" >&2
  echo "!! 提供が無い場合は docs/setup_docker.md のトラブルシュートを参照。" >&2
fi

echo "=== 6. voice_bridge の Python依存関係 ==="
apt-get install -y python3-pip
python3 -m pip install -r "${REPO_DIR}/voice_bridge/requirements.txt"

echo "=== 完了 ==="
echo "一度ログアウト/ログインし直して、dockerグループが反映されているか確認してください。"
echo "次は scripts/setup_udev_rules.sh を実行してUSB機器を固定名にしてください。"
