#!/usr/bin/env bash
# 戦略1: Dockerコンテナ内に「libedgetpuが公式サポートしていた頃のOS」を
# 再現し、その中で公式pycoralを動かす。ホストのラズパイ本体(Debian 13)は
# 一切変更しない。失敗してもコンテナを消せば元通り。
#
# 使い方:
#   bash try_docker_coral.sh
#
# 事前準備: Dockerが必要。無ければ以下でインストール(sudoパスワードが要る):
#   curl -fsSL https://get.docker.com | sudo sh
#   sudo usermod -aG docker $USER   # 実行後、一度ログアウト/ログインが必要
set -euo pipefail

MODEL_DIR="$HOME/tablet_ocr_reader/digit_model"
MODEL_FILE="digit_model_int8_edgetpu.tflite"

if ! command -v docker >/dev/null 2>&1; then
    echo "エラー: Dockerが見つかりません。上記コメントの手順でインストールしてから再実行してください。"
    exit 1
fi

if [ ! -f "$MODEL_DIR/$MODEL_FILE" ]; then
    echo "エラー: $MODEL_DIR/$MODEL_FILE が見つかりません。"
    echo "先に scripts/tablet_ocr_reader/train_digit_model.py で変換・edgetpu_compilerでコンパイルしたモデルを"
    echo "~/tablet_ocr_reader/digit_model/ に置いてください。"
    exit 1
fi

echo "=== Debian bullseye(arm64)ベースのコンテナで公式pycoral環境を構築 ==="

cat > /tmp/Dockerfile.coral_test <<'EOF'
FROM debian:bullseye-slim

# 【注意】bullseyeは既にEOL(サポート終了)のため、通常のミラー(deb.debian.org)
# からは404になる。archive.debian.orgへ向け直す必要があるが、
# bullseye-security側は実機検証の結果archive.debian.org上でも404/依存関係の
# バージョン不整合を起こすことが分かった(security更新は今回の使い捨て
# 検証コンテナには不要)。security/updatesは使わず、mainの素のbullseyeだけを
# 使うシンプルな1行構成に置き換える。
RUN echo "deb [check-valid-until=no] http://archive.debian.org/debian bullseye main" \
    > /etc/apt/sources.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl ca-certificates python3 python3-pip python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# CoralのGPG鍵検証にgnupgを入れる代わりに、この1リポジトリだけ
# [trusted=yes]で検証をスキップする(HTTPS経由でGoogle公式ドメインから
# 直接取得するため、リスクは限定的と判断)。
RUN echo "deb [trusted=yes] https://packages.cloud.google.com/apt coral-edgetpu-stable main" \
    > /etc/apt/sources.list.d/coral-edgetpu.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends libedgetpu1-std python3-pycoral \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
COPY probe.py /work/probe.py
CMD ["python3", "/work/probe.py"]
EOF

cat > /tmp/probe.py <<'EOF'
import sys
from pycoral.utils.edgetpu import make_interpreter
import numpy as np

model_path = "/work/model.tflite"
interp = make_interpreter(model_path)
interp.allocate_tensors()
inp = interp.get_input_details()[0]
dummy = np.zeros(inp["shape"], dtype=inp["dtype"])
interp.set_tensor(inp["index"], dummy)
interp.invoke()
print("PROBE_OK (Docker + 公式pycoral)")
EOF

docker build -t nokosahen-coral-test -f /tmp/Dockerfile.coral_test /tmp

echo
echo "=== コンテナ内でEdge TPU推論を試す(USBデバイスをパススルー) ==="
# --privileged はUSBの抜き差し(モード切り替え時の再enumerate)にも対応できるよう
# 広めに権限を渡す簡易版。うまくいったら --device=/dev/bus/usb/XXX/YYY まで
# 絞り込んでよい。
docker run --rm --privileged \
    -v /dev/bus/usb:/dev/bus/usb \
    -v "$MODEL_DIR/$MODEL_FILE":/work/model.tflite:ro \
    nokosahen-coral-test

echo
echo "=== 結果 ==="
echo "上に 'PROBE_OK' が出ていれば、Docker経由でEdge TPUが使えることが確認できました。"
echo "その場合、tablet_ocr_reader.py本体からもこのDockerコンテナを"
echo "呼び出す(subprocess経由でコンテナ内スクリプトに画像を渡して推論結果を受け取る)"
echo "形に作り直す必要があります(相談してから着手します)。"
