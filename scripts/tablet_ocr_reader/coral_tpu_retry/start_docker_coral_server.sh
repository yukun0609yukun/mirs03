#!/usr/bin/env bash
# Edge TPU推論を常駐のDockerコンテナ(公式pycoralが動くDebian bullseye環境)で
# 行うためのサーバーを起動する。tablet_ocr_reader.py側は
# recognizer: "edgetpu_docker" にしておけば、このサーバーへHTTPで1桁ずつ
# 問い合わせるようになる(dashboard_common.pyとは無関係、タブレットOCR単体の
# 仕組み)。
#
# 使い方:
#   bash start_docker_coral_server.sh          # ビルド+起動
#   bash start_docker_coral_server.sh stop     # 停止
#   bash start_docker_coral_server.sh logs     # ログ表示
set -euo pipefail

MODEL_DIR="$HOME/tablet_ocr_reader/digit_model"
MODEL_FILE="digit_model_int8_edgetpu.tflite"
CONTAINER_NAME="nokosahen-coral-server"
IMAGE_NAME="nokosahen-coral-server"
PORT=5005
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ACTION="${1:-start}"

if [ "$ACTION" = "stop" ]; then
    docker rm -f "$CONTAINER_NAME" 2>/dev/null || true
    echo "停止しました。"
    exit 0
fi

if [ "$ACTION" = "logs" ]; then
    docker logs -f "$CONTAINER_NAME"
    exit 0
fi

if ! command -v docker >/dev/null 2>&1; then
    echo "エラー: Dockerが見つかりません(coral_tpu_retry/README.md参照)。"
    exit 1
fi

if [ ! -f "$MODEL_DIR/$MODEL_FILE" ]; then
    echo "エラー: $MODEL_DIR/$MODEL_FILE が見つかりません。"
    exit 1
fi

echo "=== イメージをビルド(try_docker_coral.shと同じベース、常駐サーバー版) ==="
cat > /tmp/Dockerfile.coral_server <<'EOF'
FROM debian:bullseye-slim

RUN echo "deb [check-valid-until=no] http://archive.debian.org/debian bullseye main" \
    > /etc/apt/sources.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl ca-certificates python3 python3-pip python3-numpy \
    && rm -rf /var/lib/apt/lists/*

RUN echo "deb [trusted=yes] https://packages.cloud.google.com/apt coral-edgetpu-stable main" \
    > /etc/apt/sources.list.d/coral-edgetpu.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends libedgetpu1-std python3-pycoral \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
COPY server.py /work/server.py
EXPOSE 5005
CMD ["python3", "/work/server.py"]
EOF

docker build -t "$IMAGE_NAME" -f /tmp/Dockerfile.coral_server "$SCRIPT_DIR"

echo
echo "=== 既存のコンテナがあれば止めてから、常駐サーバーとして起動 ==="
docker rm -f "$CONTAINER_NAME" 2>/dev/null || true
docker run -d --name "$CONTAINER_NAME" \
    --restart unless-stopped \
    --privileged \
    -v /dev/bus/usb:/dev/bus/usb \
    -v "$MODEL_DIR/$MODEL_FILE":/work/model.tflite:ro \
    -p 127.0.0.1:$PORT:$PORT \
    "$IMAGE_NAME"

echo
echo "=== 起動確認(数秒待ってから/healthを叩く) ==="
for i in 1 2 3 4 5 6 7 8 9 10; do
    sleep 1
    if curl -s "http://127.0.0.1:$PORT/health" | grep -q '"ok": true'; then
        echo "OK: 推論サーバーが起動しました(http://127.0.0.1:$PORT)"
        echo "tablet_ocr_reader.pyの設定で recognizer: \"edgetpu_docker\" にすると使われます。"
        exit 0
    fi
done

echo "警告: 起動確認がタイムアウトしました。'docker logs $CONTAINER_NAME' でログを確認してください。"
exit 1
