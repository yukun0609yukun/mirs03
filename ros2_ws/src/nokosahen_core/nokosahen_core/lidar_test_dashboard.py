"""LiDAR・Webカメラ単体の動作確認用、最小構成のダッシュボード。

ESP32・SLAM・Nav2等、他の何も起動していない状態でも、`/scan`が来ているか・
Webカメラの映像が撮れているかを素早くブラウザで確認したい、という要望への
対応。`firmware`や`nokosahen_bringup`のフル構成を待たずに、LiDARドライバ
(rplidar_ros)を起動しただけで単体テストできる
(`nokosahen_bringup/launch/lidar_test.launch.py`とセットで使う)。

Webカメラ側はROS2トピックを経由せず、OpenCVで直接`/dev/video*`を開いて
定期的にキャプチャする(person_target_detector等、他ノードが動いていなくても
単体で確認できるようにするため)。

認証は付けていない(ハードウェア単体テスト用の一時的なツールという
位置づけのため)。本番の操作系ダッシュボードとは別ポート(既定8099)。
"""
from __future__ import annotations

import json
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler
from typing import Callable, Optional
from urllib.parse import urlparse

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from .dashboard_common import ScanController, start_dashboard_server


class WebcamPreview:
    """`/dev/video*`を直接開いて定期的にJPEGキャプチャするだけの、
    ROS2に依存しない最小限のWebカメラプレビュー。"""

    def __init__(self, node, camera_index: str, fps: float = 5.0) -> None:
        self.node = node
        self._latest_jpeg: Optional[bytes] = None
        self._lock = threading.Lock()
        self._cap = None
        self.available = False

        try:
            import cv2  # type: ignore

            self._cv2 = cv2
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(f"OpenCV未導入のためWebカメラプレビューは無効: {exc}")
            self._cv2 = None
            return

        index: "str | int" = camera_index
        try:
            index = int(camera_index)
        except ValueError:
            pass

        # デバイスパス文字列だとGStreamerバックエンドが選ばれてこのJetsonのOpenCV
        # ビルドではSEGVするため、V4L2を明示指定する(数値indexでも害はない)。
        cap = self._cv2.VideoCapture(index, self._cv2.CAP_V4L2)
        if not cap.isOpened():
            self.node.get_logger().warn(
                f"Webカメラ({camera_index})を開けませんでした。プレビューは無効のまま起動します。"
            )
            cap.release()
            return

        self._cap = cap
        self.available = True
        interval = 1.0 / max(0.5, fps)
        self._thread = threading.Thread(target=self._loop, args=(interval,), daemon=True)
        self._thread.start()

    def _loop(self, interval: float) -> None:
        while True:
            ok, frame = self._cap.read()
            if ok and frame is not None:
                ok2, encoded = self._cv2.imencode(".jpg", frame, [int(self._cv2.IMWRITE_JPEG_QUALITY), 70])
                if ok2:
                    with self._lock:
                        self._latest_jpeg = encoded.tobytes()
            time.sleep(interval)

    def jpeg_bytes(self) -> Optional[bytes]:
        with self._lock:
            return self._latest_jpeg

HTML = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>のこさへん - LiDAR単体テスト</title>
<style>
  :root { color-scheme: light; font-family: system-ui, -apple-system, "Segoe UI", sans-serif; background:#f4f6f5; color:#16211d; }
  body { margin:0; }
  header { padding:10px 18px; background:#124739; color:#fff; }
  header h1 { margin:0; font-size:18px; }
  main { max-width:900px; margin:0 auto; padding:16px; display:flex; flex-direction:column; gap:14px; align-items:center; }
  .status { width:100%; max-width:420px; padding:14px; border-radius:8px; font-weight:bold; text-align:center; font-size:16px; }
  .status-ok { background:#dff3e6; color:#0f6b3f; }
  .status-stale { background:#fff3cd; color:#8a6400; }
  .status-none { background:#faece7; color:#993c1d; }
  .metrics { display:grid; grid-template-columns:repeat(auto-fit, minmax(140px,1fr)); gap:10px; width:100%; max-width:420px; }
  .metric { background:#fff; border:1px solid #dce4df; border-radius:8px; padding:10px; text-align:center; }
  .metric span { display:block; font-size:12px; color:#5b6b63; }
  .metric b { font-size:18px; }
  canvas { background:#0c1b16; border-radius:8px; }
  .hint { font-size:12px; color:#5b6b63; max-width:420px; text-align:center; }
  #cameraView { width:100%; max-width:420px; background:#0c1b16; border-radius:8px; min-height:200px; object-fit:contain; }
  #cameraStatus { font-size:13px; color:#5b6b63; }
</style>
</head>
<body>
<header><h1>LiDAR/Webカメラ単体テスト(ESP32/SLAM/Nav2は起動していなくてOK)</h1></header>
<main>
  <div id="statusBox" class="status status-none">/scanを待っています...</div>
  <div class="metrics">
    <div class="metric"><span>検出点数</span><b id="pointCount">-</b></div>
    <div class="metric"><span>最小距離(m)</span><b id="minRange">-</b></div>
    <div class="metric"><span>最大距離(m)</span><b id="maxRange">-</b></div>
    <div class="metric"><span>更新頻度(Hz目安)</span><b id="scanHz">-</b></div>
  </div>
  <canvas id="lidarCanvas" width="360" height="360"></canvas>
  <p class="hint">緑の三角がLiDAR自身の向き(前方=画面上向き)、赤い点が検出した障害物。円は1m刻みの目安。</p>

  <h2 style="margin:0;">Webカメラの様子</h2>
  <img id="cameraView" alt="カメラ映像なし" />
  <p id="cameraStatus" class="hint">確認中...</p>
</main>
<script>
  const canvas = document.getElementById("lidarCanvas");
  const ctx = canvas.getContext("2d");
  let lastSignature = null;
  let lastChangeTime = 0;
  let changeTimestamps = [];

  function drawScan(s) {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    const cx = canvas.width / 2, cy = canvas.height / 2;
    const maxRange = Math.min(s.range_max || 6.0, 6.0);
    const scale = Math.min(cx, cy) / maxRange;

    ctx.strokeStyle = "#234339";
    ctx.lineWidth = 1;
    for (let r = 1; r <= maxRange; r += 1) {
      ctx.beginPath();
      ctx.arc(cx, cy, r * scale, 0, Math.PI * 2);
      ctx.stroke();
    }

    ctx.fillStyle = "#ff5a5a";
    let angle = s.angle_min;
    let count = 0;
    let minR = null, maxR = null;
    for (const r of s.ranges) {
      if (r !== null && r <= maxRange) {
        count++;
        if (minR === null || r < minR) minR = r;
        if (maxR === null || r > maxR) maxR = r;
        const px = cx - r * Math.sin(angle) * scale;
        const py = cy - r * Math.cos(angle) * scale;
        ctx.fillRect(px - 1.5, py - 1.5, 3, 3);
      }
      angle += s.angle_increment;
    }

    ctx.fillStyle = "#3fd68c";
    ctx.beginPath();
    ctx.moveTo(cx, cy - 9);
    ctx.lineTo(cx - 6, cy + 6);
    ctx.lineTo(cx + 6, cy + 6);
    ctx.closePath();
    ctx.fill();

    document.getElementById("pointCount").textContent = count;
    document.getElementById("minRange").textContent = minR !== null ? minR.toFixed(2) : "-";
    document.getElementById("maxRange").textContent = maxR !== null ? maxR.toFixed(2) : "-";
  }

  function setStatus(cls, text) {
    const box = document.getElementById("statusBox");
    box.className = "status " + cls;
    box.textContent = text;
  }

  async function fetchScan() {
    try {
      const res = await fetch("/api/scan");
      const s = await res.json();
      if (!s.available) {
        setStatus("status-none", "LiDARからデータが届いていません(未接続、またはドライバ未起動)");
        return;
      }
      // rangesの一部だけを軽量な「署名」として使い、データが本当に更新され
      // 続けているか(=生きているか)を見る。全部比較すると重いため先頭だけ見る。
      const signature = JSON.stringify(s.ranges.slice(0, 20)) + s.ranges.length;
      const now = performance.now();
      if (signature !== lastSignature) {
        lastSignature = signature;
        lastChangeTime = now;
        changeTimestamps.push(now);
        changeTimestamps = changeTimestamps.filter(t => now - t <= 2000);
      }
      const hz = changeTimestamps.length / 2.0;
      document.getElementById("scanHz").textContent = hz.toFixed(1);

      const staleMs = now - lastChangeTime;
      if (staleMs > 3000) {
        setStatus("status-stale", `データが${(staleMs / 1000).toFixed(1)}秒更新されていません(ドライバが止まっている可能性)`);
      } else {
        setStatus("status-ok", "LiDAR正常動作中");
      }
      drawScan(s);
    } catch (e) {
      setStatus("status-none", "サーバーに接続できません");
    }
  }
  setInterval(fetchScan, 200);
  fetchScan();

  const cameraView = document.getElementById("cameraView");
  const cameraStatus = document.getElementById("cameraStatus");
  function fetchCamera() {
    const img = new Image();
    img.onload = () => {
      cameraView.src = img.src;
      cameraStatus.textContent = "カメラ映像を取得できています";
    };
    img.onerror = () => {
      cameraStatus.textContent = "カメラ映像を取得できません(未接続、またはドライバ未起動)";
    };
    img.src = "/api/camera?t=" + Date.now();
  }
  setInterval(fetchCamera, 400);
  fetchCamera();
</script>
</body>
</html>
"""


def make_handler(html_getter: Callable[[], str], scan: ScanController, camera: WebcamPreview):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args) -> None:  # noqa: A003
            pass

        def _send_json(self, status: int, payload: dict) -> None:
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            if path == "/":
                body = html_getter().encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/api/scan":
                data = scan.scan_json()
                if data is None:
                    self._send_json(HTTPStatus.OK, {"available": False})
                else:
                    data["available"] = True
                    self._send_json(HTTPStatus.OK, data)
                return
            if path == "/api/camera":
                jpeg = camera.jpeg_bytes() if camera.available else None
                if jpeg is None:
                    self._send_json(HTTPStatus.NOT_FOUND, {"error": "camera not available"})
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpeg)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(jpeg)
                return
            self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

    return Handler


class LidarTestDashboard(Node):
    def __init__(self) -> None:
        super().__init__("lidar_test_dashboard")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8099)
        # "0"/"1"等のindexでも/dev/video0等のパスでもよい。launch側からYAML経由で
        # 渡すと"0"のような値が整数型と解釈されデフォルト(文字列型)と型不一致に
        # なるため、dynamic_typingでどちらの型も受け付けるようにする。
        self.declare_parameter(
            "camera_index", "0",
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("camera_fps", 5.0)
        self.scan = ScanController(self)
        self.camera = WebcamPreview(
            self,
            str(self.get_parameter("camera_index").value),
            float(self.get_parameter("camera_fps").value),
        )
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        handler = make_handler(lambda: HTML, self.scan, self.camera)
        self._server = start_dashboard_server(self, host, port, handler)


def main() -> None:
    rclpy.init()
    node = LidarTestDashboard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
