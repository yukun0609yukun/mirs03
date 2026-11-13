#!/usr/bin/env python3
"""Dockerコンテナ内(公式pycoralが動くDebian bullseye環境)で常駐する、
Edge TPU数字認識の推論サーバー。

tablet_ocr_reader.py本体(ホストのDebian 13/Python 3.13側)は、pycoralが
動かせないため、この常駐コンテナへHTTP経由で1桁ずつの画像を送って
推論結果だけを受け取る(DockerEdgeTpuDigitRecognizer参照)。

モデルは毎回読み込むと遅いので、起動時に1度だけロードして使い回す。
"""
from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from pycoral.utils.edgetpu import make_interpreter

MODEL_PATH = "/work/model.tflite"
DIGIT_CANVAS_SIZE = 32
PORT = 5005

_interpreter = make_interpreter(MODEL_PATH)
_interpreter.allocate_tensors()
_input_index = _interpreter.get_input_details()[0]["index"]
_output_index = _interpreter.get_output_details()[0]["index"]
_lock = threading.Lock()  # pycoral/tfliteのInterpreterは複数スレッドから同時に
# invoke()すると壊れうるため、推論はこのロックで直列化する。


def predict_digit(data: np.ndarray) -> int:
    with _lock:
        _interpreter.set_tensor(_input_index, data)
        _interpreter.invoke()
        output = _interpreter.get_tensor(_output_index)[0]
    return int(np.argmax(output))


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):  # noqa: A003 - アクセスログを静かにする
        pass

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/health":
            self._send_json(200, {"ok": True})
            return
        self._send_json(404, {"error": "not found"})

    def do_POST(self) -> None:  # noqa: N802
        if self.path != "/predict":
            self._send_json(404, {"error": "not found"})
            return
        length = int(self.headers.get("Content-Length", "0"))
        expected = DIGIT_CANVAS_SIZE * DIGIT_CANVAS_SIZE
        if length != expected:
            self._send_json(
                400,
                {"error": f"expected {expected} bytes (32x32 uint8 grayscale), got {length}"},
            )
            return
        raw = self.rfile.read(length)
        try:
            data = np.frombuffer(raw, dtype=np.uint8).reshape(
                1, DIGIT_CANVAS_SIZE, DIGIT_CANVAS_SIZE, 1
            )
            digit = predict_digit(data)
        except Exception as exc:  # noqa: BLE001 - 呼び出し元へエラー内容を返すため
            self._send_json(500, {"error": str(exc)})
            return
        self._send_json(200, {"digit": digit})

    def _send_json(self, status: int, payload: dict) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main() -> None:
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"Edge TPU推論サーバーを起動しました: 0.0.0.0:{PORT}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
