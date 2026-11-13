"""Gazeboシミュレーション上で、決済横カメラ(nokosahen_checkout)のダッシュボード
UI(映像プレビュー・メニュー割り当て)が動作する様子を確認するためのモック。

実機のRaspberry Pi 4+Webカメラは無いので、複数台分のダミーcamera_idで
`/nokosahen/checkout_cam/status`・`/nokosahen/checkout_cam/preview`を定期publishし、
`/nokosahen/checkout_cam/assign`(ダッシュボードからのメニュー割り当て)を反映する。
本番運用では使わない、シミュレーション/デモ専用のノード。
"""
from __future__ import annotations

import json
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class MockCheckoutCamNode(Node):
    def __init__(self) -> None:
        super().__init__("mock_checkout_cam")
        self.declare_parameter("camera_count", 3)
        self.declare_parameter("status_hz", 1.0)

        count = max(1, int(self.get_parameter("camera_count").value))
        self._camera_ids = [f"sim-cam-{i + 1}" for i in range(count)]
        self._menu: Dict[str, Optional[str]] = {cid: None for cid in self._camera_ids}

        self._status_pub = self.create_publisher(String, "/nokosahen/checkout_cam/status", 10)
        self._preview_pub = self.create_publisher(
            CompressedImage, "/nokosahen/checkout_cam/preview", 4
        )
        self.create_subscription(String, "/nokosahen/checkout_cam/assign", self._on_assign, 10)

        self._cv2 = None
        try:
            import cv2  # type: ignore

            self._cv2 = cv2
        except Exception as exc:  # pragma: no cover - 環境依存
            self.get_logger().warn(
                f"OpenCV未導入のためプレビュー画像は生成できません(状態表示のみ動作): {exc}"
            )

        status_hz = max(0.2, float(self.get_parameter("status_hz").value))
        self.create_timer(1.0 / status_hz, self._tick)
        self.get_logger().info(
            f"mock_checkout_cam: シミュレーション用ダミーカメラ{count}台を開始 "
            f"({', '.join(self._camera_ids)})"
        )

    def _on_assign(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        camera_id = str(data.get("camera_id", "")).strip()
        menu = str(data.get("menu", "")).strip()
        if camera_id in self._menu and menu:
            self._menu[camera_id] = menu

    def _tick(self) -> None:
        for camera_id in self._camera_ids:
            self._publish_status(camera_id)
            self._publish_preview(camera_id)

    def _publish_status(self, camera_id: str) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "camera_id": camera_id,
                "menu": self._menu.get(camera_id),
                "camera_ok": True,
                "near_count": 0,
                "counted_total": 0,
                "tracking_mode": "sim_mock",
                "stamp": self.get_clock().now().nanoseconds / 1e9,
            },
            ensure_ascii=False,
        )
        self._status_pub.publish(msg)

    def _publish_preview(self, camera_id: str) -> None:
        if self._cv2 is None:
            return
        cv2 = self._cv2
        try:
            import numpy as np

            frame = np.full((240, 320, 3), (35, 60, 48), dtype=np.uint8)
            cv2.putText(
                frame, camera_id, (12, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2
            )
            menu_text = self._menu.get(camera_id) or "(未割当)"
            cv2.putText(
                frame, menu_text, (12, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 230, 210), 2
            )
            cv2.putText(
                frame,
                "SIMULATION MOCK",
                (12, 220),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (150, 150, 150),
                1,
            )
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            if not ok:
                return
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"プレビュー画像の生成に失敗: {exc}", once=True)
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = camera_id
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self._preview_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MockCheckoutCamNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
