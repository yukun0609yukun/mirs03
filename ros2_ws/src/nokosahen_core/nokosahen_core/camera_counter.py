from __future__ import annotations

import time
from typing import Any, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .json_tools import dumps


class CameraCounter(Node):
    """Lightweight payment-side camera counter.

    This node is intentionally optional. If OpenCV/YOLO is not installed, the
    rest of the ROS 2 stack still runs and counts can be sent through Web UI.

    推論デバイスはRaspberry Pi 4 + Google Coral USB Accelerator(TPU)と
    Jetson Nanoのどちらでも動く想定(2026-07-15時点でどちらが実用になるか比較検討中)。
    コード側はどちらも同じ`model_path`パラメータを`ultralytics.YOLO()`に渡す
    だけで、Coral用にexportした`*_edgetpu.tflite`を指定すればultralyticsが
    ファイル名から自動でEdge TPUバックエンドに切り替える(この場合`device`は
    無視される)。Jetsonでは通常通り`yolov8n.pt`(CUDA/CPU自動選択)を使う。
    切り替えのための本番コード変更は不要で、`model_path`(と、Coral側なら
    ランタイム導入)を変えるだけでよい。手元で試す用のPoCは
    `poc_tests/face_count_poc/`(Raspi4+Coral手順込み)・
    `poc_tests/jetson_nano_person_count_poc/`(Jetson Nano向け)を参照。
    """

    def __init__(self) -> None:
        super().__init__("camera_counter")
        # 数値indexではなく固定デバイスパス(scripts/setup_udev_rules.shが
        # 作る/dev/webcam_checkout等)を指定する。挿すUSBポート/順番で
        # /dev/video0のインデックスが変わっても影響を受けないようにするため。
        self.declare_parameter("camera_index", "/dev/webcam_checkout")
        # 空文字なら上のcamera_index(V4L2デバイス)を使う。トピック名を
        # 指定すると、そのsensor_msgs/Imageを入力にする(Gazeboシミュレーション
        # のカメラプラグインや、将来のROSカメラドライバとの連携用)。
        # 実機のcamera_indexと同時には使わない(どちらか一方)。
        self.declare_parameter("camera_image_topic", "")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 4.0)
        self.declare_parameter("menu_hint", "")
        self.declare_parameter("line_x_ratio", 0.5)
        self.declare_parameter("enable_yolo", False)
        self.declare_parameter("model_path", "yolov8n.pt")
        # 推論デバイス。空文字(既定)ならultralyticsに自動選択させる。
        # Coral USB Accelerator用にexportした*_edgetpu.tfliteをmodel_pathに
        # 指定する構成ではこの値は無視される。Jetson等でGPU(CUDA)を明示的に
        # 使わせたい場合は"0"を指定する。
        self.declare_parameter("device", "")
        self.declare_parameter("min_confidence", 0.35)
        # 「決済端末の横を通った、かなり大きく映った人だけ」を数えるための
        # 近接フィルタ。バウンディングボックスの高さがフレーム高さに対して
        # この比率以上のときだけカウント対象にする(遠くの行列や通行人は無視)。
        # (Downloads/camera2.pyのMIN_PERSON_HEIGHT[絶対px]と同じ狙いだが、
        # このノードはカメラ解像度がwidth/heightパラメータで可変なため、
        # 解像度に依存しない比率で持たせている。)
        self.declare_parameter("min_person_height_ratio", 0.5)
        # どちら向きにラインを跨いだときだけカウントするか。
        # "both"なら従来通りどちら向きの通過も1カウントする(決済端末は
        # どちら向きに歩いても購入とみなしたいため既定は"both")。
        # Downloads/camera2.py同様の一方通行判定にしたい場合は
        # "left_to_right"/"right_to_left"を指定する。
        self.declare_parameter("direction", "both")
        # このtrack_id(YOLOの追跡ID)が何フレーム連続で検出されたら
        # 「安定した追跡」とみなしてカウント対象にするか。1〜2フレームだけの
        # 誤検出でカウントされるのを防ぐ(Downloads/camera2.pyのMIN_TRACK_HITSと同じ)。
        self.declare_parameter("min_track_hits", 3)
        # このフレーム数だけ見えなくなったtrack_idは追跡情報を破棄する
        # (Downloads/camera2.pyのTRACK_STALE_FRAMESと同じ)。
        self.declare_parameter("track_stale_frames", 60)

        self._pub = self.create_publisher(String, "/nokosahen/count_event", 20)
        self._target_period = 1.0 / max(1.0, float(self.get_parameter("fps").value))
        self._menu_hint = str(self.get_parameter("menu_hint").value)
        self._line_x_ratio = float(self.get_parameter("line_x_ratio").value)
        self._device = str(self.get_parameter("device").value).strip()
        self._min_confidence = float(self.get_parameter("min_confidence").value)
        self._min_person_height_ratio = float(
            self.get_parameter("min_person_height_ratio").value
        )
        if not (0.0 <= self._min_person_height_ratio <= 1.0):
            self.get_logger().warn(
                f"min_person_height_ratio={self._min_person_height_ratio} は0.0〜1.0の範囲外です。"
                "1.0超だと誰も検出されなくなり、カウントが恒久的に止まります。"
            )
        self._direction = str(self.get_parameter("direction").value)
        if self._direction not in ("both", "left_to_right", "right_to_left"):
            self.get_logger().warn(
                f"direction={self._direction!r} は不正な値です。'both'として扱います。"
            )
            self._direction = "both"
        self._min_track_hits = int(self.get_parameter("min_track_hits").value)
        self._track_stale_frames = int(self.get_parameter("track_stale_frames").value)

        self._prev_side: Optional[str] = None
        self._last_event_time = 0.0
        self._cap = None
        self._model = None
        self._latest_topic_frame = None

        # YOLOトラッキング(model.track)使用時の追跡状態
        # (Downloads/camera2.pyと同じ考え方: track_idごとにヒット数・前回中心x・
        # 最終検出フレームを持ち、ライン通過判定とカウント済み管理を行う)。
        self._frame_count = 0
        self._track_hits: dict = {}
        self._last_seen: dict = {}
        self._prev_center_x: dict = {}
        self._counted_ids: set = set()

        try:
            import cv2  # type: ignore

            self._cv2 = cv2
        except Exception as exc:  # pragma: no cover - depends on target machine
            self._cv2 = None
            self.get_logger().warn(f"OpenCV unavailable; camera_counter disabled: {exc}")
            return

        image_topic = str(self.get_parameter("camera_image_topic").value).strip()
        if image_topic:
            # Gazeboシミュレーション等のsensor_msgs/Imageを入力にするモード。
            # 実機のV4L2デバイスは開かない。
            self.create_subscription(Image, image_topic, self._on_image_topic, 5)
            self.get_logger().info(f"camera_counter: 画像トピック({image_topic})を入力に使用")
        else:
            camera_index = str(self.get_parameter("camera_index").value)
            # person_target_detectorと同じ理由でV4L2を明示指定(GStreamer経由でSEGVするため)。
            self._cap = self._cv2.VideoCapture(camera_index, self._cv2.CAP_V4L2)
            if not self._cap.isOpened():
                # udevルール未適用や、person_target_detectorとの共有デバイス競合等で
                # 開けない場合、ここで気づけないと_tick()が永久に(False, None)を
                # 返し続けるだけで誰も気づけない。
                self.get_logger().error(
                    f"決済カメラ({camera_index})を開けませんでした。"
                    "scripts/setup_udev_rules.shの実行や、他ノードとのデバイス競合を確認してください。"
                )
                self._cap.release()
                self._cap = None
                return
            self._cap.set(self._cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter("width").value))
            self._cap.set(self._cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter("height").value))
            self._cap.set(self._cv2.CAP_PROP_FPS, float(self.get_parameter("fps").value))

        if bool(self.get_parameter("enable_yolo").value):
            self._load_yolo()

        self.create_timer(self._target_period, self._tick)
        self.get_logger().info("camera_counter started")

    def _on_image_topic(self, msg: Image) -> None:
        try:
            if msg.encoding == "bgr8":
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif msg.encoding == "rgb8":
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                frame = frame[:, :, ::-1]
            else:
                self.get_logger().warn(
                    f"未対応のImage encoding: {msg.encoding} (bgr8/rgb8のみ対応)", once=True
                )
                return
            self._latest_topic_frame = frame.copy()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"画像トピックの変換に失敗: {exc}")

    def destroy_node(self) -> bool:
        if self._cap is not None:
            self._cap.release()
        return super().destroy_node()

    def _load_yolo(self) -> None:
        try:
            from ultralytics import YOLO  # type: ignore

            self._model = YOLO(str(self.get_parameter("model_path").value))
            self.get_logger().info("YOLO model loaded")
        except Exception as exc:  # pragma: no cover - optional dependency
            self._model = None
            self.get_logger().warn(f"YOLO unavailable; falling back to motion: {exc}")

    def _tick(self) -> None:
        if self._cap is not None:
            ok, frame = self._cap.read()
            if not ok or frame is None:
                return
        else:
            # 画像トピックモード: まだ1枚も届いていなければ何もしない。
            frame = self._latest_topic_frame
            if frame is None:
                return

        frame_height = frame.shape[0]
        if frame_height <= 0:
            return

        if self._model is not None:
            self._tick_yolo_tracking(frame, frame_height)
        else:
            self._tick_motion_fallback(frame, frame_height)

    def _tick_yolo_tracking(self, frame: Any, frame_height: int) -> None:
        """YOLOのトラッキング(model.track, persist=True)でtrack_idごとに
        ラインを跨いだ瞬間だけカウントする(Downloads/camera2.pyの
        カウントロジックをROS 2ノードとして移植したもの)。単純に毎フレームの
        検出結果を平均する旧方式より、複数人が同時に映っても取り違えにくい。
        """
        self._frame_count += 1
        width = frame.shape[1]
        line_x = width * self._line_x_ratio

        results = self._model.track(
            frame, persist=True, classes=[0], imgsz=320,
            device=(self._device or None), verbose=False,
        )
        if not results or results[0].boxes.id is None:
            self._cleanup_stale_tracks()
            return

        boxes = results[0].boxes.xyxy.cpu().numpy()
        confs = results[0].boxes.conf.cpu().numpy()
        track_ids = results[0].boxes.id.int().cpu().tolist()

        for box, conf, track_id in zip(boxes, confs, track_ids):
            if float(conf) < self._min_confidence:
                continue
            x1, y1, x2, y2 = [float(v) for v in box]
            person_height = y2 - y1
            # 近接フィルタ: 遠くにいる(=小さく映っている)人は無視する。
            if (person_height / frame_height) < self._min_person_height_ratio:
                continue

            cx = (x1 + x2) / 2.0
            self._last_seen[track_id] = self._frame_count
            self._track_hits[track_id] = self._track_hits.get(track_id, 0) + 1
            is_stable_track = self._track_hits[track_id] >= self._min_track_hits

            prev_x = self._prev_center_x.get(track_id)
            crossed_line = False
            if prev_x is not None:
                if self._direction in ("left_to_right", "both") and prev_x < line_x <= cx:
                    crossed_line = True
                if self._direction in ("right_to_left", "both") and prev_x > line_x >= cx:
                    crossed_line = True

            if crossed_line and is_stable_track and track_id not in self._counted_ids:
                self._counted_ids.add(track_id)
                self._publish_count()

            self._prev_center_x[track_id] = cx

        self._cleanup_stale_tracks()

    def _cleanup_stale_tracks(self) -> None:
        stale_ids = [
            tid
            for tid, seen in self._last_seen.items()
            if self._frame_count - seen > self._track_stale_frames
        ]
        for tid in stale_ids:
            self._last_seen.pop(tid, None)
            self._prev_center_x.pop(tid, None)
            self._track_hits.pop(tid, None)

    def _tick_motion_fallback(self, frame: Any, frame_height: int) -> None:
        """ultralytics未導入機での簡易フォールバック(フレーム差分による動体検出)。
        track_idを持たないため、Downloads/camera2.py方式のtrack_id単位の判定は
        できず、従来通り「検出された人物群の平均中心xがラインのどちら側か」で
        判定する(複数人の取り違えに弱い点は変わらないが、依存ライブラリなしで
        動く最低限の代替として維持する)。
        """
        detections = self._detect_with_motion(frame)
        centers = [
            (x, y)
            for (x, y, height) in detections
            if (height / frame_height) >= self._min_person_height_ratio
        ]
        if not centers:
            return

        width = frame.shape[1]
        line_x = width * self._line_x_ratio
        center_x = sum(point[0] for point in centers) / len(centers)
        side = "left" if center_x < line_x else "right"
        now = time.monotonic()

        if self._prev_side and self._prev_side != side and now - self._last_event_time > 1.2:
            self._publish_count()
            self._last_event_time = now
        self._prev_side = side

    def _detect_with_motion(self, frame: Any) -> List[Tuple[float, float, float]]:
        gray = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2GRAY)
        gray = self._cv2.GaussianBlur(gray, (21, 21), 0)
        if not hasattr(self, "_prev_gray"):
            self._prev_gray = gray
            return []

        diff = self._cv2.absdiff(self._prev_gray, gray)
        self._prev_gray = gray
        _, thresh = self._cv2.threshold(diff, 28, 255, self._cv2.THRESH_BINARY)
        contours, _ = self._cv2.findContours(
            thresh, self._cv2.RETR_EXTERNAL, self._cv2.CHAIN_APPROX_SIMPLE
        )
        boxes: List[Tuple[float, float, float]] = []
        for contour in contours:
            area = self._cv2.contourArea(contour)
            if area < 900:
                continue
            x, y, w, h = self._cv2.boundingRect(contour)
            boxes.append((x + w / 2.0, y + h / 2.0, float(h)))
        return boxes

    def _publish_count(self) -> None:
        menu = self._menu_hint or "unknown"
        msg = String()
        msg.data = dumps({"menu": menu, "delta": 1, "source": "camera_counter"})
        self._pub.publish(msg)
        self.get_logger().info(f"counted purchase: {menu}")


def main() -> None:
    rclpy.init()
    node = CameraCounter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

