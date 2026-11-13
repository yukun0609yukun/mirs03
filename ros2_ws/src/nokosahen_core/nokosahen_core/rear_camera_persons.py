"""後ろ向きカメラ2台(左後ろ・右後ろ)の人物検出(2026-10-22)。

前面カメラはperson_target_detectorが見る。後ろの2台(Logitech C270)は、機体の左後ろ(A)・右後ろ(B)に固定されている。
ここで人物(YOLO)を検出して、機体座標(前=0、左が正)の方位のリストとして /nokosahen/rear_camera_persons に配信する。
- person_target_detector: LiDARの脚の候補を、後ろの方位でもカメラで確認できるようにする(_camera_confirms)
- person_clear_gate: 前後のカメラのどれにも人が映っていないことの判定に使う

カメラが開けない/YOLOが無い場合は、そのカメラを「無効」として配信する(人がいないとは言わない)。
"""
from __future__ import annotations

import json
import math
import time
from typing import Any, Dict, List, Tuple

try:  # ROSなしの単体テストでも、純粋な関数だけを読み込めるようにする
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object

# 【2026-11-05】抜けた/開けなかったカメラを開き直す間隔[秒]と、同じ警告を出す最短間隔[秒]
REOPEN_INTERVAL_SEC = 10.0
WARN_INTERVAL_SEC = 30.0


class RetryTimer:
    """前回の試行から interval_s 経ったか(試行すれば時刻を更新する)。キーごとに管理。初回は即試行可。"""

    def __init__(self, interval_s: float) -> None:
        self.interval_s = interval_s
        self._last: Dict[str, float] = {}

    def due(self, key: str, now: float) -> bool:
        last = self._last.get(key)
        if last is not None and now - last < self.interval_s:
            return False
        self._last[key] = now
        return True


DEFAULT_CAMERAS = [
    {"name": "left_rear", "device": "/dev/v4l/by-id/usb-046d_0825_5DC75B50-video-index0", "yaw_deg": 135.0, "fov_deg": 60.0},
    {"name": "right_rear", "device": "/dev/v4l/by-id/usb-046d_0825_04080620-video-index0", "yaw_deg": -135.0, "fov_deg": 60.0},
]


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def box_bearing(center_x: float, width: float, yaw_deg: float, fov_deg: float) -> float:
    """画像の横位置から、機体座標の方位[rad](前=0、左が正)。画像の左が機体の左(+)側。
    前面カメラ(person_target_detector)と同じく、レンズ歪みを補正しない線形近似。"""
    normalized = (center_x / max(width, 1.0)) - 0.5  # -0.5..0.5
    return wrap_angle(math.radians(yaw_deg) - normalized * math.radians(fov_deg))


def parse_cameras(text: str) -> List[Dict[str, Any]]:
    """camerasパラメータ(JSON文字列)を検証して返す。空・不正なら既定。"""
    if not text.strip():
        return [dict(c) for c in DEFAULT_CAMERAS]
    try:
        data = json.loads(text)
    except ValueError:
        return [dict(c) for c in DEFAULT_CAMERAS]
    out = []
    for c in data if isinstance(data, list) else []:
        try:
            out.append({"name": str(c["name"]), "device": str(c["device"]),
                        "yaw_deg": float(c["yaw_deg"]), "fov_deg": float(c["fov_deg"])})
        except (KeyError, TypeError, ValueError):
            continue
    return out or [dict(c) for c in DEFAULT_CAMERAS]


class RearCameraPersons(Node):  # type: ignore[misc]
    def __init__(self) -> None:
        super().__init__("rear_camera_persons")
        self.declare_parameter("cameras", "")
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("min_confidence", 0.35)
        self.declare_parameter("fps", 3.0)
        self.declare_parameter("imgsz", 320)
        self._cams = parse_cameras(str(self.get_parameter("cameras").value))
        self._min_conf = float(self.get_parameter("min_confidence").value)
        self._imgsz = int(self.get_parameter("imgsz").value)
        self._pub = self.create_publisher(String, "/nokosahen/rear_camera_persons", 5)
        # 【2026-10-22】管理画面で見るための、検出枠つきJPEG。見ている人(購読者)がいるときだけ作る。
        self._preview_pubs = {
            c["name"]: self.create_publisher(CompressedImage, "/nokosahen/rear_camera/%s_preview" % c["name"].replace("_rear", ""), 2)
            for c in self._cams
        }
        self._caps: Dict[str, Any] = {}
        self._cap_bad: set = set()  # 開けていない/映らないカメラ(開き直しの対象。YOLO無しで人物検出が無効なだけのカメラは含めない)
        self._reopen_timer = RetryTimer(REOPEN_INTERVAL_SEC)
        self._warn_timer = RetryTimer(WARN_INTERVAL_SEC)  # 警告の間引き(同じ警告は WARN_INTERVAL_SEC に1回)
        self._model = None
        self._cv2 = None
        self._init_devices()
        self.create_timer(1.0 / max(0.5, float(self.get_parameter("fps").value)), self._tick)

    def _init_devices(self) -> None:
        try:
            import cv2  # type: ignore

            self._cv2 = cv2
            for c in self._cams:
                if not self._open_camera(c):
                    self.get_logger().warn(f"後ろ向きカメラ {c['name']} を開けません(約{REOPEN_INTERVAL_SEC:.0f}秒ごとに開き直します): {c['device']}")
                    self._reopen_timer.due(c["name"], time.monotonic())  # 起動時に試したので、次は間隔を空ける
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"OpenCV未導入のため後ろ向きカメラは無効: {exc}")
        try:
            from ultralytics import YOLO  # type: ignore

            self._model = YOLO(str(self.get_parameter("model_path").value))
            try:
                import torch  # type: ignore

                if torch.cuda.is_available():
                    self._model.to("cuda")
            except Exception:  # noqa: BLE001
                pass
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"YOLO未導入のため後ろ向きカメラの人物検出は無効: {exc}")

    def _open_camera(self, c: Dict[str, Any]) -> bool:
        """カメラを開いて self._caps に入れる。開けなければ False。"""
        cv2 = self._cv2
        cap = cv2.VideoCapture(c["device"], cv2.CAP_V4L2)
        if not cap.isOpened():
            try:
                cap.release()
            except Exception:  # noqa: BLE001
                pass
            return False
        # 【2026-10-22】USB2のハブに前面+後ろ2台がつながると、非圧縮(YUYV)では帯域が足りず、2台目が映らない。
        # MJPG(圧縮)・小さめの解像度にして、3台を同時に流せるようにする。
        for prop, val in ((cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")), (cv2.CAP_PROP_FRAME_WIDTH, 640),
                          (cv2.CAP_PROP_FRAME_HEIGHT, 480), (cv2.CAP_PROP_FPS, 10), (cv2.CAP_PROP_BUFFERSIZE, 1)):
            try:
                cap.set(prop, val)
            except Exception:  # noqa: BLE001
                pass
        self._caps[c["name"]] = cap
        return True

    def _try_reopen(self, cam: Dict[str, Any]) -> None:
        """【2026-11-05】カメラが無い/映らないときに、一定間隔で閉じて開き直す(USBが抜けて挿し直されたとき復帰するため)。"""
        if self._cv2 is None or not self._reopen_timer.due(cam["name"], time.monotonic()):
            return
        old = self._caps.pop(cam["name"], None)
        if old is not None:
            try:
                old.release()
            except Exception:  # noqa: BLE001
                pass
        try:
            if self._open_camera(cam):
                self.get_logger().info(f"後ろ向きカメラ {cam['name']} を開き直しました")
        except Exception as exc:  # noqa: BLE001
            if self._warn_timer.due("reopen:" + cam["name"], time.monotonic()):
                self.get_logger().warn(f"後ろ向きカメラ {cam['name']} の開き直しに失敗: {exc}")

    def _detect(self, cam: Dict[str, Any]) -> Tuple[bool, List[float]]:
        cap = self._caps.get(cam["name"])
        if cap is None:
            self._cap_bad.add(cam["name"])
            return False, []
        ok, frame = cap.read()
        if not ok or frame is None or frame.size == 0:
            self._cap_bad.add(cam["name"])
            return False, []
        self._cap_bad.discard(cam["name"])
        if self._model is None:
            self._publish_preview(cam, frame, [])  # YOLOなしでも映像は見られる(人物検出は無効のまま)
            return False, []
        width = frame.shape[1]
        bearings: List[float] = []
        boxes: List[Tuple[float, float, float, float, float]] = []
        for result in self._model.predict(frame, imgsz=self._imgsz, verbose=False):
            for box in result.boxes:
                if int(box.cls[0]) != 0 or float(box.conf[0]) < self._min_conf:
                    continue
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                bearings.append(box_bearing((x1 + x2) / 2.0, width, cam["yaw_deg"], cam["fov_deg"]))
                boxes.append((x1, y1, x2, y2, float(box.conf[0])))
        self._publish_preview(cam, frame, boxes)
        return True, bearings

    def _publish_preview(self, cam: Dict[str, Any], frame: Any, boxes: List[Tuple[float, float, float, float, float]]) -> None:
        pub = self._preview_pubs.get(cam["name"])
        if pub is None or pub.get_subscription_count() == 0 or self._cv2 is None:
            return
        try:
            img = frame.copy()
            for x1, y1, x2, y2, conf in boxes:
                self._cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 200, 0), 2)
                self._cv2.putText(img, "%.2f" % conf, (int(x1), max(12, int(y1) - 4)), self._cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1)
            ok, buf = self._cv2.imencode(".jpg", img, [int(self._cv2.IMWRITE_JPEG_QUALITY), 60])
            if not ok:
                return
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = cam["name"]
            msg.format = "jpeg"
            msg.data = buf.tobytes()
            pub.publish(msg)
        except Exception as exc:  # noqa: BLE001
            if self._warn_timer.due("preview:" + cam["name"], time.monotonic()):
                self.get_logger().warn("プレビューの配信に失敗: %s" % exc)

    def _tick(self) -> None:
        cams: Dict[str, Any] = {}
        all_bearings: List[float] = []
        for cam in self._cams:
            try:
                ok, bearings = self._detect(cam)
            except Exception as exc:  # noqa: BLE001
                ok, bearings = False, []
                if self._warn_timer.due("tick:" + cam["name"], time.monotonic()):  # 1tickごとに出さない
                    self.get_logger().warn(f"{cam['name']}の処理でエラー: {exc}")
            if cam["name"] in self._cap_bad and self._cv2 is not None:
                self._try_reopen(cam)  # 映らない/抜けたカメラは、一定間隔で開き直す
            cams[cam["name"]] = {"ok": ok, "count": len(bearings)}
            all_bearings.extend(bearings)
        msg = String()
        msg.data = json.dumps({"stamp": time.time(), "cams": cams, "bearings": all_bearings})
        self._pub.publish(msg)

    def destroy_node(self) -> bool:
        for cap in self._caps.values():
            try:
                cap.release()
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RearCameraPersons()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
