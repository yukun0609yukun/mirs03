from __future__ import annotations

import json
import os
import socket
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from .crossing_logic import has_crossed_line, is_near, should_count_crossing


def _dumps(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, separators=(",", ":"))


def _loads(text: str, fallback: Any = None) -> Any:
    try:
        return json.loads(text)
    except (TypeError, ValueError):
        return fallback


def _candidate_video_devices() -> List[str]:
    """接続されているWebカメラ候補デバイスを列挙する(Linux前提)。

    udevルールでの固定名付け(ベンダーID調査等の事前セットアップ)を不要にする
    ため、起動時に実際に開いて映像が取れるデバイスを自動で探す方式にしている。
    どのRaspberry Piにも同一のコード・設定をそのまま導入できるようにするのが狙い。
    """
    devices: List[str] = []
    try:
        for name in sorted(os.listdir("/dev")):
            if name.startswith("video"):
                devices.append(f"/dev/{name}")
    except OSError:
        pass
    # ソート順が"video10"<"video2"のような文字列順にならないよう番号でも並べ替える
    def _video_num(path: str) -> int:
        try:
            return int(path.rsplit("video", 1)[1])
        except (IndexError, ValueError):
            return 999
    devices.sort(key=_video_num)
    return devices


def _resolve_camera_index(cv2: Any, requested: str, width: int, height: int) -> Tuple[Optional[Any], str]:
    """カメラのVideoCaptureを開いて返す。requestedが"auto"(既定)なら、
    実際に1フレーム取得できる最初のデバイスを自動選択する。

    tty/videoの番号は挿す順番やUSBハブ構成で変わりうるため、固定パスを
    手動設定させるのではなく、ここで毎回実機を確かめてから使う。
    """
    candidates: List[str]
    if requested and requested.strip().lower() != "auto":
        candidates = [requested.strip()]
    else:
        candidates = _candidate_video_devices() or ["/dev/video0"]

    for candidate in candidates:
        cap = cv2.VideoCapture(candidate)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        ok, frame = cap.read()
        if ok and frame is not None and frame.size > 0:
            return cap, candidate
        cap.release()
    return None, ""


def _detect_camera_id() -> str:
    """このユニットを一意に識別するIDを、設定ファイルを触らずに自動で決める。

    Raspberry PiはCPUシリアル番号(/proc/cpuinfoのSerial行)が個体ごとに
    一意かつ不変なので、これを使う。同一のコード・設定を全ユニットへそのまま
    導入するだけで、各機が自分自身を自動で区別できるようにするため。
    Raspberry Pi以外(開発PC等)ではSerialが無いので、ホスト名+MACアドレス風の
    値にフォールバックする。
    """
    try:
        with open("/proc/cpuinfo", "r", encoding="utf-8") as f:
            for line in f:
                if line.startswith("Serial"):
                    serial = line.split(":", 1)[1].strip()
                    if serial and serial.strip("0"):
                        return f"rpi-{serial[-8:]}"
    except OSError:
        pass
    return f"host-{socket.gethostname()}"


class CheckoutCameraNode(Node):
    """メニュー毎の決済場所に置く独立ユニット(Webカメラ+Raspberry Pi 4)用ノード。

    設計:
      - 自機の識別(camera_id)はCPUシリアルから自動で決まるため、どの
        Raspberry Piにも全く同じコード・設定をそのまま導入すればよい
        (個体ごとの書き換え不要)。
      - どのメニュー用かは、起動時点では未定(None)。Jetson側の完成版
        ダッシュボードから`/nokosahen/checkout_cam/assign`へ割り当てが
        publishされたら、自機宛て(camera_id一致)のものだけを覚える。
        割り当てはローカルファイルに保存し、再起動しても引き継ぐ。
      - 検出はPoC(count.py)のYOLOv8トラッキング+ライン交差判定を土台にし、
        camera_counter.pyと同じ「バウンディングボックスが十分大きい(=画面の
        近く・真正面)ときだけ」という近接フィルタを組み合わせる。奥の方を
        通り過ぎる人(背景の通行人)を誤ってカウントしないようにするため。
      - 決済イベントは既存の`/nokosahen/count_event`契約(delta=1が「1つ
        消費された」= 残数-1)にそのまま乗せる。inventory_manager側の
        変更は不要。
    """

    def __init__(self) -> None:
        super().__init__("checkout_camera_node")
        # "auto"(既定)なら/dev/video*を順に開いて実際に1フレーム取得できた
        # ものを自動選択する(udevでの固定名付け等の事前セットアップ不要)。
        # 特定デバイスに固定したい場合のみ"/dev/videoN"等を明示指定する。
        self.declare_parameter("camera_index", "auto")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 8.0)
        self.declare_parameter("model_path", "yolov8n.pt")
        # 推論デバイス。空文字(既定)ならultralyticsに自動選択させる
        # (CPU限定のRaspberry Pi単体構成や、Coral TPU用にexportした
        # *_edgetpu.tfliteをmodel_pathに指定する構成ではこの値は無視される)。
        # Jetson Nano等、GPU(CUDA)を明示的に使わせたい場合は"0"を指定する。
        # ハードウェア別の使い分けはdocs/checkout_cam_setup.mdを参照。
        self.declare_parameter("device", "")
        self.declare_parameter("min_confidence", 0.4)
        self.declare_parameter("line_x_ratio", 0.5)
        # 決済端末の真正面を通った人だけを数える近接フィルタ。バウンディング
        # ボックスの高さがフレーム高さに対してこの比率以上のときだけ
        # カウント対象にする(camera_counter.pyのmin_person_height_ratioと同じ考え方)。
        self.declare_parameter("min_person_height_ratio", 0.55)
        # "both"(既定)ならどちら向きの通過も1購入とみなす。特定の向きだけを
        # 数えたい設置(例: 一方通行の導線)では"left_to_right"/"right_to_left"を指定する。
        self.declare_parameter("direction", "both")
        self.declare_parameter("min_track_hits", 3)
        self.declare_parameter("track_stale_frames", 60)
        self.declare_parameter("status_hz", 1.0)
        self.declare_parameter("preview_jpeg_quality", 55)
        self.declare_parameter(
            "menu_assignment_file", "~/.nokosahen_checkout_menu"
        )
        # camera_idを明示的に上書きしたい場合(複数ユニットを同じ開発PCで
        # テストする等)のための脱出口。空ならCPUシリアルから自動決定する。
        self.declare_parameter("camera_id_override", "")

        override = str(self.get_parameter("camera_id_override").value).strip()
        self.camera_id = override or _detect_camera_id()
        self.get_logger().info(f"checkout_camera_node: camera_id = {self.camera_id}")

        self.device = str(self.get_parameter("device").value).strip()
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.line_x_ratio = float(self.get_parameter("line_x_ratio").value)
        self.min_person_height_ratio = float(
            self.get_parameter("min_person_height_ratio").value
        )
        if not (0.0 <= self.min_person_height_ratio <= 1.0):
            self.get_logger().warn(
                f"min_person_height_ratio={self.min_person_height_ratio} は0.0〜1.0の範囲外です。"
                "1.0超だと誰も検出されなくなり、カウントが恒久的に止まります。"
            )
        self.direction = str(self.get_parameter("direction").value)
        if self.direction not in ("both", "left_to_right", "right_to_left"):
            self.get_logger().warn(
                f"direction={self.direction!r} は不正な値です。'both'として扱います。"
            )
            self.direction = "both"
        self.min_track_hits = max(1, int(self.get_parameter("min_track_hits").value))
        self.track_stale_frames = max(
            1, int(self.get_parameter("track_stale_frames").value)
        )
        self.preview_jpeg_quality = max(
            1, min(100, int(self.get_parameter("preview_jpeg_quality").value))
        )
        self.menu_assignment_file = os.path.expanduser(
            str(self.get_parameter("menu_assignment_file").value).strip()
            or "~/.nokosahen_checkout_menu"
        )
        # 完成版ダッシュボードから方向(direction)を割り当てた場合の永続化先。
        # menu_assignment_fileと同じ考え方(拡張子だけ変えたパス)で、
        # 再起動しても割り当てを引き継ぐ。
        self.direction_assignment_file = self.menu_assignment_file + "_direction"

        self.menu: Optional[str] = self._load_menu_assignment()
        saved_direction = self._load_direction_assignment()
        if saved_direction:
            self.direction = saved_direction

        # PoC(count.py)と同じ考え方のトラッキング状態
        self._frame_count = 0
        self._counted_ids: set = set()
        self._track_hits: Dict[int, int] = {}
        self._last_seen: Dict[int, int] = {}
        self._prev_center_x: Dict[int, float] = {}
        self._near_now = 0
        self._counted_total = 0
        self._camera_ok = False

        self._count_pub = self.create_publisher(String, "/nokosahen/count_event", 20)
        self._preview_pub = self.create_publisher(
            CompressedImage, "/nokosahen/checkout_cam/preview", 2
        )
        self._status_pub = self.create_publisher(
            String, "/nokosahen/checkout_cam/status", 5
        )
        self.create_subscription(
            String, "/nokosahen/checkout_cam/assign", self._on_assign, 10
        )

        self._cv2 = None
        self._model = None
        self._cap = None
        try:
            import cv2  # type: ignore

            self._cv2 = cv2
        except Exception as exc:  # pragma: no cover - 実行環境依存
            self.get_logger().error(f"OpenCV未導入のため動作できません: {exc}")
            return

        camera_index = str(self.get_parameter("camera_index").value)
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        self._cap, opened_path = _resolve_camera_index(self._cv2, camera_index, width, height)
        if self._cap is None:
            self.get_logger().error(
                "Webカメラを開けませんでした(/dev/video*を自動探索しましたが、"
                "実際に映像が取れるデバイスが見つかりません)。配線・USB接続を確認してください。"
            )
            return
        self._cap.set(self._cv2.CAP_PROP_FPS, float(self.get_parameter("fps").value))
        self._camera_ok = True
        self.get_logger().info(f"Webカメラを自動選択しました: {opened_path}")

        try:
            from ultralytics import YOLO  # type: ignore

            model_path = os.path.expanduser(str(self.get_parameter("model_path").value))
            if not os.path.isabs(model_path) and not os.path.exists(model_path):
                # 相対パスでファイルが無い場合、ultralyticsはインターネットから
                # 自動ダウンロードしようとする。本番運用はオフラインルーター
                # 接続のため、ここで気付けるよう先に警告しておく。
                self.get_logger().warn(
                    f"model_path='{model_path}' がローカルに見つかりません。"
                    "オフライン環境だとYOLOの自動ダウンロードに失敗します。"
                    "scripts/install_checkout_cam_raspi4.shで事前取得したモデルの"
                    "パスをmodel_pathに設定してください。"
                )
            self._model = YOLO(model_path)
            device_label = self.device or "auto"
            self.get_logger().info(
                f"YOLO model loaded (tracking mode): {model_path} (device={device_label})"
            )
        except Exception as exc:  # pragma: no cover - 任意依存
            self._model = None
            self.get_logger().warn(
                f"YOLO(ultralytics)未導入のため、簡易フォールバック(トラッキング無し・"
                f"左右通過検知のみ)で動作します: {exc}"
            )

        fps = max(1.0, float(self.get_parameter("fps").value))
        self.create_timer(1.0 / fps, self._tick)
        status_hz = max(0.2, float(self.get_parameter("status_hz").value))
        self.create_timer(1.0 / status_hz, self._publish_status)
        self.get_logger().info(
            f"checkout_camera_node started (menu={self.menu or '(未割当)'})"
        )

    # --- メニュー割り当て ---

    def _load_menu_assignment(self) -> Optional[str]:
        try:
            with open(self.menu_assignment_file, "r", encoding="utf-8") as f:
                menu = f.read().strip()
                return menu or None
        except OSError:
            return None

    def _save_menu_assignment(self, menu: str) -> None:
        try:
            with open(self.menu_assignment_file, "w", encoding="utf-8") as f:
                f.write(menu)
        except OSError as exc:
            self.get_logger().warn(f"メニュー割り当ての保存に失敗: {exc}")

    def _load_direction_assignment(self) -> Optional[str]:
        try:
            with open(self.direction_assignment_file, "r", encoding="utf-8") as f:
                direction = f.read().strip()
                return direction or None
        except OSError:
            return None

    def _save_direction_assignment(self, direction: str) -> None:
        try:
            with open(self.direction_assignment_file, "w", encoding="utf-8") as f:
                f.write(direction)
        except OSError as exc:
            self.get_logger().warn(f"方向割り当ての保存に失敗: {exc}")

    def _on_assign(self, msg: String) -> None:
        data = _loads(msg.data, {})
        if not isinstance(data, dict):
            return
        if str(data.get("camera_id", "")) != self.camera_id:
            return  # 他ユニット宛て

        # menu/directionはそれぞれ独立に更新できる(片方だけの変更も許す)。
        menu = str(data.get("menu", "")).strip()
        if menu:
            if menu != self.menu:
                self.get_logger().info(f"メニュー割り当てを変更: {self.menu!r} -> {menu!r}")
            self.menu = menu
            self._save_menu_assignment(menu)

        direction = str(data.get("direction", "")).strip()
        if direction:
            if direction not in ("both", "left_to_right", "right_to_left"):
                self.get_logger().warn(
                    f"割り当てられたdirection={direction!r}は不正な値のため無視します。"
                )
            else:
                if direction != self.direction:
                    self.get_logger().info(f"方向割り当てを変更: {self.direction!r} -> {direction!r}")
                self.direction = direction
                self._save_direction_assignment(direction)

    # --- 検出・カウント ---

    def destroy_node(self) -> bool:
        if self._cap is not None:
            self._cap.release()
        return super().destroy_node()

    def _tick(self) -> None:
        if self._cap is None:
            return
        try:
            ok, frame = self._cap.read()
            if not ok or frame is None or frame.size == 0:
                self._camera_ok = False
                return
            self._camera_ok = True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"フレーム取得に失敗: {exc}")
            self._camera_ok = False
            return

        self._frame_count += 1
        height, width = frame.shape[0], frame.shape[1]
        if width <= 0 or height <= 0:
            return
        line_x = width * self.line_x_ratio

        try:
            if self._model is not None:
                self._tick_tracking(frame, width, height, line_x)
            else:
                self._tick_fallback(frame, width, height, line_x)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"検出処理中にエラー(継続します): {exc}")

    def _tick_tracking(self, frame: Any, width: int, height: int, line_x: float) -> None:
        """PoC(count.py)のYOLOトラッキング+ライン交差に、近接フィルタを足したもの。

        model_path/deviceの組み合わせでハードウェアを切り替えられる:
          - Raspberry Pi単体(CPU): model_path=yolov8n.pt、device=""(既定)
          - Raspberry Pi + Coral USB Accelerator: model_path=*_edgetpu.tflite
            (deviceは無視される。Edge TPUはultralyticsがtfliteの中身から自動判別する)
          - Jetson Nano(GPU): model_path=yolov8n.pt(またはTensorRT変換済み.engine)、
            device="0"でCUDAを明示指定
        カウント判定ロジック自体はどのバックエンドでも同じ(track_id・近接フィルタ・
        追跡安定性・方向判定)。
        """
        results = self._model.track(
            frame, persist=True, classes=[0], device=(self.device or None), verbose=False
        )
        near_now = 0
        preview_boxes: List[Tuple[float, float, float, float, bool]] = []

        result = results[0] if results else None
        if result is not None and result.boxes is not None and result.boxes.id is not None:
            boxes = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            track_ids = result.boxes.id.int().cpu().tolist()

            seen_ids = set()
            for box, conf, track_id in zip(boxes, confs, track_ids):
                if float(conf) < self.min_confidence:
                    continue
                x1, y1, x2, y2 = [float(v) for v in box]
                seen_ids.add(track_id)
                self._last_seen[track_id] = self._frame_count
                self._track_hits[track_id] = self._track_hits.get(track_id, 0) + 1

                near = is_near(y1, y2, height, self.min_person_height_ratio)
                if near:
                    near_now += 1

                cx = (x1 + x2) / 2.0
                prev_x = self._prev_center_x.get(track_id)
                is_stable = self._track_hits[track_id] >= self.min_track_hits
                crossed = has_crossed_line(prev_x, cx, line_x, self.direction)

                if should_count_crossing(
                    crossed=crossed,
                    is_stable=is_stable,
                    is_near_=near,
                    already_counted=track_id in self._counted_ids,
                ):
                    self._counted_ids.add(track_id)
                    self._counted_total += 1
                    self._publish_count_event()

                self._prev_center_x[track_id] = cx
                preview_boxes.append((x1, y1, x2, y2, track_id in self._counted_ids))

            stale = [
                tid
                for tid, seen in self._last_seen.items()
                if self._frame_count - seen > self.track_stale_frames
            ]
            for tid in stale:
                self._last_seen.pop(tid, None)
                self._prev_center_x.pop(tid, None)
                self._track_hits.pop(tid, None)
                self._counted_ids.discard(tid)

        self._near_now = near_now
        self._publish_preview(frame, preview_boxes, line_x)

    def _tick_fallback(self, frame: Any, width: int, height: int, line_x: float) -> None:
        """ultralytics未導入時の簡易フォールバック(トラッキングIDなし)。

        camera_counter.pyのモーション差分方式と同じ考え方: 前回フレームとの
        差分から動体の重心を検出し、ラインの左右をまたいだら1回だけ数える。
        個体追跡が無いため、複数人が同時に通ると精度は大きく落ちる
        (決済場所は基本1人ずつ通る想定なので許容する)。
        """
        cv2 = self._cv2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        if not hasattr(self, "_prev_gray"):
            self._prev_gray = gray
            self._fallback_prev_side = None
            self._fallback_last_event = 0.0
            return

        diff = cv2.absdiff(self._prev_gray, gray)
        self._prev_gray = gray
        _, thresh = cv2.threshold(diff, 28, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 0.0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 900 or area <= best_area:
                continue
            best = cv2.boundingRect(contour)
            best_area = area

        near_now = 0
        preview_boxes: List[Tuple[float, float, float, float, bool]] = []
        if best is not None:
            x, y, w, h = best
            near = is_near(0.0, float(h), float(height), self.min_person_height_ratio)
            if near:
                near_now = 1
            cx = x + w / 2.0
            side = "left" if cx < line_x else "right"
            now = time.monotonic()
            counted = False
            direction_ok = (
                self.direction == "both"
                or (self.direction == "left_to_right" and self._fallback_prev_side == "left" and side == "right")
                or (self.direction == "right_to_left" and self._fallback_prev_side == "right" and side == "left")
            )
            if (
                near
                and self._fallback_prev_side
                and self._fallback_prev_side != side
                and direction_ok
                and now - self._fallback_last_event > 1.0
            ):
                self._publish_count_event()
                self._counted_total += 1
                self._fallback_last_event = now
                counted = True
            self._fallback_prev_side = side
            preview_boxes.append((x, y, x + w, y + h, counted))

        self._near_now = near_now
        self._publish_preview(frame, preview_boxes, line_x)

    def _publish_count_event(self) -> None:
        if not self.menu:
            self.get_logger().warn(
                "通過を検知しましたが、まだメニューが割り当てられていないため"
                "カウントを送信しません(完成版ダッシュボードで割り当ててください)。",
                once=False,
            )
            return
        msg = String()
        msg.data = _dumps(
            {"menu": self.menu, "delta": 1, "source": f"checkout_cam_{self.camera_id}"}
        )
        self._count_pub.publish(msg)
        self.get_logger().info(f"counted purchase: {self.menu} (camera_id={self.camera_id})")

    # --- プレビュー・ステータス配信 ---

    def _publish_preview(
        self,
        frame: Any,
        boxes: List[Tuple[float, float, float, float, bool]],
        line_x: float,
    ) -> None:
        try:
            cv2 = self._cv2
            annotated = frame.copy()
            h = annotated.shape[0]
            cv2.line(annotated, (int(line_x), 0), (int(line_x), h), (0, 0, 255), 2)
            for x1, y1, x2, y2, counted in boxes:
                color = (0, 220, 0) if counted else (255, 180, 0)
                cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            ok, encoded = cv2.imencode(
                ".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), self.preview_jpeg_quality]
            )
            if not ok:
                return
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            # camera_idは全ユニット共有の1トピックに載せるための識別子として
            # frame_idを流用する(受信側=ダッシュボードがcamera_id毎に振り分ける)。
            msg.header.frame_id = self.camera_id
            msg.format = "jpeg"
            msg.data = encoded.tobytes()
            self._preview_pub.publish(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"プレビューの生成に失敗: {exc}", once=True)

    def _publish_status(self) -> None:
        msg = String()
        msg.data = _dumps(
            {
                "camera_id": self.camera_id,
                "menu": self.menu,
                "direction": self.direction,
                "camera_ok": self._camera_ok,
                "near_count": self._near_now,
                "counted_total": self._counted_total,
                "tracking_mode": "yolo" if self._model is not None else "motion_fallback",
                "stamp": self.get_clock().now().nanoseconds / 1e9,
            }
        )
        self._status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CheckoutCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
