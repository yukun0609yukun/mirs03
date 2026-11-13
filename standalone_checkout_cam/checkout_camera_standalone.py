"""決済横カメラの単体版(Jetson Nano、またはROS2を入れないRaspberry Pi向け)。

`nokosahen_checkout/checkout_camera_node.py`(ROS2版、Raspberry Pi 4 + 任意で
Coral USB Accelerator向け)と全く同じ判定ロジック(`crossing_logic.py`、
YOLOトラッキング・近接フィルタ・追跡安定性・方向判定)を使うが、ROS2には
一切依存せず、カウントイベントをメインPC(Jetson Orin Nano等)の
`web_ui_node`へHTTP POSTするだけの単体スクリプト。

背景: Jetson NanoはJetPack 4系/Ubuntu 18.04世代のためROS 2 Humbleの
公式apt環境が無い(docs/production_setup.md参照)。そのためJetson Nanoを
決済カメラに使う場合はROS2ノードではなくこの単体スクリプトを使う。
Raspberry Pi 4 + Coral USB Acceleratorの構成は引き続きROS2版
(checkout_camera_node.py)で対応できる(model_pathに*_edgetpu.tfliteを
指定するだけでよい)。同じcrossing_logic.pyを共有しているため、
どちらの構成でもカウント判定の挙動は同一になる。

必要なもの:
    pip install -r requirements.txt   (opencv-python, ultralytics, requests)

Jetson Nano(JetPack)の場合、ultralytics/torchはNVIDIA提供のJetson向け
ホイールを使うこと(pip版のtorchは動かない場合がある)。詳細はREADME.md参照。

実行例:
    python checkout_camera_standalone.py \\
        --menu カレー \\
        --main-url http://192.168.1.10:8080 \\
        --device 0

    (--deviceを指定するとGPU(CUDA)を使う。Jetson Nanoでは通常"0"を指定する。
     Coral USB Acceleratorを使う場合は--model-pathに*_edgetpu.tfliteを指定し、
     --deviceは無視されるので指定不要)
"""
from __future__ import annotations

import argparse
import os
import time

import cv2
import requests
from ultralytics import YOLO

from crossing_logic import has_crossed_line, is_near, should_count_crossing


def _candidate_video_devices() -> list[str]:
    """接続されているWebカメラ候補デバイスを列挙する(Linux前提)。

    checkout_camera_node.pyと同じ考え方: どのユニットにも同一のコード・設定を
    そのまま導入できるよう、起動時に実際に開けるデバイスを自動で探す。
    """
    devices: list[str] = []
    try:
        for name in sorted(os.listdir("/dev")):
            if name.startswith("video"):
                devices.append(f"/dev/{name}")
    except OSError:
        pass

    def _video_num(path: str) -> int:
        try:
            return int(path.rsplit("video", 1)[1])
        except (IndexError, ValueError):
            return 999

    devices.sort(key=_video_num)
    return devices


def _resolve_camera(requested: str, width: int, height: int):
    candidates: list[str]
    if requested and requested.strip().lower() != "auto":
        candidates = [requested.strip()]
    else:
        candidates = _candidate_video_devices() or ["/dev/video0", 0]

    for candidate in candidates:
        cap = cv2.VideoCapture(candidate)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        ok, frame = cap.read()
        if ok and frame is not None and frame.size > 0:
            print(f"カメラを開きました: {candidate}")
            return cap
        cap.release()
    raise RuntimeError(
        "Webカメラを開けませんでした。--cameraで明示的にデバイスを指定するか、"
        "配線/USB接続を確認してください。"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="決済横カメラ単体版(Jetson Nano等)")
    parser.add_argument("--menu", required=True, help="このカメラが担当するメニュー名(例: カレー)")
    parser.add_argument(
        "--main-url", required=True,
        help="メインPC(Jetson Orin Nano等、web_ui_nodeが動いている機体)のURL"
             "(例: http://192.168.1.10:8080)。/api/countへPOSTする。",
    )
    parser.add_argument(
        "--camera", default="auto",
        help="カメラデバイス(既定autoで/dev/video*を自動探索)",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=8.0, help="処理フレームレートの目安(既定8.0)")
    parser.add_argument(
        "--model-path", default="yolov8n.pt",
        help="ultralytics YOLOのモデルファイル。Coral USB Accelerator使用時は"
             "export済みの*_edgetpu.tfliteを指定する。",
    )
    parser.add_argument(
        "--device", default="",
        help="推論デバイス(既定は空でultralyticsに自動選択させる)。"
             "Jetson NanoでGPU(CUDA)を明示したい場合は'0'を指定する。",
    )
    parser.add_argument("--min-confidence", type=float, default=0.4)
    parser.add_argument("--line-x-ratio", type=float, default=0.5)
    parser.add_argument(
        "--direction", choices=["left_to_right", "right_to_left", "both"], default="both",
        help="既定both(どちら向きの通過も1購入とみなす)",
    )
    parser.add_argument("--min-person-height-ratio", type=float, default=0.55)
    parser.add_argument("--min-track-hits", type=int, default=3)
    parser.add_argument("--track-stale-frames", type=int, default=60)
    parser.add_argument(
        "--source-tag", default="jetson_nano_camera",
        help="/api/countの'source'フィールドに載せる識別名(既定jetson_nano_camera)",
    )
    return parser.parse_args()


def post_count_event(main_url: str, menu: str, source_tag: str) -> None:
    try:
        res = requests.post(
            f"{main_url.rstrip('/')}/api/count",
            json={"menu": menu, "delta": 1, "source": source_tag},
            timeout=5,
        )
        res.raise_for_status()
        print(f"[カウント送信] menu={menu} -> {main_url}")
    except Exception as exc:  # noqa: BLE001
        # メインPCが一時的に落ちていても検出ループ自体は続ける(次のカウント時に
        # 再送を試みればよい。ここで例外を上げてループを止めない)。
        print(f"!! /api/countへの送信に失敗しました(継続します): {exc}")


def main() -> None:
    args = parse_args()

    print(f"YOLOモデルを読み込んでいます({args.model_path}, device={args.device or 'auto'})...")
    model = YOLO(args.model_path)

    cap = _resolve_camera(args.camera, args.width, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    frame_count = 0
    counted_ids: set[int] = set()
    track_hits: dict[int, int] = {}
    last_seen: dict[int, int] = {}
    prev_center_x: dict[int, float] = {}
    counted_total = 0
    min_interval = 1.0 / max(0.1, args.fps)

    print(f"起動したよ(担当メニュー: {args.menu})！終了するにはCtrl+Cを押してね。")

    try:
        while cap.isOpened():
            loop_start = time.monotonic()
            ok, frame = cap.read()
            if not ok or frame is None:
                print("フレーム取得に失敗しました")
                break

            frame_count += 1
            height, width = frame.shape[0], frame.shape[1]
            if width <= 0 or height <= 0:
                continue
            line_x = width * args.line_x_ratio

            results = model.track(
                frame, persist=True, classes=[0],
                device=(args.device or None), verbose=False,
            )

            result = results[0] if results else None
            if result is not None and result.boxes is not None and result.boxes.id is not None:
                boxes = result.boxes.xyxy.cpu().numpy()
                confs = result.boxes.conf.cpu().numpy()
                track_ids = result.boxes.id.int().cpu().tolist()

                for box, conf, track_id in zip(boxes, confs, track_ids):
                    if float(conf) < args.min_confidence:
                        continue
                    x1, y1, x2, y2 = [float(v) for v in box]
                    last_seen[track_id] = frame_count
                    track_hits[track_id] = track_hits.get(track_id, 0) + 1

                    near = is_near(y1, y2, height, args.min_person_height_ratio)
                    cx = (x1 + x2) / 2.0
                    prev_x = prev_center_x.get(track_id)
                    is_stable = track_hits[track_id] >= args.min_track_hits
                    crossed = has_crossed_line(prev_x, cx, line_x, args.direction)

                    if should_count_crossing(
                        crossed=crossed,
                        is_stable=is_stable,
                        is_near_=near,
                        already_counted=track_id in counted_ids,
                    ):
                        counted_ids.add(track_id)
                        counted_total += 1
                        print(f"【検知】ID:{track_id} が通過！ 合計: {counted_total}人")
                        post_count_event(args.main_url, args.menu, args.source_tag)

                    prev_center_x[track_id] = cx

                stale = [
                    tid for tid, seen in last_seen.items()
                    if frame_count - seen > args.track_stale_frames
                ]
                for tid in stale:
                    last_seen.pop(tid, None)
                    prev_center_x.pop(tid, None)
                    track_hits.pop(tid, None)
                    counted_ids.discard(tid)

            elapsed = time.monotonic() - loop_start
            if elapsed < min_interval:
                time.sleep(min_interval - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        print(f"終了しました。最終カウント: {counted_total}")


if __name__ == "__main__":
    main()
