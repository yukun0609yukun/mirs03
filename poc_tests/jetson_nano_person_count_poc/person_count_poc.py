"""Jetson Nano向け 人物カウントPoC(概念実証)。

`poc_tests/face_count_poc/`と同じYOLOトラッキング方式(model.track(persist=True)、
近接フィルタ・追跡安定性・一方向カウント)を、Jetson Nano単体で動かすことだけを
目的に切り出したもの。ROS2やmirs2603_nokosahenの他のコードには一切依存しない。
このフォルダだけで完結するので、このフォルダごと送ればJetson Nanoを持っている
メンバーがそのまま試せる。

Jetson NanoはJetPack 4系(Ubuntu 18.04/Python 3.6世代)のため、一般的な
`pip install ultralytics`が入れる汎用PyTorchホイールがそのままでは動かないことが
多い。セットアップ手順は README.md を参照(NVIDIA公式のJetson向けホイールが必要)。

必要なもの:
    pip install -r requirements.txt   (opencv-python, ultralytics)
    ※Jetson Nanoでは requirements.txt の通りにやると失敗しやすい。
      README.md の「Jetson Nanoでの準備」を先に読むこと。

実行:
    python person_count_poc.py
    (カメラは自動検出される。--cameraで明示指定も可能)
"""
from __future__ import annotations

import argparse
import os

import cv2
from ultralytics import YOLO

# scripts/setup_udev_rules.sh がRasPi上で作る固定名(nokosahen_core/
# camera_counter.pyの既定値と同じ)。Jetson Nanoでも同じ命名規則で固定名を
# 作っている場合はこちらを優先して使う。
KNOWN_FIXED_DEVICE_PATHS = ["/dev/webcam_checkout"]


def find_camera(max_index: int = 5) -> "str | int":
    """--cameraが指定されなかった場合にカメラを自動で選ぶ。

    1. まず固定デバイス名(/dev/webcam_checkout等)が存在すればそれを使う。
    2. 無ければ、実際に映像が取れるカメラのインデックスを0から順に探して選ぶ。
    見つからなければエラーにする。
    """
    print("カメラを自動検出しています...")
    for path in KNOWN_FIXED_DEVICE_PATHS:
        if os.path.exists(path):
            cap = cv2.VideoCapture(path)
            ok = cap.isOpened() and cap.read()[0]
            cap.release()
            if ok:
                print(f"  -> 固定デバイス名 {path} を使用します")
                return path
            print(f"  ({path} は存在するが映像を取得できなかったのでスキップ)")

    for index in range(max_index + 1):
        cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            cap.release()
            continue
        ok, _ = cap.read()
        cap.release()
        if ok:
            print(f"  -> カメラindex={index} を使用します")
            return index
    raise RuntimeError(
        f"固定デバイス名({', '.join(KNOWN_FIXED_DEVICE_PATHS)})・"
        f"index 0〜{max_index}のどれからも映像を取得できませんでした。"
        "カメラが接続されているか確認してください。"
        "見つかったカメラの番号/パスが分かっている場合は --camera <番号 or パス> で直接指定できます。"
    )


def _camera_arg(value: str) -> "str | int":
    try:
        return int(value)
    except ValueError:
        return value


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Jetson Nano向け人物カウントPoC")
    parser.add_argument(
        "--camera", type=_camera_arg, default=None,
        help="カメラのインデックス(例: 1)またはデバイスパス。指定しなければ自動検出(既定)",
    )
    parser.add_argument(
        "--model-path", default="yolov8n.pt",
        help="ultralytics YOLOのモデルファイル(既定yolov8n.pt、初回自動ダウンロード)",
    )
    parser.add_argument(
        "--line-x-ratio", type=float, default=0.5,
        help="カウント判定ラインのX位置(0.0〜1.0、既定0.5=中央)",
    )
    parser.add_argument(
        "--direction", choices=["left_to_right", "right_to_left", "both"], default="left_to_right",
        help="ラインをどちら向きにまたいだときだけカウントするか(既定left_to_right)",
    )
    parser.add_argument(
        "--min-person-height", type=int, default=400,
        help="これより小さい(px)人物は無視する(既定400。値を大きくするほど手前しか反応しなくなる)",
    )
    parser.add_argument("--min-confidence", type=float, default=0.35)
    parser.add_argument("--min-track-hits", type=int, default=3)
    parser.add_argument("--track-stale-frames", type=int, default=60)
    parser.add_argument(
        "--no-display", action="store_true",
        help="映像ウィンドウを出さない(SSH経由でディスプレイが無い環境向け。ターミナルにカウントだけ出す)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    print(f"YOLOモデルを読み込んでいます({args.model_path})...")
    model = YOLO(args.model_path)

    camera_index = args.camera if args.camera is not None else find_camera()
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"カメラ(index={camera_index})を開けませんでした")

    counter = 0
    counted_ids: set[int] = set()
    track_hits: dict[int, int] = {}
    last_seen: dict[int, int] = {}
    prev_center_x: dict[int, float] = {}
    frame_count = 0

    print("起動したよ！終了するには 'q' (--no-display時はCtrl+C) を押してね。")

    while cap.isOpened():
        ok, frame = cap.read()
        if not ok:
            print("フレーム取得に失敗しました")
            break

        frame_count += 1
        h, w, _ = frame.shape
        line_x = int(w * args.line_x_ratio)

        results = model.track(frame, persist=True, classes=[0], verbose=False)

        if results and results[0].boxes.id is not None:
            boxes = results[0].boxes.xyxy.cpu().numpy()
            confs = results[0].boxes.conf.cpu().numpy()
            track_ids = results[0].boxes.id.int().cpu().tolist()

            for box, conf, track_id in zip(boxes, confs, track_ids):
                if float(conf) < args.min_confidence:
                    continue
                x1, y1, x2, y2 = map(int, box)
                person_height = y2 - y1
                if person_height < args.min_person_height:
                    continue

                cx = int((x1 + x2) / 2)
                last_seen[track_id] = frame_count
                track_hits[track_id] = track_hits.get(track_id, 0) + 1

                prev_x = prev_center_x.get(track_id)
                is_stable_track = track_hits[track_id] >= args.min_track_hits

                crossed_line = False
                if prev_x is not None:
                    if args.direction in ("left_to_right", "both") and prev_x < line_x <= cx:
                        crossed_line = True
                    if args.direction in ("right_to_left", "both") and prev_x > line_x >= cx:
                        crossed_line = True

                if crossed_line and is_stable_track and track_id not in counted_ids:
                    counter += 1
                    counted_ids.add(track_id)
                    print(f"【検知】ID:{track_id} が通過！ 合計: {counter}人")

                prev_center_x[track_id] = cx

                if not args.no_display:
                    color = (0, 255, 0) if track_id in counted_ids else (255, 180, 0)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cy = int((y1 + y2) / 2)
                    cv2.circle(frame, (cx, cy), 5, color, -1)
                    cv2.putText(
                        frame, f"ID:{track_id} H:{person_height}", (x1, max(20, y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
                    )

        stale_ids = [tid for tid, seen in last_seen.items() if frame_count - seen > args.track_stale_frames]
        for tid in stale_ids:
            last_seen.pop(tid, None)
            prev_center_x.pop(tid, None)
            track_hits.pop(tid, None)

        if not args.no_display:
            cv2.line(frame, (line_x, 0), (line_x, h), (0, 0, 255), 2)
            cv2.putText(frame, f"Total: {counter}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow("jetson_nano_person_count_poc", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    if not args.no_display:
        cv2.destroyAllWindows()
    print(f"終了しました。最終カウント: {counter}")


if __name__ == "__main__":
    main()
