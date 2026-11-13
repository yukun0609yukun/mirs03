"""YOLO人物トラッキングによるカウントPoC(概念実証)。

Webカメラの映像からYOLOで人物を検出・追跡し、画面内の縦ラインを
指定方向にまたいだ瞬間を1回としてカウントする。`C:\\Users\\user\\Downloads\\camera2.py`
で確認できたロジック(model.track(persist=True)によるtrack_id単位の追跡・
最小検出高さでの近接フィルタ・追跡の安定性チェック)をベースにしている。
本番側(nokosahen_core/camera_counter.py)にも同じロジックを移植済み。

ROS2やmirs2603_nokosahenの他のコードには一切依存しない。このフォルダ
(face_count_poc/)だけで完結するので、このフォルダごと送れば手元でそのまま
試せる。

必要なもの:
    pip install -r requirements.txt   (opencv-python, ultralytics)
    初回実行時にyolov8n.ptが自動ダウンロードされる(要インターネット接続)。

実行:
    python face_count_poc.py
    (カメラは自動検出される。scripts/setup_udev_rules.sh設定済みのRasPiなら
    固定名/dev/webcam_checkoutを優先して使う。手動で指定したい場合は
    --camera 1 や --camera /dev/webcam_checkout のように指定)

終了:
    映像ウィンドウを選択した状態でqキー

やっていること:
    画面内の縦ライン(既定で中央)を、YOLOで追跡している人物の中心が
    指定方向にまたいだ瞬間を1回としてカウントし、ターミナルに表示する。
    同一人物の対応付けはYOLOの追跡ID(track_id)をそのまま使うので、
    顔認識+座標近接マッチングだった旧版より複数人が同時に映る状況に強い。
    ただし奥に小さく映る人・track_idが一瞬しか出ない誤検出を除外するため、
    最小検出高さ・最小追跡フレーム数のフィルタを設けている。
"""
from __future__ import annotations

import argparse
import os

import cv2
from ultralytics import YOLO

# scripts/setup_udev_rules.sh がRasPi上で作る固定名(nokosahen_core/
# camera_counter.pyの既定値と同じ)。挿すUSBポート/順番が変わってもこの名前は
# 変わらないので、存在すればindex探索より優先してこちらを使う。
KNOWN_FIXED_DEVICE_PATHS = ["/dev/webcam_checkout"]


def find_camera(max_index: int = 5) -> "str | int":
    """--cameraが指定されなかった場合にカメラを自動で選ぶ。

    1. まずscripts/setup_udev_rules.shが作る固定名(/dev/webcam_checkout等)が
       存在すればそれを使う(udevルール設定済みのRasPiではポート/順番に
       関係なく毎回同じデバイスになる)。
    2. 無ければ、実際に映像が取れるカメラのインデックスを0から順に探して選ぶ
       (udev未設定の環境、Windows/Mac向けのフォールバック)。
    見つからなければエラーにする。
    """
    print("カメラを自動検出しています...")
    for path in KNOWN_FIXED_DEVICE_PATHS:
        if os.path.exists(path):
            cap = cv2.VideoCapture(path)
            ok = cap.isOpened() and cap.read()[0]
            cap.release()
            if ok:
                print(f"  -> 固定デバイス名 {path} を使用します(setup_udev_rules.sh設定済み)")
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
        "カメラが接続されているか、他のアプリがカメラを使用中でないか確認してください。"
        "見つかったカメラの番号/パスが分かっている場合は --camera <番号 or パス> で直接指定できます。"
    )


def _camera_arg(value: str) -> "str | int":
    """--cameraはindex(整数)でもデバイスパス(/dev/video0等)でも受け付ける。"""
    try:
        return int(value)
    except ValueError:
        return value


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLO人物トラッキングカウントPoC")
    parser.add_argument(
        "--camera", type=_camera_arg, default=None,
        help="カメラのインデックス(例: 1)またはデバイスパス(例: /dev/webcam_checkout)。"
             "指定しなければ自動検出する(既定)。",
    )
    parser.add_argument(
        "--model-path", default="yolov8n.pt",
        help="ultralytics YOLOのモデルファイル(既定yolov8n.pt、初回自動ダウンロード)。"
             "Coral USB Acceleratorで試す場合はexport済みの*_edgetpu.tfliteを指定する。",
    )
    parser.add_argument(
        "--device", default="",
        help="推論デバイス(既定は空でultralyticsに自動選択させる)。"
             "Jetson等でGPU(CUDA)を明示したい場合は'0'を指定する。"
             "Coral TPU使用時(*_edgetpu.tflite指定時)はこの値は無視される。",
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
        help="これより小さい(px)人物は無視する(既定400。大きいほど手前しか反応しなくなる)",
    )
    parser.add_argument(
        "--min-confidence", type=float, default=0.35,
        help="YOLOの検出信頼度がこれ未満なら無視する(既定0.35)",
    )
    parser.add_argument(
        "--min-track-hits", type=int, default=3,
        help="この回数だけ連続で検出できたtrack_idだけをカウント対象にする(既定3、誤検出対策)",
    )
    parser.add_argument(
        "--track-stale-frames", type=int, default=60,
        help="このフレーム数だけ見えなくなったtrack_idの追跡情報を破棄する(既定60)",
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

    print("起動したよ！終了するには 'q' を押してね。")

    while cap.isOpened():
        ok, frame = cap.read()
        if not ok:
            print("フレーム取得に失敗しました")
            break

        frame_count += 1
        h, w, _ = frame.shape
        line_x = int(w * args.line_x_ratio)

        results = model.track(
            frame, persist=True, classes=[0], device=(args.device or None), verbose=False
        )

        if results and results[0].boxes.id is not None:
            boxes = results[0].boxes.xyxy.cpu().numpy()
            confs = results[0].boxes.conf.cpu().numpy()
            track_ids = results[0].boxes.id.int().cpu().tolist()

            for box, conf, track_id in zip(boxes, confs, track_ids):
                if float(conf) < args.min_confidence:
                    continue
                x1, y1, x2, y2 = map(int, box)

                # 人の「高さ」を計算(奥にいる人は小さく、手前にいる人は大きく映る)
                person_height = y2 - y1

                # 近接フィルタ: 小さく映っている(奥にいる)人は処理をスキップ
                if person_height < args.min_person_height:
                    continue

                cx = int((x1 + x2) / 2)
                last_seen[track_id] = frame_count
                track_hits[track_id] = track_hits.get(track_id, 0) + 1

                prev_x = prev_center_x.get(track_id)
                is_stable_track = track_hits[track_id] >= args.min_track_hits

                # 一方通行(または両方向)の判定
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

                # 画面の描画処理
                color = (0, 255, 0) if track_id in counted_ids else (255, 180, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                cy = int((y1 + y2) / 2)
                cv2.circle(frame, (cx, cy), 5, color, -1)

                # デバッグ用に現在の高さ(H)も画面に表示
                cv2.putText(
                    frame, f"ID:{track_id} H:{person_height}", (x1, max(20, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
                )

        stale_ids = [tid for tid, seen in last_seen.items() if frame_count - seen > args.track_stale_frames]
        for tid in stale_ids:
            last_seen.pop(tid, None)
            prev_center_x.pop(tid, None)
            track_hits.pop(tid, None)

        # カウントラインを描画
        cv2.line(frame, (line_x, 0), (line_x, h), (0, 0, 255), 2)
        cv2.putText(frame, f"Total: {counter}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("face_count_poc (YOLO person tracking)", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"終了しました。最終カウント: {counter}")


if __name__ == "__main__":
    main()
