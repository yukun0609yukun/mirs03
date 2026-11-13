"""決済端末を見て、社員証を持った手が現れた回数を数える PoC(概念実証)。

背景: `face_count_poc`/本番の`nokosahen_checkout`は「人がラインを横切ったか」で
決済を数えているが、絞り込み(近接フィルタ・一方向判定)をしても十分な精度が
出ない場合、そもそも「人全体」ではなく「決済端末に社員証をかざす手」だけを
見た方が誤検出が減るのでは、というアイデアを試すための試作PoC。

**重要な注意(このPoCの限界)**: 「本当に社員証を持っているか」まで判定して
いるわけではない。MediaPipe Handsで手を検出し、決済端末の読み取り位置付近
(画面内のROI矩形)にその手が一定時間とどまったら「かざした」とみなして
数える、というプロキシ(代理)判定。IC/バーコードの読み取り成功と連動させた
い場合は、決済端末側のセンサー(実際の読み取り成功シグナル)と組み合わせる
方が確実(このPoCはあくまで「手の出現回数」の代替案としての実力を見るためのもの)。

ROS2やmirs2603_nokosahenの他のコードには一切依存しない。このフォルダ
(badge_hand_poc/)だけで完結するので、このフォルダごと送れば手元でそのまま
試せる。

必要なもの:
    pip install -r requirements.txt   (opencv-python, mediapipe)

実行:
    python badge_hand_poc.py
    (カメラは自動検出される。手を映す位置に決済端末を置いて、その読み取り
    位置に合わせて--roiで矩形を指定するとより正確に試せる)

終了:
    映像ウィンドウを選択した状態でqキー
"""
from __future__ import annotations

import argparse
import os
import time

import cv2
import mediapipe as mp

# scripts/setup_udev_rules.sh がRasPi上で作る固定名(nokosahen_core/
# camera_counter.pyの既定値と同じ)。挿すUSBポート/順番が変わってもこの名前は
# 変わらないので、存在すればindex探索より優先してこちらを使う。
KNOWN_FIXED_DEVICE_PATHS = ["/dev/webcam_checkout"]


def find_camera(max_index: int = 5) -> "str | int":
    """--cameraが指定されなかった場合にカメラを自動で選ぶ(他PoCと同じロジック)。"""
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
        "カメラが接続されているか確認してください。"
    )


def _camera_arg(value: str) -> "str | int":
    try:
        return int(value)
    except ValueError:
        return value


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="決済端末での手検出カウントPoC")
    parser.add_argument(
        "--camera", type=_camera_arg, default=None,
        help="カメラのインデックスまたはデバイスパス。指定しなければ自動検出する。",
    )
    parser.add_argument(
        "--roi", nargs=4, type=float, default=[0.3, 0.3, 0.4, 0.4],
        metavar=("X", "Y", "W", "H"),
        help="決済端末の読み取り位置とみなすROI矩形(画面比率0.0〜1.0、既定 0.3 0.3 0.4 0.4=中央)。"
             "左上(X,Y)・幅高さ(W,H)の順。",
    )
    parser.add_argument(
        "--min-hits", type=int, default=5,
        help="このフレーム数だけ連続でROI内に手が検出できたら「かざした」と確定する(既定5、誤検出対策)",
    )
    parser.add_argument(
        "--cooldown-sec", type=float, default=1.5,
        help="1回カウントしてから次のカウントまで最低これだけ間隔を空ける(既定1.5秒、同じ手を連続で数えないため)",
    )
    parser.add_argument(
        "--min-detection-confidence", type=float, default=0.6,
        help="MediaPipe Handsの検出信頼度の下限(既定0.6)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    roi_x, roi_y, roi_w, roi_h = args.roi

    camera_index = args.camera if args.camera is not None else find_camera()
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"カメラ(index={camera_index})を開けませんでした")

    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=2,
        min_detection_confidence=args.min_detection_confidence,
        min_tracking_confidence=0.5,
    )
    mp_drawing = mp.solutions.drawing_utils

    counter = 0
    hit_streak = 0
    last_count_time = 0.0
    hand_in_roi_now = False

    print("起動したよ！終了するには 'q' を押してね。")
    print(f"ROI(決済端末の読み取り位置とみなす範囲): x={roi_x} y={roi_y} w={roi_w} h={roi_h}")

    while cap.isOpened():
        ok, frame = cap.read()
        if not ok:
            print("フレーム取得に失敗しました")
            break

        h, w, _ = frame.shape
        roi_px = (
            int(roi_x * w), int(roi_y * h),
            int((roi_x + roi_w) * w), int((roi_y + roi_h) * h),
        )

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb)

        hand_in_roi = False
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                # 手のひら中心の目安として、手首(0番)と中指付け根(9番)の中点を使う。
                wrist = hand_landmarks.landmark[0]
                middle_base = hand_landmarks.landmark[9]
                cx = int((wrist.x + middle_base.x) / 2 * w)
                cy = int((wrist.y + middle_base.y) / 2 * h)
                cv2.circle(frame, (cx, cy), 6, (255, 180, 0), -1)
                if roi_px[0] <= cx <= roi_px[2] and roi_px[1] <= cy <= roi_px[3]:
                    hand_in_roi = True

        if hand_in_roi:
            hit_streak += 1
        else:
            hit_streak = 0

        now = time.monotonic()
        is_stable = hit_streak >= args.min_hits
        if is_stable and not hand_in_roi_now and (now - last_count_time) > args.cooldown_sec:
            counter += 1
            last_count_time = now
            print(f"【検知】社員証をかざす手を検知！ 合計: {counter}回")
        hand_in_roi_now = is_stable

        roi_color = (0, 255, 0) if hand_in_roi_now else (0, 0, 255)
        cv2.rectangle(frame, (roi_px[0], roi_px[1]), (roi_px[2], roi_px[3]), roi_color, 2)
        cv2.putText(frame, f"Total: {counter}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(
            frame, f"hit_streak={hit_streak}", (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2,
        )

        cv2.imshow("badge_hand_poc (payment terminal hand detection)", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    hands.close()
    cap.release()
    cv2.destroyAllWindows()
    print(f"終了しました。最終カウント: {counter}")


if __name__ == "__main__":
    main()
