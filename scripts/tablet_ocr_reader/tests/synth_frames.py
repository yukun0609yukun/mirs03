"""テスト用の合成画像(2026-10-10追加)。模擬POSタブレット(7行・赤茶色の数字)をcv2だけで描き、
カメラ映像の劣化(ピンぼけ・ノイズ・低コントラスト)を加える。実カメラ・ブラウザ不要。
実際の模擬POS画面(mock_pos_tablet_original.html)をPlaywrightで撮った画像での検証は
docs/ocr_camera_tuning.md の「オフライン検証」に結果を記載している。
"""
import cv2
import numpy as np

ROW_VALUES = [118, 129, 353, 166, 211, 178, 103]
DIGIT_BGR = (46, 67, 184)  # 赤茶色(模擬POSの数字色 #B8432E 相当)


def mock_screen(width: int = 392, height: int = 548, values=None) -> np.ndarray:
    """縦長の模擬画面(7行、左に黒い丸+名前、右に赤茶色の太い数字)を返す(BGR)。"""
    values = values or ROW_VALUES
    img = np.zeros((height, width, 3), np.uint8)
    row_h = height / 7.0
    for i, v in enumerate(values):
        y0, y1 = int(i * row_h), int((i + 1) * row_h)
        img[y0:y1] = (251, 242, 234) if i % 2 == 0 else (248, 233, 219)
        cy = (y0 + y1) // 2
        cv2.circle(img, (int(width * 0.07), cy), int(row_h * 0.19), (140, 63, 74), -1)
        cv2.putText(img, "Menu", (int(width * 0.16), cy + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (35, 33, 27), 2, cv2.LINE_AA)
        text = str(v)
        scale = row_h / 38.0
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, scale, 5)
        cv2.putText(img, text, (int(width * 0.95) - tw, cy + th // 2), cv2.FONT_HERSHEY_DUPLEX, scale,
                    DIGIT_BGR, 5, cv2.LINE_AA)
    return img


def camera_frame(screen: np.ndarray, width: int = 1280, height: int = 960, fill: float = 0.8) -> np.ndarray:
    """画面をカメラ映像(暗い背景の中央)へ置く。fill=画面の高さが映像の高さに占める割合。"""
    h = int(height * fill)
    w = int(screen.shape[1] * h / screen.shape[0])
    s = cv2.resize(screen, (w, h), interpolation=cv2.INTER_AREA)
    f = np.full((height, width, 3), (40, 42, 40), np.uint8)
    x0, y0 = (width - w) // 2, (height - h) // 2
    f[y0:y0 + h, x0:x0 + w] = s
    return f


def degrade(frame: np.ndarray, sigma: float = 0.0, noise: float = 0.0, contrast: float = 1.0, seed: int = 0):
    """ピンぼけ(ガウスσ)・低コントラスト(128中心に縮小)・ガウスノイズ(標準偏差)を加えた画像。"""
    g = frame.astype(np.float32)
    if sigma:
        g = cv2.GaussianBlur(g, (0, 0), sigma)
    g = (g - 128.0) * contrast + 128.0
    if noise:
        g = g + np.random.default_rng(seed).normal(0.0, noise, g.shape)
    return np.clip(g, 0, 255).astype(np.uint8)
