#!/usr/bin/env python3
"""明電舎タブレットの累計販売数(0-9の数字)を認識するための、Coral Edge TPU向け
単一桁分類CNNを合成データで学習し、Edge TPU用にコンパイルするスクリプト。

【重要な限界】実際のタブレット画面の実写真・実動画を学習に使っていない
(手元に実機サンプルが無いため)。フォント・幾何変形・二値化ノイズを
ランダムに変えた合成データで学習しているため、実機での精度は未検証。
実際にRaspberry Pi+カメラ+本物のタブレットが使えるようになったら、
必ず実データでの精度確認・必要ならファインチューニングを行うこと。

設計方針:
- tablet_ocr_reader.py の preprocess_for_ocr()(グレースケール化→2倍拡大→
  大津の二値化)と同じ「二値化後の画像」を分類対象にする。二値化後は
  フォントや照明条件による差が小さくなり、合成データでの学習が実機へも
  汎化しやすくなるという判断。
- 1文字ずつの分類(0-9の10クラス)にする。複数桁の文字列(「154」等)は
  tablet_ocr_reader.py 側で輪郭抽出により1文字ずつ切り出してから、この
  モデルへ渡す設計(DIGIT_CANVAS_SIZE・DIGIT_PADDING_RATIOは向こう側と
  揃えること)。

使い方:
    python3 train_digit_model.py --out-dir ~/.nokosahen_digit_model
    (WSL/Linux + TensorFlow が必要。Edge TPUコンパイルには別途
    edgetpu_compiler が必要で、無ければ量子化済みtfliteまでを出力する)
"""
from __future__ import annotations

import argparse
import os
import random
import shutil
import subprocess
from pathlib import Path
from typing import List, Tuple

import numpy as np
from PIL import Image, ImageDraw, ImageFont

# tablet_ocr_reader.py側の文字切り出し・パディングと必ず揃えること。
DIGIT_CANVAS_SIZE = 32
DIGIT_PADDING_RATIO = 0.15

# このマシン(WSL/Ubuntu)で確認済みのフォント。太字・等幅・プロポーショナル
# を混ぜて汎化性を持たせる(実際のタブレットのフォントは不明なため)。
FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    "/usr/share/fonts/truetype/noto/NotoSans-Bold.ttf",
    "/usr/share/fonts/truetype/noto/NotoSansMono-Bold.ttf",
    "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
]


def _render_digit_raw(digit: int, font_path: str, size: int = 96) -> Image.Image:
    """1桁を大きめのキャンバスに描画してから返す(後で回転・縮小するため)。"""
    canvas = Image.new("L", (size, size), color=255)
    draw = ImageDraw.Draw(canvas)
    font_size = int(size * random.uniform(0.55, 0.8))
    font = ImageFont.truetype(font_path, font_size)
    text = str(digit)
    bbox = draw.textbbox((0, 0), text, font=font)
    w, h = bbox[2] - bbox[0], bbox[3] - bbox[1]
    x = (size - w) / 2 - bbox[0] + random.uniform(-size * 0.04, size * 0.04)
    y = (size - h) / 2 - bbox[1] + random.uniform(-size * 0.04, size * 0.04)
    draw.text((x, y), text, font=font, fill=0)
    return canvas


def _augment(img: Image.Image) -> Image.Image:
    """回転・拡縮・ぼかし・ノイズ・二値化で、タブレット撮影+二値化後の
    見た目に近づける(実写真が無いため、幾何・ノイズのランダム化で
    ある程度の汎化を狙う)。
    """
    angle = random.uniform(-12, 12)
    img = img.rotate(angle, resample=Image.BICUBIC, fillcolor=255)

    if random.random() < 0.5:
        from PIL import ImageFilter
        img = img.filter(ImageFilter.GaussianBlur(radius=random.uniform(0.3, 1.2)))

    arr = np.array(img).astype(np.float32)
    if random.random() < 0.7:
        noise = np.random.normal(0, random.uniform(5, 20), arr.shape)
        arr = arr + noise
    # 大津の二値化相当(合成データでは閾値をランダムに揺らして、実際の
    # 二値化のブレを模擬する)。
    threshold = np.clip(np.mean(arr) + random.uniform(-15, 15), 60, 200)
    arr = np.where(arr < threshold, 0, 255).astype(np.uint8)
    return Image.fromarray(arr, mode="L")


def _pad_and_resize(img: Image.Image, canvas_size: int, padding_ratio: float) -> Image.Image:
    """文字部分を検出し、周囲に余白を付けてcanvas_sizeへ正規化する。
    tablet_ocr_reader.py側の文字切り出し後の正規化と揃えること。
    """
    arr = np.array(img)
    ys, xs = np.where(arr < 128)
    if len(xs) == 0 or len(ys) == 0:
        cropped = img
    else:
        x0, x1 = max(0, xs.min() - 1), min(arr.shape[1], xs.max() + 2)
        y0, y1 = max(0, ys.min() - 1), min(arr.shape[0], ys.max() + 2)
        cropped = img.crop((x0, y0, x1, y1))

    side = max(cropped.size)
    pad = int(side * padding_ratio)
    square = Image.new("L", (side + pad * 2, side + pad * 2), color=255)
    square.paste(cropped, (pad + (side - cropped.size[0]) // 2, pad + (side - cropped.size[1]) // 2))
    return square.resize((canvas_size, canvas_size), Image.BICUBIC)


def generate_dataset(samples_per_digit: int) -> Tuple[np.ndarray, np.ndarray]:
    fonts = [f for f in FONT_CANDIDATES if os.path.exists(f)]
    if not fonts:
        raise RuntimeError("利用可能なフォントが見つかりません。FONT_CANDIDATESを確認してください。")

    images: List[np.ndarray] = []
    labels: List[int] = []
    for digit in range(10):
        for _ in range(samples_per_digit):
            font_path = random.choice(fonts)
            raw = _render_digit_raw(digit, font_path)
            aug = _augment(raw)
            norm = _pad_and_resize(aug, DIGIT_CANVAS_SIZE, DIGIT_PADDING_RATIO)
            images.append(np.array(norm, dtype=np.uint8))
            labels.append(digit)

    x = np.stack(images).astype(np.float32) / 255.0
    x = x[..., np.newaxis]
    y = np.array(labels, dtype=np.int64)
    return x, y


def build_model():
    import tensorflow as tf
    from tensorflow.keras import layers, models

    model = models.Sequential([
        layers.Input(shape=(DIGIT_CANVAS_SIZE, DIGIT_CANVAS_SIZE, 1)),
        layers.Conv2D(16, 3, activation="relu", padding="same"),
        layers.MaxPooling2D(),
        layers.Conv2D(32, 3, activation="relu", padding="same"),
        layers.MaxPooling2D(),
        layers.Conv2D(32, 3, activation="relu", padding="same"),
        layers.Flatten(),
        layers.Dense(64, activation="relu"),
        layers.Dropout(0.3),
        layers.Dense(10, activation="softmax"),
    ])
    model.compile(optimizer="adam", loss="sparse_categorical_crossentropy", metrics=["accuracy"])
    return model


def convert_to_tflite_int8(model, x_val: np.ndarray, out_path: Path) -> None:
    import tensorflow as tf

    def representative_dataset():
        for i in range(min(300, len(x_val))):
            yield [x_val[i : i + 1].astype(np.float32)]

    # 【2026-09-24追記】layers.Input(shape=...)はKerasが自動でbatch次元を
    # Noneにするため、model単体をfrom_keras_modelへ渡すと入力形状が
    # (None, 32, 32, 1)の「動的サイズ」のままtfliteへ変換されてしまう。
    # Edge TPU Compilerは静的サイズのテンソルしか受け付けず
    # ("Attempting to use a delegate that only supports static-sized
    # tensors..."で失敗する)、CPU(tfliteそのまま)では動くのにEdge TPUだけ
    # 変換に失敗する原因になっていた。
    # 【試行錯誤の記録】最初はtf.function+get_concrete_function経由で
    # batch=1を固定しようとしたが、Keras 3のresource variable(重み)が
    # 量子化キャリブレーション時に正しく解決されず
    # "READ_VARIABLE failed to invoke"で失敗した。代わりに、同じ構造で
    # batch_size=1を明示したInputを持つ新しいモデルを作り、重みだけコピー
    # してfrom_keras_model()に渡す(通常の変換パスなので上記の問題を避けられる)。
    from tensorflow.keras import Input, Model

    fixed_input = Input(
        batch_shape=(1,) + tuple(model.inputs[0].shape[1:]), dtype=model.inputs[0].dtype
    )
    fixed_model = Model(fixed_input, model(fixed_input))
    fixed_model.set_weights(model.get_weights())

    converter = tf.lite.TFLiteConverter.from_keras_model(fixed_model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset = representative_dataset
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type = tf.uint8
    converter.inference_output_type = tf.uint8
    tflite_model = converter.convert()
    out_path.write_bytes(tflite_model)


def try_compile_edgetpu(tflite_path: Path, out_dir: Path) -> bool:
    if shutil.which("edgetpu_compiler") is None:
        return False
    try:
        subprocess.run(
            ["edgetpu_compiler", "-o", str(out_dir), str(tflite_path)],
            check=True,
            capture_output=True,
            text=True,
        )
        return True
    except subprocess.CalledProcessError as exc:
        print(f"edgetpu_compilerでの変換に失敗しました: {exc.stderr}")
        return False


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", default="./digit_model_out")
    parser.add_argument("--samples-per-digit", type=int, default=1500)
    parser.add_argument("--epochs", type=int, default=15)
    args = parser.parse_args()

    out_dir = Path(os.path.expanduser(args.out_dir))
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"合成データを生成中(1桁あたり{args.samples_per_digit}枚)...")
    x, y = generate_dataset(args.samples_per_digit)
    n = len(x)
    idx = np.random.permutation(n)
    x, y = x[idx], y[idx]
    split = int(n * 0.85)
    x_train, y_train = x[:split], y[:split]
    x_val, y_val = x[split:], y[split:]
    print(f"学習データ: {len(x_train)}枚, 検証データ: {len(x_val)}枚")

    model = build_model()
    model.fit(
        x_train, y_train,
        validation_data=(x_val, y_val),
        epochs=args.epochs,
        batch_size=64,
        verbose=2,
    )

    val_loss, val_acc = model.evaluate(x_val, y_val, verbose=0)
    print(f"検証精度(合成データ内): {val_acc:.4f}")
    print("【注意】これは合成データ内での精度であり、実際のタブレット撮影画像での精度ではありません。")

    keras_path = out_dir / "digit_model.keras"
    model.save(keras_path)
    print(f"Kerasモデルを保存しました: {keras_path}")

    tflite_path = out_dir / "digit_model_int8.tflite"
    convert_to_tflite_int8(model, x_val, tflite_path)
    print(f"量子化済みTFLiteモデルを保存しました: {tflite_path}")

    compiled = try_compile_edgetpu(tflite_path, out_dir)
    if compiled:
        print(f"Edge TPU向けコンパイルに成功しました: {out_dir}/digit_model_int8_edgetpu.tflite")
    else:
        print(
            "edgetpu_compilerが見つからないため、Edge TPU向け変換はスキップしました。"
            "install_edgetpu_compiler.sh を実行してから再度このスクリプトを実行するか、"
            f"{tflite_path} を edgetpu_compiler が使える環境へコピーして手動変換してください。"
        )

    with open(out_dir / "accuracy_report.txt", "w", encoding="utf-8") as f:
        f.write(
            f"synthetic_val_accuracy={val_acc:.4f}\n"
            "この精度は合成データ(フォント+幾何/ノイズ拡張)内での検証結果であり、"
            "実際のタブレット撮影画像には未検証。実機テスト時に必ず確認すること。\n"
        )


if __name__ == "__main__":
    main()
