# 決済横カメラ 単体版(Jetson Nano / ROS2なしのRaspberry Pi向け)

**位置づけ**: `ros2_ws/src/nokosahen_checkout/`(ROS2版、Raspberry Pi 4専用)と
**全く同じ判定ロジック**(`crossing_logic.py`: YOLOトラッキング・近接フィルタ・
追跡安定性・方向判定)を使う、ROS2に依存しない単体版です。カウント結果は
ROS2トピックではなく、メインPC(Jetson Orin Nano等)の`web_ui_node`へ
HTTP POST(`/api/count`)するだけなので、ROS2が入っていない機体でもそのまま動きます。

## なぜこれが必要か

Raspberry Pi 4単体でのYOLO推論(CPU)は重く、フレームレートが十分出ないことが
分かったため、以下のどちらかへのハードウェア変更を検討中:

1. **Jetson Nano**(GPUで推論を高速化)
2. **Raspberry Pi 4 + Google Coral USB Accelerator**(Edge TPUで推論を高速化)

このうち **Jetson NanoはJetPack 4系/Ubuntu 18.04世代のため、ROS 2 Humbleの
公式apt環境が無く**、既存のROS2版(`checkout_camera_node.py`)をそのまま
動かせません(`docs/production_setup.md`の既存方針と同じ)。そのため
Jetson Nano向けにはこの単体版を使います。

**Raspberry Pi + Coral TPUの場合は、ROS2版(`checkout_camera_node.py`)を
そのまま使えます**(`model_path`にexport済みの`*_edgetpu.tflite`を指定するだけ。
コード変更は不要)。ただしROS2を入れたくない/軽量にしたい場合は、この単体版を
Raspberry Pi上でそのまま動かすことも可能です(`--model-path`に同じtfliteを指定する)。

## 準備

```bash
pip install -r requirements.txt
```

### Jetson Nano(JetPack)の場合

**重要**: 通常の`pip install ultralytics`が入れる汎用のPyTorchホイールは
Jetson Nano(aarch64 + 古いJetPack)では動かないことが多い。NVIDIA公式の
Jetson向けPyTorchホイールを使うこと:

```bash
# JetPackのバージョンに合ったホイールをNVIDIAのフォーラム/公式ページから確認して導入
# (例、JetPack 4.6.xの場合のイメージ。実際のURLはJetPackバージョンに応じて変える)
pip3 install torch-<version>-cp36-cp36m-linux_aarch64.whl
pip3 install torchvision --no-deps  # 別途ソースビルドが必要な場合あり

pip install opencv-python-headless requests
pip install ultralytics --no-deps   # torch/torchvisionは上で入れた版を使わせる
```

JetPack 4.6.x(古いUbuntu 18.04 + Python 3.6)では最近のultralytics/PyTorchが
そのままでは入らない可能性がある。うまくいかない場合は
[NVIDIA jetson-inference](https://github.com/dusty-nv/jetson-inference)や
[dusty-nv/jetson-containers](https://github.com/dusty-nv/jetson-containers)の
Dockerイメージを使う方法も検討する(このスクリプト自体はコンテナ内でも
そのまま動く、ROS2非依存のため)。

### Coral USB Acceleratorの場合(Raspberry Pi等)

1. Edge TPU用ランタイムを導入(Google公式手順):
   ```bash
   echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
   curl -fsSL https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
   sudo apt update
   sudo apt install libedgetpu1-std python3-pycoral
   ```
2. `yolov8n.pt`をEdge TPU用に変換(**開発用PC上で実行、要インターネット**。
   `edgetpu_compiler`はRaspberry Piでも動くが遅いので、可能なら別PCで変換して
   `.tflite`だけをRaspberry Piへコピーする方が楽):
   ```bash
   pip install ultralytics
   yolo export model=yolov8n.pt format=edgetpu
   # -> yolov8n_full_integer_quant_edgetpu.tflite が生成される
   ```
3. 生成された`*_edgetpu.tflite`をRaspberry Pi側に転送し、`--model-path`
   (またはROS2版なら`model_path`パラメータ)にそのパスを指定する。
   `--device`は無視される(Edge TPUはtfliteの中身からultralyticsが自動判別する)。

**注意**: Edge TPU向けのexportはINT8量子化を伴うため、通常の`.pt`より
検出精度がわずかに落ちることがある。実機で`--min-confidence`の再調整が
必要になる場合がある。

## 実行

```bash
python checkout_camera_standalone.py \
    --menu カレー \
    --main-url http://192.168.1.10:8080
```

Jetson NanoでGPU推論を明示したい場合:

```bash
python checkout_camera_standalone.py \
    --menu カレー \
    --main-url http://192.168.1.10:8080 \
    --device 0
```

Coral USB Acceleratorを使う場合:

```bash
python checkout_camera_standalone.py \
    --menu カレー \
    --main-url http://192.168.1.10:8080 \
    --model-path yolov8n_full_integer_quant_edgetpu.tflite
```

その他のオプション(ROS2版`checkout_camera_node.py`のパラメータと同じ意味):

- `--camera auto` : カメラデバイス(既定autoで`/dev/video*`を自動探索)
- `--line-x-ratio 0.5` : 通過検知に使う縦ラインの位置
- `--direction both` : `both`/`left_to_right`/`right_to_left`
- `--min-person-height-ratio 0.55` : 近接フィルタ(決済端末の真正面と判定する閾値)
- `--min-confidence 0.4` : YOLOの検出信頼度の下限
- `--min-track-hits 3` : 誤検出対策(この回数連続で検出できたtrack_idだけカウント対象にする)
- `--track-stale-frames 60` : このフレーム数だけ見えなくなったtrack_idの追跡情報を破棄
- `--source-tag jetson_nano_camera` : `/api/count`の`source`フィールドに載せる識別名

## 常駐化(Jetson Nano、systemd例)

```ini
[Unit]
Description=Nokosahen checkout camera (standalone, Jetson Nano)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=<ユーザー名>
WorkingDirectory=/home/<ユーザー名>/mirs2603_nokosahen/standalone_checkout_cam
ExecStart=/usr/bin/python3 checkout_camera_standalone.py --menu カレー --main-url http://192.168.1.10:8080 --device 0
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
```

## ROS2版との違い

| | ROS2版(`nokosahen_checkout`) | この単体版 |
|---|---|---|
| 対応ハードウェア | Raspberry Pi 4(単体 or +Coral TPU) | Jetson Nano、または任意のROS2非依存Linux機(+Coral TPU可) |
| カウント結果の送信 | ROS2トピック(`/nokosahen/count_event`) | HTTP POST(`/api/count`) |
| メニュー割り当て | 完成版ダッシュボードから動的に割り当て可能 | 起動時に`--menu`で固定指定 |
| プレビュー映像配信 | あり(ダッシュボードに低画質JPEGを表示) | 無し(このスクリプトはカウントのみ) |
| 判定ロジック本体 | `crossing_logic.py` | 同名・同内容のコピー(`crossing_logic.py`) |

判定ロジック(`crossing_logic.py`)を変更する場合は、**必ず両方のコピーを
同じ内容に保つこと**(`ros2_ws/src/nokosahen_checkout/nokosahen_checkout/crossing_logic.py`
とこのフォルダの`crossing_logic.py`)。
