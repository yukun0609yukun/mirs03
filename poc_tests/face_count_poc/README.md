# 人物カウント PoC(YOLOトラッキング版)

**目的**: 決済横の通過カウントを、YOLOの人物検出+トラッキング
(`model.track(persist=True)`)でどの程度実用になるか試すための概念実証(PoC)です。
本番側(`nokosahen_core/camera_counter.py`)にも同じロジックを移植しています。
このフォルダだけで完結していて、ROS2やプロジェクト本体には一切依存しません。
**このフォルダごとそのまま送れば、受け取った人はこれだけで試せます。**

以前はOpenCVのHaar Cascadeによる顔認識+座標近接マッチングの簡易版でしたが、
実機確認で有効だったYOLOトラッキング方式(`C:\Users\user\Downloads\camera2.py`)に
差し替えています。顔ではなく人物全体を検出するため横向き・後ろ向きでも検出でき、
YOLOの追跡ID(track_id)をそのまま使うので複数人が同時に映っても取り違えにくく
なっています。

## 準備

### Windows/Mac/一般的なPC

```bash
pip install -r requirements.txt
```

### Raspberry Pi

opencv自体は**apt版を先に試す**(pipとaptを両方使うとバージョン不一致で
クラッシュする、下記トラブルシュート参照)。ただし`ultralytics`はaptパッケージが
無いため、こちらは基本的にpip(仮想環境推奨)でインストールする。

```bash
sudo apt update
sudo apt install -y python3-opencv libatlas-base-dev
```

これでcv2がimportできれば(`python3 -c "import cv2"`がエラーなく通れば)完了。
続けて`ultralytics`だけをpipで追加する(**opencv-pythonは追加しないこと**。
apt版と衝突して下記のNumPyエラーの原因になる)。

```bash
pip install --break-system-packages ultralytics
# または仮想環境内で: pip install ultralytics
```

**注意**: `ultralytics`はPyTorchに依存する比較的重いライブラリで、Raspberry Pi
(特にPi 4)だと推論が遅くなる(数FPS程度)ことがある。実用速度が出るか自体も
このPoCで確認したいポイントの一つ。

もしopencvも含めて全部pipで入れる必要がある場合(aptで入らない環境等)は、最近の
Raspberry Pi OS(Debian 12/bookworm以降)だと素の`pip install`は
`externally-managed-environment`エラーで拒否されるため、以下のいずれかが必要:

```bash
# 方法1: システム全体を明示的に上書きする(手軽だが自己責任)
pip install --break-system-packages -r requirements.txt

# 方法2(推奨): 仮想環境を切って、その中だけで完結させる
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

カメラのアクセス権限が無い(`Permission denied`等)場合は、実行ユーザーを
`video`グループに入れてから**再ログイン**する:

```bash
sudo usermod -aG video $USER
```

### Raspberry Pi 4 + Google Coral USB Accelerator(TPU)で試す場合

上記のRasPi手順(opencv+ultralytics導入)を先に済ませた上で、以下を追加する。
`--model-path`に`*_edgetpu.tflite`を指定するだけで、コード変更なしにこのPoCも
本番の`camera_counter.py`もCoral TPUで動く(ultralyticsがファイル名の
`_edgetpu`を見て自動的にEdge TPUバックエンドへ切り替える。`--device`は
この場合無視される)。

1. **Coral Edge TPUランタイムをRasPiに導入する**(初回のみ、`libedgetpu1-std`は
   Google公式リポジトリから):
   ```bash
   echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | \
       sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
   curl -fsSL https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
   sudo apt update
   sudo apt install -y libedgetpu1-std
   ```
   (発熱・処理速度を優先したい場合は`libedgetpu1-std`の代わりに
   `libedgetpu1-max`もあるが、USB接続だとかなり熱くなるので`-std`推奨)
2. **推論ランタイムを追加する**(ultralyticsのEdge TPUバックエンドが使う):
   ```bash
   pip install --break-system-packages tflite-runtime
   ```
3. **モデルをEdge TPU向けにexportする**(RasPi上ではなく、Windows/PCや
   Jetsonなど普通にultralytics+PyTorchが動く環境で1回行えばよい。変換ツール
   [ONNX/TensorFlow]が必要なため非力なRasPi上では現実的でない):
   ```bash
   yolo export model=yolov8n.pt format=edgetpu
   # -> yolov8n_full_integer_quant_edgetpu.tflite 等が生成される
   ```
   生成された`*_edgetpu.tflite`ファイルをRasPiへコピーする。
4. **Coral USB Acceleratorを挿してから**実行する:
   ```bash
   python face_count_poc.py --model-path yolov8n_full_integer_quant_edgetpu.tflite
   ```
   (`tegrastats`のGPU使用率確認と同様に、Coral使用時は`ls -la /dev/bus/usb`や
   `lsusb`で`Global Unichip Corp.`のCoral機器が見えているかも確認するとよい)

**Jetson Nanoで試したい場合**は、こちらではなく`poc_tests/jetson_nano_person_count_poc/`
(JetPack向けのPyTorchインストール手順込み)を使うこと。「Raspi4+Coral」と
「Jetson Nano」のどちらが実用になるかを比較できるよう、両方とも同じ
YOLOトラッキング判定ロジック(このPoCと同じコード)を使っている。

### トラブルシュート: `cv2`のimportで`NumPy 1.x/2.x`エラーが出る

```
A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x...
AttributeError: _ARRAY_API not found
```

これは、**pipでインストールした`opencv-python`(NumPy 2.x同梱)と、apt版の
`python3-opencv`(NumPy 1.x向けにビルド済み)が両方入っていて、pip版が
importで優先されてしまっている**ときに起きる(エラーメッセージのパスが
`/usr/local/lib/python3.../dist-packages/cv2/...`になっていたらpip版が
使われている証拠)。`sudo apt install --reinstall`でapt版を入れ直しても、
pip版が残っている限りこのエラーは直らない。

**直し方**: pip版を明示的に削除して、apt版だけを残す。

```bash
python3 -m pip uninstall --break-system-packages -y opencv-python opencv-contrib-python numpy
sudo apt install --reinstall -y python3-opencv python3-numpy
python3 -c "import cv2; print(cv2.__version__)"
```

## 実行

```bash
python face_count_poc.py
```

**カメラは自動検出されます**(`--camera`を指定しなければ):

1. まず`scripts/setup_udev_rules.sh`を設定済みのRasPiであれば、固定名
   `/dev/webcam_checkout`(メインプログラムの`camera_counter.py`と同じ名前)を
   優先して使う。挿すUSBポート/順番が変わっても常に同じデバイスになる。
2. 固定名が無い/開けない場合は、index 0〜5を順に試して最初に映像が取れた
   ものを自動で使う(Windows/Mac、またはudev未設定の環境向け)。

特定のカメラを明示的に使いたい場合のみ指定する(番号でもデバイスパスでも可):

```bash
python face_count_poc.py --camera 1
python face_count_poc.py --camera /dev/webcam_checkout
```

その他のオプション:

- `--model-path yolov8n.pt` : ultralytics YOLOのモデルファイル(既定yolov8n.pt、初回実行時に自動ダウンロード)
- `--line-x-ratio 0.5` : カウント判定に使う縦ラインの位置(0.0=左端〜1.0=右端、既定0.5=中央)
- `--direction left_to_right` : `left_to_right`/`right_to_left`/`both`のいずれか(既定left_to_right、指定方向にラインを跨いだときだけカウント)
- `--min-person-height 400` : これより小さい(px)人物は無視する(既定400。値を大きくするほど手前[近く]しか反応しなくなる)
- `--min-confidence 0.35` : YOLOの検出信頼度がこれ未満なら無視する(既定0.35)
- `--min-track-hits 3` : この回数だけ連続で検出できたtrack_idだけをカウント対象にする(既定3、誤検出対策)
- `--track-stale-frames 60` : このフレーム数だけ見えなくなったtrack_idの追跡情報を破棄する(既定60)

## 終了方法

映像ウィンドウを選択した状態で `q` キー。

## 見るべきポイント

- 指定方向(既定は左→右)にラインをまたいだ瞬間、ターミナルに
  `【検知】ID:n が通過！ 合計: n人` と表示されるか
- 誤検出・見逃しがどのくらい起きるか(YOLOの人物検出自体はHaar Cascadeの顔検出と
  違い、横向き・後ろ向きでも検出できるはずなので、そこが改善しているか確認する)
- 複数人が同時に映ったときに、カウントがずれないか(YOLOの追跡ID[track_id]を
  そのまま使っているので、旧版[座標近接マッチング]より人が密集した状況に強いはず。
  実際どの程度か確認する)
- `--min-person-height`の値を変えながら、決済端末の脇を通った人だけを
  正しく拾えるか(値が小さすぎると奥の通行人まで拾ってしまい、大きすぎると
  誰も拾えなくなる)
- 実際に運用する予定の距離・角度・明るさで、どの程度の精度・速度(FPS)が出るか

## 既知の制約(承知の上で試してください)

- YOLOの推論自体が重いため、非力なマシン(特にRasPi)ではフレームレートが
  大きく落ちる可能性がある
- track_idはYOLOの追跡アルゴリズム(ByteTrack、ultralytics既定)任せのため、
  一瞬隠れる・重なる等でIDが振り直されることがある(`--min-track-hits`で
  多少緩和しているが、完全には防げない)
- あくまで「この方式で使い物になるかどうか」を判断するためのPoCであり、
  そのまま本番に組み込む前提のコードではない(本番側には同じロジックを
  `nokosahen_core/camera_counter.py`として別途移植済み)
