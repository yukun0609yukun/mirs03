# 人物カウント PoC(Jetson Nano版)

**目的**: 決済横のカウントを、YOLOの人物検出+トラッキング(`model.track(persist=True)`)で
**Jetson Nano単体**でどの程度実用になるか(認識精度・処理速度)を確認するための
概念実証(PoC)です。判定ロジックは`poc_tests/face_count_poc/`・本番プログラム
(`nokosahen_core/camera_counter.py`・`nokosahen_checkout`)と同じものを使っています。

ROS2やmirs2603_nokosahenの他のコードには一切依存しません。**このフォルダごと
そのまま送れば、Jetson Nanoを持っているメンバーがこれだけで試せます。**

## Jetson Nanoでの準備(ここが一番重要)

Jetson NanoはJetPack 4系(Ubuntu 18.04、Python 3.6)という古い世代のため、
**普通に`pip install ultralytics`すると失敗するか、GPUが使われずCPUだけで
動いてしまう**ことが多い。以下の順番で入れること。

### 1. JetPack自体の確認

```bash
cat /etc/nv_tegra_release
```

JetPack 4.6.x系(L4T 32.7.x)であることを前提にした手順です。バージョンが
違う場合は下記のリンク先を、お使いのJetPackバージョンに合わせて読み替えてください。

### 2. NVIDIA公式のPyTorchホイールを導入

汎用のpip版PyTorchはJetson(aarch64)では動かないので、NVIDIAが配布している
Jetson専用ホイールを使う。以下は一例(実際のURLはJetPackバージョンに応じて変わるため、
[NVIDIA公式フォーラムのPyTorch for Jetsonスレッド](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048)
で自分のJetPackバージョンに合うものを確認すること):

```bash
sudo apt update
sudo apt install -y python3-pip libopenblas-base libopenmpi-dev

# 例(JetPack 4.6.x向け。バージョンは要確認)
wget <NVIDIA公式フォーラムに掲載されているtorch-*.whlのURL> -O torch.whl
pip3 install torch.whl

# torchvisionはソースビルドが必要な場合が多い(公式フォーラムの手順に従う)
```

### 3. OpenCV・ultralyticsの導入

```bash
sudo apt install -y python3-opencv
pip3 install ultralytics --no-deps  # torch/torchvisionは手順2で入れた版を使わせる
pip3 install requests  # ultralyticsが依存する軽量パッケージ(念のため)
```

`--no-deps`を付けないと、ultralyticsが依存解決で汎用torchを再インストールしようと
して手順2の苦労が無駄になることがあるので注意。

### 4. カメラのアクセス権限

```bash
sudo usermod -aG video $USER
```

(設定後は一度再ログインが必要)

### うまくいかない場合

上記がうまくいかない/JetPack 4.6.xと合うホイールが見つからない場合、
[NVIDIA jetson-inference](https://github.com/dusty-nv/jetson-inference)や
[dusty-nv/jetson-containers](https://github.com/dusty-nv/jetson-containers)の
Dockerイメージ(JetPackバージョンに対応したPyTorch/ultralyticsが最初から
入っている)を使う方法もある。このPoCスクリプト自体はROS2非依存なので、
コンテナ内でもそのまま動く。

## 実行

```bash
python3 person_count_poc.py
```

**カメラは自動検出されます**(`--camera`を指定しなければ、`/dev/webcam_checkout`の
固定名 → 見つからなければindex 0〜5を順に試す)。

ディスプレイの無い環境(SSH接続のみ等)で試す場合は、映像ウィンドウを出さずに
ターミナルへのカウント表示だけにできる:

```bash
python3 person_count_poc.py --no-display
```

その他のオプション(既定値は`face_count_poc`と同じ):

- `--model-path yolov8n.pt` : ultralytics YOLOのモデルファイル
- `--line-x-ratio 0.5` : カウント判定に使う縦ラインの位置
- `--direction left_to_right` : `left_to_right`/`right_to_left`/`both`
- `--min-person-height 400` : これより小さい(px)人物は無視する(近接フィルタ)
- `--min-confidence 0.35` : YOLOの検出信頼度の下限
- `--min-track-hits 3` : 誤検出対策(連続検出フレーム数)
- `--track-stale-frames 60` : 追跡情報を破棄するまでのフレーム数

## 見るべきポイント

- **処理速度(FPS)**: Jetson Nanoの実機で、実用に耐える速さで動くか
  (ウィンドウのカクつき具合、ターミナルに出るカウントのタイムラグ等で体感確認)
- **GPU/CPUどちらで動いているか**: `tegrastats`コマンドを別ターミナルで実行し、
  `GR3D_FREQ`(GPU使用率)が上がっているか確認する。0%のままならCPU推論に
  フォールバックしている(手順2のPyTorch導入をやり直す)
- 誤検出・見逃しの傾向、複数人が同時に映ったときの挙動は`face_count_poc`と同様

## 本番との対応

判定ロジック(YOLOトラッキング・近接フィルタ・追跡安定性・方向判定)は
`nokosahen_checkout/crossing_logic.py`・`standalone_checkout_cam/`と同じ考え方です。
Jetson Nanoを決済カメラとして実際に本番投入する場合は、この単体PoCではなく
`standalone_checkout_cam/`(HTTP POSTで`/api/count`へ送信する版)を使ってください。
このPoCはあくまで「Jetson Nanoで認識自体がどれくらいの精度・速度で動くか」を
確認するためのものです。
