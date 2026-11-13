# のこさへん (MIRS2603)

食堂のフードロス削減を目指す看板娘ロボット「のこさへん」のソースコードです(ROS 2 Jazzy / Python)。
足回り(ESP32 + micro-ROS)、LiDAR・カメラによる人物検出、地図と自己位置推定による移動、音声案内、残数の管理、頭部 iPad の表示とアンケート、Web の管理画面を、1つのスタックにまとめています。

## このリポジトリについて

**コードだけ**を公開しています。次のものは含めていません。

| 含めていないもの | 影響 |
|---|---|
| 設計文書、変更履歴、作業日報 | 説明は、このファイルとコード内のコメントだけです |
| 機体のデータ(3D モデル・URDF・Gazebo のワールド・寸法) | `nokosahen_gazebo` のシミュレーションは、そのままでは動きません |
| ファームウェアの機体の値(`config.h` など。ピン配置・PID・車輪寸法・安全の限界値) | `firmware/` は、そのままではビルドできません |
| 設定・調整値(`nokosahen.yaml`、Nav2・SLAM のパラメータ、ビヘイビアツリーなど) | 雛形(`*.example.*`)だけがあります。実機の挙動は、再現できません |
| 学習済みモデル、画像、放送の文面、音声の設定 | — |
| アカウント・パスワード | ロボット上のローカルな設定に置きます |

- 現場のアドレスやアカウント名は、公開用に置き換えています(例: IP は `192.0.2.x` などの例示用アドレス、管理画面のアカウント名は `ADMIN_ACCOUNT`)。
- ここにあるコードは、読むための公開です。このリポジトリだけでロボットが動くことは、想定していません。

## ハードウェア構成(文化祭の構成)

| 機器 | 役割 |
|---|---|
| Jetson 1台目 | ROS 2 の本体(挙動・位置推定・管理画面・iPad 表示) |
| Jetson 2台目 | 音声(VOICEVOX による合成と再生、`voice_bridge`)。1台目と有線で接続 |
| ESP32 | 足回りのモーター制御(micro-ROS) |
| RPLIDAR | 周囲の距離計測(人物・障害物・位置推定) |
| Web カメラ | 人物検出、ほか |
| iPad | 頭部ディスプレイ(顔・アンケート・お客さん向け表示) |
| Raspberry Pi | 腹部ディスプレイなどの表示(予備・旧方式を含む) |

## リポジトリの構成

| 場所 | 内容 |
|---|---|
| `ros2_ws/src/nokosahen_core` | 本体のロジック。挙動(`behavior_director`)、人物検出(`person_target_detector`)、混雑判定、位置保持、機器の監視(`device_watchdog`)、管理画面(`full_dashboard_node`)、権限(`roles`)、iPad 表示(`display_*`)、アンケート、出店紹介の放送(`pr_intro`・`announce_node`)など |
| `ros2_ws/src/nokosahen_core/test` | 単体テスト(pytest)。設定値を含む付属データは公開していないため、一部は通りません |
| `ros2_ws/src/nokosahen_bringup` | launch ファイル(Nav2・SLAM のパラメータ、`nokosahen.yaml` などの設定値は公開していません) |
| `ros2_ws/src/nokosahen_voice` | 発話文を音声サービスへ橋渡しするノード |
| `ros2_ws/src/nokosahen_checkout` | 決済横カメラ用ユニット(Raspberry Pi 向け) |
| `ros2_ws/src/nokosahen_gazebo` | Gazebo のシミュレーション(launch のみ。モデル・ワールドは非公開) |
| `firmware/` | ESP32 のファームウェア(PlatformIO)。機体の値のヘッダは非公開 |
| `voice_bridge/` | 音声合成・再生サービス(VOICEVOX + 再生) |
| `scripts/festival/` | 文化祭用の起動スクリプトと見張り(下記) |
| `scripts/autostart/` | 電源を入れたときの自動起動(systemd) |
| `scripts/jetson2_setup/` | 2台目の音声環境のセットアップ |
| `scripts/pc/` | PC から1台目へ配置・動作確認するスクリプト |
| `scripts/raspi_display/` | Raspberry Pi の表示(アンケート・OCR テスト画面) |
| `config/` | 設定の雛形(`pr_intro.example.json`) |
| `docker/`, `docker-compose.yml` | Docker 構成 |
| `poc_tests/`, `standalone_checkout_cam/`, `tools/` | 概念実証・単体で動く補助ツール |

## ビルド

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=77
```

テストは、パッケージごとに、ファイルを指定して `pytest` で実行します(ROS 2 を使わず、PC 上で動くものが大半です)。

```bash
cd ros2_ws/src/nokosahen_core
python -m pytest test/test_roles.py -q
```

## 起動(文化祭の運用)

1台目で、文化祭用の設定で起動します。

```bash
bash scripts/festival/festival_start.sh --no-dds-wired --light-localization
```

- 起動後、約2分で管理画面が使えます。
  - 管理画面(PC): `http://<1台目のIP>:8090/`
  - スマホ操作: `http://<1台目のIP>:8095/`
  - iPad の表示: `http://<1台目のIP>:8090/display`
- 管理画面・iPad にはログインが必要です。アカウントとパスワードは、このリポジトリには入っていません(下記)。
- 電源を入れたときに自動で起動する設定は、`scripts/festival/festival_mode.sh enable` で入れます(`scripts/autostart/`)。
- 2台目の音声は、`scripts/jetson2_setup/start_voice_stack.sh` で起動します。固まったときに入れ直す見張りは `scripts/festival/keep_voice_alive.sh` です。
- 詳しい説明と引数は、`scripts/festival/README.md` にあります。

## 主な機能

- **挙動**: 人の方向を向く、人の近くで止まる、位置を保つ、緊急停止、アンケート回答中は走行を停止。
- **人物検出**: LiDAR とカメラを組み合わせて、一番近い人を選びます。
- **移動**: 地図の読み込みと初期位置の指定、手動運転(PC・スマホ)、動ける範囲の制限。
- **機器の監視**: LiDAR・カメラ・ESP32 などの状態を見て、落ちたら再起動します。
- **音声**: 出店紹介を一定の間隔で放送。音量やON/OFFは管理画面から。
- **iPad 表示**: お客さん向け、アンケート、残数入力、OCR テスト、顔。顔とアンケートは、人が近づいたときに自動で切り替えられます。
- **アンケート**: 回答は CSV でダウンロードできます。
- **権限**: 管理者・操作者(operator)・閲覧専用(guest)などのロールごとに、操作を制限しています(`roles.py`)。

## 設定と秘密情報

- 設定の値(調整値・機体の寸法など)は、このリポジトリには含めていません。雛形は `*.example.*` です(例: `config/pr_intro.example.json`)。
- アカウントとパスワードは、このリポジトリには含めていません。雛形は `ros2_ws/src/nokosahen_bringup/config/secrets.example.yaml` と `accounts.example.yaml` です。実際の値は、ロボット上のローカルな設定ファイルに置きます(`.gitignore` で除外しています)。
- 出店紹介の放送の文面は、このリポジトリには含めていません。

## ESP32 のファームウェア

```bash
cd firmware/esp32_microros_drive
pio run -t upload
```

PID ゲイン・ピン配置・車輪寸法などの機体の値は、公開していません(`include/config.h`)。

## 主なトピック

| トピック | 内容 |
|---|---|
| `/cmd_vel` | 速度指令(ESP32 へ) |
| `/odom` | オドメトリ |
| `/scan` | LiDAR のスキャン |
| `/nokosahen/crowd_state` | 混雑・人物ターゲットの状態 |
| `/nokosahen/behavior_command` | 挙動の指令 |
| `/nokosahen/announce_text` | 読み上げ文 |
| `/nokosahen/display_mode` | iPad の表示モード(`customer` / `survey` / `inventory` / `ocrtest` / `face`) |
| `/nokosahen/soft_estop` | ソフトウェアの緊急停止 |
| `/nokosahen/device_watchdog/status` | 機器の監視の状態 |
