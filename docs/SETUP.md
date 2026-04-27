# MIRS03 セットアップガイド

初期セットアップから実運用まで、段階的に説明します。

## 📋 目次

- [必要な機材](#必要な機材)
- [環境構築](#環境構築)
- [Arduino セットアップ](#arduino-セットアップ)
- [Raspberry Pi セットアップ](#raspberry-pi-セットアップ)
- [ハードウェア接続](#ハードウェア接続)
- [動作確認](#動作確認)
- [トラブルシューティング](#トラブルシューティング)

## 必要な機材

### パソコン側
- Windows / macOS / Linux（Arduino IDE インストール用）
- USB Type-A ケーブル × 1本（Arduino接続用）

### ロボット側
- **Raspberry Pi 4B** (4GB 以上推奨)
- **Arduino Mega 2560**
- **モータードライバ** (PWM制御対応)
- **DC モーター** × 2
- **エンコーダ** × 2 (回転数カウント用)
- **ライトセンサ** (アナログ入力)
- **超音波センサ** × 4 (I2C接続)
- **USB ハブ** (Raspberry Pi + Arduino 両接続用)
- **電源** (12V 推奨、調整可能)

## 環境構築

### 1️⃣ Arduino IDE のインストール

#### Windows / macOS

1. [Arduino 公式サイト](https://www.arduino.cc/en/software) にアクセス
2. Arduino IDE 2.0+ をダウンロード
3. インストーラを実行してインストール
4. インストール完了後、Arduino IDE を起動

#### Linux (Ubuntu/Debian)

```bash
sudo apt update
sudo apt install arduino arduino-mk
```

### 2️⃣ ドライバインストール

#### Windows

1. Arduino を USB で接続
2. デバイスマネージャーで「不明なデバイス」を確認
3. [CH340 ドライバ](https://www.wch.cn/download/ch341ser_exe.html) をダウンロード・インストール
4. デバイスマネージャーでポート番号を確認（例: COM3）

#### macOS

```bash
# Homebrew でドライバをインストール
brew tap macports/aqua
brew install ch340g-ch34x-macos-driver
```

#### Linux

通常、自動認識されます。���認:

```bash
ls /dev/ttyUSB*  # または /dev/ttyACM*
```

### 3️⃣ Raspberry Pi OS インストール

1. [Raspberry Pi Imager](https://www.raspberrypi.com/software/) をダウンロード
2. microSD カード (32GB 以上推奨) を準備
3. Imager で Raspberry Pi OS (Bullseye以降) を書き込み
4. SSH / VNC を有効化（設定画面で）
5. 起動・初期設定を完了

## Arduino セットアップ

### ステップ 1: スケッチファイルの準備

```bash
# GitHub からクローン
git clone https://github.com/yukun0609yukun/mirs03.git
cd mirs03/arduino
```

### ステップ 2: Arduino IDE で開く

1. Arduino IDE を起動
2. `ファイル` → `開く` → `mg5_arduino_ver210.ino` を選択
3. 関連ファイル (define.h, *.ino) が自動で開かれます

### ステップ 3: ボード設定

1. `ツール` → `ボード` → `Arduino Mega 2560` を選択
2. `ツール` → `シリアルポート` → 接続したポート（例: COM3）を選択
3. `ツール` → `シリアル通信速度` → `115200` を選択

### ステップ 4: アップロード

```
検証ボタン (✓) をクリック → 成功確認後
アップロードボタン (→) をクリック
```

完了メッセージ: `"アップロード完了"`

### ステップ 5: 動作確認 (Arduino 側)

```
ツール → シリアルモニタ
```

シリアルモニタで以下の出力が見えれば成功:

```
setup() 実行
encoder initialized
motor initialized
```

## Raspberry Pi セットアップ

### ステップ 1: リポジトリクローン

```bash
ssh pi@raspberrypi.local  # Raspberry Pi にログイン
# パスワード入力

cd ~
git clone https://github.com/yukun0609yukun/mirs03.git
cd mirs03/raspi
```

### ステップ 2: 依存ライブラリのインストール

```bash
# システムパッケージ更新
sudo apt update
sudo apt upgrade

# 開発ツール
sudo apt install build-essential git wiringpi

# I2C ツール (超音波センサ用)
sudo apt install i2c-tools python3-smbus
```

### ステップ 3: I2C 有効化

```bash
sudo raspi-config
# → Interfacing Options
# → I2C
# → Yes を選択して有効化
# → 再起動
```

確認:

```bash
sudo i2cdetect -y 1
# 超音波センサのアドレス (0x70, 0x71, 0x72, 0x73) が表示されれば OK
```

### ステップ 4: コンパイル

```bash
cd ~/mirs03/raspi
make clean
make all
```

出力ファイル:

```
bin/test_request
bin/test_uss
bin/test_position
bin/pilot_vlt_mirs
```

## ハードウェア接続

### Arduino ピン配置 (define.h)

```c
// エンコーダ
PIN_ENC_A_L = 2   // 左エンコーダ A相
PIN_ENC_B_L = 4   // 左エンコーダ B相
PIN_ENC_A_R = 3   // 右エンコーダ A相
PIN_ENC_B_R = 7   // 右エンコーダ B相

// モーター
PIN_DIR_L = 12    // 左モーター方向
PIN_PWM_L = 11    // 左モーター PWM
PIN_DIR_R = 8     // 右モーター方向
PIN_PWM_R = 9     // 右モーター PWM

// センサ
PIN_LS = A0       // ライトセンサ (アナログ)
PIN_LIGHT = 10    // ライト LED

// その他
PIN_LED = 13      // LED インジケータ
PIN_BATT = 19     // バッテリー電圧 (アナログ)
```

### Raspberry Pi GPIO 配置 (io.c)

```c
// スイッチ入力
pin_sw_f = 4      // 前方スイッチ
pin_sw_l = 7      // 左スイッチ
pin_sw_r = 8      // 右スイッチ
```

### I2C アドレス (uss.c)

```c
0x70 = 右前方超音波センサ (RF)
0x71 = 右後方超音波センサ (RB)
0x72 = 右後方超音波センサ (BR)
0x73 = 左後方超音波センサ (BL)
```

## 動作確認

### Arduino 動作確認

```cpp
// Serial Monitor でテスト関数実行
// setup() 内で以下を呼び出し:
test_encoder();      // → エンコーダ値を表示
test_distance();     // → 走行距離を表示
test_motor(100, 100); // → モーターが回転
```

### Raspberry Pi 動作確認

```bash
# 1. Arduino 接続確認
ls /dev/ttyACM*   # デバイスが表示されれば OK

# 2. test_request 実行
cd ~/mirs03/raspi
./bin/test_request
# メニューが表示され、コマンド入力可能なら OK

# 3. 超音波センサ確認
./bin/test_uss
# センサ値が表示されれば OK

# 4. メインプログラム実行
sudo ./bin/pilot_vlt_mirs
# 速度・走行距離入力後、ロボットが動作
```

## トラブルシューティング

### 1. Arduino が COM ポートに表示されない

**症状**: Arduino IDE でシリアルポートが見つからない

**原因**:
- USB ドライバ未インストール
- USB ケーブル不良
- Arduino 本体の問題

**対応**:
```bash
# Windows: デバイスマネージャーを確認
# macOS: システムレポート → USB で確認
# Linux: ls /dev/ttyUSB* または ls /dev/ttyACM*

# ドライバ再インストール
# → CH340 ドライバをダウンロード・再インストール
# → PC を再起動
```

### 2. Arduino にアップロードできない

**症状**: "アップロード失敗" エラー

**原因**:
- ボード設定が違う (Mega 2560 以外になっている)
- ポート番号が違う
- ボーレートが違う

**対応**:
```
ツール → ボード → Arduino Mega 2560
ツール → シリアルポート → 正しいポートを選択
ツール → シリアル通信速度 → 115200

→ 再度アップロード
```

### 3. シリアル通信エラー

**症状**: "COM ポートを開けません" メッセージ

**原因**:
- Arduino IDE とシリアルモニタが同時に接続
- 他のプログラムがポートを占有

**対応**:
```bash
# シリアルモニタを閉じる
# Python / putty など他のシリアル接続を終了
# → 再度接続
```

### 4. Raspberry Pi が Arduino と通信できない

**症状**: "arduino_receive() が -1 を返す"

**原因**:
- ボーレート不一致 (Arduino: 115200, raspi側: 異なる値)
- USB ケーブル接続不良
- パーミッション不足

**対応**:
```bash
# 1. ボーレート確認
# arduino/raspi.ino で 115200
# raspi/arduino.c で 115200 に統一

# 2. USB デバイス確認
ls -la /dev/ttyACM*

# 3. 権限付与
sudo usermod -a -G dialout pi
sudo reboot
```

### 5. エンコーダ値がおかしい

**症状**: "エンコーダカウント値が極端に大きい/小さい"

**原因**:
- エンコーダ接続不良
- ノイズによる誤検出

**対応**:
```cpp
// define.h でエンコーダ分解能を確認
#define ENC_RANGE 4096  // 実際の値に合わせる

// エンコーダケーブルを再接続
// → ノイズ防止フェライトコアを追加
```

### 6. モーターが動かない

**症状**: "PWM 値を設定しても回転しない"

**原因**:
- モーター電源未接続
- PWM ゲイン設定エラー
- モータードライバ故障

**対応**:
```cpp
// 1. 電源確認 (12V)
// 2. test_motor() で直接テスト
test_motor(150, 150);  // 左右 PWM = 150

// 3. 速度制御 PID ゲイン調整
// vel_ctrl.ino
const double Kp = 0.4;  // 値を調整
const double Ki = 0.1;
const double Kd = 0.0;
```

### 7. ライントレースがうまくいかない

**症状**: "黒線を見失う / 走行がぎくしゃくしている"

**原因**:
- PD ゲイン未調整
- ライトセンサの基準値がずれている

**対応**:
```cpp
// light.ino でゲイン調整
static float Kp = 0.2;  // 比例ゲイン (大きいと反応が敏感)
static float Kd = 1.0;  // 微分ゲイン (振動を抑制)

// gray = ライトセンサの中間値
static int gray = 460;  // 0-1023 の範囲で調整

// test_light() でセンサ値を確認
test_light();  // Serial Monitor で値を表示
```

### 8. 超音波センサが値を返さない

**症状**: "uss_get_rf() が -1 を返す"

**原因**:
- I2C アドレス設定エラー
- センサ電源未接続
- I2C 通信エラー

**対応**:
```bash
# I2C センサ確認
sudo i2cdetect -y 1
# 0x70, 0x71, 0x72, 0x73 が表示されれば接続 OK

# uss.c でアドレス確認
fd_rf = wiringPiI2CSetup(0x70);  // 正しいアドレス

# テスト実行
./bin/test_uss
```

### 9. 位置計算がずれる

**症状**: "実際の走行距離と計算値が異なる"

**原因**:
- パラメータ未調整 (タイヤ半径、ギア比など)
- 左右エンコーダの回転比がずれている

**対応**:
```c
// define.h でパラメータ調整
#define R_TIRE 3.999      // タイヤ半径 [cm] - 実測値に調整
#define D_TIRE 38.83      // タイヤ間隔 [cm]
#define L_R_RATIO 0.8569  // 左右タイヤ速度比

// test_distance() で走行距離をチェック
test_distance();
// 実際に 100cm 走らせて、表示値が 100cm になるよう調整
```

### 10. バッテリー電圧が表示されない

**症状**: "io_get_batt() が 0 を返す"

**原因**:
- バッテリー測定回路が接続されていない
- アナログ入力設定エラー

**対応**:
```cpp
// io.ino でバッテリーピン確認
#define PIN_BATT 19    // アナログ入力 A5 (Arduino Mega)

// 分圧比を調整 (12V を 5V 以下に変換)
// define.h
#define V_RATIO 0.5    // 12V → 5V の比率
```

### 11. Raspberry Pi の起動が遅い

**症状**: "SSH 接続に時間がかかる"

**原因**:
- ファイルシステムエラー
- ディスク空き容量不足

**対応**:
```bash
# ディスク確認
df -h

# 不要なファイルを削除
sudo apt autoremove
sudo apt autoclean

# 再起動
sudo reboot
```

### 12. コンパイルエラー

**症状**: `make all` でエラー

**原因**:
- 依存ライブラリ未インストール
- ファイルパスエラー

**対応**:
```bash
# ライブラリ確認
sudo apt install wiringpi libwiringpi-dev

# Makefile パス確認
head -n 20 Makefile

# クリーン & リコンパイル
make clean
make all -j4  # マルチスレッドコンパイル
```

---

**問題が解決しない場合**:
1. GitHub Issues で報告してください
2. `dmesg | tail` でカーネルログを確認
3. `-v` オプションでビルバッグ情報を出力

**参考リンク**:
- [Arduino トラブル対応](https://support.arduino.cc/)
- [Raspberry Pi サポート](https://www.raspberrypi.com/support/)
- [WiringPi FAQ](http://wiringpi.com/)
