# MIRS03 - Mobile Robot Control System

Raspberry Pi と Arduino を組み合わせたロボット制御システム。自律走行機能、ライントレース、超音波センサ統合など、複数の走行モードをサポート。

## 📋 目次

- [クイックスタート](#-クイックスタート)
- [プロジェクト構成](#-プロジェクト構成)
- [走行モード](#-走行モード)
- [システム構成](#-システム構成)
- [通信プロトコル](#-通信プロトコル)
- [ドキュメント](#-ドキュメント)

## 🚀 クイックスタート

### 必要な環境
- Arduino IDE 2.0+
- Raspberry Pi OS (推奨: Bullseye以降)
- GCC / Make (Raspberry Pi用)
- USB シリアル接続

### セットアップ（5分で完了）

```bash
# 1. リポジトリクローン
git clone https://github.com/yukun0609yukun/mirs03.git
cd mirs03

# 2. Arduino スケッチをアップロード
#    arduino/ フォルダの mg5_arduino_ver210.ino を Arduino IDE で開いて、
#    ボードと COM ポートを設定後、アップロード

# 3. Raspberry Pi 側をコンパイル
cd raspi
make clean
make all

# 4. 実行
./bin/test_request  # テストモード
# または
./bin/pilot_vlt_mirs  # メインプログラム
```

詳細は [SETUP.md](docs/SETUP.md) を参照してください。

## 📁 プロジェクト構成

```
mirs03/
├── arduino/              # Arduino スケッチ (C++)
│   ├── mg5_arduino_ver210.ino    # メインプログラム
│   ├── define.h          # 定数・型定義
│   ├── encoder.ino       # エンコーダ読み込み
│   ├── distance.ino      # 走行距離計算
│   ├── motor.ino         # モーター制御
│   ├── velocity.ino      # 速度制御 (PID)
│   ├── run_ctrl.ino      # 走行モード制御
│   ├── light.ino         # ライントレース
│   ├── raspi.ino         # シリアル通信
│   ├── io.ino            # I/O 制御
│   ├── slave.ino         # スレーブモード
│   └── test.ino          # テスト関数
│
├── raspi/                # Raspberry Pi プログラム (C)
│   ├── Makefile          # ビルド設定
│   ├── arduino.c/h       # Arduino 通信インターフェース
│   ├── request.c/h       # コマンド実行
│   ├── direction.c/h     # 方向補正
│   ├── position.c/h      # 位置計算
│   ├── uss.c/h           # 超音波センサ
│   ├── person.c/h        # 人検出 (カメラ連携)
│   ├── io.c/h            # GPIO 制御
│   ├── pilot*.c          # 実行プログラム
│   └── test_*.c          # テストプログラム
│
├── docs/                 # ドキュメント
│   ├── SETUP.md          # セットアップガイド
│   ├── API_REFERENCE.md  # API リファレンス
│   ├── CODE_STYLE.md     # コード規約
│   ├── FILE_STRUCTURE.md # ファイル構成詳細
│   ├── PROJECT_STRUCTURE.md  # プロジェクト構造
│   └── QUICK_REFERENCE.md    # クイックリファレンス
│
└── scripts/
    └── BUILD_GUIDE.md    # ビルド手順
```

詳細は [PROJECT_STRUCTURE.md](docs/PROJECT_STRUCTURE.md) を参照してください。

## 🎮 走行モード

| モード | 説明 | 用途 |
|--------|------|------|
| **STP** | 停止 | 緊急停止・初期状態 |
| **STR** | 直進 | 前後移動 |
| **ROT** | 回転 | その場回転 |
| **CIR** | 円弧走行 | 曲線走行 |
| **LTC** | ライントレース | 黒線追従 |
| **VEL** | 速度指定 | 左右速度独立制御 |
| **VLT** | 速度+ライントレース | ライントレース + 速度制御 |

### 走行モード実行例

```c
// Arduino 側の例
run_ctrl_set(STR, 100, 50);   // 直進: 速度 100cm/s, 距離 50cm
run_ctrl_execute();             // 制御実行

// Raspberry Pi 側の例
request_set_runmode(LTC, 25, 100);  // ライントレース: 速度 25cm/s, 距離 100cm
```

## 🔧 システム構成

### ハードウェア

```
[Raspberry Pi 4B]
    ↓ (USB シリアル)
[Arduino Mega]
    ├→ [モーター × 2] (PWM制御)
    ├→ [エンコーダ × 2] (割り込み読み込み)
    ├→ [ライトセンサ] (アナログ入力)
    └→ [各種GPIO] (LED, バッテリー監視など)

[超音波センサ × 4] (I2C接続)
    ↓ (I2C/GPIO)
[Raspberry Pi]
```

### ソフトウェア構成

```
【Raspberry Pi (C)】
  pilot_vlt_mirs.c (メインプログラム)
    ├→ arduino.c (通信)
    ├→ request.c (コマンド実行)
    ├→ direction.c (方向補正)
    ├→ person.c (人検出)
    └→ uss.c (超音波取得)

【Arduino (C++)】
  mg5_arduino_ver210.ino (メインスケッチ)
    ├→ encoder.ino (割り込み処理)
    ├→ distance.ino (距離計算)
    ├→ motor.ino (PWM出力)
    ├→ vel_ctrl.ino (PID制御)
    ├→ run_ctrl.ino (走行モード)
    └→ raspi.ino (通信処理)
```

## 💬 通信プロトコル

### Serial 通信仕様

- **ボーレート**: 115200 bps
- **フォーマット**: 7バイト固定長 + 開始フラグ (0x80)
- **周期**: ~10ms (Arduino 側制御周期)

### コマンドパケット構造

```
┌─────┬─────────────────────────┐
│0x80 │ 6 bytes (24-bit × 3)    │
├─────┼─────────────────────────┤
│Start│ Command[0] (走行モード) │
│Flag │ Command[1] (速度/半径) │
│     │ Command[2] (距離/角度) │
└─────┴─────────────────────────┘

走行モード:
  1 = STP (停止)
  2 = STR (直進)
  3 = ROT (回転)
  4 = CIR (円弧)
  5 = LTC (ライントレース)
  6 = VEL (速度指定)
  7 = VLT (速度+ライントレース)
```

## 📚 ドキュメント

| ドキュメント | 対象者 | 内容 |
|-------------|--------|------|
| [SETUP.md](docs/SETUP.md) | 全員 | 環境構築・初期設定 |
| [API_REFERENCE.md](docs/API_REFERENCE.md) | 開発者 | API 詳細説明・使用例 |
| [CODE_STYLE.md](docs/CODE_STYLE.md) | 開発者 | コード規約・設計方針 |
| [FILE_STRUCTURE.md](docs/FILE_STRUCTURE.md) | 開発者 | ファイル詳細説明 |
| [PROJECT_STRUCTURE.md](docs/PROJECT_STRUCTURE.md) | 全員 | プロジェクト全体構成 |
| [QUICK_REFERENCE.md](docs/QUICK_REFERENCE.md) | 全員 | コマンド・パラメータ |
| [BUILD_GUIDE.md](scripts/BUILD_GUIDE.md) | 開発者 | ビルド手順 |

## ⚙️ 主要パラメータ

`arduino/define.h` の定数定義（変更可能）:

```c
#define T_CTRL 10           // 制御周期 [ms]
#define R_TIRE 3.999        // タイヤ半径 [cm]
#define D_TIRE 38.83        // タイヤ間隔 [cm]
#define ENC_RANGE 4096      // エンコーダ分解能
#define GEAR_RATIO 1.00     // ギア比
#define L_R_RATIO 0.8569    // 左右タイヤ速度比
#define V_RATIO 0.5         // バッテリ分圧比
```

詳細な調整方法は [QUICK_REFERENCE.md](docs/QUICK_REFERENCE.md#-パラメータ調整) を参照。

## 🧪 テストプログラム

Arduino 側:
```cpp
test_encoder()       // エンコーダ値表示
test_distance()      // 走行距離表示
test_motor(pwm_l, pwm_r)   // モーター動作確認
test_light()         // ライトセンサ値表示
test_vel_ctrl(vel_l, vel_r)    // 速度制御テスト
test_run_ctrl(state, speed, dist)  // 走行モードテスト
```

Raspberry Pi 側:
```bash
./bin/test_request       # コマンド実行テスト
./bin/test_uss           # 超音波センサテスト
./bin/test_position      # 位置計算テスト
./bin/test_person        # 人検出テスト
```

## 🐛 トラブルシューティング

| 症状 | 原因 | 対応 |
|------|------|------|
| Arduino が接続されない | USB ドライバ未インストール | [SETUP.md](docs/SETUP.md#トラブルシューティング) 参照 |
| シリアル通信エラー | ボーレート不一致 | `raspi.ino` と `arduino.c` で同じ速度を設定 |
| 走行がうまくいかない | パラメータ未調整 | [QUICK_REFERENCE.md](docs/QUICK_REFERENCE.md) でパラメータ微調整 |
| エンコーダ値がおかしい | 接続不良 | コネクタ再挿入、接点クリーニング |

詳細は [SETUP.md - トラブルシューティング](docs/SETUP.md#トラブルシューティング) を参照してください。

## 📞 サポート

- 技術的な質問は GitHub Issues で報告してください
- プルリクエストも歓迎です
- 改善提案は Discussion で議論してください

## 📄 ライセンス

このプロジェクトはオープンソースです。自由に使用・改変・配布が可能です。

## 🔗 関連リンク

- [Arduino 公式ドキュメント](https://docs.arduino.cc/)
- [Raspberry Pi ドキュメント](https://www.raspberrypi.com/documentation/)
- [WiringPi (GPIO ライブラリ)](http://wiringpi.com/)

---

**最終更新**: 2026-04-27  
**バージョン**: 2.1