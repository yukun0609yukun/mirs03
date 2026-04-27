# MIRS03 ファイル詳細構成

すべてのファイルの役割、内容、カスタマイズ方法を説明します。

## 📋 目次

- [Arduino スケッチ](#arduino-スケッチ)
- [Raspberry Pi ソースコード](#raspberry-pi-ソースコード)
- [ヘッダファイル](#ヘッダファイル)
- [変更が必要な箇所](#変更が必要な箇所)

---

## Arduino スケッチ

### define.h

**役割**: 全 Arduino スケッチで使用する定数・型定義

**重要な定数**:
```c
#define T_CTRL 10           // 制御周期 [ms] - 最も重要
#define R_TIRE 3.999        // タイヤ半径 [cm] - 実測値に調整必須
#define D_TIRE 38.83        // タイヤ間隔 [cm]
#define ENC_RANGE 4096      // エンコーダ分解能 (2048*2)
#define GEAR_RATIO 1.00     // ギア比
#define L_R_RATIO 0.8569    // 左右タイヤ速度比 - 走行の直進性に影響
#define V_RATIO 0.5         // バッテリー分圧比
```

**ピン配置**:
```c
#define PIN_ENC_A_L 2       // 左エンコーダ A相
#define PIN_ENC_B_L 4       // 左エンコーダ B相
#define PIN_ENC_A_R 3       // 右エンコーダ A相
#define PIN_ENC_B_R 7       // 右エンコーダ B相
#define PIN_DIR_L 12        // 左モーター方向
#define PIN_PWM_L 11        // 左モーター PWM
#define PIN_DIR_R 8         // 右モーター方向
#define PIN_PWM_R 9         // 右モーター PWM
#define PIN_LS A0           // ライトセンサ
#define PIN_LIGHT 10        // ライト LED
#define PIN_LED 13          // LED インジケータ
#define PIN_BATT 19         // バッテリー電圧
```

**型定義**:
```c
run_state_t      // 走行モード (STP, STR, ROT, CIR, LTC, VEL, VLT)
serial_data_t    // シリアル通信 7バイト
middle_data_t    // エンコード中間データ
command_data_t   // コマンドデータ 3×16bit
```

**カスタマイズ方法**:
1. タイヤ実測: 直進 100cm → 実測値で `R_TIRE` 調整
2. 左右速度差: 走行時に左右の走行距離を計測 → `L_R_RATIO` 調整
3. ピン変更: Arduino ボードに合わせて PIN_* を修正

---

### mg5_arduino_ver210.ino

**役割**: メインプログラム (setup, loop)

**内容**:
```cpp
void setup() {
  io_open();       // I/O 初期化
  encoder_open();  // エンコーダ初期化
  motor_open();    // モーター初期化
  raspi_open();    // シリアル通信初期化
}

void loop() {
  // パターン 1: 直進
  run_ctrl_set(STR, 100, 100);
  run_ctrl_execute();
  vel_ctrl_execute();
  delay(T_CTRL);
  
  // またはスレーブモード
  // slave();  // Raspberry Pi のコマンド待ち
}
```

**カスタマイズ方法**:
- `loop()` 内の走行パターンを変更
- またはコメント解除で `slave()` を呼び出し (Raspberry Pi モード)

---

### encoder.ino

**役割**: エンコーダ読み込み (割り込ント処理)

**主要関数**:
- `encoder_open()`: 初期化
- `encoder_get()`: 累積カウント値取得
- `encoder_reset()`: リセット

**内部処理**:
- 割り込みハンドラ: `enc_change_l()`, `enc_change_r()`
- 回転方向判定: A, B 相の組み合わせで正転/逆転判定
- 処理時間: ~100 μs (割り込み内)

**変更が必要な場合**:
- エンコーダの分解能: `ENC_RANGE` を `define.h` で調整
- 回転方向補正: `encoder_get()` 内の `*cnt_l *= -1;` を調整

---

### distance.ino

**役割**: エンコーダ値 → 走行距離 [cm] に変換

**計算式**:
```
dist_l = enc_l × R_TIRE × 2π / ENC_RANGE / GEAR_RATIO
dist_r = enc_r × R_TIRE × 2π / ENC_RANGE / GEAR_RATIO × L_R_RATIO
```

**パラメータ依存性**:
- `R_TIRE`: ±1 mm で大きく影響 → 精密測定必須
- `L_R_RATIO`: 左右の走行距離差を補正

---

### motor.ino

**役割**: PWM 値 → モーター回転 (方向制御)

**PWM 出力**:
- `-255`: 最大後進
- `0`: 停止
- `+255`: 最大前進

**方向ピン** (DIR):
- `HIGH`: 前進
- `LOW`: 後進

---

### vel_ctrl.ino

**役割**: 速度 PID 制御

**制御フロー**:
```
目標速度 (vel_ref)
  ↓
現在速度計測 (速度 = Δ距離 / Δ時間)
  ↓
誤差計算 (err = vel_ref - vel_curr)
  ↓
PID 計算 (PWM = Kp×err + Ki×∫err + Kd×derr/dt)
  ↓
PWM 出力 → motor_set()
```

**PID ゲイン調整**:
```cpp
const double Kp = 0.4;  // 比例: 増加 → 応答敏感
const double Ki = 0.1;  // 積分: 定常偏差補正
const double Kd = 0.0;  // 微分: 振動抑制
```

**ゲイン調整手順**:
1. `Kd = Ki = 0`, `Kp` を徐々に増加
2. `Kp` が決まったら `Ki` で定常偏差を改善
3. `Kd` で振動を抑制

---

### run_ctrl.ino

**役割**: 走行モード制御 (STR, ROT, CIR, LTC など)

**走行モード別処理**:

| モード | 計算内容 | 出力 |
|--------|--------|------|
| STR | 直進距離 = (enc_l + enc_r)/2 | 左右同速度 |
| ROT | 回転角度 = (enc_l - enc_r)/D_TIRE | 左右逆向き |
| CIR | 円弧角度と半径から左右速度比 | 内輪速 < 外輪速 |
| LTC | light_mod() で左右速度修正 | ライントレース |

**減速処理**:
- 目標距離の手前から徐々に速度低下
- `dist_vel_down = 0.0` なら減速なし (即停止)

---

### light.ino

**役割**: ライントレース (PD 制御)

**センサ値 → 速度修正**:
```
light_val = analogRead(PIN_LS);  // 0-1023
err = light_val - gray           // 偏差計算
vel_mod = Kp×err + Kd×(err-err_prev)  // PD 出力
```

**ゲイン調整**:
```cpp
static float Kp = 0.2;   // 感度 (増加 → 素早い追従)
static float Kd = 1.0;   // 安定性 (増加 → 振動抑制)
static int gray = 460;   // センサ中間値 (実測で調整)
```

**gray 値の測定**:
```cpp
// test_light() を実行
void test_light() {
  while (1) {
    int light = analogRead(PIN_LS);
    Serial.println(light);
    delay(500);
  }
}
// 白地: 150-200, 黒線: 700-800 → gray = (白+黒)/2
```

---

### raspi.ino

**役割**: Raspberry Pi とのシリアル通信 (115200 bps)

**プロトコル**:
```
送受信パケット: 7 バイト固定
  [0x80] [Data 0] [Data 1] [Data 2] [Data 3] [Data 4] [Data 5]
  Start  └─────── 24-bit × 3 (bit-packed) ──────────┘
```

**エンコード/デコード**:
- `raspi_encode()`: 3×16bit → 7 バイト
- `raspi_decode()`: 7 バイト → 3×16bit

**スレーブモード実装** (`slave()`):
- Raspberry Pi からのコマンド受信待機
- コマンドに応じて走行実行
- 状態を Raspberry Pi に返信

---

### io.ino

**役割**: LED、バッテリー監視

**LED**: PIN_LED (13) で点灯/消灯

**バッテリー**: PIN_BATT (A19) で電圧監視
```cpp
V = analogRead(PIN_BATT) * 5.0 / 1024.0 / V_RATIO
// 分圧回路: 12V → 5V 以下に変換
```

---

### slave.ino

**役割**: Raspberry Pi スレーブモード実装

**コマンド処理**:
```cpp
case 1: run_ctrl_set(STP, 0, 0);              // 停止
case 2: run_ctrl_set(STR, cmd.val[1], ...);  // 直進
case 3: run_ctrl_set(ROT, cmd.val[1], ...);  // 回転
// ...
case 10: run_ctrl_get(&state, ...); raspi_send();  // 状態取得
case 11: distance_get(&dist_l, &dist_r); raspi_send();  // 距離取得
case 12: ...; raspi_send();                  // バッテリー取得
```

---

### test.ino

**役割**: 各モジュールのテスト関数

**使用方法**: `setup()` 内で呼び出し
```cpp
void setup() {
  io_open();
  encoder_open();
  motor_open();
  raspi_open();
  
  // test_distance();  // エンコーダテスト
  // test_motor(100, 100);  // モーター回転テスト
}
```

---

## Raspberry Pi ソースコード

### Makefile

**役割**: Raspberry Pi 側のビルド設定

**コンパイラ設定**:
```makefile
CC = gcc
CFLAGS = -Wall -O2 -I.
LIBS = -lwiringPi -lpthread
```

**ビルド対象**:
- `test_request`: コマンド実行テスト
- `test_uss`: 超音波センサテスト
- `test_position`: 位置計算テスト
- `pilot_vlt_mirs`: メインプログラム

---

### arduino.c / arduino.h

**役割**: Arduino とのシリアル通信インターフェース

**関数**:
- `arduino_open()`: 接続開く (/dev/ttyACM0, 115200 bps)
- `arduino_receive()`: コマンド受信・デコード
- `arduino_send()`: 状態送信・エンコード

**デバイスパス**: `/dev/ttyACM0` (Arduino Mega)
- 複数接続時: `/dev/ttyACM1`, `/dev/ttyACM2` に変更

---

### request.c / request.h

**役割**: Arduino コマンド実行 (高級インターフェース)

**関数**:
- `request_set_runmode()`: 走行モードコマンド送信
- `request_get_runmode()`: 走行状態取得
- `request_get_dist()`: 走行距離取得
- `request_get_batt()`: バッテリー電圧取得

---

### uss.c / uss.h

**役割**: 超音波センサ読み込み (I2C)

**センサアドレス**:
```c
0x70: 右前方 (RF)
0x71: 右後方 (RB)
0x72: 後方右 (BR)
0x73: 後方左 (BL)
```

**関数**:
- `uss_get_rf()`, `uss_get_rb()` 等: 距離 [cm] 取得

**変更が必要な場合**:
- アドレス変更: `wiringPiI2CSetup(0x7X)` で変更
- 測定周期: `#define t_uss 50` で変更 [ms]

---

### direction.c / direction.h

**役割**: 超音波で方向補正 (壁に対して垂直)

**処理フロー**:
1. 左右センサから距離差を計算
2. 角度補正: 回転命令を送信
3. 距離補正: 直進命令を送信
4. 繰り返し (最大 15 回)

**パラメータ**:
```c
const int count_max = 15;    // 試行回数
const int dist_uss = 17;     // センサ間距離 [cm]
const int param1 = 25;       // 角度優先の差分閾値 [cm]
const int param3 = 1;        // 完了判定の差分 [cm]
```

---

### position.c / position.h

**役割**: 位置・向きの推定計算 (走行距離から)

**座標系**: フィールド右下を原点、上が x 軸正、左が y 軸正

**関数**:
- `position_set_coord()`: 初期位置設定
- `position_straight()`: 走行距離を位置に反映
- `position_rotate()`: 回転角度を向きに反映
- `position_get_area()`: フィールド内のエリア判定 (A/B/C/D)

---

### person.c / person.h

**役割**: Python カメラプログラムの出力を読み込み (人検出)

**入力ファイル**:
- `../person_check_py/person_file.txt`: 人検出フラグ (0/1)
- `../person_check_py/person_mag.txt`: 検出領域面積

**グローバル変数**: `float area` (検出領域面積)

**変更が必要な場合**:
- ファイルパス変更: `char person_file[256]` で変更

---

### pilot_vlt_mirs.c

**役割**: メインプログラム (人検出 + ライントレース)

**処理フロー**:
1. 入力: 走行速度 [cm/s], 走行距離 [cm]
2. スムーズスタート (速度段階的上昇)
3. 人検出ループ:
   - 人がいる → 速度低下 (領域面積で制御)
   - 人がいない → 定速走行
4. 走行距離到達 → 停止

---

### pilot_vlt_org.c

**役割**: 旧版メインプログラム (人検出 → 停止)

**違い**: `pilot_vlt_mirs.c` は速度調整、`pilot_vlt_org.c` は停止

---

### test_*.c

**役割**: 各モジュールのテストプログラム

**テストファイル**:
- `test_request.c`: コマンド対話的テスト
- `test_uss.c`: 超音波センサ読み込みテスト
- `test_position.c`: 位置計算テスト
- `test_person.c`: 人検出読み込みテスト
- `test_direction.c`: 方向補正テスト
- `test_io.c`: GPIO (スイッチ) テスト

---

## ヘッダファイル

### arduino.h / request.h / uss.h など

**役割**: 関数プロトタイプ・型定義の公開

**記載内容**:
- 関数プロトタイプ
- 外部変数 (`extern`)
- 型定義
- マクロ定義

---

## 変更が必要な箇所

### 1. ハードウェア接続変更時

**Arduino ピン変更**:
```c
// define.h
#define PIN_ENC_A_L 2  // → 別ピンに変更
```

**Raspberry Pi GPIO 変更**:
```c
// io.c
static const int pin_sw_f = 4;  // → 別ピンに変更
```

**超音波センサアドレス変更**:
```c
// uss.c
fd_rf = wiringPiI2CSetup(0x70);  // → 別アドレスに変更
```

---

### 2. パラメータ調整時

**走行パラメータ**:
```c
// define.h
#define R_TIRE 3.999        // タイヤ半径 [cm] - 実測値で調整
#define L_R_RATIO 0.8569    // 左右速度比 - 走行時に計測
#define GEAR_RATIO 1.00     // ギア比 - 仕様値確認
```

**制御ゲイン**:
```cpp
// vel_ctrl.ino
const double Kp = 0.4;  // PID 比例ゲイン - 調整必須
const double Ki = 0.1;  // PID 積分ゲイン
const double Kd = 0.0;  // PID 微分ゲイン
```

**ライントレース**:
```cpp
// light.ino
static float Kp = 0.2;   // 感度 - 実験で調整
static int gray = 460;   // センサ基準値 - 白黒の中間値
```

---

### 3. 走行パターン変更時

**Arduino ローカル走行**:
```cpp
// mg5_arduino_ver210.ino の loop() を編集
void loop() {
  run_ctrl_set(STR, 100, 100);  // 走行パターン変更
  // ...
}
```

**Raspberry Pi リモート走行**:
```c
// pilot_vlt_mirs.c で走行ロジック編集
while (1) {
  if (person_check() == 1) {
    // 人検出時の処理変更
    request_set_runmode(VLT, vel_l * 0.5, vel_r);  // 速度調整
  }
}
```

---

**最終更新**: 2026-04-27
