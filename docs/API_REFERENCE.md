# MIRS03 API リファレンス

Arduino と Raspberry Pi の全モジュール・関数を詳細に説明します。

## 📋 目次

- [Arduino API](#arduino-api)
- [Raspberry Pi API](#raspberry-pi-api)
- [データ構造](#データ構造)

---

## Arduino API

### 1. encoder モジュール (encoder.ino)

エンコーダからの回転数を読み込み、割り込み処理で即座に更新します。

#### encoder_open()

```cpp
void encoder_open();
```

**説明**: エンコーダ入力ピンを初期化し、割り込みハンドラをアタッチします。

**パラメータ**: なし

**戻り値**: なし

**使用例**:
```cpp
void setup() {
  encoder_open();  // setup() 内で必ず呼び出し
}
```

**内部処理**:
- PIN_ENC_A_L, PIN_ENC_B_L, PIN_ENC_A_R, PIN_ENC_B_R を INPUT に設定
- 内部プルアップを有効化
- attachInterrupt() で enc_change_l / enc_change_r をアタッチ

---

#### encoder_get()

```cpp
void encoder_get(long *cnt_l, long *cnt_r);
```

**説明**: 累積エンコーダ値を取得します。

**パラメータ**:
- `long *cnt_l`: 左エンコーダ値 (ポインタ)
- `long *cnt_r`: 右エンコーダ値 (ポインタ)

**戻り値**: なし

**使用例**:
```cpp
long enc_l, enc_r;
encoder_get(&enc_l, &enc_r);
Serial.print("Left: ");
Serial.print(enc_l);
Serial.print(", Right: ");
Serial.println(enc_r);
```

**注意**: 回転方向の補正が含まれます (`*cnt_l *= -1`)

---

#### encoder_reset()

```cpp
void encoder_reset();
```

**説明**: エンコーダカウンタをリセット (0 に初期化)。

**パラメータ**: なし

**戻り値**: なし

**使用例**:
```cpp
void loop() {
  encoder_reset();  // 新しい走行開始前に
  // ... 走行処理 ...
}
```

**用途**: 走行開始時、あるいは距離測定をリセットしたい時

---

### 2. distance モジュール (distance.ino)

#### distance_get()

```cpp
void distance_get(double *dist_l, double *dist_r);
```

**説明**: エンコーダ値から走行距離 [cm] を計算して返します。

**パラメータ**:
- `double *dist_l`: 左走行距離 (ポインタ)
- `double *dist_r`: 右走行距離 (ポインタ)

**戻り値**: なし

**計算式**:
```
dist_l = enc_l * R_TIRE * 2π / ENC_RANGE / GEAR_RATIO
dist_r = enc_r * R_TIRE * 2π / ENC_RANGE / GEAR_RATIO * L_R_RATIO
```

**使用例**:
```cpp
double dist_l, dist_r;
distance_get(&dist_l, &dist_r);
double avg_dist = (dist_l + dist_r) / 2.0;  // 平均値
Serial.print("Distance: ");
Serial.println(avg_dist);
```

**注意**: 左右タイヤの速度比 (L_R_RATIO) が適用されます

---

### 3. motor モジュール (motor.ino)

#### motor_open()

```cpp
void motor_open();
```

**説明**: モーター PWM ピンを初期化し、PWM 出力を 0 に設定。

**パラメータ**: なし

**戻り値**: なし

**使用例**:
```cpp
void setup() {
  motor_open();  // setup() で必ず呼び出し
}
```

**内部処理**:
- PIN_DIR_L, PIN_PWM_L, PIN_DIR_R, PIN_PWM_R を OUTPUT に設定
- analogWrite() で PWM を 0 に初期化

---

#### motor_set()

```cpp
void motor_set(int pwm_l, int pwm_r);
```

**説明**: 左右モーターの PWM 値を設定 (-255 ～ +255)。

**パラメータ**:
- `int pwm_l`: 左モーター PWM (-255 ～ +255)
- `int pwm_r`: 右モーター PWM (-255 ～ +255)

**戻り値**: なし

**使用例**:
```cpp
motor_set(100, 100);   // 前進 (両モーター同速)
motor_set(100, -100);  // 回転 (左前進、右後進)
motor_set(-100, -100); // 後進
motor_set(0, 0);       // 停止
```

**内部処理**:
- PWM 値の符号で回転方向を決定
- 方向ピン (DIR) を HIGH/LOW で制御
- PWM ピンに絶対値を出力
- 範囲チェック: -255 ～ +255 に制限

---

### 4. vel_ctrl モジュール (vel_ctrl.ino)

#### vel_ctrl_set()

```cpp
void vel_ctrl_set(double vel_l, double vel_r);
```

**説明**: 目標速度 [cm/s] を設定します。

**パラメータ**:
- `double vel_l`: 目標左速度 [cm/s]
- `double vel_r`: 目標右速度 [cm/s]

**戻り値**: なし

**使用例**:
```cpp
vel_ctrl_set(25.0, 25.0);  // 直進: 25 cm/s
vel_ctrl_set(25.0, 20.0);  // カーブ: 左25, 右20
vel_ctrl_set(0.0, 0.0);    // 停止
```

**注意**: PID 制御が自動的に実行されます

---

#### vel_ctrl_execute()

```cpp
void vel_ctrl_execute();
```

**説明**: 現在の速度を測定し、PID 制御で PWM を計算・出力。

**パラメータ**: なし

**戻り値**: なし

**内部処理**:
- 距離微分で現在速度を計算: `v = Δdistance / Δtime`
- 誤差計算: `err = vel_ref - vel_curr`
- PID 制御: `PWM = Kp*err + Ki*err_sum + Kd*(err - err_prev)`
- motor_set() で PWM を出力

**使用例**:
```cpp
void loop() {
  vel_ctrl_set(25.0, 25.0);
  vel_ctrl_execute();  // loop() 内で毎回呼び出し
  delay(T_CTRL);
}
```

**PID ゲイン調整**:
```cpp
const double Kp = 0.4;  // 増加 → 応答が敏感 / 減少 → 応答が遅い
const double Ki = 0.1;  // 増加 → 定常偏差を補正 / 積分制御
const double Kd = 0.0;  // 増加 → 振動を抑制 / 微分制御
```

---

#### vel_ctrl_get()

```cpp
void vel_ctrl_get(double *vel_l, double *vel_r);
```

**説明**: 現在の実測速度 [cm/s] を取得。

**パラメータ**:
- `double *vel_l`: 実測左速度 (ポインタ)
- `double *vel_r`: 実測右速度 (ポインタ)

**戻り値**: なし

**使用例**:
```cpp
double vel_l, vel_r;
vel_ctrl_get(&vel_l, &vel_r);
Serial.print("Speed: ");
Serial.print(vel_l);
Serial.print(", ");
Serial.println(vel_r);
```

---

#### vel_ctrl_reset()

```cpp
void vel_ctrl_reset();
```

**説明**: 速度制御状態をリセット (PID 積分項をクリア、エンコーダリセット)。

**パラメータ**: なし

**戻り値**: なし

**使用例**:
```cpp
void loop() {
  if (start_signal) {
    vel_ctrl_reset();  // 新しい走行開始時
    vel_ctrl_set(25.0, 25.0);
  }
}
```

---

### 5. run_ctrl モジュール (run_ctrl.ino)

#### run_ctrl_set()

```cpp
void run_ctrl_set(run_state_t state, double speed, double dist);
```

**説明**: 走行モードと走行パラメータを設定。

**パラメータ**:
- `run_state_t state`: 走行モード (STP, STR, ROT, CIR, LTC, VEL, VLT)
- `double speed`: 速度 [cm/s] / 角速度 [deg/s] / 半径 [cm] / PWM値
- `double dist`: 走行距離 [cm] / 回転角度 [deg] / 角度 [deg] / 速度 [cm/s]

**戻り値**: なし

**走行モード詳細**:

| モード | speed の意味 | dist の意味 | 用途 |
|--------|-----------|-----------|------|
| STR | 速度 [cm/s] | 走行距離 [cm] | 直進 |
| ROT | 角速度 [deg/s] | 回転角度 [deg] | その場回転 |
| CIR | 円弧半径 [cm] | 円弧角度 [deg] | 曲線走行 |
| LTC | 速度 [cm/s] | 走行距離 [cm] | ライントレース |
| VEL | 左速度 [cm/s] | 右速度 [cm/s] | 速度指定 |
| VLT | 速度 [cm/s] | (無視) | 速度+ライントレース |

**使用例**:
```cpp
// 直進: 100 cm/s で 50 cm
run_ctrl_set(STR, 100, 50);

// 回転: 5 deg/s で 90 度
run_ctrl_set(ROT, 5, 90);

// ライントレース: 25 cm/s で 100 cm
run_ctrl_set(LTC, 25, 100);

// 円弧: 半径 30 cm で 90 度
run_ctrl_set(CIR, 30, 90);
```

---

#### run_ctrl_execute()

```cpp
void run_ctrl_execute();
```

**説明**: 設定された走行モードを実行し、速度指令値を vel_ctrl に指示。

**パラメータ**: なし

**戻り値**: なし

**内部処理**:
- 走行モードに応じた計算 (距離、角度、曲率など)
- 左右タイヤの速度差を PID で補正
- 減速処理 (減速開始距離に達したら速度を徐々に低下)
- 目標に到達したら自動的に STP に遷移

**使用例**:
```cpp
run_ctrl_set(STR, 100, 50);

while (1) {
  run_ctrl_execute();
  vel_ctrl_execute();
  delay(T_CTRL);
  
  // 完了判定
  run_state_t state;
  run_ctrl_get(&state, NULL, NULL);
  if (state == STP) break;  // 停止状態で終了
}
```

---

#### run_ctrl_get()

```cpp
void run_ctrl_get(run_state_t *state, double *speed, double *dist);
```

**説明**: 現在の走行状態を取得 (走行モード、実測速度、実測距離)。

**パラメータ**:
- `run_state_t *state`: 現在の走行モード (ポインタ)
- `double *speed`: 実測速度 (ポインタ)
- `double *dist`: 実測走行距離 (ポインタ)

**戻り値**: なし

**使用例**:
```cpp
run_state_t state;
double speed, dist;
run_ctrl_get(&state, &speed, &dist);

if (state == STP) {
  Serial.println("停止");
} else if (state == STR) {
  Serial.print("直進中 - 速度: ");
  Serial.print(speed);
  Serial.print(" cm/s, 距離: ");
  Serial.print(dist);
  Serial.println(" cm");
}
```

---

### 6. light モジュール (light.ino)

#### light_mod()

```cpp
int light_mod();
```

**説明**: ライトセンサ値から速度修正量を計算 (PD 制御)。

**パラメータ**: なし

**戻り値**: 速度修正量 [PWM相当] (-255 ～ +255)

**内部処理**:
- センサ値と基準値 (gray) の差を誤差として計算
- PD 制御: `output = Kp*err + Kd*(err - err_prev)`
- 大きい出力値 = 黒線から大きく外れている

**使用例**:
```cpp
void loop() {
  vel_ref = 25.0;  // 目標速度
  vel_mod = light_mod();  // ライントレース修正量
  
  vel_ctrl_set((vel_ref - vel_mod), (vel_ref + vel_mod));
  // 左: vel_ref - vel_mod
  // 右: vel_ref + vel_mod
  // → 黒線を追従しながら走行
}
```

**ゲイン調整**:
```cpp
static float Kp = 0.2;  // 比例ゲイン (感度)
static float Kd = 1.0;  // 微分ゲイン (安定性)
static int gray = 460;  // センサ基準値 (0-1023)
```

---

#### light_active() / light_inactive()

```cpp
void light_active();
void light_inactive();
```

**説明**: ライト LED をオン/オフします。

**使用例**:
```cpp
if (mode == LTC) {
  light_active();   // ライトをオン
  run_ctrl_set(LTC, 25, 100);
} else {
  light_inactive(); // ライトをオフ
}
```

---

### 7. raspi モジュール (raspi.ino)

#### raspi_open()

```cpp
void raspi_open();
```

**説明**: Raspberry Pi とのシリアル通信を初期化 (ボーレート 115200 bps)。

**パラメータ**: なし

**戻り値**: なし

**使用例**:
```cpp
void setup() {
  raspi_open();  // setup() で必ず呼び出し
}
```

---

#### raspi_receive()

```cpp
int raspi_receive(command_data_t *command_data);
```

**説明**: Raspberry Pi からコマンドを受信・デコード。

**パラメータ**:
- `command_data_t *command_data`: デコード後のコマンドデータ (ポインタ)

**戻り値**:
- `0`: 受信成功
- `-1`: 受信失敗 (データ不完全)

**使用例**:
```cpp
command_data_t cmd;
if (raspi_receive(&cmd) == 0) {
  Serial.print("Command: ");
  Serial.print(cmd.val[0]);  // 走行モード
  Serial.print(" ");
  Serial.print(cmd.val[1]);  // パラメータ1
  Serial.print(" ");
  Serial.println(cmd.val[2]); // パラメータ2
}
```

---

#### raspi_send()

```cpp
void raspi_send(command_data_t command_data);
```

**説明**: Arduino の状態を Raspberry Pi に送信・エンコード。

**パラメータ**:
- `command_data_t command_data`: 送信するコマンドデータ

**戻り値**: なし

**使用例**:
```cpp
command_data_t data;
data.val[0] = 2;    // STR (直進)
data.val[1] = 100;  // 速度
data.val[2] = 50;   // 距離
raspi_send(data);
```

---

### 8. io モジュール (io.ino)

#### io_open()

```cpp
void io_open();
```

**説明**: LED、バッテリー測定ピンを初期化。

**パラメータ**: なし

**戻り値**: なし

---

#### io_get_batt()

```cpp
double io_get_batt();
```

**説明**: バッテリー電圧 [V] を取得。

**戻り値**: バッテリー電圧 [V]

**計算式**:
```
V = analogRead(PIN_BATT) * 5.0 / 1024.0 / V_RATIO
```

**使用例**:
```cpp
double batt = io_get_batt();
if (batt < 9.0) {
  Serial.println("バッテリー低下警告");
  digitalWrite(PIN_LED, HIGH);  // LED 点灯
}
```

---

## Raspberry Pi API

### 1. request モジュール (request.c)

#### request_set_runmode()

```c
void request_set_runmode(run_state_t state, int speed, int dist);
```

**説明**: Arduino に走行モードコマンドを送信。

**パラメータ**:
- `run_state_t state`: 走行モード (STP, STR, ROT, CIR, LTC, VEL, VLT)
- `int speed`: 速度 [cm/s] など
- `int dist`: 走行距離 [cm] など

**戻り値**: なし

**使用例**:
```c
request_set_runmode(STR, 25, 100);  // 直進: 25 cm/s で 100 cm
request_set_runmode(LTC, 25, 100);  // ライントレース
request_set_runmode(STP, 0, 0);     // 停止
```

---

#### request_get_runmode()

```c
int request_get_runmode(run_state_t *state, int *speed, int *dist);
```

**説明**: Arduino から現在の走行状態を取得。

**パラメータ**:
- `run_state_t *state`: 走行モード (ポインタ)
- `int *speed`: 実測速度 (ポインタ)
- `int *dist`: 実測走行距離 (ポインタ)

**戻り値**:
- `0`: 通信成功
- `-1`: 通信失敗

**使用例**:
```c
run_state_t state;
int speed, dist;

if (request_get_runmode(&state, &speed, &dist) == 0) {
  printf("State: %d, Speed: %d, Distance: %d\n", state, speed, dist);
  if (state == STP) printf("走行完了\n");
}
```

---

#### request_get_dist()

```c
int request_get_dist(int *dist_l, int *dist_r);
```

**説明**: 左右の走行距離 [cm] を取得。

**パラメータ**:
- `int *dist_l`: 左走行距離 (ポインタ)
- `int *dist_r`: 右走行距離 (ポインタ)

**戻り値**:
- `0`: 通信成功
- `-1`: 通信失敗

**使用例**:
```c
int dist_l, dist_r;
request_get_dist(&dist_l, &dist_r);
printf("Left: %d cm, Right: %d cm\n", dist_l, dist_r);
```

---

#### request_get_batt()

```c
int request_get_batt(double *volt);
```

**説明**: バッテリー電圧 [V] を取得。

**パラメータ**:
- `double *volt`: バッテリー電圧 (ポインタ)

**戻り値**:
- `0`: 通信成功
- `-1`: 通信失敗

**使用例**:
```c
double volt;
if (request_get_batt(&volt) == 0) {
  printf("Battery: %.2f V\n", volt);
}
```

---

### 2. arduino モジュール (arduino.c)

底層の Serial 通信関数です。通常は request.c を使用します。

#### arduino_open()

```c
int arduino_open();
```

**説明**: Arduino とのシリアル接続を開く (/dev/ttyACM0, 115200 bps)。

**戻り値**:
- `0`: 接続成功
- `-1`: 接続失敗

---

#### arduino_send()

```c
void arduino_send(command_data_t command_data);
```

**説明**: コマンドをエンコード・送信。

---

#### arduino_receive()

```c
int arduino_receive(command_data_t *command_data);
```

**説明**: データを受信・デコード。

**戻り値**:
- `0`: 受信成功
- `-1`: 受信失敗

---

### 3. uss モジュール (uss.c)

#### uss_get_rf() / uss_get_rb() / uss_get_br() / uss_get_bl()

```c
long uss_get_rf();  // 右前方
long uss_get_rb();  // 右後方
long uss_get_br();  // 右後方
long uss_get_bl();  // 左後方
```

**説明**: 超音波センサから距離 [cm] を取得。

**戻り値**: 距離 [cm] (測定不可時は -1)

**使用例**:
```c
long dist_rf = uss_get_rf();
long dist_rb = uss_get_rb();

if (dist_rf > 0 && dist_rb > 0) {
  printf("Front: %ld cm, Back: %ld cm\n", dist_rf, dist_rb);
}
```

---

### 4. direction モジュール (direction.c)

#### direction_correct()

```c
int direction_correct(int dist);
```

**説明**: 超音波センサを用いて方向を正対補正 (壁に対して垂直に)。

**パラメータ**:
- `int dist`: 目標走行距離 [cm] (0 以下なら距離補正スキップ)

**戻り値**:
- `0`: 補正成功
- `-1`: 補正失敗 (規定回数超過)

**使用例**:
```c
if (direction_correct(100) == 0) {
  printf("方向補正完了\n");
} else {
  printf("補正失敗\n");
}
```

---

### 5. position モジュール (position.c)

#### position_set_coord()

```c
void position_set_coord(double x_s, double y_s, double dir_s);
```

**説明**: 現在位置と向きを設定。

**パラメータ**:
- `double x_s`: x 座標 [cm]
- `double y_s`: y 座標 [cm]
- `double dir_s`: 向き [度] (0 ～ 359)

**使用例**:
```c
position_set_coord(45.0, 45.0, 0.0);  // 初期位置を (45, 45) に設定
```

---

#### position_straight() / position_rotate()

```c
void position_straight(double dist);
void position_rotate(double angle);
```

**説明**: 走行距離・回転角度に基づいて位置を更新。

**使用例**:
```c
position_straight(100.0);  // 100 cm 進む
position_rotate(90.0);     // 90 度回転
```

---

#### position_get_coord() / position_get_area()

```c
void position_get_coord(double *x_g, double *y_g, double *dir_g);
char position_get_area();
```

**説明**: 現在位置・向き・エリアを取得。

**使用例**:
```c
double x, y, dir;
position_get_coord(&x, &y, &dir);
char area = position_get_area();
printf("Position: (%.1f, %.1f), Direction: %.0f deg, Area: %c\n",
       x, y, dir, area);
```

---

### 6. person モジュール (person.c)

#### person_check()

```c
int person_check();
```

**説明**: Python カメラプログラムの出力から人検出結果を読み込み。

**戻り値**:
- `1`: 人がいる
- `0`: 人がいない

**使用例**:
```c
if (person_check() == 1) {
  printf("人がいます\n");
  request_set_runmode(STP, 0, 0);  // 停止
} else {
  request_set_runmode(STR, 25, 100);  // 走行
}
```

**グローバル変数**:
```c
float area;  // 検出された人の領域面積
```

---

## データ構造

### command_data_t

```c
typedef struct {
  signed short val[3];
} command_data_t;
```

**説明**: Arduino ↔ Raspberry Pi 間でやり取りする 16-bit × 3 個のデータ。

**メンバー**:
- `val[0]`: 走行モード (1-7) または コマンド種別 (10-12)
- `val[1]`: パラメータ 1 (速度、距離など)
- `val[2]`: パラメータ 2 (距離、角度など)

---

### serial_data_t

```c
typedef struct {
  unsigned char val[7];
} serial_data_t;
```

**説明**: シリアル通信で使用する 7 バイトの生データ。

---

### run_state_t

```c
typedef enum {
  STP = 0,  // stop
  STR,      // straight
  ROT,      // rotation
  CIR,      // circle
  LTC,      // line trace
  VEL,      // specified velocity
  VLT       // line trace with velocity
} run_state_t;
```

**説明**: 走行モードを表す列挙型。

---

**最終更新**: 2026-04-27
