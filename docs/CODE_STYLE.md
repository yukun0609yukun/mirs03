# MIRS03 コード規約・設計方針

プロジェクトの保守性・可読性・パフォーマンスを維持するためのガイドラインです。

## 📋 目次

- [命名規則](#命名規則)
- [ファイル構成](#ファイル構成)
- [関数設計](#関数設計)
- [エラーハンドリング](#エラーハンドリング)
- [パフォーマンス最適化](#パフォーマンス最適化)
- [ドキュメント記述](#ドキュメント記述)
- [デバッグ技法](#デバッグ技法)
- [レビューチェックリスト](#レビューチェックリスト)

---

## 命名規則

### 変数名

#### グローバル変数

```cpp
// ✅ 良い例
static volatile long count_l = 0;     // 割り込みハンドラで更新される
static double dist_prev_l = 0.0;      // static スコープ
static int gray = 460;                // センサ基準値

// ❌ 悪い例
volatile long cnt = 0;                // 略すな (count_l に)
static double d_p_l;                  // わかりにくい (dist_prev_l に)
static int GRAY = 460;                // グローバル定数でない場合は小文字
```

**ルール**:
- スネークケース (snake_case) を使用
- `static` スコープ変数は明示的に `static` 宣言
- 割り込み内で変更される変数は `volatile` を付ける
- 略さない (圧縮するな)

#### ローカル変数

```cpp
// ✅ 良い例
void distance_get(double *dist_l, double *dist_r) {
  long enc_l, enc_r;
  // 処理
}

// ❌ 悪い例
void distance_get(double *dl, double *dr) {
  long e_l, e_r;  // わかりにくい
}
```

#### ポインタ

```cpp
// ✅ 良い例
void encoder_get(long *cnt_l, long *cnt_r);
struct motor_state *motor = &motor_data;

// ❌ 悪い例
void encoder_get(long* cnt_l, long* cnt_r);  // * を変数名に付けない
struct motor_state* motor;                    // * を型に付けない
```

**ルール**:
- ポインタの `*` は変数名の直前に配置 (`long *cnt`)
- 関数ポインタは明示的な型定義を使う

### 関数名

```cpp
// ✅ 良い例
void encoder_open();              // 初期化
void encoder_get(long *cnt);      // 取得
void motor_set(int pwm);          // 設定
int arduino_receive(...);         // 受信 (戻り値は結果コード)

// ❌ 悪い例
void init_encoder();              // init ではなく open を使う
void get_encoder_value(...);      // get_XXX ではなく XXX_get
void SetMotor();                  // PascalCase ではなく snake_case
```

**ルール**:
- スネークケース
- 初期化: `module_open()`, `module_close()`
- 取得: `module_get()`
- 設定: `module_set()`
- 実行: `module_execute()`
- テスト: `test_module()`

### 型定義

```cpp
// ✅ 良い例
typedef struct {
  unsigned char val[7];
} serial_data_t;      // _t で終わる (型定義の慣例)

typedef enum {
  STP = 0,
  STR = 1
} run_state_t;        // _t で終わる

// ❌ 悪い例
typedef struct {
  unsigned char val[7];
} SerialData;         // _t を使わない

struct motor_config {  // typedef を使わない
  int pwm;
};
```

**ルール**:
- 型定義は `_t` で終わる
- `struct` / `enum` は常に `typedef` する

### マクロ・定数

```cpp
// ✅ 良い例
#define T_CTRL 10          // 制御周期 [ms]
#define PIN_ENC_A_L 2      // ピン番号
#define GEAR_RATIO 1.00    // ギア比

const double R_TIRE = 3.999;  // C++ スタイル (型安全)

// ❌ 悪い例
#define T_CTRL 10ms        // 単位をマクロに含めない
#define pin_enc_a_l 2      // 小文字 (マクロは大文字)
#define MOTOR_PWM_MAX 255  // MOTOR_PWM_MAX ではなく PWM_MAX_VALUE で関連性を示す
```

**ルール**:
- マクロは全て大文字
- 単位はコメントで記述
- 関連する定数は接頭辞でグループ化

---

## ファイル構成

### Arduino スケッチ構成

```
arduino/
├── mg5_arduino_ver210.ino    # メイン (setup/loop)
├── define.h                  # 定数・型定義
├── encoder.ino               # 独立したモジュール
├── motor.ino
├── distance.ino
├── velocity.ino
├── run_ctrl.ino
├── light.ino
├── raspi.ino
├── io.ino
├── slave.ino                 # スレーブモード実装
└── test.ino                  # テスト関数集
```

**ルール**:
- 機能ごとに `.ino` ファイルを分割
- 定数・型定義は `define.h` に集約
- 各モジュールは `module_open()`, `module_close()`, `module_execute()` を提供
- グローバル状態は `static` で隠ぺ

### Raspberry Pi C ソースコード構成

```
raspi/
├── Makefile              # ビルド設定
├── *.c / *.h             # モジュール (ペア)
├── pilot*.c              # 実行可能ファイル
├── test_*.c              # テストプログラム
└── bin/                  # コンパイル出力先
```

**ルール**:
- ヘッダファイル (`.h`) で インターフェース定義
- ソースファイル (`.c`) で実装
- `extern` 宣言で他モジュールから利用可能に
- `static` で内部実装を隠ぺ

---

## 関数設計

### 単一責任の原則 (Single Responsibility Principle)

```cpp
// ✅ 良い例
void encoder_open()          // 初期化のみ
void encoder_get(...)        // 読み込みのみ
void encoder_reset()         // リセットのみ

// ❌ 悪い例
void encoder_initialize_and_read(long *cnt) {
  // 初期化と読み込みが混在
  // テストしにくい
}
```

### 戻り値の設計

```cpp
// ✅ 良い例
int arduino_receive(command_data_t *cmd) {
  return 0;   // 成功
  return -1;  // 失敗 (理由は一律)
}

void motor_set(int pwm) {
  // 戻り値なし (設定は失敗しない)
}

// ❌ 悪い例
int encoder_get(long *cnt) {
  // ポインタ出力 + 戻り値は混在させない
}

int motor_set(int pwm) {
  return pwm_value;  // 設定結果を返すな
}
```

**ルール**:
- 出力パラメータはポインタで返す
- 成功/失敗は戻り値で示す (0 = 成功, -1 = 失敗)
- 単純な取得は戻り値で直接返す

### 関数の長さ

```cpp
// ✅ 良い例 (20行程度)
void motor_set(int pwm_l, int pwm_r) {
  if (pwm_l > 255) pwm_l = 255;
  if (pwm_l < -255) pwm_l = -255;
  // ...
  motor_set_left(pwm_l);
  motor_set_right(pwm_r);
}

// ❌ 悪い例 (100行以上)
void run_ctrl_execute() {
  // すべての走行モード処理
  // テストしにくく、保守しにくい
  // → 走行モードごとに関数に分割
}
```

**ルール**:
- 1 関数は 50 行以下が目安
- 複雑な処理は関数に分割

---

## エラーハンドリング

### 戻り値チェック

```cpp
// ✅ 良い例
int result = arduino_receive(&cmd);
if (result != 0) {
  printf("Error: Failed to receive command\n");
  return -1;  // 上位に伝播
}

// ❌ 悪い例
arduino_receive(&cmd);  // 戻り値無視
// 失敗していても気づかない
```

### 入力チェック

```cpp
// ✅ 良い例
void motor_set(int pwm_l, int pwm_r) {
  // 範囲チェック
  if (pwm_l > 255) pwm_l = 255;
  if (pwm_l < -255) pwm_l = -255;
  // ...
}

// ❌ 悪い例
void motor_set(int pwm_l, int pwm_r) {
  // 入力チェックなし → 予期しない挙動
  analogWrite(PIN_PWM_L, pwm_l);
}
```

### エラーメッセージ

```cpp
// ✅ 良い例
if (fd < 0) {
  printf("Error: Failed to open Arduino (/dev/ttyACM0)\n");
  printf("  Check: USB connection, driver installation\n");
  return -1;
}

// ❌ 悪い例
if (fd < 0) printf("Error\n");  // わかりにくい
```

**ルール**:
- エラーメッセージは原因と対策を含める
- デバッグモード (`#ifdef __DEBUG__`) で詳細ログを出力

---

## パフォーマンス最適化

### 割り込みハンドラ

```cpp
// ✅ 良い例 (短い)
static void enc_change_l() {
  int a_curr = digitalRead(PIN_ENC_A_L);
  int b_curr = digitalRead(PIN_ENC_B_L);
  // ...
  if (...) count_l++;
  // 処理時間: ~100 μs
}

// ❌ 悪い例 (長い)
static void enc_change_l() {
  Serial.println("Encoder changed");  // Serial 出力は遅い (~ms)
  // その間、メイン処理がブロック
}
```

**ルール**:
- 割り込みハンドラは最小限の処理
- Serial/I2C などのブロッキング I/O は使わない
- 処理時間は 100 μs 以下が目安

### メモリ効率

```cpp
// ✅ 良い例
static double dist_prev_l = 0.0;  // static → RAM 固定

// ❌ 悪い例
void vel_ctrl_execute() {
  double dist_prev_l = 0.0;  // 毎回割り当て/解放
}
```

**ルール**:
- 頻繁に使う変数は `static` で確保
- 動的割り当て (malloc) は避ける (組み込みシステム)

### CPU 時間

```cpp
// ✅ 良い例
if (count % 100 == 0) {
  // 100 回に 1 回だけ実行 (スケジューリング)
  slow_operation();
}

// ❌ 悪い例
slow_operation();  // 毎回実行 → CPU 占有率 ↑
```

**ルール**:
- 制御周期内に処理時間が収まることを確認
- 遅い処理は周期を落とす

---

## ドキュメント記述

### コメント

```cpp
// ✅ 良い例
static float Kp = 0.2;  // 比例ゲイン (増加 → 反応敏感)
static float Kd = 1.0;  // 微分ゲイン (増加 → 振動抑制)

// エンコーダ回転方向の補正
*cnt_l *= -1;

// ❌ 悪い例
static float Kp = 0.2;  // ゲイン
static float Kd = 1.0;  // ゲイン2

// cnt_l に -1 を乗じる
*cnt_l *= -1;
```

**ルール**:
- WHY (なぜ) を記述、WHAT (何) ではなく
- 調整可能なパラメータには説明を必須
- コード行当たり 1 コメントを目安に

### 関数ドキュメント

```cpp
// ✅ 良い例
/**
 * Get encoder count values
 * @param cnt_l  Left encoder count (pointer, corrected for direction)
 * @param cnt_r  Right encoder count (pointer)
 * @return None
 * @note Call from loop(), called in interrupt context
 */
void encoder_get(long *cnt_l, long *cnt_r);

// ❌ 悪い例
void encoder_get(long *cnt_l, long *cnt_r);  // No documentation
```

**ルール**:
- 関数の直前に Doxygen 形式のコメントを記述
- パラメータ、戻り値、副作用を明記

---

## デバッグ技法

### Serial 出力

```cpp
// ✅ 良い例
#ifdef __DEBUG_ENCODER__
Serial.print("enc_l = ");
Serial.print(enc_l);
Serial.print(", enc_r = ");
Serial.println(enc_r);
#endif

// ❌ 悪い例
Serial.println(enc_l);  // 本番コードに残ってしまう
```

**ルール**:
- デバッグ出力は `#ifdef __DEBUG_XXX__` で囲む
- 本番ビルドでは削除される

### テスト関数

```cpp
// ✅ 良い例
void test_distance() {
  // テスト用スタブ関数
  // Serial Monitor で確認
  while (1) {
    double dist_l, dist_r;
    distance_get(&dist_l, &dist_r);
    Serial.print("dist_l = ");
    Serial.println(dist_l);
    delay(T_CTRL);
  }
}

// 使用: setup() で test_distance(); を呼び出し
```

---

## レビューチェックリスト

コード提出前に確認:

- [ ] **命名規則**
  - [ ] 変数: snake_case
  - [ ] 関数: snake_case
  - [ ] マクロ: UPPER_CASE
  - [ ] 型: _t で終わる

- [ ] **関数設計**
  - [ ] 1 関数 50 行以下
  - [ ] 単一責任の原則
  - [ ] 戻り値の意味が明確

- [ ] **エラーハンドリング**
  - [ ] 全戻り値チェック
  - [ ] 入力値チェック
  - [ ] エラーメッセージ明確

- [ ] **ドキュメント**
  - [ ] 全関数にコメント
  - [ ] 魔法数に説明
  - [ ] Doxygen 形式

- [ ] **テスト**
  - [ ] 単体テスト (test_XXX)
  - [ ] 統合テスト
  - [ ] 境界値テスト

- [ ] **パフォーマンス**
  - [ ] 割り込み処理は短い
  - [ ] 制御周期内に処理が収まる
  - [ ] メモリ過度使用なし

---

**最終更新**: 2026-04-27
