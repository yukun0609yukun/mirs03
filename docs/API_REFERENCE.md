# API Reference

このファイルは主要モジュールと使い方の概要を示します。詳細は各ソースファイルのコメントを参照してください。

## Arduino 側 (sketch)

- encoder_open() / encoder_get(&enc_l, &enc_r) / encoder_reset()
  - エンコーダの初期化・読み出し

- distance_get(&dist_l, &dist_r)
  - エンコーダ値から左右の走行距離[cm]を計算して返す

- motor_open() / motor_set(pwm_l, pwm_r)
  - モータの初期化・PWM 出力

- vel_ctrl_set(vel_l, vel_r) / vel_ctrl_execute() / vel_ctrl_reset()
  - 速度制御 (PID)

- run_ctrl_set(state, speed, dist) / run_ctrl_execute() / run_ctrl_get()
  - 走行モード管理 (STP, STR, ROT, CIR, LTC, VEL, VLT)

- raspi_open()/raspi_receive()/raspi_send()
  - Raspberry Pi とのシリアル通信ラッパー (7バイト固定パケット)

- light_mod(), light_active(), light_inactive()
  - ライントレース用処理

## Raspberry Pi 側 (C)

- arduino_open()/arduino_receive()/arduino_send()
  - シリアル通信: Arduino との送受信を行う

- request_set_runmode(state, speed, dist)
  - Arduino に走行指令を送る

- request_get_runmode(), request_get_dist(), request_get_batt()
  - Arduino から状態、距離、バッテリ情報を取得

- uss_open_*(), uss_get_*()
  - 超音波センサ(I2C)の初期化と取得

- direction_correct(int dist)
  - センサ情報を元に距離/角度補正を行う高レベル関数

- person_check()
  - 外部ファイル (person_check_py) から人検知の結果を読み込む

## データ構造

共通で使用されるデータ型:

```c
typedef struct { unsigned char val[7]; } serial_data_t; // シリアルパケット
typedef struct { unsigned char val[6]; } middle_data_t;
typedef struct { signed short val[3]; } command_data_t; // コマンド/レスポンス
```

