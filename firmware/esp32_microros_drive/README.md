# ESP32 micro-ROS drive firmware

指定ピンアサインの足回り制御 firmware です。

- Subscribe: `/cmd_vel` (`geometry_msgs/Twist`)
- Publish: `/wheel_ticks` (`std_msgs/Int32MultiArray`, `[left, right]`)
- Publish: `/estop_state` (`std_msgs/Bool`, `true`=キャタピラ主電流カットが入っている状態)

RasPi/Jetson側の`wheel_odom`ノードが`/wheel_ticks`から`/odom`とTFを生成します。

実機で追い込んだ`WHEEL_RADIUS`/`WHEEL_BASE`/PIDゲイン(前進/後退別)の最終値は
[`../esp32_microros_drive_v2/README.md`](../esp32_microros_drive_v2/README.md)
を参照(このディレクトリと`esp32_microros_drive_v2/`は現在同じ値で統一されている)。

## ピンアサイン

| 機能 | ピン |
| --- | --- |
| エンコーダ 左A (`PIN_ENC_A_L`) | GPIO4 |
| エンコーダ 左B (`PIN_ENC_B_L`) | GPIO5 |
| エンコーダ 右A (`PIN_ENC_A_R`) | GPIO13 |
| エンコーダ 右B (`PIN_ENC_B_R`) | GPIO14 |
| モータ右 DIR (`PIN_DIR_R`) | GPIO25 |
| モータ右 PWM (`PIN_PWM_R`) | GPIO26 |
| モータ左 DIR (`PIN_DIR_L`) | GPIO32 |
| モータ左 PWM (`PIN_PWM_L`) | GPIO33 |
| 緊急停止入力 (`PIN_ESTOP`) | GPIO34 |
| I2C SDA/SCL | GPIO21/GPIO22 |

## 緊急停止（キャタピラ主電流カット）スイッチの配線

`PIN_ESTOP`（GPIO34）は、モータ主電流を物理的に遮断するスイッチ/リレーの状態を**読み取るだけ**のピンです（遮断の実行自体はハードウェア側で行われ、ESP32はそれを検知して`/estop_state`にpublishするのみ）。

- GPIO34-39はESP32の入力専用ピンで、内部プルアップ/プルダウン抵抗を持たない。**外部プルダウン抵抗（例: 10kΩをGNDへ）を配線し、遮断時にHIGHになるよう結線すること**（未配線・浮遊状態だとノイズで誤検出する）。
- `/estop_state`は状態が変化した時に即時publishし、加えて`ESTOP_HEARTBEAT_MS`(100ms)周期でもハートビートとして現在値をpublishし続ける（購読側の途中参加・取りこぼし対策）。
- ソフト側は20ms周期で読み取り、`ESTOP_DEBOUNCE_SAMPLES`（デフォルト5回=約100ms）連続で同じ値が続いたら状態を確定する。
- 遮断確定時はPIDの積分項をリセットし、速度目標も0に固定する（モータは物理的に動かないが、ワインドアップ防止のため）。

## I2C（将来のディスプレイ/センサー用に予約）

GPIO21(SDA)/GPIO22(SCL)はESP32のデフォルトI2Cピンと一致しているため、配線・`Wire.begin()`以外のコード変更は不要。現時点では未使用。

## 書き込み

```bash
pio run -t upload
```

## micro-ROS Agent

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

