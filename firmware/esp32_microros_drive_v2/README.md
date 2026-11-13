# ESP32 micro-ROS drive firmware (v2: 実機実測値ベース再設計版)

`../esp32_microros_drive/`(オリジナル)と全く同じインターフェース
(`/cmd_vel`購読、`/wheel_ticks`・`/estop_state`・`/nokosahen/pid_gains_state`・
`/nokosahen/noise_guard_counts`publish、`/nokosahen/pid_gains`でのゲイン
実行時調整)を持つ、独立した別ディレクトリです。

## オリジナルとの関係(2026-07-10時点で値は統一済み)

このv2は、過去にPython+Arduinoでライントレースさせていた実機調整済み
プログラム(`Documents\高専\mirs\mg5_arduino_ver120\`)の値(`WHEEL_RADIUS`/
`WHEEL_BASE`)と、実機ゲイン調整UIで追い込んだ最終PIDゲインを反映するために
作られた検証用の別ディレクトリだった。その後の実機テストでこれらの値が
有効だと確認できたため、**現在は`../esp32_microros_drive/`(オリジナル)側にも
同じ値・同じ前進/後退別ゲイン構成を反映済みで、両者は機能的に同一**になっている。
どちらをビルド・書き込みしてもよい(本番としてはオリジナルの
`esp32_microros_drive/`を正とする想定)。

### 現在の値(両ディレクトリ共通)

| 項目 | 値 | 根拠 |
| --- | --- | --- |
| `WHEEL_RADIUS` | 0.03999 | 旧`define.h`の`R_TIRE=3.999cm`(実走行で確認済み。定規での新規測定値0.02は誤りだった) |
| `WHEEL_BASE` | 0.3883 | 旧`define.h`の`D_TIRE=38.83cm` |
| 右車輪 前進 KP/KI/KD | 6.0 / 1.0 / 0.0 | 実機ゲイン調整UIで追い込んだ最終値 |
| 右車輪 後退 KP/KI/KD | 6.0 / 0.0 / 0.0 | 同上 |
| 左車輪 前進 KP/KI/KD | 6.1 / 1.0 / 0.0 | 同上 |
| 左車輪 後退 KP/KI/KD | 6.1 / 0.0 / 0.0 | 同上 |

前進(FWD)/後退(BWD)で別ゲインを持てるようにしているのは、実機テストで
「後退すると直進からずれる」ことが分かり、キャタピラの前後非対称性を
吸収する必要があったため。

`ENC_INVERT_L/R`・`INVERT_L/R`・ピン配置・ノイズガード(ティッククランプ/
スルーレート制限)・ハードウェアウォッチドッグは、これまでの実機テストで
確認した設定をそのまま引き継いでいる。

### 台形加減速+臨界減衰(2026-07-17追加)

`/cmd_vel`で受け取った目標値(体座標系のv・ω)は、PIDに渡す前に
`config.h`の`RAMP_ACCEL_*`/`RAMP_DECEL_*`(非対称スルーレート制限)と
`RAMP_OMEGA_N_*`(臨界減衰フィルタ、ζ=1固定でオーバーシュートしない)を
通してから左右輪へ混合するようになった。PIDゲイン自体は変更していない。
estop・通信ウォッチドッグ発動時はこのランプをバイパスして即座にゼロにする
(従来通りの即時停止を維持)。実機で発進・停止の感触を見ながら
`config.h`の各定数を調整すること。

---

指定ピンアサインの足回り制御 firmware です。

- Subscribe: `/cmd_vel` (`geometry_msgs/Twist`)
- Publish: `/wheel_ticks` (`std_msgs/Int32MultiArray`, `[left, right]`)
- Publish: `/estop_state` (`std_msgs/Bool`, `true`=キャタピラ主電流カットが入っている状態)

RasPi/Jetson側の`wheel_odom`ノードが`/wheel_ticks`から`/odom`とTFを生成します。

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


## 2026-10-22 の安全強化(全体レビュー3A)

- `/cmd_vel` をファーム側でも頭打ち(直進1.2m/s・旋回4rad/s、NaN/Infは0)。`include/safety_limits.h`
- `/nokosahen/pid_gains` は負・NaN・Inf・200超が1つでもあれば全体を無視
- `/nokosahen/soft_estop`(Bool)を購読。trueの間は`/cmd_vel`を無視して即停止(ダッシュボードが1秒ごとに再送)
- 通信断(ウォッチドッグ)は、即ゼロではなく通常の減速ランプで停止。ハード/ソフト緊急停止は従来通り即ゼロ
- PCでの単体テスト: `g++ -std=c++17 -I include test/test_safety_limits.cpp -o /tmp/t && /tmp/t`
- **書き込み済み**(2026-10-22に、1台目のJetsonからesptoolで書き込み、検証OK)。前進/後退/緊急停止ボタン/通信断での停止は、実走での確認が未了
