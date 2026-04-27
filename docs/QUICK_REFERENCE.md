# Quick Reference

- シリアルボーレート: 115200
- Arduino 制御周期: T_CTRL = 10 ms
- 主なコマンド: request_set_runmode(STP/STR/ROT/CIR/LTC/VEL/VLT, speed, dist)

短いコマンド例:

Raspberry Pi から直進コマンドを送る:

```c
request_set_runmode(STR, 50, 100); // 速度50cm/s, 距離100cm
```

Arduino 側で現在の走行状態を取得する:

```c
command_data_t cmd;
cmd.val[0] = 10; // get mode
arduino_send(cmd);
// arduino_receive で応答を受け取る
```

パラメータ調整ヒント:
- PID: vel_ctrl.ino の Kp, Ki を調整
- ライントレース: light.ino の Kp, Kd, gray を調整

