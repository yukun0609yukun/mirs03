# File Structure

主要ファイルの役割を簡潔にまとめます。

arduino/
- mg5_arduino_ver210.ino: Arduino のメインループと初期化
- define.h: 定数・型定義
- encoder.ino: エンコーダ ISR
- distance.ino: エンコーダから距離計算
- motor.ino: モータ駆動
- vel_ctrl.ino: 速度 PID
- run_ctrl.ino: 走行モード管理
- raspi.ino: シリアル通信 (RPi <-> Arduino)
- io.ino: LED・バッテリ読み取り
- light.ino: ライントレース用ロジック

raspi/
- arduino.c/h: Arduino 通信インターフェース
- request.c/h: 高レベルコマンド送受信
- uss.c/h: 超音波センサ I2C 操作
- direction.c/h: 距離/角度補正アルゴリズム
- position.c/h: 位置計算ユーティリティ
- person_*.c: 人検出との連携（ファイル読み込みベース）
- pilot_*.c: 実行プログラム
- test_*.c: 単体テスト用プログラム

docs/: ドキュメント
scripts/: ビルドや補助スクリプト

