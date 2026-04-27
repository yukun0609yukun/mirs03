# Project Structure

このプロジェクトは二つの実行環境で構成されています。

1. Arduino (リアルタイム制御): センサ、エンコーダ、モータの低レベル制御と速度制御ループを担当。
2. Raspberry Pi (高レベル制御): センサ集約、経路計画、人検出、運用ロジックを担当。

データフローの概略:

Arduino: センサ読取 -> 速度制御(PID) -> モータ出力
Arduino <-> Raspberry Pi: シリアルコマンド (7バイト) で同期
Raspberry Pi: 超音波等センサ(I2C) -> 高レベル判断 -> Arduino にコマンド

