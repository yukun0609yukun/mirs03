# SETUP

このガイドは Raspberry Pi と Arduino を使って MIRS03 を動かすための手順を説明します。

## ハードウェア

- Raspberry Pi (推奨: 3B+/4)
- Arduino (スケッチは arduino/mg5_arduino_ver210.ino)
- 超音波センサ (I2C, 4台)
- モータ、エンコーダ、光センサ、電源
- USB ケーブル (Pi <-> Arduino)

## 前提ソフトウェア

Raspberry Pi:
- Raspberry Pi OS
- build-essential, make, gcc
- wiringPi (I2C / GPIO 用)
- libwiringPi-dev (インストール方法は下記)

Arduino:
- Arduino IDE または arduino-cli

### Raspberry Pi の準備 (簡易)

```bash
sudo apt update
sudo apt install -y build-essential git
# wiringPi が不要な環境では代替ライブラリを使用してください
sudo apt install -y wiringpi
```

### レポジトリの取得

```bash
git clone https://github.com/yukun0609yukun/mirs03.git
cd mirs03
git checkout documentation-setup
```

### Raspberry Pi 側のビルド

```bash
cd raspi
make all
# 実行ファイルは bin/ に出力されます
```

### Arduino スケッチのアップロード

1. Arduino IDE で `arduino/mg5_arduino_ver210.ino` を開く
2. ボードとポートを設定
3. アップロード

または arduino-cli をお使いの方は以下を参考にしてください：

```bash
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 arduino
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega:cpu=atmega2560 arduino
```

## トラブルシューティング

- Arduino が認識されない: `dmesg` でポートを確認。ドライバが必要な場合あり。
- I2C のセンサが見つからない: `i2cdetect -y 1` でアドレスを確認。
- シリアル通信が不安定: 両側でボーレート(115200)が揃っているか確認。

