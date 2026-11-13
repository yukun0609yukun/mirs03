#!/bin/bash
# Edge TPUコンパイラ(edgetpu_compiler)のインストール手順。
# x86-64のLinux(WSL2含む)専用。Raspberry Pi(ARM)上では動かないので、
# このスクリプトはPC(WSL2)側で実行し、コンパイル済みモデルだけを
# Raspberry Piへコピーする運用にすること。
#
# sudoパスワードが必要なため、この一連のコマンドはユーザー自身の端末で
# 直接実行してください。
set -e
set -o pipefail

echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
# -f: HTTPエラー(404等)でも0を返してしまうcurlのデフォルト挙動を防ぎ、
# 失敗を確実に検知する。pipefail無しだとapt-key add側の終了コードだけが
# 見られ、curl失敗(鍵が空/不正)を素通りしてしまっていた(2026-09-15レビューで発見)。
curl -f https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y edgetpu-compiler

echo "インストール完了。次のコマンドでモデルをEdge TPU向けに変換できます:"
echo "  edgetpu_compiler -o <出力先ディレクトリ> <digit_model_int8.tflite>"
