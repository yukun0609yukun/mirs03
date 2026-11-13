#!/usr/bin/env bash
# Coral TPUが今どういう状態か、一通り確認するだけのスクリプト(何も変更しない)。
set -uo pipefail

echo "=== OS ==="
cat /etc/os-release | head -3
uname -m

echo
echo "=== Coral USBアクセラレータ (lsusb) ==="
echo "起動直後(1a6e:089a)のままか、ランモード(18d1:9302)まで進んでいるか確認"
lsusb | grep -Ei "1a6e|18d1|coral|global unichip" || echo "見つかりません(挿さっていない?)"

echo
echo "=== libedgetpu ==="
dpkg -l | grep -i edgetpu || echo "libedgetpu1-std 等が未インストール"
ldconfig -p | grep -i edgetpu || echo "libedgetpu.so が見つかりません"

echo
echo "=== Docker ==="
if command -v docker >/dev/null 2>&1; then
    docker --version
else
    echo "Dockerが未インストール(戦略1を試す場合はまずインストールが必要)"
fi

echo
echo "=== venv内のPythonパッケージ ==="
VENV="$HOME/nokosahen_tablet_ocr_venv"
if [ -d "$VENV" ]; then
    "$VENV/bin/pip" list 2>/dev/null | grep -Ei "litert|tflite|pycoral|numpy|opencv"
else
    echo "venv ($VENV) が見つかりません"
fi

echo
echo "=== 現在CPUフォールバックで動くか確認(クラッシュしないことの再確認のみ) ==="
if [ -d "$VENV" ] && [ -f "$HOME/tablet_ocr_reader/digit_model/digit_model_int8.tflite" ]; then
    "$VENV/bin/python3" -c "
from ai_edge_litert.interpreter import Interpreter
i = Interpreter(model_path='$HOME/tablet_ocr_reader/digit_model/digit_model_int8.tflite')
i.allocate_tensors()
print('CPU推論: OK')
" 2>&1
else
    echo "モデルファイルが見つからないためスキップ"
fi
