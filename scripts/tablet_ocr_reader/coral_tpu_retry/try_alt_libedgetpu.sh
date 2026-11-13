#!/usr/bin/env bash
# 戦略2: Google公式libedgetpu(2021年ビルドのまま更新停止)の代わりに、
# 有志が新しめのglibc/gcc向けに再ビルドして公開しているlibedgetpu
# (feranick/libedgetpu、GitHub上の有志プロジェクト)を試す。
#
# 公式ビルドとの世代差(ABI不一致)が原因なら、これで直る可能性がある。
# システムのapt管理下には入れず、venv内のローカルパスに置くだけなので、
# 気に入らなければファイルを消すだけで元に戻せる。
#
# 使い方:
#   bash try_alt_libedgetpu.sh
set -euo pipefail

VENV="$HOME/nokosahen_tablet_ocr_venv"
WORKDIR="$HOME/coral_tpu_retry_work"
mkdir -p "$WORKDIR"
cd "$WORKDIR"

echo "=== feranick/libedgetpu の最新リリース情報を取得(URLを推測せず、GitHub APIから動的に取得) ==="
curl -s https://api.github.com/repos/feranick/libedgetpu/releases/latest > release.json

echo "アーキテクチャ(aarch64)向けのビルド済みアセットを検索..."
python3 - <<'PYEOF'
import json
with open("release.json") as f:
    data = json.load(f)
print("リリースタグ:", data.get("tag_name"))
print()
print("アセット一覧(この中からaarch64/arm64向けを手動で選んで次のステップへ):")
for a in data.get("assets", []):
    print(" -", a["name"], "->", a["browser_download_url"])
PYEOF

echo
echo "=== 次の手順(手動) ==="
echo "1. 上の一覧から、aarch64/arm64・Debian/Linux向けのファイルを選ぶ"
echo "   (例: throttled版=libedgetpu1-std相当、max版=libedgetpu1-max相当。"
echo "    まずthrottled/std版から試すこと。発熱・安定性の観点で無難)"
echo "2. 以下でダウンロード・展開する:"
echo "   curl -LO <選んだURL>"
echo "   ダウンロードした.debなら: dpkg-deb -x <ファイル名>.deb extracted/"
echo "   (パッケージ形式でなくtar.gz等なら: tar xf <ファイル名>)"
echo "3. 展開した中の libedgetpu.so.1(または同等の.so)を見つけ、以下で試す:"
echo "   LD_LIBRARY_PATH=<展開先のlibがあるディレクトリ> $VENV/bin/python3 -c \""
echo "from ai_edge_litert.interpreter import Interpreter, load_delegate"
echo "d = load_delegate('libedgetpu.so.1')"
echo "i = Interpreter(model_path='$HOME/tablet_ocr_reader/digit_model/digit_model_int8_edgetpu.tflite', experimental_delegates=[d])"
echo "i.allocate_tensors()"
echo "print('PROBE_OK (代替libedgetpu)')"
echo "\""
echo
echo "うまくいったら(PROBE_OKが出たら)、システムの/usr/lib/aarch64-linux-gnu/libedgetpu.so.1を"
echo "元の状態に戻した上で(念のためバックアップを取ってから)、この新しいものに差し替える相談をしてください。"
