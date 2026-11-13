#!/bin/bash
# 腹部ディスプレイの表示切替 (ラズパイ上で実行。1台目からは `ssh pi ./switch_kiosk.sh survey`)
#   switch_kiosk.sh customer   お客さん向け /customer_display (メニュー・残数)
#   switch_kiosk.sh survey     アンケート (ラズパイ上 127.0.0.1:8082。回答は ~/survey_data)
#   switch_kiosk.sh ocrtest    OCRテスト用の模擬POSタブレット(~/ocr_test/mock_pos_tablet.html。127.0.0.1:8084)を全画面表示
#   switch_kiosk.sh status     いまのモードと回答件数(0件なら「0件」)を表示
# 仕組み: ~/.kiosk_mode にモードを書き、Chromiumを閉じる。start_kiosk.sh のループが
# 数秒でモードに合わせて開き直す。モードは再起動後も残る(customer以外を消すには customer を実行)。
MODE_FILE="$HOME/.kiosk_mode"
case "$1" in
  customer|survey|ocrtest)
    echo "$1" > "$MODE_FILE"
    pkill -x chromium   # 'chromium' という名前のプロセスだけを閉じる(このスクリプトは巻き込まない)
    echo "switched to $1 (Chromiumが数秒で開き直します)" ;;
  status)
    echo "mode: $(cat "$MODE_FILE" 2>/dev/null || echo customer)"
    # 回答件数: 受信サーバが動いていれば /state、止まっていても回答ファイルを直接数える。
    # (以前は customer モードだと受信サーバが起動しておらず「not running」だけで件数が出なかった)
    ST=$(curl -s -m 2 http://127.0.0.1:8082/state)
    if [ -n "$ST" ]; then
      echo "$ST" | python3 -c 'import sys,json; d=json.load(sys.stdin); print("回答件数: %d件 / 回答中: %s / 実験条件: %s" % (d["count"], "はい" if d["active"] else "いいえ", d["condition"] or "未設定"))'
    else
      F="${SURVEY_DATA_DIR:-$HOME/survey_data}/responses.jsonl"
      N=$(grep -c . "$F" 2>/dev/null); N=${N:-0}
      echo "回答件数: ${N}件 (受信サーバ停止中。ファイルを直接数えました)"
    fi ;;
  *) echo "usage: $0 customer|survey|ocrtest|status"; exit 1 ;;
esac
