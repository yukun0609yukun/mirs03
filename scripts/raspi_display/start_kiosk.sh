#!/bin/bash
# 腹部ディスプレイ用 Chromiumキオスク起動スクリプト (Raspberry Pi 4 / labwc)
# 設定は ~/.kiosk_env (chmod 600) から読む:
#   KIOSK_PATH=/customer_display  KIOSK_HOSTS="203.0.113.6:8080 198.51.100.2:8080"
#   KIOSK_USER=ADMIN_ACCOUNT  KIOSK_PASS=<web_uiのパスワード>  (値に記号が入る場合は'...'で囲む)
# Jetsonが後から起動しても、ページが応答するまで待ってから表示する。
. "$HOME/.kiosk_env"
LOG=$HOME/kiosk.log
exec >>"$LOG" 2>&1
echo "=== $(date) start_kiosk"
# 二重起動防止(flockで排他。以前はps+awkで同名プロセスを探していたが、$(...)のサブシェルが
# 自分自身と同じコマンドラインを持つため「既に起動中」と誤判定して即終了し、キオスクが
# 一度も起動しなかった(2026-10-02修正))
exec 9>"/tmp/kiosk.lock"
flock -n 9 || { echo "already running (lock held)"; exit 0; }
# 表示モード: ~/.kiosk_mode が survey ならアンケート(ラズパイ上 127.0.0.1:8082)、それ以外は
# お客さん向けの KIOSK_PATH(既定)。切替は switch_kiosk.sh customer|survey。
URL=""
wait_upstream() {  # Jetsonのページが応答するまで待ち、URL(認証代行プロキシ経由)を決める
  until [ -n "$URL" ]; do
    for h in $KIOSK_HOSTS; do
      code=$(curl -s -m 5 -u "$KIOSK_USER:$KIOSK_PASS" -o /dev/null -w '%{http_code}' "http://$h$KIOSK_PATH")
      if [ "$code" = "200" ]; then URL="http://127.0.0.1:8081$KIOSK_PATH"; echo "reachable: $h"; break; fi
    done
    [ -z "$URL" ] && { echo "waiting (last http=$code)"; sleep 5; }
  done
}
# 認証代行プロキシ(kiosk_proxy.py)。URLに認証情報を埋め込むとページ内のfetchが拒否されて
# 「通信エラー」になるため、Basic認証はこちらで付与し、Chromiumは認証無しのURLを開く。
PROXY_PID=$(pgrep -f "python3 $HOME/kiosk_proxy.py" | head -1)
[ -z "$PROXY_PID" ] && { setsid nohup python3 "$HOME/kiosk_proxy.py" 9>&- >>"$LOG" 2>&1 < /dev/null & sleep 1; echo "proxy started"; }
# アンケート受信サーバ(回答は ~/survey_data に保存。ハブ側 198.51.100.1 では GET /state のみ)と
# 管理用API(kiosk_admin.py、198.51.100.1:8083、呼び出し元は1台目 198.51.100.2 のみ)。
# どのモードでも常時起動する(お客さん向け表示中でも1台目の管理画面から件数・切替ができるように)。
# 生死は curl のHTTPコードで見る(403でも「生きている」。pgrep -f は使わない)。
ensure_services() {
  [ "$(curl -s -m 2 -o /dev/null -w '%{http_code}' http://127.0.0.1:8082/state)" = "200" ] || {
    setsid nohup python3 "$HOME/survey/receiver.py" --port 8082 --also-host 198.51.100.1 9>&- >>"$HOME/survey.log" 2>&1 < /dev/null &
    sleep 1; echo "survey server started"; }
  [ "$(curl -s -m 2 -o /dev/null -w '%{http_code}' http://198.51.100.1:8083/state)" != "000" ] || {
    setsid nohup python3 "$HOME/kiosk_admin.py" --host 198.51.100.1 --port 8083 9>&- >>"$HOME/kiosk_admin.log" 2>&1 < /dev/null &
    sleep 1; echo "kiosk_admin started"; }
}
ensure_services
# Chromiumが落ちても復帰するループ(ページ側のポーリングが通信断からの復帰を待つ)。
# 毎回モードを読み直すので、switch_kiosk.sh がChromiumを閉じるだけで表示が切り替わる。
while true; do
  MODE=$(cat "$HOME/.kiosk_mode" 2>/dev/null)
  if [ "$MODE" = "survey" ]; then
    ensure_services   # 受信サーバ・管理APIが落ちていれば起動し直す
    OPEN="http://127.0.0.1:8082/"
  elif [ "$MODE" = "ocrtest" ]; then
    # OCRテスト用の模擬POS画面(静的HTMLをローカル配信。127.0.0.1のみ・LAN非公開)
    [ "$(curl -s -m 2 -o /dev/null -w '%{http_code}' http://127.0.0.1:8084/mock_pos_tablet.html)" = "200" ] || {
      setsid nohup python3 -m http.server 8084 --bind 127.0.0.1 --directory "$HOME/ocr_test" 9>&- >>"$HOME/ocr_test.log" 2>&1 < /dev/null &
      sleep 1; echo "ocr_test server started"; }
    OPEN="http://127.0.0.1:8084/mock_pos_tablet.html"
  else
    wait_upstream
    OPEN="$URL"
  fi
  echo "open ($MODE): $OPEN"
  chromium 9>&- --ozone-platform=wayland --kiosk --noerrdialogs --disable-infobars \
    --disable-session-crashed-bubble --no-first-run --no-default-browser-check \
    --user-data-dir="$HOME/.kiosk-profile" --password-store=basic \
    --check-for-update-interval=31536000 --disable-features=Translate,MediaRouter \
    --disable-pinch --overscroll-history-navigation=0 --disable-component-update \
    "$OPEN"
  echo "chromium exited $(date), restart in 2s"
  sleep 2
done
