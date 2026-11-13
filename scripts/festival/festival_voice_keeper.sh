#!/usr/bin/env bash
# 文化祭モード: 2台目のBGMをOFFに保つ見張り(2026-11-06追加)。festival_start.sh が裏で起動する。1台目で動く。
#   bash ~/mirs2603_nokosahen/scripts/festival/festival_voice_keeper.sh [--once]
# 2台目(voice_bridge)の BGM の ON/OFF は、保存されない(2台目の起動のたびに BGM_ENABLED=1 で始まる)。
# そのため、2台目が1台目より遅れて起動したり、途中で再起動したりしても、BGMがOFFになるよう、一定間隔で
# GET /bgm を見て、enabled=true なら POST /bgm {"enabled": false} を送る。2台目に届かない間は、何もせず待つ。
# 触るのは BGM の enabled だけ(音量・音声マスター・アナウンスは変えない)。認証は使わない(voice_bridgeはLAN内無認証)。
# 環境変数: FESTIVAL_VOICE_URL(既定 http://198.51.100.3:8100)、FESTIVAL_KEEPER_INTERVAL(既定 20 秒)。
# 停止: festival_start.sh が置く PID ファイル(~/.nokosahen_festival_keeper.pid)のプロセスに kill を送る(pkill -f は使わない)。
set -u
URL="${FESTIVAL_VOICE_URL:-http://198.51.100.3:8100}"
INTERVAL="${FESTIVAL_KEEPER_INTERVAL:-20}"
LOG="${FESTIVAL_KEEPER_LOG:-$HOME/.nokosahen_festival_keeper.log}"
ONCE=0; [ "${1:-}" = "--once" ] && ONCE=1

log() { echo "$(date '+%F %T') $*" >> "$LOG"; }

# 直前の状態(同じ内容を毎回ログに出さない)
LAST=""
tick() {
  local st
  st=$(curl -s -m 4 "$URL/bgm" 2>/dev/null) || st=""
  if [ -z "$st" ]; then
    [ "$LAST" = "unreachable" ] || log "2台目($URL)に届かない。待ちます"
    LAST="unreachable"; return 1
  fi
  if echo "$st" | grep -q '"enabled": *true'; then
    local r
    r=$(curl -s -m 4 -X POST -H 'Content-Type: application/json' -d '{"enabled": false}' "$URL/bgm" 2>/dev/null) || r=""
    if echo "$r" | grep -q '"enabled": *false'; then
      log "BGMをOFFにしました"; LAST="off"
    else
      log "BGMをOFFにできませんでした(応答: ${r:0:120})"; LAST="failed"; return 1
    fi
  else
    [ "$LAST" = "off" ] || [ "$LAST" = "already" ] || log "BGMはOFFです"
    LAST="already"
  fi
  return 0
}

if [ "$ONCE" = "1" ]; then tick; exit $?; fi
log "== 見張り開始(url=$URL interval=${INTERVAL}s pid=$$)"
while true; do
  tick
  sleep "$INTERVAL"
done
