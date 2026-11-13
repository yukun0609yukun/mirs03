#!/bin/bash
# 2台目Jetson用: 出店紹介の再生が固まったら、自動で音声(voice_bridge)を入れ直す見張り(2026-11-07)。
#   bash ~/keep_voice_alive.sh        (2台目で実行。常駐。ログ ~/keep_voice_alive.log)
#
# なぜ要るか: 2026-11-06〜07 に、voice_bridge の再生が固まる(出店紹介が active なのに next_in=0.0 のまま plays が増えない)ことが
#   3回あった。固まると何も鳴らないので、人が入れ直す必要があった。この見張りが、固まりを見つけて入れ直す。
#
# 判定(誤判定を避けるため、どちらも長めに待つ):
#   A) 出店紹介が active で next_in_sec が 0.0 のまま、plays(再生回数)が 90秒以上増えない -> 固まり(長い文の再生でも約30秒で終わる)
#   B) /announce_state が 90秒以上返らない -> voice_bridge が落ちている
#   active でない(1台目が止めている: 緊急停止・アンケート・手動運転など)ときは、固まりではないので何もしない。
# 入れ直し: voice_bridge を止めて start_voice_stack.sh、そのあとスピーカーの音量を95%に戻す。入れ直しの間隔は最短180秒(連続で入れ直さない)。
LOG="$HOME/keep_voice_alive.log"
URL="http://127.0.0.1:8100"
STUCK_SEC=90; DOWN_SEC=90; COOLDOWN=180
last_plays=""; stuck_since=0; down_since=0; last_restart=0

log() { echo "$(date '+%F %T') $*" >> "$LOG"; }

restart_bridge() {
  local why="$1" now; now=$(date +%s)
  if [ $((now - last_restart)) -lt $COOLDOWN ]; then return; fi
  last_restart=$now
  log "voice_bridge を入れ直します: $why"
  pkill -f "[v]oice_bridge_merged" 2>/dev/null
  sleep 3
  nohup bash "$HOME/start_voice_stack.sh" >> "$HOME/start_voice_stack.log" 2>&1 < /dev/null &
  sleep 25
  amixer -c Device sset PCM 95% >/dev/null 2>&1
  log "入れ直しました(PCM 95%)"
}

while true; do
  now=$(date +%s)
  st=$(curl -s -m 4 "$URL/announce_state")
  if [ -z "$st" ]; then
    [ "$down_since" = 0 ] && down_since=$now
    if [ $((now - down_since)) -ge $DOWN_SEC ]; then restart_bridge "応答なし(${DOWN_SEC}秒)"; down_since=0; fi
  else
    down_since=0
    read -r active plays next_in <<< "$(printf '%s' "$st" | python3 -c '
import sys, json
try:
    p = json.load(sys.stdin)["loops"].get("pr_intro") or {}
    print(p.get("active"), p.get("plays"), p.get("next_in_sec"))
except Exception:
    print("False None None")
')"
    if [ "$active" = "True" ] && [ "$next_in" = "0.0" ]; then
      if [ "$plays" = "$last_plays" ]; then
        [ "$stuck_since" = 0 ] && stuck_since=$now
        if [ $((now - stuck_since)) -ge $STUCK_SEC ]; then restart_bridge "再生が${STUCK_SEC}秒以上進まない(plays=$plays)"; stuck_since=0; fi
      else
        stuck_since=0
      fi
    else
      stuck_since=0
    fi
    last_plays="$plays"
  fi
  sleep 10
done
