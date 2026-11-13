#!/usr/bin/env bash
# 1台目Jetsonの電源投入時に、systemdサービス(nokosahen-robot.service)から呼ばれる「手動モード」の起動(2026-10-19追加)。
#   bash ~/mirs2603_nokosahen/scripts/autostart/boot_start.sh [manual]
# 手動モード = 自律行動(behavior_director)なし。Nav2・探索は、管理画面から人が開始するまで動かない(ロボットは自分では動かない)。
# 安全のため、自動起動できるのは manual だけ。full(自律行動あり)は、人が start_all.sh full --yes で起動する。
# start_all.sh manual と同じ構成(総合ダッシュボード、アンケート監視あり、ROS有線限定)。監視ループ(自動再起動つき)を
# フォアグラウンドで動かす(systemdがこのプロセスを監視する。終了したら、サービスは止まった状態になる)。
# ※2台目・OCR/Coralは、ここでは起動しない(2台目は自分のサービスで起動する。OCRは必要なときに start_all.sh --ocr)。
# ※Jetsonの時計は、再起動のたびに古い日付に戻る(RTC・インターネット無し)。起動後に、PCから start_all_with_clock.sh の
#   時計合わせ(docs/clock_sync.md)を行うこと。手動モードの動作自体は、時計に依存しない。
set -u
MODE="${1:-manual}"
if [ "$MODE" != "manual" ]; then
  echo "自動起動できるのは manual だけです(指定: $MODE)。full は start_all.sh full --yes で人が起動してください。" >&2
  exit 2
fi

REPO="${REPO:-$HOME/mirs2603_nokosahen}"
LOG="$HOME/.nokosahen_boot.log"
log() { echo "$(date '+%F %T') $*" >> "$LOG"; }

log "== boot_start.sh 開始(mode=$MODE user=$(id -un))"

# 【2026-11-06】文化祭モード(オプトイン)。フラグファイル ~/.nokosahen_festival_mode があるときだけ、festival_start.sh --boot に切り替える
# (自律行動の director あり。ただし近づく・見回す・向く・揺動はすべてOFFのまま。人が管理画面で「向く」だけONにする)。
# フラグが無い/festival_start.sh が無い場合は、下の従来どおりの手動モード。作る・消すのは scripts/festival/festival_mode.sh。
FESTIVAL_FLAG="${NOKOSAHEN_FESTIVAL_FLAG:-$HOME/.nokosahen_festival_mode}"
if [ -f "$FESTIVAL_FLAG" ] && [ -f "$REPO/scripts/festival/festival_start.sh" ]; then
  log "文化祭モードのフラグあり($FESTIVAL_FLAG)。festival_start.sh --boot へ切り替え"
  export NOKOSAHEN_FESTIVAL_FLAG="$FESTIVAL_FLAG"
  # デバイス待ち(下)をここでも行う
  for _ in $(seq 1 30); do
    [ -e /dev/esp32 ] && [ -e /dev/rplidar ] && break
    sleep 2
  done
  exec bash "$REPO/scripts/festival/festival_start.sh" --boot
fi

# ESP32とLiDARのudev名(/dev/esp32, /dev/rplidar)が現れるまで、最大60秒待つ(無くても起動は続ける。監視ループが再試行する)
for _ in $(seq 1 30); do
  [ -e /dev/esp32 ] && [ -e /dev/rplidar ] && break
  sleep 2
done
log "デバイス: esp32=$([ -e /dev/esp32 ] && echo あり || echo なし) rplidar=$([ -e /dev/rplidar ] && echo あり || echo なし)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
export EXTRA_LAUNCH_ARGS="use_full_dashboard:=true use_stage2_dashboard:=false use_survey_monitor:=true"

log "監視ループを起動(rc_only)"
exec bash "$REPO/scripts/run_persistent_base_supervised.sh" rc_only >> "$HOME/.nokosahen_supervisor_wrapper.log" 2>&1
