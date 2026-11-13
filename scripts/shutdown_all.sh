#!/usr/bin/env bash
# 1台目Jetsonで実行する「プログラム終了(全停止)」スクリプト(2026-10-13追加)。docs/system_shutdown.md
# 管理画面の「設定 > メンテナンス > プログラム終了(全停止)」から、切り離した別プロセス(setsid)として起動される。
#   bash ~/mirs2603_nokosahen/scripts/shutdown_all.sh [--delay 秒] [--no-ocr] [--voice]
#     --delay N   最初にN秒待つ(管理画面がブラウザへ応答を返し終えるのを待つ。既定3)
#     --no-ocr    タブレットOCRとCoralサーバーを止めない(既定は止める)
#     --voice     2台目の音声(voice_bridge/BGM)も止める(ssh j2。既定は止めない)
#     --poweroff  最後に、ラズパイ・2台目・1台目のOSも停止して電源を落とす(2026-10-19〜。sudoersで shutdown -h now だけを
#                 パスワード無しで許可してある前提: install_autostart.sh が1台目・2台目に入れる。ラズパイのpiユーザーは既定でsudo可)。
#                 許可が無い機器は、電源を落とさず(ログに残して)プログラムの停止だけで終わる。docs/autostart.md
# 止める順序(安全第一):
#   0) モーターの再停止(ソフト緊急停止とゼロ速度を、他のノードが落ちる前に念のためもう一度。管理画面側で発行済み)
#   1) 監視ループ(run_persistent_base_supervised.sh)  <- 先に止めないと、ROSを止めても自動で再起動される
#   2) ロボット(ROS一式)  <- stop_robot.sh --keep-dashboard (管理画面は残す)
#   3) タブレットOCR / Coral          (--no-ocr で省略)
#   4) 2台目の音声                    (--voice のときだけ)
#   5) 管理画面自身(persistent_base launch / full_dashboard)  <- これで管理画面にも入れなくなる
#   6) (--poweroff のとき)ラズパイ・2台目のOSを停止 → 最後に1台目のOSを停止
# --poweroff が無いときは、OSの電源断はしない。止まらないもの: Jetsonの電源・Wi-Fiテザリング・SSH・ラズパイの画面・2台目(--voiceで音声だけ停止)。
# 結果は ~/.nokosahen_shutdown.log に追記する(画面は途中で消えるため、後でSSHで確認する)。
# 注意: このファイル内の pkill -f / pgrep -f のパターンは 'recei[v]er' のように1文字を[]で割ること(自分自身のコマンド
#       文字列に一致して自分を巻き込むのを防ぐ)。test_system_shutdown.py が静的に確認している。
set -u
DELAY=3; OCR=1; VOICE=0; POWEROFF=0
while [ $# -gt 0 ]; do
  case "$1" in
    --delay) DELAY="${2:-3}"; shift ;;
    --no-ocr) OCR=0 ;;
    --voice) VOICE=1 ;;
    --poweroff) POWEROFF=1 ;;
    *) echo "不明な引数: $1" >&2; exit 2 ;;
  esac
  shift
done
case "$DELAY" in ''|*[!0-9]*) DELAY=3 ;; esac

LOG="${NOKOSAHEN_SHUTDOWN_LOG:-$HOME/.nokosahen_shutdown.log}"
exec >>"$LOG" 2>&1
log() { echo "$(date '+%F %T') $*"; }
log "== shutdown_all.sh 開始 (delay=$DELAY ocr=$OCR voice=$VOICE poweroff=$POWEROFF pid=$$)"

sleep "$DELAY"

log "== 0) モーターの再停止(ベストエフォート。ソフト緊急停止+ゼロ速度)"
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$HOME/mirs2603_nokosahen/scripts/dds_wired_env.sh" 2>/dev/null
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
timeout 8 ros2 topic pub -t 3 -r 10 /nokosahen/soft_estop std_msgs/msg/Bool '{data: true}' >/dev/null 2>&1 \
  && log "  ソフト緊急停止を再発行" || log "  (再発行できず。管理画面側で発行済みのはず)"
timeout 8 ros2 topic pub -t 5 -r 10 /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 \
  && log "  ゼロ速度を再発行" || log "  (ゼロ速度を再発行できず)"

log "== 1) 監視ループ(自動再起動)を止める"
for p in $(pgrep -f 'run_persistent_base_superv[i]sed'); do kill "$p" 2>/dev/null && log "  killed supervisor $p"; done
sleep 1
log "  監視ループの残り: $(pgrep -fc 'run_persistent_base_superv[i]sed')"

log "== 2) ロボット(ROS一式)を止める(管理画面は残す)"
# 注意: `コマンド | tail` だと終了コードがtailのものになり失敗を見逃す → 出力を変数に受けて終了コードを見る
out=$(bash "$HOME/mirs2603_nokosahen/scripts/stop_robot.sh" --keep-dashboard 2>&1); rc=$?
echo "$out" | tail -n 3
[ "$rc" = "0" ] || log "  (stop_robot.sh が失敗 rc=$rc。ロボット側が止まっていない可能性。下の確認を見ること)"

if [ "$OCR" = "1" ]; then
  log "== 3) タブレットOCRとCoralサーバーを止める"
  for p in $(pgrep -f 'tablet_ocr_read[e]r.py'); do kill "$p" 2>/dev/null && log "  killed OCR $p"; done
  out=$(timeout 30 docker stop nokosahen-coral-server 2>&1); rc=$?
  echo "$out" | tail -n 1
  [ "$rc" = "0" ] || log "  (Coralサーバーを止められず rc=$rc)"
else
  log "== 3) OCR/Coralは止めない(--no-ocr)"
fi

if [ "$VOICE" = "1" ]; then
  log "== 4) 2台目の音声(voice_bridge/BGM)を止める"
  out=$(timeout 40 ssh -o ConnectTimeout=6 -o BatchMode=yes j2 'bash ~/stop_voice_stack.sh' 2>&1); rc=$?
  echo "$out" | tail -n 4
  [ "$rc" = "0" ] || log "  (2台目の音声を止められず rc=$rc。2台目に届かない、または停止に失敗)"
else
  log "== 4) 2台目の音声は止めない"
fi

# OSの停止(--poweroff)。ラズパイ・2台目は、ここで「停止の予約」だけして(2秒後にshutdown)sshを正常に閉じる。
# sudoersの許可が無い機器は、予約せずに理由をログへ出す(sudo -n -l で許可の有無を先に確かめる)。
SHUTDOWN_CMD="/usr/sbin/shutdown -h now"
poweroff_remote() {  # poweroff_remote <ssh先> <表示名>
  local out rc
  out=$(timeout 25 ssh -o ConnectTimeout=6 -o BatchMode=yes "$1" "sudo -n -l $SHUTDOWN_CMD >/dev/null 2>&1 && { nohup sh -c 'sleep 2; sudo -n $SHUTDOWN_CMD' >/dev/null 2>&1 & echo SCHEDULED; } || echo NOSUDO" 2>&1); rc=$?
  case "$out" in
    *SCHEDULED*) log "  $2: OSの停止を予約しました" ;;
    *NOSUDO*)    log "  $2: shutdownのsudo許可が無いため、電源を落とせません(install_autostart.sh を実行してください)" ;;
    *)           log "  $2: 届かない、または失敗(rc=$rc)。電源は落としていません" ;;
  esac
}
if [ "$POWEROFF" = "1" ]; then
  log "== 4b) ラズパイ・2台目のOSを停止して電源を落とす(--poweroff)"
  poweroff_remote pi "ラズパイ"
  poweroff_remote j2 "2台目"
fi

log "== 確認(管理画面を止める前)"
log "  ROSノード数: $(timeout 12 ros2 node list 2>/dev/null | wc -l) (管理画面のぶん1〜2が残るのは正常)"
log "  OCR: $(pgrep -fc 'tablet_ocr_read[e]r.py')  監視ループ: $(pgrep -fc 'run_persistent_base_superv[i]sed')"

log "== 5) 管理画面自身(persistent_base launch / full_dashboard)を最後に止める"
for round in 1 2 3; do
  pkill -f 'persistent_base[.]launch[.]py' 2>/dev/null
  pkill -f 'full_dashboar[d]' 2>/dev/null
  pkill -f 'stage2_dashboar[d]' 2>/dev/null
  sleep 2
  [ "$(pgrep -fc 'full_dashboar[d]')" = "0" ] && break
done
pkill -9 -f 'full_dashboar[d]' 2>/dev/null
sleep 1
log "  管理画面の残り: $(pgrep -fc 'full_dashboar[d]')  8090待受: $(ss -ltn 2>/dev/null | grep -c ':8090 ') (0なら停止完了)"
# 意図した停止なので、ソフト緊急停止の保存状態を消す(この停止の途中で緊急停止をかけたので、残すと、次の起動が「停止中」で始まる)。
# 監視ループの自動再起動(意図しない停止)では、この保存状態が残り、停止中のまま起動する(管理画面 TeleopController)。
rm -f "$HOME/.nokosahen_soft_estop.json" "$HOME/.nokosahen_soft_estop.json.tmp"
if [ "$POWEROFF" = "1" ]; then
  log "== 6) 1台目のOSを停止して電源を落とす(--poweroff)。再起動は、一式の電源を入れ直す(自動で手動モードが起動)"
  if sudo -n -l $SHUTDOWN_CMD >/dev/null 2>&1; then
    sync
    sleep 3   # ラズパイ・2台目の停止が始まるのを待つ
    log "  shutdown -h now を実行"
    sudo -n $SHUTDOWN_CMD
  else
    log "  shutdownのsudo許可が無いため、1台目の電源は落としません(install_autostart.sh を実行してください)。プログラムの停止だけで完了"
  fi
else
  log "== shutdown_all.sh 完了。再起動は、一式の電源を入れ直す(自動起動の登録済みの場合)、または 1台目にSSHして: bash ~/mirs2603_nokosahen/scripts/start_all.sh manual"
fi
