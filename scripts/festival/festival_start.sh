#!/usr/bin/env bash
# 文化祭(高専祭、2026-11-07・08)用の起動スクリプト(2026-11-06追加)。1台目Jetsonで実行する。docs は scripts/festival/README.md
#
#   bash ~/mirs2603_nokosahen/scripts/festival/festival_start.sh [オプション]
#
# やること(順番):
#   1) (--boot でないとき)動作中のロボットを止める(stop_everything.sh)。--no-stop で省略
#   2) アンケート回答の1回きりの退避(--reset-survey-once のときだけ。~/survey_data の回答を日付つきの退避先へ「移動」して空で始める。
#      マーカー ~/.nokosahen_festival_survey_reset_done があれば2回目以降は何もしない)
#   3) 出店紹介の設定を置く(~/.nokosahen_pr_intro.json。出店の説明 -> ロボットの説明を、再生が終わってから30秒あけて交互に繰り返す)
#      announce_node がこのファイルを読んで、2台目の音声(voice_bridge)へプレイリストを送る(2台目への直接の送信は不要)
#   4) 2台目のBGMをOFFに保つ見張りを裏で起動(festival_voice_keeper.sh)
#   5) ロボット本体を起動: 自律行動(behavior_director)あり。ただし近づく・見回す・向く・揺動のスイッチは、すべてOFFのまま
#      (「人の方を向く」は、管理画面で人がONにする。ここでは自動でONにしない)。アンケート監視ON、人物検出ON、後ろカメラは既定OFF
#
# オプション:
#   --boot                   電源投入時(boot_start.sh)から呼ぶ。全停止をせず、監視ループをフォアグラウンドで動かす(systemdが監視する)。
#                            ~/.nokosahen_festival_mode の各行(下の「フラグファイル」)を、オプションとして読む
#   --reset-survey-once      アンケートを1回だけ退避してリセットする(マーカーがあれば何もしない)
#   --interval <秒>          出店紹介の間隔(パート間・一巡後とも)。既定 30。10〜600
#   --rear-cameras / --no-rear-cameras      後ろ向きカメラ2台の人物検出(既定: OFF。カメラ未搭載の構成のため)
#   --person-detector / --no-person-detector 人物検出(既定: ON)
#   --dds-wired / --no-dds-wired            ROS通信を有線NICに限定する(既定: 限定する。boot_start.sh と同じ。ハブにつながっていること)
#   --light-localization     軽量の位置推定モード(2026-11-06。地図の読み込み時に Nav2の走行系・nav2_goal_router・wall_escape・
#                            person_clear_gate を起動しない。走行は使えない。docs/light_localization.md)。既定は使わない
#   --no-light-localization  上を使わない(既定。NOKOSAHEN_LOCALIZATION_LIGHT は設定しない)
#   --no-pr-intro            出店紹介の設定を置かない(今の ~/.nokosahen_pr_intro.json をそのまま使う)
#   --no-bgm-off             BGMの見張りを起動しない
#   --no-pid-gains           ESP32のPIDゲインの見張り(pid_gains_keeper.sh。調整済みの値に保つ)を起動しない
#   --no-stop                起動前の全停止をしない
#   --prepare-only           2)と3)だけ行い、ロボットも見張りも起動しない(確認用)
#   --dry-run                何も変えず、やることだけを表示する
#   -h, --help               この説明を表示する
#
# フラグファイル(~/.nokosahen_festival_mode、--boot のとき): 1行に1語。# 以降はコメント。
#   reset_survey_once / rear_cameras / no_person_detector / no_dds_wired / light_localization / interval=45
#
# 注意: このスクリプトのコマンド行に、stop_robot.sh の停止パターン(full_dashboard・nav2_ など)を入れないこと。
#       リモートの pkill -f / pgrep -f は自分のコマンド文字列を巻き込む(パターンは 'recei[v]er' のように割る)。
set -u

REPO="${REPO:-$HOME/mirs2603_nokosahen}"
SCRIPTS="$REPO/scripts"
FEST="$SCRIPTS/festival"
FLAG_FILE="${NOKOSAHEN_FESTIVAL_FLAG:-$HOME/.nokosahen_festival_mode}"
MARKER="$HOME/.nokosahen_festival_survey_reset_done"
PR_DST="${NOKOSAHEN_PR_INTRO_FILE:-$HOME/.nokosahen_pr_intro.json}"
KEEPER_PID="$HOME/.nokosahen_festival_keeper.pid"
LOG="$HOME/.nokosahen_festival.log"

usage() { sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'; }

BOOT=0; DRY=0; RESET=0; INTERVAL=30; REAR=0; PERSON=1; DDS=1; PR=1; BGM=1; STOP_FIRST=1; PREPARE_ONLY=0; LIGHT=0; PIDG=1
PIDG_PID="$HOME/.nokosahen_pid_gains_keeper.pid"

# オプションとして解釈する(コマンド行とフラグファイルで共通)
apply_opt() {
  case "$1" in
    --reset-survey-once|reset_survey_once) RESET=1 ;;
    --rear-cameras|rear_cameras) REAR=1 ;;
    --no-rear-cameras|no_rear_cameras) REAR=0 ;;
    --person-detector) PERSON=1 ;;
    --no-person-detector|no_person_detector) PERSON=0 ;;
    --dds-wired) DDS=1 ;;
    --no-dds-wired|no_dds_wired) DDS=0 ;;
    --light-localization|light_localization) LIGHT=1 ;;
    --no-light-localization|no_light_localization) LIGHT=0 ;;
    --no-pr-intro) PR=0 ;;
    --no-bgm-off) BGM=0 ;;
    --no-pid-gains|no_pid_gains) PIDG=0 ;;
    interval=*) INTERVAL="${1#interval=}" ;;
    *) return 1 ;;
  esac
}

while [ $# -gt 0 ]; do
  case "$1" in
    --boot) BOOT=1 ;;
    --dry-run) DRY=1 ;;
    --no-stop) STOP_FIRST=0 ;;
    --prepare-only) PREPARE_ONLY=1 ;;
    --interval) shift; INTERVAL="${1:-}" ;;
    -h|--help) usage; exit 0 ;;
    *) apply_opt "$1" || { echo "不明な引数: $1 (-h で説明)"; exit 2; } ;;
  esac
  shift
done

# --boot: フラグファイルの各行をオプションとして読む(未知の語は警告して無視。起動は止めない)
if [ "$BOOT" = "1" ] && [ -f "$FLAG_FILE" ]; then
  while IFS= read -r line || [ -n "$line" ]; do
    w="${line%%#*}"; w="$(echo "$w" | tr -d '[:space:]')"
    [ -z "$w" ] && continue
    apply_opt "$w" || echo "$(date '+%F %T') フラグファイルの未知の語を無視: $w" >> "$LOG"
  done < "$FLAG_FILE"
fi

case "$INTERVAL" in ''|*[!0-9]*) echo "--interval は秒数(整数)"; exit 2 ;; esac
if [ "$INTERVAL" -lt 10 ] || [ "$INTERVAL" -gt 600 ]; then echo "--interval は 10〜600 秒"; exit 2; fi

say() { echo "[$(date +%T)] $*"; [ "$DRY" = "1" ] || echo "$(date '+%F %T') $*" >> "$LOG"; }
BOOLS() { [ "$1" = "1" ] && echo true || echo false; }
LAUNCH_ARGS="use_full_dashboard:=true use_stage2_dashboard:=false use_survey_monitor:=true use_person_target_detector:=$(BOOLS "$PERSON") use_rear_cameras:=$(BOOLS "$REAR")"
SURVEY_DIR="${SURVEY_DATA_DIR:-$HOME/survey_data}"

say "== 文化祭モード 起動設定 =="
say "起動方法=$([ "$BOOT" = 1 ] && echo '電源投入(--boot)' || echo '手動')  自律行動(behavior_director)=あり  近づく/見回す/向く/揺動=すべてOFF(向くは管理画面で手動ON)"
say "アンケート監視=ON  人物検出=$(BOOLS "$PERSON")  後ろカメラ=$(BOOLS "$REAR")  ROS有線限定=$(BOOLS "$DDS")  BGM=OFF(見張り=$(BOOLS "$BGM"))"
say "出店紹介=$(BOOLS "$PR")(間隔 ${INTERVAL}秒: 再生が終わってから、パート間・一巡後とも)  アンケート退避=$([ "$RESET" = 1 ] && echo "1回だけ($MARKER が無ければ)" || echo しない)"
say "軽量の位置推定モード=$(BOOLS "$LIGHT")(true なら地図の読み込み時に Nav2の走行系を起動しない。走行は使えない)"
say "launch引数: $LAUNCH_ARGS"

# ---- 起動前の注意(読み取りだけ) ----
# 木・ルールのファイルがあると、スイッチがOFFでも「近づく/見回す」を許可することがある(docs/behavior_tree.md)。
for f in "$HOME/.nokosahen_behavior_tree.json" "$HOME/.nokosahen_behavior_rules.json"; do
  if [ -f "$f" ]; then
    say "注意: $f があります。中身が approach/scan/wiggle を許可していると、スイッチOFFでも動くことがあります(README参照)"
  fi
done

# ---- 1) 全停止 ----
if [ "$BOOT" != "1" ] && [ "$STOP_FIRST" = "1" ] && [ "$PREPARE_ONLY" != "1" ]; then
  say "1) 動作中のロボットとOCRを止める(stop_everything.sh)"
  if [ "$DRY" = "1" ]; then echo "  (dry-run) bash $SCRIPTS/stop_everything.sh"
  elif ! out=$(bash "$SCRIPTS/stop_everything.sh" 2>&1); then
    echo "$out" | tail -n 6; say "停止に失敗した手順があるため中止します(それでも進めるなら --no-stop)"; exit 4
  else echo "$out" | tail -n 3; fi
fi

# ---- 2) アンケートの1回きりの退避 ----
reset_survey_once() {
  if [ -e "$MARKER" ]; then
    say "2) アンケート: すでに退避済み($(head -c 200 "$MARKER" | tr '\n' ' '))。何もしません"; return 0
  fi
  local ts bk
  ts="$(date +%Y%m%d_%H%M%S)"; bk="$HOME/survey_data_backup_before_festival_$ts"
  if [ "$DRY" = "1" ]; then
    echo "  (dry-run) $SURVEY_DIR/responses.* を $bk/ へ移動(condition.txt はコピーして元に残す)し、マーカー $MARKER を書く"; return 0
  fi
  # 回答保存は full_dashboard が持っている(メモリに件数の索引がある)。動作中に動かすと食い違うので、動いていたら中止する
  if curl -s -m 2 -o /dev/null "http://127.0.0.1:8090/" 2>/dev/null; then
    say "2) アンケート: 管理画面(8090)が動作中のため退避しません(止めてから。マーカーは書きません)"; return 1
  fi
  local moved=0 f
  if [ -d "$SURVEY_DIR" ]; then
    for f in "$SURVEY_DIR"/responses.*; do
      [ -e "$f" ] || continue
      [ "$moved" = "0" ] && mkdir -p "$bk"
      if mv -- "$f" "$bk/"; then moved=$((moved + 1)); else say "2) アンケート: $f を移動できません。中止します(マーカーは書きません)"; return 1; fi
    done
    if [ -f "$SURVEY_DIR/condition.txt" ] && [ -d "$bk" ]; then cp -p "$SURVEY_DIR/condition.txt" "$bk/"; fi
  fi
  if [ "$moved" = "0" ]; then
    printf '%s 退避する回答はありませんでした\n' "$ts" > "$MARKER"
    say "2) アンケート: 退避する回答が無かったため、空のまま始めます(マーカーを書きました)"
  else
    printf '%s 退避先=%s 移動=%s件(responses.*)\n' "$ts" "$bk" "$moved" > "$MARKER"
    say "2) アンケート: 回答を $bk へ移動しました(${moved}ファイル)。空の状態で始めます。condition.txt(実験条件)は元の場所に残しています"
  fi
  return 0
}
if [ "$RESET" = "1" ]; then reset_survey_once || true; else say "2) アンケート: 退避しない(--reset-survey-once なし。回答はそのまま残る)"; fi

# ---- 3) 出店紹介の設定 ----
install_pr_intro() {
  local src="$FEST/pr_intro.festival.json" ts tmp
  if [ ! -f "$src" ]; then say "3) 出店紹介: $src が無いため置けません"; return 1; fi
  ts="$(date +%Y%m%d_%H%M%S)"
  if [ "$DRY" = "1" ]; then
    echo "  (dry-run) $src の interval_sec / gap_sec を ${INTERVAL} にして $PR_DST へ置く(今のファイルが違う内容なら ${PR_DST}.bak_before_festival_$ts へ退避)"; return 0
  fi
  tmp="$PR_DST.festival_tmp.$$"
  sed -e "s/\"interval_sec\": *[0-9.]*/\"interval_sec\": $INTERVAL/" -e "s/\"gap_sec\": *[0-9.]*/\"gap_sec\": $INTERVAL/" "$src" > "$tmp" || { rm -f "$tmp"; return 1; }
  if [ -f "$PR_DST" ] && cmp -s "$tmp" "$PR_DST"; then
    rm -f "$tmp"; say "3) 出店紹介: すでに文化祭の設定です(変更なし)"; return 0
  fi
  [ -f "$PR_DST" ] && cp -p "$PR_DST" "$PR_DST.bak_before_festival_$ts"
  mv -f "$tmp" "$PR_DST" && say "3) 出店紹介: $PR_DST を文化祭の設定にしました(出店の説明 -> ロボットの説明、${INTERVAL}秒おき)。前のファイルは .bak_before_festival_$ts に退避"
}
if [ "$PR" = "1" ]; then install_pr_intro || say "3) 出店紹介の設定に失敗(続行します)"; else say "3) 出店紹介: 設定を置かない(--no-pr-intro)"; fi

if [ "$PREPARE_ONLY" = "1" ]; then say "--prepare-only: ここまでで終了します"; exit 0; fi

# ---- 4) BGMの見張り ----
start_keeper() {
  if [ "$DRY" = "1" ]; then echo "  (dry-run) 古い見張りがあれば止め、setsid nohup bash $FEST/festival_voice_keeper.sh を起動(PID: $KEEPER_PID)"; return 0; fi
  if [ -f "$KEEPER_PID" ]; then
    local old; old="$(cat "$KEEPER_PID" 2>/dev/null)"
    # PIDが再利用されていても別プロセスを止めないよう、コマンド行を確かめる
    if [ -n "$old" ] && tr '\0' ' ' < "/proc/$old/cmdline" 2>/dev/null | grep -q 'festival_voice_keepe[r]'; then kill "$old" 2>/dev/null; fi
    rm -f "$KEEPER_PID"
  fi
  setsid nohup bash "$FEST/festival_voice_keeper.sh" >/dev/null 2>&1 < /dev/null &
  echo $! > "$KEEPER_PID"
  say "4) BGMの見張りを起動しました(pid $(cat "$KEEPER_PID")。2台目のBGMをOFFに保つ。ログ ~/.nokosahen_festival_keeper.log)"
}
if [ "$BGM" = "1" ]; then start_keeper; else say "4) BGMの見張り: 起動しない(--no-bgm-off)"; fi

# ---- 4b) ESP32のPIDゲインの見張り(2026-11-06): ゲインはESP32のRAMにだけあり再起動で既定に戻るので、文化祭用の値に保つ ----
# その場旋回で前後にずれる癖(左回り+4.1/右回り-2.5 cm/rad)を、左輪kpと後退側kpの調整で抑えた値。理由・元に戻し方は pid_gains_keeper.py の先頭。
start_pid_keeper() {
  if [ "$DRY" = "1" ]; then echo "  (dry-run) 古い見張りがあれば止め、setsid nohup bash $FEST/pid_gains_keeper.sh を起動(PID: $PIDG_PID)"; return 0; fi
  if [ -f "$PIDG_PID" ]; then
    local old; old="$(cat "$PIDG_PID" 2>/dev/null)"
    if [ -n "$old" ] && tr '\0' ' ' < "/proc/$old/cmdline" 2>/dev/null | grep -q 'pid_gains_keepe[r]'; then kill "$old" 2>/dev/null; fi
    rm -f "$PIDG_PID"
  fi
  setsid nohup bash "$FEST/pid_gains_keeper.sh" >/dev/null 2>&1 < /dev/null &
  echo $! > "$PIDG_PID"
  say "4b) PIDゲインの見張りを起動しました(pid $(cat "$PIDG_PID")。ESP32のゲインを調整済みの値に保つ。ログ ~/.nokosahen_pid_gains_keeper.log。やめるなら --no-pid-gains)"
}
if [ "$PIDG" = "1" ]; then start_pid_keeper; else say "4b) PIDゲインの見張り: 起動しない(--no-pid-gains)"; fi

# ---- 5) ロボット本体 ----
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
if [ "$DDS" != "1" ]; then export NOKOSAHEN_DDS_WIRED=0; unset FASTRTPS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_USE_QOS_FROM_XML; else unset NOKOSAHEN_DDS_WIRED; fi
export EXTRA_LAUNCH_ARGS="$LAUNCH_ARGS"
# 軽量の位置推定モード: 管理画面の「この地図を読み込む」(MapLifecycleController)が環境変数を見て localization_mode.launch.py に light:=true を足す
if [ "$LIGHT" = "1" ]; then export NOKOSAHEN_LOCALIZATION_LIGHT=1; else
  unset NOKOSAHEN_LOCALIZATION_LIGHT
  [ -e "$HOME/.nokosahen_light_localization" ] && say "注意: $HOME/.nokosahen_light_localization があるため、地図の読み込みは軽量モードになります(使わないなら消す)"
fi
# 監視ループの引数は空 = 自律行動(behavior_director)あり。ただし director の動くスイッチは既定すべてOFF(behavior_director.py)
if [ "$BOOT" = "1" ]; then
  say "5) 監視ループを起動(フォアグラウンド。自律行動あり・スイッチはすべてOFF)"
  if [ "$DRY" = "1" ]; then echo "  (dry-run) EXTRA_LAUNCH_ARGS=\"$LAUNCH_ARGS\" exec bash $SCRIPTS/run_persistent_base_supervised.sh"; exit 0; fi
  exec bash "$SCRIPTS/run_persistent_base_supervised.sh" >> "$HOME/.nokosahen_supervisor_wrapper.log" 2>&1
fi
say "5) ロボット本体を起動(監視ループ付き。自律行動あり・スイッチはすべてOFF)"
if [ "$DRY" = "1" ]; then
  echo "  (dry-run) NOKOSAHEN_DDS_WIRED=$([ "$DDS" = 1 ] && echo 既定 || echo 0) EXTRA_LAUNCH_ARGS=\"$LAUNCH_ARGS\" setsid nohup bash $SCRIPTS/run_persistent_base_supervised.sh"
  say "(dry-run のため起動しません)"; exit 0
fi
setsid nohup bash "$SCRIPTS/run_persistent_base_supervised.sh" >> "$HOME/.nokosahen_supervisor_wrapper.log" 2>&1 < /dev/null &
say "起動しました。約2分後に管理画面 http://192.0.2.1:8090/ へ。「人の方を向く」だけ手動でONにする。全停止: bash $SCRIPTS/stop_everything.sh"
