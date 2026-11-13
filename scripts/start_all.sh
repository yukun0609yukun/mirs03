#!/usr/bin/env bash
# 1台目Jetsonで実行する「全部入り」の起動スクリプト(2026-10-13追加)。1個のコマンドで、状態(モード)を引数で選んで起動する。
#
#   bash ~/mirs2603_nokosahen/scripts/start_all.sh <モード> [オプション]
#
# モード(必須):
#   full      自律行動あり(behavior_directorが動く、ルール/木の動作もON)。2台目の音声も起動。要 --yes(車輪が動く)
#   manual    手動のみ(自律行動なし)。Nav2・探索はダッシュボードから人が開始する。2台目の音声も起動
#   nav_test  1台目だけの自立走行テスト(manualと同じ構成。アンケート監視なし、ROSは有線限定なし、2台目に依存しない)
#
# オプション:
#   --ocr / --no-ocr      タブレットOCRとCoral(Docker)を起動する/しない(既定: full・manualは起動、nav_testはしない)。
#                         設定ファイル(~/.nokosahen_tablet_ocr_config.json)に認証パスワードがあれば、読み取り値を残数入力へ送る
#   --voice / --no-voice  2台目の音声(voice_bridge/BGM)を起動する/しない(既定: full・manualは起動、nav_testはしない)
#   --display <customer|survey|ocrtest>  ラズパイの腹部画面のモードを切り替える(省略時は触らない)
#   --no-clock            2台目・ラズパイの時計合わせをしない(既定は合わせる)
#   --no-stop             起動前の全停止をしない(既定は、動作中のロボットを先に止める)
#   --survey-monitor / --no-survey-monitor  アンケート回答中の停止の監視(既定: full・manualはON、nav_testはOFF)
#   --dds-wired / --no-dds-wired  ROS通信を有線NICに限定する/しない(既定: full・manualは限定、nav_testはしない)
#   --person-detector / --no-person-detector  人物検出(CPUを約半コア使う)を起動する/しない(既定: full・manualは起動、nav_testはしない)
#   --rear-cameras / --no-rear-cameras  後ろ向きカメラ2台(左後ろ・右後ろ)の人物検出を起動する/しない(既定: 人物検出と同じ。カメラが無い構成は --no-rear-cameras)
#   --wait <秒>           起動後の確認までの待ち時間(既定 75)
#   --yes                 確認を省略する(fullで必須。SSH経由など対話できないとき)
#   --dry-run             実行せず、何をするかだけ表示する
#   -h, --help            この説明を表示する
#
# 例:
#   bash ~/mirs2603_nokosahen/scripts/start_all.sh manual
#   bash ~/mirs2603_nokosahen/scripts/start_all.sh full --yes --ocr
#   bash ~/mirs2603_nokosahen/scripts/start_all.sh nav_test --dry-run
# 全停止は: bash ~/mirs2603_nokosahen/scripts/stop_everything.sh [--all]
# 詳細は docs/startup.md。
# 注意: リモートの pkill -f / pgrep -f は自分のコマンド文字列を巻き込む。パターンは 'recei[v]er' のように割ること。
set -u

REPO="${REPO:-$HOME/mirs2603_nokosahen}"
SCRIPTS="$REPO/scripts"

usage() { sed -n '2,33p' "$0" | sed 's/^# \{0,1\}//'; }

MODE=""
OCR=""; VOICE=""; DISPLAY_MODE=""; CLOCK=1; STOP_FIRST=1; SURVEY=""; DDS=""; PERSON=""; REAR=""; WAIT=75; YES=0; DRY=0

while [ $# -gt 0 ]; do
  case "$1" in
    full|manual|nav_test) [ -z "$MODE" ] && MODE="$1" || { echo "モードは1つだけ指定してください"; exit 2; } ;;
    --ocr) OCR=1 ;;
    --no-ocr) OCR=0 ;;
    --voice) VOICE=1 ;;
    --no-voice) VOICE=0 ;;
    --display) shift; DISPLAY_MODE="${1:-}"; case "$DISPLAY_MODE" in customer|survey|ocrtest) ;; *) echo "--display は customer|survey|ocrtest"; exit 2 ;; esac ;;
    --no-clock) CLOCK=0 ;;
    --no-stop) STOP_FIRST=0 ;;
    --survey-monitor) SURVEY=1 ;;
    --no-survey-monitor) SURVEY=0 ;;
    --dds-wired) DDS=1 ;;
    --no-dds-wired) DDS=0 ;;
    --person-detector) PERSON=1 ;;
    --no-person-detector) PERSON=0 ;;
    --rear-cameras) REAR=1 ;;
    --no-rear-cameras) REAR=0 ;;
    --wait) shift; WAIT="${1:-75}"; case "$WAIT" in ''|*[!0-9]*) echo "--wait は秒数(整数)"; exit 2 ;; esac ;;
    --yes) YES=1 ;;
    --dry-run) DRY=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "不明な引数: $1 (-h で説明)"; exit 2 ;;
  esac
  shift
done
[ -z "$MODE" ] && { echo "モードを指定してください: full | manual | nav_test (-h で説明)"; exit 2; }

# --- モードごとの既定(オプションで上書き) ---
case "$MODE" in
  full)     SUP_ARG="";        : "${OCR:=1}"; : "${VOICE:=1}"; : "${SURVEY:=1}"; : "${DDS:=1}"; : "${PERSON:=1}" ;;
  manual)   SUP_ARG="rc_only"; : "${OCR:=1}"; : "${VOICE:=1}"; : "${SURVEY:=1}"; : "${DDS:=1}"; : "${PERSON:=1}" ;;
  # nav_test: 人物検出(person_target_detector)は、自律行動(director)が使うもの。走行テストでは使わず、CPUを約半コア食うので止める(2026-10-19)
  nav_test) SUP_ARG="rc_only"; : "${OCR:=0}"; : "${VOICE:=0}"; : "${SURVEY:=0}"; : "${DDS:=0}"; : "${PERSON:=0}" ;;
esac

if [ "$MODE" = "full" ] && [ "$YES" != "1" ] && [ "$DRY" != "1" ]; then
  echo "【確認】full は自律行動あり: ロボットが自分で動き出します。周囲を空け、電源を切れる状態にしてください。"
  echo "         (物理の緊急停止は未配線です)。続けるには --yes を付けて再実行してください。"
  exit 3
fi

BOOL() { [ "$1" = "1" ] && echo true || echo false; }
LAUNCH_ARGS="use_full_dashboard:=true use_stage2_dashboard:=false use_survey_monitor:=$(BOOL "$SURVEY") use_person_target_detector:=$(BOOL "$PERSON") use_rear_cameras:=$(BOOL "${REAR:-$PERSON}")"

say() { echo "[$(date +%T)] $*"; }
run() { if [ "$DRY" = "1" ]; then echo "  (dry-run) $*"; else "$@"; fi; }
# 出力の末尾だけ表示しつつ、コマンドの終了コードを返す(`cmd | tail` だとtailの終了コードになり失敗を見逃すため)
SSH_OPTS="-o BatchMode=yes -o ConnectTimeout=6"   # 鍵が通らないときにパスワード待ちで止まらない
run_tail() { # run_tail <行数> <コマンド...>
  local n="$1"; shift
  if [ "$DRY" = "1" ]; then echo "  (dry-run) $*"; return 0; fi
  local out rc
  out=$("$@" 2>&1); rc=$?
  echo "$out" | tail -n "$n"
  return $rc
}

say "== 起動設定 =="
say "モード=$MODE  監視ループ引数='${SUP_ARG}'  音声(2台目)=$VOICE  OCR=$OCR  アンケート監視=$SURVEY  人物検出=$PERSON  ROS有線限定=$DDS  腹部画面=${DISPLAY_MODE:-そのまま}  時計合わせ=$CLOCK  先に全停止=$STOP_FIRST"
say "launch引数: $LAUNCH_ARGS"

# 1) 時計(2台目・ラズパイを1台目に合わせる。失敗しても続行)
if [ "$CLOCK" = "1" ]; then
  say "1) 時計合わせ"
  if [ -f "$SCRIPTS/sync_clocks.sh" ]; then run_tail 4 timeout 60 bash "$SCRIPTS/sync_clocks.sh" || say "  時計合わせに失敗(続行します。2台目・ラズパイの時計がずれている可能性)"; else say "  sync_clocks.sh が無いのでスキップ"; fi
fi

# 2) 動作中のものを止める。失敗したまま起動すると、古いプロセスが残って二重起動になるので、ここで中止する
if [ "$STOP_FIRST" = "1" ]; then
  say "2) 動作中のロボットとOCRを止める(地図を作成中で未保存なら、先に保存してから実行すること)"
  if ! run_tail 6 bash "$SCRIPTS/stop_everything.sh"; then
    say "停止に失敗した手順があるため、起動を中止します。残りの確認: bash $SCRIPTS/stop_everything.sh   (それでも進めるなら --no-stop)"
    exit 4
  fi
fi

# 3) ラズパイの腹部画面
if [ -n "$DISPLAY_MODE" ]; then
  say "3) 腹部画面を $DISPLAY_MODE に切り替え"
  run_tail 2 timeout 30 ssh $SSH_OPTS pi "/home/pi/switch_kiosk.sh $DISPLAY_MODE" || say "  腹部画面を切り替えられません(続行)"
fi

# 4) 2台目の音声(voice_bridge/BGM)。2台目を先に上げる
if [ "$VOICE" = "1" ]; then
  say "4) 2台目の音声を起動(Ollamaのウォームアップは裏で約2分)"
  run_tail 5 timeout 90 ssh $SSH_OPTS j2 'bash ~/start_voice_stack.sh' || say "  2台目の音声を起動できません(音声なしで続行)"
fi

# 5) タブレットOCR + Coral
if [ "$OCR" = "1" ]; then
  say "5) Coral(Docker)とタブレットOCRを起動(設定ファイルに認証パスワードがあれば、読み取り値を残数入力へ送る)"
  run_tail 1 timeout 30 docker start nokosahen-coral-server || say "  Coralサーバーを起動できません(OCRはtesseractに落ちて失敗します)"
  if [ "$DRY" != "1" ]; then sleep 8; fi
  if [ "$DRY" = "1" ]; then
    echo "  (dry-run) cd $SCRIPTS/tablet_ocr_reader && setsid nohup python3 tablet_ocr_reader.py --config ~/.nokosahen_tablet_ocr_config.json ..."
  else
    ( cd "$SCRIPTS/tablet_ocr_reader" && env -u NOKOSAHEN_TABLET_OCR_PASSWORD setsid nohup python3 tablet_ocr_reader.py \
        --config "$HOME/.nokosahen_tablet_ocr_config.json" --verbose > "$HOME/tablet_ocr.log" 2>&1 < /dev/null & )
  fi
fi

# 6) ロボット本体(監視ループ付き)を起動
say "6) ロボット本体を起動($MODE)"
if [ "$DRY" = "1" ]; then
  echo "  (dry-run) NOKOSAHEN_DDS_WIRED=$([ "$DDS" = "1" ] && echo 1 || echo 0) EXTRA_LAUNCH_ARGS=\"$LAUNCH_ARGS\" setsid nohup bash $SCRIPTS/run_persistent_base_supervised.sh $SUP_ARG"
else
  set +u
  export ROS_DOMAIN_ID=77
  source /opt/ros/jazzy/setup.bash
  source "$HOME/lidar_src_build/install/setup.bash"
  source "$HOME/microros_ws/install/setup.bash"
  source "$REPO/ros2_ws/install/setup.bash"
  set -u
  if [ "$DDS" = "1" ]; then
    unset NOKOSAHEN_DDS_WIRED   # 既定(有線NICに限定)
  else
    export NOKOSAHEN_DDS_WIRED=0
    unset FASTRTPS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_USE_QOS_FROM_XML
  fi
  EXTRA_LAUNCH_ARGS="$LAUNCH_ARGS" setsid nohup bash "$SCRIPTS/run_persistent_base_supervised.sh" $SUP_ARG \
    >> "$HOME/.nokosahen_supervisor_wrapper.log" 2>&1 < /dev/null &
fi

# 7) 確認
if [ "$DRY" = "1" ]; then say "(dry-run のため確認は省略)"; exit 0; fi
say "7) ${WAIT}秒待ってから確認"
sleep "$WAIT"
set +u
echo "load: $(cut -d' ' -f1-3 /proc/loadavg)   時計: $(date '+%F %T')"
echo "ポート: $(for p in 8080 8090 8095; do printf "$p=%s " "$(curl -s -m5 -o /dev/null -w '%{http_code}' http://127.0.0.1:$p/)"; done)"
echo "scan: $(timeout 15 ros2 topic hz /scan 2>&1 | grep -m1 'average rate')   odom: $(timeout 12 ros2 topic hz /odom 2>&1 | grep -m1 'average rate')"
N=$(timeout 15 ros2 node list 2>/dev/null)
echo "behavior_director: $(echo "$N" | grep -c behavior_director) (full=1 / manual・nav_test=0)   survey_monitor: $(echo "$N" | grep -c survey_monitor) (ON=1 / OFF=0)"
echo "cmd_vel(5秒):"; timeout 5 ros2 topic echo /cmd_vel --field linear.x 2>/dev/null | sort -u | head -3
if [ "$VOICE" = "1" ]; then echo "音声(2台目): $(curl -s -m4 http://198.51.100.3:8100/status | head -c 90)"; fi
if [ "$OCR" = "1" ]; then echo "OCR確認画面(8091): $(curl -s -m4 -o /dev/null -w '%{http_code}' http://127.0.0.1:8091/)"; fi
echo "Traceback: $(grep -a -c Traceback "$HOME/.nokosahen_persistent_base.log" 2>/dev/null)"
echo "起動しました。ダッシュボード: http://192.0.2.1:8090/ (Nav2は「地図・走行」タブから開始)   全停止: bash $SCRIPTS/stop_everything.sh [--all]"
