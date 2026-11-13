#!/usr/bin/env bash
# PC(WSL)から1台目Jetsonへ、コードの配置 -> 時計合わせ -> ビルド -> (再起動) -> 確認 を1コマンドで行う(2026-10-24追加)。
# ロボット(1台目)へは、PCがWi-Fi「nokosahen」につながっている状態で実行する。詳細: docs/deploy_jetson1.md
#
#   bash scripts/pc/deploy_jetson1.sh [オプション]
#   wsl bash /mnt/c/Users/user/Documents/mirs2603_nokosahen/scripts/pc/deploy_jetson1.sh --dry-run     (PowerShellから)
#
# オプション:
#   --restart manual   配置とビルドのあと、manualモードで再起動する(既定)。start_all.sh manual --no-dds-wired --no-voice --no-clock --no-survey-monitor
#   --no-restart       配置とビルドだけ(再起動も起動後の確認もしない)
#   --cameras          再起動のとき、人物検出と後ろ向きカメラも起動する(既定はカメラなし)
#   --verify           --no-restart でも、動作中のロボットの確認(読み取りだけ)をする
#   --yes              「地図は消えます」の確認を省略する
#   --dry-run          1台目へつながず、やることだけを表示する
#   --host <user@host> 接続先(既定 robot@192.0.2.1 / 環境変数 NOKOSAHEN_JETSON1)
#   --key <ファイル>   SSH鍵(既定 ~/.ssh/nokosahen_jetson / 環境変数 NOKOSAHEN_SSH_KEY)
#   -h, --help         この説明を表示する
#
# 終了コード: 0=成功(警告は含む) 1=起動後の確認にNGあり 2=1台目に届かない 3=ビルド失敗
#             4=再起動に失敗(起動ログに「中止」/待ち時間切れ) 5=確認で中止 6=配置(rsync)に失敗 64=引数が不正
# 注意(2026-11-05): rsync に --delete は付けていない(1台目の地図・設定・ログなどを誤って消さないため)。
#       PCで消した/名前を変えたファイルは、1台目に古いまま残る。必要なら1台目で手動で消すこと(退避先 ~/deploy_backup_<日付> は置き換えたものだけ)。
#       時計合わせは、1台目の時計が2026年より前のときだけ(それ以外のずれは直さない)。地図の状態の確認は認証が要るため、未ログインでは「不明」になる。
# 使うsudoは「sudo -n /usr/bin/date -s」(時計合わせ。sudoers導入済み)だけ。パスワード・秘密は読まない・出さない。
# 注意: リモートの pkill -f / pgrep -f は自分のコマンド文字列を巻き込む。start_all.sh へ渡す文字列に、
#       stop_robot.sh のパターン(full_dashboard / nav2_ など)を含めないこと(test_deploy_script.py が検査)。
set -u

usage() { sed -n '2,26p' "$0" | sed 's/^# \{0,1\}//'; }

SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TARGET="${NOKOSAHEN_JETSON1:-robot@192.0.2.1}"
KEY="${NOKOSAHEN_SSH_KEY:-$HOME/.ssh/nokosahen_jetson}"
REMOTE_DIR_NAME="mirs2603_nokosahen"          # 1台目の ~ の下
PC_LOG_DIR="${NOKOSAHEN_DEPLOY_LOG_DIR:-$HOME/deploy_logs}"
START_WAIT_TOTAL="${DEPLOY_START_WAIT_TOTAL:-240}"   # 起動ログを待つ最大秒数
POLL_SEC="${DEPLOY_POLL_SEC:-6}"                      # 起動ログを見る間隔(秒)

ORIG_ARGS=("$@")   # ログ付きの再実行で、同じ引数を渡す
RESTART=manual; CAMERAS=0; YES=0; DRY=0; VERIFY=""
while [ $# -gt 0 ]; do
  case "$1" in
    --restart) shift; RESTART="${1:-}"; [ "$RESTART" = "manual" ] || { echo "--restart は manual のみ (-h で説明)"; exit 64; } ;;
    --no-restart) RESTART=none ;;
    --cameras) CAMERAS=1 ;;
    --verify) VERIFY=1 ;;
    --yes) YES=1 ;;
    --dry-run) DRY=1 ;;
    --host) shift; TARGET="${1:-}"; [ -n "$TARGET" ] || { echo "--host に接続先が必要"; exit 64; } ;;
    --key) shift; KEY="${1:-}"; [ -n "$KEY" ] || { echo "--key にファイルが必要"; exit 64; } ;;
    -h|--help) usage; exit 0 ;;
    *) echo "不明な引数: $1 (-h で説明)"; exit 64 ;;
  esac
  shift
done
if [ -z "$VERIFY" ]; then if [ "$RESTART" = "none" ]; then VERIFY=0; else VERIFY=1; fi; fi

HOST_ONLY="${TARGET#*@}"
USER_ONLY="${TARGET%%@*}"; [ "$USER_ONLY" = "$TARGET" ] && USER_ONLY="$(id -un)"
TS="${DEPLOY_TS:-$(date +%Y%m%d_%H%M%S)}"
DAY="${TS%%_*}"
REMOTE_HOME="/home/$USER_ONLY"       # 実際の値は事前確認で取得して上書きする

# --- rsyncの除外(秘密・生成物・PC専用のもの) ---
EXCLUDES=(
  .git __pycache__ '*.pyc'
  build/ install/ log/
  '*.local.yaml' '*.local.json' 'secrets.*'
  作業日報.md work/ outputs/ poc_tests/ docs/report/
  'firmware/*/.pio'
  '*.keras' digit_model
  .claude standalone_checkout_cam .agents .codex .vscode
)
# 念のための追加(.gitignoreにある秘密類)。除外一覧の検査(test_deploy_script.py)は、上の一覧と追加分を別に見る
EXCLUDES_EXTRA=(.env '.env.*' '*.pem' '*.key' 'id_rsa*' 'id_ed25519*' 'id_ecdsa*' 'credentials*.json' '.nokosahen_*')

SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3 -o StrictHostKeyChecking=accept-new)
[ -f "$KEY" ] && SSH_OPTS+=(-i "$KEY")

fmt_cmd() {  # 表示用: 空白・記号を含む引数を'...'で囲む
  local a out=""
  for a in "$@"; do
    case "$a" in *[!A-Za-z0-9_./:=@%+,-]*) out+=" '$a'" ;; *) out+=" $a" ;; esac
  done
  printf '%s' "${out# }"
}
rsh() { ssh "${SSH_OPTS[@]}" "$TARGET" "$@" < /dev/null; }
say() { echo "[$(date +%T)] $*"; }

# --- 再起動コマンド(1台目で実行する文字列。stop_robot.shのパターンに一致する語を入れない) ---
START_ARGS="manual --no-dds-wired --no-voice --no-clock --no-survey-monitor"
if [ "$CAMERAS" = "1" ]; then START_ARGS="$START_ARGS --person-detector --rear-cameras"; else START_ARGS="$START_ARGS --no-person-detector --no-rear-cameras"; fi
START_CMD="setsid nohup bash \$HOME/$REMOTE_DIR_NAME/scripts/start_all.sh $START_ARGS --wait 60 > \$HOME/start_all_single.log 2>&1 < /dev/null &"

RSYNC_ARGS=(-rltz --chmod=D755,F644 --itemize-changes --backup)
build_rsync() {
  RSYNC=(rsync "${RSYNC_ARGS[@]}" "--backup-dir=$REMOTE_HOME/deploy_backup_$DAY")
  local e
  for e in "${EXCLUDES[@]}" "${EXCLUDES_EXTRA[@]}"; do RSYNC+=("--exclude=$e"); done
  RSYNC+=(-e "ssh ${SSH_OPTS[*]}" "$REPO_ROOT/" "$TARGET:$REMOTE_DIR_NAME/")
}
BUILD_CMD="colcon build --symlink-install --packages-select nokosahen_core nokosahen_bringup"
BUILD_SRC='source /opt/ros/jazzy/setup.bash; source "$HOME/lidar_src_build/install/setup.bash"; source "$HOME/microros_ws/install/setup.bash"'

# ---------------------------------------------------------------- ドライラン
if [ "$DRY" = "1" ]; then
  build_rsync
  say "== deploy_jetson1 (ドライラン: 1台目へはつなぎません) =="
  echo "接続先: $TARGET   鍵: $KEY   配置元: $REPO_ROOT"
  echo "1) 事前確認: ping $HOST_ONLY と ssh。届かなければ終了コード2"
  echo "2) 配置(rsync)。置き換えるファイルの退避先: $REMOTE_HOME/deploy_backup_$DAY"
  echo "  (dry-run) $(fmt_cmd "${RSYNC[@]}")"
  echo "3) 時計合わせ: 1台目の時計が2026年より前なら、PCの時刻にする"
  echo "  (dry-run) ssh $TARGET sudo -n /usr/bin/date -s @<PCのepoch>"
  echo "4) ビルド(失敗したら中止、終了コード3)"
  echo "  (dry-run) ssh $TARGET '$BUILD_SRC; cd ~/$REMOTE_DIR_NAME/ros2_ws && $BUILD_CMD'"
  if [ "$RESTART" = "manual" ]; then
    echo "5) 再起動(manual、カメラ$([ "$CAMERAS" = 1 ] && echo あり || echo なし))。地図は消えます: $([ "$YES" = 1 ] && echo '確認は省略(--yes)' || echo '実行前に確認します')"
    echo "  (dry-run) 起動: $START_CMD"
    echo "  (dry-run) 起動ログ ~/start_all_single.log を最大${START_WAIT_TOTAL}秒待つ"
  else
    echo "5) 再起動: しない(--no-restart)"
  fi
  if [ "$VERIFY" = "1" ]; then
    echo "6) 起動後の確認(読み取りだけ): ダッシュボード8090、主要プロセス、/scan・/wheel_ticks・/estop_state、負荷、時計、ESP32(/wheel_ticksのHz)"
  else
    echo "6) 起動後の確認: しない"
  fi
  echo "7) ログ: PC=$PC_LOG_DIR/$TS.log  1台目=~/deploy_logs/$TS.log"
  exit 0
fi

# ---------------------------------------------------------------- 実行ログ(PCと1台目)
if [ -z "${DEPLOY_INNER:-}" ]; then
  mkdir -p "$PC_LOG_DIR" 2>/dev/null
  LOG="$PC_LOG_DIR/$TS.log"
  DEPLOY_INNER=1 DEPLOY_TS="$TS" bash "$SELF" "${ORIG_ARGS[@]}" 2>&1 | tee "$LOG"
  rc=${PIPESTATUS[0]}
  if timeout 20 ssh "${SSH_OPTS[@]}" -o ConnectTimeout=5 "$TARGET" "mkdir -p ~/deploy_logs && cat > ~/deploy_logs/$TS.log" < "$LOG" >/dev/null 2>&1; then
    echo "ログ: PC=$LOG  1台目=~/deploy_logs/$TS.log"
  else
    echo "ログ: PC=$LOG (1台目へは保存できませんでした)"
  fi
  exit "$rc"
fi

# ---------------------------------------------------------------- 本体
ROWS=()   # 結果の表: "状態|項目|内容"
row() { ROWS+=("$1|$2|$3"); }

say "== deploy_jetson1 開始 ($TARGET) =="

# 1) 事前確認
say "1) 事前確認"
PING_OK=0
if command -v ping >/dev/null 2>&1; then
  ping -c 1 -W 3 "$HOST_ONLY" >/dev/null 2>&1 && PING_OK=1
else
  PING_OK=1; echo "  (pingコマンドが無いので省略)"
fi
OUT=$(timeout 20 ssh "${SSH_OPTS[@]}" -o ConnectTimeout=6 "$TARGET" 'echo "HOME=$HOME"; command -v rsync >/dev/null && echo RSYNC=1 || echo RSYNC=0' < /dev/null 2>/dev/null)
SRC=$?
OUT=$(printf '%s' "$OUT" | tr -d '\r')
if [ "$SRC" != "0" ]; then
  echo "NG: 1台目($TARGET)に届きません (ping: $([ "$PING_OK" = 1 ] && echo 応答あり || echo 応答なし) / ssh: 接続できず)"
  echo "  - PCのWi-Fiが「nokosahen」か確認してください(1台目は起動後1〜3分でWi-Fiが立ち上がります)。"
  SSID=$(timeout 5 netsh.exe wlan show interfaces 2>/dev/null | tr -d '\r' | grep -E '^\s+SSID\s' | head -1 | sed 's/^ *//')
  [ -n "$SSID" ] && echo "  - いまのPCのWi-Fi: $SSID"
  echo "  - それでもだめなら、1台目の電源・SSH鍵($KEY)を確認してください。"
  exit 2
fi
[ "$PING_OK" = "1" ] || echo "  (注意: pingは応答なし。ICMPを通さない設定ならsshが通れば問題ありません)"
RH=$(printf '%s\n' "$OUT" | sed -n 's/^HOME=//p' | head -1); [ -n "$RH" ] && REMOTE_HOME="$RH"
if [ "$(printf '%s\n' "$OUT" | sed -n 's/^RSYNC=//p' | head -1)" != "1" ]; then
  echo "NG: 1台目に rsync がありません(ここでは何もインストールしません)"; exit 6
fi
command -v rsync >/dev/null 2>&1 || { echo "NG: PC(WSL)に rsync がありません"; exit 6; }
echo "  OK: ssh接続できました (1台目の ~ = $REMOTE_HOME)"

# 2) 配置(rsync)
say "2) 配置(rsync) 置き換えるファイルの退避先: $REMOTE_HOME/deploy_backup_$DAY"
build_rsync
RSYNC_OUT=$(mktemp)
if "${RSYNC[@]}" > "$RSYNC_OUT" 2>&1; then
  N=$(grep -c '^[<>c.][fdLDS]' "$RSYNC_OUT"); NF=$(grep -c '^>f' "$RSYNC_OUT")
  head -n 30 "$RSYNC_OUT" | sed 's/^/  /'
  [ "$N" -gt 30 ] && echo "  ...(残り省略。変更は合計 $N 件)"
  echo "  転送したファイル: $NF 件"
  row OK 配置 "転送 ${NF} 件 (退避先 ~/deploy_backup_$DAY)"
  rm -f "$RSYNC_OUT"
else
  tail -n 15 "$RSYNC_OUT" | sed 's/^/  /'; rm -f "$RSYNC_OUT"
  echo "NG: rsync に失敗しました。中止します。"; exit 6
fi
rsh "find \"\$HOME/$REMOTE_DIR_NAME/scripts\" -name '*.sh' -exec chmod u+x {} +" 2>/dev/null || echo "  (注意: scripts/*.sh の実行権限の付与に失敗)"

# 3) 時計合わせ
say "3) 時計合わせ"
PC_NOW=$(date +%s)
R_NOW=$(timeout 15 ssh "${SSH_OPTS[@]}" "$TARGET" 'date +%s' < /dev/null 2>/dev/null | tr -d '\r')
if ! [[ "$R_NOW" =~ ^[0-9]+$ ]]; then
  echo "  警告: 1台目の時計を取得できません(続行)"; row 警告 時計合わせ "取得できず"
elif [ "$R_NOW" -lt "$(date -d '2026-01-01' +%s)" ]; then
  echo "  警告: 1台目の時計が2026年より前で、明らかにずれています($(date -d "@$R_NOW" '+%F %T'))ので、PCの時刻($(date -d "@$PC_NOW" '+%F %T'))に合わせます)"
  if timeout 15 ssh "${SSH_OPTS[@]}" "$TARGET" "sudo -n /usr/bin/date -s @$PC_NOW >/dev/null 2>&1" < /dev/null; then
    echo "  OK: 合わせました"; row OK 時計合わせ "PCの時刻にした"
  else
    echo "  警告: sudo -n の date が使えず、時計を変えられません(続行。docs/clock_sync.md を確認)"
    row 警告 時計合わせ "変えられず(sudoers未導入?)"
  fi
else
  echo "  OK: 1台目の時計は2026年以降 ($(date -d "@$R_NOW" '+%F %T'))。変更しません(数分〜数日のずれは、このスクリプトでは直しません。ずれの確認は手順6の「時計」)"
fi

# 4) ビルド
say "4) ビルド: $BUILD_CMD"
BUILD_LOG=$(mktemp)
timeout 1500 ssh "${SSH_OPTS[@]}" -o ServerAliveInterval=15 "$TARGET" 'bash -s' > "$BUILD_LOG" 2>&1 <<EOS
set +u
$BUILD_SRC
cd "\$HOME/$REMOTE_DIR_NAME/ros2_ws" || exit 1
$BUILD_CMD
EOS
BRC=$?
tail -n 12 "$BUILD_LOG" | sed 's/^/  /'
rm -f "$BUILD_LOG"
if [ "$BRC" != "0" ]; then echo "NG: ビルドに失敗しました(終了コード $BRC)。再起動せずに中止します。"; exit 3; fi
echo "  OK: ビルド成功"; row OK ビルド "nokosahen_core nokosahen_bringup"

# 5) 再起動
if [ "$RESTART" = "manual" ]; then
  say "5) 再起動(manual)"
  MAPINFO=$(timeout 20 ssh "${SSH_OPTS[@]}" "$TARGET" 'bash -s' 2>/dev/null <<'EOS' | tr -d '\r'
for ep in lifecycle_status status; do
  f=$(mktemp)
  code=$(curl -s -m5 -o "$f" -w '%{http_code}' "http://127.0.0.1:8090/api/mapping/$ep" 2>/dev/null)
  echo "$ep.code=$code"
  [ "$code" = "200" ] && echo "$ep.body=$(head -c 400 "$f" | tr -d '\n')"
  rm -f "$f"
done
EOS
)
  MAP_MSG="不明(要ログイン): ダッシュボードの地図作成の状態を、ログインなしでは確認できません。未保存の地図があるかは、先にダッシュボードで確認してください"
  LC=$(printf '%s\n' "$MAPINFO" | sed -n 's/^lifecycle_status.body=//p')
  if [ -n "$LC" ]; then
    if printf '%s' "$LC" | grep -Eq '"mode": *"mapping"' && printf '%s' "$LC" | grep -Eq '"running": *true'; then
      MAP_MSG="地図作成中です。未保存なら消えます(先にダッシュボードで保存してください)"
    else
      MAP_MSG="地図作成中ではありません($(printf '%s' "$LC" | head -c 120))"
    fi
  elif [ "$(printf '%s\n' "$MAPINFO" | sed -n 's/^lifecycle_status.code=//p')" = "000" ]; then
    MAP_MSG="ダッシュボードが応答しません(ロボットは動いていない可能性)"
  fi
  echo "  地図の状態: $MAP_MSG"
  if [ "$YES" != "1" ]; then
    if [ ! -t 0 ]; then echo "確認できる端末がありません。続けるには --yes を付けてください。"; exit 5; fi
    printf '  再起動すると地図は消えます。続けますか [y/N] '
    read -r ANS
    case "$ANS" in y|Y|yes|YES) ;; *) echo "中止しました(配置とビルドは済んでいます)。"; exit 5 ;; esac
  fi
  rsh ': > $HOME/start_all_single.log'
  rsh "$START_CMD" > /dev/null 2>&1
  echo "  起動しました(裏で実行)。ログ ~/start_all_single.log を待ちます(最大${START_WAIT_TOTAL}秒)"
  T0=$(date +%s); RES=""
  while :; do
    sleep "$POLL_SEC"
    ST=$(timeout 15 ssh "${SSH_OPTS[@]}" "$TARGET" 'f=$HOME/start_all_single.log; if grep -aq "起動を中止します" "$f"; then echo ABORT; elif grep -aq "起動しました。ダッシュボード" "$f"; then echo DONE; else echo WAIT; fi' < /dev/null 2>/dev/null | tr -d '\r')
    [ "$ST" = "DONE" ] && { RES=DONE; break; }
    [ "$ST" = "ABORT" ] && { RES=ABORT; break; }
    [ $(( $(date +%s) - T0 )) -ge "$START_WAIT_TOTAL" ] && { RES=TIMEOUT; break; }
  done
  rsh 'tail -n 14 $HOME/start_all_single.log' 2>/dev/null | sed 's/^/  | /'
  if [ "$RES" != "DONE" ]; then
    row NG 再起動 "$RES (~/start_all_single.log を確認)"
    echo "NG: 再起動が完了しませんでした($RES)"; exit 4
  fi
  row OK 再起動 "manual$([ "$CAMERAS" = 1 ] && echo ' +カメラ')"
else
  say "5) 再起動: しない(--no-restart)"
fi

# 6) 起動後の確認(読み取りだけ)
NGS=0
if [ "$VERIFY" = "1" ]; then
  say "6) 起動後の確認(読み取りだけ。約20秒)"
  V=$(timeout 90 ssh "${SSH_OPTS[@]}" "$TARGET" 'bash -s' -- "$CAMERAS" 2>/dev/null <<'EOS' | tr -d '\r'
set +u
echo "http=$(curl -s -m5 -o /dev/null -w '%{http_code}' http://127.0.0.1:8090/)"
for pair in 'full_dashboard:full_dashboar[d]' 'micro_ros_agent:micro_ros_agen[t]' 'rplidar_node:rplidar_nod[e]' \
            'person_target_detector:person_target_detecto[r]' 'rear_camera_persons:rear_camera_person[s]'; do
  echo "proc.${pair%%:*}=$(pgrep -fc "${pair#*:}")"
done
echo "load=$(cut -d' ' -f1 /proc/loadavg)"
echo "cores=$(nproc)"
echo "epoch=$(date +%s)"
source /opt/ros/jazzy/setup.bash 2>/dev/null
source "$HOME/lidar_src_build/install/setup.bash" 2>/dev/null
source "$HOME/microros_ws/install/setup.bash" 2>/dev/null
source "$HOME/mirs2603_nokosahen/ros2_ws/install/setup.bash" 2>/dev/null
export ROS_DOMAIN_ID=77
unset FASTRTPS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_USE_QOS_FROM_XML
T=$(mktemp -d)
( timeout 14 ros2 topic hz /scan 2>&1 | grep -m1 'average rate' > "$T/scan" ) &
( timeout 14 ros2 topic hz /wheel_ticks 2>&1 | grep -m1 'average rate' > "$T/ticks" ) &
( timeout 10 ros2 topic list 2>/dev/null > "$T/list" ) &
( timeout 10 ros2 topic echo --once /estop_state 2>/dev/null | head -3 | tr '\n' ' ' > "$T/estop" ) &
wait
echo "scan_hz=$(sed -n 's/.*average rate: *//p' "$T/scan")"
echo "ticks_hz=$(sed -n 's/.*average rate: *//p' "$T/ticks")"
for t in /scan /wheel_ticks /estop_state; do echo "topic.$t=$(grep -cx "$t" "$T/list")"; done
echo "estop_val=$(cat "$T/estop")"
rm -rf "$T"
EOS
)
  getv() { printf '%s\n' "$V" | sed -n "s|^$1=||p" | head -1; }
  HTTP=$(getv http)
  case "$HTTP" in 200|301|302|303|401) row OK "ダッシュボード8090" "応答 $HTTP" ;; *) row NG "ダッシュボード8090" "応答なし (${HTTP:-取得できず})"; NGS=1 ;; esac
  chk_proc() { local n; n=$(getv "proc.$1"); if [ "${n:-0}" -ge 1 ] 2>/dev/null; then row OK "プロセス $1" "${n}件"; else row NG "プロセス $1" "動いていない"; NGS=1; fi; }
  chk_proc full_dashboard
  chk_proc micro_ros_agent
  chk_proc rplidar_node
  if [ "$CAMERAS" = "1" ]; then chk_proc person_target_detector; chk_proc rear_camera_persons; fi
  SH=$(getv scan_hz); TH=$(getv ticks_hz)
  if [ -n "$SH" ]; then row OK "/scan" "${SH} Hz"; else row NG "/scan" "届いていない"; NGS=1; fi
  if [ -n "$TH" ]; then row OK "/wheel_ticks" "${TH} Hz"; else row NG "/wheel_ticks" "届いていない"; NGS=1; fi
  if [ -n "$TH" ]; then row OK "ESP32の接続" "/wheel_ticks が流れている"; else row NG "ESP32の接続" "/wheel_ticks が無い(USB・電源を確認)"; NGS=1; fi
  if [ "$(getv topic./estop_state)" = "1" ]; then
    EV=$(getv estop_val); if [ -n "$EV" ]; then row OK "/estop_state" "$EV"; else row 警告 "/estop_state" "トピックあり・値は受信できず"; fi
  else row NG "/estop_state" "トピックが無い"; NGS=1; fi
  LD1=$(getv load); CORES=$(getv cores); CORES=${CORES:-6}
  if [[ "$LD1" =~ ^[0-9.]+$ ]]; then
    if awk "BEGIN{exit !($LD1 >= 2*$CORES)}"; then row NG "負荷(load average)" "$LD1 / ${CORES}コア (2倍以上)"; NGS=1
    elif awk "BEGIN{exit !($LD1 > $CORES)}"; then row 警告 "負荷(load average)" "$LD1 / ${CORES}コア (起動直後は高いことがあります)"
    else row OK "負荷(load average)" "$LD1 / ${CORES}コア"; fi
  else row 警告 "負荷(load average)" "取得できず"; fi
  EP=$(getv epoch)
  if [[ "$EP" =~ ^[0-9]+$ ]]; then
    D=$(( EP - $(date +%s) )); AD=${D#-}
    if [ "$EP" -lt "$(date -d '2026-01-01' +%s)" ]; then row NG 時計 "2026年より前 ($(date -d "@$EP" '+%F %T'))"; NGS=1
    elif [ "$AD" -le 5 ]; then row OK 時計 "PCとのずれ ${D}秒"
    else row 警告 時計 "PCとのずれ ${D}秒"; fi
  else row 警告 時計 "取得できず"; fi
else
  say "6) 起動後の確認: しない"
fi

# 結果の表
echo
echo "================ 結果 ================"
for r in "${ROWS[@]}"; do
  IFS='|' read -r st nm ct <<<"$r"
  printf '[%-2s] %s : %s\n' "$st" "$nm" "$ct"
done
echo "======================================"
if [ "$NGS" = "1" ]; then echo "確認にNGがあります。ログ(~/.nokosahen_persistent_base.log など)を確認してください。"; exit 1; fi
echo "完了しました。$([ "$RESTART" = "manual" ] && echo 'ダッシュボード: http://192.0.2.1:8090/')"
exit 0
