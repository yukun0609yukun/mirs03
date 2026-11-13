#!/usr/bin/env bash
# 文化祭モードの「電源投入での自動起動」のオン/オフ(2026-11-06追加)。1台目で実行する。sudo不要。
#   bash ~/mirs2603_nokosahen/scripts/festival/festival_mode.sh enable [reset_survey_once] [rear_cameras] [interval=45] ...
#   bash ~/mirs2603_nokosahen/scripts/festival/festival_mode.sh disable
#   bash ~/mirs2603_nokosahen/scripts/festival/festival_mode.sh status
# 仕組み: ~/.nokosahen_festival_mode(フラグファイル)があるときだけ、電源投入時の boot_start.sh が festival_start.sh --boot を
#   呼ぶ。無ければ従来どおり手動モード(自律行動なし)。フラグファイルの各行がオプション(festival_start.sh のヘッダ参照)。
#   systemdサービス(nokosahen-robot.service)は変更しない。登録済み(install_autostart.sh jetson1)が前提。
set -u
FLAG="${NOKOSAHEN_FESTIVAL_FLAG:-$HOME/.nokosahen_festival_mode}"
MARKER="$HOME/.nokosahen_festival_survey_reset_done"
case "${1:-}" in
  enable)
    shift
    for w in "$@"; do
      case "$w" in reset_survey_once|rear_cameras|no_person_detector|no_dds_wired|light_localization|interval=[0-9]*) ;; *) echo "不明な語: $w" >&2; exit 2 ;; esac
    done
    { echo "# 文化祭モード(festival_mode.sh enable $(date '+%F %T'))。1行に1語。消せば次回の電源投入から手動モードに戻る"; for w in "$@"; do echo "$w"; done; } > "$FLAG"
    echo "有効にしました: $FLAG"; cat "$FLAG"
    if grep -qx reset_survey_once "$FLAG" && [ -e "$MARKER" ]; then echo "注意: 退避済みマーカー($MARKER)があるため、アンケートの退避は行われません(やり直すなら、このマーカーを自分で消す)" ;fi ;;
  disable) rm -f "$FLAG"; echo "無効にしました(次回の電源投入から手動モード): $FLAG" ;;
  status)
    if [ -f "$FLAG" ]; then echo "文化祭モード: 有効"; cat "$FLAG"; else echo "文化祭モード: 無効(手動モードで起動)"; fi
    [ -e "$MARKER" ] && echo "アンケート退避マーカー: $(cat "$MARKER")" || echo "アンケート退避マーカー: なし(未退避)" ;;
  *) echo "使い方: $0 enable [reset_survey_once] [rear_cameras] [no_person_detector] [no_dds_wired] [light_localization] [interval=秒] | disable | status" >&2; exit 2 ;;
esac
