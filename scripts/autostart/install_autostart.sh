#!/usr/bin/env bash
# 自動起動(systemdサービス)の登録・解除(2026-10-19追加)。**sudoが必要**(人が実行する)。docs/autostart.md
#   sudo bash ~/mirs2603_nokosahen/scripts/autostart/install_autostart.sh jetson1            # 1台目: 電源投入で手動モードを起動
#   sudo bash ~/mirs2603_nokosahen/scripts/autostart/install_autostart.sh jetson2            # 2台目: 電源投入で音声スタックを起動
#   sudo bash ~/mirs2603_nokosahen/scripts/autostart/install_autostart.sh jetson1 --uninstall # 解除
#   (登録しても、いますぐには起動しない。すぐ起動するなら `--now` を付ける。1台目で--nowを付けると、ロボットの本体が起動する)
# sudo で実行したユーザー($SUDO_USER)のホームを使う。1台目は mirs03、2台目は mirs2603 のはず。
set -eu
TARGET="${1:-}"; shift || true
UNINSTALL=0; NOW=0
for a in "$@"; do
  case "$a" in
    --uninstall) UNINSTALL=1 ;;
    --now) NOW=1 ;;
    *) echo "不明な引数: $a" >&2; exit 2 ;;
  esac
done
case "$TARGET" in
  jetson1) TPL="nokosahen-robot.service.in"; UNIT="nokosahen-robot.service" ;;
  jetson2) TPL="nokosahen-voice.service.in"; UNIT="nokosahen-voice.service" ;;
  *) echo "使い方: sudo bash $0 jetson1|jetson2 [--uninstall] [--now]" >&2; exit 2 ;;
esac

if [ "$(id -u)" != "0" ]; then
  echo "sudo で実行してください(例: sudo bash $0 $TARGET)。" >&2; exit 1
fi
U="${SUDO_USER:-}"
if [ -z "$U" ] || [ "$U" = "root" ]; then
  echo "sudo を使う一般ユーザーで実行してください(root直接のログインでは、ホームが分かりません)。" >&2; exit 1
fi
HOME_DIR="$(getent passwd "$U" | cut -d: -f6)"
HERE="$(cd "$(dirname "$0")" && pwd)"
DEST="/etc/systemd/system/$UNIT"
SUDOERS="/etc/sudoers.d/nokosahen-poweroff"

if [ "$UNINSTALL" = "1" ]; then
  systemctl disable --now "$UNIT" 2>/dev/null || true
  rm -f "$DEST" "$SUDOERS"
  systemctl daemon-reload
  echo "解除しました: $UNIT と $SUDOERS(shutdownのsudo許可)"
  exit 0
fi

# 前提の確認
if [ "$TARGET" = "jetson1" ]; then
  [ -f "$HOME_DIR/mirs2603_nokosahen/scripts/autostart/boot_start.sh" ] || { echo "$HOME_DIR/mirs2603_nokosahen/scripts/autostart/boot_start.sh がありません(最新コードを配置してから)。" >&2; exit 1; }
  [ -f "$HOME_DIR/mirs2603_nokosahen/scripts/run_persistent_base_supervised.sh" ] || { echo "run_persistent_base_supervised.sh がありません。" >&2; exit 1; }
else
  [ -f "$HOME_DIR/start_voice_stack.sh" ] || { echo "$HOME_DIR/start_voice_stack.sh がありません(scripts/jetson2_setup/ からコピーしてください)。" >&2; exit 1; }
fi

sed -e "s|@USER@|$U|g" -e "s|@HOME@|$HOME_DIR|g" "$HERE/$TPL" > "$DEST"
chmod 644 "$DEST"
systemctl daemon-reload
systemctl enable "$UNIT"
echo "登録しました: $UNIT (ユーザー=$U ホーム=$HOME_DIR)。次回の電源投入から、自動で起動します。"

# 管理画面の「プログラム終了」(OSも停止)用: このユーザーが、`shutdown -h now` だけをパスワード無しで実行できるようにする。
# 他のコマンドは許可しない(引数も固定)。visudoで文法を確かめてから置く。
TMP_SUDOERS="$(mktemp)"
printf '%s ALL=(root) NOPASSWD: /usr/sbin/shutdown -h now\n' "$U" > "$TMP_SUDOERS"
if visudo -cf "$TMP_SUDOERS" >/dev/null; then
  install -m 0440 -o root -g root "$TMP_SUDOERS" "$SUDOERS"
  echo "shutdownのsudo許可を置きました: $SUDOERS($U が /usr/sbin/shutdown -h now だけをパスワード無しで実行可)"
else
  echo "警告: sudoersの文法確認に失敗したため、$SUDOERS は置きませんでした(プログラム終了でOSを停止できません)。" >&2
fi
rm -f "$TMP_SUDOERS"

if [ "$TARGET" = "jetson2" ]; then
  # PipeWire(音)のユーザーセッションを、ログインしなくても起動時から立ち上げる
  loginctl enable-linger "$U"
  systemctl enable docker 2>/dev/null || true
  echo "enable-linger を設定しました(ユーザー=$U。再起動後も音のセッションが残ります)。"
fi

if [ "$NOW" = "1" ]; then
  systemctl start "$UNIT"
  echo "起動しました: $UNIT"
fi
echo "状態の確認: systemctl status $UNIT   ログ: journalctl -u $UNIT -b"
