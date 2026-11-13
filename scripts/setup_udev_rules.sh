#!/usr/bin/env bash
# ESP32 / LiDAR(RPLIDAR S1) / 決済横Webカメラ / USB音声アダプタを、
# 挿すUSBポートや挿す順番に関係なく同じデバイス名で使えるようにする
# 対話式セットアップスクリプト。
#
# 使い方: 各機器を「1つだけ」挿した状態で、聞かれるままに進める。
# 実行後は /etc/udev/rules.d/99-nokosahen.rules が生成され、以後は
#   /dev/esp32
#   /dev/rplidar
#   /dev/webcam_checkout
# という固定名で launch/config から参照できるようになる。
# USB音声アダプタはtty/videoではないため、ALSAのby-id名の調べ方だけを案内する。
#
# 注意: 現状Webカメラは物理1台のみを想定している。person_target_detector
# (入口の人物検出、Stage2)とcamera_counter(決済カウント、Stage3)は
# どちらも /dev/webcam_checkout を参照する設定になっているため、
# 2台目のWebカメラを追加しない限り両方を同時に有効化しないこと
# (nokosahen.yamlのコメント参照)。
set -euo pipefail

RULES_FILE="/etc/udev/rules.d/99-nokosahen.rules"

pause() {
  read -r -p "$1 挿したら Enter を押してください... " _
}

list_tty() {
  ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
}

list_video() {
  ls /dev/video* 2>/dev/null || true
}

# $1: 一時退避前リスト, $2: 現在リスト -> 新規に増えたデバイスパスを1つ返す
diff_new() {
  local before="$1" after="$2"
  comm -13 <(echo "$before" | sort) <(echo "$after" | sort) | head -n1
}

add_serial_rule() {
  local dev="$1" symlink="$2"
  local info vendor product serial attrs
  info="$(udevadm info -a -n "${dev}")"
  vendor="$(echo "${info}" | grep -m1 '{idVendor}' | sed -E 's/.*=="(.*)"/\1/')"
  product="$(echo "${info}" | grep -m1 '{idProduct}' | sed -E 's/.*=="(.*)"/\1/')"
  serial="$(echo "${info}" | grep -m1 '{serial}' | sed -E 's/.*=="(.*)"/\1/' || true)"

  if [[ -n "${serial}" ]]; then
    attrs="ATTRS{idVendor}==\"${vendor}\", ATTRS{idProduct}==\"${product}\", ATTRS{serial}==\"${serial}\""
  else
    attrs="ATTRS{idVendor}==\"${vendor}\", ATTRS{idProduct}==\"${product}\""
  fi

  echo "SUBSYSTEM==\"tty\", ${attrs}, SYMLINK+=\"${symlink}\", MODE=\"0666\", GROUP=\"dialout\"" >> "${RULES_FILE}.tmp"
  echo "  -> ${dev} (vendor=${vendor} product=${product} serial=${serial:-なし}) を /dev/${symlink} に固定"
}

add_video_rule() {
  local dev="$1" symlink="$2"
  local devnum info vendor product
  devnum="$(basename "${dev}" | sed 's/video//')"
  info="$(udevadm info -a -n "${dev}")"
  vendor="$(echo "${info}" | grep -m1 '{idVendor}' | sed -E 's/.*=="(.*)"/\1/')"
  product="$(echo "${info}" | grep -m1 '{idProduct}' | sed -E 's/.*=="(.*)"/\1/')"

  echo "SUBSYSTEM==\"video4linux\", ATTRS{idVendor}==\"${vendor}\", ATTRS{idProduct}==\"${product}\", ATTR{index}==\"0\", SYMLINK+=\"${symlink}\", MODE=\"0666\"" >> "${RULES_FILE}.tmp"
  echo "  -> ${dev} (vendor=${vendor} product=${product}) を /dev/${symlink} に固定"
}

echo "=== のこさへん USB機器 固定名セットアップ ==="
echo "先に ESP32・LiDAR・Webカメラ・USB音声アダプタを全て抜いておいてください。"
read -r -p "全部抜いたら Enter... " _

: > "${RULES_FILE}.tmp"

# 1) ESP32 (micro-ROS シリアル)
before="$(list_tty)"
pause "ESP32だけを"
after="$(list_tty)"
dev="$(diff_new "${before}" "${after}")"
if [[ -z "${dev}" ]]; then
  echo "!! 新しいシリアルデバイスが見つかりませんでした。ESP32のUSBケーブル(データ線)を確認してください。" >&2
  exit 1
fi
add_serial_rule "${dev}" "esp32"

# 2) LiDAR (RPLIDAR S1)
before="$(list_tty)"
pause "LiDAR(RPLIDAR S1)だけを追加で"
after="$(list_tty)"
dev="$(diff_new "${before}" "${after}")"
if [[ -z "${dev}" ]]; then
  echo "!! 新しいシリアルデバイスが見つかりませんでした。" >&2
  exit 1
fi
add_serial_rule "${dev}" "rplidar"

# 3) 決済横Webカメラ
before="$(list_video)"
pause "決済横Webカメラだけを追加で"
after="$(list_video)"
dev="$(diff_new "${before}" "${after}")"
if [[ -z "${dev}" ]]; then
  echo "!! 新しいビデオデバイスが見つかりませんでした。" >&2
  exit 1
fi
add_video_rule "${dev}" "webcam_checkout"

mv "${RULES_FILE}.tmp" "${RULES_FILE}"
udevadm control --reload-rules
udevadm trigger

echo
echo "生成されたルール (${RULES_FILE}):"
cat "${RULES_FILE}"
echo
echo "確認:"
ls -l /dev/esp32 /dev/rplidar /dev/webcam_checkout 2>/dev/null || echo "(抜き挿しし直すと反映されます)"

echo
echo "=== USB音声出力アダプタ ==="
echo "USB音声アダプタはシリアル/videoデバイスではないため、udevルールではなく"
echo "ALSAの by-id 名で固定的に参照します。アダプタを挿した状態で次を確認してください:"
echo
echo "  ls -l /dev/snd/by-id/"
echo
echo "表示された by-id のパスを voice_bridge の設定(OUTPUT_DEVICE)に指定してください。"
echo "カード番号(hw:X,Y)は挿す順で変わるため使わないこと。"

echo
echo "=== Wi-Fi子機 ==="
echo "Wi-FiアダプタはMACアドレスで識別されるため、udevルールは不要です。"
echo "NetworkManagerで該当SSIDへの接続を1度作成しておけば、"
echo "どのUSBポートに挿しても自動的に再接続されます。"
echo "  nmcli device wifi connect \"<SSID>\" password \"<PASSWORD>\""
