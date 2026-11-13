#!/usr/bin/env bash
# 有線(ハブ)を、Wi-Fiより必ず優先して使う設定にする(経路のメトリックを固定)。
# 有線=50、Wi-Fi=700。数字が小さいほうが優先。再起動後も残る。
# 実行: sudo bash wired_priority.sh   (2台目・ラズパイそれぞれで)
set -e
nmcli -t -f NAME,TYPE connection show | while IFS=: read -r name type; do
  case "$type" in
    802-3-ethernet)
      case "$name" in docker0|l4tbr0) continue;; esac
      nmcli connection modify "$name" ipv4.route-metric 50 ipv6.route-metric 50 && echo "有線: $name -> 50";;
    802-11-wireless)
      nmcli connection modify "$name" ipv4.route-metric 700 ipv6.route-metric 700 && echo "Wi-Fi: $name -> 700";;
  esac
done
# 有効なものは再接続して反映(SSHは有線なので切れない。念のため有線を先に)
for dev in $(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="ethernet"{print $1}'); do nmcli device reapply "$dev" 2>/dev/null || true; done
for dev in $(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi"{print $1}'); do nmcli device reapply "$dev" 2>/dev/null || true; done
echo "--- ルート"; ip route | grep default
