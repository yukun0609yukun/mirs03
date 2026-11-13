#!/usr/bin/env bash
# Wi-Fi(開発用ネットワーク)から有線(機器間ハブ)への転送を止める(2026-10-22、全体レビュー1A)。
#
# 背景: 1台目のJetsonでIP転送(net.ipv4.ip_forward)が有効だと、Wi-Fiにつないだ他の端末から、有線側の
#       Jetson2(198.51.100.3)・ラズパイ(198.51.100.1)へ、認証なしで直接届いてしまう。
#       1台目自身への接続(ダッシュボード・SSH)は影響を受けない。
#
# 使い方(sudoが必要なので自分で実行):
#   bash scripts/block_wifi_forward.sh check     # 現状の確認だけ(何も変えない。sudo不要の項目もある)
#   sudo bash scripts/block_wifi_forward.sh apply   # 止める(再起動で消える)
#   sudo bash scripts/block_wifi_forward.sh remove  # 元に戻す
# 再起動後も止めたいときは install を使う(systemdの単位を入れる)。
#   sudo bash scripts/block_wifi_forward.sh install
#
# インタフェース名は環境変数で変えられる: WIFI_IF(既定 wlP1p1s0) / WIRED_IF(既定 enP8p1s0)
WIFI_IF="${WIFI_IF:-wlP1p1s0}"
WIRED_IF="${WIRED_IF:-enP8p1s0}"
TABLE="nokosahen_guard"

rules() {
  cat <<NFT
table inet ${TABLE} {
  chain forward {
    type filter hook forward priority -10; policy accept;
    iifname "${WIFI_IF}" oifname "${WIRED_IF}" drop
    iifname "${WIRED_IF}" oifname "${WIFI_IF}" drop
  }
}
NFT
}

case "${1:-check}" in
  check)
    echo "== ip_forward (1なら転送が有効) =="
    sysctl net.ipv4.ip_forward 2>&1
    echo "== 現在のnftablesのforward関連(権限がなければ空・エラー) =="
    (nft list ruleset 2>&1 || sudo -n nft list ruleset 2>&1) | grep -n -i -E "forward|masquerade|${TABLE}|${WIFI_IF}|${WIRED_IF}" || echo "(該当なし、または読み取り権限なし。sudo nft list ruleset を試してください)"
    echo "== インタフェース =="
    ip -brief addr 2>&1 | grep -E "${WIFI_IF}|${WIRED_IF}" || ip -brief addr
    ;;
  apply)
    nft delete table inet "${TABLE}" 2>/dev/null
    rules | nft -f - && echo "OK: ${WIFI_IF} <-> ${WIRED_IF} の転送を止めました(再起動で消えます)"
    ;;
  remove)
    nft delete table inet "${TABLE}" 2>/dev/null && echo "OK: 元に戻しました" || echo "(入っていませんでした)"
    ;;
  install)
    install -d /etc/nokosahen
    rules > /etc/nokosahen/block_wifi_forward.nft
    cat > /etc/systemd/system/nokosahen-block-wifi-forward.service <<UNIT
[Unit]
Description=Block forwarding between Wi-Fi and wired robot network
After=network-pre.target
Before=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/sbin/nft -f /etc/nokosahen/block_wifi_forward.nft
ExecStop=/usr/sbin/nft delete table inet ${TABLE}

[Install]
WantedBy=multi-user.target
UNIT
    systemctl daemon-reload && systemctl enable --now nokosahen-block-wifi-forward.service
    ;;
  *)
    echo "使い方: $0 check|apply|remove|install" >&2
    exit 2
    ;;
esac
