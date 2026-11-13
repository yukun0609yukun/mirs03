#!/usr/bin/env bash
# 【2026-08-05追加】Jetson 2台をLANケーブルで直接接続する構成向けに、
# 有線インターフェースへ固定IPを設定する。
#
# 【なぜ必要か】
# Jetson A(足回り・安全系)はダッシュボード等の外部公開ノードを持たないため、
# WiFi(外部LAN)には繋がず、Jetson Bへの直結LANケーブル1本のみで良い設計を
# 想定している。ルーターを介さない直結のためDHCPが無く、両機とも固定IPを
# 手動設定する必要がある(既存のWindows PC側でも同様の固定IP設定を何度か
# 行っている、docs/known_issues_and_startup_checklist.md 参照)。
# Jetson B側はWiFi(外部LAN、ダッシュボード用)とこの直結LAN(対Jetson A用)の
# 2つのネットワークに同時に繋がる想定(Linuxはマルチホーム構成に対応済み)。
#
# 使い方(各Jetsonでロール(A/B)を指定して実行、要sudo/nmcli):
#   bash scripts/setup_inter_jetson_link.sh A   # Jetson A側で実行
#   bash scripts/setup_inter_jetson_link.sh B   # Jetson B側で実行
#
# インターフェース名は既定で eth0 を想定。異なる場合は環境変数で上書き:
#   IFACE=enP8p1s0 bash scripts/setup_inter_jetson_link.sh A
#
# 設定後、対向機への疎通確認は verify_inter_jetson_link.sh を使うこと。
#
# 【未検証】2台目Jetsonのセットアップ前に用意した準備段階のスクリプトであり、
# 実機での動作確認・実際のインターフェース名の確認はまだ行っていない。
# 特に IFACE の既定値(eth0)は実機で `ip link show` により確認・調整すること。
set -euo pipefail

ROLE="${1:-}"
if [[ "${ROLE}" != "A" && "${ROLE}" != "B" ]]; then
    echo "使い方: bash $0 A|B" >&2
    exit 1
fi

IFACE="${IFACE:-eth0}"
SUBNET_PREFIX="${SUBNET_PREFIX:-192.168.50}"
CONN_NAME="nokosahen-inter-jetson"

if [[ "${ROLE}" == "A" ]]; then
    MY_IP="${SUBNET_PREFIX}.1"
else
    MY_IP="${SUBNET_PREFIX}.2"
fi

echo "=== Jetson ${ROLE}: ${IFACE} に固定IP ${MY_IP}/24 を設定します ==="

if ! command -v nmcli >/dev/null 2>&1; then
    echo "nmcliが見つかりません。NetworkManager未導入の環境では手動で" >&2
    echo "'ip addr add ${MY_IP}/24 dev ${IFACE}' 等を使ってください。" >&2
    exit 1
fi

if nmcli -t -f NAME connection show | grep -qx "${CONN_NAME}"; then
    echo "既存の接続プロファイル '${CONN_NAME}' を更新します。"
    nmcli connection modify "${CONN_NAME}" ipv4.addresses "${MY_IP}/24"
else
    echo "接続プロファイル '${CONN_NAME}' を新規作成します。"
    nmcli connection add type ethernet ifname "${IFACE}" con-name "${CONN_NAME}" \
        ipv4.addresses "${MY_IP}/24" ipv4.method manual
fi

# ゲートウェイ・DNSは直結LANでは不要(このリンク経由で外部通信はしない)。
nmcli connection modify "${CONN_NAME}" ipv4.gateway "" ipv4.dns "" ipv4.never-default yes

nmcli connection up "${CONN_NAME}"

echo
echo "=== 完了 ==="
echo "このJetsonのIP: ${MY_IP}"
echo "対向Jetsonの想定IP: ${SUBNET_PREFIX}.$([[ ${ROLE} == A ]] && echo 2 || echo 1)"
echo "確認は verify_inter_jetson_link.sh を使ってください。"
