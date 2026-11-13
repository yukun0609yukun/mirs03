#!/usr/bin/env bash
# 【2026-08-05追加】Jetson 2台分散構成で、直結LAN経由の疎通とROS 2の
# 相互発見(DDS)が実際にできているかを確認する。
#
# 使い方(どちらのJetsonで実行してもよい、ロール(A/B)を指定):
#   bash scripts/verify_inter_jetson_link.sh A   # Jetson A側で実行
#   bash scripts/verify_inter_jetson_link.sh B   # Jetson B側で実行
#
# 前提: 事前に setup_inter_jetson_link.sh で両機に固定IPを設定し、
# 対向機で persistent_base_control.launch.py / persistent_base_app.launch.py
# を起動済みであること。ROS_DOMAIN_IDが両機で一致していること
# (既定77、nokosahen.yaml/起動スクリプトと同じ値)。
#
# 【未検証】2台目Jetsonのセットアップ前に用意した準備段階のスクリプトであり、
# 実機での動作確認はまだ行っていない。
set -uo pipefail

ROLE="${1:-}"
if [[ "${ROLE}" != "A" && "${ROLE}" != "B" ]]; then
    echo "使い方: bash $0 A|B" >&2
    exit 1
fi

SUBNET_PREFIX="${SUBNET_PREFIX:-192.168.50}"
if [[ "${ROLE}" == "A" ]]; then
    PEER_IP="${SUBNET_PREFIX}.2"
    PEER_NODE="web_ui"        # Jetson B(アプリ系)にだけ存在するノード
else
    PEER_IP="${SUBNET_PREFIX}.1"
    PEER_NODE="wheel_odom"    # Jetson A(足回り系)にだけ存在するノード
fi

echo "=== 1. 対向Jetson(${PEER_IP})へのping ==="
if ping -c 3 -W 2 "${PEER_IP}"; then
    echo "OK: ping疎通あり"
else
    echo "NG: pingが通りません。LANケーブル・固定IP設定(setup_inter_jetson_link.sh)を確認してください。" >&2
    exit 1
fi

echo
echo "=== 2. ROS_DOMAIN_ID確認 ==="
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-未設定(既定0扱いになるため注意)}"

echo
echo "=== 3. 対向機のノードが見えるか(ros2 node list) ==="
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
NODE_LIST="$(timeout 5 ros2 node list 2>&1)"
echo "${NODE_LIST}"
if echo "${NODE_LIST}" | grep -qx "/${PEER_NODE}"; then
    echo "OK: 対向機のノード /${PEER_NODE} が見えています(DDS相互発見が機能しています)"
else
    echo "NG: /${PEER_NODE} が見つかりません。対向機でlaunchが起動しているか、" >&2
    echo "    ROS_DOMAIN_IDが両機で一致しているか、ファイアウォールでUDPマルチキャストが" >&2
    echo "    塞がれていないかを確認してください。" >&2
    exit 1
fi

echo
echo "=== 完了: 2台間の通信は正常です ==="
