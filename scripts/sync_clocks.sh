#!/usr/bin/env bash
# 1台目Jetsonの時刻を基準に、2台目Jetson(ssh j2)とラズパイ(ssh pi)の時計を合わせる。
# 1台目上で実行する。インターネット/NTP/RTCが無く、再起動のたびに時計が古い日付に戻るための対策。
# 詳細: docs/clock_sync.md
#
# 使い方:
#   bash ~/mirs2603_nokosahen/scripts/sync_clocks.sh            # 実際に合わせる
#   bash ~/mirs2603_nokosahen/scripts/sync_clocks.sh --check    # ずれを表示するだけ
#   HOSTS="j2" bash .../sync_clocks.sh                          # 対象を絞る
# 環境変数: THRESHOLD(許容ずれ秒、既定2) / ALLOW_OLD_BASE=1(1台目が2026年より前でも配る)
#
# 方針: 失敗した機器はスキップして警告するだけで、全体は止めない(常にexit 0、--checkも同様)。
# 2台目/ラズパイにsudoersが無い(sudo -nが通らない)場合は、人間が実行する1行を表示する。
set -u

HOSTS="${HOSTS:-j2 pi}"
THRESHOLD="${THRESHOLD:-2}"
CHECK_ONLY=0
[ "${1:-}" = "--check" ] && CHECK_ONLY=1
SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=4 -o ServerAliveInterval=2 -o ServerAliveCountMax=2 -o StrictHostKeyChecking=accept-new)
# 1台あたりの全体タイムアウト(秒)。ホストが落ちていてもハングしない
PER_HOST_TIMEOUT=15

now=$(date +%s)
if [ "$now" -lt "$(date -d '2026-01-01' +%s)" ] && [ "${ALLOW_OLD_BASE:-0}" != "1" ]; then
    echo "警告: 1台目自身の時計が古い($(date '+%F %T'))ため、同期しません(古い時刻を配らない)。" >&2
    echo "      パソコンから start_all_with_clock.sh で先に1台目を合わせてください。" >&2
    exit 0
fi

for h in $HOSTS; do
    # 1) 相手の現在時刻(epoch)を取得
    t0=$(date +%s)
    remote=$(timeout "$PER_HOST_TIMEOUT" ssh "${SSH_OPTS[@]}" "$h" 'date +%s' 2>/dev/null | tr -d '\r')
    t1=$(date +%s)
    if ! [[ "$remote" =~ ^[0-9]+$ ]]; then
        echo "[$h] 警告: ssh接続できません(電源/ケーブル/鍵を確認)。スキップします。"
        continue
    fi
    # 往復の中間時刻を基準にずれを出す(正=相手が進んでいる)
    diff=$(( remote - (t0 + t1) / 2 ))
    adiff=${diff#-}
    if [ "$adiff" -le "$THRESHOLD" ]; then
        echo "[$h] OK: ずれ ${diff}秒(許容${THRESHOLD}秒以内)。何もしません。"
        continue
    fi
    echo "[$h] ずれ ${diff}秒(相手が1台目より$([ "$diff" -gt 0 ] && echo 進んで || echo 遅れて)います)"
    if [ "$CHECK_ONLY" = "1" ]; then
        continue
    fi
    # 2) 時刻設定。@epochで渡すのでタイムゾーン差の影響を受けない
    target=$(date +%s)
    if timeout "$PER_HOST_TIMEOUT" ssh "${SSH_OPTS[@]}" "$h" "sudo -n /usr/bin/date -s @$target >/dev/null 2>&1"; then
        after=$(timeout "$PER_HOST_TIMEOUT" ssh "${SSH_OPTS[@]}" "$h" 'date +%s' 2>/dev/null | tr -d '\r')
        if [[ "$after" =~ ^[0-9]+$ ]]; then
            echo "[$h] 時計を合わせました。残りずれ $(( after - $(date +%s) ))秒"
        else
            echo "[$h] 時計を設定しましたが、確認の取得に失敗しました。"
        fi
    else
        echo "[$h] 警告: sudo -n が使えず時計を変えられません。スキップします。"
        echo "      初回のみ、人間が1回実行してください(パスワード入力あり):"
        echo "        scp ~/mirs2603_nokosahen/scripts/install_date_sudoers.sh $h:~/ && ssh -t $h 'sudo bash ~/install_date_sudoers.sh'"
        echo "      今すぐ手動で合わせるなら:"
        echo "        ssh -t $h \"sudo date -s @$target\""
    fi
done
exit 0
