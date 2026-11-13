#!/usr/bin/env bash
# パソコンから、起動前に「パソコンの時刻 -> 1台目 -> 2台目・ラズパイ」へ時計を揃える。
# パソコン側(WSL内)で実行する。ロボットの起動自体は --start を付けた時だけ行う。
# 詳細: docs/clock_sync.md
#
# 使い方(WSL):
#   bash /mnt/c/Users/user/Documents/mirs2603_nokosahen/scripts/start_all_with_clock.sh
#   bash .../start_all_with_clock.sh --start            # 時計合わせ後、自律ありで監視起動
#   bash .../start_all_with_clock.sh --start rc_only    # 同、ラジコンのみ
# Windowsから: wsl -- bash /mnt/c/Users/user/Documents/mirs2603_nokosahen/scripts/start_all_with_clock.sh
# 注意: --start は既に監視スクリプトが動いている1台目で使わないこと(二重起動になる)。
set -u

KEY="${NOKOSAHEN_SSH_KEY:-$HOME/.ssh/nokosahen_jetson}"
HOST="${NOKOSAHEN_JETSON1:-robot@192.0.2.1}"
SSH=(ssh -i "$KEY" -o BatchMode=yes -o ConnectTimeout=8 "$HOST")

# 1) パソコンの時刻(正)を1台目へ。@epochでタイムゾーン差を避ける
NOW=$(date +%s)
if "${SSH[@]}" "sudo -n /usr/bin/date -s @$NOW >/dev/null && echo \"1台目の時計OK: \$(date '+%F %T')\"" ; then
    :
else
    echo "警告: 1台目の時計を設定できません(接続不可 or sudoers未導入)。続行します。" >&2
fi

# 2) 1台目から2台へ(1台目の時計が正しくなってから)
"${SSH[@]}" 'bash ~/mirs2603_nokosahen/scripts/sync_clocks.sh' || echo "警告: sync_clocks.sh が失敗しました。" >&2

# 3) 任意: ロボット起動
if [ "${1:-}" = "--start" ]; then
    MODE="${2:-}"
    "${SSH[@]}" "nohup bash ~/mirs2603_nokosahen/scripts/run_persistent_base_supervised.sh $MODE >> ~/.nokosahen_supervisor_wrapper.log 2>&1 < /dev/null & disown; echo 起動しました"
fi
