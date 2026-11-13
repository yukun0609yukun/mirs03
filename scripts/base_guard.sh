#!/usr/bin/env bash
# 【2026-10-23】persistent_base系の二重起動ガード(docs/overlap_audit_2026-10-23.md の B5)。
# start_persistent_base*.sh の先頭で source して、nokosahen_base_guard <スクリプト名> を呼ぶ。
#
# persistent_base が二重に動くと、ESP32のmicro-ROSエージェント・rplidar(シリアルの取り合い)・director・web_uiのポート等が二重になる。
# そこで、run_persistent_base_supervised.sh と同じ排他ロック(既定 /tmp/nokosahen_supervisor.lock)を取り、
# 取れない(監視ループ、または別のstart_persistent_base*.shの起動物が動いている)か、persistent_base系のlaunchが動いていたら、
# 動いているプロセスと止め方を表示して終了コード1で拒否する。
#
# ロックの寿命: このスクリプトが開いたfd 9は、起動する ros2 launch に「わざと」引き継がせる(監視ループ側は 9>&- で閉じるが、
# こちらは nohup で投げっぱなしにするので、launchが動いている間ロックを持たせないと、2つ目を拒否できない)。
# launchが全部終われば、ロックは自動で解放される。
# 回避(意図的に二重にする・1台で分散構成を試す等): NOKOSAHEN_ALLOW_MULTI_BASE=1
# start_all.sh は先に stop_everything.sh で全停止してから監視ループを起動する(このガードを通らない)ので、矛盾しない。

# persistent_base系の動作中プロセスを表示用に集める(pgrep -f は自分の文字列を拾わないよう、パターンを割ってある)。
nokosahen_base_running() {
    pgrep -af 'run_persistent_base_superv[i]sed|launch nokosahen_bringup persistent_base[a-z_]*\.launch\.py' 2>/dev/null
}

nokosahen_base_guard() {
    local who="${1:-start_persistent_base}"
    if [ "${NOKOSAHEN_ALLOW_MULTI_BASE:-0}" = "1" ]; then
        echo "【警告】NOKOSAHEN_ALLOW_MULTI_BASE=1 のため、二重起動の確認を省略します($who)。" >&2
        return 0
    fi
    local lock="${NOKOSAHEN_SUPERVISOR_LOCK:-/tmp/nokosahen_supervisor.lock}"
    local held=0
    # ロックを開けない(別ユーザーのファイル等)ときは、ロックでの確認はあきらめて、プロセスの確認だけ行う
    if { exec 9>"$lock"; } 2>/dev/null; then
        flock -n 9 || held=1
    else
        echo "【注意】ロックファイル $lock を開けないため、プロセスの有無だけで確認します。" >&2
    fi
    local procs
    procs="$(nokosahen_base_running)"
    if [ "$held" = "1" ] || [ -n "$procs" ]; then
        {
            echo "【起動しません】すでに persistent_base 系が動いています($who)。二重に起動すると、ESP32・LiDARのシリアルの取り合い、ポート重複などで壊れます。"
            if [ -n "$procs" ]; then
                echo "動いているプロセス:"
                echo "$procs" | sed 's/^/  /'
            else
                echo "動いているプロセス: (排他ロック $lock は保持されているが、プロセス名では見つからない)"
            fi
            echo "止めるには:"
            echo "  bash ~/mirs2603_nokosahen/scripts/stop_robot.sh        (ロボットのノードを止める)"
            echo "  bash ~/mirs2603_nokosahen/scripts/stop_everything.sh    (監視ループ・OCRも含めて全部止める。systemdの監視ループもこちらで止まる)"
            echo "意図的に二重で動かすなら: NOKOSAHEN_ALLOW_MULTI_BASE=1 を付けて再実行してください。"
        } >&2
        exit 1
    fi
    return 0
}
