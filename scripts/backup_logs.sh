#!/usr/bin/env bash
# 稼働ログ(消費推移・発話履歴・接客/A/Bテスト履歴のSQLite、アンケート回答のJSONL)の定期バックアップ
# (2026-07-22追加)。実証実験中のデータ蓄積ファイルが万一壊れても復旧できる
# ように、タイムスタンプ付きでコピーを残す。
#
# 【2026-11-05変更】(Opusレビュー指摘)
#  - SQLite(WAL)は cp だと本体だけコピーされ、最新データが抜ける/壊れたコピーになる。
#    sqlite3 の ".backup" で整合の取れたコピーを作る(sqlite3 が無いときだけ従来の cp にフォールバックして警告)。
#  - 古い世代の削除は「7日より古い」(find -mtime)だと、Jetsonの時計が未来に飛ぶと全世代を消してしまう。
#    ファイルごと(DBごと)に「最新 KEEP 個(既定14)を残す」件数ベースに変えた。
#  - ~/survey_data/responses.jsonl(アンケート回答)も対象に加えた。
#
# 使い方(手動実行):
#   bash ~/mirs2603_nokosahen/scripts/backup_logs.sh
#   KEEP=30 bash ...   # 残す世代数を変える(既定14)
#
# 定期実行にはユーザーのcrontabに登録する(sudo不要):
#   crontab -e
#   # 1時間ごとにバックアップする例(14世代=約14時間分。日数を残したいなら間隔を広げるか KEEP を増やす)
#   0 * * * * bash ~/mirs2603_nokosahen/scripts/backup_logs.sh >> ~/.nokosahen_backup.log 2>&1
set -uo pipefail

BACKUP_DIR="${HOME}/.nokosahen_backups"
KEEP="${KEEP:-14}"
case "$KEEP" in ''|*[!0-9]*) KEEP=14 ;; esac
[ "$KEEP" -lt 1 ] && KEEP=1
mkdir -p "$BACKUP_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# 最新 KEEP 個(名前=タイムスタンプ順)を残して、それより古い世代を消す。$1=ファイル名の接頭辞(例 .nokosahen_consumption)、$2=拡張子
prune_old() {
    local prefix="$1" ext="$2" f n=0
    # ls -1r は名前の降順(新しい順)。接頭辞_日付_時刻.拡張子 の形だけを対象にする(他のファイルは消さない)
    while IFS= read -r f; do
        n=$((n + 1))
        if [ "$n" -gt "$KEEP" ]; then
            rm -f -- "${BACKUP_DIR}/${f}"
        fi
    done < <(cd "$BACKUP_DIR" && ls -1r -- "${prefix}"_[0-9]*_[0-9]*."${ext}" 2>/dev/null)
}

HAVE_SQLITE3=1
command -v sqlite3 >/dev/null 2>&1 || HAVE_SQLITE3=0
if [ "$HAVE_SQLITE3" -eq 0 ]; then
    echo "$(date): 警告 sqlite3 が見つかりません。cp でコピーします(WALの最新データが抜ける恐れ。sudo apt install sqlite3 を推奨)"
fi

for db in .nokosahen_consumption.db .nokosahen_utterance_log.db .nokosahen_engagement.db .nokosahen_detection_log.db; do
    src="${HOME}/${db}"
    if [ -f "$src" ]; then
        dst="${BACKUP_DIR}/${db%.db}_${TIMESTAMP}.db"
        if [ "$HAVE_SQLITE3" -eq 1 ]; then
            # シングルクォートを含むパスでも壊れないよう '' に置換する
            if sqlite3 "$src" ".backup '${dst//\'/\'\'}'"; then
                echo "$(date): バックアップ完了 ${db}"
            else
                rm -f -- "$dst"
                echo "$(date): 警告 sqlite3 のバックアップに失敗 ${db}(cp で代替します)"
                cp "$src" "$dst" && echo "$(date): バックアップ完了(cp) ${db}"
            fi
        else
            cp "$src" "$dst" && echo "$(date): バックアップ完了(cp) ${db}"
        fi
        prune_old "${db%.db}" "db"
    fi
done

# アンケートの回答(追記専用のJSONL)。コピーしている最中に追記されても、行単位なので壊れた最終行以外は読める
survey="${HOME}/survey_data/responses.jsonl"
if [ -f "$survey" ]; then
    if cp "$survey" "${BACKUP_DIR}/survey_responses_${TIMESTAMP}.jsonl"; then
        echo "$(date): バックアップ完了 survey_data/responses.jsonl"
    fi
    prune_old "survey_responses" "jsonl"
fi
