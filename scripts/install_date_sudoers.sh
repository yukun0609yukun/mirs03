#!/usr/bin/env bash
# `date -s` だけをパスワード無しで許可するsudoersを導入する(2台目Jetson・ラズパイ用)。
# 時計合わせ(sync_clocks.sh)を無人で行うため。許可するのは /usr/bin/date -s のみ。
# 各機で1回だけ、人間が実行する(sudoパスワードが必要)。1台目に導入済みのものと同等。
#   scp ~/mirs2603_nokosahen/scripts/install_date_sudoers.sh j2:~/
#   ssh -t j2 'sudo bash ~/install_date_sudoers.sh'
# (ラズパイは j2 を pi に読み替え。1台目から実行する)
set -eu

if [ "$(id -u)" -ne 0 ]; then
    echo "root権限で実行してください: sudo bash $0" >&2
    exit 1
fi
USER_NAME="${SUDO_USER:-}"
if [ -z "$USER_NAME" ] || [ "$USER_NAME" = "root" ]; then
    echo "SUDO_USERが取得できません。一般ユーザーからsudo経由で実行してください。" >&2
    exit 1
fi

DEST=/etc/sudoers.d/nokosahen-date
TMP=$(mktemp)
trap 'rm -f "$TMP"' EXIT
echo "$USER_NAME ALL=(root) NOPASSWD: /usr/bin/date -s *" > "$TMP"

# 文法チェックに通ったときだけ設置(壊れたsudoersはsudo自体を使えなくするため)
visudo -cf "$TMP"
install -m 0440 -o root -g root "$TMP" "$DEST"
visudo -c >/dev/null
echo "導入しました: $DEST"
cat "$DEST"
