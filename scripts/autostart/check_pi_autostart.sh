#!/usr/bin/env bash
# ラズパイで実行する「腹部ディスプレイが、電源投入で自動起動する設定か」の確認(2026-10-19追加)。読み取りだけ。docs/autostart.md
#   bash check_pi_autostart.sh
# 1台目に入って `ssh pi` したあとでも、パソコンから `ssh pi@198.51.100.1` で入ってもよい。
echo "== 1) start_kiosk.sh を呼ぶ自動起動の設定(見つかった場所を表示)"
found=0
for f in "$HOME/.config/labwc/autostart" /etc/xdg/labwc/autostart "$HOME/.config/wayfire.ini" "$HOME/.config/lxsession/LXDE-pi/autostart" "$HOME/.config/autostart"/*.desktop /etc/xdg/autostart/*.desktop; do
  [ -f "$f" ] && grep -l "start_kiosk" "$f" 2>/dev/null && { echo "  → $f:"; grep -n "start_kiosk" "$f" | sed 's/^/      /'; found=1; }
done
crontab -l 2>/dev/null | grep -n "start_kiosk" && { echo "  → crontab"; found=1; }
systemctl list-unit-files 2>/dev/null | grep -i "kiosk" && found=1
systemctl --user list-unit-files 2>/dev/null | grep -i "kiosk" && found=1
[ "$found" = "1" ] && echo "  OK: 自動起動の設定があります" || echo "  NG: start_kiosk.sh を自動起動する設定が見つかりません(下の対処を参照)"

echo "== 2) 必要なファイル"
for f in "$HOME/start_kiosk.sh" "$HOME/switch_kiosk.sh" "$HOME/kiosk_proxy.py" "$HOME/kiosk_admin.py" "$HOME/survey/receiver.py" "$HOME/.kiosk_env"; do
  [ -e "$f" ] && echo "  あり: $f" || echo "  なし: $f"
done

echo "== 3) いま動いているか"
echo "  start_kiosk: $(pgrep -fc 'start_kio[s]k.sh') 件   アンケート受信(8082): HTTP $(curl -s -m 3 -o /dev/null -w '%{http_code}' http://127.0.0.1:8082/state)   管理API(8083): HTTP $(curl -s -m 3 -o /dev/null -w '%{http_code}' http://198.51.100.1:8083/state)"
echo "  表示モード: $(cat "$HOME/.kiosk_mode" 2>/dev/null || echo '(未設定=お客さん向け)')"

cat <<'EOF'

== 対処(NG のとき)
ラズパイOS(labwc)なら、次の1行を ~/.config/labwc/autostart に追記すると、起動時に腹部画面が自動で立ち上がる:
    /home/pi/start_kiosk.sh &
(ファイルが無ければ作る。追記後に再起動して、画面が自動で出るか確かめる。)
EOF
