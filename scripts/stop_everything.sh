#!/usr/bin/env bash
# 1台目Jetsonで実行する「全部止める」スクリプト(2026-10-09追加)。
#   bash ~/mirs2603_nokosahen/scripts/stop_everything.sh          # 1台目のロボット(ROS一式)とOCRを止める
#   bash ~/mirs2603_nokosahen/scripts/stop_everything.sh --all    # 上に加えて、2台目の音声(voice_bridge/BGM)とCoralサーバーも止める
# 止めないもの: ラズパイの画面(Chromium)、2台目のOllama/VOICEVOX(--allでもOllama/VOICEVOXは止めない。voice_bridgeだけ)、Wi-Fiテザリング・SSH。
# 先に監視ループ(run_persistent_base_supervised.sh)を止める。これを止めないと、ROSを止めても数秒で自動再起動される。
# 注意: 地図を作成中(未保存)なら、先にダッシュボードで保存して成功を確認してから実行すること。
set -u
ALL=0; [ "${1:-}" = "--all" ] && ALL=1

echo "== 1) 監視ループ(自動再起動)を止める"
for p in $(pgrep -f 'run_persistent_base_superv[i]sed'); do kill "$p" 2>/dev/null && echo "  killed supervisor $p"; done

echo "== 2) ロボット(ROS一式)を止める"
# `コマンド | tail` だと終了コードがtailのものになり失敗を見逃す → 出力を変数に受けて終了コードを見る。失敗は最後の終了コードに反映する
FAIL=0
out=$(bash "$HOME/mirs2603_nokosahen/scripts/stop_robot.sh" 2>&1); rc=$?
echo "$out" | tail -n 3
[ "$rc" = "0" ] || { echo "  (stop_robot.sh が失敗 rc=$rc)"; FAIL=1; }

# 意図した停止なので、ソフト緊急停止の保存状態を消す(次の起動は、通常どおり「解除」で始まる)。
rm -f "$HOME/.nokosahen_soft_estop.json" "$HOME/.nokosahen_soft_estop.json.tmp"

echo "== 3) タブレットOCRを止める"
for p in $(pgrep -f 'tablet_ocr_read[e]r.py'); do kill "$p" 2>/dev/null && echo "  killed OCR $p"; done

if [ "$ALL" = "1" ]; then
  echo "== 4) 2台目の音声(voice_bridge/BGM)を止める"
  out=$(timeout 40 ssh -o ConnectTimeout=6 -o BatchMode=yes j2 'bash ~/stop_voice_stack.sh' 2>&1); rc=$?
  echo "$out" | tail -n 4
  [ "$rc" = "0" ] || { echo "  (2台目の音声を止められず rc=$rc。2台目に届かない、または停止に失敗)"; FAIL=1; }
  echo "== 5) Coralサーバー(Docker)を止める"
  out=$(timeout 30 docker stop nokosahen-coral-server 2>&1); rc=$?
  echo "$out" | tail -n 1
  [ "$rc" = "0" ] || { echo "  (Coralサーバーを止められず rc=$rc)"; FAIL=1; }
fi

sleep 3
echo "== 確認"
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null; export ROS_DOMAIN_ID=77
echo "  ROSノード数: $(timeout 12 ros2 node list 2>/dev/null | wc -l) (0なら停止完了)"
echo "  OCR: $(pgrep -fc 'tablet_ocr_read[e]r.py')  監視ループ: $(pgrep -fc 'run_persistent_base_superv[i]sed')"
echo "  モーター指令の残り(5秒): $(timeout 5 ros2 topic echo /cmd_vel --field linear.x 2>/dev/null | sort -u | head -2 | tr '\n' ' ')"
if [ "$FAIL" = "1" ]; then echo "停止処理は完了しましたが、失敗した手順があります(上の「(…失敗」を確認)。"; exit 1; fi
echo "停止処理を完了しました。"
