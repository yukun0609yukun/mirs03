#!/usr/bin/env bash
# 1台目Jetsonだけで「自立走行テスト」を行うための起動スクリプト(2026-10-09追加)。
#   bash ~/mirs2603_nokosahen/scripts/start_nav_test.sh
# 【2026-10-21】`start_all.sh nav_test` の別名に置き換えた(同じ構成: 手動のみ・総合ダッシュボードあり・アンケート監視なし・ROS通信は
# 有線NICに絞らない・人物検出なし)。以前は `stop_everything.sh | tail` で終了コードが失われ、停止に失敗しても、そのまま起動して
# 二重起動になっていた。start_all.sh は、停止に失敗したら起動を中止する。引数は、そのまま start_all.sh へ渡す(例: --ocr)。
# 起動前に、動作中のロボット一式とOCRを止める。地図を作成中(未保存)なら、先に保存してから実行すること。
exec bash "$HOME/mirs2603_nokosahen/scripts/start_all.sh" nav_test "$@"
