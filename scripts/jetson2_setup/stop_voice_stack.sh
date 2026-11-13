#!/usr/bin/env bash
# Jetson #2 の音声スタックを止める(sudo不要)。
#   bash ~/stop_voice_stack.sh         # voice_bridge(8100。BGM込み)だけ止める。OllamaとVOICEVOXは動かしたまま
#   bash ~/stop_voice_stack.sh --all   # 上に加えて、Ollama(11434)とVOICEVOX(dockerのvoicevox。stopのみ、削除しない)も止める
# 【2026-11-05】以前は引数なしでOllama/VOICEVOXまで止めていたが、stop_everything.sh --all・shutdown_all.sh(管理画面の「音声も止める」)・
# docs/system_shutdown.md は「voice_bridgeだけ止める。Ollama/VOICEVOXは止めない」としていたので、既定をそちらに揃えた。
# ※ プロセス名での pkill -f は使わず、ポートの待受PIDから止める(自分自身を巻き込まないため)。
# 注意: voicevoxコンテナを止めても docker のデーモン再起動では
#       "unless-stopped" により再度は上がらない(手動 stop したため)。再開は start_voice_stack.sh。
kill_port() {
  local port="$1" pid
  pid=$(ss -ltnp 2>/dev/null | awk -v p=":$port" '$4 ~ p"$" {print $NF}' | grep -o 'pid=[0-9]*' | head -1 | cut -d= -f2)
  if [ -n "$pid" ]; then echo "port $port pid=$pid を停止"; kill "$pid"; else echo "port $port: 待受なし"; fi
}
kill_port 8100
if [ "${1:-}" = "--all" ]; then
  kill_port 11434
  docker stop voicevox 2>/dev/null && echo "voicevox 停止" || echo "voicevox: コンテナ無し/停止済み"
else
  echo "Ollama/VOICEVOX は止めません(止めるには --all)"
fi
