#!/usr/bin/env bash
# Jetson #2 の音声スタック(Ollama + VOICEVOX Engine + voice_bridge)を起動する。sudo不要・冪等。
#   使い方: bash ~/start_voice_stack.sh        (起動済みのものは何もしない)
#   停止:   bash ~/stop_voice_stack.sh   (voice_bridgeだけ。Ollama/VOICEVOXも止めるなら --all)
# 起動順: ollama serve -> docker voicevox -> voice_bridge(0.0.0.0:8100)
# BGM: ~/bgm の音楽(wav/flac/ogg/mp3)をvoice_bridgeが流し、発話中は自動で音量を下げる(BGM_*で調整、POST /bgmで実行中も変更可)。
# 音量(2026-10-09): 発話倍率/BGM通常/BGM発話中は ~/.nokosahen_voice_levels.json(ダッシュボードの「音量」カード or
#   POST /voice_levels)が最優先。ファイルが無いときだけ環境変数 SPEECH_VOLUME/BGM_VOLUME/BGM_DUCK_VOLUME、
#   それも無ければ voice_bridge.py の既定値(docs/voice_levels.md)。ここでは既定値を固定しない。
# ログ: ~/ollama_serve.log, ~/voice_bridge.log (voicevoxは `docker logs voicevox`)
#
# 構成の前提(2026-10-02):
#   - Ollamaは公式tarball(ollama-linux-arm64.tar.zst, cuda_v13同梱)を ~/ollama に展開したもの。
#   - VOICEVOXは docker の restart=unless-stopped なので、dockerデーモン起動時に自動で上がる。
#   - voice_bridgeは ~/mirs2603_nokosahen/voice_bridge/voice_bridge.py を ~/nokosahen_venv で実行。
#     出力デバイスは ~/.nokosahen_audio.env の VOICE_OUTPUT_DEVICE(カード番号は再起動で入れ替わる
#     ことがあるので番号ではなく名前で指定する)。
set -u

OLLAMA_BIN="${OLLAMA_BIN:-$HOME/ollama/bin/ollama}"
VENV="${VENV:-$HOME/nokosahen_venv}"
BRIDGE="${BRIDGE:-$HOME/mirs2603_nokosahen/voice_bridge/voice_bridge.py}"
AUDIO_ENV="$HOME/.nokosahen_audio.env"
MODEL="${OLLAMA_MODEL:-gemma2:2b}"

log() { echo "[start_voice_stack $(date +%H:%M:%S)] $*"; }

# --- PipeWire/PulseAudioのユーザーセッションが居れば使う(無ければ待たずに素のALSAで進む) ---
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
for _ in $(seq 1 30); do
  [ -S "$XDG_RUNTIME_DIR/pipewire-0" ] && break
  sleep 2
done
[ -S "$XDG_RUNTIME_DIR/pipewire-0" ] && log "pipewire socket あり" || log "pipewire socket なし(ALSA直接で再生を試みる)"

# --- 1) Ollama ---
if curl -s -m 3 http://127.0.0.1:11434/api/version >/dev/null; then
  log "ollama: 起動済み"
else
  log "ollama: 起動します"
  OLLAMA_HOST=127.0.0.1:11434 OLLAMA_KEEP_ALIVE=-1 \
    setsid nohup "$OLLAMA_BIN" serve >>"$HOME/ollama_serve.log" 2>&1 </dev/null &
  for _ in $(seq 1 30); do
    curl -s -m 2 http://127.0.0.1:11434/api/version >/dev/null && break
    sleep 1
  done
fi
# モデルを事前にメモリへ載せ、本番相当の長さのプロンプトで1回走らせておく(初回のモデル読み込み
# 約75秒と、初めて長いプロンプトを処理するときのCUDAカーネルJIT(約40秒)を、本番の最初の発話ではなく
# ここで済ませる)。失敗しても続行。クライアントが途中で切断すると読み込みが中断されるので長めに待つ。
# 他の起動を待たせないよう、最後にバックグラウンドで実行する。
WARM_PROMPT="あなたは食堂のおばちゃんです。日本語のみで話してください。$(printf 'メニュー一覧: カレー スパイスが効いて食欲が出る。唐揚げ定食 がっつり食べたい日にぴったり。うどん 優しい味わい。%.0s' 1 2 3 4 5)お客さんにカレーをすすめる一言を考えてください。看板娘:"
warm_ollama() {
  curl -s -m 400 http://127.0.0.1:11434/api/generate   -d "{\"model\":\"$MODEL\",\"prompt\":\"$WARM_PROMPT\",\"stream\":false,\"keep_alive\":-1,\"options\":{\"num_predict\":8}}" >/dev/null 2>&1   && log "ollama: $MODEL をプリロード/ウォームアップ完了" || log "ollama: プリロード失敗(続行)"
}

# --- 2) VOICEVOX Engine (docker) ---
if curl -s -m 3 http://127.0.0.1:50021/version >/dev/null; then
  log "voicevox: 起動済み"
else
  if docker ps -a --format '{{.Names}}' | grep -qx voicevox; then
    log "voicevox: 既存コンテナを start"
    docker start voicevox >/dev/null
  else
    log "voicevox: コンテナを新規作成"
    docker run -d --name voicevox --restart unless-stopped \
      -p 127.0.0.1:50021:50021 voicevox/voicevox_engine:cpu-latest >/dev/null
  fi
  for _ in $(seq 1 60); do
    curl -s -m 2 http://127.0.0.1:50021/version >/dev/null && break
    sleep 2
  done
fi

# --- 3) voice_bridge ---
if curl -s -m 3 http://127.0.0.1:8100/status >/dev/null; then
  log "voice_bridge: 起動済み"
else
  log "voice_bridge: 起動します"
  # shellcheck disable=SC1090
  [ -f "$AUDIO_ENV" ] && . "$AUDIO_ENV"
  # PipeWireが居るときは sounddevice の "pipewire" デバイスを使う(既定シンク=USBスピーカー、
  # 24kHzのままでもPipeWireがリサンプルする。USBのhwを直接掴むと Invalid sample rate /
  # Device busy になる)。居ないとき(未ログインでlinger無し)は audio.env の名前指定で、
  # voice_bridge側のリサンプル再試行に任せる。VOICE_OUTPUT_DEVICE_FORCE で上書き可。
  if [ -n "${VOICE_OUTPUT_DEVICE_FORCE:-}" ]; then
    VOICE_OUTPUT_DEVICE="$VOICE_OUTPUT_DEVICE_FORCE"
  elif [ -S "$XDG_RUNTIME_DIR/pipewire-0" ]; then
    VOICE_OUTPUT_DEVICE="pipewire"
  fi
  export VOICE_OUTPUT_DEVICE="${VOICE_OUTPUT_DEVICE:-}"
  VOICE_OUTPUT_DEVICE="$VOICE_OUTPUT_DEVICE" \
  VOICE_BRIDGE_HOST="${VOICE_BRIDGE_HOST:-0.0.0.0}" \
  VOICE_BRIDGE_PORT=8100 \
  OLLAMA_TIMEOUT_SEC="${OLLAMA_TIMEOUT_SEC:-60}" \
  OLLAMA_KEEP_ALIVE="${OLLAMA_KEEP_ALIVE:--1}" \
  BGM_DIR="${BGM_DIR:-$HOME/bgm}" \
  BGM_ENABLED="${BGM_ENABLED:-1}" \
  BGM_HOLD_SEC="${BGM_HOLD_SEC:-1.5}" \
    setsid nohup "$VENV/bin/python" "$BRIDGE" >>"$HOME/voice_bridge.log" 2>&1 </dev/null &
  for _ in $(seq 1 20); do
    curl -s -m 2 http://127.0.0.1:8100/status >/dev/null && break
    sleep 1
  done
fi

# ウォームアップは裏で(約2分かかることがある。完了は start_voice_stack.log か `ollama ps` で確認)
warm_ollama >>"$HOME/ollama_warmup.log" 2>&1 </dev/null &

log "状態:"
curl -s -m 3 http://127.0.0.1:11434/api/version | sed 's/^/  ollama   /'; echo
curl -s -m 3 http://127.0.0.1:50021/version | sed 's/^/  voicevox /'; echo
curl -s -m 3 http://127.0.0.1:8100/status | sed 's/^/  bridge   /'; echo
exit 0
