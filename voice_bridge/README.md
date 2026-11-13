# voice_bridge

看板娘ロボットの音声出力サービス。**Dockerコンテナの外、Jetsonの素のOS上で常駐させる**（音声再生をコンテナ内でやろうとすると`/dev/snd`パススルー等が煩雑になるため。詳細は`docs/setup_docker.md`）。

`kanbanmusume`（`Downloads\kanbanmusume\kanbanmusume\main.py`）のVOICEVOX合成・Ollama生成ロジックを移植したもの。再生のみ`winsound`から`sounddevice`（Linux/aarch64向け）に差し替えている。

## セットアップ

```bash
python3 -m pip install -r requirements.txt
```

VOICEVOX Engine・Ollamaが同じマシン上で起動している前提（`127.0.0.1`）。

## 実行

```bash
python3 voice_bridge.py
```

環境変数で調整可能:

| 変数 | 既定値 | 説明 |
|---|---|---|
| `VOICEVOX_URL` | `http://127.0.0.1:50021` | VOICEVOX Engine |
| `OLLAMA_URL` | `http://127.0.0.1:11434/api/generate` | Ollama |
| `VOICEVOX_SPEAKER_ID` | `27` | 話者ID(2026-10-30に 2 から変更。Jetson2で班員が高専祭用に変えた値。docs/jetson2_team_changes.md) |
| `OLLAMA_MODEL` | `gemma2:2b` | Ollamaモデル名 |
| `VOICE_OUTPUT_DEVICE` | (OSデフォルト) | `scripts/setup_udev_rules.sh`で確認したUSB音声アダプタの固定名 |
| `VOICE_BRIDGE_PORT` | `8100` | 待受ポート |
| `VOICE_MAX_SENTENCES` | `2` | Ollamaが作った発話を先頭何文までにするか(0以下=切らない。固定文・フォールバック文は対象外) |

`persona_prompt.txt` を編集すると、AI生成時の口調・キャラクター設定を、コードを触らずに変更できる。

現在の `persona_prompt.txt` は高専祭用の「のこさへん」ばあちゃん(班員作成)。旧版(食堂のおばちゃん)は `persona_prompt_foodhall_backup.txt`。
声のパラメータ(`VOICE_QUERY_TUNING`: 速度0.76・ピッチ-0.09・抑揚2)も高専祭用に変更済み(旧: 0.97 / -0.04 / 0.88)。

## 発話内容の作られ方

`nokosahen_voice`(`generate:true`が既定)から送られてくる`context`には、メニュー一覧
(+完成版で残数差が出ている間は残数状況)と、Ollamaが使えない場合のルールベースの
フォールバック文が入っている。voice_bridgeはこの`context`を元に、`persona_prompt.txt`の
口調でOllamaに実際の発話文面を1から考えさせる(`generate_utterance_from_context`)。

- 残数カウント抜き(Stage2)の間は、メニュー一覧+メリット文だけを渡すので、AIは
  「今あるメニューについて親しみやすく話す」文面を考える。
- 完成版で決済カメラ/手動カウントにより残数の偏りが検知されると、それ以降は
  残数状況も渡されるので、AIは特定のメニューをさりげなく推す文面を考える。
- Ollama未導入・応答失敗・空応答のときは、必ずフォールバック文をそのまま話す
  (発話自体が止まることはない)。

`generate:false`にする、または`context`無しで`text`だけ送れば、AIを介さず
渡された文面をそのまま(または口調だけ言い換えて)話す従来方式に戻せる。

## API

```
POST /speak
  { "context": {"phase": "induction", "menu_items": [{"name": "からあげ定食", "merit": "..."}],
                "promote_item": "からあげ定食",
                "remaining_status": [{"name": "からあげ定食", "remaining": 20, "initial": 30}],
                "fallback_text": "本日はからあげ定食にまだ余裕があります。"},
    "generate": true }
  -> menu_items等からAIに文面を考えさせて話す(既定の使い方)

  { "text": "からあげ定食まだまだあるよ", "generate": false }
  -> 従来方式。textをそのまま話す

  -> 202 Accepted（キューに積んで即座に返す。合成・再生は非同期）

GET /status
  -> { "speaking": true, "queue_length": 1 }
```

## systemdサービス化

```ini
# /etc/systemd/system/voice-bridge.service
[Unit]
Description=nokosahen voice bridge (VOICEVOX + Ollama)
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/jetson/mirs2603_nokosahen/voice_bridge/voice_bridge.py
Restart=always
User=jetson

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable --now voice-bridge.service
```
