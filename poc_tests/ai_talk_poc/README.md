# AI会話文生成+音声出力 PoC(Jetson想定)

**目的**: Jetson上のOllama(ローカルLLM)が、食堂ロボットの「メニュー案内の一言」を
実際にどれくらいの品質・速度で生成できるか、さらにVOICEVOXで実際に声に出して
読み上げるところまでを確認するための概念実証(PoC)です。本番の`voice_bridge.py`と
同じプロンプトの組み立て方・音声合成の手順をそのまま使っているので、ここで良い
結果が出れば本番でも通用します。このフォルダだけで完結していて、ROS2や
プロジェクト本体には一切依存しません。**このフォルダごとそのまま送れば、
受け取った人はこれだけで試せます。**

文章生成だけ試したい場合はVOICEVOXが無くても動きます(読み上げをスキップするだけ)。

## 本番機とテスト機でJetPackのバージョンが違う点について

本番で使うJetson Orin NanoはJetPack Ubuntu 24.04ですが、この音声PoCを
試すテスト機は同じJetson Orin Nanoの機種でもJetPack Ubuntu 22.04です。
OllamaもPython側の依存関係(`requests`/`sounddevice`/`soundfile`)もOSの
バージョンに依存しないため、これ自体は問題になりません。

一点だけ注意が必要なのが**VOICEVOX Engine**です。aarch64 Linux向けの配布
バイナリが新しめのglibcを前提にビルドされている場合、22.04(glibc 2.35)は
24.04(glibc 2.39)より古いため、22.04のテスト機でだけ
`version 'GLIBC_2.3x' not found` のようなエラーで起動に失敗する可能性が
あります。`python ai_talk_poc.py`実行時に毎回OSバージョンを表示するように
してあるので、結果を報告する際はこの表示も一緒に共有してください。もし
テスト機でVOICEVOX Engineが起動しない場合は、文章生成(Ollama)側の結果
だけでも先に報告してもらえれば十分です(`docs/setup_docker.md`のトラブル
シュートも参照)。

## 準備

1. Jetson(または手元のPC)にOllamaをインストールする: https://ollama.com/download
2. モデルを取得する:
   ```bash
   ollama pull gemma2:2b
   ```
3. VOICEVOX Engineをインストール・起動する(音声出力を試す場合。文章生成だけなら不要):
   https://voicevox.hiroshiba.jp/
4. 依存パッケージを入れる:
   ```bash
   pip install -r requirements.txt
   ```
   (`sounddevice`はPortAudioに依存するため、Linuxで入らない場合は
   `sudo apt install libportaudio2`を先に試す)

## 実行

対話モード(状況を変えながら何度でも試せる、おすすめ)。最初に読み上げるか聞かれる:

```bash
python ai_talk_poc.py
```

`phase`(initial/induction)と、induction時は`すすめるメニュー名`を聞かれるので入力する。

1回だけ実行(スクリプトから呼びたい場合、`--speak`を付けると読み上げまで行う):

```bash
python ai_talk_poc.py --phase induction --promote-item カレー --speak
```

## 見るべきポイント

- **文章生成の速度**: Ollamaの生成にかかった秒数(結果と一緒に表示される)。
  Jetsonの実機で、実運用に耐える速さ(数秒以内)で返ってくるか
- **文章の品質**: 「食堂のおばちゃん」の口調になっているか、不自然な英語や
  見出しが混ざっていないか、参考例と同じくらいの長さ・構成になっているか
- **`induction`時に、ちゃんと指定したメニューをさりげなく勧める文章になるか**
- **音声合成・再生の速度**: VOICEVOXの合成にかかる時間、実際に聞いた印象
  (速度・音程・イントネーションの設定は本番`voice_bridge.py`と同じ値を使っている)
- Ollamaが起動していない/モデル未取得の場合はエラーメッセージが出るので、
  その場合は`ollama pull gemma2:2b`とOllama本体の起動を確認する
- VOICEVOXに繋がらない場合もエラーメッセージが出る(文章生成自体は続行できる)

## 環境変数(必要なら)

- `OLLAMA_URL`(既定 `http://127.0.0.1:11434/api/generate`)
- `OLLAMA_MODEL`(既定 `gemma2:2b`。他のモデルで試したい場合)
- `OLLAMA_NUM_PREDICT`(既定 `220`。生成する最大トークン数、長さの目安)
- `OLLAMA_TIMEOUT_SEC`(既定 `300`。Ollamaの応答を待つ秒数。Jetsonでモデルの初回読み込みが遅い場合向け)
- `VOICEVOX_URL`(既定 `http://127.0.0.1:50021`)
- `VOICEVOX_SPEAKER_ID`(既定 `2`)
- `VOICE_OUTPUT_DEVICE`(既定は空でOS既定の出力デバイスを使う。`scripts/setup_udev_rules.sh`が
  案内する`/dev/snd/by-id/...`名、または`python3 -m sounddevice`で確認したデバイス名/インデックス)

## 本番との対応

このPoCで使っているプロンプトの組み立て方(`build_context_prompt`)と音声合成の
手順(`speak_voicevox`)は、本番の`voice_bridge.py`の`_build_context_prompt()`・
`speak_voicevox()`と同じものです。ここでチューニングが必要だと分かった場合
(口調がずれる・遅すぎる・声質が合わない等)は、`voice_bridge.py`側の
`persona_prompt.txt`やOllama/VOICEVOXの設定にフィードバックしてください。
