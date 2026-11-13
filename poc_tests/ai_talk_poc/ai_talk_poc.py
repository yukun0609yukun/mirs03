"""JetsonでのAI会話文生成+音声出力 PoC(概念実証)。

Jetson上のOllama(ローカルLLM)に、食堂ロボットの「メニュー案内の一言」を
考えさせ、さらにVOICEVOXで実際に声に出して読み上げるところまでを確認する
単体テストプログラム。本番側`voice_bridge.py`の
`generate_utterance_from_context()`(文章生成)・`speak_voicevox()`
(音声合成・再生)と同じ組み立て方をそのまま使っているので、ここで良い結果が
出れば本番でも同じ組み立て方で通用する。

ROS2やmirs2603_nokosahenの他のコードには一切依存しない(voice_bridge.pyも
importしない、完全に独立したコピー)。このフォルダだけで完結するので、
このフォルダごと送ればそのまま試せる。

必要なもの:
    1. Jetson(またはPC)にOllamaをインストールし、常駐させておく
       https://ollama.com/download
    2. モデルを取得しておく:
           ollama pull gemma2:2b
    3. VOICEVOX Engineを起動しておく(音声出力を試す場合。文章生成だけなら不要)
       https://voicevox.hiroshiba.jp/
    4. pip install -r requirements.txt   (requests, sounddevice, soundfile)

実行(対話モード。何度でも状況を変えて試せる。音声で読み上げるか最初に聞かれる):
    python ai_talk_poc.py

実行(1回だけ、コマンドライン引数で状況を指定):
    python ai_talk_poc.py --phase induction --promote-item カレー --speak
"""
from __future__ import annotations

import argparse
import os
import platform
import tempfile
import time

import requests

try:
    import sounddevice as sd
    import soundfile as sf

    _VOICE_PLAYBACK_AVAILABLE = True
except Exception:  # noqa: BLE001 - sounddevice/soundfileが無い環境でも文章生成だけは試せるようにする
    _VOICE_PLAYBACK_AVAILABLE = False

OLLAMA_URL = os.environ.get("OLLAMA_URL", "http://127.0.0.1:11434/api/generate")
MODEL_NAME = os.environ.get("OLLAMA_MODEL", "gemma2:2b")
OLLAMA_NUM_PREDICT = int(os.environ.get("OLLAMA_NUM_PREDICT", "220"))
# 班員の検証で、Jetsonではモデルの初回読み込み等で60秒では足りないことがあったため300秒にした。
OLLAMA_TIMEOUT_SEC = float(os.environ.get("OLLAMA_TIMEOUT_SEC", "300"))

# 本番(voice_bridge.py)のデフォルトと同じVOICEVOX設定。
VOICEVOX_URL = os.environ.get("VOICEVOX_URL", "http://127.0.0.1:50021")
SPEAKER_ID = int(os.environ.get("VOICEVOX_SPEAKER_ID", "2"))
VOICE_OUTPUT_DEVICE = os.environ.get("VOICE_OUTPUT_DEVICE") or None
VOICE_QUERY_TUNING = {
    "speedScale": 0.97,
    "pitchScale": -0.04,
    "intonationScale": 0.88,
    "prePhonemeLength": 0.08,
    "postPhonemeLength": 0.12,
}

# 本番(persona_prompt.txt)のデフォルトと同じ口調設定。
PERSONA_PROMPT = """あなたは学校や職場の食堂で長年働いている、世話焼きで情に厚い「食堂のおばちゃん」です。
返答は必ず日本語のみで書いてください。英語や英数字の見出し、余計な注釈は入れないでください。

【必ず守ること】
- 必ず最初の一文を「お疲れ様ー、今日も来てくれてありがとねぇ。」から始め、改行してから本題(メニューのおすすめ)に入ってください。
- 渡されたメニュー一覧から今日のおすすめを1つ選び、〇〇の部分にそのメニュー名を入れて紹介してください。
- おすすめする理由を1〜2文で添えてください。理由は曜日・天気や気温・栄養バランス・気分転換・そのメニューが出る頻度が低い、など渡された情報やその場の状況に合わせて自由に選んでください。
- 「残さず食べてほしい」「お昼を抜かないでほしい」といった、食べてもらいたい気持ちを一言添えてください。
- 最後は「午後もお仕事頑張りなさい」「ゆっくりしていきなさい」のような、送り出す一言で締めてください(表現は毎回同じでなくてよい)。
- 語尾には「〜さね」「〜だよ」「〜かい?」「〜なさい」「〜まへん」など、世話焼きで少し命令口調だけど温かみのある喋り方を自然に混ぜてください。
- 文字数に厳密な上限はありません。下の参考例と同じくらいの長さ・文の数を目安にしてください。

【文体・構成の参考例(実際にこの喋り方をしています。〇〇の部分に実際のメニュー名が入ります)】

お疲れ様ー、今日も来てくれてありがとねぇ。
今日のおすすめは〇〇だよ。ヘルシーなメニューもいいけど、お昼はガッツリ食べて元気にお仕事頑張りなさい。それじゃあお仕事始まるまでゆっくりしていきなさい。

お疲れ様ー、今日も来てくれてありがとねぇ。
今日は週末だからお昼はヘルシーな○○定食にしときなさい。胃もたれでお休み潰しちゃいかんよぉ。でもお昼抜いちゃダメだからね。しっかり食べて午後もお仕事頑張りなさいよ〜。

お疲れ様ー、今日も来てくれてありがとねぇ。
月曜日なんだからカロリーの高い○○定食ーしっかり食べて、1週間の元気をチャージしなさい。お残しは許しまへんからね。

お疲れ様ー、今日も来てくれてありがとねぇ。
今日のおすすめは○○定食だよ。体にいい物いっぱい使ったから健康にいいよ。お野菜が苦手な子もいるだろうけど頑張って作ったから食べて欲しいねぇ。残さず食べてくれたらおばちゃん嬉しくて泣いちまうよ。

お疲れ様ー、今日も来てくれてありがとねぇ。
今日のおすすめは○○だよ。最近お米使った料理ばっかだったからたまにはパスタとかも食べてみてもいいんじゃないかねぇ。でも喉に詰まらせると悪いから、よく噛んでしっかり食べなさいね。それじゃあ午後もお仕事頑張りなさい。

お疲れ様ー、今日も来てくれてありがとねぇ。
今日は冷えるねぇ。体が冷えてると元気も出てこないさね。○○定食食べて体あっためなさいね。元気が出るように○○(栄養素とかスタミナ着くやつ？とか)も入ってるからね。
体温まったら、午後もお仕事頑張れるからね。ゆっくりしていきなさいねぇ。

お疲れ様ー、今日も来てくれてありがとねぇ。
今日はあっっついねぇ。食堂でいっぱい涼んでいきなさい。でも体が冷えすぎちゃうのも良くないから適度にあったまる○○定食がおすすめだよ。気温差が激しいと体調崩しやすいからねぇ。しっかりと体調管理して、これからお仕事頑張りなさいよぉ。

お疲れ様ー、今日も来てくれてありがとねぇ。
今日は○○定食(魚系)がおすすめだよ。お魚には目を良くする成分たくさん入ってるからねぇ。目が疲れてる子もいっぱい居るだろうからお魚食べて目休めなさいね。でも骨には気を付けを食べなさいね。それじゃあ午後のお仕事までゆっくりしていきなさいね。

お疲れ様ー、今日も来てくれてありがとねぇ。
今日はお肉たっぷりの○○定食がおすすめだよ。最近忙しい時期だと思うからお昼抜いちゃう子が多いんだよねぇ。でもせっかく食べに来たんだからお肉いっっぱい食べて元気だしてね。午後も行ってらっしゃい。

お疲れ様ー、今日も来てくれてありがとねぇ。
今日は○○定食なんかどうだい？滅多に出ないからこれ逃したら食べらんないよ。いつも美味しい○○定食もいいけど、たまにはこういうものありなんじゃないかな。何事も挑戦だよ。午後もお仕事頑張りなさいね。"""

# テスト用のサンプルメニュー(本番のnokosahen.yamlのannouncer設定に相当)。
DEFAULT_MENU = [
    {"name": "カレー", "merit": "スパイスが効いて食欲が出る、育ち盛りに人気の一品"},
    {"name": "唐揚げ定食", "merit": "がっつり食べたい日にぴったりのボリューム"},
    {"name": "うどん", "merit": "優しい味わいで、疲れた胃にも負担が少ない"},
]


def _print_environment_info() -> None:
    """テスト機がJetPack 22.04/24.04のどちらかを毎回表示する。

    本番機はJetPack Ubuntu 24.04、テスト機は同じJetson Orin Nanoでも
    JetPack Ubuntu 22.04と、OSのバージョンが異なる想定で使われている。
    VOICEVOX Engineのaarch64向け配布バイナリは新しめのglibcを前提に
    ビルドされていることがあり、22.04(glibc 2.35)の方が24.04(glibc 2.39)より
    古いため、22.04側でだけ"version `GLIBC_2.3x' not found"のようなエラーで
    起動に失敗する可能性がある。結果を報告する際にどちらの機体で試したか
    分かるようにしておく。
    """
    print(f"Python: {platform.python_version()}  ({platform.machine()})")
    try:
        os_release = platform.freedesktop_os_release()
        version_id = os_release.get("VERSION_ID", "?")
        print(f"OS: {os_release.get('PRETTY_NAME', '?')} (VERSION_ID={version_id})")
        if version_id not in ("24.04",):
            print(
                "   ※本番機はUbuntu 24.04想定。VOICEVOX Engineがglibc関連エラー"
                "(`version 'GLIBC_2.3x' not found`等)で起動しない場合は、"
                "このOSバージョンの違いが原因の可能性が高い(docs/setup_docker.md参照)。"
            )
    except (AttributeError, OSError):
        print("OS: 判定不可(platform.freedesktop_os_release非対応環境)")
    print("-" * 40)


def build_context_prompt(phase: str, promote_item: str | None, menu_items: list[dict]) -> str:
    """voice_bridge.pyの_build_context_prompt()と同じ組み立て方。"""
    menu_lines = "\n".join(
        f"- {m.get('name', '')}: {m.get('merit', '')}" for m in menu_items if m.get("name")
    )

    if phase == "induction" and promote_item:
        remaining_lines = "\n".join(
            f"- {m['name']}: 残り{m.get('remaining', '多め')}個(元は{m.get('initial', '?')}個)"
            for m in menu_items
        )
        instruction = (
            f"今は特に「{promote_item}」の残数に余裕があります。"
            "以下のメニュー一覧と残数状況を踏まえて、お客さんにさりげなく"
            f"「{promote_item}」をすすめる一言を考えてください。"
        )
        facts = f"<メニュー一覧>\n{menu_lines}\n\n<残数状況>\n{remaining_lines}"
    else:
        instruction = (
            "以下のメニュー一覧を踏まえて、お客さんに親しみやすく声をかける"
            "一言を考えてください。特定のメニューを無理に推す必要はありません。"
        )
        facts = f"<メニュー一覧>\n{menu_lines}"

    return f"<設定>\n{PERSONA_PROMPT}\n\n<指示>\n{instruction}\n\n{facts}\n看板娘:"


def generate(prompt: str) -> tuple[str, float]:
    """Ollamaへ問い合わせ、生成文章と所要時間(秒)を返す。"""
    payload = {
        "model": MODEL_NAME,
        "prompt": prompt,
        "stream": False,
        "options": {"num_predict": OLLAMA_NUM_PREDICT, "temperature": 0.7},
    }
    start = time.monotonic()
    res = requests.post(OLLAMA_URL, json=payload, timeout=OLLAMA_TIMEOUT_SEC)
    res.raise_for_status()
    elapsed = time.monotonic() - start
    text = res.json()["response"].strip()
    return text, elapsed


def speak_voicevox(text: str) -> None:
    """VOICEVOXで音声合成し、その場で再生する(voice_bridge.pyのspeak_voicevox()と同じ手順)。"""
    if not _VOICE_PLAYBACK_AVAILABLE:
        print("!! sounddevice/soundfileが無いため再生できません(pip install -r requirements.txt)")
        return
    try:
        query_res = requests.post(
            f"{VOICEVOX_URL}/audio_query",
            params={"text": text, "speaker": SPEAKER_ID},
            timeout=10,
        )
        query_res.raise_for_status()
        query_data = query_res.json()
        query_data.update(VOICE_QUERY_TUNING)

        synth_res = requests.post(
            f"{VOICEVOX_URL}/synthesis?speaker={SPEAKER_ID}",
            json=query_data,
            timeout=30,
        )
        synth_res.raise_for_status()

        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tf:
            tf.write(synth_res.content)
            wav_path = tf.name

        try:
            data, samplerate = sf.read(wav_path, dtype="float32")
            print("🔊 再生中...")
            sd.play(data, samplerate, device=VOICE_OUTPUT_DEVICE)
            sd.wait()
        finally:
            os.remove(wav_path)
    except Exception as exc:  # noqa: BLE001
        print(f"❌ VOICEVOX合成/再生に失敗しました: {exc}")
        print(f"   {VOICEVOX_URL} でVOICEVOX Engineが起動しているか確認してください。")


def run_once(phase: str, promote_item: str | None, speak: bool) -> None:
    prompt = build_context_prompt(phase, promote_item, DEFAULT_MENU)
    print("\n--- 送信プロンプト ---")
    print(prompt)
    print("--- 生成中... ---")
    try:
        text, elapsed = generate(prompt)
    except Exception as exc:  # noqa: BLE001
        print(f"❌ Ollamaへの接続に失敗しました: {exc}")
        print(f"   {OLLAMA_URL} でOllamaが起動しているか、モデル({MODEL_NAME})を"
              "取得済みか確認してください(ollama pull " + MODEL_NAME + ")")
        return
    print(f"\n--- 生成結果(所要{elapsed:.1f}秒) ---")
    print(text)
    print("-" * 40)
    if speak:
        speak_voicevox(text)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="AI会話文生成PoC")
    parser.add_argument(
        "--phase", choices=["initial", "induction"], default=None,
        help="initial=まだ差が出ていない段階/induction=特定メニューを推す段階。"
             "指定しなければ対話モードで毎回聞く。",
    )
    parser.add_argument(
        "--promote-item", default=None,
        help="induction時にすすめるメニュー名(例: カレー)",
    )
    parser.add_argument(
        "--speak", action="store_true",
        help="生成した文章をVOICEVOXで実際に読み上げる(既定は文章表示のみ)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    _print_environment_info()
    print(f"OLLAMA_URL={OLLAMA_URL}  MODEL={MODEL_NAME}")
    if not _VOICE_PLAYBACK_AVAILABLE:
        print("(sounddevice/soundfile未導入のため、音声読み上げは使えません。文章生成のみ試せます)")

    if args.phase is not None:
        run_once(args.phase, args.promote_item, args.speak)
        return

    print("対話モードです。毎回、状況を聞いてから生成します(Ctrl+Cで終了)。")
    speak_default = False
    if _VOICE_PLAYBACK_AVAILABLE:
        answer = input(f"VOICEVOXで読み上げますか？(y/N、{VOICEVOX_URL}が起動している前提): ").strip().lower()
        speak_default = answer in ("y", "yes")
    try:
        while True:
            phase = input("\nphase(initial/induction、空でinitial): ").strip() or "initial"
            promote_item = None
            if phase == "induction":
                promote_item = input("すすめるメニュー名(例: カレー): ").strip() or "カレー"
            run_once(phase, promote_item, speak_default)
    except KeyboardInterrupt:
        print("\n終了します。")


if __name__ == "__main__":
    main()
