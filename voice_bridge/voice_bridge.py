#!/usr/bin/env python3
"""看板娘ロボットの音声出力サービス(Jetsonのホスト側、Dockerコンテナの外で常駐)。

`Downloads/kanbanmusume/kanbanmusume/main.py` の `speak_voicevox()` /
`ask_local_ai()` / `build_system_prompt()` を移植したもの。
違いは以下:
  - 再生を `winsound`(Windows専用)から `sounddevice`+`soundfile`(Linux/aarch64)に変更。
  - 会話ループではなく、ROS2側(`nokosahen_voice`ノード)からのHTTPリクエストを
    受けて発話するだけの単方向サービスにした。
  - 発話内容の作り方は2通り:
      (a) `context`(メニュー一覧+あれば残数状況+フォールバック文)を渡すと、
          Jetson内のOllamaに`persona_prompt.txt`の口調で文面そのものを
          1から考えさせる(`generate_utterance_from_context`)。これが今の
          主経路(behavior_director/nokosahen_voiceがこの形で送ってくる)。
      (b) 従来通り`text`(既に決まった文面)だけを渡し、`generate=true`なら
          Ollamaにその文面を口調だけ言い換えさせる(`generate_with_ollama`)。
    どちらもOllama未導入/応答失敗時はfallback_text(またはtext)をそのまま話す。

起動:
    python3 voice_bridge.py

エンドポイント:
    POST /speak
        {"context": {"phase":.., "menu_items":[{"name":..,"merit":..}],
                      "promote_item":.. (induction時のみ), "remaining_status":.. (induction時のみ),
                      "fallback_text":..}, "generate": true}
            -> menu_items等からAIに文面を考えさせて話す(既定の使い方)
        {"text": "...", "generate": false}
            -> 従来方式。textをそのまま(generate=trueなら言い換えて)話す
        -> 202 Accepted (キューに積むだけで即座に返す。合成・再生は非同期)
    GET  /status
        -> {"speaking": bool, "queue_length": int, "bgm_enabled": bool, "bgm_playing": str, "bgm_gain": float}
    GET  /utterances?since=&limit=
        -> 発話履歴。timestamp/textに加え、【2026-10-06】計測列(llm_ms/tts_ms/play_ms/queue_wait_ms/
           total_ms/used_menu_names/used_remaining/fallback_reason/model/...、古い行はnull)を追加で返す。
    GET  /voice_settings, POST /voice_settings
        -> 【2026-10-06】呼びかけ音声の設定(プロンプト・メニュー名/残数を使うか・temperature等)。
           設定ファイル(VOICE_SETTINGS_FILE、既定~/.nokosahen_voice_settings.json) > 環境変数 > 既定値。
           保存すると再起動なしで次の発話から反映。POSTは部分更新(値null=既定に戻す)、不正は400。
    POST /voice_preview
        {"context": {...}, "settings": {...未保存の設定...}} -> 話さず・履歴にも残さず文面だけ生成(ドライラン)。
    GET  /voice_metrics?limit=N | ?id=行id
        -> 発話ごとの計測値の行と統計(件数/平均/中央値/p95/最大)。?id=はプロンプト全文つき1件。
    (/speak /bgm /status と同様、上の3つもLAN内に無認証。ダッシュボード経由で使うこと。)
    GET  /voice_master, POST /voice_master {"enabled": bool}
        -> 【2026-10-09】音声の「全体ON/OFF」マスタースイッチ(docs/voice_master.md)。OFFの間は
           発話(合成・再生)・Ollamaによる文面生成・BGMのすべてを止める。状態は
           VOICE_MASTER_FILE(既定~/.nokosahen_voice_master.json)に保存され再起動後も維持(既定ON)。
           OFF中の /speak は {"ok":true,"muted":true} を200で返して何もしない(キューに積まない)。
           /status にも voice_enabled が入る。enabledはtrue/falseのみ受け付ける(400)。
    GET  /voice_levels, POST /voice_levels {"speech_volume":2.5,"bgm_volume":1.0,"bgm_duck_volume":0.15}
        -> 【2026-10-09】発話音量(倍率0〜5)とBGM音量(通常0〜5、発話中のダッキング0〜通常値。1.0超はリミッタで歪みを抑える)を別々に設定。
           部分更新可(値nullで既定に戻す)、型・範囲・「ダッキング<=通常」の違反は400。実行中に反映
           (発話は次の発話から)し、VOICE_LEVELS_FILE(既定~/.nokosahen_voice_levels.json)に保存して
           再起動後も維持。優先順位は ファイル > 環境変数(SPEECH_VOLUME/BGM_VOLUME/BGM_DUCK_VOLUME) > 既定値。
           音声マスター(/voice_master)のON/OFFとは独立(OFF中も保存でき、ONに戻すと反映)。docs/voice_levels.md
    POST /announce {"id":"exploring","text":"探索中です","volume":1.0,"urgent":false}
    POST /announce_loop {"id","text","interval_sec":2〜300,"volume":0〜5,"enabled":true|false}
    POST /announce_loop {"id","parts":[{"text"(400字まで),"volume"}],"gap_sec":1〜600,"interval_sec":2〜3600,"enabled":true|false}
    POST /announce_prepare {"texts":[...最大8個],"start":true,"force":false}
        -> 【2026-10-30】文化祭の出店紹介(docs/announcements.md)。partsを渡すと「プレイリスト」: パートを順に1つずつ、パート間gap_sec秒、
           一巡したあとinterval_sec秒あけて最初から繰り返す。**再生中は合成しない**(保存済みのwavだけ。無いパートは飛ばして未作成を記録)。
           wavは /announce_prepare で事前に1回だけ合成して ~/.nokosahen_announce_cache/ へ保存(背景で1本ずつ。長い文は文ごとに分けて合成してつなぐ)。
           start=falseは状態を見るだけ。/announce に "cached_only":true を付けると保存済みの長い文章(400字)を1回だけ鳴らす(試し再生)。
    GET  /announce_state
        -> 【2026-10-13】状況アナウンス(docs/announcements.md)。textを1回合成してキャッシュし(~/.nokosahen_announce_cache/)、
           /announce は1回だけ、/announce_loop はinterval_sec秒間隔で繰り返し再生する(同じidへの再送=「生きている」信号。
           VOICE_ANNOUNCE_KEEPALIVE_SEC(既定15)秒途絶えたら自動停止、enabled=falseで停止)。発話を優先(発話中は鳴らさない)、
           BGMは発話と同じくダッキング、マスターOFFなら無音。音量は /voice_levels の announce_volume(0〜5)×volume(0〜5、最大5倍で頭打ち)。
    GET  /bgm
        -> BGM(背景音楽)の状態。{"enabled","playing","volume","duck_volume","gain","ducked","files"}
    POST /bgm
        {"enabled": bool, "volume": 0..5, "duck_volume": 0..5}  (指定した項目だけ反映、範囲外は丸める)
        volume/duck_volume は /voice_levels の bgm_volume/bgm_duck_volume と同じ値(互換のため残してある。
        変更は保存され、ダッキングが通常音量を超えるときはダッキングを通常音量に揃える)。
        -> BGM_DIR(既定~/bgm)の音楽を流し続け、発話中だけ音量を自動で下げる(ダッキング)。
           /speak と同様LAN内に無認証で開いている。
"""
from __future__ import annotations

import datetime
import io
import ipaddress
import json
import logging
import math
import os
import queue
import random
import re
import sqlite3
import shutil
import tempfile
import threading
import time
import uuid
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional
from urllib.parse import parse_qs, urlparse

import requests
import sounddevice as sd
import soundfile as sf

from announce_engine import (  # 状況アナウンス(2026-10-13)
    AnnounceCache, AnnounceEngine, PrepareWorker, parse_loop_request, parse_once_request, parse_prepare_request,
    synthesize_long,
)

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("voice_bridge")

# --- 設定(環境変数で上書き可能。デフォルトはkanbanmusumeを踏襲) ---
VOICEVOX_URL = os.environ.get("VOICEVOX_URL", "http://127.0.0.1:50021")
OLLAMA_URL = os.environ.get("OLLAMA_URL", "http://127.0.0.1:11434/api/generate")
# 【2026-10-30】既定の話者を 2 -> 27 に変更(Jetson2で班員が高専祭用に変えた値を取り込み。docs/jetson2_team_changes.md)。
# 環境変数 VOICEVOX_SPEAKER_ID(~/.nokosahen_audio.env など)で従来どおり上書きできる。
SPEAKER_ID = int(os.environ.get("VOICEVOX_SPEAKER_ID", "27"))
MODEL_NAME = os.environ.get("OLLAMA_MODEL", "gemma2:2b")
OLLAMA_NUM_PREDICT = int(os.environ.get("OLLAMA_NUM_PREDICT", "400"))
_ka = os.environ.get("OLLAMA_KEEP_ALIVE", "10m").strip()
# "-1"(常駐)のような数値文字列はOllamaに整数で渡す(文字列"-1"は時間表記として解釈できない可能性があるため)
OLLAMA_KEEP_ALIVE = int(_ka) if _ka.lstrip("-").isdigit() else _ka
# Ollamaへの問い合わせの待ち時間[秒]。以前は10秒固定だったが、Jetsonではモデルの初回読み込みや
# 生成に10秒以上かかることが班員のPoC検証で分かった(PoC側は300秒に延ばして検証)ため、
# 既定60秒にして環境変数で調整できるようにした。超えた場合は従来どおりフォールバック文面を使う。
OLLAMA_TIMEOUT_SEC = float(os.environ.get("OLLAMA_TIMEOUT_SEC", "60"))

# scripts/setup_udev_rules.sh が案内する /dev/snd/by-id/... の名前、または
# `python3 -m sounddevice` で確認したデバイス名/インデックスを指定する。
# 未設定ならOSのデフォルト出力を使う。
OUTPUT_DEVICE = os.environ.get("VOICE_OUTPUT_DEVICE") or None

HOST = os.environ.get("VOICE_BRIDGE_HOST", "0.0.0.0")
PORT = int(os.environ.get("VOICE_BRIDGE_PORT", "8100"))
# 【2026-10-13】接続元IPの許可リスト(カンマ区切り。IPまたは198.51.100.0/24形式。"*"で無効=開発時用)。
# 既定は1台目Jetson(198.51.100.2)とループバック。許可外のIPからはどのAPIも403(音声・設定は認証なしのため)。
ALLOW_DEFAULT = "198.51.100.2,127.0.0.1"


def parse_allow_list(raw: str) -> tuple:
    """VOICE_BRIDGE_ALLOWを解釈する。(全許可か, ネットワークのリスト)。読めない項目は警告して無視する(許可を増やさない側)。"""
    nets = []
    wildcard = False
    for item in str(raw or "").split(","):
        item = item.strip()
        if not item:
            continue
        if item == "*":
            wildcard = True
            continue
        try:
            nets.append(ipaddress.ip_network(item, strict=False))
        except ValueError:
            logging.getLogger("voice_bridge").warning("VOICE_BRIDGE_ALLOWの項目を無視します(IPとして読めません): %r", item)
    return wildcard, nets


def client_allowed(ip: str, allow: tuple) -> bool:
    """接続元IPが許可リスト(parse_allow_listの結果)に入っているか。ループバックは常に許可(同じ機体内の確認用)。"""
    wildcard, nets = allow
    if wildcard:
        return True
    try:
        addr = ipaddress.ip_address(ip.split("%")[0])
    except ValueError:
        return False
    if getattr(addr, "ipv4_mapped", None) is not None:  # ::ffff:198.51.100.2 形式
        addr = addr.ipv4_mapped
    if addr.is_loopback:
        return True
    return any(addr.version == n.version and addr in n for n in nets)


ALLOW = parse_allow_list(os.environ.get("VOICE_BRIDGE_ALLOW", ALLOW_DEFAULT))

# 積みすぎ防止(VOICEVOX/Ollamaが長時間ダウンしている間、無制限にメモリを
# 消費し続けないようにする)。あふれた場合は一番古い未処理分から捨てる。
# 【2026-10-02変更】既定20→1。発話は生成+再生で20〜40秒かかるため、溜めても
# 古い案内を延々と喋り続けるだけになる(未処理は最新の1件だけ残す)。
MAX_QUEUE_SIZE = int(os.environ.get("VOICE_BRIDGE_MAX_QUEUE", "1"))
# 同一内容(context/text)の発話依頼を、直前の受付からこの秒数以内は無視する。
MIN_REPEAT_SEC = float(os.environ.get("VOICE_BRIDGE_MIN_REPEAT_SEC", "45"))
# 不正/巨大なContent-Lengthを送りつけるクライアントへの防御(上限を超えたら拒否)。
MAX_CONTENT_LENGTH = 1024 * 64

# 【2026-07-22追加】いつ・何と発話したかの履歴(ダッシュボードの発話履歴
# ページから参照する)。nokosahen_core側のconsumption_log.pyと同じ考え方で、
# 別プロセス(ダッシュボード)から読み取れるようSQLiteファイルに記録する。
UTTERANCE_LOG_DB = os.path.expanduser(
    os.environ.get("VOICE_UTTERANCE_LOG_DB", "~/.nokosahen_utterance_log.db")
)


# 【2026-10-06追加】発話ごとの計測値(呼びかけ音声設定ページの「実績」)。
# 既存のutterances(id, timestamp, text)へ ALTER TABLE ADD COLUMN で足す(冪等。
# 古い行は新しい列がNULLのまま。ダッシュボード側の読み取りは古い行でも動く)。
_METRIC_COLUMNS = (
    ("request_id", "TEXT"),
    ("kind", "TEXT"),  # "context"(menu_itemsから生成) / "text"(文面指定)
    ("phase", "TEXT"),
    ("llm_ms", "REAL"),  # Ollamaの生成時間(呼ばなかった場合はNULL)
    ("tts_ms", "REAL"),  # VOICEVOXの音声合成(audio_query+synthesis)時間
    ("play_ms", "REAL"),  # 再生時間(スピーカー出力が終わるまで)
    ("queue_wait_ms", "REAL"),  # 依頼を受け付けてから処理開始までの待ち
    ("total_ms", "REAL"),  # 依頼受付から再生完了まで
    ("used_menu_names", "INTEGER"),  # 生成時のプロンプトに実際にメニュー名が入っていたか(0/1)
    ("used_remaining", "INTEGER"),  # 同、残数情報が入っていたか(0/1)
    ("fallback_reason", "TEXT"),  # フォールバック文を使った理由(無ければNULL)
    ("trimmed", "INTEGER"),  # 長さ上限で文末までに切り詰めたか(0/1)
    ("error", "TEXT"),  # 生成/合成のエラー概要(先頭200文字)
    ("model", "TEXT"),
    ("prompt", "TEXT"),  # 実際に送ったプロンプト(先頭PROMPT_LOG_MAX文字まで)
    ("temperature", "REAL"),
    ("num_predict", "INTEGER"),
    ("speak_ok", "INTEGER"),  # 合成・再生が成功したか(0/1)
)
_METRIC_NAMES = tuple(c for c, _ in _METRIC_COLUMNS)
# /utterances と /voice_metrics の一覧には載せない(大きいので、1件指定の詳細でだけ返す)。
_LIST_METRIC_NAMES = tuple(c for c in _METRIC_NAMES if c != "prompt")
_BOOL_METRICS = ("used_menu_names", "used_remaining", "trimmed", "speak_ok")
PROMPT_LOG_MAX = 4000


def _init_utterance_log() -> None:
    conn = sqlite3.connect(UTTERANCE_LOG_DB, timeout=5.0)
    try:
        # WAL: ダッシュボード側(読み取り専用)が同時アクセスしてもブロックされにくくする。
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute(
            """
            CREATE TABLE IF NOT EXISTS utterances (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                text TEXT NOT NULL
            )
            """
        )
        conn.execute(
            "CREATE INDEX IF NOT EXISTS idx_utterances_timestamp ON utterances(timestamp)"
        )
        existing = {row[1] for row in conn.execute("PRAGMA table_info(utterances)")}
        for name, typ in _METRIC_COLUMNS:
            if name not in existing:
                try:
                    conn.execute(f"ALTER TABLE utterances ADD COLUMN {name} {typ}")
                except sqlite3.OperationalError as exc:  # 別プロセスが先に追加した等(冪等)
                    if "duplicate column" not in str(exc).lower():
                        raise
        conn.commit()
    finally:
        conn.close()


def _record_utterance(text: str, meta: Optional[dict] = None) -> Optional[int]:
    """発話文面が決まった時点(実際の再生開始直前)の時刻と内容を記録する。

    speak_voicevox()はVOICEVOX側の失敗を内部で握りつぶす(戻り値で成否を
    返さない)ため、ここでは「再生を試みた」時点を記録する。再生後の計測値
    (tts_ms/play_ms/total_ms/speak_ok)は _update_utterance() で同じ行へ追記する。
    metaには生成時に分かる値(request_id/llm_ms/フラグ/prompt等)を入れる。
    戻り値: 行id(記録に失敗したらNone)。
    """
    try:
        cols = ["timestamp", "text"]
        vals: list = [datetime.datetime.now().isoformat(timespec="seconds"), text]
        for k, v in (meta or {}).items():
            if k in _METRIC_NAMES:
                cols.append(k)
                vals.append(v)
        conn = sqlite3.connect(UTTERANCE_LOG_DB, timeout=5.0)
        try:
            cur = conn.execute(
                f"INSERT INTO utterances ({', '.join(cols)}) VALUES ({', '.join('?' * len(cols))})",
                vals,
            )
            conn.commit()
            return cur.lastrowid
        finally:
            conn.close()
    except Exception as exc:  # noqa: BLE001 - ログ失敗で発話自体を止めない
        logger.warning("発話履歴の記録に失敗: %s", exc)
        return None


def _update_utterance(row_id: Optional[int], meta: dict) -> None:
    if row_id is None:
        return
    items = [(k, v) for k, v in meta.items() if k in _METRIC_NAMES]
    if not items:
        return
    try:
        conn = sqlite3.connect(UTTERANCE_LOG_DB, timeout=5.0)
        try:
            conn.execute(
                f"UPDATE utterances SET {', '.join(k + ' = ?' for k, _ in items)} WHERE id = ?",
                [v for _, v in items] + [row_id],
            )
            conn.commit()
        finally:
            conn.close()
    except Exception as exc:  # noqa: BLE001
        logger.warning("発話計測値の更新に失敗: %s", exc)


_init_utterance_log()


def _bool_fix(d: dict) -> dict:
    for b in _BOOL_METRICS:
        if d.get(b) is not None:
            d[b] = bool(d[b])
    return d


def fetch_utterances(since: str = "", limit: int = 1000) -> list:
    """読み取り専用。nokosahen_core/utterance_log.pyのfetch_utterancesと同じ
    並び・条件(timestamp昇順、sinceがあればtimestamp >= since)。
    limitは新しい側からlimit件を取り、昇順で返す。
    各行は timestamp/text に加えて計測列(_LIST_METRIC_NAMES、古い行はNone)を持つ(追加のみ)。"""
    limit = max(1, min(int(limit), 1000))
    query = f"SELECT timestamp, text, {', '.join(_LIST_METRIC_NAMES)} FROM utterances"
    params: list = []
    if since:
        query += " WHERE timestamp >= ?"
        params.append(since)
    query += " ORDER BY timestamp DESC, id DESC LIMIT ?"
    params.append(limit)
    conn = sqlite3.connect(f"file:{UTTERANCE_LOG_DB}?mode=ro", uri=True, timeout=5.0)
    try:
        rows = conn.execute(query, params).fetchall()
    finally:
        conn.close()
    rows.reverse()
    out = []
    for r in rows:
        d = {"timestamp": r[0], "text": r[1]}
        for i, n in enumerate(_LIST_METRIC_NAMES):
            d[n] = r[2 + i]
        out.append(_bool_fix(d))
    return out


def fetch_metrics_rows(limit: int = 50, row_id: Optional[int] = None) -> list:
    """計測用の行(id付き、昇順)。row_id指定なら1件(promptも含む)。"""
    names = _METRIC_NAMES if row_id is not None else _LIST_METRIC_NAMES
    query = f"SELECT timestamp, text, id, {', '.join(names)} FROM utterances"
    params: list = []
    if row_id is not None:
        query += " WHERE id = ?"
        params.append(int(row_id))
    else:
        query += " ORDER BY timestamp DESC, id DESC LIMIT ?"
        params.append(max(1, min(int(limit), 1000)))
    conn = sqlite3.connect(f"file:{UTTERANCE_LOG_DB}?mode=ro", uri=True, timeout=5.0)
    try:
        rows = conn.execute(query, params).fetchall()
    finally:
        conn.close()
    rows.reverse()
    out = []
    for r in rows:
        d = {"timestamp": r[0], "text": r[1], "id": r[2]}
        for i, n in enumerate(names):
            d[n] = r[3 + i]
        out.append(_bool_fix(d))
    return out


_STAT_FIELDS = ("llm_ms", "tts_ms", "play_ms", "queue_wait_ms", "total_ms")


def compute_metric_stats(rows: list) -> dict:
    """行(fetch_metrics_rows)から、各時間の件数/平均/中央値/p95/最大と集計を作る。

    p95は最近傍順位法(昇順n個のうち ceil(0.95n)番目)。値がNULLの行(計測前の古い行、
    LLMを呼ばなかった行のllm_ms等)は、その項目の統計から除く(count=その項目の有効件数)。
    """
    stats: dict = {}
    for f in _STAT_FIELDS:
        vals = sorted(
            float(r[f]) for r in rows
            if isinstance(r.get(f), (int, float)) and not isinstance(r.get(f), bool)
            and math.isfinite(float(r[f]))
        )
        n = len(vals)
        if n == 0:
            stats[f] = {"count": 0, "mean": None, "median": None, "p95": None, "max": None}
            continue
        mid = n // 2
        median = vals[mid] if n % 2 else (vals[mid - 1] + vals[mid]) / 2.0
        p95 = vals[max(0, math.ceil(0.95 * n) - 1)]
        stats[f] = {
            "count": n,
            "mean": round(sum(vals) / n, 1),
            "median": round(median, 1),
            "p95": round(p95, 1),
            "max": round(vals[-1], 1),
        }
    reasons: dict = {}
    for r in rows:
        fr = r.get("fallback_reason")
        if fr:
            reasons[fr] = reasons.get(fr, 0) + 1
    return {
        "rows": len(rows),
        "measured": sum(1 for r in rows if r.get("total_ms") is not None),
        "timings": stats,
        "fallback_count": sum(reasons.values()),
        "fallback_reasons": reasons,
        "used_menu_names_count": sum(1 for r in rows if r.get("used_menu_names") is True),
        "used_remaining_count": sum(1 for r in rows if r.get("used_remaining") is True),
        "speak_failed_count": sum(1 for r in rows if r.get("speak_ok") is False),
    }


# 【2026-10-30】高専祭用の声(Jetson2で班員が変えた値を取り込み。旧: speed 0.97 / pitch -0.04 / intonation 0.88)。
VOICE_QUERY_TUNING = {
    "speedScale": 0.76,
    "pitchScale": -0.09,
    "intonationScale": 2.0,
    "prePhonemeLength": 0.08,
    "postPhonemeLength": 0.12,
}

_persona_prompt_path = os.path.join(os.path.dirname(__file__), "persona_prompt.txt")
DEFAULT_PERSONA_PROMPT = (
    "あなたは学校や職場の食堂で長年働いている、世話焼きで情に厚い「食堂のおばちゃん」です。"
    "返答は必ず日本語のみで書いてください。英語や英数字の見出し、余計な注釈は入れないでください。"
    "必ず最初の一文を「お疲れ様ー、今日も来てくれてありがとねぇ。」から始め、"
    "そのあとにおすすめメニューとその理由、残さず食べてほしい気持ち、"
    "「午後もお仕事頑張りなさい」のような送り出す一言を続けてください。"
    "語尾には「〜さね」「〜だよ」「〜なさい」など、世話焼きで少し命令口調だけど"
    "温かみのある喋り方を自然に混ぜてください。"
)


# 【2026-10-30】Ollamaが作った発話は先頭の何文までを使うか(班員がJetson2で入れた「先頭2文に切る」処理を設定化)。
# 0以下=切らない。環境変数 VOICE_MAX_SENTENCES で変更できる。フォールバック文・固定文(generate=false)には使わない。
MAX_SENTENCES = int(os.environ.get("VOICE_MAX_SENTENCES", "2"))
_SENTENCE_RE = re.compile(r"[^。！？!?\n]*(?:[。！？!?]+|\n+|$)")


def limit_sentences(text: str, max_sentences: int) -> str:
    """文末(。！？!?と改行)で区切り、先頭max_sentences文だけを返す。

    - max_sentences <= 0 は「切らない」(前後の空白だけ除く)。
    - 文が max_sentences 以下、または句点などが1つも無い(1文とみなす)ときは、そのまま返す。
    - 切るときは各文の元の句点(。！？)をそのまま残し、文の途中では切らない。
    - 空・空白だけなら空文字。
    """
    text = (text or "").strip()
    if not text or max_sentences <= 0:
        return text
    sentences = [m.group(0).strip() for m in _SENTENCE_RE.finditer(text)]
    sentences = [x for x in sentences if x]
    if len(sentences) <= max_sentences:
        return text
    return "".join(sentences[:max_sentences])


def load_persona_prompt() -> str:
    """発話キャラクターのプロンプトの「標準」を外部ファイルから読む(無ければデフォルト)。

    運用担当者がコードを触らずに `persona_prompt.txt` を書き換えるだけで
    口調を調整できるようにするための、要件どおりの「事前設定できる場所」。
    【2026-10-06】実際に使われるプロンプトは、設定ファイル(voice_settings.json)の
    persona_promptがあればそれが優先される(get_voice_settings())。このファイルは「標準」。
    """
    if os.path.exists(_persona_prompt_path):
        with open(_persona_prompt_path, "r", encoding="utf-8") as f:
            text = f.read().strip()
            if text:
                return text
    return DEFAULT_PERSONA_PROMPT


# ---------------------------------------------------------------------------
# 【2026-10-06追加】呼びかけ音声の設定(ダッシュボードの /voice_settings ページから編集)。
# 優先順位: 設定ファイル(~/.nokosahen_voice_settings.json) > 環境変数 > 既定値。
# ワーカーは発話1件ごとに get_voice_settings() で読むので、保存すれば再起動なしで次の発話から効く
# (ファイルのmtime/サイズが変わったときだけ読み直すキャッシュ)。
# 注意: /voice_settings と /voice_preview も /speak /bgm /status と同様、LAN内に無認証で開いている。
# ダッシュボード(1台目)経由のログイン内に閉じ込める運用で、外部ネットへは出さないこと。
# ---------------------------------------------------------------------------
VOICE_SETTINGS_FILE = os.path.expanduser(
    os.environ.get("VOICE_SETTINGS_FILE", "~/.nokosahen_voice_settings.json")
)
VOICE_SETTINGS_VERSION = 1
PERSONA_MAX_CHARS = 4000
DEFAULT_TEMPERATURE = 0.7

BOOL_SETTINGS = ("include_menu_names", "include_remaining")
# key: (最小, 最大, 整数か)。範囲外は丸めず拒否する。
NUMERIC_LIMITS = {
    "temperature": (0.0, 2.0, False),
    "num_predict": (32, 2000, True),
    "max_chars": (0, 500, True),  # 0=上限なし
    "ollama_timeout_sec": (5.0, 300.0, False),
    "min_repeat_sec": (0.0, 600.0, False),
}
SETTING_KEYS = BOOL_SETTINGS + tuple(NUMERIC_LIMITS) + ("persona_prompt",)
_ENV_FOR_KEY = {
    "num_predict": "OLLAMA_NUM_PREDICT",
    "ollama_timeout_sec": "OLLAMA_TIMEOUT_SEC",
    "min_repeat_sec": "VOICE_BRIDGE_MIN_REPEAT_SEC",
}


def default_voice_settings() -> dict:
    """設定ファイルが無いときの値(環境変数・既定値)。モジュール定数を実行時に読む。"""
    return {
        "include_menu_names": True,
        "include_remaining": True,
        "temperature": DEFAULT_TEMPERATURE,
        "num_predict": OLLAMA_NUM_PREDICT,
        "max_chars": 0,
        "ollama_timeout_sec": OLLAMA_TIMEOUT_SEC,
        "min_repeat_sec": MIN_REPEAT_SEC,
        "persona_prompt": load_persona_prompt(),
    }


def validate_settings_update(data) -> tuple:
    """設定の更新(部分更新可)を検証する。(updates, errors)。

    updates: {キー: 値}。値がNoneのキーは「ファイルから消して既定値に戻す」(persona_prompt等)。
    型違い・範囲外・NaN/Infinity・未知のキーはすべてエラー(丸めない)。errorsは日本語の一覧。
    """
    if not isinstance(data, dict):
        return {}, ["設定はJSONオブジェクトで送ってください"]
    updates: dict = {}
    errors: list = []
    for key, value in data.items():
        if key not in SETTING_KEYS:
            errors.append(f"未知の項目です: {key}")
            continue
        if value is None:
            updates[key] = None
            continue
        if key in BOOL_SETTINGS:
            if not isinstance(value, bool):
                errors.append(f"{key}は true/false で指定してください")
            else:
                updates[key] = value
        elif key == "persona_prompt":
            if not isinstance(value, str):
                errors.append("persona_promptは文字列で指定してください")
                continue
            text = value.strip()
            if not text:
                errors.append("プロンプトを空にはできません(標準に戻すときはnullを送ってください)")
            elif len(text) > PERSONA_MAX_CHARS:
                errors.append(f"プロンプトは{PERSONA_MAX_CHARS}文字以内にしてください(現在{len(text)}文字)")
            elif "\x00" in text:
                errors.append("プロンプトに使えない文字(NUL)が含まれています")
            else:
                updates[key] = text
        else:
            lo, hi, is_int = NUMERIC_LIMITS[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                errors.append(f"{key}は数値で指定してください")
                continue
            v = float(value)
            if not math.isfinite(v):
                errors.append(f"{key}は有限の数値で指定してください")
                continue
            if is_int:
                if v != int(v):
                    errors.append(f"{key}は整数で指定してください")
                    continue
                v = int(v)
            if v < lo or v > hi:
                errors.append(f"{key}は{lo}〜{hi}の範囲で指定してください(指定: {value})")
                continue
            updates[key] = v
    return updates, errors


class VoiceSettingsStore:
    """設定ファイルの読み(mtime/サイズ変化でホットリロード)と原子的な保存(.bak付き)。"""

    def __init__(self, path: str) -> None:
        self.path = os.path.expanduser(path)
        self._lock = threading.Lock()
        self._sig = None
        self._loaded = False
        self._file_values: dict = {}
        self.load_error = ""
        # 【2026-10-13】弱いアカウント(operator)の一時的な上書き(メモリのみ。保存ファイルは変えない)。
        # 値None=既定値に戻す指定。管理者の保存・/ephemeral_reset・再起動で消える。docs/accounts.md
        self._overlay: dict = {}

    @property
    def backup_path(self) -> str:
        return self.path + ".bak"

    @staticmethod
    def _reject_constant(name):
        raise ValueError(f"JSONに{name}(NaN/Infinity)は使えません")

    def reset_ephemeral(self) -> int:
        with self._lock:
            n = len(self._overlay)
            self._overlay.clear()
        return n

    def file_values(self) -> dict:
        """ファイルで指定されている項目だけ(無い/不正なら空。不正の理由はload_error)。"""
        try:
            st = os.stat(self.path)
            sig = (st.st_mtime_ns, st.st_size)
        except OSError:
            sig = None
        with self._lock:
            if self._loaded and sig == self._sig:
                return dict(self._file_values)
            self._loaded = True
            self._sig = sig
            self._file_values = {}
            self.load_error = ""
            if sig is None:
                return {}
            try:
                with open(self.path, "r", encoding="utf-8") as f:
                    doc = json.loads(f.read(), parse_constant=self._reject_constant)
                if not isinstance(doc, dict):
                    raise ValueError("トップレベルがJSONオブジェクトではありません")
                body = {k: v for k, v in doc.items() if k not in ("version", "description")}
                updates, errors = validate_settings_update(body)
                if errors:
                    raise ValueError(" / ".join(errors))
                self._file_values = {k: v for k, v in updates.items() if v is not None}
            except (OSError, ValueError) as exc:
                self.load_error = f"設定ファイルを無視しました(環境変数・既定値で動作中): {exc}"
                logger.warning("%s: %s", self.load_error, self.path)
                self._file_values = {}
            return dict(self._file_values)

    def effective(self) -> dict:
        out = default_voice_settings()
        out.update(self.file_values())
        if self._overlay:
            defaults = default_voice_settings()
            for k, v in dict(self._overlay).items():
                out[k] = defaults[k] if v is None else v
        return out

    def sources(self) -> dict:
        fv = self.file_values()
        src = {}
        for k in SETTING_KEYS:
            if k in self._overlay:
                src[k] = "ephemeral"
            elif k in fv:
                src[k] = "file"
            elif k == "persona_prompt":
                src[k] = "persona_prompt.txt" if os.path.exists(_persona_prompt_path) else "builtin"
            elif k in _ENV_FOR_KEY and os.environ.get(_ENV_FOR_KEY[k]) is not None:
                src[k] = "env"
            else:
                src[k] = "default"
        return src

    def update(self, updates: dict, persist: bool = True) -> tuple:
        """検証済みのupdatesを保存する。(ok, errors)。値Noneのキーはファイルから外す。
        persist=Falseはメモリ上の一時上書きだけ(ファイルは変えない。operator用、docs/accounts.md)。"""
        if not persist:
            with self._lock:
                self._overlay.update(updates)
            return True, []
        with self._lock:
            current: dict = {}
            try:
                with open(self.path, "r", encoding="utf-8") as f:
                    doc = json.loads(f.read(), parse_constant=self._reject_constant)
                if isinstance(doc, dict):
                    upd, errs = validate_settings_update(
                        {k: v for k, v in doc.items() if k not in ("version", "description")}
                    )
                    if not errs:
                        current = {k: v for k, v in upd.items() if v is not None}
            except (OSError, ValueError):
                current = {}  # 無い/壊れている場合は、今回の更新分だけで作り直す(壊れた方は.bakへ残る)
            for k, v in updates.items():
                if v is None:
                    current.pop(k, None)
                else:
                    current[k] = v
            doc_out = {"version": VOICE_SETTINGS_VERSION}
            doc_out.update({k: current[k] for k in SETTING_KEYS if k in current})
            text = json.dumps(doc_out, ensure_ascii=False, indent=2) + "\n"
            directory = os.path.dirname(self.path) or "."
            tmp_path = bak_tmp = None
            try:
                os.makedirs(directory, exist_ok=True)
                fd, tmp_path = tempfile.mkstemp(prefix=".nokosahen_voice_", suffix=".tmp", dir=directory)
                with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as f:
                    f.write(text)
                    f.flush()
                    os.fsync(f.fileno())
                if os.path.exists(self.path):
                    bak_tmp = self.backup_path + ".tmp"
                    shutil.copy2(self.path, bak_tmp)
                    os.replace(bak_tmp, self.backup_path)
                    bak_tmp = None
                os.replace(tmp_path, self.path)
                tmp_path = None
            except OSError as exc:
                return False, [f"設定ファイルを書き込めませんでした(元のファイルは変更されていません): {exc}"]
            finally:
                for p in (tmp_path, bak_tmp):
                    if p:
                        try:
                            os.remove(p)
                        except OSError:
                            pass
            for k in updates:
                self._overlay.pop(k, None)  # 管理者が保存した項目の一時上書きは破棄される
            self._loaded = False  # 次の読み取りで必ず読み直す
        return True, []


settings_store = VoiceSettingsStore(VOICE_SETTINGS_FILE)


def get_voice_settings() -> dict:
    """いま有効な設定(設定ファイル > 環境変数 > 既定値)。発話1件ごとに呼ぶ。"""
    return settings_store.effective()


def settings_payload() -> dict:
    """GET /voice_settings の中身(+POST後の応答)。"""
    return {
        "ok": True,
        "settings": settings_store.effective(),
        "defaults": default_voice_settings(),
        "sources": settings_store.sources(),
        "limits": {
            "temperature": {"min": 0.0, "max": 2.0},
            **{k: {"min": lo, "max": hi} for k, (lo, hi, _i) in NUMERIC_LIMITS.items() if k != "temperature"},
            "persona_prompt": {"max_chars": PERSONA_MAX_CHARS},
        },
        "model": MODEL_NAME,
        "settings_file": settings_store.path,
        "load_error": settings_store.load_error,
        "queue": {"max_queue_size": MAX_QUEUE_SIZE},
    }


def _short_err(exc: BaseException) -> str:
    return f"{type(exc).__name__}: {exc}"[:200]


def _trim_prompt(prompt: str) -> str:
    return prompt if len(prompt) <= PROMPT_LOG_MAX else prompt[:PROMPT_LOG_MAX] + "…(以下省略)"


_SENTENCE_ENDS = ("。", "！", "？", "!", "?", "」", "\n")


def _call_ollama(prompt: str, fallback: str, settings: Optional[dict] = None,
                 trace: Optional[dict] = None) -> str:
    """Ollamaへ問い合わせ、失敗/空応答なら必ずfallbackへ逃がす共通処理。

    traceを渡すと計測結果を書き込む: llm_ms(Ollamaを待った時間)/model/temperature/
    num_predict/prompt(先頭4000文字)/fallback_reason(timeout|error|length_cut|placeholder|empty)/
    trimmed(文末までに切り詰めたか)/error。
    """
    if trace is None:
        trace = {}
    if settings is None:
        settings = get_voice_settings()
    num_predict = int(settings["num_predict"])
    temperature = float(settings["temperature"])
    max_chars = int(settings["max_chars"])
    payload = {
        "model": MODEL_NAME,
        "prompt": prompt,
        "stream": False,
        "keep_alive": OLLAMA_KEEP_ALIVE,
        "options": {"num_predict": num_predict, "temperature": temperature},
    }
    trace.update(
        model=MODEL_NAME, temperature=temperature, num_predict=num_predict,
        prompt=_trim_prompt(prompt), trimmed=False,
    )

    def fb(reason: str) -> str:
        trace["fallback_reason"] = reason
        return fallback

    t0 = time.monotonic()
    try:
        try:
            res = requests.post(OLLAMA_URL, json=payload, timeout=float(settings["ollama_timeout_sec"]))
            res.raise_for_status()
            body = res.json()
        finally:
            trace["llm_ms"] = round((time.monotonic() - t0) * 1000.0, 1)
        rewritten = body["response"].strip()
        if body.get("done_reason") == "length":
            # num_predictに達して途中で切れた(「うど」のような語の途中切れ防止)。
            # 最後の完結した文末までに切り詰め、文末が無ければフォールバック。
            cut = max(rewritten.rfind(ch) for ch in _SENTENCE_ENDS)
            if cut < 0:
                logger.warning("Ollama応答が途中で切れ文末も無いため、フォールバックで代替")
                return fb("length_cut")
            logger.warning("Ollama応答が長さ上限で切れたため、最後の文末までに切り詰め")
            rewritten = rewritten[: cut + 1].strip()
            trace["trimmed"] = True
        if max_chars > 0 and len(rewritten) > max_chars:
            cut = max(rewritten[:max_chars].rfind(ch) for ch in _SENTENCE_ENDS)
            if cut < 0:
                logger.warning("Ollama応答が最大文字数(%d)を超え文末も無いため、フォールバックで代替", max_chars)
                return fb("length_cut")
            rewritten = rewritten[: cut + 1].strip()
            trace["trimmed"] = True
        if any(ch in rewritten for ch in "○〇"):
            # メニュー名が入るべき所が「○○」のままの作り話(プロンプト例の丸写し)。
            logger.warning("Ollama応答に○○のプレースホルダが残っているため、フォールバックで代替")
            return fb("placeholder")
        if not rewritten:
            # 正常応答でも空文字列(モデルが空補完を返した等)はフォールバック対象。
            # ここを素通りすると speak_voicevox("") が呼ばれてしまう。
            logger.warning("Ollamaが空応答、フォールバックで代替")
            return fb("empty")
        return rewritten
    except Exception as exc:  # noqa: BLE001 - どんな失敗でもフォールバックへ逃がす
        logger.warning("Ollama生成に失敗、フォールバックで代替: %s", exc)
        trace["error"] = _short_err(exc)
        return fb("timeout" if isinstance(exc, requests.exceptions.Timeout) else "error")


def generate_with_ollama(text: str, trace: Optional[dict] = None,
                         settings: Optional[dict] = None) -> str:
    """Ollamaで、既に決まっている文面をキャラクターの口調に言い換える(旧方式)。"""
    if settings is None:
        settings = get_voice_settings()
    full_prompt = f"<設定>\n{settings['persona_prompt']}\n\n伝えたい内容: {text}\n看板娘:"
    if trace is not None:
        trace.update(used_menu_names=False, used_remaining=False)
    return _call_ollama(full_prompt, fallback=text, settings=settings, trace=trace)


_NO_PLACEHOLDER_RULE = (
    "メニュー名は必ず下の一覧にある名前をそのまま使い、「○○」「〇〇」という"
    "伏せ字のまま出力したり、一覧に無いメニューを作ったりしないでください。"
)
_NO_NAMES_RULE = (
    "メニュー名は一覧として与えられていないので、特定のメニュー名には触れず、"
    "「○○」「〇〇」という伏せ字や、存在しないメニューの名前も出力しないでください。"
)


def build_context_prompt_ex(context: dict, settings: Optional[dict] = None) -> tuple:
    """menu_items(+induction時はremaining_status)から、AIが1から文面を考えるための
    プロンプトを組み立てる。(prompt, flags)を返す。

    flags.used_menu_names / used_remaining は「実際にプロンプトへ入れたか」(設定がONでも、
    材料(メニュー一覧/残数状況)が無ければfalse)。設定のinclude_menu_names/include_remainingが
    falseなら、そのセクションをプロンプトから省く(メニュー名なし+残数あり、のときは
    「メニュー1」のように名前を伏せた残数だけを渡す)。両方trueのときの文面は従来と同一。
    """
    if settings is None:
        settings = get_voice_settings()
    inc_menu = bool(settings["include_menu_names"])
    inc_rem = bool(settings["include_remaining"])
    persona_prompt = settings["persona_prompt"]
    menu_items = [m for m in (context.get("menu_items") or []) if isinstance(m, dict) and m.get("name")]
    phase = context.get("phase", "initial")
    promote_item = context.get("promote_item")
    remaining_items = [
        it for it in (context.get("remaining_status") or []) if isinstance(it, dict) and it.get("name")
    ]
    induction = phase == "induction" and bool(promote_item)

    show_remaining = inc_rem and induction and bool(remaining_items)
    show_menu = inc_menu and bool(menu_items)
    menu_lines = "\n".join(f"- {m.get('name', '')}: {m.get('merit', '')}" for m in menu_items)

    if induction:
        target = f"「{promote_item}」" if inc_menu else "あるメニュー"
        lead = (
            f"今は特に{target}の残数に余裕があります。"
            if show_remaining
            else f"今は特に{target}をおすすめしたい状況です。"
        )
        materials = (["メニュー一覧"] if show_menu else []) + (["残数状況"] if show_remaining else [])
        basis = ("以下の" + "と".join(materials) + "を踏まえて、") if materials else ""
        suggest = f"「{promote_item}」" if inc_menu else "おすすめのメニュー"
        instruction = (
            f"{lead}{basis}お客さんにさりげなく{suggest}をすすめる一言を考えてください。"
            f"{_NO_PLACEHOLDER_RULE if inc_menu else _NO_NAMES_RULE}"
        )
    elif show_menu:
        instruction = (
            "以下のメニュー一覧を踏まえて、お客さんに親しみやすく声をかける"
            "一言を考えてください。特定のメニューを無理に推す必要はありません。"
            f"{_NO_PLACEHOLDER_RULE}"
        )
    else:
        instruction = (
            "お客さんに親しみやすく声をかける一言を考えてください。"
            f"特定のメニューを無理に推す必要はありません。{_NO_NAMES_RULE}"
        )

    sections = []
    if show_menu:
        sections.append(f"<メニュー一覧>\n{menu_lines}")
    if show_remaining:
        remaining_lines = "\n".join(
            f"- {(it.get('name', '') if inc_menu else 'メニュー%d' % (i + 1))}: "
            f"残り{it.get('remaining', '?')}個(元は{it.get('initial', '?')}個)"
            for i, it in enumerate(remaining_items)
        )
        sections.append(f"<残数状況>\n{remaining_lines}")
    facts = "\n\n".join(sections)
    body = f"<設定>\n{persona_prompt}\n\n<指示>\n{instruction}\n\n"
    prompt = body + (facts + "\n" if facts else "") + "看板娘:"
    flags = {
        "used_menu_names": bool(inc_menu and (menu_items or induction)),
        "used_remaining": bool(show_remaining),
        "phase": str(phase),
    }
    return prompt, flags


def _build_context_prompt(context: dict) -> str:
    """(互換用)プロンプト文字列だけを返す。"""
    return build_context_prompt_ex(context)[0]


def generate_utterance_from_context(context: dict, trace: Optional[dict] = None,
                                    settings: Optional[dict] = None) -> str:
    """menu_items(+残数状況)から、AIに発話文面そのものを考えさせる(新方式)。

    behavior_directorはメニュー内容・(あれば)残数状況・ルールベースの
    フォールバック文(fallback_text)をまとめて送ってくるだけで、実際に何を
    話すかの文章はここでOllamaに考えさせる。Ollama未導入/応答失敗/
    menu_items が無い場合はfallback_textをそのまま話す。
    traceには計測値と、プロンプトに実際に入ったメニュー名/残数のフラグを書く。
    """
    if trace is None:
        trace = {}
    if settings is None:
        settings = get_voice_settings()
    fallback = str(context.get("fallback_text", "")).strip()
    menu_items = context.get("menu_items") or []
    if not menu_items:
        # 材料(メニュー情報)が無ければAIに考えさせる意味が無い。
        trace.update(used_menu_names=False, used_remaining=False,
                     phase=str(context.get("phase", "initial")), fallback_reason="no_menu")
        return fallback
    prompt, flags = build_context_prompt_ex(context, settings)
    trace.update(flags)
    return _call_ollama(prompt, fallback=fallback, settings=settings, trace=trace)


_preview_lock = threading.Lock()


def preview_utterance(context: dict, overrides: Optional[dict] = None) -> dict:
    """ドライラン: 未保存の設定(overrides)で文面を生成するだけ。話さず、発話履歴にも記録しない。

    Ollamaを1回呼ぶ(重いので同時に1件まで)。戻り値は応答用のdict。
    """
    settings = get_voice_settings()
    if overrides:
        settings.update({k: v for k, v in overrides.items() if v is not None})
        if overrides.get("persona_prompt") is None and "persona_prompt" in overrides:
            settings["persona_prompt"] = default_voice_settings()["persona_prompt"]
    trace: dict = {}
    t0 = time.monotonic()
    text = generate_utterance_from_context(context, trace=trace, settings=settings)
    out = {
        "ok": True,
        "text": text,
        "chars": len(text),
        "elapsed_ms": round((time.monotonic() - t0) * 1000.0, 1),
        "fallback_reason": trace.get("fallback_reason"),
        "llm_ms": trace.get("llm_ms"),
        "used_menu_names": bool(trace.get("used_menu_names")),
        "used_remaining": bool(trace.get("used_remaining")),
        "trimmed": bool(trace.get("trimmed")),
        "model": trace.get("model"),
        "error": trace.get("error"),
        "prompt": trace.get("prompt"),
    }
    return out


_play_lock = threading.Lock()  # 【2026-10-13】発話とアナウンスの再生が重ならないようにする(どちらも同じ出力ストリームを使う)


def _play_audio(data, samplerate: int) -> None:
    with _play_lock:
        _play_audio_unlocked(data, samplerate)


def _play_audio_unlocked(data, samplerate: int) -> None:
    """再生する。ALSAのhwデバイス(USBスピーカー直指定など)は24kHz/モノラルを
    受け付けず "Invalid sample rate" になることがあるため、その場合はデバイスの
    既定サンプルレートへ線形補間でリサンプルし、ステレオにして再試行する
    (PipeWire経由のデバイス名なら最初の試行で通る)。"""
    try:
        sd.play(data, samplerate, device=OUTPUT_DEVICE)
        sd.wait()
        return
    except sd.PortAudioError as exc:
        logger.warning("再生デバイスが %dHz を受け付けず、リサンプルして再試行: %s", samplerate, exc)
    import numpy as np  # sounddevice/soundfile の依存で入っている

    dev_rate = int(sd.query_devices(OUTPUT_DEVICE, "output")["default_samplerate"])
    mono = data if data.ndim == 1 else data.mean(axis=1)
    n_out = int(round(len(mono) * dev_rate / samplerate))
    resampled = np.interp(
        np.linspace(0, len(mono) - 1, n_out), np.arange(len(mono)), mono
    ).astype("float32")
    stereo = np.stack([resampled, resampled], axis=1)
    sd.play(stereo, dev_rate, device=OUTPUT_DEVICE)
    sd.wait()


# ---------------------------------------------------------------------------
# 【2026-10-02追加】BGM(背景音楽)+自動ダッキング。
# 発話していない間は BGM_DIR の音楽を流し続け、発話の処理中(合成開始〜再生完了)は
# 音量を BGM_DUCK_VOLUME まで下げ、発話が終わって BGM_HOLD_SEC 経ってから戻す。
# voice_bridge と同じプロセスで鳴らすので、ダッキングのタイミングは正確。
# BGMの失敗(フォルダ空/デバイスエラー等)は警告ログだけで、発話には一切影響させない。
# ---------------------------------------------------------------------------
BGM_DIR = os.path.expanduser(os.environ.get("BGM_DIR", "~/bgm"))
BGM_EXTS = (".wav", ".flac", ".ogg", ".oga", ".mp3")


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return float(default)


def _clamp01(v: float) -> float:
    return max(0.0, min(1.0, float(v)))


def _clamp_bgm(v: float) -> float:
    """BGM音量の丸め(0〜BGM_VOLUME_MAX)。上限は音量設定の定数と同じ(下で定義)。"""
    return max(0.0, min(BGM_VOLUME_MAX, float(v)))


# ---------------------------------------------------------------------------
# 【2026-10-09追加】音量の設定(発話・BGMを別々に、実行中に、再起動後も維持)。docs/voice_levels.md
# - speech_volume: 発話の倍率(0〜5)。VOICEVOXの出力に再生前にかける(volumeScaleはVOICEVOX側で硬くクリップするので使わない)。
# - bgm_volume: BGMの通常音量(0〜5)。BGMのmp3は元がほぼフルスケール(ピーク約-0.6dBFS)なので、1.0を超える分は
#   音量そのものでなく「リミッタを通した平均レベル(RMS)の底上げ」になる(波形のピークは天井で止まる)。
# - bgm_duck_volume: 発話中のBGM音量(0〜bgm_volume)。通常より大きくはできない。
# 倍率>1.0は先読みピークリミッタ(LookaheadLimiter)で歪みを抑えつつRMSを上げる。発話とBGMは別ストリームで
# PipeWireが足し合わせるので、合計が1.0を超えて割れないよう「天井の予算」を分ける(発話SPEECH_CEILING+発話中BGM
# BGM_DUCK_CEILING < 0.95)。これが最終段(マスター)の保証。倍率1.0ちょうどの発話は素通し。
# 優先順位: 保存ファイル > 環境変数(SPEECH_VOLUME/BGM_VOLUME/BGM_DUCK_VOLUME) > 既定値。
# 音声マスタースイッチ(VoiceMaster)とは独立: OFF中でも値は変えられ、ONに戻すと設定どおりに鳴る。
# ---------------------------------------------------------------------------
VOICE_LEVELS_FILE = os.path.expanduser(
    os.environ.get("VOICE_LEVELS_FILE", "~/.nokosahen_voice_levels.json")
)
LEVEL_KEYS = ("speech_volume", "bgm_volume", "bgm_duck_volume", "announce_volume")
LEVEL_LABELS = {"speech_volume": "発話音量", "bgm_volume": "BGM通常音量", "bgm_duck_volume": "BGM発話中音量",
                "announce_volume": "アナウンス音量"}
SPEECH_VOLUME_MAX = 5.0
BGM_VOLUME_MAX = 5.0
ANNOUNCE_VOLUME_MAX = 5.0  # 【2026-10-13】状況アナウンスの倍率。発話と同じ天井・予算(発話とアナウンスは同時に鳴らさない)
LEVEL_MAX = {"speech_volume": SPEECH_VOLUME_MAX, "bgm_volume": BGM_VOLUME_MAX, "bgm_duck_volume": BGM_VOLUME_MAX,
             "announce_volume": ANNOUNCE_VOLUME_MAX}
# 画面で「実行する/やめる」の確認を出す境目(これを超える値にするとき)。docs/voice_levels.md
LEVEL_CONFIRM_ABOVE = 2.0
# 既定値(合成信号での数値実験、docs/voice_levels.md): 発話2.5倍(約+8dB、リミッタ後RMS約-16dBFS)、
# BGM通常1.0(従来のmp3そのまま)、発話中0.15(発話RMSより約10dB下)。旧(前日)既定は 発話1.8・BGM0.45・0.12。
DEFAULT_LEVELS = {"speech_volume": 2.5, "bgm_volume": 1.0, "bgm_duck_volume": 0.15, "announce_volume": 2.5}
# ピークの天井(出力は必ずこれ未満)。発話+発話中BGMの合計を0.95未満に保つ予算: 0.65 + 0.29 = 0.94 < 0.95。
SPEECH_CEILING = 0.65
BGM_CEILING = 0.94  # 発話していない間のBGM単独
BGM_DUCK_CEILING = 0.29  # 発話中(ダッキング中)のBGM
MASTER_CEILING = 0.95
LIMITER_ATTACK_MS = 4.0  # 先読み(アタック)。リミッタの遅延は2倍(約8ms)
SPEECH_FADE_IN_SEC = 0.05  # 倍率が1.0以外の発話の頭を短くフェードイン(音量変更直後のクリック/跳ね上がり対策)


def _sliding_min(a, w: int):
    """長さw窓の移動最小値(長さ len(a)-w+1)。numpyだけの倍々法(scipy不要)。"""
    import numpy as np

    if w <= 1:
        return a
    m, k = a, 1
    while k * 2 <= w:
        m = np.minimum(m[:-k], m[k:])
        k *= 2
    if k < w:
        m = np.minimum(m[:-(w - k)], m[w - k:])
    return m


def _moving_mean(a, w: int):
    """長さw窓の移動平均(長さ len(a)-w+1)。"""
    import numpy as np

    c = np.concatenate(([0.0], np.cumsum(a, dtype="float64")))
    return (c[w:] - c[:-w]) / float(w)


def _limit_gain(peak, ceiling, n: int):
    """各サンプルの絶対値peak(複数chは最大)と天井から、先読みリミッタのゲイン列(全体が収まる=1.0)を返す。
    必要ゲイン min(1, 天井/peak) の前後n点の最小値を、さらに前後n点で平均する(=滑らかに下げて戻す)。
    平均窓の半径<=最小値窓の半径なので、各点で必要ゲイン以下になり peak*gain<=天井 が保証される。"""
    import numpy as np

    req = np.minimum(1.0, ceiling / np.maximum(peak, 1e-12)).astype("float64")
    if n <= 0:
        return req
    pad = np.concatenate((np.ones(2 * n), req, np.ones(2 * n)))
    m = _sliding_min(pad, 2 * n + 1)  # 長さ len(req)+2n、m[j]は元の位置 j-n
    g = _moving_mean(m, 2 * n + 1)  # 長さ len(req)
    return np.minimum(g, req)  # 浮動小数点の誤差への念のため


def _attack_samples(samplerate: int) -> int:
    return max(1, int(round(samplerate * LIMITER_ATTACK_MS / 1000.0)))


def limit_peaks(data, ceiling: float, samplerate: int = 24000):
    """先読みピークリミッタ(全体を一度に処理)。ピークが天井以内なら波形を一切変えない。
    超える所だけ前後数msかけて滑らかにゲインを下げて戻す(歪みの少ない音量稼ぎ)。出力は必ず ±ceiling 以下。"""
    import numpy as np

    arr = np.asarray(data, dtype="float32")
    peak = np.abs(arr) if arr.ndim == 1 else np.abs(arr).max(axis=1)
    if arr.size == 0 or float(peak.max()) <= ceiling:
        return arr
    g = _limit_gain(peak, ceiling, _attack_samples(samplerate)).astype("float32")
    out = arr * (g if arr.ndim == 1 else g[:, None])
    return np.clip(out, -ceiling, ceiling).astype("float32")


class LookaheadLimiter:
    """ストリーム(ブロック)用の先読みピークリミッタ。遅延は 2*アタック(約8ms)。ceilingはブロックごと/サンプルごとに変えられる。
    process(block, ceiling)は同じ長さを返す(最初の2*アタック分は無音)。出力は各サンプルの天井以下。"""

    def __init__(self, samplerate: int, channels: int = 2) -> None:
        import numpy as np

        self.n = _attack_samples(samplerate)
        self.channels = channels
        self._x = np.zeros((2 * self.n, channels), dtype="float32")  # 未出力の入力
        self._c = np.full(2 * self.n, 1.0, dtype="float64")  # 未出力サンプルの天井
        self._gq = np.ones(4 * self.n, dtype="float64")  # 必要ゲイン(履歴2n+未出力2n)

    def process(self, block, ceiling):
        import numpy as np

        block = np.asarray(block, dtype="float32")
        k = len(block)
        n = self.n
        c = np.broadcast_to(np.asarray(ceiling, dtype="float64"), (k,)).copy()
        peak = np.abs(block).max(axis=1)
        req = np.minimum(1.0, c / np.maximum(peak, 1e-12))
        x = np.concatenate((self._x, block))
        cc = np.concatenate((self._c, c))
        gq = np.concatenate((self._gq, req))  # 長さ 4n+k
        m = _sliding_min(gq, 2 * n + 1)  # m[j] <-> gq位置 j+n
        g = _moving_mean(m, 2 * n + 1)  # g[j] <-> gq位置 j+2n (=出力サンプル j)
        g = np.minimum(g[:k], gq[2 * n:2 * n + k])
        out = x[:k] * g[:, None].astype("float32")
        out = np.clip(out, -cc[:k, None], cc[:k, None]).astype("float32")
        self._x, self._c, self._gq = x[k:], cc[k:], gq[k:]
        return out


def apply_speech_gain(data, gain: float, samplerate: int = 24000):
    """発話の波形に倍率をかけ、天井(SPEECH_CEILING)を超える所だけ先読みリミッタで滑らかに下げて返す
    (単なるクリップより歪みが少ない)。gainが1.0ちょうどなら何もしない(素通し)。
    1.0以外は頭を短くフェードインする(音量を上げた直後でも発話の頭で急に跳ねない)。"""
    import numpy as np

    arr = np.asarray(data, dtype="float32")
    if gain == 1.0:
        return arr
    y = arr * np.float32(gain)
    y = limit_peaks(y, SPEECH_CEILING, samplerate)
    nf = min(len(y), int(SPEECH_FADE_IN_SEC * samplerate))
    if nf > 1 and gain > 1.0:
        ramp = np.linspace(0.0, 1.0, nf, dtype="float32")
        y = y.copy()
        y[:nf] = y[:nf] * (ramp if y.ndim == 1 else ramp[:, None])
    return y.astype("float32")


def _num_or_none(value):
    """数値(bool・非有限は除く)ならfloat、そうでなければNone。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    v = float(value)
    return v if math.isfinite(v) else None


def validate_levels_update(data, current: dict) -> tuple:
    """POST /voice_levels の本文を検証する。(updates, errors)。丸めずに範囲外は拒否する。
    updatesは {キー: 数値 or None(既定に戻す)}。currentは現在値(部分更新時のダッキング<=通常の検証用)。"""
    if not isinstance(data, dict):
        return {}, ["本文はJSONオブジェクトで指定してください"]
    errors = []
    updates = {}
    for key, value in data.items():
        if key not in LEVEL_KEYS:
            errors.append(f"未知の項目です: {key}")
            continue
        if value is None:
            updates[key] = None
            continue
        v = _num_or_none(value)
        if v is None:
            errors.append(f"{LEVEL_LABELS[key]}は有限の数値で指定してください")
            continue
        if v < 0.0 or v > LEVEL_MAX[key]:
            errors.append(f"{LEVEL_LABELS[key]}は0〜{LEVEL_MAX[key]:g}の範囲で指定してください(指定値{v:g})")
            continue
        updates[key] = v
    if errors:
        return {}, errors
    merged = dict(current)
    for key, value in updates.items():
        merged[key] = DEFAULT_LEVELS[key] if value is None else value
    if merged["bgm_duck_volume"] > merged["bgm_volume"] + 1e-9:
        return {}, [f"BGM発話中音量({merged['bgm_duck_volume']:g})は通常音量({merged['bgm_volume']:g})以下にしてください"]
    return updates, []


class VoiceLevels:
    """発話・BGMの音量(スレッドセーフ)+原子的な永続化。変更時にリスナーを呼ぶ(BGMへ反映)。"""

    def __init__(self, path: str, env=None) -> None:
        self.path = os.path.expanduser(path)
        self._lock = threading.Lock()
        self._listeners: list = []
        self.load_error = ""
        self.persisted = True  # 直近の保存に成功したか(ファイルを書けない環境の目印)
        self.updated_at = ""
        self.from_file = False
        self.defaults = self._env_defaults(os.environ if env is None else env)
        self._values = self._load()
        # 【2026-10-13】保存されている値(ファイルの正)と、operatorの一時上書きキー。docs/accounts.md
        self._persisted = dict(self._values)
        self._ephemeral_keys: set = set()

    @staticmethod
    def _env_defaults(env) -> dict:
        """環境変数 > 既定値。環境変数が不正/範囲外なら既定値を使う(ダッキングは通常以下に揃える)。"""
        out = dict(DEFAULT_LEVELS)
        for key, name in (("speech_volume", "SPEECH_VOLUME"), ("bgm_volume", "BGM_VOLUME"),
                          ("bgm_duck_volume", "BGM_DUCK_VOLUME"), ("announce_volume", "ANNOUNCE_VOLUME")):
            raw = env.get(name)
            if raw is None or str(raw).strip() == "":
                continue
            try:
                v = float(raw)
            except ValueError:
                continue
            if math.isfinite(v) and 0.0 <= v <= LEVEL_MAX[key]:
                out[key] = v
        if out["bgm_duck_volume"] > out["bgm_volume"]:
            out["bgm_duck_volume"] = out["bgm_volume"]
        return out

    def _load(self) -> dict:
        vals = dict(self.defaults)
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                doc = json.loads(f.read())
        except FileNotFoundError:
            return vals
        except (OSError, ValueError) as exc:
            self.load_error = f"音量設定ファイルを読めないため既定値で起動しました: {exc}"
            logger.warning("%s: %s", self.load_error, self.path)
            return vals
        if not isinstance(doc, dict):
            self.load_error = "音量設定ファイルの内容が不正なため既定値で起動しました"
            logger.warning("%s: %s", self.load_error, self.path)
            return vals
        bad = []
        for key in LEVEL_KEYS:
            if key not in doc:
                continue
            v = _num_or_none(doc[key])
            if v is None or v < 0.0 or v > LEVEL_MAX[key]:
                bad.append(key)
                continue
            vals[key] = v
            self.from_file = True
        if vals["bgm_duck_volume"] > vals["bgm_volume"]:
            bad.append("bgm_duck_volume")
            vals["bgm_duck_volume"] = vals["bgm_volume"]
        if bad:
            self.load_error = "音量設定ファイルの次の項目が不正だったため既定値/補正値を使いました: " + ", ".join(bad)
            logger.warning("%s: %s", self.load_error, self.path)
        self.updated_at = str(doc.get("updated_at", ""))[:40]
        return vals

    def add_listener(self, fn) -> None:
        self._listeners.append(fn)

    @property
    def speech_volume(self) -> float:
        return self._values["speech_volume"]

    @property
    def bgm_volume(self) -> float:
        return self._values["bgm_volume"]

    @property
    def bgm_duck_volume(self) -> float:
        return self._values["bgm_duck_volume"]

    @property
    def announce_volume(self) -> float:
        return self._values["announce_volume"]

    def values(self) -> dict:
        with self._lock:
            return dict(self._values)

    def _save_locked(self) -> str:
        """原子的に保存(同じフォルダの一時ファイル->fsync->置換)。失敗時はエラー文を返す。"""
        doc = {"version": 1, **self._persisted,
               "updated_at": datetime.datetime.now().isoformat(timespec="seconds")}
        directory = os.path.dirname(self.path) or "."
        tmp_path = None
        try:
            os.makedirs(directory, exist_ok=True)
            fd, tmp_path = tempfile.mkstemp(prefix=".nokosahen_voice_levels_", suffix=".tmp", dir=directory)
            with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as f:
                f.write(json.dumps(doc, ensure_ascii=False, indent=2) + "\n")
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp_path, self.path)
            tmp_path = None
            self.updated_at = doc["updated_at"]
            return ""
        except OSError as exc:
            return str(exc)
        finally:
            if tmp_path:
                try:
                    os.remove(tmp_path)
                except OSError:
                    pass

    def update(self, updates: dict, persist: bool = True) -> dict:
        """検証済みのupdates({キー: 数値 or None=既定に戻す})を反映して保存する。
        保存に失敗してもメモリ上は反映する(persisted=False+warning、再起動で元に戻る)。
        persist=Falseはメモリ上の一時上書きだけ(保存ファイルは変えない。operator用、docs/accounts.md)。"""
        with self._lock:
            for key, value in updates.items():
                v = self.defaults[key] if value is None else float(value)
                self._values[key] = v
                if persist:
                    self._persisted[key] = v
                    self._ephemeral_keys.discard(key)  # 管理者の保存で一時上書きは破棄される
                else:
                    self._ephemeral_keys.add(key)
            if self._values["bgm_duck_volume"] > self._values["bgm_volume"]:
                self._values["bgm_duck_volume"] = self._values["bgm_volume"]
            if persist:
                if self._persisted["bgm_duck_volume"] > self._persisted["bgm_volume"]:
                    self._persisted["bgm_duck_volume"] = self._persisted["bgm_volume"]
                err = self._save_locked()
                self.persisted = not err
                if not err:
                    self.load_error = ""
                    self.from_file = True
            else:
                err = ""
            snapshot = dict(self._values)
        if err:
            logger.error("音量設定を保存できませんでした(再起動で元に戻ります): %s", err)
        logger.info("音量設定 -> %s", snapshot)
        for fn in list(self._listeners):
            try:
                fn(snapshot)
            except Exception:  # noqa: BLE001 - リスナーの失敗で設定自体を失敗させない
                logger.exception("音量設定のリスナーでエラー")
        out = self.status()
        if err:
            out["warning"] = f"音量を保存できませんでした(再起動すると元に戻ります): {err}"
        return out

    def reset_ephemeral(self) -> int:
        """operatorの一時上書きを全て破棄し、保存されている値に戻す(POST /ephemeral_reset)。破棄した項目数を返す。"""
        with self._lock:
            keys = sorted(self._ephemeral_keys)
            if not keys:
                return 0
            for key in keys:
                self._values[key] = self._persisted[key]
            self._ephemeral_keys.clear()
            snapshot = dict(self._values)
        logger.info("音量の一時上書きを破棄 -> %s", snapshot)
        for fn in list(self._listeners):
            try:
                fn(snapshot)
            except Exception:  # noqa: BLE001
                logger.exception("音量設定のリスナーでエラー")
        return len(keys)

    def set_bgm_compat(self, volume=None, duck_volume=None, persist: bool = True) -> dict:
        """既存の POST /bgm 用。0〜BGM_VOLUME_MAXに丸め(従来どおり範囲外は丸める)、ダッキングが通常音量を超えるなら通常音量に揃えて保存する。
        数値に直せない/非有限は ValueError/TypeError。"""
        updates = {}
        for key, raw in (("bgm_volume", volume), ("bgm_duck_volume", duck_volume)):
            if raw is None:
                continue
            v = float(raw)
            if not math.isfinite(v):
                raise ValueError("not finite")
            updates[key] = _clamp_bgm(v)
        if not updates:
            return self.status()
        return self.update(updates, persist=persist)

    def status(self) -> dict:
        return {
            "ok": True,
            **self.values(),
            "ephemeral_keys": sorted(self._ephemeral_keys),
            "defaults": dict(self.defaults),
            "limits": {"speech_volume": [0.0, SPEECH_VOLUME_MAX], "bgm_volume": [0.0, BGM_VOLUME_MAX],
                       "bgm_duck_volume": [0.0, "bgm_volume"], "announce_volume": [0.0, ANNOUNCE_VOLUME_MAX]},
            "from_file": self.from_file,
            "persisted": self.persisted,
            "updated_at": self.updated_at,
            "load_error": self.load_error,
        }


voice_levels = VoiceLevels(VOICE_LEVELS_FILE)


class BgmGain:
    """BGMの音量(ゲイン)制御。時計を差し替えられる(テスト用)ので、
    ダッキング/復帰の挙動を実時間を待たずに検証できる。"""

    def __init__(self, volume=0.25, duck_volume=0.07, hold_sec=1.5, down_sec=0.3,
                 up_sec=1.0, enabled=True, clock=time.monotonic) -> None:
        self.volume = _clamp_bgm(volume)
        self.duck_volume = _clamp_bgm(duck_volume)
        self.hold_sec = hold_sec
        self.down_sec = down_sec
        self.up_sec = up_sec
        self.enabled = enabled
        # 【2026-10-09】音声マスタースイッチOFF中はTrue。enabled(BGM個別のON/OFF)とは別に持ち、
        # マスターがOFFの間は目標ゲイン0(=BGMの設定は変えずに無音)、ONに戻せば設定どおりに復帰する。
        self.muted = False
        self.clock = clock
        self.gain = 0.0  # 0から始めてフェードイン
        self._duck_count = 0
        self._release_time = None
        self._lock = threading.Lock()
        self._last_logged_target = None

    def duck_begin(self) -> None:
        with self._lock:
            self._duck_count += 1

    def duck_end(self) -> None:
        with self._lock:
            self._duck_count = max(0, self._duck_count - 1)
            if self._duck_count == 0:
                self._release_time = self.clock()

    def is_ducked(self) -> bool:
        with self._lock:
            return self._ducked_locked()

    def _ducked_locked(self) -> bool:
        if self._duck_count > 0:
            return True
        return self._release_time is not None and self.clock() - self._release_time < self.hold_sec

    def active(self) -> bool:
        """BGMを鳴らすべきか(個別ONかつマスターONのとき)。"""
        return bool(self.enabled) and not self.muted

    def target(self) -> float:
        with self._lock:
            if not self.enabled or self.muted:
                return 0.0
            if self._ducked_locked():
                return min(self.duck_volume, self.volume)
            return self.volume

    def ceiling_for(self, gain: float) -> float:
        """BGMのピークの天井。発話中(ダッキング中)は BGM_DUCK_CEILING、通常は BGM_CEILING で、
        ゲインがダッキング音量→通常音量へ動くのに合わせて滑らかに行き来する。
        発話の天井(SPEECH_CEILING)との合計を MASTER_CEILING 未満に保つ(別ストリームの合算で割れないための予算)。"""
        span = self.volume - self.duck_volume
        if span < 0.05:  # ダッキングしない設定(通常=発話中): ゲインでは位置が決まらないので状態で決める
            frac = 0.0 if self.is_ducked() else 1.0
        else:
            frac = max(0.0, min(1.0, (gain - self.duck_volume) / span))
        return BGM_DUCK_CEILING + (BGM_CEILING - BGM_DUCK_CEILING) * frac

    def advance(self, dt: float):
        """dt秒ぶんゲインを目標へ近づけ、(開始ゲイン, 終了ゲイン)を返す。"""
        tgt = self.target()
        g0 = self.gain
        span = max(self.volume - self.duck_volume, 0.05)
        if tgt > g0:
            g1 = min(tgt, g0 + dt * span / max(self.up_sec, 1e-3))
        else:
            g1 = max(tgt, g0 - dt * span / max(self.down_sec, 1e-3))
        self.gain = g1
        if tgt != self._last_logged_target:
            self._last_logged_target = tgt
            logger.info("BGM目標ゲイン -> %.3f (現在%.3f, %s)", tgt, g0,
                        "OFF" if not self.active() else ("ダッキング" if tgt < self.volume else "通常"))
        return g0, g1


class BgmPlayer:
    """BGM_DIRの音楽をシャッフルして無限ループ再生するバックグラウンドスレッド。"""

    BLOCK = 1024

    def __init__(self) -> None:
        self.dir = BGM_DIR
        self.device = os.environ.get("BGM_OUTPUT_DEVICE") or OUTPUT_DEVICE
        enabled = os.environ.get("BGM_ENABLED", "1").strip().lower() not in ("", "0", "false", "off", "no")
        self.gain = BgmGain(
            volume=voice_levels.bgm_volume,  # 【2026-10-09】保存ファイル > 環境変数 > 既定値(VoiceLevels)
            duck_volume=voice_levels.bgm_duck_volume,
            hold_sec=_env_float("BGM_HOLD_SEC", 1.5),
            down_sec=_env_float("BGM_DOWN_SEC", 0.3),
            up_sec=_env_float("BGM_UP_SEC", 1.0),
            enabled=enabled,
        )
        self.playing = ""
        self.stream_ok = False
        self._wake = threading.Event()
        self._thread = None
        # ALSAのhwデバイス直指定のときは、BGMが掴んで発話の再生を妨げる恐れがあるので既定で動かさない
        # (PipeWire/Pulse経由ならミキシングされるので共存できる)。
        self.device_ok = self.device in (None, "pipewire", "pulse", "default") or \
            os.environ.get("BGM_FORCE") == "1"
        if not self.device_ok:
            logger.warning("BGM: 出力デバイス %r は他の音と共有できない可能性があるため無効(BGM_FORCE=1で強制)", self.device)

    def start(self) -> None:
        if not self.device_ok:
            return
        self._thread = threading.Thread(target=self._run, name="bgm", daemon=True)
        self._thread.start()

    def files(self) -> list:
        try:
            names = sorted(os.listdir(self.dir))
        except OSError:
            return []
        return [os.path.join(self.dir, n) for n in names if n.lower().endswith(BGM_EXTS)]

    def set_muted(self, muted: bool) -> None:
        """音声マスタースイッチから呼ぶ。BGM個別のenabled設定は変えない。"""
        self.gain.muted = bool(muted)
        self._wake.set()

    def set_params(self, enabled=None, volume=None, duck_volume=None) -> None:
        if volume is not None:
            self.gain.volume = _clamp_bgm(volume)
        if duck_volume is not None:
            self.gain.duck_volume = _clamp_bgm(duck_volume)
        if enabled is not None:
            self.gain.enabled = bool(enabled)
            self._wake.set()

    def status(self) -> dict:
        return {
            "enabled": self.gain.enabled,
            "muted": self.gain.muted,  # 音声マスターOFFで止められているか(enabledとは別)
            "available": self.device_ok,
            "playing": self.playing,
            "stream_ok": self.stream_ok,
            "volume": self.gain.volume,
            "duck_volume": self.gain.duck_volume,
            "gain": round(self.gain.gain, 4),
            "ducked": self.gain.is_ducked(),
            "dir": self.dir,
            "files": len(self.files()),
        }

    def _run(self) -> None:
        last = ""
        while True:
            try:
                if not self.gain.active() and self.gain.gain <= 0.0005:
                    self.playing = ""
                    self._wake.wait(timeout=2.0)
                    self._wake.clear()
                    continue
                files = self.files()
                if not files:
                    self.playing = ""
                    logger.warning("BGM: %s に音楽ファイルがありません(wav/flac/ogg/mp3)。30秒後に再確認", self.dir)
                    self._wake.wait(timeout=30.0)
                    self._wake.clear()
                    continue
                random.shuffle(files)
                if len(files) > 1 and files[0] == last:
                    files.append(files.pop(0))
                for path in files:
                    last = path
                    if not self.gain.active() and self.gain.gain <= 0.0005:
                        break
                    self._play_file(path)
            except Exception as exc:  # noqa: BLE001 - BGMの失敗で発話を巻き込まない
                self.playing = ""
                self.stream_ok = False
                logger.warning("BGMスレッドでエラー(15秒後に再試行): %s", exc)
                time.sleep(15.0)

    def _play_file(self, path: str) -> None:
        import numpy as np

        try:
            f = sf.SoundFile(path)
        except Exception as exc:  # noqa: BLE001
            logger.warning("BGM: %s を開けません、スキップ: %s", path, exc)
            time.sleep(1.0)
            return
        with f:
            sr = f.samplerate
            logger.info("BGM再生開始: %s (%dHz %dch)", os.path.basename(path), sr, f.channels)
            self.playing = os.path.basename(path)
            # 【2026-10-09】倍率>1のときの音割れ防止+発話との合算の予算。遅延は約8msで、ゲイン<=1の素の再生では波形は変わらない
            limiter = LookaheadLimiter(sr, channels=2)
            with sd.OutputStream(samplerate=sr, channels=2, dtype="float32",
                                 device=self.device, blocksize=self.BLOCK, latency=0.1) as stream:
                self.stream_ok = True
                while True:
                    if not self.gain.active() and self.gain.gain <= 0.0005:
                        break
                    block = f.read(self.BLOCK, dtype="float32", always_2d=True)
                    n = len(block)
                    if n == 0:
                        break
                    if block.shape[1] == 1:
                        block = np.repeat(block, 2, axis=1)
                    elif block.shape[1] > 2:
                        block = block[:, :2]
                    g0, g1 = self.gain.advance(n / float(sr))
                    out = block * np.linspace(g0, g1, n, dtype="float32")[:, None]
                    ceil = np.linspace(self.gain.ceiling_for(g0), self.gain.ceiling_for(g1), n)
                    out = np.ascontiguousarray(limiter.process(out, ceil))
                    stream.write(out)
        self.playing = ""


bgm = BgmPlayer()
voice_levels.add_listener(
    lambda v: bgm.set_params(volume=v["bgm_volume"], duck_volume=v["bgm_duck_volume"])
)


# ---------------------------------------------------------------------------
# 【2026-10-09追加】音声の全体ON/OFF(マスタースイッチ)。docs/voice_master.md
# OFFの間は 発話(合成・再生)・Ollamaによる文面生成・BGM のすべてを止める(完全に無音)。
# 状態はファイルに原子的に保存し、再起動後も維持する。ファイルが無い/壊れている/想定外の値
# のときは既定のON(従来と同じ動き)。voice_bridgeプロセスや他のAPIはOFFでも動き続ける。
# ---------------------------------------------------------------------------
VOICE_MASTER_FILE = os.path.expanduser(
    os.environ.get("VOICE_MASTER_FILE", "~/.nokosahen_voice_master.json")
)


class VoiceMaster:
    """マスタースイッチの状態(スレッドセーフ)+永続化。変更時にリスナーを呼ぶ(BGM消音・キュー破棄など)。"""

    def __init__(self, path: str) -> None:
        self.path = os.path.expanduser(path)
        self._lock = threading.Lock()
        self._listeners: list = []
        self.load_error = ""
        self.persisted = True  # 直近の保存に成功したか(ファイルを書けない環境の目印)
        self.updated_at = ""
        self.muted_requests = 0  # OFF中に受け付けて捨てた発話要求の数(起動後の累計)
        self._enabled = self._load()
        self._persisted_enabled = self._enabled  # 保存されている値(operatorの一時上書きの戻し先、docs/accounts.md)
        self.ephemeral = False

    def _load(self) -> bool:
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                doc = json.loads(f.read())
        except FileNotFoundError:
            return True
        except (OSError, ValueError) as exc:
            self.load_error = f"マスター設定ファイルを読めないためONで起動しました: {exc}"
            logger.warning("%s: %s", self.load_error, self.path)
            return True
        en = doc.get("enabled") if isinstance(doc, dict) else None
        if not isinstance(en, bool):
            self.load_error = "マスター設定ファイルの内容が不正なためONで起動しました"
            logger.warning("%s: %s", self.load_error, self.path)
            return True
        self.updated_at = str(doc.get("updated_at", ""))[:40]
        return en

    @property
    def enabled(self) -> bool:
        return self._enabled

    def add_listener(self, fn) -> None:
        self._listeners.append(fn)

    def count_muted(self) -> None:
        with self._lock:
            self.muted_requests += 1

    def _save(self, enabled: bool) -> str:
        """原子的に保存(同じフォルダに一時ファイル->fsync->置換)。失敗時はエラー文を返す。"""
        doc = {"version": 1, "enabled": enabled,
               "updated_at": datetime.datetime.now().isoformat(timespec="seconds")}
        directory = os.path.dirname(self.path) or "."
        tmp_path = None
        try:
            os.makedirs(directory, exist_ok=True)
            fd, tmp_path = tempfile.mkstemp(prefix=".nokosahen_voice_master_", suffix=".tmp", dir=directory)
            with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as f:
                f.write(json.dumps(doc, ensure_ascii=False, indent=2) + "\n")
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp_path, self.path)
            tmp_path = None
            self.updated_at = doc["updated_at"]
            return ""
        except OSError as exc:
            return str(exc)
        finally:
            if tmp_path:
                try:
                    os.remove(tmp_path)
                except OSError:
                    pass

    def set_enabled(self, enabled: bool, persist: bool = True) -> dict:
        """切り替える。保存に失敗しても、メモリ上の状態は切り替える(OFFにしたいのに音が止まらない方が危険)。
        その場合 persisted=False と warning を返し、再起動すると以前の状態に戻る。"""
        if not isinstance(enabled, bool):
            raise TypeError("enabled must be bool")
        with self._lock:
            changed = enabled != self._enabled
            self._enabled = enabled
            if persist:
                err = self._save(enabled)
                self.persisted = not err
                self._persisted_enabled = enabled
                self.ephemeral = False  # 管理者の保存で一時上書きは破棄される
                if not err:
                    self.load_error = ""
            else:
                err = ""  # operator: メモリ上だけ切り替える(保存ファイルは変えない)
                self.ephemeral = enabled != self._persisted_enabled
        if err:
            logger.error("マスター設定を保存できませんでした(再起動で元に戻ります): %s", err)
        logger.info("音声マスター -> %s", "ON" if enabled else "OFF")
        for fn in list(self._listeners):
            try:
                fn(enabled)
            except Exception:  # noqa: BLE001 - リスナーの失敗でスイッチ自体を失敗させない
                logger.exception("マスタースイッチのリスナーでエラー")
        out = self.status()
        out["changed"] = changed
        if err:
            out["warning"] = f"状態を保存できませんでした(再起動すると元に戻ります): {err}"
        return out

    def reset_ephemeral(self) -> bool:
        """operatorの一時上書きを破棄し、保存されている値に戻す(POST /ephemeral_reset)。戻したらTrue。"""
        with self._lock:
            if self._enabled == self._persisted_enabled:
                self.ephemeral = False
                return False
            self._enabled = self._persisted_enabled
            self.ephemeral = False
            enabled = self._enabled
        logger.info("音声マスターの一時上書きを破棄 -> %s", "ON" if enabled else "OFF")
        for fn in list(self._listeners):
            try:
                fn(enabled)
            except Exception:  # noqa: BLE001
                logger.exception("マスタースイッチのリスナーでエラー")
        return True

    def status(self) -> dict:
        return {
            "ok": True,
            "enabled": self._enabled,
            "ephemeral": self.ephemeral,
            "persisted": self.persisted,
            "updated_at": self.updated_at,
            "muted_requests": self.muted_requests,
            "load_error": self.load_error,
        }


def parse_master_request(data) -> tuple:
    """POST /voice_master の本文を検証する。(enabled, error)。enabledはbool(true/false)のみ。"""
    if not isinstance(data, dict):
        return None, "本文はJSONオブジェクトで指定してください"
    if set(data) - {"enabled"}:
        return None, "指定できる項目は enabled のみです"
    if not isinstance(data.get("enabled"), bool):
        return None, "enabledは true/false で指定してください"
    return data["enabled"], ""


voice_master = VoiceMaster(VOICE_MASTER_FILE)
bgm.gain.muted = not voice_master.enabled  # 起動時点でOFFならBGMも鳴らさない


_last_speak_success_time: float = 0.0
_consecutive_speak_failures: int = 0


def speak_voicevox(text: str, trace: Optional[dict] = None) -> None:
    """VOICEVOXで音声合成し、Linux(sounddevice)で再生する。

    traceを渡すと tts_ms(audio_query+synthesisの合計時間)/play_ms(再生時間)/speak_ok/
    (失敗時)speak_error を書き込む。
    """
    global _last_speak_success_time, _consecutive_speak_failures
    if trace is None:
        trace = {}
    t_tts = time.monotonic()
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
            # 【2026-07-21】Jetson実機では本番相当の文章長だと30秒で足りず
            # タイムアウトすることがある(PoC担当からの報告、2分に伸ばしたら
            # だいたい上手くいったとのこと)ため、120秒に延長。
            timeout=120,
        )
        synth_res.raise_for_status()
        trace["tts_ms"] = round((time.monotonic() - t_tts) * 1000.0, 1)

        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tf:
            tf.write(synth_res.content)
            wav_path = tf.name

        try:
            if not voice_master.enabled:
                # 合成中にマスターがOFFになった: 再生しない(失敗ではないので失敗カウンタは触らない)
                trace["muted"] = True
                logger.info("音声マスターOFFのため再生を中止: %s", text[:40])
                return
            data, samplerate = sf.read(wav_path, dtype="float32")
            # 【2026-10-09】発話音量(倍率)。次の発話から反映。ピークが大きいときだけソフトリミッタで歪みを抑える
            data = apply_speech_gain(data, voice_levels.speech_volume, samplerate)
            t_play = time.monotonic()
            try:
                _play_audio(data, samplerate)
            finally:
                trace["play_ms"] = round((time.monotonic() - t_play) * 1000.0, 1)
        finally:
            os.remove(wav_path)

        logger.info("再生完了: %s", text[:40])
        trace["speak_ok"] = True
        _last_speak_success_time = time.time()
        _consecutive_speak_failures = 0
    except Exception as exc:  # noqa: BLE001
        trace["speak_ok"] = False
        trace["speak_error"] = _short_err(exc)
        if "tts_ms" not in trace:
            trace["tts_ms"] = round((time.monotonic() - t_tts) * 1000.0, 1)
        _consecutive_speak_failures += 1
        logger.error("VOICEVOX合成/再生エラー(連続%d回目): %s", _consecutive_speak_failures, exc)
        logger.error("VOICEVOX Engineが起動しているか確認してください: %s", VOICEVOX_URL)


class SpeechQueue:
    """発話リクエストを1件ずつ順番に処理するワーカー。"""

    def __init__(self) -> None:
        self._queue: "queue.Queue[dict]" = queue.Queue(maxsize=MAX_QUEUE_SIZE)
        self._speaking = False
        self._stopping = False
        self._last_key = ""
        self._last_key_time = 0.0
        self._lock = threading.Lock()
        self._worker_thread = threading.Thread(target=self._worker, daemon=True)
        self._worker_thread.start()

    def enqueue(
        self, text: str, generate: bool, context: Optional[dict] = None
    ) -> bool:
        """キューに積む。停止中、またはキューが溢れて捨てた場合はFalseを返す。

        context(menu_items/残数状況/fallback_text)が渡された場合、実際の
        発話文面はワーカー側でAIに1から考えさせる(generate_utterance_from_context)。
        contextが無ければ従来通りtext(+generateならAIによる言い換え)を使う。
        """
        # enqueued_mono/request_id: 待ち時間・総所要時間の計測用(受付時刻はここで確定させる)
        item = {
            "text": text, "generate": generate, "context": context,
            "enqueued_mono": time.monotonic(), "request_id": uuid.uuid4().hex[:12],
        }
        # 同一内容の依頼が短時間に繰り返し来ても(送信側の再送・ループ等)積まない。
        # 【2026-10-06】しきい値は設定(min_repeat_sec、設定ファイル>環境変数)から毎回読む。
        key = json.dumps([text, generate, context], sort_keys=True, ensure_ascii=False)
        now = time.time()
        min_repeat = float(get_voice_settings()["min_repeat_sec"])
        with self._lock:
            if self._stopping:
                return False
            if key == self._last_key and now - self._last_key_time < min_repeat:
                logger.info("同一内容の発話依頼が%.0f秒以内に再度届いたため無視しました", min_repeat)
                return True
            self._last_key = key
            self._last_key_time = now
        try:
            self._queue.put_nowait(item)
            return True
        except queue.Full:
            # VOICEVOX/Ollamaが長時間ダウンして溜まり続けている状況。
            # 一番古い未処理分を1件捨ててから積み直す(無制限増加を防ぐ)。
            try:
                self._queue.get_nowait()
                self._queue.task_done()
            except queue.Empty:
                pass
            logger.warning("発話キューが上限(%d)に達したため、最も古いリクエストを破棄しました", MAX_QUEUE_SIZE)
            try:
                self._queue.put_nowait(item)
                return True
            except queue.Full:
                return False

    def clear(self) -> int:
        """未処理の依頼をすべて捨てる(マスターOFF時)。同一内容の抑止も解除し、ON後の最初の発話を通す。"""
        n = 0
        while True:
            try:
                self._queue.get_nowait()
                self._queue.task_done()
                n += 1
            except queue.Empty:
                break
        with self._lock:
            self._last_key = ""
            self._last_key_time = 0.0
        return n

    def status(self) -> dict:
        with self._lock:
            speaking = self._speaking
        return {
            "speaking": speaking,
            "queue_length": self._queue.qsize(),
            "last_speak_success_time": _last_speak_success_time,
            "consecutive_speak_failures": _consecutive_speak_failures,
        }

    def stop(self, timeout_sec: float = 5.0) -> None:
        """新規受付を止め、処理中/キュー中のリクエストの完了を待つ(タイムアウトあり)。"""
        with self._lock:
            self._stopping = True
        remaining = self._queue.qsize()
        if remaining:
            logger.info("シャットダウン: 残り%d件の発話が完了するのを待ちます", remaining)
        self._worker_thread.join(timeout=timeout_sec)
        dropped = self._queue.qsize()
        if dropped:
            logger.warning("シャットダウンタイムアウトのため%d件の発話が未処理のまま終了します", dropped)

    @staticmethod
    def _process_item(item: dict) -> None:
        """1件を 生成(Ollama) -> 履歴記録 -> 合成・再生(VOICEVOX) の順に処理し、各段階の時間を記録する。

        履歴は再生前に1行作り(従来どおり、再生に失敗しても「話そうとした」記録は残る)、
        再生が終わったら同じ行へ tts_ms/play_ms/total_ms/speak_ok を追記する。
        """
        if not voice_master.enabled:
            return  # 取り出した後にOFFになった: 生成も再生もしない
        start = time.monotonic()
        enq = item.get("enqueued_mono", start)
        settings = get_voice_settings()  # 設定は1件ごとに読む(保存したら次の発話から効く)
        trace: dict = {}
        context = item.get("context")
        generated = False  # Ollamaで作った文面か(先頭N文への短縮の対象)
        if isinstance(context, dict):
            kind = "context"
            trace["phase"] = str(context.get("phase", "initial"))
            if item.get("generate", True):
                text = generate_utterance_from_context(context, trace=trace, settings=settings)
                generated = True
            else:
                text = str(context.get("fallback_text", "")).strip() or item["text"]
                trace.update(used_menu_names=False, used_remaining=False)
        else:
            kind = "text"
            text = item["text"]
            if item.get("generate"):
                text = generate_with_ollama(text, trace=trace, settings=settings)
                generated = True
            else:
                trace.update(used_menu_names=False, used_remaining=False)
        if generated and not trace.get("fallback_reason"):
            # 【2026-10-30】Ollamaの返答は先頭MAX_SENTENCES文に切る(長話防止)。経路(text/context)にかかわらず
            # ここ(履歴記録の直前・if/elseの外)で1回だけ行う。フォールバック文や固定文は対象外。
            short = limit_sentences(text, MAX_SENTENCES)
            if short != text:
                logger.info("発話を先頭%d文に短縮しました(%d -> %d文字)", MAX_SENTENCES, len(text), len(short))
                text = short
        if not voice_master.enabled:
            # Ollamaの生成中にOFFになった: 文面は捨てる(履歴にも残さない・合成も再生もしない)
            logger.info("音声マスターOFFのため生成済みの発話を破棄しました")
            return
        meta = {
            "request_id": item.get("request_id"),
            "kind": kind,
            "queue_wait_ms": round((start - enq) * 1000.0, 1),
            **{k: trace.get(k) for k in (
                "phase", "llm_ms", "used_menu_names", "used_remaining", "fallback_reason",
                "trimmed", "error", "model", "prompt", "temperature", "num_predict")},
        }
        row_id = _record_utterance(text, meta)
        speak_voicevox(text, trace)
        if trace.get("muted"):
            _update_utterance(row_id, {"speak_ok": 0, "error": "音声マスターOFFのため再生せず"})
            return
        after = {
            "tts_ms": trace.get("tts_ms"),
            "play_ms": trace.get("play_ms"),
            "speak_ok": trace.get("speak_ok"),
            "total_ms": round((time.monotonic() - enq) * 1000.0, 1),
        }
        if trace.get("speak_error"):
            after["error"] = "; ".join(x for x in (trace.get("error"), "TTS " + trace["speak_error"]) if x)[:300]
        _update_utterance(row_id, after)

    def _worker(self) -> None:
        while True:
            item = self._queue.get()
            with self._lock:
                self._speaking = True
            bgm.gain.duck_begin()  # 合成開始の時点でBGMを下げる(必ずfinallyで解除)
            try:
                self._process_item(item)
            except Exception:  # noqa: BLE001 - キューを止めないため握りつぶしてログのみ
                logger.exception("発話処理中に予期しないエラー")
            finally:
                bgm.gain.duck_end()
                with self._lock:
                    self._speaking = False
                self._queue.task_done()

            with self._lock:
                should_stop = self._stopping and self._queue.empty()
            if should_stop:
                return


speech_queue = SpeechQueue()


# ---------------------------------------------------------------------------
# 【2026-10-13追加】状況アナウンス(「探索中です」等の定型文を、合成済み音声のキャッシュから繰り返し/1回再生)。
# 中核は announce_engine.py(docs/announcements.md)。ここは VOICEVOX・再生・音量・ダッキング・マスターとの接続だけ。
# 音量: 実効倍率 = min(ANNOUNCE_VOLUME_MAX, announce_volume(/voice_levels) × イベントごとのvolume)。
# 再生は発話と同じ apply_speech_gain(天井SPEECH_CEILING=0.65)を通し、発話とは _play_lock で同時に鳴らさないので、
# 発話中BGMの天井(0.29)との合計 0.94 < 0.95 の予算は発話と同じ。
# ---------------------------------------------------------------------------
ANNOUNCE_CACHE_DIR = os.path.expanduser(os.environ.get("VOICE_ANNOUNCE_CACHE_DIR", "~/.nokosahen_announce_cache"))
ANNOUNCE_CACHE_MAX = int(os.environ.get("VOICE_ANNOUNCE_CACHE_MAX", "64"))
ANNOUNCE_KEEPALIVE_SEC = _env_float("VOICE_ANNOUNCE_KEEPALIVE_SEC", 15.0)


def synthesize_voicevox_wav(text: str) -> bytes:
    """VOICEVOXで合成してwavのバイト列を返す(再生はしない。アナウンスのキャッシュ用)。失敗は例外。"""
    query_res = requests.post(f"{VOICEVOX_URL}/audio_query", params={"text": text, "speaker": SPEAKER_ID}, timeout=10)
    query_res.raise_for_status()
    query_data = query_res.json()
    query_data.update(VOICE_QUERY_TUNING)
    synth_res = requests.post(f"{VOICEVOX_URL}/synthesis?speaker={SPEAKER_ID}", json=query_data, timeout=120)
    synth_res.raise_for_status()
    return synth_res.content


def _announce_play(data, samplerate: int, event_volume: float) -> None:
    gain = min(ANNOUNCE_VOLUME_MAX, voice_levels.announce_volume * float(event_volume))
    _play_audio(apply_speech_gain(data, gain, samplerate), samplerate)


def _announce_speech_busy() -> bool:
    st = speech_queue.status()
    return bool(st["speaking"]) or st["queue_length"] > 0


ANNOUNCE_CACHE_MAX_MB = _env_float("VOICE_ANNOUNCE_CACHE_MAX_MB", 300.0)  # ディスク容量の上限(文化祭の長い音声が増えても溢れさせない)
announce_cache = AnnounceCache(
    ANNOUNCE_CACHE_DIR, lambda text: synthesize_long(text, synthesize_voicevox_wav),  # 80字以下は従来どおり1回で合成
    tag=json.dumps([SPEAKER_ID, VOICE_QUERY_TUNING], sort_keys=True), max_items=ANNOUNCE_CACHE_MAX,
    max_bytes=int(ANNOUNCE_CACHE_MAX_MB * 1024 * 1024),
)
announce_prepare = PrepareWorker(announce_cache)  # 文化祭の出店紹介: 事前に1回だけ合成して保存(再生中は合成しない)
announce_engine = AnnounceEngine(
    announce_cache,
    decode=lambda blob: sf.read(io.BytesIO(blob), dtype="float32"),
    play=_announce_play,
    is_muted=lambda: not voice_master.enabled,
    speech_busy=_announce_speech_busy,
    duck_begin=lambda: bgm.gain.duck_begin(),
    duck_end=lambda: bgm.gain.duck_end(),
    keepalive_sec=ANNOUNCE_KEEPALIVE_SEC,
    stop_playback=lambda: sd.stop(),
)


def _apply_master(enabled: bool) -> None:
    """マスターの状態をBGM・発話キュー・再生中の音に反映する。"""
    bgm.set_muted(not enabled)
    if not enabled:
        dropped = speech_queue.clear()
        if dropped:
            logger.info("音声マスターOFF: 未処理の発話%d件を破棄", dropped)
        try:
            sd.stop()  # 再生中の発話をその場で止める(BGMは別ストリームでゲイン0へフェード)
        except Exception as exc:  # noqa: BLE001
            logger.warning("再生の停止に失敗: %s", exc)


voice_master.add_listener(_apply_master)
voice_master.add_listener(announce_engine.on_master_change)


def make_handler():
    class Handler(BaseHTTPRequestHandler):
        # 【2026-11-05】読み取りのタイムアウト(接続だけして何も送らない相手にスレッドを占有されない)。
        timeout = 15
        def log_message(self, fmt: str, *args) -> None:  # noqa: A003
            logger.debug(fmt, *args)

        def _send_json(self, status: int, payload: dict) -> None:
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _forbidden(self) -> bool:
            """接続元IPが許可リスト(VOICE_BRIDGE_ALLOW)外なら403を返してTrue。"""
            ip = self.client_address[0]
            if client_allowed(ip, ALLOW):
                return False
            logger.warning("許可リスト外の接続元を拒否しました: %s %s", ip, self.path[:40])
            self._send_json(HTTPStatus.FORBIDDEN, {"ok": False, "error": "forbidden", "message": "この接続元は許可されていません"})
            return True

        def do_GET(self) -> None:  # noqa: N802
            if self._forbidden():
                return
            parsed = urlparse(self.path)
            path = parsed.path
            if path == "/announce_state":
                texts = [t for t in parse_qs(parsed.query).get("text", []) if 0 < len(t) <= 80]
                self._send_json(HTTPStatus.OK, {**announce_engine.state(texts), "announce_volume": voice_levels.announce_volume,
                                                "prepare": announce_prepare.summary()})
                return
            if path == "/status":
                st = speech_queue.status()
                st.update({"bgm_enabled": bgm.gain.enabled, "bgm_playing": bgm.playing,
                           "bgm_gain": round(bgm.gain.gain, 4),
                           "voice_enabled": voice_master.enabled,
                           "speech_volume": voice_levels.speech_volume,
                           "bgm_volume": voice_levels.bgm_volume,
                           "bgm_duck_volume": voice_levels.bgm_duck_volume,
                           "announce_volume": voice_levels.announce_volume})
                self._send_json(HTTPStatus.OK, st)
                return
            if path == "/bgm":
                self._send_json(HTTPStatus.OK, bgm.status())
                return
            if path == "/voice_master":
                self._send_json(HTTPStatus.OK, voice_master.status())
                return
            if path == "/voice_levels":
                self._send_json(HTTPStatus.OK, voice_levels.status())
                return
            if path == "/utterances":
                # 読み取り専用(ダッシュボードの発話履歴が別Jetsonから取得する)。
                qs = parse_qs(parsed.query)
                since = (qs.get("since") or [""])[0]
                try:
                    limit = int((qs.get("limit") or ["1000"])[0])
                except ValueError:
                    self._send_json(HTTPStatus.BAD_REQUEST, {"error": "invalid limit"})
                    return
                try:
                    items = fetch_utterances(since, limit)
                except sqlite3.Error as exc:
                    logger.warning("発話履歴の読み取りに失敗: %s", exc)
                    items = []
                self._send_json(HTTPStatus.OK, {"utterances": items})
                return
            if path == "/voice_settings":
                self._send_json(HTTPStatus.OK, settings_payload())
                return
            if path == "/voice_metrics":
                # 【2026-10-06追加】発話ごとの計測値と統計(読み取り専用)。
                # ?limit=N(1〜1000、既定50) / ?id=行id(1件+プロンプト全文)
                qs = parse_qs(parsed.query)
                try:
                    limit = int((qs.get("limit") or ["50"])[0])
                    row_id = int(qs["id"][0]) if qs.get("id") else None
                except ValueError:
                    self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "invalid limit/id"})
                    return
                if limit < 1 or limit > 1000:
                    self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "limitは1〜1000"})
                    return
                try:
                    rows = fetch_metrics_rows(limit, row_id)
                except sqlite3.Error as exc:
                    logger.warning("発話計測値の読み取りに失敗: %s", exc)
                    rows = []
                if row_id is not None:
                    if not rows:
                        self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})
                    else:
                        self._send_json(HTTPStatus.OK, {"ok": True, "row": rows[0]})
                    return
                self._send_json(
                    HTTPStatus.OK,
                    {"ok": True, "limit": limit, "rows": rows, "stats": compute_metric_stats(rows)},
                )
                return
            self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

        def _handle_announce(self, path: str, data: dict) -> None:
            """【2026-10-13】状況アナウンス。検証エラーは400(何も変えない)。マスターOFF中は受理するが鳴らさない(200+muted)。"""
            if path == "/announce_prepare":
                req, errors = parse_prepare_request(data)
                if errors:
                    self._send_json(HTTPStatus.BAD_REQUEST,
                                    {"ok": False, "errors": errors, "error": errors[0], "message": errors[0]})
                    return
                if req["start"]:  # 合成は背景で1本ずつ(この応答は待たない。VOICEVOXが落ちていても200で、状態はfailedになる)
                    announce_prepare.request(req["texts"], force=req["force"])
                self._send_json(HTTPStatus.OK, {"ok": True, "items": announce_prepare.status(req["texts"]),
                                                "prepare": announce_prepare.summary()})
                return
            if path == "/announce_loop":
                req, errors = parse_loop_request(data)
            else:
                req, errors = parse_once_request(data)
            if errors:
                self._send_json(HTTPStatus.BAD_REQUEST,
                                {"ok": False, "errors": errors, "error": errors[0], "message": errors[0]})
                return
            muted = not voice_master.enabled
            if path == "/announce_loop":
                try:
                    if req.get("playlist"):
                        st = announce_engine.set_loop(req["id"], req["enabled"], interval=req.get("interval_sec") or 60.0,
                                                      parts=req.get("parts") or [],
                                                      gap=req.get("gap_sec") or 5.0)
                    else:
                        st = announce_engine.set_loop(req["id"], req["enabled"], req.get("text", ""),
                                                      req.get("interval_sec") or 8.0, req["volume"])
                except ValueError as exc:
                    self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "errors": [str(exc)], "error": str(exc), "message": str(exc)})
                    return
                self._send_json(HTTPStatus.OK, {"ok": True, "muted": muted, "loop": st})
                return
            if muted:
                voice_master.count_muted()
                self._send_json(HTTPStatus.OK, {"ok": True, "muted": True, "accepted": False})
                return
            announce_engine.enqueue_once(req["id"], req["text"], req["volume"], req["urgent"], req.get("cached_only", False))
            self._send_json(HTTPStatus.ACCEPTED, {"ok": True, "muted": False, "accepted": True})

        def do_POST(self) -> None:  # noqa: N802
            if self._forbidden():
                return
            path = urlparse(self.path).path
            if path not in ("/speak", "/bgm", "/voice_settings", "/voice_preview", "/voice_master", "/voice_levels",
                            "/announce", "/announce_loop", "/announce_prepare", "/ephemeral_reset"):
                self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})
                return

            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                self._send_json(HTTPStatus.BAD_REQUEST, {"error": "invalid Content-Length"})
                return
            if length < 0 or length > MAX_CONTENT_LENGTH:
                self._send_json(HTTPStatus.BAD_REQUEST, {"error": "invalid Content-Length"})
                return

            try:
                raw = self.rfile.read(length).decode("utf-8")
                # 新しい設定系のエンドポイントはNaN/Infinityを構文の時点で拒否する
                data = json.loads(
                    raw,
                    parse_constant=VoiceSettingsStore._reject_constant
                    if path in ("/voice_settings", "/voice_preview", "/voice_levels", "/announce", "/announce_loop",
                                "/announce_prepare")
                    else None,
                )
            except Exception:  # noqa: BLE001
                self._send_json(HTTPStatus.BAD_REQUEST, {"error": "invalid json"})
                return

            if not isinstance(data, dict):
                self._send_json(HTTPStatus.BAD_REQUEST, {"error": "body must be a JSON object"})
                return

            if path in ("/announce", "/announce_loop", "/announce_prepare"):
                self._handle_announce(path, data)
                return

            # 【2026-10-13】弱いアカウント(operator)の一時的な変更: persist:false は保存せずメモリだけに反映する。
            # ダッシュボードの起動時に /ephemeral_reset で破棄する。docs/accounts.md
            persist = data.pop("persist", True) is not False
            if path == "/ephemeral_reset":
                self._send_json(HTTPStatus.OK, {
                    "ok": True,
                    "levels": voice_levels.reset_ephemeral(),
                    "master": voice_master.reset_ephemeral(),
                    "settings": settings_store.reset_ephemeral(),
                })
                return

            if path == "/voice_master":
                enabled, err = parse_master_request(data)
                if err:
                    self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": err, "message": err})
                    return
                self._send_json(HTTPStatus.OK, voice_master.set_enabled(enabled, persist=persist))
                return

            if path == "/voice_levels":
                updates, errors = validate_levels_update(data, voice_levels.values())
                if not errors and not updates:
                    errors = ["変更する項目がありません"]
                if errors:
                    self._send_json(HTTPStatus.BAD_REQUEST,
                                    {"ok": False, "errors": errors, "error": errors[0], "message": errors[0]})
                    return
                self._send_json(HTTPStatus.OK, voice_levels.update(updates, persist=persist))
                return

            if path == "/voice_settings":
                updates, errors = validate_settings_update(data)
                if not errors and not updates:
                    errors = ["変更する項目がありません"]
                if errors:
                    self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "errors": errors})
                    return
                ok, errors = settings_store.update(updates, persist=persist)
                if not ok:
                    self._send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "errors": errors})
                    return
                logger.info("音声設定を保存: %s", sorted(updates))
                self._send_json(HTTPStatus.OK, settings_payload())
                return

            if path == "/voice_preview":
                ctx = data.get("context")
                if not isinstance(ctx, dict):
                    self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "errors": ["contextが必要です"]})
                    return
                overrides, errors = validate_settings_update(data.get("settings") or {})
                if errors:
                    self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "errors": errors})
                    return
                if not voice_master.enabled:
                    # お試し生成はOllamaを動かす(CPU/GPUを使う)ので、全体OFF中は実行しない
                    msg = "音声が全体OFFのため、お試し生成はできません。ONに戻してから実行してください"
                    self._send_json(HTTPStatus.CONFLICT, {"ok": False, "muted": True, "errors": [msg]})
                    return
                if not _preview_lock.acquire(blocking=False):
                    self._send_json(HTTPStatus.TOO_MANY_REQUESTS,
                                    {"ok": False, "errors": ["別のお試し生成を実行中です。終わってからもう一度押してください"]})
                    return
                try:
                    result = preview_utterance(ctx, overrides)
                except Exception as exc:  # noqa: BLE001
                    logger.exception("お試し生成で予期しないエラー")
                    self._send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "errors": [_short_err(exc)]})
                    return
                finally:
                    _preview_lock.release()
                self._send_json(HTTPStatus.OK, result)
                return

            if path == "/bgm":
                try:
                    en = data.get("enabled")
                    if isinstance(en, str):
                        en = en.strip().lower() not in ("", "false", "0", "off", "no")
                    # 【2026-10-09】音量は /voice_levels と同じ値(保存もされる)。enabledだけ従来どおりBGM個別
                    voice_levels.set_bgm_compat(volume=data.get("volume"), duck_volume=data.get("duck_volume"),
                                                persist=persist)
                    bgm.set_params(enabled=None if en is None else bool(en))
                except (TypeError, ValueError):
                    self._send_json(HTTPStatus.BAD_REQUEST, {"error": "invalid value"})
                    return
                logger.info("BGM設定変更: %s", {k: data.get(k) for k in ("enabled", "volume", "duck_volume")})
                self._send_json(HTTPStatus.OK, bgm.status())
                return

            context_raw = data.get("context")
            context = context_raw if isinstance(context_raw, dict) else None

            text = str(data.get("text", "")).strip()
            if not text and context is None:
                self._send_json(HTTPStatus.BAD_REQUEST, {"error": "text or context is required"})
                return

            # bool("false")はTrueになってしまうため、真偽値そのもの以外は
            # 明示的に文字列判定する("false"/"0"/空文字は偽として扱う)。
            # contextモード(menu_itemsからAIに文面を考えさせる)は既定でtrue、
            # 旧来のtextのみモードは既定false(従来通り)にする。
            generate_raw = data.get("generate", context is not None)
            if isinstance(generate_raw, str):
                generate = generate_raw.strip().lower() not in ("", "false", "0")
            else:
                generate = bool(generate_raw)

            if not voice_master.enabled:
                # 全体OFF: 受理はするが何もしない(生成も再生もしない・キューにも積まない)。
                # 200で返すので、呼び出し側(voice_node等)は失敗扱いにしない。
                voice_master.count_muted()
                self._send_json(HTTPStatus.OK, {"ok": True, "muted": True, "accepted": False})
                return

            accepted = speech_queue.enqueue(text, generate, context=context)
            if not accepted:
                self._send_json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": "queue full"})
                return
            self._send_json(HTTPStatus.ACCEPTED, {"ok": True})

    return Handler


def main() -> None:
    bgm.start()
    announce_engine.start()
    announce_prepare.start()
    logger.info("接続元の許可リスト: %s", "全て(*)" if ALLOW[0] else [str(n) for n in ALLOW[1]] + ["ループバック"])
    logger.info("BGM: dir=%s enabled=%s volume=%.2f duck=%.2f files=%d", bgm.dir, bgm.gain.enabled,
                bgm.gain.volume, bgm.gain.duck_volume, len(bgm.files()))
    logger.info("音量: 発話x%.2f BGM %.2f ダッキング%.2f (保存ファイル%s)", voice_levels.speech_volume,
                voice_levels.bgm_volume, voice_levels.bgm_duck_volume,
                "あり" if voice_levels.from_file else "なし=環境変数/既定値")
    server = ThreadingHTTPServer((HOST, PORT), make_handler())
    logger.info("voice_bridge listening on http://%s:%s", HOST, PORT)
    logger.info("VOICEVOX_URL=%s OLLAMA_URL=%s OUTPUT_DEVICE=%s", VOICEVOX_URL, OLLAMA_URL, OUTPUT_DEVICE)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        speech_queue.stop()


if __name__ == "__main__":
    main()
