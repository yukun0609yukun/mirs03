"""文化祭(高専祭)の出店紹介アナウンス(班のロボットの特徴・クラスの店の紹介)の設定と再生の判断(純Python、ROS非依存)。

【2026-10-30追加】docs/announcements.md「文化祭の出店紹介」。既存の状況アナウンス(announce_events.py / announce_node.py)の仕組みの拡張:
紹介文を複数パートで登録し、**保存したときに1回だけ**2台目のVOICEVOXで合成してwavに保存(`/announce_prepare`。文章のハッシュで
キャッシュするので、同じ文章は再合成しない)、有効の間は**保存済みのwavを順に繰り返し再生するだけ**(再生中にVOICEVOX・Ollamaを呼ばない)。
ロボットの動き(cmd_vel・ゴール)には一切影響しない。

- 設定ファイル: `~/.nokosahen_pr_intro.json`(1台目Jetson。gitに入れない)。雛形は `config/pr_intro.example.json`(仮の文)。
    {"version": 1, "enabled": false, "interval_sec": 120, "gap_sec": 8, "volume": 1.0, "quiet_when_person_near": false, "speech_mode": "menu",
     "parts": [{"enabled": true, "title": "ロボットの特徴", "text": "(仮)…", "volume": 1.0}, ...]}
  * ファイルが無い間は「仮の文3パート・全体OFF」を画面に出す(保存して初めて有効)。仮の文は本番の文章と取り違えないよう、
    文頭の「(仮)」で判別し、画面に警告を出す。
  * interval_sec: 一巡(最後のパートを鳴らし終えて)から最初に戻るまでの間隔、gap_sec: パートとパートの間隔(いずれも再生が終わってから数える)。
  * volume: 紹介全体の倍率(0〜5)。パートごとのvolumeと掛け算(最大5で頭打ち)し、さらに2台目で announce_volume と掛かる。
- `PrIntroManager`: 状況フラグを受け、紹介を流してよいかを判断して「2台目へ送る操作(ops)」を返す。時刻は引数。
  **既存のアナウンスより必ず控えめ**(優先順位は docs/announcements.md): 移動中の注意・探索中・後退・進めず待機・人への接近(呼びかけ)・
  地図作成・手動運転・範囲外・緊急停止・アンケート回答中は流さない。止まってから SETTLE_SEC 秒たつまで再開しない。
- `PrIntroController`: ダッシュボードの `/api/pr_intro*`(保存・音声作成・試し再生)。2台目に届かなくても1台目の設定は編集・保存できる。
"""

from __future__ import annotations

import copy
import json
import math
import os
import shutil
import tempfile
import threading
import unicodedata
from typing import Any, Dict, List, Optional, Tuple

from .announce_events_proxy import AnnounceBridgeProxy

DEFAULT_FILE = "~/.nokosahen_pr_intro.json"
FILE_VERSION = 1
LOOP_ID = "pr_intro"  # 2台目の /announce_loop のid(状況アナウンスのidと重ならない)
TEST_ID = "pr_test"  # 試し再生(/announce)のid

# voice_bridge/announce_engine.py と同じ範囲(test_pr_intro.pyが一致を確認する)
TEXT_MAX = 400  # 1パートの文字数(合成の1文章としての上限。長いと合成に分単位かかる)
TOTAL_TEXT_MAX = 1600  # 全パートの合計(ディスク容量・合成時間の歯止め)
TITLE_MAX = 30
PARTS_MAX = 8
INTERVAL_MIN = 10.0
INTERVAL_MAX = 3600.0
GAP_MIN = 1.0
GAP_MAX = 600.0
VOLUME_MAX = 5.0
SEND_KEEPALIVE_SEC = 5.0  # 2台目へ「生きている」信号(同じ内容の再送)を送る間隔(2台目の停止判定は15秒)
SETTLE_SEC = 5.0  # 抑止の条件が解けてから、紹介を再開するまでの待ち(移動中の注意などの合間に割り込まない)
PREPARE_RESEND_SEC = 120.0  # 音声の事前作成の再要求の間隔(2台目の再起動・キャッシュ消失から自動で復旧する。作成済みなら何もしない)
PLACEHOLDER_MARKS = ("(仮)", "（仮）")

FIELD_ORDER = ("enabled", "interval_sec", "gap_sec", "volume", "quiet_when_person_near", "speech_mode")
# 【2026-10-31】平常時に流す発話の選択(どちらか片方だけ)。"menu"=メニュー紹介(director の営業の呼びかけ。出店紹介は止める) /
# "pr"=出店紹介(出店紹介がONの間、メニュー紹介の呼びかけは接近中も含めて止める)。
SPEECH_MODES = ("menu", "pr")
SPEECH_MODE_LABELS = {"menu": "メニュー紹介", "pr": "出店紹介"}
PART_FIELDS = ("enabled", "title", "text", "volume")

# 紹介を流さない状況フラグ(announce_events.FLAGS の名前)。どれかが立っている間は止める(既存のアナウンス・呼びかけ・走行が優先)。
BLOCK_FLAGS = ("moving_notice", "exploring", "backing_up", "blocked", "approaching_person", "mapping", "manual",
               "out_of_bounds", "estop", "exploration_complete")
BLOCK_LABELS = {
    "moving_notice": "移動中の注意が優先", "exploring": "自立探索中", "backing_up": "後退中", "blocked": "進めず待機中",
    "approaching_person": "呼びかけ(人に接近)中", "mapping": "地図作成中", "manual": "手動運転中",
    "out_of_bounds": "範囲外で停止中", "estop": "緊急停止中", "exploration_complete": "探索完了のお知らせ中",
}

DEFAULT_PARTS: List[Dict[str, Any]] = [
    {"enabled": True, "title": "ロボットの特徴(仮)", "volume": 1.0,
     "text": "(仮)私たちの班のロボットは、お客さんにアンケートを答えてもらいながら会場を自分で動き回ります。これは仮の文章です。本番の文章に差し替えてください。"},
    {"enabled": True, "title": "クラスの店の紹介(仮)", "volume": 1.0,
     "text": "(仮)私たちのクラスでは、おいしい食べ物のお店を出しています。これは仮の文章です。本番の文章に差し替えてください。"},
    {"enabled": True, "title": "お誘い(仮)", "volume": 1.0,
     "text": "(仮)ぜひお立ち寄りください。お待ちしています。これは仮の文章です。本番の文章に差し替えてください。"},
]
DEFAULTS: Dict[str, Any] = {
    "enabled": False,  # 仮の文を本番と取り違えないよう、既定はOFF
    "interval_sec": 120, "gap_sec": 8, "volume": 1.0, "quiet_when_person_near": False,
    "speech_mode": "menu",  # 従来の動き(メニュー紹介)のまま。出店紹介を流すときは "pr" にする
}


def default_config() -> Dict[str, Any]:
    cfg = copy.deepcopy(DEFAULTS)
    cfg["parts"] = copy.deepcopy(DEFAULT_PARTS)
    return cfg


def is_placeholder(text: str) -> bool:
    t = (text or "").lstrip()
    return t.startswith(PLACEHOLDER_MARKS)


def sanitize_text(value: str) -> str:
    """読み上げる文章の整形: 改行は文の区切り(。)に、タブ・連続空白は1つの空白に、制御文字・書式文字(ゼロ幅・向き制御など)・
    私用領域・未割り当て文字は取り除く。長さの検査はこのあとに行う(切り詰めない)。"""
    out = []
    for ch in value.replace("\r\n", "\n").replace("\r", "\n"):
        if ch == "\n":
            if out and out[-1] not in "。！？!?、, \n":
                out.append("。")
            continue
        if ch == "\t":
            ch = " "
        cat = unicodedata.category(ch)
        if cat in ("Cc", "Cf", "Cs", "Co", "Cn"):
            continue
        if ch.isspace():
            ch = " "
            if out and out[-1] == " ":
                continue
        out.append(ch)
    return "".join(out).strip()


def _reject_constant(name):
    raise ValueError(f"JSONに{name}(NaN/Infinity)は使えません")


def _num(value) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    v = float(value)
    return v if math.isfinite(v) else None


def _norm_num(v: float):
    return int(v) if v == int(v) else v


def _check_range(out: Dict[str, Any], errors: List[str], key: str, v: Any, lo: float, hi: float, name: str) -> None:
    n = _num(v)
    if n is None:
        errors.append(f"{name}は有限の数値で指定してください")
    elif n < lo or n > hi:
        errors.append(f"{name}は{lo:g}〜{hi:g}の範囲で指定してください")
    else:
        out[key] = _norm_num(n)


def validate_part(raw: Any, index: int) -> Tuple[Dict[str, Any], List[str]]:
    """1パートを検証・整形する。(正規化したdict, エラー一覧)。型違い・範囲外・未知の項目は拒否(丸めない)。"""
    label = f"パート{index + 1}"
    if not isinstance(raw, dict):
        return {}, [f"{label}: JSONオブジェクトにしてください"]
    errors: List[str] = []
    out: Dict[str, Any] = {"enabled": True, "title": "", "volume": 1.0}
    for k in raw:
        if k not in PART_FIELDS:
            errors.append(f"{label}: 未知の項目です: {k}")
    if "enabled" in raw:
        if isinstance(raw["enabled"], bool):
            out["enabled"] = raw["enabled"]
        else:
            errors.append(f"{label}: enabledは true/false で指定してください")
    if "title" in raw:
        if not isinstance(raw["title"], str):
            errors.append(f"{label}: 見出しは文字列で指定してください")
        else:
            title = sanitize_text(raw["title"])
            if len(title) > TITLE_MAX:
                errors.append(f"{label}: 見出しは{TITLE_MAX}文字以内にしてください")
            else:
                out["title"] = title
    if "volume" in raw:
        _check_range(out, errors, "volume", raw["volume"], 0.0, VOLUME_MAX, f"{label}: 音量")
    text = raw.get("text")
    if not isinstance(text, str):
        errors.append(f"{label}: 文章は文字列で指定してください")
    else:
        t = sanitize_text(text)
        if not t:
            errors.append(f"{label}: 文章が空です")
        elif len(t) > TEXT_MAX:
            errors.append(f"{label}: 文章は{TEXT_MAX}文字以内にしてください(いま{len(t)}文字)")
        else:
            out["text"] = t
    return out, errors


def validate_update(doc: Any) -> Tuple[Dict[str, Any], List[str]]:
    """保存リクエスト/ファイル内容(dict)を検証する。(正規化したdict, エラー一覧)。
    トップレベルの項目は部分でもよい。partsを指定したときは**パート一覧の全体を置き換える**。未知のキーはエラー(黙って無視しない)。"""
    if not isinstance(doc, dict):
        return {}, ["設定はJSONオブジェクトにしてください"]
    errors: List[str] = []
    clean: Dict[str, Any] = {}
    for k in doc:
        if k not in ("version", "description", "parts") + FIELD_ORDER:
            errors.append(f"未知のキー: {k}")
    for k in ("enabled", "quiet_when_person_near"):
        if k in doc:
            if isinstance(doc[k], bool):
                clean[k] = doc[k]
            else:
                errors.append(f"{k}は true/false で指定してください")
    if "speech_mode" in doc:
        if doc["speech_mode"] in SPEECH_MODES:
            clean["speech_mode"] = doc["speech_mode"]
        else:
            errors.append("speech_modeは " + " か ".join(SPEECH_MODES) + " で指定してください")
    if "interval_sec" in doc:
        _check_range(clean, errors, "interval_sec", doc["interval_sec"], INTERVAL_MIN, INTERVAL_MAX, "一巡後の間隔(秒)")
    if "gap_sec" in doc:
        _check_range(clean, errors, "gap_sec", doc["gap_sec"], GAP_MIN, GAP_MAX, "パート間の間隔(秒)")
    if "volume" in doc:
        _check_range(clean, errors, "volume", doc["volume"], 0.0, VOLUME_MAX, "紹介全体の音量")
    if "parts" in doc:
        parts = doc["parts"]
        if not isinstance(parts, list):
            errors.append("partsは配列にしてください")
        elif len(parts) > PARTS_MAX:
            errors.append(f"パートは{PARTS_MAX}個までです")
        else:
            clean_parts = []
            for i, p in enumerate(parts):
                cp, e = validate_part(p, i)
                errors.extend(e)
                clean_parts.append(cp)
            total = sum(len(p.get("text", "")) for p in clean_parts)
            if total > TOTAL_TEXT_MAX:
                errors.append(f"全パートの文章は合計{TOTAL_TEXT_MAX}文字以内にしてください(いま{total}文字)")
            clean["parts"] = clean_parts
    return (clean, errors) if not errors else ({}, errors)


def merged_config(clean: Optional[Dict[str, Any]], file_exists: bool) -> Dict[str, Any]:
    """検証済みの文書を既定値(仮の文3パート・全体OFF)に重ねた完全な設定を返す。ファイルが無い間は必ずOFF。"""
    cfg = default_config()
    if clean and file_exists:
        for k in FIELD_ORDER:
            if k in clean:
                cfg[k] = clean[k]
        if "parts" in clean:
            cfg["parts"] = clean["parts"]
    else:
        cfg["enabled"] = False
    return cfg


def check_consistency(cfg: Dict[str, Any]) -> List[str]:
    """重ねた結果の整合性(ONなのに流せるパートが無い、など)。"""
    errors: List[str] = []
    if cfg["enabled"] and not any(p["enabled"] for p in cfg["parts"]):
        errors.append("全体をONにするには、有効なパートが1つ以上必要です")
    return errors


def menu_speech_suppressed(cfg: Dict[str, Any]) -> bool:
    """出店紹介が選ばれていて、実際に流れる状態(ON+有効なパートあり)なら、メニュー紹介の呼びかけを止める。"""
    return bool(cfg.get("enabled")) and cfg.get("speech_mode") == "pr" and bool(enabled_parts(cfg))


def enabled_parts(cfg: Dict[str, Any]) -> List[Dict[str, Any]]:
    return [p for p in cfg["parts"] if p.get("enabled")]


def playlist_op(cfg: Dict[str, Any]) -> Dict[str, Any]:
    """2台目の /announce_loop(プレイリスト)へ送る内容。パート音量は紹介全体の音量との積(最大5で頭打ち)。"""
    g = float(cfg["volume"])
    return {"op": "playlist", "id": LOOP_ID, "interval_sec": float(cfg["interval_sec"]), "gap_sec": float(cfg["gap_sec"]),
            "parts": [{"text": p["text"], "volume": round(min(VOLUME_MAX, g * float(p["volume"])), 4)}
                      for p in enabled_parts(cfg)]}


def prepare_texts(cfg: Dict[str, Any]) -> List[str]:
    """事前に合成しておく文章(重複は除く。順序は保つ)。無効のパートも、ONにしたとき即鳴るよう用意する。"""
    seen, out = set(), []
    for p in cfg["parts"]:
        if p["text"] not in seen:
            seen.add(p["text"])
            out.append(p["text"])
    return out


def file_sig(path: str) -> str:
    try:
        st = os.stat(path)
    except OSError:
        return "missing"
    return f"{st.st_mtime_ns}-{st.st_size}"


def meta() -> Dict[str, Any]:
    """画面用の固定情報(範囲・抑止の一覧)。"""
    return {
        "limits": {"text_max": TEXT_MAX, "total_text_max": TOTAL_TEXT_MAX, "title_max": TITLE_MAX, "parts_max": PARTS_MAX,
                   "interval": [INTERVAL_MIN, INTERVAL_MAX], "gap": [GAP_MIN, GAP_MAX], "volume": [0.0, VOLUME_MAX]},
        "defaults": default_config(),
        "block_labels": [BLOCK_LABELS[k] for k in BLOCK_FLAGS],
        "speech_modes": [{"value": k, "label": SPEECH_MODE_LABELS[k]} for k in SPEECH_MODES],
        "placeholder_marks": list(PLACEHOLDER_MARKS),
    }


class PrIntroStore:
    """ダッシュボード側: 読み取りと検証つき保存(スレッドセーフ。原子的置換+`.bak`)。"""

    def __init__(self, path: str = DEFAULT_FILE) -> None:
        self.path = os.path.expanduser(path or DEFAULT_FILE)
        self._lock = threading.Lock()

    @property
    def backup_path(self) -> str:
        return self.path + ".bak"

    def read(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {"path": self.path, "exists": False, "sig": file_sig(self.path), "valid": True, "errors": [],
                               "config": merged_config(None, False)}
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                text = f.read(262144)
        except FileNotFoundError:
            return out
        except (OSError, UnicodeDecodeError) as exc:
            out["exists"] = os.path.exists(self.path)
            out["valid"], out["errors"] = False, [f"ファイルを読めません: {exc}"]
            return out
        out["exists"] = True
        try:
            doc = json.loads(text, parse_constant=_reject_constant)
        except ValueError as exc:
            out["valid"], out["errors"] = False, [f"JSONとして読めません: {exc}"]
            return out
        clean, errors = validate_update(doc)
        if not errors:
            errors = check_consistency(merged_config(clean, True))
        if errors:
            out["valid"], out["errors"] = False, errors
            return out
        out["config"] = merged_config(clean, True)
        return out

    def save(self, update: Any) -> Dict[str, Any]:
        """部分更新({"enabled"?, "interval_sec"?, ..., "parts"?: [...全体]})を検証して保存する。例外は投げない。"""
        clean, errors = validate_update(update)
        if errors:
            return {"ok": False, "kind": "invalid", "errors": errors}
        clean = {k: v for k, v in clean.items() if k in FIELD_ORDER or k == "parts"}
        if not clean:
            return {"ok": False, "kind": "invalid", "errors": ["変更する項目がありません"]}
        with self._lock:
            cur = self.read()
            base = cur["config"] if cur["valid"] else merged_config(None, False)  # 壊れたファイルは上書きで直せる(.bakに退避)
            cfg = copy.deepcopy(base)
            cfg.update(clean)
            errors = check_consistency(cfg)
            if errors:
                return {"ok": False, "kind": "invalid", "errors": errors}
            doc = {"version": FILE_VERSION, **{k: cfg[k] for k in FIELD_ORDER}, "parts": cfg["parts"]}
            text = json.dumps(doc, ensure_ascii=False, indent=2) + "\n"
            directory = os.path.dirname(self.path) or "."
            tmp_path = bak_tmp = None
            backed_up = False
            try:
                os.makedirs(directory, exist_ok=True)
                fd, tmp_path = tempfile.mkstemp(prefix=".nokosahen_pr_intro_", suffix=".tmp", dir=directory)
                with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as f:
                    f.write(text)
                    f.flush()
                    os.fsync(f.fileno())
                if os.path.exists(self.path):
                    bak_tmp = self.backup_path + ".tmp"
                    shutil.copy2(self.path, bak_tmp)
                    os.replace(bak_tmp, self.backup_path)
                    bak_tmp = None
                    backed_up = True
                os.replace(tmp_path, self.path)
                tmp_path = None
            except OSError as exc:
                return {"ok": False, "kind": "io",
                        "errors": [f"ファイルを書き込めませんでした(元のファイルは変更されていません): {exc}"]}
            finally:
                for p in (tmp_path, bak_tmp):
                    if p:
                        try:
                            os.remove(p)
                        except OSError:
                            pass
        return {"ok": True, "kind": None, "errors": [], "backed_up": backed_up, "sig": file_sig(self.path), "config": cfg}


class PrIntroReader:
    """announce_node側: 内容が変わったときだけ読み直す(1Hzで呼ぶ)。例外は投げない。
    不正なファイルは直前の有効な設定を維持する。ファイルが無ければ全体OFF。"""

    def __init__(self, path: str = DEFAULT_FILE) -> None:
        self.path = os.path.expanduser(path or DEFAULT_FILE)
        self._sig: Optional[str] = None
        self.config: Dict[str, Any] = merged_config(None, False)
        self.load_error = ""

    def poll(self) -> Tuple[Dict[str, Any], str, bool]:
        """(現在の設定, ログに出すメッセージ, 変化したか)。"""
        try:
            with open(self.path, "rb") as f:
                raw = f.read(262144)
            sig = "c:" + str(hash(raw))
        except FileNotFoundError:
            sig = "missing"
        except OSError as exc:
            sig = "err:" + str(exc)
        if sig == self._sig:
            return self.config, "", False
        self._sig = sig
        if sig == "missing":
            changed = bool(self.config.get("enabled")) or bool(self.load_error)
            self.config, self.load_error = merged_config(None, False), ""
            return self.config, ("文化祭の出店紹介の設定ファイルが無いため、OFFです" if changed else ""), changed
        try:
            doc = json.loads(raw.decode("utf-8"), parse_constant=_reject_constant)
        except (UnicodeDecodeError, ValueError) as exc:
            self.load_error = f"文化祭の出店紹介の設定ファイルを読めません(直前の設定を維持): {exc}"
            return self.config, self.load_error, False
        clean, errors = validate_update(doc)
        if not errors:
            errors = check_consistency(merged_config(clean, True))
        if errors:
            self.load_error = "文化祭の出店紹介の設定ファイルが不正です(直前の設定を維持): " + " / ".join(errors)
            return self.config, self.load_error, False
        self.load_error = ""
        self.config = merged_config(clean, True)
        return self.config, "文化祭の出店紹介の設定を読み込みました", True


class PrIntroManager:
    """状況フラグ→2台目へ送る操作。ROS・HTTPは持たない。`update(flags, context, now)` が返す ops:
      {"op": "playlist", "id", "interval_sec", "gap_sec", "parts": [{"text","volume"}]}  # 開始・設定変更・「生きている」信号の再送
      {"op": "playlist_stop", "id"}
      {"op": "prepare", "texts": [...]}  # 音声の事前作成の要求(設定変更時と一定時間ごと。作成済みなら2台目は何もしない)
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None, keepalive_send_sec: float = SEND_KEEPALIVE_SEC,
                 prepare_resend_sec: float = PREPARE_RESEND_SEC, settle_sec: float = SETTLE_SEC) -> None:
        self.config = config or merged_config(None, False)
        self.keepalive_send_sec = keepalive_send_sec
        self.prepare_resend_sec = prepare_resend_sec
        self.settle_sec = settle_sec
        self.on = False
        self.reason = ""
        self._sent_sig: Optional[str] = None
        self._last_sent = 0.0
        self._blocked_until = 0.0
        self._last_prepare = -1e9
        self._prepare_sig: Optional[str] = None

    def set_config(self, config: Dict[str, Any]) -> None:
        self.config = config

    def _blockers(self, flags: Dict[str, bool], context: Dict[str, bool]) -> List[str]:
        # 【2026-11-06】"blocked" は「近すぎる人/カメラの障害物で自律走行がホールド中(NEAR_PERSON_HOLD・CAMERA_OBSTACLE_HOLD)」。
        # 人がすぐ近くにいる=説明を聞いてほしい場面なので、quiet_when_person_near が OFF(既定)のときは、紹介を止めない。
        # 実機(会場)で、近くの人・静止物で NEAR_PERSON_HOLD が頻繁に出入りし、そのたびに紹介が止まって再開待ち(5秒)になり、ほとんど流れなかった。
        quiet_near = bool(self.config.get("quiet_when_person_near"))
        out = [BLOCK_LABELS[k] for k in BLOCK_FLAGS if flags.get(k) and (k != "blocked" or quiet_near)]
        if self.config.get("speech_mode") != "pr":
            out.append("メニュー紹介を選択中")
        if context.get("estop") and BLOCK_LABELS["estop"] not in out:
            out.append(BLOCK_LABELS["estop"])
        if context.get("survey"):
            out.append("アンケート回答中")
        if context.get("person_near") and self.config.get("quiet_when_person_near"):
            out.append("人が近い")
        return out

    def update(self, flags: Dict[str, bool], context: Dict[str, bool], now: float) -> List[Dict[str, Any]]:
        ops: List[Dict[str, Any]] = []
        cfg = self.config
        parts = enabled_parts(cfg)
        # 音声の事前作成: ONの間、設定が変わったとき/一定時間ごとに要求(2台目の再起動後も、保存済みwavが消えていても自動で作り直す)
        if cfg.get("enabled") and parts:
            psig = json.dumps(prepare_texts(cfg), ensure_ascii=False)
            if psig != self._prepare_sig or now - self._last_prepare >= self.prepare_resend_sec:
                ops.append({"op": "prepare", "texts": prepare_texts(cfg)})
                self._prepare_sig, self._last_prepare = psig, now
        blockers = self._blockers(flags, context)
        if blockers:
            self._blocked_until = now + self.settle_sec
        reason = ""
        if not cfg.get("enabled"):
            reason = "OFF"
        elif not parts:
            reason = "有効なパートがありません"
        elif blockers:
            reason = " / ".join(blockers) + "のため停止"
        elif now < self._blocked_until:
            reason = "再開待ち"
        self.reason = reason
        if not reason:
            op = playlist_op(cfg)
            sig = json.dumps(op, ensure_ascii=False, sort_keys=True)
            if (not self.on) or sig != self._sent_sig or now - self._last_sent >= self.keepalive_send_sec:
                ops.append(op)
                self.on, self._sent_sig, self._last_sent = True, sig, now
        elif self.on:
            ops.append({"op": "playlist_stop", "id": LOOP_ID})
            self.on, self._sent_sig = False, None
        return ops

    def stop_all(self) -> List[Dict[str, Any]]:
        if self.on:
            self.on, self._sent_sig = False, None
            return [{"op": "playlist_stop", "id": LOOP_ID}]
        return []

    def status(self) -> Dict[str, Any]:
        return {"enabled": bool(self.config.get("enabled")), "playing": self.on, "reason": self.reason,
                "parts": len(enabled_parts(self.config))}


class PrIntroController:
    """ダッシュボード側: `/api/pr_intro`(読み出し・保存)・`/api/pr_intro/prepare`(音声を作る)・`/api/pr_intro/test`(試し再生)。"""

    def __init__(self, store: PrIntroStore, bridge: AnnounceBridgeProxy) -> None:
        self.store = store
        self.bridge = bridge

    def _statuses(self, texts: List[str], start: bool = False, force: bool = False) -> Dict[str, Any]:
        if not texts:
            return {"ok": True, "items": []}
        res = self.bridge.prepare(texts, start=start, force=force)
        if not res.get("ok"):
            return {"ok": False, "error": res.get("error", "")}
        d = res["data"]
        return {"ok": True, "items": d.get("items", []), "prepare": d.get("prepare", {})}

    def payload(self) -> Dict[str, Any]:
        """ページの初期表示・読み戻し用。2台目に繋がらなくても1台目の設定は返す(bridge.ok=false)。"""
        rd = self.store.read()
        cfg = rd["config"]
        parts = []
        for p in cfg["parts"]:
            q = dict(p)
            q["placeholder"] = is_placeholder(p["text"])
            parts.append(q)
        out: Dict[str, Any] = {
            "ok": True, "configured": self.bridge.configured, "path": rd["path"], "exists": rd["exists"], "sig": rd["sig"],
            "valid": rd["valid"], "errors": rd["errors"], "config": {**cfg, "parts": parts}, "meta": meta(),
            "total_chars": sum(len(p["text"]) for p in cfg["parts"]),
            "placeholder": any(is_placeholder(p["text"]) for p in enabled_parts(cfg)),
            "bridge": None,
        }
        st = self._statuses([p["text"] for p in cfg["parts"]])
        if st.get("ok"):
            loop = None
            state = self.bridge.get_state()
            if state.get("ok"):
                loop = (state["data"].get("loops") or {}).get(LOOP_ID)
            out["bridge"] = {"ok": True, "items": st["items"], "prepare": st.get("prepare", {}), "loop": loop}
        else:
            out["bridge"] = {"ok": False, "error": st.get("error", "")}
        return out

    def save(self, data: Any) -> Tuple[int, Dict[str, Any]]:
        """検証して保存し、2台目に音声の作成を依頼する(文章が変わったものだけ合成される)。(HTTPステータス, 応答dict)"""
        if not isinstance(data, dict):
            return 400, {"ok": False, "errors": ["不正なリクエストです"], "message": "不正なリクエストです"}
        r = self.store.save(data)
        if not r.get("ok"):
            status = 400 if r.get("kind") == "invalid" else 500
            errs = r.get("errors") or ["保存できませんでした"]
            return status, {"ok": False, "errors": errs, "message": errs[0]}
        out: Dict[str, Any] = {"ok": True, "errors": [], "sig": r.get("sig"), "message": "保存しました"}
        if "parts" in data or "enabled" in data:
            res = self._statuses(prepare_texts(r["config"]), start=True)
            if res.get("ok"):
                busy = [i for i in res["items"] if i.get("state") in ("queued", "synthesizing")]
                out["prepare"] = {"started": True, "pending": len(busy)}
                out["message"] = ("保存しました。音声を作成中です(長い文章は数十秒〜数分かかります。完了するまでは鳴りません)"
                                  if busy else "保存しました。音声はすべて作成済みです")
            else:
                out["prepare"] = {"started": False, "error": res.get("error", "")}
                out["message"] = "保存しました。ただし2台目に届かず音声は未作成です(つながったら自動で作成します): " + res.get("error", "")
        return 200, out

    def prepare(self, data: Any) -> Tuple[int, Dict[str, Any]]:
        """保存済みの全パートの音声作成を依頼する({"force": true}で、前回失敗した分もすぐ再試行)。"""
        if not isinstance(data, dict) or any(k != "force" for k in data) or not isinstance(data.get("force", False), bool):
            return 400, {"ok": False, "errors": ["不正なリクエストです"], "message": "不正なリクエストです"}
        cfg = self.store.read()["config"]
        res = self._statuses(prepare_texts(cfg), start=True, force=bool(data.get("force")))
        if not res.get("ok"):
            msg = res.get("error", "")
            return 200, {"ok": False, "message": msg, "errors": [msg]}
        busy = [i for i in res["items"] if i.get("state") in ("queued", "synthesizing")]
        return 200, {"ok": True, "pending": len(busy),
                     "message": "音声の作成を依頼しました" if busy else "音声はすべて作成済みです(再合成は不要)"}

    def test(self, data: Any) -> Tuple[int, Dict[str, Any]]:
        """{"index": パート番号(0始まり)}。**保存済み**の文章を、作成済みの音声で1回鳴らす(合成はしない)。"""
        if not isinstance(data, dict) or any(k != "index" for k in data):
            return 400, {"ok": False, "errors": ["不正なリクエストです"], "message": "不正なリクエストです"}
        idx = data.get("index")
        cfg = self.store.read()["config"]
        if isinstance(idx, bool) or not isinstance(idx, int) or not (0 <= idx < len(cfg["parts"])):
            return 400, {"ok": False, "errors": ["パート番号が不正です"], "message": "パート番号が不正です"}
        part = cfg["parts"][idx]
        res = self._statuses([part["text"]])
        if not res.get("ok"):
            msg = res.get("error", "")
            return 200, {"ok": False, "message": msg, "errors": [msg]}
        if res["items"][0].get("state") != "ready":
            msg = "このパートの音声はまだ作成されていません(「音声を作る」を押して、完了してからお試しください)"
            return 200, {"ok": False, "message": msg, "errors": [msg]}
        vol = min(VOLUME_MAX, float(cfg["volume"]) * float(part["volume"]))
        play = self.bridge.play_cached(TEST_ID, part["text"], vol)
        if not play.get("ok"):
            msg = play.get("error", "")
            return 200, {"ok": False, "message": msg, "errors": [msg]}
        if play["data"].get("muted"):
            return 200, {"ok": True, "played": False, "muted": True,
                         "message": "音声が全体OFFのため鳴りません(音声の全体スイッチをONにしてください)"}
        return 200, {"ok": True, "played": True, "message": "鳴らします(保存済みの音声を再生するだけです)"}
