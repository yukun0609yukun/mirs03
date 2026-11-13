"""iPad表示のアンケート: 設問の定義・回答の検証・保存(純Python、ROS非依存)。docs/ipad_display.md。

ラズパイ版(scripts/raspi_display/survey/mirs_robot_survey.html の `window.ROBOT_SURVEY` と receiver.py)を
1台目(ダッシュボードと同じプロセス)へ移したもの。

引き継ぐもの(今のものと同じ)
- 設問・選択肢・文言: SURVEY_DEF が今の版(r2 = 高専祭用の kousensai_survey.html の `window.ROBOT_SURVEY` をそのまま移植)。
  設問idは途中で変えない・内容を変えたら version を上げる。**過去の版の定義は SURVEY_DEFS に残す**
  (r1 = ラズパイ版 mirs_robot_survey.html の移植。古い画面で端末内に溜まっていた r1 の回答を、版が変わっても取りこぼさず保存できる)。
  回答には survey_version を保存し、CSV でも版ごとに区別できる(設問id列は全版の和集合)。
- 保存先: `~/survey_data/`(環境変数 SURVEY_DATA_DIR か引数で変更)。`responses.jsonl`(1回答1行、追記専用、これが正本)と
  `responses.csv`(Excelで開ける集計用)。`condition.txt`(実験条件)。
- 記録形式: receiver.py と同じ(固定列 received_at, response_id, submitted_at, survey_version, condition, location,
  duration_sec + 設問id列。複数選択は " / " でつなぐ。数式インジェクション対策の csv_safe も同じ)。
- 「保存完了」の定義: JSONL に書けて fsync できた時(回答中のロボット停止は、この時に解除の準備に入る)。

変えたもの(安全・運用のため)
- 認証なしの入力口になるので、**入力を厳しく検証する**(設問idと選択肢に合うものだけ。長さ・型の上限)。
  ラズパイ版は何でも保存していた。
- 保存ファイルの上限(既定 10MB / 5万件)。超えたら保存を断る(ディスクを埋められないため)。
- CSV は保存のたびには作り直さない(1台目のCPUを使わないため)。ダウンロードの時に作る。
"""
from __future__ import annotations

import csv
import io
import json
import os
import tempfile
import re
import threading
import uuid
from datetime import datetime
from typing import Any, Dict, List, Optional, Set, Tuple

DEFAULT_DATA_DIR = "~/survey_data"
MAX_BODY_BYTES = 8 * 1024          # 回答1件のJSONの上限
MAX_FILE_BYTES = 10 * 1024 * 1024  # responses.jsonl の上限
MAX_RECORDS = 50000
MAX_CSV_BYTES = 12 * 1024 * 1024
MAX_TEXT = 80                       # 条件・場所メモの長さ

FIXED = ["received_at", "response_id", "submitted_at", "survey_version", "condition",
         "location", "duration_sec"]

# 旧版 r1(ラズパイ版 mirs_robot_survey.html の移植。2026-10-30 に r2 へ置き換えるまで使っていた)。
# 古い画面から届く r1 の回答を検証して保存するためだけに残す(今の画面は r2 を送る)。必須ではない(とばせた)。
SURVEY_DEF_R1: Dict[str, Any] = {
    "version": "r1",
    "sheet": "robot",
    "conditions": ["停止 + 音声", "移動 + 音声", "移動のみ", "展示 (高専祭)"],
    "questions": [
        {"id": "noticed_by", "type": "choice", "label": "ロボットに気づいたきっかけは？",
         "options": ["動いていたから", "声が聞こえたから", "画面が見えたから", "人が集まっていたから", "たまたま"]},
        {"id": "impression", "type": "scale", "label": "ロボットの印象は？", "min": 1, "max": 5, "faces": True,
         "minLabel": "よくない", "maxLabel": "とてもよい"},
        {"id": "approach_feel", "type": "multi", "label": "近づいてきたとき、どう感じましたか？",
         "options": ["楽しい", "気にならない", "少しこわい", "じゃまに感じた", "近づいてこなかった"], "maxSelect": 2},
        {"id": "visitor_type", "type": "choice", "label": "あなたは？",
         "options": ["小学生", "中学生", "高校・高専生", "大人"]},
    ],
}

# kousensai_survey.html (高専祭用, version "r2") を移植。設問・選択肢・文言・順序はそのまま。
# 5問とも必須(次へは答えるまで押せない)。単一選択はとばせない。
SURVEY_DEF: Dict[str, Any] = {
    "version": "r2",
    "sheet": "robot",
    "idleTitle": "ロボットについて\n教えてください",
    "idleNote": "5つの質問 ・ 約35秒",
    "startLabel": "タッチして はじめる",
    "thanks": "ありがとうございました！",
    "idleTimeoutSec": 30,       # 回答途中で放置されたら待機画面へ戻るまでの秒数
    "doneReturnSec": 10,        # 完了画面から待機画面へ戻るまでの秒数(r2: 10秒。r1 は5秒だった)
    "showBack": True,           # 「1つ前にもどる」「さいしょにもどる」を2問目以降に出す(2026-11-08。答えは残る)
    # 隠し管理画面(右上の角を3秒以内に5回タップ)の誤操作防止用PIN。認証ではない(実験条件と設置場所メモを変えるだけ)
    "adminPin": "2603",
    "conditions": ["停止 + 音声", "移動 + 音声", "移動のみ", "展示 (高専祭)"],
    "questions": [
        {"id": "impression", "type": "scale", "label": "ロボットの印象は？", "min": 1, "max": 5, "faces": True,
         "minLabel": "よくない", "maxLabel": "とてもよい", "required": True},
        {"id": "noticed_by", "type": "choice", "label": "ロボットに気づいたきっかけは？",
         "options": ["動いていたから", "声が聞こえたから", "画面が見えたから", "人が集まっていたから"], "required": True},
        {"id": "approach_feel", "type": "multi", "label": "近づいてきたとき、どう感じましたか？",
         # 【2026-11-08】選択肢「声が聞こえた」を追加(ユーザーの依頼。文化祭2日目)。r2 の版名は変えない(それまでの回答の選択肢は、今の選択肢の部分集合なので、そのまま有効)。
         "options": ["楽しい", "気にならない", "少しこわい", "じゃまに感じた", "声が聞こえた", "近づいてこなかった"], "maxSelect": 2,
         "required": True},
        {"id": "want_to_move", "type": "choice", "label": "このロボットの話している説明を聞いて、メニューを変えたいと思いましたか？",   # 【2026-11-08】旧: 「実際に移動したいと思いましたか？」(ユーザーの依頼)。id(want_to_move)・選択肢は変えない
         "options": ["とても思った", "やや思った", "あまり思わなかった", "思わなかった"], "required": True},
        {"id": "visitor_type", "type": "choice", "label": "あなたは？",
         "options": ["小学生", "中学生", "高校・高専生", "大人"], "required": True},
    ],
}

# 版 → 定義。validate_response は、届いた survey_version に合う定義で検証する(未知の版は拒否)。
SURVEY_DEFS: Dict[str, Dict[str, Any]] = {"r1": SURVEY_DEF_R1, "r2": SURVEY_DEF}

_ID_RE = re.compile(r"^[A-Za-z0-9._:-]{8,64}$")
_TIME_RE = re.compile(r"^[0-9T:.+\-Z ]{10,40}$")


def _is_int(v: Any) -> bool:
    return isinstance(v, int) and not isinstance(v, bool)


def _clean_text(v: Any, limit: int = MAX_TEXT) -> Optional[str]:
    """制御文字を含まない短い文字列だけ通す(それ以外は None)。"""
    if not isinstance(v, str) or len(v) > limit:
        return None
    if any(ord(c) < 0x20 or ord(c) == 0x7F for c in v):
        return None
    return v.strip()


def validate_condition(v: Any, definition: Optional[Dict[str, Any]] = None) -> Tuple[Optional[str], str]:
    """実験条件。空文字か、設問定義の conditions のどれか。(値, エラー)"""
    d = definition or SURVEY_DEF
    t = _clean_text(v)
    if t is None:
        return None, "conditionが不正です"
    if t and t not in d["conditions"]:
        return None, "conditionが一覧にありません"
    return t, ""


def validate_response(p: Any, definition: Optional[Dict[str, Any]] = None) -> Tuple[Optional[Dict[str, Any]], str]:
    """POSTされた回答を検証して、保存する形(余計な項目を捨てた dict)にする。(record, エラー文)。
    設問id・選択肢・段階の範囲に合うものだけ通す。未知の設問idは拒否。required の設問は答えが無いと拒否。
    definition を渡さないときは、p の survey_version に合う定義(SURVEY_DEFS)で検証する。"""
    if not isinstance(p, dict):
        return None, "回答はJSONオブジェクトにしてください"
    if definition is not None:
        d = definition
    else:
        v = p.get("survey_version")
        d = SURVEY_DEFS.get(v, SURVEY_DEF) if isinstance(v, str) else SURVEY_DEF
    rid = p.get("response_id")
    if not isinstance(rid, str) or not _ID_RE.match(rid):
        return None, "response_idが不正です"
    sub = p.get("submitted_at")
    if not isinstance(sub, str) or not _TIME_RE.match(sub):
        return None, "submitted_atが不正です"
    if p.get("survey_version") != d["version"]:
        return None, "survey_versionが違います(画面を読み込み直してください)"
    cond, err = validate_condition(p.get("condition", ""), d)
    if cond is None:
        return None, err
    loc = _clean_text(p.get("location", ""))
    if loc is None:
        return None, "locationが不正です"
    dur = p.get("duration_sec", 0)
    if not _is_int(dur) or dur < 0 or dur > 86400:
        return None, "duration_secが不正です"
    ans = p.get("answers")
    if not isinstance(ans, dict):
        return None, "answersが不正です"
    by_id = {q["id"]: q for q in d["questions"]}
    clean: Dict[str, Any] = {}
    for k, v in ans.items():
        q = by_id.get(k)
        if q is None:
            return None, f"未知の設問です: {str(k)[:30]}"
        t = q["type"]
        if t == "choice":
            if not isinstance(v, str) or v not in q["options"]:
                return None, f"{k}の選択肢が不正です"
            clean[k] = v
        elif t == "scale":
            if not _is_int(v) or v < q.get("min", 1) or v > q.get("max", 5):
                return None, f"{k}の値が範囲外です"
            clean[k] = v
        elif t == "multi":
            if not isinstance(v, list) or len(v) > int(q.get("maxSelect") or len(q["options"])):
                return None, f"{k}の選択が不正です"
            if any((not isinstance(x, str)) or x not in q["options"] for x in v) or len(set(v)) != len(v):
                return None, f"{k}の選択肢が不正です"
            clean[k] = [x for x in q["options"] if x in v]
        else:  # 設問定義に未知の型が入っていたら保存しない
            return None, f"{k}の設問型が未対応です"
    for q in d["questions"]:
        if q.get("required") and q["id"] not in clean:
            return None, f"{q['id']}の回答がありません"
        if q.get("required") and q["type"] == "multi" and not clean.get(q["id"]):
            return None, f"{q['id']}の回答がありません"
    rec = {
        "response_id": rid, "submitted_at": sub, "survey_version": d["version"], "mode": "robot",
        "condition": cond, "location": loc, "duration_sec": dur, "answers": clean, "sheet": d["sheet"],
    }
    return rec, ""


def csv_safe(v: Any) -> Any:
    """CSVの数式インジェクション対策(receiver.py と同じ)。= + - @ やタブ・改行で始まる文字列の先頭に ' を付ける。"""
    if isinstance(v, str) and v[:1] in ("=", "+", "-", "@", "\t", "\r", "\n"):
        return "'" + v
    return v


def records_to_csv(records: List[Dict[str, Any]]) -> bytes:
    """全件から CSV(UTF-8 BOM付き)を作る。設問が増えても列がずれない(receiver.py の write_csv と同じ形)。"""
    ids: List[str] = []
    for r in records:
        a = r.get("answers", {})
        if isinstance(a, dict):
            for k in a:
                if k not in ids:
                    ids.append(k)
    buf = io.StringIO(newline="")
    w = csv.writer(buf)
    w.writerow(FIXED + ids)
    for r in records:
        a = r.get("answers", {})
        if not isinstance(a, dict):
            a = {}
        row = [csv_safe(r.get(k, "")) for k in FIXED]
        for i in ids:
            v = a.get(i, "")
            row.append(csv_safe(" / ".join(map(str, v)) if isinstance(v, list) else v))
        w.writerow(row)
    return ("﻿" + buf.getvalue()).encode("utf-8")



def _atomic_write(path: str, data: bytes) -> None:
    """一意の一時ファイルに書いて fsync し、os.replace で置き換える(失敗時は一時ファイルを消して OSError を投げる)。"""
    d = os.path.dirname(path) or "."
    fd, tmp = tempfile.mkstemp(prefix=os.path.basename(path) + ".", suffix=".tmp", dir=d)
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise

class SurveyStore:
    """回答の保存。JSONL に追記して fsync できたら「保存完了」。"""

    def __init__(self, data_dir: Optional[str] = None, max_bytes: int = MAX_FILE_BYTES,
                 max_records: int = MAX_RECORDS) -> None:
        d = data_dir or os.environ.get("SURVEY_DATA_DIR") or DEFAULT_DATA_DIR
        self.data_dir = os.path.expanduser(d)
        self.jsonl = os.path.join(self.data_dir, "responses.jsonl")
        self.csv_path = os.path.join(self.data_dir, "responses.csv")
        self.cond_path = os.path.join(self.data_dir, "condition.txt")
        self.max_bytes = int(max_bytes)
        self.max_records = int(max_records)
        self._lock = threading.Lock()
        self._ids: Set[str] = set()
        self._count = 0
        self.bad_lines = 0
        self._load_index()

    # --- 読み込み ---
    def _read_all(self) -> List[Dict[str, Any]]:
        """responses.jsonl を全件読む。壊れた行は飛ばす(電源断で途中までの行など。元のファイルは変えない)。"""
        out: List[Dict[str, Any]] = []
        bad = 0
        try:
            with open(self.jsonl, encoding="utf-8", errors="replace") as f:
                for line in f:
                    if not line.strip():
                        continue
                    try:
                        rec = json.loads(line)
                    except ValueError:
                        bad += 1
                        continue
                    if isinstance(rec, dict):
                        out.append(rec)
                    else:
                        bad += 1
        except OSError:
            pass
        self.bad_lines = bad
        return out

    def _load_index(self) -> None:
        recs = self._read_all()
        self._count = len(recs)
        self._ids = {str(r.get("response_id")) for r in recs if r.get("response_id")}

    def count(self) -> int:
        return self._count

    # --- 保存 ---
    def save(self, rec: Dict[str, Any]) -> Tuple[bool, bool, Optional[str]]:
        """(ok, duplicate, エラー文字列)。ok=True は JSONL に書いて fsync できた時だけ。
        同じ response_id は書かずに ok+duplicate(HTMLの再送で二重保存しない)。例外は投げない。"""
        try:
            with self._lock:
                rid = rec.get("response_id") or ("srv-" + uuid.uuid4().hex)
                rec["response_id"] = rid
                if rid in self._ids:
                    return True, True, None
                if self._count >= self.max_records:
                    return False, False, "回答の保存件数が上限に達しました"
                try:
                    size = os.path.getsize(self.jsonl)
                except OSError:
                    size = 0
                if size >= self.max_bytes:
                    return False, False, "回答ファイルが上限サイズに達しました"
                rec = dict(rec)
                rec["received_at"] = datetime.now().isoformat(timespec="seconds")
                os.makedirs(self.data_dir, exist_ok=True)
                with open(self.jsonl, "a", encoding="utf-8") as f:
                    # 前回の書き込みが途中で切れて末尾に改行が無いと、新しい回答が壊れた行の後ろにつながって一緒に読めなくなる
                    if f.tell() > 0:
                        with open(self.jsonl, "rb") as rf:
                            rf.seek(-1, os.SEEK_END)
                            if rf.read(1) != b"\n":
                                f.write("\n")
                    f.write(json.dumps(rec, ensure_ascii=False) + "\n")
                    f.flush()
                    os.fsync(f.fileno())
                self._ids.add(rid)
                self._count += 1
            return True, False, None
        except Exception as e:  # noqa: BLE001  (ディスク満杯・権限など)
            return False, False, f"{type(e).__name__}: {e}"[:200]

    # --- CSV ---
    def csv_bytes(self) -> Tuple[Optional[bytes], str]:
        """(CSV, エラー)。上限を超えるときは作らない。ファイルの responses.csv も更新する(失敗しても無視)。"""
        with self._lock:
            recs = self._read_all()
        data = records_to_csv(recs)
        if len(data) > MAX_CSV_BYTES:
            return None, "回答CSVが大きすぎます"
        try:
            os.makedirs(self.data_dir, exist_ok=True)
            # 【2026-11-05】一時ファイル名は一意にする(固定の {pid}.tmp だと、同時に作ると同じファイルに書き合って衝突した)
            _atomic_write(self.csv_path, data)
        except OSError:
            pass
        return data, ""

    # --- 実験条件(以後の回答に記録される。再起動後も残す) ---
    def read_condition(self) -> Optional[str]:
        try:
            with open(self.cond_path, encoding="utf-8") as f:
                t = f.read().strip()[:MAX_TEXT]
            return t or None
        except OSError:
            return None

    def write_condition(self, c: str) -> None:
        c = (c or "").strip()[:MAX_TEXT]
        if not c:
            return
        try:
            os.makedirs(self.data_dir, exist_ok=True)
            # 【2026-11-05】原子的に書く(一時ファイル+fsync+os.replace)。書き込み途中の電源断で条件が空・半端にならない
            _atomic_write(self.cond_path, c.encode("utf-8"))
        except OSError:
            pass
