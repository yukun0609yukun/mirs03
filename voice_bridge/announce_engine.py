"""状況アナウンス(「探索中です」などの定型文を、合成済み音声のキャッシュから繰り返し流す)の中核。

【2026-10-13追加】docs/announcements.md。voice_bridge.py から使う(2台目Jetson)。
sounddevice/VOICEVOX/ROSには依存しない(合成・再生・時計などは呼び出し側から渡す)ので、実機なしで
テストできる(voice_bridge/tests/test_announce_loop.py)。

- AnnounceCache: 「文言+話者+合成パラメータ」をキーにした合成済みwavのキャッシュ(メモリ+ディスク)。
  同じ文言は2回目以降は合成しない(合成は数百ms〜数秒かかるため)。文言を変えるとキーが変わり再合成。
  ディスクは再起動後も使う。上限件数を超えたら最終使用の古いものから削除。
- AnnounceEngine: idごとの「繰り返しループ」と「1回だけ再生」を1本のワーカーで順番に処理する
  (再生が重ならない)。
    * 通常の発話(SpeechQueue)を優先: 発話の処理中/待ちがあるときはループは1秒後に再判定(その回は鳴らさない)、
      1回再生は最大ONCE_MAX_WAIT_SEC秒待って、それでも発話が続くなら捨てる(urgent=trueは発話の待ちを無視)。
    * BGMのダッキングは発話と同じ仕組み(duck_begin/duck_end)。
    * マスタースイッチOFFなら何も合成・再生しない。
    * ループは「生きている」信号(同じidへの /announce_loop の再送)が keepalive_sec 秒途絶えたら自動で止める
      (1台目が落ちてもアナウンスが鳴りっぱなしにならない)。
"""
from __future__ import annotations

import collections
import hashlib
import json
import logging
import math
import os
import re
import tempfile
import threading
import time
from typing import Callable, Optional

logger = logging.getLogger("voice_bridge.announce")

ID_RE = re.compile(r"^[a-z][a-z0-9_]{0,31}$")
TEXT_MAX = 80
# 【2026-10-30】文化祭の出店紹介(プレイリスト。docs/announcements.md)。長い文章を「事前に1回だけ合成」して保存済みwavを繰り返す。
PR_TEXT_MAX = 400  # 1パートの文字数(core/pr_intro.py の TEXT_MAX と同じ。テストで一致を確認)
PLAYLIST_MAX_PARTS = 8
PLAYLIST_INTERVAL_MAX = 3600.0  # 一巡したあとの間隔の上限(通常のループは300秒)
GAP_MIN = 1.0
GAP_MAX = 600.0
PREPARE_RETRY_SEC = 30.0  # 合成に失敗した文章を、自動では再要求しない最短時間(画面の「音声を作る」は即時)
PREPARE_MAX_TEXTS = PLAYLIST_MAX_PARTS
SYNTH_CHUNK_CHARS = 80  # 長い文章は文の区切りでこの長さ以下に分けて合成し、つなげる(VOICEVOXの1回の待ちを短くする)
SYNTH_DEADLINE_SEC = 600.0  # 1つの文章の合成にかけてよい合計時間
INTERVAL_MIN = 2.0
INTERVAL_MAX = 300.0
VOLUME_MAX = 5.0  # イベントごとの倍率の上限(発話・BGMと同じ0〜5)
MAX_LOOPS = 16  # 同時に覚えておくidの数(暴走・悪用の歯止め)
PENDING_ONCE_MAX = 4
ONCE_MAX_WAIT_SEC = 20.0
DEFER_SEC = 1.0  # 発話中などで鳴らせなかったループを再判定するまでの時間
SYNTH_RETRY_SEC = 10.0  # 合成に失敗したときの再試行までの最短時間


def _num(value) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    v = float(value)
    return v if math.isfinite(v) else None


def _clean_text(value, max_len: int = TEXT_MAX) -> tuple:
    if not isinstance(value, str):
        return None, "textは文字列で指定してください"
    t = value.strip()
    if not t:
        return None, "textが空です"
    if len(t) > max_len:
        return None, f"textは{max_len}文字以内にしてください"
    if any(ord(c) < 32 or ord(c) == 127 for c in t):
        return None, "textに制御文字は使えません"
    return t, ""


def _clean_id(value) -> tuple:
    if not isinstance(value, str) or not ID_RE.match(value):
        return None, "idは英小文字で始まる英小文字・数字・_の32文字以内で指定してください"
    return value, ""


def _clean_volume(value) -> tuple:
    v = _num(value)
    if v is None:
        return None, "volumeは有限の数値で指定してください"
    if v < 0.0 or v > VOLUME_MAX:
        return None, f"volumeは0〜{VOLUME_MAX:g}の範囲で指定してください"
    return v, ""


def parse_loop_request(data) -> tuple:
    """POST /announce_loop の本文を検証する。(clean dict, errors)。型違い・範囲外・未知キーは拒否(丸めない)。
    enabled=false(停止)のときは text/interval_sec を省略できる。"""
    if not isinstance(data, dict):
        return None, ["本文はJSONオブジェクトで指定してください"]
    errors = []
    for k in data:
        if k not in ("id", "text", "interval_sec", "volume", "enabled", "parts", "gap_sec"):
            errors.append(f"未知の項目です: {k}")
    if "parts" in data or "gap_sec" in data:
        return _parse_playlist_request(data, errors)
    enabled = data.get("enabled", True)
    if not isinstance(enabled, bool):
        errors.append("enabledは true/false で指定してください")
        enabled = True
    out = {"enabled": enabled}
    v, e = _clean_id(data.get("id"))
    if e:
        errors.append(e)
    out["id"] = v
    if enabled or "text" in data:
        t, e = _clean_text(data.get("text"))
        if e:
            errors.append(e)
        out["text"] = t
    if enabled or "interval_sec" in data:
        iv = _num(data.get("interval_sec"))
        if iv is None:
            errors.append("interval_secは有限の数値(秒)で指定してください")
        elif iv < INTERVAL_MIN or iv > INTERVAL_MAX:
            errors.append(f"interval_secは{INTERVAL_MIN:g}〜{INTERVAL_MAX:g}秒の範囲で指定してください")
        out["interval_sec"] = iv
    if "volume" in data:
        vol, e = _clean_volume(data["volume"])
        if e:
            errors.append(e)
        out["volume"] = vol
    else:
        out["volume"] = 1.0
    return (None, errors) if errors else (out, [])


def _parse_playlist_request(data: dict, errors: list) -> tuple:
    """POST /announce_loop の本文のうち、複数パートを順に流す「プレイリスト」(文化祭の出店紹介)。
    parts=[{"text","volume"?}]を順に1つずつ鳴らし、パート間はgap_sec秒、最後のパートの次はinterval_sec秒あけて最初から。
    再生中に合成はしない(事前に /announce_prepare で保存済みのwavだけを鳴らす)。enabled=falseのときはparts等を省略できる。"""
    enabled = data.get("enabled", True)
    if not isinstance(enabled, bool):
        errors.append("enabledは true/false で指定してください")
        enabled = True
    out = {"enabled": enabled, "playlist": True}
    v, e = _clean_id(data.get("id"))
    if e:
        errors.append(e)
    out["id"] = v
    if "text" in data:
        errors.append("partsを使うときはtextは指定できません")
    if enabled or "parts" in data:
        parts = data.get("parts")
        clean_parts = []
        if not isinstance(parts, list) or not (1 <= len(parts) <= PLAYLIST_MAX_PARTS):
            errors.append(f"partsは1〜{PLAYLIST_MAX_PARTS}個のリストで指定してください")
        else:
            for i, p in enumerate(parts):
                if not isinstance(p, dict):
                    errors.append(f"parts[{i}]はJSONオブジェクトで指定してください")
                    continue
                for k in p:
                    if k not in ("text", "volume"):
                        errors.append(f"parts[{i}]に未知の項目です: {k}")
                t, e = _clean_text(p.get("text"), PR_TEXT_MAX)
                if e:
                    errors.append(f"parts[{i}]: {e}")
                vol = 1.0
                if "volume" in p:
                    vol, e = _clean_volume(p["volume"])
                    if e:
                        errors.append(f"parts[{i}]: {e}")
                clean_parts.append({"text": t, "volume": vol})
        out["parts"] = clean_parts
    if enabled or "interval_sec" in data:
        iv = _num(data.get("interval_sec"))
        if iv is None:
            errors.append("interval_secは有限の数値(秒)で指定してください")
        elif iv < INTERVAL_MIN or iv > PLAYLIST_INTERVAL_MAX:
            errors.append(f"interval_secは{INTERVAL_MIN:g}〜{PLAYLIST_INTERVAL_MAX:g}秒の範囲で指定してください")
        out["interval_sec"] = iv
    if enabled or "gap_sec" in data:
        gp = _num(data.get("gap_sec", 5.0))
        if gp is None:
            errors.append("gap_secは有限の数値(秒)で指定してください")
        elif gp < GAP_MIN or gp > GAP_MAX:
            errors.append(f"gap_secは{GAP_MIN:g}〜{GAP_MAX:g}秒の範囲で指定してください")
        out["gap_sec"] = gp
    out["volume"] = 1.0
    return (None, errors) if errors else (out, [])


def parse_prepare_request(data) -> tuple:
    """POST /announce_prepare の本文。{"texts":[...最大8個、各400字以内], "start": true|false, "force": true|false}。
    start=falseは状態を見るだけ(合成を始めない)。force=trueは前回失敗した文章もすぐ再試行する。"""
    if not isinstance(data, dict):
        return None, ["本文はJSONオブジェクトで指定してください"]
    errors = []
    for k in data:
        if k not in ("texts", "start", "force"):
            errors.append(f"未知の項目です: {k}")
    texts = data.get("texts")
    clean = []
    if not isinstance(texts, list) or not (1 <= len(texts) <= PREPARE_MAX_TEXTS):
        errors.append(f"textsは1〜{PREPARE_MAX_TEXTS}個のリストで指定してください")
    else:
        for i, t in enumerate(texts):
            c, e = _clean_text(t, PR_TEXT_MAX)
            if e:
                errors.append(f"texts[{i}]: {e}")
            clean.append(c)
    flags = {}
    for k in ("start", "force"):
        v = data.get(k, k == "start")
        if not isinstance(v, bool):
            errors.append(f"{k}は true/false で指定してください")
        flags[k] = v is True
    if errors:
        return None, errors
    return {"texts": clean, **flags}, []


def parse_once_request(data) -> tuple:
    """POST /announce の本文を検証する。(clean dict, errors)。"""
    if not isinstance(data, dict):
        return None, ["本文はJSONオブジェクトで指定してください"]
    errors = []
    for k in data:
        if k not in ("id", "text", "volume", "urgent", "cached_only"):
            errors.append(f"未知の項目です: {k}")
    out = {}
    v, e = _clean_id(data.get("id"))
    if e:
        errors.append(e)
    out["id"] = v
    cached_only = data.get("cached_only", False)
    if not isinstance(cached_only, bool):
        errors.append("cached_onlyは true/false で指定してください")
    out["cached_only"] = cached_only is True
    # cached_only=true(保存済みのwavだけ鳴らし、無ければ合成せず破棄)のときだけ長い文章(文化祭の出店紹介の試し再生)を許す
    t, e = _clean_text(data.get("text"), PR_TEXT_MAX if out["cached_only"] else TEXT_MAX)
    if e:
        errors.append(e)
    out["text"] = t
    if "volume" in data:
        vol, e = _clean_volume(data["volume"])
        if e:
            errors.append(e)
        out["volume"] = vol
    else:
        out["volume"] = 1.0
    urgent = data.get("urgent", False)
    if not isinstance(urgent, bool):
        errors.append("urgentは true/false で指定してください")
    out["urgent"] = urgent is True
    return (None, errors) if errors else (out, [])


class AnnounceCache:
    """合成済みwav(バイト列)のキャッシュ。キー=文言+タグ(話者・合成パラメータ)のハッシュ。スレッドセーフ。"""

    def __init__(self, directory: str, synth: Callable[[str], bytes], tag: str = "", max_items: int = 64,
                 mem_items: int = 16, max_bytes: int = 0) -> None:
        self.dir = os.path.expanduser(directory)
        self.synth = synth
        self.tag = tag
        self.max_items = max(1, int(max_items))
        self.max_bytes = max(0, int(max_bytes))  # 0=容量では削らない(文化祭の長い音声が増えるので本番は上限を入れる)
        self.mem_items = max(1, int(mem_items))
        self._mem: "collections.OrderedDict[str, bytes]" = collections.OrderedDict()
        self._lock = threading.Lock()
        self.hits = 0
        self.misses = 0
        self.synth_count = 0
        self.disk_error = ""

    def key(self, text: str) -> str:
        return hashlib.sha256(json.dumps([text, self.tag], ensure_ascii=False).encode("utf-8")).hexdigest()[:32]

    def _path(self, key: str) -> str:
        return os.path.join(self.dir, key + ".wav")

    def has(self, text: str) -> bool:
        k = self.key(text)
        with self._lock:
            if k in self._mem:
                return True
        return os.path.isfile(self._path(k))

    def count(self) -> int:
        try:
            return sum(1 for n in os.listdir(self.dir) if n.endswith(".wav"))
        except OSError:
            return 0

    def get(self, text: str, allow_synth: bool = True) -> tuple:
        """(wavバイト列 or None, 出どころ "memory"|"disk"|"synth")。合成に失敗したら例外をそのまま投げる。"""
        k = self.key(text)
        with self._lock:
            if k in self._mem:
                self._mem.move_to_end(k)
                self.hits += 1
                self._touch(k)
                return self._mem[k], "memory"
        try:
            with open(self._path(k), "rb") as f:
                blob = f.read()
        except OSError:
            blob = b""
        if blob:
            with self._lock:
                self.hits += 1
                self._remember(k, blob)
            self._touch(k)
            return blob, "disk"
        if not allow_synth:
            return None, ""
        with self._lock:
            self.misses += 1
        blob = self.synth(text)
        if not blob:
            raise RuntimeError("合成結果が空です")
        with self._lock:
            self.synth_count += 1
            self._remember(k, blob)
        self._store(k, blob)
        return blob, "synth"

    def _remember(self, k: str, blob: bytes) -> None:
        self._mem[k] = blob
        self._mem.move_to_end(k)
        while len(self._mem) > self.mem_items:
            self._mem.popitem(last=False)

    def _touch(self, k: str) -> None:
        try:
            os.utime(self._path(k), None)  # 最終使用時刻(古いものから削除する目安)
        except OSError:
            pass

    def _store(self, k: str, blob: bytes) -> None:
        tmp = None
        try:
            os.makedirs(self.dir, exist_ok=True)
            fd, tmp = tempfile.mkstemp(prefix=".ann_", suffix=".tmp", dir=self.dir)
            with os.fdopen(fd, "wb") as f:
                f.write(blob)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self._path(k))
            tmp = None
            self.disk_error = ""
            self._evict()
        except OSError as exc:  # ディスクに書けなくてもメモリのキャッシュで動く
            self.disk_error = str(exc)
            logger.warning("アナウンスのキャッシュをディスクに保存できません(メモリのみで継続): %s", exc)
        finally:
            if tmp:
                try:
                    os.remove(tmp)
                except OSError:
                    pass

    def _evict(self) -> None:
        try:
            files = [os.path.join(self.dir, n) for n in os.listdir(self.dir) if n.endswith(".wav")]
        except OSError:
            return
        stats = []
        for p in files:
            try:
                st = os.stat(p)
                stats.append((st.st_mtime, st.st_size, p))
            except OSError:
                pass
        stats.sort()
        total = sum(sz for _m, sz, _p in stats)
        drop = []
        while stats and (len(stats) > self.max_items or (self.max_bytes and total > self.max_bytes and len(stats) > 1)):
            _m, sz, p = stats.pop(0)  # 最終使用の古いものから(直前に保存した最新の1件は容量超過でも残す)
            total -= sz
            drop.append(p)
        for p in drop:
            try:
                os.remove(p)
                with self._lock:
                    self._mem.pop(os.path.basename(p)[:-4], None)
            except OSError:
                pass

    def total_bytes(self) -> int:
        try:
            return sum(os.stat(os.path.join(self.dir, n)).st_size for n in os.listdir(self.dir) if n.endswith(".wav"))
        except OSError:
            return 0

    def stats(self) -> dict:
        return {"dir": self.dir, "items": self.count(), "max_items": self.max_items, "bytes": self.total_bytes(),
                "max_bytes": self.max_bytes, "hits": self.hits,
                "misses": self.misses, "synth_count": self.synth_count, "disk_error": self.disk_error}


_SENTENCE_END = "。！？!?\n"


def split_for_synthesis(text: str, max_chars: int = SYNTH_CHUNK_CHARS) -> list:
    """長い文章を、文の区切り(。！？)で max_chars 字前後以下のかたまりに分ける(合成を1回あたり短くするため)。
    区切りが無い長い文は読点(、)、それも無ければ max_chars 字ごとに切る。順番どおりにつなげると元の文章(改行は除く)になる。"""
    sentences, cur = [], ""
    for ch in text.replace("\r", ""):
        if ch == "\n":
            if cur:
                sentences.append(cur)
                cur = ""
            continue
        cur += ch
        if ch in _SENTENCE_END:
            sentences.append(cur)
            cur = ""
    if cur:
        sentences.append(cur)
    pieces = []
    for s in sentences:
        while len(s) > max_chars:
            cut = s.rfind("、", 0, max_chars)
            cut = cut + 1 if cut > 0 else max_chars
            pieces.append(s[:cut])
            s = s[cut:]
        if s:
            pieces.append(s)
    chunks, cur = [], ""
    for p in pieces:  # 短い文は、max_chars を超えない範囲でまとめる(合成の呼び出し回数を減らす)
        if cur and len(cur) + len(p) > max_chars:
            chunks.append(cur)
            cur = ""
        cur += p
    if cur:
        chunks.append(cur)
    return [c for c in chunks if c.strip()]


def join_wavs(blobs: list, pause_sec: float = 0.25) -> bytes:
    """同じ形式(サンプルレート・ビット数・チャンネル数)のwavを、間に pause_sec 秒の無音を挟んでつなげる(標準ライブラリのみ)。"""
    import io
    import wave

    if len(blobs) == 1:
        return blobs[0]
    out = io.BytesIO()
    params = None
    with wave.open(out, "wb") as w:
        for b in blobs:
            with wave.open(io.BytesIO(b), "rb") as r:
                p = r.getparams()
                if params is None:
                    params = p
                    w.setnchannels(p.nchannels)
                    w.setsampwidth(p.sampwidth)
                    w.setframerate(p.framerate)
                    silence = b"\x00" * (int(p.framerate * pause_sec) * p.nchannels * p.sampwidth)
                elif (p.nchannels, p.sampwidth, p.framerate) != (params.nchannels, params.sampwidth, params.framerate):
                    raise ValueError("wavの形式がそろっていません")
                else:
                    w.writeframes(silence)
                w.writeframes(r.readframes(r.getnframes()))
    return out.getvalue()


def synthesize_long(text: str, synth_one: Callable[[str], bytes], max_chars: int = SYNTH_CHUNK_CHARS,
                    deadline_sec: float = SYNTH_DEADLINE_SEC, clock: Callable[[], float] = time.monotonic) -> bytes:
    """長い文章を分割して合成し、1本のwavにする。max_chars以下ならそのまま1回で合成(既存の短いアナウンスと同じ)。
    合計時間が deadline_sec を超えたら例外(途中までの音声は保存しない=壊れた/途中で切れた音声を再生しない)。"""
    if len(text) <= max_chars:
        return synth_one(text)
    start = clock()
    blobs = []
    for chunk in split_for_synthesis(text, max_chars):
        if clock() - start > deadline_sec:
            raise TimeoutError(f"合成に{deadline_sec:.0f}秒以上かかったため中止しました")
        blobs.append(synth_one(chunk))
    if not blobs:
        raise ValueError("合成する文章が空です")
    return join_wavs(blobs)


class PrepareWorker:
    """文章を「事前に1回だけ」合成してキャッシュ(ディスク)へ保存する背景ワーカー(文化祭の出店紹介。画面をブロックしない)。

    - request(texts, force): すでにキャッシュ済みなら何もしない(再合成しない)。未作成なら待ち行列へ。
      前回失敗した文章は PREPARE_RETRY_SEC 秒は自動で再試行しない(force=trueなら即時)。
    - 合成は1本ずつ順番に(VOICEVOXを占有しすぎない)。再生側は合成せず、ここで作った保存済みwavだけを鳴らす。
    - status(texts): 文章ごとに {"state": "ready"|"queued"|"synthesizing"|"failed"|"missing", "error"}。
    """

    def __init__(self, cache: AnnounceCache, clock: Callable[[], float] = time.monotonic) -> None:
        self.cache = cache
        self.clock = clock
        self._lock = threading.Lock()
        self._queue: "collections.deque[str]" = collections.deque()
        self._info: "collections.OrderedDict[str, dict]" = collections.OrderedDict()  # 文章のキー -> 状態
        self._current: Optional[str] = None
        self._wake = threading.Event()
        self._stop = False
        self._thread: Optional[threading.Thread] = None
        self.done_count = 0
        self.failed_count = 0

    def _key(self, text: str) -> str:
        return self.cache.key(text)

    def request(self, texts: list, force: bool = False) -> None:
        now = self.clock()
        with self._lock:
            for t in texts:
                k = self._key(t)
                if self.cache.has(t):
                    self._info.pop(k, None)
                    continue
                st = self._info.get(k)
                if k == self._current or (st and st["state"] == "queued"):
                    continue
                if st and st["state"] == "failed" and not force and now - st["at"] < PREPARE_RETRY_SEC:
                    continue
                self._info[k] = {"state": "queued", "error": "", "at": now, "text": t}
                self._queue.append(t)
            while len(self._info) > 64:
                self._info.popitem(last=False)
        self._wake.set()

    def status(self, texts: list) -> list:
        out = []
        with self._lock:
            for t in texts:
                k = self._key(t)
                if self.cache.has(t):
                    out.append({"state": "ready", "error": ""})
                    continue
                st = self._info.get(k)
                if st is None:
                    out.append({"state": "missing", "error": ""})
                else:
                    out.append({"state": st["state"], "error": st["error"]})
        return out

    def summary(self) -> dict:
        with self._lock:
            return {"queued": len(self._queue), "synthesizing": self._current is not None,
                    "done": self.done_count, "failed": self.failed_count}

    def process_one(self) -> bool:
        """待ちの先頭を1つ合成して保存する(ワーカースレッドから呼ぶ。テストからも直接呼べる)。何かしたらTrue。"""
        with self._lock:
            if not self._queue:
                return False
            text = self._queue.popleft()
            k = self._key(text)
            self._current = k
            self._info[k] = {"state": "synthesizing", "error": "", "at": self.clock(), "text": text}
        try:
            self.cache.get(text, allow_synth=True)  # 保存はキャッシュ側で一時ファイル→置き換え(途中で壊れたwavを残さない)
            err = ""
        except Exception as exc:  # noqa: BLE001 - 合成の失敗でロボット・他の発話を巻き込まない
            err = f"{type(exc).__name__}: {exc}"[:200]
            logger.warning("事前合成に失敗(%s): %s", text[:20], err)
        with self._lock:
            self._current = None
            if err:
                self._info[k] = {"state": "failed", "error": err, "at": self.clock(), "text": text}
                self.failed_count += 1
            else:
                self._info.pop(k, None)
                self.done_count += 1
        return True

    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="announce_prepare", daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._stop = True
        self._wake.set()

    def _run(self) -> None:
        while not self._stop:
            try:
                did = self.process_one()
            except Exception:  # noqa: BLE001
                logger.exception("事前合成で予期しないエラー")
                did = False
            if not did:
                self._wake.wait(timeout=1.0)
                self._wake.clear()


class _NotPrepared(Exception):
    """保存済みのwavが無い(cached_only なのに未合成)。"""


class _Loop:
    def __init__(self, loop_id: str) -> None:
        self.id = loop_id
        self.text = ""
        self.parts: list = []  # プレイリスト: [{"text","volume"}]。空なら通常の1文ループ
        self.gap = 5.0
        self.idx = 0  # 次に鳴らすパート
        self.rounds = 0  # 最後のパートまで進んだ回数
        self.skipped_unprepared = 0  # 音声が未作成で飛ばした回数
        self.interval = 8.0
        self.volume = 1.0
        self.active = False
        self.reason = ""  # 止まっている理由("stopped"/"expired")
        self.last_keepalive = 0.0
        self.next_due = 0.0
        self.last_played_mono: Optional[float] = None
        self.last_played_ts: Optional[float] = None
        self.plays = 0
        self.skipped = 0
        self.last_error = ""


class AnnounceEngine:
    def __init__(self, cache: AnnounceCache, decode: Callable, play: Callable, is_muted: Callable[[], bool],
                 speech_busy: Callable[[], bool], duck_begin: Callable[[], None], duck_end: Callable[[], None],
                 clock: Callable[[], float] = time.monotonic, wall: Callable[[], float] = time.time,
                 keepalive_sec: float = 15.0, tick_sec: float = 0.2,
                 stop_playback: Optional[Callable[[], None]] = None) -> None:
        self.stop_playback = stop_playback  # 再生中の音をその場で止める(長いプレイリストを、停止・緊急停止の通知のために中断する)
        self._playing_playlist: Optional[str] = None
        self.cache = cache
        self.decode = decode  # wavバイト列 -> (data, samplerate)
        self.play = play  # (data, samplerate, event_volume) -> None(音量の掛け合わせは呼び出し側)
        self.is_muted = is_muted
        self.speech_busy = speech_busy
        self.duck_begin = duck_begin
        self.duck_end = duck_end
        self.clock = clock
        self.wall = wall
        self.keepalive_sec = float(keepalive_sec)
        self.tick_sec = tick_sec
        self._lock = threading.Lock()
        self._loops: "dict[str, _Loop]" = {}
        self._once: "collections.deque[dict]" = collections.deque()
        self._wake = threading.Event()
        self._stop = False
        self._thread: Optional[threading.Thread] = None
        self.once_played = 0
        self.once_dropped = 0
        self.last_once: dict = {}
        self._once_info: "dict[str, dict]" = {}  # idごとの1回再生の実績(画面表示用)

    # ---- 受付 ----
    def set_loop(self, loop_id: str, enabled: bool, text: str = "", interval: float = 8.0, volume: float = 1.0,
                 parts: Optional[list] = None, gap: float = 5.0) -> dict:
        """parts を渡すとプレイリスト(複数パートを順に鳴らす。保存済みのwavだけ・合成しない)。省略すると従来どおり1文のループ。"""
        now = self.clock()
        if parts is not None:
            return self._set_playlist(loop_id, enabled, parts, interval, gap, now)
        with self._lock:
            lp = self._loops.get(loop_id)
            if lp is None:
                if not enabled:
                    return {"id": loop_id, "active": False, "known": False}
                if len(self._loops) >= MAX_LOOPS:
                    raise ValueError(f"ループは最大{MAX_LOOPS}個までです")
                lp = _Loop(loop_id)
                self._loops[loop_id] = lp
            if not enabled:
                if lp.active:
                    lp.active, lp.reason = False, "stopped"
                return self._loop_state(lp, now)
            newly = not lp.active
            changed_text = text != lp.text
            lp.last_keepalive = now
            lp.active, lp.reason = True, ""
            old_interval = lp.interval
            lp.text, lp.interval, lp.volume = text, float(interval), float(volume)
            if newly or changed_text or lp.last_played_mono is None:
                lp.next_due = now
                lp.last_error = ""
            elif lp.interval != old_interval:
                lp.next_due = min(lp.next_due, lp.last_played_mono + lp.interval)
        self._wake.set()
        with self._lock:
            return self._loop_state(lp, now)

    def _set_playlist(self, loop_id: str, enabled: bool, parts: list, interval: float, gap: float, now: float) -> dict:
        norm = [{"text": p["text"], "volume": float(p.get("volume", 1.0))} for p in parts]
        with self._lock:
            lp = self._loops.get(loop_id)
            if lp is None:
                if not enabled:
                    return {"id": loop_id, "active": False, "known": False}
                if len(self._loops) >= MAX_LOOPS:
                    raise ValueError(f"ループは最大{MAX_LOOPS}個までです")
                lp = _Loop(loop_id)
                self._loops[loop_id] = lp
            if not enabled:
                if lp.active:
                    lp.active, lp.reason = False, "stopped"
                    self._interrupt_playlist_locked(loop_id)
                return self._loop_state(lp, now)
            newly = not lp.active
            changed = [p["text"] for p in norm] != [p["text"] for p in lp.parts]
            lp.last_keepalive = now
            lp.active, lp.reason = True, ""
            old_interval, old_gap = lp.interval, lp.gap
            lp.parts, lp.interval, lp.gap, lp.text = norm, float(interval), float(gap), ""
            if changed or lp.idx >= len(norm):
                lp.idx = 0
            if newly or changed:
                lp.next_due = now  # 起動直後・内容が変わった直後はすぐ(newlyは停止前のパートから続ける)
                lp.last_error = ""
            elif lp.last_played_mono is not None and (lp.interval != old_interval or lp.gap != old_gap):
                nxt = lp.last_played_mono + (lp.gap if lp.idx > 0 else lp.interval)
                lp.next_due = min(lp.next_due, nxt)
        self._wake.set()
        with self._lock:
            return self._loop_state(lp, now)

    def _interrupt_playlist_locked(self, loop_id: str) -> None:
        """そのプレイリストを再生中なら音を止める(止めた分は再生済みとして次のパートへ進む)。"""
        if self._playing_playlist == loop_id and self.stop_playback is not None:
            try:
                self.stop_playback()
            except Exception as exc:  # noqa: BLE001
                logger.warning("再生の中断に失敗: %s", exc)

    def enqueue_once(self, once_id: str, text: str, volume: float = 1.0, urgent: bool = False,
                     cached_only: bool = False) -> None:
        item = {"id": once_id, "text": text, "volume": float(volume), "urgent": bool(urgent), "queued": self.clock(),
                "cached_only": bool(cached_only)}
        with self._lock:
            if urgent and self._playing_playlist:
                self._interrupt_playlist_locked(self._playing_playlist)  # 緊急停止の通知は、長い紹介の再生が終わるのを待たない
            self._once = collections.deque(x for x in self._once if x["id"] != once_id)  # 同じidは最新だけ
            self._once.append(item)
            while len(self._once) > PENDING_ONCE_MAX:
                self._once.popleft()
                self.once_dropped += 1
        self._wake.set()

    def on_master_change(self, enabled: bool) -> None:
        if not enabled:
            with self._lock:
                n = len(self._once)
                self._once.clear()
                self.once_dropped += n

    # ---- ワーカー ----
    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="announce", daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._stop = True
        self._wake.set()

    def _run(self) -> None:
        while not self._stop:
            try:
                did = self.tick()
            except Exception:  # noqa: BLE001 - アナウンスの失敗で発話・BGMを巻き込まない
                logger.exception("アナウンス処理で予期しないエラー")
                did = False
            if not did:
                self._wake.wait(timeout=self.tick_sec)
                self._wake.clear()

    def tick(self) -> str:
        """1ステップ進める。何かした(再生・合成失敗・破棄)ときは内容の文字列、何もしなければ ""。"""
        now = self.clock()
        pick = None
        with self._lock:
            for lp in self._loops.values():
                if lp.active and now - lp.last_keepalive > self.keepalive_sec:
                    lp.active, lp.reason = False, "expired"
                    logger.info("アナウンスループ %s: 生きている信号が%.0f秒途絶えたため停止", lp.id, self.keepalive_sec)
            if self.is_muted():
                self.once_dropped += len(self._once)
                self._once.clear()
                for lp in self._loops.values():
                    if lp.active and lp.next_due < now + DEFER_SEC:
                        lp.next_due = now + DEFER_SEC  # ONに戻った瞬間に溜まった分をまとめて鳴らさない
                return ""
            busy = None
            while self._once:
                it = self._once[0]
                if now - it["queued"] > ONCE_MAX_WAIT_SEC:
                    self._once.popleft()
                    self.once_dropped += 1
                    logger.info("アナウンス %s: 発話が続いていたため再生せず破棄", it["id"])
                    continue
                if not it["urgent"]:
                    busy = self.speech_busy() if busy is None else busy
                    if busy:
                        break  # 発話が終わるまで待つ(ループより先に鳴らすので、ループも待たせる)
                pick = ("once", self._once.popleft())
                break
            if pick is None and not self._once:
                due = [lp for lp in self._loops.values() if lp.active and lp.next_due <= now]
                if due:
                    lp = min(due, key=lambda x: x.next_due)
                    busy = self.speech_busy() if busy is None else busy
                    if busy:
                        lp.next_due = now + DEFER_SEC
                        lp.skipped += 1
                    else:
                        pick = ("loop", lp)
        if pick is None:
            return ""
        kind, obj = pick
        return self._play_one(kind, obj)

    def _play_one(self, kind: str, obj) -> str:
        loop_id = obj.id if kind == "loop" else obj["id"]
        playlist = kind == "loop" and bool(obj.parts)
        if playlist:
            with self._lock:
                idx = obj.idx if obj.idx < len(obj.parts) else 0
                part = obj.parts[idx]
            text, volume = part["text"], part["volume"]
        else:
            text = obj.text if kind == "loop" else obj["text"]
            volume = obj.volume if kind == "loop" else obj["volume"]
        # プレイリスト・cached_only は「保存済みのwavだけ」鳴らす。無ければ合成(推論)せず、そのパートを飛ばす(再生中はVOICEVOXを呼ばない)
        cached_only = playlist or (kind == "once" and bool(obj.get("cached_only")))
        try:
            blob, _src = self.cache.get(text, allow_synth=not cached_only)
            if blob is None:
                raise _NotPrepared("音声が未作成です(管理画面の「音声を作る」を実行してください)")
            data, sr = self.decode(blob)
        except _NotPrepared as exc:
            logger.warning("アナウンス %s: %s", loop_id, exc)
            with self._lock:
                if kind == "loop":
                    obj.last_error = str(exc)[:200]
                    obj.skipped_unprepared += 1
                    self._advance_playlist(obj, self.clock(), played=False)
                else:
                    self.once_dropped += 1
            return "not_prepared"
        except Exception as exc:  # noqa: BLE001
            msg = f"合成に失敗: {exc}"
            logger.warning("アナウンス %s: %s", loop_id, msg)
            with self._lock:
                if kind == "loop":
                    obj.last_error = msg[:200]
                    obj.next_due = self.clock() + max(obj.interval, SYNTH_RETRY_SEC)
            return "synth_failed"
        if self.is_muted():  # 合成中にマスターがOFFになった
            return "muted"
        self.duck_begin()
        if playlist:
            with self._lock:
                self._playing_playlist = loop_id
        try:
            self.play(data, sr, volume)
        except Exception as exc:  # noqa: BLE001
            msg = f"再生に失敗: {exc}"
            logger.warning("アナウンス %s: %s", loop_id, msg)
            with self._lock:
                if kind == "loop":
                    obj.last_error = msg[:200]
                    obj.next_due = self.clock() + max(obj.interval, SYNTH_RETRY_SEC)
            return "play_failed"
        finally:
            if playlist:
                with self._lock:
                    self._playing_playlist = None
            self.duck_end()
        done = self.clock()
        with self._lock:
            if kind == "loop":
                obj.plays += 1
                obj.last_played_mono = done
                obj.last_played_ts = self.wall()
                obj.last_error = ""
                if playlist:
                    self._advance_playlist(obj, done, played=True)
                else:
                    obj.next_due = done + obj.interval  # 再生が終わってから interval 秒あける
            else:
                self.once_played += 1
                self.last_once = {"id": loop_id, "text": text, "ts": self.wall()}
                info = self._once_info.setdefault(loop_id, {"plays": 0})
                info.update(text=text, last_played_ts=self.wall(), plays=info["plays"] + 1)
                if len(self._once_info) > 64:
                    self._once_info.pop(next(iter(self._once_info)))
        return "played"

    def _advance_playlist(self, lp: _Loop, now: float, played: bool) -> None:
        """プレイリストの次のパートへ(ロックを持って呼ぶ)。途中のパートの次は gap 秒後、最後のパートの次は interval 秒後に最初から。
        未作成で飛ばしたときも同じ間隔で進める(全部未作成でも短い周期で暴走しない)。"""
        if not lp.parts:
            return
        nxt = lp.idx + 1 if lp.idx < len(lp.parts) else 0
        if nxt >= len(lp.parts):
            lp.idx = 0
            lp.rounds += 1
            lp.next_due = now + lp.interval
        else:
            lp.idx = nxt
            lp.next_due = now + lp.gap

    # ---- 状態 ----
    def _loop_state(self, lp: _Loop, now: float) -> dict:
        return {
            "id": lp.id,
            "active": lp.active,
            "reason": lp.reason,
            "text": lp.text,
            "playlist": bool(lp.parts),
            "parts": len(lp.parts),
            "next_part": lp.idx if lp.parts else None,
            "rounds": lp.rounds,
            "skipped_unprepared": lp.skipped_unprepared,
            "gap_sec": lp.gap if lp.parts else None,
            "interval_sec": lp.interval,
            "volume": lp.volume,
            "cached": (all(self.cache.has(p["text"]) for p in lp.parts) if lp.parts else (bool(lp.text) and self.cache.has(lp.text))),
            "plays": lp.plays,
            "skipped_for_speech": lp.skipped,
            "last_played_ts": lp.last_played_ts,
            "next_in_sec": round(max(0.0, lp.next_due - now), 1) if lp.active else None,
            "expires_in_sec": round(max(0.0, self.keepalive_sec - (now - lp.last_keepalive)), 1) if lp.active else None,
            "last_error": lp.last_error,
        }

    def state(self, texts=None) -> dict:
        """texts: キャッシュ済みか調べたい文言(最大16個)。結果は "cached": {文言: bool}。"""
        now = self.clock()
        cached = {t: self.cache.has(t) for t in list(texts or [])[:16] if isinstance(t, str) and t}
        with self._lock:
            loops = {k: self._loop_state(lp, now) for k, lp in self._loops.items()}
            pending = [x["id"] for x in self._once]
        return {
            "ok": True,
            "muted": bool(self.is_muted()),
            "keepalive_sec": self.keepalive_sec,
            "loops": loops,
            "pending_once": pending,
            "once_played": self.once_played,
            "once_dropped": self.once_dropped,
            "last_once": dict(self.last_once),
            "once": {k: dict(v) for k, v in self._once_info.items()},
            "cached": cached,
            "cache": self.cache.stats(),
        }
