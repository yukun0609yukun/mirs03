"""呼びかけ(発話依頼)の間隔設定ファイル(純Python、ROS非依存)。

【2026-10-06追加】呼びかけ音声設定ページ(/voice_settings、docs/voice_settings.md)用。

- ファイル: `~/.nokosahen_announce_settings.json`(1台目Jetson、behavior_directorと同じ機体上)
    {"version": 1, "announce_min_interval_seconds": 60}
- behavior_directorは`AnnounceSettingsReader`でmtimeを見てホットリロードする(再起動不要)。
  優先順位: このファイル > ROSパラメータ`announce_min_interval_seconds`。
  ファイルが無い/キーが無ければパラメータの値のまま(現行動作と同じ)。
  不正なファイルは全体を拒否して直前の有効な値を維持する(behavior_rulesと同じ流儀)。
- ダッシュボードは`AnnounceSettingsStore`で検証つきに保存する(原子的置換+`.bak`)。
- 範囲は10〜600秒(下限10秒: 発話の生成+再生に20〜40秒かかるため極端に短くしても詰まるだけ)。
  ※director側でさらに `max(announce_cooldown_seconds, 値)` を取る。
"""

from __future__ import annotations

import json
import math
import os
import shutil
import tempfile
import threading
from typing import Any, Dict, List, Optional, Tuple

DEFAULT_FILE = "~/.nokosahen_announce_settings.json"
FILE_VERSION = 1
INTERVAL_MIN = 10.0
INTERVAL_MAX = 600.0
KEY = "announce_min_interval_seconds"


def _reject_constant(name: str):
    raise ValueError(f"JSONに{name}(NaN/Infinity)は使えません")


def parse_interval(value: Any) -> Tuple[Optional[float], Optional[str]]:
    """間隔(秒)を検証する。(値, エラー文)。丸めずに範囲外は拒否する。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None, "発話間隔は数値(秒)で指定してください"
    v = float(value)
    if not math.isfinite(v):
        return None, "発話間隔は有限の数値で指定してください"
    if v < INTERVAL_MIN or v > INTERVAL_MAX:
        return None, f"発話間隔は{INTERVAL_MIN:g}〜{INTERVAL_MAX:g}秒の範囲で指定してください"
    return v, None


def validate_document(doc: Any) -> Tuple[Optional[Dict[str, Any]], List[str]]:
    """ファイル内容(dict)を検証する。(正規化したdict, エラー一覧)。未知キーは無視せずエラー。"""
    if not isinstance(doc, dict):
        return None, ["設定はJSONオブジェクトにしてください"]
    errors: List[str] = []
    clean: Dict[str, Any] = {}
    for k in doc:
        if k not in (KEY, "version", "description"):
            errors.append(f"未知のキー: {k}")
    if KEY in doc:
        v, err = parse_interval(doc[KEY])
        if err:
            errors.append(err)
        else:
            clean[KEY] = v
    if errors:
        return None, errors
    return clean, []


def file_sig(path: str) -> str:
    try:
        st = os.stat(path)
    except OSError:
        return "missing"
    return f"{st.st_mtime_ns}-{st.st_size}"


class AnnounceSettingsStore:
    """ダッシュボード側: 読み取りと検証つき保存(スレッドセーフ)。"""

    def __init__(self, path: str = DEFAULT_FILE) -> None:
        self.path = os.path.expanduser(path or DEFAULT_FILE)
        self._lock = threading.Lock()

    @property
    def backup_path(self) -> str:
        return self.path + ".bak"

    def read(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "path": self.path,
            "exists": False,
            "sig": file_sig(self.path),
            "announce_min_interval_seconds": None,  # None=ファイルでは未指定(directorのパラメータ値が有効)
            "valid": True,
            "errors": [],
            "limits": {"min": INTERVAL_MIN, "max": INTERVAL_MAX},
        }
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                text = f.read()
        except FileNotFoundError:
            return out
        except (OSError, UnicodeDecodeError) as exc:
            out["exists"] = os.path.exists(self.path)
            out["valid"] = False
            out["errors"] = [f"ファイルを読めません: {exc}"]
            return out
        out["exists"] = True
        try:
            doc = json.loads(text, parse_constant=_reject_constant)
        except ValueError as exc:
            out["valid"] = False
            out["errors"] = [f"JSONとして読めません: {exc}"]
            return out
        clean, errors = validate_document(doc)
        if errors or clean is None:
            out["valid"] = False
            out["errors"] = errors
            return out
        out[KEY] = clean.get(KEY)
        return out

    def save_interval(self, value: Any) -> Dict[str, Any]:
        """間隔を検証して保存する。結果dict(ok/kind/errors)を返し、例外は投げない。"""
        v, err = parse_interval(value)
        if err:
            return {"ok": False, "kind": "invalid", "errors": [err]}
        doc = {"version": FILE_VERSION, KEY: v if v != int(v) else int(v)}
        text = json.dumps(doc, ensure_ascii=False, indent=2) + "\n"
        with self._lock:
            directory = os.path.dirname(self.path) or "."
            tmp_path = None
            bak_tmp = None
            try:
                os.makedirs(directory, exist_ok=True)
                fd, tmp_path = tempfile.mkstemp(prefix=".nokosahen_announce_", suffix=".tmp", dir=directory)
                with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as f:
                    f.write(text)
                    f.flush()
                    os.fsync(f.fileno())
                backed_up = False
                if os.path.exists(self.path):
                    bak_tmp = self.backup_path + ".tmp"
                    shutil.copy2(self.path, bak_tmp)
                    os.replace(bak_tmp, self.backup_path)
                    bak_tmp = None
                    backed_up = True
                os.replace(tmp_path, self.path)
                tmp_path = None
            except OSError as exc:
                return {
                    "ok": False,
                    "kind": "io",
                    "errors": [f"ファイルを書き込めませんでした(元のファイルは変更されていません): {exc}"],
                }
            finally:
                for p in (tmp_path, bak_tmp):
                    if p:
                        try:
                            os.remove(p)
                        except OSError:
                            pass
        return {"ok": True, "kind": None, "errors": [], "backed_up": backed_up, "sig": file_sig(self.path)}


class AnnounceSettingsReader:
    """director側: mtimeを見て必要なときだけ読み直す。例外は投げない。

    `poll()`は(新しい有効値 or None, ログに出すメッセージ or "")を返す。
    - 有効値: ファイルから読んだ間隔(秒)。ファイルが無い/キー無しの場合は None
      (=ROSパラメータ値を使う)。
    - 不正なファイルは直前の有効な状態(current)を維持し、messageにエラーを返す(同じ不正内容では1回だけ)。
    """

    def __init__(self, path: str = DEFAULT_FILE) -> None:
        self.path = os.path.expanduser(path or DEFAULT_FILE)
        self._sig: Optional[str] = None  # まだ一度も見ていない
        self.current: Optional[float] = None
        self.load_error: str = ""

    def poll(self) -> Tuple[Optional[float], str, bool]:
        """(現在の有効値, メッセージ, 変化したか)。"""
        # 内容そのもので変化を判定する(小さいファイルを1Hzで読むだけ。mtimeの粒度で同サイズの書き換えを
        # 見逃さないため)。
        try:
            with open(self.path, "rb") as f:
                raw = f.read(65536)
            sig = "c:" + str(hash(raw))
        except FileNotFoundError:
            sig = "missing"
        except OSError as exc:
            sig = "err:" + str(exc)
        if sig == self._sig:
            return self.current, "", False
        self._sig = sig
        if sig == "missing":
            changed = self.current is not None or bool(self.load_error)
            self.current = None
            self.load_error = ""
            return None, ("呼びかけ間隔ファイルが無いため、パラメータ値を使います" if changed else ""), changed
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                doc = json.loads(f.read(), parse_constant=_reject_constant)
        except (OSError, ValueError) as exc:  # UnicodeDecodeErrorはValueError
            self.load_error = f"呼びかけ間隔ファイルを読めません(直前の値を維持): {exc}"
            return self.current, self.load_error, False
        clean, errors = validate_document(doc)
        if errors or clean is None:
            self.load_error = "呼びかけ間隔ファイルが不正です(直前の値を維持): " + " / ".join(errors)
            return self.current, self.load_error, False
        self.load_error = ""
        new = clean.get(KEY)
        changed = new != self.current
        self.current = new
        return new, (f"呼びかけ間隔を{new}秒に変更しました(ファイル)" if changed and new is not None else ""), changed
