"""ビヘイビアツリーファイルの「検証つき保存」と、編集ページ(/behavior_tree)用のAPIロジック(ROS非依存)。

ダッシュボード(dashboard_common.make_dashboard_handler)が使う。ロボットを直接動かすことは無い
(木の書き換えと、directorが配信する木の状態 /nokosahen/behavior_tree/status の中継だけ)。
検証は behavior_tree.py 自身の関数(validate_document)をそのまま使うので、director が読むときの
判定と食い違わない(ここで通ったものだけが保存される)。

保存の安全策は behavior_rules_store.RulesFileStore と同じ:
1つでも不正があれば何も書かない / 一時ファイル→fsync→os.replace / 直前のファイルを`.bak`に残す /
編集開始後に別の書き換えがあれば(sigが違う)上書きせず競合として返す。
director は木ファイルのmtime変化を1秒以内に自動検知して読み込む。
"""

from __future__ import annotations

import json
import os
import shutil
import tempfile
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from . import behavior_tree as bt
from .behavior_rules_store import file_sig, parse_json_strict as _parse_json_strict

MAX_BODY_BYTES = 512 * 1024


def parse_json_strict(raw: bytes) -> Tuple[Optional[Any], Optional[str]]:
    """NaN/Infinityを拒否する厳格なパース。深すぎるJSON(RecursionError)も理由つきで拒否する。"""
    try:
        return _parse_json_strict(raw)
    except RecursionError:
        return None, "JSONの入れ子が深すぎます"


class TreeFileStore:
    """木ファイル1つの読み書き。スレッドセーフ(保存は直列化)。"""

    def __init__(self, path: str) -> None:
        self.path = os.path.expanduser(path or bt.DEFAULT_TREE_FILE)
        self._lock = threading.Lock()

    @property
    def backup_path(self) -> str:
        return self.path + ".bak"

    def read(self) -> Dict[str, Any]:
        """現在のファイル内容(GET用)。壊れていても例外は投げず、理由を返す。"""
        out: Dict[str, Any] = {
            "path": self.path,
            "backup_path": self.backup_path,
            "backup_exists": os.path.exists(self.backup_path),
            "exists": False,
            "sig": file_sig(self.path),
            "document": None,
            "valid": True,
            "errors": [],
            "warnings": [],
            "raw_text": None,
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
            doc = json.loads(text, parse_constant=bt._reject_constant)  # noqa: SLF001
        except (ValueError, bt.TreeError, RecursionError) as exc:
            out["valid"] = False
            out["errors"] = [f"JSONとして読めません: {exc}"]
            out["raw_text"] = text[:20000]
            return out
        errors, warnings, norm = bt.validate_document(doc)
        out["document"] = doc if isinstance(doc, dict) else None
        out["valid"] = not errors
        out["errors"] = errors
        out["warnings"] = warnings
        if norm is not None:
            out["document"] = {k: v for k, v in norm.items() if k != "node_count"}
            out["doc_hash"] = bt.doc_hash(norm)
        if not isinstance(doc, dict):
            out["raw_text"] = text[:20000]
        return out

    def save(self, doc: Any, expected_sig: Optional[str] = None) -> Dict[str, Any]:
        """検証→バックアップ→原子的に置き換え。結果をdictで返す(例外は投げない)。
        kind: "invalid"(検証NG) | "conflict" | "io"(書き込み失敗) | None"""
        errors, warnings, norm = bt.validate_document(doc)
        if errors or norm is None:
            return {"ok": False, "kind": "invalid", "errors": errors, "warnings": warnings}
        text = bt.document_text(norm)
        with self._lock:
            current = file_sig(self.path)
            if expected_sig is not None and expected_sig != current:
                return {
                    "ok": False,
                    "kind": "conflict",
                    "errors": [
                        "編集を始めた後に木のファイルが別の操作で書き換えられました。"
                        "上書きを避けるため保存しませんでした。ページを再読み込みして最新の内容から編集し直してください。"
                    ],
                    "sig": current,
                }
            directory = os.path.dirname(self.path) or "."
            tmp_path = None
            bak_tmp = None
            try:
                os.makedirs(directory, exist_ok=True)
                fd, tmp_path = tempfile.mkstemp(prefix=".nokosahen_tree_", suffix=".tmp", dir=directory)
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
            return {
                "ok": True,
                "kind": None,
                "errors": [],
                "warnings": warnings,
                "sig": file_sig(self.path),
                "backed_up": backed_up,
                "node_count": norm["node_count"],
                "doc_hash": bt.doc_hash(norm),
            }

    def reset(self, expected_sig: Optional[str] = None) -> Dict[str, Any]:
        """既定に戻す=木のファイルを退避(.bak)して無くす(従来のルール表だけの動作に戻る)。"""
        with self._lock:
            current = file_sig(self.path)
            if expected_sig is not None and expected_sig != current:
                return {"ok": False, "kind": "conflict", "errors": [
                    "編集を始めた後にファイルが書き換えられました。ページを再読み込みしてからやり直してください。"],
                    "sig": current}
            if not os.path.exists(self.path):
                return {"ok": True, "kind": None, "errors": [], "removed": False, "sig": "missing"}
            try:
                shutil.copy2(self.path, self.backup_path)
                os.remove(self.path)
            except OSError as exc:
                return {"ok": False, "kind": "io", "errors": [f"既定に戻せませんでした: {exc}"]}
            return {"ok": True, "kind": None, "errors": [], "removed": True, "sig": "missing"}


def _pin_refs(node: dict, nid: str = "0") -> List[Tuple[str, dict]]:
    """木の中でピンを指しているノード(id, ノード辞書)。"""
    out = []
    if isinstance(node, dict):
        params = node.get("params") if isinstance(node.get("params"), dict) else {}
        if params.get("pin_id"):
            out.append((nid, node))
        for i, c in enumerate(node.get("children") or []):
            out.extend(_pin_refs(c, f"{nid}.{i}"))
    return out


class BehaviorTreeController:
    """ビヘイビアツリー編集ページ(/behavior_tree)用。
    - 木ファイル(behavior_directorの`tree_file`と同じパス)の読み込み・検証つき保存・既定に戻す。
    - `/nokosahen/behavior_tree/status`(1Hz)は、ページを開いている間だけ購読する(LazySubscription)。
    """

    STATE_TOPIC = "/nokosahen/behavior_tree/status"
    STATE_STALE_SEC = 5.0
    WARMUP_SEC = 4.0

    def __init__(self, node, tree_file: str, string_type, lazy_cls,
                 pin_controller=None, pins_payload_fn: Optional[Callable[[Any, Any], dict]] = None) -> None:
        self.node = node
        self.pin_controller = pin_controller
        self._pins_payload_fn = pins_payload_fn
        self.store = TreeFileStore(tree_file)
        self._lock = threading.Lock()
        self._state: Optional[Dict[str, Any]] = None
        self._rx = 0.0
        self._sub_started = 0.0
        self._lazy = lazy_cls(
            node, string_type, self.STATE_TOPIC, self._on_state, depth=2,
            idle_timeout_sec=15.0, on_unsubscribe=self._clear_state,
        )

    # --- 稼働状況 ---

    def _on_state(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(data, dict):
            with self._lock:
                self._state = data
                self._rx = time.monotonic()

    def _clear_state(self) -> None:
        with self._lock:
            self._state = None
            self._rx = 0.0

    def saved_hash(self, saved: Optional[Dict[str, Any]] = None) -> str:
        saved = saved if saved is not None else self.store.read()
        if not saved["exists"]:
            return bt.doc_hash(bt.default_document())
        return saved.get("doc_hash") or ""

    def live_state(self) -> Dict[str, Any]:
        was_active = self._lazy.active
        self._lazy.touch()
        now = time.monotonic()
        with self._lock:
            if not was_active:
                self._sub_started = now
                self._state = None
                self._rx = 0.0
            state, rx, started = self._state, self._rx, self._sub_started
        out: Dict[str, Any] = {
            "status": "waiting", "state": None, "age_s": None, "message": "",
            "dashboard_tree_file": self.store.path, "path_mismatch": False, "sync": "unknown",
        }
        if state is not None and rx:
            age = now - rx
            out["age_s"] = round(age, 1)
            if age <= self.STATE_STALE_SEC:
                out["status"] = "ok"
                out["state"] = state
                theirs = state.get("tree_file")
                if isinstance(theirs, str) and theirs and os.path.normpath(theirs) != os.path.normpath(self.store.path):
                    out["path_mismatch"] = True
                saved = self.saved_hash()
                loaded = state.get("doc_hash")
                if saved and loaded:
                    out["sync"] = "synced" if saved == loaded else "stale"
                return out
            out["status"] = "no_response"
            out["message"] = "behavior_director 無応答/未起動(状態が %.0f 秒以上届いていません)" % age
            return out
        if now - started < self.WARMUP_SEC:
            out["message"] = "behavior_directorの状態を待っています..."
            return out
        out["status"] = "no_response"
        out["message"] = "behavior_director 無応答/未起動(状態が届きません)"
        return out

    # --- ファイル ---

    def tree_payload(self) -> Dict[str, Any]:
        saved = self.store.read()
        payload = dict(saved)
        payload["is_default"] = not saved["exists"]
        payload["default_document"] = bt.default_document()
        if not saved["exists"]:
            payload["document"] = bt.default_document()
            payload["valid"] = True
        payload["meta"] = bt.registry_meta()
        payload["examples"] = bt.example_documents()
        payload["pin_warnings"] = self._pin_warnings(payload.get("document"))
        payload["live"] = self.live_state()
        return payload

    def validate_payload(self, body: Any) -> Tuple[int, Dict[str, Any]]:
        if not isinstance(body, dict) or not isinstance(body.get("document"), dict):
            return 400, {"ok": False, "errors": ["document(木のオブジェクト)が必要です"]}
        errors, warnings, norm = bt.validate_document(body["document"])
        out: Dict[str, Any] = {
            "ok": not errors, "errors": errors, "warnings": warnings,
            "pin_warnings": self._pin_warnings(body["document"]) if not errors else [],
        }
        if norm is not None:
            out["node_count"] = norm["node_count"]
            out["outline"] = bt.describe(norm["root"])
        return (200 if not errors else 400), out

    def save(self, body: Any) -> Tuple[int, Dict[str, Any]]:
        if not isinstance(body, dict) or not isinstance(body.get("document"), dict):
            return 400, {"ok": False, "errors": ["document(木のオブジェクト)が必要です"]}
        expected = body.get("expected_sig")
        if expected is not None and not isinstance(expected, str):
            return 400, {"ok": False, "errors": ["expected_sigは文字列にしてください"]}
        result = self.store.save(body["document"], expected_sig=expected)
        if not result["ok"]:
            status = {"invalid": 400, "conflict": 409}.get(result.get("kind"), 500)
            return status, result
        result["pin_warnings"] = self._pin_warnings(body["document"])
        result["message"] = "保存しました。behavior_directorが1秒以内に自動で読み込みます。"
        return 200, result

    def reset(self, body: Any) -> Tuple[int, Dict[str, Any]]:
        expected = body.get("expected_sig") if isinstance(body, dict) else None
        if expected is not None and not isinstance(expected, str):
            return 400, {"ok": False, "errors": ["expected_sigは文字列にしてください"]}
        result = self.store.reset(expected_sig=expected)
        if not result["ok"]:
            return {"conflict": 409}.get(result.get("kind"), 500), result
        result["message"] = (
            "既定に戻しました(従来のルール表だけの動作)。直前の木は .bak に残しています。"
            if result.get("removed") else "既に既定です(木のファイルはありません)。"
        )
        return 200, result

    def _pin_warnings(self, document: Any) -> List[str]:
        """木が指すピンが、いま読み込み中の地図に無い場合の警告(保存は妨げない。実行時は走行せず待機)。"""
        if self.pin_controller is None or self._pins_payload_fn is None or not isinstance(document, dict):
            return []
        try:
            from .map_pins import PIN_REASON_TEXT, resolve_pin

            store = self.pin_controller.store
            payload = self._pins_payload_fn(store.loaded_map(), store.list())
            out = []
            for nid, n in _pin_refs(document.get("root")):
                p = n.get("params", {})
                if n.get("type") == "set_policy" and p.get("policy") != "goto_pin":
                    continue
                if n.get("enabled", True) is False or n.get("type") == "pin_available":
                    continue
                pin, reason = resolve_pin(payload, p.get("pin_id"), p.get("pin_map"))
                if pin is None:
                    out.append(
                        f"ノード{nid}の行き先ピン「{p.get('pin_name') or p.get('pin_id')}」: "
                        f"{PIN_REASON_TEXT.get(reason, reason)}。このままでも保存できますが、"
                        "実行時はそのピンへは行かず、その場で待機します。"
                    )
            return out
        except Exception:  # noqa: BLE001 - 警告の計算失敗で保存/表示を止めない
            return []
