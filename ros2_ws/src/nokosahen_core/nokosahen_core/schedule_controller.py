"""スケジュール(時間割)のダッシュボード側のロジック(ROS非依存、publisher/subscriberは受け取って使う)。docs/schedule.md。

ダッシュボード(dashboard_common.make_dashboard_handler)が使う。ロボットを直接動かすことは無い:
- スケジュールの検証・要約・保存(~/.nokosahen_schedule.json、検証OKのものだけ)
- 「実行」: 保存済みのスケジュールを木にコンパイルして既存の木のファイル(~/.nokosahen_behavior_tree.json、source:"schedule"、
  enabled:true)へ検証つきで書き、/nokosahen/schedule/command に {"cmd":"run"} を送る。ユーザーが木のページで編集した
  木は上書きされるので、上書き前の木は世代つきで ~/.nokosahen_behavior_tree.before_schedule.<YYYYmmdd-HHMMSS>.json に
  退避する(最新5個を残す。同じ内容の退避が既にあれば新しく作らない)。
- 「停止」: 木のファイルを enabled:false にし、{"cmd":"stop"} を送る(directorが即座にその場で待機+ゴール取り消し+
  ルール表の動作へ戻る)。
- 状態: directorが配信する木の状態(/nokosahen/behavior_tree/status)の schedule 部分と、機体の時計
  (/nokosahen/robot_clock)を、見ている間だけ購読して返す。
実行するかどうかの最終判断・走行・安全層は全てdirector側。ここは木のファイルと合図を出すだけ。
"""

from __future__ import annotations

import json
import os
import re
import shutil
import tempfile
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from . import behavior_tree as bt
from . import schedule_plan as sp
from .behavior_tree_store import TreeFileStore

DEFAULT_SCHEDULE_FILE = "~/.nokosahen_schedule.json"
COMMAND_TOPIC = "/nokosahen/schedule/command"
ROBOT_CLOCK_TOPIC = "/nokosahen/robot_clock"
CLOCK_STALE_SEC = 5.0
BEFORE_SCHEDULE_INFIX = ".before_schedule."  # <木のファイル名の本体>.before_schedule.<YYYYmmdd-HHMMSS>[-N].json
BACKUP_KEEP = 5


def _atomic_write(path: str, text: str, backup: bool = False) -> None:
    directory = os.path.dirname(path) or "."
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".nokosahen_sched_", suffix=".tmp", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        if backup and os.path.exists(path):
            shutil.copy2(path, path + ".bak")
        os.replace(tmp, path)
        tmp = None
    finally:
        if tmp:
            try:
                os.remove(tmp)
            except OSError:
                pass


class ScheduleController:
    # 【2026-10-23】頭部ディスプレイの画面の切替(DisplayScheduleController)。無い構成(別ノード・テスト)では None
    display: Any = None
    # 【M8】動ける範囲の警告用(無い構成・取れない時は警告なし)。map_source: map_json() を持つもの(MapGoalController、
    # GET /api/map と同じ範囲)。boundary_source: status() を持つもの(BoundaryController、機体の元の範囲)。
    map_source: Any = None
    boundary_source: Any = None

    def __init__(self, node, schedule_file: str, behavior_tree, string_type, lazy_cls) -> None:
        """behavior_tree: BehaviorTreeController(木のファイルのパス・director状態の購読・ピン情報を共有する)。"""
        self.node = node
        self.path = os.path.expanduser(schedule_file or DEFAULT_SCHEDULE_FILE)
        self.tree = behavior_tree
        self.store: TreeFileStore = behavior_tree.store
        base, ext = os.path.splitext(self.store.path)
        self.before_base = base + BEFORE_SCHEDULE_INFIX
        self._backup_re = re.compile(re.escape(os.path.basename(self.before_base)) + r"(\d{8}-\d{6})(?:-(\d+))?\.json$")
        self._lock = threading.Lock()
        self._msg_cls = string_type
        self._pub = node.create_publisher(string_type, COMMAND_TOPIC, 10)
        self._clock: Optional[Dict[str, Any]] = None
        self._clock_rx = 0.0
        self._clock_lazy = lazy_cls(
            node, string_type, ROBOT_CLOCK_TOPIC, self._on_clock, depth=2,
            idle_timeout_sec=15.0, on_unsubscribe=self._clear_clock,
        )

    # --- 機体の時計 ---

    def _on_clock(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(data, dict):
            with self._lock:
                self._clock = data
                self._clock_rx = time.monotonic()

    def _clear_clock(self) -> None:
        with self._lock:
            self._clock = None
            self._clock_rx = 0.0

    def robot_clock(self) -> Dict[str, Any]:
        """機体が認識している時刻。届いていなければ available:false(directorの未起動・無応答)。
        server_epoch/server_iso は、このダッシュボードを動かしている機器の時刻(ずれの確認用)。"""
        self._clock_lazy.touch()
        now = time.time()
        with self._lock:
            data, rx = self._clock, self._clock_rx
        out: Dict[str, Any] = {"available": False, "server_epoch": now,
                               "server_iso": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(now))}
        if data is not None and rx:
            age = time.monotonic() - rx
            out["age_s"] = round(age, 1)
            if age <= CLOCK_STALE_SEC:
                out.update(data)
                out["available"] = True
                return out
            out["message"] = "behavior_director 無応答(機体の時刻が %.0f 秒以上届いていません)" % age
            return out
        out["message"] = "機体の時刻を待っています(behavior_director 未起動/無応答の可能性)"
        return out

    # --- 保存済みスケジュール ---

    def _read_saved(self) -> Tuple[Optional[dict], Optional[str]]:
        """(正規化したスケジュール or None, 読めない理由 or None)。ファイルが無ければ (None, None)。"""
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                text = f.read()
        except FileNotFoundError:
            return None, None
        except (OSError, UnicodeDecodeError) as exc:
            return None, f"スケジュールのファイルを読めません: {exc}"
        try:
            data = sp.parse_schedule_text(text)
        except sp.ScheduleError as exc:
            return None, str(exc)
        errors, _w, norm = sp.validate_schedule(data)
        if errors:
            return None, "保存済みのスケジュールが不正です: " + errors[0]
        return norm, None

    # --- 状態 ---

    def state(self) -> Dict[str, Any]:
        """画面用の実行状態。directorの木の状態(schedule部分)に、応答の有無・保存済みの木との一致を足したもの。"""
        live = self.tree.live_state()
        sched = {}
        director = live.get("state") if live.get("status") == "ok" else None
        if isinstance(director, dict) and isinstance(director.get("schedule"), dict):
            sched = dict(director["schedule"])
        running = bool(sched.get("running"))
        out: Dict[str, Any] = {
            "status": live.get("status"),  # ok / waiting / no_response
            "message": live.get("message", ""),
            "running": running,
            "armed": bool(sched.get("armed")),
            "startup_pending": bool(sched.get("startup_pending")),  # run_on_startup の自動実行の条件待ち(起動後120秒・時計)
            "active_entry_id": sched.get("active_entry_id"),
            "active_until": sched.get("active_until"),
            "next_entry_id": sched.get("next_entry_id"),
            "next_entry_at": sched.get("next_entry_at"),
            "clock_suspect": bool(sched.get("clock_suspect")),
            "entry_count": sched.get("entry_count", 0),
            "boundary_override": sched.get("boundary_override"),
            "tree_source": director.get("source") if isinstance(director, dict) else None,
            "mode": director.get("mode") if isinstance(director, dict) else None,
            "safety_hold": bool(director.get("safety_hold")) if isinstance(director, dict) else False,
            "sync": live.get("sync"),
            "path_mismatch": bool(live.get("path_mismatch")),
            "display_request": sched.get("display_request"),
        }
        # 頭部ディスプレイの画面: 要求(director)と、実際の切替の状態(idle/ok/pending/error、保留の理由、届かない等)
        disp = None
        if self.display is not None:
            try:
                disp = self.display.status()
            except Exception:  # noqa: BLE001 - 表示用。取れなくても状態の返却を止めない
                disp = None
        out["display"] = disp
        return out

    def payload(self) -> Dict[str, Any]:
        """GET /api/schedule"""
        norm, problem = self._read_saved()
        state = self.state()
        out = {
            "schedule": norm,
            "running": state["running"],
            "state": state,
            "robot_clock": self.robot_clock(),
        }
        if problem:
            out["schedule_error"] = problem
        if norm is not None:
            windows = sp.resolve_windows(norm["entries"])
            out["summary"] = [sp.summarize_entry(e, w) for e, w in zip(norm["entries"], windows)]
        return out

    # --- プレビュー・保存 ---

    @staticmethod
    def _schedule_from_body(body: Any) -> Any:
        if isinstance(body, dict) and "schedule" in body:
            return body["schedule"]
        return body

    def _pin_warnings(self, norm: dict) -> List[str]:
        """行き先のピンが、いま読み込み中の地図に無い場合の警告(保存は妨げない。実行時はそのピンへは行かず待機)。"""
        tree = self.tree
        if tree.pin_controller is None or tree._pins_payload_fn is None:  # noqa: SLF001
            return []
        try:
            from .map_pins import PIN_REASON_TEXT, resolve_pin

            store = tree.pin_controller.store
            payload = tree._pins_payload_fn(store.loaded_map(), store.list())  # noqa: SLF001
            out = []
            for i, e in enumerate(norm["entries"]):
                d = e["destination"]
                if d["type"] != "pin":
                    continue
                pin, reason = resolve_pin(payload, d["pin_id"], None)
                if pin is None:
                    out.append(
                        f"項目{i + 1}(id={e['id']})の行き先ピン「{d.get('pin_name') or d['pin_id']}」: "
                        f"{PIN_REASON_TEXT.get(reason, reason)}。このままでも保存できますが、"
                        "実行時はそのピンへは行かず、その場で待機します。")
            return out
        except Exception:  # noqa: BLE001 - 警告の計算失敗で保存・表示を止めない
            return []

    def _map_extent(self) -> Optional[Dict[str, float]]:
        try:
            m = self.map_source.map_json() if self.map_source is not None else None
            if not m:
                return None
            x0, y0, res = float(m["origin_x"]), float(m["origin_y"]), float(m["resolution"])
            return {"x_min": x0, "y_min": y0, "x_max": x0 + float(m["width"]) * res, "y_max": y0 + float(m["height"]) * res}
        except Exception:  # noqa: BLE001 - 警告用。取れなければ警告なし
            return None

    def _original_boundary(self) -> Optional[Dict[str, float]]:
        try:
            s = self.boundary_source.status() if self.boundary_source is not None else None
            if not s or not s.get("enabled"):
                return None  # 制限なし=どの範囲も「元より広い」ことはない
            return {"x_min": float(s["min_x"]), "y_min": float(s["min_y"]), "x_max": float(s["max_x"]), "y_max": float(s["max_y"])}
        except Exception:  # noqa: BLE001
            return None

    def _rect_warnings(self, norm: dict) -> List[str]:
        """動ける範囲(boundary.set)の、地図の外へのはみ出し・元の範囲より広い場合の警告。"""
        if not any((e.get("boundary") or {}).get("mode") == "set" for e in norm["entries"]):
            return []
        try:
            return sp.rect_warnings(norm, self._map_extent(), self._original_boundary())
        except Exception:  # noqa: BLE001
            return []

    def preview(self, body: Any) -> Tuple[int, Dict[str, Any]]:
        """POST /api/schedule/preview。検証NGでも200で ok:false を返す(画面で理由を出す用)。"""
        result = sp.preview(self._schedule_from_body(body))
        if result["ok"]:
            errors, _w, norm = sp.validate_schedule(self._schedule_from_body(body))
            if norm is not None:
                result["warnings"] = list(result["warnings"]) + self._pin_warnings(norm) + self._rect_warnings(norm)
        return 200, result

    def save(self, body: Any) -> Tuple[int, Dict[str, Any]]:
        """POST /api/schedule。検証OKのものだけ保存する(原子的に置き換え、直前のファイルは .bak に残す)。"""
        errors, warnings, norm = sp.validate_schedule(self._schedule_from_body(body))
        if errors or norm is None:
            return 400, {"ok": False, "summary": [], "errors": errors, "warnings": warnings}
        warnings = list(warnings) + self._pin_warnings(norm) + self._rect_warnings(norm)
        with self._lock:
            try:
                _atomic_write(self.path, sp.document_text(norm), backup=True)
            except OSError as exc:
                return 500, {"ok": False, "summary": [], "warnings": warnings,
                             "errors": [f"保存できませんでした(元のファイルは変更されていません): {exc}"]}
        summary = sp.summarize(norm)
        running = self.state()["running"]
        message = "スケジュールを保存しました。"
        if running:
            message += "実行中のスケジュールには反映されません。反映するには「停止」してから、もう一度「実行」してください。"
        return 200, {"ok": True, "summary": summary, "errors": [], "warnings": warnings, "message": message}

    # --- 実行・停止 ---

    def _send(self, cmd: str) -> None:
        msg = self._msg_cls()
        msg.data = json.dumps({"cmd": cmd})
        self._pub.publish(msg)

    def list_backups(self) -> List[str]:
        """退避した木のパス(新しい順)。"""
        directory = os.path.dirname(self.before_base) or "."
        found = []
        try:
            names = os.listdir(directory)
        except OSError:
            return []
        for name in names:
            m = self._backup_re.match(name)
            if m:
                found.append(((m.group(1), int(m.group(2) or 0)), os.path.join(directory, name)))
        found.sort(reverse=True)
        return [p for _k, p in found]

    def _prune_backups(self) -> None:
        for old in self.list_backups()[BACKUP_KEEP:]:
            try:
                os.remove(old)
            except OSError:
                pass

    def _backup_current_tree(self) -> Tuple[Optional[str], Optional[str]]:
        """schedule由来でない木(ユーザーが木のページで編集したもの)があれば、上書き前に退避する。
        (退避した場所 or None, エラー or None)。すでにschedule由来の木なら退避しない(元のユーザーの木の退避を消さないため)。
        退避は世代つき(最新 BACKUP_KEEP 個を残す)。同じ内容の退避が既にあればそれを返す。"""
        path = self.store.path
        if not os.path.exists(path):
            return None, None
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.loads(f.read())
            if isinstance(data, dict) and data.get("source") == "schedule":
                return None, None
        except (OSError, ValueError, UnicodeDecodeError):
            pass  # 読めない/壊れた木も、念のためそのまま退避する
        try:
            with open(path, "rb") as f:
                current = f.read()
            for existing in self.list_backups():  # 同じ内容の退避済みがあれば、重複して作らない
                try:
                    with open(existing, "rb") as f:
                        if f.read() == current:
                            return existing, None
                except OSError:
                    continue
            stamp = time.strftime("%Y%m%d-%H%M%S")
            # 同じ秒に2回(別内容で)退避した場合は -N を付ける。N は同じ秒の既存の最大+1
            # (古い方が先に消えて欠番になっても、新しい退避が「最古」に並んで消されることがないように)
            same = [os.path.basename(p) for p in self.list_backups() if f".{stamp}" in os.path.basename(p)]
            seqs = [int(self._backup_re.match(n).group(2) or 1) for n in same]
            dest = f"{self.before_base}{stamp}.json" if not seqs else f"{self.before_base}{stamp}-{max(seqs) + 1}.json"
            shutil.copy2(path, dest)
        except OSError as exc:
            return None, f"上書き前の木を退避できませんでした({exc})。実行を中止しました(木のファイルは変更していません)"
        self._prune_backups()
        return dest, None

    def run(self) -> Tuple[int, Dict[str, Any]]:
        """POST /api/schedule/run。保存済みのスケジュールをコンパイルして木のファイルへ書き、実行の合図を出す。"""
        norm, problem = self._read_saved()
        if norm is None:
            return 400, {"ok": False, "message": problem or "保存されたスケジュールがありません。先に保存してください。"}
        tree_doc = sp.compile_schedule(norm)
        with self._lock:
            backup, err = self._backup_current_tree()
            if err:
                return 500, {"ok": False, "message": err}
            result = self.store.save(tree_doc)
            if not result["ok"]:
                return (400 if result.get("kind") == "invalid" else 500), {
                    "ok": False, "message": "木へ変換できませんでした: " + " / ".join(result.get("errors", []))}
        self._send("run")
        state = self.state()
        parts = [f"スケジュール「{norm['name']}」を実行します。"]
        if backup:
            parts.append(f"木のページで編集した木は上書きされました(上書き前の木は {backup} に退避しました)。")
        else:
            parts.append("木のファイルを上書きしました(木のページで編集した木は、スケジュールの実行で置き換わります)。")
        if state["status"] != "ok":
            parts.append("behavior_directorから応答がありません。起動している場合は数秒後に状態を確認してください。"
                         "directorが再起動すると、実行はいったん止まります(run_on_startupがtrueのときだけ自動で再開)。")
        return 200, {"ok": True, "message": "".join(parts), "backup": backup,
                     "director_ok": state["status"] == "ok", "warnings": result.get("warnings", [])}

    def stop(self) -> Tuple[int, Dict[str, Any]]:
        """POST /api/schedule/stop。木のファイルを enabled:false にして、停止の合図を出す(常に合図は出す)。"""
        changed = False
        with self._lock:
            saved = self.store.read()
            doc = saved.get("document") if saved.get("valid") else None
            if isinstance(doc, dict) and doc.get("source") == "schedule" and doc.get("enabled", True):
                doc = dict(doc)
                doc["enabled"] = False
                result = self.store.save(doc)
                if not result["ok"]:
                    # 木のファイルを書けなくても、停止の合図は必ず出す(directorは合図で即座に止まる)
                    self._send("stop")
                    return 500, {"ok": False, "message": "木のファイルを無効にできませんでしたが、停止の合図は送りました: "
                                 + " / ".join(result.get("errors", []))}
                changed = True
        self._send("stop")
        msg = "スケジュールを停止しました。ロボットはその場で待機し、移動中のゴールを取り消して、時間帯・状況ルール表の動作へ戻ります。"
        if not changed:
            msg = "スケジュールは実行中ではありませんでした(停止の合図は送りました)。"
        elif self.list_backups():
            msg += f"スケジュール実行前の木は {self.list_backups()[0]} にあります(必要なら木のページで読み込み直してください)。"
        return 200, {"ok": True, "message": msg}
