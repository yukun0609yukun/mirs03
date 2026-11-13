"""スケジュールによる頭部ディスプレイの画面の切替(ダッシュボード側、ROS非依存で動くロジック)。docs/schedule.md。

【2026-10-23】木のノードset_displayは表示モードの「要求」を出すだけ。directorが /nokosahen/display_request へ配信し、
この DisplayScheduleController が受けて、要求が変わったときだけ、既存の切替(SurveyAdminProxy.set_mode =
POST /api/survey/mode と同じ経路)を呼ぶ。ラズパイ側のコードは変えない(既存のAPIを呼ぶだけ)。

安全の扱い
- アンケート回答中(ephemeral_overrides.survey_answering() と同じ判定=kiosk_adminの/stateのactiveまたは
  session_state=="answering")は、別のモードへ切り替えない(切り替えると回答が消える)。終わるまで保留(pending)し、
  終わったら切り替える。回答中かどうか判定できない(ラズパイに届かない・応答が不正)ときも切り替えない。
- 切替前のモード(スケジュールを始める前のモード)を baseline として覚え、要求が keep になった(項目が終わった・
  displayを指定しない項目・スケジュール停止)ときは baseline へ戻す(回答中なら保留)。
  baselineは小さなファイルにも残し、ダッシュボードが再起動しても元のモードへ戻せるようにする。
- 要求が古い(DISPLAY_REQUEST_TTL_SEC=20秒以上届かない=directorが止まった)ときは keep 扱い(元のモードへ戻す)。
- 繰り返しの再送(5秒おき)では、すでにそのモードなら何もしない。ユーザーが管理画面で手動で切り替えた場合も、要求が変わるまでは
  上書きし直さない(要求が変わったとき、または保留・失敗の再試行のときだけ動く)。
- ラズパイに届かない・切替に失敗したときは、ログと状態(error)に出すだけ。ロボットの走行には一切影響しない
  (このクラスは走行・director・Nav2に触れない)。
- 権限: スケジュールの実行は schedule.run(operatorに付与可)の時点で確認済み。ここでの切替は schedule.run の範囲の
  副作用として行い、表示切替の権限 display.survey_mode の台帳(一時的な変更の記録)には載せない。
"""

from __future__ import annotations

import json
import os
import tempfile
import threading
import time
from typing import Any, Callable, Dict, Optional

from . import schedule_plan as sp
from .ephemeral_overrides import survey_answering

DISPLAY_REQUEST_TOPIC = sp.DISPLAY_REQUEST_TOPIC
TICK_SEC = 2.0
RETRY_AFTER_FAILURE_SEC = 10.0   # set_modeが失敗したときの再試行間隔(失敗は切替の呼び出しに限る)
DEFAULT_BASELINE_FILE = "~/.nokosahen_schedule_display_baseline.json"
VALID_MODES = ("customer", "survey", "ocrtest", "inventory", "face")   # inventory(残数入力)・face(顔)はスケジュールからは指定しないが、元のモードとして覚える

STATE_IDLE = "idle"        # スケジュールによる切替をしていない
STATE_OK = "ok"            # 要求どおりのモード
STATE_PENDING = "pending"  # 切り替えたいが保留中(回答中・判定不能)
STATE_ERROR = "error"      # ラズパイに届かない/切替に失敗(走行には影響しない)


class _Snapshot:
    """survey_answering() に、すでに取得した/stateの応答を渡すための替え玉(同じ判定を1回の問い合わせで使うため)。"""

    def __init__(self, res: Any) -> None:
        self._res = res

    def get_state(self, fresh: bool = False) -> Any:  # noqa: ARG002
        return self._res


class DisplayScheduleController:
    def __init__(
        self,
        survey_admin: Any,
        node: Any = None,
        string_type: Any = None,
        baseline_file: Optional[str] = None,
        clock: Callable[[], float] = time.monotonic,
        ttl_sec: float = sp.DISPLAY_REQUEST_TTL_SEC,
    ) -> None:
        self.admin = survey_admin
        self.node = node
        self._clock = clock
        self._ttl = float(ttl_sec)
        self._lock = threading.Lock()
        self._busy = threading.Lock()
        self._started = clock()
        self._req: Optional[Dict[str, Any]] = None       # {"mode","entry_id","rx"}
        self._handled: Optional[str] = None               # 要求が満たされたモード(再送で何度も切り替えないため)
        self._retry_at = 0.0
        self._last_log: Optional[str] = None
        self._status: Dict[str, Any] = {"state": STATE_IDLE, "requested": None, "current": None, "pending_reason": "",
                                        "error": "", "entry_id": None, "restoring": False}
        self._baseline_path = os.path.expanduser(baseline_file) if baseline_file else None
        self._baseline: Optional[str] = self._load_baseline()
        if node is not None and string_type is not None:
            node.create_subscription(string_type, DISPLAY_REQUEST_TOPIC, self._on_msg, 10)
            node.create_timer(TICK_SEC, self._timer_cb)

    # --- baseline(スケジュールを始める前のモード)の保存 ---

    def _load_baseline(self) -> Optional[str]:
        if not self._baseline_path:
            return None
        try:
            with open(self._baseline_path, "r", encoding="utf-8") as f:
                v = json.loads(f.read()).get("baseline")
            return v if v in VALID_MODES else None
        except (OSError, ValueError, AttributeError):
            return None

    def _save_baseline(self, value: Optional[str]) -> None:
        self._baseline = value
        path = self._baseline_path
        if not path:
            return
        try:
            if value is None:
                try:
                    os.remove(path)
                except FileNotFoundError:
                    pass
                return
            directory = os.path.dirname(path) or "."
            os.makedirs(directory, exist_ok=True)
            fd, tmp = tempfile.mkstemp(prefix=".nokosahen_dispbase_", suffix=".tmp", dir=directory)
            with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as f:
                f.write(json.dumps({"baseline": value}) + "\n")
            os.replace(tmp, path)
        except OSError as exc:  # 保存できなくてもメモリ上の値で動く(再起動後に戻せないだけ)
            self._log("warn", f"頭部ディスプレイの画面の元のモードを保存できませんでした: {exc}")

    # --- 要求の受信 ---

    def _on_msg(self, msg: Any) -> None:
        self.on_request_text(getattr(msg, "data", None))

    def on_request_text(self, text: Any) -> None:
        req = sp.parse_display_request(text)
        if req is None:
            return  # 不正なメッセージは無視(状態は変えない)
        with self._lock:
            self._req = {"mode": req["mode"], "entry_id": req["entry_id"], "rx": self._clock()}

    # --- 周期処理 ---

    def _timer_cb(self) -> None:
        """タイマー(ROSのコールバック)から呼ぶ。ラズパイへの問い合わせは数秒かかり得るので別スレッドで行い、
        ダッシュボードの他の処理を止めない。前回の処理が終わっていなければ何もしない。"""
        if not self._busy.acquire(blocking=False):
            return
        def _run() -> None:
            try:
                self.tick()
            except Exception as exc:  # noqa: BLE001 - 表示の切替の不具合でダッシュボードを止めない
                self._set_status(STATE_ERROR, error=f"内部エラー: {exc!r}")
                self._log("error", f"頭部ディスプレイの画面の切替で内部エラー: {exc!r}")
            finally:
                self._busy.release()
        threading.Thread(target=_run, name="display-schedule", daemon=True).start()

    def _target(self, now: float):
        """(切り替えたいモード or None, 元のモードへ戻す途中か, 要求の項目id)。Noneは何もしない。"""
        with self._lock:
            req = dict(self._req) if self._req else None
        rx = req["rx"] if req else self._started
        fresh = (now - rx) <= self._ttl
        if req is not None and fresh and req["mode"] != "keep":
            return req["mode"], False, req["entry_id"]
        # keep(要求なし)・古い要求・起動直後でまだ要求が届かない: 元のモードがあれば戻す
        if req is not None and req["mode"] == "keep":
            waited = True   # directorが明示的にkeepを送った: すぐ戻す
        else:
            waited = not fresh
        if self._baseline is not None and waited:
            return self._baseline, True, req["entry_id"] if req else None
        return None, False, req["entry_id"] if req else None

    def tick(self) -> Dict[str, Any]:
        """要求と現在のモードを突き合わせ、必要なときだけ切り替える。例外は呼び出し側(_timer_cb)で受ける。状態を返す。"""
        now = self._clock()
        target, restoring, entry_id = self._target(now)
        if target is None:
            self._handled = None
            if self._baseline is None:
                self._set_status(STATE_IDLE)
            return self.status()
        if not restoring and target == self._handled:
            self._set_status(STATE_OK, requested=target, entry_id=entry_id)
            return self.status()   # すでに満たした要求の再送: 何もしない(ラズパイへ問い合わせもしない)
        if now < self._retry_at:
            return self.status()
        # --- ここからラズパイに問い合わせる ---
        res = self.admin.get_state(fresh=True) if getattr(self.admin, "configured", True) else None
        if not isinstance(res, dict) or not res.get("ok"):
            why = (res or {}).get("message") if isinstance(res, dict) else "頭部ディスプレイの画面の管理が未設定です"
            self._set_status(STATE_ERROR, requested=target, entry_id=entry_id, restoring=restoring,
                             error=f"ラズパイに届かないため切り替えません: {why}",
                             pending_reason="ラズパイの状態を確認できないため(切り替えません)")
            self._log("warn", f"頭部ディスプレイの画面の切替を見送り(届かない/判定不能): {why}")
            return self.status()
        state = res.get("state") if isinstance(res.get("state"), dict) else {}
        current = state.get("mode") if state.get("mode") in VALID_MODES else None
        if current == target:
            self._arrived(target, restoring, entry_id)
            return self.status()
        answering = survey_answering(_Snapshot(res))
        if answering is not False:
            reason = "アンケート回答中のため(回答が終わってから切り替えます)" if answering else \
                "アンケート回答中かどうか判定できないため(切り替えません)"
            self._set_status(STATE_PENDING, requested=target, current=current, entry_id=entry_id,
                             restoring=restoring, pending_reason=reason)
            self._log("info", f"頭部ディスプレイの画面の切替を保留({target}): {reason}")
            return self.status()
        newly_based = False
        if not restoring and self._baseline is None:
            self._save_baseline(current or "customer")   # スケジュールを始める前のモード
            newly_based = True
        status, payload = self.admin.set_mode({"mode": target})
        ok = status == 200 and isinstance(payload, dict) and bool(payload.get("ok"))
        if not ok:
            if newly_based:
                self._save_baseline(None)
            msg = (payload or {}).get("message") if isinstance(payload, dict) else ""
            self._retry_at = now + RETRY_AFTER_FAILURE_SEC
            self._set_status(STATE_ERROR, requested=target, current=current, entry_id=entry_id, restoring=restoring,
                             error=f"頭部ディスプレイの画面を{target}へ切り替えられませんでした: {msg or status}")
            self._log("error", f"頭部ディスプレイの画面の切替に失敗({target}): {msg or status}")
            return self.status()
        self._arrived(target, restoring, entry_id, switched=True)
        return self.status()

    def _arrived(self, target: str, restoring: bool, entry_id: Optional[str], switched: bool = False) -> None:
        self._retry_at = 0.0
        if restoring:
            self._save_baseline(None)
            self._handled = None
            self._set_status(STATE_IDLE, current=target)
            self._log("info", f"頭部ディスプレイの画面を元のモード({target})へ戻しました" if switched
                      else f"頭部ディスプレイの画面はすでに元のモード({target})です")
            return
        self._handled = target
        self._set_status(STATE_OK, requested=target, current=target, entry_id=entry_id)
        if switched:
            self._log("info", f"スケジュールにより頭部ディスプレイの画面を{target}へ切り替えました")

    # --- 状態(画面用) ---

    def _set_status(self, state: str, requested: Optional[str] = None, current: Optional[str] = None,
                    entry_id: Optional[str] = None, restoring: bool = False, pending_reason: str = "",
                    error: str = "") -> None:
        with self._lock:
            prev_current = self._status.get("current")
            self._status = {"state": state, "requested": requested,
                            "current": current if current is not None else prev_current,
                            "pending_reason": pending_reason, "error": error,
                            "entry_id": entry_id, "restoring": restoring}

    def status(self) -> Dict[str, Any]:
        """画面用の状態。state: idle / ok / pending / error。pendingのとき pending_reason に理由。"""
        with self._lock:
            st = dict(self._status)
            req = dict(self._req) if self._req else None
        st["baseline"] = self._baseline
        st["request_age_s"] = round(self._clock() - req["rx"], 1) if req else None
        st["pending"] = st["state"] == STATE_PENDING
        return st

    def _log(self, level: str, text: str) -> None:
        """同じ内容を続けて出さない(再試行のたびにログが増えないように)。"""
        key = f"{level}:{text}"
        if key == self._last_log:
            return
        self._last_log = key
        node = self.node
        logger = node.get_logger() if node is not None and hasattr(node, "get_logger") else None
        fn = getattr(logger, level, None) if logger is not None else None
        if fn is not None:
            try:
                fn(text)
            except Exception:  # noqa: BLE001
                pass
