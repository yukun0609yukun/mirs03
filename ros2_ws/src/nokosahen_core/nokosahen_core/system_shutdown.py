"""管理画面の「プログラム終了(全停止)」(純Python、ROS非依存)。docs/system_shutdown.md

【2026-10-13追加】1台目のロボット関連プログラム(ROS一式・管理画面自身・OCR・Coral)を全部止める。
管理画面の「OSも停止して電源を落とす」(既定ON)にチェックがあると、そのあとラズパイ・2台目・1台目のOSも停止して電源を落とす(include_poweroff。2026-10-19〜。再起動は、一式の電源を入れ直す=自動起動)。
APIで include_poweroff を省いた呼び出しは false(OSは動いたまま。2026-10-21、省いただけで電源が落ちないように)。false なら、OSは動いたまま(Jetsonの電源・Wi-Fiテザリング・SSHは生きている。再起動は人がSSHで行う)。

流れ(POST /api/system/shutdown。権限はadmin専用 = roles.API_RULES):
1. 入力検証(許可キーだけ・型・confirm完全一致) → 400
2. 機能の有効/無効、二重実行、停止スクリプトの有無、未保存の地図 → 403/409/503(**何も止めずに**理由を返す)
3. **モーターを確実に止める**(ソフト緊急停止を他ノードが落ちる前に必ず発行・目的地キャンセル・探索停止)。失敗したら中止(500)
4. 停止スクリプト(scripts/shutdown_all.sh)を**切り離した別プロセス**(start_new_session=True)で起動し、応答を返す。
   スクリプトは数秒待ってから 監視ループ → ROS一式 → OCR/Coral → (2台目の音声) → 管理画面自身 の順で止める。

**ユーザー入力は一切コマンドに混ぜない**: 起動するのは固定の引数リスト(シェルを通さない)。スクリプトのパスは設定値。
オプション(OCR/音声)は固定のフラグ文字列(--no-ocr / --voice)に変換されるだけ。
"""
from __future__ import annotations

import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

CONFIRM_TEXT = "シャットダウン"  # 画面で手打ちさせる確認文字列(完全一致。画面のUI定数と同じ)
ALLOWED_KEYS = ("confirm", "include_voice", "include_ocr", "discard_map", "include_poweroff")
DEFAULT_SCRIPT = "~/mirs2603_nokosahen/scripts/shutdown_all.sh"
DEFAULT_LOG = "~/.nokosahen_shutdown.log"
DEFAULT_DELAY_SEC = 3

# 切り離して起動するための固定の設定(テストで検証する)。シェルを通さない・親が死んでも継続する。
DETACH_KWARGS: Dict[str, Any] = {
    "stdin": subprocess.DEVNULL,
    "stdout": subprocess.DEVNULL,
    "stderr": subprocess.DEVNULL,
    "start_new_session": True,
    "close_fds": True,
    "shell": False,
}


class ShutdownError(Exception):
    """HTTPで返すエラー(status, 機械向けコード, 日本語の理由)。"""

    def __init__(self, status: int, code: str, message: str, **extra: Any) -> None:
        super().__init__(message)
        self.status = status
        self.code = code
        self.message = message
        self.extra = extra

    def body(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {"ok": False, "error": self.code, "message": self.message}
        out.update(self.extra)
        return out


def parse_request(data: Any) -> Dict[str, bool]:
    """本文を検証して {include_voice, include_ocr, discard_map, include_poweroff} を返す。不正は400。

    include_poweroff(API既定 False=明示したときだけ。管理画面は、チェック(既定ON)の値を毎回明示して送る。2026-10-21): プログラムを止めたあと、ラズパイ・2台目・1台目のOSも停止して電源を落とす
    (電源を抜く前にOSを止める。sudoersで shutdown だけを許可してある前提)。falseならOSは動いたまま(従来の動作)。

    許可キー以外・boolでない値(文字列"true"や数値は不可)・confirmの不一致は全て拒否する。
    """
    if not isinstance(data, dict):
        raise ShutdownError(400, "bad_request", "本文はJSONオブジェクトで送ってください")
    unknown = sorted(str(k) for k in data if k not in ALLOWED_KEYS)
    if unknown:
        raise ShutdownError(400, "bad_request", "許可されていない項目があります: " + ", ".join(unknown))
    confirm = data.get("confirm")
    if not isinstance(confirm, str):
        raise ShutdownError(400, "bad_request", "confirm を文字列で送ってください")
    opts: Dict[str, bool] = {}
    for key, default in (("include_voice", False), ("include_ocr", True), ("discard_map", False), ("include_poweroff", False)):
        val = data.get(key, default)
        if not isinstance(val, bool):
            raise ShutdownError(400, "bad_request", f"{key} は true/false で送ってください")
        opts[key] = val
    if confirm != CONFIRM_TEXT:  # 完全一致(前後の空白・全角半角違いも不可)
        raise ShutdownError(400, "confirm_mismatch", f"確認の文字列が違います。「{CONFIRM_TEXT}」と入力してください")
    return opts


def plan_steps(opts: Dict[str, bool]) -> List[str]:
    """停止の順序(応答とログに出す日本語の一覧。スクリプトの順序と同じ)。"""
    steps = ["モーター停止(ソフト緊急停止・目的地キャンセル)", "監視ループ(自動再起動)を止める", "ロボット(ROS一式)を止める"]
    if opts.get("include_ocr"):
        steps.append("タブレットOCRとCoralを止める")
    if opts.get("include_voice"):
        steps.append("2台目の音声(voice_bridge/BGM)を止める")
    steps.append("管理画面自身を止める")
    if opts.get("include_poweroff"):
        steps.append("ラズパイ・2台目・1台目のOSを停止して電源を落とす")
    return steps


def build_command(script: str, opts: Dict[str, bool], delay_sec: int) -> List[str]:
    """固定の引数リスト。可変なのは「設定値のスクリプトパス」と「整数の待ち秒」と「固定フラグの有無」だけ。"""
    argv = ["bash", script, "--delay", str(int(delay_sec))]
    if not opts.get("include_ocr"):
        argv.append("--no-ocr")
    if opts.get("include_voice"):
        argv.append("--voice")
    if opts.get("include_poweroff"):
        argv.append("--poweroff")
    return argv


def build_motor_stopper(teleop: Any, map_goal: Any = None, frontier_explorer: Any = None,
                        repeats: int = 3, interval_sec: float = 0.1) -> Callable[[], None]:
    """モーターを確実に止める処理(既存の仕組みの再利用)。**他のノードを止める前に必ず呼ぶ**。

    ソフト緊急停止(/nokosahen/soft_estop=true と /cmd_vel のゼロ速度。handle_soft_estopが両方出す)を数回、
    目的地のキャンセル(/nokosahen/cancel_goal。Nav2のゴールも取り消す)、探索の停止。
    緊急停止の発行に失敗したら例外を投げる(呼び出し側が停止処理を中止する)。
    """
    def stop() -> None:
        for _ in range(max(1, repeats)):
            teleop.handle_soft_estop(True)
            time.sleep(interval_sec)
        for fn in (getattr(map_goal, "cancel", None),
                   (lambda: frontier_explorer.set_enabled(False)) if frontier_explorer is not None else None):
            if fn is not None:
                try:
                    fn()
                except Exception:  # noqa: BLE001 - 緊急停止が出ていれば、目的地/探索の取り消し失敗では止めない
                    pass
    return stop


def build_map_state(map_lifecycle: Any, mapping: Any) -> Callable[[], Dict[str, Any]]:
    """地図の保存状況。「地図作成中(/nokosahen/map_building_active=true と同じ)で、作成開始後に一度も保存していない」なら未保存。"""
    def state() -> Dict[str, Any]:
        if map_lifecycle is None:
            return {"mode": "unknown", "unsaved": False, "reason": ""}
        st = map_lifecycle.status() or {}
        mode = st.get("mode", "idle")
        if mode != "mapping":
            # 【2026-10-21】管理画面だけが再起動されて、状態がidleに戻っているのに、地図作成のプロセスが外で動き続けている場合は、
            # 保存の状況を確認できないので、未保存として扱う(以前は、警告なしに、未保存の地図が失われていた)。
            ext = getattr(map_lifecycle, "external_mapping_running", None)
            try:
                external = bool(ext()) if callable(ext) else False
            except Exception:  # noqa: BLE001 - 確認に失敗しても、従来どおり(動いていない扱い)
                external = False
            if external:
                return {"mode": "mapping_external", "unsaved": True,
                        "reason": "管理画面の外(または再起動前)で地図作成が動いています。保存の状況を確認できないため、未保存として扱います。"
                                  "先に地図を保存するか、地図を捨ててよければ「未保存の地図を捨てて終了」(管理者)を選んでください"}
            return {"mode": mode, "unsaved": False, "reason": ""}
        ms = (mapping.status() if mapping is not None else {}) or {}
        if ms.get("busy"):
            return {"mode": mode, "unsaved": True, "reason": "地図の保存処理中です。終わるまで待ってから実行してください。"}
        last = ms.get("last_result") or {}
        try:
            saved = bool(last.get("ok")) and float(last.get("time", 0)) >= float(st.get("started_at", 0) or 0)
        except (TypeError, ValueError):
            saved = False
        if saved:
            return {"mode": mode, "unsaved": False, "reason": ""}
        return {"mode": mode, "unsaved": True, "reason": "地図が未保存です。先に保存してください"}
    return state


def build_nav_state(map_goal: Any) -> Callable[[], bool]:
    def active() -> bool:
        try:
            return bool(map_goal.goal_status().get("active")) if map_goal is not None else False
        except Exception:  # noqa: BLE001
            return False
    return active


class SystemShutdownController:
    """停止要求の受付(検証・事前チェック・モーター停止・切り離し起動)。実際の停止はスクリプトが行う。"""

    def __init__(
        self,
        script_path: str = DEFAULT_SCRIPT,
        enabled: bool = True,
        delay_sec: int = DEFAULT_DELAY_SEC,
        log_path: str = DEFAULT_LOG,
        stop_motors: Optional[Callable[[], None]] = None,
        map_state: Optional[Callable[[], Dict[str, Any]]] = None,
        nav_active: Optional[Callable[[], bool]] = None,
        launcher: Optional[Callable[[List[str]], Any]] = None,
    ) -> None:
        self.script_path = str(Path(script_path).expanduser())
        self.enabled = bool(enabled)
        self.delay_sec = max(2, min(30, int(delay_sec)))
        self.log_path = Path(log_path).expanduser()
        self._stop_motors = stop_motors
        self._map_state = map_state
        self._nav_active = nav_active
        self._launcher = launcher or self._spawn_detached
        self._lock = threading.Lock()
        self._in_progress = False
        self._in_progress_since = 0.0

    @staticmethod
    def _spawn_detached(argv: List[str]) -> Any:
        return subprocess.Popen(argv, **DETACH_KWARGS)

    def script_ok(self) -> bool:
        return Path(self.script_path).is_file()

    def state(self) -> Dict[str, Any]:
        """画面用の状態(秘密・パスは含めない)。"""
        ms = self._map_state() if self._map_state else {"mode": "unknown", "unsaved": False, "reason": ""}
        return {
            "ok": True,
            "enabled": self.enabled,
            "script_ok": self.script_ok(),
            "in_progress": self._in_progress,
            "map_unsaved": bool(ms.get("unsaved")),
            "map_mode": ms.get("mode"),
            "map_reason": ms.get("reason", ""),
            "nav_active": bool(self._nav_active()) if self._nav_active else False,
            "will_stop_in_sec": self.delay_sec,
        }

    def _log(self, text: str) -> None:
        try:
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(time.strftime("%Y-%m-%d %H:%M:%S ") + text + "\n")
        except OSError:
            pass  # ログに書けなくても停止は妨げない

    def request(self, account: str, ip: str, data: Any) -> Tuple[int, Dict[str, Any]]:
        """停止要求を処理する。(HTTPステータス, 本文)。成功(200)の時だけ、停止スクリプトが起動されている。"""
        try:
            return 200, self._request(account, ip, data)
        except ShutdownError as exc:
            return exc.status, exc.body()

    def _request(self, account: str, ip: str, data: Any) -> Dict[str, Any]:
        opts = parse_request(data)
        if not self.enabled:
            raise ShutdownError(403, "disabled", "この機能は無効です(設定 system_shutdown_enabled=false)")
        with self._lock:
            # 【2026-10-21】停止処理が実際には進まなかった(スクリプトが失敗して管理画面が生き残った)場合に、永久に409にならないよう、
            # 10分を過ぎたら「停止中」の印を解除する
            if self._in_progress and time.monotonic() - self._in_progress_since > 600.0:
                self._in_progress = False
            if self._in_progress:
                raise ShutdownError(409, "already_in_progress", "すでに停止処理中です。数秒後にこの画面は開けなくなります")
            if not self.script_ok():
                raise ShutdownError(503, "script_not_found", "停止スクリプトが見つかりません(設定 shutdown_script を確認してください)")
            ms = self._map_state() if self._map_state else {"unsaved": False}
            if ms.get("unsaved") and not opts["discard_map"]:
                raise ShutdownError(409, "map_unsaved", ms.get("reason") or "地図が未保存です。先に保存してください",
                                    map_unsaved=True)
            nav_active = bool(self._nav_active()) if self._nav_active else False
            self._log(f"REQUEST account={account} ip={ip} include_ocr={opts['include_ocr']} "
                      f"include_voice={opts['include_voice']} discard_map={opts['discard_map']} nav_active={nav_active}")
            # 1) 他のノードを止める前に、必ずモーターを止める。失敗したら何も止めずに中止
            try:
                if self._stop_motors is not None:
                    self._stop_motors()
            except Exception as exc:  # noqa: BLE001
                self._log(f"ABORT モーター停止に失敗: {exc}")
                raise ShutdownError(500, "motor_stop_failed", f"モーターを止められなかったため、停止処理を中止しました: {exc}")
            self._log("OK モーター停止(ソフト緊急停止・目的地キャンセル)を発行")
            # 2) 切り離した別プロセスで停止スクリプトを起動(この後、管理画面自身も止まる)
            argv = build_command(self.script_path, opts, self.delay_sec)
            try:
                proc = self._launcher(argv)
                # スクリプトは最初に delay 秒待つので、すぐ(0.4秒以内)に非0で終了していたら起動に失敗している(パス違い・ログに書けない等)
                poll = getattr(proc, "poll", None)
                if callable(poll):
                    time.sleep(0.4)
                    rc = poll()
                    if rc is not None and rc != 0:
                        raise RuntimeError(f"停止スクリプトがすぐに終了しました(終了コード {rc})")
            except Exception as exc:  # noqa: BLE001
                self._log(f"ABORT 停止スクリプトを起動できません: {exc}")
                raise ShutdownError(500, "launch_failed", f"停止スクリプトを起動できませんでした(モーターは停止済み・緊急停止中です。もう一度実行できます): {exc}")
            self._in_progress = True
            self._in_progress_since = time.monotonic()
            self._log(f"LAUNCHED {' '.join(argv)}")
        return {"ok": True, "will_stop_in_sec": self.delay_sec, "steps": plan_steps(opts),
                "nav_was_active": nav_active, "log_file": "~/" + self.log_path.name,  # 展開済みのホームパスは返さない
                "message": "停止処理を開始しました。数秒後にこの画面は開けなくなります"}
