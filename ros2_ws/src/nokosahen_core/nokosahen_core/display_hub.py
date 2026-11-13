"""iPad表示(頭部ディスプレイ)の「いま何を表示するか」を持つ状態機械(ダッシュボードと同じプロセスの中。純Python、ROS非依存)。

docs/ipad_display.md。ラズパイ版(kiosk_admin.py + receiver.py + switch_kiosk.sh)の「モード」の概念を、
1台目(Jetson1)の中へ移したもの。iPadは固定のURL `/display` を開いたままにして、この状態を1秒おきに読み、
モードが変わると中身を切り替える。

モード(ラズパイ版の customer / survey / ocrtest に、残数入力と顔を足した5つ)
  customer  お客さん向け: 今の状態・走行中などの文言 + 本日の残数
  survey    アンケート(1画面1問、約30秒)
  inventory 残数入力と、メニューの追加・名前の変更・削除・初期食数の編集(スタッフが入力する。iPadは最初に1回ログイン済み)
  ocrtest   OCRテスト(模擬POS。カメラで撮影して数字認識を試す。本番のタブレットOCRとは別)
  face      顔(ロボットの顔のアニメーションを画面いっぱいに出す。2026-11-08追加。回答の保留・保存・再起動後の保持は他のモードと同じ。
            アンケートではないので survey_monitor のロボット停止は働かない)

切り替えの入口(どれも同じ DisplayHub.request_mode に入る)
  - ROS: /nokosahen/display_mode (std_msgs/String、"survey" か {"mode":"survey"})        -> source="ros"(回答中は保留)
  - ROS: /nokosahen/display_request (スケジュール) -> DisplayScheduleController -> LocalDisplayAdmin.set_mode
  - ダッシュボード: POST /api/survey/mode (管理者・display.survey_mode を持つoperator) -> LocalDisplayAdmin.set_mode

回答中の扱い(ラズパイ版と同じ保証)
  - session_state: idle → answering(startの通知)→ saved(回答の保存に成功)→ idle(3秒後)/ abandoned(放置・end・通知途絶え)→ idle
  - active は answering の間だけ true。survey_monitor が /display/api/state を読んで /nokosahen/survey_active にする。
  - 回答中は別のモードへ切り替えない。ROSからの要求は保留して、回答が終わってから切り替える。
    ダッシュボードの手動切替は「回答中のため切り替えられません」と断る(その人の回答を消さないため)。
  - 通知が途絶えたら放棄扱い(既定120秒)。さらに、1回の回答中の最長(既定300秒)を超えたら放棄扱い
    (ログイン済みのiPadから「回答中」を送り続けられても、ロボットを止め続けられないための上限)。
"""
from __future__ import annotations

import json
import os
import random
import tempfile
import threading
import time
import unicodedata
from typing import Any, Callable, Dict, List, Optional, Tuple

from .display_survey import SURVEY_DEF, SurveyStore, validate_condition, validate_response
from .moving_notice import TITLE as MOVING_TITLE
from .survey_admin_proxy import validate_ocr_mock_request

MODES = ("customer", "survey", "inventory", "ocrtest", "face")
MODE_LABELS = {"customer": "お客さん向け", "survey": "アンケート", "inventory": "残数入力", "ocrtest": "OCRテスト(模擬POS)", "face": "顔"}
DEFAULT_MODE = "customer"
DEFAULT_MODE_FILE = "~/.nokosahen_display_mode"
# 【2026-11-08】顔⇄アンケートの自動切替(普段は顔、人が来たときだけアンケート)。ON/OFFは再起動後も保つ(既定はOFF)
DEFAULT_AUTO_FILE = "~/.nokosahen_display_auto_face"
AUTO_PRESENT_S = 1.5      # 人が(この秒数)続けて見えたら、顔→アンケート
AUTO_ABSENT_S = 8.0       # 人が(この秒数)続けて見えなかったら、アンケート→顔(回答中・保存直後は待つ)
AUTO_MIN_STAY_S = 5.0     # 切り替えてからこの秒数は、元に戻さない(ちらつき防止)

SESSION_ABANDON_S = 120.0      # 回答中の通知(ページが5秒ごと)が途絶えてから放棄とみなすまで(ラズパイ版と同じ)
SESSION_MAX_ANSWER_S = 300.0   # 1回の回答中の最長
MAX_ANSWER_COOLDOWN_S = 20.0   # 最長で打ち切った後、この秒数は新しい回答中の通知を受け付けない(送り続けて止め続けるのを防ぐ)
SAVED_SHOW_S = 3.0             # saved を /state に出しておく秒数(ロボット側が確実に観測できるように)
ABANDONED_SHOW_S = 10.0
CLIENT_ACTIVE_S = 15.0         # この秒数以内に状態を読みに来たiPadを「接続中」とみなす

OCR_ROWS = 7
OCR_DEFAULTS = [118, 129, 353, 166, 211, 178, 103]
OCR_NOT_MODE_MESSAGE = "OCRテストモードではありません(先に頭部ディスプレイをOCRテストに切り替えてください)"

MAX_MENU_NAME = 40
MAX_REMAINING = 99999
MAX_CUSTOMER_ITEMS = 12
MENU_EDIT_OPS = ("add", "rename", "delete", "set_initial")
MENU_SYNC_WAIT_S = 5.0   # menu_init を送ってから、残数管理の状態(menu_state)に反映されるのを待つ最長の秒数(これ以内は次の編集を断る)


def normalize_mode(value: Any) -> Optional[str]:
    """"survey" / " Survey " / {"mode":"survey"} のJSON文字列 → "survey"。不正は None。"""
    if isinstance(value, dict):
        value = value.get("mode")
    if not isinstance(value, str):
        return None
    t = value.strip()
    if t.startswith("{"):
        try:
            obj = json.loads(t)
        except ValueError:
            return None
        if not isinstance(obj, dict):
            return None
        return normalize_mode(obj.get("mode"))
    t = t.lower()
    return t if t in MODES else None


def validate_mode_request(data: Any) -> Tuple[Optional[str], str]:
    """POST /api/survey/mode のボディ。許すのは {"mode": 4つのどれか} だけ(survey_admin_proxy と同じ形)。"""
    if not isinstance(data, dict) or set(data) != {"mode"}:
        return None, 'リクエストは {"mode": "customer" / "survey" / "inventory" / "ocrtest" / "face"} だけにしてください'
    mode = data["mode"]
    if not isinstance(mode, str) or mode not in MODES:
        return None, "modeは customer / survey / inventory / ocrtest / face のどれかを指定してください"
    return mode, ""


# ---------------------------------------------------------------------------
# お客さん向け表示の中身(今の状態の文言 + 残数)
# ---------------------------------------------------------------------------
# behavior_director の mode → (見出し, 補足, 色合い tone: ok / info / warn。移動中の注意だけ caution=琥珀色)
_STATUS_TEXT: Dict[str, Tuple[str, str, str]] = {
    "ESTOP_HOLD": ("停止しています", "安全のため、ロボットは止まっています", "warn"),
    "MANUAL_OVERRIDE": ("スタッフが操作しています", "しばらくお待ちください", "info"),
    "SURVEY_ACTIVE_HOLD": ("アンケート回答中です", "終わるまでロボットは止まっています", "info"),
    "NEAR_PERSON_HOLD": ("止まっています", "近くに人がいるため、安全のため停止しています", "info"),
    "OUT_OF_BOUNDS_HOLD": ("一時停止中です", "安全のため、少しお待ちください", "warn"),
    "KEEPOUT_ZONE_HOLD": ("一時停止中です", "安全のため、少しお待ちください", "warn"),
    "CAMERA_OBSTACLE_HOLD": ("一時停止中です", "前に障害物があるため、少しお待ちください", "warn"),
    "MAPPING_ACTIVE_HOLD": ("準備中です", "お店の地図を作っています", "info"),
    "MAP_NOT_LOADED_HOLD": ("準備中です", "しばらくお待ちください", "info"),
    "STOP_ANNOUNCE": ("お知らせ中です", "", "ok"),
    "APPROACH_PERSON": ("近づいています", "", "ok"),
    "APPROACH_ARRIVED": ("こんにちは！", "", "ok"),
    "WAIT_FOR_SENSOR": ("準備中です", "", "info"),
    "POLICY_NO_SCAN": ("巡回中です", "ロボットが動いています", "ok"),
}
_STATUS_MOVING = ("巡回中です", "ロボットが動いています", "ok")
_STATUS_UNKNOWN = ("準備中です", "", "info")
BEHAVIOR_STALE_S = 15.0   # behavior_director の指令がこれ以上届かないときは「準備中」


def customer_status(behavior: Optional[Dict[str, Any]], behavior_age_s: Optional[float] = None) -> Dict[str, str]:
    """お客さんに見せる「今の状態」の文言。director の指令が無い・古いときは「準備中」(嘘の「走行中」を出さない)。"""
    b = behavior if isinstance(behavior, dict) else {}
    mode = b.get("mode")
    if not isinstance(mode, str) or not mode or (behavior_age_s is not None and behavior_age_s > BEHAVIOR_STALE_S):
        title, detail, tone = _STATUS_UNKNOWN
        return {"key": "UNKNOWN", "title": title, "detail": detail, "tone": tone}
    if b.get("estop_active"):
        mode = "ESTOP_HOLD"
    title, detail, tone = _STATUS_TEXT.get(mode, _STATUS_MOVING)
    return {"key": mode, "title": title, "detail": detail, "tone": tone}


MOVING_STATUS_KEY = "MOVING_NOTICE"


def moving_status(kind: str = "") -> Dict[str, str]:
    """「移動中です。ご注意ください」の状態(tone=caution。琥珀色の目立つ表示)。kind は動きの種類(go_pin / return_home / manual / exploring …)。"""
    return {"key": MOVING_STATUS_KEY, "title": MOVING_TITLE, "detail": "", "tone": "caution", "moving": str(kind or "")[:20]}


def _int(v: Any, default: int = 0) -> int:
    if isinstance(v, bool):
        return default
    try:
        return int(v)
    except (TypeError, ValueError, OverflowError):
        return default


def customer_menu(menu_state: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """inventory_manager の /nokosahen/menu_state を、お客さん向け表示用に整える(名前の長さ・個数を制限)。"""
    ms = menu_state if isinstance(menu_state, dict) else {}
    items: List[Dict[str, Any]] = []
    raw = ms.get("items")
    for it in (raw if isinstance(raw, list) else [])[:MAX_CUSTOMER_ITEMS]:
        if not isinstance(it, dict):
            continue
        name = str(it.get("name", "")).strip()[:MAX_MENU_NAME]
        if not name:
            continue
        initial = max(0, _int(it.get("initial")))
        remaining = max(0, _int(it.get("remaining")))
        sold = max(0, _int(it.get("sold")))
        items.append({"name": name, "initial": initial, "remaining": remaining, "sold": sold,
                      "ratio": round(remaining / initial, 3) if initial > 0 else 0.0})
    promote = ms.get("promote_item")
    promote = str(promote).strip()[:MAX_MENU_NAME] if isinstance(promote, str) and promote.strip() else None
    return {"items": items, "promote_item": promote}


def build_customer_payload(menu_state: Optional[Dict[str, Any]], behavior: Optional[Dict[str, Any]],
                           behavior_age_s: Optional[float] = None, announce_text: Optional[str] = None,
                           moving: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """customer / inventory の画面が使う中身。

    moving: moving_notice.MovingNotice.evaluate の結果(無い・古いときは None)。画面に出すと判定されたとき(display が True)だけ
    `out["moving"] = {"active": True, "kind": ...}` を付ける。`status` は通常の文言のまま(DisplayHub が customer モードのときだけ「移動中」に差し替える)。"""
    out: Dict[str, Any] = {"status": customer_status(behavior, behavior_age_s), "menu": customer_menu(menu_state)}
    if isinstance(moving, dict) and moving.get("display") is True:
        out["moving"] = {"active": True, "kind": str(moving.get("kind") or "")[:20]}
    if isinstance(announce_text, str) and announce_text.strip():
        out["announce"] = announce_text.strip()[:120]
    return out


def plan_inventory_update(menu_state: Optional[Dict[str, Any]], menu: Any, remaining: Any
                          ) -> Tuple[Optional[Dict[str, Any]], str]:
    """「メニュー menu の残数を remaining にする」を、既存の残数管理(inventory_manager)への入力に直す。(plan, エラー)。

    残数 = 初期数 - 累計販売数 なので、
      remaining <= 初期数 : /nokosahen/sold_total_event {"menu","sold_total": 初期数 - remaining}(絶対値。送り直しても二重計上しない。
                            消費ログにも差分が残る)
      remaining >  初期数 : 初期数も増やす。/nokosahen/menu_init で、全メニューの現状を保ったまま、このメニューだけ
                            initial=remaining+sold, remaining=remaining にする(ラズパイ版の別手段=厨房UIの「初期食数」と同じ経路)。
    """
    if not isinstance(menu, str) or not menu.strip() or len(menu.strip()) > MAX_MENU_NAME:
        return None, "menuが不正です"
    menu = menu.strip()
    if isinstance(remaining, bool) or not isinstance(remaining, int) or remaining < 0 or remaining > MAX_REMAINING:
        return None, f"残数は0〜{MAX_REMAINING}の整数にしてください"
    ms = menu_state if isinstance(menu_state, dict) else {}
    raw = ms.get("items")
    items = [it for it in (raw if isinstance(raw, list) else []) if isinstance(it, dict)]
    if not items:
        return None, "メニューが登録されていません(先に厨房の画面で本日のメニューと初期食数を登録してください)"
    target = next((it for it in items if str(it.get("name", "")).strip() == menu), None)
    if target is None:
        return None, "そのメニューは登録されていません"
    initial = max(0, _int(target.get("initial")))
    sold = max(0, _int(target.get("sold")))
    if remaining <= initial:
        return {"kind": "sold_total", "topic": "/nokosahen/sold_total_event",
                "payload": {"menu": menu, "sold_total": initial - remaining, "source": "ipad_display"},
                "initial": initial, "sold": initial - remaining}, ""
    new_items = []
    for it in items:
        name = str(it.get("name", "")).strip()
        if not name:
            continue
        if name == menu:
            new_items.append({"name": name, "initial": remaining + sold, "remaining": remaining, "sold": sold})
        else:
            new_items.append({"name": name, "initial": max(0, _int(it.get("initial"))),
                              "remaining": max(0, _int(it.get("remaining"))), "sold": max(0, _int(it.get("sold")))})
    return {"kind": "menu_init", "topic": "/nokosahen/menu_init", "payload": {"items": new_items},
            "initial": remaining + sold, "sold": sold}, ""


def _eff_initial(initial: int, remaining: int, sold: int) -> int:
    """inventory_manager の _on_menu_init と同じ式(initial = max(initial, remaining + sold))。送った内容が状態に反映されたかの比較に使う。"""
    return max(initial, remaining + sold)


def validate_menu_name(name: Any) -> Tuple[Optional[str], str]:
    """メニュー名の検証。(整えた名前, エラー)。前後の空白を除き、1〜40字、制御文字・改行・書式文字は不可。"""
    if not isinstance(name, str):
        return None, "メニュー名は文字で入力してください"
    n = name.strip()
    if not n:
        return None, "メニュー名を入力してください"
    if len(n) > MAX_MENU_NAME:
        return None, f"メニュー名は{MAX_MENU_NAME}文字までです"
    for ch in n:
        cat = unicodedata.category(ch)
        if cat.startswith("C") or cat in ("Zl", "Zp"):
            return None, "メニュー名に使えない文字(制御文字・改行など)が含まれています"
    return n, ""


def _valid_initial(v: Any) -> Tuple[Optional[int], str]:
    if isinstance(v, bool) or not isinstance(v, int) or v < 0 or v > MAX_REMAINING:
        return None, f"初期食数は0〜{MAX_REMAINING}の整数にしてください"
    return v, ""


def plan_menu_edit(menu_state: Optional[Dict[str, Any]], op: Any) -> Tuple[Optional[Dict[str, Any]], str]:
    """メニューの追加・名前の変更・削除・初期食数の変更を、`/nokosahen/menu_init` への入力(**新しい一覧の全体**)に直す。(plan, エラー)。

    inventory_manager の menu_init は「全置き換え」(送った一覧が新しい本日のメニューになり、含まれない名前は消える。空の一覧は全削除)。
    なので毎回、現在の一覧(menu_state)に1つの変更を当てた全体を返す。
      op = {"op": "add", "name": 名前, "initial": 初期食数}                 sold=0, remaining=初期食数 で末尾に追加
           {"op": "rename", "name": 旧名, "new_name": 新名}                 販売済み数・残数・初期食数はそのまま引き継ぐ
           {"op": "delete", "name": 名前}                                  一覧から除く(最後の1品を消すと本日のメニューが空になる)
           {"op": "set_initial", "name": 名前, "initial": 初期食数}         remaining = 初期 - 販売済み(0未満にならない)
    検証: 名前は1〜40字(前後の空白を除く・制御文字不可)、重複不可、最大12品、初期食数は0〜99999の整数(boolは不可)、
    存在しないメニューの変更・削除は断る。項目は操作ごとに決まったものだけ(余計な項目は断る)。
    """
    if not isinstance(op, dict):
        return None, "操作の形式が不正です"
    kind = op.get("op")
    keys = {"add": {"op", "name", "initial"}, "rename": {"op", "name", "new_name"}, "delete": {"op", "name"},
            "set_initial": {"op", "name", "initial"}}
    if not isinstance(kind, str) or kind not in keys:
        return None, "操作は add / rename / delete / set_initial のどれかにしてください"
    if set(op) != keys[kind]:
        return None, "指定できない項目、または足りない項目があります"
    name, err = validate_menu_name(op.get("name"))
    if name is None:
        return None, err
    ms = menu_state if isinstance(menu_state, dict) else {}
    raw = ms.get("items")
    cur: List[Dict[str, Any]] = []
    for it in (raw if isinstance(raw, list) else []):
        if not isinstance(it, dict):
            continue
        nm = str(it.get("name", "")).strip()
        if not nm:
            continue
        cur.append({"name": nm, "initial": max(0, _int(it.get("initial"))), "remaining": max(0, _int(it.get("remaining"))),
                    "sold": max(0, _int(it.get("sold")))})
    idx = next((i for i, it in enumerate(cur) if it["name"] == name), None)
    new = [dict(it) for it in cur]
    summary = ""
    if kind == "add":
        initial, err = _valid_initial(op.get("initial"))
        if initial is None:
            return None, err
        if idx is not None:
            return None, "同じ名前のメニューがすでにあります"
        if len(cur) >= MAX_CUSTOMER_ITEMS:
            return None, f"メニューは最大{MAX_CUSTOMER_ITEMS}品までです(先に不要なメニューを削除してください)"
        new.append({"name": name, "initial": initial, "remaining": initial, "sold": 0})
        summary = f"「{name}」を追加(初期{initial}食)"
    else:
        if idx is None:
            return None, "そのメニューは登録されていません"
        if kind == "rename":
            new_name, err = validate_menu_name(op.get("new_name"))
            if new_name is None:
                return None, err
            if new_name == name:
                return None, "名前が変わっていません"
            if any(it["name"] == new_name for it in cur):
                return None, "同じ名前のメニューがすでにあります"
            new[idx]["name"] = new_name
            summary = f"「{name}」を「{new_name}」に変更"
        elif kind == "delete":
            del new[idx]
            summary = f"「{name}」を削除"
        else:   # set_initial
            initial, err = _valid_initial(op.get("initial"))
            if initial is None:
                return None, err
            sold = new[idx]["sold"]
            new[idx]["initial"] = initial
            new[idx]["remaining"] = max(0, initial - sold)
            summary = f"「{name}」の初期食数を{initial}食に変更"
    expect = [(it["name"], _eff_initial(it["initial"], it["remaining"], it["sold"])) for it in new]
    return {"kind": "menu_init", "topic": "/nokosahen/menu_init", "payload": {"items": new}, "op": kind,
            "summary": summary, "expect": expect}, ""


# ---------------------------------------------------------------------------
# アンケートの回答セッション(receiver.py の Session を移植。上限を足した)
# ---------------------------------------------------------------------------
class AnswerSession:
    """phase: idle → answering → saved → idle / answering → abandoned → idle。active は answering の間だけ。"""

    def __init__(self, clock: Callable[[], float] = time.monotonic, timeout: float = SESSION_ABANDON_S,
                 max_answer: float = SESSION_MAX_ANSWER_S) -> None:
        self.clock = clock
        self.timeout = float(timeout)
        self.max_answer = float(max_answer)
        self.phase = "idle"
        self.since: Optional[float] = None
        self.started: Optional[float] = None       # この回答中が始まった時刻(最長の判定用)
        self.last_start: Optional[float] = None    # 最後の start/beat 通知
        self.last_activity: Optional[float] = None
        self.saved_seq = 0
        self.saved_at: Optional[float] = None
        self.cooldown_until = 0.0

    def resolve(self) -> None:
        t = self.clock()
        if self.phase == "answering":
            if t - self.last_start >= self.timeout:
                self.phase, self.since = "abandoned", self.last_start + self.timeout
            elif t - self.started >= self.max_answer:
                self.phase, self.since = "abandoned", self.started + self.max_answer
                self.cooldown_until = t + MAX_ANSWER_COOLDOWN_S
        if self.phase == "saved" and t - self.since >= SAVED_SHOW_S:
            self.phase, self.since = "idle", t
        if self.phase == "abandoned" and t - self.since >= ABANDONED_SHOW_S:
            self.phase, self.since = "idle", t

    def start(self) -> bool:
        """回答の開始(タップ)。回答中に重ねて届いても、最長の起点は進めない(beatと同じ)。
        最長で打ち切った直後(cooldown)は受け付けない(False)。放棄後に遅れて届いた beat では復活しない(復活は新しい start だけ)。"""
        self.resolve()
        t = self.clock()
        if self.phase != "answering" and t < self.cooldown_until:
            return False
        self.last_start, self.last_activity = t, t
        if self.phase != "answering":
            self.phase, self.since, self.started = "answering", t, t
        return True

    def beat(self) -> bool:
        """回答中の継続通知。answering でなければ何もしない(False)。"""
        self.resolve()
        if self.phase != "answering":
            return False
        t = self.clock()
        self.last_start, self.last_activity = t, t
        return True

    def end(self) -> None:
        self.resolve()
        t = self.clock()
        if self.phase == "answering":
            self.phase, self.since = "abandoned", t
        self.last_activity = t

    def touch(self) -> None:
        self.last_activity = self.clock()

    def saved(self) -> None:
        self.resolve()
        t = self.clock()
        self.saved_seq += 1
        self.saved_at = self.last_activity = t
        if self.phase == "answering":
            self.phase, self.since = "saved", t

    def reset(self) -> None:
        self.phase, self.since, self.started, self.last_start = "idle", self.clock(), None, None

    def state(self) -> str:
        self.resolve()
        return self.phase

    def active(self) -> bool:
        return self.state() == "answering"


def _random_digits(rng: random.Random) -> int:
    """元の模擬POSページの「数字をランダム化」と同じ: 1〜3桁。"""
    digits = 1 + int(rng.random() * 3)
    lo = 0 if digits == 1 else 10 ** (digits - 1)
    hi = 10 ** digits - 1
    return int(lo + rng.random() * (hi - lo + 1))


class DisplayHub:
    """iPad表示の現在のモードと、各モードの状態を持つ。スレッドセーフ。"""

    def __init__(
        self,
        mode_file: Optional[str] = None,
        survey_store: Optional[SurveyStore] = None,
        customer_provider: Optional[Callable[[], Dict[str, Any]]] = None,
        menu_state_provider: Optional[Callable[[], Optional[Dict[str, Any]]]] = None,
        inventory_sink: Optional[Callable[[str, Dict[str, Any]], bool]] = None,
        logger: Any = None,
        clock: Callable[[], float] = time.monotonic,
        wall: Callable[[], float] = time.time,
        rng: Optional[random.Random] = None,
        abandon_s: float = SESSION_ABANDON_S,
        max_answer_s: float = SESSION_MAX_ANSWER_S,
        auto_file: Optional[str] = None,
    ) -> None:
        self._lock = threading.RLock()
        self._auto_path = os.path.expanduser(auto_file) if auto_file else None
        self.auto_face = self._load_auto()
        self._present_since: Optional[float] = None
        self._absent_since: Optional[float] = None
        self._clock = clock
        self._wall = wall
        self._rng = rng or random.Random()
        self._logger = logger
        self._mode_path = os.path.expanduser(mode_file) if mode_file else None
        self.store = survey_store
        self._customer_provider = customer_provider
        self._menu_state_provider = menu_state_provider
        self._inventory_sink = inventory_sink
        self.session = AnswerSession(clock, abandon_s, max_answer_s)
        self.mode = self._load_mode()
        self.rev = 1                       # モードが変わるたびに増える(ログ・表示用)
        self.mode_since = clock()
        self.pending: Optional[Dict[str, Any]] = None   # {"mode","source","since"}(回答中で保留している切替)
        self.condition: Optional[str] = survey_store.read_condition() if survey_store is not None else None   # 前回の実験条件(再起動後も引き継ぐ)
        self.ocr: Dict[str, Any] = {"seq": 0, "action": None, "values": list(OCR_DEFAULTS), "at": None}
        self.ocr_seen: Optional[Dict[str, Any]] = None  # {"seq","values","at"}
        self._clients: Dict[str, float] = {}
        self.last_error: Optional[str] = None
        self._menu_lock = threading.Lock()                 # メニュー編集を1つずつにする(全置き換えなので、並行すると片方が消える)
        self._menu_sent: Optional[Tuple[List[Tuple[str, int]], float]] = None   # 直前に送った一覧(名前, 初期)と時刻

    # --- ログ・永続化 ---
    def _log(self, level: str, text: str) -> None:
        fn = getattr(self._logger, level, None) if self._logger is not None else None
        if fn is not None:
            try:
                fn(text)
            except Exception:  # noqa: BLE001
                pass

    def _load_mode(self) -> str:
        if not self._mode_path:
            return DEFAULT_MODE
        try:
            with open(self._mode_path, "r", encoding="utf-8") as f:
                m = f.read().strip()
            return m if m in MODES else DEFAULT_MODE
        except OSError:
            return DEFAULT_MODE

    def _save_mode(self, mode: str) -> None:
        if not self._mode_path:
            return
        try:
            d = os.path.dirname(self._mode_path) or "."
            os.makedirs(d, exist_ok=True)
            fd, tmp = tempfile.mkstemp(prefix=".nokosahen_dispmode_", suffix=".tmp", dir=d)
            with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as f:
                f.write(mode + "\n")
            os.replace(tmp, self._mode_path)
        except OSError as exc:  # 保存できなくてもメモリ上のモードで動く(再起動後に戻るだけ)
            self._log("warn", f"表示モードを保存できませんでした: {exc}")

    # --- 顔⇄アンケートの自動切替 ---
    def _load_auto(self) -> bool:
        if not self._auto_path:
            return False
        try:
            with open(self._auto_path, "r", encoding="utf-8") as f:
                return f.read().strip() == "on"
        except OSError:
            return False

    def _save_auto(self, on: bool) -> None:
        if not self._auto_path:
            return
        try:
            d = os.path.dirname(self._auto_path) or "."
            os.makedirs(d, exist_ok=True)
            fd, tmp = tempfile.mkstemp(prefix=".nokosahen_dispauto_", suffix=".tmp", dir=d)
            with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as f:
                f.write("on\n" if on else "off\n")
            os.replace(tmp, self._auto_path)
        except OSError as exc:
            self._log("warn", f"自動切替の設定を保存できませんでした: {exc}")

    def set_auto(self, enabled: Any, source: str = "api") -> Dict[str, Any]:
        """自動切替のON/OFF(enabled は真偽値だけ)。ONにしたとき、顔でもアンケートでもなければ顔にする(回答中は保留)。"""
        if not isinstance(enabled, bool):
            return {"ok": False, "code": 400, "message": "enabled は true か false で指定してください"}
        with self._lock:
            self.auto_face = enabled
            self._save_auto(enabled)
            self._present_since = None
            self._absent_since = None
            self._log("info", f"iPad表示の自動切替(顔⇄アンケート)を {'ON' if enabled else 'OFF'} にしました(要求元: {source})")
            if enabled and self.mode not in ("face", "survey"):
                r = self.request_mode("face", "自動切替ON", defer=True)
                return {"ok": True, "code": 200, "auto_face": True, "mode": self.mode, "deferred": r.get("deferred", False), "message": ""}
            return {"ok": True, "code": 200, "auto_face": enabled, "mode": self.mode, "deferred": False, "message": ""}

    def note_presence(self, present: bool) -> None:
        """人が見えているか(1秒おきに node から)。自動切替がONで、表示が顔かアンケートのときだけ働く。"""
        with self._lock:
            now = self._clock()
            if present:
                self._absent_since = None
                if self._present_since is None:
                    self._present_since = now
            else:
                self._present_since = None
                if self._absent_since is None:
                    self._absent_since = now
            if not self.auto_face:
                return
            self._resolve_locked()
            if self.mode == "face" and present and self._present_since is not None \
                    and now - self._present_since >= AUTO_PRESENT_S:
                self.request_mode("survey", "自動切替(人が来た)", defer=True)
            elif self.mode == "survey" and not present and self._absent_since is not None \
                    and now - self._absent_since >= AUTO_ABSENT_S \
                    and now - self.mode_since >= AUTO_MIN_STAY_S \
                    and self.session.phase not in ("answering", "saved"):
                self.request_mode("face", "自動切替(人がいなくなった)", defer=True)

    # --- モード ---
    def _apply_mode(self, mode: str, source: str) -> None:
        prev = self.mode
        self.mode = mode
        self.rev += 1
        self.mode_since = self._clock()
        self.pending = None
        self.session.reset()    # 別のモードへ移ったら回答の状態は持ち越さない(保存済みの回答は残る)
        self._save_mode(mode)
        self._log("info", f"iPad表示のモードを {prev} -> {mode} に変更しました(要求元: {source})")

    def _resolve_locked(self) -> None:
        self.session.resolve()
        if self.pending and self.session.phase not in ("answering", "saved"):
            self._apply_mode(self.pending["mode"], self.pending["source"] + "(保留していた要求)")

    def tick(self) -> None:
        """時間で進む処理(保留していた切替の実行・回答の放棄)。1秒おきに呼ぶ。状態の読み出しでも同じ処理をする。"""
        with self._lock:
            self._resolve_locked()

    def request_mode(self, mode: Any, source: str = "api", defer: bool = True) -> Dict[str, Any]:
        """モードの切替を要求する。defer=True: 回答中・保存直後は保留して、終わってから切り替える(ROS・スケジュール用)。
        defer=False: 回答中は断る(ダッシュボードの手動切替用。その人の回答を消さないため)。
        戻り値 {"ok","changed","deferred","mode","message","code"}"""
        m = normalize_mode(mode)
        if m is None:
            return {"ok": False, "changed": False, "deferred": False, "mode": self.mode, "code": 400,
                    "message": "modeは customer / survey / inventory / ocrtest / face のどれかを指定してください"}
        with self._lock:
            self._resolve_locked()
            if m == self.mode:
                self.pending = None   # すでにそのモード。保留していた別の要求は取り消す
                return {"ok": True, "changed": False, "deferred": False, "mode": self.mode, "code": 200, "message": ""}
            phase = self.session.phase
            if phase == "answering" or (defer and phase == "saved"):
                if defer:
                    self.pending = {"mode": m, "source": source, "since": self._clock()}
                    msg = "アンケート回答中のため、回答が終わってから切り替えます"
                    self._log("info", f"iPad表示の切替({m})を保留: {msg}")
                    return {"ok": True, "changed": False, "deferred": True, "mode": self.mode, "code": 200, "message": msg}
                return {"ok": False, "changed": False, "deferred": False, "mode": self.mode, "code": 409,
                        "message": "アンケート回答中のため切り替えられません(回答が終わってからもう一度押してください)"}
            self._apply_mode(m, source)
            return {"ok": True, "changed": True, "deferred": False, "mode": self.mode, "code": 200, "message": ""}

    # --- アンケートのセッション・回答 ---
    def set_condition(self, c: str) -> None:
        with self._lock:
            self.condition = c or None
            if c and self.store is not None:
                self.store.write_condition(c)

    def current_condition(self) -> Optional[str]:
        with self._lock:
            return self.condition

    def session_event(self, event: Any, condition: Any = None) -> Dict[str, Any]:
        """ページからの回答中の通知。event: start / beat / end。アンケート表示中以外は受け付けない。"""
        if event not in ("start", "beat", "end"):
            return {"ok": False, "code": 400, "message": "eventは start / beat / end のどれかにしてください"}
        cond = ""
        if condition is not None:
            cond, err = validate_condition(condition)
            if cond is None:
                return {"ok": False, "code": 400, "message": err}
        with self._lock:
            self._resolve_locked()
            if self.mode != "survey":
                return {"ok": False, "code": 409, "message": "いまはアンケートの表示中ではありません", "mode": self.mode}
            if cond:
                self.set_condition(cond)
            if event == "start":
                if not self.session.start():
                    return {"ok": True, "code": 200, "active": False, "session_state": self.session.phase,
                            "message": "回答中の通知を受け付けられません(しばらく待ってください)"}
            elif event == "beat":
                if not self.session.beat():
                    return {"ok": True, "code": 200, "active": False, "session_state": self.session.phase}
            else:
                self.session.end()
            return {"ok": True, "code": 200, "active": self.session.phase == "answering",
                    "session_state": self.session.phase}

    def submit_response(self, payload: Any, replay: bool = False) -> Tuple[int, Dict[str, Any]]:
        """回答を検証して保存する。(HTTPステータス, 本文)。保存に失敗したら saved にしない(answering のまま=ロボットは止まり続ける。
        放棄の上限で解除される)。replay=過去の未送信分の再送(今の回答者の完了ではないので保存完了の通知にしない)。"""
        if self.store is None:
            return 503, {"ok": False, "error": "アンケートの保存先が未設定です"}
        rec, err = validate_response(payload)
        if rec is None:
            return 400, {"ok": False, "error": err}
        ok, dup, serr = self.store.save(rec)
        if not ok:
            self.last_error = serr
            self._log("error", f"アンケート回答の保存に失敗: {serr}")
            return 507 if serr and "上限" in serr else 500, {"ok": False, "error": serr or "save failed"}
        with self._lock:
            if replay or dup:
                self.session.touch()
            else:
                self.session.saved()
        return 200, ({"ok": True, "duplicate": True} if dup else {"ok": True})

    # --- OCRテスト(模擬POS)---
    def ocr_command(self, data: Any) -> Tuple[int, Dict[str, Any]]:
        """模擬POSの数字をランダム化 / 元に戻す / 指定する。ocrtestモードのときだけ。数字は**ここ(サーバ)で決めて**状態に載せる。
        画面はその数字を表示するだけ(画面側で勝手に決めないので、管理画面から送った値と画面の表示が必ず一致する)。"""
        cmd, err = validate_ocr_mock_request(data)
        if cmd is None:
            return 400, {"ok": False, "error": err, "message": err}
        with self._lock:
            if self.mode != "ocrtest":
                return 409, {"ok": False, "error": "not ocrtest mode", "message": OCR_NOT_MODE_MESSAGE}
            action = cmd["action"]
            if action == "randomize":
                values = [_random_digits(self._rng) for _ in range(OCR_ROWS)]
            elif action == "reset":
                values = list(OCR_DEFAULTS)
            else:
                values = list(cmd["values"])
            seq = max(self.ocr["seq"] + 1, int(self._wall() * 1000))
            self.ocr = {"seq": seq, "action": action, "values": values, "at": int(self._wall())}
            return 200, {"ok": True, "ocr_mock": {"seq": seq, "action": action, "at": self.ocr["at"]}}

    def _ocr_public(self) -> Dict[str, Any]:
        return {"seq": self.ocr["seq"], "action": self.ocr["action"], "values": list(self.ocr["values"]),
                "at": self.ocr["at"]}

    def _ocr_admin(self) -> Optional[Dict[str, Any]]:
        if self.ocr["seq"] == 0 and self.ocr_seen is None:
            return None
        out: Dict[str, Any] = {"seq": self.ocr["seq"] or None, "action": self.ocr["action"], "at": self.ocr["at"]}
        seen = self.ocr_seen
        if seen is not None:
            out.update(ack_seq=seen["seq"], shown_values=list(seen["values"]), acked_at=int(seen["at"]),
                       ack_age_s=max(0, int(self._wall() - seen["at"])))
        return out

    # --- 接続中のiPad ---
    def note_client(self, client: str, seen_ocr_seq: Optional[int] = None) -> None:
        """状態を読みに来た端末を覚える(接続中の台数・最後に読んだ時刻を管理画面へ出すため)。
        ocrtestモードのとき、画面が表示している模擬POSの番号(seen)を受け取って、読み戻しとして記録する(読み取りだけで、状態は変えない)。"""
        now = self._clock()
        with self._lock:
            self._clients[client] = now
            if len(self._clients) > 64:
                for k in [k for k, t in self._clients.items() if now - t > 60]:
                    self._clients.pop(k, None)
            if (seen_ocr_seq is not None and self.mode == "ocrtest" and seen_ocr_seq == self.ocr["seq"]
                    and self.ocr["seq"] != 0):
                self.ocr_seen = {"seq": seen_ocr_seq, "values": list(self.ocr["values"]), "at": self._wall()}

    def clients_summary(self) -> Dict[str, Any]:
        now = self._clock()
        with self._lock:
            ages = [now - t for t in self._clients.values()]
        active = [a for a in ages if a <= CLIENT_ACTIVE_S]
        return {"connected": len(active), "last_poll_age_s": round(min(ages), 1) if ages else None}

    # --- 状態 ---
    def _base_state_locked(self) -> Dict[str, Any]:
        self._resolve_locked()
        s = self.session
        t = s.clock()
        age = None if s.last_activity is None else round(max(0.0, t - s.last_activity), 1)
        sa = None if s.saved_at is None else round(max(0.0, t - s.saved_at), 1)
        return {
            "mode": self.mode,
            "active": s.phase == "answering",
            "count": self.store.count() if self.store is not None else None,
            "last_activity_age_s": age,
            "condition": self.current_condition(),
            "updated": int(self._wall()),
            "session_state": s.phase,
            "saved_seq": s.saved_seq,
            "last_saved_age_s": sa,
        }

    def public_state(self) -> Dict[str, Any]:
        """iPadと survey_monitor が読む状態(GET /display/api/state。iPadはログイン済みのセッション、survey_monitor は同じPCの中=ループバックから認証なしで。読み取り専用)。
        survey_monitor の契約(mode, active, count, last_activity_age_s, condition, updated, session_state, saved_seq)を含む。"""
        with self._lock:
            st = self._base_state_locked()
            st.update(ok=True, rev=self.rev, mode_age_s=round(self._clock() - self.mode_since, 1),
                      pending_mode=self.pending["mode"] if self.pending else None,
                      survey_version=SURVEY_DEF["version"])
            if self.mode in ("customer", "inventory"):
                st["customer"] = self._customer()
            if self.mode == "ocrtest":
                st["ocr_mock"] = self._ocr_public()
        return st

    def admin_state(self) -> Dict[str, Any]:
        """管理画面(設定タブのアンケートカード)用。ラズパイ版 kiosk_admin の /state と同じ形 + iPadの情報。"""
        with self._lock:
            st = self._base_state_locked()
            st.update(receiver=True, auto_face=self.auto_face, pending_mode=self.pending["mode"] if self.pending else None,
                      ocr_mock=self._ocr_admin(), ipad=self.clients_summary(), backend="ipad",
                      mode_label=MODE_LABELS.get(self.mode, self.mode))
        return st

    def _customer(self) -> Dict[str, Any]:
        if self._customer_provider is None:
            return build_customer_payload(None, None, None, None)
        try:
            c = self._customer_provider()
        except Exception as exc:  # noqa: BLE001  (表示の組み立ての不具合で状態の応答を壊さない)
            self._log("warn", f"お客さん向け表示の組み立てに失敗: {exc!r}")
            return build_customer_payload(None, None, None, None)
        return self._apply_moving(c)

    def _apply_moving(self, c: Any) -> Any:
        """「移動中です」を出すのは customer モードのときだけ。残数入力(inventory)など他のモードの画面は上書きしない(moving も外す)。
        customer モードで moving が有効なら、status だけを移動中の注意に差し替える(menu・announce=残数などはそのまま)。"""
        if not isinstance(c, dict):
            return c
        mv = c.pop("moving", None)
        if self.mode == "customer" and isinstance(mv, dict) and mv.get("active") is True:
            c["status"] = moving_status(mv.get("kind", ""))
        return c

    # --- 残数入力(スタッフ) ---
    def set_inventory(self, menu: Any, remaining: Any) -> Tuple[int, Dict[str, Any]]:
        """残数の入力を既存の残数管理へ反映する。呼び出し元(display_routes)が権限(ログイン+display.inventory_input)を確認済みの前提。"""
        if self._menu_state_provider is None or self._inventory_sink is None:
            return 503, {"ok": False, "error": "残数の入力は使えません(残数管理に接続されていません)"}
        try:
            ms = self._menu_state_provider()
        except Exception:  # noqa: BLE001
            ms = None
        plan, err = plan_inventory_update(ms, menu, remaining)
        if plan is None:
            return 400, {"ok": False, "error": err, "message": err}
        try:
            sent = bool(self._inventory_sink(plan["topic"], plan["payload"]))
        except Exception as exc:  # noqa: BLE001
            self._log("error", f"残数の送信に失敗: {exc!r}")
            sent = False
        if not sent:
            return 503, {"ok": False, "error": "残数管理へ送れませんでした"}
        return 200, {"ok": True, "menu": str(menu).strip(), "remaining": remaining, "kind": plan["kind"],
                     "initial": plan["initial"], "raised_initial": plan["kind"] == "menu_init"}

    def menu_snapshot(self) -> Dict[str, Any]:
        """いまのメニュー一覧(残数管理の menu_state を整えたもの)。TLの管理画面のメニュー編集カード用。"""
        try:
            ms = self._menu_state_provider() if self._menu_state_provider is not None else None
        except Exception:  # noqa: BLE001
            ms = None
        raw = ms.get("items") if isinstance(ms, dict) else None
        items = []
        for it in (raw if isinstance(raw, list) else []):
            if isinstance(it, dict) and str(it.get("name", "")).strip():
                items.append({"name": str(it.get("name")).strip(), "initial": max(0, _int(it.get("initial"))),
                              "remaining": max(0, _int(it.get("remaining"))), "sold": max(0, _int(it.get("sold")))})
        return {"items": items, "max_items": MAX_CUSTOMER_ITEMS, "max_name": MAX_MENU_NAME, "max_initial": MAX_REMAINING,
                "editable": self._menu_state_provider is not None and self._inventory_sink is not None}

    # --- メニューの追加・名前の変更・削除・初期食数(iPad と TL の管理画面の両方から。呼び出し元が認証済みの前提) ---
    def edit_menu(self, op: Any) -> Tuple[int, Dict[str, Any]]:
        """メニューを1つ編集し、新しい一覧の全体を `/nokosahen/menu_init` で送る(inventory_manager は全置き換え)。

        直前の編集が残数管理の状態に反映される前(最長 MENU_SYNC_WAIT_S 秒)に次の編集が来ると、古い一覧を元に全体を送って
        直前の編集を消してしまうので、その間は断る(409)。"""
        if self._menu_state_provider is None or self._inventory_sink is None:
            return 503, {"ok": False, "error": "メニューの編集は使えません(残数管理に接続されていません)"}
        with self._menu_lock:
            try:
                ms = self._menu_state_provider()
            except Exception:  # noqa: BLE001
                ms = None
            plan, err = plan_menu_edit(ms, op)
            if plan is None:
                return 400, {"ok": False, "error": err, "message": err}
            sent = self._menu_sent
            if sent is not None and self._clock() - sent[1] < MENU_SYNC_WAIT_S:
                raw = (ms or {}).get("items") if isinstance(ms, dict) else None
                have = [(str(it.get("name", "")).strip(), max(0, _int(it.get("initial"))))
                        for it in (raw if isinstance(raw, list) else []) if isinstance(it, dict) and str(it.get("name", "")).strip()]
                if have != sent[0]:
                    msg = "直前の変更がまだ反映されていません。数秒待ってからもう一度押してください"
                    return 409, {"ok": False, "error": msg, "message": msg}
            try:
                ok = bool(self._inventory_sink(plan["topic"], plan["payload"]))
            except Exception as exc:  # noqa: BLE001
                self._log("error", f"メニュー編集の送信に失敗: {exc!r}")
                ok = False
            if not ok:
                return 503, {"ok": False, "error": "残数管理へ送れませんでした"}
            self._menu_sent = (plan["expect"], self._clock())
        self._log("info", f"メニュー編集: {plan['summary']} (全{len(plan['payload']['items'])}品を menu_init で送信)")
        return 200, {"ok": True, "op": plan["op"], "summary": plan["summary"], "count": len(plan["payload"]["items"]),
                     "items": [it["name"] for it in plan["payload"]["items"]]}

    def status_for_ros(self) -> Dict[str, Any]:
        """/nokosahen/display_state に流す要約(ROS側から見える現在の表示)。"""
        with self._lock:
            st = self._base_state_locked()
            return {"mode": st["mode"], "pending_mode": self.pending["mode"] if self.pending else None,
                    "session_state": st["session_state"], "active": st["active"], "rev": self.rev,
                    "clients": self.clients_summary()["connected"]}


class LocalDisplayAdmin:
    """SurveyAdminProxy(ラズパイの管理APIへの中継)と同じ呼び口で、1台目の中の DisplayHub を操作する。

    これを `survey_admin` として差し込むので、ダッシュボードの設定タブのカード・DisplayScheduleController(スケジュール)・
    一時的な上書きの復元(ephemeral_overrides)は、コードを変えずにiPad表示で動く。
    """

    configured = True

    def __init__(self, hub: DisplayHub) -> None:
        self.hub = hub

    def _state(self) -> Dict[str, Any]:
        return {"ok": True, "configured": True, "state": self.hub.admin_state()}

    def get_state(self, fresh: bool = False) -> Dict[str, Any]:  # noqa: ARG002  (メモリ上の状態なので常に最新)
        return self._state()

    def set_mode(self, data: Any) -> Tuple[int, Dict[str, Any]]:
        mode, err = validate_mode_request(data)
        if mode is None:
            return 400, {"ok": False, "configured": True, "error": err, "message": err}
        r = self.hub.request_mode(mode, "admin", defer=False)
        if not r["ok"]:
            return 200, {"ok": False, "configured": True, "error": r["message"], "message": r["message"]}
        return 200, self._state()

    def set_auto(self, data: Any) -> Tuple[int, Dict[str, Any]]:
        enabled = data.get("enabled") if isinstance(data, dict) else None
        r = self.hub.set_auto(enabled, "admin")
        if not r["ok"]:
            return 400, {"ok": False, "configured": True, "error": r["message"], "message": r["message"]}
        return 200, self._state()

    def set_ocr_mock(self, data: Any) -> Tuple[int, Dict[str, Any]]:
        code, body = self.hub.ocr_command(data)
        if code == 400:
            return 400, {"ok": False, "configured": True, "error": body.get("error"), "message": body.get("message")}
        if code != 200:
            return 200, {"ok": False, "configured": True, "error": body.get("error"), "message": body.get("message")}
        return 200, {"ok": True, "configured": True, "state": {"ocr_mock": body["ocr_mock"]}}

    def get_csv(self) -> Tuple[int, bytes, str]:
        if self.hub.store is None:
            return 404, b"", "アンケートの保存先が未設定です"
        data, err = self.hub.store.csv_bytes()
        if data is None:
            return 502, b"", err
        return 200, data, ""
