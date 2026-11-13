"""「移動中です。ご注意ください」を出す(画面と音)かどうかの判定(純Python、ROS非依存)。

【2026-10-27追加】docs/ipad_display.md・docs/announcements.md。表示と音の通知だけで、ロボットの動作(cmd_vel・ゴール・停止)には
一切影響しない。ダッシュボード(full_dashboard_node)が1つだけ判定し、
  - iPad画面(customer.status を「移動中」に差し替え。DisplayHub が customer モードのときだけ使う)
  - 状況アナウンス(`/nokosahen/moving_notice` を announce_node が読み、イベント `moving_notice` の鳴動条件にする)
の両方に同じ結果を渡す(画面と音が食い違わない)。

判定 = 「車輪が実際に動いている」(オドメトリの速さ、ヒステリシスつき) かつ 「出してよい動き」(behavior_director のモード) かつ
        「出さない状況ではない」(アンケート回答中・呼びかけ中/発話中・緊急停止・データが古い)

出す動き(director の `mode`):
    POLICY_GO_PIN       時間割・ピンの仕事の位置へ移動
    POLICY_GO_ENTRANCE  時間割(方針)で入口(ホーム)へ移動
    RETURN_HOME         接客のあとホーム・開始位置へ戻る
    MANUAL_OVERRIDE     手動運転(WASD・地図クリック。director が /nokosahen/manual_override から決めたモード)
    MAPPING_ACTIVE_HOLD 自律探索・地図作成(SLAM)中
上記以外(人・群衆への接近 APPROACH_PERSON、見回し CROWD_*、待機の揺動 IDLE_WIGGLE、壁から離れる動き、回復動作のみ など)は出さない。

画面の最短表示時間: いったん出したら、通常の解除(止まった・出さない動きに変わった)では MIN_SHOW_S 秒は出し続ける(ちらつき防止)。
ただし「嘘を出さない・人に見てもらいたい画面を優先する」ための即時解除(アンケート回答中・呼びかけ/発話中・緊急停止・
データが古い/取れない)にはこの保持を適用しない。音は保持なし(止まったらループを止める)。
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

# ---- しきい値(車輪が動いているか)。速さ[m/s]・秒 ----
MOVE_ON_MPS = 0.05        # この速さ以上が
MOVE_ON_HOLD_S = 0.5      # この秒数つづいたら「動いている」
MOVE_OFF_MPS = 0.02       # この速さ未満が
MOVE_OFF_HOLD_S = 1.5     # この秒数つづいたら「止まった」(間の速さは今の状態を保つ=ふらつきで点滅しない)
ODOM_STALE_S = 2.0        # オドメトリがこれより古い/無いときは「動いている」と言わない(表示用に10Hzで来る)
BEHAVIOR_STALE_S = 3.0    # director の指令がこれより古い/無いときは出さない(directorは約10Hz)
MIN_SHOW_S = 3.0          # 画面の最短表示時間
MAX_PLAUSIBLE_MPS = 5.0   # これを超える・NaN の速さは壊れた値として捨てる(動いている扱いにしない)

TITLE = "移動中です。ご注意ください"   # 画面の文言(音の文言は状況アナウンスの設定 `moving_notice` の text)

# 出す動き: director の mode -> 種類(画面・ログ用)
NOTICE_MODES: Dict[str, str] = {
    "POLICY_GO_PIN": "go_pin",
    "POLICY_GO_ENTRANCE": "go_entrance",
    "RETURN_HOME": "return_home",
    "MANUAL_OVERRIDE": "manual",
    "MAPPING_ACTIVE_HOLD": "exploring",
}
# 呼びかけ・お知らせ中のモード(人に画面を見てもらいたい。出さない+保持もしない)
ANNOUNCE_MODES = frozenset({"STOP_ANNOUNCE", "APPROACH_ARRIVED"})
# アンケート回答中のモード
SURVEY_MODES = frozenset({"SURVEY_ACTIVE_HOLD"})

# 出さない理由(reason)。空文字=出す
R_OK = ""
R_ODOM_STALE = "odom_stale"
R_BEHAVIOR_STALE = "behavior_stale"
R_ESTOP = "estop"
R_SURVEY = "survey"
R_SPEAKING = "speaking"
R_ANNOUNCE = "announce_mode"
R_DISPLAY_MODE = "display_mode"   # 【2026-10-30】iPadの表示モードが customer 以外(画面を出さない)→ 音も鳴らさない
R_MODE = "mode_not_listed"
R_NOT_MOVING = "not_moving"
# 保持(最短表示時間)を適用しない=すぐ消す理由
IMMEDIATE_REASONS = frozenset({R_ODOM_STALE, R_BEHAVIOR_STALE, R_ESTOP, R_SURVEY, R_SPEAKING, R_ANNOUNCE, R_DISPLAY_MODE})
CUSTOMER_MODE = "customer"   # iPadで「移動中」の画面が出る唯一の表示モード(DisplayHub._apply_moving と同じ)


NO_SOUND_KINDS = ("exploring",)   # 画面には出すが、音は鳴らさない動き(探索中は既存の「探索中です」が鳴る)


def _speed(v: Any) -> Optional[float]:
    """速さ[m/s]として使える値だけ返す(boolや文字・NaN・無限大・極端な値は None)。"""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    f = abs(float(v))
    if not math.isfinite(f) or f > MAX_PLAUSIBLE_MPS:
        return None
    return f


class MovingDetector:
    """オドメトリの速さから「車輪が動いている」を決める(ヒステリシス+最短保持)。時刻は引数。"""

    def __init__(self, on_mps: float = MOVE_ON_MPS, on_hold_s: float = MOVE_ON_HOLD_S,
                 off_mps: float = MOVE_OFF_MPS, off_hold_s: float = MOVE_OFF_HOLD_S) -> None:
        self.on_mps, self.on_hold_s = float(on_mps), float(on_hold_s)
        self.off_mps, self.off_hold_s = float(off_mps), float(off_hold_s)
        self.moving = False
        self._above_since: Optional[float] = None
        self._below_since: Optional[float] = None

    def reset(self) -> None:
        self.moving = False
        self._above_since = self._below_since = None

    def update(self, speed: Optional[float], now: float) -> bool:
        s = _speed(speed)
        if s is None:                 # 壊れた値は「動いている」の根拠にしない(止まった側に倒す)
            self.reset()
            return False
        if not self.moving:
            self._below_since = None
            if s >= self.on_mps:
                if self._above_since is None:
                    self._above_since = now
                if now - self._above_since >= self.on_hold_s:
                    self.moving, self._above_since = True, None
            else:
                self._above_since = None
        else:
            self._above_since = None
            if s < self.off_mps:
                if self._below_since is None:
                    self._below_since = now
                if now - self._below_since >= self.off_hold_s:
                    self.moving, self._below_since = False, None
            else:
                self._below_since = None
        return self.moving


def decide(moving: bool, behavior: Optional[Dict[str, Any]], behavior_age_s: Optional[float], odom_age_s: Optional[float],
           survey_answering: bool = False, speaking: bool = False,
           display_enabled: bool = False, display_mode: Optional[str] = None) -> Dict[str, Any]:
    """(ヒステリシス後の)moving と各入力から、出すか・理由・種類を決める(保持の前。状態を持たない純関数)。

    【2026-10-30】display_enabled=True(iPad表示を使う構成)のときは、iPadの表示モード display_mode が "customer" のときだけ出す
    (画面を出さないときは音も鳴らさない。モード不明=None・文字以外も出さない)。display_enabled=False(旧方式のラズパイ)は従来どおり。

    戻り値 {"notice": bool, "kind": str, "reason": str}。reason は出さない理由(出すときは "")。
    優先順: データが古い > 緊急停止 > アンケート > 呼びかけ/発話 > 出す動きか > 動いているか。"""
    b = behavior if isinstance(behavior, dict) else {}
    mode = b.get("mode") if isinstance(b.get("mode"), str) else ""
    if odom_age_s is None or odom_age_s > ODOM_STALE_S:
        return {"notice": False, "kind": "", "reason": R_ODOM_STALE}
    if not mode or behavior_age_s is None or behavior_age_s > BEHAVIOR_STALE_S:
        return {"notice": False, "kind": "", "reason": R_BEHAVIOR_STALE}
    if b.get("estop_active") or mode == "ESTOP_HOLD":
        return {"notice": False, "kind": "", "reason": R_ESTOP}
    if survey_answering or mode in SURVEY_MODES:
        return {"notice": False, "kind": "", "reason": R_SURVEY}
    if mode in ANNOUNCE_MODES:
        return {"notice": False, "kind": "", "reason": R_ANNOUNCE}
    if speaking:
        return {"notice": False, "kind": "", "reason": R_SPEAKING}
    if display_enabled and display_mode != CUSTOMER_MODE:
        return {"notice": False, "kind": "", "reason": R_DISPLAY_MODE}
    kind = NOTICE_MODES.get(mode)
    if kind is None:
        return {"notice": False, "kind": "", "reason": R_MODE}
    if not moving:
        return {"notice": False, "kind": kind, "reason": R_NOT_MOVING}
    return {"notice": True, "kind": kind, "reason": R_OK}


class MovingNotice:
    """入力(オドメトリ・director の指令)を受け、評価のたびに {"sound", "display", "kind", "reason", ...} を返す。

    sound: 音を鳴らしてよい(保持なし)。display: 画面に出す(最短表示時間 MIN_SHOW_S の保持つき)。"""

    def __init__(self, min_show_s: float = MIN_SHOW_S, detector: Optional[MovingDetector] = None) -> None:
        self.min_show_s = float(min_show_s)
        self.detector = detector or MovingDetector()
        self._speed_t: Optional[float] = None
        self._behavior: Dict[str, Any] = {}
        self._behavior_t: Optional[float] = None
        self._shown_since: Optional[float] = None   # 画面に出し始めた時刻
        self._last_kind = ""

    def on_odom(self, speed_mps: Any, now: float) -> None:
        try:
            self._speed_t = now
            self.detector.update(speed_mps, now)
        except Exception:  # noqa: BLE001  (通知の不具合でノードを止めない)
            self.detector.reset()

    def on_behavior(self, d: Any, now: float) -> None:
        if isinstance(d, dict):
            self._behavior, self._behavior_t = d, now

    def evaluate(self, now: float, survey_answering: bool = False, speaking: bool = False,
                 display_enabled: bool = False, display_mode: Optional[str] = None) -> Dict[str, Any]:
        odom_age = None if self._speed_t is None else max(0.0, now - self._speed_t)
        beh_age = None if self._behavior_t is None else max(0.0, now - self._behavior_t)
        if odom_age is None or odom_age > ODOM_STALE_S:
            self.detector.reset()      # 古いオドメトリの「動いている」を引きずらない
        try:
            r = decide(self.detector.moving, self._behavior, beh_age, odom_age, survey_answering, speaking,
                       display_enabled, display_mode)
        except Exception:  # noqa: BLE001
            r = {"notice": False, "kind": "", "reason": R_BEHAVIOR_STALE}
        notice = bool(r["notice"])
        display = notice
        # 【2026-10-27】自律探索中は、既存の「探索中です」(8秒おき)が鳴るので、「移動しています」の音は鳴らさない(画面の表示は出す。ユーザー指示)
        sound = notice and r["kind"] not in NO_SOUND_KINDS
        if notice:
            if self._shown_since is None:
                self._shown_since = now
            self._last_kind = r["kind"]
        elif self._shown_since is not None:
            held = now - self._shown_since < self.min_show_s
            if held and r["reason"] not in IMMEDIATE_REASONS:
                display = True         # ちらつき防止: 出してから MIN_SHOW_S 秒は消さない
            else:
                self._shown_since = None
        return {"sound": sound, "display": display, "kind": r["kind"] or (self._last_kind if display else ""),
                "reason": r["reason"] if not display else R_OK, "moving": self.detector.moving,
                "behavior_mode": self._behavior.get("mode") if beh_age is not None and beh_age <= BEHAVIOR_STALE_S else None}
