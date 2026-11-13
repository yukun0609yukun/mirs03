"""ビヘイビアツリー(BT)エンジンと木の検証(純Python、ROS非依存、外部ライブラリ無し)。

「どのタイミングで、どういう動作をするか」を木で組むための層。behavior_director.py から使われる。
【設計の要点】(詳細は docs/behavior_tree.md)
- 木は「方針(policy)」を選ぶだけで、ロボットを直接動かさない。木の出力は behavior_rules.Effects
  (ルール表と同じ型)で、ESTOP > MANUAL_OVERRIDE > SURVEY_ACTIVE_HOLD > NEAR_PERSON_HOLD >
  範囲外 > 立入禁止 > カメラ障害物 > MAPPING_ACTIVE_HOLD という固定の安全層(behavior_director._tick
  のコード側の分岐)の「下」にある非安全分岐にしか効かない。木の編集では安全層は外せない。
- 走行・ゴール送信は従来どおり director の出力ゲート(走行可能範囲のクランプ・立入禁止・ピンの範囲判定)
  を必ず通る。木のノードにcmd_velやゴールを直接出す手段は無い。
- tick駆動(directorの1Hzのルール周期に載せる)。RUNNINGのノードは次のtickも継続し、
  `for_seconds`や`wait`は「その間だけ方針を出し続ける」。メモリ付きSelector/Sequenceに対応。
- 木の定義はJSON(既定 ~/.nokosahen_behavior_tree.json)。ファイルが無い/不正/無効(enabled:false)なら
  従来のルール表だけの動作(既定の木=ルール表ノード1つ)で、挙動は変わらない。
- py_trees等は使わない(Jetsonにネットが無くインストールできない可能性があり、必要な機能も小さいため)。

検証CLI:  python -m nokosahen_core.behavior_tree validate <file>
"""

from __future__ import annotations

import datetime
import json
import math
import os
import re
import sys
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple

from . import behavior_rules as br

DEFAULT_TREE_FILE = "~/.nokosahen_behavior_tree.json"
MAX_NODES = 200
# スケジュール由来の木(source:"schedule")は、項目(最大50)ごとに条件・動作のノードが並ぶため上限が大きい
MAX_SCHEDULE_NODES = 800
MAX_DEPTH = 12
MAX_NAME_LEN = 60
MAX_FOR_SECONDS = 86400.0
DEFAULT_TREE_MIN_HOLD = 10.0
# 木のactivation番号はルール表単体の番号(小さい整数)と一致しないよう大きな値から始める
# (モード切替で「ゴール送信済み」の印が偶然一致して送り損ねるのを避ける)。
ACTIVATION_BASE = 1_000_000

SUCCESS = "SUCCESS"
FAILURE = "FAILURE"
RUNNING = "RUNNING"

# 固定の安全層(コード側、編集不可)。director._tickの分岐順と同じ。(キー, モード名, 日本語)
SAFETY_LAYERS: Tuple[Tuple[str, str, str], ...] = (
    ("estop", "ESTOP_HOLD", "非常停止(ハード/ソフト)"),
    ("manual", "MANUAL_OVERRIDE", "手動運転中"),
    ("survey", "SURVEY_ACTIVE_HOLD", "アンケート回答中"),
    ("near_person", "NEAR_PERSON_HOLD", "人が近すぎる"),
    ("out_of_bounds", "OUT_OF_BOUNDS_HOLD", "走行可能範囲の外"),
    ("keepout", "KEEPOUT_ZONE_HOLD", "立入禁止エリア内"),
    ("camera_obstacle", "CAMERA_OBSTACLE_HOLD", "カメラ障害物"),
    ("mapping", "MAPPING_ACTIVE_HOLD", "地図作成/探索中"),
)
SAFETY_KEYS = tuple(k for k, _m, _l in SAFETY_LAYERS)
BOUNDARY_RECT_KEYS = ("x_min", "y_min", "x_max", "y_max")

_TOP_KEYS = {"version", "description", "min_hold_seconds", "enabled", "root", "source", "run_on_startup"}
# 【2026-10-23】木の出どころ。"schedule"=スケジュール(docs/schedule.md)をコンパイルして作った木。
# この木はdirectorを起動した直後は止まっていて(run_on_startupがtrueのときだけ自動で実行)、
# /api/schedule/run の合図で「実行中(armed)」になる。ユーザーが木のページで編集した木(sourceなし)は従来どおり。
TREE_SOURCES = ("schedule",)
# set_boundaryの座標の上限(地図として現実的な範囲)
BOUNDARY_COORD_MAX_M = 10000.0
# set_displayの表示モード(頭部ディスプレイ。survey_admin_proxy.MODES と同じ並び)
DISPLAY_MODE_OPTIONS = (("customer", "お客さん向け"), ("survey", "アンケート"), ("ocrtest", "OCRテスト"))
DISPLAY_MODES = tuple(m for m, _l in DISPLAY_MODE_OPTIONS)
_NODE_KEYS = {"type", "name", "description", "enabled", "params", "children"}


class TreeError(ValueError):
    """木ファイルの検証エラー。"""


# --------------------------------------------------------------------------
# 実行時の受け渡し
# --------------------------------------------------------------------------


class Output:
    """木が出す方針の集計(ルール表のpolicy配列+paramsと同じ形)。後から出た方針が矛盾する先の方針を上書きする。"""

    def __init__(self) -> None:
        self.policies: List[str] = []
        self.params: Dict[str, Any] = {}
        # set_boundaryノードの出力: None=何も言わない(ダッシュボードで設定した元の範囲のまま) /
        # {"mode":"release"} / {"mode":"set","x_min":..,"y_min":..,"x_max":..,"y_max":..}。後に評価されたものが勝つ。
        self.boundary: Optional[Dict[str, Any]] = None
        # set_displayノードの出力: None=何も言わない(頭部ディスプレイの表示モードは変えない) / DISPLAY_MODESのどれか。後に評価されたものが勝つ。
        self.display: Optional[str] = None

    def add(self, policy: str, params: Optional[Dict[str, Any]] = None) -> None:
        if policy != "normal":
            for a, b in br._CONFLICTS:  # noqa: SLF001 (同一パッケージの定義をそのまま使う)
                other = b if policy == a else a if policy == b else None
                if other and other in self.policies:
                    self.policies.remove(other)
            if policy not in self.policies:
                self.policies.append(policy)
        if params:
            self.params.update(params)

    def final_policies(self) -> Tuple[str, ...]:
        return tuple(self.policies) if self.policies else ("normal",)


@dataclass
class Env:
    tick_no: int
    dt: datetime.datetime
    mono: float
    ctx: br.Context
    safety: Dict[str, bool]
    pin_ok: Callable[[str, Optional[str]], bool]
    rule_state: Optional[br.RuleState]
    out: Output
    trace: Dict[str, str]
    leaves: List[str]
    used_rules: bool = False


# --------------------------------------------------------------------------
# ノード
# --------------------------------------------------------------------------


class Node:
    TYPE = ""

    def __init__(self, nid: str, name: str, params: Dict[str, Any], children: List["Node"]) -> None:
        self.nid = nid
        self.name = name
        self.params = params
        self.children = children
        self._last_tick = -10

    def tick(self, env: Env) -> str:
        # 失敗した枝は方針を出さなかったものとして取り消す(「条件が成立しなかったのに途中の動作だけ残る」を防ぐ)。
        saved = (list(env.out.policies), dict(env.out.params), len(env.leaves), env.used_rules,
                 env.out.boundary, env.out.display)
        status = self._run(env)
        if status == FAILURE:
            env.out.policies[:] = saved[0]
            env.out.params.clear()
            env.out.params.update(saved[1])
            env.out.boundary = saved[4]
            env.out.display = saved[5]
            del env.leaves[saved[2]:]
            env.used_rules = saved[3]
        env.trace[self.nid] = status
        self._last_tick = env.tick_no
        return status

    def _continuing(self, env: Env) -> bool:
        """直前のtickでも評価されていたか(途切れたらRUNNINGの記憶を捨てる)。"""
        return self._last_tick == env.tick_no - 1

    def _run(self, env: Env) -> str:  # pragma: no cover
        raise NotImplementedError


class _Composite(Node):
    def __init__(self, *a: Any) -> None:
        super().__init__(*a)
        self._idx = 0

    @property
    def memory(self) -> bool:
        return bool(self.params.get("memory", False))

    def _start(self, env: Env) -> int:
        if self.memory and self._continuing(env):
            return self._idx
        return 0


class Selector(_Composite):
    """左の子から順に試し、最初にFAILUREでなかった子の結果を返す(Fallback)。"""

    TYPE = "selector"

    def _run(self, env: Env) -> str:
        i = self._start(env)
        while i < len(self.children):
            st = self.children[i].tick(env)
            if st != FAILURE:
                self._idx = i
                return st
            i += 1
        self._idx = 0
        return FAILURE


class Sequence(_Composite):
    """左の子から順に実行し、全部SUCCESSならSUCCESS。途中でFAILUREなら即FAILURE。"""

    TYPE = "sequence"

    def _run(self, env: Env) -> str:
        i = self._start(env)
        while i < len(self.children):
            st = self.children[i].tick(env)
            if st == RUNNING:
                self._idx = i
                return RUNNING
            if st == FAILURE:
                self._idx = 0
                return FAILURE
            i += 1
        self._idx = 0
        return SUCCESS


class Parallel(_Composite):
    """全ての子を毎tick評価する(出力は合成。後の子が優先)。success=all/any。"""

    TYPE = "parallel"

    def _run(self, env: Env) -> str:
        results = [c.tick(env) for c in self.children]
        if not results:
            return SUCCESS
        if self.params.get("success", "all") == "any":
            if SUCCESS in results:
                return SUCCESS
            return FAILURE if all(r == FAILURE for r in results) else RUNNING
        if FAILURE in results:
            return FAILURE
        return SUCCESS if all(r == SUCCESS for r in results) else RUNNING


class _Decorator(Node):
    @property
    def child(self) -> Node:
        return self.children[0]


class Inverter(_Decorator):
    TYPE = "inverter"

    def _run(self, env: Env) -> str:
        st = self.child.tick(env)
        return {SUCCESS: FAILURE, FAILURE: SUCCESS}.get(st, st)


class ForceSuccess(_Decorator):
    TYPE = "force_success"

    def _run(self, env: Env) -> str:
        st = self.child.tick(env)
        return RUNNING if st == RUNNING else SUCCESS


class ForceFailure(_Decorator):
    TYPE = "force_failure"

    def _run(self, env: Env) -> str:
        st = self.child.tick(env)
        return RUNNING if st == RUNNING else FAILURE


class Cooldown(_Decorator):
    """子がSUCCESSした後、指定秒数はFAILUREを返して子を評価しない(同じ動作の連発を防ぐ)。"""

    TYPE = "cooldown"

    def __init__(self, *a: Any) -> None:
        super().__init__(*a)
        self._ready_at = -math.inf

    def _run(self, env: Env) -> str:
        if env.mono < self._ready_at:
            return FAILURE
        st = self.child.tick(env)
        if st == SUCCESS:
            self._ready_at = env.mono + float(self.params["seconds"])
        return st


class Timeout(_Decorator):
    """子が連続してRUNNINGのまま指定秒数を超えたらFAILUREにする(連続して評価されている間はFAILUREのまま)。"""

    TYPE = "timeout"

    def __init__(self, *a: Any) -> None:
        super().__init__(*a)
        self._start_at: Optional[float] = None
        self._expired = False

    def _run(self, env: Env) -> str:
        if not self._continuing(env):
            self._start_at = None
            self._expired = False
        if self._expired:
            return FAILURE
        if self._start_at is not None and env.mono - self._start_at >= float(self.params["seconds"]):
            self._expired = True  # 時間切れ: 子は評価しない(方針を出さずに次の候補へ)
            return FAILURE
        st = self.child.tick(env)
        if st != RUNNING:
            self._start_at = None
            return st
        if self._start_at is None:
            self._start_at = env.mono
        return RUNNING


# --- 条件 -------------------------------------------------------------------


class Condition(Node):
    def _run(self, env: Env) -> str:
        return SUCCESS if self.check(env) else FAILURE

    def check(self, env: Env) -> bool:  # pragma: no cover
        raise NotImplementedError


class TimeBetween(Condition):
    TYPE = "time_between"

    def check(self, env: Env) -> bool:
        if br.clock_suspect(env.dt):  # 時計が合っていない(2026年より前)ときは、時刻の条件は成立しない(2026-10-21)
            return False
        s, e = br.parse_time_range(self.params["range"], "time_between")
        return br._in_range(env.dt.hour * 60 + env.dt.minute, s, e)  # noqa: SLF001


class WeekdayIn(Condition):
    TYPE = "weekday_in"

    def check(self, env: Env) -> bool:
        if br.clock_suspect(env.dt):
            return False
        return br.WEEKDAYS[env.dt.weekday()] in self.params["days"]


class DateIn(Condition):
    TYPE = "date_in"

    def check(self, env: Env) -> bool:
        if br.clock_suspect(env.dt):
            return False
        return env.dt.strftime("%Y-%m-%d") in self.params["dates"]


class CrowdIs(Condition):
    TYPE = "crowd_is"

    def check(self, env: Env) -> bool:
        return env.ctx.crowd_level in self.params["levels"]


class PersonPresent(Condition):
    TYPE = "person_present"

    def check(self, env: Env) -> bool:
        want = bool(self.params.get("value", True))
        return env.ctx.person_present is not None and env.ctx.person_present == want


class StockRatio(Condition):
    TYPE = "stock_ratio"

    def check(self, env: Env) -> bool:
        ratio = env.ctx.ratio_for(self.params.get("menu") or None)
        if ratio is None:
            return False
        v = float(self.params["value"])
        return ratio < v if self.params.get("op", "below") == "below" else ratio > v


class IdleFor(Condition):
    TYPE = "idle_for"

    def check(self, env: Env) -> bool:
        idle = env.ctx.idle_seconds
        return idle is not None and idle > float(self.params["seconds"])


class PinAvailable(Condition):
    TYPE = "pin_available"

    def check(self, env: Env) -> bool:
        return bool(env.pin_ok(self.params["pin_id"], self.params.get("pin_map")))


ROBOT_STATES = tuple(SAFETY_KEYS) + ("any_safety",)


class RobotState(Condition):
    """ロボットの状態を読むだけ(安全層そのものは木の外でコードが判定する)。"""

    TYPE = "robot_state"

    def check(self, env: Env) -> bool:
        state = self.params["state"]
        if state == "any_safety":
            return any(env.safety.get(k, False) for k in SAFETY_KEYS)
        return bool(env.safety.get(state, False))


# --- 動作(方針の出力) -------------------------------------------------------


class Action(Node):
    """方針を出力する葉。for_seconds>0なら、その秒数の間RUNNINGで方針を出し続け、過ぎたらSUCCESS。
    for_secondsが無い/0なら、評価された間だけ方針を出してSUCCESS(レベル出力)。"""

    def __init__(self, *a: Any) -> None:
        super().__init__(*a)
        self._start_at: Optional[float] = None
        self._done = False

    def duration(self) -> float:
        return float(self.params.get("for_seconds", 0.0) or 0.0)

    def emit(self, env: Env) -> None:  # pragma: no cover
        raise NotImplementedError

    def _run(self, env: Env) -> str:
        d = self.duration()
        if d <= 0:
            self.emit(env)
            env.leaves.append(self.nid)
            return SUCCESS
        if not self._continuing(env):
            self._start_at = None
            self._done = False
        if self._done:
            return SUCCESS
        if self._start_at is None:
            self._start_at = env.mono
        if env.mono - self._start_at >= d:
            self._done = True
            return SUCCESS
        self.emit(env)
        env.leaves.append(self.nid)
        return RUNNING


class PolicyAction(Action):
    """固定の方針名を出す動作。"""

    POLICY = "normal"

    def policy_and_params(self) -> Tuple[str, Dict[str, Any]]:
        return self.POLICY, {}

    def emit(self, env: Env) -> None:
        policy, params = self.policy_and_params()
        env.out.add(policy, params)


class HoldStop(PolicyAction):
    TYPE = "hold_stop"
    POLICY = "stay"


class GoToEntrance(PolicyAction):
    TYPE = "go_to_entrance"
    POLICY = "go_to_entrance"


class ReturnHome(PolicyAction):
    TYPE = "return_home"
    POLICY = "return_home"


class Quiet(PolicyAction):
    TYPE = "quiet"
    POLICY = "quiet"


class NormalAction(PolicyAction):
    TYPE = "normal"
    POLICY = "normal"


class GotoPin(PolicyAction):
    TYPE = "goto_pin"

    def policy_and_params(self) -> Tuple[str, Dict[str, Any]]:
        p = {k: self.params[k] for k in ("pin_id", "pin_name", "pin_map") if k in self.params}
        return "goto_pin", p


class Wait(PolicyAction):
    """その場で止まって待つ(発話は続く)。seconds経ったらSUCCESS。"""

    TYPE = "wait"
    POLICY = "stay"

    def duration(self) -> float:
        return float(self.params["seconds"])


class ApproachAction(PolicyAction):
    TYPE = "approach"

    def policy_and_params(self) -> Tuple[str, Dict[str, Any]]:
        return ("approach_enabled" if self.params.get("enabled", True) else "approach_disabled"), {}


class ScanAction(PolicyAction):
    TYPE = "scan"

    def policy_and_params(self) -> Tuple[str, Dict[str, Any]]:
        return ("crowd_scan_enabled" if self.params.get("enabled", True) else "crowd_scan_disabled"), {}


class FaceAction(PolicyAction):
    """その場で人の方を向く動作の許可/禁止(2026-11-03)。"""

    TYPE = "face"

    def policy_and_params(self) -> Tuple[str, Dict[str, Any]]:
        return ("face_enabled" if self.params.get("enabled", True) else "face_disabled"), {}


class WiggleAction(PolicyAction):
    TYPE = "wiggle"

    def policy_and_params(self) -> Tuple[str, Dict[str, Any]]:
        return ("idle_wiggle_on" if self.params.get("enabled", True) else "idle_wiggle_off"), {}


class SpeedScaleAction(PolicyAction):
    TYPE = "speed_scale"

    def policy_and_params(self) -> Tuple[str, Dict[str, Any]]:
        return "speed_scale", {"scale": float(self.params["scale"])}


class AnnounceAction(PolicyAction):
    """呼びかけ(発話)の最小間隔を短く/長くする。"""

    TYPE = "announce"

    def policy_and_params(self) -> Tuple[str, Dict[str, Any]]:
        p: Dict[str, Any] = {"announce_min_interval_seconds": float(self.params["interval_seconds"])}
        if self.params.get("after_facing", False):
            p["announce_after_facing"] = True  # 向く動作が終わってから依頼を出す(2026-11-03)
        return "normal", p


class SetBoundary(Action):
    """走行できる範囲を、この動作が評価されている間だけ差し替える/解除する(2026-10-23、docs/schedule.md)。
    木はcmd_velもゴールも出さない原則のまま、範囲の「希望」を出すだけ。directorが専用トピック
    /nokosahen/boundary_override へ配信し、ダッシュボードのBoundaryControllerが唯一の発行元として
    /nokosahen/travel_boundary へ反映する。評価されなくなれば(項目の時間が終われば)元の範囲へ戻る。
    解除でも、衝突監視・立入禁止・緊急停止・人が近すぎる等の安全層は常に有効。"""

    TYPE = "set_boundary"

    def emit(self, env: Env) -> None:
        if self.params["mode"] == "release":
            env.out.boundary = {"mode": "release"}
        else:
            env.out.boundary = {"mode": "set", **{k: float(self.params[k]) for k in BOUNDARY_RECT_KEYS}}


class SetDisplay(Action):
    """頭部ディスプレイ(頭部ディスプレイの画面)に表示してほしいモードを、この動作が評価されている間だけ要求する(docs/schedule.md)。
    木はラズパイを直接操作しない原則のまま、表示モードの「希望」を出すだけ。directorが専用トピック
    /nokosahen/display_request へ配信し、ダッシュボード側のDisplayScheduleControllerが、アンケート回答中でないことを
    確かめてから既存の切替経路で反映する。評価されなくなれば(項目が終われば)ダッシュボード側が元のモードへ戻す。"""

    TYPE = "set_display"

    def emit(self, env: Env) -> None:
        env.out.display = str(self.params["mode"])


class SetPolicy(PolicyAction):
    """ルール表と同じ方針名を直接指定する上級者向けの動作。"""

    TYPE = "set_policy"

    def policy_and_params(self) -> Tuple[str, Dict[str, Any]]:
        keys = ("scale", "announce_min_interval_seconds", "pin_id", "pin_name", "pin_map")
        return self.params["policy"], {k: self.params[k] for k in keys if k in self.params}


class RulesTable(Node):
    """既存のルール表(behavior_rules)をそのまま木の一部として使う葉。
    有効なルールがあればその方針を出してSUCCESS、無ければFAILURE。"""

    TYPE = "rules_table"

    def _run(self, env: Env) -> str:
        st = env.rule_state
        if st is None or st.active_rule_id is None:
            return FAILURE
        for p in st.policies:
            env.out.add(p, None)
        env.out.params.update(st.params)
        env.used_rules = True
        env.leaves.append(self.nid)
        return SUCCESS


NODE_CLASSES: Dict[str, type] = {
    c.TYPE: c
    for c in (
        Selector, Sequence, Parallel, Inverter, ForceSuccess, ForceFailure, Cooldown, Timeout,
        TimeBetween, WeekdayIn, DateIn, CrowdIs, PersonPresent, StockRatio, IdleFor, PinAvailable,
        RobotState, HoldStop, GoToEntrance, ReturnHome, Quiet, NormalAction, GotoPin, Wait,
        ApproachAction, ScanAction, FaceAction, WiggleAction, SpeedScaleAction, AnnounceAction, SetBoundary,
        SetDisplay, SetPolicy, RulesTable,
    )
}


# --------------------------------------------------------------------------
# ノード種別の定義(検証と編集画面のドロップダウンの元)
# --------------------------------------------------------------------------


def _p(key: str, label: str, typ: str, **kw: Any) -> Dict[str, Any]:
    d: Dict[str, Any] = {"key": key, "label": label, "type": typ}
    d.update(kw)
    return d


_FOR = _p("for_seconds", "続ける秒数(空=評価中だけ)", "number", min=0, max=MAX_FOR_SECONDS,
          hint="指定すると、その秒数の間この動作を続け(RUNNING)、過ぎたら成功して次へ進みます")
_PIN = [
    _p("pin_id", "行き先ピン", "pin", required=True),
    _p("pin_name", "ピン名(表示用)", "hidden"),
    _p("pin_map", "ピンの地図名", "hidden"),
]
_WEEKDAY_LABELS = {"mon": "月", "tue": "火", "wed": "水", "thu": "木", "fri": "金", "sat": "土", "sun": "日"}
_CROWD_LABELS = {"calm": "空いている", "crowded": "混雑", "unknown": "不明"}
_STATE_LABELS = {
    "estop": "非常停止中", "manual": "手動運転中", "survey": "アンケート回答中",
    "near_person": "人が近すぎる", "out_of_bounds": "走行範囲の外", "keepout": "立入禁止エリア内",
    "camera_obstacle": "カメラ障害物あり", "mapping": "地図作成/探索中",
    "any_safety": "いずれかの安全停止中",
}
_POLICY_LABELS = {
    "normal": "通常", "stay": "待機", "announce_only": "案内のみ", "approach_enabled": "接近を許可",
    "approach_disabled": "接近しない", "crowd_scan_enabled": "見回しを許可", "crowd_scan_disabled": "見回さない",
    "face_enabled": "人の方を向くのを許可", "face_disabled": "人の方を向かない",
    "idle_wiggle_on": "左右に揺れる", "idle_wiggle_off": "揺れない", "go_to_entrance": "入口へ移動",
    "return_home": "ホームへ帰還", "goto_pin": "地図のピンへ移動", "quiet": "静か(発話なし)",
    "speed_scale": "速度を落とす",
}


def _opts(values: Tuple[str, ...], labels: Dict[str, str]) -> List[List[str]]:
    return [[v, labels.get(v, v)] for v in values]


# type -> 定義。kind: composite / decorator / condition / action / leaf
NODE_SPECS: Dict[str, Dict[str, Any]] = {
    "selector": dict(kind="composite", label="セレクタ(上から試す)", icon="?", children="many",
                     desc="上の子から順に試し、最初に失敗しなかった子を採用します(if / else if / else)",
                     params=[_p("memory", "実行中の子を覚える", "bool", default=False)]),
    "sequence": dict(kind="composite", label="シーケンス(順に実行)", icon="→", children="many",
                     desc="上の子から順に実行し、全部成功したら成功。途中で失敗したらそこで失敗",
                     params=[_p("memory", "実行中の子を覚える", "bool", default=False,
                                hint="オンにすると、続けている動作(秒数指定)を飛ばさず順に進めます")]),
    "parallel": dict(kind="composite", label="パラレル(同時に評価)", icon="⇉", children="many",
                     desc="全部の子を毎回評価し、方針を合成します(後ろの子が優先)",
                     params=[_p("success", "成功の条件", "enum", default="all",
                                options=[["all", "全部成功"], ["any", "どれか成功"]])]),
    "inverter": dict(kind="decorator", label="反転", icon="!", children="one",
                     desc="子の成功と失敗を入れ替えます(条件の否定に)", params=[]),
    "force_success": dict(kind="decorator", label="常に成功", icon="✓", children="one",
                          desc="子が失敗しても成功として扱います(おまけの動作に)", params=[]),
    "force_failure": dict(kind="decorator", label="常に失敗", icon="✗", children="one",
                          desc="子が成功しても失敗として扱います(出力だけ使いたい時に)", params=[]),
    "cooldown": dict(kind="decorator", label="クールダウン", icon="⏳", children="one",
                     desc="子が成功した後、指定秒数は失敗にして子を評価しません(連発防止)",
                     params=[_p("seconds", "秒", "number", required=True, min=1, max=86400)]),
    "timeout": dict(kind="decorator", label="タイムアウト", icon="⏱", children="one",
                    desc="子が続いたまま指定秒数を超えたら失敗にします(次の候補へ切り替え)",
                    params=[_p("seconds", "秒", "number", required=True, min=1, max=86400)]),
    "time_between": dict(kind="condition", label="時間帯", icon="🕒",
                         desc="現在時刻が範囲内か(日付またぎ可。例 22:00-06:00)",
                         params=[_p("range", "時間帯(HH:MM-HH:MM)", "time_range", required=True)]),
    "weekday_in": dict(kind="condition", label="曜日", icon="📅", desc="今日の曜日が選んだ曜日のどれかか",
                       params=[_p("days", "曜日", "multi", required=True,
                                  options=_opts(br.WEEKDAYS, _WEEKDAY_LABELS))]),
    "date_in": dict(kind="condition", label="日付", icon="📆", desc="今日の日付が指定日のどれかか",
                    params=[_p("dates", "日付(YYYY-MM-DD、複数はカンマ区切り)", "dates", required=True)]),
    "crowd_is": dict(kind="condition", label="混雑度", icon="👥", desc="混雑度が選んだ値のどれかか",
                     params=[_p("levels", "混雑度", "multi", required=True,
                                options=_opts(br.CROWD_LEVELS, _CROWD_LABELS))]),
    "person_present": dict(kind="condition", label="人物の有無", icon="🙋", desc="視界に人物が(いる/いない)",
                           params=[_p("value", "人物がいる", "bool", default=True)]),
    "stock_ratio": dict(kind="condition", label="残数の割合", icon="🍱",
                        desc="メニューの残数比(0〜1)がしきい値より(少ない/多い)。メニュー未指定は最も少ないもの",
                        params=[_p("op", "比較", "enum", default="below",
                                   options=[["below", "より少ない"], ["above", "より多い"]]),
                                _p("value", "残数比(0〜1)", "number", required=True, min=0, max=1),
                                _p("menu", "メニュー名(空=最も少ないもの)", "text", maxlen=100)]),
    "idle_for": dict(kind="condition", label="人物不在の時間", icon="💤",
                     desc="人物が見えない状態が指定秒数より長く続いているか",
                     params=[_p("seconds", "秒", "number", required=True, min=0, max=86400)]),
    "pin_available": dict(kind="condition", label="ピンが使える", icon="📍",
                          desc="指定のピンが現在の地図にあり、行ける状態か", params=list(_PIN)),
    "robot_state": dict(kind="condition", label="ロボットの状態", icon="🛡",
                        desc="安全状態などを読むだけの条件です(安全停止そのものは木の外で常に優先されます)",
                        params=[_p("state", "状態", "enum", required=True,
                                   options=_opts(ROBOT_STATES, _STATE_LABELS))]),
    "hold_stop": dict(kind="action", label="停止して待機", icon="■",
                      desc="その場で止まって待ちます(新しい走行指令なし、実行中のゴールは取り消し。発話は続く)",
                      params=[_FOR]),
    "wait": dict(kind="action", label="待つ(秒数)", icon="⏸",
                 desc="その場で止まって指定秒数待ち、過ぎたら成功して次へ進みます",
                 params=[_p("seconds", "秒", "number", required=True, min=1, max=MAX_FOR_SECONDS)]),
    "go_to_entrance": dict(kind="action", label="入口へ移動", icon="🚪",
                           desc="既存の入口ゴールへ1回移動し、到着したら待機します(範囲・立入禁止の判定込み)",
                           params=[_FOR]),
    "return_home": dict(kind="action", label="ホームへ帰還", icon="🏠",
                        desc="入口(ホーム)へ戻って待機します", params=[_FOR]),
    "goto_pin": dict(kind="action", label="ピンへ移動", icon="📍",
                     desc="地図ピンの位置・向きへ1回移動して待機します。ピンが使えない時はその場で待機",
                     params=list(_PIN) + [_FOR]),
    "approach": dict(kind="action", label="接近の許可/禁止", icon="🚶",
                     desc="人物へ近づく動作を許可/禁止します",
                     params=[_p("enabled", "接近を許可", "bool", default=True), _FOR]),
    "scan": dict(kind="action", label="見回しの許可/禁止", icon="👀",
                 desc="その場での見回し旋回を許可/禁止します",
                 params=[_p("enabled", "見回しを許可", "bool", default=True), _FOR]),
    "face": dict(kind="action", label="人の方を向く(許可/禁止)", icon="🧭",
                 desc="その場で旋回して、人の多い方を向く動作を許可/禁止します(前には進みません)",
                 params=[_p("enabled", "人の方を向くのを許可", "bool", default=True), _FOR]),
    "wiggle": dict(kind="action", label="左右に揺れる", icon="↔",
                   desc="待機中に左右へ揺れる動作の入/切",
                   params=[_p("enabled", "揺れる", "bool", default=True), _FOR]),
    "speed_scale": dict(kind="action", label="速度を落とす", icon="🐢",
                        desc="自律の速度上限を係数で下げます(現状より速くはなりません)",
                        params=[_p("scale", "係数", "number", required=True,
                                   min=br.SPEED_SCALE_MIN, max=br.SPEED_SCALE_MAX), _FOR]),
    "announce": dict(kind="action", label="呼びかけ間隔", icon="📣",
                     desc="呼びかけ(発話)の最小間隔を変えます(安全停止中は適用されません)",
                     params=[_p("interval_seconds", "最小間隔(秒)", "number", required=True,
                                min=br.ANNOUNCE_INTERVAL_MIN, max=br.ANNOUNCE_INTERVAL_MAX),
                                _p("after_facing", "向き終わってから呼びかける", "bool", default=False,
                                   hint="オンにすると、人の方を向く動作が終わってから(誤差5度以内)呼びかけの依頼を出します"),
                                _FOR]),
    "quiet": dict(kind="action", label="静かにする", icon="🔇", desc="呼びかけ(発話)をしません", params=[_FOR]),
    "normal": dict(kind="action", label="通常動作", icon="●",
                   desc="何も変えず、従来どおりの自律動作(接近・見回し等)を行います", params=[_FOR]),
    "set_boundary": dict(kind="action", label="走行範囲の差し替え/解除", icon="🧱",
                         desc="この動作が評価されている間だけ、走行できる範囲を指定の矩形に差し替える/範囲の制限をなくす"
                              "(終われば元の範囲へ戻る)。解除しても衝突監視・立入禁止・緊急停止などの安全層は常に有効です",
                         params=[_p("mode", "方法", "enum", required=True,
                                    options=[["set", "この範囲にする"], ["release", "範囲の制限をなくす"]]),
                                 _p("x_min", "x最小[m](map座標)", "number", min=-BOUNDARY_COORD_MAX_M, max=BOUNDARY_COORD_MAX_M),
                                 _p("y_min", "y最小[m]", "number", min=-BOUNDARY_COORD_MAX_M, max=BOUNDARY_COORD_MAX_M),
                                 _p("x_max", "x最大[m]", "number", min=-BOUNDARY_COORD_MAX_M, max=BOUNDARY_COORD_MAX_M),
                                 _p("y_max", "y最大[m]", "number", min=-BOUNDARY_COORD_MAX_M, max=BOUNDARY_COORD_MAX_M),
                                 _FOR]),
    "set_display": dict(kind="action", label="頭部ディスプレイの画面の切替", icon="🖥",
                        desc="この動作が評価されている間だけ、頭部ディスプレイ(頭部ディスプレイの画面)を指定のモードにするよう要求する"
                             "(終われば元のモードへ戻る)。アンケート回答中は回答が消えないよう、終わるまで切り替えません。"
                             "ロボットは直接操作せず、ダッシュボードが既存の切替で反映します",
                        params=[_p("mode", "表示モード", "enum", required=True,
                                   options=[list(o) for o in DISPLAY_MODE_OPTIONS]),
                                _FOR]),
    "set_policy": dict(kind="action", label="方針を直接指定(上級)", icon="⚙",
                       desc="ルール表と同じ方針名を直接指定します(scale / 間隔 / ピンは方針に応じて必須)",
                       params=[_p("policy", "方針", "enum", required=True,
                                  options=_opts(br.POLICIES, _POLICY_LABELS)),
                               _p("scale", "係数(speed_scale用)", "number",
                                  min=br.SPEED_SCALE_MIN, max=br.SPEED_SCALE_MAX),
                               _p("announce_min_interval_seconds", "呼びかけ間隔(秒)", "number",
                                  min=br.ANNOUNCE_INTERVAL_MIN, max=br.ANNOUNCE_INTERVAL_MAX),
                               _p("pin_id", "ピン(goto_pin用)", "pin"),
                               _p("pin_name", "ピン名", "hidden"), _p("pin_map", "ピンの地図名", "hidden"),
                               _FOR]),
    "rules_table": dict(kind="leaf", label="ルール表(時間帯・状況ルール)", icon="📋",
                        desc="既存の時間帯・状況ルール表(/behavior_rules)をそのまま使います。"
                             "有効なルールがあれば成功、無ければ失敗",
                        params=[]),
}
# 子の数: many=0以上 / one=ちょうど1 / none=葉
for _t, _s in NODE_SPECS.items():
    _s.setdefault("children", "none")
assert set(NODE_SPECS) == set(NODE_CLASSES), set(NODE_SPECS) ^ set(NODE_CLASSES)

_ID_RE = br._PIN_ID_RE  # noqa: SLF001
_MAP_RE = br._PIN_MAP_RE  # noqa: SLF001
_DATE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")


def registry_meta() -> Dict[str, Any]:
    """編集画面用: ノード種別・パラメータ定義・上限・安全層(コード側の定義をそのまま渡す)。"""
    kinds = [
        ["composite", "制御(分岐・順序)"], ["decorator", "加工(1つの子を包む)"],
        ["condition", "条件(いつ・どんな時)"], ["action", "動作(何をする)"], ["leaf", "ルール表"],
    ]
    return {
        "kinds": kinds,
        "types": [dict(type=t, **s) for t, s in NODE_SPECS.items()],
        "max_nodes": MAX_NODES,
        "max_depth": MAX_DEPTH,
        "default_min_hold_seconds": DEFAULT_TREE_MIN_HOLD,
        "safety_layers": [{"key": k, "mode": m, "label": lab} for k, m, lab in SAFETY_LAYERS],
        "policies": list(br.POLICIES),
    }


# --------------------------------------------------------------------------
# 検証(厳格: 1つでも不正ならファイル全体を拒否)
# --------------------------------------------------------------------------


def _is_num(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _clean_param(spec: Dict[str, Any], value: Any, where: str) -> Any:
    typ = spec["type"]
    key = spec["key"]
    at = f"{where}.params.{key}"
    if typ == "number":
        if not _is_num(value):
            raise TreeError(f"{at}: 数値にしてください: {value!r}")
        lo, hi = spec.get("min"), spec.get("max")
        if (lo is not None and value < lo) or (hi is not None and value > hi):
            raise TreeError(f"{at}: {lo}〜{hi}の数値にしてください: {value!r}")
        return float(value)
    if typ == "bool":
        if not isinstance(value, bool):
            raise TreeError(f"{at}: true/falseにしてください: {value!r}")
        return value
    if typ in ("text", "hidden"):
        if not isinstance(value, str):
            raise TreeError(f"{at}: 文字列にしてください: {value!r}")
        maxlen = spec.get("maxlen", 100)
        if key == "pin_name":
            maxlen = br.PIN_NAME_MAX_LEN
        if len(value) > maxlen:
            raise TreeError(f"{at}: {maxlen}文字以内にしてください")
        if key == "pin_map" and not _MAP_RE.match(value):
            raise TreeError(f"{at}: 地図名(英数字/_/-の1〜64文字)にしてください: {value!r}")
        return value.strip() if typ == "text" else value
    if typ == "enum":
        allowed = [o[0] for o in spec["options"]]
        if value not in allowed:
            raise TreeError(f"{at}: 不明な値 {value!r}(使える値: {allowed})")
        return value
    if typ == "multi":
        allowed = [o[0] for o in spec["options"]]
        if not isinstance(value, list) or not value or not all(isinstance(x, str) for x in value):
            raise TreeError(f"{at}: 空でない文字列の配列にしてください")
        for x in value:
            if x not in allowed:
                raise TreeError(f"{at}: 不明な値 {x!r}(使える値: {allowed})")
        return [x for x in allowed if x in value]  # 重複除去+定義順
    if typ == "time_range":
        br.parse_time_range(value, at)
        return value.strip()
    if typ == "dates":
        if not isinstance(value, list) or not value or not all(isinstance(x, str) for x in value):
            raise TreeError(f"{at}: 'YYYY-MM-DD'の配列にしてください")
        for d in value:
            if not _DATE_RE.match(d):
                raise TreeError(f"{at}: 'YYYY-MM-DD'形式にしてください: {d!r}")
            try:
                datetime.date.fromisoformat(d)
            except ValueError:
                raise TreeError(f"{at}: 存在しない日付です: {d!r}")
        return list(value)
    if typ == "pin":
        if not isinstance(value, str) or not _ID_RE.match(value):
            raise TreeError(f"{at}: ピンのid(英数字/_/-の1〜32文字)を指定してください: {value!r}")
        return value
    raise TreeError(f"{at}: 内部エラー(未対応の型 {typ})")  # pragma: no cover


def _validate_node(raw: Any, nid: str, depth: int, counter: List[int], warnings: List[str]) -> dict:
    if depth > MAX_DEPTH:
        raise TreeError(f"木が深すぎます(最大{MAX_DEPTH}階層): ノード{nid}")
    counter[0] += 1
    if counter[0] > counter[1]:
        raise TreeError(f"ノードが多すぎます(最大{counter[1]}個)")
    if not isinstance(raw, dict):
        raise TreeError(f"ノード{nid}: オブジェクトにしてください")
    unknown = sorted(set(raw) - _NODE_KEYS)
    if unknown:
        raise TreeError(f"ノード{nid}: 未知のキー {unknown}(使えるキー: {sorted(_NODE_KEYS)})")
    typ = raw.get("type")
    if not isinstance(typ, str) or typ not in NODE_SPECS:
        raise TreeError(f"ノード{nid}: 未知のノード種別 {typ!r}(使える種別: {sorted(NODE_SPECS)})")
    spec = NODE_SPECS[typ]
    where = f"ノード{nid}({typ})"
    name = raw.get("name", "")
    if not isinstance(name, str) or len(name) > MAX_NAME_LEN:
        raise TreeError(f"{where}.name: {MAX_NAME_LEN}文字以内の文字列にしてください")
    desc = raw.get("description", "")
    if not isinstance(desc, str) or len(desc) > 300:
        raise TreeError(f"{where}.description: 300文字以内の文字列にしてください")
    enabled = raw.get("enabled", True)
    if not isinstance(enabled, bool):
        raise TreeError(f"{where}.enabled: true/falseにしてください")

    params_raw = raw.get("params", {})
    if not isinstance(params_raw, dict):
        raise TreeError(f"{where}.params: オブジェクトにしてください")
    specs = {s["key"]: s for s in spec["params"]}
    unknown = sorted(set(params_raw) - set(specs))
    if unknown:
        raise TreeError(f"{where}.params: 未知のパラメータ {unknown}(使えるもの: {sorted(specs)})")
    params: Dict[str, Any] = {}
    for key, s in specs.items():
        if key in params_raw and params_raw[key] is not None and not (
            s["type"] in ("text", "hidden") and params_raw[key] == "" and not s.get("required")
        ):
            params[key] = _clean_param(s, params_raw[key], where)
        elif s.get("required"):
            raise TreeError(f"{where}.params.{key}: 必須です({s['label']})")
    if typ == "set_policy":
        pol = params["policy"]
        if pol == "speed_scale" and "scale" not in params:
            raise TreeError(f"{where}.params.scale: speed_scaleにはscaleが必須です")
        if pol == "goto_pin" and "pin_id" not in params:
            raise TreeError(f"{where}.params.pin_id: goto_pinにはpin_idが必須です")
        if pol != "speed_scale" and "scale" in params:
            raise TreeError(f"{where}.params.scale: policyにspeed_scaleが必要です")
        if pol != "goto_pin" and any(k in params for k in ("pin_id", "pin_name", "pin_map")):
            raise TreeError(f"{where}.params.pin_id: policyにgoto_pinが必要です")
    if typ == "set_boundary":
        if params["mode"] == "set":
            missing = [k for k in BOUNDARY_RECT_KEYS if k not in params]
            if missing:
                raise TreeError(f"{where}.params: modeがsetのときは {missing} が必須です")
            if params["x_min"] >= params["x_max"] or params["y_min"] >= params["y_max"]:
                raise TreeError(f"{where}.params: 範囲が不正です(x_min<x_max かつ y_min<y_max にしてください)")
        elif any(k in params for k in BOUNDARY_RECT_KEYS):
            raise TreeError(f"{where}.params: modeがreleaseのときは範囲(x_min等)を指定できません")
    if "pin_id" not in params and any(k in params for k in ("pin_name", "pin_map")):
        raise TreeError(f"{where}.params.pin_id: pin_name/pin_mapにはpin_idが必要です")

    kids_raw = raw.get("children", [])
    if not isinstance(kids_raw, list):
        raise TreeError(f"{where}.children: 配列にしてください")
    arity = spec["children"]
    if arity == "none" and kids_raw:
        raise TreeError(f"{where}: この種別は子を持てません")
    if arity == "one" and len(kids_raw) != 1:
        raise TreeError(f"{where}: 子はちょうど1つ必要です(いま{len(kids_raw)}個)")
    kids = [_validate_node(k, f"{nid}.{i}", depth + 1, counter, warnings) for i, k in enumerate(kids_raw)]
    if arity == "one" and not kids[0].get("enabled", True):
        raise TreeError(f"{where}: 子が無効になっています(子を有効にするか、このノードも無効にしてください)")

    out: Dict[str, Any] = {"type": typ}
    if name:
        out["name"] = name
    if desc:
        out["description"] = desc
    if not enabled:
        out["enabled"] = False
    if params:
        out["params"] = params
    if arity != "none":
        out["children"] = kids
    if enabled:
        _node_warnings(typ, out, kids, nid, warnings)
    return out


def _node_warnings(typ: str, node: dict, kids: List[dict], nid: str, warnings: List[str]) -> None:
    label = f"ノード{nid}「{node.get('name') or NODE_SPECS[typ]['label']}」"
    live = [k for k in kids if k.get("enabled", True)]
    if typ in ("selector", "sequence", "parallel") and not live:
        warnings.append(f"{label}: 子が1つも有効ではありません(常に{'失敗' if typ == 'selector' else '成功'}します)")
    if typ == "selector" and not node.get("params", {}).get("memory"):
        for i, k in enumerate(live[:-1]):
            if _always_succeeds(k):
                warnings.append(
                    f"{label}: {i + 1}番目の子は常に成功するため、それより下の子は実行されません"
                )
                break
    if typ == "sequence":
        for i, k in enumerate(live[:-1]):
            if _always_fails(k):
                warnings.append(f"{label}: {i + 1}番目の子は常に失敗するため、それより下の子は実行されません")
                break


def _always_succeeds(n: dict) -> bool:
    t = n["type"]
    if NODE_SPECS[t]["kind"] == "action":
        return not (n.get("params", {}).get("for_seconds") or t == "wait")
    if t == "force_success":
        return True
    return False


def _always_fails(n: dict) -> bool:
    return n["type"] == "force_failure"


def validate_document(doc: Any) -> Tuple[List[str], List[str], Optional[dict]]:
    """木の文書(dict)を検証する。(エラー一覧[日本語], 警告一覧, 正規化した文書)。
    エラーが1つでもあれば文書はNone(保存しない)。JSONは木構造なので循環参照は書けない
    (idでの参照を許さず、深さ/個数の上限で巨大な木も拒否する)。"""
    warnings: List[str] = []
    if not isinstance(doc, dict):
        return ['トップレベルはオブジェクト({"root": {...}})にしてください'], warnings, None
    try:
        unknown = sorted(set(doc) - _TOP_KEYS)
        if unknown:
            raise TreeError(f"トップレベル: 未知のキー {unknown}(使えるキー: {sorted(_TOP_KEYS)})")
        version = doc.get("version", 1)
        if version != 1 or isinstance(version, bool):
            raise TreeError(f"versionは1のみ対応です: {version!r}")
        hold = doc.get("min_hold_seconds", DEFAULT_TREE_MIN_HOLD)
        if not _is_num(hold) or not 0 <= hold <= 600:
            raise TreeError(f"min_hold_secondsは0〜600の数値にしてください: {hold!r}")
        desc = doc.get("description", "")
        if not isinstance(desc, str) or len(desc) > 500:
            raise TreeError("descriptionは500文字以内の文字列にしてください")
        enabled = doc.get("enabled", True)
        if not isinstance(enabled, bool):
            raise TreeError("enabledはtrue/falseにしてください")
        source = doc.get("source")
        if source is not None and source not in TREE_SOURCES:
            raise TreeError(f"sourceは {list(TREE_SOURCES)} のいずれかにしてください: {source!r}")
        run_on_startup = doc.get("run_on_startup", False)
        if not isinstance(run_on_startup, bool):
            raise TreeError("run_on_startupはtrue/falseにしてください")
        if run_on_startup and source is None:
            raise TreeError('run_on_startupはsource:"schedule"の木でだけ使えます')
        if "root" not in doc:
            raise TreeError("rootが必要です")
        counter = [0, MAX_SCHEDULE_NODES if source == "schedule" else MAX_NODES]
        root = _validate_node(doc["root"], "0", 0, counter, warnings)
        if not root.get("enabled", True):
            raise TreeError("ルートノードは無効にできません")
    except (TreeError, br.RuleError) as exc:
        return [str(exc)], warnings, None
    out = {
        "version": 1,
        "description": desc,
        "min_hold_seconds": float(hold),
        "enabled": enabled,
        "root": root,
    }
    if source is not None:
        out["source"] = source
        out["run_on_startup"] = run_on_startup
    out["node_count"] = counter[0]  # 保存時は除く(下)
    return [], warnings, out


def document_text(norm: dict) -> str:
    d = {k: v for k, v in norm.items() if k != "node_count"}
    return json.dumps(d, ensure_ascii=False, indent=2, allow_nan=False) + "\n"


def doc_hash(norm: dict) -> str:
    """木の内容の短い指紋(directorが読み込み中の木と、保存済みファイルの一致確認用)。"""
    import hashlib

    return hashlib.sha1(document_text(norm).encode("utf-8")).hexdigest()[:12]


def _reject_constant(name: str):
    raise TreeError(f"JSONに{name}(NaN/Infinity)は使えません")


def parse_tree_text(text: str) -> dict:
    """JSON文字列を検証して正規化した文書にする。不正ならTreeError。"""
    try:
        data = json.loads(text, parse_constant=_reject_constant)
    except TreeError:
        raise
    except (ValueError, RecursionError) as exc:
        raise TreeError(f"JSONとして読めません: {exc}")
    errors, _w, norm = validate_document(data)
    if errors or norm is None:
        raise TreeError(errors[0] if errors else "検証に失敗しました")
    return norm


def build_tree(norm_root: dict, nid: str = "0") -> Node:
    """検証済みのノード辞書からNodeを組み立てる(無効ノードは取り除く。idはJSON上の位置のまま)。"""
    cls = NODE_CLASSES[norm_root["type"]]
    kids = [
        build_tree(c, f"{nid}.{i}")
        for i, c in enumerate(norm_root.get("children", []))
        if c.get("enabled", True)
    ]
    return cls(nid, norm_root.get("name", ""), dict(norm_root.get("params", {})), kids)


def default_document() -> dict:
    """既定の木 = ルール表ノード1つ(従来の動作そのまま)。ルール表側が最小保持時間を持つので木側は0。"""
    return {
        "version": 1,
        "description": "既定の木(ルール表だけ=従来どおりの動作)",
        "min_hold_seconds": 0.0,
        "enabled": True,
        "root": {"type": "rules_table", "name": "時間帯・状況ルール表"},
    }


def list_ids(norm_root: dict, nid: str = "0") -> List[Tuple[str, dict]]:
    """(id, ノード辞書)を深さ優先で列挙(画面の表示順)。"""
    out = [(nid, norm_root)]
    for i, c in enumerate(norm_root.get("children", [])):
        out.extend(list_ids(c, f"{nid}.{i}"))
    return out


# --------------------------------------------------------------------------
# 例の木(編集画面の「例から読み込む」と、docs/behavior_tree.md の例。テストで検証・実行している)
# --------------------------------------------------------------------------


def _n(typ: str, name: str = "", params: Optional[dict] = None, children: Optional[List[dict]] = None) -> dict:
    d: Dict[str, Any] = {"type": typ}
    if name:
        d["name"] = name
    if params:
        d["params"] = params
    if children is not None:
        d["children"] = children
    return d


def example_documents() -> List[Dict[str, Any]]:
    """[{id, title, description, document}]。ピンのidは仮(pinA)なので、使う前に実際のピンを選び直すこと。"""
    pin = {"pin_id": "pinA", "pin_name": "ピンA"}
    day = _n("selector", "一日の流れ", children=[
        _n("sequence", "閉店前はピンAへ移動", children=[
            _n("time_between", "閉店前", {"range": "16:30-17:30"}),
            _n("goto_pin", "ピンAへ", dict(pin)),
        ]),
        _n("sequence", "朝は入口で待機、人が来たら案内", children=[
            _n("time_between", "朝", {"range": "08:00-10:30"}),
            _n("selector", "人がいるか", children=[
                _n("sequence", "人が近づいたら案内", children=[
                    _n("person_present", "人がいる", {"value": True}),
                    _n("normal", "通常動作(接近して案内)"),
                ]),
                _n("go_to_entrance", "入口で待つ"),
            ]),
        ]),
        _n("sequence", "残りが少ないなら呼びかけを短く", children=[
            _n("stock_ratio", "残数が3割未満", {"op": "below", "value": 0.3}),
            _n("announce", "呼びかけ20秒間隔", {"interval_seconds": 20.0}),
        ]),
        _n("rules_table", "時間帯・状況ルール表"),
    ])
    peak = _n("selector", "ルール表+昼の例外", children=[
        _n("sequence", "昼の混雑時は静かにゆっくり", children=[
            _n("time_between", "昼", {"range": "11:30-13:00"}),
            _n("crowd_is", "混雑", {"levels": ["crowded"]}),
            _n("quiet", "発話しない"),
            _n("speed_scale", "ゆっくり", {"scale": 0.5}),
        ]),
        _n("rules_table", "時間帯・状況ルール表"),
    ])
    tour = _n("selector", "人がいない時だけ巡回", children=[
        _n("sequence", "巡回", children=[
            _n("idle_for", "人が2分見えない", {"seconds": 120.0}),
            _n("cooldown", "10分に1回まで", {"seconds": 600.0}, children=[
                _n("sequence", "入口→ピンA→待つ", {"memory": True}, children=[
                    _n("go_to_entrance", "入口へ(60秒)", {"for_seconds": 60.0}),
                    _n("goto_pin", "ピンAへ(90秒)", dict(pin, for_seconds=90.0)),
                    _n("wait", "30秒待つ", {"seconds": 30.0}),
                ]),
            ]),
        ]),
        _n("normal", "通常動作"),
    ])

    def doc(root: dict, text: str, hold: float = 10.0) -> dict:
        return {"version": 1, "description": text, "min_hold_seconds": hold, "enabled": True, "root": root}

    return [
        {"id": "day", "title": "一日の流れ(朝は入口、閉店前はピンへ)",
         "description": "朝は入口で待機し人が来たら案内、残数が少なければ呼びかけを短く、閉店前はピンAへ移動。それ以外はルール表。",
         "document": doc(day, "朝は入口で待機/人が来たら案内/残数少なめは呼びかけ短縮/閉店前はピンA")},
        {"id": "peak", "title": "ルール表+昼の混雑の例外",
         "description": "いつもはルール表どおり。昼の混雑時だけ静かに・ゆっくり動く。",
         "document": doc(peak, "ルール表を残しつつ昼の混雑だけ例外")},
        {"id": "tour", "title": "人がいない時だけ巡回",
         "description": "人が2分見えない時、入口→ピンA→30秒待つ、を10分に1回まで行う。",
         "document": doc(tour, "無人時の巡回(メモリ付きシーケンス)")},
    ]


# --------------------------------------------------------------------------
# 実行器
# --------------------------------------------------------------------------


@dataclass
class TreeState:
    effects: br.Effects
    policies: Tuple[str, ...]
    params: Dict[str, Any]
    active_rule_id: Optional[str]
    active_rule_name: Optional[str]
    activation: int
    changed: bool
    statuses: Dict[str, str]
    active_leaves: List[str]
    tick_no: int
    root_status: str
    boundary: Optional[Dict[str, Any]] = None  # set_boundaryの出力(None=元の範囲のまま)
    display: Optional[str] = None  # set_displayの出力(None=表示モードは変えない)


class TreeRunner:
    """木の保持・ホットリロード・tick評価(最小保持時間つき)。

    user_active: ファイルから有効な木を読み込めている(従来動作とは別に木で方針を決める)か。
    False(ファイル無し/不正だった/enabled:false)の間、directorは従来のルール表の結果をそのまま使う。
    """

    def __init__(
        self,
        tree_file: str = DEFAULT_TREE_FILE,
        clock: Optional[Callable[[], datetime.datetime]] = None,
        mono: Optional[Callable[[], float]] = None,
    ) -> None:
        self.tree_file = os.path.expanduser(tree_file) if tree_file else ""
        self._clock = clock or datetime.datetime.now
        self._mono = mono or time.monotonic
        self.load_error: Optional[str] = None
        self.last_log: List[str] = []
        self._file_sig: Optional[Tuple[float, int]] = None
        self._force_reload = True
        self._from_file = False
        self.doc: dict = default_document()
        self.root: Node = build_tree(self.doc["root"])
        self.warnings: List[str] = []
        self._tick_no = 0
        self._activation = ACTIVATION_BASE
        self._held: Optional[Tuple[Any, br.Effects, Tuple[str, ...], Dict[str, Any], Optional[str], Optional[str], Optional[Dict[str, Any]], Optional[str]]] = None
        # スケジュール由来の木(source:"schedule")を「実行中」にするか。directorの起動直後はFalse
        # (run_on_startupがtrueの木をファイルから最初に読んだときだけ自動でTrue)。/api/schedule/run の合図でTrue、
        # 停止の合図でFalse。ユーザー編集の木(sourceなし)には関係しない。
        self.schedule_armed = False
        # run_on_startup による自動実行は、起動から startup_delay_sec 以上たち、かつ時計が合っていない疑い(年<2026)が
        # ないときだけ(時刻ずれのまま起動直後に走り出さないため)。directorはパラメータ schedule_startup_delay_sec(既定120)を入れる。
        # このクラス単体の既定は0(待たない。時計の確認だけ行う)。
        self.startup_delay_sec = 0.0
        self.startup_run_pending = False  # 自動実行の条件待ち(ファイルを最初に読んで run_on_startup だった)
        self._startup_checked = False
        self._held_since = 0.0
        self._start_mono = self._mono()
        self._last_person_mono: Optional[float] = None
        self.last_state: Optional[TreeState] = None

    # --- 読み込み ---

    @property
    def is_schedule(self) -> bool:
        return self._from_file and self.doc.get("source") == "schedule"

    @property
    def user_active(self) -> bool:
        if not (self._from_file and bool(self.doc.get("enabled", True))):
            return False
        if self.doc.get("source") == "schedule":
            return self.schedule_armed  # スケジュールは「実行」の合図があるまで動かさない
        return True

    def arm_schedule(self, armed: bool) -> None:
        """スケジュールの実行/停止の合図(directorが/nokosahen/schedule/commandを受けて呼ぶ)。"""
        self.schedule_armed = bool(armed)
        self.startup_run_pending = False  # 明示の合図が来たら、起動時の自動実行の待ちは終わり(停止の合図なら自動では走らせない)
        self._held = None  # 切り替えは最小保持時間を待たず即時

    def _try_startup_arm(self) -> None:
        """run_on_startup の自動実行: 起動から startup_delay_sec 以上たち、時計が合っていそうなときだけ実行を始める。"""
        if not self.startup_run_pending:
            return
        d = self.doc
        if not (self._from_file and d.get("source") == "schedule" and d.get("run_on_startup") and d.get("enabled", True)):
            self.startup_run_pending = False  # 待っている間にファイルが差し替わった(条件が変わった)
            return
        if self._mono() - self._start_mono < self.startup_delay_sec:
            return
        if br.clock_suspect(self._clock()):
            return  # 時計が直るまで待つ(直れば自動で始まる)
        self.startup_run_pending = False
        self.schedule_armed = True
        self._held = None

    def request_reload(self) -> None:
        self._force_reload = True

    def set_document(self, norm: dict, from_file: bool = True) -> None:
        self.doc = norm
        self.root = build_tree(norm["root"])
        self._from_file = from_file
        self._held = None  # 切り替えは最小保持時間を待たず即時

    def check_reload(self) -> Optional[str]:
        """ファイルのmtime/サイズが変わっていれば再読み込みする。変化時の人間向けメッセージを返す。
        不正なファイルは全体を拒否して直前の木を維持し(起動直後なら従来動作のまま)、load_errorに理由を残す。"""
        path = self.tree_file
        if not path:
            return None
        try:
            st = os.stat(path)
            sig: Optional[Tuple[float, int]] = (st.st_mtime, st.st_size)
        except OSError:
            sig = None
        if sig == self._file_sig and not self._force_reload:
            return None
        self._force_reload = False
        previous_sig = self._file_sig
        self._file_sig = sig
        if sig is None:
            if previous_sig is None:
                return None
            self.set_document(default_document(), from_file=False)
            self.load_error = None
            return "ビヘイビアツリーのファイルが無くなったため従来動作(ルール表のみ)に戻しました"
        try:
            with open(path, "r", encoding="utf-8") as f:
                text = f.read()
            norm = parse_tree_text(text)
        except (TreeError, OSError, UnicodeDecodeError) as exc:
            self.load_error = f"{type(exc).__name__}: {exc}"
            return f"ビヘイビアツリーのファイルを拒否しました(直前の木/従来動作を維持): {self.load_error}"
        self.set_document(norm, from_file=True)
        if not self._startup_checked:
            self._startup_checked = True
            # 起動後の最初の読み込み: run_on_startup:trueの(有効な)スケジュールだけ、条件がそろえば実行を始める
            # (起動から startup_delay_sec 以上、かつ時計が合っているとき。_try_startup_arm)
            if norm.get("source") == "schedule" and norm.get("run_on_startup") and norm.get("enabled", True):
                self.startup_run_pending = True
                self._try_startup_arm()
        self.load_error = None
        self.warnings = validate_document(norm)[1]
        if not norm.get("enabled", True):
            return "ビヘイビアツリーを読み込みました(enabled:false のため従来動作のまま)"
        return f"ビヘイビアツリーを読み込みました: {norm.get('node_count', 0)}ノード"

    # --- 評価 ---

    def evaluate(
        self,
        ctx: br.Context,
        rule_state: Optional[br.RuleState],
        safety: Optional[Dict[str, bool]] = None,
        pin_ok: Optional[Callable[[str, Optional[str]], bool]] = None,
    ) -> TreeState:
        self._try_startup_arm()
        self._tick_no += 1
        mono = self._mono()
        if ctx.person_present:
            self._last_person_mono = mono
        if ctx.idle_seconds is None:
            base = self._last_person_mono if self._last_person_mono is not None else self._start_mono
            ctx = br.Context(
                crowd_level=ctx.crowd_level,
                person_present=ctx.person_present,
                menu_ratios=ctx.menu_ratios,
                idle_seconds=max(0.0, mono - base),
            )
        env = Env(
            tick_no=self._tick_no,
            dt=self._clock(),
            mono=mono,
            ctx=ctx,
            safety=dict(safety or {}),
            pin_ok=pin_ok or (lambda _pid, _pm: False),
            rule_state=rule_state,
            out=Output(),
            trace={},
            leaves=[],
        )
        root_status = self.root.tick(env)
        policies = env.out.final_policies()
        params = dict(env.out.params)
        rule_id = rule_name = None
        if env.used_rules and rule_state is not None:
            rule_id, rule_name = rule_state.active_rule_id, rule_state.active_rule_name
        boundary = dict(env.out.boundary) if env.out.boundary else None
        sig = (policies, tuple(sorted(params.items(), key=lambda kv: kv[0])),
               rule_state.activation if env.used_rules and rule_state is not None else None,
               tuple(sorted(boundary.items())) if boundary else None,
               env.out.display)
        hold = float(self.doc.get("min_hold_seconds", 0.0))
        changed = False
        if self._held is None or (
            sig != self._held[0] and (hold <= 0 or mono - self._held_since >= hold)
        ):
            effects = br.effects_for(policies, params) if policies != ("normal",) or params else br.NEUTRAL
            self._held = (sig, effects, policies, params, rule_id, rule_name, boundary, env.out.display)
            self._held_since = mono
            self._activation += 1
            changed = True
        _sig, effects, policies, params, rule_id, rule_name, boundary, display = self._held
        state = TreeState(
            effects=effects, policies=policies, params=dict(params),
            active_rule_id=rule_id, active_rule_name=rule_name,
            activation=self._activation, changed=changed, statuses=dict(env.trace),
            active_leaves=list(env.leaves), tick_no=self._tick_no, root_status=root_status,
            boundary=dict(boundary) if boundary else None,
            display=display,
        )
        self.last_state = state
        return state

    # --- 状態配信用 ---

    def status_dict(
        self,
        state: Optional[TreeState],
        safety: Optional[Dict[str, bool]] = None,
        mode: Optional[str] = None,
        used: Optional[bool] = None,
    ) -> Dict[str, Any]:
        """/nokosahen/behavior_tree/status のJSON中身。usedは「いま木の結果を使っているか」。"""
        safety = dict(safety or {})
        return {
            "stamp": time.time(),
            "tick": state.tick_no if state else 0,
            "tree_file": self.tree_file,
            "user_active": self.user_active if used is None else bool(used),
            "is_default": not self._from_file,
            "enabled": bool(self.doc.get("enabled", True)),
            "load_error": self.load_error,
            "doc_hash": doc_hash(self.doc),
            "node_count": sum(1 for _ in list_ids(self.doc["root"])),
            "mode": mode,
            "safety": {k: bool(safety.get(k, False)) for k in SAFETY_KEYS},
            "safety_hold": any(safety.get(k, False) for k in SAFETY_KEYS),
            "statuses": dict(state.statuses) if state else {},
            "active_leaves": list(state.active_leaves) if state else [],
            "root_status": state.root_status if state else None,
            "policy": "+".join(state.policies) if state else "normal",
            "policies": list(state.policies) if state else ["normal"],
            "params": dict(state.params) if state else {},
            "effects": state.effects.to_dict() if state else br.NEUTRAL.to_dict(),
            "active_rule_id": state.active_rule_id if state else None,
            "active_rule_name": state.active_rule_name if state else None,
            "activation": state.activation if state else None,
            "source": self.doc.get("source"),
            "schedule_armed": bool(self.schedule_armed),
            "boundary": dict(state.boundary) if state and state.boundary else None,
            "display": state.display if state else None,
        }


# --------------------------------------------------------------------------
# 検証CLI
# --------------------------------------------------------------------------


def describe(node: dict, nid: str = "0", indent: int = 0) -> List[str]:
    spec = NODE_SPECS[node["type"]]
    pad = "  " * indent
    params = node.get("params")
    off = "" if node.get("enabled", True) else " [無効]"
    title = node.get("name") or spec["label"]
    lines = [f"{pad}{spec['icon']} {title}{off} <{node['type']}>" + (f" {params}" if params else "")]
    for i, c in enumerate(node.get("children", [])):
        lines.extend(describe(c, f"{nid}.{i}", indent + 1))
    return lines


def validate_file(path: str) -> Tuple[bool, str]:
    try:
        with open(os.path.expanduser(path), "r", encoding="utf-8") as f:
            text = f.read()
    except (OSError, UnicodeDecodeError) as exc:
        return False, f"NG: ファイルを読めません: {exc}"
    try:
        data = json.loads(text, parse_constant=_reject_constant)
    except (ValueError, TreeError, RecursionError) as exc:
        return False, f"NG: JSONとして読めません: {exc}"
    errors, warnings, norm = validate_document(data)
    if errors or norm is None:
        return False, "NG: このファイルは拒否されます(読み込み時は直前の木/従来動作を維持)\n  理由: " + " / ".join(errors)
    lines = [f"OK: {norm['node_count']}ノード、最小保持時間{norm['min_hold_seconds']:g}秒"
             + ("" if norm["enabled"] else "(enabled:false=従来動作のまま)")]
    lines += ["  " + ln for ln in describe(norm["root"])]
    lines += [f"  注意: {w}" for w in warnings]
    return True, "\n".join(lines)


def main(argv: Optional[List[str]] = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if len(argv) != 2 or argv[0] != "validate":
        print("使い方: python -m nokosahen_core.behavior_tree validate <木ファイル>")
        return 2
    ok, summary = validate_file(argv[1])
    print(summary)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
