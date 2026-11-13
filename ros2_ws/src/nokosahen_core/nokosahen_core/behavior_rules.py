"""時間帯・状況に応じた「方針(policy)」を選ぶ宣言的ルール表(純Python、ROS非依存)。

behavior_director.py から使われる。ここでは「どの方針が有効か」を計算するだけで、
ロボットを直接動かすことは一切しない。ESTOP / MANUAL_OVERRIDE / NEAR_PERSON_HOLD /
各種HOLD / MAPPING_ACTIVE_HOLD は behavior_director 側で本モジュールの結果より
常に先に判定される(方針は安全系の下にある非安全分岐にしか効かない)。

ルールファイル(JSON、既定 ~/.nokosahen_behavior_rules.json):
    {
      "version": 1,
      "min_hold_seconds": 10,
      "rules": [
        {"id": "lunch_peak", "enabled": true, "name": "昼のピーク", "priority": 50,
         "when": {"time_ranges": ["11:30-13:00"], "crowd_level": ["calm"]},
         "then": {"policy": ["approach_enabled"], "params": {}}}
      ]
    }
詳細は docs/behavior_rules.md。

検証CLI:  python -m nokosahen_core.behavior_rules validate <file>
"""

from __future__ import annotations

import datetime
import json
import math
import os
import re
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

DEFAULT_RULES_FILE = "~/.nokosahen_behavior_rules.json"
DEFAULT_MIN_HOLD_SECONDS = 10.0
MAX_RULES = 100

POLICIES = (
    "normal",
    "stay",
    "announce_only",
    "approach_enabled",
    "approach_disabled",
    "crowd_scan_enabled",
    "crowd_scan_disabled",
    "face_enabled",
    "face_disabled",
    "idle_wiggle_on",
    "idle_wiggle_off",
    "go_to_entrance",
    "return_home",
    "goto_pin",
    "quiet",
    "speed_scale",
)

# 同時指定できない組(矛盾するもの)。
_CONFLICTS = (
    ("approach_enabled", "approach_disabled"),
    ("crowd_scan_enabled", "crowd_scan_disabled"),
    ("face_enabled", "face_disabled"),
    ("idle_wiggle_on", "idle_wiggle_off"),
    ("go_to_entrance", "return_home"),
    ("goto_pin", "go_to_entrance"),
    ("goto_pin", "return_home"),
)

WEEKDAYS = ("mon", "tue", "wed", "thu", "fri", "sat", "sun")
CROWD_LEVELS = ("calm", "crowded", "unknown")

SPEED_SCALE_MIN = 0.3
SPEED_SCALE_MAX = 1.0
ANNOUNCE_INTERVAL_MIN = 10.0
ANNOUNCE_INTERVAL_MAX = 600.0

_TOP_KEYS = {"version", "description", "min_hold_seconds", "rules"}
_RULE_KEYS = {"id", "enabled", "name", "description", "when", "then", "priority"}
_WHEN_KEYS = {
    "time_ranges",
    "weekdays",
    "dates",
    "crowd_level",
    "person_present",
    "remaining_ratio_below",
    "remaining_ratio_above",
    "menu",
    "idle_seconds_above",
}
_THEN_KEYS = {"policy", "params"}
# goto_pin用: 行き先の地図ピン。pin_id(map_pins.pyのID)が本体で、pin_name(表示用)と
# pin_map(そのピンを作った地図名)は表示・取り違え防止用。名前が変わってもIDで引く。
_PARAM_KEYS = {"scale", "announce_min_interval_seconds", "pin_id", "pin_name", "pin_map"}
_PIN_ID_RE = re.compile(r"^[A-Za-z0-9_\-]{1,32}$")  # map_pins._ID_RE と同じ
_PIN_MAP_RE = re.compile(r"^[A-Za-z0-9_\-]{1,64}$")  # map_pins._SAFE_MAP_NAME と同じ
PIN_NAME_MAX_LEN = 40  # map_pins.PIN_NAME_MAX_LEN と同じ

_ID_RE = re.compile(r"^[A-Za-z0-9_\-]{1,64}$")
_TIME_RANGE_RE = re.compile(r"^(\d{2}):(\d{2})-(\d{2}):(\d{2})$")
_DATE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")


class RuleError(ValueError):
    """ルールファイルの検証エラー。"""


# --------------------------------------------------------------------------
# データ構造
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Rule:
    id: str
    enabled: bool
    name: str
    priority: int
    description: str = ""
    time_ranges: Tuple[Tuple[int, int], ...] = ()  # (開始分, 終了分) 終了は排他
    weekdays: Tuple[int, ...] = ()  # 0=mon..6=sun
    dates: Tuple[str, ...] = ()
    crowd_level: Tuple[str, ...] = ()
    person_present: Optional[bool] = None
    remaining_ratio_below: Optional[float] = None
    remaining_ratio_above: Optional[float] = None
    menu: Optional[str] = None
    idle_seconds_above: Optional[float] = None
    policies: Tuple[str, ...] = ("normal",)
    params: Dict[str, Any] = field(default_factory=dict)

    def time_part_matches(self, dt: datetime.datetime) -> bool:
        """時刻・曜日・日付だけの判定(next_change_at推定にも使う)。
        【2026-10-21】時計がまだ合っていない(年がCLOCK_VALID_YEAR未満。Jetsonは再起動のたびに1970年などへ戻る)ときは、
        時刻・曜日・日付の条件を持つルールは「成立しない」(昼に「22:00-06:00は黙って待機」が成立する、などを防ぐ)。
        時刻の条件を持たないルール(混雑・残数など)は、従来どおり。"""
        if (self.weekdays or self.dates or self.time_ranges) and clock_suspect(dt):
            return False
        if self.weekdays and dt.weekday() not in self.weekdays:
            return False
        if self.dates and dt.strftime("%Y-%m-%d") not in self.dates:
            return False
        if self.time_ranges:
            minute = dt.hour * 60 + dt.minute
            if not any(_in_range(minute, s, e) for s, e in self.time_ranges):
                return False
        return True

    def matches(self, dt: datetime.datetime, ctx: "Context") -> bool:
        if not self.time_part_matches(dt):
            return False
        if self.crowd_level and ctx.crowd_level not in self.crowd_level:
            return False
        if self.person_present is not None:
            if ctx.person_present is None or ctx.person_present != self.person_present:
                return False
        if self.remaining_ratio_below is not None or self.remaining_ratio_above is not None:
            ratio = ctx.ratio_for(self.menu)
            if ratio is None:
                return False
            if self.remaining_ratio_below is not None and not ratio < self.remaining_ratio_below:
                return False
            if self.remaining_ratio_above is not None and not ratio > self.remaining_ratio_above:
                return False
        elif self.menu is not None and ctx.ratio_for(self.menu) is None:
            # menu指定のみ(比率条件なし)は「そのメニューが登録されている」ことを要求する。
            return False
        if self.idle_seconds_above is not None:
            if ctx.idle_seconds is None or not ctx.idle_seconds > self.idle_seconds_above:
                return False
        return True


# この年より前の時計は「まだ合っていない」とみなす(RTCもインターネットもなく、再起動のたびに1970年などへ戻る。docs/clock_sync.md)
CLOCK_VALID_YEAR = 2026


def clock_suspect(dt: datetime.datetime) -> bool:
    """時計が合っていない疑いがあるか(年がCLOCK_VALID_YEAR未満)。"""
    return dt.year < CLOCK_VALID_YEAR


def _in_range(minute: int, start: int, end: int) -> bool:
    """[start, end)。start>end なら日付またぎ(例 22:00-06:00)。"""
    if start < end:
        return start <= minute < end
    return minute >= start or minute < end


@dataclass
class Context:
    """ルール評価に使う現在の状況。"""

    crowd_level: str = "unknown"
    person_present: Optional[bool] = None
    # メニュー名 -> 残数比(0..1)。空ならメニュー未登録。
    menu_ratios: Dict[str, float] = field(default_factory=dict)
    idle_seconds: Optional[float] = None

    def ratio_for(self, menu: Optional[str]) -> Optional[float]:
        if not self.menu_ratios:
            return None
        if menu is not None:
            return self.menu_ratios.get(menu)
        return min(self.menu_ratios.values())  # 指定なしは最も少ないメニュー


@dataclass(frozen=True)
class Effects:
    """方針を、behavior_directorが直接使えるフラグへ展開したもの。

    既定値(NEUTRAL)は「現行動作そのまま」。
    """

    hold_position: bool = False  # 移動を伴う自律行動(接近・見回し・揺動・帰還)を全て止める
    approach_allowed: bool = True
    crowd_scan_allowed: bool = True
    # 【2026-11-02追加】ルール/木が approach_enabled / crowd_scan_enabled を「明示」したか。
    # directorの「人の方へ動くのは指定したときだけ」(スイッチOFFが既定)で、明示があればスイッチOFFでも動く。
    approach_explicit: bool = False
    scan_explicit: bool = False
    # 【2026-11-03追加】向く(その場で人の方を向く)動作。face_enabled / face_disabled。
    face_allowed: bool = True
    face_explicit: bool = False
    # 呼びかけ(メニュー紹介の発話)を、木の announce 動作/ルールの発話間隔指定で「指示した」か。
    announce_explicit: bool = False
    idle_wiggle: Optional[bool] = None  # None=従来のidle_wiggle_enabledに従う
    go_to: Optional[str] = None  # None / "entrance" / "home" / "pin"
    # go_to=="pin" のときの行き先(地図ピンのIDと、作った地図名・表示名)。
    go_to_pin_id: Optional[str] = None
    go_to_pin_map: Optional[str] = None
    go_to_pin_name: Optional[str] = None
    announce_allowed: bool = True
    announce_min_interval_seconds: Optional[float] = None
    # 【2026-11-03追加】true=向く動作が終わってから(誤差5度以内)にだけ呼びかけの依頼を出す(木のannounce after_facing)
    announce_after_facing: bool = False
    speed_scale: float = 1.0  # 0.3..1.0、現状を超えない上限係数

    def to_dict(self) -> dict:
        return {
            "hold_position": self.hold_position,
            "approach_allowed": self.approach_allowed,
            "crowd_scan_allowed": self.crowd_scan_allowed,
            "approach_explicit": self.approach_explicit,
            "scan_explicit": self.scan_explicit,
            "face_allowed": self.face_allowed,
            "face_explicit": self.face_explicit,
            "announce_explicit": self.announce_explicit,
            "idle_wiggle": self.idle_wiggle,
            "go_to": self.go_to,
            "go_to_pin_id": self.go_to_pin_id,
            "go_to_pin_map": self.go_to_pin_map,
            "go_to_pin_name": self.go_to_pin_name,
            "announce_allowed": self.announce_allowed,
            "announce_min_interval_seconds": self.announce_min_interval_seconds,
            "announce_after_facing": self.announce_after_facing,
            "speed_scale": self.speed_scale,
        }


NEUTRAL = Effects()


def effects_for(policies: Tuple[str, ...], params: Dict[str, Any]) -> Effects:
    hold = False
    approach = True
    scan = True
    approach_explicit = False
    scan_explicit = False
    face = True
    face_explicit = False
    wiggle: Optional[bool] = None
    go_to: Optional[str] = None
    announce = True
    for p in policies:
        if p in ("stay", "announce_only"):
            hold = True
            approach = False
            scan = False
            wiggle = False
        elif p == "approach_enabled":
            approach = True
            approach_explicit = True
        elif p == "approach_disabled":
            approach = False
            approach_explicit = False
        elif p == "crowd_scan_enabled":
            scan = True
            scan_explicit = True
        elif p == "crowd_scan_disabled":
            scan = False
            scan_explicit = False
        elif p == "face_enabled":
            face = True
            face_explicit = True
        elif p == "face_disabled":
            face = False
            face_explicit = False
        elif p == "idle_wiggle_on":
            wiggle = True
        elif p == "idle_wiggle_off":
            wiggle = False
        elif p == "go_to_entrance":
            go_to = "entrance"
        elif p == "return_home":
            go_to = "home"
        elif p == "goto_pin":
            go_to = "pin"
        elif p == "quiet":
            announce = False
    scale = 1.0
    if "speed_scale" in policies:
        scale = min(SPEED_SCALE_MAX, max(SPEED_SCALE_MIN, float(params.get("scale", 1.0))))
    interval = params.get("announce_min_interval_seconds")
    return Effects(
        hold_position=hold,
        approach_allowed=approach,
        crowd_scan_allowed=scan,
        approach_explicit=approach_explicit,
        scan_explicit=scan_explicit,
        face_allowed=face,
        face_explicit=face_explicit,
        announce_explicit=interval is not None,
        idle_wiggle=wiggle,
        go_to=go_to,
        go_to_pin_id=params.get("pin_id") if go_to == "pin" else None,
        go_to_pin_map=params.get("pin_map") if go_to == "pin" else None,
        go_to_pin_name=params.get("pin_name") if go_to == "pin" else None,
        announce_allowed=announce,
        announce_min_interval_seconds=float(interval) if interval is not None else None,
        announce_after_facing=bool(params.get("announce_after_facing", False)),
        speed_scale=scale,
    )


@dataclass(frozen=True)
class RuleSet:
    rules: Tuple[Rule, ...] = ()
    min_hold_seconds: float = DEFAULT_MIN_HOLD_SECONDS
    description: str = ""


# --------------------------------------------------------------------------
# 検証(厳格: 1つでも不正ならファイル全体を拒否)
# --------------------------------------------------------------------------


def _reject_constant(name: str):
    raise RuleError(f"JSONに{name}(NaN/Infinity)は使えません")


def _is_num(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def parse_time_range(text: Any, where: str) -> Tuple[int, int]:
    if not isinstance(text, str):
        raise RuleError(f"{where}: 時間帯は 'HH:MM-HH:MM' の文字列にしてください: {text!r}")
    m = _TIME_RANGE_RE.match(text.strip())
    if not m:
        raise RuleError(f"{where}: 時間帯の書式が不正です(例 '11:30-13:00'): {text!r}")
    h1, m1, h2, m2 = (int(x) for x in m.groups())
    if not (0 <= h1 <= 23 and 0 <= m1 <= 59):
        raise RuleError(f"{where}: 開始時刻が範囲外です: {text!r}")
    if not ((0 <= h2 <= 23 and 0 <= m2 <= 59) or (h2 == 24 and m2 == 0)):
        raise RuleError(f"{where}: 終了時刻が範囲外です('24:00'のみ可): {text!r}")
    start = h1 * 60 + m1
    end = h2 * 60 + m2
    if start == end:
        raise RuleError(f"{where}: 開始と終了が同じ時刻です: {text!r}")
    if end == 24 * 60:
        end = 24 * 60  # 排他的終端として翌0:00相当(minuteは最大1439なので全日末まで一致)
    return start, end


def _check_keys(obj: dict, allowed: set, where: str) -> None:
    unknown = sorted(set(obj) - allowed)
    if unknown:
        raise RuleError(f"{where}: 未知のキー {unknown}(使えるキー: {sorted(allowed)})")


def _str_list(v: Any, where: str) -> List[str]:
    if not isinstance(v, list) or not v:
        raise RuleError(f"{where}: 空でない配列にしてください")
    for x in v:
        if not isinstance(x, str):
            raise RuleError(f"{where}: 文字列の配列にしてください: {x!r}")
    return v


def _parse_rule(raw: Any, index: int) -> Rule:
    where = f"rules[{index}]"
    if not isinstance(raw, dict):
        raise RuleError(f"{where}: オブジェクトにしてください")
    _check_keys(raw, _RULE_KEYS, where)
    rid = raw.get("id")
    if not isinstance(rid, str) or not _ID_RE.match(rid):
        raise RuleError(f"{where}: idは英数字/_/-の1〜64文字にしてください: {rid!r}")
    where = f"rules[{index}]({rid})"
    enabled = raw.get("enabled", False)
    if not isinstance(enabled, bool):
        raise RuleError(f"{where}: enabledはtrue/falseにしてください: {enabled!r}")
    name = raw.get("name", rid)
    if not isinstance(name, str) or len(name) > 100:
        raise RuleError(f"{where}: nameは100文字以内の文字列にしてください")
    description = raw.get("description", "")
    if not isinstance(description, str):
        raise RuleError(f"{where}: descriptionは文字列にしてください")
    priority = raw.get("priority", 0)
    if not isinstance(priority, int) or isinstance(priority, bool) or not -1000 <= priority <= 1000:
        raise RuleError(f"{where}: priorityは-1000〜1000の整数にしてください: {priority!r}")

    when = raw.get("when", {})
    if not isinstance(when, dict):
        raise RuleError(f"{where}.when: オブジェクトにしてください")
    _check_keys(when, _WHEN_KEYS, f"{where}.when")

    time_ranges: Tuple[Tuple[int, int], ...] = ()
    if "time_ranges" in when:
        lst = when["time_ranges"]
        if not isinstance(lst, list) or not lst:
            raise RuleError(f"{where}.when.time_ranges: 空でない配列にしてください")
        time_ranges = tuple(parse_time_range(t, f"{where}.when.time_ranges") for t in lst)
    weekdays: Tuple[int, ...] = ()
    if "weekdays" in when:
        names = _str_list(when["weekdays"], f"{where}.when.weekdays")
        for n in names:
            if n not in WEEKDAYS:
                raise RuleError(f"{where}.when.weekdays: 不明な曜日 {n!r}(使える値: {list(WEEKDAYS)})")
        weekdays = tuple(sorted({WEEKDAYS.index(n) for n in names}))
    dates: Tuple[str, ...] = ()
    if "dates" in when:
        ds = _str_list(when["dates"], f"{where}.when.dates")
        for d in ds:
            if not _DATE_RE.match(d):
                raise RuleError(f"{where}.when.dates: 'YYYY-MM-DD'形式にしてください: {d!r}")
            try:
                datetime.date.fromisoformat(d)
            except ValueError:
                raise RuleError(f"{where}.when.dates: 存在しない日付です: {d!r}")
        dates = tuple(ds)
    crowd: Tuple[str, ...] = ()
    if "crowd_level" in when:
        cl = _str_list(when["crowd_level"], f"{where}.when.crowd_level")
        for c in cl:
            if c not in CROWD_LEVELS:
                raise RuleError(f"{where}.when.crowd_level: 不明な値 {c!r}(使える値: {list(CROWD_LEVELS)})")
        crowd = tuple(cl)
    person = when.get("person_present")
    if "person_present" in when and not isinstance(person, bool):
        raise RuleError(f"{where}.when.person_present: true/falseにしてください")
    ratios: Dict[str, Optional[float]] = {}
    for key in ("remaining_ratio_below", "remaining_ratio_above"):
        if key in when:
            v = when[key]
            if not _is_num(v) or not 0.0 <= v <= 1.0:
                raise RuleError(f"{where}.when.{key}: 0〜1の数値にしてください: {v!r}")
            ratios[key] = float(v)
    menu = when.get("menu")
    if "menu" in when and (not isinstance(menu, str) or not menu.strip() or len(menu) > 100):
        raise RuleError(f"{where}.when.menu: メニュー名(空でない文字列)にしてください")
    idle = None
    if "idle_seconds_above" in when:
        v = when["idle_seconds_above"]
        if not _is_num(v) or v < 0:
            raise RuleError(f"{where}.when.idle_seconds_above: 0以上の数値にしてください: {v!r}")
        idle = float(v)

    then = raw.get("then")
    if not isinstance(then, dict):
        raise RuleError(f"{where}.then: オブジェクトにしてください(policyが必要)")
    _check_keys(then, _THEN_KEYS, f"{where}.then")
    pol = then.get("policy")
    pol_list = [pol] if isinstance(pol, str) else pol
    if not isinstance(pol_list, list) or not pol_list or not all(isinstance(p, str) for p in pol_list):
        raise RuleError(f"{where}.then.policy: 方針名(または方針名の配列)にしてください: {pol!r}")
    for p in pol_list:
        if p not in POLICIES:
            raise RuleError(f"{where}.then.policy: 不明な方針 {p!r}(使える値: {list(POLICIES)})")
    if len(set(pol_list)) != len(pol_list):
        raise RuleError(f"{where}.then.policy: 同じ方針が重複しています")
    for a, b in _CONFLICTS:
        if a in pol_list and b in pol_list:
            raise RuleError(f"{where}.then.policy: {a} と {b} は同時に指定できません")
    if "normal" in pol_list and len(pol_list) > 1:
        # normalは「何もしない」ので他と併記しても無意味。パラメータのみのルールは
        # policy:["normal"] + params(announce_min_interval_seconds)で書く。
        raise RuleError(f"{where}.then.policy: normalは他の方針と併記できません")
    params = then.get("params", {})
    if not isinstance(params, dict):
        raise RuleError(f"{where}.then.params: オブジェクトにしてください")
    _check_keys(params, _PARAM_KEYS, f"{where}.then.params")
    clean: Dict[str, Any] = {}
    if "scale" in params:
        v = params["scale"]
        if not _is_num(v) or not SPEED_SCALE_MIN <= v <= SPEED_SCALE_MAX:
            raise RuleError(
                f"{where}.then.params.scale: {SPEED_SCALE_MIN}〜{SPEED_SCALE_MAX}の数値にしてください: {v!r}"
            )
        if "speed_scale" not in pol_list:
            raise RuleError(f"{where}.then.params.scale: policyにspeed_scaleが必要です")
        clean["scale"] = float(v)
    if "speed_scale" in pol_list and "scale" not in clean:
        raise RuleError(f"{where}.then.params.scale: speed_scaleにはscaleが必須です")
    if "announce_min_interval_seconds" in params:
        v = params["announce_min_interval_seconds"]
        if not _is_num(v) or not ANNOUNCE_INTERVAL_MIN <= v <= ANNOUNCE_INTERVAL_MAX:
            raise RuleError(
                f"{where}.then.params.announce_min_interval_seconds: "
                f"{ANNOUNCE_INTERVAL_MIN:g}〜{ANNOUNCE_INTERVAL_MAX:g}の数値にしてください: {v!r}"
            )
        clean["announce_min_interval_seconds"] = float(v)
    if "goto_pin" in pol_list:
        pid = params.get("pin_id")
        if not isinstance(pid, str) or not _PIN_ID_RE.match(pid):
            raise RuleError(
                f"{where}.then.params.pin_id: goto_pinには行き先ピンのid(英数字/_/-の1〜32文字)が必須です: {pid!r}"
            )
        clean["pin_id"] = pid
        if "pin_name" in params:
            pn = params["pin_name"]
            if not isinstance(pn, str) or len(pn) > PIN_NAME_MAX_LEN:
                raise RuleError(f"{where}.then.params.pin_name: {PIN_NAME_MAX_LEN}文字以内の文字列にしてください")
            clean["pin_name"] = pn
        if "pin_map" in params:
            pm = params["pin_map"]
            if not isinstance(pm, str) or not _PIN_MAP_RE.match(pm):
                raise RuleError(f"{where}.then.params.pin_map: 地図名(英数字/_/-の1〜64文字)にしてください: {pm!r}")
            clean["pin_map"] = pm
    else:
        for k in ("pin_id", "pin_name", "pin_map"):
            if k in params:
                raise RuleError(f"{where}.then.params.{k}: policyにgoto_pinが必要です")

    return Rule(
        id=rid,
        enabled=enabled,
        name=name,
        priority=priority,
        description=description,
        time_ranges=time_ranges,
        weekdays=weekdays,
        dates=dates,
        crowd_level=crowd,
        person_present=person if isinstance(person, bool) else None,
        remaining_ratio_below=ratios.get("remaining_ratio_below"),
        remaining_ratio_above=ratios.get("remaining_ratio_above"),
        menu=menu.strip() if isinstance(menu, str) else None,
        idle_seconds_above=idle,
        policies=tuple(pol_list),
        params=clean,
    )


def parse_rules_text(text: str) -> RuleSet:
    """JSON文字列を検証してRuleSetにする。不正ならRuleErrorを投げる。"""
    try:
        data = json.loads(text, parse_constant=_reject_constant)
    except RuleError:
        raise
    except ValueError as exc:
        raise RuleError(f"JSONとして読めません: {exc}")
    if not isinstance(data, dict):
        raise RuleError("トップレベルはオブジェクト({\"rules\": [...]})にしてください")
    _check_keys(data, _TOP_KEYS, "トップレベル")
    version = data.get("version", 1)
    if version != 1 or isinstance(version, bool):
        raise RuleError(f"versionは1のみ対応です: {version!r}")
    hold = data.get("min_hold_seconds", DEFAULT_MIN_HOLD_SECONDS)
    if not _is_num(hold) or not 0 <= hold <= 600:
        raise RuleError(f"min_hold_secondsは0〜600の数値にしてください: {hold!r}")
    desc = data.get("description", "")
    if not isinstance(desc, str):
        raise RuleError("descriptionは文字列にしてください")
    raw_rules = data.get("rules", [])
    if not isinstance(raw_rules, list):
        raise RuleError("rulesは配列にしてください")
    if len(raw_rules) > MAX_RULES:
        raise RuleError(f"ルールが多すぎます(最大{MAX_RULES}件)")
    rules = [_parse_rule(r, i) for i, r in enumerate(raw_rules)]
    seen = set()
    for r in rules:
        if r.id in seen:
            raise RuleError(f"idが重複しています: {r.id}")
        seen.add(r.id)
    return RuleSet(rules=tuple(rules), min_hold_seconds=float(hold), description=desc)


def build_context(crowd_state: Any, menu_state: Any) -> Context:
    """crowd_state / menu_state(既存トピックのJSON辞書)からルール評価用Contextを作る。"""
    crowd_state = crowd_state if isinstance(crowd_state, dict) else {}
    menu_state = menu_state if isinstance(menu_state, dict) else {}
    level = crowd_state.get("level", "unknown")
    if level not in CROWD_LEVELS:
        level = "unknown"
    person: Optional[bool] = None
    if crowd_state:
        person = isinstance(crowd_state.get("person_target"), dict)
    ratios: Dict[str, float] = {}
    items = menu_state.get("items")
    if isinstance(items, list):
        for item in items:
            if not isinstance(item, dict):
                continue
            name = str(item.get("name", "")).strip()
            if not name:
                continue
            ratio = item.get("ratio_remaining")
            if not _is_num(ratio):
                try:
                    initial = float(item.get("initial", 0))
                    remaining = float(item.get("remaining", 0))
                except (TypeError, ValueError):
                    continue
                if not (math.isfinite(initial) and math.isfinite(remaining)) or initial <= 0:
                    continue
                ratio = remaining / initial
            ratios[name] = max(0.0, min(1.0, float(ratio)))
    return Context(crowd_level=level, person_present=person, menu_ratios=ratios)


# --------------------------------------------------------------------------
# エンジン
# --------------------------------------------------------------------------


def make_clock(utc_offset_hours: Optional[float] = None) -> Callable[[], datetime.datetime]:
    """壁時計。offset未指定ならシステムのローカル時刻、指定ならUTC+offsetのnaive時刻。"""
    if utc_offset_hours is None:
        return datetime.datetime.now
    tz = datetime.timezone(datetime.timedelta(hours=float(utc_offset_hours)))

    def _now() -> datetime.datetime:
        return datetime.datetime.now(tz).replace(tzinfo=None)

    return _now


@dataclass
class RuleState:
    """evaluate()の結果。to_json_dict()が /nokosahen/behavior_rules/state の中身。"""

    active_rule_id: Optional[str]
    active_rule_name: Optional[str]
    policies: Tuple[str, ...]
    params: Dict[str, Any]
    effects: Effects
    since: datetime.datetime
    activation: int
    changed: bool
    next_change_at: Optional[datetime.datetime]
    load_error: Optional[str]
    rule_count: int
    enabled_rule_count: int
    rules_file: str
    now: datetime.datetime
    min_hold_seconds: float

    @property
    def policy(self) -> str:
        return "+".join(self.policies)

    def to_json_dict(self) -> dict:
        return {
            "stamp": time.time(),
            "now": self.now.isoformat(timespec="seconds"),
            # 時計が合っていない疑い(年が2026未満)。trueの間、時刻・曜日・日付の条件を持つルールは成立しない
            "clock_suspect": clock_suspect(self.now),
            "active_rule_id": self.active_rule_id,
            "active_rule_name": self.active_rule_name,
            "policy": self.policy,
            "policies": list(self.policies),
            "params": dict(self.params),
            "effects": self.effects.to_dict(),
            "since": self.since.isoformat(timespec="seconds"),
            "next_change_at": self.next_change_at.isoformat(timespec="minutes") if self.next_change_at else None,
            "load_error": self.load_error,
            "rule_count": self.rule_count,
            "enabled_rule_count": self.enabled_rule_count,
            "min_hold_seconds": self.min_hold_seconds,
            "rules_file": self.rules_file,
        }


class RuleEngine:
    """ルール表の保持・ホットリロード・評価(最小保持時間つき)。

    clock: datetime(naive)を返す関数(テストで差し替え)。
    mono:  単調増加の秒を返す関数(最小保持時間・idle計測用、テストで差し替え)。
    """

    def __init__(
        self,
        rules_file: str = DEFAULT_RULES_FILE,
        clock: Optional[Callable[[], datetime.datetime]] = None,
        mono: Optional[Callable[[], float]] = None,
    ) -> None:
        self.rules_file = os.path.expanduser(rules_file) if rules_file else ""
        self._clock = clock or datetime.datetime.now
        self._mono = mono or time.monotonic
        self.ruleset = RuleSet()
        self.load_error: Optional[str] = None
        self._file_sig: Optional[Tuple[float, int]] = None
        self._active_id: Optional[str] = None
        self._active_since_mono = self._mono()
        self._active_since_dt = self._clock()
        self._activation = 0
        self._last_person_mono: Optional[float] = None
        self._start_mono = self._mono()
        self._force_reload = True
        self._next_cache: Tuple[Any, Optional[datetime.datetime]] = (None, None)
        self.last_log: List[str] = []  # 直近の読み込み結果メッセージ(ノードがログへ流す)

    # --- 読み込み ---

    def request_reload(self) -> None:
        self._force_reload = True

    def set_ruleset(self, ruleset: RuleSet) -> None:
        self.ruleset = ruleset
        self._next_cache = (None, None)

    def check_reload(self) -> Optional[str]:
        """ファイルのmtime/サイズが変わっていれば再読み込みする。

        戻り値: 状態が変わったときの人間向けメッセージ(変化なしならNone)。
        不正なファイルは全体を拒否して直前のルールを維持し、load_errorに理由を残す。
        """
        path = self.rules_file
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
                return None  # 最初からファイルが無い: 空ルール(現行動作)、エラーでもない
            self.set_ruleset(RuleSet())
            self.load_error = None
            return "ルールファイルが無くなったためルール表を空にしました(現行動作)"
        try:
            with open(path, "r", encoding="utf-8") as f:
                text = f.read()
            ruleset = parse_rules_text(text)
        except (RuleError, OSError, UnicodeDecodeError) as exc:
            self.load_error = f"{type(exc).__name__}: {exc}"
            return f"ルールファイルを拒否しました(直前のルールを維持): {self.load_error}"
        self.set_ruleset(ruleset)
        self.load_error = None
        enabled = sum(1 for r in ruleset.rules if r.enabled)
        return f"ルールファイルを読み込みました: {len(ruleset.rules)}件(有効{enabled}件)"

    # --- 評価 ---

    def evaluate(self, ctx: Context) -> RuleState:
        dt = self._clock()
        now_m = self._mono()
        if ctx.person_present:
            self._last_person_mono = now_m
        if ctx.idle_seconds is None:
            base = self._last_person_mono if self._last_person_mono is not None else self._start_mono
            ctx = Context(
                crowd_level=ctx.crowd_level,
                person_present=ctx.person_present,
                menu_ratios=ctx.menu_ratios,
                idle_seconds=max(0.0, now_m - base),
            )

        enabled = [r for r in self.ruleset.rules if r.enabled]
        matching = [r for r in enabled if r.matches(dt, ctx)]
        # 優先度が高い順、同順位は記載順(安定ソート)。
        matching.sort(key=lambda r: -r.priority)
        candidate = matching[0].id if matching else None

        changed = False
        active_rule = next((r for r in enabled if r.id == self._active_id), None)
        if candidate != self._active_id:
            held = now_m - self._active_since_mono
            # 現在のルールが削除/無効化された場合は即座に切り替える(古い方針を残さない)。
            if active_rule is None or held >= self.ruleset.min_hold_seconds:
                self._active_id = candidate
                self._active_since_mono = now_m
                self._active_since_dt = dt
                self._activation += 1
                changed = True
                active_rule = next((r for r in enabled if r.id == candidate), None)

        if active_rule is None:
            policies: Tuple[str, ...] = ("normal",)
            params: Dict[str, Any] = {}
            effects = NEUTRAL
        else:
            policies = active_rule.policies
            params = dict(active_rule.params)
            effects = effects_for(policies, params)

        return RuleState(
            active_rule_id=active_rule.id if active_rule else None,
            active_rule_name=active_rule.name if active_rule else None,
            policies=policies,
            params=params,
            effects=effects,
            since=self._active_since_dt,
            activation=self._activation,
            changed=changed,
            next_change_at=self._estimate_next_change(enabled, dt),
            load_error=self.load_error,
            rule_count=len(self.ruleset.rules),
            enabled_rule_count=len(enabled),
            rules_file=self.rules_file,
            now=dt,
            min_hold_seconds=self.ruleset.min_hold_seconds,
        )

    def _estimate_next_change(
        self, enabled: List[Rule], dt: datetime.datetime
    ) -> Optional[datetime.datetime]:
        """時刻・曜日・日付条件だけから、次に「時間条件の成立集合」が変わる時刻を推定する。

        人数・残数など状況条件による切替は予測できない(あくまで目安)。48時間先まで探す。
        """
        timed = [r for r in enabled if r.time_ranges or r.weekdays or r.dates]
        if not timed:
            return None
        base = dt.replace(second=0, microsecond=0)
        key = (base, tuple(r.id for r in timed), id(self.ruleset))
        if self._next_cache[0] == key:
            return self._next_cache[1]
        current = tuple(r.time_part_matches(base) for r in timed)
        result: Optional[datetime.datetime] = None
        for step in range(1, 48 * 60 + 1):
            t = base + datetime.timedelta(minutes=step)
            if tuple(r.time_part_matches(t) for r in timed) != current:
                result = t
                break
        self._next_cache = (key, result)
        return result


# --------------------------------------------------------------------------
# 検証CLI
# --------------------------------------------------------------------------


def describe_rule(r: Rule) -> str:
    conds = []
    if r.time_ranges:
        conds.append(
            "時間 " + ",".join(f"{s // 60:02d}:{s % 60:02d}-{e // 60:02d}:{e % 60:02d}" for s, e in r.time_ranges)
        )
    if r.weekdays:
        conds.append("曜日 " + ",".join(WEEKDAYS[i] for i in r.weekdays))
    if r.dates:
        conds.append("日付 " + ",".join(r.dates))
    if r.crowd_level:
        conds.append("混雑 " + "/".join(r.crowd_level))
    if r.person_present is not None:
        conds.append("人物あり" if r.person_present else "人物なし")
    if r.menu:
        conds.append(f"メニュー「{r.menu}」")
    if r.remaining_ratio_below is not None:
        conds.append(f"残数比<{r.remaining_ratio_below:g}")
    if r.remaining_ratio_above is not None:
        conds.append(f"残数比>{r.remaining_ratio_above:g}")
    if r.idle_seconds_above is not None:
        conds.append(f"無人{r.idle_seconds_above:g}秒超")
    cond = " かつ ".join(conds) if conds else "(常時)"
    params = f" {r.params}" if r.params else ""
    return (
        f"[{'有効' if r.enabled else '無効'}] {r.id} 「{r.name}」 優先度{r.priority}\n"
        f"      条件: {cond}\n"
        f"      方針: {'+'.join(r.policies)}{params}"
    )


def validate_file(path: str) -> Tuple[bool, str]:
    """(OKか, 日本語の要約)を返す。"""
    try:
        with open(os.path.expanduser(path), "r", encoding="utf-8") as f:
            text = f.read()
    except (OSError, UnicodeDecodeError) as exc:
        return False, f"NG: ファイルを読めません: {exc}"
    try:
        rs = parse_rules_text(text)
    except RuleError as exc:
        return False, f"NG: このファイルは拒否されます(読み込み時は直前のルールを維持)\n  理由: {exc}"
    enabled = sum(1 for r in rs.rules if r.enabled)
    lines = [
        f"OK: ルール{len(rs.rules)}件(有効{enabled}件 / 無効{len(rs.rules) - enabled}件)、"
        f"最小保持時間{rs.min_hold_seconds:g}秒"
    ]
    for r in rs.rules:
        lines.append("  " + describe_rule(r))
    if rs.rules and not enabled:
        lines.append("  注意: 有効なルールが0件です(現行動作のまま変わりません)")
    return True, "\n".join(lines)


def main(argv: Optional[List[str]] = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if len(argv) != 2 or argv[0] != "validate":
        print("使い方: python -m nokosahen_core.behavior_rules validate <ルールファイル>")
        return 2
    ok, summary = validate_file(argv[1])
    print(summary)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
