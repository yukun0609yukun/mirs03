"""スケジュール(「何時何分にどこへ行き、どの状態になるか」の一覧)の検証・日本語の要約・木へのコンパイル
(純Python、ROS非依存、外部ライブラリ無し)。docs/schedule.md。

【2026-10-23】スケジュールは「木(behavior_tree)を直接書かなくても、時間割として並べるだけで動作を決められる」
ようにするための入口。コンパイル結果は既存のビヘイビアツリーJSON(source:"schedule")で、既存の木の検証
(behavior_tree.validate_document)をそのまま通る。新しい実行系は無く、使うノードは
selector / sequence / time_between / weekday_in / goto_pin / go_to_entrance / hold_stop / normal / approach / scan / face /
quiet / announce / speed_scale / set_boundary / set_display / rules_table だけ。木の原則(ロボットを直接動かさない、安全層は
木の外で常に有効)はそのまま。時間割に載っていない時間は rules_table(既存のルール表)で動く。

スケジュールJSON:
  {"version":1, "name":"文字列(60字以内)", "run_on_startup":false,
   "entries":[{"id":"英数字_-の1〜32字", "start":"HH:MM", "end":"HH:MM"(省略可), "weekdays":["mon",...](省略可=毎日),
     "destination":{"type":"pin","pin_id":"...","pin_name":"..."} | {"type":"entrance"} | {"type":"stay"},
     "state":{"policy":"stay|normal|approach|scan|face|quiet", "announce_interval_seconds":10..600, "speed_scale":0.3..1.0},
     "boundary":{"mode":"keep|set|release", "rect":{"x_min","y_min","x_max","y_max"}(setのとき必須)},
     "display":{"mode":"keep|customer|survey|ocrtest"}(省略=keep=頭部ディスプレイは何も変えない),
     "note":"メモ"}]}
  endを省略すると「同じ曜日で次に始まる項目の開始時刻」、無ければ24:00。endがstartより早い場合は日付またぎ(例 22:00〜02:00)。
"""

from __future__ import annotations

import datetime
import json
import math
import re
from typing import Any, Dict, List, Optional, Tuple

from . import behavior_rules as br
from . import behavior_tree as bt

SCHEDULE_VERSION = 1
MAX_ENTRIES = 50
MAX_NAME_LEN = 60
MAX_NOTE_LEN = 200
MAX_PIN_NAME_LEN = br.PIN_NAME_MAX_LEN

POLICIES = ("stay", "normal", "approach", "scan", "face", "quiet")
DESTINATION_TYPES = ("pin", "entrance", "stay")
BOUNDARY_MODES = ("keep", "set", "release")
# 頭部ディスプレイ(iPad)の表示モード。keep=何も変えない(既定)
DISPLAY_MODES = ("keep",) + bt.DISPLAY_MODES
DISPLAY_JA = {"keep": "そのまま", "customer": "お客さん向け", "survey": "アンケート", "ocrtest": "OCRテスト"}
RECT_KEYS = bt.BOUNDARY_RECT_KEYS
ENTRY_NAME_PREFIX = "entry:"
MIN_RECT_SIZE_M = 0.8  # 動ける範囲(boundary.set)の最小サイズ(幅・高さとも)。これ未満はエラー(機体は50x50cm級で、1mmや数cm四方では動けない)
RECT_MARGIN_M = 0.05   # 地図の端・元の範囲の端との比較の許容(これ以内のはみ出しは警告しない)

_TOP_KEYS = {"version", "name", "entries", "run_on_startup"}
_ENTRY_KEYS = {"id", "start", "end", "weekdays", "destination", "state", "boundary", "display", "note"}
_DISPLAY_KEYS = {"mode"}
_DEST_KEYS = {"type", "pin_id", "pin_name"}
_STATE_KEYS = {"policy", "announce_interval_seconds", "speed_scale"}
_BOUNDARY_KEYS = {"mode", "rect"}
_ID_RE = re.compile(r"^[A-Za-z0-9_\-]{1,32}$")
_HHMM_RE = re.compile(r"^(\d{2}):(\d{2})$")
_DAY = 1440
_WEEK = 7 * _DAY

WEEKDAY_JA = {"mon": "月", "tue": "火", "wed": "水", "thu": "木", "fri": "金", "sat": "土", "sun": "日"}
POLICY_JA = {
    "stay": "待機", "normal": "通常動作", "approach": "接近(人に近づく)", "scan": "見回し", "face": "人の方を向く(その場で旋回)", "quiet": "静か(発話なし)",
}


class ScheduleError(ValueError):
    """スケジュールの検証エラー(1件)。"""


# --------------------------------------------------------------------------
# 検証
# --------------------------------------------------------------------------


def _is_num(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _hhmm(text: Any, where: str, allow_24: bool = False) -> int:
    if not isinstance(text, str):
        raise ScheduleError(f"{where}: 'HH:MM' の文字列にしてください: {text!r}")
    m = _HHMM_RE.match(text)
    if not m:
        raise ScheduleError(f"{where}: 時刻の書式が不正です(例 '09:30'): {text!r}")
    h, mi = int(m.group(1)), int(m.group(2))
    if allow_24 and h == 24 and mi == 0:
        return _DAY
    if not (0 <= h <= 23 and 0 <= mi <= 59):
        raise ScheduleError(f"{where}: 時刻が範囲外です{'(終了は24:00まで)' if allow_24 else ''}: {text!r}")
    return h * 60 + mi


def fmt_minutes(minute: int) -> str:
    return "%02d:%02d" % (minute // 60, minute % 60)


def _check_keys(obj: dict, allowed: set, where: str) -> None:
    unknown = sorted(set(obj) - allowed)
    if unknown:
        raise ScheduleError(f"{where}: 未知のキー {unknown}(使えるキー: {sorted(allowed)})")


def _clean_destination(raw: Any, where: str) -> Dict[str, Any]:
    if not isinstance(raw, dict):
        raise ScheduleError(f"{where}.destination: オブジェクトにしてください")
    _check_keys(raw, _DEST_KEYS, f"{where}.destination")
    typ = raw.get("type")
    if typ not in DESTINATION_TYPES:
        raise ScheduleError(f"{where}.destination.type: {list(DESTINATION_TYPES)} のいずれかにしてください: {typ!r}")
    if typ != "pin":
        extra = [k for k in ("pin_id", "pin_name") if k in raw]
        if extra:
            raise ScheduleError(f"{where}.destination: type={typ} では {extra} を指定できません")
        return {"type": typ}
    pid = raw.get("pin_id")
    if not isinstance(pid, str) or not _ID_RE.match(pid):
        raise ScheduleError(f"{where}.destination.pin_id: ピンのid(英数字/_/-の1〜32文字)が必須です: {pid!r}")
    out: Dict[str, Any] = {"type": "pin", "pin_id": pid}
    if raw.get("pin_name") not in (None, ""):
        pname = raw["pin_name"]
        if not isinstance(pname, str) or len(pname) > MAX_PIN_NAME_LEN:
            raise ScheduleError(f"{where}.destination.pin_name: {MAX_PIN_NAME_LEN}文字以内の文字列にしてください")
        out["pin_name"] = pname
    return out


def _clean_state(raw: Any, where: str) -> Dict[str, Any]:
    if not isinstance(raw, dict):
        raise ScheduleError(f"{where}.state: オブジェクトにしてください")
    _check_keys(raw, _STATE_KEYS, f"{where}.state")
    pol = raw.get("policy")
    if pol not in POLICIES:
        raise ScheduleError(f"{where}.state.policy: {list(POLICIES)} のいずれかにしてください: {pol!r}")
    out: Dict[str, Any] = {"policy": pol}
    if raw.get("announce_interval_seconds") is not None:
        v = raw["announce_interval_seconds"]
        if not _is_num(v) or not br.ANNOUNCE_INTERVAL_MIN <= v <= br.ANNOUNCE_INTERVAL_MAX:
            raise ScheduleError(
                f"{where}.state.announce_interval_seconds: {br.ANNOUNCE_INTERVAL_MIN:g}〜{br.ANNOUNCE_INTERVAL_MAX:g}の数値にしてください: {v!r}")
        out["announce_interval_seconds"] = float(v)
    if raw.get("speed_scale") is not None:
        v = raw["speed_scale"]
        if not _is_num(v) or not br.SPEED_SCALE_MIN <= v <= br.SPEED_SCALE_MAX:
            raise ScheduleError(
                f"{where}.state.speed_scale: {br.SPEED_SCALE_MIN:g}〜{br.SPEED_SCALE_MAX:g}の数値にしてください: {v!r}")
        out["speed_scale"] = float(v)
    return out


def _clean_boundary(raw: Any, where: str) -> Dict[str, Any]:
    if not isinstance(raw, dict):
        raise ScheduleError(f"{where}.boundary: オブジェクトにしてください")
    _check_keys(raw, _BOUNDARY_KEYS, f"{where}.boundary")
    mode = raw.get("mode")
    if mode not in BOUNDARY_MODES:
        raise ScheduleError(f"{where}.boundary.mode: {list(BOUNDARY_MODES)} のいずれかにしてください: {mode!r}")
    if mode != "set":
        if raw.get("rect") is not None:
            raise ScheduleError(f"{where}.boundary.rect: mode={mode} では範囲を指定できません")
        return {"mode": mode}
    rect = raw.get("rect")
    if not isinstance(rect, dict):
        raise ScheduleError(f"{where}.boundary.rect: mode=setのときは必須です({list(RECT_KEYS)})")
    _check_keys(rect, set(RECT_KEYS), f"{where}.boundary.rect")
    out: Dict[str, float] = {}
    for k in RECT_KEYS:
        v = rect.get(k)
        if not _is_num(v) or abs(v) > bt.BOUNDARY_COORD_MAX_M:
            raise ScheduleError(
                f"{where}.boundary.rect.{k}: ±{bt.BOUNDARY_COORD_MAX_M:g}以内の有限の数値が必須です: {v!r}")
        out[k] = float(v)
    if out["x_min"] >= out["x_max"] or out["y_min"] >= out["y_max"]:
        raise ScheduleError(f"{where}.boundary.rect: 範囲が不正です(x_min<x_max かつ y_min<y_max にしてください)")
    w, h = out["x_max"] - out["x_min"], out["y_max"] - out["y_min"]
    if w < MIN_RECT_SIZE_M or h < MIN_RECT_SIZE_M:
        raise ScheduleError(
            f"{where}.boundary.rect: 範囲が小さすぎます(幅・高さとも{MIN_RECT_SIZE_M:g} m以上にしてください。"
            f"いま 幅{w:.3g} m × 高さ{h:.3g} m)")
    return {"mode": "set", "rect": out}


def _clean_display(raw: Any, where: str) -> Optional[Dict[str, Any]]:
    """表示モード。keep(何も変えない)は None(キー自体を保存しない)、それ以外は {"mode": ...}。"""
    if not isinstance(raw, dict):
        raise ScheduleError(f"{where}.display: オブジェクトにしてください")
    _check_keys(raw, _DISPLAY_KEYS, f"{where}.display")
    mode = raw.get("mode")
    if not isinstance(mode, str) or mode not in DISPLAY_MODES:
        raise ScheduleError(f"{where}.display.mode: {list(DISPLAY_MODES)} のいずれかにしてください: {mode!r}")
    return None if mode == "keep" else {"mode": mode}


def entry_display_mode(e: Dict[str, Any]) -> str:
    """項目の表示モード(keep / customer / survey / ocrtest)。"""
    d = e.get("display")
    return d.get("mode", "keep") if isinstance(d, dict) else "keep"


def _clean_entry(raw: Any, index: int) -> Dict[str, Any]:
    where = f"項目{index + 1}"
    if not isinstance(raw, dict):
        raise ScheduleError(f"{where}: オブジェクトにしてください")
    _check_keys(raw, _ENTRY_KEYS, where)
    eid = raw.get("id")
    if not isinstance(eid, str) or not _ID_RE.match(eid):
        raise ScheduleError(f"{where}.id: 英数字/_/-の1〜32文字にしてください: {eid!r}")
    where = f"項目{index + 1}(id={eid})"
    if "start" not in raw:
        raise ScheduleError(f"{where}.start: 必須です('HH:MM')")
    start = _hhmm(raw["start"], f"{where}.start")
    out: Dict[str, Any] = {"id": eid, "start": fmt_minutes(start)}
    if raw.get("end") is not None:
        end = _hhmm(raw["end"], f"{where}.end", allow_24=True)
        if end == 0:
            # end:"00:00" は「その日の終わり(24:00)」の意味に正規化する。開始が 00:00 のときに「開始と終了が同じ」という
            # 不親切なエラーになるのを防ぐ(開始が 00:00 以外でも、22:00→00:00 と 22:00→24:00 は同じ区間)。
            end = _DAY
        if end == start:
            raise ScheduleError(f"{where}: 開始と終了が同じ時刻です(24時間は 00:00〜24:00 で指定してください)")
        out["end"] = fmt_minutes(end)
    if raw.get("weekdays") is not None:
        wd = raw["weekdays"]
        if not isinstance(wd, list) or not wd or not all(isinstance(x, str) for x in wd):
            raise ScheduleError(f"{where}.weekdays: 空でない曜日の配列にしてください(省略=毎日)")
        for x in wd:
            if x not in br.WEEKDAYS:
                raise ScheduleError(f"{where}.weekdays: 不明な曜日 {x!r}(使える値: {list(br.WEEKDAYS)})")
        out["weekdays"] = [d for d in br.WEEKDAYS if d in wd]
    if "destination" not in raw:
        raise ScheduleError(f"{where}.destination: 必須です")
    out["destination"] = _clean_destination(raw["destination"], where)
    if "state" not in raw:
        raise ScheduleError(f"{where}.state: 必須です")
    out["state"] = _clean_state(raw["state"], where)
    out["boundary"] = _clean_boundary(raw["boundary"], where) if raw.get("boundary") is not None else {"mode": "keep"}
    if raw.get("display") is not None:
        disp = _clean_display(raw["display"], where)
        if disp is not None:
            out["display"] = disp
    if raw.get("note") not in (None, ""):
        note = raw["note"]
        if not isinstance(note, str) or len(note) > MAX_NOTE_LEN:
            raise ScheduleError(f"{where}.note: {MAX_NOTE_LEN}文字以内の文字列にしてください")
        out["note"] = note
    return out


def _entry_days(e: Dict[str, Any]) -> List[int]:
    wd = e.get("weekdays")
    return [br.WEEKDAYS.index(d) for d in wd] if wd else list(range(7))


def resolve_windows(entries: List[Dict[str, Any]]) -> List[Tuple[int, int]]:
    """各項目の(開始分, 終了分)。endを省略した項目は「同じ曜日で次に始まる項目の開始」、無ければ24:00(1440)。
    終了が開始以下なら日付またぎ(終了は翌日)。"""
    out: List[Tuple[int, int]] = []
    starts = [_hhmm(e["start"], "start") for e in entries]
    for i, e in enumerate(entries):
        s = starts[i]
        if e.get("end"):
            out.append((s, _hhmm(e["end"], "end", allow_24=True)))
            continue
        mine = set(_entry_days(e))
        later = [starts[j] for j, f in enumerate(entries)
                 if j != i and starts[j] > s and mine & set(_entry_days(f))]
        out.append((s, min(later) if later else _DAY))
    return out


def _segments(entry: Dict[str, Any], window: Tuple[int, int]) -> List[Tuple[int, int]]:
    """週の中の区間[a,b)の一覧(分)。日付またぎは翌日へ続く区間として表す。"""
    s, e = window
    length = (e - s) if e > s else (e + _DAY - s)
    return [(d * _DAY + s, d * _DAY + s + length) for d in _entry_days(entry)]


def _overlap_days(a: List[Tuple[int, int]], b: List[Tuple[int, int]]) -> List[int]:
    days = set()
    for a0, a1 in a:
        for b0, b1 in b:
            for shift in (-_WEEK, 0, _WEEK):
                lo, hi = max(a0, b0 + shift), min(a1, b1 + shift)
                if lo < hi:
                    days.add((lo // _DAY) % 7)
    return sorted(days)


def validate_schedule(doc: Any) -> Tuple[List[str], List[str], Optional[Dict[str, Any]]]:
    """スケジュール(dict)を検証する。(エラー一覧[日本語], 警告一覧, 正規化したスケジュール)。
    エラーが1つでもあればNone(保存・実行しない)。コンパイル後の木の検証(behavior_tree.validate_document)も
    ここで行い、木として拒否される物(ノード数の超過など)もエラーにする。"""
    warnings: List[str] = []
    if not isinstance(doc, dict):
        return ['トップレベルはオブジェクト({"name":..,"entries":[..]})にしてください'], warnings, None
    try:
        _check_keys(doc, _TOP_KEYS, "トップレベル")
        version = doc.get("version", SCHEDULE_VERSION)
        if version != SCHEDULE_VERSION or isinstance(version, bool):
            raise ScheduleError(f"versionは{SCHEDULE_VERSION}のみ対応です: {version!r}")
        name = doc.get("name")
        if not isinstance(name, str) or not name.strip() or len(name.strip()) > MAX_NAME_LEN:
            raise ScheduleError(f"nameは1〜{MAX_NAME_LEN}文字の文字列が必須です")
        run_on_startup = doc.get("run_on_startup", False)
        if not isinstance(run_on_startup, bool):
            raise ScheduleError("run_on_startupはtrue/falseにしてください")
        entries_raw = doc.get("entries")
        if not isinstance(entries_raw, list) or not entries_raw:
            raise ScheduleError("entriesは1つ以上の項目の配列にしてください")
        if len(entries_raw) > MAX_ENTRIES:
            raise ScheduleError(f"項目が多すぎます(最大{MAX_ENTRIES}個、いま{len(entries_raw)}個)")
        entries = [_clean_entry(e, i) for i, e in enumerate(entries_raw)]
        seen: Dict[str, int] = {}
        for i, e in enumerate(entries):
            if e["id"] in seen:
                raise ScheduleError(f"項目{i + 1}: idが項目{seen[e['id']] + 1}と重複しています: {e['id']!r}")
            seen[e["id"]] = i
    except ScheduleError as exc:
        return [str(exc)], warnings, None
    norm = {
        "version": SCHEDULE_VERSION,
        "name": name.strip(),
        "run_on_startup": run_on_startup,
        "entries": entries,
    }
    warnings.extend(_warnings(entries))
    try:
        tree = compile_schedule(norm)
    except Exception as exc:  # noqa: BLE001 - コンパイルできないものは保存・実行させない
        return [f"木へ変換できません: {exc}"], warnings, None
    errors, _tw, _norm_tree = bt.validate_document(tree)
    if errors:
        return [f"木へ変換した結果が拒否されました: {errors[0]}"], warnings, None
    return [], warnings, norm


def _warnings(entries: List[Dict[str, Any]]) -> List[str]:
    out: List[str] = []
    windows = resolve_windows(entries)
    segs = [_segments(e, w) for e, w in zip(entries, windows)]
    for i in range(len(entries)):
        for j in range(i + 1, len(entries)):
            days = _overlap_days(segs[i], segs[j])
            if days:
                dl = "".join(WEEKDAY_JA[br.WEEKDAYS[d]] for d in days)
                out.append(
                    f"項目{i + 1}(id={entries[i]['id']})と項目{j + 1}(id={entries[j]['id']})の時間が重なっています"
                    f"(曜日: {dl})。重なる間は上にある項目{i + 1}が優先されます")
    for i, e in enumerate(entries):
        who = f"項目{i + 1}(id={e['id']})"
        dest, state = e["destination"]["type"], e["state"]["policy"]
        if dest != "stay" and state in ("normal", "approach", "scan", "face"):
            out.append(
                f"{who}: 行き先がある項目は、到着後その場で待機します(「{POLICY_JA[state]}」の動作は行いません)。"
                "接近・見回し・向くをさせたい時間は、行き先を「その場」にしてください")
        if state == "quiet" and e["state"].get("announce_interval_seconds") is not None:
            out.append(f"{who}: 静か(発話なし)なので呼びかけ間隔は効きません")
        if e["boundary"]["mode"] == "release" and dest == "stay":
            out.append(f"{who}: 範囲を解除していますが、行き先が「その場」なので移動しません")
        if entry_display_mode(e) == "survey":
            out.append(f"{who}: アンケート表示中は、回答中ロボットが停止します。回答中は画面の切替を保留します"
                       "(回答が消えないよう、終わってから次の表示へ切り替えます)")
    return out


# --------------------------------------------------------------------------
# 日本語の要約
# --------------------------------------------------------------------------


def _time_text(e: Dict[str, Any], window: Tuple[int, int]) -> str:
    s, end = window
    if end <= s and end != _DAY:
        return f"{fmt_minutes(s)}〜翌{fmt_minutes(end)}"
    return f"{fmt_minutes(s)}〜{fmt_minutes(end)}"


def summarize_entry(e: Dict[str, Any], window: Tuple[int, int]) -> str:
    days = ""
    if e.get("weekdays"):
        days = "(" + "".join(WEEKDAY_JA[d] for d in e["weekdays"]) + ")"
    dest = e["destination"]
    if dest["type"] == "pin":
        dest_text = f"{dest.get('pin_name') or dest['pin_id']}へ行く"
    elif dest["type"] == "entrance":
        dest_text = "入口へ行く"
    else:
        dest_text = "その場にいる"
    st = e["state"]
    parts = [POLICY_JA[st["policy"]]]
    if st.get("announce_interval_seconds") is not None:
        parts.append(f"呼びかけ{st['announce_interval_seconds']:g}秒間隔")
    if st.get("speed_scale") is not None:
        parts.append(f"速度×{st['speed_scale']:g}")
    text = f"{_time_text(e, window)}{days} {dest_text} → {'、'.join(parts)}"
    b = e["boundary"]
    if b["mode"] == "release":
        text += "、範囲は解除してNav2で走行" if dest["type"] != "stay" else "、範囲は解除"
    elif b["mode"] == "set":
        r = b["rect"]
        text += "、範囲は(%g,%g)〜(%g,%g)mに制限" % (r["x_min"], r["y_min"], r["x_max"], r["y_max"])
    mode = entry_display_mode(e)
    if mode != "keep":
        text += f"、頭部ディスプレイは{DISPLAY_JA[mode]}を表示"
    return text


def summarize(norm: Dict[str, Any]) -> List[str]:
    """項目ごとに1行(項目の並び順どおり)。"""
    windows = resolve_windows(norm["entries"])
    return [summarize_entry(e, w) for e, w in zip(norm["entries"], windows)]


def rect_warnings(norm: Dict[str, Any], map_extent: Optional[Dict[str, float]] = None,
                  original: Optional[Dict[str, float]] = None) -> List[str]:
    """boundary.set の矩形に対する警告(保存は妨げない)。取れない情報(None)の分は省略する。
    map_extent: 読み込み中の地図の範囲 {"x_min","y_min","x_max","y_max"}[m]。地図の外にはみ出すと警告。
    original: 機体の元の範囲(BoundaryControllerの現在の設定の外接矩形)。同じ形式。無効(制限なし)なら None を渡す。
    矩形が元の範囲より広い(はみ出す)と「元の範囲の制限を外します」と警告する。"""
    out: List[str] = []
    for i, e in enumerate(norm.get("entries", [])):
        b = e.get("boundary") or {}
        if b.get("mode") != "set":
            continue
        r = b["rect"]
        who = f"項目{i + 1}(id={e['id']})"
        if map_extent is not None:
            over = (r["x_min"] < map_extent["x_min"] - RECT_MARGIN_M or r["y_min"] < map_extent["y_min"] - RECT_MARGIN_M
                    or r["x_max"] > map_extent["x_max"] + RECT_MARGIN_M or r["y_max"] > map_extent["y_max"] + RECT_MARGIN_M)
            if over:
                out.append(
                    f"{who}の動ける範囲が、読み込み中の地図の外にはみ出しています"
                    f"(地図は x {map_extent['x_min']:.1f}〜{map_extent['x_max']:.1f}、y {map_extent['y_min']:.1f}〜{map_extent['y_max']:.1f} m)。"
                    "地図の外は走行できません。")
        if original is not None:
            wider = (r["x_min"] < original["x_min"] - RECT_MARGIN_M or r["y_min"] < original["y_min"] - RECT_MARGIN_M
                     or r["x_max"] > original["x_max"] + RECT_MARGIN_M or r["y_max"] > original["y_max"] + RECT_MARGIN_M)
            if wider:
                out.append(
                    f"{who}の動ける範囲が、機体の元の範囲"
                    f"(x {original['x_min']:.1f}〜{original['x_max']:.1f}、y {original['y_min']:.1f}〜{original['y_max']:.1f} m)より広いため、"
                    "元の範囲の制限を外します(この項目の間は、元の範囲の外にも動けます)。")
    return out


def preview(doc: Any) -> Dict[str, Any]:
    """POST /api/schedule/preview の応答本体。"""
    errors, warnings, norm = validate_schedule(doc)
    return {
        "ok": not errors,
        "summary": summarize(norm) if norm is not None else [],
        "errors": errors,
        "warnings": warnings,
    }


# --------------------------------------------------------------------------
# 木へのコンパイル
# --------------------------------------------------------------------------


def _n(typ: str, name: str = "", params: Optional[dict] = None, children: Optional[list] = None,
       description: str = "") -> dict:
    d: Dict[str, Any] = {"type": typ}
    if name:
        d["name"] = name[: bt.MAX_NAME_LEN]
    if description:
        d["description"] = description[:300]
    if params:
        d["params"] = params
    if children is not None:
        d["children"] = children
    return d


def _range_text(a: int, b: int) -> str:
    return f"{fmt_minutes(a)}-{fmt_minutes(b)}" if b != _DAY else f"{fmt_minutes(a)}-24:00"


def _window_node(e: Dict[str, Any], window: Tuple[int, int]) -> dict:
    """項目の「いつ」を表す条件ノード1つ(構造は parse_windows が読み戻す)。"""
    s, end = window
    wraps = end <= s and end != _DAY
    days = [br.WEEKDAYS[d] for d in _entry_days(e)] if e.get("weekdays") else None
    if not wraps:
        tb = _n("time_between", params={"range": _range_text(s, end)})
        if days is None:
            return tb
        return _n("sequence", "window", children=[_n("weekday_in", params={"days": days}), tb])
    if days is None:
        return _n("time_between", params={"range": _range_text(s, end)})  # time_betweenが日付またぎに対応
    # 曜日つきの日付またぎ: 開始の曜日の夜と、その翌日の朝を、それぞれの曜日で判定する
    next_days = [br.WEEKDAYS[(br.WEEKDAYS.index(d) + 1) % 7] for d in days]
    return _n("selector", "window", children=[
        _n("sequence", "evening", children=[_n("weekday_in", params={"days": days}),
                                           _n("time_between", params={"range": _range_text(s, _DAY)})]),
        _n("sequence", "morning", children=[_n("weekday_in", params={"days": next_days}),
                                           _n("time_between", params={"range": _range_text(0, end)})]),
    ])


def _entry_node(e: Dict[str, Any], window: Tuple[int, int]) -> dict:
    kids = [_window_node(e, window)]
    b = e["boundary"]
    if b["mode"] == "release":
        kids.append(_n("set_boundary", "範囲を解除", {"mode": "release"}))
    elif b["mode"] == "set":
        kids.append(_n("set_boundary", "範囲を制限", dict({"mode": "set"}, **b["rect"])))
    disp = entry_display_mode(e)
    if disp != "keep":
        kids.append(_n("set_display", "頭部ディスプレイの画面", {"mode": disp}))
    dest = e["destination"]
    if dest["type"] == "pin":
        p = {"pin_id": dest["pin_id"]}
        if dest.get("pin_name"):
            p["pin_name"] = dest["pin_name"]
        kids.append(_n("goto_pin", "ピンへ行く", p))
    elif dest["type"] == "entrance":
        kids.append(_n("go_to_entrance", "入口へ行く"))
    st = e["state"]
    pol = st["policy"]
    kids.append({
        "stay": lambda: _n("hold_stop", "待機"),
        "normal": lambda: _n("normal", "通常動作"),
        "approach": lambda: _n("approach", "接近を許可", {"enabled": True}),
        "scan": lambda: _n("scan", "見回しを許可", {"enabled": True}),
        "face": lambda: _n("face", "向くを許可", {"enabled": True}),
        "quiet": lambda: _n("quiet", "静か"),
    }[pol]())
    if st.get("announce_interval_seconds") is not None:
        kids.append(_n("announce", "呼びかけ間隔", {"interval_seconds": st["announce_interval_seconds"]}))
    if st.get("speed_scale") is not None:
        kids.append(_n("speed_scale", "速度", {"scale": st["speed_scale"]}))
    return _n("sequence", ENTRY_NAME_PREFIX + e["id"], children=kids,
              description=summarize_entry(e, window))


def compile_schedule(norm: Dict[str, Any]) -> Dict[str, Any]:
    """検証済み(正規化済み)のスケジュールを、ビヘイビアツリーの文書にする。
    構造: selector[ 項目1(sequence: 時間条件, 範囲, 行き先, 状態...), 項目2, ..., rules_table ]。
    上の項目が優先。どの項目にも当たらない時間は rules_table(既存のルール表)=従来の動作。"""
    windows = resolve_windows(norm["entries"])
    kids = [_entry_node(e, w) for e, w in zip(norm["entries"], windows)]
    kids.append(_n("rules_table", "時間割に無い時間はルール表"))
    return {
        "version": 1,
        "description": ("スケジュール「%s」から自動生成した木(source=schedule)。木のページで編集した内容は、"
                        "スケジュールを実行すると上書きされます。" % norm["name"])[:500],
        "min_hold_seconds": 0.0,
        "enabled": True,
        "source": "schedule",
        "run_on_startup": bool(norm.get("run_on_startup", False)),
        "root": _n("selector", "スケジュール: " + norm["name"], children=kids),
    }


# --------------------------------------------------------------------------
# 頭部ディスプレイの画面の要求(director -> ダッシュボード、/nokosahen/display_request)
# --------------------------------------------------------------------------

DISPLAY_REQUEST_TOPIC = "/nokosahen/display_request"
DISPLAY_REQUEST_REPEAT_SEC = 5.0  # 変化したとき1回 + スケジュール実行中はこの間隔で再送
DISPLAY_REQUEST_TTL_SEC = 20.0    # 受け手(ダッシュボード)はこれより古い要求を「要求なし(keep)」として扱う(directorが止まった時の安全側)


def display_request_message(mode: str, entry_id: Optional[str], stamp: float) -> Dict[str, Any]:
    """{"mode":"keep|customer|survey|ocrtest","source":"schedule","entry_id":..|None,"stamp":..}。
    keep = いま表示モードの要求は無い(スケジュール停止中・項目の外・displayを指定していない項目)。"""
    return {"mode": mode if mode in DISPLAY_MODES else "keep", "source": "schedule",
            "entry_id": entry_id, "stamp": float(stamp)}


def parse_display_request(text: Any) -> Optional[Dict[str, Any]]:
    """受信したJSON文字列を検証する。不正(型・未知のモード・source違い)は None(=無視)。"""
    try:
        data = json.loads(text) if isinstance(text, (str, bytes)) else None
    except (ValueError, RecursionError):
        return None
    if not isinstance(data, dict) or data.get("source") != "schedule":
        return None
    mode = data.get("mode")
    if not isinstance(mode, str) or mode not in DISPLAY_MODES:
        return None
    entry_id = data.get("entry_id")
    if entry_id is not None and not (isinstance(entry_id, str) and len(entry_id) <= 64):
        entry_id = None
    return {"mode": mode, "entry_id": entry_id}


# --------------------------------------------------------------------------
# 読み込み(厳格なJSON)
# --------------------------------------------------------------------------


def _reject_constant(name: str):
    raise ScheduleError(f"JSONに{name}(NaN/Infinity)は使えません")


def parse_schedule_text(text: str) -> Any:
    """JSON文字列を読む(NaN/Infinityは拒否)。不正ならScheduleError。"""
    try:
        return json.loads(text, parse_constant=_reject_constant)
    except ScheduleError:
        raise
    except (ValueError, RecursionError) as exc:
        raise ScheduleError(f"JSONとして読めません: {exc}")


def document_text(norm: Dict[str, Any]) -> str:
    return json.dumps(norm, ensure_ascii=False, indent=2, allow_nan=False) + "\n"


# --------------------------------------------------------------------------
# 実行時の状態(コンパイル済みの木とノードの評価結果から、いまの項目・次の項目を求める)
# --------------------------------------------------------------------------

Window = Tuple[Optional[set], int, int]  # (開始日の曜日の集合 or None=毎日, 開始分, 終了分(終了<=開始なら翌日))


def _parse_range(text: str) -> Tuple[int, int]:
    return br.parse_time_range(text, "range")


def parse_windows(entry_node: dict) -> List[Window]:
    """_window_node が作った条件ノードから、(曜日, 開始, 終了)を読み戻す。"""
    kids = entry_node.get("children") or []
    if not kids:
        return []
    w = kids[0]
    typ = w.get("type")
    if typ == "time_between":
        s, e = _parse_range(w["params"]["range"])
        return [(None, s, e)]
    if typ == "sequence":
        days = None
        rng = None
        for c in w.get("children", []):
            if c["type"] == "weekday_in":
                days = {br.WEEKDAYS.index(d) for d in c["params"]["days"]}
            elif c["type"] == "time_between":
                rng = _parse_range(c["params"]["range"])
        return [(days, rng[0], rng[1])] if rng else []
    if typ == "selector":  # 曜日つきの日付またぎ
        evening, morning = w["children"][0], w["children"][1]
        days = {br.WEEKDAYS.index(d) for d in evening["children"][0]["params"]["days"]}
        s, _e = _parse_range(evening["children"][1]["params"]["range"])
        _s0, e = _parse_range(morning["children"][1]["params"]["range"])
        return [(days, s, e)]
    return []


def _window_at(win: Window, now: datetime.datetime) -> Optional[datetime.datetime]:
    """nowを含む時間枠なら、その終了時刻。含まなければNone。"""
    days, s, e = win
    for back in (0, 1):
        day = (now - datetime.timedelta(days=back)).date()
        if days is not None and day.weekday() not in days:
            continue
        start = datetime.datetime.combine(day, datetime.time()) + datetime.timedelta(minutes=s)
        length = (e - s) if e > s else (e + _DAY - s)
        end = start + datetime.timedelta(minutes=length)
        if start <= now < end:
            return end
    return None


def _next_start(win: Window, now: datetime.datetime) -> Optional[datetime.datetime]:
    days, s, _e = win
    best = None
    for k in range(0, 8):
        day = (now + datetime.timedelta(days=k)).date()
        if days is not None and day.weekday() not in days:
            continue
        start = datetime.datetime.combine(day, datetime.time()) + datetime.timedelta(minutes=s)
        if start > now and (best is None or start < best):
            best = start
    return best


def entry_nodes(tree_doc: dict) -> List[dict]:
    root = tree_doc.get("root") or {}
    return [c for c in root.get("children", []) if str(c.get("name", "")).startswith(ENTRY_NAME_PREFIX)]


def runtime_state(tree_doc: dict, statuses: Dict[str, str], now: datetime.datetime,
                  running: bool, armed: bool) -> Dict[str, Any]:
    """木の状態配信(/nokosahen/behavior_tree/status)の "schedule" に載せる中身。
    running: いまスケジュールで動いているか(木が有効かつ実行の合図済み)。active_entry_idは、木の評価で
    実際に採用された項目(時計が合っていない間や、どの項目にも当たらない時間は None=ルール表)。"""
    clock_bad = br.clock_suspect(now)
    out: Dict[str, Any] = {
        "running": bool(running), "armed": bool(armed), "clock_suspect": clock_bad,
        "active_entry_id": None, "active_until": None, "next_entry_id": None, "next_entry_at": None,
        "entry_count": 0,
    }
    nodes = entry_nodes(tree_doc)
    out["entry_count"] = len(nodes)
    if not running or clock_bad:
        return out
    root_kids = (tree_doc.get("root") or {}).get("children", [])
    active = None
    for i, c in enumerate(root_kids):
        if str(c.get("name", "")).startswith(ENTRY_NAME_PREFIX) and statuses.get(f"0.{i}") in (bt.SUCCESS, bt.RUNNING):
            active = c
            break
    if active is not None:
        out["active_entry_id"] = active["name"][len(ENTRY_NAME_PREFIX):]
        ends = [e for e in (_window_at(w, now) for w in parse_windows(active)) if e is not None]
        if ends:
            out["active_until"] = max(ends).isoformat(timespec="seconds")
    best = None
    for c in nodes:
        for w in parse_windows(c):
            t = _next_start(w, now)
            if t is not None and (best is None or t < best[0]):
                best = (t, c["name"][len(ENTRY_NAME_PREFIX):])
    if best is not None:
        out["next_entry_at"] = best[0].isoformat(timespec="seconds")
        out["next_entry_id"] = best[1]
    return out
