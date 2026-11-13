"""地図ピン(名前付きの場所)と、目的地の精度向上(スナップ)の純粋部分(ROS非依存)。

【2026-10-07追加】
1. ピン: 地図(mapフレーム)上の {id, name, x, y, yaw_deg, color, created_at, map_name}。
   - 地図作成中(SLAM)でも、localization中でも、いつでも「現在位置にピンを立てる」/「地図をクリックして
     ピンを立てる」ことができる。後で「このピンへ行く」と、保存した姿勢(x, y, yaw)をそのまま
     /goal_pose として送る(再クリックによる量子化はしない)。
   - 保存: 作業中のピンは常にセッションファイル(~/.nokosahen_pins_session.json)へ。地図を保存する時は
     <maps_dir>/<name>.pins.json(.view.jsonと同じ流儀)へ書き、地図を読み込む時はそこから復元する。
     地図を読み込み中の編集は、その<name>.pins.jsonへも即時に書く。新規マッピング/リセットで
     セッションのピンは空にする。書き込みは全て一時ファイル+os.replaceで原子的。
   - 【SLAMの注意】地図作成中はループクロージャで地図が後から少しずれることがあり、先に立てたピンが
     実際の場所から少しずれる場合がある。地図を保存してlocalizationで読み込んだ後に、ピンの場所で
     「現在位置に更新」で立て直すのが確実(docs/map_pins.md)。
2. snap_goal_to_free(): クリックした目的地が占有/インフレーション内/未知のセルに乗っていると、Nav2の
   プランナ(Navfn、tolerance)が「最も近い有効点」に勝手に寄せ、意図とずれた場所で止まる。
   サーバー側で地図の「最も近い空きセル(周囲のロボット半径+余裕に占有が無い)」へ先に寄せ、
   ずらした距離をUIへ返す(マーカーに反映)。ピンの目的地(goto)は保存した姿勢を厳密に送るため寄せない。
"""

from __future__ import annotations

import json
import math
import os
import re
import tempfile
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

PIN_MAX_COUNT = 200
PIN_NAME_MAX_LEN = 40
PIN_ABS_LIMIT_M = 1000.0
PIN_DEFAULT_COLORS = [
    "#e11d48", "#2563eb", "#16a34a", "#d97706", "#7c3aed", "#0891b2", "#db2777", "#4d7c0f",
]
_COLOR_RE = re.compile(r"^#[0-9a-fA-F]{6}$")
_SAFE_MAP_NAME = re.compile(r"^[A-Za-z0-9_\-]{1,64}$")
_ID_RE = re.compile(r"^[A-Za-z0-9_\-]{1,32}$")

SESSION_PATH_DEFAULT = "~/.nokosahen_pins_session.json"


class PinError(ValueError):
    """ピン操作のリクエストが不正。kind: bad_request(400) / not_found(404) / limit(409)。"""

    def __init__(self, message: str, kind: str = "bad_request") -> None:
        super().__init__(message)
        self.message = message
        self.kind = kind


# ------------------------------------------------------------------ 検証・変換
def normalize_deg(a: float) -> float:
    """度を(-180, 180]へ。"""
    a = math.fmod(a, 360.0)
    if a <= -180.0:
        a += 360.0
    elif a > 180.0:
        a -= 360.0
    return a


def yaw_deg_to_quaternion(yaw_deg: float) -> Tuple[float, float, float, float]:
    """平面内の向き(度)を(x, y, z, w)へ。"""
    half = math.radians(yaw_deg) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _finite_number(value: Any, label: str, limit: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PinError(f"{label}は数値で指定してください")
    v = float(value)
    if not math.isfinite(v):
        raise PinError(f"{label}は有限の数値で指定してください")
    if abs(v) > limit:
        raise PinError(f"{label}が範囲外です(±{limit:g}まで)")
    return v


def parse_pose(data: Any) -> Tuple[float, float, float]:
    """{x, y, yaw_deg}を検証して(x, y, yaw_deg(-180,180])を返す。"""
    if not isinstance(data, dict):
        raise PinError("位置は {x, y, yaw_deg} で指定してください")
    x = _finite_number(data.get("x"), "x", PIN_ABS_LIMIT_M)
    y = _finite_number(data.get("y"), "y", PIN_ABS_LIMIT_M)
    yaw = _finite_number(data.get("yaw_deg", 0.0), "yaw_deg", 36000.0)
    return x, y, normalize_deg(yaw)


def clean_name(value: Any) -> str:
    if value is None:
        return ""
    if not isinstance(value, str):
        raise PinError("名前は文字列で指定してください")
    name = " ".join(value.split())  # 前後・連続の空白と改行/タブを整える
    if len(name) > PIN_NAME_MAX_LEN:
        raise PinError(f"名前は{PIN_NAME_MAX_LEN}文字以内にしてください")
    return name


def clean_color(value: Any) -> Optional[str]:
    """#rrggbbのみ受け付ける(CSS注入防止)。None/空は未指定(None)。"""
    if value is None or value == "":
        return None
    if not isinstance(value, str) or not _COLOR_RE.match(value):
        raise PinError("色は #rrggbb 形式で指定してください")
    return value.lower()


def _sanitize_loaded_pin(raw: Any) -> Optional[Dict[str, Any]]:
    """ファイルから読んだ1件を検証する。壊れた件はNone(読み飛ばす)。"""
    if not isinstance(raw, dict):
        return None
    try:
        x, y, yaw = parse_pose(raw)
        name = clean_name(raw.get("name"))
        color = clean_color(raw.get("color")) or PIN_DEFAULT_COLORS[0]
    except PinError:
        return None
    pid = raw.get("id")
    if not isinstance(pid, str) or not _ID_RE.match(pid):
        return None
    created = raw.get("created_at")
    if isinstance(created, bool) or not isinstance(created, (int, float)) or not math.isfinite(created):
        created = 0.0
    mn = raw.get("map_name")
    mn = mn if isinstance(mn, str) and (mn == "" or _SAFE_MAP_NAME.match(mn)) else ""
    return {
        "id": pid, "name": name or "ピン", "x": x, "y": y, "yaw_deg": yaw,
        "color": color, "created_at": float(created), "map_name": mn,
    }


def _atomic_write(path: Path, obj: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=str(path.parent), prefix=path.name + ".", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(obj, f, ensure_ascii=False)
        os.replace(tmp, path)
    except Exception:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def _read_pins_file(path: Path) -> Tuple[List[Dict[str, Any]], Optional[str]]:
    """(ピン一覧, loaded_map) 。無い/壊れていれば([], None)。"""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:  # noqa: BLE001
        return [], None
    if not isinstance(data, dict):
        return [], None
    pins: List[Dict[str, Any]] = []
    seen = set()
    for raw in data.get("pins") if isinstance(data.get("pins"), list) else []:
        p = _sanitize_loaded_pin(raw)
        if p is not None and p["id"] not in seen and len(pins) < PIN_MAX_COUNT:
            seen.add(p["id"])
            pins.append(p)
    lm = data.get("loaded_map")
    return pins, (lm if isinstance(lm, str) and _SAFE_MAP_NAME.match(lm) else None)


# ------------------------------------------------------------------ ストア
class PinStore:
    """作業中のピン一覧と、セッション/地図ごとのJSONの読み書き。"""

    def __init__(self, maps_dir: str, session_path: str = SESSION_PATH_DEFAULT) -> None:
        self.maps_dir = Path(maps_dir).expanduser()
        self.session_path = Path(session_path).expanduser()
        self._lock = threading.Lock()
        self._pins, self._loaded_map = _read_pins_file(self.session_path)

    # ---- 保存 ----
    def map_path(self, name: str) -> Path:
        if not isinstance(name, str) or not _SAFE_MAP_NAME.match(name):
            raise ValueError("地図名は英数字・アンダースコア・ハイフンのみ、64文字以内で指定してください")
        return self.maps_dir / f"{name}.pins.json"

    def _persist(self) -> None:
        """セッション(+読み込み中の地図)へ書く。呼び出し側は_lockを保持していること。
        保存失敗でも画面操作は止めない(PinErrorにはしない)。"""
        try:
            _atomic_write(
                self.session_path,
                {"version": 1, "loaded_map": self._loaded_map, "pins": self._pins},
            )
        except Exception:  # noqa: BLE001
            pass
        if self._loaded_map:
            try:
                self._write_map_file(self._loaded_map)
            except Exception:  # noqa: BLE001
                pass

    def _write_map_file(self, name: str) -> int:
        pins = [dict(p, map_name=name) for p in self._pins]
        _atomic_write(self.map_path(name), {"version": 1, "map_name": name,
                                           "saved_at": time.time(), "pins": pins})
        return len(pins)

    def save_for_map(self, name: str) -> int:
        """地図の保存時: 現在のピンを<name>.pins.jsonへ。書いた件数を返す。
        ピンが0件でも(古いファイルを残さないよう)空で書く。"""
        with self._lock:
            return self._write_map_file(name)

    def on_map_loaded(self, name: str) -> int:
        """地図の読み込み時: <name>.pins.jsonからピンを復元する(無ければ空)。件数を返す。"""
        pins, _ = _read_pins_file(self.map_path(name))
        with self._lock:
            self._pins = pins
            self._loaded_map = name
            try:
                _atomic_write(self.session_path, {"version": 1, "loaded_map": name, "pins": pins})
            except Exception:  # noqa: BLE001
                pass
            return len(pins)

    def clear_session(self) -> None:
        """新規マッピング/リセット: 作業中のピンを空にする(保存済みの地図のピンは消さない)。"""
        with self._lock:
            self._pins = []
            self._loaded_map = None
            try:
                _atomic_write(self.session_path, {"version": 1, "loaded_map": None, "pins": []})
            except Exception:  # noqa: BLE001
                pass

    # ---- 参照 ----
    def loaded_map(self) -> Optional[str]:
        with self._lock:
            return self._loaded_map

    def list(self) -> List[Dict[str, Any]]:
        with self._lock:
            return [dict(p) for p in self._pins]

    def get(self, pin_id: Any) -> Optional[Dict[str, Any]]:
        with self._lock:
            for p in self._pins:
                if p["id"] == pin_id:
                    return dict(p)
        return None

    # ---- 変更 ----
    def add(self, name: Any, color: Any, x: float, y: float, yaw_deg: float) -> Dict[str, Any]:
        nm = clean_name(name)
        col = clean_color(color)
        with self._lock:
            if len(self._pins) >= PIN_MAX_COUNT:
                raise PinError(f"ピンは最大{PIN_MAX_COUNT}個までです。不要なピンを削除してください", "limit")
            n = len(self._pins) + 1
            existing = {p["name"] for p in self._pins}
            if not nm:
                nm = f"ピン{n}"
                while nm in existing:
                    n += 1
                    nm = f"ピン{n}"
            pin = {
                "id": uuid.uuid4().hex[:8],
                "name": nm,
                "x": float(x), "y": float(y), "yaw_deg": normalize_deg(float(yaw_deg)),
                "color": col or PIN_DEFAULT_COLORS[len(self._pins) % len(PIN_DEFAULT_COLORS)],
                "created_at": time.time(),
                "map_name": self._loaded_map or "",
            }
            self._pins.append(pin)
            self._persist()
            return dict(pin)

    def update(self, pin_id: Any, fields: Dict[str, Any]) -> Dict[str, Any]:
        """name/color/x/y/yaw_degのうち指定されたものだけを更新する(全部検証してから反映)。"""
        changes: Dict[str, Any] = {}
        if "name" in fields:
            nm = clean_name(fields["name"])
            if not nm:
                raise PinError("名前を入力してください")
            changes["name"] = nm
        if "color" in fields:
            col = clean_color(fields["color"])
            if col is None:
                raise PinError("色は #rrggbb 形式で指定してください")
            changes["color"] = col
        if "x" in fields or "y" in fields or "yaw_deg" in fields:
            for k in ("x", "y"):
                if k not in fields:
                    raise PinError("位置を更新するには x, y を両方指定してください")
            x, y, yaw = parse_pose(fields)
            changes.update({"x": x, "y": y, "yaw_deg": yaw})
        if not changes:
            raise PinError("更新する項目がありません")
        with self._lock:
            for p in self._pins:
                if p["id"] == pin_id:
                    p.update(changes)
                    self._persist()
                    return dict(p)
        raise PinError("ピンが見つかりません", "not_found")

    def delete(self, pin_id: Any) -> Dict[str, Any]:
        with self._lock:
            for i, p in enumerate(self._pins):
                if p["id"] == pin_id:
                    removed = self._pins.pop(i)
                    self._persist()
                    return dict(removed)
        raise PinError("ピンが見つかりません", "not_found")


_STORES: Dict[Tuple[str, str], PinStore] = {}
_STORES_LOCK = threading.Lock()


def get_pin_store(maps_dir: str, session_path: str = SESSION_PATH_DEFAULT) -> PinStore:
    """同じmaps_dirなら同一インスタンス(保存側・読み込み側・HTTP側で共有する)。"""
    key = (str(Path(maps_dir).expanduser()), str(Path(session_path).expanduser()))
    with _STORES_LOCK:
        st = _STORES.get(key)
        if st is None:
            st = PinStore(maps_dir, session_path)
            _STORES[key] = st
        return st


# ------------------------------------------------------------------ 行動ルールへのピン配信(2026-10-09追加)
PINS_TOPIC = "/nokosahen/pins"


def pins_payload(loaded_map: Optional[str], pins: List[Dict[str, Any]]) -> Dict[str, Any]:
    """`/nokosahen/pins`(TRANSIENT_LOCAL, JSON)の中身。behavior_director(行動ルールgoto_pin)と
    ルール編集ページが「いま読み込み中の地図のピン」を知るための配信。
    stampは配信時刻(参考値。director側は受信時刻で鮮度を見る)。"""
    return {
        "version": 1,
        "stamp": time.time(),
        "loaded_map": loaded_map if isinstance(loaded_map, str) and loaded_map else None,
        "pins": [
            {"id": p["id"], "name": p["name"], "x": p["x"], "y": p["y"],
             "yaw_deg": p["yaw_deg"], "color": p["color"]}
            for p in pins
        ],
    }


def resolve_pin(
    payload: Any, pin_id: Any, pin_map: Optional[str] = None
) -> Tuple[Optional[Dict[str, Any]], str]:
    """ルールが指すピンを、配信されたピン一覧から引く。(ピン, 理由コード)を返す。

    ピンはIDで引く(名前は変更・重複があり得る)。見つからない/疑わしいときは必ずNone(=走行しない)。
    理由: "ok" / "no_data"(配信を受けていない) / "no_map"(地図が未読み込み: 作業中のピンは
    地図に紐づかないので使わない) / "map_mismatch"(ルールの想定した地図と違う) /
    "not_found"(別地図で作ったピン・削除済み) / "invalid"(座標が壊れている)。
    """
    if not isinstance(payload, dict) or not isinstance(payload.get("pins"), list):
        return None, "no_data"
    loaded = payload.get("loaded_map")
    if not isinstance(loaded, str) or not loaded:
        return None, "no_map"
    if pin_map and pin_map != loaded:
        return None, "map_mismatch"
    if not isinstance(pin_id, str) or not pin_id:
        return None, "not_found"
    for p in payload["pins"]:
        if isinstance(p, dict) and p.get("id") == pin_id:
            try:
                x, y, yaw = parse_pose(p)
            except PinError:
                return None, "invalid"
            return {"id": pin_id, "name": str(p.get("name", "")), "x": x, "y": y,
                    "yaw_deg": yaw, "color": p.get("color")}, "ok"
    return None, "not_found"


PIN_REASON_TEXT = {
    "ok": "",
    "no_data": "ピン情報を受信していません(ダッシュボード未起動/通信断)",
    "no_map": "地図が読み込まれていません",
    "map_mismatch": "このピンは別の地図用です",
    "not_found": "ピンが現在の地図にありません(削除、または別の地図)",
    "invalid": "ピンの位置が不正です",
}


# ------------------------------------------------------------------ 目的地のスナップ
GOAL_ROBOT_RADIUS_M = 0.28   # nav2_params_*.yamlのrobot_radius
GOAL_CLEARANCE_MARGIN_M = 0.04
GOAL_SNAP_MAX_SHIFT_M = 0.5
OCCUPIED_MIN = 50   # この値以上は占有(Nav2のoccupied_threshold付近)


def _cell_of(x: float, y: float, res: float, ox: float, oy: float) -> Tuple[int, int]:
    return int(math.floor((x - ox) / res)), int(math.floor((y - oy) / res))


def snap_goal_to_free(
    width: int, height: int, resolution: float, origin_x: float, origin_y: float,
    data: Any, x: float, y: float,
    robot_radius: float = GOAL_ROBOT_RADIUS_M,
    margin: float = GOAL_CLEARANCE_MARGIN_M,
    max_shift: float = GOAL_SNAP_MAX_SHIFT_M,
) -> Dict[str, Any]:
    """目的地(x, y)が「ロボットが立てる空きセル」か判定し、そうでなければ最も近い空きセルへ寄せる。

    空きセル = 値が0(既知の自由)で、周囲 robot_radius+margin 以内に占有セル(>=50)が無い。
    dataはOccupancyGridのdata(行0=origin_yの行、1セル=1要素。-1=未知、0=自由、100=占有)。

    戻り値: {"status", "x", "y", "shift_m", "reason"}
      status: "ok"(そのまま) / "snapped"(寄せた) / "no_free"(占有セル上で、max_shift内に空きが無い)
              / "unchanged"(未知セルまたは壁際で、寄せ先が無い。そのまま送る) / "no_map"(地図が無い/不正)
    "no_free"だけは呼び出し側で拒否する(占有セルへは行けない)。
    """
    out = {"status": "no_map", "x": x, "y": y, "shift_m": 0.0, "reason": ""}
    try:
        if not (width > 0 and height > 0 and resolution > 0 and len(data) >= width * height):
            return out
        res = float(resolution)
        c0, r0 = _cell_of(x, y, res, origin_x, origin_y)
    except (TypeError, ValueError, OverflowError):
        return out
    clear_cells = (robot_radius + margin) / res
    reach = int(math.ceil(max_shift / res))
    win = int(math.ceil(clear_cells)) + reach + 1

    def value(c: int, r: int) -> int:
        if 0 <= c < width and 0 <= r < height:
            return int(data[r * width + c])
        return -1  # 地図外は未知

    # 探索窓内の占有セル(クリアランス判定用)
    occ: List[Tuple[int, int]] = []
    for r in range(r0 - win, r0 + win + 1):
        if not 0 <= r < height:
            continue
        for c in range(c0 - win, c0 + win + 1):
            if 0 <= c < width and data[r * width + c] >= OCCUPIED_MIN:
                occ.append((c, r))

    def cell_ok(c: int, r: int) -> bool:
        if value(c, r) != 0:
            return False
        for oc, orr in occ:
            if (oc - c) ** 2 + (orr - r) ** 2 <= clear_cells * clear_cells:
                return False
        return True

    v0 = value(c0, r0)
    if cell_ok(c0, r0):
        out.update(status="ok")
        return out
    # 候補を距離順(クリック点から)に調べる。セル中心へ寄せるので、寄せた先は必ずセル中心。
    best = None
    best_d2 = None
    for r in range(r0 - reach, r0 + reach + 1):
        for c in range(c0 - reach, c0 + reach + 1):
            cx = origin_x + (c + 0.5) * res
            cy = origin_y + (r + 0.5) * res
            d2 = (cx - x) ** 2 + (cy - y) ** 2
            if d2 > max_shift * max_shift:
                continue
            if best_d2 is not None and d2 >= best_d2:
                continue
            if cell_ok(c, r):
                best, best_d2 = (cx, cy), d2
    if best is not None:
        out.update(status="snapped", x=best[0], y=best[1], shift_m=math.sqrt(best_d2),
                   reason="occupied" if v0 >= OCCUPIED_MIN else ("unknown" if v0 < 0 else "near_obstacle"))
        return out
    if v0 < 0:
        out.update(status="unchanged", reason="unknown")
    elif v0 >= OCCUPIED_MIN:
        out.update(status="no_free", reason="occupied")
    else:
        # 自由セルだが壁に近い。近くに寄せ先も無い: Nav2側のtolerance任せで送る
        out.update(status="unchanged", reason="near_obstacle")
    return out
