"""地図の表示回転と「回転した走行可能範囲(行動範囲)」の純粋ロジック。

【2026-10-06追加】SLAMの地図はx軸=スタート時の機体の向きになるため、部屋の壁が
斜めに見える/斜めの部屋だと軸平行の矩形では範囲を囲めない、という問題への対応。

設計方針(重要):
- 世界座標系(Nav2/SLAM/保存地図)は一切回さない。回すのは「ダッシュボードの
  表示ビュー」と「走行可能範囲の矩形」だけ。
- 走行可能範囲は {center_x, center_y, width, height, yaw}(世界座標、yaw[rad]は
  矩形のローカルx軸の向き)。旧形式 min_x/max_x/min_y/max_y は yaw=0 の矩形として
  読み替える。旧形式の消費者(behavior_director等)が壊れないよう、状態には
  回転矩形の軸平行外接矩形を min_x/max_x/min_y/max_y として同時に載せる。
- rclpy等に依存しない(単体テスト可能、numpy不要)。frontier_explorer/dashboard_common
  から共通利用する。
"""

from __future__ import annotations

import json
import math
import os
import re
import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

SAFE_MAP_NAME = re.compile(r"^[A-Za-z0-9_\-]{1,64}$")

# 異常値(桁間違い等)を弾くための上限。屋内ロボットの地図としては十分に大きい。
MAX_BOUNDARY_SIZE_M = 1000.0
MAX_COORD_M = 1.0e5
_EPS = 1e-9


class BoundaryError(ValueError):
    """kind="malformed": 欠落/非数/非有限(HTTP 400)、kind="geometry": 幅高さ不正等(HTTP 409)。"""

    def __init__(self, message: str, kind: str = "malformed") -> None:
        super().__init__(message)
        self.kind = kind


# ---------------------------------------------------------------- 角度ユーティリティ
def normalize_deg(a: float) -> float:
    """角度[deg]を(-180, 180]へ。"""
    a = math.fmod(float(a), 360.0)
    if a <= -180.0:
        a += 360.0
    elif a > 180.0:
        a -= 360.0
    return a


def normalize_rad(a: float) -> float:
    """角度[rad]を(-pi, pi]へ。"""
    a = math.fmod(float(a), 2.0 * math.pi)
    if a <= -math.pi:
        a += 2.0 * math.pi
    elif a > math.pi:
        a -= 2.0 * math.pi
    return a


def _num(v: Any, name: str) -> float:
    if isinstance(v, bool) or v is None:
        raise BoundaryError(f"{name} は数値で指定してください")
    try:
        f = float(v)
    except (TypeError, ValueError):
        raise BoundaryError(f"{name} は数値で指定してください") from None
    if not math.isfinite(f):
        raise BoundaryError(f"{name} が有限の数値ではありません(NaN/Infinity)")
    if abs(f) > MAX_COORD_M:
        raise BoundaryError(f"{name} の値が大きすぎます")
    return f


# ---------------------------------------------------------------- 回転矩形
def make_rect(center_x: Any, center_y: Any, width: Any, height: Any, yaw: Any = 0.0) -> Dict[str, float]:
    cx = _num(center_x, "center_x")
    cy = _num(center_y, "center_y")
    w = _num(width, "width")
    h = _num(height, "height")
    yaw_f = _num(yaw, "yaw")
    if w <= 0.0 or h <= 0.0:
        raise BoundaryError("範囲が不正です(幅・高さは正の値が必要です)", kind="geometry")
    if w > MAX_BOUNDARY_SIZE_M or h > MAX_BOUNDARY_SIZE_M:
        raise BoundaryError("範囲が大きすぎます", kind="geometry")
    return {
        "center_x": cx,
        "center_y": cy,
        "width": w,
        "height": h,
        "yaw": normalize_rad(yaw_f),
    }


def rect_from_minmax(min_x: Any, max_x: Any, min_y: Any, max_y: Any) -> Dict[str, float]:
    """旧形式(軸平行)をyaw=0の矩形へ変換する。"""
    x0 = _num(min_x, "min_x")
    x1 = _num(max_x, "max_x")
    y0 = _num(min_y, "min_y")
    y1 = _num(max_y, "max_y")
    if x0 >= x1 or y0 >= y1:
        raise BoundaryError("範囲が不正です(対角の2点が同じ/逆になっていませんか？)", kind="geometry")
    return make_rect((x0 + x1) / 2.0, (y0 + y1) / 2.0, x1 - x0, y1 - y0, 0.0)


def rect_corners(rect: Dict[str, float]) -> List[Tuple[float, float]]:
    """世界座標の4隅(ローカル(-w/2,-h/2)から反時計回り)。"""
    c, s = math.cos(rect["yaw"]), math.sin(rect["yaw"])
    hw, hh = rect["width"] / 2.0, rect["height"] / 2.0
    out = []
    for lx, ly in ((-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)):
        out.append((rect["center_x"] + lx * c - ly * s, rect["center_y"] + lx * s + ly * c))
    return out


def rect_bbox(rect: Dict[str, float]) -> Tuple[float, float, float, float]:
    """軸平行外接矩形 (min_x, max_x, min_y, max_y)。旧消費者向け(真の範囲より緩い)。"""
    pts = rect_corners(rect)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (min(xs), max(xs), min(ys), max(ys))


def point_in_rect(x: Any, y: Any, rect: Dict[str, float], margin: float = 0.0) -> bool:
    """点が回転矩形の内側(境界上を含む)か。非有限の点は常に外側。

    margin>0 で矩形を全周に広げる、margin<0 で狭める。
    """
    try:
        xf, yf = float(x), float(y)
    except (TypeError, ValueError):
        return False
    if not (math.isfinite(xf) and math.isfinite(yf)):
        return False
    dx = xf - rect["center_x"]
    dy = yf - rect["center_y"]
    c, s = math.cos(rect["yaw"]), math.sin(rect["yaw"])
    # 世界→矩形ローカル(逆回転)
    lx = dx * c + dy * s
    ly = -dx * s + dy * c
    tol = _EPS * max(1.0, rect["width"], rect["height"])
    return (
        abs(lx) <= rect["width"] / 2.0 + margin + tol
        and abs(ly) <= rect["height"] / 2.0 + margin + tol
    )


# ---------------------------------------------------------------- 状態 dict(API/トピック共通)
def boundary_state(rect: Dict[str, float], enabled: bool = True) -> Dict[str, Any]:
    """新形式+旧形式(外接矩形)を両方含む状態。/nokosahen/travel_boundaryにもそのまま流す。"""
    min_x, max_x, min_y, max_y = rect_bbox(rect)
    return {
        "enabled": bool(enabled),
        "center_x": rect["center_x"],
        "center_y": rect["center_y"],
        "width": rect["width"],
        "height": rect["height"],
        "yaw": rect["yaw"],
        "yaw_deg": math.degrees(rect["yaw"]),
        "min_x": min_x,
        "max_x": max_x,
        "min_y": min_y,
        "max_y": max_y,
    }


def disabled_state() -> Dict[str, Any]:
    return {
        "enabled": False,
        "center_x": 0.0,
        "center_y": 0.0,
        "width": 0.0,
        "height": 0.0,
        "yaw": 0.0,
        "yaw_deg": 0.0,
        "min_x": 0.0,
        "max_x": 0.0,
        "min_y": 0.0,
        "max_y": 0.0,
    }


def parse_boundary_request(data: Any) -> Dict[str, float]:
    """POST /api/boundary/set のボディ(新旧どちらでも可)を検証して矩形にする。

    新形式: center_x, center_y, width, height と yaw[rad] または yaw_deg[deg](省略時0)。
    旧形式: min_x, max_x, min_y, max_y(yaw=0)。
    BoundaryError(kind=malformed→400 / geometry→409)を送出する。
    """
    if not isinstance(data, dict):
        raise BoundaryError("JSONオブジェクトで指定してください")
    if "center_x" in data or "width" in data:
        for k in ("center_x", "center_y", "width", "height"):
            if k not in data:
                raise BoundaryError(f"{k} が必要です")
        if "yaw" in data and "yaw_deg" in data:
            raise BoundaryError("yaw と yaw_deg は同時に指定できません")
        if "yaw_deg" in data:
            yaw = math.radians(_num(data["yaw_deg"], "yaw_deg"))
        else:
            yaw = data.get("yaw", 0.0)
        return make_rect(data["center_x"], data["center_y"], data["width"], data["height"], yaw)
    for k in ("min_x", "max_x", "min_y", "max_y"):
        if k not in data:
            raise BoundaryError(
                "center_x/center_y/width/height(+yaw) または min_x/max_x/min_y/max_y が必要です"
            )
    return rect_from_minmax(data["min_x"], data["max_x"], data["min_y"], data["max_y"])


def rect_from_state(state: Any) -> Optional[Dict[str, float]]:
    """保存ファイル/トピックのdictから矩形を取り出す(新形式優先、無ければ旧形式)。
    不正ならNone(呼び出し側は「無視して直前の状態を保つ」)。enabledは見ない。"""
    if not isinstance(state, dict):
        return None
    try:
        if all(k in state for k in ("center_x", "center_y", "width", "height")):
            return make_rect(
                state["center_x"], state["center_y"], state["width"], state["height"],
                state.get("yaw", 0.0),
            )
        if all(k in state for k in ("min_x", "max_x", "min_y", "max_y")):
            return rect_from_minmax(state["min_x"], state["max_x"], state["min_y"], state["max_y"])
    except BoundaryError:
        return None
    return None


def state_allows_point(state: Any, x: Any, y: Any) -> bool:
    """状態dictに対する「この点は走行可能範囲内か」。
    制限無し(enabled偽/未設定)は常にTrue(従来どおり)。enabledなのに矩形が
    読めない状態は安全側でFalse(呼び出し側は通常、不正状態をそもそも保持しない)。"""
    if not isinstance(state, dict) or not state.get("enabled"):
        return True
    rect = rect_from_state(state)
    if rect is None:
        return False
    return point_in_rect(x, y, rect)


def clamp_point_to_rect(x: Any, y: Any, rect: Dict[str, float], margin: float = 0.0) -> Tuple[float, float]:
    """点を回転矩形の内側へ射影する(矩形のローカル座標で各軸をクランプ→世界へ戻す)。

    すでに内側(境界上を含む)なら入力をそのまま返す(浮動小数の丸めで動かさない)。
    margin>0 は矩形を全周で内側へ狭める小さな余白(半幅/半高の上限を超えない範囲)。
    非有限の点は矩形の中心を返す(壊れた座標を「範囲内」として通さないため)。
    """
    try:
        xf, yf = float(x), float(y)
    except (TypeError, ValueError):
        return (rect["center_x"], rect["center_y"])
    if not (math.isfinite(xf) and math.isfinite(yf)):
        return (rect["center_x"], rect["center_y"])
    if margin == 0.0 and point_in_rect(xf, yf, rect):
        return (xf, yf)
    hw, hh = rect["width"] / 2.0, rect["height"] / 2.0
    m = max(0.0, float(margin))
    ex = max(0.0, hw - min(m, hw))
    ey = max(0.0, hh - min(m, hh))
    dx = xf - rect["center_x"]
    dy = yf - rect["center_y"]
    c, s = math.cos(rect["yaw"]), math.sin(rect["yaw"])
    lx = dx * c + dy * s
    ly = -dx * s + dy * c
    if abs(lx) <= ex and abs(ly) <= ey:
        return (xf, yf)
    lx = max(-ex, min(ex, lx))
    ly = max(-ey, min(ey, ly))
    return (
        rect["center_x"] + lx * c - ly * s,
        rect["center_y"] + lx * s + ly * c,
    )


def boundary_message_update(data: Any) -> Optional[Tuple[Dict[str, Any], Optional[Dict[str, float]]]]:
    """/nokosahen/travel_boundary のメッセージ(JSONをloadsしたdict)を受け入れてよいか判定する。

    戻り値: 受理なら (状態dict, 回転矩形 または None(enabled=false))、
    無視すべき不正メッセージ(dictでない / enabledなのに矩形が読めない・非有限・幅高さ不正)なら
    None(呼び出し側は直前の有効な状態を保つ)。新形式(center/width/height/yaw)を優先し、
    無ければ旧形式(min/max)を読む。
    """
    if not isinstance(data, dict) or "enabled" not in data:
        # 空dict({}: 壊れたJSONのフォールバック等)や"enabled"の無いものは不正。受理すると
        # 「範囲解除」と区別が付かず、壊れたメッセージで範囲が外れてしまう。
        return None
    if not data.get("enabled"):
        return (data, None)
    rect = rect_from_state(data)
    if rect is None:
        return None
    return (data, rect)


# ---------------------------------------------------------------- スケジュールによる範囲の一時的な差し替え/解除
# 【2026-10-23】docs/schedule.md。behavior_director(ビヘイビアツリーのset_boundary)が /nokosahen/boundary_override へ
# 1Hzで配信し、ダッシュボードのBoundaryControllerだけが、ユーザーが設定した範囲(元の範囲)とこの上書きを合成して
# /nokosahen/travel_boundary へ出す。購読側(director/router/explorer)は従来どおり travel_boundary だけを見ればよい。
# 形式は既存のtravel_boundary(enabled+矩形)に "active" を足しただけ:
#   {"active": false}                                  上書きなし(元の範囲)
#   {"active": true, "enabled": false, ...}            解除(範囲の制限なし)
#   {"active": true, "enabled": true, center_x.. }     この矩形に差し替え
# 安全側の規則: 1) 受け取ってからTTL(秒)を過ぎたら無効(directorが止まれば元の範囲へ戻る)
#   2) 不正なメッセージは無視(=元の範囲)  3) 上書きは保存しない(元の範囲のファイルは書き換えない)
BOUNDARY_OVERRIDE_TOPIC = "/nokosahen/boundary_override"
BOUNDARY_OVERRIDE_TTL_SEC = 5.0


def boundary_override_message(
    request: Optional[Dict[str, Any]], source: str = "schedule", stamp: Optional[float] = None
) -> Dict[str, Any]:
    """木のset_boundaryの出力({"mode":"release"} / {"mode":"set","x_min",..} / None)から配信用のdictを作る。
    不正な範囲(逆転・非有限)は「上書きなし」にする(安全側=元の範囲のまま)。"""
    out: Dict[str, Any] = {"active": False, "source": source}
    if stamp is not None:
        out["stamp"] = float(stamp)
    if not isinstance(request, dict):
        return out
    mode = request.get("mode")
    if mode == "release":
        out.update(disabled_state())
        out.update({"active": True, "enabled": False})
        return out
    if mode == "set":
        try:
            rect = rect_from_minmax(request.get("x_min"), request.get("x_max"), request.get("y_min"), request.get("y_max"))
        except BoundaryError:
            return out
        out.update(boundary_state(rect, True))
        out["active"] = True
        return out
    return out


def parse_boundary_override(data: Any) -> Optional[Dict[str, Any]]:
    """受け取った上書きメッセージが有効なら「上書き後の範囲の状態dict」(travel_boundaryと同じ形)を返す。
    上書きなし(active偽)・不正はNone(=元の範囲のまま)。"""
    if not isinstance(data, dict) or data.get("active") is not True:
        return None
    upd = boundary_message_update({k: v for k, v in data.items() if k not in ("active", "source", "stamp")})
    if upd is None:
        return None
    state, _rect = upd
    out = dict(state)
    out["override_active"] = True
    if isinstance(data.get("source"), str):
        out["override_source"] = data["source"][:40]
    return out


def load_boundary_state(data: Any) -> Dict[str, Any]:
    """永続化ファイルの内容から状態を復元する。旧形式ファイルも読める。
    壊れている/enabledなのに矩形が不正なら「制限なし」(従来の既定動作)。"""
    if not isinstance(data, dict) or not data.get("enabled"):
        return disabled_state()
    rect = rect_from_state(data)
    if rect is None:
        return disabled_state()
    return boundary_state(rect, True)


# ---------------------------------------------------------------- 壁方向の自動推定
def view_angle_from_wall_deg(wall_deg: float) -> float:
    """壁の向き(世界座標、x軸から反時計回り[deg])を水平にする表示回転角[deg]。"""
    return normalize_deg(-wall_deg)


def view_angle_from_two_points(x1: float, y1: float, x2: float, y2: float) -> float:
    """壁沿いの2点(世界座標)から、その壁を水平にする表示回転角[deg](-90,90]。"""
    if not all(math.isfinite(v) for v in (x1, y1, x2, y2)):
        raise ValueError("non-finite point")
    if math.hypot(x2 - x1, y2 - y1) < 1e-6:
        raise ValueError("2点が同じ位置です")
    wall = math.degrees(math.atan2(y2 - y1, x2 - x1))
    a = normalize_deg(-wall)
    # 直線の向きは180°対称なので(-90,90]へ畳む
    if a > 90.0:
        a -= 180.0
    elif a <= -90.0:
        a += 180.0
    return a


def estimate_wall_angle_deg(
    width: int,
    height: int,
    cells: Sequence[int],
    occupied_threshold: int = 50,
    min_samples: int = 20,
    max_samples: int = 4000,
) -> Optional[Dict[str, float]]:
    """占有セルの縁の向きのヒストグラム(90°周期)から、支配的な壁方向[deg]を推定する。

    cells: 行優先の占有値(-1未知, 0..100)。index=row*width+col、rowは世界yの増加方向
    (OccupancyGridと同じ)。返す angle_deg は壁の向き(世界x軸から反時計回り)を
    (-45, 45]に畳んだ値。壁を水平にしたい表示回転角は view_angle_from_wall_deg()。
    推定不能(占有セル不足)はNone。純Python(numpy不要)。

    手順: 占有セルごとの局所PCA(近傍の占有セルの主軸=壁の向き、直線らしさで重み付け)→ θ mod 90° の
    0.5°刻みヒストグラム→ ±1.5°平滑化でピーク→ ピーク±6°内のサンプルで
    4θの重み付き円平均(=PCA的な精密化)。
    """
    if width < 3 or height < 3 or len(cells) < width * height:
        return None
    mask = bytearray(width * height)
    occ_idx = []
    for i in range(width * height):
        if cells[i] >= occupied_threshold:
            mask[i] = 1
            occ_idx.append(i)
    if len(occ_idx) < min_samples:
        return None
    # 間引き(大きな地図でも計算量を一定に)。等間隔サンプリングで偏りを避ける。
    stride = max(1, len(occ_idx) // max_samples)
    samples: List[Tuple[float, float]] = []  # (θ mod 90 [deg], 重み)
    w = width
    rad = 8
    for i in occ_idx[::stride]:
        r, c = divmod(i, w)
        if r < rad or c < rad or r >= height - rad or c >= w - rad:
            continue
        # 近傍(半径rad)の占有セルの2次モーメント(局所PCA)から線の向きを求める。
        # 1セル単位のSobel勾配は斜め線で atan(1/2) 等に量子化されて偏るため。
        n = 0
        sx = sy = sxx = syy = sxy = 0.0
        for rr in range(r - rad, r + rad + 1):
            base = rr * w
            for cc in range(c - rad, c + rad + 1):
                if mask[base + cc]:
                    n += 1
                    dx = cc - c
                    dy = rr - r
                    sx += dx
                    sy += dy
                    sxx += dx * dx
                    syy += dy * dy
                    sxy += dx * dy
        if n < 6:
            continue
        mx, my = sx / n, sy / n
        vxx = sxx / n - mx * mx
        vyy = syy / n - my * my
        vxy = sxy / n - mx * my
        tr = vxx + vyy
        if tr <= 1e-9:
            continue
        diff = math.hypot(vxx - vyy, 2.0 * vxy)
        lin = diff / tr  # 0(丸い塊/隅)〜1(直線)
        if lin < 0.6:
            continue
        theta = math.degrees(0.5 * math.atan2(2.0 * vxy, vxx - vyy)) % 90.0
        samples.append((theta, lin * lin))
    if len(samples) < min_samples:
        return None

    nbins = 180
    hist = [0.0] * nbins
    total = 0.0
    for th, wt in samples:
        hist[int(th / 0.5) % nbins] += wt
        total += wt
    if total <= 0.0:
        return None
    smooth = [0.0] * nbins
    for i in range(nbins):
        smooth[i] = sum(hist[(i + j) % nbins] for j in range(-3, 4))
    peak = max(range(nbins), key=lambda i: smooth[i])
    peak_deg = (peak + 0.5) * 0.5

    sx = sy = sw = 0.0
    for th, wt in samples:
        d = (th - peak_deg + 45.0) % 90.0 - 45.0  # ピークとの周期差
        if abs(d) <= 6.0:
            sx += wt * math.cos(math.radians(4.0 * th))
            sy += wt * math.sin(math.radians(4.0 * th))
            sw += wt
    if sw <= 0.0:
        return None
    refined = (math.degrees(math.atan2(sy, sx)) / 4.0) % 90.0
    near = 0.0
    for th, wt in samples:
        d = (th - refined + 45.0) % 90.0 - 45.0
        if abs(d) <= 3.0:
            near += wt
    angle = refined if refined <= 45.0 else refined - 90.0
    return {
        "angle_deg": angle,
        "confidence": near / total,
        "samples": float(len(samples)),
    }


# ---------------------------------------------------------------- 表示角度の永続化
class MapViewStore:
    """現在セッションの表示回転角と、地図ごとの<name>.view.jsonの読み書き。

    - セッション値: ~/.nokosahen_map_view.json {"angle_deg", "loaded_map"}
    - 地図ごと: maps_dir/<name>.view.json {"version":1, "angle_deg", "saved_at"}
    角度は常に(-180,180]の有限値に正規化して保存する。
    """

    def __init__(self, maps_dir: str, session_path: str = "~/.nokosahen_map_view.json") -> None:
        self.maps_dir = Path(maps_dir).expanduser()
        self.session_path = Path(session_path).expanduser()
        self._lock = threading.Lock()
        self._angle = 0.0
        self._loaded_map: Optional[str] = None
        try:
            data = json.loads(self.session_path.read_text(encoding="utf-8"))
            if isinstance(data, dict):
                a = data.get("angle_deg")
                if isinstance(a, (int, float)) and not isinstance(a, bool) and math.isfinite(a):
                    self._angle = normalize_deg(a)
                lm = data.get("loaded_map")
                if isinstance(lm, str) and SAFE_MAP_NAME.match(lm):
                    self._loaded_map = lm
        except Exception:  # noqa: BLE001 - 未作成/壊れている場合は0°
            pass

    @staticmethod
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

    def _persist_session(self) -> None:
        try:
            self._atomic_write(
                self.session_path, {"angle_deg": self._angle, "loaded_map": self._loaded_map}
            )
        except Exception:  # noqa: BLE001 - 保存失敗でも表示は動かす
            pass

    def status(self) -> Dict[str, Any]:
        with self._lock:
            name = self._loaded_map
            angle = self._angle
        saved = self.read_map_angle(name) if name else None
        return {"angle_deg": angle, "loaded_map": name, "saved_angle_deg": saved}

    def set_session_angle(self, angle_deg: Any) -> float:
        if isinstance(angle_deg, bool) or angle_deg is None:
            raise ValueError("angle_deg は数値で指定してください")
        a = float(angle_deg)
        if not math.isfinite(a):
            raise ValueError("angle_deg が有限の数値ではありません")
        a = normalize_deg(a)
        with self._lock:
            self._angle = a
            self._persist_session()
        return a

    def read_map_angle(self, name: Optional[str]) -> Optional[float]:
        if not name or not SAFE_MAP_NAME.match(name):
            return None
        try:
            data = json.loads((self.maps_dir / f"{name}.view.json").read_text(encoding="utf-8"))
            a = data.get("angle_deg") if isinstance(data, dict) else None
            if isinstance(a, (int, float)) and not isinstance(a, bool) and math.isfinite(a):
                return normalize_deg(a)
        except Exception:  # noqa: BLE001
            pass
        return None

    def save_for_map(self, name: str, angle_deg: Optional[float] = None) -> float:
        if not name or not SAFE_MAP_NAME.match(name):
            raise ValueError("地図名は英数字・アンダースコア・ハイフンのみ、64文字以内で指定してください")
        with self._lock:
            a = self._angle if angle_deg is None else float(angle_deg)
        if not math.isfinite(a):
            raise ValueError("angle_deg が有限の数値ではありません")
        a = normalize_deg(a)
        self._atomic_write(
            self.maps_dir / f"{name}.view.json",
            {"version": 1, "angle_deg": a, "saved_at": time.time()},
        )
        return a

    def on_map_loaded(self, name: str) -> Optional[float]:
        """地図読み込み時: 保存された向きがあれば復元してセッション値にする。"""
        a = self.read_map_angle(name)
        with self._lock:
            self._loaded_map = name if SAFE_MAP_NAME.match(name or "") else None
            if a is not None:
                self._angle = a
            self._persist_session()
        return a

    def clear_loaded_map(self) -> None:
        with self._lock:
            self._loaded_map = None
            self._persist_session()

    def loaded_map(self) -> Optional[str]:
        with self._lock:
            return self._loaded_map


_STORES: Dict[Tuple[str, str], MapViewStore] = {}
_STORES_LOCK = threading.Lock()


def get_view_store(maps_dir: str, session_path: str = "~/.nokosahen_map_view.json") -> MapViewStore:
    """同じmaps_dirなら同一インスタンス(保存側・読み込み側・HTTP側で共有する)。"""
    key = (str(Path(maps_dir).expanduser()), str(Path(session_path).expanduser()))
    with _STORES_LOCK:
        st = _STORES.get(key)
        if st is None:
            st = MapViewStore(maps_dir, session_path)
            _STORES[key] = st
        return st
