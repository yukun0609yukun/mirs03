#!/usr/bin/env python3
"""タブレットOCR用カメラの画質まわり(2026-10-10追加)。tablet_ocr_reader.py から使う。

OCRカメラ(BUFFALO BSWHD06M、UVC固定焦点)は「ピントが合わずぼやける」ことが最大の問題で、
固定焦点はソフトでは本当のピントは直せない。そこで次の3本立てで認識しやすい画像にする。
  (a) ピント(鮮鋭度)の定量とピント合わせ支援(FocusMonitor / edge_sharpness)
  (b) UVC設定(シャープネス/コントラスト/ガンマ/明るさ/逆光補正/WB)の自動チューニング(CameraTuner)
  (c) ソフトの画質改善(複数フレームのベスト選択・平均・自動レベル・アンシャープ・CLAHE)

このモジュールはOpenCV(cv2)とnumpyだけに依存し、カメラ(cap)・フレーム取得はコールバックで受ける
ので、実機が無くてもスタブでテストできる(tests/test_camera_autotune.py, test_image_enhance.py)。
cv2/np は呼び出し側から引数で渡す(tablet_ocr_reader.py の流儀に合わせる)。
"""
from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque, Dict, List, Optional, Sequence, Tuple

logger = logging.getLogger("tablet_ocr_reader")

# =============================================================================
# (a) 鮮鋭度(ピント)の定量
# =============================================================================
# 鮮鋭度の指標は「エッジの鋭さ = 強いエッジの勾配 ÷ 画像のダイナミックレンジ」。
# ガウスぼかしσの段差エッジの最大勾配は 約 C/(2.5σ) なので、この比は 1/σ に比例し、
# 照明の明るさ・コントラスト・画面内容にほぼ依存しない(ラプラシアン分散はノイズで跳ね上がり、
# コントラストにも比例するため絶対値の判定に向かない)。解像度の違いは幅1280px基準へ正規化する。
FOCUS_SCALE = 400.0  # 表示用スコア = 比 x 400 (合成画像: 鮮明≈108, σ1.5≈78, σ2.5≈56, σ4≈38)
DEFAULT_FOCUS_THRESHOLDS = {"good": 60.0, "fair": 35.0}
MIN_CONTRAST_FOR_SCORE = 25.0  # 1〜99%点の差がこれ未満なら「対象が写っていない/真っ平ら」でスコア無し
_FOCUS_GRADE_LABELS = {"good": "良好", "fair": "やや悪い", "bad": "悪い"}


def _to_gray(cv2, img):
    if img.ndim == 2:
        return img
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def ratio_box_to_pixels(shape: Sequence[int], box: Optional[Tuple[float, float, float, float]]):
    """比率(x1,y1,x2,y2)をピクセル枠へ。boxが無ければ画面中央(幅高さの中央50%)。"""
    h, w = int(shape[0]), int(shape[1])
    if box is None:
        box = (0.25, 0.25, 0.75, 0.75)
    x1 = max(0, min(w - 1, int(box[0] * w)))
    y1 = max(0, min(h - 1, int(box[1] * h)))
    x2 = max(x1 + 1, min(w, int(box[2] * w)))
    y2 = max(y1 + 1, min(h, int(box[3] * h)))
    return x1, y1, x2, y2


def rois_bounding_box(rois) -> Optional[Tuple[float, float, float, float]]:
    """ROI(x1,y1,x2,y2属性を持つもの)群の外接枠(比率)。空ならNone。"""
    items = list(rois.values()) if hasattr(rois, "values") else list(rois or [])
    if not items:
        return None
    return (
        min(r.x1 for r in items), min(r.y1 for r in items),
        max(r.x2 for r in items), max(r.y2 for r in items),
    )


def edge_sharpness(cv2, np, img, ratio_box=None, smooth: float = 0.7) -> Optional[Dict]:
    """対象領域(ROI外接枠、無ければ画面中央)のエッジ鋭さを返す。
    戻り値: {"ratio","score","contrast","mean","clip_hi","clip_lo","sep"} か、領域が小さすぎるときNone。
    contrast不足(平坦)の場合は score=None(ratioも None)。
    """
    gray = _to_gray(cv2, img)
    h, w = gray.shape[:2]
    x1, y1, x2, y2 = ratio_box_to_pixels(gray.shape, ratio_box)
    region = gray[y1:y2, x1:x2]
    if region.shape[0] < 16 or region.shape[1] < 16:
        return None
    g = region.astype(np.float32)
    if smooth and smooth > 0:
        g = cv2.GaussianBlur(g, (0, 0), float(smooth))
    gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3) * (1.0 / 8.0)
    gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3) * (1.0 / 8.0)
    mag = np.sqrt(gx * gx + gy * gy)
    flat = g.reshape(-1)
    mflat = mag.reshape(-1)
    if flat.size > 300_000:  # 統計だけ間引く(勾配自体はフル解像度で計算済み)
        flat = flat[::2]
        mflat = mflat[::2]
    lo, hi = np.percentile(flat, (1.0, 99.0))
    contrast = float(hi - lo)
    out: Dict = {
        "ratio": None, "score": None, "contrast": contrast, "mean": float(region.mean()),
        "clip_hi": float((region >= 250).mean()), "clip_lo": float((region <= 5).mean()),
        "sep": None,
    }
    out["sep"] = _otsu_separation(cv2, np, region)
    if contrast < MIN_CONTRAST_FOR_SCORE:
        return out
    g995 = float(np.percentile(mflat, 99.5))
    ratio = g995 / contrast * (w / 1280.0)
    out["ratio"] = ratio
    out["score"] = ratio * FOCUS_SCALE
    return out


def _otsu_separation(cv2, np, region) -> float:
    """大津の二値化で前景/背景に分け、平均の差 ÷ (両クラスの標準偏差の合成+床4)。
    数字(前景)が背景から分離している度合い。前景が極端に少ない(<0.3%)/無い場合は0。"""
    thr, _ = cv2.threshold(region, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    lo = region[region <= thr]
    hi = region[region > thr]
    n = region.size
    if lo.size < n * 0.003 or hi.size < n * 0.003:
        return 0.0
    s = math.sqrt(float(lo.std()) ** 2 + float(hi.std()) ** 2 + 16.0)
    return float(abs(float(hi.mean()) - float(lo.mean())) / s)


def combined_quality(m: Dict) -> float:
    """自動調整の評価値: 鮮鋭度(比)とコントラスト(分離度)の合成 + 飽和ペナルティ。大きいほど良い。"""
    ratio = m.get("ratio") or 0.0
    sep = m.get("sep") or 0.0
    if ratio <= 0 or sep <= 0:
        return 0.0
    val = (ratio ** 0.6) * (min(sep, 12.0) ** 0.4)
    pen = 1.0
    if m.get("clip_hi", 0.0) > 0.6:
        pen *= max(0.2, 1.0 - (m["clip_hi"] - 0.6) * 2.0)
    if m.get("clip_lo", 0.0) > 0.4:
        pen *= max(0.2, 1.0 - (m["clip_lo"] - 0.4) * 2.0)
    return val * pen


def focus_grade(score: Optional[float], thresholds: Optional[Dict] = None) -> str:
    """"good"|"fair"|"bad"|"unknown"。"""
    if score is None:
        return "unknown"
    th = thresholds or DEFAULT_FOCUS_THRESHOLDS
    if score >= th["good"]:
        return "good"
    if score >= th["fair"]:
        return "fair"
    return "bad"


def focus_grade_label(grade: str) -> str:
    return _FOCUS_GRADE_LABELS.get(grade, "測定不可")


def brightness_warnings(stats: Optional[Dict]) -> List[str]:
    """ヒストグラム由来の警告(暗い/明るすぎ/コントラスト不足)。数字画面は白っぽい背景が普通なので
    「平均が高い」だけでは警告せず、白飛び(250以上の画素の割合)で判定する。"""
    if not stats:
        return []
    out: List[str] = []
    if stats.get("mean", 128) < 60:
        out.append("映像が暗すぎます。画面の明るさを上げるか、照明・カメラの向きを調整してください")
    if stats.get("clip_hi", 0.0) > 0.6:
        out.append("映像が明るすぎて白飛びしています。画面の輝度を下げる/反射・照明を避けるか、自動調整を試してください")
    if stats.get("clip_lo", 0.0) > 0.5:
        out.append("映像の大部分が黒つぶれしています。カメラの向き・露出を確認してください")
    if stats.get("contrast", 255) < 50:
        out.append("コントラストが低いです(文字と背景の差が小さい)。照明・反射を確認するか、自動調整を試してください")
    return out


FOCUS_ADVICE = (
    "ピントが合っていません。カメラをもう少し離す/近づけてください"
    "(固定焦点のカメラは画面から一般に20〜40cm以上離す必要があります。"
    "「ピント合わせモード」で鮮鋭度が最大になる距離を探してください)"
)


class FocusMonitor:
    """撮影スレッドが毎フレーム(間引いて)鮮鋭度を入れ、確認画面が読み出す(スレッドセーフ)。
    移動平均で安定させ、ピーク値(最高)を記録し、悪い状態が続いたらガイダンスを出す。"""

    def __init__(
        self, window: int = 5, thresholds: Optional[Dict] = None, warn_after_sec: float = 20.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._lock = threading.Lock()
        self._window = max(1, int(window))
        self.thresholds = dict(thresholds or DEFAULT_FOCUS_THRESHOLDS)
        self._warn_after = float(warn_after_sec)
        self._clock = clock
        self._scores: Deque[float] = deque(maxlen=self._window)
        self._last: Optional[Dict] = None
        self._avg: Optional[float] = None
        self._peak: Optional[float] = None
        self._peak_samples = 0
        self._bad_since: Optional[float] = None
        self._count = 0
        self._resolution: Optional[Tuple[int, int]] = None
        self._updated_at: Optional[float] = None
        self._last_eval_at = -1e9
        self._suspended = False
        self._startup_logged = False

    def due(self, interval_sec: float) -> bool:
        return self._clock() - self._last_eval_at >= interval_sec

    def set_suspended(self, flag: bool) -> None:
        """自動調整中などは不良判定の継続時間を数えない。"""
        with self._lock:
            self._suspended = bool(flag)
            if flag:
                self._bad_since = None

    def reset_peak(self) -> None:
        with self._lock:
            self._peak = None
            self._peak_samples = 0

    def reset(self) -> None:
        """カメラ設定やプロファイルが変わった時: 移動平均とピークを捨てる。"""
        with self._lock:
            self._scores.clear()
            self._avg = None
            self._peak = None
            self._peak_samples = 0
            self._bad_since = None

    def update(self, cv2, np, frame, ratio_box=None) -> Optional[Dict]:
        self._last_eval_at = self._clock()
        m = edge_sharpness(cv2, np, frame, ratio_box)
        with self._lock:
            self._resolution = (int(frame.shape[1]), int(frame.shape[0]))
            self._updated_at = time.time()
            self._last = m
            if m is None:
                return None
            self._count += 1
            if m["score"] is not None:
                self._scores.append(float(m["score"]))
                self._avg = sum(self._scores) / len(self._scores)
                if self._peak is None or self._avg > self._peak:
                    self._peak = self._avg
                self._peak_samples += 1
                grade = focus_grade(self._avg, self.thresholds)
            else:
                grade = "unknown"
            now = self._clock()
            if grade == "bad" and not self._suspended:
                if self._bad_since is None:
                    self._bad_since = now
            elif grade != "bad":
                self._bad_since = None
        return m

    def snapshot(self) -> Dict:
        with self._lock:
            avg, peak = self._avg, self._peak
            grade = focus_grade(avg, self.thresholds)
            last = dict(self._last) if self._last else None
            warn_sec = None if self._bad_since is None else self._clock() - self._bad_since
            advice = FOCUS_ADVICE if (warn_sec is not None and warn_sec >= self._warn_after) else None
            near_peak = bool(
                avg is not None and peak is not None and self._peak_samples >= 8 and avg >= peak * 0.93
            )
            return {
                "score": None if last is None or last.get("score") is None else round(last["score"], 1),
                "avg": None if avg is None else round(avg, 1),
                "peak": None if peak is None else round(peak, 1),
                "ratio_to_peak": None if (avg is None or not peak) else round(avg / peak, 3),
                "near_peak": near_peak,
                "grade": grade,
                "grade_label": focus_grade_label(grade),
                "thresholds": dict(self.thresholds),
                "samples": self._count,
                "resolution": None if self._resolution is None else {
                    "width": self._resolution[0], "height": self._resolution[1]},
                "brightness": None if last is None else {
                    "mean": round(last["mean"], 1), "contrast": round(last["contrast"], 1),
                    "clip_hi": round(last["clip_hi"], 3), "clip_lo": round(last["clip_lo"], 3)},
                "warnings": brightness_warnings(last),
                "advice": advice,
                "bad_for_sec": None if warn_sec is None else round(warn_sec, 1),
                "updated_at": self._updated_at,
            }

    def startup_report_due(self, min_samples: int = 5) -> bool:
        """起動直後(最初の数サンプル後)に1回だけTrueを返す。"""
        with self._lock:
            if self._startup_logged or self._count < min_samples:
                return False
            self._startup_logged = True
            return True


def diagnose(cv2, np, frame, monitor: Optional[FocusMonitor], ratio_box=None) -> Dict:
    """「診断」ボタン/起動時: 解像度・鮮鋭度・明るさを調べて警告とガイダンスを返す。"""
    out: Dict = {"ok": frame is not None, "warnings": [], "advice": None}
    if frame is None:
        out["warnings"].append("カメラ映像がまだ取得できていません")
        return out
    h, w = frame.shape[:2]
    out["resolution"] = {"width": int(w), "height": int(h)}
    m = edge_sharpness(cv2, np, frame, ratio_box)
    if m is None:
        out["warnings"].append("対象領域が小さすぎて測定できません")
        return out
    score = m["score"]
    grade = focus_grade(score, monitor.thresholds if monitor else None)
    out["focus"] = {
        "score": None if score is None else round(score, 1), "grade": grade, "grade_label": focus_grade_label(grade),
    }
    out["brightness"] = {
        "mean": round(m["mean"], 1), "contrast": round(m["contrast"], 1),
        "clip_hi": round(m["clip_hi"], 3), "clip_lo": round(m["clip_lo"], 3),
    }
    out["warnings"] = brightness_warnings(m)
    if score is None:
        out["warnings"].append("対象(数字の画面)が写っていないか、コントラストが足りず鮮鋭度を測れません")
    if grade == "bad":
        out["advice"] = FOCUS_ADVICE
    if monitor is not None:
        snap = monitor.snapshot()
        out["peak"] = snap["peak"]
        if snap.get("advice"):
            out["advice"] = snap["advice"]
    return out


# =============================================================================
# (c) ソフトの画質改善(OCR前処理)
# =============================================================================
# 設定キー image_enhance: false で全て無効。辞書で個別に上書き(未指定は既定)。
#   best_of_n        直近N枚(静止画面)のうち鮮鋭度が最大のフレームを使う。int か {"demo":3,"production":5}。1以下=無効
#   average_frames   静止している間だけ、鮮鋭な順にこの枚数まで平均してノイズを減らす。int か プロファイル別dict。1以下=無効
#   unsharp_amount   アンシャープマスクの量(0=無効、既定0.8)。
#   unsharp_radius   同じ半径(px、拡大後の画像基準、既定2.0)。
#   clahe            CLAHEのclipLimit(0/false=無効、true=2.0)。既定は無効: 合成劣化画像での検証で、均一な
#                    照明の画面ではCLAHEが二値化の再現度をわずかに下げたため(照明ムラ・反射が強い現場で1.0〜2.0を試す)。
#   auto_levels      輝度の1〜99%点を0〜255へ伸ばす(低コントラスト補正。既定true)。
#   interpolation    拡大時の補間 "lanczos"(既定)|"cubic"。
#   max_age_sec      この秒数より古いフレームは使わない(数字の更新で古い値を拾わないため。既定8)。
DEFAULT_IMAGE_ENHANCE: Dict = {
    "best_of_n": {"demo": 3, "production": 5},
    "average_frames": {"demo": 2, "production": 3},
    "unsharp_amount": 0.8,
    "unsharp_radius": 2.0,
    "clahe": 0.0,
    "auto_levels": True,
    "interpolation": "lanczos",
    "max_age_sec": 8.0,
}
HISTORY_MAX_FRAMES = 10


@dataclass(frozen=True)
class EnhanceSettings:
    best_of_n: int = 1
    average_frames: int = 1
    unsharp_amount: float = 0.0
    unsharp_radius: float = 1.5
    clahe: float = 0.0
    auto_levels: bool = False
    interpolation: str = "cubic"
    max_age_sec: float = 8.0

    @property
    def any_frame_stage(self) -> bool:
        return self.best_of_n > 1 or self.average_frames > 1

    @property
    def any_crop_stage(self) -> bool:
        return self.unsharp_amount > 0 or self.clahe > 0 or self.auto_levels

    @property
    def enabled(self) -> bool:
        return self.any_frame_stage or self.any_crop_stage or self.interpolation == "lanczos"


DISABLED_ENHANCE = EnhanceSettings()


def _num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def resolve_image_enhance(config: Dict, profile: Optional[str] = None) -> EnhanceSettings:
    """config["image_enhance"] を検証して EnhanceSettings へ。キーが無い=既定(有効で控えめ)、
    false=全て無効。不正な値は警告して既定値を使う。profile=None(従来動作)ではdemo側の値。"""
    raw = config.get("image_enhance", None)
    if raw is False:
        return DISABLED_ENHANCE
    merged = dict(DEFAULT_IMAGE_ENHANCE)
    if raw is not None and raw is not True:
        if not isinstance(raw, dict):
            logger.warning("image_enhance は辞書かfalseにしてください(%r)。既定値を使います", raw)
        else:
            for k, v in raw.items():
                if k.startswith("_"):
                    continue
                if k not in merged:
                    logger.warning("image_enhance の未知のキーを無視します: %r", k)
                    continue
                merged[k] = v
    pname = profile if profile in ("demo", "production") else "demo"

    def n_frames(key: str) -> int:
        v = merged[key]
        if isinstance(v, dict):
            v = v.get(pname, DEFAULT_IMAGE_ENHANCE[key][pname])
        if _num(v) and float(v).is_integer() and 0 <= int(v) <= HISTORY_MAX_FRAMES:
            return max(1, int(v))
        logger.warning("image_enhance.%s=%r は不正です(0〜%d の整数)。既定値を使います", key, v, HISTORY_MAX_FRAMES)
        d = DEFAULT_IMAGE_ENHANCE[key]
        return int(d[pname] if isinstance(d, dict) else d)

    def ranged(key: str, lo: float, hi: float) -> float:
        v = merged[key]
        if v is False:
            return 0.0
        if v is True and key == "clahe":
            return 2.0
        if _num(v) and lo <= float(v) <= hi:
            return float(v)
        logger.warning("image_enhance.%s=%r は不正です(%g〜%g)。既定値を使います", key, v, lo, hi)
        return float(DEFAULT_IMAGE_ENHANCE[key])

    interp = merged["interpolation"]
    if interp not in ("lanczos", "cubic"):
        logger.warning("image_enhance.interpolation=%r は不正です(lanczos/cubic)。lanczosにします", interp)
        interp = "lanczos"
    auto_levels = merged["auto_levels"]
    if not isinstance(auto_levels, bool):
        logger.warning("image_enhance.auto_levels=%r は不正です(true/false)。trueにします", auto_levels)
        auto_levels = True
    return EnhanceSettings(
        best_of_n=n_frames("best_of_n"), average_frames=n_frames("average_frames"),
        unsharp_amount=ranged("unsharp_amount", 0.0, 3.0), unsharp_radius=ranged("unsharp_radius", 0.3, 10.0),
        clahe=ranged("clahe", 0.0, 10.0), auto_levels=auto_levels, interpolation=str(interp),
        max_age_sec=ranged("max_age_sec", 0.5, 120.0),
    )


def auto_levels_gray(np, gray, lo_p: float = 1.0, hi_p: float = 99.0):
    """輝度の下位/上位パーセンタイルを0〜255へ伸ばす。ダイナミックレンジが既に十分(>=200)なら何もしない。"""
    lo, hi = np.percentile(gray, (lo_p, hi_p))
    if hi - lo >= 200 or hi - lo < 8:
        return gray
    out = (gray.astype(np.float32) - lo) * (255.0 / (hi - lo))
    return np.clip(out, 0, 255).astype(np.uint8)


def auto_levels_bgr(np, img, lo_p: float = 1.0, hi_p: float = 99.0):
    """色(彩度)を保ったまま全チャンネル共通の線形ストレッチ(色による行検出を壊さない)。"""
    lo, hi = np.percentile(img, (lo_p, hi_p))
    if hi - lo >= 200 or hi - lo < 8:
        return img
    out = (img.astype(np.float32) - lo) * (255.0 / (hi - lo))
    return np.clip(out, 0, 255).astype(np.uint8)


def unsharp_mask(cv2, np, img, amount: float, radius: float):
    """out = img + amount*(img - blur(img, radius))。amount<=0なら何もしない。"""
    if amount <= 0:
        return img
    blurred = cv2.GaussianBlur(img, (0, 0), float(radius))
    return cv2.addWeighted(img, 1.0 + amount, blurred, -amount, 0)


def clahe_gray(cv2, gray, clip_limit: float, grid: int = 4):
    if clip_limit <= 0:
        return gray
    c = cv2.createCLAHE(clipLimit=float(clip_limit), tileGridSize=(grid, grid))
    return c.apply(gray)


def interpolation_flag(cv2, name: str, scale_factor: float) -> int:
    """拡大はLANCZOS4/CUBIC、縮小はAREA。"""
    if scale_factor < 1.0:
        return cv2.INTER_AREA
    return cv2.INTER_LANCZOS4 if name == "lanczos" else cv2.INTER_CUBIC


def enhance_gray_for_ocr(cv2, np, gray, scale_factor: float, es: EnhanceSettings):
    """preprocess_for_ocr から呼ぶ: グレー画像 → 自動レベル → 拡大(Lanczos/Cubic) → CLAHE → アンシャープ。
    二値化(大津)は呼び出し側。esが全て無効なら従来どおり(Cubic拡大のみ)になる。"""
    if es.auto_levels:
        gray = auto_levels_gray(np, gray)
    gray = cv2.resize(gray, None, fx=scale_factor, fy=scale_factor,
                      interpolation=interpolation_flag(cv2, es.interpolation, scale_factor))
    if es.clahe > 0:
        gray = clahe_gray(cv2, gray, es.clahe)
    if es.unsharp_amount > 0:
        gray = unsharp_mask(cv2, np, gray, es.unsharp_amount, es.unsharp_radius)
    return gray


class FrameHistory:
    """直近の撮影フレーム(参照)を時刻付きで保持するリングバッファ(スレッドセーフ)。
    撮影スレッドがpush、run_loopがsnapshotする。カメラ設定/回転/プロファイルが変わったらclear。"""

    def __init__(self, maxlen: int = HISTORY_MAX_FRAMES, min_interval_sec: float = 0.1,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._lock = threading.Lock()
        self._buf: Deque[Tuple[float, object]] = deque(maxlen=max(1, int(maxlen)))
        self._min_interval = float(min_interval_sec)
        self._clock = clock
        self._last_push = -1e9

    def push(self, frame) -> bool:
        now = self._clock()
        with self._lock:
            if now - self._last_push < self._min_interval:
                return False
            self._last_push = now
            self._buf.append((now, frame))
            return True

    def clear(self) -> None:
        with self._lock:
            self._buf.clear()
            self._last_push = -1e9

    def snapshot(self) -> List[Tuple[float, object]]:
        with self._lock:
            return list(self._buf)

    def __len__(self) -> int:
        with self._lock:
            return len(self._buf)


def _is_static(cv2, np, ga, gb, max_block_diff: float = 45.0) -> bool:
    """2フレームが「静止(同じ画面)」か。4x4ブロック平均の差の最大値で判定する
    (センサーノイズは平均化で小さくなり、数字の変化・手ぶれによる位置ずれは大きな差として残る)。"""
    h, w = ga.shape[:2]
    if (h, w) != gb.shape[:2]:
        return False
    sa = cv2.resize(ga, (max(1, w // 4), max(1, h // 4)), interpolation=cv2.INTER_AREA)
    sb = cv2.resize(gb, (max(1, w // 4), max(1, h // 4)), interpolation=cv2.INTER_AREA)
    return float(np.abs(sa.astype(np.int16) - sb.astype(np.int16)).max()) < max_block_diff


def build_enhanced_frame(cv2, np, history, ratio_box, es: EnhanceSettings, now: Optional[float] = None):
    """(a)ベストフレーム選択 + (b)静止時の複数フレーム平均。
    history: [(timestamp, frame), ...](古い→新しい)。戻り値 (frame, info)。
    ステージが無効/フレームが足りない場合は最新フレームをそのまま返す。"""
    if not history:
        return None, {"used": 0, "averaged": 0, "best_score": None}
    newest = history[-1][1]
    info = {"used": 1, "averaged": 1, "best_score": None, "considered": 1}
    if not es.any_frame_stage or len(history) < 2:
        return newest, info
    t_now = history[-1][0] if now is None else now
    pool = [f for ts, f in history if t_now - ts <= es.max_age_sec and f.shape == newest.shape]
    pool = pool[-max(es.best_of_n, es.average_frames):]
    if len(pool) < 2:
        return newest, info
    scores = []
    for f in pool:
        m = edge_sharpness(cv2, np, f, ratio_box)
        scores.append(0.0 if m is None or m["score"] is None else float(m["score"]))
    info["considered"] = len(pool)
    recent = list(range(len(pool)))[-max(1, es.best_of_n):]
    best_i = max(recent, key=lambda i: (scores[i], i)) if es.best_of_n > 1 else len(pool) - 1
    info["best_score"] = round(scores[best_i], 1)
    best = pool[best_i]
    if es.average_frames <= 1:
        info["used"] = 1
        return best, info
    gb = _to_gray(cv2, best)
    group = [best]
    for i in sorted(range(len(pool)), key=lambda i: (-scores[i], -i)):
        if i == best_i:
            continue
        if len(group) >= es.average_frames:
            break
        if _is_static(cv2, np, gb, _to_gray(cv2, pool[i])):
            group.append(pool[i])
    info["averaged"] = len(group)
    info["used"] = len(group)
    if len(group) == 1:
        return best, info
    acc = np.zeros(best.shape, np.float32)
    for f in group:
        acc += f.astype(np.float32)
    return np.clip(acc / len(group) + 0.5, 0, 255).astype(np.uint8), info


def detection_variants(cv2, np, frame, es: EnhanceSettings):
    """detect_digit_rowsが期待行数を返せなかった時に順に試す強化版フレーム(名前, 画像)。
    ぼけ・低コントラストで赤茶色の彩度が落ちると検出が失敗するので、色を保ったまま
    コントラストを伸ばし、さらにアンシャープで輪郭を締める。"""
    if not es.any_crop_stage:
        return
    leveled = auto_levels_bgr(np, frame)
    if leveled is not frame:
        yield "levels", leveled
    yield "levels+unsharp1.5", unsharp_mask(cv2, np, leveled, 1.5, 2.0)
    yield "levels+unsharp2.5", unsharp_mask(cv2, np, leveled, 2.5, 3.0)


# =============================================================================
# (b) UVC設定の自動チューニング
# =============================================================================
# OpenCVのCAP_PROP_*でUVCを操作する(v4l2-ctlが無い環境向け)。機種ごとに範囲が違い、OpenCVは範囲を
# 返さないので、範囲外の値を設定→クランプ/拒否→読み戻し で上限・下限を探る。
TUNE_PROPS: Dict[str, Tuple[str, str]] = {
    # key: (cv2の定数名, 表示名)
    "focus": ("CAP_PROP_FOCUS", "ピント"),
    "sharpness": ("CAP_PROP_SHARPNESS", "シャープネス"),
    "contrast": ("CAP_PROP_CONTRAST", "コントラスト"),
    "gamma": ("CAP_PROP_GAMMA", "ガンマ"),
    "brightness": ("CAP_PROP_BRIGHTNESS", "明るさ"),
    "backlight": ("CAP_PROP_BACKLIGHT", "逆光補正"),
    "wb_temperature": ("CAP_PROP_WB_TEMPERATURE", "ホワイトバランス"),
}
TUNE_ORDER = ["focus", "sharpness", "contrast", "gamma", "brightness", "backlight", "wb_temperature"]
# 自動調整が補助的に触るスイッチ(AFを切る/オートWBを切る)。reapply・revertでは先に設定する。
SWITCH_PROPS: Dict[str, str] = {"autofocus": "CAP_PROP_AUTOFOCUS", "auto_wb": "CAP_PROP_AUTO_WB"}
# 項目ごとの「採用するのに必要な最低改善率」(計測ノイズへの過剰適合を避ける。WBは色検出に響くので厳しめ)
MIN_GAIN = {"wb_temperature": 0.05, "focus": 0.03}
DEFAULT_MIN_GAIN = 0.02

_HI_PROBES = [65535, 10000, 4000, 1000, 500, 255, 200, 150, 128, 100, 90, 80, 64, 63, 50, 40, 32, 31, 30, 25, 20,
              16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
_LO_PROBES = [-65535, -10000, -4000, -1000, -500, -255, -200, -150, -128, -100, -90, -80, -64, -63, -50, -40, -32,
              -31, -30, -25, -20, -16, -10, -8, -5, -4, -3, -2, -1, 0]


def cv_prop(cv2, key: str):
    name = TUNE_PROPS[key][0] if key in TUNE_PROPS else SWITCH_PROPS.get(key)
    return getattr(cv2, name, None) if name else None


def _get(cap, prop) -> Optional[float]:
    try:
        v = cap.get(prop)
    except Exception:  # noqa: BLE001
        return None
    if v is None or (isinstance(v, float) and (math.isnan(v) or math.isinf(v))):
        return None
    return float(v)


def _set(cap, prop, value) -> bool:
    try:
        return bool(cap.set(prop, float(value)))
    except Exception:  # noqa: BLE001
        return False


def probe_range(cv2, cap, key: str) -> Optional[Dict]:
    """プロパティの現在値と範囲を探る。{"cur","lo","hi"} か、効かない/読めない場合None。
    終わったら現在値へ戻す。"""
    prop = cv_prop(cv2, key)
    if prop is None:
        return None
    cur = _get(cap, prop)
    if cur is None:
        return None
    hi = lo = None
    try:
        # OpenCVが0..1へ正規化している機種(現在値が小数)
        if 0.0 <= cur <= 1.0 and abs(cur - round(cur)) > 1e-6:
            if _set(cap, prop, 1.0) and abs((_get(cap, prop) or -9) - 1.0) < 1e-6:
                hi = 1.0
            if _set(cap, prop, 0.0) and abs((_get(cap, prop) or -9) - 0.0) < 1e-6:
                lo = 0.0
        else:
            for v in _HI_PROBES:
                if not _set(cap, prop, v):
                    continue
                rb = _get(cap, prop)
                if rb is None:
                    continue
                if abs(rb - v) < 0.5 or (v > rb >= cur):  # 受理 or 上限へクランプされた
                    hi = rb
                    break
            for v in _LO_PROBES:
                if not _set(cap, prop, v):
                    continue
                rb = _get(cap, prop)
                if rb is None:
                    continue
                if abs(rb - v) < 0.5 or (v < rb <= cur):  # 受理 or 下限へクランプされた
                    lo = rb
                    break
    finally:
        _set(cap, prop, cur)
    if hi is None or lo is None or hi <= lo:
        return None
    return {"cur": cur, "lo": lo, "hi": hi}


def candidate_values(lo: float, hi: float, cur: float, n: int = 5) -> List[float]:
    """粗い探索の候補(範囲を等分、現在値も含む、重複除去、小さい順)。"""
    integer = (hi - lo) >= 4
    vals = [lo + (hi - lo) * i / (n - 1) for i in range(n)] + [cur]
    out: List[float] = []
    for v in vals:
        v = max(lo, min(hi, v))
        v = float(round(v)) if integer else round(v, 3)
        if all(abs(v - o) > 1e-9 for o in out):
            out.append(v)
    return sorted(out)


def apply_values(cv2, cap, values: Dict[str, float]) -> Dict[str, bool]:
    """保存値の再適用/元へ戻す。スイッチ(auto_wb, autofocus)を先に設定。各項目の成否を返す。"""
    results: Dict[str, bool] = {}
    ordered = [k for k in ("autofocus", "auto_wb") if k in values] + [k for k in values if k not in SWITCH_PROPS]
    for key in ordered:
        prop = cv_prop(cv2, key)
        if prop is None or not isinstance(values[key], (int, float)) or isinstance(values[key], bool):
            results[key] = False
            continue
        ok = _set(cap, prop, values[key])
        rb = _get(cap, prop)
        results[key] = bool(ok and rb is not None and abs(rb - float(values[key])) <= max(0.5, abs(values[key]) * 0.03))
    return results


def validate_tuning_record(rec) -> Optional[Dict]:
    """設定ファイルの camera_tuning[profile] を検証。{"applied":{k:数値}, "original":{k:数値}, "tuned_at":str}。
    未知のキー・非数値は捨てる。使える値が無ければNone。"""
    if not isinstance(rec, dict):
        return None

    def clean(d):
        out = {}
        if isinstance(d, dict):
            for k, v in d.items():
                if (k in TUNE_PROPS or k in SWITCH_PROPS) and isinstance(v, (int, float)) and not isinstance(v, bool) \
                        and math.isfinite(float(v)):
                    out[k] = float(v)
        return out

    applied = clean(rec.get("applied"))
    if not applied:
        return None
    return {"applied": applied, "original": clean(rec.get("original")), "tuned_at": str(rec.get("tuned_at", ""))}


class CameraTuner:
    """UVC設定の自動チューニング(撮影スレッドで実行)。

    cap: cv2.VideoCapture互換(set/get)。read_frame(): 最新フレーム(BGR)かNone。
    box_provider(): ROIの外接枠(比率)かNone(画面中央)。progress(done, total, label): 進捗通知。
    cancelled(): Trueなら中断して元へ戻す。clock/frames引数は主にテスト用。
    """

    def __init__(
        self, cv2, np, cap, read_frame: Callable[[], object], box_provider: Callable[[], object] = lambda: None,
        progress: Optional[Callable[[int, int, str], None]] = None,
        cancelled: Callable[[], bool] = lambda: False, settle_frames: int = 4, measure_frames: int = 3,
        max_seconds: float = 150.0, clock: Callable[[], float] = time.monotonic,
        props: Optional[Sequence[str]] = None,
    ) -> None:
        self.cv2, self.np, self.cap = cv2, np, cap
        self.read_frame = read_frame
        self.box_provider = box_provider
        self.progress = progress or (lambda d, t, l: None)
        self.cancelled = cancelled
        self.settle_frames = max(0, int(settle_frames))
        self.measure_frames = max(1, int(measure_frames))
        self.max_seconds = float(max_seconds)
        self.clock = clock
        self.props = [p for p in (props or TUNE_ORDER) if p in TUNE_PROPS]
        self._evals = 0
        self._done = 0
        self._total = 1
        self._t0 = 0.0

    # ---- 測定 ----
    def _measure(self) -> Optional[Dict]:
        """設定直後の数フレームを捨ててから複数フレームを測り、各指標の中央値を返す。"""
        for _ in range(self.settle_frames):
            if self.read_frame() is None:
                pass
        ms: List[Dict] = []
        misses = 0
        while len(ms) < self.measure_frames and misses < 6:
            f = self.read_frame()
            if f is None:
                misses += 1
                continue
            m = edge_sharpness(self.cv2, self.np, f, self.box_provider())
            if m is not None:
                ms.append(m)
        self._evals += 1
        self._done += 1
        if not ms:
            return None
        out = {}
        for k in ("ratio", "sep", "clip_hi", "clip_lo", "mean", "contrast"):
            vals = [m[k] for m in ms if m.get(k) is not None]
            out[k] = float(self.np.median(vals)) if vals else None
        out["value"] = combined_quality(out)
        return out

    def _over_time(self) -> bool:
        return self.clock() - self._t0 > self.max_seconds

    def _report(self, label: str) -> None:
        try:
            self.progress(min(self._done, self._total), self._total, label)
        except Exception:  # noqa: BLE001
            pass

    # ---- 本体 ----
    def run(self) -> Dict:
        cv2, cap = self.cv2, self.cap
        self._t0 = self.clock()
        result: Dict = {
            "ok": False, "status": "failed", "message": "", "changes": {}, "skipped": {}, "applied": {},
            "original": {}, "baseline": None, "final": None, "evals": 0, "elapsed_sec": 0.0, "focus_control": None,
        }
        originals: Dict[str, float] = {}  # 変更した項目の元の値(復元用)
        try:
            ranges: Dict[str, Dict] = {}
            for key in self.props:
                if key == "focus":
                    continue
                r = probe_range(cv2, cap, key)
                if r is None:
                    result["skipped"][key] = "効かない/読み戻せない(この機種では制御できません)"
                else:
                    ranges[key] = r
            focus_r = self._probe_focus(result)
            if focus_r is not None:
                ranges["focus"] = focus_r
            order = [k for k in self.props if k in ranges]
            self._total = 1 + sum(8 for _ in order) + 1
            self._report("基準を測定中")
            base = self._measure()
            if base is None or base["value"] <= 0:
                result["message"] = "対象(数字の画面)が写っていないか、コントラストが足りず評価できません。画面を写してからやり直してください"
                result["baseline"] = None if base is None else self._pub(base)
                self._restore(originals)
                return self._finish(result, originals)
            result["baseline"] = self._pub(base)
            current_val = base["value"]
            for key in order:
                if self.cancelled():
                    self._restore(originals)
                    result["status"], result["message"] = "cancelled", "中断しました(元の設定へ戻しました)"
                    return self._finish(result, originals)
                if self._over_time():
                    result["message"] = "時間制限のため途中までの結果を採用しました"
                    break
                label = TUNE_PROPS[key][1]
                self._report(f"{label}を調整中")
                outcome = self._tune_one(key, ranges[key], current_val, originals)
                if outcome[0] == "skip":
                    result["skipped"][key] = outcome[1]
                else:
                    current_val = outcome[1]
            self._report("結果を確認中")
            final = self._measure()
            result["final"] = None if final is None else self._pub(final)
            if final is None or final["value"] < base["value"] * (1.0 + DEFAULT_MIN_GAIN * 0.5):
                # 全体として改善が無ければ全て元へ戻す
                self._restore(originals)
                result.update(status="unchanged", ok=True,
                              message="設定を変えても改善しなかったため、元の設定のままにしました")
                originals.clear()
            else:
                result.update(status="improved", ok=True)
                if not result["message"]:
                    gain = (final["value"] / base["value"] - 1.0) * 100.0
                    result["message"] = f"画質評価値が {gain:+.0f}% 改善しました"
        except Exception as exc:  # noqa: BLE001
            logger.exception("画質の自動調整中にエラー(元の設定へ戻します)")
            try:
                self._restore(originals)
            except Exception:  # noqa: BLE001
                pass
            originals.clear()
            result.update(status="failed", ok=False, message=f"自動調整に失敗しました: {exc}")
        return self._finish(result, originals)

    @staticmethod
    def _pub(m: Dict) -> Dict:
        return {k: (None if m.get(k) is None else round(float(m[k]), 4)) for k in
                ("value", "ratio", "sep", "clip_hi", "mean", "contrast")}

    def _finish(self, result: Dict, originals: Dict[str, float]) -> Dict:
        result["evals"] = self._evals
        result["elapsed_sec"] = round(self.clock() - self._t0, 1)
        applied: Dict[str, float] = {}
        changes: Dict[str, Dict] = {}
        for key, before in originals.items():
            prop = cv_prop(self.cv2, key)
            after = _get(self.cap, prop) if prop is not None else None
            if after is None:
                continue
            applied[key] = after
            if key in TUNE_PROPS:
                changes[key] = {"label": TUNE_PROPS[key][1], "before": before, "after": after}
        result["original"] = dict(originals)
        result["applied"] = applied
        result["changes"] = changes
        self._done = self._total
        self._report("完了")
        return result

    def _restore(self, originals: Dict[str, float]) -> None:
        if originals:
            apply_values(self.cv2, self.cap, dict(originals))

    def _probe_focus(self, result: Dict) -> Optional[Dict]:
        """フォーカス制御が効くか試す(効かなくても動作継続)。AFを切って手動ピントを探索対象にする。"""
        cv2, cap = self.cv2, self.cap
        info = {"autofocus_settable": False, "focus_settable": False}
        result["focus_control"] = info
        af = cv_prop(cv2, "autofocus")
        fp = cv_prop(cv2, "focus")
        if "focus" not in self.props:
            return None
        if af is not None:
            before = _get(cap, af)
            if before is not None and _set(cap, af, 0.0):
                rb = _get(cap, af)
                info["autofocus_settable"] = bool(rb is not None and abs(rb - 0.0) < 0.5 and abs(before - 0.0) >= 0.5)
                _set(cap, af, before)
        if fp is None:
            result["skipped"]["focus"] = "フォーカス(固定焦点のため制御できません。カメラの距離で合わせてください)"
            return None
        r = probe_range(cv2, cap, "focus")
        if r is None:
            result["skipped"]["focus"] = "フォーカス(固定焦点のため制御できません。カメラの距離で合わせてください)"
            return None
        info["focus_settable"] = True
        return r

    def _tune_one(self, key: str, rng: Dict, base_val: float, originals: Dict[str, float]):
        """1項目を粗→細の2段階で探索し、最良値を設定する。("skip", 理由) か ("ok", 採用後の評価値)。"""
        cv2, cap = self.cv2, self.cap
        prop = cv_prop(cv2, key)
        lo, hi, cur = rng["lo"], rng["hi"], rng["cur"]
        tol = 0.5 if (hi - lo) >= 4 else 0.05 * (hi - lo)
        if key == "wb_temperature":
            # 自動WBを切らないと色温度は効かない
            aw = cv_prop(cv2, "auto_wb")
            if aw is not None:
                before_aw = _get(cap, aw)
                if before_aw is not None and before_aw >= 0.5:
                    _set(cap, aw, 0.0)
                    rb = _get(cap, aw)
                    if rb is None or rb >= 0.5:
                        return "skip", "自動ホワイトバランスを切れないため調整できません"
                    originals.setdefault("auto_wb", before_aw)
        if key == "focus":
            af = cv_prop(cv2, "autofocus")
            if af is not None:
                before_af = _get(cap, af)
                if before_af is not None and before_af >= 0.5:
                    _set(cap, af, 0.0)
                    originals.setdefault("autofocus", before_af)
        evaluated: Dict[float, float] = {}
        readbacks = set()

        def try_value(v: float) -> None:
            if v in evaluated:
                return
            if not _set(cap, prop, v):
                return
            rb = _get(cap, prop)
            if rb is None or abs(rb - v) > tol:
                return
            readbacks.add(round(rb, 3))
            m = self._measure()
            evaluated[v] = 0.0 if m is None else m["value"]

        # 基準(現在値)を今の照明で測り直す(ドリフト対策)
        _set(cap, prop, cur)
        for v in candidate_values(lo, hi, cur, 5):
            if self.cancelled() or self._over_time():
                break
            try_value(v)
        if len(readbacks) < 2:
            _set(cap, prop, cur)
            return "skip", "設定しても読み戻しが変化しない(効かない項目)"
        best_v = max(evaluated, key=lambda v: evaluated[v])
        # 細かい探索: 最良値の左右の中点
        keys = sorted(evaluated)
        i = keys.index(best_v)
        fine: List[float] = []
        integer = (hi - lo) >= 4
        for j in (i - 1, i + 1):
            if 0 <= j < len(keys):
                mid = (keys[j] + best_v) / 2.0
                mid = float(round(mid)) if integer else round(mid, 3)
                if mid not in evaluated and abs(mid - best_v) > 1e-9:
                    fine.append(mid)
        for v in fine:
            if self.cancelled() or self._over_time():
                break
            try_value(v)
        best_v = max(evaluated, key=lambda v: evaluated[v])
        cur_val = evaluated.get(cur, base_val)
        spread = max(evaluated.values()) - min(evaluated.values())
        gain_need = MIN_GAIN.get(key, DEFAULT_MIN_GAIN)
        chosen = cur
        if evaluated[best_v] > cur_val * (1.0 + gain_need):
            chosen = best_v
        if base_val > 0 and spread < base_val * 0.003:
            chosen = cur  # どの値でも評価値が変わらない(影響なし)
        _set(cap, prop, chosen)
        rb = _get(cap, prop)
        if chosen != cur and rb is not None:
            originals.setdefault(key, cur)
            return "ok", evaluated[chosen]
        # 変更しなかった項目: 補助スイッチ(AF/オートWB)を切っていたら元へ戻す
        sw = {"wb_temperature": "auto_wb", "focus": "autofocus"}.get(key)
        if sw and sw in originals:
            apply_values(cv2, cap, {sw: originals.pop(sw)})
        if spread < base_val * 0.003:
            return "skip", "値を変えても画質の評価値が変化しない(影響なし)"
        return "ok", cur_val


# =============================================================================
# カメラ設定の保存値(camera_tuning)の再適用
# =============================================================================
def tuning_profile_key(profile: Optional[str]) -> str:
    return profile if profile in ("demo", "production") else "default"


def reapply_tuning(cv2, cap, tuning: Dict[str, Dict], profile: Optional[str]) -> Dict:
    """camera_tuning[profile] を再適用する。そのプロファイルの保存値が無く、他のプロファイルの
    保存値(=UVCに残っている調整値)があれば、その「元の値」へ戻す(UVCの設定は閉じても残るため)。
    戻り値: {"mode": "applied"|"restored"|"none", "results": {...}}。例外は出さない。"""
    try:
        rec = tuning.get(tuning_profile_key(profile))
        if rec:
            return {"mode": "applied", "results": apply_values(cv2, cap, rec["applied"])}
        originals: Dict[str, float] = {}
        for other in tuning.values():
            for k, v in other.get("original", {}).items():
                originals.setdefault(k, v)
        if originals:
            return {"mode": "restored", "results": apply_values(cv2, cap, originals)}
    except Exception as exc:  # noqa: BLE001
        logger.warning("保存したカメラ調整の再適用に失敗しました: %s", exc)
    return {"mode": "none", "results": {}}


def resolve_focus_thresholds(value) -> Dict[str, float]:
    """config["focus_thresholds"] の検証。{"good":x,"fair":y} で good>fair>0 のときだけ採用、
    それ以外は警告して既定値。"""
    if value is None:
        return dict(DEFAULT_FOCUS_THRESHOLDS)
    try:
        good, fair = float(value["good"]), float(value["fair"])
        if math.isfinite(good) and math.isfinite(fair) and good > fair > 0:
            return {"good": good, "fair": fair}
    except (KeyError, TypeError, ValueError):
        pass
    logger.warning("focus_thresholds=%r は不正です({\"good\":数,\"fair\":数}でgood>fair>0)。既定値を使います", value)
    return dict(DEFAULT_FOCUS_THRESHOLDS)


def resolve_camera_tuning(value) -> Dict[str, Dict]:
    """config["camera_tuning"] を検証して {プロファイル名: record} にする。不正なものは捨てる。"""
    out: Dict[str, Dict] = {}
    if value is None:
        return out
    if not isinstance(value, dict):
        logger.warning("camera_tuning は辞書ではないため無視します: %r", value)
        return out
    for name, rec in value.items():
        if name not in ("demo", "production", "default"):
            logger.warning("camera_tuning の未知のプロファイル名を無視します: %r", name)
            continue
        clean = validate_tuning_record(rec)
        if clean is None:
            logger.warning("camera_tuning[%s] に使える値が無いため無視します", name)
            continue
        out[name] = clean
    return out
