"""iPad表示(/display)の画面サイズの判定と、OCRテスト画面(模擬POS)の物理サイズの計算(純Python、ROS非依存)。

docs/ipad_display.md の「サイズ判定」「OCRテスト画面の計算」の根拠と、画面HTML(display_page.py)の JS が同じ式を使う。
JS側の式は display_page.py の `NKSIZE-BEGIN`〜`NKSIZE-END` の中にあり、test_ipad_display.py が node で動かして
このモジュールと同じ結果になることを確かめる(式を変えたら両方直す)。

■ 考え方(なぜ CSS の mm 単位を使わないか)
  CSSの `1mm` は「96dpi換算で 3.78 CSS px」と定義されているだけで、iPadの実物の長さとは一致しない
  (iPadのCSS px は 1/132 インチ。だから `103mm` と書くと実物では約75mmにしか見えない)。
  カメラで画面を撮って数字を読むOCRテストでは、実物の長さ(mm)が合っていないと、数字の大きさが
  本物のタブレットと違って見え、ROI(読み取り範囲)の設定が合わなくなる。
  そこで、機種ごとの「CSS px あたりの実寸」を表にして、`px = mm * (ppi / 25.4)` で自分で換算する。

■ iPadの寸法(Appleの公開仕様。ネイティブ解像度 / 264ppi、CSS px は 1/2 倍)
  - 12.9インチ(iPad Pro 12.9): ネイティブ 2048x2732 @264ppi → CSS 1024x1366。表示領域 約 197.0 x 262.9 mm
    (本体外形は 215 x 281 mm。ベゼルを含むので、画面の寸法としては使えない)
  - 11インチ(iPad Pro 11 / Air 11): ネイティブ 1668x2388(Pro11、CSS 834x1194)または 1640x2360
    (Air/第10世代 10.9、CSS 820x1180) @264ppi。表示領域 約 160.5 x 229.8 mm(Pro 11)
    (本体外形は 178 x 248 mm)
  どちらも CSS px の密度は 132 px/inch(= 264ppi の半分)で同じ。つまり **同じ CSS px 数は、12.9 でも 11 でも同じ実寸**になる。
  (例外: iPad mini 6/7 は 326ppi → CSS 163 px/inch。744x1133 のとき 163 を使う)

■ 横向きが本命(2026-10-27)
  本番の12.9インチも、テスト用の11インチも、**横向き**に付ける。横向きでは幅と高さが入れ替わる:
    12.9 横: CSS 1366x1024 = 262.9 x 197.0 mm      11 横: CSS 1194x834 (Pro 11) = 229.8 x 160.5 mm / 1180x820 (Air 10.9) = 227.0 x 157.8 mm
  どちらも縦横比はほぼ 4:3(1.334 / 1.432 / 1.439)。レイアウトの基準の長さ u は「短辺(=横向きでは高さ)の1/100」なので、同じ比率で組める。
  Safari のアドレスバーやホーム画面アプリのステータスバー分、実際に使える高さは 834 → 約 760〜834 に減る。

■ 模擬POS(7インチ端末 縦向き 103.35 x 144.68 mm = 537 x 752 px)の置き方(判断の詳細は docs/ipad_display.md 7章)
  採用: **縦向きのまま・実寸のまま、横向きiPadの中央に置く**(本物のタブレットは縦向きのまま撮影するので、これが一番近い見え方。frame_rotate_deg=0 のまま)。
  入るか: 縦 752px が使える高さに入ること。12.9横=1024 → 余白 272px(73%)。11横=834 → 余白 82px(90%)、820 → 68px(92%)。
  入らないとき: 自動で入る大きさまで縮め、横の余白に知らせる(実寸ではなくなる)。または ?ocrrot=90 で横向き端末として回して置く
  (外形 752x537px なので高さ 537px で余裕。frame_rotate_deg を 270 にして撮影側で戻す)。
"""
from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

MM_PER_INCH = 25.4
CSS_PPI_FULLSIZE = 132.0    # 9.7〜13インチのiPad: 264ppi / 2
CSS_PPI_MINI = 163.0        # iPad mini 6/7: 326ppi / 2

# 判定の区分(JSの data-size と同じ値)。キーはユーザーが ?size= で指定できる名前
SIZE_KEYS = ("12.9", "11", "small", "phone", "other")

# ?size= の別名 → 区分
SIZE_ALIASES: Dict[str, str] = {
    "12.9": "12.9", "13": "12.9", "12": "12.9", "xl": "12.9",
    "11": "11", "10.9": "11", "10.5": "11", "10.2": "11", "l": "11",
    "9.7": "small", "8.3": "small", "mini": "small", "small": "small", "m": "small",
    "phone": "phone", "s": "phone",
}

# 代表的な機種(CSS px の短辺 → 説明)。docs用
KNOWN_CSS_SHORT_SIDES: Dict[int, str] = {
    1032: "iPad Pro 13(M4)",
    1024: "iPad Pro 12.9",
    834: "iPad Pro 11 / iPad Pro 10.5(834x1112)",
    820: "iPad Air 10.9 / 11、iPad 第10世代",
    810: "iPad 10.2",
    768: "iPad 9.7 / iPad mini 5(ppiが違う: mini は ?ppi=163)",
    744: "iPad mini 6/7(163 px/inch)",
}

# 横向きで使う画面の大きさ(CSS px。幅, 高さ)。本命。ブラウザの窓(アドレスバーなど)で高さが減る前の、端末そのものの大きさ
LANDSCAPE_CSS: Dict[str, Tuple[int, int]] = {"12.9": (1366, 1024), "11": (1194, 834), "11air": (1180, 820)}

# 実機iPadの寸法(docs/テスト用)。active_mm = (幅, 高さ) の縦向き表示領域
SIZE_PROFILES: Dict[str, Dict[str, Any]] = {
    "12.9": {"label": "12.9インチ(本番の機体)", "css": (1024, 1366), "native": (2048, 2732), "native_ppi": 264.0,
             "ppi": CSS_PPI_FULLSIZE},
    "11": {"label": "11インチ(テスト用)", "css": (834, 1194), "native": (1668, 2388), "native_ppi": 264.0,
           "ppi": CSS_PPI_FULLSIZE},
}


def active_area_mm(key: str, landscape: bool = False) -> Tuple[float, float]:
    """表示領域の実寸 (幅mm, 高さmm)。ネイティブ解像度 / ネイティブppi から。landscape=True なら横向き(幅と高さを入れ替える)。"""
    p = SIZE_PROFILES[key]
    w, h = p["native"]
    a = (w / p["native_ppi"] * MM_PER_INCH, h / p["native_ppi"] * MM_PER_INCH)
    return (a[1], a[0]) if landscape else a


def unit_px(app_w: float, app_h: float) -> float:
    """レイアウトの基準の長さ u[CSS px] = 画面の短辺の1/100(横向きなら高さ)。JSの `--u` と同じ。"""
    return min(app_w, app_h) / 100.0


def normalize_rot(value: Any) -> int:
    """?ocrrot= の値。90 / 270(= -90)以外は 0(縦向きのまま)。JSの nkRot と同じ。"""
    try:
        n = float(value)
    except (TypeError, ValueError):
        return 0
    if n == 90:
        return 90
    if n in (270, -90):
        return 270
    return 0


def normalize_size(value: Any) -> Optional[str]:
    """?size= の値を区分へ。未知・空は None(自動判定にする)。"""
    if value is None:
        return None
    return SIZE_ALIASES.get(str(value).strip().lower())


def classify(short_css: float, override: Any = None) -> str:
    """CSS px の短辺から区分を決める。override(?size=)があればそれを優先。
    ≥1000: 12.9 / ≥800: 11 / ≥700: small(9.7・10.2より小さい機種)/ それ未満: phone"""
    forced = normalize_size(override)
    if forced:
        return forced
    s = float(short_css)
    if s >= 1000:
        return "12.9"
    if s >= 800:
        return "11"
    if s >= 700:
        return "small"
    return "phone"


def css_ppi(short_css: float, is_ipad: bool = True, ppi_override: Optional[float] = None, forced: bool = False) -> float:
    """CSS px 1インチあたりの数。?ppi= があれば優先。iPad以外(PCのブラウザで見た場合)は 96。
    ただし ?size= で機種を指定したとき(forced)は、その機種(iPad)として 132 を使う。"""
    if ppi_override is not None and ppi_override > 0:
        return float(ppi_override)
    if not is_ipad and not forced:
        return 96.0
    if 740 <= short_css <= 750:
        return CSS_PPI_MINI
    return CSS_PPI_FULLSIZE


def px_per_mm(ppi: float, calib: float = 1.0) -> float:
    """1mm が何 CSS px か。calib は画面ごとの補正(?calib=。カードを当てて合わせる。0.85〜1.15)。"""
    return ppi / MM_PER_INCH * calib


# --- OCRテスト画面(模擬POS)の設計値。元は scripts/raspi_display/ocr_test/mock_pos_tablet_original.html ---
# 実物の明電舎タブレット(7インチ級)の画面。縦横比 3:4.2 を保って対角 7 インチ → 103.35 x 144.68 mm
MOCK_SCREEN_MM = (103.35, 144.68)
MOCK_ROWS = 7
# 元ページの数字のフォントサイズは `7.2vw`(画面の幅の7.2%。ラッパーが画面の幅=ビューポート幅にして表示していた)。
MOCK_DIGIT_FONT_RATIO = 0.072
# 数字の高さ ≒ フォントサイズ × 0.698(IBM Plex Mono の数字の高さ)。フォントが違っても数字の高さをこれに合わせる
MOCK_DIGIT_HEIGHT_EM = 0.698
MOCK_ROW_MM = MOCK_SCREEN_MM[1] / MOCK_ROWS


def mock_digit_font_mm() -> float:
    """数字のフォントサイズ[mm]。画面の幅 × 7.2% = 約 7.44 mm。"""
    return MOCK_SCREEN_MM[0] * MOCK_DIGIT_FONT_RATIO


def mock_digit_height_mm() -> float:
    """数字(0〜9)の高さ[mm]。約 5.19 mm(1行 約 20.7 mm の 25%)。"""
    return mock_digit_font_mm() * MOCK_DIGIT_HEIGHT_EM


def ocr_geometry(viewport_w: float, viewport_h: float, ppi: float = CSS_PPI_FULLSIZE, calib: float = 1.0,
                 fit: Optional[float] = None, margin_px: float = 0.0, rot: int = 0,
                 margin_y_px: Optional[float] = None, shrink: bool = False) -> Dict[str, Any]:
    """模擬POSの描画寸法(CSS px)。JSの nkOcrGeometry と同じ式。

    fit=None(既定・実寸): 7インチ実物と同じ実寸で描く。カメラの位置・ROI設定が本物のタブレットのままで使える。
    fit=0.0〜: 使える大きさ(画面-余白)の何割まで拡大/縮小して描くか(カメラを離して撮るときなど。実寸ではなくなる)。
    shrink=True: 実寸が入らないときだけ、入る大きさまで縮める(実寸ではなくなる。画面は知らせる)。
    rot=90/270: 横向きの端末として回して置く(外形の縦横が入れ替わる。数字の高さは変わらない)。
    margin_px / margin_y_px: 左右 / 上下の余白(セーフエリア)。margin_y_px 省略時は margin_px と同じ。
    戻り値: screen_w/screen_h(模擬画面の外形の幅・高さ px)、scale(実寸に対する倍率)、digit_px(数字の高さ px)、
            frac_w/frac_h(iPad画面に占める割合)、fits(はみ出さないか)、shrunk(入らず縮めたか)。
    """
    ppmm = px_per_mm(ppi, calib)
    my = margin_px if margin_y_px is None else margin_y_px
    rot = normalize_rot(rot)
    sideways = rot in (90, 270)
    w = MOCK_SCREEN_MM[0] * ppmm
    h = MOCK_SCREEN_MM[1] * ppmm
    if sideways:
        w, h = h, w
    scale = 1.0
    avail_w = max(0.0, viewport_w - 2 * margin_px)
    avail_h = max(0.0, viewport_h - 2 * my)
    if fit is not None and fit > 0:
        scale = min(avail_w * fit / w, avail_h * fit / h) if w and h else 1.0
    elif shrink and (w > avail_w + 0.5 or h > avail_h + 0.5):
        scale = min(avail_w / w, avail_h / h) if w and h else 1.0
    w *= scale
    h *= scale
    fits = w <= avail_w + 0.5 and h <= avail_h + 0.5
    return {
        "ppmm": ppmm * scale, "screen_w": w, "screen_h": h, "scale": scale,
        "digit_px": mock_digit_height_mm() * ppmm * scale,
        "digit_mm": mock_digit_height_mm() * scale,
        "frac_w": w / viewport_w if viewport_w else 0.0,
        "frac_h": h / viewport_h if viewport_h else 0.0,
        "fits": fits, "rot": rot if sideways else 0,
        "shrunk": scale < 1.0 and not (fit is not None and fit > 0),
    }


def landscape_ocr_report(keys: Tuple[str, ...] = ("12.9", "11", "11air"), usable_height_loss_px: float = 0.0) -> Dict[str, Dict[str, Any]]:
    """横向きの各画面での、模擬POS(縦向き実寸)の余白の計算(docs 7章の表の根拠)。
    usable_height_loss_px = Safariのアドレスバーなどで減る高さ(ホーム画面アプリなら 0〜セーフエリア)。"""
    out: Dict[str, Dict[str, Any]] = {}
    for k in keys:
        vw, vh = LANDSCAPE_CSS[k]
        vh = vh - usable_height_loss_px
        g = ocr_geometry(vw, vh)
        out[k] = {"viewport": (vw, vh), "mock_px": (g["screen_w"], g["screen_h"]), "height_frac": g["frac_h"],
                  "margin_top_bottom_px": (vh - g["screen_h"]) / 2.0, "margin_left_right_px": (vw - g["screen_w"]) / 2.0,
                  "max_height_loss_px": vh - g["screen_h"], "fits": g["fits"]}
    return out
