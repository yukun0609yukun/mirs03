"""iPad表示(頭部ディスプレイを本体に付けるiPad(Safari)にする、2026-10-26。docs/ipad_display.md)のテスト。

ROS・ロボット・iPad・ラズパイには一切触れない(ROSが無ければスタブ、HTTPは127.0.0.1の空きポートの本物のハンドラ)。確認すること:
- 画面サイズ: 12.9 / 11 インチの判定、向き、?size= ?ppi= ?calib= の上書き。JSの式(node で実行)が display_sizes.py と一致する
- OCRテスト画面(模擬POS)の実寸: 12.9 と 11 で数字の高さ・画面の大きさが同じ実寸(mm)になる。式と数値
- 画面HTML: 全画面(ホーム画面追加)・セーフエリア・dvh・ポーリング・復帰時の再接続・外部通信なし・JSの文法・CSP
- モード: 4つ、保存、ROSの文字列/JSON、回答中は保留(ROS)/断る(手動)、保留が終わってから実行される
- 回答中の安全: 状態機械(回答中→保存→待機/放棄)、通知途絶えの放棄、最長300秒の上限とクールダウン、survey_monitor(SurveyGate)との接続
- 認証・権限: 最初に1回だけ、厨房サイトと同じアカウントでログイン(専用のセッションCookie、期限は再起動まで、3分の自動ログアウト無し、同時1端末制限無し、失敗はロック)。
  認証なしで許すのはログイン画面・ログイン・manifest/icon と、ループバックからの state(survey_monitor)だけ。TLの管理画面のログインとは別系統
- メニュー編集: 追加・名前の変更・削除・初期食数を menu_init(全置き換え)で送る。検証、inventory_manager での効き方、iPad側とTL側(display.menu_edit)の権限
- 入力の検証: 回答の型・選択肢・長さ・サイズ上限、保存ファイルの上限、Origin/Sec-Fetch-Site、回数制限
- 残数入力: 既存の残数管理(inventory_manager)に正しく反映される(実物のハンドラで検証)。パスワードはダミーを使う(実機のものはどこにも書かない)
- スケジュール(DisplayScheduleController)・一時的な上書きの復元・設定タブのカードが、同じ「モード」で動く
実行: python3 -I ros2_ws/src/nokosahen_core/test/test_ipad_display.py
"""

import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import types
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))


def _install_stubs():
    """ROSが無い環境用のスタブ(あれば本物を使う)。"""
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m

    class Msg:
        def __init__(self, *a, **k):
            self.data = None

    class _Any:
        def __init__(self, *a, **k):
            self.data = None

    class _AutoMod(types.ModuleType):
        def __getattr__(self, item):
            if item.startswith("__"):
                raise AttributeError(item)
            return _Any

    try:
        import geometry_msgs.msg  # noqa: F401
        import nav_msgs.msg  # noqa: F401
        import std_msgs.msg  # noqa: F401
    except Exception:  # noqa: BLE001
        mod("geometry_msgs")
        mod("geometry_msgs.msg", PoseStamped=Msg, Twist=Msg)
        mod("nav_msgs")
        mod("nav_msgs.msg", Odometry=Msg)
        mod("std_msgs")
        mod("std_msgs.msg", Bool=Msg, Float32MultiArray=Msg, String=Msg, Empty=Msg)
    try:
        import rclpy.node  # noqa: F401
        import rclpy.qos  # noqa: F401
    except Exception:  # noqa: BLE001
        for n in ("rclpy", "rclpy.node", "rclpy.qos"):
            sys.modules[n] = _AutoMod(n)


_install_stubs()

from nokosahen_core import accounts as acc  # noqa: E402
from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import display_hub as dh  # noqa: E402
from nokosahen_core import display_page as dp  # noqa: E402
from nokosahen_core import display_routes as dr  # noqa: E402
from nokosahen_core import display_sizes as ds  # noqa: E402
from nokosahen_core import display_survey as dsv  # noqa: E402
from nokosahen_core import ephemeral_overrides as eo  # noqa: E402
from nokosahen_core import roles  # noqa: E402
from nokosahen_core import schedule_display as sd  # noqa: E402
from nokosahen_core import schedule_plan as sp  # noqa: E402

TMP = Path(tempfile.mkdtemp(prefix="ipad_display_"))
NODE = shutil.which("node")
OP_PASSWORD = "operator-pass-123"
OP_HASH = acc.hash_password(OP_PASSWORD, n=2 ** 10)
DISPLAY_PW = "dummy-display-pw-0000"      # テスト用のダミー。実機のパスワードではない(リポジトリにも無い)


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, s):
        self.t += s


def new_hub(tmp_name, clock=None, **kw):
    d = TMP / tmp_name
    d.mkdir(parents=True, exist_ok=True)
    clock = clock or Clock()
    store = dsv.SurveyStore(str(d / "survey"))
    hub = dh.DisplayHub(mode_file=str(d / "mode"), survey_store=store, clock=clock, wall=clock, **kw)
    return hub, clock, d


def good_answer(rid=None, **over):
    rec = {"response_id": rid or "resp-0001-abcdef", "submitted_at": "2026-10-26T01:02:03.000Z", "survey_version": "r2",
           "mode": "robot", "condition": "停止 + 音声", "location": "食堂入口", "duration_sec": 12,
           "answers": {"impression": 4, "noticed_by": "動いていたから", "approach_feel": ["楽しい", "気にならない"],
                       "want_to_move": "やや思った", "visitor_type": "大人"}}
    rec.update(over)
    return rec


# ---------------------------------------------------------------------------
# 画面サイズ
# ---------------------------------------------------------------------------
class SizesTest(unittest.TestCase):
    def test_ipad_pixel_density_and_active_area(self):
        # 12.9: 2048x2732 @264ppi → 197.0 x 262.9mm(本体外形 215x281 ではない)
        w, h = ds.active_area_mm("12.9")
        self.assertAlmostEqual(w, 197.0, delta=0.2)
        self.assertAlmostEqual(h, 262.9, delta=0.2)
        w, h = ds.active_area_mm("11")
        self.assertAlmostEqual(w, 160.5, delta=0.2)
        self.assertAlmostEqual(h, 229.8, delta=0.2)
        # CSS px の寸法は 12.9 も 11 も 132px/inch。CSS 1024px = 197.0mm、834px = 160.5mm と一致する
        self.assertAlmostEqual(1024 / ds.CSS_PPI_FULLSIZE * 25.4, 197.0, delta=0.2)
        self.assertAlmostEqual(834 / ds.CSS_PPI_FULLSIZE * 25.4, 160.5, delta=0.2)

    def test_classify_by_css_short_side(self):
        self.assertEqual(ds.classify(1024), "12.9")
        self.assertEqual(ds.classify(1032), "12.9")      # iPad Pro 13
        self.assertEqual(ds.classify(834), "11")
        self.assertEqual(ds.classify(820), "11")         # iPad Air 11 / 10.9
        self.assertEqual(ds.classify(810), "11")         # 10.2
        self.assertEqual(ds.classify(768), "small")
        self.assertEqual(ds.classify(744), "small")
        self.assertEqual(ds.classify(390), "phone")

    def test_override_wins_and_unknown_value_is_ignored(self):
        self.assertEqual(ds.classify(834, "12.9"), "12.9")
        self.assertEqual(ds.classify(1024, "11"), "11")
        self.assertEqual(ds.classify(1024, "10.9"), "11")
        self.assertEqual(ds.classify(1024, "mini"), "small")
        self.assertEqual(ds.classify(834, "bogus"), "11")     # 不明な値は自動判定のまま
        self.assertIsNone(ds.normalize_size("zzz"))

    def test_ppi_defaults(self):
        self.assertEqual(ds.css_ppi(1024), 132.0)
        self.assertEqual(ds.css_ppi(834), 132.0)
        self.assertEqual(ds.css_ppi(744), 163.0)           # iPad mini は 326ppi の半分
        self.assertEqual(ds.css_ppi(1080, is_ipad=False), 96.0)       # PCのブラウザ
        self.assertEqual(ds.css_ppi(1080, is_ipad=False, forced=True), 132.0)   # ?size= を付けたとき
        self.assertEqual(ds.css_ppi(834, ppi_override=100), 100.0)

    def test_mock_physical_size_is_identical_on_both_ipads(self):
        a = ds.ocr_geometry(1024, 1366)      # 12.9 縦
        b = ds.ocr_geometry(834, 1194)       # 11 縦
        # 模擬POSの実寸(px)・数字の高さ(mm / px)は同じ(=カメラから見た大きさが同じ)。画面に占める割合だけが違う
        self.assertAlmostEqual(a["screen_w"], b["screen_w"], places=6)
        self.assertAlmostEqual(a["screen_h"], b["screen_h"], places=6)
        self.assertAlmostEqual(a["digit_mm"], b["digit_mm"], places=9)
        self.assertAlmostEqual(a["digit_mm"], 5.19, delta=0.02)          # 数字の高さ ≒ 5.19mm
        self.assertAlmostEqual(a["screen_w"], 537.1, delta=0.2)          # 103.35mm × 5.1969px/mm
        self.assertAlmostEqual(a["screen_h"], 751.9, delta=0.2)          # 144.68mm
        self.assertTrue(a["fits"] and b["fits"])
        self.assertAlmostEqual(a["frac_w"], 0.524, delta=0.002)
        self.assertAlmostEqual(b["frac_w"], 0.644, delta=0.002)
        self.assertAlmostEqual(a["frac_h"], 0.550, delta=0.002)
        self.assertAlmostEqual(b["frac_h"], 0.630, delta=0.002)
        self.assertAlmostEqual(ds.mock_digit_font_mm(), 7.44, delta=0.01)
        self.assertAlmostEqual(ds.MOCK_ROW_MM, 20.67, delta=0.01)

    def test_mock_fits_in_landscape_and_fit_option(self):
        land = ds.ocr_geometry(1194, 834)    # 11 横: 高さ 834 に 752 が入る
        self.assertTrue(land["fits"])
        small = ds.ocr_geometry(500, 500)    # 入らない画面では fits=False
        self.assertFalse(small["fits"])
        fit = ds.ocr_geometry(834, 1194, fit=0.9)     # 画面いっぱいに拡大(実寸ではない)
        self.assertGreater(fit["scale"], 1.0)
        self.assertLessEqual(fit["screen_w"], 834 * 0.9 + 0.5)
        self.assertLessEqual(fit["screen_h"], 1194 * 0.9 + 0.5)

    def test_landscape_is_the_baseline_area_and_unit(self):
        # 横向き: 幅と高さが入れ替わる。12.9横 262.9x197.0mm、11横 229.8x160.5mm。縦横比はどちらもほぼ4:3
        w, h = ds.active_area_mm("12.9", landscape=True)
        self.assertAlmostEqual(w, 262.9, delta=0.2)
        self.assertAlmostEqual(h, 197.0, delta=0.2)
        w, h = ds.active_area_mm("11", landscape=True)
        self.assertAlmostEqual(w, 229.8, delta=0.2)
        self.assertAlmostEqual(h, 160.5, delta=0.2)
        self.assertEqual(ds.LANDSCAPE_CSS["12.9"], (1366, 1024))
        self.assertEqual(ds.LANDSCAPE_CSS["11"], (1194, 834))
        self.assertEqual(ds.LANDSCAPE_CSS["11air"], (1180, 820))
        for k in ("12.9", "11", "11air"):
            vw, vh = ds.LANDSCAPE_CSS[k]
            self.assertAlmostEqual(vw / vh, 1.35, delta=0.1, msg=k)      # ほぼ 4:3
        # 基準の長さ u = 短辺(横向きは高さ)の1/100。12.9横 10.24px / 11横 8.34px / Air 8.20px
        self.assertAlmostEqual(ds.unit_px(1366, 1024), 10.24, places=2)
        self.assertAlmostEqual(ds.unit_px(1194, 834), 8.34, places=2)
        self.assertAlmostEqual(ds.unit_px(1180, 820), 8.20, places=2)
        self.assertAlmostEqual(ds.unit_px(1024, 1366), 10.24, places=2)    # 縦向きでも同じ(短辺)

    def test_landscape_mock_stays_upright_real_size_and_fits(self):
        # 判断: 縦向きのまま・実寸のまま、横向きの中央に置く。12.9も11も実寸(537x752px)・数字5.19mm
        rep = ds.landscape_ocr_report()
        for k, r in rep.items():
            self.assertAlmostEqual(r["mock_px"][0], 537.1, delta=0.2, msg=k)
            self.assertAlmostEqual(r["mock_px"][1], 751.9, delta=0.2, msg=k)
            self.assertTrue(r["fits"], k)
        self.assertAlmostEqual(rep["12.9"]["height_frac"], 0.734, delta=0.002)
        self.assertAlmostEqual(rep["11"]["height_frac"], 0.902, delta=0.002)
        self.assertAlmostEqual(rep["11air"]["height_frac"], 0.917, delta=0.002)
        self.assertAlmostEqual(rep["11"]["margin_top_bottom_px"], 41.0, delta=0.2)      # (834-752)/2
        self.assertAlmostEqual(rep["11air"]["margin_top_bottom_px"], 34.0, delta=0.2)   # (820-752)/2
        # 数字の高さは12.9横も11横も同じ(カメラから見た大きさが同じ)
        a, b = ds.ocr_geometry(1366, 1024), ds.ocr_geometry(1194, 834)
        self.assertAlmostEqual(a["digit_mm"], b["digit_mm"], places=9)
        self.assertEqual((a["rot"], b["rot"]), (0, 0))
        # 11インチ横で、Safariのバー等で高さが 750 に減ると実寸は入らない(752 > 750)。ホーム画面アプリなら 834 から上下のセーフエリア(約24px×2)を引いても入る
        self.assertFalse(ds.ocr_geometry(1194, 750)["fits"])
        self.assertTrue(ds.ocr_geometry(1194, 834, margin_y_px=24)["fits"])
        self.assertTrue(ds.ocr_geometry(1180, 820, margin_y_px=24)["fits"])      # 772 >= 752

    def test_shrink_and_rotate_options(self):
        g = ds.ocr_geometry(1194, 750, shrink=True)          # 入らないとき: 入る大きさまで縮める
        self.assertTrue(g["fits"] and g["shrunk"])
        self.assertAlmostEqual(g["scale"], 750 / 751.9, delta=0.002)
        self.assertAlmostEqual(g["digit_mm"], ds.mock_digit_height_mm() * g["scale"], places=9)
        ok = ds.ocr_geometry(1194, 834, shrink=True)         # 入るなら実寸のまま
        self.assertEqual(ok["scale"], 1.0)
        self.assertFalse(ok["shrunk"])
        r = ds.ocr_geometry(1194, 750, rot=90)               # 横向き端末として回す: 外形 752x537 で入る。数字の高さは同じ
        self.assertTrue(r["fits"])
        self.assertAlmostEqual(r["screen_w"], 751.9, delta=0.2)
        self.assertAlmostEqual(r["screen_h"], 537.1, delta=0.2)
        self.assertAlmostEqual(r["digit_mm"], ds.mock_digit_height_mm(), places=9)
        self.assertEqual(ds.normalize_rot(90), 90)
        self.assertEqual(ds.normalize_rot("-90"), 270)
        self.assertEqual(ds.normalize_rot(270), 270)
        self.assertEqual(ds.normalize_rot("45"), 0)
        self.assertEqual(ds.normalize_rot(None), 0)

    def test_css_mm_unit_is_not_used_for_the_mock(self):
        html = dp.build_display_html().decode("utf-8")
        mock_css = html[html.index("#v-ocr{"):html.index("#ocrnote{")]
        self.assertNotRegex(mock_css, r"[0-9.]mm[ ;)}]", "CSSの mm 単位はiPadの実寸と合わない(96dpi換算)。--mm(px/mm)を使うこと")
        self.assertIn("var(--mm)", mock_css)


def _extract_nksize():
    html = dp.build_display_html().decode("utf-8")
    a = html.index("/*NKSIZE-BEGIN*/")
    b = html.index("/*NKSIZE-END*/")
    return html[a:b]


@unittest.skipUnless(NODE, "nodeが無いのでJSの式の照合は省略")
class JsParityTest(unittest.TestCase):
    """画面HTMLの NKSIZE ブロック(JS)を node で実行して、display_sizes.py と同じ結果になることを確かめる。"""

    @classmethod
    def setUpClass(cls):
        cls.js = TMP / "nksize.js"
        cls.js.write_text(_extract_nksize() + """
var cases = JSON.parse(process.argv[2]);
console.log(JSON.stringify(cases.map(function (c) {
  var s = nkSize(c);
  var g = nkOcrGeometry(c.innerW, c.innerH, s.ppmm, s.fit, s.rot, c.mx || 0, c.my || 0, !!c.shrink);
  return {size: s, geo: g};
})));
""", encoding="utf-8")

    def run_cases(self, cases):
        out = subprocess.run([NODE, str(self.js), json.dumps(cases)], capture_output=True, text=True, timeout=30)
        self.assertEqual(out.returncode, 0, out.stderr)
        return json.loads(out.stdout)

    IPAD_UA = "Mozilla/5.0 (iPad; CPU OS 17_0 like Mac OS X) AppleWebKit/605.1.15 Mobile/15E148"
    MAC_UA = "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/605.1.15 Version/17.0 Safari/605.1.15"

    def test_classification_matches_python(self):
        cases = [
            {"innerW": 1024, "innerH": 1366, "screenW": 1024, "screenH": 1366, "search": "", "ua": self.IPAD_UA},
            {"innerW": 1366, "innerH": 1024, "screenW": 1024, "screenH": 1366, "search": "", "ua": self.IPAD_UA},
            {"innerW": 834, "innerH": 1194, "screenW": 834, "screenH": 1194, "search": "", "ua": self.IPAD_UA},
            {"innerW": 820, "innerH": 1180, "screenW": 820, "screenH": 1180, "search": "", "ua": self.IPAD_UA},
            {"innerW": 1180, "innerH": 820, "screenW": 820, "screenH": 1180, "search": "", "ua": self.IPAD_UA},
            {"innerW": 744, "innerH": 1133, "screenW": 744, "screenH": 1133, "search": "", "ua": self.IPAD_UA},
            # iPadOS が Mac を名乗る(デスクトップ表示)場合も、タッチ点があればiPad
            {"innerW": 834, "innerH": 1194, "screenW": 834, "screenH": 1194, "search": "", "ua": self.MAC_UA, "touchPoints": 5},
            # PCのブラウザ(窓の大きさで判定。ppiは96)
            {"innerW": 900, "innerH": 700, "screenW": 1920, "screenH": 1080, "search": "", "ua": self.MAC_UA, "touchPoints": 0},
            # 上書き
            {"innerW": 834, "innerH": 1194, "screenW": 834, "screenH": 1194, "search": "?size=12.9", "ua": self.IPAD_UA},
            {"innerW": 1024, "innerH": 1366, "screenW": 1024, "screenH": 1366, "search": "?size=11&calib=1.05", "ua": self.IPAD_UA},
            {"innerW": 900, "innerH": 700, "screenW": 1920, "screenH": 1080, "search": "?size=11", "ua": self.MAC_UA, "touchPoints": 0},
            {"innerW": 834, "innerH": 1194, "screenW": 834, "screenH": 1194, "search": "?ppi=163&ocrfit=0.9", "ua": self.IPAD_UA},
            {"innerW": 834, "innerH": 1194, "screenW": 834, "screenH": 1194, "search": "?size=zzz&calib=99", "ua": self.IPAD_UA},
            # 横向き(本命): 12.9横 / 11横 / Air横 / バーで高さが減った状態(実寸が入らない→縮める)/ 回転して置く / セーフエリア
            {"innerW": 1194, "innerH": 834, "screenW": 834, "screenH": 1194, "search": "?debug=1", "ua": self.IPAD_UA},
            {"innerW": 1194, "innerH": 750, "screenW": 834, "screenH": 1194, "search": "", "ua": self.IPAD_UA, "shrink": True},
            {"innerW": 1194, "innerH": 750, "screenW": 834, "screenH": 1194, "search": "?ocrrot=90", "ua": self.IPAD_UA, "shrink": True},
            {"innerW": 1180, "innerH": 820, "screenW": 820, "screenH": 1180, "search": "?ocrrot=-90", "ua": self.IPAD_UA, "mx": 0, "my": 24, "shrink": True},
            {"innerW": 1366, "innerH": 1024, "screenW": 1024, "screenH": 1366, "search": "?ocrrot=45", "ua": self.IPAD_UA, "mx": 20, "my": 24},
            {"innerW": 1366, "innerH": 1024, "screenW": 1024, "screenH": 1366, "search": "?ocrfit=0.8", "ua": self.IPAD_UA, "mx": 10, "my": 24},
        ]
        res = self.run_cases(cases)
        for c, r in zip(cases, res):
            s, g = r["size"], r["geo"]
            q = dict(p.split("=") for p in c["search"].lstrip("?").split("&") if p)
            is_ipad = "iPad" in c["ua"] or ("Macintosh" in c["ua"] and c.get("touchPoints", 0) > 1)
            device_short = min(c["screenW"], c["screenH"]) if is_ipad else min(c["innerW"], c["innerH"])
            key = ds.classify(device_short, q.get("size"))
            self.assertEqual(s["key"], key, c)
            ppi_q = float(q["ppi"]) if "ppi" in q else None
            ppi = ds.css_ppi(device_short, is_ipad=is_ipad, ppi_override=ppi_q, forced=ds.normalize_size(q.get("size")) is not None)
            self.assertAlmostEqual(s["ppi"], ppi, places=6, msg=c)
            calib = min(2.0, max(0.5, float(q["calib"]))) if "calib" in q else 1.0
            self.assertAlmostEqual(s["ppmm"], ds.px_per_mm(ppi, calib), places=9, msg=c)
            fit = float(q["ocrfit"]) if "ocrfit" in q else None
            rot = ds.normalize_rot(q.get("ocrrot"))
            self.assertEqual(s["rot"], rot, c)
            py = ds.ocr_geometry(c["innerW"], c["innerH"], ppi, calib, fit, margin_px=c.get("mx", 0), rot=rot,
                                 margin_y_px=c.get("my", 0), shrink=bool(c.get("shrink")))
            for k_js, k_py in (("screen_w", "screen_w"), ("screen_h", "screen_h"), ("digit_mm", "digit_mm"),
                               ("digit_px", "digit_px"), ("frac_w", "frac_w"), ("frac_h", "frac_h"), ("ppmm", "ppmm"), ("scale", "scale")):
                self.assertAlmostEqual(g[k_js], py[k_py], places=6, msg=(c, k_js))
            self.assertEqual(g["fits"], py["fits"], c)
            self.assertEqual(g["shrunk"], py["shrunk"], c)
            self.assertEqual(s["orient"], "landscape" if c["innerW"] > c["innerH"] else "portrait", c)

    def test_known_devices_get_expected_values(self):
        res = self.run_cases([
            {"innerW": 1024, "innerH": 1366, "screenW": 1024, "screenH": 1366, "search": "", "ua": self.IPAD_UA},
            {"innerW": 834, "innerH": 1194, "screenW": 834, "screenH": 1194, "search": "", "ua": self.IPAD_UA},
        ])
        big, small = res
        self.assertEqual(big["size"]["orient"], "portrait")
        self.assertEqual((big["size"]["key"], big["size"]["layout"]), ("12.9", "xl"))
        self.assertEqual((small["size"]["key"], small["size"]["layout"]), ("11", "l"))
        self.assertAlmostEqual(big["geo"]["digit_mm"], small["geo"]["digit_mm"], places=9)
        self.assertAlmostEqual(big["geo"]["screen_w"], small["geo"]["screen_w"], places=6)

    def test_landscape_devices_are_classified_by_the_device_short_side(self):
        # 横向き(本命): 窓の高さがSafariのバーで減っても、端末の短辺(screen)で 12.9 / 11 を判定する
        res = self.run_cases([
            {"innerW": 1366, "innerH": 1024, "screenW": 1366, "screenH": 1024, "search": "", "ua": self.IPAD_UA},
            {"innerW": 1366, "innerH": 930, "screenW": 1366, "screenH": 1024, "search": "", "ua": self.IPAD_UA},
            {"innerW": 1194, "innerH": 834, "screenW": 1194, "screenH": 834, "search": "", "ua": self.IPAD_UA},
            {"innerW": 1194, "innerH": 760, "screenW": 1194, "screenH": 834, "search": "", "ua": self.IPAD_UA},
            {"innerW": 1180, "innerH": 820, "screenW": 1180, "screenH": 820, "search": "", "ua": self.IPAD_UA},
        ])
        self.assertEqual([r["size"]["key"] for r in res], ["12.9", "12.9", "11", "11", "11"])
        self.assertTrue(all(r["size"]["orient"] == "landscape" for r in res))
        self.assertTrue(all(abs(r["size"]["ppi"] - 132.0) < 1e-9 for r in res))
        self.assertEqual(res[0]["size"]["debug"], False)


# ---------------------------------------------------------------------------
# 画面HTML
# ---------------------------------------------------------------------------
class PageTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.html = dp.build_display_html().decode("utf-8")

    def test_fullscreen_homescreen_and_safe_area(self):
        h = self.html
        self.assertIn('name="apple-mobile-web-app-capable" content="yes"', h)
        self.assertIn("viewport-fit=cover", h)
        self.assertIn("user-scalable=no", h)
        self.assertIn("env(safe-area-inset-top", h)
        self.assertIn("env(safe-area-inset-bottom", h)
        self.assertIn("100dvh", h)
        self.assertIn('rel="manifest"', h)
        self.assertIn("apple-touch-icon", h)

    def test_landscape_is_the_default_and_portrait_is_a_fallback(self):
        h = self.html
        self.assertIn('<html lang="ja" data-size="l" data-orient="landscape">', h)      # 読み込み直後(JS前)から横向き
        self.assertIn("html[data-orient=portrait]", h)                                  # 縦向きは上書き(フォールバック)として持つ
        self.assertNotIn("html[data-orient=landscape]", h)                              # 横向きは既定のCSS(派生ではない)
        # 縦向きの薄い案内(押せない・OCRテスト中は出さない)
        self.assertIn('id="rotate"', h)
        self.assertIn("横向きにしてください", h)
        self.assertIn("pointer-events:none", h[h.index("#rotate{"):h.index("#rotate{") + 400])
        self.assertIn('body[data-mode=ocrtest] #rotate{display:none}', h)
        # 向きの判定は JS が data-orient に入れる(回転で再計算)
        self.assertIn('root.setAttribute("data-orient", size.orient)', h)
        self.assertIn("orientationchange", h)

    def test_customer_landscape_is_status_left_and_menu_right(self):
        h = self.html
        self.assertIn('grid-template-areas:"bar cards" "status cards" "promote cards" "announce cards"', h)
        self.assertIn("grid-template-columns:minmax(0,5fr) minmax(0,7fr)", h)      # 左=今の状態、右=残数リスト
        for lay in ("h", "v", "v2"):
            self.assertIn(f"#cards[data-lay={lay}]", h)
        # 品数ごとの列数・文字の大きさ(最大12品は3列x4行で収まる)と、はみ出すときの自動縮小
        self.assertIn('n <= 12 ? [3, "v2"', h)
        self.assertIn("function fitCards()", h)
        self.assertIn("c.scrollHeight > c.clientHeight", h)
        self.assertIn("--k", h)
        # 縦向きのフォールバックは従来の compact を使う
        self.assertIn("html[data-orient=portrait] #cards.compact .card", h)

    def test_survey_landscape_has_question_left_and_big_tiles_right(self):
        h = self.html
        self.assertIn('grid-template-areas:"q a" "q e"', h)                         # 左=設問、右=選択肢
        self.assertIn(".qwrap.is-scale", h)                                         # 顔の5段階は横いっぱい
        self.assertIn('w.className = "qwrap" + (q.type === "scale" ? " is-scale" : "")', h)
        self.assertIn('grid-template-areas:"tag go" "h go" "note go"', h)           # 待機画面: 左=題、右=大きな開始ボタン
        self.assertIn("--tap:max(56px", h)
        # 回答中の通知・保存の挙動は変えていない(5秒ごとの beat、end、保存後の同期)
        self.assertIn('hook("beat")', h)
        self.assertIn("}, 5000);", h)
        self.assertIn("/display/api/survey/answer", h)
        self.assertIn("/display/api/survey/session", h)

    def test_inventory_landscape_has_list_left_and_keypad_right(self):
        h = self.html
        for el in ('id="ibody"', 'id="ilist"', 'id="ipane"', 'id="ipKeys"', 'id="ipSide"', 'id="ipVal"'):
            self.assertIn(el, h)
        self.assertIn('grid-template-areas:"pn pn" "pv pv" "keys side" "ph ph"', h)
        self.assertIn("function paneKey(", h)
        self.assertIn("html[data-orient=portrait] #ipane{display:none}", h)           # 縦は各行で直接入力(従来)
        self.assertIn('/display/api/inventory', h)
        self.assertIn('place-items:start center', h)                                  # ログインのダイアログは上寄せ(キーボードに隠れない)

    def test_ocr_mock_is_centered_upright_with_wrapper_and_debug_in_all_modes(self):
        h = self.html
        self.assertIn('id="mockwrap"', h)
        self.assertIn('id="ocrnote"', h)
        self.assertIn('id="dbg"', h)                       # ?debug=1 はどのモードでも出る
        self.assertIn("nkOcrGeometry(vw, vh, size.ppmm, size.fit, size.rot, mx, my, true)", h)
        self.assertIn("function nkRot(", h)

    def test_layout_is_driven_by_variables_not_fixed_pixels(self):
        h = self.html
        self.assertIn("--u:calc(min(var(--app-w),var(--app-h)) / 100)", h)    # 短辺の1/100(横向きは高さ)
        for name in ("data-orient", "data-size", "--cols", "--numsz", "--nmsz"):
            self.assertIn(name, h)
        self.assertIn('landscape', h)
        # 触る部分の最小サイズ(指で押せる大きさ)
        self.assertIn("--tap:max(56px", h)

    def test_all_four_modes_and_polling(self):
        h = self.html
        for m in ("customer", "survey", "inventory", "ocrtest"):
            self.assertIn(f"body[data-mode={m}]", h)
        self.assertIn('/display/api/state', h)
        self.assertIn("visibilitychange", h)       # バックグラウンドから戻ったら再接続
        self.assertIn("pageshow", h)
        self.assertIn("wakeLock", h)                # スリープ防止(対応しているとき)
        self.assertIn("location.reload()", h)       # プログラムが更新されたら自動で読み込み直す
        self.assertIn("window.NK_BUILD", h)

    def test_works_offline_no_external_resources(self):
        # 外部のフォント・スクリプト・画像を読まない(ロボットのLANにはインターネットが無い)
        self.assertNotRegex(self.html, r"(src|href)=\"https?://")
        self.assertNotIn("fonts.googleapis", self.html)
        self.assertNotIn("@import", self.html)

    def test_survey_definition_is_embedded_and_matches_original_questions(self):
        for q in dsv.SURVEY_DEF["questions"]:
            self.assertIn(q["id"], self.html)
            self.assertIn(q["label"], self.html)
        # 今の版(r2)は kousensai_survey.html の移植。旧版(r1)は mirs_robot_survey.html の移植
        for fname, d in (("kousensai_survey.html", dsv.SURVEY_DEF), ("mirs_robot_survey.html", dsv.SURVEY_DEF_R1)):
            orig = (PKG.parents[2] / "scripts" / "raspi_display" / "survey" / fname)
            if orig.exists():
                src = orig.read_text(encoding="utf-8")
                for q in d["questions"]:
                    self.assertIn(q["id"], src)
                    # 2026-11-08: want_to_move の質問文だけ、ユーザーの依頼で元のHTMLから変えた(idは同じ。元のHTMLは無修正で保管)
                    if q["id"] == "want_to_move" and fname == "kousensai_survey.html":
                        self.assertIn("実際に移動したいと思いましたか？", src)
                        continue
                    self.assertIn(q["label"], src)
                    for o in q.get("options", []):
                        self.assertIn(o, src)
                for c in d["conditions"]:
                    self.assertIn(c, src)
                self.assertIn('version: "%s"' % d["version"], src)

    def test_survey_r2_matches_kousensai_order_and_wording(self):
        d = dsv.SURVEY_DEF
        self.assertEqual(d["version"], "r2")
        self.assertEqual([q["id"] for q in d["questions"]],
                         ["impression", "noticed_by", "approach_feel", "want_to_move", "visitor_type"])
        self.assertEqual([q["type"] for q in d["questions"]], ["scale", "choice", "multi", "choice", "choice"])
        self.assertEqual(d["idleNote"], "5つの質問 ・ 約35秒")
        self.assertEqual(d["idleTitle"], "ロボットについて\n教えてください")
        self.assertEqual(d["thanks"], "ありがとうございました！")
        self.assertEqual(d["doneReturnSec"], 10)
        self.assertEqual(d["idleTimeoutSec"], 30)
        self.assertNotIn("たまたま", json.dumps(d, ensure_ascii=False))
        self.assertTrue(all(q.get("required") for q in d["questions"]))
        self.assertEqual(dsv.SURVEY_DEFS["r1"]["version"], "r1")
        # 画面: 5段階の色、「えらんでね」、完了画面の件数、残り問数
        for s in ('data-v', 'えらんでね', 'doneCount', 'idleCount', 'bubble', 'ラスト！', '回答者です'):
            self.assertIn(s, self.html)

    def test_survey_json_cannot_break_out_of_script(self):
        j = dp._survey_json()
        self.assertNotIn("</", j)

    def test_server_text_is_never_put_in_innerhtml(self):
        # 残数・メニュー名・状態の文言はサーバ由来。textContent で入れる(innerHTMLに入れるのは固定の図形・固定の雛形だけ)
        for m in re.finditer(r"innerHTML\s*=\s*([^;\n]+)", self.html):
            rhs = m.group(1)
            self.assertNotRegex(rhs, r"\b(it|st|c|d|name|r)\.(name|title|detail|announce|remaining|label)\b", rhs)

    @unittest.skipUnless(NODE, "nodeが無いのでJSの文法検査は省略")
    def test_inline_scripts_are_valid_javascript(self):
        scripts = re.findall(r"<script>(.*?)</script>", self.html, re.S)
        self.assertGreaterEqual(len(scripts), 3)
        for i, s in enumerate(scripts):
            f = TMP / f"page_script_{i}.js"
            f.write_text(s, encoding="utf-8")
            out = subprocess.run([NODE, "--check", str(f)], capture_output=True, text=True, timeout=30)
            self.assertEqual(out.returncode, 0, out.stderr)

    def test_build_id_changes_with_content(self):
        self.assertEqual(len(dp.build_id()), 10)
        self.assertIn(dp.build_id(), self.html)

    def test_no_in_page_login_and_401_goes_back_to_the_login_page(self):
        """ログインは別のHTML。本体の画面は、状態取得が401(ロボットの再起動でセッションが消えた)になると読み込み直してログイン画面に戻る。"""
        h = self.html
        for gone in ("loginSheet", "lgUser", "refreshWho", "loginBtn", "whoText", "/api/login"):
            self.assertNotIn(gone, h, gone)
        self.assertIn("r.status === 401", h)
        self.assertIn("function authLost()", h)
        self.assertIn("nk_re401", h)                           # 読み込み直しの繰り返し防止

    def test_menu_edit_ui_is_in_the_inventory_view_and_keyboard_safe(self):
        h = self.html
        for el in ('id="editToggle"', 'id="epane"', 'id="eName"', 'id="eInit"', 'id="eGo"', 'id="eDel"', 'id="delSheet"', 'id="delYes"', 'id="delNo"'):
            self.assertIn(el, h)
        self.assertIn("/display/api/menu/edit", h)
        for op in ('"add"', '"rename"', '"delete"', '"set_initial"'):
            self.assertIn(op, h)
        # 削除は確認ダイアログの2段階(「削除…」→「削除する」)。確定を押すまで送らない(入力イベントでは fetch しない)
        self.assertIn('id="delYes"', h)
        self.assertIn("align-content:start", h[h.index("#epane{"):h.index("#epane{") + 400])      # 入力欄とボタンは上寄せ(キーボードに隠れない)
        self.assertIn("font-size:max(18px", h[h.index("#epane input{"):h.index("#epane input{") + 200])   # 16px未満だとiOSが拡大する
        self.assertIn('autocapitalize="off"', h)
        self.assertIn("-webkit-user-select:text", h[h.index("#epane input{"):h.index("#epane input{") + 400])
        self.assertIn("html[data-orient=portrait] #epane", h)                                      # 縦向きのフォールバック
        # メニュー名はサーバ由来: textContent / value で入れる(innerHTMLに入れない。既存の test_server_text_is_never_put_in_innerhtml も通る)
        self.assertNotIn("eName\").innerHTML", h)
        self.assertIn('$("eName").value = name === null ? "" : name', h)


class LoginPageTest(unittest.TestCase):
    """未ログインに返すログイン画面: ログインフォームだけ。横向き最適化・キーボードで隠れない・外部通信なし。"""

    @classmethod
    def setUpClass(cls):
        cls.html = dp.build_display_login_html().decode("utf-8")

    def test_contains_only_the_login_form(self):
        h = self.html
        for el in ('id="f"', 'id="u"', 'id="p"', 'type="password"', 'autocomplete="current-password"', 'autocomplete="username"'):
            self.assertIn(el, h)
        self.assertIn("/display/api/login", h)
        for secret in ("NK_SURVEY", "adminPin", "questions", "/display/api/state", "/display/api/survey", "/display/api/inventory", "/display/api/menu"):
            self.assertNotIn(secret, h, secret)
        self.assertNotIn("ADMIN_ACCOUNT", h)                      # アカウント名も画面に書かない
        self.assertNotRegex(h, r"(src|href)=\"https?://")

    def test_landscape_first_and_keyboard_safe(self):
        h = self.html
        self.assertIn('<html lang="ja" data-orient="landscape">', h)
        self.assertIn("grid-template-columns:minmax(0,5fr) minmax(0,6fr)", h)       # 横向き: 左=説明、右=フォーム
        self.assertIn("align-items:start", h)                                        # フォームは上寄せ(下半分はキーボードが隠す)
        self.assertIn("font-size:max(18px", h)                                       # 入力欄は大きく。16px未満だとiOSが拡大する
        self.assertIn("100dvh", h)
        self.assertIn("env(safe-area-inset-top", h)
        self.assertIn("html[data-orient=portrait]", h)
        self.assertIn("viewport-fit=cover", h)
        self.assertIn("apple-mobile-web-app-capable", h)
        self.assertIn('rel="manifest"', h)

    def test_wording_mentions_kitchen_account_and_first_time_only(self):
        self.assertIn("最初に1回だけ", self.html)
        self.assertIn("厨房サイトと同じ", self.html)

    @unittest.skipUnless(NODE, "nodeが無いのでJSの文法検査は省略")
    def test_inline_script_is_valid_javascript(self):
        scripts = re.findall(r"<script>(.*?)</script>", self.html, re.S)
        self.assertEqual(len(scripts), 1)
        f = TMP / "login_script.js"
        f.write_text(scripts[0], encoding="utf-8")
        out = subprocess.run([NODE, "--check", str(f)], capture_output=True, text=True, timeout=30)
        self.assertEqual(out.returncode, 0, out.stderr)


# ---------------------------------------------------------------------------
# 回答の検証・保存
# ---------------------------------------------------------------------------
class SurveyStoreTest(unittest.TestCase):
    def test_valid_answer_is_accepted_and_normalized(self):
        rec, err = dsv.validate_response(good_answer(extra_field="x"))
        self.assertEqual(err, "")
        self.assertNotIn("extra_field", rec)           # 余計な項目は保存しない
        self.assertEqual(rec["sheet"], "robot")
        self.assertEqual(rec["answers"]["approach_feel"], ["楽しい", "気にならない"])

    def test_rejects_bad_inputs(self):
        bad = [
            good_answer(response_id="x"),                        # 短すぎ
            good_answer(response_id="a" * 65),
            good_answer(response_id="id with space!"),
            good_answer(submitted_at="<script>"),
            good_answer(survey_version="r0"),
            good_answer(condition="でたらめ"),
            good_answer(condition="x" * 100),
            good_answer(location="a\nb"),
            good_answer(location="x" * 81),
            good_answer(duration_sec=-1),
            good_answer(duration_sec=True),
            good_answer(duration_sec="12"),
            good_answer(answers=[]),
            good_answer(answers={"unknown": 1}),
            good_answer(answers={"noticed_by": "選択肢にない"}),
            good_answer(answers={"noticed_by": ["動いていたから"]}),
            good_answer(answers={"impression": 6}),
            good_answer(answers={"noticed_by": "たまたま"}),                   # r2 では選べない(r1 の選択肢)
            good_answer(answers={"impression": 4, "noticed_by": "動いていたから", "approach_feel": ["楽しい"],
                                 "want_to_move": "思った", "visitor_type": "大人"}),   # 選択肢にない
            good_answer(answers={"impression": 4, "noticed_by": "動いていたから", "approach_feel": ["楽しい"],
                                 "visitor_type": "大人"}),                          # 必須(want_to_move)が無い
            good_answer(answers={"impression": 4, "noticed_by": "動いていたから", "approach_feel": [],
                                 "want_to_move": "やや思った", "visitor_type": "大人"}),   # 複数選択が空
            good_answer(answers={}),
            good_answer(answers={"impression": "3"}),
            good_answer(answers={"impression": True}),
            good_answer(answers={"approach_feel": ["楽しい", "気にならない", "少しこわい"]}),    # maxSelect=2
            good_answer(answers={"approach_feel": ["楽しい", "楽しい"]}),
            good_answer(answers={"approach_feel": "楽しい"}),
        ]
        for b in bad:
            rec, err = dsv.validate_response(b)
            self.assertIsNone(rec, b)
            self.assertTrue(err)
        self.assertIsNone(dsv.validate_response("string")[0])
        self.assertIsNone(dsv.validate_response(None)[0])

    def test_r2_each_field_is_validated_on_its_own(self):
        """完全な回答の1項目だけを悪くして、その項目の検証で落ちることを確かめる(他の欠けで落ちたのではない)。"""
        full = good_answer()["answers"]
        self.assertEqual(dsv.validate_response(good_answer())[1], "")
        bads = [("impression", 0), ("impression", 6), ("impression", 3.0), ("impression", None), ("impression", [3]),
                ("noticed_by", "たまたま"), ("noticed_by", ["動いていたから"]), ("noticed_by", ""), ("noticed_by", 1),
                ("approach_feel", ["楽しい", "気にならない", "少しこわい"]), ("approach_feel", ["楽しい", "楽しい"]),
                ("approach_feel", "楽しい"), ("approach_feel", ["たのしい"]), ("approach_feel", []),
                ("want_to_move", "思った"), ("want_to_move", 2), ("want_to_move", ["とても思った"]),
                ("visitor_type", "先生"), ("visitor_type", "x" * 5000)]
        for k, v in bads:
            a = dict(full)
            a[k] = v
            rec, err = dsv.validate_response(good_answer(answers=a))
            self.assertIsNone(rec, (k, v))
            self.assertIn(k, err, (k, v, err))
        for k in full:                                    # どれか1問でも欠けたら拒否(5問とも必須)
            a = dict(full)
            del a[k]
            rec, err = dsv.validate_response(good_answer(answers=a))
            self.assertIsNone(rec, k)
            self.assertIn(k, err)

    def test_old_version_r1_still_accepted_and_versions_kept_apart(self):
        """版が変わっても、古い画面に溜まっていた r1 の回答は保存できる。保存と CSV で版を区別できる。"""
        r1 = good_answer(rid="resp-r1-000001", survey_version="r1",
                         answers={"noticed_by": "たまたま", "impression": 3, "approach_feel": ["楽しい"], "visitor_type": "大人"})
        rec1, err = dsv.validate_response(r1)
        self.assertEqual(err, "")
        self.assertEqual(rec1["survey_version"], "r1")
        # r1 でも検証は厳しいまま
        self.assertIsNone(dsv.validate_response(good_answer(survey_version="r1", answers={"want_to_move": "とても思った"}))[0])
        self.assertIsNone(dsv.validate_response(good_answer(survey_version="r1", answers={"impression": 9}))[0])
        self.assertIsNone(dsv.validate_response(good_answer(survey_version="r3"))[0])
        rec2, err = dsv.validate_response(good_answer(rid="resp-r2-000001"))
        self.assertEqual(err, "")
        self.assertEqual(rec2["survey_version"], "r2")
        d = TMP / "store_mixed"
        st = dsv.SurveyStore(str(d))
        self.assertTrue(st.save(rec1)[0])
        self.assertTrue(st.save(rec2)[0])
        data, _ = st.csv_bytes()
        rows = list(__import__("csv").reader(data.decode("utf-8-sig").splitlines()))
        head = rows[0]
        self.assertIn("survey_version", head)
        self.assertEqual({r[head.index("survey_version")] for r in rows[1:]}, {"r1", "r2"})
        self.assertIn("want_to_move", head)                                      # 設問id列は全版の和集合
        r1row = next(r for r in rows[1:] if r[head.index("survey_version")] == "r1")
        self.assertEqual(r1row[head.index("want_to_move")], "")                  # r1 に無い設問は空
        self.assertEqual(r1row[head.index("noticed_by")], "たまたま")

    def test_record_and_csv_format_matches_raspi_receiver(self):
        """ラズパイ版 receiver.py と同じ記録形式(固定列・設問列・複数選択は ' / '・数式対策)。"""
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "raspi_receiver", PKG.parents[2] / "scripts" / "raspi_display" / "survey" / "receiver.py")
        rcv = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(rcv)
        self.assertEqual(rcv.FIXED, dsv.FIXED)
        recs = []
        for i in range(3):
            rec, _ = dsv.validate_response(good_answer(rid=f"resp-{i:04d}-abcdef", location="=1+1" if i == 1 else "食堂"))
            rec["received_at"] = "2026-10-26T01:02:03"
            recs.append(rec)
        tmp = TMP / "csvparity"
        tmp.mkdir(exist_ok=True)
        rcv.CSV_PATH = str(tmp / "r.csv")
        rcv.write_csv(recs)
        theirs = Path(rcv.CSV_PATH).read_bytes()
        mine = dsv.records_to_csv(recs)
        self.assertEqual(mine.replace(b"\r\n", b"\n"), theirs.replace(b"\r\n", b"\n"))
        self.assertIn("'=1+1".encode(), mine)

    def test_store_saves_dedups_and_survives_corrupt_lines(self):
        d = TMP / "store1"
        st = dsv.SurveyStore(str(d))
        rec, _ = dsv.validate_response(good_answer())
        self.assertEqual(st.save(dict(rec)), (True, False, None))
        self.assertEqual(st.save(dict(rec)), (True, True, None))      # 同じIDの再送は二重保存しない
        self.assertEqual(st.count(), 1)
        lines = (d / "responses.jsonl").read_text(encoding="utf-8").splitlines()
        self.assertEqual(len(lines), 1)
        self.assertIn("received_at", json.loads(lines[0]))
        # 電源断で途中までの行が残っていても、次の回答は新しい行に書かれ、全体が読める
        with open(d / "responses.jsonl", "a", encoding="utf-8") as f:
            f.write('{"response_id": "broken')
        st2 = dsv.SurveyStore(str(d))
        self.assertEqual(st2.count(), 1)
        self.assertEqual(st2.bad_lines, 1)
        rec2, _ = dsv.validate_response(good_answer(rid="resp-0002-abcdef"))
        self.assertTrue(st2.save(rec2)[0])
        self.assertEqual(dsv.SurveyStore(str(d)).count(), 2)
        data, err = st2.csv_bytes()
        self.assertTrue(data.startswith("﻿".encode()))
        self.assertEqual(data.decode("utf-8").count("\n"), 3)          # 見出し + 2件

    def test_file_size_and_record_caps(self):
        d = TMP / "store_cap"
        st = dsv.SurveyStore(str(d), max_bytes=600, max_records=50)
        saved = 0
        err = None
        for i in range(10):
            rec, _ = dsv.validate_response(good_answer(rid=f"resp-{i:04d}-abcdef"))
            ok, _, e = st.save(rec)
            if ok:
                saved += 1
            else:
                err = e
                break
        self.assertLess(saved, 10)
        self.assertIn("上限", err or "")
        self.assertLess((d / "responses.jsonl").stat().st_size, 600 + 700)
        st = dsv.SurveyStore(str(TMP / "store_cap2"), max_records=2)
        for i in range(2):
            self.assertTrue(st.save(dsv.validate_response(good_answer(rid=f"resp-{i:04d}-abcdef"))[0])[0])
        self.assertFalse(st.save(dsv.validate_response(good_answer(rid="resp-9999-abcdef"))[0])[0])

    def test_unwritable_directory_returns_failure_not_exception(self):
        f = TMP / "afile"
        f.write_text("x")
        st = dsv.SurveyStore(str(f / "sub"))      # ディレクトリを作れない
        ok, dup, err = st.save(dsv.validate_response(good_answer())[0])
        self.assertFalse(ok)
        self.assertTrue(err)


# ---------------------------------------------------------------------------
# モードと回答中の状態機械
# ---------------------------------------------------------------------------
class HubModeTest(unittest.TestCase):
    def test_default_is_customer_and_mode_is_persisted(self):
        hub, clock, d = new_hub("hm1")
        self.assertEqual(hub.mode, "customer")
        r = hub.request_mode("inventory", "ros")
        self.assertTrue(r["ok"] and r["changed"])
        self.assertEqual((d / "mode").read_text().strip(), "inventory")
        hub2 = dh.DisplayHub(mode_file=str(d / "mode"), survey_store=hub.store, clock=clock, wall=clock)
        self.assertEqual(hub2.mode, "inventory")       # 再起動しても残る
        (d / "mode").write_text("garbage\n")
        self.assertEqual(dh.DisplayHub(mode_file=str(d / "mode"), clock=clock, wall=clock).mode, "customer")

    def test_ros_string_and_json_forms(self):
        self.assertEqual(dh.normalize_mode("survey"), "survey")
        self.assertEqual(dh.normalize_mode("  OCRTEST \n"), "ocrtest")
        self.assertEqual(dh.normalize_mode('{"mode": "inventory"}'), "inventory")
        self.assertEqual(dh.normalize_mode({"mode": "customer"}), "customer")
        for bad in ("", "kiosk", "{", '{"mode": 3}', "[1]", None, 5, "survey;rm -rf"):
            self.assertIsNone(dh.normalize_mode(bad), bad)
        hub, _, _ = new_hub("hm2")
        r = hub.request_mode("bogus")
        self.assertFalse(r["ok"])
        self.assertEqual(hub.mode, "customer")

    def test_same_mode_is_noop_and_clears_pending(self):
        hub, clock, _ = new_hub("hm3")
        hub.request_mode("survey", "admin", defer=False)
        hub.session_event("start")
        self.assertTrue(hub.request_mode("customer", "ros")["deferred"])
        self.assertEqual(hub.pending["mode"], "customer")
        r = hub.request_mode("survey", "ros")                   # 回答中の画面を保ちたい要求 → 保留を取り消す
        self.assertTrue(r["ok"] and not r["changed"])
        self.assertIsNone(hub.pending)

    def test_ros_request_is_deferred_while_answering_and_applied_after(self):
        hub, clock, _ = new_hub("hm4")
        hub.request_mode("survey", "admin", defer=False)
        hub.session_event("start")
        r = hub.request_mode("customer", "ros", defer=True)
        self.assertTrue(r["ok"] and r["deferred"] and not r["changed"])
        self.assertEqual(hub.mode, "survey")                    # 回答中は切り替えない
        self.assertEqual(hub.public_state()["pending_mode"], "customer")
        clock.advance(10)
        hub.session_event("beat")
        hub.tick()
        self.assertEqual(hub.mode, "survey")                    # まだ回答中
        code, _ = hub.submit_response(good_answer())
        self.assertEqual(code, 200)
        hub.tick()
        self.assertEqual(hub.mode, "survey")                    # 保存直後(「ありがとう」を出している3秒)は待つ
        clock.advance(dh.SAVED_SHOW_S + 0.1)
        hub.tick()
        self.assertEqual(hub.mode, "customer")                  # 終わったら切り替わる
        self.assertIsNone(hub.pending)
        self.assertEqual(hub.session.phase, "idle")

    def test_manual_switch_is_refused_while_answering(self):
        hub, clock, _ = new_hub("hm5")
        hub.request_mode("survey", "admin", defer=False)
        hub.session_event("start")
        r = hub.request_mode("customer", "admin", defer=False)
        self.assertFalse(r["ok"])
        self.assertEqual(r["code"], 409)
        self.assertEqual(hub.mode, "survey")
        self.assertTrue(hub.session.active())                   # その人の回答は消えない
        hub.session_event("end")
        self.assertTrue(hub.request_mode("customer", "admin", defer=False)["ok"])

    def test_pending_applies_after_abandon_timeout(self):
        hub, clock, _ = new_hub("hm6")
        hub.request_mode("survey", "admin", defer=False)
        hub.session_event("start")
        hub.request_mode("inventory", "ros")
        clock.advance(dh.SESSION_ABANDON_S + 1)
        hub.tick()
        self.assertEqual(hub.mode, "inventory")

    def test_leaving_survey_resets_session(self):
        hub, clock, _ = new_hub("hm7")
        hub.request_mode("survey", "admin", defer=False)
        hub.session_event("start")
        hub.session_event("end")
        hub.request_mode("customer", "admin", defer=False)
        st = hub.public_state()
        self.assertFalse(st["active"])
        self.assertEqual(st["session_state"], "idle")


class SessionSafetyTest(unittest.TestCase):
    """ラズパイ版 receiver.py の Session と同じ保証(回答中は active、保存されるまで続く、放棄の上限)。"""

    def setUp(self):
        self.hub, self.clock, _ = new_hub("ss_" + self.id().split(".")[-1])
        self.hub.request_mode("survey", "admin", defer=False)

    def test_active_only_while_answering_until_saved(self):
        self.assertFalse(self.hub.public_state()["active"])
        self.hub.session_event("start")
        st = self.hub.public_state()
        self.assertTrue(st["active"])
        self.assertEqual(st["session_state"], "answering")
        for _ in range(10):                                   # 回答中は通知が続く限り、いつまでも回答中
            self.clock.advance(5)
            self.assertTrue(self.hub.session_event("beat")["active"])
        seq0 = self.hub.public_state()["saved_seq"]
        code, body = self.hub.submit_response(good_answer())
        self.assertEqual(code, 200)
        st = self.hub.public_state()
        self.assertFalse(st["active"])
        self.assertEqual(st["session_state"], "saved")
        self.assertEqual(st["saved_seq"], seq0 + 1)
        self.assertEqual(st["count"], 1)
        self.clock.advance(dh.SAVED_SHOW_S + 0.1)
        self.assertEqual(self.hub.public_state()["session_state"], "idle")

    def test_failed_save_keeps_answering(self):
        """保存に失敗したら saved にしない(answering のまま=ロボットは止まり続ける。放棄の上限で解除)。"""
        self.hub.session_event("start")
        self.hub.store.data_dir = str(TMP / "afile" / "x")
        self.hub.store.jsonl = str(TMP / "afile" / "x" / "r.jsonl")
        (TMP / "afile").write_text("x")
        code, body = self.hub.submit_response(good_answer(rid="resp-fail-000001"))
        self.assertEqual(code, 500)
        self.assertFalse(body["ok"])
        st = self.hub.public_state()
        self.assertTrue(st["active"])
        self.assertEqual(st["session_state"], "answering")
        self.assertEqual(st["saved_seq"], 0)

    def test_abandon_when_notifications_stop(self):
        self.hub.session_event("start")
        self.clock.advance(dh.SESSION_ABANDON_S - 1)
        self.assertTrue(self.hub.public_state()["active"])
        self.clock.advance(2)
        st = self.hub.public_state()
        self.assertFalse(st["active"])                       # 永久には止めない
        self.assertEqual(st["session_state"], "abandoned")
        self.clock.advance(dh.ABANDONED_SHOW_S + 0.1)
        self.assertEqual(self.hub.public_state()["session_state"], "idle")

    def test_late_beat_does_not_revive_abandoned_session(self):
        self.hub.session_event("start")
        self.clock.advance(dh.SESSION_ABANDON_S + 1)
        r = self.hub.session_event("beat")
        self.assertFalse(r["active"])
        self.assertFalse(self.hub.public_state()["active"])

    def test_end_abandons(self):
        self.hub.session_event("start")
        self.hub.session_event("end")
        self.assertEqual(self.hub.public_state()["session_state"], "abandoned")
        self.assertFalse(self.hub.public_state()["active"])

    def test_replay_does_not_count_as_completion(self):
        self.hub.session_event("start")
        code, _ = self.hub.submit_response(good_answer(rid="resp-old-000001"), replay=True)
        self.assertEqual(code, 200)
        st = self.hub.public_state()
        self.assertTrue(st["active"])                         # 過去の未送信分の再送は、今の回答者の完了ではない
        self.assertEqual(st["saved_seq"], 0)
        self.assertEqual(st["count"], 1)

    def test_duplicate_does_not_complete_session(self):
        self.hub.submit_response(good_answer(rid="resp-dup-000001"), replay=True)
        self.hub.session_event("start")
        code, body = self.hub.submit_response(good_answer(rid="resp-dup-000001"))
        self.assertTrue(body.get("duplicate"))
        self.assertTrue(self.hub.public_state()["active"])

    def test_max_answer_duration_and_cooldown(self):
        """認証なしの入口から「回答中」を送り続けられても、ロボットを止め続けられない。"""
        t0 = self.clock.t
        self.hub.session_event("start")
        while self.hub.public_state()["active"]:
            self.assertLess(self.clock.t - t0, dh.SESSION_MAX_ANSWER_S + 10, "最長で打ち切られない")
            self.clock.advance(5)
            self.hub.session_event("beat")
            self.hub.session_event("start")                  # startを重ねても最長の起点は進まない
        self.assertAlmostEqual(self.clock.t - t0, dh.SESSION_MAX_ANSWER_S, delta=6)
        st = self.hub.public_state()
        self.assertFalse(st["active"])
        self.assertEqual(st["session_state"], "abandoned")
        r = self.hub.session_event("start")                  # 打ち切り直後のクールダウン中は受け付けない
        self.assertFalse(r["active"])
        self.clock.advance(dh.MAX_ANSWER_COOLDOWN_S + 1)
        self.assertTrue(self.hub.session_event("start")["active"])    # 普通に回答する人はその後また使える

    def test_session_events_only_in_survey_mode(self):
        self.hub.request_mode("customer", "admin", defer=False)
        r = self.hub.session_event("start")
        self.assertFalse(r["ok"])
        self.assertEqual(r["code"], 409)
        self.assertFalse(self.hub.public_state()["active"])
        self.assertFalse(self.hub.session_event("bogus")["ok"])

    def test_condition_is_validated_and_persisted(self):
        self.assertFalse(self.hub.session_event("start", "でたらめ")["ok"])
        self.assertTrue(self.hub.session_event("start", "移動のみ")["ok"])
        self.assertEqual(self.hub.public_state()["condition"], "移動のみ")
        self.assertEqual(self.hub.store.read_condition(), "移動のみ")


class SurveyMonitorLinkTest(unittest.TestCase):
    """survey_monitor(ロボットを止める橋渡し)が、新しい /display/api/state をそのまま読めて、同じ保証になる。"""

    @classmethod
    def setUpClass(cls):
        try:
            from nokosahen_core import survey_monitor as sm
        except ImportError as exc:   # 全件実行で、先に別のテストが不完全なrclpyスタブを入れたとき(既知の干渉)。単独実行では通る
            raise unittest.SkipTest(f"survey_monitorを読み込めない(rclpyスタブの干渉): {exc}")
        cls.sm = sm

    def serve_hub(self, hub):
        handler = dc.make_dashboard_handler(lambda: b"<html></html>", lambda: {}, FakeTeleop(),
                                            auth_username="TL", auth_password="tl-pass", enforce_single_controller=False,
                                            display_hub=hub)
        srv = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        self.addCleanup(lambda: (srv.shutdown(), srv.server_close()))
        return f"http://127.0.0.1:{srv.server_address[1]}"

    def test_gate_follows_answering_and_save(self):
        hub, clock, _ = new_hub("sml1")
        hub.request_mode("survey", "admin", defer=False)
        base = self.serve_hub(hub)
        gate = self.sm.SurveyGate(debounce_sec=1.5, unreachable_hold_sec=20, answering_unreachable_hold_sec=60, post_save_hold_sec=1.5)
        t = 100.0

        def feed():
            s = self.sm.fetch_survey_state(base + "/display/api", 2.0)
            self.assertTrue(s["ok"], s)
            gate.feed(s)
            return s

        feed()
        self.assertFalse(gate.tick(t))                             # 誰も触っていない
        hub.session_event("start")
        s = feed()
        self.assertEqual(s["session_state"], "answering")
        self.assertTrue(gate.tick(t))                              # 回答中 → ロボットを止める
        t += 30
        hub.session_event("beat")
        feed()
        self.assertTrue(gate.tick(t))                              # 長く回答していても止め続ける
        hub.submit_response(good_answer())
        s = feed()
        self.assertEqual(s["session_state"], "saved")
        self.assertTrue(gate.tick(t + 0.5))                        # 保存完了の直後もすぐは動かさない
        self.assertFalse(gate.tick(t + 10))                        # その後に解除

    def test_unreachable_is_safe_side_then_releases(self):
        hub, clock, _ = new_hub("sml2")
        hub.request_mode("survey", "admin", defer=False)
        base = self.serve_hub(hub)
        gate = self.sm.SurveyGate()
        hub.session_event("start")
        gate.feed(self.sm.fetch_survey_state(base + "/display/api", 2.0))
        self.assertTrue(gate.tick(10.0))
        gate.feed({"ok": False, "error": "timeout"})                # ダッシュボードが応答しない
        self.assertTrue(gate.tick(40.0))                           # 回答中だったので60秒までは止めておく(安全側)
        self.assertFalse(gate.tick(75.0))                          # 永久には止めない

    def test_customer_mode_never_holds(self):
        hub, clock, _ = new_hub("sml3")
        base = self.serve_hub(hub)
        gate = self.sm.SurveyGate()
        gate.feed(self.sm.fetch_survey_state(base + "/display/api", 2.0))
        self.assertFalse(gate.tick(1.0))

    def test_state_contract_fields(self):
        hub, _, _ = new_hub("sml4")
        st = hub.public_state()
        for k in ("mode", "active", "count", "last_activity_age_s", "condition", "updated", "session_state", "saved_seq"):
            self.assertIn(k, st)
        self.assertIsInstance(st["active"], bool)
        self.assertIsInstance(st["saved_seq"], int)


# ---------------------------------------------------------------------------
# HTTP: 認証・権限(本物の make_dashboard_handler)
# ---------------------------------------------------------------------------
class FakeTeleop:
    soft_estop_engaged = False

    def __init__(self):
        self.drives = []
        self.cmd_vel_pub = types.SimpleNamespace(publish=lambda m: None)

    def handle_drive(self, *a, **k):
        self.drives.append((a, k))
        return True

    def handle_soft_estop(self, engaged):
        self.soft_estop_engaged = engaged

    def status(self):
        return {}


def http(method, url, body=None, cookie=None, headers=None, raw=None):
    h = {"Content-Type": "application/json"}
    if cookie:
        h["Cookie"] = cookie
    h.update(headers or {})
    data = raw if raw is not None else (None if body is None else json.dumps(body).encode())
    req = urllib.request.Request(url, data=data, method=method, headers=h)
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            b = r.read()
            return r.status, b, r.headers
    except urllib.error.HTTPError as e:
        try:
            return e.code, e.read(), e.headers
        finally:
            e.close()


def jhttp(*a, **k):
    code, b, h = http(*a, **k)
    try:
        return code, json.loads(b.decode("utf-8") or "{}"), h
    except ValueError:
        return code, {"_raw": b[:200]}, h


class HttpBase(unittest.TestCase):
    def start(self, perms=None, auth=True, single=False):
        name = "http_" + self.id().split(".")[-1]
        self.menu_state = {"items": [
            {"name": "Aランチ", "initial": 100, "remaining": 60, "sold": 40},
            {"name": "Bランチ", "initial": 80, "remaining": 80, "sold": 0},
        ]}
        self.sent = []
        self.hub, self.clock, self.dir = new_hub(
            name, menu_state_provider=lambda: self.menu_state,
            inventory_sink=lambda topic, payload: self.sent.append((topic, payload)) or True,
            customer_provider=lambda: dh.build_customer_payload(self.menu_state, {"mode": "NEAR_PERSON_HOLD"}, 1.0, None))
        accounts_file = TMP / f"accounts_{name}.yaml"
        accounts_file.write_text(
            'accounts:\n  - username: "kari-op"\n    role: operator\n'
            f'    password_hash: "{OP_HASH}"\n    enabled: true\n'
            f'\noperator_permissions: {json.dumps(perms or [])}\n', encoding="utf-8")
        perm_file = TMP / f"perm_{name}.json"
        if perm_file.exists():
            perm_file.unlink()
        self.store = acc.AccountStore(accounts_path=str(accounts_file), permissions_path=str(perm_file))
        journal = TMP / f"journal_{name}.json"
        if journal.exists():
            journal.unlink()
        self.admin = dh.LocalDisplayAdmin(self.hub)
        self.teleop = FakeTeleop()
        handler = dc.make_dashboard_handler(
            lambda: b"<html>dash</html>", lambda: {}, self.teleop, survey_admin=self.admin,
            auth_username="TL" if auth else "", auth_password="tl-pass" if auth else "",
            enforce_single_controller=single, account_store=self.store, ephemeral=eo.EphemeralOverrides(str(journal)),
            display_hub=self.hub, display_auth_username="ADMIN_ACCOUNT", display_auth_password=DISPLAY_PW)
        self.srv = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        self.base = f"http://127.0.0.1:{self.srv.server_address[1]}"
        self.addCleanup(lambda: (self.srv.shutdown(), self.srv.server_close()))

    def login(self, user, pw):
        code, d, h = jhttp("POST", self.base + "/api/login", {"username": user, "password": pw})
        self.assertEqual(code, 200, (user, d))
        return h.get("Set-Cookie", "").split(";")[0]

    def dlogin(self):
        """iPad表示のログイン(厨房サイトと同じアカウント。テストではダミーのパスワード)。専用のCookieを返す。"""
        code, d, h = jhttp("POST", self.base + "/display/api/login", {"username": "ADMIN_ACCOUNT", "password": DISPLAY_PW})
        self.assertEqual(code, 200, d)
        return h.get("Set-Cookie", "").split(";")[0]


class FakeHandler:
    """display_routes に直接渡す偽のハンドラ(外部IPからの接続など、127.0.0.1のテストサーバでは作れない状況の検証用)。"""

    def __init__(self, ip="192.168.1.50", headers=None, body=b""):
        import io
        self.client_address = (ip, 50000)
        self.headers = dict(headers or {})
        self.headers.setdefault("Content-Length", str(len(body)))
        self.headers = _CIDict(self.headers)
        self.rfile = io.BytesIO(body)
        self.wfile = io.BytesIO()
        self.status = None
        self.sent = {}

    def send_response(self, status):
        self.status = int(status)

    def send_header(self, k, v):
        self.sent[k] = v

    def end_headers(self):
        pass

    @property
    def json(self):
        return json.loads(self.wfile.getvalue().decode("utf-8") or "{}")


class _CIDict(dict):
    def get(self, k, d=None):
        for kk, v in self.items():
            if kk.lower() == k.lower():
                return v
        return d


class PublicSurfaceTest(HttpBase):
    def test_login_page_is_public_and_contains_no_state(self):
        self.start()
        code, b, h = http("GET", self.base + "/display")
        self.assertEqual(code, 200)
        self.assertIn("text/html", h["Content-Type"])
        self.assertEqual(h["Cache-Control"], "no-store")
        self.assertIn("default-src 'none'", h["Content-Security-Policy"])
        self.assertIn("frame-ancestors 'none'", h["Content-Security-Policy"])
        self.assertEqual(h["X-Content-Type-Options"], "nosniff")
        self.assertIn(b"apple-mobile-web-app-capable", b)
        self.assertEqual(b, dp.build_display_login_html())                 # 未ログインにはログイン画面だけ
        self.assertIn('id="f"', b.decode("utf-8"))
        self.assertEqual(http("GET", self.base + "/display/")[2]["Content-Type"], h["Content-Type"])
        # ログイン画面には、設問・管理用PIN・メニュー名・状態を入れない
        t = b.decode("utf-8")
        for secret in ("adminPin", "NK_SURVEY", "questions", "Aランチ", "/display/api/state", "/display/api/survey", "inventory"):
            self.assertNotIn(secret, t, secret)
        # 本体の画面は、ログイン済みにだけ返す
        ck = self.dlogin()
        code, b2, _ = http("GET", self.base + "/display", cookie=ck)
        self.assertEqual(b2, dp.build_display_html())
        self.assertNotEqual(b2, b)
        # 偽のクッキー・TL管理画面のクッキーでは本体の画面は出ない
        self.assertEqual(http("GET", self.base + "/display", cookie="nokosahen_display=forged")[1], b)
        tl = self.login("TL", "tl-pass")
        self.assertEqual(http("GET", self.base + "/display", cookie=tl)[1], b)

    def test_manifest_and_icon_are_public(self):
        self.start()
        self.assertEqual(http("GET", self.base + "/display/manifest.webmanifest")[0], 200)
        mf = json.loads(dr.MANIFEST.decode("utf-8"))
        self.assertEqual(mf["orientation"], "landscape")
        self.assertEqual(mf["display"], "fullscreen")
        self.assertEqual(mf["start_url"], "/display")
        self.assertEqual(http("GET", self.base + "/display/icon.png")[0], 200)

    def test_state_with_session_and_from_loopback(self):
        self.start()
        ck = self.dlogin()
        code, st, _ = jhttp("GET", self.base + "/display/api/state", cookie=ck)
        self.assertEqual(code, 200)
        self.assertEqual(st["mode"], "customer")
        self.assertEqual(st["build"], dp.build_id())
        self.assertEqual(st["customer"]["status"]["title"], "止まっています")
        self.assertEqual(st["customer"]["menu"]["items"][0]["remaining"], 60)
        # survey_monitor はCookieを持たない。同じPCの中(ループバック)からは、認証なしで読める(このテストのクライアントも127.0.0.1)
        code, st, _ = jhttp("GET", self.base + "/display/api/state")
        self.assertEqual(code, 200)
        self.assertEqual(st["mode"], "customer")

    def test_state_from_outside_needs_the_session(self):
        """ループバック以外(LAN上の端末)からは、ログイン済みのCookieが無い限り401。X-Forwarded-Forは信用しない。"""
        self.start()
        ctx = dc_ctx(self)
        for ip in ("203.0.113.20", "198.51.100.2", "8.8.8.8", "::ffff:192.168.0.9", "2001:db8::1"):
            hd = FakeHandler(ip)
            dr.handle_get(hd, "/display/api/state", "", ctx)
            self.assertEqual(hd.status, 401, ip)
            self.assertEqual(hd.json["error"], "auth_required")
        for hdrs in ({"X-Forwarded-For": "127.0.0.1"}, {"X-Real-IP": "127.0.0.1"}, {"Forwarded": "for=127.0.0.1"}, {"Host": "127.0.0.1"}):
            hd = FakeHandler("203.0.113.20", hdrs)
            dr.handle_get(hd, "/display/api/state", "", ctx)
            self.assertEqual(hd.status, 401, hdrs)
        for ip in ("127.0.0.1", "::1", "::ffff:127.0.0.1"):
            hd = FakeHandler(ip)
            dr.handle_get(hd, "/display/api/state", "", ctx)
            self.assertEqual(hd.status, 200, ip)
        # 外部IPでも、ログイン済みのCookieがあれば読める
        tok = ctx.auth.create()
        hd = FakeHandler("203.0.113.20", {"Cookie": f"nokosahen_display={tok}"})
        dr.handle_get(hd, "/display/api/state", "", ctx)
        self.assertEqual(hd.status, 200)
        self.assertTrue(dr.is_loopback("127.0.0.1") and not dr.is_loopback("127.0.0.2.evil") and not dr.is_loopback("192.168.1.1"))

    def test_loopback_does_not_open_the_writes(self):
        """ループバックの免除は state の読み取りだけ。書き込みは同じPCからでもログインが要る。"""
        self.start()
        for path, body in (("/display/api/survey/session", {"event": "start"}), ("/display/api/survey/answer", good_answer()),
                           ("/display/api/inventory", {"menu": "Aランチ", "remaining": 5}),
                           ("/display/api/menu/edit", {"op": "delete", "name": "Aランチ"})):
            self.assertEqual(jhttp("POST", self.base + path, body)[0], 401, path)
        self.assertEqual(self.sent, [])
        self.assertEqual(self.hub.store.count(), 0)

    def test_loopback_state_read_is_not_counted_as_an_ipad(self):
        self.start()
        jhttp("GET", self.base + "/display/api/state")                        # survey_monitor 相当
        self.assertEqual(self.hub.admin_state()["ipad"]["connected"], 0)
        jhttp("GET", self.base + "/display/api/state", cookie=self.dlogin())  # iPad
        self.assertEqual(self.hub.admin_state()["ipad"]["connected"], 1)

    def test_the_rest_of_dashboard_still_requires_login(self):
        self.start()
        for path in ("/api/survey/state", "/api/state", "/api/whoami", "/api/survey/responses.csv", "/api/menu/state"):
            self.assertEqual(http("GET", self.base + path)[0], 401, path)
        self.assertEqual(jhttp("POST", self.base + "/api/survey/mode", {"mode": "survey"})[0], 401)
        self.assertEqual(jhttp("POST", self.base + "/api/survey/ocr_mock", {"action": "reset"})[0], 401)
        self.assertEqual(jhttp("POST", self.base + "/api/menu/edit", {"op": "delete", "name": "Aランチ"})[0], 401)
        self.assertEqual(jhttp("POST", self.base + "/api/teleop", {"forward": 0.1})[0], 401)
        self.assertEqual(self.teleop.drives, [])

    def test_display_session_cannot_use_the_dashboard_and_vice_versa(self):
        """iPad用セッションは管理画面のAPIに使えない。管理画面(TL)のセッションはiPad用としては使えない(自動ログインにならない)。"""
        self.start()
        dk = self.dlogin()                       # nokosahen_display=...
        for path in ("/api/survey/state", "/api/state", "/api/whoami", "/api/menu/state"):
            self.assertEqual(http("GET", self.base + path, cookie=dk)[0], 401, path)
        for path, body in (("/api/survey/mode", {"mode": "survey"}), ("/api/menu/edit", {"op": "delete", "name": "Aランチ"}),
                           ("/api/teleop", {"forward": 0.1}), ("/api/soft_estop", {"engaged": False})):
            self.assertEqual(jhttp("POST", self.base + path, body, cookie=dk)[0], 401, path)
        self.assertEqual(self.hub.mode, "customer")
        self.assertEqual(self.sent, [])
        # 逆: TLのクッキーで /display の書き込み・外部からの state は通らない
        tl = self.login("TL", "tl-pass")
        self.assertEqual(jhttp("POST", self.base + "/display/api/inventory", {"menu": "Aランチ", "remaining": 5}, cookie=tl)[0], 401)
        self.assertEqual(jhttp("POST", self.base + "/display/api/menu/edit", {"op": "delete", "name": "Aランチ"}, cookie=tl)[0], 401)
        hd = FakeHandler("203.0.113.20", {"Cookie": tl})
        dr.handle_get(hd, "/display/api/state", "", dc_ctx(self))
        self.assertEqual(hd.status, 401)
        # guest も同じ(iPad側へは入れない)
        gk = self.login("guest", "")
        self.assertEqual(jhttp("POST", self.base + "/display/api/inventory", {"menu": "Aランチ", "remaining": 5}, cookie=gk)[0], 401)

    def test_unknown_display_paths_are_404_and_cannot_reach_other_apis(self):
        self.start()
        dk = self.dlogin()
        for path in ("/display/api/mode", "/display/api/ocr_mock", "/display/api/settings", "/display/../api/state",
                     "/display/api/survey/responses.csv", "/display/api", "/display/api/whoami", "/display/api/logout"):
            for ck in (None, dk):
                code, _, _ = http("GET", self.base + path, cookie=ck)
                self.assertEqual(code, 404, path)
                self.assertEqual(jhttp("POST", self.base + path, {"mode": "survey"}, cookie=ck)[0], 404, path)
        self.assertEqual(self.hub.mode, "customer")

    def test_session_writes_work_with_the_cookie_only_for_these_paths(self):
        """ログイン済みのiPadが書けるのは、アンケート回答の追加・回答中の通知・残数・メニュー編集だけ。モードは変わらない。"""
        self.start()
        ck = self.dlogin()
        self.hub.request_mode("survey", "admin", defer=False)
        code, d, _ = jhttp("POST", self.base + "/display/api/survey/session", {"event": "start"}, cookie=ck)
        self.assertEqual((code, d["ok"], d["active"]), (200, True, True))
        code, d, _ = jhttp("POST", self.base + "/display/api/survey/answer", good_answer(), cookie=ck)
        self.assertEqual((code, d["ok"]), (200, True))
        self.assertEqual(self.hub.store.count(), 1)
        # 余計な項目で他の状態(mode など)は変えられない
        code, d, _ = jhttp("POST", self.base + "/display/api/survey/session", {"event": "start", "mode": "customer"}, cookie=ck)
        self.assertEqual(code, 400)
        self.assertEqual(self.hub.mode, "survey")
        self.assertEqual(self.sent, [])

    def test_answer_validation_over_http(self):
        self.start()
        ck = self.dlogin()
        url = self.base + "/display/api/survey/answer"
        self.assertEqual(jhttp("POST", url, good_answer(survey_version="zz"), cookie=ck)[0], 400)
        self.assertEqual(jhttp("POST", url, good_answer(answers={"impression": 99}), cookie=ck)[0], 400)
        self.assertEqual(jhttp("POST", url, raw=b"{not json", cookie=ck)[0], 400)
        self.assertEqual(jhttp("POST", url, raw=b"[1,2]", cookie=ck)[0], 400)
        self.assertEqual(jhttp("POST", url, raw=b"\xff\xfe", cookie=ck)[0], 400)
        big = good_answer(location="x" * 9000)
        self.assertEqual(jhttp("POST", url, big, cookie=ck)[0], 413)
        # Content-Length が無い・不正
        self.assertIn(jhttp("POST", url, raw=None, headers={"Content-Length": "abc"}, cookie=ck)[0], (400, 411))
        self.assertEqual(self.hub.store.count(), 0)
        self.assertEqual(jhttp("POST", url, good_answer(), cookie=ck)[0], 200)
        self.assertEqual(jhttp("POST", url, good_answer(), cookie=ck)[0], 200)      # 同じ回答の再送は二重保存しない
        self.assertEqual(self.hub.store.count(), 1)

    def test_origin_and_fetch_site_checks(self):
        self.start()
        ck = self.dlogin()
        url = self.base + "/display/api/survey/answer"
        host = self.base.replace("http://", "")
        self.assertEqual(jhttp("POST", url, good_answer(), cookie=ck, headers={"Origin": "http://evil.example"})[0], 403)
        self.assertEqual(jhttp("POST", url, good_answer(), cookie=ck, headers={"Origin": "null"})[0], 403)
        self.assertEqual(jhttp("POST", url, good_answer(), cookie=ck, headers={"Sec-Fetch-Site": "cross-site"})[0], 403)
        self.assertEqual(jhttp("POST", url, good_answer(), cookie=ck, headers={"Sec-Fetch-Site": "same-site"})[0], 403)
        self.assertEqual(self.hub.store.count(), 0)
        ok = jhttp("POST", url, good_answer(), cookie=ck, headers={"Origin": "http://" + host, "Sec-Fetch-Site": "same-origin"})
        self.assertEqual(ok[0], 200)
        # 残数入力・メニュー編集・ログインも同じ検査
        bad = jhttp("POST", self.base + "/display/api/inventory", {"menu": "Aランチ", "remaining": 5}, cookie=ck,
                    headers={"Origin": "http://evil.example"})
        self.assertEqual(bad[0], 403)
        bad = jhttp("POST", self.base + "/display/api/menu/edit", {"op": "delete", "name": "Aランチ"}, cookie=ck,
                    headers={"Sec-Fetch-Site": "cross-site"})
        self.assertEqual(bad[0], 403)
        bad = jhttp("POST", self.base + "/display/api/login", {"username": "ADMIN_ACCOUNT", "password": DISPLAY_PW},
                    headers={"Origin": "http://evil.example"})
        self.assertEqual(bad[0], 403)
        self.assertEqual(self.sent, [])

    def test_origin_ok_unit(self):
        h = lambda **kw: types.SimpleNamespace(get=lambda k, d=None: kw.get(k.replace("-", "_"), d))
        self.assertTrue(dr.origin_ok(h(Host="a:8090")))
        self.assertTrue(dr.origin_ok(h(Host="a:8090", Origin="http://a:8090")))
        self.assertFalse(dr.origin_ok(h(Host="a:8090", Origin="http://a:8091")))
        self.assertFalse(dr.origin_ok(h(Host="a:8090", Origin="ftp://a:8090")))
        self.assertFalse(dr.origin_ok(h(Host="a:8090", Sec_Fetch_Site="cross-site")))
        self.assertTrue(dr.origin_ok(h(Host="a:8090", Sec_Fetch_Site="none")))

    def test_rate_limiter(self):
        clock = Clock(0)
        lim = dr.RateLimiter(clock=clock)
        for _ in range(30):
            self.assertTrue(lim.allow("answer", "1.2.3.4"))
        self.assertFalse(lim.allow("answer", "1.2.3.4"))           # 31回目
        self.assertTrue(lim.allow("answer", "5.6.7.8"))            # 別の端末は影響しない
        clock.advance(61)
        self.assertTrue(lim.allow("answer", "1.2.3.4"))

    def test_rate_limit_applies_over_http(self):
        self.start()
        ck = self.dlogin()
        codes = [jhttp("POST", self.base + "/display/api/survey/answer", {"x": 1}, cookie=ck)[0] for _ in range(35)]
        self.assertIn(429, codes)
        self.assertEqual(codes[0], 400)


def dc_ctx(test):
    """テストサーバと同じ部品(DisplayContext)を、偽のハンドラで直接叩くために作る。"""
    return dr.DisplayContext(test.hub, dp.build_display_html, dp.build_display_login_html, dr.DisplayAuth("ADMIN_ACCOUNT", DISPLAY_PW))


class DisplayLoginTest(HttpBase):
    """最初に1回だけのログイン: 厨房サイトと同じアカウント、専用のセッション、期限は再起動まで、TLの管理画面とは別系統。"""

    LOGIN = "/display/api/login"

    def test_login_success_sets_a_safe_cookie_and_never_echoes_the_password(self):
        self.start()
        code, d, h = jhttp("POST", self.base + self.LOGIN, {"username": "ADMIN_ACCOUNT", "password": DISPLAY_PW})
        self.assertEqual((code, d), (200, {"ok": True}))
        sc = h["Set-Cookie"]
        self.assertTrue(sc.startswith("nokosahen_display="))
        for attr in ("HttpOnly", "SameSite=Lax", "Path=/display"):
            self.assertIn(attr, sc)
        self.assertNotIn("Secure", sc)                    # http運用(Secure付きだと保存されない)
        self.assertIn("Max-Age=", sc)                     # 閉じても残る(有効期限はサーバ側=再起動まで)
        self.assertNotIn(DISPLAY_PW, sc)
        self.assertNotIn(DISPLAY_PW, json.dumps(d))
        # 管理画面のCookie名とは別(互いに入れない)
        self.assertNotIn("nokosahen_session", sc)

    def test_wrong_credentials_are_401_without_a_cookie(self):
        self.start()
        ctx = dc_ctx(self)
        combos = (("ADMIN_ACCOUNT", "wrong"), ("someone", DISPLAY_PW), ("", ""), ("ADMIN_ACCOUNT", ""), ("mirs03", DISPLAY_PW),
                  ("TL", "tl-pass"), ("guest", ""), ("kari-op", OP_PASSWORD))       # TL/guest/operator のログイン情報ではiPad側に入れない
        for i, (u, p) in enumerate(combos):
            hd = FakeHandler(f"192.168.7.{i + 1}", body=json.dumps({"username": u, "password": p}).encode())   # 端末ごとにロックが別なので、IPを変える
            dr.handle_post(hd, "/display/api/login", "", ctx)
            self.assertEqual(hd.status, 401, (u,))
            self.assertNotIn("Set-Cookie", hd.sent)
            self.assertEqual(hd.json["error"], "invalid_credentials")
        self.assertEqual(ctx.auth.count(), 0)

    def test_login_body_validation(self):
        self.start()
        for body in ({"username": "ADMIN_ACCOUNT"}, {"username": "ADMIN_ACCOUNT", "password": DISPLAY_PW, "x": 1}, {"username": 1, "password": "a"},
                     {"username": "ADMIN_ACCOUNT", "password": None}, {"username": "a" * 300, "password": "b"}):
            self.assertEqual(jhttp("POST", self.base + self.LOGIN, body)[0], 400, body)
        self.assertEqual(jhttp("POST", self.base + self.LOGIN, raw=b"[]")[0], 400)
        self.assertEqual(jhttp("POST", self.base + self.LOGIN, raw=b"{" + b"a" * 2000 + b"}")[0], 413)

    def test_repeated_failures_lock_the_device_even_for_the_right_password(self):
        self.start()
        for _ in range(5):
            self.assertEqual(jhttp("POST", self.base + self.LOGIN, {"username": "ADMIN_ACCOUNT", "password": "bad"})[0], 401)
        code, d, h = jhttp("POST", self.base + self.LOGIN, {"username": "ADMIN_ACCOUNT", "password": DISPLAY_PW})
        self.assertEqual(code, 429)
        self.assertEqual(d["error"], "too_many_attempts")
        self.assertGreater(d["retry_after_sec"], 0)
        self.assertIsNone(h.get("Set-Cookie"))
        self.assertTrue(h.get("Retry-After"))

    def test_lock_is_per_device_and_expires(self):
        lim_clock = Clock(0)
        auth = dr.DisplayAuth("ADMIN_ACCOUNT", DISPLAY_PW, limiter=acc.LoginLimiter(clock=lim_clock))
        ctx = dr.DisplayContext(None, dp.build_display_html, dp.build_display_login_html, auth)
        body = lambda pw: json.dumps({"username": "ADMIN_ACCOUNT", "password": pw}).encode()

        def post(ip, pw):
            hd = FakeHandler(ip, body=body(pw))
            dr.handle_post(hd, "/display/api/login", "", ctx)
            return hd.status
        for _ in range(5):
            self.assertEqual(post("192.168.1.10", "bad"), 401)
        self.assertEqual(post("192.168.1.10", DISPLAY_PW), 429)
        self.assertEqual(post("192.168.1.11", DISPLAY_PW), 200)          # 別の端末は影響を受けない
        lim_clock.advance(301)
        self.assertEqual(post("192.168.1.10", DISPLAY_PW), 200)          # ロックは一定時間で解ける

    def test_login_is_audited_without_secrets(self):
        self.start()
        logs = []
        ctx = dr.DisplayContext(self.hub, dp.build_display_html, dp.build_display_login_html, dr.DisplayAuth("ADMIN_ACCOUNT", DISPLAY_PW),
                                audit=lambda lv, t: logs.append((lv, t)))
        for pw in ("bad", DISPLAY_PW):
            hd = FakeHandler("192.168.1.10", body=json.dumps({"username": "ADMIN_ACCOUNT", "password": pw}).encode())
            dr.handle_post(hd, "/display/api/login", "", ctx)
        text = "\n".join(t for _, t in logs)
        self.assertIn("DISPLAY_LOGIN_FAILED", text)
        self.assertIn("DISPLAY_LOGIN_OK", text)
        self.assertIn("192.168.1.10", text)
        self.assertNotIn(DISPLAY_PW, text)
        self.assertNotIn("bad", text)

    def test_empty_password_or_username_means_nobody_can_log_in(self):
        for u, p in (("ADMIN_ACCOUNT", ""), ("", "x"), ("", "")):
            auth = dr.DisplayAuth(u, p)
            self.assertFalse(auth.enabled)
            self.assertFalse(auth.check(u, p))
            self.assertFalse(auth.check("", ""))

    def test_session_has_no_expiry_and_no_three_minute_logout(self):
        self.start()
        ck = self.dlogin()
        self.hub.request_mode("survey", "admin", defer=False)
        import time as _t
        real = _t.monotonic
        try:
            _t.monotonic = lambda: real() + 10 * 365 * 24 * 3600          # 10年後
            self.assertEqual(jhttp("POST", self.base + "/display/api/survey/session", {"event": "start"}, cookie=ck)[0], 200)
            auth = dr.DisplayAuth("ADMIN_ACCOUNT", DISPLAY_PW)
            tok = auth.create()
            _t.monotonic = lambda: real() + 20 * 365 * 24 * 3600
            self.assertTrue(auth.valid(tok))
        finally:
            _t.monotonic = real
        # 画面側にも自動ログアウトの仕組みが残っていない
        h = dp.build_display_html().decode("utf-8")
        self.assertNotIn("3 * 60 * 1000", h)
        self.assertNotIn("/api/logout", h)
        self.assertNotIn("staffTimer", h)

    def test_session_lasts_only_until_the_process_restarts(self):
        """セッションはメモリ上だけ。ハンドラ(=ダッシュボードのプロセス)を作り直すと、前のCookieは無効になる。"""
        self.start()
        ck = self.dlogin()
        self.assertEqual(jhttp("POST", self.base + "/display/api/menu/edit", {"op": "rename", "name": "Aランチ", "new_name": "Aセット"}, cookie=ck)[0], 200)
        old_base = self.base
        self.start()                                                     # 再起動(新しいハンドラ=新しいセッションストア)
        self.assertNotEqual(old_base, self.base)
        self.assertEqual(jhttp("POST", self.base + "/display/api/inventory", {"menu": "Aランチ", "remaining": 5}, cookie=ck)[0], 401)
        self.assertEqual(http("GET", self.base + "/display", cookie=ck)[1], dp.build_display_login_html())      # ログイン画面に戻る
        hd = FakeHandler("203.0.113.20", {"Cookie": ck})
        dr.handle_get(hd, "/display/api/state", "", dc_ctx(self))
        self.assertEqual(hd.status, 401)                                  # 状態取得が401 → 画面のJSがログイン画面を出す
        ck2 = self.dlogin()
        self.assertEqual(jhttp("POST", self.base + "/display/api/inventory", {"menu": "Aランチ", "remaining": 50}, cookie=ck2)[0], 200)

    def test_no_single_controller_limit_for_ipads(self):
        """TLの管理画面は同時1端末制限があるが、iPad用セッションには適用しない(複数のiPadを同時に使える)。"""
        self.start(single=True)
        tl = self.login("TL", "tl-pass")
        self.assertEqual(jhttp("POST", self.base + "/api/survey/mode", {"mode": "inventory"}, cookie=tl)[0], 200)
        a, b, c = self.dlogin(), self.dlogin(), self.dlogin()
        self.assertEqual(len({a, b, c}), 3)
        for ck in (a, b, c):
            self.assertEqual(jhttp("POST", self.base + "/display/api/inventory", {"menu": "Bランチ", "remaining": 70}, cookie=ck)[0], 200)
            self.assertEqual(jhttp("GET", self.base + "/display/api/state", cookie=ck)[0], 200)
        # TL側の占有は、iPadのログインで追い出されない
        self.assertEqual(jhttp("POST", self.base + "/api/survey/mode", {"mode": "customer"}, cookie=tl)[0], 200)

    def test_ipad_connection_count_is_kept_in_admin_state(self):
        self.start()
        ck = self.dlogin()
        self.assertEqual(self.hub.admin_state()["ipad"]["connected"], 0)
        jhttp("GET", self.base + "/display/api/state", cookie=ck)
        self.assertEqual(self.hub.admin_state()["ipad"]["connected"], 1)

    def test_ros_mode_switch_works_regardless_of_login(self):
        self.start()
        ck = self.dlogin()
        self.hub.request_mode("survey", "ros", defer=True)               # /nokosahen/display_mode 相当(ログインとは無関係)
        self.assertEqual(jhttp("GET", self.base + "/display/api/state", cookie=ck)[1]["mode"], "survey")
        self.assertEqual(jhttp("GET", self.base + "/display/api/state")[1]["mode"], "survey")

    def test_session_cap_drops_the_oldest(self):
        auth = dr.DisplayAuth("ADMIN_ACCOUNT", DISPLAY_PW, max_sessions=3)
        toks = [auth.create() for _ in range(5)]
        self.assertEqual(auth.count(), 3)
        self.assertFalse(auth.valid(toks[0]))
        self.assertTrue(auth.valid(toks[-1]))
        self.assertFalse(auth.valid(None))
        self.assertFalse(auth.valid(""))
        auth.revoke(toks[-1])
        self.assertFalse(auth.valid(toks[-1]))

    def test_inventory_body_validation_with_session(self):
        self.start()
        ck = self.dlogin()
        url = self.base + "/display/api/inventory"
        self.assertEqual(jhttp("POST", url, {"menu": "Aランチ", "remaining": 50}, cookie=ck)[0], 200)
        self.assertEqual(self.sent, [("/nokosahen/sold_total_event", {"menu": "Aランチ", "sold_total": 50, "source": "ipad_display"})])
        self.sent.clear()
        for body in ({"menu": "Aランチ"}, {"menu": "Aランチ", "remaining": 5, "x": 1}, {"menu": "Aランチ", "remaining": "5"},
                     {"menu": "Aランチ", "remaining": -1}, {"menu": "Aランチ", "remaining": 1.5}, {"menu": "Aランチ", "remaining": True},
                     {"menu": "Aランチ", "remaining": 10 ** 6}, {"menu": "存在しない", "remaining": 3}, {"menu": "", "remaining": 3},
                     {"menu": ["A"], "remaining": 3}, {"menu": "A" * 100, "remaining": 3}):
            self.assertEqual(jhttp("POST", url, body, cookie=ck)[0], 400, body)
        self.assertEqual(self.sent, [])

    def test_old_inventory_permission_is_gone(self):
        """ログイン=厨房の権限。iPad用セッションでは display.inventory_input の別チェックはしない(権限表からも外した)。"""
        self.assertNotIn("display.inventory_input", roles.CAPABILITIES)
        self.assertNotIn(("POST", "/display/api/inventory"), roles.API_RULES)
        self.start(perms=["display.inventory_input"])           # 古いアカウント設定に残っていても、未知の名前として付与されない
        self.assertNotIn("display.inventory_input", self.store.operator_permissions())


class MenuEditPlanTest(unittest.TestCase):
    """plan_menu_edit: 新しい一覧の全体を menu_init で送る形と、検証。"""

    MS = {"items": [{"name": "A", "initial": 100, "remaining": 60, "sold": 40}, {"name": "B", "initial": 80, "remaining": 80, "sold": 0}]}

    def plan(self, op, ms=None):
        return dh.plan_menu_edit(self.MS if ms is None else ms, op)

    def test_add_appends_with_sold_zero_and_remaining_equals_initial(self):
        p, err = self.plan({"op": "add", "name": "  Cランチ ", "initial": 50})
        self.assertEqual(err, "")
        self.assertEqual((p["kind"], p["topic"]), ("menu_init", "/nokosahen/menu_init"))
        self.assertEqual(p["payload"]["items"], [
            {"name": "A", "initial": 100, "remaining": 60, "sold": 40}, {"name": "B", "initial": 80, "remaining": 80, "sold": 0},
            {"name": "Cランチ", "initial": 50, "remaining": 50, "sold": 0}])
        self.assertEqual(set(p["payload"]), {"items"})

    def test_rename_keeps_sold_remaining_initial_and_order(self):
        p, err = self.plan({"op": "rename", "name": "A", "new_name": "Aセット"})
        self.assertEqual(p["payload"]["items"][0], {"name": "Aセット", "initial": 100, "remaining": 60, "sold": 40})
        self.assertEqual([i["name"] for i in p["payload"]["items"]], ["Aセット", "B"])

    def test_delete_removes_from_the_full_list(self):
        p, err = self.plan({"op": "delete", "name": "A"})
        self.assertEqual([i["name"] for i in p["payload"]["items"]], ["B"])
        p, err = dh.plan_menu_edit({"items": [{"name": "A", "initial": 1, "remaining": 1, "sold": 0}]}, {"op": "delete", "name": "A"})
        self.assertEqual(p["payload"]["items"], [])                       # 最後の1品を消すと空(inventory_manager は空の一覧を「クリア」として扱う)

    def test_set_initial_recomputes_remaining_and_never_negative(self):
        p, _ = self.plan({"op": "set_initial", "name": "A", "initial": 150})
        self.assertEqual(p["payload"]["items"][0], {"name": "A", "initial": 150, "remaining": 110, "sold": 40})
        p, _ = self.plan({"op": "set_initial", "name": "A", "initial": 30})           # 売れた数(40)より少ない
        self.assertEqual(p["payload"]["items"][0], {"name": "A", "initial": 30, "remaining": 0, "sold": 40})
        p, _ = self.plan({"op": "set_initial", "name": "B", "initial": 0})
        self.assertEqual(p["payload"]["items"][1]["remaining"], 0)

    def test_name_validation(self):
        bad = ["", "   ", "x" * 41, "a\nb", "a\tb", "a\x00b", "a\x07", "a b", "a​b", 5, None, ["A"]]
        for n in bad:
            p, err = self.plan({"op": "add", "name": n, "initial": 1})
            self.assertIsNone(p, repr(n))
            self.assertTrue(err)
        p, err = self.plan({"op": "add", "name": "x" * 40, "initial": 1})
        self.assertIsNotNone(p)
        p, err = self.plan({"op": "add", "name": "ＡＢＣ 麺(昼)・100円", "initial": 1})
        self.assertIsNotNone(p)

    def test_duplicates_are_refused(self):
        self.assertIsNone(self.plan({"op": "add", "name": "A", "initial": 1})[0])
        self.assertIsNone(self.plan({"op": "add", "name": " A ", "initial": 1})[0])
        self.assertIsNone(self.plan({"op": "rename", "name": "A", "new_name": "B"})[0])
        self.assertIsNone(self.plan({"op": "rename", "name": "A", "new_name": "A"})[0])

    def test_max_twelve(self):
        ms = {"items": [{"name": f"M{i}", "initial": 1, "remaining": 1, "sold": 0} for i in range(12)]}
        p, err = dh.plan_menu_edit(ms, {"op": "add", "name": "N", "initial": 1})
        self.assertIsNone(p)
        self.assertIn("12", err)
        self.assertIsNotNone(dh.plan_menu_edit(ms, {"op": "delete", "name": "M0"})[0])
        self.assertIsNotNone(dh.plan_menu_edit(ms, {"op": "rename", "name": "M0", "new_name": "Z"})[0])
        ms11 = {"items": ms["items"][:11]}
        self.assertIsNotNone(dh.plan_menu_edit(ms11, {"op": "add", "name": "N", "initial": 1})[0])

    def test_initial_validation(self):
        for v in (-1, 100000, 10 ** 9, 1.5, "5", None, True, False, [1]):
            self.assertIsNone(self.plan({"op": "add", "name": "N", "initial": v})[0], repr(v))
            self.assertIsNone(self.plan({"op": "set_initial", "name": "A", "initial": v})[0], repr(v))
        for v in (0, 1, 99999):
            self.assertIsNotNone(self.plan({"op": "add", "name": f"N{v}", "initial": v})[0], v)

    def test_unknown_menu_is_refused_for_change_and_delete(self):
        for op in ({"op": "rename", "name": "Z", "new_name": "Y"}, {"op": "delete", "name": "Z"}, {"op": "set_initial", "name": "Z", "initial": 3}):
            p, err = self.plan(op)
            self.assertIsNone(p)
            self.assertEqual(err, "そのメニューは登録されていません")

    def test_op_shape(self):
        for op in (None, [], "add", {}, {"op": "x"}, {"op": 5}, {"op": "add", "name": "N"}, {"op": "add", "name": "N", "initial": 1, "x": 1},
                   {"op": "delete", "name": "A", "initial": 1}, {"op": "rename", "name": "A"}, {"op": "rename", "name": "A", "new_name": "B", "initial": 1},
                   {"op": "set_initial", "name": "A"}):
            self.assertIsNone(self.plan(op)[0], op)

    def test_empty_or_missing_menu_state(self):
        for ms in (None, {}, {"items": []}, {"items": "x"}, {"items": [None, 5, {"name": ""}]}):
            p, err = dh.plan_menu_edit(ms, {"op": "add", "name": "N", "initial": 5})
            self.assertEqual(p["payload"]["items"], [{"name": "N", "initial": 5, "remaining": 5, "sold": 0}])
            self.assertIsNone(dh.plan_menu_edit(ms, {"op": "delete", "name": "N"})[0])


class MenuEditHubTest(unittest.TestCase):
    def make(self, name, items=None):
        self.ms = {"items": items if items is not None else [{"name": "A", "initial": 100, "remaining": 60, "sold": 40}]}
        self.sent = []
        hub, clock, _ = new_hub(name, menu_state_provider=lambda: self.ms,
                                inventory_sink=lambda t, p: self.sent.append((t, p)) or True)
        return hub, clock

    def test_edit_sends_the_whole_list_once(self):
        hub, _ = self.make("me1")
        code, body = hub.edit_menu({"op": "add", "name": "B", "initial": 20})
        self.assertEqual((code, body["ok"], body["count"], body["items"]), (200, True, 2, ["A", "B"]))
        self.assertEqual(len(self.sent), 1)
        self.assertEqual(self.sent[0][0], "/nokosahen/menu_init")
        self.assertEqual([i["name"] for i in self.sent[0][1]["items"]], ["A", "B"])

    def test_errors_do_not_send(self):
        hub, _ = self.make("me2")
        for op in ({"op": "delete", "name": "Z"}, {"op": "add", "name": "", "initial": 1}, None):
            code, body = hub.edit_menu(op)
            self.assertEqual((code, body["ok"]), (400, False))
        self.assertEqual(self.sent, [])

    def test_not_wired_and_sink_failure(self):
        hub, _, _ = new_hub("me3")
        self.assertEqual(hub.edit_menu({"op": "delete", "name": "A"})[0], 503)
        hub = dh.DisplayHub(menu_state_provider=lambda: {"items": [{"name": "A", "initial": 1, "remaining": 1, "sold": 0}]}, inventory_sink=lambda t, p: False)
        self.assertEqual(hub.edit_menu({"op": "delete", "name": "A"})[0], 503)
        hub = dh.DisplayHub(menu_state_provider=lambda: {"items": []}, inventory_sink=lambda t, p: 1 / 0)
        self.assertEqual(hub.edit_menu({"op": "add", "name": "N", "initial": 1})[0], 503)

    def test_second_edit_waits_until_the_first_is_reflected(self):
        """全置き換えなので、状態に反映される前の古い一覧から次の編集を作ると、直前の編集を消してしまう。その間は断る(409)。"""
        hub, clock = self.make("me4")
        self.assertEqual(hub.edit_menu({"op": "add", "name": "B", "initial": 20})[0], 200)
        code, body = hub.edit_menu({"op": "add", "name": "C", "initial": 5})            # menu_state はまだ A だけ
        self.assertEqual(code, 409)
        self.assertEqual(len(self.sent), 1)
        self.ms = {"items": [dict(i) for i in self.sent[0][1]["items"]]}                  # inventory_manager が反映した
        self.assertEqual(hub.edit_menu({"op": "add", "name": "C", "initial": 5})[0], 200)
        self.assertEqual([i["name"] for i in self.sent[1][1]["items"]], ["A", "B", "C"])  # B が消えていない
        # 反映されないまま一定時間が過ぎたら(残数管理が動いていない)、断り続けない
        code, _ = hub.edit_menu({"op": "add", "name": "D", "initial": 5})
        self.assertEqual(code, 409)
        clock.advance(dh.MENU_SYNC_WAIT_S + 1)
        self.assertEqual(hub.edit_menu({"op": "add", "name": "D", "initial": 5})[0], 200)

    def test_set_initial_below_sold_is_reflected_as_sold_when_compared(self):
        hub, clock = self.make("me5")
        self.assertEqual(hub.edit_menu({"op": "set_initial", "name": "A", "initial": 30})[0], 200)
        # inventory_manager は initial = max(initial, remaining + sold) = 40 にする。これも「反映済み」と見なす
        self.ms = {"items": [{"name": "A", "initial": 40, "remaining": 0, "sold": 40}]}
        self.assertEqual(hub.edit_menu({"op": "rename", "name": "A", "new_name": "A2"})[0], 200)

    def test_menu_snapshot(self):
        hub, _ = self.make("me6", items=[{"name": " A ", "initial": "7", "remaining": 3, "sold": 4}, {"name": ""}, "x"])
        snap = hub.menu_snapshot()
        self.assertEqual(snap["items"], [{"name": "A", "initial": 7, "remaining": 3, "sold": 4}])
        self.assertEqual((snap["max_items"], snap["editable"]), (12, True))


class MenuEditInventoryManagerTest(unittest.TestCase):
    """実物の inventory_manager のハンドラで、menu_init(全置き換え)として正しく効く・消費ログと矛盾しない。"""

    @classmethod
    def setUpClass(cls):
        from nokosahen_core import inventory_manager as im
        cls.im = im

    def make_manager(self, items):
        return InventoryIntegrationTest.make_manager(self, items)

    def snapshot(self, m):
        return InventoryIntegrationTest.snapshot(self, m)

    def apply(self, m, plan):
        return InventoryIntegrationTest.apply(self, m, plan)

    def run_edit(self, m, op):
        plan, err = dh.plan_menu_edit(self.snapshot(m), op)
        self.assertEqual(err, "")
        self.apply(m, plan)

    def test_delete_really_removes_and_is_not_merged(self):
        m = self.make_manager([{"name": "A", "initial": 100, "remaining": 60}, {"name": "B", "initial": 80}, {"name": "C", "initial": 5}])
        self.run_edit(m, {"op": "delete", "name": "B"})
        self.assertEqual(list(m._items), ["A", "C"])                       # 消えたメニューが残らない
        self.assertEqual((m._items["A"].initial, m._items["A"].remaining, m._items["A"].sold), (100, 60, 40))
        self.run_edit(m, {"op": "delete", "name": "A"})
        self.run_edit(m, {"op": "delete", "name": "C"})
        self.assertEqual(m._items, {})                                      # 空にもできる

    def test_rename_keeps_numbers_and_old_name_is_gone(self):
        m = self.make_manager([{"name": "A", "initial": 100, "remaining": 60}])
        self.run_edit(m, {"op": "rename", "name": "A", "new_name": "Aセット"})
        self.assertEqual(list(m._items), ["Aセット"])
        it = m._items["Aセット"]
        self.assertEqual((it.initial, it.remaining, it.sold), (100, 60, 40))
        # 旧名の sold_total_event は未知のメニューとして無視される(別の入力元が旧名で送ると効かない)
        m._on_sold_total_event(types.SimpleNamespace(data=json.dumps({"menu": "A", "sold_total": 90})))
        self.assertEqual(m._items["Aセット"].sold, 40)
        # 新名では、これまでどおり絶対値で効く
        m._on_sold_total_event(types.SimpleNamespace(data=json.dumps({"menu": "Aセット", "sold_total": 90})))
        self.assertEqual((m._items["Aセット"].sold, m._items["Aセット"].remaining), (90, 10))

    def test_add_and_set_initial(self):
        m = self.make_manager([{"name": "A", "initial": 100, "remaining": 60}])
        self.run_edit(m, {"op": "add", "name": "N", "initial": 25})
        self.assertEqual((m._items["N"].initial, m._items["N"].remaining, m._items["N"].sold), (25, 25, 0))
        self.run_edit(m, {"op": "set_initial", "name": "A", "initial": 120})
        self.assertEqual((m._items["A"].initial, m._items["A"].remaining, m._items["A"].sold), (120, 80, 40))
        self.run_edit(m, {"op": "set_initial", "name": "A", "initial": 30})      # 売れた数(40)より少ない: 残0、初期は売れた数に切り上がる
        self.assertEqual((m._items["A"].initial, m._items["A"].remaining, m._items["A"].sold), (40, 0, 40))
        # 反映済みかの比較に使う想定値は、この結果と一致する
        plan, _ = dh.plan_menu_edit({"items": [{"name": "A", "initial": 120, "remaining": 80, "sold": 40}]}, {"op": "set_initial", "name": "A", "initial": 30})
        self.assertEqual(plan["expect"], [("A", 40)])

    def test_sold_total_after_edit_is_consistent_with_the_consumption_log(self):
        """メニュー編集の後の「残数入力」(sold_total_event、絶対値)は、引き継いだ売れた数を基準に差分だけ消費ログへ記録される。"""
        records = []
        m = self.make_manager([{"name": "A", "initial": 100, "remaining": 60}])
        m._consumption_log = types.SimpleNamespace(record=lambda **kw: records.append(kw))
        self.run_edit(m, {"op": "rename", "name": "A", "new_name": "A2"})
        self.assertEqual(records, [])                                       # menu_init は消費ログに記録しない(販売ではない)
        plan, _ = dh.plan_inventory_update(self.snapshot(m), "A2", 50)
        self.apply(m, plan)
        self.assertEqual([(r["menu"], r["delta"], r["sold_after"]) for r in records], [("A2", 10, 50)])


class MenuEditHttpTest(HttpBase):
    URL_IPAD = "/display/api/menu/edit"
    URL_TL = "/api/menu/edit"

    def test_ipad_session_can_edit_and_it_sends_menu_init(self):
        self.start()
        ck = self.dlogin()
        code, d, _ = jhttp("POST", self.base + self.URL_IPAD, {"op": "add", "name": "Cランチ", "initial": 30}, cookie=ck)
        self.assertEqual((code, d["ok"], d["items"]), (200, True, ["Aランチ", "Bランチ", "Cランチ"]))
        topic, payload = self.sent[-1]
        self.assertEqual(topic, "/nokosahen/menu_init")
        self.assertEqual(payload["items"][-1], {"name": "Cランチ", "initial": 30, "remaining": 30, "sold": 0})

    def test_ipad_body_validation_and_unauthenticated(self):
        self.start()
        self.assertEqual(jhttp("POST", self.base + self.URL_IPAD, {"op": "delete", "name": "Aランチ"})[0], 401)
        ck = self.dlogin()
        for body in ({"op": "delete"}, {"op": "delete", "name": "ない"}, {"op": "add", "name": "N", "initial": True}, {"op": "x", "name": "A"},
                     {"op": "add", "name": "N" * 41, "initial": 1}):
            self.assertEqual(jhttp("POST", self.base + self.URL_IPAD, body, cookie=ck)[0], 400, body)
        self.assertEqual(jhttp("POST", self.base + self.URL_IPAD, raw=b"x" * 2000, cookie=ck)[0], 413)
        self.assertEqual(self.sent, [])

    def test_admin_tl_can_edit_and_read_the_list(self):
        self.start()
        ck = self.login("TL", "tl-pass")
        code, d, _ = jhttp("GET", self.base + "/api/menu/state", cookie=ck)
        self.assertEqual((code, d["ok"], d["configured"], [i["name"] for i in d["items"]]), (200, True, True, ["Aランチ", "Bランチ"]))
        self.assertEqual((d["max_items"], d["max_name"], d["max_initial"]), (12, 40, 99999))
        code, d, _ = jhttp("POST", self.base + self.URL_TL, {"op": "rename", "name": "Aランチ", "new_name": "Aセット"}, cookie=ck)
        self.assertEqual((code, d["ok"]), (200, True), d)
        self.assertEqual([i["name"] for i in self.sent[-1][1]["items"]], ["Aセット", "Bランチ"])
        self.assertEqual(jhttp("POST", self.base + self.URL_TL, {"op": "delete", "name": "ない"}, cookie=ck)[0], 400)

    def test_guest_and_operator_need_the_new_capability(self):
        self.start()
        gk = self.login("guest", "")
        self.assertEqual(jhttp("GET", self.base + "/api/menu/state", cookie=gk)[0], 200)              # 閲覧はできる
        self.assertEqual(jhttp("POST", self.base + self.URL_TL, {"op": "delete", "name": "Aランチ"}, cookie=gk)[0], 403)
        ok = self.login("kari-op", OP_PASSWORD)
        code, d, _ = jhttp("POST", self.base + self.URL_TL, {"op": "delete", "name": "Aランチ"}, cookie=ok)
        self.assertEqual(code, 403)                                                                   # 既定は保留(許可リストに無ければ拒否)
        self.assertEqual(d["missing"], ["display.menu_edit"])
        self.assertEqual(self.sent, [])

    def test_operator_with_the_capability_can_edit_but_not_other_things(self):
        self.start(perms=["display.menu_edit"])
        ck = self.login("kari-op", OP_PASSWORD)
        code, d, _ = jhttp("POST", self.base + self.URL_TL, {"op": "add", "name": "新", "initial": 3}, cookie=ck)
        self.assertEqual((code, d["ok"]), (200, True), d)
        self.assertEqual(jhttp("POST", self.base + "/api/survey/mode", {"mode": "survey"}, cookie=ck)[0], 403)
        self.assertEqual(jhttp("POST", self.base + "/api/speed_settings", {"manual_linear_mps": 0.3}, cookie=ck)[0], 403)

    def test_ipad_menu_edit_is_not_reachable_with_display_login_via_tl_api(self):
        self.start()
        dk = self.dlogin()
        self.assertEqual(jhttp("POST", self.base + self.URL_TL, {"op": "delete", "name": "Aランチ"}, cookie=dk)[0], 401)

    def test_capability_is_registered_and_in_the_rule_table(self):
        self.assertIn("display.menu_edit", roles.GRANTABLE)
        self.assertEqual(roles.CAPABILITIES["display.menu_edit"].recommend, "保留")
        self.assertEqual(roles.API_RULES[("POST", "/api/menu/edit")], "display.menu_edit")
        self.assertEqual(roles.required_capabilities("POST", "/api/menu/edit", {}), ["display.menu_edit"])
        self.assertFalse(roles.authorize("operator", [], "POST", "/api/menu/edit", {}).allowed)
        self.assertTrue(roles.authorize("operator", ["display.menu_edit"], "POST", "/api/menu/edit", {}).allowed)
        self.assertTrue(roles.authorize("admin", [], "POST", "/api/menu/edit", {}).allowed)
        self.assertFalse(roles.authorize("viewer", ["display.menu_edit"], "POST", "/api/menu/edit", {}).allowed)
        # 表に載っていない書き込みは admin 専用のまま(既定拒否の枠組み)
        self.assertEqual(roles.required_capabilities("POST", "/api/menu/other", {}), ["admin.unclassified"])
        granted, problems = roles.sanitize_operator_permissions(["display.menu_edit"])
        self.assertEqual(granted, {"display.menu_edit"})

    def test_menu_state_is_unconfigured_without_the_display_hub(self):
        h = dc.make_dashboard_handler(lambda: b"x", lambda: {}, FakeTeleop(), auth_username="TL", auth_password="tl-pass",
                                      enforce_single_controller=False)
        srv = ThreadingHTTPServer(("127.0.0.1", 0), h)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        self.addCleanup(lambda: (srv.shutdown(), srv.server_close()))
        base = f"http://127.0.0.1:{srv.server_address[1]}"
        ck = jhttp("POST", base + "/api/login", {"username": "TL", "password": "tl-pass"})[2].get("Set-Cookie", "").split(";")[0]
        self.assertEqual(jhttp("GET", base + "/api/menu/state", cookie=ck)[1]["configured"], False)


class ModeSwitchHttpTest(HttpBase):
    """ダッシュボードの管理API(既存の /api/survey/mode)から、iPadの表示が同じ「モード」で切り替わる。"""

    def test_admin_api_switches_the_display_mode_seen_by_ipad(self):
        self.start()
        ck = self.login("TL", "tl-pass")
        ck_ipad = self.dlogin()
        for m in ("survey", "inventory", "ocrtest", "customer"):
            code, d, _ = jhttp("POST", self.base + "/api/survey/mode", {"mode": m}, cookie=ck)
            self.assertEqual((code, d["ok"]), (200, True), (m, d))
            self.assertEqual(jhttp("GET", self.base + "/display/api/state", cookie=ck_ipad)[1]["mode"], m)       # ログイン済みのiPadから見える(ROSの切替と同じく、ログインとは無関係)
            self.assertEqual(jhttp("GET", self.base + "/api/survey/state", cookie=ck)[1]["state"]["mode"], m)
        self.assertEqual(jhttp("POST", self.base + "/api/survey/mode", {"mode": "kiosk"}, cookie=ck)[0], 400)
        self.assertEqual(jhttp("POST", self.base + "/api/survey/mode", {"mode": "survey", "x": 1}, cookie=ck)[0], 400)

    def test_manual_switch_refused_while_answering_via_api(self):
        self.start()
        ck = self.login("TL", "tl-pass")
        jhttp("POST", self.base + "/api/survey/mode", {"mode": "survey"}, cookie=ck)
        jhttp("POST", self.base + "/display/api/survey/session", {"event": "start"}, cookie=self.dlogin())
        code, d, _ = jhttp("POST", self.base + "/api/survey/mode", {"mode": "customer"}, cookie=ck)
        self.assertEqual(code, 200)
        self.assertFalse(d["ok"])
        self.assertIn("回答中", d["message"])
        self.assertEqual(jhttp("GET", self.base + "/display/api/state")[1]["mode"], "survey")

    def test_ocr_mock_set_via_admin_api_is_shown_to_ipad_with_readback(self):
        self.start()
        ck = self.login("TL", "tl-pass")
        ck_ipad = self.dlogin()
        code, d, _ = jhttp("POST", self.base + "/api/survey/ocr_mock", {"action": "randomize"}, cookie=ck)
        self.assertFalse(d["ok"])                                    # ocrtestモードでないと断る(従来と同じ)
        self.assertIn("OCRテストモードではありません", d["message"])
        jhttp("POST", self.base + "/api/survey/mode", {"mode": "ocrtest"}, cookie=ck)
        code, d, _ = jhttp("POST", self.base + "/api/survey/ocr_mock", {"action": "set", "values": [1, 22, 333, 4, 5, 66, 777]}, cookie=ck)
        self.assertTrue(d["ok"])
        seq = d["state"]["ocr_mock"]["seq"]
        st = jhttp("GET", self.base + "/display/api/state?seen=" + str(seq - 1), cookie=ck_ipad)[1]     # 古い番号を表示中
        self.assertEqual(st["ocr_mock"]["values"], [1, 22, 333, 4, 5, 66, 777])
        adm = jhttp("GET", self.base + "/api/survey/state", cookie=ck)[1]["state"]["ocr_mock"]
        self.assertNotIn("ack_seq", adm)                              # まだ表示したという報告は無い
        jhttp("GET", self.base + "/display/api/state?seen=" + str(seq), cookie=ck_ipad)                # iPadが表示した
        adm = jhttp("GET", self.base + "/api/survey/state", cookie=ck)[1]["state"]["ocr_mock"]
        self.assertEqual((adm["ack_seq"], adm["shown_values"]), (seq, [1, 22, 333, 4, 5, 66, 777]))
        # 不正な値は断る
        self.assertEqual(jhttp("POST", self.base + "/api/survey/ocr_mock", {"action": "set", "values": [1, 2]}, cookie=ck)[0], 400)
        self.assertEqual(jhttp("POST", self.base + "/api/survey/ocr_mock", {"action": "set", "values": [10000] * 7}, cookie=ck)[0], 400)

    def test_csv_download_requires_login_and_matches_saved_answers(self):
        self.start()
        jhttp("POST", self.base + "/display/api/survey/answer", good_answer(), cookie=self.dlogin())
        self.assertEqual(http("GET", self.base + "/api/survey/responses.csv")[0], 401)
        ck = self.login("guest", "")
        self.assertEqual(http("GET", self.base + "/api/survey/responses.csv", cookie=ck)[0], 403)
        ck = self.login("TL", "tl-pass")
        code, body, h = http("GET", self.base + "/api/survey/responses.csv", cookie=ck)
        self.assertEqual(code, 200)
        self.assertIn("resp-0001-abcdef".encode(), body)


# ---------------------------------------------------------------------------
# 残数入力: 既存の残数管理(inventory_manager)の実物のハンドラで検証
# ---------------------------------------------------------------------------
class InventoryIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from nokosahen_core import inventory_manager as im
        cls.im = im

    def make_manager(self, items):
        m = object.__new__(self.im.InventoryManager)
        m._items = {}
        m._promoted_item = None
        m._last_state = {}
        m._consumption_log = None
        m.log_file = ""
        m._log_write_failures = 0
        m._log_write_failure_limit = 10
        m.promote_gap_count = 5
        m.promote_gap_ratio = 0.15
        m.promote_hysteresis_ratio = 0.5
        m.initial_phase_seconds = 120.0
        m.get_logger = lambda: types.SimpleNamespace(info=lambda *a: None, warn=lambda *a: None, error=lambda *a: None)
        m.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=0))
        m._started_at = m.get_clock().now()
        m._publish_state = lambda: None
        m._on_menu_init(types.SimpleNamespace(data=json.dumps({"items": items})))
        return m

    def snapshot(self, m):
        return {"items": [dict(name=i.name, initial=i.initial, remaining=i.remaining, sold=i.sold) for i in m._items.values()]}

    def apply(self, m, plan):
        msg = types.SimpleNamespace(data=json.dumps(plan["payload"]))
        if plan["topic"] == "/nokosahen/sold_total_event":
            m._on_sold_total_event(msg)
        else:
            m._on_menu_init(msg)

    def test_remaining_lower_or_equal_uses_sold_total(self):
        m = self.make_manager([{"name": "A", "initial": 100}, {"name": "B", "initial": 80}])
        for want in (60, 0, 100, 37, 37):                     # 同じ値の再送でも二重計上しない
            plan, err = dh.plan_inventory_update(self.snapshot(m), "A", want)
            self.assertEqual(plan["kind"], "sold_total")
            self.apply(m, plan)
            self.assertEqual(m._items["A"].remaining, want)
            self.assertEqual(m._items["A"].initial, 100)
            self.assertEqual(m._items["B"].remaining, 80)      # 他のメニューは変わらない

    def test_remaining_above_initial_raises_initial_and_keeps_others(self):
        m = self.make_manager([{"name": "A", "initial": 100, "remaining": 60}, {"name": "B", "initial": 80, "remaining": 20}])
        before_b = (m._items["B"].initial, m._items["B"].remaining, m._items["B"].sold)
        plan, err = dh.plan_inventory_update(self.snapshot(m), "A", 130)
        self.assertEqual(plan["kind"], "menu_init")
        self.apply(m, plan)
        self.assertEqual(m._items["A"].remaining, 130)
        self.assertEqual(m._items["A"].sold, 40)               # 売れた数は保つ
        self.assertEqual(m._items["A"].initial, 170)
        self.assertEqual((m._items["B"].initial, m._items["B"].remaining, m._items["B"].sold), before_b)

    def test_plan_errors(self):
        snap = {"items": [{"name": "A", "initial": 10, "remaining": 5, "sold": 5}]}
        for args in ((None, "A", 1), ({"items": []}, "A", 1), (snap, "Z", 1), (snap, "A", -1), (snap, "A", 1.0),
                     (snap, "A", True), (snap, "A", "3"), (snap, "", 3), (snap, 5, 3), (snap, "A", 10 ** 6)):
            plan, err = dh.plan_inventory_update(*args)
            self.assertIsNone(plan, args)
            self.assertTrue(err)

    def test_hub_set_inventory_returns_503_when_not_wired(self):
        hub, _, _ = new_hub("inv_nowire")
        code, body = hub.set_inventory("A", 1)
        self.assertEqual(code, 503)

    def test_hub_set_inventory_sink_failure(self):
        hub, _, _ = new_hub("inv_sinkfail", menu_state_provider=lambda: {"items": [{"name": "A", "initial": 10, "remaining": 5, "sold": 5}]},
                            inventory_sink=lambda t, p: False)
        self.assertEqual(hub.set_inventory("A", 3)[0], 503)


class CustomerPayloadTest(unittest.TestCase):
    def test_status_text_per_director_mode(self):
        cs = dh.customer_status
        self.assertEqual(cs({"mode": "APPROACH_PERSON"}, 0.5)["tone"], "ok")
        self.assertEqual(cs({"mode": "NEAR_PERSON_HOLD"}, 0.5)["title"], "止まっています")
        self.assertEqual(cs({"mode": "ESTOP_HOLD"}, 0.5)["tone"], "warn")
        self.assertEqual(cs({"mode": "PATROL_X"}, 0.5)["title"], "巡回中です")            # 動いている通常の状態
        self.assertEqual(cs({"mode": "APPROACH_PERSON", "estop_active": True}, 0.5)["key"], "ESTOP_HOLD")   # 非常停止が優先
        # 指令が無い・古いときは「走行中」と嘘を出さない
        self.assertEqual(cs(None, None)["title"], "準備中です")
        self.assertEqual(cs({"mode": "APPROACH_PERSON"}, 999)["title"], "準備中です")
        self.assertEqual(cs({}, 1)["key"], "UNKNOWN")

    def test_menu_is_sanitized_and_limited(self):
        items = [{"name": "A" * 100, "initial": "x", "remaining": None, "sold": 3}] + [{"name": f"m{i}", "initial": 10, "remaining": 5} for i in range(30)]
        out = dh.customer_menu({"items": items + ["junk", 5, {"name": ""}], "promote_item": "  "})
        self.assertLessEqual(len(out["items"]), dh.MAX_CUSTOMER_ITEMS)
        self.assertEqual(len(out["items"][0]["name"]), dh.MAX_MENU_NAME)
        self.assertEqual(out["items"][0]["ratio"], 0.0)
        self.assertIsNone(out["promote_item"])
        self.assertEqual(dh.customer_menu(None), {"items": [], "promote_item": None})


# ---------------------------------------------------------------------------
# 【2026-10-27】「移動中です。ご注意ください」(moving_notice.py の判定を customer の「今の状態」に出す)
# ---------------------------------------------------------------------------
MOVING_ON = {"display": True, "sound": True, "kind": "go_pin", "reason": "", "moving": True}


class MovingStatusTest(unittest.TestCase):
    def provider(self, moving):
        menu = {"items": [{"name": "Aランチ", "initial": 100, "remaining": 60, "sold": 40}]}
        return lambda: dh.build_customer_payload(menu, {"mode": "POLICY_GO_PIN"}, 0.5, "おすすめです", moving)

    def hub_with(self, name, moving):
        hub, clock, d = new_hub(name, customer_provider=self.provider(moving))
        return hub

    def test_payload_marks_moving_only_when_display_is_true(self):
        p = dh.build_customer_payload(None, {"mode": "POLICY_GO_PIN"}, 0.5, None, MOVING_ON)
        self.assertEqual(p["moving"], {"active": True, "kind": "go_pin"})
        self.assertEqual(p["status"]["title"], "巡回中です")        # status は通常の文言のまま(差し替えは hub が customer のときだけ)
        for off in (None, {}, {"display": False, "sound": True}, {"display": "yes"}, "x", []):
            self.assertNotIn("moving", dh.build_customer_payload(None, {"mode": "POLICY_GO_PIN"}, 0.5, None, off), off)

    def test_customer_mode_shows_the_caution_status_and_keeps_the_remaining_counts(self):
        hub = self.hub_with("mv1", MOVING_ON)
        c = hub.public_state()["customer"]
        self.assertEqual(c["status"]["key"], "MOVING_NOTICE")
        self.assertEqual(c["status"]["title"], "移動中です。ご注意ください")
        self.assertEqual(c["status"]["tone"], "caution")
        self.assertEqual(c["status"]["moving"], "go_pin")
        self.assertNotIn("moving", c)
        self.assertEqual(c["menu"]["items"][0]["remaining"], 60)          # 残数は右側に出したまま
        self.assertEqual(c["announce"], "おすすめです")

    def test_other_modes_are_never_overwritten(self):
        hub = self.hub_with("mv2", MOVING_ON)
        for m in ("survey", "inventory", "ocrtest"):
            hub.request_mode(m, "admin", defer=False)
            st = hub.public_state()
            self.assertEqual(st["mode"], m)
            c = st.get("customer")
            if m == "inventory":                                          # 残数入力の画面も「移動中」で上書きしない
                self.assertEqual(c["status"]["title"], "巡回中です")
                self.assertNotIn("moving", c)
            else:
                self.assertIsNone(c)
        hub.request_mode("customer", "admin", defer=False)
        self.assertEqual(hub.public_state()["customer"]["status"]["key"], "MOVING_NOTICE")

    def test_not_active_keeps_the_normal_status_and_never_lies(self):
        for mv in (None, {"display": False, "reason": "odom_stale"}):
            c = self.hub_with("mv3", mv).public_state()["customer"]
            self.assertEqual(c["status"]["title"], "巡回中です")
        # director の指令が古いときは従来どおり「準備中」(移動中を出さない)
        hub, _, _ = new_hub("mv4", customer_provider=lambda: dh.build_customer_payload(None, {"mode": "POLICY_GO_PIN"}, 999, None, None))
        self.assertEqual(hub.public_state()["customer"]["status"]["title"], "準備中です")

    def test_answering_survey_is_not_touched(self):
        hub = self.hub_with("mv5", MOVING_ON)
        hub.request_mode("survey", "admin", defer=False)
        hub.session_event("start")
        st = hub.public_state()
        self.assertEqual((st["mode"], st["active"], st["session_state"]), ("survey", True, "answering"))
        self.assertNotIn("customer", st)

    def test_provider_failure_does_not_break_state(self):
        def boom():
            raise RuntimeError("x")
        hub, _, _ = new_hub("mv6", customer_provider=boom)
        self.assertEqual(hub.public_state()["customer"]["status"]["title"], "準備中です")

    def test_moving_status_helper_is_bounded(self):
        s = dh.moving_status("x" * 100)
        self.assertLessEqual(len(s["moving"]), 20)
        self.assertEqual(s["tone"], "caution")


class MovingPageTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.html = dp.build_display_html().decode("utf-8")

    def test_caution_tone_is_amber_big_and_has_a_warning_icon(self):
        h = self.html
        self.assertIn(".cstatus[data-tone=caution]{", h)
        self.assertIn("#ffb400", h)                                        # 琥珀色の地
        self.assertIn("caution: '<svg", h)                                 # 警告アイコン(固定の図形)
        self.assertIn(".cstatus[data-tone=caution] .tt{font-size:calc(var(--u) * 6.2)", h)   # 「ご注意ください」7文字が1行に収まる大きさ(2行表示)
        self.assertIn("repeating-linear-gradient", h)                      # 斜めの警戒帯
        self.assertIn("prefers-reduced-motion", h)                         # 動きを減らす設定では脈動しない

    def test_tone_comes_from_the_server_status_and_only_known_tones_are_used(self):
        h = self.html
        self.assertIn('var tone = ICONS[st.tone] ? st.tone : "info";', h)
        self.assertIn('$("cstatus").setAttribute("data-tone", tone);', h)
        self.assertIn('$("ctt").textContent = tone === "caution"', h)      # 文言は textContent(innerHTML に入れない)。注意の文言は「。」で改行

    def test_remaining_cards_are_still_rendered_alongside(self):
        h = self.html
        self.assertIn('id="cards"', h)
        self.assertIn('grid-template-areas:"bar cards" "status cards" "promote cards" "announce cards"', h)    # 横向きは左=状態/右=残数のまま

    def test_portrait_fallback_has_caution_sizes(self):
        self.assertIn("html[data-orient=portrait] .cstatus[data-tone=caution] .tt", self.html)


class MovingHttpTest(HttpBase):
    def test_ipad_reads_the_moving_status_with_the_login_session_only(self):
        self.start()
        self.hub._customer_provider = lambda: dh.build_customer_payload(
            self.menu_state, {"mode": "POLICY_GO_PIN"}, 0.5, None, MOVING_ON)
        ck = self.dlogin()
        code, st, _ = jhttp("GET", self.base + "/display/api/state", cookie=ck)
        self.assertEqual(code, 200)
        self.assertEqual(st["customer"]["status"]["tone"], "caution")
        self.assertEqual(st["customer"]["status"]["title"], "移動中です。ご注意ください")
        self.assertEqual(st["customer"]["menu"]["items"][0]["remaining"], 60)
        # ループバック(survey_monitor)から読めるのは従来どおり。認証の枠組みは変えない
        self.assertEqual(jhttp("GET", self.base + "/display/api/state")[0], 200)


# ---------------------------------------------------------------------------
# 既存の仕組み(スケジュール・一時的な上書き)が同じ「モード」で動く
# ---------------------------------------------------------------------------
class ExistingMechanismsTest(unittest.TestCase):
    def make_controller(self, name):
        hub, clock, d = new_hub(name)
        admin = dh.LocalDisplayAdmin(hub)
        ctl = sd.DisplayScheduleController(admin, baseline_file=str(d / "baseline.json"), clock=clock)
        return hub, clock, admin, ctl

    def test_schedule_display_request_switches_the_ipad(self):
        hub, clock, admin, ctl = self.make_controller("em1")
        ctl.on_request_text(json.dumps({"mode": "survey", "source": "schedule", "entry_id": "e1"}))
        ctl.tick()
        self.assertEqual(hub.mode, "survey")
        ctl.on_request_text(json.dumps({"mode": "keep", "source": "schedule", "entry_id": None}))
        ctl.tick()
        self.assertEqual(hub.mode, "customer")                  # 元のモードへ戻る

    def test_schedule_does_not_switch_while_answering(self):
        hub, clock, admin, ctl = self.make_controller("em2")
        hub.request_mode("survey", "admin", defer=False)
        hub.session_event("start")
        ctl.on_request_text(json.dumps({"mode": "ocrtest", "source": "schedule", "entry_id": "e1"}))
        st = ctl.tick()
        self.assertEqual(hub.mode, "survey")
        self.assertEqual(st["state"], sd.STATE_PENDING)
        hub.session_event("end")
        clock.advance(dh.ABANDONED_SHOW_S + 1)
        hub.tick()
        ctl.tick()
        self.assertEqual(hub.mode, "ocrtest")

    def test_survey_answering_helper_sees_the_hub(self):
        hub, _, _ = new_hub("em3")
        admin = dh.LocalDisplayAdmin(hub)
        self.assertIs(eo.survey_answering(admin), False)
        hub.request_mode("survey", "admin", defer=False)
        hub.session_event("start")
        self.assertIs(eo.survey_answering(admin), True)

    def test_ephemeral_restore_does_not_switch_while_answering(self):
        hub, _, _ = new_hub("em4")
        admin = dh.LocalDisplayAdmin(hub)
        restorers = eo.build_restorers(None, admin)
        hub.request_mode("survey", "admin", defer=False)
        hub.session_event("start")
        self.assertFalse(restorers[eo.RESTORE_SURVEY_MODE]("customer"))      # 回答中は戻さない(ジャーナルに残して再試行)
        self.assertEqual(hub.mode, "survey")
        hub.session_event("end")                                             # 回答が終わったら戻せる(ジャーナルの再試行)
        self.assertTrue(restorers[eo.RESTORE_SURVEY_MODE]("inventory"))
        self.assertEqual(hub.mode, "inventory")

    def test_admin_state_matches_dashboard_card_contract(self):
        hub, _, _ = new_hub("em5")
        st = dh.LocalDisplayAdmin(hub).get_state()["state"]
        for k in ("mode", "active", "count", "last_activity_age_s", "condition", "session_state", "receiver", "ipad"):
            self.assertIn(k, st)
        self.assertTrue(st["receiver"])                      # 「受信サーバ停止中」の警告を出さない


class RosWiringTest(unittest.TestCase):
    """full_dashboard_node の配線(ROSが無いので、ソースを読んで確認する)。"""

    SRC = (PKG / "nokosahen_core" / "full_dashboard_node.py").read_text(encoding="utf-8")

    def test_topics_and_parameters(self):
        for needle in ('"/nokosahen/display_mode"', '"/nokosahen/display_state"', '"ipad_display_enabled"',
                       "LocalDisplayAdmin(self.display_hub)", "display_hub=self.display_hub",
                       '"/nokosahen/sold_total_event"', '"/nokosahen/menu_init"', "defer=True"):
            self.assertIn(needle, self.SRC)

    def test_display_login_uses_the_same_secret_as_the_kitchen_site_and_never_holds_a_password(self):
        # web_ui_node.py と同じ仕組み(secrets_loader.resolve_password の web_ui / auth_password)で解決する
        web = (PKG / "nokosahen_core" / "web_ui_node.py").read_text(encoding="utf-8")
        self.assertIn('resolve_password(\n            self.get_logger(), "web_ui", "auth_password"', web.replace("\r\n", "\n"))
        src = self.SRC.replace("\r\n", "\n")
        self.assertIn('self.declare_parameter("display_auth_username", "ADMIN_ACCOUNT")', src)
        self.assertIn('self.declare_parameter("display_auth_password", "")', src)         # 既定は空(コードにパスワードを書かない)
        self.assertIn('self.get_logger(), "web_ui", "auth_password",\n                str(self.get_parameter("display_auth_password").value)', src)
        self.assertIn("display_auth_username=str(self.get_parameter(\"display_auth_username\").value)", src)
        y = (PKG.parents[0] / "nokosahen_bringup" / "config" / "nokosahen.yaml").read_text(encoding="utf-8")
        self.assertNotRegex(y, r"display_auth_password:\s*\S*[A-Za-z0-9]")        # yamlにパスワードを書かない

    def test_tl_settings_tab_has_the_menu_edit_card(self):
        src = self.SRC
        for needle in ('id="menuEditBlock"', 'data-cap="display.menu_edit"', "/api/menu/edit", "/api/menu/state",
                       '"追加"', 'id="menuConfirm"', "mnLoad("):
            self.assertIn(needle, src)

    def test_old_raspi_path_is_kept(self):
        self.assertIn("SurveyAdminProxy(str(self.get_parameter(\"survey_admin_url\").value))", self.SRC)
        for f in ("kiosk_admin.py", "receiver.py", "switch_kiosk.sh", "start_kiosk.sh", "mirs_robot_survey.html"):
            self.assertTrue(list((PKG.parents[2] / "scripts" / "raspi_display").rglob(f)), f)

    def test_yaml_points_survey_monitor_at_the_local_display(self):
        y = (PKG.parents[0] / "nokosahen_bringup" / "config" / "nokosahen.yaml").read_text(encoding="utf-8")
        self.assertIn('survey_url: "http://127.0.0.1:8090/display/api"', y)
        self.assertIn("ipad_display_enabled: true", y)

    def test_moving_notice_is_wired_into_the_display_provider(self):
        src = self.SRC.replace("\r\n", "\n")
        self.assertIn("build_customer_payload(self._menu_state, self._behavior_command, age, self._announce_text, moving)", src)
        self.assertIn("(time.monotonic() - self._moving_result_t) <= 1.5", src)       # 判定が止まったら「移動中」を出さない

    def test_moving_sound_gate_uses_the_same_mode_as_the_screen(self):
        """【2026-10-30】音の判定(MovingNotice)に iPad の表示モードを渡し、画面(DisplayHub._apply_moving)と同じ「customer のときだけ」にそろえる。"""
        src = self.SRC.replace("\r\n", "\n")
        self.assertIn("display_enabled=hub is not None, display_mode=(hub.mode if hub is not None else None))", src)
        import nokosahen_core.moving_notice as mvn
        self.assertEqual(mvn.CUSTOMER_MODE, "customer")
        hub_src = (Path(dh.__file__)).read_text(encoding="utf-8").replace("\r\n", "\n")
        self.assertIn('if self.mode == "customer" and isinstance(mv, dict) and mv.get("active") is True:', hub_src)
        # 画面が出ないモードでは、MovingNotice も sound=False
        for dm in ("survey", "inventory", "ocrtest"):
            hub, clock, d = new_hub("snd_" + dm, customer_provider=lambda: dh.build_customer_payload(None, {"mode": "POLICY_GO_PIN"}, 0.5, None, MOVING_ON))
            hub.request_mode(dm, "test")
            n = mvn.MovingNotice()
            for i in range(12):
                n.on_odom(0.2, i * 0.1)
                n.on_behavior({"mode": "POLICY_GO_PIN"}, i * 0.1)
                r = n.evaluate(i * 0.1, display_enabled=True, display_mode=hub.mode)
            self.assertEqual((hub.mode, r["sound"], r["display"]), (dm, False, False))

    def test_origin_ok_normalizes_case_and_default_ports(self):
        h = lambda **kw: types.SimpleNamespace(get=lambda k, d=None: kw.get(k.replace("-", "_"), d))
        self.assertTrue(dr.origin_ok(h(Host="Jetson.local:8090", Origin="http://JETSON.LOCAL:8090")))
        self.assertTrue(dr.origin_ok(h(Host="jetson.local", Origin="http://jetson.local:80")))
        self.assertTrue(dr.origin_ok(h(Host="[::1]:8090", Origin="http://[::1]:8090")))
        self.assertFalse(dr.origin_ok(h(Host="192.0.2.1:8090", Origin="http://192.0.2.1:8095")))
        self.assertFalse(dr.origin_ok(h(Host="a:8090", Origin="null")))
        self.assertFalse(dr.origin_ok(h(Host="a:8090", Origin="http://a:8090x")))


if __name__ == "__main__":
    unittest.main(verbosity=1)
