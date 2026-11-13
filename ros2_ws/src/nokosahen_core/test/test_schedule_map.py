"""かんたん時間割ページの「地図で指定」パネルのテスト(2026-10-23、ROS不要)。

- HTMLに地図パネルの要素id・関数名・APIパスが含まれる。
- 座標変換・矩形の正規化などの純粋関数(PURE-BEGIN〜PURE-ENDの間)をJSから取り出してnodeで実行し、数値で検証する。
  (nodeが無ければスキップ。ブラウザでの表示確認はここではできない)
"""
import json
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from nokosahen_core import schedule_page as sp  # noqa: E402


def _html():
    return sp.build_schedule_html().decode("utf-8")


def _pure_js(html):
    m = re.search(r"// ==PURE-BEGIN==(.*)// ==PURE-END==", html, re.S)
    assert m, "PURE区間が無い"
    return m.group(1)


class ScheduleMapHtml(unittest.TestCase):
    def setUp(self):
        self.html = _html()

    def test_ids_unique_and_present(self):
        ids = re.findall(r'id="([^"]+)"', self.html)
        self.assertEqual(len(ids), len(set(ids)), "id重複")
        for i in ("mapSection", "mapCv", "mapWrap", "mapMsg", "mapEditInfo", "mapLegend",
                  "mapZoomIn", "mapZoomOut", "mapFitBtn", "mapClearRectBtn", "mapDoneBtn"):
            self.assertIn('id="%s"' % i, self.html, i)
        self.assertIn('data-act="mapedit"', self.html)
        self.assertIn("地図で指定", self.html)

    def test_functions_and_paths(self):
        for f in ("mapWorldToScreen", "mapScreenToWorld", "mapNormRect", "mapFitView", "mapZoomAt",
                  "mapPanBy", "mapHit", "mapResizeRect", "mapMoveRect", "mapPinAt", "mapShadeOf", "mapImgIndexOf"):
            self.assertIn("function %s(" % f, self.html, f)
        for path in ("/api/map", "/api/pins", "/api/boundary"):
            self.assertIn('"%s"' % path, self.html, path)
        self.assertEqual(sp.API_MAP, "/api/map")
        self.assertNotIn("__API_", self.html)

    def test_messages(self):
        self.assertIn("地図がありません(地図作成または保存した地図の読み込み後に使えます)", self.html)
        self.assertIn("本番は範囲の角にコーンを置く想定。いまは地図上でドラッグするか、機体を角に動かして座標を確認してください", self.html)
        self.assertIn("全体表示", self.html)
        self.assertIn("pointerdown", self.html)    # マウス・タッチ共通
        self.assertIn("touch-action:none", self.html)

    def test_existing_ui_kept(self):
        for i in ("entries", "addBtn", "saveBtn", "previewBody", "runBtn", "stopBtn", "estopBtn"):
            self.assertIn('id="%s"' % i, self.html, i)
        self.assertIn('data-act="grab"', self.html)


@unittest.skipUnless(shutil.which("node"), "nodeが無い")
class ScheduleMapPureFunctions(unittest.TestCase):
    """JSの純粋関数を単独で実行して数値を確認する。"""

    def run_js(self, body):
        js = _pure_js(_html()) + "\n" + body
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "pure.js"
            p.write_text(js, encoding="utf-8")
            r = subprocess.run([shutil.which("node"), str(p)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)
        return json.loads(r.stdout)

    def test_pure_block_syntax(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "pure.js"
            p.write_text(_pure_js(_html()), encoding="utf-8")
            r = subprocess.run([shutil.which("node"), "--check", str(p)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)

    def test_roundtrip_and_orientation(self):
        out = self.run_js("""
const meta = {width: 200, height: 100, resolution: 0.05, origin_x: -3.0, origin_y: -2.0};
const v = mapFitView(meta, 800, 400);
const res = {};
res.v = v;
// 画面→map→画面の往復
const pts = [[0, 0], [400, 200], [799, 399], [123.5, 77.25]];
res.rt = pts.map(([sx, sy]) => { const w = mapScreenToWorld(v, sx, sy); return mapWorldToScreen(v, w[0], w[1]).map((n, i) => n - [sx, sy][i]); });
// 向き: yが大きいほど画面の上(syが小さい)、xが大きいほど右
res.up = mapWorldToScreen(v, 0, 1)[1] < mapWorldToScreen(v, 0, 0)[1];
res.right = mapWorldToScreen(v, 1, 0)[0] > mapWorldToScreen(v, 0, 0)[0];
// 地図の中心(origin + 半分)が画面中央
res.center = mapWorldToScreen(v, -3.0 + 5.0, -2.0 + 2.5);
// 既知の値: scale 100px/m, 中心(1,2), 画面400x300 → map(1,2)は(200,150)、map(2,3)は(300,50)
const k = {cx: 1, cy: 2, s: 100, cw: 400, ch: 300};
res.k1 = mapWorldToScreen(k, 1, 2); res.k2 = mapWorldToScreen(k, 2, 3); res.k3 = mapScreenToWorld(k, 0, 300);
console.log(JSON.stringify(res));
""")
        for row in out["rt"]:
            for d in row:
                self.assertAlmostEqual(d, 0.0, places=9)
        self.assertTrue(out["up"])
        self.assertTrue(out["right"])
        self.assertAlmostEqual(out["center"][0], 400, places=6)
        self.assertAlmostEqual(out["center"][1], 200, places=6)
        self.assertEqual(out["k1"], [200, 150])
        self.assertEqual(out["k2"], [300, 50])
        self.assertEqual(out["k3"], [-1, 0.5])

    def test_grid_row_zero_is_bottom(self):
        out = self.run_js("""
// 3x2の格子: 行0(下)の先頭セル i=0 は、画像の最下行(行1)の先頭 = 添字 3
const res = {};
res.i0 = mapImgIndexOf(0, 3, 2);   // 行0,列0 -> 画像の下の行
res.i2 = mapImgIndexOf(2, 3, 2);
res.i3 = mapImgIndexOf(3, 3, 2);   // 行1,列0 -> 画像の上の行
res.i5 = mapImgIndexOf(5, 3, 2);
res.shade = [mapShadeOf(0), mapShadeOf(1), mapShadeOf(101)];   // 未知/空き/壁(バイト=占有値+1)
console.log(JSON.stringify(res));
""")
        self.assertEqual([out["i0"], out["i2"], out["i3"], out["i5"]], [3, 5, 0, 2])
        self.assertEqual(out["shade"], [60, 255, 255 - round(100 * 2.3)])

    def test_rect_normalization(self):
        out = self.run_js("""
const res = {};
res.fwd = mapNormRect(1, 2, 4, 6);
res.rev = mapNormRect(4, 6, 1, 2);       // 逆向きのドラッグ
res.mixed = mapNormRect(4, 2, 1, 6);
const v = {cx: 0, cy: 0, s: 100, cw: 800, ch: 600};
// 画面上で右下→左上へドラッグした矩形
const a = mapScreenToWorld(v, 600, 450), b = mapScreenToWorld(v, 200, 150);
res.drag = mapNormRect(a[0], a[1], b[0], b[1]);
res.r2 = [mapRound2(1.005), mapRound2(-0.126), mapRound2(2)];
console.log(JSON.stringify(res));
""")
        want = {"x_min": 1, "y_min": 2, "x_max": 4, "y_max": 6}
        self.assertEqual(out["fwd"], want)
        self.assertEqual(out["rev"], want)
        self.assertEqual(out["mixed"], want)
        self.assertEqual(out["drag"], {"x_min": -2, "y_min": -1.5, "x_max": 2, "y_max": 1.5})
        self.assertEqual(out["r2"][2], 2)
        self.assertAlmostEqual(out["r2"][1], -0.13, places=9)

    def test_zoom_keeps_point_and_pan(self):
        out = self.run_js("""
const v = {cx: 2, cy: -1, s: 80, cw: 640, ch: 480};
const res = {};
const before = mapScreenToWorld(v, 500, 100);
const z = mapZoomAt(v, 500, 100, 2.5, 10, 1000);
res.before = before; res.after = mapScreenToWorld(z, 500, 100); res.s = z.s;
res.clamp = [mapZoomAt(v, 0, 0, 1000, 10, 200).s, mapZoomAt(v, 0, 0, 0.0001, 10, 200).s];
// パン: 画面で右へ80px動かすと、見えている世界は左へ1m
const p = mapPanBy(v, 80, 0);
res.pan = [p.cx, p.cy];
const p2 = mapPanBy(v, 0, 80);   // 画面で下へ動かす = 世界の見える範囲は上(y+)へ
res.pan2 = [p2.cx, p2.cy];
console.log(JSON.stringify(res));
""")
        self.assertAlmostEqual(out["after"][0], out["before"][0], places=9)
        self.assertAlmostEqual(out["after"][1], out["before"][1], places=9)
        self.assertAlmostEqual(out["s"], 200, places=9)
        self.assertEqual(out["clamp"], [200, 10])
        self.assertEqual(out["pan"], [1, -1])
        self.assertEqual(out["pan2"], [2, 0])

    def test_hit_resize_move(self):
        out = self.run_js("""
const v = {cx: 0, cy: 0, s: 100, cw: 800, ch: 600};
const r = {x_min: -1, y_min: -1, x_max: 1, y_max: 1};      // 画面: x 300..500, y 200..400
const res = {};
res.nw = mapHit(v, r, 303, 204, 10);   // 左上 = (x_min, y_max)
res.se = mapHit(v, r, 498, 397, 10);
res.ne = mapHit(v, r, 500, 200, 10);
res.sw = mapHit(v, r, 300, 400, 10);
res.inside = mapHit(v, r, 400, 300, 10);
res.outside = mapHit(v, r, 50, 50, 10);
// nwを掴んで(x=-2, y=3)へ: 反対側(se=(1,-1))固定
res.rz = mapResizeRect(r, "nw", -2, 3);
// seを掴んで左上より先へ反転
res.flip = mapResizeRect(r, "se", -3, 2);
res.mv = mapMoveRect(r, 0.5, -0.25);
const pins = [{id: "a", x: 0, y: 0}, {id: "b", x: 1, y: 1}];
res.pin = [mapPinAt(v, pins, 402, 303, 12), mapPinAt(v, pins, 500, 205, 12), mapPinAt(v, pins, 300, 300, 12)];
console.log(JSON.stringify(res));
""")
        self.assertEqual([out["nw"], out["se"], out["ne"], out["sw"], out["inside"], out["outside"]],
                         ["nw", "se", "ne", "sw", "move", None])
        self.assertEqual(out["rz"], {"x_min": -2, "y_min": -1, "x_max": 1, "y_max": 3})
        self.assertEqual(out["flip"], {"x_min": -3, "y_min": 1, "x_max": -1, "y_max": 2})
        self.assertEqual(out["mv"], {"x_min": -0.5, "y_min": -1.25, "x_max": 1.5, "y_max": 0.75})
        self.assertEqual([out["pin"][0] and out["pin"][0]["id"], out["pin"][1] and out["pin"][1]["id"], out["pin"][2]], ["a", "b", None])


if __name__ == "__main__":
    unittest.main()
