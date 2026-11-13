"""ダッシュボード地図ビュー(map_view.py)の単体テスト。nodeが無ければJS部分はスキップ。

地図クリック=ロボットの目的地指定なので、画面座標<->世界座標の変換が描画と
逆変換になっていること(往復で元に戻る・ズーム/全体表示後も同じ)を確認する。

実行: python ros2_ws/src/nokosahen_core/test/test_map_view.py
"""

import ast
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
from nokosahen_core.map_view import MAP_VIEW_JS, MAP_VIEW_MATH_JS  # noqa: E402

NODE = shutil.which("node")


def run_node(body: str):
    src = MAP_VIEW_MATH_JS + "\n" + body
    with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False, encoding="utf-8") as f:
        f.write(src)
        path = f.name
    r = subprocess.run([NODE, path], capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


def page_html(filename: str) -> str:
    """rclpy無しでノードのHTML定数を取り出し、プレースホルダを展開する。"""
    tree = ast.parse((PKG / "nokosahen_core" / filename).read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "HTML":
            call = node.value
            while isinstance(call, ast.Call):
                call = call.func.value  # .replace(...)の連鎖を遡る
            return call.value.replace("__MAP_VIEW_JS__", MAP_VIEW_JS)
    raise AssertionError("HTML not found")


@unittest.skipIf(NODE is None, "node not installed")
class MapViewMath(unittest.TestCase):
    def test_round_trip_world_pixel_world(self):
        out = run_node("""
        const W = 480, H = 600;
        const pts = [[0,0],[1.5,-2.25],[-6.1,4.7],[11.9,12.3]];
        let worst = 0;
        const views = [
          {cx:0, cy:0, scale:40},
          {cx:-3.3, cy:5.1, scale:17.5},
          mapFitView({minX:-2,maxX:9,minY:-1,maxY:10}, W, H, 1.0),
        ];
        // ズーム・パン後のビューでも往復が一致すること
        views.push(mapZoomAt(views[1], W, H, 123, 456, 1.7));
        views.push(mapZoomAt(views[2], W, H, 10, 590, 0.4));
        for (const v of views) for (const [x,y] of pts) {
          const p = mapViewWorldToPixel(v, W, H, x, y);
          const q = mapViewPixelToWorld(v, W, H, p[0], p[1]);
          worst = Math.max(worst, Math.abs(q[0]-x), Math.abs(q[1]-y));
        }
        console.log(JSON.stringify({worst}));
        """)
        self.assertLess(out["worst"], 1e-9)

    def test_known_axes(self):
        out = run_node("""
        const v = {cx:2, cy:3, scale:10};
        const c = mapViewWorldToPixel(v, 200, 100, 2, 3);          // 画面中心
        const r = mapViewWorldToPixel(v, 200, 100, 3, 3);          // +x = 右
        const u = mapViewWorldToPixel(v, 200, 100, 2, 4);          // +y = 上(pxのyは小さく)
        console.log(JSON.stringify({c, r, u}));
        """)
        self.assertEqual(out["c"], [100, 50])
        self.assertEqual(out["r"], [110, 50])
        self.assertEqual(out["u"], [100, 40])

    def test_zoom_keeps_point_under_cursor(self):
        out = run_node("""
        const W = 500, H = 400, v = {cx:1, cy:-2, scale:25};
        const before = mapViewPixelToWorld(v, W, H, 321, 77);
        const z = mapZoomAt(v, W, H, 321, 77, 3.3);
        const after = mapViewPixelToWorld(z, W, H, 321, 77);
        console.log(JSON.stringify({d: Math.hypot(before[0]-after[0], before[1]-after[1]), s: z.scale / v.scale}));
        """)
        self.assertLess(out["d"], 1e-9)
        self.assertAlmostEqual(out["s"], 3.3, places=9)

    def test_fit_contains_bounds_with_margin_and_uses_panel(self):
        # 実機の状況: 238x254セル@0.05m(約12x13m)のうち、約8x7.5mだけ探索済み
        out = run_node("""
        const W = 480, H = 560;
        const b = {minX:-3, maxX:5, minY:-2, maxY:5.5};
        const v = mapFitView(b, W, H, 1.0);
        const tl = mapViewWorldToPixel(v, W, H, b.minX, b.maxY);
        const br = mapViewWorldToPixel(v, W, H, b.maxX, b.minY);
        // 全グリッド(12x13m)を全面に引き伸ばす従来方式の縮尺(px/m)
        const old = Math.min(W / (238*0.05), H / (254*0.05));
        console.log(JSON.stringify({tl, br, scale: v.scale, old}));
        """)
        self.assertGreaterEqual(out["tl"][0], 0)
        self.assertGreaterEqual(out["tl"][1], 0)
        self.assertLessEqual(out["br"][0], 480)
        self.assertLessEqual(out["br"][1], 560)
        self.assertGreater(out["scale"], out["old"])

    def test_known_bounds_of(self):
        out = run_node("""
        const m = {width:6, height:4, resolution:0.5, origin_x:-1, origin_y:-2};
        const bytes = new Uint8Array(24);                 // 全部未知(0)
        bytes[1*6+2] = 1; bytes[2*6+4] = 101;             // (col2,row1) と (col4,row2)
        const b = mapKnownBoundsOf(m, bytes);
        const none = mapKnownBoundsOf(m, new Uint8Array(24));
        console.log(JSON.stringify({b, none}));
        """)
        self.assertEqual(out["b"], {"minX": 0.0, "maxX": 1.5, "minY": -1.5, "maxY": -0.5})
        self.assertIsNone(out["none"])


@unittest.skipIf(NODE is None, "node not installed")
class MapViewRotation(unittest.TestCase):
    """【2026-10-06】表示回転。描画・目的地クリック・範囲指定が同じ変換を通ることの検証。"""

    def test_round_trip_with_rotation(self):
        out = run_node("""
        const W = 480, H = 600;
        const pts = [[0,0],[1.5,-2.25],[-6.1,4.7],[11.9,12.3]];
        let worst = 0;
        for (const deg of [0, 17.5, -33, 90, 180, -179.5, 123.4]) {
          const rot = deg * Math.PI / 180;
          const views = [
            {cx:0, cy:0, scale:40, rot},
            {cx:-3.3, cy:5.1, scale:17.5, rot},
            mapFitView({minX:-2,maxX:9,minY:-1,maxY:10}, W, H, 1.0, rot),
          ];
          views.push(mapZoomAt(views[1], W, H, 123, 456, 1.7));
          views.push(mapPanBy(views[0], 31, -12));
          for (const v of views) for (const [x,y] of pts) {
            const p = mapViewWorldToPixel(v, W, H, x, y);
            const q = mapViewPixelToWorld(v, W, H, p[0], p[1]);
            worst = Math.max(worst, Math.abs(q[0]-x), Math.abs(q[1]-y));
          }
        }
        console.log(JSON.stringify({worst}));
        """)
        self.assertLess(out["worst"], 1e-9)

    def test_rot_zero_matches_old_behavior_and_rot_90_axes(self):
        out = run_node("""
        const v0 = {cx:2, cy:3, scale:10};                 // rotなし=従来
        const v90 = {cx:2, cy:3, scale:10, rot: Math.PI/2};
        const a = mapViewWorldToPixel(v0, 200, 100, 3, 3);
        const b = mapViewWorldToPixel(v90, 200, 100, 3, 3);   // +x は画面の上へ
        const c = mapViewWorldToPixel(v90, 200, 100, 2, 4);   // +y は画面の左へ
        console.log(JSON.stringify({a, b, c}));
        """)
        self.assertEqual(out["a"], [110, 50])
        self.assertAlmostEqual(out["b"][0], 100, places=9)
        self.assertAlmostEqual(out["b"][1], 40, places=9)
        self.assertAlmostEqual(out["c"][0], 90, places=9)
        self.assertAlmostEqual(out["c"][1], 50, places=9)

    def test_zoom_and_pan_keep_point_under_pointer_when_rotated(self):
        out = run_node("""
        const W = 500, H = 400, v = {cx:1, cy:-2, scale:25, rot: 0.7};
        const before = mapViewPixelToWorld(v, W, H, 321, 77);
        const z = mapZoomAt(v, W, H, 321, 77, 3.3);
        const after = mapViewPixelToWorld(z, W, H, 321, 77);
        // パン: 指の下の世界点が指と一緒に動く
        const wp = mapViewPixelToWorld(v, W, H, 200, 150);
        const pv = mapPanBy(v, 40, -25);
        const wq = mapViewPixelToWorld(pv, W, H, 240, 125);
        console.log(JSON.stringify({d: Math.hypot(before[0]-after[0], before[1]-after[1]),
          e: Math.hypot(wp[0]-wq[0], wp[1]-wq[1]), rot: z.rot}));
        """)
        self.assertLess(out["d"], 1e-9)
        self.assertLess(out["e"], 1e-9)
        self.assertEqual(out["rot"], 0.7)

    def test_fit_rotated_contains_all_corners(self):
        out = run_node("""
        const W = 480, H = 360, b = {minX:-3, maxX:5, minY:-2, maxY:5.5};
        let worstMargin = 1e9;
        for (const deg of [0, 30, -45, 90, 150]) {
          const v = mapFitView(b, W, H, 1.0, deg * Math.PI / 180);
          for (const [x,y] of [[b.minX,b.minY],[b.maxX,b.minY],[b.maxX,b.maxY],[b.minX,b.maxY]]) {
            const p = mapViewWorldToPixel(v, W, H, x, y);
            worstMargin = Math.min(worstMargin, p[0], p[1], W - p[0], H - p[1]);
          }
        }
        console.log(JSON.stringify({worstMargin}));
        """)
        self.assertGreaterEqual(out["worstMargin"], -1e-6)

    def test_known_extents_rot_is_tighter_for_rotated_room(self):
        out = run_node("""
        // 45度傾いた細長い壁(セル列)。回転を合わせれば外接矩形が細くなる。
        const m = {width:60, height:60, resolution:0.1, origin_x:0, origin_y:0};
        const bytes = new Uint8Array(60*60);
        for (let i = 5; i < 55; i++) bytes[i*60 + i] = 101;
        const e0 = mapKnownExtentsRot(m, bytes, 0);
        const e45 = mapKnownExtentsRot(m, bytes, -Math.PI/4);
        console.log(JSON.stringify({h0: e0.maxV - e0.minV, h45: e45.maxV - e45.minV}));
        """)
        self.assertLess(out["h45"], out["h0"] / 3)

    def test_image_matrix_places_grid_corners_like_world_to_pixel(self):
        out = run_node("""
        const W = 400, H = 300;
        const m = {width:80, height:50, resolution:0.05, origin_x:-1.2, origin_y:-0.7};
        let worst = 0;
        for (const deg of [0, 25, -90, 140]) {
          const v = {cx:0.3, cy:0.1, scale:60, rot: deg * Math.PI / 180};
          const [a,b,c,d,e,f] = mapImageMatrix(v, W, H, m);
          // 画像の(i,j)セル角 -> 世界(origin_x + i*res, origin_y + (H-j)*res)
          for (const [i, j] of [[0,0],[80,0],[80,50],[0,50],[13,7]]) {
            const px = a*i + c*j + e, py = b*i + d*j + f;
            const wx = m.origin_x + i*m.resolution, wy = m.origin_y + (m.height - j)*m.resolution;
            const q = mapViewWorldToPixel(v, W, H, wx, wy);
            worst = Math.max(worst, Math.abs(px - q[0]), Math.abs(py - q[1]));
          }
        }
        console.log(JSON.stringify({worst}));
        """)
        self.assertLess(out["worst"], 1e-9)

    def test_boundary_from_view_corners_is_axis_aligned_in_view(self):
        out = run_node("""
        const W = 500, H = 400;
        let worst = 0, ok = true;
        for (const deg of [0, 30, -45, 90, -170]) {
          const rot = deg * Math.PI / 180;
          const v = {cx:1.0, cy:-2.0, scale:30, rot};
          // 画面上の対角2点(ピクセル)をクリックした想定
          const pa = [120, 90], pb = [380, 300];
          const wa = mapViewPixelToWorld(v, W, H, pa[0], pa[1]);
          const wb = mapViewPixelToWorld(v, W, H, pb[0], pb[1]);
          const r = boundaryRectFromViewCorners(wa, wb, rot);
          // 4隅を画面へ戻すと、クリックした矩形と一致する(軸平行に見える)
          const corners = boundaryCornersOf(r).map(([x,y]) => mapViewWorldToPixel(v, W, H, x, y));
          const xs = corners.map(p => p[0]), ys = corners.map(p => p[1]);
          const box = [Math.min(...xs), Math.max(...xs), Math.min(...ys), Math.max(...ys)];
          worst = Math.max(worst, Math.abs(box[0]-120), Math.abs(box[1]-380), Math.abs(box[2]-90), Math.abs(box[3]-300));
          for (const p of corners) {
            const onX = Math.abs(p[0]-120) < 1e-6 || Math.abs(p[0]-380) < 1e-6;
            const onY = Math.abs(p[1]-90) < 1e-6 || Math.abs(p[1]-300) < 1e-6;
            if (!(onX && onY)) ok = false;
          }
          let dy = Math.abs(r.yaw + rot) % (2*Math.PI);
          if (dy > 1e-9 && Math.abs(dy - 2*Math.PI) > 1e-9) ok = false;
        }
        console.log(JSON.stringify({worst, ok}));
        """)
        self.assertLess(out["worst"], 1e-6)
        self.assertTrue(out["ok"])

    def test_boundary_state_forms_and_corner_resize(self):
        out = run_node("""
        const old = boundaryRectOf({min_x:-1, max_x:3, min_y:2, max_y:6});
        const rect = {center_x:1, center_y:2, width:4, height:2, yaw: 0.5};
        const cs = boundaryCornersOf(rect);
        // 角2(+,+)を外側へ動かす: 対角(角0)は動かない、向きは保たれる
        const target = [cs[2][0] + 0.8, cs[2][1] + 0.3];
        const r2 = boundaryResizeCorner(rect, 2, target);
        const cs2 = boundaryCornersOf(r2);
        const fixed = Math.hypot(cs2[0][0]-cs[0][0], cs2[0][1]-cs[0][1]);
        const moved = Math.hypot(cs2[2][0]-target[0], cs2[2][1]-target[1]);
        console.log(JSON.stringify({old, fixed, moved, yaw: r2.yaw, w: r2.width, h: r2.height}));
        """)
        self.assertEqual(out["old"], {"center_x": 1, "center_y": 4, "width": 4, "height": 4, "yaw": 0})
        self.assertLess(out["fixed"], 1e-9)
        self.assertLess(out["moved"], 1e-9)
        self.assertEqual(out["yaw"], 0.5)
        self.assertGreater(out["w"], 4.0)

    def test_boundary_corner_order_matches_python(self):
        """JSのboundaryCornersOfとサーバーのrect_cornersは同じ順・同じ座標。"""
        from nokosahen_core import map_orientation as mo
        rect = mo.make_rect(1.5, -2.0, 4.0, 2.5, 0.6)
        py = mo.rect_corners(rect)
        js = run_node("""
        console.log(JSON.stringify(boundaryCornersOf({center_x:1.5, center_y:-2.0, width:4.0, height:2.5, yaw:0.6})));
        """)
        for (px, py_), (jx, jy) in zip(py, js):
            self.assertAlmostEqual(px, jx, places=9)
            self.assertAlmostEqual(py_, jy, places=9)

    def test_pages_have_rotation_ui(self):
        for fn in ("full_dashboard_node.py", "stage2_dashboard_node.py"):
            html = page_html(fn)
            for needle in ("mapRotSlider", "mapRot90Btn", "mapRot0Btn", "mapRotAutoBtn",
                           "/api/map_view/estimate", "drawBoundaryShape", "boundaryRectFromViewCorners",
                           "drawMapCompass"):
                self.assertIn(needle, html, f"{fn}: {needle}")
            # 旧: 軸平行のmin/maxだけをPOSTする実装が残っていないこと
            self.assertNotIn("JSON.stringify({ min_x, max_x, min_y, max_y })", html)


class PageHtml(unittest.TestCase):
    def test_full_page_has_no_checkout_ui_and_map_hooks(self):
        html = page_html("full_dashboard_node.py")
        for needle in ("s-checkout", "checkoutRow", "checkout-tile", "/api/checkout_cam",
                       "<h2>決済場所カメラ", "まだどの決済カメラも", "__MAP_VIEW_JS__"):
            self.assertNotIn(needle, html)
        self.assertIn("mapViewPixelToWorld", html)
        self.assertIn("mapClickSuppressUntil", html)

    def test_stage2_page_uses_shared_map_view(self):
        html = page_html("stage2_dashboard_node.py")
        self.assertNotIn("__MAP_VIEW_JS__", html)
        self.assertIn("mapViewPixelToWorld", html)

    def test_embedded_scripts_pass_node_check(self):
        if NODE is None:
            self.skipTest("node not installed")
        import re
        for fn in ("full_dashboard_node.py", "stage2_dashboard_node.py"):
            for i, body in enumerate(re.findall(r"<script>(.*?)</script>", page_html(fn), re.S)):
                with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False, encoding="utf-8") as f:
                    f.write(body)
                r = subprocess.run([NODE, "--check", f.name], capture_output=True, text=True)
                self.assertEqual(r.returncode, 0, f"{fn} script#{i}: {r.stderr}")


if __name__ == "__main__":
    unittest.main()
