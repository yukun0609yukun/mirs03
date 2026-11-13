"""壁際の出発点の模擬テスト(nearwall_sim.py)(2026-10-22)。ROSは使わない。
実行: python3 -I ros2_ws/src/nokosahen_core/test/test_near_wall_sim.py   (表を見るには -v ではなく、そのまま実行)
読み方: docs/nearwall_sim.md
"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core import nearwall_sim as ns  # noqa: E402

CUR = ns.inflation_for(0.26)
SMALL = ns.inflation_for(0.21)
_CACHE = {}


def table():
    """全シナリオの計画成否と、離脱の結果(計画失敗か壁が近いシナリオだけ)。"""
    if not _CACHE:
        rows = ns.compare_table(0.21)
        esc = {}
        for r in rows:
            sc = r["sc"]
            esc[sc.name] = ns.run_escape(sc, CUR)
        _CACHE["rows"], _CACHE["esc"] = rows, esc
    return _CACHE["rows"], _CACHE["esc"]


def by_name(name):
    rows, esc = table()
    for r in rows:
        if r["sc"].name == name:
            return r, esc[name]
    raise KeyError(name)


class CostmapTests(unittest.TestCase):
    def test_inflation_values_from_nav_speed(self):
        self.assertAlmostEqual(CUR.inscribed, 0.26)
        self.assertAlmostEqual(CUR.radius, 0.57)
        self.assertAlmostEqual(CUR.k, 12.7)
        self.assertEqual(CUR.cost_at(0.0), 254)
        self.assertEqual(CUR.cost_at(0.26), 253)
        self.assertEqual(CUR.cost_at(0.60), 0)
        self.assertLess(CUR.cost_at(0.36), 253)
        self.assertAlmostEqual(CUR.cost_at(0.46), 252 * 0.2 ** 0 * 2.718281828 ** (-12.7 * 0.20), delta=2)  # 端20cmで約20

    def test_start_cell_only_cleared(self):
        # 出発点が内接帯の中でも、出発点自身だけは空き。隣に空きがあれば出られる
        cm = ns.build_costmap(ns.seg((-0.22, -2), (-0.22, 2)), CUR, (-2, -2.5, 3.5, 2.5))
        self.assertGreaterEqual(cm.at(*cm.cell(0.0, 0.0)), 253)  # 壁から0.22m=内接帯
        self.assertTrue(ns.plan(cm, (0.0, 0.0), (1.5, 0.0)).ok)

    def test_all_neighbors_blocked_fails(self):
        # 狭い囲い(箱の内側 幅0.4m)に出発点がいる: 隣が全て通行不可 → 失敗
        pts = ns.box_points(-0.2, -0.2, 0.2, 0.2)
        cm = ns.build_costmap(pts, CUR, (-2, -2.5, 3.5, 2.5))
        r = ns.plan(cm, (0.0, 0.0), (1.5, 0.0))
        self.assertFalse(r.ok)
        self.assertIn("隣8マス", r.reason)


class RegressionTests(unittest.TestCase):
    def test_edge_20cm_or_more_plans_ok(self):
        rows, _ = table()
        checked = 0
        for r in rows:
            sc = r["sc"]
            if sc.kind in ("back", "left", "right", "front", "corner", "ushape") and sc.edge_m >= 0.20:
                checked += 1
                self.assertTrue(r["cur"].ok, f"{sc.name}: {r['cur'].reason}")
        self.assertGreaterEqual(checked, 12)

    def test_corridor_100cm_or_more_plans_ok(self):
        for w in (1.00, 1.20):
            sc = ns.corridor(w)
            self.assertTrue(ns.plan_scenario(sc, CUR).ok, sc.name)

    def test_corridor_60cm_fails_now_but_small_estimate_passes(self):
        r, _ = by_name("通路 幅60cm")
        self.assertFalse(r["cur"].ok)
        self.assertTrue(r["small"].ok)

    def test_edge_10cm_plans_or_escapes_or_is_listed(self):
        """端10cm: 計画が成功するか、離脱の後に成功するか、成功しない理由がある。離脱で角が壁に当たらない。"""
        rows, esc = table()
        n = 0
        for r in rows:
            sc = r["sc"]
            if sc.kind == "real" or abs(sc.edge_m - 0.10) > 1e-9 or sc.kind == "corridor":
                continue
            n += 1
            e = esc[sc.name]
            if r["cur"].ok:
                continue
            self.assertTrue(e.status == "ok" or e.reason, sc.name)
        self.assertGreaterEqual(n, 6)
        # 離脱は、どのシナリオでも角を壁に当てない
        for name, e in esc.items():
            self.assertFalse(e.collided, f"{name}: {e.reason}")

    def test_edge_10cm_single_walls_escape_ok(self):
        for side in ("back", "left", "right", "front"):
            r, e = by_name(f"{side}壁 端10cm")
            self.assertTrue(r["cur"].ok)
            self.assertEqual(e.status, "ok", f"{side}: {e.reason}")
            self.assertLessEqual(e.t_clear, 4.0)
            self.assertGreater(e.min_clearance_m, 0.0)

    def test_ushape_10cm_cannot_escape_reason_listed(self):
        _, e = by_name("コの字の奥 端10cm")
        self.assertEqual(e.status, "failed")
        self.assertTrue(e.reason)

    def test_real_log_back_0_36m(self):
        """実機のログ: 後ろの壁まで0.36m(LiDAR中心)。素直なモデルでは計画成功、離脱は2秒で壁から離れる。"""
        r, e = by_name("実機ログ back0.36m(素直)")
        self.assertTrue(r["cur"].ok)
        self.assertEqual(e.status, "ok")
        self.assertLessEqual(e.t_clear, 2.5)
        self.assertFalse(e.collided)
        # 実機で計画が失敗した状況を、「コストマップ上で壁が20cm近く見える」として再現: 失敗 → 離脱の後に成功
        r, e = by_name("実機ログ back0.36m(壁20cm近く見える)")
        self.assertFalse(r["cur"].ok)
        self.assertIn("隣8マス", r["cur"].reason)
        self.assertEqual(e.status, "ok", e.reason)
        self.assertIsNotNone(e.t_plan_ok)
        self.assertGreater(e.t_plan_ok, 0.0)
        self.assertLessEqual(e.t_plan_ok, 2.0)
        self.assertFalse(e.collided)
        # 計画用だけ機体を小さく見積もれば、離脱なしでも成功
        self.assertTrue(r["small"].ok)

    def test_real_log_all_sides_escape_and_plan(self):
        for side in ("back", "left", "right", "front"):
            r, e = by_name(f"実機ログ {side}0.36m(壁20cm近く見える)")
            self.assertFalse(r["cur"].ok, side)
            self.assertEqual(e.status, "ok", f"{side}: {e.reason}")
            self.assertFalse(e.collided)
            self.assertLessEqual(e.t_plan_ok, 3.5)


class CompareTests(unittest.TestCase):
    def test_small_estimate_never_worse(self):
        rows, esc = table()
        print("\n=== 壁際の出発点: 計画成否と離脱(現状=半幅0.26 / 計画用だけ小さく=0.21) ===")
        print(ns.format_table(rows, esc))
        cur_ok = sum(1 for r in rows if r["cur"].ok)
        small_ok = sum(1 for r in rows if r["small"].ok)
        n = len(rows)
        print(f"\n計画成功: 現状 {cur_ok}/{n} ({100 * cur_ok // n}%)  小さく見積もり {small_ok}/{n} ({100 * small_ok // n}%)")
        plan_ok = {r["sc"].name: r["cur"].ok for r in rows}
        unescapable = [(n_, e.reason) for n_, e in esc.items() if e.status == "failed"]
        print("\n--- 離脱できない(または離れても計画が成功しない)シナリオと理由 ---")
        for name, reason in unescapable:
            print(f"  {name}: {reason}" + ("  [計画は最初から成功=実害なし]" if plan_ok[name] else "  [計画も失敗=要注意]"))
        worse = [r["sc"].name for r in rows if r["cur"].ok and not r["small"].ok]
        self.assertEqual(worse, [], "小さく見積もった方が悪化したシナリオがある")
        self.assertGreaterEqual(small_ok, cur_ok)


if __name__ == "__main__":
    unittest.main(verbosity=1)
