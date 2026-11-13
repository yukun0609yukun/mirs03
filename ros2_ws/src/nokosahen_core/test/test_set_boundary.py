"""木のノード set_boundary と、スケジュール由来の木(source:"schedule")の「実行中(armed)」の単体テスト(2026-10-23)。

純Python(rclpy不要)。実機・ROSには一切触れない。docs/schedule.md。
- set_boundary の検証(mode必須、setは矩形必須、releaseは矩形不可、範囲外・NaN・逆転の拒否)
- 評価: set/releaseがTreeStateのboundaryに出る、失敗した枝では取り消される、評価されなくなれば消える(=元の範囲へ戻る)
- 最小保持時間でも範囲の変化は方針の切り替えとして扱われる
- source:"schedule" の木は、armedにするまで動かない(起動直後は止まっている)。run_on_startupで自動armed
- スケジュール由来の木はノード上限が大きい。通常の木は従来の200のまま

実行: python3 -I ros2_ws/src/nokosahen_core/test/test_set_boundary.py
"""

import datetime
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core import behavior_rules as br  # noqa: E402
from nokosahen_core import behavior_tree as bt  # noqa: E402


def N(typ, children=None, **params):
    d = {"type": typ}
    if params:
        d["params"] = params
    if children is not None:
        d["children"] = children
    return d


def DOC(root, **kw):
    d = {"version": 1, "min_hold_seconds": 0, "root": root}
    d.update(kw)
    return d


RECT = dict(mode="set", x_min=0.0, y_min=-1.0, x_max=4.0, y_max=2.0)


class Clk:
    def __init__(self):
        self.dt = datetime.datetime(2026, 10, 6, 12, 0)
        self.m = 1000.0

    def adv(self, sec):
        self.dt += datetime.timedelta(seconds=sec)
        self.m += sec


def runner(doc, clk=None):
    clk = clk or Clk()
    r = bt.TreeRunner("", clock=lambda: clk.dt, mono=lambda: clk.m)
    errors, _w, norm = bt.validate_document(doc)
    assert not errors, errors
    r.set_document(norm)
    return r, clk


def ev(r, **kw):
    return r.evaluate(br.Context(), None, kw.get("safety"), None)


class ValidationTests(unittest.TestCase):
    def errs(self, **params):
        return bt.validate_document(DOC(N("set_boundary", **params)))[0]

    def test_valid_forms(self):
        self.assertEqual(self.errs(**RECT), [])
        self.assertEqual(self.errs(mode="release"), [])
        self.assertEqual(self.errs(mode="release", for_seconds=30.0), [])

    def test_mode_required_and_enum(self):
        self.assertTrue(self.errs())
        self.assertTrue(self.errs(mode="keep"))

    def test_set_needs_full_rect(self):
        for missing in ("x_min", "y_min", "x_max", "y_max"):
            p = dict(RECT)
            del p[missing]
            self.assertTrue(self.errs(**p), missing)

    def test_release_rejects_rect(self):
        self.assertTrue(self.errs(mode="release", x_min=0.0))

    def test_reversed_or_empty_rect_rejected(self):
        self.assertTrue(self.errs(**dict(RECT, x_min=5.0)))
        self.assertTrue(self.errs(**dict(RECT, y_max=-1.0)))
        self.assertTrue(self.errs(**dict(RECT, y_max=-2.0)))

    def test_out_of_range_and_non_finite_rejected(self):
        self.assertTrue(self.errs(**dict(RECT, x_max=1e9)))
        self.assertTrue(self.errs(**dict(RECT, x_max=float("nan"))))
        self.assertTrue(self.errs(**dict(RECT, x_max=float("inf"))))
        self.assertTrue(self.errs(**dict(RECT, x_max="4")))

    def test_node_is_listed_in_registry_meta(self):
        types = {t["type"]: t for t in bt.registry_meta()["types"]}
        self.assertEqual(types["set_boundary"]["kind"], "action")


class EvaluationTests(unittest.TestCase):
    def test_set_and_release_are_output(self):
        r, _ = runner(DOC(N("set_boundary", **RECT)))
        st = ev(r)
        self.assertEqual(st.boundary, {"mode": "set", "x_min": 0.0, "y_min": -1.0, "x_max": 4.0, "y_max": 2.0})
        r, _ = runner(DOC(N("sequence", [N("set_boundary", mode="release"), N("hold_stop")])))
        st = ev(r)
        self.assertEqual(st.boundary, {"mode": "release"})
        self.assertEqual(st.policies, ("stay",))
        self.assertEqual(r.status_dict(st)["boundary"], {"mode": "release"})

    def test_no_node_means_no_override(self):
        r, _ = runner(DOC(N("quiet")))
        self.assertIsNone(ev(r).boundary)

    def test_failed_branch_boundary_is_cancelled(self):
        r, _ = runner(DOC(N("selector", [
            N("sequence", [N("set_boundary", mode="release"), N("crowd_is", levels=["crowded"])]),
            N("quiet"),
        ])))
        st = ev(r)
        self.assertIsNone(st.boundary)
        self.assertEqual(st.policies, ("quiet",))

    def test_boundary_goes_away_when_not_evaluated_any_more(self):
        clk = Clk()
        r, _ = runner(DOC(N("selector", [
            N("sequence", [N("time_between", range="12:00-13:00"), N("set_boundary", mode="release")]),
            N("normal"),
        ])), clk)
        self.assertEqual(ev(r).boundary, {"mode": "release"})
        clk.adv(3600)  # 13:00
        self.assertIsNone(ev(r).boundary)

    def test_later_node_wins(self):
        r, _ = runner(DOC(N("sequence", [N("set_boundary", mode="release"), N("set_boundary", **RECT)])))
        self.assertEqual(ev(r).boundary["mode"], "set")

    def test_boundary_change_counts_as_a_change_and_respects_min_hold(self):
        clk = Clk()
        root = N("selector", [
            N("sequence", [N("time_between", range="12:00-12:10"), N("set_boundary", mode="release")]),
            N("normal"),
        ])
        r, _ = runner(DOC(root, min_hold_seconds=60), clk)
        st = ev(r)
        a0 = st.activation
        clk.adv(11 * 60)  # 範囲外れ。保持時間は過ぎている
        st = ev(r)
        self.assertIsNone(st.boundary)
        self.assertGreater(st.activation, a0)
        self.assertTrue(st.changed)


class ScheduleArmedTests(unittest.TestCase):
    def write(self, d, doc):
        p = os.path.join(d, "tree.json")
        with open(p, "w", encoding="utf-8") as f:
            json.dump(doc, f)
        return p

    def runner_for_file(self, path):
        clk = Clk()
        r = bt.TreeRunner(path, clock=lambda: clk.dt, mono=lambda: clk.m)
        r.check_reload()
        return r

    def test_schedule_tree_is_idle_until_armed(self):
        with tempfile.TemporaryDirectory() as d:
            r = self.runner_for_file(self.write(d, DOC(N("quiet"), source="schedule")))
            self.assertTrue(r.is_schedule)
            self.assertFalse(r.user_active)  # 起動したら止まっている
            r.arm_schedule(True)
            self.assertTrue(r.user_active)
            r.arm_schedule(False)
            self.assertFalse(r.user_active)

    def test_run_on_startup_arms_automatically_once(self):
        with tempfile.TemporaryDirectory() as d:
            path = self.write(d, DOC(N("quiet"), source="schedule", run_on_startup=True))
            r = self.runner_for_file(path)
            self.assertTrue(r.user_active)
            r.arm_schedule(False)
            self.write(d, DOC(N("quiet"), source="schedule", run_on_startup=True))  # 書き換え直しても自動では再開しない
            r.request_reload()
            r.check_reload()
            self.assertFalse(r.user_active)

    def test_run_on_startup_does_not_arm_a_disabled_schedule(self):
        with tempfile.TemporaryDirectory() as d:
            r = self.runner_for_file(self.write(d, DOC(N("quiet"), source="schedule", run_on_startup=True, enabled=False)))
            self.assertFalse(r.user_active)

    def test_normal_user_tree_is_unaffected(self):
        with tempfile.TemporaryDirectory() as d:
            r = self.runner_for_file(self.write(d, DOC(N("quiet"))))
            self.assertFalse(r.is_schedule)
            self.assertTrue(r.user_active)
            self.assertEqual(r.status_dict(None)["source"], None)

    def test_status_reports_source_and_armed(self):
        with tempfile.TemporaryDirectory() as d:
            r = self.runner_for_file(self.write(d, DOC(N("quiet"), source="schedule")))
            s = r.status_dict(None)
            self.assertEqual((s["source"], s["schedule_armed"]), ("schedule", False))

    def test_source_and_run_on_startup_validation(self):
        self.assertTrue(bt.validate_document(DOC(N("quiet"), source="other"))[0])
        self.assertTrue(bt.validate_document(DOC(N("quiet"), source="schedule", run_on_startup="yes"))[0])
        self.assertTrue(bt.validate_document(DOC(N("quiet"), run_on_startup=True))[0])
        errs, _w, norm = bt.validate_document(DOC(N("quiet"), source="schedule"))
        self.assertEqual(errs, [])
        self.assertEqual((norm["source"], norm["run_on_startup"]), ("schedule", False))
        self.assertNotIn("source", bt.validate_document(DOC(N("quiet")))[2])

    def test_node_limit_is_larger_only_for_schedule_trees(self):
        many = N("selector", [N("quiet") for _ in range(bt.MAX_NODES + 50)])
        self.assertTrue(bt.validate_document(DOC(many))[0])
        self.assertEqual(bt.validate_document(DOC(many, source="schedule"))[0], [])
        too_many = N("selector", [N("quiet") for _ in range(bt.MAX_SCHEDULE_NODES)])
        self.assertTrue(bt.validate_document(DOC(too_many, source="schedule"))[0])


if __name__ == "__main__":
    unittest.main()
