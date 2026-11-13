"""スケジュール(schedule_plan.py)の検証・要約・コンパイル・時刻ごとの評価の単体テスト(2026-10-23)。

純Python(rclpy不要)。実機・ROSには一切触れない。docs/schedule.md。
- 検証: 未知のキー・範囲外・時刻不正・pin_id必須・NaN/Inf・項目上限50・id重複・曜日
- 時間の重なり(同じ曜日だけ)は警告、上の項目が優先。endの省略は「同じ曜日で次に始まる項目」/24:00
- コンパイル結果が既存の木の検証(behavior_tree.validate_document)を通り、使うノードが決まったものだけ
- 既存の木のエンジンに時刻を与えて評価: 時刻ごとに期待の方針・範囲(set/release)が選ばれる。日付またぎ・曜日
- 日本語の要約
- 実行時の状態(いまの項目・次の項目)

実行: python3 -I ros2_ws/src/nokosahen_core/test/test_schedule_plan.py
"""

import copy
import datetime
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core import behavior_rules as br  # noqa: E402
from nokosahen_core import behavior_tree as bt  # noqa: E402
from nokosahen_core import schedule_plan as sp  # noqa: E402

PIN_A = {"type": "pin", "pin_id": "pinA", "pin_name": "ピンA(入口横)"}
RECT = {"x_min": 0.0, "y_min": -1.0, "x_max": 4.0, "y_max": 2.0}


def E(eid, start, end=None, dest=None, policy="stay", boundary=None, weekdays=None, **state):
    e = {"id": eid, "start": start, "destination": dest or {"type": "stay"}, "state": dict({"policy": policy}, **state)}
    if end:
        e["end"] = end
    if boundary:
        e["boundary"] = boundary
    if weekdays is not None:
        e["weekdays"] = weekdays
    return e


def S(*entries, **kw):
    d = {"version": 1, "name": "テスト", "entries": list(entries)}
    d.update(kw)
    return d


def norm_of(doc):
    errors, warnings, norm = sp.validate_schedule(doc)
    assert not errors, errors
    return norm


# 2026-10-05は月曜
def at(h, m, day=5):
    return datetime.datetime(2026, 10, day, h, m)


class Clk:
    def __init__(self, dt):
        self.dt = dt
        self.m = 1000.0


def evaluate(doc, when, rule_state=None):
    """スケジュールをコンパイルして既存の木のエンジンに時刻を与え、1回評価する。"""
    tree = sp.compile_schedule(norm_of(doc))
    errors, _w, tnorm = bt.validate_document(tree)
    assert not errors, errors
    clk = Clk(when)
    r = bt.TreeRunner("", clock=lambda: clk.dt, mono=lambda: clk.m)
    r.set_document(tnorm)
    return r.evaluate(br.Context(), rule_state, {}, None), tnorm


def rule_state(policies=("quiet",)):
    now = at(12, 0)
    return br.RuleState(
        active_rule_id="r1", active_rule_name="ルール1", policies=tuple(policies), params={},
        effects=br.effects_for(tuple(policies), {}), since=now, activation=1, changed=False,
        next_change_at=None, load_error=None, rule_count=1, enabled_rule_count=1, rules_file="", now=now,
        min_hold_seconds=0.0)


class ValidationTests(unittest.TestCase):
    def err(self, doc):
        errors, _w, norm = sp.validate_schedule(doc)
        self.assertTrue(errors, doc)
        self.assertIsNone(norm)
        return errors[0]

    def test_valid_minimal_and_full(self):
        norm = norm_of(S(E("a", "09:00")))
        self.assertEqual(norm["entries"][0]["boundary"], {"mode": "keep"})
        self.assertFalse(norm["run_on_startup"])
        full = S(
            E("lunch", "12:00", "13:00", dest=PIN_A, policy="quiet",
              boundary={"mode": "set", "rect": RECT}, weekdays=["fri", "mon"], announce_interval_seconds=30,
              speed_scale=0.5),
            run_on_startup=True)
        full["entries"][0]["note"] = "昼"
        norm = norm_of(full)
        self.assertEqual(norm["entries"][0]["weekdays"], ["mon", "fri"])
        self.assertTrue(norm["run_on_startup"])

    def test_top_level(self):
        self.err([])
        self.err({"entries": []})
        self.err(S(E("a", "09:00"), extra=1))
        self.err(S(E("a", "09:00"), version=2))
        self.err(S(E("a", "09:00"), name=""))
        self.err(S(E("a", "09:00"), name="あ" * 61))
        self.err(S(E("a", "09:00"), name=5))
        self.err(S(E("a", "09:00"), run_on_startup="yes"))
        self.err({"version": 1, "name": "x"})
        self.err(S())

    def test_entry_keys_and_id(self):
        e = E("a", "09:00")
        e["bogus"] = 1
        self.assertIn("未知のキー", self.err(S(e)))
        for bad in ("", "a b", "あ", "x" * 33, 5, None):
            self.err(S(E(bad, "09:00")))
        self.assertIn("重複", self.err(S(E("a", "09:00"), E("a", "10:00"))))

    def test_times(self):
        for bad in ("9:00", "24:00", "12:60", "ab:cd", "12:00:00", 900, None):
            self.err(S(E("a", bad)))
        for bad in ("25:00", "24:30", "7", 3):
            self.err(S(E("a", "09:00", bad)))
        self.assertIn("同じ時刻", self.err(S(E("a", "09:00", "09:00"))))
        norm_of(S(E("a", "09:00", "24:00")))
        norm_of(S(E("a", "00:00", "24:00")))

    def test_weekdays(self):
        for bad in ([], ["monday"], "mon", [1], ["mon", 2]):
            self.err(S(E("a", "09:00", weekdays=bad)))

    def test_destination(self):
        def one(dest):
            e = E("a", "09:00")
            e["destination"] = dest
            return self.err(S(e))
        one({"type": "pin"})
        one({"type": "pin", "pin_id": ""})
        one({"type": "pin", "pin_id": "a b"})
        one({"type": "pin", "pin_id": "x" * 33})
        one({"type": "pin", "pin_id": "p", "pin_name": "あ" * 41})
        one({"type": "pin", "pin_id": "p", "extra": 1})
        one({"type": "entrance", "pin_id": "p"})
        one({"type": "nowhere"})
        one({})
        one("pin")
        e = E("a", "09:00")
        del e["destination"]
        self.err(S(e))
        norm_of(S(E("a", "09:00", dest={"type": "entrance"})))
        norm_of(S(E("a", "09:00", dest={"type": "pin", "pin_id": "p"})))

    def test_state(self):
        for bad in ({"policy": "dance"}, {}, {"policy": "stay", "x": 1}, {"policy": "stay", "announce_interval_seconds": 9},
                    {"policy": "stay", "announce_interval_seconds": 601}, {"policy": "stay", "speed_scale": 0.29},
                    {"policy": "stay", "speed_scale": 1.01}, {"policy": "stay", "speed_scale": "0.5"},
                    {"policy": "stay", "speed_scale": True}):
            e = E("a", "09:00")
            e["state"] = bad
            self.err(S(e))
        e = E("a", "09:00")
        del e["state"]
        self.err(S(e))
        norm_of(S(E("a", "09:00", announce_interval_seconds=10, speed_scale=0.3)))
        norm_of(S(E("a", "09:00", announce_interval_seconds=600, speed_scale=1.0)))

    def test_boundary(self):
        def one(b):
            self.err(S(E("a", "09:00", boundary=b)))
        one({"mode": "set"})
        one({"mode": "set", "rect": {"x_min": 0, "y_min": 0, "x_max": 1}})
        one({"mode": "set", "rect": dict(RECT, x_min=9.0)})
        one({"mode": "set", "rect": dict(RECT, y_max=-5.0)})
        one({"mode": "set", "rect": dict(RECT, extra=1)})
        one({"mode": "set", "rect": dict(RECT, x_max=1e9)})
        one({"mode": "set", "rect": dict(RECT, x_max="4")})
        one({"mode": "release", "rect": RECT})
        one({"mode": "keep", "rect": RECT})
        one({"mode": "weird"})
        one({"rect": RECT})
        one({"mode": "set", "rect": RECT, "extra": 1})

    def test_boundary_minimum_size(self):
        # 幅・高さとも0.8 m以上。1 mm四方は保存できない
        tiny = {"x_min": 0.0, "y_min": 0.0, "x_max": 0.001, "y_max": 0.001}
        self.assertIn("小さすぎ", self.err(S(E("a", "09:00", boundary={"mode": "set", "rect": tiny}))))
        narrow = {"x_min": 0.0, "y_min": 0.0, "x_max": 5.0, "y_max": 0.79}
        self.assertIn("小さすぎ", self.err(S(E("a", "09:00", boundary={"mode": "set", "rect": narrow}))))
        narrow_x = {"x_min": 0.0, "y_min": 0.0, "x_max": 0.79, "y_max": 5.0}
        self.assertIn("小さすぎ", self.err(S(E("a", "09:00", boundary={"mode": "set", "rect": narrow_x}))))
        ok = {"x_min": 0.0, "y_min": 0.0, "x_max": 0.8, "y_max": 0.8}
        norm_of(S(E("a", "09:00", boundary={"mode": "set", "rect": ok})))

    def test_nan_inf(self):
        for v in (float("nan"), float("inf"), float("-inf")):
            self.err(S(E("a", "09:00", boundary={"mode": "set", "rect": dict(RECT, x_max=v)})))
            self.err(S(E("a", "09:00", speed_scale=v)))
            self.err(S(E("a", "09:00", announce_interval_seconds=v)))
        with self.assertRaises(sp.ScheduleError):
            sp.parse_schedule_text('{"version": 1, "name": "x", "entries": [], "x": NaN}')
        with self.assertRaises(sp.ScheduleError):
            sp.parse_schedule_text("{not json")
        self.assertEqual(sp.parse_schedule_text('{"a": 1}'), {"a": 1})

    def test_note(self):
        e = E("a", "09:00")
        e["note"] = "あ" * 201
        self.err(S(e))
        e["note"] = 5
        self.err(S(e))

    def test_entry_limit_50(self):
        entries = [E("e%d" % i, "%02d:%02d" % (i * 28 // 60, i * 28 % 60), dest=PIN_A, policy="quiet",
                     boundary={"mode": "set", "rect": RECT}, weekdays=["mon", "tue"], announce_interval_seconds=30,
                     speed_scale=0.5) for i in range(50)]
        errors, _w, norm = sp.validate_schedule(S(*entries))
        self.assertEqual(errors, [])
        # 項目あたりのノードが最も多い形(日付またぎ+曜日+全部入り)でも50項目が木の上限に収まる
        worst = [E("w%d" % i, "22:00", "02:00", dest=PIN_A, policy="quiet", boundary={"mode": "set", "rect": RECT},
                   weekdays=["mon"], announce_interval_seconds=30, speed_scale=0.5) for i in range(50)]
        errors, _w, norm = sp.validate_schedule(S(*worst))
        self.assertEqual(errors, [])
        tree = sp.compile_schedule(norm)
        self.assertEqual(bt.validate_document(tree)[0], [])
        entries.append(E("extra", "23:00"))
        self.assertIn("多すぎ", self.err(S(*entries)))

    def test_does_not_mutate_input(self):
        doc = S(E("a", "09:00", dest=PIN_A, boundary={"mode": "release"}))
        before = copy.deepcopy(doc)
        sp.validate_schedule(doc)
        sp.preview(doc)
        self.assertEqual(doc, before)


class RectWarningTests(unittest.TestCase):
    MAP = {"x_min": -2.0, "y_min": -2.0, "x_max": 10.0, "y_max": 8.0}

    def norm(self, rect, mode="set"):
        b = {"mode": "set", "rect": rect} if mode == "set" else {"mode": mode}
        return norm_of(S(E("a", "09:00", boundary=b)))

    def test_inside_map_and_original_has_no_warning(self):
        n = self.norm(RECT)
        self.assertEqual(sp.rect_warnings(n, self.MAP, dict(self.MAP)), [])

    def test_outside_map_warns(self):
        n = self.norm(dict(RECT, x_max=12.0))
        w = sp.rect_warnings(n, self.MAP, None)
        self.assertEqual(len(w), 1)
        self.assertIn("地図の外", w[0])
        self.assertIn("id=a", w[0])

    def test_wider_than_original_warns_release_of_limit(self):
        n = self.norm(RECT)
        original = {"x_min": 0.5, "y_min": -0.5, "x_max": 3.0, "y_max": 1.5}
        w = sp.rect_warnings(n, self.MAP, original)
        self.assertEqual(len(w), 1)
        self.assertIn("元の範囲の制限を外します", w[0])

    def test_unavailable_info_is_omitted(self):
        n = self.norm(dict(RECT, x_max=999.0))
        self.assertEqual(sp.rect_warnings(n, None, None), [])

    def test_keep_and_release_have_no_rect_warning(self):
        for mode in ("keep", "release"):
            self.assertEqual(sp.rect_warnings(self.norm(None, mode), self.MAP, {"x_min": 0, "y_min": 0, "x_max": 1, "y_max": 1}), [])

    def test_tiny_overshoot_within_margin_is_ignored(self):
        n = self.norm(dict(RECT, x_min=-2.04))
        self.assertEqual(sp.rect_warnings(n, self.MAP, None), [])


class WindowResolutionTests(unittest.TestCase):
    def windows(self, *entries):
        return sp.resolve_windows(norm_of(S(*entries))["entries"])

    def test_missing_end_is_next_start_or_midnight(self):
        self.assertEqual(self.windows(E("a", "09:00"), E("b", "12:00"), E("c", "15:30")),
                         [(540, 720), (720, 930), (930, 1440)])

    def test_missing_end_uses_next_start_regardless_of_list_order(self):
        self.assertEqual(self.windows(E("c", "15:30"), E("a", "09:00"), E("b", "12:00")),
                         [(930, 1440), (540, 720), (720, 930)])

    def test_missing_end_only_looks_at_entries_sharing_a_weekday(self):
        w = self.windows(E("a", "09:00", weekdays=["mon"]), E("b", "10:00", weekdays=["tue"]),
                         E("c", "11:00", weekdays=["mon", "tue"]))
        self.assertEqual(w, [(540, 660), (600, 660), (660, 1440)])

    def test_explicit_end_kept_and_wrap(self):
        self.assertEqual(self.windows(E("a", "22:00", "02:00")), [(1320, 120)])

    def test_end_0000_means_2400(self):
        # end:"00:00" は "24:00" と同じ意味(保存される値も 24:00 に正規化)。開始が 00:00 でもエラーにならない
        for start, win in (("00:00", (0, 1440)), ("22:00", (1320, 1440))):
            n = norm_of(S(E("a", start, "00:00", weekdays=["mon", "tue"])))
            self.assertEqual(n["entries"][0]["end"], "24:00")
            self.assertEqual(sp.resolve_windows(n["entries"]), [win])
        n0 = norm_of(S(E("a", "00:00", "00:00")))
        n24 = norm_of(S(E("a", "00:00", "24:00")))
        self.assertEqual(n0, n24)
        self.assertEqual(sp.compile_schedule(n0), sp.compile_schedule(n24))
        # 同じ開始・終了(09:00-09:00)は従来どおりエラー
        errors, _w, _n = sp.validate_schedule(S(E("a", "09:00", "09:00")))
        self.assertTrue(errors)


class OverlapWarningTests(unittest.TestCase):
    def warns(self, *entries):
        errors, warnings, norm = sp.validate_schedule(S(*entries))
        self.assertEqual(errors, [])
        return [w for w in warnings if "重なって" in w]

    def test_overlap_warns_but_is_valid(self):
        w = self.warns(E("a", "09:00", "12:00"), E("b", "11:00", "13:00"))
        self.assertEqual(len(w), 1)
        self.assertIn("a", w[0])
        self.assertIn("b", w[0])
        self.assertIn("優先", w[0])

    def test_touching_ranges_do_not_overlap(self):
        self.assertEqual(self.warns(E("a", "09:00", "12:00"), E("b", "12:00", "13:00")), [])

    def test_different_weekdays_do_not_overlap(self):
        self.assertEqual(self.warns(E("a", "09:00", "12:00", weekdays=["mon"]), E("b", "09:00", "12:00", weekdays=["tue"])), [])

    def test_same_weekday_overlaps(self):
        w = self.warns(E("a", "09:00", "12:00", weekdays=["mon", "tue"]), E("b", "09:00", "12:00", weekdays=["tue", "wed"]))
        self.assertEqual(len(w), 1)
        self.assertIn("火", w[0])
        self.assertNotIn("月", w[0].split("曜日:")[1])

    def test_wrap_overlap_spills_into_next_day(self):
        # 月曜22:00〜翌02:00 と、火曜01:00〜03:00 は重なる。月曜の01:00〜03:00とは重ならない
        self.assertEqual(len(self.warns(E("a", "22:00", "02:00", weekdays=["mon"]), E("b", "01:00", "03:00", weekdays=["tue"]))), 1)
        self.assertEqual(self.warns(E("a", "22:00", "02:00", weekdays=["mon"]), E("b", "01:00", "03:00", weekdays=["mon"])), [])

    def test_sunday_wrap_reaches_monday(self):
        self.assertEqual(len(self.warns(E("a", "23:00", "01:00", weekdays=["sun"]), E("b", "00:00", "02:00", weekdays=["mon"]))), 1)

    def test_every_day_entry_overlaps_weekday_entry(self):
        self.assertEqual(len(self.warns(E("a", "09:00", "17:00"), E("b", "10:00", "11:00", weekdays=["sat"]))), 1)

    def test_destination_with_active_state_warns(self):
        _e, warnings, _n = sp.validate_schedule(S(E("a", "09:00", dest=PIN_A, policy="approach")))
        self.assertTrue(any("到着後" in w for w in warnings))
        _e, warnings, _n = sp.validate_schedule(S(E("a", "09:00", dest=PIN_A, policy="stay")))
        self.assertEqual(warnings, [])


class SummaryTests(unittest.TestCase):
    def test_example_from_contract(self):
        norm = norm_of(S(E("lunch", "12:00", "13:00", dest=PIN_A, policy="stay", boundary={"mode": "release"})))
        self.assertEqual(sp.summarize(norm), ["12:00〜13:00 ピンA(入口横)へ行く → 待機、範囲は解除してNav2で走行"])

    def test_variants(self):
        norm = norm_of(S(
            E("a", "09:00", "10:00", dest={"type": "entrance"}, policy="quiet", weekdays=["sat", "sun"],
              announce_interval_seconds=45, speed_scale=0.5),
            E("b", "10:00", "11:00", policy="approach", boundary={"mode": "set", "rect": RECT}),
            E("c", "22:00", "02:00", dest={"type": "pin", "pin_id": "p9"}, policy="scan"),
            E("d", "11:00", "12:00", boundary={"mode": "release"}),
        ))
        lines = sp.summarize(norm)
        self.assertEqual(lines[0], "09:00〜10:00(土日) 入口へ行く → 静か(発話なし)、呼びかけ45秒間隔、速度×0.5")
        self.assertEqual(lines[1], "10:00〜11:00 その場にいる → 接近(人に近づく)、範囲は(0,-1)〜(4,2)mに制限")
        self.assertEqual(lines[2], "22:00〜翌02:00 p9へ行く → 見回し")
        self.assertEqual(lines[3], "11:00〜12:00 その場にいる → 待機、範囲は解除")

    def test_preview_shapes(self):
        ok = sp.preview(S(E("a", "09:00", "10:00"), E("b", "09:30", "10:30")))
        self.assertTrue(ok["ok"])
        self.assertEqual(len(ok["summary"]), 2)
        self.assertEqual(ok["errors"], [])
        self.assertTrue(ok["warnings"])
        bad = sp.preview(S(E("a", "99:00")))
        self.assertFalse(bad["ok"])
        self.assertEqual(bad["summary"], [])
        self.assertTrue(bad["errors"])
        self.assertEqual(set(ok), {"ok", "summary", "errors", "warnings"})


ALLOWED_NODES = {"selector", "sequence", "time_between", "weekday_in", "goto_pin", "go_to_entrance", "quiet", "announce",
                 "approach", "scan", "speed_scale", "wait", "hold_stop", "normal", "set_boundary", "set_display", "rules_table"}


def all_types(node):
    out = {node["type"]}
    for c in node.get("children", []):
        out |= all_types(c)
    return out


class CompileTests(unittest.TestCase):
    def test_compiled_tree_passes_existing_validation_and_uses_only_known_nodes(self):
        doc = S(
            E("a", "09:00", "10:00", dest=PIN_A, policy="quiet", boundary={"mode": "release"}, weekdays=["mon"],
              announce_interval_seconds=30, speed_scale=0.5),
            E("b", "10:00", "11:00", dest={"type": "entrance"}, policy="approach", boundary={"mode": "set", "rect": RECT}),
            E("c", "22:00", "02:00", policy="scan", weekdays=["fri"]),
            E("d", "12:00", "13:00", policy="normal"),
            run_on_startup=True)
        tree = sp.compile_schedule(norm_of(doc))
        errors, warnings, norm = bt.validate_document(tree)
        self.assertEqual(errors, [])
        self.assertLessEqual(all_types(tree["root"]), ALLOWED_NODES)
        self.assertEqual(norm["source"], "schedule")
        self.assertTrue(norm["run_on_startup"])
        self.assertTrue(norm["enabled"])
        self.assertEqual(norm["min_hold_seconds"], 0.0)
        self.assertEqual(tree["root"]["children"][-1]["type"], "rules_table")  # 最後はルール表へのフォールバック
        self.assertEqual([c["name"] for c in tree["root"]["children"][:-1]],
                         ["entry:a", "entry:b", "entry:c", "entry:d"])
        self.assertFalse(bt.parse_tree_text(bt.document_text(norm)).get("node_count", 0) == 0)

    def test_run_on_startup_default_false(self):
        tree = sp.compile_schedule(norm_of(S(E("a", "09:00"))))
        self.assertFalse(tree["run_on_startup"])

    def test_json_roundtrip_through_tree_file_text(self):
        tree = sp.compile_schedule(norm_of(S(E("a", "09:00", dest=PIN_A))))
        text = bt.document_text(bt.validate_document(tree)[2])
        self.assertEqual(bt.parse_tree_text(text)["source"], "schedule")


class EvaluationTests(unittest.TestCase):
    """時刻ごとに期待の方針が選ばれる(既存の木のエンジンで評価)。"""

    DAY = S(
        E("morning", "08:00", "12:00", dest={"type": "entrance"}, policy="stay"),
        E("lunch", "12:00", "13:00", dest=PIN_A, policy="quiet", boundary={"mode": "release"}),
        E("afternoon", "13:00", "17:00", policy="approach", boundary={"mode": "set", "rect": RECT}),
    )

    def pol(self, doc, when, rs=None):
        st, _ = evaluate(doc, when, rs)
        return st

    def test_each_time_selects_expected_policy(self):
        st = self.pol(self.DAY, at(9, 0))
        self.assertEqual(st.policies, ("go_to_entrance", "stay"))
        self.assertIsNone(st.boundary)
        st = self.pol(self.DAY, at(12, 30))
        self.assertEqual(st.policies, ("goto_pin", "quiet"))
        self.assertEqual(st.effects.go_to, "pin")
        self.assertEqual((st.effects.go_to_pin_id, st.effects.go_to_pin_name), ("pinA", "ピンA(入口横)"))
        self.assertFalse(st.effects.announce_allowed)
        self.assertEqual(st.boundary, {"mode": "release"})
        st = self.pol(self.DAY, at(15, 0))
        self.assertEqual(st.policies, ("approach_enabled",))
        self.assertEqual(st.boundary, dict(RECT, mode="set"))

    def test_boundaries_of_a_window_are_half_open(self):
        self.assertEqual(self.pol(self.DAY, at(11, 59)).policies, ("go_to_entrance", "stay"))
        self.assertEqual(self.pol(self.DAY, at(12, 0)).policies, ("goto_pin", "quiet"))
        self.assertEqual(self.pol(self.DAY, at(12, 59)).policies, ("goto_pin", "quiet"))
        self.assertEqual(self.pol(self.DAY, at(13, 0)).policies, ("approach_enabled",))

    def test_outside_every_entry_falls_back_to_rules_table(self):
        st = self.pol(self.DAY, at(7, 0))  # 8時前
        self.assertEqual(st.policies, ("normal",))
        self.assertIsNone(st.boundary)
        st = self.pol(self.DAY, at(7, 0), rule_state(("quiet", "speed_scale")))
        self.assertEqual(st.policies, ("quiet", "speed_scale"))
        self.assertEqual(st.active_rule_id, "r1")
        st = self.pol(self.DAY, at(18, 0), rule_state(("stay",)))  # 17時の後
        self.assertEqual(st.policies, ("stay",))
        # 項目の中ではルール表より項目が優先
        st = self.pol(self.DAY, at(9, 0), rule_state(("quiet",)))
        self.assertEqual(st.policies, ("go_to_entrance", "stay"))

    def test_boundary_returns_to_original_when_entry_ends(self):
        self.assertEqual(self.pol(self.DAY, at(12, 59)).boundary, {"mode": "release"})
        self.assertIsNone(self.pol(self.DAY, at(7, 59)).boundary)
        # 範囲を差し替えた項目の次が"keep"の項目なら、上書きは無くなる(=元の範囲へ戻る)
        doc = S(E("a", "09:00", "10:00", boundary={"mode": "release"}), E("b", "10:00", "11:00"))
        self.assertEqual(self.pol(doc, at(9, 30)).boundary, {"mode": "release"})
        self.assertIsNone(self.pol(doc, at(10, 0)).boundary)

    def test_announce_and_speed_scale(self):
        doc = S(E("a", "09:00", "10:00", policy="normal", announce_interval_seconds=25, speed_scale=0.4))
        st = self.pol(doc, at(9, 30))
        self.assertEqual(st.params["announce_min_interval_seconds"], 25.0)
        self.assertEqual(st.effects.announce_min_interval_seconds, 25.0)
        self.assertEqual(st.effects.speed_scale, 0.4)

    def test_scan_and_approach_states(self):
        doc = S(E("a", "09:00", "10:00", policy="scan"), E("b", "10:00", "11:00", policy="approach"))
        self.assertEqual(self.pol(doc, at(9, 30)).policies, ("crowd_scan_enabled",))
        self.assertEqual(self.pol(doc, at(10, 30)).policies, ("approach_enabled",))

    def test_face_state_compiles_to_face_node(self):
        self.assertIn("face", sp.POLICIES)
        _e, _w, norm = sp.validate_schedule(S(E("a", "09:00", "10:00", policy="face")))
        doc = sp.compile_schedule(norm)
        self.assertIn("face", all_types(doc["root"]))
        if "face" not in bt.NODE_SPECS:
            self.skipTest("behavior_tree.py に face がまだ無い")
        self.assertEqual(self.pol(S(E("a", "09:00", "10:00", policy="face")), at(9, 30)).policies, ("face_enabled",))

    def test_entry_order_gives_priority_in_overlap(self):
        doc = S(E("first", "09:00", "12:00", policy="stay"), E("second", "10:00", "13:00", policy="quiet"))
        self.assertEqual(self.pol(doc, at(10, 30)).policies, ("stay",))
        self.assertEqual(self.pol(doc, at(12, 30)).policies, ("quiet",))

    def test_weekdays(self):
        doc = S(E("a", "09:00", "10:00", policy="quiet", weekdays=["mon", "wed"]))
        self.assertEqual(self.pol(doc, at(9, 30, 5)).policies, ("quiet",))   # 月
        self.assertEqual(self.pol(doc, at(9, 30, 6)).policies, ("normal",))  # 火
        self.assertEqual(self.pol(doc, at(9, 30, 7)).policies, ("quiet",))   # 水

    def test_wrap_over_midnight_every_day(self):
        doc = S(E("night", "22:00", "02:00", policy="quiet"))
        for h, m, expect in ((21, 59, ("normal",)), (22, 0, ("quiet",)), (23, 59, ("quiet",)), (0, 0, ("quiet",)),
                             (1, 59, ("quiet",)), (2, 0, ("normal",)), (12, 0, ("normal",))):
            self.assertEqual(self.pol(doc, at(h, m)).policies, expect, (h, m))

    def test_wrap_over_midnight_with_weekday_uses_start_day(self):
        doc = S(E("night", "22:00", "02:00", policy="quiet", weekdays=["fri"]))
        fri, sat, thu = 9, 10, 8  # 2026-10-09は金曜
        self.assertEqual(self.pol(doc, at(23, 0, fri)).policies, ("quiet",))
        self.assertEqual(self.pol(doc, at(1, 0, sat)).policies, ("quiet",))     # 土曜の朝1時は金曜の続き
        self.assertEqual(self.pol(doc, at(1, 0, fri)).policies, ("normal",))    # 金曜の朝1時は木曜の続き=対象外
        self.assertEqual(self.pol(doc, at(23, 0, thu)).policies, ("normal",))
        self.assertEqual(self.pol(doc, at(23, 0, sat)).policies, ("normal",))
        self.assertEqual(self.pol(doc, at(2, 0, sat)).policies, ("normal",))

    def test_sunday_night_wraps_to_monday(self):
        doc = S(E("night", "23:00", "01:00", policy="quiet", weekdays=["sun"]))
        self.assertEqual(self.pol(doc, at(23, 30, 11)).policies, ("quiet",))  # 2026-10-11は日曜
        self.assertEqual(self.pol(doc, at(0, 30, 12)).policies, ("quiet",))   # 月曜0:30
        self.assertEqual(self.pol(doc, at(0, 30, 6)).policies, ("normal",))   # 火曜0:30は日曜夜の続きではない

    def test_end_of_day_24(self):
        doc = S(E("late", "23:00", "24:00", policy="quiet"))
        self.assertEqual(self.pol(doc, at(23, 59)).policies, ("quiet",))
        self.assertEqual(self.pol(doc, at(0, 0)).policies, ("normal",))

    def test_full_day_entry(self):
        doc = S(E("all", "00:00", "24:00", policy="quiet"))
        for h in (0, 11, 23):
            self.assertEqual(self.pol(doc, at(h, 0)).policies, ("quiet",))

    def test_clock_not_set_yet_falls_back_to_rules(self):
        st = self.pol(self.DAY, datetime.datetime(1970, 1, 1, 9, 0), rule_state(("stay",)))
        self.assertEqual(st.policies, ("stay",))  # 年が2026未満の時計では時刻条件は成立しない
        self.assertIsNone(st.boundary)

    def test_set_boundary_goes_only_with_its_entry_failed_branches_are_cancelled(self):
        # 項目の中の条件が1つでも落ちたら、その項目の範囲も出ない(曜日が違う)
        doc = S(E("a", "09:00", "10:00", boundary={"mode": "release"}, weekdays=["tue"]))
        self.assertIsNone(self.pol(doc, at(9, 30, 5)).boundary)
        self.assertEqual(self.pol(doc, at(9, 30, 6)).boundary, {"mode": "release"})


class RuntimeStateTests(unittest.TestCase):
    DOC = EvaluationTests.DAY

    def state(self, when, running=True, doc=None):
        st, tnorm = evaluate(doc or self.DOC, when)
        return sp.runtime_state(tnorm, st.statuses, when, running, running)

    def test_active_and_next(self):
        s = self.state(at(12, 30))
        self.assertEqual(s["active_entry_id"], "lunch")
        self.assertEqual(s["active_until"], "2026-10-05T13:00:00")
        self.assertEqual((s["next_entry_id"], s["next_entry_at"]), ("afternoon", "2026-10-05T13:00:00"))
        self.assertTrue(s["running"])
        self.assertFalse(s["clock_suspect"])
        self.assertEqual(s["entry_count"], 3)

    def test_between_entries_has_no_active(self):
        s = self.state(at(18, 0))
        self.assertIsNone(s["active_entry_id"])
        self.assertEqual((s["next_entry_id"], s["next_entry_at"]), ("morning", "2026-10-06T08:00:00"))
        s = self.state(at(6, 0))
        self.assertEqual((s["next_entry_id"], s["next_entry_at"]), ("morning", "2026-10-05T08:00:00"))

    def test_not_running_reports_nothing_active(self):
        s = self.state(at(12, 30), running=False)
        self.assertFalse(s["running"])
        self.assertIsNone(s["active_entry_id"])
        self.assertIsNone(s["next_entry_at"])

    def test_weekday_next_entry_skips_to_the_right_day(self):
        doc = S(E("w", "09:00", "10:00", weekdays=["wed"]))
        s = self.state(at(9, 30, 5), doc=doc)  # 月曜
        self.assertIsNone(s["active_entry_id"])
        self.assertEqual(s["next_entry_at"], "2026-10-07T09:00:00")

    def test_wrap_with_weekday_active_until(self):
        doc = S(E("night", "22:00", "02:00", weekdays=["fri"]))
        s = self.state(at(1, 0, 10), doc=doc)  # 土曜1時
        self.assertEqual(s["active_entry_id"], "night")
        self.assertEqual(s["active_until"], "2026-10-10T02:00:00")
        s = self.state(at(23, 0, 9), doc=doc)
        self.assertEqual(s["active_until"], "2026-10-10T02:00:00")
        self.assertEqual(s["next_entry_at"], "2026-10-16T22:00:00")

    def test_suspect_clock(self):
        when = datetime.datetime(1970, 1, 1, 12, 30)
        st, tnorm = evaluate(self.DOC, when)
        s = sp.runtime_state(tnorm, st.statuses, when, True, True)
        self.assertTrue(s["clock_suspect"])
        self.assertIsNone(s["active_entry_id"])


def D(mode):
    return {"mode": mode}


class DisplayTests(unittest.TestCase):
    """頭部ディスプレイの画面の指定 display(2026-10-23)。"""

    def with_display(self, eid, start, mode, end=None, **kw):
        e = E(eid, start, end, **kw)
        e["display"] = mode if not isinstance(mode, str) else D(mode)
        return e

    def err(self, doc):
        errors, _w, norm = sp.validate_schedule(doc)
        self.assertTrue(errors, doc)
        self.assertIsNone(norm)
        return errors[0]

    def test_omitted_and_keep_mean_no_change(self):
        n = norm_of(S(E("a", "09:00")))
        self.assertNotIn("display", n["entries"][0])
        self.assertEqual(sp.entry_display_mode(n["entries"][0]), "keep")
        n = norm_of(S(self.with_display("a", "09:00", "keep")))
        self.assertNotIn("display", n["entries"][0])

    def test_valid_modes_are_kept(self):
        for mode in ("customer", "survey", "ocrtest"):
            n = norm_of(S(self.with_display("a", "09:00", mode)))
            self.assertEqual(n["entries"][0]["display"], D(mode))
            self.assertEqual(sp.entry_display_mode(n["entries"][0]), mode)

    def test_unknown_mode_and_keys_are_rejected(self):
        self.assertIn("display.mode", self.err(S(self.with_display("a", "09:00", "vr"))))
        self.assertIn("display.mode", self.err(S(self.with_display("a", "09:00", {}))))
        self.assertIn("display.mode", self.err(S(self.with_display("a", "09:00", {"mode": 1}))))
        self.assertIn("display", self.err(S(self.with_display("a", "09:00", {"mode": "survey", "url": "x"}))))
        self.assertIn("display", self.err(S(self.with_display("a", "09:00", "survey") | {"display": "survey"})))
        self.assertIn("display", self.err(S(self.with_display("a", "09:00", ["survey"]))))

    def test_summary_mentions_the_display(self):
        n = norm_of(S(self.with_display("a", "09:00", "survey", end="10:00"), E("b", "10:00", "11:00"),
                      self.with_display("c", "11:00", "customer", end="12:00"),
                      self.with_display("d", "12:00", "ocrtest", end="13:00")))
        s = sp.summarize(n)
        self.assertIn("頭部ディスプレイはアンケートを表示", s[0])
        self.assertNotIn("頭部ディスプレイ", s[1])
        self.assertIn("頭部ディスプレイはお客さん向けを表示", s[2])
        self.assertIn("頭部ディスプレイはOCRテストを表示", s[3])

    def test_survey_warns_about_stop_and_pending(self):
        errors, warnings, _n = sp.validate_schedule(S(self.with_display("a", "09:00", "survey")))
        self.assertEqual(errors, [])
        self.assertTrue(any("アンケート" in w and "停止" in w for w in warnings), warnings)
        _e, warnings, _n = sp.validate_schedule(S(self.with_display("a", "09:00", "customer")))
        self.assertFalse(any("アンケート" in w for w in warnings))

    def test_compile_adds_set_display_only_when_not_keep(self):
        tree = sp.compile_schedule(norm_of(S(self.with_display("a", "09:00", "survey", end="10:00"),
                                             E("b", "10:00", "11:00"))))
        a, b = tree["root"]["children"][:2]
        sd = [c for c in a["children"] if c["type"] == "set_display"]
        self.assertEqual(len(sd), 1)
        self.assertEqual(sd[0]["params"], {"mode": "survey"})
        self.assertFalse([c for c in b["children"] if c["type"] == "set_display"])
        errors, _w, _norm = bt.validate_document(tree)
        self.assertEqual(errors, [])

    def test_expected_display_is_selected_for_each_time(self):
        doc = S(self.with_display("a", "09:00", "customer", end="12:00"),
                self.with_display("b", "12:00", "survey", end="13:00"),
                E("c", "13:00", "14:00"))
        self.assertIsNone(evaluate(doc, at(8, 0))[0].display)  # どの項目でもない
        self.assertEqual(evaluate(doc, at(9, 0))[0].display, "customer")
        self.assertEqual(evaluate(doc, at(11, 59))[0].display, "customer")
        self.assertEqual(evaluate(doc, at(12, 0))[0].display, "survey")
        self.assertIsNone(evaluate(doc, at(13, 0))[0].display)  # keepの項目
        self.assertIsNone(evaluate(doc, at(14, 0))[0].display)  # 項目の外

    def test_set_display_node_validation(self):
        def tree(params):
            return {"version": 1, "root": {"type": "sequence", "children": [{"type": "set_display", "params": params}]}}
        self.assertEqual(bt.validate_document(tree({"mode": "customer"}))[0], [])
        self.assertTrue(bt.validate_document(tree({}))[0])
        self.assertTrue(bt.validate_document(tree({"mode": "keep"}))[0])
        self.assertTrue(bt.validate_document(tree({"mode": "vr"}))[0])
        self.assertTrue(bt.validate_document(tree({"mode": "survey", "x": 1}))[0])

    def test_set_display_is_listed_in_registry_for_the_tree_editor(self):
        meta = bt.registry_meta()
        spec = [t for t in meta["types"] if t["type"] == "set_display"][0]
        self.assertEqual(spec["kind"], "action")
        self.assertTrue(spec["label"] and spec["desc"] and spec["icon"])
        modes = [o[0] for o in [p for p in spec["params"] if p["key"] == "mode"][0]["options"]]
        self.assertEqual(modes, ["customer", "survey", "ocrtest"])

    def test_failed_branch_does_not_leak_display(self):
        # 条件が成立しなかった枝の set_display は出力に残らない(時間外)
        doc = S(self.with_display("a", "09:00", "survey", end="10:00"))
        self.assertIsNone(evaluate(doc, at(10, 30))[0].display)

    def test_status_dict_carries_display(self):
        doc = S(self.with_display("a", "09:00", "survey", end="10:00"))
        st, tnorm = evaluate(doc, at(9, 30))
        r = bt.TreeRunner("")
        self.assertEqual(r.status_dict(st)["display"], "survey")
        self.assertIsNone(r.status_dict(None)["display"])


if __name__ == "__main__":
    unittest.main()
