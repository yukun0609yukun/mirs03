"""ビヘイビアツリー(behavior_tree.py)のエンジン・検証の単体テスト(2026-10-09)。

純Python(rclpy不要)。実機・ROSには一切触れない。
- 各ノードの意味(Selector/Sequence/Parallel/デコレータ/条件/動作)、RUNNING記憶(メモリ付き・for_seconds)
- 方針の合成(矛盾する方針は後勝ち)、最小保持時間、activation
- 検証: 未知のノード/キー/パラメータ、必須、範囲、子の数、深さ/個数の上限、NaN、無効ノード、警告
- 例の木が検証を通り、想定どおり動く
- ファイルのホットリロード(不正は直前を維持、無ければ従来動作)

実行: python ros2_ws/src/nokosahen_core/test/test_behavior_tree.py
"""

import copy
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


def dt(h, m, day=6, s=0):
    return datetime.datetime(2026, 10, day, h, m, s)  # 2026-10-06は火曜


class Clk:
    def __init__(self, start=None):
        self.dt = start or dt(12, 0)
        self.m = 1000.0

    def adv(self, sec):
        self.dt += datetime.timedelta(seconds=sec)
        self.m += sec


def N(typ, children=None, name="", **params):
    d = {"type": typ}
    if name:
        d["name"] = name
    if params:
        d["params"] = params
    if children is not None:
        d["children"] = children
    return d


def DOC(root, hold=0, **kw):
    d = {"version": 1, "min_hold_seconds": hold, "root": root}
    d.update(kw)
    return d


def runner_for(root, hold=0, clk=None):
    clk = clk or Clk()
    r = bt.TreeRunner("", clock=lambda: clk.dt, mono=lambda: clk.m)
    errors, _w, norm = bt.validate_document(DOC(root, hold))
    assert not errors, errors
    r.set_document(norm)
    return r, clk


def ev(r, ctx=None, rule_state=None, safety=None, pin_ok=None):
    return r.evaluate(ctx or br.Context(), rule_state, safety, pin_ok)


def ctx(level="calm", person=True, ratios=None, idle=None):
    return br.Context(crowd_level=level, person_present=person, menu_ratios=ratios or {}, idle_seconds=idle)


class SemanticsTests(unittest.TestCase):
    def test_selector_picks_first_non_failure(self):
        r, _ = runner_for(N("selector", [
            N("sequence", [N("crowd_is", levels=["crowded"]), N("hold_stop")]),
            N("sequence", [N("crowd_is", levels=["calm"]), N("quiet")]),
            N("go_to_entrance"),
        ]))
        self.assertEqual(ev(r, ctx("crowded")).policies, ("stay",))
        self.assertEqual(ev(r, ctx("calm")).policies, ("quiet",))
        self.assertEqual(ev(r, ctx("unknown")).policies, ("go_to_entrance",))

    def test_selector_all_fail_is_failure_and_neutral(self):
        r, _ = runner_for(N("selector", [N("crowd_is", levels=["crowded"])]))
        st = ev(r, ctx("calm"))
        self.assertEqual(st.root_status, bt.FAILURE)
        self.assertEqual(st.policies, ("normal",))
        self.assertEqual(st.effects, br.NEUTRAL)

    def test_sequence_stops_at_failure_and_merges_outputs(self):
        r, _ = runner_for(N("sequence", [N("quiet"), N("speed_scale", scale=0.5), N("crowd_is", levels=["crowded"]), N("hold_stop")]))
        st = ev(r, ctx("calm"))
        self.assertEqual(st.root_status, bt.FAILURE)
        self.assertEqual(st.policies, ("normal",))  # 失敗した枝の出力は取り消される
        r, _ = runner_for(N("sequence", [N("quiet"), N("speed_scale", scale=0.5), N("crowd_is", levels=["calm"]), N("hold_stop")]))
        st = ev(r, ctx("calm"))
        self.assertEqual(st.policies, ("quiet", "speed_scale", "stay"))
        self.assertEqual(st.effects.speed_scale, 0.5)
        self.assertFalse(st.effects.announce_allowed)

    def test_conflicting_policies_later_wins(self):
        r, _ = runner_for(N("sequence", [N("go_to_entrance"), N("return_home"), N("approach", enabled=False), N("approach", enabled=True)]))
        st = ev(r)
        self.assertEqual(st.policies, ("return_home", "approach_enabled"))
        self.assertEqual(st.effects.go_to, "home")
        self.assertTrue(st.effects.approach_allowed)

    def test_normal_action_changes_nothing_and_announce_interval(self):
        r, _ = runner_for(N("sequence", [N("normal"), N("announce", interval_seconds=20)]))
        st = ev(r)
        self.assertEqual(st.policies, ("normal",))
        self.assertEqual(st.effects.announce_min_interval_seconds, 20.0)
        self.assertTrue(st.effects.announce_allowed)

    def test_parallel_all_any(self):
        kids = [N("crowd_is", levels=["calm"]), N("crowd_is", levels=["crowded"]), N("quiet")]
        r, _ = runner_for(N("parallel", kids, success="all"))
        st = ev(r, ctx("calm"))
        self.assertEqual(st.root_status, bt.FAILURE)
        self.assertEqual(st.policies, ("normal",))  # 失敗したら出力も取り消し
        r, _ = runner_for(N("parallel", kids, success="any"))
        st = ev(r, ctx("calm"))
        self.assertEqual((st.root_status, st.policies), (bt.SUCCESS, ("quiet",)))  # 全部の子が評価される
        r, _ = runner_for(N("parallel", [N("quiet"), N("speed_scale", scale=0.5)]))
        self.assertEqual(ev(r).policies, ("quiet", "speed_scale"))

    def test_inverter_and_force(self):
        r, _ = runner_for(N("sequence", [N("inverter", [N("crowd_is", levels=["crowded"])]), N("quiet")]))
        self.assertEqual(ev(r, ctx("calm")).policies, ("quiet",))
        self.assertEqual(ev(r, ctx("crowded")).policies, ("normal",))
        r, _ = runner_for(N("sequence", [N("force_success", [N("crowd_is", levels=["crowded"])]), N("quiet")]))
        self.assertEqual(ev(r, ctx("calm")).policies, ("quiet",))
        r, _ = runner_for(N("force_failure", [N("quiet")]))
        st = ev(r)
        self.assertEqual(st.root_status, bt.FAILURE)
        self.assertEqual(st.policies, ("normal",))  # 失敗にすると出力も取り消される

    def test_conditions(self):
        def ok(node, c, clk=None):
            r, _ = runner_for(N("sequence", [node, N("quiet")]), clk=clk)
            return ev(r, c).policies == ("quiet",)

        self.assertTrue(ok(N("time_between", range="11:30-13:00"), ctx()))
        self.assertFalse(ok(N("time_between", range="13:00-14:00"), ctx()))
        self.assertTrue(ok(N("time_between", range="22:00-06:00"), ctx(), Clk(dt(23, 0))))
        self.assertTrue(ok(N("weekday_in", days=["tue"]), ctx()))
        self.assertFalse(ok(N("weekday_in", days=["mon", "wed"]), ctx()))
        self.assertTrue(ok(N("date_in", dates=["2026-10-06"]), ctx()))
        self.assertFalse(ok(N("date_in", dates=["2026-10-07"]), ctx()))
        self.assertTrue(ok(N("crowd_is", levels=["calm", "crowded"]), ctx("crowded")))
        self.assertTrue(ok(N("person_present"), ctx(person=True)))
        self.assertFalse(ok(N("person_present"), ctx(person=False)))
        self.assertTrue(ok(N("person_present", value=False), ctx(person=False)))
        self.assertFalse(ok(N("person_present", value=False), ctx(person=None)))  # 不明は成立しない
        self.assertTrue(ok(N("stock_ratio", op="below", value=0.3), ctx(ratios={"A": 0.2, "B": 0.9})))
        self.assertFalse(ok(N("stock_ratio", op="below", value=0.3), ctx(ratios={"A": 0.5})))
        self.assertTrue(ok(N("stock_ratio", op="above", value=0.5, menu="B"), ctx(ratios={"A": 0.2, "B": 0.9})))
        self.assertFalse(ok(N("stock_ratio", op="below", value=0.3), ctx(ratios={})))  # メニュー未登録
        self.assertTrue(ok(N("idle_for", seconds=60), ctx(idle=61)))
        self.assertFalse(ok(N("idle_for", seconds=60), ctx(idle=60)))  # ルール表と同じく「超」

    def test_idle_tracking_in_runner(self):
        r, clk = runner_for(N("sequence", [N("idle_for", seconds=30), N("quiet")]))
        self.assertEqual(ev(r, br.Context(person_present=True)).policies, ("normal",))
        clk.adv(31)
        self.assertEqual(ev(r, br.Context(person_present=False)).policies, ("quiet",))
        clk.adv(1)
        self.assertEqual(ev(r, br.Context(person_present=True)).policies, ("normal",))  # 人が見えたらリセット

    def test_robot_state_and_pin_available(self):
        r, _ = runner_for(N("selector", [
            N("sequence", [N("robot_state", state="any_safety"), N("quiet")]),
            N("sequence", [N("pin_available", pin_id="pinA"), N("goto_pin", pin_id="pinA")]),
        ]))
        self.assertEqual(ev(r, safety={"survey": True}).policies, ("quiet",))
        self.assertEqual(ev(r, pin_ok=lambda i, m: i == "pinA").policies, ("goto_pin",))
        self.assertEqual(ev(r, pin_ok=lambda i, m: False).policies, ("normal",))

    def test_actions_map_to_policies(self):
        cases = [
            (N("hold_stop"), "stay"), (N("go_to_entrance"), "go_to_entrance"), (N("return_home"), "return_home"),
            (N("quiet"), "quiet"), (N("goto_pin", pin_id="pinA", pin_name="A", pin_map="hall"), "goto_pin"),
            (N("approach", enabled=False), "approach_disabled"), (N("scan", enabled=False), "crowd_scan_disabled"),
            (N("wiggle"), "idle_wiggle_on"), (N("wiggle", enabled=False), "idle_wiggle_off"),
            (N("speed_scale", scale=0.4), "speed_scale"), (N("set_policy", policy="announce_only"), "announce_only"),
            (N("wait", seconds=5), "stay"),
        ]
        for node, pol in cases:
            r, _ = runner_for(node)
            st = ev(r)
            self.assertEqual(st.policies, (pol,), node)
        r, _ = runner_for(N("goto_pin", pin_id="pinA", pin_name="A", pin_map="hall"))
        e = ev(r).effects
        self.assertEqual((e.go_to, e.go_to_pin_id, e.go_to_pin_name, e.go_to_pin_map), ("pin", "pinA", "A", "hall"))

    def test_rules_table_node_passes_rule_through(self):
        e, clk = None, Clk()
        eng = br.RuleEngine("", clock=lambda: clk.dt, mono=lambda: clk.m)
        eng.set_ruleset(br.parse_rules_text(json.dumps({"min_hold_seconds": 0, "rules": [
            {"id": "a", "enabled": True, "priority": 1, "when": {"crowd_level": ["crowded"]},
             "then": {"policy": ["stay"], "params": {"announce_min_interval_seconds": 30}}}]})))
        r, _ = runner_for(N("rules_table"), clk=clk)
        c = ctx("crowded")
        st = ev(r, c, eng.evaluate(c))
        self.assertEqual(st.active_rule_id, "a")
        self.assertEqual(st.policies, ("stay",))
        self.assertEqual(st.effects.announce_min_interval_seconds, 30.0)
        c = ctx("calm")
        st = ev(r, c, eng.evaluate(c))
        self.assertEqual(st.root_status, bt.FAILURE)
        self.assertEqual(st.effects, br.NEUTRAL)
        self.assertIsNone(st.active_rule_id)


class RunningTests(unittest.TestCase):
    def test_for_seconds_runs_then_succeeds(self):
        r, clk = runner_for(N("sequence", [N("go_to_entrance", for_seconds=30), N("quiet")], memory=True))
        st = ev(r)
        self.assertEqual((st.root_status, st.policies), (bt.RUNNING, ("go_to_entrance",)))
        clk.adv(29)
        self.assertEqual(ev(r).policies, ("go_to_entrance",))
        clk.adv(2)
        st = ev(r)
        self.assertEqual((st.root_status, st.policies), (bt.SUCCESS, ("quiet",)))  # 次の子へ進む

    def test_memory_sequence_tour(self):
        r, clk = runner_for(N("sequence", [
            N("go_to_entrance", for_seconds=10), N("goto_pin", pin_id="p1", for_seconds=10), N("wait", seconds=10)], memory=True))
        seq = []
        for _ in range(35):
            seq.append(ev(r).policies[0])
            clk.adv(1)
        # 順番に進み、最後のwaitが終わったらSUCCESS(方針なし)
        self.assertEqual(seq[0], "go_to_entrance")
        self.assertIn("goto_pin", seq)
        self.assertIn("stay", seq)
        self.assertLess(seq.index("go_to_entrance"), seq.index("goto_pin"))
        self.assertLess(seq.index("goto_pin"), seq.index("stay"))
        self.assertEqual(seq[30], "normal")          # 全部終わったtickは方針なし
        self.assertEqual(seq[31], "go_to_entrance")  # ルートが評価され続けるので最初からやり直す

    def test_running_state_reset_when_not_ticked(self):
        r, clk = runner_for(N("selector", [N("sequence", [N("crowd_is", levels=["crowded"]), N("wait", seconds=10)]), N("quiet")]))
        ev(r, ctx("crowded")); clk.adv(8)
        self.assertEqual(ev(r, ctx("crowded")).policies, ("stay",))
        ev(r, ctx("calm")); clk.adv(1)           # 途切れた
        ev(r, ctx("crowded"))                    # 最初からやり直し
        clk.adv(5)
        self.assertEqual(ev(r, ctx("crowded")).policies, ("stay",))  # まだ10秒経っていない

    def test_nonmemory_sequence_action_stays_done_while_continuously_ticked(self):
        r, clk = runner_for(N("sequence", [N("hold_stop", for_seconds=5), N("quiet")]))
        ev(r); clk.adv(6)
        self.assertEqual(ev(r).policies, ("quiet",))
        clk.adv(1)
        self.assertEqual(ev(r).policies, ("quiet",))  # 連続して評価されている間は完了のまま

    def test_cooldown(self):
        r, clk = runner_for(N("selector", [N("cooldown", [N("go_to_entrance")], seconds=60), N("quiet")]))
        self.assertEqual(ev(r).policies, ("go_to_entrance",))
        clk.adv(1)
        self.assertEqual(ev(r).policies, ("quiet",))   # クールダウン中は失敗→次の候補
        clk.adv(61)
        self.assertEqual(ev(r).policies, ("go_to_entrance",))

    def test_timeout(self):
        r, clk = runner_for(N("selector", [N("timeout", [N("hold_stop", for_seconds=1000)], seconds=20), N("quiet")]))
        self.assertEqual(ev(r).policies, ("stay",))
        clk.adv(21)
        self.assertEqual(ev(r).policies, ("quiet",))
        clk.adv(5)
        self.assertEqual(ev(r).policies, ("quiet",))   # 連続評価の間は失敗のまま
        # 木が評価されなくなる(上位が別の枝へ)と、次はやり直せる
        r2, clk2 = runner_for(N("selector", [
            N("sequence", [N("crowd_is", levels=["crowded"]), N("timeout", [N("hold_stop", for_seconds=1000)], seconds=20)]),
            N("quiet")]))
        ev(r2, ctx("crowded")); clk2.adv(21)
        self.assertEqual(ev(r2, ctx("crowded")).policies, ("quiet",))
        ev(r2, ctx("calm")); clk2.adv(1)
        self.assertEqual(ev(r2, ctx("crowded")).policies, ("stay",))

    def test_selector_memory_keeps_running_child(self):
        kids = [N("sequence", [N("crowd_is", levels=["calm"]), N("quiet")]), N("hold_stop", for_seconds=100)]
        r, _ = runner_for(N("selector", kids, memory=True))
        self.assertEqual(ev(r, ctx("unknown")).policies, ("stay",))       # 2番目がRUNNING
        self.assertEqual(ev(r, ctx("calm")).policies, ("stay",))          # 覚えているので1番目へ戻らない
        r, _ = runner_for(N("selector", kids))
        ev(r, ctx("unknown"))
        self.assertEqual(ev(r, ctx("calm")).policies, ("quiet",))         # 非メモリは毎回上から


class HysteresisTests(unittest.TestCase):
    def test_min_hold_delays_switch_and_bumps_activation(self):
        root = N("selector", [N("sequence", [N("crowd_is", levels=["crowded"]), N("hold_stop")]), N("quiet")])
        r, clk = runner_for(root, hold=10)
        a = ev(r, ctx("calm"))
        self.assertEqual(a.policies, ("quiet",))
        self.assertTrue(a.changed)
        clk.adv(3)
        b = ev(r, ctx("crowded"))
        self.assertEqual(b.policies, ("quiet",))      # 保持時間内は切り替えない
        self.assertFalse(b.changed)
        self.assertEqual(b.activation, a.activation)
        clk.adv(8)
        c = ev(r, ctx("crowded"))
        self.assertEqual(c.policies, ("stay",))
        self.assertTrue(c.changed)
        self.assertGreater(c.activation, a.activation)
        self.assertGreaterEqual(c.activation, bt.ACTIVATION_BASE)

    def test_zero_hold_switches_immediately_and_same_output_keeps_activation(self):
        r, clk = runner_for(N("selector", [N("sequence", [N("crowd_is", levels=["crowded"]), N("hold_stop")]), N("quiet")]), hold=0)
        a = ev(r, ctx("calm"))
        b = ev(r, ctx("calm"))
        self.assertEqual(a.activation, b.activation)
        self.assertEqual(ev(r, ctx("crowded")).policies, ("stay",))


class ValidationTests(unittest.TestCase):
    def errs(self, doc):
        e, _w, norm = bt.validate_document(doc)
        return e, norm

    def bad(self, doc, needle=None):
        e, norm = self.errs(doc)
        self.assertTrue(e, doc)
        self.assertIsNone(norm)
        if needle:
            self.assertTrue(any(needle in x for x in e), (needle, e))

    def test_default_and_examples_valid(self):
        self.assertEqual(self.errs(bt.default_document())[0], [])
        for ex in bt.example_documents():
            e, n = self.errs(ex["document"])
            self.assertEqual(e, [], ex["id"])
            self.assertEqual(self.errs(json.loads(bt.document_text(n)))[0], [], ex["id"])  # 往復で同じ

    def test_structure_errors(self):
        self.bad([], "オブジェクト")
        self.bad({"version": 1}, "root")
        self.bad({"version": 2, "root": N("quiet")}, "version")
        self.bad({"root": N("quiet"), "bogus": 1}, "未知のキー")
        self.bad(DOC(N("nope")), "未知のノード種別")
        self.bad(DOC({"type": "quiet", "bogus": 1}), "未知のキー")
        self.bad(DOC({"type": "quiet", "ref": "0"}), "未知のキー")        # id参照(循環の元)は書けない
        self.bad(DOC(N("hold_stop", foo=1)), "未知のパラメータ")
        self.bad(DOC(N("quiet", [N("quiet")])), "子を持てません")
        self.bad(DOC(N("inverter", [])), "ちょうど1つ")
        self.bad(DOC(N("inverter", [N("quiet"), N("quiet")])), "ちょうど1つ")
        self.bad(DOC(N("inverter", [dict(N("quiet"), enabled=False)])), "子が無効")
        self.bad(DOC(dict(N("quiet"), enabled=False)), "ルート")
        self.bad(DOC(N("selector", "x")), "配列")
        self.bad(DOC(N("quiet"), hold=-1), "min_hold")
        self.bad(DOC(N("quiet"), hold=601), "min_hold")
        self.bad(DOC(N("quiet"), enabled="yes"), "enabled")

    def test_param_errors(self):
        self.bad(DOC(N("time_between")), "必須")
        self.bad(DOC(N("time_between", range="25:00-26:00")), "範囲外")
        self.bad(DOC(N("time_between", range="9-10")), "書式")
        self.bad(DOC(N("weekday_in", days=["funday"])), "不明")
        self.bad(DOC(N("weekday_in", days=[])), "空でない")
        self.bad(DOC(N("date_in", dates=["2026-02-30"])), "存在しない")
        self.bad(DOC(N("crowd_is", levels=["packed"])), "不明")
        self.bad(DOC(N("stock_ratio", op="below", value=1.5)), "1.5")
        self.bad(DOC(N("stock_ratio", op="near", value=0.5)), "不明")
        self.bad(DOC(N("idle_for", seconds=-1)), "-1")
        self.bad(DOC(N("idle_for", seconds="5")), "数値")
        self.bad(DOC(N("idle_for", seconds=True)), "数値")
        self.bad(DOC(N("cooldown", [N("quiet")], seconds=0)), "1")
        self.bad(DOC(N("speed_scale", scale=0.1)), "0.3")
        self.bad(DOC(N("speed_scale", scale=1.5)), "1.0")
        self.bad(DOC(N("announce", interval_seconds=5)), "10")
        self.bad(DOC(N("announce", interval_seconds=601)), "600")
        self.bad(DOC(N("wait", seconds=0)), "1")
        self.bad(DOC(N("goto_pin")), "必須")
        self.bad(DOC(N("goto_pin", pin_id="has space")), "ピンのid")
        self.bad(DOC(N("goto_pin", pin_id="a", pin_map="bad map!")), "地図名")
        self.bad(DOC(N("hold_stop", for_seconds=-5)), "for_seconds")
        self.bad(DOC(N("robot_state", state="flying")), "不明")
        self.bad(DOC(N("sequence", memory="yes")), "true/false")
        self.bad(DOC(N("set_policy", policy="speed_scale")), "scale")
        self.bad(DOC(N("set_policy", policy="goto_pin")), "pin_id")
        self.bad(DOC(N("set_policy", policy="stay", scale=0.5)), "speed_scale")
        self.bad(DOC(N("set_policy", policy="warp")), "不明")
        self.bad(DOC(N("rules_table", name="x" * 61)), "name")

    def test_nan_rejected(self):
        with self.assertRaises(bt.TreeError):
            bt.parse_tree_text('{"root": {"type": "idle_for", "params": {"seconds": NaN}}}')
        self.bad(DOC(N("idle_for", seconds=float("nan"))), "数値")
        self.bad(DOC(N("idle_for", seconds=float("inf"))), "数値")

    def test_depth_and_node_limits(self):
        node = N("quiet")
        for _ in range(bt.MAX_DEPTH + 1):
            node = N("inverter", [node])
        self.bad(DOC(node), "深すぎ")          # 葉の深さがMAX_DEPTH+1
        ok = N("quiet")
        for _ in range(bt.MAX_DEPTH):
            ok = N("inverter", [ok])
        self.assertEqual(self.errs(DOC(ok))[0], [])
        self.bad(DOC(N("selector", [N("quiet") for _ in range(bt.MAX_NODES)])), "多すぎ")   # 根+MAX個
        self.assertEqual(self.errs(DOC(N("selector", [N("quiet") for _ in range(bt.MAX_NODES - 1)])))[0], [])

    def test_disabled_nodes_skipped_but_ids_stable(self):
        root = N("selector", [dict(N("hold_stop"), enabled=False), N("quiet")])
        r, _ = runner_for(root)
        st = ev(r)
        self.assertEqual(st.policies, ("quiet",))
        self.assertIn("0.1", st.statuses)      # idはJSON上の位置のまま
        self.assertNotIn("0.0", st.statuses)

    def test_warnings(self):
        e, w, _ = bt.validate_document(DOC(N("selector", [N("quiet"), N("hold_stop")])))
        self.assertEqual(e, [])
        self.assertTrue(any("実行されません" in x for x in w), w)
        e, w, _ = bt.validate_document(DOC(N("selector", [])))
        self.assertTrue(any("子が1つも" in x for x in w), w)
        e, w, _ = bt.validate_document(DOC(N("selector", [N("quiet", for_seconds=5), N("hold_stop")])))
        self.assertFalse(any("実行されません" in x for x in w), w)   # 秒数指定は成功しないことがある

    def test_all_registry_types_have_class_and_defaults_validate(self):
        meta = bt.registry_meta()
        self.assertEqual({t["type"] for t in meta["types"]}, set(bt.NODE_CLASSES))
        self.assertEqual([s["key"] for s in meta["safety_layers"]][:2], ["estop", "manual"])
        json.dumps(meta)

    def test_describe_and_cli(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "t.json"
            p.write_text(bt.document_text(bt.parse_tree_text(json.dumps(bt.example_documents()[0]["document"]))), encoding="utf-8")
            ok, text = bt.validate_file(str(p))
            self.assertTrue(ok, text)
            self.assertIn("OK", text)
            p.write_text('{"root": {"type": "nope"}}', encoding="utf-8")
            ok, text = bt.validate_file(str(p))
            self.assertFalse(ok)
            self.assertIn("拒否", text)


class ExampleBehaviourTests(unittest.TestCase):
    def run_example(self, idx, start, c, **kw):
        clk = Clk(start)
        r = bt.TreeRunner("", clock=lambda: clk.dt, mono=lambda: clk.m)
        norm = bt.parse_tree_text(json.dumps(bt.example_documents()[idx]["document"]))
        r.set_document(norm)
        return r, clk

    def test_day_tree(self):
        r, clk = self.run_example(0, dt(9, 0), None)
        # 朝・人がいない: 入口で待つ / 人がいる: 通常動作(接近して案内)
        self.assertEqual(ev(r, ctx(person=False)).policies, ("go_to_entrance",))
        clk.adv(11)
        self.assertEqual(ev(r, ctx(person=True)).policies, ("normal",))
        # 閉店前はピンA
        r, clk = self.run_example(0, dt(17, 0), None)
        st = ev(r, ctx())
        self.assertEqual((st.policies, st.effects.go_to_pin_id), (("goto_pin",), "pinA"))
        # 昼・残数少なめ: 呼びかけ間隔20秒
        r, clk = self.run_example(0, dt(12, 0), None)
        st = ev(r, ctx(ratios={"A": 0.1}))
        self.assertEqual(st.effects.announce_min_interval_seconds, 20.0)
        # 昼・残数十分・ルール表なし: 従来どおり
        r, clk = self.run_example(0, dt(12, 0), None)
        self.assertEqual(ev(r, ctx(ratios={"A": 0.9})).effects, br.NEUTRAL)

    def test_peak_tree(self):
        r, _ = self.run_example(1, dt(12, 0), None)
        st = ev(r, ctx("crowded"))
        self.assertEqual(st.policies, ("quiet", "speed_scale"))
        r, _ = self.run_example(1, dt(12, 0), None)
        self.assertEqual(ev(r, ctx("calm")).effects, br.NEUTRAL)

    def test_tour_tree(self):
        r, clk = self.run_example(2, dt(12, 0), None)
        self.assertEqual(ev(r, ctx(person=True, idle=0)).policies, ("normal",))
        seq = []
        for _ in range(200):
            seq.append(ev(r, ctx(person=False, idle=200)).policies[0])
            clk.adv(1)
        order = [p for i, p in enumerate(seq) if i == 0 or p != seq[i - 1]]
        # 最小保持時間(10秒)で切り替えは遅れるが、順番は 入口→ピン→待つ→通常
        self.assertLess(order.index("go_to_entrance"), order.index("goto_pin"))
        self.assertLess(order.index("goto_pin"), order.index("stay"))
        self.assertEqual(order[-1], "normal")
        # クールダウン(10分)中は再開しない
        self.assertEqual(seq[-1], "normal")
        self.assertEqual(seq.count("go_to_entrance") + seq.count("goto_pin") + seq.count("stay") > 0, True)


class FileReloadTests(unittest.TestCase):
    def test_missing_invalid_disabled_and_good(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "tree.json"
            clk = Clk()
            r = bt.TreeRunner(str(p), clock=lambda: clk.dt, mono=lambda: clk.m)
            self.assertIsNone(r.check_reload())            # ファイル無し=従来動作、メッセージなし
            self.assertFalse(r.user_active)
            p.write_text("{not json", encoding="utf-8")
            os.utime(p, (1_000_000, 1_000_000))
            self.assertIn("拒否", r.check_reload())
            self.assertFalse(r.user_active)                # 不正→従来動作のまま
            self.assertTrue(r.load_error)
            good = DOC(N("hold_stop"))
            p.write_text(json.dumps(good), encoding="utf-8")
            os.utime(p, (1_000_100, 1_000_100))
            self.assertIn("読み込みました", r.check_reload())
            self.assertTrue(r.user_active)
            self.assertIsNone(r.load_error)
            self.assertEqual(ev(r).policies, ("stay",))
            p.write_text("{broken", encoding="utf-8")
            os.utime(p, (1_000_200, 1_000_200))
            self.assertIn("拒否", r.check_reload())
            self.assertTrue(r.user_active)                 # 直前の木を維持
            self.assertEqual(ev(r).policies, ("stay",))
            p.write_text(json.dumps(dict(good, enabled=False)), encoding="utf-8")
            os.utime(p, (1_000_300, 1_000_300))
            self.assertIn("従来動作", r.check_reload())
            self.assertFalse(r.user_active)                # enabled:false
            p.unlink()
            self.assertIn("従来動作", r.check_reload())
            self.assertFalse(r.user_active)

    def test_status_dict_shape(self):
        r, clk = runner_for(N("selector", [N("quiet")]))
        st = ev(r, safety={"estop": True})
        d = r.status_dict(st, {"estop": True}, "ESTOP_HOLD", True)
        json.dumps(d)
        self.assertTrue(d["safety_hold"])
        self.assertTrue(d["safety"]["estop"])
        self.assertFalse(d["safety"]["survey"])
        self.assertEqual(d["statuses"]["0"], bt.SUCCESS)
        self.assertEqual(d["active_leaves"], ["0.0"])
        self.assertEqual(d["mode"], "ESTOP_HOLD")
        self.assertEqual(len(d["doc_hash"]), 12)


if __name__ == "__main__":
    unittest.main()
