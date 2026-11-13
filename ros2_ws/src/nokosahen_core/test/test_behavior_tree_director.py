"""ビヘイビアツリーと behavior_director の結合テスト(2026-10-09)。

rclpy不要(未インストール時はスタブ注入)。ロボット・ROSには一切触れない(cmd_vel/ゴールはスタブのpublisherに溜まるだけ)。
最も重要な安全要件を総当たりで確認する:
- 安全層(ESTOP/MANUAL/アンケート回答中/人が近すぎる/範囲外/立入禁止/カメラ障害物/MAPPING)は、
  木が何を出していても(全動作ノード×混雑度×人物の有無)常に優先され、走行指令・ゴールが出ない。
- 既定の木(ルール表ノード1つ)は、従来のルール表だけの動作と完全に同じ(モード・ゴール・cmd_vel・発話・状態配信)。
- 木のファイルが無い/不正/無効/評価で例外 のときは従来動作。
- 木が出すゴールも、director既存の出力ゲート(範囲・立入禁止・ピン判定)を通る。
実行: python ros2_ws/src/nokosahen_core/test/test_behavior_tree_director.py
"""

import copy
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(TEST_DIR.parent))

import test_behavior_pins as tp  # noqa: E402  (スタブ導入・pin用の部品)
import test_behavior_rules as tb  # noqa: E402
from nokosahen_core import behavior_rules as br  # noqa: E402
from nokosahen_core import behavior_tree as bt  # noqa: E402

bd = tb.bd
PIN = dict(tp.PIN_A, id="pinA", name="ピンA")


def N(typ, children=None, **params):
    d = {"type": typ}
    if params:
        d["params"] = params
    if children is not None:
        d["children"] = children
    return d


def tree_doc(root, hold=0, **kw):
    d = {"version": 1, "min_hold_seconds": hold, "root": root}
    d.update(kw)
    return d


class BoolMsg:
    def __init__(self, data):
        self.data = data


@unittest.skipIf(bd is None, f"behavior_director import失敗: {tb._BD_ERR}")
class TreeDirectorBase(unittest.TestCase):
    make_node = tb.DirectorTickTests.make
    run_ticks = tb.DirectorTickTests.run_ticks
    moved = staticmethod(tb.DirectorTickTests.moved)
    _safety_cases = tb.DirectorTickTests._safety_cases

    def setUp(self):
        self._orig = (bd.Twist, bd.PoseStamped, bd.String, bd.Bool)
        bd.Twist, bd.PoseStamped, bd.String, bd.Bool = tb._Twist, tb._Pose, tb._Str, tb._Bool

    def tearDown(self):
        bd.Twist, bd.PoseStamped, bd.String, bd.Bool = self._orig

    def node(self, tree=None, rules=None, crowd="calm", target=True, now=None, pins=True, force_user=True):
        """木つきdirector。tree=None なら木のランナーを持たない(従来のdirectorそのもの)。"""
        n = self.make_node(rules, crowd=crowd, target=target)
        n.pins_stale_sec = 30.0
        n.pin_goal_timeout_sec = 120.0
        n._policy_pin_goal_key = None
        n._policy_pin_logged = None
        if pins:
            n._pins_payload = tp.payload([dict(PIN), dict(tp.PIN_B)])
            n._pins_rx_time = n._clock.now()
        if tree is not None:
            when = now or tb.dt(12, 0)
            runner = bt.TreeRunner("", clock=lambda: when, mono=lambda: n._clock.t)
            if tree != "default":
                errors, _w, norm = bt.validate_document(tree)
                assert not errors, errors
                runner.set_document(norm)
            elif force_user:
                runner._from_file = True  # 既定の木を「ファイルから読んだ木」として扱う(同値性の検証用)
            n._bt = runner
            n._bt_status_pub = tb._Pub()
            n._bt_error_logged = False
            n._rules_tick()
        n._clock.t += 10  # ゴールの5秒再送スロットルを越える
        return n

    @staticmethod
    def command(n):
        return json.loads(n._command_pub.msgs[-1].data)

    @staticmethod
    def twists(n):
        return [(t.linear.x, t.angular.z) for t in n._cmd_vel_pub.msgs]

    @staticmethod
    def goals(n):
        return [(g.pose.position.x, g.pose.position.y, g.pose.orientation.z) for g in n._goal_pub.msgs]

    @staticmethod
    def nonzero(n):
        return [t for t in n._cmd_vel_pub.msgs if t.linear.x or t.angular.z]

    def trace(self, n, seconds=3.0, refresh=None):
        modes, cmds = [], []
        for _ in range(int(round(seconds / 0.1))):
            if refresh:
                refresh(n)
            modes += self.run_ticks(n, 0.1)
            c = self.command(n)
            cmds.append((c["active_rule"], c["policy"]))
        return modes, cmds


def action_nodes():
    """動作ノードを全種類(代表的なパラメータで)。秒数指定ありも含む。"""
    base = [
        N("hold_stop"), N("go_to_entrance"), N("return_home"), N("quiet"), N("normal"),
        N("goto_pin", pin_id="pinA", pin_name="ピンA", pin_map="hall"),
        N("goto_pin", pin_id="nopin"),
        N("approach", enabled=False), N("approach", enabled=True), N("scan", enabled=False),
        N("scan", enabled=True), N("wiggle"), N("wiggle", enabled=False), N("speed_scale", scale=0.4),
        N("announce", interval_seconds=10), N("wait", seconds=30),
    ]
    base += [N("set_policy", policy=p, **({"scale": 0.5} if p == "speed_scale" else {"pin_id": "pinA"} if p == "goto_pin" else {}))
             for p in br.POLICIES]
    base += [dict(n, params=dict(n.get("params", {}), for_seconds=60.0)) for n in list(base[:8])]
    return base


class EquivalenceTests(TreeDirectorBase):
    """既定の木(ルール表ノード1つ)= 従来のルール表だけの動作(モード・ゴール・cmd_vel・発話・状態配信)。"""

    RULESETS = [None] + tb.DirectorTickTests.ALL_POLICY_RULES + [
        [tb.rule("a", when={"crowd_level": ["crowded"]}, policy="stay", priority=5),
         tb.rule("b", when={"person_present": True}, policy=["quiet", "approach_disabled"], priority=1)],
        [tb.rule("a", when={"remaining_ratio_below": 0.5}, policy="speed_scale", params={"scale": 0.4})],
    ]

    def compare(self, rules, crowd, target, setter=None, expected=None):
        label = (rules and [r["then"]["policy"] for r in rules], crowd, target, expected)
        a = self.node(None, rules, crowd, target)
        b = self.node("default", rules, crowd, target)
        self.assertTrue(b._bt.user_active)  # 木の経路で評価している(従来経路の素通りではない)
        refresh = None
        if setter:
            for x in (a, b):
                setter(x)
            refresh = lambda m, s=setter: s(m)  # noqa: E731
        ma, ca = self.trace(a, 3.0, refresh)
        mb, cb = self.trace(b, 3.0, refresh)
        self.assertEqual(ma, mb, label)
        self.assertEqual(ca, cb, label)
        self.assertEqual(self.twists(a), self.twists(b), label)
        self.assertEqual(self.goals(a), self.goals(b), label)
        self.assertEqual(len(a._cancel_goal_pub.msgs), len(b._cancel_goal_pub.msgs), label)
        self.assertEqual([m.data for m in a._announce_pub.msgs], [m.data for m in b._announce_pub.msgs], label)
        self.assertEqual(a._rule_effects, b._rule_effects, label)

        def strip(n):
            out = []
            for m in n._rules_state_pub.msgs:
                d = json.loads(m.data)
                d.pop("stamp", None)
                out.append(d)
            return out

        sa, sb = strip(a), strip(b)
        self.assertEqual(sa, sb[-len(sa):], label)  # bは構築時に1回多くtickしているので末尾で揃える

    def test_default_tree_equals_rules_only_matrix(self):
        cases = [(None, None)] + self.cases()
        count = 0
        for rules in self.RULESETS:
            for crowd in ("calm", "crowded"):
                for target in (True, False):
                    for expected, setter in cases:
                        self.compare(rules, crowd, target, setter, expected)
                        count += 1
        self.assertGreater(count, 500)

    def cases(self):
        return self._safety_cases()

    def test_no_tree_file_means_old_behaviour_and_published_status(self):
        # ファイルの無い実ランナー(user_activeでない)は従来どおり。状態は配信される。
        a = self.node(None, [tb.rule("a", policy="stay")])
        b = self.node("default", [tb.rule("a", policy="stay")], force_user=False)
        self.assertFalse(b._bt.user_active)
        ma, _ = self.trace(a)
        mb, _ = self.trace(b)
        self.assertEqual(ma, mb)
        st = json.loads(b._bt_status_pub.msgs[-1].data)
        self.assertFalse(st["user_active"])
        self.assertTrue(st["is_default"])


class SafetyBruteForceTests(TreeDirectorBase):
    """全安全状態 x 混雑度 x 人物 x 木の全動作ノード: 走行指令・ゴールが出ない。"""

    def trees(self):
        out = [("node:" + n["type"], tree_doc(n)) for n in action_nodes()]
        out += [("example:" + e["id"], e["document"]) for e in bt.example_documents()]
        out.append(("combo", tree_doc(N("parallel", [N("go_to_entrance"), N("goto_pin", pin_id="pinA"),
                                                    N("approach"), N("wiggle"), N("speed_scale", scale=0.5)]))))
        return out

    def test_safety_layers_always_win(self):
        checked = 0
        for tname, doc in self.trees():
            for crowd in ("calm", "crowded"):
                for target in (True, False):
                    for expected, setter in self._safety_cases():
                        label = (tname, crowd, target, expected)
                        n = self.node(doc, None, crowd, target)
                        setter(n)
                        refresh = {
                            "MANUAL_OVERRIDE": setter, "NEAR_PERSON_HOLD": setter, "CAMERA_OBSTACLE_HOLD": setter,
                        }.get(expected)
                        modes, _ = self.trace(n, 1.0, refresh)
                        self.assertEqual(set(modes), {expected}, label)
                        self.assertFalse(self.nonzero(n), label)
                        self.assertEqual(n._goal_pub.msgs, [], label)
                        st = json.loads(n._bt_status_pub.msgs[-1].data)
                        self.assertTrue(st["safety_hold"] or expected in ("OUT_OF_BOUNDS_HOLD", "KEEPOUT_ZONE_HOLD"), label)
                        checked += 1
        self.assertGreater(checked, 1000)

    def test_survey_hold_wins_over_every_tree(self):
        for tname, doc in self.trees():
            for crowd in ("calm", "crowded"):
                n = self.node(doc, None, crowd)
                modes = []
                for _ in range(10):
                    n._on_survey_active(BoolMsg(True))
                    modes += self.run_ticks(n, 0.1)
                self.assertEqual(set(modes), {"SURVEY_ACTIVE_HOLD"}, (tname, crowd))
                self.assertFalse(self.nonzero(n), tname)
                self.assertEqual(n._goal_pub.msgs, [], tname)
                self.assertEqual(n._announce_pub.msgs, [], tname)  # 回答中は発話もしない

    def test_tree_never_adds_speech_in_safety_modes(self):
        base = self.node(None, None)
        base._hardware_estop = True
        self.run_ticks(base, 5.0)
        for doc in (tree_doc(N("announce", interval_seconds=10)), tree_doc(N("normal")),
                    tree_doc(N("parallel", [N("announce", interval_seconds=10), N("wiggle")]))):
            n = self.node(doc, None)
            n._hardware_estop = True
            self.run_ticks(n, 5.0)
            self.assertLessEqual(len(n._announce_pub.msgs), len(base._announce_pub.msgs))
        q = self.node(tree_doc(N("quiet")), None)
        q._hardware_estop = True
        self.run_ticks(q, 5.0)
        self.assertFalse(q._announce_pub.msgs)

    def test_goal_rearmed_after_hold_released(self):
        n = self.node(tree_doc(N("go_to_entrance")), None)
        n._near_person_stop_detected = True
        n._last_near_person_stop_receive_time = n._clock.now()
        self.run_ticks(n, 0.5)
        self.assertEqual(n._goal_pub.msgs, [])
        n._near_person_stop_detected = False
        n._clock.t += 10
        n._last_near_person_stop_receive_time = n._clock.now()  # 【2026-11-04】検出ノードは出し続ける(途絶は安全側で停止になるため)
        self.run_ticks(n, 1.0)
        self.assertEqual(len(n._goal_pub.msgs), 1)


class GateTests(TreeDirectorBase):
    """木が出すゴールも、既存の出力ゲート(範囲・立入禁止・ピン判定)を通る。"""

    def test_entrance_goal_blocked_by_keepout_and_boundary(self):
        n = self.node(tree_doc(N("go_to_entrance")), None)
        n._keepout_zones = [{"min_x": -1, "max_x": 1, "min_y": -1, "max_y": 1}]  # 入口(0,0)を含む
        self.run_ticks(n, 3.0)
        self.assertEqual(n._goal_pub.msgs, [])
        self.assertFalse(self.nonzero(n))
        n2 = self.node(tree_doc(N("go_to_entrance")), None)
        n2._boundary = {"enabled": True}  # 範囲が読めない
        self.run_ticks(n2, 3.0)
        self.assertEqual(n2._goal_pub.msgs, [])

    def test_goto_pin_gates(self):
        doc = tree_doc(N("goto_pin", pin_id="pinA", pin_name="ピンA", pin_map="hall"))
        n = self.node(doc, None)
        self.run_ticks(n, 1.0)
        self.assertEqual(len(n._goal_pub.msgs), 1)  # 通常は1回だけ送る
        self.assertAlmostEqual(self.goals(n)[0][0], 2.0)
        # 範囲外のピンはクランプせず走行しない
        o = self.node(doc, None)
        o._boundary = {"enabled": True, "center_x": 0.0, "center_y": 0.0, "width": 1.0, "height": 1.0, "yaw": 0.0}
        self.assertEqual(set(self.run_ticks(o, 3.0)), {"POLICY_PIN_UNAVAILABLE"})
        self.assertEqual(o._goal_pub.msgs, [])
        # 別地図/ピン不在/配信なし
        for kw in (dict(pins=False),):
            m = self.node(doc, None, **kw)
            self.assertEqual(set(self.run_ticks(m, 3.0)), {"POLICY_PIN_UNAVAILABLE"})
            self.assertEqual(m._goal_pub.msgs, [])
        w = self.node(tree_doc(N("goto_pin", pin_id="pinA", pin_map="other")), None)
        self.assertEqual(set(self.run_ticks(w, 3.0)), {"POLICY_PIN_UNAVAILABLE"})
        self.assertEqual(w._goal_pub.msgs, [])
        k = self.node(doc, None)
        k._keepout_zones = [{"min_x": 1.5, "max_x": 2.5, "min_y": 0.5, "max_y": 1.5}]
        self.run_ticks(k, 3.0)
        self.assertEqual(k._goal_pub.msgs, [])

    def test_pin_available_condition_falls_back(self):
        doc = tree_doc(N("selector", [
            N("sequence", [N("pin_available", pin_id="pinA", pin_map="hall"), N("goto_pin", pin_id="pinA")]),
            N("hold_stop")]))
        n = self.node(doc, None)
        self.assertIn("POLICY_GO_PIN", self.run_ticks(n, 1.0))
        m = self.node(doc, None, pins=False)
        self.assertEqual(set(self.run_ticks(m, 2.0)), {"POLICY_STAY"})  # ピンが使えない時の代替枝
        self.assertEqual(m._goal_pub.msgs, [])


class BehaviourTests(TreeDirectorBase):
    def test_day_example_morning_evening(self):
        day = bt.example_documents()[0]["document"]
        # 朝・人がいない: 入口へ
        n = self.node(day, None, crowd="calm", target=False, now=tb.dt(9, 0))
        modes = self.run_ticks(n, 2.0)
        self.assertIn("POLICY_GO_ENTRANCE", modes + [self.command(n)["mode"]])
        self.assertEqual(len(n._goal_pub.msgs), 1)
        # 朝・人が近づいた: 通常動作=接近(保持時間10秒の後)
        n._crowd_state["person_target"] = {"x": 3.0, "y": 0.0}
        n._clock.t += 12
        modes = self.run_ticks(n, 3.0)
        self.assertIn("APPROACH_PERSON", modes)
        # 閉店前: ピンAへ
        e = self.node(day, None, crowd="calm", target=False, now=tb.dt(17, 0))
        modes = self.run_ticks(e, 2.0)
        self.assertIn("POLICY_GO_PIN", modes)
        self.assertAlmostEqual(self.goals(e)[0][0], 2.0)
        self.assertEqual(self.command(e)["policy"], "goto_pin")

    def test_low_stock_shortens_announce_interval(self):
        doc = tree_doc(N("selector", [N("sequence", [N("stock_ratio", op="below", value=0.3), N("announce", interval_seconds=10)])]))
        n = self.node(doc, None, target=False)
        self.run_ticks(n, 35.0)
        self.assertGreaterEqual(len(n._announce_pub.msgs), 3)  # menu A: 2/10=0.2 < 0.3
        ok = self.node(tree_doc(N("selector", [N("sequence", [N("stock_ratio", op="above", value=0.3), N("announce", interval_seconds=10)])])), None, target=False)
        self.run_ticks(ok, 35.0)
        self.assertEqual(len(ok._announce_pub.msgs), 1)

    def test_rules_table_node_keeps_rule_behaviour_and_state(self):
        rules = [tb.rule("r", policy="stay")]
        doc = tree_doc(N("selector", [N("sequence", [N("crowd_is", levels=["crowded"]), N("quiet")]), N("rules_table")]))
        n = self.node(doc, rules, crowd="calm")
        self.assertEqual(set(self.run_ticks(n, 1.0)), {"POLICY_STAY"})
        c = self.command(n)
        self.assertEqual(c["active_rule"], "r")
        self.assertEqual(c["policy"], "stay")
        self.assertEqual(json.loads(n._rules_state_pub.msgs[-1].data)["active_rule_id"], "r")
        self.assertTrue(n._bt_status_pub.msgs)

    def test_tree_removal_cancels_policy_goal(self):
        n = self.node(tree_doc(N("go_to_entrance")), None)
        self.run_ticks(n, 1.0)
        self.assertEqual(len(n._goal_pub.msgs), 1)
        before = len(n._cancel_goal_pub.msgs)
        n._bt.set_document(bt.default_document(), from_file=False)  # ファイルが消えた=従来動作へ
        self.run_ticks(n, 2.0)
        self.assertGreater(len(n._cancel_goal_pub.msgs), before)

    def test_activation_never_collides_with_rules_only(self):
        n = self.node(tree_doc(N("go_to_entrance")), None)
        self.assertGreaterEqual(n._rule_activation, bt.ACTIVATION_BASE)

    def test_status_published_with_safety_flags(self):
        n = self.node(tree_doc(N("selector", [N("go_to_entrance")])), None)
        n._hardware_estop = True
        self.run_ticks(n, 2.0)
        st = json.loads(n._bt_status_pub.msgs[-1].data)
        self.assertTrue(st["user_active"])
        self.assertTrue(st["safety"]["estop"])
        self.assertTrue(st["safety_hold"])
        self.assertEqual(st["mode"], "ESTOP_HOLD")
        self.assertEqual(st["statuses"]["0.0"], "SUCCESS")
        self.assertEqual(st["policy"], "go_to_entrance")   # 木は方針を出しているが…
        self.assertEqual(self.nonzero(n), [])               # …安全層が止めている
        self.assertEqual(n._goal_pub.msgs, [])


class FallbackTests(TreeDirectorBase):
    def test_tree_exception_falls_back_to_rules_once_logged(self):
        rules = [tb.rule("r", policy="stay")]
        base = self.node(None, rules)
        n = self.node(tree_doc(N("go_to_entrance")), rules)
        n._bt.evaluate = lambda *a, **k: 1 / 0
        n._rules_tick()
        n._rules_tick()
        self.assertEqual(n._rule_effects, base._rule_effects)  # ルール表の結果(stay)に戻る
        errs = [m for lvl, m in n._log.lines if lvl == "error"]
        self.assertEqual(len(errs), 1)                           # 同じ失敗は1回だけ記録
        self.assertEqual(set(self.run_ticks(n, 1.0)), {"POLICY_STAY"})

    def test_invalid_file_keeps_old_behaviour_and_warns(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "tree.json"
            p.write_text("{oops", encoding="utf-8")
            rules = [tb.rule("r", policy="stay")]
            base = self.node(None, rules)
            n = self.node("default", rules, force_user=False)
            n._bt = bt.TreeRunner(str(p), clock=lambda: tb.dt(12, 0), mono=lambda: n._clock.t)
            n._rules_tick()
            self.assertFalse(n._bt.user_active)
            self.assertTrue(any(lvl == "warn" and "拒否" in m for lvl, m in n._log.lines))
            mb, _ = self.trace(base)
            mn, _ = self.trace(n)
            self.assertEqual(mb, mn)
            self.assertEqual(n._rule_effects, base._rule_effects)

    def test_disabled_tree_keeps_old_behaviour(self):
        rules = [tb.rule("r", policy="stay")]
        base = self.node(None, rules)
        n = self.node(tree_doc(N("go_to_entrance"), enabled=False), rules)
        self.assertEqual(n._rule_effects, base._rule_effects)
        mb, _ = self.trace(base)
        mn, _ = self.trace(n)
        self.assertEqual(mb, mn)
        self.assertEqual(n._goal_pub.msgs, base._goal_pub.msgs)

    def test_node_without_runner_attrs_unchanged(self):
        # __init__を通さないインスタンス(従来のテスト)でも_btが無くて落ちない
        n = self.make_node(None)
        n.__dict__.pop("_bt", None)
        n._rules_tick()
        self.assertIs(n._rule_effects, br.NEUTRAL)


if __name__ == "__main__":
    unittest.main()
