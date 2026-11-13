"""「人の方へ動く動作(接近・見回し)は、指定したときだけ」の単体テスト。

指定の方法は2つ: (1) 管理画面のスイッチ(/nokosahen/behavior_director/person_interaction_enabled)、
(2) ビヘイビアツリー/ルールの approach_enabled / crowd_scan_enabled の明示。
どちらも無いとき(既定)は、接近・見回し・入口への移動をせず、その場で待つ(発話は従来どおり)。
rclpy不要。実行: python ros2_ws/src/nokosahen_core/test/test_person_interaction_gate.py
"""

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import test_behavior_rules as base  # noqa: E402

bd = base.bd
br = base.br
rule = base.rule


@unittest.skipIf(bd is None, "behavior_director import失敗")
class PersonInteractionGateTests(unittest.TestCase):
    def setUp(self):
        self.h = base.DirectorTickTests("make")
        self.h.setUp()

    def tearDown(self):
        self.h.tearDown()

    def make(self, rules=None, crowd="calm", target=True, gate=True, switch=False, scan=False):
        n = self.h.make(rules, crowd=crowd, target=target)
        n._require_interaction_opt_in = gate
        n._person_interaction_enabled = switch
        n._scan_enabled = scan  # 見回す・揺動は独立のスイッチ(2026-11-03)
        n._clock.t += 10  # 接近(2秒)・入口(5秒)ゴールの再送間隔を越える
        return n

    def modes(self, n, seconds=1.0):
        return self.h.run_ticks(n, seconds)

    # --- 既定(スイッチOFF・明示なし): 人の方へ動かない ---
    def test_default_off_does_not_approach(self):
        n = self.make()
        modes = self.modes(n)
        self.assertNotIn("APPROACH_PERSON", modes)
        self.assertEqual(set(modes), {"POLICY_NO_SCAN"})
        self.assertFalse(self.h.moved(n))

    def test_default_off_does_not_scan_when_no_person(self):
        n = self.make(target=False)
        self.assertEqual(set(self.modes(n)), {"POLICY_NO_SCAN"})
        self.assertFalse(self.h.moved(n))

    def test_default_off_crowded_sends_no_entrance_goal(self):
        n = self.make(crowd="crowded")
        self.assertEqual(set(self.modes(n)), {"STOP_ANNOUNCE"})
        self.assertFalse(n._goal_pub.msgs)

    def test_default_off_still_announces(self):
        n = self.make()
        self.modes(n)
        self.assertTrue(n._announce_pub.msgs)  # 発話は従来どおり

    # --- スイッチON: 従来どおり人の方へ動く ---
    def test_switch_on_approaches(self):
        n = self.make(switch=True)
        modes = self.modes(n)
        self.assertEqual(set(modes), {"APPROACH_PERSON"})
        self.assertTrue(n._goal_pub.msgs)

    def test_switch_on_scans_when_no_person(self):
        n = self.make(target=False, scan=True)
        self.assertEqual(set(self.modes(n)), {"CROWD_SCAN"})

    def test_approach_switch_alone_does_not_scan(self):
        n = self.make(target=False, switch=True)
        self.assertEqual(set(self.modes(n)), {"POLICY_NO_SCAN"})

    def test_switch_on_crowded_sends_entrance_goal(self):
        n = self.make(crowd="crowded", switch=True)
        self.modes(n)
        self.assertTrue(n._goal_pub.msgs)

    # --- ビヘイビアツリー/ルールの明示: スイッチOFFでも動く ---
    def test_rule_approach_enabled_overrides_off_switch(self):
        n = self.make([rule("a", policy="approach_enabled")])
        self.assertEqual(set(self.modes(n)), {"APPROACH_PERSON"})

    def test_rule_scan_enabled_overrides_off_switch(self):
        n = self.make([rule("a", policy="crowd_scan_enabled")], target=False)
        self.assertEqual(set(self.modes(n)), {"CROWD_SCAN"})

    def test_rule_approach_disabled_wins_even_if_switch_on(self):
        n = self.make([rule("a", policy="approach_disabled")], switch=True)
        self.assertNotIn("APPROACH_PERSON", self.modes(n))

    # --- ゲートを切る(従来動作)・通知 ---
    def test_gate_disabled_keeps_legacy_behaviour(self):
        n = self.make(gate=False)
        self.assertEqual(set(self.modes(n)), {"APPROACH_PERSON"})

    def test_command_reports_state(self):
        n = self.make(switch=False)
        self.modes(n)
        cmd = json.loads(n._command_pub.msgs[-1].data)
        self.assertIs(cmd["person_interaction_enabled"], False)
        self.assertIs(cmd["person_interaction_gate"], True)

    def test_on_callback_updates_state(self):
        n = self.make()
        msg = base._Bool()
        msg.data = True
        n._on_person_interaction_enabled(msg)
        self.assertTrue(n._person_interaction_enabled)
        self.assertEqual(set(self.modes(n)), {"APPROACH_PERSON"})
        msg.data = False
        n._on_person_interaction_enabled(msg)
        self.assertFalse(n._person_interaction_enabled)

    # --- Effects ---
    def test_effects_explicit_flags(self):
        e = br.effects_for(("approach_enabled",), {})
        self.assertTrue(e.approach_explicit)
        self.assertFalse(e.scan_explicit)
        e = br.effects_for(("crowd_scan_enabled",), {})
        self.assertTrue(e.scan_explicit)
        self.assertFalse(br.effects_for((), {}).approach_explicit)
        self.assertFalse(br.NEUTRAL.approach_explicit)
        # 明示の禁止は許可の明示にならない
        e = br.effects_for(("approach_disabled",), {})
        self.assertFalse(e.approach_explicit)

    # --- 左右の揺動(IDLE_WIGGLE)も同じスイッチの対象 ---
    def test_wiggle_off_when_switch_off(self):
        n = self.make(target=False)
        n._idle_wiggle_enabled = True
        modes = self.modes(n)
        self.assertNotIn("IDLE_WIGGLE", modes)
        self.assertEqual(set(modes), {"POLICY_NO_SCAN"})
        self.assertFalse(self.h.moved(n))

    def test_wiggle_works_when_switch_on(self):
        n = self.make(target=False, scan=True)
        n._idle_wiggle_enabled = True
        self.assertEqual(set(self.modes(n)), {"IDLE_WIGGLE"})

    def test_rule_wiggle_on_overrides_off_switch(self):
        n = self.make([rule("a", policy="idle_wiggle_on")], target=False)
        self.assertEqual(set(self.modes(n)), {"IDLE_WIGGLE"})

    def test_wiggle_legacy_when_gate_disabled(self):
        n = self.make(target=False, gate=False)
        n._idle_wiggle_enabled = True
        self.assertEqual(set(self.modes(n)), {"IDLE_WIGGLE"})

    # --- 呼びかけ(メニュー紹介の発話)も、ビヘイビアツリー/ルールで指示したときだけ ---
    def test_announce_default_off_when_opt_in_required(self):
        n = self.make(switch=True)
        n._require_announce_opt_in = True
        self.modes(n)
        self.assertFalse(n._announce_pub.msgs)  # 指示が無い間は、メニュー紹介の呼びかけを出さない

    def test_announce_with_tree_rule_announce_param(self):
        n = self.make([rule("a", policy="normal", params={"announce_min_interval_seconds": 20})], switch=True)
        n._require_announce_opt_in = True
        self.modes(n)
        self.assertTrue(n._announce_pub.msgs)

    def test_announce_opt_in_disabled_keeps_legacy(self):
        n = self.make(switch=True)
        n._require_announce_opt_in = False
        self.modes(n)
        self.assertTrue(n._announce_pub.msgs)

    def test_announce_quiet_still_silences_even_if_instructed(self):
        n = self.make([rule("a", policy="quiet")], switch=True)
        n._require_announce_opt_in = True
        self.modes(n)
        self.assertFalse(n._announce_pub.msgs)

    def test_effects_announce_explicit(self):
        self.assertTrue(br.effects_for(("normal",), {"announce_min_interval_seconds": 30}).announce_explicit)
        self.assertFalse(br.effects_for(("normal",), {}).announce_explicit)
        self.assertFalse(br.NEUTRAL.announce_explicit)

    def test_command_reports_announce_gate(self):
        n = self.make()
        n._require_announce_opt_in = True
        self.modes(n)
        cmd = json.loads(n._command_pub.msgs[-1].data)
        self.assertIs(cmd["announce_gate"], True)
        self.assertIs(cmd["announce_instructed"], False)


if __name__ == "__main__":
    unittest.main()
