"""行動ルールの方針 goto_pin(地図ピンへ行く)の単体テスト(2026-10-09)。

- ルール検証(pin_id必須・不正なid・矛盾する方針・pin_*の置き場所)
- ピン解決(map_pins.resolve_pin): IDで引く・名前変更/重複に強い・地図なし/別地図/削除/壊れた座標/配信なし
- behavior_director: ゴールは1回だけ・ピンの位置と向きで送る・到着で停止・方針解除で取り消し・
  ピン不在/配信途絶/別地図/範囲外/立入禁止/範囲不明では走行指令を出さない・
  総当たりの安全状態(ESTOP/MANUAL/NEAR_PERSON/カメラ障害物/MAPPING/範囲外/立入禁止)では何も出さない

ロボット・ROSには一切触れない(publisherはスタブ)。
実行: python ros2_ws/src/nokosahen_core/test/test_behavior_pins.py
"""

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import test_behavior_rules as tb  # noqa: E402  (スタブ導入・director用の部品を再利用)
from nokosahen_core import behavior_rules as br  # noqa: E402
from nokosahen_core import map_pins as mp  # noqa: E402

bd = tb.bd

PIN_A = {"id": "aaaa1111", "name": "入口", "x": 2.0, "y": 1.0, "yaw_deg": 90.0, "color": "#e11d48"}
PIN_B = {"id": "bbbb2222", "name": "入口", "x": -1.0, "y": 3.0, "yaw_deg": 0.0, "color": "#2563eb"}  # 同名


def payload(pins=(PIN_A, PIN_B), loaded="hall"):
    return mp.pins_payload(loaded, [dict(p) for p in pins])


def pin_rule(pin_id="aaaa1111", **kw):
    params = {"pin_id": pin_id, "pin_name": "入口"}
    params.update(kw.pop("params", {}))
    return tb.rule("pin", policy="goto_pin", params=params, **kw)


# ---------------------------------------------------------------------------
# 検証
# ---------------------------------------------------------------------------
class ValidationTests(unittest.TestCase):
    def bad(self, *rules):
        with self.assertRaises(br.RuleError):
            tb.ruleset(*rules)

    def test_accepts_goto_pin(self):
        rs = tb.ruleset(pin_rule(params={"pin_map": "hall"}), tb.rule("q", policy=["goto_pin", "quiet"],
                                                                      params={"pin_id": "x_1-2"}))
        self.assertEqual(rs.rules[0].params["pin_id"], "aaaa1111")
        self.assertEqual(rs.rules[0].params["pin_map"], "hall")
        self.assertEqual(rs.rules[1].policies, ("goto_pin", "quiet"))

    def test_pin_id_required_and_checked(self):
        self.bad(tb.rule("a", policy="goto_pin"))
        self.bad(tb.rule("a", policy="goto_pin", params={"pin_id": ""}))
        self.bad(tb.rule("a", policy="goto_pin", params={"pin_id": "has space"}))
        self.bad(tb.rule("a", policy="goto_pin", params={"pin_id": "x" * 33}))
        self.bad(tb.rule("a", policy="goto_pin", params={"pin_id": 123}))
        self.bad(tb.rule("a", policy="goto_pin", params={"pin_id": "../etc"}))

    def test_other_pin_params_checked(self):
        self.bad(pin_rule(params={"pin_name": "あ" * 41}))
        self.bad(pin_rule(params={"pin_name": 5}))
        self.bad(pin_rule(params={"pin_map": "bad map"}))
        self.bad(pin_rule(params={"pin_map": "../x"}))

    def test_pin_params_only_with_goto_pin(self):
        self.bad(tb.rule("a", policy="stay", params={"pin_id": "aaaa1111"}))
        self.bad(tb.rule("a", policy="normal", params={"pin_name": "x"}))

    def test_conflicts(self):
        self.bad(tb.rule("a", policy=["goto_pin", "go_to_entrance"], params={"pin_id": "a"}))
        self.bad(tb.rule("a", policy=["goto_pin", "return_home"], params={"pin_id": "a"}))

    def test_unknown_param_still_rejected(self):
        self.bad(pin_rule(params={"pin_x": 1}))

    def test_effects(self):
        eff = br.effects_for(("goto_pin",), {"pin_id": "aaaa1111", "pin_map": "hall", "pin_name": "入口"})
        self.assertEqual(eff.go_to, "pin")
        self.assertEqual((eff.go_to_pin_id, eff.go_to_pin_map, eff.go_to_pin_name), ("aaaa1111", "hall", "入口"))
        self.assertEqual(br.effects_for(("go_to_entrance",), {}).go_to_pin_id, None)
        self.assertEqual(br.effects_for(("normal",), {}), br.NEUTRAL)

    def test_state_json_roundtrip(self):
        e, _ = tb.engine_with(tb.ruleset(pin_rule()))
        d = json.loads(json.dumps(e.evaluate(br.Context()).to_json_dict()))
        self.assertEqual(d["effects"]["go_to"], "pin")
        self.assertEqual(d["effects"]["go_to_pin_id"], "aaaa1111")
        self.assertEqual(d["policy"], "goto_pin")


# ---------------------------------------------------------------------------
# ピン解決
# ---------------------------------------------------------------------------
class ResolveTests(unittest.TestCase):
    def test_resolve_by_id_not_name(self):
        pin, why = mp.resolve_pin(payload(), "bbbb2222")
        self.assertEqual(why, "ok")
        self.assertEqual((pin["x"], pin["y"]), (-1.0, 3.0))  # 同名ピンがあってもIDで一意
        renamed = [dict(PIN_A, name="受付")]
        pin, why = mp.resolve_pin(payload(renamed), "aaaa1111")
        self.assertEqual((why, pin["name"]), ("ok", "受付"))  # 名前を変えても同じピン

    def test_moved_pin_returns_new_position(self):
        pin, _ = mp.resolve_pin(payload([dict(PIN_A, x=5.0, y=6.0)]), "aaaa1111")
        self.assertEqual((pin["x"], pin["y"]), (5.0, 6.0))

    def test_failure_reasons(self):
        self.assertEqual(mp.resolve_pin(None, "aaaa1111"), (None, "no_data"))
        self.assertEqual(mp.resolve_pin({"pins": "x"}, "aaaa1111"), (None, "no_data"))
        self.assertEqual(mp.resolve_pin(payload(loaded=None), "aaaa1111"), (None, "no_map"))
        self.assertEqual(mp.resolve_pin(payload(loaded=""), "aaaa1111"), (None, "no_map"))
        self.assertEqual(mp.resolve_pin(payload(), "aaaa1111", "other_map"), (None, "map_mismatch"))
        self.assertEqual(mp.resolve_pin(payload(), "zzzz9999"), (None, "not_found"))
        self.assertEqual(mp.resolve_pin(payload(), None), (None, "not_found"))
        self.assertEqual(mp.resolve_pin(payload(), "aaaa1111", "hall")[1], "ok")

    def test_invalid_coordinates_rejected(self):
        for bad in (float("nan"), float("inf"), 1e9, "1", None):
            pl = payload([dict(PIN_A, x=bad)])
            self.assertEqual(mp.resolve_pin(pl, "aaaa1111"), (None, "invalid"), bad)

    def test_payload_shape(self):
        pl = payload()
        self.assertEqual(pl["loaded_map"], "hall")
        self.assertEqual(sorted(pl["pins"][0]), ["color", "id", "name", "x", "y", "yaw_deg"])
        json.dumps(pl)
        self.assertIsNone(mp.pins_payload(None, [])["loaded_map"])


# ---------------------------------------------------------------------------
# behavior_director
# ---------------------------------------------------------------------------
class PinDirectorTests(unittest.TestCase):
    make = tb.DirectorTickTests.make
    run_ticks = tb.DirectorTickTests.run_ticks
    moved = staticmethod(tb.DirectorTickTests.moved)
    _safety_cases = tb.DirectorTickTests._safety_cases

    def setUp(self):
        self._orig = (bd.Twist, bd.PoseStamped, bd.String, bd.Bool)
        bd.Twist, bd.PoseStamped, bd.String, bd.Bool = tb._Twist, tb._Pose, tb._Str, tb._Bool

    def tearDown(self):
        bd.Twist, bd.PoseStamped, bd.String, bd.Bool = self._orig

    def node(self, rules=None, pl="default", **kw):
        n = self.make(rules if rules is not None else [pin_rule()], **kw)
        n.pins_stale_sec = 30.0
        n.pin_goal_timeout_sec = 120.0
        n._policy_pin_goal_key = None
        n._policy_pin_logged = None
        self.feed(n, payload() if pl == "default" else pl)
        n._clock.t += 10  # ゴールの5秒再送スロットルを越える
        return n

    @staticmethod
    def feed(n, pl):
        n._pins_payload = pl
        n._pins_rx_time = n._clock.now() if pl is not None else None

    @staticmethod
    def mode(n):
        return json.loads(n._command_pub.msgs[-1].data)["mode"]

    # --- 正常系 ---
    def test_goal_sent_once_with_pin_pose(self):
        n = self.node()
        modes = self.run_ticks(n, 20.0)
        self.assertEqual(len(n._goal_pub.msgs), 1)  # 1回だけ(2秒毎の再送などはしない)
        g = n._goal_pub.msgs[0]
        self.assertEqual(g.header.frame_id, "map")
        self.assertAlmostEqual(g.pose.position.x, 2.0)
        self.assertAlmostEqual(g.pose.position.y, 1.0)
        self.assertAlmostEqual(g.pose.orientation.z, 0.7071067811865476, places=6)  # yaw 90度
        self.assertIn("POLICY_GO_PIN", modes)
        self.assertFalse([t for t in n._cmd_vel_pub.msgs if t.linear.x or t.angular.z])  # cmd_velは出さない

    def test_arrival_holds(self):
        n = self.node()
        self.run_ticks(n, 1.0)
        n._get_pose = lambda: {"x": 2.1, "y": 1.0, "yaw": 0.0}
        self.assertEqual(set(self.run_ticks(n, 1.0)), {"POLICY_AT_PIN"})
        self.assertEqual(len(n._goal_pub.msgs), 1)

    def test_timeout_stops_and_cancels(self):
        n = self.node()
        self.run_ticks(n, 1.0)
        before = len(n._cancel_goal_pub.msgs)
        n._clock.t += 130
        self.feed(n, payload())
        modes = self.run_ticks(n, 1.0)
        self.assertEqual(set(modes), {"POLICY_AT_PIN"})
        self.assertGreater(len(n._cancel_goal_pub.msgs), before)
        self.assertEqual(len(n._goal_pub.msgs), 1)

    def test_deactivation_cancels_goal(self):
        n = self.node([pin_rule(when={"crowd_level": ["crowded"]})], crowd="crowded")
        n._clock.t += 10
        self.run_ticks(n, 1.0)
        self.assertEqual(len(n._goal_pub.msgs), 1)
        before = len(n._cancel_goal_pub.msgs)
        n._crowd_state["level"] = "calm"
        n._crowd_state["person_target"] = None
        self.run_ticks(n, 2.0)
        self.assertGreater(len(n._cancel_goal_pub.msgs), before)
        self.assertEqual(len(n._goal_pub.msgs), 1)

    def test_pin_moved_resends(self):
        n = self.node()
        self.run_ticks(n, 1.0)
        self.assertEqual(len(n._goal_pub.msgs), 1)
        n._clock.t += 10
        self.feed(n, payload([dict(PIN_A, x=4.0, y=4.0), PIN_B]))
        self.run_ticks(n, 1.0)
        self.assertEqual(len(n._goal_pub.msgs), 2)
        self.assertAlmostEqual(n._goal_pub.msgs[1].pose.position.x, 4.0)

    def test_rename_does_not_change_goal(self):
        n = self.node()
        self.run_ticks(n, 1.0)
        self.feed(n, payload([dict(PIN_A, name="別名"), PIN_B]))
        self.run_ticks(n, 5.0)
        self.assertEqual(len(n._goal_pub.msgs), 1)

    def test_pin_inside_boundary_ok(self):
        n = self.node()
        n._boundary = {"enabled": True, "center_x": 0.0, "center_y": 0.0, "width": 10.0, "height": 10.0, "yaw": 0.0}
        self.run_ticks(n, 1.0)
        self.assertEqual(len(n._goal_pub.msgs), 1)

    # --- ピンが使えない: 走行しない ---
    def assertNoMotion(self, n, expected="POLICY_PIN_UNAVAILABLE", seconds=3.0):
        modes = self.run_ticks(n, seconds)
        self.assertEqual(set(modes), {expected})
        self.assertFalse(self.moved(n))

    def test_missing_pin_is_safe(self):
        self.assertNoMotion(self.node([pin_rule("zzzz9999")]))

    def test_no_pins_data_is_safe(self):
        self.assertNoMotion(self.node(pl=None))

    def test_no_map_loaded_is_safe(self):
        self.assertNoMotion(self.node(pl=payload(loaded=None)))

    def test_other_map_is_safe(self):
        self.assertNoMotion(self.node([pin_rule(params={"pin_map": "other"})]))

    def test_deleted_pin_after_start_cancels(self):
        n = self.node()
        self.run_ticks(n, 1.0)
        self.assertEqual(len(n._goal_pub.msgs), 1)
        before = len(n._cancel_goal_pub.msgs)
        self.feed(n, payload([PIN_B]))  # 削除された
        self.assertEqual(set(self.run_ticks(n, 2.0)), {"POLICY_PIN_UNAVAILABLE"})
        self.assertGreater(len(n._cancel_goal_pub.msgs), before)
        self.assertEqual(len(n._goal_pub.msgs), 1)

    def test_stale_pins_feed_is_safe(self):
        n = self.node()
        n._clock.t += 100  # 配信が30秒以上途絶えた
        self.assertNoMotion(n)

    def test_out_of_bounds_pin_rejected_not_clamped(self):
        n = self.node()
        n._boundary = {"enabled": True, "center_x": 0.0, "center_y": 0.0, "width": 1.0, "height": 1.0, "yaw": 0.0}
        self.assertNoMotion(n)
        self.assertEqual(n._goal_pub.msgs, [])  # クランプして別の場所へ送ることもしない

    def test_boundary_fixed_later_sends(self):
        n = self.node()
        n._boundary = {"enabled": True, "center_x": 0.0, "center_y": 0.0, "width": 1.0, "height": 1.0, "yaw": 0.0}
        self.run_ticks(n, 1.0)
        self.assertFalse(n._goal_pub.msgs)
        n._boundary = {"enabled": False}
        self.run_ticks(n, 1.0)
        self.assertEqual(len(n._goal_pub.msgs), 1)

    def test_keepout_pin_rejected(self):
        n = self.node()
        n._keepout_zones = [{"min_x": 1.5, "max_x": 2.5, "min_y": 0.5, "max_y": 1.5}]
        self.assertNoMotion(n)

    def test_boundary_unreadable_rejected(self):
        n = self.node()
        n._boundary = {"enabled": True}  # 範囲が読めない
        modes = self.run_ticks(n, 1.0)
        self.assertFalse(n._goal_pub.msgs)
        self.assertNotIn("POLICY_GO_PIN", modes)

    def test_unavailable_warned_once(self):
        n = self.node([pin_rule("zzzz9999")])
        self.run_ticks(n, 5.0)
        warns = [m for lvl, m in n._log.lines if lvl == "warn" and "goto_pin" in m]
        self.assertEqual(len(warns), 1)

    # --- 総当たりの安全状態 ---
    def test_safety_states_override_pin_goal(self):
        for crowd in ("calm", "crowded"):
            for expected, setter in self._safety_cases():
                label = (crowd, expected)
                n = self.node(crowd=crowd)
                setter(n)
                refresh = {
                    "MANUAL_OVERRIDE": lambda: setter(n),
                    "NEAR_PERSON_HOLD": lambda: setter(n),
                    "CAMERA_OBSTACLE_HOLD": lambda: setter(n),
                }.get(expected)
                modes = []
                for _ in range(10):
                    if refresh:
                        refresh()
                    self.feed(n, payload())
                    modes += self.run_ticks(n, 0.1)
                self.assertEqual(set(modes), {expected}, label)
                self.assertFalse(self.moved(n), label)  # ゴールもcmd_velも出さない

    def test_goal_rearmed_after_safety_hold(self):
        n = self.node()
        self.run_ticks(n, 1.0)
        self.assertEqual(len(n._goal_pub.msgs), 1)
        n._near_person_stop_detected = True
        n._last_near_person_stop_receive_time = n._clock.now()
        self.run_ticks(n, 0.5)
        self.assertEqual(len(n._goal_pub.msgs), 1)  # ホールド中は送らない
        n._near_person_stop_detected = False
        n._clock.t += 10
        n._last_near_person_stop_receive_time = n._clock.now()  # 【2026-11-04】検出ノードは出し続ける(途絶は安全側で停止になるため)
        self.feed(n, payload())
        self.run_ticks(n, 1.0)
        self.assertEqual(len(n._goal_pub.msgs), 2)  # 解除後に1回だけ送り直す

    def test_existing_rules_unchanged_without_pin_rules(self):
        n = self.make()
        n._clock.t += 3
        self.assertEqual(set(self.run_ticks(n, 1.0)), {"APPROACH_PERSON"})
        self.assertTrue(n._goal_pub.msgs)


if __name__ == "__main__":
    unittest.main()
