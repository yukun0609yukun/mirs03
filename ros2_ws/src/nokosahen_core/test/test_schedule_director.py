"""スケジュールと behavior_director の結合テスト(2026-10-23)。docs/schedule.md。

rclpy不要(未インストール時はスタブ注入)。ロボット・ROSには一切触れない(cmd_vel/ゴールはスタブのpublisherに溜まるだけ)。
確認すること:
- set_boundary: directorが /nokosahen/boundary_override へ要求(解除/差し替え)を1Hzで配信し、項目が終われば「上書きなし」になる。
  directorは /nokosahen/travel_boundary を自分では出さない(ダッシュボードのBoundaryControllerと競合しないため)。
- 範囲の要求が走行範囲に反映されるまで、directorはゴールを出さない(古い範囲で動き出さない)。
- 起動直後のスケジュールは止まっている。「実行」の合図で動き、「停止」の合図でその場で待機+ゴール取り消し+ルール表へ戻る。
- 範囲を解除していても、安全層(非常停止・立入禁止・人が近すぎる)は常に優先される。
- /nokosahen/robot_clock(機体の時刻、1Hz)。時計が合っていない(年<2026)ときは clock_suspect。
- 状態配信(/nokosahen/behavior_tree/status)の schedule(running / active_entry_id / next_entry_at)。
実行: python3 -I ros2_ws/src/nokosahen_core/test/test_schedule_director.py
"""

import datetime
import json
import sys
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(TEST_DIR.parent))

import test_behavior_rules as tb  # noqa: E402
import test_behavior_tree_director as ttd  # noqa: E402
from nokosahen_core import behavior_rules as br  # noqa: E402
from nokosahen_core import schedule_plan as sp  # noqa: E402

bd = tb.bd
PIN_A = {"type": "pin", "pin_id": "pinA", "pin_name": "ピンA"}
RECT = {"x_min": 0.0, "y_min": -1.0, "x_max": 4.0, "y_max": 2.0}


def E(eid, start, end=None, dest=None, policy="stay", boundary=None, **state):
    e = {"id": eid, "start": start, "destination": dest or {"type": "stay"}, "state": dict({"policy": policy}, **state)}
    if end:
        e["end"] = end
    if boundary:
        e["boundary"] = boundary
    return e


def schedule_tree(*entries, **kw):
    doc = {"version": 1, "name": "テスト", "entries": list(entries)}
    doc.update(kw)
    errors, _w, norm = sp.validate_schedule(doc)
    assert not errors, errors
    return sp.compile_schedule(norm)


class Msg:
    def __init__(self, text):
        self.data = text


def cmd(name):
    return Msg(json.dumps({"cmd": name}))


@unittest.skipIf(bd is None, f"behavior_director import失敗: {tb._BD_ERR}")
class ScheduleDirectorBase(ttd.TreeDirectorBase):
    def snode(self, tree, rules=None, armed=True, when=None, **kw):
        n = self.node(tree, rules, now=when or tb.dt(12, 30), **kw)
        n._boundary_override_pub = tb._Pub()
        n._robot_clock_pub = tb._Pub()
        n._display_request_pub = tb._Pub()
        n._boundary_override_req = None
        n._schedule_stop_hold_until = None
        n._schedule_stop_hold_sec = 3.0
        n._utc_offset_hours = None
        if armed and getattr(n, "_bt", None) is not None:
            n._bt.arm_schedule(True)
        n._rules_tick()
        return n

    @staticmethod
    def overrides(n):
        return [json.loads(m.data) for m in n._boundary_override_pub.msgs]

    @staticmethod
    def status(n):
        return json.loads(n._bt_status_pub.msgs[-1].data)

    def set_time(self, n, when):
        n._bt._clock = lambda: when


class BoundaryOverrideTests(ScheduleDirectorBase):
    TREE = None

    def tree(self):
        return schedule_tree(
            E("lunch", "12:00", "13:00", boundary={"mode": "release"}),
            E("pm", "13:00", "14:00", boundary={"mode": "set", "rect": RECT}),
            E("evening", "14:00", "15:00"),
        )

    def test_release_is_published_while_entry_is_active(self):
        n = self.snode(self.tree())
        last = self.overrides(n)[-1]
        self.assertEqual((last["active"], last["enabled"]), (True, False))
        self.assertEqual(n._boundary_override_req, {"mode": "release"})

    def test_set_is_published_with_the_rect(self):
        n = self.snode(self.tree(), when=tb.dt(13, 30))
        last = self.overrides(n)[-1]
        self.assertEqual((last["active"], last["enabled"]), (True, True))
        self.assertEqual((last["min_x"], last["max_x"], last["min_y"], last["max_y"]), (0.0, 4.0, -1.0, 2.0))

    def test_entry_without_boundary_and_outside_entries_publish_inactive(self):
        n = self.snode(self.tree(), when=tb.dt(14, 30))
        self.assertFalse(self.overrides(n)[-1]["active"])
        n = self.snode(self.tree(), when=tb.dt(9, 0))  # どの項目にも当たらない
        self.assertFalse(self.overrides(n)[-1]["active"])

    def test_override_ends_when_the_entry_ends(self):
        n = self.snode(self.tree())
        self.assertTrue(self.overrides(n)[-1]["active"])
        self.set_time(n, tb.dt(14, 30))
        n._clock.t += 1
        n._rules_tick()
        self.assertFalse(self.overrides(n)[-1]["active"])

    def test_published_every_rules_tick_so_a_dead_director_expires_it(self):
        n = self.snode(self.tree())
        before = len(self.overrides(n))
        self.run_ticks(n, 3.0)
        self.assertGreaterEqual(len(self.overrides(n)) - before, 3)

    def test_no_override_without_schedule(self):
        n = self.snode("default", armed=False)
        self.run_ticks(n, 2.0)
        self.assertTrue(all(not m["active"] for m in self.overrides(n)))

    def test_director_never_publishes_travel_boundary_itself(self):
        # directorのpublisherは専用トピックだけ。travel_boundaryは購読のみ(ソースで確認)
        src = (TEST_DIR.parent / "nokosahen_core" / "behavior_director.py").read_text(encoding="utf-8")
        self.assertNotIn('create_publisher(String, "/nokosahen/travel_boundary"', src)
        self.assertNotIn("create_publisher(\n            String, \"/nokosahen/travel_boundary\"", src)
        self.assertIn("BOUNDARY_OVERRIDE_TOPIC", src)

    def test_status_carries_boundary_override_and_schedule(self):
        n = self.snode(self.tree())
        s = self.status(n)
        self.assertEqual(s["schedule"]["boundary_override"], {"mode": "release"})
        self.assertTrue(s["schedule"]["running"])
        self.assertEqual(s["schedule"]["active_entry_id"], "lunch")
        self.assertEqual(s["schedule"]["next_entry_id"], "pm")
        self.assertEqual(s["schedule"]["next_entry_at"], "2026-10-06T13:00:00")
        self.assertEqual(s["source"], "schedule")


class PendingGateTests(ScheduleDirectorBase):
    def tree(self):
        return schedule_tree(E("go", "12:00", "13:00", dest=PIN_A, boundary={"mode": "release"}))

    def test_no_goal_until_the_override_reaches_travel_boundary(self):
        n = self.snode(self.tree())
        modes = self.run_ticks(n, 3.0)
        self.assertEqual(set(modes), {"POLICY_STAY"})
        self.assertEqual(n._goal_pub.msgs, [])
        self.assertFalse(self.nonzero(n))
        # ダッシュボードが合成した範囲(override_active)を配信したら動き出す
        n._boundary = {"enabled": False, "override_active": True}
        n._clock.t += 10
        modes = self.run_ticks(n, 2.0)
        self.assertEqual(len(n._goal_pub.msgs), 1)
        self.assertIn(modes[-1], ("POLICY_GO_PIN",))

    def test_without_boundary_node_goal_goes_out_immediately(self):
        n = self.snode(schedule_tree(E("go", "12:00", "13:00", dest=PIN_A)))
        self.run_ticks(n, 1.0)
        self.assertEqual(len(n._goal_pub.msgs), 1)

    def test_set_rect_is_enforced_by_existing_gates_once_reflected(self):
        # 差し替えた範囲が反映された後は、範囲外のピンへ行かない(既存の出力ゲート)
        tree = schedule_tree(E("go", "12:00", "13:00", dest=PIN_A, boundary={
            "mode": "set", "rect": {"x_min": 10.0, "y_min": 10.0, "x_max": 11.0, "y_max": 11.0}}))
        n = self.snode(tree)
        n._boundary = {"enabled": True, "override_active": True, "center_x": 10.5, "center_y": 10.5,
                       "width": 1.0, "height": 1.0, "yaw": 0.0}
        self.assertEqual(set(self.run_ticks(n, 3.0)), {"POLICY_PIN_UNAVAILABLE"})
        self.assertEqual(n._goal_pub.msgs, [])

    def test_released_range_lets_the_goal_go_to_a_pin_outside_the_users_range(self):
        n = self.snode(self.tree())
        # 反映後のtravel_boundaryは「制限なし」。以前のユーザーの範囲(狭い)は関係しない
        n._boundary = {"enabled": False, "override_active": True}
        self.run_ticks(n, 1.0)
        self.assertEqual(len(n._goal_pub.msgs), 1)


class SafetyStillWinsTests(ScheduleDirectorBase):
    def tree(self):
        return schedule_tree(E("go", "12:00", "13:00", dest=PIN_A, boundary={"mode": "release"}))

    def reflected(self, n):
        n._boundary = {"enabled": False, "override_active": True}
        n._clock.t += 10

    def test_estop_blocks_motion_even_when_released(self):
        n = self.snode(self.tree())
        self.reflected(n)
        n._hardware_estop = True
        modes = self.run_ticks(n, 3.0)
        self.assertEqual(set(modes), {"ESTOP_HOLD"})
        self.assertEqual(n._goal_pub.msgs, [])
        self.assertFalse(self.nonzero(n))

    def test_keepout_blocks_motion_even_when_released(self):
        n = self.snode(self.tree())
        self.reflected(n)
        n._get_pose = lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0}
        n._keepout_zones = [{"min_x": -1, "max_x": 1, "min_y": -1, "max_y": 1}]
        modes = self.run_ticks(n, 3.0)
        self.assertEqual(set(modes), {"KEEPOUT_ZONE_HOLD"})
        self.assertEqual(n._goal_pub.msgs, [])
        self.assertFalse(self.nonzero(n))

    def test_near_person_blocks_motion_even_when_released(self):
        n = self.snode(self.tree())
        self.reflected(n)
        n._near_person_stop_detected = True
        n._last_near_person_stop_receive_time = n._clock.now()
        modes = []
        for _ in range(10):
            n._last_near_person_stop_receive_time = n._clock.now()
            modes += self.run_ticks(n, 0.1)
        self.assertEqual(set(modes), {"NEAR_PERSON_HOLD"})
        self.assertEqual(n._goal_pub.msgs, [])

    def test_release_does_not_clear_out_of_bounds_hold_of_the_reflected_range(self):
        # 差し替えた範囲(反映済み)の外にいれば、従来どおり範囲外で止まる
        tree = schedule_tree(E("a", "12:00", "13:00", dest=PIN_A, boundary={"mode": "set", "rect": RECT}))
        n = self.snode(tree)
        n._boundary = {"enabled": True, "override_active": True, "center_x": 100.0, "center_y": 100.0,
                       "width": 1.0, "height": 1.0, "yaw": 0.0}
        n._get_pose = lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.assertEqual(set(self.run_ticks(n, 2.0)), {"OUT_OF_BOUNDS_HOLD"})
        self.assertEqual(n._goal_pub.msgs, [])


class RunStopTests(ScheduleDirectorBase):
    def tree(self, **kw):
        return schedule_tree(
            E("stay", "12:00", "13:00", dest=PIN_A, policy="stay"), **kw)

    def test_unarmed_schedule_is_idle_and_rules_table_runs(self):
        n = self.snode(self.tree(), rules=[tb.rule("r", policy="quiet")], armed=False)
        n._rules_tick()
        self.assertEqual(n._rule_policy, "quiet")
        s = self.status(n)
        self.assertFalse(s["schedule"]["running"])
        self.assertFalse(s["schedule"]["armed"])
        self.assertIsNone(s["schedule"]["active_entry_id"])
        self.assertEqual(n._goal_pub.msgs, [])
        self.assertFalse(self.overrides(n)[-1]["active"])

    def test_run_command_arms_it_immediately(self):
        n = self.snode(self.tree(), rules=[tb.rule("r", policy="quiet")], armed=False)
        n._on_schedule_command(cmd("run"))
        self.assertTrue(n._bt.user_active)
        self.assertIn("goto_pin", n._rule_policy)
        self.assertTrue(self.status(n)["schedule"]["running"])
        self.assertEqual(self.status(n)["schedule"]["active_entry_id"], "stay")

    def test_run_on_startup_runs_without_a_command(self):
        import os
        import tempfile
        from nokosahen_core import behavior_tree as bt
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "tree.json")
            for run_on_startup, expected in ((True, True), (False, False)):
                with open(path, "w", encoding="utf-8") as f:
                    f.write(bt.document_text(bt.validate_document(self.tree(run_on_startup=run_on_startup))[2]))
                runner = bt.TreeRunner(path, clock=lambda: tb.dt(12, 30), mono=lambda: 1000.0)
                runner.check_reload()
                self.assertEqual(runner.user_active, expected)

    def _startup_node(self, d, delay=120.0, run_on_startup=True):
        import os
        from nokosahen_core import behavior_tree as bt
        path = os.path.join(d, "tree.json")
        with open(path, "w", encoding="utf-8") as f:
            f.write(bt.document_text(bt.validate_document(self.tree(run_on_startup=run_on_startup))[2]))
        n = self.snode(self.tree(), rules=[tb.rule("r", policy="quiet")], armed=False)
        self.when = tb.dt(12, 30)
        n._bt = bt.TreeRunner(path, clock=lambda: self.when, mono=lambda: n._clock.t)
        n._bt.startup_delay_sec = delay  # directorのパラメータ schedule_startup_delay_sec(既定120)
        return n

    def test_run_on_startup_waits_for_the_delay(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            n = self._startup_node(d)
            n._rules_tick()
            self.assertFalse(n._bt.user_active)  # 起動直後は走らない
            self.assertTrue(self.status(n)["schedule"]["startup_pending"])
            self.assertFalse(self.status(n)["schedule"]["running"])
            n._clock.t += 60
            n._rules_tick()
            self.assertFalse(n._bt.user_active)  # 60秒ではまだ
            n._clock.t += 61
            n._rules_tick()
            self.assertTrue(n._bt.user_active)   # 120秒を過ぎた
            self.assertTrue(self.status(n)["schedule"]["running"])
            self.assertFalse(self.status(n)["schedule"]["startup_pending"])

    def test_run_on_startup_waits_for_a_valid_clock(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            n = self._startup_node(d)
            self.when = datetime.datetime(2025, 1, 1, 12, 30)  # 年<2026=clock_suspect
            n._rules_tick()
            n._clock.t += 300  # 起動から5分たっても
            n._rules_tick()
            self.assertFalse(n._bt.user_active)
            self.assertTrue(self.status(n)["schedule"]["startup_pending"])
            self.when = tb.dt(12, 30)  # 時計が直ったら始まる
            n._clock.t += 1
            n._rules_tick()
            self.assertTrue(n._bt.user_active)

    def test_stop_command_cancels_a_pending_startup_run(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            n = self._startup_node(d)
            n._rules_tick()
            n._on_schedule_command(cmd("stop"))
            n._clock.t += 300
            n._rules_tick()
            self.assertFalse(n._bt.user_active)

    def test_run_on_startup_false_never_auto_runs(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            n = self._startup_node(d, run_on_startup=False)
            n._rules_tick()
            n._clock.t += 300
            n._rules_tick()
            self.assertFalse(n._bt.user_active)
            self.assertFalse(self.status(n)["schedule"]["startup_pending"])

    def test_stop_command_holds_cancels_and_returns_to_rules(self):
        n = self.snode(self.tree(), rules=[tb.rule("r", policy="quiet")])
        n._boundary = {"enabled": False}
        self.run_ticks(n, 2.0)
        self.assertEqual(len(n._goal_pub.msgs), 1)
        n._policy_goal_outstanding = True
        cancels = len(n._cancel_goal_pub.msgs)
        zero = len(n._cmd_vel_pub.msgs)
        n._on_schedule_command(cmd("stop"))
        # 即座に: ゼロ速度、ゴール取り消し、範囲の上書きなし
        self.assertGreater(len(n._cmd_vel_pub.msgs), zero)
        self.assertEqual((n._cmd_vel_pub.msgs[zero].linear.x, n._cmd_vel_pub.msgs[zero].angular.z), (0.0, 0.0))
        self.assertGreater(len(n._cancel_goal_pub.msgs), cancels)
        self.assertTrue(n._cancel_goal_pub.msgs[-1].data)
        self.assertFalse(self.overrides(n)[-1]["active"])
        self.assertFalse(n._bt.user_active)
        self.assertFalse(self.status(n)["schedule"]["running"])
        self.assertFalse(n._policy_goal_outstanding)
        # 直後はその場で待機(hold_stop)
        self.assertTrue(n._rule_effects.hold_position)
        self.assertEqual(n._rule_policy, "stay")
        self.assertEqual(set(self.run_ticks(n, 1.0)), {"POLICY_STAY"})
        self.assertEqual(len(n._goal_pub.msgs), 1)  # 新しいゴールは出ない
        # 待機の時間が過ぎたら元のルール表の動作(ここでは quiet)
        n._schedule_stop_hold_until = bd.time.monotonic() - 1
        n._clock.t += 1
        n._rules_tick()
        self.assertEqual(n._rule_policy, "quiet")
        self.assertIsNone(n._schedule_stop_hold_until)

    def test_stop_when_not_running_does_nothing_to_the_robot(self):
        n = self.snode(self.tree(), rules=[tb.rule("r", policy="quiet")], armed=False)
        cancels = len(n._cancel_goal_pub.msgs)
        n._on_schedule_command(cmd("stop"))
        self.assertEqual(len(n._cancel_goal_pub.msgs), cancels)
        self.assertIsNone(n._schedule_stop_hold_until)
        self.assertEqual(n._rule_policy, "quiet")

    def test_garbage_commands_are_ignored(self):
        n = self.snode(self.tree(), armed=False)
        for text in ("not json", "[]", "{}", '{"cmd": "explode"}', '{"cmd": 1}'):
            n._on_schedule_command(Msg(text))
            self.assertFalse(n._bt.user_active, text)

    def test_stop_then_run_again(self):
        n = self.snode(self.tree())
        n._on_schedule_command(cmd("stop"))
        self.assertFalse(n._bt.user_active)
        n._on_schedule_command(cmd("run"))
        self.assertTrue(n._bt.user_active)

    def test_user_edited_tree_is_not_affected_by_commands(self):
        tree = {"version": 1, "min_hold_seconds": 0, "root": {"type": "quiet"}}
        n = self.snode(tree, armed=False)
        n._on_schedule_command(cmd("stop"))
        self.assertTrue(n._bt.user_active)
        self.assertEqual(n._rule_policy, "quiet")


class RobotClockTests(ScheduleDirectorBase):
    def test_payload_fields_and_tz(self):
        local = datetime.datetime(2026, 10, 23, 12, 34, 56)
        p = bd.robot_clock_payload(local, 1_800_000_000.0, 123.4, utc_offset_hours=9.0)
        self.assertEqual(p["iso"], "2026-10-23T12:34:56+09:00")
        self.assertEqual(p["tz"], "Asia/Tokyo")
        self.assertFalse(p["clock_suspect"])
        self.assertEqual((p["epoch"], p["since_boot_sec"]), (1_800_000_000.0, 123.4))
        self.assertEqual(set(p), {"epoch", "iso", "tz", "clock_suspect", "since_boot_sec"})
        utc = bd.robot_clock_payload(local, 1.0, 0.0, utc_offset_hours=0.0)
        self.assertEqual((utc["iso"], utc["tz"]), ("2026-10-23T12:34:56+00:00", "UTC"))
        other = bd.robot_clock_payload(local, 1.0, 0.0, utc_offset_hours=5.5)
        self.assertEqual((other["iso"], other["tz"]), ("2026-10-23T12:34:56+05:30", "UTC+5.5"))

    def test_clock_suspect_when_year_before_2026(self):
        p = bd.robot_clock_payload(datetime.datetime(1970, 1, 1, 0, 0, 5), 5.0, 5.0, utc_offset_hours=9.0)
        self.assertTrue(p["clock_suspect"])
        p = bd.robot_clock_payload(datetime.datetime(2025, 12, 31, 23, 59, 59), 5.0, 5.0, utc_offset_hours=9.0)
        self.assertTrue(p["clock_suspect"])
        p = bd.robot_clock_payload(datetime.datetime(2026, 1, 1, 0, 0, 0), 5.0, 5.0, utc_offset_hours=9.0)
        self.assertFalse(p["clock_suspect"])

    def test_system_timezone_is_used_without_override(self):
        p = bd.robot_clock_payload(datetime.datetime(2026, 10, 23, 12, 0, 0), 1_800_000_000.0, 1.0)
        self.assertRegex(p["iso"], r"^2026-10-23T12:00:00[+-]\d\d:\d\d$")

    def test_published_every_rules_tick_with_the_trees_clock(self):
        n = self.snode(schedule_tree(E("a", "12:00", "13:00")), when=tb.dt(12, 30))
        n._utc_offset_hours = 9.0
        before = len(n._robot_clock_pub.msgs)
        self.run_ticks(n, 3.0)
        self.assertGreaterEqual(len(n._robot_clock_pub.msgs) - before, 3)
        p = json.loads(n._robot_clock_pub.msgs[-1].data)
        self.assertEqual(p["iso"], "2026-10-06T12:30:00+09:00")  # 木の時刻条件が見ている時刻そのもの
        self.assertFalse(p["clock_suspect"])

    def test_published_without_tree_and_with_wrong_clock(self):
        n = self.snode(None, armed=False)
        n._bt = None
        n._utc_offset_hours = 9.0
        n._rules_tick()
        self.assertTrue(n._robot_clock_pub.msgs)
        n = self.snode(schedule_tree(E("a", "12:00", "13:00")), when=datetime.datetime(1970, 1, 1, 9, 0))
        n._rules_tick()
        self.assertTrue(json.loads(n._robot_clock_pub.msgs[-1].data)["clock_suspect"])
        # 時計が合っていない間はスケジュールの項目は選ばれず、ルール表のまま
        self.assertFalse(self.status(n)["schedule"]["active_entry_id"])
        self.assertTrue(self.status(n)["schedule"]["clock_suspect"])

    def test_publisher_failure_does_not_stop_the_rules_tick(self):
        n = self.snode(schedule_tree(E("a", "12:00", "13:00")))

        class Boom:
            def publish(self, m):
                raise RuntimeError("boom")

        n._robot_clock_pub = Boom()
        n._rules_tick()
        self.assertTrue(any(lvl == "warn" for lvl, _ in n._log.lines))
        self.assertTrue(self.status(n)["schedule"]["running"])


def DE(eid, start, end, mode, **kw):
    e = E(eid, start, end, **kw)
    e["display"] = {"mode": mode}
    return e


class DisplayRequestTests(ScheduleDirectorBase):
    """頭部ディスプレイの画面の要求 /nokosahen/display_request(2026-10-23)。directorは配信するだけでラズパイには触れない。"""

    def tree(self):
        return schedule_tree(
            DE("peak", "12:00", "13:00", "customer"),
            DE("survey", "13:00", "14:00", "survey"),
            E("plain", "14:00", "15:00"),
        )

    @staticmethod
    def reqs(n):
        return [json.loads(m.data) for m in n._display_request_pub.msgs]

    def tick(self, n, seconds=1.0):
        n._clock.t += seconds
        n._rules_tick()

    def test_request_for_the_active_entry_is_published_with_schema(self):
        n = self.snode(self.tree())
        r = self.reqs(n)[-1]
        self.assertEqual((r["mode"], r["source"], r["entry_id"]), ("customer", "schedule", "peak"))
        self.assertIsInstance(r["stamp"], float)
        self.assertEqual(self.status(n)["schedule"]["display_request"], "customer")

    def test_published_once_on_change_then_resent_every_5_seconds(self):
        n = self.snode(self.tree())
        self.assertEqual(len(self.reqs(n)), 1)
        for _ in range(4):
            self.tick(n, 1.0)
        self.assertEqual(len(self.reqs(n)), 1)  # 変化なし・5秒未満は再送しない
        self.tick(n, 1.5)  # 5秒を超えた
        self.assertEqual(len(self.reqs(n)), 2)
        self.tick(n, 5.0)
        self.assertEqual(len(self.reqs(n)), 3)
        self.assertEqual({r["mode"] for r in self.reqs(n)}, {"customer"})

    def test_change_of_entry_publishes_immediately(self):
        n = self.snode(self.tree())
        self.set_time(n, tb.dt(13, 30))
        self.tick(n, 1.0)
        r = self.reqs(n)
        self.assertEqual((r[-1]["mode"], r[-1]["entry_id"]), ("survey", "survey"))
        self.assertEqual(len(r), 2)

    def test_entry_without_display_or_outside_entries_requests_keep(self):
        n = self.snode(self.tree())
        self.set_time(n, tb.dt(14, 30))  # displayを指定していない項目
        self.tick(n, 1.0)
        r = self.reqs(n)[-1]
        self.assertEqual((r["mode"], r["entry_id"]), ("keep", "plain"))
        self.set_time(n, tb.dt(9, 0))  # どの項目にも当たらない
        self.tick(n, 1.0)
        r = self.reqs(n)[-1]
        self.assertEqual((r["mode"], r["entry_id"]), ("keep", None))

    def test_stop_publishes_keep_once_and_then_nothing(self):
        n = self.snode(self.tree())
        n._on_schedule_command(cmd("stop"))
        r = self.reqs(n)
        self.assertEqual(r[-1]["mode"], "keep")
        count = len(r)
        for _ in range(8):
            self.tick(n, 1.0)
        self.assertEqual(len(self.reqs(n)), count)

    def test_nothing_is_published_when_no_schedule_is_used(self):
        n = self.snode("default", armed=False)
        for _ in range(8):
            self.tick(n, 1.0)
        self.assertEqual(self.reqs(n), [])

    def test_not_armed_schedule_publishes_nothing(self):
        n = self.snode(self.tree(), armed=False)
        for _ in range(8):
            self.tick(n, 1.0)
        self.assertEqual(self.reqs(n), [])

    def test_safety_still_wins_and_director_never_talks_to_the_display(self):
        # surveyの要求中でも安全層は変わらず優先(走行指令・ゴールなし)。directorはラズパイ用のAPI/URLを持たない
        n = self.snode(schedule_tree(DE("go", "12:00", "13:00", "survey", dest=PIN_A)))
        n._hardware_estop = True
        modes = self.run_ticks(n, 3.0)
        self.assertEqual(set(modes), {"ESTOP_HOLD"})
        self.assertEqual(n._goal_pub.msgs, [])
        src = (TEST_DIR.parent / "nokosahen_core" / "behavior_director.py").read_text(encoding="utf-8")
        self.assertNotIn("survey_admin", src)
        self.assertNotIn("8083", src)

    def test_publisher_failure_does_not_stop_the_rules_tick(self):
        n = self.snode(self.tree())

        class Boom:
            def publish(self, m):
                raise RuntimeError("boom")

        n._display_request_pub = Boom()
        self.set_time(n, tb.dt(13, 30))
        self.tick(n, 1.0)
        self.assertTrue(any(lvl == "warn" and "display_request" in m for lvl, m in n._log.lines))
        self.assertTrue(self.status(n)["schedule"]["running"])

    def test_parse_display_request_validates(self):
        ok = sp.parse_display_request(json.dumps(sp.display_request_message("survey", "a", 1.0)))
        self.assertEqual(ok, {"mode": "survey", "entry_id": "a"})
        for bad in ("{", "[]", "null", json.dumps({"mode": "survey"}),
                    json.dumps({"mode": "survey", "source": "other"}),
                    json.dumps({"mode": "vr", "source": "schedule"}),
                    json.dumps({"mode": 3, "source": "schedule"}), None, 5):
            self.assertIsNone(sp.parse_display_request(bad), bad)


if __name__ == "__main__":
    unittest.main()
