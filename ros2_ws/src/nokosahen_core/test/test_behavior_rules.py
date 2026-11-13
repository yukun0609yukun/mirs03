"""時間帯・状況ルール層(behavior_rules.py)と、behavior_directorへの組み込みの単体テスト。

rclpy不要(未インストール時はスタブ注入)。実機・ROSには一切触れない。
実行: python ros2_ws/src/nokosahen_core/test/test_behavior_rules.py
"""

import datetime
import json
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def _install_stubs():
    try:
        import rclpy  # noqa: F401
        import geometry_msgs.msg  # noqa: F401
        return
    except ImportError:
        pass

    class _Msg:
        def __init__(self, *a, **k):
            self.data = None
            self.linear = types.SimpleNamespace(x=0.0)
            self.angular = types.SimpleNamespace(z=0.0)

    class _AutoMod(types.ModuleType):
        def __getattr__(self, item):
            if item.startswith("__"):
                raise AttributeError(item)
            return _Msg

    for name in list(sys.modules):
        if name.split(".")[0] in ("rclpy", "geometry_msgs", "std_msgs", "tf2_ros"):
            del sys.modules[name]
    for mod in ("rclpy", "rclpy.node", "rclpy.time", "geometry_msgs", "geometry_msgs.msg",
                "std_msgs", "std_msgs.msg", "tf2_ros"):
        sys.modules[mod] = _AutoMod(mod)


_install_stubs()

from nokosahen_core import behavior_rules as br  # noqa: E402

EXAMPLE = (
    Path(__file__).resolve().parents[2]
    / "nokosahen_bringup" / "config" / "behavior_rules.example.json"
)


def rule(rid="r", when=None, policy="stay", params=None, priority=0, enabled=True, **extra):
    d = {"id": rid, "enabled": enabled, "name": rid, "priority": priority,
         "when": when or {}, "then": {"policy": policy, "params": params or {}}}
    d.update(extra)
    return d


def ruleset(*rules, hold=10):
    return br.parse_rules_text(json.dumps({"version": 1, "min_hold_seconds": hold, "rules": list(rules)}))


def dt(h, m, day=6, s=0):
    # 2026-10-06 は火曜日
    return datetime.datetime(2026, 10, day, h, m, s)


class FakeClocks:
    def __init__(self, start):
        self.dt = start
        self.m = 1000.0

    def clock(self):
        return self.dt

    def mono(self):
        return self.m

    def advance(self, seconds):
        self.dt += datetime.timedelta(seconds=seconds)
        self.m += seconds


def engine_with(rs, start=None):
    fc = FakeClocks(start or dt(12, 0))
    e = br.RuleEngine("", clock=fc.clock, mono=fc.mono)
    e.set_ruleset(rs)
    return e, fc


class TimeRangeTests(unittest.TestCase):
    def test_normal_range_boundaries(self):
        r = ruleset(rule(when={"time_ranges": ["11:30-13:00"]})).rules[0]
        ctx = br.Context()
        self.assertFalse(r.matches(dt(11, 29, s=59), ctx))
        self.assertTrue(r.matches(dt(11, 30), ctx))      # 開始は含む
        self.assertTrue(r.matches(dt(12, 59, s=59), ctx))
        self.assertFalse(r.matches(dt(13, 0), ctx))      # 終了は含まない

    def test_overnight(self):
        r = ruleset(rule(when={"time_ranges": ["22:00-06:00"]})).rules[0]
        ctx = br.Context()
        for h, m, exp in [(21, 59, False), (22, 0, True), (23, 59, True), (0, 0, True),
                          (5, 59, True), (6, 0, False), (12, 0, False)]:
            self.assertEqual(r.matches(dt(h, m), ctx), exp, (h, m))

    def test_end_2400_and_multiple_ranges(self):
        r = ruleset(rule(when={"time_ranges": ["08:00-09:00", "20:00-24:00"]})).rules[0]
        ctx = br.Context()
        self.assertTrue(r.matches(dt(23, 59), ctx))
        self.assertTrue(r.matches(dt(8, 30), ctx))
        self.assertFalse(r.matches(dt(0, 0), ctx))

    def test_bad_time_strings_rejected(self):
        for bad in ["11:30", "25:00-26:00", "11:60-12:00", "11:30-11:30", "1:30-2:00", "a-b", 1130, "11:30-24:01"]:
            with self.assertRaises(br.RuleError, msg=repr(bad)):
                ruleset(rule(when={"time_ranges": [bad]}))

    def test_weekdays_and_dates(self):
        r = ruleset(rule(when={"weekdays": ["tue", "wed"]})).rules[0]
        ctx = br.Context()
        self.assertTrue(r.matches(dt(12, 0, day=6), ctx))   # 火
        self.assertFalse(r.matches(dt(12, 0, day=8), ctx))  # 木
        d = ruleset(rule(when={"dates": ["2026-10-07"]})).rules[0]
        self.assertFalse(d.matches(dt(12, 0, day=6), ctx))
        self.assertTrue(d.matches(dt(12, 0, day=7), ctx))
        with self.assertRaises(br.RuleError):
            ruleset(rule(when={"weekdays": ["xyz"]}))
        with self.assertRaises(br.RuleError):
            ruleset(rule(when={"dates": ["2026-02-30"]}))


class ConditionTests(unittest.TestCase):
    def test_crowd_person(self):
        r = ruleset(rule(when={"crowd_level": ["crowded"], "person_present": True})).rules[0]
        n = dt(12, 0)
        self.assertTrue(r.matches(n, br.Context("crowded", True)))
        self.assertFalse(r.matches(n, br.Context("calm", True)))
        self.assertFalse(r.matches(n, br.Context("crowded", False)))
        self.assertFalse(r.matches(n, br.Context("crowded", None)))
        with self.assertRaises(br.RuleError):
            ruleset(rule(when={"crowd_level": ["packed"]}))

    def test_remaining_ratio(self):
        r = ruleset(rule(when={"remaining_ratio_below": 0.3})).rules[0]
        n = dt(12, 0)
        self.assertTrue(r.matches(n, br.Context(menu_ratios={"A": 0.9, "B": 0.2})))   # 最小で判定
        self.assertFalse(r.matches(n, br.Context(menu_ratios={"A": 0.9, "B": 0.3})))  # 境界は含まない
        self.assertFalse(r.matches(n, br.Context()))                                   # メニュー未登録
        m = ruleset(rule(when={"menu": "A", "remaining_ratio_above": 0.5})).rules[0]
        self.assertTrue(m.matches(n, br.Context(menu_ratios={"A": 0.9, "B": 0.1})))
        self.assertFalse(m.matches(n, br.Context(menu_ratios={"B": 0.9})))

    def test_idle_seconds(self):
        e, fc = engine_with(ruleset(rule(when={"idle_seconds_above": 30}, policy="quiet")))
        self.assertIsNone(e.evaluate(br.Context("calm", False)).active_rule_id)
        fc.advance(31)
        self.assertEqual(e.evaluate(br.Context("calm", False)).active_rule_id, "r")
        e.evaluate(br.Context("calm", True))  # 人物が見えたのでリセット
        fc.advance(1)
        st = e.evaluate(br.Context("calm", False))  # 直後はidle<30秒だが最小保持10秒中
        self.assertEqual(st.active_rule_id, "r")

    def test_build_context(self):
        c = br.build_context(
            {"level": "calm", "person_target": {"x": 1}},
            {"items": [{"name": "A", "initial": 10, "remaining": 2},
                       {"name": "B", "ratio_remaining": 0.5}, {"name": ""}, "x"]})
        self.assertEqual(c.crowd_level, "calm")
        self.assertTrue(c.person_present)
        self.assertAlmostEqual(c.menu_ratios["A"], 0.2)
        self.assertEqual(c.menu_ratios["B"], 0.5)
        c2 = br.build_context({}, {})
        self.assertEqual((c2.crowd_level, c2.person_present, c2.menu_ratios), ("unknown", None, {}))


class EngineTests(unittest.TestCase):
    def test_empty_default_is_neutral(self):
        e = br.RuleEngine("")
        st = e.evaluate(br.Context("crowded", True))
        self.assertEqual(st.policy, "normal")
        self.assertIs(st.effects, br.NEUTRAL)
        self.assertIsNone(st.active_rule_id)

    def test_priority_and_disabled(self):
        e, fc = engine_with(ruleset(
            rule("low", priority=1, policy="quiet"),
            rule("high", priority=9, policy="stay"),
            rule("off", priority=99, policy="approach_disabled", enabled=False),
        ))
        st = e.evaluate(br.Context())
        self.assertEqual(st.active_rule_id, "high")
        self.assertTrue(st.effects.hold_position)

    def test_same_priority_first_wins(self):
        e, fc = engine_with(ruleset(rule("a", policy="quiet"), rule("b", policy="stay")))
        self.assertEqual(e.evaluate(br.Context()).active_rule_id, "a")

    def test_min_hold_hysteresis(self):
        rs = ruleset(rule("crowded", when={"crowd_level": ["crowded"]}, policy="stay"), hold=10)
        e, fc = engine_with(rs)
        self.assertIsNone(e.evaluate(br.Context("calm")).active_rule_id)
        fc.advance(1)
        self.assertEqual(e.evaluate(br.Context("crowded")).active_rule_id, "crowded")  # 初回は即時
        fc.advance(2)
        st = e.evaluate(br.Context("calm"))       # 2秒で外れかけてもまだ保持
        self.assertEqual(st.active_rule_id, "crowded")
        self.assertFalse(st.changed)
        fc.advance(8)                              # 計10秒
        st = e.evaluate(br.Context("calm"))
        self.assertIsNone(st.active_rule_id)
        self.assertTrue(st.changed)

    def test_removed_rule_switches_immediately(self):
        e, fc = engine_with(ruleset(rule("a", policy="stay"), hold=600))
        self.assertEqual(e.evaluate(br.Context()).active_rule_id, "a")
        e.set_ruleset(ruleset())
        self.assertEqual(e.evaluate(br.Context()).policy, "normal")

    def test_activation_counter(self):
        e, fc = engine_with(ruleset(rule("a", when={"crowd_level": ["crowded"]}, policy="stay"), hold=0))
        a0 = e.evaluate(br.Context("calm")).activation
        fc.advance(1)
        a1 = e.evaluate(br.Context("crowded")).activation
        fc.advance(1)
        a2 = e.evaluate(br.Context("crowded")).activation
        self.assertEqual(a1, a0 + 1)
        self.assertEqual(a2, a1)

    def test_next_change_at(self):
        e, fc = engine_with(ruleset(rule(when={"time_ranges": ["13:00-14:00"]})), start=dt(12, 0))
        st = e.evaluate(br.Context())
        self.assertEqual(st.next_change_at, dt(13, 0))
        self.assertIn("next_change_at", st.to_json_dict())

    def test_policy_effects(self):
        rs = ruleset(
            rule("s", policy=["speed_scale"], params={"scale": 0.5}),
        )
        eff = br.effects_for(rs.rules[0].policies, rs.rules[0].params)
        self.assertEqual(eff.speed_scale, 0.5)
        stay = br.effects_for(("stay",), {})
        self.assertTrue(stay.hold_position and not stay.approach_allowed and not stay.crowd_scan_allowed)
        self.assertFalse(br.effects_for(("quiet",), {}).announce_allowed)
        self.assertEqual(br.effects_for(("go_to_entrance",), {}).go_to, "entrance")
        self.assertEqual(br.effects_for(("return_home",), {}).go_to, "home")
        self.assertEqual(br.effects_for(("normal",), {}), br.NEUTRAL)

    def test_state_json_keys(self):
        e, fc = engine_with(ruleset(rule("a", policy=["quiet", "stay"])))
        d = json.loads(json.dumps(e.evaluate(br.Context()).to_json_dict()))
        for k in ("active_rule_id", "active_rule_name", "policy", "params", "since", "next_change_at", "load_error"):
            self.assertIn(k, d)
        self.assertEqual(d["policy"], "quiet+stay")


class ValidationTests(unittest.TestCase):
    def bad(self, text_or_obj):
        text = text_or_obj if isinstance(text_or_obj, str) else json.dumps(text_or_obj)
        with self.assertRaises(br.RuleError):
            br.parse_rules_text(text)

    def test_rejections(self):
        self.bad("not json")
        self.bad("[]")
        self.bad({"rules": [], "extra": 1})
        self.bad({"rules": [rule(extra_key=1)]})
        self.bad({"rules": [rule(when={"nope": 1})]})
        self.bad({"rules": [rule(policy="fly")]})
        self.bad({"rules": [rule(policy=["approach_enabled", "approach_disabled"])]})
        self.bad({"rules": [rule(policy="speed_scale")]})                      # scale必須
        self.bad({"rules": [rule(policy="speed_scale", params={"scale": 0.1})]})
        self.bad({"rules": [rule(policy="speed_scale", params={"scale": 1.5})]})
        self.bad({"rules": [rule(policy="stay", params={"scale": 0.5})]})
        self.bad({"rules": [rule("a"), rule("a")]})                            # id重複
        self.bad({"rules": [rule(priority=True)]})
        self.bad({"rules": [rule(enabled="yes")]})
        self.bad({"rules": [rule(when={"remaining_ratio_below": 1.5})]})
        self.bad('{"rules": [{"id": "a", "priority": NaN, "then": {"policy": "stay"}}]}')
        self.bad('{"rules": [{"id": "a", "then": {"policy": "stay"}, "when": {"idle_seconds_above": Infinity}}]}')
        self.bad({"rules": [rule(policy=["normal", "stay"])]})
        self.bad({"rules": [rule(policy="normal", params={"announce_min_interval_seconds": 1})]})

    def test_accepts_minimal(self):
        self.assertEqual(br.parse_rules_text('{"rules": []}').rules, ())
        self.assertEqual(br.parse_rules_text("{}").rules, ())

    def test_example_file_valid_and_all_disabled(self):
        text = EXAMPLE.read_text(encoding="utf-8")
        rs = br.parse_rules_text(text)
        self.assertEqual(len(rs.rules), 4)
        self.assertTrue(all(not r.enabled for r in rs.rules))
        ok, summary = br.validate_file(str(EXAMPLE))
        self.assertTrue(ok, summary)
        # 全て無効なのでどの状況でも現行動作
        e = br.RuleEngine("")
        e.set_ruleset(rs)
        for h in (0, 12, 23):
            self.assertIs(e.evaluate(br.Context("crowded", True)).effects, br.NEUTRAL)

    def test_example_rules_work_when_enabled(self):
        data = json.loads(EXAMPLE.read_text(encoding="utf-8"))
        for r in data["rules"]:
            r["enabled"] = True
        rs = br.parse_rules_text(json.dumps(data))
        e, fc = engine_with(rs, start=dt(12, 0))
        self.assertEqual(e.evaluate(br.Context("calm")).active_rule_id, "lunch_peak_calling")
        fc.dt = dt(23, 0)
        fc.m += 100
        st = e.evaluate(br.Context("calm"))
        self.assertEqual(st.active_rule_id, "closed_quiet_stay")
        self.assertFalse(st.effects.announce_allowed)
        self.assertTrue(st.effects.hold_position)

    def test_cli(self):
        ok, summary = br.validate_file(str(EXAMPLE))
        self.assertIn("OK", summary)
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "bad.json"
            p.write_text('{"rules":[{"id":"a","then":{"policy":"zzz"}}]}', encoding="utf-8")
            ok, summary = br.validate_file(str(p))
            self.assertFalse(ok)
            self.assertIn("NG", summary)
            self.assertEqual(br.main(["validate", str(p)]), 1)
            self.assertEqual(br.main(["validate", str(EXAMPLE)]), 0)
            self.assertEqual(br.main([]), 2)


class HotReloadTests(unittest.TestCase):
    def test_accept_reject_keep_previous(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "rules.json"
            e = br.RuleEngine(str(p))
            self.assertIsNone(e.check_reload())          # ファイル無し: 空ルール、エラーでもない
            self.assertEqual(e.ruleset.rules, ())
            self.assertIsNone(e.load_error)

            def write(obj_or_text, mtime):
                text = obj_or_text if isinstance(obj_or_text, str) else json.dumps(obj_or_text)
                p.write_text(text, encoding="utf-8")
                os.utime(p, (mtime, mtime))

            write({"rules": [rule("good", policy="quiet")]}, 1_000_000)
            self.assertIn("読み込みました", e.check_reload())
            self.assertEqual([r.id for r in e.ruleset.rules], ["good"])
            self.assertIsNone(e.check_reload())          # 変更なし

            write({"rules": [rule("bad", policy="fly")]}, 1_000_100)
            msg = e.check_reload()
            self.assertIn("拒否", msg)
            self.assertEqual([r.id for r in e.ruleset.rules], ["good"])   # 直前を維持
            self.assertIsNotNone(e.load_error)
            self.assertIsNone(e.check_reload())          # 同じ不正ファイルを毎秒ログしない
            st = e.evaluate(br.Context())
            self.assertEqual(st.load_error, e.load_error)

            write({"rules": [rule("good2", policy="stay")]}, 1_000_200)
            self.assertIn("読み込みました", e.check_reload())
            self.assertEqual([r.id for r in e.ruleset.rules], ["good2"])
            self.assertIsNone(e.load_error)

            e.request_reload()                           # 強制リロード(mtime不変でも)
            self.assertIn("読み込みました", e.check_reload())

            p.unlink()
            self.assertIn("無くなった", e.check_reload())
            self.assertEqual(e.ruleset.rules, ())


# ---------------------------------------------------------------------------
# behavior_director への組み込み(スタブtick)
# ---------------------------------------------------------------------------

try:
    from nokosahen_core import behavior_director as bd  # noqa: E402
    _BD_ERR = None
except Exception as exc:  # pragma: no cover
    bd = None
    _BD_ERR = exc


class _T:
    def __init__(self, sec):
        self.sec = sec

    @property
    def nanoseconds(self):
        return int(self.sec * 1e9)

    def __sub__(self, other):
        return _T(self.sec - other.sec)

    def to_msg(self):
        return None


class _Clock:
    def __init__(self):
        self.t = 1000.0

    def now(self):
        return _T(self.t)


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(("info", m))

    def warn(self, m):
        self.lines.append(("warn", m))

    def error(self, m):
        self.lines.append(("error", m))


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class _Pose:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id="")
        self.pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0),
            orientation=types.SimpleNamespace(z=0.0, w=1.0))


class _Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0)
        self.angular = types.SimpleNamespace(z=0.0)


class _Str:
    def __init__(self):
        self.data = None


class _Bool:
    def __init__(self):
        self.data = None


@unittest.skipIf(bd is None, f"behavior_director import失敗: {_BD_ERR}")
class DirectorTickTests(unittest.TestCase):
    def setUp(self):
        self._orig = (bd.Twist, bd.PoseStamped, bd.String, bd.Bool)
        bd.Twist, bd.PoseStamped, bd.String, bd.Bool = _Twist, _Pose, _Str, _Bool

    def tearDown(self):
        bd.Twist, bd.PoseStamped, bd.String, bd.Bool = self._orig

    def make(self, rules=None, crowd="calm", target=True):
        n = object.__new__(bd.BehaviorDirector)
        n._clock = _Clock()
        n.get_clock = lambda: n._clock
        n._log = _Log()
        n.get_logger = lambda: n._log
        for name in ("_cmd_vel_pub", "_goal_pub", "_cancel_goal_pub", "_command_pub",
                     "_announce_pub", "_announce_context_pub", "_rules_state_pub"):
            setattr(n, name, _Pub())
        n.announce_cooldown_seconds = 8.0
        n.announce_min_interval_seconds = 60.0
        n.enable_direct_cmd_vel = False
        n.approach_speed_mps = 0.75
        n.approach_turn_rps = 0.75
        n.approach_standoff_m = 0.6
        n.approach_goal_min_interval_sec = 2.0
        n.crowd_scan_turn_rps = 0.75
        n.crowd_scan_drive_mps = 0.75
        n.crowd_scan_approach_seconds = 4.0
        n.crowd_scan_approach_distance_m = 1.0
        n.entrance_x = 0.0
        n.entrance_y = 0.0
        n.entrance_yaw = 0.0
        n.approach_dwell_sec = 6.0
        n.return_home_timeout_sec = 30.0
        n.approach_cooldown_sec = 180.0
        n.approach_cooldown_radius_m = 1.0
        n.manual_override_timeout_sec = 1.0
        n.menu_merits = {}
        n.induction_template = "{item}"
        n.generic_initial_text = "本日のメニューをご確認ください。"
        n._boundary = {"enabled": False}
        n._keepout_zones = []
        n.camera_obstacle_timeout_sec = 1.0
        n._camera_obstacle_detected = False
        n._last_camera_obstacle_receive_time = None
        n.near_person_stop_timeout_sec = 1.0
        n._near_person_stop_detected = False
        n._last_near_person_stop_receive_time = None
        n._map_building_active = False
        n._frontier_enabled = False
        n._last_frontier_status_time = None
        n._frontier_status_timeout_sec = 10.0
        n._nav2_goal_pending = False
        n._prev_tick_mode = ""
        n._recommend_enabled = True
        n._engagement_log = types.SimpleNamespace(record_engagement=lambda *a, **k: None)
        n._idle_wiggle_enabled = False
        n._wiggle_direction = 1.0
        n._wiggle_phase_start_time = None
        n._menu_state = {"items": [{"name": "A", "initial": 10, "remaining": 2}], "phase": "initial"}
        n._crowd_state = {"level": crowd, "person_target": {"x": 3.0, "y": 0.0} if target else None}
        n._hardware_estop = False
        n._soft_estop = False
        n._manual_override_raw = False
        n._last_manual_override_time = None
        n._last_announce = ""
        n._last_announce_time = n._clock.now()
        n._last_announce_time_set = False
        n._last_goal_time = n._clock.now()
        n._last_approach_goal_time = n._clock.now()
        n._merit_rotation_index = 0
        n._last_merit_change_time = n._clock.now()
        n._arrived_at_time = None
        n._home_return_active = False
        n._home_return_start_time = None
        n._recent_serve_position = None
        n._recent_serve_time = None
        n.tf_available = False
        n._camera_person_density = None
        n._last_tick_time = n._clock.now()
        n._reset_crowd_scan(n._clock.now())
        n._rule_engine = br.RuleEngine("", clock=lambda: dt(12, 0), mono=lambda: n._clock.t)
        n._rule_effects = br.NEUTRAL
        n._rule_activation = 0
        n._rule_active_id = None
        n._rule_policy = "normal"
        n._policy_goal_done_activation = -1
        n._policy_goal_outstanding = False
        n._policy_goal_start_time = None
        n._rule_error_logged = False
        if rules is not None:
            n._rule_engine.set_ruleset(ruleset(*rules, hold=0))
        n._rules_tick()
        return n

    def run_ticks(self, n, seconds=1.0, step=0.1):
        modes = []
        steps = int(round(seconds / step))
        for i in range(steps):
            n._clock.t += step
            if i % int(round(1.0 / step)) == 0:
                n._rules_tick()
            n._tick()
            modes.append(json.loads(n._command_pub.msgs[-1].data)["mode"])
        return modes

    @staticmethod
    def moved(n):
        twists = [t for t in n._cmd_vel_pub.msgs if t.linear.x != 0.0 or t.angular.z != 0.0]
        return bool(twists) or bool(n._goal_pub.msgs)

    # --- 既定(ルール無し)は現行動作 ---
    def test_no_rules_behaviour_unchanged(self):
        n = self.make()
        n._clock.t += 3  # 接近ゴール再送間隔(2秒)を越える
        modes = self.run_ticks(n, 1.0)
        self.assertEqual(set(modes), {"APPROACH_PERSON"})
        self.assertTrue(n._goal_pub.msgs)
        cmd = json.loads(n._command_pub.msgs[-1].data)
        self.assertIsNone(cmd["active_rule"])
        self.assertEqual(cmd["policy"], "normal")
        for k in ("stamp", "mode", "crowd_level", "announce_text", "estop_active", "idle_wiggle_enabled"):
            self.assertIn(k, cmd)
        self.assertTrue(n._announce_pub.msgs)  # 起動後最初の発話

    def test_rules_state_published(self):
        n = self.make([rule("a", policy="stay")])
        d = json.loads(n._rules_state_pub.msgs[-1].data)
        self.assertEqual(d["active_rule_id"], "a")
        self.assertEqual(d["policy"], "stay")

    def test_rule_error_falls_back_to_neutral(self):
        n = self.make([rule("a", policy="stay")])
        n._rule_engine.evaluate = lambda ctx: 1 / 0
        n._rules_tick()
        self.assertIs(n._rule_effects, br.NEUTRAL)
        self.assertTrue(any(lvl == "error" for lvl, _ in n._log.lines))

    # --- 各方針の効果 ---
    def test_stay(self):
        n = self.make([rule("a", policy="stay")])
        self.assertEqual(set(self.run_ticks(n, 1.0)), {"POLICY_STAY"})
        self.assertFalse(self.moved(n))
        self.assertTrue(n._announce_pub.msgs)  # stayでも発話は続く

    def test_announce_only_and_quiet(self):
        n = self.make([rule("a", policy="announce_only")])
        self.run_ticks(n, 1.0)
        self.assertFalse(self.moved(n))
        self.assertTrue(n._announce_pub.msgs)
        q = self.make([rule("a", policy="quiet")])
        self.run_ticks(q, 1.0)
        self.assertFalse(q._announce_pub.msgs)
        self.assertTrue(q._announce_context_pub.msgs == [])

    def test_approach_disabled_falls_back_to_scan(self):
        n = self.make([rule("a", policy="approach_disabled")])
        modes = self.run_ticks(n, 1.0)
        self.assertNotIn("APPROACH_PERSON", modes)
        self.assertEqual(set(modes), {"CROWD_SCAN"})
        self.assertFalse(n._goal_pub.msgs)

    def test_crowd_scan_disabled_and_wiggle(self):
        n = self.make([rule("a", policy="crowd_scan_disabled")], target=False)
        self.assertEqual(set(self.run_ticks(n, 1.0)), {"POLICY_NO_SCAN"})
        self.assertFalse(self.moved(n))
        w = self.make([rule("a", policy="idle_wiggle_on")], target=False)
        self.assertEqual(set(self.run_ticks(w, 1.0)), {"IDLE_WIGGLE"})
        n2 = self.make([rule("a", policy="idle_wiggle_off")], target=False)
        n2._idle_wiggle_enabled = True
        self.assertEqual(set(self.run_ticks(n2, 1.0)), {"CROWD_SCAN"})

    def test_speed_scale_caps_direct_speed(self):
        n = self.make([rule("a", policy="speed_scale", params={"scale": 0.5})], target=False)
        self.run_ticks(n, 1.0)
        speeds = [abs(t.angular.z) for t in n._cmd_vel_pub.msgs if t.angular.z]
        self.assertTrue(speeds)
        self.assertTrue(all(abs(s - 0.375) < 1e-9 for s in speeds))
        base = self.make(target=False)
        self.run_ticks(base, 1.0)
        base_speeds = [abs(t.angular.z) for t in base._cmd_vel_pub.msgs if t.angular.z]
        self.assertTrue(max(speeds) < max(base_speeds))  # 現状を超えない

    def test_crowded_approach_disabled_no_entrance_goal(self):
        n = self.make([rule("a", policy="approach_disabled")], crowd="crowded")
        n._clock.t += 10
        modes = self.run_ticks(n, 1.0)
        self.assertEqual(set(modes), {"STOP_ANNOUNCE"})
        self.assertFalse(n._goal_pub.msgs)
        base = self.make(crowd="crowded")
        base._clock.t += 10
        self.run_ticks(base, 1.0)
        self.assertTrue(base._goal_pub.msgs)  # 既定は従来どおり入口ゴール

    def test_go_to_entrance_once_and_cancel_on_deactivation(self):
        n = self.make([rule("a", when={"crowd_level": ["crowded"]}, policy="go_to_entrance")], crowd="crowded")
        n._clock.t += 10
        self.run_ticks(n, 20.0)
        self.assertEqual(len(n._goal_pub.msgs), 1)  # 1回だけ
        self.assertIn("POLICY_GO_ENTRANCE", self.run_ticks(n, 0.1))
        before = len(n._cancel_goal_pub.msgs)
        n._crowd_state["level"] = "calm"
        n._crowd_state["person_target"] = None
        self.run_ticks(n, 2.0)  # 方針解除
        self.assertGreater(len(n._cancel_goal_pub.msgs), before)
        self.assertEqual(len(n._goal_pub.msgs), 1)

    def test_go_to_entrance_arrival_holds(self):
        n = self.make([rule("a", policy="go_to_entrance")])
        n._clock.t += 10
        self.run_ticks(n, 1.0)
        self.assertEqual(len(n._goal_pub.msgs), 1)
        n._get_pose = lambda: {"x": 0.1, "y": 0.0, "yaw": 0.0}
        modes = self.run_ticks(n, 1.0)
        self.assertEqual(set(modes), {"POLICY_AT_ENTRANCE"})
        self.assertEqual(len(n._goal_pub.msgs), 1)

    # --- 安全系の優先 ---
    ALL_POLICY_RULES = [
        [rule("p", policy=p, params={"scale": 0.5} if p == "speed_scale"
              else ({"pin_id": "abc12345"} if p == "goto_pin" else {}))]
        for p in br.POLICIES
    ] + [[rule("p", policy=["go_to_entrance"]), ], [rule("p", policy=["quiet", "stay"])],
         [rule("p", policy="normal", params={"announce_min_interval_seconds": 10})]]

    def _safety_cases(self):
        def estop_hw(n):
            n._hardware_estop = True

        def estop_soft(n):
            n._soft_estop = True

        def manual(n):
            n._manual_override_raw = True
            n._last_manual_override_time = n._clock.now()

        def near(n):
            n._near_person_stop_detected = True
            n._last_near_person_stop_receive_time = n._clock.now()

        def camera(n):
            n._camera_obstacle_detected = True
            n._last_camera_obstacle_receive_time = n._clock.now()

        def mapping(n):
            n._map_building_active = True

        def oob(n):
            n._boundary = {"enabled": True, "min_x": 0, "max_x": 1, "min_y": 0, "max_y": 1}
            n._get_pose = lambda: {"x": 5.0, "y": 5.0, "yaw": 0.0}

        def keepout(n):
            n._keepout_zones = [{"min_x": -1, "max_x": 1, "min_y": -1, "max_y": 1}]
            n._get_pose = lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0}

        return [
            ("ESTOP_HOLD", estop_hw), ("ESTOP_HOLD", estop_soft), ("MANUAL_OVERRIDE", manual),
            ("NEAR_PERSON_HOLD", near), ("CAMERA_OBSTACLE_HOLD", camera),
            ("MAPPING_ACTIVE_HOLD", mapping), ("OUT_OF_BOUNDS_HOLD", oob),
            ("KEEPOUT_ZONE_HOLD", keepout),
        ]

    def test_no_policy_overrides_safety(self):
        for rules in self.ALL_POLICY_RULES:
            for crowd in ("calm", "crowded"):
                for expected, setter in self._safety_cases():
                    label = (rules[0]["then"]["policy"], crowd, expected)
                    n = self.make(rules, crowd=crowd)
                    n._clock.t += 10  # 入口ゴールの5秒スロットルを越える
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
                        modes += self.run_ticks(n, 0.1)
                    self.assertEqual(set(modes), {expected}, label)
                    # 方針が何であっても、安全ホールド中は移動指令(cmd_velの非ゼロ・ゴール)を出さない
                    self.assertFalse(self.moved(n), label)

    def test_estop_policy_never_adds_speech_or_motion(self):
        # estop中は従来どおり発話だけ続く。方針は発話を減らせても増やせない。
        base = self.make(crowd="calm")
        base._hardware_estop = True
        self.run_ticks(base, 5.0)
        for rules in self.ALL_POLICY_RULES:
            n = self.make(rules)
            n._hardware_estop = True
            self.run_ticks(n, 5.0)
            self.assertFalse(self.moved(n), rules[0]["then"])
            self.assertLessEqual(len(n._announce_pub.msgs), len(base._announce_pub.msgs), rules[0]["then"])
        q = self.make([rule("q", policy="quiet")])
        q._hardware_estop = True
        self.run_ticks(q, 5.0)
        self.assertFalse(q._announce_pub.msgs)
        # 発話間隔短縮はestop中は適用されない(従来の60秒のまま)
        f = self.make([rule("f", policy="normal", params={"announce_min_interval_seconds": 10})])
        f._hardware_estop = True
        self.run_ticks(f, 30.0)
        self.assertEqual(len(f._announce_pub.msgs), 1)

    def test_goal_not_sent_during_hold_and_rearmed_after(self):
        n = self.make([rule("a", policy="go_to_entrance")])
        n._clock.t += 10
        n._near_person_stop_detected = True
        n._last_near_person_stop_receive_time = n._clock.now()
        self.run_ticks(n, 0.5)
        self.assertEqual(n._goal_pub.msgs, [])
        n._near_person_stop_detected = False
        n._clock.t += 10
        n._last_near_person_stop_receive_time = n._clock.now()  # 【2026-11-04】検出ノードは出し続ける(途絶は安全側で停止になるため)
        self.run_ticks(n, 1.0)
        self.assertEqual(len(n._goal_pub.msgs), 1)

    def test_policy_transition_logged_info(self):
        n = self.make([rule("a", policy="stay")])
        self.assertTrue(any(lvl == "info" and "方針が切り替わりました" in m for lvl, m in n._log.lines))

    def test_announce_interval_shortened_outside_safety(self):
        n = self.make([rule("f", policy="normal", params={"announce_min_interval_seconds": 10})])
        self.run_ticks(n, 35.0)
        self.assertGreaterEqual(len(n._announce_pub.msgs), 3)
        base = self.make()
        self.run_ticks(base, 35.0)
        self.assertEqual(len(base._announce_pub.msgs), 1)


if __name__ == "__main__":
    unittest.main()
