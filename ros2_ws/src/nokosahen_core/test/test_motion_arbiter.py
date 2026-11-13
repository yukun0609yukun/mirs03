"""動きの調停ノード(motion_arbiter.py)の、判定(Arbiter)と、受け取り側の配線のテスト(2026-10-22)。ROS不要。
実行: python ros2_ws/src/nokosahen_core/test/test_motion_arbiter.py
"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core.motion_arbiter import HELPERS, Arbiter, RelaxWatchdog  # noqa: E402

CORE = Path(__file__).resolve().parents[1] / "nokosahen_core"
BRINGUP = Path(__file__).resolve().parents[2] / "nokosahen_bringup" / "launch"


def none_allowed(allow):
    return all(v is False for v in allow.values())


class PriorityTests(unittest.TestCase):
    def setUp(self):
        self.a = Arbiter(plan_stale_sec=2.0)

    def test_idle_allows_both(self):
        owner, allow = self.a.decide(10.0)
        self.assertEqual(owner, "idle")
        self.assertEqual(allow, {"wall_escape": True, "router_rotate": True, "director_direct": True})

    def test_estop_beats_everything(self):
        self.a.router_rotating = True
        self.a.wall_escape_escaping = True
        self.a.wasd = True
        self.a.plan_seen(10.0)
        for flag in ("hard_estop", "soft_estop"):
            setattr(self.a, flag, True)
            owner, allow = self.a.decide(10.0)
            self.assertEqual(owner, "estop")
            self.assertTrue(none_allowed(allow))
            setattr(self.a, flag, False)

    def test_manual_beats_nav2_and_helpers(self):
        self.a.wasd = True
        self.a.router_rotating = True
        self.a.plan_seen(10.0)
        owner, allow = self.a.decide(10.0)
        self.assertEqual(owner, "manual")
        self.assertTrue(none_allowed(allow))

    def test_near_person_beats_nav2_and_helpers(self):
        self.a.near_person = True
        self.a.plan_seen(10.0)
        self.a.wall_escape_escaping = True
        owner, allow = self.a.decide(10.0)
        self.assertEqual(owner, "near_person")
        self.assertTrue(none_allowed(allow))

    # 【2026-11-04】directorが「向く(その場の旋回だけ)」を出している間は、近接停止でも、directorの直接指令だけ許可する
    def test_near_person_allows_only_director_direct_during_face_turn(self):
        self.a.near_person = True
        self.a.plan_seen(10.0)
        self.a.wall_escape_escaping = True
        self.a.set_face_turn(10.0, True)
        owner, allow = self.a.decide(10.2)
        self.assertEqual(owner, "near_person_face_turn")
        self.assertEqual(allow, {"wall_escape": False, "router_rotate": False, "director_direct": True})

    def test_face_turn_notice_expires(self):
        self.a.near_person = True
        self.a.set_face_turn(10.0, True)
        owner, allow = self.a.decide(11.5)  # 1.0秒より古い: directorが落ちた等
        self.assertEqual(owner, "near_person")
        self.assertTrue(none_allowed(allow))

    def test_face_turn_false_clears(self):
        self.a.near_person = True
        self.a.set_face_turn(10.0, True)
        self.a.set_face_turn(10.1, False)
        self.assertEqual(self.a.decide(10.2)[0], "near_person")

    def test_estop_and_manual_still_beat_face_turn(self):
        self.a.set_face_turn(10.0, True)
        self.a.near_person = True
        self.a.soft_estop = True
        self.assertEqual(self.a.decide(10.1)[0], "estop")
        self.a.soft_estop = False
        self.a.wasd = True
        self.assertEqual(self.a.decide(10.1)[0], "manual")

    def test_face_turn_without_near_person_changes_nothing(self):
        self.a.set_face_turn(10.0, True)
        self.a.plan_seen(10.0)
        self.assertEqual(self.a.decide(10.1)[0], "nav2")

    def test_nav2_with_fresh_plan_denies_helpers(self):
        self.a.plan_seen(10.0)
        self.a.wall_escape_escaping = True
        owner, allow = self.a.decide(11.0)
        self.assertEqual(owner, "nav2")
        self.assertTrue(none_allowed(allow))

    def test_stale_plan_is_not_nav2(self):
        self.a.plan_seen(10.0)
        self.assertEqual(self.a.decide(12.5)[0], "idle")

    def test_router_rotation_beats_old_plan_and_denies_wall_escape(self):
        self.a.plan_seen(10.0)       # 前の目的地の経路が残っていても、新しい目的地の先回し旋回が優先
        self.a.router_rotating = True
        owner, allow = self.a.decide(10.5)
        self.assertEqual(owner, "router_rotate")
        self.assertEqual(allow, {"wall_escape": False, "router_rotate": True, "director_direct": False})

    def test_wall_escape_denies_router_rotation(self):
        self.a.wall_escape_escaping = True
        owner, allow = self.a.decide(10.0)
        self.assertEqual(owner, "wall_escape")
        self.assertEqual(allow, {"wall_escape": True, "router_rotate": False, "director_direct": False})

    def test_never_two_helpers_allowed_unless_idle(self):
        import itertools
        for hard, soft, wasd, near, rot, esc, plan in itertools.product([False, True], repeat=7):
            a = Arbiter()
            a.hard_estop, a.soft_estop, a.wasd, a.near_person, a.router_rotating, a.wall_escape_escaping = hard, soft, wasd, near, rot, esc
            if plan:
                a.plan_seen(10.0)
            owner, allow = a.decide(10.0)
            self.assertEqual(set(allow), set(HELPERS))
            if owner != "idle":
                self.assertLessEqual(sum(allow.values()), 1, (owner, allow))
            if owner in ("estop", "manual", "near_person", "nav2"):
                self.assertTrue(none_allowed(allow), owner)


class DirectorDirect(unittest.TestCase):
    """behavior_directorの直接/cmd_vel(見回し・揺動)は、idleのときだけ許可(2026-10-23、B3)。"""

    def test_helpers_include_director_direct(self):
        self.assertIn("director_direct", HELPERS)

    def test_idle_allows_director_direct(self):
        owner, allow = Arbiter().decide(10.0)
        self.assertEqual(owner, "idle")
        self.assertTrue(allow["director_direct"])

    def test_only_idle_allows_director_direct(self):
        import itertools
        for hard, soft, wasd, near, rot, esc, plan in itertools.product([False, True], repeat=7):
            a = Arbiter()
            a.hard_estop, a.soft_estop, a.wasd, a.near_person, a.router_rotating, a.wall_escape_escaping = hard, soft, wasd, near, rot, esc
            if plan:
                a.plan_seen(10.0)
            owner, allow = a.decide(10.0)
            self.assertEqual(allow["director_direct"], owner == "idle", (owner, allow))

    def test_each_blocking_owner_denies_director_direct(self):
        for attr, owner in (("hard_estop", "estop"), ("soft_estop", "estop"), ("wasd", "manual"), ("near_person", "near_person"),
                            ("router_rotating", "router_rotate"), ("wall_escape_escaping", "wall_escape")):
            a = Arbiter()
            setattr(a, attr, True)
            o, allow = a.decide(10.0)
            self.assertEqual(o, owner)
            self.assertFalse(allow["director_direct"], owner)
        a = Arbiter()
        a.plan_seen(10.0)
        self.assertFalse(a.decide(10.5)[1]["director_direct"])  # nav2
        b = Arbiter()
        b.recovery_active["backup"] = True
        self.assertFalse(b.decide(10.5)[1]["director_direct"])


class Wiring(unittest.TestCase):
    def test_wall_escape_uses_the_grant(self):
        src = (CORE / "wall_escape.py").read_text(encoding="utf-8")
        self.assertIn('"/nokosahen/motion_grant"', src)
        self.assertIn("_grant_ok", src)
        self.assertIn("_blocked_hard", src)
        self.assertIn("_escape_manual", src)

    def test_router_uses_the_grant(self):
        src = (CORE / "nav2_goal_router.py").read_text(encoding="utf-8")
        self.assertIn('"/nokosahen/motion_grant"', src)
        self.assertIn("_rotate_granted", src)

    def test_director_uses_the_grant(self):
        src = (CORE / "behavior_director.py").read_text(encoding="utf-8")
        self.assertIn('"/nokosahen/motion_grant"', src)
        self.assertIn('"director_direct"', src)
        self.assertIn("_publish_direct", src)

    def test_arbiter_publishes_no_velocity(self):
        src = (CORE / "motion_arbiter.py").read_text(encoding="utf-8")
        self.assertNotIn("Twist", src)
        self.assertNotIn("create_publisher(Twist", src)

    def test_launch_and_entry_point(self):
        self.assertIn("motion_arbiter = nokosahen_core.motion_arbiter:main", (Path(__file__).resolve().parents[1] / "setup.py").read_text(encoding="utf-8"))
        for f in ("mapping_mode", "localization_mode"):
            self.assertIn('executable="motion_arbiter"', (BRINGUP / f"{f}.launch.py").read_text(encoding="utf-8"), f)



class RelaxWatchdogTests(unittest.TestCase):
    """【2026-10-24 H2】gateが relaxed=true のまま途絶えたら、VelocityStopを有効に戻す要求を出す。"""

    def test_nothing_before_any_status(self):
        w = RelaxWatchdog()
        self.assertFalse(w.should_restore(100.0))

    def test_relaxed_false_never_requests(self):
        w = RelaxWatchdog()
        w.on_status(0.0, False)
        for now in (1.0, 3.0, 10.0, 1000.0):
            self.assertFalse(w.should_restore(now))

    def test_heartbeat_loss_requests_restore_after_3s(self):
        w = RelaxWatchdog()
        w.on_status(0.0, True)
        w.on_status(1.0, True)
        self.assertFalse(w.should_restore(2.0))
        self.assertFalse(w.should_restore(3.9))
        self.assertTrue(w.should_restore(4.0))     # 最後の受信(1.0)から3.0秒
        self.assertFalse(w.should_restore(4.5))    # すぐには再送しない
        self.assertFalse(w.should_restore(8.9))
        self.assertTrue(w.should_restore(9.0))     # 5秒おきに再送

    def test_alive_gate_never_triggers(self):
        w = RelaxWatchdog()
        for i in range(60):
            w.on_status(float(i), True)
            self.assertFalse(w.should_restore(i + 0.9))

    def test_gate_returns_stops_requests(self):
        w = RelaxWatchdog()
        w.on_status(0.0, True)
        self.assertTrue(w.should_restore(5.0))
        w.on_status(6.0, True)                     # gateが戻った
        self.assertFalse(w.should_restore(6.5))
        w.on_status(7.0, False)                    # gateが自分で有効に戻した
        self.assertFalse(w.should_restore(100.0))

    def test_retry_now_allows_immediate_resend(self):
        w = RelaxWatchdog()
        w.on_status(0.0, True)
        self.assertTrue(w.should_restore(5.0))
        w.retry_now()                              # サービスが無くて送れなかった
        self.assertTrue(w.should_restore(5.1))

    def test_custom_stale_sec(self):
        w = RelaxWatchdog(stale_sec=1.0)
        w.on_status(0.0, True)
        self.assertFalse(w.should_restore(0.9))
        self.assertTrue(w.should_restore(1.0))

    def test_wiring_only_touches_velocity_stop(self):
        src = (CORE / "motion_arbiter.py").read_text(encoding="utf-8")
        self.assertIn('"/nokosahen/person_clear_gate/status"', src)
        self.assertIn('"/collision_monitor/set_parameters"', src)
        self.assertIn('"VelocityStop.enabled"', src)
        self.assertIn("bool_value=True", src)      # 有効に戻す向きだけ
        self.assertNotIn("bool_value=False", src)
        self.assertNotIn("Twist", src)             # 速度指令は出さない



class FlagExpiryTests(unittest.TestCase):
    """【2026-10-24 M4】recovery_active・router_rotatingは、立ててから15秒を超えたら無効と見なす。"""

    def test_router_rotating_expires_after_15s(self):
        a = Arbiter()
        a.set_router_rotating(100.0, True)
        self.assertEqual(a.decide(100.5)[0], "router_rotate")
        self.assertEqual(a.decide(115.0)[0], "router_rotate")      # ちょうど15秒はまだ有効
        owner, allow = a.decide(115.01)
        self.assertEqual(owner, "idle")                           # 張り付いた許可なし状態から戻る
        self.assertTrue(all(allow.values()))
        self.assertEqual(a.expired_flags(115.01), ["router_rotating"])

    def test_recovery_expires_after_15s(self):
        a = Arbiter()
        a.set_recovery("backup", 50.0, True)
        self.assertEqual(a.decide(60.0)[0], "nav2")
        self.assertEqual(a.decide(65.0)[0], "nav2")
        self.assertEqual(a.decide(65.01)[0], "idle")
        self.assertEqual(a.expired_flags(65.01), ["recovery:backup"])

    def test_expiry_falls_through_to_next_priority(self):
        a = Arbiter()
        a.wall_escape_escaping = True
        a.set_router_rotating(0.0, True)
        self.assertEqual(a.decide(1.0)[0], "router_rotate")
        self.assertEqual(a.decide(16.0)[0], "wall_escape")         # 回頭が張り付いていても、離脱に許可が出る

    def test_new_rising_edge_restarts_the_clock(self):
        a = Arbiter()
        a.set_router_rotating(0.0, True)
        a.set_router_rotating(10.0, True)                          # 立ったままの再送では延ばさない
        self.assertEqual(a.decide(15.5)[0], "idle")
        a.set_router_rotating(16.0, False)
        a.set_router_rotating(20.0, True)                          # 新しく立てた
        self.assertEqual(a.decide(30.0)[0], "router_rotate")
        self.assertEqual(a.decide(34.9)[0], "router_rotate")
        self.assertEqual(a.decide(35.1)[0], "idle")

    def test_recovery_cleared_normally(self):
        a = Arbiter()
        a.set_recovery("spin", 0.0, True)
        a.set_recovery("spin", 2.0, False)
        self.assertEqual(a.decide(3.0)[0], "idle")
        self.assertEqual(a.expired_flags(100.0), [])

    def test_one_recovery_expired_other_valid(self):
        a = Arbiter()
        a.set_recovery("backup", 0.0, True)
        a.set_recovery("spin", 14.0, True)
        self.assertEqual(a.decide(16.0)[0], "nav2")                # spinはまだ有効
        self.assertEqual(a.decide(29.5)[0], "idle")

    def test_direct_assignment_has_no_expiry(self):
        # 属性を直接代入した場合(従来の使い方)は時刻が無く、期限なし
        a = Arbiter()
        a.router_rotating = True
        self.assertEqual(a.decide(10000.0)[0], "router_rotate")
        b = Arbiter()
        b.recovery_active["backup"] = True
        self.assertEqual(b.decide(10000.0)[0], "nav2")

    def test_estop_and_manual_still_win_over_expiry_logic(self):
        a = Arbiter()
        a.set_router_rotating(0.0, True)
        a.hard_estop = True
        self.assertEqual(a.decide(1.0)[0], "estop")
        self.assertEqual(a.decide(99.0)[0], "estop")

    def test_plan_stale_default_is_3s(self):
        a = Arbiter()
        self.assertEqual(a.plan_stale_sec, 3.0)
        a.plan_seen(10.0)
        self.assertEqual(a.decide(13.0)[0], "nav2")
        self.assertEqual(a.decide(13.1)[0], "idle")
        src = (CORE / "motion_arbiter.py").read_text(encoding="utf-8")
        self.assertIn('declare_parameter("plan_stale_sec", 3.0)', src)


class NodeCallbacksTests(unittest.TestCase):
    """ノードの購読コールバックと_tick(替え玉)。期限つきのフラグを使い、ログと配信を確認する。"""

    def setUp(self):
        import json
        import types
        from nokosahen_core import motion_arbiter as ma
        self.json, self.ma = json, ma
        self.clock = types.SimpleNamespace(t=1000.0)
        self._saved = (ma.time, getattr(ma, "String", None))
        ma.time = types.SimpleNamespace(monotonic=lambda: self.clock.t, time=lambda: 1.7e9)
        ma.String = lambda: types.SimpleNamespace(data="")
        n = ma.MotionArbiter.__new__(ma.MotionArbiter)
        n._arb = Arbiter()
        n._backup_prev = False
        n._last_owner = None
        n._last_expired = []
        n._relax_dog = RelaxWatchdog()
        n._cm_cli = None
        self.sent = []
        n._pub = types.SimpleNamespace(publish=lambda m: self.sent.append(json.loads(m.data)))
        self.logs = []
        n.get_logger = lambda: types.SimpleNamespace(
            info=lambda m: self.logs.append(("info", m)), warn=lambda m: self.logs.append(("warn", m)))
        n._log_backup_snapshot = lambda: None
        self.n = n
        self.types = types

    def tearDown(self):
        self.ma.time, self.ma.String = self._saved[0], self._saved[1]

    def status(self, *codes):
        return self.types.SimpleNamespace(status_list=[self.types.SimpleNamespace(status=c) for c in codes])

    def test_stuck_recovery_status_expires_in_the_published_grant(self):
        self.n._on_recovery_status("backup", self.status(2))       # EXECUTING。そのあと、アクションサーバーが落ちて続報なし
        self.n._tick()
        self.assertEqual(self.sent[-1]["owner"], "nav2")
        self.clock.t += 16.0
        self.n._tick()
        self.assertEqual(self.sent[-1]["owner"], "idle")
        self.assertTrue(self.sent[-1]["allow"]["wall_escape"])
        self.assertTrue(any(k == "warn" and "15秒" in m for k, m in self.logs))
        n_warn = sum(1 for k, _ in self.logs if k == "warn")
        self.n._tick()
        self.assertEqual(sum(1 for k, _ in self.logs if k == "warn"), n_warn)   # 毎周期は警告しない

    def test_router_rotating_callback_stamps_time(self):
        self.n._arb.set_router_rotating(self.clock.t, True)
        self.n._tick()
        self.assertEqual(self.sent[-1]["owner"], "router_rotate")
        self.clock.t += 15.5
        self.n._tick()
        self.assertEqual(self.sent[-1]["owner"], "idle")

    def test_node_wiring_uses_the_stamped_setters(self):
        src = (CORE / "motion_arbiter.py").read_text(encoding="utf-8")
        self.assertIn("set_router_rotating(time.monotonic()", src)
        self.assertIn("set_recovery(name, time.monotonic()", src)
        self.assertNotIn('setattr(self._arb, "router_rotating"', src)


if __name__ == "__main__":
    unittest.main(verbosity=1)
