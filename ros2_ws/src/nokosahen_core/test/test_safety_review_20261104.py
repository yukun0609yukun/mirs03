"""2026-11-04 Opusレビュー(重大度「高」: 安全)の修正のテスト。ROS不要。

- 指摘1: 近接停止の免除中に位置が取れなくても mode が None にならない(免除しない=従来どおり停止)。_tickのガード
- 指摘2: カメラ障害物停止の免除は「実際に旋回だけが選ばれる」ときだけ。位置不明なら免除しない。前進指令0件
- 指摘4: 近接停止(near_person_stop)の受信が一度あったのに途絶えたら、安全側(director / motion_arbiter / wall_escape)
- 指摘5: 持ち主 near_person_face_turn を、判定リスト(goal_router / detour)が見落とさない
"""
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import test_face_person as tfp  # noqa: E402
import test_wall_escape_safety as twes  # noqa: E402
from nokosahen_core import motion_arbiter as ma  # noqa: E402
from nokosahen_core import wall_escape as we  # noqa: E402
from nokosahen_core.near_person_watch import NearPersonWatch  # noqa: E402
import test_goal_router_grant as tgrg  # noqa: E402,F401  (rclpyの替え玉を入れてから nav2_goal_router を読む)

gr = tgrg.r

CORE = Path(__file__).resolve().parents[1] / "nokosahen_core"


class DirectorBase(unittest.TestCase):
    def setUp(self):
        self.f = tfp.FacePersonTests("test_all_off_does_nothing")
        self.f.setUp()

    def tearDown(self):
        self.f.tearDown()


@unittest.skipIf(tfp.bd is None, "behavior_director import失敗")
class NoPoseTests(DirectorBase):
    def test_near_person_without_pose_stays_hold_and_does_not_crash(self):
        f = self.f
        n = f.make(face=True, lidar=())
        self.assertIsNone(n._get_pose())
        f.near(n, rng=2.0)
        n._get_pose = lambda: None  # near() が入れた位置を外す(位置が取れない状況)
        modes = f.run_near(n, 2.0)
        self.assertEqual(set(modes), {"NEAR_PERSON_HOLD"})  # 位置が取れない: 免除しない(従来どおり停止)
        self.assertFalse([t for t in f.twists(n) if t.angular.z != 0.0 or t.linear.x != 0.0])

    def test_face_only_step_never_returns_none(self):
        f = self.f
        n = f.make(face=True, lidar=())
        mode = n._face_only_step(n.get_clock().now())
        self.assertIsInstance(mode, str)
        self.assertEqual(mode, "NEAR_PERSON_HOLD")

    def test_tick_guard_when_mode_is_not_a_string(self):
        f = self.f
        n = f.make(scan=True, face=False, lidar=())
        n._crowd_scan_step = lambda *a, **k: None  # 想定外にNoneが返る経路
        modes = f.modes(n, 0.5)
        self.assertEqual(set(modes), {"POLICY_NO_SCAN"})

    def test_near_person_with_pose_still_exempt(self):
        f = self.f
        n = f.with_pose(f.make(face=True, lidar=(40,)))
        f.near(n, rng=0.6)
        modes = f.run_near(n, 1.2)
        self.assertEqual(set(modes[-4:]), {"FACE_PERSON"})


@unittest.skipIf(tfp.bd is None, "behavior_director import失敗")
class CameraGuardTests(DirectorBase):
    def test_no_pose_camera_obstacle_never_moves_forward(self):
        f = self.f
        n = f.make(face=True, scan=True, lidar=())
        self.assertIsNone(n._get_pose())
        modes = f.run_with_camera(n, 40.0)
        self.assertEqual(set(modes), {"CAMERA_OBSTACLE_HOLD"})
        self.assertEqual([t for t in f.twists(n) if t.linear.x != 0.0], [])

    def test_with_pose_camera_exemption_still_works(self):
        f = self.f
        n = f.with_pose(f.make(face=True, lidar=()))
        modes = f.run_with_camera(n, 1.0)
        self.assertEqual(set(modes), {"FACE_SWEEP"})

    def test_late_guard_stops_when_exempt_tick_picks_forward_mode(self):
        f = self.f
        n = f.with_pose(f.make(face=True, scan=True, lidar=()))
        n._face_sweep_step = lambda now: None  # 免除したのに旋回が選ばれず、見回しに落ちる想定外の経路
        n._crowd_scan_step = lambda now, dt: "CROWD_APPROACH"
        modes = f.run_with_camera(n, 0.5)
        self.assertEqual(set(modes), {"CAMERA_OBSTACLE_HOLD"})

    def test_exempt_context_picks_face_or_sweep_first(self):
        # 免除の条件(_face_only_context)が真なら、calm/crowdedの分岐は face か sweep を先に選び、揺動・CROWD_*には落ちない
        f = self.f
        for crowd_maker in ("calm", "crowded"):
            n = f.with_pose(f.make(face=True, scan=True, lidar=()) if crowd_maker == "calm" else f.make_crowded(lidar=()))
            n._idle_wiggle_enabled = True
            modes = set(f.modes(n, 1.0))
            self.assertTrue(modes <= {"FACE_SWEEP", "FACE_PERSON", "POLICY_NO_SCAN"}, (crowd_maker, modes))


@unittest.skipIf(tfp.bd is None, "behavior_director import失敗")
class DirectorStaleTests(DirectorBase):
    def send(self, n, value):
        n._near_person_stop_detected = value
        n._last_near_person_stop_receive_time = n._clock.now()
        n._near_person_stale_warned = False

    def tick_mode(self, n, dt=0.1):
        n._clock.t += dt
        n._tick()
        return json.loads(n._command_pub.msgs[-1].data)["mode"]

    def test_never_received_does_not_stop(self):
        n = self.f.make(face=False, lidar=())
        self.assertIsNone(n._last_near_person_stop_receive_time)
        n._clock.t += 100
        self.assertFalse(n._near_person_too_close)
        self.assertNotEqual(self.tick_mode(n), "NEAR_PERSON_HOLD")

    def test_stops_after_silence_and_resumes(self):
        n = self.f.make(face=False, lidar=())
        self.send(n, False)
        n._clock.t += 1.0
        self.assertFalse(n._near_person_too_close)  # 途絶の許容(2秒)内
        n._clock.t += 1.5
        self.assertTrue(n._near_person_too_close)
        self.assertEqual(self.tick_mode(n), "NEAR_PERSON_HOLD")
        self.send(n, False)  # 再開
        self.assertFalse(n._near_person_too_close)

    def test_warns_only_once(self):
        n = self.f.make(face=False, lidar=())
        self.send(n, False)
        n._clock.t += 5.0
        for _ in range(20):
            n._tick()
        warns = [m for lvl, m in n._log.lines if lvl == "warn" and "near_person_stop" in m]
        self.assertEqual(len(warns), 1)

    def test_disabled_by_parameter(self):
        n = self.f.make(face=False, lidar=())
        n.near_person_stale_sec = 0.0
        self.send(n, False)
        n._clock.t += 100
        self.assertFalse(n._near_person_too_close)

    def test_manual_override_not_affected(self):
        n = self.f.make(face=False, lidar=())
        self.send(n, False)
        n._clock.t += 5.0
        n._manual_override_raw = True
        n._last_manual_override_time = n._clock.now()
        self.assertEqual(self.tick_mode(n, 0.0), "MANUAL_OVERRIDE")


class WatchAndArbiterTests(unittest.TestCase):
    def test_watch(self):
        w = NearPersonWatch(2.0)
        self.assertFalse(w.effective(100.0))  # 一度も受信なし
        w.on_message(100.0, False)
        self.assertFalse(w.effective(101.0))
        self.assertTrue(w.effective(102.5))
        self.assertTrue(w.should_warn(102.5))
        self.assertFalse(w.should_warn(103.0))
        w.on_message(104.0, False)
        self.assertFalse(w.effective(104.5))
        self.assertTrue(NearPersonWatch(2.0).__class__ is NearPersonWatch)
        off = NearPersonWatch(0.0)
        off.on_message(0.0, False)
        self.assertFalse(off.effective(1000.0))

    def test_arbiter_never_received_is_idle(self):
        a = ma.Arbiter()
        self.assertEqual(a.decide(1000.0)[0], "idle")

    def test_arbiter_stale_is_near_person(self):
        a = ma.Arbiter()
        a.set_near_person(10.0, False)
        self.assertEqual(a.decide(11.0)[0], "idle")
        owner, allow = a.decide(13.0)
        self.assertEqual(owner, "near_person")
        self.assertTrue(all(v is False for v in allow.values()))
        a.set_near_person(13.5, False)
        self.assertEqual(a.decide(14.0)[0], "idle")

    def test_arbiter_stale_disabled(self):
        a = ma.Arbiter(near_person_stale_sec=0.0)
        a.set_near_person(10.0, False)
        self.assertEqual(a.decide(1000.0)[0], "idle")

    def test_arbiter_stale_does_not_allow_face_turn(self):
        a = ma.Arbiter()
        a.set_near_person(10.0, False)
        a.set_face_turn(12.9, True)
        self.assertEqual(a.decide(13.0)[0], "near_person_face_turn")  # 途絶でも、旋回だけの許可は従来の近接停止と同じ扱い


class WallEscapeStaleTests(twes.Base):
    def test_never_received_not_blocked(self):
        n = self.make()
        n._near_person_time = None
        self.assertFalse(n._blocked_hard())

    def test_stale_blocks_and_resumes(self):
        n = self.make()
        n._near_person_stale = 2.0
        n._on_near_person(types_bool(False))
        self.assertFalse(n._blocked_hard())
        n._t += 3.0
        self.assertTrue(n._blocked_hard())
        n._on_near_person(types_bool(False))
        self.assertFalse(n._blocked_hard())

    def test_disabled(self):
        n = self.make()
        n._near_person_stale = 0.0
        n._on_near_person(types_bool(False))
        n._t += 100.0
        self.assertFalse(n._blocked_hard())


def types_bool(v):
    return type("M", (), {"data": v})()


class OwnerListTests(unittest.TestCase):
    def test_goal_router_discards_face_turn_owner(self):
        for owner in ma.OWNERS:
            expected = owner in ("estop", "manual") or owner.startswith("near_person")
            self.assertEqual(gr.discard_goal_on_revoke(owner), expected, owner)
        self.assertFalse(gr.discard_goal_on_revoke(None))

    def test_detour_source_uses_prefix_match(self):
        src = (CORE / "nav2_detour.py").read_text(encoding="utf-8")
        self.assertIn('owner.startswith("near_person")', src)
        self.assertIn('owner.startswith("near_person")', (CORE / "nav2_goal_router.py").read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
