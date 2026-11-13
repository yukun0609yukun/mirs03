"""その場で人の方を向く動作(向く / face)の単体テスト(2026-11-03、docs/face_person_design.md)。

3つの動作(近づく・見回す・向く)が独立のスイッチで動くこと、優先順位、旋回だけ(前進しない)、
5度/10度のヒステリシス、複数候補で多い方を向く、安全停止・範囲が先に効くこと、
木の face ノード / after_facing / ルールの face_enabled・face_disabled の検証。
rclpy不要。実行: python ros2_ws/src/nokosahen_core/test/test_face_person.py
"""

import json
import math
import sys
import time
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import test_behavior_rules as base  # noqa: E402
import test_behavior_tree as tbt  # noqa: E402
from nokosahen_core import map_orientation as mo  # noqa: E402

bd = base.bd
br = base.br
rule = base.rule
bt = tbt.bt


def cands(lidar=(), camera=()):
    """person_candidates の JSON と同じ形(bearing は度で指定、range は 2.0 m)。"""
    return {
        "stamp": 1.0,
        "frame": "laser",
        "lidar": [{"bearing": math.radians(d), "range": 2.0, "confirmed": False} for d in lidar],
        "camera": [{"bearing": math.radians(d)} for d in camera],
    }


@unittest.skipIf(bd is None, "behavior_director import失敗")
class FacePersonTests(unittest.TestCase):
    def setUp(self):
        self.h = base.DirectorTickTests("make")
        self.h.setUp()

    def tearDown(self):
        self.h.tearDown()

    def make(self, rules=None, target=False, approach=False, scan=False, face=False, lidar=(0,), camera=(),
             gate=True):
        n = self.h.make(rules, crowd="calm", target=target)
        n._require_interaction_opt_in = gate
        n._person_interaction_enabled = approach
        n._scan_enabled = scan
        n._face_enabled = face
        self.set_cands(n, lidar, camera)
        n._clock.t += 10  # 接近ゴールの再送間隔を越える
        return n

    @staticmethod
    def set_cands(n, lidar=(), camera=(), age=0.0):
        n._person_candidates = cands(lidar, camera)
        n._person_candidates_mono = time.monotonic() - age

    def modes(self, n, seconds=1.0):
        return self.h.run_ticks(n, seconds)

    @staticmethod
    def last_cmd(n):
        return json.loads(n._command_pub.msgs[-1].data)

    @staticmethod
    def twists(n):
        return list(n._cmd_vel_pub.msgs)

    # --- 一瞬の見逃し(センサの取りこぼし)で、旋回が止まってカクつかない(2026-11-04、シミュレーションで見つけた) ---
    def test_short_candidate_dropout_does_not_stop_the_turn(self):
        n = self.make(face=True, lidar=(60,))
        self.modes(n, 0.3)
        self.set_cands(n, lidar=())  # 候補が一瞬消えた
        modes = self.modes(n, 0.3)
        self.assertEqual(set(modes), {"FACE_PERSON"})
        self.assertTrue([t for t in self.twists(n) if t.angular.z != 0.0])

    def test_long_candidate_dropout_stops_after_grace(self):
        n = self.make(face=True, lidar=(60,))
        self.modes(n, 0.3)
        self.set_cands(n, lidar=())
        modes = self.modes(n, 2.0)
        self.assertEqual(modes[-1], "POLICY_NO_SCAN")
        self.assertIn("FACE_PERSON", modes[:3])

    # --- 混雑(crowded)でも向く(2026-11-04: 以前はcalmのときだけで、人が多いと向かなかった) ---
    def make_crowded(self, approach=False, face=True, lidar=(40,)):
        n = self.h.make(None, crowd="crowded", target=False)
        n._require_interaction_opt_in = True
        n._person_interaction_enabled = approach
        n._scan_enabled = False
        n._face_enabled = face
        self.set_cands(n, lidar, ())
        n._clock.t += 10
        return n

    def test_crowded_faces_person(self):
        n = self.make_crowded()
        self.assertEqual(set(self.modes(n)), {"FACE_PERSON"})
        self.assertTrue([t for t in self.twists(n) if t.angular.z != 0.0])
        self.assertFalse(n._goal_pub.msgs)

    def test_crowded_face_off_stays_stop_announce(self):
        n = self.make_crowded(face=False)
        self.assertEqual(set(self.modes(n)), {"STOP_ANNOUNCE"})
        self.assertFalse(self.h.moved(n))

    def test_crowded_approach_on_still_goes_to_entrance(self):
        n = self.make_crowded(approach=True)
        self.assertEqual(set(self.modes(n)), {"STOP_ANNOUNCE"})
        self.assertTrue(n._goal_pub.msgs)

    def test_crowded_no_candidates_stops_and_announces(self):
        n = self.make_crowded(lidar=())
        self.assertEqual(set(self.modes(n)), {"STOP_ANNOUNCE"})

    # --- 2026-11-04: 監視範囲(ONにした向きの前180度)・人がいない間の見回し(FACE_SWEEP)・カメラ障害物停止の免除 ---
    def with_pose(self, n, yaw=0.0):
        """位置(向き)を返すスタブ。self.yaw を書き換えると、今の向きが変わる。"""
        self.yaw = yaw
        n._get_pose = lambda: {"x": 0.0, "y": 0.0, "yaw": self.yaw}
        return n

    @staticmethod
    def camera_obstacle(n):
        n._camera_obstacle_detected = True
        n._last_camera_obstacle_receive_time = n._clock.now()

    def run_with_camera(self, n, seconds=0.5):
        modes = []
        for _ in range(int(round(seconds / 0.1))):
            self.camera_obstacle(n)
            modes += self.modes(n, 0.1)
        return modes

    def test_no_person_sweeps_in_place_without_moving_forward(self):
        n = self.with_pose(self.make(face=True, lidar=()))
        modes = self.modes(n, 1.0)
        self.assertEqual(set(modes), {"FACE_SWEEP"})
        tw = self.twists(n)
        self.assertTrue(tw)
        self.assertTrue(all(t.linear.x == 0.0 for t in tw))   # 旋回だけ
        self.assertTrue(any(t.angular.z > 0.0 for t in tw))    # まず左へ
        self.assertFalse(n._goal_pub.msgs)
        cmd = self.last_cmd(n)
        self.assertIs(cmd["face_sweep_active"], True)
        self.assertIs(cmd["face_active"], False)

    def test_sweep_turns_back_at_the_edge_of_the_range(self):
        n = self.with_pose(self.make(face=True, lidar=()))
        self.modes(n, 0.2)                       # 基準の向き(0度)を確定
        self.yaw = math.radians(88.0)            # 左の端(90度)の手前に来た
        n._cmd_vel_pub.msgs.clear()
        self.modes(n, 0.3)
        self.assertTrue(all(t.angular.z < 0.0 for t in self.twists(n)))   # 折り返して右へ
        self.yaw = math.radians(-88.0)
        n._cmd_vel_pub.msgs.clear()
        self.modes(n, 0.3)
        self.assertTrue(all(t.angular.z > 0.0 for t in self.twists(n)))   # 右の端でまた左へ

    def test_outside_the_range_returns_inside(self):
        n = self.with_pose(self.make(face=True, lidar=()))
        self.modes(n, 0.2)
        self.yaw = math.radians(120.0)           # 範囲の外に出ている
        n._cmd_vel_pub.msgs.clear()
        self.modes(n, 0.3)
        self.assertTrue(all(t.angular.z < 0.0 for t in self.twists(n)))

    def test_person_in_range_is_faced_person_behind_is_not(self):
        n = self.with_pose(self.make(face=True, lidar=(60,)))
        self.assertEqual(set(self.modes(n, 0.5)), {"FACE_PERSON"})
        n = self.with_pose(self.make(face=True, lidar=(150,)))   # 範囲の外(後ろ寄り)
        self.assertEqual(set(self.modes(n, 0.5)), {"FACE_SWEEP"})
        self.assertTrue(all(t.linear.x == 0.0 for t in self.twists(n)))

    def test_reference_heading_is_fixed_when_switched_on(self):
        n = self.with_pose(self.make(face=True, lidar=()))
        self.modes(n, 0.2)
        self.assertAlmostEqual(n._face_ref_yaw, 0.0)
        self.yaw = math.radians(57.3)            # 車体が回っても、基準は動かない
        self.modes(n, 0.2)
        self.assertAlmostEqual(n._face_ref_yaw, 0.0)
        # 基準から見て100度(範囲の外)の人: 車体から見ると約43度だが、範囲の外なので向かない
        self.set_cands(n, lidar=(43,))
        self.assertEqual(set(self.modes(n, 0.5)), {"FACE_SWEEP"})
        # スイッチをOFF→ONすると、そのときの向きが新しい基準になる
        for v in (False, True):
            msg = base._Bool()
            msg.data = v
            n._on_face_enabled(msg)
        self.assertIsNone(n._face_ref_yaw)
        self.modes(n, 0.2)
        self.assertAlmostEqual(n._face_ref_yaw, math.radians(57.3))

    def test_face_turn_speed_follows_face_turn_rps(self):
        # 【2026-11-04】向く(FACE_PERSON/FACE_SWEEP)の旋回速度は face_turn_rps(実機の既定1.5 rad/s=手動運転の旋回と同じ)。見回し(crowd_scan_turn_rps)とは別
        n = self.with_pose(self.make(face=True, lidar=(80,)))
        n.face_turn_rps = 1.5
        n.crowd_scan_turn_rps = 0.75
        self.modes(n, 0.5)
        peak = max(abs(t.angular.z) for t in self.twists(n))
        self.assertAlmostEqual(peak, 1.5, places=6)
        n = self.with_pose(self.make(face=True, lidar=()))
        n.face_turn_rps = 1.5
        n.crowd_scan_turn_rps = 0.75
        self.modes(n, 0.5)
        self.assertAlmostEqual(max(abs(t.angular.z) for t in self.twists(n)), 1.5, places=6)

    def test_switch_off_does_not_sweep(self):
        n = self.with_pose(self.make(face=False, lidar=()))
        self.assertEqual(set(self.modes(n, 1.0)), {"POLICY_NO_SCAN"})
        self.assertFalse(self.h.moved(n))

    def test_unknown_pose_does_not_sweep(self):
        n = self.make(face=True, lidar=())
        n._get_pose = lambda: None
        self.assertNotIn("FACE_SWEEP", self.modes(n, 0.5))
        self.assertFalse([t for t in self.twists(n) if t.angular.z != 0.0])

    def test_face_only_ignores_camera_obstacle_while_turning_in_place(self):
        n = self.with_pose(self.make(face=True, lidar=()))
        modes = self.run_with_camera(n)
        self.assertNotIn("CAMERA_OBSTACLE_HOLD", modes)
        self.assertEqual(set(modes), {"FACE_SWEEP"})
        n = self.with_pose(self.make(face=True, lidar=(40,)))
        self.assertEqual(set(self.run_with_camera(n)), {"FACE_PERSON"})

    def test_camera_obstacle_still_stops_when_face_is_off(self):
        n = self.with_pose(self.make(face=False, lidar=()))
        self.assertEqual(set(self.run_with_camera(n)), {"CAMERA_OBSTACLE_HOLD"})

    def test_camera_obstacle_still_stops_when_approach_may_drive(self):
        n = self.with_pose(self.make(face=True, approach=True, lidar=()))
        self.assertEqual(set(self.run_with_camera(n)), {"CAMERA_OBSTACLE_HOLD"})

    # レビュー指摘5: 前へ進む動きが選ばれ得るとき(ホーム帰還・地図作成・方針の移動)は、カメラ障害物停止を免除しない
    def test_camera_obstacle_still_stops_during_home_return(self):
        n = self.with_pose(self.make(face=True, lidar=()))
        n._home_return_active = True
        modes = self.run_with_camera(n)
        self.assertNotIn("FACE_SWEEP", modes)
        self.assertNotIn("FACE_PERSON", modes)

    def test_camera_obstacle_still_holds_while_mapping(self):
        n = self.with_pose(self.make(face=True, lidar=()))
        n._map_building_active = True
        modes = self.run_with_camera(n)
        self.assertNotIn("FACE_SWEEP", modes)
        self.assertNotIn("FACE_PERSON", modes)

    # レビュー指摘6: 基準の向きを取り直す
    def test_reference_heading_resets_after_manual_override(self):
        n = self.with_pose(self.make(face=True, lidar=()))
        self.modes(n, 0.2)
        self.assertAlmostEqual(n._face_ref_yaw, 0.0)
        n._manual_override_raw = True
        n._last_manual_override_time = n._clock.now()
        self.modes(n, 0.2)
        n._manual_override_raw = False
        self.yaw = math.radians(90.0)  # 手動運転で向きが変わった
        self.modes(n, 0.3)
        self.assertAlmostEqual(n._face_ref_yaw, math.radians(90.0))

    def test_reference_heading_resets_when_face_becomes_allowed(self):
        n = self.with_pose(self.make(face=False, lidar=()))
        self.modes(n, 0.2)
        self.yaw = math.radians(40.0)
        n._face_enabled = True  # スイッチ以外(ルール・木の明示)で許可になる場合も、許可に変わった時点で取り直す
        self.modes(n, 0.3)
        self.assertAlmostEqual(n._face_ref_yaw, math.radians(40.0))

    def test_exemption_can_be_turned_off(self):
        n = self.with_pose(self.make(face=True, lidar=()))
        n.camera_guard_off_in_place_turn = False
        self.assertEqual(set(self.run_with_camera(n)), {"CAMERA_OBSTACLE_HOLD"})

    def test_sweep_does_not_replace_legacy_wiggle_when_gate_is_off(self):
        n = self.with_pose(self.make(face=False, lidar=(), gate=False))
        n._idle_wiggle_enabled = True
        self.assertEqual(set(self.modes(n, 0.5)), {"IDLE_WIGGLE"})

    # --- 3つの独立 ---
    def test_all_off_does_nothing(self):
        n = self.make()
        self.assertEqual(set(self.modes(n)), {"POLICY_NO_SCAN"})
        self.assertFalse(self.h.moved(n))

    def test_only_face_switch_faces(self):
        n = self.make(face=True, lidar=(40,))
        self.assertEqual(set(self.modes(n)), {"FACE_PERSON"})

    def test_only_approach_switch_does_not_face_or_scan(self):
        n = self.make(approach=True, target=False, lidar=(40,))
        modes = self.modes(n)
        self.assertEqual(set(modes), {"POLICY_NO_SCAN"})
        self.assertFalse(self.h.moved(n))

    def test_only_scan_switch_scans_and_does_not_face(self):
        n = self.make(scan=True, lidar=(40,))
        modes = self.modes(n)
        self.assertEqual(set(modes), {"CROWD_SCAN"})

    def test_face_switch_does_not_scan_or_wiggle_without_person(self):
        n = self.make(face=True, lidar=())
        n._person_candidates = cands()
        n._idle_wiggle_enabled = True
        self.assertEqual(set(self.modes(n)), {"POLICY_NO_SCAN"})
        self.assertFalse(self.h.moved(n))

    def test_wiggle_follows_scan_switch_not_approach(self):
        n = self.make(approach=True, lidar=())
        n._idle_wiggle_enabled = True
        self.assertEqual(set(self.modes(n)), {"POLICY_NO_SCAN"})
        n2 = self.make(scan=True, lidar=())
        n2._idle_wiggle_enabled = True
        self.assertEqual(set(self.modes(n2)), {"IDLE_WIGGLE"})

    def test_gate_off_legacy_allows_face(self):
        n = self.make(gate=False, lidar=(40,))
        self.assertEqual(set(self.modes(n)), {"FACE_PERSON"})

    def test_no_person_no_face(self):
        n = self.make(face=True, lidar=())
        self.assertNotIn("FACE_PERSON", self.modes(n))
        self.assertFalse(self.last_cmd(n)["face_active"])

    def test_stale_candidates_are_ignored(self):
        n = self.make(face=True, lidar=(40,))
        self.set_cands(n, (40,), age=5.0)
        self.assertNotIn("FACE_PERSON", self.modes(n))

    # --- 優先順位 ---
    def test_approach_wins_over_face_when_both_allowed(self):
        n = self.make(approach=True, face=True, target=True, lidar=(40,))
        self.assertEqual(set(self.modes(n)), {"APPROACH_PERSON"})

    def test_face_when_approach_not_allowed(self):
        n = self.make(face=True, target=True, lidar=(40,))
        self.assertEqual(set(self.modes(n)), {"FACE_PERSON"})

    def test_face_when_approach_disabled_by_rule(self):
        n = self.make([rule("a", policy="approach_disabled")], approach=True, face=True, target=True, lidar=(40,))
        self.assertEqual(set(self.modes(n)), {"FACE_PERSON"})

    # --- 旋回だけ / 向き / ヒステリシス ---
    def test_never_drives_forward_and_turns_left(self):
        n = self.make(face=True, lidar=(40,))
        self.modes(n)
        tw = self.twists(n)
        self.assertTrue(tw)
        self.assertTrue(all(t.linear.x == 0.0 for t in tw))
        self.assertTrue(all(t.angular.z > 0 for t in tw))
        self.assertFalse(n._goal_pub.msgs)

    def test_person_on_right_turns_right(self):
        n = self.make(face=True, lidar=(-40,))
        self.modes(n)
        tw = self.twists(n)
        self.assertTrue(tw and all(t.angular.z < 0 and t.linear.x == 0.0 for t in tw))

    def test_stops_within_5deg_and_reengages_only_over_10deg(self):
        n = self.make(face=True, lidar=(3,))
        self.modes(n, 0.5)
        self.assertEqual(self.twists(n)[-1].angular.z, 0.0)
        self.assertTrue(self.last_cmd(n)["face_aligned"])
        self.set_cands(n, (8,))  # 5度超でも10度以内 → 止まったまま
        self.modes(n, 0.5)
        self.assertEqual(self.twists(n)[-1].angular.z, 0.0)
        self.assertTrue(self.last_cmd(n)["face_aligned"])
        self.set_cands(n, (13,))  # 10度超 → 再び旋回
        self.modes(n, 0.5)
        self.assertGreater(self.twists(n)[-1].angular.z, 0.0)
        self.assertFalse(self.last_cmd(n)["face_aligned"])

    def test_faces_the_nearest_person(self):
        # 【2026-11-06】「人数の多い側」ではなく「一番近い人」を向く(以前の test_faces_the_side_with_more_people を置き換え)
        def at(spec):
            return {"stamp": 1.0, "frame": "laser", "camera": [],
                    "lidar": [{"bearing": math.radians(d), "range": r, "confirmed": False} for d, r in spec]}
        n = self.make(face=True)
        n._person_candidates = at([(60, 3.0), (62, 3.0), (58, 3.0), (-60, 1.5)])   # 左に3人(遠い)、右に1人(近い)
        n._person_candidates_mono = time.monotonic()
        self.modes(n)
        cmd = self.last_cmd(n)
        self.assertEqual(cmd["mode"], "FACE_PERSON")
        self.assertLess(cmd["face_bearing_deg"], -40.0)
        self.assertLess(self.twists(n)[-1].angular.z, 0.0)
        n2 = self.make(face=True)
        n2._person_candidates = at([(60, 1.5), (-60, 3.0), (-62, 3.0), (-58, 3.0)])
        n2._person_candidates_mono = time.monotonic()
        self.modes(n2)
        self.assertGreater(self.last_cmd(n2)["face_bearing_deg"], 40.0)
        self.assertGreater(self.twists(n2)[-1].angular.z, 0.0)

    def test_command_keys(self):
        n = self.make(face=True, scan=True, lidar=(40,))
        self.modes(n)
        cmd = self.last_cmd(n)
        for k, v in (("scan_enabled", True), ("face_enabled", True), ("face_active", True), ("face_aligned", False)):
            self.assertIs(cmd[k], v)
        self.assertAlmostEqual(cmd["face_bearing_deg"], 40.0, delta=1.0)
        n2 = self.make()
        self.modes(n2)
        cmd = self.last_cmd(n2)
        self.assertIs(cmd["face_active"], False)
        self.assertIsNone(cmd["face_bearing_deg"])
        self.assertIs(cmd["face_enabled"], False)
        self.assertIs(cmd["scan_enabled"], False)

    def test_face_start_and_end_logged_once(self):
        n = self.make(face=True, lidar=(40,))
        self.modes(n)
        n._person_candidates = cands()
        self.modes(n)
        lines = [m for _l, m in n._log.lines]
        self.assertEqual(sum("人の方を向く動作を開始" in m for m in lines), 1)
        self.assertEqual(sum("人の方を向く動作を終了" in m for m in lines), 1)

    def test_switch_callbacks(self):
        n = self.make()
        msg = base._Bool()
        msg.data = True
        n._on_face_enabled(msg)
        n._on_scan_enabled(msg)
        self.assertTrue(n._face_enabled and n._scan_enabled)
        msg.data = False
        n._on_face_enabled(msg)
        n._on_scan_enabled(msg)
        self.assertFalse(n._face_enabled or n._scan_enabled)

    def test_candidates_callback(self):
        n = self.make(face=True, lidar=())
        n._person_candidates = None
        n._person_candidates_mono = None
        m = base._Str()
        m.data = json.dumps(cands((40,)))
        n._on_person_candidates(m)
        self.assertEqual(set(self.modes(n)), {"FACE_PERSON"})

    # --- 安全が先 ---
    def near(self, n, rng=2.0, bearing=40):
        """近接停止が出ている状態にする。人候補の距離・全周で一番近い物までの距離は rng [m]。"""
        n._near_person_stop_detected = True
        n._last_near_person_stop_receive_time = n._clock.now()
        if n._get_pose() is None:  # 【2026-11-04】近接停止の免除は、位置が取れるときだけ(取れなければ従来どおり停止)
            n._get_pose = lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._near_args = (rng, bearing)
        self.refresh(n)

    def refresh(self, n, rng=None, age=0.0):
        """新しい候補メッセージが届いた状態にする(単調時計の時刻が毎回変わる)。"""
        r, bearing = self._near_args
        r = r if rng is None else rng
        c = cands((bearing,))
        c["lidar"][0]["range"] = r
        c["nearest_m"] = r
        c["nearest_age_s"] = age  # 【2026-11-05】その距離の元のスキャンを受けてからの経過秒(検出ノードの契約)
        n._person_candidates = c
        self._mono = getattr(self, "_mono", time.monotonic()) + 0.001
        n._person_candidates_mono = self._mono
        n._person_candidates_rx = n._clock.now().nanoseconds / 1e9  # ROS時計の受信時刻

    def run_near(self, n, seconds=1.0, rng=None, dt=0.1):
        """近接停止が出たまま、新しい候補メッセージを毎tick受けながら進める。modeの列を返す。"""
        modes = []
        for _ in range(int(round(seconds / dt))):
            n._clock.t += dt
            n._last_near_person_stop_receive_time = n._clock.now()
            self.refresh(n, rng)
            n._tick()
            modes.append(json.loads(n._command_pub.msgs[-1].data)["mode"])
        return modes

    # 【2026-11-04】向く(その場の旋回だけ)の間は、近接停止を効かせない。すぐ前の人の方も向く。
    def test_near_person_does_not_hold_face_turn(self):
        n = self.make(face=True, lidar=(40,))
        self.near(n, rng=0.6)
        modes = self.run_near(n, 1.2)
        self.assertEqual(modes[0], "NEAR_PERSON_HOLD")  # 最初の約0.4秒は、新しい測定を数回受けて距離を確かめる
        self.assertEqual(set(modes[-4:]), {"FACE_PERSON"})
        self.assertTrue([t for t in self.twists(n) if t.angular.z != 0.0])
        self.assertFalse([t for t in self.twists(n) if t.linear.x != 0.0])  # 前進は出さない

    def test_near_person_hold_when_object_inside_swept_circle(self):
        n = self.make(face=True, lidar=(40,))
        self.near(n, rng=0.30)  # 機体が回って触れる範囲の中(人でも物でも)
        self.assertEqual(set(self.run_near(n, 1.0)), {"NEAR_PERSON_HOLD"})
        self.assertFalse([t for t in self.twists(n) if t.angular.z != 0.0])

    def test_repeating_one_old_message_does_not_release_hold(self):
        # レビュー指摘: 同じ古い1通の繰り返しでは、免除しない(検出が遅れているとき、古い距離で旋回しない)
        n = self.make(face=True, lidar=(40,))
        self.near(n, rng=0.6)
        self._rx_fixed = n._person_candidates_rx
        modes = []
        for _ in range(12):  # メッセージは更新しない(mono固定)
            n._clock.t += 0.1
            n._last_near_person_stop_receive_time = n._clock.now()
            n._person_candidates_mono = self._mono
            n._person_candidates_rx = self._rx_fixed  # 受信時刻も同じ(古い1通の繰り返し)
            n._tick()
            modes.append(json.loads(n._command_pub.msgs[-1].data)["mode"])
        self.assertEqual(set(modes), {"NEAR_PERSON_HOLD"})

    def test_near_person_hold_when_distance_unknown(self):
        n = self.make(face=True, lidar=(40,))
        self.near(n, rng=0.6)
        n._person_candidates_mono = time.monotonic() - 10.0  # 候補が古い
        n._person_candidates_rx = n._clock.now().nanoseconds / 1e9 - 10.0
        self.assertEqual(set(self.modes(n, 0.5)), {"NEAR_PERSON_HOLD"})

    def test_near_person_hold_when_no_nearest_m(self):
        # nearest_m が無い(古い検出ノード等)・数でないときは、確認できないので停止のまま
        n = self.make(face=True, lidar=(40,))
        self.near(n, rng=0.6)
        for bad in (None, "0.6", True, float("nan")):
            n._near_clear_hist = []
            modes = []
            for _ in range(8):
                n._clock.t += 0.1
                n._last_near_person_stop_receive_time = n._clock.now()
                self.refresh(n)
                if bad is None:
                    n._person_candidates.pop("nearest_m")
                else:
                    n._person_candidates["nearest_m"] = bad
                n._tick()
                modes.append(json.loads(n._command_pub.msgs[-1].data)["mode"])
            self.assertEqual(set(modes), {"NEAR_PERSON_HOLD"}, bad)

    def test_near_person_hold_when_approach_on(self):
        n = self.make(face=True, approach=True, lidar=(40,))
        self.near(n, rng=0.6)
        self.assertNotIn("FACE_PERSON", self.run_near(n, 1.0))

    def test_near_person_hold_when_face_off(self):
        n = self.make(face=False, lidar=(40,))
        self.near(n, rng=0.6)
        self.assertEqual(set(self.run_near(n, 1.0)), {"NEAR_PERSON_HOLD"})

    def test_near_person_hold_when_param_off(self):
        n = self.make(face=True, lidar=(40,))
        n.face_turn_ignores_near_person = False
        self.near(n, rng=0.6)
        self.assertEqual(set(self.run_near(n, 1.0)), {"NEAR_PERSON_HOLD"})

    def test_near_person_hold_during_home_return(self):
        n = self.make(face=True, lidar=(40,))
        self.near(n, rng=0.6)
        n._home_return_active = True
        self.assertNotIn("FACE_PERSON", self.run_near(n, 1.0))

    def test_near_person_exempt_only_ends_in_face_modes(self):
        # レビュー指摘4: 免除したときに選ばれるのは FACE_PERSON / FACE_SWEEP だけ(直進を含む見回し等には行かない)。ゲート無効(従来動作)でも
        n = self.make(face=True, lidar=(), gate=False)
        self.near(n, rng=0.6)
        modes = self.run_near(n, 1.5)
        self.assertTrue(set(modes) <= {"NEAR_PERSON_HOLD", "FACE_PERSON", "FACE_SWEEP"}, set(modes))

    def test_near_person_sweep_is_exempt_when_far_enough(self):
        n = self.make(face=True, lidar=())
        n._get_pose = lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.near(n, rng=0.6, bearing=170)  # 範囲の外(後ろ)の人: 向かず見回し
        modes = self.run_near(n, 1.5)
        self.assertEqual(set(modes[-4:]), {"FACE_SWEEP"})
        self.assertFalse([t for t in self.twists(n) if t.linear.x != 0.0])

    def test_single_close_reading_returns_to_hold(self):
        # LiDARの距離は1回ごとにばらつく。免除中に1回でも近い測定が来たら、停止へ戻る
        n = self.make(face=True, lidar=(40,))
        self.near(n, rng=0.6)
        self.run_near(n, 1.0)
        modes = self.run_near(n, 0.1, rng=0.30)
        self.assertEqual(modes, ["NEAR_PERSON_HOLD"])

    # --- 2026-11-05: 古いスキャンの再送・nearest_age_s・基準の引き上げ・取り消し・crowd_state途絶・位置なし ---
    def run_near_age(self, n, seconds, age_of, dt=0.1):
        modes = []
        for i in range(int(round(seconds / dt))):
            n._clock.t += dt
            n._last_near_person_stop_receive_time = n._clock.now()
            self.refresh(n, None, age=age_of(i))
            n._tick()
            modes.append(json.loads(n._command_pub.msgs[-1].data)["mode"])
        return modes

    def test_resent_old_scan_does_not_release_hold(self):
        # 検出ノードがタイマーごとに同じスキャンの nearest_m を出し直す(nearest_age_s は時間とともに増える)=新しい測定ではない
        n = self.make(face=True, lidar=(40,))
        self.near(n, rng=0.6)
        modes = self.run_near_age(n, 1.5, lambda i: 0.1 * i if i < 6 else 0.6)
        self.assertEqual(set(modes), {"NEAR_PERSON_HOLD"})

    def test_age_over_limit_or_bad_age_holds(self):
        n = self.make(face=True, lidar=(40,))
        self.near(n, rng=0.6)
        for bad in (0.6, -0.1, float("nan"), "0.1", True, None):
            n._near_clear_hist = []
            modes = []
            for _ in range(8):
                n._clock.t += 0.1
                n._last_near_person_stop_receive_time = n._clock.now()
                self.refresh(n)
                if bad is None:
                    n._person_candidates.pop("nearest_age_s")  # 古い検出ノード(契約なし)も安全側
                else:
                    n._person_candidates["nearest_age_s"] = bad
                n._tick()
                modes.append(json.loads(n._command_pub.msgs[-1].data)["mode"])
            self.assertEqual(set(modes), {"NEAR_PERSON_HOLD"}, bad)

    def test_fresh_scans_with_small_age_release(self):
        n = self.make(face=True, lidar=(40,))
        self.near(n, rng=0.6)
        modes = self.run_near_age(n, 1.5, lambda i: 0.05)
        self.assertEqual(modes[-1], "FACE_PERSON")

    def test_start_threshold_raised_to_055(self):
        n = self.make(face=True, lidar=(40,))
        self.near(n, rng=0.52)  # 以前(0.45)なら免除した距離
        self.assertEqual(set(self.run_near(n, 1.5)), {"NEAR_PERSON_HOLD"})
        n2 = self.make(face=True, lidar=(40,))
        self.near(n2, rng=0.57)
        self.assertEqual(self.run_near(n2, 1.5)[-1], "FACE_PERSON")

    def test_exemption_entry_does_not_cancel_without_pending_goal(self):
        n = self.make(face=True, lidar=(40,))
        self.near(n, rng=0.6)
        self.run_near(n, 1.0)  # 停止に入った(取り消し1回)
        n._nav2_goal_pending = False
        before = len(n._cancel_goal_pub.msgs)
        n._near_exempt_entered = False
        n._face_only_step(n._clock.now())
        self.assertEqual(len(n._cancel_goal_pub.msgs), before)  # 送ったゴールが無ければ取り消さない
        n._nav2_goal_pending = True
        n._face_only_step(n._clock.now())
        self.assertEqual(len(n._cancel_goal_pub.msgs), before + 1)

    def test_crowd_state_dropout_does_not_exempt(self):
        n = self.make(face=True, lidar=(40,))
        self.near(n, rng=0.6)
        self.run_near(n, 1.0)
        n._crowd_state_mono = time.monotonic() - 100.0  # crowd_monitorが途絶えた
        self.assertEqual(set(self.run_near(n, 0.5)), {"NEAR_PERSON_HOLD"})

    def test_no_pose_candidates_limited_to_front_90(self):
        n = self.make(face=True, lidar=(40,))
        n._get_pose = lambda: None
        self.set_cands(n, lidar=(120,))  # 後ろ寄り(機体基準120度): 位置が無いなら対象外
        self.assertIsNone(n._face_target_bearing(n._clock.now()))
        n._face_selector = None
        self.set_cands(n, lidar=(60,))
        b = n._face_target_bearing(n._clock.now())
        self.assertAlmostEqual(math.degrees(b), 60, delta=1.0)

    def test_no_pose_grace_stops_turning(self):
        n = self.make(face=True, lidar=(40,))
        n._get_pose = lambda: None
        self.assertIsNotNone(n._face_target_bearing(n._clock.now()))
        self.set_cands(n, lidar=())
        n._clock.t += 0.3  # 猶予の中
        self.assertEqual(n._face_target_bearing(n._clock.now()), 0.0)  # 古い機体基準の方位で回り続けない

    def test_get_pose_stale_tf_is_none(self):
        n = self.make(face=True)
        n.tf_available = True
        n._clock.t = 100.0
        stamp = types.SimpleNamespace(sec=95, nanosec=0)
        tf = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=stamp),
            transform=types.SimpleNamespace(
                translation=types.SimpleNamespace(x=1.0, y=2.0),
                rotation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
        n.tf_buffer = types.SimpleNamespace(lookup_transform=lambda *a, **k: tf)
        saved = {k: sys.modules.get(k) for k in ('rclpy', 'rclpy.time')}
        self.addCleanup(lambda: sys.modules.update({k: v for k, v in saved.items() if v is not None}))
        sys.modules['rclpy.time'] = types.SimpleNamespace(Time=lambda: None)  # スタブの rclpy.time を、Time()が使える形にする
        sys.modules['rclpy'] = types.SimpleNamespace(time=sys.modules['rclpy.time'])
        real = type(n)._get_pose
        self.assertIsNone(real(n))  # 5秒前の変換=固まっている
        tf.header.stamp = types.SimpleNamespace(sec=99, nanosec=500000000)
        self.assertIsNotNone(real(n))
        del tf.header  # stampが取れない形式でも壊れない
        self.assertIsNotNone(real(n))

    def test_hysteresis_stays_exempt_between_stay_and_start(self):
        # 始める基準(0.55)と続ける基準(0.48)の間(0.50)は、免除中なら続ける
        n = self.make(face=True, lidar=(40,))
        self.near(n, rng=0.6)
        self.run_near(n, 1.0)
        self.assertEqual(set(self.run_near(n, 0.5, rng=0.50)), {"FACE_PERSON"})

    def test_hysteresis_does_not_start_between_stay_and_start(self):
        n = self.make(face=True, lidar=(40,))
        self.near(n, rng=0.50)
        self.assertEqual(set(self.run_near(n, 1.5)), {"NEAR_PERSON_HOLD"})

    def test_exemption_cancels_nav2_goal_once(self):
        n = self.make(face=True, lidar=(40,))
        self.near(n, rng=0.6)
        self.run_near(n, 1.5)
        count = len(n._cancel_goal_pub.msgs)  # 停止に入った1回+免除に入った1回
        self.assertLessEqual(count, 2)
        self.run_near(n, 1.5)
        self.assertEqual(len(n._cancel_goal_pub.msgs), count)  # 免除中は毎tick出さない

    def test_estop_and_manual_win(self):
        n = self.make(face=True, lidar=(40,))
        n._hardware_estop = True
        self.assertEqual(set(self.modes(n, 0.5)), {"ESTOP_HOLD"})

    # --- 範囲 ---
    def boundary(self, n, pose):
        n._boundary = mo.boundary_state(mo.make_rect(0.0, 0.0, 4.0, 4.0, 0.0), enabled=True)
        n._get_pose = lambda: pose

    def test_boundary_with_room_faces(self):
        n = self.make(face=True, lidar=(40,))
        self.boundary(n, {"x": 0.0, "y": 0.0, "yaw": 0.0})
        self.assertEqual(set(self.modes(n)), {"FACE_PERSON"})

    def test_boundary_edge_blocks_rotation(self):
        n = self.make(face=True, lidar=(40,))
        self.boundary(n, {"x": 1.8, "y": 0.0, "yaw": 0.0})
        modes = self.modes(n)
        self.assertEqual(set(modes), {"POLICY_NO_SCAN"})
        self.assertFalse([t for t in self.twists(n) if t.angular.z != 0.0 or t.linear.x != 0.0])
        self.assertTrue(self.last_cmd(n)["boundary_motion_blocked"])
        self.assertFalse(self.last_cmd(n)["face_active"])

    def test_boundary_unknown_pose_blocks_rotation(self):
        n = self.make(face=True, lidar=(40,))
        self.boundary(n, None)
        self.assertEqual(set(self.modes(n)), {"POLICY_NO_SCAN"})
        self.assertFalse([t for t in self.twists(n) if t.angular.z != 0.0])

    # --- ルール / 木の明示と禁止 ---
    def test_rule_face_enabled_overrides_off_switch(self):
        n = self.make([rule("a", policy="face_enabled")], face=False, lidar=(40,))
        self.assertEqual(set(self.modes(n)), {"FACE_PERSON"})

    def test_rule_face_disabled_wins_over_switch(self):
        n = self.make([rule("a", policy="face_disabled")], face=True, lidar=(40,))
        self.assertNotIn("FACE_PERSON", self.modes(n))

    def run_tree(self, root, n):
        r, _ = tbt.runner_for(root)
        st = tbt.ev(r, tbt.ctx("calm", True))
        n._rules_tick = lambda: None
        n._rule_effects = st.effects
        return st

    def test_tree_face_node_explicit_overrides_off_switch(self):
        n = self.make(face=False, lidar=(40,))
        self.run_tree(tbt.N("face"), n)
        self.assertEqual(set(self.modes(n)), {"FACE_PERSON"})

    def test_tree_face_disabled_wins_over_switch(self):
        n = self.make(face=True, lidar=(40,))
        self.run_tree(tbt.N("face", enabled=False), n)
        self.assertNotIn("FACE_PERSON", self.modes(n))

    # --- after_facing ---
    def after_facing_node(self, after=True):
        return tbt.N("sequence", [tbt.N("face"), tbt.N("announce", interval_seconds=30, after_facing=after)])

    def test_after_facing_announces_while_turning_at_regular_interval(self):
        # 【2026-11-04変更】向き終わるのを待たない。旋回中(向きを変えている間)も、いつもと同じ間隔で呼びかける
        n = self.make(face=False, lidar=(40,))
        self.run_tree(self.after_facing_node(), n)
        n._require_announce_opt_in = True
        self.modes(n, 1.0)
        self.assertEqual(len(n._announce_pub.msgs), 1)  # 旋回中でも出す(最初の1回)
        self.assertFalse(self.last_cmd(n)["face_aligned"])
        self.modes(n, 5.0)
        self.assertEqual(len(n._announce_pub.msgs), 1)  # 間隔(既定の最短間隔)の間は出さない
        n._clock.t += 70.0  # 間隔(60秒)が過ぎたら、向きが合う前でも、また出す
        self.modes(n, 0.5)
        self.assertEqual(len(n._announce_pub.msgs), 2)

    def test_after_facing_silent_when_not_in_a_face_mode(self):
        # 向く動作の間以外(向く対象も見回しも無い=範囲の外などで待機)は、after_facingの方針なので出さない
        n = self.make(face=False, lidar=())
        n._get_pose = lambda: None  # 位置が分からない: 見回さず待機(POLICY_NO_SCAN)
        self.run_tree(self.after_facing_node(), n)
        n._require_announce_opt_in = True
        self.modes(n, 1.0)
        self.assertFalse(n._announce_pub.msgs)

    def test_near_person_exempt_turn_still_announces(self):
        # 近接停止を免除して向いている間は、営業の呼びかけを止めない(旋回中も同じ間隔)
        n = self.make(face=True, lidar=(40,))
        n._require_announce_opt_in = False
        self.near(n, rng=0.6)
        modes = self.run_near(n, 1.5)
        self.assertEqual(set(modes[-4:]), {"FACE_PERSON"})
        self.assertTrue(n._announce_pub.msgs)

    def test_near_person_hold_still_silences_announce(self):
        # 免除できない(物が触れる範囲にある)ときの近接停止は、これまでどおり呼びかけない
        n = self.make(face=True, lidar=(40,))
        n._require_announce_opt_in = False
        self.near(n, rng=0.30)
        self.run_near(n, 1.5)
        self.assertFalse(n._announce_pub.msgs)

    def test_without_after_facing_announces_while_turning(self):
        n = self.make(face=False, lidar=(40,))
        self.run_tree(self.after_facing_node(after=False), n)
        n._require_announce_opt_in = True
        self.modes(n, 1.0)
        self.assertTrue(n._announce_pub.msgs)

    def test_after_facing_announces_as_before_when_face_unavailable(self):
        n = self.make(face=False, lidar=(40,))
        root = tbt.N("sequence", [tbt.N("face", enabled=False),
                                  tbt.N("announce", interval_seconds=30, after_facing=True)])
        self.run_tree(root, n)
        n._require_announce_opt_in = True
        self.modes(n, 1.0)
        self.assertTrue(n._announce_pub.msgs)  # 向くが禁止 → 永久に黙らない

    def test_after_facing_announces_when_face_switch_not_on(self):
        n = self.make(face=False, lidar=(40,))
        root = tbt.N("announce", interval_seconds=30, after_facing=True)
        self.run_tree(root, n)
        n._require_announce_opt_in = True
        self.modes(n, 1.0)
        self.assertTrue(n._announce_pub.msgs)  # 向くが使えない(スイッチOFF・明示なし)

    def test_after_facing_silent_with_no_person_when_face_allowed(self):
        n = self.make(face=False, lidar=())
        n._person_candidates = cands()
        self.run_tree(self.after_facing_node(), n)
        n._require_announce_opt_in = True
        self.modes(n, 1.0)
        self.assertFalse(n._announce_pub.msgs)


class RuleAndTreeValidationTests(unittest.TestCase):
    def test_rule_policies_exist(self):
        self.assertIn("face_enabled", br.POLICIES)
        self.assertIn("face_disabled", br.POLICIES)

    def test_conflicting_pair_rejected(self):
        with self.assertRaises(br.RuleError):
            base.ruleset(rule("a", policy=["face_enabled", "face_disabled"]))

    def test_single_policy_accepted_and_combinable(self):
        base.ruleset(rule("a", policy="face_enabled"))
        base.ruleset(rule("b", policy=["face_disabled", "approach_enabled"]))

    def test_effects(self):
        e = br.effects_for(("face_enabled",), {})
        self.assertTrue(e.face_allowed and e.face_explicit)
        e = br.effects_for(("face_disabled",), {})
        self.assertFalse(e.face_allowed)
        self.assertFalse(e.face_explicit)
        self.assertTrue(br.NEUTRAL.face_allowed)
        self.assertFalse(br.NEUTRAL.face_explicit)
        self.assertFalse(br.NEUTRAL.announce_after_facing)
        self.assertTrue(br.effects_for(("normal",), {"announce_min_interval_seconds": 20.0,
                                                     "announce_after_facing": True}).announce_after_facing)
        d = br.effects_for(("face_enabled",), {}).to_dict()
        self.assertIs(d["face_allowed"], True)
        self.assertIs(d["face_explicit"], True)
        self.assertIs(d["announce_after_facing"], False)

    def test_tree_face_node(self):
        r, _ = tbt.runner_for(tbt.N("face"))
        st = tbbt_ev(r)
        self.assertEqual(st.policies, ("face_enabled",))
        self.assertTrue(st.effects.face_explicit)
        r, _ = tbt.runner_for(tbt.N("face", enabled=False))
        st = tbbt_ev(r)
        self.assertEqual(st.policies, ("face_disabled",))
        self.assertFalse(st.effects.face_allowed)

    def test_tree_face_node_validation(self):
        errors, _w, _n = bt.validate_document(tbt.DOC(tbt.N("face", enabled="yes")))
        self.assertTrue(errors)
        errors, _w, _n = bt.validate_document(tbt.DOC(tbt.N("face", bogus=1)))
        self.assertTrue(errors)
        self.assertIn("face", bt.NODE_SPECS)
        self.assertIn("face_enabled", bt._POLICY_LABELS)  # noqa: SLF001

    def test_tree_announce_after_facing(self):
        r, _ = tbt.runner_for(tbt.N("announce", interval_seconds=20, after_facing=True))
        st = tbbt_ev(r)
        self.assertTrue(st.effects.announce_after_facing)
        self.assertEqual(st.effects.announce_min_interval_seconds, 20.0)
        r, _ = tbt.runner_for(tbt.N("announce", interval_seconds=20))
        self.assertFalse(tbbt_ev(r).effects.announce_after_facing)

    def test_tree_announce_after_facing_rejects_non_bool(self):
        for bad in ("true", 1, 0, "yes"):
            errors, _w, _n = bt.validate_document(
                tbt.DOC(tbt.N("announce", interval_seconds=20, after_facing=bad)))
            self.assertTrue(errors, bad)


def tbbt_ev(r):
    return tbt.ev(r, tbt.ctx("calm", True))


if __name__ == "__main__":
    unittest.main()
