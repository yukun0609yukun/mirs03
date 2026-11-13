"""「向く」機能のシミュレーション(test/face_person_sim.py)の結果を判定するテスト。ROS・実機なし。
実行: python ros2_ws/src/nokosahen_core/test/test_face_person_sim.py
"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import face_person_sim as fs  # noqa: E402


@unittest.skipIf(fs.bd is None, "behavior_director import失敗")
class FaceSimTests(unittest.TestCase):
    def run_scn(self, name):
        sim, target = fs.SCENARIOS[name]()
        self.addCleanup(sim.close)
        return sim, target, fs.summarize(name, sim, target)

    def assert_faces(self, name, tol_deg=6.0, align_within=8.0):
        sim, target, r = self.run_scn(name)
        self.assertLessEqual(abs(r["final_error_deg"]), tol_deg, r)
        self.assertIsNotNone(r["aligned_at_s"], r)
        self.assertLessEqual(r["aligned_at_s"], align_within, r)
        self.assertEqual(r["max_linear"], 0.0, r)  # 前進しない
        return sim, r

    def test_single_person_left(self):
        self.assert_faces("single_left", align_within=4.0)

    def test_single_person_at_the_side_turns_without_overshoot(self):
        sim, r = self.assert_faces("single_side", align_within=8.0)
        self.assertLessEqual(r["reversals"], 2)

    def test_person_behind_is_outside_the_range_and_ignored(self):
        """真後ろの人は、監視範囲(ONにした向きの前180度)の外。向かず、範囲の中を往復して見回すだけ(前進しない)。"""
        sim, target, r = self.run_scn("single_behind")
        self.assertEqual(r["max_linear"], 0.0, r)
        self.assertEqual(set(r["modes"]), {"FACE_SWEEP"}, r)
        yaws = [math.degrees(x["yaw"]) for x in sim.log]
        self.assertLessEqual(max(yaws), 95.0)
        self.assertGreaterEqual(min(yaws), -95.0)
        self.assertGreater(max(yaws), 60.0)   # 範囲の端近くまで見回す
        self.assertLess(min(yaws), -60.0)

    def test_sweeps_while_nobody_then_faces_the_person_who_appears(self):
        sim, r = self.assert_faces("sweep_then_person", tol_deg=8.0, align_within=14.0)
        modes_before = {x["mode"] for x in sim.log if x["t"] < 5.0}
        self.assertEqual(modes_before, {"FACE_SWEEP"})
        # 現れた後は向く(見回しに戻らない)。現れてから向き終わるまで数秒
        self.assertLessEqual(r["aligned_at_s"] - 5.0, 5.0, r)

    def test_faces_the_side_with_more_people(self):
        sim, r = self.assert_faces("crowd_majority", tol_deg=10.0, align_within=6.0)
        # 【2026-11-06】4人ともほぼ同じ距離(約2.6 m)。右の人は方位差が90度超なので間を向かず、左前の3人の側(正の方位)を向く
        self.assertGreater(math.degrees(sim.robot["yaw"]), 10.0)

    def test_tracks_a_moving_person_without_oscillating(self):
        sim, r = self.assert_faces("moving_person", tol_deg=10.0, align_within=6.0)
        self.assertLessEqual(r["reversals"], 6)

    def test_wall_clutter_does_not_pull_the_heading(self):
        self.assert_faces("wall_clutter", tol_deg=8.0)

    def test_switch_off_does_not_move(self):
        sim, target, r = self.run_scn("switch_off")
        self.assertEqual(r["max_linear"], 0.0)
        self.assertLess(abs(sim.robot["yaw"]), 1e-9)  # まったく旋回しない
        self.assertEqual(set(r["modes"]), {"POLICY_NO_SCAN"})

    def test_near_person_face_turn_faces_close_person(self):
        # 【2026-11-04】すぐ前(触れる範囲の外)の人は、近接停止が出ていても、その場の旋回で向く
        sim, target, r = self.run_scn("near_person_face_turn")
        self.assert_faces("near_person_face_turn")
        self.assertLess(abs(sim.robot["x"]), 1e-9)
        self.assertLess(abs(sim.robot["y"]), 1e-9)

    def test_near_person_safety_stop_wins(self):
        sim, target, r = self.run_scn("near_person_hold")
        self.assertLess(abs(sim.robot["yaw"]), 1e-9)
        self.assertEqual(set(r["modes"]), {"NEAR_PERSON_HOLD"})

    def test_boundary_with_room_faces_and_stays_inside(self):
        sim, r = self.assert_faces("boundary_room")
        self.assertTrue(r["inside_boundary"], r)

    def test_boundary_edge_does_not_rotate(self):
        sim, target, r = self.run_scn("boundary_edge")
        self.assertLess(abs(sim.robot["yaw"]), 1e-9)
        self.assertTrue(r["inside_boundary"], r)
        self.assertEqual(set(r["modes"]), {"POLICY_NO_SCAN"})

    def test_rotated_boundary_stays_inside(self):
        sim, r = self.assert_faces("rotated_boundary_corner", align_within=8.0)
        self.assertTrue(r["inside_boundary"], r)

    def test_after_facing_speaks_while_turning(self):
        # 【2026-11-04変更】向き終わるのを待たず、向く動作を始めたら(旋回中でも)呼びかける
        sim, target, r = self.run_scn("after_facing")
        self.assertIsNotNone(r["aligned_at_s"], r)
        self.assertIsNotNone(r["first_announce_s"], r)
        self.assertLess(r["first_announce_s"], r["aligned_at_s"])

    def test_robust_over_random_seeds(self):
        """センサのノイズ・見逃しの乱数を変えても、向きが合う(複数の乱数で20回)。"""
        bad = []
        for seed in range(20):
            p = fs.Person(2.0, 1.4)
            sim = fs.Sim(people=[p], seed=seed).run(10)
            err = abs(sim.heading_error_deg(fs.world_bearing((0, 0), p)))
            if err > 8.0 or sim.max_linear() != 0.0:
                bad.append((seed, round(err, 1)))
            sim.close()
        self.assertEqual(bad, [])

    def test_robust_with_noisy_sensors(self):
        """ノイズを大きく、見逃しも増やしても、向く(精度は少し落ちてよい)。"""
        errs = []
        for seed in range(10):
            p = fs.Person(2.0, -1.2)
            sensor = fs.SensorModel(seed, bearing_sigma_deg=4.0, lidar_miss=0.2, camera_miss=0.4)
            sim = fs.Sim(people=[p], sensor=sensor).run(12)
            errs.append(abs(sim.heading_error_deg(fs.world_bearing((0, 0), p))))
            sim.close()
        self.assertLessEqual(max(errs), 15.0, errs)


if __name__ == "__main__":
    unittest.main()
