"""2026-11-05 実機で見つかった「向く」が回り続ける件の修正(純粋ロジック)。

1. 旋回指令の下限(min_rps): 0.6 rad/s 前後以下では車体が回らなかった
2. 範囲の縁のヒステリシス(in_monitor_range_sticky): 追っている人が縁で出入りして目標がちらつかない
3. 機体の枠(LiDAR中心から近すぎる候補)を向く対象にしない
"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core import crowd_bearing as cb  # noqa: E402


class MinTurnSpeedTests(unittest.TestCase):
    def test_small_error_command_is_at_least_min(self):
        # 誤差23°: 従来は約0.57 rad/s で車体が回らなかった。下限0.9なら0.9以上
        ang, aligned = cb.face_turn_command(math.radians(23), max_rps=1.5, aligned=False, min_rps=0.9)
        self.assertFalse(aligned)
        self.assertGreaterEqual(abs(ang), 0.9 - 1e-9)
        self.assertGreater(ang, 0)

    def test_default_keeps_old_behavior(self):
        ang, _ = cb.face_turn_command(math.radians(23), max_rps=1.5, aligned=False)
        self.assertLess(abs(ang), 0.9)

    def test_right_turn_sign_kept(self):
        ang, _ = cb.face_turn_command(math.radians(-23), max_rps=1.5, aligned=False, min_rps=0.9)
        self.assertLess(ang, 0)
        self.assertGreaterEqual(abs(ang), 0.9 - 1e-9)

    def test_aligned_zone_still_stops(self):
        ang, aligned = cb.face_turn_command(math.radians(4), max_rps=1.5, aligned=False, min_rps=0.9)
        self.assertEqual(ang, 0.0)
        self.assertTrue(aligned)
        ang, aligned = cb.face_turn_command(math.radians(9), max_rps=1.5, aligned=True, min_rps=0.9)
        self.assertEqual(ang, 0.0)
        self.assertTrue(aligned)

    def test_min_never_exceeds_max(self):
        ang, _ = cb.face_turn_command(math.radians(30), max_rps=0.5, aligned=False, min_rps=0.9)
        self.assertLessEqual(abs(ang), 0.5 + 1e-9)

    def test_large_error_uses_max(self):
        ang, _ = cb.face_turn_command(math.radians(120), max_rps=1.5, aligned=False, min_rps=0.9)
        self.assertAlmostEqual(abs(ang), 1.5)

    def test_bad_min_is_ignored(self):
        ang, _ = cb.face_turn_command(math.radians(23), max_rps=1.5, aligned=False, min_rps=float("nan"))
        self.assertLess(abs(ang), 0.9)


class StickyMonitorRangeTests(unittest.TestCase):
    # 基準の向き 0、機体の向き 0 のとき、範囲は 機体基準の方位 ±90°
    def test_inside_is_true(self):
        self.assertTrue(cb.in_monitor_range_sticky(math.radians(80), 0.0, 0.0, None))

    def test_outside_without_last_target_is_false(self):
        self.assertFalse(cb.in_monitor_range_sticky(math.radians(100), 0.0, 0.0, None))

    def test_just_outside_but_following_is_true(self):
        # 追っていた人(世界の95°)が、縁(90°)の少し外(100°)へ出ても残す
        self.assertTrue(cb.in_monitor_range_sticky(math.radians(100), 0.0, 0.0, math.radians(95)))

    def test_far_outside_even_when_following_is_false(self):
        self.assertFalse(cb.in_monitor_range_sticky(math.radians(125), 0.0, 0.0, math.radians(95)))

    def test_other_person_near_edge_not_sticky(self):
        # 追っていた人(世界の-40°)とは別の人が縁の外(100°)にいる: 残さない
        self.assertFalse(cb.in_monitor_range_sticky(math.radians(100), 0.0, 0.0, math.radians(-40)))

    def test_bad_values(self):
        self.assertFalse(cb.in_monitor_range_sticky(None, 0.0, 0.0, 0.0))
        self.assertFalse(cb.in_monitor_range_sticky(math.radians(100), float("nan"), 0.0, 0.0))


class BodyCandidateTests(unittest.TestCase):
    def test_close_lidar_is_body(self):
        self.assertTrue(cb.is_body_candidate({"bearing": 1.58, "range": 0.3}))

    def test_person_range_is_not_body(self):
        self.assertFalse(cb.is_body_candidate({"bearing": 0.0, "range": 0.6}))

    def test_camera_only_is_not_body(self):
        self.assertFalse(cb.is_body_candidate({"bearing": 0.0, "range": None}))

    def test_custom_range_and_bad_input(self):
        self.assertFalse(cb.is_body_candidate({"range": 0.3}, body_range_m=0.2))
        self.assertFalse(cb.is_body_candidate(None))
        self.assertFalse(cb.is_body_candidate({"range": "x"}))

    def test_body_candidate_not_counted_after_filter(self):
        fused = cb.fuse_candidates(
            [{"bearing": 1.58, "range": 0.3, "confirmed": False}], [])
        kept = [c for c in fused if not cb.is_body_candidate(c)]
        self.assertEqual(kept, [])


class TargetStickinessTests(unittest.TestCase):
    """実機(2026-11-05)の再現: 追っているカメラの人(重み0.25)から、LiDARだけの足2つ(合計0.9)が1〜2秒見えるたびに切り替わって往復した。"""

    @staticmethod
    def _cams(extra):
        return [{"bearing": math.radians(10), "range": None, "weight": cb.CAMERA_ONLY_WEIGHT}] + extra

    def test_short_lidar_legs_do_not_steal_the_target(self):
        legs = [{"bearing": math.radians(69), "range": 1.3, "weight": cb.LIDAR_ONLY_WEIGHT * cb.LEG_PAIR_FACTOR},
                {"bearing": math.radians(78), "range": 1.1, "weight": cb.LIDAR_ONLY_WEIGHT * cb.LEG_PAIR_FACTOR}]
        sel = cb.FaceBearingSelector()
        b = sel.update(0.0, self._cams([]))
        self.assertAlmostEqual(math.degrees(b), 10, delta=1)
        t = 0.1
        while t < 2.0:  # 足が2秒弱見えている間
            b = sel.update(t, self._cams(legs))
            self.assertAlmostEqual(math.degrees(b), 10, delta=1, msg=f"t={t}")
            t += 0.2
        b = sel.update(2.1, self._cams([]))  # 足が消えた → 元のまま
        self.assertAlmostEqual(math.degrees(b), 10, delta=1)

    def test_lasting_crowd_still_wins(self):
        legs = [{"bearing": math.radians(69), "range": 1.3, "weight": 1.0},
                {"bearing": math.radians(78), "range": 1.1, "weight": 1.0}]
        sel = cb.FaceBearingSelector()
        sel.update(0.0, self._cams([]))
        t = 0.1
        b = None
        while t < cb.SWITCH_HOLD_SEC + 1.0:
            b = sel.update(t, self._cams(legs))
            t += 0.2
        self.assertGreater(math.degrees(b), 60)


class SimWithMinTurnTests(unittest.TestCase):
    """実際の director(スタブ)で、下限0.9でも往復せず整列し、前進が出ないことを見る(モーターの加減速は模さない)。"""

    def _run(self, person):
        import face_person_sim as fps
        s = fps.Sim(people=[person])
        s.n.face_turn_min_rps = 0.9
        s.run(12)
        err = abs(s.heading_error_deg(fps.world_bearing((0, 0), person)))
        out = (err, s.reversals(), s.first_aligned_time(), s.max_linear())
        s.close()
        return out

    def test_converges_without_hunting(self):
        import face_person_sim as fps
        for p in (fps.Person(2.0, 1.2), fps.Person(2.0, 0.4)):
            err, rev, t_al, max_lin = self._run(p)
            self.assertLess(err, 6.0)
            self.assertEqual(rev, 0)
            self.assertIsNotNone(t_al)
            self.assertEqual(max_lin, 0.0)


if __name__ == "__main__":
    unittest.main()
