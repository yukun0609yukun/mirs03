import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from nokosahen_core import crowd_bearing as cb  # noqa: E402

R = math.radians


def cand(deg, w=1.0, rng=None):
    return {"bearing": R(deg), "range": rng, "weight": w, "source": "both"}


class FuseTest(unittest.TestCase):
    def test_match(self):
        o = cb.fuse_candidates([{"bearing": R(10), "range": 2.0, "confirmed": False}],
                               [{"bearing": R(14)}])
        self.assertEqual(len(o), 1)
        self.assertEqual(o[0]["source"], "both")
        self.assertEqual(o[0]["weight"], 1.0)
        self.assertEqual(o[0]["range"], 2.0)

    def test_lidar_only(self):
        o = cb.fuse_candidates([{"bearing": 0.1, "range": 1.0, "confirmed": False}], [])
        self.assertEqual((o[0]["source"], o[0]["weight"]), ("lidar", cb.LIDAR_ONLY_WEIGHT))

    def test_camera_only(self):
        o = cb.fuse_candidates([], [{"bearing": 0.1}])
        self.assertEqual((o[0]["source"], o[0]["weight"], o[0]["range"]),
                         ("camera", cb.CAMERA_ONLY_WEIGHT, None))

    def test_confirmed(self):
        o = cb.fuse_candidates([{"bearing": 0.1, "range": 1.0, "confirmed": True}], [])
        self.assertEqual((o[0]["source"], o[0]["weight"]), ("both", 1.0))

    def test_no_double_count(self):
        o = cb.fuse_candidates(
            [{"bearing": R(10), "range": 1, "confirmed": False},
             {"bearing": R(15), "range": 2, "confirmed": False}],
            [{"bearing": R(14)}])
        self.assertEqual(len(o), 2)
        both = [c for c in o if c["source"] == "both"]
        self.assertEqual(len(both), 1)
        self.assertAlmostEqual(both[0]["bearing"], R(15))
        self.assertEqual(sum(1 for c in o if c["source"] == "lidar"), 1)

    def test_boundary(self):
        for d in (8, 7.9, -8):
            o = cb.fuse_candidates([{"bearing": 0.0, "range": 1, "confirmed": False}],
                                   [{"bearing": R(d)}])
            self.assertEqual(len(o), 1, d)
        o = cb.fuse_candidates([{"bearing": 0.0, "range": 1, "confirmed": False}],
                               [{"bearing": R(8.5)}])
        self.assertEqual(len(o), 2)

    def test_invalid(self):
        o = cb.fuse_candidates(
            [{"bearing": float("nan"), "range": 1}, {"bearing": "x"}, None, {"bearing": True},
             {"bearing": 0.2, "range": "bad", "confirmed": False}],
            [{"bearing": float("inf")}, {"bearing": None}, 5, {}])
        self.assertEqual(len(o), 1)
        self.assertIsNone(o[0]["range"])
        self.assertEqual(cb.fuse_candidates(None, "abc"), [])

    def test_wrap(self):
        o = cb.fuse_candidates([], [{"bearing": 3 * math.pi / 2}])
        self.assertAlmostEqual(o[0]["bearing"], -math.pi / 2)

    def test_wrap_match_across_pi(self):
        o = cb.fuse_candidates([{"bearing": R(178), "range": 1, "confirmed": False}],
                               [{"bearing": R(-178)}])
        self.assertEqual(len(o), 1)


class SelectorTest(unittest.TestCase):
    def test_single(self):
        s = cb.FaceBearingSelector()
        self.assertAlmostEqual(s.update(0, [cand(40, 0.5)]), R(40))

    def test_none_resets(self):
        s = cb.FaceBearingSelector()
        s.update(0, [cand(0)])
        self.assertIsNone(s.update(1, []))
        # リセット後は保持時間なしで新しい扇形へ
        self.assertAlmostEqual(s.update(1.1, [cand(90)]), R(90))

    def test_multi_majority(self):
        s = cb.FaceBearingSelector()
        b = s.update(0, [cand(-60), cand(50), cand(60), cand(65, 0.5)])
        self.assertAlmostEqual(math.degrees(b), (50 + 60 + 65 * 0.5) / 2.5, places=0)

    def test_weights_not_count(self):
        s = cb.FaceBearingSelector()
        b = s.update(0, [cand(-90, 1.0), cand(90, 0.5), cand(92, 0.4)])
        self.assertLess(b, 0)  # 1.0 > 0.9

    def test_tie_nearest(self):
        s = cb.FaceBearingSelector()
        b = s.update(0, [cand(-90, 1.0, 3.0), cand(90, 1.0, 1.5)])
        self.assertAlmostEqual(b, R(90))

    def test_tie_current(self):
        s = cb.FaceBearingSelector()
        s.update(0, [cand(-90, 1.0)])
        b = s.update(0.1, [cand(-90, 1.0), cand(90, 1.2)])  # 1.2 vs 1.0*1.2 の同点
        self.assertAlmostEqual(b, R(-90))

    def test_sector_boundary(self):
        s = cb.FaceBearingSelector()
        # 14度は扇形0、16・17度は扇形1(境界は15度)
        b = s.update(0, [cand(14, 1.0), cand(16, 1.0), cand(17, 1.0)])
        self.assertAlmostEqual(math.degrees(b), 16.5, places=1)

    def test_front_not_boundary(self):
        s = cb.FaceBearingSelector()
        b = s.update(0, [cand(-5), cand(5)])
        self.assertAlmostEqual(b, 0.0, places=6)

    def test_rear_wrap(self):
        s = cb.FaceBearingSelector()
        b = s.update(0, [cand(178), cand(-178)])
        self.assertAlmostEqual(abs(b), math.pi, places=3)

    def test_hold_then_switch(self):
        s = cb.FaceBearingSelector()
        s.update(0, [cand(0, 1.0)])
        crowd = [cand(0, 1.0), cand(90, 1.0), cand(92, 1.0)]
        hold = cb.SWITCH_HOLD_SEC
        self.assertAlmostEqual(s.update(0.1, crowd), 0.0, places=6)  # 2.0 > 1.6 だが保持時間未満
        self.assertAlmostEqual(s.update(0.1 + hold * 0.5, crowd), 0.0, places=6)
        self.assertAlmostEqual(s.update(0.1 + hold * 0.9, crowd), 0.0, places=6)
        b = s.update(0.1 + hold + 0.2, crowd)
        self.assertAlmostEqual(math.degrees(b), 91, places=0)

    def test_hysteresis_gain(self):
        s = cb.FaceBearingSelector()
        s.update(0, [cand(0, 1.0)])
        b = None
        for t in range(1, 50):
            b = s.update(t, [cand(0, 1.0), cand(90, 1.15)])  # 1.15 < 1.2
        self.assertAlmostEqual(b, 0.0, places=6)

    def test_flicker_boundary(self):
        s = cb.FaceBearingSelector()
        outs = []
        for i in range(100):
            deg = 14.0 if i % 2 == 0 else 16.0  # 扇形境界(15度)をまたいで揺れる
            outs.append(s.update(i * 0.1, [cand(deg)]))
        self.assertTrue(all(o is not None for o in outs))
        changes = sum(1 for a, b in zip(outs, outs[1:]) if abs(math.degrees(a - b)) > 5)
        self.assertEqual(changes, 0)

    def test_flicker_two_groups(self):
        s = cb.FaceBearingSelector()
        sectors = []
        for i in range(100):
            w2 = 1.0 + (0.3 if i % 2 else -0.3)  # 毎tick大小が入れ替わる
            b = s.update(i * 0.1, [cand(-90, 1.0), cand(90, w2)])
            sectors.append(round(math.degrees(b) / 90))
        changes = sum(1 for a, b in zip(sectors, sectors[1:]) if a != b)
        self.assertLessEqual(changes, 2)


class MonitorRangeTest(unittest.TestCase):
    """【2026-11-04】監視範囲: スイッチONの向き(ref)の前 ±90度。"""

    def test_inside_and_outside(self):
        self.assertTrue(cb.in_monitor_range(R(60), 0.0, 0.0))
        self.assertTrue(cb.in_monitor_range(R(-89), 0.0, 0.0))
        self.assertFalse(cb.in_monitor_range(R(91), 0.0, 0.0))
        self.assertFalse(cb.in_monitor_range(R(180), 0.0, 0.0))

    def test_uses_world_heading_not_the_current_front(self):
        # 車体が今45度を向いている。機体基準で-60度の人は、世界では-15度 → 範囲の中
        self.assertTrue(cb.in_monitor_range(R(-60), R(45), 0.0))
        # 機体基準で+60度は、世界では105度 → 範囲の外
        self.assertFalse(cb.in_monitor_range(R(60), R(45), 0.0))

    def test_wraps_around_pi(self):
        self.assertTrue(cb.in_monitor_range(R(10), R(170), R(-170)))   # 差は20度
        self.assertFalse(cb.in_monitor_range(R(10), R(170), R(10)))    # 差は170度

    def test_bad_input_is_outside(self):
        self.assertFalse(cb.in_monitor_range(None, 0.0, 0.0))
        self.assertFalse(cb.in_monitor_range(0.0, float("nan"), 0.0))


class SweepDirectionTest(unittest.TestCase):
    def test_keeps_direction_in_the_middle(self):
        self.assertEqual(cb.sweep_direction(0.0, 0.0, 1.0), 1.0)
        self.assertEqual(cb.sweep_direction(R(30), 0.0, -1.0), -1.0)

    def test_turns_back_before_each_edge(self):
        self.assertEqual(cb.sweep_direction(R(86), 0.0, 1.0), -1.0)
        self.assertEqual(cb.sweep_direction(R(-86), 0.0, -1.0), 1.0)
        self.assertEqual(cb.sweep_direction(R(80), 0.0, 1.0), 1.0)    # まだ手前

    def test_returns_when_outside(self):
        self.assertEqual(cb.sweep_direction(R(120), 0.0, 1.0), -1.0)
        self.assertEqual(cb.sweep_direction(R(-120), 0.0, -1.0), 1.0)

    def test_relative_to_reference(self):
        self.assertEqual(cb.sweep_direction(R(106), R(20), 1.0), -1.0)   # 基準20度から見て86度: 折り返す
        self.assertEqual(cb.sweep_direction(R(100), R(20), 1.0), 1.0)    # 80度: まだ

    def test_full_sweep_stays_inside(self):
        yaw, d, lo, hi = 0.0, 1.0, 0.0, 0.0
        for _ in range(4000):
            d = cb.sweep_direction(yaw, 0.0, d)
            yaw += d * 0.02
            lo, hi = min(lo, yaw), max(hi, yaw)
        self.assertLess(hi, R(91))
        self.assertGreater(lo, R(-91))
        self.assertGreater(hi, R(80))
        self.assertLess(lo, R(-80))


class WeightTest(unittest.TestCase):
    """【2026-11-04】LiDARの足を優先: LiDARだけ > カメラだけ。"""

    def test_lidar_outweighs_camera_only(self):
        self.assertGreater(cb.LIDAR_ONLY_WEIGHT, cb.CAMERA_ONLY_WEIGHT)
        self.assertEqual((cb.LIDAR_ONLY_WEIGHT, cb.CAMERA_ONLY_WEIGHT), (0.75, 0.25))

    def test_selector_prefers_the_lidar_sector_over_camera_only_sector(self):
        sel = cb.FaceBearingSelector()
        cands = cb.fuse_candidates([{"bearing": R(-60), "range": 2.0, "confirmed": False}], [{"bearing": R(60)}])
        b = sel.update(0.0, cands)
        self.assertAlmostEqual(math.degrees(b), -60.0, places=3)

    def test_one_lidar_only_beats_two_camera_only(self):
        sel = cb.FaceBearingSelector()
        cands = cb.fuse_candidates([{"bearing": R(-60), "range": 2.0, "confirmed": False}],
                                   [{"bearing": R(60)}, {"bearing": R(62)}])
        self.assertAlmostEqual(math.degrees(sel.update(0.0, cands)), -60.0, places=3)  # 0.25+0.25=0.5 < 0.75


class TurnTest(unittest.TestCase):
    def test_stop_at_5(self):
        self.assertEqual(cb.face_turn_command(R(4.9), 0.5, False), (0.0, True))
        w, a = cb.face_turn_command(R(6), 0.5, False)
        self.assertNotEqual(w, 0.0)
        self.assertFalse(a)

    def test_no_reengage_until_10(self):
        for d in (6, 9, -9.9):
            self.assertEqual(cb.face_turn_command(R(d), 0.5, True), (0.0, True))
        w, a = cb.face_turn_command(R(10.5), 0.5, True)
        self.assertGreater(w, 0)
        self.assertFalse(a)

    def test_direction(self):
        self.assertGreater(cb.face_turn_command(R(40), 0.5, False)[0], 0)
        self.assertLess(cb.face_turn_command(R(-40), 0.5, False)[0], 0)

    def test_wrap_shortest(self):
        self.assertLess(cb.face_turn_command(R(270), 0.5, False)[0], 0)

    def test_speed_limits(self):
        self.assertAlmostEqual(cb.face_turn_command(R(120), 0.5, False)[0], 0.5)
        self.assertAlmostEqual(cb.face_turn_command(R(-179), 0.5, False)[0], -0.5)
        self.assertAlmostEqual(cb.face_turn_command(R(30), 0.5, False)[0], 0.25)
        self.assertAlmostEqual(cb.face_turn_command(R(6), 0.5, False)[0], 0.125)  # 下限 0.25*max
        self.assertAlmostEqual(cb.face_turn_command(R(45), 0.8, False)[0], 0.6)

    def test_bad_input(self):
        self.assertEqual(cb.face_turn_command(float("nan"), 0.5, True), (0.0, True))
        self.assertEqual(cb.face_turn_command(1.0, 0.0, False), (0.0, False))


class TrackingWhileRotatingTest(unittest.TestCase):
    """ロボットが旋回すると、人は機体基準で扇形をまたいで動く。現在の扇形が空になったら、すぐ今の方位を返す
    (切替待ちの古い方位を返し続けない。2026-11-04、シミュレーションで見つけた)。"""

    def test_single_person_sweeping_across_sectors_is_tracked_every_tick(self):
        sel = cb.FaceBearingSelector()
        t = 0.0
        for deg in range(-170, 0, 3):   # 人が -170度 から 0度 へ、機体基準で動いていく(ロボットが旋回している)
            t += 0.1
            b = sel.update(t, [{"bearing": math.radians(deg), "range": 2.0, "weight": 1.0, "source": "both"}])
            self.assertIsNotNone(b)
            self.assertAlmostEqual(math.degrees(b), deg, delta=1.0, msg=f"t={t:.1f} deg={deg}")

    def test_far_candidate_after_dropout_waits_the_hold_time(self):
        """今の人が消えて、遠く(45度より離れた)の別の人だけが残ったときは、見逃しの可能性があるので、1秒は待つ。"""
        sel = cb.FaceBearingSelector()
        # 【2026-11-06】向く先は一番近い人(左の2人が近い=1.0 m、右の人は2.0 m)
        left = [{"bearing": math.radians(40), "range": 1.0, "weight": 1.0}, {"bearing": math.radians(42), "range": 1.0, "weight": 1.0}]
        right = {"bearing": math.radians(-80), "range": 2.0, "weight": 1.0}
        b = sel.update(0.0, left + [right])
        self.assertGreater(b, 0)
        b = sel.update(0.1, [right])            # 左の2人が一瞬消えた
        self.assertGreater(b, 0)                # まだ左(直前の方位)
        hold = cb.SWITCH_HOLD_SEC
        b = sel.update(0.1 + hold * 0.5, [right])
        self.assertGreater(b, 0)
        b = sel.update(0.1 + hold + 0.2, [right])  # 保持時間以上続いた → 右へ
        self.assertAlmostEqual(math.degrees(b), -80, delta=1.0)

    def test_nearby_candidate_after_crossing_sector_is_followed_immediately(self):
        sel = cb.FaceBearingSelector()
        b = sel.update(0.0, [{"bearing": math.radians(14), "range": 2.0, "weight": 1.0}])
        b = sel.update(0.1, [{"bearing": math.radians(17), "range": 2.0, "weight": 1.0}])  # 扇形の境界(15度)をまたいだ
        self.assertAlmostEqual(math.degrees(b), 17, delta=0.5)


def _pt(deg, r):
    return {"bearing": R(deg), "range": r, "confirmed": False}


def _leg_xy(cx, cy):
    """机体基準の位置(x前,y左)から LiDAR候補を作る。"""
    return {"bearing": math.atan2(cy, cx), "range": math.hypot(cx, cy), "confirmed": False}


class LegDeskTest(unittest.TestCase):
    """【2026-11-04】机・椅子の脚(静止物)より、カメラ確認済みの人を選ぶ。"""

    def _best_bearing(self, cands):
        sel = cb.FaceBearingSelector()
        return sel.update(0.0, cands)

    def test_four_leg_desk_loses_to_confirmed_person(self):
        # 60度の方に、0.6m四方の机の4本脚(2m先)。正面に、カメラ確認済みの人(脚2本)。
        # 【2026-11-06】カメラの視野内の机の脚は、検出ノードが rejected(人と認められない)にする。向く先は「一番近い人」なので、
        # 机の脚が人と同じ距離にあると間を向いてしまう。rejected の脚は(他に候補があれば)対象にしない。
        desk = [dict(_leg_xy(1.0 + dx, 1.73 + dy), rejected=True) for dx in (0.0, 0.6) for dy in (0.0, 0.6)]
        person = [{"bearing": R(-2), "range": 2.0, "confirmed": True},
                  {"bearing": R(2), "range": 2.0, "confirmed": True}]
        fused = cb.fuse_candidates(desk + person, [{"bearing": R(-2)}, {"bearing": R(2)}])
        b = self._best_bearing(fused)
        self.assertLess(abs(math.degrees(b)), 15)

    def test_leg_pair_heavier_than_single_leg(self):
        pair = [_leg_xy(2.0, 0.1), _leg_xy(2.0, -0.1)]
        single = [_pt(90, 2.0)]
        fused = cb.fuse_candidates(pair + single, [])
        wp = sum(c["weight"] for c in fused if abs(c["bearing"]) < R(20))
        ws = sum(c["weight"] for c in fused if abs(c["bearing"] - R(90)) < R(5))
        self.assertGreater(wp, ws)
        self.assertLess(wp, 2 * cb.LIDAR_ONLY_WEIGHT)  # 2本でも1人分程度

    def test_rejected_cluster_is_nearly_ignored(self):
        l = [{"bearing": R(30), "range": 2.0, "confirmed": False, "rejected": True}]
        o = cb.fuse_candidates(l, [])
        self.assertLessEqual(o[0]["weight"], 0.1 + 1e-9)
        # 印の無い古いメッセージは従来どおり
        o = cb.fuse_candidates([{"bearing": R(30), "range": 2.0, "confirmed": False}], [])
        self.assertEqual(o[0]["weight"], cb.LIDAR_ONLY_WEIGHT)

    def test_rejected_loses_to_unrejected(self):
        l = [{"bearing": R(60), "range": 2.0, "confirmed": False, "rejected": True},
             {"bearing": R(-60), "range": 2.0, "confirmed": False}]
        b = self._best_bearing(cb.fuse_candidates(l, []))
        self.assertLess(b, 0)


if __name__ == "__main__":
    unittest.main()
