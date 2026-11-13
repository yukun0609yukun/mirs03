"""【2026-11-06】向く先を「一番近い人」にした FaceBearingSelector のテスト(純粋ロジック + face_person_sim の閉ループ)。
設計: docs/face_person_design.md 7.8章。"""
import math
import os
import random
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from nokosahen_core import crowd_bearing as cb  # noqa: E402

R = math.radians
D = math.degrees


def lc(deg, rng, w=cb.LIDAR_ONLY_WEIGHT):
    return {"bearing": R(deg), "range": rng, "weight": w, "source": "lidar"}


def cam(deg):
    return {"bearing": R(deg), "range": None, "weight": cb.CAMERA_ONLY_WEIGHT, "source": "camera"}


def adiff(a_deg, b_deg):
    return abs((a_deg - b_deg + 180.0) % 360.0 - 180.0)


def noisy(rng, persons, sb=1.5, sr=0.05, miss=0.05):
    """persons: [(bearing_deg, range_m)] をセンサ雑音つきの候補にする(LiDARだけ)。"""
    out = []
    for b, r in persons:
        if rng.random() < miss:
            continue
        out.append(lc(b + rng.gauss(0, sb), r + rng.gauss(0, sr)))
    return out


def run(sel, persons_fn, seconds, rng, t0=0.0, hz=10.0, yaw=None):
    """persons_fn(t) -> [(deg, range)]。戻り値: 方位[deg]の列(Noneは除く)。"""
    outs = []
    n = int(seconds * hz)
    for i in range(n):
        t = t0 + i / hz
        c = noisy(rng, persons_fn(t))
        if not c:       # behavior_director は候補が空のときは update を呼ばない(猶予の間、直前を保つ)
            continue
        b = sel.update(t, c, yaw=yaw)
        if b is not None:
            outs.append(D(b))
    return outs


def make_crowd(rng, n):
    """一番近い人(1.2〜1.8 m)と、それより0.8 m以上遠い n-1 人。方位は±80度で10度以上離す。戻り値 (人のリスト, 一番近い人の方位)。"""
    r0 = rng.uniform(1.2, 1.8)
    bearings = []
    while len(bearings) < n:
        b = rng.uniform(-80, 80)
        if all(abs(b - x) >= 10 for x in bearings):
            bearings.append(b)
    persons = [(bearings[0], r0)] + [(b, rng.uniform(r0 + 0.8, 5.5)) for b in bearings[1:]]
    return persons, bearings[0]


class NearestLogicTest(unittest.TestCase):
    def test_nearest_person_is_faced_for_1_2_5_10_people(self):
        for n in (1, 2, 5, 10):
            for seed in range(12):
                rng = random.Random(1000 * n + seed)
                persons, expect = make_crowd(rng, n)
                sel = cb.FaceBearingSelector()
                outs = run(sel, lambda t: persons, 6.0, rng)
                self.assertLessEqual(adiff(outs[-1], expect), 4.0, f"n={n} seed={seed} outs={outs[-1]:.1f} expect={expect:.1f}")
                # 最初から最後まで、ずっと一番近い人
                self.assertTrue(all(adiff(o, expect) <= 6.0 for o in outs[2:]), f"n={n} seed={seed}")
                self.assertEqual(sel.anchor_switches, 0)

    def test_nearest_not_majority(self):
        # 左に遠い3人、右に近い1人: 多数派ではなく近い方
        sel = cb.FaceBearingSelector()
        b = sel.update(0.0, [lc(60, 3.0), lc(62, 3.1), lc(58, 3.2), lc(-60, 1.5)])
        self.assertAlmostEqual(D(b), -60.0, places=3)

    def test_same_distance_pair_looks_between(self):
        cases = [
            ((20, 2.0), (-20, 2.1), 0.0),        # 左右(正面の間)
            ((-10, 2.0), (10, 2.05), 0.0),       # 正面
            ((30, 2.0), (50, 2.1), 40.0),        # 左寄り
            ((-70, 2.0), (-30, 2.2), -50.0),     # 右寄り
        ]
        for a, b, expect in cases:
            sel = cb.FaceBearingSelector()
            out = sel.update(0.0, [lc(*a), lc(*b)])
            self.assertAlmostEqual(D(out), expect, places=3, msg=f"{a} {b}")

    def test_same_distance_pair_across_pi(self):
        sel = cb.FaceBearingSelector()
        out = D(sel.update(0.0, [lc(170, 2.0), lc(-170, 2.0)]))
        self.assertLess(adiff(out, 180.0), 1e-6)       # 真後ろ(0度や90度ではない)
        sel = cb.FaceBearingSelector()
        out = D(sel.update(0.0, [lc(175, 2.0), lc(-165, 2.1)]))
        self.assertLess(adiff(out, -175.0), 1e-6)
        sel = cb.FaceBearingSelector()
        out = D(sel.update(0.0, [lc(-175, 2.0), lc(165, 2.1)]))   # 順序を逆にしても同じ
        self.assertLess(adiff(out, 175.0), 1e-6)

    def test_same_distance_tolerance(self):
        # 0.25 m以内(近い方の10%の大きい方)は同じ距離
        sel = cb.FaceBearingSelector()
        self.assertAlmostEqual(D(sel.update(0.0, [lc(20, 2.0), lc(-20, 2.2)])), 0.0, places=3)
        # 2.0 と 2.4(差0.4 > 0.25): 同じ距離ではない → 近い方
        sel = cb.FaceBearingSelector()
        self.assertAlmostEqual(D(sel.update(0.0, [lc(20, 2.4), lc(-20, 2.0)])), -20.0, places=3)
        # 遠いと10%が効く: 6.0 と 6.5(差0.5 <= 0.6)は同じ距離
        sel = cb.FaceBearingSelector()
        self.assertAlmostEqual(D(sel.update(0.0, [lc(20, 6.5), lc(-20, 6.0)])), 0.0, places=3)

    def test_pair_too_far_apart_looks_at_one_deterministically(self):
        # 方位差160度(>90度)は間を向かない。同距離なら左(正)
        for order in ((lc(-80, 2.0), lc(80, 2.0)), (lc(80, 2.0), lc(-80, 2.0))):
            sel = cb.FaceBearingSelector()
            self.assertAlmostEqual(D(sel.update(0.0, list(order))), 80.0, places=3)
        # 少し差があれば近い方
        sel = cb.FaceBearingSelector()
        self.assertAlmostEqual(D(sel.update(0.0, [lc(80, 2.2), lc(-80, 2.0)])), -80.0, places=3)
        # 方位差ちょうど80度(<=90度)は間
        sel = cb.FaceBearingSelector()
        self.assertAlmostEqual(D(sel.update(0.0, [lc(-40, 2.0), lc(40, 2.0)])), 0.0, places=3)

    def test_three_at_same_distance_looks_at_middle_of_the_span(self):
        sel = cb.FaceBearingSelector()
        out = sel.update(0.0, [lc(-30, 2.0), lc(0, 2.0), lc(10, 2.1)])
        self.assertAlmostEqual(D(out), -10.0, places=3)     # 両端(-30と10)の真ん中
        # 開きが90度を超える3人目は入れない: -50,0,50 は両端100度 → 一番近い2人ではなく開き90度以内(左端から)
        sel = cb.FaceBearingSelector()
        out = D(sel.update(0.0, [lc(-50, 2.0), lc(0, 2.0), lc(50, 2.0)]))
        self.assertAlmostEqual(out, 25.0, places=3)         # anchor=左(50) と 0 の間

    def test_pair_is_not_used_when_a_nearer_person_exists(self):
        sel = cb.FaceBearingSelector()
        out = sel.update(0.0, [lc(20, 3.0), lc(-20, 3.0), lc(60, 1.5)])
        self.assertAlmostEqual(D(out), 60.0, places=3)


class StabilityTest(unittest.TestCase):
    @staticmethod
    def _jumps(outs, thr=20.0):
        return sum(1 for a, b in zip(outs, outs[1:]) if adiff(a, b) > thr)

    def test_many_people_with_noise_do_not_make_the_bearing_jump(self):
        for n in (3, 5, 10):
            rng = random.Random(7 + n)
            persons, expect = make_crowd(rng, n)
            sel = cb.FaceBearingSelector()
            outs = run(sel, lambda t: persons, 40.0, rng)
            self.assertEqual(self._jumps(outs), 0, f"n={n}")
            self.assertEqual(sel.anchor_switches, 0, f"n={n}")
            self.assertLess(max(outs) - min(outs), 12.0, f"n={n}")   # センサ雑音の範囲だけ

    def test_two_people_at_almost_the_same_distance_swap_but_do_not_make_it_jump(self):
        # 方位差100度(間を向かない)の2人が、距離のゆらぎで近さが入れ替わり続ける
        for seed in range(6):
            rng = random.Random(seed)
            persons = [(-50, 2.00), (50, 2.08)]
            sel = cb.FaceBearingSelector()
            outs = run(sel, lambda t: persons, 40.0, rng, hz=10.0)
            self.assertEqual(self._jumps(outs), 0, f"seed={seed}")
            self.assertEqual(sel.anchor_switches, 0, f"seed={seed}")

    def test_pair_in_range_stays_in_the_middle_with_noise(self):
        # 方位差60度で距離がほぼ同じ2人: 間(0度)を向き続ける。組の出入りで飛ばない
        for seed in range(6):
            rng = random.Random(100 + seed)
            persons = [(-30, 2.00), (30, 2.22)]     # 差0.22 は許容(0.25)ぎりぎりでゆらぐ
            sel = cb.FaceBearingSelector()
            outs = run(sel, lambda t: persons, 40.0, rng)
            # 組に入る瞬間(最初の数秒、距離の中央値が定まるまで)の1回は許す。その後は組の出入りで飛ばない(出る側に1.5倍のヒステリシス)
            self.assertEqual(self._jumps(outs[30:], thr=15.0), 0, f"seed={seed}")
            self.assertEqual(sel.anchor_switches, 0)

    def test_people_walking_around_far_away_do_not_pull_the_target(self):
        rng = random.Random(3)
        near = (10.0, 1.4)

        def persons(t):
            ps = [near]
            for k in range(8):
                ps.append((-70 + 20 * k + 15 * math.sin(0.4 * t + k), 3.0 + 1.5 * (0.5 + 0.5 * math.sin(0.3 * t + 2 * k))))
            return ps

        sel = cb.FaceBearingSelector()
        outs = run(sel, persons, 40.0, rng)
        self.assertEqual(sel.anchor_switches, 0)
        self.assertTrue(all(adiff(o, 10.0) < 8.0 for o in outs))


class SwitchingTest(unittest.TestCase):
    def _follow(self, other_range, secs):
        """今の人: -60度 3.0 m。別の人: +60度 other_range が secs 秒続く(方位差120度=距離が近くても間を向かない)。"""
        sel = cb.FaceBearingSelector()
        rng = random.Random(5)
        run(sel, lambda t: [(-60, 3.0)], 5.0, rng)
        outs = run(sel, lambda t: [(-60, 3.0), (60, other_range)], secs, rng, t0=5.0)
        return sel, outs

    def test_slightly_nearer_person_does_not_take_over(self):
        for r in (2.95, 2.85, 2.8):      # 差が0.05〜0.2 m(0.3 mの切替条件に雑音で届かない範囲)、1.3倍にも届かない
            sel, outs = self._follow(r, 30.0)
            self.assertEqual(sel.anchor_switches, 0, f"r={r}")
            self.assertTrue(all(adiff(o, -60) < 6 for o in outs), f"r={r}")

    def test_clearly_nearer_person_takes_over_only_after_the_hold(self):
        sel, outs = self._follow(2.3, 2.2)            # 0.7 m近い。まだ2.5秒たっていない
        self.assertTrue(all(adiff(o, -60) < 6 for o in outs))
        sel, outs = self._follow(2.3, 4.0)            # 4秒続いた → 替わる
        self.assertLess(adiff(outs[-1], 60), 6)
        self.assertEqual(sel.anchor_switches, 1)

    def test_ratio_condition(self):
        # 1.3倍以上近い(差は0.3 m未満でも): 近距離 0.9 m の人に対して 0.65 m(差0.25 m、1.38倍)
        sel = cb.FaceBearingSelector()
        rng = random.Random(2)
        a = [(-60, 0.9)]
        run(sel, lambda t: a, 3.0, rng)
        outs = run(sel, lambda t: a + [(60, 0.65)], 2.0, rng, t0=3.0)
        self.assertLess(adiff(outs[-1], -60), 6)       # まだ2.5秒たっていない
        outs = run(sel, lambda t: a + [(60, 0.65)], 4.0, rng, t0=5.0)
        self.assertLess(adiff(outs[-1], 60), 6)

    def test_short_visit_of_a_nearer_person_is_ignored(self):
        sel = cb.FaceBearingSelector()
        rng = random.Random(9)
        run(sel, lambda t: [(-40, 3.0)], 5.0, rng)
        outs = run(sel, lambda t: [(-40, 3.0), (40, 1.5)], 1.8, rng, t0=5.0)   # 1.8秒だけ通りすがり
        outs += run(sel, lambda t: [(-40, 3.0)], 5.0, rng, t0=6.8)
        self.assertEqual(sel.anchor_switches, 0)
        self.assertTrue(all(adiff(o, -40) < 6 for o in outs))

    def test_intermittent_nearer_person_resets_the_timer(self):
        sel = cb.FaceBearingSelector()
        rng = random.Random(11)
        run(sel, lambda t: [(-40, 3.0)], 5.0, rng)
        t = 5.0
        for _ in range(6):   # 1.5秒見える・1.5秒消える を繰り返す(各回2.5秒に届かない)
            run(sel, lambda t_: [(-40, 3.0), (40, 1.5)], 1.5, rng, t0=t)
            t += 1.5
            run(sel, lambda t_: [(-40, 3.0)], 1.5, rng, t0=t)
            t += 1.5
        self.assertEqual(sel.anchor_switches, 0)

    def test_lost_person_is_replaced_by_a_nearby_one_immediately(self):
        sel2 = cb.FaceBearingSelector()
        sel2.update(0.0, [lc(30, 2.0), lc(-70, 4.0)])
        for t in (0.1, 0.2, 0.3):
            sel2.update(t, [lc(30, 2.0), lc(-70, 4.0)])
        # 30度の人が消えて、1秒後に60度(30度差=45度以内)の人: すぐ付く
        b = sel2.update(1.5, [lc(60, 2.5), lc(-70, 4.0)])
        self.assertAlmostEqual(D(b), 60.0, places=3)

    def test_far_person_after_loss_waits_the_hold_time(self):
        sel = cb.FaceBearingSelector()
        sel.update(0.0, [lc(30, 1.0), lc(-80, 2.0)])
        sel.update(0.1, [lc(30, 1.0), lc(-80, 2.0)])
        # 30度の人が消えた。-80度は110度離れている → 追跡が切れても少し待つ(直前の方位を返す)
        b = sel.update(1.2, [lc(-80, 2.0)])
        self.assertAlmostEqual(D(b), 30.0, places=3)
        b = sel.update(1.2 + cb.NEAREST_HOLD_SEC * 0.5, [lc(-80, 2.0)])
        self.assertAlmostEqual(D(b), 30.0, places=3)
        b = sel.update(0.1 + cb.TRACK_KEEP_SEC + cb.NEAREST_HOLD_SEC + 0.3, [lc(-80, 2.0)])
        self.assertAlmostEqual(D(b), -80.0, places=3)

    def test_range_is_smoothed_by_median(self):
        # 1回だけ極端に近く測れた(外れ値)では切り替わる条件にならない
        sel = cb.FaceBearingSelector()
        for i in range(30):
            r2 = 0.5 if i == 15 else 3.0
            sel.update(i * 0.1, [lc(-40, 3.0), lc(40, r2)])
        self.assertEqual(sel.anchor_switches, 0)


class RangeUnknownTest(unittest.TestCase):
    def test_fuse_keeps_lidar_range_for_camera_confirmed_person(self):
        out = cb.fuse_candidates([{"bearing": R(10), "range": 2.3, "confirmed": False}], [{"bearing": R(14)}])
        self.assertEqual(len(out), 1)
        self.assertEqual((out[0]["source"], out[0]["range"]), ("both", 2.3))
        sel = cb.FaceBearingSelector()
        self.assertAlmostEqual(D(sel.update(0.0, out)), 10.0, places=3)

    def test_camera_only_next_to_a_lidar_person_inherits_its_range(self):
        # 同じ方位(8度以内)のカメラだけの候補は、その人の距離を引き継ぎ、別の人として数えない(間を向かない)
        sel = cb.FaceBearingSelector()
        b = sel.update(0.0, [lc(20, 2.0), cam(26)])
        self.assertAlmostEqual(D(b), 20.0, places=3)

    def test_camera_only_is_ignored_while_any_distance_is_known(self):
        sel = cb.FaceBearingSelector()
        b = sel.update(0.0, [cam(-60), lc(40, 4.0)])
        self.assertAlmostEqual(D(b), 40.0, places=3)

    def test_all_unknown_uses_the_weight_vote(self):
        sel = cb.FaceBearingSelector()
        b = sel.update(0.0, [cam(-60), cam(50), cam(52)])
        self.assertGreater(D(b), 40.0)

    def test_unknown_to_known_transition_is_safe(self):
        sel = cb.FaceBearingSelector()
        sel.update(0.0, [cam(30)])
        b = sel.update(0.1, [cam(30), lc(32, 2.0)])
        self.assertAlmostEqual(D(b), 32.0, places=3)

    def test_rejected_candidates_are_used_only_when_alone(self):
        weak = {"bearing": R(-30), "range": 1.0, "weight": cb.REJECTED_WEIGHT, "source": "lidar"}
        sel = cb.FaceBearingSelector()
        self.assertAlmostEqual(D(sel.update(0.0, [weak, lc(40, 3.0)])), 40.0, places=3)
        sel = cb.FaceBearingSelector()
        self.assertAlmostEqual(D(sel.update(0.0, [weak])), -30.0, places=3)


class RobustnessTest(unittest.TestCase):
    def test_bad_input_never_raises(self):
        sel = cb.FaceBearingSelector()
        bad = [None, "x", 5, {}, {"bearing": float("nan"), "weight": 1.0}, {"bearing": 0.1, "weight": float("nan")},
               {"bearing": 0.1, "weight": 1.0, "range": float("nan")}, {"bearing": 0.1, "weight": 1.0, "range": float("inf")},
               {"bearing": 0.1, "weight": 1.0, "range": -1.0}, {"bearing": 0.1, "weight": 1.0, "range": 0.0},
               {"bearing": True, "weight": 1.0}, {"bearing": 0.1, "weight": 1.0, "range": "2"}]
        for yaw in (None, float("nan"), 0.0, "a"):
            sel.reset()
            sel.update(0.0, bad, yaw=yaw)
            sel.update(float("nan"), [lc(10, 1.0)], yaw=yaw)
            self.assertIsNone(sel.update(1.0, [], yaw=yaw))
            self.assertIsNone(sel.update(1.0, None, yaw=yaw))
            self.assertIsNone(sel.update(1.0, "abc", yaw=yaw))

    def test_yaw_presence_change_is_safe(self):
        sel = cb.FaceBearingSelector()
        sel.update(0.0, [lc(10, 2.0)], yaw=1.0)
        b = sel.update(0.1, [lc(10, 2.0)], yaw=None)
        self.assertAlmostEqual(D(b), 10.0, places=3)

    def test_rotating_robot_keeps_the_same_world_target(self):
        # ロボットが旋回して機体基準の方位が動いても、世界の方位が同じ人を追い続ける(替わらない)
        sel = cb.FaceBearingSelector()
        world = [(50.0, 1.5), (-20.0, 3.0), (10.0, 2.5)]
        yaw = 0.0
        for i in range(120):
            yaw += R(0.5)
            ps = [(((w - D(yaw) + 180) % 360) - 180, r) for w, r in world]
            b = sel.update(i * 0.1, [lc(*p) for p in ps], yaw=yaw)
            self.assertLess(adiff(D(b) + D(yaw), 50.0), 0.5)
        self.assertEqual(sel.anchor_switches, 0)


# ------------------------------------------------------------------ 閉ループ(本物の behavior_director + face_person_sim)
try:
    import face_person_sim as fps  # noqa: E402
    _SIM_OK = fps.bd is not None
except Exception:   # pragma: no cover
    fps = None
    _SIM_OK = False


@unittest.skipUnless(_SIM_OK, "behavior_director / sim を読み込めない")
class ClosedLoopTest(unittest.TestCase):
    def _sim(self, persons_xy, seconds, seed=1):
        people = [fps.Person(x, y) for x, y in persons_xy]
        s = fps.Sim(people=people, seed=seed)
        s.run(seconds)
        self.addCleanup(s.close)
        return s

    @staticmethod
    def _xy(bearing_deg, r):
        return r * math.cos(R(bearing_deg)), r * math.sin(R(bearing_deg))

    def test_faces_the_nearest_for_1_2_5_10_people(self):
        for n in (1, 2, 5, 10):
            for seed in (1, 2):
                rng = random.Random(500 + 10 * n + seed)
                persons, expect = make_crowd(rng, n)
                s = self._sim([self._xy(b, r) for b, r in persons], 26.0, seed=seed)
                err = D(fps.wrap(R(expect) - s.robot["yaw"]))
                self.assertLessEqual(abs(err), 9.0, f"n={n} seed={seed} err={err:.1f}")
                self.assertLessEqual(s.reversals(), 8, f"n={n} seed={seed}")
                self.assertEqual(s.max_linear(), 0.0)

    def test_same_distance_pair_looks_between_in_the_loop(self):
        s = self._sim([self._xy(30, 2.5), self._xy(-20, 2.55)], 20.0)
        self.assertLessEqual(abs(D(s.robot["yaw"]) - 5.0), 9.0)

    def test_crowd_does_not_make_the_heading_wander(self):
        rng = random.Random(77)
        persons, expect = make_crowd(rng, 10)
        s = self._sim([self._xy(b, r) for b, r in persons], 45.0)
        late = [D(r["yaw"]) for r in s.log if r["t"] > 25.0]
        self.assertLess(max(late) - min(late), 6.0)          # 後半20秒は向きがほぼ動かない
        self.assertLessEqual(s.reversals(), 10)
        sel = s.n._face_selector
        self.assertLessEqual(sel.anchor_switches, 1)

    def test_two_equal_distance_people_far_apart_do_not_flip(self):
        # 方位差が90度を超える同距離の2人(以前の実機の入れ替わりに近い状況): 向く先が往復しない
        s = self._sim([self._xy(60, 2.5), self._xy(-60, 2.55)], 40.0)
        self.assertLessEqual(s.reversals(), 6)
        self.assertLessEqual(s.n._face_selector.anchor_switches, 1)


if __name__ == "__main__":
    unittest.main()
