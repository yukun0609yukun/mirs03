"""2026-11-06 カメラ検出の追跡による安定化(CameraTrackBank)と候補の階層化(TierFilter)。実機の記録の再生を含む。"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import face_camera_replay as fr  # noqa: E402
from nokosahen_core import crowd_bearing as cb  # noqa: E402
from nokosahen_core.camera_tracks import CameraTrackBank, TierFilter  # noqa: E402

D = math.radians


def bank(**kw):
    kw.setdefault("confirm_n", 2)
    return CameraTrackBank(**kw)


class TrackTests(unittest.TestCase):
    def test_single_frame_detection_is_ignored(self):
        b = bank()
        self.assertEqual(b.update(0.0, 0.0, [D(30)]), [])

    def test_confirm_after_second_distinct_detection(self):
        b = bank()
        b.update(0.0, 0.0, [D(30)])
        out = b.update(0.8, 0.0, [D(32)])
        self.assertEqual(len(out), 1)
        self.assertAlmostEqual(math.degrees(out[0]), 30 + 0.3 * 2, places=3)   # 平滑化(alpha=0.3)

    def test_burst_identical_messages_do_not_confirm(self):
        b = bank()
        for k in range(10):                     # 同じ内容が繰り返し届いても1回の観測
            out = b.update(0.2 * k, 0.0, [D(30)])
        self.assertEqual(out, [])
        self.assertEqual(b.observations, 1)

    def test_burst_during_rotation_keeps_world_bearing(self):
        b = bank()
        b.update(0.0, 0.0, [D(30)])
        b.update(0.8, 0.0, [D(30.5)])
        # 検出の内容は同じまま、機体が30度旋回: 世界方位は動かず、機体基準の方位だけ変わる
        out = b.update(1.0, D(30), [D(30.5)])
        self.assertAlmostEqual(math.degrees(out[0]), 0.35, delta=1.0)

    def test_hold_then_expire(self):
        b = bank(hold_sec=1.6)
        b.update(0.0, 0.0, [D(30)])
        b.update(0.8, 0.0, [D(31)])
        self.assertEqual(len(b.update(2.0, 0.0, [])), 1)       # 1.2秒: まだ保持
        self.assertEqual(b.update(3.0, 0.0, []), [])           # 2.2秒: 期限切れ

    def test_held_track_leaves_view_but_world_bearing_kept(self):
        b = bank()
        b.update(0.0, 0.0, [D(60)])
        b.update(0.8, 0.0, [D(61)])
        out = b.update(1.2, D(90), [])                          # 90度旋回して視野から出た
        self.assertAlmostEqual(math.degrees(out[0]), 60.3 - 90, delta=1.0)

    def test_match_tolerance(self):
        b = bank(match_deg=12)
        b.update(0.0, 0.0, [D(30)])
        b.update(0.8, 0.0, [D(50)])                             # 20度離れている: 別の対象(どちらも1回)
        self.assertEqual(b.update(1.6, 0.0, []), [])
        b.update(2.4, 0.0, [D(40)])                             # 50度の対象に近い(10度)=同じ対象
        self.assertEqual(len(b.update(3.2, 0.0, [D(41)])), 1)

    def test_wraparound_at_180(self):
        b = bank()
        b.update(0.0, D(170), [D(15)])        # 世界 185 = -175
        out = b.update(0.8, D(-175), [D(9)])  # 世界 -166: 11度差で同じ対象
        self.assertEqual(len(out), 1)
        world = math.degrees(out[0]) + (-175.0)
        self.assertAlmostEqual(((world + 180) % 360) - 180, -175.0 + 0.3 * 9, delta=0.5)

    def test_nan_none_empty(self):
        b = bank()
        self.assertEqual(b.update(0.0, 0.0, []), [])
        self.assertEqual(b.update(0.0, 0.0, None), [])
        self.assertEqual(b.update(0.1, 0.0, [float("nan"), None, "x", float("inf")]), [])
        self.assertAlmostEqual(b.update(float("nan"), 0.0, [D(10)])[0], D(10))
        self.assertAlmostEqual(b.update(0.2, None, [D(10)])[0], D(10))      # 位置なし: そのまま返す

    def test_missing_yaw_resets_tracks(self):
        b = bank()
        b.update(0.0, 0.0, [D(30)])
        b.update(0.8, 0.0, [D(31)])
        b.update(1.0, None, [])
        self.assertEqual(b.update(1.2, 0.0, []), [])

    def test_clock_going_back_resets(self):
        b = bank()
        b.update(100.0, 0.0, [D(30)])
        b.update(100.8, 0.0, [D(31)])
        self.assertEqual(b.update(1.0, 0.0, []), [])

    def test_confirm_n_one_passes_immediately(self):
        self.assertEqual(len(CameraTrackBank(confirm_n=1).update(0.0, 0.0, [D(30)])), 1)


def cand(src, bearing_deg, rng=None, w=1.0):
    return {"bearing": D(bearing_deg), "range": rng, "weight": w, "source": src}


class TierTests(unittest.TestCase):
    def test_tier1_hides_tier2(self):
        t = TierFilter()
        out = t.update(0.0, 0.0, [cand("lidar", -140, 0.8, 0.75), cand("both", -60, 1.7)])
        self.assertEqual([c["source"] for c in out], ["both"])

    def test_tier1_held_when_missing_then_tier2_after_delay(self):
        t = TierFilter(tier2_after_sec=4.0, hold_sec=2.0)
        t.update(0.0, 0.0, [cand("both", -60, 1.7)])
        lid = [cand("lidar", -140, 0.8, 0.75)]
        out = t.update(1.0, 0.0, lid)
        self.assertEqual([c["source"] for c in out], ["both"])     # 覚えている階層1
        self.assertEqual(t.update(3.0, 0.0, lid), [])               # 保持を過ぎたが、まだ階層2は使わない
        out = t.update(4.5, 0.0, lid)
        self.assertEqual([c["source"] for c in out], ["lidar"])
        self.assertTrue(t.tier2_active)

    def test_held_tier1_follows_world_bearing(self):
        t = TierFilter()
        t.update(0.0, 0.0, [cand("both", 60, 1.7)])
        out = t.update(0.5, D(90), [])
        self.assertAlmostEqual(math.degrees(out[0]["bearing"]), -30, delta=0.01)

    def test_no_tier1_ever_uses_tier2_immediately(self):
        t = TierFilter(tier2_after_sec=4.0)
        lid = [cand("lidar", 150, 1.0, 0.75)]
        self.assertEqual(len(t.update(0.0, 0.0, lid)), 1)

    def test_no_yaw_passthrough_and_bad_input(self):
        t = TierFilter()
        lid = [cand("lidar", 10, 1.0, 0.75)]
        self.assertEqual(t.update(0.0, None, lid), lid)
        self.assertEqual(t.update(0.0, 0.0, None), [])
        self.assertEqual(t.update(float("nan"), 0.0, lid), lid)

    def test_camera_only_track_is_tier1(self):
        t = TierFilter()
        out = t.update(0.0, 0.0, [cand("camera", 20, None, 0.25), cand("lidar", -140, 0.8, 0.75)])
        self.assertEqual([c["source"] for c in out], ["camera"])

    def test_static_object_memory_lowers_weight_in_tier2(self):
        t = TierFilter(tier2_after_sec=0.0)
        rej = cand("lidar", 30, 1.0, 0.1)
        for k in range(10):                                    # 2秒、カメラに否定され続けた
            t.update(0.2 * k, 0.0, [dict(rej)])
        out = t.update(2.2, D(120), [cand("lidar", -90, 1.0, 0.75)])   # 視野の外に回ったので否定は届かない。世界方位は同じ
        self.assertLessEqual(out[0]["weight"], 0.1 + 1e-9)


class ReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.recs = fr.load()

    def test_recording_loaded(self):
        self.assertEqual(len(self.recs), 244)

    def test_baseline_values(self):
        m = fr.metrics(fr.replay(self.recs))
        self.assertEqual(m["jumps"], 11)
        m = fr.metrics(fr.replay(fr.camera_only(self.recs)))
        self.assertEqual(m["jumps"], 10)

    def test_camera_only_jumps_greatly_reduced(self):
        base = fr.metrics(fr.replay(fr.camera_only(self.recs)))
        new = fr.metrics(fr.replay(fr.camera_only(self.recs), tracker=CameraTrackBank()))
        self.assertLessEqual(new["jumps"], 3)
        self.assertLess(new["jumps"], base["jumps"] / 2)
        self.assertGreater(new["longest_hold_s"], 2 * base["longest_hold_s"])
        self.assertLess(new["mean_diff_deg"], 0.6 * base["mean_diff_deg"])

    def test_full_recording_not_worse(self):
        base = fr.metrics(fr.replay(self.recs))
        new = fr.metrics(fr.replay(self.recs, tracker=CameraTrackBank(), tiers=TierFilter()))
        self.assertLessEqual(new["jumps"], base["jumps"] + 2)
        self.assertLess(new["none_ratio"], 0.03)

    def test_oscillation_reproduction(self):
        """10:40の実機観測: 近い静止物が複数+遠いカメラ確認済みの人。従来は往復(ジャンプ多数)、階層化では起きない。"""
        base, new = [], []
        for sd in range(6):
            recs = fr.synth_oscillation(sd)
            base.append(fr.metrics(fr.replay(recs, ref_yaw_deg=164)))
            new.append(fr.metrics(fr.replay(recs, tracker=CameraTrackBank(), tiers=TierFilter(), ref_yaw_deg=164)))
        self.assertGreaterEqual(sum(m["jumps"] for m in base) / 6, 8)
        self.assertLessEqual(max(m["jumps"] for m in new), 1)
        self.assertGreaterEqual(min(m["longest_hold_s"] for m in new), 10.0)
        self.assertLess(sum(m["none_ratio"] for m in new) / 6, 0.1)

    def test_statics_only_no_oscillation_with_memory(self):
        new = [fr.metrics(fr.replay(fr.synth_oscillation(sd, person_world=None), tracker=CameraTrackBank(),
                                    tiers=TierFilter(), ref_yaw_deg=164)) for sd in range(6)]
        self.assertLessEqual(max(m["jumps"] for m in new), 3)
        self.assertLessEqual(sum(m["jumps"] for m in new) / 6, 2.5)

    def test_person_behind_only_gets_faced_after_delay(self):
        # 最初は前に確認済みの人(階層1)がいて、居なくなる。後ろにLiDARだけの人(カメラの視野の外)が居る: 階層2で向く。ただし tier2_after_sec 遅れる
        recs = [{"t": round(0.2 * k, 3), "yaw": 0.0, "camera": [10.0] if k < 10 else [],
                 "lidar": [{"b": 10.0, "r": 1.5, "conf": True, "rej": False}] * (1 if k < 10 else 0)
                 + [{"b": 170.0, "r": 1.5, "conf": False, "rej": False}]} for k in range(80)]
        out = fr.replay(recs, tiers=TierFilter())
        gone = recs[10]["t"] - 0.2      # 最後に見えた時刻
        first = next(o["t"] for o in out if o["t"] > gone + 3.0 and o["world"] is not None and abs(o["world"]) > 90)
        self.assertGreaterEqual(first - gone, 3.9)
        self.assertLessEqual(first - gone, 4.8)
        self.assertAlmostEqual(abs(out[-1]["world"]), 170.0, delta=1.0)

    def test_reaction_time_limits(self):
        r = fr.reaction_times(CameraTrackBank)
        self.assertLessEqual(r["appear_mean"], 1.5)
        self.assertLessEqual(r["appear_max"], 3.5)
        self.assertLessEqual(r["swap_mean"], 4.5)
        self.assertLessEqual(r["swap_max"], 5.5)

    def test_selector_spec_constants_unchanged(self):
        self.assertEqual((cb.NEAREST_HOLD_SEC, cb.SWITCH_MARGIN_M, cb.SWITCH_RATIO, cb.FUSE_TOL_DEG), (2.5, 0.3, 1.3, 8))


if __name__ == "__main__":
    unittest.main()
