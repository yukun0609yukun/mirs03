"""ピント(鮮鋭度)の定量・UVC自動調整・確認画面APIの単体テスト(2026-10-10追加)。

実行: python scripts/tablet_ocr_reader/tests/test_camera_autotune.py
実カメラ・Coral・ブラウザは不要。UVCカメラはスタブ(FakeUvcCap)で、設定値(シャープネス・コントラスト
等)に応じて合成した画像を返すので、探索・選択・スキップ・元に戻す・保存・再適用を通しで確認できる。
"""
import json
import os
import socket
import sys
import tempfile
import threading
import time
import unittest
import unittest.mock as mock
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import ocr_camera_quality as cq  # noqa: E402
import tablet_ocr_reader as t  # noqa: E402
from synth_frames import camera_frame, degrade, mock_screen  # noqa: E402

BOX = (0.3, 0.1, 0.7, 0.9)
_BASE = camera_frame(mock_screen(), 640, 480, 0.8)
_BLUR_CACHE = {}


def _blurred(sigma: float):
    key = round(sigma, 1)
    if key not in _BLUR_CACHE:
        _BLUR_CACHE[key] = cv2.GaussianBlur(_BASE.astype(np.float32), (0, 0), max(0.3, key))
    return _BLUR_CACHE[key]


class FakeUvcCap:
    """UVCカメラのスタブ。propsは {キー: dict(lo, hi, val, mode)}。
    mode: "clamp"=範囲外はクランプ(uvcvideo) / "reject"=範囲外はset=False / "ignore"=setはTrueだが値が変わらない
    (効かない項目) / "absent"=get=-1・set=False(項目が無い)。画像は設定値から合成する。"""

    NAMES = {
        "brightness": cv2.CAP_PROP_BRIGHTNESS, "contrast": cv2.CAP_PROP_CONTRAST, "gamma": cv2.CAP_PROP_GAMMA,
        "sharpness": cv2.CAP_PROP_SHARPNESS, "backlight": cv2.CAP_PROP_BACKLIGHT,
        "wb_temperature": cv2.CAP_PROP_WB_TEMPERATURE, "auto_wb": cv2.CAP_PROP_AUTO_WB,
        "focus": cv2.CAP_PROP_FOCUS, "autofocus": cv2.CAP_PROP_AUTOFOCUS,
    }

    def __init__(self, props=None, noise=3.0, focus_peak=None):
        d = {
            "brightness": dict(lo=-64, hi=64, val=0, mode="clamp"),
            "contrast": dict(lo=0, hi=100, val=20, mode="clamp"),
            "gamma": dict(lo=72, hi=500, val=100, mode="ignore"),
            "sharpness": dict(lo=0, hi=6, val=1, mode="clamp"),
            "backlight": dict(lo=0, hi=2, val=0, mode="absent"),
            "wb_temperature": dict(lo=2800, hi=6500, val=4600, mode="clamp"),
            "auto_wb": dict(lo=0, hi=1, val=1, mode="clamp"),
            "focus": dict(lo=0, hi=255, val=0, mode="absent"),
            "autofocus": dict(lo=0, hi=1, val=1, mode="absent"),
        }
        for k, v in (props or {}).items():
            d[k] = dict(d.get(k, {}), **v)
        self.p = d
        self.noise = noise
        self.focus_peak = focus_peak  # 手動ピントの最良位置(None=ピントは画像に影響しない)
        self.by_id = {v: k for k, v in self.NAMES.items()}
        self.reads = 0
        self.set_log = []
        self.released = False

    def isOpened(self):
        return True

    def release(self):
        self.released = True

    def set(self, prop, value):
        key = self.by_id.get(prop)
        self.set_log.append((key, value))
        if key is None:
            return True
        c = self.p[key]
        if c["mode"] == "absent":
            return False
        if c["mode"] == "ignore":
            return True
        if c["mode"] == "reject":
            if not (c["lo"] <= value <= c["hi"]):
                return False
        c["val"] = float(min(c["hi"], max(c["lo"], value)))
        return True

    def get(self, prop):
        key = self.by_id.get(prop)
        if key is None:
            return 0.0
        c = self.p[key]
        return -1.0 if c["mode"] == "absent" else float(c["val"])

    def value(self, key):
        return self.p[key]["val"]

    def grab(self):
        return True

    def retrieve(self):
        return self.read()

    def read(self):
        self.reads += 1
        s = self.p["sharpness"]["val"]
        sigma = 3.0 - 0.4 * s
        if self.focus_peak is not None and self.p["focus"]["mode"] != "absent":
            sigma += abs(self.p["focus"]["val"] - self.focus_peak) / 40.0
        factor = 0.3 + 0.9 * self.p["contrast"]["val"] / 100.0
        shift = self.p["brightness"]["val"] * 0.5
        g = (_blurred(sigma) - 128.0) * factor + 128.0 + shift
        rng = np.random.default_rng(self.reads)
        g = g + rng.normal(0.0, self.noise + 1.0 * s, g.shape)
        return True, np.clip(g, 0, 255).astype(np.uint8)


def make_tuner(cap, **kw):
    def read_frame():
        ok, f = cap.read()
        return f if ok else None

    args = dict(settle_frames=1, measure_frames=2, box_provider=lambda: BOX)
    args.update(kw)
    return cq.CameraTuner(cv2, np, cap, read_frame, **args)


class FocusMetricTest(unittest.TestCase):
    def score(self, img):
        return cq.edge_sharpness(cv2, np, img, BOX)["score"]

    def test_monotonic_in_blur(self):
        scores = [self.score(degrade(_BASE, sigma=s)) for s in (0.0, 1.5, 2.5, 4.0)]
        self.assertEqual(scores, sorted(scores, reverse=True))
        self.assertGreater(scores[0], scores[-1] * 2.0)

    def test_contrast_invariant(self):
        a = self.score(_BASE)
        b = self.score(degrade(_BASE, contrast=0.4))
        self.assertAlmostEqual(a, b, delta=a * 0.15)

    def test_noise_does_not_inflate(self):
        a = self.score(degrade(_BASE, sigma=3.0))
        b = self.score(degrade(_BASE, sigma=3.0, noise=12.0))
        self.assertLess(b, a * 1.3)

    def test_flat_image_has_no_score(self):
        m = cq.edge_sharpness(cv2, np, np.full((480, 640, 3), 120, np.uint8), BOX)
        self.assertIsNone(m["score"])

    def test_resolution_normalized(self):
        big = camera_frame(mock_screen(), 1280, 960, 0.8)
        small = cv2.resize(big, (640, 480), interpolation=cv2.INTER_AREA)
        a, b = self.score(big), self.score(small)
        self.assertLess(abs(a - b) / a, 0.5)  # 解像度が半分でも同じ程度のスコア(幅1280基準へ正規化)

    def test_center_region_when_no_box(self):
        self.assertIsNotNone(cq.edge_sharpness(cv2, np, _BASE, None)["ratio"])

    def test_tiny_region_returns_none(self):
        self.assertIsNone(cq.edge_sharpness(cv2, np, _BASE, (0.5, 0.5, 0.51, 0.51)))

    def test_grades(self):
        self.assertEqual(cq.focus_grade(80), "good")
        self.assertEqual(cq.focus_grade(50), "fair")
        self.assertEqual(cq.focus_grade(10), "bad")
        self.assertEqual(cq.focus_grade(None), "unknown")
        self.assertEqual(cq.focus_grade(50, {"good": 45, "fair": 20}), "good")

    def test_real_blur_maps_to_grades(self):
        self.assertEqual(cq.focus_grade(self.score(_BASE)), "good")
        self.assertEqual(cq.focus_grade(self.score(degrade(_BASE, sigma=4.0))), "bad")

    def test_combined_quality_prefers_sharp_and_separated(self):
        def q(img):
            return cq.combined_quality(cq.edge_sharpness(cv2, np, img, BOX))

        self.assertGreater(q(_BASE), q(degrade(_BASE, sigma=3.0)))
        self.assertGreater(q(_BASE), q(degrade(_BASE, contrast=0.3)))
        self.assertEqual(cq.combined_quality({"ratio": None, "sep": None}), 0.0)

    def test_combined_quality_penalizes_blown_out(self):
        m = {"ratio": 0.2, "sep": 5.0, "clip_hi": 0.2, "clip_lo": 0.0}
        blown = dict(m, clip_hi=0.95)
        self.assertLess(cq.combined_quality(blown), cq.combined_quality(m))


class FocusMonitorTest(unittest.TestCase):
    def make(self, **kw):
        self.now = [0.0]
        return cq.FocusMonitor(window=3, clock=lambda: self.now[0], **kw)

    def test_moving_average_peak_and_reset(self):
        mon = self.make()
        for s in (0.0, 1.0, 2.5):
            mon.update(cv2, np, degrade(_BASE, sigma=s), BOX)
        snap = mon.snapshot()
        self.assertIsNotNone(snap["avg"])
        self.assertGreaterEqual(snap["peak"], snap["avg"])
        self.assertEqual(snap["resolution"], {"width": 640, "height": 480})
        mon.reset_peak()
        self.assertIsNone(mon.snapshot()["peak"])

    def test_grade_and_guidance_after_bad_for_a_while(self):
        mon = self.make(warn_after_sec=20.0)
        blurred = degrade(_BASE, sigma=4.5)
        mon.update(cv2, np, blurred, BOX)
        self.assertEqual(mon.snapshot()["grade"], "bad")
        self.assertIsNone(mon.snapshot()["advice"])  # すぐには出さない
        self.now[0] = 25.0
        mon.update(cv2, np, blurred, BOX)
        snap = mon.snapshot()
        self.assertIn("ピントが合っていません", snap["advice"])
        self.assertIn("離す/近づけ", snap["advice"])
        self.assertIn("20〜40cm", snap["advice"])
        # 良くなったらガイダンスは消える
        mon.update(cv2, np, _BASE, BOX)
        for _ in range(3):
            mon.update(cv2, np, _BASE, BOX)
        self.assertIsNone(mon.snapshot()["advice"])

    def test_suspended_does_not_count_bad_time(self):
        mon = self.make(warn_after_sec=5.0)
        mon.set_suspended(True)
        mon.update(cv2, np, degrade(_BASE, sigma=4.5), BOX)
        self.now[0] = 30.0
        mon.update(cv2, np, degrade(_BASE, sigma=4.5), BOX)
        self.assertIsNone(mon.snapshot()["advice"])

    def test_near_peak_flag(self):
        mon = self.make()
        for _ in range(10):
            mon.update(cv2, np, _BASE, BOX)
        self.assertTrue(mon.snapshot()["near_peak"])
        for _ in range(3):
            mon.update(cv2, np, degrade(_BASE, sigma=3.0), BOX)
        self.assertFalse(mon.snapshot()["near_peak"])

    def test_due_throttle(self):
        mon = self.make()
        self.assertTrue(mon.due(0.3))
        mon.update(cv2, np, _BASE, BOX)
        self.assertFalse(mon.due(0.3))
        self.now[0] = 0.5
        self.assertTrue(mon.due(0.3))

    def test_startup_report_once(self):
        mon = self.make()
        for _ in range(5):
            mon.update(cv2, np, _BASE, BOX)
        self.assertTrue(mon.startup_report_due(5))
        self.assertFalse(mon.startup_report_due(5))


class BrightnessWarningTest(unittest.TestCase):
    def warn(self, img):
        return cq.brightness_warnings(cq.edge_sharpness(cv2, np, img, BOX))

    def test_normal_bright_screen_has_no_warning(self):
        self.assertEqual(self.warn(_BASE), [])  # 白っぽい画面は正常

    def test_dark(self):
        w = self.warn(np.clip(_BASE.astype(np.int16) * 0.2, 0, 255).astype(np.uint8))
        self.assertTrue(any("暗" in x for x in w))

    def test_blown_out(self):
        w = self.warn(np.clip(_BASE.astype(np.int16) + 120, 0, 255).astype(np.uint8))
        self.assertTrue(any("白飛び" in x or "明るすぎ" in x for x in w))

    def test_low_contrast(self):
        w = self.warn(degrade(_BASE, contrast=0.2))
        self.assertTrue(any("コントラスト" in x for x in w))

    def test_diagnose_report(self):
        d = cq.diagnose(cv2, np, degrade(_BASE, sigma=4.5), None, BOX)
        self.assertEqual(d["resolution"], {"width": 640, "height": 480})
        self.assertEqual(d["focus"]["grade"], "bad")
        self.assertIn("離す/近づけ", d["advice"])
        self.assertIn("mean", d["brightness"])
        self.assertFalse(cq.diagnose(cv2, np, None, None)["ok"])
        self.assertIsNone(cq.diagnose(cv2, np, _BASE, None, BOX)["advice"])


class ProbeRangeTest(unittest.TestCase):
    def test_clamp_and_reject_modes_find_range(self):
        for mode in ("clamp", "reject"):
            with self.subTest(mode=mode):
                cap = FakeUvcCap({"contrast": dict(lo=0, hi=100, val=20, mode=mode)})
                r = cq.probe_range(cv2, cap, "contrast")
                self.assertEqual((r["lo"], r["hi"], r["cur"]), (0, 100, 20))
                self.assertEqual(cap.value("contrast"), 20)  # 元の値へ戻る

    def test_negative_range(self):
        cap = FakeUvcCap()
        r = cq.probe_range(cv2, cap, "brightness")
        self.assertEqual((r["lo"], r["hi"]), (-64, 64))

    def test_value_already_at_max(self):
        cap = FakeUvcCap({"contrast": dict(lo=0, hi=130, val=130, mode="clamp")})
        r = cq.probe_range(cv2, cap, "contrast")
        self.assertEqual((r["lo"], r["hi"]), (0, 130))

    def test_unsupported_returns_none(self):
        cap = FakeUvcCap()
        self.assertIsNone(cq.probe_range(cv2, cap, "backlight"))  # get=-1
        self.assertIsNone(cq.probe_range(cv2, cap, "gamma"))  # setは通るが読み戻しが変わらない

    def test_candidates(self):
        self.assertEqual(cq.candidate_values(0, 100, 20, 5), [0.0, 20.0, 25.0, 50.0, 75.0, 100.0])
        self.assertEqual(cq.candidate_values(0, 6, 1, 5), [0.0, 1.0, 2.0, 3.0, 4.0, 6.0])
        small = cq.candidate_values(0, 1, 0.5, 5)
        self.assertTrue(all(0 <= v <= 1 for v in small))


class TunerTest(unittest.TestCase):
    def test_improves_and_reports_changes_and_skips(self):
        cap = FakeUvcCap()
        res = make_tuner(cap).run()
        self.assertEqual(res["status"], "improved", res)
        self.assertTrue(res["ok"])
        self.assertGreater(res["final"]["value"], res["baseline"]["value"] * 1.1)
        # 悪い初期値(sharpness=1, contrast=20)が良くなっている
        self.assertGreater(cap.value("sharpness"), 1)
        self.assertGreater(cap.value("contrast"), 20)
        self.assertEqual(res["changes"]["sharpness"]["before"], 1)
        self.assertEqual(res["changes"]["sharpness"]["after"], cap.value("sharpness"))
        self.assertIn("コントラスト", res["changes"]["contrast"]["label"])
        # 効かない項目・無い項目はスキップ(理由付き)
        self.assertIn("gamma", res["skipped"])
        self.assertIn("backlight", res["skipped"])
        self.assertIn("focus", res["skipped"])  # 固定焦点
        self.assertEqual(cap.value("gamma"), 100)
        self.assertEqual(res["applied"]["contrast"], cap.value("contrast"))
        self.assertEqual(res["original"]["contrast"], 20)
        self.assertGreater(res["evals"], 10)

    def test_unneeded_items_stay_and_auto_wb_restored(self):
        cap = FakeUvcCap()
        res = make_tuner(cap).run()
        # WB色温度は画像に影響しない合成なので変更されず、切ったオートWBは元へ戻る
        self.assertEqual(cap.value("wb_temperature"), 4600)
        self.assertEqual(cap.value("auto_wb"), 1)
        self.assertNotIn("wb_temperature", res["changes"])
        self.assertNotIn("auto_wb", res["original"])

    def test_wb_applied_when_it_helps(self):
        class WbCap(FakeUvcCap):
            def read(self):
                ok, f = super().read()
                # 色温度が高いほどコントラストが上がる合成(効く項目)
                k = (self.p["wb_temperature"]["val"] - 2800) / 3700.0
                g = (f.astype(np.float32) - 128.0) * (0.6 + 0.5 * k) + 128.0
                return ok, np.clip(g, 0, 255).astype(np.uint8)

        cap = WbCap({"contrast": dict(val=100), "sharpness": dict(val=6)})
        res = make_tuner(cap).run()
        if "wb_temperature" in res["changes"]:
            self.assertEqual(cap.value("auto_wb"), 0)  # 色温度を触ったのでオートWBは切れている
            self.assertEqual(res["original"]["auto_wb"], 1)  # 元へ戻せるよう記録

    def test_revert_restores_original_values(self):
        cap = FakeUvcCap()
        res = make_tuner(cap).run()
        cq.apply_values(cv2, cap, res["original"])
        self.assertEqual(cap.value("sharpness"), 1)
        self.assertEqual(cap.value("contrast"), 20)

    def test_no_improvement_restores_everything(self):
        # すでに最良に近い設定(コントラスト・シャープネスとも上限付近): 変えても改善しない
        cap = FakeUvcCap({"contrast": dict(val=85), "sharpness": dict(val=4), "wb_temperature": dict(mode="absent")})
        before = {k: cap.value(k) for k in ("contrast", "sharpness", "brightness")}
        res = make_tuner(cap).run()
        if res["status"] == "unchanged":
            self.assertEqual(res["changes"], {})
            self.assertEqual({k: cap.value(k) for k in before}, before)
        else:
            self.assertGreaterEqual(res["final"]["value"], res["baseline"]["value"])

    def test_flat_scene_is_rejected_without_changes(self):
        class FlatCap(FakeUvcCap):
            def read(self):
                return True, np.full((480, 640, 3), 100, np.uint8)

        cap = FlatCap()
        res = make_tuner(cap).run()
        self.assertFalse(res["ok"] and res["status"] == "improved")
        self.assertEqual(res["changes"], {})
        self.assertEqual(cap.value("contrast"), 20)

    def test_cancel_restores_and_reports(self):
        cap = FakeUvcCap()
        calls = [0]

        def cancelled():
            calls[0] += 1
            return calls[0] > 3

        res = make_tuner(cap, cancelled=cancelled).run()
        self.assertEqual(res["status"], "cancelled")
        self.assertEqual((cap.value("sharpness"), cap.value("contrast")), (1, 20))

    def test_exception_restores_and_reports_failed(self):
        cap = FakeUvcCap()
        n = [0]

        def read_frame():
            n[0] += 1
            if n[0] > 60:
                raise RuntimeError("usb unplugged")
            ok, f = cap.read()
            return f

        res = cq.CameraTuner(cv2, np, cap, read_frame, box_provider=lambda: BOX, settle_frames=1, measure_frames=2).run()
        self.assertEqual(res["status"], "failed")
        self.assertIn("usb unplugged", res["message"])
        self.assertEqual((cap.value("sharpness"), cap.value("contrast")), (1, 20))

    def test_time_limit_keeps_partial_result(self):
        cap = FakeUvcCap()
        t0 = [0.0]

        def clock():
            t0[0] += 1.0
            return t0[0]

        res = make_tuner(cap, clock=clock, max_seconds=40.0).run()
        self.assertIn(res["status"], ("improved", "unchanged"))
        self.assertLess(res["evals"], 40)

    def test_progress_reported(self):
        cap = FakeUvcCap()
        seen = []
        make_tuner(cap, progress=lambda d, tot, label: seen.append((d, tot, label))).run()
        self.assertGreater(len(seen), 3)
        self.assertEqual(seen[-1][2], "完了")
        self.assertEqual(seen[-1][0], seen[-1][1])
        self.assertTrue(all(0 <= d <= tot for d, tot, _ in seen))

    def test_focus_control_tuned_when_available(self):
        cap = FakeUvcCap({
            "focus": dict(lo=0, hi=255, val=0, mode="clamp"), "autofocus": dict(val=1, mode="clamp"),
        }, focus_peak=120)
        res = make_tuner(cap).run()
        self.assertTrue(res["focus_control"]["focus_settable"])
        self.assertIn("focus", res["changes"], res)
        self.assertLess(abs(cap.value("focus") - 120), 70)
        self.assertEqual(cap.value("autofocus"), 0)  # 手動ピントを使うのでAFは切れる
        self.assertEqual(res["original"]["autofocus"], 1)

    def test_focus_unavailable_is_reported_and_continues(self):
        cap = FakeUvcCap()
        res = make_tuner(cap).run()
        self.assertFalse(res["focus_control"]["focus_settable"])
        self.assertIn("固定焦点", res["skipped"]["focus"])

    def test_values_are_integers_for_wide_ranges(self):
        cap = FakeUvcCap()
        res = make_tuner(cap).run()
        for k, v in res["applied"].items():
            self.assertEqual(v, round(v), k)


class SavedTuningTest(unittest.TestCase):
    def test_validate_and_resolve(self):
        good = {"applied": {"sharpness": 4, "contrast": 60.0}, "original": {"sharpness": 1}, "tuned_at": "x"}
        self.assertEqual(cq.validate_tuning_record(good)["applied"], {"sharpness": 4.0, "contrast": 60.0})
        for bad in (None, 5, {}, {"applied": {}}, {"applied": {"evil": 1}}, {"applied": {"sharpness": "x"}},
                    {"applied": {"sharpness": True}}, {"applied": {"sharpness": float("nan")}}):
            self.assertIsNone(cq.validate_tuning_record(bad), bad)
        with self.assertLogs("tablet_ocr_reader", level="WARNING"):
            out = cq.resolve_camera_tuning({"demo": good, "weird": good, "production": {"applied": {}}})
        self.assertEqual(set(out), {"demo"})
        self.assertEqual(cq.resolve_camera_tuning(None), {})

    def test_reapply_for_profile_and_restore_for_other_profile(self):
        cap = FakeUvcCap()
        tuning = cq.resolve_camera_tuning({
            "production": {"applied": {"sharpness": 5, "contrast": 70}, "original": {"sharpness": 1, "contrast": 20}},
        })
        r = cq.reapply_tuning(cv2, cap, tuning, "production")
        self.assertEqual(r["mode"], "applied")
        self.assertEqual((cap.value("sharpness"), cap.value("contrast")), (5, 70))
        # 調整の無いdemoへ切替: UVCに残っている調整値を「元の値」へ戻す
        r = cq.reapply_tuning(cv2, cap, tuning, "demo")
        self.assertEqual(r["mode"], "restored")
        self.assertEqual((cap.value("sharpness"), cap.value("contrast")), (1, 20))
        self.assertEqual(cq.reapply_tuning(cv2, cap, {}, "demo")["mode"], "none")

    def test_reapply_never_raises(self):
        class Boom:
            def set(self, *a):
                raise OSError("x")

            get = set

        tuning = cq.resolve_camera_tuning({"demo": {"applied": {"sharpness": 3}}})
        self.assertIn(cq.reapply_tuning(cv2, Boom(), tuning, "demo")["mode"], ("applied", "none"))

    def test_auto_wb_applied_before_temperature(self):
        cap = FakeUvcCap()
        cq.apply_values(cv2, cap, {"wb_temperature": 5000, "auto_wb": 0})
        keys = [k for k, _ in cap.set_log]
        self.assertLess(keys.index("auto_wb"), keys.index("wb_temperature"))

    def test_controller_reapplies_on_open_and_switch(self):
        caps = []

        def opener(cv2_, idx, params):
            cap = FakeUvcCap()
            caps.append(cap)
            return cap, {"width": params["width"], "height": params["height"], "fps": None, "fourcc": "MJPG",
                         "sharpness": None}

        ctl = t.CameraController(cv2, "auto", opener)
        ctl.tuning = cq.resolve_camera_tuning({
            "production": {"applied": {"sharpness": 5, "contrast": 70}, "original": {"sharpness": 1, "contrast": 20}},
        })
        ctl.tuning_profile = "production"
        info = ctl.open(dict(t.DEFAULT_CAMERA_PROFILES["production"]))
        self.assertEqual(info["tuning"], "applied")
        self.assertEqual((caps[0].value("sharpness"), caps[0].value("contrast")), (5, 70))
        self.assertTrue(ctl.switch(dict(t.DEFAULT_CAMERA_PROFILES["demo"]), tuning_profile="demo"))
        self.assertEqual(ctl.info["tuning"], "restored")
        self.assertEqual(ctl.tuning_profile, "demo")
        self.assertTrue(ctl.switch(dict(t.DEFAULT_CAMERA_PROFILES["production"]), tuning_profile="production"))
        self.assertEqual(caps[-1].value("contrast"), 70)

    def test_controller_without_tuning_is_untouched(self):
        cap = FakeUvcCap()
        ctl = t.CameraController(cv2, "auto", lambda *a: (cap, {"width": 800, "height": 600}))
        ctl.open(dict(t.DEFAULT_CAMERA_PROFILES["demo"]))
        self.assertEqual(cap.set_log, [])

    def test_failed_switch_restores_tuning_profile(self):
        calls = [0]

        def opener(cv2_, idx, params):
            calls[0] += 1
            if calls[0] == 2:
                raise RuntimeError("cannot open")
            return FakeUvcCap(), {"width": 800, "height": 600}

        ctl = t.CameraController(cv2, "auto", opener)
        ctl.tuning_profile = "demo"
        ctl.open(dict(t.DEFAULT_CAMERA_PROFILES["demo"]))
        self.assertFalse(ctl.switch(dict(t.DEFAULT_CAMERA_PROFILES["production"]), tuning_profile="production"))
        self.assertEqual(ctl.tuning_profile, "demo")


def _state(profile="production"):
    st = t.PreviewState()
    st.init_camera(profile, t.resolve_camera_profiles({}), {"width": 640, "height": 480, "fps": 12.0, "fourcc": "MJPG"})
    return st


class PreviewStateAutotuneTest(unittest.TestCase):
    def test_request_lifecycle(self):
        st = _state()
        self.assertFalse(st.autotune_busy())
        self.assertEqual(st.request_autotune("start"), (True, ""))
        self.assertTrue(st.autotune_busy())
        self.assertEqual(st.snapshot_settings()["autotune"]["state"], "requested")
        ok, why = st.request_autotune("start")
        self.assertFalse(ok)
        self.assertIn("実行中", why)
        self.assertEqual(st.take_autotune_request(), "start")
        self.assertIsNone(st.take_autotune_request())
        self.assertEqual(st.snapshot_autotune()["state"], "running")
        st.set_autotune_progress(3, 10, "コントラストを調整中")
        self.assertEqual(st.snapshot_autotune()["progress"], {"done": 3, "total": 10, "label": "コントラストを調整中"})
        st.finish_autotune("start", {"status": "improved"}, "done", "ok")
        self.assertFalse(st.autotune_busy())
        self.assertEqual(st.snapshot_autotune()["result"], {"status": "improved"})
        self.assertTrue(st.request_autotune("revert")[0])

    def test_cancel(self):
        st = _state()
        self.assertFalse(st.request_autotune("cancel")[0])
        st.request_autotune("start")
        self.assertTrue(st.request_autotune("cancel")[0])  # 開始前なら取り消し
        self.assertFalse(st.autotune_busy())
        self.assertIsNone(st.take_autotune_request())
        st.request_autotune("start")
        st.take_autotune_request()
        self.assertFalse(st.autotune_cancelled())
        st.request_autotune("cancel")
        self.assertTrue(st.autotune_cancelled())
        st.finish_autotune("start", None, "cancelled", "x")
        self.assertFalse(st.autotune_cancelled())

    def test_refused_while_profile_switching(self):
        st = _state("demo")
        st.request_camera_profile("production")
        self.assertFalse(st.request_autotune("start")[0])

    def test_saved_info_in_snapshot(self):
        st = _state("production")
        st.set_tuning_saved({"production": {"applied": {"sharpness": 4.0}, "original": {"sharpness": 1.0}, "tuned_at": "t"}})
        saved = st.snapshot_settings()["autotune"]["saved"]
        self.assertEqual(saved["applied"], {"sharpness": 4.0})
        self.assertEqual(saved["profile"], "production")
        st2 = _state("demo")
        st2.set_tuning_saved({"production": {"applied": {"sharpness": 4.0}}})
        self.assertIsNone(st2.snapshot_settings()["autotune"]["saved"])

    def test_reset_history_on_rotation_and_profile_switch(self):
        st = _state("demo")
        st.history.push(_BASE)
        self.assertEqual(len(st.history), 1)
        st.set_rotate_deg(90)
        self.assertEqual(len(st.history), 0)
        st.history.push(_BASE)
        st.request_camera_profile("production")
        st.finish_camera_switch("production", {"width": 640, "height": 480})
        self.assertEqual(len(st.history), 0)

    def test_focus_in_readings_snapshot(self):
        st = _state()
        st.focus.update(cv2, np, _BASE, BOX)
        snap = st.snapshot_readings()
        self.assertIn("focus", snap)
        self.assertEqual(snap["autotune_state"], "idle")
        self.assertIsNotNone(snap["focus"]["avg"])


class RunAutotuneRequestTest(unittest.TestCase):
    def setUp(self):
        self.cap = FakeUvcCap()
        self.st = _state("production")
        self.ctl = t.CameraController(cv2, "auto", lambda *a: (self.cap, {"width": 640, "height": 480}))
        self.ctl.open(dict(t.DEFAULT_CAMERA_PROFILES["production"]))
        self.ctl.tuning_profile = "production"
        self.saved = []
        patcher = mock.patch.object(
            cq.CameraTuner, "__init__", autospec=True,
            side_effect=lambda self_, *a, **k: self._orig_init(self_, *a, **dict(k, settle_frames=1, measure_frames=2)),
        )
        self._orig_init = cq.CameraTuner.__init__
        patcher.start()
        self.addCleanup(patcher.stop)

    def run_action(self, action):
        self.assertTrue(self.st.request_autotune(action)[0])
        return t.run_autotune_request(self.st, self.ctl, cv2, lambda d: self.saved.append(json.loads(json.dumps(d))))

    def test_start_saves_tuning_and_updates_state(self):
        self.assertTrue(self.run_action("start"))
        snap = self.st.snapshot_autotune()
        self.assertEqual(snap["state"], "done", snap)
        self.assertEqual(snap["result"]["status"], "improved")
        self.assertIn("production", self.ctl.tuning)
        self.assertEqual(self.ctl.tuning["production"]["applied"]["sharpness"], self.cap.value("sharpness"))
        self.assertEqual(self.ctl.tuning["production"]["original"]["sharpness"], 1)
        self.assertEqual(len(self.saved), 1)
        self.assertEqual(self.saved[0]["production"]["applied"]["contrast"], self.cap.value("contrast"))
        self.assertIsNotNone(snap["saved"])
        self.assertIsNotNone(self.st.get_raw_frame())  # 調整中の最新フレームが共有されている
        self.assertEqual(len(self.st.history), 0)  # 古い設定のフレームは捨てる

    def test_revert_restores_and_clears_saved(self):
        self.run_action("start")
        self.assertTrue(self.run_action("revert"))
        snap = self.st.snapshot_autotune()
        self.assertEqual(snap["state"], "reverted", snap)
        self.assertEqual((self.cap.value("sharpness"), self.cap.value("contrast")), (1, 20))
        self.assertNotIn("production", self.ctl.tuning)
        self.assertEqual(self.saved[-1], {})  # 設定ファイルのcamera_tuningは空(None保存)になる
        self.assertIsNone(snap["saved"])

    def test_second_tune_keeps_first_original(self):
        self.run_action("start")
        first_applied = dict(self.ctl.tuning["production"]["applied"])
        self.run_action("start")
        rec = self.ctl.tuning["production"]
        self.assertEqual(rec["original"]["sharpness"], 1)  # 一番最初の値
        self.assertTrue(set(first_applied) <= set(rec["applied"]))
        self.run_action("revert")
        self.assertEqual(self.cap.value("sharpness"), 1)

    def test_revert_without_saved_fails_gracefully(self):
        self.run_action("revert")
        snap = self.st.snapshot_autotune()
        self.assertEqual(snap["state"], "failed")
        self.assertIn("元に戻す", snap["message"])

    def test_exception_does_not_escape_and_state_is_failed(self):
        with mock.patch.object(cq.CameraTuner, "run", side_effect=RuntimeError("boom")):
            self.assertTrue(self.run_action("start"))
        snap = self.st.snapshot_autotune()
        self.assertEqual(snap["state"], "failed")
        self.assertIn("boom", snap["message"])
        self.assertFalse(self.st.autotune_busy())

    def test_persist_failure_does_not_escape(self):
        def bad(d):
            raise OSError("disk full")

        self.assertTrue(self.st.request_autotune("start")[0])
        t.run_autotune_request(self.st, self.ctl, cv2, bad)
        self.assertEqual(self.st.snapshot_autotune()["state"], "failed")
        self.assertFalse(self.st.autotune_busy())

    def test_no_camera(self):
        self.ctl.release()
        self.assertTrue(self.st.request_autotune("start")[0])
        t.run_autotune_request(self.st, self.ctl, cv2, None)
        self.assertEqual(self.st.snapshot_autotune()["state"], "failed")

    def test_no_request_is_noop(self):
        self.assertFalse(t.run_autotune_request(self.st, self.ctl, cv2, None))

    def test_focus_monitor_resumes_after_run(self):
        self.run_action("start")
        self.assertFalse(self.st.focus._suspended)


class ApiTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.cfg = os.path.join(self.dir, "cfg.json")
        with open(self.cfg, "w", encoding="utf-8") as f:
            json.dump({"auth_password": "", "camera_index": "auto"}, f)
        self.state = _state("production")
        self.server = t.start_preview_server(self.state, "127.0.0.1", 0, self.cfg)
        self.base = f"http://127.0.0.1:{self.server.server_address[1]}"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def req(self, path, body=None, method=None, raw=None):
        data = raw if raw is not None else (None if body is None else json.dumps(body).encode())
        r = urllib.request.Request(self.base + path, data=data, method=method or ("POST" if data is not None else "GET"),
                                   headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(r, timeout=5) as resp:
                return resp.status, json.load(resp)
        except urllib.error.HTTPError as e:
            try:
                return e.code, json.load(e)
            except ValueError:
                return e.code, None
            finally:
                e.close()

    def test_post_autotune_default_and_explicit_start(self):
        code, body = self.req("/api/autotune", {})
        self.assertEqual(code, 200)
        self.assertTrue(body["ok"])
        self.assertEqual(body["autotune"]["state"], "requested")
        self.assertEqual(self.state.take_autotune_request(), "start")

    def test_busy_is_409(self):
        self.assertEqual(self.req("/api/autotune", {"action": "start"})[0], 200)
        code, body = self.req("/api/autotune", {"action": "revert"})
        self.assertEqual(code, 409)
        self.assertIn("実行中", body["error"])

    def test_invalid_actions_rejected(self):
        for bad in ({"action": "reboot"}, {"action": 1}, {"action": None}, {"action": ""}, {"action": ["start"]},
                    {"action": "START"}):
            with self.subTest(bad=bad):
                self.assertEqual(self.req("/api/autotune", bad)[0], 400)
        self.assertEqual(self.req("/api/autotune", raw=b"not json")[0], 400)
        self.assertEqual(self.req("/api/autotune", raw=b"[1]")[0], 400)
        self.assertFalse(self.state.autotune_busy())

    def test_cancel_without_run_is_409(self):
        self.assertEqual(self.req("/api/autotune", {"action": "cancel"})[0], 409)

    def test_settings_json_has_autotune(self):
        code, d = self.req("/api/settings.json")
        self.assertEqual(code, 200)
        at = d["autotune"]
        self.assertEqual(at["state"], "idle")
        self.assertTrue(at["available"])
        self.assertIn("progress", at)
        self.assertIn("saved", at)

    def test_settings_post_unaffected(self):
        code, d = self.req("/api/settings", {"ocr_scale_factor": 2.5})
        self.assertEqual(code, 200)
        self.assertIn("autotune", d)

    def test_focus_json_and_reset_peak(self):
        self.state.focus.update(cv2, np, _BASE, BOX)
        code, d = self.req("/api/focus.json")
        self.assertEqual(code, 200)
        self.assertTrue(d["available"])
        self.assertEqual(d["grade"], "good")
        self.assertEqual(d["grade_label"], "良好")
        self.assertIsNotNone(d["peak"])
        self.assertEqual(self.req("/api/focus", {"action": "reset_peak"})[0], 200)
        self.assertIsNone(self.req("/api/focus.json")[1]["peak"])
        self.assertEqual(self.req("/api/focus", {"action": "nope"})[0], 400)

    def test_readings_json_has_focus(self):
        self.state.focus.update(cv2, np, _BASE, BOX)
        d = self.req("/api/readings.json")[1]
        self.assertIn("focus", d)
        self.assertEqual(d["autotune_state"], "idle")

    def test_diagnose(self):
        d = self.req("/api/diagnose", {})[1]
        self.assertFalse(d["ok"])  # まだフレームが無い
        self.state.update_raw_frame(degrade(_BASE, sigma=4.5))
        code, d = self.req("/api/diagnose", {})
        self.assertEqual(code, 200)
        self.assertTrue(d["ok"])
        self.assertEqual(d["resolution"], {"width": 640, "height": 480})
        self.assertIn("離す/近づけ", d["advice"])

    def test_html_has_controls(self):
        with urllib.request.urlopen(self.base + "/", timeout=5) as r:
            html = r.read().decode("utf-8")
        for needle in ("画質を自動調整", "元に戻す", "ピント合わせモード", "診断", "/api/autotune", "/api/focus.json"):
            self.assertIn(needle, html)


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class RunLoopPauseTest(unittest.TestCase):
    """run_loop全体をスタブで回し、自動調整中は読み取り(認識)が止まり、終われば再開することを確かめる。"""

    def test_recognition_paused_while_autotune_runs(self):
        real_sleep = time.sleep
        stop = threading.Event()
        captured = {}
        recog_calls = [0]
        gate = threading.Event()

        def fast_sleep(sec):
            if stop.is_set():
                raise SystemExit  # スレッドを終わらせる
            real_sleep(min(sec, 0.02))

        def fake_recognize(*a, **k):
            recog_calls[0] += 1
            return 42

        def fake_start_server(state, *a, **k):
            captured["state"] = state
            return mock.MagicMock()

        def blocking_run(self_):
            gate.wait(10)
            return {"ok": True, "status": "unchanged", "message": "stub", "changes": {}, "skipped": {},
                    "applied": {}, "original": {}, "baseline": None, "final": None, "evals": 0, "elapsed_sec": 0.0}

        cfg = dict(t.DEFAULT_CONFIG, auth_password="x", camera_profile="demo", preview_port=_free_port(),
                   preview_host="127.0.0.1", _config_path=os.path.join(tempfile.mkdtemp(), "c.json"))
        cap = FakeUvcCap()
        patches = [
            mock.patch.object(cv2, "VideoCapture", lambda *a, **k: cap),
            mock.patch.object(t, "fetch_slot_info", lambda *a, **k: {}),
            mock.patch.object(t, "recognize_digits", fake_recognize),
            mock.patch.object(t, "build_recognizer", lambda c: None),
            mock.patch.object(t, "start_preview_server", fake_start_server),
            mock.patch.object(cq.CameraTuner, "run", blocking_run),
            mock.patch.object(time, "sleep", fast_sleep),
        ]
        for p in patches:
            p.start()
        th = threading.Thread(target=lambda: t.run_loop(cfg), daemon=True)
        try:
            th.start()
            deadline = time.monotonic() + 15
            while recog_calls[0] < 14 and time.monotonic() < deadline:
                real_sleep(0.05)
            self.assertGreaterEqual(recog_calls[0], 14, "認識が走っていない(2周期分=14回以上を期待)")
            st = captured["state"]
            self.assertTrue(st.request_autotune("start")[0])
            deadline = time.monotonic() + 10
            while st.snapshot_autotune()["state"] != "running" and time.monotonic() < deadline:
                real_sleep(0.02)
            self.assertEqual(st.snapshot_autotune()["state"], "running")
            real_sleep(0.3)  # 実行中の周期の途中だったものが終わるのを待つ
            frozen = recog_calls[0]
            real_sleep(0.6)
            self.assertEqual(recog_calls[0], frozen, "自動調整中に認識が走った")
            gate.set()
            deadline = time.monotonic() + 10
            while st.snapshot_autotune()["state"] == "running" and time.monotonic() < deadline:
                real_sleep(0.02)
            self.assertEqual(st.snapshot_autotune()["state"], "done")
            deadline = time.monotonic() + 10
            while recog_calls[0] <= frozen and time.monotonic() < deadline:
                real_sleep(0.05)
            self.assertGreater(recog_calls[0], frozen, "自動調整後に認識が再開しない")
        finally:
            gate.set()
            stop.set()
            th.join(5)
            real_sleep(0.3)  # 撮影スレッドも(スタブのsleepで)終わるまで待ってからパッチを外す
            for p in patches[::-1]:
                p.stop()


def _load_proxy():
    import importlib.util

    path = os.path.normpath(os.path.join(
        HERE, "..", "..", "..", "ros2_ws", "src", "nokosahen_core", "nokosahen_core", "ocr_admin_proxy.py"))
    spec = importlib.util.spec_from_file_location("ocr_admin_proxy_under_test", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class AdminProxyContractTest(unittest.TestCase):
    """総合ダッシュボード側 ocr_admin_proxy.py と、このOCRの確認画面APIの契約(実サーバーで確認)。"""

    def setUp(self):
        self.mod = _load_proxy()
        self.state = _state("production")
        self.server = t.start_preview_server(self.state, "127.0.0.1", 0, None)
        self.proxy = self.mod.OcrAdminProxy(f"http://127.0.0.1:{self.server.server_address[1]}")

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def test_state_includes_quality(self):
        self.state.focus.update(cv2, np, _BASE, BOX)
        d = self.proxy.get_state()
        self.assertTrue(d["ok"], d)
        q = d["state"]["quality"]
        self.assertEqual(q["focus"]["grade"], "good")
        self.assertEqual(q["autotune"]["state"], "idle")
        self.assertFalse(q["autotune"]["has_saved"])

    def test_set_autotune_forwards_and_validates(self):
        self.assertEqual(self.proxy.set_autotune({"action": "start"}), (200, {"ok": True, "configured": True, "action": "start"}))
        self.assertEqual(self.state.take_autotune_request(), "start")
        for bad in ({}, {"action": "x"}, {"action": "start", "x": 1}, None, [], {"action": 1}):
            self.assertEqual(self.proxy.set_autotune(bad)[0], 400, bad)

    def test_busy_reason_is_propagated(self):
        self.proxy.set_autotune({"action": "start"})
        code, d = self.proxy.set_autotune({"action": "revert"})
        self.assertEqual(code, 200)
        self.assertFalse(d["ok"])
        self.assertIn("実行中", d["message"])

    def test_old_ocr_without_quality_is_tolerated(self):
        self.assertIsNone(self.mod.summarize_quality({}, {}))
        self.assertIsNone(self.mod.summarize_quality(None, None))


if __name__ == "__main__":
    unittest.main()
