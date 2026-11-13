"""OCR前処理の画質改善(ベストフレーム選択・複数フレーム平均・自動レベル・アンシャープ・CLAHE・
Lanczos拡大)の単体テスト(2026-10-10追加)。

実行: python scripts/tablet_ocr_reader/tests/test_image_enhance.py
合成した模擬POS画面(tests/synth_frames.py)にピンぼけ・ノイズ・低コントラストを加えた劣化画像で、
前処理あり/なしを比べる。Coral(桁認識)とTesseractは使わず、鮮鋭度・行検出・桁の切り出しまでを確認する。
"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import ocr_camera_quality as cq  # noqa: E402
import tablet_ocr_reader as t  # noqa: E402
from synth_frames import ROW_VALUES, camera_frame, degrade, mock_screen  # noqa: E402

BOX = (0.3, 0.1, 0.7, 0.9)
SCREEN = mock_screen()
CLEAN = camera_frame(SCREEN, 1280, 960, 0.8)
E = cq.EnhanceSettings


def score(img):
    return cq.edge_sharpness(cv2, np, img, BOX)["score"]


CLEAN_ROWS = t.detect_digit_rows(cv2, np, CLEAN)


def rows_match(sel, ref=CLEAN_ROWS, tol=25):
    """検出した7行が、鮮明画像での行の縦位置(上端・下端がtol px以内)と一致するか。
    (横は、ぼけで桁の端が欠けたりノイズの塊が混ざったりするので見ない)"""
    if sel is None or len(sel) != len(ref):
        return False
    return all(abs(r[1] - q[1]) <= tol and abs(r[3] - q[3]) <= tol for r, q in zip(sel, ref))


def make_history(frames, dt=0.2):
    return [(i * dt, f) for i, f in enumerate(frames)]


class ResolveSettingsTest(unittest.TestCase):
    def test_defaults_enabled_and_modest(self):
        es = cq.resolve_image_enhance({}, "production")
        self.assertEqual((es.best_of_n, es.average_frames), (5, 3))
        self.assertGreater(es.unsharp_amount, 0)
        self.assertLessEqual(es.unsharp_amount, 1.0)  # 控えめ
        self.assertTrue(es.auto_levels)
        self.assertEqual(es.interpolation, "lanczos")
        self.assertEqual(es.clahe, 0.0)  # 検証で悪化したため既定は無効
        self.assertTrue(es.enabled)

    def test_profile_specific_n(self):
        demo = cq.resolve_image_enhance({}, "demo")
        prod = cq.resolve_image_enhance({}, "production")
        self.assertEqual((demo.best_of_n, demo.average_frames), (3, 2))
        self.assertGreater(prod.best_of_n, demo.best_of_n)  # 本番は低fpsなので幅は枚数より時間で稼ぐ
        legacy = cq.resolve_image_enhance({}, None)
        self.assertEqual(legacy.best_of_n, demo.best_of_n)

    def test_false_disables_everything(self):
        es = cq.resolve_image_enhance({"image_enhance": False}, "production")
        self.assertFalse(es.enabled)
        self.assertFalse(es.any_frame_stage or es.any_crop_stage)
        self.assertEqual(es.interpolation, "cubic")

    def test_individual_overrides(self):
        es = cq.resolve_image_enhance({"image_enhance": {
            "best_of_n": {"demo": 2, "production": 7}, "average_frames": 1, "unsharp_amount": 0,
            "clahe": 2.5, "auto_levels": False, "interpolation": "cubic"}}, "production")
        self.assertEqual((es.best_of_n, es.average_frames), (7, 1))
        self.assertEqual(es.unsharp_amount, 0.0)
        self.assertEqual(es.clahe, 2.5)
        self.assertFalse(es.auto_levels)
        self.assertEqual(es.interpolation, "cubic")
        es2 = cq.resolve_image_enhance({"image_enhance": {"clahe": True, "best_of_n": 0}}, "demo")
        self.assertEqual(es2.clahe, 2.0)
        self.assertEqual(es2.best_of_n, 1)  # 0=無効

    def test_invalid_values_fall_back_with_warning(self):
        bad = {"image_enhance": {"best_of_n": 99, "average_frames": "x", "unsharp_amount": -1, "unsharp_radius": 50,
                                 "clahe": "high", "auto_levels": "yes", "interpolation": "bilinear", "bogus": 1}}
        with self.assertLogs("tablet_ocr_reader", level="WARNING") as cm:
            es = cq.resolve_image_enhance(bad, "production")
        self.assertGreaterEqual(len(cm.output), 7)
        self.assertEqual(es, cq.resolve_image_enhance({}, "production"))
        with self.assertLogs("tablet_ocr_reader", level="WARNING"):
            self.assertTrue(cq.resolve_image_enhance({"image_enhance": "on"}, "demo").enabled)

    def test_focus_thresholds_validation(self):
        self.assertEqual(cq.resolve_focus_thresholds(None), cq.DEFAULT_FOCUS_THRESHOLDS)
        self.assertEqual(cq.resolve_focus_thresholds({"good": 80, "fair": 40}), {"good": 80.0, "fair": 40.0})
        for bad in ({"good": 10, "fair": 40}, {"good": 5}, "x", {"good": float("nan"), "fair": 1}):
            with self.assertLogs("tablet_ocr_reader", level="WARNING"):
                self.assertEqual(cq.resolve_focus_thresholds(bad), cq.DEFAULT_FOCUS_THRESHOLDS)


class PrimitiveStagesTest(unittest.TestCase):
    def test_auto_levels_stretches_low_contrast(self):
        low = degrade(CLEAN, contrast=0.4)
        g = cv2.cvtColor(low, cv2.COLOR_BGR2GRAY)
        out = cq.auto_levels_gray(np, g)
        self.assertGreater(float(np.percentile(out, 99) - np.percentile(out, 1)),
                           float(np.percentile(g, 99) - np.percentile(g, 1)) * 1.8)

    def test_auto_levels_leaves_normal_and_flat_images(self):
        g = cv2.cvtColor(CLEAN, cv2.COLOR_BGR2GRAY)
        self.assertIs(cq.auto_levels_gray(np, g), g)
        flat = np.full((50, 50), 100, np.uint8)
        self.assertIs(cq.auto_levels_gray(np, flat), flat)

    def test_auto_levels_bgr_keeps_hue_order(self):
        low = degrade(CLEAN, contrast=0.4)
        out = cq.auto_levels_bgr(np, low)
        hsv = cv2.cvtColor(out, cv2.COLOR_BGR2HSV)
        hsv_low = cv2.cvtColor(low, cv2.COLOR_BGR2HSV)
        self.assertGreater(int((hsv[..., 1] > 60).sum()), int((hsv_low[..., 1] > 60).sum()))

    def test_unsharp_increases_edge_sharpness_of_blurred_image(self):
        blurred = degrade(CLEAN, sigma=3.0)
        sharp = cq.unsharp_mask(cv2, np, blurred, 1.5, 3.0)
        self.assertGreater(score(sharp), score(blurred) * 1.15)
        self.assertEqual(sharp.shape, blurred.shape)
        self.assertEqual(sharp.dtype, np.uint8)

    def test_unsharp_zero_is_identity(self):
        self.assertIs(cq.unsharp_mask(cv2, np, CLEAN, 0.0, 2.0), CLEAN)

    def test_clahe_raises_local_contrast(self):
        g = cv2.cvtColor(degrade(CLEAN, contrast=0.3), cv2.COLOR_BGR2GRAY)
        out = cq.clahe_gray(cv2, g, 2.0)
        self.assertGreater(float(out.std()), float(g.std()))
        self.assertIs(cq.clahe_gray(cv2, g, 0.0), g)

    def test_interpolation_flags(self):
        self.assertEqual(cq.interpolation_flag(cv2, "lanczos", 2.0), cv2.INTER_LANCZOS4)
        self.assertEqual(cq.interpolation_flag(cv2, "cubic", 2.0), cv2.INTER_CUBIC)
        self.assertEqual(cq.interpolation_flag(cv2, "lanczos", 0.5), cv2.INTER_AREA)

    def test_rois_bounding_box(self):
        rois = {"A": t.Roi(0.5, 0.1, 0.9, 0.2), "B": t.Roi(0.55, 0.3, 0.95, 0.4)}
        self.assertEqual(cq.rois_bounding_box(rois), (0.5, 0.1, 0.95, 0.4))
        self.assertIsNone(cq.rois_bounding_box({}))


class PreprocessIntegrationTest(unittest.TestCase):
    ROW = CLEAN[int(960 * 0.1):int(960 * 0.2), 640:1000]

    def test_enhance_none_is_exactly_the_legacy_path(self):
        crop = CLEAN[100:200, 600:1000]
        legacy = t.preprocess_for_ocr(cv2, crop, 2.0)
        self.assertTrue(np.array_equal(legacy, t.preprocess_for_ocr(cv2, crop, 2.0, None)))
        self.assertTrue(np.array_equal(legacy, t.preprocess_for_ocr(cv2, crop, 2.0, cq.DISABLED_ENHANCE)))
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        manual = cv2.resize(gray, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
        _, thr = cv2.threshold(manual, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        self.assertTrue(np.array_equal(legacy, thr))

    def test_enhanced_output_is_binary_and_scaled(self):
        crop = CLEAN[100:200, 600:1000]
        es = cq.resolve_image_enhance({}, "production")
        out = t.preprocess_for_ocr(cv2, crop, 2.5, es)
        self.assertEqual(out.shape, (250, 1000))
        self.assertEqual(set(np.unique(out)) - {0, 255}, set())

    def test_clean_image_result_stays_close_to_legacy(self):
        # 既存の認識挙動を悪化させない: 鮮明な画像では二値化結果がほぼ同じ
        es = cq.resolve_image_enhance({}, "production")
        for y0 in range(0, 7):
            y1, y2 = int(960 * 0.1) + y0 * 90, int(960 * 0.1) + y0 * 90 + 60
            crop = CLEAN[y1:y2, 700:1000]
            a = t.preprocess_for_ocr(cv2, crop, 2.0) > 0
            b = t.preprocess_for_ocr(cv2, crop, 2.0, es) > 0
            self.assertGreater(float((a == b).mean()), 0.95)

    def test_blurred_digits_still_segment_into_three(self):
        es = cq.resolve_image_enhance({}, "production")
        rows = t.detect_digit_rows(cv2, np, CLEAN)
        self.assertEqual(len(rows), 7)
        blurred = degrade(CLEAN, sigma=2.0)
        ok = 0
        for (x1, y1, x2, y2) in rows:
            segs = t.segment_digits(cv2, t.preprocess_for_ocr(cv2, blurred[y1:y2, x1:x2], 2.0, es))
            ok += len(segs) == len(str(ROW_VALUES[rows.index((x1, y1, x2, y2))]))
        self.assertGreaterEqual(ok, 4)

    def test_recognize_digits_threads_enhance_to_preprocess(self):
        seen = []

        class Rec:
            def predict_digit(self, img):
                return 7

        orig = t.preprocess_for_ocr

        def spy(cv2_, crop, scale, enhance=None):
            seen.append(enhance)
            return orig(cv2_, crop, scale, enhance)

        es = cq.resolve_image_enhance({}, "demo")
        t.preprocess_for_ocr = spy
        try:
            val = t.recognize_digits(cv2, self.ROW, Rec(), 2.0, es)
        finally:
            t.preprocess_for_ocr = orig
        self.assertIsNotNone(val)
        self.assertIs(seen[0], es)


class FrameHistoryTest(unittest.TestCase):
    def test_interval_maxlen_clear(self):
        now = [0.0]
        h = cq.FrameHistory(maxlen=3, min_interval_sec=0.1, clock=lambda: now[0])
        self.assertTrue(h.push(CLEAN))
        now[0] = 0.05
        self.assertFalse(h.push(CLEAN))  # 間引く
        for i in range(1, 6):
            now[0] = i * 0.2
            self.assertTrue(h.push(CLEAN))
        self.assertEqual(len(h), 3)
        h.clear()
        self.assertEqual(len(h), 0)
        self.assertTrue(h.push(CLEAN))


class BuildEnhancedFrameTest(unittest.TestCase):
    ES = cq.resolve_image_enhance({}, "production")

    def test_disabled_or_single_frame_returns_newest(self):
        f = degrade(CLEAN, sigma=1)
        out, info = cq.build_enhanced_frame(cv2, np, make_history([f]), BOX, self.ES)
        self.assertIs(out, f)
        off = cq.resolve_image_enhance({"image_enhance": False}, "production")
        out, _ = cq.build_enhanced_frame(cv2, np, make_history([CLEAN, f]), BOX, off)
        self.assertIs(out, f)
        self.assertEqual(cq.build_enhanced_frame(cv2, np, [], BOX, self.ES)[0], None)

    def test_best_of_n_picks_sharpest(self):
        es = E(best_of_n=5, average_frames=1)
        frames = [degrade(CLEAN, sigma=s) for s in (3.0, 2.0, 0.0, 2.5, 3.0)]  # 3枚目が最も鮮明
        out, info = cq.build_enhanced_frame(cv2, np, make_history(frames), BOX, es)
        self.assertIs(out, frames[2])
        self.assertEqual(info["averaged"], 1)

    def test_best_of_n_window_is_last_n(self):
        es = E(best_of_n=2, average_frames=1)
        frames = [CLEAN, degrade(CLEAN, sigma=3), degrade(CLEAN, sigma=2)]  # 最新2枚の中ではσ2が最良
        out, _ = cq.build_enhanced_frame(cv2, np, make_history(frames), BOX, es)
        self.assertIs(out, frames[2])

    def test_average_reduces_noise(self):
        es = E(best_of_n=1, average_frames=5)
        frames = [degrade(CLEAN, noise=15, seed=i) for i in range(5)]
        out, info = cq.build_enhanced_frame(cv2, np, make_history(frames), BOX, es)
        self.assertEqual(info["averaged"], 5)

        def mse(a):
            return float(((a.astype(np.float32) - CLEAN.astype(np.float32)) ** 2).mean())

        self.assertLess(mse(out), mse(frames[-1]) * 0.35)

    def test_changed_digit_frame_is_not_averaged_in(self):
        es = E(best_of_n=1, average_frames=4)
        other = camera_frame(mock_screen(values=[118, 129, 353, 166, 211, 178, 104]), 1280, 960, 0.8)
        frames = [other, CLEAN, CLEAN.copy(), CLEAN.copy()]  # 古い1枚だけ数字が違う(103→104)
        out, info = cq.build_enhanced_frame(cv2, np, make_history(frames), BOX, es)
        self.assertEqual(info["averaged"], 3)
        self.assertTrue(np.array_equal(out, CLEAN))  # 古い数字が混ざらない

    def test_shifted_frame_is_excluded(self):
        es = E(best_of_n=1, average_frames=3)
        shifted = np.roll(CLEAN, 12, axis=1)
        out, info = cq.build_enhanced_frame(cv2, np, make_history([shifted, CLEAN, CLEAN.copy()]), BOX, es)
        self.assertEqual(info["averaged"], 2)

    def test_stale_frames_are_ignored(self):
        es = E(best_of_n=5, average_frames=3, max_age_sec=2.0)
        stale = degrade(CLEAN, sigma=0.0)
        newest = degrade(CLEAN, sigma=2.0)
        hist = [(0.0, stale), (10.0, newest)]
        out, info = cq.build_enhanced_frame(cv2, np, hist, BOX, es)
        self.assertIs(out, newest)  # 10秒前のフレームは使わない

    def test_shape_mismatch_ignored(self):
        es = E(best_of_n=3, average_frames=3)
        small = cv2.resize(CLEAN, (640, 480))
        out, _ = cq.build_enhanced_frame(cv2, np, make_history([small, CLEAN, CLEAN.copy()]), BOX, es)
        self.assertEqual(out.shape, CLEAN.shape)

    def test_best_of_n_on_noisy_sequence_improves_sharpness_score(self):
        es = E(best_of_n=5, average_frames=1)
        seq = [degrade(CLEAN, sigma=s) for s in (3.0, 2.5, 1.0, 2.8, 3.2)]
        out, _ = cq.build_enhanced_frame(cv2, np, make_history(seq), BOX, es)
        self.assertGreater(score(out), score(seq[-1]) * 1.3)  # 最新フレームをそのまま使うより鮮鋭


class DegradedPipelineTest(unittest.TestCase):
    """劣化画像(ピンぼけ+ノイズ+低コントラスト)で、前処理あり/なしの行検出を比較する。"""
    ES = cq.resolve_image_enhance({}, "production")

    def detect_rows(self, frames):
        """run_loopと同じ流れ: 履歴→強化フレーム→検出(失敗なら強化版で再検出)。(行数が7で成立したか)。"""
        enh, _ = cq.build_enhanced_frame(cv2, np, make_history(frames), BOX, self.ES)
        sel = t.select_best_rows(t.detect_digit_rows(cv2, np, enh), 7)
        if sel is None:
            for _, vf in cq.detection_variants(cv2, np, enh, self.ES):
                sel = t.select_best_rows(t.detect_digit_rows(cv2, np, vf), 7)
                if sel is not None:
                    break
        return sel

    def test_noisy_blurred_frames_raw_fails_pipeline_finds_seven_rows(self):
        for kw in (dict(sigma=3, noise=12), dict(sigma=3, noise=12, contrast=0.5),
                   dict(sigma=2.5, noise=10, contrast=0.6)):
            with self.subTest(**kw):
                frames = [degrade(CLEAN, seed=i, **kw) for i in range(5)]
                raw = t.select_best_rows(t.detect_digit_rows(cv2, np, frames[-1]), 7)
                self.assertFalse(rows_match(raw), "生フレームでは正しく検出できない条件のはず(テスト条件の見直しが必要)")
                sel = self.detect_rows(frames)
                self.assertEqual(len(sel), 7)
                self.assertTrue(rows_match(sel), sel)  # 7行が実際の行位置と一致する

    def test_pipeline_never_worse_on_easy_conditions(self):
        for kw in ({}, dict(sigma=1.5), dict(sigma=2.5), dict(sigma=4)):
            with self.subTest(**kw):
                frames = [degrade(CLEAN, seed=i, **kw) for i in range(5)]
                self.assertTrue(rows_match(t.select_best_rows(t.detect_digit_rows(cv2, np, frames[-1]), 7)))
                self.assertTrue(rows_match(self.detect_rows(frames)))

    def test_focus_score_improves_with_best_of_n_on_mixed_quality(self):
        frames = [degrade(CLEAN, sigma=s, seed=i) for i, s in enumerate((3.5, 3.0, 1.2, 3.5, 3.2))]
        enh, _ = cq.build_enhanced_frame(cv2, np, make_history(frames), BOX, self.ES)
        self.assertGreater(score(enh), score(frames[-1]) * 1.5)

    def test_detection_variants_raise_contrast_and_sharpness(self):
        low = degrade(CLEAN, sigma=2.5, contrast=0.4)
        variants = dict(cq.detection_variants(cv2, np, low, self.ES))
        self.assertIn("levels", variants)
        self.assertIn("levels+unsharp1.5", variants)
        g_low = cv2.cvtColor(low, cv2.COLOR_BGR2GRAY)
        g_lv = cv2.cvtColor(variants["levels"], cv2.COLOR_BGR2GRAY)
        self.assertGreater(float(g_lv.std()), float(g_low.std()) * 1.8)
        self.assertGreater(score(variants["levels+unsharp2.5"]), score(variants["levels"]))
        for v in variants.values():
            self.assertEqual(v.shape, low.shape)
            self.assertEqual(v.dtype, np.uint8)

    def test_variants_disabled_when_no_crop_stage(self):
        es = cq.resolve_image_enhance({"image_enhance": {"auto_levels": False, "unsharp_amount": 0}}, "demo")
        self.assertEqual(list(cq.detection_variants(cv2, np, CLEAN, es)), [])

    def test_gray_stage_sharpness_improves_for_blurred_roi(self):
        crop = degrade(CLEAN, sigma=3.0)[100:860, 640:1000]
        g = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        old = cv2.resize(g, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
        new = cq.enhance_gray_for_ocr(cv2, np, g, 2.0, self.ES)
        self.assertEqual(old.shape, new.shape)
        self.assertGreater(score(new), score(old))


if __name__ == "__main__":
    unittest.main()
