"""frame_rotate_deg(撮影直後のフレーム回転)の単体テスト(2026-10-09追加)。

実行: python scripts/tablet_ocr_reader/tests/test_frame_rotate.py
(カメラ・Tesseract・Edge TPUは不要。確認用ダッシュボードのHTTPは127.0.0.1の
空きポートで実際に立ち上げて検証する。)
"""
import json
import os
import sys
import tempfile
import unittest
import urllib.error
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import tablet_ocr_reader as t  # noqa: E402

import cv2  # type: ignore  # noqa: E402
import numpy as np  # noqa: E402


class NormalizeRotateDegTest(unittest.TestCase):
    def test_valid_values(self):
        for v in (0, 90, 180, 270, 90.0):
            with self.subTest(v=v):
                self.assertEqual(t.normalize_rotate_deg(v), int(v))

    def test_invalid_values_warn_and_become_zero(self):
        for v in (45, 360, -90, 91, 90.5, "90", None, True, [90]):
            with self.subTest(v=v):
                with self.assertLogs("tablet_ocr_reader", level="WARNING"):
                    self.assertEqual(t.normalize_rotate_deg(v), 0)

    def test_default_in_config_is_zero(self):
        self.assertEqual(t.DEFAULT_CONFIG["frame_rotate_deg"], 0)


class RotateFrameTest(unittest.TestCase):
    def setUp(self):
        # 高さ3 x 幅5の非正方形。各画素が一意な値(B=x, G=y)
        self.h, self.w = 3, 5
        self.frame = np.zeros((self.h, self.w, 3), np.uint8)
        for y in range(self.h):
            for x in range(self.w):
                self.frame[y, x] = (x, y, 7)

    def test_zero_is_same_object(self):
        self.assertIs(t.rotate_frame(cv2, self.frame, 0), self.frame)

    def test_90_clockwise(self):
        out = t.rotate_frame(cv2, self.frame, 90)
        self.assertEqual(out.shape[:2], (self.w, self.h))  # 幅と高さが入れ替わる
        # 時計回り90度: 元(x,y) -> 新(x'=h-1-y, y'=x)。左上(0,0)は右上(h-1,0)へ
        for y in range(self.h):
            for x in range(self.w):
                self.assertEqual(tuple(out[x, self.h - 1 - y]), (x, y, 7))

    def test_180(self):
        out = t.rotate_frame(cv2, self.frame, 180)
        self.assertEqual(out.shape[:2], (self.h, self.w))
        for y in range(self.h):
            for x in range(self.w):
                self.assertEqual(tuple(out[self.h - 1 - y, self.w - 1 - x]), (x, y, 7))

    def test_270_counterclockwise(self):
        out = t.rotate_frame(cv2, self.frame, 270)
        self.assertEqual(out.shape[:2], (self.w, self.h))
        # 反時計回り90度: 元(x,y) -> 新(x'=y, y'=w-1-x)。左上(0,0)は左下(0,w-1)へ
        for y in range(self.h):
            for x in range(self.w):
                self.assertEqual(tuple(out[self.w - 1 - x, y]), (x, y, 7))

    def test_90_then_270_restores(self):
        back = t.rotate_frame(cv2, t.rotate_frame(cv2, self.frame, 90), 270)
        self.assertTrue(np.array_equal(back, self.frame))

    def test_roi_crop_uses_rotated_width_and_height(self):
        # 640x480のフレームを90度回すと480x640。比率ROIは回転後の幅/高さで画素へ変換される
        frame = np.zeros((480, 640, 3), np.uint8)
        rot = t.rotate_frame(cv2, frame, 90)
        crop = t.crop_roi(cv2, rot, t.Roi(0.5, 0.25, 1.0, 0.5))
        self.assertEqual(crop.shape[:2], (160, 240))  # 高さ640*0.25、幅480*0.5
        box = (100, 200, 300, 400)
        roi = t._pixel_box_to_roi(rot, box)
        self.assertAlmostEqual(roi.x2, 300 / 480)
        self.assertAlmostEqual(roi.y2, 400 / 640)


class DetectAfterRotateTest(unittest.TestCase):
    W, H = 1280, 960

    def _upright(self, n_rows=7):
        img = np.full((self.H, self.W, 3), 255, np.uint8)
        red = (30, 30, 200)
        for i in range(n_rows):
            y = 80 + i * 110
            for k, ch in enumerate("12"):
                cv2.putText(img, ch, (900 + k * 55, y), cv2.FONT_HERSHEY_SIMPLEX, 2.2, red, 6)
        return img

    def test_sideways_camera_image_fails_without_rotation_and_works_with_it(self):
        upright = self._upright()
        for camera_deg, fix_deg in ((90, 270), (270, 90), (180, 180)):
            with self.subTest(camera_deg=camera_deg):
                camera = t.rotate_frame(cv2, upright, camera_deg)  # 横に回って写った映像
                if camera_deg != 180:
                    self.assertEqual(camera.shape[:2], (self.W, self.H))
                    plain = t.select_best_rows(t.detect_digit_rows(cv2, np, camera), 7)
                    self.assertIsNone(plain)  # 回転しないと7行にならない
                fixed = t.rotate_frame(cv2, camera, fix_deg)
                self.assertTrue(np.array_equal(fixed, upright))
                boxes = t.select_best_rows(t.detect_digit_rows(cv2, np, fixed), 7)
                self.assertIsNotNone(boxes)
                self.assertEqual(len(boxes), 7)

    def test_locker_locks_with_rotated_frame_dimensions(self):
        camera = t.rotate_frame(cv2, self._upright(), 90)
        fixed = t.rotate_frame(cv2, camera, 270)
        lk = t.RoiLocker(required=3)
        for _ in range(3):
            boxes = t.select_best_rows(t.detect_digit_rows(cv2, np, fixed), 7)
            lk.observe(boxes, fixed.shape[1], fixed.shape[0])
        self.assertTrue(lk.is_locked)
        rois = t.ratio_boxes_to_rois(lk.locked)
        self.assertEqual(len(rois), 7)
        # ROIは回転後(1280x960)の座標: 数字は右寄り(x>0.6)に縦並びでyが昇順
        ys = [r.y1 for r in rois.values()]
        self.assertEqual(ys, sorted(ys))
        self.assertTrue(all(r.x1 > 0.6 for r in rois.values()))


class PreviewStateRotateTest(unittest.TestCase):
    def test_initial_and_snapshot(self):
        s = t.PreviewState(2.0, 270)
        self.assertEqual(s.get_rotate_deg(), 270)
        self.assertEqual(s.snapshot_readings()["frame_rotate_deg"], 270)
        self.assertEqual(t.PreviewState().get_rotate_deg(), 0)

    def test_change_invalidates_rois_and_requests_relock(self):
        s = t.PreviewState(2.0, 0)
        s.update_dynamic_rois({"A": t.Roi(0, 0, 1, 1)})
        s.update_raw_frame(np.zeros((4, 4, 3), np.uint8))
        self.assertFalse(s.set_rotate_deg(0))  # 変化なしは何もしない
        self.assertFalse(s.consume_relock_request())
        self.assertTrue(s.set_rotate_deg(90))
        self.assertEqual(s.snapshot_dynamic_rois(), {})
        self.assertIsNone(s.get_raw_frame())
        self.assertTrue(s.consume_relock_request())

    def test_stale_frame_from_old_rotation_is_dropped(self):
        s = t.PreviewState(2.0, 0)
        s.set_rotate_deg(90)
        s.update_raw_frame(np.zeros((4, 4, 3), np.uint8), 0)  # 切替前の角度で回したフレーム
        self.assertIsNone(s.get_raw_frame())
        s.update_raw_frame(np.zeros((4, 4, 3), np.uint8), 90)
        self.assertIsNotNone(s.get_raw_frame())


class SettingsApiTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.cfg = os.path.join(self.dir, "cfg.json")
        with open(self.cfg, "w", encoding="utf-8") as f:
            json.dump({"auth_password": "", "camera_index": "auto", "ocr_scale_factor": 2.0}, f)
        self.state = t.PreviewState(2.0, 0)
        self.server = t.start_preview_server(self.state, "127.0.0.1", 0, self.cfg)
        self.base = f"http://127.0.0.1:{self.server.server_address[1]}"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def _post(self, body):
        req = urllib.request.Request(
            self.base + "/api/settings", data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"}, method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status
        except urllib.error.HTTPError as e:
            e.close()
            return e.code

    def _cfg(self):
        with open(self.cfg, encoding="utf-8") as f:
            return json.load(f)

    def test_get_settings_includes_rotation(self):
        with urllib.request.urlopen(self.base + "/api/settings.json", timeout=5) as r:
            data = json.load(r)
        self.assertEqual(data["frame_rotate_deg"], 0)
        self.assertEqual(data["ocr_scale_factor"], 2.0)

    def test_post_rotation_applies_persists_and_requests_relock(self):
        self.assertEqual(self._post({"frame_rotate_deg": 270}), 200)
        self.assertEqual(self.state.get_rotate_deg(), 270)
        self.assertTrue(self.state.consume_relock_request())
        saved = self._cfg()
        self.assertEqual(saved["frame_rotate_deg"], 270)
        self.assertEqual(saved["ocr_scale_factor"], 2.0)  # 既存の項目は保たれる
        self.assertEqual(saved["auth_password"], "")
        with urllib.request.urlopen(self.base + "/api/settings.json", timeout=5) as r:
            self.assertEqual(json.load(r)["frame_rotate_deg"], 270)

    def test_invalid_rotation_rejected_and_nothing_changes(self):
        for bad in (45, -90, 360, "90", 90.5, True, None, [90]):
            with self.subTest(bad=bad):
                self.assertEqual(self._post({"frame_rotate_deg": bad}), 400)
        self.assertEqual(self.state.get_rotate_deg(), 0)
        self.assertNotIn("frame_rotate_deg", self._cfg())
        self.assertFalse(self.state.consume_relock_request())

    def test_scale_only_post_still_works_and_empty_is_400(self):
        self.assertEqual(self._post({"ocr_scale_factor": 3.0}), 200)
        self.assertEqual(self.state.get_scale_factor(), 3.0)
        self.assertEqual(self.state.get_rotate_deg(), 0)
        self.assertEqual(self._cfg()["ocr_scale_factor"], 3.0)
        self.assertNotIn("frame_rotate_deg", self._cfg())
        self.assertEqual(self._post({}), 400)
        self.assertEqual(self._post({"ocr_scale_factor": 9.0}), 400)

    def test_same_rotation_does_not_request_relock(self):
        self.assertEqual(self._post({"frame_rotate_deg": 0}), 200)
        self.assertFalse(self.state.consume_relock_request())

    def test_load_config_roundtrip(self):
        self._post({"frame_rotate_deg": 90})
        cfg = t.load_config(self.cfg)
        self.assertEqual(t.normalize_rotate_deg(cfg["frame_rotate_deg"]), 90)


if __name__ == "__main__":
    unittest.main()
