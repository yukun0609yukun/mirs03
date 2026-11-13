"""前方カメラの開き直し判断(camera_recovery、rclpy・cv2不要)と、person_target_detector への配線の静的確認。"""
import importlib.util
import sys
import unittest
from pathlib import Path

_DIR = Path(__file__).resolve().parents[1] / "nokosahen_core"
_spec = importlib.util.spec_from_file_location("camera_recovery_under_test", _DIR / "camera_recovery.py")
cr = importlib.util.module_from_spec(_spec)
sys.modules["camera_recovery_under_test"] = cr
_spec.loader.exec_module(cr)


class CameraRecoveryTest(unittest.TestCase):
    def test_release_only_after_continuous_failure(self):
        c = cr.CameraRecovery(3.0, 10.0)
        self.assertFalse(c.on_frame_fail(0.0))
        self.assertFalse(c.on_frame_fail(2.0))
        c.on_frame_ok()                         # 途中で取れたらやり直し
        self.assertFalse(c.on_frame_fail(2.5))
        self.assertFalse(c.on_frame_fail(5.0))
        self.assertTrue(c.on_frame_fail(5.6))
        self.assertTrue(c.down)
        self.assertFalse(c.on_frame_fail(6.0))  # 解放の合図は1回だけ

    def test_reopen_interval_and_no_fast_loop(self):
        c = cr.CameraRecovery(3.0, 10.0)
        c.on_frame_fail(0.0)
        self.assertTrue(c.on_frame_fail(3.0))
        self.assertFalse(c.should_reopen(3.5))   # 解放直後はすぐ開かない
        self.assertFalse(c.should_reopen(12.9))
        self.assertTrue(c.should_reopen(13.0))
        self.assertFalse(c.should_reopen(14.0))  # 試行したら次は10秒後
        self.assertTrue(c.should_reopen(23.0))

    def test_startup_failure_retries_later(self):
        c = cr.CameraRecovery(3.0, 10.0)
        c.mark_down(100.0)
        self.assertFalse(c.should_reopen(105.0))
        self.assertTrue(c.should_reopen(110.0))
        c.opened()
        self.assertFalse(c.down)
        self.assertFalse(c.should_reopen(200.0))

    def test_ok_after_down_reports_recovery(self):
        c = cr.CameraRecovery(1.0, 5.0)
        c.on_frame_fail(0.0); c.on_frame_fail(1.0)
        self.assertTrue(c.on_frame_ok())
        self.assertFalse(c.down)

    def test_clock_regress_does_not_release(self):
        c = cr.CameraRecovery(3.0, 10.0)
        c.on_frame_fail(100.0)
        self.assertFalse(c.on_frame_fail(50.0))


class WiringTest(unittest.TestCase):
    src = (_DIR / "person_target_detector.py").read_text(encoding="utf-8")

    def test_reopen_wired_and_lidar_path_untouched(self):
        s = self.src
        self.assertIn("self._camera_recovery.on_frame_fail", s)
        self.assertIn("self._try_reopen_front_camera()", s)
        self.assertIn("_camera_go_down", s)
        # 起動時にカメラを開けなくても、カメラ無効(enable_camera=False)にして諦めない
        i = s.index("if not self._open_front_camera():")
        self.assertNotIn("self.enable_camera = False", s[i:i + 700])
        # 落ちている間はカメラ由来の候補を出さない: 新鮮さ(CAMERA_FRESH_SEC)で camera_active を決める既存の仕組みのまま
        self.assertIn("now - self._camera_bearings_stamp <= CAMERA_FRESH_SEC", s)


if __name__ == "__main__":
    unittest.main()
