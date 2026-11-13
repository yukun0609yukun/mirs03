"""カメラのプロファイル(demo/production)切替の単体テスト(2026-10-09追加)。

実行: python scripts/tablet_ocr_reader/tests/test_camera_profile.py
(実カメラは不要。cv2.VideoCaptureはスタブに差し替える。)
"""
import json
import os
import sys
import tempfile
import types
import unittest
import urllib.error
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import numpy as np  # noqa: E402
import tablet_ocr_reader as t  # noqa: E402


class FakeCap:
    """最大解像度(max_size)までしか出せないUVCカメラのスタブ。要求より大きい解像度は
    最大解像度へ丸める(V4L2の挙動)。sharpness_range=NoneならSHARPNESS非対応。"""

    def __init__(self, max_size=(1280, 960), sharpness_range=(0, 6), opened=True):
        self.max_size = max_size
        self.req = (640, 480)
        self.opened = opened
        self.props = {}
        self.sharp_range = sharpness_range
        self.sharp = 3.0
        self.released = False
        self.fail_read = False

    def isOpened(self):
        return self.opened

    def release(self):
        self.released = True

    def set(self, prop, value):
        if prop == FakeCV2.CAP_PROP_FRAME_WIDTH:
            self.req = (int(value), self.req[1])
        elif prop == FakeCV2.CAP_PROP_FRAME_HEIGHT:
            self.req = (self.req[0], int(value))
        elif prop == FakeCV2.CAP_PROP_SHARPNESS:
            if self.sharp_range is None or not (self.sharp_range[0] <= value <= self.sharp_range[1]):
                return False
            self.sharp = float(value)
        self.props[prop] = value
        return True

    def get(self, prop):
        if prop == FakeCV2.CAP_PROP_SHARPNESS:
            return self.sharp if self.sharp_range is not None else -1.0
        if prop == FakeCV2.CAP_PROP_FPS:
            return float(self.props.get(prop, 0.0))
        if prop == FakeCV2.CAP_PROP_FOURCC:
            return float(self.props.get(prop, 0))
        return 0.0

    def read(self):
        if self.fail_read:
            return False, None
        w = min(self.req[0], self.max_size[0])
        h = min(self.req[1], self.max_size[1])
        return True, np.zeros((h, w, 3), np.uint8)


class FakeCV2:
    CAP_PROP_FOURCC = 6
    CAP_PROP_FRAME_WIDTH = 3
    CAP_PROP_FRAME_HEIGHT = 4
    CAP_PROP_FPS = 5
    CAP_PROP_BUFFERSIZE = 38
    CAP_PROP_SHARPNESS = 20
    cap_factory = staticmethod(lambda: FakeCap())

    @staticmethod
    def VideoWriter_fourcc(a, b, c, d):
        return ord(a) | (ord(b) << 8) | (ord(c) << 16) | (ord(d) << 24)

    @classmethod
    def VideoCapture(cls, device):
        return cls.cap_factory()


class ResolveProfilesTest(unittest.TestCase):
    def test_defaults(self):
        p = t.resolve_camera_profiles({})
        self.assertEqual((p["demo"]["width"], p["demo"]["height"]), (800, 600))
        self.assertEqual(p["demo"]["capture_fps"], 30)
        self.assertGreaterEqual(p["demo"]["preview_fps"], 10)
        self.assertEqual((p["production"]["width"], p["production"]["height"]), (1280, 960))
        self.assertEqual(p["production"]["fourcc"], "MJPG")
        self.assertEqual(p["production"]["sharpness"], "max")
        self.assertLessEqual(p["production"]["preview_fps"], 1.0)
        self.assertEqual(p["production"]["preview_max_width"], 960)
        self.assertEqual(p["production"]["poll_interval_sec"], 5.0)

    def test_defaults_are_copies(self):
        p = t.resolve_camera_profiles({})
        p["demo"]["width"] = 1
        self.assertEqual(t.DEFAULT_CAMERA_PROFILES["demo"]["width"], 800)

    def test_override(self):
        p = t.resolve_camera_profiles({"camera_profiles": {"demo": {"width": 640, "height": 480, "preview_fps": 15}}})
        self.assertEqual((p["demo"]["width"], p["demo"]["height"], p["demo"]["preview_fps"]), (640, 480, 15))
        self.assertEqual(p["demo"]["capture_fps"], 30)  # 未指定は既定値
        self.assertEqual(p["production"]["width"], 1280)

    def test_invalid_overrides_ignored_with_warning(self):
        cfg = {"camera_profiles": {"demo": {"width": "big", "preview_fps": -1, "nokey": 1, "fourcc": "MJPEG"},
                                    "weird": {"width": 640}}}
        with self.assertLogs("tablet_ocr_reader", level="WARNING") as cm:
            p = t.resolve_camera_profiles(cfg)
        self.assertEqual(p["demo"], t.DEFAULT_CAMERA_PROFILES["demo"])
        self.assertNotIn("weird", p)
        self.assertGreaterEqual(len(cm.output), 5)
        with self.assertLogs("tablet_ocr_reader", level="WARNING"):
            t.resolve_camera_profiles({"camera_profiles": [1]})

    def test_select_profile(self):
        self.assertIsNone(t.select_camera_profile({}))  # キー無し=従来動作
        self.assertEqual(t.select_camera_profile({"camera_profile": "production"}), "production")
        with self.assertLogs("tablet_ocr_reader", level="WARNING"):
            self.assertEqual(t.select_camera_profile({"camera_profile": "x"}), "demo")
        with self.assertLogs("tablet_ocr_reader", level="WARNING"):
            self.assertEqual(t.select_camera_profile({"camera_profile": None}), "demo")

    def test_missing_config_file_defaults_to_demo(self):
        with self.assertLogs("tablet_ocr_reader", level="WARNING"):
            cfg = t.load_config(os.path.join(tempfile.mkdtemp(), "none.json"))
        self.assertEqual(t.select_camera_profile(cfg), "demo")

    def test_legacy_params_keep_width_height(self):
        lp = t.legacy_open_params({"width": 1280, "height": 960})
        self.assertEqual((lp["width"], lp["height"], lp["size_fallback"]), (1280, 960, False))
        self.assertEqual(t.DEFAULT_CONFIG["width"], 1280)  # 従来のキーはそのまま

    def test_size_candidates(self):
        self.assertEqual(t.camera_size_candidates(1280, 960), [(1280, 960), (1280, 720), (800, 600), (640, 480)])
        self.assertEqual(t.camera_size_candidates(800, 600), [(800, 600), (640, 480)])


class OpenCameraExTest(unittest.TestCase):
    def _open(self, cap, **kw):
        FakeCV2.cap_factory = staticmethod(lambda: cap)
        args = dict(width=1280, height=960, fourcc="MJPG", capture_fps=12, size_fallback=True)
        args.update(kw)
        return t.open_camera_ex(FakeCV2, "fake0", **args)

    def test_gets_requested_size(self):
        cap, info = self._open(FakeCap())
        self.assertEqual((info["width"], info["height"]), (1280, 960))
        self.assertFalse(info["fallback"])
        self.assertEqual(info["fourcc"], "MJPG")
        self.assertEqual(info["fps"], 12.0)

    def test_falls_back_to_next_size(self):
        cap, info = self._open(FakeCap(max_size=(800, 600)))
        self.assertEqual((info["width"], info["height"]), (800, 600))
        self.assertTrue(info["fallback"])
        self.assertEqual((info["requested"]["width"], info["requested"]["height"]), (1280, 960))

    def test_no_fallback_in_legacy_mode_keeps_whatever_driver_gives(self):
        cap, info = self._open(FakeCap(max_size=(800, 600)), size_fallback=False)
        self.assertEqual((info["width"], info["height"]), (800, 600))

    def test_all_reads_fail_raises(self):
        c = FakeCap()
        c.fail_read = True
        with self.assertRaises(RuntimeError):
            self._open(c)
        self.assertTrue(c.released)

    def test_sharpness_max_picks_driver_maximum(self):
        cap, info = self._open(FakeCap(sharpness_range=(0, 6)), sharpness="max")
        self.assertTrue(info["sharpness"]["applied"])
        self.assertEqual(info["sharpness"]["readback"], 6.0)
        self.assertEqual(info["sharpness"]["before"], 3.0)

    def test_sharpness_unsupported_continues(self):
        cap, info = self._open(FakeCap(sharpness_range=None), sharpness="max")
        self.assertFalse(info["sharpness"]["applied"])
        self.assertEqual((info["width"], info["height"]), (1280, 960))

    def test_sharpness_numeric_readback(self):
        cap, info = self._open(FakeCap(sharpness_range=(0, 6)), sharpness=4)
        self.assertTrue(info["sharpness"]["applied"])
        cap, info = self._open(FakeCap(sharpness_range=(0, 6)), sharpness=99)
        self.assertFalse(info["sharpness"]["applied"])


def _fake_opener_factory(behaviors):
    """behaviors: 呼び出し順に 'ok' / 'fail' / (w,h) を返すopener。"""
    calls = []

    def opener(cv2, index, params):
        calls.append(dict(params))
        b = behaviors.pop(0)
        if b == "fail":
            raise RuntimeError("open failed")
        cap = FakeCap()
        return cap, {"width": params["width"], "height": params["height"], "fps": 12.0, "fourcc": "MJPG",
                     "sharpness": {"applied": True, "readback": 6.0, "before": 3.0}
                     if params.get("sharpness") is not None else None}

    opener.calls = calls
    return opener


class ControllerTest(unittest.TestCase):
    P = t.DEFAULT_CAMERA_PROFILES

    def test_switch_success_releases_old_camera(self):
        op = _fake_opener_factory(["ok", "ok"])
        ctl = t.CameraController(None, "auto", op)
        ctl.open(dict(self.P["demo"]))
        old = ctl.cap
        self.assertTrue(ctl.switch(dict(self.P["production"], size_fallback=True)))
        self.assertTrue(old.released)
        self.assertEqual(ctl.info["width"], 1280)

    def test_switch_failure_restores_old_settings(self):
        op = _fake_opener_factory(["ok", "fail", "ok"])
        ctl = t.CameraController(None, "auto", op)
        ctl.open(dict(self.P["demo"]))
        self.assertFalse(ctl.switch(dict(self.P["production"])))
        self.assertIsNotNone(ctl.cap)
        self.assertEqual(ctl.info["width"], 800)  # 元(demo)へ戻っている
        self.assertIn("open failed", ctl.last_error)

    def test_both_fail_then_recover(self):
        op = _fake_opener_factory(["ok", "fail", "fail", "ok"])
        ctl = t.CameraController(None, "auto", op)
        ctl.open(dict(self.P["demo"]))
        self.assertFalse(ctl.switch(dict(self.P["production"])))
        self.assertIsNone(ctl.cap)
        self.assertTrue(ctl.recover())
        self.assertIsNotNone(ctl.cap)

    def test_sharpness_restored_when_leaving_production(self):
        op = _fake_opener_factory(["ok", "ok"])
        ctl = t.CameraController(None, "auto", op)
        ctl.open(dict(self.P["production"], size_fallback=True))
        ctl.switch(dict(self.P["demo"], size_fallback=True))
        self.assertEqual(op.calls[1]["sharpness"], 3.0)  # 変更前の値へ戻す


class ApplySwitchTest(unittest.TestCase):
    def _setup(self, behaviors):
        op = _fake_opener_factory(["ok"] + behaviors)
        ctl = t.CameraController(None, "auto", op)
        ctl.open(dict(t.DEFAULT_CAMERA_PROFILES["demo"]))
        st = t.PreviewState()
        st.init_camera("demo", t.resolve_camera_profiles({}), ctl.info)
        return st, ctl

    def test_no_request_is_noop(self):
        st, ctl = self._setup([])
        self.assertFalse(t.apply_camera_switch(st, ctl))

    def test_same_profile_is_not_requested(self):
        st, ctl = self._setup([])
        self.assertFalse(st.request_camera_profile("demo"))

    def test_success_transition(self):
        st, ctl = self._setup(["ok"])
        st.update_dynamic_rois({"A": t.Roi(0, 0, 1, 1)})
        st.update_raw_frame(np.zeros((4, 4, 3), np.uint8))
        saved = []
        self.assertTrue(st.request_camera_profile("production"))
        s = st.snapshot_settings()
        self.assertEqual(s["camera_profile"], "production")  # 要求直後から表示は新しい方
        self.assertTrue(s["camera_switching"])
        self.assertTrue(t.apply_camera_switch(st, ctl, saved.append))
        s = st.snapshot_settings()
        self.assertEqual((s["camera_profile"], s["camera_switching"], s["camera_error"]), ("production", False, None))
        self.assertEqual((s["camera_actual"]["width"], s["camera_actual"]["height"]), (1280, 960))
        self.assertEqual(st.get_camera_params()["preview_fps"], 1.0)
        self.assertEqual(st.snapshot_dynamic_rois(), {})
        self.assertIsNone(st.get_raw_frame())
        self.assertTrue(st.consume_relock_request())
        self.assertEqual(saved, ["production"])

    def test_failure_keeps_old_profile_and_reports(self):
        st, ctl = self._setup(["fail", "ok"])
        saved = []
        st.request_camera_profile("production")
        t.apply_camera_switch(st, ctl, saved.append)
        s = st.snapshot_settings()
        self.assertEqual(s["camera_profile"], "demo")
        self.assertFalse(s["camera_switching"])
        self.assertIn("open failed", s["camera_error"])
        self.assertEqual(saved, [])
        self.assertEqual(s["camera_actual"]["width"], 800)

    def test_persist_error_does_not_raise(self):
        st, ctl = self._setup(["ok"])
        st.request_camera_profile("production")

        def boom(name):
            raise OSError("disk")

        with self.assertLogs("tablet_ocr_reader", level="WARNING"):
            t.apply_camera_switch(st, ctl, boom)
        self.assertEqual(st.snapshot_settings()["camera_profile"], "production")

    def test_legacy_state_reports_no_profile(self):
        st = t.PreviewState()
        st.init_camera(None, t.resolve_camera_profiles({}), {"width": 1280, "height": 960, "fps": None, "fourcc": "MJPG"})
        s = st.snapshot_settings()
        self.assertIsNone(s["camera_profile"])
        self.assertIsNone(st.get_camera_params())  # preview_*等は従来のキーを使う
        self.assertEqual(set(s["camera_profiles"]), {"demo", "production"})


class CameraApiTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.cfg = os.path.join(self.dir, "cfg.json")
        with open(self.cfg, "w", encoding="utf-8") as f:
            json.dump({"auth_password": "", "camera_index": "auto", "width": 1280, "height": 960}, f)
        self.state = t.PreviewState()
        self.state.init_camera("demo", t.resolve_camera_profiles({}), {"width": 800, "height": 600, "fps": 30.0, "fourcc": "MJPG"})
        self.server = t.start_preview_server(self.state, "127.0.0.1", 0, self.cfg)
        self.base = f"http://127.0.0.1:{self.server.server_address[1]}"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def _post(self, body):
        req = urllib.request.Request(self.base + "/api/settings", data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.load(r)
        except urllib.error.HTTPError as e:
            e.close()
            return e.code, None

    def test_get_contract(self):
        with urllib.request.urlopen(self.base + "/api/settings.json", timeout=5) as r:
            d = json.load(r)
        self.assertEqual(d["camera_profile"], "demo")
        self.assertEqual(set(d["camera_profiles"]), {"demo", "production"})
        self.assertEqual(d["camera_actual"]["width"], 800)
        self.assertIn("fps", d["camera_actual"])
        self.assertIn("fourcc", d["camera_actual"])
        self.assertEqual(d["frame_rotate_deg"], 0)
        self.assertIn("ocr_scale_factor", d)

    def test_post_valid_returns_new_settings(self):
        code, body = self._post({"camera_profile": "production"})
        self.assertEqual(code, 200)
        self.assertEqual(body["camera_profile"], "production")
        self.assertTrue(body["ok"])
        self.assertIsNotNone(self.state.take_camera_request())

    def test_post_invalid_rejected_nothing_changes(self):
        for bad in ("x", "Demo", 1, None, True, ["demo"], ""):
            with self.subTest(bad=bad):
                self.assertEqual(self._post({"camera_profile": bad})[0], 400)
        self.assertIsNone(self.state.take_camera_request())
        self.assertEqual(self.state.snapshot_settings()["camera_profile"], "demo")

    def test_combined_with_rotation_and_invalid_combo_changes_nothing(self):
        code, body = self._post({"camera_profile": "production", "frame_rotate_deg": 270})
        self.assertEqual(code, 200)
        self.assertEqual((body["camera_profile"], body["frame_rotate_deg"]), ("production", 270))
        self.assertEqual(self._post({"camera_profile": "demo", "frame_rotate_deg": 45})[0], 400)
        self.assertEqual(self.state.snapshot_settings()["camera_profile"], "production")  # 不正な組は無視

    def test_persisted_by_switch_not_by_post(self):
        # 設定ファイルへの書き戻しは、切替が成功した後(apply_camera_switch)に行う
        self._post({"camera_profile": "production"})
        with open(self.cfg, encoding="utf-8") as f:
            self.assertNotIn("camera_profile", json.load(f))
        op = _fake_opener_factory(["ok", "ok"])
        ctl = t.CameraController(None, "auto", op)
        ctl.open(dict(t.DEFAULT_CAMERA_PROFILES["demo"]))
        t.apply_camera_switch(self.state, ctl, lambda n: t.save_config_updates(self.cfg, {"camera_profile": n}))
        with open(self.cfg, encoding="utf-8") as f:
            saved = json.load(f)
        self.assertEqual(saved["camera_profile"], "production")
        self.assertEqual(saved["width"], 1280)  # 既存の項目は保たれる
        self.assertEqual(t.select_camera_profile(t.load_config(self.cfg)), "production")


if __name__ == "__main__":
    unittest.main()
