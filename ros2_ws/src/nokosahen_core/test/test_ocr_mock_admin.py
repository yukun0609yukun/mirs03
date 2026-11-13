"""模擬POS(OCRテスト)の数字変更 と OCR画質プロファイル切替(管理画面)のテスト。

rclpy不要、ロボットにも実機にも何も送らない。
1. ラズパイ側 scripts/raspi_display/kiosk_admin.py: POST /ocr_mock のボディ検証・送信元制限・
   ocrtestモード以外は409・ocr_cmd.json の原子的書込み・seqの単調増加・GET /state の ocr_mock。
2. 1台目側 survey_admin_proxy.py の set_ocr_mock と、ダッシュボードの POST /api/survey/ocr_mock
   (guest 403、未ログイン401、不正400、ラズパイの409理由の伝達)。
3. 1台目側 ocr_admin_proxy.py(OCR API契約: GET /api/settings.json + /api/readings.json、
   POST /api/settings {"camera_profile"})と、ダッシュボードの /api/ocr/state・POST /api/ocr/profile。
4. 管理画面のJS構文(node --check)と、カード/ID の存在、緊急停止ドックが残っていること。
5. ラッパー ocr_test/mock_pos_tablet.html のコマンド実行ロジックを実ブラウザ(Playwright + Edge)で確認
   (Playwright/Edgeが無い環境では skip)。

実行: python ros2_ws/src/nokosahen_core/test/test_ocr_mock_admin.py
"""

import ast
import importlib.util
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import types
import unittest
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest import mock

PKG = Path(__file__).resolve().parents[1]
REPO = PKG.parents[2]
RASPI = REPO / "scripts" / "raspi_display"
OCR_TEST = RASPI / "ocr_test"
sys.path.insert(0, str(PKG))

_TMP = tempfile.mkdtemp(prefix="ocr_mock_admin_test_")
os.environ["SURVEY_DATA_DIR"] = os.path.join(_TMP, "data")
os.environ["KIOSK_MODE_FILE"] = os.path.join(_TMP, "kiosk_mode")
os.environ["OCR_TEST_DIR"] = os.path.join(_TMP, "ocr_test")


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


kiosk_admin = _load("kiosk_admin_ocrm", RASPI / "kiosk_admin.py")


def _install_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m

    class Msg:
        def __init__(self, *a, **k):
            self.data = None

    mod("geometry_msgs")
    mod("geometry_msgs.msg", PoseStamped=Msg, Twist=Msg)
    mod("nav_msgs")
    mod("nav_msgs.msg", Odometry=Msg)
    mod("std_msgs")
    mod("std_msgs.msg", Bool=Msg, Float32MultiArray=Msg, String=Msg, Empty=Msg)


try:
    import geometry_msgs.msg  # noqa: F401
    import nav_msgs.msg  # noqa: F401
    import std_msgs.msg as _std  # noqa: F401
except Exception:  # noqa: BLE001
    _install_stubs()

from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core.ocr_admin_proxy import (  # noqa: E402
    OcrAdminProxy, summarize_readings, validate_profile_request)
from nokosahen_core.survey_admin_proxy import (  # noqa: E402
    SurveyAdminProxy, validate_ocr_mock_request)


def http(method, url, body=None, raw=None, headers=None, timeout=10):
    data = raw if raw is not None else (None if body is None else json.dumps(body).encode())
    req = urllib.request.Request(url, data=data, method=method, headers=headers or {})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, r.read(), dict(r.headers)
    except urllib.error.HTTPError as e:
        try:
            return e.code, e.read(), dict(e.headers)
        finally:
            e.close()


def jhttp(*a, **k):
    code, body, _ = http(*a, **k)
    return code, json.loads(body.decode("utf-8"))


def serve(handler):
    srv = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, srv.server_address[1]


VALID7 = [1, 2, 3, 4, 5, 6, 7]


# ======================================================================
# 1. kiosk_admin.py
# ======================================================================
class OcrMockParseTest(unittest.TestCase):
    def parse(self, obj):
        raw = obj if isinstance(obj, bytes) else json.dumps(obj).encode()
        return kiosk_admin.parse_ocr_mock_body(raw)

    def test_valid(self):
        self.assertEqual(self.parse({"action": "randomize"}), ("randomize", None, None))
        self.assertEqual(self.parse({"action": "reset"}), ("reset", None, None))
        self.assertEqual(self.parse({"action": "set", "values": VALID7}), ("set", VALID7, None))
        self.assertEqual(self.parse({"action": "set", "values": [0, 9999, 0, 9999, 0, 9999, 0]})[2], None)

    def test_invalid(self):
        bad = [
            {}, [], "x", {"action": "delete"}, {"action": 1}, {"action": None},
            {"action": "randomize", "values": VALID7},          # randomize/resetにvaluesは付けない
            {"action": "reset", "extra": 1},
            {"action": "set"}, {"action": "set", "values": VALID7, "x": 1},
            {"action": "set", "values": VALID7[:6]}, {"action": "set", "values": VALID7 + [8]},
            {"action": "set", "values": "1234567"},
            {"action": "set", "values": [1, 2, 3, 4, 5, 6, -1]},
            {"action": "set", "values": [1, 2, 3, 4, 5, 6, 10000]},
            {"action": "set", "values": [1, 2, 3, 4, 5, 6, 1.5]},
            {"action": "set", "values": [1, 2, 3, 4, 5, 6, "7"]},
            {"action": "set", "values": [1, 2, 3, 4, 5, 6, True]},
            {"action": "set", "values": [1, 2, 3, 4, 5, 6, None]},
        ]
        for b in bad:
            self.assertIsNotNone(self.parse(b)[2], b)
        self.assertIsNotNone(self.parse(b"not json")[2])
        self.assertIsNotNone(self.parse(b"\xff\xfe")[2])
        self.assertIsNotNone(self.parse(b'{"action":"reset","pad":"' + b"x" * 600 + b'"}')[2])  # 上限超過
        self.assertIsNotNone(kiosk_admin.parse_ocr_mock_body(None)[2])


class OcrCmdWriteTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(dir=_TMP)
        self.file = os.path.join(self.dir, "ocr_cmd.json")
        self._p1 = mock.patch.object(kiosk_admin, "OCR_DIR", self.dir)
        self._p2 = mock.patch.object(kiosk_admin, "OCR_CMD_FILE", self.file)
        self._p1.start()
        self._p2.start()

    def tearDown(self):
        self._p1.stop()
        self._p2.stop()

    def test_writes_json_and_no_tmp_left(self):
        c = kiosk_admin.write_ocr_cmd("set", VALID7, now=1000.5)
        d = json.loads(Path(self.file).read_text(encoding="utf-8"))
        self.assertEqual(d, c)
        self.assertEqual((d["seq"], d["action"], d["values"], d["at"]), (1000500, "set", VALID7, 1000))
        self.assertEqual(os.listdir(self.dir), ["ocr_cmd.json"])
        d2 = kiosk_admin.write_ocr_cmd("randomize", None, now=1001)
        self.assertNotIn("values", d2)

    def test_seq_strictly_increases_even_with_same_or_earlier_clock(self):
        a = kiosk_admin.write_ocr_cmd("reset", None, now=500)["seq"]
        b = kiosk_admin.write_ocr_cmd("reset", None, now=500)["seq"]
        c = kiosk_admin.write_ocr_cmd("reset", None, now=400)["seq"]   # 時計が戻っても増える
        self.assertTrue(a < b < c)

    def test_atomic_replace_old_file_survives_failure(self):
        kiosk_admin.write_ocr_cmd("reset", None, now=10)
        before = Path(self.file).read_text(encoding="utf-8")
        seen = {}

        def boom(src, dst):
            seen["tmp_exists"] = os.path.exists(src)
            seen["tmp_valid"] = json.loads(Path(src).read_text(encoding="utf-8"))["action"] == "randomize"
            raise OSError("disk")

        with mock.patch.object(kiosk_admin.os, "replace", boom):
            with self.assertRaises(OSError):
                kiosk_admin.write_ocr_cmd("randomize", None, now=11)
        self.assertEqual((seen["tmp_exists"], seen["tmp_valid"]), (True, True))   # 一時ファイルは完全な形で書かれてからrename
        self.assertEqual(Path(self.file).read_text(encoding="utf-8"), before)    # 本体は壊れない

    def test_summary(self):
        self.assertIsNone(kiosk_admin.ocr_mock_summary())
        Path(self.file).write_text("{broken", encoding="utf-8")
        self.assertIsNone(kiosk_admin.ocr_mock_summary())
        kiosk_admin.write_ocr_cmd("reset", None, now=77)
        self.assertEqual(kiosk_admin.ocr_mock_summary(), {"seq": 77000, "action": "reset", "at": 77})


class OcrMockServerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv, cls.port = serve(kiosk_admin.Handler)
        cls.base = f"http://127.0.0.1:{cls.port}"

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def setUp(self):
        self._allowed = set(kiosk_admin.ALLOWED_CLIENTS)
        kiosk_admin.ALLOWED_CLIENTS = {"127.0.0.1"}
        kiosk_admin.RECEIVER = "http://127.0.0.1:9"   # receiverは不在(fallback_state)
        shutil.rmtree(kiosk_admin.OCR_DIR, ignore_errors=True)
        Path(kiosk_admin.MODE_FILE).write_text("ocrtest\n", encoding="utf-8")

    def tearDown(self):
        kiosk_admin.ALLOWED_CLIENTS = self._allowed

    def post(self, body, raw=None):
        return jhttp("POST", self.base + "/ocr_mock", body=body, raw=raw)

    def test_forbidden_for_other_sources(self):
        kiosk_admin.ALLOWED_CLIENTS = {"198.51.100.2"}
        self.assertEqual(self.post({"action": "reset"})[0], 403)
        self.assertFalse(os.path.exists(kiosk_admin.OCR_CMD_FILE))

    def test_ok_writes_command(self):
        code, d = self.post({"action": "set", "values": VALID7})
        self.assertEqual((code, d["ok"]), (200, True))
        f = json.loads(Path(kiosk_admin.OCR_CMD_FILE).read_text(encoding="utf-8"))
        self.assertEqual((f["action"], f["values"], f["seq"]), ("set", VALID7, d["ocr_mock"]["seq"]))
        code, d2 = self.post({"action": "randomize"})
        self.assertEqual(code, 200)
        self.assertGreater(d2["ocr_mock"]["seq"], d["ocr_mock"]["seq"])

    def test_state_has_ocr_mock(self):
        self.assertIsNone(jhttp("GET", self.base + "/state")[1]["ocr_mock"])
        _, d = self.post({"action": "reset"})
        st = jhttp("GET", self.base + "/state")[1]
        self.assertEqual(st["ocr_mock"]["seq"], d["ocr_mock"]["seq"])
        self.assertEqual(st["ocr_mock"]["action"], "reset")

    def test_409_when_not_ocrtest_and_nothing_written(self):
        for m in ("customer", "survey"):
            Path(kiosk_admin.MODE_FILE).write_text(m + "\n", encoding="utf-8")
            code, d = self.post({"action": "randomize"})
            self.assertEqual((code, d["ok"]), (409, False))
            self.assertIn("OCRテスト", d["message"])
        self.assertFalse(os.path.exists(kiosk_admin.OCR_CMD_FILE))

    def test_bad_input_400_before_mode_check(self):
        Path(kiosk_admin.MODE_FILE).write_text("customer\n", encoding="utf-8")
        for b in ({}, {"action": "x"}, {"action": "set", "values": [1]}, {"action": "reset", "values": VALID7}):
            self.assertEqual(self.post(b)[0], 400, b)
        self.assertEqual(self.post(None, raw=b"nope")[0], 400)
        self.assertEqual(self.post(None, raw=b"{" + b" " * 600 + b"}")[0], 400)
        self.assertFalse(os.path.exists(kiosk_admin.OCR_CMD_FILE))

    def test_mode_endpoint_unaffected(self):
        self.assertEqual(jhttp("POST", self.base + "/mode", body={"action": "reset"})[0], 400)
        self.assertEqual(jhttp("POST", self.base + "/nope", body={})[0], 404)


# ======================================================================
# 2./3. 1台目側プロキシとダッシュボードAPI(ラズパイ・OCRはスタブ)
# ======================================================================
class _PiStub(BaseHTTPRequestHandler):
    mode = "ocrtest"
    hits = []

    def log_message(self, *a):
        pass

    def _send(self, code, obj):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        self._send(200, {"mode": _PiStub.mode, "count": 0, "active": False, "receiver": True})

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(n).decode())
        _PiStub.hits.append((self.path, body))
        if self.path == "/ocr_mock" and _PiStub.mode != "ocrtest":
            return self._send(409, {"ok": False, "message": "OCRテストモードではありません(スタブ)"})
        self._send(200, {"ok": True, "ocr_mock": {"seq": 5, "action": body.get("action"), "at": 1}})


class _OcrStub(BaseHTTPRequestHandler):
    """tablet_ocr_reader.py のAPI契約(GET /api/settings.json, /api/readings.json, POST /api/settings)"""
    settings = {}
    readings = {}
    fail = 0
    garbage = False
    readings_fail = False
    hits = []

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        if not isinstance(body, bytes):
            body = json.dumps(body, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        _OcrStub.hits.append(("GET", self.path))
        if _OcrStub.fail:
            return self._send(_OcrStub.fail, {})
        if _OcrStub.garbage:
            return self._send(200, b"not json")
        if self.path == "/api/settings.json":
            return self._send(200, _OcrStub.settings)
        if self.path == "/api/readings.json":
            if _OcrStub.readings_fail:
                return self._send(500, {})
            return self._send(200, _OcrStub.readings)
        self._send(404, {})

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(n).decode())
        _OcrStub.hits.append(("POST", self.path, body))
        if _OcrStub.fail:
            return self._send(_OcrStub.fail, {})
        if self.path != "/api/settings":
            return self._send(404, {})
        _OcrStub.settings = dict(_OcrStub.settings, camera_profile=body["camera_profile"])
        self._send(200, {"ok": True})


def _reset_ocr_stub():
    _OcrStub.settings = {
        "ocr_scale_factor": 2.0, "frame_rotate_deg": 90, "camera_profile": "demo",
        "camera_profiles": {
            "demo": {"width": 1280, "height": 720, "capture_fps": 30, "preview_fps": 10, "poll_interval_sec": 1.0},
            "production": {"width": 3840, "height": 2160, "capture_fps": 1, "preview_fps": 1, "poll_interval_sec": 5.0},
        },
        "camera_actual": {"width": 1280, "height": 720, "fps": 29.8, "fourcc": "MJPG"},
    }
    _OcrStub.readings = {
        "updated_at": time.time() - 3,
        "readings": {
            "A": {"raw_value": 118, "confirmed_value": 118, "menu_name": "Aランチ", "sent": True, "estimated": False},
            "B": {"raw_value": 12, "confirmed_value": 129, "menu_name": "Bランチ", "sent": True, "estimated": True},
            "curry": {"raw_value": None, "confirmed_value": None, "menu_name": None, "sent": False, "estimated": False},
        },
        "roi_lock": {"mode": "auto_lock", "locked": True, "progress": 5, "required": 5},
        "frame_rotate_deg": 90,
    }
    _OcrStub.fail = 0
    _OcrStub.garbage = False
    _OcrStub.readings_fail = False
    _OcrStub.hits = []


class SurveyProxyOcrMockTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv, cls.port = serve(_PiStub)
        cls.url = f"http://127.0.0.1:{cls.port}"

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def setUp(self):
        _PiStub.mode = "ocrtest"
        _PiStub.hits = []

    def test_validate_matches_pi_rules(self):
        self.assertEqual(validate_ocr_mock_request({"action": "reset"}), ({"action": "reset"}, ""))
        self.assertEqual(validate_ocr_mock_request({"action": "set", "values": VALID7})[0],
                         {"action": "set", "values": VALID7})
        for b in (None, {}, {"action": "x"}, {"action": "reset", "values": VALID7}, {"action": "set"},
                  {"action": "set", "values": VALID7[:3]}, {"action": "set", "values": VALID7[:6] + [10000]},
                  {"action": "set", "values": VALID7[:6] + [True]}, {"action": "set", "values": VALID7[:6] + [1.0]},
                  {"action": "set", "values": VALID7, "x": 1}):
            self.assertIsNone(validate_ocr_mock_request(b)[0], b)

    def test_forward_and_reject(self):
        p = SurveyAdminProxy(self.url)
        code, d = p.set_ocr_mock({"action": "set", "values": VALID7})
        self.assertEqual((code, d["ok"]), (200, True))
        self.assertEqual(_PiStub.hits, [("/ocr_mock", {"action": "set", "values": VALID7})])
        self.assertEqual(p.set_ocr_mock({"action": "bogus"})[0], 400)
        self.assertEqual(len(_PiStub.hits), 1)    # 不正はラズパイへ届かない

    def test_pi_409_reason_is_passed_on(self):
        _PiStub.mode = "customer"
        code, d = SurveyAdminProxy(self.url).set_ocr_mock({"action": "randomize"})
        self.assertEqual((code, d["ok"]), (200, False))
        self.assertIn("OCRテストモードではありません", d["message"])

    def test_unreachable_and_unconfigured(self):
        code, d = SurveyAdminProxy("http://127.0.0.1:9").set_ocr_mock({"action": "reset"})
        self.assertEqual((code, d["ok"]), (200, False))
        self.assertIn("届きません", d["message"])
        self.assertFalse(SurveyAdminProxy("").set_ocr_mock({"action": "reset"})[1]["configured"])


class OcrProxyContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv, cls.port = serve(_OcrStub)
        cls.url = f"http://127.0.0.1:{cls.port}"

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def setUp(self):
        _reset_ocr_stub()

    def test_unconfigured(self):
        p = OcrAdminProxy("")
        self.assertFalse(p.configured)
        self.assertFalse(p.get_state()["configured"])
        self.assertFalse(p.set_profile({"camera_profile": "demo"})[1]["configured"])

    def test_state_summary(self):
        d = OcrAdminProxy(self.url).get_state()
        self.assertTrue(d["ok"])
        s = d["state"]
        self.assertEqual((s["profile"], s["profile_supported"], s["frame_rotate_deg"]), ("demo", True, 90))
        self.assertEqual(s["camera_actual"]["width"], 1280)
        self.assertEqual(s["profiles"]["production"]["poll_interval_sec"], 5.0)
        self.assertEqual(len(s["readings"]), 7)                       # 固定の7スロット(欠けは seen:false)
        self.assertEqual([r["slot"] for r in s["readings"]][:4], ["A", "B", "C", "curry"])
        a = s["readings"][0]
        self.assertEqual((a["confirmed"], a["raw"], a["sent"], a["estimated"], a["seen"]), (118, 118, True, False, True))
        self.assertTrue(s["readings"][1]["estimated"])
        self.assertFalse(s["readings"][2]["seen"])                    # C は未報告
        self.assertTrue(s["roi_lock"]["locked"])
        self.assertGreaterEqual(s["updated_age_s"], 2.0)
        # 画像は取りに行かない(OCRが「閲覧中」とみなして負荷が上がるため)
        self.assertEqual({h[1] for h in _OcrStub.hits}, {"/api/settings.json", "/api/readings.json"})

    def test_old_ocr_without_profile_support(self):
        _OcrStub.settings = {"ocr_scale_factor": 2.0, "frame_rotate_deg": 0}
        s = OcrAdminProxy(self.url).get_state()["state"]
        self.assertEqual((s["profile"], s["profile_supported"], s["profiles"]), (None, False, {}))

    def test_readings_failure_keeps_settings(self):
        _OcrStub.readings_fail = True
        d = OcrAdminProxy(self.url).get_state()
        self.assertTrue(d["ok"])
        self.assertIsNone(d["state"]["readings"])
        self.assertIn("読み取り結果", d["state"]["readings_error"])

    def test_cache_and_errors(self):
        clock = [0.0]
        p = OcrAdminProxy(self.url, clock=lambda: clock[0])
        p.get_state()
        n = len(_OcrStub.hits)
        p.get_state()
        self.assertEqual(len(_OcrStub.hits), n)       # TTL内はOCRへ問い合わせない
        clock[0] = 5.0
        _OcrStub.fail = 500
        d = p.get_state()
        self.assertEqual(d["ok"], False)
        self.assertIn("HTTP 500", d["message"])
        _OcrStub.fail = 0
        _OcrStub.garbage = True
        clock[0] = 10.0
        self.assertFalse(p.get_state()["ok"])
        d = OcrAdminProxy("http://127.0.0.1:9", timeout_s=1).get_state()
        self.assertEqual((d["ok"], d["configured"]), (False, True))
        self.assertIn("届きません", d["message"])

    def test_set_profile_forward_validation_and_cache_drop(self):
        p = OcrAdminProxy(self.url)
        self.assertEqual(p.get_state()["state"]["profile"], "demo")
        code, d = p.set_profile({"camera_profile": "production"})
        self.assertEqual((code, d["ok"], d["profile"]), (200, True, "production"))
        self.assertIn(("POST", "/api/settings", {"camera_profile": "production"}), _OcrStub.hits)
        self.assertEqual(p.get_state()["state"]["profile"], "production")   # キャッシュは捨てられている
        n = len([h for h in _OcrStub.hits if h[0] == "POST"])
        for b in (None, {}, {"camera_profile": "fast"}, {"camera_profile": 1}, {"camera_profile": "demo", "x": 1},
                  {"profile": "demo"}, {"camera_profile": ["demo"]}):
            self.assertEqual(p.set_profile(b)[0], 400, b)
        self.assertEqual(len([h for h in _OcrStub.hits if h[0] == "POST"]), n)   # 不正はOCRへ届かない

    def test_set_profile_failures_are_reported(self):
        _OcrStub.fail = 400
        code, d = OcrAdminProxy(self.url).set_profile({"camera_profile": "demo"})
        self.assertEqual((code, d["ok"]), (200, False))
        self.assertIn("400", d["message"])
        code, d = OcrAdminProxy("http://127.0.0.1:9").set_profile({"camera_profile": "demo"})
        self.assertEqual((code, d["ok"]), (200, False))

    def test_validate_and_summarize(self):
        self.assertEqual(validate_profile_request({"camera_profile": "demo"}), ("demo", ""))
        self.assertIsNone(validate_profile_request({"camera_profile": "x"})[0])
        self.assertIsNone(summarize_readings({}))
        self.assertIsNone(summarize_readings({"readings": []}))
        extra = summarize_readings({"readings": {"zzz": {"raw_value": 1, "confirmed_value": 2, "sent": False}}})
        self.assertEqual(extra[-1]["slot"], "zzz")                    # 未知のスロットも落とさない


class FakeTeleop:
    soft_estop_engaged = False


class DashboardApiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pi, cls.pi_port = serve(_PiStub)
        cls.ocr, cls.ocr_port = serve(_OcrStub)

    @classmethod
    def tearDownClass(cls):
        for s in (cls.pi, cls.ocr):
            s.shutdown()
            s.server_close()

    def setUp(self):
        _reset_ocr_stub()
        _PiStub.mode = "ocrtest"
        _PiStub.hits = []
        self.survey = SurveyAdminProxy(f"http://127.0.0.1:{self.pi_port}")
        self.ocr_proxy = OcrAdminProxy(f"http://127.0.0.1:{self.ocr_port}")
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, FakeTeleop(), survey_admin=self.survey, ocr_admin=self.ocr_proxy,
            auth_username="TL", auth_password="pw", enforce_single_controller=False)
        self.srv, self.port = serve(handler)
        self.tl = self.login("TL", "pw")
        self.guest = self.login("guest", "")

    def tearDown(self):
        self.srv.shutdown()
        self.srv.server_close()

    def login(self, user, pw):
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.port}/api/login", data=json.dumps({"username": user, "password": pw}).encode(),
            method="POST", headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.headers.get("Set-Cookie", "").split(";")[0]

    def call(self, method, path, body=None, cookie=None):
        h = {"Content-Type": "application/json"}
        c = self.tl if cookie is None else cookie
        if c:
            h["Cookie"] = c
        code, b, _ = http(method, f"http://127.0.0.1:{self.port}{path}", body=body, headers=h)
        return code, json.loads(b.decode("utf-8"))

    # ---- 模擬POS ----
    def test_ocr_mock_requires_login_and_tl_only(self):
        body = {"action": "set", "values": VALID7}
        self.assertEqual(self.call("POST", "/api/survey/ocr_mock", body, cookie="")[0], 401)
        self.assertEqual(self.call("POST", "/api/survey/ocr_mock", body, cookie=self.guest)[0], 403)
        self.assertEqual(_PiStub.hits, [])
        code, d = self.call("POST", "/api/survey/ocr_mock", body)
        self.assertEqual((code, d["ok"]), (200, True))
        self.assertEqual(_PiStub.hits, [("/ocr_mock", body)])

    def test_ocr_mock_invalid_400_and_409_reason(self):
        for b in ({}, {"action": "x"}, {"action": "set", "values": [1]}, {"action": "reset", "values": VALID7}):
            self.assertEqual(self.call("POST", "/api/survey/ocr_mock", b)[0], 400, b)
        self.assertEqual(_PiStub.hits, [])
        _PiStub.mode = "customer"
        code, d = self.call("POST", "/api/survey/ocr_mock", {"action": "randomize"})
        self.assertEqual((code, d["ok"]), (200, False))
        self.assertIn("OCRテストモードではありません", d["message"])

    def test_ocr_mock_pi_down(self):
        self.survey.base_url = "http://127.0.0.1:9"
        code, d = self.call("POST", "/api/survey/ocr_mock", {"action": "reset"})
        self.assertEqual((code, d["ok"]), (200, False))

    # ---- OCR ----
    def test_ocr_state_login_and_guest_can_view(self):
        self.assertEqual(self.call("GET", "/api/ocr/state", cookie="")[0], 401)
        for ck in (self.tl, self.guest):
            code, d = self.call("GET", "/api/ocr/state", cookie=ck)
            self.assertEqual((code, d["ok"], d["state"]["profile"]), (200, True, "demo"))

    def test_ocr_profile_tl_only_and_validation(self):
        body = {"camera_profile": "production"}
        self.assertEqual(self.call("POST", "/api/ocr/profile", body, cookie="")[0], 401)
        self.assertEqual(self.call("POST", "/api/ocr/profile", body, cookie=self.guest)[0], 403)
        self.assertEqual(self.call("POST", "/api/ocr/profile", {"camera_profile": "bogus"})[0], 400)
        self.assertEqual(self.call("POST", "/api/ocr/profile", {})[0], 400)
        self.assertEqual([h for h in _OcrStub.hits if h[0] == "POST"], [])
        code, d = self.call("POST", "/api/ocr/profile", body)
        self.assertEqual((code, d["ok"]), (200, True))
        self.assertEqual(_OcrStub.settings["camera_profile"], "production")

    def test_ocr_down_is_error_json_not_exception(self):
        self.ocr_proxy.base_url = "http://127.0.0.1:9"
        self.ocr_proxy.timeout_s = 1.0
        self.ocr_proxy._cache = None
        code, d = self.call("GET", "/api/ocr/state")
        self.assertEqual((code, d["ok"]), (200, False))
        self.assertIn("届きません", d["message"])
        code, d = self.call("POST", "/api/ocr/profile", {"camera_profile": "demo"})
        self.assertEqual((code, d["ok"]), (200, False))

    def test_unconfigured_hides_cards(self):
        self.ocr_proxy.base_url = ""
        self.assertFalse(self.call("GET", "/api/ocr/state")[1]["configured"])


# ======================================================================
# 4. 画面(HTML/JS)
# ======================================================================
def _dashboard_html():
    """full_dashboard_node.py の総合ダッシュボードHTML(rclpyをimportせず、ASTから文字列定数を取り出す)"""
    tree = ast.parse((PKG / "nokosahen_core" / "full_dashboard_node.py").read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str) and 'id="surveyBlock"' in node.value:
            return node.value
    raise AssertionError("dashboard html not found")


class PageTest(unittest.TestCase):
    def test_cards_in_settings_tab_before_estop_dock(self):
        html = _dashboard_html()
        i_settings = html.index('id="tab-settings"')
        for el in ("ocrMockBlock", "ocrBlock"):
            i = html.index(f'id="{el}"')
            self.assertGreater(i, i_settings)
            self.assertLess(i, html.index('id="estopDock"'))
        for el in ("ocrMockRandBtn", "ocrMockResetBtn", "ocrMockSetBtn", "ocrMockHint", "ocrMockWhy",
                   "ocrProfile", "ocrRes", "ocrFps", "ocrInterval", "ocrRoi", "ocrRows", "ocrErr",
                   "ocrToDemoBtn", "ocrToProdBtn", "ocrConfirm", "ocrConfirmYes", "ocrConfirmNo", "ocrLink"):
            self.assertIn(f'id="{el}"', html)
        self.assertEqual(html.count('class="ocrm-in"'), 7)
        for api in ("/api/survey/ocr_mock", "/api/ocr/state", "/api/ocr/profile"):
            self.assertIn(api, html)
        # 【2026-10-21】OCR確認画面(8091)は1台目の内側だけ。管理画面が映像を中継する(直接の:8091リンクは無い)
        self.assertNotIn('":8091/"', html)
        self.assertIn("/api/ocr/preview.jpg", html)
        self.assertIn('id="ocrViewWrap"', html)
        self.assertIn("estopDock", html)

    def test_estop_dock_still_fixed_on_top(self):
        src = (PKG / "nokosahen_core" / "nav_layout.py").read_text(encoding="utf-8")   # ドックのCSSは共通のNAV_CSS
        self.assertRegex(src, r"#estopDock\s*\{[^}]*position:\s*fixed[^}]*z-index:\s*2000")

    def test_dashboard_js_parses(self):
        node = shutil.which("node")
        if not node:
            self.skipTest("node がありません")
        html = _dashboard_html()
        scripts = re.findall(r"<script[^>]*>(.*?)</script>", html, re.S)
        self.assertTrue(scripts)
        js = max(scripts, key=len)
        self.assertIn("loadOcr", js)
        with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False, encoding="utf-8", dir=_TMP) as f:
            f.write(js)
        r = subprocess.run([node, "--check", f.name], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)

    def test_wrapper_js_parses(self):
        node = shutil.which("node")
        if not node:
            self.skipTest("node がありません")
        html = (OCR_TEST / "mock_pos_tablet.html").read_text(encoding="utf-8")
        js = re.findall(r"<script>(.*?)</script>", html, re.S)[-1]
        with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False, encoding="utf-8", dir=_TMP) as f:
            f.write(js)
        r = subprocess.run([node, "--check", f.name], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)

    def test_original_page_unchanged_markers(self):
        # 元ページは変更しない方針: ラッパーが依存するIDとクラスが残っていること
        orig = (OCR_TEST / "mock_pos_tablet_original.html").read_text(encoding="utf-8")
        self.assertIn('id="randomizeBtn"', orig)
        self.assertIn('id="resetBtn"', orig)
        self.assertEqual(orig.count('class="count-input"'), 7)


# ======================================================================
# 5. ラッパーのコマンド実行(実ブラウザ)
# ======================================================================
try:
    from playwright.sync_api import sync_playwright
    _HAVE_PW = True
except Exception:  # noqa: BLE001
    _HAVE_PW = False

DEFAULTS = [118, 129, 353, 166, 211, 178, 103]


@unittest.skipUnless(_HAVE_PW, "playwright がありません")
class WrapperBrowserTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dir = os.path.join(_TMP, "ocr_served")
        shutil.copytree(OCR_TEST, cls.dir)
        s = socket_free_port()
        cls.port = s
        cls.proc = subprocess.Popen(
            [sys.executable, "-m", "http.server", str(s), "--bind", "127.0.0.1", "--directory", cls.dir],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.0)
        cls.pw = sync_playwright().start()
        try:
            cls.browser = cls.pw.chromium.launch(channel="msedge")
        except Exception as e:  # noqa: BLE001
            cls.pw.stop()
            cls.proc.terminate()
            raise unittest.SkipTest(f"Edge を起動できません: {e}")

    @classmethod
    def tearDownClass(cls):
        try:
            cls.browser.close()
            cls.pw.stop()
        finally:
            cls.proc.terminate()

    def setUp(self):
        self.cmd = os.path.join(self.dir, "ocr_cmd.json")
        if os.path.exists(self.cmd):
            os.remove(self.cmd)
        self.ctx = self.browser.new_context(viewport={"width": 800, "height": 480})
        self.ctx.route(re.compile(r"https?://fonts\.(googleapis|gstatic)\.com/.*"), lambda r: r.abort())
        self.page = self.ctx.new_page()

    def tearDown(self):
        self.ctx.close()

    def write(self, obj, raw=None):
        tmp = self.cmd + ".tmp"
        Path(tmp).write_text(raw if raw is not None else json.dumps(obj), encoding="utf-8")
        os.replace(tmp, self.cmd)

    def open(self):
        self.page.goto(f"http://127.0.0.1:{self.port}/mock_pos_tablet.html")
        self.page.wait_for_function(
            "() => { const f = document.getElementById('f'); return f.contentDocument && f.contentDocument.querySelectorAll('.count-input').length === 7; }")

    def values(self):
        return self.page.evaluate(
            "() => Array.from(document.getElementById('f').contentDocument.querySelectorAll('.count-input')).map(e => e.value)")

    def wait_values(self, pred, timeout=4000):
        self.page.wait_for_function(
            "(src) => { const f = document.getElementById('f'); const v = Array.from(f.contentDocument.querySelectorAll('.count-input')).map(e => e.value); return eval(src)(v); }",
            arg=pred, timeout=timeout)
        return self.values()

    def test_commands_apply_in_iframe(self):
        self.write({"seq": 100, "action": "set", "values": [9, 9, 9, 9, 9, 9, 9]})   # 読込前から在るコマンド=処理済み
        self.open()
        time.sleep(2.5)
        self.assertEqual(self.values(), [str(x) for x in DEFAULTS])                  # 再読込で再実行されない

        self.write({"seq": 101, "action": "set", "values": [1, 22, 333, 4444, 0, 9999, 7]})
        self.assertEqual(self.wait_values("v => v[0] === '1'"), ["1", "22", "333", "4444", "0", "9999", "7"])

        self.write({"seq": 102, "action": "randomize"})
        v = self.wait_values("v => v.join() !== '1,22,333,4444,0,9999,7'")
        self.assertTrue(all(x.isdigit() and len(x) <= 3 for x in v), v)              # 元ページの乱数化(1〜3桁)

        self.write({"seq": 103, "action": "reset"})
        self.assertEqual(self.wait_values("v => v[0] === '118'"), [str(x) for x in DEFAULTS])

        # 無視される: 不正(個数違い・範囲外・型違い)・壊れたファイル・seqが増えない再書込み
        for bad in ({"seq": 104, "action": "set", "values": [1, 2, 3]},
                    {"seq": 105, "action": "set", "values": [1, 2, 3, 4, 5, 6, 10000]},
                    {"seq": 106, "action": "set", "values": [1, 2, 3, 4, 5, 6, "7"]},
                    {"seq": 107, "action": "explode"},
                    {"seq": 103, "action": "set", "values": VALID7}):
            self.write(bad)
            time.sleep(1.6)
            self.assertEqual(self.values(), [str(x) for x in DEFAULTS], bad)
        self.write(None, raw="{broken")
        time.sleep(1.6)
        self.assertEqual(self.values(), [str(x) for x in DEFAULTS])
        os.remove(self.cmd)                                                            # ファイルが消えても静かに続く
        time.sleep(1.6)
        self.write({"seq": 200, "action": "set", "values": VALID7})                  # その後の新しいコマンドは効く
        self.assertEqual(self.wait_values("v => v[6] === '7'"), [str(x) for x in VALID7])

        shot = os.environ.get("OCR_MOCK_SHOT")
        if shot:
            self.page.screenshot(path=shot)

    def test_reload_does_not_replay(self):
        self.open()
        self.write({"seq": 300, "action": "set", "values": [5, 5, 5, 5, 5, 5, 5]})
        self.wait_values("v => v[0] === '5'")
        self.open()                      # 再読込: 同じ seq=300 のファイルが残っているが、初回なので基準にするだけ
        time.sleep(2.5)
        self.assertEqual(self.values(), [str(x) for x in DEFAULTS])

    def test_works_with_no_command_file(self):
        self.open()
        time.sleep(2.2)
        self.assertEqual(self.values(), [str(x) for x in DEFAULTS])
        self.write({"seq": 5, "action": "randomize"})    # ファイル無しで開いた後に現れたものは新規として実行
        self.wait_values("v => v.join() !== '118,129,353,166,211,178,103'")


def socket_free_port():
    import socket
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def tearDownModule():
    shutil.rmtree(_TMP, ignore_errors=True)


if __name__ == "__main__":
    unittest.main(verbosity=1)
