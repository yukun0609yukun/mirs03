"""「設定が届いたか」の読み戻し(2026-10-13、docs/dashboard_field_sync.md)のテスト。対象は次の3項目。

  1. 推奨発話ON/OFF   ... directorが現在値を状態に載せ、画面は実際の値が一致するまで確認する
                         (director側の単体テストは test_readback_director.py)
  2. 模擬POSの数字    ... 模擬画面(ラッパーHTML)が「適用したseq・画面の7つの数字」をラズパイの kiosk_admin
                         (POST /ocr_mock_ack、ループバック限定)へ知らせ、画面が ack_seq/shown_values を照合する
  3. 混雑判定の背景キャリブレーション ... crowd_monitorがrequest_id付きで受理/完了/失敗を状態トピックに返し、画面が待つ

確かめること(ロボット・ROS・ラズパイ・2台目には一切触れない。APIはスタブのローカルHTTPサーバー、画面は実ブラウザ=Playwright/Edge):
  A. crowd_monitor: 要求の受理(started)→次のスキャンで完了(done)/保存失敗(failed)。request_idの引き継ぎ・TRANSIENT_LOCAL・従来のBoolの互換
  B. CrowdBackgroundController と ダッシュボードのAPI(POSTでrequest_id、GETで結果、guestの扱い)
  C. kiosk_admin の /ocr_mock_ack: 送信元がループバックのみ・Origin・ボディ上限・値の検証・原子的保存・心拍でファイルを書き換えない・
     /state の ocr_mock への反映・127.0.0.1側の専用待受はackだけ
  D. SurveyAdminProxy: 模擬POS操作の直後はキャッシュを捨てる(古い状態で読み戻しをしない)
  E. ラッパーHTML(実ブラウザ): ocr_cmd.json を書き換え → 数字が変わる → ackが届く(画面上の直接書き換えも)。ackが届かなくても表示は動く
  F. 管理画面(実ブラウザ): 3項目それぞれの 一致/不一致/タイムアウトの表示

実行: python ros2_ws/src/nokosahen_core/test/test_readback.py (普通のpythonで。-I は付けない)
(playwright / Edge / numpy が無い環境ではその部分だけスキップ)
"""
import http.client
import importlib.util
import json
import os
import re
import sys
import tempfile
import threading
import time
import types
import unittest
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest import mock
from urllib.parse import urlparse

TEST_DIR = Path(__file__).resolve().parent
PKG = TEST_DIR.parent
REPO = PKG.parents[2]
RASPI = REPO / "scripts" / "raspi_display"
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(TEST_DIR))

_TMP = tempfile.mkdtemp(prefix="readback_test_")
os.environ["SURVEY_DATA_DIR"] = os.path.join(_TMP, "data")
os.environ["KIOSK_MODE_FILE"] = os.path.join(_TMP, "kiosk_mode")
OCR_DIR = os.path.join(_TMP, "ocr_test")
os.environ["OCR_TEST_DIR"] = OCR_DIR
os.makedirs(OCR_DIR, exist_ok=True)


# ----------------------------------------------------------------------
# rclpyが無いPC用のスタブ(実物があれば何もしない)
# ----------------------------------------------------------------------
class _Msg:
    def __init__(self, *a, **k):
        self.data = None


class FakeNode:
    """rclpy.node.Node の代わり。crowd_monitor が使う分だけ。パラメータの上書きは overrides で行う"""
    overrides = {}

    def __init__(self, name):
        self.name = name
        self._params = {}
        self.subs = []
        self.pubs = []
        self.timers = []

    def declare_parameter(self, n, v):
        self._params[n] = self.overrides.get(n, v)

    def get_parameter(self, n):
        return types.SimpleNamespace(value=self._params[n])

    def get_logger(self):
        log = types.SimpleNamespace()
        log.info = log.warn = log.error = lambda *a, **k: None
        return log

    def create_subscription(self, typ, topic, cb, qos):
        self.subs.append((topic, cb, qos))
        return object()

    def create_publisher(self, typ, topic, qos):
        p = types.SimpleNamespace(topic=topic, qos=qos, sent=[])
        p.publish = lambda m, _p=p: _p.sent.append(m.data)
        self.pubs.append(p)
        return p

    def create_timer(self, period, cb):
        self.timers.append((period, cb))
        return object()

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=int(time.time() * 1e9)))


def _install_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    class _Enum:
        RELIABLE = "reliable"
        TRANSIENT_LOCAL = "transient_local"

    class QoSProfile:
        def __init__(self, depth=10, reliability=None, durability=None, **k):
            self.depth, self.reliability, self.durability = depth, reliability, durability

    mod("geometry_msgs")
    mod("geometry_msgs.msg", PoseStamped=_Msg, Twist=_Msg)
    mod("nav_msgs")
    mod("nav_msgs.msg", Odometry=_Msg)
    mod("std_msgs")
    mod("std_msgs.msg", Bool=_Msg, Float32MultiArray=_Msg, String=_Msg, Empty=_Msg)
    mod("sensor_msgs")
    mod("sensor_msgs.msg", LaserScan=_Msg)
    mod("rclpy")
    mod("rclpy.node", Node=FakeNode)
    mod("rclpy.qos", QoSProfile=QoSProfile, QoSDurabilityPolicy=_Enum, QoSReliabilityPolicy=_Enum)


try:
    import rclpy  # noqa: F401
    import geometry_msgs.msg  # noqa: F401
    import nav_msgs.msg  # noqa: F401
    import sensor_msgs.msg  # noqa: F401
    import std_msgs.msg  # noqa: F401
    STUBBED = False
except Exception:  # noqa: BLE001
    _install_stubs()
    STUBBED = True

try:
    import numpy  # noqa: F401
    HAVE_NP = True
except Exception:  # noqa: BLE001
    HAVE_NP = False

import test_dashboard_field_sync as tfs  # noqa: E402  (ページの組み立て・APIスタブの土台を流用。実行はされない)
from nokosahen_core.survey_admin_proxy import SurveyAdminProxy  # noqa: E402

try:
    from nokosahen_core import dashboard_common as dc  # noqa: E402
    _DC_ERR = None
except Exception as exc:  # pragma: no cover
    dc = None
    _DC_ERR = exc
try:
    from nokosahen_core import crowd_monitor as cm  # noqa: E402
    _CM_ERR = None
except Exception as exc:  # pragma: no cover
    cm = None
    _CM_ERR = exc


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


kiosk_admin = _load("kiosk_admin_readback", RASPI / "kiosk_admin.py")
VALID7 = [11, 22, 33, 44, 55, 66, 77]
DEFAULTS = [118, 129, 353, 166, 211, 178, 103]


# ======================================================================
# A. crowd_monitor
# ======================================================================
class _Scan:
    def __init__(self, n=200, bad=(0, 1, 2)):
        self.angle_min = -1.0
        self.angle_increment = 0.01
        self.range_min = 0.1
        self.range_max = 10.0
        self.ranges = [float("inf") if i in bad else 3.0 for i in range(n)]


def _msg(payload):
    m = _Msg()
    m.data = json.dumps(payload)
    return m


@unittest.skipUnless(cm is not None and HAVE_NP and STUBBED, f"crowd_monitor(スタブ)を使えません: {_CM_ERR}")
class CrowdMonitorStateTests(unittest.TestCase):
    def make(self, bg_path=None):
        FakeNode.overrides = {"background_file": bg_path or os.path.join(_TMP, "bg", "crowd_bg.json")}
        return cm.CrowdMonitor()

    @staticmethod
    def calib_pub(n):
        return [p for p in n.pubs if p.topic == "/nokosahen/crowd_calibration_state"][0]

    @staticmethod
    def states(n):
        return [json.loads(x) for x in CrowdMonitorStateTests.calib_pub(n).sent]

    def test_request_is_acknowledged_then_done_with_same_request_id(self):
        n = self.make()
        n._on_calibrate_request(_msg({"request_id": "cal-1"}))
        s = self.states(n)
        self.assertEqual([(x["request_id"], x["state"]) for x in s], [("cal-1", "started")])
        n._on_scan(_Scan())
        s = self.states(n)
        self.assertEqual([(x["request_id"], x["state"]) for x in s], [("cal-1", "started"), ("cal-1", "done")])
        done = s[-1]
        self.assertEqual(done["background_frames"], 197)       # 200点のうち有限は197点
        self.assertEqual(done["error"], "")
        self.assertIsInstance(done["at"], float)
        self.assertTrue(os.path.exists(n._background_path))
        n._on_scan(_Scan())                                     # 次のスキャンでは何も出さない(1要求につき1回)
        self.assertEqual(len(self.states(n)), 2)

    def test_state_topic_is_transient_local_reliable(self):
        n = self.make()
        qos = self.calib_pub(n).qos
        self.assertEqual((qos.depth, qos.reliability, qos.durability), (1, "reliable", "transient_local"))
        self.assertIn("/nokosahen/crowd_monitor/calibrate_request", [s[0] for s in n.subs])
        self.assertIn("/nokosahen/crowd_monitor/calibrate_background", [s[0] for s in n.subs])   # 従来のBoolも残す

    def test_save_failure_is_reported_as_failed(self):
        blocker = os.path.join(_TMP, "not_a_dir")
        Path(blocker).write_text("x")
        n = self.make(os.path.join(blocker, "sub", "bg.json"))   # 親がファイルなので作れない
        n._on_calibrate_request(_msg({"request_id": "cal-2"}))
        n._on_scan(_Scan())
        last = self.states(n)[-1]
        self.assertEqual((last["request_id"], last["state"]), ("cal-2", "failed"))
        self.assertIn("保存に失敗", last["error"])

    def test_legacy_bool_still_calibrates_with_empty_request_id(self):
        n = self.make()
        b = _Msg()
        b.data = True
        n._on_calibrate(b)
        self.assertEqual(self.states(n), [])                    # 従来経路は受理の通知を出さない
        n._on_scan(_Scan())
        last = self.states(n)[-1]
        self.assertEqual((last["request_id"], last["state"]), ("", "done"))

    def test_no_request_no_state(self):
        n = self.make()
        n._on_scan(_Scan())
        self.assertEqual(self.states(n), [])

    def test_bad_request_payload_still_calibrates(self):
        n = self.make()
        bad = _Msg()
        bad.data = "{not json"
        n._on_calibrate_request(bad)
        n._on_scan(_Scan())
        self.assertEqual([(x["request_id"], x["state"]) for x in self.states(n)], [("", "started"), ("", "done")])


# ======================================================================
# B. CrowdBackgroundController とダッシュボードのAPI
# ======================================================================
class _CtlNode:
    def __init__(self):
        self.pubs = {}
        self.subs = []

    def create_publisher(self, _t, topic, _q):
        return self.pubs.setdefault(topic, types.SimpleNamespace(sent=[], publish=None))

    def create_subscription(self, typ, topic, cb, qos):
        self.subs.append((topic, cb, qos))


@unittest.skipIf(dc is None, f"dashboard_common import失敗: {_DC_ERR}")
class CrowdControllerTests(unittest.TestCase):
    def make(self):
        n = _CtlNode()
        orig = n.create_publisher

        def cp(t, topic, q):
            p = orig(t, topic, q)
            p.publish = lambda m, _p=p: _p.sent.append(m.data)
            return p
        n.create_publisher = cp
        return n, dc.CrowdBackgroundController(n)

    def test_request_carries_unique_request_id(self):
        n, c = self.make()
        ok1, msg1, r1 = c.request_calibration()
        ok2, _m, r2 = c.request_calibration()
        self.assertTrue(ok1 and ok2)
        self.assertNotEqual(r1, r2)
        sent = [json.loads(x) for x in n.pubs[c.REQUEST_TOPIC].sent]
        self.assertEqual([s["request_id"] for s in sent], [r1, r2])
        self.assertEqual(c.last_request_id, r2)
        self.assertIn("要求を送りました", msg1)                  # 「記録しました」とは言わない(まだ確認できていない)

    def test_calibrate_keeps_old_two_value_return(self):
        _n, c = self.make()
        ok, message = c.calibrate()
        self.assertTrue(ok)
        self.assertIsInstance(message, str)

    def test_status_before_and_after_state(self):
        n, c = self.make()
        self.assertEqual(c.status(), {"state": None, "request_id": None})
        topic, cb, qos = n.subs[0]
        self.assertEqual(topic, c.STATE_TOPIC)
        m = _Msg()
        m.data = json.dumps({"request_id": "x", "state": "done", "at": 1.0, "background_frames": 5, "error": ""})
        cb(m)
        self.assertEqual(c.status()["state"], "done")
        bad = _Msg()
        bad.data = "garbage"
        cb(bad)                                                  # 壊れた配信は無視(直前の値を保つ)
        self.assertEqual(c.status()["request_id"], "x")
        arr = _Msg()
        arr.data = "[1]"
        cb(arr)
        self.assertEqual(c.status()["request_id"], "x")


class _FakeTeleop:
    soft_estop_engaged = False


def _http(method, url, body=None, headers=None):
    p = urlparse(url)
    c = http.client.HTTPConnection(p.hostname, p.port, timeout=10)
    data = body if isinstance(body, (bytes, type(None))) else json.dumps(body).encode()
    c.request(method, p.path + (("?" + p.query) if p.query else ""), body=data, headers=headers or {})
    r = c.getresponse()
    raw = r.read()
    out = (r.status, raw, dict(r.getheaders()))
    c.close()
    return out


@unittest.skipIf(dc is None, f"dashboard_common import失敗: {_DC_ERR}")
class CrowdDashboardApiTests(unittest.TestCase):
    def setUp(self):
        self.node = _CtlNode()
        orig = self.node.create_publisher

        def cp(t, topic, q):
            p = orig(t, topic, q)
            p.publish = lambda m, _p=p: _p.sent.append(m.data)
            return p
        self.node.create_publisher = cp
        self.ctl = dc.CrowdBackgroundController(self.node)
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, _FakeTeleop(), crowd_background=self.ctl,
            auth_username="TL", auth_password="pw", enforce_single_controller=False)
        self.srv = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        self.srv.daemon_threads = True
        self.port = self.srv.server_address[1]
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        self.tl = self.login("TL", "pw")
        self.guest = self.login("guest", "")

    def tearDown(self):
        self.srv.shutdown()
        self.srv.server_close()

    def login(self, user, pw):
        code, _b, h = _http("POST", f"http://127.0.0.1:{self.port}/api/login", {"username": user, "password": pw},
                            {"Content-Type": "application/json"})
        return h.get("Set-Cookie", "").split(";")[0]

    def call(self, method, path, body=None, cookie=None):
        h = {"Content-Type": "application/json"}
        c = self.tl if cookie is None else cookie
        if c:
            h["Cookie"] = c
        code, raw, _ = _http(method, f"http://127.0.0.1:{self.port}{path}", body, h)
        try:
            return code, json.loads(raw.decode("utf-8"))
        except ValueError:
            return code, None

    def test_post_returns_request_id_and_get_returns_monitor_result(self):
        code, d = self.call("POST", "/api/crowd_background/calibrate", {})
        self.assertEqual(code, 200)
        self.assertTrue(d["ok"])
        rid = d["request_id"]
        self.assertTrue(rid)
        self.assertEqual(json.loads(self.node.pubs[self.ctl.REQUEST_TOPIC].sent[-1])["request_id"], rid)
        code, s = self.call("GET", "/api/crowd_background/state")
        self.assertEqual((code, s["state"]), (200, None))       # まだcrowd_monitorから何も返っていない
        m = _Msg()
        m.data = json.dumps({"request_id": rid, "state": "done", "at": 1.0, "background_frames": 9, "error": ""})
        self.node.subs[0][1](m)
        code, s = self.call("GET", "/api/crowd_background/state")
        self.assertEqual((s["request_id"], s["state"], s["background_frames"]), (rid, "done", 9))

    def test_auth(self):
        self.assertEqual(self.call("GET", "/api/crowd_background/state", cookie="")[0], 401)
        self.assertEqual(self.call("POST", "/api/crowd_background/calibrate", {}, cookie="")[0], 401)
        self.assertEqual(self.call("POST", "/api/crowd_background/calibrate", {}, cookie=self.guest)[0], 403)
        self.assertEqual(self.node.pubs.get(self.ctl.REQUEST_TOPIC) and self.node.pubs[self.ctl.REQUEST_TOPIC].sent, [])
        self.assertEqual(self.call("GET", "/api/crowd_background/state", cookie=self.guest)[0], 200)   # 閲覧は可

    def test_not_available_system(self):
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, _FakeTeleop(),
            auth_username="TL", auth_password="pw", enforce_single_controller=False)
        srv = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        try:
            port = srv.server_address[1]
            _c, _b, h = _http("POST", f"http://127.0.0.1:{port}/api/login", {"username": "TL", "password": "pw"},
                              {"Content-Type": "application/json"})
            ck = h.get("Set-Cookie", "").split(";")[0]
            code, _r, _h = _http("POST", f"http://127.0.0.1:{port}/api/crowd_background/calibrate", {},
                                 {"Content-Type": "application/json", "Cookie": ck})
            self.assertEqual(code, 404)
        finally:
            srv.shutdown()
            srv.server_close()


# ======================================================================
# C. kiosk_admin の /ocr_mock_ack
# ======================================================================
def _reset_kiosk():
    kiosk_admin._ack_mem = None
    for f in os.listdir(OCR_DIR):
        if f.startswith("ocr_"):
            os.remove(os.path.join(OCR_DIR, f))
    if os.path.exists(os.environ["KIOSK_MODE_FILE"]):
        os.remove(os.environ["KIOSK_MODE_FILE"])


class KioskAckLogicTests(unittest.TestCase):
    def setUp(self):
        _reset_kiosk()

    def test_loopback_allowed(self):
        for ip in ("127.0.0.1", "127.0.0.2", "::1", "::ffff:127.0.0.1"):
            self.assertTrue(kiosk_admin.loopback_allowed(ip), ip)
        for ip in ("198.51.100.2", "198.51.100.1", "203.0.113.6", "0.0.0.0", "::ffff:198.51.100.2", "8.8.8.8", "", "garbage", None):
            self.assertFalse(kiosk_admin.loopback_allowed(ip), str(ip))

    def test_parse_body_validation(self):
        ok = json.dumps({"seq": 5, "values": VALID7}).encode()
        self.assertEqual(kiosk_admin.parse_ocr_ack_body(ok), (5, VALID7, None))
        self.assertEqual(kiosk_admin.parse_ocr_ack_body(json.dumps({"seq": 0, "values": [0] * 7}).encode())[0], 0)
        bad = [
            b"", b"{", b"[]", b"null", b"\xff\xfe",
            json.dumps({"seq": 5}).encode(), json.dumps({"values": VALID7}).encode(),
            json.dumps({"seq": 5, "values": VALID7, "x": 1}).encode(),
            json.dumps({"seq": -1, "values": VALID7}).encode(),
            json.dumps({"seq": True, "values": VALID7}).encode(),
            json.dumps({"seq": 1.5, "values": VALID7}).encode(),
            json.dumps({"seq": "5", "values": VALID7}).encode(),
            json.dumps({"seq": 2 ** 60, "values": VALID7}).encode(),
            json.dumps({"seq": 5, "values": VALID7[:6]}).encode(),
            json.dumps({"seq": 5, "values": VALID7 + [1]}).encode(),
            json.dumps({"seq": 5, "values": VALID7[:6] + [10000]}).encode(),
            json.dumps({"seq": 5, "values": VALID7[:6] + [-1]}).encode(),
            json.dumps({"seq": 5, "values": VALID7[:6] + [1.5]}).encode(),
            json.dumps({"seq": 5, "values": VALID7[:6] + ["7"]}).encode(),
            json.dumps({"seq": 5, "values": VALID7[:6] + [True]}).encode(),
            json.dumps({"seq": 5, "values": "1234567"}).encode(),
            b"x" * 600,
        ]
        for b in bad:
            seq, vals, err = kiosk_admin.parse_ocr_ack_body(b)
            self.assertIsNotNone(err, b[:40])
            self.assertIsNone(seq)
        self.assertIsNotNone(kiosk_admin.parse_ocr_ack_body(None)[2])

    def test_record_writes_atomically_and_only_on_change(self):
        a = kiosk_admin.record_ocr_ack(7, VALID7, now=1000)
        self.assertEqual(a, {"ack_seq": 7, "shown_values": VALID7, "acked_at": 1000})
        path = kiosk_admin.OCR_ACK_FILE
        self.assertEqual(json.load(open(path, encoding="utf-8")), a)
        self.assertEqual([f for f in os.listdir(OCR_DIR) if f.endswith(".tmp")], [])
        before = os.stat(path).st_mtime_ns
        with mock.patch.object(kiosk_admin.os, "replace", side_effect=AssertionError("心拍でファイルを書き換えてはいけない")):
            b = kiosk_admin.record_ocr_ack(7, VALID7, now=1005)  # 心拍: 同じ内容
        self.assertEqual(b["acked_at"], 1005)
        self.assertEqual(os.stat(path).st_mtime_ns, before)
        self.assertEqual(kiosk_admin.read_ocr_ack()["acked_at"], 1005)   # 更新時刻はメモリ
        c = kiosk_admin.record_ocr_ack(7, [1, 2, 3, 4, 5, 6, 7], now=1010)  # 数字が変わった
        self.assertEqual(json.load(open(path, encoding="utf-8"))["shown_values"], [1, 2, 3, 4, 5, 6, 7])
        d = kiosk_admin.record_ocr_ack(8, [1, 2, 3, 4, 5, 6, 7], now=1011)  # seqが変わった
        self.assertEqual(json.load(open(path, encoding="utf-8"))["ack_seq"], 8)
        self.assertEqual((c["ack_seq"], d["ack_seq"]), (7, 8))

    def test_ack_survives_restart_via_file(self):
        kiosk_admin.record_ocr_ack(9, VALID7, now=2000)
        kiosk_admin._ack_mem = None                      # kiosk_adminの再起動
        self.assertEqual(kiosk_admin.read_ocr_ack(), {"ack_seq": 9, "shown_values": VALID7, "acked_at": 2000})
        Path(kiosk_admin.OCR_ACK_FILE).write_text("{broken", encoding="utf-8")
        kiosk_admin._ack_mem = None
        self.assertIsNone(kiosk_admin.read_ocr_ack())    # 壊れたファイルは無いものとして扱う

    def test_summary_shapes(self):
        self.assertIsNone(kiosk_admin.ocr_mock_summary())
        cmd = kiosk_admin.write_ocr_cmd("reset", None, now=77)
        self.assertEqual(kiosk_admin.ocr_mock_summary(), {"seq": 77000, "action": "reset", "at": 77})   # ackが無ければ従来の形
        kiosk_admin.record_ocr_ack(cmd["seq"], DEFAULTS, now=100)
        s = kiosk_admin.ocr_mock_summary(now=103)
        self.assertEqual(s, {"seq": 77000, "action": "reset", "at": 77, "ack_seq": 77000,
                             "shown_values": DEFAULTS, "acked_at": 100, "ack_age_s": 3})
        os.remove(kiosk_admin.OCR_CMD_FILE)               # コマンドファイルが無くてもackだけは見える
        s = kiosk_admin.ocr_mock_summary(now=103)
        self.assertEqual((s["seq"], s["ack_seq"]), (None, 77000))


def _serve(handler_cls, port=0):
    srv = ThreadingHTTPServer(("127.0.0.1", port), handler_cls)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


ORIGIN = "http://127.0.0.1:8084"


class KioskAckHttpTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.main = _serve(kiosk_admin.Handler)
        cls.loop = _serve(kiosk_admin.LoopbackHandler)
        cls.mp, cls.lp = cls.main.server_address[1], cls.loop.server_address[1]

    @classmethod
    def tearDownClass(cls):
        for s in (cls.main, cls.loop):
            s.shutdown()
            s.server_close()

    def setUp(self):
        _reset_kiosk()
        self._allowed = kiosk_admin.ALLOWED_CLIENTS
        kiosk_admin.ALLOWED_CLIENTS = {"127.0.0.1"}

    def tearDown(self):
        kiosk_admin.ALLOWED_CLIENTS = self._allowed

    def post(self, port, body, headers=None, path="/ocr_mock_ack"):
        raw = body if isinstance(body, bytes) else json.dumps(body).encode()
        h = {"Content-Type": "text/plain"}
        h.update(headers or {})
        code, b, rh = _http("POST", f"http://127.0.0.1:{port}{path}", raw, h)
        return code, (json.loads(b.decode()) if b else None), rh

    def test_ack_is_accepted_from_loopback_and_reflected_in_state(self):
        for port in (self.mp, self.lp):
            _reset_kiosk()
            code, d, h = self.post(port, {"seq": 5, "values": VALID7}, {"Origin": ORIGIN})
            self.assertEqual((code, d), (200, {"ok": True}), port)
            self.assertEqual(h.get("Access-Control-Allow-Origin"), ORIGIN)   # ラッパーが応答を読める(このオリジンだけ)
            self.assertEqual(kiosk_admin.read_ocr_ack()["shown_values"], VALID7)
        kiosk_admin.write_ocr_cmd("set", VALID7, now=5)
        code, raw, _ = _http("GET", f"http://127.0.0.1:{self.mp}/state")
        om = json.loads(raw.decode())["ocr_mock"]
        self.assertEqual((om["seq"], om["ack_seq"], om["shown_values"]), (5000, 5, VALID7))
        self.assertIn("acked_at", om)
        self.assertIn("ack_age_s", om)

    def test_no_origin_header_is_ok_for_local_tools(self):
        code, d, h = self.post(self.lp, {"seq": 1, "values": VALID7})
        self.assertEqual(code, 200)
        self.assertNotIn("Access-Control-Allow-Origin", h)

    def test_non_loopback_source_is_rejected(self):
        with mock.patch.object(kiosk_admin, "loopback_allowed", return_value=False):
            for port in (self.mp, self.lp):
                code, d, _h = self.post(port, {"seq": 5, "values": VALID7}, {"Origin": ORIGIN})
                self.assertEqual(code, 403, port)
        self.assertIsNone(kiosk_admin.read_ocr_ack())
        self.assertFalse(os.path.exists(kiosk_admin.OCR_ACK_FILE))

    def test_foreign_origin_is_rejected(self):
        for origin in ("http://evil.example", "http://127.0.0.1:9999", "null", "http://localhost:8084"):
            code, d, h = self.post(self.lp, {"seq": 5, "values": VALID7}, {"Origin": origin})
            self.assertEqual(code, 403, origin)
            self.assertNotIn("Access-Control-Allow-Origin", h)
        self.assertIsNone(kiosk_admin.read_ocr_ack())

    def test_bad_bodies_and_limits(self):
        for body in ({"seq": 5}, {"seq": 5, "values": VALID7[:6]}, {"seq": 5, "values": VALID7[:6] + [10000]},
                     {"seq": -1, "values": VALID7}, {"seq": 5, "values": VALID7, "x": 1}, b"{", b"x" * 600):
            code, d, _h = self.post(self.lp, body)
            self.assertEqual(code, 400, str(body)[:50])
        # Content-Lengthだけ大きく宣言して本文を送らない/負の値でも、読み込みで待たされず400
        c = http.client.HTTPConnection("127.0.0.1", self.lp, timeout=5)
        c.putrequest("POST", "/ocr_mock_ack")
        c.putheader("Content-Length", "100000")
        c.endheaders()
        self.assertEqual(c.getresponse().status, 400)
        c.close()
        self.assertIsNone(kiosk_admin.read_ocr_ack())

    def test_loopback_listener_only_serves_ack(self):
        self.assertEqual(_http("GET", f"http://127.0.0.1:{self.lp}/state")[0], 404)
        self.assertEqual(_http("GET", f"http://127.0.0.1:{self.lp}/responses.csv")[0], 404)
        for path, body in (("/mode", {"mode": "survey"}), ("/ocr_mock", {"action": "randomize"})):
            self.assertEqual(self.post(self.lp, body, path=path)[0], 404, path)
        self.assertFalse(os.path.exists(kiosk_admin.OCR_CMD_FILE))
        self.assertFalse(os.path.exists(os.environ["KIOSK_MODE_FILE"]))

    def test_admin_endpoints_still_guarded_by_client_allowlist(self):
        kiosk_admin.ALLOWED_CLIENTS = {"198.51.100.2"}
        self.assertEqual(_http("GET", f"http://127.0.0.1:{self.mp}/state")[0], 403)
        self.assertEqual(self.post(self.mp, {"action": "randomize"}, path="/ocr_mock")[0], 403)
        # ackはallowlistではなくループバックで判定される(別経路)
        self.assertEqual(self.post(self.mp, {"seq": 1, "values": VALID7})[0], 200)

    def test_preflight(self):
        def opt(port, origin, path="/ocr_mock_ack"):
            h = {"Access-Control-Request-Method": "POST", "Access-Control-Request-Headers": "content-type"}
            if origin:
                h["Origin"] = origin
            return _http("OPTIONS", f"http://127.0.0.1:{port}{path}", None, h)
        for port in (self.mp, self.lp):
            code, _b, h = opt(port, ORIGIN)
            self.assertEqual(code, 204)
            self.assertEqual(h.get("Access-Control-Allow-Origin"), ORIGIN)
            self.assertIn("POST", h.get("Access-Control-Allow-Methods", ""))
            self.assertIn("Content-Type", h.get("Access-Control-Allow-Headers", ""))
            self.assertEqual(opt(port, "http://evil.example")[0], 403)
            self.assertEqual(opt(port, None)[0], 403)
            self.assertEqual(opt(port, ORIGIN, "/ocr_mock")[0], 403)


# ======================================================================
# D. SurveyAdminProxy
# ======================================================================
class ProxyCacheTests(unittest.TestCase):
    def test_cache_is_dropped_after_ocr_mock_so_readback_is_fresh(self):
        p = SurveyAdminProxy("http://x.invalid")
        calls = []

        def fake(path, payload, timeout):
            calls.append(path)
            if path == "/ocr_mock":
                return {"ok": True, "ocr_mock": {"seq": 12, "action": "reset", "at": 1}}
            return {"ok": True, "mode": "ocrtest", "ocr_mock": {"seq": 12, "ack_seq": len(calls)}}
        p._request = fake
        p.get_state()
        p.get_state()
        self.assertEqual(calls, ["/state"])                       # TTL内は問い合わせない
        code, d = p.set_ocr_mock({"action": "reset"})
        self.assertEqual((code, d["ok"], d["state"]["ocr_mock"]["seq"]), (200, True, 12))
        p.get_state()
        self.assertEqual(calls, ["/state", "/ocr_mock", "/state"])   # 操作の直後は必ず取り直す


# ======================================================================
# E. ラッパーHTML(実ブラウザ)
# ======================================================================
class _Quiet(SimpleHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


def wait_for(fn, timeout=8.0, step=0.1, page=None):
    """条件が真になるまで待つ。pageを渡すと待ちの間もPlaywrightのイベント(route等)を処理する
    (time.sleepだとsync APIのコールバックが止まり、付け替えたackのリクエストが進まない)"""
    end = time.time() + timeout
    while time.time() < end:
        v = fn()
        if v:
            return v
        if page is not None:
            page.wait_for_timeout(int(step * 1000))
        else:
            time.sleep(step)
    return fn()


@unittest.skipUnless(tfs.HAVE_PW, "playwright がありません")
class WrapperAckTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import shutil
        for f in ("mock_pos_tablet.html", "mock_pos_tablet_original.html"):
            shutil.copy(RASPI / "ocr_test" / f, os.path.join(OCR_DIR, f))
        cls.web = _serve(partial(_Quiet, directory=OCR_DIR))       # ラズパイの python3 -m http.server 8084 の代わり
        cls.wp = cls.web.server_address[1]
        cls.kiosk = _serve(kiosk_admin.Handler)                    # ラズパイの kiosk_admin(127.0.0.1:8083)の代わり
        cls.kp = cls.kiosk.server_address[1]
        cls._orig_origin = kiosk_admin.ACK_ORIGIN
        kiosk_admin.ACK_ORIGIN = f"http://127.0.0.1:{cls.wp}"      # 本番は :8084。テストのポートに合わせる
        cls.pw = tfs.sync_playwright().start()
        try:
            cls.browser = cls.pw.chromium.launch(channel="msedge")
        except Exception as e:  # noqa: BLE001
            cls.pw.stop()
            raise unittest.SkipTest(f"Edge を起動できません: {e}")

    @classmethod
    def tearDownClass(cls):
        kiosk_admin.ACK_ORIGIN = cls._orig_origin
        try:
            cls.browser.close()
            cls.pw.stop()
        finally:
            for s in (cls.web, cls.kiosk):
                s.shutdown()
                s.server_close()

    def setUp(self):
        _reset_kiosk()
        self.base_cmd = kiosk_admin.write_ocr_cmd("reset", None)   # ページが最初に読んで「処理済み」にする基準
        self.ctx = self.browser.new_context(viewport={"width": 800, "height": 480})
        self.ctx.route(re.compile(r"https?://fonts\.(googleapis|gstatic)\.com/.*"), lambda r: r.abort())
        self.errors = []
        self.page = self.ctx.new_page()
        self.page.on("pageerror", lambda e: self.errors.append(str(e)))

    def tearDown(self):
        self.ctx.close()

    def redirect_ack(self):
        # ページは本番どおり http://127.0.0.1:8083/ocr_mock_ack へ送る。テストでは立てたkiosk_adminのポートへ付け替える
        self.ctx.route("http://127.0.0.1:8083/**", lambda r: r.continue_(url=f"http://127.0.0.1:{self.kp}/ocr_mock_ack"))

    def open(self):
        self.page.goto(f"http://127.0.0.1:{self.wp}/mock_pos_tablet.html")
        self.page.wait_for_function(
            "() => { const d = document.getElementById('f').contentDocument; return d && d.querySelectorAll('.count-input').length === 7; }", timeout=8000)

    def shown(self):
        return self.page.evaluate(
            "() => Array.from(document.getElementById('f').contentDocument.querySelectorAll('.count-input')).map(e => e.value)")

    def ack(self):
        return kiosk_admin.read_ocr_ack()

    def test_initial_page_reports_baseline_seq_and_shown_values(self):
        self.redirect_ack()
        self.open()
        a = wait_for(lambda: self.ack(), 6, page=self.page)
        self.assertIsNotNone(a, "起動直後にackが届かない")
        self.assertEqual(a["ack_seq"], self.base_cmd["seq"])      # 初回に読んだseqは「処理済み」として報告
        self.assertEqual(a["shown_values"], DEFAULTS)

    def test_command_is_applied_and_acked(self):
        self.redirect_ack()
        self.open()
        wait_for(lambda: self.ack(), 6, page=self.page)
        cmd = kiosk_admin.write_ocr_cmd("set", VALID7)             # 管理画面→kiosk_admin→ocr_cmd.json と同じ書き方
        a = wait_for(lambda: (self.ack() or {}).get("ack_seq") == cmd["seq"] and self.ack(), 8, page=self.page)
        self.assertTrue(a, "適用したseqのackが届かない")
        self.assertEqual(a["shown_values"], VALID7)
        self.assertEqual(self.shown(), [str(v) for v in VALID7])   # 画面にも実際に出ている
        # kiosk_adminの/stateが、管理画面の照合に必要な項目を返す
        s = kiosk_admin.ocr_mock_summary()
        self.assertEqual((s["seq"], s["ack_seq"], s["shown_values"]), (cmd["seq"], cmd["seq"], VALID7))

        cmd2 = kiosk_admin.write_ocr_cmd("randomize", None)
        a2 = wait_for(lambda: (self.ack() or {}).get("ack_seq") == cmd2["seq"] and self.ack(), 8, page=self.page)
        self.assertTrue(a2)
        self.assertEqual([str(v) for v in a2["shown_values"]], self.shown())   # ランダム化後の「実際の表示」を報告
        self.assertNotEqual(a2["shown_values"], VALID7)

        cmd3 = kiosk_admin.write_ocr_cmd("reset", None)
        a3 = wait_for(lambda: (self.ack() or {}).get("ack_seq") == cmd3["seq"] and self.ack(), 8, page=self.page)
        self.assertEqual(a3["shown_values"], DEFAULTS)
        self.assertEqual(self.errors, [])

    def test_direct_edit_on_screen_is_reported_with_same_seq(self):
        self.redirect_ack()
        self.open()
        a0 = wait_for(lambda: self.ack(), 6, page=self.page)
        self.page.evaluate("""() => { const e = document.getElementById('f').contentDocument.querySelectorAll('.count-input')[0];
                                      e.value = '999'; e.dispatchEvent(new Event('input', { bubbles: true })); }""")
        a = wait_for(lambda: (self.ack() or {}).get("shown_values", [0])[0] == 999 and self.ack(), 5, page=self.page)
        self.assertTrue(a, "画面上の直接書き換えがackに出ない")
        self.assertEqual(a["ack_seq"], a0["ack_seq"])              # 新しいコマンドではないのでseqは変わらない

    def test_blank_field_is_not_reported_as_a_number(self):
        self.redirect_ack()
        self.open()
        a0 = wait_for(lambda: self.ack(), 6, page=self.page)
        self.page.evaluate("""() => { const e = document.getElementById('f').contentDocument.querySelectorAll('.count-input')[2];
                                      e.value = ''; e.dispatchEvent(new Event('input', { bubbles: true })); }""")
        self.page.wait_for_timeout(2500)
        self.assertEqual(self.ack()["shown_values"], a0["shown_values"])   # 空欄の間は送らない(検証に通らない値を送らない)

    def test_heartbeat_updates_time_without_rewriting_file(self):
        self.redirect_ack()
        self.open()
        a0 = wait_for(lambda: self.ack(), 6, page=self.page)
        mt = os.stat(kiosk_admin.OCR_ACK_FILE).st_mtime_ns
        t0 = a0["acked_at"]
        a1 = wait_for(lambda: self.ack()["acked_at"] > t0 and self.ack(), 12, page=self.page)
        self.assertTrue(a1, "5秒おきの心拍が届かない")
        self.assertEqual(os.stat(kiosk_admin.OCR_ACK_FILE).st_mtime_ns, mt)

    def test_display_keeps_working_when_ack_cannot_be_sent(self):
        self.ctx.route("http://127.0.0.1:8083/**", lambda r: r.abort())
        self.open()
        cmd = kiosk_admin.write_ocr_cmd("set", VALID7)
        wait_for(lambda: self.shown() == [str(v) for v in VALID7], 8, page=self.page)
        self.assertEqual(self.shown(), [str(v) for v in VALID7])
        self.page.wait_for_timeout(1500)
        self.assertEqual(self.errors, [])
        self.assertIsNone(self.ack())
        self.assertGreater(cmd["seq"], 0)


# ======================================================================
# F. 管理画面(実ブラウザ): 3項目の 一致/不一致/タイムアウトの表示
# ======================================================================
class Rb:
    """管理画面の相手役(director・ラズパイ・crowd_monitor)の状態"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.director = True
        self.silent = False            # directorが現在値を返さない(未起動/古い版)
        self.rec_applies = True        # directorが受け取る
        self.requested = None
        self.mode = "ocrtest"
        self.seq = 1000
        self.ack_seq = 1000
        self.shown = list(DEFAULTS)
        self.ocr_applies = True
        self.ocr_override = None       # 画面が指定と違う数字を出している
        self.crowd = "done"            # done / failed / started / silent
        self.crowd_t0 = None
        self.crowd_n = 0
        self.log = []


class RbHandler(tfs.Handler):
    rb = None

    def do_GET(self):  # noqa: N802
        path = urlparse(self.path).path
        rb = self.rb
        if path == "/api/state":
            d = {"soft_estop_engaged": False, "behavior_age_s": 1, "mode": "NORMAL", "idle_wiggle_enabled": False,
                 "recommend_requested": rb.requested}
            if not rb.silent:
                d["recommend_enabled"] = rb.director
            return self._send(200, d)
        if path == "/api/survey/state":
            om = {"seq": rb.seq, "action": "set", "at": 1}
            if rb.ack_seq is not None:
                om.update(ack_seq=rb.ack_seq, shown_values=list(rb.shown), acked_at=1, ack_age_s=1)
            return self._send(200, {"ok": True, "configured": True, "state": {
                "mode": rb.mode, "count": 3, "active": False, "receiver": True, "session_state": "idle",
                "last_activity_age_s": 5, "condition": "A", "ocr_mock": om}})
        if path == "/api/crowd_background/state":
            if rb.crowd == "silent" or rb.crowd_t0 is None:
                return self._send(200, {"state": None, "request_id": None})
            rid = f"r{rb.crowd_n}"
            if rb.crowd == "stale":     # 前回の要求の結果が残っているだけ(今回のrequest_idとは別)
                return self._send(200, {"request_id": "old", "state": "done", "at": 1.0, "background_frames": 1, "error": ""})
            if rb.crowd == "started" or time.time() - rb.crowd_t0 < 1.5:
                return self._send(200, {"request_id": rid, "state": "started", "at": 1.0, "background_frames": 0, "error": ""})
            if rb.crowd == "failed":
                return self._send(200, {"request_id": rid, "state": "failed", "at": 1.0, "background_frames": 0, "error": "背景ファイルの保存に失敗"})
            return self._send(200, {"request_id": rid, "state": "done", "at": 1.0, "background_frames": 1234, "error": ""})
        return super().do_GET()

    def do_POST(self):  # noqa: N802
        path = urlparse(self.path).path
        rb = self.rb
        if path == "/api/recommend_enabled":
            body = self._body()
            rb.log.append((path, body))
            rb.requested = bool(body.get("enabled"))
            if rb.rec_applies:
                rb.director = rb.requested
            return self._send(200, {"ok": True, "message": "推奨発話を" + ("有効" if rb.requested else "無効") + "にしました", "enabled": rb.requested})
        if path == "/api/survey/ocr_mock":
            body = self._body()
            rb.log.append((path, body))
            rb.seq += 1
            seq = rb.seq
            act = body.get("action")
            new = body.get("values") if act == "set" else (list(DEFAULTS) if act == "reset" else [3, 14, 159, 26, 5, 35, 89])

            def ack():
                time.sleep(0.4)
                rb.shown = list(rb.ocr_override or new)
                rb.ack_seq = seq
            if rb.ocr_applies:
                threading.Thread(target=ack, daemon=True).start()
            return self._send(200, {"ok": True, "configured": True, "state": {"ok": True, "ocr_mock": {"seq": seq, "action": act, "at": 1}}})
        if path == "/api/crowd_background/calibrate":
            rb.log.append((path, self._body()))
            rb.crowd_n += 1
            rb.crowd_t0 = time.time()
            return self._send(200, {"ok": True, "message": "要求を送りました", "request_id": f"r{rb.crowd_n}"})
        return super().do_POST()


@unittest.skipUnless(tfs.HAVE_PW, "playwright がありません")
class DashboardReadbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.stub = tfs.Stub()
        cls.rb = Rb()
        RbHandler.stub = cls.stub
        RbHandler.rb = cls.rb
        RbHandler.pages = tfs.PAGES
        cls.srv = _serve(RbHandler)
        cls.port = cls.srv.server_address[1]
        cls.pw = tfs.sync_playwright().start()
        try:
            cls.browser = cls.pw.chromium.launch(channel="msedge")
        except Exception as e:  # noqa: BLE001
            cls.pw.stop()
            cls.srv.shutdown()
            raise unittest.SkipTest(f"Edge を起動できません: {e}")

    @classmethod
    def tearDownClass(cls):
        try:
            cls.browser.close()
            cls.pw.stop()
        finally:
            cls.srv.shutdown()
            cls.srv.server_close()

    def setUp(self):
        self.stub.reset()
        self.rb.reset()
        self.ctx = self.browser.new_context(viewport={"width": 1280, "height": 900})
        self.ctx.route(re.compile(r"https?://fonts\.(googleapis|gstatic)\.com/.*"), lambda r: r.abort())
        self.page = self.ctx.new_page()
        self.errors = []
        self.page.on("pageerror", lambda e: self.errors.append(str(e)))
        self.page.on("dialog", lambda d: d.accept())

    def tearDown(self):
        self.ctx.close()

    def open(self, hash_, wait):
        self.page.goto(f"http://127.0.0.1:{self.port}/{hash_}")
        self.page.wait_for_function(wait, timeout=10000)
        self.page.evaluate("document.querySelectorAll('details').forEach(d => { d.open = true; })")

    def click(self, id_):
        self.page.evaluate(f"document.getElementById('{id_}').click()")

    def text(self, sel):
        return self.page.text_content(sel) or ""

    def wait_text(self, sel, needle, timeout=15000):
        self.page.wait_for_function("([s, n]) => (document.querySelector(s).textContent || '').includes(n)", arg=[sel, needle], timeout=timeout)

    # ---- 1. 推奨発話 ----
    def open_rec(self):
        self.open("#tab-voice", "() => document.getElementById('recommendStateText').textContent.includes('ON(')")

    def test_recommend_shows_director_value_with_default_note(self):
        self.open_rec()
        self.assertIn("ON(起動時の既定", self.text("#recommendStateText"))
        self.assertTrue(self.page.evaluate("document.getElementById('recommendDirectorValue').checked"))

    def test_recommend_confirmed_when_director_value_matches(self):
        self.open_rec()
        self.click("recommendOffBtn")
        self.wait_text("#behaviorControlHint", "directorがOFFになったことを確認しました")
        self.wait_text("#recommendStateText", "OFF(確認済み)")
        self.assertFalse(self.page.evaluate("document.getElementById('recommendDirectorValue').checked"))
        self.click("recommendOnBtn")
        self.wait_text("#behaviorControlHint", "directorがONになったことを確認しました")
        self.wait_text("#recommendStateText", "ON(確認済み)")
        self.assertEqual(self.errors, [])

    def test_recommend_not_reaching_director_is_reported(self):
        self.rb.rec_applies = False
        self.open_rec()
        self.click("recommendOffBtn")
        self.wait_text("#behaviorControlHint", "directorに届いていません", 20000)
        self.wait_text("#recommendStateText", "未確認")
        self.assertIn("ON", self.text("#recommendStateText"))      # director側の実際の値(ON)を出す
        self.assertNotIn("確認済み", self.text("#recommendStateText"))

    def test_recommend_director_not_reporting_is_not_called_success(self):
        self.rb.silent = True
        self.open("#tab-voice", "() => document.getElementById('recommendStateText').textContent.includes('未確認')")
        self.click("recommendOffBtn")
        self.wait_text("#behaviorControlHint", "届いたか確認できません", 20000)
        self.assertNotIn("確認しました", self.text("#behaviorControlHint"))

    # ---- 2. 模擬POS ----
    def open_ocr(self):
        self.open("#tab-settings", "() => document.getElementById('surveyMode').textContent.includes('OCRテスト')")

    def test_ocr_set_confirmed_with_displayed_values(self):
        self.open_ocr()
        for k, v in enumerate(VALID7):
            self.page.fill(f".ocrm-in >> nth={k}", str(v))
        self.click("ocrMockSetBtn")
        self.wait_text("#ocrMockHint", "模擬画面に反映されました")
        self.assertIn("表示: 11/22/33/44/55/66/77", self.text("#ocrMockHint"))
        self.assertEqual(self.rb.log[-1], ("/api/survey/ocr_mock", {"action": "set", "values": VALID7}))
        self.assertEqual(self.errors, [])

    def test_ocr_set_not_applied_times_out_with_clear_message(self):
        self.rb.ocr_applies = False
        self.rb.ack_seq = 1000                                       # 古いack(送ったseqより小さい)のまま
        self.open_ocr()
        self.page.fill(".ocrm-in >> nth=0", "777")
        self.click("ocrMockSetBtn")
        self.wait_text("#ocrMockHint", "反映が確認できません", 30000)
        self.assertIn("模擬画面を開いていますか", self.text("#ocrMockHint"))
        self.assertNotIn("反映されました", self.text("#ocrMockHint"))

    def test_ocr_screen_showing_other_numbers_is_reported_as_mismatch(self):
        self.rb.ocr_override = [1, 1, 1, 1, 1, 1, 1]
        self.open_ocr()
        for k, v in enumerate(VALID7):
            self.page.fill(f".ocrm-in >> nth={k}", str(v))
        self.click("ocrMockSetBtn")
        self.wait_text("#ocrMockHint", "指定と違います", 30000)
        self.assertIn("表示: 1/1/1/1/1/1/1", self.text("#ocrMockHint"))

    def test_ocr_randomize_copies_actual_display_into_inputs(self):
        self.open_ocr()
        self.click("ocrMockRandBtn")
        self.wait_text("#ocrMockHint", "模擬画面に反映されました")
        vals = self.page.evaluate("Array.from(document.querySelectorAll('.ocrm-in')).map(e => e.value)")
        self.assertEqual(vals, ["3", "14", "159", "26", "5", "35", "89"])
        self.assertIn("表示: 3/14/159/26/5/35/89", self.text("#ocrMockHint"))

    def test_ocr_randomize_not_applied_is_reported(self):
        self.rb.ocr_applies = False
        self.open_ocr()
        self.click("ocrMockRandBtn")
        self.wait_text("#ocrMockHint", "反映が確認できません", 30000)

    def test_ocr_reset_confirmed(self):
        self.open_ocr()
        self.page.fill(".ocrm-in >> nth=3", "5")
        self.click("ocrMockResetBtn")
        self.wait_text("#ocrMockHint", "模擬画面に反映されました")
        self.assertIn("表示: 118/129/353/166/211/178/103", self.text("#ocrMockHint"))

    # ---- 3. 混雑判定の背景キャリブレーション ----
    def open_cal(self):
        self.open("#tab-settings", "() => !!document.getElementById('calibrateBgBtn') && !!window.NKField")

    def test_calibration_done_is_confirmed(self):
        self.open_cal()
        self.click("calibrateBgBtn")
        self.wait_text("#calibrateBgHint", "記録しました(確認済み", 20000)
        self.assertIn("1234", self.text("#calibrateBgHint"))
        self.assertEqual(self.errors, [])

    def test_calibration_failed_shows_reason(self):
        self.rb.crowd = "failed"
        self.open_cal()
        self.click("calibrateBgBtn")
        self.wait_text("#calibrateBgHint", "記録に失敗しました: 背景ファイルの保存に失敗", 20000)

    def test_calibration_without_crowd_monitor_is_not_called_success(self):
        self.rb.crowd = "silent"
        self.open_cal()
        self.click("calibrateBgBtn")
        self.wait_text("#calibrateBgHint", "crowd_monitorが応答しません", 40000)
        self.assertIn("届いたか確認できません", self.text("#calibrateBgHint"))
        self.assertNotIn("記録しました", self.text("#calibrateBgHint"))

    def test_calibration_accepted_but_no_scan(self):
        self.rb.crowd = "started"
        self.open_cal()
        self.click("calibrateBgBtn")
        self.wait_text("#calibrateBgHint", "完了を確認できません", 40000)
        self.assertNotIn("記録しました", self.text("#calibrateBgHint"))

    def test_calibration_ignores_result_of_other_request(self):
        # 前回の結果(別のrequest_id)が残っていても、今回の要求の結果と取り違えない
        self.rb.crowd = "stale"
        self.open_cal()
        self.click("calibrateBgBtn")
        self.wait_text("#calibrateBgHint", "crowd_monitorが応答しません", 40000)
        self.assertNotIn("記録しました", self.text("#calibrateBgHint"))


class StaticReadbackTests(unittest.TestCase):
    def test_stage2_page_also_waits_for_request_id(self):
        html = tfs.PAGES["stage2"]
        self.assertIn("/api/crowd_background/state", html)
        self.assertIn("request_id", html)
        self.assertIn("crowd_monitorが応答しません", html)

    def test_full_page_uses_nkfield_transactions(self):
        html = tfs.PAGES["full"]
        for key in ('NKField.begin("recommend"', 'NKField.begin("ocrmock"', 'NKField.begin("crowdcal"'):
            self.assertIn(key, html)
        self.assertNotIn("指定した値を表示しました", html)           # 「送りました」だけで成功と言う文言は残さない
        self.assertNotIn("ランダム化しました(1秒ほどで画面に反映されます)", html)

    def test_wrapper_original_is_untouched_and_ack_only_added(self):
        orig = (RASPI / "ocr_test" / "mock_pos_tablet_original.html").read_bytes()
        self.assertNotIn(b"ocr_mock_ack", orig)
        wrapper = (RASPI / "ocr_test" / "mock_pos_tablet.html").read_text(encoding="utf-8")
        self.assertIn("http://127.0.0.1:8083/ocr_mock_ack", wrapper)
        self.assertIn("pollCmd", wrapper)


if __name__ == "__main__":
    unittest.main()
