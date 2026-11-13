"""アンケート管理(頭部ディスプレイの表示切替・回答件数・回答CSV)のテスト。

3つの部品をまとめて確認する(rclpy不要、ロボットには何も送らない)。
1. ラズパイ側 scripts/raspi_display/survey/receiver.py: 送信元判定(198.51.100.x からは GET /state だけ)、
   セッション(active)の放棄タイムアウト、/state の内容、ループバックからの回答POST・件数。
   [2026-10-09] 回答中は「保存完了」まで続く: session_state(idle/answering/saved/abandoned)の遷移、
   保存の成功/失敗(失敗なら answering のまま)、saved_seq、?replay=1 の再送は完了にしない。
2. ラズパイ側 scripts/raspi_display/kiosk_admin.py: 呼び出し元を 198.51.100.2 のみに制限する判定、
   POST /mode のボディ検証、切替・状態・CSV(切替スクリプトは差し替えて実際には実行しない)。
3. 1台目側 survey_admin_proxy.py と、ダッシュボードの /api/survey/*(中継のエラー処理、
   guestは閲覧のみ、未ログイン401、HTMLにカードが入っていること)。

実行: python ros2_ws/src/nokosahen_core/test/test_survey_admin.py
"""

import importlib.util
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
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
sys.path.insert(0, str(PKG))

# ---- ラズパイ側モジュールの読み込み(環境変数でデータ置き場をテンポラリへ向ける) ----
_TMP = tempfile.mkdtemp(prefix="survey_admin_test_")
os.environ["SURVEY_DATA_DIR"] = os.path.join(_TMP, "data")
os.environ["KIOSK_MODE_FILE"] = os.path.join(_TMP, "kiosk_mode")


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


receiver = _load("survey_receiver", RASPI / "survey" / "receiver.py")
kiosk_admin = _load("kiosk_admin", RASPI / "kiosk_admin.py")


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
from nokosahen_core.survey_admin_proxy import (  # noqa: E402
    MAX_CSV_BYTES, SurveyAdminProxy, validate_mode_request)


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


# ======================================================================
# 1. receiver.py
# ======================================================================
class ReceiverLogicTest(unittest.TestCase):
    def test_allowed_hub_can_only_read_state(self):
        a = receiver.allowed
        self.assertTrue(a("198.51.100.2", "GET", "/state"))
        self.assertTrue(a("198.51.100.3", "GET", "/state?x=1"))
        for method, path in [("POST", "/api/responses"), ("POST", "/session/start"), ("GET", "/api/session/start"),
                             ("GET", "/api/responses.csv"), ("GET", "/"), ("POST", "/state"),
                             ("GET", "/api/condition?c=x"), ("GET", "/api/status")]:
            self.assertFalse(a("198.51.100.2", method, path), (method, path))

    def test_allowed_loopback_everything_others_nothing(self):
        a = receiver.allowed
        self.assertTrue(a("127.0.0.1", "POST", "/api/responses"))
        self.assertTrue(a("::1", "POST", "/session/end"))
        self.assertTrue(a("127.0.0.1", "GET", "/"))
        for ip in ("203.0.113.6", "198.51.100.12", "8.8.8.8", "", "garbage", None):
            self.assertFalse(a(ip, "GET", "/state"), ip)
            self.assertFalse(a(ip, "POST", "/api/responses"), ip)

    def test_session_times_out_to_inactive(self):
        t = [1000.0]
        s = receiver.Session(clock=lambda: t[0], timeout=45.0)
        self.assertFalse(s.active())
        s.start("条件A")
        self.assertTrue(s.active())
        t[0] += 44
        self.assertTrue(s.active())
        t[0] += 2
        self.assertFalse(s.active())  # 通知が途絶えたら安全側(false)に戻る
        s.start()
        t[0] += 30
        s.start()                       # heartbeat
        t[0] += 30
        self.assertTrue(s.active())
        s.end()
        self.assertFalse(s.active())
        self.assertEqual(s.condition, "条件A")

    def test_default_abandon_timeout_is_120s_and_env_overrides(self):
        self.assertEqual(receiver.SESSION_TIMEOUT_S, 120.0)
        self.assertEqual(receiver.Session().timeout, 120.0)
        os.environ["SURVEY_ABANDON_TIMEOUT_S"] = "300"
        try:
            self.assertEqual(_load("survey_receiver_env", RASPI / "survey" / "receiver.py").SESSION_TIMEOUT_S, 300.0)
            os.environ["SURVEY_ABANDON_TIMEOUT_S"] = "abc"      # 壊れた値は既定へ
            self.assertEqual(_load("survey_receiver_env2", RASPI / "survey" / "receiver.py").SESSION_TIMEOUT_S, 120.0)
            os.environ["SURVEY_ABANDON_TIMEOUT_S"] = "-5"
            self.assertEqual(_load("survey_receiver_env3", RASPI / "survey" / "receiver.py").SESSION_TIMEOUT_S, 120.0)
        finally:
            del os.environ["SURVEY_ABANDON_TIMEOUT_S"]

    def make_session(self, timeout=120.0):
        t = [1000.0]
        return receiver.Session(clock=lambda: t[0], timeout=timeout), t

    def test_state_machine_idle_answering_saved_idle(self):
        s, t = self.make_session()
        self.assertEqual(s.state(), "idle")
        s.start("A")
        self.assertEqual(s.state(), "answering")
        self.assertTrue(s.active())
        t[0] += 100                     # 考え込んでいる間(45秒を超えても)は answering のまま
        self.assertTrue(s.active())
        s.saved()
        self.assertEqual((s.state(), s.active(), s.saved_seq), ("saved", False, 1))
        t[0] += receiver.SAVED_SHOW_S - 0.5
        self.assertEqual(s.state(), "saved")
        t[0] += 1.0
        self.assertEqual(s.state(), "idle")  # savedは短い間だけ
        self.assertEqual(s.saved_seq, 1)     # カウンタは減らない

    def test_abandon_timeout_then_idle(self):
        s, t = self.make_session(timeout=120.0)
        s.start()
        t[0] += 119
        self.assertEqual(s.state(), "answering")
        s.start()                       # heartbeat で延長
        t[0] += 119
        self.assertEqual(s.state(), "answering")
        t[0] += 2
        self.assertEqual((s.state(), s.active()), ("abandoned", False))
        t[0] += receiver.ABANDONED_SHOW_S
        self.assertEqual(s.state(), "idle")

    def test_end_while_answering_is_abandoned(self):
        s, t = self.make_session()
        s.start()
        s.end()
        self.assertEqual(s.state(), "abandoned")
        s.end()                         # idle側でのendは何も変えない
        t[0] += 100
        self.assertEqual(s.state(), "idle")
        s.end()
        self.assertEqual(s.state(), "idle")

    def test_end_does_not_cancel_saved(self):
        s, t = self.make_session()
        s.start()
        s.saved()
        s.end()
        self.assertEqual(s.state(), "saved")

    def test_saved_without_answering_only_counts(self):
        # 回答中でない時の保存(再送の成功など)は saved_seq が増えるだけで saved 状態にはしない
        s, t = self.make_session()
        s.saved()
        self.assertEqual((s.state(), s.saved_seq), ("idle", 1))

    def test_restart_after_saved_and_after_abandoned(self):
        s, t = self.make_session()
        s.start()
        s.saved()
        s.start()                       # 次の人がすぐ始めた
        self.assertEqual(s.state(), "answering")
        t[0] += 125
        self.assertEqual(s.state(), "abandoned")
        s.start()
        self.assertEqual(s.state(), "answering")

    def test_save_record_success_duplicate_and_failure(self):
        os.makedirs(receiver.DATA, exist_ok=True)
        for p in (receiver.JSONL, receiver.CSV_PATH):
            if os.path.exists(p):
                os.remove(p)
        ok, dup, err = receiver.save_record({"response_id": "a", "answers": {"q1": "x"}})
        self.assertEqual((ok, dup, err), (True, False, None))
        self.assertEqual(receiver.count_responses(), 1)
        self.assertTrue(os.path.exists(receiver.CSV_PATH))
        self.assertEqual(receiver.save_record({"response_id": "a", "answers": {}}), (True, True, None))
        self.assertEqual(receiver.count_responses(), 1)  # 二重に書かない
        # CSVが書けない → 失敗(ok=False)。JSONLには残っているので、再送は duplicate で成功し CSV も直る
        with mock.patch.object(receiver, "write_csv", side_effect=OSError("disk full")):
            ok, dup, err = receiver.save_record({"response_id": "b", "answers": {}})
        self.assertFalse(ok)
        self.assertIn("disk full", err)
        os.remove(receiver.CSV_PATH)
        self.assertEqual(receiver.save_record({"response_id": "b", "answers": {}}), (True, True, None))
        self.assertTrue(os.path.exists(receiver.CSV_PATH))
        self.assertEqual(receiver.count_responses(), 2)
        # JSONLが書けない(パスがディレクトリ) → 失敗。例外は投げない
        with mock.patch.object(receiver, "JSONL", receiver.DATA):
            ok, dup, err = receiver.save_record({"response_id": "c", "answers": {}})
        self.assertFalse(ok)
        self.assertTrue(err)
        self.assertEqual(receiver.count_responses(), 2)

    def test_query_param(self):
        self.assertEqual(receiver.query_param("/api/session/start?condition=%E5%81%9C%E6%AD%A2", "condition"), "停止")
        self.assertIsNone(receiver.query_param("/x", "condition"))


class ReceiverServerTest(unittest.TestCase):
    """実際にHTTPで /state・回答POST・セッションを確認(テストからは127.0.0.1なので書き込みも通る)"""

    @classmethod
    def setUpClass(cls):
        os.makedirs(receiver.DATA, exist_ok=True)
        cls.srv, cls.port = serve(receiver.Handler)
        cls.base = f"http://127.0.0.1:{cls.port}"

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def setUp(self):
        for p in (receiver.JSONL, receiver.CSV_PATH, receiver.COND_PATH, receiver.MODE_FILE):
            if os.path.exists(p):
                os.remove(p)
        receiver.session.__init__()  # 状態を初期化

    def state(self):
        code, d = jhttp("GET", self.base + "/state")
        self.assertEqual(code, 200)
        return d

    def test_state_zero_count_and_defaults(self):
        d = self.state()
        self.assertEqual(set(d), {"mode", "active", "count", "last_activity_age_s", "condition", "updated",
                                  "session_state", "saved_seq", "last_saved_age_s"})
        self.assertEqual((d["session_state"], d["saved_seq"], d["last_saved_age_s"]), ("idle", 0, None))
        self.assertEqual(d["count"], 0)          # 回答0件でも 0 を返す
        self.assertEqual(d["mode"], "customer")  # モードファイルが無ければ customer
        self.assertFalse(d["active"])
        self.assertIsNone(d["last_activity_age_s"])
        self.assertIsNone(d["condition"])
        self.assertIsInstance(d["updated"], int)

    def test_mode_follows_file(self):
        Path(receiver.MODE_FILE).write_text("survey\n", encoding="utf-8")
        self.assertEqual(self.state()["mode"], "survey")
        Path(receiver.MODE_FILE).write_text("junk", encoding="utf-8")
        self.assertEqual(self.state()["mode"], "customer")

    def test_session_start_end_post_and_get(self):
        code, _ = jhttp("POST", self.base + "/session/start?condition=%E5%81%9C%E6%AD%A2", raw=b"")
        self.assertEqual(code, 200)
        d = self.state()
        self.assertTrue(d["active"])
        self.assertEqual(d["condition"], "停止")
        self.assertIsNotNone(d["last_activity_age_s"])
        jhttp("GET", self.base + "/api/session/end")  # 既存HTMLが使うGET経路
        self.assertFalse(self.state()["active"])
        jhttp("GET", self.base + "/api/session/start")
        self.assertTrue(self.state()["active"])
        jhttp("POST", self.base + "/session/end", raw=b"")
        self.assertFalse(self.state()["active"])

    def test_answer_until_saved_flow(self):
        # 最初のタッチ(start) → answering(active=true) → 回答POST成功 → saved(active=false, saved_seq増加)
        jhttp("GET", self.base + "/api/session/start")
        d = self.state()
        self.assertEqual((d["session_state"], d["active"], d["saved_seq"]), ("answering", True, 0))
        jhttp("GET", self.base + "/api/session/start")   # 5秒ごとの継続通知
        self.assertEqual(self.state()["session_state"], "answering")
        code, r = jhttp("POST", self.base + "/api/responses",
                        body={"response_id": "f1", "condition": "x", "answers": {"q1": "a"}})
        self.assertEqual((code, r), (200, {"ok": True}))
        d = self.state()
        self.assertEqual((d["session_state"], d["active"], d["saved_seq"], d["count"]), ("saved", False, 1, 1))
        self.assertIsNotNone(d["last_saved_age_s"])
        self.assertLess(d["last_saved_age_s"], 5)

    def test_save_failure_keeps_answering(self):
        jhttp("GET", self.base + "/api/session/start")
        with mock.patch.object(receiver, "write_csv", side_effect=OSError("disk full")):
            code, r = jhttp("POST", self.base + "/api/responses", body={"response_id": "g1", "answers": {}})
        self.assertEqual(code, 500)
        self.assertFalse(r["ok"])
        self.assertIn("disk full", r["error"])
        d = self.state()
        self.assertEqual((d["session_state"], d["active"], d["saved_seq"]), ("answering", True, 0))
        # 直ったあとのHTMLの再送で保存完了になる(JSONLには残っているので二重保存にはならない)
        code, r = jhttp("POST", self.base + "/api/responses", body={"response_id": "g1", "answers": {}})
        self.assertEqual((code, r["ok"], r.get("duplicate")), (200, True, True))
        d = self.state()
        self.assertEqual((d["session_state"], d["active"], d["saved_seq"], d["count"]), ("saved", False, 1, 1))

    def test_unwritable_jsonl_keeps_answering(self):
        jhttp("GET", self.base + "/api/session/start")
        with mock.patch.object(receiver, "JSONL", receiver.DATA):  # ディレクトリには書けない
            code, r = jhttp("POST", self.base + "/api/responses", body={"response_id": "h1", "answers": {}})
        self.assertEqual((code, r["ok"]), (500, False))
        self.assertEqual(self.state()["session_state"], "answering")

    def test_bad_body_is_400_and_keeps_answering(self):
        jhttp("GET", self.base + "/api/session/start")
        code, r = jhttp("POST", self.base + "/api/responses", raw=b"not json")
        self.assertEqual((code, r["ok"]), (400, False))
        code, r = jhttp("POST", self.base + "/api/responses", raw=b"[1,2]")
        self.assertEqual((code, r["ok"]), (400, False))
        d = self.state()
        self.assertEqual((d["session_state"], d["saved_seq"]), ("answering", 0))

    def test_replay_does_not_complete_current_session(self):
        jhttp("GET", self.base + "/api/session/start")
        code, r = jhttp("POST", self.base + "/api/responses?replay=1", body={"response_id": "old1", "answers": {}})
        self.assertEqual((code, r["ok"]), (200, True))
        d = self.state()
        self.assertEqual((d["session_state"], d["saved_seq"], d["count"]), ("answering", 0, 1))  # 保存はしたが完了通知にしない

    def test_abandon_timeout_and_saved_decay_over_http(self):
        t = [5000.0]
        receiver.session.clock = lambda: t[0]
        receiver.session.timeout = 120.0
        jhttp("GET", self.base + "/api/session/start")
        t[0] += 60
        self.assertEqual(self.state()["session_state"], "answering")   # 45秒を超えても回答中のまま
        t[0] += 61
        d = self.state()
        self.assertEqual((d["session_state"], d["active"]), ("abandoned", False))
        t[0] += receiver.ABANDONED_SHOW_S
        self.assertEqual(self.state()["session_state"], "idle")
        jhttp("GET", self.base + "/api/session/start")
        jhttp("POST", self.base + "/api/responses", body={"response_id": "k1", "answers": {}})
        self.assertEqual(self.state()["session_state"], "saved")
        t[0] += receiver.SAVED_SHOW_S + 0.1
        d = self.state()
        self.assertEqual((d["session_state"], d["saved_seq"]), ("idle", 1))

    def test_end_post_while_answering_is_abandoned(self):
        jhttp("GET", self.base + "/api/session/start")
        jhttp("GET", self.base + "/api/session/end")
        d = self.state()
        self.assertEqual((d["session_state"], d["active"]), ("abandoned", False))

    def test_condition_notify_persisted(self):
        jhttp("GET", self.base + "/api/condition?c=%E7%A7%BB%E5%8B%95")
        self.assertEqual(self.state()["condition"], "移動")
        self.assertEqual(Path(receiver.COND_PATH).read_text(encoding="utf-8"), "移動")

    def test_count_after_responses_and_duplicates(self):
        for i in range(3):
            code, d = jhttp("POST", self.base + "/api/responses",
                            body={"response_id": f"r{i}", "condition": "x", "answers": {"q1": "a"}})
            self.assertEqual((code, d["ok"]), (200, True))
        jhttp("POST", self.base + "/api/responses", body={"response_id": "r1", "answers": {}})  # 重複
        self.assertEqual(self.state()["count"], 3)
        code, d = jhttp("GET", self.base + "/api/status")
        self.assertEqual(d["count"], 3)
        code, body, _ = http("GET", self.base + "/api/responses.csv")
        self.assertEqual(code, 200)
        self.assertEqual(body.decode("utf-8-sig").count("\n"), 4)  # ヘッダ+3行


# ======================================================================
# 2. kiosk_admin.py
# ======================================================================
class KioskAdminLogicTest(unittest.TestCase):
    def test_client_allowed(self):
        c = kiosk_admin.client_allowed
        self.assertTrue(c("198.51.100.2"))
        self.assertTrue(c("::ffff:198.51.100.2"))
        for ip in ("198.51.100.1", "198.51.100.3", "198.51.100.22", "127.0.0.1", "203.0.113.6", "", "x", None):
            self.assertFalse(c(ip), ip)
        self.assertTrue(c("127.0.0.1", {"127.0.0.1"}))

    def test_parse_mode_body(self):
        p = kiosk_admin.parse_mode_body
        self.assertEqual(p(b'{"mode":"survey"}'), ("survey", None))
        self.assertEqual(p(b'{"mode": "customer"}'), ("customer", None))
        self.assertEqual(p(b'{"mode":"ocrtest"}'), ("ocrtest", None))
        self.assertIsNone(p(b'{"mode":"OCRTEST"}')[0])
        for raw in (b'', b'not json', b'[]', b'{}', b'{"mode":"status"}', b'{"mode":"survey; rm -rf ~"}',
                    b'{"mode":["survey"]}', b'{"mode":1}', b'{"mode":null}',
                    b'{"mode":"survey","x":1}', b'{"Mode":"survey"}', b'\xff\xfe', b'{"mode":"' + b'a' * 400 + b'"}', None):
            m, err = p(raw)
            self.assertIsNone(m, raw)
            self.assertTrue(err, raw)


class KioskAdminServerTest(unittest.TestCase):
    """テストのクライアントは127.0.0.1。許可リストを切り替えて 403/200 を確認する"""

    @classmethod
    def setUpClass(cls):
        cls.srv, cls.port = serve(kiosk_admin.Handler)
        cls.base = f"http://127.0.0.1:{cls.port}"
        # receiver 本体を別ポートで立て、kiosk_admin の RECEIVER に向ける
        cls.rsrv, cls.rport = serve(receiver.Handler)
        kiosk_admin.RECEIVER = f"http://127.0.0.1:{cls.rport}"

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()
        cls.rsrv.shutdown()
        cls.rsrv.server_close()

    def setUp(self):
        self._allowed = set(kiosk_admin.ALLOWED_CLIENTS)
        kiosk_admin.ALLOWED_CLIENTS = {"127.0.0.1"}
        os.makedirs(receiver.DATA, exist_ok=True)
        for p in (receiver.JSONL, receiver.CSV_PATH, receiver.COND_PATH, receiver.MODE_FILE):
            if os.path.exists(p):
                os.remove(p)
        receiver.session.__init__()
        self.calls = []

        def fake_run(cmd, **kw):
            self.calls.append(cmd)
            Path(kiosk_admin.MODE_FILE).write_text(cmd[-1] + "\n", encoding="utf-8")  # switch_kiosk.sh の効果
            return subprocess.CompletedProcess(cmd, 0, b"", b"")

        self._p = mock.patch.object(kiosk_admin.subprocess, "run", fake_run)
        self._p.start()

    def tearDown(self):
        self._p.stop()
        kiosk_admin.ALLOWED_CLIENTS = self._allowed

    def test_forbidden_for_other_sources(self):
        kiosk_admin.ALLOWED_CLIENTS = {"198.51.100.2"}  # テストのクライアント(127.0.0.1)は対象外になる
        self.assertEqual(jhttp("GET", self.base + "/state")[0], 403)
        self.assertEqual(jhttp("POST", self.base + "/mode", body={"mode": "survey"})[0], 403)
        self.assertEqual(http("GET", self.base + "/responses.csv")[0], 403)
        self.assertEqual(self.calls, [])  # 切替スクリプトは呼ばれない

    def test_state_with_and_without_receiver(self):
        Path(receiver.MODE_FILE).write_text("survey\n", encoding="utf-8")
        code, d = jhttp("GET", self.base + "/state")
        self.assertEqual((code, d["ok"], d["receiver"], d["mode"], d["count"]), (200, True, True, "survey", 0))
        with mock.patch.object(kiosk_admin, "RECEIVER", "http://127.0.0.1:9"):  # 届かない
            Path(kiosk_admin.DATA).mkdir(parents=True, exist_ok=True)
            Path(kiosk_admin.DATA, "responses.jsonl").write_text('{"a":1}\n\n{"a":2}\n', encoding="utf-8")
            code, d = jhttp("GET", self.base + "/state")
        self.assertEqual((code, d["receiver"], d["count"], d["mode"], d["active"]), (200, False, 2, "survey", False))
        self.assertEqual((d["session_state"], d["saved_seq"]), ("idle", None))  # receiver不在 → 待機扱い(止めない)

    def test_state_mirrors_session_fields(self):
        jhttp("GET", f"http://127.0.0.1:{self.rport}/api/session/start")
        code, d = jhttp("GET", self.base + "/state")
        self.assertEqual((d["session_state"], d["active"], d["saved_seq"]), ("answering", True, 0))

    def test_mode_switch_valid(self):
        code, d = jhttp("POST", self.base + "/mode", body={"mode": "survey"})
        self.assertEqual((code, d["ok"], d["changed"], d["mode"]), (200, True, True, "survey"))
        self.assertEqual(self.calls, [[kiosk_admin.SWITCH, "survey"]])
        code, d = jhttp("POST", self.base + "/mode", body={"mode": "survey"})  # 同じモード: 何もしない
        self.assertEqual((code, d["changed"]), (200, False))
        self.assertEqual(len(self.calls), 1)
        code, d = jhttp("POST", self.base + "/mode", body={"mode": "customer"})
        self.assertEqual((d["changed"], d["mode"]), (True, "customer"))
        code, d = jhttp("POST", self.base + "/mode", body={"mode": "ocrtest"})  # OCRテスト用の模擬POS
        self.assertEqual((code, d["changed"], d["mode"]), (200, True, "ocrtest"))
        code, d = jhttp("GET", self.base + "/state")
        self.assertEqual(d["mode"], "ocrtest")  # receiverはsurvey/customerしか返さないので管理APIが上書き

    def test_mode_switch_rejects_bad_input(self):
        for raw in (b'', b'{"mode":"status"}', b'{"mode":"survey","x":1}', b'garbage', b'[1]', b'{"mode":"a b"}'):
            code, d = jhttp("POST", self.base + "/mode", raw=raw, headers={"Content-Type": "application/json"})
            self.assertEqual(code, 400, raw)
            self.assertFalse(d["ok"])
        code, _ = jhttp("POST", self.base + "/mode", raw=b'{"mode":"survey"}' + b' ' * 400)  # 長すぎる
        self.assertEqual(code, 400)
        self.assertEqual(jhttp("POST", self.base + "/other", body={"mode": "survey"})[0], 404)
        self.assertEqual(self.calls, [])

    def test_mode_switch_script_failure(self):
        self._p.stop()
        with mock.patch.object(kiosk_admin.subprocess, "run",
                               lambda cmd, **kw: subprocess.CompletedProcess(cmd, 3, b"", b"")):
            code, d = jhttp("POST", self.base + "/mode", body={"mode": "survey"})
        self.assertEqual((code, d["ok"]), (500, False))
        self._p.start()

    def test_csv_relay_and_missing(self):
        jhttp("POST", f"http://127.0.0.1:{self.rport}/api/responses", body={"response_id": "z", "answers": {"q": "1"}})
        code, body, hdr = http("GET", self.base + "/responses.csv")
        self.assertEqual(code, 200)
        self.assertIn("response_id", body.decode("utf-8-sig"))
        self.assertIn("text/csv", hdr.get("Content-Type", ""))


# ======================================================================
# 3. 1台目: SurveyAdminProxy とダッシュボードの中継
# ======================================================================
class _PiStub(BaseHTTPRequestHandler):
    state = {"mode": "customer", "active": False, "count": 5, "last_activity_age_s": 12.5,
             "condition": "停止 + 音声", "updated": 1, "receiver": True,
             "session_state": "answering", "saved_seq": 2, "last_saved_age_s": 30.0}
    csv = "response_id,q1\r\nr1,a\r\n".encode("utf-8")
    fail = 0           # 0以外ならそのHTTPステータスを返す
    garbage = False
    hits = []

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        _PiStub.hits.append(("GET", self.path))
        if _PiStub.fail:
            return self._send(_PiStub.fail, b"{}")
        if _PiStub.garbage:
            return self._send(200, b"not json")
        if self.path == "/state":
            return self._send(200, json.dumps(_PiStub.state).encode())
        if self.path == "/responses.csv":
            return self._send(200, _PiStub.csv, "text/csv")
        self._send(404, b"{}")

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(n).decode())
        _PiStub.hits.append(("POST", self.path, body))
        if _PiStub.fail:
            return self._send(_PiStub.fail, b"{}")
        _PiStub.state = dict(_PiStub.state, mode=body["mode"])
        self._send(200, json.dumps(dict(_PiStub.state, ok=True, changed=True)).encode())


class ProxyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv, cls.port = serve(_PiStub)
        cls.url = f"http://127.0.0.1:{cls.port}"

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def setUp(self):
        _PiStub.fail = 0
        _PiStub.garbage = False
        _PiStub.hits = []
        _PiStub.state = dict(_PiStub.state, mode="customer")

    def test_unconfigured(self):
        p = SurveyAdminProxy("")
        self.assertFalse(p.configured)
        self.assertFalse(p.get_state()["configured"])
        self.assertEqual(p.set_mode({"mode": "survey"})[1]["configured"], False)
        self.assertEqual(p.get_csv()[0], 404)

    def test_state_passes_session_fields_through(self):
        r = SurveyAdminProxy(self.url).get_state()
        self.assertEqual((r["state"]["session_state"], r["state"]["saved_seq"]), ("answering", 2))

    def test_get_state_ok_and_cache(self):
        t = [0.0]
        p = SurveyAdminProxy(self.url, clock=lambda: t[0])
        r = p.get_state()
        self.assertTrue(r["ok"])
        self.assertEqual(r["state"]["count"], 5)
        p.get_state()
        self.assertEqual(len(_PiStub.hits), 1)   # 2秒以内はキャッシュ
        t[0] += 3
        p.get_state()
        self.assertEqual(len(_PiStub.hits), 2)

    def test_unreachable_and_errors_are_reported_not_raised(self):
        p = SurveyAdminProxy("http://127.0.0.1:9", timeout_s=1.0)
        r = p.get_state()
        self.assertFalse(r["ok"])
        self.assertTrue(r["configured"])
        self.assertIn("届きません", r["message"])
        p = SurveyAdminProxy(self.url)
        _PiStub.fail = 500
        r = p.get_state()
        self.assertFalse(r["ok"])
        self.assertIn("500", r["message"])
        _PiStub.fail = 0
        _PiStub.garbage = True
        p = SurveyAdminProxy(self.url)
        self.assertFalse(p.get_state()["ok"])
        self.assertEqual(p.get_csv()[0], 200)  # CSV側のスタブは固定。garbageでも状態系だけ失敗

    def test_set_mode_validation_and_forward(self):
        p = SurveyAdminProxy(self.url)
        for bad in (None, {}, {"mode": "status"}, {"mode": "survey", "x": 1}, {"mode": 1}, [], "survey", {"mode": "a; b"}):
            code, r = p.set_mode(bad)
            self.assertEqual(code, 400, bad)
            self.assertFalse(r["ok"])
        self.assertEqual(_PiStub.hits, [])   # 不正な入力はラズパイに転送しない
        code, r = p.set_mode({"mode": "survey"})
        self.assertEqual((code, r["ok"], r["state"]["mode"]), (200, True, "survey"))
        self.assertEqual(_PiStub.hits[-1], ("POST", "/mode", {"mode": "survey"}))
        _PiStub.fail = 500
        code, r = p.set_mode({"mode": "customer"})
        self.assertEqual((code, r["ok"]), (200, False))

    def test_set_mode_drops_cache(self):
        p = SurveyAdminProxy(self.url)
        self.assertEqual(p.get_state()["state"]["mode"], "customer")
        p.set_mode({"mode": "survey"})
        self.assertEqual(p.get_state()["state"]["mode"], "survey")  # 切替直後に取り直す

    def test_csv(self):
        p = SurveyAdminProxy(self.url)
        code, body, err = p.get_csv()
        self.assertEqual((code, body, err), (200, _PiStub.csv, ""))
        _PiStub.fail = 500
        self.assertEqual(p.get_csv()[0], 502)
        _PiStub.fail = 0
        old = _PiStub.csv
        _PiStub.csv = b"x" * (MAX_CSV_BYTES + 10)
        try:
            code, body, err = p.get_csv()
        finally:
            _PiStub.csv = old
        self.assertEqual((code, body), (502, b""))
        self.assertIn("大きすぎ", err)
        self.assertEqual(SurveyAdminProxy("http://127.0.0.1:9", timeout_s=1.0).get_csv()[0], 502)

    def test_validate_mode_request(self):
        self.assertEqual(validate_mode_request({"mode": "customer"}), ("customer", ""))
        self.assertIsNone(validate_mode_request({"mode": "Customer"})[0])
        self.assertEqual(validate_mode_request({"mode": "ocrtest"}), ("ocrtest", ""))


class FakeTeleop:
    soft_estop_engaged = False


class DashboardApiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pi, cls.pi_port = serve(_PiStub)

    @classmethod
    def tearDownClass(cls):
        cls.pi.shutdown()
        cls.pi.server_close()

    def setUp(self):
        _PiStub.fail = 0
        _PiStub.garbage = False
        _PiStub.hits = []
        _PiStub.state = dict(_PiStub.state, mode="customer")
        self.proxy = SurveyAdminProxy(f"http://127.0.0.1:{self.pi_port}")
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, FakeTeleop(), survey_admin=self.proxy,
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

    def call(self, method, path, body=None, cookie=None, raw=False):
        h = {"Content-Type": "application/json"}
        c = self.tl if cookie is None else cookie
        if c:
            h["Cookie"] = c
        code, b, hdr = http(method, f"http://127.0.0.1:{self.port}{path}", body=body, headers=h)
        return (code, b, hdr) if raw else (code, json.loads(b.decode("utf-8")))

    def test_requires_login(self):
        self.assertEqual(self.call("GET", "/api/survey/state", cookie="")[0], 401)
        self.assertEqual(self.call("POST", "/api/survey/mode", {"mode": "survey"}, cookie="")[0], 401)
        self.assertEqual(self.call("GET", "/api/survey/responses.csv", cookie="", raw=True)[0], 401)

    def test_state_and_guest_can_view(self):
        for ck in (self.tl, self.guest):
            code, d = self.call("GET", "/api/survey/state", cookie=ck)
            self.assertEqual((code, d["ok"], d["state"]["count"]), (200, True, 5))

    def test_mode_switch_tl_only(self):
        code, d = self.call("POST", "/api/survey/mode", {"mode": "survey"})
        self.assertEqual((code, d["ok"], d["state"]["mode"]), (200, True, "survey"))
        self.assertEqual(self.call("POST", "/api/survey/mode", {"mode": "customer"}, cookie=self.guest)[0], 403)
        self.assertEqual(self.call("POST", "/api/survey/mode", {"mode": "bogus"})[0], 400)
        self.assertEqual(self.call("POST", "/api/survey/mode", {})[0], 400)
        posts = [h for h in _PiStub.hits if h[0] == "POST"]
        self.assertEqual(len(posts), 1)  # 拒否/不正はラズパイに届いていない

    def test_pi_down_is_error_json_not_exception(self):
        self.proxy.base_url = "http://127.0.0.1:9"
        self.proxy.timeout_s = 1.0
        self.proxy._cache = None
        code, d = self.call("GET", "/api/survey/state")
        self.assertEqual((code, d["ok"]), (200, False))
        self.assertIn("届きません", d["message"])
        code, d = self.call("POST", "/api/survey/mode", {"mode": "survey"})
        self.assertEqual((code, d["ok"]), (200, False))
        code, d = self.call("GET", "/api/survey/responses.csv")
        self.assertEqual(code, 502)

    def test_csv_download(self):
        code, body, hdr = self.call("GET", "/api/survey/responses.csv", raw=True)
        self.assertEqual(code, 200)
        self.assertEqual(body, _PiStub.csv)
        self.assertIn("attachment", hdr.get("Content-Disposition", ""))

    def test_unconfigured_hides_feature(self):
        self.proxy.base_url = ""
        self.assertFalse(self.call("GET", "/api/survey/state")[1]["configured"])
        self.assertEqual(self.call("GET", "/api/survey/responses.csv", raw=True)[0], 404)


class PageTest(unittest.TestCase):
    def test_card_in_settings_tab_and_js_parses(self):
        # rclpyが無い環境でも動くよう、import せずソースの文字列で確認する
        html = (PKG / "nokosahen_core" / "full_dashboard_node.py").read_text(encoding="utf-8")
        html = html[:html.index("MOBILE_HTML = ")]
        i_settings = html.index('id="tab-settings"')
        i_card = html.index('id="surveyBlock"')
        self.assertGreater(i_card, i_settings)
        self.assertLess(i_card, html.index('id="estopDock"'))
        for el in ("surveyMode", "surveyCount", "surveyActive", "surveySession", "surveyLast", "surveyCond", "surveyErr",
                   "surveyToSurveyBtn", "surveyToCustomerBtn", "surveyCsvLink", "surveyConfirm"):
            self.assertIn(f'id="{el}"', html)
        self.assertIn("回答中のためロボット停止中", html)
        self.assertIn("s.session_state", html)
        self.assertIn("/api/survey/state", html)
        self.assertIn("/api/survey/mode", html)
        self.assertIn('id="estopDock"', html)  # 緊急停止ドックは残っている


def tearDownModule():
    shutil.rmtree(_TMP, ignore_errors=True)


if __name__ == "__main__":
    unittest.main(verbosity=1)
