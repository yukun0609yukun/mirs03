"""survey_monitor(ラズパイのアンケート状態 -> /nokosahen/survey_active)の単体テスト。

rclpy不要(未インストール時はスタブ注入)。ラズパイ実機・ROSには一切触れない。
HTTPはこのプロセス内のスタブサーバー(127.0.0.1の空きポート)を使う。
実行: python ros2_ws/src/nokosahen_core/test/test_survey_monitor.py
"""

import json
import sys
import threading
import time
import types
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def _install_stubs():
    try:
        import rclpy  # noqa: F401
        import rclpy.qos  # noqa: F401
        import std_msgs.msg  # noqa: F401
        return
    except ImportError:
        pass

    class _Any:
        def __init__(self, *a, **k):
            self.data = None

    class _AutoMod(types.ModuleType):
        def __getattr__(self, item):
            if item.startswith("__"):
                raise AttributeError(item)
            return _Any

    for name in list(sys.modules):
        if name.split(".")[0] in ("rclpy", "std_msgs"):
            del sys.modules[name]
    for mod in ("rclpy", "rclpy.node", "rclpy.qos", "std_msgs", "std_msgs.msg"):
        sys.modules[mod] = _AutoMod(mod)


_install_stubs()

from nokosahen_core import survey_monitor as sm  # noqa: E402


class _State:
    body = b"{}"
    status = 200
    delay = 0.0


class _Handler(BaseHTTPRequestHandler):
    def do_GET(self):  # noqa: N802
        if _State.delay:
            time.sleep(_State.delay)
        if self.path != "/state":
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(_State.status)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        try:
            self.wfile.write(_State.body)
        except OSError:
            pass

    def log_message(self, *a):
        pass


def state_json(active=True, mode="survey", **extra):
    d = {"mode": mode, "active": active, "count": 3, "last_activity_age_s": 1.5,
         "condition": "A", "updated": 1.0e9}
    d.update(extra)
    return json.dumps(d).encode()


class FetchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), _Handler)
        cls.srv.daemon_threads = True
        cls.url = f"http://127.0.0.1:{cls.srv.server_address[1]}"
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def setUp(self):
        _State.body, _State.status, _State.delay = b"{}", 200, 0.0

    def test_active_true(self):
        _State.body = state_json(True)
        r = sm.fetch_survey_state(self.url)
        self.assertTrue(r["ok"])
        self.assertIs(r["active"], True)
        self.assertEqual((r["mode"], r["count"], r["age"], r["condition"]), ("survey", 3, 1.5, "A"))

    def test_active_false_and_null_age(self):
        _State.body = state_json(False, last_activity_age_s=None, condition=None)
        r = sm.fetch_survey_state(self.url + "/")  # 末尾スラッシュ可
        self.assertTrue(r["ok"])
        self.assertIs(r["active"], False)
        self.assertIsNone(r["age"])

    def test_customer_mode(self):
        _State.body = state_json(False, mode="customer")
        self.assertEqual(sm.fetch_survey_state(self.url)["mode"], "customer")

    def test_invalid_payloads_are_unknown(self):
        for body in (b"not json", b"[]", b"null", b"", b"{}", b'{"active":"true"}',
                     b'{"active":1}', b'{"active":null}', b"\xff\xfe"):
            _State.body = body
            r = sm.fetch_survey_state(self.url)
            self.assertFalse(r["ok"], body)

    def test_http_error_is_unknown(self):
        _State.status = 500
        _State.body = state_json(True)
        self.assertFalse(sm.fetch_survey_state(self.url)["ok"])

    def test_new_session_fields(self):
        _State.body = state_json(True, session_state="answering", saved_seq=4, last_saved_age_s=2.0)
        r = sm.fetch_survey_state(self.url)
        self.assertEqual((r["session_state"], r["saved_seq"]), ("answering", 4))

    def test_old_receiver_without_session_fields(self):
        _State.body = state_json(True)  # 古いラズパイ: session_state/saved_seq が無い
        r = sm.fetch_survey_state(self.url)
        self.assertTrue(r["ok"])
        self.assertIsNone(r["session_state"])
        self.assertIsNone(r["saved_seq"])

    def test_bad_session_field_types_ignored(self):
        _State.body = state_json(True, session_state=5, saved_seq="3")
        r = sm.fetch_survey_state(self.url)
        self.assertTrue(r["ok"])
        self.assertIsNone(r["session_state"])
        self.assertIsNone(r["saved_seq"])
        _State.body = state_json(True, saved_seq=True)  # boolはintではない
        self.assertIsNone(sm.fetch_survey_state(self.url)["saved_seq"])

    def test_timeout_is_unknown_and_bounded(self):
        _State.body = state_json(True)
        _State.delay = 2.0
        t0 = time.monotonic()
        r = sm.fetch_survey_state(self.url, timeout=0.3)
        self.assertFalse(r["ok"])
        self.assertLess(time.monotonic() - t0, 1.5)

    def test_unreachable_is_unknown(self):
        r = sm.fetch_survey_state("http://127.0.0.1:1", timeout=0.3)
        self.assertFalse(r["ok"])
        self.assertTrue(r["error"])


OK_T = {"ok": True, "active": True, "mode": "survey", "count": 1, "age": 0.5}
OK_F = {"ok": True, "active": False, "mode": "survey", "count": 1, "age": 9.0}
OK_C = {"ok": True, "active": False, "mode": "customer", "count": 0, "age": None}
ERR = {"ok": False, "error": "URLError"}


class GateTests(unittest.TestCase):
    def gate(self):
        return sm.SurveyGate(debounce_sec=1.5, unreachable_hold_sec=20.0)

    def run_for(self, g, sample, t0, seconds, step=0.2):
        out = []
        g.feed(sample)
        t = t0
        while t <= t0 + seconds + 1e-9:
            out.append((round(t, 2), g.tick(t)))
            t += step
        return out

    def test_no_response_yet_is_false(self):
        g = self.gate()
        self.assertFalse(g.tick(0.0))
        g.feed(ERR)
        self.assertFalse(g.tick(100.0))

    def test_true_immediately(self):
        g = self.gate()
        g.feed(OK_T)
        self.assertTrue(g.tick(0.0))

    def test_false_is_debounced(self):
        g = self.gate()
        g.feed(OK_T)
        g.tick(0.0)
        g.feed(OK_F)
        self.assertTrue(g.tick(0.2))  # false確認の最初
        self.assertTrue(g.tick(1.5))  # 0.2+1.3 < 1.5秒
        self.assertFalse(g.tick(1.8))  # 0.2+1.6 >= 1.5

    def test_true_during_debounce_resets(self):
        g = self.gate()
        g.feed(OK_T)
        g.tick(0.0)
        g.feed(OK_F)
        g.tick(0.2)
        g.feed(OK_T)
        self.assertTrue(g.tick(1.0))
        g.feed(OK_F)
        self.assertTrue(g.tick(1.2))   # デバウンスは新しく数え直し
        self.assertTrue(g.tick(2.5))
        self.assertFalse(g.tick(2.8))

    def test_unreachable_holds_then_releases(self):
        g = self.gate()
        g.feed(OK_T)
        g.tick(0.0)
        g.feed(ERR)
        for t in (0.2, 5.0, 19.9, 20.0):
            self.assertTrue(g.tick(t), t)
        self.assertFalse(g.tick(20.5))
        self.assertFalse(g.tick(21.0))  # 戻らない
        self.assertFalse(g.reachable)
        self.assertEqual(g.last_error, "URLError")

    def test_unreachable_hold_measured_from_last_true(self):
        g = self.gate()
        g.feed(OK_T)
        g.tick(0.0)
        g.tick(10.0)  # 10秒までtrueを確認し続けた
        g.feed(ERR)
        self.assertTrue(g.tick(29.0))
        self.assertFalse(g.tick(30.5))

    def test_recovery_after_unreachable_true_again(self):
        g = self.gate()
        g.feed(OK_T)
        g.tick(0.0)
        g.feed(ERR)
        g.tick(5.0)
        g.feed(OK_T)
        self.assertTrue(g.tick(5.2))
        self.assertTrue(g.reachable)
        self.assertIsNone(g.last_error)

    def test_unreachable_after_false_still_debounces_to_false(self):
        g = self.gate()
        g.feed(OK_T)
        g.tick(0.0)
        g.feed(OK_F)
        g.tick(0.2)
        g.feed(ERR)  # falseを確認したあとに途切れた: デバウンスで解除(20秒ホールドしない)
        self.assertTrue(g.tick(1.0))
        self.assertFalse(g.tick(1.8))

    def test_unreachable_when_false_stays_false(self):
        g = self.gate()
        g.feed(OK_F)
        g.tick(0.0)
        g.feed(ERR)
        self.assertFalse(g.tick(1.0))

    def test_customer_mode_always_false_immediately(self):
        g = self.gate()
        g.feed(OK_T)
        self.assertTrue(g.tick(0.0))
        g.feed(OK_C)
        self.assertFalse(g.tick(0.2))  # デバウンス無し
        c = dict(OK_C, active=True)    # 矛盾した応答でもcustomerならfalse
        g.feed(c)
        self.assertFalse(g.tick(0.4))

    def test_invalid_json_treated_as_unreachable(self):
        g = self.gate()
        g.feed(OK_T)
        g.tick(0.0)
        g.feed({"ok": False, "error": "invalid_json"})
        self.assertTrue(g.tick(10.0))
        self.assertFalse(g.tick(25.0))
        self.assertEqual(g.last_error, "invalid_json")

    def test_negative_params_clamped(self):
        g = sm.SurveyGate(-1.0, -5.0)
        self.assertEqual((g.debounce_sec, g.unreachable_hold_sec), (0.0, 0.0))


def ANS(seq=0):
    return {"ok": True, "active": True, "mode": "survey", "count": 1, "age": 0.5,
            "session_state": "answering", "saved_seq": seq}


def SAVED(seq=1):
    return {"ok": True, "active": False, "mode": "survey", "count": 2, "age": 0.1,
            "session_state": "saved", "saved_seq": seq}


def IDLE(seq=1):
    return {"ok": True, "active": False, "mode": "survey", "count": 2, "age": 9.0,
            "session_state": "idle", "saved_seq": seq}


def ABANDONED(seq=0):
    return {"ok": True, "active": False, "mode": "survey", "count": 1, "age": 120.0,
            "session_state": "abandoned", "saved_seq": seq}


class SaveGateTests(unittest.TestCase):
    """2026-10-09: 回答中は保存完了まで保持。保存後は短く保持して解除。放棄・到達不能の上限。"""

    def gate(self):
        return sm.SurveyGate(debounce_sec=1.5, unreachable_hold_sec=20.0,
                             answering_unreachable_hold_sec=60.0, post_save_hold_sec=1.5)

    def test_answering_holds_for_a_long_time(self):
        g = self.gate()
        g.feed(ANS())
        for t in range(0, 300, 5):  # 回答中のsampleが続く間はずっとtrue(解除は保存完了か放棄だけ)
            self.assertTrue(g.tick(float(t)))

    def test_saved_then_short_hold_then_release(self):
        g = self.gate()
        g.feed(ANS(0))
        self.assertTrue(g.tick(0.0))
        g.feed(SAVED(1))
        self.assertTrue(g.tick(10.0))    # 保存完了を初めて観測
        self.assertTrue(g.tick(11.0))    # 1.0秒後はまだ保持
        self.assertFalse(g.tick(11.6))   # 1.5秒経過で解除
        g.feed(IDLE(1))
        self.assertFalse(g.tick(12.0))

    def test_post_save_hold_longer_than_debounce_wins(self):
        g = sm.SurveyGate(debounce_sec=0.5, post_save_hold_sec=3.0)
        g.feed(ANS(0))
        g.tick(0.0)
        g.feed(SAVED(1))
        g.tick(1.0)
        self.assertTrue(g.tick(3.9))     # デバウンス0.5秒は過ぎても保存後ホールド3秒の間は保つ
        self.assertFalse(g.tick(4.1))

    def test_saved_seq_increase_without_seeing_saved_state(self):
        # saved(3秒)の窓を取りこぼしても、saved_seqの増加で保存完了を検知する
        g = self.gate()
        g.feed(ANS(0))
        g.tick(0.0)
        g.feed(IDLE(1))
        self.assertTrue(g.tick(5.0))
        self.assertTrue(g.tick(6.0))
        self.assertFalse(g.tick(6.6))

    def test_first_sample_seq_is_baseline_not_a_save(self):
        g = self.gate()
        g.feed(IDLE(7))                  # 起動直後に見た saved_seq は基準にするだけ
        self.assertFalse(g.tick(0.0))
        g.feed(ANS(7))
        self.assertTrue(g.tick(1.0))
        g.feed(IDLE(7))                  # 保存なしでidleに戻った(end)ならデバウンスだけ
        self.assertTrue(g.tick(1.2))
        self.assertFalse(g.tick(2.8))

    def test_saved_state_persisting_does_not_extend_hold(self):
        g = self.gate()
        g.feed(ANS(0))
        g.tick(0.0)
        g.feed(SAVED(1))
        g.tick(1.0)
        g.feed(SAVED(1))                 # savedが続いて見えても、同じ保存で保持を延ばさない
        self.assertTrue(g.tick(2.0))
        self.assertFalse(g.tick(2.6))
        self.assertFalse(g.tick(3.5))

    def test_two_saves_in_a_row(self):
        g = self.gate()
        g.feed(ANS(0))
        g.tick(0.0)
        g.feed(SAVED(1))
        g.tick(1.0)
        g.feed(ANS(1))                   # 次の人がすぐ回答を始めた
        self.assertTrue(g.tick(1.5))
        g.feed(SAVED(2))
        self.assertTrue(g.tick(5.0))
        self.assertTrue(g.tick(6.0))
        self.assertFalse(g.tick(6.6))

    def test_abandoned_released_by_debounce(self):
        g = self.gate()
        g.feed(ANS(0))
        g.tick(0.0)
        g.feed(ABANDONED(0))             # ラズパイ側が放棄タイムアウト(120秒)でfalseにした
        self.assertTrue(g.tick(130.0))
        self.assertFalse(g.tick(131.6))  # 放棄は保存後ホールドなし、通常のデバウンスだけ

    def test_unreachable_while_answering_holds_longer(self):
        g = self.gate()
        g.feed(ANS(0))
        g.tick(0.0)
        g.feed(ERR)
        self.assertTrue(g.tick(25.0))    # 通常の保持(20秒)は超えても、回答中だったので保持
        self.assertTrue(g.tick(59.0))
        self.assertFalse(g.tick(61.0))   # 上限60秒で解除(ケーブル抜けで永久に止まらない)

    def test_unreachable_hold_is_measured_from_last_true(self):
        g = self.gate()
        g.feed(ANS(0))
        g.tick(0.0)
        g.feed(ERR)
        g.tick(40.0)
        g.feed(ANS(0))                   # 復帰してまた回答中を確認
        self.assertTrue(g.tick(41.0))
        g.feed(ERR)
        self.assertTrue(g.tick(100.0))   # 41+60=101まで
        self.assertFalse(g.tick(102.0))

    def test_unreachable_when_not_answering_uses_short_hold(self):
        g = self.gate()
        g.feed(dict(OK_T, session_state="idle", saved_seq=0))  # 異常な組合せでも answering でなければ短い保持
        g.tick(0.0)
        g.feed(ERR)
        self.assertTrue(g.tick(19.0))
        self.assertFalse(g.tick(21.0))

    def test_old_receiver_behaves_as_before(self):
        # session_state/saved_seq が無い古いラズパイ: 従来どおり(active+デバウンス+20秒保持)
        g = self.gate()
        g.feed(OK_T)
        self.assertTrue(g.tick(0.0))
        g.feed(OK_F)
        self.assertTrue(g.tick(0.2))
        self.assertFalse(g.tick(1.8))
        g.feed(OK_T)
        g.tick(10.0)
        g.feed(ERR)
        self.assertTrue(g.tick(29.0))
        self.assertFalse(g.tick(31.0))   # 古い相手は短い保持(20秒)のまま

    def test_customer_mode_overrides_answering(self):
        g = self.gate()
        g.feed(ANS(0))
        g.tick(0.0)
        g.feed(dict(ANS(0), mode="customer"))
        self.assertFalse(g.tick(0.2))

    def test_default_params(self):
        g = sm.SurveyGate()
        self.assertEqual((g.debounce_sec, g.unreachable_hold_sec, g.answering_unreachable_hold_sec,
                          g.post_save_hold_sec), (1.5, 20.0, 60.0, 1.5))


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class NodeTests(unittest.TestCase):
    """ノードのpublishロジック(rclpy無し、object.__new__で組み立て)。"""

    def make(self, url="http://x"):
        n = object.__new__(sm.SurveyMonitor)
        n._url = url
        n._gate = sm.SurveyGate(1.5, 20.0)
        n._lock = threading.Lock()
        n._latest = None
        n._active_pub, n._status_pub = _Pub(), _Pub()
        return n

    def test_publish_active_follows_latest_sample(self):
        n = self.make()
        sm_time = [100.0]
        orig = sm.time.monotonic
        sm.time.monotonic = lambda: sm_time[0]
        try:
            n._publish_active()
            self.assertIs(n._active_pub.msgs[-1].data, False)  # 起動直後はfalse
            n._latest = OK_T
            n._publish_active()
            self.assertIs(n._active_pub.msgs[-1].data, True)
            self.assertIsNone(n._latest)
            n._latest = ERR
            sm_time[0] += 5
            n._publish_active()
            self.assertIs(n._active_pub.msgs[-1].data, True)   # 一瞬の断はホールド
            sm_time[0] += 30
            n._publish_active()
            self.assertIs(n._active_pub.msgs[-1].data, False)  # 長い断は解除
        finally:
            sm.time.monotonic = orig

    def test_status_json_keys(self):
        n = self.make()
        n._gate.feed(dict(OK_T, condition="B"))
        n._gate.tick(0.0)
        n._publish_status()
        d = json.loads(n._status_pub.msgs[-1].data)
        for k in ("active", "mode", "count", "age", "reachable", "last_error"):
            self.assertIn(k, d)
        self.assertTrue(d["active"])
        self.assertTrue(d["reachable"])
        self.assertEqual(d["mode"], "survey")

    def test_status_includes_session_fields(self):
        n = self.make()
        n._gate.feed(dict(ANS(3)))
        n._gate.tick(0.0)
        n._publish_status()
        d = json.loads(n._status_pub.msgs[-1].data)
        self.assertEqual((d["session_state"], d["saved_seq"]), ("answering", 3))

    def test_status_old_receiver_has_null_session_fields(self):
        n = self.make()
        n._gate.feed(OK_T)
        n._publish_status()
        d = json.loads(n._status_pub.msgs[-1].data)
        self.assertIsNone(d["session_state"])
        self.assertIsNone(d["saved_seq"])

    def test_status_when_unreachable(self):
        n = self.make()
        n._gate.feed(ERR)
        n._publish_status()
        d = json.loads(n._status_pub.msgs[-1].data)
        self.assertFalse(d["reachable"])
        self.assertEqual(d["last_error"], "URLError")
        self.assertFalse(d["active"])

    def test_empty_url_disabled_flag(self):
        n = self.make(url="")
        n._publish_status()
        self.assertFalse(json.loads(n._status_pub.msgs[-1].data)["enabled"])


if __name__ == "__main__":
    unittest.main(verbosity=1)
