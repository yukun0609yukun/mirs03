"""ルール編集ページ(/behavior_rules)のサーバー側API・保存ロジックの結合テスト。

ROS 2が無いPCでも動くよう、rclpy系モジュールをスタブしてdashboard_commonを読み込み、
実際にHTTPサーバーを立てて /api/behavior_rules* を叩く(test_boundary_api.pyと同じ方式)。
確認すること:
- 検証(behavior_rules自身の関数): NaN/未知の方針/不正な時刻/重複id等は400で日本語のエラー一覧、
  ファイルは作られも変更もされない。
- 保存は原子的(一時ファイル→置換)、直前のファイルが .bak に残る、書き込み失敗でも元のファイルは無傷。
- 競合検知(編集開始後に別の書き換えがあれば409)。
- 保存後に再読み込みトピック(Empty)がpublishされる。
- guestは閲覧できるが保存/再読み込みは403。未ログインは401。
- 稼働状況(LazySubscription)・次の24時間の予定プレビュー。
ロボットには何も送らない(publisherは全てスタブ)。

実行: python ros2_ws/src/nokosahen_core/test/test_behavior_rules_api.py
"""

import datetime
import json
import os
import sys
import tempfile
import threading
import types
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest import mock

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))


def _install_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

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

from nokosahen_core import behavior_rules as br  # noqa: E402
from nokosahen_core import behavior_rules_store as store  # noqa: E402
from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core.behavior_rules_page import build_behavior_rules_html  # noqa: E402


class _EmptyMsg:
    pass


if dc.Empty is None:  # スタブ環境(std_msgsにEmptyが無い)でも再読み込み通知を検証できるようにする
    dc.Empty = _EmptyMsg


class FakeLogger:
    def warn(self, *a, **k):
        pass

    info = error = debug = warn


class FakePub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class FakeNode:
    def __init__(self):
        self.pubs = {}
        self.subs = {}

    def create_publisher(self, typ, topic, depth):
        self.pubs[topic] = FakePub()
        return self.pubs[topic]

    def create_subscription(self, typ, topic, cb, depth):
        sub = object()
        self.subs[topic] = (sub, cb)
        return sub

    def destroy_subscription(self, sub):
        for k, (s, _cb) in list(self.subs.items()):
            if s is sub:
                del self.subs[k]

    def create_timer(self, period, cb):
        return None

    def get_logger(self):
        return FakeLogger()


class FakeTeleop:
    soft_estop_engaged = False


def rule(rid="r1", **kw):
    d = {
        "id": rid, "enabled": True, "name": "テスト " + rid, "priority": 10,
        "when": {"time_ranges": ["11:30-13:00"]},
        "then": {"policy": ["stay"], "params": {}},
    }
    d.update(kw)
    return d


def doc(*rules, hold=10):
    return {"version": 1, "description": "", "min_hold_seconds": hold, "rules": list(rules)}


class ApiBase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.rules_path = self.dir / "rules.json"
        self.node = FakeNode()
        self.ctrl = dc.BehaviorRulesController(self.node, str(self.rules_path))
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>",
            lambda: {},
            FakeTeleop(),
            behavior_rules=self.ctrl,
            behavior_rules_html_getter=build_behavior_rules_html,
            auth_username="TL",
            auth_password="pw",
            enforce_single_controller=False,
        )
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        self.port = self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.tl = self.login("TL", "pw")
        self.guest = self.login("guest", "")

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.tmp.cleanup()

    def login(self, user, password):
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.port}/api/login",
            data=json.dumps({"username": user, "password": password}).encode(),
            method="POST", headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=10) as r:
            cookie = r.headers.get("Set-Cookie", "")
        return cookie.split(";")[0]

    def call(self, method, path, body=None, raw=None, cookie=None, text=False):
        cookie = self.tl if cookie is None else cookie
        data = raw if raw is not None else (None if body is None else json.dumps(body).encode())
        headers = {"Content-Type": "application/json"}
        if cookie:
            headers["Cookie"] = cookie
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.port}{path}", data=data, method=method, headers=headers
        )
        try:
            with urllib.request.urlopen(req, timeout=10) as r:
                payload = r.read().decode("utf-8")
                return r.status, (payload if text else json.loads(payload))
        except urllib.error.HTTPError as e:
            try:
                payload = e.read().decode("utf-8")
                return e.code, (payload if text else json.loads(payload))
            finally:
                e.close()

    def save(self, document, expected_sig=None, **kw):
        body = {"document": document}
        if expected_sig is not None:
            body["expected_sig"] = expected_sig
        return self.call("POST", "/api/behavior_rules/save", body, **kw)

    def leftovers(self):
        return sorted(p.name for p in self.dir.iterdir() if p.name.endswith(".tmp"))


class ReadAndAuthTests(ApiBase):
    def test_page_has_face_policy_labels(self):
        html = build_behavior_rules_html().decode("utf-8")
        self.assertIn("face_enabled:", html)
        self.assertIn("face_disabled:", html)

    def test_page_is_served_and_has_estop_dock(self):
        st, html = self.call("GET", "/behavior_rules", text=True)
        self.assertEqual(st, 200)
        self.assertIn("時間帯・状況ルール", html)
        self.assertIn('id="estopDock"', html)

    def test_unauthenticated_api_is_401(self):
        st, _ = self.call("GET", "/api/behavior_rules", cookie="")
        self.assertEqual(st, 401)
        st, _ = self.save(doc(rule()), cookie="")
        self.assertEqual(st, 401)

    def test_get_missing_file(self):
        st, d = self.call("GET", "/api/behavior_rules")
        self.assertEqual(st, 200)
        self.assertFalse(d["exists"])
        self.assertIsNone(d["document"])
        self.assertTrue(d["valid"])
        self.assertEqual(d["sig"], "missing")
        self.assertIn("go_to_entrance", d["meta"]["policies"])
        self.assertEqual(d["path"], str(self.rules_path))

    def test_get_broken_existing_file_reports_errors_without_raising(self):
        self.rules_path.write_text("{not json", encoding="utf-8")
        st, d = self.call("GET", "/api/behavior_rules")
        self.assertEqual(st, 200)
        self.assertFalse(d["valid"])
        self.assertTrue(d["errors"])
        self.assertIsNone(d["document"])

    def test_guest_can_read_but_not_write(self):
        st, d = self.call("GET", "/api/behavior_rules", cookie=self.guest)
        self.assertEqual(st, 200)
        st, _ = self.call("GET", "/behavior_rules", cookie=self.guest, text=True)
        self.assertEqual(st, 200)
        st, _ = self.call("GET", "/api/behavior_rules/preview", cookie=self.guest)
        self.assertIn(st, (200, 409))
        st, d = self.save(doc(rule()), cookie=self.guest)
        self.assertEqual(st, 403)
        self.assertFalse(self.rules_path.exists())
        st, _ = self.call("POST", "/api/behavior_rules/reload", {}, cookie=self.guest)
        self.assertEqual(st, 403)
        self.assertEqual(self.node.pubs["/nokosahen/behavior_rules/reload"].sent, [])

    def test_example_endpoint_returns_all_disabled_valid_document(self):
        st, d = self.call("GET", "/api/behavior_rules/example")
        self.assertEqual(st, 200, d)
        rules = d["document"]["rules"]
        self.assertTrue(rules)
        self.assertTrue(all(r["enabled"] is False for r in rules))
        errors, _rs, _t = store.validate_document(d["document"])
        self.assertEqual(errors, [])


class SaveTests(ApiBase):
    def test_valid_save_writes_file_and_publishes_reload(self):
        st, d = self.save(doc(rule("a"), rule("b", enabled=False)))
        self.assertEqual(st, 200, d)
        self.assertTrue(d["ok"])
        self.assertTrue(d["reload_published"])
        self.assertEqual(d["rule_count"], 2)
        self.assertEqual(d["enabled_rule_count"], 1)
        self.assertEqual(len(self.node.pubs["/nokosahen/behavior_rules/reload"].sent), 1)
        # behavior_directorが実際に読むのと同じ関数で読めること
        rs = br.parse_rules_text(self.rules_path.read_text(encoding="utf-8"))
        self.assertEqual([r.id for r in rs.rules], ["a", "b"])
        self.assertEqual(self.leftovers(), [])
        # 再読み込み(楽観更新ではなくサーバーの内容)
        st, g = self.call("GET", "/api/behavior_rules")
        self.assertEqual(g["sig"], d["sig"])
        self.assertEqual([r["id"] for r in g["document"]["rules"]], ["a", "b"])
        self.assertTrue(g["valid"])

    def test_backup_of_previous_file_is_kept(self):
        self.assertEqual(self.save(doc(rule("old")))[0], 200)
        first = self.rules_path.read_bytes()
        st, d = self.save(doc(rule("new")))
        self.assertEqual(st, 200)
        self.assertTrue(d["backed_up"])
        self.assertEqual(Path(str(self.rules_path) + ".bak").read_bytes(), first)
        self.assertIn(b'"new"', self.rules_path.read_bytes())
        self.assertEqual(self.leftovers(), [])

    def test_first_save_has_no_backup(self):
        st, d = self.save(doc(rule()))
        self.assertEqual(st, 200)
        self.assertFalse(d["backed_up"])
        self.assertFalse(Path(str(self.rules_path) + ".bak").exists())

    def test_empty_rules_allowed(self):
        st, d = self.save(doc())
        self.assertEqual(st, 200, d)
        self.assertEqual(d["rule_count"], 0)

    def test_invalid_payloads_rejected_and_nothing_written(self):
        bad = {
            "unknown policy": doc(rule(then={"policy": ["fly_away"], "params": {}})),
            "bad time": doc(rule(when={"time_ranges": ["25:00-26:00"]})),
            "bad time format": doc(rule(when={"time_ranges": ["11:30"]})),
            "same start end": doc(rule(when={"time_ranges": ["10:00-10:00"]})),
            "unknown key": doc(rule(bogus=1)),
            "unknown when key": doc(rule(when={"moon": True})),
            "conflicting policies": doc(rule(then={"policy": ["approach_enabled", "approach_disabled"], "params": {}})),
            "bad id": doc(rule("bad id!")),
            "enabled not bool": doc(rule(enabled="yes")),
            "priority out of range": doc(rule(priority=99999)),
            "speed scale out of range": doc(rule(then={"policy": ["speed_scale"], "params": {"scale": 5}})),
            "bad weekday": doc(rule(when={"weekdays": ["xyz"]})),
            "bad date": doc(rule(when={"dates": ["2026-02-30"]})),
            "bad hold": doc(hold=100000),
            "rules not list": {"version": 1, "rules": "x"},
            "version 2": {"version": 2, "rules": []},
        }
        for label, document in bad.items():
            st, d = self.save(document)
            self.assertEqual(st, 400, (label, d))
            self.assertFalse(d["ok"], label)
            self.assertTrue(d["errors"], label)
            self.assertFalse(self.rules_path.exists(), label)
        self.assertEqual(self.leftovers(), [])
        self.assertEqual(self.node.pubs["/nokosahen/behavior_rules/reload"].sent, [])

    def test_unknown_policy_message_is_japanese_and_names_the_rule(self):
        st, d = self.save(doc(rule("zz", then={"policy": ["fly_away"], "params": {}})))
        self.assertEqual(st, 400)
        self.assertTrue(any("rules[0](zz)" in e and "fly_away" in e and "方針" in e for e in d["errors"]), d["errors"])

    def test_nan_and_infinity_rejected(self):
        for token in ("NaN", "Infinity", "-Infinity"):
            raw = ('{"document": {"version": 1, "rules": [{"id": "a", "enabled": true, "priority": %s, '
                   '"when": {}, "then": {"policy": ["stay"]}}]}}' % token).encode()
            st, d = self.call("POST", "/api/behavior_rules/save", raw=raw)
            self.assertEqual(st, 400, token)
            self.assertFalse(d["ok"])
            self.assertTrue(any("NaN" in e or "Infinity" in e for e in d["errors"]), d["errors"])
        # 値の中のNaN(remaining_ratio_below)も同様
        raw = (b'{"document": {"rules": [{"id": "a", "when": {"remaining_ratio_below": NaN},'
               b' "then": {"policy": ["stay"]}}]}}')
        st, _ = self.call("POST", "/api/behavior_rules/save", raw=raw)
        self.assertEqual(st, 400)
        self.assertFalse(self.rules_path.exists())

    def test_nan_in_direct_python_call_rejected_too(self):
        errors, rs, text = store.validate_document(doc(rule(priority=float("nan"))))
        self.assertTrue(errors)
        self.assertIsNone(rs)
        self.assertIsNone(text)
        errors, _rs, _t = store.validate_document(doc(rule(when={"idle_seconds_above": float("inf")})))
        self.assertTrue(errors)

    def test_malformed_bodies_rejected(self):
        for raw in (b"", b"not json", b"[1,2,3]", b'{"nodoc": 1}', b'{"document": []}', b'{"document": 5}'):
            st, d = self.call("POST", "/api/behavior_rules/save", raw=raw)
            self.assertEqual(st, 400, raw)
            self.assertFalse(d["ok"])
            self.assertTrue(d["errors"])
        self.assertFalse(self.rules_path.exists())

    def test_multiple_errors_reported_together_and_duplicates(self):
        st, d = self.save(doc(
            rule("a", then={"policy": ["nope"], "params": {}}),
            rule("b", when={"time_ranges": ["99:99-10:00"]}),
            rule("c"), rule("c"),
        ))
        self.assertEqual(st, 400)
        self.assertGreaterEqual(len(d["errors"]), 3, d["errors"])
        self.assertTrue(any("重複" in e for e in d["errors"]), d["errors"])

    def test_invalid_save_leaves_existing_file_and_backup_untouched(self):
        self.assertEqual(self.save(doc(rule("good1")))[0], 200)
        self.assertEqual(self.save(doc(rule("good2")))[0], 200)
        main_before = self.rules_path.read_bytes()
        bak_path = Path(str(self.rules_path) + ".bak")
        bak_before = bak_path.read_bytes()
        st, _ = self.save(doc(rule("x", then={"policy": ["nope"], "params": {}})))
        self.assertEqual(st, 400)
        self.assertEqual(self.rules_path.read_bytes(), main_before)
        self.assertEqual(bak_path.read_bytes(), bak_before)
        self.assertEqual(self.leftovers(), [])

    def test_atomic_write_failure_keeps_original(self):
        self.assertEqual(self.save(doc(rule("orig")))[0], 200)
        before = self.rules_path.read_bytes()
        real_replace = os.replace

        def failing_replace(src, dst):
            if str(dst) == str(self.rules_path):
                raise OSError("disk full (simulated)")
            return real_replace(src, dst)

        with mock.patch.object(store.os, "replace", side_effect=failing_replace):
            st, d = self.save(doc(rule("new")))
        self.assertEqual(st, 500, d)
        self.assertFalse(d["ok"])
        self.assertEqual(self.rules_path.read_bytes(), before)  # 元のファイルは無傷
        self.assertEqual(self.leftovers(), [])                   # 一時ファイルも残さない
        br.parse_rules_text(self.rules_path.read_text(encoding="utf-8"))  # 読めるまま
        self.assertEqual(len(self.node.pubs["/nokosahen/behavior_rules/reload"].sent), 1)  # 失敗時は通知しない(最初の1回のみ)

    def test_atomic_write_failure_on_first_save_creates_nothing(self):
        with mock.patch.object(store.os, "fsync", side_effect=OSError("io error (simulated)")):
            st, d = self.save(doc(rule()))
        self.assertEqual(st, 500, d)
        self.assertFalse(self.rules_path.exists())
        self.assertEqual(self.leftovers(), [])

    def test_conflict_detected_when_file_changed_after_load(self):
        self.assertEqual(self.save(doc(rule("a")))[0], 200)
        _, g = self.call("GET", "/api/behavior_rules")
        sig = g["sig"]
        # 他の誰かが書き換える
        self.rules_path.write_text(json.dumps(doc(rule("someone_else")), ensure_ascii=False), encoding="utf-8")
        os.utime(self.rules_path, (1, 2))
        before = self.rules_path.read_bytes()
        st, d = self.save(doc(rule("mine")), expected_sig=sig)
        self.assertEqual(st, 409, d)
        self.assertEqual(self.rules_path.read_bytes(), before)
        # 最新のsigで保存すれば通る
        _, g2 = self.call("GET", "/api/behavior_rules")
        st, _ = self.save(doc(rule("mine")), expected_sig=g2["sig"])
        self.assertEqual(st, 200)

    def test_expected_sig_missing_for_new_file(self):
        st, d = self.save(doc(rule("a")), expected_sig="missing")
        self.assertEqual(st, 200, d)
        st, d = self.save(doc(rule("b")), expected_sig="missing")  # 既にファイルがある
        self.assertEqual(st, 409)

    def test_large_document_over_64kb_is_accepted(self):
        rules = [rule(f"r{i:03d}", description="あ" * 400) for i in range(100)]
        raw = json.dumps({"document": doc(*rules)}, ensure_ascii=False).encode("utf-8")
        self.assertGreater(len(raw), 64 * 1024)
        st, d = self.call("POST", "/api/behavior_rules/save", raw=raw)
        self.assertEqual(st, 200, d)
        self.assertEqual(d["rule_count"], 100)

    def test_too_many_rules_rejected(self):
        st, d = self.save(doc(*[rule(f"r{i}") for i in range(101)]))
        self.assertEqual(st, 400)

    def test_reload_endpoint_publishes_empty(self):
        st, d = self.call("POST", "/api/behavior_rules/reload", {})
        self.assertEqual(st, 200)
        self.assertTrue(d["ok"])
        self.assertEqual(len(self.node.pubs["/nokosahen/behavior_rules/reload"].sent), 1)


class LiveStateTests(ApiBase):
    TOPIC = "/nokosahen/behavior_rules/state"

    def feed(self, payload):
        _sub, cb = self.node.subs[self.TOPIC]
        msg = types.SimpleNamespace(data=payload if isinstance(payload, str) else json.dumps(payload))
        cb(msg)

    def test_idle_means_no_subscription_until_touched(self):
        self.assertNotIn(self.TOPIC, self.node.subs)
        st, d = self.call("GET", "/api/behavior_rules/state")
        self.assertEqual(st, 200)
        self.assertIn(self.TOPIC, self.node.subs)  # ページを開いた時だけ購読開始
        self.assertEqual(d["status"], "waiting")

    def test_ok_state_and_stale_then_no_response(self):
        self.call("GET", "/api/behavior_rules/state")
        self.feed({"active_rule_id": "a", "active_rule_name": "A", "policy": "stay",
                   "enabled_rule_count": 1, "rule_count": 2, "rules_file": str(self.rules_path)})
        st, d = self.call("GET", "/api/behavior_rules/state")
        self.assertEqual(d["status"], "ok")
        self.assertEqual(d["state"]["active_rule_id"], "a")
        self.assertFalse(d["path_mismatch"])
        # 配信が途切れたら無応答
        with self.ctrl._lock:
            self.ctrl._rx -= 60.0
        st, d = self.call("GET", "/api/behavior_rules/state")
        self.assertEqual(d["status"], "no_response")
        self.assertIsNone(d["state"])
        self.assertIn("無応答", d["message"])

    def test_never_received_becomes_no_response_after_warmup(self):
        self.call("GET", "/api/behavior_rules/state")
        with self.ctrl._lock:
            self.ctrl._sub_started -= 60.0
        st, d = self.call("GET", "/api/behavior_rules/state")
        self.assertEqual(d["status"], "no_response")

    def test_malformed_state_messages_ignored(self):
        self.call("GET", "/api/behavior_rules/state")
        self.feed({"policy": "normal", "rules_file": str(self.rules_path)})
        self.feed("not json")
        self.feed("[1,2]")
        st, d = self.call("GET", "/api/behavior_rules/state")
        self.assertEqual(d["status"], "ok")
        self.assertEqual(d["state"]["policy"], "normal")

    def test_path_mismatch_flagged(self):
        self.call("GET", "/api/behavior_rules/state")
        self.feed({"policy": "normal", "rules_file": "/somewhere/else.json"})
        st, d = self.call("GET", "/api/behavior_rules/state")
        self.assertTrue(d["path_mismatch"])

    def test_unsubscribe_clears_cached_state(self):
        self.call("GET", "/api/behavior_rules/state")
        self.feed({"policy": "stay", "rules_file": str(self.rules_path)})
        self.ctrl._clear_state()
        with self.ctrl._lock:
            self.assertIsNone(self.ctrl._state)


class PreviewTests(ApiBase):
    def test_preview_from_saved_rules_time_only_and_situational_flagged(self):
        sit = rule("crowdy", priority=50, when={"time_ranges": ["09:00-10:00"], "crowd_level": ["crowded"]},
                   then={"policy": ["approach_disabled"], "params": {}})
        timed = rule("lunch", priority=10, when={"time_ranges": ["11:30-13:00"]},
                     then={"policy": ["quiet", "stay"], "params": {}})
        self.assertEqual(self.save(doc(timed, sit, rule("off", enabled=False)))[0], 200)
        fixed_now = datetime.datetime(2026, 10, 6, 8, 0)
        self.ctrl._clock = lambda: fixed_now
        st, d = self.call("GET", "/api/behavior_rules/preview")
        self.assertEqual(st, 200, d)
        segs = d["segments"]
        self.assertEqual(segs[0]["start"], "2026-10-06T08:00")
        self.assertEqual(segs[-1]["end"], "2026-10-07T08:00")
        for a, b in zip(segs, segs[1:]):
            self.assertEqual(a["end"], b["start"])  # 隙間・重なり無し
        lunch = [s for s in segs if s["definite"] and s["definite"]["id"] == "lunch"]
        self.assertEqual(len(lunch), 1)
        self.assertEqual((lunch[0]["start"], lunch[0]["end"]), ("2026-10-06T11:30", "2026-10-06T13:00"))
        self.assertEqual(lunch[0]["definite"]["policy"], "quiet+stay")
        crowdy = [s for s in segs if any(p["id"] == "crowdy" for p in s["possible"])]
        self.assertEqual(len(crowdy), 1)
        self.assertEqual((crowdy[0]["start"], crowdy[0]["end"]), ("2026-10-06T09:00", "2026-10-06T10:00"))
        self.assertIsNone(crowdy[0]["definite"])  # 状況条件ルールは「確定」にしない
        self.assertIn("予測できない", d["note"])
        self.assertEqual(d["next_time_change_at"], "2026-10-06T09:00")
        self.assertEqual(d["enabled_rule_count"], 2)

    def test_preview_overnight_and_weekdays(self):
        night = rule("night", when={"time_ranges": ["22:00-06:00"], "weekdays": ["tue"]},
                     then={"policy": ["quiet"], "params": {}})
        self.assertEqual(self.save(doc(night))[0], 200)
        self.ctrl._clock = lambda: datetime.datetime(2026, 10, 6, 20, 0)  # 火曜
        st, d = self.call("GET", "/api/behavior_rules/preview")
        self.assertEqual(st, 200)
        active = [s for s in d["segments"] if s["definite"]]
        # 火曜22:00から日付を跨いで水曜06:00までは「曜日=現在の暦日」で判定(火曜の22:00-24:00のみ該当)
        self.assertEqual(active[0]["start"], "2026-10-06T22:00")
        self.assertEqual(active[0]["end"], "2026-10-07T00:00")

    def test_preview_without_rules(self):
        self.ctrl._clock = lambda: datetime.datetime(2026, 10, 6, 8, 0)
        st, d = self.call("GET", "/api/behavior_rules/preview")
        self.assertEqual(st, 200)
        self.assertEqual(d["enabled_rule_count"], 0)
        self.assertEqual(len(d["segments"]), 1)
        self.assertIsNone(d["segments"][0]["definite"])

    def test_preview_with_broken_saved_file_is_409(self):
        self.rules_path.write_text("{broken", encoding="utf-8")
        st, d = self.call("GET", "/api/behavior_rules/preview")
        self.assertEqual(st, 409)
        self.assertFalse(d["ok"])


if __name__ == "__main__":
    unittest.main()
