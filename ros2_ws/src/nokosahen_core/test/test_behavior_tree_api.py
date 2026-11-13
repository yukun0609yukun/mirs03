"""ビヘイビアツリー編集ページ(/behavior_tree)のサーバー側API・保存ロジックの結合テスト(2026-10-09)。

ROS 2が無いPCでも動くよう、rclpy系モジュールをスタブしてdashboard_commonを読み込み、
実際にHTTPサーバーを立てて /api/behavior_tree* を叩く(test_behavior_rules_api.pyと同じ方式)。
確認すること:
- 検証(behavior_tree自身の関数): 未知のノード/不正な値/NaN/深さ・個数の上限は400で日本語のエラー一覧、
  ファイルは作られも変更もされない。
- 保存は原子的(一時ファイル→置換)、直前のファイルが .bak に残る、競合は409。
- guestは閲覧できるが保存/検証/既定に戻すは403。未ログインは401。
- 既定に戻す=ファイルを退避して無くす(従来のルール表のみの動作)。
- ライブ状態(LazySubscription)・保存済みとdirectorの木の一致判定。
- ページHTML(安全層・緊急停止ドック・JS構文)。
ロボットには何も送らない(publisherは全てスタブ)。

実行: python ros2_ws/src/nokosahen_core/test/test_behavior_tree_api.py
"""

import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import types
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import test_behavior_rules_api as tra  # noqa: E402  (スタブ導入・FakeNode等を再利用)
from nokosahen_core import behavior_tree as bt  # noqa: E402
from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import map_pins as mp  # noqa: E402
from nokosahen_core.behavior_tree_page import build_behavior_tree_html  # noqa: E402
from nokosahen_core.behavior_tree_store import BehaviorTreeController  # noqa: E402


def N(typ, children=None, **params):
    d = {"type": typ}
    if params:
        d["params"] = params
    if children is not None:
        d["children"] = children
    return d


def doc(root=None, hold=10, **kw):
    d = {"version": 1, "description": "", "min_hold_seconds": hold, "root": root or N("selector", [N("hold_stop")])}
    d.update(kw)
    return d


class FakePinStore:
    def __init__(self, loaded="hall", pins=()):
        self._loaded, self._pins = loaded, list(pins)

    def loaded_map(self):
        return self._loaded

    def list(self):
        return self._pins


class FakePinController:
    def __init__(self, **kw):
        self.store = FakePinStore(**kw)


class ApiBase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.tree_path = self.dir / "tree.json"
        self.node = tra.FakeNode()
        self.pins = FakePinController(
            pins=[{"id": "pinA", "name": "ピンA", "x": 1.0, "y": 2.0, "yaw_deg": 0.0, "color": "#e11d48"}]
        )
        self.ctrl = BehaviorTreeController(
            self.node, str(self.tree_path), dc.String, dc.LazySubscription,
            pin_controller=self.pins, pins_payload_fn=mp.pins_payload,
        )
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, tra.FakeTeleop(),
            behavior_tree=self.ctrl, behavior_tree_html_getter=build_behavior_tree_html,
            auth_username="TL", auth_password="pw", enforce_single_controller=False,
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

    login = tra.ApiBase.login
    call = tra.ApiBase.call

    def save(self, document, expected_sig=None, **kw):
        body = {"document": document}
        if expected_sig is not None:
            body["expected_sig"] = expected_sig
        return self.call("POST", "/api/behavior_tree", body, **kw)

    def leftovers(self):
        return sorted(p.name for p in self.dir.iterdir() if p.name.endswith(".tmp"))


class PageTests(ApiBase):
    def test_page_served_with_safety_and_estop(self):
        st, html = self.call("GET", "/behavior_tree", text=True)
        self.assertEqual(st, 200)
        self.assertIn("ビヘイビアツリー", html)
        self.assertIn('id="estopDock"', html)
        self.assertIn("安全層", html)
        self.assertIn("/api/behavior_tree", html)

    def test_face_and_after_facing_in_page_meta(self):
        """向く(face)と after_facing は、ページが木のメタ情報(behavior_tree.py)から描く。
        behavior_tree.py に face が入るまではスキップ。"""
        if "face" not in bt.NODE_SPECS:
            self.skipTest("behavior_tree.py に face がまだ無い")
        self.assertEqual(bt.NODE_SPECS["face"]["kind"], "action")
        keys = [p["key"] for p in bt.NODE_SPECS["announce"]["params"]]
        self.assertIn("after_facing", keys)
        spec = next(p for p in bt.NODE_SPECS["announce"]["params"] if p["key"] == "after_facing")
        self.assertEqual(spec["type"], "bool")
        self.assertIs(spec.get("default"), False)
        # ページのJSは bool パラメータを汎用に描く(select、data-bool)
        html = build_behavior_tree_html().decode("utf-8")
        self.assertIn('data-bool="1"', html)

    def test_page_js_is_valid(self):
        node = shutil.which("node")
        if not node:
            self.skipTest("nodeが無い")
        html = build_behavior_tree_html().decode("utf-8")
        js = re.search(r"<script>(.*)</script>", html, re.S).group(1)
        p = self.dir / "page.js"
        p.write_text(js, encoding="utf-8")
        r = subprocess.run([node, "--check", str(p)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)

    def test_unauthenticated_page_and_api(self):
        st, _ = self.call("GET", "/api/behavior_tree", cookie="")
        self.assertEqual(st, 401)
        st, _ = self.save(doc(), cookie="")
        self.assertEqual(st, 401)
        st, _ = self.call("POST", "/api/behavior_tree/validate", {"document": doc()}, cookie="")
        self.assertEqual(st, 401)
        st, _ = self.call("POST", "/api/behavior_tree/reset", {}, cookie="")
        self.assertEqual(st, 401)


class FileTests(ApiBase):
    def test_get_missing_returns_default_tree(self):
        st, d = self.call("GET", "/api/behavior_tree")
        self.assertEqual(st, 200)
        self.assertFalse(d["exists"])
        self.assertTrue(d["is_default"])
        self.assertEqual(d["document"]["root"]["type"], "rules_table")
        self.assertEqual(d["sig"], "missing")
        self.assertIn("types", d["meta"])
        self.assertEqual(len(d["examples"]), 3)
        self.assertEqual({t["type"] for t in d["meta"]["types"]}, set(bt.NODE_CLASSES))
        self.assertEqual(d["meta"]["safety_layers"][0]["key"], "estop")
        self.assertIn("live", d)

    def test_save_valid_then_get(self):
        st, d = self.save(doc())
        self.assertEqual(st, 200, d)
        self.assertTrue(d["ok"])
        self.assertFalse(d["backed_up"])
        self.assertTrue(self.tree_path.exists())
        self.assertEqual(self.leftovers(), [])
        st, g = self.call("GET", "/api/behavior_tree")
        self.assertTrue(g["exists"])
        self.assertFalse(g["is_default"])
        self.assertTrue(g["valid"])
        self.assertEqual(g["document"]["root"]["type"], "selector")
        # 保存内容はdirectorのパーサーでそのまま読める
        self.assertEqual(bt.parse_tree_text(self.tree_path.read_text(encoding="utf-8"))["node_count"], 2)

    def test_second_save_keeps_backup(self):
        self.save(doc(N("hold_stop")))
        first = self.tree_path.read_text(encoding="utf-8")
        st, d = self.save(doc(N("quiet")))
        self.assertEqual(st, 200)
        self.assertTrue(d["backed_up"])
        self.assertEqual(Path(str(self.tree_path) + ".bak").read_text(encoding="utf-8"), first)

    def test_invalid_rejected_and_nothing_written(self):
        bad = [
            (doc(N("nope")), "未知のノード種別"),
            (doc(N("time_between", range="99:00-10:00")), "範囲外"),
            (doc(N("inverter", [])), "ちょうど1つ"),
            (doc(N("idle_for", seconds=-1)), "seconds"),
            ({"root": N("quiet"), "evil": 1}, "未知のキー"),
            (doc(N("selector", [N("quiet") for _ in range(bt.MAX_NODES)])), "多すぎ"),
        ]
        for document, needle in bad:
            st, d = self.save(document)
            self.assertEqual(st, 400, document)
            self.assertFalse(d["ok"])
            self.assertTrue(any(needle in e for e in d["errors"]), (needle, d["errors"]))
        self.assertFalse(self.tree_path.exists())
        self.assertEqual(self.leftovers(), [])
        # 既存ファイルがあれば無傷
        self.save(doc(N("quiet")))
        before = self.tree_path.read_bytes()
        st, _ = self.save(doc(N("nope")))
        self.assertEqual(st, 400)
        self.assertEqual(self.tree_path.read_bytes(), before)

    def test_nan_and_deep_json_rejected(self):
        for token in ("NaN", "Infinity", "-Infinity"):
            raw = ('{"document": {"root": {"type": "idle_for", "params": {"seconds": %s}}}}' % token).encode()
            st, d = self.call("POST", "/api/behavior_tree", raw=raw)
            self.assertEqual(st, 400, token)
            self.assertFalse(self.tree_path.exists())
        deep = ("[" * 5000 + "]" * 5000).encode()
        st, d = self.call("POST", "/api/behavior_tree", raw=b'{"document": ' + deep + b"}")
        self.assertEqual(st, 400)
        self.assertFalse(self.tree_path.exists())

    def test_oversize_body_rejected(self):
        raw = json.dumps({"document": doc(), "pad": "x" * (600 * 1024)}).encode()
        st, d = self.call("POST", "/api/behavior_tree", raw=raw)
        self.assertEqual(st, 400)
        self.assertFalse(self.tree_path.exists())

    def test_non_object_body_and_missing_document(self):
        st, _ = self.call("POST", "/api/behavior_tree", raw=b"[1,2]")
        self.assertEqual(st, 400)
        st, _ = self.call("POST", "/api/behavior_tree", {"nodoc": 1})
        self.assertEqual(st, 400)
        st, _ = self.call("POST", "/api/behavior_tree", {"document": doc(), "expected_sig": 5})
        self.assertEqual(st, 400)

    def test_conflict_detected(self):
        st, g = self.call("GET", "/api/behavior_tree")
        sig0 = g["sig"]                      # "missing"
        self.save(doc(N("quiet")))           # 別の人が先に保存
        st, d = self.save(doc(N("hold_stop")), expected_sig=sig0)
        self.assertEqual(st, 409)
        self.assertEqual(d["kind"], "conflict")
        self.assertIn("quiet", self.tree_path.read_text(encoding="utf-8"))
        st, g = self.call("GET", "/api/behavior_tree")
        st, d = self.save(doc(N("hold_stop")), expected_sig=g["sig"])
        self.assertEqual(st, 200)

    def test_write_failure_leaves_original(self):
        self.save(doc(N("quiet")))
        before = self.tree_path.read_bytes()
        real = os.replace

        def boom(src, dst):
            raise OSError("disk full")

        os.replace = boom
        try:
            st, d = self.save(doc(N("hold_stop")))
        finally:
            os.replace = real
        self.assertEqual(st, 500)
        self.assertEqual(d["kind"], "io")
        self.assertEqual(self.tree_path.read_bytes(), before)
        self.assertEqual(self.leftovers(), [])

    def test_corrupt_existing_file_reported(self):
        self.tree_path.write_text("{oops", encoding="utf-8")
        st, g = self.call("GET", "/api/behavior_tree")
        self.assertEqual(st, 200)
        self.assertTrue(g["exists"])
        self.assertFalse(g["valid"])
        self.assertTrue(g["errors"])
        st, d = self.save(doc())          # 壊れていても上書き保存はできる(.bakに残る)
        self.assertEqual(st, 200)
        self.assertTrue(d["backed_up"])

    def test_validate_endpoint_does_not_write(self):
        st, d = self.call("POST", "/api/behavior_tree/validate", {"document": doc()})
        self.assertEqual(st, 200)
        self.assertTrue(d["ok"])
        self.assertEqual(d["node_count"], 2)
        self.assertTrue(d["outline"])
        st, d = self.call("POST", "/api/behavior_tree/validate", {"document": doc(N("nope"))})
        self.assertEqual(st, 400)
        self.assertFalse(d["ok"])
        st, d = self.call(
            "POST", "/api/behavior_tree/validate", {"document": doc(N("selector", [N("quiet"), N("hold_stop")]))}
        )
        self.assertEqual(st, 200)
        self.assertTrue(d["warnings"])       # 到達しない子の警告
        self.assertFalse(self.tree_path.exists())

    def test_reset_removes_file_keeps_backup(self):
        st, d = self.call("POST", "/api/behavior_tree/reset", {})
        self.assertEqual(st, 200)
        self.assertFalse(d["removed"])       # もともと無い
        self.save(doc(N("quiet")))
        saved = self.tree_path.read_text(encoding="utf-8")
        st, g = self.call("GET", "/api/behavior_tree")
        st, d = self.call("POST", "/api/behavior_tree/reset", {"expected_sig": "stale"})
        self.assertEqual(st, 409)
        self.assertTrue(self.tree_path.exists())
        st, d = self.call("POST", "/api/behavior_tree/reset", {"expected_sig": g["sig"]})
        self.assertEqual(st, 200)
        self.assertTrue(d["removed"])
        self.assertFalse(self.tree_path.exists())
        self.assertEqual(Path(str(self.tree_path) + ".bak").read_text(encoding="utf-8"), saved)
        st, g = self.call("GET", "/api/behavior_tree")
        self.assertTrue(g["is_default"])

    def test_pin_warnings(self):
        ok = doc(N("goto_pin", pin_id="pinA", pin_name="ピンA"))
        st, d = self.save(ok)
        self.assertEqual(st, 200)
        self.assertEqual(d["pin_warnings"], [])
        missing = doc(N("selector", [N("goto_pin", pin_id="gone", pin_name="消えたピン")]))
        st, d = self.save(missing)
        self.assertEqual(st, 200)                      # 警告のみで保存は通る
        self.assertTrue(any("消えたピン" in w for w in d["pin_warnings"]))
        st, g = self.call("GET", "/api/behavior_tree")
        self.assertTrue(g["pin_warnings"])
        self.pins.store._loaded = None                 # 地図なし
        st, d = self.call("POST", "/api/behavior_tree/validate", {"document": ok})
        self.assertTrue(d["pin_warnings"])


class PermissionTests(ApiBase):
    def test_guest_can_read_but_not_write(self):
        self.save(doc())
        before = self.tree_path.read_bytes()
        for path in ("/api/behavior_tree", "/api/behavior_tree/state", "/behavior_tree"):
            st, _ = self.call("GET", path, cookie=self.guest, text=path == "/behavior_tree")
            self.assertEqual(st, 200, path)
        st, _ = self.save(doc(N("quiet")), cookie=self.guest)
        self.assertEqual(st, 403)
        st, _ = self.call("POST", "/api/behavior_tree/validate", {"document": doc()}, cookie=self.guest)
        self.assertEqual(st, 403)
        st, _ = self.call("POST", "/api/behavior_tree/reset", {}, cookie=self.guest)
        self.assertEqual(st, 403)
        self.assertEqual(self.tree_path.read_bytes(), before)


class LiveStateTests(ApiBase):
    TOPIC = "/nokosahen/behavior_tree/status"

    def feed(self, payload):
        _sub, cb = self.node.subs[self.TOPIC]
        cb(types.SimpleNamespace(data=payload if isinstance(payload, str) else json.dumps(payload)))

    def test_idle_means_no_subscription_until_touched(self):
        self.assertNotIn(self.TOPIC, self.node.subs)
        st, d = self.call("GET", "/api/behavior_tree/state")
        self.assertEqual(st, 200)
        self.assertIn(self.TOPIC, self.node.subs)
        self.assertEqual(d["status"], "waiting")

    def test_sync_synced_stale_and_default(self):
        self.call("GET", "/api/behavior_tree/state")
        # 木ファイルなし: directorも既定の木 → 一致
        self.feed({"doc_hash": bt.doc_hash(bt.default_document()), "tree_file": str(self.tree_path), "user_active": False})
        st, d = self.call("GET", "/api/behavior_tree/state")
        self.assertEqual(d["status"], "ok")
        self.assertEqual(d["sync"], "synced")
        self.assertFalse(d["path_mismatch"])
        # 保存した直後はdirectorがまだ古い木 → stale、読み込んだら synced
        st, s = self.save(doc())
        self.feed({"doc_hash": bt.doc_hash(bt.default_document()), "tree_file": str(self.tree_path)})
        st, d = self.call("GET", "/api/behavior_tree/state")
        self.assertEqual(d["sync"], "stale")
        self.feed({"doc_hash": s["doc_hash"], "tree_file": str(self.tree_path)})
        st, d = self.call("GET", "/api/behavior_tree/state")
        self.assertEqual(d["sync"], "synced")
        st, g = self.call("GET", "/api/behavior_tree")
        self.assertEqual(g["live"]["sync"], "synced")

    def test_path_mismatch_and_stale_feed_then_no_response(self):
        self.call("GET", "/api/behavior_tree/state")
        self.feed({"doc_hash": "x", "tree_file": "/somewhere/else.json"})
        st, d = self.call("GET", "/api/behavior_tree/state")
        self.assertTrue(d["path_mismatch"])
        with self.ctrl._lock:
            self.ctrl._rx -= 60.0
        st, d = self.call("GET", "/api/behavior_tree/state")
        self.assertEqual(d["status"], "no_response")
        self.assertIsNone(d["state"])
        self.assertIn("無応答", d["message"])

    def test_garbage_feed_ignored(self):
        self.call("GET", "/api/behavior_tree/state")
        self.feed("not json")
        st, d = self.call("GET", "/api/behavior_tree/state")
        self.assertIsNone(d["state"])

    def test_doc_hash_matches_runner(self):
        # directorが配信するdoc_hashと、保存ファイル由来のhashが同じ計算になっている
        st, s = self.save(doc())
        runner = bt.TreeRunner(str(self.tree_path))
        runner.check_reload()
        self.assertEqual(runner.status_dict(None)["doc_hash"], s["doc_hash"])


if __name__ == "__main__":
    unittest.main()
