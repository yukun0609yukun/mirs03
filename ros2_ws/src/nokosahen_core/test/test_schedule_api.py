"""スケジュールのAPI(/api/schedule*, /api/robot_clock)の結合テスト(2026-10-23)。docs/schedule.md。

ROS 2が無いPCでも動くよう、rclpy系モジュールをスタブしてdashboard_commonを読み込み、
実際にHTTPサーバーを立てて叩く(test_behavior_tree_api.pyと同じ方式)。
確認すること:
- 権限: 未ログイン401。guestは閲覧(GET)できるが 検証/保存/実行/停止 は403。
  operatorは 実行=schedule.run を許可された時だけ、停止=既定で可、保存/検証=付与しても不可(管理者のみ)。
- 検証(schedule_plan): 不正は保存されない(400)、プレビューは200で ok:false と理由。NaNは400。
- 保存: ~/.nokosahen_schedule.json(schedule_file)へ原子的に、直前のファイルは .bak。
- 実行: 保存済みのスケジュールを既存の木のファイルへコンパイルして書く(source:"schedule"、enabled:true、既存のパーサーで読める)。
  ユーザーが編集した木は上書き前に世代つき .before_schedule.<日時>.json へ退避(最新5個、同じ内容は重複しない、2回目の実行では退避を消さない)。{"cmd":"run"}を送る。
- 停止: 木のファイルを enabled:false にし {"cmd":"stop"} を送る(実行中でなくても合図は送る。ユーザーの木は触らない)。
- 状態: directorの木の状態(schedule部分)・機体の時計(robot_clock)の中継。届かない時は available:false。
ロボットには何も送らない(publisherは全てスタブ)。
実行: python3 -I ros2_ws/src/nokosahen_core/test/test_schedule_api.py
"""

import json
import os
import sys
import tempfile
import threading
import time
import types
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import test_behavior_rules_api as tra  # noqa: E402  (スタブ導入・FakeNode等を再利用)
from nokosahen_core import accounts as acc  # noqa: E402
from nokosahen_core import behavior_tree as bt  # noqa: E402
from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import map_pins as mp  # noqa: E402
from nokosahen_core import schedule_plan as sp  # noqa: E402
from nokosahen_core.behavior_tree_store import BehaviorTreeController  # noqa: E402
from nokosahen_core.schedule_controller import ScheduleController  # noqa: E402

OP_PASSWORD = "operator-pass-123"
OP_HASH = acc.hash_password(OP_PASSWORD, n=2 ** 10)
PIN_A = {"type": "pin", "pin_id": "pinA", "pin_name": "ピンA(入口横)"}
RECT = {"x_min": 0.0, "y_min": -1.0, "x_max": 4.0, "y_max": 2.0}


def E(eid, start, end=None, dest=None, policy="stay", boundary=None, **state):
    e = {"id": eid, "start": start, "destination": dest or {"type": "stay"}, "state": dict({"policy": policy}, **state)}
    if end:
        e["end"] = end
    if boundary:
        e["boundary"] = boundary
    return e


def sched(*entries, **kw):
    d = {"version": 1, "name": "昼のスケジュール", "entries": list(entries) or [
        E("lunch", "12:00", "13:00", dest=PIN_A, boundary={"mode": "release"}),
        E("pm", "13:00", "14:00", policy="quiet", boundary={"mode": "set", "rect": RECT}),
    ]}
    d.update(kw)
    return d


class Msg:
    def __init__(self, text):
        self.data = text


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


def serve(handler):
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


class ApiBase(unittest.TestCase):
    ACCOUNTS = False

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.tree_path = self.dir / "tree.json"
        self.sched_path = self.dir / "schedule.json"
        self.node = tra.FakeNode()
        self.pins = FakePinController(
            pins=[{"id": "pinA", "name": "ピンA", "x": 1.0, "y": 2.0, "yaw_deg": 0.0, "color": "#e11d48"}])
        self.tree_ctrl = BehaviorTreeController(
            self.node, str(self.tree_path), dc.String, dc.LazySubscription,
            pin_controller=self.pins, pins_payload_fn=mp.pins_payload)
        self.ctrl = ScheduleController(self.node, str(self.sched_path), self.tree_ctrl, dc.String, dc.LazySubscription)
        kw = {}
        if self.ACCOUNTS:
            accounts_file = self.dir / "accounts.yaml"
            self.perm_file = self.dir / "perm.json"
            self.write_accounts([])
            kw["account_store"] = acc.AccountStore(accounts_path=str(accounts_file), permissions_path=str(self.perm_file))
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, tra.FakeTeleop(),
            behavior_tree=self.tree_ctrl, schedule=self.ctrl,
            auth_username="TL", auth_password="pw", enforce_single_controller=False, **kw)
        self.server = serve(handler)
        self.port = self.server.server_address[1]
        self.tl = self.login("TL", "pw")
        self.guest = self.login("guest", "")

    def backups(self):
        return sorted(self.dir.glob("tree.before_schedule.*.json"))

    def write_accounts(self, perms):
        (self.dir / "accounts.yaml").write_text(
            'accounts:\n  - username: "kari-op"\n    role: operator\n'
            f'    password_hash: "{OP_HASH}"\n    enabled: true\n'
            f'    \noperator_permissions: {json.dumps(perms)}\n', encoding="utf-8")

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.tmp.cleanup()

    login = tra.ApiBase.login
    call = tra.ApiBase.call

    # --- ヘルパー ---

    def commands(self):
        return [json.loads(m.data)["cmd"] for m in self.node.pubs["/nokosahen/schedule/command"].sent]

    def save(self, schedule, **kw):
        return self.call("POST", "/api/schedule", {"schedule": schedule}, **kw)

    def tree_doc(self):
        return json.loads(self.tree_path.read_text(encoding="utf-8"))

    def feed_director(self, schedule_state, **extra):
        self.tree_ctrl.live_state()  # 購読を始める
        state = {"tree_file": str(self.tree_path), "schedule": schedule_state, "source": "schedule", "mode": "POLICY_STAY"}
        state.update(extra)
        self.tree_ctrl._on_state(Msg(json.dumps(state)))

    def feed_clock(self, **kw):
        self.ctrl.robot_clock()
        d = {"epoch": 1_800_000_000.0, "iso": "2026-10-23T12:34:56+09:00", "tz": "Asia/Tokyo",
             "clock_suspect": False, "since_boot_sec": 321.0}
        d.update(kw)
        self.ctrl._on_clock(Msg(json.dumps(d)))

    def leftovers(self):
        return sorted(p.name for p in self.dir.iterdir() if p.name.endswith(".tmp"))


class AuthTests(ApiBase):
    def test_unauthenticated(self):
        for method, path in (("GET", "/api/schedule"), ("GET", "/api/robot_clock"), ("POST", "/api/schedule"),
                             ("POST", "/api/schedule/preview"), ("POST", "/api/schedule/run"),
                             ("POST", "/api/schedule/stop")):
            st, _ = self.call(method, path, {} if method == "POST" else None, cookie="")
            self.assertEqual(st, 401, path)

    def test_guest_can_read_but_not_write(self):
        self.save(sched())
        st, d = self.call("GET", "/api/schedule", cookie=self.guest)
        self.assertEqual(st, 200)
        self.assertEqual(d["schedule"]["name"], "昼のスケジュール")
        st, _ = self.call("GET", "/api/robot_clock", cookie=self.guest)
        self.assertEqual(st, 200)
        before = self.sched_path.read_bytes()
        for path, body in (("/api/schedule", {"schedule": sched()}), ("/api/schedule/preview", {"schedule": sched()}),
                           ("/api/schedule/run", {}), ("/api/schedule/stop", {})):
            st, _ = self.call("POST", path, body, cookie=self.guest)
            self.assertEqual(st, 403, path)
        self.assertEqual(self.sched_path.read_bytes(), before)
        self.assertFalse(self.tree_path.exists())
        self.assertEqual(self.commands(), [])


class GetTests(ApiBase):
    def test_empty(self):
        st, d = self.call("GET", "/api/schedule")
        self.assertEqual(st, 200)
        self.assertEqual(set(d) & {"schedule", "running", "state", "robot_clock"}, {"schedule", "running", "state", "robot_clock"})
        self.assertIsNone(d["schedule"])
        self.assertFalse(d["running"])
        self.assertFalse(d["robot_clock"]["available"])
        self.assertIn("status", d["state"])

    def test_robot_clock_relay(self):
        st, d = self.call("GET", "/api/robot_clock")
        self.assertEqual(st, 200)
        self.assertFalse(d["available"])
        self.assertIn("server_epoch", d)
        self.feed_clock()
        st, d = self.call("GET", "/api/robot_clock")
        self.assertTrue(d["available"])
        self.assertEqual((d["iso"], d["tz"], d["clock_suspect"]), ("2026-10-23T12:34:56+09:00", "Asia/Tokyo", False))
        self.assertEqual(d["since_boot_sec"], 321.0)
        st, g = self.call("GET", "/api/schedule")
        self.assertEqual(g["robot_clock"]["iso"], d["iso"])
        # 届かなくなったら古い時刻を出さない
        self.ctrl._clock_rx -= 60
        st, d = self.call("GET", "/api/robot_clock")
        self.assertFalse(d["available"])
        self.assertIn("無応答", d["message"])

    def test_suspect_clock_is_relayed(self):
        self.feed_clock(iso="1970-01-01T09:00:05+09:00", clock_suspect=True)
        st, d = self.call("GET", "/api/robot_clock")
        self.assertTrue(d["clock_suspect"])

    def test_running_state_comes_from_director(self):
        self.save(sched())
        self.feed_director({"running": True, "armed": True, "active_entry_id": "lunch", "active_until": "2026-10-23T13:00:00",
                            "next_entry_id": "pm", "next_entry_at": "2026-10-23T13:00:00", "clock_suspect": False,
                            "entry_count": 2, "boundary_override": {"mode": "release"}})
        st, d = self.call("GET", "/api/schedule")
        self.assertTrue(d["running"])
        s = d["state"]
        self.assertEqual((s["status"], s["active_entry_id"], s["next_entry_id"]), ("ok", "lunch", "pm"))
        self.assertEqual(s["next_entry_at"], "2026-10-23T13:00:00")
        self.assertEqual(s["boundary_override"], {"mode": "release"})
        self.assertEqual(s["mode"], "POLICY_STAY")
        self.assertEqual(len(d["summary"]), 2)

    def test_startup_pending_and_safety_hold_are_relayed(self):
        self.feed_director({"running": False, "armed": False, "startup_pending": True}, safety_hold=True)
        st, d = self.call("GET", "/api/schedule")
        self.assertTrue(d["state"]["startup_pending"])
        self.assertTrue(d["state"]["safety_hold"])
        self.feed_director({"running": True, "armed": True})
        st, d = self.call("GET", "/api/schedule")
        self.assertFalse(d["state"]["startup_pending"])

    def test_not_running_when_director_is_silent(self):
        st, d = self.call("GET", "/api/schedule")
        self.assertFalse(d["running"])
        self.assertIn(d["state"]["status"], ("waiting", "no_response"))


class PreviewTests(ApiBase):
    def test_valid(self):
        st, d = self.call("POST", "/api/schedule/preview", {"schedule": sched()})
        self.assertEqual(st, 200)
        self.assertTrue(d["ok"])
        self.assertEqual(d["summary"][0], "12:00〜13:00 ピンA(入口横)へ行く → 待機、範囲は解除してNav2で走行")
        self.assertEqual(d["errors"], [])
        self.assertIsInstance(d["warnings"], list)
        self.assertFalse(self.sched_path.exists())  # プレビューは何も保存しない

    def test_invalid_returns_ok_false_with_reasons(self):
        st, d = self.call("POST", "/api/schedule/preview", {"schedule": sched(E("a", "25:00"))})
        self.assertEqual(st, 200)
        self.assertFalse(d["ok"])
        self.assertTrue(d["errors"])
        self.assertEqual(d["summary"], [])

    def test_overlap_is_a_warning(self):
        st, d = self.call("POST", "/api/schedule/preview", {"schedule": sched(E("a", "09:00", "11:00"), E("b", "10:00", "12:00"))})
        self.assertTrue(d["ok"])
        self.assertTrue(any("重なって" in w for w in d["warnings"]))

    def test_missing_pin_warning(self):
        s = sched(E("a", "09:00", "10:00", dest={"type": "pin", "pin_id": "ghost", "pin_name": "幽霊"}))
        st, d = self.call("POST", "/api/schedule/preview", {"schedule": s})
        self.assertTrue(d["ok"])
        self.assertTrue(any("幽霊" in w for w in d["warnings"]), d["warnings"])
        st, d = self.call("POST", "/api/schedule/preview", {"schedule": sched()})
        self.assertFalse(any("ピン" in w and "見つ" in w for w in d["warnings"]))

    def test_rect_warnings_in_preview_and_save(self):
        class FakeMap:
            def map_json(self):
                return {"origin_x": -1.0, "origin_y": -2.0, "resolution": 0.05, "width": 200, "height": 200}  # x -1..9, y -2..8

        class FakeBoundary:
            def __init__(self, st):
                self.st = st

            def status(self):
                return self.st

        s = sched(E("a", "09:00", "10:00", boundary={"mode": "set", "rect": dict(RECT, x_max=12.0)}))
        # 情報源が無い構成では警告なし
        st, d = self.call("POST", "/api/schedule/preview", {"schedule": s})
        self.assertTrue(d["ok"])
        self.assertFalse(any("地図" in w or "元の範囲" in w for w in d["warnings"]), d["warnings"])
        self.ctrl.map_source = FakeMap()
        self.ctrl.boundary_source = FakeBoundary({"enabled": True, "min_x": 0.5, "max_x": 3.0, "min_y": -0.5, "max_y": 1.5})
        st, d = self.call("POST", "/api/schedule/preview", {"schedule": s})
        self.assertTrue(d["ok"])
        self.assertTrue(any("地図の外" in w for w in d["warnings"]), d["warnings"])
        self.assertTrue(any("元の範囲の制限を外します" in w for w in d["warnings"]), d["warnings"])
        st, d = self.save(s)  # 警告は保存を妨げない
        self.assertEqual(st, 200, d)
        self.assertTrue(any("地図の外" in w for w in d["warnings"]))
        # 元の範囲が無効(制限なし)なら元の範囲の警告は出ない。取得で例外でも警告なし(保存・表示を止めない)
        self.ctrl.boundary_source = FakeBoundary({"enabled": False})
        st, d = self.call("POST", "/api/schedule/preview", {"schedule": s})
        self.assertFalse(any("元の範囲" in w for w in d["warnings"]), d["warnings"])
        self.ctrl.map_source = object()
        st, d = self.call("POST", "/api/schedule/preview", {"schedule": s})
        self.assertTrue(d["ok"])
        self.assertFalse(any("地図の外" in w for w in d["warnings"]))

    def test_tiny_rect_is_rejected_by_api(self):
        s = sched(E("a", "09:00", "10:00", boundary={"mode": "set", "rect": {"x_min": 0, "y_min": 0, "x_max": 0.001, "y_max": 0.001}}))
        st, d = self.call("POST", "/api/schedule/preview", {"schedule": s})
        self.assertFalse(d["ok"])
        self.assertIn("小さすぎ", d["errors"][0])
        st, d = self.save(s)
        self.assertEqual(st, 400)

    def test_bad_bodies(self):
        st, d = self.call("POST", "/api/schedule/preview", raw=b"not json")
        self.assertEqual(st, 400)
        self.assertFalse(d["ok"])
        raw = b'{"schedule": {"version": 1, "name": "x", "entries": [{"id": "a", "start": "09:00", "destination": {"type": "stay"}, "state": {"policy": "stay", "speed_scale": NaN}}]}}'
        st, d = self.call("POST", "/api/schedule/preview", raw=raw)
        self.assertEqual(st, 400)
        st, d = self.call("POST", "/api/schedule/preview", {"unrelated": 1})
        self.assertFalse(d["ok"])


class SaveTests(ApiBase):
    def test_save_valid(self):
        st, d = self.save(sched())
        self.assertEqual(st, 200, d)
        self.assertTrue(d["ok"])
        self.assertEqual(len(d["summary"]), 2)
        self.assertTrue(self.sched_path.exists())
        self.assertEqual(self.leftovers(), [])
        saved = json.loads(self.sched_path.read_text(encoding="utf-8"))
        self.assertEqual(saved["entries"][0]["id"], "lunch")
        st, g = self.call("GET", "/api/schedule")
        self.assertEqual(g["schedule"], saved)

    def test_body_may_be_the_schedule_itself(self):
        st, d = self.call("POST", "/api/schedule", sched())
        self.assertEqual(st, 200, d)

    def test_invalid_is_rejected_and_nothing_written(self):
        for bad in (sched(E("a", "09:00", "09:00")), {"schedule": 5}, sched(E("a", "09:00"), E("a", "10:00")),
                    {**sched(), "extra": 1}):
            st, d = self.call("POST", "/api/schedule", bad if "schedule" in bad else {"schedule": bad})
            self.assertEqual(st, 400, bad)
            self.assertFalse(d["ok"])
            self.assertTrue(d["errors"])
        self.assertFalse(self.sched_path.exists())
        self.save(sched())
        before = self.sched_path.read_bytes()
        st, _ = self.save(sched(E("a", "99:99")))
        self.assertEqual(st, 400)
        self.assertEqual(self.sched_path.read_bytes(), before)

    def test_nan_rejected(self):
        raw = b'{"schedule": {"version": 1, "name": "x", "entries": [{"id": "a", "start": "09:00", "destination": {"type": "stay"}, "state": {"policy": "stay", "speed_scale": Infinity}}]}}'
        st, _ = self.call("POST", "/api/schedule", raw=raw)
        self.assertEqual(st, 400)
        self.assertFalse(self.sched_path.exists())

    def test_second_save_keeps_backup(self):
        self.save(sched())
        first = self.sched_path.read_text(encoding="utf-8")
        self.save(sched(E("z", "08:00", "09:00")))
        self.assertEqual(Path(str(self.sched_path) + ".bak").read_text(encoding="utf-8"), first)

    def test_save_does_not_touch_the_tree_or_the_robot(self):
        self.save(sched())
        self.assertFalse(self.tree_path.exists())
        self.assertEqual(self.commands(), [])  # 保存では合図を出さない(出すのはrun/stopだけ)

    def test_save_while_running_warns_it_is_not_applied(self):
        self.feed_director({"running": True, "armed": True})
        st, d = self.save(sched())
        self.assertIn("反映されません", d["message"])

    def test_max_entries_accepted_and_51_rejected(self):
        entries = [E("e%d" % i, "%02d:%02d" % (i * 28 // 60, i * 28 % 60), dest=PIN_A, policy="quiet",
                     boundary={"mode": "set", "rect": RECT}, announce_interval_seconds=30, speed_scale=0.5) for i in range(50)]
        st, d = self.save(sched(*entries))
        self.assertEqual(st, 200, d)
        entries.append(E("extra", "23:59"))
        st, d = self.save(sched(*entries))
        self.assertEqual(st, 400)

    def test_corrupt_saved_file_is_reported(self):
        self.sched_path.write_text("{broken", encoding="utf-8")
        st, d = self.call("GET", "/api/schedule")
        self.assertEqual(st, 200)
        self.assertIsNone(d["schedule"])
        self.assertIn("schedule_error", d)
        st, d = self.call("POST", "/api/schedule/run", {})
        self.assertEqual(st, 400)
        self.assertFalse(d["ok"])


class RunTests(ApiBase):
    def test_run_without_saved_schedule(self):
        st, d = self.call("POST", "/api/schedule/run", {})
        self.assertEqual(st, 400)
        self.assertFalse(d["ok"])
        self.assertFalse(self.tree_path.exists())
        self.assertEqual(self.commands(), [])

    def test_run_writes_compiled_tree_and_sends_run(self):
        self.save(sched(run_on_startup=True))
        st, d = self.call("POST", "/api/schedule/run", {})
        self.assertEqual(st, 200, d)
        self.assertTrue(d["ok"])
        self.assertIn("上書き", d["message"])
        doc = self.tree_doc()
        self.assertEqual(doc["source"], "schedule")
        self.assertTrue(doc["enabled"])
        self.assertTrue(doc["run_on_startup"])
        self.assertEqual(doc["root"]["children"][-1]["type"], "rules_table")
        # directorと同じ読み込み(検証)を通る
        norm = bt.parse_tree_text(self.tree_path.read_text(encoding="utf-8"))
        self.assertEqual(norm["source"], "schedule")
        self.assertEqual(self.commands(), ["run"])
        self.assertEqual(self.leftovers(), [])
        # 木のページ(既存API)からも読める
        st, g = self.call("GET", "/api/behavior_tree")
        self.assertEqual(g["document"]["source"], "schedule")

    def test_user_tree_is_backed_up_before_overwrite(self):
        user = {"version": 1, "description": "手で作った木", "min_hold_seconds": 10, "enabled": True,
                "root": {"type": "selector", "children": [{"type": "quiet"}]}}
        self.tree_path.write_text(json.dumps(user, ensure_ascii=False), encoding="utf-8")
        original = self.tree_path.read_bytes()
        self.save(sched())
        st, d = self.call("POST", "/api/schedule/run", {})
        self.assertEqual(st, 200, d)
        self.assertEqual(len(self.backups()), 1)
        self.assertEqual(self.backups()[0].read_bytes(), original)
        self.assertIn("before_schedule", d["message"])
        self.assertEqual(d["backup"], str(self.backups()[0]))
        self.assertIn(d["backup"], d["message"])
        self.assertEqual(self.tree_doc()["source"], "schedule")
        # もう一度実行しても、ユーザーの木の退避は消えない(今あるのはスケジュール由来の木なので退避しない)
        st, d = self.call("POST", "/api/schedule/run", {})
        self.assertEqual(st, 200)
        self.assertEqual([p.read_bytes() for p in self.backups()], [original])
        self.assertIsNone(d["backup"])

    def _user_tree(self, text):
        doc = {"version": 1, "description": text, "min_hold_seconds": 10, "enabled": True,
               "root": {"type": "selector", "children": [{"type": "quiet"}]}}
        self.tree_path.write_text(json.dumps(doc, ensure_ascii=False), encoding="utf-8")
        return self.tree_path.read_bytes()

    def test_run_stop_edit_run_keeps_both_user_trees(self):
        self.save(sched())
        first = self._user_tree("最初の木")
        self.call("POST", "/api/schedule/run", {})
        self.call("POST", "/api/schedule/stop", {})
        second = self._user_tree("編集後の木")
        st, d = self.call("POST", "/api/schedule/run", {})
        self.assertEqual(st, 200, d)
        self.assertEqual(sorted(p.read_bytes() for p in self.backups()), sorted([first, second]))
        self.assertEqual(Path(d["backup"]).read_bytes(), second)

    def test_same_content_backup_is_not_duplicated(self):
        self.save(sched())
        original = self._user_tree("同じ木")
        self.call("POST", "/api/schedule/run", {})
        self.call("POST", "/api/schedule/stop", {})
        self._user_tree("同じ木")  # 内容が同じ木に戻した
        st, d = self.call("POST", "/api/schedule/run", {})
        self.assertEqual(st, 200, d)
        self.assertEqual(len(self.backups()), 1)
        self.assertEqual(Path(d["backup"]).read_bytes(), original)

    def test_only_latest_five_backups_are_kept(self):
        self.save(sched())
        for i in range(8):
            self._user_tree(f"木{i}")
            st, d = self.call("POST", "/api/schedule/run", {})
            self.assertEqual(st, 200, d)
            self.call("POST", "/api/schedule/stop", {})
        files = self.backups()
        self.assertEqual(len(files), 5)
        descs = sorted(json.loads(p.read_text(encoding="utf-8"))["description"] for p in files)
        self.assertEqual(descs, [f"木{i}" for i in range(3, 8)])  # 古い3個(木0〜2)が消えている

    def test_corrupt_user_tree_is_also_backed_up(self):
        self.tree_path.write_text("{ broken", encoding="utf-8")
        self.save(sched())
        st, d = self.call("POST", "/api/schedule/run", {})
        self.assertEqual(st, 200, d)
        self.assertEqual([p.read_text(encoding="utf-8") for p in self.backups()], ["{ broken"])

    def test_no_backup_when_there_was_no_tree(self):
        self.save(sched())
        self.call("POST", "/api/schedule/run", {})
        self.assertEqual(self.backups(), [])

    def test_run_reports_when_director_is_silent(self):
        self.save(sched())
        st, d = self.call("POST", "/api/schedule/run", {})
        self.assertFalse(d["director_ok"])
        self.assertIn("応答", d["message"])
        self.feed_director({"running": False, "armed": False})
        st, d = self.call("POST", "/api/schedule/run", {})
        self.assertTrue(d["director_ok"])

    def test_run_uses_the_saved_schedule_not_the_request_body(self):
        self.save(sched())
        st, d = self.call("POST", "/api/schedule/run", {"schedule": sched(E("hack", "00:00"))})
        self.assertEqual(st, 200)
        names = [c.get("name") for c in self.tree_doc()["root"]["children"]]
        self.assertIn("entry:lunch", names)
        self.assertNotIn("entry:hack", names)


class StopTests(ApiBase):
    def test_stop_disables_the_schedule_tree_and_sends_stop(self):
        self.save(sched())
        self.call("POST", "/api/schedule/run", {})
        st, d = self.call("POST", "/api/schedule/stop", {})
        self.assertEqual(st, 200, d)
        self.assertTrue(d["ok"])
        doc = self.tree_doc()
        self.assertFalse(doc["enabled"])
        self.assertEqual(doc["source"], "schedule")
        self.assertEqual(self.commands(), ["run", "stop"])
        # 無効な木はdirectorで従来動作(ルール表のみ)になる
        runner = bt.TreeRunner(str(self.tree_path))
        runner.check_reload()
        self.assertFalse(runner.user_active)

    def test_stop_when_nothing_runs_still_sends_the_signal(self):
        st, d = self.call("POST", "/api/schedule/stop", {})
        self.assertEqual(st, 200)
        self.assertTrue(d["ok"])
        self.assertIn("実行中ではありません", d["message"])
        self.assertEqual(self.commands(), ["stop"])
        self.assertFalse(self.tree_path.exists())

    def test_stop_does_not_touch_a_user_tree(self):
        user = {"version": 1, "min_hold_seconds": 10, "enabled": True, "root": {"type": "quiet"}}
        self.tree_path.write_text(json.dumps(user), encoding="utf-8")
        before = self.tree_path.read_bytes()
        st, d = self.call("POST", "/api/schedule/stop", {})
        self.assertEqual(st, 200)
        self.assertEqual(self.tree_path.read_bytes(), before)
        self.assertEqual(self.commands(), ["stop"])

    def test_stop_twice_is_harmless(self):
        self.save(sched())
        self.call("POST", "/api/schedule/run", {})
        self.call("POST", "/api/schedule/stop", {})
        st, d = self.call("POST", "/api/schedule/stop", {})
        self.assertEqual(st, 200)
        self.assertFalse(self.tree_doc()["enabled"])

    def test_stop_message_points_to_the_saved_user_tree(self):
        self.tree_path.write_text(json.dumps({"version": 1, "root": {"type": "quiet"}}), encoding="utf-8")
        self.save(sched())
        self.call("POST", "/api/schedule/run", {})
        st, d = self.call("POST", "/api/schedule/stop", {})
        self.assertIn("before_schedule", d["message"])

    def test_run_then_stop_then_run_again(self):
        self.save(sched())
        for _ in range(2):
            self.call("POST", "/api/schedule/run", {})
            self.assertTrue(self.tree_doc()["enabled"])
            self.call("POST", "/api/schedule/stop", {})
            self.assertFalse(self.tree_doc()["enabled"])
        self.assertEqual(self.commands(), ["run", "stop", "run", "stop"])


class OperatorTests(ApiBase):
    ACCOUNTS = True

    def op(self, perms):
        self.write_accounts(perms)
        # 許可リストの変更を反映するため、サーバー内のAccountStoreを作り直さずに読み直せる管理APIを使う
        st, d = self.call("POST", "/api/accounts/permissions", {"operator_permissions": perms})
        self.assertEqual(st, 200, d)
        return self.login("kari-op", OP_PASSWORD)

    def test_operator_default_can_only_stop(self):
        ck = self.op([])
        self.save(sched())
        for path, body in (("/api/schedule", {"schedule": sched()}), ("/api/schedule/preview", {"schedule": sched()}),
                           ("/api/schedule/run", {})):
            st, d = self.call("POST", path, body, cookie=ck)
            self.assertEqual(st, 403, path)
        self.assertFalse(self.tree_path.exists())
        st, d = self.call("POST", "/api/schedule/stop", {}, cookie=ck)
        self.assertEqual(st, 200)
        st, _ = self.call("GET", "/api/schedule", cookie=ck)
        self.assertEqual(st, 200)

    def test_run_permission_allows_run_but_never_save(self):
        ck = self.op(["schedule.run"])
        self.save(sched())
        st, d = self.call("POST", "/api/schedule/run", {}, cookie=ck)
        self.assertEqual(st, 200, d)
        self.assertEqual(self.tree_doc()["source"], "schedule")
        st, d = self.call("POST", "/api/schedule", {"schedule": sched(E("x", "01:00"))}, cookie=ck)
        self.assertEqual(st, 403)
        st, d = self.call("POST", "/api/schedule/preview", {"schedule": sched()}, cookie=ck)
        self.assertEqual(st, 403)
        self.assertEqual(json.loads(self.sched_path.read_text(encoding="utf-8"))["entries"][0]["id"], "lunch")

    def test_save_permission_cannot_be_granted(self):
        st, d = self.call("POST", "/api/accounts/permissions", {"operator_permissions": ["behavior.tree_edit"]})
        self.assertEqual(st, 400)


if __name__ == "__main__":
    unittest.main()
