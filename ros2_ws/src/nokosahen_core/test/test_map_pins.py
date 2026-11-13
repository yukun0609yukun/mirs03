"""地図ピン(名前付きの場所)と目的地の精度向上(2026-10-07)のテスト。

- PinStore: 追加/更新/削除/上限/検証/原子的書き込み/地図の保存・読み込みの往復/セッションの消去
- snap_goal_to_free: 空きセル/占有セル/壁際/未知セル/地図の端
- HTTP(ROSはスタブ): /api/pins(+add/update/delete/goto)、/api/goal(スナップ・向き)、/api/goal/status|cancel
  (有効 / NaN / 巨大値 / guest 403 / 送ったメッセージの中身 / ソフト緊急停止409 / 範囲外409 / 現在位置なし409)
- JS(nodeがあれば): クリック->世界座標の往復(表示回転・セル内の位置)、ピンのタップ判定
- nav2 params: 許容誤差が引き締まり、速度の値は変わらないこと

ロボットには何も送らない(publisherはスタブ)。
実行: python ros2_ws/src/nokosahen_core/test/test_map_pins.py
"""

import json
import math
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
from http.server import ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace as NS
from unittest import mock

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
BRINGUP = PKG.parent / "nokosahen_bringup"


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
    mod("std_msgs.msg", Bool=Msg, Float32MultiArray=Msg, String=Msg)


try:
    import geometry_msgs.msg  # noqa: F401
    import nav_msgs.msg  # noqa: F401
except Exception:  # noqa: BLE001
    _install_stubs()

from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import map_pins as mp  # noqa: E402
from nokosahen_core import nav_speed as ns  # noqa: E402
from nokosahen_core.map_view import MAP_VIEW_MATH_JS  # noqa: E402

NODE = shutil.which("node")


# ------------------------------------------------------------------ PinStore
class PinStoreTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.maps = self.dir / "maps"
        self.sess = self.dir / "session.json"
        self.store = mp.PinStore(str(self.maps), str(self.sess))

    def tearDown(self):
        self.tmp.cleanup()

    def test_add_update_delete(self):
        p = self.store.add("入口", "#E11D48", 1.25, -2.5, 90)
        self.assertEqual((p["name"], p["x"], p["y"], p["yaw_deg"], p["color"]), ("入口", 1.25, -2.5, 90.0, "#e11d48"))
        self.assertEqual(p["map_name"], "")
        self.assertTrue(p["created_at"] > 0)
        self.assertEqual(len(self.store.list()), 1)
        q = self.store.update(p["id"], {"name": "レジ前", "x": 3.0, "y": 4.0, "yaw_deg": 270})
        self.assertEqual((q["name"], q["x"], q["y"], q["yaw_deg"]), ("レジ前", 3.0, 4.0, -90.0))  # 正規化
        self.assertEqual(q["color"], "#e11d48")  # 指定しなかった項目は保つ
        d = self.store.delete(p["id"])
        self.assertEqual(d["id"], p["id"])
        self.assertEqual(self.store.list(), [])

    def test_auto_name_and_color(self):
        a = self.store.add("", None, 0, 0, 0)
        b = self.store.add(None, None, 1, 0, 0)
        self.assertEqual((a["name"], b["name"]), ("ピン1", "ピン2"))
        self.assertNotEqual(a["color"], b["color"])
        self.assertTrue(all(c.startswith("#") and len(c) == 7 for c in (a["color"], b["color"])))

    def test_name_whitespace_and_limit(self):
        p = self.store.add("  出口\n 付近  ", None, 0, 0, 0)
        self.assertEqual(p["name"], "出口 付近")
        self.store.add("あ" * 40, None, 0, 0, 0)  # ちょうど40文字はOK
        with self.assertRaises(mp.PinError):
            self.store.add("あ" * 41, None, 0, 0, 0)
        with self.assertRaises(mp.PinError):
            self.store.add(123, None, 0, 0, 0)

    def test_validation(self):
        bad_colors = ["red", "#fff", "#12345g", "javascript:alert(1)", 5, "#1234567", "url(x)"]
        for c in bad_colors:
            with self.assertRaises(mp.PinError, msg=repr(c)):
                self.store.add("n", c, 0, 0, 0)
        p = self.store.add("n", None, 0, 0, 0)
        for fields in ({}, {"name": ""}, {"name": "   "}, {"color": None}, {"color": "bad"},
                       {"x": 1.0}, {"x": float("nan"), "y": 0}, {"x": 0, "y": float("inf")},
                       {"x": 1001, "y": 0}, {"x": 0, "y": -1e9}, {"x": "1", "y": 0}, {"x": True, "y": 0},
                       {"x": 0, "y": 0, "yaw_deg": float("nan")}):
            with self.assertRaises(mp.PinError, msg=repr(fields)):
                self.store.update(p["id"], fields)
        # 失敗した更新は何も変えない
        self.assertEqual(self.store.get(p["id"]), p)
        with self.assertRaises(mp.PinError) as cm:
            self.store.update("nope", {"name": "x"})
        self.assertEqual(cm.exception.kind, "not_found")
        with self.assertRaises(mp.PinError) as cm:
            self.store.delete("nope")
        self.assertEqual(cm.exception.kind, "not_found")

    def test_parse_pose(self):
        self.assertEqual(mp.parse_pose({"x": 1, "y": 2, "yaw_deg": 540}), (1.0, 2.0, 180.0))
        self.assertEqual(mp.parse_pose({"x": 1, "y": 2}), (1.0, 2.0, 0.0))
        for b in (None, [], "x", {"x": 1}, {"x": 1e6, "y": 0}, {"x": 0, "y": float("-inf")}):
            with self.assertRaises(mp.PinError, msg=repr(b)):
                mp.parse_pose(b)

    def test_limit_200(self):
        for i in range(mp.PIN_MAX_COUNT):
            self.store.add(f"p{i}", None, i * 0.01, 0, 0)
        with self.assertRaises(mp.PinError) as cm:
            self.store.add("x", None, 0, 0, 0)
        self.assertEqual(cm.exception.kind, "limit")
        self.assertEqual(len(self.store.list()), 200)
        # 1つ消せばまた追加できる
        self.store.delete(self.store.list()[0]["id"])
        self.store.add("again", None, 0, 0, 0)

    def test_session_persists_across_instances(self):
        p = self.store.add("A", "#2563eb", 1.5, 2.5, 45)
        again = mp.PinStore(str(self.maps), str(self.sess))
        self.assertEqual(again.list(), [p])

    def test_atomic_write_leaves_no_temp_and_survives_failure(self):
        self.store.add("A", None, 1, 1, 0)
        before = self.sess.read_text(encoding="utf-8")
        # 書き込み中の失敗(os.replaceが落ちる)でも、既存ファイルは壊れず、一時ファイルも残らない
        with mock.patch("nokosahen_core.map_pins.os.replace", side_effect=OSError("disk")):
            self.store.add("B", None, 2, 2, 0)  # 保存失敗でも画面操作は止めない
        self.assertEqual(self.sess.read_text(encoding="utf-8"), before)
        json.loads(before)
        leftovers = [p.name for p in self.dir.iterdir() if p.name.endswith(".tmp")]
        self.assertEqual(leftovers, [])
        # 通常の保存でも一時ファイルは残らない
        self.store.add("C", None, 3, 3, 0)
        self.assertEqual([p.name for p in self.dir.iterdir() if p.name.endswith(".tmp")], [])

    def test_map_save_load_round_trip(self):
        a = self.store.add("入口", "#e11d48", 1.123456789, -2.987654321, 33.3)
        b = self.store.add("棚", "#16a34a", 5.0, 6.0, -120.0)
        n = self.store.save_for_map("shop1")
        self.assertEqual(n, 2)
        f = self.maps / "shop1.pins.json"
        data = json.loads(f.read_text(encoding="utf-8"))
        self.assertEqual(data["map_name"], "shop1")
        self.assertTrue(all(p["map_name"] == "shop1" for p in data["pins"]))
        # 別のセッションで読み込むと、x,y,yawがビット一致で復元される(厳密に送るため)
        other = mp.PinStore(str(self.maps), str(self.dir / "other_session.json"))
        self.assertEqual(other.on_map_loaded("shop1"), 2)
        got = {p["id"]: p for p in other.list()}
        for src in (a, b):
            g = got[src["id"]]
            self.assertEqual((g["x"], g["y"], g["yaw_deg"], g["name"], g["color"]), (src["x"], src["y"], src["yaw_deg"], src["name"], src["color"]))
            self.assertEqual(g["map_name"], "shop1")
        self.assertEqual(other.loaded_map(), "shop1")

    def test_loaded_map_edits_write_through(self):
        self.store.add("入口", None, 1, 1, 0)
        self.store.save_for_map("m1")
        self.store.on_map_loaded("m1")
        p = self.store.add("棚", None, 2, 2, 0)
        on_disk = json.loads((self.maps / "m1.pins.json").read_text(encoding="utf-8"))
        self.assertEqual(len(on_disk["pins"]), 2)
        self.assertEqual(p["map_name"], "m1")
        self.store.delete(p["id"])
        on_disk = json.loads((self.maps / "m1.pins.json").read_text(encoding="utf-8"))
        self.assertEqual(len(on_disk["pins"]), 1)

    def test_load_missing_file_gives_empty(self):
        self.store.add("残るべきでない", None, 0, 0, 0)
        self.assertEqual(self.store.on_map_loaded("no_such_map"), 0)
        self.assertEqual(self.store.list(), [])

    def test_clear_session_keeps_saved_map_pins(self):
        self.store.add("入口", None, 1, 1, 0)
        self.store.save_for_map("m2")
        self.store.clear_session()
        self.assertEqual(self.store.list(), [])
        self.assertIsNone(self.store.loaded_map())
        self.assertEqual(json.loads(self.sess.read_text(encoding="utf-8"))["pins"], [])
        self.assertTrue((self.maps / "m2.pins.json").exists())  # 保存済みの地図のピンは消さない
        self.assertEqual(mp.PinStore(str(self.maps), str(self.sess)).list(), [])  # 再起動後も空

    def test_corrupt_files_do_not_crash(self):
        self.sess.write_text("{not json", encoding="utf-8")
        self.assertEqual(mp.PinStore(str(self.maps), str(self.sess)).list(), [])
        self.maps.mkdir(parents=True, exist_ok=True)
        (self.maps / "bad.pins.json").write_text(json.dumps({"pins": [
            {"id": "ok1", "name": "OK", "x": 1, "y": 2, "yaw_deg": 0, "color": "#e11d48"},
            {"id": "nan", "name": "NaN", "x": float("nan"), "y": 2, "yaw_deg": 0, "color": "#e11d48"},
            {"id": "far", "name": "far", "x": 1e9, "y": 2, "yaw_deg": 0, "color": "#e11d48"},
            {"id": "dup", "name": "a", "x": 1, "y": 1}, {"id": "dup", "name": "b", "x": 2, "y": 2},
            "junk", 5, None, {"name": "no id", "x": 1, "y": 1},
        ]}), encoding="utf-8")
        self.assertEqual(self.store.on_map_loaded("bad"), 2)  # ok1とdupの1件目だけ
        self.assertEqual({p["id"] for p in self.store.list()}, {"ok1", "dup"})

    def test_unsafe_map_names_rejected(self):
        for n in ("../x", "a/b", "", "a b", "x" * 65):
            with self.assertRaises(ValueError, msg=n):
                self.store.save_for_map(n)

    def test_quaternion(self):
        for deg in (0, 30, 90, -45, 179, -179, 180):
            q = mp.yaw_deg_to_quaternion(deg)
            self.assertAlmostEqual(math.sqrt(sum(v * v for v in q)), 1.0)
            self.assertEqual((q[0], q[1]), (0.0, 0.0))
            yaw = math.atan2(2 * q[3] * q[2], 1 - 2 * q[2] * q[2])
            self.assertAlmostEqual(math.cos(yaw - math.radians(deg)), 1.0, places=9)
        self.assertAlmostEqual(mp.yaw_deg_to_quaternion(90)[2], math.sqrt(0.5))
        self.assertEqual(mp.yaw_deg_to_quaternion(0), (0.0, 0.0, 0.0, 1.0))


# ------------------------------------------------------------------ snap
def make_grid(w, h, fill=0):
    return [fill] * (w * h)


def set_cell(data, w, c, r, v):
    data[r * w + c] = v


class SnapTests(unittest.TestCase):
    RES = 0.05
    OX, OY = -1.0, -1.0
    W = H = 60  # 3m x 3m

    def grid(self):
        return make_grid(self.W, self.H, 0)

    def snap(self, data, x, y, **kw):
        return mp.snap_goal_to_free(self.W, self.H, self.RES, self.OX, self.OY, data, x, y, **kw)

    def center(self, c, r):
        return self.OX + (c + 0.5) * self.RES, self.OY + (r + 0.5) * self.RES

    def test_inside_free_unchanged(self):
        d = self.grid()
        x, y = self.center(30, 30)
        out = self.snap(d, x + 0.013, y - 0.011)
        self.assertEqual(out["status"], "ok")
        self.assertEqual((out["x"], out["y"], out["shift_m"]), (x + 0.013, y - 0.011, 0.0))

    def test_inside_occupied_snaps_to_nearest_free_with_clearance(self):
        d = self.grid()
        # 縦の壁(列30、全行)。ロボット半径0.28+0.04=0.32m(6.4セル)以内は不可
        for r in range(self.H):
            set_cell(d, self.W, 30, r, 100)
        x, y = self.center(30, 30)
        out = self.snap(d, x, y)
        self.assertEqual(out["status"], "snapped")
        self.assertGreaterEqual(out["shift_m"], 0.32 - 0.05)
        self.assertLessEqual(out["shift_m"], 0.5)
        # 寄せた先は壁から半径+余裕以上離れている
        self.assertGreater(abs(out["x"] - x), 0.3)
        # セル中心に寄せている
        c = (out["x"] - self.OX) / self.RES - 0.5
        self.assertAlmostEqual(c, round(c), places=9)
        self.assertEqual(out["reason"], "occupied")

    def test_near_wall_free_cell_is_pushed_out(self):
        d = self.grid()
        for r in range(self.H):
            set_cell(d, self.W, 30, r, 100)
        # 壁の2セル右(10cm): 自由セルだが半径内 -> 寄せる
        x, y = self.center(32, 30)
        out = self.snap(d, x, y)
        self.assertEqual(out["status"], "snapped")
        self.assertEqual(out["reason"], "near_obstacle")
        self.assertGreater(out["x"], x)  # 壁から離れる側(右)へ
        # 壁から十分離れた所は変更なし
        x2, y2 = self.center(45, 30)
        self.assertEqual(self.snap(d, x2, y2)["status"], "ok")

    def test_occupied_with_no_free_within_range_is_no_free(self):
        d = make_grid(self.W, self.H, 100)  # 全部占有
        x, y = self.center(30, 30)
        self.assertEqual(self.snap(d, x, y)["status"], "no_free")

    def test_unknown_cell_snaps_if_free_near_else_unchanged(self):
        d = self.grid()
        for r in range(25, 36):
            for c in range(25, 36):
                set_cell(d, self.W, c, r, -1)  # 0.55m角の未知領域
        x, y = self.center(30, 30)  # 未知の中心: 最寄りの自由は0.3m弱先
        out = self.snap(d, x, y)
        self.assertEqual(out["status"], "snapped")
        self.assertEqual(out["reason"], "unknown")
        self.assertLessEqual(out["shift_m"], 0.5)
        # 全面未知: 寄せ先が無いのでそのまま(Nav2のallow_unknownに任せる)
        allunk = make_grid(self.W, self.H, -1)
        out = self.snap(allunk, x, y)
        self.assertEqual((out["status"], out["x"], out["y"]), ("unchanged", x, y))

    def test_map_edge_and_outside(self):
        d = self.grid()
        # 端のセル: 地図外は未知扱いだが占有ではないので、端の自由セルはOK
        x, y = self.center(0, 0)
        self.assertEqual(self.snap(d, x, y)["status"], "ok")
        # 地図の外: 未知セルとして、近くに自由が無ければそのまま
        out = self.snap(d, self.OX - 5.0, self.OY - 5.0)
        self.assertEqual(out["status"], "unchanged")
        # 地図の外だが端に近い(0.2m外): 地図内の自由セルへ寄せられる
        out = self.snap(d, self.OX - 0.2, 0.0)
        self.assertEqual(out["status"], "snapped")
        self.assertGreaterEqual(out["x"], self.OX)
        # 壁が端に接している場合も壁の外側へ寄せる
        for r in range(self.H):
            set_cell(d, self.W, 0, r, 100)
        out = self.snap(d, *self.center(0, 30))
        self.assertEqual(out["status"], "snapped")

    def test_bad_map_inputs(self):
        for args in ((0, 10, 0.05, 0, 0, [0] * 100), (10, 10, 0.0, 0, 0, [0] * 100), (10, 10, 0.05, 0, 0, [0] * 5)):
            out = mp.snap_goal_to_free(*args, 0.1, 0.1)
            self.assertEqual(out["status"], "no_map")

    def test_max_shift_respected(self):
        d = self.grid()
        for r in range(self.H):
            set_cell(d, self.W, 30, r, 100)
        x, y = self.center(30, 30)
        out = self.snap(d, x, y, max_shift=0.1)  # 0.1m以内には空きが無い
        self.assertEqual(out["status"], "no_free")

    def test_rotated_grid_orientation_row0_is_origin_y(self):
        # 行0 = origin_yの行(下)。上側(y大)の壁にだけ近い点は下へ寄せられる
        d = self.grid()
        for c in range(self.W):
            set_cell(d, self.W, c, 50, 100)  # y = OY + 50.5*0.05 付近
        wx, wy = self.center(30, 50)
        out = self.snap(d, wx, wy - 0.05)  # 壁のすぐ下
        self.assertEqual(out["status"], "snapped")
        self.assertLess(out["y"], wy - 0.05)  # 壁から遠ざかる=下(y小)


# ------------------------------------------------------------------ HTTPスタブ
class FakeLogger:
    def warn(self, *a, **k):
        pass

    info = error = debug = warn


class FakePub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class FakeClock:
    def now(self):
        return self

    def to_msg(self):
        return NS(sec=1234, nanosec=5)


class FakeNode:
    def __init__(self):
        self.pubs = {}
        self.subs = {}

    def create_publisher(self, typ, topic, depth):
        self.pubs[topic] = FakePub()
        return self.pubs[topic]

    def create_subscription(self, typ, topic, cb, qos):
        self.subs[topic] = cb
        return NS(topic=topic)

    def create_timer(self, period, cb):
        return None

    def get_clock(self):
        return FakeClock()

    def get_logger(self):
        return FakeLogger()


class FakePoseStamped:
    def __init__(self):
        self.header = NS(frame_id="", stamp=None)
        self.pose = NS(position=NS(x=0.0, y=0.0, z=0.0), orientation=NS(x=0.0, y=0.0, z=0.0, w=0.0))


class FakeBool:
    def __init__(self):
        self.data = None


class FakeRobotPose:
    def __init__(self):
        self.pose = {"x": 0.5, "y": 0.25, "yaw": math.radians(30), "age_s": 0.1}

    def get_pose(self):
        return None if self.pose is None else dict(self.pose)


class FakeLifecycle:
    def __init__(self, mode="localization"):
        self.mode = mode

    def status(self):
        return {"mode": self.mode, "loaded_map": "lab", "running": True, "last_error": ""}


class FakeTeleop:
    def __init__(self):
        self.soft_estop_engaged = False
        self.goal_mode = 0

    def activate_goal_mode(self):
        self.goal_mode += 1


class ApiBase(unittest.TestCase):
    def setUp(self):
        self._saved = (dc.PoseStamped, dc.Bool)
        dc.PoseStamped = FakePoseStamped
        dc.Bool = FakeBool
        self.tmp = tempfile.TemporaryDirectory()
        tmp = Path(self.tmp.name)
        self.node = FakeNode()
        self.pose = FakeRobotPose()
        self.life = FakeLifecycle()
        self.map_goal = dc.MapGoalController(self.node, robot_pose=self.pose)
        self.store = mp.PinStore(str(tmp / "maps"), str(tmp / "session.json"))
        self.pins = dc.PinController(self.node, self.store, self.pose, self.map_goal, self.life)
        self.boundary = dc.BoundaryController(self.node, str(tmp / "boundary.json"))
        self.teleop = FakeTeleop()
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, self.teleop,
            map_goal=self.map_goal, pins=self.pins, boundary=self.boundary,
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
        dc.PoseStamped, dc.Bool = self._saved
        self.tmp.cleanup()

    def login(self, user, password):
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.port}/api/login",
            data=json.dumps({"username": user, "password": password}).encode(),
            method="POST", headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.headers.get("Set-Cookie", "").split(";")[0]

    def call(self, method, path, body=None, raw=None, cookie=None):
        cookie = self.tl if cookie is None else cookie
        data = raw if raw is not None else (None if body is None else json.dumps(body).encode())
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.port}{path}", data=data, method=method,
            headers={"Content-Type": "application/json", "Cookie": cookie},
        )
        try:
            with urllib.request.urlopen(req, timeout=10) as r:
                return r.status, json.loads(r.read().decode("utf-8"))
        except urllib.error.HTTPError as e:
            try:
                return e.code, json.loads(e.read().decode("utf-8"))
            finally:
                e.close()

    def goals(self):
        return self.node.pubs["/goal_pose"].sent

    def add_pin(self, **kw):
        body = {"name": "入口", "from": {"x": 2.0, "y": -1.0, "yaw_deg": 90}}
        body.update(kw)
        st, d = self.call("POST", "/api/pins/add", body)
        self.assertEqual(st, 200, d)
        return d["pin"]


class PinsApi(ApiBase):
    def test_requires_login(self):
        req = urllib.request.Request(f"http://127.0.0.1:{self.port}/api/pins", method="GET")
        with self.assertRaises(urllib.error.HTTPError) as cm:
            urllib.request.urlopen(req, timeout=10)
        self.assertEqual(cm.exception.code, 401)
        cm.exception.close()

    def test_add_from_explicit_pose_and_list(self):
        pin = self.add_pin(color="#2563eb")
        self.assertEqual((pin["x"], pin["y"], pin["yaw_deg"], pin["color"]), (2.0, -1.0, 90.0, "#2563eb"))
        st, d = self.call("GET", "/api/pins")
        self.assertEqual(st, 200)
        self.assertEqual(d["count"], 1)
        self.assertEqual(d["max"], 200)
        self.assertEqual(d["pins"][0]["id"], pin["id"])
        # ロボット(0.5,0.25)からの距離
        self.assertAlmostEqual(d["pins"][0]["distance_m"], math.hypot(1.5, 1.25), places=3)
        self.assertAlmostEqual(d["pose"]["yaw_deg"], 30.0, places=6)

    def test_add_from_robot_uses_current_pose(self):
        st, d = self.call("POST", "/api/pins/add", {"name": "ここ", "from": "robot"})
        self.assertEqual(st, 200, d)
        p = d["pin"]
        self.assertEqual((p["x"], p["y"]), (0.5, 0.25))
        self.assertAlmostEqual(p["yaw_deg"], 30.0, places=9)

    def test_slam_note_when_mapping(self):
        self.life.mode = "mapping"
        st, d = self.call("POST", "/api/pins/add", {"from": "robot"})
        self.assertEqual(st, 200)
        self.assertIn("ループクロージャ", d["message"])
        st, d = self.call("GET", "/api/pins")
        self.assertIn("ループクロージャ", d["note"])
        self.life.mode = "localization"
        st, d = self.call("GET", "/api/pins")
        self.assertEqual(d["note"], "")

    def test_add_from_robot_without_pose_is_409(self):
        self.pose.pose = None
        st, d = self.call("POST", "/api/pins/add", {"from": "robot"})
        self.assertEqual(st, 409)
        self.assertEqual(d["error"], "no_pose")
        self.assertIn("位置合わせ", d["message"])  # localizationモードの案内
        self.life.mode = "mapping"
        st, d = self.call("POST", "/api/pins/add", {"from": "robot"})
        self.assertEqual(st, 409)
        self.assertIn("TF", d["message"])
        self.assertEqual(self.store.list(), [])

    def test_stale_pose_is_refused(self):
        self.pose.pose["age_s"] = 5.5
        st, d = self.call("POST", "/api/pins/add", {"from": "robot"})
        self.assertEqual(st, 409)
        self.pose.pose["age_s"] = 4.9
        st, d = self.call("POST", "/api/pins/add", {"from": "robot"})
        self.assertEqual(st, 200)
        self.pose.pose["age_s"] = None  # 時刻が取れない時は従来どおり使える
        st, d = self.call("POST", "/api/pins/add", {"from": "robot"})
        self.assertEqual(st, 200)

    def test_add_validation_errors_are_400(self):
        bad = [
            {"from": {"x": float("nan"), "y": 0, "yaw_deg": 0}},
            {"from": {"x": 1e9, "y": 0, "yaw_deg": 0}},
            {"from": {"x": "1", "y": 0}},
            {"from": {"x": 0, "y": 0, "yaw_deg": float("inf")}},
            {"from": "robotx"},
            {"from": 5},
            {},
            {"from": {"x": 0, "y": 0}, "name": "あ" * 41},
            {"from": {"x": 0, "y": 0}, "name": 5},
            {"from": {"x": 0, "y": 0}, "color": "red"},
        ]
        for body in bad:
            st, d = self.call("POST", "/api/pins/add", body)
            self.assertEqual(st, 400, (body, d))
        st, d = self.call("POST", "/api/pins/add", raw=b'{"from": {"x": NaN, "y": 0}}')
        self.assertEqual(st, 400)
        self.assertEqual(self.store.list(), [])

    def test_limit_200_is_409(self):
        for i in range(200):
            self.store.add(f"p{i}", None, 0, 0, 0)
        st, d = self.call("POST", "/api/pins/add", {"from": {"x": 0, "y": 0}})
        self.assertEqual(st, 409)
        self.assertEqual(d["error"], "limit")

    def test_update_rename_recolor_and_move(self):
        pin = self.add_pin()
        st, d = self.call("POST", "/api/pins/update", {"id": pin["id"], "name": "裏口", "color": "#16a34a"})
        self.assertEqual(st, 200, d)
        self.assertEqual((d["pin"]["name"], d["pin"]["color"], d["pin"]["x"]), ("裏口", "#16a34a", 2.0))
        st, d = self.call("POST", "/api/pins/update", {"id": pin["id"], "x": 7.5, "y": 8.5, "yaw_deg": -30})
        self.assertEqual(st, 200)
        self.assertEqual((d["pin"]["x"], d["pin"]["y"], d["pin"]["yaw_deg"]), (7.5, 8.5, -30.0))
        # 現在位置に更新
        st, d = self.call("POST", "/api/pins/update", {"id": pin["id"], "from": "robot"})
        self.assertEqual(st, 200, d)
        self.assertEqual((d["pin"]["x"], d["pin"]["y"]), (0.5, 0.25))
        self.assertEqual(d["pin"]["name"], "裏口")
        self.pose.pose = None
        st, d = self.call("POST", "/api/pins/update", {"id": pin["id"], "from": "robot"})
        self.assertEqual(st, 409)
        st, d = self.call("POST", "/api/pins/update", {"id": pin["id"], "x": float("nan"), "y": 0})
        self.assertEqual(st, 400)
        st, d = self.call("POST", "/api/pins/update", {"id": "zzz", "name": "x"})
        self.assertEqual(st, 404)

    def test_delete(self):
        pin = self.add_pin()
        st, d = self.call("POST", "/api/pins/delete", {"id": pin["id"]})
        self.assertEqual(st, 200)
        st, d = self.call("POST", "/api/pins/delete", {"id": pin["id"]})
        self.assertEqual(st, 404)
        self.assertEqual(self.call("GET", "/api/pins")[1]["count"], 0)

    def test_guest_can_read_but_not_write(self):
        pin = self.add_pin()
        st, d = self.call("GET", "/api/pins", cookie=self.guest)
        self.assertEqual(st, 200)
        self.assertEqual(d["count"], 1)
        for path, body in (("/api/pins/add", {"from": "robot"}),
                           ("/api/pins/update", {"id": pin["id"], "name": "x"}),
                           ("/api/pins/delete", {"id": pin["id"]}),
                           ("/api/pins/goto", {"id": pin["id"]}),
                           ("/api/goal/cancel", {})):
            st, d = self.call("POST", path, body, cookie=self.guest)
            self.assertEqual(st, 403, path)
        self.assertEqual(len(self.store.list()), 1)
        self.assertEqual(self.goals() if "/goal_pose" in self.node.pubs else [], [])


class GotoApi(ApiBase):
    def test_goto_publishes_exact_stored_pose(self):
        # 量子化されない値(丸め誤差が出やすい値)でも、保存した姿勢が厳密にそのまま出る
        x, y, yaw = 1.2345678901234, -9.8765432109876, 123.456789
        pin = self.add_pin(**{"from": {"x": x, "y": y, "yaw_deg": yaw}})
        st, d = self.call("POST", "/api/pins/goto", {"id": pin["id"]})
        self.assertEqual(st, 200, d)
        self.assertTrue(d["ok"])
        self.assertEqual(len(self.goals()), 1)
        g = self.goals()[0]
        self.assertEqual(g.header.frame_id, "map")
        self.assertEqual((g.header.stamp.sec, g.header.stamp.nanosec), (1234, 5))
        self.assertEqual((g.pose.position.x, g.pose.position.y), (x, y))
        q = g.pose.orientation
        self.assertEqual((q.x, q.y), (0.0, 0.0))
        self.assertAlmostEqual(q.z, math.sin(math.radians(yaw) / 2), places=12)
        self.assertAlmostEqual(q.w, math.cos(math.radians(yaw) / 2), places=12)
        self.assertEqual(self.teleop.goal_mode, 1)
        self.assertEqual(d["goal"]["x"], x)
        self.assertEqual(d["goal"]["yaw_deg"], pin["yaw_deg"])

    def test_goto_is_repeatable_from_anywhere(self):
        pin = self.add_pin()
        first = None
        for rx, ry in ((0.0, 0.0), (30.0, -12.0), (-5.0, 8.0)):
            self.pose.pose.update(x=rx, y=ry)
            st, d = self.call("POST", "/api/pins/goto", {"id": pin["id"]})
            self.assertEqual(st, 200)
            g = self.goals()[-1]
            cur = (g.pose.position.x, g.pose.position.y, g.pose.orientation.z, g.pose.orientation.w)
            first = first or cur
            self.assertEqual(cur, first)  # ロボットの位置に依らず毎回同じ目的地
            self.assertAlmostEqual(d["distance_m"], math.hypot(2.0 - rx, -1.0 - ry))

    def test_goto_soft_estop_409(self):
        pin = self.add_pin()
        self.teleop.soft_estop_engaged = True
        st, d = self.call("POST", "/api/pins/goto", {"id": pin["id"]})
        self.assertEqual(st, 409)
        self.assertEqual(d["error"], "estop_engaged")
        self.assertEqual(self.goals(), [])
        self.assertEqual(self.teleop.goal_mode, 0)

    def test_goto_outside_boundary_409(self):
        pin = self.add_pin(**{"from": {"x": 8.0, "y": 8.0, "yaw_deg": 0}})
        ok, _ = self.boundary.set_boundary(-1.0, 1.0, -1.0, 1.0)
        self.assertTrue(ok)
        st, d = self.call("POST", "/api/pins/goto", {"id": pin["id"]})
        self.assertEqual(st, 409)
        self.assertEqual(d["error"], "outside_boundary")
        self.assertEqual(self.goals(), [])
        # 範囲内のピンは通る
        inside = self.add_pin(name="中", **{"from": {"x": 0.2, "y": 0.2, "yaw_deg": 0}})
        st, d = self.call("POST", "/api/pins/goto", {"id": inside["id"]})
        self.assertEqual(st, 200)

    def test_goto_pose_missing_or_stale_409(self):
        pin = self.add_pin()
        self.pose.pose = None
        st, d = self.call("POST", "/api/pins/goto", {"id": pin["id"]})
        self.assertEqual(st, 409)
        self.assertEqual(d["error"], "no_pose")
        self.pose.pose = {"x": 0, "y": 0, "yaw": 0, "age_s": 9.0}
        st, d = self.call("POST", "/api/pins/goto", {"id": pin["id"]})
        self.assertEqual(st, 409)
        self.assertEqual(self.goals(), [])

    def test_goto_unknown_pin_404_and_bad_id(self):
        for body in ({"id": "nope"}, {}, {"id": 5}, {"id": None}):
            st, d = self.call("POST", "/api/pins/goto", body)
            self.assertEqual(st, 404, body)
        self.assertEqual(self.goals() if "/goal_pose" in self.node.pubs else [], [])

    def test_goto_warns_when_pin_is_in_wall(self):
        w = h = 40
        data = [0] * (w * h)
        for r in range(h):
            data[r * w + 20] = 100
        self.map_goal.latest_map = NS(info=NS(width=w, height=h, resolution=0.05, origin=NS(position=NS(x=0.0, y=0.0))), data=data)
        pin = self.add_pin(**{"from": {"x": 1.025, "y": 1.0, "yaw_deg": 0}})  # 壁のセル
        st, d = self.call("POST", "/api/pins/goto", {"id": pin["id"]})
        self.assertEqual(st, 200)
        self.assertIn("warning", d)
        # それでも送るのは保存した姿勢そのまま(寄せない)
        self.assertEqual(self.goals()[0].pose.position.x, 1.025)

    def test_goal_status_tracks_pin_and_arrival(self):
        pin = self.add_pin(**{"from": {"x": 1.0, "y": 0.0, "yaw_deg": 0}})
        self.call("POST", "/api/pins/goto", {"id": pin["id"]})
        st, d = self.call("GET", "/api/goal/status")
        self.assertEqual(st, 200)
        self.assertTrue(d["active"])
        self.assertEqual(d["state"], "sent")
        self.assertEqual(d["goal"]["pin_name"], "入口")
        self.assertAlmostEqual(d["distance_m"], math.hypot(0.5, 0.25), places=6)
        # 別のゴールの完了通知は無視する
        cb = self.node.subs["/nokosahen/nav2_goal_status"]
        cb(NS(data=json.dumps({"event": "finished", "status": 4, "target_x": 9.0, "target_y": 9.0})))
        self.assertTrue(self.call("GET", "/api/goal/status")[1]["active"])
        # ロボットが目的地の6cm手前・向き5°ずれで到着
        self.pose.pose.update(x=0.94, y=0.0, yaw=math.radians(5))
        cb(NS(data=json.dumps({"event": "finished", "status": 4, "target_x": 1.0, "target_y": 0.0})))
        d = self.call("GET", "/api/goal/status")[1]
        self.assertFalse(d["active"])
        self.assertEqual(d["state"], "succeeded")
        self.assertAlmostEqual(d["arrival"]["dist_cm"], 6.0, places=1)
        self.assertAlmostEqual(d["arrival"]["yaw_err_deg"], 5.0, places=1)

    def test_goal_status_failed_and_cancel(self):
        self.call("POST", "/api/goal", {"x": 1.0, "y": 1.0})
        cb = self.node.subs["/nokosahen/nav2_goal_status"]
        cb(NS(data=json.dumps({"event": "rejected", "status": 0, "target_x": 1.0, "target_y": 1.0})))
        d = self.call("GET", "/api/goal/status")[1]
        self.assertEqual((d["state"], d["event"]), ("failed", "rejected"))
        # 中止: 既存の/nokosahen/cancel_goalへTrue
        self.call("POST", "/api/goal", {"x": 1.5, "y": 1.0})
        st, d = self.call("POST", "/api/goal/cancel", {})
        self.assertEqual(st, 200)
        self.assertTrue(d["was_active"])
        sent = self.node.pubs["/nokosahen/cancel_goal"].sent
        self.assertTrue(sent and all(m.data is True for m in sent))
        self.assertEqual(self.call("GET", "/api/goal/status")[1]["state"], "canceled")


class GoalApi(ApiBase):
    def grid(self):
        w = h = 60
        data = [0] * (w * h)
        for r in range(h):
            data[r * w + 30] = 100  # x=0.5付近の縦の壁(origin 0,0, res 0.05)
        self.map_goal.latest_map = NS(info=NS(width=w, height=h, resolution=0.05, origin=NS(position=NS(x=0.0, y=0.0))), data=data)

    def yaw_of(self, g):
        q = g.pose.orientation
        return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def test_plain_goal_faces_travel_direction(self):
        # 【2026-10-19】向きの指定が無いとき、従来は常にmapの0度だった(到着のたびに余計な旋回)。現在位置(0.5, 0.25)から
        # 目的地(1.25, -0.5)へ向かう方向(-45度)にする。
        st, d = self.call("POST", "/api/goal", {"x": 1.25, "y": -0.5})
        self.assertEqual(st, 200, d)
        self.assertTrue(d["ok"])
        self.assertFalse(d["snapped"])
        g = self.goals()[0]
        self.assertEqual((g.pose.position.x, g.pose.position.y), (1.25, -0.5))
        self.assertAlmostEqual(self.yaw_of(g), -45.0, places=6)
        # 「向きを指定した」扱いではない(画面に向きの矢印・向きの差を出さない)
        self.assertFalse(d["goal"]["yaw_specified"])

    def test_plain_goal_identity_without_robot_pose_or_when_too_close(self):
        self.pose.pose = None  # 現在位置が取れない → 従来どおり
        st, d = self.call("POST", "/api/goal", {"x": 1.25, "y": -0.5})
        self.assertEqual(st, 200, d)
        q = self.goals()[0].pose.orientation
        self.assertEqual((q.x, q.y, q.z, q.w), (0.0, 0.0, 0.0, 1.0))
        # 現在位置が古い(age_s > 閾値) → 従来どおり
        self.pose.pose = {"x": 0.5, "y": 0.25, "yaw": 0.0, "age_s": 99.0}
        st, d = self.call("POST", "/api/goal", {"x": 1.25, "y": -0.5})
        q = self.goals()[-1].pose.orientation
        self.assertEqual((q.x, q.y, q.z, q.w), (0.0, 0.0, 0.0, 1.0))
        # 近すぎる(0.3 m未満)目的地は、進行方向が定まらないので向きを付けない
        self.pose.pose = {"x": 1.0, "y": 1.0, "yaw": 0.0, "age_s": 0.1}
        st, d = self.call("POST", "/api/goal", {"x": 1.1, "y": 1.1})
        q = self.goals()[-1].pose.orientation
        self.assertEqual((q.x, q.y, q.z, q.w), (0.0, 0.0, 0.0, 1.0))

    def test_explicit_heading_is_not_overridden_by_travel_direction(self):
        st, d = self.call("POST", "/api/goal", {"x": 1.25, "y": -0.5, "yaw_deg": 170})
        self.assertEqual(st, 200, d)
        self.assertAlmostEqual(self.yaw_of(self.goals()[0]), 170.0, places=6)
        self.assertTrue(d["goal"]["yaw_specified"])

    def test_goal_with_heading(self):
        st, d = self.call("POST", "/api/goal", {"x": 1.0, "y": 2.0, "yaw_deg": 90})
        self.assertEqual(st, 200)
        self.assertEqual(d["goal"]["yaw_deg"], 90.0)
        q = self.goals()[0].pose.orientation
        self.assertAlmostEqual(q.z, math.sqrt(0.5), places=12)
        self.assertAlmostEqual(q.w, math.sqrt(0.5), places=12)

    def test_goal_validation(self):
        for body in ({"x": float("nan"), "y": 0}, {"x": 0, "y": float("inf")}, {"y": 1}, {"x": "a", "y": 0},
                     {"x": 0, "y": 0, "yaw_deg": float("nan")}, {"x": 0, "y": 0, "yaw_deg": "x"}, {"x": 0, "y": 0, "yaw_deg": True}):
            st, d = self.call("POST", "/api/goal", body)
            self.assertEqual(st, 400, body)
        self.assertEqual(self.goals() if "/goal_pose" in self.node.pubs else [], [])

    def test_guest_goal_403(self):
        st, d = self.call("POST", "/api/goal", {"x": 1, "y": 1}, cookie=self.guest)
        self.assertEqual(st, 403)

    def test_goal_in_wall_is_snapped_and_reported(self):
        self.grid()
        # 壁のセル(列30 -> x=1.5..1.55)のど真ん中
        st, d = self.call("POST", "/api/goal", {"x": 1.525, "y": 1.5})
        self.assertEqual(st, 200, d)
        self.assertTrue(d["snapped"])
        self.assertGreater(d["shift_m"], 0.0)
        self.assertLessEqual(d["shift_m"], 0.5)
        self.assertEqual(d["requested"], {"x": 1.525, "y": 1.5})
        g = self.goals()[0]
        self.assertEqual((g.pose.position.x, g.pose.position.y), (d["goal"]["x"], d["goal"]["y"]))
        self.assertNotEqual(g.pose.position.x, 1.525)
        self.assertIn("cm", d["message"])
        st, s = self.call("GET", "/api/goal/status")
        self.assertTrue(s["goal"]["snapped"])
        self.assertEqual(s["goal"]["requested_x"], 1.525)

    def test_goal_snap_can_be_disabled_and_no_free_is_409(self):
        self.grid()
        st, d = self.call("POST", "/api/goal", {"x": 1.525, "y": 1.5, "snap": False})
        self.assertEqual(st, 200)
        self.assertFalse(d["snapped"])
        self.assertEqual(self.goals()[0].pose.position.x, 1.525)
        w = h = 30
        self.map_goal.latest_map = NS(info=NS(width=w, height=h, resolution=0.05, origin=NS(position=NS(x=0.0, y=0.0))), data=[100] * (w * h))
        before = len(self.goals())
        st, d = self.call("POST", "/api/goal", {"x": 0.75, "y": 0.75})
        self.assertEqual(st, 409)
        self.assertEqual(d["error"], "no_free_cell")
        self.assertEqual(len(self.goals()), before)

    def test_goal_guards_soft_estop_and_boundary(self):
        self.teleop.soft_estop_engaged = True
        st, d = self.call("POST", "/api/goal", {"x": 0.1, "y": 0.1})
        self.assertEqual((st, d["error"]), (409, "estop_engaged"))
        self.teleop.soft_estop_engaged = False
        self.boundary.set_boundary(-1.0, 1.0, -1.0, 1.0)
        st, d = self.call("POST", "/api/goal", {"x": 5.0, "y": 0.0})
        self.assertEqual((st, d["error"]), (409, "outside_boundary"))
        self.assertEqual(self.goals() if "/goal_pose" in self.node.pubs else [], [])

    def test_snapped_goal_outside_boundary_is_refused(self):
        self.grid()
        self.boundary.set_boundary(0.0, 1.5, 0.0, 3.0)  # 壁(x=1.5)ぎりぎり。寄せ先が範囲外になりうる
        st, d = self.call("POST", "/api/goal", {"x": 1.45, "y": 1.5})
        # 寄せた先が範囲外なら409(範囲内の左側へ寄せられれば200)
        if st == 200:
            self.assertLessEqual(d["goal"]["x"], 1.5 + 1e-9)
        else:
            self.assertEqual(d["error"], "outside_boundary")


class NoControllerApi(unittest.TestCase):
    def test_routes_404_without_pins_controller(self):
        tmp = tempfile.TemporaryDirectory()
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, FakeTeleop(), auth_username="", enforce_single_controller=False)
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        port = server.server_address[1]
        try:
            for method, path in (("GET", "/api/pins"), ("POST", "/api/pins/add"), ("POST", "/api/pins/goto"), ("POST", "/api/goal/cancel")):
                req = urllib.request.Request(f"http://127.0.0.1:{port}{path}", data=b"{}" if method == "POST" else None, method=method,
                                             headers={"Content-Type": "application/json"})
                with self.assertRaises(urllib.error.HTTPError) as cm:
                    urllib.request.urlopen(req, timeout=10)
                self.assertEqual(cm.exception.code, 404, path)
                cm.exception.close()
        finally:
            server.shutdown()
            server.server_close()
            tmp.cleanup()


# ------------------------------------------------------------------ 地図保存/読込との連携
class LifecycleIntegration(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.store = mp.PinStore(str(self.dir / "maps"), str(self.dir / "session.json"))
        self.node = FakeNode()

    def tearDown(self):
        self.tmp.cleanup()

    def test_reset_view_clears_session_pins_but_not_saved(self):
        self.store.add("入口", None, 1, 1, 0)
        self.store.save_for_map("m1")
        life = dc.MapLifecycleController.__new__(dc.MapLifecycleController)
        life.node = self.node
        life.pin_store = self.store
        life.view_store = NS(clear_loaded_map=lambda: None, set_session_angle=lambda a: None)
        life._reset_view()
        self.assertEqual(self.store.list(), [])
        self.assertTrue((self.dir / "maps" / "m1.pins.json").exists())

    def test_save_map_message_and_files(self):
        self.store.add("入口", None, 1, 1, 0)
        maps = self.dir / "maps"
        maps.mkdir(parents=True, exist_ok=True)

        class Done:
            returncode = 0
            stdout = "ok"
            stderr = ""

        mapping = dc.MappingController.__new__(dc.MappingController)
        mapping.node = self.node
        mapping.maps_dir = maps
        mapping.pin_store = self.store
        mapping.view_store = NS(save_for_map=lambda n: 12.5)
        mapping._lock = threading.Lock()
        mapping._busy = False
        mapping._last_result = {}
        mapping._camera_obstacle_points = set()
        mapping._robot_pose = None
        with mock.patch.object(dc.subprocess, "run", return_value=Done()):
            ok, _ = mapping.save_map("shop2")
            self.assertTrue(ok)
            for _ in range(100):
                if not mapping.status()["busy"]:
                    break
                threading.Event().wait(0.05)
        res = mapping.status()["last_result"]
        self.assertTrue(res["ok"], res)
        self.assertIn("ピン1件", res["message"])
        self.assertIn("ループクロージャ", res["message"])  # 保存時の注意
        self.assertEqual(len(json.loads((maps / "shop2.pins.json").read_text(encoding="utf-8"))["pins"]), 1)


# ------------------------------------------------------------------ JS(クリック->世界座標の精度)
def run_node(body):
    src = MAP_VIEW_MATH_JS + "\n" + body
    with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False, encoding="utf-8") as f:
        f.write(src)
        path = f.name
    try:
        r = subprocess.run([NODE, path], capture_output=True, text=True, timeout=60)
    finally:
        os.unlink(path)
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


@unittest.skipIf(NODE is None, "node not installed")
class ClickPrecisionTransform(unittest.TestCase):
    def test_click_to_world_round_trip_with_rotation_and_subcell_offsets(self):
        out = run_node("""
        const W = 733, H = 517;   // 端数のあるキャンバスサイズ
        const m = {width: 238, height: 254, resolution: 0.05, origin_x: -5.13, origin_y: -7.77};
        let worst = 0, worstCell = 0;
        for (const rotDeg of [0, 7.5, 30, 90, -45, 137, 180]) {
          for (const scale of [20, 41.3, 160, 600]) {
            const view = {cx: 1.234, cy: -2.345, scale: scale, rot: rotDeg * Math.PI / 180};
            for (const [x, y] of [[0.0, 0.0], [1.2345, -2.3456], [-3.0123, 4.0987], [5.5, -6.4]]) {
              // セル内の細かい位置(0.4セル分ずつずらす)も含めて、世界->画面->世界が一致する
              for (const off of [0, 0.013, 0.027, 0.0499]) {
                const wx = x + off, wy = y - off;
                const p = mapViewWorldToPixel(view, W, H, wx, wy);
                const q = mapViewPixelToWorld(view, W, H, p[0], p[1]);
                worst = Math.max(worst, Math.hypot(q[0] - wx, q[1] - wy));
              }
            }
          }
        }
        console.log(JSON.stringify({worst}));
        """)
        self.assertLess(out["worst"], 1e-9)  # 1e-9 m = 1nm: 変換自体に量子化も反転ミスも無い

    def test_click_hits_the_cell_that_is_drawn_there(self):
        # 画面のクリック位置 -> 世界座標 -> セル(col,row) が、地図画像として描かれているセルと同じか。
        # 描画は mapImageMatrix(行0=最北)。セル(c,r)の中心(画像座標 (c+.5, H-1-r+.5))を描画行列で画面へ
        # 写した点をクリックしたら、その世界座標のセルが(c,r)であること。
        out = run_node("""
        const W = 640, H = 480;
        const m = {width: 100, height: 80, resolution: 0.05, origin_x: -2.5, origin_y: -2.0};
        let bad = 0, n = 0, worst = 0;
        for (const rotDeg of [0, 30, 90, -45, 170]) {
          const view = {cx: 0.3, cy: -0.4, scale: 90, rot: rotDeg * Math.PI / 180};
          const T = mapImageMatrix(view, W, H, m);
          for (const [c, r] of [[0, 0], [99, 79], [50, 40], [13, 61], [77, 5]]) {
            const ix = c + 0.5, iy = (m.height - 1 - r) + 0.5;          // offscreen画像のセル中心
            const px = T[0] * ix + T[2] * iy + T[4], py = T[1] * ix + T[3] * iy + T[5];
            const w = mapViewPixelToWorld(view, W, H, px, py);
            const cc = Math.floor((w[0] - m.origin_x) / m.resolution);
            const rr = Math.floor((w[1] - m.origin_y) / m.resolution);
            n++; if (cc !== c || rr !== r) bad++;
            worst = Math.max(worst, Math.hypot(w[0] - (m.origin_x + (c + 0.5) * m.resolution), w[1] - (m.origin_y + (r + 0.5) * m.resolution)));
          }
        }
        console.log(JSON.stringify({bad, n, worst}));
        """)
        self.assertEqual(out["bad"], 0)
        self.assertLess(out["worst"], 1e-9)

    def test_drag_heading_goal_matches_world_yaw(self):
        out = run_node("""
        const W = 800, H = 600, res = [];
        for (const rotDeg of [0, 30, 90, -45]) {
          const view = {cx: 2.0, cy: -1.0, scale: 50, rot: rotDeg * Math.PI / 180};
          const p0 = [300, 250];
          const p = mapPoseFromScreenDrag(view, W, H, p0, [p0[0] + 80, p0[1]]);   // 画面右へドラッグ
          const w0 = mapViewPixelToWorld(view, W, H, p0[0], p0[1]);
          res.push({rot: rotDeg, yaw: p.yaw_deg, dx: p.x - w0[0], dy: p.y - w0[1]});
        }
        console.log(JSON.stringify(res));
        """)
        for o in out:
            self.assertAlmostEqual(o["dx"], 0.0, places=9)
            self.assertAlmostEqual(o["dy"], 0.0, places=9)
            d = (o["yaw"] + o["rot"] + 180.0) % 360.0 - 180.0
            self.assertAlmostEqual(d, 0.0, places=7)  # 画面右 = 世界の -rot 方向

    def test_pin_hit_test_with_rotation(self):
        out = run_node("""
        const W = 640, H = 480;
        const pins = [{id: "a", x: 1.0, y: 1.0}, {id: "b", x: 1.1, y: 1.0}, {id: "c", x: -3, y: 2}];
        const res = [];
        for (const rotDeg of [0, 30, 90, -45]) {
          const view = {cx: 0, cy: 0, scale: 80, rot: rotDeg * Math.PI / 180};
          const pa = mapViewWorldToPixel(view, W, H, 1.0, 1.0);
          const pc = mapViewWorldToPixel(view, W, H, -3, 2);
          res.push({
            rot: rotDeg,
            onA: mapPinHit(view, W, H, pins, pa[0] + 2, pa[1] - 2, 18),
            onC: mapPinHit(view, W, H, pins, pc[0] - 5, pc[1] + 5, 18),
            far: mapPinHit(view, W, H, pins, pc[0] + 200, pc[1] + 200, 18),
            nearest: mapPinHit(view, W, H, pins, (pa[0] + mapViewWorldToPixel(view, W, H, 1.1, 1.0)[0]) / 2, pa[1], 18) !== null,
          });
        }
        console.log(JSON.stringify(res));
        """)
        for o in out:
            self.assertEqual((o["onA"], o["onC"], o["far"]), ("a", "c", None), o)
            self.assertTrue(o["nearest"])


# ------------------------------------------------------------------ nav2 params
class GoalPrecisionParams(unittest.TestCase):
    YAML = BRINGUP / "config" / "nav2_params_jetson.yaml"

    def test_jetson_yaml_has_tight_tolerances(self):
        import yaml

        d = yaml.safe_load(self.YAML.read_text(encoding="utf-8"))
        c = d["controller_server"]["ros__parameters"]
        self.assertEqual(c["general_goal_checker"]["xy_goal_tolerance"], 0.08)
        self.assertEqual(c["general_goal_checker"]["yaw_goal_tolerance"], 0.15)
        self.assertTrue(c["general_goal_checker"]["stateful"])  # 一度位置が合ったら再判定しない(振動防止)
        self.assertEqual(c["FollowPath"]["xy_goal_tolerance"], c["general_goal_checker"]["xy_goal_tolerance"])
        self.assertEqual(d["planner_server"]["ros__parameters"]["GridBased"]["tolerance"], 0.2)
        self.assertLess(d["planner_server"]["ros__parameters"]["GridBased"]["tolerance"], 0.5)
        # 振動対策の批評(減速して寄せる)は残っている
        self.assertIn("RotateToGoal", c["FollowPath"]["critics"])
        self.assertIn("GoalAlign", c["FollowPath"]["critics"])
        self.assertEqual(c["FollowPath"]["RotateToGoal.slowing_factor"], 5.0)

    def test_block_matches_yaml_and_applies_to_any_params_file(self):
        import yaml

        for name in ("nav2_params_jetson.yaml", "nav2_params_raspi4.yaml"):
            d = yaml.safe_load((BRINGUP / "config" / name).read_text(encoding="utf-8"))
            out = ns.apply_goal_precision_to_params(d)
            c = out["controller_server"]["ros__parameters"]
            self.assertEqual(c["general_goal_checker"]["xy_goal_tolerance"], 0.08, name)
            self.assertEqual(c["general_goal_checker"]["yaw_goal_tolerance"], 0.15, name)
            self.assertEqual(c["FollowPath"]["xy_goal_tolerance"], 0.08, name)
            self.assertEqual(out["planner_server"]["ros__parameters"]["GridBased"]["tolerance"], 0.2, name)
            self.assertEqual(c["progress_checker"]["movement_time_allowance"], 6.0, name)
        # 入力は変更しない
        raspi = yaml.safe_load((BRINGUP / "config" / "nav2_params_raspi4.yaml").read_text(encoding="utf-8"))
        before = json.dumps(raspi, sort_keys=True)
        ns.apply_goal_precision_to_params(raspi)
        self.assertEqual(json.dumps(raspi, sort_keys=True), before)
        self.assertEqual(raspi["controller_server"]["ros__parameters"]["general_goal_checker"]["xy_goal_tolerance"], 0.18)

    def test_speed_semantics_unchanged(self):
        import yaml

        d = yaml.safe_load(self.YAML.read_text(encoding="utf-8"))
        a = ns.apply_nav_speed_to_params(d, 0.5)
        b = ns.apply_goal_precision_to_params(a)
        # 速度の変換は目的地精度のブロックを触らず、逆に精度ブロックも速度の値を触らない
        for node in (ns.NODE_CONTROLLER, ns.NODE_SMOOTHER, ns.NODE_COLLISION):
            for k, v in ns.live_parameters(0.5)[node].items():
                cur = b[node]["ros__parameters"]
                for part in k.split("."):
                    cur = cur[part]
                self.assertEqual(cur, v, k)
        live = ns.live_parameters(0.5)
        flat = {k for vals in live.values() for k in vals}
        self.assertFalse(any("goal_tolerance" in k or k.endswith("tolerance") for k in flat))  # 実行時の速度変更は精度に触れない
        for node, vals in ns.GOAL_PRECISION.items():
            self.assertFalse(set(vals) & set(live.get(node, {})))

    def test_write_rewritten_params_includes_goal_precision(self):
        import yaml

        with tempfile.TemporaryDirectory() as tmp:
            out = ns.write_rewritten_params(str(self.YAML), 0.5, os.path.join(tmp, "n.yaml"))
            d = yaml.safe_load(Path(out).read_text(encoding="utf-8"))
            self.assertEqual(d["controller_server"]["ros__parameters"]["general_goal_checker"]["xy_goal_tolerance"], 0.08)
            out2 = ns.write_rewritten_params(str(self.YAML), 0.5, os.path.join(tmp, "n2.yaml"), goal_precision=False)
            d2 = yaml.safe_load(Path(out2).read_text(encoding="utf-8"))
            self.assertEqual(d2["planner_server"]["ros__parameters"]["GridBased"]["tolerance"], 0.2)  # yamlの値(参考)


if __name__ == "__main__":
    unittest.main()
