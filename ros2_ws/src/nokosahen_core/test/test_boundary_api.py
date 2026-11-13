"""走行可能範囲(回転矩形)と地図の表示回転のHTTP API/コントローラの結合テスト。

ROS 2が無いPCでも動くよう、rclpy系のモジュールをスタブしてdashboard_commonを読み込み、
実際にHTTPサーバーを立てて /api/boundary(/set,/clear), /api/goal, /api/map_view* を叩く。
NaN/不正入力は400、範囲外の目的地は409、範囲未設定は従来どおり制限なし、を確認する。
ロボットには何も送らない(publisherもTeleop/MapGoalもスタブ)。

実行: python ros2_ws/src/nokosahen_core/test/test_boundary_api.py
"""

import json
import math
import sys
import tempfile
import threading
import types
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

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
    mod("std_msgs.msg", Bool=Msg, Float32MultiArray=Msg, String=Msg)


try:
    import geometry_msgs.msg  # noqa: F401
    import nav_msgs.msg  # noqa: F401  (他テストが部分スタブを入れていても足りない分を補う)
except Exception:  # noqa: BLE001
    _install_stubs()

from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import map_orientation as mo  # noqa: E402


class FakeLogger:
    def warn(self, *a, **k):
        pass

    info = error = debug = warn


class FakePub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(json.loads(msg.data))


class FakeNode:
    def __init__(self):
        self.pubs = {}

    def create_publisher(self, typ, topic, depth):
        self.pubs[topic] = FakePub()
        return self.pubs[topic]

    def create_subscription(self, *a, **k):
        return None

    def get_logger(self):
        return FakeLogger()


class FakeTeleop:
    soft_estop_engaged = False

    def __init__(self):
        self.goal_mode = 0

    def activate_goal_mode(self):
        self.goal_mode += 1


class FakeInfo:
    def __init__(self, w, h):
        self.width, self.height, self.resolution = w, h, 0.05


class FakeMap:
    def __init__(self, w, h, data):
        self.info = FakeInfo(w, h)
        self.data = data


class FakeMapGoal:
    def __init__(self):
        self.goals = []
        self.latest_map = None

    def send_goal(self, x, y, yaw_deg=None, meta=None):
        self.goals.append((x, y))

    def check_goal(self, x, y, wall_margin=True, rect=None):  # 2026-10-07: 目的地のスナップ(地図なし=そのまま)
        return {"status": "no_map", "x": x, "y": y, "shift_m": 0.0, "reason": ""}


class Api(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        tmp = Path(self.tmp.name)
        self.node = FakeNode()
        self.boundary = dc.BoundaryController(self.node, str(tmp / "boundary.json"))
        self.teleop = FakeTeleop()
        self.map_goal = FakeMapGoal()
        self.mapping = dc.MappingController.__new__(dc.MappingController)
        self.mapping.view_store = mo.MapViewStore(str(tmp / "maps"), str(tmp / "view.json"))
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>",
            lambda: {},
            self.teleop,
            map_goal=self.map_goal,
            mapping=self.mapping,
            boundary=self.boundary,
            auth_username="",
            enforce_single_controller=False,
        )
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        self.port = self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.tmp.cleanup()

    def call(self, method, path, body=None, raw=None):
        data = raw if raw is not None else (None if body is None else json.dumps(body).encode())
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.port}{path}", data=data, method=method,
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(req, timeout=10) as r:
                return r.status, json.loads(r.read().decode("utf-8"))
        except urllib.error.HTTPError as e:
            try:
                return e.code, json.loads(e.read().decode("utf-8"))
            finally:
                e.close()

    # ---------------------------------------------------------------- boundary
    def test_absent_boundary_means_no_restriction(self):
        st, b = self.call("GET", "/api/boundary")
        self.assertEqual(st, 200)
        self.assertFalse(b["enabled"])
        st, _ = self.call("POST", "/api/goal", {"x": 1e3, "y": -1e3})
        self.assertEqual(st, 200)
        self.assertEqual(self.map_goal.goals, [(1e3, -1e3)])

    def test_old_form_post_becomes_yaw0_rect_and_get_returns_both_forms(self):
        st, d = self.call("POST", "/api/boundary/set", {"min_x": -1, "max_x": 3, "min_y": 0, "max_y": 2})
        self.assertEqual(st, 200, d)
        st, b = self.call("GET", "/api/boundary")
        self.assertTrue(b["enabled"])
        self.assertEqual((b["min_x"], b["max_x"], b["min_y"], b["max_y"]), (-1, 3, 0, 2))
        self.assertEqual((b["center_x"], b["center_y"], b["width"], b["height"], b["yaw"]), (1, 1, 4, 2, 0))
        # トピックにも新旧両方が流れる(旧消費者はmin/maxだけで動く)
        sent = self.node.pubs["/nokosahen/travel_boundary"].sent[-1]
        self.assertTrue(all(k in sent for k in ("min_x", "max_x", "min_y", "max_y", "center_x", "yaw")))

    def test_rotated_post_goal_enforced_with_true_rect_not_bbox(self):
        st, d = self.call(
            "POST", "/api/boundary/set",
            {"center_x": 0, "center_y": 0, "width": 6, "height": 2, "yaw_deg": 30},
        )
        self.assertEqual(st, 200, d)
        _, b = self.call("GET", "/api/boundary")
        self.assertAlmostEqual(b["yaw_deg"], 30.0)
        # bboxは回転矩形の外接
        self.assertAlmostEqual(b["max_x"], 3 * math.cos(math.radians(30)) + 1 * math.sin(math.radians(30)))
        # 矩形の内側(長辺方向 2.5m)は通る
        c, s = math.cos(math.radians(30)), math.sin(math.radians(30))
        st, _ = self.call("POST", "/api/goal", {"x": 2.5 * c, "y": 2.5 * s})
        self.assertEqual(st, 200)
        # bboxの隅(軸平行なら範囲内)は回転矩形の外 -> 拒否
        st, d = self.call("POST", "/api/goal", {"x": b["max_x"] - 0.01, "y": b["max_y"] - 0.01})
        self.assertEqual(st, 409)
        self.assertEqual(d["error"], "outside_boundary")
        self.assertIn("範囲", d["message"])
        self.assertEqual(len(self.map_goal.goals), 1)  # 拒否された目的地は送られていない
        # 解除すれば再び制限なし
        self.call("POST", "/api/boundary/clear", {})
        st, _ = self.call("POST", "/api/goal", {"x": b["max_x"] - 0.01, "y": b["max_y"] - 0.01})
        self.assertEqual(st, 200)

    def test_nan_and_malformed_rejected_with_400_and_state_unchanged(self):
        self.call("POST", "/api/boundary/set", {"center_x": 0, "center_y": 0, "width": 2, "height": 2})
        before = self.call("GET", "/api/boundary")[1]
        bad_bodies = [
            b'{"center_x": NaN, "center_y": 0, "width": 2, "height": 2}',
            b'{"center_x": 0, "center_y": 0, "width": Infinity, "height": 2}',
            b'{"center_x": 0, "center_y": 0, "width": 2, "height": 2, "yaw": NaN}',
            b'{"min_x": NaN, "max_x": 1, "min_y": 0, "max_y": 1}',
            b'{"min_x": 0, "max_x": 1}',
            b'{"center_x": "abc", "center_y": 0, "width": 2, "height": 2}',
            b'{}',
            b'not json',
        ]
        for raw in bad_bodies:
            st, d = self.call("POST", "/api/boundary/set", raw=raw)
            self.assertEqual(st, 400, (raw, d))
        for body in ({"center_x": 0, "center_y": 0, "width": 0, "height": 1},
                     {"min_x": 2, "max_x": 1, "min_y": 0, "max_y": 1}):
            st, d = self.call("POST", "/api/boundary/set", body)
            self.assertEqual(st, 409, (body, d))
        self.assertEqual(self.call("GET", "/api/boundary")[1], before)

    def test_goal_nan_still_400(self):
        st, _ = self.call("POST", "/api/goal", raw=b'{"x": NaN, "y": 1}')
        self.assertEqual(st, 400)

    def test_persisted_old_file_loads_as_yaw0(self):
        tmp = Path(self.tmp.name)
        (tmp / "old.json").write_text(
            json.dumps({"enabled": True, "min_x": 0.0, "max_x": 4.0, "min_y": -1.0, "max_y": 1.0}),
            encoding="utf-8",
        )
        bc = dc.BoundaryController(FakeNode(), str(tmp / "old.json"))
        st = bc.status()
        self.assertTrue(st["enabled"])
        self.assertEqual((st["center_x"], st["width"], st["height"], st["yaw"]), (2.0, 4.0, 2.0, 0.0))
        self.assertTrue(bc.allows_point(3.9, 0.9))
        self.assertFalse(bc.allows_point(4.1, 0.0))
        # 再保存/再読込で回転矩形が保たれる
        bc.set_from_request({"center_x": 1, "center_y": 1, "width": 3, "height": 1, "yaw_deg": -45})
        bc2 = dc.BoundaryController(FakeNode(), str(tmp / "old.json"))
        self.assertAlmostEqual(bc2.status()["yaw_deg"], -45.0)

    # ---------------------------------------------------------------- map_view
    def test_map_view_set_get_and_validation(self):
        st, d = self.call("GET", "/api/map_view")
        self.assertEqual((st, d["angle_deg"]), (200, 0.0))
        st, d = self.call("POST", "/api/map_view/set", {"angle_deg": 30.5})
        self.assertEqual((st, d["angle_deg"]), (200, 30.5))
        self.assertEqual(self.call("GET", "/api/map_view")[1]["angle_deg"], 30.5)
        st, d = self.call("POST", "/api/map_view/set", {"angle_deg": 270})
        self.assertEqual(d["angle_deg"], -90.0)
        for raw in (b'{"angle_deg": NaN}', b'{"angle_deg": "x"}', b'{}', b'{"wall_points": [1,2]}'):
            st, d = self.call("POST", "/api/map_view/set", raw=raw)
            self.assertEqual(st, 400, raw)
        # 2点指定(世界座標): 20度の壁 -> -20度
        p = (math.cos(math.radians(20)), math.sin(math.radians(20)))
        st, d = self.call("POST", "/api/map_view/set", {"wall_points": [0, 0, p[0], p[1]]})
        self.assertEqual(st, 200)
        self.assertAlmostEqual(d["angle_deg"], -20.0)

    def test_map_view_estimate_and_save(self):
        # 30度傾いた部屋の占有グリッドから壁方向を推定し、水平になる角度を返す
        sys.path.insert(0, str(PKG / "test"))
        from test_map_orientation import WallAngle
        W, H, cells = WallAngle.make_room(30.0)
        self.map_goal.latest_map = FakeMap(W, H, cells)
        st, d = self.call("POST", "/api/map_view/estimate", {"apply": True})
        self.assertEqual(st, 200, d)
        self.assertTrue(d["applied"])
        self.assertLess(abs(d["angle_deg"] - (-30.0)), 2.0)
        self.assertAlmostEqual(self.call("GET", "/api/map_view")[1]["angle_deg"], d["angle_deg"])
        # 保存: 地図名が必要。保存すると<name>.view.jsonができ、読み込みで復元される
        st, d2 = self.call("POST", "/api/map_view/save", {})
        self.assertEqual(st, 409)
        st, d2 = self.call("POST", "/api/map_view/save", {"name": "room_a"})
        self.assertEqual(st, 200, d2)
        vs = self.mapping.view_store
        self.assertTrue((vs.maps_dir / "room_a.view.json").exists())
        self.call("POST", "/api/map_view/set", {"angle_deg": 5})
        self.assertAlmostEqual(vs.on_map_loaded("room_a"), d["angle_deg"], places=6)
        self.assertAlmostEqual(self.call("GET", "/api/map_view")[1]["angle_deg"], d["angle_deg"], places=6)
        # 不正な名前(パストラバーサル)
        st, _ = self.call("POST", "/api/map_view/save", {"name": "../x"})
        self.assertEqual(st, 409)

    def test_estimate_without_map_or_walls(self):
        st, d = self.call("POST", "/api/map_view/estimate", {})
        self.assertEqual(st, 409)
        self.map_goal.latest_map = FakeMap(50, 50, [-1] * 2500)
        st, d = self.call("POST", "/api/map_view/estimate", {})
        self.assertEqual(st, 200)
        self.assertFalse(d["ok"])


if __name__ == "__main__":
    unittest.main()
