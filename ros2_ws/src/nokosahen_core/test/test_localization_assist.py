"""「ロボットに現在位置を教える」機能(2026-10-07)のテスト。

- 純粋関数: 位置の確かさの分類、共分散・クォータニオン、リクエスト検証、地図範囲判定
- JS(nodeがあれば): 画面ドラッグ->世界座標の位置+向き(表示回転 0/30/90/-45)と往復
- HTTP(ROSはスタブ): /api/initial_pose, /api/global_localization, /api/localization
  (有効 / NaN / 巨大値 / guest 403 / localization未起動の案内 / 送ったメッセージの中身)
ロボットには何も送らない(publisherもサービスクライアントもスタブ)。

実行: python ros2_ws/src/nokosahen_core/test/test_localization_assist.py
"""

import enum
import json
import math
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
    import nav_msgs.msg  # noqa: F401
except Exception:  # noqa: BLE001
    _install_stubs()

from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import localization_assist as la  # noqa: E402
from nokosahen_core.map_view import MAP_VIEW_MATH_JS  # noqa: E402

NODE = shutil.which("node")


# --------------------------------------------------------------------- 純粋関数
def cov(vx=0.0, vy=0.0, vyaw=0.0):
    c = [0.0] * 36
    c[0], c[7], c[35] = vx, vy, vyaw
    return c


class QualityTests(unittest.TestCase):
    def test_waiting_when_no_amcl_pose(self):
        q = la.classify_localization_quality(None)
        self.assertEqual((q["state"], q["label"]), ("waiting", "位置合わせ待ち"))

    def test_good_check_unknown_thresholds(self):
        # sqrt(0.02+0.02)=0.2m, yaw std 0.1 -> 良好
        q = la.classify_localization_quality(cov(0.02, 0.02, 0.01))
        self.assertEqual((q["state"], q["label"]), ("good", "良好"))
        self.assertAlmostEqual(q["pos_std_m"], 0.2)
        self.assertAlmostEqual(q["yaw_std_rad"], 0.1)
        # 位置 0.5m -> 要確認
        self.assertEqual(la.classify_localization_quality(cov(0.125, 0.125, 0.01))["state"], "check")
        # 位置は良好でも向きが 0.3rad -> 要確認
        self.assertEqual(la.classify_localization_quality(cov(0.01, 0.01, 0.09))["state"], "check")
        # 位置 1.2m -> 未確定
        self.assertEqual(la.classify_localization_quality(cov(0.72, 0.72, 0.01))["state"], "unknown")
        # 向きが大きくばらつく(全方向) -> 未確定
        self.assertEqual(la.classify_localization_quality(cov(0.01, 0.01, 3.0))["state"], "unknown")

    def test_boundaries_are_strict(self):
        self.assertEqual(la.classify_localization_quality(cov(0.044, 0.044, 0.0))["state"], "good")   # 0.3m未満
        self.assertEqual(la.classify_localization_quality(cov(0.044, 0.044, 0.0401))["state"], "check")  # yaw>=0.2
        self.assertEqual(la.classify_localization_quality(cov(0.5, 0.5, 0.0))["state"], "unknown")    # ちょうど1.0m

    def test_nan_inf_and_malformed_are_unknown_not_good(self):
        for c in (cov(float("nan"), 0, 0), cov(0, float("inf"), 0), [1.0] * 3, "xx"):
            self.assertEqual(la.classify_localization_quality(c)["state"], "unknown", c)

    def test_negative_variance_does_not_crash(self):
        self.assertEqual(la.classify_localization_quality(cov(-1e-9, -1e-9, -1e-9))["state"], "good")


class MathTests(unittest.TestCase):
    def test_initial_covariance_is_rviz_default(self):
        c = la.initial_pose_covariance()
        self.assertEqual(len(c), 36)
        self.assertEqual((c[0], c[7]), (0.25, 0.25))
        self.assertAlmostEqual(c[35], 0.0685, places=3)
        self.assertEqual(sum(1 for v in c if v != 0.0), 3)

    def test_yaw_quaternion_round_trip(self):
        for deg in (0, 30, 90, -45, 179, -179, 180):
            q = la.yaw_to_quaternion(math.radians(deg))
            self.assertAlmostEqual(math.sqrt(sum(v * v for v in q)), 1.0)
            self.assertEqual((q[0], q[1]), (0.0, 0.0))
            back = math.degrees(la.quaternion_to_yaw(*q))
            self.assertAlmostEqual(math.cos(math.radians(back - deg)), 1.0, places=9)
        self.assertAlmostEqual(la.yaw_to_quaternion(math.pi / 2)[2], math.sqrt(0.5))

    def test_parse_valid_and_normalizes_yaw(self):
        x, y, yaw = la.parse_initial_pose_request({"x": 1.5, "y": -2, "yaw_deg": 270})
        self.assertEqual((x, y), (1.5, -2.0))
        self.assertAlmostEqual(math.degrees(yaw), -90.0)
        self.assertAlmostEqual(math.degrees(la.parse_initial_pose_request({"x": 0, "y": 0, "yaw_deg": -180})[2]), 180.0)

    def test_parse_rejects_bad(self):
        bad = [
            None, [], "x", {}, {"x": 1, "y": 2},
            {"x": float("nan"), "y": 0, "yaw_deg": 0}, {"x": 0, "y": float("inf"), "yaw_deg": 0},
            {"x": 0, "y": 0, "yaw_deg": float("nan")},
            {"x": 1000, "y": 0, "yaw_deg": 0}, {"x": 0, "y": -1e9, "yaw_deg": 0}, {"x": 999.9, "y": 1000, "yaw_deg": 0},
            {"x": "1", "y": 0, "yaw_deg": 0}, {"x": True, "y": 0, "yaw_deg": 0}, {"x": None, "y": 0, "yaw_deg": 0},
            {"x": 0, "y": 0, "yaw_deg": 1e9},
        ]
        for b in bad:
            with self.assertRaises(la.PoseRequestError, msg=repr(b)):
                la.parse_initial_pose_request(b)
        la.parse_initial_pose_request({"x": 999.9, "y": -999.9, "yaw_deg": 0})

    def test_point_in_map_bounds(self):
        info = NS(resolution=0.05, width=100, height=80, origin=NS(position=NS(x=-1.0, y=-2.0)))
        self.assertTrue(la.point_in_map_bounds(info, 0.0, 0.0))
        self.assertTrue(la.point_in_map_bounds(info, -1.0, -2.0))
        self.assertFalse(la.point_in_map_bounds(info, 4.1, 0.0))
        self.assertFalse(la.point_in_map_bounds(info, 0.0, -2.1))
        self.assertIsNone(la.point_in_map_bounds(None, 0.0, 0.0))
        self.assertIsNone(la.point_in_map_bounds(NS(resolution=0, width=1, height=1, origin=NS(position=NS(x=0, y=0))), 0, 0))


# ------------------------------------------------------------- 画面->世界座標(JS)
def run_node(body):
    src = MAP_VIEW_MATH_JS + "\n" + body
    with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False, encoding="utf-8") as f:
        f.write(src)
        path = f.name
    r = subprocess.run([NODE, path], capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


@unittest.skipIf(NODE is None, "node not installed")
class PoseTransformTests(unittest.TestCase):
    def test_drag_gives_world_pose_with_rotation(self):
        out = run_node("""
        const W = 800, H = 600, res = [];
        for (const rotDeg of [0, 30, 90, -45]) {
          const view = {cx: 2.0, cy: -1.0, scale: 50, rot: rotDeg * Math.PI / 180};
          const c = [W / 2, H / 2];
          // 画面右へ / 画面上へ(canvasはy下向き) / 画面左へ / 画面下へ ドラッグ
          const dirs = {right: [100, 0], up: [0, -100], left: [-100, 0], down: [0, 100]};
          const o = {rot: rotDeg};
          for (const k in dirs) {
            const p = mapPoseFromScreenDrag(view, W, H, c, [c[0] + dirs[k][0], c[1] + dirs[k][1]]);
            o[k] = p.yaw_deg; o.x = p.x; o.y = p.y;
          }
          res.push(o);
        }
        console.log(JSON.stringify(res));
        """)
        for o in out:
            r = o["rot"]
            # 画面の中心はビューの中心(cx,cy)=(2,-1)
            self.assertAlmostEqual(o["x"], 2.0, places=9)
            self.assertAlmostEqual(o["y"], -1.0, places=9)
            # 画面右の世界yawは-rot、画面上は90-rot(表示がrot回っていると世界の向きは逆に回る)
            for key, expect in (("right", -r), ("up", 90 - r), ("left", 180 - r), ("down", -90 - r)):
                d = (o[key] - expect + 180.0) % 360.0 - 180.0
                self.assertAlmostEqual(d, 0.0, places=7, msg=f"rot={r} {key}: {o[key]} vs {expect}")
        # 具体値(回転0: 右=0°、上=+90°(世界yは上向き)、回転90: 右=-90°)
        by = {o["rot"]: o for o in out}
        self.assertAlmostEqual(by[0]["right"], 0.0)
        self.assertAlmostEqual(by[0]["up"], 90.0)
        self.assertAlmostEqual(by[90]["right"], -90.0)
        self.assertAlmostEqual(by[30]["right"], -30.0)
        self.assertAlmostEqual(by[-45]["right"], 45.0)
        self.assertTrue(all(-180.0 < o[k] <= 180.0 for o in out for k in ("right", "up", "left", "down")))

    def test_round_trip_world_pose_screen_world_pose(self):
        out = run_node("""
        const W = 640, H = 480, res = [];
        for (const rotDeg of [0, 30, 90, -45, 170]) {
          const view = {cx: -3.3, cy: 5.1, scale: 37, rot: rotDeg * Math.PI / 180};
          for (const pose of [{x: 1.5, y: -2.25, yaw: 0.7}, {x: -4, y: 6.2, yaw: -2.9}, {x: -3.3, y: 5.1, yaw: 3.0}]) {
            const p0 = mapViewWorldToPixel(view, W, H, pose.x, pose.y);
            const ang = mapPoseScreenAngle(view, pose.yaw);          // 画面上で矢印が指す角度
            const p1 = [p0[0] + 80 * Math.cos(ang), p0[1] + 80 * Math.sin(ang)];
            const back = mapPoseFromScreenDrag(view, W, H, p0, p1);
            let dy = back.yaw_deg - pose.yaw * 180 / Math.PI;
            dy = ((dy + 180) % 360 + 360) % 360 - 180;
            res.push({rot: rotDeg, dx: back.x - pose.x, dy: back.y - pose.y, dyaw: dy});
          }
        }
        console.log(JSON.stringify(res));
        """)
        self.assertEqual(len(out), 15)
        for o in out:
            self.assertAlmostEqual(o["dx"], 0.0, places=9)
            self.assertAlmostEqual(o["dy"], 0.0, places=9)
            self.assertAlmostEqual(o["dyaw"], 0.0, places=7)

    def test_world_points_and_y_flip(self):
        out = run_node("""
        console.log(JSON.stringify([
          mapPoseFromWorldPoints([1, 1], [2, 1]), mapPoseFromWorldPoints([1, 1], [1, 2]),
          mapPoseFromWorldPoints([1, 1], [0, 1]), mapPoseFromWorldPoints([1, 1], [1, 0]),
        ]));
        """)
        self.assertEqual([round(p["yaw_deg"], 6) for p in out], [0.0, 90.0, 180.0, -90.0])


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


class FakeFuture:
    def __init__(self, exc=None):
        self._exc = exc

    def add_done_callback(self, cb):
        cb(self)

    def exception(self):
        return self._exc


class FakeClient:
    def __init__(self, present=True, exc=None):
        self.present = present
        self.exc = exc
        self.calls = 0
        self.timeouts = []

    def wait_for_service(self, timeout_sec=None):
        self.timeouts.append(timeout_sec)
        return self.present

    def call_async(self, req):
        self.calls += 1
        return FakeFuture(self.exc)


class FakeEmptySrv:
    class Request:
        pass


class FakePWC:
    def __init__(self):
        self.header = NS(frame_id="", stamp=None)
        self.pose = NS(
            pose=NS(position=NS(x=0.0, y=0.0, z=0.0), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)),
            covariance=[0.0] * 36,
        )


class FakeNode:
    def __init__(self):
        self.pubs = {}
        self.subs = []
        self.client = FakeClient()

    def create_publisher(self, typ, topic, depth):
        self.pubs[topic] = FakePub()
        return self.pubs[topic]

    def create_subscription(self, typ, topic, cb, qos):
        sub = NS(typ=typ, topic=topic, cb=cb, qos=qos)
        self.subs.append(sub)
        return sub

    def destroy_subscription(self, sub):
        self.subs.remove(sub)

    def create_timer(self, period, cb):
        return None

    def create_client(self, typ, name):
        self.client_name = name
        return self.client

    def get_clock(self):
        return FakeClock()

    def get_logger(self):
        return FakeLogger()


class FakeLifecycle:
    def __init__(self):
        self.st = {"mode": "localization", "loaded_map": "lab", "running": True, "last_error": ""}

    def status(self):
        return dict(self.st)


class FakeRobotPose:
    def __init__(self):
        self.pose = None

    def get_pose(self):
        return self.pose


class FakeMapGoal:
    latest_map = None


class FakeTeleop:
    soft_estop_engaged = False

    def __init__(self):
        self.goal_mode = 0

    def activate_goal_mode(self):
        self.goal_mode += 1


class _FakeRel(enum.Enum):
    RELIABLE = 1
    BEST_EFFORT = 2


class _FakeDur(enum.Enum):
    TRANSIENT_LOCAL = 1
    VOLATILE = 2


class _FakeQoSProfile:
    def __init__(self, depth=10, reliability=None, durability=None):
        self.depth, self.reliability, self.durability = depth, reliability, durability


class ApiBase(unittest.TestCase):
    def setUp(self):
        # 他のテストが入れたスタブ/実rclpyの有無に関わらず動くよう、使う型を差し替える
        self._saved = (dc.PoseWithCovarianceStamped, dc.EmptySrv)
        dc.PoseWithCovarianceStamped = FakePWC
        dc.EmptySrv = FakeEmptySrv
        self._saved_modules = {k: sys.modules.get(k) for k in ("rclpy", "rclpy.qos")}
        try:
            import rclpy.qos  # noqa: F401
        except Exception:  # noqa: BLE001
            sys.modules["rclpy"] = types.ModuleType("rclpy")
            sys.modules["rclpy.qos"] = types.ModuleType("rclpy.qos")
            sys.modules["rclpy.qos"].QoSProfile = _FakeQoSProfile
            sys.modules["rclpy.qos"].QoSReliabilityPolicy = _FakeRel
            sys.modules["rclpy.qos"].QoSDurabilityPolicy = _FakeDur
        self.node = FakeNode()
        self.life = FakeLifecycle()
        self.robot_pose = FakeRobotPose()
        self.map_goal = FakeMapGoal()
        self.ctrl = dc.LocalizationAssistController(
            self.node, self.life, map_goal=self.map_goal, robot_pose=self.robot_pose,
            repeat_count=3, repeat_interval_sec=0.01, service_timeout_sec=0.2,
        )
        self.teleop = FakeTeleop()
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, self.teleop,
            localization=self.ctrl, auth_username="TL", auth_password="pw",
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
        dc.PoseWithCovarianceStamped, dc.EmptySrv = self._saved
        for k, v in self._saved_modules.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v

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

    def wait_published(self):
        th = self.ctrl._publish_thread  # noqa: SLF001
        if th is not None:
            th.join(5)


class InitialPoseApi(ApiBase):
    def test_valid_pose_published_with_frame_covariance_and_quaternion(self):
        st, d = self.call("POST", "/api/initial_pose", {"x": 1.25, "y": -0.5, "yaw_deg": 90})
        self.assertEqual(st, 200, d)
        self.assertTrue(d["ok"])
        self.assertNotIn("warning", d)
        self.wait_published()
        sent = self.node.pubs["/initialpose"].sent
        self.assertEqual(len(sent), 3)  # 3回(0.3秒おき・テストでは短縮)
        m = sent[0]
        self.assertEqual(m.header.frame_id, "map")
        self.assertEqual((m.header.stamp.sec, m.header.stamp.nanosec), (1234, 5))
        self.assertEqual((m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z), (1.25, -0.5, 0.0))
        o = m.pose.pose.orientation
        self.assertAlmostEqual(o.z, math.sin(math.pi / 4))
        self.assertAlmostEqual(o.w, math.cos(math.pi / 4))
        self.assertEqual((o.x, o.y), (0.0, 0.0))
        c = m.pose.covariance
        self.assertEqual(len(c), 36)
        self.assertEqual((c[0], c[7]), (0.25, 0.25))
        self.assertAlmostEqual(c[35], 0.0685, places=3)
        self.assertEqual(sum(1 for v in c if v != 0.0), 3)
        # ロボットは動かさない: 走行モードにもしない・他のトピックにも出さない
        self.assertEqual(self.teleop.goal_mode, 0)
        self.assertEqual(set(self.node.pubs), {"/initialpose"})

    def test_negative_yaw_and_wraparound(self):
        self.call("POST", "/api/initial_pose", {"x": 0, "y": 0, "yaw_deg": -270})  # = +90
        self.wait_published()
        o = self.node.pubs["/initialpose"].sent[-1].pose.pose.orientation
        self.assertAlmostEqual(o.z, math.sin(math.pi / 4))

    def test_nan_and_huge_and_malformed_are_400_and_publish_nothing(self):
        bodies = [
            b'{"x": NaN, "y": 0, "yaw_deg": 0}', b'{"x": 0, "y": Infinity, "yaw_deg": 0}',
            b'{"x": 0, "y": 0, "yaw_deg": NaN}', b'{"x": 1e9, "y": 0, "yaw_deg": 0}',
            b'{"x": 1000, "y": 0, "yaw_deg": 0}', b'{"x": 0, "y": 0}', b'{}',
            b'{"x": "a", "y": 0, "yaw_deg": 0}', b'[1,2,3]', b'not json',
        ]
        for raw in bodies:
            st, d = self.call("POST", "/api/initial_pose", raw=raw)
            self.assertEqual(st, 400, (raw, d))
            self.assertFalse(d["ok"])
            self.assertTrue(d["message"])
        self.wait_published()
        self.assertEqual(self.node.pubs["/initialpose"].sent, [])

    def test_guest_is_403_and_publishes_nothing(self):
        st, d = self.call("POST", "/api/initial_pose", {"x": 0, "y": 0, "yaw_deg": 0}, cookie=self.guest)
        self.assertEqual(st, 403)
        st, d = self.call("POST", "/api/global_localization", {}, cookie=self.guest)
        self.assertEqual(st, 403)
        self.assertEqual(self.node.pubs["/initialpose"].sent, [])
        self.assertEqual(self.node.client.calls, 0)
        # 閲覧(位置の確かさ)はguestでも可
        self.assertEqual(self.call("GET", "/api/localization", cookie=self.guest)[0], 200)

    def test_requires_login(self):
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.port}/api/initial_pose", data=b"{}", method="POST",
            headers={"Content-Type": "application/json"},
        )
        with self.assertRaises(urllib.error.HTTPError) as cm:
            urllib.request.urlopen(req, timeout=10)
        self.assertEqual(cm.exception.code, 401)
        cm.exception.close()

    def test_not_in_localization_mode_is_rejected_with_message(self):
        for st_ in ({"mode": "idle", "running": False}, {"mode": "mapping", "running": True},
                    {"mode": "localization", "running": False}):
            self.life.st.update(st_)
            st, d = self.call("POST", "/api/initial_pose", {"x": 0, "y": 0, "yaw_deg": 0})
            self.assertEqual(st, 409, st_)
            self.assertEqual(d["error"], "not_localization")
            self.assertIn("地図を読み込んで", d["message"])
            self.assertIn("localizationモード", d["message"])
            st, d = self.call("POST", "/api/global_localization", {})
            self.assertEqual(st, 409)
        self.wait_published()
        self.assertEqual(self.node.pubs["/initialpose"].sent, [])
        self.assertEqual(self.node.client.calls, 0)

    def test_outside_map_bounds_warns_but_still_sends(self):
        self.map_goal.latest_map = NS(info=NS(resolution=0.05, width=100, height=100,
                                              origin=NS(position=NS(x=-1.0, y=-1.0))))
        st, d = self.call("POST", "/api/initial_pose", {"x": 0.0, "y": 0.0, "yaw_deg": 0})
        self.assertEqual(st, 200)
        self.assertNotIn("warning", d)
        st, d = self.call("POST", "/api/initial_pose", {"x": 50.0, "y": 0.0, "yaw_deg": 0})
        self.assertEqual(st, 200, d)
        self.assertIn("範囲の外", d["warning"])
        self.assertIn("注意", d["message"])
        self.wait_published()
        self.assertGreaterEqual(len(self.node.pubs["/initialpose"].sent), 3)

    def test_new_pose_supersedes_running_burst(self):
        self.ctrl.repeat_interval_sec = 0.05
        self.call("POST", "/api/initial_pose", {"x": 1, "y": 1, "yaw_deg": 0})
        self.call("POST", "/api/initial_pose", {"x": 2, "y": 2, "yaw_deg": 0})
        self.wait_published()
        import time as _t
        _t.sleep(0.3)
        xs = [m.pose.pose.position.x for m in self.node.pubs["/initialpose"].sent]
        self.assertEqual(xs[-1], 2)
        self.assertEqual(xs.count(2), 3)  # 最後の指定は3回出る(古い指定は途中で打ち切り)


class GlobalLocalizationApi(ApiBase):
    def test_calls_service_and_explains_driving(self):
        st, d = self.call("POST", "/api/global_localization", {})
        self.assertEqual(st, 200, d)
        self.assertEqual(self.node.client.calls, 1)
        self.assertEqual(self.node.client_name, "/reinitialize_global_localization")
        self.assertEqual(self.node.client.timeouts, [0.2])
        self.assertIn("少し走らせる", d["message"])

    def test_service_absent_is_clear_error(self):
        self.node.client.present = False
        st, d = self.call("POST", "/api/global_localization", {})
        self.assertEqual(st, 503)
        self.assertEqual(d["error"], "service_unavailable")
        self.assertIn("/reinitialize_global_localization", d["message"])
        self.assertIn("見つかりません", d["message"])
        self.assertEqual(self.node.client.calls, 0)

    def test_service_failure_is_reported(self):
        self.node.client.exc = RuntimeError("boom")
        st, d = self.call("POST", "/api/global_localization", {})
        self.assertEqual(st, 503)
        self.assertIn("boom", d["message"])

    def test_timeout_without_response(self):
        class Never(FakeFuture):
            def add_done_callback(self, cb):
                pass

        self.node.client.call_async = lambda req: Never()
        st, d = self.call("POST", "/api/global_localization", {})
        self.assertEqual(st, 503)
        self.assertIn("応答しませんでした", d["message"])

    def test_global_localization_resets_stale_estimate(self):
        self.ctrl._on_amcl_pose(self._amcl_msg(0.01, 0.01, 0.01))  # noqa: SLF001
        self.assertEqual(self.ctrl.status()["quality"], "good")
        self.call("POST", "/api/global_localization", {})
        self.assertEqual(self.ctrl.status()["quality"], "waiting")

    @staticmethod
    def _amcl_msg(vx, vy, vyaw, x=1.0, y=2.0, yaw=0.0):
        m = FakePWC()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.pose.covariance = cov(vx, vy, vyaw)
        return m


class StatusApi(ApiBase):
    def test_amcl_subscription_is_lazy_with_matching_qos(self):
        self.assertEqual(self.node.subs, [])  # 誰も見ていなければ購読しない
        st, d = self.call("GET", "/api/localization")
        self.assertEqual(st, 200)
        self.assertEqual(len(self.node.subs), 1)
        sub = self.node.subs[0]
        self.assertEqual(sub.topic, "/amcl_pose")
        self.assertIs(sub.typ, FakePWC)
        q = sub.qos
        self.assertEqual((q.depth, q.reliability.name, q.durability.name), (1, "RELIABLE", "TRANSIENT_LOCAL"))

    def test_waiting_then_quality_from_covariance(self):
        st, d = self.call("GET", "/api/localization")
        self.assertEqual((d["quality"], d["label"]), ("waiting", "位置合わせ待ち"))
        self.assertTrue(d["localization_active"])
        self.assertIn("位置合わせ待ち", d["text"])
        cb = self.node.subs[0].cb
        cb(GlobalLocalizationApi._amcl_msg(0.02, 0.02, 0.01, x=1.0, y=2.0, yaw=math.pi / 2))
        st, d = self.call("GET", "/api/localization")
        self.assertEqual((d["quality"], d["label"]), ("good", "良好"))
        self.assertAlmostEqual(d["pos_std_m"], 0.2)
        self.assertAlmostEqual(d["amcl_pose"]["yaw_deg"], 90.0)
        self.assertIn("位置の確かさ: 良好", d["text"])
        cb(GlobalLocalizationApi._amcl_msg(0.125, 0.125, 0.01))
        self.assertEqual(self.call("GET", "/api/localization")[1]["label"], "要確認")
        cb(GlobalLocalizationApi._amcl_msg(5.0, 5.0, 3.0))
        self.assertEqual(self.call("GET", "/api/localization")[1]["label"], "未確定")

    def test_estimate_cleared_after_sending_pose(self):
        self.call("GET", "/api/localization")
        self.node.subs[0].cb(GlobalLocalizationApi._amcl_msg(0.01, 0.01, 0.01))
        self.assertEqual(self.call("GET", "/api/localization")[1]["quality"], "good")
        self.call("POST", "/api/initial_pose", {"x": 0, "y": 0, "yaw_deg": 0})
        self.wait_published()
        self.assertEqual(self.call("GET", "/api/localization")[1]["quality"], "waiting")

    def test_unsubscribe_clears_latest(self):
        self.call("GET", "/api/localization")
        self.node.subs[0].cb(GlobalLocalizationApi._amcl_msg(0.01, 0.01, 0.01))
        self.ctrl._lazy._last_touch -= 100  # noqa: SLF001
        self.ctrl._lazy._idle_check()  # noqa: SLF001
        self.assertEqual(self.node.subs, [])
        self.assertIsNone(self.ctrl._latest)  # noqa: SLF001

    def test_inactive_when_not_localization(self):
        self.life.st.update({"mode": "mapping"})
        st, d = self.call("GET", "/api/localization")
        self.assertEqual(st, 200)
        self.assertFalse(d["localization_active"])
        self.assertEqual(d["quality"], "inactive")

    def test_tf_availability_none_stale_and_fresh(self):
        self.assertFalse(self.call("GET", "/api/localization")[1]["tf_available"])
        self.robot_pose.pose = {"x": 0, "y": 0, "yaw": 0, "age_s": 30.0}
        self.assertFalse(self.call("GET", "/api/localization")[1]["tf_available"])  # 古い=居座っているだけ
        self.robot_pose.pose = {"x": 0, "y": 0, "yaw": 0, "age_s": 0.2}
        self.assertTrue(self.call("GET", "/api/localization")[1]["tf_available"])
        self.robot_pose.pose = {"x": 0, "y": 0, "yaw": 0, "age_s": None}
        self.assertTrue(self.call("GET", "/api/localization")[1]["tf_available"])


class NoControllerApi(unittest.TestCase):
    def test_routes_404_without_controller(self):
        handler = dc.make_dashboard_handler(
            lambda: b"", lambda: {}, FakeTeleop(), auth_username="", enforce_single_controller=False)
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        try:
            req = urllib.request.Request(
                f"http://127.0.0.1:{server.server_address[1]}/api/initial_pose", data=b"{}", method="POST",
                headers={"Content-Type": "application/json"})
            with self.assertRaises(urllib.error.HTTPError) as cm:
                urllib.request.urlopen(req, timeout=10)
            self.assertEqual(cm.exception.code, 404)
            cm.exception.close()
        finally:
            server.shutdown()
            server.server_close()


if __name__ == "__main__":
    unittest.main()
