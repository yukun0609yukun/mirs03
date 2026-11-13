"""管理画面の「壁から離れる」と「壁際で動けない」状態表示のテスト(2026-10-22)。ROS不要。

- WallEscapeController: 状態の保持、古くなったらunknown、triggerの公開
- API: GET /api/wall_escape/status(guestも可)、POST /api/wall_escape/trigger
  (guest 403 / 権限なしoperator 403 / 権限ありoperator 200 / admin 200 / 緊急停止中 409)
- HTMLにボタンのid(wallEscapeBtn)とdata-cap、roles.API_RULES・GRANTABLEに載っていること
- wall_escape.py の status_dict(stuck判定)。rclpyが無くても動くよう、__new__で最低限の属性だけ与える

実行: python3 -I ros2_ws/src/nokosahen_core/test/test_wall_escape_dashboard.py
"""
import json
import sys
import types
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(TEST_DIR.parent))

import test_account_roles_http as T  # noqa: E402  (ROSの替え玉の用意と、サーバー起動の補助)
import test_dashboard_layout as L  # noqa: E402

from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import roles  # noqa: E402
from nokosahen_core import wall_escape as we  # noqa: E402


class FakeTime:
    def __init__(self, ns):
        self.nanoseconds = ns

    def __sub__(self, other):
        return FakeTime(self.nanoseconds - other.nanoseconds)


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class FakePub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg.data)


class FakeNode:
    def __init__(self):
        self.pubs = {}
        self.subs = {}

    def create_publisher(self, typ, topic, depth):
        self.pubs[topic] = FakePub()
        return self.pubs[topic]

    def create_subscription(self, typ, topic, cb, depth):
        self.subs[topic] = cb


def msg(d):
    m = types.SimpleNamespace()
    m.data = d if isinstance(d, str) else json.dumps(d)
    return m


def make_controller():
    clock = FakeClock()
    node = FakeNode()
    return dc.WallEscapeController(node, clock=clock), node, clock


class ControllerTest(unittest.TestCase):
    def test_topics(self):
        c, node, _ = make_controller()
        self.assertIn("/nokosahen/wall_escape/status", node.subs)
        self.assertIn("/nokosahen/wall_escape/trigger", node.pubs)

    def test_unknown_until_first_status(self):
        c, _, _ = make_controller()
        s = c.status()
        self.assertEqual(s["state"], "unknown")
        self.assertFalse(s["stuck"])
        self.assertTrue(s["reason"])

    def test_keeps_status(self):
        c, node, _ = make_controller()
        node.subs["/nokosahen/wall_escape/status"](msg({"state": "escaping", "min_range_m": 0.31, "reason": "", "stuck": True}))
        s = c.status()
        self.assertEqual((s["state"], s["min_range_m"], s["stuck"]), ("escaping", 0.31, True))

    def test_stale_becomes_unknown(self):
        c, node, clock = make_controller()
        node.subs["/nokosahen/wall_escape/status"](msg({"state": "idle", "min_range_m": None, "reason": "", "stuck": True}))
        clock.t += 4.9
        self.assertEqual(c.status()["state"], "idle")
        clock.t += 0.2
        s = c.status()
        self.assertEqual(s["state"], "unknown")
        self.assertFalse(s["stuck"])  # 古い「動けない」は見せない

    def test_bad_json_is_ignored(self):
        c, node, _ = make_controller()
        node.subs["/nokosahen/wall_escape/status"](msg("not json"))
        node.subs["/nokosahen/wall_escape/status"](msg("[1,2]"))
        self.assertEqual(c.status()["state"], "unknown")

    def test_trigger_publishes_true(self):
        c, node, _ = make_controller()
        c.trigger()
        self.assertEqual(node.pubs["/nokosahen/wall_escape/trigger"].sent, [True])


class ApiTest(T.Base):
    def start_wall(self, perms=None, estop=False):
        T.Bridge.posts = []
        T.Pi.posts = []
        self.teleop = T.FakeTeleop()
        self.teleop.soft_estop_engaged = estop
        self.store = self.make_store(True, perms)
        self.node = FakeNode()
        self.wall = dc.WallEscapeController(self.node)
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, self.teleop,
            auth_username="TL", auth_password="tl-pass", enforce_single_controller=False,
            account_store=self.store, ephemeral=T.eo.EphemeralOverrides(None), wall_escape=self.wall)
        self.srv, self.port = T.serve(handler)
        self.base = f"http://127.0.0.1:{self.port}"
        self.addCleanup(self._stop)

    def sent(self):
        return self.node.pubs["/nokosahen/wall_escape/trigger"].sent

    def test_status_readable_by_guest(self):
        self.start_wall()
        self.node.subs["/nokosahen/wall_escape/status"](msg({"state": "idle", "min_range_m": 0.4, "reason": "", "stuck": True}))
        code, d = self.call("GET", "/api/wall_escape/status", ck=self.cookie("guest", ""))
        self.assertEqual(code, 200)
        self.assertEqual((d["state"], d["stuck"]), ("idle", True))

    def test_status_unknown_when_nothing_received(self):
        self.start_wall()
        code, d = self.call("GET", "/api/wall_escape/status", ck=self.cookie("TL", "tl-pass"))
        self.assertEqual((code, d["state"]), (200, "unknown"))

    def test_guest_403(self):
        self.start_wall()
        code, _ = self.call("POST", "/api/wall_escape/trigger", {}, ck=self.cookie("guest", ""))
        self.assertEqual(code, 403)
        self.assertEqual(self.sent(), [])

    def test_operator_without_permission_403(self):
        self.start_wall(perms=[])
        code, _ = self.call("POST", "/api/wall_escape/trigger", {}, ck=self.cookie("kari-op", T.OP_PASSWORD))
        self.assertEqual(code, 403)
        self.assertEqual(self.sent(), [])

    def test_operator_with_permission_200(self):
        self.start_wall(perms=["nav.wall_escape"])
        code, d = self.call("POST", "/api/wall_escape/trigger", {}, ck=self.cookie("kari-op", T.OP_PASSWORD))
        self.assertEqual(code, 200)
        self.assertTrue(d["ok"])
        self.assertEqual(self.sent(), [True])

    def test_admin_200(self):
        self.start_wall()
        code, _ = self.call("POST", "/api/wall_escape/trigger", {}, ck=self.cookie("TL", "tl-pass"))
        self.assertEqual(code, 200)
        self.assertEqual(self.sent(), [True])

    def test_estop_409(self):
        self.start_wall(perms=["nav.wall_escape"], estop=True)
        for user, pw in (("TL", "tl-pass"), ("kari-op", T.OP_PASSWORD)):
            code, d = self.call("POST", "/api/wall_escape/trigger", {}, ck=self.cookie(user, pw))
            self.assertEqual(code, 409, user)
            self.assertEqual(d["error"], "estop_engaged")
        self.assertEqual(self.sent(), [])


class StaticTest(unittest.TestCase):
    def test_html_has_button(self):
        html = L.pages()["HTML"]
        self.assertIn('id="wallEscapeBtn"', html)
        self.assertIn('data-cap="nav.wall_escape"', html)
        self.assertIn('id="wallStuckNote"', html)
        self.assertIn("/api/wall_escape/trigger", html)
        self.assertIn("/api/wall_escape/status", html)
        self.assertEqual(html.count('id="wallEscapeBtn"'), 1)

    def test_button_in_drive_card(self):
        html = L.pages()["HTML"]
        i = html.index('<section class="card s-drive">')
        j = html.index("</section>", i)
        self.assertIn('id="wallEscapeBtn"', html[i:j])

    def test_roles_tables(self):
        self.assertEqual(roles.API_RULES[("POST", "/api/wall_escape/trigger")], "nav.wall_escape")
        self.assertIn("nav.wall_escape", roles.GRANTABLE)
        self.assertEqual(roles.CAPABILITIES["nav.wall_escape"].recommend, "保留")  # 2026-10-24: 速い(0.7 m/s)ので、既定は保留(管理者が決める)
        self.assertTrue(roles.authorize("operator", ["nav.wall_escape"], "POST", "/api/wall_escape/trigger").allowed)
        self.assertFalse(roles.authorize("operator", [], "POST", "/api/wall_escape/trigger").allowed)
        self.assertTrue(roles.authorize("admin", [], "POST", "/api/wall_escape/trigger").allowed)
        self.assertFalse(roles.authorize("viewer", ["nav.wall_escape"], "POST", "/api/wall_escape/trigger").allowed)


class NodeStatusTest(unittest.TestCase):
    """wall_escape.py の status_dict(ROSなし。__new__で必要な属性だけ与える)。"""

    def make(self, points, abort_ago=None, now=1000.0):
        n = we.WallEscape.__new__(we.WallEscape)
        n._t = now
        n._now = lambda: FakeTime(int(n._t * 1e9))
        n._points = points
        n._scan_time = n._now()
        n._stale = 1.0
        n._trigger = 0.45
        n._last_abort = 0.0 if abort_ago is None else now - abort_ago
        n._escaping_until = None
        n._wasd = n._soft_estop = n._hard_estop = n._near_person = n._router_rotating = False
        n._grant = None
        n._grant_time = None
        n._escape_manual = False
        n._manual_reason = ("", 0.0)
        n._last_stop = ("", 0.0)
        return n

    def test_stuck_needs_close_wall_and_recent_abort(self):
        self.assertTrue(self.make([(0.3, 0.0)], abort_ago=2.0).status_dict()["stuck"])
        self.assertFalse(self.make([(0.5, 0.0)], abort_ago=2.0).status_dict()["stuck"])  # 壁が遠い
        self.assertFalse(self.make([(0.3, 0.0)], abort_ago=6.0).status_dict()["stuck"])  # 中止が古い
        self.assertFalse(self.make([(0.3, 0.0)], abort_ago=None).status_dict()["stuck"])  # 中止なし

    def test_state_fields(self):
        d = self.make([(0.3, 0.0)], abort_ago=1.0).status_dict()
        self.assertEqual(d["state"], "idle")
        self.assertEqual(d["min_range_m"], 0.3)
        self.assertEqual(set(d), {"state", "min_range_m", "reason", "stuck"})
        n = self.make([(0.3, 0.0)])
        n._escaping_until = n._t + 3
        self.assertEqual(n.status_dict()["state"], "escaping")
        n = self.make([(0.3, 0.0)])
        n._soft_estop = True
        d = n.status_dict()
        self.assertEqual(d["state"], "blocked")
        self.assertTrue(d["reason"])

    def test_no_scan_gives_null_range(self):
        n = self.make([(0.3, 0.0)], abort_ago=1.0)
        n._t += 5.0
        d = n.status_dict()
        self.assertIsNone(d["min_range_m"])
        self.assertFalse(d["stuck"])

    def test_failed_after_aborted_escape(self):
        n = self.make([(0.3, 0.0)])
        n._last_stop = ("時間切れ", n._t - 2.0)
        d = n.status_dict()
        self.assertEqual((d["state"], d["reason"]), ("failed", "時間切れ"))
        n._last_stop = ("十分に離れました", n._t - 2.0)
        self.assertEqual(n.status_dict()["state"], "idle")


if __name__ == "__main__":
    unittest.main()
