"""nav2_goal_routerの迂回の処理(nav2_detour.py)のテスト(2026-10-25)。ROSは使わない(スタブ)。

Nav2の目的地が中止(status 6)になったときの、始める条件・計画/判定/送信の流れ・中止の扱い・後退しないこと(/cmd_velへ出さない)を確かめる。
実行: python3 -I ros2_ws/src/nokosahen_core/test/test_detour_router.py
"""
import json
import math
import os
import sys
import time
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

for _name, _attrs in (
    ("rclpy", ()), ("rclpy.action", ("ActionClient",)), ("rclpy.duration", ("Duration",)),
    ("rclpy.node", ("Node",)), ("geometry_msgs", ()), ("geometry_msgs.msg", ("PoseStamped", "Twist")),
    ("std_msgs", ()), ("std_msgs.msg", ("Bool", "String")),
):
    try:
        __import__(_name)
    except Exception:  # noqa: BLE001
        _m = types.ModuleType(_name)
        for _a in _attrs:
            setattr(_m, _a, object)
        sys.modules[_name] = _m

from nokosahen_core import nav2_detour as nd  # noqa: E402
from nokosahen_core import nav2_goal_router as r  # noqa: E402

NS = types.SimpleNamespace


def box(cx, cy, w, d, step=0.03):
    pts = []
    x = cx - w / 2
    while x <= cx + w / 2 + 1e-9:
        pts += [(x, cy - d / 2), (x, cy + d / 2)]
        x += step
    y = cy - d / 2
    while y <= cy + d / 2 + 1e-9:
        pts += [(cx - w / 2, y), (cx + w / 2, y)]
        y += step
    return pts


def line(x0, y0, x1, y1, step=0.1):
    n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]


def fake_pose(x=0.0, y=0.0, yaw=0.0):
    return NS(header=NS(frame_id="map", stamp=None), pose=NS(position=NS(x=x, y=y), orientation=NS(z=math.sin(yaw / 2), w=math.cos(yaw / 2))))


class FakeString:
    def __init__(self, data=""):
        self.data = data


class FakePub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(("info", m))

    def warn(self, m):
        self.lines.append(("warn", m))

    def error(self, m):
        self.lines.append(("error", m))


class FakeClient:
    def __init__(self, ready=True):
        self.ready = ready

    def server_is_ready(self):
        return self.ready


GOAL = (4.0, 0.0)


class DetourRouterTests(unittest.TestCase):
    def setUp(self):
        self._orig_pose = nd.PoseStamped
        self._orig_str = (nd.String, r.String)
        nd.String = r.String = FakeString
        nd.PoseStamped = lambda: NS(header=NS(frame_id="", stamp=None), pose=NS(position=NS(x=0, y=0), orientation=NS(z=0, w=1)))
        self.n = self.make()

    def tearDown(self):
        nd.PoseStamped = self._orig_pose
        nd.String, r.String = self._orig_str

    def make(self):
        n = r.Nav2GoalRouter.__new__(r.Nav2GoalRouter)
        n._log = FakeLogger()
        n.get_logger = lambda: n._log
        n.get_clock = lambda: NS(now=lambda: NS(to_msg=lambda: None))
        n._goal_seq = 7
        n._active_goal_seq = 7
        n._hardware_estop = False
        n._soft_estop = False
        n._survey_active = False
        n._survey_active_time = None
        n._wasd_active_prev = False
        n._grant_time = None
        n._grant_owner = None
        n._grant_rotate = True
        n._goal_status_pub = FakePub()
        n._cmd_vel_pub = FakePub()
        n._nav_cmd_pub = FakePub()
        n._client = FakeClient()
        n._compute_client = FakeClient()
        n._detour_enabled = True
        n._detour = nd.DetourMachine(max_attempts=2, settle_sec=1.0)
        n._detour_lookahead = 4.0
        n._detour_margin = 0.03
        n._detour_max_points = 150
        n._detour_offsets = (0.7, 1.0, 1.4)
        n._detour_pre_rotate = math.radians(20.0)
        n._nav_pose = fake_pose(*GOAL)
        n._detour_goal_pose = None
        n._boundary_rect = None
        n._sent = []
        n._send_navigate = lambda pose: n._sent.append((pose.pose.position.x, pose.pose.position.y))
        n._cancelled = []
        n._cancel_active_goal = lambda reason="": n._cancelled.append(reason)
        n._fallback = None
        n._begin_fallback_rotation = lambda pose, seq: setattr(n, "_fallback", (pose, seq))
        n._spin_client = None
        n._plan_requests = []
        n.plans = {}      # tag別(wp_aはidx別)に返す経路。Noneなら計画失敗
        n.robot = (0.0, 0.0, 0.0)
        n.scan = []

        def plan(tag, target, idx):
            n._plan_requests.append((tag, target, idx))
            key = tag if tag != "wp_a" and tag != "wp_b" else (tag, idx)
            path = n.plans.get(key, n.plans.get(tag))
            n._detour_plan_done(n._detour.sid, tag, path)

        n._detour_plan = plan
        n._robot_pose_map = lambda: n.robot
        n._detour_scan_map = lambda pose, max_range: list(n.scan)
        return n

    def events(self):
        return [json.loads(m.data) for m in self.n._goal_status_pub.sent]

    def result(self, status, xy=GOAL):
        return self.n._detour_handle_result(status, xy)

    def run_settle(self, n=None):
        n = n or self.n
        m = n._detour
        m._deadline = 0.0     # 1秒待ち終わった扱い
        n._detour_apply(m.tick(time.monotonic()))

    # ---- 始める条件 ----
    def test_abort_starts_detour_and_suppresses_finished(self):
        self.assertTrue(self.result(6))
        self.assertTrue(self.n._detour.active)
        self.assertEqual(self.events(), [])    # 中止の通知はまだ出さない
        self.assertEqual(self.n._sent, [])     # 1秒待つ前は何も送らない
        self.assertEqual(self.n._detour_seq, 7)

    def test_not_started_on_other_statuses_or_disabled(self):
        self.assertFalse(self.result(4))
        self.assertFalse(self.result(5))
        self.assertFalse(self.n._detour.active)
        self.n._detour_enabled = False
        self.assertFalse(self.result(6))
        m = self.make()
        m._detour = None
        self.assertFalse(m._detour_handle_result(6, GOAL))

    def test_not_started_when_estop_manual_survey_or_near_person(self):
        for setup in (
            lambda n: setattr(n, "_soft_estop", True),
            lambda n: setattr(n, "_hardware_estop", True),
            lambda n: setattr(n, "_wasd_active_prev", True),
            lambda n: (setattr(n, "_grant_time", time.monotonic()), setattr(n, "_grant_owner", "near_person")),
            lambda n: (setattr(n, "_grant_time", time.monotonic()), setattr(n, "_grant_owner", "estop")),
            lambda n: (setattr(n, "_grant_time", time.monotonic()), setattr(n, "_grant_owner", "manual")),
        ):
            n = self.make()
            setup(n)
            self.assertFalse(n._detour_handle_result(6, GOAL))
            self.assertFalse(n._detour.active)

    def test_not_started_when_goal_was_cancelled_or_replaced(self):
        self.n._goal_seq = 8      # 受理したあとに取り消し/新しい目的地が来た
        self.assertFalse(self.result(6))

    def test_not_started_when_servers_unready_or_other_frame(self):
        self.n._compute_client.ready = False
        self.assertFalse(self.result(6))
        self.n._compute_client.ready = True
        self.n._nav_pose.header.frame_id = "base_link"
        self.assertFalse(self.result(6))
        self.n._nav_pose = fake_pose(1.0, 1.0)     # 別の目的地の結果
        self.n._nav_pose.header.frame_id = "map"
        self.assertFalse(self.result(6))

    # ---- 流れ ----
    def test_direct_clear_path_sends_original_goal_without_moving_back(self):
        self.n.plans["direct"] = line(0, 0, 4, 0)
        self.result(6)
        self.run_settle()
        self.assertEqual(self.n._plan_requests, [("direct", "goal", 0)])
        self.assertEqual(self.n._sent, [GOAL])
        self.assertEqual(self.events(), [])
        # 送った目的地が着いた
        self.assertTrue(self.result(4))
        self.assertEqual([e["event"] for e in self.events()], ["finished"])
        self.assertEqual(self.events()[0]["status"], 4)
        self.assertEqual(self.n._cmd_vel_pub.sent, [])
        self.assertEqual(self.n._nav_cmd_pub.sent, [])

    def test_blocked_direct_uses_waypoint_then_goal(self):
        self.n.scan = box(2.0, 0.0, 0.5, 0.5)
        self.n.robot = (0.0, 0.0, math.atan2(1.24, 2.0))   # 中継点の向きを向いている(旋回なしで送る)
        self.n.plans["direct"] = line(0, 0, 4, 0)   # 箱を通る -> 越せない
        self.n.plans["wp_a"] = None
        self.n.plans[("wp_a", 0)] = None
        self.result(6)
        # 中継点は、箱の外側を通る経路を返す(中継点ごとに、機体->中継点と中継点->目的地)
        cands_seen = []

        def plan(tag, target, idx):
            self.n._plan_requests.append((tag, target, idx))
            cands = self.n._detour_cands
            if tag == "direct":
                path = line(0, 0, 4, 0)
            elif tag == "wp_a":
                cands_seen.append(cands[idx])
                path = line(0, 0, cands[idx][0], cands[idx][1])
            else:
                path = line(cands[idx][0], cands[idx][1], GOAL[0], GOAL[1])
            self.n._detour_plan_done(self.n._detour.sid, tag, path)

        self.n._detour_plan = plan
        self.run_settle()
        self.assertEqual(len(self.n._sent), 1)
        wp = self.n._sent[0]
        self.assertNotEqual(wp, GOAL)
        self.assertGreater(abs(wp[1]), 0.8)                       # 箱の横
        self.assertEqual(self.n._detour.attempts, 1)
        self.assertEqual(self.events(), [])
        # 中継点に着いたら元の目的地を送る
        self.assertTrue(self.result(4, xy=wp))
        self.assertEqual(self.n._sent[1], GOAL)
        self.assertTrue(self.result(4))
        self.assertEqual([(e["event"], e["status"]) for e in self.events()], [("finished", 4)])
        self.assertEqual(self.n._cmd_vel_pub.sent, [])

    def test_all_blocked_gives_up_without_sending_and_reports_detour_failed(self):
        self.n.scan = box(2.0, 0.0, 0.5, 0.5)
        self.n.plans["direct"] = line(0, 0, 4, 0)
        # 中継点への経路も箱を通る(越せない)
        self.n.plans["wp_a"] = line(0, 0, 4, 0)
        self.n.plans["wp_b"] = line(0, 0, 4, 0)
        self.result(6)
        self.run_settle()
        self.assertEqual(self.n._sent, [])
        ev = self.events()
        self.assertEqual([e["event"] for e in ev], ["detour_failed", "finished"])
        self.assertEqual(ev[0]["reason"], "no_clear_route")
        self.assertEqual((ev[0]["target_x"], ev[0]["target_y"]), GOAL)
        self.assertEqual(ev[1]["status"], 6)      # 従来の消費側(探索の除外等)のための中止の通知
        self.assertFalse(self.n._detour.active)
        self.assertEqual(self.n._cmd_vel_pub.sent, [])
        self.assertEqual(self.n._nav_cmd_pub.sent, [])

    def test_no_scan_means_not_clear(self):
        self.n.plans["direct"] = line(0, 0, 4, 0)
        self.n._detour_scan_map = lambda pose, max_range: None
        self.result(6)
        self.run_settle()
        self.assertEqual(self.n._sent, [])
        self.assertEqual(self.events()[0]["event"], "detour_failed")

    def test_resent_goal_abort_does_not_start_a_new_detour(self):
        self.n.plans["direct"] = line(0, 0, 4, 0)
        self.result(6)
        self.run_settle()
        sid = self.n._detour.sid
        self.assertTrue(self.result(6))                  # 送り直した目的地もまた中止 -> 迂回の処理の中で扱う
        self.assertEqual(self.n._detour.sid, sid)         # 新しい処理は始めない
        self.assertEqual(self.events(), [])
        self.run_settle()
        self.assertEqual(self.n._sent, [GOAL, GOAL])
        self.assertTrue(self.result(6))                  # attempts 2回で終わり
        self.assertEqual([e["event"] for e in self.events()], ["detour_failed", "finished"])
        self.assertEqual(self.events()[0]["reason"], "attempts_exhausted")
        self.assertFalse(self.n._detour.active)

    def test_send_rejected_gives_up(self):
        self.n.plans["direct"] = line(0, 0, 4, 0)
        self.result(6)
        self.run_settle()
        self.n._detour_send_error("rejected")
        self.assertEqual([e["event"] for e in self.events()], ["detour_failed", "finished"])
        self.assertEqual(self.events()[0]["reason"], "rejected")
        self.n._detour_send_error("rejected")       # 二重に呼んでも何も起きない
        self.assertEqual(len(self.events()), 2)

    # ---- 取り消し ----
    def test_tick_cancels_on_new_goal_estop_manual(self):
        for setup in (
            lambda n: setattr(n, "_goal_seq", 99),
            lambda n: setattr(n, "_soft_estop", True),
            lambda n: setattr(n, "_wasd_active_prev", True),
            lambda n: (setattr(n, "_grant_time", time.monotonic()), setattr(n, "_grant_owner", "near_person")),
        ):
            n = self.make()
            n._detour_handle_result(6, GOAL)
            self.assertTrue(n._detour.active)
            setup(n)
            n._detour_tick()
            self.assertFalse(n._detour.active)
            self.assertEqual([e["event"] for e in [json.loads(m.data) for m in n._goal_status_pub.sent]], ["finished"])
            self.assertEqual(json.loads(n._goal_status_pub.sent[0].data)["status"], 5)
            self.assertEqual(n._sent, [])
            n._detour_tick()      # 取り消し後は何もしない
            self.assertEqual(len(n._goal_status_pub.sent), 1)

    def test_cancel_while_sending_ignores_late_result(self):
        self.n.plans["direct"] = line(0, 0, 4, 0)
        self.result(6)
        self.run_settle()
        self.n._goal_seq = 99
        self.n._detour_tick()
        self.assertFalse(self.n._detour.active)
        self.assertEqual([e["status"] for e in self.events()], [5])
        # 取り消し後に届く古い結果(status 5)で、再び迂回の処理は始まらない
        self.assertFalse(self.n._detour_handle_result(5, GOAL))
        self.assertFalse(self.n._detour.active)

    def test_near_person_during_send_cancels_nav2_goal(self):
        self.n.plans["direct"] = line(0, 0, 4, 0)
        self.result(6)
        self.run_settle()
        self.n._grant_time = time.monotonic()
        self.n._grant_owner = "near_person"
        self.n._detour_tick()
        self.assertEqual(len(self.n._cancelled), 1)

    def test_double_start_is_ignored(self):
        self.result(6)
        sid = self.n._detour.sid
        self.assertFalse(self.n._detour_handle_result(6, GOAL))    # 計画中に中止の結果が届いても、新しい処理は始めない
        self.assertEqual(self.n._detour.sid, sid)

    # ---- 旋回 ----
    def test_rotation_needed_requires_grant_and_uses_fallback_without_spin(self):
        self.n.robot = (0.0, 0.0, math.pi)          # 目的地と反対向き
        self.n.plans["direct"] = line(0, 0, 4, 0)
        self.result(6)
        self.run_settle()
        # Spinが使えないので、既存の代替の回頭へ
        self.assertEqual(self.n._sent, [])
        self.assertIsNotNone(self.n._fallback)

    def test_rotation_not_granted_gives_up(self):
        n = self.make()
        n.robot = (0.0, 0.0, math.pi)
        n.plans["direct"] = line(0, 0, 4, 0)
        n._grant_rotate = False
        n._grant_time = time.monotonic()
        n._grant_owner = "nav2"
        n._detour_handle_result(6, GOAL)
        self.run_settle(n)
        events = [json.loads(m.data) for m in n._goal_status_pub.sent]
        self.assertEqual(events[0]["reason"], "rotate_not_granted")
        self.assertEqual(n._sent, [])

    def test_small_heading_error_sends_without_rotation(self):
        self.n.robot = (0.0, 0.0, math.radians(10))
        self.n.plans["direct"] = line(0, 0, 4, 0)
        self.result(6)
        self.run_settle()
        self.assertEqual(self.n._sent, [GOAL])
        self.assertIsNone(self.n._fallback)

    # ---- 純関数 ----
    def test_helpers(self):
        poses = [NS(pose=NS(position=NS(x=float(i), y=0.0))) for i in range(3)] + [NS(pose=NS(position=NS(x=float("nan"), y=0.0)))]
        self.assertEqual(nd.path_to_xy(poses), [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])
        self.assertAlmostEqual(nd.end_heading([(0, 0), (0, 1), (1, 1)]), 0.0, places=6)
        self.assertIsNone(nd.end_heading([(0, 0)]))
        z, w = nd.yaw_to_quaternion_zw(math.pi)
        self.assertAlmostEqual(z, 1.0)
        self.assertAlmostEqual(w, 0.0, places=6)

    def test_router_wiring_is_static(self):
        src = open(os.path.join(os.path.dirname(__file__), "..", "nokosahen_core", "nav2_goal_router.py"), encoding="utf-8").read()
        self.assertIn("class Nav2GoalRouter(DetourMixin, Node)", src)
        self.assertIn("self._detour_init()", src)
        self.assertIn("self._detour_handle_result(int(result.status), goal_xy)", src)
        det = open(os.path.join(os.path.dirname(__file__), "..", "nokosahen_core", "nav2_detour.py"), encoding="utf-8").read()
        self.assertNotIn("/cmd_vel\"", det)             # /cmd_velへ直接出さない(安全層を通る/cmd_vel_nav・Nav2の目的地だけ)
        self.assertNotIn("_cmd_vel_pub.publish", det)
        self.assertNotIn("_nav_cmd_pub.publish", det)
        self.assertIn('"compute_path_to_pose"', det)
        for p in ("detour_enabled", "detour_max_attempts"):
            self.assertIn(f'declare_parameter("{p}"', det)


if __name__ == "__main__":
    unittest.main(verbosity=1)
