"""走行可能範囲の中で「動く前に」外へ出ないかを計算する boundary_motion_guard と、
behavior_director の直接動作(見回し・揺動・直進)への組み込みの単体テスト。rclpy不要。
実行: python ros2_ws/src/nokosahen_core/test/test_boundary_motion_guard.py
"""

import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import test_behavior_rules as base  # noqa: E402
from nokosahen_core import boundary_motion_guard as g  # noqa: E402
from nokosahen_core import map_orientation as mo  # noqa: E402

bd = base.bd


def rect(w=4.0, h=4.0, cx=0.0, cy=0.0, yaw=0.0):
    return mo.make_rect(cx, cy, w, h, yaw)


class GuardMathTests(unittest.TestCase):
    def test_no_rect_means_free(self):
        self.assertTrue(g.can_rotate_in_place(None, None))
        self.assertTrue(g.can_drive(None, None))
        self.assertEqual(g.safe_drive_distance(None, None, 1.0), 1.0)

    def test_unknown_pose_is_blocked(self):
        r = rect()
        self.assertFalse(g.can_rotate_in_place(r, None))
        self.assertEqual(g.safe_drive_distance(r, None, 1.0), 0.0)
        self.assertFalse(g.can_drive(r, {"x": float("nan"), "y": 0.0, "yaw": 0.0}))

    def test_rotate_needs_circumradius_plus_margin_from_every_edge(self):
        r = rect(4.0, 4.0)  # 中心(0,0)、縁は±2
        edge = 2.0 - g.ROTATE_MARGIN_M
        self.assertTrue(g.can_rotate_in_place(r, {"x": 0.0, "y": 0.0, "yaw": 0.0}))
        self.assertTrue(g.can_rotate_in_place(r, {"x": edge - 0.01, "y": 0.0, "yaw": 0.0}))
        self.assertFalse(g.can_rotate_in_place(r, {"x": edge + 0.01, "y": 0.0, "yaw": 0.0}))
        self.assertFalse(g.can_rotate_in_place(r, {"x": 0.0, "y": -(edge + 0.01), "yaw": 1.0}))

    def test_rotate_corner_of_rotated_rect(self):
        r = rect(4.0, 2.0, yaw=math.radians(30))
        self.assertTrue(g.can_rotate_in_place(r, {"x": 0.0, "y": 0.0, "yaw": 0.0}))
        self.assertFalse(g.can_rotate_in_place(r, {"x": 1.5, "y": 0.0, "yaw": 0.0}))

    def test_drive_distance_stops_before_the_edge(self):
        r = rect(4.0, 4.0)
        p = {"x": 0.0, "y": 0.0, "yaw": 0.0}  # +x向き。機体の前面は x+0.26。縁2.0、余白0.05
        d = g.safe_drive_distance(r, p, 5.0)
        # 前面(x+0.26)+余白0.05が縁2.0に着く手前 = 1.69 m
        self.assertAlmostEqual(d, 1.65, delta=0.06)
        self.assertLess(d + 0.26 + g.SAFETY_MARGIN_M, 2.0 + 1e-6)

    def test_drive_blocked_when_already_outside_or_touching(self):
        r = rect(4.0, 4.0)
        self.assertEqual(g.safe_drive_distance(r, {"x": 1.9, "y": 0.0, "yaw": 0.0}, 1.0), 0.0)
        self.assertFalse(g.can_drive(r, {"x": 1.5, "y": 0.0, "yaw": 0.0}))  # 先読み0.35 mで出る
        self.assertTrue(g.can_drive(r, {"x": 1.0, "y": 0.0, "yaw": 0.0}))

    def test_reverse_direction(self):
        r = rect(4.0, 4.0)
        p = {"x": -1.5, "y": 0.0, "yaw": 0.0}  # +x向きで後退すると -x側の縁へ
        self.assertFalse(g.can_drive(r, p, direction=-1.0))
        self.assertTrue(g.can_drive(r, p, direction=1.0))

    def test_footprint_corners_inside_check(self):
        r = rect(0.8, 0.8)  # 小さい範囲: 半幅0.4 < 旋回に必要な0.42 → 旋回は不可。ただし向きを変えずに置くだけなら収まる
        self.assertFalse(g.can_rotate_in_place(r, {"x": 0.0, "y": 0.0, "yaw": 0.0}))
        self.assertTrue(g.footprint_inside(r, 0.0, 0.0, 0.0))


@unittest.skipIf(bd is None, "behavior_director import失敗")
class DirectorBoundaryGuardTests(unittest.TestCase):
    """範囲(enabled)を設定し、directorの直接動作が外へ出ないことを確かめる。"""

    def setUp(self):
        self.h = base.DirectorTickTests("make")
        self.h.setUp()

    def tearDown(self):
        self.h.tearDown()

    def make(self, pose, target=False, switch=True, w=4.0, h=4.0):
        n = self.h.make(None, crowd="calm", target=target)
        n._require_interaction_opt_in = True
        n._person_interaction_enabled = switch
        n._scan_enabled = switch  # 見回す・揺動は「見回す」スイッチ(2026-11-03、近づくとは独立)
        n._boundary = mo.boundary_state(rect(w, h), enabled=True)
        n._get_pose = lambda: pose
        n._clock.t += 10
        return n

    def modes(self, n, seconds=1.0):
        return self.h.run_ticks(n, seconds)

    def test_scan_rotates_when_there_is_room(self):
        n = self.make({"x": 0.0, "y": 0.0, "yaw": 0.0})
        self.assertEqual(set(self.modes(n)), {"CROWD_SCAN"})
        self.assertTrue(self.h.moved(n))

    def test_scan_does_not_rotate_near_the_edge(self):
        n = self.make({"x": 1.8, "y": 0.0, "yaw": 0.0})  # 縁2.0まで0.2 m: 旋回すると角が出る
        modes = self.modes(n)
        self.assertNotIn("CROWD_SCAN", modes)
        self.assertFalse(self.h.moved(n))

    def test_scan_blocked_when_pose_unknown(self):
        n = self.make(None)
        self.assertNotIn("CROWD_SCAN", self.modes(n))
        self.assertFalse(self.h.moved(n))

    def test_wiggle_does_not_rotate_near_the_edge(self):
        n = self.make({"x": 1.8, "y": 0.0, "yaw": 0.0})
        n._idle_wiggle_enabled = True
        modes = self.modes(n)
        self.assertNotIn("IDLE_WIGGLE", modes)
        self.assertFalse(self.h.moved(n))

    def test_wiggle_works_with_room(self):
        n = self.make({"x": 0.0, "y": 0.0, "yaw": 0.0})
        n._idle_wiggle_enabled = True
        self.assertEqual(set(self.modes(n)), {"IDLE_WIGGLE"})

    def test_scan_drive_phase_stops_before_the_edge(self):
        n = self.make({"x": 1.5, "y": 0.0, "yaw": 0.0})  # 回転はできる(縁まで0.5 > 0.418)が、直進0.35 mは外へ出る
        n._scan_phase = "approaching"
        n._scan_phase_start_time = n._clock.now()
        n._scan_start_pose = {"x": 1.5, "y": 0.0, "yaw": 0.0}
        n._scan_last_pose = n._scan_start_pose
        self.modes(n, 0.5)
        drives = [t for t in n._cmd_vel_pub.msgs if t.linear.x > 0]
        self.assertEqual(drives, [])  # 前進の指令は出さない

    def test_scan_drive_runs_when_clear(self):
        n = self.make({"x": 0.0, "y": 0.0, "yaw": 0.0})
        n._scan_phase = "approaching"
        n._scan_phase_start_time = n._clock.now()
        n._scan_start_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        n._scan_last_pose = n._scan_start_pose
        self.modes(n, 0.5)
        self.assertTrue([t for t in n._cmd_vel_pub.msgs if t.linear.x > 0])

    def test_no_boundary_keeps_legacy(self):
        n = self.h.make(None, crowd="calm", target=False)
        n._require_interaction_opt_in = False
        n._boundary = {"enabled": False}
        n._get_pose = lambda: None
        n._clock.t += 10
        self.assertEqual(set(self.modes(n)), {"CROWD_SCAN"})

    def test_goal_clamp_keeps_margin_from_the_edge(self):
        n = self.make({"x": 0.0, "y": 0.0, "yaw": 0.0})
        x, y = n._clamp_to_boundary(10.0, 0.0)
        self.assertLessEqual(x, 2.0 - g.GOAL_MARGIN_M + 1e-6)
        self.assertAlmostEqual(x, 2.0 - g.GOAL_MARGIN_M, delta=1e-6)

    def test_command_reports_block_reason(self):
        n = self.make({"x": 1.8, "y": 0.0, "yaw": 0.0})
        self.modes(n)
        cmd = json.loads(n._command_pub.msgs[-1].data)
        self.assertTrue(cmd.get("boundary_motion_blocked"))


def _line(x0, y0, x1, y1, step=0.05):
    n = max(1, int(round(math.hypot(x1 - x0, y1 - y0) / step)))
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]


class PathExitTests(unittest.TestCase):
    def test_inside_straight_and_curve(self):
        r = rect(4.0, 4.0)
        self.assertFalse(g.path_exit_info(_line(-1.5, 0, 1.5, 0), r)["exits"])
        curve = [(1.5 * math.cos(t / 20.0), 1.5 * math.sin(t / 20.0)) for t in range(0, 63)]
        self.assertFalse(g.path_exit_info(curve, r)["exits"])

    def test_along_edge_from_inside(self):
        r = rect(4.0, 4.0)
        self.assertFalse(g.path_exit_info(_line(-1.5, 1.95, 1.5, 1.95), r)["exits"])

    def test_exits_by_more_than_min_run(self):
        r = rect(4.0, 4.0)
        info = g.path_exit_info(_line(0, 0, 0, 2.6), r)
        self.assertTrue(info["exits"])
        self.assertGreaterEqual(info["run_m"], 0.3)
        self.assertIsNotNone(info["first_xy"])

    def test_brief_blip_is_ignored(self):
        r = rect(4.0, 4.0)
        pts = _line(1.0, 0.0, 2.2, 0.0) + _line(2.2, 0.0, 1.0, 0.0)[1:]
        self.assertFalse(g.path_exit_info(pts, r)["exits"])

    def test_leading_outside_returning_is_ignored(self):
        r = rect(4.0, 4.0)
        self.assertFalse(g.path_exit_info(_line(2.6, 0, 0, 0), r)["exits"])
        pts = _line(2.6, 0, 0, 0) + _line(0, 0, 0, 2.8)[1:]
        self.assertTrue(g.path_exit_info(pts, r)["exits"])

    def test_degenerate_inputs_do_not_exit(self):
        r = rect(4.0, 4.0)
        far = _line(0, 0, 0, 5.0)
        self.assertFalse(g.path_exit_info(far, None)["exits"])
        self.assertFalse(g.path_exit_info([(0, 0)], r)["exits"])
        self.assertFalse(g.path_exit_info([], r)["exits"])
        nan = float("nan")
        self.assertFalse(g.path_exit_info([(0, 0), (nan, 5.0), (0, nan)], r)["exits"])
        self.assertTrue(g.path_exit_info(far[:30] + [(nan, nan)] + far[30:], r)["exits"])

    def test_rotated_rect(self):
        r = rect(4.0, 2.0, cx=1.0, cy=-1.0, yaw=math.radians(30))
        c, s = math.cos(math.radians(30)), math.sin(math.radians(30))

        def w(lx, ly):
            return (1.0 + lx * c - ly * s, -1.0 + lx * s + ly * c)

        inside = [w(x / 20.0, 0.9) for x in range(-35, 36)]
        self.assertFalse(g.path_exit_info(inside, r)["exits"])
        out = [w(0.0, y / 20.0) for y in range(0, 41)]
        self.assertTrue(g.path_exit_info(out, r)["exits"])
        blip = [w(0.0, y / 20.0) for y in list(range(0, 22)) + list(range(21, -1, -1))]
        self.assertFalse(g.path_exit_info(blip, r)["exits"])


if __name__ == "__main__":
    unittest.main()
