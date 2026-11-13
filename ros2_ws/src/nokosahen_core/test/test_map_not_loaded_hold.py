"""地図を読み込んでいない間は、自律動作をせず止まる(MAP_NOT_LOADED_HOLD、2026-11-04)の単体テスト。

「地図を読み込んでいない」= 地図上の位置(map->base_link)が map_missing_grace_sec 秒以上取れない。
rclpy不要。実行: python -m pytest -q test/test_map_not_loaded_hold.py
"""

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import test_behavior_rules as base  # noqa: E402

bd = base.bd


@unittest.skipIf(bd is None, "behavior_director import失敗")
class MapNotLoadedHoldTests(unittest.TestCase):
    def setUp(self):
        self.h = base.DirectorTickTests("make")
        self.h.setUp()

    def tearDown(self):
        self.h.tearDown()

    def make(self, pose=None, require=True, face=True, approach=False, scan=False, crowd="calm", target=True):
        n = self.h.make(None, crowd=crowd, target=target)
        n._require_interaction_opt_in = True
        n._person_interaction_enabled = approach
        n._scan_enabled = scan
        n._face_enabled = face
        n.require_map_loaded = require
        n.map_missing_grace_sec = 3.0
        n._get_pose = lambda: pose
        n._clock.t += 10
        return n

    def modes(self, n, seconds):
        return self.h.run_ticks(n, seconds)

    def cmd(self, n):
        return json.loads(n._command_pub.msgs[-1].data)

    def test_holds_and_does_not_move_when_pose_missing(self):
        n = self.make(pose=None, approach=True, scan=True, face=True)
        self.modes(n, 3.5)  # 猶予(3秒)の間は従来どおり(一瞬のTF欠けで止めないため)
        goals = len(n._goal_pub.msgs)
        sent = len(n._cmd_vel_pub.msgs)
        modes = self.modes(n, 6.0)
        self.assertEqual(modes[-1], "MAP_NOT_LOADED_HOLD")
        self.assertTrue(self.cmd(n)["map_not_loaded"])
        self.assertEqual(len(n._goal_pub.msgs), goals)  # 止まったあとは、近づく(目的地の送信)もしない
        self.assertFalse([t for t in n._cmd_vel_pub.msgs[sent:] if t.linear.x != 0.0 or t.angular.z != 0.0])

    def test_short_gap_does_not_hold(self):
        # 一瞬のTF欠けでは止めない(猶予 map_missing_grace_sec 秒)
        n = self.make(pose=None)
        modes = self.modes(n, 2.0)
        self.assertNotIn("MAP_NOT_LOADED_HOLD", modes)

    def test_normal_behaviour_when_pose_available(self):
        n = self.make(pose={"x": 0.0, "y": 0.0, "yaw": 0.0}, face=True)
        modes = self.modes(n, 6.0)
        self.assertNotIn("MAP_NOT_LOADED_HOLD", modes)
        self.assertFalse(self.cmd(n)["map_not_loaded"])

    def test_resumes_when_map_loaded(self):
        n = self.make(pose=None)
        self.modes(n, 5.0)
        self.assertEqual(self.cmd(n)["mode"], "MAP_NOT_LOADED_HOLD")
        n._get_pose = lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0}
        modes = self.modes(n, 1.0)
        self.assertNotIn("MAP_NOT_LOADED_HOLD", modes)

    def test_disabled_keeps_legacy(self):
        n = self.make(pose=None, require=False)
        self.assertNotIn("MAP_NOT_LOADED_HOLD", self.modes(n, 6.0))

    def test_mapping_is_not_blocked_by_this_hold(self):
        # 地図作成中・探索中は、別の(従来の)停止。この停止にはならない
        n = self.make(pose=None)
        n._map_building_active = True
        modes = self.modes(n, 6.0)
        self.assertNotIn("MAP_NOT_LOADED_HOLD", modes)
        self.assertEqual(modes[-1], "MAPPING_ACTIVE_HOLD")

    def test_manual_override_is_not_blocked(self):
        n = self.make(pose=None)
        n._manual_override_raw = True
        n._last_manual_override_time = n._clock.now()
        modes = self.modes(n, 0.5)
        self.assertEqual(set(modes), {"MANUAL_OVERRIDE"})

    def test_estop_still_wins(self):
        n = self.make(pose=None)
        n._hardware_estop = True
        self.assertEqual(self.modes(n, 5.0)[-1], "ESTOP_HOLD")

    def test_hold_cancels_nav2_goal_and_stops(self):
        n = self.make(pose=None)
        n._nav2_goal_pending = True
        self.modes(n, 5.0)
        self.assertTrue(n._cancel_goal_pub.msgs)

    def test_mode_in_safety_and_hold_sets(self):
        self.assertIn("MAP_NOT_LOADED_HOLD", bd._SAFETY_MODES)
        self.assertIn("MAP_NOT_LOADED_HOLD", bd._HOLD_CANCEL_MODES)
        self.assertIn("MAP_NOT_LOADED_HOLD", bd._GOAL_CANCELLING_HOLD_MODES)


if __name__ == "__main__":
    unittest.main()
