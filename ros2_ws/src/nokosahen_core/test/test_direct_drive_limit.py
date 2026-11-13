"""directorが直接/cmd_velへ出す前進(衝突監視を通らない経路)の速度上限のテスト(2026-10-13)。

衝突監視(collision_monitor)は cmd_vel_smoothed -> cmd_vel の間にあり、directorが直接出す
CROWD_APPROACH(見回し後の直進)と接近フォールバック(enable_direct_cmd_vel)はそこを通らない。
そのため直進は DIRECT_DRIVE_MAX_MPS(0.5 m/s)で頭打ちにする。旋回(CROWD_SCAN/IDLE_WIGGLE)は変えない。
同時に、設定(yaml・コードの既定)がその上限に収まっていること、docsが現状を書いていることも見張る。

rclpy不要(スタブ)。ロボット・ROSには触れない。
実行: python ros2_ws/src/nokosahen_core/test/test_direct_drive_limit.py
"""

import re
import sys
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
PKG = TEST_DIR.parent
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(PKG))

import test_behavior_rules as tbr  # noqa: E402  (rclpyスタブ注入とdirectorの組み立てを流用)

bd = tbr.bd
REPO = PKG.parent.parent.parent
YAML = REPO / "ros2_ws" / "src" / "nokosahen_bringup" / "config" / "nokosahen.yaml"


@unittest.skipIf(bd is None, f"behavior_director import失敗: {tbr._BD_ERR}")
class DirectDriveLimitTests(unittest.TestCase):
    def setUp(self):
        self.helper = tbr.DirectorTickTests("run_ticks")
        self.helper.setUp()
        self.addCleanup(self.helper.tearDown)

    def linear(self, n):
        return [t.linear.x for t in n._cmd_vel_pub.msgs if t.linear.x]

    def test_constant_is_within_shortest_stop_zone(self):
        # 衝突監視の最短の前進停止領域(50cm)は0.6 m/sまで。直進の上限はそれ以下。
        self.assertLessEqual(bd.DIRECT_DRIVE_MAX_MPS, 0.6)
        self.assertGreater(bd.DIRECT_DRIVE_MAX_MPS, 0.0)
        # ファームの減速2.0 m/s^2での停止距離が、最短領域(50cm)より十分短い。
        self.assertLess(bd.DIRECT_DRIVE_MAX_MPS ** 2 / (2 * 2.0), 0.1)

    def test_crowd_approach_clamped(self):
        for configured in (0.75, 1.1, 5.0):
            n = self.helper.make(crowd="calm", target=False)
            n.crowd_scan_drive_mps = configured
            now = n._clock.now()
            n._scan_phase = "approaching"
            n._scan_phase_start_time = now
            n._scan_start_pose = None
            mode = n._crowd_scan_step(now, 0.1)
            self.assertEqual(mode, "CROWD_APPROACH")
            self.assertEqual(self.linear(n), [bd.DIRECT_DRIVE_MAX_MPS], configured)

    def test_crowd_approach_below_limit_unchanged(self):
        n = self.helper.make(crowd="calm", target=False)
        n.crowd_scan_drive_mps = 0.3
        now = n._clock.now()
        n._scan_phase = "approaching"
        n._scan_phase_start_time = now
        n._scan_start_pose = None
        n._crowd_scan_step(now, 0.1)
        self.assertEqual(self.linear(n), [0.3])

    def test_speed_scale_still_applies(self):
        n = self.helper.make(crowd="calm", target=False)
        n._rule_effects = tbr.br.NEUTRAL
        n.crowd_scan_drive_mps = 0.4
        n._scaled = lambda v: v * 0.5  # 方針のspeed_scale 0.5
        now = n._clock.now()
        n._scan_phase = "approaching"
        n._scan_phase_start_time = now
        n._scan_start_pose = None
        n._crowd_scan_step(now, 0.1)
        self.assertEqual(self.linear(n), [0.2])

    def test_approach_fallback_clamped(self):
        n = self.helper.make(crowd="calm", target=False)
        n.enable_direct_cmd_vel = True
        n.approach_speed_mps = 1.1  # ダッシュボードで上げた値(最大1.1)
        self.assertEqual(n._approach_target(None), "idle")
        self.assertEqual(self.linear(n), [bd.DIRECT_DRIVE_MAX_MPS])

    def test_approach_fallback_off_by_default(self):
        n = self.helper.make(crowd="calm", target=False)
        self.assertFalse(n.enable_direct_cmd_vel)
        n._approach_target(None)
        self.assertEqual(self.linear(n), [])

    def test_rotation_not_clamped(self):
        # 旋回(見回し・揺動)は従来どおり。
        n = self.helper.make(crowd="calm", target=False)
        now = n._clock.now()
        n._crowd_scan_step(now, 0.1)
        turns = [t.angular.z for t in n._cmd_vel_pub.msgs if t.angular.z]
        self.assertEqual(turns, [n.crowd_scan_turn_rps])

    def test_yaml_and_default_within_limit(self):
        text = YAML.read_text(encoding="utf-8")
        m = re.search(r"^\s*crowd_scan_drive_mps:\s*([0-9.]+)", text, re.M)
        self.assertIsNotNone(m)
        self.assertLessEqual(float(m.group(1)), bd.DIRECT_DRIVE_MAX_MPS)
        src = (PKG / "nokosahen_core" / "behavior_director.py").read_text(encoding="utf-8")
        m = re.search(r'declare_parameter\("crowd_scan_drive_mps",\s*([0-9.]+)\)', src)
        self.assertIsNotNone(m)
        self.assertLessEqual(float(m.group(1)), bd.DIRECT_DRIVE_MAX_MPS)

    def test_docs_describe_direct_paths(self):
        doc = (REPO / "docs" / "open_items.md").read_text(encoding="utf-8")
        for word in ("CROWD_APPROACH", "DIRECT_DRIVE_MAX_MPS", "collision_monitor", "cmd_vel_smoothed"):
            self.assertIn(word, doc)


if __name__ == "__main__":
    unittest.main(verbosity=1)
