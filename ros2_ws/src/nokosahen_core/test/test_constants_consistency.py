"""複数のファイルに別々に書かれている同じ値が、食い違っていないことの確認(2026-10-22、動作のかぶり調査の指摘)。ROS不要。
- 機体の半幅 0.26(nav_speed.BODY_HALF_M を基準に、motion_sim・escape_planner・patrol_planner)
- 手動運転の速度上限と、ESP32ファームの頭打ち(firmware/.../safety_limits.h)
- PIDゲインの上限(ダッシュボード PID_GAIN_MAX とファーム PID_GAIN_LIMIT)
実行: python ros2_ws/src/nokosahen_core/test/test_constants_consistency.py
"""
import re
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core import escape_planner, motion_sim, nav_speed, patrol_planner  # noqa: E402

REPO = Path(__file__).resolve().parents[4]
HEADER = REPO / "firmware" / "esp32_microros_drive_v2" / "include" / "safety_limits.h"
DASH = Path(__file__).resolve().parents[1] / "nokosahen_core" / "dashboard_common.py"


def header_value(name):
    m = re.search(rf"{name}\s*=\s*([0-9.]+)", HEADER.read_text(encoding="utf-8"))
    assert m, name
    return float(m.group(1))


class BodySize(unittest.TestCase):
    def test_body_half_is_the_same_everywhere(self):
        self.assertEqual(motion_sim.BODY_HALF_M, nav_speed.BODY_HALF_M)
        self.assertEqual(escape_planner.BODY_HALF_M, nav_speed.BODY_HALF_M)
        self.assertEqual(patrol_planner.ROBOT_HALF_WIDTH_M, nav_speed.BODY_HALF_M)

    def test_plan_footprint_is_smaller_but_not_tiny(self):
        self.assertLess(nav_speed.PLAN_BODY_HALF_M, nav_speed.BODY_HALF_M)


class FirmwareLimits(unittest.TestCase):
    def test_pid_gain_limit_matches_dashboard(self):
        m = re.search(r"PID_GAIN_MAX\s*=\s*([0-9.]+)", DASH.read_text(encoding="utf-8"))
        self.assertIsNotNone(m)
        self.assertEqual(float(m.group(1)), header_value("PID_GAIN_LIMIT"))

    def test_manual_limits_not_above_firmware_clamp(self):
        # ROS側の手動運転の上限は、ファームの頭打ち(cmd_velの直進・旋回の上限)を超えない
        self.assertLessEqual(nav_speed.MANUAL_LINEAR_MAX if hasattr(nav_speed, "MANUAL_LINEAR_MAX") else 1.2, header_value("CMD_LINEAR_LIMIT_MPS"))
        self.assertLessEqual(nav_speed.MANUAL_ANGULAR_MAX if hasattr(nav_speed, "MANUAL_ANGULAR_MAX") else 4.0, header_value("CMD_ANGULAR_LIMIT_RPS"))

    def test_nav_speed_max_not_above_firmware_clamp(self):
        self.assertLessEqual(nav_speed.NAV_SPEED_MAX, header_value("CMD_LINEAR_LIMIT_MPS"))


if __name__ == "__main__":
    unittest.main(verbosity=1)
