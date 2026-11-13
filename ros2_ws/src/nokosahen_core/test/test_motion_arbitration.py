"""動きの調停(docs/motion_priority.md)が、コードに入っていることの静的な確認(2026-10-22)。ROS不要。
実行: python ros2_ws/src/nokosahen_core/test/test_motion_arbitration.py
"""
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "nokosahen_core"
WALL = (ROOT / "wall_escape.py").read_text(encoding="utf-8")
ROUTER = (ROOT / "nav2_goal_router.py").read_text(encoding="utf-8")
DOC = (Path(__file__).resolve().parents[4] / "docs" / "motion_priority.md")


class WallEscapeYields(unittest.TestCase):
    def test_blocked_includes_router_rotating(self):
        m = re.search(r"def _blocked_hard\(self\) -> bool:\n(.*?)\n\n", WALL.replace("\r\n", "\n"), re.S)
        self.assertIsNotNone(m)
        for token in ("_wasd", "_soft_estop", "_hard_estop", "_near_person", "_router_rotating"):
            self.assertIn(token, m.group(1))

    def test_proactive_waits_for_router_rotation(self):
        src = WALL.replace("\r\n", "\n")
        self.assertIn("_proactive_due", src)
        self.assertIn("+ 0.4", src)
        # 目的地を受けた時点では、すぐ動き出さない(_start_escapeを直接呼ばない)
        i = src.index("def _on_goal(self, msg)")
        j = src.index("def _on_cancel", i)
        self.assertNotIn("self._start_escape()", src[i:j])

    def test_yields_to_nav2_when_a_plan_exists(self):
        self.assertIn('"/plan"', WALL)
        self.assertIn("Nav2が経路を作れたため任せます", WALL)

    def test_manual_trigger_cancels_nav2_goal_first_and_ignores_own_cancel(self):
        src = WALL.replace("\r\n", "\n")
        i = src.index("def _on_trigger")
        j = src.index("def status_dict", i)
        body = src[i:j]
        self.assertIn("_cancel_pub.publish", body)
        self.assertIn("_self_cancel_until", body)
        self.assertIn("_self_cancel_until", src[src.index("def _on_cancel"):src.index("def _on_plan")])

    def test_commands_only_through_cmd_vel_nav(self):
        self.assertIn('"/cmd_vel_nav"', WALL)
        self.assertNotIn('create_publisher(Twist, "/cmd_vel",', WALL)


class RouterPublishesRotation(unittest.TestCase):
    def test_router_publishes_router_rotating(self):
        self.assertIn('"/nokosahen/router_rotating"', ROUTER)
        self.assertIn("_publish_rotating", ROUTER)
        src = ROUTER.replace("\r\n", "\n")
        i = src.index("def _publish_rotating")
        body = src[i:i + 500]
        for token in ("_spin_handle", "_fallback", "_rotating_hint_until"):
            self.assertIn(token, body)

    def test_router_commands_only_through_cmd_vel_nav(self):
        self.assertIn('create_publisher(Twist, "/cmd_vel_nav"', ROUTER)


class Doc(unittest.TestCase):
    def test_doc_lists_priorities(self):
        t = DOC.read_text(encoding="utf-8")
        for token in ("緊急停止", "手動運転", "衝突監視", "近接停止", "router_rotating", "/plan", "wall_escape"):
            self.assertIn(token, t)


if __name__ == "__main__":
    unittest.main(verbosity=1)
