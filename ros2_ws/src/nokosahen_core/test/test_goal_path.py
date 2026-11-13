"""目的地(/goal_pose)の経路まわりの静的テスト(2026-10-19)。ROS・実機は使わない。

実機ログで、Nav2のbt_navigatorが/goal_poseを直接購読して、nav2_goal_router(緊急停止・アンケート・範囲の確認、背後の
目的地の先回し旋回)を素通りして走り出していた疑いが見つかった(ルーターの旋回判断の7ms後にbt_navigatorが走り出した)。
Nav2側の購読を付け替える(SetRemap)ことと、ルーターの先回し旋回の設定を、静的に確認する。
"""
import re
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
LAUNCH = REPO / "ros2_ws" / "src" / "nokosahen_bringup" / "launch"
ROUTER = REPO / "ros2_ws" / "src" / "nokosahen_core" / "nokosahen_core" / "nav2_goal_router.py"


class Bt_navigatorGoalTopicTest(unittest.TestCase):
    def test_nav2_goal_pose_is_remapped_in_both_nav2_launches(self):
        for name in ("mapping_mode.launch.py", "localization_mode.launch.py"):
            t = (LAUNCH / name).read_text(encoding="utf-8")
            self.assertIn("SetRemap", t, name)
            self.assertRegex(t, r'SetRemap\(src="goal_pose", dst=NAV2_UNUSED_GOAL_TOPIC\)', name)
            self.assertIn('NAV2_UNUSED_GOAL_TOPIC = "/nokosahen/nav2_unused_goal_pose"', t, name)
            # 付け替えは navigation_launch(bt_navigatorを含む)のincludeと同じGroupActionの中
            m = re.search(r"GroupAction\(\s*\[(.*?)\]\s*\)", t, re.S)
            self.assertIsNotNone(m, name)
            self.assertIn("SetRemap", m.group(1), name)
            self.assertIn("navigation_launch.py", m.group(1), name)

    def test_nobody_publishes_the_unused_topic_and_router_still_uses_goal_pose(self):
        r = ROUTER.read_text(encoding="utf-8")
        self.assertIn('create_subscription(PoseStamped, "/goal_pose"', r)  # ルーターは、これまでどおり/goal_poseだけを受ける
        for p in (REPO / "ros2_ws" / "src").rglob("*.py"):
            if p.name.startswith("test_") or "nav2_unused_goal_pose" not in p.read_text(encoding="utf-8", errors="ignore"):
                continue
            self.assertTrue(p.parent.name == "launch", f"{p}: 付け替え先のトピックは起動ファイル以外で使わない")


class PreRotateTest(unittest.TestCase):
    def test_min_distance_default_is_small(self):
        r = ROUTER.read_text(encoding="utf-8")
        self.assertRegex(r, r'declare_parameter\("pre_rotate_min_distance", 0\.3\)')

    def test_stale_tf_is_logged_not_silent(self):
        r = ROUTER.read_text(encoding="utf-8")
        self.assertIn("先回し旋回の判断を省きます", r)
        self.assertGreaterEqual(r.count("先回し旋回の判断を省きます"), 2)  # 変換が取れない/古い、の両方

    def test_pre_rotation_angle_pure_function(self):
        import importlib.util
        import math
        import sys
        import types
        # rclpy等に依存せず、純関数だけを取り出して確認する
        src = ROUTER.read_text(encoding="utf-8")
        start = src.index("def wrap_angle")
        end = src.index("class Nav2GoalRouter")
        ns = {"math": math, "Optional": __import__("typing").Optional}
        exec(src[start:end], ns)  # noqa: S102 - 自分のリポジトリの純関数だけ
        f = ns["pre_rotation_angle"]
        # 真後ろ(180°)で、0.5 m離れた目的地: 0.3 m未満ではないので旋回する
        self.assertAlmostEqual(abs(f(0.0, 0.0, 0.0, -0.5, 0.0, 100.0, 0.3)), math.pi, places=6)
        # 0.2 mの背後は省く / 前方は旋回しない / 側方(90°)は閾値100°未満なので旋回しない
        self.assertIsNone(f(0.0, 0.0, 0.0, -0.2, 0.0, 100.0, 0.3))
        self.assertIsNone(f(0.0, 0.0, 0.0, 1.0, 0.0, 100.0, 0.3))
        self.assertIsNone(f(0.0, 0.0, 0.0, 0.0, 1.0, 100.0, 0.3))
        # 137°は旋回する(実機ログの例)
        d = f(0.19, 0.54, 0.0, 1.01 - 2 * 0.82, 0.30 + 0.5 * 0.6, 100.0, 0.3)
        self.assertIsNotNone(d)


if __name__ == "__main__":
    unittest.main(verbosity=1)
