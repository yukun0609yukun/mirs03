"""営業の呼びかけと状況アナウンスの二重発話の防止(B9, 2026-10-23)のテスト。ROS・ロボット不要。
実行: python3 -I ros2_ws/src/nokosahen_core/test/test_sales_speech_suppress.py
"""
import sys
import types
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(TEST_DIR.parent))

import test_behavior_rules as tbr  # noqa: E402  (rclpyスタブ注入とdirectorの組み立てを流用)
from nokosahen_core import announce_events as ae  # noqa: E402

bd = tbr.bd


class _BoolMsg:
    def __init__(self, data):
        self.data = data


class SituationListTests(unittest.TestCase):
    def test_list_matches_announce_side(self):
        # 一覧は announce_events の状況判定(フラグ・文脈)の名前と同じものを使う
        for name in ae.SALES_SUPPRESS_SITUATIONS:
            self.assertTrue(name in ae.FLAGS or name in ae.CONTEXT_KEYS or name == "near_person_stop", name)
        self.assertEqual(set(ae.SALES_SUPPRESS_SITUATIONS), {"survey", "near_person_stop", "mapping", "exploring"})

    def test_blocked_names(self):
        self.assertEqual(ae.sales_speech_blocked({}), [])
        self.assertEqual(ae.sales_speech_blocked({"survey": False, "mapping": False}), [])
        self.assertEqual(ae.sales_speech_blocked({"exploring": True, "survey": True}), ["survey", "exploring"])
        self.assertEqual(ae.sales_speech_blocked({"unrelated": True}), [])


@unittest.skipIf(bd is None, f"behavior_director import失敗: {tbr._BD_ERR}")
class DirectorSuppressTests(unittest.TestCase):
    def setUp(self):
        self.helper = tbr.DirectorTickTests("run_ticks")
        self.helper.setUp()
        self.addCleanup(self.helper.tearDown)

    def make(self):
        n = self.helper.make()
        n._clock.t += 3
        return n

    def test_default_announces_first_time(self):
        n = self.make()
        self.helper.run_ticks(n, 1.0)
        self.assertTrue(n._announce_context_pub.msgs)

    def test_map_building_suppresses(self):
        n = self.make()
        n._map_building_active = True
        self.helper.run_ticks(n, 1.0)
        self.assertEqual(n._announce_pub.msgs, [])
        self.assertEqual(n._announce_context_pub.msgs, [])

    def test_exploring_suppresses_when_not_map_building(self):
        n = self.make()
        n._frontier_enabled = True
        n._last_frontier_status_time = n._clock.now()
        self.assertTrue(n._sales_speech_suppressed())

    def test_near_person_stop_suppresses(self):
        n = self.make()
        n._on_near_person_stop(_BoolMsg(True))
        self.assertTrue(n._sales_speech_suppressed())
        n._clock.t += 5
        # 【2026-11-04変更】一度でも受信して途絶えたら、安全側で近接停止(true)と同じ扱い(検出ノードが落ちたとき止まるように)。
        self.assertTrue(n._sales_speech_suppressed())
        # 途絶の判定を無効にすると(near_person_stale_sec<=0)、従来どおり、配信が途絶えたら失効する
        n.near_person_stale_sec = 0.0
        self.assertFalse(n._sales_speech_suppressed())

    def test_survey_suppresses(self):
        n = self.make()
        n._on_survey_active(_BoolMsg(True))
        self.assertTrue(n._sales_speech_suppressed())

    def test_resumes_after_situation_ends(self):
        n = self.make()
        n._map_building_active = True
        self.helper.run_ticks(n, 1.0)
        self.assertEqual(n._announce_context_pub.msgs, [])
        n._map_building_active = False
        n._clock.t += 3
        self.helper.run_ticks(n, 1.0)
        self.assertTrue(n._announce_context_pub.msgs)

    def test_pr_intro_selected_suppresses_menu_speech(self):
        from nokosahen_core import pr_intro as pri
        n = self.make()
        cfg = pri.merged_config({"enabled": True, "speech_mode": "pr", "parts": [{"enabled": True, "title": "t", "text": "紹介です。", "volume": 1.0}]}, True)
        n._pr_reader = types.SimpleNamespace(config=cfg)
        self.assertTrue(n._sales_speech_suppressed())
        self.helper.run_ticks(n, 1.0)
        self.assertEqual(n._announce_pub.msgs, [])
        self.assertEqual(n._announce_context_pub.msgs, [])
        cfg["speech_mode"] = "menu"  # メニュー紹介を選ぶと、また出る
        self.assertFalse(n._sales_speech_suppressed())
        n._clock.t += 3
        self.helper.run_ticks(n, 1.0)
        self.assertTrue(n._announce_context_pub.msgs)

    def test_pr_intro_selected_suppresses_even_when_param_off(self):
        from nokosahen_core import pr_intro as pri
        n = self.make()
        n._suppress_sales_while_announcing = False
        n._pr_reader = types.SimpleNamespace(config=pri.merged_config({"enabled": True, "speech_mode": "pr", "parts": [{"enabled": True, "title": "t", "text": "紹介です。", "volume": 1.0}]}, True))
        self.assertTrue(n._sales_speech_suppressed())

    def test_param_off_keeps_legacy(self):
        n = self.make()
        n._suppress_sales_while_announcing = False
        n._map_building_active = True
        n._on_survey_active(_BoolMsg(True))
        self.assertFalse(n._sales_speech_suppressed())


if __name__ == "__main__":
    unittest.main()
