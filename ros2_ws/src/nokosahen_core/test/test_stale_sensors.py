"""センサー・ノードが途絶えたときに、古い人物・混雑度を使い続けないことのテスト(2026-10-21、全体レビューの指摘)。

- behavior_director: crowd_stateがCROWD_STATE_STALE_SEC秒以上届かなければ{}(不明・人物なし)として扱う
- crowd_monitor / person_target_detector: /scanが途絶えたら、混雑度を"unknown"、人物を出さず、近接停止を安全側(true)にする
rclpy・ROS・ロボットは使わない(スタブ)。実行: python ros2_ws/src/nokosahen_core/test/test_stale_sensors.py
"""
import json
import sys
import time
import types
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(TEST_DIR.parent))

import test_behavior_rules as tbr  # noqa: E402  (rclpyスタブ注入とdirectorの組み立てを流用)

bd = tbr.bd


class _Msg:
    def __init__(self, data):
        self.data = data


@unittest.skipIf(bd is None, f"behavior_director import失敗: {tbr._BD_ERR}")
class DirectorCrowdStateTest(unittest.TestCase):
    def make(self):
        return tbr.make_director() if hasattr(tbr, "make_director") else object.__new__(bd.BehaviorDirector)

    def test_fresh_state_is_used(self):
        d = object.__new__(bd.BehaviorDirector)
        d._crowd_state = {}
        d._crowd_state_mono = None
        d._on_crowd_state(_Msg(json.dumps({"level": "calm", "person_target": {"x": 1.0}})))
        self.assertEqual(d._fresh_crowd_state().get("level"), "calm")
        self.assertIsNotNone(d._fresh_crowd_state().get("person_target"))

    def test_stale_state_is_ignored(self):
        d = object.__new__(bd.BehaviorDirector)
        d._crowd_state = {}
        d._crowd_state_mono = None
        d._on_crowd_state(_Msg(json.dumps({"level": "calm", "person_target": {"x": 1.0}})))
        d._crowd_state_mono = time.monotonic() - (bd.CROWD_STATE_STALE_SEC + 1.0)  # crowd_monitorが落ちて、3秒以上届かない
        self.assertEqual(d._fresh_crowd_state(), {})
        self.assertEqual(d._fresh_crowd_state().get("level", "unknown"), "unknown")
        self.assertIsNone(d._fresh_crowd_state().get("person_target"))

    def test_state_without_receive_time_keeps_old_behavior(self):
        d = object.__new__(bd.BehaviorDirector)
        d._crowd_state = {"level": "crowded"}  # 受信を通っていない(未受信の既定・単体テストの直接代入)
        d._crowd_state_mono = None
        self.assertEqual(d._fresh_crowd_state(), {"level": "crowded"})

    def test_receiving_again_recovers(self):
        d = object.__new__(bd.BehaviorDirector)
        d._crowd_state = {"level": "calm"}
        d._crowd_state_mono = time.monotonic() - 100.0
        self.assertEqual(d._fresh_crowd_state(), {})
        d._on_crowd_state(_Msg(json.dumps({"level": "crowded"})))
        self.assertEqual(d._fresh_crowd_state().get("level"), "crowded")


class ScanStaleStaticTest(unittest.TestCase):
    """crowd_monitor / person_target_detector は rclpy・numpy・cv2 に依存するので、ソースの静的な確認だけを行う。"""

    def read(self, name):
        return (TEST_DIR.parent / "nokosahen_core" / name).read_text(encoding="utf-8")

    def test_person_target_detector_handles_stale_scan(self):
        t = self.read("person_target_detector.py")
        self.assertIn("SCAN_STALE_SEC = 1.0", t)
        self.assertIn("self._last_scan_mono = time.monotonic()", t)
        self.assertIn("def _scan_is_stale", t)
        # 途絶えたら、クラスタを空にして、近接停止はtrue(安全側)を配信する
        self.assertIn("self._latest_clusters = []", t)
        i = t.index("if self._scan_is_stale():", t.index("def _publish_near_person_stop"))
        self.assertIn("msg.data = True", t[i:i + 700])
        # 【2026-11-05】一度も受信していなくても、起動から猶予を超えたら stale(require_scan)。判定は scan_geometry.scan_is_stale
        self.assertIn("scan_is_stale(", t)
        self.assertIn("SCAN_STARTUP_GRACE_SEC", t)

    def test_crowd_monitor_handles_stale_scan(self):
        t = self.read("crowd_monitor.py")
        self.assertIn("SCAN_STALE_SEC = 1.5", t)
        self.assertIn('"level": "unknown" if stale else self._level', t)
        self.assertIn('"person_target": None if stale else self._current_person_target()', t)
        self.assertIn("self._last_scan_mono = time.monotonic()", t)

    def test_scan_stale_function_semantics(self):
        # _scan_is_stale の本体だけを取り出して、時刻の扱いを確かめる
        for name in ("person_target_detector.py", "crowd_monitor.py"):
            t = self.read(name)
            start = t.index("    def _scan_is_stale(self)")
            end = t.index("\n    def ", start + 10)
            from nokosahen_core.scan_geometry import scan_is_stale
            ns = {"time": time, "SCAN_STALE_SEC": 1.0, "scan_is_stale": scan_is_stale, "SCAN_STARTUP_GRACE_SEC": 3.0}
            exec("class X:\n" + t[start:end], ns)  # noqa: S102 - 自分のリポジトリの短い関数だけ
            x = ns["X"]()
            x._last_scan_mono = None
            self.assertFalse(x._scan_is_stale(), name)  # 一度も受信していない(起動直後=猶予内)
            if name == "person_target_detector.py":
                x._started_mono = time.monotonic() - 10.0
                x.require_scan = True
                self.assertTrue(x._scan_is_stale(), name)  # 起動から猶予を超えても未受信=途絶扱い
                x.require_scan = False
                self.assertFalse(x._scan_is_stale(), name)  # LiDARを使わない構成
            x._last_scan_mono = time.monotonic()
            self.assertFalse(x._scan_is_stale(), name)  # 受信直後
            x._last_scan_mono = time.monotonic() - 5.0
            self.assertTrue(x._scan_is_stale(), name)  # 5秒途絶え


if __name__ == "__main__":
    unittest.main(verbosity=1)
