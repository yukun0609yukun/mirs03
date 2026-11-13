"""推奨発話ON/OFFの読み戻し(2026-10-13、docs/dashboard_field_sync.md)のうち、director側とダッシュボードの送り手側の単体テスト。

確かめること(rclpy不要・未インストール時はスタブ。ロボット・ROSには一切触れない):
1. behavior_directorが推奨発話の現在値(recommend_enabled)と最終変更時刻(recommend_updated_at)を状態(behavior_command)に載せる
2. directorの受け側(_on_recommend_enabled)は冪等: 同じ値の再送では状態も時刻もA/B記録も変わらない。違う値なら変わる
3. BehaviorDirectorControlController: 設定するまでは何も再送せず、設定後は5秒ごとに同じ値を再送する(director再起動後も反映)。
   指定した値は recommend_requested として残る(画面が「director側の値と一致するか」を照合するのに使う)

実行: python ros2_ws/src/nokosahen_core/test/test_readback_director.py
"""

import json
import sys
import types
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(TEST_DIR.parent))

import test_behavior_rules as tbr  # noqa: E402  (rclpyスタブ注入とdirectorの組み立てを流用)

bd = tbr.bd


def _ensure_dashboard_stubs():
    """tbrが入れるrclpyスタブにはnav_msgs等が無いので、dashboard_commonのimportに足りない分だけ足す(実物があれば何もしない)。"""
    for name in ("nav_msgs", "nav_msgs.msg", "geometry_msgs", "geometry_msgs.msg", "std_msgs", "std_msgs.msg"):
        try:
            __import__(name)
        except Exception:  # noqa: BLE001
            sys.modules[name] = types.ModuleType(name)
    for modname, attrs in (("nav_msgs.msg", ("Odometry",)), ("geometry_msgs.msg", ("PoseStamped", "Twist")),
                           ("std_msgs.msg", ("Bool", "Float32MultiArray", "String", "Empty"))):
        m = sys.modules[modname]
        for a in attrs:
            if not hasattr(m, a):
                setattr(m, a, type(a, (), {"__init__": lambda self, *x, **k: setattr(self, "data", None)}))


_ensure_dashboard_stubs()
try:
    from nokosahen_core.dashboard_common import BehaviorDirectorControlController
    _DC_ERR = None
except Exception as exc:  # pragma: no cover
    BehaviorDirectorControlController = None
    _DC_ERR = exc


class _BoolMsg:
    def __init__(self, data):
        self.data = data


def last_command(n):
    return json.loads(n._command_pub.msgs[-1].data)


@unittest.skipIf(bd is None, f"behavior_director import失敗: {tbr._BD_ERR}")
class DirectorRecommendStateTests(unittest.TestCase):
    def setUp(self):
        self.helper = tbr.DirectorTickTests("run_ticks")
        self.helper.setUp()
        self.addCleanup(self.helper.tearDown)

    def make(self):
        n = self.helper.make(crowd="calm")
        self.records = []
        n._engagement_log = types.SimpleNamespace(
            record_engagement=lambda *a, **k: None,
            record_mode_change=lambda ts, enabled: self.records.append(enabled),
        )
        n._recommend_updated_at = 100.0
        return n

    def test_state_carries_current_value_and_updated_at(self):
        n = self.make()
        self.helper.run_ticks(n, 0.3)
        cmd = last_command(n)
        self.assertIs(cmd["recommend_enabled"], True)
        self.assertEqual(cmd["recommend_updated_at"], 100.0)
        self.assertIn("idle_wiggle_enabled", cmd)   # 既存のキーは不変

    def test_change_is_reflected_in_state(self):
        n = self.make()
        n._on_recommend_enabled(_BoolMsg(False))
        self.helper.run_ticks(n, 0.2)
        cmd = last_command(n)
        self.assertIs(cmd["recommend_enabled"], False)
        self.assertGreater(cmd["recommend_updated_at"], 100.0)
        n._on_recommend_enabled(_BoolMsg(True))
        self.helper.run_ticks(n, 0.2)
        self.assertIs(last_command(n)["recommend_enabled"], True)
        self.assertEqual(self.records, [False, True])

    def test_receiver_is_idempotent(self):
        n = self.make()
        n._on_recommend_enabled(_BoolMsg(False))
        stamp = n._recommend_updated_at
        for _ in range(5):                       # ダッシュボードの5秒ごとの再送
            n._on_recommend_enabled(_BoolMsg(False))
        self.assertEqual(n._recommend_updated_at, stamp)
        self.assertEqual(self.records, [False])  # A/B記録は変化した1回だけ
        n._on_recommend_enabled(_BoolMsg(True))
        n._on_recommend_enabled(_BoolMsg(True))
        self.assertEqual(self.records, [False, True])

    def test_old_fixture_without_timestamp_does_not_break_tick(self):
        n = self.helper.make(crowd="calm")       # _recommend_updated_atを持たない組み立て(既存テストと同じ)
        self.helper.run_ticks(n, 0.2)
        self.assertIsNone(last_command(n)["recommend_updated_at"])


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg.data)


class _Node:
    def __init__(self):
        self.pubs = {}
        self.timers = []

    def create_publisher(self, _t, topic, _q):
        return self.pubs.setdefault(topic, _Pub())

    def create_timer(self, period, cb):
        self.timers.append((period, cb))


@unittest.skipIf(BehaviorDirectorControlController is None, f"dashboard_common import失敗: {_DC_ERR}")
class ControllerRepublishTests(unittest.TestCase):
    TOPIC = "/nokosahen/behavior_director/recommend_enabled"

    def test_nothing_sent_until_user_sets(self):
        n = _Node()
        c = BehaviorDirectorControlController(n)
        self.assertEqual(n.timers[0][0], 5.0)
        n.timers[0][1]()
        self.assertEqual(n.pubs[self.TOPIC].sent, [])
        self.assertIsNone(c.recommend_requested)

    def test_republish_every_tick_after_set(self):
        n = _Node()
        c = BehaviorDirectorControlController(n)
        c.set_recommend_enabled(False)
        n.timers[0][1]()
        n.timers[0][1]()
        self.assertEqual(n.pubs[self.TOPIC].sent, [False, False, False])
        self.assertIs(c.recommend_requested, False)
        c.set_recommend_enabled(True)
        n.timers[0][1]()
        self.assertEqual(n.pubs[self.TOPIC].sent[-2:], [True, True])
        self.assertIs(c.recommend_requested, True)

    def test_idle_wiggle_and_demo_are_not_republished(self):
        n = _Node()
        c = BehaviorDirectorControlController(n)
        c.set_idle_wiggle_enabled(True)
        c.trigger_demo()
        n.timers[0][1]()
        self.assertEqual(n.pubs["/nokosahen/behavior_director/idle_wiggle_enabled"].sent, [True])
        self.assertEqual(n.pubs["/nokosahen/behavior_director/force_demo"].sent, [True])

    def test_node_without_timer_still_works(self):
        class NoTimer:
            def __init__(self):
                self.p = _Pub()

            def create_publisher(self, *_a, **_k):
                return self.p

        n = NoTimer()
        c = BehaviorDirectorControlController(n)
        self.assertEqual(c.set_recommend_enabled(True)[0], True)


if __name__ == "__main__":
    unittest.main()
