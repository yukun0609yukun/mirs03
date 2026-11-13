"""【2026-11-05】Opusレビュー(2026-11-04)の安全項目: 近接停止中の目的地拒否(ルーター/探索)、向く方位の世界固定(crowd_bearing)。

rclpy不要(スタブ)。director側(免除の測定・基準・取り消し等)は test_face_person.py にある。
実行: python -m pytest -q -p no:cacheprovider test/test_review_2026_11_05_safety.py
"""

import math
import sys
import types
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(TEST_DIR.parent))

import test_behavior_rules as tbr  # noqa: E402
import test_explore_approach as tea  # noqa: E402
import test_survey_hold as tsh  # noqa: E402
from nokosahen_core import crowd_bearing as cb  # noqa: E402

router = tsh.router
fe = tea.fe


class _Bool:
    def __init__(self, data):
        self.data = data


class _PathMsg:
    header = types.SimpleNamespace(frame_id="map")
    poses = []


@unittest.skipIf(router is None, "nav2_goal_router import失敗")
class RouterNearPersonTests(unittest.TestCase):
    def make(self):
        helper = tsh.RouterSurveyTests("test_no_survey_goal_passes")
        r = helper.make()
        helper.stub_add(r)
        r._near_plan_cancel_time = -10.0
        r._plan_last_time = -10.0
        return r

    def test_goal_rejected_while_near_person_stop(self):
        r = self.make()
        r._on_near_person_stop(_Bool(True))
        r._on_goal_pose(tsh._pose())
        self.assertEqual(r._client.sent, [])
        self.assertEqual(r.statuses[-1][0], "near_person_blocked")

    def test_goal_passes_without_near_person_stop(self):
        r = self.make()
        r._on_goal_pose(tsh._pose())  # 未受信
        self.assertEqual(len(r._client.sent), 1)
        r._on_near_person_stop(_Bool(False))
        r._on_goal_pose(tsh._pose())
        self.assertEqual(len(r._client.sent), 2)

    def test_goal_passes_again_after_near_person_stop_ends_or_stale(self):
        r = self.make()
        r._on_near_person_stop(_Bool(True))
        r._clock.t += 1.5  # 受信が1秒を超えて途絶: ルーターは止めない(途絶の停止はdirector/arbiter)
        r._on_goal_pose(tsh._pose())
        self.assertEqual(len(r._client.sent), 1)

    def test_manual_click_goal_is_not_blocked(self):
        r = self.make()
        r._on_near_person_stop(_Bool(True))
        r._on_manual_override_flag(_Bool(True))  # 地図クリック追従は manual_override を出す
        r._on_goal_pose(tsh._pose())
        self.assertEqual(len(r._client.sent), 1)
        r._clock.t += 2.0  # 手動の再送が止まったら、また拒否
        r._on_near_person_stop(_Bool(True))
        r._on_goal_pose(tsh._pose())
        self.assertEqual(len(r._client.sent), 1)
        self.assertEqual(r.statuses[-1][0], "near_person_blocked")

    def test_plan_during_near_stop_cancels_again(self):
        r = self.make()
        r._on_near_person_stop(_Bool(True))
        handle = r._goal_handle
        r._on_plan(_PathMsg())
        self.assertEqual(handle.cancelled, 1)
        r._on_plan(_PathMsg())  # 1秒以内の再計画では重ねない
        self.assertEqual(handle.cancelled, 1)

    def test_plan_without_near_stop_does_not_cancel(self):
        r = self.make()
        r._on_plan(_PathMsg())
        self.assertEqual(r._goal_handle.cancelled, 0)


@unittest.skipIf(fe is None, "frontier_explorer import失敗")
class FrontierNearPersonTests(unittest.TestCase):
    def make(self):
        n = object.__new__(fe.FrontierExplorer)
        n._clock = tbr._Clock()
        n.get_clock = lambda: n._clock
        n.get_logger = lambda: tbr._Log()
        n._goal_pub = tbr._Pub()
        n._enabled = True
        n._hardware_estop = n._soft_estop = False
        n._manual_override_raw = False
        n._last_manual_override_time = None
        n._user_cancel = None
        n._latest_map = object()
        n._get_pose = lambda: (1.0, 1.0)
        n._find_frontiers = lambda g, r: [(1.0, 4.0, 10)]
        n._boundary = {"enabled": False}
        n._boundary_rect = None
        n._keepout_zones = []
        n._blacklist = []
        n.blacklist_radius_m = 0.5
        n.blacklist_sec = 120.0
        n.person_avoid_radius_m = 1.0
        n.person_avoid_timeout_sec = 2.0
        n._tracked_person = None
        n._tracked_person_receive_time = None
        n._current_goal = (1.0, 4.0)
        n._goal_sent_time = n._clock.now()
        n._status = "exploring"
        self.statuses = []
        n._publish_status = lambda extra=None: self.statuses.append(extra)
        orig = fe.PoseStamped
        fe.PoseStamped = tbr._Pose
        self.addCleanup(lambda: setattr(fe, "PoseStamped", orig))
        return n

    def test_pauses_and_sends_no_goal_while_near_person_stop(self):
        n = self.make()
        n._on_near_person_stop(_Bool(True))
        n._current_goal = None
        n._tick()
        self.assertEqual(n._goal_pub.msgs, [])
        self.assertEqual(n._status, "paused")
        self.assertEqual(self.statuses[-1], {"reason": "near_person_stop"})

    def test_drops_current_goal_and_resumes_after_stop_clears(self):
        n = self.make()
        n._on_near_person_stop(_Bool(True))
        n._tick()
        self.assertIsNone(n._current_goal)
        n._on_near_person_stop(_Bool(False))
        n._tick()
        self.assertEqual(len(n._goal_pub.msgs), 1)  # 解除後は新しい目的地を出す


class FaceBearingWorldHoldTests(unittest.TestCase):
    @staticmethod
    def cand(b, w=1.0):
        return {"bearing": b, "weight": w, "range": 2.0}

    def test_held_bearing_follows_body_rotation_when_yaw_known(self):
        s = cb.FaceBearingSelector()
        # 現在の扇形(0度付近)に人がいる。次に、離れた扇形(+120度)にだけ候補がある=切替待ち。その間に機体が +30度 回った。
        self.assertAlmostEqual(s.update(0.0, [self.cand(0.0)], yaw=0.0), 0.0, places=6)
        out = s.update(0.2, [self.cand(math.radians(120))], yaw=math.radians(30))
        # 世界の向きを保つので、機体基準では -30度(旋回した分だけ戻る)。古い 0度のままではない。
        self.assertAlmostEqual(out, math.radians(-30), places=6)

    def test_without_yaw_unchanged(self):
        s = cb.FaceBearingSelector()
        s.update(0.0, [self.cand(0.0)])
        out = s.update(0.2, [self.cand(math.radians(120))])
        self.assertAlmostEqual(out, 0.0, places=6)

    def test_in_front_half(self):
        self.assertTrue(cb.in_front_half(math.radians(89)))
        self.assertTrue(cb.in_front_half(math.radians(-90)))
        self.assertFalse(cb.in_front_half(math.radians(120)))
        self.assertFalse(cb.in_front_half(None))


if __name__ == "__main__":
    unittest.main()
