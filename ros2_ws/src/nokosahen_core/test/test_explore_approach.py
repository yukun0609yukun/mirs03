"""探索中のアンケート停止の無効化と、走行可能範囲を制限したときの人への接近(射影した立ち位置で人の方を向く)の単体テスト。

rclpy不要(未インストール時はスタブ注入)。ロボット・ROSには一切触れない(cmd_vel/ゴールはスタブのpublisherに溜まるだけ)。
実行: python ros2_ws/src/nokosahen_core/test/test_explore_approach.py
"""

import json
import math
import sys
import types
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(TEST_DIR.parent))

import test_behavior_rules as tbr  # noqa: E402  (rclpyスタブ注入とdirectorの組み立てを流用)
import test_survey_hold as tsh  # noqa: E402  (nav2_goal_routerの組み立てを流用)
from nokosahen_core import map_orientation as mo  # noqa: E402

bd = tbr.bd
router = tsh.router
try:
    import nav_msgs.msg  # noqa: F401
except Exception:  # noqa: BLE001  (ROS無しのPC用スタブ。frontier_explorerのimportを通すだけ)
    _nm = types.ModuleType("nav_msgs")
    _nmm = types.ModuleType("nav_msgs.msg")
    _nmm.OccupancyGrid = object
    _nm.msg = _nmm
    sys.modules["nav_msgs"], sys.modules["nav_msgs.msg"] = _nm, _nmm
try:
    from nokosahen_core import frontier_explorer as fe  # noqa: E402
    _FE_ERR = None
except Exception as exc:  # pragma: no cover
    fe = None
    _FE_ERR = exc


class _BoolMsg:
    def __init__(self, data):
        self.data = data


class _StrMsg:
    def __init__(self, payload):
        self.data = json.dumps(payload)


def nonzero_twists(n):
    return [t for t in n._cmd_vel_pub.msgs if t.linear.x != 0.0 or t.angular.z != 0.0]


def cancel_count(n):
    return sum(1 for m in n._cancel_goal_pub.msgs if m.data)


def yaw_of(goal):
    o = goal.pose.orientation
    return 2.0 * math.atan2(o.z, o.w)


def angdiff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


@unittest.skipIf(bd is None, f"behavior_director import失敗: {tbr._BD_ERR}")
class DirectorExploreSurveyTests(unittest.TestCase):
    """探索(地図作成)中は、アンケート回答中でも止まらない。通常の自律走行では従来どおり止まる。"""

    def setUp(self):
        self.helper = tbr.DirectorTickTests("run_ticks")
        self.helper.setUp()
        self.addCleanup(self.helper.tearDown)

    def go(self, n, seconds, survey=True, refresh=None):
        modes = []
        for _ in range(int(round(seconds / 0.1))):
            if survey is not None and int(round(n._clock.t * 10)) % 2 == 0:
                n._on_survey_active(_BoolMsg(survey))
            if refresh:
                refresh(n)
            modes += self.helper.run_ticks(n, 0.1)
        return modes

    @staticmethod
    def frontier_on(n):
        n._on_frontier_status(_StrMsg({"enabled": True, "status": "exploring"}))

    def test_normal_driving_still_stops_for_survey(self):
        n = self.helper.make(crowd="calm")
        n._clock.t += 10
        self.assertEqual(set(self.go(n, 2.0)), {"SURVEY_ACTIVE_HOLD"})
        self.assertGreater(cancel_count(n), 0)
        self.assertEqual(nonzero_twists(n), [])
        self.assertEqual(n._goal_pub.msgs, [])

    def test_exploring_ignores_survey(self):
        n = self.helper.make(crowd="calm")
        n._clock.t += 10
        self.go(n, 0.1)
        self.assertEqual(self.go(n, 0.1)[-1], "SURVEY_ACTIVE_HOLD")  # 探索前は止まる(前提の確認)
        n = self.helper.make(crowd="calm")
        n._clock.t += 10
        modes = self.go(n, 3.0, refresh=self.frontier_on)
        self.assertEqual(set(modes), {"MAPPING_ACTIVE_HOLD"})
        # directorは走行指令も、探索のNav2ゴールの取り消しも出さない(アンケートで探索を止めない)
        self.assertEqual(cancel_count(n), 0)
        self.assertEqual(nonzero_twists(n), [])
        self.assertEqual(n._goal_pub.msgs, [])

    def test_map_building_also_ignores_survey(self):
        n = self.helper.make(crowd="calm")
        n._clock.t += 10
        n._map_building_active = True
        self.assertEqual(set(self.go(n, 2.0)), {"MAPPING_ACTIVE_HOLD"})
        self.assertEqual(cancel_count(n), 0)

    def test_survey_resumes_when_exploring_ends(self):
        n = self.helper.make(crowd="calm")
        n._clock.t += 10
        self.assertEqual(set(self.go(n, 1.0, refresh=self.frontier_on)), {"MAPPING_ACTIVE_HOLD"})
        # 探索の状態配信が途絶えて失効(10秒)したら、回答中なので従来どおり止まる
        modes = self.go(n, 12.0)
        self.assertEqual(modes[-1], "SURVEY_ACTIVE_HOLD")

    def test_safety_still_wins_during_exploring_and_survey(self):
        # 探索中+アンケート中でも、ESTOP・手動・近接・カメラ・範囲外・立入禁止の優先順位は不変で、走行指令は出ない
        for expected, setter in self.helper._safety_cases():
            n = self.helper.make(crowd="calm")
            n._clock.t += 10
            setter(n)
            refresh = {
                "MANUAL_OVERRIDE": lambda m, s=setter: s(m),
                "NEAR_PERSON_HOLD": lambda m, s=setter: s(m),
                "CAMERA_OBSTACLE_HOLD": lambda m, s=setter: s(m),
            }.get(expected)

            def both(m, r=refresh):
                self.frontier_on(m)
                if r:
                    r(m)

            modes = self.go(n, 1.0, refresh=both)
            self.assertEqual(set(modes), {expected}, expected)
            self.assertEqual(nonzero_twists(n), [], expected)
            self.assertEqual(n._goal_pub.msgs, [], expected)

    def test_survey_stays_silent_for_announce_even_when_exploring(self):
        n = self.helper.make(crowd="calm")
        n._clock.t += 10
        self.go(n, 3.0, refresh=self.frontier_on)
        self.assertEqual(n._announce_pub.msgs, [])  # 回答中の呼びかけは従来どおりしない


@unittest.skipIf(router is None, f"nav2_goal_router import失敗: {tsh._ROUTER_ERR}")
class RouterExploreSurveyTests(unittest.TestCase):
    make = tsh.RouterSurveyTests.make
    stub_add = staticmethod(tsh.RouterSurveyTests.stub_add)

    def explore(self, r, enabled=True):
        r._on_frontier_status(_StrMsg({"enabled": enabled}))

    def test_default_not_exploring(self):
        r = self.make()
        self.assertFalse(r._mapping_or_exploring)

    def test_goal_passes_during_survey_when_exploring(self):
        r = self.make()
        self.stub_add(r)
        self.explore(r)
        r._on_survey_active(_BoolMsg(True))
        r._on_goal_pose(tsh._pose())
        self.assertEqual(len(r._client.sent), 1)
        self.assertEqual(r.statuses, [])

    def test_running_goal_not_cancelled_by_survey_when_exploring(self):
        r = self.make()
        self.stub_add(r)
        self.explore(r)
        handle = r._goal_handle
        r._on_survey_active(_BoolMsg(True))
        self.assertEqual(handle.cancelled, 0)
        self.assertEqual(r._cmd_vel_pub.msgs, [])
        self.assertIsNone(r._stop_burst_until)

    def test_map_building_counts_as_exploring(self):
        r = self.make()
        self.stub_add(r)
        r._on_map_building_active(_BoolMsg(True))
        r._on_survey_active(_BoolMsg(True))
        r._on_goal_pose(tsh._pose())
        self.assertEqual(len(r._client.sent), 1)

    def test_normal_driving_survey_still_blocks(self):
        r = self.make()
        self.stub_add(r)
        self.explore(r, enabled=False)  # 探索OFF
        handle = r._goal_handle
        r._on_survey_active(_BoolMsg(True))
        self.assertEqual(handle.cancelled, 1)
        r._on_goal_pose(tsh._pose())
        self.assertEqual(r._client.sent, [])
        self.assertEqual(r.statuses[-1][0], "survey_blocked")

    def test_exploring_status_expires(self):
        r = self.make()
        self.stub_add(r)
        self.explore(r)
        r._on_survey_active(_BoolMsg(True))
        r._clock.t += 11.0  # 探索の状態配信が途絶えて失効
        r._on_survey_active(_BoolMsg(True))
        r._on_goal_pose(tsh._pose())
        self.assertEqual(r._client.sent, [])
        self.assertEqual(r.statuses[-1][0], "survey_blocked")

    def test_other_safety_unchanged_when_exploring(self):
        r = self.make()
        self.stub_add(r)
        self.explore(r)
        r._on_survey_active(_BoolMsg(True))
        r._hardware_estop = True
        r._on_goal_pose(tsh._pose())
        self.assertEqual(r.statuses[-1][0], "estop_blocked")
        r._hardware_estop = False
        r._boundary_rect = mo.make_rect(0.0, 0.0, 2.0, 2.0, 0.0)
        r._on_goal_pose(tsh._pose(x=5.0))
        self.assertEqual(r.statuses[-1][0], "outside_boundary")


@unittest.skipIf(fe is None, f"frontier_explorer import失敗: {_FE_ERR}")
class FrontierGoalTests(unittest.TestCase):
    def test_goal_has_travel_heading_and_no_cmd_vel(self):
        orig = fe.PoseStamped
        fe.PoseStamped = tbr._Pose
        self.addCleanup(lambda: setattr(fe, "PoseStamped", orig))
        n = object.__new__(fe.FrontierExplorer)
        n._clock = tbr._Clock()
        n.get_clock = lambda: n._clock
        n.get_logger = lambda: tbr._Log()
        n._goal_pub = tbr._Pub()
        n._enabled = True
        n._hardware_estop = n._soft_estop = False
        n._manual_override_raw = False
        n._last_manual_override_time = None
        n._latest_map = object()
        n._get_pose = lambda: (1.0, 1.0)
        n._find_frontiers = lambda g, r: [(1.0, 4.0, 10)]  # 真上(+y)のフロンティア
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
        n._current_goal = None
        n._goal_sent_time = None
        n._status = "exploring"
        n._publish_status = lambda extra=None: None
        n._tick()
        self.assertEqual(len(n._goal_pub.msgs), 1)
        g = n._goal_pub.msgs[0]
        self.assertEqual((g.pose.position.x, g.pose.position.y), (1.0, 4.0))
        self.assertAlmostEqual(yaw_of(g), math.pi / 2, places=6)  # 進行方向(+y)。以前はyaw 0固定
        # 探索ノードは/cmd_velのpublisherを持たない(速度はNav2=nav_max_speedだけ)
        src = (TEST_DIR.parent / "nokosahen_core" / "frontier_explorer.py").read_text(encoding="utf-8")
        self.assertNotIn("/cmd_vel", src.replace("/cmd_velを出さず", ""))
        self.assertNotIn("Twist", src)


@unittest.skipIf(fe is None, f"frontier_explorer import失敗: {_FE_ERR}")
class FrontierWallSafeTargetTests(unittest.TestCase):
    """目標を壁から機体端30 cm以上離れた点へ寄せる(patrol_planner)。無効/地図なしは従来どおり。"""

    @staticmethod
    def fake_map():
        w, h, res = 100, 80, 0.05
        data = [0 if (2 <= c < w - 2 and 2 <= r < h - 2) else 100 for r in range(h) for c in range(w)]
        return types.SimpleNamespace(
            info=types.SimpleNamespace(width=w, height=h, resolution=res,
                                       origin=types.SimpleNamespace(position=types.SimpleNamespace(x=0.0, y=0.0))),
            data=data)

    def make(self, margin=0.30):
        n = object.__new__(fe.FrontierExplorer)
        n._latest_map = self.fake_map()
        n._boundary = {"enabled": False}
        n._boundary_rect = None
        n.goal_wall_margin_m = margin
        return n

    def test_snaps_away_from_wall(self):
        t = self.make()._wall_safe_target(0.3, 2.0)   # 壁の面(x=0.10)から機体端で-0.06 m
        self.assertIsNotNone(t)
        self.assertGreaterEqual(t[0] - 0.10 - 0.26, 0.30 - 1e-9)

    def test_center_stays_and_disabled_or_no_map_is_none(self):
        n = self.make()
        t = n._wall_safe_target(2.5, 2.0)
        self.assertEqual(t, (2.5, 2.0))
        n.goal_wall_margin_m = 0.0
        self.assertIsNone(n._wall_safe_target(0.3, 2.0))
        n.goal_wall_margin_m = 0.30
        n._latest_map = None
        self.assertIsNone(n._wall_safe_target(0.3, 2.0))


# 範囲: 原点中心 4m x 4m (x,y とも -2..2)
RECT = mo.make_rect(0.0, 0.0, 4.0, 4.0, 0.0)


@unittest.skipIf(bd is None, f"behavior_director import失敗: {tbr._BD_ERR}")
class BoundaryApproachTests(unittest.TestCase):
    """範囲を制限したとき、人が範囲外/境界付近でも「範囲の中で止まり、人の方を向き、話す」。"""

    def setUp(self):
        self.helper = tbr.DirectorTickTests("run_ticks")
        self.helper.setUp()
        self.addCleanup(self.helper.tearDown)

    def make(self, person, boundary=True, pose=(0.0, 0.0, 0.0)):
        """personはmap座標系の人の位置(固定)。directorへ渡すtargetは機体基準(laser)なので、機体の位置・向きから換算する。"""
        n = self.helper.make(crowd="calm", target=False)
        n._person_world = person
        if boundary:
            n._boundary = mo.boundary_state(RECT, True)
        self.set_pose(n, *pose)
        n._clock.t += 10  # 接近ゴールの再送間隔を越える
        return n

    @staticmethod
    def set_pose(n, x, y, yaw):
        n._get_pose = lambda: {"x": x, "y": y, "yaw": yaw}
        dx, dy = n._person_world[0] - x, n._person_world[1] - y
        n._crowd_state = {
            "level": "calm",
            "person_target": {
                "x": dx * math.cos(yaw) + dy * math.sin(yaw),
                "y": -dx * math.sin(yaw) + dy * math.cos(yaw),
                "frame_id": "laser",
            },
        }

    def tick(self, n, seconds=0.1):
        return self.helper.run_ticks(n, seconds)

    # (a) 範囲外の人 -> 接近ゴールは範囲の内側へ射影される
    def test_a_goal_is_projected_inside_range(self):
        n = self.make(person=(5.0, 3.0))
        self.assertEqual(self.tick(n)[-1], "APPROACH_PERSON")
        g = n._goal_pub.msgs[-1]
        self.assertEqual(g.header.frame_id, "map")
        self.assertTrue(mo.point_in_rect(g.pose.position.x, g.pose.position.y, RECT))
        # 縁(x=2)から、機体の旋回に必要なぶん(GOAL_MARGIN_M)内側までしか行かない(2026-11-02: 到着時の旋回で角が出ないように)
        from nokosahen_core.boundary_motion_guard import GOAL_MARGIN_M
        self.assertAlmostEqual(g.pose.position.x, 2.0 - GOAL_MARGIN_M, places=6)
        self.assertLess(g.pose.position.y, 2.0 + 1e-9)

    # (c) 射影した目的地の向きは、人の方位(以前は送信時の機体の向きだった)
    def test_c_goal_yaw_faces_person(self):
        n = self.make(person=(5.0, 3.0), pose=(0.0, 0.0, math.radians(-100)))
        self.tick(n)
        g = n._goal_pub.msgs[-1]
        want = math.atan2(3.0 - g.pose.position.y, 5.0 - g.pose.position.x)
        self.assertAlmostEqual(angdiff(yaw_of(g), want), 0.0, places=6)
        self.assertGreater(abs(angdiff(yaw_of(g), math.radians(-100))), 0.5)  # 機体の向きのままではない

    # (b)->(c)->(d) 縁に着いて人と違う向き: その場で向く目的地を送り続け、向いたら到着、発話
    def test_b_to_d_arrive_turn_then_dwell_and_speak(self):
        n = self.make(person=(5.0, 3.0))
        self.tick(n)
        stand = (n._goal_pub.msgs[-1].pose.position.x, n._goal_pub.msgs[-1].pose.position.y)
        # 縁に着いたが、人と逆向き(yaw=180度)
        self.set_pose(n, stand[0], stand[1], math.pi)
        n._clock.t += 3  # 再送間隔
        before = len(n._goal_pub.msgs)
        self.assertEqual(self.tick(n)[-1], "APPROACH_PERSON")
        self.assertEqual(len(n._goal_pub.msgs), before + 1)
        g = n._goal_pub.msgs[-1]
        self.assertAlmostEqual(g.pose.position.x, stand[0], places=6)  # 位置は立ち位置のまま(その場で旋回)
        self.assertAlmostEqual(g.pose.position.y, stand[1], places=6)
        want = math.atan2(3.0 - stand[1], 5.0 - stand[0])
        self.assertAlmostEqual(angdiff(yaw_of(g), want), 0.0, places=6)
        # 人の方を向いた -> 到着。停止して(cmd_velゼロ)、ホームへ戻らず滞在(dwell)する
        self.set_pose(n, stand[0], stand[1], want + 0.1)
        n._arrived_at_time = None
        n._clock.t += 3  # 直前の旋回ゴール送信から再送間隔が過ぎた後(送信直後の2秒は従来どおり「移動中」扱い)
        modes = self.tick(n, 1.0)
        self.assertEqual(set(modes), {"APPROACH_ARRIVED"})
        self.assertEqual(nonzero_twists(n), [])
        # 発話は既存の発話間隔(60秒)の仕組みでそのまま出る(最初の1回は即時)
        self.assertTrue(n._announce_pub.msgs)
        # 滞在時間(approach_dwell_sec=6s)を過ぎると接客1件完了としてホームへ帰還を始める
        modes = self.tick(n, 7.0)
        self.assertIn("RETURN_HOME", modes)

    def test_far_from_stand_point_keeps_navigating(self):
        n = self.make(person=(5.0, 3.0), pose=(0.0, 0.0, 0.0))
        modes = self.helper.run_ticks(n, 3.0)
        self.assertEqual(set(modes), {"APPROACH_PERSON"})

    def test_gives_up_turning_after_timeout_and_still_arrives(self):
        n = self.make(person=(5.0, 3.0))
        self.tick(n)
        stand = (n._goal_pub.msgs[-1].pose.position.x, n._goal_pub.msgs[-1].pose.position.y)
        self.set_pose(n, stand[0], stand[1], math.pi)  # 旋回できない(向きが変わらない)
        modes = self.helper.run_ticks(n, 10.0)
        self.assertIn("APPROACH_PERSON", modes)
        self.assertEqual(modes[-1], "APPROACH_ARRIVED")  # 8秒で向きを諦めて到着扱い(無限に待たない)

    # 範囲外の人が会話距離(standoff)以内にいる: 動かず、その場で向きを変える
    def test_close_person_outside_range_turns_in_place(self):
        # 機体は縁(x=2)に居て、人は範囲の外(x=2.4)、standoff(0.6)以内。機体は-x向き(人と逆)
        n = self.make(person=(2.4, 0.0), pose=(2.0, 0.0, math.pi))
        self.assertEqual(self.tick(n)[-1], "APPROACH_PERSON")
        g = n._goal_pub.msgs[-1]
        self.assertAlmostEqual(g.pose.position.x, 2.0, places=6)
        self.assertAlmostEqual(angdiff(yaw_of(g), 0.0), 0.0, places=6)  # +x(人の方)を向く
        self.set_pose(n, 2.0, 0.0, 0.05)
        self.assertEqual(set(self.tick(n, 0.5)), {"APPROACH_ARRIVED"})

    # 範囲内の人は従来どおり(範囲の有無で動作が変わらない)
    def test_person_inside_range_unchanged(self):
        a = self.make(person=(1.0, 0.5), boundary=True)
        b = self.make(person=(1.0, 0.5), boundary=False)
        self.tick(a)
        self.tick(b)
        ga, gb = a._goal_pub.msgs[-1], b._goal_pub.msgs[-1]
        self.assertEqual(ga.header.frame_id, "laser")
        self.assertEqual((ga.header.frame_id, ga.pose.position.x, ga.pose.position.y, yaw_of(ga)),
                         (gb.header.frame_id, gb.pose.position.x, gb.pose.position.y, yaw_of(gb)))

    def test_no_boundary_far_person_unchanged(self):
        n = self.make(person=(5.0, 3.0), boundary=False)
        self.tick(n)
        g = n._goal_pub.msgs[-1]
        self.assertEqual(g.header.frame_id, "laser")  # 範囲なし: 人の近くへ(機体基準のまま)
        d = math.hypot(5.0, 3.0)
        self.assertAlmostEqual(g.pose.position.x, 5.0 * (d - 0.6) / d, places=6)

    def test_close_person_no_boundary_arrives_without_turn_goal(self):
        n = self.make(person=(2.4, 0.0), boundary=False, pose=(2.0, 0.0, math.pi))
        self.assertEqual(self.tick(n)[-1], "APPROACH_ARRIVED")
        self.assertEqual(n._goal_pub.msgs, [])  # 従来動作: 向き直しのゴールは出さない

    # 立入禁止エリアは射影でも避けられない -> 旋回の目的地も送らない
    def test_stand_point_in_keepout_sends_nothing(self):
        n = self.make(person=(5.0, 3.0))
        n._keepout_zones = [{"min_x": 1.5, "max_x": 2.5, "min_y": 1.0, "max_y": 3.0}]
        self.tick(n)
        n._goal_pub.msgs.clear()
        self.set_pose(n, 2.0, 1.2, math.pi)
        n._clock.t += 3
        self.tick(n)
        self.assertEqual(n._goal_pub.msgs, [])

    # 安全は不変: 範囲外へ出ていれば止まる
    def test_out_of_bounds_still_holds(self):
        n = self.make(person=(5.0, 3.0), pose=(5.0, 5.0, 0.0))
        self.assertEqual(set(self.tick(n, 1.0)), {"OUT_OF_BOUNDS_HOLD"})
        self.assertEqual(n._goal_pub.msgs, [])


if __name__ == "__main__":
    unittest.main()
