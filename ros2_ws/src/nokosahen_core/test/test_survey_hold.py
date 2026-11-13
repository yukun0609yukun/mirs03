"""アンケート回答中の停止(behavior_directorのSURVEY_ACTIVE_HOLD / nav2_goal_routerのゴール拒否)の単体テスト。

rclpy不要(未インストール時はスタブ注入)。ロボット・ROSには一切触れない(cmd_vel/ゴールは
スタブのpublisherに溜まるだけ)。
実行: python ros2_ws/src/nokosahen_core/test/test_survey_hold.py
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

for _name, _attrs in (("rclpy.action", ("ActionClient",)), ("rclpy.duration", ("Duration",))):
    if _name not in sys.modules:
        try:
            __import__(_name)
        except Exception:  # noqa: BLE001
            _m = types.ModuleType(_name)
            for _a in _attrs:
                setattr(_m, _a, object)
            sys.modules[_name] = _m

bd = tbr.bd
try:
    from nokosahen_core import nav2_goal_router as router  # noqa: E402
    _ROUTER_ERR = None
except Exception as exc:  # pragma: no cover
    router = None
    _ROUTER_ERR = exc


class _BoolMsg:
    def __init__(self, data):
        self.data = data


def nonzero_twists(n):
    return [t for t in n._cmd_vel_pub.msgs if t.linear.x != 0.0 or t.angular.z != 0.0]


def cancel_count(n):
    return sum(1 for m in n._cancel_goal_pub.msgs if m.data)


@unittest.skipIf(bd is None, f"behavior_director import失敗: {tbr._BD_ERR}")
class DirectorSurveyHoldTests(unittest.TestCase):
    def setUp(self):
        self.helper = tbr.DirectorTickTests("run_ticks")
        self.helper.setUp()
        self.addCleanup(self.helper.tearDown)

    def make(self, **kw):
        return self.helper.make(**kw)

    def survey(self, n, value=True):
        n._on_survey_active(_BoolMsg(value))

    def go(self, n, seconds, survey=True, refresh=None):
        """5Hzのsurvey_active配信を模して回す(surveyがNoneなら配信しない)。"""
        modes = []
        for _ in range(int(round(seconds / 0.1))):
            if survey is not None and int(round(n._clock.t * 10)) % 2 == 0:
                self.survey(n, survey)
            if refresh:
                refresh(n)
            modes += self.helper.run_ticks(n, 0.1)
        return modes

    # --- 基本: 止まる ---
    def test_hold_mode_no_motion_cancels_goal_and_silent(self):
        n = self.make(crowd="calm")
        n._clock.t += 10
        modes = self.go(n, 3.0)
        self.assertEqual(set(modes), {"SURVEY_ACTIVE_HOLD"})
        self.assertEqual(nonzero_twists(n), [])
        self.assertEqual(n._goal_pub.msgs, [])
        self.assertGreater(cancel_count(n), 0)
        self.assertEqual(n._announce_pub.msgs, [])  # 回答中は発話しない
        self.assertTrue(n._cmd_vel_pub.msgs)  # ゼロ速度は出している

    def test_crowded_and_no_target_also_hold(self):
        for crowd, target in (("crowded", False), ("calm", False), ("unknown", False)):
            n = self.make(crowd=crowd, target=target)
            n._clock.t += 10
            self.assertEqual(set(self.go(n, 1.0)), {"SURVEY_ACTIVE_HOLD"}, crowd)
            self.assertEqual(nonzero_twists(n), [])
            self.assertEqual(n._goal_pub.msgs, [])

    def test_wiggle_and_scan_stop(self):
        n = self.make(crowd="calm", target=False)
        n._idle_wiggle_enabled = True
        self.assertEqual(set(self.go(n, 2.0)), {"SURVEY_ACTIVE_HOLD"})
        self.assertEqual(nonzero_twists(n), [])

    # --- 解除後の復帰 ---
    def test_resumes_after_release(self):
        n = self.make(crowd="calm")
        n._clock.t += 10
        self.assertEqual(set(self.go(n, 1.0)), {"SURVEY_ACTIVE_HOLD"})
        modes = self.go(n, 1.0, survey=False)
        self.assertNotIn("SURVEY_ACTIVE_HOLD", modes)
        self.assertTrue(self.helper.moved(n))  # 接近の再開(従来動作)

    def test_gate_chain_holds_until_saved_then_resumes(self):
        # [2026-10-09] SurveyGate(実物)→director: 回答中は長くても止まり続け、保存完了の短い保持のあとに再開する
        import importlib.util
        from unittest import mock
        names = ("rclpy.node", "rclpy.qos", "std_msgs", "std_msgs.msg")
        with mock.patch.dict(sys.modules, {k: mock.MagicMock() for k in names}):  # rclpyの有無に依らず読み込む
            spec = importlib.util.spec_from_file_location(
                "nokosahen_core._survey_monitor_chain", str(TEST_DIR.parent / "nokosahen_core" / "survey_monitor.py"))
            sm = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(sm)
        gate = sm.SurveyGate()
        n = self.make(crowd="calm")
        n._clock.t += 10
        step = [0]

        def run(sample, seconds):
            modes = []
            for _ in range(int(round(seconds / 0.1))):
                gate.feed(sample)
                v = gate.tick(n._clock.t)
                if step[0] % 2 == 0:   # 5Hz配信
                    self.survey(n, v)
                step[0] += 1
                modes += self.helper.run_ticks(n, 0.1)
            return modes

        ans = {"ok": True, "active": True, "mode": "survey", "session_state": "answering", "saved_seq": 0}
        saved = {"ok": True, "active": False, "mode": "survey", "session_state": "saved", "saved_seq": 1}
        idle = dict(saved, session_state="idle")
        modes = run(ans, 90.0)            # 90秒考え込んでも止まったまま(動かない)
        self.assertEqual(set(modes), {"SURVEY_ACTIVE_HOLD"})
        self.assertEqual(nonzero_twists(n), [])
        self.assertEqual(n._goal_pub.msgs, [])
        modes = run(saved, 1.0)           # 保存完了の直後(post_save_hold 1.5秒)もまだ止まる
        self.assertEqual(set(modes), {"SURVEY_ACTIVE_HOLD"})
        modes = run(idle, 3.0)            # 解除後は通常動作へ
        self.assertNotIn("SURVEY_ACTIVE_HOLD", modes[-5:])
        self.assertTrue(self.helper.moved(n))

    def test_resumes_when_feed_stops(self):
        # monitorが落ちてtrueのまま途絶えても、失効(3秒)して通常動作へ戻る
        n = self.make(crowd="calm")
        n._clock.t += 10
        self.go(n, 1.0)
        modes = self.go(n, 5.0, survey=None)
        self.assertEqual(modes[0], "SURVEY_ACTIVE_HOLD")
        self.assertNotEqual(modes[-1], "SURVEY_ACTIVE_HOLD")

    def test_no_announce_burst_right_after_release(self):
        n = self.make(crowd="calm")
        self.go(n, 5.0)
        self.assertEqual(n._announce_pub.msgs, [])
        self.go(n, 1.0, survey=False)
        self.assertEqual(n._announce_pub.msgs, [])  # 発話間隔(60秒)はHOLD中も進めてある

    # --- 従来動作(survey_activeが無い/false) ---
    def test_never_received_is_unchanged(self):
        a = self.make(crowd="calm")
        b = self.make(crowd="calm")
        a._clock.t += 10
        b._clock.t += 10
        ma = self.go(a, 3.0, survey=None)
        mb = self.go(b, 3.0, survey=False)
        self.assertEqual(ma, mb)
        self.assertNotIn("SURVEY_ACTIVE_HOLD", ma)
        self.assertEqual(len(a._goal_pub.msgs), len(b._goal_pub.msgs))
        self.assertEqual(len(a._announce_pub.msgs), len(b._announce_pub.msgs))

    def test_node_without_survey_attrs_unchanged(self):
        # __init__を通さない(購読が無い)インスタンスでも属性参照で落ちず、ホールドしない
        n = self.make(crowd="calm")
        for name in ("_survey_active", "_last_survey_active_time"):
            n.__dict__.pop(name, None)
        self.assertFalse(n._survey_hold)

    # --- 安全状態 x アンケート中の総当たり ---
    def test_safety_priority_and_no_motion_matrix(self):
        cases = self.helper._safety_cases()
        for rules in [None] + self.helper.ALL_POLICY_RULES:
            for crowd in ("calm", "crowded"):
                # アンケート中だけ(安全状態なし): 方針が何でもSURVEY_ACTIVE_HOLD、走行指令なし
                n = self.make(rules=rules, crowd=crowd)
                n._clock.t += 10
                label = (rules and rules[0]["then"]["policy"], crowd, "survey_only")
                self.assertEqual(set(self.go(n, 1.0)), {"SURVEY_ACTIVE_HOLD"}, label)
                self.assertEqual(nonzero_twists(n), [], label)
                self.assertEqual(n._goal_pub.msgs, [], label)
                for expected, setter in cases:
                    label = (rules and rules[0]["then"]["policy"], crowd, expected)
                    n = self.make(rules=rules, crowd=crowd)
                    n._clock.t += 10
                    setter(n)
                    refresh = {
                        "MANUAL_OVERRIDE": lambda m, s=setter: s(m),
                        "NEAR_PERSON_HOLD": lambda m, s=setter: s(m),
                        "CAMERA_OBSTACLE_HOLD": lambda m, s=setter: s(m),
                    }.get(expected)
                    modes = self.go(n, 1.0, refresh=refresh)
                    # 既存の安全/手動の優先順位はアンケート中でも変わらない
                    # (NEAR_PERSON/CAMERA/範囲外等はSURVEYと同じ「止まる」なので、どちらかの停止モード)
                    if expected in ("ESTOP_HOLD", "MANUAL_OVERRIDE"):
                        self.assertEqual(set(modes), {expected}, label)
                    else:
                        self.assertTrue(
                            set(modes) <= {expected, "SURVEY_ACTIVE_HOLD"}, (label, set(modes))
                        )
                    self.assertEqual(nonzero_twists(n), [], label)
                    self.assertEqual(n._goal_pub.msgs, [], label)

    def test_manual_override_not_blocked_by_survey(self):
        # 手動運転中はdirectorは何も出さない(WASD/地図クリックは別経路)。HOLDがcancel_goal/ゼロ速度で
        # 手動指令を潰さない
        n = self.make(crowd="calm")
        n._manual_override_raw = True
        n._last_manual_override_time = n._clock.now()
        modes = self.go(
            n, 1.0,
            refresh=lambda m: setattr(m, "_last_manual_override_time", m._clock.now()),
        )
        self.assertEqual(set(modes), {"MANUAL_OVERRIDE"})
        self.assertEqual(cancel_count(n), 0)
        self.assertEqual(n._cmd_vel_pub.msgs, [])

    def test_estop_wins_and_manual_resumes_survey_hold_after(self):
        n = self.make(crowd="calm")
        n._hardware_estop = True
        self.assertEqual(set(self.go(n, 0.5)), {"ESTOP_HOLD"})
        n._hardware_estop = False
        self.assertEqual(set(self.go(n, 0.5)), {"SURVEY_ACTIVE_HOLD"})

    def test_survey_is_safety_mode_for_policy(self):
        self.assertIn("SURVEY_ACTIVE_HOLD", bd._SAFETY_MODES)
        self.assertIn("SURVEY_ACTIVE_HOLD", bd._GOAL_CANCELLING_HOLD_MODES)

    def test_policy_goal_not_sent_during_survey_and_rearmed_after(self):
        n = self.make(rules=[tbr.rule("a", policy="go_to_entrance")])
        n._clock.t += 10
        self.go(n, 1.0)
        self.assertEqual(n._goal_pub.msgs, [])
        n._clock.t += 10
        self.go(n, 1.0, survey=False)
        self.assertEqual(len(n._goal_pub.msgs), 1)


class _Future:
    def add_done_callback(self, cb):
        pass


class _Client:
    def __init__(self):
        self.sent = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        self.sent.append(goal)
        return _Future()


class _GoalAction:
    class Goal:
        def __init__(self):
            self.pose = None


class _Handle:
    def __init__(self):
        self.cancelled = 0
        self.accepted = True

    def cancel_goal_async(self):
        self.cancelled += 1
        return _Future()

    def get_result_async(self):
        return _Future()


def _pose(x=1.0, y=0.0, frame="map"):
    p = types.SimpleNamespace()
    p.header = types.SimpleNamespace(frame_id=frame)
    p.pose = types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y))
    return p


@unittest.skipIf(router is None, f"nav2_goal_router import失敗: {_ROUTER_ERR}")
class RouterSurveyTests(unittest.TestCase):
    def make(self):
        r = object.__new__(router.Nav2GoalRouter)
        r._navigate_action = _GoalAction
        r._client = _Client()
        r._hardware_estop = False
        r._soft_estop = False
        r._boundary_rect = None
        r._clock = tbr._Clock()
        r.get_clock = lambda: r._clock
        r._goal_handle = _Handle()
        r._goal_active = True
        r._spin_handle = None
        r._goal_seq = 0
        r._cmd_vel_pub = tbr._Pub()
        r._stop_burst_seconds = 1.5
        r._stop_burst_until = None
        r.statuses = []
        r.get_logger = lambda: types.SimpleNamespace(
            warn=lambda m: None, info=lambda m: None, error=lambda m: None
        )
        r._publish_goal_status = lambda event, status=0, target=None: r.statuses.append((event, target))
        self._orig = (router.Twist, router.Duration)
        router.Twist = tbr._Twist
        router.Duration = lambda seconds=0.0: tbr._T(seconds)
        self.addCleanup(lambda: (setattr(router, "Twist", self._orig[0]),
                                 setattr(router, "Duration", self._orig[1])))
        return r

    @staticmethod
    def stub_add(r):
        # _stop_burst_until = now + Duration の足し算のために_Tへ__add__を足す
        tbr._T.__add__ = lambda a, b: tbr._T(a.sec + b.sec)
        tbr._T.__ge__ = lambda a, b: a.sec >= b.sec

    def test_no_survey_goal_passes(self):
        r = self.make()
        r._on_goal_pose(_pose())
        self.assertEqual(len(r._client.sent), 1)
        self.assertEqual(r.statuses, [])

    def test_goal_rejected_while_survey_active(self):
        r = self.make()
        self.stub_add(r)
        r._on_survey_active(_BoolMsg(True))
        r._on_goal_pose(_pose())
        self.assertEqual(r._client.sent, [])
        self.assertEqual(r.statuses[-1][0], "survey_blocked")
        self.assertEqual(r.statuses[-1][1], (1.0, 0.0))

    def test_running_goal_cancelled_with_zero_velocity_on_start(self):
        r = self.make()
        self.stub_add(r)
        handle = r._goal_handle
        r._on_survey_active(_BoolMsg(True))
        self.assertEqual(handle.cancelled, 1)
        self.assertTrue(r._cmd_vel_pub.msgs)
        self.assertIsNotNone(r._stop_burst_until)
        # 再送(5Hz)では毎回取り消し直さない(立ち上がりだけ)
        for _ in range(5):
            r._clock.t += 0.2
            r._on_survey_active(_BoolMsg(True))
        self.assertEqual(handle.cancelled, 1)

    def test_goal_accepted_after_release(self):
        r = self.make()
        self.stub_add(r)
        r._on_survey_active(_BoolMsg(True))
        r._on_survey_active(_BoolMsg(False))
        r._on_goal_pose(_pose())
        self.assertEqual(len(r._client.sent), 1)

    def test_feed_expiry_releases_block(self):
        r = self.make()
        self.stub_add(r)
        r._on_survey_active(_BoolMsg(True))
        r._clock.t += 2.0
        r._on_goal_pose(_pose())
        self.assertEqual(r._client.sent, [])
        r._clock.t += 2.0  # 最後の受信から4秒: 失効
        r._on_goal_pose(_pose())
        self.assertEqual(len(r._client.sent), 1)

    def test_estop_status_precedes_survey(self):
        r = self.make()
        self.stub_add(r)
        r._hardware_estop = True
        r._on_survey_active(_BoolMsg(True))
        r._on_goal_pose(_pose())
        self.assertEqual(r.statuses[-1][0], "estop_blocked")

    def test_accepted_goal_cancelled_if_survey_started_in_flight(self):
        r = self.make()
        self.stub_add(r)
        r._goal_active = False
        r._goal_handle = None
        r._on_survey_active(_BoolMsg(True))
        gh = _Handle()
        gh.get_result_async = lambda: _Future()
        r._on_goal_response(types.SimpleNamespace(result=lambda: gh), (1.0, 0.0))
        self.assertEqual(gh.cancelled, 1)

    def test_accepted_goal_cancelled_if_cancel_arrived_while_waiting_for_acceptance(self):
        # 【2026-10-21】送信→受理の間に取り消し(緊急停止・手動運転・探索の停止・新しい目的地)が来て_goal_seqが進んだ場合、
        # あとから受理された目的地はすぐ取り消す(従来は_goal_activeがFalseで取り消す対象が無く、そのまま走っていた)。
        r = self.make()
        self.stub_add(r)
        r._goal_active = False
        r._goal_handle = None
        sent_seq = 7
        r._goal_seq = 8  # 送信後に取り消しが来て、世代が進んだ
        gh = _Handle()
        gh.get_result_async = lambda: _Future()
        r._on_goal_response(types.SimpleNamespace(result=lambda: gh), (1.0, 0.0), sent_seq)
        self.assertEqual(gh.cancelled, 1)
        self.assertFalse(r._goal_active)
        self.assertIsNone(r._goal_handle)

    def test_accepted_goal_kept_when_no_cancel_in_between(self):
        r = self.make()
        self.stub_add(r)
        r._goal_active = False
        r._goal_handle = None
        r._goal_seq = 8
        gh = _Handle()
        gh.get_result_async = lambda: _Future()
        r._on_goal_response(types.SimpleNamespace(result=lambda: gh), (1.0, 0.0), 8)
        self.assertEqual(gh.cancelled, 0)
        self.assertTrue(r._goal_active)
        self.assertIs(r._goal_handle, gh)

    def test_wasd_start_not_affected(self):
        # 手動(WASD)の立ち上がり処理はアンケート状態に依存しない
        r = self.make()
        self.stub_add(r)
        r._wasd_active_prev = False
        r._on_survey_active(_BoolMsg(True))
        h = r._goal_handle
        r._on_wasd_active(_BoolMsg(True))
        self.assertEqual(h.cancelled, 2)  # survey開始で1回 + WASD開始で1回(従来どおり)

    def test_attrs_missing_means_not_blocking(self):
        r = object.__new__(router.Nav2GoalRouter)
        self.assertFalse(r._survey_blocking)


if __name__ == "__main__":
    unittest.main(verbosity=1)
