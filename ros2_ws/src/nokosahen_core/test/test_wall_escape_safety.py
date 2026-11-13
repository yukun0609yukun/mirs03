"""wall_escape.py のレビュー指摘の修正のテスト(2026-10-24)。ROSは使わない(WallEscapeを__new__で作り、必要な属性とpublisherだけ替え玉)。

- H3: 先読み(pick_motion)に渡す点はlookahead_radius_m(1.5 m)まで。逃げる向き(flee_angle)は従来の0.7 m。点が多いときは間引く
- M2: _tickの例外でノードを落とさず、ゼロ速度を出して離脱を終える
- M3: 同じ目的地の再送で_retry_usedを戻さない(5 cm以内・60秒)。離脱の総回数の上限(5分で3回)

実行: python3 -I ros2_ws/src/nokosahen_core/test/test_wall_escape_safety.py
"""
import json
import math
import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core import motion_sim as ms  # noqa: E402
from nokosahen_core import wall_escape as we  # noqa: E402
from nokosahen_core.retry_marker import RetryMarker  # noqa: E402


class FakeTime:
    def __init__(self, ns):
        self.nanoseconds = ns

    def __sub__(self, other):
        return FakeTime(self.nanoseconds - other.nanoseconds)


class FakePub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(("info", m))

    def warn(self, m):
        self.lines.append(("warn", m))

    def error(self, m):
        self.lines.append(("error", m))


def FakeTwist():
    return types.SimpleNamespace(linear=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                                 angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0))


def FakeString():
    return types.SimpleNamespace(data="")


def make_scan(points, n=360):
    """points: [(距離, 角度rad)]。nステップの全周スキャン。他は無限遠(無効)。"""
    inc = 2 * math.pi / n
    ranges = [float("inf")] * n
    for r, a in points:
        i = int(round((a + math.pi) / inc)) % n
        ranges[i] = r
    return types.SimpleNamespace(ranges=ranges, angle_min=-math.pi, angle_increment=inc, range_min=0.05, range_max=10.0)


def make_goal(x=1.0, y=2.0, yaw_w=1.0):
    return types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id="map"),
        pose=types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y, z=0.0),
                                   orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=yaw_w)))


class Base(unittest.TestCase):
    def setUp(self):
        self._saved = {k: getattr(we, k, None) for k in ("Twist", "String", "Bool")}
        we.Twist = FakeTwist
        we.String = FakeString
        we.Bool = lambda data=False: types.SimpleNamespace(data=data)

    def tearDown(self):
        for k, v in self._saved.items():
            setattr(we, k, v)

    def make(self, scan_points=None, t=1000.0):
        n = we.WallEscape.__new__(we.WallEscape)
        n._t = t
        n._now = lambda: FakeTime(int(n._t * 1e9))
        n._enabled = True
        n._trigger, n._clear, n._radius = 0.45, 0.55, 0.7
        n._lookahead, n._look_max = 1.5, 150
        n._lin, n._ang_max, n._max_sec, n._stale, n._goal_memory = 0.7, 1.8, 10.0, 1.0, 30.0
        n._proactive = True
        n._scan_msg = make_scan(scan_points or [])
        n._points_cache = None
        n._look_cache = None
        n._scan_time = n._now()
        n._last_goal = None
        n._last_goal_time = None
        n._retry_used = False
        n._pending_retry = False
        n._wasd = n._soft_estop = n._hard_estop = n._near_person = n._router_rotating = False
        n._escaping_until = None
        n._proactive_due = None
        n._grant = None
        n._grant_time = None
        n._escape_manual = False
        n._manual_reason = ("", 0.0)
        n._last_stop = ("", 0.0)
        n._last_abort = 0.0
        n._self_cancel_until = 0.0
        n._cmd_pub, n._status_pub, n._goal_pub, n._cancel_pub = FakePub(), FakePub(), FakePub(), FakePub()
        n._retry_marker = RetryMarker(2.0, match=True)
        n._pose_cache = None
        n._waypoint = None
        n._costmap = None
        n._log = FakeLogger()
        n.get_logger = lambda: n._log
        return n

    def start(self, n):
        """離脱中の状態にする(_start_escapeの中のログ・状態配信を含む)。"""
        n._start_escape()


class ThinPointsTests(unittest.TestCase):
    def test_small_lists_unchanged(self):
        pts = [(1.0, i * 0.01) for i in range(150)]
        self.assertEqual(we.thin_points(pts, 150), pts)
        self.assertEqual(we.thin_points([], 150), [])

    def test_thins_evenly_keeping_angle_order(self):
        pts = [(1.0, -math.pi + i * 0.005) for i in range(1080)]
        out = we.thin_points(pts, 150)
        self.assertEqual(len(out), 150)
        angles = [a for _, a in out]
        self.assertEqual(angles, sorted(angles))
        self.assertEqual(out[0], pts[0])
        steps = {round(b - a, 6) for a, b in zip(angles, angles[1:])}
        self.assertLessEqual(max(steps) - min(steps), 0.005 + 1e-6)   # 等間隔(元の1刻みの違いまで)
        self.assertTrue(all(p in pts for p in out))

    def test_just_over_the_limit(self):
        pts = [(1.0, float(i)) for i in range(151)]
        out = we.thin_points(pts, 150)
        self.assertEqual(len(out), 150)
        self.assertEqual(len({a for _, a in out}), 150)


class LookaheadTests(Base):
    def escape_scene(self):
        # 後ろ0.3 mに壁(離脱のきっかけ)。前方1.0 mに物(0.7 mより遠い)。機体は真っすぐ前へ逃げたくなる
        wall = [(0.30, math.pi + k * 0.01) for k in range(-6, 7)]
        ahead = [(1.0, k * 0.01) for k in range(-6, 7)]
        return wall + ahead

    def test_flee_points_stay_within_repulsion_radius(self):
        n = self.make(self.escape_scene())
        self.assertTrue(all(r <= 0.7 for r, _ in n._points))
        self.assertTrue(any(r > 0.7 for r, _ in n._look_points))
        self.assertTrue(all(r <= 1.5 for r, _ in n._look_points))
        # 1.6 m以遠は先読みにも入らない
        n2 = self.make(self.escape_scene() + [(1.6, 0.3), (2.0, -0.3)])
        self.assertFalse(any(r > 1.5 for r, _ in n2._look_points))

    def test_pick_motion_receives_points_up_to_lookahead(self):
        n = self.make(self.escape_scene())
        got = {}

        def fake_pick(points, flee, *a, **k):
            got["points"] = list(points)
            got["flee"] = flee
            return (0.0, 0.0)

        saved = we.pick_motion
        we.pick_motion = fake_pick
        try:
            self.start(n)
            n._tick()
        finally:
            we.pick_motion = saved
        far = [p for p in got["points"] if math.hypot(*p) > 0.7]
        self.assertTrue(far, "0.7 mより遠い点が先読みに渡っていない")
        self.assertTrue(all(math.hypot(*p) <= 1.5 + 1e-6 for p in got["points"]))
        # 逃げる向きは0.7 m以内の点だけで決まる(後ろの壁から遠ざかる = 前向き)
        self.assertAlmostEqual(abs(got["flee"]), 0.0, delta=0.2)
        self.assertEqual(got["flee"], we.flee_angle(n._points, 0.7))

    def test_far_object_now_rejects_the_forward_dash(self):
        pts = self.escape_scene()
        xy = ms.to_xy(pts)
        # 旧: 0.7 m以内だけ -> 前方1.0 mの物が見えず、前進(1.05 m進む)を選んで当たる
        old = ms.pick_motion(ms.to_xy([p for p in pts if p[0] < 0.7]), 0.0, 0.7, 1.8)
        self.assertIsNotNone(old)
        self.assertIsNotNone(ms.first_collision_time(xy, old[0], old[1]), "前提: 旧方式の選択は前方の物に当たる")
        # 新: 1.5 mまで -> 当たらない動き(または動かない)
        n = self.make(pts)
        self.start(n)
        n._tick()
        if n._cmd_pub.sent:
            cmd = n._cmd_pub.sent[-1]
            if cmd.linear.x != 0.0 or cmd.angular.z != 0.0:
                self.assertIsNone(ms.first_collision_time(xy, cmd.linear.x, cmd.angular.z))

    def test_many_points_are_thinned(self):
        wall = [(0.30, math.pi + k * 0.002) for k in range(-3, 4)]
        ring = [(1.2, -math.pi + i * (2 * math.pi / 1080)) for i in range(1080)]
        scan = make_scan([], n=1080)
        for r, a in ring + wall:
            i = int(round((a + math.pi) / scan.angle_increment)) % 1080
            scan.ranges[i] = r
        n = self.make()
        n._scan_msg = scan
        pts = n._look_points
        self.assertLessEqual(len(pts), 150)
        self.assertGreaterEqual(len(pts), 140)
        angles = [a for _, a in pts]
        self.assertEqual(angles, sorted(angles))

    def test_look_cache_resets_on_new_scan(self):
        n = self.make(self.escape_scene())
        first = list(n._look_points)
        n._on_scan(make_scan([(0.5, 0.0)]))
        self.assertNotEqual(first, n._look_points)


class SourceTests(unittest.TestCase):
    def test_parameters_declared(self):
        src = (Path(__file__).resolve().parents[1] / "nokosahen_core" / "wall_escape.py").read_text(encoding="utf-8")
        self.assertIn('declare_parameter("lookahead_radius_m", 1.5)', src)
        self.assertIn('declare_parameter("lookahead_max_points", 150)', src)
        # 逃げる向きの計算は従来どおりrepulsion_radiusの点
        self.assertIn("flee_angle(self._points, self._radius)", src)
        # 離脱の速さは変えない(通常の走行と同じ)
        self.assertIn('declare_parameter("escape_linear_mps", NAV_SPEED_DEFAULT)', src)



class TickErrorTests(Base):
    def scene(self):
        return [(0.30, math.pi + k * 0.01) for k in range(-6, 7)]

    def raising_pick(self, *a, **k):
        raise RuntimeError("boom")

    def test_exception_stops_escape_with_zero_velocity_and_node_survives(self):
        n = self.make(self.scene())
        self.start(n)
        self.assertIsNotNone(n._escaping_until)
        saved = we.pick_motion
        we.pick_motion = self.raising_pick
        try:
            n._tick()          # 例外を外へ出さない
        finally:
            we.pick_motion = saved
        self.assertIsNone(n._escaping_until)
        self.assertFalse(n._pending_retry)
        last = n._cmd_pub.sent[-1]
        self.assertEqual((last.linear.x, last.angular.z), (0.0, 0.0))
        self.assertTrue(any(k == "error" for k, _ in n._log.lines))
        d = n.status_dict()
        self.assertEqual((d["state"], d["reason"]), ("failed", "内部エラー"))
        n._tick()              # 次のtickも普通に動く(何もしない)
        self.assertIsNone(n._escaping_until)

    def test_exception_does_not_resend_goal(self):
        n = self.make(self.scene())
        n._last_goal = make_goal()
        n._last_goal_time = n._now()
        self.start(n)
        saved = we.pick_motion
        we.pick_motion = self.raising_pick
        try:
            n._tick()
        finally:
            we.pick_motion = saved
        self.assertEqual(n._goal_pub.sent, [])
        self.assertFalse(n._retry_used)

    def test_exception_while_idle_publishes_nothing(self):
        n = self.make(self.scene())
        n._proactive_due = n._t - 1.0       # 先に離れる処理の期限が来ている

        def bad():
            raise RuntimeError("x")
        n._wall_close = bad
        n._tick()
        self.assertEqual(n._cmd_pub.sent, [])   # 動かしていないので、Nav2の指令を邪魔するゼロは出さない
        self.assertIsNone(n._proactive_due)
        self.assertIsNone(n._escaping_until)

    def test_error_handler_survives_publish_failure(self):
        n = self.make(self.scene())
        self.start(n)

        class BadPub:
            def publish(self, m):
                raise RuntimeError("pub")
        n._cmd_pub = BadPub()
        n._status_pub = BadPub()
        saved = we.pick_motion
        we.pick_motion = self.raising_pick
        try:
            n._tick()
        finally:
            we.pick_motion = saved
        self.assertIsNone(n._escaping_until)


def _old_first_collision_time(points, v, w, horizon=1.5, dt=0.05, half=ms.BODY_HALF_M, margin=0.02):
    """最適化前の実装(比較用)。"""
    h = half + margin
    n = int(math.ceil(horizon / dt))
    for i in range(n + 1):
        t = i * dt
        pose = ms.pose_at(v, w, t)
        for px, py in points:
            if ms.inside_body(px, py, pose, h):
                return t
    return None


class MotionSimOptimizationTests(unittest.TestCase):
    def test_same_results_as_the_old_implementation(self):
        import random
        rng = random.Random(7)
        cands = ms.default_candidates(0.7, 1.8) + [(0.3, 0.9), (-0.5, -1.2), (0.0, 1.8)]
        for _ in range(40):
            pts = [(rng.uniform(-1.6, 1.6), rng.uniform(-1.6, 1.6)) for _ in range(rng.choice([0, 1, 5, 80, 300]))]
            for v, w in cands:
                self.assertEqual(ms.first_collision_time(pts, v, w), _old_first_collision_time(pts, v, w), (v, w, len(pts)))
                self.assertEqual(ms.first_collision_time(pts, v, w, horizon=1.0, dt=0.1, half=0.3, margin=0.0),
                                 _old_first_collision_time(pts, v, w, horizon=1.0, dt=0.1, half=0.3, margin=0.0))

    def test_pick_motion_unchanged(self):
        pts = ms.to_xy([(0.30, math.pi + k * 0.01) for k in range(-6, 7)] + [(1.0, k * 0.01) for k in range(-6, 7)])
        got = ms.pick_motion(pts, 0.0, 0.7, 1.8)
        self.assertIsNotNone(got)
        self.assertIsNone(ms.first_collision_time(pts, got[0], got[1]))

    def test_faster_with_many_points(self):
        import time
        # 当たらない(全ステップを調べ切る最悪の場合)1000点。外形から十分遠い点だけ
        pts = [(1.5 + 0.001 * i, 1.4 + 0.001 * (i % 97)) for i in range(1000)]
        cands = ms.default_candidates(0.7, 1.8)

        def best_time(fn):
            best = 1e9
            for _ in range(3):
                t0 = time.perf_counter()
                for v, w in cands:
                    fn(pts, v, w)
                best = min(best, time.perf_counter() - t0)
            return best
        new = best_time(ms.first_collision_time)
        old = best_time(_old_first_collision_time)
        self.assertLess(new, old, f"new={new:.3f}s old={old:.3f}s")



ABORT = json.dumps({"event": "finished", "status": 6})


def abort_msg():
    return types.SimpleNamespace(data=ABORT)


class RetryKeptTests(Base):
    """M3: 同じ目的地の再送で_retry_usedを戻さない / 離脱の総回数の上限。"""

    WALL = [(0.30, math.pi + k * 0.01) for k in range(-6, 7)]

    def drive_one_escape(self, n, goal):
        """目的地を受け、中止され、壁際で離脱し、離れて、同じ目的地を出し直すまで。"""
        n._on_goal(goal)
        n._proactive_due = None              # (先に離れる予約は、この試験では使わない)
        n._on_goal_status(abort_msg())
        self.assertIsNotNone(n._escaping_until, "離脱が始まるはず")
        n._on_scan(make_scan([]))            # 壁から十分に離れた
        n._tick()
        self.assertIsNone(n._escaping_until)
        echo = n._goal_pub.sent[-1]
        n._on_goal(echo)                     # 自分の出し直しが/goal_poseに戻ってくる(目印で無視される)
        n._on_scan(make_scan(self.WALL))     # 次の中止のときは、また壁際

    def test_same_goal_resend_keeps_retry_used(self):
        n = self.make(self.WALL)
        g = make_goal(1.0, 2.0)
        self.drive_one_escape(n, g)
        self.assertTrue(n._retry_used)
        # 他の出し元が、同じ目的地を数秒おきに再送し、そのたびに中止になる
        for i in range(10):
            n._t += 3.0
            n._on_scan(make_scan(self.WALL))
            n._on_goal(make_goal(1.0, 2.0))
            self.assertTrue(n._retry_used, i)
            self.assertIsNone(n._proactive_due, i)   # 先に離れる処理も予約しない
            n._on_goal_status(abort_msg())
            self.assertIsNone(n._escaping_until, i)  # 離脱は繰り返さない
        self.assertEqual(len(n._escape_times), 1)
        self.assertEqual(len(n._goal_pub.sent), 1)    # 出し直しも1回だけ

    def test_within_5cm_is_same_but_beyond_is_new(self):
        n = self.make(self.WALL)
        self.drive_one_escape(n, make_goal(1.0, 2.0))
        n._t += 1.0
        n._on_goal(make_goal(1.03, 2.03))           # 約4.2 cm(5 cm以内)
        self.assertTrue(n._retry_used)
        n._t += 1.0
        n._on_goal(make_goal(1.0, 2.09))            # 直前(1.03, 2.03)から約6.7 cm
        self.assertFalse(n._retry_used)
        self.assertIsNotNone(n._proactive_due)      # 新しい目的地は、従来どおり先に離れる

    def test_keeps_for_60_seconds_after_the_retry_only(self):
        n = self.make(self.WALL)
        self.drive_one_escape(n, make_goal(1.0, 2.0))
        n._t += 59.0
        n._on_goal(make_goal(1.0, 2.0))
        self.assertTrue(n._retry_used)
        n._t += 2.0                                  # やり直しから61秒
        n._on_goal(make_goal(1.0, 2.0))
        self.assertFalse(n._retry_used)              # 戻る(総回数の上限が別に効く)

    def test_different_goal_resets_as_before(self):
        n = self.make(self.WALL)
        self.drive_one_escape(n, make_goal(1.0, 2.0))
        n._on_goal(make_goal(5.0, 5.0))
        self.assertFalse(n._retry_used)

    def test_first_goal_unchanged(self):
        n = self.make(self.WALL)
        n._on_goal(make_goal(1.0, 2.0))
        self.assertFalse(n._retry_used)
        self.assertIsNotNone(n._proactive_due)


class EscapeCapTests(Base):
    WALL = RetryKeptTests.WALL

    def start_auto(self, n, x):
        """別々の目的地(xが違う)で中止 -> 自動の離脱を始める。始まったらTrue。"""
        n._on_scan(make_scan(self.WALL))
        n._on_goal(make_goal(x, 0.0))
        n._proactive_due = None
        n._on_goal_status(abort_msg())
        started = n._escaping_until is not None
        n._stop_escape("テスト")
        n._pending_retry = False
        n._on_scan(make_scan(self.WALL))
        if started:
            n._last_stop = ("", 0.0)
        return started

    def test_fourth_escape_in_5_minutes_is_refused_with_reason(self):
        n = self.make(self.WALL)
        for i in range(3):
            n._t += 20.0
            self.assertTrue(self.start_auto(n, 10.0 + i), i)
        n._t += 20.0
        self.assertFalse(self.start_auto(n, 20.0))
        d = n.status_dict()
        self.assertEqual(d["state"], "failed")
        self.assertIn("上限", d["reason"])
        self.assertIn("3回", d["reason"])
        self.assertTrue(any(k == "warn" and "上限" in m for k, m in n._log.lines))

    def test_allowed_again_after_the_window(self):
        n = self.make(self.WALL)
        for i in range(3):
            n._t += 10.0
            self.assertTrue(self.start_auto(n, 10.0 + i))
        n._t += 301.0 - 20.0   # 最初の離脱(t+10)から5分以上
        self.assertTrue(self.start_auto(n, 30.0))

    def test_proactive_start_is_also_capped(self):
        n = self.make(self.WALL)
        for i in range(3):
            n._t += 10.0
            self.assertTrue(self.start_auto(n, 10.0 + i))
        n._t += 10.0
        n._on_goal(make_goal(99.0, 0.0))
        n._t += 1.0                                  # 0.4秒の待ちを過ぎる
        n._on_scan(make_scan(self.WALL))
        n._tick()
        self.assertIsNone(n._escaping_until)
        self.assertIn("上限", n.status_dict()["reason"])

    def test_manual_escape_is_not_capped_nor_counted(self):
        n = self.make(self.WALL)
        for i in range(3):
            n._t += 10.0
            self.assertTrue(self.start_auto(n, 10.0 + i))
        n._t += 10.0
        n._on_scan(make_scan(self.WALL))
        n._on_trigger(types.SimpleNamespace(data=True))
        self.assertIsNotNone(n._escaping_until)      # 手動(ボタン)は上限の対象外
        self.assertTrue(n._escape_manual)
        self.assertEqual(len(n._escape_times), 3)    # 数えない

    def test_parameters_declared(self):
        src = (Path(__file__).resolve().parents[1] / "nokosahen_core" / "wall_escape.py").read_text(encoding="utf-8")
        self.assertIn('declare_parameter("max_escapes_per_window", 3)', src)
        self.assertIn('declare_parameter("escape_window_sec", 300.0)', src)


if __name__ == "__main__":
    unittest.main()
