"""person_clear_gate.ClearGate と rear_camera_persons の純粋な関数のテスト(2026-10-22)。ROSは使わない。
実行: python3 -I test/test_person_clear_gate.py   (nokosahen_coreディレクトリで)
"""
import json
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core import person_clear_gate as pcg  # noqa: E402
from nokosahen_core import rear_camera_persons as rcp  # noqa: E402

PKG = Path(__file__).resolve().parents[1] / "nokosahen_core"
DEG = math.pi / 180.0

OK_REAR = {"left_rear": {"ok": True, "count": 0}, "right_rear": {"ok": True, "count": 0}}


def make_clear(gate, now, front=0, rear=None):
    gate.set_front(now, front)
    gate.set_rear(now, OK_REAR if rear is None else rear)


class ClearGateTests(unittest.TestCase):
    def setUp(self):
        self.g = pcg.ClearGate(clear_hold_sec=1.0, stale_sec=1.5)

    def feed(self, now, **kw):
        make_clear(self.g, now, **kw)

    def hold_until_true(self, start=0.0):
        """start から条件を揃え続け、relaxがTrueになることを確認する。"""
        self.feed(start)
        self.assertFalse(self.g.relax(start))
        self.feed(start + 0.5)
        self.assertFalse(self.g.relax(start + 0.5))
        self.feed(start + 1.0)
        self.assertTrue(self.g.relax(start + 1.0))

    # (1) 全部クリアのときだけ、保持時間後にTrue
    def test_true_only_after_hold(self):
        self.hold_until_true()

    def test_stays_true_while_clear(self):
        self.hold_until_true()
        self.feed(5.0)
        self.assertTrue(self.g.relax(5.0))

    def test_no_data_false(self):
        self.assertFalse(self.g.relax(0.0))
        self.assertFalse(self.g.relax(100.0))

    # (2) 一つでも崩れたら、すぐFalse。再びそろっても保持時間やり直し
    def assert_blocked_then_restart(self, break_fn, restore_fn=None):
        self.hold_until_true()
        break_fn()
        self.assertFalse(self.g.relax(1.1))
        if restore_fn:
            restore_fn()
        self.feed(1.2)
        # 再びそろった直後はFalse(保持時間やり直し)
        self.assertFalse(self.g.relax(1.2))
        self.feed(2.0)
        self.assertFalse(self.g.relax(2.0))
        self.feed(2.2)
        self.assertTrue(self.g.relax(2.2))

    def test_front_person(self):
        self.assert_blocked_then_restart(lambda: self.g.set_front(1.1, 1))

    def test_rear_count_positive(self):
        bad = {"left_rear": {"ok": True, "count": 1}, "right_rear": {"ok": True, "count": 0}}
        self.assert_blocked_then_restart(lambda: self.g.set_rear(1.1, bad))

    def test_rear_other_camera_count_positive(self):
        bad = {"left_rear": {"ok": True, "count": 0}, "right_rear": {"ok": True, "count": 2}}
        self.assert_blocked_then_restart(lambda: self.g.set_rear(1.1, bad))

    def test_rear_not_ok(self):
        bad = {"left_rear": {"ok": False, "count": 0}, "right_rear": {"ok": True, "count": 0}}
        self.assert_blocked_then_restart(lambda: self.g.set_rear(1.1, bad))

    def test_rear_missing_ok_key(self):
        bad = {"left_rear": {"count": 0}, "right_rear": {"ok": True, "count": 0}}
        self.assert_blocked_then_restart(lambda: self.g.set_rear(1.1, bad))

    def test_rear_missing_count_key_is_not_clear(self):
        bad = {"left_rear": {"ok": True}, "right_rear": {"ok": True, "count": 0}}
        self.assert_blocked_then_restart(lambda: self.g.set_rear(1.1, bad))

    def test_front_stale(self):
        self.hold_until_true()
        # 後ろだけ更新し、前面は古いまま(1.0 -> 2.6 で1.6秒経過)
        self.g.set_rear(2.6, OK_REAR)
        self.assertFalse(self.g.relax(2.6))

    def test_rear_stale(self):
        self.hold_until_true()
        self.g.set_front(2.6, 0)
        self.assertFalse(self.g.relax(2.6))

    def test_stale_boundary_is_still_fresh(self):
        # stale_sec ちょうどは新しい扱い(>で古い判定)
        g = pcg.ClearGate(clear_hold_sec=0.0, stale_sec=1.5)
        g.set_front(0.0, 0)
        g.set_rear(0.0, OK_REAR)
        self.assertTrue(g.relax(1.5))
        self.assertFalse(g.relax(1.51))

    def test_stale_then_fresh_restarts_hold(self):
        self.hold_until_true()
        self.assertFalse(self.g.relax(10.0))  # 全部古い
        self.feed(10.1)
        self.assertFalse(self.g.relax(10.1))
        self.feed(11.1)
        self.assertTrue(self.g.relax(11.1))

    def test_front_not_received(self):
        self.g.set_rear(0.0, OK_REAR)
        for t in (0.0, 1.0, 2.0):
            self.g.set_rear(t, OK_REAR)
            self.assertFalse(self.g.relax(t))

    def test_rear_not_received(self):
        for t in (0.0, 1.0, 2.0):
            self.g.set_front(t, 0)
            self.assertFalse(self.g.relax(t))

    def test_near_person_stop(self):
        def brk():
            self.g.near_person_stop = True

        def restore():
            self.g.near_person_stop = False
        self.assert_blocked_then_restart(brk, restore)

    def test_estop(self):
        def brk():
            self.g.estop = True

        def restore():
            self.g.estop = False
        self.assert_blocked_then_restart(brk, restore)

    def test_target_seen(self):
        self.hold_until_true()
        self.g.target_seen(1.1)
        self.feed(1.1)
        self.assertFalse(self.g.relax(1.1))
        # stale_sec以内は引き続きFalse
        self.feed(2.5)
        self.assertFalse(self.g.relax(2.5))
        # stale_secを過ぎたら条件はそろうが、保持時間はやり直し
        self.feed(2.7)
        self.assertFalse(self.g.relax(2.7))
        self.feed(3.0)
        self.assertFalse(self.g.relax(3.0))
        self.feed(3.7)
        self.assertTrue(self.g.relax(3.7))

    def test_old_target_seen_ignored(self):
        self.g.target_seen(-10.0)
        self.hold_until_true()

    def test_momentary_break_resets_timer(self):
        self.feed(0.0)
        self.assertFalse(self.g.relax(0.0))
        self.feed(0.9)
        self.assertFalse(self.g.relax(0.9))
        self.g.set_front(0.95, 1)
        self.assertFalse(self.g.relax(0.95))
        self.feed(1.0)
        self.assertFalse(self.g.relax(1.0))   # 0.0から1.0秒だが、途中で途切れたのでFalse
        self.feed(1.9)
        self.assertFalse(self.g.relax(1.9))
        self.feed(2.0)
        self.assertTrue(self.g.relax(2.0))

    # (3) rearが空dict/None
    def test_rear_empty_dict(self):
        for t in (0.0, 1.0, 2.0):
            self.g.set_front(t, 0)
            self.g.set_rear(t, {})
            self.assertFalse(self.g.relax(t))

    def test_rear_none(self):
        for t in (0.0, 1.0, 2.0):
            self.g.set_front(t, 0)
            self.g.set_rear(t, None)
            self.assertFalse(self.g.relax(t))


class WrapAngleTests(unittest.TestCase):
    def test_wrap(self):
        self.assertAlmostEqual(rcp.wrap_angle(0.0), 0.0)
        self.assertAlmostEqual(rcp.wrap_angle(3 * math.pi / 2), -math.pi / 2)
        self.assertAlmostEqual(rcp.wrap_angle(-3 * math.pi / 2), math.pi / 2)
        self.assertAlmostEqual(rcp.wrap_angle(2 * math.pi), 0.0)
        for a in (-10.0, -4.0, 4.0, 10.0):
            w = rcp.wrap_angle(a)
            self.assertTrue(-math.pi <= w <= math.pi)


class BoxBearingTests(unittest.TestCase):
    def test_center_is_yaw(self):
        self.assertAlmostEqual(rcp.box_bearing(320, 640, 135.0, 60.0), 135.0 * DEG)
        self.assertAlmostEqual(rcp.box_bearing(320, 640, -135.0, 60.0), -135.0 * DEG)
        self.assertAlmostEqual(rcp.box_bearing(320, 640, 0.0, 60.0), 0.0)

    def test_left_edge_is_plus(self):
        self.assertAlmostEqual(rcp.box_bearing(0, 640, 135.0, 60.0), 165.0 * DEG)
        self.assertAlmostEqual(rcp.box_bearing(0, 640, 0.0, 60.0), 30.0 * DEG)
        self.assertAlmostEqual(rcp.box_bearing(0, 640, -135.0, 60.0), -105.0 * DEG)

    def test_right_edge_is_minus(self):
        self.assertAlmostEqual(rcp.box_bearing(640, 640, 135.0, 60.0), 105.0 * DEG)
        self.assertAlmostEqual(rcp.box_bearing(640, 640, 0.0, 60.0), -30.0 * DEG)
        self.assertAlmostEqual(rcp.box_bearing(640, 640, -135.0, 60.0), -165.0 * DEG)

    def test_left_greater_than_right(self):
        l = rcp.box_bearing(100, 640, 135.0, 60.0)
        r = rcp.box_bearing(500, 640, 135.0, 60.0)
        self.assertGreater(l, r)

    def test_wraps_around_pi(self):
        # yaw=180: 左端 +210 -> -150、右端 150
        self.assertAlmostEqual(rcp.box_bearing(0, 640, 180.0, 60.0), -150.0 * DEG)
        self.assertAlmostEqual(rcp.box_bearing(640, 640, 180.0, 60.0), 150.0 * DEG)
        # yaw=-180 でも対称
        self.assertAlmostEqual(rcp.box_bearing(0, 640, -180.0, 60.0), -150.0 * DEG)
        # yaw=-135, fov=120: 右端 -135-60=-195 -> 165 (±180をまたぐ)
        self.assertAlmostEqual(rcp.box_bearing(640, 640, -135.0, 120.0), 165.0 * DEG)
        # yaw=135, fov=120: 左端 135+60=195 -> -165
        self.assertAlmostEqual(rcp.box_bearing(0, 640, 135.0, 120.0), -165.0 * DEG)

    def test_range_always_normalized(self):
        for yaw in (-180, -135, 0, 135, 180, 270):
            for x in (0, 100, 320, 540, 640):
                b = rcp.box_bearing(x, 640, float(yaw), 90.0)
                self.assertTrue(-math.pi <= b <= math.pi)

    def test_zero_width_does_not_crash(self):
        self.assertAlmostEqual(rcp.box_bearing(0, 0, 135.0, 60.0), 165.0 * DEG)


class ParseCamerasTests(unittest.TestCase):
    def default(self):
        return [dict(c) for c in rcp.DEFAULT_CAMERAS]

    def test_default_cameras_contents(self):
        names = [c["name"] for c in rcp.DEFAULT_CAMERAS]
        self.assertEqual(names, ["left_rear", "right_rear"])
        left, right = rcp.DEFAULT_CAMERAS
        self.assertGreater(left["yaw_deg"], 90.0)    # 左後ろ: +側
        self.assertLess(right["yaw_deg"], -90.0)     # 右後ろ: -側
        for c in rcp.DEFAULT_CAMERAS:
            self.assertTrue(c["device"])
            self.assertGreater(c["fov_deg"], 0.0)

    def test_empty_and_blank(self):
        self.assertEqual(rcp.parse_cameras(""), self.default())
        self.assertEqual(rcp.parse_cameras("   \n"), self.default())

    def test_invalid_json(self):
        self.assertEqual(rcp.parse_cameras("{not json"), self.default())

    def test_not_a_list(self):
        self.assertEqual(rcp.parse_cameras('{"name": "a"}'), self.default())
        self.assertEqual(rcp.parse_cameras("123"), self.default())

    def test_empty_list(self):
        self.assertEqual(rcp.parse_cameras("[]"), self.default())

    def test_missing_required_key(self):
        for missing in ("name", "device", "yaw_deg", "fov_deg"):
            cam = {"name": "x", "device": "/dev/video0", "yaw_deg": 90, "fov_deg": 60}
            del cam[missing]
            self.assertEqual(rcp.parse_cameras(json.dumps([cam])), self.default(), missing)

    def test_bad_value_type(self):
        cam = {"name": "x", "device": "/dev/video0", "yaw_deg": "abc", "fov_deg": 60}
        self.assertEqual(rcp.parse_cameras(json.dumps([cam])), self.default())

    def test_valid(self):
        cams = [
            {"name": "a", "device": "/dev/video0", "yaw_deg": 90, "fov_deg": 70},
            {"name": "b", "device": "/dev/video2", "yaw_deg": -90.5, "fov_deg": 65.5},
        ]
        out = rcp.parse_cameras(json.dumps(cams))
        self.assertEqual(out, [
            {"name": "a", "device": "/dev/video0", "yaw_deg": 90.0, "fov_deg": 70.0},
            {"name": "b", "device": "/dev/video2", "yaw_deg": -90.5, "fov_deg": 65.5},
        ])

    def test_bad_entry_skipped_good_kept(self):
        cams = [{"name": "bad"}, {"name": "a", "device": "d", "yaw_deg": 1, "fov_deg": 2}]
        out = rcp.parse_cameras(json.dumps(cams))
        self.assertEqual([c["name"] for c in out], ["a"])

    def test_default_is_copy(self):
        out = rcp.parse_cameras("")
        out[0]["name"] = "changed"
        self.assertEqual(rcp.DEFAULT_CAMERAS[0]["name"], "left_rear")


class StaticSourceTests(unittest.TestCase):
    def setUp(self):
        self.src = (PKG / "person_clear_gate.py").read_text(encoding="utf-8")

    def test_param_is_velocity_stop_enabled(self):
        self.assertIn('PARAM = "VelocityStop.enabled"', self.src)

    def test_no_bare_dot_velocity_stop(self):
        # ".VelocityStop" (名前空間つきパラメータ名など)は使わない
        self.assertNotIn(".VelocityStop", self.src.replace('"VelocityStop.enabled"', ""))

    def test_footprint_approach_never_disabled(self):
        self.assertNotIn("FootprintApproach.enabled", self.src)

    def test_rear_stop_never_touched(self):
        # 【2026-10-24 H1】後退用の停止領域(RearStop)は常に有効。ゲートは名前も触れない
        self.assertNotIn("RearStop", self.src)
        self.assertNotIn("backward", self.src)


class FailSafeAndWiringTests(unittest.TestCase):
    ROOT = Path(__file__).resolve().parents[2]

    def test_gate_restores_enabled_on_error_and_exit(self):
        src = (self.ROOT / "nokosahen_core" / "nokosahen_core" / "person_clear_gate.py").read_text(encoding="utf-8")
        self.assertIn("self._send(True)", src)          # 例外時
        self.assertIn("node.restore_blocking(", src)    # 終了時(rclpyが生きているうちに同期で送る)
        self.assertIn("_applied_time", src)             # 定期的な再送

    def test_launch_condition_uses_use_rear_cameras(self):
        core = (self.ROOT / "nokosahen_bringup" / "launch" / "core.launch.py").read_text(encoding="utf-8")
        self.assertIn('DeclareLaunchArgument("use_rear_cameras"', core)
        self.assertIn("use_rear_cameras", core.split('executable="rear_camera_persons"')[1][:500])
        pb = (self.ROOT / "nokosahen_bringup" / "launch" / "persistent_base.launch.py").read_text(encoding="utf-8")
        self.assertIn('"use_rear_cameras": use_rear_cameras', pb)

    def test_start_all_option(self):
        src = (self.ROOT.parents[1] / "scripts" / "start_all.sh").read_text(encoding="utf-8")
        self.assertIn("--no-rear-cameras", src)
        self.assertIn("use_rear_cameras:=", src)



# ---- 【2026-10-24 H2】終了時の復元(同期)と、状態(心拍)の配信。rclpy・rcl_interfacesは替え玉 ----
class _Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class _FakeFuture:
    def __init__(self, done=True):
        self._done = done
        self.callbacks = []

    def add_done_callback(self, cb):
        self.callbacks.append(cb)

    def done(self):
        return self._done


class _FakeClient:
    def __init__(self, ready=True, done=True):
        self.ready = ready
        self.done = done
        self.sent = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, req):
        self.sent.append([(p.name, p.value.bool_value) for p in req.parameters])
        return _FakeFuture(self.done)


class _FakePub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(json.loads(msg.data))


class _FakeRclpy:
    def __init__(self):
        self.spin_calls = []

    def spin_until_future_complete(self, node, fut, timeout_sec=None):
        self.spin_calls.append(timeout_sec)


class _FakeSetParameters:
    class Request:
        def __init__(self):
            self.parameters = []


def _fake_param():
    return _Msg(name=None, value=None)


class NodeFakeTests(unittest.TestCase):
    NAMES = ("rclpy", "Parameter", "ParameterType", "ParameterValue", "SetParameters", "String")

    def setUp(self):
        self._saved = {n: getattr(pcg, n, None) for n in self.NAMES}
        self.rclpy = _FakeRclpy()
        pcg.rclpy = self.rclpy
        pcg.Parameter = _fake_param
        pcg.ParameterType = _Msg(PARAMETER_BOOL=1)
        pcg.ParameterValue = lambda type, bool_value: _Msg(type=type, bool_value=bool_value)
        pcg.SetParameters = _FakeSetParameters
        pcg.String = lambda: _Msg(data="")
        n = pcg.PersonClearGate.__new__(pcg.PersonClearGate)
        n._on = True
        n._relaxed_req = False
        n._applied = None
        n._applied_time = 0.0
        n._cli = _FakeClient()
        n._status_pub = _FakePub()
        n._gate = pcg.ClearGate()
        n.get_logger = lambda: _Msg(info=lambda m: None, warn=lambda m: None, error=lambda m: None)
        self.n = n

    def tearDown(self):
        for k, v in self._saved.items():
            setattr(pcg, k, v)

    def test_status_json_has_relaxed_and_stamp(self):
        d = json.loads(pcg.build_status(True, 123.5))
        self.assertEqual(d, {"relaxed": True, "stamp": 123.5})
        self.assertIs(json.loads(pcg.build_status(False, 1.0))["relaxed"], False)

    def test_status_follows_the_last_request(self):
        self.n._publish_status()
        self.assertFalse(self.n._status_pub.sent[-1]["relaxed"])
        self.n._send(False)                       # 無効にする要求
        self.n._publish_status()
        self.assertTrue(self.n._status_pub.sent[-1]["relaxed"])
        self.assertIsInstance(self.n._status_pub.sent[-1]["stamp"], float)
        self.n._send(True)                        # 有効に戻す要求
        self.n._publish_status()
        self.assertFalse(self.n._status_pub.sent[-1]["relaxed"])

    def test_status_unchanged_when_service_not_ready(self):
        self.n._cli.ready = False
        self.assertIsNone(self.n._send(False))
        self.assertFalse(self.n._relaxed_req)

    def test_restore_blocking_sends_true_and_waits_up_to_2s(self):
        self.n._send(False)
        self.assertTrue(self.n.restore_blocking(2.0))
        self.assertEqual(self.n._cli.sent[-1], [("VelocityStop.enabled", True)])
        self.assertEqual(self.rclpy.spin_calls, [2.0])
        self.assertFalse(self.n._relaxed_req)
        self.assertFalse(self.n._on)              # 待つ間にタイマーが再び無効にしない

    def test_restore_blocking_without_service_or_response(self):
        self.n._cli.ready = False
        self.assertFalse(self.n.restore_blocking(2.0))
        self.assertEqual(self.rclpy.spin_calls, [])
        self.n._cli.ready = True
        self.n._cli.done = False                  # 応答が来ない
        self.assertFalse(self.n.restore_blocking(2.0))
        self.assertEqual(self.rclpy.spin_calls, [2.0])

    def test_restore_blocking_swallows_errors(self):
        def boom(*a, **k):
            raise RuntimeError("context closed")
        self.rclpy.spin_until_future_complete = boom
        self.assertFalse(self.n.restore_blocking(2.0))

    def test_tick_error_requests_enable(self):
        def bad():
            raise RuntimeError("x")
        self.n._tick_inner = bad
        self.n._send(False)
        self.n._tick()
        self.assertEqual(self.n._cli.sent[-1], [("VelocityStop.enabled", True)])
        self.assertFalse(self.n._relaxed_req)


class SignalTests(unittest.TestCase):
    def test_handlers_set_event_for_sigterm_and_sigint(self):
        import signal
        old = (signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM))
        try:
            ev = pcg.install_stop_handlers()
            self.assertFalse(ev.is_set())
            signal.raise_signal(signal.SIGTERM)
            self.assertTrue(ev.is_set())
            ev.clear()
            signal.raise_signal(signal.SIGINT)    # KeyboardInterruptにならず、Eventが立つ
            self.assertTrue(ev.is_set())
        finally:
            signal.signal(signal.SIGINT, old[0])
            signal.signal(signal.SIGTERM, old[1])

    def test_main_disables_rclpy_signal_handlers(self):
        src = (PKG / "person_clear_gate.py").read_text(encoding="utf-8")
        main = src[src.index("def main()"):]
        self.assertIn("SignalHandlerOptions.NO", main)
        self.assertIn("restore_blocking(2.0)", main)
        self.assertNotIn("rclpy.spin(node)", main)  # spin_onceで回し、停止の合図(Event)を見る


if __name__ == "__main__":
    unittest.main()
