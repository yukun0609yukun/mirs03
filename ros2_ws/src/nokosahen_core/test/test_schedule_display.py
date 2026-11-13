"""スケジュールによる頭部ディスプレイの画面の切替(schedule_display.DisplayScheduleController)の単体テスト(2026-10-23)。

ROS・ラズパイ・ロボットには一切触れない(ラズパイは替え玉)。docs/schedule.md。確認すること:
- 要求が変わったときだけ切り替える。繰り返しの再送で、すでにそのモードなら何もしない(問い合わせもしない)
- アンケート回答中は別のモードへ切り替えない(pending+理由)。終わったら切り替える
- 判定できない(ラズパイに届かない・応答が不正)ときは切り替えない。ロボットには影響しない(エラーを状態に出す)
- 項目が終わったら(keep)・スケジュール停止・要求が途絶えたら、スケジュール開始前のモードへ戻す(回答中なら保留)
- 手動で切り替えた場合、要求が変わるまで上書きし直さない
- 不正な要求は無視。baselineはファイルに残り、再起動後も戻せる
実行: python3 -I ros2_ws/src/nokosahen_core/test/test_schedule_display.py
"""

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR.parent))

from nokosahen_core import schedule_display as sd  # noqa: E402
from nokosahen_core import schedule_plan as sp  # noqa: E402


class FakeAdmin:
    """SurveyAdminProxy の替え玉(get_state / set_mode / configured)。"""

    def __init__(self, mode="customer", answering=False):
        self.configured = True
        self.mode = mode
        self.active = answering
        self.session_state = "answering" if answering else "idle"
        self.reachable = True
        self.malformed = False
        self.set_fails = False
        self.get_calls = 0
        self.set_calls = []

    def get_state(self, fresh=False):
        self.get_calls += 1
        if not self.reachable:
            return {"ok": False, "configured": True, "error": "x", "message": "ラズパイに届きません: timeout"}
        if self.malformed:
            return {"ok": True, "configured": True, "state": "???"}
        return {"ok": True, "configured": True, "state": {
            "mode": self.mode, "active": self.active, "session_state": self.session_state}}

    def set_mode(self, data):
        self.set_calls.append(data["mode"])
        if self.set_fails:
            return 200, {"ok": False, "configured": True, "message": "ラズパイに届きません: boom"}
        self.mode = data["mode"]
        return 200, {"ok": True, "configured": True, "state": {"mode": data["mode"]}}

    def answer(self, on=True):
        self.active = on
        self.session_state = "answering" if on else "idle"


class Clk:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def req(mode, entry="e1", source="schedule"):
    return json.dumps({"mode": mode, "source": source, "entry_id": entry, "stamp": 1.0})


class Base(unittest.TestCase):
    def setUp(self):
        self.admin = FakeAdmin()
        self.clk = Clk()
        self.tmp = tempfile.TemporaryDirectory()
        self.base_path = os.path.join(self.tmp.name, "baseline.json")
        self.ctl = self.make()

    def tearDown(self):
        self.tmp.cleanup()

    def make(self):
        return sd.DisplayScheduleController(self.admin, baseline_file=self.base_path, clock=self.clk)

    def send(self, mode, entry="e1"):
        self.ctl.on_request_text(req(mode, entry))

    def tick(self, advance=1.0):
        self.clk.t += advance
        return self.ctl.tick()


class SwitchingTests(Base):
    def test_switches_when_the_request_arrives(self):
        self.send("survey")
        st = self.tick()
        self.assertEqual(self.admin.set_calls, ["survey"])
        self.assertEqual(self.admin.mode, "survey")
        self.assertEqual((st["state"], st["requested"], st["current"], st["baseline"]), ("ok", "survey", "survey", "customer"))

    def test_repeated_requests_do_nothing_when_already_in_that_mode(self):
        self.send("survey")
        self.tick()
        gets = self.admin.get_calls
        for _ in range(6):  # 5秒おきの再送
            self.clk.t += 5
            self.send("survey")
            self.tick(0.1)
        self.assertEqual(self.admin.set_calls, ["survey"])
        self.assertEqual(self.admin.get_calls, gets)  # 満たした要求の再送では、ラズパイへ問い合わせもしない

    def test_already_in_the_requested_mode_does_not_call_set_mode(self):
        self.admin.mode = "customer"
        self.send("customer")
        st = self.tick()
        self.assertEqual(self.admin.set_calls, [])
        self.assertEqual(st["state"], "ok")
        self.assertIsNone(st["baseline"])  # 切り替えていない=戻す元は無い

    def test_a_changed_request_switches_again_and_keeps_the_original_baseline(self):
        self.send("survey")
        self.tick()
        self.send("ocrtest", "e2")
        self.tick()
        self.assertEqual(self.admin.set_calls, ["survey", "ocrtest"])
        self.assertEqual(self.ctl.status()["baseline"], "customer")  # 最初の元のモードのまま

    def test_idle_when_nothing_requested(self):
        st = self.tick()
        self.assertEqual(st["state"], "idle")
        self.assertEqual((self.admin.get_calls, self.admin.set_calls), (0, []))  # スケジュールを使っていなければ問い合わせない

    def test_invalid_requests_are_ignored(self):
        for bad in ("{", "[]", req("vr"), req("survey", source="other"), json.dumps({"mode": "survey"}), None):
            self.ctl.on_request_text(bad)
        self.tick()
        self.assertEqual((self.admin.get_calls, self.admin.set_calls), (0, []))

    def test_manual_change_is_not_overwritten_until_the_request_changes(self):
        self.send("customer")
        self.tick()
        self.admin.mode = "survey"  # 管理画面からの手動切替
        for _ in range(4):
            self.clk.t += 5
            self.send("customer")
            self.tick(0.1)
        self.assertEqual(self.admin.mode, "survey")
        self.assertEqual(self.admin.set_calls, [])
        self.send("ocrtest", "e2")  # 要求が変わったら従う
        self.tick()
        self.assertEqual(self.admin.mode, "ocrtest")


class AnsweringGuardTests(Base):
    def test_no_switch_while_answering_and_pending_with_reason(self):
        self.admin.mode = "survey"
        self.admin.answer(True)
        self.send("customer")
        st = self.tick()
        self.assertEqual(self.admin.set_calls, [])
        self.assertEqual(self.admin.mode, "survey")
        self.assertEqual(st["state"], "pending")
        self.assertTrue(st["pending"])
        self.assertIn("回答中", st["pending_reason"])
        self.assertEqual(st["requested"], "customer")
        self.assertEqual(st["current"], "survey")

    def test_switches_after_the_answer_finishes(self):
        self.admin.mode = "survey"
        self.admin.answer(True)
        self.send("customer")
        self.tick()
        self.tick()
        self.assertEqual(self.admin.set_calls, [])
        self.admin.answer(False)
        st = self.tick()
        self.assertEqual(self.admin.set_calls, ["customer"])
        self.assertEqual(st["state"], "ok")
        self.assertEqual(st["pending_reason"], "")

    def test_session_state_answering_alone_also_blocks(self):
        self.admin.mode = "survey"
        self.admin.session_state = "answering"
        self.send("customer")
        self.assertEqual(self.tick()["state"], "pending")
        self.assertEqual(self.admin.set_calls, [])

    def test_unjudgeable_state_never_switches(self):
        self.admin.malformed = True
        self.send("survey")
        st = self.tick()
        self.assertEqual(self.admin.set_calls, [])
        self.assertEqual(st["state"], "pending")
        self.assertIn("判定できない", st["pending_reason"])

    def test_unreachable_raspi_never_switches_and_reports_error(self):
        self.admin.reachable = False
        self.send("survey")
        for _ in range(3):
            st = self.tick()
        self.assertEqual(self.admin.set_calls, [])
        self.assertEqual(st["state"], "error")
        self.assertIn("届かない", st["error"])
        self.admin.reachable = True  # 復旧したら切り替える
        st = self.tick()
        self.assertEqual(self.admin.set_calls, ["survey"])
        self.assertEqual(st["state"], "ok")

    def test_unconfigured_admin_reports_error_without_crashing(self):
        self.admin.configured = False
        self.send("survey")
        st = self.tick()
        self.assertEqual(st["state"], "error")
        self.assertEqual(self.admin.set_calls, [])


class FailureTests(Base):
    def test_set_mode_failure_is_reported_retried_later_and_does_not_leave_a_baseline(self):
        self.admin.set_fails = True
        self.send("survey")
        st = self.tick()
        self.assertEqual(st["state"], "error")
        self.assertIn("切り替えられません", st["error"])
        self.assertIsNone(st["baseline"])
        self.tick(1.0)  # 失敗直後は再試行を待つ
        self.assertEqual(self.admin.set_calls, ["survey"])
        self.admin.set_fails = False
        self.tick(sd.RETRY_AFTER_FAILURE_SEC)
        self.assertEqual(self.admin.set_calls, ["survey", "survey"])
        self.assertEqual(self.ctl.status()["state"], "ok")

    def test_exceptions_in_the_admin_are_not_propagated_by_the_timer_callback(self):
        class Boom(FakeAdmin):
            def get_state(self, fresh=False):
                raise RuntimeError("boom")

        self.admin = Boom()
        self.ctl = self.make()
        self.send("survey")
        # _timer_cb は別スレッド。例外はスレッド内で受けて状態に出す
        self.ctl._timer_cb()
        for _ in range(50):
            if self.ctl._busy.acquire(blocking=False):
                self.ctl._busy.release()
                break
            import time
            time.sleep(0.01)
        self.assertEqual(self.ctl.status()["state"], "error")


class RestoreTests(Base):
    def test_keep_request_returns_to_the_mode_before_the_schedule(self):
        self.send("survey")
        self.tick()
        self.send("keep", None)  # 項目が終わった
        st = self.tick()
        self.assertEqual(self.admin.set_calls, ["survey", "customer"])
        self.assertEqual((st["state"], st["baseline"]), ("idle", None))

    def test_next_entry_mode_wins_over_restoring(self):
        self.send("survey", "a")
        self.tick()
        self.send("ocrtest", "b")
        self.tick()
        self.send("keep", None)
        self.tick()
        self.assertEqual(self.admin.set_calls, ["survey", "ocrtest", "customer"])

    def test_restore_is_pending_while_answering(self):
        self.send("survey")
        self.tick()
        self.admin.answer(True)
        self.send("keep", None)
        st = self.tick()
        self.assertEqual(st["state"], "pending")
        self.assertTrue(st["restoring"])
        self.assertEqual(self.admin.mode, "survey")
        self.assertEqual(self.admin.set_calls, ["survey"])
        self.admin.answer(False)
        st = self.tick()
        self.assertEqual(self.admin.mode, "customer")
        self.assertEqual(st["state"], "idle")

    def test_stale_request_is_treated_as_keep(self):
        self.send("survey")
        self.tick()
        self.assertEqual(self.admin.mode, "survey")
        self.clk.t += sp.DISPLAY_REQUEST_TTL_SEC + 1  # directorが止まった
        self.ctl.tick()
        self.assertEqual(self.admin.mode, "customer")

    def test_heartbeat_keeps_the_request_alive(self):
        self.send("survey")
        self.tick()
        for _ in range(10):
            self.clk.t += 5
            self.send("survey")
            self.ctl.tick()
        self.assertEqual(self.admin.mode, "survey")
        self.assertEqual(self.admin.set_calls, ["survey"])

    def test_already_in_the_requested_mode_leaves_it_after_the_schedule(self):
        self.admin.mode = "survey"  # 元からアンケートだった
        self.send("survey")
        self.tick()
        self.send("keep", None)
        self.tick()
        self.assertEqual(self.admin.set_calls, [])
        self.assertEqual(self.admin.mode, "survey")

    def test_baseline_survives_a_dashboard_restart(self):
        self.send("survey")
        self.tick()
        self.assertTrue(os.path.exists(self.base_path))
        ctl2 = self.make()  # ダッシュボードの再起動
        self.assertEqual(ctl2.status()["baseline"], "customer")
        # 再起動直後は、directorの要求が届くのを待つ(すぐには戻さない)
        ctl2.tick()
        self.assertEqual(self.admin.mode, "survey")
        self.clk.t += 1
        ctl2.on_request_text(req("survey"))
        ctl2.tick()
        self.assertEqual(self.admin.mode, "survey")
        self.assertEqual(self.admin.set_calls, ["survey"])
        # 要求が来ないまま期限が過ぎたら戻す
        ctl3 = self.make()
        self.clk.t += sp.DISPLAY_REQUEST_TTL_SEC + 1
        ctl3.tick()
        self.assertEqual(self.admin.mode, "customer")
        self.assertFalse(os.path.exists(self.base_path))

    def test_no_baseline_file_is_left_when_nothing_was_switched(self):
        self.send("customer")
        self.tick()
        self.assertFalse(os.path.exists(self.base_path))


class WiringTests(unittest.TestCase):
    def test_schedule_controller_state_carries_display_status(self):
        src = (TEST_DIR.parent / "nokosahen_core" / "schedule_controller.py").read_text(encoding="utf-8")
        self.assertIn('out["display"]', src)
        node_src = (TEST_DIR.parent / "nokosahen_core" / "full_dashboard_node.py").read_text(encoding="utf-8")
        self.assertIn("DisplayScheduleController(", node_src)
        self.assertIn("self.schedule.display = self.schedule_display", node_src)

    def test_schedule_state_includes_display_and_survives_a_broken_display(self):
        from nokosahen_core.schedule_controller import ScheduleController

        class Tree:
            def live_state(self):
                return {"status": "ok", "state": {"schedule": {"running": True, "display_request": "survey"}}}

        sc = ScheduleController.__new__(ScheduleController)
        sc.tree = Tree()
        self.assertIsNone(sc.state()["display"])  # 切替の仕組みが無い構成
        admin = FakeAdmin(answering=True, mode="survey")
        sc.display = sd.DisplayScheduleController(admin, clock=Clk())
        sc.display.on_request_text(req("customer"))
        sc.display.tick()
        st = sc.state()
        self.assertEqual(st["display_request"], "survey")
        self.assertEqual((st["display"]["state"], st["display"]["requested"]), ("pending", "customer"))

        class Broken:
            def status(self):
                raise RuntimeError("x")

        sc.display = Broken()
        self.assertIsNone(sc.state()["display"])

    def test_controller_never_touches_robot_motion(self):
        src = (TEST_DIR.parent / "nokosahen_core" / "schedule_display.py").read_text(encoding="utf-8")
        for word in ("cmd_vel", "Twist", "goal", "create_publisher"):
            self.assertNotIn(word, src.replace("ゴール", ""), word)

    def test_subscription_and_timer_are_created_when_a_node_is_given(self):
        class Node:
            def __init__(self):
                self.subs, self.timers = [], []

            def create_subscription(self, typ, topic, cb, depth):
                self.subs.append(topic)

            def create_timer(self, sec, cb):
                self.timers.append(sec)

        n = Node()
        sd.DisplayScheduleController(FakeAdmin(), n, str)
        self.assertEqual(n.subs, ["/nokosahen/display_request"])
        self.assertEqual(n.timers, [sd.TICK_SEC])


if __name__ == "__main__":
    unittest.main()
