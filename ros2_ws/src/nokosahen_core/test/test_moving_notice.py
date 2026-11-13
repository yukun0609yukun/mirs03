"""「移動中です。ご注意ください」の判定(moving_notice.py)のテスト。ROS不要の純Python。2026-10-27

  - ヒステリシス: 動き始め(0.05 m/s以上が0.5秒)・止まった(0.02 m/s未満が1.5秒)・ふらつき・壊れた値
  - 動き×出す/出さない(director の mode 全種)、アンケート・呼びかけ・発話・緊急停止での抑止、データが古いとき
  - 画面の最短表示時間(3秒)と、すぐ消す理由(保持を適用しない)
  - 動作に影響しない(cmd_vel・ゴール・rclpyを使わない)
実行: python3 ros2_ws/src/nokosahen_core/test/test_moving_notice.py
"""
import re
import sys
import unittest
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from nokosahen_core import moving_notice as mn  # noqa: E402


def run_speeds(det, samples):
    """samples=[(t, speed)]。各サンプルで update し、最後の moving を返す。"""
    r = False
    for t, v in samples:
        r = det.update(v, t)
    return r


class DetectorTest(unittest.TestCase):
    def test_start_needs_005_for_half_a_second(self):
        d = mn.MovingDetector()
        self.assertFalse(run_speeds(d, [(0.0, 0.06), (0.2, 0.06), (0.4, 0.06)]))
        self.assertTrue(d.update(0.06, 0.5))                       # 0.5秒つづいた
        d = mn.MovingDetector()
        # 途中で0.05未満が混ざったら数え直し(0.5から0.9は0.4秒)
        self.assertFalse(run_speeds(d, [(0.0, 0.06), (0.3, 0.06), (0.4, 0.04), (0.5, 0.06), (0.9, 0.06)]))
        self.assertTrue(d.update(0.06, 1.0))

    def test_below_005_never_starts(self):
        d = mn.MovingDetector()
        self.assertFalse(run_speeds(d, [(t / 10, 0.049) for t in range(0, 100)]))

    def test_stop_needs_below_002_for_1_5s(self):
        d = mn.MovingDetector()
        run_speeds(d, [(t / 10, 0.2) for t in range(0, 10)])
        self.assertTrue(d.moving)
        self.assertTrue(run_speeds(d, [(1.0, 0.01), (1.5, 0.01), (2.4, 0.01)]))   # まだ1.4秒
        self.assertFalse(d.update(0.01, 2.5))                                     # 1.5秒つづいた

    def test_wobble_between_the_thresholds_does_not_blink(self):
        d = mn.MovingDetector()
        run_speeds(d, [(t / 10, 0.2) for t in range(0, 10)])
        # 停止直前のふらつき(0.02〜0.05)では解除しない。0.01が瞬間に混ざっても1.5秒つづかなければ解除しない
        seq = [(1.0 + i * 0.1, v) for i, v in enumerate([0.04, 0.03, 0.025, 0.01, 0.03, 0.01, 0.04, 0.02, 0.01, 0.03] * 3)]
        self.assertTrue(run_speeds(d, seq))
        # 止まっている間の微小なノイズ(0.03〜0.045)で動き始めたことにしない
        d = mn.MovingDetector()
        self.assertFalse(run_speeds(d, [(t / 10, 0.03 + (0.015 if t % 7 == 0 else 0)) for t in range(0, 200)]))

    def test_reverse_speed_counts(self):
        d = mn.MovingDetector()
        self.assertTrue(run_speeds(d, [(0.0, -0.2), (0.6, -0.2)]))

    def test_broken_values_never_mean_moving_and_clear_it(self):
        for bad in (None, float("nan"), float("inf"), "0.3", True, [], 99.0):
            d = mn.MovingDetector()
            self.assertFalse(run_speeds(d, [(0.0, bad), (1.0, bad), (2.0, bad)]), bad)
        d = mn.MovingDetector()
        run_speeds(d, [(0.0, 0.3), (0.6, 0.3)])
        self.assertTrue(d.moving)
        self.assertFalse(d.update(float("nan"), 0.7))              # 壊れた値で「動いている」を引きずらない


def beh(mode, **kw):
    return {"mode": mode, **kw}


class DecideTest(unittest.TestCase):
    def d(self, mode, moving=True, **kw):
        extra = {k: kw.pop(k) for k in ("estop_active",) if k in kw}
        return mn.decide(moving, beh(mode, **extra), kw.pop("beh_age", 0.1), kw.pop("odom_age", 0.1), **kw)

    def test_the_chosen_movements_are_shown_when_moving(self):
        for mode, kind in (("POLICY_GO_PIN", "go_pin"), ("RETURN_HOME", "return_home"), ("MANUAL_OVERRIDE", "manual"),
                           ("MAPPING_ACTIVE_HOLD", "exploring"), ("POLICY_GO_ENTRANCE", "go_entrance")):
            r = self.d(mode)
            self.assertTrue(r["notice"], mode)
            self.assertEqual((r["kind"], r["reason"]), (kind, ""), mode)

    def test_everything_else_is_not_shown_even_when_moving(self):
        for mode in ("APPROACH_PERSON", "CROWD_SCAN", "CROWD_TURN_TO_CROWD", "CROWD_APPROACH", "IDLE_WIGGLE", "POLICY_STAY",
                     "POLICY_AT_PIN", "POLICY_AT_ENTRANCE", "POLICY_PIN_UNAVAILABLE", "POLICY_NO_SCAN", "RETURN_HOME_ARRIVED",
                     "RETURN_HOME_TIMEOUT", "WAIT_FOR_SENSOR", "NEAR_PERSON_HOLD", "OUT_OF_BOUNDS_HOLD", "KEEPOUT_ZONE_HOLD",
                     "CAMERA_OBSTACLE_HOLD", "APPROACH_ARRIVED", "STOP_ANNOUNCE", "SURVEY_ACTIVE_HOLD", "ESTOP_HOLD", "SOMETHING_NEW"):
            r = self.d(mode)
            self.assertFalse(r["notice"], mode)

    def test_not_moving_means_no_notice_even_in_a_listed_mode(self):
        r = self.d("POLICY_GO_PIN", moving=False)
        self.assertFalse(r["notice"])
        self.assertEqual(r["reason"], mn.R_NOT_MOVING)

    def test_survey_announce_speaking_estop_suppress(self):
        self.assertEqual(self.d("POLICY_GO_PIN", survey_answering=True)["reason"], mn.R_SURVEY)
        self.assertEqual(self.d("SURVEY_ACTIVE_HOLD")["reason"], mn.R_SURVEY)
        self.assertEqual(self.d("STOP_ANNOUNCE")["reason"], mn.R_ANNOUNCE)
        self.assertEqual(self.d("APPROACH_ARRIVED")["reason"], mn.R_ANNOUNCE)
        self.assertEqual(self.d("POLICY_GO_PIN", speaking=True)["reason"], mn.R_SPEAKING)
        self.assertEqual(self.d("MANUAL_OVERRIDE", estop_active=True)["reason"], mn.R_ESTOP)
        self.assertEqual(self.d("MAPPING_ACTIVE_HOLD", survey_answering=True)["reason"], mn.R_SURVEY)   # 探索中でもアンケート中は出さない

    def test_display_mode_gate_all_combinations(self):
        """【2026-10-30】iPad表示が有効なら customer のときだけ出す(画面を出さないときは音も鳴らさない)。無効(旧ラズパイ方式)は従来どおり。"""
        for mode in mn.NOTICE_MODES:
            for enabled in (False, True):
                for dm in ("customer", "survey", "inventory", "ocrtest", None, "", "Customer", 5):
                    for moving in (True, False):
                        r = self.d(mode, moving=moving, display_enabled=enabled, display_mode=dm)
                        if enabled and dm != "customer":
                            self.assertEqual((r["notice"], r["reason"]), (False, mn.R_DISPLAY_MODE), (mode, enabled, dm, moving))
                        else:
                            self.assertEqual(r["notice"], moving, (mode, enabled, dm, moving))
                            self.assertEqual(r["reason"], "" if moving else mn.R_NOT_MOVING)
        # 優先順: 古いデータ・緊急停止・アンケート・発話の理由が、表示モードの理由より先に出る
        self.assertEqual(self.d("POLICY_GO_PIN", display_enabled=True, display_mode="survey", odom_age=None)["reason"], mn.R_ODOM_STALE)
        self.assertEqual(self.d("MANUAL_OVERRIDE", display_enabled=True, display_mode="survey", estop_active=True)["reason"], mn.R_ESTOP)
        self.assertEqual(self.d("POLICY_GO_PIN", display_enabled=True, display_mode="survey", survey_answering=True)["reason"], mn.R_SURVEY)
        self.assertEqual(self.d("POLICY_GO_PIN", display_enabled=True, display_mode="survey", speaking=True)["reason"], mn.R_SPEAKING)
        # 出さない動きは、モードが何でも従来の理由
        self.assertEqual(self.d("IDLE_WIGGLE", display_enabled=True, display_mode="customer")["reason"], mn.R_MODE)

    def test_stale_or_missing_data_never_shows(self):
        self.assertEqual(self.d("POLICY_GO_PIN", odom_age=mn.ODOM_STALE_S + 0.1)["reason"], mn.R_ODOM_STALE)
        self.assertEqual(self.d("POLICY_GO_PIN", odom_age=None)["reason"], mn.R_ODOM_STALE)
        self.assertEqual(self.d("POLICY_GO_PIN", beh_age=mn.BEHAVIOR_STALE_S + 0.1)["reason"], mn.R_BEHAVIOR_STALE)
        self.assertEqual(self.d("POLICY_GO_PIN", beh_age=None)["reason"], mn.R_BEHAVIOR_STALE)
        for bad in (None, {}, {"mode": ""}, {"mode": 5}, [], "x"):
            r = mn.decide(True, bad, 0.1, 0.1)
            self.assertFalse(r["notice"], bad)

    def test_the_listed_modes_exist_in_behavior_director(self):
        src = (PKG / "nokosahen_core" / "behavior_director.py").read_text(encoding="utf-8")
        for mode in list(mn.NOTICE_MODES) + list(mn.ANNOUNCE_MODES) + list(mn.SURVEY_MODES):
            self.assertIn(f'"{mode}"', src, mode)


class NoticeTest(unittest.TestCase):
    def feed(self, n, t, speed=0.2, mode="POLICY_GO_PIN", **beh_kw):
        n.on_odom(speed, t)
        n.on_behavior({"mode": mode, **beh_kw}, t)

    def walk(self, n, t0, t1, speed=0.2, mode="POLICY_GO_PIN", step=0.1, beh_kw=None, **kw):
        t = t0
        r = None
        while t <= t1 + 1e-9:
            self.feed(n, t, speed, mode, **(beh_kw or {}))
            r = n.evaluate(t, **kw)
            t += step
        return r

    def test_start_then_sound_and_display(self):
        n = mn.MovingNotice()
        r = self.walk(n, 0.0, 0.4)
        self.assertFalse(r["sound"] or r["display"])
        r = self.walk(n, 0.5, 1.0)
        self.assertTrue(r["sound"] and r["display"])
        self.assertEqual(r["kind"], "go_pin")

    def test_exploring_shows_the_banner_but_plays_no_sound(self):
        # 2026-10-27(ユーザー指示): 探索中は既存の「探索中です」が鳴るので、「移動しています」の音は鳴らさない。画面の表示は出す
        n = mn.MovingNotice()
        r = self.walk(n, 0.0, 1.0, mode="MAPPING_ACTIVE_HOLD")
        self.assertTrue(r["display"])
        self.assertFalse(r["sound"])
        self.assertEqual(r["kind"], "exploring")
        for mode in ("POLICY_GO_PIN", "RETURN_HOME", "MANUAL_OVERRIDE", "POLICY_GO_ENTRANCE"):
            n = mn.MovingNotice()
            r = self.walk(n, 0.0, 1.0, mode=mode)
            self.assertTrue(r["sound"] and r["display"], mode)

    def test_display_mode_gate_silences_sound_and_display_together(self):
        # customer 以外(survey の待機中・inventory・ocrtest・不明)では、動いていても画面も音も出ない。customer に戻ると出る。切替直後に保持で残らない
        for dm in ("survey", "inventory", "ocrtest", None):
            n = mn.MovingNotice()
            r = self.walk(n, 0.0, 2.0, display_enabled=True, display_mode=dm)
            self.assertEqual((r["sound"], r["display"], r["reason"]), (False, False, mn.R_DISPLAY_MODE), dm)
        n = mn.MovingNotice()
        r = self.walk(n, 0.0, 2.0, display_enabled=True, display_mode="customer")
        self.assertTrue(r["sound"] and r["display"])
        r = self.walk(n, 2.1, 2.3, display_enabled=True, display_mode="survey")      # 表示モードが変わったら、最短表示の保持を待たずに両方消える
        self.assertEqual((r["sound"], r["display"]), (False, False))
        r = self.walk(n, 2.4, 3.0, display_enabled=True, display_mode="customer")
        self.assertTrue(r["sound"] and r["display"])
        # iPad表示が無効(旧ラズパイ方式)なら、表示モードに関係なく従来どおり
        r = self.walk(mn.MovingNotice(), 0.0, 2.0, display_enabled=False, display_mode="survey")
        self.assertTrue(r["sound"] and r["display"])
        r = self.walk(mn.MovingNotice(), 0.0, 2.0)
        self.assertTrue(r["sound"] and r["display"])
        # 探索中: customer なら画面だけ(音なしは既存どおり)、customer 以外なら両方なし
        r = self.walk(mn.MovingNotice(), 0.0, 2.0, mode="MAPPING_ACTIVE_HOLD", display_enabled=True, display_mode="customer")
        self.assertEqual((r["display"], r["sound"]), (True, False))
        r = self.walk(mn.MovingNotice(), 0.0, 2.0, mode="MAPPING_ACTIVE_HOLD", display_enabled=True, display_mode="survey")
        self.assertEqual((r["display"], r["sound"]), (False, False))

    def test_long_trip_then_stop_clears_sound_and_display_together(self):
        n = mn.MovingNotice()
        self.walk(n, 0.0, 10.0)
        r = self.walk(n, 10.1, 11.5, speed=0.0)                 # 止まって1.5秒つづくまでは「動いている」のまま
        self.assertTrue(r["sound"])
        r = self.walk(n, 11.6, 11.7, speed=0.0)
        self.assertFalse(r["sound"])
        self.assertFalse(r["display"])                          # 出し始めから3秒以上たっているので保持しない

    def test_short_trip_keeps_the_banner_for_min_show_but_not_the_sound(self):
        n = mn.MovingNotice()
        self.walk(n, 0.0, 0.6)                                  # 0.5秒で表示開始
        r = self.walk(n, 0.7, 1.0, speed=0.0)
        self.assertTrue(r["sound"])                             # 速さ0が1.5秒つづくまで moving は保たれる
        r = self.walk(n, 1.1, 2.3, speed=0.0)                   # 2.2秒で解除。表示開始0.5+3.0=3.5秒まで保持
        self.assertFalse(r["sound"])
        self.assertTrue(r["display"])
        self.assertEqual(r["kind"], "go_pin")
        r = self.walk(n, 2.4, 3.4, speed=0.0)
        self.assertTrue(r["display"])
        r = self.walk(n, 3.5, 3.6, speed=0.0)
        self.assertFalse(r["display"])

    def test_mode_change_to_an_unlisted_movement_is_held_but_announce_mode_is_immediate(self):
        n = mn.MovingNotice()
        self.walk(n, 0.0, 1.0)
        r = self.walk(n, 1.1, 1.3, mode="IDLE_WIGGLE")          # 出さない動きに変わった: 保持(ちらつき防止)
        self.assertFalse(r["sound"])
        self.assertTrue(r["display"])
        r = self.walk(n, 1.4, 1.5, mode="STOP_ANNOUNCE")        # 呼びかけ: すぐ消す(人に画面を見てもらう)
        self.assertFalse(r["display"])
        self.assertEqual(r["reason"], mn.R_ANNOUNCE)

    def test_immediate_reasons_ignore_the_hold(self):
        for kw, reason in (({"survey_answering": True}, mn.R_SURVEY), ({"speaking": True}, mn.R_SPEAKING)):
            n = mn.MovingNotice()
            self.walk(n, 0.0, 1.0)
            r = self.walk(n, 1.1, 1.2, **kw)
            self.assertFalse(r["display"] or r["sound"], kw)
            self.assertEqual(r["reason"], reason)
        n = mn.MovingNotice()
        self.walk(n, 0.0, 1.0)
        r = self.walk(n, 1.1, 1.2, mode="MANUAL_OVERRIDE", beh_kw={"estop_active": True})
        self.assertFalse(r["display"] or r["sound"])
        self.assertEqual(r["reason"], mn.R_ESTOP)

    def test_stale_odom_clears_at_once_and_does_not_resume_from_old_state(self):
        n = mn.MovingNotice()
        self.walk(n, 0.0, 1.0)
        t, r = 1.1, None
        while t < 4.0:                                          # オドメトリだけ途絶(directorは来る)
            n.on_behavior({"mode": "POLICY_GO_PIN"}, t)
            r = n.evaluate(t)
            t += 0.1
        self.assertFalse(r["display"] or r["sound"])
        self.assertEqual(r["reason"], mn.R_ODOM_STALE)
        r = self.walk(n, 4.0, 4.3)                              # 復帰しても0.5秒つづくまでは出さない
        self.assertFalse(r["display"])

    def test_stale_director_clears_at_once(self):
        n = mn.MovingNotice()
        self.walk(n, 0.0, 1.0)
        t, r = 1.1, None
        while t < 4.5:
            n.on_odom(0.2, t)
            r = n.evaluate(t)
            t += 0.1
        self.assertFalse(r["display"] or r["sound"])
        self.assertEqual(r["reason"], mn.R_BEHAVIOR_STALE)

    def test_never_seen_data_shows_nothing(self):
        r = mn.MovingNotice().evaluate(100.0)
        self.assertFalse(r["sound"] or r["display"])

    def test_bad_inputs_do_not_raise(self):
        n = mn.MovingNotice()
        n.on_odom("x", 0.0)
        n.on_odom(None, 0.1)
        n.on_behavior("junk", 0.2)
        n.on_behavior(None, 0.2)
        r = n.evaluate(0.3, survey_answering=None, speaking=None)
        self.assertFalse(r["display"])

    def test_notice_resumes_on_a_new_trip(self):
        n = mn.MovingNotice()
        self.walk(n, 0.0, 1.0)
        self.walk(n, 1.1, 6.0, speed=0.0)
        r = self.walk(n, 6.1, 7.0)
        self.assertTrue(r["sound"] and r["display"])


class SafetyTest(unittest.TestCase):
    """表示と音の通知だけ。動作(速度指令・ゴール・停止)には触れない。"""

    def test_module_has_no_ros_and_no_motion_output(self):
        src = (PKG / "nokosahen_core" / "moving_notice.py").read_text(encoding="utf-8")
        code = "\n".join(l for l in src.splitlines() if not l.strip().startswith("#"))
        code = re.sub(r'""".*?"""', "", code, flags=re.S)
        for needle in ("rclpy", "cmd_vel", "Twist", "goal_pose", "publish", "subprocess", "socket"):
            self.assertNotIn(needle, code, needle)

    def test_dashboard_wiring_only_reads_and_publishes_the_notice_topic(self):
        src = (PKG / "nokosahen_core" / "full_dashboard_node.py").read_text(encoding="utf-8").replace("\r\n", "\n")
        m = re.search(r"def _moving_tick\(self\).*?\n    def ", src, re.S)
        self.assertIsNotNone(m)
        body = m.group(0)
        self.assertIn("except Exception", body)                 # 例外で本体を止めない
        self.assertEqual(re.findall(r"\.publish\(([^)]*)\)", body), ["out"])
        self.assertIn('"/nokosahen/moving_notice"', src)
        self.assertIn("self._moving_notice.on_odom(msg.twist.twist.linear.x", src)

    def test_announce_node_reads_the_notice_topic(self):
        src = (PKG / "nokosahen_core" / "announce_node.py").read_text(encoding="utf-8")
        self.assertIn('"/nokosahen/moving_notice"', src)
        self.assertIn("on_moving_notice", src)


if __name__ == "__main__":
    unittest.main(verbosity=1)
