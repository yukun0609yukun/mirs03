"""向く旋回中の位置保持(nokosahen_core/position_hold.py と behavior_director の _hold_*)のテスト。ROS・実機なし。
シミュレーション(test/face_hold_sim.py)は実機で測ったずれ(左回りだけ約1.4 cm/rad、2026-11-06)とファームのランプを模す。
実行: python -m pytest -q -p no:cacheprovider test/test_position_hold.py
"""
import math
import random
import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import face_hold_sim as hs  # noqa: E402
from nokosahen_core import position_hold as ph  # noqa: E402

NOKOSAHEN_YAML = Path(__file__).resolve().parents[2] / "nokosahen_bringup" / "config" / "nokosahen.yaml"


class PureLogicTests(unittest.TestCase):
    def test_sector_min_range(self):
        n = 360
        inc = 2 * math.pi / n
        r = [math.inf] * n
        self.assertAlmostEqual(ph.sector_min_range(r, -math.pi, inc, 0.0, math.radians(30), 0.15, 12.0), 12.0)  # 全部 inf=遠い
        r[180] = 0.5  # 角度 0(前)
        self.assertAlmostEqual(ph.sector_min_range(r, -math.pi, inc, 0.0, math.radians(30), 0.15, 12.0), 0.5)
        self.assertAlmostEqual(ph.sector_min_range(r, -math.pi, inc, math.pi, math.radians(30), 0.15, 12.0), 12.0)  # 後ろは別
        nan = [float("nan")] * n
        self.assertIsNone(ph.sector_min_range(nan, -math.pi, inc, 0.0, math.radians(30), 0.15, 12.0))  # 測れない=不明
        self.assertIsNone(ph.sector_min_range([], -math.pi, inc, 0.0, 0.5, 0.15, 12.0))
        self.assertIsNone(ph.sector_min_range(r, -math.pi, 0.0, 0.0, 0.5, 0.15, 12.0))
        near0 = [0.05] * n  # range_min 未満(測れていない)
        self.assertIsNone(ph.sector_min_range(near0, -math.pi, inc, 0.0, 0.5, 0.15, 12.0))

    def test_predict_wz_uses_firmware_ramp(self):
        # 静止から 1.5 rad/s を指令: 加速 1.6 rad/s^2 → 0.3 秒後は 0.48
        self.assertAlmostEqual(ph.predict_wz(0.0, 1.5, 0.3, 1.6, 4.0), 0.48, places=2)
        # 反転: 減速 4.0 rad/s^2 → 0.3 秒後は 1.5 - 1.2 = 0.3
        self.assertAlmostEqual(ph.predict_wz(1.5, -1.5, 0.3, 1.6, 4.0), 0.3, places=2)

    def mk(self, **kw):
        return ph.PositionHold(ph.HoldParams(**kw))

    def test_learns_and_cancels_constant_drift_left_only(self):
        """左回りで odom が +0.021 m/s 進む(1.4 cm/rad × 1.5 rad/s)なら、c≈0.014 を学習して linear≈-0.021 を足す(上限 0.05)。"""
        h = self.mk()
        t = 0.0
        u = 0.0
        for _ in range(300):
            t += 0.1
            v = 0.014 * 1.5 + u
            u = h.step(t, 1.5, v_odom=v, wz_odom=1.5, pos=(0, 0, 0, "map"), clear_fwd=5.0, clear_back=5.0)
        self.assertAlmostEqual(h.c[1], 0.014, delta=0.0015)
        self.assertAlmostEqual(u, -0.021, delta=0.003)
        self.assertEqual(h.c[-1], 0.0)  # 右回りは学習していない

    def test_output_never_exceeds_limits_even_with_garbage(self):
        rng = random.Random(3)
        h = self.mk()
        t = 0.0
        for _ in range(3000):
            t += 0.1
            wz = rng.choice([1.5, -1.5, 0.0, 0.9, float("nan"), float("inf")])
            v = rng.choice([0.5, -0.5, 0.0, float("nan"), None, 3.0])
            pos = rng.choice([(0, 0, 0, "map"), (rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-3, 3), "map"), None,
                              (float("nan"), 0, 0, "map")])
            u = h.step(t, wz, v_odom=v, wz_odom=rng.choice([1.5, None, float("nan")]), pos=pos,
                       clear_fwd=rng.choice([5.0, None, 0.1, float("nan")]), clear_back=rng.choice([5.0, None, 0.1]),
                       allow=rng.random() > 0.1)
            self.assertTrue(math.isfinite(u))
            self.assertLessEqual(abs(u), 0.06 + 1e-9)

    def test_blocked_conditions_give_zero_immediately(self):
        h = self.mk()
        t = 0.0
        u = 0.0
        for _ in range(100):  # 補正が出ている状態にする
            t += 0.1
            u = h.step(t, 1.5, v_odom=0.021 + u, wz_odom=1.5, pos=(0, 0, 0, "map"), clear_fwd=5.0, clear_back=5.0)
        self.assertLess(u, -0.01)
        base = dict(wz_odom=1.5, pos=(0, 0, 0, "map"), clear_fwd=5.0, clear_back=5.0)
        cases = {
            "allow=False": dict(base, allow=False, v_odom=0.0),
            "odom stale": dict(base, v_odom=None),
            "pos stale": dict(base, v_odom=0.0, pos=None),
            "front/back unknown": dict(base, v_odom=0.0, clear_fwd=None, clear_back=None),
            "too close": dict(base, v_odom=0.0, clear_fwd=0.3, clear_back=0.3),
            "nan odom": dict(base, v_odom=float("nan")),
        }
        for name, kw in cases.items():
            hh = self.mk()
            tt = 0.0
            uu = 0.0
            for _ in range(100):
                tt += 0.1
                uu = hh.step(tt, 1.5, v_odom=0.021 + uu, wz_odom=1.5, pos=(0, 0, 0, "map"), clear_fwd=5.0, clear_back=5.0)
            self.assertLess(uu, -0.01, name)
            tt += 0.1
            self.assertEqual(hh.step(tt, 1.5, **kw), 0.0, name)

    def test_direction_specific_clearance(self):
        """進む方向の距離だけを見る。後退(左回りの補正)は後ろが近ければ出さない。前が近くても後退は出せる。"""
        def run(fwd, back):
            h = self.mk()
            t = 0.0
            u = 0.0
            for _ in range(100):
                t += 0.1
                u = h.step(t, 1.5, v_odom=0.021 + u, wz_odom=1.5, pos=(0, 0, 0, "map"), clear_fwd=fwd, clear_back=back)
            return u
        self.assertLess(run(0.2, 5.0), -0.01)   # 前が近い・後ろは広い → 後退の補正は出る
        self.assertEqual(run(5.0, 0.2), 0.0)    # 後ろが近い → 出さない
        self.assertEqual(run(5.0, 0.59), 0.0)   # 0.6 m 未満

    def test_outer_loop_hysteresis_and_limits(self):
        h = self.mk()
        t = 0.0
        outs = []
        for _ in range(10):  # まず原点でアンカーを取る
            t += 0.1
            h.step(t, 0.0, v_odom=0.0, wz_odom=0.0, pos=(0.0, 0.0, 0.0, "map"), clear_fwd=5.0, clear_back=5.0)
        x = 0.10  # 10 cm 前へずれている
        for i in range(300):
            t += 0.1
            u = h.step(t, 0.0, v_odom=0.0, wz_odom=0.0, pos=(x, 0.0, 0.0, "map"), clear_fwd=5.0, clear_back=5.0)
            outs.append(u)
            x += u * 0.1   # 理想的な追従
        self.assertLess(min(outs), -0.029)           # 後ろへ戻す(下限 0.03 以上)
        self.assertGreaterEqual(min(outs), -0.06 - 1e-9)  # 戻す速度の上限
        self.assertEqual(max(outs), 0.0)             # 前へは出さない
        self.assertLessEqual(abs(x), 0.03)           # stop_m 付近で戻し終わる
        self.assertEqual(h.state, ph.IDLE)
        self.assertEqual(h.episodes, 1)
        # 戻し終わった後(2.5〜7 cm の間)は、再び動き出さない=ヒステリシス
        t += 0.1
        self.assertEqual(h.step(t, 0.0, v_odom=0.0, wz_odom=0.0, pos=(0.05, 0.0, 0.0, "map"), clear_fwd=5.0, clear_back=5.0), 0.0)
        self.assertEqual(h.episodes, 1)

    def test_outer_loop_ignores_lateral_and_respects_total_limit(self):
        h = self.mk()
        t = 0.0
        for _ in range(60):  # 横に 12 cm(車体の向き 0 で y 方向)=戻せないので動かない
            t += 0.1
            self.assertEqual(h.step(t, 0.0, v_odom=0.0, wz_odom=0.0, pos=(0.0, 0.0, 0.0, "map"), clear_fwd=5, clear_back=5), 0.0)
        for _ in range(60):
            t += 0.1
            self.assertEqual(h.step(t, 0.0, v_odom=0.0, wz_odom=0.0, pos=(0.0, 0.12, 0.0, "map"), clear_fwd=5, clear_back=5), 0.0)
        # 上限(25 cm)超: 前後の成分があっても、以後ぜんぶ補正しない
        for _ in range(60):
            t += 0.1
            self.assertEqual(h.step(t, 0.0, v_odom=0.0, wz_odom=0.0, pos=(0.30, 0.0, 0.0, "map"), clear_fwd=5, clear_back=5), 0.0)
        self.assertTrue(h.over)
        t += 0.1
        self.assertEqual(h.step(t, 1.5, v_odom=0.02, wz_odom=1.5, pos=(0.30, 0.0, 0.0, "map"), clear_fwd=5, clear_back=5), 0.0)

    def test_outer_loop_timeout_and_stall(self):
        h = self.mk()
        t = 0.0
        for _ in range(30):
            t += 0.1
            h.step(t, 0.0, v_odom=0.0, wz_odom=0.0, pos=(0.0, 0.0, 0.0, "map"), clear_fwd=5, clear_back=5)
        # 動かない車体(位置が変わらない): 停滞で打ち切る
        outs = []
        for _ in range(100):
            t += 0.1
            outs.append(h.step(t, 0.0, v_odom=0.0, wz_odom=0.0, pos=(0.10, 0.0, 0.0, "map"), clear_fwd=5, clear_back=5))
        self.assertEqual(h.state, ph.IDLE)
        self.assertTrue(any(o < 0 for o in outs))
        self.assertEqual(outs[-1], 0.0)  # 停滞のあとは冷却中で出さない

    def test_gap_resets_anchor(self):
        h = self.mk()
        h.step(1.0, 0.0, v_odom=0.0, pos=(0.0, 0.0, 0.0, "map"), clear_fwd=5, clear_back=5)
        self.assertEqual(h.anchor, (0.0, 0.0))
        h.step(10.0, 0.0, v_odom=0.0, pos=(1.0, 0.0, 0.0, "map"), clear_fwd=5, clear_back=5)  # 9 秒空いた=新しい動作
        self.assertEqual(h.anchor, (1.0, 0.0))


class SimTests(unittest.TestCase):
    def run_sim(self, fn, *a, **kw):
        s = fn(*a, **kw)
        self.addCleanup(s.close)
        return s

    # ---- 既定(無効)----
    def test_default_is_off_and_in_place_turn_has_zero_linear(self):
        """既定は無効。無効のときは従来どおり、その場の旋回では前進は常に 0(コマンドは 0.0 そのもの)。"""
        for fn in (hs.run_sweep, hs.run_alternating):
            s = self.run_sim(fn, False, 40.0)
            self.assertEqual(s.max_abs_linear(), 0.0)
            self.assertTrue(all(r["hold"]["enabled"] is False and r["hold"]["ff_mps"] == 0.0 for r in s.log))
            self.assertTrue(any(abs(r["wz"]) > 0.5 for r in s.log))

    def test_director_default_class_attribute_is_off(self):
        self.assertFalse(hs.bd.BehaviorDirector._face_hold_enabled)

    def test_yaml_documents_default_off(self):
        text = NOKOSAHEN_YAML.read_text(encoding="utf-8")
        self.assertIn("face_position_hold_enabled: false", text)

    # ---- 補正OFFのずれの再現(実機の測定と合う)----
    def test_off_reproduces_measured_drift(self):
        # 実機: 左回り約 6.5 rad で 9.5 cm(1.46 cm/rad)。一方向の旋回(右へは回らない)でモデルを確かめる。
        s = hs.DriftSim(hold=False, robot=(0, 0, 0))
        self.addCleanup(s.close)
        s.n.face_turn_rps = 1.5
        for _ in range(int(6.5 / 1.0 * 10)):
            s.n._cmd_vel_pub.msgs.clear()
            s._plant(0.0, 1.0)  # 1.0 rad/s の左旋回を 6.5 秒
            s.log.append({"x": s.robot["x"], "y": s.robot["y"], "yaw": s.robot["yaw"]})
        # 旋回しながら進むので弧を描くが、移動した距離は k*角度
        path = sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(s.log, s.log[1:]))
        self.assertAlmostEqual(path, 0.014 * 6.5, delta=0.02)

    def test_off_sweep_drifts_far(self):
        s = self.run_sim(hs.run_sweep, False, 120.0)
        self.assertGreater(s.displacement(), 0.25)  # 約2分の往復で 25 cm 以上、前へ(測定どおり)
        self.assertGreater(s.forward_displacement(), 0.25)

    # ---- 補正ON ----
    def test_on_keeps_displacement_small_sweep_and_alternating(self):
        for seed in (1, 2, 3):
            s = self.run_sim(hs.run_sweep, True, 120.0, seed)
            self.assertLess(s.max_displacement(), 0.06, ("sweep", seed))
            self.assertLessEqual(s.max_abs_linear(), 0.05 + 1e-9)
            a = self.run_sim(hs.run_alternating, True, 120.0, seed)
            b = self.run_sim(hs.run_alternating, False, 120.0, seed)
            self.assertLess(a.max_displacement(), 0.10, ("alternating", seed))
            self.assertLess(a.max_displacement(), b.max_displacement() * 0.4)  # OFFの4割未満
            self.assertLessEqual(a.max_abs_linear(), 0.05 + 1e-9)

    def test_on_has_no_oscillation(self):
        """補正の符号が頻繁に反転しない(発振しない)。左回りの補正は後退向き1方向だけ。"""
        for seed in (1, 2):
            s = self.run_sim(hs.run_sweep, True, 120.0, seed)
            self.assertLessEqual(s.linear_sign_reversals(), 2)
            self.assertLessEqual(max((r["vx"] for r in s.log), default=0.0), 0.012)  # 右回りの補正(前向き)は小さい
            a = self.run_sim(hs.run_alternating, True, 120.0, seed)
            self.assertLessEqual(a.linear_sign_reversals(), 6)

    def test_on_does_not_worsen_alignment(self):
        for seed in (1, 2, 3):
            off, tgt = hs.run_single_turn(False, seed)
            on, _ = hs.run_single_turn(True, seed)
            self.addCleanup(off.close)
            self.addCleanup(on.close)
            self.assertEqual(off.first_aligned_time(), on.first_aligned_time())
            e_off = abs(math.degrees(hs.wrap(tgt - off.robot["yaw"])))
            e_on = abs(math.degrees(hs.wrap(tgt - on.robot["yaw"])))
            self.assertLessEqual(e_on, e_off + 1.0)
            self.assertLessEqual(on.displacement(), off.displacement() + 1e-6)
        # 往復の回数(整列の回数)も、OFFと同じ程度
        a = self.run_sim(hs.run_alternating, True, 120.0, 1)
        b = self.run_sim(hs.run_alternating, False, 120.0, 1)
        self.assertLessEqual(abs(a.n_aligned_events() - b.n_aligned_events()), 6)

    def test_outer_loop_returns_forward_back_component_only(self):
        for fwd, expect_back in ((0.12, True), (-0.12, True)):
            s = self.run_sim(hs.run_offset, fwd, 0.0)
            self.assertLess(abs(s.robot["x"]), 0.03)
            self.assertLessEqual(s.max_abs_linear(), 0.06 + 1e-9)
            self.assertEqual(s.linear_sign_reversals(), 0)
        lat = self.run_sim(hs.run_offset, 0.0, 0.12)   # 横ずれ: 戻さない(動かない)
        self.assertEqual(lat.max_abs_linear(), 0.0)
        self.assertAlmostEqual(lat.robot["y"], 0.12, places=2)
        far = self.run_sim(hs.run_offset, 0.30, 0.0)   # 上限超過: 動かず警告
        self.assertEqual(far.max_abs_linear(), 0.0)
        self.assertTrue(any(lv == "warn" for lv, _ in far.n._log.lines))
        # 戻している間に旋回が必要になったら、戻しは中断する(linear は 0 に戻る)
        s2 = hs.run_offset(0.12, 0.0, seconds=3.6)
        self.addCleanup(s2.close)

    # ---- 安全条件: 1つでも外れたら linear=0(補正が出るはずの同じ状況との対比つき)----
    def baseline_has_correction(self, **kw):
        s = self.run_sim(hs.run_sweep, True, 30.0, **kw)
        self.assertGreater(s.linear_nonzero_ticks(), 20, "対照: 条件が無ければ補正は出る")
        return s

    def assert_no_linear(self, s, turning_expected=True):
        self.assertEqual(s.max_abs_linear(), 0.0)
        if turning_expected:
            self.assertTrue(any(abs(r["wz"]) > 0.5 for r in s.log), "旋回は続く")

    def test_gate_odom_stale_and_missing(self):
        self.baseline_has_correction()
        self.assert_no_linear(self.run_sim(hs.run_sweep, True, 30.0, odom_ok=False))
        s = hs.DriftSim(hold=True, robot=(0, 0, 0), odom_ok=False)
        self.addCleanup(s.close)
        s.hooks.append(lambda sim: setattr(sim.n, "_hold_odom", (sim.n._clock.now().nanoseconds / 1e9, float("nan"), 0.0, 0.0, 0.0, 1.5)))
        s.run(30.0)
        self.assert_no_linear(s)

    def test_gate_scan_stale_or_blocked(self):
        self.assert_no_linear(self.run_sim(hs.run_sweep, True, 30.0, scan_ok=False))   # 距離が取れない/古い
        ring = [(0.45 * math.cos(math.radians(a)), 0.45 * math.sin(math.radians(a))) for a in range(0, 360, 10)]
        s = self.run_sim(hs.run_sweep, True, 30.0, obstacles=ring)                      # 前後とも 0.45 m に物がある
        self.assert_no_linear(s)

    def test_gate_obstacle_only_behind_blocks_backward(self):
        # 左回りの補正は後退向き。後ろ(世界の -x)に物があり、車体がほぼ前向きの間だけ後ろの扇形に入る。
        # 補正は、後ろに物が見えているtickでは 0。見えていないtickでは出る。
        s = hs.DriftSim(hold=True, robot=(0, 0, 0), obstacles=[(-0.45, 0.0)] + [(-0.45, dy) for dy in (-0.1, 0.1)])
        self.addCleanup(s.close)
        s.run(60.0)
        seen_back_blocked = 0
        prev = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        for r in s.log:
            # tickの判断は、その1つ前の姿勢で行われる。物(世界の (-0.45, 0))の、機体基準の方位と距離
            dx, dy = -0.45 - prev["x"], 0.0 - prev["y"]
            ang = hs.wrap(math.atan2(dy, dx) - prev["yaw"])
            blocked = abs(hs.wrap(ang - math.pi)) <= math.radians(20) and math.hypot(dx, dy) < 0.55
            if blocked:
                seen_back_blocked += 1
                self.assertGreaterEqual(r["vx"], -1e-12, r)  # 後退の補正は出ていない
            prev = r
        self.assertGreater(seen_back_blocked, 10)

    def test_gate_near_person_stop_exempt_turn(self):
        p = hs.Person(0.6, 0.4)
        s = hs.DriftSim(hold=True, robot=(0, 0, 0), people=[p], near_person=True)
        self.addCleanup(s.close)
        s.run(20.0)
        self.assertIn("FACE_PERSON", s.modes())
        self.assertEqual(s.max_abs_linear(), 0.0)
        self.assertEqual(s.n._hold_block_reason(), "near_person")

    def test_gate_estop_manual_survey_mapping_camera(self):
        def mk(**kw):
            s = hs.DriftSim(hold=True, robot=(0, 0, 0), **kw)
            self.addCleanup(s.close)
            return s

        def estop(sim):
            sim.n._hardware_estop = True

        def manual(sim):
            sim.n._manual_override_raw = True
            sim.n._last_manual_override_time = sim.n._clock.now()

        def survey(sim):
            sim.n._survey_active = True
            sim.n._last_survey_active_time = sim.n._clock.now()

        def mapping(sim):
            sim.n._map_building_active = True

        for name, hook, reason, turning in (("estop", estop, "estop", False), ("manual", manual, "manual", False),
                                           ("survey", survey, "survey", False), ("mapping", mapping, "mapping", False)):
            s = mk()
            s.hooks.append(hook)
            s.run(15.0)
            self.assertEqual(s.max_abs_linear(), 0.0, name)
            self.assertEqual(s.n._hold_block_reason(), reason, name)
        # カメラ障害物: 旋回だけは免除で続くが、前進/後退の補正は出さない
        def cam(sim):
            sim.n._camera_obstacle_detected = True
            sim.n._last_camera_obstacle_receive_time = sim.n._clock.now()

        s = mk()
        s.hooks.append(cam)
        s.run(30.0)
        self.assert_no_linear(s)
        self.assertEqual(s.n._hold_block_reason(), "camera_obstacle")

    def test_gate_map_not_loaded(self):
        s = hs.DriftSim(hold=True, robot=(0, 0, 0), pose_ok=False)
        self.addCleanup(s.close)
        s.n.require_map_loaded = True
        s.run(15.0)
        self.assertEqual(s.max_abs_linear(), 0.0)
        self.assertEqual(s.n._hold_block_reason(), "map_not_loaded")

    def test_gate_boundary_edge_never_drives_out(self):
        from nokosahen_core import boundary_motion_guard as bmg
        from nokosahen_core import map_orientation as mo
        rect = mo.make_rect(0.0, 0.0, 1.4, 1.4, 0.0)   # 狭い範囲: 旋回はよいが、先へ進むと機体が縁に近い
        s = hs.DriftSim(hold=True, robot=(0.0, 0.0, 0.0), boundary=rect)
        self.addCleanup(s.close)
        s.run(60.0)
        prev = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        for r in s.log:
            if abs(r["vx"]) > 0.0:   # 判断は1つ前の姿勢で行われる
                self.assertTrue(bmg.can_drive(rect, {"x": prev["x"], "y": prev["y"], "yaw": prev["yaw"]}, 1.0 if r["vx"] > 0 else -1.0), r)
            prev = r
        self.assertTrue(s.inside_always())

    def test_gate_no_grant(self):
        s = hs.DriftSim(hold=True, robot=(0, 0, 0))
        self.addCleanup(s.close)
        s.n._direct_grant = hs.bd.GrantTracker("director_direct", 1.0, True)
        s.n._direct_grant.on_message('{"allow": {"director_direct": false}}', s.n._now_sec())  # arbiterが許可していない
        self.assertEqual(s.n._hold_block_reason(), "no_grant")
        s.n._direct_grant.on_message('{"allow": {"director_direct": true}}', s.n._now_sec())
        self.assertEqual(s.n._hold_block_reason(), "")

    # ---- 状態の出力・パラメータ・購読 ----
    def test_command_json_face_hold_status(self):
        s = self.run_sim(hs.run_sweep, True, 30.0)
        fh = s.log[-1]["hold"]
        for k in ("enabled", "active", "drift_m", "ff_mps", "anchor"):
            self.assertIn(k, fh)
        self.assertTrue(fh["enabled"])
        self.assertTrue(any(r["hold"]["active"] for r in s.log))
        self.assertIsNotNone(fh["anchor"])
        lines = [m for _lv, m in s.n._log.lines]
        self.assertTrue(any("アンカーを記録" in m for m in lines))
        self.assertTrue(any("補正を開始" in m for m in lines))

    def test_runtime_toggle_via_param_callback(self):
        mod = types.ModuleType("rcl_interfaces.msg")
        mod.SetParametersResult = lambda successful: types.SimpleNamespace(successful=successful)
        pkg = types.ModuleType("rcl_interfaces")
        old = {k: sys.modules.get(k) for k in ("rcl_interfaces", "rcl_interfaces.msg")}
        sys.modules["rcl_interfaces"], sys.modules["rcl_interfaces.msg"] = pkg, mod
        try:
            s = hs.DriftSim(hold=False, robot=(0, 0, 0))
            self.addCleanup(s.close)
            s.run(10.0)
            self.assertEqual(s.max_abs_linear(), 0.0)
            res = s.n._on_hold_params([types.SimpleNamespace(name="face_position_hold_enabled", value=True)])
            self.assertTrue(res.successful)
            self.assertTrue(s.n._face_hold_enabled)
            s.run(30.0)
            self.assertGreater(s.linear_nonzero_ticks(), 5)
            s.n._on_hold_params([types.SimpleNamespace(name="face_position_hold_enabled", value=False)])
            n_before = len(s.log)
            s.run(10.0)
            self.assertTrue(all(r["vx"] == 0.0 for r in s.log[n_before:]))   # 無効に戻すと即 0
        finally:
            for k, v in old.items():
                if v is None:
                    sys.modules.pop(k, None)
                else:
                    sys.modules[k] = v

    def test_odom_and_scan_callbacks_parse_messages(self):
        s = hs.DriftSim(hold=False, robot=(0, 0, 0))
        self.addCleanup(s.close)
        n = s.n
        q = types.SimpleNamespace(x=0.0, y=0.0, z=math.sin(0.25), w=math.cos(0.25))
        msg = types.SimpleNamespace(
            pose=types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=1.0, y=2.0), orientation=q)),
            twist=types.SimpleNamespace(twist=types.SimpleNamespace(linear=types.SimpleNamespace(x=0.02), angular=types.SimpleNamespace(z=1.4))))
        n._on_hold_odom(msg)
        self.assertAlmostEqual(n._hold_odom[1], 0.02)
        self.assertAlmostEqual(n._hold_odom[4], 0.5, places=6)
        self.assertAlmostEqual(n._hold_odom[5], 1.4)
        n._on_hold_odom(types.SimpleNamespace())   # 壊れたメッセージ
        self.assertIsNone(n._hold_odom)
        now = n._now_sec()
        scan = hs.make_scan(s.robot, [(0.5, 0.0)])
        n._hold_scan = (now, scan)
        n._hold_odom = (now, 0.0, 0.0, 0.0, 0.0, 0.0)
        pos, v, wz, fwd, back = n._hold_inputs(now)
        self.assertLess(fwd, 0.6)
        self.assertGreater(back, 5.0)
        self.assertEqual(pos[3], "map")
        pos, v, wz, fwd, back = n._hold_inputs(now + 1.0)   # 1秒後=古い
        self.assertIsNone(v)
        self.assertIsNone(fwd)

    def test_hold_error_never_stops_turning(self):
        s = hs.DriftSim(hold=True, robot=(0, 0, 0))
        self.addCleanup(s.close)
        s.n._hold_get = lambda: 1 / 0
        s.run(10.0)
        self.assertEqual(s.max_abs_linear(), 0.0)
        self.assertTrue(any(abs(r["wz"]) > 0.5 for r in s.log))
        self.assertTrue(any("位置保持でエラー" in m for _lv, m in s.n._log.lines))


if __name__ == "__main__":
    unittest.main()
