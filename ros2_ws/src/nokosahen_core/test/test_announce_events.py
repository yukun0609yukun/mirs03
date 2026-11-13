"""状況アナウンス(1台目側、announce_events.py / announce_node.py)のテスト。docs/announcements.md

ロボット・ROS・2台目には触れない(rclpyはスタブ、voice_bridgeはhttp.serverのスタブ、時刻は引数)。
1. 定義表の検証(範囲・型・未知のキー/イベント)と保存(原子的+.bak、部分更新)、ファイル無し=全体OFF、不正ファイルは直前の設定を維持
2. AnnounceManager: ループの開始/停止/生きている信号の再送/設定変更、once+クールダウン、min_active、
   人が近い抑制、アンケート中OFF(設定で変更可)、緊急停止の専用経路(urgent・緊急停止中でも出る)、全体OFF/個別OFF
3. SituationBuilder: 既存トピックの値からフラグを作る(失効・後退・探索完了・手動運転中の後退除外)
4. AnnounceSender: voice_bridgeへのHTTP、マスターOFF中は送らない(停止は送る)、届かないとき静かに諦める(ログは間引き)
5. 既定の動作: 定義表が無い/全OFFなら何も送らない。announce_nodeは動作系のトピックを出さない

実行: python ros2_ws/src/nokosahen_core/test/test_announce_events.py
"""

import json
import os
import sys
import tempfile
import threading
import types
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

for _n in ("rclpy", "rclpy.node", "geometry_msgs", "geometry_msgs.msg", "std_msgs", "std_msgs.msg"):
    if _n not in sys.modules:
        try:
            __import__(_n)
        except Exception:  # noqa: BLE001
            m = types.ModuleType(_n)
            sys.modules[_n] = m
            if _n == "rclpy.node":
                m.Node = type("Node", (), {})
            if _n == "geometry_msgs.msg":
                m.Twist = type("Twist", (), {})
            if _n == "std_msgs.msg":
                m.Bool = type("Bool", (), {})
                m.String = type("String", (), {})

from nokosahen_core import announce_events as ae  # noqa: E402
from nokosahen_core import announce_node as an  # noqa: E402

TMP = tempfile.mkdtemp(prefix="announce_events_test_")


def cfg(enabled=True, **over):
    """全イベントがそろった設定。over={イベントid: {項目: 値}}。"""
    c = ae.merged_config({"enabled": enabled}, True)
    for eid, f in over.items():
        c["events"][eid].update(f)
    return c


def ctx(**k):
    return {"survey": False, "person_near": False, "estop": False, **k}


def flags(**k):
    return {t: False for t in ae.FLAGS} | k


class ValidationTest(unittest.TestCase):
    def test_defaults_are_valid(self):
        for eid in ae.EVENT_DEFS:
            f, e = ae.validate_event_fields(eid, ae.default_event(eid))
            self.assertEqual(e, [], eid)

    def test_bad_values(self):
        bad = [
            {"enabled": "yes"}, {"enabled": 1}, {"text": ""}, {"text": "  "}, {"text": "あ" * 81}, {"text": 3},
            {"text": "a\x07b"}, {"mode": "repeat"}, {"mode": None}, {"interval_sec": 1.9}, {"interval_sec": 301},
            {"interval_sec": "8"}, {"interval_sec": True}, {"interval_sec": float("nan")}, {"interval_sec": float("inf")},
            {"volume": -0.1}, {"volume": 5.01}, {"volume": None}, {"cooldown_sec": -1}, {"cooldown_sec": 3601},
            {"min_active_sec": 61}, {"quiet_when_person_near": "true"}, {"unknown": 1},
        ]
        for b in bad:
            _, e = ae.validate_event_fields("exploring", b)
            self.assertTrue(e, b)
        self.assertTrue(ae.validate_event_fields("exploring", [1])[1])

    def test_limits_match_voice_bridge(self):
        sys.path.insert(0, str(PKG.parents[2] / "voice_bridge"))
        import announce_engine as eng  # noqa: E402
        self.assertEqual((ae.TEXT_MAX, ae.INTERVAL_MIN, ae.INTERVAL_MAX, ae.VOLUME_MAX),
                         (eng.TEXT_MAX, eng.INTERVAL_MIN, eng.INTERVAL_MAX, eng.VOLUME_MAX))
        for eid in ae.EVENT_DEFS:
            self.assertTrue(eng.ID_RE.match(eid), eid)

    def test_document(self):
        c, e = ae.validate_document({"enabled": True, "events": {"exploring": {"volume": 2}}})
        self.assertEqual((e, c["events"]["exploring"]["volume"]), ([], 2))
        for doc in ({"x": 1}, {"enabled": "1"}, {"events": {"nope": {}}}, {"events": []}, [],
                    {"events": {"exploring": {"mode": "x"}}}):
            self.assertTrue(ae.validate_document(doc)[1], doc)

    def test_event_ids_are_stable(self):
        self.assertEqual(set(ae.EVENT_DEFS), {"exploring", "backing_up", "blocked_waiting", "approaching_person",
                                              "emergency_stop", "manual_driving", "mapping_started",
                                              "exploration_complete", "leaving_boundary", "moving_notice"})
        for d in ae.EVENT_DEFS.values():
            self.assertIn(d["trigger"], ae.FLAGS)


class StoreTest(unittest.TestCase):
    def setUp(self):
        self.path = os.path.join(tempfile.mkdtemp(dir=TMP), "ev.json")
        self.store = ae.AnnounceEventsStore(self.path)

    def test_missing_file_is_master_off_with_recommended_defaults(self):
        r = self.store.read()
        self.assertEqual((r["exists"], r["valid"], r["enabled"]), (False, True, False))
        self.assertEqual(r["events"]["exploring"]["text"], "探索中です。ご注意ください")
        self.assertEqual(r["events"]["exploring"]["interval_sec"], 8)

    def test_save_partial_writes_all_events_and_enables(self):
        res = self.store.save({"events": {"exploring": {"text": "探索中", "interval_sec": 5}}})
        self.assertTrue(res["ok"], res)
        r = self.store.read()
        self.assertTrue(r["exists"] and r["enabled"])
        self.assertEqual((r["events"]["exploring"]["text"], r["events"]["exploring"]["interval_sec"]), ("探索中", 5))
        doc = json.load(open(self.path, encoding="utf-8"))
        self.assertEqual(set(doc["events"]), set(ae.EVENT_DEFS))
        self.assertEqual(doc["events"]["backing_up"]["text"], "後ろに下がります")

    def test_second_save_keeps_other_fields_and_makes_backup(self):
        self.store.save({"events": {"exploring": {"text": "A"}}})
        res = self.store.save({"events": {"backing_up": {"enabled": False}}, "enabled": False})
        self.assertTrue(res["backed_up"])
        r = self.store.read()
        self.assertEqual((r["enabled"], r["events"]["exploring"]["text"], r["events"]["backing_up"]["enabled"]),
                         (False, "A", False))
        self.assertTrue(os.path.exists(self.path + ".bak"))

    def test_invalid_save_changes_nothing(self):
        self.store.save({"events": {"exploring": {"text": "A"}}})
        before = open(self.path, "rb").read()
        for bad in ({"events": {"exploring": {"interval_sec": 1}}}, {"events": {"zzz": {}}}, {}, {"bogus": 1}):
            res = self.store.save(bad)
            self.assertFalse(res["ok"], bad)
        self.assertEqual(open(self.path, "rb").read(), before)

    def test_broken_file_reported_and_overwritable(self):
        open(self.path, "w").write("{broken")
        r = self.store.read()
        self.assertFalse(r["valid"])
        self.assertFalse(r["enabled"])
        self.assertTrue(self.store.save({"events": {"exploring": {"text": "A"}}})["ok"])
        self.assertTrue(self.store.read()["valid"])

    def test_nan_rejected_in_file(self):
        open(self.path, "w").write('{"enabled": true, "events": {"exploring": {"volume": NaN}}}')
        self.assertFalse(self.store.read()["valid"])


class ReaderTest(unittest.TestCase):
    def test_missing_valid_invalid_cycle(self):
        path = os.path.join(tempfile.mkdtemp(dir=TMP), "ev.json")
        rd = ae.AnnounceEventsReader(path)
        c, msg, changed = rd.poll()
        self.assertFalse(c["enabled"])
        ae.AnnounceEventsStore(path).save({"events": {"exploring": {"text": "あ"}}})
        c, msg, changed = rd.poll()
        self.assertTrue(changed and c["enabled"])
        self.assertEqual(c["events"]["exploring"]["text"], "あ")
        self.assertFalse(rd.poll()[2])  # 変化なし
        open(path, "w").write('{"enabled": true, "events": {"exploring": {"volume": 99}}}')
        c2, msg, changed = rd.poll()
        self.assertFalse(changed)
        self.assertEqual(c2["events"]["exploring"]["text"], "あ")  # 不正なら直前の有効な設定を維持
        self.assertIn("不正", msg)
        os.remove(path)
        c3, msg, changed = rd.poll()
        self.assertFalse(c3["enabled"])  # ファイルが消えたら全体OFF


class ManagerTest(unittest.TestCase):
    def ops(self, m, now, ctx_=None, **fl):
        return m.update(flags(**fl), ctx_ or ctx(), now)

    def test_default_no_file_sends_nothing(self):
        m = ae.AnnounceManager()
        for t in range(0, 100, 1):
            self.assertEqual(self.ops(m, t, exploring=True, estop=True, backing_up=True), [])

    def test_master_off_with_events_on(self):
        m = ae.AnnounceManager(cfg(enabled=False))
        self.assertEqual(self.ops(m, 0, exploring=True), [])
        self.assertEqual(m.status()["events"]["exploring"]["reason"], "全体OFF")

    def test_loop_start_keepalive_stop(self):
        m = ae.AnnounceManager(cfg(exploring={"min_active_sec": 0}))
        o = self.ops(m, 0, exploring=True)
        self.assertEqual(o, [{"op": "loop", "id": "exploring", "text": "探索中です。ご注意ください",
                              "interval_sec": 8.0, "volume": 1.0}])
        self.assertEqual(self.ops(m, 1, exploring=True), [])
        self.assertEqual(self.ops(m, 4.9, exploring=True), [])
        self.assertEqual(len(self.ops(m, 5.0, exploring=True)), 1)  # 生きている信号
        self.assertEqual(self.ops(m, 6, exploring=False), [{"op": "loop_stop", "id": "exploring"}])
        self.assertEqual(self.ops(m, 7, exploring=False), [])

    def test_min_active_delays_loop(self):
        m = ae.AnnounceManager(cfg())  # exploring min_active=2
        self.assertEqual(self.ops(m, 0, exploring=True), [])
        self.assertEqual(self.ops(m, 1.9, exploring=True), [])
        self.assertEqual(len(self.ops(m, 2.0, exploring=True)), 1)
        m2 = ae.AnnounceManager(cfg())  # 一瞬で終わる状況では鳴らさない
        self.ops(m2, 0, exploring=True)
        self.assertEqual(self.ops(m2, 1, exploring=False), [])

    def test_config_change_resends_immediately(self):
        m = ae.AnnounceManager(cfg(exploring={"min_active_sec": 0}))
        self.ops(m, 0, exploring=True)
        m.set_config(cfg(exploring={"min_active_sec": 0, "text": "探索しています", "interval_sec": 5, "volume": 2}))
        o = self.ops(m, 1, exploring=True)
        self.assertEqual((o[0]["text"], o[0]["interval_sec"], o[0]["volume"]), ("探索しています", 5.0, 2.0))
        m.set_config(cfg(exploring={"enabled": False}))
        self.assertEqual(self.ops(m, 2, exploring=True), [{"op": "loop_stop", "id": "exploring"}])

    def test_mode_loop_to_once_stops_loop(self):
        m = ae.AnnounceManager(cfg(exploring={"min_active_sec": 0}))
        self.ops(m, 0, exploring=True)
        m.set_config(cfg(exploring={"min_active_sec": 0, "mode": "once"}))
        o = self.ops(m, 1, exploring=True)
        self.assertEqual([x["op"] for x in o], ["loop_stop", "once"])

    def test_once_fires_on_rising_edge_with_cooldown(self):
        m = ae.AnnounceManager(cfg(approaching_person={"cooldown_sec": 120}))
        o = self.ops(m, 0, approaching_person=True)
        self.assertEqual(o[0]["op"], "once")
        self.assertEqual(self.ops(m, 1, approaching_person=True), [])  # 続いていても再発話しない
        self.assertEqual(self.ops(m, 5, approaching_person=False), [])
        self.assertEqual(self.ops(m, 10, approaching_person=True), [])  # クールダウン中
        self.assertEqual(m.status()["events"]["approaching_person"]["reason"], "クールダウン中")
        self.assertEqual(self.ops(m, 20, approaching_person=False), [])
        self.assertEqual(self.ops(m, 130, approaching_person=True)[0]["op"], "once")

    def test_cooldown_blocked_fires_later_in_same_run(self):
        m = ae.AnnounceManager(cfg(approaching_person={"cooldown_sec": 30}))
        self.ops(m, 0, approaching_person=True)
        self.ops(m, 1, approaching_person=False)
        self.assertEqual(self.ops(m, 5, approaching_person=True), [])
        self.assertEqual(self.ops(m, 31, approaching_person=True)[0]["op"], "once")

    def test_blocked_waiting_needs_4s(self):
        m = ae.AnnounceManager(cfg())
        self.assertEqual(self.ops(m, 0, blocked=True), [])
        self.assertEqual(self.ops(m, 3.9, blocked=True), [])
        self.assertEqual(self.ops(m, 4.0, blocked=True)[0]["id"], "blocked_waiting")

    def test_person_near_suppresses_quiet_events_only(self):
        m = ae.AnnounceManager(cfg())
        self.assertEqual(self.ops(m, 0, ctx(person_near=True), mapping=True), [])  # mapping_started: 人が近いと出さない
        self.assertEqual(m.status()["events"]["mapping_started"]["reason"], "人が近い")
        self.assertEqual(self.ops(m, 1, ctx(person_near=True), mapping=True), [])
        o = self.ops(m, 2, ctx(person_near=False), mapping=True)  # 人が離れたら(状況が続いていれば)出す
        self.assertEqual(o[0]["id"], "mapping_started")
        m2 = ae.AnnounceManager(cfg(exploring={"min_active_sec": 0}))  # exploringは既定で人が近くても出す
        self.assertEqual(self.ops(m2, 0, ctx(person_near=True), exploring=True)[0]["op"], "loop")

    def test_person_near_stops_running_loop(self):
        m = ae.AnnounceManager(cfg(exploring={"min_active_sec": 0, "quiet_when_person_near": True}))
        self.ops(m, 0, exploring=True)
        self.assertEqual(self.ops(m, 1, ctx(person_near=True), exploring=True),
                         [{"op": "loop_stop", "id": "exploring"}])

    def test_survey_suppresses_by_default_and_is_configurable(self):
        m = ae.AnnounceManager(cfg(exploring={"min_active_sec": 0}))
        self.assertEqual(self.ops(m, 0, ctx(survey=True), exploring=True), [])
        self.assertEqual(m.status()["events"]["exploring"]["reason"], "アンケート回答中")
        m2 = ae.AnnounceManager(cfg(exploring={"min_active_sec": 0, "during_survey": True}))
        self.assertEqual(self.ops(m2, 0, ctx(survey=True), exploring=True)[0]["op"], "loop")

    def test_emergency_stop_dedicated_path(self):
        m = ae.AnnounceManager(cfg())
        o = self.ops(m, 0, ctx(estop=True, survey=True), estop=True)
        self.assertEqual(len(o), 1)
        self.assertEqual((o[0]["id"], o[0]["op"], o[0]["urgent"]), ("emergency_stop", "once", True))
        self.assertEqual(o[0]["volume"], 1.5)
        m2 = ae.AnnounceManager(cfg(exploring={"min_active_sec": 0}))  # 他のイベントは緊急停止中は出さない
        self.assertEqual(self.ops(m2, 0, ctx(estop=True), exploring=True), [])
        self.assertEqual(m2.status()["events"]["exploring"]["reason"], "緊急停止中")
        m3 = ae.AnnounceManager(cfg(emergency_stop={"enabled": False}))  # 個別OFFなら出ない
        self.assertEqual(self.ops(m3, 0, ctx(estop=True), estop=True), [])

    def test_moving_notice_loops_every_20s_and_starts_at_once(self):
        # 【2026-10-27】「ロボットが移動しています。ご注意ください」: 動き始めにすぐ1回(min_active_sec=0)、動いている間は20秒間隔のループ
        m = ae.AnnounceManager(cfg())
        o = self.ops(m, 0, moving_notice=True)
        self.assertEqual(o, [{"op": "loop", "id": "moving_notice", "text": "ロボットが移動しています。ご注意ください",
                              "interval_sec": 20.0, "volume": 1.0}])
        self.assertEqual(self.ops(m, 1, moving_notice=True), [])
        self.assertEqual(len(self.ops(m, 5.0, moving_notice=True)), 1)  # 生きている信号(2台目の停止判定は15秒)
        self.assertEqual(self.ops(m, 6, moving_notice=False), [{"op": "loop_stop", "id": "moving_notice"}])

    def test_moving_notice_is_quiet_during_survey_and_estop_and_is_independent_of_other_events(self):
        m = ae.AnnounceManager(cfg())
        self.assertEqual(self.ops(m, 0, ctx(survey=True), moving_notice=True), [])
        self.assertEqual(m.status()["events"]["moving_notice"]["reason"], "アンケート回答中")
        m = ae.AnnounceManager(cfg())
        self.assertEqual(self.ops(m, 0, ctx(estop=True), moving_notice=True), [])
        self.assertEqual(m.status()["events"]["moving_notice"]["reason"], "緊急停止中")
        m = ae.AnnounceManager(cfg(moving_notice={"enabled": False}))
        self.assertEqual(self.ops(m, 0, moving_notice=True), [])           # 個別OFF
        m = ae.AnnounceManager(cfg(enabled=False))
        self.assertEqual(self.ops(m, 0, moving_notice=True), [])           # 全体OFF
        m = ae.AnnounceManager()                                           # 設定ファイルが無い間は何も鳴らさない(従来どおり)
        self.assertEqual(self.ops(m, 0, moving_notice=True), [])
        # 既存のイベントの動作は変えない(探索中は従来どおり8秒のループ)
        m = ae.AnnounceManager(cfg(exploring={"min_active_sec": 0}))
        self.assertEqual(self.ops(m, 0, exploring=True)[0]["interval_sec"], 8.0)

    def test_moving_notice_text_interval_volume_are_editable(self):
        ok, errs = ae.validate_event_fields("moving_notice", {"text": "移動中", "interval_sec": 30, "volume": 2, "enabled": False})
        self.assertEqual(errs, [])
        m = ae.AnnounceManager(cfg(moving_notice={"text": "移動中", "interval_sec": 30, "volume": 2}))
        o = self.ops(m, 0, moving_notice=True)
        self.assertEqual((o[0]["text"], o[0]["interval_sec"], o[0]["volume"]), ("移動中", 30.0, 2.0))

    def test_manual_driving_default_off(self):
        m = ae.AnnounceManager(cfg())
        self.assertEqual(self.ops(m, 0, manual=True), [])
        self.assertEqual(self.ops(m, 5, manual=True), [])

    def test_stop_all(self):
        m = ae.AnnounceManager(cfg(exploring={"min_active_sec": 0}, backing_up={"min_active_sec": 0}))
        self.ops(m, 0, exploring=True, backing_up=True)
        self.assertEqual({o["id"] for o in m.stop_all()}, {"exploring", "backing_up"})
        self.assertEqual(m.stop_all(), [])


class SituationTest(unittest.TestCase):
    def test_frontier_exploring_and_expiry(self):
        s = ae.SituationBuilder()
        s.on_frontier_status({"enabled": True, "status": "exploring"}, 0)
        self.assertTrue(s.flags(1)["exploring"])
        self.assertTrue(s.flags(9.9)["exploring"])
        self.assertFalse(s.flags(10.1)["exploring"])  # 失効
        s.on_frontier_status({"enabled": True, "status": "paused", "reason": "estop"}, 11)
        self.assertFalse(s.flags(11)["exploring"])
        s.on_frontier_status({"enabled": False, "status": "exploring"}, 12)
        self.assertFalse(s.flags(12)["exploring"])

    def test_exploration_complete_only_after_exploring(self):
        s = ae.SituationBuilder()
        s.on_frontier_status({"enabled": True, "status": "complete"}, 0)  # 起動直後の古いcomplete
        self.assertFalse(s.flags(0)["exploration_complete"])
        s.on_frontier_status({"enabled": True, "status": "exploring"}, 1)
        s.on_frontier_status({"enabled": True, "status": "complete"}, 2)
        self.assertTrue(s.flags(3)["exploration_complete"])
        self.assertFalse(s.flags(8)["exploration_complete"])

    def test_modes_from_behavior_command(self):
        s = ae.SituationBuilder()
        for mode, key in (("CAMERA_OBSTACLE_HOLD", "blocked"), ("NEAR_PERSON_HOLD", "blocked"),
                          ("APPROACH_PERSON", "approaching_person"), ("OUT_OF_BOUNDS_HOLD", "out_of_bounds"),
                          ("KEEPOUT_ZONE_HOLD", "out_of_bounds")):
            s.on_behavior_command({"mode": mode}, 0)
            f = s.flags(1)
            self.assertTrue(f[key], mode)
            self.assertEqual(sum(f.values()), 1, mode)
        s.on_behavior_command({"mode": "ESTOP_HOLD", "estop_active": True}, 0)
        self.assertTrue(s.flags(1)["estop"] and s.context(1)["estop"])
        s.on_behavior_command({"mode": "MANUAL_OVERRIDE", "manual_override": True}, 0)
        self.assertTrue(s.flags(1)["manual"])
        self.assertFalse(s.flags(5)["manual"])  # directorが止まったら失効

    def test_moving_notice_flag_follows_the_dashboard_result_and_expires(self):
        s = ae.SituationBuilder()
        self.assertFalse(s.flags(0)["moving_notice"])                       # 届いていない
        s.on_moving_notice({"sound": True, "display": True}, 0)
        self.assertTrue(s.flags(1)["moving_notice"])
        self.assertFalse(s.flags(3.5)["moving_notice"])                     # ダッシュボードが止まったら失効(鳴りっぱなしにしない)
        s.on_moving_notice({"sound": False, "display": True}, 4)            # 画面だけ保持中の間は鳴らさない
        self.assertFalse(s.flags(4.1)["moving_notice"])
        for bad in ("true", 1, None, [], {"sound": "yes"}):
            s.on_moving_notice(bad, 5)
            self.assertFalse(s.flags(5.1)["moving_notice"], bad)
        s.on_moving_notice({"sound": True}, 6)
        s.on_behavior_command({"mode": "ESTOP_HOLD", "estop_active": True}, 6)
        self.assertTrue(s.context(6.1)["estop"])                           # 緊急停止は context 側で Manager が抑える

    def test_moving_notice_is_silent_when_the_screen_is_not_customer_mode(self):
        """【2026-10-30】iPadの表示モードが customer 以外(ダッシュボードが sound=False, reason=display_mode で流す)なら、announce は鳴らさない。読む側は変更なし。"""
        s = ae.SituationBuilder()
        s.on_moving_notice({"sound": False, "display": False, "kind": "", "reason": "display_mode", "moving": True}, 0)
        self.assertFalse(s.flags(0.1)["moving_notice"])
        s.on_moving_notice({"sound": True, "display": True, "kind": "go_pin", "reason": "", "moving": True}, 1)   # customer に戻れば鳴る
        self.assertTrue(s.flags(1.1)["moving_notice"])

    def test_backing_up_detection(self):
        s = ae.SituationBuilder()
        s.on_cmd_vel(-0.1, 0.0)
        self.assertFalse(s.flags(0.1)["backing_up"])  # 0.6秒続いてから
        s.on_cmd_vel(-0.1, 0.4)
        s.on_cmd_vel(-0.1, 0.7)
        self.assertTrue(s.flags(0.7)["backing_up"])
        s.on_cmd_vel(0.0, 1.0)
        self.assertTrue(s.flags(1.0)["backing_up"])   # 一瞬0が混ざっても途切れない
        s.on_cmd_vel(0.2, 2.0)
        self.assertFalse(s.flags(2.0)["backing_up"])
        s.on_cmd_vel(0.2, 3.0)
        s.on_cmd_vel(-0.1, 4.0)
        self.assertFalse(s.flags(4.1)["backing_up"])  # 前進のあとの新しい後退はまた0.6秒待つ

    def test_manual_backing_excluded(self):
        s = ae.SituationBuilder()
        s.on_behavior_command({"mode": "MANUAL_OVERRIDE", "manual_override": True}, 0)
        for t in (0.0, 0.5, 1.0):
            s.on_cmd_vel(-0.2, t)
        self.assertFalse(s.flags(1.0)["backing_up"])

    def test_context(self):
        s = ae.SituationBuilder()
        s.on_survey_active(True, 0)
        self.assertTrue(s.context(1)["survey"])
        self.assertFalse(s.context(4)["survey"])
        s.on_crowd_state({"level": "calm", "person_target": {"x": 1}}, 10)
        self.assertTrue(s.context(11)["person_near"])
        self.assertFalse(s.context(14)["person_near"])
        s.on_crowd_state({"level": "calm", "person_target": None}, 20)
        self.assertFalse(s.context(21)["person_near"])
        s.on_near_person_stop(True, 30)
        self.assertTrue(s.context(31)["person_near"])
        s.on_behavior_command({"mode": "NEAR_PERSON_HOLD"}, 40)
        self.assertTrue(s.context(41)["person_near"])

    def test_mapping_flag(self):
        s = ae.SituationBuilder()
        s.on_map_building_active(True, 0)
        self.assertTrue(s.flags(1)["mapping"])
        self.assertFalse(s.flags(5)["mapping"])


class _Bridge(BaseHTTPRequestHandler):
    posts = []
    voice_enabled = True
    fail = False

    def log_message(self, *a):
        pass

    def _reply(self, code, body):
        data = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        self._reply(200, {"voice_enabled": _Bridge.voice_enabled})

    def do_POST(self):
        n = int(self.headers.get("Content-Length", "0"))
        body = json.loads(self.rfile.read(n).decode())
        if _Bridge.fail:
            self._reply(500, {"error": "x"})
            return
        _Bridge.posts.append((self.path, body))
        self._reply(200, {"ok": True})


class SenderTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), _Bridge)
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()
        cls.url = f"http://127.0.0.1:{cls.srv.server_address[1]}"

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def setUp(self):
        _Bridge.posts = []
        _Bridge.voice_enabled = True
        _Bridge.fail = False
        self.logs = []
        self.s = an.AnnounceSender(self.url, 2.0, log=self.logs.append, status_every_sec=0.0)

    def test_sends_loop_stop_once(self):
        self.s.handle({"op": "loop", "id": "exploring", "text": "探索中です", "interval_sec": 8.0, "volume": 1.0})
        self.s.handle({"op": "loop_stop", "id": "exploring"})
        self.s.handle({"op": "once", "id": "emergency_stop", "text": "止", "volume": 1.5, "urgent": True})
        self.assertEqual(_Bridge.posts[0], ("/announce_loop", {"id": "exploring", "text": "探索中です",
                                                               "interval_sec": 8.0, "volume": 1.0, "enabled": True}))
        self.assertEqual(_Bridge.posts[1], ("/announce_loop", {"id": "exploring", "enabled": False}))
        self.assertEqual(_Bridge.posts[2], ("/announce", {"id": "emergency_stop", "text": "止", "volume": 1.5,
                                                         "urgent": True}))
        self.assertEqual(self.s.sent, 3)

    def test_master_off_not_sent_but_stop_is(self):
        _Bridge.voice_enabled = False
        self.s.handle({"op": "loop", "id": "a", "text": "A", "interval_sec": 8.0, "volume": 1.0})
        self.s.handle({"op": "once", "id": "a", "text": "A", "volume": 1.0})
        self.assertEqual(_Bridge.posts, [])
        self.assertEqual(self.s.skipped_muted, 2)
        self.s.handle({"op": "loop_stop", "id": "a"})
        self.assertEqual(len(_Bridge.posts), 1)

    def test_unreachable_is_quiet(self):
        s = an.AnnounceSender("http://127.0.0.1:1", 0.3, log=self.logs.append, status_every_sec=999)
        for _ in range(5):
            s.handle({"op": "loop", "id": "a", "text": "A", "interval_sec": 8.0, "volume": 1.0})
        self.assertEqual(s.failed, 5)
        self.assertEqual(len(self.logs), 1)  # ログは間引く
        self.assertTrue(s.last_error)

    def test_http_error_counts_failure(self):
        _Bridge.fail = True
        self.s.handle({"op": "loop", "id": "a", "text": "A", "interval_sec": 8.0, "volume": 1.0})
        self.assertEqual(self.s.failed, 1)

    def test_queue_overflow_drops_oldest(self):
        s = an.AnnounceSender(self.url)
        s.submit([{"op": "loop_stop", "id": f"x{i}"} for i in range(70)])
        self.assertGreater(s.dropped, 0)

    def test_thread_end_to_end(self):
        self.s.status_every_sec = 0.2
        self.s.start()
        self.s.submit([{"op": "loop", "id": "a", "text": "A", "interval_sec": 8.0, "volume": 1.0}])
        for _ in range(50):
            if _Bridge.posts:
                break
            threading.Event().wait(0.1)
        self.s.stop()
        self.assertEqual(len(_Bridge.posts), 1)

    def test_node_source_never_publishes_motion(self):
        src = (PKG / "nokosahen_core" / "announce_node.py").read_text(encoding="utf-8")
        self.assertNotIn("/goal_pose", src)
        self.assertNotIn("create_publisher(Twist", src)
        self.assertNotIn("cancel_goal", src)


if __name__ == "__main__":
    unittest.main()
