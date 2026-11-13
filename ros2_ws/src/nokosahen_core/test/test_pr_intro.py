"""文化祭の出店紹介(pr_intro.py、announce_node の送信、dashboard の /api/pr_intro*、設定画面のカード)のテスト。docs/announcements.md

ロボット・ROS・2台目・VOICEVOXには触れない(rclpyはスタブ、voice_bridgeはhttp.serverのスタブ、合成はスタブ、時刻は引数)。
1. 検証・整形: 文字数(400/合計)・パート数・間隔・音量・未知の項目・制御文字/ゼロ幅文字の除去・仮の文の判定
2. 保存: 既定はOFF・仮の文、保存の原子性(.bak・書き込み失敗で元のファイルを守る)、不正ファイルは直前の有効な設定を維持
3. PrIntroManager: 再生の判断(間隔の再送・設定変更・抑止の一覧・再開待ち・事前合成の要求の再送)、既存アナウンスとの優先順位
4. Controller: 保存→2台目に音声作成を依頼(文章が変わったときだけ)、音声が未作成なら試し再生しない、2台目が落ちていても保存できる
5. AnnounceSender: playlist / playlist_stop / prepare を2台目の正しいエンドポイントへ。マスターOFF中も止めと事前作成は送る
6. ダッシュボードの権限: 未ログイン401・guest閲覧のみ・TL保存、roles.API_RULES(announce.pr_intro は保留=付与不可)
7. 画面: JS構文、カードの要素、2台目(announce_engine.py)と定数が一致、雛形JSONが有効

実行: python ros2_ws/src/nokosahen_core/test/test_pr_intro.py
"""

import importlib.util
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import types
import unittest
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
REPO = PKG.parents[2]
sys.path.insert(0, str(PKG))

for _n in ("rclpy", "rclpy.node", "geometry_msgs", "geometry_msgs.msg", "nav_msgs", "nav_msgs.msg", "std_msgs", "std_msgs.msg"):
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
                m.PoseStamped = type("PoseStamped", (), {})
            if _n == "nav_msgs.msg":
                m.Odometry = type("Odometry", (), {})
            if _n == "std_msgs.msg":
                for k in ("Bool", "String", "Float32MultiArray", "Empty"):
                    setattr(m, k, type(k, (), {}))

from nokosahen_core import announce_events as ae  # noqa: E402
from nokosahen_core import announce_events_proxy as ap  # noqa: E402
from nokosahen_core import announce_node as an  # noqa: E402
from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import pr_intro as pr  # noqa: E402
from nokosahen_core import roles  # noqa: E402
from nokosahen_core.announce_settings_page import build_announce_settings_html  # noqa: E402

TMP = tempfile.mkdtemp(prefix="pr_intro_test_")


def newpath():
    return tempfile.mkdtemp(dir=TMP) + "/pr.json"


def good_parts(n=2):
    return [{"enabled": True, "title": f"見出し{i}", "text": f"本番の文章{i}です。", "volume": 1.0} for i in range(n)]


# ---------------------------------------------------------------- 1. 検証・整形
class ValidateTest(unittest.TestCase):
    def test_defaults_are_off_and_placeholder(self):
        cfg = pr.default_config()
        self.assertFalse(cfg["enabled"])
        self.assertGreaterEqual(len(cfg["parts"]), 2)
        self.assertTrue(all(pr.is_placeholder(p["text"]) for p in cfg["parts"]))
        self.assertEqual(pr.validate_update(cfg)[1], [])
        self.assertFalse(pr.merged_config({"enabled": True}, False)["enabled"])  # ファイルが無ければ必ずOFF

    def test_sanitize(self):
        self.assertEqual(pr.sanitize_text("  こんにちは​、\x07世界‮  "), "こんにちは、世界")
        self.assertEqual(pr.sanitize_text("一行目\n二行目\r\n三行目"), "一行目。二行目。三行目")
        self.assertEqual(pr.sanitize_text("終わり。\n次"), "終わり。次")
        self.assertEqual(pr.sanitize_text("a\t\tb   c"), "a b c")

    def test_text_and_total_limits(self):
        ok = {"parts": [{"text": "あ" * pr.TEXT_MAX}]}
        self.assertEqual(pr.validate_update(ok)[1], [])
        self.assertNotEqual(pr.validate_update({"parts": [{"text": "あ" * (pr.TEXT_MAX + 1)}]})[1], [])
        self.assertNotEqual(pr.validate_update({"parts": [{"text": "​"}]})[1], [])  # 除去したら空
        total = [{"text": "あ" * pr.TEXT_MAX}] * (pr.TOTAL_TEXT_MAX // pr.TEXT_MAX + 1)
        self.assertLessEqual(len(total), pr.PARTS_MAX)
        self.assertNotEqual(pr.validate_update({"parts": total})[1], [])
        self.assertNotEqual(pr.validate_update({"parts": [{"text": "あ"}] * (pr.PARTS_MAX + 1)})[1], [])

    def test_ranges_and_types(self):
        for key, lo, hi in (("interval_sec", pr.INTERVAL_MIN, pr.INTERVAL_MAX), ("gap_sec", pr.GAP_MIN, pr.GAP_MAX),
                            ("volume", 0.0, pr.VOLUME_MAX)):
            self.assertEqual(pr.validate_update({key: lo})[1], [])
            self.assertEqual(pr.validate_update({key: hi})[1], [])
            self.assertNotEqual(pr.validate_update({key: lo - 0.5 if lo else -0.1})[1], [], key)
            self.assertNotEqual(pr.validate_update({key: hi + 1})[1], [], key)
            for bad in ("5", True, None, float("nan"), float("inf"), [1]):
                self.assertNotEqual(pr.validate_update({key: bad})[1], [], (key, bad))
        for bad in ({"enabled": "yes"}, {"quiet_when_person_near": 1}, {"nope": 1}, {"parts": "x"}, {"parts": ["x"]},
                    {"parts": [{"text": "a", "bogus": 1}]}, {"parts": [{"text": 5}]}, {"parts": [{"text": "a", "volume": 6}]},
                    {"parts": [{"text": "a", "enabled": "y"}]}, {"parts": [{"text": "a", "title": "あ" * 31}]}, [1], "x"):
            self.assertNotEqual(pr.validate_update(bad)[1], [], bad)

    def test_engine_constants_match(self):
        spec = importlib.util.spec_from_file_location("announce_engine_for_test", REPO / "voice_bridge" / "announce_engine.py")
        eng = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(eng)
        self.assertEqual(pr.TEXT_MAX, eng.PR_TEXT_MAX)
        self.assertEqual(pr.PARTS_MAX, eng.PLAYLIST_MAX_PARTS)
        self.assertEqual((pr.GAP_MIN, pr.GAP_MAX), (eng.GAP_MIN, eng.GAP_MAX))
        self.assertLessEqual(pr.INTERVAL_MAX, eng.PLAYLIST_INTERVAL_MAX)
        self.assertGreaterEqual(pr.INTERVAL_MIN, eng.INTERVAL_MIN)
        self.assertEqual(pr.VOLUME_MAX, eng.VOLUME_MAX)
        # 2台目が受け付けるプレイリストを、1台目が作る内容で通せる(キー名の食い違いを防ぐ)
        op = pr.playlist_op(pr.default_config())
        req, errs = eng.parse_loop_request({"id": op["id"], "parts": op["parts"], "interval_sec": op["interval_sec"],
                                            "gap_sec": op["gap_sec"], "enabled": True})
        self.assertEqual(errs, [])
        req, errs = eng.parse_prepare_request({"texts": pr.prepare_texts(pr.default_config()), "start": True})
        self.assertEqual(errs, [])

    def test_example_json_is_valid_placeholder_and_off(self):
        doc = json.loads((REPO / "config" / "pr_intro.example.json").read_text(encoding="utf-8"))
        clean, errs = pr.validate_update(doc)
        self.assertEqual(errs, [])
        self.assertFalse(clean["enabled"])
        self.assertTrue(all(pr.is_placeholder(p["text"]) for p in clean["parts"]))

    def test_placeholder_detection(self):
        self.assertTrue(pr.is_placeholder("(仮)ああ"))
        self.assertTrue(pr.is_placeholder("  （仮）ああ"))
        self.assertFalse(pr.is_placeholder("本番です(仮)"))


# ---------------------------------------------------------------- 2. 保存
class StoreTest(unittest.TestCase):
    def test_missing_file_gives_default_off(self):
        s = pr.PrIntroStore(newpath())
        r = s.read()
        self.assertEqual((r["exists"], r["valid"], r["config"]["enabled"]), (False, True, False))

    def test_save_roundtrip_and_backup(self):
        s = pr.PrIntroStore(newpath())
        r = s.save({"enabled": True, "parts": good_parts(), "interval_sec": 300, "gap_sec": 10})
        self.assertTrue(r["ok"], r)
        self.assertFalse(r["backed_up"])
        c = s.read()["config"]
        self.assertEqual((c["enabled"], c["interval_sec"], c["gap_sec"], len(c["parts"])), (True, 300, 10, 2))
        r = s.save({"volume": 2})  # 部分更新: partsは保たれる
        self.assertTrue(r["backed_up"])
        self.assertTrue(os.path.isfile(s.backup_path))
        c = s.read()["config"]
        self.assertEqual((c["volume"], len(c["parts"]), c["interval_sec"]), (2, 2, 300))
        self.assertTrue(Path(s.path).read_text(encoding="utf-8").startswith("{"))
        self.assertNotIn("\r", Path(s.path).read_bytes().decode())

    def test_invalid_saves_nothing(self):
        s = pr.PrIntroStore(newpath())
        for bad in ({}, {"gap_sec": 0}, {"parts": [{"text": ""}]}, {"nope": 1}, {"enabled": True, "parts": [{"enabled": False, "text": "あ"}]}):
            r = s.save(bad)
            self.assertFalse(r["ok"], bad)
        self.assertFalse(os.path.exists(s.path))

    def test_enabling_without_enabled_part_is_rejected(self):
        s = pr.PrIntroStore(newpath())
        s.save({"parts": good_parts()})
        self.assertFalse(s.save({"enabled": True, "parts": [{"enabled": False, "text": "あ"}]})["ok"])

    def test_write_failure_keeps_old_file(self):
        s = pr.PrIntroStore(newpath())
        s.save({"parts": good_parts(), "gap_sec": 7})
        before = Path(s.path).read_bytes()
        orig = os.replace

        def boom(a, b):
            raise OSError("disk full")

        os.replace = boom
        try:
            r = s.save({"gap_sec": 9})
        finally:
            os.replace = orig
        self.assertFalse(r["ok"])
        self.assertEqual(r["kind"], "io")
        self.assertEqual(Path(s.path).read_bytes(), before)
        self.assertEqual([n for n in os.listdir(os.path.dirname(s.path)) if n.endswith(".tmp")], [])

    def test_broken_file_reported_and_overwritable(self):
        p = newpath()
        Path(p).write_text("{ not json")
        s = pr.PrIntroStore(p)
        r = s.read()
        self.assertFalse(r["valid"])
        self.assertTrue(s.save({"parts": good_parts()})["ok"])
        self.assertTrue(os.path.isfile(p + ".bak"))
        self.assertTrue(s.read()["valid"])

    def test_nan_in_file_rejected(self):
        p = newpath()
        Path(p).write_text('{"gap_sec": NaN}')
        self.assertFalse(pr.PrIntroStore(p).read()["valid"])


class ReaderTest(unittest.TestCase):
    def test_hot_reload_and_keep_last_good(self):
        p = newpath()
        rd = pr.PrIntroReader(p)
        cfg, msg, changed = rd.poll()
        self.assertFalse(changed)
        self.assertFalse(cfg["enabled"])
        pr.PrIntroStore(p).save({"enabled": True, "parts": good_parts()})
        cfg, msg, changed = rd.poll()
        self.assertTrue(changed)
        self.assertTrue(cfg["enabled"])
        self.assertFalse(rd.poll()[2])
        Path(p).write_text('{"gap_sec": 9999}')
        cfg2, msg, changed = rd.poll()
        self.assertFalse(changed)
        self.assertTrue(cfg2["enabled"])  # 直前の有効な設定を維持
        self.assertTrue(rd.load_error)
        os.remove(p)
        cfg3, msg, changed = rd.poll()
        self.assertTrue(changed)
        self.assertFalse(cfg3["enabled"])


# ---------------------------------------------------------------- 3. PrIntroManager
def on_cfg(**over):
    c = pr.merged_config({"enabled": True, "parts": good_parts(3), "interval_sec": 120, "gap_sec": 8, "speech_mode": "pr"}, True)
    c.update(over)
    return c


def kinds(ops):
    return [o["op"] for o in ops]


class SpeechModeTest(unittest.TestCase):
    """【2026-10-31】出店紹介とメニュー紹介は、どちらか片方だけ(同時に流さない)。"""

    def test_default_is_menu(self):
        self.assertEqual(pr.default_config()["speech_mode"], "menu")
        self.assertEqual(pr.merged_config(None, False)["speech_mode"], "menu")
        self.assertEqual([m["value"] for m in pr.meta()["speech_modes"]], ["menu", "pr"])

    def test_validate(self):
        self.assertEqual(pr.validate_update({"speech_mode": "pr"})[0], {"speech_mode": "pr"})
        self.assertTrue(pr.validate_update({"speech_mode": "both"})[1])
        self.assertTrue(pr.validate_update({"speech_mode": None})[1])

    def test_menu_mode_stops_pr_intro(self):
        m = pr.PrIntroManager(on_cfg(speech_mode="menu"), settle_sec=0)
        ops = m.update({}, {}, 0.0)
        self.assertNotIn("playlist", kinds(ops))
        self.assertIn("メニュー紹介を選択中", m.reason)

    def test_switch_to_menu_stops_a_running_loop(self):
        m = pr.PrIntroManager(on_cfg(), settle_sec=0)
        self.assertIn("playlist", kinds(m.update({}, {}, 0.0)))
        m.set_config(on_cfg(speech_mode="menu"))
        self.assertEqual(m.update({}, {}, 1.0), [{"op": "playlist_stop", "id": "pr_intro"}])

    def test_menu_speech_suppressed_only_when_pr_selected_and_playing(self):
        self.assertTrue(pr.menu_speech_suppressed(on_cfg()))
        self.assertFalse(pr.menu_speech_suppressed(on_cfg(speech_mode="menu")))
        self.assertFalse(pr.menu_speech_suppressed(on_cfg(enabled=False)))
        cfg = on_cfg()
        for p in cfg["parts"]:
            p["enabled"] = False
        self.assertFalse(pr.menu_speech_suppressed(cfg))

    def test_store_roundtrip(self):
        d = tempfile.mkdtemp()
        try:
            st = pr.PrIntroStore(os.path.join(d, "pr.json"))
            r = st.save({"enabled": True, "parts": good_parts(2), "speech_mode": "pr"})
            self.assertTrue(r["ok"], r)
            self.assertEqual(st.read()["config"]["speech_mode"], "pr")
        finally:
            shutil.rmtree(d, ignore_errors=True)


class ManagerTest(unittest.TestCase):
    def test_off_sends_nothing(self):
        m = pr.PrIntroManager(pr.merged_config(None, False))
        self.assertEqual(m.update({}, {}, 0.0), [])
        self.assertEqual(m.reason, "OFF")

    def test_start_keepalive_and_stop(self):
        m = pr.PrIntroManager(on_cfg(), settle_sec=0)
        ops = m.update({}, {}, 100.0)
        self.assertEqual(kinds(ops), ["prepare", "playlist"])
        pl = ops[1]
        self.assertEqual((pl["id"], pl["interval_sec"], pl["gap_sec"], len(pl["parts"])), ("pr_intro", 120.0, 8.0, 3))
        self.assertEqual(m.update({}, {}, 102.0), [])  # 5秒未満は再送しない
        self.assertEqual(kinds(m.update({}, {}, 105.1)), ["playlist"])  # 生きている信号
        ops = m.update({"estop": True}, {"estop": True}, 106.0)
        self.assertEqual(ops, [{"op": "playlist_stop", "id": "pr_intro"}])
        self.assertEqual(m.update({"estop": True}, {"estop": True}, 107.0), [])  # 止まっている間は何も送らない

    def test_config_change_resends_immediately_and_prepare(self):
        m = pr.PrIntroManager(on_cfg(), settle_sec=0)
        m.update({}, {}, 0.0)
        cfg = on_cfg()
        cfg["parts"][0]["text"] = "差し替えた文章です。"
        m.set_config(cfg)
        ops = m.update({}, {}, 1.0)
        self.assertEqual(kinds(ops), ["prepare", "playlist"])
        self.assertIn("差し替えた文章です。", ops[0]["texts"])
        self.assertEqual(ops[1]["parts"][0]["text"], "差し替えた文章です。")

    def test_prepare_resent_periodically_for_recovery(self):
        m = pr.PrIntroManager(on_cfg(), settle_sec=0, prepare_resend_sec=120)
        m.update({}, {}, 0.0)
        self.assertNotIn("prepare", kinds(m.update({}, {}, 60.0)))
        self.assertIn("prepare", kinds(m.update({}, {}, 121.0)))

    def test_every_blocker_stops_playlist(self):
        for flag in pr.BLOCK_FLAGS:
            if flag == "blocked":
                continue  # 近すぎる人でのホールド。下の test_blocked_follows_quiet_when_person_near
            m = pr.PrIntroManager(on_cfg(), settle_sec=0)
            m.update({}, {}, 0.0)
            ops = m.update({flag: True}, {}, 1.0)
            self.assertEqual(kinds(ops), ["playlist_stop"], flag)
            self.assertIn("停止", m.reason)
        for ctx in ({"survey": True}, {"estop": True}):
            m = pr.PrIntroManager(on_cfg(), settle_sec=0)
            m.update({}, {}, 0.0)
            self.assertEqual(kinds(m.update({}, ctx, 1.0)), ["playlist_stop"], ctx)

    def test_blocked_follows_quiet_when_person_near(self):
        """【2026-11-06】"blocked"(近すぎる人/カメラ障害物でホールド中)は、人がすぐ近くにいる=説明を聞いてほしい場面。
        quiet_when_person_near が OFF(既定)なら紹介を止めない(実機で NEAR_PERSON_HOLD が頻繁に出入りして、ほとんど流れなかった)。
        ON なら従来どおり止める。"""
        m = pr.PrIntroManager(on_cfg(quiet_when_person_near=False), settle_sec=5)
        m.update({}, {}, 0.0)
        self.assertEqual(m.update({"blocked": True}, {}, 1.0), [])  # 止めない
        self.assertTrue(m.on)
        self.assertEqual(m.reason, "")
        self.assertEqual(m.update({"blocked": False}, {}, 2.0), [])  # 出入りしても再開待ちにならない
        self.assertEqual(m.reason, "")
        m = pr.PrIntroManager(on_cfg(quiet_when_person_near=True), settle_sec=0)
        m.update({}, {}, 0.0)
        self.assertEqual(kinds(m.update({"blocked": True}, {}, 1.0)), ["playlist_stop"])
        self.assertIn("停止", m.reason)
        # 他の抑止(移動中・手動・緊急停止など)は、quiet_when_person_near に関係なく止める
        m = pr.PrIntroManager(on_cfg(quiet_when_person_near=False), settle_sec=0)
        m.update({}, {}, 0.0)
        self.assertEqual(kinds(m.update({"blocked": True, "manual": True}, {}, 1.0)), ["playlist_stop"])

    def test_every_blocker_is_a_known_situation_flag(self):
        self.assertTrue(set(pr.BLOCK_FLAGS) <= set(ae.FLAGS))
        # 既存イベントの trigger は全て紹介の抑止対象に含まれる(既存アナウンスが鳴る状況では紹介は鳴らさない)
        triggers = {d["trigger"] for d in ae.EVENT_DEFS.values()}
        self.assertTrue(triggers <= set(pr.BLOCK_FLAGS), triggers - set(pr.BLOCK_FLAGS))

    def test_moving_notice_has_priority_and_resume_waits(self):
        m = pr.PrIntroManager(on_cfg(), settle_sec=5)
        m.update({}, {}, 0.0)  # 再開待ちの前に一度も止めていないので開始は即時でない?
        # 開始直後は settle の影響を受けない(阻害要因が立っていなかった)
        self.assertTrue(m.on)
        self.assertEqual(kinds(m.update({"moving_notice": True}, {}, 10.0)), ["playlist_stop"])
        self.assertEqual(m.update({}, {}, 11.0), [])
        self.assertEqual(m.reason, "再開待ち")
        self.assertEqual(m.update({}, {}, 14.9), [])
        self.assertEqual(kinds(m.update({}, {}, 15.1)), ["playlist"])  # 解除から5秒後に再開

    def test_person_near_only_when_configured(self):
        m = pr.PrIntroManager(on_cfg(quiet_when_person_near=False), settle_sec=0)
        m.update({}, {}, 0.0)
        self.assertEqual(m.update({}, {"person_near": True}, 1.0), [])
        self.assertTrue(m.on)
        m = pr.PrIntroManager(on_cfg(quiet_when_person_near=True), settle_sec=0)
        m.update({}, {}, 0.0)
        self.assertEqual(kinds(m.update({}, {"person_near": True}, 1.0)), ["playlist_stop"])

    def test_disabled_parts_not_sent_and_volume_is_product_capped(self):
        cfg = on_cfg(volume=3)
        cfg["parts"][1]["enabled"] = False
        cfg["parts"][0]["volume"] = 2
        cfg["parts"][2]["volume"] = 1.5
        op = pr.playlist_op(cfg)
        self.assertEqual([p["volume"] for p in op["parts"]], [5.0, 4.5])
        self.assertEqual(len(op["parts"]), 2)
        self.assertEqual(len(pr.prepare_texts(cfg)), 3)  # 無効のパートも事前に作っておく

    def test_no_enabled_part_sends_nothing(self):
        cfg = on_cfg()
        for p in cfg["parts"]:
            p["enabled"] = False
        m = pr.PrIntroManager(cfg)
        self.assertEqual(m.update({}, {}, 0.0), [])

    def test_stop_all(self):
        m = pr.PrIntroManager(on_cfg(), settle_sec=0)
        m.update({}, {}, 0.0)
        self.assertEqual(m.stop_all(), [{"op": "playlist_stop", "id": "pr_intro"}])
        self.assertEqual(m.stop_all(), [])

    def test_existing_announce_manager_unaffected(self):
        """既存の状況アナウンスは、紹介の設定・状態に関係なく従来どおり動く(紹介は別のマネージャー)。"""
        mgr = ae.AnnounceManager(ae.merged_config({"enabled": True}, True))
        ops = mgr.update({"moving_notice": True}, {}, 0.0)
        self.assertEqual([o["id"] for o in ops if o["op"] == "loop"], ["moving_notice"])


# ---------------------------------------------------------------- 4. Controller(2台目はスタブ)
class _Bridge(BaseHTTPRequestHandler):
    """2台目のスタブ: 文章ごとの作成状態を持ち、合成は呼ばれたら記録するだけ。"""
    posts = []
    ready = set()
    synth = []
    muted = False
    fail_prepare = False

    def log_message(self, *a):
        pass

    def _reply(self, code, body):
        data = json.dumps(body, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path.startswith("/announce_state"):
            self._reply(200, {"ok": True, "muted": _Bridge.muted, "loops": {"pr_intro": {"active": True, "next_part": 1, "rounds": 2}},
                              "once": {}, "cached": {}, "cache": {"items": 3}, "announce_volume": 2.5})
        else:
            self._reply(404, {"error": "not found"})

    def do_POST(self):
        n = int(self.headers.get("Content-Length", "0"))
        body = json.loads(self.rfile.read(n).decode())
        _Bridge.posts.append((self.path, body))
        if self.path == "/announce_prepare":
            items = []
            for t in body["texts"]:
                if t not in _Bridge.ready and body.get("start"):
                    _Bridge.synth.append(t)  # 実機は背景で合成する(ここでは「依頼された」ことだけ記録)
                    _Bridge.ready.add(t)
                items.append({"state": "ready" if t in _Bridge.ready else "missing", "error": ""})
            self._reply(200, {"ok": True, "items": items, "prepare": {"queued": 0, "synthesizing": False}})
        elif self.path == "/announce":
            self._reply(200, {"ok": True, "muted": True} if _Bridge.muted else {"ok": True})
        else:
            self._reply(404, {"error": "not found"})


def serve(handler):
    srv = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, srv.server_address[1]


def reset_bridge():
    _Bridge.posts, _Bridge.ready, _Bridge.synth, _Bridge.muted = [], set(), [], False


class ControllerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge, cls.bport = serve(_Bridge)
        cls.url = f"http://127.0.0.1:{cls.bport}"

    @classmethod
    def tearDownClass(cls):
        cls.bridge.shutdown()
        cls.bridge.server_close()

    def setUp(self):
        reset_bridge()
        self.c = pr.PrIntroController(pr.PrIntroStore(newpath()), ap.AnnounceBridgeProxy(self.url))

    def test_payload_default_is_placeholder_and_off(self):
        p = self.c.payload()
        self.assertTrue(p["ok"])
        self.assertFalse(p["exists"])
        self.assertFalse(p["config"]["enabled"])
        self.assertTrue(p["placeholder"])
        self.assertTrue(all(q["placeholder"] for q in p["config"]["parts"]))
        self.assertTrue(p["bridge"]["ok"])
        self.assertEqual(len(p["bridge"]["items"]), len(p["config"]["parts"]))
        self.assertEqual(p["bridge"]["loop"]["next_part"], 1)
        self.assertEqual(p["meta"]["limits"]["text_max"], pr.TEXT_MAX)
        # 状態の取得だけでは合成を依頼しない
        self.assertEqual(_Bridge.synth, [])

    def test_save_requests_synthesis_once_per_changed_text(self):
        code, r = self.c.save({"enabled": True, "parts": good_parts(2)})
        self.assertEqual((code, r["ok"]), (200, True))
        self.assertEqual(_Bridge.synth, ["本番の文章0です。", "本番の文章1です。"])
        # 間隔だけの保存では音声作成を依頼しない
        n = len(_Bridge.posts)
        self.c.save({"gap_sec": 12})
        self.assertEqual(len(_Bridge.posts), n)
        # 同じ内容で保存し直しても、2台目に作成済みなら再合成を依頼しない
        _Bridge.synth.clear()
        self.c.save({"parts": good_parts(2)})
        self.assertEqual(_Bridge.synth, [])
        # 1つだけ差し替えると、そのパートだけ合成される
        parts = good_parts(2)
        parts[1]["text"] = "差し替えた本番の文章です。"
        self.c.save({"parts": parts})
        self.assertEqual(_Bridge.synth, ["差し替えた本番の文章です。"])

    def test_save_validation_400_does_not_touch_bridge(self):
        for bad in ({}, {"parts": [{"text": "あ" * 401}]}, {"gap_sec": 0}, {"x": 1}, [1], "x"):
            code, r = self.c.save(bad)
            self.assertEqual(code, 400, bad)
            self.assertFalse(r["ok"])
        self.assertEqual(_Bridge.posts, [])
        self.assertFalse(self.c.payload()["exists"])

    def test_bridge_down_still_saves(self):
        c = pr.PrIntroController(pr.PrIntroStore(newpath()), ap.AnnounceBridgeProxy("http://127.0.0.1:1", 0.3))
        p = c.payload()
        self.assertFalse(p["bridge"]["ok"])
        code, r = c.save({"enabled": True, "parts": good_parts()})
        self.assertEqual((code, r["ok"]), (200, True))
        self.assertFalse(r["prepare"]["started"])
        self.assertIn("自動で作成", r["message"])
        self.assertTrue(c.payload()["config"]["enabled"])
        self.assertFalse(c.prepare({})[1]["ok"])
        self.assertFalse(c.test({"index": 0})[1]["ok"])

    def test_unconfigured(self):
        c = pr.PrIntroController(pr.PrIntroStore(newpath()), ap.AnnounceBridgeProxy(""))
        self.assertFalse(c.payload()["configured"])
        self.assertTrue(c.save({"parts": good_parts()})[1]["ok"])

    def test_test_play_requires_prepared_audio_and_never_synthesizes(self):
        self.c.store.save({"parts": good_parts(2)})
        code, r = self.c.test({"index": 0})
        self.assertEqual((code, r["ok"]), (200, False))
        self.assertIn("まだ作成されていません", r["message"])
        self.assertFalse([p for p in _Bridge.posts if p[0] == "/announce"])
        self.assertEqual(_Bridge.synth, [])
        _Bridge.ready.add("本番の文章0です。")
        code, r = self.c.test({"index": 0})
        self.assertEqual((r["ok"], r["played"]), (True, True))
        path, body = _Bridge.posts[-1]
        self.assertEqual((path, body["cached_only"], body["text"], body["id"]), ("/announce", True, "本番の文章0です。", "pr_test"))
        _Bridge.muted = True
        self.assertTrue(self.c.test({"index": 0})[1]["muted"])

    def test_test_and_prepare_validation(self):
        for b in ({}, {"index": -1}, {"index": 99}, {"index": "0"}, {"index": True}, {"index": 0, "x": 1}, [1]):
            self.assertEqual(self.c.test(b)[0], 400, b)
        for b in ({"force": "yes"}, {"x": 1}, [1]):
            self.assertEqual(self.c.prepare(b)[0], 400, b)
        self.assertEqual(self.c.prepare({})[0], 200)

    def test_prepare_force_passes_flag(self):
        self.c.prepare({"force": True})
        self.assertTrue(_Bridge.posts[-1][1]["force"])


# ---------------------------------------------------------------- 5. AnnounceSender
class SenderTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge, cls.bport = serve(_Bridge)
        cls.url = f"http://127.0.0.1:{cls.bport}"

    @classmethod
    def tearDownClass(cls):
        cls.bridge.shutdown()
        cls.bridge.server_close()

    def setUp(self):
        reset_bridge()
        self.s = an.AnnounceSender(self.url, 2.0, status_every_sec=999)
        self.s._last_status = 1e18  # /status の確認を抑える

    def test_playlist_ops_go_to_announce_loop_and_prepare(self):
        m = pr.PrIntroManager(on_cfg(), settle_sec=0)
        for op in m.update({}, {}, 0.0):
            self.s.handle(op)
        paths = [p[0] for p in _Bridge.posts]
        self.assertEqual(paths, ["/announce_prepare", "/announce_loop"])
        loop = _Bridge.posts[1][1]
        self.assertEqual((loop["id"], loop["enabled"], loop["gap_sec"], loop["interval_sec"]), ("pr_intro", True, 8.0, 120.0))
        self.assertEqual(len(loop["parts"]), 3)
        self.assertNotIn("text", loop)

    def test_master_off_still_sends_stop_and_prepare(self):
        self.s.voice_enabled = False
        self.s.handle({"op": "playlist", "id": "pr_intro", "parts": [{"text": "あ", "volume": 1}], "interval_sec": 60, "gap_sec": 5})
        self.assertEqual(_Bridge.posts, [])
        self.s.handle({"op": "playlist_stop", "id": "pr_intro"})
        self.s.handle({"op": "prepare", "texts": ["あ"]})
        self.assertEqual([p[0] for p in _Bridge.posts], ["/announce_loop", "/announce_prepare"])

    def test_node_wires_the_pr_manager_and_isolates_failures(self):
        src = (PKG / "nokosahen_core" / "announce_node.py").read_text(encoding="utf-8")
        self.assertIn("PrIntroManager", src)
        self.assertIn("pr_intro_file", src)
        # 紹介の判定の例外が、状況アナウンスを巻き込まない(try/exceptで囲む)
        self.assertRegex(src, r"try:\s*\n\s*ops = ops \+ self\._pr_manager\.update")
        for banned in ("cmd_vel", "create_publisher(Twist"):
            self.assertNotIn("create_publisher(Twist", src)


# ---------------------------------------------------------------- 6. ダッシュボードの権限
class FakeTeleop:
    soft_estop_engaged = False


def http(method, url, body=None, headers=None, timeout=10):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method=method, headers=headers or {})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read().decode("utf-8"))
        finally:
            e.close()


class DashboardApiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge, cls.bport = serve(_Bridge)
        cls.url = f"http://127.0.0.1:{cls.bport}"

    @classmethod
    def tearDownClass(cls):
        cls.bridge.shutdown()
        cls.bridge.server_close()

    def setUp(self):
        reset_bridge()
        self.ctrl = pr.PrIntroController(pr.PrIntroStore(newpath()), ap.AnnounceBridgeProxy(self.url))
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, FakeTeleop(), pr_intro=self.ctrl,
            announce_settings_html_getter=build_announce_settings_html,
            auth_username="TL", auth_password="pw", enforce_single_controller=False)
        self.srv, self.port = serve(handler)
        self.tl = self.login("TL", "pw")
        self.guest = self.login("guest", "")

    def tearDown(self):
        self.srv.shutdown()
        self.srv.server_close()

    def login(self, user, pw):
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.port}/api/login", data=json.dumps({"username": user, "password": pw}).encode(),
            method="POST", headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.headers.get("Set-Cookie", "").split(";")[0]

    def call(self, method, path, body=None, cookie=None):
        h = {"Content-Type": "application/json"}
        c = self.tl if cookie is None else cookie
        if c:
            h["Cookie"] = c
        return http(method, f"http://127.0.0.1:{self.port}{path}", body=body, headers=h)

    def test_requires_login(self):
        self.assertEqual(self.call("GET", "/api/pr_intro", cookie="")[0], 401)
        for path, body in (("/api/pr_intro", {"parts": good_parts()}), ("/api/pr_intro/prepare", {}), ("/api/pr_intro/test", {"index": 0})):
            self.assertEqual(self.call("POST", path, body, cookie="")[0], 401, path)
        self.assertEqual(_Bridge.posts, [])

    def test_guest_views_but_cannot_write(self):
        for ck in (self.tl, self.guest):
            code, d = self.call("GET", "/api/pr_intro", cookie=ck)
            self.assertEqual((code, d["ok"]), (200, True))
        for path, body in (("/api/pr_intro", {"parts": good_parts()}), ("/api/pr_intro/prepare", {}), ("/api/pr_intro/test", {"index": 0})):
            self.assertEqual(self.call("POST", path, body, cookie=self.guest)[0], 403, path)
        self.assertFalse([p for p in _Bridge.posts if p[1].get("start") or p[0] == "/announce"])
        self.assertFalse(self.ctrl.store.read()["exists"])

    def test_tl_save_prepare_test(self):
        code, d = self.call("POST", "/api/pr_intro", {"enabled": True, "parts": good_parts(), "interval_sec": 90})
        self.assertEqual((code, d["ok"]), (200, True))
        code, d = self.call("GET", "/api/pr_intro")
        self.assertEqual((d["config"]["enabled"], d["config"]["interval_sec"]), (True, 90))
        self.assertEqual(self.call("POST", "/api/pr_intro", {"gap_sec": 0})[0], 400)
        self.assertEqual(self.call("POST", "/api/pr_intro", {"parts": [{"text": "あ" * 401}]})[0], 400)
        self.assertEqual(self.call("POST", "/api/pr_intro/prepare", {})[0], 200)
        code, d = self.call("POST", "/api/pr_intro/test", {"index": 0})
        self.assertEqual((code, d["ok"]), (200, True))

    def test_roles_classification_and_operator_not_grantable(self):
        self.assertEqual(roles.required_capabilities("POST", "/api/pr_intro"), ["announce.pr_intro"])
        self.assertEqual(roles.required_capabilities("POST", "/api/pr_intro/prepare"), ["announce.pr_intro"])
        self.assertEqual(roles.required_capabilities("POST", "/api/pr_intro/test"), ["announce.test"])
        cap = roles.CAPABILITIES["announce.pr_intro"]
        self.assertEqual(cap.tier, roles.TIER_DEFERRED)  # operatorへの付与は保留(付与不可)
        self.assertNotIn("announce.pr_intro", [n for n, c in roles.CAPABILITIES.items() if c.tier == roles.TIER_GRANTABLE])


# ---------------------------------------------------------------- 7. 画面
class PageTest(unittest.TestCase):
    def setUp(self):
        self.html = build_announce_settings_html().decode("utf-8")

    def test_js_syntax(self):
        node = shutil.which("node")
        if not node:
            self.skipTest("nodeがありません")
        scripts = re.findall(r"<script>(.*?)</script>", self.html, re.S)
        self.assertEqual(len(scripts), 1)
        f = Path(tempfile.mkdtemp(dir=TMP)) / "page.js"
        f.write_text(scripts[0], encoding="utf-8")
        r = subprocess.run([node, "--check", str(f)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)

    def test_card_present_and_safe(self):
        h = self.html
        self.assertEqual(h.count('id="prIntro"'), 1)
        for needle in ("/api/pr_intro", "/api/pr_intro/prepare", "/api/pr_intro/test", "pr_save", "pr_prepare", "pr_add",
                       "保存して音声を作る", "文化祭の出店紹介", "既定はOFF", 'data-act="test"', "(仮)",
                       'name="pr_mode"', 'id="secEvents"', 'id="secPr"', "speech_mode"):
            self.assertIn(needle, h, needle)
        # 可変テキストはエスケープして埋め込む
        self.assertIn("escapeHtml(p.text)", h)
        self.assertIn("escapeHtml(p.title)", h)
        # 定期取得は NKField.poll だけ(setIntervalを増やさない)
        self.assertNotIn("setInterval(", h.split("window.NKField = ")[-1])

    def test_existing_card_structure_kept(self):
        for el in ("aeMaster", "aeVol", "aeVolNum", "saveBtn", "discardBtn", "confirmBar", "estopDock", "events"):
            self.assertEqual(self.html.count(f'id="{el}"'), 1, el)


if __name__ == "__main__":
    unittest.main()
