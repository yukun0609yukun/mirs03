"""iPad表示の「顔」モード(face。2026-11-08。docs/ipad_display.md)のテスト。

ロボット・iPad・ラズパイには触れない(ROSが無ければスタブ、HTTPは127.0.0.1の空きポートの本物のハンドラ)。確認すること:
- 顔のHTMLがユーザーの元のHTMLと一字も違わない(docs/robot_face_cute_blush.html と照合)
- モード: face を受け付ける/不正値は拒否、保存と再起動後の復元、ROSの文字列/JSON、回答中は保留(ROS)/断る(手動)
- /display の画面: 顔は iframe(sandbox・srcdoc)に隔離し、face モードに入るときだけ作り、出るときに取り除く
- GET /display/face: ログインが要る(未ログインは顔を返さない)、ログイン後は元のHTMLそのもの
- 管理画面のHTMLに「顔を表示」ボタン、権限 display.survey_mode(持たないアカウントは403)
実行: python3 -I ros2_ws/src/nokosahen_core/test/test_face_mode_20261108.py
"""
import json
import re
import subprocess
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import test_ipad_display as T  # noqa: E402  (ROSのスタブ・HTTPの道具を共有する)

dh = T.dh
dp = T.dp
PKG = T.PKG
REPO = PKG.parents[2]
ORIGINAL = REPO / "docs" / "robot_face_cute_blush.html"


class FaceHtmlTest(unittest.TestCase):
    def test_face_html_is_identical_to_the_original_copy(self):
        from nokosahen_core.face_html import FACE_HTML
        self.assertEqual(FACE_HTML.encode("utf-8"), ORIGINAL.read_bytes())

    def test_original_features_are_present_and_no_external_communication(self):
        from nokosahen_core.face_html import FACE_HTML
        for needle in ('<svg id="face"', 'viewBox="0 0 1000 700"', "requestAnimationFrame", "pointerdown", 'id="smileBlush"'):
            self.assertIn(needle, FACE_HTML)
        stripped = FACE_HTML.replace('xmlns="http://www.w3.org/2000/svg"', "")
        for bad in ("http://", "https://", "fetch(", "XMLHttpRequest", "WebSocket"):
            self.assertNotIn(bad, stripped)


class FaceModeHubTest(unittest.TestCase):
    def test_face_is_a_mode_with_a_label(self):
        self.assertIn("face", dh.MODES)
        self.assertEqual(dh.MODE_LABELS["face"], "顔")
        self.assertEqual(set(dh.MODES), set(dh.MODE_LABELS))

    def test_validate_and_normalize(self):
        self.assertEqual(dh.validate_mode_request({"mode": "face"}), ("face", ""))
        for bad in ({"mode": "Face"}, {"mode": "faces"}, {"mode": ""}, {"mode": "face", "x": 1}, {"mode": 1}, {}, [], None, "face"):
            self.assertIsNone(dh.validate_mode_request(bad)[0], bad)
        self.assertEqual(dh.normalize_mode("face"), "face")
        self.assertEqual(dh.normalize_mode("  FACE \n"), "face")
        self.assertEqual(dh.normalize_mode('{"mode": "face"}'), "face")
        self.assertEqual(dh.normalize_mode({"mode": "face"}), "face")
        self.assertIsNone(dh.normalize_mode("face;ls"))

    def test_persisted_and_restored_after_restart(self):
        hub, clock, d = T.new_hub("face1")
        r = hub.request_mode("face", "admin", defer=False)
        self.assertTrue(r["ok"] and r["changed"])
        self.assertEqual((d / "mode").read_text().strip(), "face")
        hub2 = dh.DisplayHub(mode_file=str(d / "mode"), survey_store=hub.store, clock=clock, wall=clock)
        self.assertEqual(hub2.mode, "face")
        st = hub2.public_state()
        self.assertEqual(st["mode"], "face")
        self.assertFalse(st["active"])
        self.assertNotIn("customer", st)
        self.assertNotIn("ocr_mock", st)
        self.assertEqual(hub2.admin_state()["mode_label"], "顔")

    def test_ros_string_and_json_forms(self):
        for i, payload in enumerate(("face", '{"mode":"face"}', {"mode": "face"})):
            hub, _, _ = T.new_hub("face2_%d" % i)
            r = hub.request_mode(payload, "ros", defer=True)
            self.assertTrue(r["ok"] and r["changed"], payload)
            self.assertEqual(hub.mode, "face")
        hub, _, _ = T.new_hub("face2b")
        self.assertFalse(hub.request_mode("face!", "ros")["ok"])
        self.assertEqual(hub.mode, "customer")

    def test_ros_node_passes_the_string_to_request_mode_defer(self):
        src = (PKG / "nokosahen_core" / "full_dashboard_node.py").read_text(encoding="utf-8").replace("\r\n", "\n")
        self.assertIn('hub.request_mode(msg.data, "ros", defer=True)', src)

    def test_switch_during_survey_answer_is_deferred_or_refused(self):
        hub, clock, _ = T.new_hub("face3")
        hub.request_mode("survey", "admin", defer=False)
        hub.session_event("start")
        r = hub.request_mode("face", "ros", defer=True)
        self.assertTrue(r["ok"] and r["deferred"] and not r["changed"])
        self.assertEqual(hub.mode, "survey")
        self.assertEqual(hub.public_state()["pending_mode"], "face")
        r = hub.request_mode("face", "admin", defer=False)
        self.assertFalse(r["ok"])
        self.assertEqual(r["code"], 409)
        self.assertTrue(hub.session.active())
        hub.session_event("end")
        clock.advance(dh.SAVED_SHOW_S + 0.1)
        hub.tick()
        self.assertEqual(hub.mode, "face")
        self.assertIsNone(hub.pending)

    def test_face_never_stops_the_robot(self):
        """survey以外では回答中にならない(ロボット停止は働かない)。顔の間は session イベントも無視される。"""
        hub, _, _ = T.new_hub("face4")
        hub.request_mode("face", "admin", defer=False)
        hub.session_event("start")
        st = hub.public_state()
        self.assertFalse(st["active"])
        self.assertEqual(st["session_state"], "idle")

    def test_leaving_face_goes_back_to_customer(self):
        hub, _, _ = T.new_hub("face5")
        hub.request_mode("face", "admin", defer=False)
        self.assertTrue(hub.request_mode("customer", "admin", defer=False)["changed"])
        self.assertIn("customer", hub.public_state())

    def test_schedule_remembers_face_as_a_baseline_mode(self):
        self.assertIn("face", T.sd.VALID_MODES)


class FacePageTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.html = dp.build_display_html().decode("utf-8")

    def test_face_view_is_an_isolated_iframe_created_only_in_face_mode(self):
        h = self.html
        self.assertIn('id="v-face"', h)
        self.assertIn("body[data-mode=face] #v-face", h)
        self.assertIn('f.setAttribute("sandbox", "allow-scripts")', h)       # 親のページと隔離(同じオリジンではない)
        self.assertIn("f.srcdoc = window.NK_FACE", h)
        self.assertIn("window.NK_FACE = ", h)
        self.assertIn('m !== "face"', h)
        self.assertIn('if (m === "face") faceEnter();', h)
        self.assertIn('if (m === "face") { faceLeave(); }', h)
        # 初期のHTMLに iframe の要素は無い(face モードに入るまで顔は動かない)
        body = h.split("</style>", 1)[1].split("<script", 1)[0]
        self.assertNotIn("<iframe", body)
        self.assertNotIn('id="face"', body)

    def test_face_svg_does_not_clash_with_page_ids(self):
        """顔の id は iframe の中だけ。親のページの id は顔のHTMLの id と重ならない。"""
        from nokosahen_core.face_html import FACE_HTML
        face_ids = set(re.findall(r'id="([^"]+)"', FACE_HTML))
        template_ids = set(re.findall(r'\sid="([^"]+)"', dp._TEMPLATE))
        self.assertTrue(face_ids)
        self.assertEqual(face_ids & template_ids, set())

    def test_face_literal_roundtrips_and_cannot_break_the_page(self):
        from nokosahen_core.face_html import FACE_HTML
        lit = dp._face_json()
        self.assertNotIn("</", lit)
        self.assertNotIn("<!--", lit)
        self.assertEqual(json.loads(lit.replace("<\\/", "</").replace("<\\!--", "<!--")), FACE_HTML)
        # 親のページの script の開始・終了の数が合う(顔の中の <script> は文字列の中)
        self.assertEqual(self.html.count("</script>"), self.html.count("<script") - lit.count("<script"))

    def test_login_page_has_no_face(self):
        login = dp.build_display_login_html().decode("utf-8")
        self.assertNotIn("NK_FACE", login)
        self.assertNotIn("robot_face", login)

    def test_page_keeps_safe_area_polling_and_login_loss_handling(self):
        h = self.html
        for needle in ("viewport-fit=cover", "/display/api/state", "authLost", "wakeLock", "orientationchange", 'id="conn"'):
            self.assertIn(needle, h)

    @unittest.skipUnless(T.NODE, "nodeが無いので省略")
    def test_enter_leave_with_a_fake_dom(self):
        m = re.search(r"  function faceEnter\(\) \{.*?\n  function faceLeave\(\) \{.*?\n  \}\n", self.html, re.S)
        self.assertTrue(m)
        js = r"""
var host = {children: [], get firstChild() { return this.children[0] || null; },
  appendChild(c) { this.children.push(c); }, removeChild(c) { this.children.splice(this.children.indexOf(c), 1); }};
var document = {createElement(t) { return {tag: t, attrs: {}, setAttribute(k, v) { this.attrs[k] = v; }}; }};
var window = {NK_FACE: "<html>face</html>"};
function $(id) { return host; }
""" + m.group(0) + r"""
faceEnter(); faceEnter();
if (host.children.length !== 1) throw new Error("iframe must be created once");
var f = host.children[0];
if (f.tag !== "iframe" || f.srcdoc !== "<html>face</html>" || f.attrs.sandbox !== "allow-scripts") throw new Error("bad iframe");
faceLeave();
if (host.children.length !== 0) throw new Error("iframe must be removed");
faceEnter(); faceLeave(); faceLeave();
console.log("ok");
"""
        r = subprocess.run([T.NODE, "-e", js], capture_output=True, text=True, timeout=30)
        self.assertEqual((r.returncode, r.stdout.strip()), (0, "ok"), r.stderr)


class FaceHttpTest(T.HttpBase):
    def test_face_url_requires_the_display_login(self):
        from nokosahen_core.face_html import FACE_HTML
        self.start()
        code, body, h = T.http("GET", self.base + "/display/face")
        self.assertEqual(code, 401)
        self.assertEqual(body, dp.build_display_login_html())           # 顔は返さずログイン画面
        self.assertNotIn(b'id="face"', body)
        # TL のログインではiPad用の入口に入れない
        tl = self.login("TL", "tl-pass")
        self.assertEqual(T.http("GET", self.base + "/display/face", cookie=tl)[0], 401)
        ck = self.dlogin()
        code, body, h = T.http("GET", self.base + "/display/face", cookie=ck)
        self.assertEqual(code, 200)
        self.assertEqual(h["Content-Type"], "text/html; charset=utf-8")
        self.assertEqual(body, FACE_HTML.encode("utf-8"))
        self.assertEqual(body, ORIGINAL.read_bytes())
        self.assertEqual(h["Cache-Control"], "no-store")
        self.assertIn("frame-ancestors 'none'", h["Content-Security-Policy"])
        self.assertEqual(T.http("GET", self.base + "/display/face/", cookie=ck)[0], 404)
        self.assertEqual(T.http("POST", self.base + "/display/face", {}, cookie=ck)[0], 404)

    def test_admin_switch_to_face_is_seen_by_ipad(self):
        self.start()
        ck = self.login("TL", "tl-pass")
        code, d, _ = T.jhttp("POST", self.base + "/api/survey/mode", {"mode": "face"}, cookie=ck)
        self.assertEqual((code, d["ok"]), (200, True), d)
        self.assertEqual(T.jhttp("GET", self.base + "/api/survey/state", cookie=ck)[1]["state"]["mode"], "face")
        ipad = self.dlogin()
        self.assertEqual(T.jhttp("GET", self.base + "/display/api/state", cookie=ipad)[1]["mode"], "face")
        # /display は face のときも同じ本体の画面(顔はJSが作る)。ログイン前はログイン画面のまま
        self.assertEqual(T.http("GET", self.base + "/display")[1], dp.build_display_login_html())
        self.assertEqual(T.http("GET", self.base + "/display", cookie=ipad)[1], dp.build_display_html())
        T.jhttp("POST", self.base + "/api/survey/mode", {"mode": "customer"}, cookie=ck)
        self.assertEqual(T.jhttp("GET", self.base + "/display/api/state", cookie=ipad)[1]["mode"], "customer")

    def test_permission_display_survey_mode(self):
        self.start(perms=["display.survey_mode"])
        op = self.login("kari-op", T.OP_PASSWORD)
        self.assertEqual(T.jhttp("POST", self.base + "/api/survey/mode", {"mode": "face"}, cookie=op)[0], 200)
        guest = self.login("guest", "")
        self.assertEqual(T.jhttp("POST", self.base + "/api/survey/mode", {"mode": "face"}, cookie=guest)[0], 403)
        self.assertEqual(T.jhttp("POST", self.base + "/api/survey/mode", {"mode": "face"})[0], 401)
        self.assertEqual(T.jhttp("POST", self.base + "/api/survey/mode", {"mode": "faces"}, cookie=op)[0], 400)

    def test_operator_without_the_permission_gets_403(self):
        self.start(perms=["display.menu_edit"])
        op = self.login("kari-op", T.OP_PASSWORD)
        code, d, _ = T.jhttp("POST", self.base + "/api/survey/mode", {"mode": "face"}, cookie=op)
        self.assertEqual(code, 403)
        self.assertEqual(d["missing"], ["display.survey_mode"])
        self.assertEqual(self.hub.mode, "customer")


class FaceAdminUiTest(unittest.TestCase):
    SRC = (PKG / "nokosahen_core" / "full_dashboard_node.py").read_text(encoding="utf-8").replace("\r\n", "\n")

    def test_button_next_to_the_other_display_buttons(self):
        s = self.SRC
        self.assertIn('<button id="surveyToFaceBtn" type="button" data-mode="face">顔を表示</button>', s)
        i = s.index('id="surveyToInvBtn"')
        j = s.index('id="surveyToFaceBtn"')
        k = s.index('id="surveyCsvLink"')
        self.assertTrue(i < j < k)
        b = s.index('id="surveyBlock"')
        self.assertIn('data-cap="display.survey_mode"', s[b - 40:b + 80])
        self.assertIn("お客さん向けに戻す", s[b:k])
        for needle in ('svEls.toFace.addEventListener("click", () => svAsk("face"))', 'svEls.toFace.disabled = readOnlyAccount || s.mode === "face"',
                       'face: "顔"', 's.mode === "face" ? "顔"', 'mode === "face" ? "頭部ディスプレイにロボットの顔を出します'):
            self.assertIn(needle, s)
        self.assertIn('postForMessage("/api/survey/mode"', s)           # 既存のボタンと同じAPI

    def test_summary_badge_shows_face(self):
        nav = (PKG / "nokosahen_core" / "nav_layout.py").read_text(encoding="utf-8")
        self.assertIn("/^顔/.test(sm)", nav)

    def test_mobile_page_has_no_display_card(self):
        """スマホ版には頭部ディスプレイの切替カード自体が無い(ボタンの並びがあるのはPC版の設定タブだけ)。"""
        dc = (PKG / "nokosahen_core" / "dashboard_common.py").read_text(encoding="utf-8")
        self.assertNotIn("surveyToInvBtn", dc)


if __name__ == "__main__":
    unittest.main()
