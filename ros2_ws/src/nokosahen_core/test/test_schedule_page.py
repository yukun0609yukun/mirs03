"""かんたん時間割ページ(/schedule)のHTML・配信のテスト(2026-10-23、ROS不要)。

- HTMLが生成でき、必要なid・APIパス文字列・説明バナー・リンク・時計の表示要素が含まれる。
- JSの構文をnodeで検査する(nodeが無ければスキップ)。
- dashboard_commonが /schedule を既存ページと同じ認証で配信する(APIの実装はバックエンド側で、ここでは確認しない)。
"""
import json
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import test_behavior_rules_api as tra  # noqa: E402  (スタブ導入・FakeNode等を再利用)
from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import schedule_page as sp  # noqa: E402
from nokosahen_core.nav_layout import LINK_GROUPS  # noqa: E402
from nokosahen_core.page_guide import ROBOT_CLOCK_API  # noqa: E402


class SchedulePageHtml(unittest.TestCase):
    def setUp(self):
        self.html = sp.build_schedule_html().decode("utf-8")

    def test_title_and_no_leftover_placeholders(self):
        self.assertIn("<title>のこさへん - かんたん時間割</title>", self.html)
        self.assertNotIn("__PAGE_GUIDE", self.html)
        self.assertNotIn("__API_", self.html)

    def test_api_paths(self):
        for path in (sp.API_SCHEDULE, sp.API_SCHEDULE_PREVIEW, sp.API_SCHEDULE_RUN,
                     sp.API_SCHEDULE_STOP, sp.API_PINS, sp.API_BOUNDARY, ROBOT_CLOCK_API):
            self.assertIn('"%s"' % path, self.html, path)
        self.assertEqual(sp.API_SCHEDULE_PREVIEW, "/api/schedule/preview")
        self.assertEqual(sp.API_SCHEDULE_RUN, "/api/schedule/run")
        self.assertEqual(sp.API_SCHEDULE_STOP, "/api/schedule/stop")
        self.assertEqual(ROBOT_CLOCK_API, "/api/robot_clock")

    def test_required_ids_and_ui(self):
        ids = re.findall(r'id="([^"]+)"', self.html)
        self.assertEqual(len(ids), len(set(ids)), "id重複")
        for i in ("pgClockTime", "pgClockDiff", "pgClockWarn", "runBtn", "stopBtn", "runBadge", "curItem",
                  "nextSwitch", "runOnStartup", "entries", "addBtn", "exampleBtn", "previewBody",
                  "saveBtn", "previewBtn", "estopBtn", "schName", "pinInfo"):
            self.assertIn('id="%s"' % i, self.html, i)
        for s in ("このスケジュールで動け", "本番は範囲の角にコーンを置く想定", "解除してNav2で走行",
                  "日本語のプレビュー", "時計を取得できません", "既定はOFF", "機体の時計"):
            self.assertIn(s, self.html, s)

    def test_banner_and_links(self):
        for href in ("/schedule", "/behavior_tree", "/behavior_rules"):
            self.assertIn('href="%s"' % href, self.html)
        self.assertIn('id="pgGuide"', self.html)
        self.assertIn("pg-clock big", self.html)

    def test_face_policy_selectable(self):
        self.assertIn('["face", "人の方を向く"]', self.html)

    def test_phone_layout(self):
        self.assertIn('name="viewport"', self.html)
        self.assertIn("@media (max-width:640px)", self.html)

    def test_page_js_is_valid(self):
        node = shutil.which("node")
        if not node:
            self.skipTest("nodeが無い")
        js = re.search(r"<script>(.*)</script>", self.html, re.S).group(1)
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "page.js"
            p.write_text(js, encoding="utf-8")
            r = subprocess.run([node, "--check", str(p)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)

    def test_nav_link_exists(self):
        links = [item[0] for _g, _t, items in LINK_GROUPS for item in items]
        self.assertIn("/schedule", links)


def _js_function(js, name):
    """JS文字列から `function name(...) {...}` を波括弧の対応で取り出す(文字列・正規表現リテラル内の波括弧は無い前提の関数だけに使う)。"""
    start = js.index("function %s(" % name)
    i = js.index("{", start)
    depth = 0
    for j in range(i, len(js)):
        if js[j] == "{":
            depth += 1
        elif js[j] == "}":
            depth -= 1
            if depth == 0:
                return js[start:j + 1]
    raise AssertionError(name)


class ScheduleDisplayUi(unittest.TestCase):
    """頭部ディスプレイの画面(display)の指定UI(2026-10-23)。"""

    def setUp(self):
        self.html = sp.build_schedule_html().decode("utf-8")
        self.js = re.search(r"<script>(.*)</script>", self.html, re.S).group(1)

    def test_select_state_and_texts_exist(self):
        ids = re.findall(r'id="([^"]+)"', self.html)
        self.assertEqual(len(ids), len(set(ids)), "id重複")
        self.assertIn('id="dispState"', self.html)
        self.assertIn('data-f="display"', self.html)
        for s in ("頭部ディスプレイの画面", "お客さん向け", "アンケート", "OCRテスト", "そのまま", "保留中", "回答が消えない"):
            self.assertIn(s, self.html, s)
        for m in ("keep", "customer", "survey", "ocrtest"):
            self.assertIn('"%s"' % m, self.js, m)

    def test_existing_ids_and_apis_are_kept(self):
        for i in ("runBtn", "stopBtn", "runBadge", "curItem", "nextSwitch", "entries", "addBtn", "saveBtn",
                  "previewBody", "mapCv", "stateRaw", "runOnStartup"):
            self.assertIn('id="%s"' % i, self.html, i)

    def test_examples_use_display(self):
        self.assertIn('display: "customer"', self.html)

    def _node(self, script):
        node = shutil.which("node")
        if not node:
            self.skipTest("nodeが無い")
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "t.js"
            p.write_text(script, encoding="utf-8")
            r = subprocess.run([node, str(p)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)
        return r.stdout.strip()

    def test_entry_roundtrip_and_display_text_in_node(self):
        funcs = "\n".join(_js_function(self.js, n) for n in ("entryFromApi", "entryToApi", "displayText"))
        script = """
const WD = [["mon","月"],["tue","火"],["wed","水"],["thu","木"],["fri","金"],["sat","土"],["sun","日"]];
const RECT_KEYS = [["x_min"],["y_min"],["x_max"],["y_max"]];
const DISPLAY_JA = { customer: "お客さん向け", survey: "アンケート", ocrtest: "OCRテスト" };
function pinName(id, fb) { return fb || ""; }
function toNum(v) { if (v === null || v === undefined || String(v).trim() === "") return null; const n = Number(v); return isFinite(n) ? n : null; }
%s
const out = {};
out.keep_omits = entryToApi(entryFromApi({ id: "a", start: "09:00", destination: { type: "stay" }, state: { policy: "normal" } }));
out.survey = entryToApi(entryFromApi({ id: "a", start: "09:00", destination: { type: "stay" }, state: { policy: "normal" }, display: { mode: "survey" } }));
out.legacy = entryFromApi({ id: "a", start: "09:00" }).display;
out.t_pending = displayText({ display: { state: "pending", current: "survey", requested: "customer", pending_reason: "アンケート回答中のため" } });
out.t_error = displayText({ display: { state: "error", current: "customer", error: "ラズパイに届きません" } });
out.t_ok = displayText({ display: { state: "ok", current: "survey" } });
out.t_none = displayText({});
out.t_idle = displayText({ display: { state: "idle" } });
console.log(JSON.stringify(out));
""" % funcs
        out = json.loads(self._node(script))
        self.assertNotIn("display", out["keep_omits"])
        self.assertEqual(out["survey"]["display"], {"mode": "survey"})
        self.assertEqual(out["legacy"], "keep")
        self.assertIn("いまアンケート", out["t_pending"])
        self.assertIn("保留中", out["t_pending"])
        self.assertIn("回答中のため", out["t_pending"])
        self.assertIn("お客さん向け", out["t_pending"])
        self.assertIn("エラー", out["t_error"])
        self.assertIn("走行には影響しません", out["t_error"])
        self.assertIn("いまアンケート", out["t_ok"])
        self.assertEqual(out["t_none"], "指定なし(そのまま)")
        self.assertEqual(out["t_idle"], "指定なし(そのまま)")


class ScheduleStateAlerts(unittest.TestCase):
    """バックエンドの state(clock_suspect / status / safety_hold / boundary_override / armed / path_mismatch)の注意表示。"""

    def setUp(self):
        self.html = sp.build_schedule_html().decode("utf-8")
        self.js = re.search(r"<script>(.*)</script>", self.html, re.S).group(1)

    def test_html_has_alert_container_and_texts(self):
        self.assertIn('id="stateAlerts"', self.html)
        ids = re.findall(r'id="([^"]+)"', self.html)
        self.assertEqual(len(ids), len(set(ids)), "id重複")
        for s in ("機体の時計が合っていないため、時刻の条件は成立しません(スケジュールは動きません)",
                  "directorから応答がありません", "安全のため停止中(緊急停止・範囲外など)",
                  "動ける範囲を上書き中", "木のファイルのパスが一致していません", "起動後2分たってから"):
            self.assertIn(s, self.html, s)
        for i in ("runBtn", "stopBtn", "runBadge", "curItem", "nextSwitch", "dispState", "stateRaw", "runOnStartup"):
            self.assertIn('id="%s"' % i, self.html, i)
        self.assertIn("renderStateAlerts(d, st)", self.js)

    def test_alert_logic_in_node(self):
        node = shutil.which("node")
        if not node:
            self.skipTest("nodeが無い")
        funcs = "\n".join(_js_function(self.js, n) for n in ("boundaryText", "stateAlerts"))
        script = """
%s
const cases = {
  ok: stateAlerts({}, { status: "ok", running: true, armed: true }),
  clock: stateAlerts({}, { status: "ok", clock_suspect: true }),
  clock_from_robot_clock: stateAlerts({ robot_clock: { clock_suspect: true } }, { status: "ok" }),
  noresp: stateAlerts({}, { status: "no_response", message: "x" }),
  waiting: stateAlerts({}, { status: "waiting" }),
  hold: stateAlerts({}, { status: "ok", safety_hold: true, running: true, armed: true }),
  rel: stateAlerts({}, { status: "ok", boundary_override: { mode: "release" }, running: true, armed: true }),
  set: stateAlerts({}, { status: "ok", boundary_override: { mode: "set", x_min: 0, y_min: -1, x_max: 4, y_max: 2 }, running: true, armed: true }),
  armed_idle: stateAlerts({}, { status: "ok", armed: true, running: false }),
  startup: stateAlerts({}, { status: "ok", armed: false, running: false, startup_pending: true }),
  mismatch: stateAlerts({}, { status: "ok", path_mismatch: true }),
};
console.log(JSON.stringify(cases));
""" % funcs
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "t.js"
            p.write_text(script, encoding="utf-8")
            r = subprocess.run([node, str(p)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)
        c = json.loads(r.stdout)

        def has(case, kind, text):
            return any(k == kind and text in t for k, t in c[case])
        self.assertEqual(c["ok"], [])
        self.assertTrue(has("clock", "err", "機体の時計が合っていない"))
        self.assertTrue(has("clock_from_robot_clock", "err", "機体の時計が合っていない"))
        self.assertTrue(has("noresp", "err", "directorから応答がありません"))
        self.assertTrue(has("waiting", "warn", "応答を待っています"))
        self.assertTrue(has("hold", "warn", "安全のため停止中"))
        self.assertTrue(has("rel", "info", "動ける範囲を上書き中(解除)"))
        self.assertTrue(has("set", "info", "指定: x 0〜4 m、y -1〜2 m"))
        self.assertTrue(has("armed_idle", "warn", "まだ実行中になっていません"))
        self.assertTrue(has("startup", "info", "自動実行を待っています"))
        self.assertFalse(has("startup", "warn", "まだ実行中になっていません"))
        self.assertTrue(has("mismatch", "err", "木のファイルのパスが一致していません"))
        # 応答が無いときは、armed なのに running でない警告は重ねない
        self.assertFalse(has("noresp", "warn", "まだ実行中になっていません"))


class SchedulePageServing(unittest.TestCase):
    def setUp(self):
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, tra.FakeTeleop(),
            schedule_html_getter=sp.build_schedule_html,
            auth_username="TL", auth_password="pw", enforce_single_controller=False,
        )
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        self.port = self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.tl = self.login("TL", "pw")
        self.guest = self.login("guest", "")

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    login = tra.ApiBase.login
    call = tra.ApiBase.call

    def test_served_to_logged_in_accounts(self):
        for cookie in (self.tl, self.guest):
            st, html = self.call("GET", "/schedule", text=True, cookie=cookie)
            self.assertEqual(st, 200)
            self.assertIn("かんたん時間割", html)

    def test_requires_login(self):
        st, body = self.call("GET", "/schedule", cookie="", text=True)   # ログイン画面へ転送される
        self.assertNotIn("<title>のこさへん - かんたん時間割</title>", body)


if __name__ == "__main__":
    unittest.main()
