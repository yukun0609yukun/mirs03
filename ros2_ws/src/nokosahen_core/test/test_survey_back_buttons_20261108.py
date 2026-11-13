"""アンケートの「1つ前にもどる」「さいしょにもどる」(2026-11-08、ユーザーの依頼)。

- 現行版 r2 で showBack が True。全設問(1問目含む)に線画アイコンの2ボタンを出す(待機・完了では出ない)。1問目の「1つ前」は待機へ、「さいしょ」は無効
- 戻っても答えは残る。最後まで答えると送信は1回だけ。送信内容は検証(validate_response)を通る
- node があれば、画面のJSをダミーのDOMで実際に動かして状態遷移を検証する(無ければその部分は skip)
"""
import json
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core import display_page as dp  # noqa: E402
from nokosahen_core import display_survey as dsv  # noqa: E402

HTML = dp.build_display_html().decode("utf-8")
NODE = shutil.which("node")


class StaticTests(unittest.TestCase):
    def test_show_back_true_in_r2_only(self):
        self.assertIs(dsv.SURVEY_DEF["showBack"], True)
        self.assertEqual(dsv.SURVEY_DEF["version"], "r2")
        self.assertIn('"showBack": true', HTML)
        self.assertNotIn("showBack", dsv.SURVEY_DEF_R1)

    def test_buttons_exist_with_wording_inside_ask_screen_only(self):
        m = re.search(r'<section class="screen" id="ask" hidden>(.*?)</section>', HTML, re.S)
        self.assertIsNotNone(m)
        ask = m.group(1)
        for bid, label in (("back", "1つ前にもどる"), ("first", "さいしょにもどる")):
            m2 = re.search(r'<button[^>]*id="%s"[^>]*>(.*?)</button>' % bid, ask, re.S)
            self.assertIsNotNone(m2, bid)
            tag = re.search(r'<button[^>]*id="%s"[^>]*>' % bid, ask).group(0)
            self.assertIn('aria-label="%s"' % label, tag)
            self.assertIn('title="%s"' % label, tag)
            inner = m2.group(1)
            self.assertTrue(inner.strip().startswith("<svg"), bid)       # 文字ではなく線画SVG
            self.assertEqual(re.sub(r"<[^>]*>", "", inner).strip(), "")  # 見える文字なし
        css = re.search(r"\.sbottom \.btn\.nav svg\{([^}]*)\}", HTML).group(1)
        for k in ("fill:none", "stroke:currentColor", "stroke-linecap:round"):
            self.assertIn(k, css)
        for sid in ("idle", "done"):
            sec = re.search(r'<section class="screen" id="%s".*?</section>' % sid, HTML, re.S).group(0)
            self.assertNotIn('id="back"', sec)
            self.assertNotIn('id="first"', sec)
        # 「つぎへ」は右端のまま(最後の子)で、2つのボタンはその左に並ぶ
        order = [ask.index(i) for i in ('id="back"', 'id="first"', 'id="skip"', 'id="next"')]
        self.assertEqual(order, sorted(order))

    def test_shown_on_all_questions_and_buttons_do_not_need_an_answer(self):
        self.assertIn('$("back").hidden = !S.showBack;', HTML)
        self.assertIn('$("first").hidden = !S.showBack;', HTML)
        self.assertIn('$("first").disabled = idx === 0;', HTML)
        self.assertNotIn('$("back").disabled', HTML)
        self.assertIn('$("first").addEventListener("click"', HTML)

    def test_buttons_are_touch_sized_and_spaced(self):
        css = re.search(r"\.sbottom \.btn\.nav\{([^}]*)\}", HTML).group(1)
        self.assertIn("margin-right", css)
        self.assertIn("min-height:var(--tap)", re.search(r"\n\.btn\{(min-height[^}]*)\}", HTML).group(1))
        nav = re.search(r"\.sbottom \.btn\.nav\{([^}]*)\}", HTML).group(1)
        self.assertIn("min-height:56px", nav)
        self.assertIn("max(56px", nav)


# ---- JS の状態遷移(node) ----
_STUB = r"""
function mkEl(id) {
  var cls = new Set();
  var el = {id: id, hidden: false, disabled: false, textContent: "", innerHTML: "", children: [], style: {setProperty: function () {}},
    listeners: {}, attrs: {},
    classList: {add: function (c) { cls.add(c); }, remove: function (c) { cls.delete(c); }, contains: function (c) { return cls.has(c); },
                toggle: function (c, on) { if (on === undefined) on = !cls.has(c); if (on) cls.add(c); else cls.delete(c); }},
    appendChild: function (c) { this.children.push(c); return c; },
    setAttribute: function (k, v) { this.attrs[k] = v; },
    addEventListener: function (ev, fn) { (this.listeners[ev] = this.listeners[ev] || []).push(fn); },
    fire: function (ev) { (this.listeners[ev] || []).forEach(function (f) { f({key: ""}); }); },
    querySelectorAll: function (sel) { var out = []; (function w(n) { n.children.forEach(function (c) { if (c.classList.contains(sel.slice(1))) out.push(c); w(c); }); })(this); return out; }};
  Object.defineProperty(el, "className", {get: function () { return Array.from(cls).join(" "); }, set: function (v) { cls.clear(); String(v).split(/\s+/).filter(Boolean).forEach(function (c) { cls.add(c); }); }});
  Object.defineProperty(el, "textContent", {get: function () { return this._t || ""; }, set: function (v) { this._t = v; if (v === "") this.children = []; }});
  Object.defineProperty(el, "innerHTML", {get: function () { return this._h || ""; }, set: function (v) {
    this._h = v; if (v === "<span></span><span></span>") this.children = [mkEl(""), mkEl("")]; }});
  return el;
}
var els = {};
var document = {getElementById: function (id) { return els[id] || (els[id] = mkEl(id)); }, addEventListener: function () {},
                createElement: function () { return mkEl(""); }};
var window = {addEventListener: function () {}, crypto: undefined};
var localStorage = {getItem: function () { return null; }, setItem: function () {}};
var mode = "survey";
var calls = [];
var fetch = function (url, o) { calls.push({url: url, body: o && o.body ? JSON.parse(o.body) : null}); return Promise.resolve({ok: true, status: 200}); };
var setInterval = function () { return 1; }, clearInterval = function () {}, setTimeout = function () { return 1; }, clearTimeout = function () {};
var stats = function () {};
var $ = function (id) { return document.getElementById(id); };
"""

_DRIVER = r"""
function tiles() { return $("qwrap").querySelectorAll(".tile"); }
function selected() { return tiles().map(function (t, i) { return t.classList.contains("sel") ? i : -1; }).filter(function (i) { return i >= 0; }); }
function vis(id) { return !$(id).hidden; }
function snap() { return {qno: $("qno").textContent, back: vis("back"), first: vis("first"), firstDisabled: $("first").disabled, idle: vis("idle"), ask: vis("ask"), nextDisabled: $("next").disabled, sel: selected()}; }
var log = {};
$("idle").fire("click");
log.q1 = snap();
// 1問目の「さいしょ」は無効、「1つ前」= 待機へ(答えは捨てる)。もう一度始めて以降は通常どおり
$("first").fire("click"); log.first_at_q1 = snap();
tiles()[1].fire("click");
$("back").fire("click"); log.back_at_q1 = snap();
$("idle").fire("click"); log.restart = snap();
tiles()[3].fire("click");                  // 印象 = 4
$("next").fire("click");
log.q2 = snap();
tiles()[0].fire("click"); $("next").fire("click");       // 気づき = 動いていたから
tiles()[0].fire("click"); tiles()[1].fire("click");      // 近づく: 楽しい + 気にならない
log.q3_two = snap();
$("next").fire("click");
log.q4_unanswered = snap();
$("back").fire("click");                  // 答えなしでも押せる → 3問目へ
log.back_to_q3 = snap();
$("back").fire("click");
log.back_to_q2 = snap();
$("next").fire("click"); $("next").fire("click");   // q3 → q4
$("first").fire("click");                 // 4問目(未回答)から最初へ
log.first = snap();
// 最後まで: q1 → q5
$("next").fire("click");                  // q2(答え残り)
log.q2_again = snap();
$("next").fire("click"); $("next").fire("click");   // q3, q4
tiles()[1].fire("click"); $("next").fire("click");  // q4 = やや思った
tiles()[2].fire("click");                 // q5 = 高校・高専生
log.q5 = snap();
log.q5_next = $("next").textContent;
$("next").fire("click");                  // 終了
setImmediate(function () {
log.done = {ask: vis("ask"), done: vis("done"), idle: vis("idle")};
var answerCalls = calls.filter(function (c) { return c.url.indexOf("/display/api/survey/answer") === 0; });
log.answers = answerCalls.map(function (c) { return {url: c.url, body: c.body}; });
log.events = calls.filter(function (c) { return c.url.indexOf("/session") > 0; }).map(function (c) { return c.body.event; });
process.stdout.write(JSON.stringify(log));
});
"""


def _run_node(show_back=True):
    m = re.search(r'(  var KEY = "mirs_robot_survey";.*?)  /\* 隠し管理 \*/', HTML, re.S)
    assert m, "survey JS not found"
    s = json.loads(dsv_json(show_back))
    js = ("var S = %s;\n" % json.dumps(s, ensure_ascii=False)) + _STUB + m.group(1) + _DRIVER
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "t.js"
        p.write_text(js, encoding="utf-8")
        r = subprocess.run([NODE, str(p)], capture_output=True, text=True, timeout=30, encoding="utf-8")
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


def dsv_json(show_back):
    d = dict(dsv.SURVEY_DEF)
    d["showBack"] = show_back
    return json.dumps(d, ensure_ascii=False)


@unittest.skipUnless(NODE, "node がない")
class StateMachineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.log = _run_node(True)

    def test_first_question_shows_both_first_disabled(self):
        q1 = self.log["q1"]
        self.assertEqual(q1["qno"], "1 / 5")
        self.assertTrue(q1["back"])
        self.assertTrue(q1["first"])
        self.assertTrue(q1["firstDisabled"])
        self.assertFalse(self.log["q2"]["firstDisabled"])

    def test_back_on_first_question_returns_to_idle_without_submit(self):
        self.assertEqual(self.log["first_at_q1"]["qno"], "1 / 5")
        self.assertTrue(self.log["first_at_q1"]["ask"])
        b = self.log["back_at_q1"]
        self.assertTrue(b["idle"])
        self.assertFalse(b["ask"])
        self.assertTrue(self.log["restart"]["ask"])
        self.assertEqual(self.log["restart"]["sel"], [])        # 待機に戻ると答えは捨てられる

    def test_all_questions_show_both(self):
        for k in ("q1", "q2", "q3_two", "q4_unanswered", "q2_again", "q5"):
            self.assertTrue(self.log[k]["back"], k)
            self.assertTrue(self.log[k]["first"], k)

    def test_required_next_still_blocked_but_back_works_without_answer(self):
        self.assertTrue(self.log["q2"]["nextDisabled"])
        self.assertTrue(self.log["q4_unanswered"]["nextDisabled"])
        self.assertEqual(self.log["back_to_q3"]["qno"], "3 / 5")

    def test_back_keeps_answers_and_multi_select(self):
        self.assertEqual(self.log["q3_two"]["sel"], [0, 1])          # maxSelect=2
        self.assertEqual(self.log["back_to_q3"]["sel"], [0, 1])
        self.assertEqual(self.log["back_to_q2"]["qno"], "2 / 5")
        self.assertEqual(self.log["back_to_q2"]["sel"], [0])

    def test_first_goes_to_question_1_with_answer_kept(self):
        f = self.log["first"]
        self.assertEqual(f["qno"], "1 / 5")
        self.assertEqual(f["sel"], [3])
        self.assertTrue(f["back"])
        self.assertTrue(f["firstDisabled"])
        self.assertEqual(self.log["q2_again"]["sel"], [0])

    def test_submit_exactly_once_with_final_answers_and_valid(self):
        self.assertEqual(self.log["done"], {"ask": False, "done": True, "idle": False})
        self.assertEqual(len(self.log["answers"]), 1)
        rec = self.log["answers"][0]["body"]
        self.assertEqual(rec["answers"], {"impression": 4, "noticed_by": "動いていたから",
                                          "approach_feel": ["楽しい", "気にならない"],
                                          "want_to_move": "やや思った", "visitor_type": "高校・高専生"})
        self.assertEqual(self.log["events"].count("start"), 2)   # 途中で待機に戻って始め直した分
        self.assertEqual(self.log["events"].count("end"), 1)     # 1問目の「1つ前」で待機へ戻った通知
        # 検証に通る(送信内容を変えていない)
        saved, err = dsv.validate_response(rec)
        self.assertEqual(err, "")
        self.assertIsNotNone(saved)

    def test_show_back_false_hides_buttons(self):
        log = _run_node(False)
        for k in ("q1", "q2", "q3_two", "q5"):
            self.assertFalse(log[k]["back"], k)
            self.assertFalse(log[k]["first"], k)


if __name__ == "__main__":
    unittest.main()
