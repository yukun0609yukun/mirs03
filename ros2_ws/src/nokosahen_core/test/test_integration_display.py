"""iPad表示まわりの「統合テスト」(部品をつないで、通しで動くか)。ROS・ロボット・iPad・LiDAR・Nav2・モーターには一切触れない。

test_ipad_display.py は部品ごとの検証(単体寄り)。こちらは、実物の部品を**つないだ状態**で、利用者の操作の流れを最初から最後まで確かめる。
  - 本物の HTTP サーバ(`make_dashboard_handler`、127.0.0.1 の空きポート)
  - DisplayHub(モード・アンケートの回答セッション・残数入力・メニュー編集)と SurveyStore(実際にファイルへ保存)
  - InventoryManager の実物のハンドラ(`_on_menu_init` `_on_sold_total_event` `_make_state`。ROSなしで呼べる部分)
  - survey_monitor の fetch_survey_state + SurveyGate(実際に /display/api/state を HTTP で読む)
  - moving_notice.MovingNotice と announce_events の SituationBuilder / AnnounceManager
シナリオ(各シナリオは独立。サーバは終了時に必ず止める)
  (a) ログイン→表示         未ログイン401・失敗5回でロック・ログインでCookie・以降は再ログイン不要・サーバ再作成でセッション無効
  (b) モード切替             TL/operator の API、ROS相当(request_mode)、回答中は保留→回答後に切替、権限なし403
  (c) アンケート→ロボット停止 session start → SurveyGate が「止める」→ 回答送信 → saved → 解除。最長300秒+クールダウン・放置・応答なしは安全側
  (d) メニュー編集・残数入力  iPad と TL の menu/edit → menu_init → inventory_manager の menu_state → customer 表示・おすすめ
  (e) 移動中の注意との連動    MovingNotice → customer.status(caution)、出さない条件、探索中は音なし、announce_events との整合
  (f) 権限・認証の境界        iPad のセッションと管理画面のセッションは相互に使えない、/display の認証なしの範囲、Origin 違反
アンケートの設問は display_survey.SURVEY_DEF から組み立てる(設問の文言に依存しない)。
パスワードはダミー(実機のものはどこにも書かない)。
実行: python3 -I ros2_ws/src/nokosahen_core/test/test_integration_display.py   (全体で1分以内)
"""

import json
import re
import socket
import sys
import threading
import types
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))                 # python -I でもテスト補助(test_ipad_display)を読めるように
sys.path.insert(0, str(HERE.parent))

import test_ipad_display as tid  # noqa: E402  (ROSが無い環境用のスタブを入れる。HTTP・時計の補助を再利用する)
from nokosahen_core import accounts as acc  # noqa: E402
from nokosahen_core import announce_events as ae  # noqa: E402
from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import display_hub as dh  # noqa: E402
from nokosahen_core import display_page as dp  # noqa: E402
from nokosahen_core import display_survey as dsv  # noqa: E402
from nokosahen_core import ephemeral_overrides as eo  # noqa: E402
from nokosahen_core import moving_notice as mvn  # noqa: E402

try:
    import yaml  # noqa: F401
    HAVE_YAML = True
except ImportError:
    HAVE_YAML = False

TMP = tid.TMP
DISPLAY_PW = "dummy-display-pw-0000"        # ダミー。実機のパスワードではない
TL_PW = "dummy-tl-pass"
OP_PW = "operator-pass-123"
OP_HASH = acc.hash_password(OP_PW, n=2 ** 10)
OUTSIDE = "203.0.113.20"                   # LAN上のiPad(ループバック以外)として扱う接続元
OUTSIDE2 = "203.0.113.21"

PKG = HERE.parent
_counter = {"n": 0}


# ---------------------------------------------------------------------------
# アンケートの回答を SURVEY_DEF から組み立てる(設問の文言・数・選択肢の中身に依存しない)
# ---------------------------------------------------------------------------
def valid_answers(definition=None, pick=0):
    """SURVEY_DEF の全設問に、検証に通る回答を入れた dict。pick は何番目の選択肢を選ぶか。"""
    d = definition or dsv.SURVEY_DEF
    out = {}
    for q in d["questions"]:
        t = q["type"]
        if t == "choice":
            out[q["id"]] = q["options"][min(pick, len(q["options"]) - 1)]
        elif t == "scale":
            out[q["id"]] = q.get("max", 5)
        elif t == "multi":
            out[q["id"]] = [q["options"][min(pick, len(q["options"]) - 1)]]
        else:
            raise AssertionError(f"テストが未対応の設問型です: {t}")
    return out


def valid_response(rid=None, definition=None):
    d = definition or dsv.SURVEY_DEF
    _counter["n"] += 1
    return {"response_id": rid or f"resp-int-{_counter['n']:06d}", "submitted_at": "2026-10-30T01:02:03.000Z",
            "survey_version": d["version"], "mode": "robot", "condition": d["conditions"][0], "location": "食堂入口",
            "duration_sec": 12, "answers": valid_answers(d)}


def invalid_response_variants(definition=None):
    """検証に通らない回答のいろいろ(どれも保存されてはならない)。"""
    d = definition or dsv.SURVEY_DEF
    out = []
    r = valid_response(); r["survey_version"] = "zz-old"; out.append(("版が違う", r))
    r = valid_response(); r["answers"]["__unknown_q__"] = "x"; out.append(("未知の設問", r))
    q = next(q for q in d["questions"] if q["type"] == "choice")
    r = valid_response(); r["answers"][q["id"]] = "選択肢に無い文字列"; out.append(("選択肢外", r))
    r = valid_response(); r["condition"] = "一覧にない条件"; out.append(("条件外", r))
    r = valid_response(); r["response_id"] = "x"; out.append(("response_id不正", r))
    return out


# ---------------------------------------------------------------------------
# 実物の InventoryManager(ROSなし)
# ---------------------------------------------------------------------------
class _T:
    """rclpy の Time の代わり(引き算で Duration 相当=nanoseconds を持つものを返す)。"""

    def __init__(self, ns):
        self.nanoseconds = ns

    def __sub__(self, other):
        return _T(self.nanoseconds - other.nanoseconds)


def make_manager(clk, items):
    """ROSを起動せずに InventoryManager のハンドラ(_on_menu_init / _on_sold_total_event / _make_state)を呼べる状態にする。"""
    from nokosahen_core import inventory_manager as im
    m = object.__new__(im.InventoryManager)
    m._items = {}
    m._promoted_item = None
    m._last_state = {}
    m._consumption_log = None
    m.log_file = ""
    m._log_write_failures = 0
    m._log_write_failure_limit = 10
    m.promote_gap_count = 5
    m.promote_gap_ratio = 0.15
    m.promote_hysteresis_ratio = 0.5
    m.initial_phase_seconds = 120.0
    m.get_logger = lambda: types.SimpleNamespace(info=lambda *a, **k: None, warn=lambda *a, **k: None, error=lambda *a, **k: None)
    m.get_clock = lambda: types.SimpleNamespace(now=lambda: _T(int(clk() * 1e9)))
    m._started_at = m.get_clock().now()
    m._publish_state = lambda: None
    m._on_menu_init(types.SimpleNamespace(data=json.dumps({"items": items})))
    return m


class _Server(ThreadingHTTPServer):
    """接続元IPを差し替えられるサーバ(本物のハンドラに「LAN上のiPadから来た」と見せる。`peer` が None なら本当の接続元=127.0.0.1)。"""
    daemon_threads = True
    peer = None

    def get_request(self):
        conn, addr = super().get_request()
        if self.peer:
            addr = (self.peer,) + tuple(addr[1:])
        return conn, addr


class Rig:
    """つないだ一式。1シナリオ=1つ作り、終了時に必ずサーバを止める。"""

    def __init__(self, tc, name, perms=None, menu=True, abandon_s=dh.SESSION_ABANDON_S, max_answer_s=dh.SESSION_MAX_ANSWER_S):
        self.tc = tc
        self.name = name
        self.dir = TMP / ("int_" + name)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.clk = tid.Clock(1000.0)
        self.sent = []
        self.srv = None
        self._extra = []
        self.mgr = make_manager(self.clk, [
            {"name": "Aランチ", "initial": 100, "remaining": 60, "sold": 40},
            {"name": "Bランチ", "initial": 80, "remaining": 80, "sold": 0}]) if menu else None
        self.menu_view = None
        self.sync()
        # 画面の customer 部分(full_dashboard_node._display_customer と同じ組み立て)
        self.behavior = None
        self.beh_rx = None
        self.mres = None
        self.mres_t = -1e9
        self.announce_text = None
        self.hub = dh.DisplayHub(
            mode_file=str(self.dir / "mode"), survey_store=dsv.SurveyStore(str(self.dir / "survey")), clock=self.clk, wall=self.clk,
            menu_state_provider=(lambda: self.menu_view) if menu else None,
            inventory_sink=self._sink if menu else None,
            customer_provider=self._customer, abandon_s=abandon_s, max_answer_s=max_answer_s)
        # 移動中の注意・状況アナウンス
        self.mn = mvn.MovingNotice()
        self.sit = ae.SituationBuilder()
        cfg = ae.merged_config(None, False)
        cfg["enabled"] = True
        self.am = ae.AnnounceManager(cfg)
        self.speaking = False
        self.ops_log = []
        # アカウント(TL=管理者、kari-op=operator、guest)
        af = TMP / f"int_accounts_{name}.yaml"
        with open(af, "w", encoding="utf-8", newline="\n") as f:
            f.write('accounts:\n  - username: "kari-op"\n    role: operator\n'
                    f'    password_hash: "{OP_HASH}"\n    enabled: true\n'
                    f'\noperator_permissions: {json.dumps(perms or [])}\n')
        pf = TMP / f"int_perm_{name}.json"
        if pf.exists():
            pf.unlink()
        self.store = acc.AccountStore(accounts_path=str(af), permissions_path=str(pf))
        jf = TMP / f"int_journal_{name}.json"
        if jf.exists():
            jf.unlink()
        self.journal = jf
        self.start_server()
        tc.addCleanup(self.close)
        self._ipad = None

    # --- 部品 ---
    def _sink(self, topic, payload):
        """iPad/TL からの残数入力・メニュー編集を、InventoryManager の実物のハンドラへ流す(ROSトピックの代わり)。"""
        self.sent.append((topic, payload))
        msg = types.SimpleNamespace(data=json.dumps(payload))
        if topic == "/nokosahen/sold_total_event":
            self.mgr._on_sold_total_event(msg)
        elif topic == "/nokosahen/menu_init":
            self.mgr._on_menu_init(msg)
        else:
            return False
        return True

    def sync(self):
        """InventoryManager の状態(/nokosahen/menu_state)をダッシュボードが受け取った状態にする(これを呼ぶまでは古いまま=反映待ち)。"""
        if self.mgr is not None:
            self.menu_view = json.loads(json.dumps(self.mgr._make_state()))

    def _customer(self):
        now = self.clk()
        age = (now - self.beh_rx) if self.beh_rx else None
        moving = self.mres if (now - self.mres_t) <= 1.5 else None      # 判定が止まっていたら「移動中」を出さない(full_dashboard_nodeと同じ)
        return dh.build_customer_payload(self.menu_view, self.behavior, age, self.announce_text, moving)

    def start_server(self):
        handler = dc.make_dashboard_handler(
            lambda: b"<html>dash</html>", lambda: {}, tid.FakeTeleop(), survey_admin=dh.LocalDisplayAdmin(self.hub),
            auth_username="TL", auth_password=TL_PW, enforce_single_controller=False, account_store=self.store,
            ephemeral=eo.EphemeralOverrides(str(self.journal)), display_hub=self.hub,
            display_auth_username="ADMIN_ACCOUNT", display_auth_password=DISPLAY_PW)
        self.srv = _Server(("127.0.0.1", 0), handler)
        threading.Thread(target=self.srv.serve_forever, kwargs={"poll_interval": 0.02}, daemon=True).start()
        self.base = f"http://127.0.0.1:{self.srv.server_address[1]}"
        self._ipad = None
        return self.base

    def stop_server(self):
        if self.srv is not None:
            self.srv.shutdown()
            self.srv.server_close()
            self.srv = None

    def restart_server(self):
        """ダッシュボードのプロセスを再起動した状態(DisplayAuth も作り直し。セッションはメモリ上だけなので全部無効になる)。"""
        self.stop_server()
        return self.start_server()

    def close(self):
        self.stop_server()
        for s in self._extra:
            try:
                s.close()
            except OSError:
                pass

    # --- HTTP ---
    def req(self, method, path, body=None, cookie=None, peer=None, headers=None, raw=None, retry=2):
        """retry: 接続が切られたとき(Windowsで、サーバが本文を読まずに401を返して閉じると起きる既知の競合)に再試行する回数。副作用の無い(拒否されるはずの)要求にだけ使う。"""
        self.srv.peer = peer
        for i in range(retry + 1):
            try:
                return tid.jhttp(method, self.base + path, body, cookie=cookie, headers=headers, raw=raw)
            except ConnectionError:
                if i >= retry:
                    raise

    def rawreq(self, method, path, body=None, cookie=None, peer=None, headers=None):
        self.srv.peer = peer
        return tid.http(method, self.base + path, body, cookie=cookie, headers=headers)

    def login_display(self, peer=None, password=DISPLAY_PW):
        code, d, h = self.req("POST", "/display/api/login", {"username": "ADMIN_ACCOUNT", "password": password}, peer=peer)
        return code, d, (h.get("Set-Cookie", "") or "").split(";")[0]

    def ipad(self):
        """iPad のログイン済みCookie(1回だけログインして使い回す=実機と同じ)。"""
        if self._ipad is None:
            code, d, ck = self.login_display()
            self.tc.assertEqual(code, 200, d)
            self._ipad = ck
        return self._ipad

    def admin(self, user="TL", pw=TL_PW):
        if user == "kari-op" and not HAVE_YAML:
            self.tc.skipTest("PyYAML が無いため operator のアカウント表を読めない(この環境の制約。test_ipad_display の同種のテストも同様に失敗する)")
        code, d, h = self.req("POST", "/api/login", {"username": user, "password": pw})
        self.tc.assertEqual(code, 200, (user, d))
        return (h.get("Set-Cookie", "") or "").split(";")[0]

    def state(self, cookie=None, peer=None):
        """iPad が読む /display/api/state。cookie が None なら ipad() のCookie。"""
        code, st, _ = self.req("GET", "/display/api/state", cookie=self.ipad() if cookie is None else cookie, peer=peer)
        self.tc.assertEqual(code, 200, st)
        return st

    def set_mode_tl(self, mode, ck=None):
        return self.req("POST", "/api/survey/mode", {"mode": mode}, cookie=ck or self.admin())

    def session(self, event, **extra):
        return self.req("POST", "/display/api/survey/session", dict({"event": event}, **extra), cookie=self.ipad())

    def answer(self, payload):
        return self.req("POST", "/display/api/survey/answer", payload, cookie=self.ipad())

    # --- 移動中の注意・状況アナウンス(full_dashboard_node._moving_tick / announce_node と同じ配線) ---
    def feed(self, speed, mode, **beh):
        """車輪の速さ(オドメトリ)と director の指令を受け取る。"""
        now = self.clk()
        d = dict({"mode": mode}, **beh)
        self.behavior, self.beh_rx = d, now
        self.mn.on_odom(speed, now)
        self.mn.on_behavior(d, now)
        self.sit.on_behavior_command(d, now)

    def moving_tick(self):
        now = self.clk()
        answering = bool(self.hub.status_for_ros().get("active"))        # full_dashboard_node と同じ(アンケート回答中か)
        r = self.mn.evaluate(now, survey_answering=answering, speaking=self.speaking,
                             display_enabled=True, display_mode=self.hub.mode)     # 【2026-10-30】iPad表示のモードが customer のときだけ(画面と音をそろえる)
        self.mres, self.mres_t = r, now
        self.sit.on_moving_notice(dict({"stamp": now}, **r), now)
        return r

    def announce(self):
        now = self.clk()
        ops = self.am.update(self.sit.flags(now), self.sit.context(now), now)
        self.ops_log.extend(ops)
        return ops

    def step(self, dt, speed, mode, odom=True, behavior=True, tick=True, **beh):
        self.clk.advance(dt)
        if odom or behavior:
            now = self.clk()
            d = dict({"mode": mode}, **beh)
            if behavior:
                self.behavior, self.beh_rx = d, now
                self.mn.on_behavior(d, now)
                self.sit.on_behavior_command(d, now)
            if odom:
                self.mn.on_odom(speed, now)
        r = self.moving_tick() if tick else self.mres
        return r, self.announce()

    def run(self, seconds, speed, mode, dt=0.25, **kw):
        r, ops = None, []
        for _ in range(int(round(seconds / dt))):
            r, o = self.step(dt, speed, mode, **kw)
            ops.extend(o)
        return r, ops


def loop_ops(ops, eid="moving_notice"):
    return [o for o in ops if o.get("id") == eid and o["op"] == "loop"]


def stop_ops(ops, eid="moving_notice"):
    return [o for o in ops if o.get("id") == eid and o["op"] == "loop_stop"]


# ---------------------------------------------------------------------------
# (a) ログイン → 表示
# ---------------------------------------------------------------------------
class LoginToDisplayFlowTest(unittest.TestCase):
    def test_not_logged_in_is_401_from_outside_but_the_login_page_is_shown(self):
        rig = Rig(self, "a1")
        code, d, _ = rig.req("GET", "/display/api/state", peer=OUTSIDE)
        self.assertEqual((code, d.get("error")), (401, "auth_required"))
        code, body, _ = rig.rawreq("GET", "/display", peer=OUTSIDE)
        self.assertEqual((code, body), (200, dp.build_display_login_html()))        # ログイン画面だけ(状態・設問を含まない)
        self.assertNotIn("Aランチ", body.decode("utf-8"))
        # 同じPCの中(survey_monitor)からは認証なしで読める
        self.assertEqual(rig.req("GET", "/display/api/state")[0], 200)

    def test_five_failures_lock_that_device_only_and_the_right_password_is_then_refused(self):
        rig = Rig(self, "a2")
        for i in range(5):
            code, d, ck = rig.login_display(peer=OUTSIDE, password="wrong-" + str(i))
            self.assertEqual((code, d.get("error"), ck), (401, "invalid_credentials", ""))
        code, d, h = rig.req("POST", "/display/api/login", {"username": "ADMIN_ACCOUNT", "password": DISPLAY_PW}, peer=OUTSIDE)
        self.assertEqual((code, d.get("error")), (429, "too_many_attempts"))
        self.assertGreater(d["retry_after_sec"], 0)
        self.assertIsNone(h.get("Set-Cookie"))
        self.assertEqual(rig.req("GET", "/display/api/state", peer=OUTSIDE)[0], 401)       # ロック中は入れない
        code, _, ck = rig.login_display(peer=OUTSIDE2)                                     # 別の端末は影響を受けない
        self.assertEqual(code, 200)
        self.assertEqual(rig.req("GET", "/display/api/state", cookie=ck, peer=OUTSIDE2)[0], 200)

    def test_login_once_then_every_request_works_without_logging_in_again(self):
        rig = Rig(self, "a3")
        code, d, h = rig.req("POST", "/display/api/login", {"username": "ADMIN_ACCOUNT", "password": DISPLAY_PW}, peer=OUTSIDE)
        self.assertEqual((code, d), (200, {"ok": True}))
        sc = h["Set-Cookie"]
        self.assertTrue(sc.startswith("nokosahen_display="))
        for attr in ("HttpOnly", "SameSite=Lax", "Path=/display"):
            self.assertIn(attr, sc)
        self.assertNotIn(DISPLAY_PW, sc + json.dumps(d))
        ck = sc.split(";")[0]
        # 本体の画面(ログイン画面ではない)と状態。時間が経っても(放置しても)再ログインは要らない
        for hours in (0, 1, 12, 24 * 7):
            rig.clk.advance(hours * 3600)
            code, body, _ = rig.rawreq("GET", "/display", cookie=ck, peer=OUTSIDE)
            self.assertEqual((code, body), (200, dp.build_display_html()))
            st = rig.req("GET", "/display/api/state", cookie=ck, peer=OUTSIDE)[1]
            self.assertEqual(st["mode"], "customer")
            self.assertEqual({i["name"] for i in st["customer"]["menu"]["items"]}, {"Aランチ", "Bランチ"})
        # ROSやTLからのモード切替は、ログインとは無関係にその iPad の表示を変える
        rig.hub.request_mode("inventory", "ros")
        self.assertEqual(rig.req("GET", "/display/api/state", cookie=ck, peer=OUTSIDE)[1]["mode"], "inventory")

    def test_restarting_the_server_invalidates_the_session_and_login_works_again(self):
        rig = Rig(self, "a4")
        _, _, ck = rig.login_display(peer=OUTSIDE)
        self.assertEqual(rig.req("GET", "/display/api/state", cookie=ck, peer=OUTSIDE)[0], 200)
        rig.restart_server()                                    # DisplayAuth が作り直される(ロボット/ダッシュボードの再起動)
        self.assertEqual(rig.req("GET", "/display/api/state", cookie=ck, peer=OUTSIDE)[0], 401)
        code, body, _ = rig.rawreq("GET", "/display", cookie=ck, peer=OUTSIDE)
        self.assertEqual(body, dp.build_display_login_html())  # 古いCookieではログイン画面に戻る
        self.assertEqual(rig.req("POST", "/display/api/survey/session", {"event": "start"}, cookie=ck, peer=OUTSIDE)[0], 401)
        code, _, ck2 = rig.login_display(peer=OUTSIDE)
        self.assertEqual(code, 200)
        self.assertEqual(rig.req("GET", "/display/api/state", cookie=ck2, peer=OUTSIDE)[0], 200)
        self.assertEqual(rig.req("GET", "/display/api/state", cookie=ck, peer=OUTSIDE)[0], 401)      # 古い方は戻らない


# ---------------------------------------------------------------------------
# (b) モード切替
# ---------------------------------------------------------------------------
class ModeSwitchFlowTest(unittest.TestCase):
    CYCLE = ("survey", "inventory", "ocrtest", "customer")

    def test_tl_api_cycles_all_modes_and_ipad_follows_and_it_is_persisted(self):
        rig = Rig(self, "b1")
        tl = rig.admin()
        ipad = rig.ipad()
        rev = rig.state(ipad)["rev"]
        for m in self.CYCLE:
            code, d, _ = rig.set_mode_tl(m, tl)
            self.assertEqual((code, d["ok"]), (200, True), (m, d))
            st = rig.state(ipad, peer=OUTSIDE)                       # iPad(LAN)から見える
            self.assertEqual(st["mode"], m)
            self.assertGreater(st["rev"], rev)
            rev = st["rev"]
            self.assertEqual(rig.req("GET", "/api/survey/state", cookie=tl)[1]["state"]["mode"], m)    # 管理画面にも同じモード
            self.assertEqual((rig.dir / "mode").read_text(encoding="utf-8").strip(), m)               # 再起動後も引き継ぐ
            if m in ("customer", "inventory"):
                self.assertIn("customer", st)
            else:
                self.assertNotIn("customer", st)
        # 再起動(DisplayHub を作り直し)しても最後のモードに戻る
        h2 = dh.DisplayHub(mode_file=str(rig.dir / "mode"), survey_store=dsv.SurveyStore(str(rig.dir / "survey")), clock=rig.clk, wall=rig.clk)
        self.assertEqual(h2.mode, "customer")
        rig.set_mode_tl("ocrtest", tl)
        h3 = dh.DisplayHub(mode_file=str(rig.dir / "mode"), survey_store=dsv.SurveyStore(str(rig.dir / "survey")), clock=rig.clk, wall=rig.clk)
        self.assertEqual(h3.mode, "ocrtest")

    def test_operator_with_the_capability_can_switch_and_without_it_cannot(self):
        rig = Rig(self, "b2", perms=["display.survey_mode"])
        op = rig.admin("kari-op", OP_PW)
        for m in self.CYCLE:
            self.assertEqual(rig.set_mode_tl(m, op)[0], 200)
            self.assertEqual(rig.state()["mode"], m)
        rig2 = Rig(self, "b2b", perms=[])
        op2 = rig2.admin("kari-op", OP_PW)
        code, d, _ = rig2.set_mode_tl("survey", op2)
        self.assertEqual(code, 403)
        self.assertEqual(rig2.state()["mode"], "customer")           # 変わっていない

    def test_guest_and_not_logged_in_and_ipad_session_cannot_switch(self):
        rig = Rig(self, "b3")
        guest = rig.admin("guest", "")
        self.assertEqual(rig.set_mode_tl("survey", guest)[0], 403)
        self.assertEqual(rig.req("POST", "/api/survey/mode", {"mode": "survey"})[0], 401)
        self.assertEqual(rig.req("POST", "/api/survey/mode", {"mode": "survey"}, cookie=rig.ipad())[0], 401)     # iPad用セッションでは管理APIに入れない
        self.assertEqual(rig.state()["mode"], "customer")
        # 形が不正なものは400(TL)
        tl = rig.admin()
        self.assertEqual(rig.req("POST", "/api/survey/mode", {"mode": "kiosk"}, cookie=tl)[0], 400)
        self.assertEqual(rig.req("POST", "/api/survey/mode", {"mode": "survey", "x": 1}, cookie=tl)[0], 400)

    def test_ros_topic_equivalent_cycles_modes_in_string_and_json_forms(self):
        rig = Rig(self, "b4")
        forms = (("survey", "survey"), ('{"mode":"inventory"}', "inventory"), ("ocrtest", "ocrtest"), ('{"mode": "customer"}', "customer"))
        for payload, want in forms:
            r = rig.hub.request_mode(payload, "ros", defer=True)       # _on_display_mode が呼ぶのと同じ
            self.assertTrue(r["ok"], r)
            self.assertEqual(rig.state(peer=OUTSIDE)["mode"], want)
        before = rig.state()["rev"]
        for bad in ("kiosk", "", None, '{"mode":5}', "{"):
            self.assertFalse(rig.hub.request_mode(bad, "ros", defer=True)["ok"], bad)
        self.assertEqual((rig.state()["mode"], rig.state()["rev"]), ("customer", before))      # 不正な要求は無視

    def test_switch_is_deferred_while_answering_then_applied_after_the_answer(self):
        rig = Rig(self, "b5")
        tl = rig.admin()
        rig.set_mode_tl("survey", tl)
        self.assertTrue(rig.session("start")[1]["active"])
        # ROS(時間割など)からの要求は保留
        r = rig.hub.request_mode("customer", "ros", defer=True)
        self.assertEqual((r["ok"], r["deferred"], r["changed"]), (True, True, False))
        st = rig.state()
        self.assertEqual((st["mode"], st["pending_mode"], st["session_state"], st["active"]), ("survey", "customer", "answering", True))
        # TLの手動切替は断る(その人の回答を消さない)
        code, d, _ = rig.set_mode_tl("inventory", tl)
        self.assertEqual(code, 200)
        self.assertFalse(d["ok"])
        self.assertIn("回答中", d["message"])
        self.assertEqual(rig.state()["mode"], "survey")
        # 回答が保存されても、保存の表示中(SAVED_SHOW_S)はまだ切り替えない
        self.assertEqual(rig.answer(valid_response())[0], 200)
        st = rig.state()
        self.assertEqual((st["mode"], st["session_state"], st["pending_mode"]), ("survey", "saved", "customer"))
        rig.clk.advance(dh.SAVED_SHOW_S + 0.1)
        st = rig.state()
        self.assertEqual((st["mode"], st["pending_mode"], st["session_state"], st["active"]), ("customer", None, "idle", False))
        self.assertEqual((rig.dir / "mode").read_text(encoding="utf-8").strip(), "customer")

    def test_deferred_switch_is_applied_when_the_respondent_walks_away(self):
        rig = Rig(self, "b6")
        rig.set_mode_tl("survey")
        rig.session("start")
        rig.hub.request_mode("inventory", "ros", defer=True)
        rig.clk.advance(dh.SESSION_ABANDON_S - 1)
        self.assertEqual(rig.state()["mode"], "survey")             # まだ放棄ではない
        rig.clk.advance(2)
        st = rig.state()
        self.assertEqual((st["mode"], st["pending_mode"]), ("inventory", None))

    def test_leaving_the_survey_mode_clears_the_answering_state(self):
        rig = Rig(self, "b7")
        rig.set_mode_tl("survey")
        rig.session("end")                                          # 回答していないとき(idle)は普通に切り替わる
        self.assertEqual(rig.set_mode_tl("ocrtest")[1]["ok"], True)
        st = rig.state()
        self.assertEqual((st["mode"], st["active"]), ("ocrtest", False))
        # survey でないモードでは回答中の通知を受け付けない(ロボットを止める入口にならない)
        code, d, _ = rig.session("start")
        self.assertEqual(code, 409)
        self.assertFalse(rig.state()["active"])


# ---------------------------------------------------------------------------
# (c) アンケート回答 → ロボット停止(survey_monitor の SurveyGate)
# ---------------------------------------------------------------------------
class SurveyStopLinkFlowTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            from nokosahen_core import survey_monitor as sm
        except ImportError as exc:     # 全件実行で、先に別のテストが不完全なrclpyスタブを入れたとき(既知の干渉)
            raise unittest.SkipTest(f"survey_monitorを読み込めない(rclpyスタブの干渉): {exc}")
        cls.sm = sm

    def setUp(self):
        self.rig = Rig(self, "c_" + self.id().split(".")[-1][:40])
        # ノード(survey_monitor)の既定値と同じ
        self.gate = self.sm.SurveyGate(debounce_sec=1.5, unreachable_hold_sec=20.0, answering_unreachable_hold_sec=60.0, post_save_hold_sec=1.5)

    def poll(self, dt=0.0, timeout=2.0):
        """時間を進め、survey_monitor と同じく /display/api/state(認証なし・ループバック)を HTTP で読んで、ロボットを止めるか(survey_active)を返す。"""
        self.rig.clk.advance(dt)
        s = self.sm.fetch_survey_state(self.rig.base + "/display/api", timeout)
        self.gate.feed(s)
        return self.gate.tick(self.rig.clk()), s

    def test_full_flow_start_answering_submit_saved_release(self):
        rig = self.rig
        rig.set_mode_tl("survey")
        stop, s = self.poll()
        self.assertEqual((stop, s["ok"], s["mode"], s["session_state"]), (False, True, "survey", "idle"))      # 誰も触っていない
        self.assertTrue(rig.session("start", condition=dsv.SURVEY_DEF["conditions"][0])[1]["active"])
        stop, s = self.poll(0.5)
        self.assertEqual((stop, s["session_state"], s["active"]), (True, "answering", True))                  # 回答中 → ロボットを止める
        for _ in range(20):                                                                                  # 100秒かけて答えても止め続ける(5秒おきの通知)
            rig.clk.advance(5)
            self.assertTrue(rig.session("beat")[1]["active"])
            self.assertTrue(self.poll()[0])
        seq0 = s["saved_seq"]
        code, d, _ = rig.answer(valid_response())                                                            # 検証に通る回答(SURVEY_DEFから組み立て)
        self.assertEqual((code, d), (200, {"ok": True}))
        stop, s = self.poll(0.1)
        self.assertEqual((s["session_state"], s["saved_seq"], s["active"]), ("saved", seq0 + 1, False))
        self.assertTrue(stop)                                                                                # 保存直後はすぐ動かさない
        self.assertTrue(self.poll(0.5)[0])
        self.assertFalse(self.poll(1.5)[0])                                                                  # 一定時間後に解除
        self.assertFalse(self.poll(3.0)[0])                                                                  # saved の表示が終わって idle になっても止めない
        # 回答は実際にファイルへ保存され、TLのCSVに出る
        self.assertEqual(rig.hub.store.count(), 1)
        code, body, _ = rig.rawreq("GET", "/api/survey/responses.csv", cookie=rig.admin())
        self.assertEqual(code, 200)
        self.assertIn(b"resp-int-", body)

    def test_invalid_answer_is_rejected_and_the_robot_keeps_waiting(self):
        rig = self.rig
        rig.set_mode_tl("survey")
        rig.session("start")
        self.assertTrue(self.poll(0.5)[0])
        for why, bad in invalid_response_variants():
            code, d, _ = rig.answer(bad)
            self.assertEqual(code, 400, why)
            self.assertFalse(d["ok"], why)
        stop, s = self.poll(0.5)
        self.assertEqual((stop, s["session_state"], s["count"]), (True, "answering", 0))                      # 保存されない・回答中のまま
        self.assertEqual(rig.hub.store.count(), 0)
        # 同じ response_id の再送は二重に保存されない
        r = valid_response()
        self.assertEqual(rig.answer(r)[1], {"ok": True})
        self.assertEqual(rig.answer(r)[1], {"ok": True, "duplicate": True})
        self.assertEqual(rig.hub.store.count(), 1)

    def test_longest_answering_is_300_seconds_then_cooldown_then_new_session_ok(self):
        rig = self.rig
        rig.set_mode_tl("survey")
        rig.session("start")
        t0 = rig.clk()
        stop = True
        while rig.clk() - t0 < dh.SESSION_MAX_ANSWER_S - 5:            # 通知を送り続けても(放棄にならなくても)最長で打ち切る
            rig.clk.advance(5)
            rig.session("beat")
            stop = self.poll()[0]
            self.assertTrue(stop)
        rig.clk.advance(10)                                             # 300秒を越えた
        rig.session("beat")                                             # この通知では復活しない
        stop, s = self.poll()
        self.assertEqual(s["session_state"], "abandoned")
        self.assertFalse(s["active"])
        self.assertTrue(stop)                                           # デバウンスの間は保つ
        self.assertFalse(self.poll(2.0)[0])                             # 解除
        self.assertEqual(rig.session("start")[1]["active"], False)      # クールダウン中: 新しい回答中の通知は受け付けない
        self.assertFalse(self.poll(1.0)[0])
        rig.clk.advance(dh.MAX_ANSWER_COOLDOWN_S + 1)
        self.assertTrue(rig.session("start")[1]["active"])              # クールダウンが明けたらまた回答できる
        self.assertTrue(self.poll(0.5)[0])

    def test_walking_away_releases_after_the_abandon_timeout(self):
        rig = self.rig
        rig.set_mode_tl("survey")
        rig.session("start")
        self.assertTrue(self.poll(0.5)[0])
        self.assertTrue(self.poll(dh.SESSION_ABANDON_S - 5)[0])         # 通知が途絶えてもしばらくは止めておく
        stop, s = self.poll(6)                                          # 放棄とみなす
        self.assertEqual(s["session_state"], "abandoned")
        self.assertFalse(self.poll(2.0)[0])                             # 待機に復帰(ロボットが動ける)
        # 回答画面の「終了」(end)も同様に解除される
        rig.clk.advance(dh.ABANDONED_SHOW_S + 1)
        rig.session("start")
        self.assertTrue(self.poll(0.5)[0])
        rig.session("end")
        self.assertTrue(self.poll(0.5)[0])                              # 解除の確認(デバウンス1.5秒)は、falseを見てから数える
        self.assertFalse(self.poll(2.0)[0])

    def test_unreachable_server_is_safe_side_then_releases(self):
        rig = self.rig
        rig.set_mode_tl("survey")
        rig.session("start")
        self.assertTrue(self.poll(0.5)[0])
        rig.stop_server()                                               # ダッシュボードが落ちた(接続拒否)
        stop, s = self.poll(5)                                          # 実際の接続拒否(Windowsは1回2秒ほどかかるので、以降は同じ失敗の応答を渡す)
        self.assertEqual(s["ok"], False)
        self.assertTrue(stop)                                           # 回答中だったので安全側(止める)
        failed = {"ok": False, "error": s["error"]}
        for dt, want in ((50, True), (10, False)):                      # 約55秒後もまだ止める(answering の保持は60秒)/ 永久には止めない
            rig.clk.advance(dt)
            self.gate.feed(failed)
            self.assertEqual(self.gate.tick(rig.clk()), want, dt)

    def test_unresponsive_server_times_out_into_safe_side_then_releases(self):
        rig = self.rig
        rig.set_mode_tl("survey")
        rig.session("start")
        self.assertTrue(self.poll(0.5)[0])
        # 受け付けるが応答しないサーバ(ハング)に差し替える: 接続はできるが読み出しがタイムアウトする
        dead = socket.socket()
        dead.bind(("127.0.0.1", 0))
        dead.listen(5)
        rig._extra.append(dead)
        url = f"http://127.0.0.1:{dead.getsockname()[1]}/display/api"
        gate = self.gate

        def poll_dead(dt):
            rig.clk.advance(dt)
            s = self.sm.fetch_survey_state(url, 0.4)
            gate.feed(s)
            return gate.tick(rig.clk()), s
        stop, s = poll_dead(5)
        self.assertEqual(s["ok"], False)
        self.assertIn("timed out", s["error"].lower())
        self.assertTrue(stop)
        self.assertTrue(poll_dead(40)[0])
        self.assertFalse(poll_dead(20)[0])

    def test_customer_mode_never_holds_even_if_someone_posts_a_session(self):
        rig = self.rig
        self.assertEqual(rig.state()["mode"], "customer")
        code, d, _ = rig.session("start")
        self.assertEqual(code, 409)                                     # アンケート表示中以外は受け付けない
        stop, s = self.poll(0.5)
        self.assertEqual((stop, s["mode"], s["active"]), (False, "customer", False))
        # アンケートで回答中に customer へ戻す要求は保留され、戻るまでは止める(回答が終わると解除)
        rig.set_mode_tl("survey")
        rig.session("start")
        rig.hub.request_mode("customer", "schedule", defer=True)
        self.assertTrue(self.poll(0.5)[0])
        rig.answer(valid_response())
        self.assertTrue(self.poll(0.5)[0])
        rig.clk.advance(dh.SAVED_SHOW_S + 0.2)
        stop, s = self.poll(0.1)
        self.assertEqual((s["mode"], s["active"]), ("customer", False))
        self.assertFalse(self.poll(2.0)[0])

    def test_state_contract_read_by_survey_monitor_is_complete(self):
        rig = self.rig
        rig.set_mode_tl("survey")
        stop, s = self.poll()
        for k in ("ok", "active", "mode", "count", "age", "condition", "session_state", "saved_seq"):
            self.assertIn(k, s)
            self.assertIsNotNone(s[k] if k not in ("age", "condition") else 0, k)
        self.assertIsInstance(s["saved_seq"], int)


# ---------------------------------------------------------------------------
# (d) メニュー編集・残数入力 → 残数管理(inventory_manager の実物)
# ---------------------------------------------------------------------------
def names(st):
    return [i["name"] for i in st["customer"]["menu"]["items"]]


def by_name(st, n):
    return next(i for i in st["customer"]["menu"]["items"] if i["name"] == n)


class MenuInventoryFlowTest(unittest.TestCase):
    EDIT_IPAD = "/display/api/menu/edit"
    EDIT_TL = "/api/menu/edit"

    def edit_ipad(self, rig, body):
        return rig.req("POST", self.EDIT_IPAD, body, cookie=rig.ipad())

    def edit_tl(self, rig, body, ck=None):
        return rig.req("POST", self.EDIT_TL, body, cookie=ck or rig.admin())

    def test_ipad_and_tl_edits_flow_through_menu_init_into_inventory_manager(self):
        rig = Rig(self, "d1")
        # iPad: 追加。menu_init の形は全品の全置き換え({name, initial, remaining, sold})
        code, d, _ = self.edit_ipad(rig, {"op": "add", "name": "Cランチ", "initial": 30})
        self.assertEqual((code, d["ok"], d["items"]), (200, True, ["Aランチ", "Bランチ", "Cランチ"]))
        topic, payload = rig.sent[-1]
        self.assertEqual(topic, "/nokosahen/menu_init")
        self.assertEqual(payload["items"], [{"name": "Aランチ", "initial": 100, "remaining": 60, "sold": 40},
                                            {"name": "Bランチ", "initial": 80, "remaining": 80, "sold": 0},
                                            {"name": "Cランチ", "initial": 30, "remaining": 30, "sold": 0}])
        self.assertEqual(list(rig.mgr._items), ["Aランチ", "Bランチ", "Cランチ"])      # 実物のハンドラが受け取った
        rig.sync()
        st = rig.state()
        self.assertEqual(names(st), ["Aランチ", "Bランチ", "Cランチ"])
        self.assertEqual((by_name(st, "Cランチ")["remaining"], by_name(st, "Cランチ")["sold"]), (30, 0))
        self.assertEqual((by_name(st, "Aランチ")["remaining"], by_name(st, "Aランチ")["sold"]), (60, 40))     # 売れた数の引き継ぎ
        # TL: 名前の変更(数は引き継ぐ)
        code, d, _ = self.edit_tl(rig, {"op": "rename", "name": "Aランチ", "new_name": "Aセット"})
        self.assertEqual((code, d["ok"]), (200, True), d)
        rig.sync()
        st = rig.state()
        self.assertEqual(names(st), ["Aセット", "Bランチ", "Cランチ"])
        self.assertEqual((by_name(st, "Aセット")["initial"], by_name(st, "Aセット")["remaining"], by_name(st, "Aセット")["sold"]), (100, 60, 40))
        # iPad: 削除(消えたメニューが残らない)
        self.assertEqual(self.edit_ipad(rig, {"op": "delete", "name": "Bランチ"})[0], 200)
        rig.sync()
        self.assertEqual(names(rig.state()), ["Aセット", "Cランチ"])
        self.assertNotIn("Bランチ", rig.mgr._items)
        # TL: 初期食数(売れた数は保つ)
        self.assertEqual(self.edit_tl(rig, {"op": "set_initial", "name": "Aセット", "initial": 120})[0], 200)
        rig.sync()
        a = by_name(rig.state(), "Aセット")
        self.assertEqual((a["initial"], a["remaining"], a["sold"]), (120, 80, 40))
        # 売れた数より少ない初期数: 残0、初期数は売れた数に切り上がる
        rig.clk.advance(dh.MENU_SYNC_WAIT_S + 1)
        self.assertEqual(self.edit_ipad(rig, {"op": "set_initial", "name": "Aセット", "initial": 10})[0], 200)
        rig.sync()
        a = by_name(rig.state(), "Aセット")
        self.assertEqual((a["initial"], a["remaining"], a["sold"]), (40, 0, 40))
        # TLの管理画面が見るメニュー一覧も同じ
        d = rig.req("GET", "/api/menu/state", cookie=rig.admin())[1]
        self.assertEqual([i["name"] for i in d["items"]], ["Aセット", "Cランチ"])

    def test_twelve_item_limit_and_input_validation_leave_the_manager_untouched(self):
        rig = Rig(self, "d2")
        for i in range(10):                                            # すでに2品: 12品まで
            rig.clk.advance(dh.MENU_SYNC_WAIT_S + 1)
            code, d, _ = self.edit_ipad(rig, {"op": "add", "name": f"品{i}", "initial": 5})
            self.assertEqual(code, 200, (i, d))
            rig.sync()
        self.assertEqual(len(rig.mgr._items), 12)
        sent = len(rig.sent)
        rig.clk.advance(dh.MENU_SYNC_WAIT_S + 1)
        code, d, _ = self.edit_ipad(rig, {"op": "add", "name": "13品目", "initial": 5})
        self.assertEqual(code, 400)
        self.assertFalse(d["ok"])
        self.assertEqual(len(rig.mgr._items), 12)
        self.assertEqual(len(rig.state()["customer"]["menu"]["items"]), 12)
        bad = ({"op": "add", "name": "Aランチ", "initial": 5}, {"op": "delete", "name": "ない"}, {"op": "add", "name": "N", "initial": True},
               {"op": "add", "name": "N" * 41, "initial": 1}, {"op": "set_initial", "name": "Aランチ", "initial": -1}, {"op": "zzz", "name": "A"})
        for b in bad:
            self.assertEqual(self.edit_ipad(rig, b)[0], 400, b)
            self.assertEqual(self.edit_tl(rig, b)[0], 400, b)
        self.assertEqual(len(rig.sent), sent)                           # 何も送られていない

    def test_back_to_back_edits_return_409_until_the_first_is_reflected_and_nothing_is_lost(self):
        rig = Rig(self, "d3")
        self.assertEqual(self.edit_ipad(rig, {"op": "add", "name": "X", "initial": 10})[0], 200)
        # menu_state にまだ反映されていない(sync前)ので、2つ目は断る(古い一覧で全置き換えして X を消さないため)
        code, d, _ = self.edit_tl(rig, {"op": "add", "name": "Y", "initial": 10})
        self.assertEqual(code, 409)
        self.assertIn("反映", d["message"])
        self.assertEqual(list(rig.mgr._items), ["Aランチ", "Bランチ", "X"])      # X は残っている・Y は入っていない
        rig.sync()                                                       # 反映されたら続けて編集できる
        self.assertEqual(self.edit_tl(rig, {"op": "add", "name": "Y", "initial": 10})[0], 200)
        rig.sync()
        self.assertEqual(list(rig.mgr._items), ["Aランチ", "Bランチ", "X", "Y"])
        self.assertEqual(names(rig.state()), ["Aランチ", "Bランチ", "X", "Y"])
        # 反映が来ないまま待ち時間が過ぎたら、また編集できる(永久に詰まらない)
        self.assertEqual(self.edit_ipad(rig, {"op": "delete", "name": "X"})[0], 200)
        self.assertEqual(self.edit_ipad(rig, {"op": "delete", "name": "Y"})[0], 409)
        rig.clk.advance(dh.MENU_SYNC_WAIT_S + 0.5)
        rig.sync()
        self.assertEqual(self.edit_ipad(rig, {"op": "delete", "name": "Y"})[0], 200)

    def test_inventory_input_updates_the_customer_screen_and_keeps_other_menus(self):
        rig = Rig(self, "d4")
        ipad = rig.ipad()
        code, d, _ = rig.req("POST", "/display/api/inventory", {"menu": "Aランチ", "remaining": 30}, cookie=ipad)
        self.assertEqual((code, d["ok"], d["kind"]), (200, True, "sold_total"))
        self.assertEqual(rig.sent[-1][0], "/nokosahen/sold_total_event")
        a = rig.mgr._items["Aランチ"]
        self.assertEqual((a.initial, a.remaining, a.sold), (100, 30, 70))
        self.assertEqual(rig.mgr._items["Bランチ"].remaining, 80)
        rig.sync()
        st = rig.state()
        self.assertEqual((by_name(st, "Aランチ")["remaining"], by_name(st, "Aランチ")["ratio"]), (30, 0.3))
        self.assertEqual(by_name(st, "Bランチ")["remaining"], 80)
        # 同じ値を送り直しても二重に計上されない
        rig.req("POST", "/display/api/inventory", {"menu": "Aランチ", "remaining": 30}, cookie=ipad)
        self.assertEqual(rig.mgr._items["Aランチ"].sold, 70)
        # 初期数より多い残数: 初期数を増やす(menu_init経由)・売れた数は保つ
        code, d, _ = rig.req("POST", "/display/api/inventory", {"menu": "Bランチ", "remaining": 95}, cookie=ipad)
        self.assertEqual((code, d["kind"], d["raised_initial"]), (200, "menu_init", True))
        b = rig.mgr._items["Bランチ"]
        self.assertEqual((b.remaining, b.sold, b.initial), (95, 0, 95))
        self.assertEqual(rig.mgr._items["Aランチ"].remaining, 30)        # 他は変わらない
        # 不正な入力は400で、何も変わらない
        n = len(rig.sent)
        for body in ({"menu": "ない", "remaining": 1}, {"menu": "Aランチ", "remaining": -1}, {"menu": "Aランチ", "remaining": "3"},
                     {"menu": "Aランチ", "remaining": 1.5}, {"menu": "Aランチ", "remaining": 10 ** 6}, {"menu": "Aランチ"}):
            self.assertEqual(rig.req("POST", "/display/api/inventory", body, cookie=ipad)[0], 400, body)
        self.assertEqual(len(rig.sent), n)

    def test_old_name_is_unknown_after_rename_and_the_new_name_takes_input(self):
        rig = Rig(self, "d5")
        self.assertEqual(self.edit_ipad(rig, {"op": "rename", "name": "Aランチ", "new_name": "Aセット"})[0], 200)
        rig.sync()
        ipad = rig.ipad()
        self.assertEqual(rig.req("POST", "/display/api/inventory", {"menu": "Aランチ", "remaining": 10}, cookie=ipad)[0], 400)
        self.assertEqual(rig.req("POST", "/display/api/inventory", {"menu": "Aセット", "remaining": 10}, cookie=ipad)[0], 200)
        rig.sync()
        a = by_name(rig.state(), "Aセット")
        self.assertEqual((a["remaining"], a["sold"]), (10, 90))

    def test_recommendation_follows_the_remaining_counts_after_the_initial_phase(self):
        rig = Rig(self, "d6")
        ipad = rig.ipad()
        rig.req("POST", "/display/api/inventory", {"menu": "Aランチ", "remaining": 90}, cookie=ipad)
        rig.req("POST", "/display/api/inventory", {"menu": "Bランチ", "remaining": 20}, cookie=ipad)
        rig.sync()
        self.assertIsNone(rig.state()["customer"]["menu"]["promote_item"])           # 立ち上がりの120秒間は「おすすめ」を出さない
        rig.clk.advance(rig.mgr.initial_phase_seconds + 1)
        rig.sync()
        self.assertEqual(rig.state()["customer"]["menu"]["promote_item"], "Aランチ")  # 多く残っている方
        # 逆転: B が多くなればおすすめは B
        rig.req("POST", "/display/api/inventory", {"menu": "Bランチ", "remaining": 80}, cookie=ipad)
        rig.req("POST", "/display/api/inventory", {"menu": "Aランチ", "remaining": 10}, cookie=ipad)
        rig.sync()
        self.assertEqual(rig.state()["customer"]["menu"]["promote_item"], "Bランチ")
        # おすすめのメニューを削除したら、おすすめは(1品だけになるので)消える
        rig.clk.advance(dh.MENU_SYNC_WAIT_S + 1)
        self.assertEqual(self.edit_ipad(rig, {"op": "delete", "name": "Bランチ"})[0], 200)
        rig.sync()
        st = rig.state()
        self.assertEqual(names(st), ["Aランチ"])
        self.assertIsNone(st["customer"]["menu"]["promote_item"])

    def test_inventory_mode_screen_and_edit_need_login_and_work_in_inventory_mode(self):
        rig = Rig(self, "d7")
        rig.set_mode_tl("inventory")
        st = rig.state()
        self.assertEqual(st["mode"], "inventory")
        self.assertEqual(names(st), ["Aランチ", "Bランチ"])
        # ログインしていない端末(LAN)は残数も編集も触れない
        self.assertEqual(rig.req("POST", "/display/api/inventory", {"menu": "Aランチ", "remaining": 1}, peer=OUTSIDE)[0], 401)
        self.assertEqual(rig.req("POST", self.EDIT_IPAD, {"op": "delete", "name": "Aランチ"}, peer=OUTSIDE)[0], 401)
        self.assertEqual(list(rig.mgr._items), ["Aランチ", "Bランチ"])
        self.assertEqual(rig.sent, [])

    def test_not_wired_inventory_manager_is_a_clear_503_not_a_crash(self):
        rig = Rig(self, "d8", menu=False)
        ipad = rig.ipad()
        self.assertEqual(rig.req("POST", "/display/api/inventory", {"menu": "A", "remaining": 1}, cookie=ipad)[0], 503)
        self.assertEqual(rig.req("POST", self.EDIT_IPAD, {"op": "add", "name": "A", "initial": 1}, cookie=ipad)[0], 503)
        self.assertEqual(rig.req("GET", "/api/menu/state", cookie=rig.admin())[1]["editable"], False)


# ---------------------------------------------------------------------------
# (e) 移動中の注意との連動
# ---------------------------------------------------------------------------
MOVING_KINDS = (("POLICY_GO_PIN", "go_pin"), ("POLICY_GO_ENTRANCE", "go_entrance"), ("RETURN_HOME", "return_home"),
                ("MANUAL_OVERRIDE", "manual"), ("MAPPING_ACTIVE_HOLD", "exploring"))
NOT_LISTED = ("APPROACH_PERSON", "IDLE_WIGGLE", "CROWD_SCAN", "NEAR_PERSON_HOLD", "WALL_ESCAPE", "PATROL_X", "IDLE")


class MovingFlowTest(unittest.TestCase):
    def status(self, rig):
        return rig.state()["customer"]["status"]

    def test_wiring_matches_full_dashboard_node(self):
        """このテストの配線(Rig.moving_tick / _customer)が、実際のノードのコードとずれていないことの番人。ずれたらこのテストを直す。"""
        src = (PKG / "nokosahen_core" / "full_dashboard_node.py").read_text(encoding="utf-8")
        for needle in ('answering = bool(hub.status_for_ros().get("active")) if hub is not None else False',
                       "self._moving_notice.evaluate(now, survey_answering=answering, speaking=bool(self._audio_busy),",
                       "display_enabled=hub is not None, display_mode=(hub.mode if hub is not None else None))",
                       "(time.monotonic() - self._moving_result_t) <= 1.5",
                       "build_customer_payload(self._menu_state, self._behavior_command, age, self._announce_text, moving)",
                       'self._moving_notice.on_odom(msg.twist.twist.linear.x, time.monotonic())'):
            self.assertIn(needle, src)
        ann = (PKG / "nokosahen_core" / "announce_node.py").read_text(encoding="utf-8")
        self.assertIn("self._situation.on_moving_notice(loads(msg.data, {}), self._now())", ann)

    def test_go_pin_shows_caution_with_sound_then_clears_after_stopping(self):
        rig = Rig(self, "e1")
        rig.run(2.0, 0.0, "POLICY_GO_PIN")                                  # 止まっている: 出ない
        self.assertNotEqual(self.status(rig)["key"], dh.MOVING_STATUS_KEY)
        r, ops = rig.run(1.5, 0.3, "POLICY_GO_PIN")                          # 動き出して0.5秒つづく → 出す
        st = self.status(rig)
        self.assertEqual((st["key"], st["tone"], st["title"], st["moving"]), (dh.MOVING_STATUS_KEY, "caution", mvn.TITLE, "go_pin"))
        self.assertEqual(rig.state()["customer"]["menu"]["items"][0]["name"], "Aランチ")          # 右側の残数は消えない
        self.assertEqual(rig.state()["customer"]["menu"]["items"][0]["remaining"], 60)
        self.assertTrue(r["sound"] and r["display"])
        lo = loop_ops(ops)
        self.assertEqual(len(lo), 1)                                         # 音: 移動し始めに20秒おきの繰り返しを1回開始
        self.assertEqual((lo[0]["interval_sec"], lo[0]["text"]), (20.0, "ロボットが移動しています。ご注意ください"))
        # 動き続ける間は出し続け、繰り返しの開始は(生存信号以外)増えない
        r, ops = rig.run(10.0, 0.3, "POLICY_GO_PIN")
        self.assertEqual(self.status(rig)["key"], dh.MOVING_STATUS_KEY)
        self.assertEqual(stop_ops(ops), [])
        # 止まる: 約1.5秒後に「止まった」。最短3秒表示の保持があるので、すぐは消えない場合がある。遅くとも数秒で消えて音も止まる
        r, ops = rig.run(2.0, 0.0, "POLICY_GO_PIN")
        r, ops2 = rig.run(3.0, 0.0, "POLICY_GO_PIN")
        self.assertNotEqual(self.status(rig)["key"], dh.MOVING_STATUS_KEY)
        self.assertEqual(len(stop_ops(ops + ops2)), 1)
        self.assertFalse(rig.mres["display"])

    def test_every_listed_motion_shows_and_every_other_motion_does_not(self):
        rig = Rig(self, "e2")
        for mode, kind in MOVING_KINDS:
            rig.mn = mvn.MovingNotice()
            rig.run(2.0, 0.3, mode)
            st = self.status(rig)
            self.assertEqual((mode, st["key"], st["moving"]), (mode, dh.MOVING_STATUS_KEY, kind))
            rig.run(5.0, 0.0, mode)                                          # 止めて元に戻す
        for mode in NOT_LISTED:
            rig.mn = mvn.MovingNotice()
            rig.run(3.0, 0.3, mode)                                          # 動いていても、出す動きでなければ出さない
            self.assertNotEqual(self.status(rig)["key"], dh.MOVING_STATUS_KEY, mode)
            self.assertFalse(rig.sit.flags(rig.clk())["moving_notice"], mode)

    def test_exploring_shows_but_makes_no_moving_sound(self):
        rig = Rig(self, "e3")
        r, ops = rig.run(4.0, 0.3, "MAPPING_ACTIVE_HOLD")
        self.assertEqual(self.status(rig)["moving"], "exploring")             # 画面は出る
        self.assertEqual((r["display"], r["sound"]), (True, False))           # 音は鳴らさない(既存の「探索中です」が鳴る)
        self.assertEqual(loop_ops(ops), [])
        self.assertFalse(rig.sit.flags(rig.clk())["moving_notice"])
        # 探索から手動運転に変わると音が始まる
        r, ops = rig.run(2.0, 0.3, "MANUAL_OVERRIDE")
        self.assertTrue(r["sound"])
        self.assertEqual(len(loop_ops(ops)), 1)

    def test_survey_mode_and_answering_never_show_the_notice(self):
        rig = Rig(self, "e4")
        rig.run(2.0, 0.3, "POLICY_GO_PIN")
        self.assertEqual(self.status(rig)["key"], dh.MOVING_STATUS_KEY)
        rig.set_mode_tl("survey")                                            # アンケート表示中: 画面は customer 以外なので出ない
        st = rig.state()
        self.assertEqual(st["mode"], "survey")
        self.assertNotIn("customer", st)
        # 【2026-10-30仕様変更】アンケートが待機中(回答中ではない)のとき、画面を出さないので、音も鳴らさない(画面と音の条件をそろえる)
        r, ops = rig.run(1.0, 0.3, "POLICY_GO_PIN")
        self.assertEqual((r["display"], r["sound"], r["reason"]), (False, False, mvn.R_DISPLAY_MODE))
        self.assertEqual(loop_ops(ops), [])
        self.assertFalse(rig.sit.flags(rig.clk())["moving_notice"])
        rig.set_mode_tl("inventory")                                         # 残数入力の画面も「移動中」で上書きしない(音も鳴らさない)
        r, ops = rig.run(1.0, 0.3, "POLICY_GO_PIN")
        self.assertEqual((r["display"], r["sound"]), (False, False))
        self.assertEqual(loop_ops(ops), [])
        self.assertNotEqual(self.status(rig)["key"], dh.MOVING_STATUS_KEY)
        self.assertNotIn("moving", rig.state()["customer"])
        # 回答中: 判定そのものが出さない(画面も音も)。解除の最短表示の保持も適用しない
        rig.set_mode_tl("survey")
        rig.session("start")
        r, ops = rig.run(1.0, 0.3, "POLICY_GO_PIN")
        self.assertEqual((r["display"], r["sound"], r["reason"]), (False, False, mvn.R_SURVEY))
        self.assertEqual(loop_ops(ops), [])
        # 回答が終わって customer に戻れば、また出る
        rig.answer(valid_response())
        rig.clk.advance(dh.SAVED_SHOW_S + 0.1)
        rig.set_mode_tl("customer")
        rig.run(3.0, 0.3, "POLICY_GO_PIN")
        self.assertEqual(self.status(rig)["key"], dh.MOVING_STATUS_KEY)

    def test_survey_gate_signal_also_silences_the_announcement(self):
        """survey_monitor の survey_active(SurveyGate)が announce の context になり、during_survey=False の音を抑える(二重の安全)。"""
        try:
            from nokosahen_core import survey_monitor as sm
        except ImportError as exc:
            self.skipTest(f"survey_monitorを読み込めない: {exc}")
        rig = Rig(self, "e4b")
        gate = sm.SurveyGate()
        rig.set_mode_tl("survey")
        rig.session("start")
        # 判定側(MovingNotice)の「回答中」を意図的に効かせず、announce 側の抑制だけを確かめる(MovingNotice を直接 sound=True にする)
        for _ in range(8):
            rig.clk.advance(0.25)
            gate.feed(sm.fetch_survey_state(rig.base + "/display/api", 2.0))
            rig.sit.on_survey_active(gate.tick(rig.clk()), rig.clk())
            rig.sit.on_moving_notice({"sound": True, "display": True, "kind": "go_pin"}, rig.clk())
            rig.sit.on_behavior_command({"mode": "POLICY_GO_PIN"}, rig.clk())
            ops = rig.announce()
            self.assertEqual(loop_ops(ops), [])
        self.assertEqual(rig.am.status()["events"]["moving_notice"]["reason"], "アンケート回答中")
        rig.session("end")
        for dt in (0.5, 2.0):                                              # falseを見てから1.5秒で解除
            rig.clk.advance(dt)
            gate.feed(sm.fetch_survey_state(rig.base + "/display/api", 2.0))
            rig.sit.on_survey_active(gate.tick(rig.clk()), rig.clk())
        rig.sit.on_moving_notice({"sound": True, "display": True, "kind": "go_pin"}, rig.clk())
        rig.sit.on_behavior_command({"mode": "POLICY_GO_PIN"}, rig.clk())
        self.assertEqual(len(loop_ops(rig.announce())), 1)

    def test_speaking_and_estop_clear_it_immediately_even_inside_the_minimum_show_time(self):
        rig = Rig(self, "e5")
        rig.run(2.0, 0.3, "POLICY_GO_PIN")
        self.assertEqual(self.status(rig)["key"], dh.MOVING_STATUS_KEY)
        rig.speaking = True                                                   # 呼びかけ・発話中
        r, ops = rig.step(0.25, 0.3, "POLICY_GO_PIN")
        self.assertEqual((r["display"], r["sound"], r["reason"]), (False, False, mvn.R_SPEAKING))
        self.assertNotEqual(self.status(rig)["key"], dh.MOVING_STATUS_KEY)
        self.assertEqual(len(stop_ops(ops)), 1)
        rig.speaking = False
        rig.run(2.0, 0.3, "POLICY_GO_PIN")
        self.assertEqual(self.status(rig)["key"], dh.MOVING_STATUS_KEY)
        r, ops = rig.step(0.25, 0.3, "POLICY_GO_PIN", estop_active=True)      # 緊急停止
        self.assertEqual((r["display"], r["reason"]), (False, mvn.R_ESTOP))
        st = self.status(rig)
        self.assertEqual(st["key"], "ESTOP_HOLD")                              # 通常の「非常停止」の表示が優先
        self.assertEqual(len(stop_ops(ops)), 1)
        # 呼びかけ中のモード
        rig.run(2.0, 0.3, "POLICY_GO_PIN")
        r, ops = rig.step(0.25, 0.3, "STOP_ANNOUNCE")
        self.assertEqual((r["display"], r["reason"]), (False, mvn.R_ANNOUNCE))
        r, ops = rig.step(0.25, 0.3, "SURVEY_ACTIVE_HOLD")
        self.assertEqual((r["display"], r["reason"]), (False, mvn.R_SURVEY))

    def test_stale_data_never_shows_a_lie(self):
        rig = Rig(self, "e6")
        rig.run(2.0, 0.3, "MANUAL_OVERRIDE")
        self.assertEqual(self.status(rig)["key"], dh.MOVING_STATUS_KEY)
        # オドメトリが途絶える(2秒より古い)→ 消える
        r, ops = rig.run(2.5, 0.3, "MANUAL_OVERRIDE", odom=False)
        self.assertEqual((r["display"], r["reason"]), (False, mvn.R_ODOM_STALE))
        self.assertNotEqual(self.status(rig)["key"], dh.MOVING_STATUS_KEY)
        self.assertEqual(len(stop_ops(ops)), 1)
        # 復帰
        rig.run(2.0, 0.3, "MANUAL_OVERRIDE")
        self.assertEqual(self.status(rig)["key"], dh.MOVING_STATUS_KEY)
        # director の指令が途絶える(3秒より古い)→ 消える
        r, ops = rig.run(3.5, 0.3, "MANUAL_OVERRIDE", behavior=False)
        self.assertFalse(r["display"])
        self.assertNotEqual(self.status(rig)["key"], dh.MOVING_STATUS_KEY)
        # 判定そのものが止まった(ダッシュボードの4Hz判定が止まる): 1.5秒より古い結果は画面に使わない・音のフラグも3秒で落ちる
        rig.run(2.0, 0.3, "MANUAL_OVERRIDE")
        self.assertEqual(self.status(rig)["key"], dh.MOVING_STATUS_KEY)
        rig.run(1.75, 0.3, "MANUAL_OVERRIDE", tick=False)
        self.assertNotEqual(self.status(rig)["key"], dh.MOVING_STATUS_KEY)                 # 画面は1.5秒で「移動中」を使わない
        self.assertTrue(rig.sit.flags(rig.clk())["moving_notice"])                          # 音のフラグは3秒まで保つ(announce側の新しさの上限)
        rig.run(1.5, 0.3, "MANUAL_OVERRIDE", tick=False)
        self.assertFalse(rig.sit.flags(rig.clk())["moving_notice"])                         # 3秒で落ちる(鳴りっぱなしにならない)

    def test_broken_speed_values_never_count_as_moving(self):
        rig = Rig(self, "e7")
        for bad in (float("nan"), float("inf"), 99.0, None, "0.3", True):
            rig.mn = mvn.MovingNotice()
            r, _ = rig.run(3.0, bad, "POLICY_GO_PIN")
            self.assertFalse(r["display"], bad)
            self.assertNotEqual(self.status(rig)["key"], dh.MOVING_STATUS_KEY, bad)

    def test_screen_and_sound_decisions_agree_over_a_matrix_of_situations(self):
        """画面(iPad)と音(announce_events のフラグ)が、同じ判定から出る: 音が鳴るなら画面は出ている。画面の有無は MovingNotice と一致。"""
        rig = Rig(self, "e8")
        modes = [m for m, _ in MOVING_KINDS] + ["APPROACH_PERSON", "IDLE_WIGGLE", "STOP_ANNOUNCE", "SURVEY_ACTIVE_HOLD", "ESTOP_HOLD", "NEAR_PERSON_HOLD"]
        n = 0
        for mode in modes:
            for speed in (0.0, 0.3):
                for speaking in (False, True):
                    for estop in (False, True):
                        rig.mn = mvn.MovingNotice()
                        rig.sit = ae.SituationBuilder()
                        rig.speaking = speaking
                        rig.clk.advance(30)
                        rig.run(2.5, speed, mode, estop_active=estop)
                        st = rig.state()["customer"]["status"]
                        shown = st["key"] == dh.MOVING_STATUS_KEY
                        sound = rig.sit.flags(rig.clk())["moving_notice"]
                        tag = (mode, speed, speaking, estop)
                        self.assertEqual(shown, bool(rig.mres["display"]), tag)
                        self.assertEqual(sound, bool(rig.mres["sound"]), tag)
                        if sound:
                            self.assertTrue(shown, tag)
                        expect_show = (speed > 0 and not speaking and not estop and mode in dict(MOVING_KINDS))
                        self.assertEqual(shown, expect_show, tag)
                        self.assertEqual(sound, expect_show and mode != "MAPPING_ACTIVE_HOLD", tag)
                        n += 1
        self.assertEqual(n, len(modes) * 8)

    def test_sound_is_a_loop_every_20_seconds_and_stops_when_the_flag_clears(self):
        rig = Rig(self, "e9")
        r, ops = rig.run(5.0, 0.3, "RETURN_HOME")
        self.assertEqual(len(loop_ops(ops)), 1)
        self.assertEqual(loop_ops(ops)[0]["interval_sec"], 20.0)
        r, ops = rig.run(60.0, 0.3, "RETURN_HOME")                            # 生存信号の再送だけ(KEEPALIVE)。停止は出ない
        self.assertEqual(stop_ops(ops), [])
        self.assertTrue(all(o["interval_sec"] == 20.0 for o in loop_ops(ops)))
        r, ops = rig.run(4.0, 0.0, "RETURN_HOME")
        self.assertEqual(len(stop_ops(ops)), 1)
        self.assertFalse(rig.am.status()["events"]["moving_notice"]["loop_on"])


# ---------------------------------------------------------------------------
# (f) 権限・認証の境界
# ---------------------------------------------------------------------------
class AuthBoundaryFlowTest(unittest.TestCase):
    ADMIN_GET = ("/api/survey/state", "/api/menu/state", "/api/survey/responses.csv")
    ADMIN_POST = (("/api/survey/mode", {"mode": "survey"}), ("/api/menu/edit", {"op": "delete", "name": "Aランチ"}),
                  ("/api/speed_settings", {"manual_linear_mps": 0.3}))
    DISPLAY_WRITE = (("/display/api/survey/session", {"event": "start"}), ("/display/api/survey/answer", None),
                     ("/display/api/inventory", {"menu": "Aランチ", "remaining": 1}),
                     ("/display/api/menu/edit", {"op": "delete", "name": "Aランチ"}))

    def test_ipad_session_cannot_use_the_admin_api(self):
        rig = Rig(self, "f1")
        ipad = rig.ipad()
        for p in self.ADMIN_GET:
            self.assertEqual(rig.rawreq("GET", p, cookie=ipad)[0], 401, p)
        for p, body in self.ADMIN_POST:
            self.assertEqual(rig.req("POST", p, body, cookie=ipad)[0], 401, p)
        self.assertEqual(rig.state()["mode"], "customer")
        self.assertEqual(rig.sent, [])
        # iPad の Cookie は /display 以外には送られない(Path=/display)
        sc = rig.req("POST", "/display/api/login", {"username": "ADMIN_ACCOUNT", "password": DISPLAY_PW})[2]["Set-Cookie"]
        self.assertIn("Path=/display", sc)

    def test_admin_sessions_do_not_log_the_ipad_in(self):
        rig = Rig(self, "f2", perms=["display.survey_mode", "display.menu_edit"])
        for user, pw in (("TL", TL_PW), ("kari-op", OP_PW), ("guest", "")):
            ck = rig.admin(user, pw)
            code, body, _ = rig.rawreq("GET", "/display", cookie=ck, peer=OUTSIDE)
            self.assertEqual((code, body), (200, dp.build_display_login_html()), user)       # 本体の画面にならない
            self.assertEqual(rig.req("GET", "/display/api/state", cookie=ck, peer=OUTSIDE)[0], 401, user)
            for p, body in self.DISPLAY_WRITE:
                self.assertEqual(rig.req("POST", p, body if body is not None else valid_response(), cookie=ck)[0], 401, (user, p))
        # 管理画面のログイン情報では iPad のログインに入れない(逆も)
        for u, p in (("TL", TL_PW), ("kari-op", OP_PW), ("guest", "")):
            self.assertEqual(rig.req("POST", "/display/api/login", {"username": u, "password": p}, peer="192.168.9.9")[0], 401, u)
        self.assertEqual(rig.req("POST", "/api/login", {"username": "ADMIN_ACCOUNT", "password": DISPLAY_PW})[0], 401)
        self.assertEqual(rig.state()["mode"], "customer")
        self.assertEqual(rig.hub.store.count(), 0)

    def test_without_login_only_the_login_page_manifest_icon_and_loopback_state_are_open(self):
        rig = Rig(self, "f3")
        open_paths = {"/display", "/display/", "/display/manifest.webmanifest", "/display/icon.png", "/display/apple-touch-icon.png"}
        candidates = sorted(open_paths | {"/display/api/state", "/display/api/login", "/display/api/inventory", "/display/api/menu/edit",
                                          "/display/api/survey/session", "/display/api/survey/answer", "/display/api/survey/responses.csv",
                                          "/display/api", "/display/x", "/display/../api/survey/state", "/display//api/state", "/display/api/state/x",
                                          "/display/api/menu/state", "/display/index.html", "/display/.git/config"})
        ok_from_outside = set()
        for p in candidates:
            code, body, _ = rig.rawreq("GET", p, peer=OUTSIDE)
            if code == 200:
                ok_from_outside.add(p)
            self.assertNotIn("Aランチ".encode("utf-8"), body, p)                  # 認証なしで状態(メニュー名)を返さない
            self.assertNotIn(b"adminPin", body, p)
        self.assertEqual(ok_from_outside, open_paths)
        # ループバック(同じPCの survey_monitor)なら state も読める。ほかは同じ
        self.assertEqual(rig.req("GET", "/display/api/state")[0], 200)
        # 書き込みは同じPCからでもログインが要る(login 以外はすべて401/404)
        for p, body in self.DISPLAY_WRITE:
            for peer in (None, OUTSIDE):
                self.assertEqual(rig.req("POST", p, body if body is not None else valid_response(), peer=peer, retry=3)[0], 401, (p, peer))
        for p in ("/display/api/state", "/display/api/unknown", "/display/x"):
            self.assertIn(rig.req("POST", p, {}, peer=OUTSIDE, retry=3)[0], (401, 404), p)
        self.assertEqual(rig.sent, [])
        self.assertEqual(rig.hub.store.count(), 0)

    def test_cross_origin_posts_are_refused_even_with_a_valid_login(self):
        rig = Rig(self, "f4")
        ipad = rig.ipad()
        rig.set_mode_tl("survey")
        host = rig.base.replace("http://", "")
        evil_headers = (({"Origin": "http://evil.example"}, "別サイトのOrigin"), ({"Origin": "null"}, "Origin: null"),
                        ({"Sec-Fetch-Site": "cross-site"}, "cross-site"), ({"Sec-Fetch-Site": "same-site"}, "same-site"),
                        ({"Origin": f"http://{host}.evil.example"}, "Hostに似たOrigin"), ({"Origin": f"https://{host}x"}, "ポート違い"))
        posts = list(self.DISPLAY_WRITE)
        posts[1] = ("/display/api/survey/answer", valid_response())
        for hdr, why in evil_headers:
            for p, body in posts:
                code, d, _ = rig.req("POST", p, body, cookie=ipad, headers=hdr)
                self.assertEqual((code, d.get("error")), (403, "bad_origin"), (why, p))
            code, d, _ = rig.req("POST", "/display/api/login", {"username": "ADMIN_ACCOUNT", "password": DISPLAY_PW}, headers=hdr)
            self.assertEqual(code, 403, why)                      # ログインも別サイトからは受け付けない
        self.assertEqual(rig.sent, [])
        self.assertEqual(rig.hub.store.count(), 0)
        self.assertFalse(rig.state()["active"])                   # 回答中にもなっていない
        # 同じオリジン(ブラウザが付ける Origin / Sec-Fetch-Site)は通る
        good = {"Origin": rig.base, "Sec-Fetch-Site": "same-origin"}
        self.assertEqual(rig.req("POST", "/display/api/survey/session", {"event": "start"}, cookie=ipad, headers=good)[0], 200)
        self.assertEqual(rig.req("POST", "/display/api/survey/answer", valid_response(), cookie=ipad, headers=good)[0], 200)

    def test_admin_api_posts_with_a_foreign_origin_are_refused(self):
        """【2026-10-30】管理画面側(TL)のPOSTも、別サイトのOrigin/Sec-Fetch-Siteなら403で、モードは変わらない。同一オリジン・ヘッダ無しは通る。"""
        rig = Rig(self, "f5")
        tl = rig.admin()
        before = rig.state()["mode"]
        for hdr in ({"Origin": "http://evil.example", "Sec-Fetch-Site": "cross-site"}, {"Origin": "http://evil.example"}, {"Sec-Fetch-Site": "cross-site"}):
            code, d, _ = rig.req("POST", "/api/survey/mode", {"mode": "survey"}, cookie=tl, headers=hdr)
            self.assertEqual((code, d.get("error")), (403, "cross_origin"), hdr)
            self.assertEqual(rig.state()["mode"], before)
        code, d, _ = rig.req("POST", "/api/survey/mode", {"mode": "survey"}, cookie=tl, headers={"Origin": rig.base, "Sec-Fetch-Site": "same-origin"})
        self.assertEqual(code, 200)
        self.assertEqual(rig.state()["mode"], "survey")
        code, d, _ = rig.req("POST", "/api/survey/mode", {"mode": "customer"}, cookie=tl)      # ヘッダ無し(curl・内部呼び出し)も従来どおり
        self.assertEqual(code, 200)


# ---------------------------------------------------------------------------
# 実機用スモークテスト(scripts/pc/smoke_test_jetson1.py)の確認: 同じ確認を PC 上の模擬サーバに対して通す
# ---------------------------------------------------------------------------
class SmokeScriptAgainstLocalRigTest(unittest.TestCase):
    """実機に実行する前に、スモークテストのスクリプト自体が正しく動く(合格すべきものは合格し、モードは必ず元に戻る)ことを確かめる。"""

    @classmethod
    def setUpClass(cls):
        import importlib.util
        path = PKG.parents[2] / "scripts" / "pc" / "smoke_test_jetson1.py"
        spec = importlib.util.spec_from_file_location("smoke_test_jetson1", path)
        cls.smoke = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.smoke)

    def make(self, rig):
        sm = self.smoke.Smoke("127.0.0.1", 5.0)
        sm.admin_port = rig.srv.server_address[1]
        return sm

    def fails(self, sm):
        return [(n, d) for st, n, d in sm.results if st == "FAIL"]

    def test_public_checks_and_login_pass_and_the_password_is_never_printed(self):
        import contextlib
        import io
        rig = Rig(self, "g1")
        rig.srv.peer = OUTSIDE                                           # 実機のPCは LAN 上の別端末(ループバックではない)
        sm = self.make(rig)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            sm.check_display_public()
            ck = sm.check_display_login(DISPLAY_PW)
        out = buf.getvalue()
        self.assertIsNotNone(ck)
        self.assertEqual(self.fails(sm), [], out)
        self.assertNotIn(DISPLAY_PW, out)
        self.assertNotIn(ck.split("=", 1)[1], out)                      # Cookie の値も出さない
        self.assertGreaterEqual(sum(1 for st, _, _ in sm.results if st == "PASS"), 15)

    def test_login_checks_are_skipped_without_the_password(self):
        import contextlib
        import io
        rig = Rig(self, "g2")
        sm = self.make(rig)
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertIsNone(sm.check_display_login(""))
        self.assertEqual([st for st, _, _ in sm.results], ["SKIP"])

    def test_mode_switch_is_restored_even_in_each_original_mode(self):
        import contextlib
        import io
        for orig in ("customer", "inventory", "ocrtest", "survey"):
            rig = Rig(self, "g3_" + orig)
            rig.hub.request_mode(orig, "ros")
            sm = self.make(rig)
            with contextlib.redirect_stdout(io.StringIO()) as buf:
                sm.check_mode_switch("TL", TL_PW, None)
            self.assertEqual(self.fails(sm), [], (orig, buf.getvalue()))
            self.assertEqual(rig.state()["mode"], orig)                  # 元のモードに戻っている

    def test_mode_switch_is_skipped_while_someone_is_answering_and_nothing_changes(self):
        import contextlib
        import io
        rig = Rig(self, "g4")
        rig.set_mode_tl("survey")
        rig.session("start")
        rev = rig.state()["rev"]
        sm = self.make(rig)
        with contextlib.redirect_stdout(io.StringIO()):
            sm.check_mode_switch("TL", TL_PW, None)
        self.assertEqual([st for st, n, _ in sm.results if "モード切替" in n or "元に" in n], ["SKIP"])
        st = rig.state()
        self.assertEqual((st["mode"], st["rev"], st["active"]), ("survey", rev, True))       # 回答中のものは触らない

    def test_wrong_admin_password_fails_clearly_and_changes_nothing(self):
        import contextlib
        import io
        rig = Rig(self, "g5")
        sm = self.make(rig)
        with contextlib.redirect_stdout(io.StringIO()):
            sm.check_mode_switch("TL", "wrong-password", None)
        self.assertEqual(len(self.fails(sm)), 1)
        self.assertEqual(rig.state()["mode"], "customer")


if __name__ == "__main__":
    unittest.main(verbosity=1)
