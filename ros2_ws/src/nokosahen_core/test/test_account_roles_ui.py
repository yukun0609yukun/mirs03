"""operator(弱いアカウント)の画面のテスト(2026-10-13追加、docs/accounts.md)。実ブラウザ(Playwright/Edge)。

本物の make_dashboard_handler で完成版ダッシュボードのHTMLを配信し(2台目・ラズパイは偽サーバー、ロボットには何も送らない)、
1. operator(許可なし): 許可されていない入力欄・ボタンが disabled になり理由(title)とカードの注意が出る。
   ナビゲーション・ログアウト・緊急停止(かける)は使える。ヘッダーに「一時的な変更(再起動で元に戻ります)」が出る。
   「アカウントと権限」カードは出ない。画面の無効化を外して送ってもサーバーが403。
2. operator(音量・表示切替を許可): 該当カードだけ有効、他は無効のまま(BGMのON/OFF=未実装、テスト発話=未許可)
3. admin: 「アカウントと権限」カード(アカウント一覧・許可チェックリスト・一時的な上書き)。admin_only/未実装は選択不可、
   保存は確認バー付き、operatorの一時的な変更の件数が出て「全て破棄」で2台目に/ephemeral_resetが届く
4. ページにJSエラーが出ない
スクリーンショットを1枚、SCREENSHOT環境変数のパス(既定はテンポラリ)に保存する。

実行: python ros2_ws/src/nokosahen_core/test/test_account_roles_ui.py (普通のpythonで。-I は付けない)
(playwright / Edge が無い環境ではスキップ)
"""
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

try:
    from playwright.sync_api import sync_playwright
    HAVE_PW = True
except Exception:  # noqa: BLE001
    HAVE_PW = False

import test_account_roles_http as H  # noqa: E402  (偽の2台目/ラズパイ・ログイン用の部品を再利用)
import test_dashboard_field_sync as FS  # noqa: E402  (rclpy無しでHTMLを組み立てる部品を再利用)

from nokosahen_core import accounts as acc  # noqa: E402
from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import ephemeral_overrides as eo  # noqa: E402

SHOT = os.environ.get("SCREENSHOT") or os.path.join(tempfile.gettempdir(), "accounts_card.png")


@unittest.skipUnless(HAVE_PW, "playwright がありません")
class AccountsUiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge_srv, bport = H.serve(H.Bridge)
        cls.pi_srv, pport = H.serve(H.Pi)
        cls.bridge_url = f"http://127.0.0.1:{bport}"
        cls.pi_url = f"http://127.0.0.1:{pport}"
        cls.pw = sync_playwright().start()
        cls.browser = cls.pw.chromium.launch(channel="msedge", headless=True)

    @classmethod
    def tearDownClass(cls):
        cls.browser.close()
        cls.pw.stop()
        for s in (cls.bridge_srv, cls.pi_srv):
            s.shutdown()
            s.server_close()

    def start(self, perms):
        H.Bridge.posts = []
        H.Pi.posts = []
        H.Pi.mode = "customer"
        tmp = Path(tempfile.mkdtemp(prefix="acct_ui_"))
        (tmp / "accounts.yaml").write_text(
            'accounts:\n  - username: "kari-op"\n    role: operator\n'
            f'    password_hash: "{H.OP_HASH}"\n    enabled: true\n'
            '  - username: "CHANGE_ME_OPERATOR"\n    role: operator\n    password_hash: ""\n    enabled: false\n',
            encoding="utf-8")
        store = acc.AccountStore(accounts_path=str(tmp / "accounts.yaml"), permissions_path=str(tmp / "perm.json"))
        if perms:
            store.save_permissions(perms, "setup")
        self.eph = eo.EphemeralOverrides()
        handler = dc.make_dashboard_handler(
            lambda: FS.PAGES["full"].encode("utf-8"), lambda: {}, H.FakeTeleop(),
            bgm=dc.BgmProxy(self.bridge_url), voice_master=dc.VoiceMasterProxy(self.bridge_url),
            voice_levels=dc.VoiceLevelsProxy(self.bridge_url), survey_admin=dc.SurveyAdminProxy(self.pi_url),
            auth_username="TL", auth_password="tl-pass", enforce_single_controller=False,
            account_store=store, ephemeral=self.eph,
            voice_settings=H.VoiceSettingsController(H.AnnounceSettingsStore(str(tmp / "announce.json")),
                                                     H.VoiceSettingsProxy(self.bridge_url)),
            voice_settings_html_getter=lambda: FS.PAGES["voice"].encode("utf-8"))
        self.srv, port = H.serve(handler)
        self.base = f"http://127.0.0.1:{port}"
        self.addCleanup(lambda: (self.srv.shutdown(), self.srv.server_close()))

    def page_for(self, user, pw):
        ctx = self.browser.new_context(viewport={"width": 1280, "height": 1800})
        self.addCleanup(ctx.close)
        r = ctx.request.post(self.base + "/api/login", data=json.dumps({"username": user, "password": pw}),
                             headers={"Content-Type": "application/json"})
        self.assertEqual(r.status, 200)
        page = ctx.new_page()
        self.errors = []
        page.on("pageerror", lambda e: self.errors.append(str(e)))
        page.goto(self.base + "/")
        page.wait_for_function("() => window.NKRole && window.NKRole.who() !== null", timeout=15000)
        page.wait_for_timeout(500)
        return page

    def tab(self, page, name):
        page.evaluate("(n) => { document.querySelector('#tabs [data-tab=' + n + ']').click(); }", name)
        page.wait_for_timeout(300)
        # 非表示のカード(2台目の状態を読み込むまで display:none)が出るのを待つ
        page.wait_for_timeout(1500)
        page.evaluate("() => window.NKRole && window.NKRole.refresh()")
        page.wait_for_timeout(500)

    def state(self, page, sel):
        return page.evaluate("""(s) => { const e = document.querySelector(s); return e ? {disabled: e.disabled, title: e.title || ''} : null; }""", sel)

    # ---------------------------------------------------------------
    def test_operator_without_permissions(self):
        self.start([])
        page = self.page_for("kari-op", H.OP_PASSWORD)
        banner = page.inner_text("#nkRoleBanner")
        self.assertIn("再起動すると元に戻ります", banner)
        self.assertIn("kari-op", banner)
        self.tab(page, "voice")
        st = self.state(page, "#vlApplyBtn")
        self.assertTrue(st["disabled"])
        self.assertIn("許可されていません", st["title"])
        self.assertTrue(self.state(page, "#vlSpeechNum")["disabled"])
        self.assertTrue(self.state(page, "#voiceMasterBtn")["disabled"])
        self.tab(page, "settings")
        self.assertTrue(self.state(page, "#surveyToSurveyBtn")["disabled"])
        calib = self.state(page, "#calibrateBgBtn")
        self.assertTrue(calib["disabled"])
        self.assertIn("管理者のみ", calib["title"])
        self.assertGreaterEqual(page.locator(".nk-role-note").count(), 1)
        self.assertIn("管理者のみ", page.locator(".nk-role-note").first.inner_text())
        # 使えるもの: ナビゲーション(リンク)・ログアウト・緊急停止ボタン
        self.assertFalse(self.state(page, "#logoutBtn")["disabled"])
        self.assertFalse(self.state(page, "#estopBtn")["disabled"])
        # 管理カードは出ない(adminだけ)
        self.assertTrue(page.evaluate("() => document.getElementById('grpAccounts').hidden"))
        # 画面の無効化を外して送っても、サーバーが403
        code = page.evaluate("""async () => { document.getElementById('vlApplyBtn').disabled = false;
          const r = await fetch('/api/speed_settings', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({manual_linear_mps: 0.3})}); return r.status; }""")
        self.assertEqual(code, 403)
        code = page.evaluate("""async () => (await fetch('/api/voice_levels', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({speech_volume: 2})})).status""")
        self.assertEqual(code, 403)
        self.assertEqual(H.Bridge.posts, [])
        self.assertEqual(self.errors, [])

    def test_operator_with_voice_levels_and_survey_mode(self):
        self.start(["voice.levels", "display.survey_mode"])
        page = self.page_for("kari-op", H.OP_PASSWORD)
        self.tab(page, "voice")
        self.assertFalse(self.state(page, "#vlApplyBtn")["disabled"])
        self.assertFalse(self.state(page, "#vlSpeechNum")["disabled"])
        self.assertTrue(self.state(page, "#vlTestBtn")["disabled"], "テスト発話は別の権限(voice.test_speak)")
        self.assertTrue(self.state(page, "#voiceMasterBtn")["disabled"], "全体ON/OFFは別の権限(voice.master)")
        bgm = self.state(page, "#bgmToggleBtn")
        if bgm is not None:
            self.assertTrue(bgm["disabled"])
            self.assertIn("管理者のみ", bgm["title"], "BGMのON/OFFは付与できない権限(未実装)")
        self.tab(page, "settings")
        self.assertFalse(self.state(page, "#surveyToSurveyBtn")["disabled"])
        self.assertTrue(self.state(page, "#ocrMockRandBtn")["disabled"], "模擬POSは別の権限(display.ocr_mock)")
        self.assertTrue(self.state(page, "#ocrToDemoBtn")["disabled"], "OCR画質は付与できない権限")
        # 許可された操作は通り、2台目には persist:false で届く
        code = page.evaluate("""async () => (await fetch('/api/voice_levels', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({speech_volume: 2.5})})).status""")
        self.assertEqual(code, 200)
        self.assertEqual(H.Bridge.posts[-1], ("/voice_levels", {"speech_volume": 2.5, "persist": False}))
        page.evaluate("() => window.NKRole.refresh()")
        page.wait_for_timeout(800)
        self.assertIn("現在の一時的な変更: 1 件", page.inner_text("#nkRoleBanner"))
        self.assertEqual(self.errors, [])

    def test_admin_accounts_card(self):
        self.start(["voice.levels"])
        # operatorが先に一時的な変更をしておく
        _c, _d, setck = H.http("POST", self.base + "/api/login", {"username": "kari-op", "password": H.OP_PASSWORD})
        H.http("POST", self.base + "/api/voice_levels", {"speech_volume": 3.0}, setck.split(";")[0])
        page = self.page_for("TL", "tl-pass")
        self.assertIn("一時的な上書きが 1 件", page.inner_text("#nkRoleBanner"))
        self.tab(page, "settings")
        page.wait_for_function("() => !document.getElementById('grpAccounts').hidden", timeout=15000)
        rows = page.inner_text("#acctRows")
        self.assertIn("TL", rows)
        self.assertIn("guest", rows)
        self.assertIn("kari-op", rows)
        self.assertIn("CHANGE_ME_OPERATOR", rows)
        self.assertIn("プレースホルダー", rows)
        self.assertNotIn(H.OP_HASH, page.content())
        self.assertNotIn("tl-pass", page.content())
        # 許可チェックリスト: 付与可能は選べる / admin_only・未実装は選べない
        enabled = page.evaluate("() => Array.from(document.querySelectorAll('#permList input[data-perm]')).filter(e => !e.disabled).map(e => e.dataset.perm)")
        disabled = page.evaluate("() => Array.from(document.querySelectorAll('#permList input[data-perm]')).filter(e => e.disabled).map(e => e.dataset.perm)")
        self.assertIn("voice.levels", enabled)
        self.assertIn("voice.master", enabled)
        for must in ("robot.speed", "map.boundary", "behavior.tree_edit", "accounts.manage", "voice.bgm_onoff"):
            self.assertIn(must, disabled)
            self.assertNotIn(must, enabled)
        self.assertTrue(page.evaluate("() => document.querySelector('#permList input[data-perm=\"voice.levels\"]').checked"))
        # 変更 -> 確認バー -> 保存
        page.evaluate("() => document.querySelector('#permList input[data-perm=\"voice.master\"]').click()")
        self.assertIn("未保存", page.inner_text("#permBadge"))
        page.evaluate("() => document.getElementById('permSaveBtn').click()")
        self.assertFalse(page.evaluate("() => document.getElementById('permConfirm').hidden"))
        self.assertIn("全体ON/OFF", page.inner_text("#permConfirmText"))
        page.evaluate("() => document.getElementById('permConfirmYes').click()")
        page.wait_for_function("() => document.getElementById('permHint').textContent.includes('保存しました')", timeout=10000)
        c, who = H.http("GET", self.base + "/api/whoami", None, setck.split(";")[0])[:2]
        self.assertIn("voice.master", who["permissions"])
        # 一時的な上書きの一覧
        eph = page.inner_text("#ephRows")
        self.assertIn("発話音量", eph)
        self.assertIn("kari-op", eph)
        # スクリーンショット(設定タブの「アカウントと権限」)
        page.locator("#grpAccounts").screenshot(path=SHOT)
        # 全て破棄 -> 確認バー -> 2台目へ /ephemeral_reset
        H.Bridge.posts = []
        page.evaluate("() => document.getElementById('ephClearBtn').click()")
        page.evaluate("() => document.getElementById('ephConfirmYes').click()")
        page.wait_for_function("() => document.getElementById('ephHint').textContent.includes('破棄')", timeout=10000)
        self.assertIn(("/ephemeral_reset", {}), H.Bridge.posts)
        self.assertEqual(self.eph.count(), 0)
        self.assertEqual(self.errors, [])

    def test_operator_manual_drive_reaches_server_and_page_does_not_block_it(self):
        # 【2026-11-06 仕様変更】以前は「operatorの運転は画面側で止まり、サーバーに届かない」を検証していた。
        # 手動運転(robot.manual_drive)は弱いアカウントも既定で可になったので、「画面は止めず、サーバーが受け付ける。
        # 速度設定など管理者専用の操作は従来どおり403」に変えた(ロール別に挙動が決まっているという意図は維持)。
        self.start([])
        page = self.page_for("kari-op", H.OP_PASSWORD)
        page.wait_for_function("() => window.NKRole && window.NKRole.who() !== null", timeout=15000)
        self.assertIn("robot.manual_drive", page.evaluate("() => window.NKRole.who().permissions"))
        r = page.evaluate("""async () => { const a = await fetch('/api/teleop', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({forward: 0.5, turn: 0})});
          const b = await fetch('/api/teleop', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({forward: 0, turn: 0})});
          const c = await fetch('/api/speed_settings', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({manual_linear_mps: 0.3})});
          return [a.status, (await a.json()).ok, b.status, (await b.json()).reason || null, c.status]; }""")
        self.assertEqual(r, [200, True, 200, None, 403])

    def test_voice_settings_page_follows_permissions(self):
        self.start(["voice.announce_interval"])
        page = self.page_for("kari-op", H.OP_PASSWORD)
        page.goto(self.base + "/voice_settings")
        page.wait_for_function("() => window.NKRole && window.NKRole.who() !== null && document.getElementById('fTemp')", timeout=15000)
        page.wait_for_timeout(1500)
        self.assertFalse(self.state(page, "#intervalInput")["disabled"])
        self.assertTrue(self.state(page, "#fTemp")["disabled"])
        self.assertTrue(self.state(page, "#personaText")["disabled"])
        self.assertFalse(self.state(page, "#saveBtn")["disabled"], "発話間隔の権限があれば保存ボタンは有効")
        self.assertFalse(self.state(page, "#tabBtn-prompt")["disabled"], "タブは使える")
        self.assertIn("再起動すると元に戻ります", page.inner_text("#nkRoleBanner"))
        self.assertEqual(self.errors, [])
        # 権限なしなら保存ボタンも無効
        self.start([])
        page = self.page_for("kari-op", H.OP_PASSWORD)
        page.goto(self.base + "/voice_settings")
        page.wait_for_function("() => window.NKRole && window.NKRole.who() !== null", timeout=15000)
        page.wait_for_timeout(1500)
        self.assertTrue(self.state(page, "#saveBtn")["disabled"])
        self.assertTrue(self.state(page, "#intervalInput")["disabled"])

    def test_admin_other_pages_get_no_operator_lock(self):
        self.start([])
        page = self.page_for("TL", "tl-pass")
        self.tab(page, "voice")
        self.assertFalse(self.state(page, "#vlApplyBtn")["disabled"])
        self.assertEqual(page.locator(".nk-role-note").count(), 0)


if __name__ == "__main__":
    unittest.main(verbosity=1)
