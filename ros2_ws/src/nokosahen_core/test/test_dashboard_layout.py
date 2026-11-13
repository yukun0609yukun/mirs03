"""完成版ダッシュボード(PC版・スマホ版)のレイアウト再構成(2026-10-06)の回帰テスト。

JSが参照する要素idが新しいマークアップに全て存在すること、id重複が無いこと、
全リンクが残っていること、緊急停止・運転まわりの安全処理(JS)が消えていないことを確認する。
rclpy無しで動く(ノードのHTML定数をASTから取り出して評価する)。
"""
import ast
import importlib.util
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

PKG = Path(__file__).resolve().parents[1] / "nokosahen_core"
NODE = shutil.which("node")


def _load(name):
    spec = importlib.util.spec_from_file_location(name, PKG / (name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def pages():
    mv, nl = _load("map_view"), _load("nav_layout")
    ns = {
        "LOGO_DATA_URI": "", "MAP_VIEW_JS": mv.MAP_VIEW_JS, "NAV_CSS": nl.NAV_CSS, "NAV_JS": nl.NAV_JS,
        "link_cards_html": nl.link_cards_html, "nav_drawer_html": nl.nav_drawer_html,
        "nav_header_html": nl.nav_header_html,
        "ACCOUNTS_CARD_HTML": _load("accounts_ui").ACCOUNTS_CARD_HTML, "ACCOUNTS_CARD_JS": _load("accounts_ui").ACCOUNTS_CARD_JS,
        "SHUTDOWN_CARD_HTML": _load("system_shutdown_ui").SHUTDOWN_CARD_HTML, "SHUTDOWN_CARD_JS": _load("system_shutdown_ui").SHUTDOWN_CARD_JS,
    }
    # eval対象はこのリポジトリ自身のソース内の文字列連結式のみ(外部入力ではない)
    tree = ast.parse((PKG / "full_dashboard_node.py").read_text(encoding="utf-8"))
    out = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") in ("HTML", "MOBILE_HTML"):
            out[node.targets[0].id] = eval(compile(ast.Expression(node.value), "x", "eval"), ns)  # noqa: S307
    return out


def split(html):
    scripts = re.findall(r"<script>(.*?)</script>", html, re.S)
    markup = re.sub(r"<script>.*?</script>", "", html, flags=re.S)
    markup = re.sub(r"<style>.*?</style>", "", markup, flags=re.S)
    return markup, "\n".join(scripts), scripts


# JSが実行時に作る要素(静的マークアップには無くてよい)
DYNAMIC_IDS = {"errorBanner", "estopWarning", "tlActiveBanner"}

REQUIRED_LINKS = [
    "/utterance_log", "/voice_settings", "/behavior_rules", "/detection_log", "/step_response",
    "/consumption_graph", "/recommendation_effectiveness", "/activity_report", "/audience_display",
    "/map_editor", "/behavior_tree",
]


class FullDashboardLayout(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pages = pages()

    def _check_ids(self, key):
        html = self.pages[key]
        markup, js, _ = split(html)
        static_ids = re.findall(r'\bid="([^"]+)"', markup)
        dup = sorted({i for i in static_ids if static_ids.count(i) > 1})
        self.assertEqual(dup, [], f"{key}: duplicate ids")
        # JSの文字列で組み立てられるid(map_viewが挿入する回転UI等)も「存在する」とみなす
        js_defined = set(re.findall(r'\bid="([^"]+)"', js)) | set(re.findall(r'\.id\s*=\s*"([^"]+)"', js))
        defined = set(static_ids) | js_defined | DYNAMIC_IDS
        # poseEl("id")はmap_viewの現在位置指定UI用のgetElementByIdの別名(2026-10-07)
        referenced = set(re.findall(r'(?:getElementById|poseEl)\("([^"]+)"\)', js))
        referenced |= set(re.findall(r"querySelector\(\"#([A-Za-z0-9_-]+)\"\)", js))
        missing = sorted(referenced - defined)
        self.assertEqual(missing, [], f"{key}: JS references ids that are not in the markup")
        return referenced

    def test_pc_ids_exist(self):
        ref = self._check_ids("HTML")
        for must in ("estopBtn", "resumeBtn", "mapCanvas", "lidarCanvas", "cameraView", "speedSaveBtn",
                     "bgmBlock", "mapStartBtn", "boundarySetBtn", "explorationStartBtn", "forceDemoBtn",
                     "calibrateBgBtn", "menuRows", "mobileUrl", "logoutBtn",
                     # 2026-10-07 現在位置の指定・位置の確かさ・不確かな時の目的地確認
                     "poseToolBtn", "globalLocBtn", "poseHint", "locChip", "locDetail", "locWarn",
                     "poseConfirmBar", "poseConfirmText", "poseSendBtn", "poseRetryBtn",
                     "goalConfirmBar", "goalConfirmText", "goalGoBtn", "goalCancelBtn"):
            self.assertIn(must, ref)

    def test_localization_ui_wiring(self):
        html = self.pages["HTML"]
        markup = split(html)[0]
        # 確認バーは最初は隠れている(既定は送らない)
        for i in ("poseConfirmBar", "goalConfirmBar", "locWarn"):
            self.assertRegex(markup, r'id="%s"[^>]*\bhidden\b' % i)
        self.assertIn("クリックで走行するのは位置の確かさが「良好」になってからにしてください", markup)
        self.assertIn("位置 (", html)
        self.assertIn("このまま走行しますか", html)
        # 目的地クリックは確認ゲート経由(直接/api/goalへ送らない)。確認後の送信だけが/api/goalへ送る
        self.assertIn("requestGoal(x, y);", html)
        self.assertNotIn('postJson("/api/goal", { x, y });', html)
        self.assertIn("/api/initial_pose", html)
        self.assertIn("/api/global_localization", html)
        self.assertIn("TFなし(位置合わせ前)", html)
        # ポップアップ(confirm/alert)は使わない: 新機能のコードにwindow.confirmの呼び出しが無いこと
        start = html.index("現在位置の指定(AMCLの初期位置)・位置の確かさ")
        end = html.index("表示回転のUI・サーバー同期")
        self.assertNotRegex(html[start:end], r"\b(confirm|alert|prompt)\(")

    def test_map_panel_layout(self):
        """【2026-10-08】地図が主役のレイアウト: 1本のツールバー+ポップオーバー、通知ストリップ、独立したピンカード。"""
        html = self.pages["HTML"]
        markup, js, _ = split(html)
        css = re.search(r"<style>(.*?)</style>", html, re.S).group(1)
        ids = set(re.findall(r'\bid="([^"]+)"', markup)) | set(re.findall(r'\bid="([^"]+)"', js))
        for i in ("mapToolbar", "mapStage", "mapNotes", "pinMenuBtn", "pinPop", "mapHelpBtn", "mapHelpPop",
                  "pinColorDd", "pinColorDot", "pinColorTxt", "pinsCard", "pinList", "pinCount",
                  "mapFitBtn", "mapRotMenuBtn", "mapRotPop", "mapRotSlider", "mapRotNum", "mapRot90Btn",
                  "mapRot0Btn", "mapRotAutoBtn", "mapRotPickBtn", "mapRotSaveBtn", "estopDock"):
            self.assertIn(i, ids)
        self.assertIn("mapRotHint", js)
        # 地図キャンバスはツールの数に関係なく大きさを確保(480px以上/スマホは60vh)
        self.assertRegex(css, r"\.map-stage \{[^}]*height:max\(480px")
        self.assertIn("60vh", css)
        # キャンバスと通知ストリップは同じステージの中(通知はキャンバスを押し縮めない)
        stage = markup[markup.index('id="mapStage"'):]
        stage = stage[:stage.index("</section>")]
        for i in ("mapCanvas", "mapNotes", "pinSelBar", "goalProgressBar", "pinPlaceBar", "locWarn",
                  "poseConfirmBar", "goalConfirmBar", "poseHint", "pinHint"):
            self.assertIn('id="%s"' % i, stage)
        # 確認バーは最初は隠れている/ピン一覧は右カラム(sidecol)にある/緊急停止ドックは最前面のまま
        self.assertGreater(markup.index('id="pinList"'), markup.index('class="sidecol"'))
        self.assertIn("z-index:2000", css)
        self.assertLess(int(re.search(r"\.map-notes \{[^}]*z-index:(\d+)", css).group(1)), 2000)
        # s-drive/s-sensors/s-folds/s-pins/s-mapのsectionが全て閉じている(以前はs-driveの閉じ忘れで入れ子になっていた)
        pane = markup[markup.index('id="tab-map"'):markup.index('id="tab-voice"')]
        self.assertEqual(pane.count("<section"), pane.count("</section>"))

    def test_mobile_ids_exist(self):
        ref = self._check_ids("MOBILE_HTML")
        for must in ("estopBtn", "speedSaveBtn", "wiggleOnBtn", "bgmBlock", "speedSection"):
            self.assertIn(must, ref)

    def test_all_links_present_in_pc_page(self):
        html = self.pages["HTML"]
        for href in REQUIRED_LINKS:
            self.assertIn('href="%s"' % href, html)
        self.assertIn('data-ocr="1"', html)
        # ドロワー(全タブ共通)にも同じリンクがあること
        drawer = html[html.index('id="menuDrawer"'):html.index("</aside>")]
        for href in REQUIRED_LINKS:
            self.assertIn('href="%s"' % href, drawer)

    def test_pc_tabs_and_estop_dock(self):
        html = self.pages["HTML"]
        for t in ("dash", "map", "voice", "logs", "settings"):
            self.assertIn('data-tab="%s"' % t, html)
            self.assertIn('data-pane="%s"' % t, html)
        self.assertIn('id="estopDock"', html)
        # 緊急停止ドックは全タブの外(main の外)にある
        self.assertGreater(html.index('id="estopDock"'), html.index("</main>"))
        self.assertIn("z-index:2000", html)
        # タブはbuttonではなくa(guestで全button/input/selectを無効化しても切り替えられるように)
        self.assertNotRegex(html, r"<button[^>]*data-tab=")

    def test_mobile_estop_and_drive_come_first(self):
        html = self.pages["MOBILE_HTML"]
        self.assertLess(html.index('id="estopBtn"'), html.index('class="dpad"'))
        self.assertLess(html.index('class="dpad"'), html.index('id="speedSection"'))
        self.assertIn("position:sticky", html)

    def test_safety_js_untouched(self):
        pc, mob = self.pages["HTML"], self.pages["MOBILE_HTML"]
        for needle in ("releaseAllKeys", 'addEventListener("blur"', 'addEventListener("pagehide"',
                       "visibilitychange", "window.confirm = ", "window.alert = ", "syncEstop",
                       "estopIgnoreSyncUntil", "teleopSeq"):
            self.assertIn(needle, pc, f"PC: {needle}")
        for needle in ("pointercancel", "lostpointercapture", "touchcancel", "stopDrive",
                       'addEventListener("blur"', "visibilitychange", "window.alert = ", "syncEstop",
                       "estopIgnoreSyncUntil", "teleopSeq"):
            self.assertIn(needle, mob, f"mobile: {needle}")

    @unittest.skipIf(NODE is None, "node not installed")
    def test_scripts_pass_node_check(self):
        for key, html in self.pages.items():
            for i, body in enumerate(split(html)[2]):
                with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False, encoding="utf-8") as f:
                    f.write(body)
                r = subprocess.run([NODE, "--check", f.name], capture_output=True, text=True)
                self.assertEqual(r.returncode, 0, f"{key} script#{i}: {r.stderr}")


    # ---- 【2026-10-10】配置の不変条件(全体再設計) ----
    def _pane(self, name):
        markup = split(self.pages["HTML"])[0]
        i = markup.index('id="tab-%s"' % name)
        nxt = [markup.find('data-pane="%s"' % t, i + 10) for t in ("dash", "map", "voice", "logs", "settings") if t != name]
        nxt = [n for n in nxt if n > 0]
        end = min(nxt) if nxt else markup.index("</main>")
        return markup[i:end]

    def test_each_tab_has_its_required_cards(self):
        need = {
            "dash": ("statusStrip", "sumStop", "sumMode", "sumLoc", "sumVoice", "sumBgm", "sumSurvey", "sumOcr", "mode", "modeJa", "menuRows"),
            "map": ("mapStage", "pinsCard", "resumeBtn", "speedSaveBtn", "navSpeedBtn", "mapStartBtn", "mapResetBtn", "boundarySetBtn", "lidarCanvas"),
            "voice": ("voiceMasterBlock", "voiceLevelsBlock", "bgmBlock", "bgmToggleBtn", "bgmVolume", "forceDemoBtn"),
            "logs": (),
            "settings": ("surveyBlock", "ocrMockBlock", "ocrBlock", "ocrTuneBtn", "calibrateBgBtn", "grpMaint", "grpRules"),
        }
        for tab, ids in need.items():
            pane = self._pane(tab)
            for i in ids:
                self.assertIn('id="%s"' % i, pane, f"{tab}: {i}")
        # 動作ルールのリンクは設定タブ、発話履歴・呼びかけ音声設定は発話・音声タブ、ログはログタブ
        self.assertIn('href="/behavior_rules"', self._pane("settings"))
        self.assertIn('href="/behavior_tree"', self._pane("settings"))
        self.assertIn('href="/voice_settings"', self._pane("voice"))
        self.assertIn('href="/utterance_log"', self._pane("voice"))
        self.assertIn('href="/detection_log"', self._pane("logs"))
        self.assertNotIn('href="/behavior_rules"', self._pane("voice"))

    def test_group_headings_exist(self):
        for tab in ("voice", "settings", "logs"):
            self.assertIn('class="grp-title"', self._pane(tab), tab)
        st = self._pane("settings")
        # 設定タブの順序: 頭部ディスプレイ -> タブレットOCR -> 動作ルール -> メンテナンス
        order = [st.index('id="%s"' % g) for g in ("grpDisplay", "grpOcr", "grpRules", "grpMaint")]
        self.assertEqual(order, sorted(order))
        # 音声タブの先頭は全体ON/OFF、その次に音量・BGM、実験・デモは最後
        vp = self._pane("voice")
        self.assertLess(vp.index('id="voiceMasterBlock"'), vp.index('id="voiceLevelsBlock"'))
        self.assertLess(vp.index('id="voiceLevelsBlock"'), vp.index('id="grpVoiceDemo"'))

    def test_bgm_card_merged_into_volume_card(self):
        """旧BGMカードは音量カードの一部になった(BGMの音量が2枚のカードに分かれていない)。"""
        vp = self._pane("voice")
        card = vp[vp.index('id="voiceLevelsBlock"'):vp.index('id="grpVoiceLinks"')]
        for i in ("bgmBlock", "bgmStatus", "bgmToggleBtn", "bgmVolume", "bgmDuckVolume", "bgmSaveBtn", "bgmHint"):
            self.assertIn('id="%s"' % i, card, i)

    def test_dangerous_ops_are_separated(self):
        mp = self._pane("map")
        # 地図リセットは専用の区画(danger-zone)で、作成開始/停止とは別行
        i = mp.index('id="mapResetBtn"')
        self.assertIn("danger-zone", mp[i - 120:i])
        self.assertLess(mp.index('id="mapStartBtn"'), i)
        # 速度設定は折りたたみの中(運転カードの主役は「自律運転に戻す」)
        self.assertLess(mp.index('id="resumeBtn"'), mp.index('id="speedFold"'))
        self.assertLess(mp.index('id="speedFold"'), mp.index('id="speedSaveBtn"'))
        self.assertIn('class="btns danger-zone"', mp)
        self.assertIn('window.confirm', self.pages["HTML"])

    def test_header_badges_and_status_wiring(self):
        html = self.pages["HTML"]
        for i in ("voiceOffChip", "softEstopChip", "estopBadge", "menuToggle", "logoutBtn"):
            self.assertIn('id="%s"' % i, html)
        # 状態バッジ(動作中/停止中/要確認)を各カードが持つ
        for i in ("voiceMasterBadge", "vlBadge", "surveyBadge", "ocrMockBadge", "ocrBadge", "driveBadge"):
            self.assertIn('id="%s"' % i, html, i)
        self.assertIn("function tick()", html)
        self.assertIn("SURVEY_ACTIVE_HOLD:", html)
        # スマホ幅でヘッダー右側が折り返す(はみ出さない)
        self.assertRegex(html, r"\.header-right \{ flex:1 1 100%; flex-wrap:wrap; \}")

    def test_estop_dock_stays_fixed_and_topmost(self):
        html = self.pages["HTML"]
        css = re.search(r"<style>(.*?)</style>", html, re.S).group(1)
        self.assertRegex(css, r"#estopDock \{[^}]*position:fixed[^}]*z-index:2000")
        # 他のどの要素も2000以上のz-indexを持たない(ドックが常に最前面)
        self.assertEqual([z for z in re.findall(r"z-index:\s*(\d+)", html) if int(z) > 2000], [])
        self.assertGreater(html.index('id="estopDock"'), html.index("</main>"))
        self.assertEqual(html.count('id="estopBtn"'), 1)

    def test_no_duplicate_ocr_link_card_in_settings(self):
        """OCRの入口は、OCRカード内の「映像を表示」ボタン(ocrLink。2026-10-21〜、確認画面8091は1台目の内側だけなので、映像は管理画面が中継)に一本化。設定タブにリンクカードの重複を置かない。"""
        st = self._pane("settings")
        self.assertNotIn('data-ocr="1"', st)
        self.assertIn('id="ocrLink"', st)


if __name__ == "__main__":
    unittest.main()
