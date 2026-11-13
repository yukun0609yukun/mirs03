"""ダッシュボードの値編集と自動更新(ポーリング)の競合を防ぐ共通ヘルパー NKField(2026-10-10、docs/dashboard_field_sync.md)のテスト。

確かめること(ロボット・ROS・2台目・ラズパイには触れない。APIはスタブのローカルHTTPサーバー、画面は実ブラウザ=Playwright/Edge):
1. 入力中(フォーカス・IME変換中・スライダーのドラッグ中・未送信の編集)に定期取得が来ても、入力値が上書きされない
2. 「適用/保存」で送るのは触った欄だけ。送信後はサーバーの値を読み直して一致を確認し、一致しなければ警告を出して入力値を残す
3. 送信前に始まった古い取得結果で、画面が一瞬古い値に戻らない(世代番号で捨てる)
4. 範囲外の値は入力時に表示される
5. 非表示のタブではポーリングを止め、戻ったらすぐ取り直す
6. 全ての <input>/<select>/<textarea> が共通ヘルパーで保護されている(例外は理由つきの明示リストだけ)
7. 画面ごとの具体的な入力: 速度設定・音量(スライダー)・BGM・ピン名(日本語入力のEnter)・地図の向き・走行可能範囲・
   アンケート/OCRの切替・行動ルール・ビヘイビアツリー・呼びかけ音声設定・Stage1のPIDゲイン・スマホ版

実行: python ros2_ws/src/nokosahen_core/test/test_dashboard_field_sync.py (普通のpythonで。-I は付けない)
(playwright / Edge / node が無い環境ではその部分だけスキップ)
"""
import ast
import copy
import importlib.util
import json
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

PKG = Path(__file__).resolve().parents[1]
SRC = PKG / "nokosahen_core"
sys.path.insert(0, str(PKG))
NODE = shutil.which("node")
_TMP = tempfile.mkdtemp(prefix="field_sync_test_")

try:
    from playwright.sync_api import sync_playwright
    HAVE_PW = True
except Exception:  # noqa: BLE001
    HAVE_PW = False


# ======================================================================
# ページの組み立て(rclpy無しでノードのHTML定数を取り出し、サーバーと同じプレースホルダ展開をする)
# ======================================================================
def _load(name):
    spec = importlib.util.spec_from_file_location("fs_" + name, SRC / (name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_MV, _NL, _AU = _load("map_view"), _load("nav_layout"), _load("accounts_ui")


def _consts(filename, names):
    ns = {
        "LOGO_DATA_URI": "", "MAP_VIEW_JS": _MV.MAP_VIEW_JS, "NAV_CSS": _NL.NAV_CSS, "NAV_JS": _NL.NAV_JS,
        "NK_FIELD_JS": _NL.NK_FIELD_JS, "link_cards_html": _NL.link_cards_html,
        "nav_drawer_html": _NL.nav_drawer_html, "nav_header_html": _NL.nav_header_html,
        "ACCOUNTS_CARD_HTML": _AU.ACCOUNTS_CARD_HTML, "ACCOUNTS_CARD_JS": _AU.ACCOUNTS_CARD_JS,
        "SHUTDOWN_CARD_HTML": _load("system_shutdown_ui").SHUTDOWN_CARD_HTML,
        "SHUTDOWN_CARD_JS": _load("system_shutdown_ui").SHUTDOWN_CARD_JS,
    }
    tree = ast.parse((SRC / filename).read_text(encoding="utf-8"))
    out = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") in names:
            html = eval(compile(ast.Expression(node.value), "x", "eval"), ns)  # noqa: S307 (このリポジトリ自身のソースの式のみ)
            out[node.targets[0].id] = html.replace("__NK_FIELD_JS__", _NL.NK_FIELD_JS).replace("__MAP_VIEW_JS__", _MV.MAP_VIEW_JS)
    return out


def build_pages():
    from nokosahen_core.behavior_rules_page import build_behavior_rules_html
    from nokosahen_core.behavior_tree_page import build_behavior_tree_html
    from nokosahen_core.voice_settings_page import build_voice_settings_html
    full = _consts("full_dashboard_node.py", ("HTML", "MOBILE_HTML"))
    s2 = _consts("stage2_dashboard_node.py", ("HTML", "MOBILE_HTML"))
    s1 = _consts("stage1_dashboard_node.py", ("HTML",))
    return {
        "full": full["HTML"], "mobile": full["MOBILE_HTML"],
        "stage2": s2["HTML"], "stage2_mobile": s2["MOBILE_HTML"], "stage1": s1["HTML"],
        "rules": build_behavior_rules_html().decode("utf-8"),
        "tree": build_behavior_tree_html().decode("utf-8"),
        "voice": build_voice_settings_html().decode("utf-8"),
    }


PAGES = build_pages()
ROUTES = {"full": "/", "mobile": "/mobile", "stage2": "/stage2", "stage2_mobile": "/stage2_mobile", "stage1": "/stage1",
          "rules": "/behavior_rules", "tree": "/behavior_tree", "voice": "/voice_settings"}


# ======================================================================
# APIスタブ
# ======================================================================
class Stub:
    def __init__(self):
        self.reset()

    def reset(self):
        self.log = []                 # (method, path, body)
        self.ignore = set()           # 受け取っても値を変えないAPI(反映されない状況を作る)
        self.fail = set()             # 500を返すAPI
        self.delay = {}               # path -> 応答までの秒数
        self.speed = {"manual_linear_mps": 0.8, "manual_angular_rps": 1.5, "auto_linear_mps": 0.75, "auto_angular_rps": 0.75}
        self.limits = {"manual_linear_min": 0.01, "manual_linear_max": 1.2, "manual_angular_min": 0.01, "manual_angular_max": 4.0,
                       "auto_linear_max": 1.1, "auto_angular_max": 3.0}
        self.nav = {"nav_max_speed_mps": 0.7, "nav_max_speed_source": "default", "nav_speed_default": 0.7,
                    "read_back": {"running": False, "nodes": {}}}
        self.bgm = {"enabled": True, "volume": 0.5, "duck_volume": 0.2}
        self.levels = {"speech_volume": 1.0, "bgm_volume": 0.5, "bgm_duck_volume": 0.2}
        self.master = True
        self.survey_mode = "customer"
        self.survey_delay = 0.0
        self.ocr_profile = "demo"
        self.ocr_delay = 0.0
        self.pins = [{"id": "p1", "name": "入口", "color": "#e11d48", "x": 1.0, "y": 2.0, "yaw_deg": 0.0, "distance_m": 1.5}]
        self.map_angle = 0.0
        self.boundary = {"enabled": False}
        self.gains = {k: 1.0 for k in ("rkp_fwd", "rki_fwd", "rkd_fwd", "rkp_bwd", "rki_bwd", "rkd_bwd",
                                       "lkp_fwd", "lki_fwd", "lkd_fwd", "lkp_bwd", "lki_bwd", "lkd_bwd")}
        self.rules = {"version": 1, "description": "", "min_hold_seconds": 10, "rules": [
            {"id": "r1", "enabled": True, "name": "昼", "priority": 5, "when": {}, "then": {"policy": ["normal"], "params": {}}}]}
        self.rules_director = None    # directorが読んでいるルール(None=保存内容と同じ)
        self.tree = {"version": 1, "min_hold_seconds": 10, "enabled": True, "root": {
            "type": "selector", "children": [{"type": "wait", "params": {"seconds": 30}}]}}
        self.vs_settings = {"include_menu_names": True, "include_remaining": True, "temperature": 0.7, "num_predict": 200,
                            "max_chars": 0, "ollama_timeout_sec": 60.0, "min_repeat_sec": 30.0, "persona_prompt": "あなたは看板娘です"}
        self.vs_interval = 60

    def note(self, method, path, body):
        self.log.append((method, path, body))

    def posts(self, path):
        return [b for (m, p, b) in self.log if m == "POST" and p == path]

    # ---- 速度
    def speed_payload(self):
        p = dict(self.speed)
        p["limits"] = dict(self.limits)
        p["max_linear_mps"] = 1.1
        return p

    def set_speed(self, body):
        for k, v in body.items():
            if k not in self.speed or not isinstance(v, (int, float)):
                continue
            base = k.replace("_mps", "").replace("_rps", "")
            hi = self.limits[base + "_max"]
            lo = self.limits.get(base + "_min", 0.01)
            self.speed[k] = max(lo, min(hi, float(v)))

    # ---- 各ページ用の文書
    def rules_payload(self):
        return {"path": "/tmp/rules.json", "exists": True, "backup_exists": False, "valid": True, "errors": [],
                "sig": "sig-%d" % len(json.dumps(self.rules)), "document": copy.deepcopy(self.rules),
                "meta": {"policies": ["normal", "stay", "quiet"], "conflicts": [], "weekdays": ["mon", "tue"],
                         "crowd_levels": ["calm", "crowded"], "speed_scale_min": 0.3, "speed_scale_max": 1.0,
                         "announce_interval_min": 10, "announce_interval_max": 600, "max_rules": 50, "default_min_hold_seconds": 10},
                "pin_warnings": []}

    def rules_live(self):
        doc = self.rules_director or self.rules
        rs = doc["rules"]
        return {"status": "ok", "state": {"rule_count": len(rs), "enabled_rule_count": len([r for r in rs if r.get("enabled")]),
                                          "active_rule_id": None, "policy": "normal", "min_hold_seconds": 10, "rules_file": "/tmp/rules.json"},
                "dashboard_rules_file": "/tmp/rules.json", "path_mismatch": False, "message": ""}

    def tree_payload(self):
        types = [
            {"type": "selector", "kind": "composite", "icon": "S", "label": "セレクター", "desc": "", "children": "many", "params": []},
            {"type": "wait", "kind": "action", "icon": "W", "label": "待機", "desc": "", "children": "none",
             "params": [{"key": "seconds", "type": "number", "label": "秒", "min": 0, "max": 600, "required": True}]},
        ]
        return {"exists": True, "valid": True, "errors": [], "warnings": [], "pin_warnings": [], "path": "/tmp/tree.json",
                "sig": "t-%d" % len(json.dumps(self.tree)), "document": copy.deepcopy(self.tree), "default_document": copy.deepcopy(self.tree),
                "examples": [], "live": {"status": "ok", "sync": "synced", "state": {"statuses": {}, "active_leaves": [], "node_count": 2, "user_active": True}},
                "meta": {"types": types, "kinds": [["composite", "制御"], ["action", "動作"]], "safety_layers": []}}

    def vs_payload(self):
        lim = {"temperature": {"min": 0.0, "max": 2.0}, "num_predict": {"min": 32, "max": 2000}, "max_chars": {"min": 0, "max": 500},
               "ollama_timeout_sec": {"min": 5.0, "max": 300.0}, "min_repeat_sec": {"min": 0.0, "max": 600.0}}
        return {"ok": True, "configured": True,
                "announce": {"announce_min_interval_seconds": self.vs_interval, "limits": {"min": 10, "max": 600}, "path": "/tmp/a.json", "exists": True, "valid": True},
                "voice": {"ok": True, "settings": dict(self.vs_settings), "limits": lim, "sources": {}, "defaults": dict(self.vs_settings),
                          "model": "m", "settings_file": "/tmp/v.json"}}


class Handler(BaseHTTPRequestHandler):
    stub = None
    pages = None

    def log_message(self, *a):
        pass

    def _send(self, code, obj, ctype="application/json"):
        data = obj.encode("utf-8") if isinstance(obj, str) else json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype + "; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(data)
        except Exception:  # noqa: BLE001
            pass

    def _body(self):
        n = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(n) if n else b""
        try:
            return json.loads(raw.decode("utf-8")) if raw else {}
        except Exception:  # noqa: BLE001
            return {}

    def do_GET(self):  # noqa: N802
        s, path = self.stub, urlparse(self.path).path
        for name, route in ROUTES.items():
            if path == route:
                return self._send(200, self.pages[name], "text/html")
        d = s.delay.get(path)
        if d:
            time.sleep(d)
        s.note("GET", path, None)
        if path == "/api/whoami":
            return self._send(200, {"account": "admin", "restricted": False})
        if path == "/api/speed_settings":
            return self._send(200, s.speed_payload())
        if path == "/api/nav_speed":
            return self._send(200, s.nav)
        if path == "/api/bgm":
            return self._send(200, {"ok": True, "configured": True, "status": dict(s.bgm, playing=False, available=True)})
        if path == "/api/voice_master":
            return self._send(200, {"ok": True, "configured": True, "status": {"enabled": s.master, "persisted": True}})
        if path == "/api/voice_levels":
            return self._send(200, {"ok": True, "configured": True, "status": dict(s.levels, persisted=True)})
        if path == "/api/survey/state":
            return self._send(200, {"ok": True, "configured": True, "state": {
                "mode": s.survey_mode, "count": 3, "active": False, "receiver": True, "session_state": "idle",
                "last_activity_age_s": 5, "condition": "A"}})
        if path == "/api/ocr/state":
            return self._send(200, {"ok": True, "configured": True, "state": {
                "profile": s.ocr_profile, "profile_supported": True, "profiles": {}, "readings": [], "updated_age_s": 1}})
        if path == "/api/pins":
            return self._send(200, {"pins": copy.deepcopy(s.pins), "max": 200, "mode": "idle", "loaded_map": "m1"})
        if path == "/api/map_view":
            return self._send(200, {"angle_deg": s.map_angle})
        if path == "/api/boundary":
            return self._send(200, s.boundary)
        if path == "/api/pid_gains":
            return self._send(200, s.gains)
        if path == "/api/state":
            return self._send(200, {"soft_estop_engaged": False, "behavior_age_s": 1, "mode": "NORMAL"})
        if path == "/api/behavior_rules":
            return self._send(200, s.rules_payload())
        if path == "/api/behavior_rules/state":
            return self._send(200, s.rules_live())
        if path == "/api/behavior_rules/preview":
            return self._send(200, {"ok": True, "segments": [], "enabled_rule_count": 1, "hours": 24, "now": "2026-10-10T10:00:00", "min_hold_seconds": 10})
        if path == "/api/behavior_tree":
            return self._send(200, s.tree_payload())
        if path == "/api/behavior_tree/state":
            return self._send(200, s.tree_payload()["live"])
        if path == "/api/voice_settings":
            return self._send(200, s.vs_payload())
        return self._send(404, {"error": "stub: not found"})

    def do_POST(self):  # noqa: N802
        s, path = self.stub, urlparse(self.path).path
        body = self._body()
        s.note("POST", path, body)
        d = s.delay.get(path)
        if d:
            time.sleep(d)
        if path in s.fail:
            return self._send(500, {"ok": False, "message": "stub: 失敗させました"})
        if path == "/api/speed_settings":
            if "speed" not in s.ignore:
                s.set_speed(body)
            return self._send(200, dict(s.speed_payload(), ok=True))
        if path == "/api/bgm":
            if "bgm" not in s.ignore:
                for k in ("enabled", "volume", "duck_volume"):
                    if k in body:
                        s.bgm[k] = body[k]
            return self._send(200, {"ok": True, "configured": True, "status": dict(s.bgm)})
        if path == "/api/voice_levels":
            if "levels" not in s.ignore:
                for k, v in body.items():
                    if k in s.levels and isinstance(v, (int, float)):
                        s.levels[k] = float(v)
            return self._send(200, {"ok": True, "configured": True, "status": dict(s.levels)})
        if path == "/api/voice_master":
            if "master" not in s.ignore:
                s.master = bool(body.get("enabled"))
            return self._send(200, {"ok": True, "configured": True, "status": {"enabled": s.master}})
        if path == "/api/survey/mode":
            target = body.get("mode")

            def apply():
                time.sleep(s.survey_delay)
                if "survey" not in s.ignore:
                    s.survey_mode = target
            threading.Thread(target=apply, daemon=True).start()
            return self._send(200, {"ok": True})
        if path == "/api/ocr/profile":
            target = body.get("camera_profile")

            def apply2():
                time.sleep(s.ocr_delay)
                s.ocr_profile = target
            threading.Thread(target=apply2, daemon=True).start()
            return self._send(200, {"ok": True})
        if path == "/api/pins/update":
            for p in s.pins:
                if p["id"] == body.get("id") and "name" in body:
                    p["name"] = body["name"]
            return self._send(200, {"ok": True, "message": "更新しました"})
        if path == "/api/map_view/set":
            if "mapview" not in s.ignore:
                s.map_angle = float(body.get("angle_deg", 0))
            return self._send(200, {"ok": True, "angle_deg": s.map_angle})
        if path == "/api/boundary/set":
            if "boundary" not in s.ignore:
                s.boundary = dict(body, enabled=True)
            return self._send(200, {"ok": True, "message": "走行可能範囲を設定しました"})
        if path == "/api/pid_gains":
            if "gains" not in s.ignore:
                s.gains.update({k: float(v) for k, v in body.items() if k in s.gains})
            return self._send(200, {"ok": True})
        if path == "/api/behavior_rules/save":
            doc = body.get("document")
            if "rules" not in s.ignore:
                s.rules = copy.deepcopy(doc)
            return self._send(200, {"ok": True, "message": "保存しました", "warnings": []})
        if path == "/api/behavior_tree":
            if "tree" not in s.ignore:
                s.tree = copy.deepcopy(body.get("document"))
            return self._send(200, {"ok": True, "message": "保存しました"})
        if path == "/api/voice_settings/save":
            if "vs" not in s.ignore:
                a = body.get("announce")
                if a:
                    s.vs_interval = a["announce_min_interval_seconds"]
                s.vs_settings.update(body.get("voice") or {})
            return self._send(200, {"ok": True, "message": "保存しました"})
        return self._send(200, {"ok": True})


# ======================================================================
# 1. 静的な確認(ブラウザ不要)
# ======================================================================
class StaticTest(unittest.TestCase):
    def test_all_page_scripts_parse(self):
        if not NODE:
            self.skipTest("node がありません")
        for name, html in PAGES.items():
            scripts = re.findall(r"<script>(.*?)</script>", html, re.S)
            self.assertTrue(scripts, name)
            for i, body in enumerate(scripts):
                p = Path(_TMP) / f"{name}_{i}.js"
                p.write_text(body, encoding="utf-8")
                r = subprocess.run([NODE, "--check", str(p)], capture_output=True, text=True)
                self.assertEqual(r.returncode, 0, f"{name} script#{i}: {r.stderr}")

    def test_helper_present_in_every_page_and_before_use(self):
        for name, html in PAGES.items():
            self.assertIn("window.NKField = {", html, name)
            self.assertNotIn("__NK_FIELD_JS__", html, name)
            self.assertLess(html.index("window.NKField = {"), html.index("NKField.guard"), name)

    def test_nav_js_contains_helper_and_dock_untouched(self):
        self.assertIn("window.NKField = {", _NL.NAV_JS)
        nav = (SRC / "nav_layout.py").read_text(encoding="utf-8")
        self.assertRegex(nav, r"#estopDock\s*\{[^}]*position:\s*fixed[^}]*z-index:\s*2000")

    def test_input_polls_go_through_nkfield_poll(self):
        # 入力欄に書き込む定期取得を、素の setInterval で動かしていないこと(非表示で止まる/重ならない共通の仕組みを通す)
        banned = ("loadBgm", "loadVoiceLevels", "loadVoiceMaster", "loadSurvey", "loadOcr", "refreshMapList", "fetchBoundary",
                  "pollMapView", "pollPins", "loadSpeedSettings", "loadNavSpeed", "loadGains", "loadPins", "fetchZones")
        for name, html in PAGES.items():
            for fn in banned:
                self.assertIsNone(re.search(r"setInterval\(\s*(\(\)\s*=>\s*)?%s\b" % fn, html), f"{name}: setInterval({fn}")

    def test_line_endings_preserved(self):
        for fn in ("full_dashboard_node.py", "nav_layout.py", "map_view.py", "stage2_dashboard_node.py", "stage1_dashboard_node.py",
                   "behavior_rules_page.py", "dashboard_common.py"):
            b = (SRC / fn).read_bytes()
            self.assertEqual(b.count(b"\r\n"), b.count(b"\n"), fn + ": CRLFを維持")
        for fn in ("behavior_tree_page.py", "voice_settings_page.py"):
            b = (SRC / fn).read_bytes()
            self.assertEqual(b.count(b"\r\n"), 0, fn + ": LFを維持")

    # ---- 静的に確認できるページの入力欄(ブラウザで開かない画面)は、全て理由つきの例外リストに載っていること
    STATIC_EXEMPT = {
        # ログイン画面: 送信するだけの2欄でポーリングは無い
        "_LOGIN_HTML": {"username", "password"},
        # 一覧・分析ページ: 絞り込みの選択欄。ポーリングは表を作り直すだけで、これらの欄の値は書かない
        "_CONSUMPTION_GRAPH_HTML": {"sourceFilter"},
        "_UTTERANCE_LOG_HTML": {"searchBox"},
        "_RECOMMENDATION_EFFECTIVENESS_HTML": {"windowMinutes"},
        "_ACTIVITY_REPORT_HTML": {"periodSelect"},
        # ステップ応答: 1回の実行パラメータ(押した時に読むだけ。ポーリングは実行状況のみ)
        "_STEP_RESPONSE_HTML": {"runDirection", "runTarget", "runTargetDeg", "runDuration"},
        # 地図エディタ: 入力欄なし(クリック操作のみ)
        "_MAP_EDITOR_HTML": set(),
    }

    def test_static_pages_inputs_are_all_listed_as_exempt(self):
        tree = ast.parse((SRC / "dashboard_common.py").read_text(encoding="utf-8"))
        found = {}
        for node in tree.body:
            if isinstance(node, ast.Assign):
                nm = node.targets[0].id if isinstance(node.targets[0], ast.Name) else ""
                base = node.value
                while isinstance(base, ast.Call) and isinstance(base.func, ast.Attribute):
                    base = base.func.value   # .replace(...)等の連鎖を遡る
                if nm.endswith("_HTML") and isinstance(base, ast.Constant) and isinstance(base.value, str):
                    markup = re.sub(r"<script>.*?</script>", "", base.value, flags=re.S)
                    found[nm] = set(re.findall(r'<(?:input|select|textarea)\b[^>]*\bid="([^"]+)"', markup))
        for nm, ids in found.items():
            self.assertLessEqual(ids, self.STATIC_EXEMPT.get(nm, set()), f"{nm}: 例外リストに無い入力欄 {ids - self.STATIC_EXEMPT.get(nm, set())}")
        for nm in self.STATIC_EXEMPT:
            self.assertIn(nm, found)
        # step_responseの結果表のチェックボックス(JSで作る・選択を保持したまま再描画する)も例外として明示
        self.assertIn("selectedIds", (SRC / "dashboard_common.py").read_text(encoding="utf-8"))

    def test_nokosahen_pages_without_ids_use_helper_classes(self):
        # idが無い入力欄(模擬POSの7つの数字欄)は、クラスで一括登録していること
        self.assertIn('NKField.guardAll(".ocrm-in")', PAGES["full"])

    def test_keepout_zones_republished_periodically(self):
        src = (SRC / "dashboard_common.py").read_text(encoding="utf-8")
        i = src.index("class KeepOutZoneController")
        j = src.index("class FrontierExplorerController")
        self.assertIn("create_timer(5.0, self._publish)", src[i:j])


# ======================================================================
# 2. ブラウザでの動作
# ======================================================================
@unittest.skipUnless(HAVE_PW, "playwright がありません")
class BrowserBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.stub = Stub()
        Handler.stub = cls.stub
        Handler.pages = PAGES
        cls.srv = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        cls.srv.daemon_threads = True
        cls.port = cls.srv.server_address[1]
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()
        cls.pw = sync_playwright().start()
        try:
            cls.browser = cls.pw.chromium.launch(channel="msedge")
        except Exception as e:  # noqa: BLE001
            cls.pw.stop()
            cls.srv.shutdown()
            raise unittest.SkipTest(f"Edge を起動できません: {e}")

    @classmethod
    def tearDownClass(cls):
        try:
            cls.browser.close()
            cls.pw.stop()
        finally:
            cls.srv.shutdown()

    def setUp(self):
        self.stub.reset()
        self.ctx = self.browser.new_context(viewport={"width": 1280, "height": 900})
        self.ctx.route(re.compile(r"https?://fonts\.(googleapis|gstatic)\.com/.*"), lambda r: r.abort())
        self.page = self.ctx.new_page()
        self.errors = []
        self.page.on("pageerror", lambda e: self.errors.append(str(e)))

    def tearDown(self):
        self.ctx.close()

    def open(self, key, hash_="", wait="() => window.NKField"):
        self.page.goto(f"http://127.0.0.1:{self.port}{ROUTES[key]}{hash_}")
        self.page.wait_for_function(wait, timeout=8000)
        self.page.evaluate("document.querySelectorAll('details').forEach(d => { d.open = true; })")

    def val(self, sel):
        return self.page.input_value(sel)

    def blur(self):
        self.page.evaluate("document.activeElement && document.activeElement.blur()")

    def text(self, sel):
        return self.page.text_content(sel) or ""

    def wait_text(self, sel, needle, timeout=15000):
        self.page.wait_for_function("([s, n]) => (document.querySelector(s).textContent || '').includes(n)", arg=[sel, needle], timeout=timeout)

    def settle(self, ms=300):
        self.page.wait_for_timeout(ms)


class SpeedTest(BrowserBase):
    def test_edit_survives_poll_and_only_touched_field_is_sent(self):
        self.open("full", "#tab-map", "() => document.getElementById('manualLinear') && document.getElementById('manualLinear').value !== ''")
        self.assertEqual(self.val("#manualLinear"), "0.8")
        self.page.fill("#autoAngular", "0.5")                     # 触った欄は1つだけ
        self.stub.speed["manual_linear_mps"] = 0.9                # 他の端末が手動直進を変えた
        self.page.evaluate("loadSpeedSettings()")                 # 定期取得
        self.assertEqual(self.val("#autoAngular"), "0.5", "入力中の値が上書きされた")
        self.assertEqual(self.val("#manualLinear"), "0.9", "触っていない欄は最新に追従する")
        self.blur()
        self.stub.speed["auto_angular_rps"] = 1.0                 # 他の端末が同じ欄を変えても、未保存の編集は消さない
        self.page.evaluate("loadSpeedSettings()")
        self.assertEqual(self.val("#autoAngular"), "0.5", "フォーカスが外れても未送信(dirty)の編集は残る")
        self.page.click("#speedSaveBtn")
        self.wait_text("#speedSaveHint", "保存しました")
        self.assertEqual(self.stub.posts("/api/speed_settings")[-1], {"auto_angular_rps": 0.5}, "触った欄だけ送る")
        self.assertEqual(self.stub.speed["manual_linear_mps"], 0.9, "触っていない欄を古い値で上書きしていない")
        self.assertEqual(self.val("#autoAngular"), "0.5")
        self.stub.speed["auto_angular_rps"] = 0.9                 # 保存が済めば、また最新に追従する
        self.page.evaluate("loadSpeedSettings()")
        self.assertEqual(self.val("#autoAngular"), "0.9")

    def test_not_applied_on_server_warns_and_keeps_typed_value(self):
        self.open("full", "#tab-map", "() => document.getElementById('manualLinear').value !== ''")
        self.stub.ignore.add("speed")
        self.page.fill("#autoLinear", "0.6")
        self.page.click("#speedSaveBtn")
        self.wait_text("#speedSaveHint", "反映されていません")
        self.assertIn("サーバー 0.75", self.text("#speedSaveHint"))
        self.assertEqual(self.val("#autoLinear"), "0.6", "反映されなかった入力値は残す")
        self.assertTrue(self.page.evaluate("NKField.isDirty('autoLinear')"))
        self.page.evaluate("loadSpeedSettings()")                 # その後の定期取得でも入力値は消えない
        self.assertEqual(self.val("#autoLinear"), "0.6")

    def test_out_of_range_note_and_clamp_message(self):
        self.open("full", "#tab-map", "() => document.getElementById('manualLinear').value !== ''")
        self.page.fill("#autoLinear", "5")
        note = self.page.text_content("#autoLinear + .nk-note")
        self.assertIn("範囲は", note)
        self.assertIn("1.1", note)
        self.page.click("#speedSaveBtn")
        self.wait_text("#speedSaveHint", "丸め")
        self.assertEqual(self.stub.speed["auto_linear_mps"], 1.1)
        self.assertEqual(self.val("#autoLinear"), "1.1", "丸められた値(サーバーの値)に更新される")

    def test_stale_poll_started_before_save_does_not_revert(self):
        self.open("full", "#tab-map", "() => document.getElementById('manualLinear').value !== ''")
        self.page.evaluate("""() => {
          const el = document.getElementById('manualAngular'); window.__w = [];
          const d = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value');
          Object.defineProperty(el, 'value', { get() { return d.get.call(this); }, set(v) { window.__w.push(String(v)); d.set.call(this, v); } });
        }""")
        self.page.fill("#manualAngular", "2.5")
        self.stub.delay["/api/speed_settings"] = 0            # 取得は遅い応答(保存より前に始まり、保存の後に着く)
        self.page.evaluate("window.__p = loadSpeedSettings()")
        self.page.click("#speedSaveBtn")
        self.wait_text("#speedSaveHint", "保存しました")
        self.page.evaluate("window.__p")
        self.assertEqual(self.val("#manualAngular"), "2.5")
        self.assertNotIn("1.5", self.page.evaluate("window.__w"), "古い値(1.5)が一瞬でも書かれた")

    def test_mobile_and_stage2_send_only_touched(self):
        for key, hash_ in (("mobile", ""), ("stage2", ""), ("stage2_mobile", "")):
            self.stub.reset()
            self.open(key, hash_, "() => document.getElementById('manualLinear') && document.getElementById('manualLinear').value !== ''")
            self.page.fill("#manualAngular", "2")
            self.page.evaluate("loadSpeedSettings()")
            self.assertEqual(self.val("#manualAngular"), "2", key)
            self.page.click("#speedSaveBtn")
            self.wait_text("#speedSaveHint", "保存しました")
            self.assertEqual(self.stub.posts("/api/speed_settings")[-1], {"manual_angular_rps": 2}, key)

    def test_nav_speed_shows_default_not_a_stale_applied_value_when_nav2_is_down(self):
        # Nav2が再起動されると、最後に反映した値は効かなくなる(既定値に戻る)。その古い値を「現在値」として見せない
        self.stub.nav = {"nav_max_speed_mps": 1.0, "nav_max_speed_source": "live", "nav_speed_default": 0.7,
                         "read_back": {"running": False, "nodes": {}}}
        self.open("full", "#tab-map", "() => document.getElementById('navSpeed').value !== ''")
        self.assertEqual(self.val("#navSpeed"), "0.7")
        self.assertIn("もう一度", self.text("#navSpeedInfo"))

    def test_nav_and_all_speed_fields_are_guarded(self):
        self.open("full", "#tab-map")
        self.page.fill("#navSpeed", "0.4")
        self.page.evaluate("loadNavSpeed()")
        self.assertEqual(self.val("#navSpeed"), "0.4")
        self.page.fill("#allSpeed", "3")
        self.assertIn("範囲は", self.page.text_content("#allSpeed + .nk-note"))


class HelperUnitTest(BrowserBase):
    def test_ime_composition_blocks_overwrite_until_end(self):
        self.open("full", "#tab-map")
        self.page.evaluate("document.getElementById('mapSaveName').focus()")
        self.page.evaluate("document.getElementById('mapSaveName').dispatchEvent(new CompositionEvent('compositionstart'))")
        self.blur()
        self.assertFalse(self.page.evaluate("NKField.set('mapSaveName', 'SERVER')"), "変換中なのに上書きされた")
        self.assertNotEqual(self.val("#mapSaveName"), "SERVER")
        self.page.evaluate("document.getElementById('mapSaveName').dispatchEvent(new CompositionEvent('compositionend'))")
        self.assertEqual(self.val("#mapSaveName"), "SERVER", "変換が終わったら保留した最新値を反映する")

    def test_real_ime_composition_via_cdp(self):
        self.open("full", "#tab-map")
        self.page.evaluate("document.getElementById('mapSaveName').focus()")
        cdp = self.ctx.new_cdp_session(self.page)
        cdp.send("Input.imeSetComposition", {"text": "にほん", "selectionStart": 3, "selectionEnd": 3})
        self.assertFalse(self.page.evaluate("NKField.set('mapSaveName', 'SERVER')"))
        self.assertEqual(self.val("#mapSaveName"), "にほん", "変換中の文字が上書きされた")
        cdp.send("Input.insertText", {"text": "日本"})
        self.assertFalse(self.page.evaluate("NKField.set('mapSaveName', 'SERVER2')"), "入力済み未送信(dirty/フォーカス)の値が上書きされた")
        self.assertEqual(self.val("#mapSaveName"), "日本")

    def test_enter_during_composition_is_not_enter(self):
        self.open("full")
        self.assertFalse(self.page.evaluate("NKField.isEnter({key:'Enter', isComposing:true, keyCode:229})"))
        self.assertFalse(self.page.evaluate("NKField.isEnter({key:'Enter', isComposing:false, keyCode:229})"))
        self.assertTrue(self.page.evaluate("NKField.isEnter({key:'Enter', isComposing:false, keyCode:13})"))

    def test_poll_pauses_while_hidden_and_resumes_on_visible(self):
        self.open("full")
        self.page.evaluate("""() => {
          window.__n = 0; window.__hidden = true;
          Object.defineProperty(document, 'hidden', { configurable: true, get() { return window.__hidden; } });
          NKField.poll(() => { window.__n++; }, 120);
        }""")
        self.settle(700)
        self.assertEqual(self.page.evaluate("window.__n"), 0, "非表示の間に動いた")
        self.page.evaluate("window.__hidden = false; document.dispatchEvent(new Event('visibilitychange'))")
        self.settle(60)
        self.assertGreaterEqual(self.page.evaluate("window.__n"), 1, "戻った時にすぐ取り直していない")

    def test_poll_does_not_overlap(self):
        self.open("full")
        n = self.page.evaluate("""async () => {
          let running = 0, maxr = 0, calls = 0;
          const p = NKField.poll(async () => { running++; maxr = Math.max(maxr, running); calls++; await new Promise((r) => setTimeout(r, 300)); running--; }, 50);
          await new Promise((r) => setTimeout(r, 800)); clearInterval(p.timer);
          return [maxr, calls];
        }""")
        self.assertEqual(n[0], 1)
        self.assertGreaterEqual(n[1], 2)

    def test_defer_runs_after_editing_ends(self):
        self.open("full", "#tab-map")
        self.page.evaluate("document.getElementById('mapSaveName').focus(); window.__ran = 0;")
        self.assertFalse(self.page.evaluate("NKField.defer(document.getElementById('mapSaveName').parentElement, () => { window.__ran++; })"))
        self.assertEqual(self.page.evaluate("window.__ran"), 0)
        self.blur()
        self.settle(100)
        self.assertEqual(self.page.evaluate("window.__ran"), 1)

    def test_set_options_keeps_open_select_and_skips_identical(self):
        self.open("full", "#tab-map")
        self.assertTrue(self.page.evaluate("NKField.setOptions('mapLoadSelect', ['a', 'b'])"))
        self.assertFalse(self.page.evaluate("NKField.setOptions('mapLoadSelect', ['a', 'b'])"), "中身が同じなのに作り直した")
        self.page.evaluate("document.getElementById('mapLoadSelect').value = 'b'; document.getElementById('mapLoadSelect').focus()")
        self.assertFalse(self.page.evaluate("NKField.setOptions('mapLoadSelect', ['a', 'b', 'c'])"), "操作中の選択欄を作り直した")
        self.blur()
        self.assertTrue(self.page.evaluate("NKField.setOptions('mapLoadSelect', ['a', 'b', 'c'])"))
        self.assertEqual(self.val("#mapLoadSelect"), "b", "選択が消えた")


class VoiceLevelsTest(BrowserBase):
    def open_voice(self):
        self.open("full", "#tab-voice", "() => document.getElementById('vlSpeechNum').value !== ''")

    def test_slider_drag_blocks_overwrite_and_pending_applies_after(self):
        self.open_voice()
        self.assertEqual(self.val("#vlSpeechNum"), "1")
        self.page.dispatch_event("#vlSpeech", "pointerdown")        # ドラッグ開始(まだ値は動かしていない)
        self.stub.levels["speech_volume"] = 2.0
        self.page.evaluate("loadVoiceLevels(false)")
        self.assertEqual(self.val("#vlSpeechNum"), "1", "ドラッグ中に上書きされた")
        self.page.dispatch_event("#vlSpeech", "pointerup")
        self.settle(50)
        self.page.evaluate("window.dispatchEvent(new Event('pointerup'))")
        self.assertEqual(self.val("#vlSpeechNum"), "2", "ドラッグが終わったら最新を反映する")

    def test_apply_sends_only_touched_and_verifies(self):
        self.open_voice()
        self.page.fill("#vlBgmNum", "0.8")
        self.stub.levels["speech_volume"] = 1.5                      # 他の端末が発話音量を変えた
        self.page.evaluate("loadVoiceLevels(false)")
        self.assertEqual(self.val("#vlBgmNum"), "0.8")
        self.assertEqual(self.val("#vlSpeechNum"), "1.5", "触っていない欄は追従する")
        self.page.click("#vlApplyBtn")
        self.wait_text("#vlHint", "反映しました")
        self.assertEqual(self.stub.posts("/api/voice_levels")[-1], {"bgm_volume": 0.8}, "触った欄だけ送る")
        self.assertEqual(self.stub.levels["speech_volume"], 1.5)
        self.assertEqual(self.val("#vlBgm"), "0.8")

    def test_not_applied_warns_and_keeps_input(self):
        self.open_voice()
        self.stub.ignore.add("levels")
        self.page.fill("#vlSpeechNum", "1.8")
        self.page.click("#vlApplyBtn")
        self.wait_text("#vlHint", "反映されていません")
        self.assertEqual(self.val("#vlSpeechNum"), "1.8")
        self.assertIn("反映されていません", self.page.text_content("#vlSpeechNum + .nk-note"))

    def test_stale_poll_started_before_apply_does_not_flicker(self):
        self.open_voice()
        self.page.evaluate("""() => {
          const el = document.getElementById('vlSpeechNum'); window.__w = [];
          const d = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value');
          Object.defineProperty(el, 'value', { get() { return d.get.call(this); }, set(v) { window.__w.push(String(v)); d.set.call(this, v); } });
        }""")
        self.page.fill("#vlSpeechNum", "1.2")
        self.stub.delay["/api/voice_levels"] = 0.6                   # 取得は遅い(適用より前に始まり、適用の後に着く)
        self.page.evaluate("window.__p = loadVoiceLevels(false)")
        self.stub.delay.pop("/api/voice_levels")
        self.page.click("#vlApplyBtn")
        self.wait_text("#vlHint", "反映しました")
        self.page.evaluate("window.__p")
        self.assertEqual(self.val("#vlSpeechNum"), "1.2")
        self.assertNotIn("1", self.page.evaluate("window.__w"), "古い値(1)が一瞬でも書かれた")

    def test_out_of_range_note(self):
        self.open_voice()
        self.page.fill("#vlSpeechNum", "9")
        self.assertIn("範囲は 0〜5", self.page.text_content("#vlSpeechNum + .nk-note"))

    def test_voice_master_switch_waits_for_reflection(self):
        self.open_voice()
        self.page.wait_for_function("document.getElementById('voiceMasterBtn').textContent.includes('OFF')")
        self.stub.ignore.add("master")
        self.page.click("#voiceMasterBtn")
        self.page.click("#voiceMasterYes")
        self.wait_text("#voiceMasterHint", "まだ反映が確認できません", 15000)


class BgmTest(BrowserBase):
    def test_pc_simple_inputs(self):
        self.open("full", "#tab-voice", "() => document.getElementById('bgmVolume').value !== ''")
        self.page.fill("#bgmVolume", "0.9")
        self.blur()
        self.stub.bgm["duck_volume"] = 0.3
        self.page.evaluate("loadBgm(false)")
        self.assertEqual(self.val("#bgmVolume"), "0.9", "フォーカスが外れた未保存の編集が上書きされた")
        self.assertEqual(self.val("#bgmDuckVolume"), "0.3")
        self.page.click("#bgmSaveBtn")
        self.wait_text("#bgmHint", "保存しました")
        self.assertEqual(self.stub.posts("/api/bgm")[-1], {"volume": 0.9}, "触った欄だけ送る")

    def test_mobile_simple_inputs_and_mismatch(self):
        self.open("mobile", "", "() => document.getElementById('bgmVolume').value !== ''")
        self.stub.ignore.add("bgm")
        self.page.fill("#bgmDuckVolume", "0.1")
        self.page.evaluate("loadBgm(false)")
        self.assertEqual(self.val("#bgmDuckVolume"), "0.1")
        self.page.click("#bgmSaveBtn")
        self.wait_text("#bgmHint", "反映されていません")
        self.assertEqual(self.val("#bgmDuckVolume"), "0.1")
        self.assertEqual(self.stub.posts("/api/bgm")[-1], {"duck_volume": 0.1})


class MapTest(BrowserBase):
    def open_map(self):
        self.open("full", "#tab-map", "() => document.getElementById('pinList') && document.querySelector('#pinList .pin-row')")

    def rename_box(self):
        self.page.click("#pinList .pin-head")                      # ピンを選ぶと操作ボタンが出る
        self.page.click("#pinList .pin-row button:has-text('名前変更')")
        self.page.wait_for_selector(".pin-edit-input")

    def test_pin_rename_survives_poll_and_ime_enter_does_not_save(self):
        self.open_map()
        self.rename_box()
        self.page.fill(".pin-edit-input", "新しい名前")
        self.stub.pins[0]["distance_m"] = 3.0
        self.page.evaluate("pollPins()")
        self.assertEqual(self.val(".pin-edit-input"), "新しい名前", "入力中のピン名が一覧の作り直しで消えた")
        self.page.evaluate("document.querySelector('.pin-edit-input').dispatchEvent(new KeyboardEvent('keydown', {key: 'Enter', isComposing: true, keyCode: 229, bubbles: true}))")
        self.settle(200)
        self.assertEqual(self.stub.posts("/api/pins/update"), [], "日本語の変換確定のEnterで保存された")
        self.page.press(".pin-edit-input", "Enter")
        self.settle(500)
        self.assertEqual(self.stub.posts("/api/pins/update"), [{"id": "p1", "name": "新しい名前"}])
        self.page.wait_for_function("document.querySelector('#pinList .pin-name').textContent === '新しい名前'")

    def test_rotation_failure_is_reported_and_not_silently_reverted(self):
        self.open_map()
        self.stub.fail.add("/api/map_view/set")
        self.page.dispatch_event("#mapRotSlider", "pointerdown")
        self.page.evaluate("document.getElementById('mapRotSlider').value = '40'; document.getElementById('mapRotSlider').dispatchEvent(new Event('input', {bubbles: true}))")
        self.wait_text("#mapRotHint", "この画面の向きだけ変わっています", 8000)
        self.page.evaluate("pollMapView()")
        self.assertEqual(self.val("#mapRotSlider"), "40", "送信に失敗したのに黙って元の角度へ戻った")

    def test_rotation_poll_does_not_override_while_dragging(self):
        self.open_map()
        self.page.dispatch_event("#mapRotSlider", "pointerdown")
        self.stub.map_angle = 90.0
        self.page.evaluate("mapRotLastEdit = 0; pollMapView()")
        self.settle(100)
        self.assertEqual(self.val("#mapRotSlider"), "0", "ドラッグ中に他の角度で上書きされた")
        self.page.evaluate("window.dispatchEvent(new Event('pointerup'))")
        self.page.evaluate("mapRotLastEdit = 0; pollMapView()")
        self.settle(100)
        self.assertEqual(self.val("#mapRotSlider"), "90")

    def test_rotation_applied_value_is_verified(self):
        self.open_map()
        self.stub.ignore.add("mapview")
        self.page.evaluate("document.getElementById('mapRotNum').value = '30'; document.getElementById('mapRotNum').dispatchEvent(new Event('change', {bubbles: true}))")
        self.wait_text("#mapRotHint", "反映されていません", 8000)

    def test_boundary_poll_discarded_while_dragging_corner_and_stale_result(self):
        self.open_map()
        self.stub.boundary = {"enabled": True, "center_x": 1, "center_y": 1, "width": 2, "height": 2, "yaw": 0, "min_x": 0, "max_x": 2, "min_y": 0, "max_y": 2}
        self.page.evaluate("mapBoundaryDrag = {id: 1}; currentBoundary = null; fetchBoundary()")
        self.settle(300)
        self.assertIsNone(self.page.evaluate("currentBoundary"), "角のドラッグ中に取得結果で上書きされた")
        self.page.evaluate("mapBoundaryDrag = null; fetchBoundary()")
        self.page.wait_for_function("currentBoundary && currentBoundary.enabled")
        # 送信前に始まった遅い取得は、送信(世代更新)の後に着いても捨てる
        self.stub.delay["/api/boundary"] = 0.5
        self.page.evaluate("window.__p = fetchBoundary()")
        self.page.evaluate("NKField.bump('boundary'); currentBoundary = {enabled: false}")
        self.page.evaluate("window.__p")
        self.assertFalse(self.page.evaluate("currentBoundary.enabled"), "古い取得結果で範囲が戻った")

    def test_boundary_verify_reports_mismatch(self):
        self.open_map()
        self.stub.boundary = {"enabled": True, "center_x": 9, "center_y": 9, "width": 2, "height": 2, "yaw": 0}
        self.page.evaluate("window.__r = boundaryVerify({center_x: 1, center_y: 1, width: 2, height: 2, yaw: 0}, document.getElementById('boundaryHint'))")
        self.assertFalse(self.page.evaluate("window.__r"))
        self.assertIn("反映されていません", self.text("#boundaryHint"))
        self.stub.boundary = {"enabled": True, "center_x": 1, "center_y": 1, "width": 2, "height": 2, "yaw": 0}
        self.assertTrue(self.page.evaluate("boundaryVerify({center_x: 1, center_y: 1, width: 2, height: 2, yaw: 0}, document.getElementById('boundaryHint'))"))


class SurveyOcrTest(BrowserBase):
    def open_settings(self):
        self.open("full", "#tab-settings", "() => document.getElementById('surveyMode').textContent.includes('お客さん')")

    def switch_survey(self):
        self.page.click("#surveyToSurveyBtn")
        self.page.click("#surveyConfirmYes")

    def test_survey_switch_confirms_after_raspi_delay_and_poll_does_not_revert(self):
        self.open_settings()
        self.stub.survey_delay = 2.0
        self.switch_survey()
        self.settle(500)
        self.page.evaluate("loadSurvey()")                           # 切り替え確認中の定期取得は表示を戻さない
        self.wait_text("#surveyHint", "切り替えました。", 15000)
        self.assertIn("アンケート表示中", self.text("#surveyMode"))

    def test_survey_switch_not_reflected_is_reported(self):
        self.open_settings()
        self.stub.ignore.add("survey")
        self.switch_survey()
        self.wait_text("#surveyHint", "まだ反映が確認できません", 30000)

    def test_ocr_profile_switch_waits_until_reflected(self):
        self.open("full", "#tab-settings", "() => document.getElementById('ocrProfile').textContent.includes('デモ')")
        self.stub.ocr_delay = 2.0
        self.page.click("#ocrToProdBtn")
        self.page.click("#ocrConfirmYes")
        self.wait_text("#ocrHint", "に切り替えました。", 20000)

    def test_mock_pos_inputs_are_kept(self):
        self.open_settings()
        self.page.fill(".ocrm-in >> nth=0", "777")
        self.page.evaluate("loadSurvey()")
        self.assertEqual(self.page.input_value(".ocrm-in >> nth=0"), "777")


class RulesPageTest(BrowserBase):
    def open_rules(self):
        self.open("rules", "", "() => document.querySelector('#rulesList .rule')")
        self.page.click("#rulesList [data-act='edit']")
        self.page.wait_for_selector("[data-f='priority']")

    def test_edit_survives_polls_and_range_note(self):
        self.open_rules()
        self.page.fill("[data-f='priority']", "5000")
        self.assertIn("範囲は", self.page.text_content("[data-f='priority'] + .nk-note"))
        self.page.fill("[data-f='priority']", "7")
        self.page.press("[data-f='priority']", "Tab")
        self.assertEqual(self.page.evaluate("draft.rules[0].priority"), 7)
        self.stub.pins.append({"id": "p2", "name": "窓", "color": "#2563eb", "x": 0, "y": 0, "yaw_deg": 0, "distance_m": 1})
        self.page.evaluate("loadPins(); pollLive()")
        self.settle(200)
        self.assertEqual(self.val("[data-f='priority']"), "7")

    def test_inert_while_saving_and_director_mismatch_is_reported(self):
        self.open_rules()
        self.stub.delay["/api/behavior_rules/save"] = 0.7
        self.stub.rules_director = {"rules": []}                    # directorはまだ空のルールを読んでいる
        self.page.fill("[data-f='priority']", "9")
        self.page.press("[data-f='priority']", "Tab")
        self.page.click("#saveBtn")
        self.settle(250)
        self.assertTrue(self.page.evaluate("document.getElementById('rulesList').inert"), "保存中なのに編集できる")
        self.wait_text("#saveStatus", "まだ保存したルールを読み込んでいません", 20000)
        self.assertFalse(self.page.evaluate("document.getElementById('rulesList').inert"))
        self.assertEqual(self.stub.rules["rules"][0]["priority"], 9)

    def test_director_sync_confirmed(self):
        self.open_rules()
        self.page.fill("[data-f='priority']", "3")
        self.page.press("[data-f='priority']", "Tab")
        self.page.click("#saveBtn")
        self.wait_text("#saveStatus", "directorが保存したルールを読み込みました", 15000)

    def test_min_hold_range_note(self):
        self.open_rules()
        self.page.fill("#minHold", "9999")
        self.assertIn("範囲は", self.page.text_content("#minHold + .nk-note"))


class TreePageTest(BrowserBase):
    def open_tree(self):
        self.open("tree", "", "() => document.querySelector('#treeBox .node')")
        self.page.click("[data-act='edit'][data-id='0.0']")
        self.page.wait_for_selector("[data-pk='seconds']")

    def test_range_note_edit_and_deferred_rerender(self):
        self.open_tree()
        self.page.fill("[data-pk='seconds']", "700")
        self.assertIn("範囲は", self.page.text_content("[data-pk='seconds'] + .nk-note"))
        self.page.fill("[data-pk='seconds']", "40")
        self.page.evaluate("document.querySelector('[data-pk=\"seconds\"]').__mark = 1")
        self.stub.pins.append({"id": "p2", "name": "窓", "color": "#2563eb", "x": 0, "y": 0, "yaw_deg": 0, "distance_m": 1})
        self.page.evaluate("loadPins()")
        self.settle(100)
        self.assertEqual(self.page.evaluate("document.querySelector('[data-pk=\"seconds\"]').__mark"), 1, "編集中なのに木が作り直された")
        self.page.press("[data-pk='seconds']", "Tab")             # 確定
        self.settle(300)
        self.assertEqual(self.page.evaluate("draft.root.children[0].params.seconds"), 40)
        self.assertEqual(self.val("[data-pk='seconds']"), "40")

    def test_json_pane_edit_counts_as_unsaved(self):
        self.open_tree()
        self.page.click("#tabJson")
        self.page.fill("#jsonBox", '{"x": 1}')
        self.assertTrue(self.page.evaluate("isDirty()"), "JSON欄の未反映の編集が未保存として数えられない")

    def test_save_ok_and_mismatch_with_inert(self):
        self.open_tree()
        self.page.fill("[data-pk='seconds']", "55")
        self.page.press("[data-pk='seconds']", "Tab")
        self.stub.delay["/api/behavior_tree"] = 0.6
        self.page.click("#saveBtn")
        self.settle(200)
        self.assertTrue(self.page.evaluate("document.getElementById('treeBox').inert"), "保存中なのに編集できる")
        self.page.wait_for_function("!document.getElementById('treeBox').inert", timeout=10000)
        self.assertEqual(self.stub.tree["root"]["children"][0]["params"]["seconds"], 55)
        self.wait_text("#saveStatus", "directorが保存した木を読み込みました", 15000)

    def test_save_not_applied_warns(self):
        self.open_tree()
        self.stub.ignore.add("tree")
        self.page.fill("[data-pk='seconds']", "66")
        self.page.press("[data-pk='seconds']", "Tab")
        self.page.click("#saveBtn")
        self.wait_text("#globalMsg", "一致しません", 15000)


class VoiceSettingsPageTest(BrowserBase):
    def open_vs(self):
        self.open("voice", "", "() => document.getElementById('fTemp') && document.getElementById('fTemp').value !== ''")

    def test_range_note_and_success(self):
        self.open_vs()
        self.page.fill("#fTemp", "5")
        self.assertIn("範囲は", self.page.text_content("#fTemp + .nk-note"))
        self.page.fill("#fTemp", "0.9")
        self.page.click("#saveBtn")
        self.page.wait_for_function("document.getElementById('globalMsg').textContent.includes('保存しました')", timeout=8000)
        self.assertEqual(self.stub.vs_settings["temperature"], 0.9)
        self.assertEqual(self.stub.posts("/api/voice_settings/save")[-1], {"voice": {"temperature": 0.9}}, "変更した項目だけ送る")

    def test_not_applied_warns_and_keeps_input(self):
        self.open_vs()
        self.stub.ignore.add("vs")
        self.page.fill("#intervalInput", "120")
        self.page.click("#saveBtn")
        self.wait_text("#globalMsg", "反映されていません", 15000)
        self.assertEqual(self.val("#intervalInput"), "120")
        self.assertTrue(self.page.evaluate("isDirty()"))

    def test_inert_while_saving(self):
        self.open_vs()
        self.stub.delay["/api/voice_settings/save"] = 0.6
        self.page.fill("#fNum", "300")
        self.page.click("#saveBtn")
        self.settle(200)
        self.assertTrue(self.page.evaluate("document.getElementById('tab-basic').inert"))
        self.page.wait_for_function("!document.getElementById('tab-basic').inert", timeout=10000)


class Stage1Test(BrowserBase):
    def open_s1(self):
        self.open("stage1", "", "() => document.getElementById('gainRkpFwd') && document.getElementById('gainRkpFwd').value !== ''")

    def test_gains_dirty_protected_and_empty_never_sent_as_zero(self):
        self.open_s1()
        self.page.fill("#gainRkpFwd", "2.5")
        self.blur()
        self.stub.gains["lkp_bwd"] = 4.0
        self.page.evaluate("loadGains()")
        self.assertEqual(self.val("#gainRkpFwd"), "2.5", "未保存のゲイン編集がポーリングで消えた")
        self.assertEqual(self.val("#gainLkpBwd"), "1", "未保存の編集がある間は全体の更新を見送る(対応関係を保つ)")
        self.page.fill("#gainRkiFwd", "")
        self.page.click("#gainSaveBtn")
        self.wait_text("#gainSaveHint", "空欄")
        self.assertEqual(self.stub.posts("/api/pid_gains"), [], "空欄を0として送った")
        self.page.fill("#gainRkiFwd", "0.5")
        self.page.click("#gainSaveBtn")
        self.wait_text("#gainSaveHint", "ESP32の値で確認済み", 10000)
        self.assertEqual(len(self.stub.posts("/api/pid_gains")[-1]), 12)

    def test_gains_not_applied_warns(self):
        self.open_s1()
        self.stub.ignore.add("gains")
        self.page.fill("#gainRkdFwd", "3")
        self.page.click("#gainSaveBtn")
        self.wait_text("#gainSaveHint", "ESP32に反映されていません", 15000)
        self.assertEqual(self.val("#gainRkdFwd"), "3")


# ======================================================================
# 3. 全入力の網羅(実ブラウザで、登録されていない入力欄が無いこと)
# ======================================================================
# 共通ヘルパーに登録されていない入力欄の、理由つき例外(これ以外は全て登録済みでなければならない)
RUNTIME_EXEMPT = {
    # Stage1テストページ: 1回の試験走行のパラメータ(「実行」を押した時に読むだけ。ポーリングは書き換えない)
    "stage1": {"testMode", "testDistance", "testAngle", "testDuration"},
}


@unittest.skipUnless(HAVE_PW, "playwright がありません")
class CoverageTest(BrowserBase):
    UNPROTECTED_JS = """() => {
      const g = new Set(NKField.guarded()), w = NKField.watched();
      return Array.from(document.querySelectorAll('input,select,textarea'))
        .filter((e) => !g.has(e) && !w.some((r) => r.contains(e)))
        .map((e) => e.id || ('.' + e.className) || e.outerHTML.slice(0, 80));
    }"""

    def check(self, key, ready, hash_="", prep=None):
        self.open(key, hash_, ready)
        if prep:
            prep()
        self.settle(600)
        left = set(self.page.evaluate(self.UNPROTECTED_JS))
        exempt = RUNTIME_EXEMPT.get(key, set())
        self.assertEqual(left - exempt, set(), f"{key}: 共通ヘルパーで保護されていない入力欄がある")
        total = self.page.evaluate("document.querySelectorAll('input,select,textarea').length")
        self.assertGreater(total, 0, key)

    def test_full(self):
        self.check("full", "() => document.getElementById('vlSpeechNum').value !== ''", "#tab-voice")

    def test_mobile(self):
        self.check("mobile", "() => document.getElementById('bgmVolume').value !== ''")

    def test_stage2(self):
        self.check("stage2", "() => document.getElementById('manualLinear').value !== ''")

    def test_stage2_mobile(self):
        self.check("stage2_mobile", "() => document.getElementById('manualLinear').value !== ''")

    def test_stage1(self):
        self.check("stage1", "() => document.getElementById('gainRkpFwd').value !== ''")

    def test_voice_settings(self):
        self.check("voice", "() => document.getElementById('fTemp').value !== ''")

    def test_rules_with_every_kind_of_dynamic_field(self):
        def prep():
            self.page.click("#rulesList [data-act='edit']")
            self.page.wait_for_selector("[data-f='priority']")
        self.check("rules", "() => document.querySelector('#rulesList .rule')", prep=prep)

    def test_tree_with_editor_open(self):
        def prep():
            self.page.click("[data-act='edit'][data-id='0.0']")
            self.page.wait_for_selector("[data-pk='seconds']")
        self.check("tree", "() => document.querySelector('#treeBox .node')", prep=prep)

    def test_dynamic_template_inputs_are_inside_watched_roots(self):
        # 動的に作る入力欄(innerHTMLのテンプレート)は、watch済みの領域(rulesList / treeBox)の中だけで作られること
        rules_js = re.findall(r"<script>(.*?)</script>", PAGES["rules"], re.S)[0]
        self.assertIn('NKField.watch("rulesList")', rules_js)
        self.assertEqual(len(re.findall(r"\$\(\"(?:rulesList)\"\)\.innerHTML|list\.innerHTML", rules_js)), 2)   # 入力欄を作る場所は rulesList(renderRules)だけ
        tree_js = re.findall(r"<script>(.*?)</script>", PAGES["tree"], re.S)[0]
        self.assertIn('NKField.watch("treeBox")', tree_js)
        self.assertEqual(tree_js.count('$("treeBox").innerHTML'), 1)


if __name__ == "__main__":
    try:
        unittest.main(verbosity=1)
    finally:
        shutil.rmtree(_TMP, ignore_errors=True)
