"""PCだけで、iPad表示(/display)のサイトを起動する(ROS・Jetson・ロボット不要)。2026-10-27

実際のコード(nokosahen_core の dashboard_common / display_hub / display_routes)を、そのまま使う。
ロボットには何も送らない。残数・メニュー・「今の状態」の文言は、ここで作った見本データ(実機の値ではない)。

使い方(WSL。PowerShellからは wsl python3 ...):
    python3 -I scripts/pc/serve_display_local.py [ポート(既定 8090)] [--password パスワード] [--tl] [--moving auto|on|off]

起動すると、同じネットワークのタブレットから開くURLを表示する:
    http://<このPCのIP>:<ポート>/display        iPad用の表示(固定URL。ホーム画面に追加して使う想定)
    http://<このPCのIP>:<ポート>/               モードを切り替える画面(PCやスマホのブラウザから。タブレットでも可)
画面のサイズは自動判定。11インチの確認は /display?size=11&debug=1 のように上書きできる。

iPad表示のログイン(本番と同じ方式): /display を最初に開くと、ログイン画面が出る。入れば、あとは再ログイン不要(このスクリプトを止めるまで)。
  アカウント名 : ADMIN_ACCOUNT(厨房サイトと同じ名前)
  パスワード   : 環境変数 DISPLAY_TEST_PASSWORD、または --password。どちらも無ければ、起動のたびにランダムな一時パスワードを作り、
                 **起動時に画面へ1回だけ**表示する。実機(厨房サイト)のパスワードは使わない・読まない。
  メニュー編集 : 「残数入力」モードの右上「メニュー編集」から。追加・名前の変更・削除・初期食数が、ここの見本データで動く。
  --tl         : トップ / を、TLの管理画面(設定タブの「メニュー編集」カードも見られる)にする。認証なし(開発用)。ロボットには何も送らない。
  --moving     : 「移動中です。ご注意ください」の見本(customerモードの「今の状態」)。auto=他の状態と一緒に数秒おきに順番に出る(既定)、
                 on=出しっぱなし(見た目の確認用)、off=出さない。本物の判定ロジック(moving_notice.py)に見本の速さと動きを入れて動かす。

トップ / のモード切替の画面は認証なし(開発用)。同じネットワークの人なら誰でも切り替えられるので、自宅/校内の信頼できるWi-Fiだけで使うこと。
WSL2から外のタブレットにつなぐには、Windows側でポート転送が要る(scripts/pc/serve_display_local.ps1 が設定する)。
"""
import json
import os
import secrets
import sys
import tempfile
import threading
import time
import types
from http.server import ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PKG = ROOT / "ros2_ws/src/nokosahen_core"
sys.path.insert(0, str(PKG))


def _install_stubs():
    """ROSが無い環境用のスタブ(あれば本物を使う)。テスト(test_ipad_display.py)と同じ流儀。"""
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m

    class Msg:
        def __init__(self, *a, **k):
            self.data = None

    class _Any:
        def __init__(self, *a, **k):
            self.data = None

    class _AutoMod(types.ModuleType):
        def __getattr__(self, item):
            if item.startswith("__"):
                raise AttributeError(item)
            return _Any

    try:
        import geometry_msgs.msg  # noqa: F401
        import nav_msgs.msg  # noqa: F401
        import std_msgs.msg  # noqa: F401
    except Exception:  # noqa: BLE001
        mod("geometry_msgs")
        mod("geometry_msgs.msg", PoseStamped=Msg, Twist=Msg)
        mod("nav_msgs")
        mod("nav_msgs.msg", Odometry=Msg)
        mod("std_msgs")
        mod("std_msgs.msg", Bool=Msg, Float32MultiArray=Msg, String=Msg, Empty=Msg)
    try:
        import rclpy.node  # noqa: F401
        import rclpy.qos  # noqa: F401
    except Exception:  # noqa: BLE001
        for n in ("rclpy", "rclpy.node", "rclpy.qos"):
            sys.modules[n] = _AutoMod(n)


_install_stubs()

from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import display_hub as dh  # noqa: E402
from nokosahen_core.display_survey import SurveyStore  # noqa: E402



def _parse_args(argv):
    port, password, tl, moving = 8090, "", False, "auto"
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--password" and i + 1 < len(argv):
            password = argv[i + 1]
            i += 1
        elif a.startswith("--password="):
            password = a.split("=", 1)[1]
        elif a == "--tl":
            tl = True
        elif a == "--moving" and i + 1 < len(argv) and argv[i + 1] in ("auto", "on", "off"):
            moving = argv[i + 1]
            i += 1
        elif a.isdigit():
            port = int(a)
        else:
            sys.exit(f"不明な引数です: {a}\n使い方: serve_display_local.py [ポート] [--password パスワード] [--tl] [--moving auto|on|off]")
        i += 1
    return port, password, tl, moving


PORT, _ARG_PASSWORD, USE_TL, MOVING_ARG = _parse_args(sys.argv[1:])
DISPLAY_USER = "ADMIN_ACCOUNT"

# 見本の残数(実機の値ではない)。形は inventory_manager の /nokosahen/menu_state に合わせる
MENU = {"state": {"items": [
    {"name": "日替わり定食", "initial": 60, "remaining": 24, "sold": 36},
    {"name": "カレーライス", "initial": 60, "remaining": 3, "sold": 57},
    {"name": "うどん", "initial": 41, "remaining": 0, "sold": 41},
], "promote_item": "日替わり定食"}}

# 「今の状態」の文言の見本。実機では behavior_director のモードから決まる。ここでは数秒ごとに順番に切り替えて見せる
# 見本の動き: (director のモード, 車輪の速さ[m/s])。POLICY_GO_PIN・RETURN_HOME・MANUAL_OVERRIDE・MAPPING_ACTIVE_HOLD で動いている間だけ「移動中」が出る
SAMPLE_BEHAVIOR = [("POLICY_NO_SCAN", 0.0), ("POLICY_GO_PIN", 0.25), ("STOP_ANNOUNCE", 0.0), ("RETURN_HOME", 0.25), ("APPROACH_ARRIVED", 0.0),
                   ("MANUAL_OVERRIDE", 0.3), ("SURVEY_ACTIVE_HOLD", 0.0), ("MAPPING_ACTIVE_HOLD", 0.2), ("IDLE_WIGGLE", 0.1), ("ESTOP_HOLD", 0.0)]
_beh = {"i": 0, "t": 0.0}
_moving = {"notice": None, "hub": None}


def _inventory_sink(topic, payload):
    """残数の入力を、見本の残数に反映する(実機では inventory_manager へ送る)。"""
    items = MENU["state"]["items"]
    if topic.endswith("sold_total_event"):
        for it in items:
            if it["name"] == payload.get("menu"):
                it["sold"] = int(payload["sold_total"])
                it["remaining"] = max(0, it["initial"] - it["sold"])
    else:
        MENU["state"]["items"] = [dict(i) for i in payload.get("items", [])]
    return True


def _customer():
    from nokosahen_core.moving_notice import MovingNotice
    now = time.monotonic()
    if now - _beh["t"] > 8.0:
        _beh["t"] = now
        _beh["i"] = (_beh["i"] + 1) % len(SAMPLE_BEHAVIOR)
    mode, speed = SAMPLE_BEHAVIOR[_beh["i"]]
    if MOVING_ARG == "on":
        mode, speed = "POLICY_GO_PIN", 0.25
    if _moving["notice"] is None:
        _moving["notice"] = MovingNotice()
    n = _moving["notice"]
    n.on_odom(speed, now)
    n.on_behavior({"mode": mode}, now)
    hub = _moving["hub"]
    answering = bool(hub and hub.status_for_ros().get("active"))
    r = n.evaluate(now, survey_answering=answering) if MOVING_ARG != "off" else None
    return dh.build_customer_payload(MENU["state"], {"mode": mode}, 0.0, None, r)


CONTROL_HTML = """<!doctype html>
<html lang="ja"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>表示モードの切替</title>
<style>
body{font-family:-apple-system,"Hiragino Sans","Yu Gothic",sans-serif;margin:0;padding:16px;background:#f4f5f7;color:#1c2330}
h1{font-size:20px;margin:0 0 4px} p{margin:4px 0 12px;color:#4a5568;font-size:14px}
.row{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:12px;margin:12px 0}
button{font-size:18px;padding:18px 12px;border-radius:12px;border:2px solid #2b6cb0;background:#fff;color:#1a365d;cursor:pointer}
button.on{background:#2b6cb0;color:#fff} button:active{opacity:.7}
#st{font-size:15px;padding:10px 12px;background:#fff;border-radius:8px;border:1px solid #cbd5e0;white-space:pre-wrap}
.small{font-size:13px;color:#4a5568}
</style></head><body>
<h1>表示モードの切替(開発用)</h1>
<p>iPadで <b>/display</b> を開いたまま、ここのボタンを押すと、1秒ほどで iPad の画面が切り替わります。</p>
<div class="row">
<button data-m="customer">お客さん向け<br><span class="small">今の状態の文言と残数</span></button>
<button data-m="survey">アンケート<br><span class="small">設問に回答</span></button>
<button data-m="inventory">残数入力<br><span class="small">スタッフが入力</span></button>
<button data-m="ocrtest">OCRテスト<br><span class="small">模擬POS</span></button>
</div>
<div id="st">状態を取得中…</div>
<p class="small">PCの確認用: <a href="/display" target="_blank">/display を開く</a> / <a href="/display?debug=1" target="_blank">/display?debug=1</a> / <a href="/api/survey/state" target="_blank">状態(JSON)</a></p>
<script>
const st=document.getElementById('st');
async function refresh(){
  try{
    const r=await fetch('/api/survey/state',{cache:'no-store'}); const s=(await r.json()).state||{};
    document.querySelectorAll('button[data-m]').forEach(b=>b.classList.toggle('on',b.dataset.m===s.mode));
    st.textContent='いまのモード: '+s.mode+(s.pending_mode?'(保留中: '+s.pending_mode+')':'')+'\\n';
  }catch(e){st.textContent='状態を取得できません: '+e}
}
document.querySelectorAll('button[data-m]').forEach(b=>b.addEventListener('click',async()=>{
  st.textContent='切り替え中…';
  try{
    const r=await fetch('/api/survey/mode',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mode:b.dataset.m})});
    const d=await r.json().catch(()=>({})); if(!r.ok||d.ok===false){st.textContent='切り替えられません: '+(d.message||d.error||r.status);return}
  }catch(e){st.textContent='失敗: '+e;return}
  refresh();
}));
refresh(); setInterval(refresh,1500);
</script></body></html>
"""


class _FakeTeleop:
    """ダッシュボードのハンドラが要求する操作部品の代わり。ロボットには何も出さない。"""
    def __getattr__(self, name):
        return lambda *a, **k: {}


def _tl_html():
    """--tl: TLの管理画面(設定タブ)。本物の full_dashboard_node の画面HTMLをそのまま返す(ROSのノードは起動しない)。"""
    from nokosahen_core import full_dashboard_node as fd
    return fd.HTML.encode("utf-8")


def main():
    password = os.environ.get("DISPLAY_TEST_PASSWORD", "") or _ARG_PASSWORD
    random_pw = not password
    if random_pw:
        password = secrets.token_urlsafe(9)     # 起動のたびに新しい一時パスワード(実機のものではない)
    tmp = Path(tempfile.mkdtemp(prefix="nk_display_local_"))
    hub = dh.DisplayHub(
        mode_file=str(tmp / "display_mode"),
        survey_store=SurveyStore(str(tmp / "survey")),
        customer_provider=_customer,
        menu_state_provider=lambda: MENU["state"],
        inventory_sink=_inventory_sink,
    )
    _moving["hub"] = hub
    admin = dh.LocalDisplayAdmin(hub)

    def tick():
        hub.tick()
        threading.Timer(1.0, tick).start()
    tick()

    page = _tl_html if USE_TL else (lambda: CONTROL_HTML.encode("utf-8"))
    handler = dc.make_dashboard_handler(
        page, lambda: {"menu_state": MENU["state"]}, _FakeTeleop(),
        survey_admin=admin, auth_username="", auth_password="",       # トップ / とTLの管理画面は認証なし(開発用)
        enforce_single_controller=False, display_hub=hub,
        display_auth_username=DISPLAY_USER, display_auth_password=password,   # /display は本番と同じログイン
    )
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), handler)
    print(f"起動しました(ポート {PORT})。ROS・ロボットは使いません。止めるには Ctrl+C。")
    print(f"  iPad用   : http://<このPCのIP>:{PORT}/display")
    print(f"  切替画面 : http://<このPCのIP>:{PORT}/" + ("(TLの管理画面。--tl)" if USE_TL else ""))
    print(f"  /display のログイン: アカウント名 {DISPLAY_USER}")
    if random_pw:
        print(f"                       パスワード {password}   ← 一時パスワード(今回の起動だけ有効。この1回しか表示しません)")
    else:
        print("                       パスワード: 指定されたもの(DISPLAY_TEST_PASSWORD または --password。表示しません)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
