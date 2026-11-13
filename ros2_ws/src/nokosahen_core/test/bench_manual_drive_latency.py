#!/usr/bin/env python
"""手動運転(POST /api/teleop)の応答時間の測定ハーネス(2026-11-06追加)。ROS・実機は使わない。

「ボタンを押してから動くまでが遅い」の原因を、PC上で絞り込むための道具。本物の make_dashboard_handler / make_mobile_handler を
ThreadingHTTPServer で起動し、rclpy の代わりに替え玉(FakeNode)を使う。本物の TeleopController(handle_drive も本物)を使うので、
ロック・seq処理・publish呼び出しも測れる。2台目(voice_bridge)・ラズパイ・OCR(8091)は偽サーバー。ロボットには何も送らない。

【測るもの】(--part)
  http    a/b) HTTP往復時間。ブラウザと同じく100 ms周期で POST /api/teleop(forward/turn/seq/tab_id)を送り、1回ごとの往復を測る。
          条件 (--conds): none=背景なし / bg=PC版ページ相当の背景ポーリング+iPad+survey_monitor / slow=bgに加えて2台目・OCRが2秒遅い / dead=2台目・OCRが4秒応答しない(各プロキシの3秒タイムアウトで切れる。2台目の電源断・OCR停止の模型) /
          slow_nopool=slowでブラウザの同時接続上限(6)を外す(サーバー側の遅れだけを見る) / hog / hog<NN>=bgに加えてサーバー内にPythonのCPU占有スレッド1本(hog=常時100%、hog75=75%など)
          (rclpyのexecutorのコールバックがGILを取り合う状況の模型)。
          クライアントは Chrome と同じ「1オリジン6接続まで」を模す(--pool 6)。待ち時間も往復に含める。
  profile c) サーバー内の内訳。do_POST の中の各段階(オリジン検査・認可・権限・JSON読み・handle_drive・応答)の時間と cProfile、
          1リクエストあたりのファイルI/O(stat/open/fsync/replace)の回数。
  loop    d) behavior_director._tick と motion_arbiter.Arbiter.decide を10万回、_face_target_bearing(10人)も。
  all     上のすべて

【使い方】
  python bench_manual_drive_latency.py --part http --n 300
  python bench_manual_drive_latency.py --pkg-root ../bench_wt_xxxx/ros2_ws/src/nokosahen_core --part http --json out.json   # 別のコミットのworktree
  python bench_manual_drive_latency.py --part profile
  python bench_manual_drive_latency.py --part loop --loops 100000
各worktreeのパッケージ(--pkg-root = nokosahen_core/ を含むフォルダ)を import する。古いコミットで無いAPI(display_hub等)は自動で外す。

PC(Windows)の値は、Jetsonの「相対値の目安」にすぎない(CPU・ディスク・ネットワークの絶対値は違う)。
"""
from __future__ import annotations

import argparse
import cProfile
import http.client
import inspect
import json
import os
import pstats
import statistics
import sys
import tempfile
import threading
import time
import types
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_PKG_ROOT = HERE.parent


# --------------------------------------------------------------------------------------
# rclpy・メッセージの替え玉(本物があればそれを使う)
# --------------------------------------------------------------------------------------
def install_stubs():
    try:
        import geometry_msgs.msg  # noqa: F401
        import nav_msgs.msg  # noqa: F401
        import std_msgs.msg  # noqa: F401
        return
    except Exception:  # noqa: BLE001
        pass

    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m

    class Msg:
        def __init__(self, *a, **k):
            self.data = None
            self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.header = types.SimpleNamespace(frame_id="", stamp=None)

    mod("geometry_msgs")
    mod("geometry_msgs.msg", PoseStamped=Msg, Twist=Msg)
    mod("nav_msgs")
    mod("nav_msgs.msg", Odometry=Msg)
    mod("std_msgs")
    mod("std_msgs.msg", Bool=Msg, Float32MultiArray=Msg, String=Msg, Empty=Msg)


class FakePub:
    def __init__(self):
        self.n = 0

    def publish(self, msg):
        self.n += 1


class FakeLogger:
    def __getattr__(self, name):
        return lambda *a, **k: None


class FakeNode:
    """rclpyノードの替え玉。create_timer は executor 役の1スレッドが期限が来たら順に呼ぶ(rclpy.spin の単スレッド実行の模型)。"""

    def __init__(self):
        self.timers = []
        self._stop = False

    def create_publisher(self, *a, **k):
        return FakePub()

    def create_subscription(self, *a, **k):
        return object()

    def destroy_subscription(self, *a, **k):
        pass

    def create_timer(self, period, cb, *a, **k):
        self.timers.append([period, cb, time.monotonic() + period])
        return object()

    def get_logger(self):
        return FakeLogger()

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=int(time.monotonic() * 1e9)))

    def declare_parameter(self, *a, **k):
        return None

    def spin_in_thread(self):
        def run():
            while not self._stop:
                now = time.monotonic()
                for t in self.timers:
                    if now >= t[2]:
                        t[2] = now + t[0]
                        try:
                            t[1]()
                        except Exception:  # noqa: BLE001
                            pass
                time.sleep(0.005)
        threading.Thread(target=run, daemon=True).start()

    def stop(self):
        self._stop = True


# --------------------------------------------------------------------------------------
# 偽の上流(2台目 voice_bridge / OCR 8091 / ラズパイ)。どのパスでも delay 秒待ってから、JSONを返す
# --------------------------------------------------------------------------------------
class Upstream(BaseHTTPRequestHandler):
    delay = 0.0

    def log_message(self, *a):
        pass

    def _reply(self):
        if type(self).delay > 0:
            time.sleep(type(self).delay)
        try:
            n = int(self.headers.get("Content-Length", "0"))
            if n:
                self.rfile.read(n)
        except Exception:  # noqa: BLE001
            pass
        body = json.dumps({"ok": True, "speech_volume": 2.0, "bgm_volume": 1.0, "bgm_duck_volume": 0.15, "enabled": True,
                           "settings": {}, "camera_profile": "balanced", "camera_profiles": {}}).encode()
        try:
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except Exception:  # noqa: BLE001
            pass

    do_GET = _reply
    do_POST = _reply


def serve(handler):
    srv = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, srv.server_address[1]


# --------------------------------------------------------------------------------------
# 本物の制御部品に与える、軽い偽物(中身の重さは実機と違うが、JSONの大きさはそれらしくする)
# --------------------------------------------------------------------------------------
class FakeMapGoal:
    def __init__(self):
        n = 200 * 200
        self._json = {"width": 200, "height": 200, "resolution": 0.05, "origin": [0, 0], "data": [0] * n}

    def map_json(self):
        return self._json

    def status(self):
        return {"active": False}

    def __getattr__(self, name):  # goal_status など、読むだけの問い合わせは固定の小さい応答
        if name.startswith("__"):
            raise AttributeError(name)
        return lambda *a, **k: {"ok": True, "active": False, "state": "idle"}


class FakeScan:
    def scan_json(self):
        return {"angle_min": -3.14, "angle_increment": 0.0175, "ranges": [1.5] * 360, "range_max": 12.0}


class FakeStatus:
    def __init__(self, payload=None):
        self.payload = payload or {"ok": True, "state": "idle", "pose": [0, 0, 0]}

    def status(self):
        return dict(self.payload)

    def list_maps(self):
        return []

    def current_gains(self):
        return None


class FakeCam:
    def __init__(self):
        self.jpeg = b"\xff\xd8" + os.urandom(40000) + b"\xff\xd9"

    def jpeg_bytes(self):
        return self.jpeg


def make_state_getter():
    st = {"crowd_level": "calm", "mode": "idle", "estop_active": False, "manual_override": False,
          "odom": {"x": 1.0, "y": 2.0, "yaw": 0.1}, "robot_pose": {"x": 1, "y": 2, "yaw": 0.1},
          "menu_state": {"items": [{"name": "m%d" % i, "remaining": i} for i in range(8)]},
          "trail": [[i * 0.1, i * 0.1] for i in range(200)], "planned_path": [[i * 0.1, 0] for i in range(100)]}
    return lambda: dict(st)


def _kw(fn, **kwargs):
    """関数が受け取れる引数だけ渡す(古いコミットに無い引数を外す)。"""
    params = inspect.signature(fn).parameters
    return {k: v for k, v in kwargs.items() if k in params}


class Rig:
    """サーバー一式。pkg_root のコミットのコードで起動する。"""

    def __init__(self, pkg_root: Path, tmp: Path, use_display: bool = True, upstream_url: str = ""):
        install_stubs()
        sys.path.insert(0, str(pkg_root))
        # テストと同じ import (パスで本当に対象のコミットのものが読まれたか確認する)
        import nokosahen_core  # noqa: F401
        from nokosahen_core import accounts as acc
        from nokosahen_core import dashboard_common as dc
        self.dc = dc
        self.acc = acc
        self.pkg_file = Path(dc.__file__).resolve()
        self.tmp = tmp
        self.node = FakeNode()
        if upstream_url:
            self.up = None
            up_url = upstream_url
        else:
            self.up, self.up_port = serve(Upstream)
            up_url = f"http://127.0.0.1:{self.up_port}"
        self.up_url = up_url

        from nokosahen_core.bgm_proxy import BgmProxy
        from nokosahen_core.survey_admin_proxy import SurveyAdminProxy
        from nokosahen_core.voice_levels_proxy import VoiceLevelsProxy
        from nokosahen_core.voice_master_proxy import VoiceMasterProxy
        from nokosahen_core.ocr_admin_proxy import OcrAdminProxy
        from nokosahen_core.voice_settings_controller import VoiceSettingsController
        from nokosahen_core.voice_settings_proxy import VoiceSettingsProxy
        from nokosahen_core.announce_settings import AnnounceSettingsStore

        tele_kw = _kw(dc.TeleopController.__init__, node=self.node,
                      soft_estop_state_file=str(tmp / "soft_estop.json"),
                      manual_person_guard_state_file=str(tmp / "mpg.json"))
        self.teleop = dc.TeleopController(**tele_kw)
        self.node.spin_in_thread()

        accounts_file = tmp / "accounts.yaml"
        accounts_file.write_text("accounts: []\noperator_permissions: []\n", encoding="utf-8")
        self.store = acc.AccountStore(accounts_path=str(accounts_file), permissions_path=str(tmp / "perm.json"))

        self.display_hub = None
        if use_display:
            try:
                from nokosahen_core.display_hub import DisplayHub
                from nokosahen_core.display_survey import SurveyStore
                self.display_hub = DisplayHub(
                    mode_file=str(tmp / "display_mode.json"),
                    survey_store=SurveyStore(str(tmp / "survey")),
                    customer_provider=lambda: {"status": "ok", "menu": [], "announce": ""},
                    menu_state_provider=lambda: {"items": []}, inventory_sink=lambda t, p: True)
            except Exception as exc:  # noqa: BLE001
                print(f"[rig] iPad表示(display_hub)は使えません(古いコミット?): {exc!r}", file=sys.stderr)
                self.display_hub = None

        self.ann = AnnounceSettingsStore(str(tmp / "announce.json"))
        self.vs = VoiceSettingsController(self.ann, VoiceSettingsProxy(up_url))
        cam = FakeCam()
        survey = SurveyAdminProxy(up_url)
        if self.display_hub is not None:
            try:
                from nokosahen_core.full_dashboard_node import LocalDisplayAdmin  # noqa: F401
            except Exception:  # noqa: BLE001
                pass
        common = dict(
            html_getter=lambda: b"<html><body>x</body></html>", state_getter=make_state_getter(), teleop=self.teleop,
            map_goal=FakeMapGoal(), scan=FakeScan(), camera=cam, camera_rear={"left": cam, "right": cam},
            mapping=FakeStatus(), map_lifecycle=FakeStatus(), frontier_explorer=FakeStatus(),
            bgm=BgmProxy(up_url), voice_master=VoiceMasterProxy(up_url), voice_levels=VoiceLevelsProxy(up_url),
            survey_admin=survey, ocr_admin=OcrAdminProxy(up_url), voice_settings=self.vs,
            auth_username="TL", auth_password="tl-pass", enforce_single_controller=False,
            account_store=self.store, display_hub=self.display_hub, display_auth_username="ADMIN_ACCOUNT",
            display_auth_password="ipad-pass")
        self.pc_handler = dc.make_dashboard_handler(**_kw(dc.make_dashboard_handler, **common))
        mob = dict(common)
        mob.update(restricted_username="", restricted_password="")
        self.mobile_handler = dc.make_mobile_handler(
            **_kw(dc.make_mobile_handler, html_getter=common["html_getter"], state_getter=common["state_getter"],
                  teleop=self.teleop, bgm=common["bgm"], auth_username="TL", auth_password="tl-pass",
                  enforce_single_controller=False, account_store=self.store, voice_levels=common["voice_levels"],
                  voice_master=common["voice_master"]))
        self.pc_srv, self.pc_port = serve(self.pc_handler)
        self.mobile_srv, self.mobile_port = serve(self.mobile_handler)
        self.has_display = self.display_hub is not None

    def close(self):
        self.node.stop()
        for s in (self.pc_srv, self.mobile_srv, self.up):
            if s is None:
                continue
            try:
                s.shutdown()
                s.server_close()
            except Exception:  # noqa: BLE001
                pass


# --------------------------------------------------------------------------------------
# ブラウザの模型
# --------------------------------------------------------------------------------------
class Browser:
    """1オリジンあたり同時 pool 接続まで(Chrome は 6)。待ち時間も含めて往復を測る。"""

    def __init__(self, port: int, pool: int = 6, cookie: str = ""):
        self.port = port
        self.sem = threading.BoundedSemaphore(pool) if pool and pool > 0 else None
        self.cookie = cookie

    def request(self, method, path, body=None, timeout=20.0, cookie=None):
        """(status, 全体ms[接続待ち含む], 接続待ちms)。"""
        t0 = time.perf_counter()
        if self.sem is not None:
            self.sem.acquire()
        t1 = time.perf_counter()
        try:
            conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=timeout)
            headers = {"Host": f"127.0.0.1:{self.port}", "Origin": f"http://127.0.0.1:{self.port}",
                       "Sec-Fetch-Site": "same-origin", "Accept": "*/*"}
            ck = self.cookie if cookie is None else cookie
            if ck:
                headers["Cookie"] = ck
            data = None
            if body is not None:
                data = json.dumps(body).encode()
                headers["Content-Type"] = "application/json"
            conn.request(method, path, body=data, headers=headers)
            resp = conn.getresponse()
            resp.read()
            status = resp.status
            conn.close()
        except Exception:  # noqa: BLE001
            status = -1
        finally:
            if self.sem is not None:
                self.sem.release()
        t2 = time.perf_counter()
        return status, (t2 - t0) * 1000.0, (t1 - t0) * 1000.0

    def login(self, user, pw, path="/api/login"):
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=20)
        conn.request("POST", path, body=json.dumps({"username": user, "password": pw}).encode(),
                     headers={"Content-Type": "application/json", "Origin": f"http://127.0.0.1:{self.port}",
                              "Host": f"127.0.0.1:{self.port}"})
        r = conn.getresponse()
        r.read()
        ck = (r.getheader("Set-Cookie") or "").split(";")[0]
        conn.close()
        if r.status != 200 or not ck:
            raise RuntimeError(f"login failed {r.status}")
        self.cookie = ck
        return ck


# PC版ページの定期問い合わせ(full_dashboard_node.py の setInterval / NKField.poll から。ms)
PC_POLLS = [
    ("/api/state", 500), ("/api/scan", 500), ("/api/map", 2000), ("/api/mapping/status", 3000),
    ("/api/exploration/status", 2000), ("/api/mapping/lifecycle_status", 2000), ("/api/mapping/list", 5000),
    ("/api/boundary", 5000), ("/api/wall_escape/status", 1500), ("/api/manual_person_guard", 1500),
    ("/api/speed_settings", 15000), ("/api/bgm", 5000), ("/api/voice_master", 5000), ("/api/voice_levels", 5000),
    ("/api/survey/state", 5000), ("/api/menu/state", 5000), ("/api/ocr/state", 5000), ("/api/whoami", 10000),
    ("/api/goal/status", 700), ("/api/pins", 2000), ("/api/map_view", 3000),
]
# カメラは「読み込み完了→400/700 ms後に次」の直列(onload待ち)
PC_CAMERA_LOOPS = [("/api/camera", 0.4), ("/api/camera?which=left", 0.7), ("/api/camera?which=right", 0.7)]
MOBILE_POLLS = [("/api/state", 500), ("/api/speed_settings", 15000), ("/api/bgm", 5000), ("/api/whoami", 10000)]


class Background:
    """背景のポーリング一式。stop() で止める。"""

    def __init__(self, rig: Rig, kind: str, pool: int, cookie: str, with_ipad: bool = True):
        self.stop_ev = threading.Event()
        self.threads = []
        self.counts = {"req": 0, "err": 0}
        self.browser = Browser(rig.pc_port if kind == "pc" else rig.mobile_port, pool, cookie)
        polls = PC_POLLS if kind == "pc" else MOBILE_POLLS
        for path, ms in polls:
            self._spawn(self._ticker, path, ms / 1000.0)
        if kind == "pc":
            for path, gap in PC_CAMERA_LOOPS:
                self._spawn(self._serial, path, gap)
        if with_ipad and getattr(rig, "has_display", False) and kind == "pc":
            # iPad: ログイン済みで /display/api/state を約1 Hz(直列)。survey_monitor: 127.0.0.1 から Cookie なしで 1.5 Hz
            ipad = Browser(rig.pc_port, 0)
            try:
                ipad.login("ADMIN_ACCOUNT", "ipad-pass", "/display/api/login")
            except Exception as exc:  # noqa: BLE001
                print(f"[bg] iPadログイン失敗: {exc!r}", file=sys.stderr)
            self.ipad = ipad
            self._spawn(self._serial_with, ipad, "/display/api/state", 1.0)
            mon = Browser(rig.pc_port, 0)
            self._spawn(self._ticker_with, mon, "/display/api/state", 1.0 / 1.5)

    def _spawn(self, fn, *a):
        t = threading.Thread(target=fn, args=a, daemon=True)
        t.start()
        self.threads.append(t)

    def _one(self, br, path):
        st, ms, _ = br.request("GET", path, cookie=None if br is self.browser else None)
        self.counts["req"] += 1
        if st < 0:
            self.counts["err"] += 1

    def _ticker(self, path, period):
        self._ticker_with(self.browser, path, period)

    def _ticker_with(self, br, path, period):
        nxt = time.perf_counter()
        inflight = threading.Semaphore(8)  # 1ポーラーあたりの未完了の上限(暴走防止)
        while not self.stop_ev.is_set():
            nxt += period
            if inflight.acquire(blocking=False):
                def go():
                    try:
                        self._one(br, path)
                    finally:
                        inflight.release()
                threading.Thread(target=go, daemon=True).start()
            d = nxt - time.perf_counter()
            if d > 0:
                self.stop_ev.wait(d)

    def _serial(self, path, gap):
        self._serial_with(self.browser, path, gap)

    def _serial_with(self, br, path, gap):
        while not self.stop_ev.is_set():
            self._one(br, path)
            self.stop_ev.wait(gap)

    def stop(self):
        self.stop_ev.set()


class CpuHog:
    """サーバーと同じプロセスで動く、Pythonの計算だけをするスレッド(rclpyのコールバックやJPEG処理がGILを占有する状況の模型)。
    duty=1.0は常時占有(最悪)、0.5は20 msのうち10 ms計算して10 ms寝る。"""

    def __init__(self, n=1, duty=1.0):
        self.stop_ev = threading.Event()
        self.duty = duty
        for _ in range(n):
            threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        x = 0
        period = 0.02
        while not self.stop_ev.is_set():
            if self.duty >= 1.0:
                for i in range(20000):
                    x += i * i % 7
            else:
                t_end = time.perf_counter() + period * self.duty
                while time.perf_counter() < t_end:
                    for i in range(2000):
                        x += i * i % 7
                time.sleep(period * (1.0 - self.duty))

    def stop(self):
        self.stop_ev.set()


def pct(vals, p):
    if not vals:
        return float("nan")
    s = sorted(vals)
    k = min(len(s) - 1, max(0, int(round(p / 100.0 * (len(s) - 1)))))
    return s[k]


def summarize(vals):
    if not vals:
        return {"n": 0}
    return {"n": len(vals), "p50": pct(vals, 50), "p95": pct(vals, 95), "p99": pct(vals, 99), "max": max(vals),
            "mean": statistics.fmean(vals)}


def run_drive(rig: Rig, handler_kind: str, n: int, pool: int, cond: str):
    """条件 cond で n 回(動く側の)運転POSTを100 ms周期で送り、往復時間を返す。"""
    port = rig.pc_port if handler_kind == "pc" else rig.mobile_port
    drv = Browser(port, pool)
    drv.login("TL", "tl-pass")
    # 運転タブも背景と同じブラウザ(同じ6接続)に属する。背景ありのときは、接続枠を共有させる
    bg = None
    hog = None
    if cond in ("bg", "slow", "slow_nopool", "dead") or cond.startswith("hog"):
        Upstream.delay = {"slow": 2.0, "slow_nopool": 2.0, "dead": 4.0}.get(cond, 0.0)
        bgpool = 0 if cond == "slow_nopool" else pool
        bg = Background(rig, handler_kind, bgpool, drv.cookie)
        if bgpool:
            drv.sem = bg.browser.sem   # 同じ枠を共有
        else:
            drv.sem = None
        bg.browser.cookie = drv.cookie
        time.sleep(1.5)  # 背景が定常状態になるまで
    else:
        Upstream.delay = 0.0
    seq = 0
    tab = "benchtab0123456789"
    drive_lat, first_lat, queue_wait = [], [], []
    skipped = 0
    errors = 0
    in_flight_since = 0.0
    lock = threading.Lock()
    results = []

    def send(fwd, turn, s):
        nonlocal in_flight_since
        st, ms, qw = drv.request("POST", "/api/teleop", {"forward": fwd, "turn": turn, "seq": s, "tab_id": tab})
        with lock:
            results.append((fwd, turn, st, ms, qw, s))
        return st

    # 3秒前進→1秒停止 を繰り返す(JSと同じ規則: 停止は常に送る、前進は前回が未完了なら1秒まで見送る)
    period = 0.1
    t_next = time.perf_counter()
    sent_drive = 0
    ticks = 0
    in_flight = {"since": 0.0, "seq": 0}
    ths = []
    first_seq = set()
    while sent_drive < n:
        phase = (ticks % 40)
        forward = 1.0 if phase < 30 else 0.0
        stop = forward == 0.0
        ticks += 1
        if not stop and in_flight["since"] and (time.perf_counter() - in_flight["since"]) < 1.0:
            skipped += 1
        else:
            seq += 1
            if forward != 0.0 and phase == 0:
                first_seq.add(seq)
            s = seq

            def go(fwd=forward, s=s, is_stop=stop):
                if not is_stop:
                    in_flight["since"] = time.perf_counter()
                    in_flight["seq"] = s
                st = send(fwd, 0.0, s)
                if not is_stop and in_flight["seq"] == s:
                    in_flight["since"] = 0.0
                return st
            th = threading.Thread(target=go, daemon=True)
            th.start()
            ths.append(th)
            if not stop:
                sent_drive += 1
        t_next += period
        d = t_next - time.perf_counter()
        if d > 0:
            time.sleep(d)
        else:
            t_next = time.perf_counter()  # 遅れたら詰め直す(setIntervalの挙動に近い)
    for th in ths:
        th.join(timeout=30)
    if bg:
        bg.stop()
    if hog:
        hog.stop()
    Upstream.delay = 0.0
    for fwd, turn, st, ms, qw, s in results:
        if st != 200:
            errors += 1
        if fwd != 0.0:
            drive_lat.append(ms)
            queue_wait.append(qw)
            if s in first_seq:
                first_lat.append(ms)
    return {"drive": summarize(drive_lat), "press_first": summarize(first_lat), "queue_wait": summarize(queue_wait),
            "skipped_ticks": skipped, "errors": errors, "ticks": ticks,
            "bg_requests": bg.counts["req"] if bg else 0, "bg_errors": bg.counts["err"] if bg else 0}


class Remote:
    """別プロセスのサーバーのポート情報(Rigの代わり)。"""

    def __init__(self, pc_port, mobile_port, has_display):
        self.pc_port, self.mobile_port, self.has_display = pc_port, mobile_port, has_display


def spawn_server(pkg_root: Path, upstream_url: str, hog_duty: float):
    import subprocess
    cmd = [sys.executable, str(Path(__file__).resolve()), "--serve", "--pkg-root", str(pkg_root),
           "--upstream-url", upstream_url, "--hog-duty", str(hog_duty)]
    if os.environ.get("BENCH_NODELAY") == "1":
        cmd.append("--nodelay")
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    line = p.stdout.readline().split()
    if not line or line[0] != "PORTS":
        p.kill()
        raise RuntimeError("server did not start")
    return p, Remote(int(line[1]), int(line[2]), line[3] == "1")


def part_http(args, pkg_root):
    """サーバーは条件ごとに別プロセスで起動する(クライアントのスレッドとGILを取り合わないようにするため)。"""
    out = {}
    up, up_port = serve(Upstream)
    up_url = f"http://127.0.0.1:{up_port}"
    for h in args.handlers:
        for cond in args.conds:
            duty = 0.0   # hog / hog<NN>(例 hog75 = 75%)は、サーバー内のCPU占有スレッドの占有率
            if cond.startswith("hog"):
                duty = int(cond[3:]) / 100.0 if cond[3:].isdigit() else 1.0
            proc, rig = spawn_server(pkg_root, up_url, duty)
            try:
                r = run_drive(rig, h, args.n, args.pool, cond)
            finally:
                try:
                    proc.stdin.close()
                    proc.wait(timeout=10)
                except Exception:  # noqa: BLE001
                    proc.kill()
            out[f"{h}/{cond}"] = r
            d = r["drive"]
            print(f"  {h:6s} {cond:12s} n={d['n']:4d} p50={d['p50']:8.2f} p95={d['p95']:8.2f} p99={d['p99']:8.2f} max={d['max']:9.2f} ms"
                  f" | queue_wait p95={r['queue_wait']['p95']:7.2f} | skipped={r['skipped_ticks']} err={r['errors']} bg_req={r['bg_requests']}",
                  flush=True)
    up.shutdown()
    return out


def serve_mode(args):
    """--serve: 別プロセスでサーバー一式を起動し、ポートを標準出力に出して、標準入力が閉じるまで待つ。"""
    tmp = Path(tempfile.mkdtemp(prefix="bench_drive_srv_"))
    rig = Rig(Path(args.pkg_root).resolve(), tmp, upstream_url=args.upstream_url)
    if os.environ.get("BENCH_NOSTAT") == "1":   # 実験用: アカウントファイルのstatをやめたら(上限の見積り)どれだけ速くなるか
        rig.acc.AccountStore._reload = lambda self, force=False: None
    if args.nodelay:
        rig.pc_handler.disable_nagle_algorithm = True
        rig.mobile_handler.disable_nagle_algorithm = True
    if args.hog_duty > 0:
        CpuHog(1, args.hog_duty)
    print(f"PORTS {rig.pc_port} {rig.mobile_port} {1 if rig.has_display else 0}", flush=True)
    try:
        sys.stdin.read()
    finally:
        rig.close()


# --------------------------------------------------------------------------------------
# c) サーバー内の内訳
# --------------------------------------------------------------------------------------
def part_profile(args, rig):
    dc = rig.dc
    acc = rig.acc
    stage = {}

    def timed(name, fn):
        def w(*a, **k):
            t0 = time.perf_counter()
            try:
                return fn(*a, **k)
            finally:
                stage.setdefault(name, []).append((time.perf_counter() - t0) * 1000.0)
        return w

    # モジュールの関数・クラスのメソッドを時間計測つきに差し替える(対象のコミットのコードをそのまま呼ぶ)
    dc._post_origin_guard = timed("origin_guard", dc._post_origin_guard)
    dc._capability_gate = timed("capability_gate", dc._capability_gate)
    dc._resolve_control_account = timed("resolve_account(session+gate)", dc._resolve_control_account)
    acc.AccountStore.role_of = timed("accounts.role_of", acc.AccountStore.role_of)
    acc.AccountStore.operator_permissions = timed("accounts.operator_permissions", acc.AccountStore.operator_permissions)
    dc.TeleopController.handle_drive = timed("teleop.handle_drive", dc.TeleopController.handle_drive)
    dc.parse_teleop_seq = timed("parse_teleop_seq", dc.parse_teleop_seq)
    dc.build_teleop_source = timed("build_teleop_source", dc.build_teleop_source)
    H = rig.pc_handler
    H._read_json = timed("read_json", H._read_json)
    H._send_json = timed("send_json(+inject_role_ui+write)", H._send_json)
    orig_post = H.do_POST
    prof = cProfile.Profile()
    total = []

    def do_post(self):
        t0 = time.perf_counter()
        prof.enable()
        try:
            orig_post(self)
        finally:
            prof.disable()
            total.append((time.perf_counter() - t0) * 1000.0)
    H.do_POST = do_post

    # ファイルI/Oの回数を数える
    counters = {"os.stat": 0, "open": 0, "os.fsync": 0, "os.replace": 0, "os.listdir/scandir": 0, "Path.is_file": 0}
    import builtins
    real = {"stat": os.stat, "open": builtins.open, "fsync": os.fsync, "replace": os.replace,
            "listdir": os.listdir, "scandir": os.scandir, "is_file": Path.is_file}
    counting = {"on": False}
    tid = {"id": None}

    def cnt(key, f):
        def w(*a, **k):
            if counting["on"] and threading.get_ident() == tid["id"]:
                counters[key] += 1
            return f(*a, **k)
        return w
    os.stat = cnt("os.stat", real["stat"])
    builtins.open = cnt("open", real["open"])
    os.fsync = cnt("os.fsync", real["fsync"])
    os.replace = cnt("os.replace", real["replace"])
    os.listdir = cnt("os.listdir/scandir", real["listdir"])
    Path.is_file = cnt("Path.is_file", real["is_file"])

    # do_POST を呼ぶスレッドを特定してから数える
    orig2 = H.do_POST

    def do_post2(self):
        tid["id"] = threading.get_ident()
        counting["on"] = True
        try:
            orig2(self)
        finally:
            counting["on"] = False
    H.do_POST = do_post2

    br = Browser(rig.pc_port, 0)
    br.login("TL", "tl-pass")
    n = args.n
    # ウォームアップ
    for i in range(20):
        br.request("POST", "/api/teleop", {"forward": 1.0, "turn": 0.0, "seq": i + 1, "tab_id": "t"})
    for k in list(stage):
        stage[k].clear()
    total.clear()
    for k in counters:
        counters[k] = 0
    prof_n_before = n
    rtt = []
    for i in range(n):
        st, ms, _ = br.request("POST", "/api/teleop", {"forward": 1.0 if i % 40 < 30 else 0.0, "turn": 0.0, "seq": 100 + i, "tab_id": "t"})
        rtt.append(ms)
        time.sleep(0.01)
    os.stat, builtins.open, os.fsync, os.replace, os.listdir, Path.is_file = (
        real["stat"], real["open"], real["fsync"], real["replace"], real["listdir"], real["is_file"])
    res = {"client_rtt_ms": summarize(rtt), "do_POST_total_ms": summarize(total), "stages_ms": {},
           "io_per_request": {k: v / max(1, n) for k, v in counters.items()}}
    print(f"  クライアント往復(背景なし, 逐次) {summarize(rtt)}")
    print(f"  do_POST 全体(サーバー内) {summarize(total)}")
    print("  段階別(1回あたりの平均 ms / p95 / 最大 / 回数)  ※入れ子は重複して数える")
    for k, v in sorted(stage.items(), key=lambda kv: -statistics.fmean(kv[1])):
        s = summarize(v)
        res["stages_ms"][k] = s
        print(f"    {k:42s} mean={s['mean']:.4f} p95={s['p95']:.4f} max={s['max']:.4f} n={s['n']}")
    print("  1リクエストあたりのファイルI/O回数:", {k: round(v, 2) for k, v in res["io_per_request"].items()})
    st = pstats.Stats(prof)
    st.sort_stats("tottime")
    import io
    buf = io.StringIO()
    st.stream = buf
    st.print_stats(14)
    print("  cProfile(tottime上位14, do_POST内だけ):")
    print("\n".join("    " + l for l in buf.getvalue().splitlines() if l.strip())[:3500])
    return res


# --------------------------------------------------------------------------------------
# d) director / arbiter の周期
# --------------------------------------------------------------------------------------
def part_loop(args, pkg_root):
    sys.path.insert(0, str(pkg_root))
    sys.path.insert(0, str(HERE))
    res = {}
    from nokosahen_core.motion_arbiter import Arbiter
    a = Arbiter(plan_stale_sec=2.0)
    N = args.loops
    states = [dict(), dict(wasd=True), dict(near_person=True), dict(router_rotating=True)]
    ts = []
    t_all = time.perf_counter()
    for i in range(N):
        s = states[i & 3]
        for k, v in s.items():
            setattr(a, k, v)
        t0 = time.perf_counter()
        a.decide(10.0 + i * 0.1)
        ts.append((time.perf_counter() - t0) * 1e6)
        for k in s:
            setattr(a, k, False)
    print(f"  Arbiter.decide x{N}: mean={statistics.fmean(ts):.2f}us p50={pct(ts,50):.2f} p99={pct(ts,99):.2f} p99.9={pct(ts,99.9):.2f} max={max(ts):.1f}us (予算 100000us)")
    res["arbiter_us"] = {"mean": statistics.fmean(ts), "p99": pct(ts, 99), "max": max(ts)}
    try:
        import face_person_sim as fps
    except Exception as exc:  # noqa: BLE001
        print(f"  face_person_sim を読めない(古いコミット?): {exc!r}")
        return res
    from nokosahen_core import crowd_bearing as cb
    people = [fps.Person(2.0 + 0.3 * i, -1.5 + 0.35 * i) for i in range(10)]
    for label, plist in (("1人", people[:1]), ("10人", people)):
        sim = fps.Sim(people=plist, face=True, approach=False, scan=False)
        # 収束させてから、_tick だけを繰り返す(候補は固定)
        sim.run(5)
        n = sim.n
        ts = []
        for i in range(N):
            n._clock.t += 0.1
            n._person_candidates_mono = time.monotonic()
            n._person_candidates_rx = n._clock.now().nanoseconds / 1e9
            t0 = time.perf_counter()
            n._tick()
            ts.append((time.perf_counter() - t0) * 1e6)
            if len(n._cmd_vel_pub.msgs) > 1000:
                del n._cmd_vel_pub.msgs[:]
                del n._command_pub.msgs[:]
        print(f"  director._tick x{N} (人{label}, face ON): mean={statistics.fmean(ts):.1f}us p50={pct(ts,50):.1f} p99={pct(ts,99):.1f} p99.9={pct(ts,99.9):.1f} max={max(ts):.0f}us (予算 100000us)")
        res[f"director_tick_{label}_us"] = {"mean": statistics.fmean(ts), "p99": pct(ts, 99), "max": max(ts)}
        sim.close()
    # FaceBearingSelector 単体(10人の候補)
    sel = cb.FaceBearingSelector()
    import math
    # director が update に渡す形(融合済みの候補: bearing[rad]/weight/range[m])。10人が違う距離・方位にいる
    cands = [{"bearing": math.radians(-40 + 8 * i), "weight": 1.0 if i % 2 == 0 else 0.6, "range": 1.2 + 0.3 * i,
              "source": "lidar+camera" if i % 2 == 0 else "lidar"} for i in range(10)]
    ts = []
    t = 0.0
    for i in range(N):
        t += 0.1
        t0 = time.perf_counter()
        try:
            sel.update(t, cands, 0.0)
        except TypeError:
            sel.update(t, cands)
        ts.append((time.perf_counter() - t0) * 1e6)
    print(f"  FaceBearingSelector.update x{N} (候補10人): mean={statistics.fmean(ts):.1f}us p99={pct(ts,99):.1f} p99.9={pct(ts,99.9):.1f} max={max(ts):.0f}us")
    res["face_selector_10_us"] = {"mean": statistics.fmean(ts), "p99": pct(ts, 99), "max": max(ts)}
    print(f"  (loop全体 {time.perf_counter()-t_all:.1f}s)")
    return res


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:  # noqa: BLE001
        pass
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pkg-root", default=str(DEFAULT_PKG_ROOT), help="nokosahen_core/ パッケージを含むフォルダ(worktree なら <wt>/ros2_ws/src/nokosahen_core)")
    ap.add_argument("--part", default="http", choices=["http", "profile", "loop", "all"])
    ap.add_argument("--n", type=int, default=300, help="1条件あたりの運転POST数(100 ms周期)")
    ap.add_argument("--pool", type=int, default=6, help="ブラウザの同時接続数(0=無制限)")
    ap.add_argument("--handlers", default="pc,mobile")
    ap.add_argument("--conds", default="none,bg,slow,dead,slow_nopool,hog50,hog")
    ap.add_argument("--loops", type=int, default=100000)
    ap.add_argument("--json", default="")
    ap.add_argument("--serve", action="store_true", help="(内部用)別プロセスでサーバーだけを起動する")
    ap.add_argument("--upstream-url", default="")
    ap.add_argument("--hog-duty", type=float, default=0.0)
    ap.add_argument("--nodelay", action="store_true", help="サーバーのHandlerに disable_nagle_algorithm=True を付けて比べる(ヘッダと本文を2回に分けて送る書き方とNagle/遅延ACKの影響の確認用)")
    args = ap.parse_args()
    args.handlers = [x for x in args.handlers.split(",") if x]
    args.conds = [x for x in args.conds.split(",") if x]
    if args.serve:
        serve_mode(args)
        return
    pkg_root = Path(args.pkg_root).resolve()
    out = {"python": sys.version.split()[0], "platform": sys.platform, "pkg_root": str(pkg_root), "n": args.n}
    print(f"# pkg_root={pkg_root}  python={out['python']}  n={args.n} pool={args.pool}")
    if args.part in ("http", "all"):
        print("[http] 運転POST往復(ms)  サーバーは別プロセス")
        out["http"] = part_http(args, pkg_root)
    if args.part in ("profile", "all"):
        tmp = Path(tempfile.mkdtemp(prefix="bench_drive_"))
        rig = Rig(pkg_root, tmp)
        print(f"# 読み込んだ dashboard_common: {rig.pkg_file}")
        try:
            print("[profile] サーバー内の内訳")
            out["profile"] = part_profile(args, rig)
        finally:
            rig.close()
    if args.part in ("loop", "all"):
        print("[loop] director / arbiter")
        out["loop"] = part_loop(args, pkg_root)
    if args.json:
        Path(args.json).write_text(json.dumps(out, indent=1, ensure_ascii=False), encoding="utf-8")


if __name__ == "__main__":
    main()
