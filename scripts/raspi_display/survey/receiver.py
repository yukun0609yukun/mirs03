#!/usr/bin/env python3
"""
MIRS ロボットアンケート 受信サーバ (Jetson / Raspberry Pi 用)

[のこさへん] mirs-robot-survey の receiver.py をラズパイ(腹部ディスプレイ)用に取り込んだもの。
変更点は2つだけ: (1)回答の保存先を環境変数 SURVEY_DATA_DIR で指定可能に(既定 ~/survey_data)、
(2)待受を --host で指定可能に(既定 127.0.0.1=ラズパイ自身の画面からのみ)。

標準ライブラリだけで動きます (pip install 不要)。インターネット接続も不要です。
  - アンケート画面 (mirs_robot_survey.html) を配信する
  - POST /api/responses       … 回答を受け取り data/ に保存 (JSONL と CSV)
  - GET  /api/responses.csv   … 集計用 CSV をダウンロード
  - GET  /api/status          … 件数の確認
  - GET  /api/session/start   … 回答開始の通知 (on_session_start を書き換えて走行停止などに使う)
  - GET  /api/session/end     … 回答終了の通知
  - GET  /state              … [のこさへん追加] 読み取り専用の状態JSON(ハブ側 198.51.100.1 でも待受。下記)
  - POST /session/start, /session/end … 上の /api/session/* と同じ(POSTでも可)
  - GET  /api/condition?c=…   … 実験条件名の通知(HTMLが送る。/state の condition に出る)

[のこさへん追加] ハブ(198.51.100.0/24)連携:
  --also-host 198.51.100.1 を付けると、ハブ側NICでも待受する(ラズパイの eth0 が後から上がっても
  バックグラウンドで再試行する)。ハブ側から許すのは GET /state だけ。回答POST・セッション通知・
  CSV・HTML配信などはループバック(127.0.0.1=ラズパイ自身の画面)からのみで、それ以外は403。
  /state: {"mode","active","count","last_activity_age_s","condition","updated",
           "session_state","saved_seq","last_saved_age_s"}
  [2026-10-09] 「保存完了」連動: 回答中(active=true)は、回答が responses.jsonl/csv に書けた時(=POST /api/responses
  が成功した時)に終わる。session_state は idle(誰も操作していない) / answering(回答中。active=true は
  この間だけ) / saved(保存完了の直後 SAVED_SHOW_S 秒だけ) / abandoned(無通知で放棄タイムアウト。
  ABANDONED_SHOW_S 秒だけ)。放棄タイムアウトは既定120秒(--abandon-timeout か環境変数
  SURVEY_ABANDON_TIMEOUT_S で変更)。saved_seq は保存完了のたびに増える単調増加カウンタ。

起動:
    python3 receiver.py              # ポート 8080
    python3 receiver.py --port 8000

Jetson 自身の画面なら http://localhost:8080/ 、
同じネットワークのタブレットなら http://<Jetson の IP>:8080/ を開きます。
"""
import argparse
import csv
import json
import os
import ipaddress
import threading
import time
import uuid
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BASE = os.path.dirname(os.path.abspath(__file__))
DATA = os.environ.get("SURVEY_DATA_DIR") or os.path.expanduser("~/survey_data")
PAGE = os.path.join(BASE, "mirs_robot_survey.html")
JSONL = os.path.join(DATA, "responses.jsonl")
CSV_PATH = os.path.join(DATA, "responses.csv")
FIXED = ["received_at", "response_id", "submitted_at", "survey_version", "condition",
         "location", "duration_sec"]
lock = threading.Lock()
MODE_FILE = os.environ.get("KIOSK_MODE_FILE") or os.path.expanduser("~/.kiosk_mode")
COND_PATH = os.path.join(DATA, "condition.txt")


def _env_float(name, default):
    try:
        v = float(os.environ.get(name, ""))
        return v if v > 0 else default
    except ValueError:
        return default


# 回答中の通知(HTMLが5秒ごと)が途絶えてから「放棄」とみなすまでの秒数。考え込んでいる間にロボットが
# 動き出さないよう長め(120秒)。HTMLが壊れても永久には止めないための上限(--abandon-timeout で調整)
SESSION_TIMEOUT_S = _env_float("SURVEY_ABANDON_TIMEOUT_S", 120.0)
SAVED_SHOW_S = 3.0         # 保存完了(saved)を /state に出しておく秒数(ロボット側が確実に観測できるように)
ABANDONED_SHOW_S = 10.0    # 放棄(abandoned)を /state に出しておく秒数
HUB_NET = ipaddress.ip_network("198.51.100.0/24")


# ---------------------------------------------------------------------------
# ロボット連携: 必要に応じて書き換えてください
# ---------------------------------------------------------------------------
def on_session_start():
    """来場者がアンケートを開始した。例: 走行を一時停止する、発話を止める"""
    print(f"[{now()}] session start")


def on_session_end():
    """アンケートが終了した (完了・放置どちらも)。例: 巡回を再開する"""
    print(f"[{now()}] session end")
# ---------------------------------------------------------------------------


def now():
    return datetime.now().isoformat(timespec="seconds")


def load_all():
    """responses.jsonl を全件読む。【2026-10-21】壊れた行(書き込み中の電源断・ディスク満杯で途中までの行など)は、
    飛ばして警告するだけにする。以前は1行でも壊れていると例外になり、以後の回答がすべて保存できなくなった(save_recordが毎回失敗し、
    回答中のままロボットが120秒止まる)。"""
    if not os.path.exists(JSONL):
        return []
    out = []
    bad = 0
    with open(JSONL, encoding="utf-8", errors="replace") as f:
        for l in f:
            if not l.strip():
                continue
            try:
                rec = json.loads(l)
            except ValueError:
                bad += 1
                continue
            if isinstance(rec, dict):
                out.append(rec)
            else:
                bad += 1
    if bad:
        print(f"[{now()}] 警告: responses.jsonl に読めない行が {bad} 件あります(飛ばしました。元のファイルは変更していません)")
    return out


def csv_safe(v):
    """【2026-10-21】CSVの数式インジェクション対策。文字列が = + - @ やタブ・改行で始まると、Excelが数式として実行しうるため、
    先頭に ' を付けて文字列として扱わせる(数値はそのまま)。"""
    if isinstance(v, str) and v[:1] in ("=", "+", "-", "@", "\t", "\r", "\n"):
        return "'" + v
    return v


def write_csv(records):
    """全件から CSV を作り直す (設問が増えても列がずれない)"""
    ids = []
    for r in records:
        for k in r.get("answers", {}):
            if k not in ids:
                ids.append(k)
    with open(CSV_PATH, "w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(FIXED + ids)
        for r in records:
            a = r.get("answers", {})
            if not isinstance(a, dict):
                a = {}
            row = [csv_safe(r.get(k, "")) for k in FIXED]
            for i in ids:
                v = a.get(i, "")
                row.append(csv_safe(" / ".join(map(str, v)) if isinstance(v, list) else v))
            w.writerow(row)


# ---------------------------------------------------------------------------
# [のこさへん追加] 状態管理と送信元判定(純粋なロジック。テストは test_survey_admin.py)
# ---------------------------------------------------------------------------
def is_loopback(ip):
    """送信元がラズパイ自身(127.0.0.0/8, ::1)か。判定できなければ False"""
    try:
        return ipaddress.ip_address(str(ip).split("%")[0]).is_loopback
    except ValueError:
        return False


def is_hub(ip):
    """送信元がハブ内(198.51.100.0/24)か"""
    try:
        return ipaddress.ip_address(str(ip).split("%")[0]) in HUB_NET
    except ValueError:
        return False


def allowed(ip, method, path):
    """許可するか。読み取り GET /state はループバックとハブ内、それ以外は全てループバックのみ"""
    if is_loopback(ip):
        return True
    return method == "GET" and path.split("?")[0] == "/state" and is_hub(ip)


class Session:
    """回答セッションの状態。clock は時刻関数(テストで差し替える)

    session_state: idle → answering(start通知) → saved(回答の保存に成功) → idle(SAVED_SHOW_S秒後)
                                              ↘ abandoned(end通知 か 通知途絶えが timeout 秒) → idle(ABANDONED_SHOW_S秒後)
    active() は answering の間だけ true(従来の意味)。
    """

    def __init__(self, clock=time.time, timeout=None):
        self.clock = clock
        self.timeout = SESSION_TIMEOUT_S if timeout is None else float(timeout)
        self.lock = threading.Lock()
        self.phase = "idle"
        self.since = None          # 現在の phase になった時刻
        self.last_start = None     # 最後の start 通知(heartbeat含む)
        self.last_activity = None  # 最後の start/end/回答 の時刻
        self.saved_seq = 0         # 保存完了の回数(単調増加)
        self.saved_at = None
        self.condition = None

    def _resolve(self, t):
        """時間で自動的に進む遷移(呼び出し側がロックを持つ)"""
        if self.phase == "answering" and t - self.last_start >= self.timeout:
            self.phase, self.since = "abandoned", self.last_start + self.timeout
        if self.phase == "saved" and t - self.since >= SAVED_SHOW_S:
            self.phase, self.since = "idle", t
        if self.phase == "abandoned" and t - self.since >= ABANDONED_SHOW_S:
            self.phase, self.since = "idle", t

    def start(self, condition=None):
        with self.lock:
            t = self.clock()
            self.last_start, self.last_activity = t, t
            if self.phase != "answering":
                self.phase, self.since = "answering", t
            if condition:
                self.condition = condition

    def end(self, condition=None):
        """end通知(HTMLが回答途中で放置されて待機画面に戻った時など)。回答中なら放棄扱い"""
        with self.lock:
            t = self.clock()
            self._resolve(t)
            if self.phase == "answering":
                self.phase, self.since = "abandoned", t
            self.last_activity = t
            if condition:
                self.condition = condition

    def touch(self):
        with self.lock:
            self.last_activity = self.clock()

    def saved(self):
        """回答の保存(ディスクへの書き込み)に成功した。回答中だったなら saved に移る"""
        with self.lock:
            t = self.clock()
            self._resolve(t)
            self.saved_seq += 1
            self.saved_at = self.last_activity = t
            if self.phase == "answering":
                self.phase, self.since = "saved", t

    def state(self):
        with self.lock:
            self._resolve(self.clock())
            return self.phase

    def active(self):
        return self.state() == "answering"


session = Session()


def read_mode():
    try:
        with open(MODE_FILE, encoding="utf-8") as f:
            m = f.read().strip()
    except OSError:
        m = ""
    return "survey" if m == "survey" else "customer"


def count_responses():
    """回答件数(JSONLの空でない行数。ファイルが無ければ0)"""
    n = 0
    try:
        with open(JSONL, encoding="utf-8") as f:
            for l in f:
                if l.strip():
                    n += 1
    except OSError:
        pass
    return n


def read_condition():
    if session.condition:
        return session.condition
    try:
        with open(COND_PATH, encoding="utf-8") as f:
            return f.read().strip()[:80] or None
    except OSError:
        return None


def set_condition(c):
    c = (c or "").strip()[:80]
    if not c:
        return
    session.condition = c
    try:
        os.makedirs(DATA, exist_ok=True)
        with open(COND_PATH, "w", encoding="utf-8") as f:
            f.write(c)
    except OSError:
        pass


def build_state(sess=None, now_t=None):
    sess = sess or session
    t = sess.clock() if now_t is None else now_t
    age = None if sess.last_activity is None else round(max(0.0, t - sess.last_activity), 1)
    sa = None if sess.saved_at is None else round(max(0.0, t - sess.saved_at), 1)
    return {"mode": read_mode(), "active": sess.active(), "count": count_responses(),
            "last_activity_age_s": age, "condition": read_condition(), "updated": int(t),
            "session_state": sess.state(), "saved_seq": sess.saved_seq, "last_saved_age_s": sa}


def save_record(p):
    """回答1件をディスク(responses.jsonl と responses.csv)へ保存する。(ok, duplicate, エラー文字列)。
    ok=True はJSONLもCSVも書けた時だけ(「保存完了」の定義)。重複 response_id は書かずに ok+duplicate
    (HTMLの再送で二重保存しない。CSVは作り直して前回の書き損じも直す)。例外は投げず ok=False で返す"""
    try:
        with lock:
            records = load_all()
            # 【2026-10-21】response_id が無い回答は、None同士が「同じ」と判定されて2件目以降が重複として捨てられ、
            # 保存していないのに「保存完了」を返していた。受信側で一意のIDを付けて、別の回答として保存する。
            if not p.get("response_id"):
                p["response_id"] = "srv-" + uuid.uuid4().hex
            if any(r.get("response_id") == p.get("response_id") for r in records):
                write_csv(records)
                return True, True, None
            p["received_at"] = now()
            os.makedirs(DATA, exist_ok=True)
            with open(JSONL, "a", encoding="utf-8") as f:
                # 前回の書き込みが途中で切れて末尾に改行が無いと、新しい回答がその壊れた行の後ろにつながって、一緒に読めなくなる。
                # 先に改行を入れて、新しい回答を必ず新しい行にする。
                if f.tell() > 0:
                    with open(JSONL, "rb") as rf:
                        rf.seek(-1, os.SEEK_END)
                        if rf.read(1) != b"\n":
                            f.write("\n")
                f.write(json.dumps(p, ensure_ascii=False) + "\n")
                f.flush()
                os.fsync(f.fileno())
            records.append(p)
            write_csv(records)
        return True, False, None
    except Exception as e:  # noqa: BLE001  (ディスク満杯・権限・壊れたJSONLなど)
        return False, False, f"{type(e).__name__}: {e}"[:200]


def query_param(path, key):
    from urllib.parse import urlparse, parse_qs
    v = parse_qs(urlparse(path).query).get(key)
    return v[0] if v else None


class Handler(BaseHTTPRequestHandler):
    def cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def json(self, code, obj):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.cors()
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def end_headers(self):
        # 設定変更がすぐ反映されるよう、ブラウザにキャッシュさせない
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(204)
        self.cors()
        self.end_headers()

    def guard(self, method):
        """送信元アドレスで判定(198.51.100.x からは GET /state 以外を403)"""
        if allowed(self.client_address[0], method, self.path):
            return True
        self.json(403, {"ok": False, "error": "forbidden"})
        return False

    def do_POST(self):
        if not self.guard("POST"):
            return
        p0 = self.path.split("?")[0]
        if p0 in ("/session/start", "/session/end"):
            return self.session_event(p0.endswith("start"))
        if p0 != "/api/responses":
            return self.json(404, {"ok": False})
        try:
            n = int(self.headers.get("Content-Length", 0))
            p = json.loads(self.rfile.read(n).decode("utf-8"))
            if not isinstance(p, dict):
                raise ValueError("body must be a JSON object")
        except Exception as e:  # noqa: BLE001
            return self.json(400, {"ok": False, "error": str(e)})
        ok, dup, err = save_record(p)
        if not ok:  # 保存に失敗: 保存完了にしない(answering のまま。ロボットは止まり続ける)
            print(f"[{now()}] save FAILED {p.get('response_id')}: {err}")
            return self.json(500, {"ok": False, "error": err})
        # ?replay=1 は過去の未送信分の再送(今の回答者の完了ではない)ので保存完了の通知にしない
        if query_param(self.path, "replay") == "1":
            session.touch()
        else:
            session.saved()
        if not dup:
            print(f"[{now()}] saved {p.get('response_id')} ({p.get('condition')})")
        self.json(200, {"ok": True, "duplicate": True} if dup else {"ok": True})

    def session_event(self, is_start):
        c = query_param(self.path, "condition")
        if is_start:
            session.start(c)
            on_session_start()
        else:
            session.end(c)
            on_session_end()
        if c:
            set_condition(c)
        return self.json(200, {"ok": True})

    def do_GET(self):
        if not self.guard("GET"):
            return
        p0 = self.path.split("?")[0]
        if p0 == "/state":
            return self.json(200, build_state())
        if p0 == "/api/condition":
            set_condition(query_param(self.path, "c"))
            return self.json(200, {"ok": True})
        if self.path == "/api/status":
            return self.json(200, {"ok": True, "count": count_responses()})
        if self.path == "/api/responses.csv":
            with lock:
                write_csv(load_all())
            with open(CSV_PATH, "rb") as f:
                body = f.read()
            self.send_response(200)
            self.cors()
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Disposition", "attachment; filename=robot_survey.csv")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            return self.wfile.write(body)
        if p0 == "/api/session/start":
            return self.session_event(True)
        if p0 == "/api/session/end":
            return self.session_event(False)
        if self.path.split("?")[0] in ("/", "/index.html", "/mirs_robot_survey.html"):
            with open(PAGE, "rb") as f:  # 設定を変えたらブラウザを再読み込みするだけで反映
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            return self.wfile.write(body)
        return self.json(404, {"ok": False})  # 回答データなど他のファイルは配信しない

    def log_message(self, fmt, *args):
        if "/api/" in self.path and not self.path.startswith("/api/session"):
            return  # 回答の保存はすでに出力しているので省略
        if self.path.startswith("/state"):
            return  # 管理画面が数秒おきに読むので省略
        super().log_message(fmt, *args)


def serve_also(host, port):
    """ハブ側NICで待受(NICが未起動でも上がるまで10秒ごとに再試行)"""
    while True:
        try:
            srv = ThreadingHTTPServer((host, port), Handler)
        except OSError as e:
            print(f"[{now()}] {host}:{port} 待受できず({e})。10秒後に再試行")
            time.sleep(10)
            continue
        print(f"[{now()}] ハブ側待受 http://{host}:{port}/state (読み取りのみ)")
        srv.serve_forever()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--abandon-timeout", type=float, default=0,
                    help="回答中の通知が途絶えてから放棄とみなす秒数(既定120。環境変数 SURVEY_ABANDON_TIMEOUT_S でも可)")
    ap.add_argument("--also-host", default="", help="ハブ側NICのアドレス(例 198.51.100.1)。GET /state だけ許可")
    args = ap.parse_args()
    if args.abandon_timeout > 0:
        session.timeout = args.abandon_timeout
    print(f"MIRS robot survey: http://{args.host}:{args.port}/  (保存先 {DATA})")
    if args.also_host:
        threading.Thread(target=serve_also, args=(args.also_host, args.port), daemon=True).start()
    ThreadingHTTPServer((args.host, args.port), Handler).serve_forever()
