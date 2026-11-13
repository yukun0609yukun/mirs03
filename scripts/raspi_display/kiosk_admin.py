#!/usr/bin/env python3
"""
腹部ディスプレイ 管理用API (ラズパイ上。標準ライブラリのみ)

1台目の総合ダッシュボード(管理画面)から、ラズパイの表示モード切替と状態確認をするための小さなAPI。
ハブ側NIC(既定 198.51.100.1)のポート8083で待受し、**呼び出し元は 198.51.100.2(1台目)だけ**許可する
(送信元アドレスで判定。それ以外は403)。リクエストはダッシュボードが認証済みユーザーの操作として中継する。

  GET  /state            アンケート受信サーバ(127.0.0.1:8082 /state)の内容 + receiver の生死
                         {"mode","active","count","last_activity_age_s","condition","updated","receiver":bool,
                          "session_state","saved_seq","last_saved_age_s"}   (receiverの値をそのまま中継。落ちていれば idle/null)
                         receiver が落ちていても mode と回答件数(ファイルを直接数える)は返す
  POST /mode             {"mode":"survey"|"customer"|"ocrtest"} → switch_kiosk.sh を呼んで表示を切り替える
                         (すでに同じモードなら何もしない。ボディはこのJSONだけ。他は400)
  POST /ocr_mock         {"action":"randomize"|"reset"|"set","values":[7個の整数0〜9999]}  (valuesはsetのみ)
                         模擬POS画面(ocrtestモード)の数字を変える。~/ocr_test/ocr_cmd.json を原子的に書き、
                         画面側(ocr_test/mock_pos_tablet.html)が1秒おきに読んで実行する。
                         ocrtestモードでなければ409。GET /state にも ocr_mock:{seq,action,at} を載せる
  POST /ocr_mock_ack     {"seq":N,"values":[7個の整数0〜9999]}  【2026-10-13追加】模擬POS画面(ラッパーHTML)が「適用した
                         seq」と「画面に実際に出ている7つの数字」を報告する読み戻し。**送信元がループバック(127.0.0.1)の
                         ときだけ**受け付ける(ラッパーは同じラズパイのChromium。別ポートなのでCORSは http://127.0.0.1:8084
                         だけ許可)。~/ocr_test/ocr_ack.json に原子的に保存し(変化があった時だけ。心拍は更新時刻をメモリに持つ)、
                         GET /state の ocr_mock に ack_seq / shown_values / acked_at / ack_age_s を足す(ackが一度も無ければ従来の形)。
                         この受付は 127.0.0.1:<ポート> の専用待受(サーバは198.51.100.1と127.0.0.1の2つ。後者はこのPOSTだけ)。
  GET  /responses.csv    回答CSV(上限 2MB。個人情報を持たない設計)

起動: python3 kiosk_admin.py --host 198.51.100.1 --port 8083   (start_kiosk.sh が起動する)
"""
import argparse
import ipaddress
import json
import os
import subprocess
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ALLOWED_CLIENTS = {"198.51.100.2"}          # 1台目(有線)のみ
MODES = ("survey", "customer", "ocrtest")   # ocrtest = OCRテスト用の模擬POS画面
MAX_BODY = 256                            # POST /mode のボディ上限(バイト)
MAX_OCR_BODY = 512                        # POST /ocr_mock のボディ上限(バイト)
OCR_ACTIONS = ("randomize", "reset", "set")
OCR_ROWS = 7                              # 模擬POSの行数(A/B/C/カレー/麺類/アラカルトB/アラカルト100円)
OCR_MAX_VALUE = 9999
MAX_ACK_BODY = 512                        # POST /ocr_mock_ack のボディ上限(バイト)
ACK_ORIGIN = "http://127.0.0.1:8084"      # ラッパーHTML(python3 -m http.server 8084 --bind 127.0.0.1)のオリジン。CORSで許す唯一
MAX_CSV = 2 * 1024 * 1024
RECEIVER = "http://127.0.0.1:8082"
HOME = os.path.expanduser("~")
MODE_FILE = os.environ.get("KIOSK_MODE_FILE") or os.path.join(HOME, ".kiosk_mode")
DATA = os.environ.get("SURVEY_DATA_DIR") or os.path.join(HOME, "survey_data")
SWITCH = os.environ.get("SWITCH_SCRIPT") or os.path.join(HOME, "switch_kiosk.sh")
OCR_DIR = os.environ.get("OCR_TEST_DIR") or os.path.join(HOME, "ocr_test")
OCR_CMD_FILE = os.path.join(OCR_DIR, "ocr_cmd.json")
OCR_ACK_FILE = os.path.join(OCR_DIR, "ocr_ack.json")
switch_lock = threading.Lock()
ocr_lock = threading.Lock()
ack_lock = threading.Lock()
_ack_mem = None                           # 最後のack {"ack_seq","shown_values","acked_at"}(心拍で更新時刻だけ変わるものはファイルに書かない)


# ---------------------------------------------------------------------------
# 純粋なロジック(テストは test_survey_admin.py)
# ---------------------------------------------------------------------------
def client_allowed(ip, allowed=None):
    """送信元IPが許可リストにあるか。IPv4射影アドレス(::ffff:a.b.c.d)も正規化して比べる"""
    allowed = ALLOWED_CLIENTS if allowed is None else allowed
    try:
        a = ipaddress.ip_address(str(ip).split("%")[0])
    except ValueError:
        return False
    if getattr(a, "ipv4_mapped", None):
        a = a.ipv4_mapped
    return str(a) in allowed


def loopback_allowed(ip):
    """送信元がループバック(127.0.0.0/8, ::1, ::ffff:127.x)か。ack(POST /ocr_mock_ack)はこれだけ許す"""
    try:
        a = ipaddress.ip_address(str(ip).split("%")[0])
    except ValueError:
        return False
    if getattr(a, "ipv4_mapped", None):
        a = a.ipv4_mapped
    return a.is_loopback


def parse_ocr_ack_body(raw):
    """POST /ocr_mock_ack のボディを検証して (seq, values, エラー文字列) を返す。
    許すのは {"seq": 0以上の整数, "values": [7個の整数 0〜9999]} のみ(余計な項目・bool・小数・文字列は不可)"""
    if raw is None or len(raw) > MAX_ACK_BODY:
        return None, None, "body too large"
    try:
        obj = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        return None, None, "invalid json"
    if not isinstance(obj, dict) or set(obj) != {"seq", "values"}:
        return None, None, 'body must be {"seq":N,"values":[...]}'
    seq = obj["seq"]
    if isinstance(seq, bool) or not isinstance(seq, int) or seq < 0 or seq > 2 ** 53:
        return None, None, "seq must be a non-negative integer"
    v = obj["values"]
    if not isinstance(v, list) or len(v) != OCR_ROWS:
        return None, None, f"values must be a list of {OCR_ROWS} integers"
    for x in v:
        if isinstance(x, bool) or not isinstance(x, int) or x < 0 or x > OCR_MAX_VALUE:
            return None, None, f"each value must be an integer 0..{OCR_MAX_VALUE}"
    return seq, list(v), None


def parse_mode_body(raw):
    """POST /mode のボディを検証して (mode, エラー文字列) を返す。許すのは {"mode": survey|customer|ocrtest} のみ"""
    if raw is None or len(raw) > MAX_BODY:
        return None, "body too large"
    try:
        obj = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        return None, "invalid json"
    if not isinstance(obj, dict) or set(obj) != {"mode"}:
        return None, 'body must be {"mode": ...}'
    m = obj["mode"]
    if not isinstance(m, str) or m not in MODES:
        return None, "mode must be survey, customer or ocrtest"
    return m, None


def parse_ocr_mock_body(raw):
    """POST /ocr_mock のボディを検証して (action, values or None, エラー文字列) を返す。
    許すのは {"action": randomize|reset} か {"action": "set", "values": [7個の整数 0〜9999]} のみ"""
    if raw is None or len(raw) > MAX_OCR_BODY:
        return None, None, "body too large"
    try:
        obj = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        return None, None, "invalid json"
    if not isinstance(obj, dict) or "action" not in obj:
        return None, None, 'body must be {"action": ...}'
    action = obj["action"]
    if not isinstance(action, str) or action not in OCR_ACTIONS:
        return None, None, "action must be randomize, reset or set"
    if action != "set":
        if set(obj) != {"action"}:
            return None, None, "values are only allowed for set"
        return action, None, None
    if set(obj) != {"action", "values"}:
        return None, None, 'set needs {"action":"set","values":[...]}'
    v = obj["values"]
    if not isinstance(v, list) or len(v) != OCR_ROWS:
        return None, None, f"values must be a list of {OCR_ROWS} integers"
    for x in v:
        # bool は int のサブクラスなので除外。小数・文字列も不可
        if isinstance(x, bool) or not isinstance(x, int) or x < 0 or x > OCR_MAX_VALUE:
            return None, None, f"each value must be an integer 0..{OCR_MAX_VALUE}"
    return action, list(v), None


def read_ocr_cmd(path=None):
    """現在の ocr_cmd.json を読む(無い・壊れていれば None)"""
    try:
        with open(path or OCR_CMD_FILE, encoding="utf-8") as f:
            d = json.load(f)
        return d if isinstance(d, dict) else None
    except (OSError, ValueError):
        return None


def _valid_ack(d):
    if not isinstance(d, dict) or isinstance(d.get("ack_seq"), bool) or not isinstance(d.get("ack_seq"), int):
        return None
    v = d.get("shown_values")
    if not isinstance(v, list) or len(v) != OCR_ROWS or any(isinstance(x, bool) or not isinstance(x, int) for x in v):
        return None
    at = d.get("acked_at")
    if isinstance(at, bool) or not isinstance(at, int):
        return None
    return {"ack_seq": d["ack_seq"], "shown_values": list(v), "acked_at": at}


def read_ocr_ack():
    """模擬POS画面が最後に報告した {ack_seq, shown_values, acked_at}(無い・壊れていれば None)"""
    with ack_lock:
        if _ack_mem is not None:
            return dict(_ack_mem)
    try:
        with open(OCR_ACK_FILE, encoding="utf-8") as f:
            return _valid_ack(json.load(f))
    except (OSError, ValueError):
        return None


def record_ocr_ack(seq, values, now=None):
    """ackを記録する。seq・数字が前回と同じ(心拍)ならメモリの更新時刻だけ変え、変わった時だけ
    ~/ocr_test/ocr_ack.json を原子的に書く(SDカードへの書込みを減らす)。保存したackを返す"""
    global _ack_mem
    with ack_lock:
        prev = _ack_mem
        if prev is None:
            try:
                with open(OCR_ACK_FILE, encoding="utf-8") as f:
                    prev = _valid_ack(json.load(f))
            except (OSError, ValueError):
                prev = None
        ack = {"ack_seq": seq, "shown_values": list(values), "acked_at": int(time.time() if now is None else now)}
        changed = prev is None or prev["ack_seq"] != seq or prev["shown_values"] != list(values)
        _ack_mem = ack
        if changed:
            os.makedirs(OCR_DIR, exist_ok=True)
            tmp = OCR_ACK_FILE + f".{os.getpid()}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(ack, f, ensure_ascii=False)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, OCR_ACK_FILE)
        return dict(ack)


def ocr_mock_summary(now=None):
    d = read_ocr_cmd()
    ack = read_ocr_ack()
    has_cmd = bool(d) and isinstance(d.get("seq"), int)
    if not has_cmd and ack is None:
        return None
    out = ({"seq": d["seq"], "action": d.get("action"), "at": d.get("at")} if has_cmd
           else {"seq": None, "action": None, "at": None})
    if ack is not None:
        # 画面が実行した結果(読み戻し)。ack_age_s=最後の報告からの秒数(心拍は約5秒おき。長く途切れたら画面が止まっている)
        out.update(ack, ack_age_s=max(0, int((time.time() if now is None else now) - ack["acked_at"])))
    return out


def write_ocr_cmd(action, values, now=None):
    """ocr_cmd.json を原子的に書く(一時ファイル→rename)。seq は時刻(ミリ秒)ベースで必ず前回より大きくする。
    画面側は seq が増えたときだけ実行する。書いた内容を返す"""
    with ocr_lock:
        prev = read_ocr_cmd()
        seq = int((time.time() if now is None else now) * 1000)
        if prev and isinstance(prev.get("seq"), int) and seq <= prev["seq"]:
            seq = prev["seq"] + 1
        cmd = {"seq": seq, "action": action, "at": int(seq // 1000)}
        if action == "set":
            cmd["values"] = values
        os.makedirs(OCR_DIR, exist_ok=True)
        tmp = OCR_CMD_FILE + f".{os.getpid()}.tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(cmd, f, ensure_ascii=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, OCR_CMD_FILE)
        return cmd


def read_mode():
    try:
        with open(MODE_FILE, encoding="utf-8") as f:
            m = f.read().strip()
            return m if m in MODES else "customer"
    except OSError:
        return "customer"


def count_lines(path):
    n = 0
    try:
        with open(path, encoding="utf-8") as f:
            n = sum(1 for l in f if l.strip())
    except OSError:
        pass
    return n


def fallback_state():
    """receiver が応答しないときの状態(モードと回答件数だけはファイルから)"""
    return {"mode": read_mode(), "active": False,
            "count": count_lines(os.path.join(DATA, "responses.jsonl")),
            "last_activity_age_s": None, "condition": None, "updated": int(time.time()),
            "session_state": "idle", "saved_seq": None, "last_saved_age_s": None,
            "receiver": False, "ocr_mock": ocr_mock_summary()}


def get_state():
    try:
        with urllib.request.urlopen(RECEIVER + "/state", timeout=2) as r:
            st = json.loads(r.read(65536).decode("utf-8"))
        st["receiver"] = True
        st["ocr_mock"] = ocr_mock_summary()
        st["mode"] = read_mode()   # receiverは survey/customer だけ返す。ocrtest は管理API側で上書き
        return st
    except Exception:  # noqa: BLE001  (落ちている・応答が壊れている)
        return fallback_state()


def switch_mode(mode):
    """表示モードを切り替える。(changed, エラー or None)"""
    if mode not in MODES:
        return False, "bad mode"
    with switch_lock:
        if read_mode() == mode:
            return False, None
        try:
            r = subprocess.run([SWITCH, mode], capture_output=True, timeout=10)
        except (OSError, subprocess.TimeoutExpired) as e:
            return False, f"switch failed: {e}"
        if r.returncode != 0:
            return False, f"switch_kiosk.sh exit {r.returncode}"
        return True, None


class Handler(BaseHTTPRequestHandler):
    def send_json(self, code, obj):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        if self.headers.get("Origin") == ACK_ORIGIN and self.path.split("?")[0] == "/ocr_mock_ack":
            self.send_header("Access-Control-Allow-Origin", ACK_ORIGIN)   # ラッパーが応答を読めるように(この1オリジンだけ)
        self.end_headers()
        self.wfile.write(body)

    def guard(self):
        if client_allowed(self.client_address[0]):
            return True
        self.send_json(403, {"ok": False, "error": "forbidden"})
        return False

    def do_OPTIONS(self):
        # ackのCORSプリフライト(ラッパーが application/json で送る場合)。許すのはACK_ORIGINからの /ocr_mock_ack だけ
        if self.path.split("?")[0] == "/ocr_mock_ack" and loopback_allowed(self.client_address[0]) \
                and self.headers.get("Origin") == ACK_ORIGIN:
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", ACK_ORIGIN)
            self.send_header("Access-Control-Allow-Methods", "POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Access-Control-Max-Age", "600")
            self.send_header("Content-Length", "0")
            self.end_headers()
            return
        self.send_json(403, {"ok": False, "error": "forbidden"})

    def do_GET(self):
        if not self.guard():
            return
        p = self.path.split("?")[0]
        if p == "/state":
            return self.send_json(200, dict(get_state(), ok=True))
        if p == "/responses.csv":
            try:
                with urllib.request.urlopen(RECEIVER + "/api/responses.csv", timeout=3) as r:
                    body = r.read(MAX_CSV + 1)
            except Exception:  # noqa: BLE001  (receiver不在ならファイルを直接)
                try:
                    with open(os.path.join(DATA, "responses.csv"), "rb") as f:
                        body = f.read(MAX_CSV + 1)
                except OSError:
                    return self.send_json(404, {"ok": False, "error": "no csv yet"})
            if len(body) > MAX_CSV:
                return self.send_json(413, {"ok": False, "error": "csv too large"})
            self.send_response(200)
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            return self.wfile.write(body)
        return self.send_json(404, {"ok": False})

    def do_POST(self):
        p = self.path.split("?")[0]
        if p == "/ocr_mock_ack":
            return self.post_ocr_ack()   # 送信元はループバックのみ(guard()の198.51.100.2制限とは別)
        if not self.guard():
            return
        if p == "/ocr_mock":
            return self.post_ocr_mock()
        if p != "/mode":
            return self.send_json(404, {"ok": False})
        try:
            n = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            n = -1
        if n < 0 or n > MAX_BODY:
            return self.send_json(400, {"ok": False, "error": "bad length"})
        mode, err = parse_mode_body(self.rfile.read(n))
        if err:
            return self.send_json(400, {"ok": False, "error": err})
        changed, err = switch_mode(mode)
        if err:
            return self.send_json(500, {"ok": False, "error": err})
        return self.send_json(200, dict(get_state(), ok=True, changed=changed, mode=read_mode()))

    def post_ocr_mock(self):
        try:
            n = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            n = -1
        if n < 0 or n > MAX_OCR_BODY:
            return self.send_json(400, {"ok": False, "error": "bad length"})
        action, values, err = parse_ocr_mock_body(self.rfile.read(n))
        if err:
            return self.send_json(400, {"ok": False, "error": err, "message": err})
        if read_mode() != "ocrtest":
            msg = "OCRテストモードではありません(先に腹部ディスプレイをOCRテストに切り替えてください)"
            return self.send_json(409, {"ok": False, "error": "not ocrtest mode", "message": msg})
        try:
            cmd = write_ocr_cmd(action, values)
        except OSError as e:
            return self.send_json(500, {"ok": False, "error": f"write failed: {e}"})
        return self.send_json(200, {"ok": True, "ocr_mock": {"seq": cmd["seq"], "action": cmd["action"], "at": cmd["at"]}})

    def post_ocr_ack(self):
        """模擬POS画面からの読み戻し報告。ループバック以外は403、Originがあれば ACK_ORIGIN 以外は403"""
        if not loopback_allowed(self.client_address[0]):
            return self.send_json(403, {"ok": False, "error": "forbidden"})
        origin = self.headers.get("Origin")
        if origin is not None and origin != ACK_ORIGIN:
            return self.send_json(403, {"ok": False, "error": "forbidden origin"})
        try:
            n = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            n = -1
        if n < 0 or n > MAX_ACK_BODY:
            return self.send_json(400, {"ok": False, "error": "bad length"})
        seq, values, err = parse_ocr_ack_body(self.rfile.read(n))
        if err:
            return self.send_json(400, {"ok": False, "error": err})
        try:
            record_ocr_ack(seq, values)
        except OSError as e:
            return self.send_json(500, {"ok": False, "error": f"write failed: {e}"})
        return self.send_json(200, {"ok": True})

    def log_message(self, fmt, *args):
        if "/state" in self.path:
            return
        super().log_message(fmt, *args)


class LoopbackHandler(Handler):
    """127.0.0.1側の専用待受。受け付けるのは POST /ocr_mock_ack(と そのプリフライト)だけ。
    管理用の操作(モード切替・模擬POS操作・CSV・状態)は一切できない"""

    def do_GET(self):
        self.send_json(404, {"ok": False})

    def do_POST(self):
        if self.path.split("?")[0] != "/ocr_mock_ack":
            return self.send_json(404, {"ok": False})
        return self.post_ocr_ack()


def serve_loopback(port):
    """ack専用の127.0.0.1待受(別スレッド)。起動できなくても管理APIには影響しない(10秒ごとに再試行)"""
    while True:
        try:
            srv = ThreadingHTTPServer(("127.0.0.1", port), LoopbackHandler)
        except OSError as e:
            print(f"127.0.0.1:{port} 待受できず({e})。10秒後に再試行", flush=True)
            time.sleep(10)
            continue
        print(f"kiosk_admin: http://127.0.0.1:{port}/ocr_mock_ack (ackのみ・ループバック限定)", flush=True)
        srv.serve_forever()


def serve(host, port):
    """NICが未起動でも上がるまで10秒ごとに再試行"""
    while True:
        try:
            srv = ThreadingHTTPServer((host, port), Handler)
        except OSError as e:
            print(f"{host}:{port} 待受できず({e})。10秒後に再試行", flush=True)
            time.sleep(10)
            continue
        print(f"kiosk_admin: http://{host}:{port}/ (許可: {sorted(ALLOWED_CLIENTS)})", flush=True)
        srv.serve_forever()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="198.51.100.1")
    ap.add_argument("--port", type=int, default=8083)
    ap.add_argument("--allow", action="append", help="許可する送信元IP(既定 198.51.100.2)。テスト用")
    a = ap.parse_args()
    if a.allow:
        ALLOWED_CLIENTS = set(a.allow)
    # 二重起動防止(eth0が下がっている間に start_kiosk.sh が何度も起動しようとしても1つだけ残す)
    try:
        import fcntl
        _lock = open(os.environ.get("KIOSK_ADMIN_LOCK", "/tmp/kiosk_admin.lock"), "w")
        fcntl.flock(_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except ImportError:
        pass
    except OSError:
        print("kiosk_admin: すでに起動中", flush=True)
        raise SystemExit(0)
    # 【2026-10-13追加】模擬POS画面のack受付(127.0.0.1)。--hostがすでに127.0.0.1(テスト)なら本体が兼ねる
    if a.host != "127.0.0.1":
        threading.Thread(target=serve_loopback, args=(a.port,), daemon=True).start()
    serve(a.host, a.port)
