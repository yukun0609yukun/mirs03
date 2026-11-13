#!/usr/bin/env python3
"""腹部ディスプレイ用の小さな認証代行プロキシ(ラズパイ上、標準ライブラリのみ)。

Chromiumを `http://ユーザー:パスワード@ホスト/...` で開くと、ページ内の fetch("/api/...")
が「URLに認証情報が含まれる」として拒否され、画面が「通信エラー」のままになる。
そこで 127.0.0.1:8081 で待ち受け、受けたリクエストをJetson(web_ui)へそのまま転送
しつつ、Basic認証のヘッダーをこちらで付与する。Chromiumは認証情報の無い
`http://127.0.0.1:8081/customer_display` を開くだけでよい。

設定は ~/.kiosk_env(KIOSK_HOSTS / KIOSK_USER / KIOSK_PASS)から読む。
転送先は KIOSK_HOSTS を順に試し、繋がった最初のホストを使う(WiFi/有線の切り替え)。
127.0.0.1にしか待ち受けないので、外部からは使えない。
"""
import base64
import http.client
import os
import shlex
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

LISTEN = ("127.0.0.1", int(os.environ.get("KIOSK_PROXY_PORT", "8081")))
HOP_BY_HOP = {
    "connection", "keep-alive", "proxy-authenticate", "proxy-authorization",
    "te", "trailers", "transfer-encoding", "upgrade",
}


def load_env(path):
    env = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, value = line.partition("=")
            try:
                parts = shlex.split(value)
            except ValueError:
                parts = [value]
            env[key.strip()] = parts[0] if parts else ""
    return env


ENV = load_env(os.path.expanduser("~/.kiosk_env"))
HOSTS = ENV.get("KIOSK_HOSTS", "").split()
AUTH = "Basic " + base64.b64encode(
    f"{ENV.get('KIOSK_USER', '')}:{ENV.get('KIOSK_PASS', '')}".encode("utf-8")
).decode("ascii")


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):  # noqa: A003 - 静かに動かす
        pass

    def _forward(self):
        length = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(length) if length else None
        headers = {
            k: v for k, v in self.headers.items()
            if k.lower() not in HOP_BY_HOP | {"host", "authorization", "content-length"}
        }
        headers["Authorization"] = AUTH
        last_error = None
        for host in HOSTS:
            conn = None
            try:
                h, _, port = host.partition(":")
                conn = http.client.HTTPConnection(h, int(port or 80), timeout=10)
                conn.request(self.command, self.path, body=body, headers=headers)
                res = conn.getresponse()
                data = res.read()
            except Exception as exc:  # noqa: BLE001 - 次のホストを試す
                last_error = exc
                if conn is not None:
                    conn.close()
                continue
            self.send_response(res.status)
            for k, v in res.getheaders():
                if k.lower() in HOP_BY_HOP | {"content-length", "www-authenticate"}:
                    continue
                self.send_header(k, v)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(data)
            conn.close()
            return
        msg = f"upstream unreachable: {last_error}".encode("utf-8")
        self.send_response(502)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(msg)))
        self.end_headers()
        self.wfile.write(msg)

    do_GET = do_POST = do_PUT = do_DELETE = do_HEAD = _forward


if __name__ == "__main__":
    if not HOSTS:
        sys.exit("KIOSK_HOSTS is empty")
    ThreadingHTTPServer(LISTEN, Handler).serve_forever()
