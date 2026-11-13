"""BgmProxy(voice_bridge /bgm中継)の単体テスト。スタブのhttp.serverに対して検証する。

実行: python ros2_ws/src/nokosahen_core/test/test_bgm_proxy.py
(rclpy不要。nokosahen_core.bgm_proxyは標準ライブラリのみ。)
"""

import json
import sys
import threading
import unittest
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nokosahen_core.bgm_proxy import BgmProxy  # noqa: E402

STATE = {"enabled": True, "volume": 0.3, "duck_volume": 0.1, "playing": True, "files": ["a.mp3"]}
POSTS = []
MODE = {"fail_status": 0}


class _Stub(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if MODE["fail_status"]:
            self._send(MODE["fail_status"], {})
        elif self.path == "/bgm":
            self._send(200, STATE)
        else:
            self._send(404, {})

    def do_POST(self):
        n = int(self.headers.get("Content-Length", "0"))
        data = json.loads(self.rfile.read(n))
        POSTS.append(data)
        STATE.update(data)
        self._send(200, STATE)


class BgmProxyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = HTTPServer(("127.0.0.1", 0), _Stub)
        threading.Thread(target=cls.server.serve_forever, daemon=True).start()
        cls.url = f"http://127.0.0.1:{cls.server.server_port}/"  # 末尾スラッシュも許容

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()

    def setUp(self):
        POSTS.clear()
        MODE["fail_status"] = 0
        self.p = BgmProxy(self.url, timeout_s=1.0)

    def test_unconfigured(self):
        p = BgmProxy("")
        self.assertFalse(p.configured)
        r = p.get_status()
        self.assertFalse(r["ok"])
        self.assertFalse(r["configured"])
        self.assertEqual(p.set({"enabled": True})[1]["configured"], False)

    def test_get_status(self):
        r = self.p.get_status()
        self.assertTrue(r["ok"])
        self.assertEqual(r["status"]["files"], ["a.mp3"])

    def test_get_http_error_and_unreachable(self):
        MODE["fail_status"] = 500
        r = self.p.get_status()
        self.assertFalse(r["ok"])
        self.assertIn("500", r["error"])
        dead = BgmProxy("http://127.0.0.1:1", timeout_s=0.5).get_status()
        self.assertFalse(dead["ok"])
        self.assertIn("接続できません", dead["error"])

    def test_set_forwards_and_clamps(self):
        code, r = self.p.set({"enabled": False, "volume": 9, "duck_volume": -2})
        self.assertEqual(code, 200)
        self.assertTrue(r["ok"])
        self.assertEqual(POSTS[-1], {"enabled": False, "volume": 5.0, "duck_volume": 0.0})

    def test_set_partial_and_int(self):
        code, _ = self.p.set({"volume": 1})
        self.assertEqual(code, 200)
        self.assertEqual(POSTS[-1], {"volume": 1.0})

    def test_set_rejects_bad_input(self):
        bad = [
            {}, [], None,
            {"enabled": 1}, {"enabled": "true"},
            {"volume": True}, {"volume": "0.5"}, {"volume": None},
            {"volume": float("nan")}, {"duck_volume": float("inf")},
            {"unknown": 1},
        ]
        for data in bad:
            code, r = self.p.set(data)
            self.assertEqual(code, 400, data)
            self.assertFalse(r["ok"])
        self.assertEqual(POSTS, [])  # 不正入力はvoice_bridgeへ転送されない

    def test_ignores_unknown_keys(self):
        self.p.set({"volume": 0.5, "evil": "x"})
        self.assertEqual(POSTS[-1], {"volume": 0.5})


if __name__ == "__main__":
    unittest.main()
