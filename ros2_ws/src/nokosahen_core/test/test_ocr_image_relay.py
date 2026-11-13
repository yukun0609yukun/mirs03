"""管理画面によるOCRカメラ映像の中継と、アンケートCSVの制限のテスト(2026-10-21)。

- OCRの確認画面(8091)は、認証なしのため、1台目の内側(127.0.0.1)だけで待ち受ける。パソコンの管理画面(ログイン必須)が、
  カメラ映像(/api/ocr/preview.jpg)と認識画像(/api/ocr/processed.jpg)を中継する
- 回答CSV(/api/survey/responses.csv)は、閲覧専用のguestには出さない(管理者とoperatorだけ)
実行: python ros2_ws/src/nokosahen_core/test/test_ocr_image_relay.py
"""
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(TEST_DIR.parent))

import test_account_roles_http as H  # noqa: E402  (FakeTeleop・serve・http・operatorのハッシュを流用)
from nokosahen_core import accounts as acc  # noqa: E402
from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core.ocr_admin_proxy import OcrAdminProxy  # noqa: E402

JPEG = b"\xff\xd8\xff\xe0" + b"\x00" * 64 + b"\xff\xd9"
TMP = Path(tempfile.mkdtemp(prefix="ocr_relay_"))


class FakeOcr(BaseHTTPRequestHandler):
    mode = "ok"  # ok / none(映像なし) / notjpeg / huge

    def log_message(self, *a):
        pass

    def do_GET(self):  # noqa: N802
        if self.path.startswith("/api/preview.jpg") or self.path.startswith("/api/processed.jpg"):
            m = type(self).mode
            if m == "none":
                self.send_response(404)
                self.send_header("Content-Length", "2")
                self.end_headers()
                self.wfile.write(b"{}")
                return
            body = b"<html>x</html>" if m == "notjpeg" else (b"\xff" * 3_000_000 if m == "huge" else JPEG)
            self.send_response(200)
            self.send_header("Content-Type", "text/html" if m == "notjpeg" else "image/jpeg")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_response(404)
        self.send_header("Content-Length", "2")
        self.end_headers()
        self.wfile.write(b"{}")


class FakeSurvey:
    configured = True

    def get_state(self):
        return {"ok": True, "configured": True}

    def get_csv(self):
        return 200, b"a,b\n1,2\n", ""


def raw_get(url, cookie):
    req = urllib.request.Request(url, headers={"Cookie": cookie})
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            return r.status, r.read(), r.headers.get("Content-Type", "")
    except urllib.error.HTTPError as e:
        try:
            return e.code, e.read(), e.headers.get("Content-Type", "")
        finally:
            e.close()


class ProxyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv, cls.port = H.serve(FakeOcr)

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def setUp(self):
        FakeOcr.mode = "ok"
        self.p = OcrAdminProxy(f"http://127.0.0.1:{self.port}")

    def test_images_are_relayed(self):
        for kind in ("preview", "processed"):
            code, body = self.p.get_image(kind)
            self.assertEqual((code, body), (200, JPEG), kind)

    def test_no_frame_yet_is_404_and_not_jpeg_or_huge_is_502(self):
        FakeOcr.mode = "none"
        self.assertEqual(self.p.get_image("preview")[0], 404)
        FakeOcr.mode = "notjpeg"
        self.assertEqual(self.p.get_image("preview")[0], 502)
        FakeOcr.mode = "huge"
        self.assertEqual(self.p.get_image("preview")[0], 502)

    def test_unknown_kind_unconfigured_and_unreachable(self):
        self.assertEqual(self.p.get_image("../etc/passwd")[0], 404)  # 許す種類は固定(パスを受け付けない)
        self.assertEqual(OcrAdminProxy("").get_image("preview")[0], 404)
        self.assertEqual(OcrAdminProxy("http://127.0.0.1:9").get_image("preview")[0], 502)


class DashboardRelayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ocr_srv, cls.ocr_port = H.serve(FakeOcr)

    @classmethod
    def tearDownClass(cls):
        cls.ocr_srv.shutdown()
        cls.ocr_srv.server_close()

    def start(self):
        FakeOcr.mode = "ok"
        d = Path(tempfile.mkdtemp(prefix="relay_", dir=TMP))
        (d / "accounts.yaml").write_text(
            'accounts:\n  - username: "kari-op"\n    role: operator\n'
            f'    password_hash: "{H.OP_HASH}"\n    enabled: true\n', encoding="utf-8")
        store = acc.AccountStore(accounts_path=str(d / "accounts.yaml"), permissions_path=str(d / "perm.json"))
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, H.FakeTeleop(), auth_username="TL", auth_password="tl-pass",
            enforce_single_controller=False, account_store=store,
            ocr_admin=OcrAdminProxy(f"http://127.0.0.1:{self.ocr_port}"), survey_admin=FakeSurvey())
        self.srv, port = H.serve(handler)
        self.base = f"http://127.0.0.1:{port}"
        self.addCleanup(lambda: (self.srv.shutdown(), self.srv.server_close()))

    def cookie(self, user, pw):
        code, d, ck = H.http("POST", self.base + "/api/login", {"username": user, "password": pw})
        self.assertEqual(code, 200)
        return ck.split(";")[0]

    def test_images_require_login_and_work_for_every_logged_in_account(self):
        self.start()
        # 未ログイン → 401(画像は出ない)
        code, _, _ = raw_get(self.base + "/api/ocr/preview.jpg", "")
        self.assertEqual(code, 401)
        for user, pw in (("TL", "tl-pass"), ("kari-op", H.OP_PASSWORD), ("guest", "")):
            ck = self.cookie(user, pw)
            for path in ("/api/ocr/preview.jpg", "/api/ocr/processed.jpg"):
                code, body, ctype = raw_get(self.base + path + "?t=123", ck)  # ?t= はキャッシュ回避
                self.assertEqual(code, 200, (user, path))
                self.assertEqual(body, JPEG)
                self.assertIn("image/jpeg", ctype)

    def test_image_unavailable_returns_json_error_not_an_image(self):
        self.start()
        ck = self.cookie("TL", "tl-pass")
        FakeOcr.mode = "none"
        code, body, ctype = raw_get(self.base + "/api/ocr/preview.jpg", ck)
        self.assertEqual(code, 404)
        self.assertIn("application/json", ctype)

    def test_survey_csv_is_not_for_guest(self):
        self.start()
        code, body, _ = raw_get(self.base + "/api/survey/responses.csv", self.cookie("guest", ""))
        self.assertEqual(code, 403)
        for user, pw in (("TL", "tl-pass"), ("kari-op", H.OP_PASSWORD)):
            code, body, ctype = raw_get(self.base + "/api/survey/responses.csv", self.cookie(user, pw))
            self.assertEqual(code, 200, user)
            self.assertEqual(body, b"a,b\n1,2\n")


class StaticTest(unittest.TestCase):
    def test_ocr_reader_listens_on_loopback_by_default(self):
        root = TEST_DIR.parents[3]
        t = (root / "scripts" / "tablet_ocr_reader" / "tablet_ocr_reader.py").read_text(encoding="utf-8")
        self.assertIn('"preview_host": "127.0.0.1"', t)
        self.assertIn('config.get("preview_host", "127.0.0.1")', t)
        self.assertNotIn('"preview_host": "0.0.0.0"', t)
        self.assertNotIn('config.get("preview_host", "0.0.0.0")', t)
        ex = (root / "scripts" / "tablet_ocr_reader" / "tablet_ocr_config.example.json").read_text(encoding="utf-8")
        self.assertIn('"preview_host": "127.0.0.1"', ex)


if __name__ == "__main__":
    unittest.main(verbosity=1)
