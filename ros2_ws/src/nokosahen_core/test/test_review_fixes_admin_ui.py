"""全体レビュー11Aの小さな修正のテスト(2026-10-22)。ROSは使わない。"""
import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core import http_auth  # noqa: E402


class _Srv:
    def __init__(self, port):
        self.server_address = ("0.0.0.0", port)


class _H:
    def __init__(self, port=None, cookie=None):
        if port is not None:
            self.server = _Srv(port)
        self.headers = {"Cookie": cookie} if cookie else {}


class CookieNameTests(unittest.TestCase):
    def test_name_differs_per_port(self):
        self.assertNotEqual(http_auth.session_cookie_name(_H(8090)), http_auth.session_cookie_name(_H(8095)))

    def test_fallback_without_server(self):
        self.assertEqual(http_auth.session_cookie_name(_H()), http_auth.SESSION_COOKIE_NAME)
        self.assertEqual(http_auth.session_cookie_name(None), http_auth.SESSION_COOKIE_NAME)

    def test_cookie_for_other_port_is_ignored(self):
        h8090 = _H(8090)
        hdr = http_auth.session_cookie_header("tok", h8090).split(";")[0]
        self.assertEqual(http_auth.get_session_token(_H(8090, hdr)), "tok")
        self.assertIsNone(http_auth.get_session_token(_H(8095, hdr)))

    def test_clear_header_uses_same_name(self):
        h = _H(8095)
        self.assertTrue(http_auth.clear_session_cookie_header(h).startswith(http_auth.session_cookie_name(h) + "="))


class CredentialTests(unittest.TestCase):
    def test_both_compared(self):
        with mock.patch.object(http_auth.hmac, "compare_digest", return_value=False) as m:
            self.assertFalse(http_auth._credentials_match("a", "b", "x", "y"))
            self.assertEqual(m.call_count, 2)

    def test_match(self):
        self.assertTrue(http_auth._credentials_match("あ", "い", "あ", "い"))
        self.assertFalse(http_auth._credentials_match("あ", "x", "あ", "い"))


if __name__ == "__main__":
    unittest.main(verbosity=1)
