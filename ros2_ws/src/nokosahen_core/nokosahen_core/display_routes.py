"""iPad表示 `/display` の HTTP 入口(ダッシュボードのサーバ 8090 に同居。専用のログインと専用のセッション)。docs/ipad_display.md。

ダッシュボードの既定は「TLなどのアカウントで認証が要る」だが、iPadは固定URLを開きっぱなしにして画面だけを出す端末なので、
このモジュールの**決まったパスだけ**を、ダッシュボードの認証より前に処理する(dashboard_common の do_GET/do_POST の先頭から呼ぶ)。
ただし**最初に1回だけ**、厨房サイト(web_ui、ポート8080)と同じアカウント名・パスワードでログインする。ログインすると専用のセッションCookieを受け取り、
以降は再ログイン不要(有効期限は「このプロセスの再起動まで」。メモリ上だけ。3分などの自動ログアウトは無い)。ROSからの切替(/nokosahen/display_mode など)は、
ログインとは無関係にiPadの内容を変える。**TLの管理画面のログイン(TL/operator/guest)とは別系統**(互いのセッションでは入れない)。

認証なしで許す範囲(最小限)
  GET  /display                       ログイン画面のHTML(状態は読まない。ログインフォームだけ。ログイン済みなら本体の画面)
  POST /display/api/login             ログイン(失敗が続くと端末IPごとにロック)
  GET  /display/manifest.webmanifest, /display/icon.png, /display/apple-touch-icon.png   ホーム画面追加用
  GET  /display/api/state             **ループバック(127.0.0.1/::1)からだけ**。survey_monitor(同じ1台目の中で動き、Cookieを持たない)が読む。
                                      外部IPからはログイン済みのCookieが無い限り401。X-Forwarded-For は信用しない(リバースプロキシ越しではない前提)
ログイン済みセッションで許すもの
  GET  /display/face                  顔(ロボットの顔のアニメーションだけのHTML。Safariで単独で開いて確認する用。未ログインはログイン画面を401で返す)
  GET  /display/api/state             モード・回答中か・お客さん向けの文言と残数
  POST /display/api/survey/session    回答中の通知(start/beat/end。アンケート表示中のときだけ有効)
  POST /display/api/survey/answer     アンケート回答の追加(設問に合う内容だけ。保存ファイルの上限あり)
  POST /display/api/inventory         残数の入力
  POST /display/api/menu/edit         メニューの追加・名前の変更・削除・初期食数
  モード変更・模擬POSの数字・設定は、ここには無い(従来どおり /api/survey/mode など、TLの管理画面のログインと権限が要る)
  同時1端末制限(SingleControllerGate)はこのセッションには適用しない(複数のiPadを同時に使える)。

防御
  - Origin / Sec-Fetch-Site: ブラウザから別のサイトに載せたページが送ったPOSTは拒否(同じオリジンだけ)。
  - 本文の上限(バイト)と型の厳密な検証。端末(IP)ごとの回数制限。ログインの失敗はロック(LoginLimiter)。
  - Cookie は HttpOnly・SameSite=Lax・Path=/display(httpの運用なので Secure は付けない)。パスワードはログ・応答に出さない。
  - 応答に CSP・nosniff・no-store・frame拒否を付ける。
"""
from __future__ import annotations

import base64
import hmac
import json
import secrets
import threading
import time
from collections import deque
from http import HTTPStatus
from http.cookies import SimpleCookie
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple
from urllib.parse import parse_qs, urlparse

from .accounts import LoginLimiter
from .display_hub import DisplayHub
from .face_html import FACE_HTML

DISPLAY_PREFIX = "/display"
DISPLAY_COOKIE = "nokosahen_display"
COOKIE_MAX_AGE_S = 365 * 24 * 3600     # ブラウザ側は長く持たせる(閉じても残る)。サーバ側は再起動で全て無効になる
MAX_SESSION_BODY = 512
MAX_ANSWER_BODY = 8 * 1024
MAX_INVENTORY_BODY = 512
MAX_LOGIN_BODY = 1024
MAX_MENU_BODY = 1024
MAX_CRED_LEN = 256
MAX_DISPLAY_SESSIONS = 128

# 端末(IP)ごとの1分あたりの上限
LIMITS = {"state": 600, "session": 90, "answer": 30, "inventory": 60, "menu": 60, "login": 20, "other": 120}

LOOPBACK_ADDRS = frozenset({"127.0.0.1", "::1", "::ffff:127.0.0.1"})

_CSP = ("default-src 'none'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; img-src 'self' data:; "
        "connect-src 'self'; manifest-src 'self'; frame-src about:; base-uri 'none'; form-action 'none'; frame-ancestors 'none'")


def is_display_path(path: str) -> bool:
    return path == DISPLAY_PREFIX or path.startswith(DISPLAY_PREFIX + "/")


def is_loopback(addr: Any) -> bool:
    """接続元(TCPの相手)がこのPC自身か。X-Forwarded-For などのヘッダは見ない(偽装できるため)。"""
    return str(addr) in LOOPBACK_ADDRS


class RateLimiter:
    """単純な窓方式の回数制限(キーごとに直近 window_s 秒の件数)。"""

    def __init__(self, clock: Callable[[], float] = time.monotonic, window_s: float = 60.0) -> None:
        self._clock = clock
        self._window = float(window_s)
        self._hits: Dict[Tuple[str, str], Deque[float]] = {}
        self._lock = threading.Lock()

    def allow(self, kind: str, client: str, limit: Optional[int] = None) -> bool:
        lim = LIMITS.get(kind, LIMITS["other"]) if limit is None else limit
        now = self._clock()
        with self._lock:
            q = self._hits.setdefault((kind, client), deque())
            while q and now - q[0] > self._window:
                q.popleft()
            if len(q) >= lim:
                return False
            q.append(now)
            if len(self._hits) > 512:   # 端末が多すぎる(攻撃)ときのメモリ上限。古いキーから捨てる
                for k in [k for k, v in self._hits.items() if not v or now - v[-1] > self._window][:256]:
                    self._hits.pop(k, None)
            return True


class DisplayAuth:
    """iPad表示専用のログインとセッション(TLの管理画面の SessionStore / SingleControllerGate とは別物)。

    - アカウント名・パスワードは厨房サイト(web_ui)と同じものを、呼び出し元が同じ仕組みで解決して渡す(ここには保存しない・出力しない)。
    - セッションはメモリ上だけ。**期限は無い**(プロセスの再起動まで。作り直すと全て無効)。数が多すぎるときだけ古いものから捨てる。
    - ログイン失敗は端末IPごとに LoginLimiter で数える(連続で失敗するとしばらくロック)。
    """

    def __init__(self, username: str, password: str, limiter: Optional[LoginLimiter] = None,
                 max_sessions: int = MAX_DISPLAY_SESSIONS) -> None:
        self._user = (username or "").encode("utf-8")
        self._pass = (password or "").encode("utf-8")
        self.enabled = bool(self._user) and bool(self._pass)    # どちらかが空なら誰もログインできない(空パスワードで入れてしまうのを防ぐ)
        self.limiter = limiter or LoginLimiter()
        self._tokens: Dict[str, float] = {}
        self._max = max(1, int(max_sessions))
        self._lock = threading.Lock()

    def check(self, username: Any, password: Any) -> bool:
        """資格情報の照合(一定時間比較。ユーザー名とパスワードは両方比べてから結果を合わせる)。"""
        if not self.enabled or not isinstance(username, str) or not isinstance(password, str):
            return False
        u_ok = hmac.compare_digest(username.encode("utf-8"), self._user)
        p_ok = hmac.compare_digest(password.encode("utf-8"), self._pass)
        return u_ok and p_ok

    def create(self) -> str:
        token = secrets.token_urlsafe(32)
        with self._lock:
            while len(self._tokens) >= self._max:
                del self._tokens[min(self._tokens, key=self._tokens.get)]   # type: ignore[arg-type]
            self._tokens[token] = time.monotonic()
        return token

    def valid(self, token: Optional[str]) -> bool:
        if not token:
            return False
        with self._lock:
            return token in self._tokens

    def revoke(self, token: Optional[str]) -> None:
        with self._lock:
            self._tokens.pop(token or "", None)

    def count(self) -> int:
        with self._lock:
            return len(self._tokens)


def cookie_token(handler: Any) -> Optional[str]:
    header = handler.headers.get("Cookie")
    if not header:
        return None
    c: SimpleCookie = SimpleCookie()
    try:
        c.load(header)
    except Exception:  # noqa: BLE001
        return None
    m = c.get(DISPLAY_COOKIE)
    return m.value if m else None


def set_cookie_header(token: str) -> str:
    # Secure は付けない(http運用。Secure付きだとhttpでは保存されない)。Path=/display で、管理画面など他のパスには送られない
    return f"{DISPLAY_COOKIE}={token}; Path=/display; HttpOnly; SameSite=Lax; Max-Age={COOKIE_MAX_AGE_S}"


def origin_ok(headers: Any) -> bool:
    """ブラウザから送られたPOSTが、同じオリジン(このサーバ)からのものか。
    Origin があれば Host と同じであること。Sec-Fetch-Site があれば same-origin / none であること。
    どちらも無い(curl・サーバ同士など、ブラウザ以外)は通す(CSRFはブラウザの話。ログインは別に確認する)。"""
    get = headers.get
    sfs = get("Sec-Fetch-Site")
    if sfs is not None and sfs.strip().lower() not in ("same-origin", "none"):
        return False
    origin = get("Origin")
    if origin is not None and origin.strip() != "null":
        host = get("Host") or ""
        try:
            o = urlparse(origin.strip())
            if o.scheme.lower() not in ("http", "https") or not host or not o.hostname:
                return False
            default_port = 443 if o.scheme.lower() == "https" else 80
            o_port = o.port or default_port
            h = urlparse("//" + host.strip())   # Host は "名前" か "名前:ポート"(IPv6 は [::1]:8090)
            if not h.hostname:
                return False
            h_port = h.port or default_port      # Host にポートが無い=その方式の既定ポート
            # ホスト名は大文字小文字を区別しない。ポートは既定(80/443)の省略を同じとみなす
            return o.hostname.lower() == h.hostname.lower() and o_port == h_port
        except ValueError:                       # 壊れたポートなど
            return False
    elif origin is not None:                     # "null"(サンドボックス化された文書・リダイレクト越し等)は拒否
        return False
    return True


def _icon_bytes() -> Optional[bytes]:
    try:
        from .assets import LOGO_DATA_URI
        head, _, b64 = LOGO_DATA_URI.partition(",")
        if "image/png" in head:
            return base64.b64decode(b64)
    except Exception:  # noqa: BLE001
        pass
    return None


MANIFEST = json.dumps({
    "name": "のこさへん 頭部ディスプレイ", "short_name": "のこさへん", "start_url": "/display", "scope": "/display",
    "display": "fullscreen", "orientation": "landscape", "background_color": "#0f2e24", "theme_color": "#0f2e24",
    "icons": [{"src": "/display/icon.png", "sizes": "200x200", "type": "image/png"}],
}, ensure_ascii=False).encode("utf-8")


class DisplayContext:
    """ダッシュボードのハンドラから渡す部品。"""

    def __init__(
        self,
        hub: DisplayHub,
        page_getter: Callable[[], bytes],
        login_page_getter: Callable[[], bytes],
        auth: DisplayAuth,
        audit: Optional[Callable[[str, str], None]] = None,
        limiter: Optional[RateLimiter] = None,
        build: str = "",
    ) -> None:
        self.build = build            # 画面のプログラムの版。変わると開きっぱなしのiPadが自動で読み込み直す
        self.hub = hub
        self.page_getter = page_getter              # ログイン済みに返す本体の画面
        self.login_page_getter = login_page_getter  # 未ログインに返すログイン画面(状態を含まない)
        self.auth = auth
        self.audit = audit or (lambda level, text: None)
        self.limiter = limiter or RateLimiter()
        self.icon = _icon_bytes()


def _send(handler: Any, status: int, body: bytes, ctype: str, cache: str = "no-store",
          extra: Optional[List[Tuple[str, str]]] = None) -> None:
    handler.send_response(status)
    handler.send_header("Content-Type", ctype)
    handler.send_header("Content-Length", str(len(body)))
    handler.send_header("Cache-Control", cache)
    handler.send_header("X-Content-Type-Options", "nosniff")
    handler.send_header("Referrer-Policy", "no-referrer")
    handler.send_header("X-Frame-Options", "DENY")
    handler.send_header("Content-Security-Policy", _CSP)
    for k, v in (extra or []):
        handler.send_header(k, v)
    handler.end_headers()
    handler.wfile.write(body)


def _json(handler: Any, status: int, payload: Dict[str, Any], extra: Optional[List[Tuple[str, str]]] = None) -> None:
    _send(handler, status, json.dumps(payload, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8", extra=extra)


def _client(handler: Any) -> str:
    ca = getattr(handler, "client_address", None)
    return str(ca[0]) if ca else "?"


def _read_json(handler: Any, limit: int) -> Tuple[Optional[Dict[str, Any]], Optional[Tuple[int, str]]]:
    """(本文dict, エラー(ステータス, 文))。Content-Length必須・上限・UTF-8・JSONオブジェクトだけ。"""
    try:
        n = int(handler.headers.get("Content-Length", ""))
    except ValueError:
        return None, (HTTPStatus.LENGTH_REQUIRED, "Content-Lengthが必要です")
    if n < 0:
        return None, (HTTPStatus.BAD_REQUEST, "Content-Lengthが不正です")
    if n > limit:
        return None, (HTTPStatus.REQUEST_ENTITY_TOO_LARGE, "本文が大きすぎます")
    try:
        data = json.loads(handler.rfile.read(n).decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        return None, (HTTPStatus.BAD_REQUEST, "JSONが不正です")
    if not isinstance(data, dict):
        return None, (HTTPStatus.BAD_REQUEST, "JSONオブジェクトにしてください")
    return data, None


def _too_many(handler: Any) -> None:
    _json(handler, HTTPStatus.TOO_MANY_REQUESTS, {"ok": False, "error": "too_many_requests"})


def _unauthorized(handler: Any) -> None:
    _json(handler, HTTPStatus.UNAUTHORIZED, {"ok": False, "error": "auth_required"})


def handle_get(handler: Any, path: str, query: str, ctx: DisplayContext) -> bool:
    """/display 配下のGET。扱ったら True。"""
    if not is_display_path(path):
        return False
    client = _client(handler)
    kind = "state" if path == "/display/api/state" else "other"
    if not ctx.limiter.allow(kind, client):
        _too_many(handler)
        return True
    logged_in = ctx.auth.valid(cookie_token(handler))
    if path in ("/display", "/display/"):
        # 状態は読まない。ログイン済みなら本体の画面、未ログインならログインフォームだけの画面
        _send(handler, HTTPStatus.OK, ctx.page_getter() if logged_in else ctx.login_page_getter(), "text/html; charset=utf-8")
        return True
    if path == "/display/face":
        # 顔(元のHTMLそのもの)。/display と同じログインが要る。未ログインは顔を返さず、ログイン画面(ログインすると読み込み直して顔になる)を401で返す
        if not logged_in:
            _send(handler, HTTPStatus.UNAUTHORIZED, ctx.login_page_getter(), "text/html; charset=utf-8")
            return True
        _send(handler, HTTPStatus.OK, FACE_HTML.encode("utf-8"), "text/html; charset=utf-8")
        return True
    if path == "/display/api/state":
        # ログイン済みのiPad、または同じPCの中の survey_monitor(ループバック。Cookieなし)だけ
        if not logged_in and not is_loopback(client):
            _unauthorized(handler)
            return True
        seen = None
        q = parse_qs(query)
        raw = (q.get("seen") or [None])[0]
        if raw is not None and raw.isdigit() and len(raw) <= 16:
            seen = int(raw)
        if logged_in:
            ctx.hub.note_client(client, seen)     # 接続中のiPadとして数える(survey_monitor は数えない)
        st = ctx.hub.public_state()
        if ctx.build:
            st["build"] = ctx.build
        _json(handler, HTTPStatus.OK, st)
        return True
    if path == "/display/manifest.webmanifest":
        _send(handler, HTTPStatus.OK, MANIFEST, "application/manifest+json", cache="max-age=3600")
        return True
    if path in ("/display/icon.png", "/display/apple-touch-icon.png") and ctx.icon:
        _send(handler, HTTPStatus.OK, ctx.icon, "image/png", cache="max-age=86400")
        return True
    _json(handler, HTTPStatus.NOT_FOUND, {"ok": False, "error": "not_found"})
    return True


def _handle_login(handler: Any, client: str, ctx: DisplayContext) -> None:
    if not ctx.limiter.allow("login", client):
        _too_many(handler)
        return
    data, err = _read_json(handler, MAX_LOGIN_BODY)
    if data is None:
        _json(handler, err[0], {"ok": False, "error": err[1]})
        return
    user, pw = data.get("username"), data.get("password")
    if set(data) != {"username", "password"} or not isinstance(user, str) or not isinstance(pw, str) \
            or len(user) > MAX_CRED_LEN or len(pw) > MAX_CRED_LEN:
        _json(handler, HTTPStatus.BAD_REQUEST, {"ok": False, "error": 'リクエストは {"username": 文字, "password": 文字} だけにしてください'})
        return
    keys = (f"ip:{client}",)
    wait = ctx.auth.limiter.retry_after(keys)
    if wait > 0:
        ctx.audit("warning", f"DISPLAY_LOGIN_LOCKED ip={client} retry_after={int(wait) + 1}s")
        _json(handler, HTTPStatus.TOO_MANY_REQUESTS, {"ok": False, "error": "too_many_attempts", "retry_after_sec": int(wait) + 1},
              extra=[("Retry-After", str(int(wait) + 1))])
        return
    if not ctx.auth.check(user, pw):
        ctx.auth.limiter.record_failure(keys)
        ctx.audit("warning", f"DISPLAY_LOGIN_FAILED ip={client}")      # アカウント名・パスワードは出さない
        _json(handler, HTTPStatus.UNAUTHORIZED, {"ok": False, "error": "invalid_credentials"})
        return
    ctx.auth.limiter.record_success(keys)
    token = ctx.auth.create()
    ctx.audit("info", f"DISPLAY_LOGIN_OK ip={client} sessions={ctx.auth.count()}")
    _json(handler, HTTPStatus.OK, {"ok": True}, extra=[("Set-Cookie", set_cookie_header(token))])


def handle_post(handler: Any, path: str, query: str, ctx: DisplayContext) -> bool:
    """/display 配下のPOST。扱ったら True。"""
    if not is_display_path(path):
        return False
    client = _client(handler)
    if not origin_ok(handler.headers):
        _json(handler, HTTPStatus.FORBIDDEN, {"ok": False, "error": "bad_origin"})
        return True
    if path == "/display/api/login":
        _handle_login(handler, client, ctx)
        return True
    known = ("/display/api/survey/session", "/display/api/survey/answer", "/display/api/inventory", "/display/api/menu/edit")
    if path not in known:
        _json(handler, HTTPStatus.NOT_FOUND, {"ok": False, "error": "not_found"})
        return True
    # ここから先は、ログイン済みのセッションだけ(ループバックでも免除しない)
    if not ctx.auth.valid(cookie_token(handler)):
        _unauthorized(handler)
        return True
    if path == "/display/api/survey/session":
        if not ctx.limiter.allow("session", client):
            _too_many(handler)
            return True
        data, err = _read_json(handler, MAX_SESSION_BODY)
        if data is None:
            _json(handler, err[0], {"ok": False, "error": err[1]})
            return True
        if not set(data) <= {"event", "condition"}:
            _json(handler, HTTPStatus.BAD_REQUEST, {"ok": False, "error": "不明な項目があります"})
            return True
        r = ctx.hub.session_event(data.get("event"), data.get("condition"))
        _json(handler, r.pop("code", 200), r)
        return True
    if path == "/display/api/survey/answer":
        if not ctx.limiter.allow("answer", client):
            _too_many(handler)
            return True
        data, err = _read_json(handler, MAX_ANSWER_BODY)
        if data is None:
            _json(handler, err[0], {"ok": False, "error": err[1]})
            return True
        replay = (parse_qs(query).get("replay") or [""])[0] == "1"
        code, body = ctx.hub.submit_response(data, replay=replay)
        _json(handler, code, body)
        return True
    if path == "/display/api/inventory":
        if not ctx.limiter.allow("inventory", client):
            _too_many(handler)
            return True
        data, err = _read_json(handler, MAX_INVENTORY_BODY)
        if data is None:
            _json(handler, err[0], {"ok": False, "error": err[1]})
            return True
        if set(data) != {"menu", "remaining"}:
            _json(handler, HTTPStatus.BAD_REQUEST, {"ok": False, "error": 'リクエストは {"menu": 名前, "remaining": 整数} だけにしてください'})
            return True
        code, body = ctx.hub.set_inventory(data["menu"], data["remaining"])
        ctx.audit("info" if code == 200 else "warning",
                  f"DISPLAY_INVENTORY ip={client} menu={str(data['menu'])[:40]!r} remaining={data['remaining']!r} -> {code}")
        _json(handler, code, body)
        return True
    # /display/api/menu/edit
    if not ctx.limiter.allow("menu", client):
        _too_many(handler)
        return True
    data, err = _read_json(handler, MAX_MENU_BODY)
    if data is None:
        _json(handler, err[0], {"ok": False, "error": err[1]})
        return True
    code, body = ctx.hub.edit_menu(data)
    ctx.audit("info" if code == 200 else "warning",
              f"DISPLAY_MENU_EDIT ip={client} op={str(data.get('op'))[:20]!r} name={str(data.get('name'))[:40]!r} -> {code}")
    _json(handler, code, body)
    return True
