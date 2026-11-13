"""ダッシュボード/Web UIの簡易HTTP Basic認証。

同じLAN(決済カメラ用ルーター等)に他の端末がいても、意図しない第三者が
ダッシュボードや残数入力画面を開いたり操作したりできないようにするための
最低限の認証。TLS無しの平文HTTP上でのBasic認証なので、学内LAN程度の
脅威モデル向けの簡易対策であり、インターネットに公開する用途は想定しない。
"""
from __future__ import annotations

import base64
import binascii
import hmac
import secrets
import threading
import time
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler
from typing import Dict, Optional, Tuple


def _decode_basic_auth(handler: BaseHTTPRequestHandler) -> Optional[tuple]:
    """Authorizationヘッダを(送信されたユーザー名, パスワード)にデコードする。

    ヘッダが無い/形式不正ならNoneを返す。
    """
    header = handler.headers.get("Authorization", "")
    if not header.startswith("Basic "):
        return None
    try:
        decoded = base64.b64decode(header[len("Basic "):], validate=True).decode("utf-8")
    except (binascii.Error, UnicodeDecodeError, ValueError):
        return None
    sent_user, sep, sent_pass = decoded.partition(":")
    if not sep:
        return None
    return sent_user, sent_pass


def _credentials_match(sent_user: str, sent_pass: str, username: str, password: str) -> bool:
    # 単純な==は文字列長・内容によって比較時間が変わりタイミング攻撃の
    # 手がかりになりうるため、hmac.compare_digestで一定時間比較する。
    # hmac.compare_digestはstr同士だとASCII以外の文字(ひらがな等の
    # パスワードを想定)でTypeErrorになるため、必ずUTF-8バイト列にしてから渡す。
    # 【2026-10-21】`and`の短絡評価だと、ユーザー名が違うときはパスワードを比べずに早く返り、応答時間から「ユーザー名の正誤」が
    # 分かってしまう。両方を必ず比べてから、結果を合わせる。
    user_ok = hmac.compare_digest(sent_user.encode("utf-8"), username.encode("utf-8"))
    pass_ok = hmac.compare_digest(sent_pass.encode("utf-8"), password.encode("utf-8"))
    return user_ok and pass_ok


def check_basic_auth(handler: BaseHTTPRequestHandler, username: str, password: str) -> bool:
    """Authorizationヘッダを検証する。

    username/passwordが両方とも空なら「認証無効」として常にTrueを返す
    (auth_username/auth_passwordパラメータを空にすれば、これまで通り
    認証無しで使い続けられる)。
    """
    if not username and not password:
        return True
    decoded = _decode_basic_auth(handler)
    if decoded is None:
        return False
    sent_user, sent_pass = decoded
    return _credentials_match(sent_user, sent_pass, username, password)


def identify_account(
    handler: BaseHTTPRequestHandler, accounts: Dict[str, str]
) -> Optional[str]:
    """複数アカウント(例: 操作用アカウント+パスワード無しのguest)に対応した
    Basic認証チェック。一致したアカウント名を返す(一致無しならNone)。

    accountsは{ユーザー名: パスワード}。パスワードが空文字のアカウントは
    そのユーザー名を送りさえすればパスワード無しで認証成功する
    (誰でも入れる「閲覧専用guest」用途)。
    """
    decoded = _decode_basic_auth(handler)
    if decoded is None:
        return None
    sent_user, sent_pass = decoded
    for account_name, account_password in accounts.items():
        if _credentials_match(sent_user, sent_pass, account_name, account_password):
            return account_name
    return None


def send_auth_required(handler: BaseHTTPRequestHandler, realm: str) -> None:
    """401を返し、ブラウザ標準のBasic認証ダイアログを出させる。"""
    body = "認証が必要です".encode("utf-8")
    handler.send_response(401)
    handler.send_header("WWW-Authenticate", f'Basic realm="{realm}"')
    handler.send_header("Content-Type", "text/plain; charset=utf-8")
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


def check_login_credentials(
    username: str, password: str, accounts: Dict[str, str]
) -> Optional[str]:
    """ログインフォーム(専用ページ)から送られた生のユーザー名/パスワードを
    accountsと照合する(identify_accountのAuthorizationヘッダ版に対応する、
    フォームPOST版)。一致したアカウント名を返す(一致無しならNone)。
    """
    for account_name, account_password in accounts.items():
        if _credentials_match(username, password, account_name, account_password):
            return account_name
    return None


# 【2026-07-21追加】ダッシュボード/スマホ操作ページ専用のログイン。
#
# ブラウザ標準のBasic認証ダイアログには「ログアウト」の概念が無く、realmを
# 切り替えて資格情報を無効化する方法もブラウザによって挙動が不安定
# (一部のブラウザはfetch/XHRの401でも古い資格情報を先読みで再送してしまい、
# ログイン画面に戻らずそのまま操作画面に戻ってきてしまうことがある)。
# そのため、独自のログインページ(HTML)+セッションCookieに切り替える。
# セッションはこのプロセスのメモリ上にのみ保持する(プロセス再起動で全員
# ログアウトになるが、ダッシュボード用途では許容範囲)。
SESSION_COOKIE_NAME = "nokosahen_session"


class SessionStore:
    """ログイン成功時に発行するセッショントークン→アカウント名の対応表。

    【2026-09-30追加】無制限に溜まり続けないよう、最終利用からidle_timeout_sec
    (既定24時間)を過ぎたセッションは失効させ、保持数がmax_sessionsを超えたら
    最終利用が最も古いものから破棄する。
    """

    def __init__(self, idle_timeout_sec: float = 24 * 3600.0, max_sessions: int = 256, max_guest_sessions: int = 16) -> None:
        # token -> (account, 最終利用のmonotonic時刻)
        self._sessions: Dict[str, Tuple[str, float]] = {}
        self._lock = threading.Lock()
        self._idle_timeout_sec = idle_timeout_sec
        self._max_sessions = max(1, int(max_sessions))
        # 【2026-10-21】パスワードなしのguestは、連続ログインで他のアカウント(TL・operator)のセッションを追い出せないよう、
        # guest同士でだけ上限を設ける(溢れたらguestの最も古いものを捨てる)。全体の上限でも、まずguestから捨てる。
        self._max_guest_sessions = max(1, int(max_guest_sessions))

    def _purge_expired_locked(self, now: float) -> None:
        expired = [
            t for t, (_, last) in self._sessions.items()
            if now - last > self._idle_timeout_sec
        ]
        for t in expired:
            del self._sessions[t]

    def create(self, account: str) -> str:
        token = secrets.token_urlsafe(24)
        now = time.monotonic()
        with self._lock:
            self._purge_expired_locked(now)
            if account == "guest":
                guests = [t for t, (a, _) in self._sessions.items() if a == "guest"]
                while len(guests) >= self._max_guest_sessions:
                    oldest = min(guests, key=lambda t: self._sessions[t][1])
                    del self._sessions[oldest]
                    guests.remove(oldest)
            while len(self._sessions) >= self._max_sessions:
                guests = [t for t, (a, _) in self._sessions.items() if a == "guest"]
                pool = guests or list(self._sessions)  # まずguestから捨てる
                oldest = min(pool, key=lambda t: self._sessions[t][1])
                del self._sessions[oldest]
            self._sessions[token] = (account, now)
        return token

    def resolve(self, token: Optional[str]) -> Optional[str]:
        if not token:
            return None
        now = time.monotonic()
        with self._lock:
            entry = self._sessions.get(token)
            if entry is None:
                return None
            account, last = entry
            if now - last > self._idle_timeout_sec:
                del self._sessions[token]
                return None
            self._sessions[token] = (account, now)
            return account

    def revoke(self, token: Optional[str]) -> None:
        if not token:
            return
        with self._lock:
            self._sessions.pop(token, None)


def session_cookie_name(handler: Optional[BaseHTTPRequestHandler] = None) -> str:
    """セッションCookieの名前。【2026-10-21】PC版(8090)とスマホ版(8095)は別々のセッション表を持つのに、Cookie名が同じで、Cookieはポートで
    分かれないため、同じブラウザで片方にログインするともう片方が401になっていた(互いにログアウトし合う)。待受ポートを名前に含める。
    ポートが分からない(テスト用の偽のハンドラなど)ときは、従来の名前。"""
    try:
        port = int(handler.server.server_address[1])  # type: ignore[union-attr]
        return f"{SESSION_COOKIE_NAME}_{port}"
    except Exception:  # noqa: BLE001
        return SESSION_COOKIE_NAME


def get_session_token(handler: BaseHTTPRequestHandler) -> Optional[str]:
    header = handler.headers.get("Cookie")
    if not header:
        return None
    cookie: SimpleCookie = SimpleCookie()
    try:
        cookie.load(header)
    except Exception:  # noqa: BLE001
        return None
    morsel = cookie.get(session_cookie_name(handler))
    return morsel.value if morsel else None


def session_cookie_header(token: str, handler: Optional[BaseHTTPRequestHandler] = None) -> str:
    # Max-Age指定無し=ブラウザを閉じるまでのセッションCookie(ログアウトまで
    # 保持したいが、これまでのBasic認証キャッシュと同程度の寿命に留める)。
    return f"{session_cookie_name(handler)}={token}; Path=/; HttpOnly; SameSite=Lax"


def clear_session_cookie_header(handler: Optional[BaseHTTPRequestHandler] = None) -> str:
    return f"{session_cookie_name(handler)}=; Path=/; HttpOnly; SameSite=Lax; Max-Age=0"
