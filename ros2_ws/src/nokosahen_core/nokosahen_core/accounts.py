"""弱いアカウント(operator)の「アカウント表」とパスワードハッシュ、ログイン試行の回数制限(純Python、ROS非依存)。

【2026-10-13追加】docs/accounts.md。

- 既存のアカウント(管理者=auth_username/ADMIN_ACCOUNT/TL、guest、スマホ版のrestricted)の認証方式・動作は一切変えない。
  ここで扱うのは **operatorロールの新アカウントだけ**。同じ `/api/login`(同じログイン画面)から入れる。
- **既定では無効**: アカウント名・パスワードは「後で決める」ので、プレースホルダー
  (`username: CHANGE_ME_OPERATOR` / `enabled: false` / `password_hash: ""`)だけを持つ。
  `enabled:false`・プレースホルダー名・ハッシュ空・ハッシュ不正のいずれかなら **ログインできない**(空パスワードも不可)。
- パスワードは **塩付きハッシュ(hashlib.scrypt、無い環境ではpbkdf2_hmac)** で保存。平文は保存しない。
  ハッシュ生成は `scripts/make_account_hash.py`(対話入力、標準出力にハッシュだけ)。
- アカウント定義の置き場(git管理外): 環境変数 `NOKOSAHEN_ACCOUNTS_FILE` → `~/.nokosahen_accounts.yaml` →
  `ros2_ws/src/nokosahen_bringup/config/accounts.local.yaml`(.gitignore済み `*.local.yaml`)。雛形は
  `config/accounts.example.yaml`。ファイルが無ければプレースホルダー(無効)だけ。
- operatorの許可リスト(`operator_permissions`)は、管理画面から保存した永続ファイル
  (`~/.nokosahen_operator_permissions.json`)があればそれ、無ければアカウント定義ファイルの値、無ければ空(=何も許可しない)。
  どちらも `roles.sanitize_operator_permissions` で検証し、admin_only/未知の名前は付与しない。
"""
from __future__ import annotations

import base64
import binascii
import datetime
import hashlib
import hmac
import json
import os
import secrets as _pysecrets
import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, FrozenSet, Iterable, List, Optional, Tuple

from . import roles

PLACEHOLDER_PREFIX = "CHANGE_ME"
DEFAULT_PLACEHOLDER_USERNAME = "CHANGE_ME_OPERATOR"
DEFAULT_PERMISSIONS_FILE = "~/.nokosahen_operator_permissions.json"
USERNAME_MAX = 64

# ---------------------------------------------------------------------------
# パスワードハッシュ
# ---------------------------------------------------------------------------
SCRYPT_N = 2 ** 14
SCRYPT_R = 8
SCRYPT_P = 1
PBKDF2_ITERATIONS = 600_000
_MAXMEM = 64 * 1024 * 1024


def _b64(raw: bytes) -> str:
    return base64.urlsafe_b64encode(raw).decode("ascii").rstrip("=")


def _unb64(text: str) -> bytes:
    return base64.urlsafe_b64decode(text + "=" * (-len(text) % 4))


def hash_password(password: str, *, scheme: Optional[str] = None, n: int = SCRYPT_N,
                  iterations: int = PBKDF2_ITERATIONS) -> str:
    """塩付きハッシュ文字列を作る。空のパスワードは拒否(ValueError)。

    形式: `scrypt$N$r$p$salt$hash`(または `pbkdf2_sha256$iterations$salt$hash`)。
    """
    if not isinstance(password, str) or password == "":
        raise ValueError("パスワードが空です")
    salt = _pysecrets.token_bytes(16)
    pw = password.encode("utf-8")
    if (scheme or ("scrypt" if hasattr(hashlib, "scrypt") else "pbkdf2_sha256")) == "scrypt":
        dk = hashlib.scrypt(pw, salt=salt, n=n, r=SCRYPT_R, p=SCRYPT_P, maxmem=_MAXMEM, dklen=32)
        return f"scrypt${n}${SCRYPT_R}${SCRYPT_P}${_b64(salt)}${_b64(dk)}"
    dk = hashlib.pbkdf2_hmac("sha256", pw, salt, iterations, dklen=32)
    return f"pbkdf2_sha256${iterations}${_b64(salt)}${_b64(dk)}"


def parse_hash(stored: Any) -> Optional[Tuple[str, tuple, bytes, bytes]]:
    """ハッシュ文字列を(方式, パラメータ, 塩, ハッシュ)に分解。不正ならNone。"""
    if not isinstance(stored, str) or not stored:
        return None
    parts = stored.split("$")
    try:
        if parts[0] == "scrypt" and len(parts) == 6:
            n, r, p = int(parts[1]), int(parts[2]), int(parts[3])
            if not (2 <= n <= 2 ** 20 and n & (n - 1) == 0 and 1 <= r <= 32 and 1 <= p <= 16):
                return None
            return "scrypt", (n, r, p), _unb64(parts[4]), _unb64(parts[5])
        if parts[0] == "pbkdf2_sha256" and len(parts) == 4:
            it = int(parts[1])
            if not (1000 <= it <= 10_000_000):
                return None
            return "pbkdf2_sha256", (it,), _unb64(parts[2]), _unb64(parts[3])
    except (ValueError, binascii.Error):
        return None
    return None


def verify_password(password: Any, stored: Any) -> bool:
    """パスワードとハッシュを照合する。空パスワード・不正/空のハッシュは常にFalse(一定時間比較)。"""
    if not isinstance(password, str) or password == "":
        return False
    parsed = parse_hash(stored)
    if parsed is None:
        return False
    scheme, params, salt, expected = parsed
    pw = password.encode("utf-8")
    try:
        if scheme == "scrypt":
            n, r, p = params
            actual = hashlib.scrypt(pw, salt=salt, n=n, r=r, p=p, maxmem=_MAXMEM, dklen=len(expected))
        else:
            actual = hashlib.pbkdf2_hmac("sha256", pw, salt, params[0], dklen=len(expected))
    except (ValueError, AttributeError, OSError):
        return False
    return hmac.compare_digest(actual, expected)


# ---------------------------------------------------------------------------
# ログイン試行の回数制限
# ---------------------------------------------------------------------------
class LoginLimiter:
    """失敗回数が`max_failures`回に達したら`lockout_sec`秒ロックする(キーごと。成功でリセット)。

    ブルートフォース対策。キーは「アカウント名」と「接続元IP」の両方を渡す(どちらかがロック中なら拒否)。
    時計は差し替え可能(テスト用)。メモリ上のみ(再起動で解除)。
    """

    def __init__(self, max_failures: int = 5, window_sec: float = 600.0, lockout_sec: float = 300.0,
                 clock: Callable[[], float] = time.monotonic, max_keys: int = 1024) -> None:
        self.max_failures = max(1, int(max_failures))
        self.window_sec = float(window_sec)
        self.lockout_sec = float(lockout_sec)
        self._clock = clock
        self._max_keys = max_keys
        self._lock = threading.Lock()
        self._fails: Dict[str, List[float]] = {}
        self._locked_until: Dict[str, float] = {}

    def _purge(self, now: float) -> None:
        for k in [k for k, t in self._locked_until.items() if t <= now]:
            del self._locked_until[k]
            self._fails.pop(k, None)
        if len(self._fails) > self._max_keys:
            for k in sorted(self._fails, key=lambda k: self._fails[k][-1] if self._fails[k] else 0)[: len(self._fails) // 2]:
                self._fails.pop(k, None)

    def retry_after(self, keys: Iterable[str]) -> float:
        """ロック中なら残り秒数、そうでなければ0。"""
        now = self._clock()
        with self._lock:
            self._purge(now)
            return max([self._locked_until.get(k, 0.0) - now for k in keys] + [0.0])

    def record_failure(self, keys: Iterable[str]) -> None:
        now = self._clock()
        with self._lock:
            self._purge(now)
            for k in keys:
                hist = [t for t in self._fails.get(k, []) if now - t <= self.window_sec]
                hist.append(now)
                self._fails[k] = hist
                if len(hist) >= self.max_failures:
                    self._locked_until[k] = now + self.lockout_sec

    def record_success(self, keys: Iterable[str]) -> None:
        with self._lock:
            for k in keys:
                self._fails.pop(k, None)
                self._locked_until.pop(k, None)


# ---------------------------------------------------------------------------
# アカウント表
# ---------------------------------------------------------------------------
def is_placeholder_username(name: str) -> bool:
    n = (name or "").strip()
    return n == "" or n.upper().startswith(PLACEHOLDER_PREFIX)


def default_account_definitions() -> List[Dict[str, Any]]:
    return [{"username": DEFAULT_PLACEHOLDER_USERNAME, "role": roles.ROLE_OPERATOR,
             "password_hash": "", "enabled": False}]


def _candidate_account_files() -> List[Path]:
    paths: List[Path] = []
    env_path = os.environ.get("NOKOSAHEN_ACCOUNTS_FILE", "").strip()
    if env_path:
        paths.append(Path(env_path).expanduser())
    paths.append(Path("~/.nokosahen_accounts.yaml").expanduser())
    here = Path(__file__).resolve()
    for parent in here.parents:
        cand = parent / "nokosahen_bringup" / "config" / "accounts.local.yaml"
        if cand.parent.is_dir():
            paths.append(cand)
            break
    try:
        from ament_index_python.packages import get_package_share_directory

        paths.append(Path(get_package_share_directory("nokosahen_bringup")) / "config" / "accounts.local.yaml")
    except Exception:  # noqa: BLE001 - ROS環境外(単体テスト等)では無視
        pass
    return paths


class AccountStore:
    """operatorアカウントの定義と許可リストの読み込み・保存(スレッドセーフ、ファイルの更新を自動検知)。

    `reserved_names`: 既存アカウント名(管理者・guest・restricted)。operatorに同じ名前は使えない(起動時に弾く)。
    `accounts_path`/`permissions_path`: テスト用に明示できる(Noneなら自動探索 / 既定の永続ファイル)。
    `accounts_path=""` は「ファイルを探さない」(プレースホルダーだけ)。
    """

    def __init__(
        self,
        reserved_names: Iterable[str] = ("guest",),
        accounts_path: Optional[str] = None,
        permissions_path: Optional[str] = None,
        clock: Callable[[], float] = time.time,
    ) -> None:
        self._lock = threading.Lock()
        self._clock = clock
        self._reserved = {str(n) for n in reserved_names if n}
        # 管理者・スマホ用の専用アカウントとして登録された名前(guestは含まない)。set_reservedで、複数のサーバー(PC版・スマホ版)が
        # 同じストアを共有しても消えないよう、追加するだけで置き換えない。role_of は、ここにある名前だけをadminにする。
        self._admin_names = {n for n in self._reserved if n != "guest"}
        self._restricted_names: set = set()  # 【2026-11-04】スマホ版の制限付きアカウント(restricted_account)。adminではなくrestrictedロール
        self._accounts_path = accounts_path
        self._candidates: Optional[List[Path]] = None
        env_perm = os.environ.get("NOKOSAHEN_OPERATOR_PERMISSIONS_FILE", "").strip()
        self.permissions_path = os.path.expanduser(permissions_path or env_perm or DEFAULT_PERMISSIONS_FILE)
        self._sig: Optional[Tuple] = None
        self._accounts: Dict[str, Dict[str, Any]] = {}
        self._file_permissions: List[str] = []
        self._perm_file_permissions: Optional[List[str]] = None
        self._granted: FrozenSet[str] = frozenset()
        self.problems: List[str] = []
        self.source_path = ""
        self._last_login: Dict[str, float] = {}
        self._dummy_hash = hash_password(_pysecrets.token_urlsafe(12))
        self._reload(force=True)

    def set_reserved(self, names: Iterable[str], restricted_names: Iterable[str] = ()) -> None:
        """【2026-11-04】`restricted_names`は予約名のうち制限付きアカウント(restricted_account)。adminではなくrestrictedロールになる
        (以前はadmin扱いで、緊急停止の解除や動作スイッチまで通っていた)。一度restrictedに登録された名前は、後の登録でadminに戻らない。"""
        with self._lock:
            self._reserved = {str(n) for n in names if n}
            self._restricted_names |= {str(n) for n in restricted_names if n and str(n) != "guest"}
            self._admin_names |= {n for n in self._reserved if n != "guest"}
            self._admin_names -= self._restricted_names
        self._reload(force=True)

    # --- 読み込み ---
    def _accounts_file(self) -> Optional[Path]:
        if self._accounts_path is not None:
            return Path(self._accounts_path).expanduser() if self._accounts_path else None
        if self._candidates is None:
            self._candidates = _candidate_account_files()  # 探索先は1回だけ決める(リクエストごとにament探索をしない)
        for p in self._candidates:
            try:
                if p.is_file():
                    return p
            except OSError:
                continue
        return None

    @staticmethod
    def _stat_sig(path: Optional[Path]) -> Optional[Tuple]:
        if path is None:
            return None
        try:
            st = os.stat(path)
            return (str(path), st.st_mtime_ns, st.st_size)
        except OSError:
            return None

    def _reload(self, force: bool = False) -> None:
        apath = self._accounts_file()
        sig = (self._stat_sig(apath), self._stat_sig(Path(self.permissions_path)))
        with self._lock:
            if not force and sig == self._sig:
                return
            self._sig = sig
            problems: List[str] = []
            defs: List[Any] = default_account_definitions()
            file_perms: List[str] = []
            self.source_path = ""
            if apath is not None and sig[0] is not None:
                try:
                    import yaml

                    doc = yaml.safe_load(apath.read_text(encoding="utf-8")) or {}
                    if not isinstance(doc, dict):
                        raise ValueError("トップレベルがマッピングではありません")
                    if "accounts" in doc:
                        if not isinstance(doc["accounts"], list):
                            raise ValueError("accounts はリストにしてください")
                        defs = doc["accounts"]
                    if isinstance(doc.get("operator_permissions"), list):
                        file_perms = [str(x) for x in doc["operator_permissions"]]
                    self.source_path = str(apath)
                except Exception as exc:  # noqa: BLE001 - 壊れていたら全アカウント無効(fail safe)
                    problems.append(f"アカウント定義ファイルを読めないため、operatorは全て無効です: {exc}")
                    defs = default_account_definitions()
            accounts: Dict[str, Dict[str, Any]] = {}
            for i, d in enumerate(defs):
                acc, why = self._normalize(d, accounts)
                if why:
                    problems.append(f"accounts[{i}]: {why}")
                if acc is not None:
                    accounts[acc["username"]] = acc
            self._accounts = accounts
            self._file_permissions = file_perms
            perm_file: Optional[List[str]] = None
            if sig[1] is not None:
                try:
                    with open(self.permissions_path, "r", encoding="utf-8") as f:
                        pdoc = json.loads(f.read())
                    if not isinstance(pdoc, dict) or not isinstance(pdoc.get("operator_permissions"), list):
                        raise ValueError("operator_permissions がリストではありません")
                    perm_file = [str(x) for x in pdoc["operator_permissions"]]
                except (OSError, ValueError) as exc:
                    problems.append(f"許可リストの永続ファイルを読めないため、何も許可しません: {exc}")
                    perm_file = []
            self._perm_file_permissions = perm_file
            chosen = perm_file if perm_file is not None else file_perms
            self._granted, perm_problems = roles.sanitize_operator_permissions(chosen)
            self.problems = problems + perm_problems

    def _normalize(self, d: Any, existing: Dict[str, Any]) -> Tuple[Optional[Dict[str, Any]], str]:
        if not isinstance(d, dict):
            return None, "定義がマッピングではありません"
        name = d.get("username")
        if not isinstance(name, str) or not name.strip():
            return None, "username が空です"
        name = name.strip()
        if len(name) > USERNAME_MAX or any(ord(c) < 32 for c in name):
            return None, "username が長すぎる/制御文字を含みます"
        role = str(d.get("role", roles.ROLE_OPERATOR))
        if role != roles.ROLE_OPERATOR:
            return None, f"{name}: ここで定義できるのは role: operator だけです(admin/viewerは従来のアカウント)"
        if name in existing:
            return None, f"{name}: username が重複しています"
        if name in self._reserved:
            return None, f"{name}: 既存アカウント名と同じです(使えません)"
        h = d.get("password_hash", "")
        h = h if isinstance(h, str) else ""
        enabled_flag = d.get("enabled") is True
        placeholder = is_placeholder_username(name)
        hash_ok = parse_hash(h) is not None
        return {
            "username": name, "role": role, "password_hash": h,
            "enabled_flag": enabled_flag, "placeholder": placeholder, "hash_ok": hash_ok,
            # ログインできるのは「有効フラグ + プレースホルダーでない名前 + 有効なハッシュ」が揃ったときだけ
            "can_login": bool(enabled_flag and not placeholder and hash_ok),
        }, ""

    # --- 公開API ---
    def refresh(self) -> None:
        self._reload()

    def is_operator_name(self, username: Any) -> bool:
        """この名前はoperatorアカウント(無効なものも含む)か。ログイン処理の振り分け用。"""
        self._reload()
        return isinstance(username, str) and username in self._accounts

    def role_of(self, account: str) -> str:
        """セッションのアカウント名からロールを決める(サーバー側で毎回決定。クライアント申告は使わない)。
        operatorとして有効に定義されていて、ログイン可能な名前だけがoperator。guestはviewer。
        adminは、管理者・スマホ用の専用アカウントとして登録された名前(set_reserved)だけ。
        【2026-10-21】以前は「それ以外は全部admin」だった。operatorを削除・無効化・定義ファイルが壊れた直後に、ログイン中の
        そのセッションがadminに昇格してしまう(安全側の「無効」と逆)ため、未登録・無効な名前は閲覧のみ(viewer)にする。
        """
        if account == "guest":
            return roles.ROLE_VIEWER
        if account == "":
            return roles.ROLE_ADMIN  # 認証無効(auth_usernameが空。Stage1等の開発用途)は従来どおり。ログインしたセッションの名前は空にならない
        self._reload()
        acc = self._accounts.get(account)
        if acc is not None:
            return roles.ROLE_OPERATOR if acc.get("can_login") else roles.ROLE_VIEWER
        if account in self._restricted_names:
            return roles.ROLE_RESTRICTED
        if account in self._admin_names:
            return roles.ROLE_ADMIN
        return roles.ROLE_VIEWER

    def operator_permissions(self) -> FrozenSet[str]:
        self._reload()
        return self._granted

    def verify_operator(self, username: Any, password: Any) -> bool:
        """operatorのログイン照合。無効/プレースホルダー/ハッシュ無し/空パスワードは必ずFalse。"""
        self._reload()
        acc = self._accounts.get(username) if isinstance(username, str) else None
        if acc is None or not acc["can_login"] or not isinstance(password, str) or password == "":
            verify_password("x", self._dummy_hash)  # 応答時間を揃える
            return False
        return verify_password(password, acc["password_hash"])

    def note_login(self, account: str) -> None:
        with self._lock:
            self._last_login[account] = self._clock()

    def save_permissions(self, names: Iterable[str], updated_by: str = "") -> Tuple[FrozenSet[str], List[str]]:
        """許可リストを検証して永続ファイルへ原子的に保存する。admin_only/未知を含むと PermissionConfigError。"""
        granted, _ = roles.sanitize_operator_permissions(list(names), strict=True)
        doc = {
            "version": 1,
            "operator_permissions": sorted(granted),
            "updated_at": datetime.datetime.now().isoformat(timespec="seconds"),
            "updated_by": str(updated_by)[:64],
        }
        directory = os.path.dirname(self.permissions_path) or "."
        tmp_path = None
        os.makedirs(directory, exist_ok=True)
        try:
            fd, tmp_path = tempfile.mkstemp(prefix=".nokosahen_operator_perm_", suffix=".tmp", dir=directory)
            with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as f:
                f.write(json.dumps(doc, ensure_ascii=False, indent=2) + "\n")
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp_path, self.permissions_path)
            tmp_path = None
        finally:
            if tmp_path:
                try:
                    os.remove(tmp_path)
                except OSError:
                    pass
        self._reload(force=True)
        return self._granted, self.problems

    def listing(self, admin_names: Iterable[str] = (), restricted_names: Iterable[str] = ()) -> List[Dict[str, Any]]:
        """管理画面用のアカウント一覧(パスワード・ハッシュは含めない)。"""
        self._reload()

        def last(name: str) -> Optional[str]:
            t = self._last_login.get(name)
            return datetime.datetime.fromtimestamp(t).isoformat(timespec="seconds") if t else None

        rows: List[Dict[str, Any]] = []
        for n in admin_names:
            if n:
                rows.append({"username": n, "role": roles.ROLE_ADMIN, "enabled": True, "builtin": True,
                             "last_login": last(n), "note": "既存の管理者アカウント"})
        for n in restricted_names:
            if n:
                rows.append({"username": n, "role": roles.ROLE_RESTRICTED, "enabled": True, "builtin": True,
                             "last_login": last(n), "note": "スマホ版の限定アカウント(従来どおり)"})
        rows.append({"username": "guest", "role": roles.ROLE_VIEWER, "enabled": True, "builtin": True,
                     "last_login": last("guest"), "note": "閲覧のみ"})
        for acc in self._accounts.values():
            if acc["can_login"]:
                note = ""
            elif acc["placeholder"]:
                note = "未設定(プレースホルダー)。名前とパスワードを決めるまでログインできません"
            elif not acc["hash_ok"]:
                note = "パスワードハッシュが未設定/不正のためログインできません"
            else:
                note = "enabled:false のためログインできません"
            rows.append({"username": acc["username"], "role": acc["role"], "enabled": acc["can_login"],
                         "builtin": False, "placeholder": acc["placeholder"], "last_login": last(acc["username"]),
                         "note": note})
        return rows
