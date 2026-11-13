"""パスワード等の秘密情報をgit管理外から読み込む共通ヘルパー。

nokosahen.yaml(gitに載る)には実パスワードを書かず、空文字かプレースホルダー
("CHANGE_ME")だけを置く。実値は次の優先順位で解決する。

1. ROSパラメータの値(空でもプレースホルダーでもない場合。ros2 run --ros-args -p での
   一時上書き用。yamlには実値を書かないこと)
2. 環境変数 `NOKOSAHEN_<セクション名>_<キー名>`(大文字。例: NOKOSAHEN_FULL_DASHBOARD_AUTH_PASSWORD)
3. 秘密ファイル(git管理外)の `<セクション名>: <キー名>: 値`。探す場所は順に
   - 環境変数 `NOKOSAHEN_SECRETS_FILE` が指すファイル
   - `~/.nokosahen_secrets.yaml`
   - リポジトリの `ros2_ws/src/nokosahen_bringup/config/secrets.local.yaml`
   - インストール済み nokosahen_bringup パッケージの `config/secrets.local.yaml`
   雛形は `config/secrets.example.yaml`(これをコピーして secrets.local.yaml を作る)。

どれでも解決できなかった場合は空文字を返す。呼び出し側(resolve_password)は
その場合に「認証が黙って無効になる/空パスワードで入れてしまう」ことが無いよう、
起動ごとにランダムなパスワードへ切り替えて警告ログを出す。
"""
from __future__ import annotations

import os
import secrets as _pysecrets
from pathlib import Path
from typing import Any, Dict, Optional

PLACEHOLDER_PREFIX = "CHANGE_ME"

_cache: Dict[str, Dict[str, Any]] = {}
# 同じ(セクション,キー)でPC版/スマホ版など複数ハンドラが呼んでも警告と生成値を1回に揃える
_generated: Dict[tuple, str] = {}


def is_unset(value: Optional[str]) -> bool:
    """空文字・プレースホルダーなら「未設定」とみなす。"""
    v = (value or "").strip()
    return v == "" or v.startswith(PLACEHOLDER_PREFIX)


def _candidate_files() -> list:
    paths = []
    env_path = os.environ.get("NOKOSAHEN_SECRETS_FILE", "").strip()
    if env_path:
        paths.append(Path(env_path).expanduser())
    paths.append(Path("~/.nokosahen_secrets.yaml").expanduser())
    # ソース(symlink-install/開発時): このファイルから上に辿ってリポジトリ内のconfigを探す
    here = Path(__file__).resolve()
    for parent in here.parents:
        cand = parent / "nokosahen_bringup" / "config" / "secrets.local.yaml"
        if cand.parent.is_dir():
            paths.append(cand)
            break
    try:
        from ament_index_python.packages import get_package_share_directory

        paths.append(
            Path(get_package_share_directory("nokosahen_bringup")) / "config" / "secrets.local.yaml"
        )
    except Exception:  # noqa: BLE001 - ROS環境外(単体テスト等)では無視
        pass
    return paths


def _load_file() -> Dict[str, Any]:
    if "data" in _cache:
        return _cache["data"]
    data: Dict[str, Any] = {}
    for path in _candidate_files():
        try:
            if not path.is_file():
                continue
            import yaml

            loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            if isinstance(loaded, dict):
                data = loaded
                _cache["path"] = {"path": str(path)}
                break
        except Exception:  # noqa: BLE001
            continue
    _cache["data"] = data
    return data


def resolve_secret(section: str, key: str, param_value: str = "") -> str:
    """param_value → 環境変数 → 秘密ファイルの順で秘密の値を解決する。未解決なら空文字。"""
    if not is_unset(param_value):
        return str(param_value)
    env_name = f"NOKOSAHEN_{section}_{key}".upper().replace("-", "_")
    env_val = os.environ.get(env_name, "")
    if not is_unset(env_val):
        return env_val
    section_data = _load_file().get(section)
    if isinstance(section_data, dict):
        val = section_data.get(key)
        if val is not None and not is_unset(str(val)):
            return str(val)
    return ""


def resolve_password(logger: Any, section: str, key: str, param_value: str, username: str) -> str:
    """`username`が設定されているアカウントのパスワードを解決する(fail safe版)。

    解決できない場合は認証を無効化も空パスワード許可もせず、この起動限りの
    ランダムなパスワードを使って警告を出す(=そのアカウントではログインできない)。
    usernameが空の場合は「認証無効」の明示的な開発用設定とみなし、警告だけ出す。
    """
    value = resolve_secret(section, key, param_value)
    if value:
        return value
    if (section, key) in _generated:
        return _generated[(section, key)]
    if username:
        logger.warning(
            f"[{section}] {key} が未設定(空/CHANGE_ME)です。config/secrets.local.yaml か "
            f"環境変数 NOKOSAHEN_{section}_{key}".upper().replace("-", "_")
            + " で設定してください。設定されるまで、アカウント "
            f"'{username}' にはこの起動限りのランダムなパスワードが使われ、ログインできません。"
        )
        _generated[(section, key)] = _pysecrets.token_urlsafe(18)
        return _generated[(section, key)]
    if key != "auth_password":
        return ""  # 制限付きアカウント等: usernameが空なら単に無効(警告不要)
    logger.warning(
        f"[{section}] auth_username と {key} が両方とも空のため認証が無効です"
        "(開発用の明示的な設定でない場合は secrets.local.yaml を確認してください)。"
    )
    _generated[(section, key)] = ""
    return ""
