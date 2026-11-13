#!/usr/bin/env python3
"""operatorアカウント用のパスワードハッシュを作る(docs/accounts.md)。

使い方:  python scripts/make_account_hash.py
  - パスワードは対話入力(画面に表示されない)。引数・環境変数・ログには出さない。
  - 標準出力にはハッシュ文字列だけを1行出す(accounts.local.yaml の password_hash: に貼る)。
  - 案内や確認メッセージは標準エラー出力へ出すので、 `python ... > hash.txt` や `$(python ...)` でも使える。
"""
import getpass
import importlib.util
import sys
from pathlib import Path

_ACC = Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "nokosahen_core" / "nokosahen_core"


def _load_accounts():
    # nokosahen_core/__init__.py や rclpy に依存せず、単体で読み込む
    import types

    pkg = types.ModuleType("_nk_pkg")
    pkg.__path__ = [str(_ACC)]
    sys.modules["_nk_pkg"] = pkg
    for name in ("roles", "accounts"):
        spec = importlib.util.spec_from_file_location(f"_nk_pkg.{name}", str(_ACC / f"{name}.py"))
        mod = importlib.util.module_from_spec(spec)
        sys.modules[f"_nk_pkg.{name}"] = mod
        spec.loader.exec_module(mod)
    return sys.modules["_nk_pkg.accounts"]


def main() -> int:
    acc = _load_accounts()
    print("operatorアカウントのパスワードを入力してください(表示されません)。", file=sys.stderr)
    p1 = getpass.getpass("パスワード: ")
    if not p1:
        print("空のパスワードは使えません。", file=sys.stderr)
        return 1
    if len(p1) < 8:
        print("8文字以上にしてください。", file=sys.stderr)
        return 1
    p2 = getpass.getpass("もう一度: ")
    if p1 != p2:
        print("一致しません。", file=sys.stderr)
        return 1
    print(acc.hash_password(p1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
