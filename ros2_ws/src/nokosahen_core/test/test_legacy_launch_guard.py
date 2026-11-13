"""旧構成の起動ファイルのガードのテスト(2026-10-21、全体レビューの指摘)。ROSは使わない(ガード部分だけを取り出して実行する)。

- jetson_robot / jetson_robot_full / localization_robot / raspi4_robot の launch は、既定では起動せず、案内を出して止まる
- NOKOSAHEN_ALLOW_LEGACY_LAUNCH=1 を付けたときだけ通す
- 案内に、いまの起動の方法(start_all.sh)が書いてある
- いまの運用で使う mapping_mode / localization_mode には、ガードを入れていない(使えなくならない)
実行: python ros2_ws/src/nokosahen_core/test/test_legacy_launch_guard.py
"""
import os
import re
import unittest
from pathlib import Path
from unittest import mock

LAUNCH = Path(__file__).resolve().parents[2] / "nokosahen_bringup" / "launch"
LEGACY = ("jetson_robot", "jetson_robot_full", "localization_robot", "raspi4_robot")
CURRENT = ("mapping_mode", "localization_mode", "persistent_base")


def guard_namespace(src):
    """launchファイルから、ガードの部分(import os 〜 _legacy_launch_guard)だけを取り出して実行する。"""
    start = src.index("import os")
    end = src.index("def generate_launch_description")
    ns = {}
    exec(src[start:end], ns)  # noqa: S102 - 自分のリポジトリのlaunchファイルの、ガード部分だけ
    return ns


class LegacyGuardTest(unittest.TestCase):
    def test_every_legacy_launch_calls_the_guard_first(self):
        for name in LEGACY:
            src = (LAUNCH / f"{name}.launch.py").read_text(encoding="utf-8")
            i = src.index("def generate_launch_description")
            body = src[i:].split("\n", 2)
            self.assertIn(f'_legacy_launch_guard("{name}.launch.py")', body[1] + body[2][:200], name)

    def test_guard_blocks_by_default_and_allows_with_env(self):
        for name in LEGACY:
            ns = guard_namespace((LAUNCH / f"{name}.launch.py").read_text(encoding="utf-8"))
            guard = ns["_legacy_launch_guard"]
            with mock.patch.dict(os.environ, {}, clear=False):
                os.environ.pop("NOKOSAHEN_ALLOW_LEGACY_LAUNCH", None)
                with self.assertRaises(RuntimeError) as cm:
                    guard(f"{name}.launch.py")
                msg = str(cm.exception)
                self.assertIn(f"{name}.launch.py", msg)
                self.assertIn("start_all.sh", msg)
                self.assertIn("NOKOSAHEN_ALLOW_LEGACY_LAUNCH=1", msg)
            for bad in ("0", "", "true", "yes"):
                with mock.patch.dict(os.environ, {"NOKOSAHEN_ALLOW_LEGACY_LAUNCH": bad}):
                    with self.assertRaises(RuntimeError, msg=bad):
                        guard(name)
            with mock.patch.dict(os.environ, {"NOKOSAHEN_ALLOW_LEGACY_LAUNCH": "1"}):
                guard(name)  # 例外を出さない

    def test_current_launch_files_are_not_guarded(self):
        for name in CURRENT:
            src = (LAUNCH / f"{name}.launch.py").read_text(encoding="utf-8")
            self.assertNotIn("_legacy_launch_guard", src, name)
            self.assertNotIn("NOKOSAHEN_ALLOW_LEGACY_LAUNCH", src, name)

    def test_syntax(self):
        for name in LEGACY:
            compile((LAUNCH / f"{name}.launch.py").read_text(encoding="utf-8"), name, "exec")


if __name__ == "__main__":
    unittest.main(verbosity=1)
