"""ソフト緊急停止の状態の保存・復元のテスト(2026-10-21、全体レビューの指摘)。rclpy不要(スタブ)。

- 変更のたびに保存し、起動時に復元して「停止中」から始める(監視ループの再起動で、緊急停止が勝手に解除されない)
- 保存先なし・ファイルなし・壊れたファイル・解除済みは、従来どおり「解除」で起動
- 意図した停止のスクリプト(shutdown_all.sh / stop_everything.sh)は、保存ファイルを消す
実行: python ros2_ws/src/nokosahen_core/test/test_soft_estop_persist.py
"""
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

TEST_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(TEST_DIR.parent))

import test_teleop_seq as tts  # noqa: E402  (rclpyスタブの注入と_Nodeを流用)

dc = tts.dc
REPO = TEST_DIR.parents[3]


@unittest.skipIf(dc is None, f"dashboard_common import失敗: {tts._IMPORT_ERR}")
class PersistTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = str(Path(self.tmp.name) / "estop.json")

    def make(self, path=None):
        return dc.TeleopController(tts._Node(), soft_estop_state_file=self.path if path is None else path)

    def test_default_no_persistence_starts_released(self):
        t = dc.TeleopController(tts._Node())
        self.assertFalse(t.soft_estop_engaged)
        t.handle_soft_estop(True)  # 保存先が無くても、例外を出さない
        self.assertTrue(t.soft_estop_engaged)

    def test_engaged_state_is_saved_and_restored_on_restart(self):
        t = self.make()
        self.assertFalse(t.soft_estop_engaged)  # ファイルが無い → 解除で起動
        t.handle_soft_estop(True)
        self.assertEqual(json.loads(Path(self.path).read_text(encoding="utf-8"))["engaged"], True)
        # 監視ループがROS一式を再起動した想定 → 停止中のまま起動
        t2 = self.make()
        self.assertTrue(t2.soft_estop_engaged)
        self.assertTrue(t2.status()["soft_estop_engaged"])

    def test_release_is_saved_and_next_start_is_released(self):
        t = self.make()
        t.handle_soft_estop(True)
        t.handle_soft_estop(False)
        self.assertEqual(json.loads(Path(self.path).read_text(encoding="utf-8"))["engaged"], False)
        self.assertFalse(self.make().soft_estop_engaged)

    def test_broken_or_unexpected_file_means_released(self):
        for content in ("", "not json", "[1]", '{"engaged": "yes"}', '{"engaged": 1}', '{"engaged": false}'):
            Path(self.path).write_text(content, encoding="utf-8")
            self.assertFalse(self.make().soft_estop_engaged, content)

    def test_unwritable_path_does_not_break_estop(self):
        t = self.make(path=str(Path(self.tmp.name) / "no_such_dir" / "x.json"))
        t.handle_soft_estop(True)  # 保存に失敗しても、緊急停止そのものは効く
        self.assertTrue(t.soft_estop_engaged)

    def test_no_temp_file_left_behind(self):
        t = self.make()
        t.handle_soft_estop(True)
        self.assertFalse(Path(self.path + ".tmp").exists())


class IntentionalStopClearsStateTest(unittest.TestCase):
    def test_scripts_remove_the_state_file(self):
        for name in ("shutdown_all.sh", "stop_everything.sh"):
            t = (REPO / "scripts" / name).read_text(encoding="utf-8")
            self.assertIn('rm -f "$HOME/.nokosahen_soft_estop.json"', t, name)

    def test_full_dashboard_passes_the_state_file(self):
        t = (TEST_DIR.parent / "nokosahen_core" / "full_dashboard_node.py").read_text(encoding="utf-8")
        self.assertIn('declare_parameter("soft_estop_state_file", "~/.nokosahen_soft_estop.json")', t)
        self.assertIn('soft_estop_state_file=str(self.get_parameter("soft_estop_state_file").value)', t)


if __name__ == "__main__":
    unittest.main(verbosity=1)
