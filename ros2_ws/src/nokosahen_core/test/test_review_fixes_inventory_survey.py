"""全体レビュー(Opus、2026-10-21)の指摘のうち、残数管理とアンケート受信の入力検査・壊れたファイル対策のテスト。

- inventory_manager: null・無限大・オブジェクトでない本文でも、例外でノードが落ちない(落ちると、メモリ上の本日の残数がすべて消える)
- receiver.py(ラズパイのアンケート受信): 壊れたJSONL行を飛ばす・途中で切れた末尾の次に保存しても読める・response_idが無い回答も保存する・
  CSVの数式インジェクション対策
実行: python ros2_ws/src/nokosahen_core/test/test_review_fixes_inventory_survey.py
"""
import importlib.util
import json
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent
REPO = TEST_DIR.parents[3]
sys.path.insert(0, str(TEST_DIR))
sys.path.insert(0, str(TEST_DIR.parent))

import test_behavior_rules as tbr  # noqa: E402,F401  (rclpyスタブの注入を流用)

try:
    from nokosahen_core import inventory_manager as im
    _IM_ERR = None
except Exception as exc:  # pragma: no cover
    im = None
    _IM_ERR = exc


class _Msg:
    def __init__(self, data):
        self.data = data


class _Log:
    def __init__(self):
        self.lines = []

    def warn(self, m):
        self.lines.append(m)

    def info(self, m):
        self.lines.append(m)

    def debug(self, m):
        self.lines.append(m)


class _Clock:
    def now(self):
        return 0


def make_inventory():
    n = object.__new__(im.InventoryManager)
    n._items = {}
    n._started_at = None
    n._consumption_log = None
    n.log_file = ""
    n.get_logger = lambda: n._log
    n._log = _Log()
    n.get_clock = lambda: _Clock()
    n._publish_state = lambda: None
    n._log_event = lambda *a, **k: None
    return n


@unittest.skipIf(im is None, f"inventory_manager import失敗: {_IM_ERR}")
class InventoryValidationTest(unittest.TestCase):
    def test_menu_init_skips_bad_item_but_keeps_good_ones(self):
        n = make_inventory()
        payload = {"items": [
            {"name": "カレー", "initial": 30},
            {"name": "うどん", "initial": None},          # TypeError
            {"name": "唐揚げ", "initial": "abc"},          # ValueError
            {"name": "ラーメン", "initial": 1e999},        # OverflowError (json.loadsはinfにする)
            {"name": "丼", "initial": 20, "remaining": 5},
        ]}
        n._on_menu_init(_Msg(json.dumps(payload)))  # 例外を出さない
        self.assertEqual(set(n._items), {"カレー", "丼"})
        self.assertEqual(n._items["カレー"].initial, 30)
        self.assertEqual(n._items["丼"].remaining, 5)

    def test_non_object_payloads_are_ignored(self):
        n = make_inventory()
        n._items = {"カレー": im.MenuItem("カレー", 30, 30, 0)}
        for bad in ("[1,2,3]", "null", '"x"', "12"):
            n._on_menu_init(_Msg(bad))
            n._on_count_event(_Msg(bad))
            n._on_sold_total_event(_Msg(bad))
        self.assertEqual(n._items["カレー"].remaining, 30)  # 在庫は消えない・変わらない
        n._on_menu_init(_Msg('{"items": 5}'))
        self.assertEqual(set(n._items), {"カレー"})

    def test_infinite_delta_and_sold_total_are_ignored(self):
        n = make_inventory()
        n._items = {"カレー": im.MenuItem("カレー", 30, 30, 0)}
        n._on_count_event(_Msg('{"menu": "カレー", "delta": 1e999}'))
        n._on_sold_total_event(_Msg('{"menu": "カレー", "sold_total": 1e999}'))
        self.assertEqual((n._items["カレー"].remaining, n._items["カレー"].sold), (30, 0))

    def test_normal_events_still_work(self):
        n = make_inventory()
        n._items = {"カレー": im.MenuItem("カレー", 30, 30, 0)}
        n._on_count_event(_Msg('{"menu": "カレー", "delta": 2}'))
        self.assertEqual(n._items["カレー"].remaining, 28)
        n._on_sold_total_event(_Msg('{"menu": "カレー", "sold_total": 5}'))
        self.assertEqual((n._items["カレー"].remaining, n._items["カレー"].sold), (25, 5))


def load_receiver(data_dir):
    os.environ["SURVEY_DATA_DIR"] = str(data_dir)
    path = REPO / "scripts" / "raspi_display" / "survey" / "receiver.py"
    spec = importlib.util.spec_from_file_location("survey_receiver_fix_%d" % abs(hash(str(data_dir))), str(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class ReceiverRobustnessTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.r = load_receiver(Path(self.tmp.name) / "data")

    def test_corrupt_line_is_skipped_and_new_saves_still_work(self):
        r = self.r
        ok, dup, err = r.save_record({"response_id": "a", "answers": {"q1": "1"}})
        self.assertTrue(ok, err)
        # 書き込み途中で電源が切れた想定: 末尾に、途中までの行(改行なし)
        with open(r.JSONL, "a", encoding="utf-8") as f:
            f.write('{"response_id": "broken", "answers": {"q1"')
        ok, dup, err = r.save_record({"response_id": "b", "answers": {"q1": "2"}})
        self.assertTrue(ok, err)  # 以前は、1行でも壊れていると、以後の回答がすべて失敗していた
        recs = r.load_all()
        self.assertEqual([x["response_id"] for x in recs], ["a", "b"])  # 壊れた行は飛ばされ、新しい回答は別の行に入る
        # 新しい回答が、壊れた行の末尾につながっていない(別の行になっている)
        lines = [l for l in Path(r.JSONL).read_text(encoding="utf-8").splitlines() if l.strip()]
        self.assertEqual(len(lines), 3)
        self.assertEqual(json.loads(lines[-1])["response_id"], "b")

    def test_missing_response_id_is_saved_as_separate_records(self):
        r = self.r
        for i in range(3):
            ok, dup, err = r.save_record({"answers": {"q1": str(i)}})
            self.assertTrue(ok, err)
            self.assertFalse(dup)  # 以前は2件目以降がNone同士で「重複」とされ、保存せずに保存完了を返していた
        recs = r.load_all()
        self.assertEqual(len(recs), 3)
        self.assertEqual(len({x["response_id"] for x in recs}), 3)

    def test_duplicate_response_id_is_still_deduplicated(self):
        r = self.r
        self.assertEqual(r.save_record({"response_id": "x", "answers": {}})[:2], (True, False))
        self.assertEqual(r.save_record({"response_id": "x", "answers": {}})[:2], (True, True))
        self.assertEqual(len(r.load_all()), 1)

    def test_csv_formula_injection_is_neutralised(self):
        r = self.r
        self.assertEqual(r.csv_safe("=1+1"), "'=1+1")
        self.assertEqual(r.csv_safe("+cmd"), "'+cmd")
        self.assertEqual(r.csv_safe("-2"), "'-2")
        self.assertEqual(r.csv_safe("@SUM(A1)"), "'@SUM(A1)")
        self.assertEqual(r.csv_safe("普通の文字"), "普通の文字")
        self.assertEqual(r.csv_safe(5), 5)
        ok, _, err = r.save_record({"response_id": "f", "location": "=HYPERLINK(\"http://x\")", "answers": {"q1": "=1+1"}})
        self.assertTrue(ok, err)
        text = Path(r.CSV_PATH).read_text(encoding="utf-8-sig")
        self.assertIn("'=HYPERLINK", text)
        self.assertIn("'=1+1", text)
        self.assertNotIn(",=1+1", text)


class StartNavTestWrapperTest(unittest.TestCase):
    def test_start_nav_test_is_an_alias_of_start_all(self):
        t = (REPO / "scripts" / "start_nav_test.sh").read_text(encoding="utf-8")
        self.assertIn('start_all.sh" nav_test "$@"', t)
        code = "\n".join(l for l in t.splitlines() if not l.lstrip().startswith("#"))
        self.assertNotIn("| tail", code)  # コード部分で、終了コードを失うパイプを使わない(説明のコメントは対象外)


if __name__ == "__main__":
    unittest.main(verbosity=1)
