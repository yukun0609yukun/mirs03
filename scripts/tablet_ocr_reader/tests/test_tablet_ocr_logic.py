"""tablet_ocr_reader.pyの純粋ロジックの単体テスト(2026-10-02追加)。

実行: python -m unittest discover -s scripts/tablet_ocr_reader/tests -v
(カメラ・Tesseract・Edge TPU・のこさへん本体は不要。detect_digit_rowsの
テストだけはcv2が無い環境ではスキップされる。)
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import tablet_ocr_reader as t  # noqa: E402

try:
    import cv2  # type: ignore
    import numpy as np
except ImportError:  # pragma: no cover
    cv2 = None


class SelectBestRowsTest(unittest.TestCase):
    def _col(self, n, x1=800, x2=900, y0=50, pitch=100):
        return [(x1, y0 + i * pitch, x2, y0 + i * pitch + 40) for i in range(n)]

    def test_fewer_than_expected_returns_none(self):
        self.assertIsNone(t.select_best_rows(self._col(6)))

    def test_exact_seven_sorted_by_y(self):
        boxes = list(reversed(self._col(7)))
        self.assertEqual(t.select_best_rows(boxes), self._col(7))

    def test_extras_far_from_median_column_are_dropped(self):
        rows = self._col(7)
        extras = [(100, 120, 200, 160), (1100, 470, 1200, 510)]  # 左端・右端のノイズ
        result = t.select_best_rows(rows + extras)
        self.assertEqual(result, rows)

    def test_result_is_sorted_by_y_even_if_extra_in_middle(self):
        rows = self._col(7)
        extra = (820, 175, 920, 215)  # 縦列に近い余分な行(y順で途中に入る)
        result = t.select_best_rows(rows + [extra])
        self.assertEqual(len(result), 7)
        self.assertEqual([b[1] for b in result], sorted(b[1] for b in result))
        # 縦列に最も近い7個(元の7行の方がx中心が中央値に近い)が残る
        self.assertNotIn(extra, result)


class JumpGuardTest(unittest.TestCase):
    def test_first_read_without_baseline_passes(self):
        g = t.JumpGuard(50, 6)
        self.assertEqual(g.check(120, None), (120, None))

    def test_increase_and_equal_pass(self):
        g = t.JumpGuard(50, 6)
        self.assertEqual(g.check(10, 10), (10, None))
        self.assertEqual(g.check(59, 10), (59, None))  # 差49は許容(tab.pyと同じ <50)

    def test_decrease_rejected(self):
        g = t.JumpGuard(50, 6)
        v, reason = g.check(9, 10)
        self.assertIsNone(v)
        self.assertIn("減少", reason)

    def test_big_jump_rejected(self):
        g = t.JumpGuard(50, 6)
        v, reason = g.check(60, 10)  # 差50は棄却
        self.assertIsNone(v)
        self.assertIn("急増", reason)

    def test_none_and_disabled_passthrough(self):
        self.assertEqual(t.JumpGuard(50, 6).check(None, 10), (None, None))
        self.assertEqual(t.JumpGuard(0, 6).check(1, 100), (1, None))

    def test_consistent_reject_is_accepted_after_override_reads(self):
        g = t.JumpGuard(50, 6)
        results = [g.check(2, 100)[0] for _ in range(5)]
        self.assertEqual(results, [None] * 5)
        v, reason = g.check(2, 100)  # 6回目で受理(日替わりリセット/誤確定の訂正)
        self.assertEqual(v, 2)
        self.assertIn("受理", reason)
        # 受理後、スタビライザーがまだ確定させる前(baselineは100のまま)でも同じ値は通る
        self.assertEqual(g.check(2, 100), (2, None))
        self.assertEqual(g.check(3, 100), (3, None))
        # スタビライザーが値を確定させた(baselineが変わった)後は通常判定に戻る
        self.assertEqual(g.check(4, 2), (4, None))
        self.assertIsNone(g.check(1, 4)[0])

    def test_alternating_rejected_values_do_not_override(self):
        g = t.JumpGuard(50, 4)
        for v in (2, 3, 2, 3, 2, 3, 2, 3):
            self.assertIsNone(g.check(v, 100)[0])

    def test_in_range_read_resets_reject_streak(self):
        g = t.JumpGuard(50, 3)
        self.assertIsNone(g.check(2, 100)[0])
        self.assertIsNone(g.check(2, 100)[0])
        self.assertEqual(g.check(101, 100), (101, None))  # 正常な読み取りで連続が途切れる
        self.assertIsNone(g.check(2, 100)[0])  # 数え直し(1回目)
        self.assertIsNone(g.check(2, 100)[0])


class RoiLockerTest(unittest.TestCase):
    W, H = 1280, 960

    def _rows(self, dx=0, dy=0):
        return [(800 + dx, 60 + i * 120 + dy, 900 + dx, 100 + i * 120 + dy) for i in range(7)]

    def test_locks_after_required_consistent_cycles(self):
        lk = t.RoiLocker(required=3, tolerance=0.04)
        self.assertFalse(lk.observe(self._rows(), self.W, self.H))
        self.assertFalse(lk.observe(self._rows(dx=5), self.W, self.H))
        self.assertTrue(lk.observe(self._rows(dy=-5), self.W, self.H))
        self.assertTrue(lk.is_locked)
        self.assertEqual(len(lk.locked), 7)
        # ロック後は何を渡しても変化しない
        self.assertFalse(lk.observe(self._rows(dx=300), self.W, self.H))

    def test_none_or_wrong_count_breaks_the_run(self):
        lk = t.RoiLocker(required=3)
        lk.observe(self._rows(), self.W, self.H)
        lk.observe(self._rows(), self.W, self.H)
        lk.observe(None, self.W, self.H)
        self.assertEqual(lk.progress, 0)
        lk.observe(self._rows(), self.W, self.H)
        self.assertFalse(lk.is_locked)

    def test_inconsistent_positions_restart_the_run(self):
        lk = t.RoiLocker(required=3, tolerance=0.04)
        lk.observe(self._rows(), self.W, self.H)
        lk.observe(self._rows(), self.W, self.H)
        lk.observe(self._rows(dx=300), self.W, self.H)  # 大きくずれた
        self.assertEqual(lk.progress, 1)
        self.assertFalse(lk.is_locked)

    def test_locked_box_contains_all_samples_with_margin(self):
        lk = t.RoiLocker(required=2, tolerance=0.04, margin_ratio=0.05)
        lk.observe(self._rows(), self.W, self.H)
        lk.observe(self._rows(dx=10), self.W, self.H)
        x1, y1, x2, y2 = lk.locked[0]
        self.assertLessEqual(x1, 800 / self.W)
        self.assertGreaterEqual(x2, 910 / self.W)
        self.assertTrue(0.0 <= x1 < x2 <= 1.0 and 0.0 <= y1 < y2 <= 1.0)

    def test_reset_after_consecutive_unreadable_cycles(self):
        lk = t.RoiLocker(required=1, reset_after_fail_cycles=3)
        self.assertTrue(lk.observe(self._rows(), self.W, self.H))
        self.assertFalse(lk.note_cycle_result(False))
        self.assertFalse(lk.note_cycle_result(False))
        self.assertFalse(lk.note_cycle_result(True))  # 読めたのでカウンタはリセット
        self.assertFalse(lk.note_cycle_result(False))
        self.assertFalse(lk.note_cycle_result(False))
        self.assertTrue(lk.note_cycle_result(False))
        self.assertFalse(lk.is_locked)

    def test_load_and_config_roundtrip(self):
        lk = t.RoiLocker(required=1)
        lk.observe(self._rows(), self.W, self.H)
        as_dict = t.rois_to_config_dict(lk.locked)
        self.assertEqual(list(as_dict), t.FULL_ROW_ORDER)
        parsed = t.parse_locked_rois(as_dict)
        self.assertEqual(len(parsed), 7)
        other = t.RoiLocker()
        other.load(parsed)
        self.assertTrue(other.is_locked)
        self.assertIsNone(t.parse_locked_rois(None))
        self.assertIsNone(t.parse_locked_rois({"A": {"x1": 0}}))


@unittest.skipIf(cv2 is None, "cv2/numpyが無いためdetect_digit_rowsの合成画像テストはスキップ")
class DetectDigitRowsSyntheticTest(unittest.TestCase):
    W, H = 1280, 960

    def _frame(self, n_rows=7, extra_blobs=()):
        img = np.full((self.H, self.W, 3), 255, np.uint8)
        red = (30, 30, 200)  # BGRの赤茶色
        for i in range(n_rows):
            y = 80 + i * 110
            for k, ch in enumerate("12"):  # 桁を離して2輪郭にする(1輪郭だけの行は除外されるため)
                cv2.putText(img, ch, (900 + k * 55, y), cv2.FONT_HERSHEY_SIMPLEX, 2.2, red, 6)
        for (x, y) in extra_blobs:
            for k, ch in enumerate("34"):
                cv2.putText(img, ch, (x + k * 55, y), cv2.FONT_HERSHEY_SIMPLEX, 2.2, red, 6)
        return img

    def test_detects_seven_rows(self):
        boxes = t.detect_digit_rows(cv2, np, self._frame())
        self.assertEqual(len(boxes), 7)

    def test_extra_row_is_reduced_to_best_seven(self):
        # 8行目を少し左にずれた位置に置く(行の右端が中央値から外れないのでdetect側は残す)
        frame = self._frame(extra_blobs=[(860, 80 + 7 * 110)])
        boxes = t.detect_digit_rows(cv2, np, frame)
        self.assertGreaterEqual(len(boxes), 7)
        best = t.select_best_rows(boxes, 7)
        self.assertIsNotNone(best)
        self.assertEqual(len(best), 7)

    def test_locker_locks_on_synthetic_frames(self):
        lk = t.RoiLocker(required=3)
        for _ in range(3):
            boxes = t.select_best_rows(t.detect_digit_rows(cv2, np, self._frame()), 7)
            lk.observe(boxes, self.W, self.H)
        self.assertTrue(lk.is_locked)


@unittest.skipIf(cv2 is None, "cv2/numpyが無いためdetect_digit_rowsの合成画像テストはスキップ")
class SingleDigitRowTest(unittest.TestCase):
    """残数が1桁の行(輪郭1個)が捨てられて検出行数が6になる不具合の回帰テスト(2026-10-05)。"""
    W, H = 1280, 960
    RED = (30, 30, 200)

    def _frame(self, digit_counts=(1, 2, 3, 1, 2, 1, 2), noise=()):
        img = np.full((self.H, self.W, 3), 255, np.uint8)
        for i, n in enumerate(digit_counts):
            y = 80 + i * 110
            for k in range(n):  # 右端(x=955)を揃えて桁を左へ増やす
                x = 955 - (n - 1 - k) * 55
                cv2.putText(img, str((i + k) % 9 + 1), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 2.2, self.RED, 6)
        for (x, y, scale, text) in noise:
            cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, self.RED, 6 if scale > 1 else 2)
        return img

    def test_mixed_digit_counts_give_seven_rows(self):
        boxes = t.detect_digit_rows(cv2, np, self._frame())
        self.assertEqual(len(boxes), 7)
        self.assertEqual([b[1] for b in boxes], sorted(b[1] for b in boxes))

    def test_all_single_digits_only_with_few_multi_rows(self):
        boxes = t.detect_digit_rows(cv2, np, self._frame((1, 1, 2, 1, 1, 2, 1)))
        self.assertEqual(len(boxes), 7)

    def test_all_single_digit_rows_without_any_multi_row_returns_empty(self):
        # 基準になる複数輪郭の行が無ければ従来通り何も返さない
        self.assertEqual(t.detect_digit_rows(cv2, np, self._frame((1,) * 7)), [])

    def test_lone_small_blob_is_rejected(self):
        frame = self._frame(noise=[(300, 520, 0.8, "8")])
        self.assertEqual(len(t.detect_digit_rows(cv2, np, frame)), 7)

    def test_single_blob_far_off_column_is_rejected(self):
        # 高さは同じだが列(x=955付近)から大きく外れた単独の数字
        frame = self._frame(noise=[(200, 80 + 55 + 3 * 110, 2.2, "7")])
        boxes = t.detect_digit_rows(cv2, np, frame)
        self.assertEqual(len(boxes), 7)
        self.assertTrue(all(b[0] > 600 for b in boxes))

    def test_single_blob_in_column_but_wrong_height_is_rejected(self):
        frame = self._frame(noise=[(950, 80 + 55 + 2 * 110, 1.0, "9")])
        self.assertEqual(len(t.detect_digit_rows(cv2, np, frame)), 7)

    def test_select_best_rows_still_works_with_extra_column_row(self):
        # 縦列上にもう1行(1桁)あれば8行になり、select_best_rowsが7行へ絞る
        frame = self._frame(noise=[(955, 80 + 7 * 110, 2.2, "4")])
        boxes = t.detect_digit_rows(cv2, np, frame)
        self.assertEqual(len(boxes), 8)
        best = t.select_best_rows(boxes, 7)
        self.assertEqual(len(best), 7)

    def test_locker_locks_with_single_digit_rows(self):
        lk = t.RoiLocker(required=3)
        for _ in range(3):
            boxes = t.select_best_rows(t.detect_digit_rows(cv2, np, self._frame()), 7)
            self.assertIsNotNone(boxes)
            lk.observe(boxes, self.W, self.H)
        self.assertTrue(lk.is_locked)

    def test_merge_kernel_option_runs_and_keeps_rows(self):
        boxes = t.detect_digit_rows(cv2, np, self._frame(), merge_kernel_w=20)
        self.assertEqual(len(boxes), 7)


class NumbersTxtWriterTest(unittest.TestCase):
    def setUp(self):
        import tempfile
        self.dir = tempfile.mkdtemp()
        self.path = os.path.join(self.dir, "numbers.txt")

    def _lines(self):
        if not os.path.exists(self.path):
            return []
        with open(self.path, encoding="utf-8") as f:
            return f.read().splitlines()

    def test_disabled_when_path_empty(self):
        w = t.NumbersTxtWriter("")
        self.assertIsNone(w.update({"A": 1}))
        self.assertFalse(w.enabled)

    def test_all_unknown_writes_nothing(self):
        w = t.NumbersTxtWriter(self.path)
        self.assertIsNone(w.update({s: None for s in t.FULL_ROW_ORDER}))
        self.assertEqual(self._lines(), [])

    def test_order_and_unknown_marker_and_on_change_only(self):
        w = t.NumbersTxtWriter(self.path)
        w.update({"A": 5, "B": 12})
        w.update({"A": 5, "B": 12})  # 変化なし
        w.update({"A": 5, "B": 13, "alacarte_100": 0})
        self.assertEqual(self._lines(), ["5,12,---,---,---,---,---", "5,13,---,---,---,---,0"])

    def test_every_cycle_when_not_on_change_only(self):
        w = t.NumbersTxtWriter(self.path, on_change_only=False)
        w.update({"A": 1})
        w.update({"A": 1})
        self.assertEqual(len(self._lines()), 2)

    def test_none_keeps_previous_value(self):
        w = t.NumbersTxtWriter(self.path)
        w.update({"A": 1, "B": 2})
        w.update({"A": None, "B": 3})
        self.assertEqual(self._lines()[-1], "1,3,---,---,---,---,---")

    def test_estimated_not_written_by_default(self):
        w = t.NumbersTxtWriter(self.path)
        w.update({"A": 10}, {"A": False})
        w.update({"A": 14}, {"A": True})  # 推定値: 直近の実測10のまま、変化なしなので追記なし
        self.assertEqual(self._lines(), ["10,---,---,---,---,---,---"])
        w2 = t.NumbersTxtWriter(self.path, include_estimated=True)
        w2.update({"A": 14}, {"A": True})
        self.assertEqual(self._lines()[-1], "14,---,---,---,---,---,---")

    def test_timestamp_prefix(self):
        w = t.NumbersTxtWriter(self.path, with_timestamp=True, clock=lambda: 0.0)
        out = w.update({"A": 1})
        self.assertRegex(out, r"^\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d,1,---")

    def test_write_failure_never_raises_and_warns_once(self):
        bad = os.path.join(self.dir, "no_such_dir", "x.txt")
        w = t.NumbersTxtWriter(bad)
        with self.assertLogs("tablet_ocr_reader", level="WARNING") as cm:
            self.assertIsNone(w.update({"A": 1}))
            self.assertIsNone(w.update({"A": 2}))
        self.assertEqual(len(cm.output), 1)


if __name__ == "__main__":
    unittest.main()
