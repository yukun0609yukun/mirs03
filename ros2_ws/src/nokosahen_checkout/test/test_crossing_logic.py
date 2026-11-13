"""checkout_camera_nodeの「目の前の人だけを1回だけ数える」判定ロジックの単体テスト。

rclpy/OpenCV/ultralyticsが無い環境でも(このpytest実行環境含め)そのまま実行できるように、
検証対象をROS 2に依存しない純粋関数(crossing_logic.py)として切り出してある。
"""
import unittest

from nokosahen_checkout.crossing_logic import (
    has_crossed_line,
    is_near,
    should_count_crossing,
)


class TestIsNear(unittest.TestCase):
    def test_large_box_is_near(self) -> None:
        # 高さ480pxのフレームに対して高さ300px(比率0.625)の人物 -> 閾値0.55以上で近いと判定
        self.assertTrue(is_near(y1=100.0, y2=400.0, height=480.0, min_person_height_ratio=0.55))

    def test_small_box_is_not_near(self) -> None:
        # 奥を通り過ぎる背景の通行人を想定した小さいボックス
        self.assertFalse(is_near(y1=200.0, y2=250.0, height=480.0, min_person_height_ratio=0.55))

    def test_exactly_at_threshold_counts_as_near(self) -> None:
        # 比率がちょうど閾値と同じ場合は「近い」側に倒す(>=)
        self.assertTrue(is_near(y1=0.0, y2=264.0, height=480.0, min_person_height_ratio=0.55))

    def test_zero_height_is_never_near(self) -> None:
        self.assertFalse(is_near(y1=0.0, y2=100.0, height=0.0, min_person_height_ratio=0.55))


class TestHasCrossedLine(unittest.TestCase):
    def test_no_previous_position_means_not_crossed(self) -> None:
        # 初検出フレーム(前回位置が無い)ではまだ判定できない
        self.assertFalse(has_crossed_line(prev_x=None, cx=320.0, line_x=320.0))

    def test_left_to_right_crossing_detected(self) -> None:
        self.assertTrue(has_crossed_line(prev_x=300.0, cx=340.0, line_x=320.0))

    def test_right_to_left_crossing_detected(self) -> None:
        self.assertTrue(has_crossed_line(prev_x=340.0, cx=300.0, line_x=320.0))

    def test_staying_on_same_side_is_not_crossed(self) -> None:
        self.assertFalse(has_crossed_line(prev_x=280.0, cx=300.0, line_x=320.0))

    def test_left_to_right_direction_ignores_right_to_left(self) -> None:
        self.assertFalse(
            has_crossed_line(prev_x=340.0, cx=300.0, line_x=320.0, direction="left_to_right")
        )

    def test_left_to_right_direction_detects_matching_crossing(self) -> None:
        self.assertTrue(
            has_crossed_line(prev_x=300.0, cx=340.0, line_x=320.0, direction="left_to_right")
        )

    def test_right_to_left_direction_ignores_left_to_right(self) -> None:
        self.assertFalse(
            has_crossed_line(prev_x=300.0, cx=340.0, line_x=320.0, direction="right_to_left")
        )

    def test_right_to_left_direction_detects_matching_crossing(self) -> None:
        self.assertTrue(
            has_crossed_line(prev_x=340.0, cx=300.0, line_x=320.0, direction="right_to_left")
        )


class TestShouldCountCrossing(unittest.TestCase):
    def test_counts_when_all_conditions_met(self) -> None:
        self.assertTrue(
            should_count_crossing(
                crossed=True, is_stable=True, is_near_=True, already_counted=False
            )
        )

    def test_ignores_distant_person_even_if_crossed(self) -> None:
        # 「目の前の人だけに反応する」要件: 近接フィルタが最優先で効くこと
        self.assertFalse(
            should_count_crossing(
                crossed=True, is_stable=True, is_near_=False, already_counted=False
            )
        )

    def test_ignores_unstable_track(self) -> None:
        # トラッキング開始直後のノイズ的な誤検出を弾く
        self.assertFalse(
            should_count_crossing(
                crossed=True, is_stable=False, is_near_=True, already_counted=False
            )
        )

    def test_does_not_double_count_same_track(self) -> None:
        # 同じtrack_idを2回数えない(1人1回)
        self.assertFalse(
            should_count_crossing(
                crossed=True, is_stable=True, is_near_=True, already_counted=True
            )
        )

    def test_not_crossed_never_counts(self) -> None:
        self.assertFalse(
            should_count_crossing(
                crossed=False, is_stable=True, is_near_=True, already_counted=False
            )
        )


if __name__ == "__main__":
    unittest.main()
