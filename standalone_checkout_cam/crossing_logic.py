"""決済カメラの「目の前の人だけを1回だけ数える」判定ロジック。

`ros2_ws/src/nokosahen_checkout/nokosahen_checkout/crossing_logic.py` と
同一内容のコピー。ROS2/OpenCV/ultralyticsに依存しない純粋関数のみなので、
ROS2が無い環境(Jetson Nano等)でもこのファイル1つをこのフォルダに置くだけで
動く。判定ロジックを変更する場合は、必ずros2_ws側のオリジナルも合わせて
更新すること(2箇所の内容がズレると、同じハードのRaspberry Pi版と
Jetson Nano版でカウント挙動が違ってしまう)。
"""
from __future__ import annotations

from typing import Optional


def is_near(y1: float, y2: float, height: float, min_person_height_ratio: float) -> bool:
    """バウンディングボックスがフレームに対して十分大きい(=画面の近く・真正面)か。"""
    if height <= 0:
        return False
    return ((y2 - y1) / height) >= min_person_height_ratio


def has_crossed_line(
    prev_x: Optional[float], cx: float, line_x: float, direction: str = "both"
) -> bool:
    """中心Xが、前回フレームから今回フレームの間にラインをまたいだか。

    direction="both"(既定)ならどちら向きの通過も検知する(決済端末は
    近づいて離れる向きがどちらでも1回の購入とみなしたいため)。
    "left_to_right"/"right_to_left"を指定すると、その向きの通過だけを
    検知する。
    """
    if prev_x is None:
        return False
    left_to_right = prev_x < line_x <= cx
    right_to_left = prev_x > line_x >= cx
    if direction == "left_to_right":
        return left_to_right
    if direction == "right_to_left":
        return right_to_left
    return left_to_right or right_to_left


def should_count_crossing(
    *, crossed: bool, is_stable: bool, is_near_: bool, already_counted: bool
) -> bool:
    """このtrack_idの通過をカウントすべきか(近接+安定+未カウントのライン通過のみ)。"""
    return crossed and is_stable and is_near_ and not already_counted
