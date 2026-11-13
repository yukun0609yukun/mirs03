"""近接停止(/nokosahen/near_person_stop)の受信途絶の見張り(判定だけ。ROS不要)。【2026-11-04】

person_target_detector は近接停止を10Hz前後で出し続ける。このノードが落ちると、最後の値(false)が張り付いて、近接停止が黙って効かなくなる。
そこで「一度でも受信があったのに、stale_sec 秒を超えて途絶えた」ときは、近接停止 true と同じ扱い(安全側)にする。
**一度も受信していない間は適用しない**(検出ノードを起動しない構成=近接停止なしの運用が普通にあり、永久停止にしてはいけない)。
stale_sec が 0 以下なら無効。
"""
from typing import Optional

NEAR_PERSON_STALE_SEC = 2.0


class NearPersonWatch:
    def __init__(self, stale_sec: float = NEAR_PERSON_STALE_SEC) -> None:
        self.stale_sec = float(stale_sec)
        self.value = False
        self.last_time: Optional[float] = None
        self.warned = False

    def on_message(self, now: float, value: bool) -> None:
        self.value = bool(value)
        self.last_time = now
        self.warned = False  # 再開したら、次の途絶でまた1回警告する

    def is_stale(self, now: float) -> bool:
        return self.stale_sec > 0 and self.last_time is not None and (now - self.last_time) > self.stale_sec

    def effective(self, now: float) -> bool:
        """近接停止として扱うか(最後の値が true、または途絶)。"""
        return self.value or self.is_stale(now)

    def should_warn(self, now: float) -> bool:
        """途絶中、警告を出すべきか(途絶ごとに1回だけTrue)。"""
        if self.is_stale(now) and not self.warned:
            self.warned = True
            return True
        return False
