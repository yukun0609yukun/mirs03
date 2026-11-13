"""前方カメラの「読み込み失敗が続いたら解放して、間隔を空けて開き直す」判断(純粋ロジック。rclpy・cv2不要)。

person_target_detector が使う。USBの抜け・挿し直し、カメラの固まりから自動で復帰するため
(rear_camera_persons の RetryTimer と同じ考え方)。時計は単調時計を渡す。
"""
from __future__ import annotations

from typing import Optional


class CameraRecovery:
    def __init__(self, release_after_sec: float = 3.0, reopen_interval_sec: float = 10.0) -> None:
        self.release_after_sec = max(0.5, float(release_after_sec))
        self.reopen_interval_sec = max(1.0, float(reopen_interval_sec))
        self._fail_since: Optional[float] = None
        self._last_attempt: Optional[float] = None
        self.down = False

    def on_frame_ok(self) -> bool:
        """フレームが取れた。落ちていた状態から戻ったとき True。"""
        self._fail_since = None
        was_down = self.down
        self.down = False
        return was_down

    def on_frame_fail(self, now: float) -> bool:
        """フレームが取れなかった。解放すべき(落ちたと確定した)ときに初めて True を返す。"""
        if self.down:
            return False
        if self._fail_since is None or now < self._fail_since:
            self._fail_since = now
            return False
        if now - self._fail_since >= self.release_after_sec:
            self.down = True
            self._last_attempt = now        # 解放した直後は、すぐには開き直さない
            return True
        return False

    def mark_down(self, now: float) -> None:
        """起動時に開けなかった場合など、最初から落ちている状態にする(次の試行は間隔を空ける)。"""
        self.down = True
        self._last_attempt = now

    def should_reopen(self, now: float) -> bool:
        """落ちている間、前回の試行から reopen_interval_sec 経っていれば True(True を返したら試行した扱い)。"""
        if not self.down:
            return False
        if self._last_attempt is not None and 0 <= now - self._last_attempt < self.reopen_interval_sec:
            return False
        self._last_attempt = now
        return True

    def opened(self) -> None:
        """開き直しに成功した。失敗計測をやり直す。"""
        self.down = False
        self._fail_since = None
