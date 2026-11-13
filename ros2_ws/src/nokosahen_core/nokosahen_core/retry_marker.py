"""wall_escape が出し直した /goal_pose の目印(2026-10-23, 動作のかぶり調査 B7)。

従来は「次に届く /goal_pose は自分の出し直し」というフラグ(_ignore_next_goal)だった。しかし、出し直しの前後に別の出し元
(ダッシュボード・director・explorer)の目的地が先に届くと、そちらを誤って「自分の出し直し」と扱って記憶せず、
自分の出し直しのほうはフラグが残り続ける(次の本物の目的地まで)恐れがあった。

ここでは出し直した目的地の (x, y, yaw) と時刻を覚え、届いた目的地が一致したときだけ「自分の出し直し」と判定する。
一致しない目的地は通常どおり扱い、目印は ttl_sec 秒で失効する(残り続けない)。
match=False は従来動作(次の1件を無条件で無視、失効なし)。ROSなしで使える純粋なクラス。
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

Pose3 = Tuple[float, float, float]


def pose_key(msg) -> Pose3:
    """PoseStamped から (x, y, yaw)。"""
    p = msg.pose.position
    q = msg.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return float(p.x), float(p.y), yaw


class RetryMarker:
    def __init__(self, ttl_sec: float = 2.0, tol_xy: float = 1e-3, tol_yaw: float = 1e-3, match: bool = True) -> None:
        self.ttl = float(ttl_sec)
        self.tol_xy = float(tol_xy)
        self.tol_yaw = float(tol_yaw)
        self.match = bool(match)
        self._pose: Optional[Pose3] = None
        self._until = 0.0
        self._active = False

    def mark(self, pose: Pose3, now: float) -> None:
        """出し直す直前に呼ぶ。now は秒。"""
        self._pose = pose
        self._until = now + self.ttl
        self._active = True

    def pending(self, now: float) -> bool:
        if not self._active:
            return False
        return (not self.match) or now < self._until

    def consume(self, pose: Pose3, now: float) -> bool:
        """届いた目的地が自分の出し直しなら True(目印は使い切る)。それ以外は False(目印は失効まで残る)。"""
        if not self._active:
            return False
        if not self.match:
            self.clear()
            return True
        if now >= self._until:
            self.clear()
            return False
        ref = self._pose
        dyaw = abs(math.atan2(math.sin(pose[2] - ref[2]), math.cos(pose[2] - ref[2])))
        if abs(pose[0] - ref[0]) <= self.tol_xy and abs(pose[1] - ref[1]) <= self.tol_xy and dyaw <= self.tol_yaw:
            self.clear()
            return True
        return False

    def clear(self) -> None:
        self._active = False
        self._pose = None
