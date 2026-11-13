"""動きの重複対策の、純粋な判定部品(2026-10-23、docs/overlap_audit_2026-10-23.md の B2/B3/B6)。ROS不要。

- HoldCancelEdge: HOLD系で /nokosahen/cancel_goal を出すのを、状態が切り替わった最初の1回だけにする(B2)
- UserCancelHold: 「ユーザー中止」を受けてから一定時間、ゴールの発行を止める(B6)
- GrantTracker: /nokosahen/motion_grant の受信状況から、自分が動いてよいかを返す(B3)
いずれも時刻は引数で受け取る(単体テスト用)。
"""
from __future__ import annotations

import json
from typing import Optional


class HoldCancelEdge:
    """HOLD中の cancel_goal を、HOLDに入った(または別のHOLDに切り替わった)最初の1回だけにする。

    once=False なら従来どおり毎tick出す。HOLD以外のモードを通ると記憶をリセットする
    (次にHOLDへ入ったとき、また最初の1回を出す)。ゼロ速度(/cmd_vel)は、ここでは扱わない=呼び出し側が毎tick出す。
    """

    def __init__(self, once: bool = True) -> None:
        self.once = once
        self._mode: Optional[str] = None

    def should_cancel(self, mode: str) -> bool:
        if not self.once:
            return True
        if mode == self._mode:
            return False
        self._mode = mode
        return True

    def left_hold(self) -> None:
        """HOLD以外のモードになった(次のHOLDで、また最初の1回を出す)。"""
        self._mode = None


class UserCancelHold:
    """ユーザー中止(/nokosahen/user_cancel=true)を受けてから hold_sec 秒の間、ゴールの発行を止める。

    hold_sec <= 0 なら無効(常に active()=False)。専用トピックなので、自分が出した cancel_goal とは混ざらない。
    """

    def __init__(self, hold_sec: float = 10.0) -> None:
        self.hold_sec = float(hold_sec)
        self._until: Optional[float] = None

    @property
    def enabled(self) -> bool:
        return self.hold_sec > 0.0

    @property
    def pending(self) -> bool:
        """ユーザー中止を一度でも受けたか(時刻を取る前の軽い判定。受けていなければ時計を読まずに済む)。"""
        return self.enabled and self._until is not None

    def on_user_cancel(self, now: float) -> None:
        if self.enabled:
            self._until = now + self.hold_sec

    def active(self, now: float) -> bool:
        return self.enabled and self._until is not None and now < self._until

    def clear(self) -> None:
        self._until = None


class GrantTracker:
    """/nokosahen/motion_grant(JSON)を保持し、名前ごとの許可を返す。

    - 一度も受信していない=motion_arbiterの無い構成は、従来どおり許可
    - 受信が stale_sec 秒途絶えたら不許可(安全側)
    - 受信した内容に名前が無い/壊れている場合は不許可
    required=False なら常に許可(無効化用)。
    """

    def __init__(self, name: str, stale_sec: float = 1.0, required: bool = True) -> None:
        self.name = name
        self.stale_sec = stale_sec
        self.required = required
        self._allowed = False
        self._time: Optional[float] = None

    def on_message(self, data: str, now: float) -> None:
        try:
            self._allowed = bool(json.loads(data).get("allow", {}).get(self.name, False))
        except (ValueError, TypeError, AttributeError):
            self._allowed = False
        self._time = now

    def ok(self, now: float) -> bool:
        if not self.required:
            return True
        if self._time is None:
            return True
        if now - self._time > self.stale_sec:
            return False
        return self._allowed
