"""迂回の処理の状態機械(2026-10-25)。ROS不要の純粋なクラス。入力=イベント、出力=次の動作(アクション)のリスト。

Nav2の目的地が、BTの回復(2回)でも失敗して中止(status 6)になったとき、後退せずに「経路をシミュレーションして、越せると分かってから進む」ための流れ:

    待つ(コストマップとLiDARの更新) -> 元の目的地への経路を計画 -> シミュレーションで越せるか判定
      越せる      -> (必要なら経路の向きへ旋回して)元の目的地を送る        [attemptsを1つ消費]
      越せない    -> 中継点の候補を順に: 中継点への経路・中継点から目的地への経路を計画 -> 両方が越せるか判定
                    最初に越せた組 -> 中継点を送る -> 着いたら元の目的地を送る [attemptsを1つ消費]
      どれも越せない -> 後退せずに諦める("fail")
    送った目的地がまた中止(6)になったら、attemptsが残っていれば最初からやり直す(新しい迂回の処理は始めない=再帰しない)。

機械は経路そのものを持たない(計画・シミュレーション・送信は呼び出し側=ルーターが行い、結果の真偽だけをイベントで返す)。
イベントはsid(start()のたびに増える)を付ける。古いsidのイベント(取り消し後に届いた計画結果など)は無視する。
アクション(タプル):
    ("plan", tag, target, idx)   tag: "direct"(機体から元の目的地) / "wp_a"(機体から中継点idx) / "wp_b"(中継点idxから元の目的地)
    ("candidates",)              中継点の候補を作って candidates() で返す
    ("check", kind, idx)         kind: "direct" / "wp"。シミュレーションして verdict() で返す
    ("send", target, idx)        target: "goal"(元の目的地) / "wp"(中継点idx)。必要なら経路の向きへ旋回してから送る
    ("fail", reason)             諦める。reasonは no_candidates / no_clear_route / attempts_exhausted / timeout など
    ("done",)                    元の目的地に着いた
    ("cancelled", reason)        外から取り消された
"""
from __future__ import annotations

from typing import List, Optional, Tuple

IDLE = "idle"
SETTLE = "settle"
PLAN_DIRECT = "plan_direct"
CHECK_DIRECT = "check_direct"
NEED_CAND = "need_candidates"
PLAN_WP_A = "plan_wp_a"
PLAN_WP_B = "plan_wp_b"
CHECK_WP = "check_wp"
SEND_DIRECT = "send_direct"
SEND_WP = "send_wp"
SEND_GOAL_AFTER_WP = "send_goal_after_wp"

SEND_STATES = (SEND_DIRECT, SEND_WP, SEND_GOAL_AFTER_WP)
SUCCEEDED, CANCELED = 4, 5   # action_msgs/GoalStatus

Action = Tuple


class DetourMachine:
    def __init__(self, max_attempts: int = 2, settle_sec: float = 1.0, plan_timeout_sec: float = 5.0,
                 total_timeout_sec: float = 60.0) -> None:
        self.max_attempts = max(1, int(max_attempts))
        self.settle_sec = max(0.0, float(settle_sec))
        self.plan_timeout_sec = max(0.5, float(plan_timeout_sec))
        self.total_timeout_sec = max(1.0, float(total_timeout_sec))
        self.state = IDLE
        self.sid = 0
        self.attempts = 0
        self.goal_xy: Optional[Tuple[float, float]] = None
        self.n_cand = 0
        self.idx = 0
        self.rejected = 0           # 実行中にstart()された回数(二重実行はしない)
        self.last_reason = ""
        self._deadline = 0.0
        self._t_start = 0.0

    @property
    def active(self) -> bool:
        return self.state != IDLE

    @property
    def sending(self) -> bool:
        return self.state in SEND_STATES

    # ---- 入口 ----
    def start(self, goal_xy: Tuple[float, float], now: float) -> List[Action]:
        if self.active:
            self.rejected += 1
            return []
        self.sid += 1
        self.attempts = 0
        self.goal_xy = (float(goal_xy[0]), float(goal_xy[1]))
        self.n_cand = 0
        self.idx = 0
        self.last_reason = ""
        self._t_start = now
        self._enter_settle(now)
        return []

    def cancel(self, reason: str = "cancel") -> List[Action]:
        if not self.active:
            return []
        self.state = IDLE
        self.last_reason = reason
        return [("cancelled", reason)]

    def tick(self, now: float) -> List[Action]:
        """時間で進める(待ち終わり・各ステップの期限・全体の期限)。"""
        if not self.active:
            return []
        if self.state not in SEND_STATES and now - self._t_start > self.total_timeout_sec:
            return self._fail("timeout")
        if self.state == SETTLE:
            if now >= self._deadline:
                self.state = PLAN_DIRECT
                self._deadline = now + self.plan_timeout_sec
                return [("plan", "direct", "goal", 0)]
            return []
        if self.state in (PLAN_DIRECT, PLAN_WP_A, PLAN_WP_B, NEED_CAND, CHECK_DIRECT, CHECK_WP) and now >= self._deadline:
            # 計画やシミュレーションの返事が来ない -> その計画は失敗として扱う
            return self._step_failed(now, "step_timeout")
        return []

    # ---- 結果のイベント ----
    def plan_result(self, sid: int, tag: str, ok: bool, now: float) -> List[Action]:
        if sid != self.sid:
            return []
        expect = {PLAN_DIRECT: "direct", PLAN_WP_A: "wp_a", PLAN_WP_B: "wp_b"}.get(self.state)
        if expect is None or tag != expect:
            return []
        if not ok:
            return self._step_failed(now, "plan_failed")
        if self.state == PLAN_DIRECT:
            self.state = CHECK_DIRECT
            self._deadline = now + self.plan_timeout_sec
            return [("check", "direct", 0)]
        if self.state == PLAN_WP_A:
            self.state = PLAN_WP_B
            self._deadline = now + self.plan_timeout_sec
            return [("plan", "wp_b", "goal", self.idx)]
        self.state = CHECK_WP
        self._deadline = now + self.plan_timeout_sec
        return [("check", "wp", self.idx)]

    def candidates(self, sid: int, count: int, now: float) -> List[Action]:
        if sid != self.sid or self.state != NEED_CAND:
            return []
        if count <= 0:
            return self._fail("no_candidates")
        self.n_cand = int(count)
        self.idx = 0
        return self._plan_wp(now)

    def verdict(self, sid: int, clear: bool, now: float, reason: str = "") -> List[Action]:
        if sid != self.sid or self.state not in (CHECK_DIRECT, CHECK_WP):
            return []
        if not clear:
            return self._step_failed(now, reason or "collision")
        self.attempts += 1
        if self.state == CHECK_DIRECT:
            self.state = SEND_DIRECT
            return [("send", "goal", 0)]
        self.state = SEND_WP
        return [("send", "wp", self.idx)]

    def send_failed(self, sid: int, reason: str) -> List[Action]:
        """送れなかった(旋回の許可が無い・サーバーに拒否された等)。後退せず諦める。"""
        if sid != self.sid or self.state not in SEND_STATES:
            return []
        return self._fail(reason or "send_failed")

    def goal_done(self, sid: int, status: int, now: float) -> List[Action]:
        """送った目的地の結果(action_msgs/GoalStatus)。"""
        if sid != self.sid or self.state not in SEND_STATES:
            return []
        if status == CANCELED:
            self.state = IDLE
            self.last_reason = "goal_canceled"
            return [("cancelled", "goal_canceled")]
        if status == SUCCEEDED:
            if self.state == SEND_WP:
                self.state = SEND_GOAL_AFTER_WP
                return [("send", "goal", 0)]
            self.state = IDLE
            return [("done",)]
        # 中止など。新しい迂回の処理は始めず(再帰しない)、attemptsが残っていればこの処理の中でやり直す
        if self.attempts >= self.max_attempts:
            return self._fail("attempts_exhausted")
        self.n_cand = 0
        self.idx = 0
        self._enter_settle(now)
        return []

    # ---- 内部 ----
    def _enter_settle(self, now: float) -> None:
        self.state = SETTLE
        self._deadline = now + self.settle_sec

    def _plan_wp(self, now: float) -> List[Action]:
        self.state = PLAN_WP_A
        self._deadline = now + self.plan_timeout_sec
        return [("plan", "wp_a", "wp", self.idx)]

    def _step_failed(self, now: float, reason: str) -> List[Action]:
        """今の候補(直接/中継点idx)は使えない。次の候補へ。無ければ諦める。"""
        self.last_reason = reason
        if self.state in (PLAN_DIRECT, CHECK_DIRECT):
            self.state = NEED_CAND
            self._deadline = now + self.plan_timeout_sec
            return [("candidates",)]
        if self.state == NEED_CAND:
            return self._fail("no_candidates")
        self.idx += 1
        if self.idx >= self.n_cand:
            return self._fail("no_clear_route")
        return self._plan_wp(now)

    def _fail(self, reason: str) -> List[Action]:
        self.state = IDLE
        self.last_reason = reason
        return [("fail", reason)]
