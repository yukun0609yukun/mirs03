from __future__ import annotations

import threading
import time
import urllib.request
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .json_tools import dumps, loads

# 【2026-10-09追加】ラズパイ腹部画面のアンケートに誰かが回答している間、ロボットを止めるための
# 橋渡しノード。ラズパイはROSノードを持たないので、こちら(1台目)から
# GET <survey_url>/state をポーリングして /nokosahen/survey_active (Bool) に変換する。
# 受け取る側: behavior_director(SURVEY_ACTIVE_HOLD)と nav2_goal_router(ゴール拒否+取り消し)。
# 詳細・安全側の判断は docs/survey_robot_hold.md。


def fetch_survey_state(url: str, timeout: float = 0.8) -> Dict[str, Any]:
    """`<url>/state` を取得して検証した結果を返す。失敗は {"ok": False, "error": 理由}。

    契約: {"mode":"survey"|"customer","active":bool,"count":int,
           "last_activity_age_s":float|null,"condition":str|null,"updated":epoch秒,
           "session_state":"idle"|"answering"|"saved"|"abandoned"(2026-10-09追加。古いラズパイには無い),
           "saved_seq":int(保存完了の単調増加カウンタ。同上)}
    activeがboolでない/JSONが壊れている場合は「不明」(ok=False)として扱う。
    """
    target = url.rstrip("/") + "/state"
    try:
        with urllib.request.urlopen(target, timeout=timeout) as resp:  # noqa: S310
            body = resp.read(65536)
    except Exception as exc:  # noqa: BLE001  通信エラー/タイムアウト/HTTPエラー全般
        if hasattr(exc, "close"):
            exc.close()  # HTTPErrorのレスポンスを閉じる
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}"[:200]}
    data = loads(body.decode("utf-8", "replace"), None)
    if not isinstance(data, dict) or not isinstance(data.get("active"), bool):
        return {"ok": False, "error": "invalid_json"}
    mode = data.get("mode")
    return {
        "ok": True,
        "active": data["active"],
        "mode": mode if isinstance(mode, str) else None,
        "count": data.get("count") if isinstance(data.get("count"), int) else None,
        "age": data.get("last_activity_age_s")
        if isinstance(data.get("last_activity_age_s"), (int, float))
        else None,
        "condition": data.get("condition") if isinstance(data.get("condition"), str) else None,
        "session_state": data.get("session_state") if isinstance(data.get("session_state"), str) else None,
        "saved_seq": data.get("saved_seq")
        if isinstance(data.get("saved_seq"), int) and not isinstance(data.get("saved_seq"), bool)
        else None,
    }


class SurveyGate:
    """ラズパイの応答列から、ロボットに渡す survey_active を決める(時刻は呼び出し側が渡す)。

    ・active=true            → true(session_state=="answering" の間。回答が保存されるまで続く)
    ・active=false           → falseになってから debounce_sec 経つまでtrueを保つ(直後の急発進防止)
    ・保存完了を検知          → session_state=="saved" か saved_seq の増加を見たら、その時点から
                               post_save_hold_sec だけtrueを保つ(回答直後に急発進しない。debounceと長い方)
    ・放棄(abandoned)         → ラズパイ側が放棄タイムアウト(既定120秒)でfalseにする。ここは普通のfalse扱い
    ・到達不能/エラー/不正JSON → 直前がtrueなら最後にtrueを確認してから unreachable_hold_sec は
                               trueを保つ(一瞬の通信途切れで動き出さない)。超えたらfalse
                               (ラズパイ停止/ケーブル抜けで永久に止まり続けない)。
                               直前が回答中(answering)だった場合は answering_unreachable_hold_sec(長め)
    ・session_state が無い(古いラズパイ) → 従来どおり(active/debounce/unreachable_hold_sec だけ)
    ・mode=="customer"       → 常にfalse(アンケート画面ではない)
    ・一度も応答が無い間      → false(従来どおり)
    """

    def __init__(
        self,
        debounce_sec: float = 1.5,
        unreachable_hold_sec: float = 20.0,
        answering_unreachable_hold_sec: float = 60.0,
        post_save_hold_sec: float = 1.5,
    ) -> None:
        self.debounce_sec = max(0.0, float(debounce_sec))
        self.unreachable_hold_sec = max(0.0, float(unreachable_hold_sec))
        self.answering_unreachable_hold_sec = max(0.0, float(answering_unreachable_hold_sec))
        self.post_save_hold_sec = max(0.0, float(post_save_hold_sec))
        self.out = False
        self.reachable = False
        self.last_error: Optional[str] = "no_response_yet"
        self.sample: Dict[str, Any] = {}
        self._false_since: Optional[float] = None
        self._last_true: Optional[float] = None
        self._last_answering = False  # 最後にtrueを確認した時の session_state が answering か
        self._seq_seen: Optional[int] = None  # 観測済みの saved_seq(最初の値は基準にするだけ)
        self._hold_seq: Optional[int] = None  # 保存後ホールドを始めた saved_seq
        self._post_save_until: Optional[float] = None

    def feed(self, sample: Dict[str, Any]) -> None:
        self.reachable = bool(sample.get("ok"))
        self.last_error = None if self.reachable else str(sample.get("error") or "unknown")
        if self.reachable:
            self.sample = sample

    def tick(self, now: float) -> bool:
        s = self.sample
        if self.reachable and s.get("mode") == "customer":
            self.out = False
            self._false_since = None
            self._post_save_until = None
            return False
        if self.reachable:
            self._observe_save(s, now)
        if self.reachable and s.get("active") is True:
            self.out = True
            self._false_since = None
            self._last_true = now
            self._last_answering = s.get("session_state") == "answering"
            return True
        if not self.out:
            return False
        if self.reachable or self._false_since is not None:
            # 直近の応答がfalse(到達不能中でも、falseを確認済みならデバウンスで解除)
            if self._false_since is None:
                self._false_since = now
            saving = self._post_save_until is not None and now < self._post_save_until
            if now - self._false_since >= self.debounce_sec and not saving:
                self.out = False
                self._false_since = None
        else:
            hold = self.answering_unreachable_hold_sec if self._last_answering else self.unreachable_hold_sec
            if self._last_true is None or now - self._last_true > hold:
                self.out = False
        return self.out

    def _observe_save(self, s: Dict[str, Any], now: float) -> None:
        """保存完了(session_state=="saved" か saved_seq の増加)を初めて見た時刻から post_save_hold_sec を数える"""
        seq = s.get("saved_seq")
        grew = isinstance(seq, int) and self._seq_seen is not None and seq > self._seq_seen
        if isinstance(seq, int):
            self._seq_seen = seq if self._seq_seen is None else max(self._seq_seen, seq)
        if (grew or s.get("session_state") == "saved") and (seq is None or seq != self._hold_seq):
            self._hold_seq = seq
            self._post_save_until = now + self.post_save_hold_sec


class SurveyMonitor(Node):
    def __init__(self) -> None:
        super().__init__("survey_monitor")
        self.declare_parameter("survey_url", "http://198.51.100.1:8082")
        self.declare_parameter("poll_hz", 1.5)
        self.declare_parameter("http_timeout_sec", 0.8)
        self.declare_parameter("debounce_sec", 1.5)
        self.declare_parameter("unreachable_hold_sec", 20.0)
        self.declare_parameter("answering_unreachable_hold_sec", 60.0)
        self.declare_parameter("post_save_hold_sec", 1.5)
        self.declare_parameter("publish_hz", 5.0)
        self._url = str(self.get_parameter("survey_url").value or "").strip()
        poll_hz = min(5.0, max(0.2, float(self.get_parameter("poll_hz").value)))
        self._timeout = max(0.1, float(self.get_parameter("http_timeout_sec").value))
        self._gate = SurveyGate(
            float(self.get_parameter("debounce_sec").value),
            float(self.get_parameter("unreachable_hold_sec").value),
            float(self.get_parameter("answering_unreachable_hold_sec").value),
            float(self.get_parameter("post_save_hold_sec").value),
        )
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._active_pub = self.create_publisher(Bool, "/nokosahen/survey_active", qos)
        self._status_pub = self.create_publisher(String, "/nokosahen/survey_status", qos)
        self._lock = threading.Lock()
        self._latest: Optional[Dict[str, Any]] = None
        self._stop = threading.Event()
        self._poll_period = 1.0 / poll_hz
        if self._url:
            threading.Thread(target=self._poll_loop, name="survey_poll", daemon=True).start()
            self.get_logger().info(f"アンケート状態を監視: {self._url}/state ({poll_hz:.1f}Hz)")
        else:
            self._gate.last_error = "disabled"
            self.get_logger().info("survey_urlが空のためアンケート監視は無効(常にfalse)")
        self._publish_active()  # 起動直後に1回(以降はtimerで継続)
        self.create_timer(
            1.0 / max(1.0, float(self.get_parameter("publish_hz").value)), self._publish_active
        )
        self.create_timer(1.0, self._publish_status)

    def _poll_loop(self) -> None:
        while not self._stop.is_set():
            sample = fetch_survey_state(self._url, self._timeout)
            with self._lock:
                self._latest = sample
            self._stop.wait(self._poll_period)

    def _publish_active(self) -> None:
        with self._lock:
            sample, self._latest = self._latest, None
        if sample is not None:
            self._gate.feed(sample)
        msg = Bool()
        msg.data = bool(self._gate.tick(time.monotonic()))
        self._active_pub.publish(msg)

    def _publish_status(self) -> None:
        g, s = self._gate, self._gate.sample
        msg = String()
        msg.data = dumps(
            {
                "active": g.out,
                "raw_active": s.get("active"),
                "mode": s.get("mode"),
                "count": s.get("count"),
                "age": s.get("age"),
                "condition": s.get("condition"),
                "session_state": s.get("session_state"),
                "saved_seq": s.get("saved_seq"),
                "reachable": g.reachable,
                "last_error": g.last_error,
                "enabled": bool(self._url),
            }
        )
        self._status_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._stop.set()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SurveyMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
