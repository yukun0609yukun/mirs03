"""カメラで人がいないと確認できている間は、壁が近くても止まらずに進む(2026-10-22、あなたの指示)。

衝突監視(Nav2 collision_monitor)の速度別の停止領域(VelocityStop)は、壁・人を区別せず、機体の近くに点が3つ以上あると止める。
前面カメラと後ろ向き2台のすべてが「人はいない」と確認できているとき(新しい結果で、人0人)だけ、このVelocityStopを無効にする。
- FootprintApproach(1.2秒先の衝突を予測して減速する領域)は、有効のまま残す(壁に突っ込まないための最後の防御)
- 人が映った・カメラの結果が古い・LiDARの近接停止がtrue・目標の人物がいる・緊急停止、のどれかで、すぐ有効に戻す
- 「人なし」が続いてclear_hold_sec秒たってから無効にする(一瞬の見逃しで外さない)
カメラの死角(真横の約60〜105°と真後ろの約±30°)がある。そこの人は、LiDARの近接停止(near_person_stop)が頼り。
"""
from __future__ import annotations

import json
import signal
import threading
import time
from typing import Dict, Optional

try:  # ROSなしの単体テストでも、純粋な部分だけを読み込めるようにする
    import rclpy
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object
try:  # rclpyのSIGINT/SIGTERMの自動処理を切るため(終了時の復元を、rclpyが生きているうちに行う)
    from rclpy.signals import SignalHandlerOptions
except ImportError:  # pragma: no cover
    SignalHandlerOptions = None

# 【2026-10-24】ゲートの状態(1Hz)。motion_arbiterがこれを見て、relaxed=trueのまま途絶えたらVelocityStopを有効に戻す(フェイルセーフ)。
# 管理画面にも出せるよう、relaxed(今、停止領域を外す要求を出している=true)を含める。
STATUS_TOPIC = "/nokosahen/person_clear_gate/status"


def build_status(relaxed: bool, stamp: float) -> str:
    """状態のJSON文字列: {"relaxed": bool, "stamp": epoch秒}。"""
    return json.dumps({"relaxed": bool(relaxed), "stamp": float(stamp)})


class ClearGate:
    """時刻を引数で受け取る、判定だけのクラス(単体テスト用)。"""

    def __init__(self, clear_hold_sec: float = 1.0, stale_sec: float = 1.5) -> None:
        self.clear_hold_sec = clear_hold_sec
        self.stale_sec = stale_sec
        self.front: Optional[tuple] = None        # (時刻, 人数)
        self.rear: Optional[tuple] = None         # (時刻, {カメラ名: {"ok":bool,"count":int}})
        self.near_person_stop = False
        self.target_seen_time: Optional[float] = None
        self.estop = False
        self._clear_since: Optional[float] = None

    def set_front(self, now: float, count: int) -> None:
        self.front = (now, int(count))

    def set_rear(self, now: float, cams: Dict[str, dict]) -> None:
        self.rear = (now, cams)

    def target_seen(self, now: float) -> None:
        self.target_seen_time = now

    def _all_clear(self, now: float) -> bool:
        if self.estop or self.near_person_stop:
            return False
        if self.target_seen_time is not None and now - self.target_seen_time <= self.stale_sec:
            return False
        if self.front is None or now - self.front[0] > self.stale_sec or self.front[1] != 0:
            return False
        if self.rear is None or now - self.rear[0] > self.stale_sec or not self.rear[1]:
            return False
        for info in self.rear[1].values():
            if not info.get("ok") or int(info.get("count", 1)) != 0:
                return False
        return True

    def relax(self, now: float) -> bool:
        """VelocityStopを無効にしてよいか。人の可能性があれば、すぐFalse。"""
        if not self._all_clear(now):
            self._clear_since = None
            return False
        if self._clear_since is None:
            self._clear_since = now
        return now - self._clear_since >= self.clear_hold_sec


class PersonClearGate(Node):  # type: ignore[misc]
    TARGET_NODE = "/collision_monitor"
    PARAM = "VelocityStop.enabled"

    def __init__(self) -> None:
        super().__init__("person_clear_gate")
        self.declare_parameter("enabled", True)
        self.declare_parameter("clear_hold_sec", 1.0)
        self.declare_parameter("stale_sec", 1.5)
        self._on = bool(self.get_parameter("enabled").value)
        self._gate = ClearGate(float(self.get_parameter("clear_hold_sec").value), float(self.get_parameter("stale_sec").value))
        self._applied: Optional[bool] = None   # VelocityStop.enabled に最後に設定した値(未設定ならNone)
        self._applied_time = 0.0               # 最後に設定を送った時刻(定期的に再送する)
        self._cli = self.create_client(SetParameters, f"{self.TARGET_NODE}/set_parameters")
        self._relaxed_req = False              # 最後に出した要求が「無効にする」か(状態の配信用。送ったときに更新)
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 5)
        self.create_timer(1.0, self._publish_status)
        self.create_subscription(String, "/nokosahen/camera_person_density", self._on_front, 5)
        self.create_subscription(String, "/nokosahen/rear_camera_persons", self._on_rear, 5)
        self.create_subscription(Bool, "/nokosahen/near_person_stop", lambda m: setattr(self._gate, "near_person_stop", bool(m.data)), 10)
        self.create_subscription(Bool, "/nokosahen/soft_estop", self._on_soft, 10)
        self.create_subscription(Bool, "/estop_state", self._on_hard, 10)
        self.create_subscription(String, "/nokosahen/person_target", lambda m: self._on_target(m), 10)
        self._soft = False
        self._hard = False
        self.create_timer(0.5, self._tick)  # 2026-10-23: 0.2→0.5秒(CPU削減)
        self.get_logger().info(f"person_clear_gate: enabled={self._on}")

    def _now(self) -> float:
        return time.monotonic()

    def _on_front(self, msg) -> None:
        try:
            self._gate.set_front(self._now(), int(json.loads(msg.data).get("person_count", 1)))
        except (ValueError, TypeError):
            pass

    def _on_rear(self, msg) -> None:
        try:
            cams = json.loads(msg.data).get("cams", {})
            if isinstance(cams, dict):
                self._gate.set_rear(self._now(), cams)
        except (ValueError, TypeError):
            pass

    def _on_target(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(data, dict) and data.get("status") not in (None, "none", "lost", ""):
            self._gate.target_seen(self._now())

    def _on_soft(self, msg) -> None:
        self._soft = bool(msg.data)
        self._gate.estop = self._soft or self._hard

    def _on_hard(self, msg) -> None:
        self._hard = bool(msg.data)
        self._gate.estop = self._soft or self._hard

    def _publish_status(self) -> None:
        msg = String()
        msg.data = build_status(self._relaxed_req, time.time())
        self._status_pub.publish(msg)

    def _tick(self) -> None:
        try:
            self._tick_inner()
        except Exception as exc:  # noqa: BLE001
            # 【2026-10-22】例外でゲートが止まると、無効にした停止領域が残る。必ず有効に戻す要求を出し、自分は落とさない。
            self.get_logger().error(f"person_clear_gateの処理でエラー(停止領域を有効に戻します): {exc}")
            self._applied = None
            self._send(True)

    def _tick_inner(self) -> None:
        relax = self._on and self._gate.relax(self._now())
        want_enabled = not relax  # VelocityStop.enabled: 人がいる/不明なら有効
        # 5秒おきに同じ値を再送する(collision_monitorがlifecycleで再起動してyamlの値に戻っても、無効の状態が食い違ったまま黙らない)
        if want_enabled == self._applied and self._now() - self._applied_time < 5.0:
            return
        self._send(want_enabled)

    def _send(self, want_enabled: bool) -> None:
        if not self._cli.service_is_ready():
            return None  # collision_monitorがまだ。有効のまま(安全側)
        req = SetParameters.Request()
        p = Parameter()
        p.name = self.PARAM
        p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=want_enabled)
        req.parameters = [p]
        fut = self._cli.call_async(req)
        fut.add_done_callback(lambda f, v=want_enabled: self._on_set(f, v))
        self._relaxed_req = not want_enabled
        return fut

    def restore_blocking(self, timeout_sec: float = 2.0) -> bool:
        """終了時: VelocityStop.enabled=true の依頼を同期で送り、応答を最長timeout_sec秒待つ。rclpyが生きているうちに呼ぶ。
        送れた(応答を待った)ならTrue。サービスが無い/例外ならFalse。"""
        self._on = False  # 待つ間にタイマーが「無効」を送り直さない
        try:
            fut = self._send(True)
            if fut is None:
                return False
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)
            return bool(fut.done())
        except Exception:  # noqa: BLE001
            return False

    def _on_set(self, fut, value: bool) -> None:
        try:
            res = fut.result().results[0]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"VelocityStop.enabledの設定に失敗: {exc}")
            return
        if res.successful:
            self._applied = value
            self._applied_time = self._now()
            self.get_logger().info(
                "人がいないと確認できたので、壁際の停止領域(VelocityStop)を無効にします" if not value
                else "人がいる/不明のため、壁際の停止領域(VelocityStop)を有効に戻します"
            )
        else:
            self.get_logger().warn(f"VelocityStop.enabledを設定できません: {res.reason}")


def install_stop_handlers() -> threading.Event:
    """SIGINT/SIGTERMで立つEventを返す。rclpyの自動処理(contextを先に閉じる)を切った上で、自前で受ける。"""
    ev = threading.Event()

    def _handler(signum, frame) -> None:  # noqa: ARG001
        ev.set()

    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)
    return ev


def main() -> None:
    # 【2026-10-24】rclpyは既定で、SIGINT/SIGTERMを受けるとcontextを先に閉じる。すると終了時の復元のSetParametersが送れない。
    # 自動処理を切り(SignalHandlerOptions.NO)、自分で受けて、rclpyが生きているうちに復元してから終わる。
    if SignalHandlerOptions is not None:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    else:  # pragma: no cover
        rclpy.init()
    node = PersonClearGate()
    stop = install_stop_handlers()
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了するときは、無効にしていた壁際の停止領域を有効に戻す依頼を出してから終わる(最長2秒待つ)
        node.restore_blocking(2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
