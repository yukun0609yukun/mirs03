"""動きの調停ノード(2026-10-22、docs/motion_priority.md)。

速度指令(`/cmd_vel_nav`)を出す仕組みが増えた(Nav2のコントローラ、ルーターの先回し旋回(Spin・代替の回頭)、wall_escape)。
それぞれが「自分が動いてよいか」を別々に判断すると、同時に出して混ざる。そこで、**動いてよいかの判断を、このノード1つに集める**。

- 入力: 緊急停止(ハード・ソフト)、手動運転(WASD)、近接停止、ルーターの回頭中、Nav2の経路(/plan)、wall_escapeの状態
- 出力: `/nokosahen/motion_grant`(String、JSON): {"owner": 今の動きの持ち主, "allow": {"wall_escape": bool, "router_rotate": bool, "director_direct": bool}, "stamp": 時刻}
  を10Hzで配信する。wall_escape・ルーターの代替の回頭・behavior_directorの直接/cmd_vel(見回しの旋回と直進・アイドル揺動・直接接近)は、
  許可(allow)が出ているときだけ速度指令を出す。

優先順位(上ほど強い): estop > manual(WASD) > near_person > nav2(経路を追従中) > router_rotate > wall_escape > idle
- estop・manual・near_person・nav2 のとき: 3つとも許可しない
- router_rotate(先回し旋回中)のとき: 代替の回頭だけ許可。ほかは許可しない
- wall_escape(離脱中)のとき: wall_escapeだけ許可。ほかは許可しない
- idle(誰も動かしていない)のとき: 全部に許可(先に動き出した方が持ち主になり、次の周期で相手が止まる)
  director_directは idle のときだけ許可(directorの直接出力は、ほかの持ち主が動かしている間は出さない)

person_clear_gateの見張り(2026-10-24): gateは壁際の停止領域(VelocityStop)を無効にするノードで、状態を1Hzで`/nokosahen/person_clear_gate/status`に出す。
relaxed=true(無効にしている)を最後に受けてから3秒以上更新が無ければ、gateが落ちた/固まったと見なし、`/collision_monitor/set_parameters`で
VelocityStop.enabled=trueを送って戻す(フェイルセーフ)。このノードがすることはその操作だけ。

期限(2026-10-24): router_rotating・Nav2の回復動作(BackUp・Spin)の「動いている」は、立ててから15秒を超えたら無効と見なす
(相手のノードが動作中に落ちると、最後の値が張り付き、許可が出なくなるため)。経路(/plan)の鮮度は plan_stale_sec(3.0秒)。

このノードが落ちた/まだ起動していないとき: 受け取り側は、一度も受信していなければ従来どおり(許可)、受信が途絶えたら(1秒)不許可(安全側)。
このノードは速度指令を出さない(許可を出すだけ)。
"""
from __future__ import annotations

import json
import time

from .pose_relay import ROBOT_POSE_TOPIC, PoseCache
from typing import Dict, List, Optional, Tuple

try:  # ROSなしの単体テストでも、判定の部分だけを読み込めるようにする
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from nav_msgs.msg import Path
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object
    Path = None
    GoalStatusArray = None
    LaserScan = None
try:
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
except ImportError:  # pragma: no cover
    SetParameters = None
try:
    from nav2_msgs.msg import CollisionMonitorState
except ImportError:  # pragma: no cover
    CollisionMonitorState = None
    Buffer = None

OWNERS = ("estop", "manual", "near_person", "near_person_face_turn", "nav2", "router_rotate", "wall_escape", "idle")
FACE_TURN_FRESH_SEC = 1.0  # directorの「その場の旋回だけ」の通知がこの時間より古ければ無効(directorが落ちたら近接停止に戻る)
from .near_person_watch import NEAR_PERSON_STALE_SEC, NearPersonWatch  # noqa: E402

HELPERS = ("wall_escape", "router_rotate", "director_direct")


class Arbiter:
    """時刻を引数で受け取る、判定だけのクラス(単体テスト用)。

    recovery_active・router_rotatingは、それぞれ立てた時刻(false→trueになった時刻)を持ち、flag_expire_sec(15秒)を超えたら無効(false)と見なす
    (2026-10-24: BackUp中・回頭中に相手のノードが落ちると、最後の値が張り付いて、許可が出ない状態が続いた)。
    set_router_rotating()/set_recovery()で立てたものだけが期限の対象。属性を直接代入した場合は時刻が無く、期限なし(従来どおり)。"""

    def __init__(self, plan_stale_sec: float = 3.0, flag_expire_sec: float = 15.0,
                 near_person_stale_sec: float = NEAR_PERSON_STALE_SEC) -> None:
        self.plan_stale_sec = plan_stale_sec
        # 【2026-11-04】near_person_stop の受信が一度あったのに途絶えたら、近接停止と同じ扱い(一度も受信していなければ適用しない。0以下で無効)
        self._near_watch = NearPersonWatch(near_person_stale_sec)
        self.flag_expire_sec = flag_expire_sec
        self.router_rotating_since: Optional[float] = None
        self.recovery_since: Dict[str, float] = {}
        self.hard_estop = False
        self.soft_estop = False
        self.wasd = False
        self.near_person = False
        self.face_turn_time: Optional[float] = None  # directorが「向く(その場の旋回だけ)」を出している通知を最後に受けた時刻(trueのときだけ更新)
        self.router_rotating = False
        self.wall_escape_escaping = False
        self.plan_time: Optional[float] = None   # 2点以上の経路を最後に受け取った時刻
        # Nav2の回復動作(BackUp・Spin)が実行中か(アクションの状態が ACCEPTED/EXECUTING/CANCELING)。経路が途切れる間も、Nav2が動かしている
        self.recovery_active: Dict[str, bool] = {}

    def set_near_person(self, now: float, value: bool) -> None:
        """/nokosahen/near_person_stop の受信。値と受信時刻を覚える。"""
        self.near_person = bool(value)
        self._near_watch.on_message(now, value)

    def near_person_stale(self, now: float) -> bool:
        return self._near_watch.is_stale(now)

    def set_face_turn(self, now: float, value: bool) -> None:
        """directorの /nokosahen/face_inplace_turn。trueを受けた時刻を覚え、falseなら消す。"""
        self.face_turn_time = now if value else None

    def face_turn_valid(self, now: float) -> bool:
        return self.face_turn_time is not None and (now - self.face_turn_time) <= FACE_TURN_FRESH_SEC

    def plan_seen(self, now: float) -> None:
        self.plan_time = now

    def set_router_rotating(self, now: float, value: bool) -> None:
        value = bool(value)
        if value and not self.router_rotating:
            self.router_rotating_since = now
        elif not value:
            self.router_rotating_since = None
        self.router_rotating = value

    def set_recovery(self, name: str, now: float, active: bool) -> None:
        active = bool(active)
        if active and not self.recovery_active.get(name):
            self.recovery_since[name] = now
        elif not active:
            self.recovery_since.pop(name, None)
        self.recovery_active[name] = active

    def _expired(self, since: Optional[float], now: float) -> bool:
        return since is not None and now - since > self.flag_expire_sec

    def router_rotating_valid(self, now: float) -> bool:
        return bool(self.router_rotating) and not self._expired(self.router_rotating_since, now)

    def recovery_valid(self, now: float) -> bool:
        return any(a and not self._expired(self.recovery_since.get(n), now) for n, a in self.recovery_active.items())

    def expired_flags(self, now: float) -> List[str]:
        """期限切れで無効扱いになっているフラグの名前(ログ用)。"""
        out = []
        if self.router_rotating and self._expired(self.router_rotating_since, now):
            out.append("router_rotating")
        out += [f"recovery:{n}" for n, a in self.recovery_active.items() if a and self._expired(self.recovery_since.get(n), now)]
        return out

    def nav2_active(self, now: float) -> bool:
        if self.recovery_valid(now):
            return True
        return self.plan_time is not None and now - self.plan_time <= self.plan_stale_sec

    def decide(self, now: float) -> Tuple[str, Dict[str, bool]]:
        if self.hard_estop or self.soft_estop:
            return "estop", {h: False for h in HELPERS}
        if self.wasd:
            return "manual", {h: False for h in HELPERS}
        if self.near_person or self.near_person_stale(now):
            # 【2026-11-04】directorが「向く(その場の旋回だけ。前へ進まない)」を出している間は、近接停止でも、directorの直接指令(旋回)だけ許可する。
            # すぐ前にいる人の方も向けるように。ほかの動き(Nav2・wall_escape・先回し旋回)は、これまでどおり止める。
            if self.face_turn_valid(now):
                return "near_person_face_turn", {"wall_escape": False, "router_rotate": False, "director_direct": True}
            return "near_person", {h: False for h in HELPERS}
        # 先回し旋回中は、Nav2はまだ走らせていない(旋回のあとに目的地を送る)ので、nav2より先に判定する
        if self.router_rotating_valid(now):
            return "router_rotate", {"wall_escape": False, "router_rotate": True, "director_direct": False}
        if self.nav2_active(now):
            return "nav2", {h: False for h in HELPERS}
        if self.wall_escape_escaping:
            return "wall_escape", {"wall_escape": True, "router_rotate": False, "director_direct": False}
        return "idle", {h: True for h in HELPERS}


class RelaxWatchdog:
    """person_clear_gateの状態(1Hz)の見張り。時刻を引数で受け取る、判定だけのクラス(単体テスト用)。

    relaxed=true(gateが停止領域を無効にしている)を最後に受けてから stale_sec 秒以上、更新が無い間は、
    should_restore()がTrueを返す(resend_sec秒おき)。relaxed=falseなら何も要求しない。新しい状態を受けたら仕切り直す。"""

    def __init__(self, stale_sec: float = 3.0, resend_sec: float = 5.0) -> None:
        self.stale_sec = stale_sec
        self.resend_sec = resend_sec
        self.relaxed = False
        self.last_time: Optional[float] = None
        self._last_restore: Optional[float] = None

    def on_status(self, now: float, relaxed: bool) -> None:
        self.relaxed = bool(relaxed)
        self.last_time = now
        self._last_restore = None

    def retry_now(self) -> None:
        """要求を送れなかったとき: 待たずに次の周期で再要求させる。"""
        self._last_restore = None

    def should_restore(self, now: float) -> bool:
        if not self.relaxed or self.last_time is None:
            return False
        if now - self.last_time < self.stale_sec:
            return False
        if self._last_restore is not None and now - self._last_restore < self.resend_sec:
            return False
        self._last_restore = now
        return True


class MotionArbiter(Node):  # type: ignore[misc]
    def __init__(self) -> None:
        super().__init__("motion_arbiter")
        self.declare_parameter("plan_stale_sec", 3.0)  # 【2026-10-24】2.0 -> 3.0
        self.declare_parameter("near_person_stale_sec", NEAR_PERSON_STALE_SEC)  # 【2026-11-04】0以下で無効
        self._arb = Arbiter(float(self.get_parameter("plan_stale_sec").value),
                            near_person_stale_sec=float(self.get_parameter("near_person_stale_sec").value))
        self._pub = self.create_publisher(String, "/nokosahen/motion_grant", 5)
        self._last_owner: Optional[str] = None
        self._last_expired: List[str] = []
        self.create_subscription(Bool, "/nokosahen/wasd_active", lambda m: setattr(self._arb, "wasd", bool(m.data)), 10)
        self.create_subscription(Bool, "/nokosahen/soft_estop", lambda m: setattr(self._arb, "soft_estop", bool(m.data)), 10)
        self.create_subscription(Bool, "/estop_state", lambda m: setattr(self._arb, "hard_estop", bool(m.data)), 10)
        self.create_subscription(Bool, "/nokosahen/near_person_stop", lambda m: self._arb.set_near_person(time.monotonic(), bool(m.data)), 10)
        self.create_subscription(Bool, "/nokosahen/face_inplace_turn", lambda m: self._arb.set_face_turn(time.monotonic(), bool(m.data)), 10)
        self.create_subscription(Bool, "/nokosahen/router_rotating", lambda m: self._arb.set_router_rotating(time.monotonic(), bool(m.data)), 10)
        self.create_subscription(String, "/nokosahen/wall_escape/status", self._on_wall_status, 5)
        if Path is not None:
            self.create_subscription(Path, "/plan", self._on_plan, 5)
        if GoalStatusArray is not None:
            for name in ("backup", "spin"):
                self.create_subscription(GoalStatusArray, f"/{name}/_action/status",
                                         lambda m, n=name: self._on_recovery_status(n, m), 5)
        # 【2026-10-23】後退(BTの回復動作 BackUp)が始まった瞬間の状況を1行で記録する(「何回か行き来したら急に後退が増えた」原因の調査用)。
        # スキャンは最新の1件を覚えるだけ(処理は後退の開始時だけ)。
        self._last_scan = None
        self._cm_state = None
        self._pose_cache = None
        if LaserScan is not None:
            self.create_subscription(LaserScan, "/scan", lambda m: setattr(self, "_last_scan", m), 2)
        if CollisionMonitorState is not None:
            self.create_subscription(CollisionMonitorState, "/collision_monitor_state", lambda m: setattr(self, "_cm_state", (m.action_type, m.polygon_name, time.monotonic())), 5)
        # 【2026-10-26】map→odomの古さは、/tfを直接購読せず、pose_relay(5Hz)の /nokosahen/robot_pose のstampから出す(CPU削減)
        self._pose_cache = PoseCache()
        self.create_subscription(String, ROBOT_POSE_TOPIC, lambda m: self._pose_cache.update(m.data), 2)
        self._backup_prev = False
        # person_clear_gateの見張り(2026-10-24)。速度指令は出さず、VelocityStopを有効に戻す依頼だけを出す
        self._relax_dog = RelaxWatchdog()
        self._cm_cli = self.create_client(SetParameters, "/collision_monitor/set_parameters") if SetParameters is not None else None
        self.create_subscription(String, "/nokosahen/person_clear_gate/status", self._on_gate_status, 5)
        self.create_timer(0.1, self._tick)
        self.get_logger().info("motion_arbiter: 動いてよいかの判定を集約します(docs/motion_priority.md)")

    def _on_plan(self, msg) -> None:
        if len(getattr(msg, "poses", [])) >= 2:
            self._arb.plan_seen(time.monotonic())

    def _on_recovery_status(self, name: str, msg) -> None:
        try:
            active = any(int(s.status) in (1, 2, 3) for s in msg.status_list)
            self._arb.set_recovery(name, time.monotonic(), active)
            if name == "backup":
                if active and not self._backup_prev:
                    self._log_backup_snapshot()
                self._backup_prev = active
        except (TypeError, ValueError, AttributeError):
            pass

    def _log_backup_snapshot(self) -> None:
        """後退が始まった時の状況: LiDARの前・左・右・後ろの最短距離、map→odomの座標変換の古さ、衝突監視の状態、負荷。"""
        try:
            import math
            parts = []
            s = self._last_scan
            if s is not None:
                sect = {"前": 9.9, "左": 9.9, "右": 9.9, "後": 9.9}
                for i, r in enumerate(s.ranges):
                    if not math.isfinite(r) or r < max(s.range_min, 0.05):
                        continue
                    a = math.degrees(s.angle_min + i * s.angle_increment) % 360.0
                    k = "前" if (a < 45 or a >= 315) else "左" if a < 135 else "後" if a < 225 else "右"
                    sect[k] = min(sect[k], r)
                parts.append("LiDAR最短[m] " + " ".join(f"{k}{v:.2f}" for k, v in sect.items()))
            if self._pose_cache is not None:
                age = self._pose_cache.map_odom_age(self.get_clock().now().nanoseconds / 1e9)
                if age is not None:
                    parts.append(f"map→odomの古さ {age:.2f}秒")
                else:
                    parts.append("map→odom 取得不可")
            if self._cm_state is not None:
                act, poly, ts = self._cm_state
                parts.append(f"衝突監視 action={act} polygon={poly}({time.monotonic() - ts:.1f}秒前)")
            try:
                with open("/proc/loadavg") as f:
                    parts.append("load " + f.read().split()[0])
            except OSError:
                pass
            self.get_logger().info("【後退の記録】" + " / ".join(parts))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"後退の記録に失敗: {exc}")

    def _on_wall_status(self, msg) -> None:
        try:
            self._arb.wall_escape_escaping = json.loads(msg.data).get("state") == "escaping"
        except (ValueError, TypeError, AttributeError):
            pass

    def _on_gate_status(self, msg) -> None:
        try:
            self._relax_dog.on_status(time.monotonic(), bool(json.loads(msg.data).get("relaxed", False)))
        except (ValueError, TypeError, AttributeError):
            pass

    def _check_gate(self) -> None:
        if not self._relax_dog.should_restore(time.monotonic()):
            return
        if self._cm_cli is None or not self._cm_cli.service_is_ready():
            self._relax_dog.retry_now()  # 送れなかった。次の周期で再試行
            return
        req = SetParameters.Request()
        p = Parameter()
        p.name = "VelocityStop.enabled"
        p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)
        req.parameters = [p]
        self._cm_cli.call_async(req)
        self.get_logger().warn("person_clear_gateの状態が途絶えました。壁際の停止領域(VelocityStop)を有効に戻します")

    def _tick(self) -> None:
        try:
            self._check_gate()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"gateの見張りでエラー: {exc}")
        now = time.monotonic()
        expired = self._arb.expired_flags(now)
        if expired != self._last_expired:
            self._last_expired = expired
            if expired:
                self.get_logger().warn(f"期限(15秒)を超えたため無効と見なします: {', '.join(expired)}(相手のノードが止まった可能性)")
        if self._arb._near_watch.should_warn(now):
            self.get_logger().warn("near_person_stop の受信が途絶えました(person_target_detectorが止まった可能性)。安全側で近接停止と同じ扱いにします")
        owner, allow = self._arb.decide(now)
        if owner != self._last_owner:
            self.get_logger().info(f"動きの持ち主: {self._last_owner} -> {owner}")
            self._last_owner = owner
        msg = String()
        msg.data = json.dumps({"owner": owner, "allow": allow, "stamp": time.time()})
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MotionArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
