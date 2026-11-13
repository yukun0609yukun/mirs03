"""壁際からの脱出(2026-10-22)。

機体が壁に近すぎる(経路計画の出発点が「通れない」コスト帯に入る)と、Nav2の経路計画(NavFn)は出発点から経路を
作れず、目的地が前方に開けていても「Failed to create plan」で中止になり、動かなくなる。後退(BackUp)による回復も、
衝突監視が止めるため、抜け出せない。

目的地が中止(Nav2の終了状態 aborted)になり、LiDARで壁が近い(trigger_distance_m以内)ときだけ、壁から遠ざかる向きへ
低速で動いて(escape_linear_mps)、十分に離れたら、直前の目的地を1回だけ出し直す。
- 速度指令は /cmd_vel_nav に出す(速度制限→衝突監視を通る。安全層は迂回しない)
- 緊急停止(ハード/ソフト)・手動運転・近接停止・目的地の取り消しがあれば、すぐやめる
- 動く時間は最長max_escape_sec秒、目的地ごとに1回まで
"""
from __future__ import annotations

import json
import math
from typing import List, Optional, Tuple

from .escape_planner import GridInfo, find_escape_waypoint
from .motion_sim import pick_motion, to_xy
from .near_person_watch import NEAR_PERSON_STALE_SEC, NearPersonWatch
from .nav_speed import BASE_THETA_VEL, NAV_SPEED_DEFAULT
from .pose_relay import ROBOT_POSE_TOPIC, PoseCache
from .retry_marker import RetryMarker, pose_key

try:  # ROSなしの単体テストでも、純粋な関数だけを読み込めるようにする
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object
try:  # 中継点(グローバルコストマップ+TF)用。無ければ従来の反応的な離脱だけ
    from nav_msgs.msg import OccupancyGrid, Path
    from rclpy.time import Time
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
except ImportError:  # pragma: no cover
    OccupancyGrid = None
    Path = None
    Buffer = None


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def scan_points(ranges, angle_min: float, angle_increment: float, range_min: float, range_max: float,
                max_range: float) -> List[Tuple[float, float]]:
    """(距離, 角度rad)のうち、有効でmax_range以内のものだけ。"""
    out = []
    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r < max(range_min, 0.05) or r > min(range_max, max_range):
            continue
        out.append((float(r), angle_min + i * angle_increment))
    return out


def thin_points(points: List[Tuple[float, float]], max_points: int = 150) -> List[Tuple[float, float]]:
    """点がmax_pointsを超えるとき、均等(等間隔)に間引いてmax_points個にする。元の並び(角度の昇順)は保つ。超えなければそのまま。
    先読みの衝突シミュレーション(pick_motion)は、点の数に比例して重いため(2026-10-24)。"""
    n = len(points)
    if max_points <= 0 or n <= max_points:
        return list(points)
    return [points[(i * n) // max_points] for i in range(max_points)]


def min_range(points: List[Tuple[float, float]]) -> Optional[float]:
    return min((r for r, _ in points), default=None)


def flee_angle(points: List[Tuple[float, float]], radius: float) -> Optional[float]:
    """近い点ほど強く、壁から遠ざかる向き(機体の前=0、左が正、rad)。近い点が無ければNone。"""
    vx = vy = 0.0
    for r, a in points:
        if r >= radius:
            continue
        w = radius - r
        vx -= w * math.cos(a)
        vy -= w * math.sin(a)
    if math.hypot(vx, vy) < 1e-6:
        return None
    return math.atan2(vy, vx)


def escape_command(flee: float, linear: float, angular_max: float, gain: float = 1.2) -> Tuple[float, float]:
    """遠ざかる向きflee(rad)へ動くための(直進m/s, 旋回rad/s)。
    ほぼ前方なら前進しながら向きを合わせる。真横寄りなら、旋回を主にして少しだけ前へ(差動二輪は横に動けない)。
    後ろ側(|flee|>=120°)が遠ざかる向きのときだけ、低速で後退する。"""
    a = wrap_angle(flee)
    ang = max(-angular_max, min(angular_max, gain * a))
    if abs(a) <= math.radians(60):
        return linear, ang
    if abs(a) >= math.radians(120):
        return -linear, 0.0
    return 0.5 * linear, ang


class WallEscape(Node):  # type: ignore[misc]
    # 先読みに使う点の範囲と上限の既定値(__init__を通さない単体テストでも動くように)
    _lookahead = 1.5
    _look_max = 150
    _look_cache = None
    # 【2026-10-24 M3】同じ目的地の再送で「やり直し済み」を戻さない範囲と、離脱の総回数の上限(__init__を通さないテストでも動くように)
    SAME_GOAL_M = 0.05        # 直前の目的地とxyがこの距離以内なら、同じ目的地とみなす
    SAME_GOAL_SEC = 60.0      # やり直してからこの秒数以内は、同じ目的地の再送で_retry_usedを維持する
    _retry_used_at = None
    _escape_times = None      # 自動の離脱を始めた時刻(秒)のリスト
    _max_escapes = 3
    _escape_window = 300.0
    _limit_log_time = -1e9

    def __init__(self) -> None:
        super().__init__("wall_escape")
        self.declare_parameter("enabled", True)
        self.declare_parameter("trigger_distance_m", 0.45)
        self.declare_parameter("clear_distance_m", 0.55)
        self.declare_parameter("repulsion_radius_m", 0.7)
        # 【2026-10-24】先読み(pick_motion)に渡す点の範囲。逃げる向き(flee_angle)は従来どおりrepulsion_radius_m(0.7 m)の点だけで決めるが、
        # 0.7 m/s x 1.5秒 = 約1.05 m進むので、0.7〜1.3 m先の物も評価しないと、離脱中に正面の壁へ突っ込む。点が多いときはlookahead_max_pointsに間引く。
        self.declare_parameter("lookahead_radius_m", 1.5)
        self.declare_parameter("lookahead_max_points", 150)
        # 【2026-10-23】通常の走行と同じ速さにした(0.10 m/s・0.6 rad/s では遅く、6秒のタイムアウトで失敗した)。時間も6→10秒。
        # 角が壁に当たらないかは、動かす前に先読み(motion_sim)で確認し、衝突監視も通る(速さを上げても安全の枠は同じ)。
        self.declare_parameter("escape_linear_mps", NAV_SPEED_DEFAULT)
        self.declare_parameter("escape_angular_max_rps", BASE_THETA_VEL)
        self.declare_parameter("max_escape_sec", 10.0)
        self.declare_parameter("scan_stale_sec", 1.0)
        self.declare_parameter("goal_memory_sec", 30.0)
        self.declare_parameter("proactive", True)
        # 【2026-10-24】離脱の総回数の上限。到達できない目的地を他の出し元が数秒おきに再送する構成で、離脱が繰り返されるのを防ぐ。超えたら離脱しない
        self.declare_parameter("max_escapes_per_window", 3)
        self.declare_parameter("escape_window_sec", 300.0)
        self.declare_parameter("retry_marker_match", True)  # 出し直しの目印を(x,y,yaw)一致で判定。falseなら従来(次の1件を無条件で無視)
        self.declare_parameter("retry_marker_ttl_sec", 2.0)  # 目印の有効秒数(過ぎたら捨てる)
        self._enabled = bool(self.get_parameter("enabled").value)
        self._trigger = float(self.get_parameter("trigger_distance_m").value)
        self._clear = float(self.get_parameter("clear_distance_m").value)
        self._radius = float(self.get_parameter("repulsion_radius_m").value)
        self._lookahead = max(self._radius, float(self.get_parameter("lookahead_radius_m").value))
        self._look_max = int(self.get_parameter("lookahead_max_points").value)
        self._lin = float(self.get_parameter("escape_linear_mps").value)
        self._ang_max = float(self.get_parameter("escape_angular_max_rps").value)
        self._max_sec = float(self.get_parameter("max_escape_sec").value)
        self._stale = float(self.get_parameter("scan_stale_sec").value)
        self._goal_memory = float(self.get_parameter("goal_memory_sec").value)
        self._proactive = bool(self.get_parameter("proactive").value)
        self._max_escapes = int(self.get_parameter("max_escapes_per_window").value)
        self._escape_window = float(self.get_parameter("escape_window_sec").value)

        self._points: List[Tuple[float, float]] = []
        self._scan_time = None
        self._last_goal: Optional[PoseStamped] = None
        self._last_goal_time = None
        self._retry_used = False  # 記憶している目的地を、すでに1回やり直したか
        # 自分が出し直した目的地を、記憶の更新に使わない。(x,y,yaw)の一致で判定し、時間で失効する(B7)
        self._retry_marker = RetryMarker(float(self.get_parameter("retry_marker_ttl_sec").value),
                                         match=bool(self.get_parameter("retry_marker_match").value))
        self._wasd = False
        self._soft_estop = False
        self._hard_estop = False
        self._near_person = False
        # 【2026-11-04】near_person_stop の受信が一度あったのに途絶えたら、近接停止と同じ扱い(一度も受信していなければ適用しない。0以下で無効)
        self.declare_parameter("near_person_stale_sec", NEAR_PERSON_STALE_SEC)
        self._near_person_stale = float(self.get_parameter("near_person_stale_sec").value)
        self._near_person_time = None
        self._near_person_stale_warned = False
        self._escaping_until = None
        self._pending_retry = False

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self._goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 5)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)
        self.create_subscription(String, "/nokosahen/nav2_goal_status", self._on_goal_status, 10)
        self.create_subscription(Bool, "/nokosahen/wasd_active", lambda m: setattr(self, "_wasd", bool(m.data)), 10)
        self.create_subscription(Bool, "/nokosahen/soft_estop", lambda m: setattr(self, "_soft_estop", bool(m.data)), 10)
        self.create_subscription(Bool, "/estop_state", lambda m: setattr(self, "_hard_estop", bool(m.data)), 10)
        self.create_subscription(Bool, "/nokosahen/near_person_stop", self._on_near_person, 10)
        self.create_subscription(Bool, "/nokosahen/cancel_goal", self._on_cancel, 10)
        # 【2026-10-22】動きの調停(docs/motion_priority.md)。ルーターが先回し旋回(Nav2のSpinまたは代替の回頭)をしている間は、
        # こちらは動かない(同じ/cmd_vel_navに2つが出して互いに邪魔するのを防ぐ)。Nav2が経路を作れた(/plan)ら、自分の離脱はやめてNav2に任せる。
        self._router_rotating = False
        self._proactive_due = None
        # 動きの調停ノード(motion_arbiter)の許可。一度も受信していなければ従来どおり(許可)、受信が途絶えたら不許可(安全側)
        self._grant = None
        self._grant_time = None
        self._escape_manual = False
        self.create_subscription(String, "/nokosahen/motion_grant", self._on_grant, 5)
        self._self_cancel_until = 0.0
        self._cancel_pub = self.create_publisher(Bool, "/nokosahen/cancel_goal_quiet", 10)  # 静かな取り消し(バーストなし)
        self.create_subscription(Bool, "/nokosahen/router_rotating", lambda m: setattr(self, "_router_rotating", bool(m.data)), 10)
        if Path is not None:
            self.create_subscription(Path, "/plan", self._on_plan, 5)
        self._setup_waypoint()
        self.create_timer(0.1, self._tick)
        # 【2026-10-22】管理画面の「壁から離れる」ボタン(手動の離脱)と、「壁際で動けない」状態の配信。手動の離脱は目的地を出し直さない。
        self._manual_reason = ("", 0.0)  # 手動の離脱を断った理由と時刻
        self._last_stop = ("", 0.0)  # 直近の離脱の終了理由と時刻
        self._last_abort = 0.0  # Nav2の目的地が中止(aborted)になった時刻。0=なし
        self.create_subscription(Bool, "/nokosahen/wall_escape/trigger", self._on_trigger, 10)
        self._status_pub = self.create_publisher(String, "/nokosahen/wall_escape/status", 10)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(f"wall_escape: enabled={self._enabled} trigger={self._trigger}m")

    def _now(self):
        return self.get_clock().now()

    def _on_scan(self, msg) -> None:
        # 【2026-10-23】毎スキャン(10Hz)で1080点をPythonで処理するとCPUを約2割使い、Nav2のTFが遅れて経路追従が失敗した。
        # 最新のメッセージだけ覚え、点への変換は、使うとき(離脱の判定・状態配信)にだけ行う。
        self._scan_msg = msg
        self._points_cache = None
        self._look_cache = None
        self._scan_time = self._now()

    @property
    def _points(self) -> List[Tuple[float, float]]:
        cache = getattr(self, "_points_cache", None)
        if cache is None:
            m = getattr(self, "_scan_msg", None)
            cache = [] if m is None else scan_points(m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max, self._radius)
            self._points_cache = cache
        return cache

    @_points.setter
    def _points(self, value) -> None:
        self._points_cache = value
        self._look_cache = None

    @property
    def _look_points(self) -> List[Tuple[float, float]]:
        """先読み用の点(機体座標の極座標)。lookahead_radius_m以内、均等に間引いて最大lookahead_max_points個。使うときだけ作る。"""
        cache = self._look_cache
        if cache is None:
            m = getattr(self, "_scan_msg", None)
            if m is None:
                pts = list(self._points)
            else:
                pts = scan_points(m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max, self._lookahead)
            cache = thin_points(pts, self._look_max)
            self._look_cache = cache
        return cache

    def _on_goal(self, msg) -> None:
        # 自分が出し直した目的地は、記憶を更新しない(同じ目的地の再試行は1回まで)
        if self._retry_marker.consume(pose_key(msg), self._now().nanoseconds / 1e9):
            return
        now = self._now().nanoseconds / 1e9
        # 【2026-10-24】他の出し元が同じ目的地を数秒おきに再送すると、毎回「やり直し済み」が戻り、離脱が繰り返された。
        # 直前の目的地とxyが5 cm以内なら、やり直してから60秒以内は維持する(60秒たてば戻る。総回数の上限は別に効く)。
        if not self._is_same_goal_resend(msg, now):
            self._retry_used = False
        self._last_goal = msg
        self._last_goal_time = self._now()
        self._pending_retry = False
        # 【2026-10-22】目的地が出た時点で壁が近ければ、失敗を待たずに先に壁から離れる。Nav2は、経路が作れないと回復動作を5回
        # 繰り返してから中止(数十秒)するため、中止を待つと、壁際で長い間動かない。離れたあとで、同じ目的地を出し直す。
        # 【2026-10-22】すぐには始めず0.4秒待つ(同じ目的地でルーターが先回し旋回を始めたら、そちらを優先して、こちらは動かない)
        # 同じ目的地の再送で、やり直し済みが維持されているときは、先に離れる処理も始めない(再送のたびに離脱を繰り返さない)
        if self._proactive and self._enabled and not self._retry_used:
            self._proactive_due = self._now().nanoseconds / 1e9 + 0.4

    def _is_same_goal_resend(self, msg, now: float) -> bool:
        """やり直し済みの目的地と同じ(xyが5 cm以内)目的地が、やり直しから60秒以内に再送されたか。"""
        old = self._last_goal
        if not self._retry_used or old is None or self._retry_used_at is None:
            return False
        if now - self._retry_used_at > self.SAME_GOAL_SEC:
            return False
        try:
            d = math.hypot(msg.pose.position.x - old.pose.position.x, msg.pose.position.y - old.pose.position.y)
        except AttributeError:
            return False
        return d <= self.SAME_GOAL_M

    def _escape_budget_ok(self) -> bool:
        """自動の離脱を始めてよいか。直近escape_window_sec(5分)にmax_escapes(3回)始めていたら、始めない(理由は状態のreasonに出す)。
        手動の離脱(管理画面のボタン)には使わない。"""
        now = self._now().nanoseconds / 1e9
        times = [x for x in (self._escape_times or []) if now - x < self._escape_window]
        self._escape_times = times
        if len(times) < self._max_escapes:
            return True
        reason = f"離脱の回数の上限(直近{self._escape_window / 60.0:g}分で{self._max_escapes}回)に達したため、離脱しません"
        self._last_stop = (reason, now)
        if now - self._limit_log_time >= 10.0:
            self._limit_log_time = now
            self.get_logger().warn(reason)
        return False

    def _on_cancel(self, msg) -> None:
        if msg.data and self._now().nanoseconds / 1e9 < self._self_cancel_until:
            return  # 手動の離脱の前に、自分が出した取り消し
        if msg.data:
            self._pending_retry = False
            self._last_goal = None
            self._stop_escape("目的地の取り消し")

    def _on_plan(self, msg) -> None:
        """Nav2が経路を作れた = Nav2が動かせる。自動の離脱(目的地の出し直しがある方)は、そこでやめて任せる(コントローラと同時に出さない)。"""
        if self._escaping_until is not None and self._pending_retry and len(getattr(msg, "poses", [])) >= 2:
            self._pending_retry = False
            self._stop_escape("Nav2が経路を作れたため任せます")

    def _on_goal_status(self, msg) -> None:
        try:
            ev = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if ev.get("event") != "finished" or int(ev.get("status", 0)) != 6:  # 6 = aborted
            return
        self._last_abort = self._now().nanoseconds / 1e9
        if not self._enabled or self._last_goal is None or self._escaping_until is not None:
            return
        if self._last_goal_time is None or (self._now() - self._last_goal_time).nanoseconds / 1e9 > self._goal_memory:
            return
        if self._retry_used:
            return  # この目的地は、すでに1回やり直した
        if not self._blocked() and self._wall_close() and self._escape_budget_ok():
            self._start_escape()

    def _scan_fresh(self) -> bool:
        return self._scan_time is not None and (self._now() - self._scan_time).nanoseconds / 1e9 <= self._stale

    def _wall_close(self) -> bool:
        m = min_range(self._points)
        return self._scan_fresh() and m is not None and m < self._trigger

    def _on_grant(self, msg) -> None:
        try:
            self._grant = bool(json.loads(msg.data).get("allow", {}).get("wall_escape", False))
        except (ValueError, TypeError, AttributeError):
            return
        self._grant_time = self._now().nanoseconds / 1e9

    def _grant_ok(self) -> bool:
        if self._grant_time is None:
            return True   # motion_arbiterが無い構成は、従来どおり
        if self._now().nanoseconds / 1e9 - self._grant_time > 1.0:
            return False  # 受信が途絶えた
        return bool(self._grant)

    def _on_near_person(self, msg) -> None:
        self._near_person = bool(msg.data)
        self._near_person_time = self._now().nanoseconds / 1e9
        self._near_person_stale_warned = False

    def _near_person_stale_now(self) -> bool:
        stale = float(getattr(self, "_near_person_stale", NEAR_PERSON_STALE_SEC))
        t = getattr(self, "_near_person_time", None)
        if stale <= 0 or t is None:
            return False  # 無効、または一度も受信していない(近接停止なしの構成)
        if self._now().nanoseconds / 1e9 - t <= stale:
            return False
        if not getattr(self, "_near_person_stale_warned", False):
            self._near_person_stale_warned = True
            self.get_logger().warn("near_person_stop の受信が途絶えました(person_target_detectorが止まった可能性)。安全側で止まります")
        return True

    def _blocked_hard(self) -> bool:
        """調停の許可に関係なく止める条件(緊急停止・手動運転・近接停止・ルーターの回頭中)。"""
        return self._wasd or self._soft_estop or self._hard_estop or self._near_person or self._near_person_stale_now() or self._router_rotating

    def _blocked(self) -> bool:
        return self._blocked_hard() or not self._grant_ok()

    def _start_escape(self, manual: bool = False) -> None:
        self._escape_manual = False
        if not manual:
            self._escape_times = (self._escape_times or []) + [self._now().nanoseconds / 1e9]
        self._escaping_until = self._now().nanoseconds / 1e9 + self._max_sec
        self._pending_retry = True
        self.get_logger().info(f"壁際から離れます(最短 {min_range(self._points):.2f} m)")
        self._plan_waypoint()
        self._publish_status()

    def _stop_escape(self, reason: str) -> None:
        if self._escaping_until is not None:
            self._escaping_until = None
            self._cmd_pub.publish(Twist())
            self.get_logger().info(f"壁際からの脱出を終了: {reason}")
            self._last_stop = (reason, self._now().nanoseconds / 1e9)
            self._publish_status()

    def _tick(self) -> None:
        try:
            self._tick_inner()
        except Exception as exc:  # noqa: BLE001
            # 【2026-10-24】例外でタイマーが止まる/ノードが落ちると、最後の速度指令が残りうる。離脱中ならゼロ速度を出して終え、ノードは落とさない。
            self._on_tick_error(exc)

    def _on_tick_error(self, exc: Exception) -> None:
        escaping = self._escaping_until is not None
        self._proactive_due = None
        self._pending_retry = False  # 例外のあとに、目的地を出し直さない
        try:
            self.get_logger().error(f"wall_escapeの処理でエラー(離脱をやめます): {exc!r}")
        except Exception:  # noqa: BLE001
            pass
        if not escaping:
            return  # 動かしていないなら、ゼロ速度でNav2の指令を邪魔しない
        self._escaping_until = None
        try:
            self._cmd_pub.publish(Twist())
        except Exception:  # noqa: BLE001
            pass
        try:
            self._last_stop = ("内部エラー", self._now().nanoseconds / 1e9)
            self._publish_status()
        except Exception:  # noqa: BLE001
            pass

    def _tick_inner(self) -> None:
        due = self._proactive_due
        if due is not None and self._now().nanoseconds / 1e9 >= due:
            self._proactive_due = None
            if self._escaping_until is None and not self._blocked() and self._wall_close() and self._escape_budget_ok():
                self._start_escape()
        if self._escaping_until is None:
            return
        now = self._now().nanoseconds / 1e9
        if (self._blocked_hard() if self._escape_manual else self._blocked()):
            self._pending_retry = False
            self._stop_escape("緊急停止/手動運転/近接停止")
            return
        if not self._scan_fresh():
            self._pending_retry = False
            self._stop_escape("LiDAR途絶")
            return
        m = min_range(self._points)
        if m is None or m >= self._clear:
            self._stop_escape("十分に離れました")
            self._retry_goal()
            return
        if now >= self._escaping_until:
            self._pending_retry = False
            self._stop_escape("時間切れ")
            return
        flee = flee_angle(self._points, self._radius)
        if flee is None:
            self._pending_retry = False
            self._stop_escape("逃げる向きがありません")
            return
        # 【2026-10-22】先読みの衝突シミュレーション(motion_sim.py): 候補の動き(前進・後退・旋回・弧)ごとに、機体の外形(52cm角)を
        # 動かして角が壁(LiDARの点)に当たらないかを調べ、当たらない動きのうち壁から離れるものだけを選ぶ。なければ動かない。
        # 先読みに渡す点は、逃げる向きの計算(0.7 m以内)より遠く(lookahead_radius_m=1.5 m)まで。離脱で進む約1.05 m先の物も評価する(2026-10-24)。
        picked = pick_motion(to_xy(self._look_points), flee, self._lin, self._ang_max, **self._guidance())
        if picked is None:
            self._pending_retry = False
            self._stop_escape("角が当たらずに動ける向きがありません")
            return
        lin, ang = picked
        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        self._cmd_pub.publish(cmd)

    # ---- 壁を避ける中継点 / 目的地の向き(2026-10-22) ----
    # 方式: 中継点を /goal_pose に出してNav2で行く方式は採らず、離脱の動きの評価(pick_motion)に「中継点の向き」を加点する方式にした。
    # 理由: 壁際ではNavFnが出発点から経路を作れないのが問題の根本で、/goal_pose方式だとその中継点も同じ理由で失敗しうる上、
    # 目的地の記憶・出し直し・無限ループ防止の状態が増える。pick_motionなら /cmd_vel_nav 経由・当たる動きの除外・最長秒数・
    # 1目的地1回の既存の枠のまま、向きの偏りだけを足せる(中継点が見つからなければ従来どおり)。
    def _setup_waypoint(self) -> None:
        self._waypoint = None  # map座標の中継点
        self._costmap = None
        self._pose_cache = None
        if OccupancyGrid is None or rclpy is None:
            return
        # 【2026-10-26】/tfを直接購読せず、pose_relay(5Hz)の /nokosahen/robot_pose から機体位置を得る(CPU削減)
        self._pose_cache = PoseCache()
        self.create_subscription(String, ROBOT_POSE_TOPIC, lambda m: self._pose_cache.update(m.data), 2)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._on_costmap, qos)

    def _on_costmap(self, msg) -> None:
        info = msg.info
        self._costmap = GridInfo(msg.data, info.width, info.height, info.resolution,
                                 info.origin.position.x, info.origin.position.y)

    def _robot_pose(self) -> Optional[Tuple[float, float, float]]:
        """map座標の機体位置(x, y, yaw)。取れなければNone。"""
        if self._pose_cache is None:
            return None
        bp = self._pose_cache.base(self._now().nanoseconds / 1e9)
        if bp is None or bp[3] > 1.5:  # 未受信/TFが取れていない/古い位置(2026-10-26: 古さの確認を追加。ナビ側と同じ1.5秒)
            return None
        return bp[0], bp[1], bp[2]

    def _goal_xy(self) -> Optional[Tuple[float, float]]:
        g = self._last_goal
        if g is None or getattr(g.header, "frame_id", "map") not in ("map", ""):
            return None
        return g.pose.position.x, g.pose.position.y

    def _plan_waypoint(self) -> None:
        self._waypoint = None
        pose = self._robot_pose()
        if self._costmap is None or pose is None:
            return
        wp = find_escape_waypoint(self._costmap, (pose[0], pose[1]), self._goal_xy())
        self._waypoint = wp
        if wp is not None:
            self.get_logger().info(f"壁を避ける中継点: ({wp[0]:.2f}, {wp[1]:.2f})")

    def _guidance(self) -> dict:
        """pick_motionに渡す、目的地/中継点の向き(機体座標rad)。使えないものは渡さない(従来どおり)。"""
        out = {}
        pose = self._robot_pose()
        if pose is None:
            return out
        x, y, yaw = pose
        wp = self._waypoint
        if wp is not None:
            if math.hypot(wp[0] - x, wp[1] - y) < 0.12:
                self._waypoint = None  # 着いた
            else:
                out["waypoint_dir"] = wrap_angle(math.atan2(wp[1] - y, wp[0] - x) - yaw)
        g = self._goal_xy()
        if g is not None and math.hypot(g[0] - x, g[1] - y) > 0.3:
            out["goal_dir"] = wrap_angle(math.atan2(g[1] - y, g[0] - x) - yaw)
        return out

    def _reject_manual(self, reason: str) -> None:
        self._manual_reason = (reason, self._now().nanoseconds / 1e9)
        self.get_logger().info(f"手動の壁離れを断りました: {reason}")

    def _on_trigger(self, msg) -> None:
        """手動の離脱を1回始める(目的地は出し直さない)。緊急停止・手動運転・近接停止中は断る。"""
        if not msg.data:
            return
        if not self._enabled:
            self._reject_manual("壁から離れる機能が無効です")
        elif self._blocked_hard():
            self._reject_manual("緊急停止・手動運転・近接停止のどれかが有効です")
        elif self._escaping_until is not None:
            self._reject_manual("すでに離れています")
        elif not self._scan_fresh():
            self._reject_manual("LiDARの値が届いていません")
        elif (min_range(self._points) or 99.0) >= self._clear:
            self._reject_manual("壁から十分に離れています")
        else:
            self._manual_reason = ("", 0.0)
            # 手動の離脱は、実行中のNav2の目的地を先に取り消す(Nav2のコントローラと同時に/cmd_vel_navへ出さない)
            self._self_cancel_until = self._now().nanoseconds / 1e9 + 1.0
            self._cancel_pub.publish(Bool(data=True))
            self._start_escape(manual=True)
            self._escape_manual = True
            self._pending_retry = False  # 手動のときは、目的地を出し直さない

    def status_dict(self) -> dict:
        """管理画面向けの状態。state: idle|escaping|failed|blocked。stuck=壁が近く(trigger未満)、直近5秒以内に目的地が中止になった。"""
        now = self._now().nanoseconds / 1e9
        m = min_range(self._points) if self._scan_fresh() else None
        state, reason = "idle", ""
        if self._escaping_until is not None:
            state = "escaping"
        elif self._blocked():
            state, reason = "blocked", "緊急停止・手動運転・近接停止のどれかが有効です"
        elif self._last_stop[1] and now - self._last_stop[1] <= 10.0 and self._last_stop[0] not in ("十分に離れました", "目的地の取り消し"):
            state, reason = "failed", self._last_stop[0]
        if not reason and self._manual_reason[1] and now - self._manual_reason[1] <= 10.0:
            reason = self._manual_reason[0]
        stuck = m is not None and m < self._trigger and self._last_abort > 0.0 and now - self._last_abort <= 5.0
        return {"state": state, "min_range_m": None if m is None else round(m, 3), "reason": reason, "stuck": bool(stuck)}

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(self.status_dict(), ensure_ascii=False)
        self._status_pub.publish(msg)

    def _retry_goal(self) -> None:
        if not self._pending_retry or self._last_goal is None:
            return
        self._pending_retry = False
        goal = self._last_goal
        self._retry_used = True
        self._retry_used_at = self._now().nanoseconds / 1e9
        self._retry_marker.mark(pose_key(goal), self._now().nanoseconds / 1e9)
        self.get_logger().info("壁から離れたので、直前の目的地をもう一度出します")
        self._goal_pub.publish(goal)


def main() -> None:
    rclpy.init()
    node = WallEscape()
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
