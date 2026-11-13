from __future__ import annotations

import math
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    from nav_msgs.msg import Path
except ImportError:  # テスト用(rclpy無し環境)
    Path = None

from .boundary_motion_guard import path_exit_info
from .json_tools import dumps, loads
from .map_orientation import boundary_message_update, point_in_rect
from .motion_sim import first_collision_time
from .nav2_detour import DetourMixin
from .nav_speed import BASE_THETA_VEL
from .pose_relay import ROBOT_POSE_TOPIC, PoseCache

try:  # 単体テストのスタブ環境にsensor_msgsが無くても読み込めるように
    from sensor_msgs.msg import LaserScan
except ImportError:  # pragma: no cover
    LaserScan = None

# 【2026-10-26】機体の位置は /tf を直接購読せず、pose_relay(5Hz)の /nokosahen/robot_pose から得る(CPU削減。/tf 70Hzの購読が重かった)。


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# 【2026-10-24】代替の回頭が「調停ノードの許可の取り消し」で中止されたとき、取り消した持ち主がこれらなら、目的地をNav2へ送らず捨てる
# (緊急停止・手動運転・近接停止の最中に、中止のついでにNav2へ目的地を送ると、止めたはずの走行が始まる)。それ以外(nav2/wall_escape等)は従来どおり送る。
# 【2026-11-04】近接停止の持ち主は "near_person" のほか "near_person_face_turn"(近接停止中の旋回だけ許可)もある。前方一致で判定する。
GRANT_DISCARD_OWNERS = ("estop", "manual", "near_person")


def discard_goal_on_revoke(owner: Optional[str]) -> bool:
    if not isinstance(owner, str):
        return False
    return owner in GRANT_DISCARD_OWNERS or owner.startswith("near_person")


def pre_rotation_angle(
    robot_x: float, robot_y: float, robot_yaw: float,
    goal_x: float, goal_y: float,
    threshold_deg: float = 100.0, min_distance: float = 0.6,
) -> Optional[float]:
    """目的地が背後寄りなら、先にその場で回る相対角度[rad]を返す。回る必要が無ければNone。

    近い目的地(min_distance未満)は、向きが決まらず意味が薄いので回さない。
    """
    dx, dy = goal_x - robot_x, goal_y - robot_y
    if not all(math.isfinite(v) for v in (dx, dy, robot_yaw)):
        return None
    if math.hypot(dx, dy) < min_distance:
        return None
    delta = wrap_angle(math.atan2(dy, dx) - robot_yaw)
    if abs(delta) < math.radians(threshold_deg):
        return None
    return delta


class Nav2GoalRouter(DetourMixin, Node):
    """Route high-level /goal_pose messages into the Nav2 NavigateToPose action.

    手動運転(WASD)・緊急停止(ソフト/ハード)・behavior_directorのHOLD状態が
    始まったら、その時点でNav2が実行中のゴールを明示的にキャンセルする。
    キャンセルしないと、Nav2(controller_server/velocity_smoother)は既に
    受理済みのゴールに向けて/cmd_velを出し続け、手動運転の速度指令や
    緊急停止のゼロ速度と競合してしまうため。

    【2026-07-16 追記】cancel_goal_async()は非同期で、Nav2側
    (controller_server→velocity_smoother→collision_monitor→/cmd_vel)が
    実際にキャンセルを反映して/cmd_velを出さなくなるまでにはラグがある
    (velocity_smootherのvelocity_timeout分は特に遅れうる)。このノードが
    直接/cmd_velへゼロ速度を、キャンセル要求と同時に・かつ一定時間
    連続して送ることで、Nav2側の停止が遅れてもゼロ速度が上書きし続ける
    ようにする(「緊急停止を押しても(慣性で)しばらく動き続ける」対策)。

    【2026-07-17 修正】以前は/nokosahen/manual_override(true)を無条件で
    このキャンセル+ゼロ速度バーストのトリガーにしていたが、これは2つの
    バグを生んでいた。
    (1) 地図クリックでのゴール追従(TeleopController.activate_goal_mode)も
        behavior_directorとの競合回避のため同じmanual_overrideを0.5秒毎に
        送り続ける設計のため、送ったばかりの自分のゴールを自分で
        キャンセルし続けてしまい、地図クリック走行が構造的に機能しなかった。
    (2) WASD運転中もhandle_drive()が呼ばれる毎(約10Hz)にmanual_overrideを
        再送するため、そのたびにゼロ速度バースト(20Hz×1.5秒)が再スタートし、
        WASDの速度指令とゼロ速度が混ざり続けてガタつく(実機で報告された
        振動症状の一因)。
    そのため、WASD運転の開始だけを示す専用トピック
    (/nokosahen/wasd_active、TeleopController.handle_drive()内でのみ発行)を
    新設し、そちらは「falseからtrueへ変わった瞬間」だけキャンセルを発火する
    (継続中の再送では発火しない)。地図クリック追従はこのトピックを
    publishしないため、自分のゴールを誤ってキャンセルすることも無くなる。
    estop・behavior_directorのHOLD状態(/nokosahen/cancel_goal、範囲外/
    カメラ障害物)は、手動運転と同時に起こらない(behavior_director側の
    分岐順でMANUAL_OVERRIDEが常に優先されるため)ので、従来通りtrueが
    届くたびに毎回発火してよい(安全側に倒しても他の系統と競合しない)。

    【2026-07-17 再修正】エッジ検知後もなお「WASDを押すと少し振動する」
    症状が実機で残っていたため、WASD運転開始時のトリガーからゼロ速度
    バースト自体を外した(ゴールキャンセルのみ行う)。WASD側は既に自分の
    望む速度を約10Hzで継続publishしているので、そこへこのノードのゼロ速度
    まで混ざると、Nav2の残存出力・ゼロ・WASDの三つ巴になりガタつく。estop/
    HOLDは他に正当な駆動指令が存在しないためゼロ速度バーストが安全側に働くが、
    WASDの場合はWASD自身の指令こそが「正しい」ものなので、ゼロで上書きする
    必要が無い(ゴールキャンセル+cancel_goal_async後段のNav2側停止だけで
    十分)。
    """

    # 先回し旋回の状態の既定値(__init__を通さない単体テストや、spinが使えない環境でも従来どおり動く)
    _pre_rotate_enabled = False
    _spin_client = None
    _spin_handle = None
    _pose_cache = None  # PoseCache(2026-10-26。__init__を通さない単体テストでは無い=位置不明=従来の安全側)
    _goal_seq = 0
    _grant_owner = None  # 直近のmotion_grantのowner(2026-10-24)
    # アンケート回答中ホールドの既定値(購読が無い/__init__を通さない場合は従来どおり=ブロックしない)
    _survey_active = False
    _survey_active_time = None
    # 【2026-10-10追加】地図作成(SLAM)中・自動探索中の判定(behavior_directorの_mapping_or_exploringと同じ定義)。
    # 探索中はお客さんがいない前提で、アンケート回答中でもゴールを拒否・取り消ししない。未受信=探索中ではない=従来どおり。
    _map_building_active = False
    _frontier_enabled = False
    _frontier_status_time = None
    _FRONTIER_STATUS_TIMEOUT_SEC = 10.0
    # 【2026-11-05】近接停止(/nokosahen/near_person_stop=true)の間は、手動運転以外の出し元(探索など)の目的地を受け付けない。
    # 未受信=近接停止なしの構成=従来どおり。手動運転(地図クリック追従)は manual_override(0.5秒ごとに再送)で見分ける。
    _near_person_stop = False
    _near_person_stop_time = None
    _manual_override_time = None
    _NEAR_PERSON_STOP_TIMEOUT_SEC = 1.0  # director の near_person_stop_timeout_sec と同じ
    _MANUAL_OVERRIDE_TIMEOUT_SEC = 1.5
    _near_plan_cancel_time = 0.0

    def __init__(self) -> None:
        super().__init__("nav2_goal_router")
        self.declare_parameter("action_name", "navigate_to_pose")
        # 【2026-08-06】以前はwait_for_server(timeout_sec=server_wait_seconds)で
        # ブロッキング待機していたが、rclpyはデフォルトSingleThreadedExecutorの
        # ため、その間20Hzのゼロ速度バーストtimer含む全処理が止まっていた。
        # 現在はserver_is_ready()(非ブロッキング)を使うため実質未使用だが、
        # 既存のnokosahen.yaml設定を壊さないためパラメータ自体は残す。
        self.declare_parameter("server_wait_seconds", 0.2)
        # Nav2側のキャンセル反映が遅れても確実に停止させるため、この秒数だけ
        # ゼロ速度を/cmd_velへ連続publishし続ける(velocity_smootherの
        # velocity_timeoutより長めに取っている)。
        self.declare_parameter("stop_burst_seconds", 1.5)
        self.declare_parameter("stop_burst_hz", 20.0)
        # 【2026-10-09追加】目的地がほぼ真後ろのとき、Nav2に任せると反対向きの旋回で計算が止まって
        # 動けなかった(実機)。先にNav2のspin(その場旋回)で目的地の方を向いてから経路追従に入る。
        self.declare_parameter("pre_rotate_enabled", True)
        self.declare_parameter("pre_rotate_threshold_deg", 100.0)
        # 【2026-10-19】0.6 -> 0.3 m。0.6 m未満の近い背後の目的地は旋回せずNav2(DWB)に任せていたが、DWBは背後の経路が苦手で
        # 「近い後ろへ行けない」原因になる。回転(約1〜2秒)のほうが確実なので、0.3 m未満だけ省く。
        self.declare_parameter("pre_rotate_min_distance", 0.3)
        self.declare_parameter("pre_rotate_time_allowance", 20.0)

        try:
            from nav2_msgs.action import NavigateToPose  # type: ignore

            self._navigate_action = NavigateToPose
        except Exception as exc:
            self._navigate_action = None
            self.get_logger().error(f"nav2_msgs is unavailable: {exc}")
            return

        action_name = str(self.get_parameter("action_name").value)
        self._server_wait_seconds = float(
            self.get_parameter("server_wait_seconds").value
        )
        self._stop_burst_seconds = max(
            0.0, float(self.get_parameter("stop_burst_seconds").value)
        )
        stop_burst_hz = max(1.0, float(self.get_parameter("stop_burst_hz").value))
        self._client = ActionClient(self, self._navigate_action, action_name)
        self._pre_rotate_enabled = bool(self.get_parameter("pre_rotate_enabled").value)
        self._pre_rotate_threshold = float(self.get_parameter("pre_rotate_threshold_deg").value)
        self._pre_rotate_min_dist = float(self.get_parameter("pre_rotate_min_distance").value)
        self._pre_rotate_allowance = float(self.get_parameter("pre_rotate_time_allowance").value)
        self._spin_client = None
        self._spin_handle = None
        self._goal_seq = 0  # 新しい目的地・キャンセルで増やし、古い旋回の完了通知を無視する
        self._pose_cache = PoseCache()
        self.create_subscription(String, ROBOT_POSE_TOPIC, self._on_robot_pose, 2)
        try:
            from nav2_msgs.action import Spin  # type: ignore

            self._spin_action = Spin
            self._spin_client = ActionClient(self, Spin, "spin")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"先回し旋回は使えません: {exc}")
            self._spin_client = None
        self._goal_handle = None
        self._goal_active = False
        # 【2026-08-06追加】現在実行中のゴール座標。/nokosahen/nav2_goal_status
        # に含めることで、frontier_explorer/behavior_directorが「自分が送った
        # ゴールの完了通知かどうか」を区別できるようにする(このトピックは
        # 送信元を区別せず全員に配信されるため、両方が同時に稼働していると
        # 互いの完了通知で自分のゴール追跡状態を誤ってクリアし合っていた)。
        self._goal_xy: Optional[Tuple[float, float]] = None
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # 代替の回頭は、速度制限→衝突監視を通る入口(/cmd_vel_nav)へ出す(安全層は迂回しない)
        self._nav_cmd_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self._fallback = None
        self._fallback_rotate_sec = 5.0
        # 【2026-10-22】先回し旋回(Spin・代替の回頭)の間はtrue。wall_escapeは、その間は動かない(docs/motion_priority.md)
        self._rotating_pub = self.create_publisher(Bool, "/nokosahen/router_rotating", 5)
        # 動きの調停ノード(motion_arbiter)の許可。一度も受信していなければ従来どおり(許可)、受信が途絶えたら不許可
        self._grant_rotate = None
        self._grant_time = None
        self.create_subscription(String, "/nokosahen/motion_grant", self._on_motion_grant, 5)
        self._rotating_hint_until = 0.0
        self._rotating_last = None
        self.create_timer(0.1, self._publish_rotating)
        # 代替の回頭の前に、機体の外形(52cm角)が回るときに角が壁に当たらないかを調べるためのLiDAR(2026-10-22)
        self._scan_msg = None   # 最新のスキャン(変換は、代替の回頭の判定のときだけ。2026-10-23: 毎回1080点を処理してCPUを使っていた)
        self._scan_xy_time = None
        if LaserScan is not None:
            self.create_subscription(LaserScan, "/scan", self._on_scan_for_sim, 5)
        self.create_timer(0.1, self._fallback_tick)
        # ゴールが受理/完了/失敗したことを他ノード(frontier_explorer等)へ
        # 知らせる。これが無いと、ゴール送信元は「近づいたか」「タイムアウト
        # したか」でしか完了を推測できず、Nav2が早期に失敗(経路計画不可等)
        # しても気づくまで無駄に待ち続けてしまう。
        self._goal_status_pub = self.create_publisher(
            String, "/nokosahen/nav2_goal_status", 10
        )
        self._stop_burst_until = None
        # WASD運転の「立ち上がり」検知用(falseから来た直後のtrueだけに反応する)。
        self._wasd_active_prev = False
        # 【2026-09-30追加】ハード/ソフト緊急停止の状態をラッチする。以前は
        # 「trueが届いた瞬間にキャンセルする」だけで、estop中に届いた新しい
        # /goal_poseは普通にNav2へ送られてしまっていた。engaged中は新規ゴールを
        # 拒否し、受理直後のゴールも即キャンセルする。
        self._hardware_estop = False
        self._soft_estop = False
        # 【2026-10-09追加】アンケート回答中(/nokosahen/survey_active)は新規ゴールを拒否し、
        # 実行中のゴールを取り消してゼロ速度を出す。配信が途絶えたら(3秒)失効して通常に戻る。
        self._survey_active = False
        self._survey_active_time = None
        self._map_building_active = False
        self._frontier_enabled = False
        self._frontier_status_time = None
        # 【2026-10-06追加】走行可能範囲(回転矩形)の多重防御。ダッシュボードの/api/goalと
        # frontier_explorerは範囲外のゴールを既に拒否するが、/goal_poseへ他の発行元から
        # 届いたゴールもここで拒否する。範囲が未設定(enabled偽/未受信)なら制限しない。
        # 不正なメッセージ(NaN/キー欠落等)は無視して直前の有効な範囲を保つ。
        # 判定できるのはframe_idが"map"(または空)のゴールだけ(機体基準フレーム等は
        # 変換できないため素通し=従来どおり。behavior_directorは自分で範囲へクランプ済み)。
        self._boundary_rect = None
        self.create_subscription(
            String, "/nokosahen/travel_boundary", self._on_boundary, 10
        )
        # 【2026-11-03】Nav2の経路(/plan)が走行可能範囲の外へ出るなら、その目的地への走行を止める。
        # 正しい経路は止めない: 許容0.05 m・連続0.25 m・2回連続・先頭(範囲外から戻る区間)は無視。
        self._current_goal_xy = None
        self._plan_last_time = 0.0
        self._plan_violations = 0
        self._plan_blocked_xy = None
        self._plan_blocked_until = 0.0
        if Path is not None:
            self.create_subscription(Path, "/plan", self._on_plan, 5)
        self.create_timer(1.0 / stop_burst_hz, self._stop_burst_tick)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal_pose, 10)
        # 【2026-11-05】近接停止中は探索など手動以外の目的地を拒否する(directorの取り消しは入った1回だけのため)
        self.create_subscription(Bool, "/nokosahen/near_person_stop", self._on_near_person_stop, 10)
        self.create_subscription(Bool, "/nokosahen/manual_override", self._on_manual_override_flag, 10)
        self.create_subscription(
            Bool, "/nokosahen/wasd_active", self._on_wasd_active, 10
        )
        self.create_subscription(Bool, "/nokosahen/soft_estop", self._on_soft_estop, 10)
        self.create_subscription(Bool, "/estop_state", self._on_hardware_estop, 10)
        self.create_subscription(
            Bool, "/nokosahen/survey_active", self._on_survey_active, 10
        )
        self.create_subscription(
            Bool, "/nokosahen/map_building_active", self._on_map_building_active, 5
        )
        self.create_subscription(
            String, "/nokosahen/frontier_explorer/status", self._on_frontier_status, 5
        )
        # 【2026-10-22】静かな取り消し: 実行中のNav2の目的地だけを取り消す(/cmd_velへのゼロ速度バーストは出さない)。
        # 管理画面の「壁から離れる」が、取り消しの直後に出す離脱の速度と、バーストのゼロが交互に混ざるのを防ぐ。
        self.create_subscription(Bool, "/nokosahen/cancel_goal_quiet", self._on_quiet_cancel, 10)
        self.create_subscription(
            Bool, "/nokosahen/cancel_goal", self._on_safety_cancel_trigger, 10
        )
        self._detour_init()  # 【2026-10-25】回復でも中止になった目的地は、経路をシミュレーションしてから進む(docs/detour.md)
        self.get_logger().info(f"routing /goal_pose to Nav2 action: {action_name}")

    def _on_boundary(self, msg: String) -> None:
        update = boundary_message_update(loads(msg.data, None))
        if update is None:
            self.get_logger().warn(
                "travel_boundary: 範囲が不正/不足のメッセージを無視しました(直前の範囲を維持)"
            )
            return
        self._boundary_rect = update[1]

    def _on_near_person_stop(self, msg: Bool) -> None:
        self._near_person_stop = bool(msg.data)
        self._near_person_stop_time = self.get_clock().now()

    def _on_manual_override_flag(self, msg: Bool) -> None:
        if bool(msg.data):
            self._manual_override_time = self.get_clock().now()

    @property
    def _manual_driving(self) -> bool:
        t = self._manual_override_time
        if t is None:
            return False
        return (self.get_clock().now() - t).nanoseconds / 1e9 <= self._MANUAL_OVERRIDE_TIMEOUT_SEC

    @property
    def _near_person_blocking(self) -> bool:
        """近接停止中(trueを受信してから1秒以内)で、手動運転でないならTrue。未受信・false・途絶はFalse(途絶の停止はdirector/arbiterが担当)。"""
        if not self._near_person_stop or self._near_person_stop_time is None:
            return False
        if (self.get_clock().now() - self._near_person_stop_time).nanoseconds / 1e9 > self._NEAR_PERSON_STOP_TIMEOUT_SEC:
            return False
        return not self._manual_driving

    def _on_plan(self, msg) -> None:
        now = time.monotonic()
        # 【2026-11-05】近接停止中に計画が出た=取り消しの後に送られた目的地が走っている。取り消しを出し直す(1秒に1回まで)。
        if self._goal_active and self._near_person_blocking and now - self._near_plan_cancel_time >= 1.0:
            self._near_plan_cancel_time = now
            self._trigger_cancel("近接停止中に経路が出た")
        if now - self._plan_last_time < 0.5:  # 1秒あたり最大2回
            return
        self._plan_last_time = now
        frame = str(getattr(msg.header, "frame_id", "") or "")
        rect = self._boundary_rect
        if rect is None or not self._goal_active or frame not in ("", "map"):
            self._plan_violations = 0
            return
        points = [(p.pose.position.x, p.pose.position.y) for p in getattr(msg, "poses", ())]
        if not path_exit_info(points, rect)["exits"]:
            self._plan_violations = 0
            return
        self._plan_violations += 1
        if self._plan_violations < 2:
            return
        self._plan_violations = 0
        target = self._current_goal_xy
        self.get_logger().warn("経路が走行可能範囲の外へ出るため、この目的地への走行を止めます")
        self._cancel_active_goal("経路が走行可能範囲の外へ出る")
        if target is not None:
            self._plan_blocked_xy = target
            self._plan_blocked_until = now + 10.0
        self._publish_goal_status("path_outside_boundary", target=target)

    def _goal_outside_boundary(self, pose: PoseStamped) -> bool:
        rect = self._boundary_rect
        if rect is None:
            return False
        frame = str(getattr(pose.header, "frame_id", "") or "")
        if frame not in ("map", ""):
            return False
        return not point_in_rect(pose.pose.position.x, pose.pose.position.y, rect)

    def _on_quiet_cancel(self, msg: Bool) -> None:
        if msg.data:
            self._cancel_active_goal("壁から離れる(手動)")

    def _stop_burst_tick(self) -> None:
        if self._stop_burst_until is None:
            return
        if self.get_clock().now() >= self._stop_burst_until:
            self._stop_burst_until = None
            return
        self._cmd_vel_pub.publish(Twist())

    def _on_wasd_active(self, msg: Bool) -> None:
        active = bool(msg.data)
        if active and not self._wasd_active_prev:
            # falseからtrueへ変わった瞬間(=WASD運転が今始まった)だけ発火する。
            # 【2026-07-17再修正】ここではゴールのキャンセルのみ行い、ゼロ速度
            # バーストは出さない。WASD側は自分自身が既に望む速度を継続的に
            # (約10Hz)publishしているので、そこへnav2_goal_router発のゼロ速度
            # まで混ざると三つ巴(Nav2の残存出力・ゼロ・WASD)になり、かえって
            # ガタつきが悪化する(実機で確認: エッジ検知だけでは解消しきれない
            # 「少し振動する」症状の原因)。estopと違い、WASD運転中は既に
            # 正当な駆動指令が存在するため、ゼロで上書きする必要が無い。
            self._cancel_active_goal("WASD運転開始")
        self._wasd_active_prev = active

    @property
    def _estop_active(self) -> bool:
        return self._hardware_estop or self._soft_estop

    def _on_map_building_active(self, msg: Bool) -> None:
        self._map_building_active = bool(msg.data)

    def _on_frontier_status(self, msg: String) -> None:
        data = loads(msg.data, {})
        if not isinstance(data, dict):
            return
        self._frontier_enabled = bool(data.get("enabled", False))
        self._frontier_status_time = self.get_clock().now()

    @property
    def _mapping_or_exploring(self) -> bool:
        """地図作成(SLAM)中、または自動探索が有効な間True。未受信ならFalse(behavior_directorと同じ定義)。"""
        if self._map_building_active:
            return True
        if not self._frontier_enabled or self._frontier_status_time is None:
            return False
        elapsed = (self.get_clock().now() - self._frontier_status_time).nanoseconds / 1e9
        return elapsed <= self._FRONTIER_STATUS_TIMEOUT_SEC

    @property
    def _survey_blocking(self) -> bool:
        """アンケート回答中(最後の受信がtrueで3秒以内)ならTrue。未受信はFalse=従来どおり。
        【2026-10-10】地図作成中/自動探索中は常にFalse(探索はアンケートで止めない。通常の自律走行は従来どおり止める)。"""
        if not self._survey_active or self._survey_active_time is None:
            return False
        if self._mapping_or_exploring:
            return False
        return (self.get_clock().now() - self._survey_active_time).nanoseconds / 1e9 <= 3.0

    def _on_survey_active(self, msg: Bool) -> None:
        was = self._survey_blocking
        self._survey_active = bool(msg.data)
        self._survey_active_time = self.get_clock().now()
        if self._survey_blocking and not was:  # 探索中は_survey_blockingがFalseのままなので取り消さない
            # 回答開始の瞬間に取り消し+ゼロ速度バースト(以降の再送では発火しない)。
            self._trigger_cancel("アンケート回答開始")

    def _on_soft_estop(self, msg: Bool) -> None:
        self._soft_estop = bool(msg.data)
        self._on_safety_cancel_trigger(msg)

    def _on_hardware_estop(self, msg: Bool) -> None:
        self._hardware_estop = bool(msg.data)
        self._on_safety_cancel_trigger(msg)

    def _on_safety_cancel_trigger(self, msg: Bool) -> None:
        # estop・behavior_directorのHOLD状態(範囲外/カメラ障害物)は、他に
        # 正当な駆動指令が存在しない(何も出さなければ本当に停止したい)ため、
        # trueが届くたびに毎回ゼロ速度バーストを発火して安全側に倒してよい。
        if bool(msg.data):
            self._trigger_cancel()

    def _trigger_cancel(self, reason: str = "estop/HOLD検知") -> None:
        # ゴールの有無に関わらず、まずゼロ速度を即座に・一定時間出し続ける。
        # Nav2側のキャンセル反映を待たずに惰性駆動を止めるための保険
        # (estop/HOLD専用。WASD運転側は_cancel_active_goal()を直接使う)。
        self._cmd_vel_pub.publish(Twist())
        self._stop_burst_until = self.get_clock().now() + Duration(
            seconds=self._stop_burst_seconds
        )
        self._cancel_active_goal(reason)

    def _cancel_active_goal(self, reason: str = "手動運転検知") -> None:
        # 先回し旋回の最中なら、旋回を取り消し、旋回後に送る予定だった目的地も捨てる。
        self._goal_seq += 1
        spin_handle, self._spin_handle = self._spin_handle, None
        if spin_handle is not None:
            self.get_logger().info(f"{reason}、先回し旋回をキャンセルします")
            spin_handle.cancel_goal_async().add_done_callback(self._on_cancel_response)
        if not self._goal_active or self._goal_handle is None:
            return
        self.get_logger().info(f"{reason}、実行中のNav2ゴールをキャンセルします")
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._on_cancel_response)

    def _on_cancel_response(self, future) -> None:
        try:
            future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Nav2ゴールのキャンセルに失敗: {exc}")

    def _on_goal_pose(self, pose: PoseStamped) -> None:
        if self._navigate_action is None:
            return
        goal_xy = (pose.pose.position.x, pose.pose.position.y)
        if self._estop_active:
            # 緊急停止中(ハード/ソフト)は新しいゴールを受け付けない。
            self.get_logger().warn("緊急停止中のため/goal_poseを拒否しました")
            self._publish_goal_status("estop_blocked", target=goal_xy)
            return
        if self._survey_blocking:
            # アンケート回答中は新しいゴールを受け付けない(手動WASDは別経路なので妨げない)。
            self.get_logger().warn("アンケート回答中のため/goal_poseを拒否しました")
            self._publish_goal_status("survey_blocked", target=goal_xy)
            return
        if self._near_person_blocking:
            # 【2026-11-05】近接停止中は、探索など手動以外の出し元の目的地を受け付けない(手動運転=地図クリックは除く)。
            self.get_logger().warn("近接停止中のため/goal_poseを拒否しました")
            self._publish_goal_status("near_person_blocked", target=goal_xy)
            return
        if self._goal_outside_boundary(pose):
            self.get_logger().warn("走行可能範囲の外のため/goal_poseを拒否しました")
            self._publish_goal_status("outside_boundary", target=goal_xy)
            return
        blocked = getattr(self, "_plan_blocked_xy", None)
        if blocked is not None and time.monotonic() < getattr(self, "_plan_blocked_until", 0.0):
            if math.hypot(goal_xy[0] - blocked[0], goal_xy[1] - blocked[1]) <= 0.15:
                self._publish_goal_status("path_outside_boundary", target=goal_xy)
                return
        self._current_goal_xy = goal_xy
        # 【2026-08-06修正】wait_for_server()はブロッキングで、rclpyの既定
        # SingleThreadedExecutorを最大server_wait_seconds秒止めてしまい、その間
        # 20Hzのゼロ速度バーストtimer等すべての処理が進まなくなっていた。
        # server_is_ready()は非ブロッキングな確認のみ行う。
        if not self._client.server_is_ready():
            self.get_logger().warn("Nav2 action server is not available yet")
            self._publish_goal_status("server_unavailable", target=goal_xy)
            return

        # 【2026-10-22】新しい目的地を受け付けたので、直前の取り消しのゼロ速度バーストは打ち切る(新しい動きとゼロが/cmd_velで混ざらない)
        self._stop_burst_until = None
        # 新しい目的地が来たら、旋回中だった古い目的地は捨てる。
        self._goal_seq += 1
        seq = self._goal_seq
        old_spin, self._spin_handle = self._spin_handle, None
        if old_spin is not None:
            old_spin.cancel_goal_async().add_done_callback(self._on_cancel_response)

        delta = self._pre_rotation_for(pose)
        if delta is not None:
            # 【2026-10-22】走行中のゴールが残ると、DWBの指令と旋回の指令が混ざって旋回が遅れる/不安定になる
            if self._goal_active and self._goal_handle is not None:
                self._goal_handle.cancel_goal_async().add_done_callback(self._on_cancel_response)
            self.get_logger().info(
                f"目的地が背後寄り(相対{math.degrees(delta):.0f}°)のため、先にその場で旋回します"
            )
            self._rotating_hint_until = time.monotonic() + 1.0  # Spinを送ってから受理されるまでも、回頭中として扱う
            spin_goal = self._spin_action.Goal()
            spin_goal.target_yaw = float(delta)
            spin_goal.time_allowance = Duration(seconds=self._pre_rotate_allowance).to_msg()
            fut = self._spin_client.send_goal_async(spin_goal)
            fut.add_done_callback(
                lambda f, pose=pose, seq=seq: self._on_spin_response(f, pose, seq)
            )
            return
        self._send_navigate(pose)

    def _on_robot_pose(self, msg) -> None:
        if self._pose_cache is not None:
            self._pose_cache.update(msg.data)

    def _base_pose(self):
        """(x, y, yaw, 古さ[秒])。/nokosahen/robot_poseが一度も来ていない/map→base_linkが取れていなければNone。"""
        if self._pose_cache is None:
            return None
        return self._pose_cache.base(self.get_clock().now().nanoseconds / 1e9)

    def _pre_rotation_for(self, pose: PoseStamped) -> Optional[float]:
        """先回し旋回が必要なら相対角度を返す。条件が揃わない/位置が不明ならNone(従来どおり)。"""
        if not self._pre_rotate_enabled or self._spin_client is None or self._pose_cache is None:
            return None
        if not self._spin_client.server_is_ready():
            self.get_logger().warn("先回し旋回の判断を省きます(/spinのアクションサーバーが準備できていません)")
            return None
        frame = str(getattr(pose.header, "frame_id", "") or "")
        if frame not in ("map", ""):
            return None
        bp = self._base_pose()
        if bp is None:
            # 黙って旋回を省くと、背後の目的地が「動かない」原因に気づけない(2026-10-19: ログを出す)
            self.get_logger().warn(f"先回し旋回の判断を省きます(map→base_linkの座標変換を取得できません: {ROBOT_POSE_TOPIC}が未受信、またはpose_relayがTFを取れていません)")
            return None
        rx, ry, yaw, age = bp
        # 古い位置では向きを誤るので、旋回を省く(負荷で座標変換が遅れているときに起きる)。age は座標変換のstampからの古さ。
        if age > 1.5:  # 【2026-10-22】0.5 -> 1.5秒(負荷時に旋回が静かに省かれて、背後の目的地が動かなかった)
            self.get_logger().warn(
                f"先回し旋回の判断を省きます(座標変換が{age:.2f}秒古い。CPU負荷が高い可能性)。背後の目的地は旋回なしでNav2へ送ります"
            )
            return None
        delta = pre_rotation_angle(
            rx, ry, yaw, pose.pose.position.x, pose.pose.position.y,
            self._pre_rotate_threshold, self._pre_rotate_min_dist,
        )
        # 【2026-10-26】壁際で回ると角が壁に当たるときは、旋回を始めない。Spinは衝突監視に止められて進まず、20秒(time_allowance)
        # 待ってから代替の回頭も「角が壁に当たる」で中止し、やっとNav2へ送られていた(実機:目的地を指定しても20秒以上経路計算が始まらない)。
        if delta is not None and self._rotation_would_collide(math.copysign(BASE_THETA_VEL, delta)):
            self.get_logger().warn("壁が近く、回ると角が当たるため、先回し旋回を省いてすぐNav2へ送ります")
            return None
        return delta

    def _on_spin_response(self, future, pose: PoseStamped, seq: int) -> None:
        if seq != self._goal_seq:
            # 旋回の受理前に取り消された。受理されていれば止める。
            try:
                h = future.result()
                if h is not None and h.accepted:
                    h.cancel_goal_async().add_done_callback(self._on_cancel_response)
            except Exception:  # noqa: BLE001
                pass
            return
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"先回し旋回の送信に失敗、そのまま走行します: {exc}")
            self._send_navigate(pose)
            return
        if not handle.accepted:
            self.get_logger().warn("先回し旋回が受け付けられず、そのまま走行します")
            self._send_navigate(pose)
            return
        self._spin_handle = handle
        if self._estop_active or self._survey_blocking:
            self._trigger_cancel()
            return
        handle.get_result_async().add_done_callback(
            lambda f, pose=pose, seq=seq: self._on_spin_result(f, pose, seq)
        )

    def _on_spin_result(self, future, pose: PoseStamped, seq: int) -> None:
        if seq != self._goal_seq:
            return  # 取り消し済み・新しい目的地に差し替え済み
        self._spin_handle = None
        status = 0
        try:
            status = int(future.result().status)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"先回し旋回の結果取得に失敗: {exc}")
        if self._estop_active or self._survey_blocking:
            return
        if status != 4:  # SUCCEEDED以外(Nav2のSpinが衝突チェックで拒否した等)
            # 【2026-10-22】後退できないDWBに任せると動かないので、速度指令(衝突監視を通る)で向きだけ合わせてから走る
            self.get_logger().warn(f"先回し旋回が完了せず(status {status})、代わりに速度指令で回頭します")
            self._begin_fallback_rotation(pose, seq)
            return
        self._send_navigate(pose)

    def _on_scan_for_sim(self, msg) -> None:
        self._scan_msg = msg
        self._scan_xy_time = time.monotonic()

    def _scan_points_xy(self) -> list:
        msg = self._scan_msg
        pts = []
        if msg is None:
            return pts
        for i, r in enumerate(msg.ranges):
            if r is None or not math.isfinite(r) or r < max(msg.range_min, 0.05) or r > 1.0:
                continue
            a = msg.angle_min + i * msg.angle_increment
            pts.append((r * math.cos(a), r * math.sin(a)))
        return pts

    def _rotation_would_collide(self, w: float) -> bool:
        """その場でw[rad/s]で1秒回ると、機体の角が壁(LiDARの点)に当たるか。LiDARが古い/無いときは判断できないのでFalse(従来どおり)。"""
        if self._scan_xy_time is None or time.monotonic() - self._scan_xy_time > 1.0:
            return False
        return first_collision_time(self._scan_points_xy(), 0.0, w, horizon=1.0) is not None

    def _heading_error(self, pose: PoseStamped) -> Optional[float]:
        """目的地までの方位と機体の向きの差[rad]。位置が取れなければNone。"""
        bp = self._base_pose()
        if bp is None or bp[3] > 1.5:  # 位置不明/古い(2026-10-26: 古さの確認を追加)なら判断できない=None(呼び出し側は回頭をやめてNav2へ送る)
            return None
        rx, ry, yaw, _age = bp
        dx, dy = pose.pose.position.x - rx, pose.pose.position.y - ry
        if not all(math.isfinite(v) for v in (dx, dy, yaw)):
            return None
        return wrap_angle(math.atan2(dy, dx) - yaw)

    def _on_motion_grant(self, msg) -> None:
        try:
            data = loads(msg.data)
            self._grant_rotate = bool(data.get("allow", {}).get("router_rotate", False))
            owner = data.get("owner")
            self._grant_owner = owner if isinstance(owner, str) else None
        except (ValueError, TypeError, AttributeError):
            return
        self._grant_time = time.monotonic()

    def _rotate_granted(self) -> bool:
        if self._grant_time is None:
            return True
        if time.monotonic() - self._grant_time > 1.0:
            return False
        return bool(self._grant_rotate)

    def _publish_rotating(self) -> None:
        rotating = self._spin_handle is not None or self._fallback is not None or time.monotonic() < self._rotating_hint_until
        if rotating != self._rotating_last:
            self._rotating_last = rotating
            self._rotating_pub.publish(Bool(data=bool(rotating)))

    def _begin_fallback_rotation(self, pose: PoseStamped, seq: int) -> None:
        self._fallback = {"pose": pose, "seq": seq, "deadline": time.monotonic() + self._fallback_rotate_sec}

    def _fallback_tick(self) -> None:
        fb = self._fallback
        if fb is None:
            return
        if fb["seq"] != self._goal_seq or self._estop_active or self._survey_blocking:
            self._fallback = None  # 取り消し・新しい目的地・緊急停止
            return
        err = self._heading_error(fb["pose"])
        if not self._rotate_granted():
            # 調停ノードが許可しない(他の動きが優先)。回頭はやめる。
            owner = self._grant_owner if self._grant_time is not None and time.monotonic() - self._grant_time <= 1.0 else None
            self._fallback = None
            self._nav_cmd_pub.publish(Twist())
            if discard_goal_on_revoke(owner):
                # 【2026-10-24】緊急停止・手動運転・近接停止が取り消した: 目的地は送らず捨てる(世代を進めて、受理待ちの古い通知も無効にする)
                self._goal_seq += 1
                self.get_logger().warn(f"代替の回頭は、調停ノードの許可が取り消された(持ち主 {owner})ため中止し、目的地は送らず破棄します")
                return
            # それ以外(nav2/wall_escape等): そのままNav2へ(Nav2の衝突監視が守る)
            self.get_logger().warn("代替の回頭は、調停ノードの許可が出ないため中止してNav2へ送ります")
            self._send_navigate(fb["pose"])
            return
        if err is None or abs(err) < math.radians(30.0) or time.monotonic() >= fb["deadline"]:
            self._fallback = None
            self._nav_cmd_pub.publish(Twist())
            self._send_navigate(fb["pose"])
            return
        cmd = Twist()
        cmd.angular.z = max(-BASE_THETA_VEL, min(BASE_THETA_VEL, 1.5 * err))  # 通常の旋回の速さ(2026-10-23。0.9 rad/sだった)
        if 0.0 < abs(cmd.angular.z) < 0.3:
            cmd.angular.z = math.copysign(0.3, cmd.angular.z)
        if self._rotation_would_collide(cmd.angular.z):
            # 【2026-10-22】回ると角が壁に当たる(先読み)。回頭はやめて、そのままNav2へ(壁際はwall_escapeが離れる)
            self.get_logger().warn("代替の回頭は、角が壁に当たるため中止してNav2へ送ります")
            self._fallback = None
            self._nav_cmd_pub.publish(Twist())
            self._send_navigate(fb["pose"])
            return
        self._nav_cmd_pub.publish(cmd)

    def _send_navigate(self, pose: PoseStamped) -> None:
        goal_xy = (pose.pose.position.x, pose.pose.position.y)
        goal = self._navigate_action.Goal()
        goal.pose = pose
        self._nav_pose = pose  # 迂回の処理が、中止になった目的地を送り直すために覚えておく
        seq = self._goal_seq  # 送った時点の世代。受理を待つ間に取り消し・新しい目的地が来たかを、受理のときに判定する
        future = self._client.send_goal_async(goal)
        future.add_done_callback(
            lambda future, goal_xy=goal_xy, seq=seq: self._on_goal_response(future, goal_xy, seq)
        )

    def _on_goal_response(self, future, goal_xy: Tuple[float, float], seq: Optional[int] = None) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            # 【2026-08-06追加】future.result()が例外を投げるケース
            # (アクションサーバーとの通信断等)を未処理のままexecutorに
            # 伝播させていた。
            self.get_logger().warn(f"Nav2ゴールの送信に失敗: {exc}")
            self._publish_goal_status("send_failed", target=goal_xy)
            self._detour_send_error("send_failed")
            return
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected goal")
            self._publish_goal_status("rejected", target=goal_xy)
            self._detour_send_error("rejected")
            return
        if seq is not None and seq != self._goal_seq:
            # 【2026-10-21】送信してから受理されるまでの間に、取り消し(緊急停止・手動運転・探索の停止・新しい目的地)が来た。
            # その時点では_goal_activeがまだFalseで、取り消す対象が無かったため、あとから受理されたこの目的地がそのまま走ってしまう
            # (CPU負荷で受理待ちが数百msに伸びると起きる)。受理されたハンドルを、すぐ取り消す。
            self.get_logger().info("受理待ちの間に取り消しが来たため、受理されたNav2ゴールをすぐキャンセルします")
            goal_handle.cancel_goal_async().add_done_callback(self._on_cancel_response)
            return
        self._goal_handle = goal_handle
        self._goal_active = True
        self._goal_xy = goal_xy
        self._active_goal_seq = seq if seq is not None else self._goal_seq  # 受理した時点の世代(あとで取り消しが来たかの判定用)
        if self._estop_active or self._survey_blocking:
            # 送信〜受理の間に緊急停止(またはアンケート回答開始)が入った場合、受理されたゴールが
            # そのまま走り出さないよう即座にキャンセルする。
            self._trigger_cancel()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, goal_handle=goal_handle, goal_xy=goal_xy: self._on_result(
                future, goal_handle, goal_xy
            )
        )

    def _on_result(self, future, goal_handle, goal_xy: Tuple[float, float]) -> None:
        # 【2026-07-17修正】ゴールが差し替えられた(behavior_directorの2秒毎の
        # 再送等でpreemptionが起きた)後に、古いゴールの結果通知がここへ届いても
        # self._goal_active/_goal_handleを巻き戻してはいけない。巻き戻すと
        # 「今まさに実行中の新しいゴール」が存在するのに_goal_active=Falseに
        # なってしまい、直後に緊急停止を押しても_trigger_cancel()の
        # `if not self._goal_active` で早期returnしてキャンセルできない
        # (「緊急停止を押しても止まらない」の再発要因だった)。
        if goal_handle is not self._goal_handle:
            return
        self._goal_active = False
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            # 結果取得の失敗(通信断等)を未処理のままexecutorへ伝播させない。
            self.get_logger().warn(f"Nav2ゴールの結果取得に失敗: {exc}")
            self._publish_goal_status("result_failed", target=goal_xy)
            self._detour_send_error("result_failed")
            return
        if self._detour_handle_result(int(result.status), goal_xy):
            return  # 迂回の処理が引き継いだ(中止の通知は、迂回の処理の結果が出てから)
        self.get_logger().info(f"Nav2 goal finished with status {result.status}")
        self._publish_goal_status("finished", status=int(result.status), target=goal_xy)

    def _publish_goal_status(
        self, event: str, status: int = 0, target: Optional[Tuple[float, float]] = None
    ) -> None:
        payload = {"event": event, "status": status}
        if target is not None:
            payload["target_x"] = target[0]
            payload["target_y"] = target[1]
        msg = String()
        msg.data = dumps(payload)
        self._goal_status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = Nav2GoalRouter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

