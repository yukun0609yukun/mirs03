"""3つのダッシュボード(Stage1/Stage2/完成版)で共有するバックエンド部品。

各ダッシュボードノードは、表示したいトピックだけ購読して`state_getter`を
実装し、この共通部品(WASD操作・ソフト緊急停止・地図クリック運転)と
組み合わせて使う。HTML自体は各ノードが個別に持つ(要件により意図的に
共有テンプレート化していない)。
"""
from __future__ import annotations

import base64
import csv
import json
import logging
import math
import os
import re
import signal
import subprocess
import tempfile
import threading
import time
from collections import deque
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple
from urllib.parse import parse_qs, urlparse

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray, String

try:
    # 【2026-10-06】behavior_rulesの再読み込みトリガー(Empty)。無い環境(テスト用スタブ等)でも
    # このモジュール自体は読み込めるようにする(その場合は再読み込み通知だけ送れない)。
    from std_msgs.msg import Empty
except Exception:  # pragma: no cover - 環境依存
    Empty = None  # type: ignore

try:
    # 【2026-10-07】位置合わせ機能(/initialpose・/amcl_pose)。無い環境(テスト用スタブ等)でも読み込めるように。
    from geometry_msgs.msg import PoseWithCovarianceStamped
except Exception:  # pragma: no cover - 環境依存
    PoseWithCovarianceStamped = None  # type: ignore

try:
    from std_srvs.srv import Empty as EmptySrv  # AMCLの/reinitialize_global_localization
except Exception:  # pragma: no cover - 環境依存
    EmptySrv = None  # type: ignore

from . import roles as _roles
from .accounts import AccountStore, LoginLimiter
from .accounts_ui import inject_role_ui
from .assets import LOGO_DATA_URI
from .system_shutdown import SystemShutdownController
from .system_shutdown import parse_request as system_shutdown_parse_request
from .ephemeral_overrides import (
    JOURNALED_ITEMS,
    EphemeralOverrides,
    build_restorers,
    items_for as _eph_items_for,
    reset_remote,
    start_startup_reset,
)
from .nav_layout import NK_FIELD_JS
from .behavior_rules import make_clock as _rules_make_clock
from .behavior_rules_store import (
    MAX_BODY_BYTES as RULES_MAX_BODY_BYTES,
    RulesFileStore,
    build_preview as build_rules_preview,
    load_example as load_rules_example,
    parse_json_strict,
)
from .behavior_tree_store import BehaviorTreeController
from .schedule_controller import ScheduleController
from .behavior_tree_store import parse_json_strict as parse_tree_json_strict
from .behavior_tree_store import MAX_BODY_BYTES as TREE_MAX_BODY_BYTES
from .bgm_proxy import BgmProxy
from .voice_master_proxy import VoiceMasterProxy
from .voice_levels_proxy import VoiceLevelsProxy
from .announce_events_proxy import AnnounceEventsController
from .pr_intro import PrIntroController
from .survey_admin_proxy import SurveyAdminProxy
from . import display_routes as _display_routes
from .display_page import build_display_html as _build_display_html, build_display_login_html as _build_display_login_html, build_id as _display_build_id
from .ocr_admin_proxy import OcrAdminProxy
from .consumption_log import ConsumptionLog
from .detection_log import DetectionLog
from .engagement_log import EngagementLog
from .map_orientation import (
    BOUNDARY_OVERRIDE_TOPIC,
    BOUNDARY_OVERRIDE_TTL_SEC,
    BoundaryError,
    boundary_state,
    disabled_state,
    estimate_wall_angle_deg,
    get_view_store,
    load_boundary_state,
    parse_boundary_override,
    parse_boundary_request,
    rect_from_state,
    state_allows_point,
    view_angle_from_two_points,
    view_angle_from_wall_deg,
)
from .map_pins import (
    PinError,
    PinStore,
    PINS_TOPIC,
    get_pin_store,
    normalize_deg as normalize_pin_deg,
    pins_payload,
    resolve_pin,
    PIN_REASON_TEXT,
    parse_pose as parse_pin_pose,
    GOAL_SNAP_MAX_SHIFT_M,
    snap_goal_to_free,
    yaw_deg_to_quaternion,
)
from .nav_speed import (
    AUTO_ANGULAR_MAX_RADPS,
    AUTO_LINEAR_MAX_MPS,
    MANUAL_ANGULAR_MAX_RADPS,
    MANUAL_ANGULAR_MIN_RADPS,
    MANUAL_LINEAR_MAX_MPS,
    MANUAL_LINEAR_MIN_MPS,
    NAV_SPEED_CONFIRM_ABOVE,
    NAV_SPEED_MAX,
    NAV_SPEED_MIN,
    NavSpeedController,
    manual_confirm_needed,
    manual_speed_limits,
    normalize_nav_speed,
    precheck_nav_request,
)
from .localization_assist import (
    QUALITY_LABELS,
    PoseRequestError,
    classify_localization_quality,
    initial_pose_covariance,
    parse_initial_pose_request,
    point_in_map_bounds,
    quaternion_to_yaw,
    yaw_to_quaternion,
)
from .http_auth import (
    SessionStore,
    check_login_credentials,
    clear_session_cookie_header,
    get_session_token,
    session_cookie_header,
)
from .step_response_analysis import compute_step_response_metrics
from .utterance_log import UtteranceLogReader
from .voice_settings_controller import VoiceSettingsController

# 【2026-09-30追加】手動/自動の直進速度の共通上限[m/s]。
# ESP32ファームは1制御周期(20ms)あたりの車輪ティック増分を
# MAX_TICK_DELTA_PER_CONTROL=400でクランプするため、これを超える速度指令は
# 黙って飽和して実際には出ない。400ティック/20ms=20000ティック/s、
# COUNTS_PER_REV=4096・WHEEL_RADIUS=0.03999mから
# 20000/4096*2*pi*0.03999 = 約1.23 m/s が物理的な頭打ち。そこに余裕を持たせた値。
# UIで入力できる値・サーバーが受け付ける値を全てこれ以下に揃える(behavior_director.pyの
# MAX_DRIVE_LINEAR_MPSと同じ値にすること。UIのinput max属性・注記も1.1)。
# 【2026-10-10】これは「自律側(director自動・Nav2)」の上限のまま。手動走行の上限は別で
# nav_speed.MANUAL_LINEAR_MAX_MPS(1.2)/MANUAL_ANGULAR_MAX_RADPS(4.0)に引き上げた。
MAX_DRIVE_LINEAR_MPS = AUTO_LINEAR_MAX_MPS

try:
    # 【注意】pathlib.Pathを既に`from pathlib import Path`でimportしているため、
    # nav_msgs.msg.Path(経路メッセージ型)は必ず別名でimportすること
    # (同名で上書きすると、ファイルパス処理をしている全箇所が壊れる)。
    from nav_msgs.msg import OccupancyGrid
    from nav_msgs.msg import Path as NavPath
except Exception:  # pragma: no cover - 環境依存
    OccupancyGrid = None  # type: ignore
    NavPath = None  # type: ignore

try:
    from sensor_msgs.msg import CompressedImage, LaserScan
except Exception:  # pragma: no cover - 環境依存
    CompressedImage = None  # type: ignore
    LaserScan = None  # type: ignore

try:
    from tf2_ros import Buffer, TransformListener
except Exception:  # pragma: no cover - 環境依存
    Buffer = None  # type: ignore
    TransformListener = None  # type: ignore


# PIDゲインの上限(負・NaN・無限大・これを超える値は400。2026-10-21)と、立入禁止エリアの座標の上限[m]
PID_GAIN_MAX = 200.0
KEEPOUT_COORD_MAX_M = 10000.0

# 【2026-07-17追加】TL(フル操作)とrestricted_account(制限付き)が同時にログインして
# いる状況で、両者が同時に運転操作した場合にTLを優先させるための優先度。
# ログイン自体はSingleControllerGate側でアカウント別に共存を許可しており、
# ここはコマンドレベル(実際に駆動指令が競合した瞬間だけ)の優先度。
# 【2026-09-30追加】ダッシュボード用に間引いた低頻度オドメトリ(wheel_odom_node
# が/odomとは別に10Hzで配信)。/odom(約50Hz)を直接購読するとrclpyの受信待ち
# 機構だけでCPUを食うため、表示・ステップ応答テスト用にはこちらを使う。
DASHBOARD_ODOM_TOPIC = "/nokosahen/odom_ui"


class LazySubscription:
    """「見ている人がいる間だけ」購読するためのヘルパー。

    LiDAR(10Hz)・カメラプレビュー(約12Hz)のように高頻度で流れるトピックは、
    購読しているだけでメッセージが届くたびにrclpyのexecutorが回り、閲覧者が
    誰もいなくてもCPUを消費する(2026-09-30、py-spyで実測)。touch()が呼ばれた
    時に購読を開始し、idle_timeout_sec秒間touch()が無ければ購読を解除する。
    購読開始直後の数百msはまだデータが無い(None)ので、呼び出し側は
    「まだ届いていない」を許容すること(ダッシュボードは繰り返しポーリングする)。
    購読解除時はon_unsubscribeを呼ぶ。呼び出し側は保持している最新値をそこで
    Noneに戻すこと(解除中は更新されないので、残すと再度開いた時に古い
    LiDAR/カメラ画像が「今の映像」として表示されてしまう)。
    """

    def __init__(self, node, msg_type, topic: str, callback, depth: int = 2,
                 idle_timeout_sec: float = 15.0, on_unsubscribe=None) -> None:
        self._node = node
        self._msg_type = msg_type
        self._topic = topic
        self._callback = callback
        self._depth = depth
        self._idle_timeout_sec = idle_timeout_sec
        self._on_unsubscribe = on_unsubscribe
        self._sub = None
        self._last_touch = 0.0
        self._lock = threading.Lock()
        node.create_timer(5.0, self._idle_check)

    def touch(self) -> None:
        with self._lock:
            self._last_touch = time.monotonic()
            if self._sub is None:
                self._sub = self._node.create_subscription(
                    self._msg_type, self._topic, self._callback, self._depth
                )

    def _idle_check(self) -> None:
        with self._lock:
            if self._sub is None:
                return
            if time.monotonic() - self._last_touch > self._idle_timeout_sec:
                self._node.destroy_subscription(self._sub)
                self._sub = None
                if self._on_unsubscribe is not None:
                    try:
                        self._on_unsubscribe()
                    except Exception:  # noqa: BLE001
                        pass

    @property
    def active(self) -> bool:
        with self._lock:
            return self._sub is not None


TELEOP_PRIORITY_FULL = 2  # TL等、速度設定も含むフル操作アカウント
TELEOP_PRIORITY_RESTRICTED = 1  # restricted_account等、運転+緊急停止のみの制限付きアカウント

# 【2026-11-06追加】手動運転の優先度はロールで決める。admin(TL等)だけがFULL。operator(班員の弱いアカウント)・restricted(restricted_account)は
# RESTRICTED=TLが直近運転中(_priority_hold_sec以内)なら指令を破棄される。viewer(guest)は /api/teleop 自体が403で、ここへ来ない。
# 安全上の前提: 手動運転は人が目で見て操作する。operatorの運転も、緊急停止(中は動かない)・速度上限(manual_linear/angular。
# 速度設定は管理者専用)・watchdog(指令が途切れたら停止)・人避け(manual_person_guard)に従う。
def teleop_priority_for_role(role: str) -> int:
    return TELEOP_PRIORITY_FULL if role == _roles.ROLE_ADMIN else TELEOP_PRIORITY_RESTRICTED


_TAB_ID_INVALID_RE = re.compile(r"[^A-Za-z0-9_-]")
_TELEOP_SEQ_MAX = 2 ** 53
_TELEOP_SEQ_TRACKED_MAX = 256


def sanitize_tab_id(value: Any) -> Optional[str]:
    """ページ固有tab_idを[A-Za-z0-9_-]{1,16}へ正規化する。無効/未指定はNone(従来動作)。"""
    if not isinstance(value, str):
        return None
    cleaned = _TAB_ID_INVALID_RE.sub("", value)[:16]
    return cleaned or None


def build_teleop_source(account: str, ip: str, tab_id: Any = None) -> str:
    """`account@IP`(tab_idがあれば`account@IP#tab_id`)の運転元識別子を作る。"""
    tab = sanitize_tab_id(tab_id)
    base = f"{account}@{ip}"
    return f"{base}#{tab}" if tab else base


def parse_teleop_seq(value: Any) -> Optional[int]:
    """/api/teleopのseqを非負整数へ。不正/未指定(旧クライアント)はNone。"""
    if isinstance(value, bool):
        return None
    if isinstance(value, float):
        if not math.isfinite(value) or value != int(value):
            return None
        value = int(value)
    if not isinstance(value, int) or value < 0 or value > _TELEOP_SEQ_MAX:
        return None
    return value


# 【2026-10-24追加】手動運転の人避け(manual_person_guard、docs/manual_person_guard.md)。
# /nokosahen/near_person_stop(person_target_detectorが、スキャンを処理するたびに出す=変化時だけではなく継続的に来る)が
# この秒数以上届かないときは「人物検出が動いていない(不明)」として扱う。
PERSON_GUARD_STALE_SEC = 3.0
# 人が近い間の旋回の角速度の倍率。前進は止めるが、後退・旋回は許す(人から離れられるように)。ただし機体は52cm角で、
# その場旋回でも角が人の脚に当たりうるため、衝突の勢いを抑える目的で角速度を半分にする(完全に止めると、向きを変えて離れられない)。
PERSON_GUARD_TURN_SCALE = 0.5
PERSON_GUARD_NO_DETECTOR_MSG = "人物検出が動いていないため人避けは効きません"


def person_guard_decision(enabled: bool, near_active: bool, age_sec: Optional[float]) -> Dict[str, Any]:
    """手動運転の人避けの判定(純粋関数、ROS不要)。

    age_sec: 最後に/nokosahen/near_person_stopを受信してからの秒数(未受信=None)。
    - detector_alive: age_secがPERSON_GUARD_STALE_SEC以内(受信できている)。
    - blocked: 人避けONで、人物検出が生きていて、かつ人が近い(true)。このとき前進をゼロにし、旋回は半分にする。
    - warning: 人避けONなのに人物検出が動いていないときの警告(前進は許す=カメラ・検出ノードなしの構成でも手動運転できるように)。
    """
    alive = age_sec is not None and age_sec >= 0.0 and age_sec <= PERSON_GUARD_STALE_SEC
    blocked = bool(enabled) and alive and bool(near_active)
    warning = PERSON_GUARD_NO_DETECTOR_MSG if (enabled and not alive) else ""
    return {"detector_alive": alive, "blocked": blocked, "warning": warning}


def apply_person_guard(forward: float, turn: float, blocked: bool) -> Tuple[float, float, bool]:
    """blockedのとき、前進成分(forward>0)をゼロにし、旋回を半分にする。戻り値は(forward, turn, 前進を止めたか)。
    blockedでなければ入力をそのまま返す(OFFの既定動作と完全に同じ)。"""
    if not blocked:
        return forward, turn, False
    stopped = forward > 0.0
    return (0.0 if stopped else forward), turn * PERSON_GUARD_TURN_SCALE, stopped


class TeleopController:
    """WASD運転とソフト緊急停止をまとめる。

    - `handle_drive()`が呼ばれるたびに`/nokosahen/manual_override`をtrueに
      してから`/cmd_vel`へ速度指令を出す(behavior_director側は
      manual_override中は自律のgoal/cmd_velを止める設計)。
    - `watchdog_sec`以内に次の`handle_drive()`が来なければ自動でゼロ速度に
      戻し、manual_overrideも解除する(ブラウザが閉じた/通信切断した場合の
      安全策)。
    - 【2026-07-17追加】TLとrestricted_accountが同時にログインしている間、実際に
      運転コマンドが競合した瞬間だけTLを優先する(セッション自体を奪うの
      ではなく、コマンド単位の優先制御)。
    """

    def __init__(
        self,
        node,
        cmd_vel_topic: str = "/cmd_vel",
        linear_speed: float = 0.15,
        angular_speed: float = 0.6,
        watchdog_sec: float = 0.5,
        soft_estop_state_file: Optional[str] = None,
        manual_person_guard_state_file: Optional[str] = None,
    ) -> None:
        self.node = node
        # 【2026-10-24】手動運転の人避け(既定OFF)の保存先(None/空=保存しない)。ソフト緊急停止と同じ方式。docs/manual_person_guard.md
        self._manual_person_guard_state_file = (
            os.path.expanduser(manual_person_guard_state_file) if manual_person_guard_state_file else ""
        )
        self.manual_person_guard = self._load_manual_person_guard_state()
        # 直近の運転指令で、人避けにより前進をゼロにしたか(/api/teleopの応答 manual_person_guard_blocked に使う)
        self.last_guard_blocked = False
        # near_person_stopを最後に受信した時刻(time.monotonic。未受信=None)
        self._near_person_time: Optional[float] = None
        # 【2026-10-21】ソフト緊急停止の状態の保存先(None/空=保存しない)。ソフト緊急停止中に監視ループがROS一式を再起動すると、
        # 以前は「解除」の状態で立ち上がり、緊急停止のラッチが全ノードで外れていた。変更のたびに保存し、起動時に復元して、
        # 「停止中」から始める(起動後に、管理者が解除する)。意図した停止(shutdown_all.sh / stop_everything.sh)は、このファイルを消す。
        self._soft_estop_state_file = os.path.expanduser(soft_estop_state_file) if soft_estop_state_file else ""
        self.cmd_vel_pub = node.create_publisher(Twist, cmd_vel_topic, 10)
        self.manual_override_pub = node.create_publisher(Bool, "/nokosahen/manual_override", 10)
        # WASD運転が実際に始まった瞬間だけをnav2_goal_routerに知らせる専用
        # トピック。manual_overrideは地図クリックのゴール追従(goal_mode)も
        # 同じ値を再送し続けるため、そちらと兼用するとnav2_goal_router側が
        # 自分で送ったゴールを自分でキャンセルしてしまう(2026-07-17レビューで
        # 発見、nav2_goal_router.pyのdocstring参照)。
        self.wasd_active_pub = node.create_publisher(Bool, "/nokosahen/wasd_active", 10)
        self.soft_estop_pub = node.create_publisher(Bool, "/nokosahen/soft_estop", 10)
        # 起動時のパラメータ値も上限を超えないようにする(ファーム側で黙って飽和するため)。
        # 【2026-10-10】手動の上限はnav_speed.MANUAL_*(直進1.2 / 旋回4.0)。旋回も起動値を丸める。
        self.linear_speed = min(MANUAL_LINEAR_MAX_MPS, linear_speed)
        self.angular_speed = min(MANUAL_ANGULAR_MAX_RADPS, angular_speed)
        self.watchdog_sec = watchdog_sec
        self.last_cmd_time = 0.0
        self.soft_estop_engaged = self._load_soft_estop_state()
        # person_target_detectorがLiDARで「人が50cm圏内にいる」と検知した状態。
        # 【2026-07-17変更】当初はWASDの駆動指令そのものを止めていたが、実機
        # テストで「手動操作中は操作者がロボットのすぐそばにいるのが普通なのに
        # 動けなくなって不便」と判明したため、手動運転中はこの安全装置を無効化
        # する方針に変更(behavior_director.py側もMANUAL_OVERRIDE中はこの
        # チェックをしない設計にした)。ここではstatus()で状態を見えるように
        # するためだけに保持し、handle_drive()の送信は止めない。
        self.near_person_stop_active = False
        node.create_subscription(
            Bool, "/nokosahen/near_person_stop", self._on_near_person_stop, 5
        )
        # 地図クリックによるNav2ゴール追従中は、WASDのようにブラウザ側が
        # 継続的に何かを送り続けるわけではないので(1回POSTするだけ)、
        # behavior_directorのmanual_override_timeout_sec(1秒)がすぐに
        # 切れてしまい、ゴール到達前に自律ロジック(混雑時停止等)が復帰して
        # Nav2と/cmd_velを奪い合ってしまう。ゴール追従中はこのタイマーで
        # manual_overrideを送り続けて維持し、明示的にresume_autonomyが
        # 呼ばれる(または手動運転が始まる)まで維持する。
        self.goal_mode_active = False
        # TL(TELEOP_PRIORITY_FULL)が最後に実際の運転コマンド(forward/turnが
        # 非ゼロ)を送った時刻。この時刻からpriority_hold_sec以内は、それより
        # 低い優先度のアカウントの運転コマンドを破棄する。
        self._last_high_priority_drive_time = 0.0
        self._priority_hold_sec = 1.0
        # 最後に実際の運転コマンド(非ゼロ)を送った送信元(アカウント+端末)。
        # ページを開いているだけの別端末のアイドル(0,0)呼び出しが、運転中
        # ユーザーの操作を停止させてしまうのを防ぐために使う(2026-09-30)。
        self._last_drive_source: Optional[str] = None
        # 【2026-10-03追加】sourceごとの最新seq。古いseqの運転コマンド(遅れて届いた
        # 前進が最後の停止を追い越す)を破棄するため。HTTPはスレッド並行で処理される。
        self._drive_seq: Dict[str, int] = {}
        self._drive_lock = threading.RLock()
        # soft_estopの定期再送用(遅れて起動したノードが取りこぼさないように)
        self._last_soft_estop_pub_time = 0.0
        node.create_timer(0.1, self._watchdog_tick)
        node.create_timer(0.5, self._goal_keepalive_tick)

    def _watchdog_tick(self) -> None:
        # soft_estopはvolatile QoSで1回しかpublishしないと、後から起動した
        # ノード(frontier_explorer/nav2_goal_router/behavior_director)が
        # 取りこぼす。約1Hzで現在値を再送して追従させる(2026-09-30)。
        now = time.monotonic()
        if (now - self._last_soft_estop_pub_time) >= 1.0:
            self._last_soft_estop_pub_time = now
            msg = Bool()
            msg.data = self.soft_estop_engaged
            self.soft_estop_pub.publish(msg)
        if self.soft_estop_engaged:
            return
        if self.last_cmd_time and (time.monotonic() - self.last_cmd_time) > self.watchdog_sec:
            self.cmd_vel_pub.publish(Twist())
            self.last_cmd_time = 0.0
            self.publish_manual_override(False)
            self._publish_wasd_active(False)

    def _goal_keepalive_tick(self) -> None:
        if self.goal_mode_active and not self.soft_estop_engaged:
            self.publish_manual_override(True)

    def activate_goal_mode(self) -> None:
        self.goal_mode_active = True
        self.publish_manual_override(True)

    def deactivate_goal_mode(self) -> None:
        self.goal_mode_active = False

    def publish_manual_override(self, active: bool) -> None:
        msg = Bool()
        msg.data = active
        self.manual_override_pub.publish(msg)

    def _publish_wasd_active(self, active: bool) -> None:
        msg = Bool()
        msg.data = active
        self.wasd_active_pub.publish(msg)

    def publish_wasd_active(self, active: bool) -> None:
        """/nokosahen/wasd_active(motion_arbiterの「手動運転が持ち主」の入力)。走行テストなど、WASD以外の直接/cmd_vel走行も使う。"""
        self._publish_wasd_active(active)

    def is_driving(self) -> bool:
        """WASD(handle_drive)が直近watchdog_sec以内に運転入力を出しているか。走行テストの後片付けで、WASDの手動運転を消さないための判定。"""
        return bool(self.last_cmd_time) and (time.monotonic() - self.last_cmd_time) <= self.watchdog_sec

    def _on_near_person_stop(self, msg: Bool) -> None:
        self.near_person_stop_active = bool(msg.data)
        self._near_person_time = time.monotonic()

    def person_guard_state(self) -> Dict[str, Any]:
        """手動運転の人避けの現在の判定(person_guard_decisionの結果 + 設定)。"""
        t = self._near_person_time
        age = None if t is None else (time.monotonic() - t)
        d = person_guard_decision(self.manual_person_guard, self.near_person_stop_active, age)
        d["enabled"] = bool(self.manual_person_guard)
        d["near_person_stop"] = bool(self.near_person_stop_active) and d["detector_alive"]
        return d

    def set_manual_person_guard(self, enabled: bool, persist: bool = True) -> None:
        """人避けのON/OFFを切り替える。persist=Falseは、operatorの一時的な変更(保存しない=再起動で保存済みの値に戻る)。"""
        self.manual_person_guard = bool(enabled)
        if not enabled:
            self.last_guard_blocked = False
        if persist:
            self._save_manual_person_guard_state(self.manual_person_guard)

    def _load_manual_person_guard_state(self) -> bool:
        """保存された状態を読む。保存先が無い・ファイルが無い・壊れている場合は False(OFF)。"""
        path = self._manual_person_guard_state_file
        if not path:
            return False
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, ValueError):
            return False
        return isinstance(data, dict) and data.get("enabled") is True

    def _save_manual_person_guard_state(self, enabled: bool) -> None:
        """状態を保存する(原子的に。失敗しても切替そのものは止めない)。"""
        path = self._manual_person_guard_state_file
        if not path:
            return
        try:
            tmp = path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump({"enabled": bool(enabled), "at": time.time()}, f)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, path)
        except OSError:
            pass

    def handle_drive(
        self,
        forward: float,
        turn: float,
        priority: int = TELEOP_PRIORITY_FULL,
        source: Optional[str] = None,
        seq: Optional[int] = None,
    ) -> bool:
        """運転コマンドを受け付ける。

        【2026-10-03追加】seq(クライアントの単調増加カウンタ)が同一sourceの
        既出の最大値より小さい場合は、遅れて届いた古いコマンドとして何もせず
        True(エラー表示を出さない)で破棄する。seq未指定(旧クライアント)は
        従来動作。停止(0,0)もseqが古ければ破棄されるが、停止は常に最新seqを
        持つため後着の古い前進には負けない。"""
        with self._drive_lock:
            if seq is not None and source is not None:
                last = self._drive_seq.get(source)
                if last is not None and seq < last:
                    return True
                self._drive_seq.pop(source, None)
                self._drive_seq[source] = seq if last is None else max(last, seq)
                while len(self._drive_seq) > _TELEOP_SEQ_TRACKED_MAX:
                    self._drive_seq.pop(next(iter(self._drive_seq)))
            return self._handle_drive_impl(forward, turn, priority, source)

    def _handle_drive_impl(
        self,
        forward: float,
        turn: float,
        priority: int = TELEOP_PRIORITY_FULL,
        source: Optional[str] = None,
    ) -> bool:
        """運転コマンドを受け付ける(handle_driveの本体)。戻り値はTrue=受理、False=優先度負けで破棄。

        破棄(False)は「TLが直近priority_hold_sec以内に実際の運転コマンドを
        送っている間、それより低い優先度のアカウントのコマンドを無視する」
        場合のみ発生する(2026-07-17追加)。アイドル呼び出し(forward/turnが
        両方0)は元々何もしないため優先度チェックの対象外にしており、TLの
        ダッシュボードタブが開いているだけ(操作はしていない)でrestricted_accountを
        永久にブロックすることは無い。

        sourceは呼び出し元の識別子(アカウント+端末等)。走行中に「別のsource」
        からアイドル(0,0)呼び出しが来ても停止処理は行わず無視する(別端末で
        アイドル画面を開いているだけで、運転中のユーザーの操作が止まるのを
        防ぐ)。停止は、最後に運転したsource自身のアイドル呼び出しか、
        watchdog_sec経過後の_watchdog_tick()が担う。source=Noneは従来動作。
        """
        # ダッシュボードのJSは100ms毎に無条件でここを呼ぶ(ページを開いている
        # だけで、キーを押していなくても呼ばれる)。forward/turnが両方0の
        # 「アイドル状態」でもmanual_overrideをtrueにし続けると、ページを
        # 開いているだけで自律行動が永久に停止してしまう。実際に運転入力が
        # あるときだけmanual_overrideとcmd_velを送り、アイドル時は何もしない
        # (最後の運転入力からwatchdog_sec経過すれば_watchdog_tick()が
        # ゼロ速度とmanual_override=falseを送って自律運転に戻す)。
        # 【重要】estopチェックより先にこのアイドル判定を行うこと。estop中も
        # 100ms毎のアイドル呼び出し自体は来続けるため、逆順だとestop中は
        # 常にFalseを返すことになり、呼び出し側がestop解除まで誤入力と
        # 見分けられずエラー表示を出し続けてしまう。
        # 【2026-10-02追加】NaN/Infは受理しない。max(-1.0, min(1.0, nan))は
        # Pythonでは1.0(全速)になってしまうため、非有限値は不正入力として
        # ゼロ速度を出し、Falseを返す(安全側)。idle判定より先に行う
        # (nan==0.0はFalseなのでidle扱いにならず、運転側へ流れてしまう)。
        if not (math.isfinite(forward) and math.isfinite(turn)):
            self.cmd_vel_pub.publish(Twist())
            return False
        if forward == 0.0 and turn == 0.0:
            # 【2026-08-08修正】直前まで実際に走行中だった場合(ボタンを離した
            # 直後の呼び出し)は、ここで即座にゼロ速度へ戻す。以前は単に
            # 何もせずreturnするだけで、実際の停止は_watchdog_tick()が
            # watchdog_sec(既定0.5秒)経過を検知するまで行われなかったため、
            # ボタンを離してから最大0.5秒ほど惰走し続けてしまっていた
            # (「ボタンを押している間だけ動く」という画面の説明に反する)。
            # ブラウザが閉じた/通信断でこの呼び出し自体が来なくなるケースの
            # 安全策としては、_watchdog_tick()を引き続き残す。
            if self.last_cmd_time:
                if (
                    source is not None
                    and self._last_drive_source is not None
                    and source != self._last_drive_source
                ):
                    # 別端末/別アカウントのアイドル呼び出しは無視する。
                    return True
                self.cmd_vel_pub.publish(Twist())
                self.last_cmd_time = 0.0
                self.publish_manual_override(False)
                self._publish_wasd_active(False)
            return True
        if self.soft_estop_engaged:
            # 【2026-08-05修正】以前はここでTrue(受理)を返していたため、
            # 緊急停止中にWASDを押しても指令(cmd_vel/manual_override)は
            # 一切publishされないのに、ブラウザ側は"成功"と表示してしまい、
            # 「サーバーは受理しているのに実機が動かない」という紛らわしい
            # 状態になっていた(実機診断で発見)。Falseを返し、呼び出し側で
            # soft_estop_engagedを見てestop起因と判別できるようにする。
            return False

        now = time.monotonic()
        if priority >= TELEOP_PRIORITY_FULL:
            self._last_high_priority_drive_time = now
        elif (now - self._last_high_priority_drive_time) <= self._priority_hold_sec:
            # TLが直近運転中なので、下位アカウントのコマンドは破棄する。
            return False

        # WASDでの手動運転はゴール追従より優先する(操作している間、
        # ゴールキープアライブによる干渉を止める)。
        self.goal_mode_active = False
        self._last_drive_source = source
        # 【2026-10-24】手動運転の人避け(既定OFF=この分岐は何もしない)。ONで、人が近い(かつ人物検出が生きている)間は
        # 前進をゼロにし、旋回は半分にする。後退はそのまま通す。人物検出が途絶/未受信のときは前進を許す(状態に警告)。
        self.last_guard_blocked = False
        if self.manual_person_guard:
            blocked = self.person_guard_state()["blocked"]
            forward, turn, self.last_guard_blocked = apply_person_guard(forward, turn, blocked)
        self.publish_manual_override(True)
        self._publish_wasd_active(True)
        twist = Twist()
        # 【2026-10-10】入力は±1に丸め、設定値も上限内だが、最後にもう一度手動の上限で必ず丸める(多重防御)。
        twist.linear.x = max(-MANUAL_LINEAR_MAX_MPS, min(MANUAL_LINEAR_MAX_MPS, max(-1.0, min(1.0, forward)) * self.linear_speed))
        twist.angular.z = max(-MANUAL_ANGULAR_MAX_RADPS, min(MANUAL_ANGULAR_MAX_RADPS, max(-1.0, min(1.0, turn)) * self.angular_speed))
        self.cmd_vel_pub.publish(twist)
        self.last_cmd_time = time.monotonic()
        return True

    def set_max_speed(self, linear: Optional[float], angular: Optional[float]) -> None:
        """WASD手動運転の最高速度をダッシュボードから実行時に変更する。

        0以下や異常値(入力ミス)でモータが暴走しないよう範囲を制限する。
        Noneの項目は変更しない(片方だけの更新を許す)。
        """
        # 【2026-10-02追加】NaN/Infは無視する(min/maxで上限値に張り付くため)。
        if linear is not None and not math.isfinite(linear):
            linear = None
        if angular is not None and not math.isfinite(angular):
            angular = None
        if linear is not None:
            # 上限はファームの速度クランプ(約1.23 m/s)に余裕を持たせた値(手動はnav_speed.MANUAL_LINEAR_MAX_MPS)。
            self.linear_speed = max(MANUAL_LINEAR_MIN_MPS, min(MANUAL_LINEAR_MAX_MPS, linear))
        if angular is not None:
            self.angular_speed = max(MANUAL_ANGULAR_MIN_RADPS, min(MANUAL_ANGULAR_MAX_RADPS, angular))

    def _load_soft_estop_state(self) -> bool:
        """保存された状態を読む。保存先が無い・ファイルが無い・壊れている場合は False(従来どおり、解除の状態で起動)。
        保存されていた状態が「停止中」なら True(停止中のまま起動する)。"""
        path = self._soft_estop_state_file
        if not path:
            return False
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, ValueError):
            return False
        engaged = isinstance(data, dict) and data.get("engaged") is True
        if engaged:
            try:
                self.node.get_logger().warn("前回のソフト緊急停止が解除されていないため、停止中の状態で起動します(管理者が解除してください)")
            except Exception:  # noqa: BLE001
                pass
        return engaged

    def _save_soft_estop_state(self, engaged: bool) -> bool:
        """状態を保存する(原子的に。失敗しても緊急停止の処理は止めない)。

        【2026-11-05】一時ファイル名を固定の `path+".tmp"` から一意の名前(mkstemp)に変えた
        (停止と解除が同時に来ても、同じ一時ファイルに書き合って壊れない)。os.replace の失敗は
        握りつぶさずログに残す(メモリの状態とファイルが食い違ったことが分かるように)。成功したかを返す。
        """
        path = self._soft_estop_state_file
        if not path:
            return True
        tmp = None
        try:
            d = os.path.dirname(path) or "."
            fd, tmp = tempfile.mkstemp(prefix=os.path.basename(path) + ".", suffix=".tmp", dir=d)
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump({"engaged": bool(engaged), "at": time.time()}, f)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, path)
            tmp = None
            return True
        except OSError as exc:
            try:
                self.node.get_logger().error(
                    f"緊急停止の状態を保存できませんでした(メモリの状態={bool(engaged)}、ファイルは古いまま): {exc}")
            except Exception:
                pass
            return False
        finally:
            if tmp:
                try:
                    os.unlink(tmp)
                except OSError:
                    pass

    def handle_soft_estop(self, engaged: bool) -> None:
        # 【2026-11-05】全体を _drive_lock で囲む(停止と解除、手動運転との取り合いで、メモリの状態・保存ファイル・
        # publish の順序が食い違わないように。RLockなので内側の publish_manual_override 等の再取得は問題ない)。
        with self._drive_lock:
            self.soft_estop_engaged = engaged
            self._save_soft_estop_state(engaged)
            msg = Bool()
            msg.data = engaged
            self.soft_estop_pub.publish(msg)
            if engaged:
                self.goal_mode_active = False
                self.cmd_vel_pub.publish(Twist())
                self._publish_wasd_active(False)
            else:
                self.publish_manual_override(False)

    def status(self) -> dict:
        teleop_active = bool(self.last_cmd_time) and (
            time.monotonic() - self.last_cmd_time
        ) <= self.watchdog_sec
        return {
            "soft_estop_engaged": self.soft_estop_engaged,
            "teleop_active": teleop_active,
            "manual_linear_mps": self.linear_speed,
            "manual_angular_rps": self.angular_speed,
            "near_person_stop_active": self.near_person_stop_active,
            "manual_person_guard": bool(self.manual_person_guard),
            "manual_person_guard_blocked": bool(self.person_guard_state()["blocked"]),
        }


class AutoSpeedController:
    """自律移動(behavior_directorの接近/巡回時の速度)の最高速度を、ダッシュ
    ボードから実行時に変更するための橋渡し。

    behavior_directorは別プロセスなので、直接値を書き換えることはできない。
    ここでは`/nokosahen/behavior_speed_override`へpublishするだけで、実際の
    反映はbehavior_director側(該当トピックを購読)が行う。ダッシュボード表示用
    に、最後に設定した値をこちら側でも保持しておく。
    """

    def __init__(
        self,
        node,
        topic: str = "/nokosahen/behavior_speed_override",
        # 【2026-07-16確定→2026-09-30文言修正】自動運転の既定は0.75 m/s・
        # 0.75 rad/s。手動運転側の既定値(nokosahen.yamlのmanual_linear_speed等、
        # 現在1.0 m/s)とは独立した値で、「手動の半分」という関係は保証しない。
        # behavior_directorのapproach_speed_mps/approach_turn_rpsの既定値・
        # nokosahen.yamlとも必ず揃えること。
        default_linear: float = 0.75,
        default_angular: float = 0.75,
    ) -> None:
        self.node = node
        self._pub = node.create_publisher(String, topic, 10)
        self.linear_mps = default_linear
        self.angular_rps = default_angular
        # 【2026-10-10追加】ユーザーが一度でも値を設定したら、5秒ごとに再publishする。
        # 以前は1回しかpublishしておらず、behavior_directorを再起動すると既定値に戻るのに、
        # 画面は設定した値を表示し続けていた(directorの_on_speed_overrideは冪等)。
        # 未設定の間は何も出さない(directorの既定=従来動作のまま)。
        self._user_set = False
        create_timer = getattr(node, "create_timer", None)
        if create_timer is not None:
            create_timer(5.0, self._republish)

    def _publish_current(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {"linear_mps": self.linear_mps, "angular_rps": self.angular_rps}
        )
        self._pub.publish(msg)

    def _republish(self) -> None:
        if self._user_set:
            self._publish_current()

    def set_max_speed(self, linear: Optional[float], angular: Optional[float]) -> None:
        # 【2026-10-02追加】NaN/Infは無視する(min/maxで上限値に張り付くため)。
        if linear is not None and not math.isfinite(linear):
            linear = None
        if angular is not None and not math.isfinite(angular):
            angular = None
        if linear is not None:
            # 上限はファームの速度クランプ(約1.23 m/s)に余裕を持たせた共通値。
            self.linear_mps = max(0.01, min(MAX_DRIVE_LINEAR_MPS, linear))
        if angular is not None:
            self.angular_rps = max(0.01, min(AUTO_ANGULAR_MAX_RADPS, angular))
        self._user_set = True
        self._publish_current()

    def status(self) -> dict:
        return {"auto_linear_mps": self.linear_mps, "auto_angular_rps": self.angular_rps}


class PidGainsController:
    """足回りPIDゲインの実行時調整(Stage1専用)と、距離/角度・時間・方向を
    指定した簡易走行テスト(Stage1/Stage2/完成版共通)をまとめる。

    PIDゲイン調整(`/nokosahen/pid_gains`購読、ESP32ファームウェア側が即座に
    上書きし現在値を`/nokosahen/pid_gains_state`へ返す)はStage1(ESP32+
    Windows単体試験)専用の想定で、stage1_dashboard_nodeだけがゲイン編集UIを
    公開する(要件通り「ESP32+Windowsのテスト環境の時だけ」)。

    一方、走行テスト(`run_drive_test()`、距離/時間/方向を指定してオドメトリ
    ベースで停止する)は【2026-09-24】ステップ応答テスト機能追加に伴い
    Stage2/完成版ダッシュボード(実機Jetson、Nav2・behavior_directorが同時に
    動いている環境)でも使えるようlist_gainsの対象を広げた。Stage1のように
    直接`/cmd_vel`へpublishするだけでは、Nav2/behavior_directorが同時に
    ゴール追従や見回し等で/cmd_velを送ってきて競合するため、走行テスト中は
    TeleopControllerの`/nokosahen/manual_override`(WASD手動運転時と全く同じ
    仕組み)を送り続け、behavior_director/frontier_explorerを自律側から
    退避させる(behavior_director.py Line 1005 MANUAL_OVERRIDE分岐参照)。
    Stage1ではこの発行先(TeleopController.manual_override_pub)を誰も
    購読していないため無害。
    """

    MAX_TEST_DURATION_SEC = 15.0
    MAX_LINEAR_MPS = 0.5
    MAX_ANGULAR_RPS = 3.0
    # ステップ応答テストの安全上限。距離指定の走行テスト(forward/backward)は
    # これを超えるdistance_mをエラーにする(/api/drive_testハンドラ側で判定)。
    MAX_TEST_DISTANCE_M = 3.0
    # 【2026-09-24追加】回転(rotate_left/rotate_right)版ステップ応答テストの
    # 安全上限。3.0m(MAX_TEST_DISTANCE_M、部屋を横断しかねない距離)と違い、
    # その場回転は1周(360°)しても部屋の外へ飛び出すような危険は無く、
    # PID調整用のステップ応答テストとしても1周分の応答を見たいケースは
    # 正当なので、360°(=1周)を上限とする。これを超えるtarget_degをエラーに
    # する(/api/step_response/runハンドラ側で判定)。
    MAX_TEST_ROTATE_DEG = 360.0

    def __init__(self, node, teleop: "TeleopController", cmd_vel_topic: str = "/cmd_vel") -> None:
        self.node = node
        # ソフト緊急停止ボタン(TeleopController側)が押された時に、実行中の
        # 走行テストを即座に打ち切るために参照を持つ。TeleopControllerの
        # soft_estopは/cmd_velへ1回ゼロ速度をpublishするだけなので、これを
        # 見ずに走行テストが100ms毎の指令を送り続けると、次の指令で
        # 緊急停止が即座に上書きされてしまう(実際にこの不具合が発生した)。
        self._teleop = teleop
        self._gains_pub = node.create_publisher(Float32MultiArray, "/nokosahen/pid_gains", 10)
        self._cmd_vel_pub = node.create_publisher(Twist, cmd_vel_topic, 10)
        # 【2026-10-23追加】(B3)走行テスト(直接/cmd_vel)は、開始時にNav2の目的地を取り消し、/nokosahen/wasd_active=trueを出して
        # motion_arbiterの持ち主を manual にする(wall_escape・directorの直接出力・代替の回頭が同時に動かない)。終了時にfalseへ戻す。
        # パラメータ direct_test_claims_manual=false で従来動作(manual_overrideだけ)に戻せる。
        self._claims_manual = True
        try:
            node.declare_parameter("direct_test_claims_manual", True)
            self._claims_manual = bool(node.get_parameter("direct_test_claims_manual").value)
        except Exception:  # noqa: BLE001 - パラメータを持たないスタブのノードでは既定(有効)のまま
            pass
        self._cancel_goal_pub = node.create_publisher(Bool, "/nokosahen/cancel_goal", 10)
        self._current_gains: Optional[List[float]] = None
        node.create_subscription(
            Float32MultiArray, "/nokosahen/pid_gains_state", self._on_gains_state, 10
        )
        # 走行テストが「時間になるまで一定速度で送り続ける」だけだと、実際に
        # 目標距離/角度へ到達したかを見ておらず、PIDが暴れて想定より速く/遅く
        # 進んだ場合にオーバーシュートしたり届かなかったりする。オドメトリを
        # 購読しておき、目標に到達し次第(時間が余っていても)即座に停止できる
        # ようにする。
        self._latest_odom: Optional[Tuple[float, float, float]] = None  # (x, y, yaw)
        node.create_subscription(Odometry, DASHBOARD_ODOM_TOPIC, self._on_odom, 10)
        # 【2026-09-30追加】走行テスト(オーバーシュート測定)中だけ、間引いていない
        # 本物の/odom(約50Hz)を購読する。10Hzに間引いたodom_uiだと最大100ms古く、
        # 0.4m/sでは最大4cmの誤差になり、テストの目的(行き過ぎの測定)に足りない。
        # テスト以外では購読しない(LazySubscription、idle_timeout後に自動解除)。
        # 他のodom利用側(ダッシュボードのstate用等)にも同じ高頻度値を流せるよう、
        # add_odom_listener()でコールバックを登録できる。
        self._odom_listeners: List[Callable[[Any], None]] = []
        self._fast_odom_seen = False
        self._odom_fast = LazySubscription(
            node, Odometry, "/odom", self._on_odom_fast, depth=10, idle_timeout_sec=5.0
        )
        self._test_lock = threading.Lock()
        self._test_running = False
        # 【2026-09-24追加】ダッシュボード「実行」ボタン(ステップ応答テスト)用。
        # 完了した直近のテストのディレクトリ名(=StepResponseResultsControllerの
        # id)を保持し、フロントエンドが完了検知後すぐそのグラフへジャンプ
        # できるようにする。
        self._step_response_last_result_id: Optional[str] = None
        self._step_response_error: Optional[str] = None

    def _on_gains_state(self, msg: Float32MultiArray) -> None:
        self._current_gains = list(msg.data)

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._latest_odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def add_odom_listener(self, callback: Callable[[Any], None]) -> None:
        self._odom_listeners.append(callback)

    def _on_odom_fast(self, msg: Odometry) -> None:
        self._fast_odom_seen = True
        self._on_odom(msg)
        for callback in self._odom_listeners:
            callback(msg)

    def _begin_fast_odom(self, timeout_sec: float = 1.0) -> None:
        """高頻度odomの購読を始め、最初の1件が届く(=開始位置が新しい値になる)まで待つ。"""
        self._fast_odom_seen = False
        self._odom_fast.touch()
        deadline = time.monotonic() + timeout_sec
        while not self._fast_odom_seen and time.monotonic() < deadline:
            time.sleep(0.02)

    def current_gains(self) -> Optional[Dict[str, float]]:
        """前進(fwd)/後退(bwd)で別々に持てるゲイン12個を返す。

        後退すると直進からずれる(モーター/キャタピラの前後非対称性)ことが
        実機テストで分かったため、片側のゲインだけでなく前進/後退でも
        別の値を持てるようにしている。
        """
        if self._current_gains is None or len(self._current_gains) < 12:
            return None
        (
            rkp_fwd, rki_fwd, rkd_fwd, rkp_bwd, rki_bwd, rkd_bwd,
            lkp_fwd, lki_fwd, lkd_fwd, lkp_bwd, lki_bwd, lkd_bwd,
        ) = self._current_gains[:12]
        return {
            "rkp_fwd": rkp_fwd, "rki_fwd": rki_fwd, "rkd_fwd": rkd_fwd,
            "rkp_bwd": rkp_bwd, "rki_bwd": rki_bwd, "rkd_bwd": rkd_bwd,
            "lkp_fwd": lkp_fwd, "lki_fwd": lki_fwd, "lkd_fwd": lkd_fwd,
            "lkp_bwd": lkp_bwd, "lki_bwd": lki_bwd, "lkd_bwd": lkd_bwd,
        }

    def set_gains(
        self,
        rkp_fwd: float, rki_fwd: float, rkd_fwd: float,
        rkp_bwd: float, rki_bwd: float, rkd_bwd: float,
        lkp_fwd: float, lki_fwd: float, lkd_fwd: float,
        lkp_bwd: float, lki_bwd: float, lkd_bwd: float,
    ) -> None:
        msg = Float32MultiArray()
        msg.data = [
            float(rkp_fwd), float(rki_fwd), float(rkd_fwd),
            float(rkp_bwd), float(rki_bwd), float(rkd_bwd),
            float(lkp_fwd), float(lki_fwd), float(lkd_fwd),
            float(lkp_bwd), float(lki_bwd), float(lkd_bwd),
        ]
        self._gains_pub.publish(msg)

    def test_running(self) -> bool:
        with self._test_lock:
            return self._test_running

    # --- 走行テスト(直接/cmd_vel)の、手動運転としての宣言(B3) ---
    def _begin_direct_drive(self) -> None:
        """走行テストの開始時: 実行中のNav2の目的地を取り消し(取りこぼし対策で2回)、手動運転(wasd_active)を宣言する。"""
        if not self._claims_manual:
            return
        msg = Bool()
        msg.data = True
        self._cancel_goal_pub.publish(msg)
        self._cancel_goal_pub.publish(msg)
        self._teleop.publish_wasd_active(True)

    def _keep_direct_drive(self) -> None:
        """走行テスト中の毎周期: manual_override(従来どおり)と、wasd_active(arbiterの持ち主=manual)を出し続ける。"""
        self._teleop.publish_manual_override(True)
        if self._claims_manual:
            self._teleop.publish_wasd_active(True)

    def _end_direct_drive(self) -> None:
        """走行テストの終了時(finally): ゼロ速度を出し、手動運転の宣言を戻す。
        WASDが同時に運転中(直近watchdog_sec以内)なら、manual_override・wasd_activeはWASD側のもの=消さない(WASDのwatchdogが戻す)。"""
        self._cmd_vel_pub.publish(Twist())
        if self._teleop.is_driving():
            return
        self._teleop.publish_manual_override(False)
        if self._claims_manual:
            self._teleop.publish_wasd_active(False)

    def run_drive_test(
        self,
        linear_mps: float,
        angular_rps: float,
        duration_s: float,
        target_distance_m: Optional[float] = None,
        target_rotate_rad: Optional[float] = None,
    ) -> bool:
        """指定した速度で走行し、目標(距離or回転角)に到達するか指定秒数が
        経過したら停止する。終了時(例外時・緊急停止時含む)は必ず停止する。

        target_distance_m(直進/後退用)またはtarget_rotate_rad(回転用)を
        渡すと、オドメトリ(`/odom`)を見ながら実際にその量だけ動いた時点で、
        duration_sが残っていても即座に停止する(PIDが暴れて想定より速く
        進んだ場合のオーバーシュート防止)。/odomが来ていない場合は
        duration_sだけを見る従来動作にフォールバックする。

        既に別のテストが進行中、またはソフト緊急停止が作動中なら新規開始を
        拒否する(/cmd_velの競合防止・安全のため)。速度・時間は安全のため
        上限でクランプする。
        """
        if self._teleop.soft_estop_engaged:
            return False
        with self._test_lock:
            if self._test_running:
                return False
            self._test_running = True

        linear_mps = max(-self.MAX_LINEAR_MPS, min(self.MAX_LINEAR_MPS, linear_mps))
        angular_rps = max(-self.MAX_ANGULAR_RPS, min(self.MAX_ANGULAR_RPS, angular_rps))
        duration_s = max(0.1, min(self.MAX_TEST_DURATION_SEC, duration_s))
        self._begin_fast_odom()
        start_odom = self._latest_odom
        self._begin_direct_drive()

        def _target_reached() -> bool:
            if start_odom is None or self._latest_odom is None:
                return False
            x0, y0, yaw0 = start_odom
            x, y, yaw = self._latest_odom
            if target_distance_m is not None:
                traveled = math.hypot(x - x0, y - y0)
                return traveled >= abs(target_distance_m)
            if target_rotate_rad is not None:
                diff = (yaw - yaw0 + math.pi) % (2.0 * math.pi) - math.pi
                return abs(diff) >= abs(target_rotate_rad)
            return False

        def _worker() -> None:
            try:
                twist = Twist()
                twist.linear.x = linear_mps
                twist.angular.z = angular_rps
                end_time = time.monotonic() + duration_s
                while time.monotonic() < end_time:
                    self._odom_fast.touch()
                    # 緊急停止・目標到達のどちらかが成立したら、残り時間が
                    # あっても即座に打ち切る(次のsleep(0.1)を待たず検知する)。
                    if self._teleop.soft_estop_engaged or _target_reached():
                        break
                    # WASD手動運転(TeleopController.handle_drive())と同じ
                    # manual_overrideを送り続け、Stage2/完成版ではNav2・
                    # behavior_director・frontier_explorerを自律側から一時的に
                    # 退避させる(Stage1では購読者が居ないため無害)。
                    self._keep_direct_drive()
                    self._cmd_vel_pub.publish(twist)
                    time.sleep(0.1)
            finally:
                self._end_direct_drive()
                with self._test_lock:
                    self._test_running = False

        threading.Thread(target=_worker, daemon=True).start()
        return True

    def step_response_status(self) -> dict:
        return {
            "step_response_test_running": self.test_running(),
            "step_response_last_result_id": self._step_response_last_result_id,
            "step_response_error": self._step_response_error,
        }

    def run_step_response_test(
        self,
        direction: str,
        target_m: float,
        duration_s: float,
        rate_hz: float,
        speed_mps: float,
        os_threshold_pct: float,
        tol_pct: float,
        results_dir: str,
        target_deg: float = 0.0,
        angular_rps: float = 1.0,
    ) -> bool:
        """ダッシュボードの「実行」ボタン用。

        forward/backwardでは目標走行距離target_m[m]を、rotate_left/
        rotate_rightでは目標回転角度target_deg[度]をステップ入力として与え、
        実際の応答(走行距離[m]または回転角[度])の時間応答を記録・判定して
        results_dirに保存する(scripts/step_response_test/step_response_test.py
        のCLI版と同じ考え方をサーバー側で完結させたもの)。

        【回転モードの角度追跡について】run_drive_test()のtarget_rotate_rad
        判定は「開始時のyawとの単純な差分」を±180°でラップして見ているため、
        目標が180°を超えると壊れる(270°回転しても差分は-90°に見えてしまう)。
        MAX_TEST_ROTATE_DEG=360°まで許容するこのメソッドではそれでは困るため、
        代わりに「1サンプルごとの差分」(角速度がMAX_ANGULAR_RPSでクランプされ
        rate_hzも最大20Hzなので、1サンプルあたりの回転量は180°を大きく
        下回る)を毎ループ積算していく方式で、180°を超える回転量も正しく
        追跡する。

        run_drive_test()と同じ`_test_lock`/`_test_running`を共有し、
        同時に2つの走行テストが/cmd_velを取り合わないようにする。速度・
        距離/角度・記録時間・周期は全て安全のため上限でクランプする(呼び出し元の
        HTTPハンドラ側でも距離/角度はMAX_TEST_DISTANCE_M/MAX_TEST_ROTATE_DEGで
        チェック済みだが、ここでも二重にクランプする)。
        """
        if self._teleop.soft_estop_engaged:
            return False
        # 【2026-10-02追加】NaN/Infは受け付けない(呼び出し側のHTTPハンドラでも弾くが二重に守る)。
        if not all(
            math.isfinite(v)
            for v in (
                target_m, duration_s, rate_hz, speed_mps,
                os_threshold_pct, tol_pct, target_deg, angular_rps,
            )
        ):
            return False
        with self._test_lock:
            if self._test_running:
                return False
            self._test_running = True
        self._step_response_last_result_id = None
        self._step_response_error = None

        is_rotate = direction in ("rotate_left", "rotate_right")

        if is_rotate:
            target_deg = max(0.01, min(self.MAX_TEST_ROTATE_DEG, abs(target_deg)))
            duration_s = max(1.0, min(60.0, duration_s))
            rate_hz = max(1.0, min(20.0, rate_hz))
            angular_speed = max(0.05, min(self.MAX_ANGULAR_RPS, abs(angular_rps)))
            if direction == "rotate_right":
                angular_speed = -angular_speed
            linear_mps = 0.0
            target_value = target_deg
            speed_value = abs(angular_speed)
        else:
            target_m = max(0.01, min(self.MAX_TEST_DISTANCE_M, abs(target_m)))
            duration_s = max(1.0, min(60.0, duration_s))
            rate_hz = max(1.0, min(20.0, rate_hz))
            linear_mps = max(0.05, min(self.MAX_LINEAR_MPS, abs(speed_mps)))
            if direction == "backward":
                linear_mps = -linear_mps
            angular_speed = 0.0
            target_value = target_m
            speed_value = abs(linear_mps)

        def _traveled_m(start_odom: Optional[Tuple[float, float, float]]) -> float:
            if start_odom is None or self._latest_odom is None:
                return 0.0
            x0, y0, _ = start_odom
            x, y, _ = self._latest_odom
            return math.hypot(x - x0, y - y0)

        def _make_rotation_tracker(start_odom: Optional[Tuple[float, float, float]]):
            # 開始角度との単純な差分ではなく、直前サンプルとの差分(こちらは
            # 十分小さいのでラップアラウンドしない)を積算していくことで、
            # 180°を超える回転量も正しく追跡できるようにする。
            state = {
                "prev_yaw": start_odom[2] if start_odom is not None else None,
                "cum_rad": 0.0,
            }

            def _sample_deg() -> float:
                if self._latest_odom is None:
                    return abs(math.degrees(state["cum_rad"]))
                _, _, yaw = self._latest_odom
                if state["prev_yaw"] is not None:
                    delta = (yaw - state["prev_yaw"] + math.pi) % (2.0 * math.pi) - math.pi
                    state["cum_rad"] += delta
                state["prev_yaw"] = yaw
                return abs(math.degrees(state["cum_rad"]))

            return _sample_deg

        def _worker() -> None:
            try:
                self._begin_fast_odom()
                start_odom = self._latest_odom
                self._begin_direct_drive()
                twist = Twist()
                twist.linear.x = linear_mps
                twist.angular.z = angular_speed
                rotation_tracker = _make_rotation_tracker(start_odom) if is_rotate else None
                times: List[float] = []
                responses: List[float] = []
                driving = True
                t_start = time.monotonic()
                period = 1.0 / rate_hz
                n_samples = max(1, int(round(duration_s * rate_hz)))
                for i in range(n_samples + 1):
                    target_wall = t_start + i * period
                    now = time.monotonic()
                    if target_wall > now:
                        time.sleep(target_wall - now)
                    self._odom_fast.touch()
                    estop = self._teleop.soft_estop_engaged
                    current = rotation_tracker() if is_rotate else _traveled_m(start_odom)
                    if driving and (estop or current >= target_value):
                        driving = False
                    # 走行中・記録中いずれもmanual_overrideを送り続け、記録が
                    # 終わるまで(整定を見ている間も)自律行動に割り込まれない
                    # ようにする。estop時はcmd_velはゼロのみpublishする。
                    self._keep_direct_drive()
                    self._cmd_vel_pub.publish(twist if (driving and not estop) else Twist())
                    times.append(time.monotonic() - t_start)
                    responses.append(current)

                metrics = compute_step_response_metrics(
                    times, responses, target_value,
                    os_threshold_pct=os_threshold_pct, tol_pct=tol_pct,
                )
                result_id = self._save_step_response_result(
                    results_dir, direction, target_value, duration_s, rate_hz,
                    speed_value, os_threshold_pct, tol_pct, times, responses, metrics,
                )
                self._step_response_last_result_id = result_id
            except Exception as exc:  # noqa: BLE001 - UIにエラー内容を出すため、ここで一旦捕まえて記録する
                self._step_response_error = str(exc)
                self.node.get_logger().error(f"ステップ応答テストに失敗しました: {exc}")
            finally:
                self._end_direct_drive()
                with self._test_lock:
                    self._test_running = False

        threading.Thread(target=_worker, daemon=True).start()
        return True

    @staticmethod
    def _save_step_response_result(
        results_dir: str,
        direction: str,
        target_value: float,
        duration_s: float,
        rate_hz: float,
        speed_value: float,
        os_threshold_pct: float,
        tol_pct: float,
        times: List[float],
        responses: List[float],
        metrics,
    ) -> str:
        is_rotate = direction in ("rotate_left", "rotate_right")
        executed_at = datetime.now().astimezone()
        run_name = "{ts}_{mode}_{target:.2f}{unit}".format(
            ts=executed_at.strftime("%Y%m%d_%H%M%S"),
            mode=direction,
            target=target_value,
            unit="deg" if is_rotate else "m",
        )
        run_dir = Path(results_dir).expanduser() / run_name
        run_dir.mkdir(parents=True, exist_ok=True)

        with open(run_dir / "data.csv", "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["t_s", "rotation_deg" if is_rotate else "distance_m"])
            for t, d in zip(times, responses):
                writer.writerow([f"{t:.4f}", f"{d:.5f}"])

        result = {
            "executed_at": executed_at.isoformat(),
            "run_name": run_name,
            "params": {
                # 【注意】is_rotate(direction=rotate_left/rotate_right)の場合、
                # target_m/speed_mpsという歴史的なフィールド名のまま角度[度]・
                # 角速度[deg/s相当]の値を入れている(JSONスキーマをforward/
                # backwardと共通化するため)。単位の判別は必ずdirectionを見て
                # 行うこと(list_results()・/step_response ページのJS参照)。
                "target_m": target_value,
                "duration_s": duration_s,
                "rate_hz": rate_hz,
                "os_threshold_pct": os_threshold_pct,
                "tol_pct": tol_pct,
                "sim": False,
                "direction": direction,
                "speed_mps": speed_value,
                "source": "dashboard",
            },
            "times_s": times,
            "distances_m": responses,
            "metrics": metrics.to_dict(),
        }
        with open(run_dir / "result.json", "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)

        return run_name


class StepResponseResultsController:
    """走行距離ステップ応答テスト(scripts/step_response_test/
    step_response_test.py)が保存したCSV/JSON結果を、ダッシュボードの
    `/step_response`ページから一覧・比較表示するための読み取り専用コントローラ。

    走行指令の実行はPidGainsController.run_drive_test()が担い、記録・判定・
    ファイル保存はCLI側(step_response_test.py)がこのクラスと同じ
    ディレクトリ規約(`<results_dir>/<実行id>/result.json`)で行う設計。ここでは
    保存済みのJSONを列挙・読み込みするだけで、ロボット制御には一切関与しない。
    """

    def __init__(self, results_dir: str = "~/.nokosahen_step_response_tests") -> None:
        self.results_dir = Path(results_dir).expanduser()

    def list_results(self) -> List[dict]:
        if not self.results_dir.is_dir():
            return []
        entries: List[dict] = []
        for child in self.results_dir.iterdir():
            if not child.is_dir():
                continue
            result_path = child / "result.json"
            if not result_path.is_file():
                continue
            try:
                with open(result_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
            except Exception:
                continue
            metrics = data.get("metrics", {}) or {}
            params = data.get("params", {}) or {}
            entries.append(
                {
                    "id": child.name,
                    "executed_at": data.get("executed_at"),
                    "target_m": metrics.get("target_m"),
                    "judgement": metrics.get("judgement"),
                    "overshoot_pct": metrics.get("overshoot_pct"),
                    "sim": bool(params.get("sim", False)),
                    "direction": params.get("direction"),
                }
            )
        entries.sort(key=lambda e: e.get("executed_at") or "", reverse=True)
        return entries

    def read_result(self, result_id: str) -> Optional[dict]:
        # list_results()が返したディレクトリ名以外を受け付けない
        # (ディレクトリトラバーサル防止)。
        if not result_id or "/" in result_id or "\\" in result_id or ".." in result_id:
            return None
        result_path = self.results_dir / result_id / "result.json"
        if not result_path.is_file():
            return None
        try:
            with open(result_path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return None


class RobotPoseController:
    """TF(map->base_link)からロボットの現在位置・向きを取得する。

    SLAM中(map->odom->base_link)でもlocalizationモード(AMCL)でも同じ
    map->base_linkの変換を見るだけで動くので、どちらの起動方法でも共通で使える。
    """

    def __init__(self, node, map_frame: str = "map", base_frame: str = "base_link") -> None:
        self.node = node
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.available = Buffer is not None
        if self.available:
            self.tf_buffer = Buffer()
            # 【2026-09-30変更】/tf(約38Hz)を、このノードとは別の専用ノード+専用
            # executor(別スレッド)で受ける。購読の多いこのノードのexecutorで受けると、
            # 1メッセージ毎の待ち受け構築コストが大きい(実測: 空ノードでも/tf
            # 1本の購読で1コアの約13%)。
            # 【注意】TransformListener(spin_thread=True)に本ノードをそのまま渡すのは
            # NG: 同じノードを2つ目のexecutorにも追加して回す実装のため、待ち受けが
            # 二重になりかえって重くなる(2026-09-30に実測で確認)。必ず専用ノードを渡す。
            import rclpy

            self._tf_node = rclpy.create_node(
                f"{node.get_name()}_tf_listener",
                # 【2026-10-02追加】launch全体の`-r __node:=full_dashboard`が
                # この専用ノードにも適用され、同名ノードが二重登録される
                # (Publisher already registered警告、ros2 node infoで見えない)
                # のを防ぐ。グローバルCLI引数(リマップ等)を引き継がない。
                use_global_arguments=False,
            )
            self.tf_listener = TransformListener(
                self.tf_buffer, self._tf_node, spin_thread=True
            )

    def get_pose(self) -> Optional[dict]:
        if not self.available:
            return None
        try:
            import rclpy.time

            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        # 【2026-10-07追加】変換の時刻からの経過秒。localizationモードではmap->odomはAMCLが
        # 出すので、AMCLが止まると最後の変換がバッファに残り続け、古いマーカーが居座る。
        # 表示側が古さで「TFなし」に切り替えられるよう添える(取れなければNone)。
        age_s = None
        try:
            import rclpy.time

            stamp = rclpy.time.Time.from_msg(tf.header.stamp)
            if stamp.nanoseconds > 0:
                age_s = max(0.0, (self.node.get_clock().now() - stamp).nanoseconds / 1e9)
        except Exception:
            age_s = None
        return {"x": t.x, "y": t.y, "yaw": yaw, "age_s": age_s}


# map->base_linkの変換がこの秒数より古ければ「TFなし」扱い(表示の古いマーカー防止)
TF_STALE_SEC = 5.0


def fresh_robot_pose(robot_pose: Optional["RobotPoseController"]) -> Optional[dict]:
    """map->base_linkの姿勢。取得できない/古い(age_s > TF_STALE_SEC)/非有限ならNone。
    ピンを立てる・ピンへ行くなど、「今の位置」を数値として使う処理はこれを通す。"""
    if robot_pose is None:
        return None
    try:
        pose = robot_pose.get_pose()
    except Exception:  # noqa: BLE001
        return None
    if not pose:
        return None
    age = pose.get("age_s")
    if age is not None and age > TF_STALE_SEC:
        return None
    try:
        if not all(math.isfinite(float(pose[k])) for k in ("x", "y", "yaw")):
            return None
    except (KeyError, TypeError, ValueError):
        return None
    return pose


class RouteTrailController:
    """Nav2の計画経路(`/plan`)と、実際に通った軌跡を地図上に可視化するための
    データ源(2026-09-15追加)。発表・実証実験向けに「自律走行が経路を
    考えて動いている」ことを地図上の線として見せる目的。

    軌跡(trail)はRobotPoseControllerと同じmap->base_linkのTFを定期的に
    取得して蓄積する(SLAM中でもlocalizationモードでも動く)。無制限に
    伸び続けないよう、最小移動距離未満の点は間引き、最大保持点数を
    超えたら古い方から自動的に捨てる(dequeのmaxlen)。

    _trail/_planned_pathは、ROS側のタイマー/購読コールバック(rclpy実行
    スレッド)と、HTTPハンドラ側(ThreadingHTTPServerによりリクエスト毎に
    別スレッド)の両方から読み書きされる。他のコントローラ
    (MappingController等)と同じくthreading.Lockで保護する。
    """

    # Nav2の計画経路(/plan)は経路が長いとポーズ数が数百に及ぶことがあり、
    # /api/state(500ms毎にポーリングされる)のたびに丸めてJSON化していると
    # 無駄なCPU負荷になる。地図上の線としては間引いても見た目にほぼ
    # 影響が無いため、保持点数をこの上限までダウンサンプルする。
    MAX_PLANNED_PATH_POINTS = 300

    def __init__(
        self,
        node,
        robot_pose: RobotPoseController,
        trail_hz: float = 1.0,
        min_trail_distance_m: float = 0.15,
        max_trail_points: int = 1000,
    ) -> None:
        self.node = node
        self.robot_pose = robot_pose
        self.min_trail_distance_m = min_trail_distance_m
        self._lock = threading.Lock()
        self._trail: Deque[Tuple[float, float]] = deque(maxlen=max_trail_points)
        self._planned_path: List[Tuple[float, float]] = []
        if NavPath is not None:
            node.create_subscription(NavPath, "/plan", self._on_plan, 5)
        node.create_timer(1.0 / max(0.2, trail_hz), self._on_trail_tick)

    def _on_plan(self, msg) -> None:
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(points) > self.MAX_PLANNED_PATH_POINTS:
            step = len(points) / self.MAX_PLANNED_PATH_POINTS
            points = [points[int(i * step)] for i in range(self.MAX_PLANNED_PATH_POINTS)]
        with self._lock:
            self._planned_path = points

    def _on_trail_tick(self) -> None:
        pose = self.robot_pose.get_pose()
        if pose is None:
            return
        x, y = pose["x"], pose["y"]
        with self._lock:
            if self._trail:
                last_x, last_y = self._trail[-1]
                if math.hypot(x - last_x, y - last_y) < self.min_trail_distance_m:
                    return
            self._trail.append((x, y))

    def clear(self) -> None:
        """地図作成/localizationの開始・リセット時に古い軌跡を残さないよう
        呼び出す(MapLifecycleControllerのon_started/on_stoppedから)。"""
        with self._lock:
            self._trail.clear()
            self._planned_path = []

    def status(self) -> Dict[str, Any]:
        with self._lock:
            planned_path = list(self._planned_path)
            trail = list(self._trail)
        return {
            "planned_path": [[round(x, 3), round(y, 3)] for x, y in planned_path],
            "trail": [[round(x, 3), round(y, 3)] for x, y in trail],
        }


class MapGoalController:
    """`/map`を購読して間引き済みJSONで配信し、地図クリックからの目的地送信を担当する。"""

    def __init__(self, node, robot_pose: Optional["RobotPoseController"] = None) -> None:
        self.node = node
        # 【2026-10-07追加】到着誤差(ロボットの姿勢と目的地の差)を測るのに使う。後から代入してもよい。
        self.robot_pose = robot_pose
        self.available = OccupancyGrid is not None
        self.goal_pub = node.create_publisher(PoseStamped, "/goal_pose", 10)
        # 【2026-10-07追加】中止(既存の/nokosahen/cancel_goal。nav2_goal_routerがNav2のゴールを
        # キャンセルしてゼロ速度を出す。frontier_explorer/behavior_directorも同じ経路を使う)。
        self.cancel_pub = node.create_publisher(Bool, "/nokosahen/cancel_goal", 10)
        # 【2026-10-23追加】ユーザーの「中止」だけが出す専用トピック。frontier_explorer・behavior_directorが受けて、
        # 一定時間(user_cancel_hold_sec)目的地を出し直さない(各ノード自身の/nokosahen/cancel_goalとは区別する)。
        self.user_cancel_pub = node.create_publisher(Bool, "/nokosahen/user_cancel", 10)
        self.latest_map = None
        # 【2026-10-08追加】map_json()の結果キャッシュ。762x550(約42万セル)をPythonで毎リクエスト
        # base64化するとJetson(load average 13)で数百msのCPUを食うため、受信した地図ごとに1回だけ作る。
        # _map_seq は新しい地図を受け取る/破棄するたびに増える通し番号(JSONの"map_id"、ページ側の再デコード省略用)。
        self._map_seq = 0
        self._map_cache: Optional[Tuple[Any, Dict[str, Any]]] = None
        self._map_cache_lock = threading.Lock()
        # 直近に送った目的地の追跡(/nokosahen/nav2_goal_statusの完了通知と突き合わせる)。
        self._goal_lock = threading.Lock()
        self._goal: Optional[Dict[str, Any]] = None
        node.create_subscription(String, "/nokosahen/nav2_goal_status", self._on_goal_status, 10)
        if self.available:
            # 【2026-10-07修正】読み込んだ地図(localizationモード)のmap_serverは、地図を1回だけ
            # TRANSIENT_LOCAL(保持)で配信する。受信側が既定の揮発性(VOLATILE)だとその1回を受け取れず、
            # 保存した地図を読み込んでも画面に出なかった(SLAM中はslam_toolboxが定期的に出し直すため見えていた)。
            # slam_toolbox/map_serverの配信(RELIABLE+TRANSIENT_LOCAL)と同じ設定で受信する。
            from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
            _map_qos = QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
            node.create_subscription(OccupancyGrid, "/map", self._on_map, _map_qos)

    def _on_map(self, msg) -> None:
        with self._map_cache_lock:
            self._map_seq += 1
            self._map_cache = None
            self.latest_map = msg

    def clear(self) -> None:
        """SLAM/localizationが停止した後、もう更新されない古い地図を表示し
        続けないようにキャッシュを破棄する(MapLifecycleControllerのstop/reset
        から呼ばれる)。"""
        with self._map_cache_lock:
            self._map_seq += 1
            self._map_cache = None
            self.latest_map = None

    def map_json(self) -> Optional[dict]:
        with self._map_cache_lock:
            m = self.latest_map
            if m is None:
                return None
            cached = self._map_cache
            if cached is not None and cached[0] is m:
                return cached[1]
            seq = self._map_seq
        # occupancyの値は-1(未知)〜100(占有)。+1して0〜101にすれば1byteに収まる
        # ので、巨大なJSON配列にせずbase64にまとめて転送量を抑える。
        raw = bytes((v + 1) & 0xFF for v in m.data)
        out = {
            "width": m.info.width,
            "height": m.info.height,
            "resolution": m.info.resolution,
            "origin_x": m.info.origin.position.x,
            "origin_y": m.info.origin.position.y,
            "data_base64": base64.b64encode(raw).decode("ascii"),
            # 追加フィールド(従来のキーは不変)。同じ値なら地図は前回と同一=ページは再デコード不要。
            "map_id": seq,
        }
        with self._map_cache_lock:
            if self.latest_map is m:  # 作っている間に新しい地図/clearが来ていたらキャッシュしない
                self._map_cache = (m, out)
        return out

    def _wall_margin_m(self) -> float:
        """クリック目的地を壁から機体端で離す距離[m]。ノードのパラメータ goal_wall_margin_m(既定0.30、0以下で無効)。"""
        if getattr(self, "_goal_wall_margin_cache", None) is None:
            val = 0.30
            try:
                try:
                    self.node.declare_parameter("goal_wall_margin_m", 0.30)
                except Exception:  # noqa: BLE001 - 宣言済み/スタブ
                    pass
                val = float(self.node.get_parameter("goal_wall_margin_m").value)
            except Exception:  # noqa: BLE001 - パラメータが読めなければ既定
                val = 0.30
            self._goal_wall_margin_cache = val
        return self._goal_wall_margin_cache

    def check_goal(self, x: float, y: float, wall_margin: bool = True, rect: Optional[dict] = None) -> Dict[str, Any]:
        """読み込み中の地図で、目的地が「ロボットが立てる空きセル」か調べ、そうでなければ最も近い
        空きセルを返す(snap_goal_to_free)。地図が無ければstatus="no_map"。何もpublishしない。

        【2026-10-22追加】wall_margin=True(既定)かつ goal_wall_margin_m>0 のとき、結果の点が壁から機体端で
        goal_wall_margin_m 未満なら、さらに壁から離れた最寄りの可走セルへ寄せる(patrol_planner.snap_to_drivable)。
        rectを渡せば(走行可能範囲の回転矩形)その内側へ寄せる。クリック点から0.5 m以内に寄せ先が無ければ従来の結果のまま。"""
        m = self.latest_map
        if m is None:
            return {"status": "no_map", "x": x, "y": y, "shift_m": 0.0, "reason": ""}
        try:
            res = snap_goal_to_free(
                int(m.info.width), int(m.info.height), float(m.info.resolution),
                float(m.info.origin.position.x), float(m.info.origin.position.y),
                m.data, x, y,
            )
        except Exception:  # noqa: BLE001 - 判定の失敗でゴール送信を妨げない
            return {"status": "no_map", "x": x, "y": y, "shift_m": 0.0, "reason": ""}
        margin = self._wall_margin_m() if wall_margin else 0.0
        if margin <= 0.0 or res["status"] in ("no_free", "no_map"):
            return res
        try:
            from .patrol_planner import grid_from_msg, snap_to_drivable
            grid = grid_from_msg(m)
            # クリックした点から0.5 m以内(従来のsnapと同じ上限。UIの文言「0.5 m以内」を保つ)で探す。無ければ従来の結果のまま
            wall = snap_to_drivable(grid, x, y, margin_m=margin, rect=rect, max_shift_m=GOAL_SNAP_MAX_SHIFT_M)
            if wall["status"] == "snapped":
                res = dict(res)
                res.update(status="snapped", x=wall["x"], y=wall["y"],
                           shift_m=math.hypot(wall["x"] - x, wall["y"] - y),
                           reason=res["reason"] if res["status"] == "snapped" else "wall_margin")
        except Exception:  # noqa: BLE001 - 余白の判定に失敗しても従来の結果で送る
            pass
        return res

    TRAVEL_HEADING_MIN_DIST_M = 0.3  # これ未満の近い目的地は、進行方向が定まらないので向きを付けない

    def _travel_heading_deg(self, x: float, y: float) -> Optional[float]:
        """現在位置(map)から(x, y)へ向かう方向[度]。現在位置が取れない・近すぎるときはNone。"""
        pose = fresh_robot_pose(self.robot_pose)
        if pose is None:
            return None
        dx, dy = float(x) - float(pose["x"]), float(y) - float(pose["y"])
        if math.hypot(dx, dy) < self.TRAVEL_HEADING_MIN_DIST_M:
            return None
        return math.degrees(math.atan2(dy, dx))

    def send_goal(
        self, x: float, y: float, yaw_deg: Optional[float] = None,
        meta: Optional[Dict[str, Any]] = None,
    ) -> None:
        """/goal_poseへmapフレームの目的地をpublishする。yaw_degがNoneなら従来どおり向きは
        単位(w=1.0)、指定があればその向き(度、map座標系)のクォータニオンにする。
        x, yは丸めずにそのまま入れる(ピンへ行く時に保存した姿勢を厳密に送るため)。"""
        goal = PoseStamped()
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        if yaw_deg is None:
            # 【2026-10-19】向きの指定が無いとき、従来は常にmapの0度(+x方向)だった。到着のたびに、その向きへ余計な旋回が
            # 入っていた。現在位置から目的地へ向かう方向(進行方向)にすれば、到着後の旋回が要らない
            # (自立探索は2026-10-10に同様に直した)。現在位置が取れない・目的地が近すぎる(向きが決まらない)ときは従来どおり。
            # 「向きを指定した」扱いにはしない(yaw_specified=False)ので、画面の矢印・向きの差は出さない。
            travel = self._travel_heading_deg(x, y)
            if travel is None:
                goal.pose.orientation.w = 1.0
            else:
                qx, qy, qz, qw = yaw_deg_to_quaternion(float(travel))
                goal.pose.orientation.x = qx
                goal.pose.orientation.y = qy
                goal.pose.orientation.z = qz
                goal.pose.orientation.w = qw
        else:
            qx, qy, qz, qw = yaw_deg_to_quaternion(float(yaw_deg))
            goal.pose.orientation.x = qx
            goal.pose.orientation.y = qy
            goal.pose.orientation.z = qz
            goal.pose.orientation.w = qw
        with self._goal_lock:
            self._goal = {
                "x": float(x), "y": float(y),
                "yaw_deg": 0.0 if yaw_deg is None else float(yaw_deg),
                "yaw_specified": yaw_deg is not None,
                "sent_at": time.time(), "state": "sent", "event": None,
                "arrival": None, "meta": dict(meta or {}),
            }
        self.goal_pub.publish(goal)

    # ---- 目的地の進捗・到着誤差 ----
    def _on_goal_status(self, msg) -> None:
        try:
            data = json.loads(msg.data)
            event = str(data.get("event"))
            tx = float(data["target_x"])
            ty = float(data["target_y"])
        except Exception:  # noqa: BLE001
            return
        with self._goal_lock:
            g = self._goal
            if g is None or g["state"] != "sent":
                return
            if not (math.isclose(tx, g["x"], abs_tol=1e-3) and math.isclose(ty, g["y"], abs_tol=1e-3)):
                return  # 別の送信元(探索・行動ロジック)のゴールの通知
            if event == "finished":
                try:
                    status = int(data.get("status", 0))
                except (TypeError, ValueError):
                    status = 0
                g["state"] = {4: "succeeded", 5: "canceled", 6: "aborted"}.get(status, "finished")
            else:
                g["state"] = "failed"
            g["event"] = event
            gx, gy, gyaw = g["x"], g["y"], g["yaw_deg"]
        pose = fresh_robot_pose(self.robot_pose)
        if pose is not None:
            d = math.hypot(pose["x"] - gx, pose["y"] - gy)
            with self._goal_lock:
                if self._goal is g:
                    g["arrival"] = {
                        "dist_cm": round(d * 100.0, 1),
                        "yaw_err_deg": round(normalize_pin_deg(math.degrees(pose["yaw"]) - gyaw), 1),
                        "at": time.time(),
                    }

    def goal_status(self) -> Dict[str, Any]:
        with self._goal_lock:
            g = dict(self._goal) if self._goal else None
        if g is None:
            return {"active": False, "state": "none", "goal": None}
        pose = fresh_robot_pose(self.robot_pose)
        dist = None
        yaw_err = None
        pose_out = None
        if pose is not None:
            dist = math.hypot(pose["x"] - g["x"], pose["y"] - g["y"])
            yaw_err = normalize_pin_deg(math.degrees(pose["yaw"]) - g["yaw_deg"])
            pose_out = {"x": pose["x"], "y": pose["y"], "yaw_deg": math.degrees(pose["yaw"])}
        goal = {"x": g["x"], "y": g["y"], "yaw_deg": g["yaw_deg"], "yaw_specified": g["yaw_specified"]}
        goal.update(g["meta"])
        return {
            "active": g["state"] == "sent",
            "state": g["state"],
            "event": g["event"],
            "goal": goal,
            "sent_at": g["sent_at"],
            "sent_age_s": round(time.time() - g["sent_at"], 1),
            "distance_m": dist,
            "yaw_err_deg": yaw_err,
            "arrival": g["arrival"],
            "robot_pose": pose_out,
        }

    def cancel(self) -> bool:
        """進行中の目的地を中止する(既存の/nokosahen/cancel_goal。2回送るのはfrontier_explorerと同じ)。"""
        msg = Bool()
        msg.data = True
        self.cancel_pub.publish(msg)
        self.cancel_pub.publish(msg)
        self.user_cancel_pub.publish(msg)
        with self._goal_lock:
            g = self._goal
            was_active = bool(g and g["state"] == "sent")
            if was_active:
                g["state"] = "canceled"
                g["event"] = "cancel_requested"
        return was_active


class WallEscapeController:
    """【2026-10-22追加】壁際で動けないときの「壁から離れる」。wall_escapeノードの状態(/nokosahen/wall_escape/status、JSON)を
    保持し、手動の離脱(/nokosahen/wall_escape/trigger、Bool)を依頼する。状態が stale_sec 秒以上届かなければ state="unknown"。
    実際に動くか(緊急停止・手動運転・近接停止・LiDAR途絶・十分離れている場合は断る)はノード側が決め、理由は状態のreasonに出る。"""

    STALE_SEC = 5.0

    def __init__(self, node, stale_sec: float = STALE_SEC, clock: Callable[[], float] = time.monotonic) -> None:
        self.node = node
        self.stale_sec = float(stale_sec)
        self._clock = clock
        self._lock = threading.Lock()
        self._status: Optional[Dict[str, Any]] = None
        self._received_at: Optional[float] = None
        self.trigger_pub = node.create_publisher(Bool, "/nokosahen/wall_escape/trigger", 10)
        node.create_subscription(String, "/nokosahen/wall_escape/status", self._on_status, 10)

    def _on_status(self, msg) -> None:
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not isinstance(d, dict):
            return
        with self._lock:
            self._status = d
            self._received_at = self._clock()

    def status(self) -> Dict[str, Any]:
        with self._lock:
            d, at = self._status, self._received_at
        age = None if at is None else max(0.0, self._clock() - at)
        if d is None or age is None or age >= self.stale_sec:
            return {"state": "unknown", "min_range_m": None, "stuck": False, "age_s": None if age is None else round(age, 1),
                    "reason": "壁の状態が届いていません(wall_escapeが動いていない可能性があります)"}
        m = d.get("min_range_m")
        return {
            "state": str(d.get("state", "unknown")),
            "min_range_m": float(m) if isinstance(m, (int, float)) and not isinstance(m, bool) else None,
            "stuck": bool(d.get("stuck", False)),
            "reason": str(d.get("reason", "") or ""),
            "age_s": round(age, 1),
        }

    def trigger(self) -> None:
        msg = Bool()
        msg.data = True
        self.trigger_pub.publish(msg)


_SAFE_MAP_NAME = re.compile(r"^[A-Za-z0-9_\-]{1,64}$")


class MappingController:
    """SLAM(slam_toolbox)で作成中の地図を、CLIを手で叩かずWeb UIから保存する
    ための橋渡し。

    実体は`ros2 run nav2_map_server map_saver_cli`をサブプロセスで呼ぶだけ
    (docs/mapping_and_calibration.mdに書いていた手動手順をUI化したもの)。
    保存先はプロジェクト内の固定ディレクトリ配下のみに限定し、ファイル名は
    ホワイトリスト(英数字・アンダースコア・ハイフンのみ)で検証してから
    パスを組み立てる(Webから受け取った文字列をそのままシェル/パス構築に
    渡すとコマンドインジェクション/パストラバーサルの危険があるため)。
    保存は数秒かかりうるので、HTTPハンドラのスレッドをブロックしないよう
    別スレッドで実行する。
    """

    def __init__(
        self, node, maps_dir: str, robot_pose: Optional["RobotPoseController"] = None,
        pin_store: Optional[PinStore] = None,
    ) -> None:
        self.node = node
        self.maps_dir = Path(maps_dir).expanduser()
        self.maps_dir.mkdir(parents=True, exist_ok=True)
        # 【2026-10-06】地図の表示回転角(<name>.view.json)。地図保存時に併せて保存する。
        self.view_store = get_view_store(str(self.maps_dir))
        # 【2026-10-07】地図ピン(<name>.pins.json)。地図保存時に併せて保存する。
        self.pin_store = pin_store if pin_store is not None else get_pin_store(str(self.maps_dir))
        self._lock = threading.Lock()
        self._busy = False
        self._last_result: Dict[str, Any] = {}

        # LiDARの走査面に映らない障害物(person_target_detectorがカメラで
        # 検知、機体基準のx,yで配信)を、保存する地図にも書き込めるように
        # ワールド座標(map座標系)へ変換して蓄積しておく。robot_poseが
        # 無い(tf2_ros未導入)環境では位置変換できないため何もしない。
        self._robot_pose = robot_pose
        # 同じ障害物がカメラの視界に入り続ける間、毎フレーム(最大5Hz程度)
        # ほぼ同じ座標が届き続けるため、生のリストで貯めると長時間の
        # マッピングでは際限なく増え続けてしまう。0.1m格子に丸めたsetに
        # することで、同じ場所への重複追加を防ぐ。
        self._camera_obstacle_points: set = set()
        node.create_subscription(
            String,
            "/nokosahen/camera_obstacle_point",
            self._on_camera_obstacle_point,
            10,
        )

    def _on_camera_obstacle_point(self, msg: String) -> None:
        if self._robot_pose is None:
            return
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        pose = self._robot_pose.get_pose()
        if pose is None:
            return
        try:
            x = float(data["x"])
            y = float(data["y"])
        except (KeyError, TypeError, ValueError):
            return
        # 機体基準(frame_id、原点はほぼ自機中心)のx,yを、自機の現在位置・
        # 向き(map座標系)で回転・平行移動してワールド座標にする。
        yaw = pose["yaw"]
        world_x = pose["x"] + x * math.cos(yaw) - y * math.sin(yaw)
        world_y = pose["y"] + x * math.sin(yaw) + y * math.cos(yaw)
        grid_cell = (round(world_x / 0.1), round(world_y / 0.1))
        with self._lock:
            self._camera_obstacle_points.add(grid_cell)

    def clear_camera_obstacle_points(self) -> None:
        """新しいマッピングセッションを開始する際、前回分の蓄積を破棄する
        (MapLifecycleController.start_mappingから呼ばれる)。"""
        with self._lock:
            self._camera_obstacle_points = set()

    def status(self) -> Dict[str, Any]:
        with self._lock:
            return {"busy": self._busy, "last_result": dict(self._last_result)}

    def save_map(self, name: str) -> Tuple[bool, str]:
        name = (name or "").strip()
        if not name:
            name = time.strftime("map_%Y%m%d_%H%M%S")
        if not _SAFE_MAP_NAME.match(name):
            return False, "ファイル名は英数字・アンダースコア・ハイフンのみ、64文字以内で指定してください"
        with self._lock:
            if self._busy:
                return False, "他の保存処理が実行中です。しばらくしてから再度お試しください"
            self._busy = True
        dest = self.maps_dir / name

        def _worker() -> None:
            try:
                result = subprocess.run(
                    # 【2026-10-07修正】map_saver_cliの地図受信の待ち時間は既定2秒で、負荷が高い
                    # (load average約12)と発行元の発見に間に合わず「Failed to spin map subscription」で
                    # 毎回失敗していた。待ち時間を20秒に延ばし、全体のタイムアウトも60秒にする。
                    ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(dest),
                     "--ros-args", "-p", "save_map_timeout:=20.0",
                     "-p", "map_subscribe_transient_local:=true"],
                    capture_output=True,
                    text=True,
                    timeout=60,
                )
                ok = result.returncode == 0
                message = (result.stdout + result.stderr).strip()[-2000:]
                if ok:
                    burned = self._burn_camera_obstacles(dest)
                    if burned:
                        message += f" (カメラ検知の障害物{burned}件を書き込み済み)"
                    try:
                        angle = self.view_store.save_for_map(name)
                        message += f" (表示の向き{angle:.1f}°を{name}.view.jsonに保存)"
                    except Exception as exc:  # noqa: BLE001 - 向きの保存失敗は地図保存の失敗にしない
                        message += f" (表示の向きの保存に失敗: {exc})"
                    try:
                        pin_store = getattr(self, "pin_store", None)
                        if pin_store is not None:
                            n_pins = pin_store.save_for_map(name)
                            if n_pins:
                                message += (
                                    f" (ピン{n_pins}件を{name}.pins.jsonに保存。注意: 地図作成中に立てたピンは、"
                                    "地図の補正(ループクロージャ)で実際の場所から少しずれていることがあります。"
                                    "保存した地図を読み込み、ずれたピンはその場所で「現在位置に更新」してください)"
                                )
                    except Exception as exc:  # noqa: BLE001 - ピンの保存失敗は地図保存の失敗にしない
                        message += f" (ピンの保存に失敗: {exc})"
            except Exception as exc:  # noqa: BLE001
                ok = False
                message = str(exc)
            with self._lock:
                self._busy = False
                self._last_result = {
                    "ok": ok,
                    "name": name,
                    "path": str(dest),
                    "message": message,
                    "time": time.time(),
                }
            self.node.get_logger().info(
                f"地図保存{'成功' if ok else '失敗'}: {name} ({dest}.yaml/.pgm)"
            )

        threading.Thread(target=_worker, daemon=True).start()
        return True, f"保存を開始しました({name})。数秒後にステータスを確認してください"

    def _burn_camera_obstacles(self, dest: Path) -> int:
        """マッピング中にカメラで検知した障害物点を、map_saver_cliが書き出した
        直後の.pgmへ直接焼き込む(LiDARの走査面に映らなかった物も、保存後の
        地図上では「占有」として扱われるようにするため)。

        cv2/PyYAML未導入、またはyaml/pgmの読み込みに失敗した場合は基本の
        地図保存自体は既に成功しているので、ここでは警告に留めて成功のまま
        返す(障害物の書き込みはあくまで追加のおまけ機能)。
        """
        with self._lock:
            points = list(self._camera_obstacle_points)
        if not points:
            return 0
        try:
            import cv2  # type: ignore
            import yaml  # type: ignore
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(
                f"障害物の地図書き込みをスキップ(cv2/PyYAML未導入): {exc}"
            )
            return 0
        yaml_path = dest.with_suffix(".yaml")
        try:
            with open(yaml_path, "r", encoding="utf-8") as f:
                meta = yaml.safe_load(f)
            resolution = float(meta["resolution"])
            origin_x = float(meta["origin"][0])
            origin_y = float(meta["origin"][1])
            pgm_path = yaml_path.parent / str(meta["image"])
            img = cv2.imread(str(pgm_path), cv2.IMREAD_UNCHANGED)
            if img is None:
                self.node.get_logger().warn(f"障害物の地図書き込みに失敗: {pgm_path}を読み込めません")
                return 0
            height, width = img.shape[0], img.shape[1]
            written = 0
            radius_px = 2  # 位置推定の誤差を考慮し、周囲数pxもまとめて占有にする
            for grid_x, grid_y in points:
                # _on_camera_obstacle_pointで0.1m格子の整数インデックスに
                # 丸めて保存しているので、ここでワールド座標に戻す。
                world_x = grid_x * 0.1
                world_y = grid_y * 0.1
                col = int(round((world_x - origin_x) / resolution))
                row = height - 1 - int(round((world_y - origin_y) / resolution))
                if not (0 <= row < height and 0 <= col < width):
                    continue
                for dr in range(-radius_px, radius_px + 1):
                    for dc in range(-radius_px, radius_px + 1):
                        rr, cc = row + dr, col + dc
                        if 0 <= rr < height and 0 <= cc < width:
                            img[rr, cc] = 0  # ROS map_server慣例: 0=占有
                written += 1
            cv2.imwrite(str(pgm_path), img)
            return written
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(f"障害物の地図書き込みに失敗: {exc}")
            return 0


LIGHT_LOCALIZATION_ENV = "NOKOSAHEN_LOCALIZATION_LIGHT"
LIGHT_LOCALIZATION_FLAG = "~/.nokosahen_light_localization"
LIGHT_MODE_MESSAGE = "軽量モードのため走行は使えません(目的地の指定・探索は無効です)。通常モードに戻すには docs/light_localization.md を参照してください。"


def localization_light_requested(env: Optional[Dict[str, str]] = None, flag_path: Optional[str] = None) -> bool:
    """【2026-11-06】軽量の位置推定モード(Nav2の走行系を起動しない)を使うか。
    環境変数 NOKOSAHEN_LOCALIZATION_LIGHT が 1/true/yes/on、またはフラグファイル ~/.nokosahen_light_localization が存在すれば真。
    どちらも無ければ偽(従来どおり)。環境変数が 0/false/no/off と明示されていれば、フラグファイルより優先して偽。"""
    e = os.environ if env is None else env
    v = str(e.get(LIGHT_LOCALIZATION_ENV, "")).strip().lower()
    if v in ("1", "true", "yes", "on"):
        return True
    if v in ("0", "false", "no", "off"):
        return False
    try:
        return Path(flag_path or LIGHT_LOCALIZATION_FLAG).expanduser().exists()
    except OSError:
        return False


class MapLifecycleController:
    """地図モード(SLAMでの作成 / 保存済み地図でのlocalization)を、ダッシュ
    ボードのボタンから開始・停止・リセット・読み込みできるようにする。

    ESP32・LiDAR・カメラ・ダッシュボード自体(persistent_base.launch.py)は
    起動したまま、`mapping_mode.launch.py` / `localization_mode.launch.py`
    だけを別プロセス(`ros2 launch`)として起動・終了する。このプロセスを
    止めてもダッシュボード自体は落ちない(別プロセスなので)。

    切り替え前に必ず現在のプロセスを止めてから次を起動するので、SLAMとAMCLが
    同時に二重起動してNav2の入力元が競合することはない。「リセット」は
    プロセスを止めるだけの操作と同じ(slam_toolboxは新規プロセスで必ず
    真っ白な地図から始まるため、保存していない進捗はここで破棄される)。
    """

    def __init__(
        self,
        node,
        maps_dir: str,
        log_path: str = "~/.nokosahen_map_lifecycle.log",
        on_stopped: Optional[Callable[[], None]] = None,
        on_started: Optional[Callable[[], None]] = None,
        remote_ssh_host: str = "",
        pin_store: Optional[PinStore] = None,
    ) -> None:
        self.node = node
        self.maps_dir = Path(maps_dir).expanduser()
        self.maps_dir.mkdir(parents=True, exist_ok=True)
        self.view_store = get_view_store(str(self.maps_dir))
        # 【2026-10-07】地図ピン。新規マッピング/リセットで作業中のピンを空にし、地図の読み込みで復元する。
        self.pin_store = pin_store if pin_store is not None else get_pin_store(str(self.maps_dir))
        self._log_path = Path(log_path).expanduser()
        # 障害調査用に、mapping_mode/localization_mode(slam_toolbox・Nav2・
        # nav2_goal_router等)の標準出力/エラー出力をファイルに残す
        # (以前はDEVNULLに捨てていたため、緊急停止が効かない等の不具合が
        # 起きても後からログで原因を追えなかった)。
        self._on_stopped = on_stopped
        # 新しいマッピングセッションを開始する際、前回分のカメラ障害物蓄積
        # (MappingController側)を破棄するためのコールバック。
        self._on_started = on_started
        # 【2026-08-06追加】Jetson 2台分散構成向け。このダッシュボード
        # (アプリ系、Jetson B)はNav2/SLAM/frontier_explorerを持たず、それらは
        # 足回り・安全系(Jetson A)側にある想定のため、mapping_mode.launch.py /
        # localization_mode.launch.pyはSSH経由でJetson A上で起動する。
        # 空文字なら従来通りローカルでsubprocess起動する(単一Jetson構成)。
        # 【前提】Jetson B→AへSSH鍵認証が設定済みであること。
        self.remote_ssh_host = remote_ssh_host.strip()
        # 【2026-09-15追加】地図作成(SLAM)・localization中はYOLO人物検知
        # (person_target_detector)を休止させ、Nav2/SLAM+YOLOの同時稼働による
        # CPU過負荷(現地検証でload average 17〜40を記録)を軽減する。
        # 地図作成中は基本的に手動運転(または探索モード任せ)で、自律接近・
        # 案内は不要なため休止させても実運用上の支障は無いという判断。
        self._mapping_active_pub = node.create_publisher(Bool, "/nokosahen/mapping_active", 5)
        # 【2026-09-30追加】mapping_activeはlocalization中もTrueになる(YOLO間引き用)
        # ため、behavior_directorが「地図作成中だけ自律行動(cmd_vel/Nav2ゴール)を
        # 抑止」する用途には使えない(通常運用のlocalizationまで止まってしまう)。
        # 地図作成(SLAM)モード中だけTrueになる専用トピックを別に出す。
        self._map_building_pub = node.create_publisher(Bool, "/nokosahen/map_building_active", 5)
        # _lock: _proc/_mode/_loaded_map/_last_errorを守る短時間だけ保持するロック
        # (status()が使う。SSHや停止待ちの間は絶対に保持しない)。
        # _op_lock: start/stop/reset/load_map同士の排他用。こちらは長時間
        # (停止待ち最大8秒、外部プロセス確認のssh最大5秒x2)保持してよい。
        self._lock = threading.Lock()
        self._op_lock = threading.Lock()
        self._proc: Optional[subprocess.Popen] = None
        self._mode = "idle"  # "idle" / "mapping" / "localization"
        self._loaded_map: Optional[str] = None
        self._last_error = ""
        # 遅れて起動したノード(behavior_director等)がvolatile QoSの単発publishを
        # 取りこぼしても追従できるよう、約1Hzで現在の地図作成状態を再送する。
        node.create_timer(1.0, self._republish_map_building)

    def _republish_map_building(self) -> None:
        # 【2026-10-02追加】先にプロセスの生死を確認する。以前はstatus()が呼ばれた時
        # (ページを開いている時)しかクラッシュに気付かず、誰も開いていないと
        # _mode="mapping"のまま/nokosahen/map_building_active=Trueを再送し続け、
        # behavior_directorの自律行動が止まったままになっていた。
        self._reap_exited_process()
        msg = Bool()
        msg.data = self._mode == "mapping"
        self._map_building_pub.publish(msg)

    def _reap_exited_process(self) -> None:
        """起動していたプロセスが終了していたら、状態をidleへ戻して通知する。

        意図した停止(_stop_locked)では先に_mode="idle"になるため、ここで
        異常終了として扱うのは「mapping/localization中のままプロセスだけが
        終了した」場合のみ。"""
        with self._lock:
            proc = self._proc
            if proc is None:
                return
            code = proc.poll()
            if code is None:
                return
            was_mode = self._mode
            self._proc = None
            self._mode = "idle"
            self._loaded_map = None
            crashed = was_mode in ("mapping", "localization")
            if crashed:
                self._last_error = (
                    f"地図作成プロセスが異常終了(code {code})。ログ: {self._log_path}"
                    if was_mode == "mapping"
                    else f"localizationプロセスが異常終了(code {code})。ログ: {self._log_path}"
                )
        if crashed:
            try:
                self.node.get_logger().error(self._last_error)
            except Exception:  # noqa: BLE001
                pass
            # mapping_active/map_building_activeをFalseへ戻し、behavior_directorの
            # 自律行動の抑止を解除する(_modeは既にidle)。
            self._publish_mapping_active(False)
            # 【2026-10-03追加】_stop_locked()と同様、地図表示側(MapGoalController)の
            # 古い地図/軌跡キャッシュを破棄する(ロック外で呼ぶ)。
            if self._on_stopped is not None:
                try:
                    self._on_stopped()
                except Exception as exc:  # noqa: BLE001
                    try:
                        self.node.get_logger().warn(f"地図表示のクリアに失敗: {exc}")
                    except Exception:  # noqa: BLE001
                        pass

    def _ssh_prefix(self) -> List[str]:
        if not self.remote_ssh_host:
            return []
        # -ttでptyを強制する(リモートプロセスグループへSIGINTが確実に
        # 届くようにするため。pty無しだとssh経由のシグナル転送が不安定)。
        return ["ssh", "-tt", self.remote_ssh_host]

    def _ssh_prefix_notty(self) -> List[str]:
        # 出力を読むだけの短いコマンド用。-ttだとリモートのstderrやCRLFが
        # stdoutに混ざって状態文字列の判定が崩れるため、pty無し(-T)にする。
        if not self.remote_ssh_host:
            return []
        return ["ssh", "-T", self.remote_ssh_host]

    def _build_cmd(self, args: List[str]) -> List[str]:
        return self._ssh_prefix() + args

    def _external_process_running(self) -> bool:
        """このコントローラが把握していない(SSH等で外部から起動された)
        mapping_mode/localization_modeプロセスが既に動いていないか確認する。

        2026-08-06追加: SSH経由の手動起動とダッシュボード操作が重なり、
        frontier_explorer/nav2_goal_routerが二重起動してload average 40台の
        深刻な過負荷が実機で発生したため(自分自身が起動したself._procだけを
        見る従来のstart_mapping()/load_map()ではこれを検知できなかった)。
        remote_ssh_host設定時は、起動先(Jetson A)側でpgrepする。
        """
        patterns = ["mapping_mode.launch.py", "localization_mode.launch.py"]
        prefix = self._ssh_prefix()
        for pat in patterns:
            try:
                result = subprocess.run(
                    prefix + ["pgrep", "-f", pat],
                    capture_output=True,
                    text=True,
                    timeout=5,
                )
            except Exception as exc:  # noqa: BLE001
                self.node.get_logger().warn(f"外部プロセス確認に失敗(続行): {exc}")
                continue
            if result.stdout.strip():
                return True
        return False

    def external_mapping_running(self) -> bool:
        """このコントローラが把握していない(管理画面の再起動前・SSHからの起動)地図作成(mapping_mode)が、いま動いているか。
        【2026-10-21】管理画面だけが再起動されると、状態(_mode)がidleに戻る一方で、地図作成のプロセスは動き続ける(別セッション)。
        その状態で「プログラム終了」を押すと、未保存の地図が、警告なしに失われていた。保存の状況は分からないので、
        呼び出し側(system_shutdown.build_map_state)は、未保存として扱う。pgrepの失敗は「動いていない」(従来どおり)。
        自分自身(pgrep)に一致しないよう、パターンは文字を割って書く。"""
        if self._proc is not None and self._proc.poll() is None:
            return False  # このコントローラが起動したもの(modeが把握している)
        try:
            result = subprocess.run(
                self._ssh_prefix() + ["pgrep", "-f", "mapping_mode[.]launch[.]py"],
                capture_output=True, text=True, timeout=5,
            )
        except Exception:  # noqa: BLE001
            return False
        return bool(result.stdout.strip())

    def status(self) -> Dict[str, Any]:
        # 起動していたプロセスが自然終了/クラッシュしていた場合、表示上の状態も
        # idleへ追従させ、last_errorを立てる(2026-10-02: 通知・publishも行う)。
        self._reap_exited_process()
        with self._lock:
            alive = self._proc is not None and self._proc.poll() is None
            return {
                "mode": self._mode,
                "started_at": getattr(self, "_mapping_started_at", 0.0),
                "loaded_map": self._loaded_map,
                "light": bool(getattr(self, "_light", False)) and self._mode == "localization",
                "running": alive,
                "last_error": self._last_error,
            }

    def is_light(self) -> bool:
        """軽量の位置推定モードで稼働中か(Nav2の走行系が無い。目的地・探索は使えない)。"""
        with self._lock:
            return bool(getattr(self, "_light", False)) and self._mode == "localization"

    def list_maps(self) -> List[str]:
        return sorted(p.stem for p in self.maps_dir.glob("*.yaml"))

    def _publish_mapping_active(self, active: bool) -> None:
        msg = Bool()
        msg.data = active
        self._mapping_active_pub.publish(msg)
        building = Bool()
        building.data = bool(active and self._mode == "mapping")
        self._map_building_pub.publish(building)

    def _stop_locked(self) -> None:
        """現在のプロセスを停止する。呼び出し側は_op_lockを保持していること
        (_lockは保持しない。停止待ち最大8秒の間もstatus()をブロックしない)。"""
        # 先にmodeを外してから停止通知(map_building_activeを即座にFalseへ)。
        with self._lock:
            proc = self._proc
            self._mode = "idle"
        self._publish_mapping_active(False)
        if proc is not None:
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGINT)
                proc.wait(timeout=8)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
        with self._lock:
            if self._proc is proc:
                self._proc = None
            self._mode = "idle"
            self._loaded_map = None
        # SLAM/localizationを止めたら、地図表示側が古い(もう更新されない)
        # 地図をずっとキャッシュしたまま表示し続けてしまうため、呼び出し側
        # (MapGoalController)のキャッシュも合わせて破棄する。
        if self._on_stopped is not None:
            try:
                self._on_stopped()
            except Exception as exc:  # noqa: BLE001
                self.node.get_logger().warn(f"地図表示のクリアに失敗: {exc}")

    def stop(self) -> Tuple[bool, str]:
        with self._op_lock:
            self._stop_locked()
            return True, "地図モードを停止しました"

    def _reset_view(self) -> None:
        """新しい地図(白紙/新規SLAM)は座標系が変わるので、表示の回転は0°に戻す。"""
        try:
            self.view_store.clear_loaded_map()
            self.view_store.set_session_angle(0.0)
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(f"表示回転のリセットに失敗: {exc}")
        # 新しい地図は座標系が変わるので、作業中のピン(セッション)も空にする(保存済みの地図のピンは消さない)。
        try:
            self.pin_store.clear_session()
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(f"ピンのリセットに失敗: {exc}")

    def reset(self) -> Tuple[bool, str]:
        with self._op_lock:
            self._stop_locked()
            self._reset_view()
            return True, "地図をリセットしました(次回開始時は白紙から作成します)"

    def _open_log(self):
        """mapping_mode/localization_mode(slam_toolbox・Nav2・nav2_goal_router等)
        の標準出力/エラー出力を書き出す先を開く。起動の度に上書きし、
        直近1回分のログだけを見れば障害調査できるようにする。"""
        self._log_path.parent.mkdir(parents=True, exist_ok=True)
        return open(self._log_path, "w", encoding="utf-8", errors="replace")

    def _lifecycle(self, *args: str) -> str:
        """`ros2 lifecycle <args>`を実行して標準出力を返す(失敗時は空文字)。"""
        try:
            result = subprocess.run(
                self._ssh_prefix_notty() + ["ros2", "lifecycle", *args],
                capture_output=True,
                text=True,
                timeout=30,
                stdin=subprocess.DEVNULL,
            )
        except Exception:  # noqa: BLE001
            return ""
        # 行毎にstripし、状態行(unconfigured/inactive/active)があればそれを返す
        # (警告行等が混ざっても判定が崩れないようにする)。
        lines = [ln.strip() for ln in result.stdout.splitlines()]
        for ln in lines:
            if re.match(r"^(unconfigured|inactive|active)", ln):
                return ln
        return result.stdout.strip()

    def _ensure_slam_active(self, proc) -> None:
        """slam_toolboxがactiveにならないまま止まった場合に有効化する。

        【2026-09-30追加】起動直後の高負荷(load average約9)で、launchが送る
        configureの応答(/slam_toolbox/change_state)がタイムアウトし、launchが
        成功と認識できず次のactivateに進まないことがあった。slam_toolboxが
        inactiveのまま/mapが一切出ず、地図が表示されない不具合になっていた。
        起動から一定時間、状態を見てinactive/unconfiguredなら補う。
        """
        time.sleep(25)
        deadline = time.time() + 120
        while time.time() < deadline:
            with self._lock:
                if self._proc is not proc or proc.poll() is not None:
                    return  # 停止/再起動された
            state = self._lifecycle("get", "/slam_toolbox")
            if state.startswith("active"):
                return
            if state.startswith("unconfigured"):
                self._lifecycle("set", "/slam_toolbox", "configure")
                state = "inactive"
            if state.startswith("inactive"):
                out = self._lifecycle("set", "/slam_toolbox", "activate")
                self.node.get_logger().warn(f"slam_toolboxが未activeだったため有効化しました: {out}")
            time.sleep(5)

    def start_mapping(self) -> Tuple[bool, str]:
        with self._op_lock:
            self._stop_locked()
            if self._external_process_running():
                return False, (
                    "mapping_mode/localization_modeが既に(このダッシュボード外で)"
                    "起動中のようです。二重起動を避けるため、SSH等で手動停止してから"
                    "再度お試しください。"
                )
            cmd = self._build_cmd(["ros2", "launch", "nokosahen_bringup", "mapping_mode.launch.py"])
            try:
                log_file = self._open_log()
                new_proc = subprocess.Popen(
                    cmd,
                    start_new_session=True,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                )
                log_file.close()
            except Exception as exc:  # noqa: BLE001
                with self._lock:
                    self._last_error = str(exc)
                return False, f"地図作成の開始に失敗しました: {exc}"
            with self._lock:
                self._proc = new_proc
                self._mode = "mapping"
                self._mapping_started_at = time.time()  # 【2026-10-13】プログラム終了の未保存判定用(開始後に保存したか)
                self._loaded_map = None
                self._last_error = ""
            self._reset_view()
            self._publish_mapping_active(True)
            if self._on_started is not None:
                try:
                    self._on_started()
                except Exception as exc:  # noqa: BLE001
                    self.node.get_logger().warn(f"新規マッピングセッションの初期化に失敗: {exc}")
            threading.Thread(
                target=self._ensure_slam_active, args=(new_proc,), daemon=True
            ).start()
            return True, "地図作成(SLAM)を開始しました"

    def load_map(self, name: str) -> Tuple[bool, str]:
        name = (name or "").strip()
        if not _SAFE_MAP_NAME.match(name):
            return False, "地図名は英数字・アンダースコア・ハイフンのみ、64文字以内で指定してください"
        map_yaml = self.maps_dir / f"{name}.yaml"
        # 【注意】remote_ssh_host設定時、このexists()チェックは実際にlaunchが
        # 走るリモート側(Jetson A)ではなく、このダッシュボードが動いている
        # ローカル(Jetson B)のファイルシステムを見ている。地図ファイルを
        # 両機に同期する仕組みは未実装のため、分散構成では地図の保存先
        # (maps_dir)を共有ストレージにするか、rsync等で同期する必要がある。
        if not map_yaml.exists():
            return False, f"地図ファイルが見つかりません: {map_yaml}"
        with self._op_lock:
            self._stop_locked()
            if self._external_process_running():
                return False, (
                    "mapping_mode/localization_modeが既に(このダッシュボード外で)"
                    "起動中のようです。二重起動を避けるため、SSH等で手動停止してから"
                    "再度お試しください。"
                )
            light = localization_light_requested()
            launch_args = [
                "ros2", "launch", "nokosahen_bringup", "localization_mode.launch.py",
                f"map:={map_yaml}",
            ]
            if light:
                launch_args.append("light:=true")  # 【2026-11-06】Nav2の走行系を起動しない
            cmd = self._build_cmd(launch_args)
            try:
                log_file = self._open_log()
                new_proc = subprocess.Popen(
                    cmd,
                    start_new_session=True,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                )
                log_file.close()
            except Exception as exc:  # noqa: BLE001
                with self._lock:
                    self._last_error = str(exc)
                return False, f"地図の読み込みに失敗しました: {exc}"
            with self._lock:
                self._proc = new_proc
                self._mode = "localization"
                self._loaded_map = name
                self._last_error = ""
                self._light = light
            self._publish_mapping_active(True)
            restored = None
            try:
                restored = self.view_store.on_map_loaded(name)
            except Exception as exc:  # noqa: BLE001
                self.node.get_logger().warn(f"表示の向きの復元に失敗: {exc}")
            extra = f"、表示の向き{restored:.1f}°を復元" if restored is not None else ""
            try:
                n_pins = self.pin_store.on_map_loaded(name)
                extra += f"、ピン{n_pins}件を復元"
            except Exception as exc:  # noqa: BLE001
                self.node.get_logger().warn(f"ピンの復元に失敗: {exc}")
            mode_txt = "localization軽量モード(走行不可)" if light else "localizationモード"
            return True, f"地図「{name}」を読み込みました({mode_txt}{extra})"


class LocalizationAssistError(Exception):
    """位置合わせ機能の実行時エラー。kind: not_localization / service_unavailable / unsupported"""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind
        self.message = message


class LocalizationAssistController:
    """「ロボットに現在位置を教える」機能(2026-10-07追加)。

    - set_initial_pose(): AMCLへ初期位置を教える(/initialpose へ PoseWithCovarianceStamped を
      数回publish)。ロボットは動かさない(cmd_vel/Nav2ゴールは一切出さない)。
    - global_localization(): AMCLの/reinitialize_global_localization(std_srvs/Empty)を呼び、
      パーティクルを地図全体に撒き直す(ロボットを少し走らせると収束する)。
    - status(): /amcl_pose(AMCLはRELIABLE+TRANSIENT_LOCAL・depth1で出す。それに合わせて購読)の
      共分散から「位置の確かさ」を分類し、tfのmap->base_linkの有無と合わせて返す。
      /amcl_poseは誰も見ていなければ購読しない(LazySubscription)。

    localizationモード(MapLifecycleController.status()のmode=="localization"かつ稼働中)
    でなければ、初期位置・全体位置推定の要求は拒否する(AMCLが居ないため)。
    """

    def __init__(
        self,
        node,
        map_lifecycle: Optional["MapLifecycleController"],
        map_goal: Optional["MapGoalController"] = None,
        robot_pose: Optional["RobotPoseController"] = None,
        repeat_count: int = 3,
        repeat_interval_sec: float = 0.3,
        service_timeout_sec: float = 3.0,
    ) -> None:
        self.node = node
        self.map_lifecycle = map_lifecycle
        self.map_goal = map_goal
        self.robot_pose = robot_pose
        self.repeat_count = max(1, int(repeat_count))
        self.repeat_interval_sec = float(repeat_interval_sec)
        self.service_timeout_sec = float(service_timeout_sec)
        self._lock = threading.Lock()
        self._latest: Optional[Dict[str, Any]] = None
        self._publish_gen = 0
        self._publish_thread: Optional[threading.Thread] = None
        self.available = PoseWithCovarianceStamped is not None
        self._pub = None
        self._client = None
        self._lazy = None
        if self.available:
            self._pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
            qos: Any = 1
            try:
                from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

                qos = QoSProfile(
                    depth=1,
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                )
            except Exception:  # noqa: BLE001  # pragma: no cover - 環境依存
                pass
            self._lazy = LazySubscription(
                node, PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose,
                depth=qos, idle_timeout_sec=15.0, on_unsubscribe=self._clear_latest,
            )
        if EmptySrv is not None:
            self._client = node.create_client(EmptySrv, "/reinitialize_global_localization")

    # ---- /amcl_pose ----
    def _on_amcl_pose(self, msg) -> None:
        try:
            p = msg.pose.pose
            q = p.orientation
            latest = {
                "x": float(p.position.x),
                "y": float(p.position.y),
                "yaw": quaternion_to_yaw(q.x, q.y, q.z, q.w),
                "covariance": [float(v) for v in msg.pose.covariance],
                "rx": time.monotonic(),
            }
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._latest = latest

    def _clear_latest(self) -> None:
        with self._lock:
            self._latest = None

    # ---- 状態 ----
    def _localization_active(self) -> Tuple[bool, Dict[str, Any]]:
        if self.map_lifecycle is None:
            return False, {}
        st = self.map_lifecycle.status()
        return (st.get("mode") == "localization" and bool(st.get("running"))), st

    def status(self) -> Dict[str, Any]:
        if self._lazy is not None:
            self._lazy.touch()
        active, ls = self._localization_active()
        pose = self.robot_pose.get_pose() if self.robot_pose is not None else None
        age = pose.get("age_s") if pose else None
        tf_ok = pose is not None and (age is None or age <= TF_STALE_SEC)
        with self._lock:
            latest = dict(self._latest) if self._latest else None
        out: Dict[str, Any] = {
            "mode": ls.get("mode", "idle"),
            "loaded_map": ls.get("loaded_map"),
            "localization_active": active,
            "tf_available": bool(tf_ok),
            "amcl_pose": None,
            "age_sec": None,
        }
        if not active:
            out.update({"quality": "inactive", "label": QUALITY_LABELS["inactive"],
                        "pos_std_m": None, "yaw_std_rad": None,
                        "text": "位置の確かさ: 地図を読み込むと表示されます(localization未起動)"})
            return out
        q = classify_localization_quality(latest["covariance"] if latest else None)
        out["quality"] = q["state"]
        out["label"] = q["label"]
        out["pos_std_m"] = q["pos_std_m"]
        out["yaw_std_rad"] = q["yaw_std_rad"]
        if latest:
            out["amcl_pose"] = {"x": latest["x"], "y": latest["y"], "yaw_deg": math.degrees(latest["yaw"])}
            out["age_sec"] = max(0.0, time.monotonic() - latest["rx"])
        if q["state"] == "waiting":
            out["text"] = "位置の確かさ: 位置合わせ待ち(地図上で現在位置を指定してください)"
        elif q["pos_std_m"] is None:
            out["text"] = f"位置の確かさ: {q['label']}"
        else:
            out["text"] = (
                f"位置の確かさ: {q['label']}(位置のばらつき {q['pos_std_m']:.2f} m / "
                f"向き {math.degrees(q['yaw_std_rad']):.0f}°)"
            )
        return out

    def _require_localization(self, what: str) -> None:
        active, _ = self._localization_active()
        if not active:
            raise LocalizationAssistError(
                "not_localization",
                f"{what}は、保存した地図を読み込んで「localizationモード」で動いている時だけ使えます。"
                "先に「地図の作成・保存・読込」で地図を読み込んでください。",
            )

    # ---- 初期位置 ----
    def _build_msg(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = initial_pose_covariance()
        return msg

    def _publish_burst(self, gen: int, x: float, y: float, yaw: float) -> None:
        for i in range(self.repeat_count):
            with self._lock:
                if gen != self._publish_gen:
                    return  # 新しい指定に置き換えられた
            try:
                self._pub.publish(self._build_msg(x, y, yaw))
            except Exception as exc:  # noqa: BLE001
                self.node.get_logger().warn(f"/initialposeのpublishに失敗: {exc}")
                return
            if i < self.repeat_count - 1:
                time.sleep(self.repeat_interval_sec)

    def set_initial_pose(self, data: Any) -> Dict[str, Any]:
        """検証して/initialposeを数回publishする(背景スレッド)。PoseRequestError(400) /
        LocalizationAssistError(409) を投げうる。戻り値は{ok, message, warning?, x, y, yaw_deg}。"""
        x, y, yaw = parse_initial_pose_request(data)
        if not self.available or self._pub is None:
            raise LocalizationAssistError("unsupported", "この環境では位置合わせを使えません(geometry_msgsが無い)")
        self._require_localization("位置合わせ")
        warning = ""
        latest_map = getattr(self.map_goal, "latest_map", None) if self.map_goal is not None else None
        if latest_map is not None:
            inside = point_in_map_bounds(latest_map.info, x, y)
            if inside is False:
                warning = "指定した位置は読み込んだ地図の範囲の外です。場所を間違えていないか確認してください(送信はしました)。"
        with self._lock:
            self._publish_gen += 1
            gen = self._publish_gen
        self._clear_latest()  # 古い推定を「今の位置」として出さない
        th = threading.Thread(target=self._publish_burst, args=(gen, x, y, yaw), daemon=True)
        self._publish_thread = th
        th.start()
        msg = (
            f"位置 ({x:.2f}, {y:.2f}) m、向き {math.degrees(yaw):.0f}° をAMCLへ送りました。"
            "「位置の確かさ」が「良好」になるまで待ち、必要なら少し走らせて確かめてください。"
        )
        out: Dict[str, Any] = {"ok": True, "message": msg, "x": x, "y": y, "yaw_deg": math.degrees(yaw)}
        if warning:
            out["warning"] = warning
            out["message"] = msg + " 注意: " + warning
        return out

    # ---- 全体位置推定 ----
    def global_localization(self) -> Dict[str, Any]:
        self._require_localization("自動の位置探索")
        if self._client is None:
            raise LocalizationAssistError("unsupported", "この環境では全体位置推定を使えません(std_srvsが無い)")
        result: Dict[str, Any] = {}

        def worker() -> None:
            try:
                if not self._client.wait_for_service(timeout_sec=self.service_timeout_sec):
                    result["error"] = LocalizationAssistError(
                        "service_unavailable",
                        "AMCLのサービス(/reinitialize_global_localization)が見つかりません。"
                        "localizationの起動直後は数十秒待つか、地図を読み込み直してください。",
                    )
                    return
                fut = self._client.call_async(EmptySrv.Request())
                done = threading.Event()
                fut.add_done_callback(lambda _f: done.set())
                if not done.wait(self.service_timeout_sec):
                    result["error"] = LocalizationAssistError(
                        "service_unavailable",
                        f"AMCLが{self.service_timeout_sec:.0f}秒以内に応答しませんでした。",
                    )
                    return
                exc = fut.exception()
                if exc is not None:
                    result["error"] = LocalizationAssistError(
                        "service_unavailable", f"AMCLの呼び出しに失敗しました: {exc}")
                    return
                result["ok"] = True
            except Exception as exc:  # noqa: BLE001
                result["error"] = LocalizationAssistError(
                    "service_unavailable", f"AMCLの呼び出しに失敗しました: {exc}")

        th = threading.Thread(target=worker, daemon=True)
        th.start()
        th.join(self.service_timeout_sec * 2 + 1.0)
        if th.is_alive():
            raise LocalizationAssistError("service_unavailable", "AMCLの呼び出しがタイムアウトしました。")
        if "error" in result:
            raise result["error"]
        self._clear_latest()
        return {
            "ok": True,
            "message": "地図全体から位置を探し始めました。ロボットを安全な場所で少し走らせる(手動運転)と、"
                       "位置の確かさが「良好」へ収束します。収束するまで地図クリックの自動走行はしないでください。",
        }


class PinController:
    """地図ピン(名前付きの場所)の追加・更新・削除と、「このピンへ行く」の下ごしらえ(2026-10-07追加)。

    現在位置はRobotPoseController(tf map->base_link)から取る。SLAM中はいつでも取れる。localization中は
    AMCLが推定を出してから(それまではTFが無い)。取れない/古い(age_s > TF_STALE_SEC)ときは、
    曖昧な位置にピンを立てたり走り出したりしないよう、はっきりした日本語で拒否する(PinError kind=no_pose)。
    実際のゴール送信(ガードを含む)はmake_dashboard_handlerが/api/goalと同じ経路で行う。
    """

    SLAM_NOTE = (
        "地図作成中のピンは、後でループクロージャ(地図の自動補正)によって、実際の場所から少しずれる"
        "ことがあります。地図を保存して読み込んだあと、ずれたピンは現地で「現在位置に更新」してください。"
    )

    def __init__(
        self,
        node,
        store: PinStore,
        robot_pose: Optional["RobotPoseController"] = None,
        map_goal: Optional["MapGoalController"] = None,
        map_lifecycle: Optional["MapLifecycleController"] = None,
    ) -> None:
        self.node = node
        self.store = store
        self.robot_pose = robot_pose
        self.map_goal = map_goal
        self.map_lifecycle = map_lifecycle
        # 【2026-10-09】行動ルールのgoto_pin用に、読み込み中の地図のピンを`/nokosahen/pins`
        # (TRANSIENT_LOCAL, JSON)で配信する。変化時と5秒ごと(鮮度の合図)に出す。
        self._pins_pub = None
        self._pins_sig = None
        self._pins_last_pub = 0.0
        try:
            try:
                from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

                pub_qos = QoSProfile(
                    depth=1,
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                )
            except Exception:  # noqa: BLE001 - rclpy無し(テスト用スタブ)
                pub_qos = 1
            self._pins_pub = node.create_publisher(String, PINS_TOPIC, pub_qos)
            create_timer = getattr(node, "create_timer", None)
            if create_timer is not None:
                create_timer(2.0, self.publish_pins)
        except Exception as exc:  # noqa: BLE001 - 配信できなくても画面操作は止めない
            self._pins_pub = None
            try:
                node.get_logger().warn(f"ピン配信({PINS_TOPIC})を開始できません: {exc}")
            except Exception:  # noqa: BLE001
                pass

    def publish_pins(self, force: bool = False) -> bool:
        """読み込み中の地図とそのピンを配信する。変化が無ければ5秒おき以外は出さない。"""
        if self._pins_pub is None:
            return False
        try:
            pins = self.store.list()
            loaded = self.store.loaded_map()
            sig = json.dumps([loaded, pins], sort_keys=True, ensure_ascii=False)
            now = time.monotonic()
            if not force and sig == self._pins_sig and now - self._pins_last_pub < 5.0:
                return False
            msg = String()
            msg.data = json.dumps(pins_payload(loaded, pins), ensure_ascii=False)
            self._pins_pub.publish(msg)
            self._pins_sig = sig
            self._pins_last_pub = now
            return True
        except Exception:  # noqa: BLE001
            return False

    def rule_pin_warnings(self, document: Any) -> List[str]:
        """ルール文書のgoto_pinが指すピンが、いま読み込み中の地図に無い場合の警告(日本語)。
        保存は妨げない(実行時にdirectorが無視して走行しない)。"""
        out: List[str] = []
        rules = document.get("rules") if isinstance(document, dict) else None
        if not isinstance(rules, list):
            return out
        payload = pins_payload(self.store.loaded_map(), self.store.list())
        for i, r in enumerate(rules):
            if not isinstance(r, dict):
                continue
            then = r.get("then") if isinstance(r.get("then"), dict) else {}
            pol = then.get("policy")
            pols = [pol] if isinstance(pol, str) else (pol if isinstance(pol, list) else [])
            if "goto_pin" not in pols:
                continue
            params = then.get("params") if isinstance(then.get("params"), dict) else {}
            pin, reason = resolve_pin(payload, params.get("pin_id"), params.get("pin_map"))
            if pin is None:
                label = r.get("name") or r.get("id") or f"rules[{i}]"
                out.append(
                    f"ルール「{label}」(rules[{i}])の行き先ピン「{params.get('pin_name') or params.get('pin_id')}」: "
                    f"{PIN_REASON_TEXT.get(reason, reason)}。このままでも保存できますが、"
                    "実行時はそのピンへは行かず、その場で待機します。"
                )
        return out

    def _mode(self) -> str:
        if self.map_lifecycle is None:
            return "idle"
        try:
            return str(self.map_lifecycle.status().get("mode", "idle"))
        except Exception:  # noqa: BLE001
            return "idle"

    def _require_pose(self, what: str) -> dict:
        pose = fresh_robot_pose(self.robot_pose)
        if pose is not None:
            return pose
        if self._mode() == "localization":
            msg = (
                f"{what}にはロボットの現在位置が必要ですが、まだ分かりません(位置合わせ前、またはAMCLの推定が"
                f"{TF_STALE_SEC:g}秒以上古い)。先に「現在位置を指定」で位置合わせしてください。"
            )
        else:
            msg = (
                f"{what}にはロボットの現在位置が必要ですが、取得できません(TF map→base_linkが無い、または"
                f"{TF_STALE_SEC:g}秒以上古い)。地図作成(SLAM)または保存した地図の読み込みが動いているか確認してください。"
            )
        raise PinError(msg, "no_pose")

    @staticmethod
    def _pose_dict(pose: dict) -> Dict[str, float]:
        return {"x": float(pose["x"]), "y": float(pose["y"]), "yaw_deg": normalize_pin_deg(math.degrees(pose["yaw"]))}

    def status(self) -> Dict[str, Any]:
        from .map_pins import PIN_MAX_COUNT, PIN_NAME_MAX_LEN

        pose = fresh_robot_pose(self.robot_pose)
        mode = self._mode()
        pins = self.store.list()
        for p in pins:
            if pose is not None:
                p["distance_m"] = round(math.hypot(p["x"] - pose["x"], p["y"] - pose["y"]), 3)
            else:
                p["distance_m"] = None
        return {
            "pins": pins,
            "count": len(pins),
            "max": PIN_MAX_COUNT,
            "name_max": PIN_NAME_MAX_LEN,
            "loaded_map": self.store.loaded_map(),
            "mode": mode,
            "pose": (dict(self._pose_dict(pose), age_s=pose.get("age_s")) if pose is not None else None),
            "note": self.SLAM_NOTE if mode == "mapping" else "",
        }

    def _pose_from_request(self, source: Any, what: str) -> Tuple[float, float, float]:
        if source == "robot":
            p = self._pose_dict(self._require_pose(what))
            return p["x"], p["y"], p["yaw_deg"]
        if isinstance(source, dict):
            return parse_pin_pose(source)
        raise PinError("位置は from='robot' か {x, y, yaw_deg} で指定してください")

    def add(self, data: Dict[str, Any]) -> Dict[str, Any]:
        if "from" not in data:
            raise PinError("from('robot' または {x, y, yaw_deg}) が必要です")
        x, y, yaw = self._pose_from_request(data["from"], "現在位置にピンを立てる")
        pin = self.store.add(data.get("name"), data.get("color"), x, y, yaw)
        msg = f"ピン「{pin['name']}」を ({x:.2f}, {y:.2f}) m、向き {yaw:.0f}° に立てました。"
        if self._mode() == "mapping":
            msg += " " + self.SLAM_NOTE
        self.publish_pins(force=True)
        return {"ok": True, "pin": pin, "message": msg}

    def update(self, data: Dict[str, Any]) -> Dict[str, Any]:
        fields = {k: data[k] for k in ("name", "color", "x", "y", "yaw_deg") if k in data}
        if "from" in data:
            x, y, yaw = self._pose_from_request(data["from"], "ピンの位置の更新")
            fields.update({"x": x, "y": y, "yaw_deg": yaw})
        pin = self.store.update(data.get("id"), fields)
        self.publish_pins(force=True)
        return {"ok": True, "pin": pin, "message": f"ピン「{pin['name']}」を更新しました。"}

    def delete(self, data: Dict[str, Any]) -> Dict[str, Any]:
        pin = self.store.delete(data.get("id"))
        self.publish_pins(force=True)
        return {"ok": True, "pin": pin, "message": f"ピン「{pin['name']}」を削除しました。"}

    def resolve_goto(self, data: Dict[str, Any]) -> Tuple[Dict[str, Any], dict]:
        """goto前の検証: 保存済みのピンと、現在の(新しい)ロボット姿勢を返す。"""
        pin = self.store.get(data.get("id"))
        if pin is None:
            raise PinError("ピンが見つかりません", "not_found")
        pose = self._require_pose("ピンへ行く")
        return pin, pose


class BoundaryController:
    """この機体が自律走行してよい範囲(map座標系の矩形)をダッシュボードから
    設定し、`/nokosahen/travel_boundary`へpublishする。

    【2026-10-06】範囲は回転した矩形 {center_x, center_y, width, height, yaw[rad]}
    (世界座標)。旧形式 min_x/max_x/min_y/max_y は yaw=0 の矩形として読み替え、
    状態/トピックには軸平行外接矩形を min_*/max_* として同時に載せる(旧消費者の
    behavior_director等はそのまま動くが、外接矩形で判定するため回転矩形の隅は
    厳密には守れない。frontier_explorerと/api/goalは真の回転矩形で判定する)。
    純粋ロジックは map_orientation.py(単体テスト済み)。

    behavior_director側がこれを購読し、範囲外に出そうな自律動作(混雑時の
    ゴール送信・見回し中の直進等)を止める安全策として使う。手動運転(WASD)は
    対象外(操作者が見ながら運転しているため)。

    設定はこのダッシュボードプロセスの再起動後も保つため、ファイルにも
    保存しておき起動時に読み込んで即座に再publishする(MappingControllerの
    保存先と同じ考え方で、ホーム配下の固定ファイルにする)。
    """

    def __init__(self, node, state_path: str) -> None:
        self.node = node
        self.state_path = Path(state_path).expanduser()
        self.pub = node.create_publisher(String, "/nokosahen/travel_boundary", 10)
        self._lock = threading.Lock()
        self._state = self._load()
        # 【2026-10-23】スケジュールによる一時的な差し替え/解除(docs/schedule.md)。directorが
        # /nokosahen/boundary_override へ1Hzで配信する。travel_boundaryを出すのはこのコントローラだけ
        # (directorが直接publishすると、下の5秒ごとの再送と競合して範囲が行き来する)ので、ここで
        # 「ユーザーが設定した範囲(self._state、ファイルに保存される元の範囲)」と上書きを合成して出す。
        # 上書きはTTL(BOUNDARY_OVERRIDE_TTL_SEC)内に届き続けている間だけ有効で、保存はしない。
        self._override: Optional[Dict[str, Any]] = None
        self._override_rx = 0.0
        self._last_pub: Optional[str] = None
        create_subscription = getattr(node, "create_subscription", None)
        if create_subscription is not None:
            create_subscription(String, BOUNDARY_OVERRIDE_TOPIC, self._on_override, 10)
        self._publish()
        # 【2026-10-06追加】購読側(behavior_director/frontier_explorer/nav2_goal_router)が
        # ダッシュボードより後に起動/再起動しても範囲を受け取れるよう、同じ状態を5秒ごとに
        # 再publishする(以前は設定時と起動時の1回だけで、後から起動した購読側は
        # 「範囲なし」のまま動いていた)。内容は変わらないので購読側の動作は同じ。
        create_timer = getattr(node, "create_timer", None)
        if create_timer is not None:
            create_timer(5.0, self._publish)
            create_timer(1.0, self._publish_if_changed)  # 上書きの開始/終了/期限切れを1秒以内に反映する

    def _load(self) -> Dict[str, Any]:
        try:
            data = json.loads(self.state_path.read_text(encoding="utf-8"))
        except Exception:  # noqa: BLE001 - 未作成/壊れている場合は既定(制限無し)を使う
            return disabled_state()
        return load_boundary_state(data)  # 旧形式(min/maxのみ)のファイルも読める

    def _save(self) -> None:
        try:
            self.state_path.parent.mkdir(parents=True, exist_ok=True)
            self.state_path.write_text(
                json.dumps(self._state, ensure_ascii=False), encoding="utf-8"
            )
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(f"走行可能範囲の保存に失敗: {exc}")

    def _on_override(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        state = parse_boundary_override(data)  # 不正・active偽はNone=上書きなし
        with self._lock:
            self._override = state
            self._override_rx = time.monotonic() if state is not None else 0.0
        self._publish_if_changed()

    def _effective_locked(self) -> Dict[str, Any]:
        """いま有効な範囲(上書きが新しければ上書き、無ければユーザーが設定した元の範囲)。"""
        if self._override is not None and time.monotonic() - self._override_rx <= BOUNDARY_OVERRIDE_TTL_SEC:
            return dict(self._override)
        return dict(self._state)

    def effective_status(self) -> Dict[str, Any]:
        """購読側が見ている範囲(travel_boundaryに出している内容)。"""
        with self._lock:
            return self._effective_locked()

    def override_info(self) -> Dict[str, Any]:
        """スケジュールによる上書きの有無(画面表示用)。"""
        with self._lock:
            eff = self._effective_locked()
        return {"active": bool(eff.get("override_active")), "enabled": bool(eff.get("enabled")),
                "source": eff.get("override_source")}

    def _publish(self) -> None:
        with self._lock:
            text = json.dumps(self._effective_locked(), ensure_ascii=False)
            self._last_pub = text
        msg = String()
        msg.data = text
        self.pub.publish(msg)

    def _publish_if_changed(self) -> None:
        with self._lock:
            text = json.dumps(self._effective_locked(), ensure_ascii=False)
            changed = text != self._last_pub
        if changed:
            self._publish()

    def status(self) -> Dict[str, Any]:
        """ユーザーが設定した範囲(元の範囲)。スケジュールの上書きは含めない(編集画面の値が上書きで変わらないように)。"""
        with self._lock:
            return dict(self._state)

    def allows_point(self, x: float, y: float) -> bool:
        """(x,y)が走行可能範囲内か。範囲未設定/解除中は常にTrue(従来どおり)。
        スケジュールの上書き中は、購読側(director/router/explorer)と同じ「いま有効な範囲」で判定する。"""
        with self._lock:
            return state_allows_point(self._effective_locked(), x, y)

    def set_from_request(self, data: Any) -> Tuple[bool, str]:
        """POSTボディ(新形式/旧形式)から設定。BoundaryErrorは呼び出し側(HTTP)で扱う。"""
        rect = parse_boundary_request(data)
        with self._lock:
            self._state = boundary_state(rect, True)
            self._save()
        self._publish()
        return True, "走行可能範囲を設定しました"

    def set_boundary(
        self, min_x: float, max_x: float, min_y: float, max_y: float
    ) -> Tuple[bool, str]:
        """旧API(軸平行)互換。"""
        try:
            return self.set_from_request(
                {"min_x": min_x, "max_x": max_x, "min_y": min_y, "max_y": max_y}
            )
        except BoundaryError as exc:
            return False, str(exc)

    def clear_boundary(self) -> Tuple[bool, str]:
        with self._lock:
            self._state = disabled_state()
            self._save()
        self._publish()
        return True, "走行可能範囲の制限を解除しました"


class KeepOutZoneController:
    """地図内の「ここには入らないで」という立入禁止エリア(map座標系の矩形、
    複数可)をダッシュボードから設定し、`/nokosahen/keepout_zones`へpublishする。

    BoundaryController(外周の走行可能範囲、1つだけ)とは別物で、こちらは
    地図の内側に机・柱等を避けるための矩形をいくつでも追加できる
    (2026-07-17、ユーザー要望: カメラでの家具認識が当てにならない場合の
    代替/補助策)。behavior_director側がこれを購読し、機体の現在位置が
    いずれかのゾーン内に入ったら自律動作を止める(手動運転WASDは対象外、
    BoundaryControllerと同じ設計方針)。

    【既知の制限】BoundaryController同様、経路計画そのものにゾーンを
    組み込んでいるわけではなく、「入ってしまったら止める」という事後的な
    安全策にとどまる(Nav2のcostmapフィルタ等への統合は未実装)。
    """

    def __init__(self, node, state_path: str) -> None:
        self.node = node
        self.state_path = Path(state_path).expanduser()
        self.pub = node.create_publisher(String, "/nokosahen/keepout_zones", 10)
        self._lock = threading.Lock()  # 追加・削除・全削除のIDと一覧の更新を直列にする(同時に追加するとIDが重複した)
        self._zones: List[Dict[str, Any]] = self._load()
        self._next_id = (max([z["id"] for z in self._zones], default=0)) + 1
        self._publish()
        # 【2026-10-10追加】設定時と起動時の1回だけだと、後から起動/再起動した購読側(behavior_director/frontier_explorer)は
        # 「立入禁止エリアなし」のまま動く。BoundaryControllerと同じく5秒ごとに同じ内容を再publishする(購読側の処理は冪等)。
        create_timer = getattr(node, "create_timer", None)
        if create_timer is not None:
            create_timer(5.0, self._publish)

    def _load(self) -> List[Dict[str, Any]]:
        try:
            data = json.loads(self.state_path.read_text(encoding="utf-8"))
            if isinstance(data, list):
                return [z for z in data if isinstance(z, dict) and "id" in z]
        except Exception:  # noqa: BLE001 - 未作成/壊れている場合は空(制限無し)を使う
            pass
        return []

    def _save(self) -> None:
        try:
            self.state_path.parent.mkdir(parents=True, exist_ok=True)
            self.state_path.write_text(
                json.dumps(self._zones, ensure_ascii=False), encoding="utf-8"
            )
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(f"立入禁止エリアの保存に失敗: {exc}")

    def _publish(self) -> None:
        msg = String()
        msg.data = json.dumps(self._zones, ensure_ascii=False)
        self.pub.publish(msg)

    def status(self) -> Dict[str, Any]:
        return {"zones": list(self._zones)}

    def add_zone(
        self, min_x: float, max_x: float, min_y: float, max_y: float
    ) -> Tuple[bool, str]:
        # 【2026-10-21】NaN・無限大は、比較が常にFalseになるため、以前は検査を通って「追加しました」と返るのに、どの点にも一致しない
        # (効かない)エリアが保存されていた。有限で、地図として現実的な範囲(±KEEPOUT_COORD_MAX_M)の値だけを受け付ける。
        if not all(math.isfinite(v) and abs(v) <= KEEPOUT_COORD_MAX_M for v in (min_x, max_x, min_y, max_y)):
            return False, f"座標は、有限の数で、±{KEEPOUT_COORD_MAX_M:g} m 以内にしてください"
        if min_x >= max_x or min_y >= max_y:
            return False, "範囲が不正です(対角の2点が同じ/逆になっていませんか？)"
        with self._lock:
            zone_id = self._next_id
            self._next_id += 1
            self._zones.append(
                {
                    "id": zone_id,
                    "min_x": min_x,
                    "max_x": max_x,
                    "min_y": min_y,
                    "max_y": max_y,
                }
            )
            self._save()
        self._publish()
        return True, f"立入禁止エリアを追加しました(id={zone_id})"

    def remove_zone(self, zone_id: int) -> Tuple[bool, str]:
        with self._lock:
            before = len(self._zones)
            self._zones = [z for z in self._zones if z.get("id") != zone_id]
            if len(self._zones) == before:
                return False, "指定されたidの立入禁止エリアが見つかりません"
            self._save()
        self._publish()
        return True, "立入禁止エリアを削除しました"

    def clear(self) -> Tuple[bool, str]:
        with self._lock:
            self._zones = []
            self._save()
        self._publish()
        return True, "立入禁止エリアを全て削除しました"


class FrontierExplorerController:
    """マッピング中の自律探索(frontier_explorer)を、ダッシュボードのボタンから
    有効/無効に切り替える。frontier_explorerノード自体はmapping_mode.launch.py
    が常時起動しており、このトピックで待機/探索を切り替えるだけ(地図モードで
    ない間はそもそもfrontier_explorerが起動していないので、statusは取得できない)。
    """

    def __init__(self, node) -> None:
        self.pub = node.create_publisher(
            Bool, "/nokosahen/frontier_explorer/enabled", 5
        )
        self._last_status: Dict[str, Any] = {}
        node.create_subscription(
            String,
            "/nokosahen/frontier_explorer/status",
            self._on_status,
            5,
        )

    def _on_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(data, dict):
            self._last_status = data

    def set_enabled(self, enabled: bool) -> Tuple[bool, str]:
        msg = Bool()
        msg.data = enabled
        self.pub.publish(msg)
        return True, ("自律探索を開始しました" if enabled else "自律探索を停止しました")

    def status(self) -> Dict[str, Any]:
        return dict(self._last_status)


class BehaviorDirectorControlController:
    """A/Bテスト(推奨発話ON/OFF)の切り替えと、デモ用強制トリガーを
    ダッシュボードのボタンから操作するための薄いラッパー(2026-07-22追加)。

    実際の処理は`behavior_director.py`側(`_on_recommend_enabled`/
    `_on_force_demo`)にあり、ここはトピックをpublishするだけ。
    """

    def __init__(self, node) -> None:
        self.recommend_pub = node.create_publisher(
            Bool, "/nokosahen/behavior_director/recommend_enabled", 5
        )
        self.force_demo_pub = node.create_publisher(
            Bool, "/nokosahen/behavior_director/force_demo", 5
        )
        # 【2026-08-08追加】人物未検出時の待機動作を、LiDAR/カメラ判定を使う
        # CROWD_SCANから、左右へ1秒ずつ交互に回転するだけの単純な演出に
        # 切り替えるトグル(behavior_director.py側の実処理を参照)。
        self.idle_wiggle_pub = node.create_publisher(
            Bool, "/nokosahen/behavior_director/idle_wiggle_enabled", 5
        )
        # 【2026-11-02追加】人の方へ動く動作(接近・見回し)のスイッチ。既定はOFF(その場で待つ)。
        # 再送はしない: directorを再起動したら安全側のOFFへ戻り、画面はdirectorの報告値(behavior_command)を表示する。
        self.person_interaction_pub = node.create_publisher(
            Bool, "/nokosahen/behavior_director/person_interaction_enabled", 5
        )
        # 【2026-11-03追加】見回す(scan)・人の方を向く(face)の独立スイッチ。再送はしない(再起動でOFFに戻る安全側)。
        self.scan_enabled_pub = node.create_publisher(
            Bool, "/nokosahen/behavior_director/scan_enabled", 5
        )
        self.face_enabled_pub = node.create_publisher(
            Bool, "/nokosahen/behavior_director/face_enabled", 5
        )
        # 【2026-10-13追加】推奨発話ON/OFFは、一度でも設定したら5秒ごとに同じ値を再送する
        # (AutoSpeedControllerと同じ。以前は1回きりで、directorを再起動すると既定(ON)に戻るのに
        # 画面は設定値のままだった。directorの_on_recommend_enabledは冪等)。director側の現在値は
        # behavior_command(/api/stateのrecommend_enabled)に載るので、画面で読み戻して確認する。
        # 未設定の間は何も送らない(directorの起動時既定のまま)。
        self.recommend_requested: Optional[bool] = None
        create_timer = getattr(node, "create_timer", None)
        if create_timer is not None:
            create_timer(5.0, self._republish_recommend)

    def _republish_recommend(self) -> None:
        if self.recommend_requested is not None:
            msg = Bool()
            msg.data = self.recommend_requested
            self.recommend_pub.publish(msg)

    def set_recommend_enabled(self, enabled: bool) -> Tuple[bool, str]:
        msg = Bool()
        msg.data = enabled
        self.recommend_requested = bool(enabled)
        self.recommend_pub.publish(msg)
        return True, ("推奨発話を有効にしました" if enabled else "推奨発話を無効にしました(A/Bテスト用)")

    def trigger_demo(self) -> Tuple[bool, str]:
        msg = Bool()
        msg.data = True
        self.force_demo_pub.publish(msg)
        return True, "同一人物クールダウンを解除しました。目の前にいる人物への接近を試みます。"

    def set_person_interaction_enabled(self, enabled: bool) -> Tuple[bool, str]:
        msg = Bool()
        msg.data = bool(enabled)
        self.person_interaction_pub.publish(msg)
        return True, (
            "人の方へ動く動作(接近)を許可しました。人を見つけると近づきます"
            if enabled
            else "人の方へ動く動作を止めました。人がいても、その場で待ちます"
        )

    def set_scan_enabled(self, enabled: bool) -> Tuple[bool, str]:
        msg = Bool()
        msg.data = bool(enabled)
        self.scan_enabled_pub.publish(msg)
        return True, (
            "見回す動作を許可しました。人がいないときは、旋回して探します"
            if enabled
            else "見回す動作を止めました。人がいないときは、その場で待ちます"
        )

    def set_face_enabled(self, enabled: bool) -> Tuple[bool, str]:
        msg = Bool()
        msg.data = bool(enabled)
        self.face_enabled_pub.publish(msg)
        return True, (
            "人の方を向く動作を許可しました。人がいると、その場で旋回して人の多い方を向きます"
            if enabled
            else "人の方を向く動作を止めました"
        )

    def set_idle_wiggle_enabled(self, enabled: bool) -> Tuple[bool, str]:
        msg = Bool()
        msg.data = enabled
        self.idle_wiggle_pub.publish(msg)
        return True, ("左右振動の待機動作を有効にしました" if enabled else "左右振動の待機動作を無効にしました")


class BehaviorRulesController:
    """時間帯・状況ルール層(behavior_rules、docs/behavior_rules.md)の編集ページ(/behavior_rules)用。

    【2026-10-06追加】
    - ルールファイル(behavior_directorの`rules_file`と同じパス)の読み込み・検証つき保存。
      検証はbehavior_rules.py自身の関数で行い、不正なら何も書かない。保存は原子的で、
      直前のファイルは`.bak`に残す(behavior_rules_store.RulesFileStore)。
    - 保存後に`/nokosahen/behavior_rules/reload`(Empty)をpublishしてbehavior_directorに
      即時再読み込みさせる(送れなくてもdirector側は1秒以内にmtime変化で自動検知する)。
    - `/nokosahen/behavior_rules/state`(1Hz)は、ページを開いている間だけ購読する
      (LazySubscription、誰も見ていなければ購読もしない)。
    ロボットを動かす機能は無い(ルールの書き換えと再読み込みの通知だけ)。
    """

    STATE_TOPIC = "/nokosahen/behavior_rules/state"
    RELOAD_TOPIC = "/nokosahen/behavior_rules/reload"
    STATE_STALE_SEC = 5.0  # 1Hz配信なので、これだけ途切れたら無応答
    WARMUP_SEC = 4.0  # 購読開始直後、最初の1件を待つ猶予

    def __init__(self, node, rules_file: str, example_file: str = "",
                 utc_offset_hours: Optional[float] = None,
                 pin_controller: Optional["PinController"] = None) -> None:
        self.node = node
        # 【2026-10-09】goto_pin(地図ピンへ行く)の行き先が現在の地図に在るかの警告用。
        self.pin_controller = pin_controller
        self.store = RulesFileStore(rules_file)
        self.example_file = example_file
        self._clock = _rules_make_clock(utc_offset_hours)
        self._lock = threading.Lock()
        self._state: Optional[Dict[str, Any]] = None
        self._rx = 0.0
        self._sub_started = 0.0
        self._reload_pub = (
            node.create_publisher(Empty, self.RELOAD_TOPIC, 5) if Empty is not None else None
        )
        self._lazy = LazySubscription(
            node, String, self.STATE_TOPIC, self._on_state, depth=2,
            idle_timeout_sec=15.0, on_unsubscribe=self._clear_state,
        )

    # --- 稼働状況(/nokosahen/behavior_rules/state) ---

    def _on_state(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(data, dict):
            with self._lock:
                self._state = data
                self._rx = time.monotonic()

    def _clear_state(self) -> None:
        with self._lock:
            self._state = None
            self._rx = 0.0

    def live_state(self) -> Dict[str, Any]:
        was_active = self._lazy.active
        self._lazy.touch()
        now = time.monotonic()
        with self._lock:
            if not was_active:
                self._sub_started = now
                self._state = None
                self._rx = 0.0
            state, rx, started = self._state, self._rx, self._sub_started
        out: Dict[str, Any] = {
            "status": "waiting",
            "state": None,
            "age_s": None,
            "dashboard_rules_file": self.store.path,
            "path_mismatch": False,
            "message": "",
        }
        if state is not None and rx:
            age = now - rx
            out["age_s"] = round(age, 1)
            if age <= self.STATE_STALE_SEC:
                out["status"] = "ok"
                out["state"] = state
                theirs = state.get("rules_file")
                if (
                    isinstance(theirs, str) and theirs
                    and os.path.normpath(theirs) != os.path.normpath(self.store.path)
                ):
                    out["path_mismatch"] = True
                return out
            out["status"] = "no_response"
            out["message"] = "behavior_director 無応答/未起動(状態が %.0f 秒以上届いていません)" % age
            return out
        if now - started < self.WARMUP_SEC:
            out["message"] = "behavior_directorの状態を待っています..."
            return out
        out["status"] = "no_response"
        out["message"] = "behavior_director 無応答/未起動(状態が届きません)"
        return out

    # --- ルールファイル ---

    def rules_payload(self) -> Dict[str, Any]:
        payload = self.store.read()
        payload["meta"] = rules_editor_meta()
        payload["pin_warnings"] = self._pin_warnings(payload.get("document"))
        return payload

    def _pin_warnings(self, document: Any) -> List[str]:
        if self.pin_controller is None:
            return []
        try:
            return self.pin_controller.rule_pin_warnings(document)
        except Exception:  # noqa: BLE001 - 警告の計算失敗で保存/表示を止めない
            return []

    def example_payload(self) -> Tuple[int, Dict[str, Any]]:
        doc, path, err = load_rules_example(self.example_file)
        if doc is None:
            return 404, {"ok": False, "errors": [err or "例ファイルを読めません"], "path": path}
        return 200, {"ok": True, "document": doc, "path": path}

    def preview_payload(self, hours: int = 24) -> Tuple[int, Dict[str, Any]]:
        ruleset, errors = self.store.load_ruleset()
        if ruleset is None:
            return 409, {"ok": False, "errors": ["保存済みファイルが不正なため予定を計算できません"] + errors}
        out = build_rules_preview(ruleset, self._clock(), hours)
        out["ok"] = True
        out["rules_file"] = self.store.path
        return 200, out

    def save(self, body: Any) -> Tuple[int, Dict[str, Any]]:
        if not isinstance(body, dict) or not isinstance(body.get("document"), dict):
            return 400, {"ok": False, "errors": ["document(ルール文書のオブジェクト)が必要です"]}
        expected = body.get("expected_sig")
        if expected is not None and not isinstance(expected, str):
            return 400, {"ok": False, "errors": ["expected_sigは文字列にしてください"]}
        result = self.store.save(body["document"], expected_sig=expected)
        if not result["ok"]:
            status = {"invalid": 400, "conflict": 409}.get(result.get("kind"), 500)
            return status, result
        result["reload_published"] = self._publish_reload()
        result["warnings"] = self._pin_warnings(body["document"])
        result["message"] = (
            "保存しました。behavior_directorへ再読み込みを通知しました。"
            if result["reload_published"]
            else "保存しました(再読み込み通知は送れませんでした。behavior_directorが1秒以内に自動で検知します)。"
        )
        return 200, result

    def request_reload(self) -> Tuple[int, Dict[str, Any]]:
        ok = self._publish_reload()
        return 200, {
            "ok": ok,
            "message": "再読み込みを通知しました" if ok else "再読み込みの通知を送れませんでした",
        }

    def _publish_reload(self) -> bool:
        if self._reload_pub is None:
            return False
        try:
            self._reload_pub.publish(Empty())
            return True
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(f"behavior_rulesの再読み込み通知に失敗: {exc}")
            return False


def rules_editor_meta() -> Dict[str, Any]:
    """編集フォーム用の選択肢・上下限(behavior_rules.pyの定義をそのまま渡す)。"""
    from . import behavior_rules as _br

    return {
        "policies": list(_br.POLICIES),
        "conflicts": [list(c) for c in _br._CONFLICTS],  # noqa: SLF001
        "weekdays": list(_br.WEEKDAYS),
        "crowd_levels": list(_br.CROWD_LEVELS),
        "speed_scale_min": _br.SPEED_SCALE_MIN,
        "speed_scale_max": _br.SPEED_SCALE_MAX,
        "announce_interval_min": _br.ANNOUNCE_INTERVAL_MIN,
        "announce_interval_max": _br.ANNOUNCE_INTERVAL_MAX,
        "max_rules": _br.MAX_RULES,
        "default_min_hold_seconds": _br.DEFAULT_MIN_HOLD_SECONDS,
    }


class CrowdBackgroundController:
    """混雑判定(crowd_monitor)の背景(壁・什器等の静的な近さの基準)を、
    ダッシュボードのボタン1つでキャリブレーションできるようにする。

    設置場所によっては正面2m以内に壁がある向きもあり得るため、単純な
    距離しきい値だけだと無人でも「混雑」と誤判定してしまう。この背景を
    一度記録しておくことで、crowd_monitor側は「背景よりも明確に近いもの」
    だけを人物として数えるようになる(dashboard_common.py内のcrowd_monitor
    連携専用、実際の判定ロジック自体はcrowd_monitor.py側にある)。
    """

    REQUEST_TOPIC = "/nokosahen/crowd_monitor/calibrate_request"
    STATE_TOPIC = "/nokosahen/crowd_calibration_state"

    def __init__(self, node) -> None:
        # 【2026-10-13変更】要求にrequest_idを付け(String JSON)、crowd_monitorが返す結果
        # (STATE_TOPIC、TRANSIENT_LOCAL)を受けて、画面が「受理された/完了した/失敗した」を確認できるようにした。
        # 以前はBoolを1回publishするだけで、crowd_monitorが止まっていても成功と表示していた。
        self.node = node
        self.pub = node.create_publisher(String, self.REQUEST_TOPIC, 5)
        self._seq = 0
        self._lock = threading.Lock()
        self._last: Optional[Dict[str, Any]] = None
        self.last_request_id = ""
        try:
            from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

            qos: Any = QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
        except Exception:  # noqa: BLE001 - rclpy無し(テスト)では深さだけ指定
            qos = 1
        node.create_subscription(String, self.STATE_TOPIC, self._on_state, qos)

    def _on_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(data, dict):
            with self._lock:
                self._last = data

    def request_calibration(self) -> Tuple[bool, str, str]:
        """(ok, message, request_id)。結果はstatus()で読み戻す(crowd_monitorが返すまで確認できない)。"""
        with self._lock:
            self._seq += 1
            rid = f"cal-{int(time.time() * 1000)}-{self._seq}"
            self.last_request_id = rid
        msg = String()
        msg.data = json.dumps({"request_id": rid})
        self.pub.publish(msg)
        return (
            True,
            "要求を送りました。crowd_monitorが次のLiDARスキャンを混雑判定の背景として記録します"
            "(部屋を無人にした状態で実行してください)",
            rid,
        )

    def calibrate(self) -> Tuple[bool, str]:
        ok, message, _rid = self.request_calibration()
        return ok, message

    def status(self) -> Dict[str, Any]:
        """crowd_monitorが最後に返した結果(request_id, state, at, background_frames, error)。未受信ならstate:None"""
        with self._lock:
            last = dict(self._last) if self._last else None
        if last is None:
            return {"state": None, "request_id": None}
        last.setdefault("state", None)
        last.setdefault("request_id", None)
        return last


class ScanController:
    """`/scan`を購読し、ダッシュボードでLiDARの見え方(ロボット中心のレーダー風表示)
    を描画するためのJSONを提供する。"""

    def __init__(self, node) -> None:
        self.node = node
        self.available = LaserScan is not None
        self.latest_scan = None
        # 2026-09-30: 閲覧者がいる間だけ/scanを購読する(LazySubscription参照)。
        self._lazy = (
            LazySubscription(
                node, LaserScan, "/scan", self._on_scan, depth=10,
                on_unsubscribe=self._clear_scan,
            )
            if self.available
            else None
        )

    def _on_scan(self, msg) -> None:
        self.latest_scan = msg

    def _clear_scan(self) -> None:
        # 購読解除中に古いスキャンを「現在のもの」として返さないため破棄する。
        self.latest_scan = None

    def scan_json(self) -> Optional[dict]:
        if self._lazy is not None:
            self._lazy.touch()
        msg = self.latest_scan
        if msg is None:
            return None
        # 無限大/範囲外(未検出)はnullにして、フロント側で「そこには何も
        # 無かった」と区別できるようにする(0や range_max だと誤解を招くため)。
        ranges = [
            r if (math.isfinite(r) and msg.range_min <= r <= msg.range_max) else None
            for r in msg.ranges
        ]
        return {
            "angle_min": msg.angle_min,
            "angle_increment": msg.angle_increment,
            "range_min": msg.range_min,
            "range_max": msg.range_max,
            "ranges": ranges,
        }


class CameraPreviewController:
    """`person_target_detector`が配信する検出枠つきJPEGを購読し、ダッシュボードで
    「Webカメラが何を見てどれくらい認識しているか」を確認できるようにする。

    ポーリング間隔(ダッシュボード側で数百ms毎に再取得)なので滑らかな動画には
    ならない(静止画が数コマ/秒で切り替わる程度)。カメラ自体のcamera_fps
    (既定5fps)以上のなめらかさは元々出ない。
    """

    def __init__(self, node, topic: str = "/nokosahen/front_camera/preview") -> None:
        self.node = node
        self.available = CompressedImage is not None
        self.latest_jpeg: Optional[bytes] = None
        # 2026-09-30: 閲覧者がいる間だけ購読する(LazySubscription参照)。
        self._lazy = (
            LazySubscription(
                node, CompressedImage, topic, self._on_image, depth=2,
                on_unsubscribe=self._clear_image,
            )
            if self.available
            else None
        )

    def _on_image(self, msg) -> None:
        self.latest_jpeg = bytes(msg.data)

    def _clear_image(self) -> None:
        # 購読解除中に古い画像を「現在のもの」として返さないため破棄する。
        self.latest_jpeg = None

    def jpeg_bytes(self) -> Optional[bytes]:
        if self._lazy is not None:
            self._lazy.touch()
        return self.latest_jpeg


class CheckoutCamRegistry:
    """メニュー毎の決済場所ユニット(`nokosahen_checkout`、Webカメラ+Raspberry Pi 4)の
    一覧・映像・メニュー割り当てを管理する。

    各ユニットは`camera_id`(そのRaspberry PiのCPUシリアルから自動生成)を
    載せて共有トピックに配信するので、ここでcamera_id毎に振り分けて保持する。
    ダッシュボードからのメニュー割り当ては`/nokosahen/checkout_cam/assign`へ
    publishし、対象ユニットだけがそれを見て自分の設定を更新する。
    """

    def __init__(
        self,
        node,
        status_topic: str = "/nokosahen/checkout_cam/status",
        preview_topic: str = "/nokosahen/checkout_cam/preview",
        assign_topic: str = "/nokosahen/checkout_cam/assign",
        offline_timeout_sec: float = 5.0,
    ) -> None:
        self.node = node
        self.offline_timeout_sec = offline_timeout_sec
        self._status: Dict[str, dict] = {}
        self._status_time: Dict[str, float] = {}
        self._jpeg: Dict[str, bytes] = {}
        self._assign_pub = node.create_publisher(String, assign_topic, 10)
        node.create_subscription(String, status_topic, self._on_status, 20)
        self._preview_available = CompressedImage is not None
        if self._preview_available:
            node.create_subscription(CompressedImage, preview_topic, self._on_preview, 4)

    def _on_status(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        camera_id = str(data.get("camera_id", "")).strip()
        if not camera_id:
            return
        self._status[camera_id] = data
        self._status_time[camera_id] = time.monotonic()

    def _on_preview(self, msg) -> None:
        camera_id = str(msg.header.frame_id).strip()
        if not camera_id:
            return
        self._jpeg[camera_id] = bytes(msg.data)

    def list_cameras(self) -> List[dict]:
        now = time.monotonic()
        out = []
        for camera_id, status in self._status.items():
            last_seen = self._status_time.get(camera_id, 0.0)
            entry = dict(status)
            entry["camera_id"] = camera_id
            entry["online"] = (now - last_seen) <= self.offline_timeout_sec
            entry["has_preview"] = camera_id in self._jpeg
            out.append(entry)
        out.sort(key=lambda e: e["camera_id"])
        return out

    def jpeg_bytes(self, camera_id: str) -> Optional[bytes]:
        return self._jpeg.get(camera_id)

    def assign(
        self, camera_id: str, menu: str, direction: Optional[str] = None
    ) -> None:
        payload: Dict[str, Any] = {"camera_id": camera_id, "menu": menu}
        if direction:
            # "both"/"left_to_right"/"right_to_left"。省略時は対象ユニット側の
            # 現在値を変更しない(menuだけの割り当てと互換性を保つため)。
            payload["direction"] = direction
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._assign_pub.publish(msg)


class SingleControllerGate:
    """操作用アカウント(例: TL)を同時に1台の端末だけに制限するための簡易ロック。

    HTTP Basic認証はステートレス(ログイン/ログアウトの概念が無く、同じ
    ユーザー名/パスワードを複数の端末が同時に送ってきても区別できない)ため、
    接続元IPアドレスを「今使っている端末」の代理識別子として扱う。
    `timeout_sec`の間アクセスが無ければ自動的に解放するので、ブラウザを
    閉じ忘れた場合も次の人がすぐ使える(watchdog系の既存パターンと同じ考え方)。

    PC版ダッシュボードとスマホ版ページはそれぞれ別インスタンスを持たせ、
    「PC1人+スマホ1人」を独立にカウントする(要件通り)。
    """

    # 【方針確定・2026-07-15】この同時1端末制限は完成版(full_dashboard_node、
    # 実際にカフェテリアで稼働させる本番用ダッシュボード)にのみ必要な機能。
    # Stage1/Stage2はベンチ試験・開発用のツールであり、複数端末から同時に
    # 見たり操作したりできる方が開発効率が良いため、常に無制限にする。
    # このため制限の有無は共有のグローバルフラグではなく、呼び出し側
    # (make_dashboard_handler/make_mobile_handlerのenforce_single_controller
    # 引数)から渡すインスタンス単位の設定にしている。
    def __init__(self, timeout_sec: float = 8.0, enforce: bool = True) -> None:
        self._timeout_sec = timeout_sec
        self._enforce = enforce
        # 【2026-07-17変更】以前は全アカウント共有の単一ホルダー(IPのみ)だった
        # ため、TLとrestricted_accountのような別アカウント同士でも片方しか同時に
        # ログインできなかった。アカウント名をキーにした辞書にすることで、
        # 「TL1台+restricted_account1台」のように異なるアカウントは同時ログインを許可
        # しつつ、同一アカウントの複数端末は従来通り1台に制限する。
        self._holders: Dict[str, Tuple[str, float]] = {}  # account -> (ip, last_seen)
        self._lock = threading.Lock()

    def try_acquire(self, client_ip: str, account: str) -> bool:
        if not self._enforce:
            return True
        now = time.monotonic()
        with self._lock:
            holder = self._holders.get(account)
            if (
                holder is None
                or holder[0] == client_ip
                or (now - holder[1]) > self._timeout_sec
            ):
                self._holders[account] = (client_ip, now)
                return True
            return False

    def release(self, account: str) -> None:
        """明示的なログアウト時に即座に枠を解放する(通常はtimeout_sec経過待ち)。"""
        with self._lock:
            self._holders.pop(account, None)


# 【2026-10-13追加】同時1端末制限の有効/無効を、設定ファイル(enforce_single_controller、本番はtrue)に対して
# 環境変数NOKOSAHEN_ENFORCE_SINGLE_CONTROLLERで一時的に上書きする。開発・試験で複数の端末(有線とWi-Fiの
# 両方を持つPCなど)から同じアカウントで開きたいときだけ `NOKOSAHEN_ENFORCE_SINGLE_CONTROLLER=0` を付けて
# 起動する(設定ファイルは本番値のまま触らない)。0/false/off/no=無効、1/true/on/yes=有効、空・未設定=設定ファイルの値。
ENFORCE_SINGLE_CONTROLLER_ENV = "NOKOSAHEN_ENFORCE_SINGLE_CONTROLLER"


def resolve_enforce_single_controller(param_value: bool, env: Optional[Dict[str, str]] = None, logger: Any = None) -> bool:
    """設定ファイルの値`param_value`と環境変数から、実際に使う同時1端末制限の真偽を決める。"""
    raw = (os.environ if env is None else env).get(ENFORCE_SINGLE_CONTROLLER_ENV, "").strip().lower()
    if not raw:
        return bool(param_value)
    if raw in ("0", "false", "off", "no"):
        value = False
    elif raw in ("1", "true", "on", "yes"):
        value = True
    else:
        if logger is not None:
            logger.warn(f"{ENFORCE_SINGLE_CONTROLLER_ENV}={raw!r}は解釈できないため無視します(設定ファイルの値を使用)。")
        return bool(param_value)
    if logger is not None and value != bool(param_value):
        logger.warn(f"{ENFORCE_SINGLE_CONTROLLER_ENV}により同時1端末制限を{'有効' if value else '無効'}にしています(設定ファイルは{bool(param_value)})。")
    return value


_OCCUPIED_MESSAGE = "他の端末がこのアカウントで操作中です。しばらくしてから再度お試しください。"

# 【2026-10-13追加】同時1端末制限で2台目の端末が弾かれたときの案内ページ(画面を開いたとき用)。
# 以前は画面を開くと生のJSON({"error":"in_use",...})がそのまま表示されて意味が分からなかった。
# 相手が操作をやめて解放される(timeout_sec、既定8秒)まで5秒ごとに自動で読み直す。
_OCCUPIED_HTML = (
    "<!doctype html><html lang=\"ja\"><head><meta charset=\"utf-8\" />"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />"
    "<meta http-equiv=\"refresh\" content=\"5\" /><title>のこさへん - 他の端末が操作中</title>"
    "<style>body{font-family:system-ui,-apple-system,\"Segoe UI\",sans-serif;background:#f4f6f5;color:#16211d;"
    "display:flex;align-items:center;justify-content:center;min-height:100vh;margin:0}"
    ".c{background:#fff;border:1px solid #dce4df;border-radius:12px;padding:24px;max-width:340px;box-sizing:border-box}"
    "h1{font-size:17px;margin:0 0 10px}p{font-size:14px;line-height:1.6;margin:8px 0}a{color:#176b56}</style></head>"
    "<body><div class=\"c\" role=\"alert\"><h1>他の端末が操作中です</h1>"
    "<p>このアカウントは、いま別の端末で操作されています。同時に操作できるのは1台だけです。</p>"
    "<p>相手の操作が止まると(約10秒後)、このページは自動で読み直します。</p>"
    "<p>見るだけなら<a href=\"/login\">ログイン画面</a>の「閲覧専用(ゲスト)で見る」を使ってください。</p>"
    "</div></body></html>"
).encode("utf-8")


def _send_occupied_response(handler: BaseHTTPRequestHandler, as_page: bool) -> None:
    """操作権を他の端末が持っているときの409応答。画面は案内ページ、APIは従来どおりJSON(error=in_use)。"""
    if as_page:
        handler.send_response(HTTPStatus.CONFLICT)
        handler.send_header("Content-Type", "text/html; charset=utf-8")
        handler.send_header("Content-Length", str(len(_OCCUPIED_HTML)))
        handler.send_header("Cache-Control", "no-store")
        handler.end_headers()
        handler.wfile.write(_OCCUPIED_HTML)
        return
    handler._send_json(  # type: ignore[attr-defined]
        HTTPStatus.CONFLICT, {"error": "in_use", "message": _OCCUPIED_MESSAGE}
    )


def _resolve_control_account(
    handler: BaseHTTPRequestHandler,
    auth_username: str,
    gate: SingleControllerGate,
    sessions: SessionStore,
) -> Tuple[Optional[str], Optional[str]]:
    """セッションCookieからログイン済みアカウントを解決する。戻り値は`(account, error)`。

    `error`は`None`(通過)、`"auth_required"`(401/ログイン画面へ誘導すべき)、
    `"occupied"`(操作アカウントが他の端末で使用中、409にすべき)のいずれか。
    auth_usernameが空なら認証自体を無効化する(従来通り、Stage1等の開発用途)。

    ログイン時の資格情報チェック(ユーザー名/パスワード照合)は`/api/login`
    ハンドラ側で行い、ここでは発行済みセッショントークンの解決だけを行う。
    """
    if not auth_username:
        return "", None
    account = sessions.resolve(get_session_token(handler))
    if account is None:
        return None, "auth_required"
    if account != "guest":
        client_ip = handler.client_address[0]
        if not gate.try_acquire(client_ip, account):
            return None, "occupied"
    return account, None


_LOGIN_HTML = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>のこさへん - ログイン</title>
<style>
  :root { color-scheme: light; font-family: system-ui, -apple-system, "Segoe UI", sans-serif; background:#f4f6f5; color:#16211d; }
  html, body { margin:0; height:100%; }
  body { display:flex; align-items:center; justify-content:center; }
  .card { background:#fff; border:1px solid #dce4df; border-radius:12px; padding:28px; width:100%; max-width:320px; box-sizing:border-box; }
  .logo-title { display:flex; align-items:center; gap:10px; justify-content:center; margin-bottom:18px; }
  .logo-title img { height:40px; width:40px; border-radius:8px; }
  h1 { margin:0; font-size:17px; }
  label { display:block; font-size:13px; margin:12px 0 4px; }
  input { width:100%; box-sizing:border-box; font-size:16px; padding:8px; border:1px solid #cfd8d3; border-radius:6px; }
  button { width:100%; padding:11px; font-size:15px; font-weight:700; border:0; border-radius:8px; background:#176b56; color:#fff; margin-top:16px; cursor:pointer; }
  .guest-btn { background:#4e5e57; margin-top:8px; }
  #errorMsg { color:#993c1d; font-size:13px; margin-top:10px; min-height:1.2em; text-align:center; }
</style>
</head>
<body>
<div class="card">
  <div class="logo-title">
    <img src="__NOKOSAHEN_LOGO__" alt="のこさへん" />
    <h1>のこさへん ログイン</h1>
  </div>
  <form id="loginForm">
    <label for="username">ユーザー名</label>
    <input id="username" name="username" autocomplete="username" autocapitalize="off" />
    <label for="password">パスワード</label>
    <input id="password" name="password" type="password" autocomplete="current-password" />
    <button type="submit">ログイン</button>
  </form>
  <button id="guestBtn" class="guest-btn">閲覧専用(ゲスト)で見る</button>
  <p id="errorMsg"></p>
</div>
<script>
  async function login(username, password) {
    const errorMsg = document.getElementById("errorMsg");
    errorMsg.textContent = "";
    try {
      const res = await fetch("/api/login", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ username, password }),
      });
      if (res.ok) {
        location.href = "/";
        return;
      }
      errorMsg.textContent = res.status === 429
        ? "ログインの失敗が続いたため、しばらく(数分)ログインできません。時間をおいてお試しください。"
        : "ユーザー名またはパスワードが違います。";
    } catch (e) {
      errorMsg.textContent = "接続に失敗しました。しばらくしてから再度お試しください。";
    }
  }
  document.getElementById("loginForm").addEventListener("submit", (ev) => {
    ev.preventDefault();
    login(document.getElementById("username").value, document.getElementById("password").value);
  });
  document.getElementById("guestBtn").addEventListener("click", () => login("guest", ""));
</script>
</body>
</html>
""".replace("__NOKOSAHEN_LOGO__", LOGO_DATA_URI)

_LOGIN_HTML_BYTES = _LOGIN_HTML.encode("utf-8")


def _send_redirect_to_login(handler: BaseHTTPRequestHandler) -> None:
    handler.send_response(HTTPStatus.FOUND)
    handler.send_header("Location", "/login")
    handler.send_header("Content-Length", "0")
    handler.end_headers()


_AUDIT = logging.getLogger("nokosahen.audit")
if not _AUDIT.handlers:
    # 監査ログ(誰が・いつ・何を): 時刻つきで標準エラー(ROSのログ)へ。パスワード・ハッシュ・本文は出さない。
    _audit_handler = logging.StreamHandler()
    _audit_handler.setFormatter(logging.Formatter("%(asctime)s [AUDIT] %(message)s"))
    _AUDIT.addHandler(_audit_handler)
    _AUDIT.setLevel(logging.INFO)


class _DenyLog:
    """operatorの拒否(403)の直近の記録。同じ(アカウント, パス)の連続した拒否(例: 手動運転の画面が毎100msに送る停止指令)で
    監査ログと一覧が埋まらないよう、同じ拒否は`window_sec`秒に1回だけ記録し、省略した回数を次の記録に添える。"""

    def __init__(self, window_sec: float = 60.0) -> None:
        self.recent: Deque[Dict[str, Any]] = deque(maxlen=50)
        self._seen: Dict[Tuple[str, str], List[float]] = {}
        self._window = window_sec
        self._lock = threading.Lock()

    def note(self, account: str, path: str, record: Dict[str, Any]) -> Tuple[bool, int]:
        """(記録するか, 省略した回数)。"""
        now = time.monotonic()
        with self._lock:
            ent = self._seen.get((account, path))
            if ent is not None and now - ent[0] < self._window:
                ent[1] += 1
                return False, 0
            skipped = int(ent[1]) if ent else 0
            self._seen[(account, path)] = [now, 0]
            if len(self._seen) > 512:
                self._seen.clear()
            self.recent.append(record)
        return True, skipped


def _whoami_payload(accounts_db: AccountStore, eph: EphemeralOverrides, account: str) -> Dict[str, Any]:
    """`/api/whoami`の中身(ロール・operatorの許可権限・一時的な上書きの件数)。画面の無効化用(権限チェックの本体はサーバー側)。"""
    role = accounts_db.role_of(account)
    who: Dict[str, Any] = {"account": account, "role": role}
    if role == _roles.ROLE_OPERATOR:
        who["permissions"] = sorted(accounts_db.operator_permissions() | _roles.ROLE_BASE_CAPS[_roles.ROLE_OPERATOR])
        who["admin_only"] = sorted(_roles.NOT_GRANTABLE)
    if role in (_roles.ROLE_ADMIN, _roles.ROLE_OPERATOR):
        who["ephemeral_count"] = eph.count()
    return who


def _manual_person_guard_payload(teleop: "TeleopController", accounts_db: AccountStore, account: str) -> Dict[str, Any]:
    """`GET /api/manual_person_guard`の中身。can_changeは画面の無効化用(権限チェックの本体はPOST側のサーバー判定)。"""
    st = teleop.person_guard_state()
    role = accounts_db.role_of(account)
    can = "drive.person_guard" in _roles.role_caps(role, accounts_db.operator_permissions())
    return {
        "enabled": bool(st["enabled"]),
        "blocked": bool(st["blocked"]),
        "near_person_stop": bool(st["near_person_stop"]),
        "detector_alive": bool(st["detector_alive"]),
        "can_change": bool(can),
    }


def _capability_gate(
    handler: BaseHTTPRequestHandler,
    account: str,
    path: str,
    data: dict,
    accounts_db: AccountStore,
    denied_log: "_DenyLog",
    role: Optional[str] = None,
) -> bool:
    """書き込みAPIの権限チェック(サーバー側で必ず行う。画面の無効化は補助)。通ればTrue、403を返したらFalse。

    【2026-10-13追加】docs/accounts.md。adminは常に通す(従来動作のまま)。operatorは roles.API_RULES の
    必要な権限を全て持っているときだけ通す(表に無いAPI=admin専用)。ロールはセッションのアカウント名から
    サーバー側で毎回決める(クライアントの申告は使わない)。拒否・許可はどちらも監査ログに残す。
    """
    # 【2026-11-06】呼び出し側が role_of の結果を渡せる(運転POSTは100 msごと。role_of は毎回アカウント/権限ファイルを stat するので、1リクエストで2回呼ばない)
    if role is None:
        role = accounts_db.role_of(account)
    if role == _roles.ROLE_ADMIN:
        return True
    ip = handler.client_address[0] if getattr(handler, "client_address", None) else "?"
    dec = _roles.authorize(role, accounts_db.operator_permissions(), "POST", path, data)
    if dec.allowed:
        _AUDIT.info("ALLOW account=%s role=%s ip=%s POST %s caps=%s", account, role, ip, path, ",".join(dec.required))
        return True
    logged, skipped = denied_log.note(account, path, {
        "at": datetime.now().isoformat(timespec="seconds"), "account": account, "role": role,
        "ip": ip, "path": path, "missing": list(dec.missing),
    })
    if logged:
        _AUDIT.warning("DENY account=%s role=%s ip=%s POST %s need=%s missing=%s%s", account, role, ip, path,
                       ",".join(dec.required), ",".join(dec.missing),
                       f" (直近の同じ拒否 {skipped} 回は省略)" if skipped else "")
    handler._send_json(  # type: ignore[attr-defined]
        HTTPStatus.FORBIDDEN,
        {
            "error": "forbidden_capability",
            "message": "この操作は管理者のみです。" if dec.admin_only
            else "この操作は許可されていません(管理者に権限の付与を依頼してください)。",
            "admin_only": dec.admin_only,
            "required": list(dec.required),
            "missing": list(dec.missing),
        },
    )
    return False


def _post_origin_guard(handler: Any) -> bool:
    """管理画面(PC版8090・スマホ版8095)のすべてのPOST用。別サイトからのブラウザのPOST(Origin が別ホスト・Sec-Fetch-Site が cross-site/same-site)は 403 で断る。
    Origin も Sec-Fetch-Site も無い(curl・内部呼び出し・テスト)は通す。判定は display_routes.origin_ok(iPad側と共通)。"""
    try:
        ok = _display_routes.origin_ok(handler.headers)
    except Exception:  # noqa: BLE001  (判定の不具合で操作を止めない=従来どおり通す)
        return True
    if ok:
        return True
    err = "bad_origin" if urlparse(handler.path).path.startswith("/display/") else "cross_origin"   # iPad側(/display)は従来のエラー名のまま
    # 【2026-11-05】本文を読まずに403を返して閉じると、未読の本文が残ったまま接続が閉じられ、TCPのリセットで相手が403を読めないことがある
    # (Windowsの WinError 10053 で test_admin_origin_guard が時々落ちていた原因)。小さい本文は読み捨ててから返す。
    try:
        _n = int(handler.headers.get("Content-Length") or 0)
        if 0 < _n <= 65536:
            handler.rfile.read(_n)
    except Exception:  # noqa: BLE001
        pass
    handler._send_json(HTTPStatus.FORBIDDEN, {"error": err, "message": "別のサイトからの操作は受け付けません(この画面を開き直してください)。"})
    return False


def _handle_login_post(
    handler: BaseHTTPRequestHandler,
    data: dict,
    accounts: Dict[str, str],
    sessions: SessionStore,
    account_store: Optional[AccountStore] = None,
    limiter: Optional[LoginLimiter] = None,
) -> None:
    """`/api/login`の共通処理(PC版・スマホ版で共有)。

    【2026-10-13】operatorアカウント(account_storeに定義されたもの)は、ハッシュ照合+失敗回数制限(ロックアウト)でログインする。
    【2026-11-04】回数制限は名前によらず全ログインに掛ける(TL・restricted_accountの総当たり対策)。guestだけは対象外(パスワード無しの閲覧用)。
    - operator: 従来どおり キー=(アカウント名, 接続元IP全体)。
    - それ以外(TL・restricted_account・未知の名前): キー=(アカウント名, 接続元IP+アカウント名)。IP全体では数えないので、
      他のアカウント(operator等)の失敗がTLをロックしない。ロック中は正しいパスワードでも429(retry_after_sec)。成功でリセット。
    """
    username = str(data.get("username", ""))
    password = str(data.get("password", ""))
    ip = handler.client_address[0] if getattr(handler, "client_address", None) else "?"
    is_operator = account_store is not None and account_store.is_operator_name(username)
    limited = limiter is not None and username != "guest"
    uname = username[:64]  # キーの肥大化防止(長い名前を大量に送られても1キーが大きくならない)
    keys = (f"user:{uname}", f"ip:{ip}") if is_operator else (f"user:{uname}", f"ipuser:{ip}|{uname}")
    if limited:
        wait = limiter.retry_after(keys)
        if wait > 0:
            _AUDIT.warning("LOGIN_LOCKED account=%s ip=%s retry_after=%ds", uname, ip, int(wait) + 1)
            body = json.dumps(
                {"ok": False, "error": "too_many_attempts", "retry_after_sec": int(wait) + 1}, ensure_ascii=False
            ).encode("utf-8")
            handler.send_response(HTTPStatus.TOO_MANY_REQUESTS)
            handler.send_header("Content-Type", "application/json; charset=utf-8")
            handler.send_header("Retry-After", str(int(wait) + 1))
            handler.send_header("Content-Length", str(len(body)))
            handler.end_headers()
            handler.wfile.write(body)
            return
    if is_operator:
        account = username if account_store.verify_operator(username, password) else None
    else:
        account = check_login_credentials(username, password, accounts)
    if limited:
        if account is not None:
            limiter.record_success(keys)
        else:
            limiter.record_failure(keys)
            _AUDIT.warning("LOGIN_FAILED account=%s ip=%s", uname, ip)
    if account is None:
        body = json.dumps(
            {"ok": False, "error": "invalid_credentials"}, ensure_ascii=False
        ).encode("utf-8")
        handler.send_response(HTTPStatus.UNAUTHORIZED)
        handler.send_header("Content-Type", "application/json; charset=utf-8")
        handler.send_header("Content-Length", str(len(body)))
        handler.end_headers()
        handler.wfile.write(body)
        return
    token = sessions.create(account)
    if account_store is not None:
        account_store.note_login(account)
    body = json.dumps({"ok": True, "account": account}, ensure_ascii=False).encode("utf-8")
    handler.send_response(HTTPStatus.OK)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Set-Cookie", session_cookie_header(token, handler))
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


def _handle_logout_post(
    handler: BaseHTTPRequestHandler,
    account: str,
    gate: SingleControllerGate,
    sessions: SessionStore,
) -> None:
    """`/api/logout`の共通処理(PC版・スマホ版で共有)。"""
    gate.release(account)
    sessions.revoke(get_session_token(handler))
    body = json.dumps({"ok": True}, ensure_ascii=False).encode("utf-8")
    handler.send_response(HTTPStatus.OK)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Set-Cookie", clear_session_cookie_header(handler))
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


def _speed_settings_payload(
    teleop: TeleopController,
    auto_speed: Optional[AutoSpeedController],
    nav_speed: Optional[NavSpeedController] = None,
) -> Dict[str, Any]:
    payload = dict(teleop.status())
    if auto_speed is not None:
        payload.update(auto_speed.status())
    # 【2026-10-07追加】Nav2(ゴール追従)の最高速度。behavior_directorの自動速度
    # (auto_linear_mps)とは別のつまみ。ここは再読込なしのキャッシュ値(表示用)。
    if nav_speed is not None:
        payload.update(nav_speed.status())
    # UI表示用: 直進速度の上限(ファーム側の速度制限)。値は全て「実際に適用された(丸め後の)値」。
    payload["max_linear_mps"] = MAX_DRIVE_LINEAR_MPS  # 自律側(director自動・Nav2)の直進上限。手動は下のlimits
    # 【2026-10-10】手動/自動それぞれの上限と、手動の確認しきい値(画面のmax属性・確認バーはこれを使う)。
    payload["limits"] = manual_speed_limits()
    return payload


_SPEED_SETTING_LABELS = {
    "manual_linear_mps": "手動直進",
    "manual_angular_rps": "手動旋回",
    "auto_linear_mps": "自動移動直進",
    "auto_angular_rps": "自動移動旋回",
}


def _speed_settings_response(
    data: Dict[str, Any],
    teleop: TeleopController,
    auto_speed: Optional[AutoSpeedController],
    nav_speed: Optional[NavSpeedController] = None,
) -> Dict[str, Any]:
    """POST /api/speed_settings の応答。適用後の値に加え、要求値が上限/下限で
    丸められた項目を `clamped`(要求値・適用値)と `message` で返す。"""
    payload = _speed_settings_payload(teleop, auto_speed, nav_speed)
    clamped = []
    for key, label in _SPEED_SETTING_LABELS.items():
        if key not in payload or data.get(key) in (None, ""):
            continue
        try:
            requested = float(data[key])
        except (TypeError, ValueError):
            continue
        applied = float(payload[key])
        if abs(requested - applied) > 1e-6:
            clamped.append({"key": key, "label": label, "requested": requested, "applied": applied})
    payload["clamped"] = clamped
    if clamped:
        payload["message"] = "指定値が上限/下限を超えていたため丸めました: " + "、".join(
            f"{c['label']} {c['requested']:g}→{c['applied']:g}" for c in clamped
        )
    return payload


def _apply_speed_settings(
    data: Dict[str, Any],
    teleop: TeleopController,
    auto_speed: Optional[AutoSpeedController],
) -> Optional[str]:
    """dataから最高速度設定を反映する。不正な値があればエラーメッセージを返す。

    PC版(`make_dashboard_handler`)・スマホ版(`make_mobile_handler`)の両方の
    `/api/speed_settings`から共通で使う。
    """

    def _optional_float(key: str) -> Optional[float]:
        if key not in data or data[key] in (None, ""):
            return None
        try:
            value = float(data[key])
        except (TypeError, ValueError):
            raise ValueError(key)
        # 【2026-10-02追加】json.loadsはNaN/Infinityを受け付けるため拒否する。
        if not math.isfinite(value):
            raise ValueError(key)
        return value

    try:
        manual_linear = _optional_float("manual_linear_mps")
        manual_angular = _optional_float("manual_angular_rps")
        auto_linear = _optional_float("auto_linear_mps")
        auto_angular = _optional_float("auto_angular_rps")
    except ValueError as exc:
        return f"{exc.args[0]} must be a number"

    if manual_linear is not None or manual_angular is not None:
        teleop.set_max_speed(manual_linear, manual_angular)
    if auto_speed is not None and (auto_linear is not None or auto_angular is not None):
        auto_speed.set_max_speed(auto_linear, auto_angular)
    return None


def _manual_high_check(data: Dict[str, Any], teleop: TeleopController) -> List[str]:
    """手動の要求値で確認が要る項目名(数値でない値はここでは無視。検証は_apply_speed_settingsが行う)。"""
    vals: Dict[str, Optional[float]] = {}
    for key in ("manual_linear_mps", "manual_angular_rps"):
        try:
            v = float(data[key]) if key in data and data[key] not in (None, "") else None
        except (TypeError, ValueError):
            v = None
        vals[key] = v if v is not None and math.isfinite(v) else None
    return manual_confirm_needed(vals["manual_linear_mps"], vals["manual_angular_rps"],
                                 teleop.linear_speed, teleop.angular_speed)


def _speed_settings_post(
    data: Dict[str, Any],
    teleop: TeleopController,
    auto_speed: Optional[AutoSpeedController],
    nav_speed: Optional[NavSpeedController] = None,
) -> Tuple[int, Dict[str, Any]]:
    """POST /api/speed_settings の本体。(HTTPステータス, 応答)を返す。

    手動/自動(director)の速度に加え、【2026-10-07追加】`nav_max_speed_mps`(Nav2の
    最高速度)も受け付ける。Nav2側が不正値/要確認(0.6 m/s超でconfirm_highなし)の場合は、
    他の項目も含めて何も書かずに400で返す。Nav2が未起動なら他の項目は保存した上で
    409(値は次回起動用に記憶)。この関数はパラメータを書くだけでcmd_velは出さない。"""
    # 【2026-10-07追加】`all_linear_mps`: 手動直進・director自動直進・Nav2最高速度を同じ値にする
    # (旋回は変えない)。値は[0.1,1.1]に丸める。0.6超はconfirm_high必須。Nav2が未起動でも
    # 手動とdirectorは反映し、結果は`unified.targets`に対象ごとに返す(Nav2未起動は409)。
    unify = False
    if "all_linear_mps" in data and data["all_linear_mps"] not in (None, ""):
        unify = True
        raw_all = data["all_linear_mps"]
        try:
            v_all = normalize_nav_speed(raw_all)
        except ValueError:
            return HTTPStatus.BAD_REQUEST, {
                "error": "all_linear_mps must be a finite number",
                "message": "全て同じ速度にする値は有限の数値で指定してください。",
            }
        if v_all > NAV_SPEED_CONFIRM_ABOVE and data.get("confirm_high") is not True:
            return HTTPStatus.BAD_REQUEST, {
                "error": "confirm_required", "confirm_required": True,
                "all_linear_mps": v_all, "nav_max_speed_mps": v_all,
                "message": f"{NAV_SPEED_CONFIRM_ABOVE:g} m/s を超える速度には確認が必要です(confirm_high)。",
            }
        data = dict(data)
        data["manual_linear_mps"] = v_all
        data["auto_linear_mps"] = v_all
        data["nav_max_speed_mps"] = v_all
    nav_requested = data.get("nav_max_speed_mps") not in (None, "") if "nav_max_speed_mps" in data else False
    if nav_requested:
        if nav_speed is None and unify:
            pass  # 統一操作ではNav2側だけ「利用不可」として報告し、手動/directorは反映する
        elif nav_speed is None:
            return HTTPStatus.NOT_FOUND, {
                "error": "nav_speed_unavailable",
                "message": "この画面ではNav2の最高速度は変更できません(PC版の速度設定を使ってください)。",
            }
        pre = precheck_nav_request(data["nav_max_speed_mps"], data.get("confirm_high"))
        if pre is not None:
            return pre
    # 【2026-10-10追加】手動の新しい高い領域(直進>1.1 / 旋回>3.0)へ「変える」には確認(confirm_high)が必要。
    # 何も書かずに400(他の項目も保存しない)。値が現在値と同じ項目は対象外(他の欄を直しただけで再確認しない)。
    need = _manual_high_check(data, teleop)
    if need and data.get("confirm_high") is not True:
        return HTTPStatus.BAD_REQUEST, {
            "error": "confirm_required", "confirm_required": True, "manual_high": need,
            "message": "手動の速度を高い値に上げるには確認が必要です(confirm_high)。広い場所で、電源を切れる状態で行ってください。",
        }
    err = _apply_speed_settings(data, teleop, auto_speed)
    if err:
        return HTTPStatus.BAD_REQUEST, {"error": err}
    resp = _speed_settings_response(data, teleop, auto_speed, nav_speed)
    if unify:
        resp.pop("clamped", None)
        resp.pop("message", None)
    if not nav_requested:
        return HTTPStatus.OK, resp
    if nav_speed is None:
        status, nav_payload = HTTPStatus.NOT_FOUND, {
            "error": "nav_speed_unavailable", "message": "この画面ではNav2の最高速度は変更できません。"}
    else:
        status, nav_payload = nav_speed.apply(data["nav_max_speed_mps"], data.get("confirm_high") is True)
    resp["nav"] = nav_payload
    resp["nav_max_speed_mps"] = nav_payload.get("nav_max_speed_mps", resp.get("nav_max_speed_mps"))
    msgs = [m for m in (resp.get("message"), nav_payload.get("message")) if m]
    resp["message"] = " ".join(msgs)
    if status != HTTPStatus.OK:
        resp["error"] = nav_payload.get("error", "nav_speed_failed")
    if unify:
        resp["unified"] = _unified_report(raw_all, v_all, resp, status, nav_payload)
        resp["message"] = resp["unified"]["message"]
    return status, resp


def _unified_report(
    raw: Any, v: float, resp: Dict[str, Any], nav_status: int, nav_payload: Dict[str, Any]
) -> Dict[str, Any]:
    """『全て同じ速度にする』の対象ごとの結果(成功/失敗/Nav2未起動)と丸め情報。"""
    targets = []
    for key, label in (("manual_linear_mps", "手動直進"), ("auto_linear_mps", "director自動直進(接近・見回り)")):
        applied = resp.get(key)
        if applied is not None and abs(float(applied) - v) < 1e-6:
            targets.append({"key": key, "label": label, "status": "成功", "applied": float(applied)})
        else:
            targets.append({"key": key, "label": label, "status": "失敗",
                            "detail": "この画面では設定できません" if applied is None else f"適用値 {applied}"})
    if nav_status == HTTPStatus.OK and nav_payload.get("ok"):
        nav = {"status": "成功", "applied": nav_payload.get("nav_max_speed_mps")}
    elif nav_status == HTTPStatus.CONFLICT:
        nav = {"status": "Nav2未起動", "detail": nav_payload.get("message", "")}
    else:
        nav = {"status": "失敗", "detail": nav_payload.get("message") or nav_payload.get("error", "")}
    nav.update({"key": "nav_max_speed_mps", "label": "自動走行(Nav2)"})
    targets.append(nav)
    clamped = abs(float(raw) - v) > 1e-9
    parts = [f"{t['label']}: {t['status']}" for t in targets]
    msg = f"全て {v:g} m/s にしました。" if all(t["status"] == "成功" for t in targets) else f"全て {v:g} m/s を指定しました。"
    if clamped:
        msg += f" 指定値 {float(raw):g} は範囲外のため {v:g} に丸めました(範囲 {NAV_SPEED_MIN:g}〜{NAV_SPEED_MAX:g})。"
    msg += " " + " / ".join(parts)
    if nav["status"] != "成功":
        msg += f" ({nav.get('detail', '')})"
    return {"requested": float(raw), "applied": v, "clamped": clamped, "targets": targets, "message": msg}


def make_dashboard_handler(
    html_getter: Callable[[], bytes],
    state_getter: Callable[[], Dict[str, Any]],
    teleop: TeleopController,
    map_goal: Optional[MapGoalController] = None,
    scan: Optional[ScanController] = None,
    camera: Optional[CameraPreviewController] = None,
    camera_rear: Optional[Dict[str, CameraPreviewController]] = None,
    checkout_cams: Optional[CheckoutCamRegistry] = None,
    auto_speed: Optional[AutoSpeedController] = None,
    nav_speed: Optional[NavSpeedController] = None,
    pid_gains: Optional[PidGainsController] = None,
    mapping: Optional[MappingController] = None,
    map_lifecycle: Optional[MapLifecycleController] = None,
    boundary: Optional[BoundaryController] = None,
    keepout_zones: Optional[KeepOutZoneController] = None,
    map_editor_html_getter: Optional[Callable[[], bytes]] = None,
    crowd_background: Optional[CrowdBackgroundController] = None,
    frontier_explorer: Optional[FrontierExplorerController] = None,
    consumption_log: Optional[ConsumptionLog] = None,
    consumption_graph_html_getter: Optional[Callable[[], bytes]] = None,
    utterance_log: Optional[UtteranceLogReader] = None,
    utterance_log_html_getter: Optional[Callable[[], bytes]] = None,
    detection_log: Optional[DetectionLog] = None,
    detection_log_html_getter: Optional[Callable[[], bytes]] = None,
    step_response_results: Optional[StepResponseResultsController] = None,
    step_response_html_getter: Optional[Callable[[], bytes]] = None,
    engagement_log: Optional[EngagementLog] = None,
    recommendation_effectiveness_html_getter: Optional[Callable[[], bytes]] = None,
    activity_report_html_getter: Optional[Callable[[], bytes]] = None,
    audience_display_html_getter: Optional[Callable[[], bytes]] = None,
    behavior_control: Optional[BehaviorDirectorControlController] = None,
    behavior_rules: Optional[BehaviorRulesController] = None,
    behavior_rules_html_getter: Optional[Callable[[], bytes]] = None,
    behavior_tree: Optional[BehaviorTreeController] = None,
    behavior_tree_html_getter: Optional[Callable[[], bytes]] = None,
    schedule_html_getter: Optional[Callable[[], bytes]] = None,  # 【2026-10-23】かんたん時間割(/schedule)
    schedule: Optional[ScheduleController] = None,
    bgm: Optional[BgmProxy] = None,
    voice_master: Optional[VoiceMasterProxy] = None,
    voice_levels: Optional[VoiceLevelsProxy] = None,
    survey_admin: Optional[SurveyAdminProxy] = None,
    ocr_admin: Optional[OcrAdminProxy] = None,
    auth_username: str = "",
    auth_password: str = "",
    enforce_single_controller: bool = True,
    voice_settings: Optional[VoiceSettingsController] = None,
    voice_settings_html_getter: Optional[Callable[[], bytes]] = None,
    localization: Optional["LocalizationAssistController"] = None,
    pins: Optional["PinController"] = None,
    account_store: Optional[AccountStore] = None,
    ephemeral: Optional[EphemeralOverrides] = None,
    startup_reset: bool = False,
    announce_events: Optional[AnnounceEventsController] = None,
    pr_intro: Optional[PrIntroController] = None,
    announce_settings_html_getter: Optional[Callable[[], bytes]] = None,
    system_shutdown: Optional[SystemShutdownController] = None,
    wall_escape: Optional["WallEscapeController"] = None,
    display_hub: Optional[Any] = None,
    display_auth_username: str = "ADMIN_ACCOUNT",
    display_auth_password: str = "",
):
    """`/`, `/api/state`, `/api/teleop`, `/api/soft_estop`(, `/api/map`, `/api/goal`, `/api/wall_escape/status`, `/api/wall_escape/trigger`,
    `/api/goal/status`, `/api/goal/cancel`, `/api/pins`(+`/add`,`/update`,`/delete`,`/goto`),
    `/api/resume_autonomy`, `/api/scan`, `/api/camera`, `/api/checkout_cams`,
    `/api/checkout_cam_image`, `/api/checkout_cam_assign`, `/api/speed_settings`,
    `/api/mapping/status`, `/api/mapping/save`, `/api/mapping/lifecycle_status`,
    `/api/mapping/list`, `/api/mapping/start`, `/api/mapping/stop`,
    `/api/mapping/reset`, `/api/mapping/load`, `/api/localization`,
    `/api/initial_pose`, `/api/global_localization`, `/api/boundary`,
    `/api/boundary/set`, `/api/boundary/clear`, `/api/map_view`,
    `/api/map_view/set`, `/api/map_view/estimate`, `/api/map_view/save`, `/map_editor`,
    `/api/keepout_zones`, `/api/keepout_zones/add`, `/api/keepout_zones/remove`,
    `/api/keepout_zones/clear`, `/api/crowd_background/calibrate`,
    `/api/exploration/start`, `/api/exploration/stop`, `/api/exploration/status`,
    `/api/whoami`, `/step_response`, `/api/step_response/list`,
    `/api/step_response/result`, `/api/step_response/run`,
    `/api/step_response/run_status`, `/api/bgm`, `/behavior_rules`,
    `/api/behavior_rules`(+`/state`,`/preview`,`/example`,`/save`,`/reload`),
    `/voice_settings`, `/api/voice_settings`(+`/metrics`,`/save`,`/preview`,`/speak`))を持つHTTPハンドラを
    組み立てる。

    auth_username/auth_passwordを両方とも空(既定)にすると認証無し(従来通り)。
    どちらかを設定すると、認証が必要になる。実際に使えるアカウントは2種類:
    - `auth_username`/`auth_password`(例: TL): フル操作可能。ただし
      `enforce_single_controller`が真の間は同時に1台の端末だけ(この関数
      呼び出し1回=1ポート単位でカウント)。
    - `guest`(パスワード無し): 人数無制限だが閲覧専用(POST系は全て拒否)。

    `enforce_single_controller`は完成版(full_dashboard_node)でのみ真にする
    想定(実運用では同時操作の衝突防止が必要なため)。Stage1/Stage2はベンチ
    試験・開発用のツールなので偽にして、複数端末から同時にアクセスできる
    ようにする。
    """
    gate = SingleControllerGate(enforce=enforce_single_controller)
    sessions = SessionStore()
    login_accounts = {auth_username: auth_password, "guest": ""}
    # 【2026-10-13追加】operator(弱いアカウント)の枠組み。既定は無効(プレースホルダーのみ)。docs/accounts.md
    accounts_db = account_store if account_store is not None else AccountStore()
    accounts_db.set_reserved(["guest", auth_username])
    login_limiter = LoginLimiter()
    eph = ephemeral if ephemeral is not None else EphemeralOverrides()
    denied_log = _DenyLog()

    # 【2026-10-26追加】iPad表示 `/display`(docs/ipad_display.md)。決まったパスだけを、ダッシュボードの認証より前に処理する(do_GET/do_POSTの先頭)。
    # 【2026-10-27変更】最初に1回だけ、厨房サイト(web_ui)と同じアカウント・パスワードでログインする専用の方式にした。セッションはこのPCのメモリ上だけで、
    # 期限は再起動まで(上の SessionStore / SingleControllerGate / LoginLimiter とは別の入れ物。TLの管理画面のログインとは互いに入れない)。
    # 許す範囲は display_routes.py の冒頭を参照。パスワードが解決できない/空のときは誰もログインできない(display_auth_password は空のまま渡さない)。
    display_ctx = None
    if display_hub is not None:
        display_ctx = _display_routes.DisplayContext(
            display_hub, _build_display_html, _build_display_login_html,
            _display_routes.DisplayAuth(display_auth_username, display_auth_password),
            audit=lambda level, text: getattr(_AUDIT, level, _AUDIT.info)(text),
            build=_display_build_id(),
        )

    def _eph_restorers():
        return build_restorers(getattr(voice_settings, "announce", None), survey_admin)

    def _eph_remote_urls():
        return [
            getattr(voice_levels, "base_url", ""), getattr(voice_master, "base_url", ""),
            getattr(bgm, "base_url", ""), getattr(getattr(voice_settings, "proxy", None), "base_url", ""),
        ]

    def _eph_baseline(item: str):
        """operator書き込みの直前の値(外部に実値が書かれる項目だけ。起動時にここへ戻す)。"""
        try:
            if item == "announce.min_interval" and voice_settings is not None:
                return voice_settings.announce.read().get("announce_min_interval_seconds")
            if item == "display.survey_mode" and survey_admin is not None:
                return ((survey_admin.get_state() or {}).get("state") or {}).get("mode")
        except Exception:  # noqa: BLE001
            return None
        return None

    def _eph_prepare(path: str, data: dict, persist: bool) -> Dict[str, Any]:
        base: Dict[str, Any] = {}
        if persist:
            return base
        for item in _eph_items_for(path, data):
            if item in JOURNALED_ITEMS and not eph.has_item(item):
                base[item] = _eph_baseline(item)
        return base

    def _eph_commit(path: str, data: dict, account: str, persist: bool, base: Dict[str, Any],
                    keep: Optional[Callable[[str], bool]] = None) -> None:
        """保存が成功した項目について、admin=一時上書きを破棄(adminの値が永続の正) / operator=台帳に記録。"""
        items = {k: v for k, v in _eph_items_for(path, data).items() if keep is None or keep(k)}
        if persist:
            eph.discard(items)
            return
        for item, value in items.items():
            eph.record(item, value, account, restore=JOURNALED_ITEMS.get(item), baseline=base.get(item))

    if startup_reset:
        # 起動時: 前回のoperatorの一時的な変更を元に戻し、voice_bridgeのメモリ上の一時上書きを破棄する。
        start_startup_reset(eph, _eph_restorers(), _eph_remote_urls())

    def _view_store():
        """地図の表示回転(MapViewStore)。mapping/map_lifecycleが共有しているものを使う。"""
        src = mapping if mapping is not None else map_lifecycle
        return getattr(src, "view_store", None)

    class Handler(BaseHTTPRequestHandler):
        # 【2026-10-21】読み取りのタイムアウト。本文を送らない接続・リクエスト行を送らない接続を大量に張られても、
        # スレッドとソケットが永久に残らないようにする(1台目はCPU負荷が高く、固まると緊急停止の操作も届かなくなる)。
        timeout = 15
        def log_message(self, fmt: str, *args: Any) -> None:  # noqa: A003
            pass

        def _boundary_rect_now(self) -> Optional[Dict[str, float]]:
            """有効な走行可能範囲の回転矩形(無効/未設定/読めないときはNone)。目的地を壁から離すときの範囲に使う。"""
            if boundary is None:
                return None
            try:
                st = boundary.status()
                return rect_from_state(st) if st.get("enabled") else None
            except Exception:  # noqa: BLE001
                return None

        def _goal_guard(self, x: float, y: float) -> bool:
            """目的地を送る前の共通ガード(/api/goalと/api/pins/gotoで共有)。拒否したら応答を送ってFalse。"""
            # 【2026-10-06追加】走行可能範囲(回転矩形)の外への目的地は拒否する。
            # 範囲未設定/解除中は従来どおり制限なし。
            # 【2026-11-06】軽量の位置推定モードでは、Nav2が無いので目的地を受け付けない。
            if map_lifecycle is not None and map_lifecycle.is_light():
                self._send_json(
                    HTTPStatus.CONFLICT, {"ok": False, "error": "light_localization", "message": LIGHT_MODE_MESSAGE}
                )
                return False
            if boundary is not None and not boundary.allows_point(x, y):
                self._send_json(
                    HTTPStatus.CONFLICT,
                    {
                        "ok": False,
                        "error": "outside_boundary",
                        "message": "目的地が走行可能範囲(行動範囲)の外です。範囲の内側をクリックしてください。",
                    },
                )
                return False
            if teleop.soft_estop_engaged:
                # 【2026-09-30追加】ソフト緊急停止中は移動指令を受け付けない。
                self._send_json(
                    HTTPStatus.CONFLICT,
                    {"error": "estop_engaged", "message": "緊急停止中です。解除してから操作してください。"},
                )
                return False
            return True

        def _send_pin_error(self, exc: PinError) -> None:
            status = {
                "bad_request": HTTPStatus.BAD_REQUEST,
                "not_found": HTTPStatus.NOT_FOUND,
            }.get(exc.kind, HTTPStatus.CONFLICT)
            self._send_json(status, {"ok": False, "error": exc.kind, "message": exc.message})

        def _authorize(self, require_write: bool, redirect: bool = False) -> Optional[str]:
            account, err = _resolve_control_account(self, auth_username, gate, sessions)
            if err == "auth_required":
                if redirect:
                    _send_redirect_to_login(self)
                else:
                    self._send_json(HTTPStatus.UNAUTHORIZED, {"error": "auth_required"})
                return None
            if err == "occupied":
                _send_occupied_response(self, redirect)
                return None
            if require_write and account == "guest":
                self._send_json(
                    HTTPStatus.FORBIDDEN,
                    {"error": "guest_read_only", "message": "guestアカウントは閲覧専用です。"},
                )
                return None
            return account

        def _send(self, status: int, body: bytes, content_type: str) -> None:
            body = inject_role_ui(body, content_type, urlparse(self.path).path)  # operator用の画面制御(NKRole)
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _send_json(self, status: int, payload: dict) -> None:
            self._send(
                status,
                json.dumps(payload, ensure_ascii=False).encode("utf-8"),
                "application/json; charset=utf-8",
            )

        def _read_json(self) -> dict:
            # 【2026-10-06】生ボディを self._raw_body に残す(ルール保存はNaN等を厳格に拒否する
            # ため別途パースする)。ルール保存だけは上限を大きくする(100件のルールで64KBを超えうる)。
            self._raw_body = None
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                return {}
            limit = (
                RULES_MAX_BODY_BYTES
                if urlparse(self.path).path == "/api/behavior_rules/save"
                else TREE_MAX_BODY_BYTES
                if urlparse(self.path).path in ("/api/behavior_tree", "/api/behavior_tree/validate",
                                                "/api/schedule", "/api/schedule/preview")
                else 1024 * 64
            )
            if length <= 0 or length > limit:
                return {}
            try:
                self._raw_body = self.rfile.read(length)
                data = json.loads(self._raw_body.decode("utf-8"))
            except Exception:  # noqa: BLE001
                return {}
            return data if isinstance(data, dict) else {}

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            # 【2026-10-26追加】iPad表示 /display は専用のログイン(厨房サイトと同じアカウント。最初に1回だけ)。ダッシュボードの認証より前に処理する。
            if display_ctx is not None and _display_routes.is_display_path(path):
                _display_routes.handle_get(self, path, urlparse(self.path).query, display_ctx)
                return
            if path == "/login":
                self._send(HTTPStatus.OK, _LOGIN_HTML_BYTES, "text/html; charset=utf-8")
                return
            is_page = (
                path in ("/", "/index.html")
                or (path == "/map_editor" and map_editor_html_getter is not None)
                or (
                    path == "/consumption_graph"
                    and consumption_graph_html_getter is not None
                )
                or (path == "/utterance_log" and utterance_log_html_getter is not None)
                or (path == "/detection_log" and detection_log_html_getter is not None)
                or (path == "/step_response" and step_response_html_getter is not None)
                or (
                    path == "/recommendation_effectiveness"
                    and recommendation_effectiveness_html_getter is not None
                )
                or (path == "/activity_report" and activity_report_html_getter is not None)
                or (path == "/audience_display" and audience_display_html_getter is not None)
                or (path == "/behavior_rules" and behavior_rules_html_getter is not None)
                or (path == "/behavior_tree" and behavior_tree_html_getter is not None)
                or (path == "/schedule" and schedule_html_getter is not None)
                or (path == "/voice_settings" and voice_settings_html_getter is not None)
                or (path == "/announce_settings" and announce_settings_html_getter is not None)
            )
            account = self._authorize(require_write=False, redirect=is_page)
            if account is None:
                return
            if path in ("/", "/index.html"):
                self._send(HTTPStatus.OK, html_getter(), "text/html; charset=utf-8")
                return
            if path == "/api/whoami":
                self._send_json(HTTPStatus.OK, _whoami_payload(accounts_db, eph, account))
                return
            # 【2026-10-13追加】プログラム終了カードの状態(adminのみ。未保存の地図・走行中・二重実行の有無)。docs/system_shutdown.md
            if path == "/api/system/shutdown/state":
                _role = accounts_db.role_of(account)
                if system_shutdown is None or "system.shutdown" not in _roles.role_caps(_role, accounts_db.operator_permissions()):
                    self._send_json(HTTPStatus.FORBIDDEN, {"error": "forbidden_capability", "message": "この操作は許可されていません(管理者に権限の付与を依頼してください)。"})
                    return
                _st = dict(system_shutdown.state())
                _st["can_discard_map"] = _role == _roles.ROLE_ADMIN  # 未保存の地図を捨てて終了できるのは管理者だけ
                self._send_json(HTTPStatus.OK, _st)
                return
            # 【2026-10-13追加】アカウント・権限の管理画面用(adminのみ)。パスワード・ハッシュは返さない。
            if path == "/api/accounts":
                if accounts_db.role_of(account) != _roles.ROLE_ADMIN:
                    self._send_json(HTTPStatus.FORBIDDEN, {"error": "admin_only", "message": "この操作は管理者のみです。"})
                    return
                self._send_json(HTTPStatus.OK, {
                    "ok": True,
                    "accounts": accounts_db.listing(admin_names=[auth_username]),
                    "capabilities": _roles.describe_capabilities(),
                    "operator_permissions": sorted(accounts_db.operator_permissions()),
                    "problems": list(accounts_db.problems),
                    "ephemeral": {"count": eph.count(), "items": eph.items(),
                                  "pending_restore": eph.pending_restore_count()},
                    "recent_denied": list(denied_log.recent)[-20:],
                })
                return
            # 【2026-10-03追加】BGM状態(voice_bridgeの/bgmを中継)。guestを含む全ログイン
            # アカウントが閲覧可。bgmがNoneまたはbgm_url空ならconfigured:falseを返し、
            # UI側は項目ごと隠す。
            # 【2026-10-09追加】腹部ディスプレイ(アンケート)の状態・回答CSV(ラズパイの管理APIを中継)。
            # 全ログインアカウントが閲覧可(guest含む)。survey_admin_url空ならconfigured:false。
            if path == "/api/survey/state":
                self._send_json(
                    HTTPStatus.OK,
                    survey_admin.get_state() if survey_admin is not None else {"ok": False, "configured": False, "error": "アンケート管理未設定"},
                )
                return
            # 【2026-10-27追加】メニュー編集カード用に、いまのメニュー一覧(残数管理の /nokosahen/menu_state)を返す。全ログインアカウントが閲覧可(guest含む)。
            # iPad表示(DisplayHub)が無い構成では configured:false(カードごと隠す)。
            if path == "/api/menu/state":
                if display_hub is None:
                    self._send_json(HTTPStatus.OK, {"ok": False, "configured": False})
                else:
                    self._send_json(HTTPStatus.OK, dict(display_hub.menu_snapshot(), ok=True, configured=True))
                return
            # 【2026-10-09追加】タブレットOCRの状態(画質プロファイル・実効解像度・読み取り値。OCRの8091 APIを中継)。
            # 全ログインアカウントが閲覧可(guest含む)。ocr_admin_url空ならconfigured:false。
            if path == "/api/ocr/state":
                self._send_json(
                    HTTPStatus.OK,
                    ocr_admin.get_state() if ocr_admin is not None else {"ok": False, "configured": False, "error": "タブレットOCR管理未設定"},
                )
                return
            # 【2026-10-21】OCRのカメラ映像の中継(OCRの確認画面8091は、1台目の内側だけで待ち受ける)。ログインした全アカウントが見られる。
            # ?t=... はブラウザのキャッシュ回避(pathにはクエリが含まれない)。取れないときは、画像の代わりにJSONを返す。
            if path in ("/api/ocr/preview.jpg", "/api/ocr/processed.jpg"):
                kind = "preview" if path.endswith("preview.jpg") else "processed"
                code, body = ocr_admin.get_image(kind) if ocr_admin is not None else (404, b"")
                if code != 200:
                    self._send_json(HTTPStatus.NOT_FOUND if code == 404 else HTTPStatus.BAD_GATEWAY,
                                    {"ok": False, "error": "映像を取得できません(OCRが停止中か、まだ映像がありません)"})
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/api/survey/responses.csv":
                # 【2026-10-21】回答CSV(来場者の回答)は、閲覧専用のguestには出さない(管理者とoperatorだけ)。
                if accounts_db.role_of(account) == _roles.ROLE_VIEWER:
                    self._send_json(HTTPStatus.FORBIDDEN, {"error": "forbidden_capability",
                                                           "message": "アンケートの回答CSVは、管理者とoperatorだけがダウンロードできます。"})
                    return
                if survey_admin is None or not survey_admin.configured:
                    self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "アンケート管理未設定"})
                    return
                code, body, err = survey_admin.get_csv()
                if code != 200:
                    self._send_json(code, {"ok": False, "error": err, "message": err})
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/csv; charset=utf-8")
                self.send_header("Content-Disposition", 'attachment; filename="robot_survey.csv"')
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/api/bgm":
                self._send_json(
                    HTTPStatus.OK,
                    bgm.get_status() if bgm is not None else {"ok": False, "configured": False, "error": "BGM未設定"},
                )
                return
            # 【2026-10-09追加】音声の全体ON/OFF状態(voice_bridgeの/voice_masterを中継)。guestを含む
            # 全ログインアカウントが閲覧可。voice_masterがNoneまたはvoice_master_url空ならconfigured:false。
            if path == "/api/voice_master":
                self._send_json(
                    HTTPStatus.OK,
                    voice_master.get_status() if voice_master is not None else {"ok": False, "configured": False, "error": "音声の全体ON/OFF未設定"},
                )
                return
            # 【2026-10-09追加】音量(発話・BGM通常・BGM発話中)の現在値(voice_bridgeの/voice_levelsを中継)。
            # guestを含む全ログインアカウントが閲覧可。voice_levelsがNoneまたはURL空ならconfigured:false。
            if path == "/api/voice_levels":
                self._send_json(
                    HTTPStatus.OK,
                    voice_levels.get_status() if voice_levels is not None else {"ok": False, "configured": False, "error": "音量設定未設定"},
                )
                return
            if path == "/api/state":
                self._send_json(HTTPStatus.OK, state_getter())
                return
            # 【2026-10-24追加】手動運転の人避け(既定OFF)の状態。guestを含む全ログインアカウントが閲覧可。docs/manual_person_guard.md
            if path == "/api/manual_person_guard":
                self._send_json(HTTPStatus.OK, _manual_person_guard_payload(teleop, accounts_db, account))
                return
            if path == "/api/speed_settings":
                self._send_json(HTTPStatus.OK, _speed_settings_payload(teleop, auto_speed, nav_speed))
                return
            if path == "/api/nav_speed" and nav_speed is not None:
                # 実行中のNav2から現在値を読み戻す(最大3秒/ノード)。表示専用で何も書かない。
                body = dict(nav_speed.status())
                body["read_back"] = nav_speed.read_back()
                self._send_json(HTTPStatus.OK, body)
                return
            if path == "/api/map" and map_goal is not None:
                data = map_goal.map_json()
                if data is None:
                    self._send_json(HTTPStatus.NOT_FOUND, {"error": "map not available yet"})
                else:
                    self._send_json(HTTPStatus.OK, data)
                return
            # 【2026-10-13追加】混雑判定の背景キャリブレーションの結果(crowd_monitorが返したもの)。
            # 画面はPOSTで受け取ったrequest_idと一致するdone/failedを待つ。全ログインアカウントが閲覧可。
            if path == "/api/crowd_background/state" and crowd_background is not None:
                self._send_json(HTTPStatus.OK, crowd_background.status())
                return
            if path == "/api/scan" and scan is not None:
                data = scan.scan_json()
                if data is None:
                    self._send_json(HTTPStatus.NOT_FOUND, {"error": "scan not available yet"})
                else:
                    self._send_json(HTTPStatus.OK, data)
                return
            if path == "/api/mapping/status" and mapping is not None:
                self._send_json(HTTPStatus.OK, mapping.status())
                return
            if path == "/api/mapping/lifecycle_status" and map_lifecycle is not None:
                self._send_json(HTTPStatus.OK, map_lifecycle.status())
                return
            if path == "/api/mapping/list" and map_lifecycle is not None:
                self._send_json(HTTPStatus.OK, {"maps": map_lifecycle.list_maps()})
                return
            if path == "/api/pins" and pins is not None:
                # 【2026-10-07】地図ピン一覧(guestも閲覧可)。距離はロボットの現在位置から。
                self._send_json(HTTPStatus.OK, pins.status())
                return
            if path == "/api/wall_escape/status" and wall_escape is not None:
                # 【2026-10-22】壁際で動けない状態(guestも閲覧可)。
                self._send_json(HTTPStatus.OK, wall_escape.status())
                return
            if path == "/api/goal/status" and map_goal is not None:
                # 【2026-10-07】直近の目的地の進捗(残り距離・到着誤差)。guestも閲覧可。
                self._send_json(HTTPStatus.OK, map_goal.goal_status())
                return
            if path == "/api/localization" and localization is not None:
                # 【2026-10-07】位置の確かさ(AMCLの共分散)・localization稼働状態・tfの有無。
                self._send_json(HTTPStatus.OK, localization.status())
                return
            if path == "/api/boundary" and boundary is not None:
                self._send_json(HTTPStatus.OK, boundary.status())
                return
            if path == "/api/map_view" and _view_store() is not None:
                self._send_json(HTTPStatus.OK, _view_store().status())
                return
            if path == "/api/keepout_zones" and keepout_zones is not None:
                self._send_json(HTTPStatus.OK, keepout_zones.status())
                return
            if path == "/map_editor" and map_editor_html_getter is not None:
                self._send(HTTPStatus.OK, map_editor_html_getter(), "text/html; charset=utf-8")
                return
            if path == "/consumption_graph" and consumption_graph_html_getter is not None:
                self._send(
                    HTTPStatus.OK, consumption_graph_html_getter(), "text/html; charset=utf-8"
                )
                return
            if path == "/api/consumption_log" and consumption_log is not None:
                query = parse_qs(urlparse(self.path).query)
                since = (query.get("since") or [None])[0]
                self._send_json(
                    HTTPStatus.OK, {"events": consumption_log.fetch_events(since=since)}
                )
                return
            if path == "/utterance_log" and utterance_log_html_getter is not None:
                self._send(
                    HTTPStatus.OK, utterance_log_html_getter(), "text/html; charset=utf-8"
                )
                return
            if path == "/api/utterance_log" and utterance_log is not None:
                query = parse_qs(urlparse(self.path).query)
                since = (query.get("since") or [None])[0]
                # 【2026-10-02追加】取得元(source)とフォールバック時のerrorも返す
                # (リモート失敗時に古いローカルDBを黙って見せないため)。
                self._send_json(HTTPStatus.OK, utterance_log.fetch_utterances_ex(since=since))
                return
            if path == "/detection_log" and detection_log_html_getter is not None:
                self._send(
                    HTTPStatus.OK, detection_log_html_getter(), "text/html; charset=utf-8"
                )
                return
            if path == "/api/detection_log" and detection_log is not None:
                query = parse_qs(urlparse(self.path).query)
                since = (query.get("since") or [None])[0]
                self._send_json(
                    HTTPStatus.OK,
                    {"detections": detection_log.fetch_detections(since=since)},
                )
                return
            if path == "/step_response" and step_response_html_getter is not None:
                self._send(
                    HTTPStatus.OK, step_response_html_getter(), "text/html; charset=utf-8"
                )
                return
            if path == "/api/step_response/list" and step_response_results is not None:
                self._send_json(HTTPStatus.OK, {"results": step_response_results.list_results()})
                return
            if path == "/api/step_response/result" and step_response_results is not None:
                query = parse_qs(urlparse(self.path).query)
                result_id = (query.get("id") or [None])[0]
                result = step_response_results.read_result(result_id) if result_id else None
                if result is None:
                    self._send_json(HTTPStatus.NOT_FOUND, {"error": "result not found"})
                else:
                    self._send_json(HTTPStatus.OK, result)
                return
            if path == "/api/step_response/run_status" and pid_gains is not None:
                self._send_json(HTTPStatus.OK, pid_gains.step_response_status())
                return
            if (
                path == "/recommendation_effectiveness"
                and recommendation_effectiveness_html_getter is not None
            ):
                self._send(
                    HTTPStatus.OK,
                    recommendation_effectiveness_html_getter(),
                    "text/html; charset=utf-8",
                )
                return
            if path == "/activity_report" and activity_report_html_getter is not None:
                self._send(
                    HTTPStatus.OK, activity_report_html_getter(), "text/html; charset=utf-8"
                )
                return
            if path == "/audience_display" and audience_display_html_getter is not None:
                self._send(
                    HTTPStatus.OK, audience_display_html_getter(), "text/html; charset=utf-8"
                )
                return
            # 【2026-10-06追加】時間帯・状況ルールの編集ページ(閲覧は全アカウント可、変更はPOSTでguest403)。
            if path == "/behavior_rules" and behavior_rules_html_getter is not None:
                self._send(
                    HTTPStatus.OK, behavior_rules_html_getter(), "text/html; charset=utf-8"
                )
                return
            if path == "/api/behavior_rules/state" and behavior_rules is not None:
                self._send_json(HTTPStatus.OK, behavior_rules.live_state())
                return
            if path == "/api/behavior_rules/example" and behavior_rules is not None:
                status, payload = behavior_rules.example_payload()
                self._send_json(status, payload)
                return
            if path == "/api/behavior_rules/preview" and behavior_rules is not None:
                status, payload = behavior_rules.preview_payload()
                self._send_json(status, payload)
                return
            if path == "/api/behavior_rules" and behavior_rules is not None:
                self._send_json(HTTPStatus.OK, behavior_rules.rules_payload())
                return
            # 【2026-10-09追加】ビヘイビアツリーの編集ページ(閲覧は全アカウント可、保存/既定に戻すはPOSTでguest403)。
            if path == "/behavior_tree" and behavior_tree_html_getter is not None:
                self._send(
                    HTTPStatus.OK, behavior_tree_html_getter(), "text/html; charset=utf-8"
                )
                return
            # 【2026-10-23追加】かんたん時間割ページ(閲覧は全アカウント可。保存・実行のAPIは別途、書き込み権限が必要)。
            if path == "/schedule" and schedule_html_getter is not None:
                self._send(
                    HTTPStatus.OK, schedule_html_getter(), "text/html; charset=utf-8"
                )
                return
            if path == "/api/behavior_tree/state" and behavior_tree is not None:
                self._send_json(HTTPStatus.OK, behavior_tree.live_state())
                return
            if path == "/api/behavior_tree" and behavior_tree is not None:
                self._send_json(HTTPStatus.OK, behavior_tree.tree_payload())
                return
            # 【2026-10-23追加】スケジュール(時間割)と機体の時計。閲覧は全アカウント(guest含む)。docs/schedule.md
            if path == "/api/schedule" and schedule is not None:
                self._send_json(HTTPStatus.OK, schedule.payload())
                return
            if path == "/api/robot_clock" and schedule is not None:
                self._send_json(HTTPStatus.OK, schedule.robot_clock())
                return
            # 【2026-10-06追加】呼びかけ音声設定ページ(閲覧は全アカウント可、変更・お試し・試し発話はPOSTでguest403)。
            # 2台目(voice_bridge)へはこのサーバーが中継する(ブラウザは直接呼ばない)。
            # 【2026-10-13追加】状況アナウンス設定ページ(docs/announcements.md)。閲覧は全ログインアカウント(guest含む)。
            if path == "/announce_settings" and announce_settings_html_getter is not None:
                self._send(HTTPStatus.OK, announce_settings_html_getter(), "text/html; charset=utf-8")
                return
            if path == "/api/announce_events" and announce_events is not None:
                self._send_json(HTTPStatus.OK, announce_events.payload())
                return
            # 【2026-10-30追加】文化祭の出店紹介の設定・音声の作成状況(docs/announcements.md)。閲覧は全ログインアカウント(guest含む)。
            if path == "/api/pr_intro" and pr_intro is not None:
                self._send_json(HTTPStatus.OK, pr_intro.payload())
                return
            if path == "/voice_settings" and voice_settings_html_getter is not None:
                self._send(
                    HTTPStatus.OK, voice_settings_html_getter(), "text/html; charset=utf-8"
                )
                return
            if path == "/api/voice_settings" and voice_settings is not None:
                self._send_json(HTTPStatus.OK, voice_settings.payload())
                return
            if path == "/api/voice_settings/metrics" and voice_settings is not None:
                query = parse_qs(urlparse(self.path).query)
                try:
                    limit = int((query.get("limit") or ["50"])[0])
                    row_id = int(query["id"][0]) if query.get("id") else None
                except ValueError:
                    self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "limit/idは整数で指定してください"})
                    return
                if limit < 1 or limit > 500:
                    self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "limitは1〜500で指定してください"})
                    return
                status, payload = voice_settings.metrics(limit, row_id)
                self._send_json(status, payload)
                return
            if path == "/api/engagement_log" and engagement_log is not None:
                query = parse_qs(urlparse(self.path).query)
                since = (query.get("since") or [None])[0]
                self._send_json(
                    HTTPStatus.OK,
                    {
                        "engagements": engagement_log.fetch_engagements(since=since),
                        "mode_changes": engagement_log.fetch_mode_changes(since=since),
                    },
                )
                return
            if path == "/api/exploration/status" and frontier_explorer is not None:
                self._send_json(HTTPStatus.OK, frontier_explorer.status())
                return
            if path == "/api/camera" and camera is not None:
                # 【2026-10-22】?which=left|right で、後ろ向きカメラ(左後ろ・右後ろ)。省略は正面
                which = (parse_qs(urlparse(self.path).query).get("which") or [""])[0]
                ctrl = camera_rear.get(which) if (camera_rear and which) else camera
                if ctrl is None:
                    self._send_json(HTTPStatus.NOT_FOUND, {"error": "unknown camera"})
                    return
                jpeg = ctrl.jpeg_bytes()
                if jpeg is None:
                    self._send_json(HTTPStatus.NOT_FOUND, {"error": "camera preview not available yet"})
                else:
                    self._send(HTTPStatus.OK, jpeg, "image/jpeg")
                return
            if path == "/api/checkout_cams" and checkout_cams is not None:
                self._send_json(HTTPStatus.OK, {"cameras": checkout_cams.list_cameras()})
                return
            if path == "/api/checkout_cam_image" and checkout_cams is not None:
                query = parse_qs(urlparse(self.path).query)
                camera_id = (query.get("id") or [""])[0]
                jpeg = checkout_cams.jpeg_bytes(camera_id) if camera_id else None
                if jpeg is None:
                    self._send_json(HTTPStatus.NOT_FOUND, {"error": "camera image not available yet"})
                else:
                    self._send(HTTPStatus.OK, jpeg, "image/jpeg")
                return
            if path == "/api/pid_gains" and pid_gains is not None:
                gains = pid_gains.current_gains()
                if gains is None:
                    self._send_json(HTTPStatus.NOT_FOUND, {"error": "gains not reported yet"})
                else:
                    self._send_json(HTTPStatus.OK, gains)
                return
            self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

        def do_POST(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            # 【2026-10-30追加】書き込みAPI(ログインを含む全POST)は同じオリジンからだけ(CSRFの多層防御)。docs/ipad_display.md・docs/accounts.md
            if not _post_origin_guard(self):
                return
            if path == "/api/login":
                _handle_login_post(self, self._read_json(), login_accounts, sessions, accounts_db, login_limiter)
                return
            # 【2026-10-26追加】iPad表示 /display/api/... のPOST(ログイン以外は、iPad専用のセッションが要る。管理画面のログインとは別)。
            if display_ctx is not None and _display_routes.is_display_path(path):
                _display_routes.handle_post(self, path, urlparse(self.path).query, display_ctx)
                return

            # 【2026-10-21】ログアウトは、閲覧専用のguestにも許す(以前は書込み扱いで403になり、guestのセッションが24時間残った)
            account = self._authorize(require_write=(path != "/api/logout"))
            if account is None:
                return
            data = self._read_json()

            if path == "/api/logout":
                _handle_logout_post(self, account, gate, sessions)
                return

            # 【2026-10-13追加】権限チェック(operatorは許可された操作だけ)。persist=False=operator=変更は一時的。
            _role = accounts_db.role_of(account)  # 【2026-11-06】1回だけ(以前は _capability_gate と persist で2回呼び、毎回 stat が2×2回走っていた)
            if not _capability_gate(self, account, path, data, accounts_db, denied_log, role=_role):
                return
            persist = _role == _roles.ROLE_ADMIN

            # 【2026-10-13追加】アカウント・権限の管理(adminのみ。operatorは roles.API_RULES で403)。docs/accounts.md
            if path == "/api/accounts/permissions":
                names = data.get("operator_permissions")
                if not isinstance(names, list):
                    self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "bad_request",
                                                             "message": "operator_permissions をリストで送ってください"})
                    return
                try:
                    granted, problems = accounts_db.save_permissions(names, updated_by=account)
                except _roles.PermissionConfigError as exc:
                    _AUDIT.warning("PERMISSIONS_REJECTED account=%s detail=%s", account, exc)
                    self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "invalid_permissions",
                                                             "message": str(exc)})
                    return
                except OSError as exc:
                    self._send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"ok": False, "error": "io",
                                                                      "message": f"保存できませんでした: {exc}"})
                    return
                _AUDIT.info("PERMISSIONS_SAVED account=%s granted=%s", account, ",".join(sorted(granted)))
                self._send_json(HTTPStatus.OK, {"ok": True, "operator_permissions": sorted(granted),
                                                "problems": problems})
                return
            # 【2026-10-13追加】プログラム終了(全停止)。adminと、system.shutdownを許可されたoperator(roles.API_RULES)。OS停止(include_poweroff)を含む。docs/system_shutdown.md
            # 応答を返したあと、切り離した別プロセス(scripts/shutdown_all.sh)が管理画面自身も含めて止める。
            if path == "/api/system/shutdown":
                if system_shutdown is None:
                    self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not_available", "message": "この構成ではプログラム終了を使えません"})
                    return
                ip = self.client_address[0] if getattr(self, "client_address", None) else "?"
                code, body = system_shutdown.request(account, ip, data)
                # 【2026-11-05】監査ログは parse_request の結果をそのまま記録する(以前は既定値を include_poweroff=True と
                # 独自に仮定していて、APIの既定(False)と食い違った)。成功(200)のときだけ使うので、検証は必ず通る。
                try:
                    _o = system_shutdown_parse_request(data)
                except Exception:
                    _o = {}
                opts = ",".join(k for k, v in _o.items() if v is True)
                if code == 200:
                    _AUDIT.warning("SYSTEM_SHUTDOWN account=%s ip=%s options=%s", account, ip, opts or "-")
                else:
                    _AUDIT.warning("SYSTEM_SHUTDOWN_REJECTED account=%s ip=%s status=%s error=%s", account, ip, code, body.get("error"))
                self._send_json(code, body)
                return
            if path == "/api/ephemeral/clear":
                res = eph.clear_all(_eph_restorers())
                _ok_urls, ng_urls = reset_remote(_eph_remote_urls())
                res["remote_reset_failed"] = ng_urls
                res["ok"] = not res["restore_failed"] and not ng_urls
                _AUDIT.info("EPHEMERAL_CLEARED account=%s cleared=%s", account, res["cleared"])
                self._send_json(HTTPStatus.OK, res)
                return

            if path == "/api/pid_gains" and pid_gains is not None:
                fields = [
                    "rkp_fwd", "rki_fwd", "rkd_fwd", "rkp_bwd", "rki_bwd", "rkd_bwd",
                    "lkp_fwd", "lki_fwd", "lkd_fwd", "lkp_bwd", "lki_bwd", "lkd_bwd",
                ]
                try:
                    values = [float(data[f]) for f in fields]
                except (KeyError, TypeError, ValueError):
                    self._send_json(
                        HTTPStatus.BAD_REQUEST,
                        {"error": f"{', '.join(fields)} are required numbers"},
                    )
                    return
                # 【2026-10-21】NaN・無限大・負・巨大な値は、ファームが検査せずに採用する(負のゲインは正帰還で暴走する)。範囲を確かめてから送る。
                if not all(math.isfinite(v) and 0.0 <= v <= PID_GAIN_MAX for v in values):
                    self._send_json(
                        HTTPStatus.BAD_REQUEST,
                        {"error": "bad_gain", "message": f"ゲインは 0 以上 {PID_GAIN_MAX:g} 以下の有限の数にしてください"},
                    )
                    return
                pid_gains.set_gains(*values)
                self._send_json(HTTPStatus.OK, {"ok": True})
                return

            if path == "/api/drive_test" and pid_gains is not None:
                if pid_gains.test_running():
                    self._send_json(
                        HTTPStatus.CONFLICT,
                        {"error": "test_running", "message": "既にテスト走行中です。終了までお待ちください。"},
                    )
                    return
                mode = str(data.get("mode", ""))
                try:
                    duration_s = float(data["duration_s"])
                except (KeyError, TypeError, ValueError):
                    self._send_json(HTTPStatus.BAD_REQUEST, {"error": "duration_s is required"})
                    return
                if mode in ("forward", "backward"):
                    try:
                        distance_m = float(data["distance_m"])
                    except (KeyError, TypeError, ValueError):
                        self._send_json(HTTPStatus.BAD_REQUEST, {"error": "distance_m is required"})
                        return
                    if duration_s <= 0:
                        self._send_json(HTTPStatus.BAD_REQUEST, {"error": "duration_s must be positive"})
                        return
                    # 【2026-09-30追加】方向はmode(forward/backward)で決まるため、
                    # distance_mは正の値のみ受け付ける(nan/infや負値/0は拒否)。
                    if not math.isfinite(distance_m) or not math.isfinite(duration_s):
                        self._send_json(HTTPStatus.BAD_REQUEST, {"error": "distance_m/duration_s must be finite"})
                        return
                    if distance_m <= 0:
                        self._send_json(
                            HTTPStatus.BAD_REQUEST,
                            {"error": "distance_m must be positive (direction is set by mode)"},
                        )
                        return
                    if abs(distance_m) > pid_gains.MAX_TEST_DISTANCE_M:
                        self._send_json(
                            HTTPStatus.BAD_REQUEST,
                            {
                                "error": "distance_too_large",
                                "message": f"distance_mは±{pid_gains.MAX_TEST_DISTANCE_M}m以内にしてください。",
                            },
                        )
                        return
                    linear = distance_m / duration_s
                    if mode == "backward":
                        linear = -linear
                    started = pid_gains.run_drive_test(
                        linear, 0.0, duration_s, target_distance_m=distance_m
                    )
                elif mode in ("rotate_left", "rotate_right"):
                    try:
                        rotate_deg = float(data["rotate_deg"])
                    except (KeyError, TypeError, ValueError):
                        self._send_json(HTTPStatus.BAD_REQUEST, {"error": "rotate_deg is required"})
                        return
                    if duration_s <= 0:
                        self._send_json(HTTPStatus.BAD_REQUEST, {"error": "duration_s must be positive"})
                        return
                    # 【2026-09-30追加】方向はmode(rotate_left/right)で決まるため、
                    # rotate_degは0超〜180度以内の有限値のみ受け付ける。
                    if (
                        not math.isfinite(rotate_deg)
                        or not math.isfinite(duration_s)
                        or rotate_deg <= 0
                        or rotate_deg > 180.0
                    ):
                        self._send_json(
                            HTTPStatus.BAD_REQUEST,
                            {
                                "error": "rotate_deg_out_of_range",
                                "message": "rotate_degは0より大きく180以下(度)にしてください(方向はmodeで指定)。",
                            },
                        )
                        return
                    angular = math.radians(rotate_deg) / duration_s
                    if mode == "rotate_right":
                        angular = -angular
                    started = pid_gains.run_drive_test(
                        0.0, angular, duration_s,
                        target_rotate_rad=math.radians(rotate_deg),
                    )
                else:
                    self._send_json(HTTPStatus.BAD_REQUEST, {"error": "mode must be forward/backward/rotate_left/rotate_right"})
                    return
                if not started:
                    self._send_json(
                        HTTPStatus.CONFLICT,
                        {"error": "estop_engaged", "message": "緊急停止中は走行テストを開始できません。"},
                    )
                    return
                self._send_json(HTTPStatus.OK, {"ok": True})
                return

            if path == "/api/step_response/run" and pid_gains is not None and step_response_results is not None:
                if pid_gains.test_running():
                    self._send_json(
                        HTTPStatus.CONFLICT,
                        {"error": "test_running", "message": "既にテスト走行中です。終了までお待ちください。"},
                    )
                    return
                direction = str(data.get("direction", "forward"))
                if direction not in ("forward", "backward", "rotate_left", "rotate_right"):
                    self._send_json(
                        HTTPStatus.BAD_REQUEST,
                        {"error": "direction must be forward/backward/rotate_left/rotate_right"},
                    )
                    return
                is_rotate = direction in ("rotate_left", "rotate_right")
                try:
                    duration_s = float(data.get("duration_s", 8.0))
                    rate_hz = float(data.get("rate_hz", 10.0))
                    os_threshold_pct = float(data.get("os_threshold_pct", 5.0))
                    tol_pct = float(data.get("tol_pct", 2.0))
                    if is_rotate:
                        target_deg = float(data["target_deg"])
                        angular_rps = float(data.get("angular_rps", 1.0))
                        target_m = 0.0
                        speed_mps = 0.0
                    else:
                        target_m = float(data["target_m"])
                        speed_mps = float(data.get("speed_mps", 0.4))
                        target_deg = 0.0
                        angular_rps = 0.0
                except (KeyError, TypeError, ValueError):
                    self._send_json(
                        HTTPStatus.BAD_REQUEST,
                        {
                            "error": "target_m (forward/backward) or target_deg (rotate_left/rotate_right) is required (number)"
                        },
                    )
                    return
                # 【2026-10-02追加】json.loadsはNaN/Infinityを受け付ける。NaNは
                # 「<=0」「>上限」の比較をどちらもすり抜け、下流のmin/maxで上限値に
                # 張り付いて最大距離/角度・最高速度で走ってしまうため、全て有限値のみ受け付ける。
                if not all(
                    math.isfinite(v)
                    for v in (
                        duration_s, rate_hz, os_threshold_pct, tol_pct,
                        target_deg, target_m, speed_mps, angular_rps,
                    )
                ):
                    self._send_json(
                        HTTPStatus.BAD_REQUEST,
                        {
                            "error": "non_finite_value",
                            "message": "数値にNaN/Infinityは指定できません。",
                        },
                    )
                    return
                if is_rotate:
                    if target_deg <= 0:
                        self._send_json(HTTPStatus.BAD_REQUEST, {"error": "target_deg must be positive"})
                        return
                    if target_deg > pid_gains.MAX_TEST_ROTATE_DEG:
                        self._send_json(
                            HTTPStatus.BAD_REQUEST,
                            {
                                "error": "rotate_too_large",
                                "message": f"target_degは{pid_gains.MAX_TEST_ROTATE_DEG}度以内にしてください。",
                            },
                        )
                        return
                else:
                    if target_m <= 0:
                        self._send_json(HTTPStatus.BAD_REQUEST, {"error": "target_m must be positive"})
                        return
                    if target_m > pid_gains.MAX_TEST_DISTANCE_M:
                        self._send_json(
                            HTTPStatus.BAD_REQUEST,
                            {
                                "error": "distance_too_large",
                                "message": f"target_mは{pid_gains.MAX_TEST_DISTANCE_M}m以内にしてください。",
                            },
                        )
                        return
                started = pid_gains.run_step_response_test(
                    direction, target_m, duration_s, rate_hz, speed_mps,
                    os_threshold_pct, tol_pct, str(step_response_results.results_dir),
                    target_deg=target_deg, angular_rps=angular_rps,
                )
                if not started:
                    self._send_json(
                        HTTPStatus.CONFLICT,
                        {"error": "estop_engaged", "message": "緊急停止中は走行テストを開始できません。"},
                    )
                    return
                self._send_json(HTTPStatus.OK, {"ok": True})
                return

            if path == "/api/teleop":
                try:
                    forward = float(data.get("forward", 0.0))
                    turn = float(data.get("turn", 0.0))
                except (TypeError, ValueError):
                    self._send_json(HTTPStatus.BAD_REQUEST, {"error": "forward/turn must be numbers"})
                    return
                # 【2026-10-02追加】NaN/Infinityは拒否(NaNは全速になってしまう)。
                if not (math.isfinite(forward) and math.isfinite(turn)):
                    teleop.cmd_vel_pub.publish(Twist())
                    self._send_json(HTTPStatus.BAD_REQUEST, {"error": "forward/turn must be finite numbers"})
                    return
                # 【2026-11-06】operator(弱いアカウント)も運転できるので、優先度はロールで決める(adminだけFULL。operatorはTLの運転中は破棄される)。
                accepted = teleop.handle_drive(
                    forward, turn, priority=teleop_priority_for_role(_role),
                    source=build_teleop_source(account, self.client_address[0], data.get("tab_id")),
                    seq=parse_teleop_seq(data.get("seq")),
                )
                if not accepted and teleop.soft_estop_engaged:
                    self._send_json(
                        HTTPStatus.CONFLICT,
                        {"error": "estop_engaged", "message": "緊急停止中です。解除してから操作してください。"},
                    )
                    return
                self._send_json(HTTPStatus.OK, {
                    "ok": accepted,
                    # 人避けで前進をゼロにしたとき true(画面に理由を出す)。アイドル(0,0)は常に false。
                    "manual_person_guard_blocked": bool(accepted and (forward != 0.0 or turn != 0.0) and getattr(teleop, "last_guard_blocked", False)),
                })
                return

            # 【2026-10-24追加】手動運転の人避けのON/OFF。guestは_authorize(require_write=True)で403、operatorは
            # roles.API_RULESの drive.person_guard を持つときだけ(POST全体が管理者専用 + 付与可能)。
            # 本文は {"enabled": true|false} の厳密なboolのみ(それ以外は400)。operatorの変更は一時的(保存しない)。
            if path == "/api/manual_person_guard":
                enabled = data.get("enabled") if isinstance(data, dict) else None
                if not isinstance(enabled, bool):
                    self._send_json(HTTPStatus.BAD_REQUEST, {"error": "bad_request", "message": "enabled は true か false で指定してください。"})
                    return
                before = bool(teleop.manual_person_guard)
                teleop.set_manual_person_guard(enabled, persist=persist)
                _AUDIT.warning("MANUAL_PERSON_GUARD account=%s ip=%s enabled=%s->%s persist=%s", account,
                               self.client_address[0] if getattr(self, "client_address", None) else "?", before, enabled, persist)
                self._send_json(HTTPStatus.OK, dict(_manual_person_guard_payload(teleop, accounts_db, account), ok=True))
                return

            if path == "/api/speed_settings":
                status, body = _speed_settings_post(data, teleop, auto_speed, nav_speed)
                self._send_json(status, body)
                return

            if path == "/api/soft_estop":
                # 【2026-11-04】解除は engaged が厳密に false のときだけ(null・0・""・[]・無しは「停止をかける」)。roles._estop_rule と同じ式
                engaged = data.get("engaged") is not False
                teleop.handle_soft_estop(engaged)
                self._send_json(HTTPStatus.OK, {"ok": True})
                return

            if path == "/api/resume_autonomy":
                teleop.deactivate_goal_mode()
                teleop.publish_manual_override(False)
                self._send_json(HTTPStatus.OK, {"ok": True})
                return

            if path == "/api/goal":
                if map_goal is None:
                    self._send_json(HTTPStatus.NOT_FOUND, {"error": "map/nav not available on this system"})
                    return
                try:
                    x = float(data["x"])
                    y = float(data["y"])
                except (KeyError, TypeError, ValueError):
                    self._send_json(HTTPStatus.BAD_REQUEST, {"error": "x/y are required numbers"})
                    return
                # 【2026-10-02追加】NaN/Infinityの目的地は拒否する。
                if not (math.isfinite(x) and math.isfinite(y)):
                    self._send_json(HTTPStatus.BAD_REQUEST, {"error": "x/y must be finite numbers"})
                    return
                # 【2026-10-07追加】到着の向き(任意)。無ければ従来どおり単位クォータニオン。
                yaw_deg = None
                if data.get("yaw_deg") is not None:
                    try:
                        if isinstance(data["yaw_deg"], bool):
                            raise ValueError
                        yaw_deg = float(data["yaw_deg"])
                    except (TypeError, ValueError):
                        yaw_deg = float("nan")
                    if not math.isfinite(yaw_deg):
                        self._send_json(HTTPStatus.BAD_REQUEST, {"error": "yaw_deg must be a finite number"})
                        return
                    yaw_deg = normalize_pin_deg(yaw_deg)
                if not self._goal_guard(x, y):
                    return
                # 【2026-10-07追加】占有セル・壁際(ロボット半径内)・未知セルの目的地は、Nav2のプランナが
                # 最も近い有効点へ勝手に寄せて意図とずれた所で止まる。先にサーバー側で最も近い空きセルへ寄せ、
                # ずらした距離を返す(UIのマーカーに反映)。snap=falseで無効にできる。
                req_x, req_y = x, y
                snap = {"status": "no_map", "x": x, "y": y, "shift_m": 0.0, "reason": ""}
                if data.get("snap", True) is not False:
                    _rect = self._boundary_rect_now()
                    snap = map_goal.check_goal(x, y, rect=_rect) if _rect is not None else map_goal.check_goal(x, y)
                if snap["status"] == "no_free":
                    self._send_json(
                        HTTPStatus.CONFLICT,
                        {
                            "ok": False,
                            "error": "no_free_cell",
                            "message": "目的地が壁や障害物の上で、近く(0.5 m以内)に走行できる場所がありません。"
                                       "通路の中央あたりをクリックしてください。",
                        },
                    )
                    return
                snapped = snap["status"] == "snapped"
                if snapped:
                    x, y = float(snap["x"]), float(snap["y"])
                    if not self._goal_guard(x, y):
                        return
                # 目的地送信中は自律ロジックのgoal送信と競合しないよう手動運転扱いにする。
                # (WASDのウォッチドッグと違い、ゴール到達で自動解除はしないため、
                #  ダッシュボードの「自律に戻す」ボタン(/api/resume_autonomy)で戻す)。
                # activate_goal_mode()がタイマーでmanual_overrideを送り続けるので、
                # behavior_director側のmanual_override_timeout_secで途中で
                # 自律ロジックへ戻ってしまうことはない。
                teleop.activate_goal_mode()
                map_goal.send_goal(
                    x, y, yaw_deg,
                    meta={"source": "click", "requested_x": req_x, "requested_y": req_y,
                          "snapped": snapped, "shift_m": snap["shift_m"] if snapped else 0.0},
                )
                message = "目的地を送りました。"
                if snapped:
                    message = (
                        f"目的地が{'壁・障害物の上' if snap['reason'] == 'occupied' else '壁際・未知の場所'}だったため、"
                        f"{snap['shift_m'] * 100:.0f} cm ずらした走行できる場所へ送りました。"
                    )
                self._send_json(
                    HTTPStatus.OK,
                    {
                        "ok": True,
                        "message": message,
                        "goal": {"x": x, "y": y, "yaw_deg": 0.0 if yaw_deg is None else yaw_deg,
                                 "yaw_specified": yaw_deg is not None},
                        "requested": {"x": req_x, "y": req_y},
                        "snapped": snapped,
                        "shift_m": snap["shift_m"] if snapped else 0.0,
                        "snap_status": snap["status"],
                    },
                )
                return

            if path == "/api/goal/cancel":
                if map_goal is None:
                    self._send_json(HTTPStatus.NOT_FOUND, {"error": "map/nav not available on this system"})
                    return
                # 【2026-10-07追加】目的地の中止(既存の/nokosahen/cancel_goalと同じ経路。ゼロ速度も出る)。
                was_active = map_goal.cancel()
                self._send_json(
                    HTTPStatus.OK,
                    {"ok": True, "was_active": was_active,
                     "message": "目的地を中止しました(ロボットを止めます)。自律運転に戻すには「自律運転に戻す」を押してください。"},
                )
                return

            if path == "/api/wall_escape/trigger":
                # 【2026-10-22】「壁から離れる」。書き込み権限(guest不可)・operatorの権限(nav.wall_escape)は上で確認済み。
                if wall_escape is None:
                    self._send_json(HTTPStatus.NOT_FOUND, {"error": "wall_escape not available on this system"})
                    return
                if teleop.soft_estop_engaged:
                    self._send_json(HTTPStatus.CONFLICT, {"ok": False, "error": "estop_engaged",
                                                          "message": "緊急停止中です。解除してから操作してください。"})
                    return
                wall_escape.trigger()
                _AUDIT.info("WALL_ESCAPE_TRIGGER account=%s", account)
                self._send_json(HTTPStatus.OK, {"ok": True, "status": wall_escape.status(),
                                                "message": "壁から離れる操作を依頼しました。動けない場合は理由が表示されます。"})
                return

            if path in ("/api/pins/add", "/api/pins/update", "/api/pins/delete", "/api/pins/goto"):
                # 【2026-10-07追加】地図ピン。書き込み権限(guest不可)は上で確認済み。
                if pins is None:
                    self._send_json(HTTPStatus.NOT_FOUND, {"error": "pins not available on this system"})
                    return
                try:
                    if path == "/api/pins/add":
                        result = pins.add(data)
                    elif path == "/api/pins/update":
                        result = pins.update(data)
                    elif path == "/api/pins/delete":
                        result = pins.delete(data)
                    else:
                        if map_goal is None:
                            self._send_json(HTTPStatus.NOT_FOUND, {"error": "map/nav not available on this system"})
                            return
                        # 保存した姿勢(x, y, yaw)をそのまま/goal_poseへ(再クリックの量子化もスナップもしない)。
                        # /api/goalと同じガード(走行範囲・ソフト緊急停止)を通す。
                        pin = pins.store.get(data.get("id"))
                        if pin is None:
                            raise PinError("ピンが見つかりません", "not_found")
                        if not self._goal_guard(pin["x"], pin["y"]):
                            return
                        pin, pose = pins.resolve_goto(data)
                        warning = ""
                        chk = map_goal.check_goal(pin["x"], pin["y"], wall_margin=False)
                        if chk["status"] in ("snapped", "no_free", "unchanged"):
                            warning = (
                                "ピンの場所が壁・障害物のそば(またはロボットが通れない場所)です。Nav2が最も近い"
                                "到達できる地点で止まるため、ピンを立て直すことをおすすめします。"
                            )
                        teleop.activate_goal_mode()
                        map_goal.send_goal(
                            pin["x"], pin["y"], pin["yaw_deg"],
                            meta={"source": "pin", "pin_id": pin["id"], "pin_name": pin["name"],
                                  "snapped": False, "shift_m": 0.0},
                        )
                        dist = math.hypot(pin["x"] - pose["x"], pin["y"] - pose["y"])
                        result = {
                            "ok": True,
                            "pin": pin,
                            "goal": {"x": pin["x"], "y": pin["y"], "yaw_deg": pin["yaw_deg"], "yaw_specified": True},
                            "distance_m": dist,
                            "message": f"ピン「{pin['name']}」へ向かいます(約{dist:.1f} m)。",
                        }
                        if warning:
                            result["warning"] = warning
                            result["message"] += " 注意: " + warning
                except PinError as exc:
                    self._send_pin_error(exc)
                    return
                self._send_json(HTTPStatus.OK, result)
                return

            if path in ("/api/initial_pose", "/api/global_localization"):
                # 【2026-10-07追加】ロボットに現在位置を教える(AMCLの初期位置 / 全体位置推定)。
                # ロボットは動かさない(cmd_vel・Nav2ゴールは出さない)。書き込み権限(guest不可)は上で確認済み。
                if localization is None:
                    self._send_json(
                        HTTPStatus.NOT_FOUND,
                        {"error": "localization not available on this system"},
                    )
                    return
                try:
                    if path == "/api/initial_pose":
                        result = localization.set_initial_pose(data)
                    else:
                        result = localization.global_localization()
                except PoseRequestError as exc:
                    self._send_json(
                        HTTPStatus.BAD_REQUEST,
                        {"ok": False, "error": "bad_request", "message": str(exc)},
                    )
                    return
                except LocalizationAssistError as exc:
                    status = (
                        HTTPStatus.SERVICE_UNAVAILABLE if exc.kind == "service_unavailable"
                        else HTTPStatus.CONFLICT
                    )
                    self._send_json(status, {"ok": False, "error": exc.kind, "message": exc.message})
                    return
                self._send_json(HTTPStatus.OK, result)
                return

            if path in ("/api/map_view/set", "/api/map_view/estimate", "/api/map_view/save"):
                # 【2026-10-06追加】地図の表示回転(世界座標系は回さない。表示と範囲指定だけ)。
                vs = _view_store()
                if vs is None:
                    self._send_json(
                        HTTPStatus.NOT_FOUND,
                        {"error": "map view not available on this system"},
                    )
                    return
                if path == "/api/map_view/set":
                    try:
                        if "wall_points" in data:
                            pts = data["wall_points"]
                            if not isinstance(pts, (list, tuple)) or len(pts) != 4:
                                raise ValueError("wall_points は [x1,y1,x2,y2] で指定してください")
                            angle = view_angle_from_two_points(*[float(v) for v in pts])
                        else:
                            angle = data["angle_deg"]
                        applied = vs.set_session_angle(angle)
                    except KeyError:
                        self._send_json(
                            HTTPStatus.BAD_REQUEST,
                            {"ok": False, "error": "angle_deg (or wall_points) is required"},
                        )
                        return
                    except (TypeError, ValueError) as exc:
                        self._send_json(
                            HTTPStatus.BAD_REQUEST,
                            {"ok": False, "error": str(exc), "message": str(exc)},
                        )
                        return
                    self._send_json(HTTPStatus.OK, {"ok": True, "angle_deg": applied})
                    return
                if path == "/api/map_view/estimate":
                    m = map_goal.latest_map if map_goal is not None else None
                    if m is None:
                        self._send_json(
                            HTTPStatus.CONFLICT,
                            {"ok": False, "message": "地図がまだありません。地図が表示されてから実行してください。"},
                        )
                        return
                    est = estimate_wall_angle_deg(m.info.width, m.info.height, list(m.data))
                    if est is None:
                        self._send_json(
                            HTTPStatus.OK,
                            {"ok": False, "message": "壁(占有セル)が足りず向きを推定できませんでした。もう少し地図を作ってから、または2点指定で合わせてください。"},
                        )
                        return
                    angle = view_angle_from_wall_deg(est["angle_deg"])
                    applied = False
                    if data.get("apply", True):
                        angle = vs.set_session_angle(angle)
                        applied = True
                    self._send_json(
                        HTTPStatus.OK,
                        {
                            "ok": True,
                            "wall_angle_deg": est["angle_deg"],
                            "angle_deg": angle,
                            "confidence": est["confidence"],
                            "applied": applied,
                        },
                    )
                    return
                # /api/map_view/save: この向きを地図の基準として<name>.view.jsonへ保存
                name = str(data.get("name", "") or vs.loaded_map() or "").strip()
                if not name:
                    self._send_json(
                        HTTPStatus.CONFLICT,
                        {"ok": False, "message": "地図名が必要です(地図を読み込み済みでなければ名前を入力してください)"},
                    )
                    return
                try:
                    saved = vs.save_for_map(name)
                except (OSError, ValueError) as exc:
                    self._send_json(
                        HTTPStatus.CONFLICT, {"ok": False, "message": f"保存できません: {exc}"}
                    )
                    return
                self._send_json(
                    HTTPStatus.OK,
                    {"ok": True, "name": name, "angle_deg": saved,
                     "message": f"表示の向き{saved:.1f}°を地図「{name}」の基準として保存しました"},
                )
                return

            if path == "/api/mapping/save":
                if mapping is None:
                    self._send_json(
                        HTTPStatus.NOT_FOUND,
                        {"error": "mapping not available on this system"},
                    )
                    return
                name = str(data.get("name", ""))
                ok, message = mapping.save_map(name)
                self._send_json(
                    HTTPStatus.OK if ok else HTTPStatus.CONFLICT,
                    {"ok": ok, "message": message},
                )
                return

            if path in (
                "/api/mapping/start",
                "/api/mapping/stop",
                "/api/mapping/reset",
                "/api/mapping/load",
            ):
                if map_lifecycle is None:
                    self._send_json(
                        HTTPStatus.NOT_FOUND,
                        {"error": "map lifecycle not available on this system"},
                    )
                    return
                if path == "/api/mapping/start":
                    ok, message = map_lifecycle.start_mapping()
                elif path == "/api/mapping/stop":
                    ok, message = map_lifecycle.stop()
                elif path == "/api/mapping/reset":
                    ok, message = map_lifecycle.reset()
                else:
                    ok, message = map_lifecycle.load_map(str(data.get("name", "")))
                self._send_json(
                    HTTPStatus.OK if ok else HTTPStatus.CONFLICT,
                    {"ok": ok, "message": message},
                )
                return

            if path in ("/api/boundary/set", "/api/boundary/clear"):
                if boundary is None:
                    self._send_json(
                        HTTPStatus.NOT_FOUND,
                        {"error": "boundary not available on this system"},
                    )
                    return
                if path == "/api/boundary/clear":
                    ok, message = boundary.clear_boundary()
                else:
                    # 新形式{center_x,center_y,width,height,yaw|yaw_deg}と旧形式
                    # {min_x,max_x,min_y,max_y}の両方を受け付ける。NaN/Infinity・欠落・
                    # 非数は400、幅高さが不正等は409(旧APIと同じ)。
                    try:
                        ok, message = boundary.set_from_request(data)
                    except BoundaryError as exc:
                        self._send_json(
                            HTTPStatus.BAD_REQUEST if exc.kind == "malformed" else HTTPStatus.CONFLICT,
                            {"ok": False, "error": str(exc), "message": str(exc)},
                        )
                        return
                self._send_json(
                    HTTPStatus.OK if ok else HTTPStatus.CONFLICT,
                    {"ok": ok, "message": message},
                )
                return

            if path in ("/api/keepout_zones/add", "/api/keepout_zones/remove", "/api/keepout_zones/clear"):
                if keepout_zones is None:
                    self._send_json(
                        HTTPStatus.NOT_FOUND,
                        {"error": "keepout_zones not available on this system"},
                    )
                    return
                if path == "/api/keepout_zones/clear":
                    ok, message = keepout_zones.clear()
                elif path == "/api/keepout_zones/remove":
                    try:
                        zone_id = int(data["id"])
                    except (KeyError, TypeError, ValueError):
                        self._send_json(HTTPStatus.BAD_REQUEST, {"error": "id is required"})
                        return
                    ok, message = keepout_zones.remove_zone(zone_id)
                else:
                    try:
                        min_x = float(data["min_x"])
                        max_x = float(data["max_x"])
                        min_y = float(data["min_y"])
                        max_y = float(data["max_y"])
                    except (KeyError, TypeError, ValueError):
                        self._send_json(
                            HTTPStatus.BAD_REQUEST,
                            {"error": "min_x/max_x/min_y/max_y are required numbers"},
                        )
                        return
                    ok, message = keepout_zones.add_zone(min_x, max_x, min_y, max_y)
                self._send_json(
                    HTTPStatus.OK if ok else HTTPStatus.CONFLICT,
                    {"ok": ok, "message": message},
                )
                return

            if path in ("/api/exploration/start", "/api/exploration/stop"):
                if frontier_explorer is None:
                    self._send_json(
                        HTTPStatus.NOT_FOUND,
                        {"error": "frontier explorer not available on this system"},
                    )
                    return
                if path == "/api/exploration/start" and teleop.soft_estop_engaged:
                    # 【2026-09-30追加】ソフト緊急停止中は探索を開始させない。
                    self._send_json(
                        HTTPStatus.CONFLICT,
                        {"error": "estop_engaged", "message": "緊急停止中です。解除してから操作してください。"},
                    )
                    return
                if path == "/api/exploration/start":
                    # 【2026-10-22】地図クリックの目的地を送ったあとは「自律に戻す」を押すまで手動扱い(manual_override)が続き、
                    # 探索が「manual_overrideで一時停止」のまま動かなかった。探索の開始は自律に戻す意思表示なので、ここで解除する。
                    teleop.deactivate_goal_mode()
                    teleop.publish_manual_override(False)
                ok, message = frontier_explorer.set_enabled(path == "/api/exploration/start")
                self._send_json(HTTPStatus.OK, {"ok": ok, "message": message})
                return

            # 【2026-10-03追加】BGM ON/OFF・音量(voice_bridgeの/bgmへ中継)。書込み扱いなので
            # guestは上の_authorize(require_write=True)で403。入力検証はBgmProxy.set()。
            if path == "/api/bgm" and bgm is not None:
                status, payload = bgm.set(data, persist=persist)
                if status == 200 and payload.get("ok"):
                    _eph_commit(path, data, account, persist, {})
                self._send_json(status, payload)
                return

            # 【2026-10-09追加】音声の全体ON/OFF(voice_bridgeの/voice_masterへ中継)。書込み扱いなので
            # guestは上の_authorize(require_write=True)で403。入力検証はVoiceMasterProxy.set()
            # ({"enabled": true/false}のみ。文字列や他の項目は400)。
            if path == "/api/voice_master" and voice_master is not None:
                status, payload = voice_master.set(data, persist=persist)
                if status == 200 and payload.get("ok"):
                    _eph_commit(path, data, account, persist, {})
                self._send_json(status, payload)
                return

            # 【2026-10-09追加】音量の変更(voice_bridgeの/voice_levelsへ中継、部分更新・null=既定に戻す)。
            # 書込み扱いなのでguestは上の_authorize(require_write=True)で403。入力検証はVoiceLevelsProxy.set()
            # (範囲外・型違い・未知の項目・ダッキング>通常は400で、2台目へは送らない)。
            if path == "/api/voice_levels" and voice_levels is not None:
                status, payload = voice_levels.set(data, persist=persist)
                if status == 200 and payload.get("ok"):
                    _eph_commit(path, data, account, persist, {})
                self._send_json(status, payload)
                return

            # 【2026-10-13追加】状況アナウンスの設定保存・試し再生(docs/announcements.md)。書込み扱いなのでguestは上の
            # _authorize(require_write=True)で403、operatorは roles.API_RULES に載るまでadmin専用(認可はそちらで一元管理)。
            # 入力検証はAnnounceEventsController(範囲外・型違い・未知の項目・未知のイベントは400で何も保存しない)。
            if path == "/api/announce_events" and announce_events is not None:
                status, payload = announce_events.save(data)
                self._send_json(status, payload)
                return
            if path == "/api/announce_events/test" and announce_events is not None:
                status, payload = announce_events.test(data)
                self._send_json(status, payload)
                return
            # 【2026-10-30追加】文化祭の出店紹介(docs/announcements.md)。保存(→音声の作成を2台目へ依頼)・音声を作る・保存済み音声の試し再生。
            # 書込み扱いなのでguestは上の認可で403、operatorは roles.API_RULES の announce.pr_intro(保留=付与不可)/announce.test。
            # 入力検証はPrIntroController(範囲外・型違い・未知の項目・長すぎる文章は400で何も保存しない)。
            if path == "/api/pr_intro" and pr_intro is not None:
                status, payload = pr_intro.save(data)
                self._send_json(status, payload)
                return
            if path == "/api/pr_intro/prepare" and pr_intro is not None:
                status, payload = pr_intro.prepare(data)
                self._send_json(status, payload)
                return
            if path == "/api/pr_intro/test" and pr_intro is not None:
                status, payload = pr_intro.test(data)
                self._send_json(status, payload)
                return

            # 【2026-10-27追加】メニューの追加・名前の変更・削除・初期食数(iPad表示の編集と同じ DisplayHub.edit_menu。新しい一覧の全体を
            # /nokosahen/menu_init で送る)。guestは上の認可で403、operatorは roles.API_RULES の display.menu_edit を許可されたときだけ。
            # 入力検証は DisplayHub/plan_menu_edit(名前1〜40字・重複不可・最大12品・初期食数0〜99999・存在しない品の変更/削除は拒否)。
            if path == "/api/menu/edit" and display_hub is not None:
                status, payload = display_hub.edit_menu(data)
                _AUDIT.log(logging.INFO if status == 200 else logging.WARNING, "MENU_EDIT account=%s op=%r name=%r -> %s",
                           account, str(data.get("op"))[:20], str(data.get("name"))[:40], status)
                self._send_json(status, payload)
                return

            # 【2026-10-09追加】腹部ディスプレイの表示モード切替(ラズパイのkiosk_admin.pyへ中継)。
            # guestは上の認可で403。入力検証はSurveyAdminProxy.set_mode()({"mode":"survey|customer"}のみ)。
            if path == "/api/survey/mode" and survey_admin is not None:
                base = _eph_prepare(path, data, persist)
                status, payload = survey_admin.set_mode(data)
                if status == 200 and payload.get("ok"):
                    _eph_commit(path, data, account, persist, base)
                self._send_json(status, payload)
                return

            # 【2026-11-08追加】顔⇄アンケートの自動切替のON/OFF({"enabled": true|false})。iPad表示(LocalDisplayAdmin)のときだけ。
            # 権限は表示切替と同じ display.survey_mode(roles.API_RULES)。
            if path == "/api/survey/auto" and survey_admin is not None:
                fn = getattr(survey_admin, "set_auto", None)
                if fn is None:
                    self._send_json(HTTPStatus.NOT_IMPLEMENTED, {"ok": False, "error": "unsupported", "message": "この表示方式では自動切替は使えません"})
                    return
                status, payload = fn(data)
                _AUDIT.info("DISPLAY_AUTO account=%s enabled=%r -> %s", account, data.get("enabled"), status)
                self._send_json(status, payload)
                return

            # 【2026-10-09追加】模擬POS(OCRテスト画面)の数字をランダム化/元に戻す/指定する(ラズパイのkiosk_admin.pyへ中継)。
            # guestは上の認可で403。入力検証はSurveyAdminProxy.set_ocr_mock()(許可リスト・7個の0〜9999)。
            if path == "/api/survey/ocr_mock" and survey_admin is not None:
                base = _eph_prepare(path, data, persist)
                status, payload = survey_admin.set_ocr_mock(data)
                if status == 200 and payload.get("ok"):
                    _eph_commit(path, data, account, persist, base)
                self._send_json(status, payload)
                return

            # 【2026-10-09追加】OCRカメラの画質プロファイル切替(デモ/本番。OCRの POST /api/settings へ中継)。
            # guestは上の認可で403。入力検証はOcrAdminProxy.set_profile()({"camera_profile":"demo|production"}のみ)。
            if path == "/api/ocr/profile" and ocr_admin is not None:
                status, payload = ocr_admin.set_profile(data)
                self._send_json(status, payload)
                return

            # 【2026-10-10追加】OCRカメラの画質の自動調整/元に戻す/中断(OCRの POST /api/autotune へ中継)。
            # guestは上の認可で403。入力検証はOcrAdminProxy.set_autotune()({"action":"start|revert|cancel"}のみ)。
            if path == "/api/ocr/autotune" and ocr_admin is not None:
                status, payload = ocr_admin.set_autotune(data)
                self._send_json(status, payload)
                return

            if path == "/api/recommend_enabled" and behavior_control is not None:
                enabled = data.get("enabled")  # 【2026-11-04】必須(無い・壊れた本文でONにならないように)
                if not isinstance(enabled, bool):  # 【2026-10-21】文字列"false"がTrue(ON)になっていた。true/false以外(無い場合を含む)は400
                    self._send_json(HTTPStatus.BAD_REQUEST, {"error": "bad_request", "message": "enabled は true / false で送ってください"})
                    return
                ok, message = behavior_control.set_recommend_enabled(enabled)
                self._send_json(HTTPStatus.OK, {"ok": ok, "message": message, "enabled": enabled})
                return

            # 【2026-11-04追加】近づく・見回す・向く・待機の揺動のスイッチ(スマホ版と同じAPI)。
            # PC版のサーバーに受け口が無く、ボタンを押しても404でONにならなかった。
            for _api, _setter in (
                ("/api/person_interaction", "set_person_interaction_enabled"),
                ("/api/scan_enabled", "set_scan_enabled"),
                ("/api/face_enabled", "set_face_enabled"),
                ("/api/idle_wiggle", "set_idle_wiggle_enabled"),
            ):
                if path == _api and behavior_control is not None:
                    enabled = data.get("enabled")  # 必須(本文が空でONにならないように)
                    if not isinstance(enabled, bool):  # true/false以外(無い場合を含む)は400
                        self._send_json(HTTPStatus.BAD_REQUEST, {"error": "bad_request", "message": "enabled は true / false で送ってください"})
                        return
                    ok, message = getattr(behavior_control, _setter)(enabled)
                    self._send_json(HTTPStatus.OK, {"ok": ok, "message": message, "enabled": enabled})
                    return

            # 【2026-10-06追加】時間帯・状況ルールの保存/再読み込み(guestは上の認可で403)。
            # 保存は検証→バックアップ→原子的置換。不正なら400で理由一覧を返し、何も書かない。
            if path == "/api/behavior_rules/save" and behavior_rules is not None:
                body, err = parse_json_strict(getattr(self, "_raw_body", None) or b"")
                if err is not None or not isinstance(body, dict):
                    self._send_json(
                        HTTPStatus.BAD_REQUEST,
                        {"ok": False, "errors": [err or "JSONオブジェクトで送ってください(本文が空/大きすぎる場合も含む)"]},
                    )
                    return
                status, payload = behavior_rules.save(body)
                self._send_json(status, payload)
                return
            if path == "/api/behavior_rules/reload" and behavior_rules is not None:
                status, payload = behavior_rules.request_reload()
                self._send_json(status, payload)
                return

            # 【2026-10-09追加】ビヘイビアツリーの検証/保存/既定に戻す(guestは上の認可で403)。
            # 保存は検証→バックアップ→原子的置換。不正なら400で理由一覧を返し、何も書かない。
            if path in ("/api/behavior_tree", "/api/behavior_tree/validate", "/api/behavior_tree/reset")                     and behavior_tree is not None:
                body, err = parse_tree_json_strict(getattr(self, "_raw_body", None) or b"")
                if err is not None or not isinstance(body, dict):
                    self._send_json(
                        HTTPStatus.BAD_REQUEST,
                        {"ok": False, "errors": [err or "JSONオブジェクトで送ってください(本文が空/大きすぎる場合も含む)"]},
                    )
                    return
                if path == "/api/behavior_tree":
                    status, payload = behavior_tree.save(body)
                elif path == "/api/behavior_tree/validate":
                    status, payload = behavior_tree.validate_payload(body)
                else:
                    status, payload = behavior_tree.reset(body)
                self._send_json(status, payload)
                return

            # 【2026-10-23追加】スケジュールの検証/保存/実行/停止(guestは上の認可で403、operatorは roles.API_RULES の権限)。
            # 保存は検証OKだけ。実行は保存済みのスケジュールを木のファイルへコンパイルして書き、directorへ合図する。docs/schedule.md
            if path in ("/api/schedule", "/api/schedule/preview", "/api/schedule/run", "/api/schedule/stop") and schedule is not None:
                if path in ("/api/schedule/run", "/api/schedule/stop"):
                    status, payload = schedule.run() if path == "/api/schedule/run" else schedule.stop()
                    _AUDIT.warning("SCHEDULE_%s account=%s ok=%s", path.rsplit("/", 1)[1].upper(), account, payload.get("ok"))
                    self._send_json(status, payload)
                    return
                body, err = parse_tree_json_strict(getattr(self, "_raw_body", None) or b"")
                if err is not None or not isinstance(body, dict):
                    self._send_json(
                        HTTPStatus.BAD_REQUEST,
                        {"ok": False, "summary": [], "warnings": [],
                         "errors": [err or "JSONオブジェクトで送ってください(本文が空/大きすぎる場合も含む)"]},
                    )
                    return
                status, payload = schedule.preview(body) if path == "/api/schedule/preview" else schedule.save(body)
                if path == "/api/schedule":
                    _AUDIT.info("SCHEDULE_SAVED account=%s ok=%s", account, payload.get("ok"))
                self._send_json(status, payload)
                return

            # 【2026-10-06追加】呼びかけ音声設定の保存・お試し生成(ドライラン)・試し発話。
            # guestは上の_authorize(require_write=True)で403。入力検証はVoiceSettingsController。
            if path == "/api/voice_settings/save" and voice_settings is not None:
                base = _eph_prepare(path, data, persist)
                status, payload = voice_settings.save(data, persist=persist)
                if isinstance(payload, dict):
                    ann_ok = bool((payload.get("announce") or {}).get("ok"))
                    voice_ok = bool((payload.get("voice") or {}).get("ok"))
                    _eph_commit(path, data, account, persist, base,
                                keep=lambda k: ann_ok if k == "announce.min_interval" else voice_ok)
                self._send_json(status, payload)
                return
            if path == "/api/voice_settings/preview" and voice_settings is not None:
                try:
                    menu_state = (state_getter() or {}).get("menu_state")
                except Exception:  # noqa: BLE001 - 状態が取れなくても見本の文脈で試せる
                    menu_state = None
                status, payload = voice_settings.preview(data, menu_state)
                self._send_json(status, payload)
                return
            if path == "/api/voice_settings/speak" and voice_settings is not None:
                status, payload = voice_settings.speak(data)
                self._send_json(status, payload)
                return

            if path == "/api/force_demo" and behavior_control is not None:
                ok, message = behavior_control.trigger_demo()
                self._send_json(HTTPStatus.OK, {"ok": ok, "message": message})
                return

            if path == "/api/crowd_background/calibrate":
                if crowd_background is None:
                    self._send_json(
                        HTTPStatus.NOT_FOUND,
                        {"error": "crowd background not available on this system"},
                    )
                    return
                ok, message, request_id = crowd_background.request_calibration()
                self._send_json(HTTPStatus.OK, {"ok": ok, "message": message, "request_id": request_id})
                return

            if path == "/api/checkout_cam_assign":
                if checkout_cams is None:
                    self._send_json(
                        HTTPStatus.NOT_FOUND,
                        {"error": "checkout cameras not available on this system"},
                    )
                    return
                camera_id = str(data.get("camera_id", "")).strip()
                menu = str(data.get("menu", "")).strip()
                direction = str(data.get("direction", "")).strip()
                if direction and direction not in ("both", "left_to_right", "right_to_left"):
                    self._send_json(
                        HTTPStatus.BAD_REQUEST,
                        {"error": "direction must be one of both/left_to_right/right_to_left"},
                    )
                    return
                # menu/directionはどちらか一方だけの更新も許す(片方だけ変えたい場合があるため)。
                if not camera_id or (not menu and not direction):
                    self._send_json(
                        HTTPStatus.BAD_REQUEST,
                        {"error": "camera_id and (menu or direction) are required"},
                    )
                    return
                checkout_cams.assign(camera_id, menu, direction or None)
                self._send_json(HTTPStatus.OK, {"ok": True})
                return

            self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

    return Handler


def make_mobile_handler(
    html_getter: Callable[[], bytes],
    state_getter: Callable[[], Dict[str, Any]],
    teleop: TeleopController,
    auto_speed: Optional[AutoSpeedController] = None,
    auth_username: str = "",
    auth_password: str = "",
    restricted_username: str = "",
    restricted_password: str = "",
    enforce_single_controller: bool = True,
    behavior_control: Optional[BehaviorDirectorControlController] = None,
    bgm: Optional[BgmProxy] = None,
    nav_speed: Optional[NavSpeedController] = None,
    account_store: Optional[AccountStore] = None,
    ephemeral: Optional[EphemeralOverrides] = None,
):
    """スマホから機体の移動制御(手動運転・ソフト緊急停止・最高速度設定)だけを
    行うための最小限のHTTPハンドラ(`/`, `/api/state`, `/api/teleop`,
    `/api/soft_estop`, `/api/speed_settings`)。

    地図クリック運転・LiDAR/カメラ映像・決済横カメラ・残数テーブルは扱わない
    (それらはPC版の操作盤ダッシュボード側を参照)。認証は呼び出し側と同じ
    auth_username/auth_password をそのまま渡すことで、PC版と共通アカウントになる
    (ただし同時に1台までの制限はPC版とは独立にカウントする、
    `make_dashboard_handler`参照)。`guest`(パスワード無し・人数無制限・閲覧専用)
    も同様に使える。`enforce_single_controller`の意味は`make_dashboard_handler`
    と同じ(完成版でのみ真、Stage1/Stage2は偽)。
    `teleop`/`auto_speed`はPC版ダッシュボードと同じインスタンスを渡すことを想定
    しており、それによりソフト緊急停止の状態・最高速度設定がPC版と食い違わない
    (同じNode内の同じオブジェクトを両方のHTTPサーバから参照するだけなので)。

    `restricted_username`/`restricted_password`(2026-07-17追加、スマホ版限定):
    機体の手動運転・ソフト緊急停止はできるが、最高速度設定(/api/speed_settings
    のPOST)だけはできない専用アカウント。TL/guestとは別枠で、空文字のままなら
    このアカウント自体を無効化する。
    """
    gate = SingleControllerGate(enforce=enforce_single_controller)
    sessions = SessionStore()
    login_accounts = {auth_username: auth_password, "guest": ""}
    if restricted_username:
        login_accounts[restricted_username] = restricted_password
    # 【2026-10-13追加】operator(弱いアカウント)。PC版と同じアカウント表・同じ権限表を使う。docs/accounts.md
    accounts_db = account_store if account_store is not None else AccountStore()
    accounts_db.set_reserved(["guest", auth_username, restricted_username], restricted_names=[restricted_username])  # 【2026-11-04】restricted_accountはadminでなくrestrictedロール
    login_limiter = LoginLimiter()
    eph = ephemeral if ephemeral is not None else EphemeralOverrides()
    denied_log = _DenyLog()

    class Handler(BaseHTTPRequestHandler):
        # 【2026-10-21】読み取りのタイムアウト。本文を送らない接続・リクエスト行を送らない接続を大量に張られても、
        # スレッドとソケットが永久に残らないようにする(1台目はCPU負荷が高く、固まると緊急停止の操作も届かなくなる)。
        timeout = 15
        def log_message(self, fmt: str, *args: Any) -> None:  # noqa: A003
            pass

        def _authorize(self, require_write: bool, redirect: bool = False) -> Optional[str]:
            account, err = _resolve_control_account(self, auth_username, gate, sessions)
            if err == "auth_required":
                if redirect:
                    _send_redirect_to_login(self)
                else:
                    self._send_json(HTTPStatus.UNAUTHORIZED, {"error": "auth_required"})
                return None
            if err == "occupied":
                _send_occupied_response(self, redirect)
                return None
            if require_write and account == "guest":
                self._send_json(
                    HTTPStatus.FORBIDDEN,
                    {"error": "guest_read_only", "message": "guestアカウントは閲覧専用です。"},
                )
                return None
            return account

        def _send(self, status: int, body: bytes, content_type: str) -> None:
            body = inject_role_ui(body, content_type, urlparse(self.path).path)  # operator用の画面制御(NKRole)
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _send_json(self, status: int, payload: dict) -> None:
            self._send(
                status,
                json.dumps(payload, ensure_ascii=False).encode("utf-8"),
                "application/json; charset=utf-8",
            )

        def _read_json(self) -> dict:
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                return {}
            if length <= 0 or length > 1024 * 64:
                return {}
            try:
                data = json.loads(self.rfile.read(length).decode("utf-8"))
            except Exception:  # noqa: BLE001
                return {}
            return data if isinstance(data, dict) else {}

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            if path == "/login":
                self._send(HTTPStatus.OK, _LOGIN_HTML_BYTES, "text/html; charset=utf-8")
                return
            is_page = path in ("/", "/index.html")
            account = self._authorize(require_write=False, redirect=is_page)
            if account is None:
                return
            if path in ("/", "/index.html"):
                self._send(HTTPStatus.OK, html_getter(), "text/html; charset=utf-8")
                return
            if path == "/api/whoami":
                self._send_json(
                    HTTPStatus.OK,
                    {
                        "account": account,
                        # スマホ版JS側で速度設定UIを無効化するためのフラグ
                        # (機体操作+緊急停止のみ許可、速度変更は不可のアカウント)。
                        "restricted": bool(restricted_username) and account == restricted_username,
                        **{k: v for k, v in _whoami_payload(accounts_db, eph, account).items() if k != "account"},
                    },
                )
                return
            # 【2026-10-03追加】BGM状態(voice_bridgeの/bgmを中継)。guestを含む全ログイン
            # アカウントが閲覧可。bgmがNoneまたはbgm_url空ならconfigured:falseを返し、
            # UI側は項目ごと隠す。
            if path == "/api/bgm":
                self._send_json(
                    HTTPStatus.OK,
                    bgm.get_status() if bgm is not None else {"ok": False, "configured": False, "error": "BGM未設定"},
                )
                return
            if path == "/api/state":
                self._send_json(HTTPStatus.OK, state_getter())
                return
            # 【2026-10-24追加】手動運転の人避け(既定OFF)の状態。guestを含む全ログインアカウントが閲覧可。docs/manual_person_guard.md
            if path == "/api/manual_person_guard":
                self._send_json(HTTPStatus.OK, _manual_person_guard_payload(teleop, accounts_db, account))
                return
            if path == "/api/speed_settings":
                self._send_json(HTTPStatus.OK, _speed_settings_payload(teleop, auto_speed, nav_speed))
                return
            if path == "/api/nav_speed" and nav_speed is not None:
                # 実行中のNav2から現在値を読み戻す(最大3秒/ノード)。表示専用で何も書かない。
                body = dict(nav_speed.status())
                body["read_back"] = nav_speed.read_back()
                self._send_json(HTTPStatus.OK, body)
                return
            self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

        def do_POST(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            # 【2026-10-30追加】書き込みAPI(ログインを含む全POST)は同じオリジンからだけ(CSRFの多層防御)。docs/ipad_display.md・docs/accounts.md
            if not _post_origin_guard(self):
                return
            if path == "/api/login":
                _handle_login_post(self, self._read_json(), login_accounts, sessions, accounts_db, login_limiter)
                return

            # 【2026-10-21】ログアウトは、閲覧専用のguestにも許す(以前は書込み扱いで403になり、guestのセッションが24時間残った)
            account = self._authorize(require_write=(path != "/api/logout"))
            if account is None:
                return
            data = self._read_json()

            if path == "/api/logout":
                _handle_logout_post(self, account, gate, sessions)
                return

            # 【2026-10-13追加】権限チェック(PC版と同じ)。スマホ版のoperatorは手動運転・速度設定は不可。
            _role = accounts_db.role_of(account)  # 【2026-11-06】1回だけ(以前は _capability_gate と persist で2回呼び、毎回 stat が2×2回走っていた)
            if not _capability_gate(self, account, path, data, accounts_db, denied_log, role=_role):
                return
            persist = _role == _roles.ROLE_ADMIN

            if path == "/api/teleop":
                try:
                    forward = float(data.get("forward", 0.0))
                    turn = float(data.get("turn", 0.0))
                except (TypeError, ValueError):
                    self._send_json(HTTPStatus.BAD_REQUEST, {"error": "forward/turn must be numbers"})
                    return
                # 【2026-10-02追加】NaN/Infinityは拒否(NaNは全速になってしまう)。
                if not (math.isfinite(forward) and math.isfinite(turn)):
                    teleop.cmd_vel_pub.publish(Twist())
                    self._send_json(HTTPStatus.BAD_REQUEST, {"error": "forward/turn must be finite numbers"})
                    return
                # 【2026-07-17追加】restricted_username(restricted_account)はTLより
                # 優先度が低い。TLが直近運転中ならここでコマンドが破棄され、
                # ok:falseかつreason:"tl_active"を返す(モバイルJS側でバナー表示)。
                # 【2026-11-06】operatorもTLより低い優先度(restricted_accountと同じ)。ロールで決める。
                priority = teleop_priority_for_role(_role)
                accepted = teleop.handle_drive(
                    forward, turn, priority=priority,
                    source=build_teleop_source(account, self.client_address[0], data.get("tab_id")),
                    seq=parse_teleop_seq(data.get("seq")),
                )
                if accepted:
                    self._send_json(HTTPStatus.OK, {"ok": True})
                elif teleop.soft_estop_engaged:
                    # 【2026-08-05追加】TL優先度負け(tl_active)と緊急停止中を
                    # 区別する。以前はどちらもok:falseのみで、estop中は
                    # モバイルJS側が"TLが操作中です"と誤表示していた。
                    self._send_json(
                        HTTPStatus.OK,
                        {"ok": False, "reason": "estop_engaged", "message": "緊急停止中です。解除してから操作してください。"},
                    )
                else:
                    self._send_json(
                        HTTPStatus.OK,
                        {"ok": False, "reason": "tl_active", "message": "TLが操作中です"},
                    )
                return

            if path == "/api/soft_estop":
                # 【2026-11-04】解除は engaged が厳密に false のときだけ(null・0・""・[]・無しは「停止をかける」)。roles._estop_rule と同じ式
                engaged = data.get("engaged") is not False
                teleop.handle_soft_estop(engaged)
                self._send_json(HTTPStatus.OK, {"ok": True})
                return

            if path == "/api/speed_settings":
                if restricted_username and account == restricted_username:
                    self._send_json(
                        HTTPStatus.FORBIDDEN,
                        {
                            "error": "restricted_account",
                            "message": "このアカウントでは速度設定を変更できません。",
                        },
                    )
                    return
                status, body = _speed_settings_post(data, teleop, auto_speed, nav_speed)
                self._send_json(status, body)
                return

            # 【2026-10-03追加】BGM ON/OFF・音量(voice_bridgeの/bgmへ中継)。書込み扱いなので
            # guestは上の_authorize(require_write=True)で403。入力検証はBgmProxy.set()。
            if path == "/api/bgm" and bgm is not None:
                status, payload = bgm.set(data, persist=persist)
                if status == 200 and payload.get("ok"):
                    items = _eph_items_for(path, data)
                    if persist:
                        eph.discard(items)
                    else:
                        for item, value in items.items():
                            eph.record(item, value, account)
                self._send_json(status, payload)
                return

            if path == "/api/person_interaction" and behavior_control is not None:
                enabled = data.get("enabled")  # 【2026-11-04】必須(無い・壊れた本文でONにならないように)
                if not isinstance(enabled, bool):  # true/false以外(無い場合を含む)は400(文字列"false"がON扱いにならないように)
                    self._send_json(HTTPStatus.BAD_REQUEST, {"error": "bad_request", "message": "enabled は true / false で送ってください"})
                    return
                ok, message = behavior_control.set_person_interaction_enabled(enabled)
                self._send_json(HTTPStatus.OK, {"ok": ok, "message": message, "enabled": enabled})
                return

            if path == "/api/scan_enabled" and behavior_control is not None:
                enabled = data.get("enabled")  # 【2026-11-04】必須(無い・壊れた本文でONにならないように)
                if not isinstance(enabled, bool):  # true/false以外(無い場合を含む)は400
                    self._send_json(HTTPStatus.BAD_REQUEST, {"error": "bad_request", "message": "enabled は true / false で送ってください"})
                    return
                ok, message = behavior_control.set_scan_enabled(enabled)
                self._send_json(HTTPStatus.OK, {"ok": ok, "message": message, "enabled": enabled})
                return

            if path == "/api/face_enabled" and behavior_control is not None:
                enabled = data.get("enabled")  # 【2026-11-04】必須(無い・壊れた本文でONにならないように)
                if not isinstance(enabled, bool):  # true/false以外(無い場合を含む)は400
                    self._send_json(HTTPStatus.BAD_REQUEST, {"error": "bad_request", "message": "enabled は true / false で送ってください"})
                    return
                ok, message = behavior_control.set_face_enabled(enabled)
                self._send_json(HTTPStatus.OK, {"ok": ok, "message": message, "enabled": enabled})
                return

            if path == "/api/idle_wiggle" and behavior_control is not None:
                enabled = data.get("enabled")  # 【2026-11-04】必須(無い・壊れた本文でONにならないように)
                if not isinstance(enabled, bool):  # 【2026-10-21】文字列"false"がTrue(ON)になっていた。true/false以外(無い場合を含む)は400
                    self._send_json(HTTPStatus.BAD_REQUEST, {"error": "bad_request", "message": "enabled は true / false で送ってください"})
                    return
                ok, message = behavior_control.set_idle_wiggle_enabled(enabled)
                self._send_json(HTTPStatus.OK, {"ok": ok, "message": message, "enabled": enabled})
                return

            self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

    return Handler


def start_dashboard_server(node, host: str, port: int, handler_cls) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer((host, port), handler_cls)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    node.get_logger().info(f"dashboard listening on http://{host}:{port}")
    return server


_MAP_EDITOR_HTML = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>のこさへん - 地図の範囲編集</title>
<style>
  :root { color-scheme: light; font-family: system-ui, -apple-system, "Segoe UI", sans-serif; background:#f4f6f5; color:#16211d; }
  html, body { height:100%; margin:0; overflow:hidden; }
  header { box-sizing:border-box; height:44px; padding:0 18px; background:#124739; color:#fff; display:flex; justify-content:space-between; align-items:center; gap:8px; }
  header a { color:#cfe9df; text-decoration:none; font-size:13px; }
  main {
    height:calc(100vh - 44px); box-sizing:border-box; padding:12px; display:grid; gap:12px;
    grid-template-columns: 300px 1fr;
  }
  @media (max-width:800px) {
    html, body { overflow:auto; }
    main { height:auto; grid-template-columns:1fr; }
  }
  .sidebar { display:flex; flex-direction:column; gap:12px; overflow-y:auto; min-height:0; }
  section { background:#fff; border:1px solid #dce4df; border-radius:8px; padding:12px; flex:0 0 auto; }
  h2 { margin:0 0 8px; font-size:14px; }
  .row { display:flex; gap:8px; flex-wrap:wrap; align-items:center; }
  button { padding:8px 12px; font-size:13px; border:0; border-radius:6px; background:#176b56; color:#fff; cursor:pointer; }
  button.danger { background:#a32d2d; }
  button.secondary { background:#4e5e57; }
  .hint { font-size:12px; color:#53635c; margin-top:6px; }
  .zone-list { list-style:none; margin:8px 0 0; padding:0; display:flex; flex-direction:column; gap:6px; }
  .zone-list li { display:flex; justify-content:space-between; align-items:center; background:#f7f2ea; border-radius:6px; padding:6px 10px; font-size:12px; }
  .zone-list button { padding:4px 8px; font-size:11px; }
  .legend { font-size:12px; color:#53635c; display:flex; gap:14px; flex-wrap:wrap; flex:0 0 auto; margin-bottom:8px; }
  .legend span { display:inline-flex; align-items:center; gap:6px; }
  .legend i { width:14px; height:14px; border-radius:2px; display:inline-block; }
  .map-pane { display:flex; flex-direction:column; min-height:0; }
  .canvas-wrap { flex:1 1 auto; min-height:0; display:flex; align-items:center; justify-content:center; }
  canvas { max-width:100%; max-height:100%; background:#0c1b16; border-radius:8px; cursor:crosshair; touch-action:none; }
</style>
</head>
<body>
<header>
  <h1 style="margin:0;font-size:18px;">地図の範囲編集</h1>
  <a href="/">&larr; 操作盤に戻る</a>
</header>
<main>
  <div class="sidebar">
    <section>
      <h2>走行可能範囲(外周、1つだけ)</h2>
      <div class="row">
        <button id="boundarySetBtn">地図を2回クリックして設定</button>
        <button id="boundaryClearBtn" class="secondary">解除</button>
      </div>
      <p class="hint" id="boundaryHint">この画面は軸平行の矩形のみ設定できます(回転した範囲は操作盤の地図で設定)。既存の回転範囲は真の形で表示します。</p>
    </section>

    <section>
      <h2>立入禁止エリア(内側、いくつでも追加可)</h2>
      <div class="row">
        <button id="zoneAddBtn">地図を2回クリックして追加</button>
        <button id="zoneClearAllBtn" class="danger">全て削除</button>
      </div>
      <p class="hint" id="zoneHint"></p>
      <ul class="zone-list" id="zoneList"></ul>
    </section>
  </div>

  <section class="map-pane">
    <div class="legend">
      <span><i style="background:#ffb000;"></i>走行可能範囲(外周)</span>
      <span><i style="background:#e0453f;"></i>立入禁止エリア</span>
      <span><i style="background:#ff5a3c;border-radius:50%;"></i>現在位置</span>
    </div>
    <div class="canvas-wrap">
      <canvas id="mapCanvas" width="800" height="800"></canvas>
    </div>
  </section>
</main>
<script>
__NK_FIELD_JS__
  // 401(未ログイン/セッション切れ)を受け取ったら自動でログイン画面へ誘導する。
  (function () {
    const rawFetch = window.fetch.bind(window);
    window.fetch = async (...args) => {
      const res = await rawFetch(...args);
      if (res.status === 401 && location.pathname !== "/login") {
        location.href = "/login";
      }
      return res;
    };
  })();

  const canvas = document.getElementById("mapCanvas");
  const ctx = canvas.getContext("2d");
  let mapMeta = null;
  let robotPose = null;
  let currentBoundary = null;
  let currentZones = [];

  function worldToPixel(x, y) {
    if (!mapMeta) return null;
    const px = (x - mapMeta.origin_x) / mapMeta.resolution;
    const py = mapMeta.height - (y - mapMeta.origin_y) / mapMeta.resolution;
    return [px * (canvas.width / mapMeta.width), py * (canvas.height / mapMeta.height)];
  }

  function pixelToWorld(px, py) {
    const gx = px * (mapMeta.width / canvas.width);
    const gy = py * (mapMeta.height / canvas.height);
    const x = gx * mapMeta.resolution + mapMeta.origin_x;
    const y = (mapMeta.height - gy) * mapMeta.resolution + mapMeta.origin_y;
    return [x, y];
  }

  async function postJson(url, body) {
    const res = await fetch(url, {
      method: "POST", headers: {"Content-Type": "application/json"},
      body: JSON.stringify(body || {}),
    });
    return res.json();
  }

  function drawRect(minX, minY, maxX, maxY, color, dashed) {
    const p1 = worldToPixel(minX, minY);
    const p2 = worldToPixel(maxX, maxY);
    if (!p1 || !p2) return;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    if (dashed) ctx.setLineDash([6, 4]);
    ctx.strokeRect(
      Math.min(p1[0], p2[0]), Math.min(p1[1], p2[1]),
      Math.abs(p2[0] - p1[0]), Math.abs(p2[1] - p1[1])
    );
    ctx.restore();
  }

  // 【2026-10-06】走行可能範囲は回転矩形(center/width/height/yaw)。真の形で描く。
  // 旧形式のみ(yawなし)ならmin/maxの軸平行矩形として描く。
  function drawBoundaryPoly(b, color) {
    let cx, cy, w, h, yaw;
    if (b.width !== undefined && b.center_x !== undefined) {
      cx = b.center_x; cy = b.center_y; w = b.width; h = b.height; yaw = b.yaw || 0;
    } else {
      cx = (b.min_x + b.max_x) / 2; cy = (b.min_y + b.max_y) / 2;
      w = b.max_x - b.min_x; h = b.max_y - b.min_y; yaw = 0;
    }
    const c = Math.cos(yaw), s = Math.sin(yaw);
    const pts = [[-w/2, -h/2], [w/2, -h/2], [w/2, h/2], [-w/2, h/2]].map(
      ([lx, ly]) => worldToPixel(cx + lx * c - ly * s, cy + lx * s + ly * c));
    if (pts.some(p => !p)) return;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    pts.forEach((p, i) => { if (i === 0) ctx.moveTo(p[0], p[1]); else ctx.lineTo(p[0], p[1]); });
    ctx.closePath();
    ctx.stroke();
    ctx.restore();
  }

  function drawRobot() {
    if (!robotPose || !mapMeta) return;
    const p = worldToPixel(robotPose.x, robotPose.y);
    if (!p) return;
    ctx.fillStyle = "#ff5a3c";
    ctx.beginPath();
    ctx.arc(p[0], p[1], 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#ff5a3c";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(p[0], p[1]);
    ctx.lineTo(p[0] + 18 * Math.cos(-robotPose.yaw), p[1] + 18 * Math.sin(-robotPose.yaw));
    ctx.stroke();
  }

  function redraw() {
    if (currentBoundary && currentBoundary.enabled) {
      drawBoundaryPoly(currentBoundary, "#ffb000");
    }
    for (const z of currentZones) {
      drawRect(z.min_x, z.min_y, z.max_x, z.max_y, "#e0453f", false);
    }
    drawRobot();
  }

  async function fetchMap() {
    try {
      const res = await fetch("/api/map");
      if (!res.ok) {
        // 地図リセット直後等、サーバー側の地図が無くなった状態。表示側も
        // 古い地図の描画を消す(2026-08-06、他のダッシュボードと同じ修正)。
        if (mapMeta !== null) {
          mapMeta = null;
          ctx.clearRect(0, 0, canvas.width, canvas.height);
        }
        return;
      }
      const m = await res.json();
      mapMeta = m;
      const bytes = Uint8Array.from(atob(m.data_base64), c => c.charCodeAt(0));
      const img = ctx.createImageData(m.width, m.height);
      for (let i = 0; i < m.width * m.height; i++) {
        const v = bytes[i] - 1;
        let shade;
        if (v < 0) shade = 60;
        else shade = 255 - Math.round(v * 2.3);
        const row = m.height - 1 - Math.floor(i / m.width);
        const col = i % m.width;
        const idx = (row * m.width + col) * 4;
        img.data[idx] = img.data[idx+1] = img.data[idx+2] = Math.max(0, Math.min(255, shade));
        img.data[idx+3] = 255;
      }
      const off = document.createElement("canvas");
      off.width = m.width; off.height = m.height;
      off.getContext("2d").putImageData(img, 0, 0);
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.drawImage(off, 0, 0, canvas.width, canvas.height);
      redraw();
    } catch (e) { /* 無視、次のポーリングで再試行 */ }
  }

  async function fetchState() {
    try {
      const res = await fetch("/api/state");
      if (!res.ok) return;
      const s = await res.json();
      if (s.robot_pose) { robotPose = s.robot_pose; redraw(); }
    } catch (e) { /* 無視 */ }
  }

  async function fetchBoundary() {
    const tk = NKField.ticket("boundary");
    try {
      const res = await fetch("/api/boundary");
      if (!res.ok) return;
      const b = await res.json();
      if (!NKField.fresh(tk)) return;  // 送信前に始まった古い取得結果は捨てる(範囲が一瞬元に戻らないように)
      currentBoundary = b;
      redraw();
    } catch (e) { /* 無視 */ }
  }

  let zonesSig = "";
  async function fetchZones() {
    const tk = NKField.ticket("zones");
    try {
      const res = await fetch("/api/keepout_zones");
      if (!res.ok) return;
      const d = await res.json();
      if (!NKField.fresh(tk)) return;
      currentZones = d.zones || [];
      const sig = JSON.stringify(currentZones);
      if (sig !== zonesSig) { zonesSig = sig; renderZoneList(); }  // 中身が変わった時だけ一覧を作り直す(押そうとしたボタンが消えないように)
      redraw();
    } catch (e) { /* 無視 */ }
  }

  function renderZoneList() {
    const ul = document.getElementById("zoneList");
    ul.innerHTML = "";
    for (const z of currentZones) {
      const li = document.createElement("li");
      const label = document.createElement("span");
      label.textContent = `#${z.id}: (${z.min_x.toFixed(2)}, ${z.min_y.toFixed(2)}) - (${z.max_x.toFixed(2)}, ${z.max_y.toFixed(2)})`;
      const del = document.createElement("button");
      del.className = "danger";
      del.textContent = "削除";
      del.addEventListener("click", async () => {
        NKField.bump("zones");
        const d = await postJson("/api/keepout_zones/remove", { id: z.id });
        document.getElementById("zoneHint").textContent = d.message || "";
        NKField.bump("zones");
        await fetchZones();
        if (d.ok !== false && currentZones.some((q) => q.id === z.id)) document.getElementById("zoneHint").textContent = "削除が反映されていません(サーバーにまだ残っています)。もう一度押してください";
      });
      li.appendChild(label);
      li.appendChild(del);
      ul.appendChild(li);
    }
  }

  // --- クリックでの範囲指定(走行可能範囲・立入禁止エリア共通の2点クリック方式) ---
  let pickMode = null; // null / "boundary" / "zone"
  let firstPoint = null;
  const boundaryHint = document.getElementById("boundaryHint");
  const zoneHint = document.getElementById("zoneHint");

  document.getElementById("boundarySetBtn").addEventListener("click", () => {
    pickMode = "boundary";
    firstPoint = null;
    boundaryHint.textContent = "地図上で範囲の1つ目の角をクリックしてください";
  });
  document.getElementById("boundaryClearBtn").addEventListener("click", async () => {
    pickMode = null;
    firstPoint = null;
    NKField.bump("boundary");
    const d = await postJson("/api/boundary/clear", {});
    boundaryHint.textContent = d.message || "";
    NKField.bump("boundary");
    await fetchBoundary();
    if (d.ok !== false && currentBoundary && currentBoundary.enabled) boundaryHint.textContent = "範囲の解除が反映されていません(サーバーでは範囲が有効のままです)。もう一度押してください";
  });
  document.getElementById("zoneAddBtn").addEventListener("click", () => {
    pickMode = "zone";
    firstPoint = null;
    zoneHint.textContent = "地図上で立入禁止エリアの1つ目の角をクリックしてください";
  });
  document.getElementById("zoneClearAllBtn").addEventListener("click", async () => {
    pickMode = null;
    firstPoint = null;
    NKField.bump("zones");
    const d = await postJson("/api/keepout_zones/clear", {});
    zoneHint.textContent = d.message || "";
    NKField.bump("zones");
    await fetchZones();
    if (d.ok !== false && currentZones.length) zoneHint.textContent = "全削除が反映されていません(サーバーにまだ残っています)。もう一度押してください";
  });

  canvas.addEventListener("click", (ev) => {
    if (!mapMeta || !pickMode) return;
    const rect = canvas.getBoundingClientRect();
    const px = (ev.clientX - rect.left) * (canvas.width / rect.width);
    const py = (ev.clientY - rect.top) * (canvas.height / rect.height);
    const [x, y] = pixelToWorld(px, py);

    if (!firstPoint) {
      firstPoint = [x, y];
      const hint = pickMode === "boundary" ? boundaryHint : zoneHint;
      hint.textContent = "2つ目の角(対角)をクリックしてください";
      return;
    }
    const [x1, y1] = firstPoint;
    const min_x = Math.min(x1, x), max_x = Math.max(x1, x);
    const min_y = Math.min(y1, y), max_y = Math.max(y1, y);
    const mode = pickMode;
    pickMode = null;
    firstPoint = null;
    (async () => {
      const near = (a, b) => Math.abs(Number(a) - Number(b)) < 0.02;
      if (mode === "boundary") {
        NKField.bump("boundary");
        const d = await postJson("/api/boundary/set", { min_x, max_x, min_y, max_y });
        boundaryHint.textContent = d.message || "";
        NKField.bump("boundary");
        await fetchBoundary();
        const b = currentBoundary;  // 送った範囲がサーバーに保存されたか確かめる
        if (d.ok !== false && !(b && b.enabled && near(b.min_x, min_x) && near(b.max_x, max_x) && near(b.min_y, min_y) && near(b.max_y, max_y))) {
          boundaryHint.textContent = "走行可能範囲が反映されていません(サーバーの範囲が送った範囲と違います)。もう一度設定してください";
        }
      } else {
        NKField.bump("zones");
        const d = await postJson("/api/keepout_zones/add", { min_x, max_x, min_y, max_y });
        zoneHint.textContent = d.message || "";
        NKField.bump("zones");
        await fetchZones();
        if (d.ok !== false && !currentZones.some((z) => near(z.min_x, min_x) && near(z.max_x, max_x) && near(z.min_y, min_y) && near(z.max_y, max_y))) {
          zoneHint.textContent = "立入禁止エリアが反映されていません(サーバーの一覧に見当たりません)。もう一度設定してください";
        }
      }
    })();
  });

  setInterval(fetchMap, 2000);
  setInterval(fetchState, 1000);
  NKField.poll(fetchBoundary, 5000);
  NKField.poll(fetchZones, 5000);
  fetchMap();
  fetchState();
  fetchBoundary();
  fetchZones();
</script>
</body>
</html>
"""


def build_map_editor_html() -> bytes:
    """走行可能範囲・立入禁止エリアを大画面の地図上で編集する専用ページ。

    stage2_dashboard_node.py/full_dashboard_node.py の両方から、メインの
    操作盤ダッシュボードとは別ページ(`/map_editor`)として配信される
    (2026-07-17、ユーザー要望: 埋め込みの小さい地図では範囲指定がしづらい
    ため、専用の大画面ページを別途用意したい)。
    """
    return _MAP_EDITOR_HTML.replace("__NK_FIELD_JS__", NK_FIELD_JS).encode("utf-8")


# 【2026-07-22追加】カメラ/手動によるメニュー消費(count_event)の推移を
# グラフで見えるようにする専用ページ(`/consumption_graph`)。CDN等の外部
# ライブラリは使わず(現地運用がインターネット非接続の想定のため)、素の
# Canvas 2D APIで折れ線(累計販売数の推移)を描画する。
_CONSUMPTION_GRAPH_HTML = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>のこさへん - 消費推移グラフ</title>
<style>
  :root { color-scheme: light; font-family: system-ui, -apple-system, "Segoe UI", sans-serif; background:#f4f6f5; color:#16211d; }
  html, body { height:100%; margin:0; overflow:hidden; }
  header { box-sizing:border-box; height:44px; padding:0 18px; background:#124739; color:#fff; display:flex; justify-content:space-between; align-items:center; gap:8px; }
  header a { color:#cfe9df; text-decoration:none; font-size:13px; }
  main {
    height:calc(100vh - 44px); box-sizing:border-box; padding:12px; display:grid; gap:12px;
    grid-template-columns: 260px 1fr;
  }
  @media (max-width:800px) {
    html, body { overflow:auto; }
    main { height:auto; grid-template-columns:1fr; }
  }
  .sidebar { display:flex; flex-direction:column; gap:12px; overflow-y:auto; min-height:0; }
  section { background:#fff; border:1px solid #dce4df; border-radius:8px; padding:12px; flex:0 0 auto; }
  h2 { margin:0 0 8px; font-size:14px; }
  .row { display:flex; gap:8px; flex-wrap:wrap; align-items:center; }
  button { padding:8px 12px; font-size:13px; border:0; border-radius:6px; background:#176b56; color:#fff; cursor:pointer; }
  select { padding:6px; font-size:13px; border:1px solid #cfd8d3; border-radius:6px; }
  .hint { font-size:12px; color:#53635c; margin-top:6px; }
  .legend { font-size:12px; color:#53635c; display:flex; gap:14px; flex-wrap:wrap; flex:0 0 auto; margin-bottom:8px; }
  .legend span { display:inline-flex; align-items:center; gap:6px; }
  .legend i { width:14px; height:14px; border-radius:2px; display:inline-block; }
  .estimated-badge { display:inline-block; padding:1px 6px; border-radius:10px; background:#f0e6fb; color:#6a3fa0; font-size:11px; font-weight:600; }
  .graph-pane { display:flex; flex-direction:column; min-height:0; gap:10px; }
  .canvas-wrap { flex:1 1 auto; min-height:0; display:flex; align-items:center; justify-content:center; }
  canvas { max-width:100%; max-height:100%; background:#fff; border:1px solid #dce4df; border-radius:8px; }
  table { width:100%; border-collapse:collapse; font-size:12px; }
  th, td { border-bottom:1px solid #e4ebe7; padding:4px 6px; text-align:left; }
  .table-wrap { max-height:220px; overflow-y:auto; border:1px solid #dce4df; border-radius:8px; }
  .export-btn { width:100%; margin-top:6px; }
  @media print {
    header, .sidebar { display:none; }
    main { grid-template-columns: 1fr; height:auto; }
    html, body { overflow:visible; }
    .table-wrap { max-height:none; overflow:visible; }
  }
</style>
</head>
<body>
<header>
  <h1 style="margin:0;font-size:18px;">消費推移グラフ</h1>
  <a href="/">&larr; 操作盤に戻る</a>
</header>
<main>
  <div class="sidebar">
    <section>
      <h2>絞り込み</h2>
      <div class="row">
        <label style="font-size:13px;">検知元
          <select id="sourceFilter">
            <option value="all">すべて</option>
            <option value="camera">カメラのみ</option>
            <option value="manual">手動入力のみ</option>
          </select>
        </label>
      </div>
      <div class="row" style="margin-top:8px;">
        <button id="refreshBtn">更新</button>
      </div>
      <p class="hint">「カメラのみ」は決済横カメラ(source: checkout_cam_*)による検知だけ、「手動入力のみ」はそれ以外(残数入力画面の+1/-1操作等)だけを表示します。</p>
    </section>

    <section>
      <h2>エクスポート(今表示中のデータ)</h2>
      <button id="exportCsvBtn" class="export-btn">CSVで保存(Excelで開けます)</button>
      <button id="exportTxtBtn" class="export-btn">TXTで保存</button>
      <button id="exportPdfBtn" class="export-btn">PDFとして保存(印刷)</button>
      <p class="hint">PDFはブラウザの印刷機能を使います(日本語もそのまま出力されます)。「送信先」で「PDFに保存」を選んでください。</p>
    </section>
  </div>

  <section class="graph-pane">
    <div class="legend" id="legend"></div>
    <div class="canvas-wrap">
      <canvas id="graphCanvas" width="900" height="480"></canvas>
    </div>
    <div class="table-wrap">
      <table>
        <thead><tr><th>時刻</th><th>メニュー</th><th>増減</th><th>検知元</th><th>残数</th><th>累計販売</th><th>種別</th></tr></thead>
        <tbody id="eventRows"><tr><td colspan="7">読み込み中...</td></tr></tbody>
      </table>
    </div>
  </section>
</main>
<script>
  // 401(未ログイン/セッション切れ)を受け取ったら自動でログイン画面へ誘導する。
  (function () {
    const rawFetch = window.fetch.bind(window);
    window.fetch = async (...args) => {
      const res = await rawFetch(...args);
      if (res.status === 401 && location.pathname !== "/login") {
        location.href = "/login";
      }
      return res;
    };
  })();

  const PALETTE = ["#176b56", "#a32d2d", "#2d6ca3", "#b57a1f", "#6a3d9a", "#1f8a8a", "#c0447a", "#5a7d1f"];
  const canvas = document.getElementById("graphCanvas");
  const ctx = canvas.getContext("2d");

  function isCameraSource(source) {
    return typeof source === "string" && source.startsWith("checkout_cam_");
  }

  function filterEvents(events) {
    const mode = document.getElementById("sourceFilter").value;
    if (mode === "camera") return events.filter((e) => isCameraSource(e.source));
    if (mode === "manual") return events.filter((e) => !isCameraSource(e.source));
    return events;
  }

  function drawGraph(events) {
    const w = canvas.width, h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    const padL = 50, padR = 16, padT = 16, padB = 36;
    const plotW = w - padL - padR, plotH = h - padT - padB;

    if (events.length === 0) {
      ctx.fillStyle = "#53635c";
      ctx.font = "13px system-ui, sans-serif";
      ctx.fillText("表示できるデータがありません", padL, padT + 20);
      return;
    }

    // メニュー毎に、時刻順の「累計販売数」の推移(階段状)を作る。
    const byMenu = {};
    for (const e of events) {
      if (!byMenu[e.menu]) byMenu[e.menu] = [];
      byMenu[e.menu].push(e);
    }
    const menus = Object.keys(byMenu).sort();

    const times = events.map((e) => new Date(e.timestamp).getTime());
    const tMin = Math.min(...times), tMax = Math.max(...times);
    const tSpan = Math.max(1, tMax - tMin);
    const yMax = Math.max(1, ...events.map((e) => e.sold_after));

    function xOf(t) { return padL + ((t - tMin) / tSpan) * plotW; }
    function yOf(v) { return padT + plotH - (v / yMax) * plotH; }

    // 軸・グリッド
    ctx.strokeStyle = "#e4ebe7";
    ctx.fillStyle = "#53635c";
    ctx.font = "11px system-ui, sans-serif";
    const yTicks = 5;
    for (let i = 0; i <= yTicks; i++) {
      const v = Math.round((yMax / yTicks) * i);
      const y = yOf(v);
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w - padR, y); ctx.stroke();
      ctx.fillText(String(v), 6, y + 4);
    }
    const xTicks = 4;
    for (let i = 0; i <= xTicks; i++) {
      const t = tMin + (tSpan / xTicks) * i;
      const x = xOf(t);
      const d = new Date(t);
      const label = `${String(d.getHours()).padStart(2,"0")}:${String(d.getMinutes()).padStart(2,"0")}`;
      ctx.fillText(label, x - 14, h - padB + 16);
    }

    ctx.strokeStyle = "#16211d";
    ctx.beginPath();
    ctx.moveTo(padL, padT); ctx.lineTo(padL, padT + plotH); ctx.lineTo(w - padR, padT + plotH);
    ctx.stroke();

    // メニュー毎の階段グラフ
    const legend = document.getElementById("legend");
    legend.innerHTML = "";
    menus.forEach((menu, i) => {
      const color = PALETTE[i % PALETTE.length];
      const pts = byMenu[menu];
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      // 推定値(TrendPredictorの外挿予測、2026-09-25追加)へつながる区間は
      // 破線にして、実測に基づく区間と見分けられるようにする。
      let prevX = null, prevY = null;
      pts.forEach((e) => {
        const x = xOf(new Date(e.timestamp).getTime());
        const y = yOf(e.sold_after);
        if (prevY !== null) {
          ctx.setLineDash(e.estimated ? [5, 4] : []);
          ctx.beginPath();
          ctx.moveTo(prevX, prevY);
          ctx.lineTo(x, prevY);
          ctx.lineTo(x, y);
          ctx.stroke();
        }
        prevX = x; prevY = y;
      });
      ctx.setLineDash([]);
      ctx.lineWidth = 1;

      // 推定値の点には中抜きの丸印を重ねて、実測点と見分けられるようにする。
      pts.forEach((e) => {
        if (!e.estimated) return;
        const x = xOf(new Date(e.timestamp).getTime());
        const y = yOf(e.sold_after);
        ctx.beginPath();
        ctx.arc(x, y, 4, 0, Math.PI * 2);
        ctx.fillStyle = "#fff";
        ctx.fill();
        ctx.lineWidth = 2;
        ctx.strokeStyle = color;
        ctx.stroke();
        ctx.lineWidth = 1;
      });

      const totalSold = pts.length ? pts[pts.length - 1].sold_after : 0;
      const estimatedCount = pts.filter((e) => e.estimated).length;
      const item = document.createElement("span");
      item.innerHTML = `<i style="background:${color};"></i>${escapeHtml(menu)}(累計${totalSold}食)` +
        (estimatedCount ? ` <span class="estimated-badge">推定${estimatedCount}件含む</span>` : "");
      legend.appendChild(item);
    });
  }

  function escapeHtml(value) {
    return String(value).replace(/[&<>"']/g, (ch) => ({
      "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"
    }[ch]));
  }

  function renderTable(events) {
    const tbody = document.getElementById("eventRows");
    if (events.length === 0) {
      tbody.innerHTML = '<tr><td colspan="7">表示できるデータがありません</td></tr>';
      return;
    }
    // 新しいものが上に来るよう、時刻降順で表示する。
    const rows = events.slice().reverse().map((e) => {
      const src = isCameraSource(e.source) ? `カメラ(${escapeHtml(e.source)})` : escapeHtml(e.source);
      const kind = e.estimated ? '<span class="estimated-badge">推定</span>' : "実測";
      return `<tr><td>${escapeHtml(e.timestamp)}</td><td>${escapeHtml(e.menu)}</td>` +
        `<td>${e.delta > 0 ? "-" + e.delta : "+" + (-e.delta)}</td><td>${src}</td>` +
        `<td>${e.remaining_after}</td><td>${e.sold_after}</td><td>${kind}</td></tr>`;
    });
    tbody.innerHTML = rows.join("");
  }

  let allEvents = [];
  async function loadEvents() {
    try {
      const res = await fetch("/api/consumption_log");
      if (!res.ok) return;
      const data = await res.json();
      allEvents = data.events || [];
      refresh();
    } catch (e) { /* 通信エラー時は前回表示のまま */ }
  }

  function refresh() {
    const filtered = filterEvents(allEvents);
    drawGraph(filtered);
    renderTable(filtered);
  }

  document.getElementById("sourceFilter").addEventListener("change", refresh);
  document.getElementById("refreshBtn").addEventListener("click", loadEvents);
  loadEvents();
  setInterval(loadEvents, 15000);

  // --- エクスポート(今表示中=絞り込み後のデータをそのまま出力する) ---
  function downloadBlob(content, filename, mimeType) {
    const blob = new Blob([content], { type: mimeType });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
  }

  function csvField(value) {
    const s = String(value);
    if (/[",\\n]/.test(s)) return '"' + s.replace(/"/g, '""') + '"';
    return s;
  }

  const EXPORT_HEADER = ["時刻", "メニュー", "増減", "検知元", "残数", "累計販売", "種別"];

  function exportRowsOf(events) {
    return events.map((e) => [
      e.timestamp, e.menu, e.delta, e.source, e.remaining_after, e.sold_after,
      e.estimated ? "推定" : "実測",
    ]);
  }

  document.getElementById("exportCsvBtn").addEventListener("click", () => {
    const rows = [EXPORT_HEADER, ...exportRowsOf(filterEvents(allEvents))];
    // ExcelでUTF-8のCSVを開いた時に文字化けしないよう、BOMを先頭に付ける。
    const csv = "﻿" + rows.map((r) => r.map(csvField).join(",")).join("\\n");
    downloadBlob(csv, "consumption_log.csv", "text/csv;charset=utf-8");
  });

  document.getElementById("exportTxtBtn").addEventListener("click", () => {
    const rows = [EXPORT_HEADER, ...exportRowsOf(filterEvents(allEvents))];
    const txt = rows.map((r) => r.join("\t")).join("\\n");
    downloadBlob(txt, "consumption_log.txt", "text/plain;charset=utf-8");
  });

  document.getElementById("exportPdfBtn").addEventListener("click", () => {
    window.print();
  });
</script>
</body>
</html>
"""


def build_consumption_graph_html() -> bytes:
    """カメラ/手動によるメニュー消費の推移をグラフ+一覧表で見せる専用ページ。

    `full_dashboard_node.py`から、メインの操作盤ダッシュボードとは別ページ
    (`/consumption_graph`)として配信される(2026-07-22、ユーザー要望:
    いつ・どのメニューが・何食減ったかをタイムスタンプ付きで記録し、
    HTMLからグラフとして見えるようにしたい)。
    """
    return _CONSUMPTION_GRAPH_HTML.encode("utf-8")


# 【2026-07-22追加】いつ・何と発話したかの履歴を一覧で見るための専用ページ
# (`/utterance_log`)。voice_bridge.pyが記録するSQLiteを読み取るだけ。
_UTTERANCE_LOG_HTML = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>のこさへん - 発話履歴</title>
<style>
  :root { color-scheme: light; font-family: system-ui, -apple-system, "Segoe UI", sans-serif; background:#f4f6f5; color:#16211d; }
  html, body { height:100%; margin:0; }
  header { box-sizing:border-box; height:44px; padding:0 18px; background:#124739; color:#fff; display:flex; justify-content:space-between; align-items:center; gap:8px; }
  header a { color:#cfe9df; text-decoration:none; font-size:13px; }
  main { max-width:900px; margin:0 auto; padding:16px; }
  .row { display:flex; gap:8px; flex-wrap:wrap; align-items:center; margin-bottom:12px; }
  button { padding:8px 12px; font-size:13px; border:0; border-radius:6px; background:#176b56; color:#fff; cursor:pointer; }
  input[type="text"] { padding:6px; font-size:13px; border:1px solid #cfd8d3; border-radius:6px; flex:1 1 auto; min-width:160px; }
  table { width:100%; border-collapse:collapse; font-size:13px; background:#fff; border:1px solid #dce4df; border-radius:8px; overflow:hidden; }
  th, td { border-bottom:1px solid #e4ebe7; padding:8px 10px; text-align:left; vertical-align:top; }
  th:first-child, td:first-child { white-space:nowrap; width:11em; }
  .hint { font-size:12px; color:#53635c; margin-top:6px; }
</style>
</head>
<body>
<header>
  <h1 style="margin:0;font-size:18px;">発話履歴</h1>
  <a href="/">&larr; 操作盤に戻る</a>
</header>
<main>
  <div class="row">
    <input type="text" id="searchBox" placeholder="発話内容で絞り込み(部分一致)" />
    <button id="refreshBtn">更新</button>
  </div>
  <div id="sourceNotice" class="hint" style="display:none;"></div>
  <table>
    <thead><tr><th>時刻</th><th>発話内容</th></tr></thead>
    <tbody id="rows"><tr><td colspan="2">読み込み中...</td></tr></tbody>
  </table>
  <p class="hint">新しいものが上に表示されます。一覧は15秒ごとに自動更新されます。</p>
</main>
<script>
  // 401(未ログイン/セッション切れ)を受け取ったら自動でログイン画面へ誘導する。
  (function () {
    const rawFetch = window.fetch.bind(window);
    window.fetch = async (...args) => {
      const res = await rawFetch(...args);
      if (res.status === 401 && location.pathname !== "/login") {
        location.href = "/login";
      }
      return res;
    };
  })();

  function escapeHtml(value) {
    return String(value).replace(/[&<>"']/g, (ch) => ({
      "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"
    }[ch]));
  }

  let allUtterances = [];

  function render() {
    const q = document.getElementById("searchBox").value.trim();
    const filtered = q
      ? allUtterances.filter((u) => u.text.includes(q))
      : allUtterances;
    const tbody = document.getElementById("rows");
    if (filtered.length === 0) {
      tbody.innerHTML = '<tr><td colspan="2">表示できるデータがありません</td></tr>';
      return;
    }
    const rows = filtered.slice().reverse().map(
      (u) => `<tr><td>${escapeHtml(u.timestamp)}</td><td>${escapeHtml(u.text)}</td></tr>`
    );
    tbody.innerHTML = rows.join("");
  }

  async function load() {
    try {
      const res = await fetch("/api/utterance_log");
      if (!res.ok) return;
      const data = await res.json();
      allUtterances = data.utterances || [];
      showSource(data);
      render();
    } catch (e) { /* 通信エラー時は前回表示のまま */ }
  }

  // 【2026-10-02追加】取得元の表示。リモート(voice_bridge)の取得に失敗してローカルDB
  // (古いデータの可能性)へフォールバックしている時は、目立つ警告を出す。リモート取得時は
  // 取得件数の上限(最新N件のみ)も知らせる。
  function showSource(data) {
    const el = document.getElementById("sourceNotice");
    if (data.source === "local-fallback") {
      el.style.cssText = "display:block;background:#faece7;color:#993c1d;padding:8px 10px;border-radius:6px;margin-bottom:10px;font-size:13px;font-weight:700;";
      el.textContent = "警告: voice_bridge(リモート)から取得できなかったため、この機体のローカルDBの内容(古い・不完全な可能性があります)を表示しています。 原因: " + (data.error || "不明");
    } else if (data.source === "remote") {
      el.style.cssText = "display:block;";
      el.textContent = "取得元: voice_bridge(リモート)。最新" + (data.remote_limit || "") + "件までを表示しています(それより古い履歴はここには表示されません)。";
    } else {
      el.style.display = "none";
    }
  }

  document.getElementById("searchBox").addEventListener("input", render);
  document.getElementById("refreshBtn").addEventListener("click", load);
  load();
  setInterval(load, 15000);
</script>
</body>
</html>
"""


def build_utterance_log_html() -> bytes:
    """発話履歴(いつ・何と発話したか)を一覧表示する専用ページ。

    `stage2_dashboard_node.py`/`full_dashboard_node.py`の両方から、メインの
    操作盤ダッシュボードとは別ページ(`/utterance_log`)として配信される。
    """
    return _UTTERANCE_LOG_HTML.encode("utf-8")


# 【2026-07-24追加】本番環境でしか収集できない人物検知の生ログ(いつ・どの
# くらいの距離で・カメラで確認できたか・同時に何クラスタ見えていたか)を
# 一覧+累計人数で見るための専用ページ(`/detection_log`)。明電舎でのPoC
# 現地検証当日にユーザー要望で追加。stage2/完成版どちらのダッシュボードからも使う。
_DETECTION_LOG_HTML = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>のこさへん - 人物検知ログ</title>
<style>
  :root { color-scheme: light; font-family: system-ui, -apple-system, "Segoe UI", sans-serif; background:#f4f6f5; color:#16211d; }
  html, body { height:100%; margin:0; }
  header { box-sizing:border-box; height:44px; padding:0 18px; background:#124739; color:#fff; display:flex; justify-content:space-between; align-items:center; gap:8px; }
  header a { color:#cfe9df; text-decoration:none; font-size:13px; }
  main { max-width:900px; margin:0 auto; padding:16px; display:flex; flex-direction:column; gap:16px; }
  section { background:#fff; border:1px solid #dce4df; border-radius:8px; padding:14px; }
  h2 { margin:0 0 10px; font-size:14px; }
  .row { display:flex; gap:8px; flex-wrap:wrap; align-items:center; }
  button { padding:8px 12px; font-size:13px; border:0; border-radius:6px; background:#176b56; color:#fff; cursor:pointer; }
  .stat-row { display:flex; gap:14px; flex-wrap:wrap; }
  .stat { background:#eef3f0; border-radius:8px; padding:10px 16px; min-width:140px; }
  .stat span { display:block; font-size:11px; color:#53635c; }
  .stat b { font-size:22px; }
  table { width:100%; border-collapse:collapse; font-size:12px; }
  th, td { border-bottom:1px solid #e4ebe7; padding:6px 8px; text-align:left; }
  .table-wrap { max-height:320px; overflow-y:auto; border:1px solid #dce4df; border-radius:8px; }
  .hint { font-size:12px; color:#53635c; }
</style>
</head>
<body>
<header>
  <h1 style="margin:0;font-size:18px;">人物検知ログ</h1>
  <a href="/">&larr; 操作盤に戻る</a>
</header>
<main>
  <section>
    <div class="row">
      <button id="refreshBtn">更新</button>
      <button id="exportCsvBtn">CSVで保存</button>
      <button id="exportTxtBtn">TXTで保存</button>
    </div>
    <div class="stat-row" style="margin-top:10px;">
      <div class="stat"><span>累計検知人数</span><b id="totalCount">-</b></div>
      <div class="stat"><span>うちカメラで確認できた回数</span><b id="confirmedCount">-</b></div>
    </div>
    <p class="hint">同一人物を追跡し続けている間は重複カウントしません(新しく検知した瞬間だけを1件とします)。「カメラで確認」は、その方向にカメラ映像でも人らしき物が写っていたか(方位の一致)を示すもので、その人がロボットの方を向いていたか(視線・顔向き)までは判定していません。</p>
  </section>
  <section>
    <h2>検知ごとの内訳</h2>
    <div class="table-wrap">
      <table>
        <thead><tr><th>時刻</th><th>距離(m)</th><th>カメラ確認</th><th>同時クラスタ数</th></tr></thead>
        <tbody id="rows"><tr><td colspan="4">読み込み中...</td></tr></tbody>
      </table>
    </div>
  </section>
</main>
<script>
  // 401(未ログイン/セッション切れ)を受け取ったら自動でログイン画面へ誘導する。
  (function () {
    const rawFetch = window.fetch.bind(window);
    window.fetch = async (...args) => {
      const res = await rawFetch(...args);
      if (res.status === 401 && location.pathname !== "/login") {
        location.href = "/login";
      }
      return res;
    };
  })();

  function escapeHtml(value) {
    return String(value).replace(/[&<>"']/g, (ch) => ({
      "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"
    }[ch]));
  }

  let detections = [];

  function render() {
    document.getElementById("totalCount").textContent = detections.length;
    document.getElementById("confirmedCount").textContent =
      detections.filter((d) => d.camera_confirmed).length;

    const tbody = document.getElementById("rows");
    if (detections.length === 0) {
      tbody.innerHTML = '<tr><td colspan="4">表示できるデータがありません</td></tr>';
      return;
    }
    const rows = detections.slice().reverse().map((d) => {
      const dist = d.distance_m === null || d.distance_m === undefined ? "-" : d.distance_m.toFixed(2);
      const cam = d.camera_confirmed ? "あり" : "なし";
      return `<tr><td>${escapeHtml(d.timestamp)}</td><td>${dist}</td><td>${cam}</td><td>${d.cluster_count}</td></tr>`;
    });
    tbody.innerHTML = rows.join("");
  }

  async function load() {
    try {
      const res = await fetch("/api/detection_log");
      if (!res.ok) return;
      const data = await res.json();
      detections = data.detections || [];
      render();
    } catch (e) { /* 通信エラー時は前回表示のまま */ }
  }

  function downloadBlob(content, filename, mimeType) {
    const blob = new Blob([content], { type: mimeType });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url; a.download = filename;
    document.body.appendChild(a); a.click(); a.remove();
    URL.revokeObjectURL(url);
  }

  function csvField(value) {
    const s = String(value);
    if (/[",\\n]/.test(s)) return '"' + s.replace(/"/g, '""') + '"';
    return s;
  }

  const EXPORT_HEADER = ["時刻", "距離(m)", "カメラ確認", "同時クラスタ数"];
  function exportRows() {
    return [EXPORT_HEADER, ...detections.map((d) => [
      d.timestamp, d.distance_m ?? "", d.camera_confirmed ? "あり" : "なし", d.cluster_count,
    ])];
  }

  document.getElementById("refreshBtn").addEventListener("click", load);
  document.getElementById("exportCsvBtn").addEventListener("click", () => {
    const csv = "﻿" + exportRows().map((r) => r.map(csvField).join(",")).join("\\n");
    downloadBlob(csv, "detection_log.csv", "text/csv;charset=utf-8");
  });
  document.getElementById("exportTxtBtn").addEventListener("click", () => {
    downloadBlob(exportRows().map((r) => r.join("\t")).join("\\n"), "detection_log.txt", "text/plain;charset=utf-8");
  });
  load();
  setInterval(load, 15000);
</script>
</body>
</html>
"""


def build_detection_log_html() -> bytes:
    """人物検知の生ログ(累計検知人数・カメラ確認の有無等)を一覧表示する専用ページ。

    `stage2_dashboard_node.py`/`full_dashboard_node.py`の両方から、メインの
    操作盤ダッシュボードとは別ページ(`/detection_log`)として配信される
    (2026-07-24追加)。
    """
    return _DETECTION_LOG_HTML.encode("utf-8")


# 【2026-09-24追加】走行距離のステップ応答テスト(scripts/step_response_test/
# step_response_test.py)が保存した過去の結果を一覧・比較表示する専用ページ
# (`/step_response`)。グラフ描画はCDNに頼らずCanvas 2Dへ自前描画する(ロボットが
# インターネットに繋がらない現場でも動く要件のため)。stage1/stage2/完成版の
# どのダッシュボードからも同じ結果ディレクトリを見に行くので共通で使う。
_STEP_RESPONSE_HTML = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>のこさへん - ステップ応答テスト</title>
<style>
  :root { color-scheme: light; font-family: system-ui, -apple-system, "Segoe UI", sans-serif; background:#f4f6f5; color:#16211d; }
  html, body { height:100%; margin:0; }
  header { box-sizing:border-box; height:44px; padding:0 18px; background:#124739; color:#fff; display:flex; justify-content:space-between; align-items:center; gap:8px; }
  header a { color:#cfe9df; text-decoration:none; font-size:13px; }
  main { max-width:920px; margin:0 auto; padding:16px; display:flex; flex-direction:column; gap:16px; }
  section { background:#fff; border:1px solid #dce4df; border-radius:8px; padding:14px; }
  h2 { margin:0 0 10px; font-size:14px; }
  .row { display:flex; gap:8px; flex-wrap:wrap; align-items:center; }
  button { padding:8px 12px; font-size:13px; border:0; border-radius:6px; background:#176b56; color:#fff; cursor:pointer; }
  button:disabled { background:#a9b6b0; cursor:default; }
  table { width:100%; border-collapse:collapse; font-size:12px; }
  th, td { border-bottom:1px solid #e4ebe7; padding:6px 8px; text-align:left; vertical-align:middle; }
  .table-wrap { max-height:280px; overflow-y:auto; border:1px solid #dce4df; border-radius:8px; }
  .hint { font-size:12px; color:#53635c; }
  .badge { padding:3px 10px; border-radius:999px; font-weight:700; font-size:12px; display:inline-block; }
  .badge-ok { background:#e1f5ee; color:#0f6e56; }
  .badge-ng { background:#faece7; color:#993c1d; }
  .swatch { display:inline-block; width:11px; height:11px; border-radius:3px; margin-right:5px; vertical-align:-1px; }
  canvas { width:100%; height:320px; display:block; background:#fbfcfb; border:1px solid #e4ebe7; border-radius:8px; }
  .metrics-table td:first-child, .metrics-table th:first-child { white-space:nowrap; width:11em; color:#53635c; }
  .empty { padding:16px; text-align:center; color:#7c8a83; font-size:13px; }
  input[type="number"], select { padding:6px 8px; font-size:13px; border:1px solid #cfd8d3; border-radius:6px; }
  label { font-size:13px; display:flex; flex-direction:column; gap:3px; }
  .estop-btn { background:#a32d2d; }
  .estop-btn.engaged { background:#5c1717; }
  .run-btn { background:#176b56; }
  .run-status { font-size:13px; font-weight:700; }
</style>
</head>
<body>
<header>
  <h1 style="margin:0;font-size:18px;">ステップ応答テスト</h1>
  <a href="/">&larr; 操作盤に戻る</a>
</header>
<main>
  <section>
    <h2>テスト実行</h2>
    <p class="hint">必ず周囲に人・障害物が無いこと、いつでも止められる状態を確認してから実行してください。実行中は自律行動(見回し・接近等)を一時停止します。</p>
    <div class="row">
      <label>方向
        <select id="runDirection">
          <option value="forward">前進</option>
          <option value="backward">後退</option>
          <option value="rotate_left">その場回転(左)</option>
          <option value="rotate_right">その場回転(右)</option>
        </select>
      </label>
      <label id="runTargetDistLabel">目標距離(m、最大3.0)
        <input id="runTarget" type="number" step="0.1" min="0.1" max="3.0" value="1.0" style="width:90px" />
      </label>
      <label id="runTargetAngleLabel" style="display:none;">目標回転角度(度、最大360.0)
        <input id="runTargetDeg" type="number" step="10" min="1" max="360" value="90" style="width:90px" />
      </label>
      <label>記録時間(s)
        <input id="runDuration" type="number" step="0.5" min="2" max="30" value="8" style="width:90px" />
      </label>
      <button id="runBtn" class="run-btn">実行</button>
    </div>
    <p class="hint run-status" id="runStatus">&nbsp;</p>
    <button id="estopBtn" class="estop-btn" style="margin-top:6px;">緊急停止(ソフト)</button>
    <p class="hint" id="estopHint">&nbsp;</p>
  </section>
  <section>
    <h2>過去の結果</h2>
    <p class="hint">最大2件までチェックして比較できます。CLI(scripts/step_response_test/step_response_test.py)を実行すると一覧に追加されます。</p>
    <div class="row">
      <button id="refreshBtn">更新</button>
    </div>
    <div class="table-wrap" style="margin-top:10px;">
      <table>
        <thead><tr><th></th><th>実行日時</th><th>種別</th><th>目標</th><th>オーバーシュート</th><th>判定</th></tr></thead>
        <tbody id="rows"><tr><td colspan="6">読み込み中...</td></tr></tbody>
      </table>
    </div>
  </section>
  <section>
    <h2>グラフ</h2>
    <div id="chartEmpty" class="empty">一覧から結果を選んでください</div>
    <canvas id="chart" width="1000" height="320" style="display:none;"></canvas>
    <div id="legend" class="row" style="margin-top:8px;"></div>
  </section>
  <section>
    <h2>指標</h2>
    <div class="table-wrap">
      <table class="metrics-table">
        <tbody id="metricsRows"><tr><td colspan="3" class="empty">一覧から結果を選んでください</td></tr></tbody>
      </table>
    </div>
  </section>
</main>
<script>
  // 401(未ログイン/セッション切れ)を受け取ったら自動でログイン画面へ誘導する。
  (function () {
    const rawFetch = window.fetch.bind(window);
    window.fetch = async (...args) => {
      const res = await rawFetch(...args);
      if (res.status === 401 && location.pathname !== "/login") {
        location.href = "/login";
      }
      return res;
    };
  })();

  function escapeHtml(value) {
    return String(value).replace(/[&<>"']/g, (ch) => ({
      "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"
    }[ch]));
  }

  const SERIES_COLORS = ["#176b56", "#c0392b"];
  let allResults = [];
  const selectedIds = [];       // 最大2件
  const resultCache = {};       // id -> 詳細JSON(times_s/distances_m/metrics等)

  function fmtNum(v, digits, unit) {
    return (v === null || v === undefined) ? "N/A" : v.toFixed(digits) + (unit || "");
  }

  function isRotateDirection(dir) {
    return dir === "rotate_left" || dir === "rotate_right";
  }

  function directionLabel(dir) {
    switch (dir) {
      case "backward": return "実機(後退)";
      case "rotate_left": return "実機(左回転)";
      case "rotate_right": return "実機(右回転)";
      case "forward": default: return "実機(前進)";
    }
  }

  function renderRows() {
    const tbody = document.getElementById("rows");
    if (allResults.length === 0) {
      tbody.innerHTML = '<tr><td colspan="6">表示できる結果がありません(まだテストを実行していません)</td></tr>';
      return;
    }
    const rows = allResults.map((r) => {
      const checked = selectedIds.includes(r.id) ? "checked" : "";
      const kind = r.sim ? "sim" : directionLabel(r.direction);
      const badgeClass = r.judgement === "OK" ? "badge-ok" : "badge-ng";
      const os = (r.overshoot_pct === null || r.overshoot_pct === undefined) ? "-" : r.overshoot_pct.toFixed(1) + "%";
      const targetUnit = isRotateDirection(r.direction) ? "°" : " m";
      const target = (r.target_m === null || r.target_m === undefined) ? "-" : r.target_m.toFixed(2) + targetUnit;
      return `<tr>
        <td><input type="checkbox" data-id="${escapeHtml(r.id)}" ${checked} /></td>
        <td>${escapeHtml(r.executed_at || "-")}</td>
        <td>${escapeHtml(kind)}</td>
        <td>${target}</td>
        <td>${os}</td>
        <td><span class="badge ${badgeClass}">${escapeHtml(r.judgement || "-")}</span></td>
      </tr>`;
    });
    tbody.innerHTML = rows.join("");
    tbody.querySelectorAll("input[type=checkbox]").forEach((el) => {
      el.addEventListener("change", onCheckboxChange);
    });
  }

  async function onCheckboxChange(ev) {
    const id = ev.target.dataset.id;
    if (ev.target.checked) {
      if (selectedIds.length >= 2) {
        // 3件目は最も古い選択を外して2件までに保つ。
        const removed = selectedIds.shift();
      }
      selectedIds.push(id);
      await ensureResultLoaded(id);
    } else {
      const idx = selectedIds.indexOf(id);
      if (idx >= 0) selectedIds.splice(idx, 1);
    }
    renderRows();
    renderChartAndMetrics();
  }

  async function ensureResultLoaded(id) {
    if (resultCache[id]) return resultCache[id];
    try {
      const res = await fetch("/api/step_response/result?id=" + encodeURIComponent(id));
      if (!res.ok) return null;
      const data = await res.json();
      resultCache[id] = data;
      return data;
    } catch (e) {
      return null;
    }
  }

  function renderChartAndMetrics() {
    const canvas = document.getElementById("chart");
    const empty = document.getElementById("chartEmpty");
    const legend = document.getElementById("legend");
    const metricsBody = document.getElementById("metricsRows");
    const series = selectedIds.map((id) => resultCache[id]).filter((d) => d);

    if (series.length === 0) {
      canvas.style.display = "none";
      empty.style.display = "";
      legend.innerHTML = "";
      metricsBody.innerHTML = '<tr><td colspan="3" class="empty">一覧から結果を選んでください</td></tr>';
      return;
    }
    canvas.style.display = "";
    empty.style.display = "none";

    legend.innerHTML = series.map((s, i) => {
      const label = (s.run_name || s.executed_at || "").replace(/_/g, " ");
      return `<span><span class="swatch" style="background:${SERIES_COLORS[i]}"></span>${escapeHtml(label)}</span>`;
    }).join("");

    drawChart(canvas, series);
    renderMetricsTable(metricsBody, series);
  }

  function renderMetricsTable(tbody, series) {
    // 【単位の決め方】result.jsonのtarget_m/d_max等は歴史的なフィールド名の
    // ままrotate_left/rotate_rightでも角度[度]を入れているため、単位・行名は
    // 各resultのparams.directionから判定する。複数選択(比較)時に距離系と
    // 回転系を混在させると単位表記が最初に選んだ結果基準になる点に注意
    // (元々このページは比較対象の種別が揃っている前提で使うツールのため、
    // 混在ケースの表示を作り込むのはスコープ外とする)。
    const rotate = series.length > 0 && isRotateDirection((series[0].params || {}).direction);
    const unit = rotate ? " 度" : " m";
    const targetLabel = rotate ? "目標回転角度" : "目標距離";
    const maxLabel = rotate ? "最大到達角度" : "最大到達距離";
    const rowsDef = [
      ["判定", (m) => `<span class="badge ${m.judgement === "OK" ? "badge-ok" : "badge-ng"}">${escapeHtml(m.judgement)}</span>`],
      [targetLabel, (m) => fmtNum(m.target_m, 3, unit)],
      [maxLabel, (m) => fmtNum(m.d_max, 3, unit)],
      ["オーバーシュート", (m) => fmtNum(m.overshoot_m, 3, unit) + " (" + fmtNum(m.overshoot_pct, 2, "%") + ")"],
      ["ピーク時刻", (m) => fmtNum(m.peak_time_s, 3, " s")],
      ["立ち上がり時間(10-90%)", (m) => fmtNum(m.rise_time_s, 3, " s")],
      ["整定時間", (m) => fmtNum(m.settling_time_s, 3, " s")],
      ["定常偏差", (m) => fmtNum(m.steady_state_error_m, 3, unit)],
    ];
    const header = "<tr><th>指標</th>" + series.map((_, i) => `<th>結果${i + 1}</th>`).join("") + "</tr>";
    const rows = rowsDef.map(([label, fn]) => {
      const cells = series.map((s) => `<td>${fn(s.metrics)}</td>`).join("");
      return `<tr><td>${label}</td>${cells}</tr>`;
    });
    tbody.innerHTML = header + rows.join("");
  }

  function drawChart(canvas, series) {
    const dpr = window.devicePixelRatio || 1;
    const cssWidth = canvas.clientWidth || 1000;
    const cssHeight = 320;
    canvas.width = cssWidth * dpr;
    canvas.height = cssHeight * dpr;
    const ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, cssWidth, cssHeight);

    const padL = 52, padR = 16, padT = 14, padB = 32;
    const plotW = cssWidth - padL - padR;
    const plotH = cssHeight - padT - padB;

    let maxT = 0, maxD = 0, minD = 0;
    series.forEach((s) => {
      const times = s.times_s || [];
      const dists = s.distances_m || [];
      maxT = Math.max(maxT, times.length ? times[times.length - 1] : 0);
      maxD = Math.max(maxD, s.metrics.d_max || 0, s.metrics.target_m || 0);
      minD = Math.min(minD, ...dists, 0);
    });
    maxT = maxT || 1;
    maxD = (maxD || 1) * 1.12;

    const xToPx = (t) => padL + (t / maxT) * plotW;
    const yToPx = (d) => padT + plotH - ((d - minD) / (maxD - minD)) * plotH;

    ctx.strokeStyle = "#e4ebe7";
    ctx.fillStyle = "#53635c";
    ctx.font = "11px system-ui, sans-serif";
    ctx.lineWidth = 1;
    const yTicks = 5;
    for (let i = 0; i <= yTicks; i++) {
      const d = minD + (maxD - minD) * (i / yTicks);
      const y = yToPx(d);
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(padL + plotW, y); ctx.stroke();
      ctx.textAlign = "right"; ctx.textBaseline = "middle";
      ctx.fillText(d.toFixed(2), padL - 8, y);
    }
    const xTicks = 6;
    for (let i = 0; i <= xTicks; i++) {
      const t = maxT * (i / xTicks);
      const x = xToPx(t);
      ctx.textAlign = "center"; ctx.textBaseline = "top";
      ctx.fillText(t.toFixed(1) + "s", x, padT + plotH + 6);
    }

    // 許容帯(1件表示時のみ、複数だと帯が重なって読みにくいため)。
    if (series.length === 1) {
      const m = series[0].metrics;
      const tolAbs = m.target_m * (m.tol_pct / 100.0);
      if (tolAbs > 0) {
        ctx.fillStyle = "rgba(23,107,86,0.10)";
        const yTop = yToPx(m.target_m + tolAbs), yBot = yToPx(m.target_m - tolAbs);
        ctx.fillRect(padL, yTop, plotW, yBot - yTop);
      }
    }

    series.forEach((s, i) => {
      const color = SERIES_COLORS[i % SERIES_COLORS.length];
      const target = s.metrics.target_m;
      // 目標距離の水平線
      ctx.strokeStyle = color;
      ctx.setLineDash([5, 4]);
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(padL, yToPx(target));
      ctx.lineTo(padL + plotW, yToPx(target));
      ctx.stroke();
      ctx.setLineDash([]);

      // d(t)の実線
      const times = s.times_s || [];
      const dists = s.distances_m || [];
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();
      times.forEach((t, idx) => {
        const x = xToPx(t), y = yToPx(dists[idx]);
        if (idx === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      });
      ctx.stroke();

      // ピーク点
      if (s.metrics.peak_time_s !== null && s.metrics.peak_time_s !== undefined) {
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.arc(xToPx(s.metrics.peak_time_s), yToPx(s.metrics.d_max), 4, 0, Math.PI * 2);
        ctx.fill();
      }
    });

    // 軸
    ctx.strokeStyle = "#16211d";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(padL, padT); ctx.lineTo(padL, padT + plotH); ctx.lineTo(padL + plotW, padT + plotH);
    ctx.stroke();
  }

  async function load() {
    try {
      const res = await fetch("/api/step_response/list");
      if (!res.ok) return;
      const data = await res.json();
      allResults = data.results || [];
      renderRows();
    } catch (e) { /* 通信エラー時は前回表示のまま */ }
  }

  document.getElementById("refreshBtn").addEventListener("click", load);
  window.addEventListener("resize", () => { if (selectedIds.length) renderChartAndMetrics(); });
  load();

  // --- ソフト緊急停止(このページ単独でも押せるようにする、実行ボタンの安全要件) ---
  const estopBtn = document.getElementById("estopBtn");
  const estopHint = document.getElementById("estopHint");
  let estopEngaged = false;
  let estopPending = false;
  let estopIgnoreSyncUntil = 0;
  function renderEstop(engaged) {
    estopEngaged = engaged;
    estopBtn.textContent = engaged ? "緊急停止を解除" : "緊急停止(ソフト)";
    estopBtn.classList.toggle("engaged", engaged);
  }
  // 【2026-10-02修正】表示を/api/stateのsoft_estop_engaged(サーバー側の実際の状態)に同期させる。
  // 以前は押した瞬間に表示だけ反転し、res.okを見ていなかったため、403/409/500でも
  // 「緊急停止中です」と表示され、実際には止まっていないことがあった。
  async function syncEstop() {
    if (estopPending || Date.now() < estopIgnoreSyncUntil) return;
    try {
      const res = await fetch("/api/state");
      if (!res.ok) return;
      const s = await res.json();
      if (estopPending || Date.now() < estopIgnoreSyncUntil) return;
      if (typeof s.soft_estop_engaged === "boolean") renderEstop(s.soft_estop_engaged);
    } catch (e) { /* 次のポーリングで再試行 */ }
  }
  setInterval(syncEstop, 1000);
  syncEstop();
  estopBtn.addEventListener("click", async () => {
    if (estopPending) return; // 二重送信防止
    const prev = estopEngaged;
    const engaged = !prev;
    if (!engaged && !confirm("本当に解除しますか？")) return;
    if (estopPending) return; // confirm中に別経路で開始された場合
    estopPending = true;
    estopBtn.disabled = true;
    renderEstop(engaged);
    let ok = false;
    let failMsg = "";
    try {
      const res = await fetch("/api/soft_estop", {
        method: "POST", headers: {"Content-Type":"application/json"},
        body: JSON.stringify({ engaged }),
      });
      ok = res.ok;
      if (!ok) {
        failMsg = `サーバーが拒否しました(${res.status})`;
        try { const d = await res.json(); if (d && d.message) failMsg = d.message; } catch (e2) { /* 本文がJSONでなくても既定メッセージのまま続行 */ }
      }
    } catch (e) {
      failMsg = "通信エラー: サーバーに接続できません";
    } finally {
      estopPending = false;
      estopBtn.disabled = false;
      estopIgnoreSyncUntil = Date.now() + 1500;
    }
    if (ok) {
      estopHint.textContent = engaged ? "緊急停止中です。解除するまで実行できません。" : "解除しました。";
      estopHint.style.color = "";
      return;
    }
    renderEstop(prev);
    estopHint.style.color = "#a32d2d";
    estopHint.textContent = engaged
      ? `緊急停止の送信に失敗しました(${failMsg})。機体は止まっていない可能性があります。物理の非常停止を使ってください。`
      : `緊急停止の解除に失敗しました(${failMsg})。緊急停止中のままです。`;
    if (engaged) alert("緊急停止の送信に失敗しました。機体は止まっていない可能性があります。物理の非常停止を使ってください。");
  });

  // --- テスト実行(確認ダイアログ→開始→完了until自動ポーリング) ---
  const runBtn = document.getElementById("runBtn");
  const runStatus = document.getElementById("runStatus");
  const runDirectionSel = document.getElementById("runDirection");
  const runTargetDistLabel = document.getElementById("runTargetDistLabel");
  const runTargetAngleLabel = document.getElementById("runTargetAngleLabel");
  const MAX_TARGET_M = 3.0;
  const MAX_TARGET_DEG = 360.0;
  let polling = false;

  // 距離入力/角度入力の表示切り替え(stage1_dashboard_nodeのtestModeSel
  // 切り替えと同じUXパターン)。
  runDirectionSel.addEventListener("change", () => {
    const rotate = isRotateDirection(runDirectionSel.value);
    runTargetDistLabel.style.display = rotate ? "none" : "";
    runTargetAngleLabel.style.display = rotate ? "" : "none";
  });

  async function pollRunStatus() {
    if (polling) return;
    polling = true;
    try {
      while (true) {
        const res = await fetch("/api/step_response/run_status");
        if (!res.ok) break;
        const s = await res.json();
        if (!s.step_response_test_running) {
          if (s.step_response_error) {
            runStatus.textContent = "エラー: " + s.step_response_error;
          } else if (s.step_response_last_result_id) {
            runStatus.textContent = "完了しました。結果を表示します。";
            await load();
            if (!selectedIds.includes(s.step_response_last_result_id)) {
              if (selectedIds.length >= 2) selectedIds.shift();
              selectedIds.push(s.step_response_last_result_id);
            }
            await ensureResultLoaded(s.step_response_last_result_id);
            renderRows();
            renderChartAndMetrics();
          }
          break;
        }
        runStatus.textContent = "実行中...";
        await new Promise((r) => setTimeout(r, 700));
      }
    } catch (e) {
      runStatus.textContent = "通信エラー: 実行状況を確認できません";
    } finally {
      polling = false;
      runBtn.disabled = false;
    }
  }

  runBtn.addEventListener("click", async () => {
    const direction = runDirectionSel.value;
    const rotate = isRotateDirection(direction);
    const duration = Number(document.getElementById("runDuration").value);
    const target = rotate
      ? Number(document.getElementById("runTargetDeg").value)
      : Number(document.getElementById("runTarget").value);
    const maxTarget = rotate ? MAX_TARGET_DEG : MAX_TARGET_M;
    const unitLabel = rotate ? "度" : "m";
    if (!(target > 0) || target > maxTarget) {
      runStatus.textContent = `目標${rotate ? "回転角度" : "距離"}は0より大きく${maxTarget}${unitLabel}以内にしてください。`;
      return;
    }
    if (estopEngaged) {
      runStatus.textContent = "緊急停止中は実行できません。解除してから実行してください。";
      return;
    }
    const dirLabel = { forward: "前進", backward: "後退", rotate_left: "左回転", rotate_right: "右回転" }[direction] || direction;
    const actionText = rotate ? `${target}度回転する` : `${target}m走行する`;
    const ok = window.confirm(
      `${dirLabel}方向へ${actionText}テストを開始します。\n` +
      `周囲に人・障害物が無いこと、いつでも緊急停止できることを確認しましたか？`
    );
    if (!ok) return;

    runBtn.disabled = true;
    runStatus.textContent = "開始しています...";
    const payload = rotate
      ? { direction, target_deg: target, duration_s: duration }
      : { direction, target_m: target, duration_s: duration };
    try {
      const res = await fetch("/api/step_response/run", {
        method: "POST", headers: {"Content-Type":"application/json"},
        body: JSON.stringify(payload),
      });
      if (!res.ok) {
        let msg = `開始できませんでした(${res.status})`;
        try { const d = await res.json(); if (d && d.message) msg = d.message; } catch (e2) { /* 本文がJSONでなくても既定メッセージのまま続行 */ }
        runStatus.textContent = msg;
        runBtn.disabled = false;
        return;
      }
      pollRunStatus();
    } catch (e) {
      runStatus.textContent = "通信エラー: サーバーに接続できません";
      runBtn.disabled = false;
    }
  });
</script>
</body>
</html>
"""


def build_step_response_html() -> bytes:
    """走行距離ステップ応答テストの過去結果を一覧・比較表示する専用ページ。

    `stage1_dashboard_node.py`/`stage2_dashboard_node.py`/`full_dashboard_node.py`
    のいずれからも、メインの操作盤ダッシュボードとは別ページ(`/step_response`)
    として配信される(2026-09-24追加)。
    """
    return _STEP_RESPONSE_HTML.encode("utf-8")


# 【2026-07-22追加】推奨(接客)がどれだけ売上に繋がったかを見るための専用
# ページ(`/recommendation_effectiveness`、完成版ダッシュボードのみ)。
# engagement_log(接客履歴+A/Bモード)とconsumption_log(消費履歴)を突き
# 合わせて、反応率・A/Bモード比較を計算する。
_RECOMMENDATION_EFFECTIVENESS_HTML = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>のこさへん - 推奨効果分析</title>
<style>
  :root { color-scheme: light; font-family: system-ui, -apple-system, "Segoe UI", sans-serif; background:#f4f6f5; color:#16211d; }
  html, body { height:100%; margin:0; }
  header { box-sizing:border-box; height:44px; padding:0 18px; background:#124739; color:#fff; display:flex; justify-content:space-between; align-items:center; gap:8px; }
  header a { color:#cfe9df; text-decoration:none; font-size:13px; }
  main { max-width:960px; margin:0 auto; padding:16px; display:flex; flex-direction:column; gap:16px; }
  section { background:#fff; border:1px solid #dce4df; border-radius:8px; padding:14px; }
  h2 { margin:0 0 10px; font-size:14px; }
  .row { display:flex; gap:8px; flex-wrap:wrap; align-items:center; }
  button { padding:8px 12px; font-size:13px; border:0; border-radius:6px; background:#176b56; color:#fff; cursor:pointer; }
  select { padding:6px; font-size:13px; border:1px solid #cfd8d3; border-radius:6px; }
  .stat-row { display:flex; gap:14px; flex-wrap:wrap; }
  .stat { background:#eef3f0; border-radius:8px; padding:10px 16px; min-width:140px; }
  .stat span { display:block; font-size:11px; color:#53635c; }
  .stat b { font-size:22px; }
  table { width:100%; border-collapse:collapse; font-size:12px; }
  th, td { border-bottom:1px solid #e4ebe7; padding:6px 8px; text-align:left; }
  .table-wrap { max-height:260px; overflow-y:auto; border:1px solid #dce4df; border-radius:8px; }
  .yes { color:#0f6e56; font-weight:700; }
  .no { color:#993c1d; }
  .hint { font-size:12px; color:#53635c; }
</style>
</head>
<body>
<header>
  <h1 style="margin:0;font-size:18px;">推奨効果分析</h1>
  <a href="/">&larr; 操作盤に戻る</a>
</header>
<main>
  <section>
    <h2>反応率(推奨した接客のうち、実際に購入に繋がった割合)</h2>
    <div class="row" style="margin-bottom:10px;">
      <label style="font-size:13px;">反応とみなす時間 <select id="windowMinutes">
        <option value="3">3分以内</option>
        <option value="5" selected>5分以内</option>
        <option value="10">10分以内</option>
        <option value="15">15分以内</option>
      </select></label>
      <button id="refreshBtn">更新</button>
    </div>
    <div class="stat-row">
      <div class="stat"><span>総接客回数</span><b id="totalEngagements">-</b></div>
      <div class="stat"><span>うち推奨あり</span><b id="recommendedEngagements">-</b></div>
      <div class="stat"><span>購入に繋がった回数</span><b id="respondedCount">-</b></div>
      <div class="stat"><span>反応率</span><b id="responseRate">-</b></div>
    </div>
  </section>

  <section>
    <h2>A/Bテスト比較(推奨ON/OFF時間帯ごとの販売ペース)</h2>
    <div class="stat-row">
      <div class="stat"><span>推奨ON中(件/時)</span><b id="rateOn">-</b></div>
      <div class="stat"><span>推奨OFF中(件/時)</span><b id="rateOff">-</b></div>
    </div>
    <p class="hint">ダッシュボードのA/Bトグルで推奨発話をON/OFFした時間帯ごとに、実際の販売数(全メニュー合計)を時間あたりに正規化して比較します。データが少ないうちは参考値程度に見てください。</p>
  </section>

  <section>
    <h2>接客ごとの内訳</h2>
    <div class="table-wrap">
      <table>
        <thead><tr><th>時刻</th><th>推奨メニュー</th><th>反応</th></tr></thead>
        <tbody id="engagementRows"><tr><td colspan="3">読み込み中...</td></tr></tbody>
      </table>
    </div>
  </section>
</main>
<script>
  // 401(未ログイン/セッション切れ)を受け取ったら自動でログイン画面へ誘導する。
  (function () {
    const rawFetch = window.fetch.bind(window);
    window.fetch = async (...args) => {
      const res = await rawFetch(...args);
      if (res.status === 401 && location.pathname !== "/login") {
        location.href = "/login";
      }
      return res;
    };
  })();

  function escapeHtml(value) {
    return String(value).replace(/[&<>"']/g, (ch) => ({
      "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"
    }[ch]));
  }

  let engagements = [];
  let modeChanges = [];
  let consumptionEvents = [];

  function respondedWithin(engagement, windowMs) {
    if (!engagement.promoted_menu) return false;
    const start = new Date(engagement.timestamp).getTime();
    const end = start + windowMs;
    return consumptionEvents.some((e) => {
      if (e.menu !== engagement.promoted_menu || e.delta <= 0) return false;
      const t = new Date(e.timestamp).getTime();
      return t >= start && t <= end;
    });
  }

  function computeResponseRate() {
    const windowMs = Number(document.getElementById("windowMinutes").value) * 60000;
    const total = engagements.length;
    const recommended = engagements.filter((e) => e.promoted_menu);
    const responded = recommended.filter((e) => respondedWithin(e, windowMs));

    document.getElementById("totalEngagements").textContent = total;
    document.getElementById("recommendedEngagements").textContent = recommended.length;
    document.getElementById("respondedCount").textContent = responded.length;
    document.getElementById("responseRate").textContent = recommended.length
      ? Math.round((responded.length / recommended.length) * 100) + "%"
      : "-";

    const tbody = document.getElementById("engagementRows");
    if (engagements.length === 0) {
      tbody.innerHTML = '<tr><td colspan="3">表示できるデータがありません</td></tr>';
      return;
    }
    const rows = engagements.slice().reverse().map((e) => {
      const menu = e.promoted_menu ? escapeHtml(e.promoted_menu) : "(推奨なし)";
      let reaction = "-";
      if (e.promoted_menu) {
        reaction = respondedWithin(e, windowMs)
          ? '<span class="yes">あり</span>'
          : '<span class="no">なし</span>';
      }
      return `<tr><td>${escapeHtml(e.timestamp)}</td><td>${menu}</td><td>${reaction}</td></tr>`;
    });
    tbody.innerHTML = rows.join("");
  }

  function computeAbComparison() {
    // mode_changesを時系列に並べ、各期間(enabled=true/false)の長さと、
    // その期間内に発生したconsumption_events(delta>0)の件数を集計する。
    if (modeChanges.length === 0) {
      document.getElementById("rateOn").textContent = "-";
      document.getElementById("rateOff").textContent = "-";
      return;
    }
    const sorted = modeChanges.slice().sort(
      (a, b) => new Date(a.timestamp) - new Date(b.timestamp)
    );
    const now = Date.now();
    let onMs = 0, offMs = 0, onCount = 0, offCount = 0;
    for (let i = 0; i < sorted.length; i++) {
      const start = new Date(sorted[i].timestamp).getTime();
      const end = i + 1 < sorted.length ? new Date(sorted[i + 1].timestamp).getTime() : now;
      const duration = Math.max(0, end - start);
      const count = consumptionEvents.filter((e) => {
        if (e.delta <= 0) return false;
        const t = new Date(e.timestamp).getTime();
        return t >= start && t < end;
      }).length;
      if (sorted[i].enabled) { onMs += duration; onCount += count; }
      else { offMs += duration; offCount += count; }
    }
    const perHour = (count, ms) => (ms > 0 ? (count / (ms / 3600000)).toFixed(2) : "-");
    document.getElementById("rateOn").textContent = perHour(onCount, onMs);
    document.getElementById("rateOff").textContent = perHour(offCount, offMs);
  }

  async function load() {
    try {
      const [engRes, consRes] = await Promise.all([
        fetch("/api/engagement_log"),
        fetch("/api/consumption_log"),
      ]);
      if (engRes.ok) {
        const d = await engRes.json();
        engagements = d.engagements || [];
        modeChanges = d.mode_changes || [];
      }
      if (consRes.ok) {
        const d = await consRes.json();
        consumptionEvents = d.events || [];
      }
      computeResponseRate();
      computeAbComparison();
    } catch (e) { /* 通信エラー時は前回表示のまま */ }
  }

  document.getElementById("windowMinutes").addEventListener("change", computeResponseRate);
  document.getElementById("refreshBtn").addEventListener("click", load);
  load();
  setInterval(load, 15000);
</script>
</body>
</html>
"""


def build_recommendation_effectiveness_html() -> bytes:
    """推奨(接客)がどれだけ購入に繋がったかを見る専用ページ。

    `full_dashboard_node.py`から、メインの操作盤ダッシュボードとは別ページ
    (`/recommendation_effectiveness`)として配信される(2026-07-22追加)。
    """
    return _RECOMMENDATION_EFFECTIVENESS_HTML.encode("utf-8")


# 【2026-07-22追加】発表資料にそのまま使える集計を出す日報/週報ページ
# (`/activity_report`、完成版ダッシュボードのみ)。consumption_log/
# engagement_logの生データを期間で絞り込み、集計してCSV/TXTで出力できる。
_ACTIVITY_REPORT_HTML = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>のこさへん - 日報/週報</title>
<style>
  :root { color-scheme: light; font-family: system-ui, -apple-system, "Segoe UI", sans-serif; background:#f4f6f5; color:#16211d; }
  html, body { height:100%; margin:0; }
  header { box-sizing:border-box; height:44px; padding:0 18px; background:#124739; color:#fff; display:flex; justify-content:space-between; align-items:center; gap:8px; }
  header a { color:#cfe9df; text-decoration:none; font-size:13px; }
  main { max-width:960px; margin:0 auto; padding:16px; display:flex; flex-direction:column; gap:16px; }
  section { background:#fff; border:1px solid #dce4df; border-radius:8px; padding:14px; }
  h2 { margin:0 0 10px; font-size:14px; }
  .row { display:flex; gap:8px; flex-wrap:wrap; align-items:center; }
  button { padding:8px 12px; font-size:13px; border:0; border-radius:6px; background:#176b56; color:#fff; cursor:pointer; }
  select { padding:6px; font-size:13px; border:1px solid #cfd8d3; border-radius:6px; }
  .stat-row { display:flex; gap:14px; flex-wrap:wrap; }
  .stat { background:#eef3f0; border-radius:8px; padding:10px 16px; min-width:140px; }
  .stat span { display:block; font-size:11px; color:#53635c; }
  .stat b { font-size:22px; }
  table { width:100%; border-collapse:collapse; font-size:12px; }
  th, td { border-bottom:1px solid #e4ebe7; padding:6px 8px; text-align:left; }
  canvas { max-width:100%; background:#fff; border:1px solid #dce4df; border-radius:8px; }
</style>
</head>
<body>
<header>
  <h1 style="margin:0;font-size:18px;">日報/週報</h1>
  <a href="/">&larr; 操作盤に戻る</a>
</header>
<main>
  <section>
    <div class="row">
      <label style="font-size:13px;">集計期間 <select id="periodSelect">
        <option value="1">当日</option>
        <option value="7" selected>直近7日</option>
        <option value="30">直近30日</option>
      </select></label>
      <button id="refreshBtn">更新</button>
      <button id="exportBtn">CSVで保存</button>
      <button id="exportTxtBtn">TXTで保存</button>
    </div>
  </section>

  <section>
    <h2>サマリー</h2>
    <div class="stat-row">
      <div class="stat"><span>総販売数</span><b id="totalSold">-</b></div>
      <div class="stat"><span>カメラ検知</span><b id="cameraCount">-</b></div>
      <div class="stat"><span>手動入力</span><b id="manualCount">-</b></div>
      <div class="stat"><span>総接客回数</span><b id="totalEngagements">-</b></div>
      <div class="stat"><span>反応率(5分以内)</span><b id="responseRate">-</b></div>
    </div>
  </section>

  <section>
    <h2>メニュー別販売数</h2>
    <table>
      <thead><tr><th>メニュー</th><th>販売数</th></tr></thead>
      <tbody id="menuRows"><tr><td colspan="2">読み込み中...</td></tr></tbody>
    </table>
  </section>

  <section>
    <h2>時間帯別の販売分布(0-23時)</h2>
    <canvas id="hourCanvas" width="880" height="220"></canvas>
  </section>
</main>
<script>
  // 401(未ログイン/セッション切れ)を受け取ったら自動でログイン画面へ誘導する。
  (function () {
    const rawFetch = window.fetch.bind(window);
    window.fetch = async (...args) => {
      const res = await rawFetch(...args);
      if (res.status === 401 && location.pathname !== "/login") {
        location.href = "/login";
      }
      return res;
    };
  })();

  let summary = null;

  function sinceIso() {
    const days = Number(document.getElementById("periodSelect").value);
    const d = new Date(Date.now() - days * 86400000);
    return d.toISOString().slice(0, 19);
  }

  function isCameraSource(source) {
    return typeof source === "string" && source.startsWith("checkout_cam_");
  }

  function computeSummary(events, engagements) {
    const menuTotals = {};
    const hourTotals = new Array(24).fill(0);
    let totalSold = 0, cameraCount = 0, manualCount = 0;
    events.forEach((e) => {
      if (e.delta <= 0) return;
      menuTotals[e.menu] = (menuTotals[e.menu] || 0) + e.delta;
      totalSold += e.delta;
      const hour = new Date(e.timestamp).getHours();
      hourTotals[hour] += e.delta;
      if (isCameraSource(e.source)) cameraCount += e.delta; else manualCount += e.delta;
    });
    const windowMs = 5 * 60000;
    const recommended = engagements.filter((en) => en.promoted_menu);
    const responded = recommended.filter((en) => {
      const start = new Date(en.timestamp).getTime();
      const end = start + windowMs;
      return events.some((e) => e.menu === en.promoted_menu && e.delta > 0 &&
        new Date(e.timestamp).getTime() >= start && new Date(e.timestamp).getTime() <= end);
    });
    return {
      totalSold, cameraCount, manualCount, menuTotals, hourTotals,
      totalEngagements: engagements.length,
      responseRate: recommended.length ? Math.round((responded.length / recommended.length) * 100) : null,
    };
  }

  function escapeHtml(value) {
    return String(value).replace(/[&<>"']/g, (ch) => ({
      "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"
    }[ch]));
  }

  function render() {
    if (!summary) return;
    document.getElementById("totalSold").textContent = summary.totalSold;
    document.getElementById("cameraCount").textContent = summary.cameraCount;
    document.getElementById("manualCount").textContent = summary.manualCount;
    document.getElementById("totalEngagements").textContent = summary.totalEngagements;
    document.getElementById("responseRate").textContent =
      summary.responseRate === null ? "-" : summary.responseRate + "%";

    const menuNames = Object.keys(summary.menuTotals).sort();
    const tbody = document.getElementById("menuRows");
    tbody.innerHTML = menuNames.length
      ? menuNames.map((m) => `<tr><td>${escapeHtml(m)}</td><td>${summary.menuTotals[m]}</td></tr>`).join("")
      : '<tr><td colspan="2">表示できるデータがありません</td></tr>';

    const canvas = document.getElementById("hourCanvas");
    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    const maxV = Math.max(1, ...summary.hourTotals);
    const barW = canvas.width / 24;
    ctx.fillStyle = "#176b56";
    summary.hourTotals.forEach((v, h) => {
      const barH = (v / maxV) * (canvas.height - 20);
      ctx.fillRect(h * barW + 2, canvas.height - barH - 16, barW - 4, barH);
      ctx.fillStyle = "#53635c";
      ctx.font = "9px system-ui, sans-serif";
      ctx.fillText(String(h), h * barW + 2, canvas.height - 4);
      ctx.fillStyle = "#176b56";
    });
  }

  async function load() {
    try {
      const since = sinceIso();
      const [consRes, engRes] = await Promise.all([
        fetch("/api/consumption_log?since=" + encodeURIComponent(since)),
        fetch("/api/engagement_log?since=" + encodeURIComponent(since)),
      ]);
      const events = consRes.ok ? (await consRes.json()).events || [] : [];
      const engagements = engRes.ok ? (await engRes.json()).engagements || [] : [];
      summary = computeSummary(events, engagements);
      render();
    } catch (e) { /* 通信エラー時は前回表示のまま */ }
  }

  function downloadBlob(content, filename, mimeType) {
    const blob = new Blob([content], { type: mimeType });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url; a.download = filename;
    document.body.appendChild(a); a.click(); a.remove();
    URL.revokeObjectURL(url);
  }

  function buildReportLines() {
    if (!summary) return [];
    const lines = [
      `集計期間: 直近${document.getElementById("periodSelect").value}日`,
      `総販売数: ${summary.totalSold}`,
      `カメラ検知: ${summary.cameraCount}`,
      `手動入力: ${summary.manualCount}`,
      `総接客回数: ${summary.totalEngagements}`,
      `反応率(5分以内): ${summary.responseRate === null ? "-" : summary.responseRate + "%"}`,
      "",
      "メニュー別販売数:",
    ];
    Object.keys(summary.menuTotals).sort().forEach((m) => {
      lines.push(`  ${m}: ${summary.menuTotals[m]}`);
    });
    return lines;
  }

  document.getElementById("periodSelect").addEventListener("change", load);
  document.getElementById("refreshBtn").addEventListener("click", load);
  document.getElementById("exportBtn").addEventListener("click", () => {
    const csv = "﻿" + buildReportLines().join("\\n");
    downloadBlob(csv, "activity_report.csv", "text/csv;charset=utf-8");
  });
  document.getElementById("exportTxtBtn").addEventListener("click", () => {
    downloadBlob(buildReportLines().join("\\n"), "activity_report.txt", "text/plain;charset=utf-8");
  });
  load();
</script>
</body>
</html>
"""


def build_activity_report_html() -> bytes:
    """発表資料にそのまま使える集計(日報/週報)を出す専用ページ。

    `full_dashboard_node.py`から、メインの操作盤ダッシュボードとは別ページ
    (`/activity_report`)として配信される(2026-07-22追加)。
    """
    return _ACTIVITY_REPORT_HTML.encode("utf-8")


# 【2026-07-22追加】発表・実証実験でロボットの隣に置く観客向けの表示画面
# (`/audience_display`、完成版ダッシュボードのみ)。操作者向けの専門的な
# 表示ではなく、大きい文字・平易な言葉で「今何をしているか」だけを見せる。
_AUDIENCE_DISPLAY_HTML = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>のこさへん - 案内表示</title>
<style>
  :root { color-scheme: light; font-family: system-ui, -apple-system, "Segoe UI", sans-serif; }
  html, body { height:100%; margin:0; background:#124739; color:#fff; overflow:hidden; }
  main { height:100%; display:flex; flex-direction:column; align-items:center; justify-content:center; gap:32px; padding:24px; box-sizing:border-box; text-align:center; }
  .status { font-size:clamp(28px, 6vw, 56px); font-weight:700; }
  .announce { font-size:clamp(20px, 4vw, 36px); background:rgba(255,255,255,0.12); border-radius:16px; padding:24px 32px; max-width:90%; min-height:1.4em; }
  .logo-title { display:flex; align-items:center; gap:12px; }
  .logo-title img { height:48px; width:48px; border-radius:10px; }
</style>
</head>
<body>
<main>
  <div class="logo-title">
    <img src="__NOKOSAHEN_LOGO__" alt="のこさへん" />
    <div style="font-size:20px;">のこさへん</div>
  </div>
  <div class="status" id="status">読み込み中...</div>
  <div class="announce" id="announce">-</div>
</main>
<script>
  // 401(未ログイン/セッション切れ)を受け取ったら自動でログイン画面へ誘導する。
  (function () {
    const rawFetch = window.fetch.bind(window);
    window.fetch = async (...args) => {
      const res = await rawFetch(...args);
      if (res.status === 401 && location.pathname !== "/login") {
        location.href = "/login";
      }
      return res;
    };
  })();

  const FRIENDLY_MODE = {
    CROWD_SCAN: "周りを見て回っています",
    FACE_SWEEP: "周りを見て回っています",
    MAP_NOT_LOADED_HOLD: "準備中です",
    FACE_PERSON: "お客様の方を向いています",
    CROWD_TURN_TO_CROWD: "周りを見て回っています",
    CROWD_APPROACH: "周りを見て回っています",
    APPROACH_PERSON: "お客様のところへ向かっています",
    APPROACH_ARRIVED: "ご案内中です",
    STOP_ANNOUNCE: "ご案内中です",
    RETURN_HOME: "定位置へ戻っています",
    RETURN_HOME_ARRIVED: "定位置で待機しています",
    RETURN_HOME_TIMEOUT: "定位置へ戻っています",
    WAIT_FOR_SENSOR: "準備中です",
    MANUAL_OVERRIDE: "手動運転中です",
    ESTOP_HOLD: "一時停止しています",
    NEAR_PERSON_HOLD: "安全のため一時停止しています",
    OUT_OF_BOUNDS_HOLD: "安全のため一時停止しています",
    KEEPOUT_ZONE_HOLD: "安全のため一時停止しています",
    CAMERA_OBSTACLE_HOLD: "安全のため一時停止しています",
    SURVEY_ACTIVE_HOLD: "アンケート回答中のため停止しています",
    POLICY_STAY: "定位置で待機しています",
    POLICY_GO_ENTRANCE: "定位置へ向かっています",
    POLICY_AT_ENTRANCE: "定位置で待機しています",
    POLICY_GO_PIN: "定位置へ向かっています",
    POLICY_AT_PIN: "定位置で待機しています",
    POLICY_PIN_UNAVAILABLE: "定位置で待機しています",
    POLICY_NO_SCAN: "定位置で待機しています",
  };

  async function poll() {
    try {
      const res = await fetch("/api/state");
      if (!res.ok) return;
      const s = await res.json();
      document.getElementById("status").textContent = FRIENDLY_MODE[s.mode] || "動作中です";
      document.getElementById("announce").textContent = s.announce_text || "-";
    } catch (e) { /* 無視、次のポーリングで再試行 */ }
  }
  setInterval(poll, 1500);
  poll();
</script>
</body>
</html>
""".replace("__NOKOSAHEN_LOGO__", LOGO_DATA_URI)


def build_audience_display_html() -> bytes:
    """発表・実証実験の観客向けに、大きい文字で今の状況だけを見せる専用ページ。

    `full_dashboard_node.py`から、メインの操作盤ダッシュボードとは別ページ
    (`/audience_display`)として配信される(2026-07-22追加)。
    """
    return _AUDIENCE_DISPLAY_HTML.encode("utf-8")
