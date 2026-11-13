from __future__ import annotations

import datetime
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

from . import announce_events as _announce_events
from . import announce_settings as _announce_settings
from . import pr_intro as _pr_intro
from . import behavior_rules as _rules
from . import boundary_motion_guard as _bmg
from . import crowd_bearing as _cb
from . import camera_tracks as _ct
from . import position_hold as _ph
from . import behavior_tree as _bt_mod
from . import schedule_plan as _schedule_plan
from .engagement_log import EngagementLog
from .json_tools import dumps, loads
from .motion_hold_logic import GrantTracker, HoldCancelEdge, UserCancelHold
from .map_orientation import (
    BOUNDARY_OVERRIDE_TOPIC,
    boundary_message_update,
    boundary_override_message,
    clamp_point_to_rect,
    point_in_rect,
    rect_from_state,
)
from .map_pins import PIN_REASON_TEXT, PINS_TOPIC, resolve_pin

# 直進速度の上限[m/s]。dashboard_common.pyのMAX_DRIVE_LINEAR_MPSと同じ値にすること
# (ESP32ファームが車輪ティック増分を400/20msでクランプし、約1.23 m/sで飽和するため、
# 余裕を持たせて1.1。詳細はdashboard_common.pyのコメント参照)。
# crowd_state(crowd_monitorが2Hzで配信)がこの秒数以上届かなければ、古い値を使わず「不明・人物なし」として扱う(2026-10-21)
CROWD_STATE_STALE_SEC = 3.0
# 【2026-11-03】向く動作で使う人の候補(/nokosahen/person_candidates)の有効期間[秒]。これより古い候補は使わない。
FACE_CANDIDATES_STALE_SEC = 1.5
NEAR_CLEAR_WINDOW_SEC = 1.0      # 近接停止の免除: 周りの物との距離を、新しく届いた測定でこの秒数ぶん見て判断する
NEAR_CLEAR_MIN_SAMPLES = 3       # 新しい測定がこの回数以上、
NEAR_CLEAR_MIN_SPAN_SEC = 0.4    # この秒数にわたって続いていること
NEAR_CLEAR_MAX_AGE_SEC = 0.5     # 最後の新しい測定が、これより古ければ確認できない(検出が遅れている)
NEAR_CLEAR_START_M = 0.55        # 免除を始める基準(全周の一番近い物までの距離)。旋回の外接円0.37 m+余白。【2026-11-05】0.45→0.55(LiDARは高さ12 cmの平面だけ。天板・張り出し・上半身の余裕)
NEAR_CLEAR_STAY_M = 0.48         # 免除を続ける基準(ばらつきで旋回と停止が交互にならないよう、始める基準より小さい)
# 向く動作で、候補が一瞬(センサの取りこぼし)消えても、この秒数は直前の向きを保って旋回を続ける(止まってカクつかない)。
FACE_LOSS_GRACE_SEC = 0.8
POSE_STALE_SEC = 1.5  # 【2026-11-05】map->base_link の最新変換がこれより古ければ、位置なし扱い(TFが固まった)

MAX_DRIVE_LINEAR_MPS = 1.1

# 【2026-10-13】directorが直接/cmd_velへ出す「前進」(見回し後の群衆方向への直進CROWD_APPROACH、
# enable_direct_cmd_vel有効時の接近フォールバック)の速度上限[m/s]。この経路はNav2の衝突監視
# (collision_monitor、cmd_vel_smoothed->cmd_velの間)を通らず、LiDARの前方停止もない(近接停止は人物のみ、
# カメラ障害物は机の天板等のみ)ため、通常のNav2最高速度やMAX_DRIVE_LINEAR_MPSより低く抑える。
# 0.5 m/sは衝突監視の最短の前進停止領域(50 cm、0.6 m/sまで)の範囲内の速度で、ファームの減速2.0 m/s^2での
# 停止距離は約6 cm。1回の直進は最大1.0 m/4秒に制限済み。詳細はdocs/open_items.md。
DIRECT_DRIVE_MAX_MPS = 0.5

try:
    from tf2_ros import Buffer, TransformListener
except Exception:  # pragma: no cover - 環境依存
    Buffer = None  # type: ignore
    TransformListener = None  # type: ignore


ROBOT_CLOCK_TOPIC = "/nokosahen/robot_clock"
SCHEDULE_COMMAND_TOPIC = "/nokosahen/schedule/command"


def _uptime_sec() -> float:
    """OSが起動してからの秒数(取れなければプロセスの単調時計)。"""
    try:
        with open("/proc/uptime", "r", encoding="utf-8") as f:
            return float(f.read().split()[0])
    except (OSError, ValueError, IndexError):
        return time.monotonic()


def robot_clock_payload(local_now: datetime.datetime, epoch: float, since_boot_sec: float,
                        utc_offset_hours: Optional[float] = None) -> dict:
    """機体が「いま何時か」と認識している時刻(木・ルール表の時刻条件が使っているもの)のJSON。
    local_now: 木の時計(naiveな現地時刻)。utc_offset_hours: utc_offset_overrideの値(無ければシステムのローカルTZ)。"""
    if utc_offset_hours is None:
        offset = datetime.datetime.fromtimestamp(epoch).astimezone().utcoffset() or datetime.timedelta(0)
    else:
        offset = datetime.timedelta(hours=float(utc_offset_hours))
    aware = local_now.replace(tzinfo=datetime.timezone(offset))
    hours = offset.total_seconds() / 3600.0
    tz = "Asia/Tokyo" if hours == 9.0 else ("UTC" if hours == 0.0 else "UTC%+g" % hours)
    return {
        "epoch": float(epoch),
        "iso": aware.isoformat(timespec="seconds"),
        "tz": tz,
        "clock_suspect": bool(_rules.clock_suspect(local_now)),  # 年が2026未満=時計が合っていない(docs/clock_sync.md)
        "since_boot_sec": float(since_boot_sec),
    }


class BehaviorDirector(Node):
    """High-level state machine for guide behavior."""

    # 【2026-10-23】スケジュール(docs/schedule.md)。__init__を通さないテスト用インスタンス向けの既定値
    _boundary_override_pub = None
    _robot_clock_pub = None
    _boundary_override_req: Optional[dict] = None  # 木のset_boundaryが今出している要求(None=上書きなし)
    _schedule_stop_hold_until: Optional[float] = None  # 停止の合図の後、この単調時計の時刻までその場で待機
    _schedule_stop_hold_sec = 3.0
    _stop_hold_activation = 2_000_000
    _utc_offset_hours: Optional[float] = None
    # 【2026-10-23】頭部ディスプレイの画面の要求(/nokosahen/display_request、木のset_display)。directorはラズパイを直接操作せず、
    # 要求を配信するだけ(受けるのはダッシュボードのDisplayScheduleController)
    _display_request_pub = None
    _display_last_key: Optional[tuple] = None  # 最後に配信した(モード, 項目id)
    _display_last_pub_mono = 0.0
    _schedule_active_entry_id: Optional[str] = None

    # 【2026-10-23】__init__で設定する。クラス側の既定None=未設定(__init__を通さないテスト用インスタンス向け)
    _hold_edge: Optional[HoldCancelEdge] = None
    _user_cancel: Optional[UserCancelHold] = None
    _direct_grant: Optional[GrantTracker] = None

    # アンケート回答中ホールドの既定値(購読が無い/__init__を通さない場合は従来どおり=ホールドしない)
    _suppress_sales_while_announcing = True  # B9。__init__でパラメータの値に上書き(テスト用の組み立てでも既定が効く)
    _survey_active = False
    _last_survey_active_time = None
    _survey_active_timeout_sec = 3.0
    # 【2026-10-10追加】走行可能範囲のせいで人の近くまで行けないとき、範囲の縁(射影した立ち位置)で
    # 人の方を向く判定。立ち位置にこの距離[m]以内なら「着いた」、向きの誤差がこの角度[rad](約20度)以内なら
    # 「人の方を向いた」とみなす。向けないまま待つ最長秒数を過ぎたら向きを諦めて到着扱いにする。
    boundary_stop_tolerance_m = 0.3
    boundary_face_tolerance_rad = 0.35
    boundary_face_timeout_sec = 8.0
    _boundary_face_since = None
    _boundary_face_seen = None

    # 【2026-11-03追加】3つの動作(近づく/見回す/向く)は独立。スイッチの既定はOFF(__init__でパラメータの値に上書き)。
    # 向く動作(その場で人の多い方を向く)の状態。__init__を通さないテスト用インスタンスでも動くよう、クラス側に既定を置く。
    _scan_enabled = False
    _face_enabled = False
    _person_candidates: Optional[dict] = None  # /nokosahen/person_candidates の最後の値
    _person_candidates_mono: Optional[float] = None  # 受信した単調時計の時刻
    _person_candidates_rx: Optional[float] = None  # 【2026-11-05】受信したROS時計の時刻[秒](近接停止の免除の鮮度用。単調時計と混ぜない)
    _face_selector = None
    _face_active = False
    _face_aligned = False
    _face_bearing: Optional[float] = None
    _face_ref_yaw: Optional[float] = None  # 向く動作の監視範囲の基準(スイッチONにしたときの車体の向き[世界rad])
    _face_sweep_dir = 1.0
    camera_guard_off_in_place_turn = True
    face_turn_ignores_near_person = True
    require_map_loaded = False  # __init__を通さないテスト用インスタンスは無効。実機は __init__ で、位置(地図)が取れる構成のとき有効
    map_missing_grace_sec = 3.0
    _pose_missing_since = None
    face_turn_rps = 0.75  # __init__を通さないテスト用インスタンスの既定(実機は __init__ で 1.5)
    face_turn_min_rps = 0.0  # 同上(実機は __init__ で 0.9)
    body_range_m = 0.36  # 同上: LiDAR候補のうち、この距離より近いものは機体の枠として無視する
    _face_last_world = None
    # 【2026-11-06】カメラ検出の追跡による安定化と、候補の階層化(docs/face_person_design.md 7.9章)。__init__を通さないテスト用インスタンスの既定も有効
    face_camera_track_enabled = True
    face_tier_enabled = True
    face_camera_track_confirm_n = _ct.CONFIRM_N
    face_camera_track_hold_sec = _ct.HOLD_SEC
    face_camera_track_match_deg = _ct.MATCH_DEG
    face_camera_track_alpha = _ct.SMOOTH_ALPHA
    face_tier2_after_sec = _ct.TIER2_AFTER_SEC
    _camera_tracker = None
    _tier_filter = None
    # 【2026-11-06】向く旋回中の位置保持(docs/face_person_design.md 7.10章)。既定は無効。__init__を通さないテスト用インスタンスも無効。
    _face_hold_enabled = False
    _hold = None
    _hold_odom = None   # (受信時刻, linear.x, x, y, yaw, angular.z)
    _hold_scan = None   # (受信時刻, LaserScan)
    _hold_subs_made = False
    _hold_err_logged = False
    _near_clear_hist: list = []
    _near_clear_last_mono = None
    _near_exempt_active = False
    _near_exempt_entered = False
    _face_ok_prev = False

    def __init__(self) -> None:
        super().__init__("behavior_director")
        self.declare_parameter("announce_cooldown_seconds", 8.0)
        # 【2026-10-02追加】発話依頼(announce_context)を出す最短間隔[秒]。
        # 以前はcooldown(8秒)ごとに同じ内容を再依頼してしまい、voice_bridgeの
        # 生成+再生(20〜40秒)が追いつかずキューが溢れて喋り続けていた。
        self.declare_parameter("announce_min_interval_seconds", 60.0)
        # 【2026-10-23 B9】announce_node(状況アナウンス)が鳴る状況の間は、営業の呼びかけの依頼を出さない。falseで従来どおり。
        self.declare_parameter("suppress_sales_while_announcing", True)
        self._suppress_sales_while_announcing = bool(self.get_parameter("suppress_sales_while_announcing").value)
        # 【注意、2026-07-17レビューで指摘】これは_approach_target()が
        # LiDAR/カメラの人物targetを見失った際にNav2ゴールの代わりに直接
        # /cmd_velで前進させるかどうかのフラグ。CROWD_SCAN(見回し行動)は
        # このフラグの値に関わらず常に直接/cmd_velで駆動する(見回しは
        # そもそもNav2ゴールという概念に馴染まない単純な旋回/前進のため)。
        # 名前だけ見ると「システム全体の直接cmd_vel許可」に読めるが、実際は
        # _approach_targetのフォールバック挙動のみを制御する点に注意。
        self.declare_parameter("enable_direct_cmd_vel", False)
        # 自動運転の速度(0.75 m/s・0.75 rad/s)。手動運転(現在は直進1.0 m/s・旋回
        # 1.5 rad/s、nokosahen.yaml)とは独立した値で、「手動の半分」という関係は保証しない。
        self.declare_parameter("approach_speed_mps", 0.75)
        self.declare_parameter("approach_turn_rps", 0.75)
        # 人物のちょうど立っている座標まで行ってしまわないよう、この距離だけ
        # 手前でゴールを打ち切る(会話に適した距離で停止させる)。
        self.declare_parameter("approach_standoff_m", 0.6)
        # 毎tick(timer_hz、既定2Hz)ゴールを送り直すとNav2が絶えず再計画して
        # 実際には前進できなくなるため、ゴールの再送間隔を空ける。
        self.declare_parameter("approach_goal_min_interval_sec", 2.0)
        # LiDARで確定した人物がまだいない間の「見回し」行動用。既定は
        # 旋回0.75 rad/s・直進0.5 m/s(手動運転とは独立。直進はDIRECT_DRIVE_MAX_MPSで頭打ち)。
        self.declare_parameter("crowd_scan_turn_rps", 0.75)
        # 【2026-11-04】向く(FACE_PERSON/FACE_SWEEP、その場の旋回)の旋回速度[rad/s]。手動運転の旋回(full_dashboard の angular_speed_rps、1.5)と同じ。
        self.declare_parameter("face_turn_rps", 1.5)
        # 【2026-11-05】向く旋回の指令の下限[rad/s]。実機で 0.6 rad/s 前後以下では車体が回らなかった(オドメトリが動かない)ため。0なら従来どおり。
        self.declare_parameter("face_turn_min_rps", 0.9)
        # 【2026-11-05】LiDARの中心からこの距離[m]より近いLiDAR候補は、機体の枠(四隅の支柱は0.37 m)なので、向く対象にしない。0で無効。
        self.declare_parameter("face_body_range_m", 0.36)
        # 【2026-11-06】向く動作のカメラ検出の安定化(世界方位で追跡・確認・保持・平滑化)と、候補の階層化(カメラ確認済みの人を優先)。false で従来どおり。
        self.declare_parameter("face_camera_track_enabled", True)
        self.declare_parameter("face_camera_track_confirm_n", _ct.CONFIRM_N)
        self.declare_parameter("face_camera_track_hold_sec", _ct.HOLD_SEC)
        self.declare_parameter("face_camera_track_match_deg", _ct.MATCH_DEG)
        self.declare_parameter("face_camera_track_alpha", _ct.SMOOTH_ALPHA)
        self.declare_parameter("face_tier_enabled", True)
        self.declare_parameter("face_tier2_after_sec", _ct.TIER2_AFTER_SEC)
        # 【2026-11-06】向く旋回中の位置保持(その場旋回なのに左回りで前へ進む実機のずれを、odom/地図上の位置で打ち消す)。
        # 既定は無効。実機で試して有効にする(`ros2 param set /behavior_director face_position_hold_enabled true`)。docs/face_person_design.md 7.10章。
        self.declare_parameter("face_position_hold_enabled", False)
        self.declare_parameter("face_position_hold_max_ff_mps", _ph.HoldParams.max_ff_mps)
        self.declare_parameter("face_position_hold_return_speed_mps", _ph.HoldParams.return_speed_mps)
        self.declare_parameter("face_position_hold_start_m", _ph.HoldParams.start_m)
        self.declare_parameter("face_position_hold_stop_m", _ph.HoldParams.stop_m)
        self.declare_parameter("face_position_hold_max_total_m", _ph.HoldParams.max_total_m)
        self.declare_parameter("face_position_hold_clear_m", _ph.HoldParams.clear_fwd_m)
        self.declare_parameter("crowd_scan_drive_mps", 0.5)
        # 見回しで一番人が多かった方向へ向き直った後、直進する上限時間/距離
        # (どちらか早く満たした方で切り上げて再評価する。この間にLiDARが
        # 人物を確定できればAPPROACH_PERSONへ引き継ぐ)。
        self.declare_parameter("crowd_scan_approach_seconds", 4.0)
        self.declare_parameter("crowd_scan_approach_distance_m", 1.0)
        self.declare_parameter("entrance_x", 0.0)
        self.declare_parameter("entrance_y", 0.0)
        self.declare_parameter("entrance_yaw", 0.0)
        # 【2026-07-17追加】人物にapproach_standoff_mまで接近して止まった
        # (APPROACH_ARRIVED)後、発話が終わる頃合いまでその場に留まってから、
        # entrance_x/y/yawへ自動で戻る(RETURN_HOME)。
        self.declare_parameter("approach_dwell_sec", 6.0)
        self.declare_parameter("return_home_timeout_sec", 30.0)
        # 【2026-07-17追加】一度接近・案内した人物に、しつこく何度も接近し
        # 続けないための同一人物クールダウン。直近に接近完了した位置から
        # approach_cooldown_radius_m以内にいる人物は、approach_cooldown_min
        # 分の間は新たな接近対象にしない(見回しへフォールバックする)。
        self.declare_parameter("approach_cooldown_min", 3.0)
        self.declare_parameter("approach_cooldown_radius_m", 1.0)
        # 【2026-07-16】2Hzのままだと、CROWD_SCANの直接cmd_vel駆動やestop時の
        # 停止publishがNav2側の制御周期(controller_frequency、8〜12Hz程度)より
        # 遅く、緊急停止・手動運転への切り替えが体感できるほど遅延する
        # (最悪0.5秒近く反応しない)ことが実機で確認された。10Hzに上げて
        # 停止側が確実に上書きできるようにする。
        self.declare_parameter("timer_hz", 10.0)
        # /nokosahen/manual_overrideは「trueを一度受け取ったら、次にfalseが
        # 来るまでずっと手動運転扱い」という単純なラッチだと、送信側(ダッシュ
        # ボード以外の手動テスト等)がfalseを送り忘れた/送れなかった場合に、
        # 自律の安全ロジック(混雑時の停止等)が永久に働かなくなり、その間に
        # 最後に送られた/cmd_velがGazebo/実機側で延々適用され続ける
        # (どちらのdiff_driveにもコマンドタイムアウトが無いため)。
        # これを防ぐため、trueは一定時間ごとに再送されなければ自動的に
        # 失効(false扱い)するタイムアウト式にする。
        self.declare_parameter("manual_override_timeout_sec", 1.0)

        # 発話内容の設定ブロック。コードを触らずに `nokosahen.yaml` 側で
        # 「差が出るまでの間に話す各メニューのメリット」と「差が出た後の
        # 宣伝文のテンプレート」を調整できるようにする。
        # menu_merit_names[i] と menu_merit_texts[i] が対になる(辞書は
        # ROS 2パラメータとして直接宣言できないため、対になる配列で表現)。
        self.declare_parameter("menu_merit_names", [""])
        self.declare_parameter("menu_merit_texts", [""])
        self.declare_parameter(
            "induction_template",
            "本日は{item}にまだ余裕があります。できたてのうちにいかがですか。",
        )
        self.declare_parameter(
            "generic_initial_text", "本日のメニューをご確認ください。お好きなものをどうぞ。"
        )

        self.announce_cooldown_seconds = float(
            self.get_parameter("announce_cooldown_seconds").value
        )
        self.announce_min_interval_seconds = max(
            self.announce_cooldown_seconds,
            float(self.get_parameter("announce_min_interval_seconds").value),
        )
        # 【2026-10-06追加】発話間隔のホットリロード設定ファイル(ダッシュボードの
        # /voice_settings ページが書く。docs/voice_settings.md)。優先順位:
        # このファイル > 上のROSパラメータ。ファイルが無ければパラメータの値のまま。
        self.declare_parameter(
            "announce_settings_file", _announce_settings.DEFAULT_FILE
        )
        self._announce_param_interval = self.announce_min_interval_seconds
        self._announce_reader = _announce_settings.AnnounceSettingsReader(
            str(self.get_parameter("announce_settings_file").value)
        )
        # 【2026-10-31】出店紹介(文化祭)とメニュー紹介を同時に流さない。出店紹介が選ばれている間は営業の呼びかけを出さない
        # (設定は announce_node と同じ ~/.nokosahen_pr_intro.json の speech_mode。ファイルが無い/不正なら従来どおりメニュー紹介)。
        self.declare_parameter("pr_intro_file", _pr_intro.DEFAULT_FILE)
        self._pr_reader = _pr_intro.PrIntroReader(str(self.get_parameter("pr_intro_file").value))
        self._refresh_announce_settings()
        self.enable_direct_cmd_vel = bool(
            self.get_parameter("enable_direct_cmd_vel").value
        )
        self.approach_speed_mps = min(
            MAX_DRIVE_LINEAR_MPS, float(self.get_parameter("approach_speed_mps").value)
        )
        self.approach_turn_rps = float(
            self.get_parameter("approach_turn_rps").value
        )
        self.approach_standoff_m = max(
            0.0, float(self.get_parameter("approach_standoff_m").value)
        )
        self.approach_goal_min_interval_sec = max(
            0.1, float(self.get_parameter("approach_goal_min_interval_sec").value)
        )
        self.crowd_scan_turn_rps = float(self.get_parameter("crowd_scan_turn_rps").value)
        self.face_turn_rps = max(0.05, float(self.get_parameter("face_turn_rps").value))
        self.face_turn_min_rps = max(0.0, float(self.get_parameter("face_turn_min_rps").value))
        self.body_range_m = max(0.0, float(self.get_parameter("face_body_range_m").value))
        self.face_camera_track_enabled = bool(self.get_parameter("face_camera_track_enabled").value)
        self.face_camera_track_confirm_n = max(1, int(self.get_parameter("face_camera_track_confirm_n").value))
        self.face_camera_track_hold_sec = max(0.0, float(self.get_parameter("face_camera_track_hold_sec").value))
        self.face_camera_track_match_deg = max(0.0, float(self.get_parameter("face_camera_track_match_deg").value))
        self.face_camera_track_alpha = float(self.get_parameter("face_camera_track_alpha").value)
        self.face_tier_enabled = bool(self.get_parameter("face_tier_enabled").value)
        self.face_tier2_after_sec = max(0.0, float(self.get_parameter("face_tier2_after_sec").value))
        self._face_hold_enabled = bool(self.get_parameter("face_position_hold_enabled").value)
        self._hold = None  # 最初に使うとき、パラメータから作る(_hold_get)
        self.add_on_set_parameters_callback(self._on_hold_params)
        # 【2026-11-04】地図を読み込んでいない(地図上の位置が取れない)間は、自律動作(向く・見回す・近づく・揺動・ホーム帰還)をせず、
        # 「地図を読み込んでください」と表示して止まる(MAP_NOT_LOADED_HOLD)。地図作成中/探索中・手動運転は対象外。
        # 位置が取れない状態が map_missing_grace_sec 秒続いたときだけ(一瞬のTF欠けでは止めない)。false で無効。
        self.declare_parameter("require_map_loaded", True)
        self.declare_parameter("map_missing_grace_sec", 3.0)
        self.map_missing_grace_sec = max(0.0, float(self.get_parameter("map_missing_grace_sec").value))
        self.crowd_scan_drive_mps = float(self.get_parameter("crowd_scan_drive_mps").value)
        self.crowd_scan_approach_seconds = max(
            0.5, float(self.get_parameter("crowd_scan_approach_seconds").value)
        )
        self.crowd_scan_approach_distance_m = max(
            0.1, float(self.get_parameter("crowd_scan_approach_distance_m").value)
        )
        self.entrance_x = float(self.get_parameter("entrance_x").value)
        self.entrance_y = float(self.get_parameter("entrance_y").value)
        self.entrance_yaw = float(self.get_parameter("entrance_yaw").value)
        self.approach_dwell_sec = max(
            0.0, float(self.get_parameter("approach_dwell_sec").value)
        )
        self.return_home_timeout_sec = max(
            1.0, float(self.get_parameter("return_home_timeout_sec").value)
        )
        self.approach_cooldown_sec = max(
            0.0, float(self.get_parameter("approach_cooldown_min").value) * 60.0
        )
        self.approach_cooldown_radius_m = max(
            0.0, float(self.get_parameter("approach_cooldown_radius_m").value)
        )
        timer_hz = max(0.5, float(self.get_parameter("timer_hz").value))
        self.manual_override_timeout_sec = max(
            0.1, float(self.get_parameter("manual_override_timeout_sec").value)
        )

        merit_names = [str(n) for n in self.get_parameter("menu_merit_names").value if str(n)]
        merit_texts = [str(t) for t in self.get_parameter("menu_merit_texts").value if str(t)]
        if len(merit_names) != len(merit_texts):
            # zip()は短い方に無警告で合わせてしまうため、設定ミス(names/textsの
            # 更新忘れ)でローテーション機能全体が無警告で無効化されるのを防ぐ。
            self.get_logger().warn(
                "menu_merit_names(%d件)とmenu_merit_texts(%d件)の数が一致していません。"
                "対応しない末尾は無視されます。nokosahen.yamlを確認してください。"
                % (len(merit_names), len(merit_texts))
            )
        self.menu_merits = dict(zip(merit_names, merit_texts))

        default_induction_template = (
            "本日は{item}にまだ余裕があります。できたてのうちにいかがですか。"
        )
        induction_template = str(self.get_parameter("induction_template").value)
        try:
            induction_template.format(item="dummy")
        except (KeyError, ValueError, IndexError) as exc:
            # {item}以外のプレースホルダの誤字・閉じ忘れの`{`等があると、毎tick
            # 例外になりノードが壊れ続ける。起動時に一度自己テストしておき、
            # 壊れているならデフォルトへフォールバックする。
            self.get_logger().error(
                f"induction_templateの書式が不正なためデフォルトを使います: {exc}"
            )
            induction_template = default_induction_template
        self.induction_template = induction_template
        self.generic_initial_text = str(self.get_parameter("generic_initial_text").value)

        # 走行可能範囲(矩形、map座標系)。ダッシュボードの地図クリック2回で
        # 設定される(BoundaryController)。手動運転(WASD)は対象外で、自律動作
        # (混雑時のゴール送信・見回し中の直進等)が範囲外へ出ないようにするための
        # 安全策としてのみ使う。未設定(enabled=false)なら制限しない。
        # 【2026-10-06】範囲は回転矩形(center_x/center_y/width/height/yaw)。旧形式
        # (min_x/max_x/min_y/max_y)の発行元とも互換(yaw=0の矩形として読む)。
        # 軸平行外接矩形(min/max)だけで判定すると、斜めの範囲の外側(外接矩形の
        # 角の余白)を「範囲内」と誤認するため、判定・クランプは真の矩形で行う。
        self._boundary: dict = {"enabled": False}
        self.create_subscription(
            String, "/nokosahen/travel_boundary", self._on_boundary, 10
        )

        # 【2026-07-17】地図内の「ここには入らないで」立入禁止エリア(矩形、
        # 複数可、map座標系)。ダッシュボードの地図編集ページ(/map_editor)から
        # 設定される(KeepOutZoneController)。BoundaryController(外周1つの
        # 走行可能範囲)とは別物。手動運転(WASD)は対象外、自律動作のみ対象
        # (走行可能範囲と同じ設計方針)。
        self._keepout_zones: list = []
        self.create_subscription(
            String, "/nokosahen/keepout_zones", self._on_keepout_zones, 10
        )

        # LiDARは足の高さの水平面しか見ておらず、机の天板・ガラス・段差等の
        # 障害物を見逃す。person_target_detectorがカメラで検知した
        # 「近くに大きく写っている物」の情報を使い、自律走行を安全側に止める。
        # person_target同様、メッセージが届かなくなったら(視界から消えた等)
        # 古い検知結果を永久に信じ続けないよう受信時刻で鮮度チェックする。
        self.declare_parameter("camera_obstacle_timeout_sec", 1.0)
        # 【2026-11-04】向く(その場の旋回だけ)で動いているとき、カメラの障害物検知による停止を効かせない(既定true)。それ以外は有効。
        self.declare_parameter("camera_guard_off_in_place_turn", True)
        self.camera_guard_off_in_place_turn = bool(self.get_parameter("camera_guard_off_in_place_turn").value)
        # 【2026-11-04】向く(その場の旋回だけ)で動いているとき、近接停止(NEAR_PERSON_HOLD)を効かせない(すぐ前の人の方も向けるように)。
        # ただし、機体が回って触れる範囲(外接円+余白)に人が入っているとき・人との距離が確認できないときは、停止のまま。
        self.declare_parameter("face_turn_ignores_near_person", True)
        self.face_turn_ignores_near_person = bool(self.get_parameter("face_turn_ignores_near_person").value)
        self._face_turn_pub = self.create_publisher(Bool, "/nokosahen/face_inplace_turn", 5)
        self.camera_obstacle_timeout_sec = max(
            0.1, float(self.get_parameter("camera_obstacle_timeout_sec").value)
        )
        self._camera_obstacle_detected = False
        self._last_camera_obstacle_receive_time = None
        self.create_subscription(
            String, "/nokosahen/camera_obstacle", self._on_camera_obstacle, 5
        )

        # 【2026-07-17】person_target_detectorがLiDAR脚クラスタで検知した
        # 「人がnear_person_stop_distance_m(既定0.5m)以内にいる」通知。
        # 【2026-09-30コメント修正】実際の_tick()の優先順位は
        # ESTOP_HOLD > MANUAL_OVERRIDE > NEAR_PERSON_HOLD(この安全停止)で、
        # 手動運転(WASD)中はこの停止は適用されない(操作者がロボットのすぐ
        # そばにいるのが普通のため、上書きしない方針に変更済み)。
        # TeleopController(dashboard_common.py)側も同じトピックを購読するが、
        # 状態表示用に保持するだけでWASDの駆動指令はブロックしない。
        # 自律動作(見回し・接近・自動地図作成等)に対してのみ最優先で止める。
        self.declare_parameter("near_person_stop_timeout_sec", 1.0)
        self.near_person_stop_timeout_sec = max(
            0.1, float(self.get_parameter("near_person_stop_timeout_sec").value)
        )
        self._near_person_stop_detected = False
        self._last_near_person_stop_receive_time = None
        # 【2026-11-04】受信が一度あったのに途絶えたら(person_target_detectorが落ちた等)、近接停止 true と同じ扱い(安全側)。
        # 一度も受信していない間は適用しない(検出ノードなしの構成は従来どおり)。0以下で無効。
        self.declare_parameter("near_person_stale_sec", 2.0)
        self.near_person_stale_sec = float(self.get_parameter("near_person_stale_sec").value)
        self._near_person_stale_warned = False
        self.create_subscription(
            Bool, "/nokosahen/near_person_stop", self._on_near_person_stop, 5
        )

        # 【2026-09-30追加】地図作成(SLAM)中・自動探索中は、自律動作(見回しの
        # 旋回、入口へのNav2ゴール、人物接近等)がfrontier_explorer/手動運転の
        # /cmd_velやNav2ゴールと奪い合わないよう抑止する。
        # ・/nokosahen/map_building_active: MapLifecycleControllerが地図作成
        #   (SLAM)モード中だけTrueで約1Hz再送する(/nokosahen/mapping_activeは
        #   localization中もTrueになり通常運用まで止めてしまうため使わない)。
        # ・frontier_explorer/status: enabled=Trueが一定時間内に届いている間は
        #   探索中とみなす(ノード停止時に抑止したままになるのを防ぐため失効あり)。
        # 何も受信していない間は「非アクティブ」(従来通り自律動作する)。
        self._map_building_active = False
        self._frontier_enabled = False
        self._last_frontier_status_time = None
        self._frontier_status_timeout_sec = 10.0
        self.create_subscription(
            Bool, "/nokosahen/map_building_active", self._on_map_building_active, 5
        )
        self.create_subscription(
            String, "/nokosahen/frontier_explorer/status", self._on_frontier_status, 5
        )
        # 自分がNav2ゴールを送った可能性があるか(直接cmd_velモードへ切り替える時に
        # 1回だけ/nokosahen/cancel_goalを送るための記録)。
        self._nav2_goal_pending = False
        # 前tickのmode(直接cmd_velで動くモードからMAPPING_ACTIVE_HOLDへ
        # 切り替わった瞬間に1回だけ停止を送るための記録)。
        self._prev_tick_mode = ""

        # 【2026-10-09追加】ラズパイ腹部画面のアンケート回答中は止まって待機する
        # (SURVEY_ACTIVE_HOLD)。survey_monitorノードが/nokosahen/survey_activeを
        # 5Hzで配信する。一度も届かない/配信が途絶えた(失効秒数を超えた)間は
        # 「回答中ではない」=従来と完全に同じ動き(ノード未起動でも変わらない)。
        self._survey_active = False
        self._last_survey_active_time = None
        self._survey_active_timeout_sec = 3.0
        self.create_subscription(
            Bool, "/nokosahen/survey_active", self._on_survey_active, 5
        )

        # 【2026-07-22追加】接客(接近+案内)履歴とA/Bテスト(推奨発話ON/OFF)の
        # 記録先。完成版ダッシュボードの推奨効果分析・日報週報が読み取る。
        self.declare_parameter("engagement_db_file", "~/.nokosahen_engagement.db")
        self._engagement_log = EngagementLog(
            str(self.get_parameter("engagement_db_file").value)
        )
        # A/Bテスト: falseにすると、induction phaseでも特定メニューを推さず
        # 常に汎用の呼びかけ(_rotating_merit_text/generic instruction)にする。
        # ロボットの接近・発話自体は変えず「特定メニューの推奨」だけを止める
        # ことで、推奨の効果だけを切り分けて比較できるようにする。
        self.declare_parameter("recommend_enabled", True)
        self._recommend_enabled = bool(self.get_parameter("recommend_enabled").value)
        # 【2026-10-13追加】現在値を状態(behavior_command)に載せるための最終変更時刻(エポック秒)。
        self._recommend_updated_at = datetime.datetime.now().timestamp()
        self._engagement_log.record_mode_change(
            datetime.datetime.now().isoformat(timespec="seconds"), self._recommend_enabled
        )
        self.create_subscription(
            Bool,
            "/nokosahen/behavior_director/recommend_enabled",
            self._on_recommend_enabled,
            5,
        )
        # 【2026-07-22追加】デモ用強制トリガー。生の人流に頼らず、今LiDAR/
        # カメラで検知している人物へ確実に接近・発話させたい時に、
        # 同一人物クールダウンだけを解除する(状態機械自体は変更しない)。
        self.create_subscription(
            Bool,
            "/nokosahen/behavior_director/force_demo",
            self._on_force_demo,
            5,
        )
        # 【2026-08-08追加】人物がまだ見つかっていない間の待機動作として、
        # LiDAR/カメラの判定を一切使わず単純に左右へ1秒ずつ交互に回転する
        # 演出(CROWD_SCANの代わり)。スマホ操作画面からON/OFFできる。
        self.declare_parameter("idle_wiggle_enabled", False)
        self._idle_wiggle_enabled = bool(self.get_parameter("idle_wiggle_enabled").value)
        self.create_subscription(
            Bool,
            "/nokosahen/behavior_director/idle_wiggle_enabled",
            self._on_idle_wiggle_enabled,
            5,
        )
        self._wiggle_direction = 1.0
        self._wiggle_phase_start_time: Optional[object] = None

        # 【2026-11-02追加】人の方へ動く動作(接近・見回し・入口への呼びかけ移動)は「指定したときだけ」。
        # 指定 = 管理画面のスイッチ(下のトピック)がON、または時間割/ビヘイビアツリー/ルールが
        # approach_enabled / crowd_scan_enabled を明示した間。どちらも無ければ、fullで起動しても
        # その場で待つ(発話は従来どおり)。require_person_interaction_opt_in=false で従来動作(常に許可)。
        self.declare_parameter("require_person_interaction_opt_in", True)
        self._require_interaction_opt_in = bool(
            self.get_parameter("require_person_interaction_opt_in").value
        )
        # 呼びかけ(メニュー紹介の発話)も、ビヘイビアツリーの announce 動作(またはルールの発話間隔指定)で
        # 指示した間だけ。false で従来動作(常に出す)。安全のお知らせ(状況アナウンス)・出店紹介は別経路で影響なし。
        self.declare_parameter("require_announce_opt_in", True)
        self._require_announce_opt_in = bool(self.get_parameter("require_announce_opt_in").value)
        self.declare_parameter("person_interaction_enabled", False)
        self._person_interaction_enabled = bool(
            self.get_parameter("person_interaction_enabled").value
        )
        self.create_subscription(
            Bool,
            "/nokosahen/behavior_director/person_interaction_enabled",
            self._on_person_interaction_enabled,
            5,
        )
        # 【2026-11-03追加】見回す・向くは、近づくとは独立のスイッチ(どれも既定OFF、再起動でOFFに戻る)。
        self.declare_parameter("scan_enabled", False)
        self._scan_enabled = bool(self.get_parameter("scan_enabled").value)
        self.create_subscription(
            Bool, "/nokosahen/behavior_director/scan_enabled", self._on_scan_enabled, 5
        )
        self.declare_parameter("face_enabled", False)
        self._face_enabled = bool(self.get_parameter("face_enabled").value)
        self.create_subscription(
            Bool, "/nokosahen/behavior_director/face_enabled", self._on_face_enabled, 5
        )
        # 人の候補(LiDAR全候補+カメラの人ごとの方位)。向く動作の目標方位を決める材料。
        self.create_subscription(
            String, "/nokosahen/person_candidates", self._on_person_candidates, 5
        )

        self._menu_state: dict = {}
        self._crowd_state: dict = {}
        self._crowd_state_mono: Optional[float] = None  # 最後にcrowd_stateを受け取った単調時計の時刻
        self._hardware_estop = False
        self._soft_estop = False
        self._manual_override_raw = False
        self._last_manual_override_time = None
        self._last_announce = ""
        self._last_announce_time = self.get_clock().now()
        self._last_announce_time_set = False
        self._last_goal_time = self.get_clock().now()
        self._last_approach_goal_time = self.get_clock().now()
        self._merit_rotation_index = 0
        self._last_merit_change_time = self.get_clock().now()

        # 【2026-07-17追加】接近完了(APPROACH_ARRIVED)後のホーム帰還(RETURN_HOME)。
        self._arrived_at_time: Optional[object] = None
        self._home_return_active = False
        self._home_return_start_time: Optional[object] = None
        # 【2026-07-17追加】直近に接近完了した人物の位置(map座標系)+時刻。
        # 同一人物への連続アプローチを防ぐクールダウン判定に使う。
        self._recent_serve_position: Optional[tuple] = None
        self._recent_serve_time: Optional[object] = None

        # 見回し(CROWD_SCAN)行動の状態。LiDARで確定した人物(target)がまだ
        # いない間、旋回しながらカメラの人数密度を見て一番多かった方向を
        # 覚えておき、そちらへ向き直ってから前進する。
        #
        # 回転量・移動量は「コマンドしたturn_rps/speed × 経過時間」という
        # 見積もり(デッドレコニング)だけに頼らず、map->base_link のTF
        # (slam_toolboxのLiDARスキャンマッチング + ホイールオドメトリの
        # 融合結果。RobotPoseControllerと同じ仕組み)から実測する。
        # エンコーダだけの見積もりは車輪の空転・ズレで「進んだつもり」に
        # なりやすいため、実際にどれだけ回転・移動したかをTFで確認してから
        # 見回し完了/接近完了を判定する。
        self.tf_available = Buffer is not None
        self.require_map_loaded = bool(self.get_parameter("require_map_loaded").value) and self.tf_available
        if self.tf_available:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        self._camera_person_density: Optional[dict] = None
        self._last_tick_time = self.get_clock().now()
        self._reset_crowd_scan(self.get_clock().now())

        self.create_subscription(
            String, "/nokosahen/menu_state", self._on_menu_state, 10
        )
        self.create_subscription(
            String, "/nokosahen/crowd_state", self._on_crowd_state, 10
        )
        self.create_subscription(
            String,
            "/nokosahen/camera_person_density",
            self._on_camera_density,
            5,
        )
        self.create_subscription(Bool, "/estop_state", self._on_hardware_estop, 10)
        # ダッシュボードからのソフト緊急停止(GPIO34のハード遮断とは別に、
        # ソフトウェア側から即座に停止させたい場合の手段)。
        self.create_subscription(Bool, "/nokosahen/soft_estop", self._on_soft_estop, 10)
        # ダッシュボードのWASD操作/地図クリックでの手動運転中は、この自律
        # ロジックがgoal/cmd_velを出さないようにするためのフラグ。
        self.create_subscription(
            Bool, "/nokosahen/manual_override", self._on_manual_override, 10
        )
        # 完成版ダッシュボードから自動移動(接近/巡回)の最高速度を実行時に
        # 変更するための購読。WASD手動運転の最高速度(TeleopController側)とは
        # 別系統。
        self.create_subscription(
            String,
            "/nokosahen/behavior_speed_override",
            self._on_speed_override,
            10,
        )

        self._command_pub = self.create_publisher(
            String, "/nokosahen/behavior_command", 10
        )
        self._announce_pub = self.create_publisher(
            String, "/nokosahen/announce_text", 10
        )
        # nokosahen_voice/voice_bridgeがAI(Ollama)に実際の発話文面を考えさせる
        # ための材料(メニュー一覧+メリット文、induction時は残数状況も)。
        # ルールベースの/nokosahen/announce_textはダッシュボード表示用・
        # AI生成失敗時のフォールバックとして引き続き並行publishする。
        self._announce_context_pub = self.create_publisher(
            String, "/nokosahen/announce_context", 10
        )
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        # 走行可能範囲外・カメラ障害物によるHOLD中は、nav2_goal_router側に
        # 実行中のNav2ゴールをキャンセルさせる(このノード自身はcmd_velを
        # ゼロにするだけで、送信済みのNav2ゴールを止める手段を持たないため。
        # 2026-07-17レビューで発見: HOLD中もNav2がゴールへ向けて動き続け、
        # 境界外へ出て行く余地があった)。
        self._cancel_goal_pub = self.create_publisher(Bool, "/nokosahen/cancel_goal", 10)

        # 【2026-10-23追加】(B2)HOLD系のcancel_goalは、HOLDに入った(別のHOLDに切り替わった)最初の1回だけ。
        # 毎tick出すとルーターのゼロ速度バーストが延び続ける。ゼロ速度(/cmd_vel)は従来どおり毎tick出す。
        # false にすると従来どおり毎tick出す。
        self.declare_parameter("hold_cancel_once", True)
        self._hold_edge = HoldCancelEdge(bool(self.get_parameter("hold_cancel_once").value))
        # 【2026-10-23追加】(B6)ダッシュボードの「中止」(/nokosahen/user_cancel)を受けたら、
        # user_cancel_hold_sec秒の間、目的地(入口・ピン・人物接近)を出し直さない。0以下で無効。
        # 専用トピックにした理由: 自分が出した/nokosahen/cancel_goalと取り違えず、ユーザーの中止を取りこぼさない。
        self.declare_parameter("user_cancel_hold_sec", 10.0)
        self._user_cancel = UserCancelHold(float(self.get_parameter("user_cancel_hold_sec").value))
        self.create_subscription(Bool, "/nokosahen/user_cancel", self._on_user_cancel, 10)
        # 【2026-10-23追加】(B3)見回しの旋回・直進・アイドル揺動・direct_cmd_velの直接/cmd_velは、
        # motion_arbiterの許可(/nokosahen/motion_grant の allow.director_direct)があるときだけ出す。
        # 一度も受信していない(arbiter無し)=許可、受信が1秒途絶えたら不許可。falseで許可制を無効化。
        self.declare_parameter("direct_cmd_vel_requires_grant", True)
        self._direct_grant = GrantTracker(
            "director_direct", 1.0, bool(self.get_parameter("direct_cmd_vel_requires_grant").value)
        )
        self.create_subscription(String, "/nokosahen/motion_grant", self._on_motion_grant, 5)

        # 【2026-10-06追加】時間帯・状況に応じた方針(policy)のルール層。
        # 既定ではルール表が空=現行動作そのまま。方針はESTOP/MANUAL_OVERRIDE/
        # 各種HOLD/MAPPING_ACTIVE_HOLDより下の「非安全分岐」にしか効かない。
        # 詳細は docs/behavior_rules.md。
        self.declare_parameter("rules_file", _rules.DEFAULT_RULES_FILE)
        self.declare_parameter("utc_offset_override_enabled", False)
        self.declare_parameter("utc_offset_hours", 9.0)
        offset = (
            float(self.get_parameter("utc_offset_hours").value)
            if bool(self.get_parameter("utc_offset_override_enabled").value)
            else None
        )
        self._rule_engine = _rules.RuleEngine(
            str(self.get_parameter("rules_file").value),
            clock=_rules.make_clock(offset),
        )
        self._rule_effects = _rules.NEUTRAL
        self._rule_activation = 0
        self._rule_active_id: Optional[str] = None
        self._rule_policy = "normal"
        self._policy_goal_done_activation = -1
        self._policy_goal_outstanding = False
        self._policy_goal_start_time: Optional[object] = None
        self._rule_error_logged = False
        self._rules_state_pub = self.create_publisher(
            String, "/nokosahen/behavior_rules/state", 10
        )
        # 【2026-10-09追加】ビヘイビアツリー(docs/behavior_tree.md)。木は「方針(Effects)」を選ぶだけで、
        # 上のESTOP〜MAPPING_ACTIVE_HOLDの固定の安全分岐(_tick)より下の非安全分岐にしか効かない。
        # ファイルが無い/不正/enabled:falseなら従来のルール表の結果をそのまま使う(動作は変わらない)。
        self.declare_parameter("tree_file", _bt_mod.DEFAULT_TREE_FILE)
        self._bt = _bt_mod.TreeRunner(
            str(self.get_parameter("tree_file").value),
            clock=_rules.make_clock(offset),
        )
        # run_on_startup による自動実行は、起動から何秒以上たち、かつ時計が合っている(clock_suspectでない)ときだけ
        self.declare_parameter("schedule_startup_delay_sec", 120.0)
        self._bt.startup_delay_sec = max(0.0, float(self.get_parameter("schedule_startup_delay_sec").value))
        self._bt_error_logged = False
        self._bt_status_pub = self.create_publisher(
            String, "/nokosahen/behavior_tree/status", 10
        )
        # 【2026-10-23追加】スケジュール(docs/schedule.md)。
        # - /nokosahen/boundary_override: 木のset_boundaryが出す「走行範囲の差し替え/解除」の要求を1Hzで配信する。
        #   directorが直接/nokosahen/travel_boundaryへ出すと、ダッシュボードのBoundaryControllerの5秒ごとの再送と
        #   競合して範囲が行き来するため、travel_boundaryを出すのはBoundaryControllerだけにして、そちらで合成する。
        #   この配信が止まれば(directorの停止・故障)、BoundaryControllerはTTL後に元の範囲へ戻す(安全側)。
        # - /nokosahen/robot_clock: 機体が認識している時刻(1Hz)。ダッシュボードの「機体の時計」表示用。
        # - /nokosahen/schedule/command: 「実行」「停止」の合図(JSON {"cmd":"run"|"stop"})。
        self._boundary_override_pub = self.create_publisher(String, BOUNDARY_OVERRIDE_TOPIC, 10)
        self._robot_clock_pub = self.create_publisher(String, ROBOT_CLOCK_TOPIC, 10)
        self._display_request_pub = self.create_publisher(String, _schedule_plan.DISPLAY_REQUEST_TOPIC, 10)
        self.declare_parameter("schedule_stop_hold_sec", 3.0)
        self._schedule_stop_hold_sec = max(0.0, float(self.get_parameter("schedule_stop_hold_sec").value))
        self._utc_offset_hours = offset
        self.create_subscription(String, SCHEDULE_COMMAND_TOPIC, self._on_schedule_command, 10)
        # 【2026-10-09追加】方針goto_pin(地図ピンへ行く)。ダッシュボードが配信する
        # `/nokosahen/pins`(読み込み中の地図のピン、TRANSIENT_LOCAL)を受け取り、ルールの
        # pin_idで引く。配信が途絶えた/地図が違う/ピンが無いときは走行せずその場で待機する。
        self.declare_parameter("pins_stale_sec", 30.0)
        self.declare_parameter("pin_goal_timeout_sec", 120.0)
        self.pins_stale_sec = max(6.0, float(self.get_parameter("pins_stale_sec").value))
        self.pin_goal_timeout_sec = max(10.0, float(self.get_parameter("pin_goal_timeout_sec").value))
        self._pins_payload: Optional[dict] = None
        self._pins_rx_time = None
        self._policy_pin_goal_key: Optional[tuple] = None
        self._policy_pin_logged: Optional[str] = None
        try:
            from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

            pins_qos = QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
        except Exception:  # pragma: no cover - 環境依存
            pins_qos = 1
        self.create_subscription(String, PINS_TOPIC, self._on_pins, pins_qos)
        self.create_subscription(
            Empty, "/nokosahen/behavior_rules/reload", self._on_rules_reload, 5
        )
        self._rules_timer = self.create_timer(1.0, self._rules_tick)
        self._timer = self.create_timer(1.0 / timer_hz, self._tick)

    def _on_pins(self, msg: String) -> None:
        data = loads(msg.data, None)
        if isinstance(data, dict) and isinstance(data.get("pins"), list):
            self._pins_payload = data
            self._pins_rx_time = self.get_clock().now()

    def _resolve_policy_pin(self, eff):
        """方針goto_pinの行き先ピンを、いま読み込み中の地図から引く。(ピン, 理由コード)。
        配信が古い(pins_stale_sec超)場合はno_data扱い=走行しない(別地図を読み込み直した後の
        古いピンへ向かわないため)。"""
        payload = getattr(self, "_pins_payload", None)
        rx = getattr(self, "_pins_rx_time", None)
        if payload is None or rx is None:
            return None, "no_data"
        if (self.get_clock().now() - rx).nanoseconds / 1e9 > getattr(self, "pins_stale_sec", 30.0):
            return None, "no_data"
        return resolve_pin(payload, eff.go_to_pin_id, eff.go_to_pin_map)

    def _on_rules_reload(self, msg: Empty) -> None:
        self._rule_engine.request_reload()
        self.get_logger().info("ルール再読み込みを要求されました")

    def _refresh_announce_settings(self) -> None:
        """発話間隔ファイル(announce_settings_file)の変更を反映する。例外は投げない。

        ファイルが有効なら その値(10〜600秒、ただしcooldown以上)、無ければROSパラメータの値。
        不正なファイルは直前の値を維持してWARNログ(同じ内容では1回だけ)。
        """
        reader = getattr(self, "_announce_reader", None)
        if reader is None:
            return
        try:
            value, message, changed = reader.poll()
            if message:
                if reader.load_error:
                    self.get_logger().warn(message)
                else:
                    self.get_logger().info(message)
            if changed:
                base = self._announce_param_interval if value is None else value
                self.announce_min_interval_seconds = max(
                    self.announce_cooldown_seconds, float(base)
                )
        except Exception as exc:  # 設定ファイルの不具合でノードを止めない
            self.get_logger().error(f"発話間隔設定の読み込みに失敗(直前の値を維持): {exc!r}")
        pr_reader = getattr(self, "_pr_reader", None)
        if pr_reader is not None:
            try:
                _cfg, _msg, changed = pr_reader.poll()
                if changed:
                    self.get_logger().info(
                        "出店紹介の設定を読み込みました(メニュー紹介の呼びかけ: "
                        + ("止める" if _pr_intro.menu_speech_suppressed(_cfg) else "出す") + ")"
                    )
            except Exception as exc:  # 出店紹介の設定の不具合で方針層を止めない(従来どおりメニュー紹介)
                self.get_logger().error(f"出店紹介の設定の読み込みに失敗(直前の設定を維持): {exc!r}")

    def _publish_robot_clock(self) -> None:
        """機体の時計(木が時刻条件に使っている現地時刻)を1Hzで配信する。例外は投げない。"""
        pub = self._robot_clock_pub
        if pub is None:
            return
        try:
            runner = getattr(self, "_bt", None)
            local_now = runner._clock() if runner is not None else _rules.make_clock(self._utc_offset_hours)()  # noqa: SLF001
            msg = String()
            msg.data = dumps(robot_clock_payload(local_now, time.time(), _uptime_sec(), self._utc_offset_hours))
            pub.publish(msg)
        except Exception as exc:  # noqa: BLE001 - 表示用の配信で方針層を止めない
            self.get_logger().warn(f"robot_clockの配信に失敗: {exc!r}")

    def _publish_boundary_override(self, request: Optional[dict]) -> None:
        """木のset_boundaryの要求を配信する(None=上書きなし)。毎秒送り続けることで、止まれば自動で元の範囲へ戻る。"""
        self._boundary_override_req = dict(request) if request else None
        pub = self._boundary_override_pub
        if pub is None:
            return
        msg = String()
        msg.data = dumps(boundary_override_message(self._boundary_override_req, stamp=time.time()))
        pub.publish(msg)

    def _publish_display_request(self, ts) -> None:
        """木のset_displayが出した、頭部ディスプレイの画面の表示モードの要求を配信する。例外は投げない。
        変化したとき1回 + スケジュール実行中はDISPLAY_REQUEST_REPEAT_SEC(5秒)ごとに再送する(受け手が途中から起動しても拾える)。
        スケジュールが実行中でなくなったときは、keep(要求なし)を1回だけ送る(受け手が元のモードへ戻す)。
        director自身はラズパイに触れない。受け手側にも「古い要求は無視」の期限(DISPLAY_REQUEST_TTL_SEC)がある。"""
        pub = self._display_request_pub
        if pub is None:
            return
        try:
            runner = getattr(self, "_bt", None)
            running = bool(ts is not None and runner is not None and runner.is_schedule)
            mode = (getattr(ts, "display", None) if running else None) or "keep"
            entry_id = self._schedule_active_entry_id if running else None
            key = (mode, entry_id)
            now = self.get_clock().now().nanoseconds / 1e9
            last = self._display_last_key
            if last is None and not running:
                return  # スケジュールを使っていない間は何も配信しない
            if key == last and not (running and now - self._display_last_pub_mono >= _schedule_plan.DISPLAY_REQUEST_REPEAT_SEC):
                return
            if key != last:
                self.get_logger().info(
                    f"頭部ディスプレイの画面の要求: {mode}(項目={entry_id})" if mode != "keep" else "頭部ディスプレイの画面の要求: なし(元のモードへ戻します)")
            msg = String()
            msg.data = dumps(_schedule_plan.display_request_message(mode, entry_id, time.time()))
            pub.publish(msg)
            self._display_last_key = key
            self._display_last_pub_mono = now
        except Exception as exc:  # noqa: BLE001 - 表示の配信で方針層を止めない
            self.get_logger().warn(f"display_requestの配信に失敗: {exc!r}")

    def _boundary_override_pending(self) -> bool:
        """木が出している範囲の要求が、ダッシュボード経由のtravel_boundaryにまだ反映されていないか。
        反映前(古い範囲のまま)に移動を始めないよう、ゴールを出すのは反映を確認してから(最大でも1〜2秒)。
        ダッシュボードが無い/止まっている時は反映されないので、その場で待機し続ける(安全側)。"""
        want = self._boundary_override_req is not None
        have = bool(self._boundary.get("override_active", False))
        return want != have

    def _on_schedule_command(self, msg: String) -> None:
        """ダッシュボードの「実行」「停止」ボタンの合図。ファイル(木)の書き込みは合図の前に済んでいる。"""
        data = loads(msg.data, None)
        cmd = data.get("cmd") if isinstance(data, dict) else None
        runner = getattr(self, "_bt", None)
        if runner is None or cmd not in ("run", "stop"):
            return
        if cmd == "run":
            runner.arm_schedule(True)
            runner.request_reload()
            self.get_logger().info("スケジュールの実行を開始します")
        else:
            was_running = bool(runner.user_active and runner.is_schedule)
            runner.arm_schedule(False)
            runner.request_reload()
            if not was_running:
                self._rules_tick()  # 動いていなかった: 何も止めない(状態の配信だけ更新)
                return
            # 即座にその場で待機し、方針で送っていたゴールを取り消して、ルール表の動作へ戻る。
            self._publish_stop()
            self._nav2_goal_pending = True  # 送っていたかもしれないゴールを、取り消しの共通手順(1回だけ)で必ず取り消す
            self._cancel_goal_once()
            self._policy_goal_outstanding = False
            self._policy_pin_goal_key = None
            self._policy_goal_done_activation = -1
            self._publish_boundary_override(None)
            if self._schedule_stop_hold_sec > 0:
                self._schedule_stop_hold_until = time.monotonic() + self._schedule_stop_hold_sec
            self.get_logger().info("スケジュールを停止しました(その場で待機してルール表の動作へ戻ります)")
        self._rules_tick()

    def _rules_tick(self) -> None:
        """1Hzでルール表を評価し、方針(effects)を更新する。例外時は現行動作へ戻す。"""
        self._publish_robot_clock()
        self._refresh_announce_settings()
        try:
            message = self._rule_engine.check_reload()
            if message:
                if self._rule_engine.load_error:
                    self.get_logger().warn(message)
                else:
                    self.get_logger().info(message)
            ctx = _rules.build_context(self._fresh_crowd_state(), self._menu_state)
            st = self._rule_engine.evaluate(ctx)
        except Exception as exc:  # 方針層の不具合でノードを止めない(現行動作へ戻す)
            self._rule_effects = _rules.NEUTRAL
            self._rule_active_id = None
            self._rule_policy = "normal"
            if not self._rule_error_logged:
                self._rule_error_logged = True
                self.get_logger().error(f"ルール評価に失敗しました(現行動作に戻します): {exc!r}")
            return
        self._rule_error_logged = False
        ts = self._bt_evaluate(ctx, st)
        try:
            self._publish_boundary_override(ts.boundary if ts is not None else None)
        except Exception as exc:  # noqa: BLE001 - 失敗したら上書きなし(元の範囲)のまま
            self._boundary_override_req = None
            self.get_logger().warn(f"boundary_overrideの配信に失敗: {exc!r}")
        self._publish_display_request(ts)
        if ts is not None:
            # 木の結果を使う(ルール表はRulesTableノードとして木の一部。ルール表の状態配信は従来どおり)。
            if ts.changed:
                self.get_logger().info(
                    "方針が切り替わりました(ビヘイビアツリー): ルール=%s 方針=%s params=%s 動作ノード=%s"
                    % (ts.active_rule_id, "+".join(ts.policies), ts.params, ts.active_leaves)
                )
            self._rule_effects = ts.effects
            self._rule_activation = ts.activation
            self._rule_active_id = ts.active_rule_id
            self._rule_policy = "+".join(ts.policies)
        else:
            if st.changed:
                self.get_logger().info(
                    "方針が切り替わりました: ルール=%s(%s) 方針=%s params=%s"
                    % (st.active_rule_id, st.active_rule_name, st.policy, st.params)
                )
            self._rule_effects = st.effects
            self._rule_activation = st.activation
            self._rule_active_id = st.active_rule_id
            self._rule_policy = st.policy
        hold_until = self._schedule_stop_hold_until
        if hold_until is not None:
            if ts is None and time.monotonic() < hold_until:
                # スケジュール停止の直後: 数秒はその場で待機(ルール表の動作へ戻る前の落ち着き)
                self._rule_effects = _rules.effects_for(("stay",), {})
                self._rule_activation = self._stop_hold_activation
                self._rule_policy = "stay"
            else:
                self._schedule_stop_hold_until = None
        msg = String()
        msg.data = dumps(st.to_json_dict())
        self._rules_state_pub.publish(msg)

    def _bt_safety(self) -> dict:
        """木の条件ノード(robot_state)とライブ表示用の、安全層の成立状況(読むだけ)。"""
        out = {}
        for key, attr in (
            ("estop", "_estop_active"),
            ("manual", "_manual_override"),
            ("survey", "_survey_hold"),
            ("near_person", "_near_person_too_close"),
            ("out_of_bounds", "_out_of_bounds"),
            ("keepout", "_in_keepout_zone"),
            ("camera_obstacle", "_camera_obstacle_ahead"),
            ("mapping", "_mapping_or_exploring"),
        ):
            try:
                out[key] = bool(getattr(self, attr))
            except Exception:  # 読めない時は「成立していない」扱い(安全分岐自体は_tickが別に判定する)
                out[key] = False
        return out

    def _bt_pin_ok(self, pin_id, pin_map=None) -> bool:
        """木の条件ノード(pin_available)用: そのピンが今の地図にあり行ける状態か。"""
        payload = getattr(self, "_pins_payload", None)
        rx = getattr(self, "_pins_rx_time", None)
        if payload is None or rx is None:
            return False
        if (self.get_clock().now() - rx).nanoseconds / 1e9 > getattr(self, "pins_stale_sec", 30.0):
            return False
        pin, _reason = resolve_pin(payload, pin_id, pin_map)
        return pin is not None

    def _bt_evaluate(self, ctx, st):
        """ビヘイビアツリーを1tick評価して状態を配信する。木の結果を使う時だけTreeStateを返す
        (ファイル無し/不正/無効/評価失敗はNone=従来のルール表の結果をそのまま使う)。例外は投げない。"""
        runner = getattr(self, "_bt", None)
        if runner is None:
            return None
        try:
            message = runner.check_reload()
            if message:
                if runner.load_error:
                    self.get_logger().warn(message)
                else:
                    self.get_logger().info(message)
            safety = self._bt_safety()
            ts = runner.evaluate(ctx, st, safety, self._bt_pin_ok)
            used = runner.user_active
            msg = String()
            status = runner.status_dict(ts, safety, getattr(self, "_prev_tick_mode", None), used)
            # 【2026-10-23】スケジュールの状態(実行中か、いまの項目、次の項目の時刻)。スケジュール由来の木でなければ中身は空。
            status["schedule"] = _schedule_plan.runtime_state(
                runner.doc if runner.is_schedule else {}, ts.statuses if ts is not None else {},
                runner._clock(), bool(used and runner.is_schedule), bool(runner.schedule_armed),  # noqa: SLF001
            )
            status["schedule"]["boundary_override"] = (
                dict(ts.boundary) if ts is not None and ts.boundary else None)
            status["schedule"]["display_request"] = ts.display if ts is not None else None
            status["schedule"]["startup_pending"] = bool(getattr(runner, "startup_run_pending", False))  # 起動時の自動実行の条件待ち
            self._schedule_active_entry_id = status["schedule"].get("active_entry_id")
            msg.data = dumps(status)
            self._bt_status_pub.publish(msg)
        except Exception as exc:  # 木の不具合でノードを止めない(従来のルール表の結果に戻す)
            if not getattr(self, "_bt_error_logged", False):
                self._bt_error_logged = True
                self.get_logger().error(
                    f"ビヘイビアツリーの評価に失敗しました(ルール表のみの従来動作に戻します): {exc!r}"
                )
            return None
        self._bt_error_logged = False
        return ts if used else None

    def _scaled(self, value: float) -> float:
        """方針speed_scaleによる自律速度の上限係数(0..1、現状値を超えない)。"""
        return value * min(1.0, max(0.0, self._rule_effects.speed_scale))

    def _policy_hold_step(self, now, eff) -> str:
        """方針stay/announce_only/go_to_*の間の動作(自律の移動指令は出さない)。

        go_to_*は、方針が有効になってから1回だけ既存の入口ゴール(_publish_entrance_goal、
        境界クランプ・立入禁止判定込み)を送り、到着/タイムアウトまたは方針解除で
        取り消す。到着後は停止して待機する(stay相当)。
        """
        self._arrived_at_time = None
        self._reset_crowd_scan(now)
        if self._home_return_active:
            self._home_return_active = False
            self._home_return_start_time = None
        if eff.go_to is not None and self._boundary_override_pending():
            # スケジュールの「範囲の差し替え/解除」が走行範囲にまだ反映されていない間は、古い範囲で動き出さずに待つ
            self._publish_stop()
            return "POLICY_STAY"
        if eff.go_to == "pin":
            return self._policy_pin_step(now, eff)
        if eff.go_to is not None:
            if self._policy_goal_done_activation != self._rule_activation:
                sent = self._publish_entrance_goal()
                if sent is not None:  # True=送信、False=立入禁止等で送れない。Noneは再送待ち
                    self._policy_goal_done_activation = self._rule_activation
                    self._policy_goal_outstanding = bool(sent)
                    self._policy_goal_start_time = now
                    if not sent:
                        self._publish_stop()
                        return "POLICY_AT_ENTRANCE"
                return "POLICY_GO_ENTRANCE"
            if self._policy_goal_outstanding:
                pose = self._get_pose()
                arrived = (
                    pose is not None
                    and math.hypot(pose["x"] - self.entrance_x, pose["y"] - self.entrance_y)
                    <= self.approach_standoff_m
                )
                started = self._policy_goal_start_time
                timed_out = (
                    started is not None
                    and (now - started).nanoseconds / 1e9 >= self.return_home_timeout_sec
                )
                if not (arrived or timed_out):
                    return "POLICY_GO_ENTRANCE"
                self._policy_goal_outstanding = False
                self._cancel_goal_once()
            self._publish_stop()
            return "POLICY_AT_ENTRANCE"
        self._publish_stop()
        self._cancel_goal_once()
        return "POLICY_STAY"

    def _policy_pin_step(self, now, eff) -> str:
        """方針goto_pinの間の動作。ピンへのゴールを1回送り(5秒の再送スロットルは入口ゴールと共通)、
        到着/タイムアウト/方針解除で取り消して待機する。ピンが使えない間は走行せず停止する。
        安全系の分岐(上位)に到達した場合はここへ来ない。"""
        pin, reason = self._resolve_policy_pin(eff)
        if pin is None:
            self._policy_pin_goal_key = None
            self._policy_goal_outstanding = False
            self._note_pin_unavailable(eff, PIN_REASON_TEXT.get(reason, reason))
            self._publish_stop()
            self._cancel_goal_once()
            return "POLICY_PIN_UNAVAILABLE"
        key = (self._rule_activation, pin["id"], pin["x"], pin["y"], pin["yaw_deg"])
        if self._policy_pin_goal_key != key:
            # 新しい方針の開始、またはピンが移動/付け替えられた: 目的地を送り直す。
            self._policy_pin_goal_key = key
            self._policy_goal_done_activation = -1
        if self._policy_goal_done_activation != self._rule_activation:
            sent, why = self._publish_pin_goal(pin)
            if sent is None:  # 再送間隔内。次のtickで再試行
                return "POLICY_GO_PIN"
            if not sent:
                # 範囲外/立入禁止/範囲不明: 走行しない。done印は付けない(範囲設定が直れば次tickで送る)。
                self._note_pin_unavailable(eff, why)
                self._publish_stop()
                self._cancel_goal_once()
                return "POLICY_PIN_UNAVAILABLE"
            self._policy_goal_done_activation = self._rule_activation
            self._policy_goal_outstanding = True
            self._policy_goal_start_time = now
            self._policy_pin_logged = None
            return "POLICY_GO_PIN"
        if self._policy_goal_outstanding:
            pose = self._get_pose()
            arrived = (
                pose is not None
                and math.hypot(pose["x"] - pin["x"], pose["y"] - pin["y"]) <= self.approach_standoff_m
            )
            started = self._policy_goal_start_time
            timed_out = (
                started is not None
                and (now - started).nanoseconds / 1e9 >= getattr(self, "pin_goal_timeout_sec", 120.0)
            )
            if not (arrived or timed_out):
                return "POLICY_GO_PIN"
            self._policy_goal_outstanding = False
            self._cancel_goal_once()
        self._publish_stop()
        return "POLICY_AT_PIN"

    def _note_pin_unavailable(self, eff, why: str) -> None:
        """ピンへ行けない理由をWARNで1回だけ記録する(同じ理由では繰り返さない)。"""
        tag = f"{eff.go_to_pin_id}:{why}"
        if getattr(self, "_policy_pin_logged", None) != tag:
            self._policy_pin_logged = tag
            self.get_logger().warn(
                "方針goto_pin: ピン「%s」(id=%s)へは行きません(その場で待機): %s"
                % (eff.go_to_pin_name or "", eff.go_to_pin_id, why)
            )

    def _on_menu_state(self, msg: String) -> None:
        data = loads(msg.data, {})
        if isinstance(data, dict):
            self._menu_state = data

    def _on_crowd_state(self, msg: String) -> None:
        data = loads(msg.data, {})
        if isinstance(data, dict):
            self._crowd_state = data
            self._crowd_state_mono = time.monotonic()

    def _fresh_crowd_state(self) -> dict:
        """crowd_stateが、CROWD_STATE_STALE_SEC秒以内に届いていればそれを、届いていない(crowd_monitorが落ちた・途絶えた)なら{}を返す。
        【2026-10-21】以前は最後に受けた値(例: calmと人物target)が永久に使われ、いない人物へ接近し続けた(全体レビュー)。
        {}は「level=unknown・人物なし」と同じ扱い(未受信の起動直後と同じ=従来の挙動)。"""
        last = getattr(self, "_crowd_state_mono", None)
        if last is None:
            return self._crowd_state  # 受信(コールバック)を一度も通っていない値(未受信の既定{}、または単体テストが直接入れた値)
        if (time.monotonic() - last) > CROWD_STATE_STALE_SEC:
            return {}
        return self._crowd_state

    def _on_camera_density(self, msg: String) -> None:
        data = loads(msg.data, {})
        if isinstance(data, dict):
            self._camera_person_density = data

    def _on_boundary(self, msg: String) -> None:
        data = loads(msg.data, None)
        if not isinstance(data, dict):
            return
        # enabled=trueなのに範囲が読めない(キー欠落・NaN/Infinity・幅高さ不正)と、
        # _out_of_bounds/_clamp_to_boundary/_approach_targetが壊れる(2026-07-17
        # レビューで発見)。不正なpublishは無視して直前の有効な状態を保つ。
        # 【2026-10-06】新形式(center/width/height/yaw)・旧形式(min/max)の両対応。
        if boundary_message_update(data) is None:
            self.get_logger().warn(
                "travel_boundary: enabled=trueなのに範囲が不正/不足しています。無視します"
            )
            return
        self._boundary = data

    def _boundary_rect(self) -> Optional[dict]:
        """現在の走行可能範囲の回転矩形。未設定(enabled偽)はNone。
        enabledなのに読めない状態はNone(呼び出し側で_boundary_unreadableを併用)。"""
        if not self._boundary.get("enabled"):
            return None
        return rect_from_state(self._boundary)

    @property
    def _boundary_unreadable(self) -> bool:
        """enabledなのに矩形を読めない(通常は_on_boundaryが弾くため起きない)。安全側で扱う。"""
        return bool(self._boundary.get("enabled")) and self._boundary_rect() is None

    def _on_keepout_zones(self, msg: String) -> None:
        data = loads(msg.data, [])
        if not isinstance(data, list):
            return
        # 個々のゾーンに範囲キーが欠けているものが混ざっていても、そのゾーン
        # だけ無視して残りは使う(1件の不正な追加で全体が死なないように、
        # travel_boundaryより1段階寛容にしている)。
        valid = [
            z
            for z in data
            if isinstance(z, dict)
            and all(k in z for k in ("min_x", "max_x", "min_y", "max_y"))
        ]
        self._keepout_zones = valid

    def _point_in_keepout(self, x: float, y: float) -> bool:
        return any(
            z["min_x"] <= x <= z["max_x"] and z["min_y"] <= y <= z["max_y"]
            for z in self._keepout_zones
        )

    @property
    def _in_keepout_zone(self) -> bool:
        if not self._keepout_zones:
            return False
        pose = self._get_pose()
        if pose is None:
            return False
        return self._point_in_keepout(pose["x"], pose["y"])

    def _on_camera_obstacle(self, msg: String) -> None:
        data = loads(msg.data, {})
        if isinstance(data, dict):
            self._camera_obstacle_detected = bool(data.get("obstacle_detected", False))
            self._last_camera_obstacle_receive_time = self.get_clock().now()

    @property
    def _camera_obstacle_ahead(self) -> bool:
        if not self._camera_obstacle_detected or self._last_camera_obstacle_receive_time is None:
            return False
        elapsed = (
            self.get_clock().now() - self._last_camera_obstacle_receive_time
        ).nanoseconds / 1e9
        return elapsed <= self.camera_obstacle_timeout_sec

    def _on_near_person_stop(self, msg: Bool) -> None:
        self._near_person_stop_detected = bool(msg.data)
        self._last_near_person_stop_receive_time = self.get_clock().now()
        self._near_person_stale_warned = False

    def _on_recommend_enabled(self, msg: Bool) -> None:
        enabled = bool(msg.data)
        # 冪等: ダッシュボードは設定後5秒ごとに同じ値を再送する(director再起動後も反映するため)。
        # 同じ値なら何もしない(ログ・A/B記録を増やさない)。
        if enabled == self._recommend_enabled:
            return
        self._recommend_enabled = enabled
        self._recommend_updated_at = datetime.datetime.now().timestamp()
        self._engagement_log.record_mode_change(
            datetime.datetime.now().isoformat(timespec="seconds"), enabled
        )
        self.get_logger().info(
            f"A/Bテスト: 推奨発話を{'有効' if enabled else '無効'}にしました"
        )

    def _on_idle_wiggle_enabled(self, msg: Bool) -> None:
        enabled = bool(msg.data)
        if enabled == self._idle_wiggle_enabled:
            return
        self._idle_wiggle_enabled = enabled
        self._wiggle_direction = 1.0
        self._wiggle_phase_start_time = None
        self.get_logger().info(
            f"左右振動の待機動作を{'有効' if enabled else '無効'}にしました"
        )

    def _on_person_interaction_enabled(self, msg: Bool) -> None:
        enabled = bool(msg.data)
        if enabled == self._person_interaction_enabled:
            return
        self._person_interaction_enabled = enabled
        self.get_logger().info(
            f"人の方へ近づく動作を{'許可' if enabled else '止め'}ました(スイッチ)"
        )

    def _on_scan_enabled(self, msg: Bool) -> None:
        enabled = bool(msg.data)
        if enabled == self._scan_enabled:
            return
        self._scan_enabled = enabled
        self.get_logger().info(f"見回す動作(旋回・左右の揺動)を{'許可' if enabled else '止め'}ました(スイッチ)")

    def _on_face_enabled(self, msg: Bool) -> None:
        enabled = bool(msg.data)
        if enabled == self._face_enabled:
            return
        self._face_enabled = enabled
        # 【2026-11-04】監視範囲の基準(スイッチをONにしたときの車体の向き)。ONのたびに取り直す。位置が分かった最初のtickで確定する。
        self._face_ref_yaw = None
        self._face_sweep_dir = 1.0
        self.get_logger().info(f"人の方を向く動作を{'許可' if enabled else '止め'}ました(スイッチ)")

    def _on_person_candidates(self, msg: String) -> None:
        data = loads(msg.data, None)
        if isinstance(data, dict):
            self._person_candidates = data
            self._person_candidates_mono = time.monotonic()
            self._person_candidates_rx = self._now_sec()

    def _fresh_person_candidates(self, now_s=None, yaw=None) -> list:
        """直近(FACE_CANDIDATES_STALE_SEC秒以内)の候補を、融合した形(_cb.fuse_candidates)で返す。古い/無いなら[]。
        【2026-11-06】now_s/yaw(位置)が分かるときは、融合の前にカメラ検出を追跡で安定化する(_ct.CameraTrackBank)。位置が無ければ従来どおり。"""
        last = getattr(self, "_person_candidates_mono", None)
        data = getattr(self, "_person_candidates", None)
        if last is None or not isinstance(data, dict):
            return []
        if (time.monotonic() - last) > FACE_CANDIDATES_STALE_SEC:
            return []
        try:
            camera = data.get("camera") or []
            if self.face_camera_track_enabled and yaw is not None and now_s is not None:
                if self._camera_tracker is None:
                    self._camera_tracker = _ct.CameraTrackBank(
                        match_deg=self.face_camera_track_match_deg, confirm_n=self.face_camera_track_confirm_n,
                        hold_sec=self.face_camera_track_hold_sec, smooth_alpha=self.face_camera_track_alpha)
                raw = [c.get("bearing") for c in camera if isinstance(c, dict)]
                camera = [{"bearing": b} for b in self._camera_tracker.update(now_s, yaw, raw)]
            return list(_cb.fuse_candidates(data.get("lidar") or [], camera))
        except Exception:
            return []

    def _interaction_ok(self, kind: str, allowed: bool, explicit: bool) -> bool:
        """3つの動作(kind: "approach"=近づく / "scan"=見回す / "face"=向く)をしてよいか。方針で許可されていて、
        かつ指定がある(その動作のスイッチON、またはルール/木の明示)ときだけ True。オプトイン無効なら従来どおり allowed。"""
        if not allowed:
            return False
        if not getattr(self, "_require_interaction_opt_in", False):
            return True
        if kind == "scan":
            switch = getattr(self, "_scan_enabled", False)
        elif kind == "face":
            switch = getattr(self, "_face_enabled", False)
        else:
            switch = getattr(self, "_person_interaction_enabled", False)
        return bool(switch or explicit)

    def _person_interaction_ok(self, allowed: bool, explicit: bool) -> bool:
        """近づく動作のゲート(従来名。_interaction_ok("approach", ...) と同じ)。"""
        return self._interaction_ok("approach", allowed, explicit)

    def _reset_face(self) -> None:
        """向く動作の状態を捨てる(向く動作が選ばれなくなったときに1回)。"""
        was_active = self._face_active
        self._face_active = False
        self._face_aligned = False
        self._face_bearing = None
        if self._face_selector is not None:
            self._face_selector.reset()
        for st in (self._camera_tracker, self._tier_filter):
            if st is not None:
                st.reset()
        if was_active:
            self.get_logger().info("人の方を向く動作を終了しました")

    def _face_person_step(self, now, bearing: float) -> str:
        """その場で旋回して、人の多い方(目標方位bearing: ロボット正面を0・左が正[rad]。機体基準なのでTF不要)を向く。
        出す指令は旋回だけ(linearは出さない)。範囲の縁などで旋回できないときは止まって POLICY_NO_SCAN。"""
        if not self._boundary_motion_ok("rotate"):
            self._publish_stop()
            self._reset_face()
            return "POLICY_NO_SCAN"
        if not self._face_active:
            self.get_logger().info("人の方を向く動作を開始しました")
        self._face_active = True
        self._face_bearing = float(bearing)
        max_rps = self._scaled(self.face_turn_rps)
        angular, aligned_next = _cb.face_turn_command(
            float(bearing), max_rps=max_rps, aligned=self._face_aligned,
            min_rps=min(max_rps, self._scaled(self.face_turn_min_rps)),
        )
        self._face_aligned = bool(aligned_next)
        twist = Twist()
        twist.linear.x = self._hold_linear(float(angular))  # 位置保持が無効(既定)なら 0.0
        twist.angular.z = float(angular)
        self._publish_direct(twist)
        return "FACE_PERSON"

    def _face_sweep_step(self, now) -> Optional[str]:
        """人が誰もいない間、その場で、基準の向きの前180度の範囲を往復して見回す(旋回だけ。前進しない)。
        人が範囲に現れたら、次のtickで _face_person_step に切り替わる(1周してから向くのではなく、その時の候補で向く)。
        位置(向き)が分からない・範囲の縁などで旋回できないときは、止まって待つ。"""
        pose = self._get_pose()
        ref = getattr(self, "_face_ref_yaw", None)
        if pose is None or ref is None:
            return None  # 向きが分からない: 範囲を決められないので、見回さない(呼び出し側は、ほかの動作へ進む)
        if not self._boundary_motion_ok("rotate"):
            self._publish_stop()
            self._reset_face()
            return "POLICY_NO_SCAN"
        self._face_sweep_dir = _cb.sweep_direction(pose["yaw"], ref, getattr(self, "_face_sweep_dir", 1.0))
        twist = Twist()
        twist.angular.z = float(self._face_sweep_dir * self._scaled(self.face_turn_rps))
        twist.linear.x = self._hold_linear(float(twist.angular.z))  # 位置保持が無効(既定)なら 0.0
        self._publish_direct(twist)
        return "FACE_SWEEP"

    # ------------------------------------------------------------------ 【2026-11-06】向く旋回中の位置保持
    # 純粋ロジックは position_hold.PositionHold。ここは「入力を集める」「安全条件を確かめる」だけ。既定は無効(linear=0.0のまま)。
    def _on_hold_params(self, params):
        from rcl_interfaces.msg import SetParametersResult

        for p in params:
            if p.name == "face_position_hold_enabled":
                self._face_hold_enabled = bool(p.value)
                if not self._face_hold_enabled and self._hold is not None:
                    self._hold.reset("無効化")
                self.get_logger().info(f"向く旋回中の位置保持を{'有効' if self._face_hold_enabled else '無効'}にしました")
            elif p.name.startswith("face_position_hold_"):
                self._hold = None  # 次に使うとき、新しい値で作り直す(学習したcは捨てる)
        return SetParametersResult(successful=True)

    def _hold_get(self):
        if self._hold is None:
            hp = _ph.HoldParams()
            try:
                gp = lambda n: float(self.get_parameter(n).value)  # noqa: E731
                hp.max_ff_mps = max(0.0, min(0.05, gp("face_position_hold_max_ff_mps")))
                hp.return_speed_mps = max(0.0, min(0.08, gp("face_position_hold_return_speed_mps")))
                hp.return_min_mps = min(hp.return_min_mps, hp.return_speed_mps)
                hp.start_m = max(0.02, gp("face_position_hold_start_m"))
                hp.stop_m = max(0.0, min(hp.start_m * 0.8, gp("face_position_hold_stop_m")))
                hp.max_total_m = max(hp.start_m, min(0.25, gp("face_position_hold_max_total_m")))
                hp.clear_fwd_m = hp.clear_back_m = max(0.6, gp("face_position_hold_clear_m"))
            except Exception:  # noqa: BLE001 - テスト用インスタンス(パラメータ無し)は既定値
                pass
            lg = self.get_logger()
            self._hold = _ph.PositionHold(hp, log=lambda lv, m: (lg.warn if lv == "warn" else lg.info)(m))
        return self._hold

    def _hold_ensure_subs(self) -> None:
        """/odom と /scan は、位置保持が有効になって初めて購読する(無効のときは負荷ゼロ)。"""
        if self._hold_subs_made:
            return
        self._hold_subs_made = True
        try:
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import LaserScan
            from rclpy.qos import qos_profile_sensor_data

            self.create_subscription(Odometry, "/odom", self._on_hold_odom, qos_profile_sensor_data)
            self.create_subscription(LaserScan, "/scan", self._on_hold_scan, qos_profile_sensor_data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"位置保持: /odom・/scan を購読できません({exc})。補正は出しません")

    def _on_hold_odom(self, msg) -> None:
        try:
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self._hold_odom = (self._now_sec(), float(msg.twist.twist.linear.x),
                               float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), yaw,
                               float(msg.twist.twist.angular.z))
        except Exception:  # noqa: BLE001
            self._hold_odom = None

    def _on_hold_scan(self, msg) -> None:
        self._hold_scan = (self._now_sec(), msg)

    def _hold_inputs(self, now_s: float):
        """(pos, v_odom, wz_odom, clear_fwd, clear_back)。古い/取れないものは None。NaN/Inf は None。"""
        stale = 0.5
        odom = self._hold_odom
        odom_ok = odom is not None and 0.0 <= (now_s - odom[0]) <= stale and all(math.isfinite(v) for v in odom[1:])
        v_odom = odom[1] if odom_ok else None
        wz_odom = odom[5] if odom_ok else None
        pose = self._get_pose()  # 地図上の位置(AMCL/SLAMのTF。古ければ None)
        if pose is not None and all(math.isfinite(float(pose[k])) for k in ("x", "y", "yaw")):
            pos = (float(pose["x"]), float(pose["y"]), float(pose["yaw"]), "map")
        elif odom_ok:
            pos = (odom[2], odom[3], odom[4], "odom")
        else:
            pos = None
        fwd = back = None
        sc = self._hold_scan
        if sc is not None and 0.0 <= (now_s - sc[0]) <= stale:
            m = sc[1]
            half = math.radians(30.0)
            args = (m.angle_min, m.angle_increment)
            lim = (float(m.range_min), float(m.range_max))
            fwd = _ph.sector_min_range(m.ranges, *args, 0.0, half, *lim)
            back = _ph.sector_min_range(m.ranges, *args, math.pi, half, *lim)
        return pos, v_odom, wz_odom, fwd, back

    def _hold_block_reason(self) -> str:
        """位置保持の補正を出してはいけない状況の理由(空文字=よい)。director側の安全条件。"""
        try:
            if self._near_person_too_close or getattr(self, "_near_exempt_active", False):
                return "near_person"  # 近接停止中、および近接停止を免除した旋回(_face_only_step)は従来どおり linear=0
            if self._estop_active:
                return "estop"
            if self._manual_override:
                return "manual"
            if self._survey_hold:
                return "survey"
            if self._mapping_or_exploring:
                return "mapping"
            if self._out_of_bounds or self._in_keepout_zone:
                return "out_of_bounds"
            if self._camera_obstacle_ahead:
                return "camera_obstacle"  # 旋回だけのときの免除は、前進/後退の補正には適用しない
            if getattr(self, "require_map_loaded", False) and self._get_pose() is None:
                return "map_not_loaded"
            if getattr(self, "_direct_grant", None) is not None and not self._direct_grant.ok(self._now_sec()):
                return "no_grant"
        except Exception:  # noqa: BLE001 - 判定できないときは出さない
            return "check_failed"
        return ""

    def _hold_boundary_ok(self, direction: float) -> bool:
        """動く先(前=+1/後ろ=-1)が走行可能範囲の外に出ないか。範囲が無ければ True。判定できなければ False。block_reasonの表示は変えない。"""
        boundary = getattr(self, "_boundary", None)
        if not isinstance(boundary, dict) or not boundary.get("enabled"):
            return True
        rect = self._boundary_rect()
        pose = self._get_pose()
        if rect is None or pose is None:
            return False
        return bool(_bmg.can_drive(rect, pose, direction))

    def _hold_linear(self, angular: float) -> float:
        """向く旋回の指令(angular)に足す linear.x[m/s]。無効(既定)・条件が外れたときは 0.0。例外でも旋回は止めない。"""
        if not self._face_hold_enabled:
            return 0.0
        try:
            hold = self._hold_get()
            self._hold_ensure_subs()
            now_s = self._now_sec()
            reason = self._hold_block_reason()
            pos, v_odom, wz_odom, fwd, back = self._hold_inputs(now_s)
            lin = hold.step(now_s, angular, allow=not reason, block_reason=reason, pos=pos,
                            v_odom=v_odom, wz_odom=wz_odom, clear_fwd=fwd, clear_back=back)
            if lin != 0.0 and not self._hold_boundary_ok(1.0 if lin > 0 else -1.0):
                hold.step(now_s, angular, allow=False, block_reason="boundary")
                lin = 0.0
            if lin != 0.0 and not math.isfinite(lin):
                lin = 0.0
            return float(lin)
        except Exception as exc:  # noqa: BLE001
            if not self._hold_err_logged:
                self._hold_err_logged = True
                self.get_logger().warn(f"位置保持でエラー({exc!r})。補正を出さずに旋回だけ続けます")
            return 0.0

    def _hold_release(self) -> None:
        """向く動作でなくなった(FACE_PERSON/FACE_SWEEP 以外のmode)tick: 位置保持の状態を捨てる。"""
        h = self._hold
        if h is not None and (h.anchor is not None or h.state != _ph.IDLE):
            h.reset("向く動作を終了")

    def _hold_status(self, mode: str) -> dict:
        """command JSON の face_hold(キーは常に出す)。"""
        if not self._face_hold_enabled:
            return {"enabled": False, "active": False, "drift_m": None, "ff_mps": 0.0, "anchor": None}
        h = self._hold
        st = h.status() if h is not None else {}
        lin = float(st.get("linear", 0.0)) if mode in ("FACE_PERSON", "FACE_SWEEP") else 0.0
        return {"enabled": True, "active": abs(lin) >= 0.002, "drift_m": st.get("drift_m"), "ff_mps": lin,
                "anchor": st.get("anchor"), "state": st.get("state"), "along_m": st.get("along_m"),
                "lateral_m": st.get("lateral_m"), "over_limit": st.get("over_limit", False),
                "reason": st.get("reason", ""), "c_left": st.get("c_left"), "c_right": st.get("c_right")}

    def _face_sweep_ok(self, eff) -> bool:
        """人がいない間の見回し(FACE_SWEEP)をしてよいか。向くが許可で、かつ明示(スイッチON/ルール・木の明示)があるとき。
        オプトイン無効(従来動作)では、向くのスイッチを見ずに許可になるので、見回しはしない(従来の揺動・見回しのまま)。"""
        return self._interaction_ok("face", eff.face_allowed, eff.face_explicit) and bool(
            getattr(self, "_face_enabled", False) or eff.face_explicit
        )

    def _map_not_loaded(self) -> bool:
        """地図を読み込んでいない(地図上の位置が、map_missing_grace_sec 秒以上、取れない)か。require_map_loaded が偽なら常に偽。"""
        if not getattr(self, "require_map_loaded", False):
            return False
        if self._get_pose() is not None:
            self._pose_missing_since = None
            return False
        now_s = self._now_sec()
        if self._pose_missing_since is None:
            self._pose_missing_since = now_s
        return (now_s - self._pose_missing_since) >= self.map_missing_grace_sec

    def _face_only_context(self, eff) -> bool:
        """いま「向く(その場の旋回だけ)」以外に、前へ進む動きが選ばれ得ない状況か。全部が成り立つときだけ True。
        向くが明示的にON(許可)、近づくが使えない、方針の移動(go_to)・待機・ホーム帰還・地図作成/探索・走行可能範囲の外・立入禁止エリアが無い。"""
        if not self._face_sweep_ok(eff):
            return False
        # 【2026-11-04】位置(向き)が取れないと、見回し(FACE_SWEEP)が使えず、揺動・見回し(前進を含む)に落ちるので、免除しない。
        if self._get_pose() is None:
            return False
        if self._person_interaction_ok(eff.approach_allowed, eff.approach_explicit):
            return False
        if eff.go_to is not None or eff.hold_position or self._home_return_active or self._mapping_or_exploring:
            return False
        return not (self._out_of_bounds or self._in_keepout_zone)

    def _near_person_exempt(self, eff) -> bool:
        """近接停止(NEAR_PERSON_HOLD)を、向く(その場の旋回だけ)のために免除してよいか。全部が成り立つときだけ True。
        - _face_only_context(前進する動きが選ばれ得ない)
        - 向く用の候補メッセージの「全周で一番近い物までの距離 nearest_m」(脚の形でない物も含む。基準スキャンの壁・フレームは除く)が、
          直近 NEAR_CLEAR_WINDOW_SEC 秒に **新しく届いた** 測定すべてで基準以上(始める0.55 m / 続ける0.48 m。【2026-11-05】LiDARは高さ12 cmの平面だけなので余裕を持たせた)。新しい測定が
          NEAR_CLEAR_MIN_SAMPLES 回以上・NEAR_CLEAR_MIN_SPAN_SEC 秒にわたり、最後の測定が NEAR_CLEAR_MAX_AGE_SEC 秒以内であること。
          (同じ古いメッセージの繰り返しでは数えない。距離が確認できない・古いときは免除しない=停止のまま)"""
        ok = False
        try:
            if getattr(self, "face_turn_ignores_near_person", True) and self._face_only_context(eff):
                ok = self._near_clear_ok()
        finally:
            self._near_exempt_active = ok
            if not ok:
                self._near_exempt_entered = False
        return ok

    def _near_clear_ok(self) -> bool:
        """【2026-11-05】時計はROS時計に統一(受信時刻 _person_candidates_rx)。測定の時刻 = 受信時刻 - nearest_age_s(その距離の元になった
        /scan を受けてからの経過秒、payload生成時点)。検出ノードがタイマーで古いスキャンの nearest_m を出し直しても、測定の時刻が
        進まないので「新しい測定」に数えない。nearest_age_s が無い(古い検出ノード)・数でない・NaN・負・NEAR_CLEAR_MAX_AGE_SEC 超なら免除しない。"""
        data = getattr(self, "_person_candidates", None)
        rx = getattr(self, "_person_candidates_rx", None)
        now_s = self._now_sec()
        nearest = data.get("nearest_m") if isinstance(data, dict) else None
        age = data.get("nearest_age_s") if isinstance(data, dict) else None
        if (
            rx is None
            or (now_s - rx) > FACE_CANDIDATES_STALE_SEC
            or isinstance(nearest, bool)
            or not isinstance(nearest, (int, float))
            or not math.isfinite(float(nearest))
            or isinstance(age, bool)
            or not isinstance(age, (int, float))
            or not math.isfinite(float(age))
            or float(age) < 0.0
            or float(age) > NEAR_CLEAR_MAX_AGE_SEC
        ):
            self._near_clear_hist = []  # 距離を確認できない・測定が古い: 履歴も捨てる
            return False
        meas_t = float(rx) - float(age)  # その距離を測った(スキャンを受けた)時刻
        hist = [(t, r) for (t, r) in getattr(self, "_near_clear_hist", []) if now_s - t <= NEAR_CLEAR_WINDOW_SEC]
        if not hist or meas_t > hist[-1][0] + 1e-6:  # 測定の時刻が進んだときだけ、新しい測定として数える
            hist.append((meas_t, float(nearest)))
        self._near_clear_hist = hist
        if len(hist) < NEAR_CLEAR_MIN_SAMPLES or (hist[-1][0] - hist[0][0]) < NEAR_CLEAR_MIN_SPAN_SEC:
            return False
        if now_s - hist[-1][0] > NEAR_CLEAR_MAX_AGE_SEC:
            return False
        thr = NEAR_CLEAR_STAY_M if getattr(self, "_near_exempt_active", False) else NEAR_CLEAR_START_M
        return all(r >= thr for _t, r in hist)

    def _face_only_step(self, now) -> str:  # 必ず文字列を返す
        """近接停止を免除したときの動き: 向く(人がいれば)か、見回し(いなければ)。旋回だけで、前進は出さない。"""
        self._near_exempt_entered = True
        # 【2026-11-05】Nav2ゴールを送っていた可能性があるとき(_nav2_goal_pending)だけ取り消す。以前は入った最初のtickで必ず取り消し、
        # ルーターのゼロ速度(1.5秒)と旋回がぶつかっていた。
        self._cancel_goal_once()
        bearing = self._face_target_bearing(now)
        mode = self._face_person_step(now, bearing) if bearing is not None else self._face_sweep_step(now)
        self._reset_crowd_scan(now)
        if not isinstance(mode, str) or not mode:
            # 【2026-11-04】旋回できない(位置が取れない等)ときは None を返さず、止まって近接停止のまま待つ。
            self._publish_stop()
            return "NEAR_PERSON_HOLD"
        return mode

    def _camera_guard_exempt(self, eff) -> bool:
        """向く(その場の旋回)だけで動いているとき、カメラの障害物検知の停止を効かせない(前には進まないのでぶつからない)。
        前へ進む動きが選ばれ得る状況(近づく・方針の移動/待機・ホーム帰還・地図作成など)では免除しない(_face_only_context)。"""
        if not getattr(self, "camera_guard_off_in_place_turn", True):
            return False
        return self._face_only_context(eff)

    def _announce_facing_ok(self, eff, mode: str) -> bool:
        """announce の after_facing: 向く動作が許可されているときは、向く動作の間(FACE_PERSON/FACE_SWEEP)だけ依頼を出す。
        【2026-11-04変更】向き終わるのを待たない。向きを変えている間(旋回中)も、いつもと同じ間隔で呼びかける(間隔は _publish_announce_if_needed)。
        向く動作が許可でない(使えない)ときは従来どおり出す(永久に黙らないため)。接近中(APPROACH_*)も従来どおり。"""
        if not getattr(eff, "announce_after_facing", False):
            return True
        if not self._interaction_ok("face", eff.face_allowed, eff.face_explicit):
            return True
        if mode.startswith("APPROACH"):
            return True
        return mode in ("FACE_PERSON", "FACE_SWEEP")

    def _face_target_bearing(self, now) -> Optional[float]:
        """直近の候補から、向くべき方位[rad](ロボット正面0・左が正)を決める。候補が無ければ None。"""
        if self._face_selector is None:
            self._face_selector = _cb.FaceBearingSelector()
        now_s = now.nanoseconds / 1e9
        pose = self._get_pose()
        cands = self._fresh_person_candidates(now_s, pose["yaw"] if pose is not None else None)
        # 【2026-11-04】監視範囲: スイッチをONにしたときの車体の向きの前180度。範囲の外の人は対象にしない。
        # 位置(向き)が分からないときは絞れないので、従来どおり全部を対象にする。
        if pose is not None and getattr(self, "_face_ref_yaw", None) is None:
            self._face_ref_yaw = float(pose["yaw"])
        ref = getattr(self, "_face_ref_yaw", None)
        # 【2026-11-05】機体の枠(LiDAR中心から近すぎる候補。四隅の支柱など)は人ではないので、向く対象にしない。
        if cands and self.body_range_m > 0:
            cands = [c for c in cands if not _cb.is_body_candidate(c, self.body_range_m)]
        if cands and pose is not None and ref is not None:
            # 【2026-11-05】今追っている人は、範囲の縁で出入りしても対象に残す(縁のヒステリシス)。向く先がちらつかないように。
            last_world = getattr(self, "_face_last_world", None) if self._face_active else None
            cands = [c for c in cands
                     if _cb.in_monitor_range_sticky(c.get("bearing"), pose["yaw"], ref, last_world)]
        elif cands and pose is None:
            # 【2026-11-05】位置が取れないときは基準の向きが分からないので、機体基準で正面±90度だけにする(見回しはしないが、人がいれば向く)。
            cands = [c for c in cands if _cb.in_front_half(c.get("bearing"))]
        if self.face_tier_enabled and pose is not None and (cands or self._tier_filter is not None):
            # 【2026-11-06】階層化: カメラ確認済みの人(階層1)があれば階層1だけ。無いときだけ、LiDARだけの物(階層2)
            if self._tier_filter is None:
                self._tier_filter = _ct.TierFilter(tier2_after_sec=self.face_tier2_after_sec)
            cands = self._tier_filter.update(now_s, pose["yaw"], cands)
        bearing = self._face_selector.update(now_s, cands, yaw=(pose["yaw"] if pose is not None else None)) if cands else None
        if bearing is not None:
            self._face_last_seen = now_s
            self._face_last_rel = float(bearing)
            # 位置が分かれば、見えた方位を「世界の向き」で覚える(旋回しても目標がずれない)
            self._face_last_world = (pose["yaw"] + float(bearing)) if pose is not None else None
            return bearing
        # 候補が消えた: 猶予の間は直前の向きを保つ。猶予を過ぎたら止める。
        last = getattr(self, "_face_last_seen", None)
        if last is not None and (now_s - last) <= FACE_LOSS_GRACE_SEC:
            world = getattr(self, "_face_last_world", None)
            if world is not None and pose is not None:
                return math.atan2(math.sin(world - pose["yaw"]), math.cos(world - pose["yaw"]))
            # 【2026-11-05】位置が無い間は旋回して機体基準の方位がずれても分からない。古い機体基準の方位で回り続けないよう、旋回を止める(0=向き済み)。
            return 0.0 if getattr(self, "_face_last_rel", None) is not None else None
        self._face_last_seen = None
        self._face_selector.reset()
        return None

    def _on_force_demo(self, msg: Bool) -> None:
        if not bool(msg.data):
            return
        # 同一人物クールダウンを解除する。今LiDAR/カメラで検知している
        # 人物がいれば、次のtickで通常の接近ロジックがそのまま働く。
        self._recent_serve_position = None
        self._recent_serve_time = None
        # 【2026-08-06修正】ホーム帰還中(接客直後、まさにデモボタンを押したく
        # なる場面)だと_home_return_activeがcrowd_level=="calm"の分岐より
        # 優先されてしまい、return_home_timeout_sec(既定30秒)が経過するまで
        # このトリガーが実質無視されていた(ダッシュボードは即時接近するかの
        # ような文言を返すのに実際は最大30秒待たされる不整合があった)。
        # デモ用トリガーは帰還中でも即座に次tickの接近ロジックへ戻す。
        self._home_return_active = False
        self._home_return_start_time = None
        self.get_logger().info("デモ用トリガー: 同一人物クールダウンを解除しました")

    @property
    def _near_person_too_close(self) -> bool:
        if self._last_near_person_stop_receive_time is None:
            return False  # 一度も受信していない: 近接停止なしの構成(従来どおり)
        elapsed = (
            self.get_clock().now() - self._last_near_person_stop_receive_time
        ).nanoseconds / 1e9
        stale_sec = float(getattr(self, "near_person_stale_sec", 2.0))
        if stale_sec > 0 and elapsed > stale_sec:
            # 【2026-11-04】受信途絶: 安全側(止まる)。警告は途絶ごとに1回だけ。
            if not getattr(self, "_near_person_stale_warned", False):
                self._near_person_stale_warned = True
                self.get_logger().warn(
                    f"near_person_stop の受信が{stale_sec:.1f}秒を超えて途絶えました(person_target_detectorが止まった可能性)。安全側で止まります"
                )
            return True
        if not self._near_person_stop_detected:
            return False
        return elapsed <= self.near_person_stop_timeout_sec

    @property
    def _out_of_bounds(self) -> bool:
        """自律動作を止めるべきか(走行可能範囲の外に出ているか)を判定する。

        位置が分からない場合(TF未取得、SLAM/localization未起動等)は判定
        しようがないため、制限しない(範囲内扱い)。この機能自体が位置推定
        前提のものなので、なりゆきに任せるほかない。
        """
        if not self._boundary.get("enabled"):
            return False
        pose = self._get_pose()
        if pose is None:
            return False
        rect = self._boundary_rect()
        if rect is None:
            return True  # enabledなのに矩形が読めない: 安全側(自律停止)
        return not point_in_rect(pose["x"], pose["y"], rect)

    def _clamp_to_boundary(self, x: float, y: float) -> tuple:
        """点を走行可能範囲(回転矩形)の内側へ射影する。範囲内ならそのまま返す。"""
        if not self._boundary.get("enabled"):
            return x, y
        rect = self._boundary_rect()
        if rect is None:
            return x, y  # 呼び出し側(_publish_entrance_goal等)が_boundary_unreadableで先に止める
        # 【2026-11-02】縁ぎりぎりへ向かうと、到着時の旋回で機体の角が出る。目的地は、縁から旋回に必要なぶん内側へ。
        return clamp_point_to_rect(x, y, rect, margin=_bmg.GOAL_MARGIN_M)

    def _boundary_motion_ok(self, kind: str) -> bool:
        """directorが直接/cmd_velへ出す動き(見回しの旋回・直進、揺動、直接接近)が、走行可能範囲の外へ出ないかを、
        動く前に計算して確かめる。True=動いてよい。範囲が未設定なら常にTrue(従来どおり)。
        範囲があるのに位置が分からない・範囲が読めないときは、安全側でFalse(動かない)。
        kind: "rotate"(その場旋回。機体の外接円+余白が範囲の内側) / "drive"(前進。先読みの距離まで機体の4隅が内側)。"""
        boundary = getattr(self, "_boundary", None)
        if not isinstance(boundary, dict) or not boundary.get("enabled"):
            return True
        rect = self._boundary_rect()
        if rect is None:
            self._boundary_block_reason = "範囲が読めない"
            return False
        pose = self._get_pose()
        if pose is None:
            self._boundary_block_reason = "位置が分からない"
            return False
        ok = _bmg.can_rotate_in_place(rect, pose) if kind == "rotate" else _bmg.can_drive(rect, pose, 1.0)
        if not ok:
            self._boundary_block_reason = "範囲の縁に近く、旋回すると機体が出る" if kind == "rotate" else "このまま進むと範囲の外へ出る"
        return ok

    def _on_hardware_estop(self, msg: Bool) -> None:
        self._hardware_estop = bool(msg.data)

    def _on_soft_estop(self, msg: Bool) -> None:
        self._soft_estop = bool(msg.data)

    def _on_map_building_active(self, msg: Bool) -> None:
        self._map_building_active = bool(msg.data)

    def _on_survey_active(self, msg: Bool) -> None:
        self._survey_active = bool(msg.data)
        self._last_survey_active_time = self.get_clock().now()

    @property
    def _survey_hold(self) -> bool:
        """アンケート回答中(最後の受信がtrueで、配信途絶で失効していない)の間True。"""
        if not self._survey_active or self._last_survey_active_time is None:
            return False
        elapsed = (
            self.get_clock().now() - self._last_survey_active_time
        ).nanoseconds / 1e9
        return elapsed <= self._survey_active_timeout_sec

    def _on_frontier_status(self, msg: String) -> None:
        data = loads(msg.data, {})
        if not isinstance(data, dict):
            return
        self._frontier_enabled = bool(data.get("enabled", False))
        self._last_frontier_status_time = self.get_clock().now()

    @property
    def _mapping_or_exploring(self) -> bool:
        """地図作成(SLAM)中、または自動探索が有効な間True。未受信ならFalse。"""
        if self._map_building_active:
            return True
        if not self._frontier_enabled or self._last_frontier_status_time is None:
            return False
        elapsed = (
            self.get_clock().now() - self._last_frontier_status_time
        ).nanoseconds / 1e9
        return elapsed <= self._frontier_status_timeout_sec

    def _on_manual_override(self, msg: Bool) -> None:
        self._manual_override_raw = bool(msg.data)
        if self._manual_override_raw:
            self._last_manual_override_time = self.get_clock().now()

    def _on_speed_override(self, msg: String) -> None:
        data = loads(msg.data, {})
        if not isinstance(data, dict):
            return
        # 入力ミスで暴走・停止しないよう範囲を制限する(TeleopController.
        # set_max_speedと同じ考え方)。
        if "linear_mps" in data:
            try:
                # 上限はAutoSpeedController(dashboard_common.py)と揃えている
                # (MAX_DRIVE_LINEAR_MPS)。ここだけ違う値だと、ダッシュボードの
                # 設定値と実際の動作速度が食い違う(2026-07-16レビューで発見)。
                self.approach_speed_mps = max(
                    0.01, min(MAX_DRIVE_LINEAR_MPS, float(data["linear_mps"]))
                )
            except (TypeError, ValueError):
                self.get_logger().warn(f"behavior_speed_override: 不正なlinear_mps: {data.get('linear_mps')!r}")
        if "angular_rps" in data:
            try:
                self.approach_turn_rps = max(0.01, min(3.0, float(data["angular_rps"])))
            except (TypeError, ValueError):
                self.get_logger().warn(f"behavior_speed_override: 不正なangular_rps: {data.get('angular_rps')!r}")

    @property
    def _estop_active(self) -> bool:
        return self._hardware_estop or self._soft_estop

    @property
    def _manual_override(self) -> bool:
        """一定時間trueが再送されなければ自動的に失効する手動運転フラグ。

        送信側が明示的にfalseを送らなくても(通信断・テストコマンドの
        送り忘れ等)、一定時間で自律の安全ロジックへ戻れるようにするため。
        """
        if not self._manual_override_raw or self._last_manual_override_time is None:
            return False
        elapsed = (self.get_clock().now() - self._last_manual_override_time).nanoseconds / 1e9
        return elapsed <= self.manual_override_timeout_sec

    def _rotating_merit_text(self) -> str:
        """残数差が出るまでの間、メニューのメリットを順番に案内する。"""
        if not self.menu_merits:
            return self.generic_initial_text

        now = self.get_clock().now()
        elapsed = (now - self._last_merit_change_time).nanoseconds / 1e9
        if elapsed >= self.announce_min_interval_seconds:
            self._merit_rotation_index += 1
            self._last_merit_change_time = now

        names = list(self.menu_merits.keys())
        name = names[self._merit_rotation_index % len(names)]
        return self.menu_merits[name]

    def _announce_text(self) -> str:
        promote_item = self._menu_state.get("promote_item")
        phase = self._menu_state.get("phase", "initial")
        if self._recommend_enabled and phase == "induction" and promote_item:
            try:
                return self.induction_template.format(item=promote_item)
            except (KeyError, ValueError, IndexError) as exc:
                # 起動時の自己テストを抜けてもなお失敗するケースへの多重防御。
                self.get_logger().error(f"induction_templateの適用に失敗: {exc}")
                return self.generic_initial_text
        return self._rotating_merit_text()

    def _current_menu_items_for_ai(self) -> list:
        """AIへ渡すメニュー一覧を組み立てる。

        残数管理(inventory_manager、iPadの初期食数入力)で実際に入力された
        メニュー名を優先する。nokosahen.yamlにそのメニュー名のメリット文が
        設定されていればそれを添え、無ければ空文字のまま渡す(AIには名前だけ
        伝わり、メリットは自分で考えることになる)。まだ食数が未入力
        (itemsが空)の間だけ、従来通りnokosahen.yaml設定のメニュー一覧に
        フォールバックする。
        """
        entered_items = self._menu_state.get("items") or []
        names = [
            str(item.get("name", "")).strip()
            for item in entered_items
            if isinstance(item, dict) and str(item.get("name", "")).strip()
        ]
        if names:
            return [
                {"name": name, "merit": self.menu_merits.get(name, "")}
                for name in names
            ]
        # 【2026-10-02変更】食数が未入力(メニュー未登録)の間はAIに材料を渡さない。
        # 以前はnokosahen.yamlの既定メニューへフォールバックしていたが、実際には
        # 出していないメニューを案内してしまううえ、AIが「○○定食」と作り話をする
        # 原因になった。空にするとvoice_bridgeがfallback_textを話す。
        return []

    def _build_announce_context(self, fallback_text: str) -> dict:
        """AIに実際の発話文面を考えさせるための材料をまとめる。

        残数カウントがまだ機能していない間(Stage2等、phaseが常にinitialの
        まま)はメニュー一覧+メリット文だけを渡す。実際に残数差が出て
        `induction`へ切り替わったら(完成版で決済カメラ/手動カウントが機能
        している状況)残数状況も追加で渡す。AI側が使えない/失敗した場合は
        fallback_text(このクラスの既存ルールベースのテキスト)をそのまま話す。
        """
        phase = self._menu_state.get("phase", "initial")
        promote_item = self._menu_state.get("promote_item")
        context: dict = {
            "phase": phase,
            "menu_items": self._current_menu_items_for_ai(),
            "fallback_text": fallback_text,
        }
        if self._recommend_enabled and phase == "induction" and promote_item:
            context["promote_item"] = promote_item
            context["remaining_status"] = [
                {
                    "name": item.get("name"),
                    "remaining": item.get("remaining"),
                    "initial": item.get("initial"),
                }
                for item in self._menu_state.get("items", [])
                if isinstance(item, dict) and item.get("name")
            ]
        return context

    def _sales_speech_suppressed(self) -> bool:
        """【B9】アンケート中・近接停止・地図作成/探索中のどれかが立っている間、営業の呼びかけの依頼を出さない。
        状況の一覧は announce_events.SALES_SUPPRESS_SITUATIONS(announce_nodeの状況アナウンスと共通)。
        パラメータ suppress_sales_while_announcing=false で無効化(従来どおり)。"""
        pr_reader = getattr(self, "_pr_reader", None)
        if pr_reader is not None and _pr_intro.menu_speech_suppressed(pr_reader.config):
            return True  # 出店紹介が選ばれている間は、メニュー紹介の呼びかけ(接近中も含む)を出さない
        if not self._suppress_sales_while_announcing:
            return False
        return bool(_announce_events.sales_speech_blocked({
            "survey": self._survey_hold,
            # 【2026-11-04】近接停止を免除して向いている間(_near_person_exempt)は、呼びかけを止めない(旋回中も同じ間隔で呼びかける)
            "near_person_stop": self._near_person_too_close and not getattr(self, "_near_exempt_active", False),
            "mapping": self._map_building_active,
            "exploring": self._mapping_or_exploring,
        }))

    def _publish_announce_if_needed(self, text: str, min_interval: Optional[float] = None) -> None:
        now = self.get_clock().now()
        elapsed = (now - self._last_announce_time).nanoseconds / 1e9
        interval = (
            self.announce_min_interval_seconds
            if min_interval is None
            else max(self.announce_cooldown_seconds, float(min_interval))
        )
        # 【2026-10-02修正】内容が変わった/同じに関わらず、最短間隔(既定60秒)
        # 未満なら新たな発話依頼を出さない(旧: 同一文面でもcooldown=8秒ごと、
        # メリット文のローテーションで文面が変わると毎回依頼していた)。
        # 起動後の最初の1回だけは即時に出す。
        if self._last_announce_time_set and elapsed < interval:
            return
        self._last_announce_time_set = True
        msg = String()
        msg.data = text
        self._announce_pub.publish(msg)

        context_msg = String()
        context_msg.data = dumps(self._build_announce_context(text))
        self._announce_context_pub.publish(context_msg)

        self._last_announce = text
        self._last_announce_time = now

    def _publish_stop(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    def _publish_cancel_goal(self) -> None:
        msg = Bool()
        msg.data = True
        self._cancel_goal_pub.publish(msg)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_user_cancel(self, msg: Bool) -> None:
        if msg.data:
            if self._user_cancel is None:
                self._user_cancel = UserCancelHold(10.0)
            self._user_cancel.on_user_cancel(self._now_sec())
            self._nav2_goal_pending = False  # ルーターが取り消す。自分の取り消しは重ねない

    def _goals_suppressed(self) -> bool:
        """ユーザー中止の直後(user_cancel_hold_sec秒)は、目的地を出さない。"""
        return self._user_cancel is not None and self._user_cancel.active(self._now_sec())

    def _on_motion_grant(self, msg: String) -> None:
        if self._direct_grant is None:
            self._direct_grant = GrantTracker("director_direct", 1.0, True)
        self._direct_grant.on_message(msg.data, self._now_sec())

    def _publish_direct(self, twist: Twist) -> bool:
        """directorが直接/cmd_velへ出す速度(見回し・揺動・直接接近)。arbiterの許可がないときは出さない。"""
        if self._direct_grant is not None and not self._direct_grant.ok(self._now_sec()):
            return False
        self._cmd_vel_pub.publish(twist)
        return True

    def _hold_cancel(self, mode: str) -> None:
        """HOLD系のNav2ゴール取り消し。切り替わった最初の1回だけ(hold_cancel_once=falseなら毎tick)。"""
        if self._hold_edge is None:  # __init__を通さずに作った(テスト用の)インスタンスでも動くよう、遅延生成
            self._hold_edge = HoldCancelEdge(True)
        if self._hold_edge.should_cancel(mode):
            self._nav2_goal_pending = False
            self._publish_cancel_goal()

    def _cancel_goal_once(self) -> None:
        """Nav2ゴールを送っていた可能性がある場合に限り、1回だけキャンセルを送る。

        接近/入口帰還(Nav2駆動)から、直接cmd_velで動くモード(見回し・
        アイドル揺動)や停止モードへ切り替わる時に呼ぶ。呼ばないと、
        送信済みのNav2ゴールが動き続けて/cmd_velと競合する。
        """
        if self._nav2_goal_pending:
            self._nav2_goal_pending = False
            self._publish_cancel_goal()

    def _publish_entrance_goal(self) -> Optional[bool]:
        """入口ゴールを送る。戻り値: True=送信、None=再送間隔(5秒)内で未送信、
        False=立入禁止エリア内のため送らない(既存呼び出し元は戻り値を使わない)。"""
        now = self.get_clock().now()
        if (now - self._last_goal_time).nanoseconds / 1e9 < 5.0:
            return None
        if self._goals_suppressed():
            return None  # ユーザー中止の直後は出し直さない
        # entrance_x/yは"map"座標系なので、走行可能範囲(同じ"map"座標系)へ
        # そのままクランプできる(entrance座標が範囲外に設定されていても、
        # 範囲の縁までしか呼びかけに行かせない)。
        if self._boundary_unreadable:
            return False  # 範囲が読めない状態ではゴールを送らない(安全側)
        entrance_x, entrance_y = self._clamp_to_boundary(self.entrance_x, self.entrance_y)
        if self._point_in_keepout(entrance_x, entrance_y):
            # 呼びかけ地点自体が立入禁止エリアの中にある。クランプでは
            # 内側の矩形を避けようがないため、このゴールは送らない。
            return False
        goal = PoseStamped()
        goal.header.stamp = now.to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = entrance_x
        goal.pose.position.y = entrance_y
        goal.pose.orientation.z = math.sin(self.entrance_yaw / 2.0)
        goal.pose.orientation.w = math.cos(self.entrance_yaw / 2.0)
        self._goal_pub.publish(goal)
        self._nav2_goal_pending = True
        self._last_goal_time = now
        return True

    def _publish_pin_goal(self, pin: dict):
        """地図ピンへのゴールを送る。戻り値は(結果, 理由): (True, "")=送信、(None, "")=再送間隔(5秒)内で
        未送信、(False, 理由)=安全のため送らない。

        入口ゴールと違い、走行可能範囲の外・立入禁止エリアの中のピンはクランプせず**拒否**する
        (ピンの位置を勝手に動かさない)。範囲が読めない状態も拒否。向きはピンの保存値。"""
        now = self.get_clock().now()
        if (now - self._last_goal_time).nanoseconds / 1e9 < 5.0:
            return None, ""
        if self._goals_suppressed():
            return None, ""  # ユーザー中止の直後は出し直さない
        if self._boundary_unreadable:
            return False, "走行可能範囲が読めないためゴールを送りません"
        if self._boundary.get("enabled"):
            rect = self._boundary_rect()
            if rect is None or not point_in_rect(pin["x"], pin["y"], rect):
                return False, "ピンが走行可能範囲の外にあります"
        if self._point_in_keepout(pin["x"], pin["y"]):
            return False, "ピンが立入禁止エリアの中にあります"
        yaw = math.radians(pin["yaw_deg"])
        goal = PoseStamped()
        goal.header.stamp = now.to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = float(pin["x"])
        goal.pose.position.y = float(pin["y"])
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self._goal_pub.publish(goal)
        self._nav2_goal_pending = True
        self._last_goal_time = now
        return True, ""

    def _approach_target(self, target: Optional[dict]) -> str:
        """人物へ接近するゴールを送る。戻り値は"arrived"/"navigating"/"idle"。

        target の x,y は person_target_detector の frame_id(既定"laser"、
        機体中心付近が原点)で表される座標なので、原点方向へ
        approach_standoff_m だけ手前に縮めたゴールを送ることで、人物の
        真上まで行かず会話に適した距離で止まらせる。
        """
        if target and "x" in target and "y" in target:
            x = float(target.get("x", 0.0))
            y = float(target.get("y", 0.0))
            distance = math.hypot(x, y)
            if distance <= self.approach_standoff_m:
                if self._person_outside_boundary(target):
                    # 範囲の外にいる人が会話距離にいる。立ち位置(今の場所)で人の方を向いてから止まる。
                    pose = self._get_pose()
                    status = self._face_person_at_boundary(
                        target, pose, (pose["x"], pose["y"]), self.get_clock().now()
                    )
                    if status is not None:
                        return status
                # 十分近い(会話に適した距離)ので新しいゴールは出さず停止する。
                self._publish_stop()
                self._cancel_goal_once()
                return "arrived"

            now = self.get_clock().now()
            elapsed = (now - self._last_approach_goal_time).nanoseconds / 1e9
            if elapsed < self.approach_goal_min_interval_sec:
                return "navigating"
            if self._goals_suppressed():
                return "navigating"  # ユーザー中止の直後は目的地を出し直さない(待機)

            scale = (distance - self.approach_standoff_m) / distance
            goal_x = x * scale
            goal_y = y * scale
            goal_frame = str(target.get("frame_id", "map"))
            yaw = math.atan2(y, x)

            if self._boundary.get("enabled"):
                # targetは機体基準(frame_id、既定"laser")の座標なので、走行可能
                # 範囲(map座標系)と比較するにはmapへ変換する必要がある
                # (2026-07-17レビューで発見: 接近ゴールには境界クランプが
                # 一切適用されておらず、人物次第で範囲外まで出て行けた)。
                pose = self._get_pose()
                if pose is not None:
                    world_x = pose["x"] + goal_x * math.cos(pose["yaw"]) - goal_y * math.sin(pose["yaw"])
                    world_y = pose["y"] + goal_x * math.sin(pose["yaw"]) + goal_y * math.cos(pose["yaw"])
                    clamped_x, clamped_y = self._clamp_to_boundary(world_x, world_y)
                    if clamped_x != world_x or clamped_y != world_y:
                        # 範囲外まで近づこうとしている。範囲の縁までしか
                        # 行かせない(mapフレームのゴールに切り替える)。
                        goal_frame = "map"
                        goal_x = clamped_x
                        goal_y = clamped_y
                        # 【2026-10-10変更】縁に着いたら人の方を向く(以前は送信時の機体の向き=人と無関係)。
                        person = self._target_world_xy(target)
                        yaw = self._bearing_or(clamped_x, clamped_y, person, pose["yaw"])
                        # 縁(立ち位置)に着いていれば、その場で人の方を向き終えるまで待って「到着」にする。
                        status = self._face_person_at_boundary(target, pose, (clamped_x, clamped_y), now)
                        if status is not None:
                            return status

            if self._keepout_zones:
                # 立入禁止エリアはmap座標系で設定されているため、goalが
                # まだ機体基準フレームのままなら変換してから判定する。
                pose = self._get_pose()
                if pose is not None:
                    if goal_frame == "map":
                        check_x, check_y = goal_x, goal_y
                    else:
                        check_x = pose["x"] + goal_x * math.cos(pose["yaw"]) - goal_y * math.sin(pose["yaw"])
                        check_y = pose["y"] + goal_x * math.sin(pose["yaw"]) + goal_y * math.cos(pose["yaw"])
                    if self._point_in_keepout(check_x, check_y):
                        # 接近ゴールが立入禁止エリアの中に入ってしまう。
                        # このゴールは送らずに停止する(接近を諦める)。
                        self._publish_stop()
                        self._cancel_goal_once()
                        return "idle"

            goal = PoseStamped()
            goal.header.stamp = now.to_msg()
            goal.header.frame_id = goal_frame
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y
            goal.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.orientation.w = math.cos(yaw / 2.0)
            self._goal_pub.publish(goal)
            self._nav2_goal_pending = True
            self._last_approach_goal_time = now
            return "navigating"

        if self.enable_direct_cmd_vel and self._boundary_motion_ok("drive"):
            twist = Twist()
            twist.linear.x = min(self._scaled(self.approach_speed_mps), DIRECT_DRIVE_MAX_MPS)
            twist.angular.z = self._scaled(self.approach_turn_rps)
            self._publish_direct(twist)
        return "idle"

    @staticmethod
    def _bearing_or(from_x: float, from_y: float, to_xy, fallback: float) -> float:
        """(from_x, from_y)から見たto_xyの方位[rad]。同じ点/不明ならfallback。"""
        if to_xy is None:
            return fallback
        dx, dy = to_xy[0] - from_x, to_xy[1] - from_y
        if math.hypot(dx, dy) < 1e-3:
            return fallback
        return math.atan2(dy, dx)

    def _person_outside_boundary(self, target: dict) -> bool:
        """走行可能範囲が有効で、人が範囲の外にいる(位置が分かる)ときだけTrue。範囲未設定はFalse=従来どおり。"""
        if not self._boundary.get("enabled"):
            return False
        rect = self._boundary_rect()
        if rect is None or self._get_pose() is None:
            return False
        world = self._target_world_xy(target)
        if world is None:
            return False
        return not point_in_rect(world[0], world[1], rect)

    def _face_person_at_boundary(self, target: dict, pose, stand_xy, now) -> Optional[str]:
        """範囲制限のため人の近くまで行けないとき、立ち位置stand_xy(範囲の縁へ射影した点)で人の方を向く。

        戻り値: None=まだ立ち位置に着いていない(呼び出し側が通常どおり目的地を送る)、
        "arrived"=着いて人の方を向いた(または向けないまま待つ上限を超えた)、
        "navigating"=着いたが向きがまだ(立ち位置で人の方位を向く目的地を送り、Nav2がその場で旋回する)。
        走行は全てNav2ゴール経由(自分でcmd_velは出さない)。安全層(近接停止・衝突監視・範囲外等)はそのまま効く。"""
        gx, gy = stand_xy
        if math.hypot(pose["x"] - gx, pose["y"] - gy) > self.boundary_stop_tolerance_m:
            self._boundary_face_since = None
            return None
        person = self._target_world_xy(target)
        bearing = self._bearing_or(gx, gy, person, pose["yaw"])
        err = math.atan2(math.sin(bearing - pose["yaw"]), math.cos(bearing - pose["yaw"]))
        # 途切れた(別のモードを挟んだ)後の古い開始時刻で、向きを試さず諦めないようにする。
        if (
            self._boundary_face_since is None
            or self._boundary_face_seen is None
            or (now - self._boundary_face_seen).nanoseconds / 1e9 > 3.0
        ):
            self._boundary_face_since = now
        self._boundary_face_seen = now
        waited = (now - self._boundary_face_since).nanoseconds / 1e9
        if abs(err) <= self.boundary_face_tolerance_rad or waited >= self.boundary_face_timeout_sec:
            self._publish_stop()
            self._cancel_goal_once()
            return "arrived"
        if self._keepout_zones and self._point_in_keepout(gx, gy):
            # 立ち位置が立入禁止エリアの中(射影では避けられない)。旋回の目的地も送らず、接近を諦めて停止する。
            self._publish_stop()
            self._cancel_goal_once()
            return "idle"
        elapsed = (now - self._last_approach_goal_time).nanoseconds / 1e9
        if elapsed >= self.approach_goal_min_interval_sec:
            goal = PoseStamped()
            goal.header.stamp = now.to_msg()
            goal.header.frame_id = "map"
            goal.pose.position.x = float(gx)
            goal.pose.position.y = float(gy)
            goal.pose.orientation.z = math.sin(bearing / 2.0)
            goal.pose.orientation.w = math.cos(bearing / 2.0)
            self._goal_pub.publish(goal)
            self._nav2_goal_pending = True
            self._last_approach_goal_time = now
        return "navigating"

    def _get_pose(self) -> Optional[dict]:
        """map->base_link のTF(実測)から現在位置・向きを取得する。

        slam_toolbox(LiDARスキャンマッチング)とホイールオドメトリの融合結果
        なので、コマンド速度×経過時間だけの見積もりよりも実際の回転量・
        移動量に近い。TFが引けない(SLAM/localizationが動いていない等)場合は
        Noneを返し、呼び出し側でコマンド値からの見積もりにフォールバックする。
        """
        if not self.tf_available:
            return None
        try:
            import rclpy.time

            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        # 【2026-11-05】最新の変換が古い(TFが固まった)ときは位置なし扱い。stamp が取れない形式(スタブ・静的変換の0)は検査しない。
        try:
            st = tf.header.stamp
            stamp_s = float(st.sec) + float(st.nanosec) / 1e9
            if stamp_s > 0.0 and (self._now_sec() - stamp_s) > POSE_STALE_SEC:
                return None
        except Exception:  # noqa: BLE001
            pass
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        return {"x": t.x, "y": t.y, "yaw": yaw}

    def _target_world_xy(self, target: dict) -> Optional[tuple]:
        """targetのx,y(機体基準、frame_id)をmap座標系へ変換する。

        TFが引けない場合はNone(呼び出し側はクールダウン判定を諦めて通常通り
        接近する=安全側ではなく機能維持側に倒す。人物への接近自体は他の
        安全策(近接停止・境界等)で別途保護されているため)。
        """
        pose = self._get_pose()
        if pose is None:
            return None
        x = float(target.get("x", 0.0))
        y = float(target.get("y", 0.0))
        world_x = pose["x"] + x * math.cos(pose["yaw"]) - y * math.sin(pose["yaw"])
        world_y = pose["y"] + x * math.sin(pose["yaw"]) + y * math.cos(pose["yaw"])
        return world_x, world_y

    def _target_on_cooldown(self, target: dict, now) -> bool:
        """直近に接近完了した人物とほぼ同じ位置(approach_cooldown_radius_m以内)
        の候補を、approach_cooldown_sec以内なら「クールダウン中」として扱う
        (2026-07-17追加、しつこい連続アプローチの防止)。
        """
        if self._recent_serve_position is None or self._recent_serve_time is None:
            return False
        elapsed = (now - self._recent_serve_time).nanoseconds / 1e9
        if elapsed > self.approach_cooldown_sec:
            return False
        world = self._target_world_xy(target)
        if world is None:
            return False
        dist = math.hypot(
            world[0] - self._recent_serve_position[0], world[1] - self._recent_serve_position[1]
        )
        return dist <= self.approach_cooldown_radius_m

    def _return_home_step(self, now) -> str:
        """接近・案内完了後、entrance_x/y/yawへ自動で戻る(2026-07-17追加)。

        _publish_entrance_goal()(既存、境界クランプ+5秒の再送スロットル済み)
        をそのまま再利用する。TFが引けず到達判定ができない場合は
        return_home_timeout_secで諦めて通常運転に戻る(安全に倒すため、
        帰還に失敗しても自律ロジック自体は止めない)。
        """
        pose = self._get_pose()
        if pose is not None:
            dist = math.hypot(pose["x"] - self.entrance_x, pose["y"] - self.entrance_y)
            if dist <= self.approach_standoff_m:
                self._home_return_active = False
                self._home_return_start_time = None
                self._publish_stop()
                self._cancel_goal_once()
                return "RETURN_HOME_ARRIVED"
        if self._home_return_start_time is not None:
            elapsed = (now - self._home_return_start_time).nanoseconds / 1e9
            if elapsed >= self.return_home_timeout_sec:
                self._home_return_active = False
                self._home_return_start_time = None
                # 帰還を諦めるので、送信済みの入口ゴールも取り消す。
                self._publish_stop()
                self._cancel_goal_once()
                return "RETURN_HOME_TIMEOUT"
        self._publish_entrance_goal()
        return "RETURN_HOME"

    def _reset_crowd_scan(self, now) -> None:
        self._scan_phase = "scanning"
        self._scan_phase_start_time = now
        self._scan_rotated_rad = 0.0
        self._scan_best_bearing: Optional[float] = None
        self._scan_best_count = 0
        self._scan_remaining_turn_rad = 0.0
        pose = self._get_pose()
        self._scan_start_pose = pose
        self._scan_last_pose = pose

    def _crowd_scan_step(self, now, dt: float) -> str:
        """LiDARで確定した人物(target)がまだいない間の「見回し」行動。

        旋回しながらカメラの人数密度(camera_person_density)を見て、
        一番人が多く映っていた瞬間の方位を覚えておく。1周見回し終えたら
        その方位へ向き直り、crowd_scan_approach_seconds秒/
        crowd_scan_approach_distance_mのどちらか早い方まで直進する
        (この間にLiDARが人物を確定できれば、次tickでAPPROACH_PERSONへ
        自然に引き継がれる)。crowd_levelが"crowded"に変われば_tick側の
        分岐が優先されるため、ここでは時間/距離の上限は安全策として持つ。

        回転量・移動量は、エンコーダだけの見積もり(コマンド値×経過時間)
        ではなく、_get_pose()(LiDARスキャンマッチング込みのTF)で実測してから
        「これだけ見回した/進んだ」と確定させる。TFが引けない場合のみ
        見積もりにフォールバックする。
        """
        pose = self._get_pose()

        # 【2026-11-02】範囲の外へ出ないことを、動く前に計算する(旋回=その場、直進=先読み)。出る恐れがあれば止めて待つ。
        if not self._boundary_motion_ok("drive" if self._scan_phase == "approaching" else "rotate"):
            self._publish_stop()
            self._reset_crowd_scan(now)
            return "POLICY_NO_SCAN"

        if self._scan_phase == "scanning":
            twist = Twist()
            twist.angular.z = self._scaled(self.crowd_scan_turn_rps)
            self._publish_direct(twist)

            if pose is not None and self._scan_last_pose is not None:
                dyaw = math.atan2(
                    math.sin(pose["yaw"] - self._scan_last_pose["yaw"]),
                    math.cos(pose["yaw"] - self._scan_last_pose["yaw"]),
                )
                self._scan_rotated_rad += abs(dyaw)
            else:
                self._scan_rotated_rad += self._scaled(self.crowd_scan_turn_rps) * dt
            self._scan_last_pose = pose

            density = self._camera_person_density
            if density:
                count = int(density.get("person_count", 0) or 0)
                bearing = density.get("avg_bearing_rad")
                if count > self._scan_best_count and bearing is not None:
                    self._scan_best_count = count
                    # 実測回転量(見回し開始時点を基準)にカメラのbearingを
                    # 加えて、絶対方位として覚える。
                    self._scan_best_bearing = self._scan_rotated_rad + float(bearing)
                self._camera_person_density = None  # 使い回し防止(1サンプル1回のみ評価)

            if self._scan_rotated_rad >= 2.0 * math.pi:
                if self._scan_best_bearing is not None and self._scan_best_count > 0:
                    # -pi..piに正規化し、最短方向へ向き直る回転量を求める。
                    self._scan_remaining_turn_rad = math.atan2(
                        math.sin(self._scan_best_bearing), math.cos(self._scan_best_bearing)
                    )
                    self._scan_phase = "turning"
                    self._scan_last_pose = pose
                else:
                    # 誰も見つからなければ見回しをやり直す。
                    self._reset_crowd_scan(now)
            return "CROWD_SCAN"

        if self._scan_phase == "turning":
            direction = 1.0 if self._scan_remaining_turn_rad >= 0 else -1.0
            twist = Twist()
            twist.angular.z = direction * self._scaled(self.crowd_scan_turn_rps)
            self._publish_direct(twist)

            if pose is not None and self._scan_last_pose is not None:
                consumed = math.atan2(
                    math.sin(pose["yaw"] - self._scan_last_pose["yaw"]),
                    math.cos(pose["yaw"] - self._scan_last_pose["yaw"]),
                )
            else:
                consumed = direction * self._scaled(self.crowd_scan_turn_rps) * dt
            self._scan_last_pose = pose
            self._scan_remaining_turn_rad -= consumed

            # 残り角が十分小さいか、符号が反転した(行き過ぎた)ら向き直り完了。
            overshoot = (direction > 0) != (self._scan_remaining_turn_rad >= 0)
            if abs(self._scan_remaining_turn_rad) <= math.radians(2.0) or overshoot:
                self._publish_stop()
                self._scan_phase = "approaching"
                self._scan_phase_start_time = now
                self._scan_start_pose = pose
                self._scan_last_pose = pose
            return "CROWD_TURN_TO_CROWD"

        # approaching
        elapsed = (now - self._scan_phase_start_time).nanoseconds / 1e9
        distance_travelled = 0.0
        if pose is not None and self._scan_start_pose is not None:
            distance_travelled = math.hypot(
                pose["x"] - self._scan_start_pose["x"], pose["y"] - self._scan_start_pose["y"]
            )
        if (
            distance_travelled >= self.crowd_scan_approach_distance_m
            or elapsed >= self.crowd_scan_approach_seconds
        ):
            self._reset_crowd_scan(now)
            return "CROWD_SCAN"
        twist = Twist()
        twist.linear.x = min(self._scaled(self.crowd_scan_drive_mps), DIRECT_DRIVE_MAX_MPS)
        self._publish_direct(twist)
        return "CROWD_APPROACH"

    def _idle_wiggle_step(self, now, dt: float) -> str:
        """CROWD_SCANの代わりに使う、センサー判定を一切見ない待機動作。

        カメラ・LiDARの情報を使わず、単純に左右へ1秒ずつ交互に回転する
        だけ(見回しではなく、その場で「動いている」ことを見せる演出用)。
        """
        if not self._boundary_motion_ok("rotate"):  # 【2026-11-02】範囲の縁に近い・位置不明なら揺れない
            self._publish_stop()
            self._wiggle_phase_start_time = None
            return "POLICY_NO_SCAN"
        if self._wiggle_phase_start_time is None:
            self._wiggle_phase_start_time = now
        elapsed = (now - self._wiggle_phase_start_time).nanoseconds / 1e9
        if elapsed >= 1.0:
            self._wiggle_direction *= -1.0
            self._wiggle_phase_start_time = now
        twist = Twist()
        twist.angular.z = self._wiggle_direction * self._scaled(self.crowd_scan_turn_rps)
        self._publish_direct(twist)
        return "IDLE_WIGGLE"

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = max(0.01, (now - self._last_tick_time).nanoseconds / 1e9)
        self._last_tick_time = now

        self._boundary_block_reason = ""  # 直接動作が範囲のために止められたときだけ、この tick の間に理由が入る
        crowd_state = self._fresh_crowd_state()  # 古い(途絶えた)crowd_stateは使わない
        crowd_level = crowd_state.get("level", "unknown")
        target = crowd_state.get("person_target")
        announce_text = self._announce_text()
        eff = self._rule_effects

        if eff.go_to is None and self._policy_goal_outstanding:
            # 方針が解除された: 方針で送った入口ゴールを取り消して停止する。
            self._policy_goal_outstanding = False
            self._publish_stop()
            self._cancel_goal_once()

        camera_exempt = False  # 【2026-11-04】カメラ障害物停止を、このtickで免除したか(下の後段ガード用)
        if self._estop_active:
            # 主電流が遮断されていて物理的に動けない間は、ナビゲーション
            # ゴールや接近指令を出しても無意味なので止める。発話だけ続ける。
            # ハード/ソフトどちらのestopでも最優先(手動運転中でも無視しない)。
            mode = "ESTOP_HOLD"
            self._publish_stop()
            self._reset_crowd_scan(now)
        elif self._manual_override:
            # ダッシュボードのWASD操作/地図クリックによる手動運転中。
            # ここではgoalもcmd_velも一切出さない(ダッシュボード側が/cmd_vel
            # または/goal_poseを直接publishするので、自律ロジックと競合させない)。
            mode = "MANUAL_OVERRIDE"
            self._reset_crowd_scan(now)
        elif self._survey_hold and not self._mapping_or_exploring:
            # 【2026-10-09追加】お客さんがラズパイ腹部画面のアンケートに回答中は、
            # 走行(cmd_vel)・新規ゴールを出さず、実行中のNav2ゴールも取り消して止まる。
            # 手動運転(上のMANUAL_OVERRIDE)とESTOPはこれより上で優先処理済み。
            # 解除後は通常動作へ戻る(HOLD前の目的地は再開しない=他のHOLDと同じ)。
            # 【2026-10-10変更】地図作成中/自動探索中(_mapping_or_exploring)はこの停止を適用しない。
            # 探索中はお客さんがいない前提で、画面に触れても探索を止めない(下のMAPPING_ACTIVE_HOLDへ進む。
            # directorは走行指令を出さず、探索のNav2ゴールも取り消さない)。ほかの安全は全て有効のまま。
            # 通常の自律走行(地図クリック・ピン・ルール/木)中は従来どおりここで止まる。
            mode = "SURVEY_ACTIVE_HOLD"
            self._publish_stop()
            self._hold_cancel(mode)
            self._reset_crowd_scan(now)
        elif self._near_person_too_close and bool(crowd_state) and self._near_person_exempt(eff):
            # 【2026-11-05】crowd_stateが途絶えている(センサ欠測)ときは免除せず、下の従来の近接停止にする。
            # 【2026-11-04】向く(その場の旋回だけ)の間は、近接停止を効かせない。免除の条件は _near_person_exempt。
            mode = self._face_only_step(now)
        elif self._near_person_too_close:
            # 【2026-07-17変更】以前はestopの次に優先し手動運転中も止めていたが、
            # 実機テストで「操作者がロボットのすぐそばにいるのが普通の手動操作中
            # まで動けなくなって不便」と判明したため、手動運転(上のMANUAL_OVERRIDE
            # 分岐)は対象外にした。境界/障害物ホールドと同じ「操作者が見ながら
            # 運転しているので自律側の安全策は要らない」という方針に合わせる。
            # 自律動作(見回し・接近・自動地図作成等)ではこれまで通り最優先で止める。
            mode = "NEAR_PERSON_HOLD"
            self._publish_stop()
            self._hold_cancel(mode)
            self._reset_crowd_scan(now)
        elif self._out_of_bounds:
            # ダッシュボードで設定した走行可能範囲の外に出ている間は、自律動作
            # (ゴール送信・見回し中の直進等)を止める。手動運転(WASD、上の
            # MANUAL_OVERRIDE分岐)は対象外(操作者が見ながら運転しているため)。
            mode = "OUT_OF_BOUNDS_HOLD"
            self._publish_stop()
            self._hold_cancel(mode)
            self._reset_crowd_scan(now)
        elif self._in_keepout_zone:
            # ダッシュボードの地図編集ページで設定した立入禁止エリアに入って
            # いる間は自律動作を止める。手動運転(WASD)は対象外(走行可能範囲と
            # 同じ設計方針、操作者が見ながら運転しているため)。
            mode = "KEEPOUT_ZONE_HOLD"
            self._publish_stop()
            self._hold_cancel(mode)
            self._reset_crowd_scan(now)
        elif self._camera_obstacle_ahead and not (camera_exempt := self._camera_guard_exempt(eff)):
            # LiDARの走査面に映らない障害物(机の天板・ガラス・段差等)を
            # カメラが検知している間は自律走行を止める。手動運転(WASD)は
            # 対象外(操作者が実際の映像・状況を見て判断できるため)。
            mode = "CAMERA_OBSTACLE_HOLD"
            self._publish_stop()
            self._hold_cancel(mode)
            self._reset_crowd_scan(now)
        elif self._mapping_or_exploring:
            # 【2026-09-30追加】地図作成(SLAM)中/自動探索中は、自律動作
            # (見回しの旋回・入口ゴール・人物接近等)を出さない。/cmd_velも
            # 出さない(探索のNav2走行や手動運転の指令と奪い合わないため)。
            # ESTOP・MANUAL_OVERRIDE・各種HOLDはこれより上で優先処理済み。
            mode = "MAPPING_ACTIVE_HOLD"
            self._arrived_at_time = None
            self._home_return_active = False
            self._home_return_start_time = None
            # 【2026-10-02追加】直接cmd_velで動くモードから切り替わった瞬間だけ
            # 1回停止を送る。送らないと最後のcmd_vel(旋回等)がESP32の
            # watchdog(0.5秒)まで残る。毎tickは送らない(Nav2/探索と奪い合わない)。
            if self._prev_tick_mode in _DIRECT_CMD_VEL_MODES:
                self._publish_stop()
            self._cancel_goal_once()
            self._reset_crowd_scan(now)
        elif self._map_not_loaded():
            # 【2026-11-04追加】地図を読み込んでいない間は、自律動作をしない(挙動がおかしくなるため)。管理画面に「地図を読み込んでください」と出す。
            # 手動運転・地図作成/探索(上の分岐)・各種安全の停止は、これより上で処理済み。
            mode = "MAP_NOT_LOADED_HOLD"
            self._publish_stop()
            self._hold_cancel(mode)
            self._reset_crowd_scan(now)
        elif eff.hold_position or eff.go_to is not None:
            # 【2026-10-06追加】ルール層の方針(stay/announce_only/go_to_*)。
            # 安全系の分岐(上)より下なので、これらが先に判定された場合は到達しない。
            mode = self._policy_hold_step(now, eff)
        elif self._home_return_active:
            # 【2026-07-17追加】接近・案内完了後のホーム帰還中は、crowd_levelの
            # 変動(離れていく人物がまだ視界に残っている等)に関わらず帰還を
            # 最後まで続ける(crowded/calmの分岐より優先)。
            mode = self._return_home_step(now)
            self._reset_crowd_scan(now)
        elif crowd_level == "crowded":
            # 【2026-11-04】混雑でも「向く」を使う(人が多いほど向く対象が増える。以前はcalmのときだけで、混雑だと向かなかった)。
            # 近づくが使えるときは従来どおり入口へ向かう(向くより近づくを優先)。
            face_bearing = None
            if not self._person_interaction_ok(eff.approach_allowed, eff.approach_explicit) and self._interaction_ok(
                "face", eff.face_allowed, eff.face_explicit
            ):
                face_bearing = self._face_target_bearing(now)
            sweep_mode = None
            if (
                face_bearing is None
                and not self._person_interaction_ok(eff.approach_allowed, eff.approach_explicit)
                and self._face_sweep_ok(eff)
            ):
                sweep_mode = self._face_sweep_step(now)
            if face_bearing is not None:
                self._cancel_goal_once()
                mode = self._face_person_step(now, face_bearing)
            elif sweep_mode is not None:
                self._cancel_goal_once()
                mode = sweep_mode
            else:
                mode = "STOP_ANNOUNCE"
                self._publish_stop()
                if self._person_interaction_ok(eff.approach_allowed, eff.approach_explicit):
                    self._publish_entrance_goal()
                else:
                    # 方針approach_disabled: 呼びかけ地点へも向かわない(停止して発話のみ)。
                    self._cancel_goal_once()
            self._reset_crowd_scan(now)
        elif crowd_level == "calm":
            if (
                isinstance(target, dict)
                and self._person_interaction_ok(eff.approach_allowed, eff.approach_explicit)
                and not self._target_on_cooldown(target, now)
            ):
                status = self._approach_target(target)
                if status == "arrived":
                    if self._arrived_at_time is None:
                        self._arrived_at_time = now
                    elapsed = (now - self._arrived_at_time).nanoseconds / 1e9
                    if elapsed >= self.approach_dwell_sec:
                        # 発話が終わった頃合いなので、この人物をクールダウン
                        # 対象として記録してからホームへ帰還を開始する。
                        world = self._target_world_xy(target)
                        if world is not None:
                            self._recent_serve_position = world
                            self._recent_serve_time = now
                        # 【2026-07-22追加】接客(接近+発話)が1件完了した記録。
                        # 推奨効果分析・来客数/反応率・日報週報が読み取る。
                        promote_item = self._menu_state.get("promote_item")
                        phase = self._menu_state.get("phase", "initial")
                        promoted_menu = (
                            promote_item
                            if self._recommend_enabled and phase == "induction"
                            else None
                        )
                        self._engagement_log.record_engagement(
                            datetime.datetime.now().isoformat(timespec="seconds"),
                            promoted_menu,
                            self._recommend_enabled,
                        )
                        self._arrived_at_time = None
                        self._boundary_face_since = None
                        self._home_return_active = True
                        self._home_return_start_time = now
                        mode = self._return_home_step(now)
                    else:
                        mode = "APPROACH_ARRIVED"
                else:
                    self._arrived_at_time = None
                    mode = "APPROACH_PERSON"
                self._reset_crowd_scan(now)
            else:
                self._arrived_at_time = None
                self._boundary_face_since = None
                # 接近(Nav2駆動)から直接cmd_velの見回し/揺動へ切り替わるので、
                # 送信済みのNav2ゴールを1回だけ取り消す。
                self._cancel_goal_once()
                face_bearing = None
                if self._interaction_ok("face", eff.face_allowed, eff.face_explicit):
                    # 【2026-11-03】向く: 人がいれば(候補あり)その場で人の多い方を向く。近づくが使えない/許可でないときの代わり。
                    face_bearing = self._face_target_bearing(now)
                wiggle = self._idle_wiggle_enabled if eff.idle_wiggle is None else eff.idle_wiggle
                # 【2026-11-03】左右の揺動は「見回す」スイッチの対象(以前は近づくのスイッチ)。木/ルールの idle_wiggle_on の明示があれば動く。
                wiggle = self._interaction_ok("scan", bool(wiggle), eff.idle_wiggle is True)
                sweep_mode = None
                if face_bearing is None and self._face_sweep_ok(eff):
                    # 【2026-11-04】人がいない間も、向くがONなら、その場で前180度を往復して見回す(旋回して探して直進する「見回す」とは別)。
                    sweep_mode = self._face_sweep_step(now)
                if face_bearing is not None:
                    mode = self._face_person_step(now, face_bearing)
                    self._reset_crowd_scan(now)
                elif sweep_mode is not None:
                    mode = sweep_mode
                    self._reset_crowd_scan(now)
                elif wiggle:
                    mode = self._idle_wiggle_step(now, dt)
                elif self._interaction_ok("scan", eff.crowd_scan_allowed, eff.scan_explicit):
                    mode = self._crowd_scan_step(now, dt)
                else:
                    # 方針crowd_scan_disabled: 見回さずその場で待機する。
                    mode = "POLICY_NO_SCAN"
                    self._publish_stop()
                    self._reset_crowd_scan(now)
        else:
            mode = "WAIT_FOR_SENSOR"
            self._publish_stop()
            self._cancel_goal_once()
            self._reset_crowd_scan(now)

        # 【2026-11-04】ガード: modeが文字列でない(想定外の経路)なら、止まって待つ。
        if not isinstance(mode, str) or not mode:
            mode = "POLICY_NO_SCAN"
            self._publish_stop()
            self._reset_crowd_scan(now)
        # 【2026-11-04】カメラ障害物停止を免除したtickで、旋回だけ(FACE_PERSON/FACE_SWEEP)以外が選ばれたら、その動きは出さず停止に戻す。
        if camera_exempt is True and self._camera_obstacle_ahead and mode not in _CAMERA_EXEMPT_OK_MODES:
            mode = "CAMERA_OBSTACLE_HOLD"
            self._publish_stop()
            self._hold_cancel(mode)
            self._reset_crowd_scan(now)

        if mode in _GOAL_CANCELLING_HOLD_MODES and self._policy_goal_outstanding:
            # 安全系ホールドがNav2ゴールを取り消した。方針が続くなら、ホールド解除後に
            # 入口ゴールを送り直せるよう再武装する(ホールド中は送らない)。
            self._policy_goal_outstanding = False
            self._policy_goal_done_activation = -1

        if mode not in _HOLD_CANCEL_MODES and self._hold_edge is not None:
            self._hold_edge.left_hold()
        self._prev_tick_mode = mode

        # 近接停止が出ていない間は、免除の状態を捨てる(次に近接停止が出たとき、新しく距離を確かめ直す)。
        if not self._near_person_too_close:
            self._near_exempt_entered = False
            self._near_exempt_active = False
            self._near_clear_hist = []
        # 【2026-11-04】向く動作の監視範囲の基準向きを取り直す: 向くが許可でなかった状態から許可になったとき(スイッチ・ルール・木)、
        # および、機体の位置・向きが変わる動き(手動運転・接近・ホーム帰還・直進を含む見回し)をしたあと。
        face_now = self._interaction_ok("face", eff.face_allowed, eff.face_explicit)
        if (face_now and not getattr(self, "_face_ok_prev", False)) or mode in _POSE_CHANGING_MODES:
            self._face_ref_yaw = None
            self._face_sweep_dir = 1.0
        self._face_ok_prev = face_now

        face_active = mode == "FACE_PERSON"
        if mode not in ("FACE_PERSON", "FACE_SWEEP"):
            self._hold_release()  # 【2026-11-06】位置保持: 向く動作でなくなったら状態を捨てる
        # motion_arbiterへ: いま「向く(その場の旋回だけ)」を出している(近接停止中でも、この旋回だけ許可してよい)。毎tick出す(古くなれば無効)。
        try:
            self._face_turn_pub.publish(Bool(data=mode in ("FACE_PERSON", "FACE_SWEEP")))
        except Exception:  # noqa: BLE001 - テスト用インスタンス(__init__を通さない)でも動くように
            pass
        if not face_active and (self._face_active or self._face_aligned or self._face_bearing is not None):
            self._reset_face()
        command = {
            "stamp": self.get_clock().now().nanoseconds / 1e9,
            "mode": mode,
            "crowd_level": crowd_level,
            "promote_item": self._menu_state.get("promote_item"),
            "announce_text": announce_text,
            "direct_cmd_vel_enabled": self.enable_direct_cmd_vel,
            "estop_active": self._estop_active,
            "hardware_estop": self._hardware_estop,
            "soft_estop": self._soft_estop,
            "manual_override": self._manual_override,
            "mapping_active": self._mapping_or_exploring,
            "idle_wiggle_enabled": self._idle_wiggle_enabled,
            # 【2026-11-03追加】見回す・向くのスイッチと向く動作の状態(キー追加のみ)。
            "scan_enabled": bool(getattr(self, "_scan_enabled", False)),
            "face_enabled": bool(getattr(self, "_face_enabled", False)),
            "face_active": bool(face_active),
            "face_bearing_deg": (math.degrees(self._face_bearing) if face_active and self._face_bearing is not None else None),
            "face_aligned": bool(face_active and self._face_aligned),
            "face_sweep_active": mode == "FACE_SWEEP",
            # 【2026-11-06追加】向く旋回中の位置保持の状態(既定は無効。キー追加のみ)
            "face_hold": self._hold_status(mode),
            "map_not_loaded": mode == "MAP_NOT_LOADED_HOLD",
            "face_ref_yaw_deg": (math.degrees(self._face_ref_yaw) if getattr(self, "_face_ref_yaw", None) is not None else None),
            # 【2026-11-02追加】人の方へ動く動作のスイッチ(キー追加のみ)。gate=Falseなら従来どおり常に許可。
            "person_interaction_enabled": bool(getattr(self, "_person_interaction_enabled", False)),
            "person_interaction_gate": bool(getattr(self, "_require_interaction_opt_in", False)),
            "announce_gate": bool(getattr(self, "_require_announce_opt_in", False)),
            "boundary_motion_blocked": bool(getattr(self, "_boundary_block_reason", "")),
            "boundary_block_reason": str(getattr(self, "_boundary_block_reason", "")),
            "announce_instructed": bool(getattr(eff, "announce_explicit", False)),
            # 【2026-10-13追加】推奨発話ON/OFFの現在値(ダッシュボードの読み戻し確認用。キー追加のみ)。
            "recommend_enabled": self._recommend_enabled,
            "recommend_updated_at": getattr(self, "_recommend_updated_at", None),
            # 【2026-10-06追加】ルール層(キー追加のみ、既存キーは不変)。
            "active_rule": self._rule_active_id,
            "policy": self._rule_policy,
        }
        msg = String()
        msg.data = dumps(command)
        self._command_pub.publish(msg)
        if self._survey_hold:
            # 回答中の呼びかけは邪魔なので発話しない。終了直後にすぐ喋り出さないよう、
            # 発話間隔のタイマーも進めておく。
            self._last_announce_time = now
            self._last_announce_time_set = True
        elif self._sales_speech_suppressed():
            pass  # 【B9】announce_nodeの状況アナウンスが鳴る状況の間は、営業の呼びかけを出さない(二重発話の防止)
        elif eff.announce_allowed and (
            not getattr(self, "_require_announce_opt_in", False) or eff.announce_explicit
        ) and self._announce_facing_ok(eff, mode):
            # 発話間隔の短縮(方針)は安全系ホールド/estop中は適用しない。
            interval = None if mode in _SAFETY_MODES else eff.announce_min_interval_seconds
            self._publish_announce_if_needed(announce_text, interval)


# 【2026-11-04】カメラ障害物停止を免除してよいmode(その場の旋回だけ。止まって待つ POLICY_NO_SCAN / NEAR_PERSON_HOLD も動かないので可)。
_CAMERA_EXEMPT_OK_MODES = frozenset({"FACE_PERSON", "FACE_SWEEP", "POLICY_NO_SCAN", "NEAR_PERSON_HOLD", "CAMERA_OBSTACLE_HOLD"})

# 安全系のモード(ルール層の方針は、これらの挙動も発話頻度も変えられない)。
_SAFETY_MODES = frozenset(
    {
        "ESTOP_HOLD",
        "MANUAL_OVERRIDE",
        "NEAR_PERSON_HOLD",
        "OUT_OF_BOUNDS_HOLD",
        "KEEPOUT_ZONE_HOLD",
        "CAMERA_OBSTACLE_HOLD",
        "MAPPING_ACTIVE_HOLD",
        "MAP_NOT_LOADED_HOLD",
        "SURVEY_ACTIVE_HOLD",
    }
)
# Nav2ゴールのキャンセルを伴うホールド(方針の入口ゴール再武装の対象)。
_GOAL_CANCELLING_HOLD_MODES = frozenset(
    {
        "NEAR_PERSON_HOLD",
        "OUT_OF_BOUNDS_HOLD",
        "KEEPOUT_ZONE_HOLD",
        "CAMERA_OBSTACLE_HOLD",
        "MAPPING_ACTIVE_HOLD",
        "MAP_NOT_LOADED_HOLD",
        "SURVEY_ACTIVE_HOLD",
    }
)

# 【2026-10-23】cancel_goalを「入った最初の1回だけ」出すHOLDモード(_hold_cancel)。
_HOLD_CANCEL_MODES = frozenset(
    {
        "SURVEY_ACTIVE_HOLD",
        "NEAR_PERSON_HOLD",
        "OUT_OF_BOUNDS_HOLD",
        "KEEPOUT_ZONE_HOLD",
        "CAMERA_OBSTACLE_HOLD",
        "MAP_NOT_LOADED_HOLD",
    }
)

# 機体の位置・向きが大きく変わる動き(向く動作の監視範囲の基準向きを取り直す対象)。
_POSE_CHANGING_MODES = frozenset(
    {
        "MANUAL_OVERRIDE",
        "APPROACH_PERSON",
        "APPROACH_ARRIVED",
        "RETURN_HOME",
        "RETURN_HOME_ARRIVED",
        "CROWD_SCAN",
        "CROWD_TURN_TO_CROWD",
        "CROWD_APPROACH",
    }
)

# 直接/cmd_velを出して動くモード(MAPPING_ACTIVE_HOLDへの切替時に停止を送る対象)。
_DIRECT_CMD_VEL_MODES = frozenset(
    {
        "CROWD_SCAN",
        "CROWD_TURN_TO_CROWD",
        "CROWD_APPROACH",
        "IDLE_WIGGLE",
        "APPROACH_PERSON",
        "FACE_PERSON",
        "FACE_SWEEP",
    }
)


def main() -> None:
    rclpy.init()
    node = BehaviorDirector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
