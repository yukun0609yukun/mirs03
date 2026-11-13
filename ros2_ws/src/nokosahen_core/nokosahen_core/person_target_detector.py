from __future__ import annotations

import datetime
import math
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import Bool, String

from .camera_recovery import CameraRecovery
from .detection_log import DetectionLog
from .json_tools import dumps, loads
from .crowd_bearing import FUSE_TOL_DEG
from .pose_relay import ROBOT_POSE_TOPIC, PoseCache
from .scan_geometry import (
    RawObstacleMemory,
    cluster_front_and_wide,
    cluster_leg_points,
    in_place_turn_ended,
    lidar_confirms_obstacle,
    lidar_priority_active,
    nearest_range,
    scan_is_stale,
    validate_background,
    wide_point_mask,
)


# /scanがこの秒数以上途絶えたら、古い人物クラスタを使い続けない(2026-10-21)
SCAN_STALE_SEC = 1.0

# 前面カメラ/後ろ向きカメラの検出方位を「新しい」とみなす秒数(_camera_confirmsと同じ値)
CAMERA_FRESH_SEC = 1.0
REAR_CAMERA_FRESH_SEC = 1.5

# 【2026-11-05】起動から一度も /scan が来ないとき、この秒数を超えたら途絶と同じ扱い(近接停止=true)にする
SCAN_STARTUP_GRACE_SEC = 3.0
# 【2026-11-05】pose_relay の位置(/nokosahen/robot_pose)をこの秒数より古ければ「位置不明」(追跡なしの従来動作)にする
POSE_FRESH_SEC = 1.5
# 【2026-11-05】旋回が終わった瞬間に出し直す、抑制前のカメラ障害物判定の有効期間(directorの camera_obstacle_timeout_sec と同程度)
RAW_OBSTACLE_FRESH_SEC = 1.0


def pose_from_cache(cache: Optional[PoseCache], now_sec: float, max_age_s: float = POSE_FRESH_SEC) -> Optional[dict]:
    """【2026-11-05】pose_relay の位置(PoseCache)から {"x","y","yaw"} を返す。未受信・古い・負の古さ(時計ずれ)は None(追跡なしの従来動作)。"""
    if cache is None:
        return None
    base = cache.base(now_sec)
    if base is None:
        return None
    x, y, yaw, age = base
    if age > max_age_s or age < -max_age_s:
        return None
    return {"x": x, "y": y, "yaw": yaw}


def _wrap_pi(angle: float) -> float:
    """角度を-π〜πへ正規化する。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def build_person_candidates(
    stamp: float,
    frame: str,
    clusters: List[Tuple[float, float, float]],
    camera_bearings: List[float],
    camera_active: bool,
    tolerance_rad: float,
    scan_stale: bool = False,
    nearest_m: Optional[float] = None,
    front_camera_half_rad: Optional[float] = None,
    nearest_age_s: Optional[float] = None,
) -> Dict[str, Any]:
    """/nokosahen/person_candidates のペイロード(dict)を作る純粋関数(2026-11-03、設計書7.6)。

    clusters: (bearing, x, y)。LiDARの脚幅クラスタ全部(カメラで棄却されたものも含む)。
    camera_bearings: 人ごとの方位(ロボット正面0・左が正[rad])。camera_active=Falseや
      古い結果のときは呼び出し側で空にして渡す。
    camera_active: カメラ(YOLO)で人を判定できる構成か。Falseなら全clusterがconfirmed=false。
    scan_stale: /scan途絶時はlidarを空にする(古い候補を使い続けない)。
    front_camera_half_rad: 前方カメラの視野の半角[rad]。camera_active かつ、この視野内でカメラに人と一致しなかった
      LiDAR候補に "rejected": true を付ける(【2026-11-04】机・椅子の脚を人として重く数えないため)。None なら付けない。
    nearest_age_s: nearest_m の元になった /scan を受信してからの経過秒(payload生成時点。time.monotonic()で測る)。
      【2026-11-05】directorが「古いスキャンの再送」を新しい測定と数えないため。nearest_m を出すときだけ出す。
    """
    lidar: List[Dict[str, Any]] = []
    if not scan_stale:
        for bearing, x, y in clusters:
            confirmed = False
            if camera_active:
                for cam in camera_bearings:
                    diff = abs(_wrap_pi(bearing - cam))
                    if diff <= tolerance_rad:
                        confirmed = True
                        break
            item = {
                "bearing": _wrap_pi(float(bearing)),
                "range": math.hypot(x, y),
                "confirmed": confirmed,
            }
            if (
                camera_active
                and not confirmed
                and front_camera_half_rad is not None
                and abs(_wrap_pi(float(bearing))) <= front_camera_half_rad
            ):
                item["rejected"] = True
            lidar.append(item)
    camera = (
        [{"bearing": _wrap_pi(float(b))} for b in camera_bearings] if camera_active else []
    )
    out = {"stamp": stamp, "frame": frame, "lidar": lidar, "camera": camera}
    # 全周で、基準スキャン(壁・柱・機体のフレーム)を除いた、一番近い物までの距離[m]。脚の形でない物も含む。
    # 無い(近くに何も無い)は大きな値、LiDARが途絶えているときは入れない(=確認できない)。
    if nearest_m is not None and not scan_stale:
        out["nearest_m"] = float(nearest_m)
        if nearest_age_s is not None:
            out["nearest_age_s"] = max(0.0, float(nearest_age_s))
    return out


class PersonTargetDetector(Node):
    """機体正面に接近してくる人物の位置を推定し、`/nokosahen/person_target` へpublishする。

    `mock_sensors.py` が流していたダミーの`person_target`を実機データへ
    置き換えるための本番用ノード。トピック形状は `mock_sensors` と同じ
    `{frame_id, x, y, confidence, source}` なので、`crowd_monitor` /
    `behavior_director` 側は無改造でそのまま使える。

    設計方針(ユーザー方針: 「足の高さのLiDAR + Webカメラを併用し、
    人が少ないときはそちらへ動きながら話しかける」):
      - 位置(x, y)の根拠は2D LiDAR(足の高さに設置)のクラスタ重心。
        脚1本分の幅に収まるクラスタだけを候補にする(机・壁など大きな
        平面はここで弾かれる)。
      - Webカメラ(YOLOv8 person検出)は、LiDARクラスタの方位と画面上の
        人物中心から出した概算方位を突き合わせる「確認」役に使う。
        カメラで人らしきものが見えている方位に一致するLiDARクラスタが
        あれば信頼度を上げ、一致しなければ静止物(誤検出)として捨てる。
      - カメラが使えない環境(OpenCV/YOLO未インストール、カメラ未接続)でも
        LiDARだけで動くよう、カメラ関連は失敗してもノード全体は動き続ける
        (`camera_counter.py`と同じ縮退方針)。
    """

    def __init__(self) -> None:
        super().__init__("person_target_detector")

        self.declare_parameter("front_angle_deg", 140.0)
        self.declare_parameter("near_distance_m", 3.0)
        # 【2026-11-04】向く(その場で人の方を向く)用の人候補を探す角度(全周=360)。接近・近接停止は front_angle_deg のまま。
        # 機体の真横や後ろにいる人を、向く対象にするため(前方140度だけだと、真横に立つと候補が出ない)。
        self.declare_parameter("candidate_angle_deg", 360.0)
        self.declare_parameter("cluster_gap_m", 0.10)
        self.declare_parameter("leg_min_width_m", 0.04)
        self.declare_parameter("leg_max_width_m", 0.35)
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("frame_id", "laser")
        # 【2026-07-17追加】LiDAR四隅の段ボール支柱(強度確保用、上に物体が乗る
        # 構造)が常時近距離クラスタとして誤検知される問題への対策。
        # crowd_monitorと同じ背景スキャン(無人時に1回記録した基準)を読み込み、
        # 「背景よりも明確に近い点」だけを人物候補にする。crowd_monitorの
        # キャリブレーションボタンで生成済みのファイルをそのまま流用する
        # (起動時に1回読み込むだけで、再キャリブレーションを反映するには
        # このノードの再起動が必要)。
        self.declare_parameter("background_file", "~/.nokosahen_crowd_background.json")
        self.declare_parameter("background_clearance_m", 0.3)
        # 【2026-07-17】人物がこの距離以内に来たら危険なので、カメラでの確認
        # (camera_bearing_tolerance_deg一致)を待たずに即座に止める安全策。
        # カメラ確認前のLiDAR脚クラスタ(_latest_clusters)を直接見るため、
        # 見せかけの誤検出(壁際の物等)より「本当に人がいるのに反応し損ねる」
        # リスクを避ける側に倒す(安全側優先)。
        # ユーザー指示で一時的にオフにできるようenable化(既定は有効のまま)。
        self.declare_parameter("enable_near_person_stop", True)
        # 【2026-11-05】起動から一度も /scan が来ないまま scan_startup_grace_sec を超えたら、途絶と同じ扱い(近接停止=true、候補のlidar空)にする。
        # LiDARを使わない構成(use_lidar:=false 等)で永久停止にならないよう、false にできる。
        self.declare_parameter("require_scan", True)
        self.declare_parameter("scan_startup_grace_sec", SCAN_STARTUP_GRACE_SEC)
        self.declare_parameter("near_person_stop_distance_m", 0.5)
        # 【2026-07-17】単独の脚幅クラスタ1つだけでは、椅子・机の脚等の細い
        # 物でも誤って「人がいる」と反応しうる。人は両足(すね)が人の立ち幅
        # 程度の間隔で2本並んでいるはずなので、そのペアを検知できた場合は
        # 「本物の人物である確度が高い」とみなし、near_person_stop_distance_mより
        # 手前(leg_pair_caution_distance_m)からでも安全停止させる。
        # 【安全方針】あくまで追加の早期警戒であり、near_person_stop_distance_m
        # 単独クラスタでの停止条件自体は緩めない(ペアが見えない=片足しか
        # 見えていない状況でも既定の距離では必ず止まる)。
        self.declare_parameter("leg_pair_min_gap_m", 0.05)
        self.declare_parameter("leg_pair_max_gap_m", 0.6)
        self.declare_parameter("leg_pair_caution_distance_m", 0.8)

        # 【2026-07-17】ユーザー要望: カメラで確認した人物を見失わないよう、
        # tickごとに「一番近いクラスタ」を選び直すのではなく、直前に確認した
        # 人物のmap座標系での位置を覚えておき、次tickでもその近くのクラスタを
        # 優先して同一人物とみなす(ID追跡)。TFが引けない環境では追跡できない
        # ため、従来通り毎tick最近傍を選ぶ挙動にフォールバックする。
        self.declare_parameter("track_gate_distance_m", 0.7)
        self.declare_parameter("track_lost_timeout_sec", 2.0)

        self.declare_parameter("enable_camera", True)
        # 数値indexではなく固定デバイスパスを指定する(camera_counterと同じ理由)。
        # 現状の物理台数は1台(決済用と共用)。2台目を導入したら別パスに変更する。
        self.declare_parameter("camera_index", "/dev/webcam_checkout")
        # 空文字ならcamera_index(V4L2デバイス)を使う。トピック名を指定すると
        # sensor_msgs/Imageを入力にする(Gazeboシミュレーション連携用)。
        self.declare_parameter("camera_image_topic", "")
        # 【2026-09-24変更】機体のWebカメラをサンワサプライCMS-V43BK(水平画角
        # 150°の超広角レンズ)に交換したため60.0→150.0。実運用値は
        # nokosahen.yamlのperson_target_detectorセクション側(同じく更新済み)。
        self.declare_parameter("camera_fov_deg", 150.0)
        self.declare_parameter("camera_bearing_tolerance_deg", 15.0)
        # 【2026-11-05】/nokosahen/person_candidates の confirmed を判定する許容角。directorの融合(crowd_bearing.FUSE_TOL_DEG=8°)と
        # 同じ値にして、同じ人を「確認済みのLiDAR」+「カメラだけ」で二重に数えないようにする。person_target(接近)の確認は従来の
        # camera_bearing_tolerance_deg(15°)のまま。
        self.declare_parameter("candidate_confirm_tolerance_deg", float(FUSE_TOL_DEG))
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("min_confidence", 0.35)
        # 【2026-11-06】前方カメラの自動復旧(USBの抜け・固まり)。フレームが取れない状態がこの秒数続いたらカメラを解放し、
        # 以後 camera_reopen_interval_sec ごとに開き直す。落ちている間はカメラ由来の候補を出さない(LiDAR候補・近接停止は止めない)。
        self.declare_parameter("camera_release_after_sec", 3.0)
        self.declare_parameter("camera_reopen_interval_sec", 10.0)
        self.declare_parameter("camera_fps", 5.0)
        # 【2026-10-08追加】地図作成/localization中(/nokosahen/mapping_active=true)は、YOLO推論を
        # この回数に1回だけ実行する(既定2=従来の「半分」。1なら間引かない)。実機でYOLOがCPU 54%を
        # 占めていたため、yamlで3-4へ上げられるようにした。映像プレビュー(別スレッド)には影響しない。
        self.declare_parameter("mapping_inference_divisor", 2)
        # 【2026-10-05追加】V4L2カメラの取得モード。空文字/0なら従来通りドライバ
        # 既定のまま(何も設定しない)。CMS-V43BKは何も指定しないと640x480(4:3)で
        # 開き、これは16:9センサー(1280x720)の左右を切り出したクロップ(縦の画角は
        # 同じで水平画角だけが約75%に狭まる)になる。広い視野を使うには
        # camera_fourcc="MJPG", camera_width=1280, camera_height=720 を指定する。
        # camera_fov_deg は「実際に取得するフレームの全幅」が写す水平画角であり、
        # モードを変えたら画角も見直すこと(方位はフレーム幅で正規化して計算)。
        self.declare_parameter("camera_width", 0)
        self.declare_parameter("camera_height", 0)
        self.declare_parameter("camera_fourcc", "")
        # 0なら設定しない。1にすると最新フレームだけを保持し遅延が減る。
        self.declare_parameter("camera_buffer_size", 0)
        # ダッシュボードで「カメラが何を見てどれくらい認識しているか」を
        # 確認できるように、検出枠を描いたJPEGを配信する(既定オン)。
        # 帯域・CPUが厳しい環境向けにオフにできるようにしておく。
        self.declare_parameter("publish_camera_preview", True)
        # 【2026-07-17】プレビュー配信はLiDAR/YOLO判定(camera_fps)とは別の
        # 独立したタイマーにし、判定処理の重さに引っ張られず、ダッシュボード側の
        # 見た目をなるべくリアルタイムに保つ(ユーザー要望: 判定用の生映像は
        # そのままで良いが、html管理画面へ飛ばす方は画質より速さを優先したい)。
        # JPEG品質も60→35に下げ、エンコード・転送を軽くしてフレームレートを稼ぐ。
        self.declare_parameter("preview_fps", 12.0)
        self.declare_parameter("preview_jpeg_quality", 35)
        # 【2026-09-15追加】カメラの解像度設定は特にしていない(ドライバの既定
        # 解像度のまま)ため、機種によってはプレビューもYOLO判定用の生解像度
        # (720p等)でJPEGエンコードされ続け、無駄にCPU・帯域を使っていた。
        # ダッシュボードの<img>表示にはそこまでの解像度は不要なため、
        # エンコード直前に幅をこのサイズまで縮小する(アスペクト比は維持、
        # 既にこれ以下の幅ならそのまま)。preview_fps=12Hzは変えない
        # (2026-07-17の「速さ優先」要望はそのまま尊重)。
        self.declare_parameter("preview_max_width", 480)
        # LiDARは足の高さの水平面しか見ておらず、机の天板・ガラス・段差等
        # LiDARの走査面に映らない障害物を見逃す。カメラで「画面下部(近く)に
        # クラスを問わず大きく写っている物」を検出したら障害物とみなし、
        # behavior_director側で自律走行を止める安全策に使う。
        self.declare_parameter("enable_obstacle_guard", True)
        self.declare_parameter("obstacle_min_confidence", 0.4)
        # 【2026-11-04】カメラの障害物検知よりLiDARの距離を優先する。カメラが障害物と見た方向で、LiDARがこの距離[m]以内に
        # 何も測っていなければ(=十分遠い)、カメラの障害物は無視する。0以下で無効(従来どおりカメラだけで判断)。
        self.declare_parameter("obstacle_lidar_confirm_m", 1.0)
        self.declare_parameter("obstacle_lidar_half_width_deg", 20.0)
        # 【2026-11-04】LiDAR優先は、その場の旋回(向く)の間だけ。それ以外はカメラだけで障害物と判断する(天板・ガラス・段差を止めるため)。
        # falseにすると、常にLiDAR優先。
        self.declare_parameter("obstacle_lidar_priority_only_in_place_turn", True)
        self.declare_parameter("obstacle_min_box_height_ratio", 0.35)
        self.declare_parameter("obstacle_bottom_zone_ratio", 0.6)
        # 障害物のワールド座標を地図に書き込む(MappingController側)ための
        # 概算位置。単眼カメラなので実測距離は無く、near_distance_m級の
        # 「かなり近い」を前提にこの固定距離にいるものとして方位だけ使う。
        self.declare_parameter("obstacle_assumed_distance_m", 0.6)
        # 【2026-07-24追加】本番環境でしか収集できない人物検知の生ログ
        # (いつ・どのくらいの距離で・カメラで確認できたか・同時に何クラスタ
        # 見えていたか)をSQLiteに記録する。明電舎でのPoC現地検証当日に
        # ユーザー要望で追加。空文字にすると記録を無効化できる。
        self.declare_parameter("detection_db_file", "~/.nokosahen_detection_log.db")

        self.front_angle_rad = math.radians(float(self.get_parameter("front_angle_deg").value))
        self.near_distance_m = float(self.get_parameter("near_distance_m").value)
        self.candidate_angle_rad = math.radians(
            max(float(self.get_parameter("front_angle_deg").value), min(360.0, float(self.get_parameter("candidate_angle_deg").value)))
        )
        self.cluster_gap_m = float(self.get_parameter("cluster_gap_m").value)
        self.leg_min_width_m = float(self.get_parameter("leg_min_width_m").value)
        self.leg_max_width_m = float(self.get_parameter("leg_max_width_m").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        publish_hz = max(0.5, float(self.get_parameter("publish_hz").value))
        self.background_clearance_m = max(
            0.0, float(self.get_parameter("background_clearance_m").value)
        )
        self._background_path = Path(
            str(self.get_parameter("background_file").value)
        ).expanduser()
        self._background: Optional[Dict[str, Any]] = self._load_background()
        self.enable_near_person_stop = bool(
            self.get_parameter("enable_near_person_stop").value
        )
        self.near_person_stop_distance_m = max(
            0.0, float(self.get_parameter("near_person_stop_distance_m").value)
        )
        self.leg_pair_min_gap_m = max(0.0, float(self.get_parameter("leg_pair_min_gap_m").value))
        self.leg_pair_max_gap_m = max(
            self.leg_pair_min_gap_m, float(self.get_parameter("leg_pair_max_gap_m").value)
        )
        self.leg_pair_caution_distance_m = max(
            self.near_person_stop_distance_m,
            float(self.get_parameter("leg_pair_caution_distance_m").value),
        )
        self.track_gate_distance_m = max(
            0.05, float(self.get_parameter("track_gate_distance_m").value)
        )
        self.track_lost_timeout_sec = max(
            0.1, float(self.get_parameter("track_lost_timeout_sec").value)
        )

        # 設定ミスは例外にならず「検出が恒久的にゼロになる」形で静かに壊れる
        # ため、明らかにおかしい値は起動時に警告しておく。
        if self.cluster_gap_m <= 0.0:
            self.get_logger().warn(
                "cluster_gap_m が0以下です。全クラスタが1点になり検出が機能しません。"
            )
        if self.leg_min_width_m > self.leg_max_width_m:
            self.get_logger().warn(
                "leg_min_width_m が leg_max_width_m を超えています。"
                "全クラスタが幅判定で棄却され検出が機能しません。"
            )

        self.require_scan = bool(self.get_parameter("require_scan").value)
        self.scan_startup_grace_sec = max(0.0, float(self.get_parameter("scan_startup_grace_sec").value))
        self._started_mono = time.monotonic()
        self._camera_tick_failure_count = 0
        self._preview_tick_failure_count = 0
        # 直近のYOLO検出枠。プレビュー用タイマー(preview_fps)は判定タイマー
        # (camera_fps)より速く回るため、検出枠は判定タイマー側で更新した
        # ものをそのまま使い回す(1〜数フレーム分古いことはあるが、目視確認
        # 用の枠表示としては実用上問題ない)。
        self._latest_preview_boxes: List[Tuple[float, float, float, float, float]] = []

        self.enable_camera = bool(self.get_parameter("enable_camera").value)
        self.camera_fov_rad = math.radians(float(self.get_parameter("camera_fov_deg").value))
        self.camera_bearing_tolerance_rad = math.radians(
            float(self.get_parameter("camera_bearing_tolerance_deg").value)
        )
        self.candidate_confirm_tolerance_rad = math.radians(
            float(self.get_parameter("candidate_confirm_tolerance_deg").value)
        )
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.publish_camera_preview = bool(self.get_parameter("publish_camera_preview").value)
        self.preview_jpeg_quality = max(
            1, min(100, int(self.get_parameter("preview_jpeg_quality").value))
        )
        self.preview_fps = max(1.0, float(self.get_parameter("preview_fps").value))
        self.preview_max_width = max(160, int(self.get_parameter("preview_max_width").value))
        self.enable_obstacle_guard = bool(self.get_parameter("enable_obstacle_guard").value)
        self.obstacle_min_confidence = float(self.get_parameter("obstacle_min_confidence").value)
        self.obstacle_lidar_confirm_m = float(self.get_parameter("obstacle_lidar_confirm_m").value)
        self.obstacle_lidar_half_width_rad = math.radians(
            max(1.0, float(self.get_parameter("obstacle_lidar_half_width_deg").value))
        )
        self.obstacle_lidar_priority_only_in_place_turn = bool(
            self.get_parameter("obstacle_lidar_priority_only_in_place_turn").value
        )
        self._raw_obstacles = RawObstacleMemory()  # 【2026-11-05】LiDAR優先で抑制する前の、カメラ障害物の方位
        self._in_place_turn_mono = None  # directorの /nokosahen/face_inplace_turn(true)を最後に受けた時刻
        self._latest_nearest_m = None
        self._latest_scan_arrays = None  # (angles, ranges, range_min, range_max, 受信時刻)
        self.obstacle_min_box_height_ratio = float(
            self.get_parameter("obstacle_min_box_height_ratio").value
        )
        self.obstacle_bottom_zone_ratio = float(
            self.get_parameter("obstacle_bottom_zone_ratio").value
        )
        self.obstacle_assumed_distance_m = float(
            self.get_parameter("obstacle_assumed_distance_m").value
        )
        detection_db_file = str(self.get_parameter("detection_db_file").value).strip()
        self._detection_log = DetectionLog(detection_db_file) if detection_db_file else None
        self._preview_pub = self.create_publisher(
            CompressedImage, "/nokosahen/front_camera/preview", 2
        )

        self._latest_clusters: List[Tuple[float, float, float]] = []  # (bearing, x, y)
        # 【2026-10-21】最後に/scanを受け取った時刻(単調時計)。LiDARが止まる・USBが抜けると_latest_clustersが更新されず、
        # 古い脚の位置を人物targetや近接停止として出し続けて、いない人物へ接近し続けていた(全体レビュー)。
        self._last_scan_mono: Optional[float] = None
        self._scan_stale_warned = False
        self._camera_bearings: List[float] = []
        self._camera_bearings_stamp = 0.0
        # 【2026-10-22】後ろ向きカメラ2台(rear_camera_persons)の検出方位(機体座標)。後ろの脚の候補をカメラで確認できるようにする
        self._rear_bearings: List[float] = []
        self._rear_bearings_stamp = 0.0
        self.create_subscription(String, "/nokosahen/rear_camera_persons", self._on_rear_persons, 5)
        self._scan_angle_increment = math.radians(1.0)  # 最初のスキャン受信まで使う暫定値

        # 【2026-07-17】人物のID追跡用。map座標系の位置が取れる場合のみ機能する。
        # 【2026-11-05】自分で /tf(約70Hz)を購読するのをやめ、pose_relay の /nokosahen/robot_pose(5Hz)から取る
        # (nav2_goal_router / wall_escape / motion_arbiter と同じ。rclpyは/tf購読1本でCPU十数%かかるため)。
        self._pose_cache = PoseCache()
        self.create_subscription(
            String, ROBOT_POSE_TOPIC, lambda m: self._pose_cache.update(m.data), 2
        )
        self._tracked_world: Optional[Tuple[float, float]] = None
        # 【2026-08-06追加】TFが引けない環境(ラジコン運用オンリー等、SLAM未起動)
        # 向けのフォールバック追跡。機体基準座標(frame_id)での直近位置を覚えて
        # おき、mapへ変換できない間もこちらでID追跡する。ロボット自体が移動すると
        # 精度は落ちるが、「TFが無いと毎tick新規検知扱いになり、
        # detection_logへ5Hzで記録され続ける」不具合(実機で18000件/時間規模の
        # 過剰記録を確認)を防ぐ方が優先度が高いと判断した。
        self._tracked_local: Optional[Tuple[float, float]] = None
        self._tracked_last_seen: Optional[float] = None

        self._pub = self.create_publisher(String, "/nokosahen/person_target", 10)
        # 【2026-11-03】人が複数いるとき「一番多い方を向く」ための全候補(LiDAR全クラスタ+カメラの
        # 人ごとの方位)。person_targetは最良の1人だけなので別トピックにした(設計書7.6)。
        self._candidates_pub = self.create_publisher(String, "/nokosahen/person_candidates", 5)
        # 追跡中の人物のmap座標系での位置。frontier_explorer(自動地図作成)が
        # 「人がいない方を優先して探索する」際に使う(2026-07-17)。
        self._tracked_world_pub = self.create_publisher(
            String, "/nokosahen/tracked_person_world", 5
        )
        # カメラ確認を待たず、LiDAR脚クラスタが近すぎたら即座に安全停止させる
        # ための専用トピック(behavior_director・TeleopControllerの両方が購読、
        # WASD手動運転中も含めて止める。behavior_director.pyのdocstring参照)。
        self._near_person_stop_pub = self.create_publisher(
            Bool, "/nokosahen/near_person_stop", 5
        )
        # LiDARの脚クラスタと一致しない(=近くに確定した1人がいない)場合でも、
        # 「今カメラに何人映っていて、だいたいどの方位に多いか」を見回し行動
        # (behavior_director)に渡すためのトピック。YOLO未導入時はpublishしない。
        self._density_pub = self.create_publisher(
            String, "/nokosahen/camera_person_density", 5
        )
        # LiDARの走査面に映らない障害物(机の天板・ガラス・段差等)をカメラで
        # 補足するための信号。behavior_director側で自律走行の安全停止に使う。
        self._obstacle_pub = self.create_publisher(
            String, "/nokosahen/camera_obstacle", 5
        )
        # マッピング中、検知した障害物の概算位置(機体基準の方位+仮定距離)を
        # MappingController側へ渡し、保存する地図に書き込めるようにする。
        self._obstacle_point_pub = self.create_publisher(
            String, "/nokosahen/camera_obstacle_point", 5
        )
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(Bool, "/nokosahen/face_inplace_turn", self._on_in_place_turn, 5)
        self.create_timer(1.0 / publish_hz, self._publish_target)

        # 【2026-09-15追加】地図作成(SLAM)・localization中はNav2/SLAM側の
        # CPU使用が増えるため、YOLO推論の頻度を半分に間引いて負荷を下げる
        # (現地検証でYOLO+Nav2+SLAM同時稼働時にload average 17〜40を記録した
        # ことを踏まえた対策)。障害物検知(camera_obstacle_point、地図作成中の
        # 天板・ガラス等の書き込みに使われる)は引き続き必要なため、完全停止は
        # せず頻度を落とすだけにする。MapLifecycleController(dashboard_common.py)
        # がmapping_mode/localization_modeの開始・終了に合わせてpublishする。
        self._mapping_active = False
        self.create_subscription(
            Bool, "/nokosahen/mapping_active", self._on_mapping_active, 5
        )
        self._camera_tick_count = 0
        try:
            self._mapping_inference_divisor = max(
                1, int(self.get_parameter("mapping_inference_divisor").value)
            )
        except (TypeError, ValueError):
            self._mapping_inference_divisor = 2

        self._cv2 = None
        self._model = None
        self._cap = None
        self._latest_topic_frame = None
        self._image_topic_mode = False
        # 【2026-07-17】self._cap(VideoCapture)は判定タイマー(rclpy executorの
        # メインスレッド)とプレビュー用バックグラウンドスレッドの両方から
        # read()されるため、排他制御が必要。
        self._cap_lock = threading.Lock()
        self._camera_recovery = CameraRecovery(
            float(self.get_parameter("camera_release_after_sec").value),
            float(self.get_parameter("camera_reopen_interval_sec").value),
        )
        self._camera_device = ""
        self._preview_thread: Optional[threading.Thread] = None
        self._preview_stop_event = threading.Event()
        if self.enable_camera:
            self._init_camera()

    # --- LiDAR側: 脚幅クラスタ抽出 ---

    def _load_background(self) -> Optional[Dict[str, Any]]:
        """crowd_monitorが生成した背景スキャン(無人時の基準)を読み込む。

        LiDAR四隅の段ボール支柱等、常時近距離にある静的な物を人物候補から
        除外するために使う(crowd_monitor.pyと同じファイル・同じ考え方)。
        起動時に1回だけ読み込むため、キャリブレーションボタンを押した後に
        反映するにはこのノードの再起動が必要。
        """
        try:
            data = loads(self._background_path.read_text(encoding="utf-8"))
        except Exception:  # noqa: BLE001 - 未作成でも通常運用に支障は無い
            return None
        # 【2026-11-05】null・文字列が混ざる/形式が違う背景は無視する(使うと毎スキャン例外になるため。ログは1回)。
        data, reason = validate_background(data)
        if data is None:
            self.get_logger().warn(f"人物近接判定の背景ファイルが不正なため無視します({reason}): {self._background_path}")
            return None
        self.get_logger().info(
            f"人物近接判定の背景(柱等の基準スキャン)を読み込みました: {self._background_path}"
        )
        return data

    def _background_ranges_if_compatible(self, msg: LaserScan) -> Optional[List[float]]:
        """今のスキャンと角度構成が一致する背景データがあれば、そのrangesを返す。

        crowd_monitor.pyの同名メソッドと同じ考え方(要素数・angle_min・
        angle_incrementが一致しなければNoneを返しフォールバックさせる)。
        """
        bg = self._background
        if bg is None:
            return None
        bg_ranges = bg.get("ranges")
        if not isinstance(bg_ranges, list) or len(bg_ranges) != len(msg.ranges):
            return None
        if abs(float(bg.get("angle_min", 0.0)) - msg.angle_min) > 1e-3:
            return None
        if abs(float(bg.get("angle_increment", 0.0)) - msg.angle_increment) > 1e-6:
            return None
        return bg_ranges

    def _on_scan(self, msg: LaserScan) -> None:
        if math.isfinite(msg.angle_increment) and msg.angle_increment > 0.0:
            self._scan_angle_increment = msg.angle_increment

        background_ranges = self._background_ranges_if_compatible(msg)

        # 【2026-08-06】以前はPythonのfor文で全レイ(RPLIDAR S1で約1440点×
        # publish_hz(既定10Hz)=14,400反復/秒)を1点ずつ判定しており、CPU負荷の
        # 一因になっていた(実機のload average高騰と合わせて指摘された)。
        # numpyでベクトル化する。副次的な改善として、以前は`angle`を
        # angle_incrementずつ加算する累積方式だったため浮動小数点誤差が
        # 蓄積し得たが、msg.angle_min + i*increment で各角度を独立計算する
        # ここではその誤差が生じない。
        half = self.front_angle_rad / 2.0
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        n = len(ranges)
        angles = msg.angle_min + np.arange(n, dtype=np.float64) * msg.angle_increment
        max_range = min(msg.range_max, self.near_distance_m)

        in_range = (
            (angles >= -half)
            & (angles <= half)
            & np.isfinite(ranges)
            & (ranges >= msg.range_min)
            & (ranges <= max_range)
        )

        if background_ranges is not None and len(background_ranges) == n:
            # 背景側がその方向を有効に測れていた(柱・壁等がある)場合のみ、
            # 「背景よりも明確に近いか」を追加条件にする(2026-07-17、
            # LiDAR四隅の段ボール支柱の誤検知対策)。背景が無限遠/未計測
            # ならnear_distance_mだけの判定のままにする。
            bg = np.asarray(background_ranges, dtype=np.float64)
            bg_finite = np.isfinite(bg)
            too_close_to_background = bg_finite & (ranges > (bg - self.background_clearance_m))
            in_range &= ~too_close_to_background

        self._latest_scan_arrays = (angles, ranges, float(msg.range_min), float(msg.range_max), time.monotonic())

        # 向く用の候補は、前方より広い角度(既定は全周)から探す。前方だけで足りるなら同じ結果を使い回す。
        # 【2026-11-05】広い範囲を使うときは、全周の点を1回だけクラスタリングし、前方の結果はそこから前方の範囲で切り取って出す
        # (以前は全周と前方で2回クラスタリングしていた。結果は同じ)。全周のときは±π(真後ろ)の継ぎ目をつなぐ。
        if self.candidate_angle_rad > self.front_angle_rad + 1e-6:
            wide = wide_point_mask(
                in_range, angles, ranges, msg.range_min, max_range, half, self.candidate_angle_rad,
                too_close_to_background if (background_ranges is not None and len(background_ranges) == n) else None,
            )
            widx = np.nonzero(wide)[0]
            clusters, self._latest_candidate_clusters = cluster_front_and_wide(
                list(zip(angles[widx].tolist(), ranges[widx].tolist())),
                self.cluster_gap_m,
                max(1e-6, self._scan_angle_increment) * 3.0,
                self.leg_min_width_m,
                self.leg_max_width_m,
                half,
                wrap_full=self.candidate_angle_rad >= 2.0 * math.pi - 1e-6,
            )
        else:
            idx = np.nonzero(in_range)[0]
            points: List[Tuple[float, float]] = list(zip(angles[idx].tolist(), ranges[idx].tolist()))
            clusters = self._cluster_points(points)
            self._latest_candidate_clusters = clusters
        self._latest_clusters = clusters
        # 向く動作の近接停止の免除(旋回で機体が触れないか)の確認用: 全周で、一番近い物までの距離。
        # 【2026-11-04】基準スキャンは使わない(機体が動いた先の近い物を、基準の壁と誤って除いて見逃すため)。
        # 支柱は scan_self_filter 済みの /scan で除去済み。自己反射の取りこぼしで近い値が出たら免除しない(安全側)。
        self._latest_nearest_m = nearest_range(ranges, msg.range_min, max_range)
        self._last_scan_mono = time.monotonic()
        self._scan_stale_warned = False

    def _cluster_points(self, points: List[Tuple[float, float]]) -> List[Tuple[float, float, float]]:
        # 距離差だけでなく角度差も見て、フィルタで除外されたレイを挟んで別々の物体が偶然近い距離を持つ場合に
        # 誤って同一クラスタへ統合されないようにする(本体は scan_geometry.cluster_leg_points)。
        return cluster_leg_points(
            points,
            self.cluster_gap_m,
            max(1e-6, self._scan_angle_increment) * 3.0,
            self.leg_min_width_m,
            self.leg_max_width_m,
        )

    @staticmethod
    def _group_width(group: List[Tuple[float, float]]) -> float:
        if len(group) < 2:
            return 0.0
        a1, r1 = group[0]
        a2, r2 = group[-1]
        x1, y1 = r1 * math.cos(a1), r1 * math.sin(a1)
        x2, y2 = r2 * math.cos(a2), r2 * math.sin(a2)
        return math.hypot(x2 - x1, y2 - y1)

    # --- カメラ側: 方位の確認用 ---

    def _apply_camera_mode(self) -> None:
        """camera_width/height/fourcc/buffer_size が指定されていればV4L2に設定し、
        実際に適用されたモードをログに出す(未指定なら従来通り何も設定しない)。"""
        cv2 = self._cv2
        cap = self._cap
        fourcc = str(self.get_parameter("camera_fourcc").value).strip()
        width = int(self.get_parameter("camera_width").value)
        height = int(self.get_parameter("camera_height").value)
        buffer_size = int(self.get_parameter("camera_buffer_size").value)
        try:
            # FOURCCは解像度より先に設定する(後だとドライバが形式を選び直す)。
            if len(fourcc) == 4:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            if width > 0 and height > 0:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            if buffer_size > 0:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, buffer_size)
            code = int(cap.get(cv2.CAP_PROP_FOURCC))
            applied_fourcc = "".join(chr((code >> (8 * i)) & 0xFF) for i in range(4))
            applied_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            applied_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.get_logger().info(
                f"person_target_detector: カメラモード 要求={fourcc or '-'} {width}x{height} "
                f"buffer={buffer_size} / 適用={applied_fourcc} {applied_w}x{applied_h} "
                f"camera_fov_deg={math.degrees(self.camera_fov_rad):.0f}"
            )
            if width > 0 and height > 0 and (applied_w, applied_h) != (width, height):
                self.get_logger().warn(
                    f"要求した{width}x{height}と異なる{applied_w}x{applied_h}で開きました"
                )
        except Exception as exc:  # pragma: no cover - 実行環境依存
            self.get_logger().warn(f"カメラモード設定に失敗(既定のまま継続): {exc}")

    def _open_front_camera(self) -> bool:
        """V4L2で前方カメラを開く(起動時と開き直しの共通部分)。開けたら True。"""
        cap = self._cv2.VideoCapture(self._camera_device, self._cv2.CAP_V4L2)
        if not cap.isOpened():
            try:
                cap.release()
            except Exception:  # noqa: BLE001
                pass
            return False
        with self._cap_lock:
            self._cap = cap
        self._apply_camera_mode()
        return True

    def _camera_go_down(self) -> None:
        """読み込み失敗が続いたカメラを解放する(検出タイマー・プレビューは cap=None を見て何もしなくなる)。"""
        with self._cap_lock:
            cap, self._cap = self._cap, None
        if cap is not None:
            try:
                cap.release()
            except Exception:  # noqa: BLE001
                pass
        self.get_logger().warn(
            f"正面カメラが{self._camera_recovery.release_after_sec:.0f}秒以上映らないため解放しました。"
            f"LiDAR単独で動作し、約{self._camera_recovery.reopen_interval_sec:.0f}秒ごとに開き直します"
        )

    def _try_reopen_front_camera(self) -> None:
        if self._image_topic_mode or not self._camera_device or self._cv2 is None:
            return
        if not self._camera_recovery.should_reopen(time.monotonic()):
            return
        try:
            if not Path(self._camera_device).exists():
                self.get_logger().warn("正面カメラのデバイスがまだありません(待機中)", once=True)
                return
            if self._open_front_camera():
                self._camera_recovery.opened()
                self._camera_tick_failure_count = 0
                self.get_logger().info("正面カメラを開き直しました")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"正面カメラの開き直しに失敗: {exc}")

    def _init_camera(self) -> None:
        try:
            import cv2  # type: ignore

            self._cv2 = cv2
        except Exception as exc:  # pragma: no cover - 実行環境依存
            self.get_logger().warn(f"OpenCV未導入のためカメラ確認は無効: {exc}")
            self.enable_camera = False
            return

        image_topic = str(self.get_parameter("camera_image_topic").value).strip()
        if image_topic:
            # Gazeboシミュレーション等のsensor_msgs/Imageを入力にするモード。
            # 実機のV4L2デバイスは開かない。
            self._image_topic_mode = True
            self.create_subscription(Image, image_topic, self._on_image_topic, 5)
            self.get_logger().info(f"person_target_detector: 画像トピック({image_topic})を入力に使用")
        else:
            camera_index = str(self.get_parameter("camera_index").value)
            # 数値ではないデバイスパス(/dev/webcam_checkout等)を渡すと、バックエンド
            # 自動選択でGStreamerが選ばれてこのJetson上のOpenCVビルドではSEGVする。
            # V4L2を明示することでGStreamerパイプラインを経由させない。
            self._camera_device = camera_index
            if not self._open_front_camera():
                # 【2026-11-06】以前はここでカメラ無効にして終了(以後開き直さない)だった。今は、LiDAR単独で動きつつ、
                # 一定間隔でカメラを開き直す(挿し直し・遅れて認識されたときに自動で戻る)。YOLOの読み込みも続ける。
                self.get_logger().warn(
                    f"正面カメラを開けなかったため、LiDAR単独で動作します(約{self._camera_recovery.reopen_interval_sec:.0f}秒ごとに開き直します): {camera_index}"
                )
                self._camera_recovery.mark_down(time.monotonic())

        # カメラ自体は開けているので、YOLOの有無に関わらずここでタイマーを登録する
        # (_on_camera_tickは_model is Noneなら検出をスキップする)。
        fps = max(1.0, float(self.get_parameter("camera_fps").value))
        self.create_timer(1.0 / fps, self._on_camera_tick)
        # 【2026-07-17再修正】プレビューは当初rclpyの別タイマーとして分離したが、
        # rclpyの実行はデフォルトでシングルスレッドのため、YOLO推論(数百ms級)が
        # メインスレッドを占有している間はプレビュー側のタイマーも一切進めず、
        # 実測1.8Hz程度(12Hz設定なのに)しか出ないことが実機で判明した。
        # 真にYOLO推論と並行させるため、rclpyのタイマーではなく独立した
        # Pythonスレッドでプレビューの取得・エンコード・配信を回す
        # (self._cap.read()はcap_lockで排他制御)。
        if self.publish_camera_preview:
            self._preview_thread = threading.Thread(
                target=self._preview_loop, daemon=True
            )
            self._preview_thread.start()

        try:
            from ultralytics import YOLO  # type: ignore

            self._model = YOLO(str(self.get_parameter("model_path").value))
            # 【2026-08-07追加】ultralyticsは明示的に.to()しない限りCPUを
            # デフォルトデバイスとして使う(GPU対応PyTorch導入済みでも自動では
            # GPUに載らないことを実機で確認済み)。CUDAが使えるなら明示的に
            # GPUへ載せる(実機でperson_target_detectorが1コア100%超を
            # 占有しCPU過負荷の一因になっていたのを解消するため)。
            try:
                import torch  # type: ignore

                if torch.cuda.is_available():
                    self._model.to("cuda")
                    self.get_logger().info("YOLO推論をGPU(CUDA)で実行します")
                else:
                    self.get_logger().info("CUDAが利用できないため、YOLO推論はCPUで実行します")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"GPUへの切り替えに失敗、CPUのまま継続します: {exc}")
            self.get_logger().info("正面カメラによる方位確認を有効化しました")
        except Exception as exc:  # pragma: no cover - 任意依存
            # YOLOが無くてもLiDAR単独判定(enable_camera=False)には切り替えるが、
            # カメラ自体は開けているのでダッシュボードでの映像プレビューは継続する。
            self.get_logger().warn(
                f"YOLO未導入のため人物検出は無効(映像プレビューはLiDAR単独判定のまま継続): {exc}"
            )
            self.enable_camera = False

    def _on_image_topic(self, msg: Image) -> None:
        try:
            if msg.encoding == "bgr8":
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif msg.encoding == "rgb8":
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                frame = frame[:, :, ::-1]
            else:
                self.get_logger().warn(
                    f"未対応のImage encoding: {msg.encoding} (bgr8/rgb8のみ対応)", once=True
                )
                return
            self._latest_topic_frame = frame.copy()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"画像トピックの変換に失敗: {exc}")

    def _on_mapping_active(self, msg: Bool) -> None:
        active = bool(msg.data)
        if active == self._mapping_active:
            return
        self._mapping_active = active
        self.get_logger().info(
            f"地図作成/localizationの{'開始' if active else '終了'}を検知、"
            f"YOLO推論頻度を{f'1/{self._mapping_inference_divisor}に間引きます' if active else '通常に戻します'}"
        )

    def _on_camera_tick(self) -> None:
        if self._cap is None and not self._image_topic_mode:
            self._try_reopen_front_camera()   # 落ちている間は開き直しだけ(検出・カメラ由来の候補は出さない)
            return
        # 地図作成/localization中は、Nav2/SLAM側のCPU負荷が増える分だけ
        # YOLO推論(このtickの本体)を間引く。障害物検知は完全には止めたくない
        # ため、丸ごとスキップはせず2tickに1回だけ実行する。
        self._camera_tick_count += 1
        if self._mapping_active and (
            self._camera_tick_count % self._mapping_inference_divisor != 0
        ):
            return
        # 「カメラが失敗してもノード全体は動き続ける」という設計方針を実行時にも
        # 保証するため、フレーム取得・推論のどこで失敗してもここで握りつぶす
        # (LiDAR単独判定にフォールバックし、ノードごとクラッシュさせない)。
        try:
            frame = self._grab_frame()
            if frame is None:
                if self._cap is not None:
                    if self._camera_recovery.on_frame_fail(time.monotonic()):
                        self._camera_go_down()
                        return
                    self._camera_tick_failure_count += 1
                    if self._camera_tick_failure_count % 30 == 1:
                        self.get_logger().warn(
                            f"正面カメラからフレームを取得できません(連続{self._camera_tick_failure_count}回)"
                        )
                return

            if self._cap is not None:
                self._camera_recovery.on_frame_ok()
            width = frame.shape[1]
            height = frame.shape[0]
            if width <= 0 or height <= 0:
                return

            # YOLO(ultralytics)未導入の環境でも、ダッシュボードでの映像確認自体は
            # できるようにする(検出枠なしのプレビューのみ配信、LiDAR単独判定は継続)。
            boxes_for_preview: List[Tuple[float, float, float, float, float]] = []
            if self._model is not None:
                results = self._model.predict(frame, imgsz=320, verbose=False)
                bearings: List[float] = []
                obstacle_bearings: List[float] = []
                raw_obstacle_bearings: List[float] = []  # 【2026-11-05】LiDAR優先で抑制する前の判定
                for result in results:
                    for box in result.boxes:
                        cls = int(box.cls[0])
                        conf = float(box.conf[0])
                        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                        center_x = (x1 + x2) / 2.0
                        # 画面中心からのずれをカメラの水平画角に写像して概算方位にする。
                        # 【注意】これはレンズ歪みを一切補正しない単純な線形近似
                        # (正規化ピクセル位置に画角をそのまま掛けるだけ)。現在の
                        # camera_fov_deg=150(超広角/魚眼系レンズ)では、画面中央
                        # 付近はまずまず正確でも、画面端に近いほど実際の方位との
                        # ズレが大きくなりやすい(魚眼レンズは端に近いほど単位
                        # ピクセルあたりの実角度が小さくなる傾向があるため)。
                        # camera_bearing_tolerance_deg(LiDARとの方位一致判定)を
                        # 実機で見ながら調整すること(docs/mapping_and_calibration.md
                        # §5.1/5.3参照)。
                        normalized = (center_x / width) - 0.5  # -0.5..0.5
                        bearing = -normalized * self.camera_fov_rad

                        # clsが人物(0)以外の場合のみ障害物候補にする(人物は
                        # LiDAR脚検出+approach_standoff_mで既に安全に扱われて
                        # おり、ここに含めると1〜2m先に立っている人自身が
                        # 障害物判定されてCAMERA_OBSTACLE_HOLDに落ち、接近
                        # 行動が自己矛盾でブロックされ続けていた
                        # (2026-07-17レビューで発見)。カメラ障害物ガードは
                        # あくまで机の天板・ガラス・段差等、LiDARが見落とす
                        # 「人以外」の物を補う目的に限定する。
                        if (
                            self.enable_obstacle_guard
                            and conf >= self.obstacle_min_confidence
                            and cls != 0
                        ):
                            box_height = y2 - y1
                            if (
                                box_height / height >= self.obstacle_min_box_height_ratio
                                and y2 / height >= self.obstacle_bottom_zone_ratio
                            ):
                                raw_obstacle_bearings.append(bearing)
                                if self._lidar_confirms_obstacle(bearing):
                                    obstacle_bearings.append(bearing)

                        if cls != 0 or conf < self.min_confidence:
                            continue
                        bearings.append(bearing)
                        boxes_for_preview.append((x1, y1, x2, y2, conf))

                self._camera_bearings = bearings
                self._camera_bearings_stamp = time.monotonic()
                self._publish_camera_density(bearings)
                if self.enable_obstacle_guard:
                    self._raw_obstacles.update(raw_obstacle_bearings, time.monotonic())
                    self._publish_camera_obstacle(bool(obstacle_bearings))
                    for bearing in obstacle_bearings:
                        self._publish_camera_obstacle_point(bearing)

            self._camera_tick_failure_count = 0
            # 実際のpublishは独立した_on_preview_tick側で行う(この枠は次の
            # プレビューtickまでそのまま使い回される)。
            self._latest_preview_boxes = boxes_for_preview
        except Exception as exc:  # noqa: BLE001
            self._camera_tick_failure_count += 1
            if self._camera_tick_failure_count % 30 == 1:
                self.get_logger().warn(f"正面カメラの処理中にエラー(LiDAR単独で継続): {exc}")

    def _grab_frame(self) -> Optional[Any]:
        """判定用タイマー(メインスレッド)とプレビュー用スレッドの両方から使う
        共通のフレーム取得。self._cap.read()はスレッドセーフではないため
        cap_lockで排他制御する(image_topic_modeは単なる属性読み出しなので
        ロック不要)。
        """
        if self._cap is not None:
            with self._cap_lock:
                cap = self._cap
                ok, frame = cap.read() if cap is not None else (False, None)
            if not ok or frame is None or frame.size == 0:
                return None
            return frame
        return self._latest_topic_frame

    def _preview_loop(self) -> None:
        """独立したバックグラウンドスレッドでプレビューの取得・エンコード・
        配信だけを回す(_on_camera_tick内のYOLO推論がメインスレッドを
        ブロックしていても、こちらは並行して進められるようにするため。
        rclpyのタイマーに乗せていた頃は実測1.8Hz程度しか出なかった
        (preview_fps=12Hz設定でも)問題への対策、2026-07-17)。
        """
        period = 1.0 / self.preview_fps
        while not self._preview_stop_event.is_set():
            start = time.monotonic()
            try:
                # 【2026-11-05】見ている人(購読者)がいなければ、読み込み・エンコードをしない(YOLOの判定側のフレーム取得は別なので止めない)。
                counter = getattr(self._preview_pub, "get_subscription_count", None)
                frame = None if (counter is not None and counter() == 0) else self._grab_frame()
                if frame is not None:
                    self._publish_camera_preview(frame, self._latest_preview_boxes)
                    self._preview_tick_failure_count = 0
            except Exception as exc:  # noqa: BLE001
                # プレビューはあくまで目視確認用のおまけ機能なので、失敗しても
                # LiDAR/YOLO判定側(_on_camera_tick)には一切影響させない。
                self._preview_tick_failure_count += 1
                if self._preview_tick_failure_count % 30 == 1:
                    self.get_logger().warn(f"カメラプレビューの配信に失敗: {exc}")
            elapsed = time.monotonic() - start
            self._preview_stop_event.wait(max(0.0, period - elapsed))

    def _publish_camera_density(self, bearings: List[float]) -> None:
        """現在のカメラフレームに何人・だいたいどの方位に映っているかを配信する。

        LiDARの脚クラスタと一致した「確定した1人」とは別に、見回し行動
        (behavior_director)が「今向いている方向にどれだけ人が写っているか」を
        比較して一番多い方向を選べるようにするための、カメラ単独の情報。
        """
        msg = String()
        avg_bearing = sum(bearings) / len(bearings) if bearings else None
        msg.data = dumps(
            {
                "stamp": time.time(),
                "person_count": len(bearings),
                "avg_bearing_rad": avg_bearing,
            }
        )
        self._density_pub.publish(msg)

    def _on_in_place_turn(self, msg: Bool) -> None:
        was_turning = self._in_place_turn_mono is not None
        self._in_place_turn_mono = time.monotonic() if bool(msg.data) else None
        # 【2026-11-05】旋回が終わった(前進に移る)瞬間に、抑制なしのカメラ障害物判定を即座に出し直す。旋回中は「LiDAR優先」で
        # 抑制した結果(false)を出していたので、次のカメラ判定(0.2秒、地図作成中0.8秒)まで本物の障害物が伝わらなかった。
        if in_place_turn_ended(was_turning, bool(msg.data)) and self.enable_obstacle_guard:
            self._republish_unsuppressed_obstacle()

    def _republish_unsuppressed_obstacle(self) -> None:
        raw = self._raw_obstacles.fresh(time.monotonic(), RAW_OBSTACLE_FRESH_SEC)
        if raw is None:
            return  # 判定が無い/古い: 出し直さない(次のカメラ判定を待つ)
        self._publish_camera_obstacle(bool(raw))

    def _lidar_confirms_obstacle(self, bearing: float) -> bool:
        """カメラが障害物と見た方位 bearing(機体基準、左が正)の近くで、LiDARがobstacle_lidar_confirm_m以内に何かを測っているか。
        LiDARを優先するための判定: 近くに何も測っていなければ False(カメラの障害物は十分遠い/誤認識として無視する)。
        機能が無効(<=0)・LiDARの結果が無い/古い・判断できないときは True(従来どおりカメラの障害物を採用=安全側)。"""
        age_turn = None if self._in_place_turn_mono is None else time.monotonic() - self._in_place_turn_mono
        if not lidar_priority_active(age_turn, self.obstacle_lidar_priority_only_in_place_turn):
            return True  # その場の旋回の間以外は、カメラだけで障害物と判断する(従来どおり)
        arr = self._latest_scan_arrays
        angles = ranges = None
        rmin = rmax = 0.0
        age = None
        if arr is not None:
            angles, ranges, rmin, rmax, stamp = arr
            age = time.monotonic() - stamp
        return lidar_confirms_obstacle(
            angles, ranges, rmin, rmax, bearing, self.obstacle_lidar_confirm_m,
            self.obstacle_lidar_half_width_rad, age, SCAN_STALE_SEC,
        )

    def _publish_camera_obstacle(self, detected: bool) -> None:
        msg = String()
        msg.data = dumps({"stamp": time.time(), "obstacle_detected": detected})
        self._obstacle_pub.publish(msg)

    def _publish_camera_obstacle_point(self, bearing: float) -> None:
        """検知した障害物の概算位置(機体基準、frame_idと同じ座標系)を配信する。

        単眼カメラなので実測距離は無く、obstacle_assumed_distance_mだけ
        前方(bearing方向)にあるものとして扱う。マッピング中の
        MappingControllerがこれを受け取り、自機の現在位置(TF)と合わせて
        ワールド座標に変換し、保存する地図へ書き込む際に使う。
        """
        x = self.obstacle_assumed_distance_m * math.cos(bearing)
        y = self.obstacle_assumed_distance_m * math.sin(bearing)
        msg = String()
        msg.data = dumps(
            {"stamp": time.time(), "frame_id": self.frame_id, "x": x, "y": y}
        )
        self._obstacle_point_pub.publish(msg)

    def _publish_camera_preview(
        self, frame: Any, boxes: List[Tuple[float, float, float, float, float]]
    ) -> None:
        """検出枠を描いたJPEGを配信する(ダッシュボードでの目視確認用)。

        描画・エンコードの失敗はここだけで握りつぶす。人物検出のロジック
        (bearingの算出)には一切影響させない(あくまで確認用のおまけ機能)。
        """
        try:
            annotated = frame.copy()
            for x1, y1, x2, y2, conf in boxes:
                pt1 = (int(x1), int(y1))
                pt2 = (int(x2), int(y2))
                self._cv2.rectangle(annotated, pt1, pt2, (0, 220, 0), 2)
                self._cv2.putText(
                    annotated,
                    f"{conf:.2f}",
                    (pt1[0], max(0, pt1[1] - 6)),
                    self._cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 220, 0),
                    1,
                    self._cv2.LINE_AA,
                )
            # ダッシュボードの<img>表示にカメラの生解像度は不要なため、
            # エンコード前に幅をpreview_max_widthまで縮小する(既にそれ以下
            # ならそのまま)。検出枠は上で既に描画済みなので、縮小後もズレない。
            h, w = annotated.shape[:2]
            if w > self.preview_max_width:
                # 【2026-09-30変更】INTER_AREAでの直接縮小はpy-spy実測でこのノードの
                # CPUの約6%を占めていたため、先に整数倍で間引いてから軽い補間で
                # 縮小する(プレビュー専用、検出処理には影響しない)。
                # 検出枠は間引き前に描画済みのため、間引き幅が大きいと細い線が
                # 消える。間引きは最大2に抑え、残りはINTER_LINEARで縮小する。
                step = min(2, max(1, w // self.preview_max_width))
                small = annotated[::step, ::step] if step > 1 else annotated
                sh, sw = small.shape[:2]
                if sw > self.preview_max_width:
                    small = self._cv2.resize(
                        small,
                        (self.preview_max_width, max(1, int(round(sh * self.preview_max_width / sw)))),
                        interpolation=self._cv2.INTER_LINEAR,
                    )
                else:
                    small = small.copy()
                annotated = small
            ok, encoded = self._cv2.imencode(
                ".jpg", annotated, [int(self._cv2.IMWRITE_JPEG_QUALITY), self.preview_jpeg_quality]
            )
            if not ok:
                return
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = encoded.tobytes()
            self._preview_pub.publish(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"カメラプレビューの生成に失敗: {exc}", once=True)

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """周期性(±π)を考慮した角度差の絶対値。"""
        diff = a - b
        return abs(math.atan2(math.sin(diff), math.cos(diff)))

    def _on_rear_persons(self, msg: String) -> None:
        try:
            data = loads(msg.data)
            bearings = [float(b) for b in data.get("bearings", [])]
        except (ValueError, TypeError, AttributeError):
            return
        self._rear_bearings = bearings
        self._rear_bearings_stamp = time.monotonic()

    def _camera_confirms(self, bearing: float) -> str:
        """'confirmed'(実際にカメラと一致)/'passthrough'(無効時・古い場合は無条件承認)/'rejected'を返す。"""
        if not self.enable_camera:
            return "passthrough"
        if time.monotonic() - self._camera_bearings_stamp > 1.0:
            # カメラの検出結果が古すぎる場合は判定材料にしない
            return "passthrough"
        for cam_bearing in self._camera_bearings:
            if self._angle_diff(bearing, cam_bearing) <= self.camera_bearing_tolerance_rad:
                return "confirmed"
        # 後ろ向きカメラ(新しい結果のときだけ)
        if time.monotonic() - self._rear_bearings_stamp <= 1.5:
            for cam_bearing in self._rear_bearings:
                if self._angle_diff(bearing, cam_bearing) <= self.camera_bearing_tolerance_rad:
                    return "confirmed"
        return "rejected"

    # --- 出力 ---

    def _get_pose(self) -> Optional[dict]:
        """map->base_link の現在位置・向きを取得する。位置が
        取れない環境ではNoneを返し、呼び出し側は追跡なしの従来動作へ
        フォールバックする。
        """
        # 【2026-11-05】自分でTFを引かず、pose_relay の /nokosahen/robot_pose から取る(古い・未受信なら None=追跡なしの従来動作)。
        return pose_from_cache(self._pose_cache, self.get_clock().now().nanoseconds / 1e9)

    @staticmethod
    def _to_world(pose: dict, x: float, y: float) -> Tuple[float, float]:
        world_x = pose["x"] + x * math.cos(pose["yaw"]) - y * math.sin(pose["yaw"])
        world_y = pose["y"] + x * math.sin(pose["yaw"]) + y * math.cos(pose["yaw"])
        return world_x, world_y

    def _publish_tracked_world(self, world: Optional[Tuple[float, float]]) -> None:
        msg = String()
        if world is None:
            msg.data = dumps({"tracked": False})
        else:
            msg.data = dumps({"tracked": True, "x": world[0], "y": world[1], "stamp": time.time()})
        self._tracked_world_pub.publish(msg)

    def _publish_target(self) -> None:
        """LiDAR脚クラスタ+カメラ確認から人物targetを選んで配信する。

        【2026-07-17】単に毎tick「一番近いクラスタ」を選び直すのではなく、
        直前に確認した人物のmap座標系位置(_tracked_world)を覚えておき、
        その近く(track_gate_distance_m以内)に候補があれば同一人物とみなして
        優先する(ユーザー要望: 見失わずに追跡し続けたい)。近くに候補が
        無ければ見失ったとみなし(track_lost_timeout_sec)、従来通り最も
        近いクラスタを新規に選ぶ。TFが引けない環境では追跡できないため、
        毎tick最近傍を選ぶ従来動作にフォールバックする。
        """
        if self._scan_is_stale():
            # /scanが途絶えた: 古いクラスタで人物を出し続けない(いない人物へ接近しない)。近接停止は下で安全側(true)にする。
            self._latest_clusters = []
        self._publish_near_person_stop()
        self._publish_person_candidates()

        candidates: List[Tuple[Tuple[float, float, float], str]] = []
        for cluster in self._latest_clusters:
            status = self._camera_confirms(cluster[0])
            if status == "rejected":
                continue
            candidates.append((cluster, status))
        if not candidates:
            self._tracked_world = None
            self._tracked_last_seen = None
            self._publish_tracked_world(None)
            return

        pose = self._get_pose()
        now = time.monotonic()

        enriched: List[Tuple[Tuple[float, float, float], str, Optional[Tuple[float, float]]]] = []
        for cluster, status in candidates:
            _, cx, cy = cluster
            world = self._to_world(pose, cx, cy) if pose is not None else None
            enriched.append((cluster, status, world))

        chosen = None
        track_fresh = (
            self._tracked_last_seen is not None
            and now - self._tracked_last_seen <= self.track_lost_timeout_sec
        )
        if track_fresh and pose is not None and self._tracked_world is not None:
            best = None
            best_dist = None
            for item in enriched:
                world = item[2]
                if world is None:
                    continue
                d = math.hypot(
                    world[0] - self._tracked_world[0], world[1] - self._tracked_world[1]
                )
                if d <= self.track_gate_distance_m and (best_dist is None or d < best_dist):
                    best = item
                    best_dist = d
            chosen = best
        elif track_fresh and pose is None and self._tracked_local is not None:
            # 【2026-08-06追加】TFが引けない環境向けのフォールバック追跡。
            # 機体基準座標(cx, cy)での近さだけを見る(map変換した場合ほど
            # 正確ではないが、「TF無しだと毎tick新規扱いになる」問題を防ぐ)。
            best = None
            best_dist = None
            for item in enriched:
                (_, cx, cy), _, _ = item
                d = math.hypot(cx - self._tracked_local[0], cy - self._tracked_local[1])
                if d <= self.track_gate_distance_m and (best_dist is None or d < best_dist):
                    best = item
                    best_dist = d
            chosen = best

        is_new_track = chosen is None
        if chosen is None:
            # 追跡対象が無い/見失った場合は、従来通り最も近いクラスタを新規に選ぶ。
            chosen = min(enriched, key=lambda item: math.hypot(item[0][1], item[0][2]))

        (bearing, x, y), status, world = chosen
        confidence = {"confirmed": 0.9, "passthrough": 0.6}[status]

        if is_new_track and self._detection_log is not None:
            # 【2026-07-24追加】同一人物を追跡し続けている間は記録しない
            # (5Hzで同じ人を何度も数えてしまうため)。新規に人物を検知した
            # 瞬間だけを1件として記録する。
            self._detection_log.record(
                timestamp=datetime.datetime.now().isoformat(timespec="seconds"),
                distance_m=math.hypot(x, y),
                camera_confirmed=(status == "confirmed"),
                cluster_count=len(candidates),
            )

        # 【2026-08-06修正】以前はworldがNone(TF無し)の場合_tracked_last_seenも
        # Noneに戻していたため、TF無し環境では毎tick「見失った」扱いになり、
        # is_new_trackが常にTrueになっていた(detection_logへの過剰記録の原因)。
        # _tracked_local(機体基準座標)は常に更新し、TF無し環境でもID追跡の
        # 鮮度(_tracked_last_seen)を維持できるようにする。
        self._tracked_world = world
        self._tracked_local = (x, y)
        self._tracked_last_seen = now
        self._publish_tracked_world(self._tracked_world)

        msg = String()
        msg.data = dumps(
            {
                "frame_id": self.frame_id,
                "x": x,
                "y": y,
                "confidence": confidence,
                "source": "person_target_detector",
            }
        )
        self._pub.publish(msg)

    def _publish_person_candidates(self) -> None:
        """/nokosahen/person_candidates を配信する(候補が無くても空リストで毎tick配信)。
        既存のクラスタ・カメラ方位を集めるだけで、新しい走査はしない。購読者がいなければ文字列化を省く。"""
        try:
            counter = getattr(self._candidates_pub, "get_subscription_count", None)
            if counter is not None and counter() <= 0:
                return
            now = time.monotonic()
            camera_active = bool(self.enable_camera) and (
                now - self._camera_bearings_stamp <= CAMERA_FRESH_SEC
            )
            bearings: List[float] = list(self._camera_bearings) if camera_active else []
            if camera_active and now - self._rear_bearings_stamp <= REAR_CAMERA_FRESH_SEC:
                bearings.extend(self._rear_bearings)
            payload = build_person_candidates(
                time.time(),
                self.frame_id,
                getattr(self, "_latest_candidate_clusters", self._latest_clusters),
                bearings,
                camera_active,
                self.candidate_confirm_tolerance_rad,
                self._scan_is_stale(),
                self._latest_nearest_m,
                self.camera_fov_rad / 2.0,
                (now - self._last_scan_mono) if self._last_scan_mono is not None else None,
            )
            msg = String()
            msg.data = dumps(payload)
            self._candidates_pub.publish(msg)
        except Exception as exc:  # noqa: BLE001 - 候補配信の失敗でperson_target本体を止めない
            self.get_logger().warn(f"person_candidatesの配信に失敗: {exc}", once=True)

    def _scan_is_stale(self) -> bool:
        """/scanを一度でも受け取ったあと、SCAN_STALE_SEC秒以上途絶えていればTrue。
        【2026-11-05】一度も受け取っていなくても、require_scan が真で起動から scan_startup_grace_sec を超えたらTrue(途絶と同じ扱い)。"""
        return scan_is_stale(
            getattr(self, "_last_scan_mono", None),
            time.monotonic(),
            getattr(self, "_started_mono", time.monotonic()),
            SCAN_STALE_SEC,
            getattr(self, "require_scan", True),
            getattr(self, "scan_startup_grace_sec", SCAN_STARTUP_GRACE_SEC),
        )

    def _publish_near_person_stop(self) -> None:
        """LiDAR脚クラスタ(カメラ確認前、_latest_clusters)のうち最も近い距離が
        near_person_stop_distance_m以内なら危険と判断し、即座に安全停止を
        知らせる。カメラの確認(_camera_confirms)を待たない理由は、確認漏れで
        「本当に人がいるのに反応しない」方が「机の脚等を誤検出して止まる」より
        危険だから(安全側優先)。

        単独クラスタの判定に加えて、両足(すね)らしき2クラスタが人の立ち幅
        程度の間隔で並んでいる「脚ペア」を検知できた場合は、本物の人物である
        確度が高いとみなし、near_person_stop_distance_mより手前
        (leg_pair_caution_distance_m)からでも安全側に停止させる
        (2026-07-17、ユーザー要望: 脚ペアのサイズ・間隔を使って人物認識の
        精度を上げたい)。単独クラスタでの停止条件は緩めていないため、
        片足しか見えていない状況でも既定の距離では必ず止まる。
        """
        detected = False
        if not self.enable_near_person_stop:
            msg = Bool()
            msg.data = False
            self._near_person_stop_pub.publish(msg)
            return
        if self._scan_is_stale():
            # LiDARが見えていない間は、人がいないとは言えないので、安全側(停止)にする(受信が再開すれば自動で解ける)。
            if not self._scan_stale_warned:
                self._scan_stale_warned = True
                self.get_logger().warn(f"/scanが{SCAN_STALE_SEC:.1f}秒以上途絶えました。人物の出力を止め、近接停止を安全側(true)にします")
            msg = Bool()
            msg.data = True
            self._near_person_stop_pub.publish(msg)
            return

        clusters = self._latest_clusters
        if clusters:
            nearest = min(math.hypot(x, y) for _, x, y in clusters)
            detected = nearest <= self.near_person_stop_distance_m

        if not detected:
            n = len(clusters)
            for i in range(n):
                _, x1, y1 = clusters[i]
                if math.hypot(x1, y1) > self.leg_pair_caution_distance_m:
                    continue
                for j in range(i + 1, n):
                    _, x2, y2 = clusters[j]
                    gap = math.hypot(x2 - x1, y2 - y1)
                    if self.leg_pair_min_gap_m <= gap <= self.leg_pair_max_gap_m:
                        detected = True
                        break
                if detected:
                    break

        msg = Bool()
        msg.data = detected
        self._near_person_stop_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._preview_stop_event.set()
        if self._preview_thread is not None:
            self._preview_thread.join(timeout=2.0)
        with self._cap_lock:
            cap, self._cap = self._cap, None
        if cap is not None:
            cap.release()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = PersonTargetDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
