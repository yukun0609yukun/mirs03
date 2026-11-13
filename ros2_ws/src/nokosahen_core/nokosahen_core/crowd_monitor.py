from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from .json_tools import dumps, loads
from .scan_geometry import validate_background

# /scanがこの秒数以上途絶えたら、混雑度・人物を「不明」にする(2026-10-21)
SCAN_STALE_SEC = 1.5


class CrowdMonitor(Node):
    """Estimate entrance crowd level from a 2D LiDAR scan."""

    def __init__(self) -> None:
        super().__init__("crowd_monitor")
        self.declare_parameter("front_angle_deg", 120.0)
        self.declare_parameter("near_distance_m", 2.0)
        self.declare_parameter("crowded_point_count", 80)
        self.declare_parameter("calm_point_count", 25)
        self.declare_parameter("publish_hz", 2.0)
        # person_target_detectorは「今まさに候補がいる間だけ」publishする設計
        # なので、人が視界から去った後もそれを知らせるメッセージは来ない。
        # タイムスタンプで古さを判定しないと、一度検出した人物位置を
        # 永久に「まだそこにいる」と扱い続けてしまう(behavior_directorが
        # 存在しない人物へ接近し続ける)。
        self.declare_parameter("person_target_timeout_sec", 2.0)
        # 本番設置場所によっては、正面に壁・什器等が2m以内にある向きがあり得る。
        # near_distance_mだけで判定すると、そういった静的な近さを人物と誤って
        # 「混雑」と判定してしまう。対策として、無人時に1回スキャンを基準
        # (background)として記録しておき、実際の計測がその基準よりも
        # background_clearance_m以上近い場合だけを「何か(人物)がいる」と
        # 数える。基準を記録していない/対応する角度が無い場合は、従来通り
        # near_distance_mだけでの判定にフォールバックする。
        self.declare_parameter("background_file", "~/.nokosahen_crowd_background.json")
        self.declare_parameter("background_clearance_m", 0.3)

        self.front_angle_rad = math.radians(
            float(self.get_parameter("front_angle_deg").value)
        )
        self.near_distance_m = float(self.get_parameter("near_distance_m").value)
        self.crowded_point_count = int(
            self.get_parameter("crowded_point_count").value
        )
        self.calm_point_count = int(self.get_parameter("calm_point_count").value)
        publish_hz = max(0.5, float(self.get_parameter("publish_hz").value))
        self.person_target_timeout_sec = max(
            0.1, float(self.get_parameter("person_target_timeout_sec").value)
        )
        if self.calm_point_count >= self.crowded_point_count:
            self.get_logger().warn(
                f"calm_point_count({self.calm_point_count})が"
                f"crowded_point_count({self.crowded_point_count})以上です。"
                "levelが一度も更新されなくなる(unknownのまま固定)か、"
                "意図しない挙動になる可能性があります。nokosahen.yamlを確認してください。"
            )

        self._level = "unknown"
        # 【2026-10-21】最後に/scanを受け取った時刻(単調時計)。LiDARが止まっても、混雑度が最後の値のまま配信され続け、
        # directorが「calm」と人物を信じて動き続けていた。途絶えたら"unknown"(directorは待機)を配信する(全体レビュー)。
        self._last_scan_mono: Optional[float] = None
        self._last_point_count = 0
        self._last_occupancy = 0.0
        self._last_person_target: Optional[dict] = None
        self._last_person_target_time = 0.0

        self.background_clearance_m = max(
            0.0, float(self.get_parameter("background_clearance_m").value)
        )
        self._background_path = Path(
            str(self.get_parameter("background_file").value)
        ).expanduser()
        self._background: Optional[Dict[str, Any]] = self._load_background()
        self._calibrate_pending = False
        # 【2026-10-13追加】キャリブレーションの受理・完了を返す(ダッシュボードが読み戻して確認する)。
        # 要求は/nokosahen/crowd_monitor/calibrate_request(String JSON {"request_id":...})で受け、
        # 結果は/nokosahen/crowd_calibration_state(TRANSIENT_LOCAL)に
        # {request_id, state: started|done|failed, at, background_frames, error}で返す。
        # 従来のBool(calibrate_background)も受けるが、request_idは空になる。
        self._calibrate_request_id = ""

        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(
            String, "/nokosahen/person_target", self._on_person_target, 10
        )
        self.create_subscription(
            Bool, "/nokosahen/crowd_monitor/calibrate_background", self._on_calibrate, 5
        )
        self.create_subscription(
            String, "/nokosahen/crowd_monitor/calibrate_request", self._on_calibrate_request, 5
        )
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

        self._calib_pub = self.create_publisher(
            String,
            "/nokosahen/crowd_calibration_state",
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._pub = self.create_publisher(String, "/nokosahen/crowd_state", 10)
        self._timer = self.create_timer(1.0 / publish_hz, self._publish)

    def _load_background(self) -> Optional[Dict[str, Any]]:
        try:
            data = loads(self._background_path.read_text(encoding="utf-8"))
        except Exception:  # noqa: BLE001 - 未作成でも通常運用に支障は無い
            return None
        # 【2026-11-05】null・文字列が混ざる/形式が違う背景は無視する(使うと毎スキャン例外になるため。ログは1回)。
        data, reason = validate_background(data)
        if data is None:
            self.get_logger().warn(f"混雑判定の背景ファイルが不正なため無視します({reason}): {self._background_path}")
            return None
        self.get_logger().info(
            f"混雑判定の背景(壁等の基準スキャン)を読み込みました: {self._background_path}"
        )
        return data

    def _on_calibrate(self, msg: Bool) -> None:
        if bool(msg.data):
            self._calibrate_pending = True
            self.get_logger().info(
                "次のLiDARスキャンを混雑判定の背景として記録します"
                "(部屋を無人にしてから実行してください)"
            )

    def _publish_calibration_state(
        self, request_id: str, state: str, frames: int = 0, error: str = ""
    ) -> None:
        msg = String()
        msg.data = dumps(
            {
                "request_id": request_id,
                "state": state,
                "at": time.time(),
                "background_frames": int(frames),
                "error": error,
            }
        )
        self._calib_pub.publish(msg)

    def _on_calibrate_request(self, msg: String) -> None:
        data = loads(msg.data, {})
        rid = str(data.get("request_id") or "") if isinstance(data, dict) else ""
        self._calibrate_pending = True
        self._calibrate_request_id = rid
        self._publish_calibration_state(rid, "started")
        self.get_logger().info(
            "次のLiDARスキャンを混雑判定の背景として記録します"
            "(部屋を無人にしてから実行してください)"
        )

    def _save_background(self, msg: LaserScan) -> str:
        """背景を記録する。保存に失敗したときはその理由(成功なら空文字)を返す。"""
        data = {
            "angle_min": msg.angle_min,
            "angle_increment": msg.angle_increment,
            "ranges": [float(r) for r in msg.ranges],
        }
        error = ""
        try:
            self._background_path.parent.mkdir(parents=True, exist_ok=True)
            self._background_path.write_text(dumps(data), encoding="utf-8")
        except Exception as exc:  # noqa: BLE001
            error = f"背景ファイルの保存に失敗(今回は有効ですが再起動で消えます): {exc}"
            self.get_logger().warn(f"混雑判定の背景の保存に失敗: {exc}")
        self._background = data
        self.get_logger().info("混雑判定の背景を記録しました")
        return error

    def _on_person_target(self, msg: String) -> None:
        data = loads(msg.data, {})
        if isinstance(data, dict):
            self._last_person_target = data
            self._last_person_target_time = time.monotonic()

    def _current_person_target(self) -> Optional[dict]:
        if self._last_person_target is None:
            return None
        if time.monotonic() - self._last_person_target_time > self.person_target_timeout_sec:
            return None
        return self._last_person_target

    def _background_ranges_if_compatible(self, msg: LaserScan) -> Optional[List[float]]:
        """今のスキャンと角度構成が一致する背景データがあれば、そのrangesを返す。

        LiDARのスキャンモードが変わる(角度刻み・点数が変わる)と背景データを
        インデックスでそのまま突き合わせられなくなるため、要素数と
        angle_min/angle_increment が十分近いことを確認してから使う。
        一致しない場合はNone(呼び出し側はnear_distance_mだけでの判定に戻る)。
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

    def _scan_is_stale(self) -> bool:
        """/scanを一度でも受け取ったあと、SCAN_STALE_SEC秒以上途絶えていればTrue。一度も受け取っていない間はFalse(従来どおり)。"""
        last = getattr(self, "_last_scan_mono", None)
        return last is not None and (time.monotonic() - last) > SCAN_STALE_SEC

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan_mono = time.monotonic()
        if self._calibrate_pending:
            self._calibrate_pending = False
            rid = self._calibrate_request_id
            self._calibrate_request_id = ""
            error = self._save_background(msg)
            frames = int(np.count_nonzero(np.isfinite(np.asarray(msg.ranges, dtype=np.float64))))
            self._publish_calibration_state(
                rid, "failed" if error else "done", frames, error
            )

        background_ranges = self._background_ranges_if_compatible(msg)

        # 【2026-09-15】以前はPythonのfor文で全レイ(RPLIDAR S1で約1440点)を
        # 1点ずつ判定しており、publish_hzとは無関係にスキャン到着ごと(実機で
        # 秒10〜15回程度)に走っていたためCPU負荷の一因になっていた
        # (person_target_detector.pyの_on_scanで先に行った対策と同種の問題)。
        # numpyでベクトル化する。
        half = self.front_angle_rad / 2.0
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        n = len(ranges)
        angles = msg.angle_min + np.arange(n, dtype=np.float64) * msg.angle_increment

        in_front = (angles >= -half) & (angles <= half) & np.isfinite(ranges)
        valid = in_front & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        valid_points = int(np.count_nonzero(valid))

        is_near = valid & (ranges <= self.near_distance_m)
        if background_ranges is not None and len(background_ranges) == n:
            # 背景側がその方向を有効に測れていた(壁等がある)場合のみ、
            # 「背景よりも明確に近いか」を追加条件にする。背景が
            # 無限遠/未計測ならnear_distance_mだけの判定のままにする。
            bg = np.asarray(background_ranges, dtype=np.float64)
            bg_finite = np.isfinite(bg)
            stricter_near = ranges <= (bg - self.background_clearance_m)
            is_near = np.where(is_near & bg_finite, stricter_near, is_near)

        near_points = int(np.count_nonzero(is_near))

        occupancy = near_points / max(1, valid_points)
        self._last_point_count = near_points
        self._last_occupancy = occupancy

        if near_points >= self.crowded_point_count:
            self._level = "crowded"
        elif near_points <= self.calm_point_count:
            self._level = "calm"

    def _publish(self) -> None:
        stale = self._scan_is_stale()  # LiDARが途絶えている間は、混雑度も人物も「不明」(古い値を信じさせない)
        msg = String()
        msg.data = dumps(
            {
                "stamp": self.get_clock().now().nanoseconds / 1e9,
                "level": "unknown" if stale else self._level,
                "near_point_count": 0 if stale else self._last_point_count,
                "occupancy": 0.0 if stale else self._last_occupancy,
                "near_distance_m": self.near_distance_m,
                "person_target": None if stale else self._current_person_target(),
                "scan_stale": stale,
            }
        )
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CrowdMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

