"""LiDAR自己反射フィルタ(機体の四隅の支柱を「無い」ものとして扱う)。

rplidar_nodeを/scan_rawにremapし、このノードが/scanを再配信する。
マスク窓(mask_centers_deg ± mask_half_width_deg)内で range < max_range_m のビームだけ
inf(REP-117: 範囲外)にする。窓内でもmax_range_m以上のビーム(支柱の向こうの実物)は
そのまま残す。header/frame/角度/時刻は入力のまま。enabled:=falseで素通し。
"""

from typing import Optional, Sequence, Tuple

import numpy as np

DEFAULT_CENTERS_DEG = [-134.75, -43.42, 43.92, 141.24]
DEFAULT_HALF_WIDTH_DEG = 6.0
DEFAULT_MAX_RANGE_M = 0.50


def build_window_mask(
    angle_min: float,
    angle_increment: float,
    n: int,
    centers_deg: Sequence[float],
    half_width_deg: float,
) -> np.ndarray:
    """各ビームがいずれかの窓内にあるかの真偽配列(角度は±180度で折り返して比較)。"""
    angles_deg = np.degrees(angle_min + angle_increment * np.arange(n))
    mask = np.zeros(n, dtype=bool)
    for c in centers_deg:
        diff = (angles_deg - float(c) + 180.0) % 360.0 - 180.0
        mask |= np.abs(diff) <= float(half_width_deg)
    return mask


def apply_filter(ranges: np.ndarray, window_mask: np.ndarray, max_range_m: float) -> np.ndarray:
    """窓内かつ有限かつ range<max_range_m のビームをinfにした新しい配列を返す。"""
    out = np.array(ranges, dtype=np.float32, copy=True)
    hit = window_mask & np.isfinite(out) & (out < max_range_m)
    out[hit] = np.inf
    return out


class ScanMaskCache:
    """angle_min/increment/本数が変わった時だけマスクを作り直す。"""

    def __init__(self) -> None:
        self._key: Optional[Tuple] = None
        self.mask: Optional[np.ndarray] = None

    def get(self, angle_min, angle_increment, n, centers_deg, half_width_deg) -> Tuple[np.ndarray, bool]:
        key = (float(angle_min), float(angle_increment), int(n),
               tuple(float(c) for c in centers_deg), float(half_width_deg))
        rebuilt = key != self._key
        if rebuilt:
            self.mask = build_window_mask(angle_min, angle_increment, n, centers_deg, half_width_deg)
            self._key = key
        return self.mask, rebuilt


def main(args=None) -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import LaserScan

    class ScanSelfFilter(Node):
        def __init__(self) -> None:
            super().__init__("scan_self_filter")
            self.declare_parameter("mask_centers_deg", DEFAULT_CENTERS_DEG)
            self.declare_parameter("mask_half_width_deg", DEFAULT_HALF_WIDTH_DEG)
            self.declare_parameter("max_range_m", DEFAULT_MAX_RANGE_M)
            self.declare_parameter("enabled", True)
            self._cache = ScanMaskCache()
            # rplidar_node(rplidar_ros)は/scanを RELIABLE で配信しており、購読側
            # (crowd_monitor/person_target_detector/ダッシュボード)もRELIABLE購読なので、
            # 配信側は同じRELIABLEにする(BEST_EFFORTだとQoS非互換で届かない)。
            # 購読(/scan_raw)はRELIABLE配信にもBEST_EFFORT配信にも繋がるBEST_EFFORT。
            pub_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            sub_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
            self._pub = self.create_publisher(LaserScan, "/scan", pub_qos)
            self._sub = self.create_subscription(LaserScan, "/scan_raw", self._on_scan, sub_qos)

        def _on_scan(self, msg: LaserScan) -> None:
            if not self.get_parameter("enabled").value:
                self._pub.publish(msg)
                return
            centers = list(self.get_parameter("mask_centers_deg").value)
            half = float(self.get_parameter("mask_half_width_deg").value)
            max_r = float(self.get_parameter("max_range_m").value)
            ranges = np.asarray(msg.ranges, dtype=np.float32)
            mask, rebuilt = self._cache.get(msg.angle_min, msg.angle_increment, ranges.size, centers, half)
            if rebuilt:
                self.get_logger().info(
                    f"scan_self_filter: {int(mask.sum())}/{ranges.size}ビームがマスク窓内 "
                    f"(centers={centers}, half_width={half}deg, max_range={max_r}m)"
                )
            out = LaserScan()
            out.header = msg.header
            out.angle_min = msg.angle_min
            out.angle_max = msg.angle_max
            out.angle_increment = msg.angle_increment
            out.time_increment = msg.time_increment
            out.scan_time = msg.scan_time
            out.range_min = msg.range_min
            out.range_max = msg.range_max
            out.ranges = apply_filter(ranges, mask, max_r).tolist()
            out.intensities = msg.intensities
            self._pub.publish(out)

    rclpy.init(args=args)
    node = ScanSelfFilter()
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
