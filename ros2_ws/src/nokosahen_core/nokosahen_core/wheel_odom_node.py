from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster


def _wrapped_int32_diff(current: int, previous: int) -> int:
    """符号付き32bit整数のラップアラウンドを考慮した current - previous。"""
    diff = (current - previous) & 0xFFFFFFFF
    if diff >= 0x80000000:
        diff -= 0x100000000
    return diff


class WheelOdomNode(Node):
    """Convert ESP32 encoder ticks into odometry for Nav2."""

    def __init__(self) -> None:
        super().__init__("wheel_odom")
        self.declare_parameter("counts_per_rev", 4096.0)
        self.declare_parameter("wheel_radius", 0.0391)
        self.declare_parameter("wheel_base", 0.424)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        # 【2026-09-30追加】物理的にあり得ないtick増分(ESP32リセットでtickが
        # 0付近へ飛んだ等)を積分してオドメトリが瞬間移動するのを防ぐ閾値。
        # 車輪速がこの値を超える、またはtick間隔dtがmax_tick_dt_secを超える
        # サンプルは積分せず、前回tickを取り直して破棄する。
        self.declare_parameter("max_wheel_speed_mps", 3.0)
        self.declare_parameter("max_tick_dt_sec", 1.0)

        self.counts_per_rev = float(self.get_parameter("counts_per_rev").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_base = float(self.get_parameter("wheel_base").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.max_wheel_speed_mps = max(0.1, float(self.get_parameter("max_wheel_speed_mps").value))
        self.max_tick_dt_sec = max(0.05, float(self.get_parameter("max_tick_dt_sec").value))

        self._prev_left: Optional[int] = None
        self._prev_right: Optional[int] = None
        self._prev_time = None
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._odom_pub = self.create_publisher(Odometry, "/odom", 20)
        # 【2026-09-30追加】ダッシュボード表示用に間引いた低頻度オドメトリ。
        # /odomは50Hz前後で流れるため、これをダッシュボード(Python/rclpy)が
        # 購読すると、届くたびにexecutorの待ち受け機構が回りCPUを食う
        # (py-spyで実測、full_dashboardのCPUの約7割がこの機構自体)。表示・
        # ステップ応答テストには10Hzで十分なので、別トピックで配信する。
        # Nav2/SLAM等は従来どおり/odomを使うので影響しない。
        self.declare_parameter("ui_odom_hz", 10.0)
        self._ui_odom_period_ns = int(1e9 / max(1.0, float(self.get_parameter("ui_odom_hz").value)))
        self._last_ui_odom_ns = 0
        self._odom_ui_pub = self.create_publisher(Odometry, "/nokosahen/odom_ui", 5)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Int32MultiArray, "/wheel_ticks", self._on_ticks, 50)

    def _on_ticks(self, msg: Int32MultiArray) -> None:
        if len(msg.data) < 2:
            return
        left = int(msg.data[0])
        right = int(msg.data[1])
        now = self.get_clock().now()

        if self._prev_left is None or self._prev_right is None:
            self._prev_left = left
            self._prev_right = right
            self._prev_time = now
            return

        dt = (now - self._prev_time).nanoseconds / 1e9 if self._prev_time else 0.0
        if dt <= 0.0:
            return

        # ESP32側のticks_data(int32_t)は長時間走行すると符号付き32bitで
        # オーバーフロー・ラップアラウンドする(ファームウェア側は
        # unsignedキャストで正しくラップして送ってくる)。Pythonのintは
        # 無限精度なので、ここで単純減算するとラップ直後に巨大な誤差分が
        # 出てオドメトリが瞬間移動してしまう。32bit符号付きの剰余算で
        # ラップを正しく扱う。
        d_left_ticks = _wrapped_int32_diff(left, self._prev_left)
        d_right_ticks = _wrapped_int32_diff(right, self._prev_right)
        self._prev_left = left
        self._prev_right = right
        self._prev_time = now

        meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.counts_per_rev
        d_left = d_left_ticks * meters_per_tick
        d_right = d_right_ticks * meters_per_tick

        # 物理的にあり得ない増分/間隔のサンプルは積分せずに破棄する
        # (prev_ticks/timeは上で既に今回値へ取り直し済み)。
        if (
            dt > self.max_tick_dt_sec
            or abs(d_left) / dt > self.max_wheel_speed_mps
            or abs(d_right) / dt > self.max_wheel_speed_mps
        ):
            self.get_logger().warn(
                f"wheel_ticksが異常(dt={dt:.2f}s, dL={d_left:.2f}m, dR={d_right:.2f}m)。"
                "ESP32リセット等の可能性があるため、このサンプルを破棄して再初期化します",
                throttle_duration_sec=2.0,
            )
            return
        d_center = (d_left + d_right) / 2.0
        d_yaw = (d_right - d_left) / self.wheel_base

        heading = self._yaw + d_yaw / 2.0
        self._x += d_center * math.cos(heading)
        self._y += d_center * math.sin(heading)
        self._yaw = math.atan2(math.sin(self._yaw + d_yaw), math.cos(self._yaw + d_yaw))

        vx = d_center / dt
        wz = d_yaw / dt
        self._publish_odom(now, vx, wz)

    def _publish_odom(self, stamp, vx: float, wz: float) -> None:
        qz = math.sin(self._yaw / 2.0)
        qw = math.cos(self._yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        odom.pose.covariance[0] = 0.03
        odom.pose.covariance[7] = 0.03
        odom.pose.covariance[35] = 0.08
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[35] = 0.1
        self._odom_pub.publish(odom)
        now_ns = stamp.nanoseconds
        if now_ns - self._last_ui_odom_ns >= self._ui_odom_period_ns:
            self._last_ui_odom_ns = now_ns
            self._odom_ui_pub.publish(odom)

        if not self.publish_tf:
            return
        tf = TransformStamped()
        tf.header.stamp = stamp.to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf)


def main() -> None:
    rclpy.init()
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

