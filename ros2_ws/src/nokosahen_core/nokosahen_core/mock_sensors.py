from __future__ import annotations

import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray, String

from .json_tools import dumps


class MockSensors(Node):
    """Windows/development helper that fakes menu counts, LiDAR, and wheel ticks."""

    def __init__(self) -> None:
        super().__init__("mock_sensors")
        self.declare_parameter("auto_init", True)
        self.declare_parameter("menu_names", ["カレー", "唐揚げ定食", "うどん"])
        self.declare_parameter("initial_count", 30)
        self.declare_parameter("scan_hz", 5.0)

        self._menu_names = list(self.get_parameter("menu_names").value)
        self._initial_count = int(self.get_parameter("initial_count").value)
        scan_hz = max(1.0, float(self.get_parameter("scan_hz").value))

        self._scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self._ticks_pub = self.create_publisher(Int32MultiArray, "/wheel_ticks", 10)
        self._menu_init_pub = self.create_publisher(
            String, "/nokosahen/menu_init", 10
        )
        self._count_pub = self.create_publisher(
            String, "/nokosahen/count_event", 10
        )
        self._person_pub = self.create_publisher(
            String, "/nokosahen/person_target", 10
        )

        self._tick_left = 0
        self._tick_right = 0
        self._scan_count = 0
        self._rng = random.Random(90)

        self.create_timer(1.0 / scan_hz, self._publish_scan)
        self.create_timer(0.1, self._publish_ticks)
        self.create_timer(7.0, self._publish_count_event)
        self.create_timer(5.0, self._publish_person_target)
        if bool(self.get_parameter("auto_init").value):
            self.create_timer(1.0, self._publish_initial_once)
        self._did_init = False

    def _publish_initial_once(self) -> None:
        if self._did_init:
            return
        msg = String()
        msg.data = dumps(
            {
                "items": [
                    {"name": name, "initial": self._initial_count}
                    for name in self._menu_names
                ]
            }
        )
        self._menu_init_pub.publish(msg)
        self._did_init = True

    def _publish_scan(self) -> None:
        crowded = (self._scan_count // 60) % 2 == 0
        self._scan_count += 1

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "laser"
        scan.angle_min = -math.pi / 2.0
        scan.angle_max = math.pi / 2.0
        scan.angle_increment = math.radians(1.0)
        scan.range_min = 0.05
        scan.range_max = 8.0
        scan.ranges = []

        for deg in range(-90, 91):
            if crowded and -50 <= deg <= 50 and self._rng.random() < 0.75:
                scan.ranges.append(0.7 + self._rng.random() * 1.0)
            elif not crowded and -35 <= deg <= 35 and self._rng.random() < 0.12:
                scan.ranges.append(1.0 + self._rng.random() * 2.0)
            else:
                scan.ranges.append(float("inf"))
        self._scan_pub.publish(scan)

    def _publish_ticks(self) -> None:
        self._tick_left += 4
        self._tick_right += 4
        msg = Int32MultiArray()
        msg.data = [self._tick_left, self._tick_right]
        self._ticks_pub.publish(msg)

    def _publish_count_event(self) -> None:
        if not self._menu_names:
            return
        weighted = self._menu_names + [self._menu_names[0]]
        menu = self._rng.choice(weighted)
        msg = String()
        msg.data = dumps({"menu": menu, "delta": 1, "source": "mock"})
        self._count_pub.publish(msg)

    def _publish_person_target(self) -> None:
        msg = String()
        msg.data = dumps(
            {
                "frame_id": "map",
                "x": 1.2 + self._rng.random() * 0.6,
                "y": -0.5 + self._rng.random(),
                "confidence": 0.7,
                "source": "mock",
            }
        )
        self._person_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MockSensors()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

