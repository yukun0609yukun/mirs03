#!/usr/bin/env python3
"""ESP32足回りの手動駆動テストツール(Stage1、車輪を浮かせた状態で使うこと)。

WASDのダッシュボード(比例操作)とは別に、「この速度で正転/後転させたら
エンコーダはどう反応するか」を数値で指定して確認するためのCLIツール。
指定した速度で一定時間だけ/cmd_velを送り、開始/終了時の/wheel_ticksを
表示するだけのシンプルな作り(ROS 2環境がsource済みであること以外、
追加のパッケージビルドは不要)。

使い方(要: ROS 2環境をsource済み、micro-ROS Agentが起動中):
    python3 scripts/drive_test.py --linear 0.05 --duration 2       # 前進
    python3 scripts/drive_test.py --linear -0.05 --duration 2      # 後退
    python3 scripts/drive_test.py --angular 0.3 --duration 2       # その場旋回(左回り)
    python3 scripts/drive_test.py --angular -0.3 --duration 2      # その場旋回(右回り)
    python3 scripts/drive_test.py --linear 0.05 --angular 0.1 --duration 3

安全のため、指定時間が経過したら必ず最後にゼロ速度を送って終了する。
Ctrl-Cで中断した場合も同様(finally節で送信)。
"""
from __future__ import annotations

import argparse
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class DriveTestNode(Node):
    def __init__(self, linear: float, angular: float, duration: float, hz: float) -> None:
        super().__init__("drive_test")
        self.linear = linear
        self.angular = angular
        self.duration = duration
        self.hz = hz
        self.start_ticks: Optional[List[int]] = None
        self.latest_ticks: Optional[List[int]] = None
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Int32MultiArray, "/wheel_ticks", self._on_ticks, 10)

    def _on_ticks(self, msg: Int32MultiArray) -> None:
        self.latest_ticks = list(msg.data)
        if self.start_ticks is None:
            self.start_ticks = list(msg.data)

    def run(self) -> None:
        # 開始前のtick取得待ち(/wheel_ticksが来ていないとESP32/Agent未接続の
        # 可能性が高いので、最大2秒待って諦める)。
        deadline = time.monotonic() + 2.0
        while self.start_ticks is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.start_ticks is None:
            self.get_logger().warn(
                "/wheel_ticksを受信できませんでした。"
                "micro-ROS Agentが起動しているか、ESP32が接続されているか確認してください。"
            )
            self.start_ticks = [0, 0]

        self.get_logger().info(
            f"開始tick: L={self.start_ticks[0]} R={self.start_ticks[1]} / "
            f"指令: linear={self.linear:+.3f} m/s, angular={self.angular:+.3f} rad/s, "
            f"{self.duration:.1f}秒間"
        )

        twist = Twist()
        twist.linear.x = self.linear
        twist.angular.z = self.angular
        period = 1.0 / self.hz
        end_time = time.monotonic() + self.duration
        try:
            while time.monotonic() < end_time:
                self._pub.publish(twist)
                rclpy.spin_once(self, timeout_sec=period)
        finally:
            # 指定時間経過でもCtrl-C中断でも、必ずゼロ速度で締める
            # (ESP32側のウォッチドッグ(500ms)より確実に早く止める)。
            self._pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.2)

        end_ticks = self.latest_ticks or self.start_ticks
        self.get_logger().info(f"終了tick: L={end_ticks[0]} R={end_ticks[1]}")
        self.get_logger().info(
            "差分(tick): "
            f"L={end_ticks[0] - self.start_ticks[0]:+d} "
            f"R={end_ticks[1] - self.start_ticks[1]:+d}"
        )
        self.get_logger().info(
            "前進指令で両輪とも正の値になっていれば正常(逆なら"
            "firmware/esp32_microros_drive/include/config.hのINVERT_L/INVERT_Rを切り替える)。"
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="ESP32足回りの手動駆動テスト(Stage1)。必ず車輪を浮かせた状態で使うこと。",
    )
    parser.add_argument(
        "--linear", type=float, default=0.0, help="直進速度 m/s(正=前進、負=後退。既定0.0)"
    )
    parser.add_argument(
        "--angular", type=float, default=0.0, help="旋回速度 rad/s(正=左回り、負=右回り。既定0.0)"
    )
    parser.add_argument("--duration", type=float, default=2.0, help="送り続ける時間(秒、既定2.0)")
    parser.add_argument("--hz", type=float, default=10.0, help="/cmd_vel送信頻度(Hz、既定10.0)")
    args = parser.parse_args()

    if args.linear == 0.0 and args.angular == 0.0:
        parser.error("--linearまたは--angularのどちらかは0以外を指定してください")
    if args.duration <= 0.0:
        parser.error("--durationは0より大きい値にしてください")

    rclpy.init()
    node = DriveTestNode(args.linear, args.angular, args.duration, max(1.0, args.hz))
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
