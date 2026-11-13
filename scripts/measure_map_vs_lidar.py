#!/usr/bin/env python3
"""地図の壁と、実際のLiDARの壁のずれを測る(2026-10-22、1台目で実行する。読み取りだけ・ロボットは動かさない)。

背景: 壁まで0.36 m(LiDAR)なのに、地図(global_costmap)上は機体が通れない扱いだった。地図の壁が実際より内側に出ている(太い/ずれている)なら、
経路計画の失敗は地図の作り方が原因で、(経路計画用の外形を小さくするより)地図を直すほうが根本的。

使い方(1台目で。ロボットの手動運転モードが動いていて、地図(SLAMまたは保存した地図)が出ている状態):
    source /opt/ros/jazzy/setup.bash; source ~/mirs2603_nokosahen/ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=77
    python3 ~/mirs2603_nokosahen/scripts/measure_map_vs_lidar.py [--map-topic /map] [--seconds 8]
出力: 機体の周り8方位(前=0、左が正)で、「LiDARの最短距離」と「/map上の最寄りの壁までの距離」と、その差(map - lidar)。
差が+(地図の壁が遠い=地図は壁を細く/外に描いている)でなく−(地図の壁が近い=地図の壁が太い/内側にずれている)なら、地図の壁が実際より内側。
"""
import argparse
import math
import time

import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-topic", default="/map")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--occupied", type=int, default=65, help="この値以上を壁(占有)とみなす")
    args = ap.parse_args()

    rclpy.init()
    node = Node("measure_map_vs_lidar")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    got = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, args.map_topic, lambda m: got.__setitem__("map", m), qos)
    node.create_subscription(LaserScan, "/scan", lambda m: got.__setitem__("scan", m), 5)
    end = time.time() + args.seconds
    while time.time() < end and not ("map" in got and "scan" in got):
        rclpy.spin_once(node, timeout_sec=0.2)
    t0 = time.time()
    while time.time() - t0 < 1.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if "map" not in got or "scan" not in got:
        print("地図またはスキャンが取れません(map:%s scan:%s)" % ("map" in got, "scan" in got))
        return 1
    try:
        tf = buf.lookup_transform("map", "base_link", rclpy.time.Time()).transform
    except Exception as exc:  # noqa: BLE001
        print("map→base_linkの座標変換が取れません:", exc)
        return 1
    rx, ry, ryaw = tf.translation.x, tf.translation.y, yaw_of(tf.rotation)
    m, s = got["map"], got["scan"]
    info = m.info
    res = info.resolution

    def occupied(x, y):
        cx = int((x - info.origin.position.x) / res)
        cy = int((y - info.origin.position.y) / res)
        if not (0 <= cx < info.width and 0 <= cy < info.height):
            return False
        return m.data[cy * info.width + cx] >= args.occupied

    # LiDAR: 方位8つの最短距離
    lidar = {}
    for i, r in enumerate(s.ranges):
        if not math.isfinite(r) or r < max(s.range_min, 0.05):
            continue
        a = math.degrees(s.angle_min + i * s.angle_increment) % 360.0
        k = int(((a + 22.5) % 360.0) // 45.0) * 45
        k = k if k <= 180 else k - 360
        lidar[k] = min(lidar.get(k, 9.9), r)

    # 地図: 各方位の中心線に沿って、機体から最初の占有マスまでの距離(各方位±15度の最小)
    def map_dist(center_deg):
        best = 9.9
        for off in (-15, -7.5, 0, 7.5, 15):
            a = ryaw + math.radians(center_deg + off)
            d = 0.05
            while d < 3.0:
                if occupied(rx + d * math.cos(a), ry + d * math.sin(a)):
                    best = min(best, d)
                    break
                d += res / 2.0
        return best

    print("機体位置(map): x=%.2f y=%.2f yaw=%.0f度   地図の解像度 %.3f m" % (rx, ry, math.degrees(ryaw), res))
    print("%6s  %8s  %8s  %8s" % ("方位", "LiDAR[m]", "地図[m]", "地図-LiDAR"))
    diffs = []
    for k in sorted(lidar):
        md = map_dist(k)
        if md < 9.0 and lidar[k] < 3.0:
            diffs.append(md - lidar[k])
        print("%6d  %8.2f  %8.2f  %+9.2f" % (k, lidar[k], md, md - lidar[k]))
    if diffs:
        avg = sum(diffs) / len(diffs)
        print("\n平均のずれ(地図-LiDAR): %+.3f m" % avg)
        if avg < -0.04:
            print("→ 地図の壁が実際より内側(太い/ずれ)。地図を直すのが根本対策(壁を1マス細くする、保存した地図で位置合わせ、SLAMの設定)。")
        elif avg > 0.04:
            print("→ 地図の壁が実際より外側。壁際で衝突に近い判断になる。位置合わせ(AMCL/slam)のずれを確認。")
        else:
            print("→ 地図とLiDARはほぼ一致。壁際で動けない原因は、地図ではなくコストの余白・経路計画の設定側。")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
