"""機体の位置(map→base_link)と map→odom を、TFから取って低頻度(5Hz)のトピックへ中継する(2026-10-26)。

背景: Jetson1のCPU過負荷(load 10〜14)の原因の1つが、自作ノード(nav2_goal_router / wall_escape / motion_arbiter)が
それぞれ TransformListener で /tf(約70Hz)を購読していたこと。rclpyは1メッセージごとの受信処理が重く、/tf 47Hzの購読1本で
CPU 13.5%(2026-09-30の空ノード実測)かかる。この3ノードが使うのは「目的地を受けたときの向き」「離脱の中継点」
「後退開始のログ」だけで、70Hzは要らないため、/tf を受けるのをこのノード1つに集約し、5Hzの小さなトピックで配る。

トピック /nokosahen/robot_pose (std_msgs/String, JSON):
  {"base":     {"stamp": TFの時刻[秒], "x": .., "y": .., "yaw": ..},     # map→base_link(取れたときだけ)
   "map_odom": {"stamp": TFの時刻[秒]}}                                   # map→odom(取れたときだけ。古さのログ用)
- stamp は「配信した時刻」ではなく、TF(lookup_transformの結果)の header.stamp。受け取る側は stamp から古さを計算する。
  よって、このノードが止まる/TFが途絶えると、受け取る側では古さが増え、従来の「古い位置では判断しない」分岐に落ちる。
- TFが取れなかった変換は、そのキーを入れない(古い値を新しく見せない)。どちらも取れなければ何も配信しない。
"""
from __future__ import annotations

import json
import math
from typing import Dict, Optional, Tuple

try:  # ROSなしの単体テストでも、純粋な部分だけを読み込めるようにする
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
except ImportError:  # pragma: no cover
    rclpy = None
    Node = object

ROBOT_POSE_TOPIC = "/nokosahen/robot_pose"
RELAY_HZ = 5.0


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def stamp_to_sec(sec: int, nanosec: int) -> float:
    return float(sec) + float(nanosec) * 1e-9


def build_pose_message(
    base: Optional[Tuple[float, float, float, float]],
    map_odom_stamp: Optional[float],
) -> Optional[str]:
    """配信するJSON文字列を作る。base=(stamp, x, y, yaw)。値が有限でないものは入れない。どちらも無ければNone(配信しない)。"""
    out: Dict[str, dict] = {}
    if base is not None and all(math.isfinite(v) for v in base):
        out["base"] = {"stamp": float(base[0]), "x": float(base[1]), "y": float(base[2]), "yaw": float(base[3])}
    if map_odom_stamp is not None and math.isfinite(map_odom_stamp):
        out["map_odom"] = {"stamp": float(map_odom_stamp)}
    if not out:
        return None
    return json.dumps(out)


class PoseCache:
    """受け取る側が使う、最後に受信した位置の入れ物。古さは常に(今 - TFのstamp)で出す。時刻は呼ぶ側が渡す(単体テスト用)。"""

    def __init__(self) -> None:
        self._base: Optional[Tuple[float, float, float, float]] = None   # (stamp, x, y, yaw)
        self._map_odom: Optional[float] = None                           # stamp

    def update(self, text: str) -> bool:
        """メッセージ(JSON文字列)を取り込む。壊れていればFalseで、前の値は変えない。含まれないキーは「取れなかった」=Noneにする。"""
        try:
            data = json.loads(text)
            if not isinstance(data, dict):
                return False
            base = None
            b = data.get("base")
            if isinstance(b, dict):
                v = (float(b["stamp"]), float(b["x"]), float(b["y"]), float(b["yaw"]))
                if all(math.isfinite(t) for t in v):
                    base = v
            mo = None
            m = data.get("map_odom")
            if isinstance(m, dict):
                s = float(m["stamp"])
                if math.isfinite(s):
                    mo = s
        except (ValueError, TypeError, KeyError):
            return False
        self._base, self._map_odom = base, mo
        return True

    def base(self, now_sec: float) -> Optional[Tuple[float, float, float, float]]:
        """(x, y, yaw, 古さ[秒])。一度も届いていない/最後の配信に含まれていなければNone。古さの判定は呼ぶ側(従来と同じ閾値)で行う。"""
        if self._base is None:
            return None
        stamp, x, y, yaw = self._base
        return x, y, yaw, now_sec - stamp

    def map_odom_age(self, now_sec: float) -> Optional[float]:
        if self._map_odom is None:
            return None
        return now_sec - self._map_odom


class PoseRelay(Node):  # type: ignore[misc]
    def __init__(self) -> None:
        super().__init__("pose_relay")
        self.declare_parameter("rate_hz", RELAY_HZ)
        rate = max(0.5, float(self.get_parameter("rate_hz").value))
        # このノードだけが /tf を購読する(他の自作ノードは /nokosahen/robot_pose を使う)
        self._buf = Buffer()
        self._listener = TransformListener(self._buf, self)
        self._pub = self.create_publisher(String, ROBOT_POSE_TOPIC, 2)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f"pose_relay: map→base_link / map→odom を {rate:.1f}Hz で {ROBOT_POSE_TOPIC} へ中継します")

    def _lookup(self, child: str):
        try:
            return self._buf.lookup_transform("map", child, rclpy.time.Time())
        except Exception:  # noqa: BLE001 TFが未到着など。古い値は出さない
            return None

    def _tick(self) -> None:
        base = None
        tf = self._lookup("base_link")
        if tf is not None:
            q, t = tf.transform.rotation, tf.transform.translation
            base = (stamp_to_sec(tf.header.stamp.sec, tf.header.stamp.nanosec), t.x, t.y,
                    yaw_from_quaternion(q.x, q.y, q.z, q.w))
        mo = None
        tf2 = self._lookup("odom")
        if tf2 is not None:
            mo = stamp_to_sec(tf2.header.stamp.sec, tf2.header.stamp.nanosec)
        text = build_pose_message(base, mo)
        if text is not None:
            self._pub.publish(String(data=text))


def main() -> None:
    rclpy.init()
    node = PoseRelay()
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
