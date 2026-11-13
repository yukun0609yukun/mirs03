#!/usr/bin/env python3
"""ESP32のPIDゲインを、文化祭用に調整した値に保つ見張り(2026-11-06)。

ESP32のPIDゲインは RAM にだけある(再起動すると既定に戻る)。そこで、ESP32が出す /nokosahen/pid_gains_state を見て、
目標の値と違えば /nokosahen/pid_gains へ送り直す。ESP32が再起動して既定値に戻っても、数秒で元に戻る。

目標の値(12要素): [rkp_fwd,rki_fwd,rkd_fwd, rkp_bwd,rki_bwd,rkd_bwd, lkp_fwd,lki_fwd,lkd_fwd, lkp_bwd,lki_bwd,lkd_bwd]
  既定(ファームの値) : 6.0,1.0,0.0, 6.0,0.0,0.0, 6.1,1.0,0.0, 6.1,0.0,0.0
  文化祭用(2026-11-06) : 6.0,1.0,0.0, 6.6,0.0,0.0, 7.1,1.0,0.0, 7.8,0.0,0.0
    実機の測定: その場旋回(0.9 rad/s)で、左回りは前へ約+4.1 cm/rad、右回りは後ろへ約-2.5 cm/rad ずれていた。
    左輪の kp を上げ、後退側の kp を少し上げると、左回り+0.02、右回り+0.8 cm/rad(絶対値1以下)に収まり、直進(0.2 m/s)は
    横ずれ0・向きの変化1°未満のままだった。docs/face_person_design.md 7.10章。

使い方: python3 pid_gains_keeper.py [--once] [--gains a,b,...(12個)] [--interval 20]
  --once  : 目標と違えば送って、一致を確かめて終了する
環境変数 NOKOSAHEN_PID_GAINS でも目標を指定できる。元の既定に戻すには --gains 6.0,1.0,0.0,6.0,0.0,0.0,6.1,1.0,0.0,6.1,0.0,0.0
安全: 各値は 0〜20 の有限の数だけ受け付ける。cmd_vel は一切出さない。
"""
import argparse
import math
import os
import sys
import time

FESTIVAL_GAINS = [6.0, 1.0, 0.0, 6.6, 0.0, 0.0, 7.1, 1.0, 0.0, 7.8, 0.0, 0.0]
GAIN_MAX = 20.0


def parse_gains(text):
    vals = [float(x) for x in text.split(",")]
    if len(vals) != 12 or not all(math.isfinite(v) and 0.0 <= v <= GAIN_MAX for v in vals):
        raise ValueError("12個の 0〜%g の数が必要です: %r" % (GAIN_MAX, text))
    return vals


def close(a, b, tol=0.06):
    return len(a) == len(b) and all(abs(x - y) <= tol for x, y in zip(a, b))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--gains", default=os.environ.get("NOKOSAHEN_PID_GAINS", ""))
    ap.add_argument("--interval", type=float, default=20.0)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args(argv)
    target = parse_gains(args.gains) if args.gains else list(FESTIVAL_GAINS)
    if args.dry_run:
        print("目標のゲイン:", target)
        return 0

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray

    class K(Node):
        def __init__(self):
            super().__init__("pid_gains_keeper")
            self.pub = self.create_publisher(Float32MultiArray, "/nokosahen/pid_gains", 10)
            self.state = None
            self.state_t = 0.0
            self.create_subscription(Float32MultiArray, "/nokosahen/pid_gains_state", self.on_state, 10)

        def on_state(self, m):
            self.state = [float(x) for x in m.data]
            self.state_t = time.monotonic()

        def spin(self, sec):
            end = time.time() + sec
            while time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.05)

        def send(self):
            m = Float32MultiArray()
            m.data = list(target)
            for _ in range(5):
                self.pub.publish(m)
                self.spin(0.1)

    rclpy.init()
    k = K()
    applied = 0
    t_start = time.monotonic()
    while True:
        k.spin(1.0)
        fresh = k.state is not None and time.monotonic() - k.state_t < 5.0
        if fresh and not close(k.state, target):
            k.send()
            k.spin(1.5)
            applied += 1
            print("%s PIDゲインを目標に送りました(%d回目): 前=%s -> 目標=%s" % (time.strftime("%F %T"), applied, k.state, target), flush=True)
        elif fresh and applied == 0 and args.once:
            print("すでに目標の値です:", k.state)
            return 0
        if args.once:
            if fresh and close(k.state, target):
                print("目標の値になりました:", k.state)
                return 0
            if time.monotonic() - t_start > 90:
                print("90秒待っても ESP32 の状態が取れない/一致しません: state=%s" % (k.state,))
                return 1
            continue
        k.spin(max(1.0, args.interval - 1.0))


if __name__ == "__main__":
    sys.exit(main())
