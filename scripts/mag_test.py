#!/usr/bin/env python3
"""地磁気センサ(HMC5883L / QMC5883L互換)の試験読み取り。標準ライブラリだけで動く(smbus2不要)。2026-10-16追加。

ラズパイで実行する:
  python3 mag_test.py            # 連続表示(Ctrl+Cで終了)
  python3 mag_test.py --cal 20   # 20秒間センサをゆっくり水平に1回転以上させ、補正値(オフセット)を求めて表示
  python3 mag_test.py --still 10 # 10秒間じっとさせ、向きのばらつき(標準偏差)を表示

使い方の目安:
- 机の上で、モーター・スピーカー・画面から離して試す。次に、機体に載せた状態で同じことをして、差を見る。
- 向き(heading)は「センサのX軸が磁北から何度か」。機体の前を基準にするなら、取り付け向きの補正が別途要る。
"""
import argparse
import fcntl
import math
import os
import statistics
import struct
import sys
import time

I2C_SLAVE = 0x0703


class Bus:
    def __init__(self, bus=1):
        self.fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)

    def select(self, addr):
        fcntl.ioctl(self.fd, I2C_SLAVE, addr)

    def write(self, addr, reg, val):
        self.select(addr)
        os.write(self.fd, bytes([reg, val]))

    def read(self, addr, reg, n):
        self.select(addr)
        os.write(self.fd, bytes([reg]))
        data = os.read(self.fd, n)
        if len(data) != n:
            raise OSError(f"I2C読み取りが短い({len(data)}/{n}バイト)")
        return data


def detect(bus):
    """('hmc', 0x1E) / ('qmc', 0x0D) / None。"""
    try:
        if bus.read(0x1E, 0x0A, 3) == b"H43":
            return "hmc", 0x1E
    except OSError:
        pass
    try:
        if bus.read(0x0D, 0x0D, 1)[0] == 0xFF:
            return "qmc", 0x0D
    except OSError:
        pass
    return None


def setup(bus, kind, addr):
    if kind == "hmc":
        bus.write(addr, 0x00, 0x78)  # 8回平均, 75Hz
        bus.write(addr, 0x01, 0x20)  # ゲイン 1.3Ga
        bus.write(addr, 0x02, 0x00)  # 連続測定
        time.sleep(0.1)
    else:
        bus.write(addr, 0x0B, 0x01)  # SET/RESET period
        bus.write(addr, 0x09, 0x1D)  # OSR512, 8G, 200Hz, 連続
        time.sleep(0.1)


def sample(bus, kind, addr):
    """(x, y, z) を返す(生の値)。まだデータが無い・読み取りに失敗したときは None。"""
    try:
        return _sample(bus, kind, addr)
    except OSError:
        return None


def _sample(bus, kind, addr):
    if kind == "hmc":
        raw = bus.read(addr, 0x03, 6)
        x, z, y = struct.unpack(">hhh", raw)  # 並びは X, Z, Y
        if -4096 in (x, y, z):
            return None  # オーバーフロー
        return x, y, z
    st = bus.read(addr, 0x06, 1)[0]
    if not st & 0x01:
        return None
    return struct.unpack("<hhh", bus.read(addr, 0x00, 6))


def heading(x, y, off=(0.0, 0.0), scale=(1.0, 1.0)):
    h = math.degrees(math.atan2((y - off[1]) * scale[1], (x - off[0]) * scale[0]))
    return h % 360.0


def collect(bus, kind, addr, seconds, hz=20.0):
    out, t_end = [], time.time() + seconds
    while time.time() < t_end:
        s = sample(bus, kind, addr)
        if s:
            out.append(s)
        time.sleep(1.0 / hz)
    return out


def circ_mean(degs):
    """向きの平均(度)。0/360をまたいでも正しく計算する(円平均)。"""
    sx = sum(math.cos(math.radians(d)) for d in degs)
    sy = sum(math.sin(math.radians(d)) for d in degs)
    m = math.degrees(math.atan2(sy, sx)) % 360.0
    return 0.0 if m >= 360.0 - 1e-9 else m  # 浮動小数点の誤差で 360.0 と出るのを 0.0 に


def circ_range(degs):
    """向きの最大-最小(度)。平均からの差の最大幅で測るので、0/360をまたいでも正しい。"""
    m = circ_mean(degs)
    diffs = [((d - m + 180.0) % 360.0) - 180.0 for d in degs]
    return max(diffs) - min(diffs)


def circ_std(degs):
    """向きの標準偏差(度)。0/360をまたいでも正しく計算する。"""
    sx = sum(math.cos(math.radians(d)) for d in degs) / len(degs)
    sy = sum(math.sin(math.radians(d)) for d in degs) / len(degs)
    r = max(min(math.hypot(sx, sy), 1.0), 1e-9)
    return math.degrees(math.sqrt(-2.0 * math.log(r)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bus", type=int, default=1)
    ap.add_argument("--cal", type=float, default=0, metavar="秒", help="回転させて補正値を求める")
    ap.add_argument("--still", type=float, default=0, metavar="秒", help="静止させてばらつきを測る")
    args = ap.parse_args()

    try:
        bus = Bus(args.bus)
    except OSError as exc:
        sys.exit(f"/dev/i2c-{args.bus} を開けません({exc})。I2Cを有効にしましたか? (sudo raspi-config nonint do_i2c 0 → 再起動)")
    found = detect(bus)
    if not found:
        sys.exit("センサが見つかりません(0x1E / 0x0D)。配線(SDA=3番, SCL=5番, 3.3V=1番, GND=6番)とはんだ付けを確認してください。")
    kind, addr = found
    print(f"検出: {'HMC5883L' if kind == 'hmc' else 'QMC5883L互換'} (アドレス 0x{addr:02X})")
    setup(bus, kind, addr)

    if args.cal:
        print(f"{args.cal:.0f}秒間、センサを水平に保ったままゆっくり1回転以上させてください…")
        data = collect(bus, kind, addr, args.cal)
        if len(data) < 20:
            sys.exit("データが少なすぎます。")
        xs, ys, zs = [d[0] for d in data], [d[1] for d in data], [d[2] for d in data]
        off = ((max(xs) + min(xs)) / 2.0, (max(ys) + min(ys)) / 2.0)
        rx, ry = (max(xs) - min(xs)) / 2.0, (max(ys) - min(ys)) / 2.0
        avg = (rx + ry) / 2.0
        scale = (avg / rx if rx else 1.0, avg / ry if ry else 1.0)
        print(f"サンプル数 {len(data)}")
        print(f"オフセット x={off[0]:.1f} y={off[1]:.1f}   スケール x={scale[0]:.3f} y={scale[1]:.3f}")
        print(f"XY平面の円の半径 x={rx:.0f} y={ry:.0f}(ほぼ等しいほど、周囲の磁気の影響が小さい。差が大きければ『ゆがみ大』)")
        print(f"Z成分 平均 {statistics.mean(zs):.0f}")
        return

    if args.still:
        print(f"{args.still:.0f}秒間、センサを動かさないでください…")
        data = collect(bus, kind, addr, args.still)
        if len(data) < 10:
            sys.exit("データが少なすぎます。")
        hs = [heading(d[0], d[1]) for d in data]
        mags = [math.sqrt(sum(v * v for v in d)) for d in data]
        print(f"サンプル数 {len(data)}  向き 平均 {circ_mean(hs):.1f}°  標準偏差 {circ_std(hs):.2f}°  最大-最小 {circ_range(hs):.1f}°")
        print(f"磁場の強さ 平均 {statistics.mean(mags):.0f}(単位は機種依存。回すたびに大きく変わるなら周囲のゆがみ大)")
        return

    print("連続表示(Ctrl+Cで終了)。向き=センサX軸が磁北から何度か(補正なし)")
    try:
        while True:
            s = sample(bus, kind, addr)
            if s:
                x, y, z = s
                print(f"x={x:6d} y={y:6d} z={z:6d}  向き={heading(x, y):6.1f}°  強さ={math.sqrt(x*x+y*y+z*z):7.0f}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
