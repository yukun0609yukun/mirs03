"""手動運転の応答時間ハーネス(bench_manual_drive_latency.py)の軽い pytest/unittest ラッパ(2026-11-06追加)。

CIで遅さの「絶対値」は見ない(PCごとに違う)。確かめるのは次の2つだけ:
- 背景なしで運転POST(PC版・スマホ版)が全部200で返り、往復の最悪値が非常識に大きくない(2秒未満)
- 運転の経路(do_POST)で、リクエストごとのディスクへの書き込み・fsync・open がない(読むのは stat だけ)
実行: python -m pytest ros2_ws/src/nokosahen_core/test/test_bench_manual_drive_latency.py -q
  または python ros2_ws/src/nokosahen_core/test/test_bench_manual_drive_latency.py
詳しい測定は bench_manual_drive_latency.py を直接実行する(--help)。
"""
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import bench_manual_drive_latency as bench  # noqa: E402


class Args:
    n = 60
    pool = 6
    handlers = ["pc", "mobile"]
    conds = ["none", "bg"]


class BenchSmoke(unittest.TestCase):
    def test_drive_post_roundtrip_without_and_with_background(self):
        out = bench.part_http(Args, bench.DEFAULT_PKG_ROOT)
        for key, r in out.items():
            self.assertEqual(r["errors"], 0, key)
            self.assertEqual(r["drive"]["n"], Args.n, key)
            self.assertLess(r["drive"]["max"], 2000.0, key)

    def test_no_disk_write_on_drive_path(self):
        rig = bench.Rig(bench.DEFAULT_PKG_ROOT, Path(tempfile.mkdtemp(prefix="bench_unit_")))
        try:
            class A:
                n = 40
            res = bench.part_profile(A, rig)
        finally:
            rig.close()
        io = res["io_per_request"]
        self.assertEqual(io["os.fsync"], 0)
        self.assertEqual(io["os.replace"], 0)
        self.assertEqual(io["open"], 0)


if __name__ == "__main__":
    unittest.main()
