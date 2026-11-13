"""scripts/mag_test.py の純関数(向き・円平均・ばらつき)のテスト。I2C機器は使わない。2026-10-16。"""
import importlib.util
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[4] / "scripts" / "mag_test.py"


def load():
    try:
        spec = importlib.util.spec_from_file_location("mag_test", SCRIPT)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    except ModuleNotFoundError:  # Windowsにはfcntlが無い(ラズパイで使うスクリプト)
        return None


M = load()


@unittest.skipIf(M is None, "fcntl が無い環境(Windows)")
class MagMathTest(unittest.TestCase):
    def test_heading(self):
        self.assertAlmostEqual(M.heading(1, 0), 0.0)
        self.assertAlmostEqual(M.heading(0, 1), 90.0)
        self.assertAlmostEqual(M.heading(0, -1), 270.0)
        self.assertAlmostEqual(M.heading(11, 10, off=(10, 10)), 0.0)

    def test_circular_mean_across_north(self):
        self.assertAlmostEqual(M.circ_mean([359.0, 1.0, 0.0]), 0.0, places=6)
        self.assertAlmostEqual(M.circ_mean([10.0, 20.0, 30.0]), 20.0, places=6)

    def test_range_and_std_across_north(self):
        self.assertAlmostEqual(M.circ_range([359.0, 1.0, 0.0]), 2.0, places=6)
        self.assertLess(M.circ_std([359.0, 1.0, 0.0]), 1.5)
        self.assertAlmostEqual(M.circ_range([10.0, 10.0]), 0.0)


if __name__ == "__main__":
    unittest.main()
