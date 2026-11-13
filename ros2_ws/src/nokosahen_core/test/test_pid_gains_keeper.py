"""scripts/festival/pid_gains_keeper.py(ESP32のPIDゲインを文化祭用の値に保つ見張り)の純粋な部分の検査。
ROS・実機は使わない(parse_gains / close / 既定値)。実機での測定: docs/face_person_design.md 7.10章。"""
import importlib.util
import math
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
SCRIPT = ROOT / "scripts" / "festival" / "pid_gains_keeper.py"


def _load():
    spec = importlib.util.spec_from_file_location("pid_gains_keeper", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class PidGainsKeeperTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.k = _load()

    def test_festival_gains_are_12_finite_and_in_range(self):
        g = self.k.FESTIVAL_GAINS
        self.assertEqual(len(g), 12)
        self.assertTrue(all(math.isfinite(v) and 0.0 <= v <= self.k.GAIN_MAX for v in g))

    def test_festival_gains_change_only_kp_of_left_wheel_and_backward(self):
        """元の値(6.0,1.0,0.0, 6.0,0.0,0.0, 6.1,1.0,0.0, 6.1,0.0,0.0)からの変更は、kp だけ(ki・kd は据え置き)。変更幅は ±40% 以内。"""
        orig = [6.0, 1.0, 0.0, 6.0, 0.0, 0.0, 6.1, 1.0, 0.0, 6.1, 0.0, 0.0]
        new = self.k.FESTIVAL_GAINS
        for i, (a, b) in enumerate(zip(orig, new)):
            if i % 3 != 0:  # ki, kd
                self.assertEqual(a, b, i)
            elif a > 0:
                self.assertLessEqual(abs(b - a) / a, 0.40 + 1e-9, i)

    def test_parse_gains_accepts_and_rejects(self):
        self.assertEqual(len(self.k.parse_gains(",".join(["1"] * 12))), 12)
        for bad in ("1,2,3", ",".join(["1"] * 11), ",".join(["-1"] + ["1"] * 11), ",".join(["99"] + ["1"] * 11),
                    ",".join(["nan"] + ["1"] * 11), ",".join(["inf"] + ["1"] * 11), "a," * 11 + "a"):
            with self.assertRaises(ValueError, msg=bad):
                self.k.parse_gains(bad)

    def test_close_tolerance(self):
        g = self.k.FESTIVAL_GAINS
        self.assertTrue(self.k.close(g, [v + 0.01 for v in g]))
        self.assertFalse(self.k.close(g, [6.0, 1.0, 0.0, 6.0, 0.0, 0.0, 6.1, 1.0, 0.0, 6.1, 0.0, 0.0]))  # 元の値は一致しない
        self.assertFalse(self.k.close(g, g[:11]))
        self.assertFalse(self.k.close(g, None or []))

    def test_dry_run_prints_target(self):
        out = subprocess.run([sys.executable, str(SCRIPT), "--dry-run"], capture_output=True, timeout=30)
        self.assertEqual(out.returncode, 0)

    def test_never_publishes_cmd_vel(self):
        text = SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn("/cmd_vel", text.replace("cmd_vel は一切出さない", ""))
        self.assertNotIn("Twist", text)

    def test_festival_start_wires_the_keeper_with_opt_out(self):
        text = (ROOT / "scripts" / "festival" / "festival_start.sh").read_text(encoding="utf-8")
        self.assertIn("pid_gains_keeper.sh", text)
        self.assertIn("--no-pid-gains", text)
        self.assertTrue((ROOT / "scripts" / "festival" / "pid_gains_keeper.sh").is_file())


if __name__ == "__main__":
    unittest.main()
