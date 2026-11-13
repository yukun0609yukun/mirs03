"""PC版サーバー(make_dashboard_handler)にも、スマホ版と同じスイッチのAPIの受け口があること。
2026-11-04: PC側に受け口が無く、近づく/見回す/向く/揺動のボタンが404でONにならなかった。
rclpy不要。実行: python test/test_pc_handler_switch_routes.py
"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
SRC = (Path(__file__).resolve().parents[1] / "nokosahen_core" / "dashboard_common.py").read_text(encoding="utf-8")

ROUTES = ("/api/person_interaction", "/api/scan_enabled", "/api/face_enabled", "/api/idle_wiggle")


class PcHandlerSwitchRoutes(unittest.TestCase):
    def test_pc_and_mobile_handlers_route_all_switches(self):
        a, b, c = SRC.index("def make_dashboard_handler("), SRC.index("def make_mobile_handler("), len(SRC)
        for name, src in (("PC", SRC[a:b]), ("スマホ", SRC[b:c])):
            for r in ROUTES:
                self.assertIn(f'"{r}"', src, f"{name}版に {r} の受け口が無い")


if __name__ == "__main__":
    unittest.main()
