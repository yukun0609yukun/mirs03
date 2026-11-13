"""Nav2のbond(lifecycle_managerの死活監視)の送信間隔を0.5秒に下げた設定の確認(2026-10-26、CPU削減)。
/bond は管理ノード10個 x 0.1秒 = 約200件/秒で、全員が購読するためJetsonのCPUを食っていた。
実行: python3 -I test/test_nav2_bond.py   (nokosahen_coreディレクトリで)
"""
import unittest
from pathlib import Path

import yaml

BRINGUP = Path(__file__).resolve().parents[2] / "nokosahen_bringup"
# Jazzy の navigation_launch.py / localization_launch.py の lifecycle_manager が管理するノード(コストマップはcontroller/plannerの中)
MANAGED = ("amcl", "map_server", "controller_server", "smoother_server", "planner_server", "behavior_server",
           "velocity_smoother", "collision_monitor", "bt_navigator", "waypoint_follower", "docking_server")
BOND_TIMEOUT = 4.0  # lifecycle_managerの既定(このリポジトリは上書きしていない)


class BondHeartbeatTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.params = yaml.safe_load((BRINGUP / "config" / "nav2_params_jetson.yaml").read_text(encoding="utf-8"))

    def test_every_managed_node_has_slow_heartbeat(self):
        for n in MANAGED:
            p = self.params[n]["ros__parameters"]
            self.assertEqual(p.get("bond_heartbeat_period"), 0.5, n)

    def test_timeout_is_much_longer_than_heartbeat(self):
        for n in MANAGED:
            self.assertGreater(BOND_TIMEOUT, self.params[n]["ros__parameters"]["bond_heartbeat_period"] * 5, n)

    def test_bond_timeout_not_overridden_shorter(self):
        for n, v in self.params.items():
            if isinstance(v, dict) and "bond_timeout" in v.get("ros__parameters", {}):
                self.assertGreater(v["ros__parameters"]["bond_timeout"], 0.5 * 5, n)

    def test_file_keeps_crlf(self):
        raw = (BRINGUP / "config" / "nav2_params_jetson.yaml").read_bytes()
        self.assertEqual(raw.count(b"\n"), raw.count(b"\r\n"))


if __name__ == "__main__":
    unittest.main(verbosity=1)
