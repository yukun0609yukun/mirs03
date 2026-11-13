"""nav2_goal_router: 代替の回頭が調停の許可の取り消しで中止したときの、目的地の扱いのテスト(2026-10-24 M5)。ROSは使わない。

取り消した持ち主(motion_grantのowner)が estop / manual / near_person のときは、目的地をNav2へ送らず破棄する(世代を進め、ログに理由)。
nav2 / wall_escape など、それ以外は従来どおり送る。

実行: python3 -I ros2_ws/src/nokosahen_core/test/test_goal_router_grant.py
"""
import json
import os
import sys
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

for _name, _attrs in (
    ("rclpy", ()), ("rclpy.action", ("ActionClient",)), ("rclpy.duration", ("Duration",)),
    ("rclpy.node", ("Node",)), ("geometry_msgs", ()), ("geometry_msgs.msg", ("PoseStamped", "Twist")),
    ("std_msgs", ()), ("std_msgs.msg", ("Bool", "String")),
):
    try:
        __import__(_name)
    except Exception:  # noqa: BLE001
        _m = types.ModuleType(_name)
        for _a in _attrs:
            setattr(_m, _a, object)
        sys.modules[_name] = _m

from nokosahen_core import nav2_goal_router as r  # noqa: E402


class FakePub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(("info", m))

    def warn(self, m):
        self.lines.append(("warn", m))

    def error(self, m):
        self.lines.append(("error", m))


def FakeTwist():
    return types.SimpleNamespace(linear=types.SimpleNamespace(x=0.0), angular=types.SimpleNamespace(z=0.0))


class RouterGrantTests(unittest.TestCase):
    def setUp(self):
        self._twist = r.Twist
        r.Twist = FakeTwist

    def tearDown(self):
        r.Twist = self._twist

    def make(self):
        n = r.Nav2GoalRouter.__new__(r.Nav2GoalRouter)
        n._goal_seq = 5
        n._hardware_estop = False
        n._soft_estop = False
        n._nav_cmd_pub = FakePub()
        n._log = FakeLogger()
        n.get_logger = lambda: n._log
        n._sent = []
        n._send_navigate = lambda pose: n._sent.append(pose)
        n._pose = object()
        n._fallback = {"pose": n._pose, "seq": 5, "deadline": 1e12}
        return n

    def grant(self, n, owner, rotate=False):
        n._on_motion_grant(types.SimpleNamespace(data=json.dumps(
            {"owner": owner, "allow": {"wall_escape": False, "router_rotate": rotate, "director_direct": False}, "stamp": 0})))

    def test_discard_owners(self):
        for owner in ("estop", "manual", "near_person"):
            self.assertTrue(r.discard_goal_on_revoke(owner), owner)
        for owner in ("nav2", "wall_escape", "router_rotate", "idle", None, ""):
            self.assertFalse(r.discard_goal_on_revoke(owner), owner)

    def test_revoked_by_estop_manual_near_person_discards_the_goal(self):
        for owner in ("estop", "manual", "near_person"):
            n = self.make()
            self.grant(n, owner)
            n._fallback_tick()
            self.assertEqual(n._sent, [], owner)                  # Nav2へ送らない
            self.assertIsNone(n._fallback, owner)                 # 回頭は終わる
            self.assertEqual(n._goal_seq, 6, owner)               # 世代を進める(古い通知を無効に)
            self.assertEqual(len(n._nav_cmd_pub.sent), 1, owner)  # 回頭を止めるゼロ速度は出す
            self.assertTrue(any(owner in m and "破棄" in m for _, m in n._log.lines), owner)

    def test_revoked_by_nav2_or_wall_escape_still_sends(self):
        for owner in ("nav2", "wall_escape", "router_rotate", "idle"):
            n = self.make()
            self.grant(n, owner)
            n._fallback_tick()
            self.assertEqual(n._sent, [n._pose], owner)           # 従来どおり
            self.assertIsNone(n._fallback, owner)
            self.assertEqual(n._goal_seq, 5, owner)
            self.assertEqual(len(n._nav_cmd_pub.sent), 1, owner)

    def test_stale_grant_owner_is_not_used(self):
        n = self.make()
        self.grant(n, "manual")
        n._grant_time -= 5.0       # 古い許可(受信が途絶えた): 持ち主は不明扱い -> 従来どおり送る
        n._fallback_tick()
        self.assertEqual(n._sent, [n._pose])

    def test_granted_rotation_continues_or_finishes_as_before(self):
        n = self.make()
        self.grant(n, "router_rotate", rotate=True)
        n._fallback_tick()          # 向きが取れない(TFなし)ので、回頭を終えて送る = 従来どおり
        self.assertEqual(n._sent, [n._pose])
        self.assertEqual(n._goal_seq, 5)

    def test_owner_is_kept_from_the_grant(self):
        n = self.make()
        self.assertIsNone(n._grant_owner)
        self.grant(n, "near_person")
        self.assertEqual(n._grant_owner, "near_person")
        n._on_motion_grant(types.SimpleNamespace(data="not json"))
        self.assertEqual(n._grant_owner, "near_person")            # 壊れた通知では更新しない

    def test_older_cancel_paths_unchanged(self):
        # 新しい目的地・緊急停止で回頭が打ち切られる従来の経路は、そのまま
        n = self.make()
        n._goal_seq = 9
        n._fallback_tick()
        self.assertIsNone(n._fallback)
        self.assertEqual(n._sent, [])
        n = self.make()
        n._soft_estop = True
        n._fallback_tick()
        self.assertIsNone(n._fallback)
        self.assertEqual(n._sent, [])


if __name__ == "__main__":
    unittest.main()
