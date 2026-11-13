"""スケジュールによる走行範囲の一時的な差し替え/解除(/nokosahen/boundary_override)のテスト(2026-10-23)。

ROS 2が無いPCでも動くよう、rclpy系をスタブしてdashboard_commonを読み込む(test_boundary_api.pyと同じ方式)。
ロボットには何も送らない。docs/schedule.md。
確認すること:
- map_orientation: 木のset_boundaryの出力 → 配信用メッセージ → 上書き後の範囲(不正は上書きなし=安全側)
- BoundaryController: travel_boundaryを出すのはコントローラだけ。上書き中は上書きの範囲、終われば元の範囲へ戻る。
  上書きはTTLを過ぎたら無効(directorが止まれば元の範囲に戻る)。不正なメッセージは無視。
  元の範囲のファイルは上書きでは書き換わらない。設定画面の値(status)は上書きの影響を受けない。
  上書き中にユーザーが範囲を設定しても、上書きが終わった時に新しい元の範囲へ戻る。
実行: python3 -I ros2_ws/src/nokosahen_core/test/test_boundary_override.py
"""

import json
import sys
import tempfile
import unittest
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import test_boundary_api as tba  # noqa: E402  (スタブ導入・FakeNodeを再利用)
from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import map_orientation as mo  # noqa: E402

dc_time = dc.time


class Msg:
    def __init__(self, text):
        self.data = text


class SubNode(tba.FakeNode):
    def __init__(self):
        super().__init__()
        self.subs = {}
        self.timers = []

    def create_subscription(self, typ, topic, cb, depth):
        self.subs[topic] = cb
        return object()

    def create_timer(self, period, cb):
        self.timers.append((period, cb))
        return object()


RELEASE = {"mode": "release"}
SET = {"mode": "set", "x_min": 0.0, "y_min": -1.0, "x_max": 4.0, "y_max": 2.0}


class MessageTests(unittest.TestCase):
    def test_release_message(self):
        msg = mo.boundary_override_message(RELEASE, stamp=1.0)
        self.assertEqual((msg["active"], msg["enabled"]), (True, False))
        state = mo.parse_boundary_override(msg)
        self.assertFalse(state["enabled"])
        self.assertTrue(state["override_active"])

    def test_set_message(self):
        msg = mo.boundary_override_message(SET)
        self.assertEqual((msg["active"], msg["enabled"]), (True, True))
        state = mo.parse_boundary_override(json.loads(json.dumps(msg)))
        self.assertTrue(state["enabled"])
        self.assertEqual((state["min_x"], state["max_x"], state["min_y"], state["max_y"]), (0.0, 4.0, -1.0, 2.0))
        rect = mo.rect_from_state(state)
        self.assertTrue(mo.point_in_rect(1.0, 0.0, rect))
        self.assertFalse(mo.point_in_rect(5.0, 0.0, rect))

    def test_no_request_is_inactive(self):
        self.assertEqual(mo.boundary_override_message(None)["active"], False)
        self.assertIsNone(mo.parse_boundary_override(mo.boundary_override_message(None)))

    def test_invalid_requests_become_inactive(self):
        for bad in ({"mode": "set", "x_min": 3.0, "x_max": 1.0, "y_min": 0.0, "y_max": 1.0},
                    {"mode": "set", "x_min": float("nan"), "x_max": 1.0, "y_min": 0.0, "y_max": 1.0},
                    {"mode": "set"}, {"mode": "bogus"}, "release", 5):
            self.assertFalse(mo.boundary_override_message(bad)["active"], bad)

    def test_parse_rejects_garbage(self):
        for bad in (None, [], "x", {}, {"active": False}, {"active": "true"}, {"active": True},
                    {"active": True, "enabled": True},
                    {"active": True, "enabled": True, "center_x": 0, "center_y": 0, "width": -1, "height": 1}):
            self.assertIsNone(mo.parse_boundary_override(bad), bad)


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "boundary.json"
        self.node = SubNode()
        self.bc = dc.BoundaryController(self.node, str(self.path))
        self.pub = self.node.pubs["/nokosahen/travel_boundary"]
        self.clock = [1000.0]
        self._orig_mono = dc.time.monotonic
        dc.time.monotonic = lambda: self.clock[0]

    def tearDown(self):
        dc.time.monotonic = self._orig_mono
        self.tmp.cleanup()

    def override(self, request):
        self.node.subs[mo.BOUNDARY_OVERRIDE_TOPIC](Msg(json.dumps(mo.boundary_override_message(request))))

    def last(self):
        return self.pub.sent[-1]

    def user_set(self):
        self.bc.set_from_request({"min_x": -10.0, "max_x": 10.0, "min_y": -10.0, "max_y": 10.0})

    def test_subscribes_to_the_dedicated_topic_and_is_the_only_publisher(self):
        self.assertIn(mo.BOUNDARY_OVERRIDE_TOPIC, self.node.subs)
        self.assertEqual(set(self.node.pubs), {"/nokosahen/travel_boundary"})

    def test_without_override_nothing_changes(self):
        self.assertFalse(self.last()["enabled"])
        self.user_set()
        self.assertTrue(self.last()["enabled"])
        self.assertNotIn("override_active", self.last())
        self.assertEqual(self.bc.override_info()["active"], False)

    def test_release_then_original_comes_back(self):
        self.user_set()
        self.override(RELEASE)
        self.assertFalse(self.last()["enabled"])
        self.assertTrue(self.last()["override_active"])
        self.assertTrue(self.bc.allows_point(100.0, 100.0))  # 解除中は範囲外の点も許可(購読側と同じ)
        self.override(None)  # 項目が終わった: 上書きなし
        self.assertTrue(self.last()["enabled"])
        self.assertNotIn("override_active", self.last())
        self.assertEqual(self.last()["min_x"], -10.0)
        self.assertFalse(self.bc.allows_point(100.0, 100.0))

    def test_set_replaces_the_range_while_active(self):
        self.user_set()
        self.override(SET)
        self.assertEqual((self.last()["min_x"], self.last()["max_x"]), (0.0, 4.0))
        self.assertFalse(self.bc.allows_point(8.0, 0.0))  # 元の範囲内でも、差し替え中の範囲の外
        self.assertTrue(self.bc.allows_point(2.0, 0.0))
        self.override(None)
        self.assertEqual((self.last()["min_x"], self.last()["max_x"]), (-10.0, 10.0))

    def test_set_works_when_user_has_no_range(self):
        self.override(SET)
        self.assertTrue(self.last()["enabled"])
        self.override(None)
        self.assertFalse(self.last()["enabled"])

    def test_override_expires_after_ttl_when_director_stops(self):
        self.user_set()
        self.override(RELEASE)
        self.assertFalse(self.last()["enabled"])
        self.clock[0] += mo.BOUNDARY_OVERRIDE_TTL_SEC - 1
        self.assertFalse(self.bc.effective_status()["enabled"])  # まだ有効
        self.clock[0] += 2
        self.assertTrue(self.bc.effective_status()["enabled"])   # 期限切れ=元の範囲
        self.assertFalse(self.bc.allows_point(100.0, 100.0))
        # 1秒周期の確認で、配信も元の範囲へ戻る
        period, cb = [t for t in self.node.timers if t[0] == 1.0][0]
        cb()
        self.assertTrue(self.last()["enabled"])
        self.assertNotIn("override_active", self.last())

    def test_refresh_keeps_override_alive_and_does_not_spam(self):
        self.user_set()
        self.override(RELEASE)
        n = len(self.pub.sent)
        for _ in range(10):
            self.clock[0] += 1
            self.override(RELEASE)  # directorの1Hz再送
        self.assertEqual(len(self.pub.sent), n)  # 内容が同じなら再配信しない(5秒周期の再送だけ)
        self.assertFalse(self.bc.effective_status()["enabled"])

    def test_garbage_is_ignored_and_does_not_release(self):
        self.user_set()
        for text in ("not json", "[]", "{}", '{"active": true}', '{"active": true, "enabled": true}',
                     '{"active": true, "enabled": true, "center_x": 0, "center_y": 0, "width": 0, "height": 1}'):
            self.node.subs[mo.BOUNDARY_OVERRIDE_TOPIC](Msg(text))
            self.assertTrue(self.last()["enabled"], text)
            self.assertNotIn("override_active", self.last())

    def test_status_and_file_keep_the_users_range(self):
        self.user_set()
        saved = self.path.read_text(encoding="utf-8")
        self.override(RELEASE)
        self.assertTrue(self.bc.status()["enabled"])
        self.assertEqual(self.bc.status()["min_x"], -10.0)
        self.assertEqual(self.path.read_text(encoding="utf-8"), saved)
        # 再起動(新しいコントローラ)しても元の範囲は上書きの影響を受けない
        bc2 = dc.BoundaryController(SubNode(), str(self.path))
        self.assertTrue(bc2.status()["enabled"])
        self.assertEqual(bc2.status()["min_x"], -10.0)
        self.assertTrue(bc2.effective_status()["enabled"])

    def test_user_change_during_override_applies_after_it_ends(self):
        self.override(RELEASE)
        self.bc.set_from_request({"min_x": -2.0, "max_x": 2.0, "min_y": -2.0, "max_y": 2.0})
        self.assertFalse(self.last()["enabled"])  # 解除中のまま
        self.assertTrue(self.bc.status()["enabled"])
        self.override(None)
        self.assertTrue(self.last()["enabled"])
        self.assertEqual(self.last()["max_x"], 2.0)

    def test_clear_during_override_then_override_ends(self):
        self.user_set()
        self.override(SET)
        self.bc.clear_boundary()
        self.assertEqual(self.last()["max_x"], 4.0)  # 上書き(差し替え)が優先のまま
        self.override(None)
        self.assertFalse(self.last()["enabled"])

    def test_override_info(self):
        self.override(RELEASE)
        info = self.bc.override_info()
        self.assertEqual((info["active"], info["enabled"], info["source"]), (True, False, "schedule"))


if __name__ == "__main__":
    unittest.main()
