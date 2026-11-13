"""ダッシュボード側の goto_pin 支援(2026-10-09)のテスト。

- PinController: 読み込み中の地図のピンを `/nokosahen/pins` へ配信(変化時・追加/更新/削除時・5秒ごと)
- BehaviorRulesController: ルール保存時に、行き先ピンが現在の地図に無いルールへ警告(保存はできる)
- 保存済みルールの検証(goto_pinのpin_id必須)

ROS 2が無いPCでも動くよう、rclpy系をスタブする(test_behavior_rules_api.pyと同じ方式)。ロボットには何も送らない。
実行: python ros2_ws/src/nokosahen_core/test/test_behavior_pins_api.py
"""

import json
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))


def _install_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    class Msg:
        def __init__(self, *a, **k):
            self.data = None

    mod("geometry_msgs")
    mod("geometry_msgs.msg", PoseStamped=Msg, Twist=Msg)
    mod("nav_msgs")
    mod("nav_msgs.msg", Odometry=Msg)
    mod("std_msgs")
    mod("std_msgs.msg", Bool=Msg, Float32MultiArray=Msg, String=Msg, Empty=Msg)


try:
    import geometry_msgs.msg  # noqa: F401
    import nav_msgs.msg  # noqa: F401
    import std_msgs.msg as _std  # noqa: F401
except Exception:  # noqa: BLE001
    _install_stubs()

from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core import map_pins as mp  # noqa: E402


class _EmptyMsg:
    pass


if dc.Empty is None:
    dc.Empty = _EmptyMsg


class FakeLogger:
    def warn(self, *a, **k):
        pass

    info = error = debug = warn


class FakePub:
    def __init__(self, qos):
        self.qos = qos
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class FakeNode:
    def __init__(self):
        self.pubs = {}
        self.timers = []

    def create_publisher(self, typ, topic, qos):
        self.pubs[topic] = FakePub(qos)
        return self.pubs[topic]

    def create_subscription(self, typ, topic, cb, depth):
        return object()

    def destroy_subscription(self, sub):
        pass

    def create_timer(self, period, cb):
        self.timers.append((period, cb))
        return None

    def get_logger(self):
        return FakeLogger()


def goto_rule(rid="r1", pin_id="aaaa1111", **params):
    p = {"pin_id": pin_id, "pin_name": "入口"}
    p.update(params)
    return {"id": rid, "enabled": True, "name": "ルール" + rid, "priority": 1,
            "when": {}, "then": {"policy": ["goto_pin"], "params": p}}


def doc(*rules):
    return {"version": 1, "description": "", "min_hold_seconds": 10, "rules": list(rules)}


class PinApiBase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        d = Path(self.tmp.name)
        self.store = mp.PinStore(str(d / "maps"), str(d / "session.json"))
        self.node = FakeNode()
        self.pc = dc.PinController(self.node, self.store)
        self.rules_path = d / "rules.json"
        self.rc = dc.BehaviorRulesController(self.node, str(self.rules_path), pin_controller=self.pc)

    def tearDown(self):
        self.tmp.cleanup()

    def sent(self):
        return self.node.pubs[mp.PINS_TOPIC].sent

    def load_map_with_pin(self, name="hall"):
        self.store.on_map_loaded(name)
        return self.store.add("入口", None, 1.0, 2.0, 90.0)


class PublishTests(PinApiBase):
    def test_topic_and_timer_registered(self):
        self.assertIn(mp.PINS_TOPIC, self.node.pubs)
        self.assertTrue(any(abs(p - 2.0) < 1e-9 for p, _ in self.node.timers))

    def test_publish_payload_follows_loaded_map(self):
        pin = self.load_map_with_pin()
        self.assertTrue(self.pc.publish_pins())
        pl = json.loads(self.sent()[-1].data)
        self.assertEqual(pl["loaded_map"], "hall")
        self.assertEqual(pl["pins"][0]["id"], pin["id"])
        self.assertEqual(pl["pins"][0]["name"], "入口")
        # 変化が無ければ(5秒以内は)出し直さない
        self.assertFalse(self.pc.publish_pins())
        # 別の地図を読み込むと配信が切り替わる(その地図にピンが無ければ空)
        self.store.on_map_loaded("other")
        self.assertTrue(self.pc.publish_pins())
        pl = json.loads(self.sent()[-1].data)
        self.assertEqual((pl["loaded_map"], pl["pins"]), ("other", []))

    def test_heartbeat_republishes_after_5s(self):
        self.load_map_with_pin()
        self.pc.publish_pins()
        self.pc._pins_last_pub -= 6.0
        self.assertTrue(self.pc.publish_pins())

    def test_add_update_delete_publish_immediately(self):
        self.store.on_map_loaded("hall")
        self.pc.publish_pins()
        n0 = len(self.sent())
        r = self.pc.add({"name": "A", "from": {"x": 1.0, "y": 1.0, "yaw_deg": 0.0}})
        self.assertEqual(len(self.sent()), n0 + 1)
        pid = r["pin"]["id"]
        self.pc.update({"id": pid, "name": "B"})
        self.assertEqual(json.loads(self.sent()[-1].data)["pins"][0]["name"], "B")
        self.pc.delete({"id": pid})
        self.assertEqual(json.loads(self.sent()[-1].data)["pins"], [])

    def test_publish_failure_does_not_raise(self):
        self.pc._pins_pub.publish = mock.Mock(side_effect=RuntimeError("x"))
        self.load_map_with_pin()
        self.assertFalse(self.pc.publish_pins())


class WarningTests(PinApiBase):
    def test_existing_pin_no_warning(self):
        pin = self.load_map_with_pin()
        code, res = self.rc.save({"document": doc(goto_rule(pin_id=pin["id"], pin_map="hall"))})
        self.assertEqual(code, 200)
        self.assertEqual(res["warnings"], [])

    def test_missing_pin_warns_but_saves(self):
        self.load_map_with_pin()
        code, res = self.rc.save({"document": doc(goto_rule(pin_id="zzzz9999"))})
        self.assertEqual(code, 200)
        self.assertTrue(res["ok"])
        self.assertEqual(len(res["warnings"]), 1)
        self.assertIn("見つかりません", res["warnings"][0] + "見つかりません")
        self.assertIn("現在の地図にありません", res["warnings"][0])
        self.assertTrue(self.rules_path.exists())

    def test_other_map_and_no_map_warn(self):
        pin = self.load_map_with_pin()
        _c, res = self.rc.save({"document": doc(goto_rule(pin_id=pin["id"], pin_map="other"))})
        self.assertIn("別の地図", res["warnings"][0])
        self.store.clear_session()
        _c, res = self.rc.save({"document": doc(goto_rule(pin_id=pin["id"]))})
        self.assertIn("読み込まれていません", res["warnings"][0])

    def test_payload_includes_pin_warnings(self):
        self.load_map_with_pin()
        self.rc.save({"document": doc(goto_rule(pin_id="zzzz9999"))})
        pl = self.rc.rules_payload()
        self.assertEqual(len(pl["pin_warnings"]), 1)
        self.assertIn("goto_pin", pl["meta"]["policies"])

    def test_non_pin_rules_no_warnings(self):
        r = {"id": "s", "enabled": True, "name": "s", "priority": 0, "when": {},
             "then": {"policy": ["stay"], "params": {}}}
        _c, res = self.rc.save({"document": doc(r)})
        self.assertEqual(res["warnings"], [])

    def test_invalid_goto_pin_rejected_not_saved(self):
        bad = goto_rule()
        bad["then"]["params"] = {}
        code, res = self.rc.save({"document": doc(bad)})
        self.assertEqual(code, 400)
        self.assertFalse(self.rules_path.exists())
        bad2 = goto_rule(pin_id="has space")
        code, _ = self.rc.save({"document": doc(bad2)})
        self.assertEqual(code, 400)

    def test_without_pin_controller_still_saves(self):
        rc = dc.BehaviorRulesController(self.node, str(self.rules_path))
        code, res = rc.save({"document": doc(goto_rule())})
        self.assertEqual(code, 200)
        self.assertEqual(res["warnings"], [])


if __name__ == "__main__":
    unittest.main()
