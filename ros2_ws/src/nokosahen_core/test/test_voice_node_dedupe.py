"""voice_node の発話依頼の重複排除のテスト(2026-10-21、全体レビューの指摘)。rclpy・requestsはスタブ(実物があっても使わない)。

- 内容が同じでも、DEDUP_WINDOW_SEC(5秒)を過ぎた次の周期の依頼は送る(以前は永久に送らず、ロボットが黙っていた)
- 5秒以内の同じ内容の再送は、重複として捨てる
- 送信に失敗した・音声が全体OFFだった依頼は「送信済み」と記録しない(復旧後に同じ内容でも送る)
実行: python ros2_ws/src/nokosahen_core/test/test_voice_node_dedupe.py
"""
import importlib.util
import json
import sys
import types
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
SRC = REPO / "ros2_ws" / "src" / "nokosahen_voice" / "nokosahen_voice" / "voice_node.py"


def load_module(posts, fail=False):
    """voice_node.py を、スタブのrclpy/requests/std_msgsで読み込む(posts: 送信された本文のリスト)。"""
    req = types.ModuleType("requests")

    class _Resp:
        def raise_for_status(self):
            if fail:
                raise RuntimeError("500")

    def post(url, json=None, timeout=None):  # noqa: A002
        posts.append(json)
        return _Resp()

    req.post = post
    req.get = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("no"))
    rclpy = types.ModuleType("rclpy")
    node = types.ModuleType("rclpy.node")
    node.Node = type("Node", (), {})
    std = types.ModuleType("std_msgs")
    stdm = types.ModuleType("std_msgs.msg")
    stdm.Bool = type("Bool", (), {})
    stdm.String = type("String", (), {})
    saved = {k: sys.modules.get(k) for k in ("requests", "rclpy", "rclpy.node", "std_msgs", "std_msgs.msg")}
    sys.modules.update({"requests": req, "rclpy": rclpy, "rclpy.node": node, "std_msgs": std, "std_msgs.msg": stdm})
    try:
        spec = importlib.util.spec_from_file_location("voice_node_under_test", str(SRC))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
    return mod


class _Msg:
    def __init__(self, data):
        self.data = data


class _Log:
    def __init__(self):
        self.lines = []

    def warn(self, m):
        self.lines.append(("warn", m))

    def debug(self, m):
        self.lines.append(("debug", m))

    def info(self, m):
        self.lines.append(("info", m))


def make(mod):
    n = object.__new__(mod.VoiceNode)
    n.voice_bridge_url = "http://x:8100"
    n.generate = True
    n.timeout = 1.0
    n._last_context_raw = ""
    n._last_context_time = float("-inf")
    n._voice_enabled = True
    n.get_logger = lambda: _Log()
    return n


CTX = json.dumps({"phase": "initial", "menu_items": ["カレー"], "fallback_text": "本日のカレーは…"})


class DedupeTest(unittest.TestCase):
    def test_same_content_is_sent_again_after_the_window(self):
        posts = []
        mod = load_module(posts)
        n = make(mod)
        t = [100.0]
        mod.time = types.SimpleNamespace(monotonic=lambda: t[0])
        n._on_announce_context(_Msg(CTX))
        self.assertEqual(len(posts), 1)
        t[0] += 2.0  # 5秒以内の同じ内容 → 重複として捨てる
        n._on_announce_context(_Msg(CTX))
        self.assertEqual(len(posts), 1)
        t[0] += 60.0  # 次の周期(同じ内容) → 送る(以前は永久に送らなかった)
        n._on_announce_context(_Msg(CTX))
        self.assertEqual(len(posts), 2)

    def test_different_content_is_sent_immediately(self):
        posts = []
        mod = load_module(posts)
        n = make(mod)
        mod.time = types.SimpleNamespace(monotonic=lambda: 100.0)
        n._on_announce_context(_Msg(CTX))
        n._on_announce_context(_Msg(json.dumps({"phase": "induction", "menu_items": [], "fallback_text": "x"})))
        self.assertEqual(len(posts), 2)

    def test_failed_send_is_not_recorded_as_sent(self):
        posts = []
        mod = load_module(posts, fail=True)
        n = make(mod)
        mod.time = types.SimpleNamespace(monotonic=lambda: 100.0)
        n._on_announce_context(_Msg(CTX))
        n._on_announce_context(_Msg(CTX))  # 失敗していたので、同じ内容でもすぐに再送される
        self.assertEqual(len(posts), 2)
        self.assertEqual(n._last_context_raw, "")

    def test_master_off_is_not_recorded_as_sent(self):
        posts = []
        mod = load_module(posts)
        n = make(mod)
        mod.time = types.SimpleNamespace(monotonic=lambda: 100.0)
        n._voice_enabled = False
        n._on_announce_context(_Msg(CTX))
        self.assertEqual(posts, [])
        n._voice_enabled = True  # ONに戻ったあと、同じ内容の依頼は送る
        n._on_announce_context(_Msg(CTX))
        self.assertEqual(len(posts), 1)

    def test_empty_and_invalid_are_ignored(self):
        posts = []
        mod = load_module(posts)
        n = make(mod)
        mod.time = types.SimpleNamespace(monotonic=lambda: 100.0)
        for bad in ("", "   ", "not json", "[1,2]"):
            n._on_announce_context(_Msg(bad))
        self.assertEqual(posts, [])


if __name__ == "__main__":
    unittest.main(verbosity=1)
