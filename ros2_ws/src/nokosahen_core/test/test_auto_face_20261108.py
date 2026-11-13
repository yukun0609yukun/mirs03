"""顔⇄アンケートの自動切替(2026-11-08)。普段は顔、人が来たときだけアンケート。ON/OFFは保存され、既定はOFF。"""
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core import display_hub as dh  # noqa: E402
from nokosahen_core.roles import API_RULES  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def make(tmp, mode="face", auto=True):
    clk = Clock()
    mf = os.path.join(tmp, "mode")
    af = os.path.join(tmp, "auto")
    Path(mf).write_text(mode + "\n")
    hub = dh.DisplayHub(mode_file=mf, auto_file=af, clock=clk)
    if auto:
        hub.set_auto(True)
    return hub, clk, af


def feed(hub, clk, present, seconds, step=1.0):
    n = int(seconds / step)
    for _ in range(n):
        clk.t += step
        hub.note_presence(present)


class AutoFaceTests(unittest.TestCase):
    def test_default_off_and_persisted(self):
        with tempfile.TemporaryDirectory() as d:
            hub, clk, af = make(d, auto=False)
            self.assertFalse(hub.auto_face)
            self.assertTrue(hub.set_auto(True)["ok"])
            hub2 = dh.DisplayHub(mode_file=os.path.join(d, "mode"), auto_file=af, clock=clk)
            self.assertTrue(hub2.auto_face)
            hub2.set_auto(False)
            hub3 = dh.DisplayHub(mode_file=os.path.join(d, "mode"), auto_file=af, clock=clk)
            self.assertFalse(hub3.auto_face)

    def test_only_bool_accepted(self):
        with tempfile.TemporaryDirectory() as d:
            hub, _, _ = make(d, auto=False)
            for bad in ("true", 1, None, "on", []):
                self.assertEqual(hub.set_auto(bad)["code"], 400)
            self.assertFalse(hub.auto_face)

    def test_off_does_nothing(self):
        with tempfile.TemporaryDirectory() as d:
            hub, clk, _ = make(d, auto=False)
            feed(hub, clk, True, 10)
            self.assertEqual(hub.mode, "face")

    def test_person_arrives_then_survey_then_back_to_face(self):
        with tempfile.TemporaryDirectory() as d:
            hub, clk, _ = make(d)
            feed(hub, clk, False, 5)
            self.assertEqual(hub.mode, "face")
            feed(hub, clk, True, 1)          # 1秒では切り替えない(通りすがり)
            self.assertEqual(hub.mode, "face")
            feed(hub, clk, True, 3)
            self.assertEqual(hub.mode, "survey")
            feed(hub, clk, False, 6)         # 8秒未満は戻さない
            self.assertEqual(hub.mode, "survey")
            feed(hub, clk, False, 6)
            self.assertEqual(hub.mode, "face")

    def test_blip_does_not_switch(self):
        with tempfile.TemporaryDirectory() as d:
            hub, clk, _ = make(d)
            for _ in range(5):
                feed(hub, clk, True, 1)
                feed(hub, clk, False, 1)
            self.assertEqual(hub.mode, "face")

    def test_not_back_to_face_while_answering(self):
        with tempfile.TemporaryDirectory() as d:
            hub, clk, _ = make(d)
            feed(hub, clk, True, 3)
            self.assertEqual(hub.mode, "survey")
            self.assertTrue(hub.session.start())
            self.assertEqual(hub.session.phase, "answering")
            feed(hub, clk, False, 20)        # 回答中は、人が見えなくても顔に戻さない
            self.assertEqual(hub.mode, "survey")
            self.assertEqual(hub.session.phase, "answering")

    def test_other_modes_are_left_alone(self):
        with tempfile.TemporaryDirectory() as d:
            for m in ("customer", "inventory", "ocrtest"):
                hub, clk, _ = make(d, mode=m, auto=False)
                hub.auto_face = True               # ONのまま、顔でもアンケートでもない表示は触らない
                feed(hub, clk, True, 5)
                self.assertEqual(hub.mode, m)
                feed(hub, clk, False, 30)
                self.assertEqual(hub.mode, m)

    def test_turning_on_moves_to_face(self):
        with tempfile.TemporaryDirectory() as d:
            hub, _, _ = make(d, mode="customer", auto=False)
            hub.set_auto(True)
            self.assertEqual(hub.mode, "face")

    def test_manual_choice_is_kept_until_presence_changes(self):
        with tempfile.TemporaryDirectory() as d:
            hub, clk, _ = make(d)
            hub.request_mode("survey", "admin", defer=False)
            feed(hub, clk, False, 20)
            self.assertEqual(hub.mode, "face")   # 自動がONなら、人がいなければ顔になる(OFFにすれば固定)
            hub.set_auto(False)
            hub.request_mode("survey", "admin", defer=False)
            feed(hub, clk, False, 60)
            self.assertEqual(hub.mode, "survey")

    def test_admin_state_and_local_admin(self):
        with tempfile.TemporaryDirectory() as d:
            hub, _, _ = make(d, auto=False)
            adm = dh.LocalDisplayAdmin(hub)
            code, body = adm.set_auto({"enabled": True})
            self.assertEqual(code, 200)
            self.assertIs(body["state"]["auto_face"], True)
            code, _ = adm.set_auto({"enabled": "yes"})
            self.assertEqual(code, 400)
            self.assertIs(hub.auto_face, True)

    def test_permission_rule(self):
        self.assertEqual(API_RULES[("POST", "/api/survey/auto")], "display.survey_mode")


if __name__ == "__main__":
    unittest.main()
