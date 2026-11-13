"""分散構成(persistent_base_control.launch.py)に、後ろ向きカメラの人物検出(rear_camera_persons)が入っていることの静的検査。
core.launch.py と同じ条件(use_person_target_detector と use_rear_cameras の両方がtrue)で起動する。ROSは不要。
"""
import ast
import unittest
from pathlib import Path

LAUNCH = Path(__file__).resolve().parents[2] / "nokosahen_bringup" / "launch"
CONTROL = LAUNCH / "persistent_base_control.launch.py"
CORE = LAUNCH / "core.launch.py"


def node_block(text, executable):
    """Node( ... executable="X" ... ) の呼び出し部分の文字列を返す。"""
    tree = ast.parse(text)
    lines = text.splitlines()
    for n in ast.walk(tree):
        if isinstance(n, ast.Call) and getattr(n.func, "id", "") == "Node":
            for kw in n.keywords:
                if kw.arg == "executable" and isinstance(kw.value, ast.Constant) and kw.value.value == executable:
                    return "\n".join(lines[n.lineno - 1:n.end_lineno])
    return None


class ControlLaunchRearCamerasTest(unittest.TestCase):
    def setUp(self):
        self.text = CONTROL.read_text(encoding="utf-8")

    def test_rear_camera_persons_node_exists_next_to_person_target_detector(self):
        self.assertIsNotNone(node_block(self.text, "rear_camera_persons"), "rear_camera_personsが起動されていない")
        self.assertLess(self.text.index('executable="person_target_detector"'),
                        self.text.index('executable="rear_camera_persons"'))

    def test_use_rear_cameras_declared_default_true_and_used_in_condition(self):
        self.assertIn('DeclareLaunchArgument("use_rear_cameras", default_value="true")', self.text)
        block = node_block(self.text, "rear_camera_persons")
        self.assertIn("use_rear_cameras", block)
        self.assertIn("use_person_target_detector", block)
        self.assertIn("PythonExpression", self.text.split("def ")[0])  # importされている

    def test_same_condition_as_core_launch(self):
        def cond(text):
            b = node_block(text, "rear_camera_persons")
            self.assertIsNotNone(b)
            return [l.strip() for l in b.splitlines() if "condition=" in l][0]
        self.assertEqual(cond(self.text), cond(CORE.read_text(encoding="utf-8")))

    def test_python_syntax(self):
        ast.parse(self.text)


if __name__ == "__main__":
    unittest.main(verbosity=1)
