"""軽量の位置推定モード(2026-11-06)の静的検査。ROS・実機は使わない。"""
import ast
import os
import re
import shutil
import subprocess
import tempfile
import typing
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
LAUNCH = REPO / "ros2_ws/src/nokosahen_bringup/launch/localization_mode.launch.py"
DASH = REPO / "ros2_ws/src/nokosahen_core/nokosahen_core/dashboard_common.py"
FEST = REPO / "scripts/festival"
SRC = LAUNCH.read_text(encoding="utf-8")


def _funcs(path, names):
    """ファイルから指定の関数・定数だけを取り出して実行できる名前空間にする(ROS依存のimportを避ける)。"""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    ns = {"os": os, "Path": Path, "Optional": typing.Optional, "Dict": typing.Dict}
    consts = {"LIGHT_LOCALIZATION_ENV", "LIGHT_LOCALIZATION_FLAG", "LIGHT_MODE_MESSAGE"}
    body = [n for n in tree.body if (isinstance(n, ast.FunctionDef) and n.name in names)
            or (isinstance(n, ast.Assign) and getattr(n.targets[0], "id", "") in consts)]
    exec(compile(ast.Module(body=body, type_ignores=[]), str(path), "exec"), ns)
    return ns


class LaunchTest(unittest.TestCase):
    def test_full_nav_enabled(self):
        f = _funcs(LAUNCH, {"_truthy", "full_nav_enabled"})["full_nav_enabled"]
        self.assertTrue(f("true", "false"))
        self.assertTrue(f("True", ""))
        self.assertFalse(f("true", "true"))
        self.assertFalse(f("true", "1"))
        self.assertFalse(f("false", "false"))
        self.assertFalse(f("false", "true"))

    def test_declares_light_default_false(self):
        self.assertRegex(SRC, r'DeclareLaunchArgument\("light", default_value="false"\)')

    def _node_block(self, name):
        m = re.search(r'Node\(\s*package="nokosahen_core",\s*executable="%s".*?\n            \),' % name, SRC, re.S)
        self.assertIsNotNone(m, name)
        return m.group(0)

    def test_always_nodes_have_no_condition(self):
        for n in ("pose_relay", "motion_arbiter"):
            self.assertNotIn("condition=", self._node_block(n), n)

    def test_nav_nodes_conditioned_on_full_nav(self):
        for n in ("nav2_goal_router", "wall_escape", "person_clear_gate"):
            self.assertIn("condition=IfCondition(full_nav)", self._node_block(n), n)
        self.assertIn("not in ('true','1','yes')", SRC)

    def test_nav2_navigation_include_only_when_full(self):
        i = SRC.index("localization_launch.py")
        j = SRC.index("if use_nav2:")
        k = SRC.index("navigation_launch.py")
        self.assertLess(i, j)
        self.assertLess(j, k)
        self.assertIn("full_nav_enabled(", SRC[SRC.index("def _nav2_includes"):j])


class ControllerTest(unittest.TestCase):
    def setUp(self):
        self.f = _funcs(DASH, {"localization_light_requested"})["localization_light_requested"]
        self.tmp = tempfile.TemporaryDirectory()
        self.flag = str(Path(self.tmp.name) / "flag")

    def tearDown(self):
        self.tmp.cleanup()

    def test_default_off(self):
        self.assertFalse(self.f({}, self.flag))

    def test_env(self):
        for v in ("1", "true", "YES", "on"):
            self.assertTrue(self.f({"NOKOSAHEN_LOCALIZATION_LIGHT": v}, self.flag), v)
        self.assertFalse(self.f({"NOKOSAHEN_LOCALIZATION_LIGHT": "garbage"}, self.flag))

    def test_flag_file(self):
        Path(self.flag).write_text("")
        self.assertTrue(self.f({}, self.flag))
        self.assertFalse(self.f({"NOKOSAHEN_LOCALIZATION_LIGHT": "0"}, self.flag))

    def test_load_map_cmd_static(self):
        src = DASH.read_text(encoding="utf-8")
        m = re.search(r"light = localization_light_requested\(\)\n(.*?)cmd = self\._build_cmd\(launch_args\)", src, re.S)
        self.assertIsNotNone(m)
        self.assertIn('f"map:={map_yaml}"', m.group(1))
        self.assertIn('launch_args.append("light:=true")', m.group(1))
        self.assertIn("if light:", m.group(1))

    def test_goal_guard_rejects_when_light(self):
        src = DASH.read_text(encoding="utf-8")
        g = src[src.index("def _goal_guard"):]
        self.assertLess(g.index("map_lifecycle.is_light()"), g.index("boundary.allows_point"))
        self.assertIn("軽量モードのため走行は使えません", src)


class FestivalScriptTest(unittest.TestCase):
    def test_mode_accepts_word(self):
        self.assertIn("light_localization", (FEST / "festival_mode.sh").read_text(encoding="utf-8"))

    def test_start_options_and_default(self):
        s = (FEST / "festival_start.sh").read_text(encoding="utf-8")
        self.assertIn("--light-localization|light_localization) LIGHT=1", s)
        self.assertIn("LIGHT=0", s)
        self.assertIn("export NOKOSAHEN_LOCALIZATION_LIGHT=1", s)

    def test_dry_run_default_and_flag(self):
        bash = shutil.which("bash")
        if not bash:
            self.skipTest("bashが無い")
        with tempfile.TemporaryDirectory() as h:
            def run(*a):
                e = dict(os.environ, HOME=Path(h).as_posix(), REPO=REPO.as_posix())
                return subprocess.run([bash, (FEST / "festival_start.sh").as_posix(), "--dry-run", "--no-stop", *a],
                                      capture_output=True, text=True, env=e, encoding="utf-8",
                                      stdin=subprocess.DEVNULL).stdout
            self.assertIn("軽量の位置推定モード=false", run())
            self.assertIn("軽量の位置推定モード=true", run("--light-localization"))
            self.assertIn("軽量の位置推定モード=false", run("--light-localization", "--no-light-localization"))


if __name__ == "__main__":
    unittest.main()
