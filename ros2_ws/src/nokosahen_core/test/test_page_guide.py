"""3ページ共通の説明バナー+機体の時計(page_guide)が、既存2ページに足されていることの確認(ROS不要)。"""
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from nokosahen_core.behavior_rules_page import build_behavior_rules_html  # noqa: E402
from nokosahen_core.behavior_tree_page import build_behavior_tree_html  # noqa: E402


class PageGuide(unittest.TestCase):
    def pages(self):
        return {
            "tree": build_behavior_tree_html().decode("utf-8"),
            "rules": build_behavior_rules_html().decode("utf-8"),
        }

    def test_banner_links_and_clock(self):
        for name, html in self.pages().items():
            self.assertNotIn("__PAGE_GUIDE", html, name)
            self.assertNotIn("__NK_FIELD_JS__", html, name)
            for needle in ('id="pgGuide"', 'href="/schedule"', 'href="/behavior_tree"',
                           'href="/behavior_rules"', 'id="pgClockTime"', 'id="pgClockWarn"',
                           "/api/robot_clock", "時計を取得できません", "かんたん時間割"):
                self.assertIn(needle, html, f"{name}: {needle}")
            self.assertEqual(html.count('id="pgClockTime"'), 1)

    def test_existing_ids_kept(self):
        pages = self.pages()
        for i in ("estopBtn", "liveSection", "treeBox", "saveBtn"):
            self.assertIn('id="%s"' % i, pages["tree"])
        for i in ("estopBtn", "liveSection", "previewBody"):
            self.assertIn('id="%s"' % i, pages["rules"])

    def test_js_valid(self):
        node = shutil.which("node")
        if not node:
            self.skipTest("nodeが無い")
        for name, html in self.pages().items():
            js = re.search(r"<script>(.*)</script>", html, re.S).group(1)
            with tempfile.TemporaryDirectory() as d:
                p = Path(d) / "page.js"
                p.write_text(js, encoding="utf-8")
                r = subprocess.run([node, "--check", str(p)], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, name + r.stderr)


if __name__ == "__main__":
    unittest.main()
