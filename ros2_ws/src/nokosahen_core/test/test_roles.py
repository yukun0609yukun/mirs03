"""ロール(権限レベル)と権限(capability)の仕組みのテスト(2026-10-13追加、docs/accounts.md)。

rclpy・ネットワーク不要(純Python)。
- ロール表の整合(operator <= admin、admin_onlyは付与不可、デフォルト拒否)
- 許可リストの検証(admin_only / deferred / 未知の名前は付与されない。strictなら例外)
- 緊急停止: かける=全員可 / 解除=admin・operator(2026-11-06〜。restricted・viewerは不可)
- 全書き込みAPIの分類の網羅: dashboard_common.py の do_POST にある全パスが分類表に載っていること
  (表に無いAPIはadmin専用になること、表の古い項目が残っていないこと)

実行: python ros2_ws/src/nokosahen_core/test/test_roles.py
"""
import re
import sys
import unittest
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from nokosahen_core import roles  # noqa: E402

SRC = (PKG / "nokosahen_core" / "dashboard_common.py").read_text(encoding="utf-8")


def post_bodies():
    """dashboard_common.py の各 do_POST の本文(PC版・スマホ版)を返す。"""
    out = []
    for m in re.finditer(r"        def do_POST\(self\) -> None:.*?\n    return Handler\n", SRC, re.S):
        out.append(m.group(0))
    return out


class RoleTableTest(unittest.TestCase):
    def test_tables_validate(self):
        roles.validate_tables()  # 例外が出なければ整合している

    def test_rank_order(self):
        r = roles.ROLE_RANK
        self.assertLess(r[roles.ROLE_VIEWER], r[roles.ROLE_OPERATOR])
        self.assertLess(r[roles.ROLE_OPERATOR], r[roles.ROLE_ADMIN])

    def test_operator_never_exceeds_admin(self):
        admin = roles.role_caps(roles.ROLE_ADMIN)
        # 許可しうる最大(付与可能な全部)でもadminを超えない
        everything = roles.role_caps(roles.ROLE_OPERATOR, list(roles.CAPABILITIES))
        self.assertTrue(everything <= admin)
        self.assertFalse(everything & roles.NOT_GRANTABLE)
        self.assertEqual(everything, roles.SAFE_FOR_ALL | roles.GRANTABLE | roles.ROLE_DEFAULT)

    def test_viewer_has_nothing(self):
        self.assertEqual(roles.role_caps(roles.ROLE_VIEWER, list(roles.CAPABILITIES)), frozenset())

    def test_default_operator_is_safe_only(self):
        # 【2026-11-06】+「人の方を向く」スイッチ(ロールの既定)
        self.assertEqual(roles.role_caps(roles.ROLE_OPERATOR), roles.SAFE_FOR_ALL | roles.ROLE_DEFAULT)
        self.assertEqual(roles.ROLE_DEFAULT, {"behavior.face_switch", "robot.manual_drive", "robot.estop_release", "map.load", "map.initial_pose"})  # 2026-11-06: 手動運転もロール既定
        self.assertEqual(roles.SAFE_FOR_ALL, {"session.logout", "safe.estop_engage", "safe.goal_cancel",
                                              "safe.schedule_stop"})

    def test_tiers_partition(self):
        every = set(roles.CAPABILITIES)
        parts = [roles.GRANTABLE, roles.DEFERRED, roles.ADMIN_ONLY, roles.SAFE_FOR_ALL, roles.ROLE_DEFAULT]
        self.assertEqual(set().union(*parts), every)
        self.assertEqual(sum(len(p) for p in parts), len(every))

    def test_critical_capabilities_are_admin_only_locked(self):
        # ロボットの根幹・安全に関わるもの(コードで固定)
        # 【2026-11-06】robot.manual_drive はロール既定(operator・restrictedが持つ)に移した。ここには含めない(下で別途確認)
        for cap in ("robot.estop_config", "robot.speed", "robot.pid_firmware",
                    "robot.exploration", "nav.autonomy_resume", "map.pins_edit", "map.localize", "map.view",
                    "map.lifecycle", "map.boundary", "map.keepout", "behavior.rules_edit", "behavior.tree_edit",
                    "behavior.control", "crowd.calibrate", "camera.assign", "accounts.manage",
                    "ephemeral.manage", "system.single_controller", "system.secrets", "admin.unclassified"):
            self.assertIn(cap, roles.ADMIN_ONLY, cap)

    def test_manual_drive_is_role_default_not_grantable_list(self):
        self.assertIn("robot.manual_drive", roles.ROLE_DEFAULT)
        self.assertNotIn("robot.manual_drive", roles.ADMIN_ONLY)
        self.assertNotIn("robot.manual_drive", roles.GRANTABLE)
        for r in (roles.ROLE_ADMIN, roles.ROLE_OPERATOR, roles.ROLE_RESTRICTED):
            self.assertIn("robot.manual_drive", roles.role_caps(r), r)
        self.assertNotIn("robot.manual_drive", roles.role_caps(roles.ROLE_VIEWER, list(roles.CAPABILITIES)))
        # operatorに増えたのは手動運転だけ。緊急停止の解除・速度・PID・探索・自律走行へ戻す等はadmin専用のまま
        # (緊急停止の解除は2026-11-06にoperatorへ許可。設定変更 robot.estop_config は管理者専用のまま)
        for cap in ("robot.estop_config", "robot.speed", "robot.pid_firmware", "robot.exploration", "nav.autonomy_resume",
                    "behavior.control", "map.pins_edit", "map.localize", "map.boundary", "accounts.manage"):
            self.assertNotIn(cap, roles.role_caps(roles.ROLE_OPERATOR, list(roles.CAPABILITIES)), cap)

    def test_every_capability_has_description(self):
        for c in roles.CAPABILITIES.values():
            self.assertTrue(c.label, c.name)
            if c.tier != roles.TIER_SAFE:
                self.assertTrue(c.risk, c.name)
            if c.tier == roles.TIER_GRANTABLE:
                self.assertIn(c.recommend, ("許可", "保留"), c.name)


class SanitizeTest(unittest.TestCase):
    def test_grantable_accepted(self):
        got, problems = roles.sanitize_operator_permissions(["voice.levels", "display.survey_mode"])
        self.assertEqual(got, {"voice.levels", "display.survey_mode"})
        self.assertEqual(problems, [])

    def test_admin_only_deferred_and_unknown_are_not_granted(self):
        got, problems = roles.sanitize_operator_permissions(
            ["voice.levels", "robot.speed", "voice.bgm_onoff", "nope.nope", "accounts.manage"])
        self.assertEqual(got, {"voice.levels"})
        self.assertEqual(len(problems), 4)
        # operatorのロール権限にも入らない
        caps = roles.role_caps(roles.ROLE_OPERATOR, ["robot.speed", "accounts.manage", "voice.bgm_onoff"])
        self.assertEqual(caps, roles.SAFE_FOR_ALL | roles.ROLE_DEFAULT)

    def test_strict_raises(self):
        with self.assertRaises(roles.PermissionConfigError):
            roles.sanitize_operator_permissions(["robot.speed"], strict=True)
        with self.assertRaises(roles.PermissionConfigError):
            roles.sanitize_operator_permissions(["unknown"], strict=True)
        roles.sanitize_operator_permissions(["voice.master"], strict=True)

    def test_empty_and_none(self):
        self.assertEqual(roles.sanitize_operator_permissions([])[0], frozenset())
        self.assertEqual(roles.sanitize_operator_permissions(None)[0], frozenset())


class AuthorizeTest(unittest.TestCase):
    def auth(self, role, perms, path, data=None):
        return roles.authorize(role, perms, "POST", path, data)

    def test_admin_allows_everything_even_unknown(self):
        for path in ("/api/teleop", "/api/speed_settings", "/api/never_seen_before"):
            self.assertTrue(self.auth("admin", [], path, {}).allowed, path)

    def test_default_deny_for_operator(self):
        # 何も許可していない: 付与可能な操作も全て拒否
        for (m, path), rule in roles.API_RULES.items():
            caps = roles.required_capabilities(m, path, {})
            allowed = self.auth("operator", [], path, {}).allowed
            if all(c in (roles.SAFE_FOR_ALL | roles.ROLE_DEFAULT) for c in caps):
                self.assertTrue(allowed, path)
            else:
                self.assertFalse(allowed, path)

    def test_unknown_api_is_admin_only(self):
        d = self.auth("operator", list(roles.GRANTABLE), "/api/new_dangerous_thing", {})
        self.assertFalse(d.allowed)
        self.assertTrue(d.admin_only)
        self.assertEqual(roles.required_capabilities("POST", "/api/new_dangerous_thing"), ["admin.unclassified"])
        # GETは表に無い=書き込みとして扱わない(呼び出し側でGETは認可対象外)。メソッド違いも未分類
        self.assertEqual(roles.required_capabilities("PUT", "/api/voice_levels"), ["admin.unclassified"])

    def test_grant_one_at_a_time(self):
        self.assertFalse(self.auth("operator", [], "/api/voice_levels", {"speech_volume": 2}).allowed)
        self.assertTrue(self.auth("operator", ["voice.levels"], "/api/voice_levels", {"speech_volume": 2}).allowed)
        # 別の権限は増えない
        self.assertFalse(self.auth("operator", ["voice.levels"], "/api/voice_master", {"enabled": False}).allowed)
        self.assertFalse(self.auth("operator", ["voice.levels"], "/api/survey/mode", {"mode": "survey"}).allowed)

    def test_admin_only_in_permissions_does_not_grant(self):
        # 【2026-11-06】/api/teleop(手動運転)はロール既定になったので、ここからは外した(下の test_operator_can_drive で確認)
        for path in ("/api/speed_settings", "/api/pid_gains", "/api/exploration/start", "/api/resume_autonomy", "/api/boundary/set", "/api/behavior_tree",
                     "/api/global_localization", "/api/mapping/reset", "/api/mapping/save", "/api/mapping/start", "/api/accounts/permissions"):  # 2026-11-07: initial_pose・mapping/loadはoperatorも可
            d = self.auth("operator", list(roles.CAPABILITIES), path, {})
            self.assertFalse(d.allowed, path)
            self.assertTrue(d.admin_only, path)

    def test_operator_can_drive_even_with_empty_permissions_viewer_cannot(self):
        self.assertTrue(self.auth("operator", [], "/api/teleop", {"forward": 1}).allowed)
        self.assertTrue(self.auth("restricted", [], "/api/teleop", {"forward": 1}).allowed)
        self.assertFalse(self.auth("viewer", list(roles.CAPABILITIES), "/api/teleop", {"forward": 1}).allowed)

    def test_operator_can_load_map_and_set_initial_pose_only(self):
        # 【2026-11-07】読み込みと初期位置の指定だけ。作成・保存・リセット・グローバル再位置合わせ・範囲・ピン・表示回転は管理者専用のまま
        for p in ("/api/mapping/load", "/api/initial_pose"):
            self.assertTrue(self.auth("operator", [], p, {}).allowed, p)
            self.assertFalse(self.auth("restricted", [], p, {}).allowed, p)
            self.assertFalse(self.auth("viewer", list(roles.CAPABILITIES), p, {}).allowed, p)
        for p in ("/api/mapping/start", "/api/mapping/stop", "/api/mapping/save", "/api/mapping/reset", "/api/global_localization",
                  "/api/boundary/set", "/api/pins/add", "/api/map_view/save", "/api/keepout_zones/add"):
            self.assertFalse(self.auth("operator", list(roles.CAPABILITIES), p, {}).allowed, p)
        self.assertEqual(roles.required_capabilities("POST", "/api/mapping/load", {}), ["map.load"])
        self.assertEqual(roles.required_capabilities("POST", "/api/initial_pose", {}), ["map.initial_pose"])
        self.assertIn("map.lifecycle", roles.ADMIN_ONLY)
        self.assertIn("map.localize", roles.ADMIN_ONLY)

    def test_estop_engage_for_all_release_admin_and_operator(self):
        self.assertTrue(self.auth("operator", [], "/api/soft_estop", {"engaged": True}).allowed)
        self.assertTrue(self.auth("operator", [], "/api/soft_estop", {}).allowed)  # 既定=かける(従来の解釈)
        # 【2026-11-06】解除はoperatorも可(許可リストが空でも)。restricted・viewerは不可。設定変更(robot.estop_config)はadmin専用
        self.assertTrue(self.auth("operator", [], "/api/soft_estop", {"engaged": False}).allowed)
        for role in ("restricted", "viewer"):
            self.assertFalse(self.auth(role, list(roles.CAPABILITIES), "/api/soft_estop", {"engaged": False}).allowed, role)
        self.assertIn("robot.estop_release", roles.ROLE_DEFAULT)
        self.assertIn("robot.estop_config", roles.ADMIN_ONLY)
        self.assertNotIn("robot.estop_config", roles.role_caps(roles.ROLE_OPERATOR, list(roles.CAPABILITIES)))
        # 厳密判定は維持: engaged が false 以外(null・0・""・[]・"false"・無し)は「かける」
        for body in ({}, {"engaged": None}, {"engaged": 0}, {"engaged": ""}, {"engaged": []}, {"engaged": "false"}):
            self.assertEqual(roles.required_capabilities("POST", "/api/soft_estop", body), ["safe.estop_engage"], body)
        self.assertTrue(self.auth("admin", [], "/api/soft_estop", {"engaged": False}).allowed)

    def test_goal_cancel_and_logout_for_operator(self):
        self.assertTrue(self.auth("operator", [], "/api/goal/cancel", {}).allowed)
        self.assertTrue(self.auth("operator", [], "/api/logout", {}).allowed)

    def test_bgm_enabled_needs_deferred_cap(self):
        # 音量だけなら voice.levels
        self.assertTrue(self.auth("operator", ["voice.levels"], "/api/bgm", {"volume": 1}).allowed)
        # ON/OFFは現状付与不可(一時的反映が未実装)
        d = self.auth("operator", list(roles.CAPABILITIES), "/api/bgm", {"enabled": False})
        self.assertFalse(d.allowed)
        self.assertIn("voice.bgm_onoff", d.missing)

    def test_voice_settings_save_needs_each_part(self):
        body = {"announce": {"announce_min_interval_seconds": 60}}
        self.assertFalse(self.auth("operator", ["voice.announce_text"], "/api/voice_settings/save", body).allowed)
        self.assertTrue(self.auth("operator", ["voice.announce_interval"], "/api/voice_settings/save", body).allowed)
        both = {"announce": {}, "voice": {"temperature": 0.5}}
        self.assertFalse(self.auth("operator", ["voice.announce_interval"], "/api/voice_settings/save", both).allowed)
        self.assertTrue(self.auth("operator", ["voice.announce_interval", "voice.announce_text"],
                                  "/api/voice_settings/save", both).allowed)

    def test_schedule_permissions(self):
        # 実行は許可リストでoperatorに付与できる(既定=保留)。編集・保存・検証は管理者のみ。停止は止める方向なので全員可。
        self.assertIn("schedule.run", roles.GRANTABLE)
        self.assertEqual(roles.CAPABILITIES["schedule.run"].recommend, "保留")
        self.assertFalse(self.auth("operator", [], "/api/schedule/run", {}).allowed)
        self.assertTrue(self.auth("operator", ["schedule.run"], "/api/schedule/run", {}).allowed)
        self.assertTrue(self.auth("operator", [], "/api/schedule/stop", {}).allowed)
        self.assertIn("safe.schedule_stop", roles.SAFE_FOR_ALL)
        for p in ("/api/schedule", "/api/schedule/preview"):
            d = self.auth("operator", list(roles.CAPABILITIES), p, {})
            self.assertFalse(d.allowed, p)
            self.assertTrue(d.admin_only, p)
            self.assertTrue(self.auth("admin", [], p, {}).allowed)
        self.assertTrue(self.auth("admin", [], "/api/schedule/run", {}).allowed)
        self.assertFalse(self.auth("viewer", ["schedule.run"], "/api/schedule/run", {}).allowed)

    def test_goal_free_point_is_not_grantable(self):
        d = self.auth("operator", ["nav.goto_pin", "nav.goto_point"], "/api/goal", {"x": 1, "y": 1})
        self.assertFalse(d.allowed)
        self.assertTrue(self.auth("operator", ["nav.goto_pin"], "/api/pins/goto", {"id": "a"}).allowed)

    def test_dynamic_rule_with_garbage_body_is_safe(self):
        self.assertEqual(roles.required_capabilities("POST", "/api/soft_estop", None), ["safe.estop_engage"])
        self.assertEqual(roles.required_capabilities("POST", "/api/bgm", "oops"), ["voice.levels"])


class ClassificationCoverageTest(unittest.TestCase):
    """全書き込みAPI(do_POSTの全パス)が分類表に載っている / 意図的にadmin専用と分類されている。"""

    def test_found_two_do_post(self):
        self.assertEqual(len(post_bodies()), 2, "PC版とスマホ版のdo_POSTが見つかりません")

    def test_every_post_path_is_classified(self):
        paths = set()
        for body in post_bodies():
            paths |= set(re.findall(r'"(/api/[A-Za-z0-9_/]+)"', body))
        paths -= roles.PUBLIC_POST_PATHS
        self.assertGreater(len(paths), 40)
        table = {p for (m, p) in roles.API_RULES if m == "POST"}
        missing = sorted(paths - table)
        self.assertEqual(missing, [], f"分類表に無い書き込みAPI(admin専用扱いになるが、意図を明示するため表に追加すること): {missing}")

    def test_no_stale_table_entries(self):
        paths = set()
        for body in post_bodies():
            paths |= set(re.findall(r'"(/api/[A-Za-z0-9_/]+)"', body))
        # 表の項目はどれも実在するAPI(実装の削除・改名で古い項目が残らない)
        for (m, p) in roles.API_RULES:
            if p in ("/api/accounts/permissions", "/api/ephemeral/clear"):
                self.assertIn(p, SRC)
                continue
            self.assertIn(p, paths, f"表にあるが do_POST に無い: {p}")

    def test_grantable_caps_are_each_used_by_some_api(self):
        used = set()
        for key in roles.API_RULES:
            used |= set(roles._rule_static_caps(roles.API_RULES[key]))
        for cap in roles.GRANTABLE | roles.DEFERRED:
            self.assertIn(cap, used, cap)

    def test_robot_core_apis_are_not_grantable(self):
        core = ["/api/speed_settings",  # /api/teleop は2026-11-06から弱いアカウントも可
                 "/api/pid_gains", "/api/drive_test", "/api/step_response/run",
                "/api/resume_autonomy", "/api/global_localization", "/api/map_view/set",
                "/api/map_view/save", "/api/mapping/start", "/api/mapping/stop", "/api/mapping/reset",
                "/api/mapping/save", "/api/boundary/set", "/api/boundary/clear",
                "/api/keepout_zones/add", "/api/keepout_zones/remove", "/api/keepout_zones/clear",
                "/api/exploration/start", "/api/exploration/stop", "/api/behavior_rules/save",
                "/api/behavior_rules/reload", "/api/behavior_tree", "/api/behavior_tree/reset",
                "/api/behavior_tree/validate", "/api/crowd_background/calibrate", "/api/checkout_cam_assign",
                "/api/schedule", "/api/schedule/preview",
                "/api/recommend_enabled", "/api/idle_wiggle", "/api/person_interaction",
                "/api/scan_enabled", "/api/force_demo", "/api/pins/add",  # face_enabled は2026-11-06から弱いアカウントも可
                "/api/pins/update", "/api/pins/delete", "/api/accounts/permissions", "/api/ephemeral/clear",
                "/api/goal"]
        for p in core:
            caps = roles.required_capabilities("POST", p, {})
            self.assertTrue(all(c in roles.NOT_GRANTABLE for c in caps), (p, caps))
            d = roles.authorize("operator", list(roles.CAPABILITIES), "POST", p, {})
            self.assertFalse(d.allowed, p)


if __name__ == "__main__":
    unittest.main(verbosity=1)
