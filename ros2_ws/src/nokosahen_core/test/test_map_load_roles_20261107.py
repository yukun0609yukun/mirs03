"""地図の読み込み(map.load)と初期位置の指定(map.initial_pose)を、弱いアカウント(operator)にも許可した確認(2026-11-07)。
PC版(8090)の本物のHTTPハンドラを実際に起動して確認する(地図の操作はPC版にしか無い)。ロボット・ROSには触れない(替え玉が呼び出しを記録するだけ)。
ロール: admin(TL) / operator(許可リストが空でも) / viewer(guest) / 未ログイン。restricted(restricted_account)はスマホ版にこれらのAPIが無く、
権限表(roles.authorize)で据え置き(不可)を確認する。

表: 地図を読み込む x 地図作成開始 x 地図を保存 x 地図リセット x 初期位置の指定 x 自動で位置を探す(グローバル再位置合わせ)
  operator: 読み込み=可、初期位置=可、他=403 / guest=403 / 未ログイン=401 / admin=全部可
(地図の「削除」のAPIは無い。もし足されたら表に無いAPIはadmin専用=operatorは403になる)

実行: python ros2_ws/src/nokosahen_core/test/test_map_load_roles_20261107.py
"""
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import test_account_roles_http as t  # noqa: E402  (スタブ注入・偽サーバー・http()を流用)

dc, roles = t.dc, t.roles
serve, http = t.serve, t.http
PW = {"TL": "tl-pass", "kari-op": t.OP_PASSWORD}


class FakeLifecycle:
    def __init__(self):
        self.calls = []

    def start_mapping(self):
        self.calls.append(("start",))
        return True, "ok"

    def stop(self):
        self.calls.append(("stop",))
        return True, "ok"

    def reset(self):
        self.calls.append(("reset",))
        return True, "ok"

    def load_map(self, name):
        self.calls.append(("load", name))
        return True, "ok"


class FakeMapping:
    def __init__(self):
        self.calls = []

    def save_map(self, name):
        self.calls.append(("save", name))
        return True, "ok"


class FakeLocalization:
    def __init__(self):
        self.calls = []

    def set_initial_pose(self, data):
        self.calls.append(("pose", dict(data)))
        return {"ok": True}

    def global_localization(self):
        self.calls.append(("global",))
        return {"ok": True}


ACTIONS = [
    ("地図を読み込む", "/api/mapping/load", {"name": "festival1"}),
    ("地図作成開始", "/api/mapping/start", {}),
    ("地図を保存", "/api/mapping/save", {"name": "x"}),
    ("地図リセット", "/api/mapping/reset", {}),
    ("初期位置の指定", "/api/initial_pose", {"x": 1.0, "y": 2.0, "yaw_deg": 90}),
    ("自動で位置を探す", "/api/global_localization", {}),
]
EXPECT = {
    "TL": (200,) * 6,
    "kari-op": (200, 403, 403, 403, 200, 403),
    "guest": (403,) * 6,
    None: (401,) * 6,
}


class MapRoles(unittest.TestCase):
    def setUp(self):
        self.lc, self.mp, self.loc = FakeLifecycle(), FakeMapping(), FakeLocalization()
        tmp = t.TMP
        n = time.monotonic_ns()
        af = tmp / f"acc_map_{n}.yaml"
        af.write_text('accounts:\n  - username: "kari-op"\n    role: operator\n'
                      f'    password_hash: "{t.OP_HASH}"\n    enabled: true\n\noperator_permissions: []\n', encoding="utf-8")
        store = t.acc.AccountStore(accounts_path=str(af), permissions_path=str(tmp / f"perm_map_{n}.json"))
        pc = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, t.FakeTeleop(), map_lifecycle=self.lc, mapping=self.mp,
            localization=self.loc, auth_username="TL", auth_password=PW["TL"], enforce_single_controller=False,
            account_store=store, ephemeral=t.eo.EphemeralOverrides(None))
        self.srv, port = serve(pc)
        self.base = f"http://127.0.0.1:{port}"
        self.addCleanup(lambda: (self.srv.shutdown(), self.srv.server_close()))

    def run_table(self, user):
        ck = None
        if user is not None:
            pw = "" if user == "guest" else PW[user]
            code, d, c = http("POST", self.base + "/api/login", {"username": user, "password": pw})
            self.assertEqual(code, 200, user)
            ck = c.split(";")[0]
        row = []
        for (name, path, body), want in zip(ACTIONS, EXPECT[user]):
            code, _d, _ = http("POST", self.base + path, body, ck)
            row.append(code)
            self.assertEqual(code, want, (user, name, code))
        print(f"\n[{user}] {row}")

    def test_admin(self):
        self.run_table("TL")
        self.assertEqual(self.lc.calls, [("load", "festival1"), ("start",), ("reset",)])
        self.assertEqual(self.mp.calls, [("save", "x")])
        self.assertEqual(self.loc.calls, [("pose", {"x": 1.0, "y": 2.0, "yaw_deg": 90}), ("global",)])

    def test_operator_load_and_initial_pose_only(self):
        self.run_table("kari-op")
        # 実際に呼ばれたのは読み込みと初期位置だけ(作成・保存・リセット・グローバル再位置合わせは呼ばれていない)
        self.assertEqual(self.lc.calls, [("load", "festival1")])
        self.assertEqual(self.mp.calls, [])
        self.assertEqual(self.loc.calls, [("pose", {"x": 1.0, "y": 2.0, "yaw_deg": 90})])

    def test_guest(self):
        self.run_table("guest")
        self.assertEqual((self.lc.calls, self.mp.calls, self.loc.calls), ([], [], []))

    def test_not_logged_in(self):
        self.run_table(None)
        self.assertEqual((self.lc.calls, self.mp.calls, self.loc.calls), ([], [], []))

    def test_operator_load_rejects_bad_names_in_controller_only(self):
        # 名前の検証は MapLifecycleController.load_map 側(英数字・_・-のみ)。ここでは権限の関門を通ることだけ確認する
        code, d, c = http("POST", self.base + "/api/login", {"username": "kari-op", "password": PW["kari-op"]})
        ck = c.split(";")[0]
        self.assertEqual(http("POST", self.base + "/api/mapping/load", {"name": "../x"}, ck)[0], 200)  # 替え玉は検証しない=関門は通過
        who = http("GET", self.base + "/api/whoami", None, ck)[1]
        for cap in ("map.load", "map.initial_pose"):
            self.assertIn(cap, who["permissions"])
        for cap in ("map.lifecycle", "map.localize", "map.boundary", "map.pins_edit", "map.view"):
            self.assertNotIn(cap, who["permissions"])

    def test_restricted_unchanged(self):
        for p in ("/api/mapping/load", "/api/initial_pose", "/api/mapping/start", "/api/global_localization"):
            self.assertFalse(roles.authorize("restricted", [], "POST", p, {}).allowed, p)
        self.assertNotIn("map.load", roles.ROLE_BASE_CAPS[roles.ROLE_RESTRICTED])
        self.assertNotIn("map.initial_pose", roles.ROLE_BASE_CAPS[roles.ROLE_RESTRICTED])


class UiMarkers(unittest.TestCase):
    def test_buttons_marked(self):
        import re
        import test_dashboard_field_sync as FS  # rclpy無しでHTMLを組み立てる部品
        html = FS.PAGES["full"]
        self.assertTrue(re.search(r'data-cap="map\.load">\s*<select id="mapLoadSelect">', html))
        self.assertIn('id="poseToolBtn" type="button" class="tb-btn" data-cap="map.initial_pose"', html)
        self.assertIn('id="poseConfirmBar" class="loc-bar" data-cap="map.initial_pose"', html)
        # 作成・保存・リセット・自動で位置を探すには目印を付けない(operatorは無効のまま)
        for ident in ("mapStartBtn", "mapStopBtn", "mapSaveBtn", "mapResetBtn", "globalLocBtn"):
            i = html.index(f'id="{ident}"')
            tag = html[html.rfind("<", 0, i):html.index(">", i)]
            self.assertNotIn("data-cap", tag, ident)


if __name__ == "__main__":
    unittest.main(verbosity=2)
