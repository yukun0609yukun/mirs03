#!/usr/bin/env python3
"""実機(Jetson1)の読み取り中心のスモークテスト(HTTPだけ。SSHはしない。ロボットは動かさない)。

PC(Wi-Fi「nokosahen」につながっている)から Jetson1 の Web 画面へ HTTP で問い合わせ、「立ち上がっていて、
iPad表示(/display)まで通しで見えるか」を確かめる。標準ライブラリだけ(pip不要)。

使い方:
    python3 -I scripts/pc/smoke_test_jetson1.py [--host 192.0.2.1] [--timeout 5]

環境変数(任意。無い部分は「スキップ(理由)」と出す。値はコード・ログ・出力に書かない/出さない):
    SMOKE_DISPLAY_PASSWORD   iPad表示のログインのパスワード(アカウントは ADMIN_ACCOUNT。厨房サイトと同じ)。無ければ「ログイン後の確認」を飛ばす
    SMOKE_ADMIN_USER / SMOKE_ADMIN_PASSWORD
                             管理画面(8090)のログイン(TL か、権限 display.survey_mode のある operator)。両方あるときだけ、
                             表示モードの切替(POST /api/survey/mode)を試し、**必ず元のモードに戻す**

実機に対して行うこと(すべて安全な操作だけ)
    - GET: 管理画面(8090)・スマホ版(8095)・厨房サイト(8080)の応答、/display のログイン画面、manifest、未ログインの state
    - POST /display/api/login: 間違ったパスワードで1回(拒否されること)、正しいパスワードで1回(Cookie取得)
    - POST /api/survey/mode: 管理画面のログインが与えられ、かつアンケート回答中でないときだけ。元のモードに戻す
    しないこと: アンケートの回答・session start/end、メニュー編集、残数入力、走行・停止・ゴールなどロボットに関わる操作

注意
    - 間違ったパスワードの試行は、このPCのIPアドレスごとの失敗回数に数えられる(5回続けて失敗すると、このPCだけ約5分ロックされる。
      別のiPadや他の端末には影響しない)。1回の実行で数えられるのは1回で、正しいログインに成功すると0に戻る。
    - 管理画面のログインは、本番で「同時1端末制限」が有効なときは、同じアカウントを別の端末で使っている最中だと断られることがある
      (8秒以内にアクセスがあった端末が優先)。そのときは失敗と出る。

終了コード: 0=失敗なし(スキップは失敗ではない)、1=失敗あり、2=Jetson1に届かない
"""
from __future__ import annotations

import argparse
import http.client
import json
import os
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

DISPLAY_USER = "ADMIN_ACCOUNT"
EXPECT_MANIFEST_NAME = "のこさへん 頭部ディスプレイ"
NEW_WORD = "頭部ディスプレイ"
OLD_WORD = "腹部ディスプレイ"
MODES = ("customer", "survey", "inventory", "ocrtest")
SESSION_STATES = ("idle", "answering", "saved", "abandoned")

PASS, FAIL, SKIP = "PASS", "FAIL", "SKIP"


class Resp:
    def __init__(self, status: int, headers: List[Tuple[str, str]], body: bytes) -> None:
        self.status = status
        self.headers = headers
        self.body = body

    def header(self, name: str) -> Optional[str]:
        for k, v in self.headers:
            if k.lower() == name.lower():
                return v
        return None

    def set_cookies(self) -> List[str]:
        return [v for k, v in self.headers if k.lower() == "set-cookie"]

    def text(self) -> str:
        return self.body.decode("utf-8", "replace")

    def json(self) -> Optional[Dict[str, Any]]:
        try:
            d = json.loads(self.text())
            return d if isinstance(d, dict) else None
        except ValueError:
            return None


class Unreachable(Exception):
    pass


class Smoke:
    def __init__(self, host: str, timeout: float) -> None:
        self.host = host
        self.timeout = timeout
        self.admin_port = 8090          # テスト(PC上の模擬サーバ)ではここを差し替える
        self.results: List[Tuple[str, str, str]] = []

    # --- HTTP ---
    def request(self, port: int, method: str, path: str, body: Optional[Dict[str, Any]] = None,
                cookie: Optional[str] = None) -> Resp:
        c = http.client.HTTPConnection(self.host, port, timeout=self.timeout)
        try:
            h = {"Accept": "*/*", "User-Agent": "smoke_test_jetson1"}
            data = None
            if body is not None:
                data = json.dumps(body).encode("utf-8")
                h["Content-Type"] = "application/json"
            if cookie:
                h["Cookie"] = cookie
            c.request(method, path, body=data, headers=h)
            r = c.getresponse()
            return Resp(r.status, r.getheaders(), r.read(2 * 1024 * 1024))
        except (OSError, http.client.HTTPException) as exc:
            raise Unreachable(f"{self.host}:{port} {method} {path}: {type(exc).__name__}: {exc}") from exc
        finally:
            c.close()

    # --- 結果 ---
    def record(self, status: str, name: str, detail: str = "") -> None:
        self.results.append((status, name, detail))
        tail = f" -- {detail}" if detail else ""
        print(f"[{status}] {name}{tail}", flush=True)

    def check(self, name: str, ok: bool, detail: str = "", fail_detail: str = "") -> bool:
        self.record(PASS if ok else FAIL, name, detail if ok else (fail_detail or detail))
        return ok

    def skip(self, name: str, reason: str) -> None:
        self.record(SKIP, name, reason)

    @staticmethod
    def cookie_of(resp: Resp, prefix: str) -> Optional[str]:
        for sc in resp.set_cookies():
            first = sc.split(";", 1)[0].strip()
            if first.startswith(prefix):
                return first
        return None

    # --- 確認 ---
    def run(self, display_pw: str, admin_user: str, admin_pw: str) -> None:
        self.check_ports()
        self.check_display_public()
        cookie = self.check_display_login(display_pw)
        if admin_user and admin_pw:
            self.check_mode_switch(admin_user, admin_pw, cookie)
        else:
            self.skip("モード切替(POST /api/survey/mode)と元に戻す", "環境変数 SMOKE_ADMIN_USER / SMOKE_ADMIN_PASSWORD が未指定")

    def check_ports(self) -> None:
        r = self.request(self.admin_port, "GET", "/")
        self.check("管理画面 8090 / はログインへ302", r.status == 302 and "/login" in (r.header("Location") or ""),
                    f"HTTP {r.status} Location={r.header('Location')}", f"HTTP {r.status} Location={r.header('Location')}(期待: 302 /login)")
        r = self.request(8095, "GET", "/")
        self.check("スマホ版 8095 / はログインへ302", r.status == 302 and "/login" in (r.header("Location") or ""),
                    f"HTTP {r.status} Location={r.header('Location')}", f"HTTP {r.status} Location={r.header('Location')}(期待: 302 /login)")
        r = self.request(8080, "GET", "/")
        self.check("厨房サイト 8080 / はBasic認証で401", r.status == 401 and "Basic" in (r.header("WWW-Authenticate") or ""),
                    f"HTTP {r.status}", f"HTTP {r.status} WWW-Authenticate={r.header('WWW-Authenticate')}(期待: 401 Basic)")
        r = self.request(self.admin_port, "GET", "/api/survey/state")
        self.check("管理API 8090 /api/survey/state は未ログインで401", r.status == 401, f"HTTP {r.status}", f"HTTP {r.status}(期待: 401)")

    def check_display_public(self) -> None:
        r = self.request(self.admin_port, "GET", "/display")
        txt = r.text()
        ok = r.status == 200 and "text/html" in (r.header("Content-Type") or "")
        self.check("/display はログイン画面(200, HTML)", ok, f"HTTP {r.status}", f"HTTP {r.status} Content-Type={r.header('Content-Type')}(期待: 200 text/html)")
        self.check(f"/display のページに「{NEW_WORD}」が出る", NEW_WORD in txt, "", "文言が無い(古いプログラムのまま?)")
        self.check(f"/display のページに旧文言「{OLD_WORD}」が出ていない", OLD_WORD not in txt, "", "旧文言が残っている(最新のコードが配置されていない?)")
        self.check("未ログインのログイン画面に状態(メニュー名・設問)を含まない", "adminPin" not in txt and "NK_SURVEY" not in txt, "", "ログイン画面に内部情報が含まれる")
        r = self.request(self.admin_port, "GET", "/display/api/state")
        j = r.json() or {}
        self.check("未ログインの /display/api/state は401", r.status == 401 and j.get("error") == "auth_required",
                    f"HTTP {r.status}", f"HTTP {r.status} body={r.text()[:80]!r}(期待: 401 auth_required。200ならLAN上の誰でも状態を読める)")
        r = self.request(self.admin_port, "GET", "/display/manifest.webmanifest")
        j = r.json() or {}
        self.check(f"manifest の name は「{EXPECT_MANIFEST_NAME}」", r.status == 200 and j.get("name") == EXPECT_MANIFEST_NAME,
                    f"HTTP {r.status}", f"HTTP {r.status} name={j.get('name')!r}")
        r = self.request(self.admin_port, "GET", "/display/icon.png")
        self.check("/display/icon.png は画像(200)", r.status == 200 and "image/png" in (r.header("Content-Type") or ""),
                    f"HTTP {r.status}", f"HTTP {r.status} Content-Type={r.header('Content-Type')}")
        # 間違ったパスワード(1回だけ。パスワードの値は出さない)
        r = self.request(self.admin_port, "POST", "/display/api/login", {"username": DISPLAY_USER, "password": "smoke-test-wrong-password"})
        j = r.json() or {}
        if r.status == 429:
            self.record(FAIL, "間違ったパスワードは invalid_credentials で拒否", "HTTP 429 (このPCのIPが失敗の繰り返しでロック中。約5分待つか、正しいパスワードで入れば解除)")
        else:
            self.check("間違ったパスワードは invalid_credentials で拒否(Cookieなし)",
                       r.status == 401 and j.get("error") == "invalid_credentials" and not r.set_cookies(),
                       f"HTTP {r.status}", f"HTTP {r.status} error={j.get('error')} cookie={'あり' if r.set_cookies() else 'なし'}")

    def check_display_login(self, pw: str) -> Optional[str]:
        names = "正しいログイン→Cookie取得, /display/api/state の構造"
        if not pw:
            self.skip(f"iPad表示のログイン後の確認({names}, 本体ページの文言)", "環境変数 SMOKE_DISPLAY_PASSWORD が未指定(未実施)")
            return None
        r = self.request(self.admin_port, "POST", "/display/api/login", {"username": DISPLAY_USER, "password": pw})
        j = r.json() or {}
        if r.status == 429:
            self.record(FAIL, "正しいログインでCookie取得", "HTTP 429 (このPCのIPが失敗の繰り返しでロック中。約5分待ってもう一度)")
            return None
        cookie = self.cookie_of(r, "nokosahen_display=")
        attrs = " ".join(r.set_cookies()).lower()
        ok = r.status == 200 and j.get("ok") is True and cookie is not None
        self.check("正しいログインでCookie取得(HTTP 200, ok)", ok, f"HTTP {r.status}",
                   f"HTTP {r.status} error={j.get('error')}(パスワードが違う/アカウントが無効?)")
        if not ok:
            return None
        self.check("Cookie は HttpOnly・Path=/display", "httponly" in attrs and "path=/display" in attrs, "", f"Set-Cookie の属性が想定と違う: {attrs[:120]!r}")
        # 本体の画面
        r = self.request(self.admin_port, "GET", "/display", cookie=cookie)
        txt = r.text()
        self.check("ログイン後の /display は本体の画面(ログイン画面ではない)", r.status == 200 and "/display/api/state" in txt,
                   f"HTTP {r.status}", f"HTTP {r.status}(本体の画面のはずが、状態を読むコードが無い)")
        self.check(f"本体の画面に「{NEW_WORD}」が出て、旧文言「{OLD_WORD}」が出ていない", NEW_WORD in txt and OLD_WORD not in txt, "", "文言が想定と違う")
        # 状態
        r = self.request(self.admin_port, "GET", "/display/api/state", cookie=cookie)
        st = r.json()
        if r.status != 200 or st is None:
            self.record(FAIL, "ログイン後の /display/api/state は200でJSON", f"HTTP {r.status} body={r.text()[:80]!r}")
            return cookie
        self.record(PASS, "ログイン後の /display/api/state は200でJSON", f"HTTP {r.status}")
        mode = st.get("mode")
        self.check("state.mode は既知のモード", mode in MODES, f"mode={mode}", f"mode={mode!r}")
        self.check("state.session_state は既知の値", st.get("session_state") in SESSION_STATES, f"session_state={st.get('session_state')}",
                   f"session_state={st.get('session_state')!r}")
        if st.get("session_state") == "answering":
            print("       (注意: いまアンケートに回答中の人がいる可能性があります)")
        self.check("state に build(画面のプログラムの版)がある", isinstance(st.get("build"), str) and bool(st.get("build")), f"build={st.get('build')}",
                   "build が無い(古いコード?)")
        if mode in ("customer", "inventory"):
            self.check_customer(st.get("customer"))
        else:
            self.check("customer/inventory 以外のモードでは customer を含まない", "customer" not in st, f"mode={mode}", f"mode={mode} なのに customer がある")
        return cookie

    def check_customer(self, c: Any) -> None:
        if not isinstance(c, dict):
            self.record(FAIL, "customer の構造", f"customer が辞書でない: {type(c).__name__}")
            return
        status = c.get("status")
        ok = (isinstance(status, dict) and all(isinstance(status.get(k), str) for k in ("key", "title", "tone"))
              and status.get("tone") in ("ok", "info", "warn", "caution"))
        self.check("customer.status に key / title / tone がある(状態の文言)", ok, f"{status.get('key')}: {status.get('title')} ({status.get('tone')})" if ok else "",
                   f"status={status!r}")
        menu = c.get("menu")
        items = menu.get("items") if isinstance(menu, dict) else None
        if not isinstance(items, list):
            self.record(FAIL, "customer.menu.items(残数)", f"menu={menu!r}")
            return
        bad = [i for i in items if not (isinstance(i, dict) and isinstance(i.get("name"), str)
                                        and all(isinstance(i.get(k), int) and not isinstance(i.get(k), bool) for k in ("initial", "remaining", "sold")))]
        self.check("customer.menu.items は name / initial / remaining / sold の整数", not bad,
                   f"{len(items)}品" + ("(メニュー未登録)" if not items else ""), f"不正な項目: {bad[:2]!r}")
        self.check("menu.promote_item は null か文字", menu.get("promote_item") is None or isinstance(menu.get("promote_item"), str), "",
                   f"promote_item={menu.get('promote_item')!r}")

    def check_mode_switch(self, user: str, pw: str, display_cookie: Optional[str]) -> None:
        name = "モード切替と元に戻す"
        r = self.request(self.admin_port, "POST", "/api/login", {"username": user, "password": pw})
        ck = self.cookie_of(r, "nokosahen_session")
        if r.status != 200 or ck is None:
            j = r.json() or {}
            self.record(FAIL, "管理画面(8090)のログイン", f"HTTP {r.status} error={j.get('error')}(アカウント/パスワードが違う、または別の端末が使用中?)")
            return
        self.record(PASS, "管理画面(8090)のログイン", f"HTTP {r.status}")
        r = self.request(self.admin_port, "GET", "/api/survey/state", cookie=ck)
        j = r.json() or {}
        state = j.get("state") if isinstance(j.get("state"), dict) else None
        if r.status != 200 or state is None or state.get("mode") not in MODES:
            self.record(FAIL, "管理画面の表示モードの取得(GET /api/survey/state)", f"HTTP {r.status} body={r.text()[:100]!r}")
            return
        original = state["mode"]
        self.record(PASS, "管理画面の表示モードの取得", f"現在のモード={original}")
        if state.get("session_state") == "answering" or state.get("active"):
            self.skip(name, "アンケート回答中の人がいるため、切り替えない")
            return
        target = "inventory" if original != "inventory" else "customer"        # アンケートには切り替えない(回答中の通知を受ける画面になるため)
        switched = False
        try:
            r = self.request(self.admin_port, "POST", "/api/survey/mode", {"mode": target}, cookie=ck)
            j = r.json() or {}
            if r.status == 403:
                self.record(FAIL, f"モード切替 {original} -> {target}", "HTTP 403(このアカウントに display.survey_mode の権限が無い)")
                return
            switched = r.status == 200 and j.get("ok") is True
            self.check(f"モード切替 {original} -> {target}", switched, f"HTTP {r.status}",
                       f"HTTP {r.status} ok={j.get('ok')} message={j.get('message')!r}")
            if switched:
                time.sleep(0.5)
                seen = None
                if display_cookie:
                    s2 = self.request(self.admin_port, "GET", "/display/api/state", cookie=display_cookie).json() or {}
                    seen = s2.get("mode")
                    where = "iPad用の /display/api/state"
                else:
                    s2 = (self.request(self.admin_port, "GET", "/api/survey/state", cookie=ck).json() or {}).get("state") or {}
                    seen = s2.get("mode")
                    where = "管理画面の /api/survey/state"
                self.check(f"切替が{where}に反映される", seen == target, f"mode={seen}", f"mode={seen!r}(期待 {target})")
        finally:
            # 必ず元に戻す(切り替えに成功したか不明でも、元のモードを要求する)
            ok = False
            last = ""
            for _ in range(3):
                try:
                    r = self.request(self.admin_port, "POST", "/api/survey/mode", {"mode": original}, cookie=ck)
                    j = r.json() or {}
                    s3 = (self.request(self.admin_port, "GET", "/api/survey/state", cookie=ck).json() or {}).get("state") or {}
                    ok = s3.get("mode") == original
                    last = f"HTTP {r.status} mode={s3.get('mode')}"
                except Unreachable as exc:
                    last = str(exc)
                if ok:
                    break
                time.sleep(1.0)
            self.check(f"元のモード({original})に戻した", ok, last,
                       f"戻せていない({last})。**手動で表示モードを {original} に戻してください**(管理画面の設定タブ)")


def main() -> int:
    ap = argparse.ArgumentParser(description="Jetson1 のスモークテスト(HTTPのみ・ロボットは動かさない)")
    ap.add_argument("--host", default="192.0.2.1", help="Jetson1 のIP(既定 192.0.2.1)")
    ap.add_argument("--timeout", type=float, default=5.0, help="1回のHTTPの待ち時間[秒]")
    args = ap.parse_args()
    display_pw = os.environ.get("SMOKE_DISPLAY_PASSWORD", "")
    admin_user = os.environ.get("SMOKE_ADMIN_USER", "")
    admin_pw = os.environ.get("SMOKE_ADMIN_PASSWORD", "")
    print(f"Jetson1 スモークテスト host={args.host}(読み取り中心・ロボットは動かさない)")
    print("環境変数: "
          f"SMOKE_DISPLAY_PASSWORD={'指定あり' if display_pw else '未指定'}, "
          f"SMOKE_ADMIN_USER/PASSWORD={'指定あり' if admin_user and admin_pw else '未指定'}")
    s = Smoke(args.host, args.timeout)
    try:
        s.run(display_pw, admin_user, admin_pw)
    except Unreachable as exc:
        print(f"[ERROR] Jetson1 に届きません: {exc}")
        print("        PCがWi-Fi「nokosahen」につながっているか、Jetson1が起動しているか、ダッシュボード(8090)が動いているかを確認してください。")
        return 2
    n = {PASS: 0, FAIL: 0, SKIP: 0}
    for st, _, _ in s.results:
        n[st] += 1
    print(f"\n結果: 合格 {n[PASS]} / 失敗 {n[FAIL]} / スキップ {n[SKIP]}")
    for st, name, detail in s.results:
        if st == FAIL:
            print(f"  失敗: {name} -- {detail}")
    return 1 if n[FAIL] else 0


if __name__ == "__main__":
    sys.exit(main())
