#!/usr/bin/env python3
"""動作確認・デモ用: 明電舎タブレットの累計販売数入力の代わりに、ランダムな
数値を一定間隔で/api/sold_totalへ送り続け、customer_displayの残数表示が
実際に変化する様子を確認できるようにするダミーデータ生成スクリプト。

本番用ではない(tablet_ocr_reader.pyの代わりにはならない)。実行中は
/nokosahen/sold_total_eventが本物のタブレット読み取り値のように上書きされ
続けるため、本番運用中は使わないこと。

使い方:
    python3 dummy_sales_feed.py
    python3 dummy_sales_feed.py --base-url http://192.0.2.250:8080 --interval 3
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import random
import time
import urllib.error
import urllib.request


def get_items(base_url: str, auth_header: str) -> list:
    req = urllib.request.Request(
        f"{base_url}/api/state", headers={"Authorization": auth_header}
    )
    with urllib.request.urlopen(req, timeout=5) as resp:
        data = json.load(resp)
    return (data.get("menu_state") or {}).get("items") or []


def post_sold_total(base_url: str, auth_header: str, menu: str, sold_total: int) -> None:
    body = json.dumps(
        {"menu": menu, "sold_total": sold_total, "source": "dummy_sales_feed"}
    ).encode("utf-8")
    req = urllib.request.Request(
        f"{base_url}/api/sold_total",
        data=body,
        method="POST",
        headers={"Authorization": auth_header, "Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=5):
        pass


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://127.0.0.1:8080")
    parser.add_argument("--username", default="ADMIN_ACCOUNT")
    parser.add_argument(
        "--password", default=os.environ.get("NOKOSAHEN_WEB_UI_AUTH_PASSWORD", ""),
        help="残数入力画面のパスワード(既定: 環境変数NOKOSAHEN_WEB_UI_AUTH_PASSWORD)",
    )
    parser.add_argument("--interval", type=float, default=4.0, help="送信間隔(秒)")
    parser.add_argument("--min-step", type=int, default=1)
    parser.add_argument("--max-step", type=int, default=5)
    args = parser.parse_args()

    token = base64.b64encode(f"{args.username}:{args.password}".encode()).decode()
    auth_header = f"Basic {token}"

    items = get_items(args.base_url, auth_header)
    if not items:
        print("メニューが登録されていません。先に残数入力画面で初期食数を送信してください。")
        return

    initial = {item["name"]: max(1, int(item["initial"])) for item in items}
    sold = {name: 0 for name in initial}
    print(f"ダミーデータ配信を開始します(Ctrl+Cで終了): {list(initial.keys())}")

    try:
        while True:
            for name in list(initial.keys()):
                step = random.randint(args.min_step, args.max_step)
                sold[name] += step
                # 完売まで達したら0に戻し、残数の変化を継続して見せる
                if sold[name] >= initial[name]:
                    sold[name] = 0
                try:
                    post_sold_total(args.base_url, auth_header, name, sold[name])
                    print(f"送信: {name} sold_total={sold[name]}")
                except (urllib.error.URLError, urllib.error.HTTPError) as exc:
                    print(f"送信失敗 {name}: {exc}")
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("停止しました")


if __name__ == "__main__":
    main()
