"""メニュー事前登録用の専用サイト(管理者向け)。

日々の食数入力画面(`web_ui_node.py`、iPad等でその場で入力)とは別に、
2週間分程度のメニュー予定をあらかじめまとめて登録しておくためのページ。
ここで登録した内容は`web_ui_node.py`が起動日の日付で参照し、その日の
メニュー名と目安の食数を自動で入力欄に反映する(実際の食数はその日の
状況に応じて調整して送信する運用は変わらない)。

データはSQLite(`menu_schedule_store.py`)に保存し、`web_ui_node.py`と
同じDBファイルを共有する。
"""
from __future__ import annotations

import datetime
import os
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict
from urllib.parse import parse_qs, urlparse

import rclpy
from rclpy.node import Node

from .secrets_loader import resolve_password
from .http_auth import check_basic_auth, send_auth_required
from .json_tools import dumps, loads
from .menu_schedule_store import MenuScheduleStore

HTML = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>のこさへん - メニュー事前登録</title>
<style>
  :root { color-scheme: light; font-family: system-ui, -apple-system, "Segoe UI", sans-serif; background:#f4f6f5; color:#16211d; }
  body { margin:0; }
  header { padding:14px 18px; background:#124739; color:#fff; }
  header h1 { margin:0; font-size:20px; }
  main { max-width:900px; margin:0 auto; padding:14px; display:flex; flex-direction:column; gap:12px; }
  .day-card { background:#fff; border:1px solid #dce4df; border-radius:8px; padding:12px 14px; }
  .day-card.today { border-color:#176b56; box-shadow:0 0 0 2px rgba(23,107,86,0.15); }
  .day-title { font-weight:700; margin-bottom:8px; display:flex; justify-content:space-between; align-items:center; }
  .menu-row { display:grid; grid-template-columns:1fr auto; gap:8px; margin-bottom:6px; align-items:end; }
  label { display:block; font-size:12px; color:#53635c; margin-bottom:3px; }
  input { width:100%; box-sizing:border-box; padding:8px; border:1px solid #c7d2cc; border-radius:6px; font-size:15px; }
  button { min-height:36px; border:0; border-radius:6px; padding:0 12px; background:#176b56; color:#fff; font-weight:700; font-size:14px; }
  button.secondary { background:#4e5e57; }
  button.warning { background:#9a3f1d; }
  .actions { display:flex; flex-wrap:wrap; gap:8px; margin-top:8px; }
  .hint { font-size:12px; color:#5b6b63; }
  .saved-hint { font-size:12px; color:#176b56; margin-left:8px; }
</style>
</head>
<body>
<header><h1>メニュー事前登録(2週間分の目安)</h1></header>
<p class="hint" style="max-width:900px;margin:0 auto;padding:0 14px;">
  各日の上から1番目=A、2番目=B、3番目=C として、明電舎タブレットの
  「Aランチ」「Bランチ」「Cランチ」に対応させます(初期食数入力画面と同じ並び順)。
</p>
<main id="days"></main>
<script>
  // 【2026-08-07追加】初期食数入力画面(web_ui_node.py)と同じ考え方で、
  // 上から1〜3番目の行をA/B/Cとして視覚的に対応させる(位置だけの目印で、
  // サーバー側のデータ構造は変えていない)。
  const SLOT_LABELS = ["A", "B", "C"];
  function slotLabelFor(index) {
    return SLOT_LABELS[index] || "";
  }
  function refreshSlotLabels(iso) {
    const rows = document.querySelectorAll(`.day-card[data-date="${iso}"] .menu-row`);
    rows.forEach((div, index) => {
      const badge = div.querySelector(".slot-badge");
      if (badge) badge.textContent = slotLabelFor(index);
    });
  }

  async function postJson(path, payload) {
    const res = await fetch(path, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify(payload),
    });
    if (!res.ok) throw new Error(await res.text());
  }

  function escapeHtml(value) {
    return String(value).replace(/[&<>"']/g, (ch) => ({
      "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"
    }[ch]));
  }

  function todayIso() {
    const d = new Date();
    return d.getFullYear() + "-" + String(d.getMonth() + 1).padStart(2, "0") + "-" + String(d.getDate()).padStart(2, "0");
  }

  function dateLabel(iso) {
    const d = new Date(iso + "T00:00:00");
    const days = ["日", "月", "火", "水", "木", "金", "土"];
    return `${iso}(${days[d.getDay()]})`;
  }

  function addDays(iso, n) {
    const d = new Date(iso + "T00:00:00");
    d.setDate(d.getDate() + n);
    return d.getFullYear() + "-" + String(d.getMonth() + 1).padStart(2, "0") + "-" + String(d.getDate()).padStart(2, "0");
  }

  // 【2026-10-02追加】読み込んだ予定側の初期数(initial)を行ごとに保持し、保存時に
  // そのまま送り返す(以前はnameだけ送っていたため、保存する度に予定側の初期数が0に
  // 戻っていた)。メニュー名を書き換えた行は別メニュー扱いなので0にする。
  function menuRowHtml(name, iso, initial) {
    const safeInitial = Number.isFinite(Number(initial)) ? Number(initial) : 0;
    return `
      <div class="menu-row" data-orig-name="${escapeHtml(name)}" data-initial="${safeInitial}">
        <div><label><span class="slot-badge" style="display:inline-block;min-width:20px;font-weight:800;color:#176b56;"></span>メニュー名</label><input class="name" value="${escapeHtml(name)}"></div>
        <div><button type="button" class="warning" onclick="this.closest('.menu-row').remove(); refreshSlotLabels('${iso}');">削除</button></div>
      </div>
    `;
  }

  // 何も登録が無い日は0行(手動で「行を追加」を3回押す)ではなく、
  // 最初からA/B/C分の空欄を3行出しておく(タブレット読み取り対象の3枠を
  // 埋めるだけで済むようにするため)。
  const DEFAULT_ROW_COUNT = 3;

  function dayCardHtml(iso, items) {
    const isToday = iso === todayIso();
    const displayItems = items.length
      ? items
      : Array.from({ length: DEFAULT_ROW_COUNT }, () => ({ name: "" }));
    const rows = displayItems.map(it => menuRowHtml(it.name, iso, it.initial)).join("");
    return `
      <div class="day-card${isToday ? " today" : ""}" data-date="${iso}">
        <div class="day-title">
          <span>${dateLabel(iso)}${isToday ? "(本日)" : ""}</span>
          <span class="saved-hint" data-role="saved-hint"></span>
        </div>
        <div class="rows">${rows}</div>
        <div class="actions">
          <button onclick="saveDay('${iso}')">保存</button>
          <button class="secondary" onclick="addRow('${iso}')">行を追加</button>
        </div>
      </div>
    `;
  }

  function addRow(iso) {
    const card = document.querySelector(`.day-card[data-date="${iso}"] .rows`);
    card.insertAdjacentHTML("beforeend", menuRowHtml("", iso, 0));
    refreshSlotLabels(iso);
  }

  async function saveDay(iso) {
    const card = document.querySelector(`.day-card[data-date="${iso}"]`);
    const allRows = [...card.querySelectorAll(".menu-row")].map(div => {
      const name = div.querySelector(".name").value.trim();
      const sameName = name === (div.dataset.origName || "").trim();
      return { name, initial: sameName ? Number(div.dataset.initial || 0) : 0 };
    });
    const items = allRows.filter(item => item.name);
    // 【2026-10-02追加】上から1〜3番目がA/B/C(明電舎タブレットの固定位置)に対応する。
    // 途中に空欄の行があると、保存時に空欄が詰められてCがBにずれ、タブレットの行と
    // 対応しなくなるため、確認を取ってから保存する(登録データは空欄を保持できない)。
    const firstBlank = allRows.findIndex(item => !item.name);
    if (firstBlank !== -1 && allRows.slice(firstBlank).some(item => item.name)) {
      const ok = confirm(
        "途中に空欄の行があります。保存すると空欄が詰められ、下の行が繰り上がって" +
        "A/B/Cの対応がずれます。このまま保存しますか？(キャンセルで保存を中止します)"
      );
      if (!ok) return;
    }
    try {
      await postJson("/api/schedule", {date: iso, items});
      const hint = card.querySelector('[data-role="saved-hint"]');
      hint.textContent = "保存しました";
      setTimeout(() => { hint.textContent = ""; }, 2000);
    } catch (e) {
      alert("保存に失敗しました: " + e);
    }
  }

  async function loadDays() {
    const start = todayIso();
    const res = await fetch("/api/schedule?start=" + start + "&days=14");
    const data = await res.json();
    const schedule = data.schedule || {};
    const daysEl = document.getElementById("days");
    daysEl.innerHTML = "";
    for (let i = 0; i < 14; i++) {
      const iso = addDays(start, i);
      daysEl.insertAdjacentHTML("beforeend", dayCardHtml(iso, schedule[iso] || []));
      refreshSlotLabels(iso);
    }
  }
  loadDays();
</script>
</body>
</html>
"""


class MenuScheduleNode(Node):
    def __init__(self) -> None:
        super().__init__("menu_schedule")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8085)
        self.declare_parameter("auth_username", "ADMIN_ACCOUNT")
        self.declare_parameter("auth_password", "")
        self.declare_parameter("db_path", "~/.nokosahen_menu_schedule.db")

        db_path = os.path.expanduser(str(self.get_parameter("db_path").value))
        self.store = MenuScheduleStore(db_path)
        self.get_logger().info(f"menu_schedule: DB={db_path}")

        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        auth_username = str(self.get_parameter("auth_username").value)
        auth_password = resolve_password(
            self.get_logger(), "menu_schedule", "auth_password",
            str(self.get_parameter("auth_password").value),
            str(self.get_parameter("auth_username").value),
        )
        handler = self._make_handler(auth_username, auth_password)
        self._server = ThreadingHTTPServer((host, port), handler)
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()
        self.get_logger().info(f"menu_schedule listening on http://{host}:{port}")

    def destroy_node(self) -> bool:
        self._server.shutdown()
        self._server.server_close()
        return super().destroy_node()

    def _make_handler(self, auth_username: str, auth_password: str):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt: str, *args: Any) -> None:
                node.get_logger().debug(fmt % args)

            def _check_auth(self) -> bool:
                if check_basic_auth(self, auth_username, auth_password):
                    return True
                send_auth_required(self, "Nokosahen Menu Schedule")
                return False

            def _send(self, status: int, body: bytes, content_type: str) -> None:
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def _send_json(self, status: int, payload: dict) -> None:
                self._send(status, dumps(payload).encode("utf-8"), "application/json; charset=utf-8")

            def _read_json(self) -> dict:
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                except ValueError:
                    return {}
                if length <= 0 or length > 1024 * 64:
                    return {}
                try:
                    raw = self.rfile.read(length).decode("utf-8")
                except UnicodeDecodeError:
                    return {}
                data = loads(raw, {})
                return data if isinstance(data, dict) else {}

            def do_GET(self) -> None:  # noqa: N802
                if not self._check_auth():
                    return
                parsed = urlparse(self.path)
                path = parsed.path
                if path in ("/", "/index.html"):
                    self._send(HTTPStatus.OK, HTML.encode("utf-8"), "text/html; charset=utf-8")
                    return
                if path == "/api/schedule":
                    qs = parse_qs(parsed.query)
                    start = (qs.get("start") or [""])[0].strip()
                    if not start:
                        start = datetime.date.today().isoformat()
                    try:
                        days = max(1, min(60, int((qs.get("days") or ["14"])[0])))
                    except ValueError:
                        days = 14
                    try:
                        start_date = datetime.date.fromisoformat(start)
                    except ValueError:
                        self._send_json(HTTPStatus.BAD_REQUEST, {"error": "invalid start date"})
                        return
                    end_date = start_date + datetime.timedelta(days=days - 1)
                    schedule = node.store.list_range(start_date.isoformat(), end_date.isoformat())
                    self._send_json(HTTPStatus.OK, {"schedule": schedule})
                    return
                self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

            def do_POST(self) -> None:  # noqa: N802
                if not self._check_auth():
                    return
                path = urlparse(self.path).path
                data = self._read_json()
                if path == "/api/schedule":
                    date_str = str(data.get("date", "")).strip()
                    try:
                        datetime.date.fromisoformat(date_str)
                    except ValueError:
                        self._send_json(HTTPStatus.BAD_REQUEST, {"error": "date must be YYYY-MM-DD"})
                        return
                    items = data.get("items", [])
                    if not isinstance(items, list):
                        self._send_json(HTTPStatus.BAD_REQUEST, {"error": "items must be a list"})
                        return
                    node.store.set_menu_for_date(date_str, items)
                    self._send_json(HTTPStatus.OK, {"ok": True})
                    return
                self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

        return Handler


def main() -> None:
    rclpy.init()
    node = MenuScheduleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
