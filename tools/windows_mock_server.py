from __future__ import annotations

import json
import random
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse


HTML = """<!doctype html>
<html lang="ja">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>のこさへん Windows mock</title>
  <style>
    body { margin: 0; font-family: system-ui, sans-serif; background: #f4f6f5; color: #17211d; }
    header { padding: 14px 18px; background: #124739; color: white; display: flex; justify-content: space-between; gap: 10px; }
    h1 { margin: 0; font-size: 22px; }
    main { max-width: 1080px; margin: 0 auto; padding: 14px; display: grid; grid-template-columns: 360px 1fr; gap: 14px; }
    section { background: white; border: 1px solid #dce4df; border-radius: 8px; padding: 14px; }
    h2 { margin: 0 0 12px; font-size: 17px; }
    label { display: block; margin: 0 0 5px; color: #53635c; font-size: 13px; }
    input { width: 100%; box-sizing: border-box; border: 1px solid #c7d2cc; border-radius: 6px; padding: 10px; font-size: 16px; }
    button { border: 0; border-radius: 6px; min-height: 40px; padding: 0 14px; background: #176b56; color: white; font-weight: 700; }
    button.warn { background: #9a3f1d; }
    .row { display: grid; grid-template-columns: 1.3fr .8fr; gap: 8px; margin-bottom: 10px; }
    .actions, .buttons { display: flex; flex-wrap: wrap; gap: 8px; }
    .metrics { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; margin-bottom: 12px; }
    .metric { background: #eef3f0; border-radius: 8px; padding: 12px; }
    .metric b { display: block; font-size: 24px; }
    .pill { display: inline-flex; align-items: center; min-height: 28px; padding: 0 10px; border-radius: 999px; background: #e3eee9; color: #124739; font-weight: 700; }
    table { width: 100%; border-collapse: collapse; }
    th, td { border-bottom: 1px solid #e4ebe7; padding: 10px 8px; text-align: left; }
    th { color: #53635c; font-size: 13px; }
    @media (max-width: 760px) { main { grid-template-columns: 1fr; padding: 10px; } .metrics { grid-template-columns: 1fr; } header { flex-direction: column; } }
  </style>
</head>
<body>
  <header><h1>のこさへん Mock</h1><span id="badge" class="pill">standby</span></header>
  <main>
    <section>
      <h2>初期食数</h2>
      <div id="inputs"></div>
      <div class="actions">
        <button onclick="submitInitial()">送信</button>
        <button onclick="addRow('', 0)">追加</button>
        <button class="warn" onclick="toggleAuto()">自動カウント切替</button>
      </div>
    </section>
    <section>
      <h2>状態</h2>
      <div class="metrics">
        <div class="metric"><span>初期</span><b id="initial">0</b></div>
        <div class="metric"><span>残数</span><b id="remaining">0</b></div>
        <div class="metric"><span>販売</span><b id="sold">0</b></div>
      </div>
      <table>
        <thead><tr><th>メニュー</th><th>初期</th><th>残数</th><th>販売</th><th></th></tr></thead>
        <tbody id="rows"></tbody>
      </table>
      <h2 style="margin-top:18px;">カウント</h2>
      <div id="buttons" class="buttons"></div>
      <h2 style="margin-top:18px;">挙動</h2>
      <p id="behavior">waiting</p>
      <p id="announce"></p>
    </section>
  </main>
  <script>
    const inputs = document.getElementById("inputs");
    function addRow(name, count) {
      const div = document.createElement("div");
      div.className = "row";
      div.innerHTML = `<div><label>名前</label><input class="name" value="${name}"></div><div><label>食数</label><input class="count" type="number" value="${count}"></div>`;
      inputs.appendChild(div);
    }
    ["カレー", "唐揚げ定食", "うどん"].forEach(name => addRow(name, 30));
    async function post(path, payload) {
      await fetch(path, {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(payload)});
    }
    async function submitInitial() {
      const items = [...document.querySelectorAll(".row")].map(row => ({
        name: row.querySelector(".name").value.trim(),
        initial: Number(row.querySelector(".count").value || 0)
      })).filter(item => item.name);
      await post("/api/menu/init", {items});
    }
    async function count(menu) { await post("/api/count", {menu, delta: 1}); }
    async function toggleAuto() { await post("/api/auto/toggle", {}); }
    function render(state) {
      const badge = `${state.phase}${state.promote_item ? " / " + state.promote_item : ""}`;
      document.getElementById("badge").textContent = badge;
      document.getElementById("initial").textContent = state.totals.initial;
      document.getElementById("remaining").textContent = state.totals.remaining;
      document.getElementById("sold").textContent = state.totals.sold;
      document.getElementById("rows").innerHTML = state.items.map(item => `<tr><td>${item.name}</td><td>${item.initial}</td><td>${item.remaining}</td><td>${item.sold}</td><td><button onclick='count(${JSON.stringify(item.name)})'>+1</button></td></tr>`).join("");
      document.getElementById("buttons").innerHTML = state.items.map(item => `<button onclick='count(${JSON.stringify(item.name)})'>${item.name}</button>`).join("");
      document.getElementById("behavior").textContent = `${state.behavior.mode} / crowd: ${state.behavior.crowd_level} / auto: ${state.auto_count}`;
      document.getElementById("announce").textContent = state.announce_text;
    }
    async function poll() {
      const res = await fetch("/api/state");
      render(await res.json());
    }
    setInterval(poll, 1000);
    submitInitial().then(poll);
  </script>
</body>
</html>
"""


class State:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.items = []
        self.started_at = time.monotonic()
        self.auto_count = True
        self.crowd_level = "calm"
        self.rng = random.Random(90)

    def init_items(self, items):
        with self.lock:
            self.items = [
                {
                    "name": str(item.get("name", "")).strip(),
                    "initial": max(0, int(item.get("initial", 0))),
                    "remaining": max(0, int(item.get("initial", 0))),
                    "sold": 0,
                }
                for item in items
                if str(item.get("name", "")).strip()
            ]
            self.started_at = time.monotonic()

    def count(self, menu: str, delta: int = 1):
        with self.lock:
            for item in self.items:
                if item["name"] == menu:
                    used = min(max(0, delta), item["remaining"])
                    item["remaining"] -= used
                    item["sold"] += used
                    return

    def snapshot(self):
        with self.lock:
            items = [dict(item) for item in self.items]
            elapsed = time.monotonic() - self.started_at
            totals = {
                "initial": sum(item["initial"] for item in items),
                "remaining": sum(item["remaining"] for item in items),
                "sold": sum(item["sold"] for item in items),
            }
            promote_item = None
            if elapsed >= 20.0 and len(items) >= 2:
                max_remaining = max(item["remaining"] for item in items)
                min_remaining = min(item["remaining"] for item in items)
                if max_remaining - min_remaining >= 3:
                    promote_item = max(items, key=lambda item: item["remaining"])["name"]
            phase = "induction" if promote_item else "initial"
            self.crowd_level = "crowded" if int(time.monotonic() / 20) % 2 == 0 else "calm"
            if self.crowd_level == "crowded":
                mode = "STOP_ANNOUNCE"
            else:
                mode = "APPROACH_PERSON"
            announce = (
                f"本日は{promote_item}にまだ余裕があります。できたてのうちにいかがですか。"
                if promote_item
                else "本日のメニューをご確認ください。お好きなものをどうぞ。"
            )
            return {
                "phase": phase,
                "promote_item": promote_item,
                "items": items,
                "totals": totals,
                "behavior": {"mode": mode, "crowd_level": self.crowd_level},
                "announce_text": announce,
                "auto_count": self.auto_count,
            }

    def auto_tick(self):
        while True:
            time.sleep(4)
            with self.lock:
                if not self.auto_count or not self.items:
                    continue
                candidates = [item for item in self.items if item["remaining"] > 0]
                if not candidates:
                    continue
                weighted = candidates + [candidates[0]]
                item = self.rng.choice(weighted)
                item["remaining"] -= 1
                item["sold"] += 1


state = State()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def send_bytes(self, status, body, content_type):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, status, payload):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_bytes(status, body, "application/json; charset=utf-8")

    def read_json(self):
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        return json.loads(self.rfile.read(length).decode("utf-8"))

    def do_GET(self):
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            self.send_bytes(HTTPStatus.OK, HTML.encode("utf-8"), "text/html; charset=utf-8")
        elif path == "/api/state":
            self.send_json(HTTPStatus.OK, state.snapshot())
        else:
            self.send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

    def do_POST(self):
        path = urlparse(self.path).path
        data = self.read_json()
        if path == "/api/menu/init":
            state.init_items(data.get("items", []))
            self.send_json(HTTPStatus.OK, {"ok": True})
        elif path == "/api/count":
            state.count(str(data.get("menu", "")), int(data.get("delta", 1)))
            self.send_json(HTTPStatus.OK, {"ok": True})
        elif path == "/api/auto/toggle":
            with state.lock:
                state.auto_count = not state.auto_count
                enabled = state.auto_count
            self.send_json(HTTPStatus.OK, {"ok": True, "auto_count": enabled})
        else:
            self.send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})


def main():
    threading.Thread(target=state.auto_tick, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", 8080), Handler)
    print("Windows mock UI: http://localhost:8080")
    print("LAN/iPad: http://<this PC IP>:8080")
    server.serve_forever()


if __name__ == "__main__":
    main()

