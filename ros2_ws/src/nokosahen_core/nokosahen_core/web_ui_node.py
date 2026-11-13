from __future__ import annotations

import datetime
import json
import os
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict
from urllib.parse import parse_qs, urlparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .assets import LOGO_DATA_URI
from .secrets_loader import resolve_password
from .http_auth import check_basic_auth, send_auth_required
from .json_tools import dumps, loads
from .menu_schedule_store import MenuScheduleStore


HTML = """<!doctype html>
<html lang="ja">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>のこさへん</title>
  <style>
    :root {
      color-scheme: light;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #f4f6f5;
      color: #16211d;
    }
    body { margin: 0; }
    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding: 14px 18px;
      background: #124739;
      color: white;
    }
    .logo-title { display: flex; align-items: center; gap: 10px; }
    .logo-title img { height: 40px; width: 40px; border-radius: 8px; }
    h1 { margin: 0; font-size: 22px; letter-spacing: 0; }
    main {
      display: grid;
      grid-template-columns: minmax(280px, 380px) 1fr;
      gap: 14px;
      padding: 14px;
      max-width: 1120px;
      margin: 0 auto;
    }
    section {
      background: white;
      border: 1px solid #dce4df;
      border-radius: 8px;
      padding: 14px;
    }
    h2 { margin: 0 0 12px; font-size: 17px; }
    label { display: block; font-size: 13px; color: #53635c; margin-bottom: 5px; }
    input {
      width: 100%;
      box-sizing: border-box;
      padding: 10px;
      border: 1px solid #c7d2cc;
      border-radius: 6px;
      font-size: 16px;
    }
    input[readonly] {
      background: #eef1ef;
      color: #53635c;
    }
    button {
      min-height: 40px;
      border: 0;
      border-radius: 6px;
      padding: 0 14px;
      background: #176b56;
      color: white;
      font-weight: 700;
      font-size: 15px;
    }
    button.secondary { background: #4e5e57; }
    button.warning { background: #9a3f1d; }
    .menu-row {
      display: grid;
      grid-template-columns: 1.4fr .8fr auto;
      gap: 8px;
      margin-bottom: 10px;
      align-items: end;
    }
    .actions { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 12px; }
    .status {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
      margin-bottom: 12px;
    }
    .metric {
      background: #eef3f0;
      border-radius: 8px;
      padding: 12px;
    }
    .metric b { display: block; font-size: 24px; }
    table { width: 100%; border-collapse: collapse; }
    th, td { border-bottom: 1px solid #e4ebe7; padding: 10px 8px; text-align: left; }
    th { color: #53635c; font-size: 13px; }
    .count-grid { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 8px; }
    .pill {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      min-height: 28px;
      padding: 4px 10px;
      border-radius: 999px;
      background: #e3eee9;
      color: #124739;
      font-weight: 700;
      font-size: 13px;
    }
    .pill button { min-height: 24px; padding: 0 8px; font-size: 12px; }
    /* 【2026-10-02追加】送信結果のトーストと、状態取得エラーの帯 */
    #toast {
      position: fixed; left: 50%; bottom: 24px; transform: translateX(-50%);
      max-width: 90vw; padding: 10px 18px; border-radius: 8px; font-size: 14px; font-weight: 700;
      color: #fff; background: #176b56; box-shadow: 0 2px 10px rgba(0,0,0,0.25);
      display: none; z-index: 1000;
    }
    #toast.error { background: #a32d2d; }
    #connBanner {
      display: none; background: #faece7; color: #993c1d; padding: 8px 14px;
      font-size: 13px; font-weight: 700; text-align: center;
    }
    @media (max-width: 760px) {
      main { grid-template-columns: 1fr; padding: 10px; }
      .status { grid-template-columns: 1fr; }
      header { align-items: flex-start; gap: 8px; flex-direction: column; }
    }
  </style>
</head>
<body>
  <header>
    <div class="logo-title">
      <img src="__NOKOSAHEN_LOGO__" alt="のこさへん" />
      <h1>のこさへん</h1>
    </div>
    <span id="phase" class="pill">standby</span>
  </header>
  <div id="connBanner"></div>
  <div id="toast" role="status"></div>
  <main>
    <section>
      <h2>初期食数</h2>
      <p style="font-size:12px;color:#53635c;margin-top:-6px;">
        上から1番目=A、2番目=B、3番目=C として、明電舎タブレットの
        「Aランチ」「Bランチ」「Cランチ」に対応させます。並び順を
        タブレットと揃えて、本日の実際のメニュー名と作った数を入力してください。
      </p>
      <div id="menuInputs"></div>
      <div class="actions">
        <button onclick="submitInitial()">送信</button>
      </div>
    </section>

    <section>
      <h2>残数</h2>
      <div class="status">
        <div class="metric"><span>初期</span><b id="totalInitial">0</b></div>
        <div class="metric"><span>残数</span><b id="totalRemaining">0</b></div>
        <div class="metric"><span>販売</span><b id="totalSold">0</b></div>
      </div>
      <table>
        <thead><tr><th>メニュー</th><th>初期</th><th>残数</th><th>販売</th><th>タブレット累計販売数を入力</th><th>手動訂正</th></tr></thead>
        <tbody id="stateRows"></tbody>
      </table>
      <h2 style="margin-top:18px;">カウント</h2>
      <div id="countButtons" class="count-grid"></div>
      <h2 style="margin-top:18px;">挙動</h2>
      <p id="behavior">waiting</p>
      <p id="announce"></p>
      <p style="margin-top:18px;font-size:13px;color:#53635c;">
        Raspberry Pi等で利用者向けに表示する画面:
        <a href="/customer_display" target="_blank">/customer_display</a>
        (ログインは共通、閲覧専用)
      </p>
    </section>
  </main>
  <script>
    const defaults = ["カレー", "唐揚げ定食", "うどん"];
    const menuInputs = document.getElementById("menuInputs");

    function escapeHtml(value) {
      return String(value).replace(/[&<>"']/g, (ch) => ({
        "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"
      }[ch]));
    }

    // 【2026-08-07追加】明電舎タブレット読み取り(Raspberry Pi + Coral構成)は
    // タブレット上から1番目/2番目/3番目の行(Aランチ/Bランチ/Cランチ)の数字
    // だけを固定位置で読み取る設計のため、この画面の並び順(上から1,2,3番目)を
    // そのままA/B/Cに対応させる。4番目以降はタブレット読み取りの対象外。
    const SLOT_LABELS = ["A", "B", "C"];

    function slotLabelFor(index) {
      return SLOT_LABELS[index] || "";
    }

    function refreshSlotLabels() {
      [...menuInputs.querySelectorAll(".menu-row")].forEach((div, index) => {
        const badge = div.querySelector(".slot-badge");
        if (badge) badge.textContent = slotLabelFor(index);
      });
    }

    function row(name = "", amount = 0) {
      // item.nameはサーバー側(他の人が/api/menu/initで入力したメニュー名)
      // 由来の場合があるため、value属性への埋め込み前に必ずエスケープする
      // (未エスケープだと"を含む名前で属性が壊れ、HTML/スクリプト注入になる)。
      const safeAmount = Number.isFinite(Number(amount)) ? Number(amount) : 0;
      const div = document.createElement("div");
      div.className = "menu-row";
      // メニュー名は専用サイト(メニュー事前登録)で決めるものなので、ここでは
      // 編集できないようにする(readonly、送信時の.value読み取りは従来通り
      // 効く)。ここで入力するのは残数の数値だけ。
      div.innerHTML = `
        <div><label><span class="slot-badge" style="display:inline-block;min-width:20px;font-weight:800;color:#176b56;"></span> 名前</label><input class="name" value="${escapeHtml(name)}" readonly></div>
        <div><label>残数</label><input class="initial" type="number" min="0" value="${safeAmount}"></div>
        <div><button type="button" class="warning" onclick="this.closest('.menu-row').remove(); refreshSlotLabels();">削除</button></div>
      `;
      menuInputs.appendChild(div);
      refreshSlotLabels();
    }

    // メニュー事前登録(専用サイトであらかじめ入力した2週間分の予定)から
    // 本日の分を自動で読み込む。名前・並び順は常にこちらを正として使う
    // (事前登録側を後から編集した場合、リロードで確実に反映されるように
    // するため)。同じ並び順で本日分を送信済みなら、その残数入力欄には
    // 予定側の値ではなく実際に送信済みの初期数を復元する(入力し直す
    // 手間を省く)。事前登録が無ければ送信済み状態、それも無ければ
    // 固定デフォルトの順にフォールバックする。
    async function fetchScheduledItems() {
      try {
        const res = await fetch("/api/menu/scheduled");
        if (!res.ok) return [];
        const data = await res.json();
        return data.items || [];
      } catch (e) {
        return [];
      }
    }

    async function fetchStateItems() {
      try {
        const res = await fetch("/api/state");
        if (!res.ok) return [];
        const data = await res.json();
        return (data.menu_state && data.menu_state.items) || [];
      } catch (e) {
        return [];
      }
    }

    async function initializeRows() {
      const [scheduled, stateItems] = await Promise.all([fetchScheduledItems(), fetchStateItems()]);
      menuInputs.innerHTML = "";
      if (scheduled.length) {
        scheduled.forEach((item, index) => {
          const existing = stateItems[index];
          row(item.name, existing ? existing.initial : (item.initial || 0));
        });
      } else if (stateItems.length) {
        stateItems.forEach(item => row(item.name, item.initial));
      } else {
        defaults.forEach(name => row(name, 30));
      }
    }
    initializeRows();

    // 【2026-10-02追加】送信結果(成功/失敗)を画面下部に表示する。
    let toastTimer = null;
    function showToast(message, isError) {
      const el = document.getElementById("toast");
      el.textContent = message;
      el.className = isError ? "error" : "";
      el.style.display = "block";
      clearTimeout(toastTimer);
      toastTimer = setTimeout(() => { el.style.display = "none"; }, isError ? 6000 : 2500);
    }

    async function postJson(path, payload) {
      const res = await fetch(path, {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(payload)
      });
      if (!res.ok) {
        let detail = "";
        try {
          const text = await res.text();
          try {
            const d = JSON.parse(text);
            detail = (d && (d.message || d.error)) || text;
          } catch (e2) { detail = text; }
        } catch (e3) { /* 本文なし */ }
        throw new Error(`HTTP ${res.status}${detail ? ": " + detail : ""}`);
      }
    }

    // 成功すればtrue、失敗(通信エラー・HTTPエラー)ならエラートーストを出してfalseを返す。
    async function postWithToast(path, payload, okMessage, failLabel) {
      try {
        await postJson(path, payload);
        showToast(okMessage, false);
        return true;
      } catch (err) {
        showToast(`${failLabel}に失敗しました(${err && err.message ? err.message : err})`, true);
        return false;
      }
    }

    async function submitInitial() {
      const items = [...document.querySelectorAll(".menu-row")].map(div => ({
        name: div.querySelector(".name").value.trim(),
        initial: Number(div.querySelector(".initial").value || 0)
      })).filter(item => item.name);
      return postWithToast("/api/menu/init", {items}, "初期食数を送信しました", "初期食数の送信");
    }

    async function count(menu, delta = 1) {
      return postWithToast(
        "/api/count", {menu, delta, source: "web_ui"},
        `${menu} を ${delta > 0 ? "+" : ""}${delta} しました`, "カウント");
    }

    // 明電舎タブレット(POSレジ集計)に表示されている「累計販売数」を
    // そのまま入力してもらう方式。残数 = 初期数 - 累計販売数 で自動計算される
    // (通過人数カメラ方式より正確なため、こちらを基本の運用にする)。
    async function submitSoldTotal(menu, soldTotal) {
      return postWithToast(
        "/api/sold_total", {menu, sold_total: soldTotal, source: "web_ui_tablet"},
        `${menu} の累計販売数 ${soldTotal} を反映しました`, "累計販売数の反映");
    }

    // 【2026-10-02修正】以前は毎秒(poll)stateRowsをinnerHTMLで丸ごと作り直しており、
    // 「累計販売数を入力」欄に打ち込んでいる最中の値・フォーカスが1秒ごとに消えていた
    // (ボタンも作り直しの瞬間のクリックが無効になる)。メニュー名の並びが変わった時だけ
    // 作り直し(入力途中の値とフォーカスは名前ごとに引き継ぐ)、通常は数値セルだけを更新する。
    let stateRowsKey = null;
    function renderStateRows(items) {
      const tbody = document.getElementById("stateRows");
      const pills = document.getElementById("countButtons");
      const key = JSON.stringify(items.map(item => item.name));
      if (key !== stateRowsKey) {
        const typed = {};
        let focusedMenu = null;
        let focusedPos = null;
        tbody.querySelectorAll("tr[data-menu]").forEach(tr => {
          const input = tr.querySelector(".sold-total-input");
          if (!input) return;
          if (input.value !== "") typed[tr.dataset.menu] = input.value;
          if (document.activeElement === input) {
            focusedMenu = tr.dataset.menu;
            focusedPos = input.selectionStart;
          }
        });
        // onclick属性にJSON.stringifyしたメニュー名を直接埋め込むと、名前に
        // シングルクォートが含まれた際にHTML属性が途中で終端し壊れる。
        // data-*属性 + イベント委譲(下のaddEventListener)に置き換えて回避する。
        tbody.innerHTML = items.map(item => `
          <tr data-menu="${escapeHtml(item.name)}">
            <td>${escapeHtml(item.name)}</td><td class="cell-initial"></td><td class="cell-remaining"></td><td class="cell-sold"></td>
            <td>
              <input type="number" min="0" class="sold-total-input" style="width:100px;display:inline-block;" placeholder="累計販売数">
              <button data-menu="${escapeHtml(item.name)}" data-action="soldTotal">反映</button>
            </td>
            <td>
              <button class="warning" data-menu="${escapeHtml(item.name)}" data-delta="1">+1</button>
              <button class="secondary" data-menu="${escapeHtml(item.name)}" data-delta="-1">-1(訂正)</button>
            </td>
          </tr>
        `).join("");
        pills.innerHTML = items.map(item => `
          <span class="pill">
            ${escapeHtml(item.name)}
            <button data-menu="${escapeHtml(item.name)}" data-delta="1">+1</button>
            <button class="secondary" data-menu="${escapeHtml(item.name)}" data-delta="-1">-1</button>
          </span>
        `).join("");
        tbody.querySelectorAll("tr[data-menu]").forEach(tr => {
          const input = tr.querySelector(".sold-total-input");
          if (typed[tr.dataset.menu] !== undefined) input.value = typed[tr.dataset.menu];
          if (tr.dataset.menu === focusedMenu) {
            input.focus();
            try { input.setSelectionRange(focusedPos, focusedPos); } catch (e) { /* type=numberは未対応 */ }
          }
        });
        stateRowsKey = key;
      }
      const trs = tbody.querySelectorAll("tr[data-menu]");
      items.forEach((item, index) => {
        const tr = trs[index];
        if (!tr) return;
        tr.querySelector(".cell-initial").textContent = item.initial;
        tr.querySelector(".cell-remaining").textContent = item.remaining;
        tr.querySelector(".cell-sold").textContent = item.sold;
      });
    }

    function render(state) {
      const menu = state.menu_state || {};
      const items = menu.items || [];
      document.getElementById("phase").textContent =
        `${menu.phase || "standby"}${menu.promote_item ? " / " + menu.promote_item : ""}`;
      const totals = menu.totals || {};
      document.getElementById("totalInitial").textContent = totals.initial || 0;
      document.getElementById("totalRemaining").textContent = totals.remaining || 0;
      document.getElementById("totalSold").textContent = totals.sold || 0;
      renderStateRows(items);
      const behavior = state.behavior_command || {};
      document.getElementById("behavior").textContent =
        `${behavior.mode || "waiting"} / crowd: ${behavior.crowd_level || "unknown"}`;
      document.getElementById("announce").textContent = state.announce_text || "";
    }

    function setConnBanner(message) {
      const el = document.getElementById("connBanner");
      el.textContent = message;
      el.style.display = message ? "block" : "none";
    }

    async function poll() {
      try {
        const res = await fetch("/api/state");
        if (!res.ok) {
          // 【2026-10-02追加】非OK応答(401/500等)を無視して古い表示のままにしない。
          setConnBanner(`状態を取得できません(HTTP ${res.status})。表示は最新ではありません。`);
          return;
        }
        render(await res.json());
        setConnBanner("");
      } catch (err) {
        setConnBanner("サーバーに接続できません。表示は最新ではありません。");
        document.getElementById("behavior").textContent = String(err);
      }
    }
    setInterval(poll, 1000);
    poll();

    // data-menu/data-delta を持つボタンのクリックをまとめて処理する
    // (innerHTMLで動的に差し替えても効くよう、documentへの委譲にしている)。
    document.addEventListener("click", (event) => {
      const btn = event.target.closest("button[data-menu]");
      if (!btn) return;
      if (btn.dataset.action === "soldTotal") {
        const input = btn.previousElementSibling;
        const soldTotal = Number(input && input.value);
        if (!Number.isFinite(soldTotal) || soldTotal < 0) {
          alert("累計販売数は0以上の数値で入力してください");
          return;
        }
        // 送信に成功した時だけ入力欄を空にする(失敗時は入力値を残して再送できるようにする)。
        submitSoldTotal(btn.dataset.menu, soldTotal).then((ok) => { if (ok && input) input.value = ""; });
        return;
      }
      count(btn.dataset.menu, Number(btn.dataset.delta));
    });
  </script>
</body>
</html>
""".replace("__NOKOSAHEN_LOGO__", LOGO_DATA_URI)


# 【2026-08-07追加】Raspberry Piの画面にChromiumで開いてもらう、利用者向けの
# 残数表示ページ。読み取り専用(操作ボタンは一切無い)。操作用の残数入力画面
# (上のHTML)と同じ認証(auth_username/auth_password、既定ADMIN_ACCOUNT/<パスワード(ADMIN_ACCOUNT)>)を
# そのまま使う。Raspberry Pi側は表示するだけなので性能要件はごく軽い。
CUSTOMER_DISPLAY_HTML = """<!doctype html>
<html lang="ja">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no" />
  <title>のこさへん - 本日の残数</title>
  <style>
    /* 【2026-08-21調整】Raspberry Pi公式7インチタッチディスプレイ(旧リビジョン、
       800x480横向き)専用のキオスク表示に最適化。画面が小さい分スクロールは
       一切発生させず、A/B/C最大3枚が確実に1画面に収まるようレイアウトを
       詰めている。タッチ操作は行わない前提(閲覧専用)なので、誤操作防止に
       文字選択・タップハイライト・ズームも無効化している。 */
    :root {
      color-scheme: light;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #0f2e24;
      color: #16211d;
    }
    html, body {
      margin: 0;
      height: 100%;
      overflow: hidden;
      -webkit-user-select: none;
      user-select: none;
      -webkit-touch-callout: none;
      -webkit-tap-highlight-color: transparent;
      cursor: none;
    }
    body { display: flex; flex-direction: column; }
    header {
      display: flex;
      align-items: center;
      gap: 10px;
      padding: 10px 16px;
      background: #124739;
      color: white;
      flex: 0 0 auto;
    }
    header img { height: 34px; width: 34px; border-radius: 8px; }
    header h1 { margin: 0; font-size: 20px; }
    #updatedAt { margin-left: auto; font-size: 12px; opacity: 0.8; }
    #promoteBanner {
      display: none;
      margin: 8px 16px 0;
      padding: 8px 14px;
      border-radius: 10px;
      background: #ffce54;
      color: #4a2e00;
      font-size: 15px;
      font-weight: 700;
      text-align: center;
      flex: 0 0 auto;
    }
    main {
      flex: 1 1 auto;
      min-height: 0;
      padding: 10px 14px 12px;
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      gap: 10px;
    }
    .card {
      background: white;
      border-radius: 12px;
      padding: 10px 14px;
      box-shadow: 0 2px 8px rgba(0,0,0,0.18);
      display: flex;
      flex-direction: column;
      justify-content: center;
      min-height: 0;
    }
    .card .name {
      font-size: 16px;
      font-weight: 700;
      margin-bottom: 4px;
      white-space: nowrap;
      overflow: hidden;
      text-overflow: ellipsis;
    }
    .card .remaining { font-size: 40px; font-weight: 800; line-height: 1; }
    .card .unit { font-size: 14px; font-weight: 500; margin-left: 4px; }
    .card .bar-track {
      margin-top: 6px;
      height: 8px;
      border-radius: 999px;
      background: #e4ebe7;
      overflow: hidden;
    }
    .card .bar-fill { height: 100%; border-radius: 999px; }
    .status-ok .remaining { color: #176b56; }
    .status-ok .bar-fill { background: #176b56; }
    .status-low .remaining { color: #c07a00; }
    .status-low .bar-fill { background: #e0a418; }
    .status-out .remaining { color: #9a3f1d; }
    .status-out .bar-fill { background: #9a3f1d; }
    .status-out .card { opacity: 0.85; }
    .status-label { margin-top: 4px; font-size: 11px; font-weight: 700; }
    .status-ok .status-label { color: #176b56; }
    .status-low .status-label { color: #c07a00; }
    .status-out .status-label { color: #9a3f1d; }
    #empty { padding: 24px; color: white; font-size: 16px; text-align: center; }
    /* 【2026-09-15追加】このページはraspiの7インチキオスク(800x480)専用では
       なく、操作ダッシュボードからtarget="_blank"でスタッフのスマホ・狭い
       ウィンドウからも確認用に開かれる。800x480最適化時に元あった
       狭幅フォールバックを誤って削除していたため復活させる(レビューで発見)。 */
    @media (max-width: 380px) {
      header h1 { font-size: 16px; }
      header img { height: 26px; width: 26px; }
      .card .remaining { font-size: 32px; }
      main { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <header>
    <img src="__NOKOSAHEN_LOGO__" alt="のこさへん" />
    <h1>本日の残数</h1>
    <span id="updatedAt"></span>
  </header>
  <div id="promoteBanner"></div>
  <main id="cards"><div id="empty">読み込み中...</div></main>
  <script>
    // 【重要】このページは401(未ログイン)を受けても自動でログイン画面へは
    // 飛ばさない(Raspberry Piでキオスク的に開きっぱなしにする用途のため、
    // 認証切れで勝手に別画面へ遷移すると気づきにくい)。代わりに画面上に
    // エラー表示を出すだけにする。
    function escapeHtml(value) {
      return String(value).replace(/[&<>"']/g, (ch) => ({
        "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"
      }[ch]));
    }

    function statusOf(item) {
      if (item.remaining <= 0) return { cls: "status-out", label: "本日完売" };
      const ratio = item.initial > 0 ? item.remaining / item.initial : 1;
      if (ratio <= 0.2) return { cls: "status-low", label: "残りわずかです" };
      return { cls: "status-ok", label: "" };
    }

    function render(state) {
      const menu = state.menu_state || {};
      const items = menu.items || [];
      const cards = document.getElementById("cards");
      if (!items.length) {
        cards.innerHTML = '<div id="empty">本日のメニューは準備中です</div>';
      } else {
        cards.innerHTML = items.map(item => {
          const st = statusOf(item);
          const ratio = item.initial > 0 ? Math.max(0, Math.min(1, item.remaining / item.initial)) : 0;
          return `
            <div class="${st.cls}">
              <div class="card">
                <div class="name">${escapeHtml(item.name)}</div>
                <div><span class="remaining">${item.remaining}</span><span class="unit">食</span></div>
                <div class="bar-track"><div class="bar-fill" style="width:${Math.round(ratio * 100)}%;"></div></div>
                ${st.label ? `<div class="status-label">${st.label}</div>` : ""}
              </div>
            </div>
          `;
        }).join("");
      }

      // 推奨中のメニューがあれば、ロボットの呼びかけと同じ内容を画面上でも
      // 視覚的に後押しする(2026-07-24現地検証で「音声のみでは効果が薄い、
      // 視覚的アプローチを強化する」との方針が出たことを踏まえた追加)。
      const banner = document.getElementById("promoteBanner");
      if (menu.phase === "induction" && menu.promote_item) {
        banner.textContent = `本日は「${menu.promote_item}」にまだ余裕があります。できたてのうちにぜひどうぞ!`;
        banner.style.display = "block";
      } else {
        banner.style.display = "none";
      }

      document.getElementById("updatedAt").textContent =
        "更新: " + new Date().toLocaleTimeString("ja-JP");
    }

    async function poll() {
      try {
        const res = await fetch("/api/state");
        if (!res.ok) {
          document.getElementById("updatedAt").textContent = `エラー(${res.status})`;
          return;
        }
        render(await res.json());
      } catch (err) {
        document.getElementById("updatedAt").textContent = "通信エラー";
      }
    }
    setInterval(poll, 2000);
    poll();
  </script>
</body>
</html>
""".replace("__NOKOSAHEN_LOGO__", LOGO_DATA_URI)


class WebUiNode(Node):
    def __init__(self) -> None:
        super().__init__("web_ui")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        # 残数入力画面のBasic認証。両方とも空にすれば従来通り認証無しになる。
        self.declare_parameter("auth_username", "ADMIN_ACCOUNT")
        self.declare_parameter("auth_password", "")
        # メニュー事前登録(menu_schedule_node)と同じDBファイルを参照する。
        # 専用サイト側でまとめて登録した内容を、日付に応じて自動で読み込む。
        self.declare_parameter("db_path", "~/.nokosahen_menu_schedule.db")

        db_path = os.path.expanduser(str(self.get_parameter("db_path").value))
        self.schedule_store = MenuScheduleStore(db_path)

        self._state_lock = threading.Lock()
        self._state: Dict[str, Any] = {
            "menu_state": {},
            "behavior_command": {},
            "announce_text": "",
        }

        self._menu_init_pub = self.create_publisher(
            String, "/nokosahen/menu_init", 10
        )
        self._count_pub = self.create_publisher(
            String, "/nokosahen/count_event", 50
        )
        # 【2026-08-07追加】明電舎タブレットの累計販売数入力用。
        self._sold_total_pub = self.create_publisher(
            String, "/nokosahen/sold_total_event", 20
        )
        self.create_subscription(
            String, "/nokosahen/menu_state", self._on_menu_state, 10
        )
        self.create_subscription(
            String, "/nokosahen/behavior_command", self._on_behavior_command, 10
        )
        self.create_subscription(
            String, "/nokosahen/announce_text", self._on_announce_text, 10
        )

        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        handler = self._make_handler()
        self._server = ThreadingHTTPServer((host, port), handler)
        self._server_thread = threading.Thread(
            target=self._server.serve_forever, daemon=True
        )
        self._server_thread.start()
        self.get_logger().info(f"web UI listening on http://{host}:{port}")

    def destroy_node(self) -> bool:
        self._server.shutdown()
        self._server.server_close()
        return super().destroy_node()

    def _on_menu_state(self, msg: String) -> None:
        with self._state_lock:
            self._state["menu_state"] = loads(msg.data, {})

    def _on_behavior_command(self, msg: String) -> None:
        with self._state_lock:
            self._state["behavior_command"] = loads(msg.data, {})

    def _on_announce_text(self, msg: String) -> None:
        with self._state_lock:
            self._state["announce_text"] = msg.data

    def _publish_json(self, publisher, payload: dict) -> None:
        msg = String()
        msg.data = dumps(payload)
        publisher.publish(msg)

    def _snapshot(self) -> dict:
        with self._state_lock:
            return json.loads(dumps(self._state))

    def _make_handler(self):
        node = self
        auth_username = str(self.get_parameter("auth_username").value)
        auth_password = resolve_password(
            self.get_logger(), "web_ui", "auth_password",
            str(self.get_parameter("auth_password").value),
            str(self.get_parameter("auth_username").value),
        )

        class Handler(BaseHTTPRequestHandler):
            # 【2026-10-21】読み取りのタイムアウト。本文を送らない接続・リクエスト行を送らない接続を大量に張られても、
            # スレッドとソケットが永久に残らないようにする(1台目はCPU負荷が高く、固まると緊急停止の操作も届かなくなる)。
            timeout = 15
            def log_message(self, fmt: str, *args: Any) -> None:
                node.get_logger().debug(fmt % args)

            def _check_auth(self) -> bool:
                if check_basic_auth(self, auth_username, auth_password):
                    return True
                send_auth_required(self, "Nokosahen Web UI")
                return False

            def _send(self, status: int, body: bytes, content_type: str) -> None:
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def _send_json(self, status: int, payload: dict) -> None:
                self._send(
                    status,
                    dumps(payload).encode("utf-8"),
                    "application/json; charset=utf-8",
                )

            def _read_json(self) -> dict:
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                except ValueError:
                    return {}
                if length <= 0 or length > 1024 * 64:
                    return {}
                try:
                    # 不正なUTF-8バイト列(decode)や壊れたJSON(loads内部)を
                    # まとめて弾く。これが無いとUnicodeDecodeErrorが
                    # そのまま例外として上まで漏れ、リクエスト処理スレッドが
                    # 素のトレースバックで落ちてしまう。
                    raw = self.rfile.read(length).decode("utf-8")
                except UnicodeDecodeError:
                    return {}
                data = loads(raw, {})
                # JSON構文としては正しくてもdictでない場合(数値・配列・null等)、
                # 呼び出し側の.get()がAttributeErrorで落ちないようにする。
                return data if isinstance(data, dict) else {}

            def do_GET(self) -> None:
                if not self._check_auth():
                    return
                parsed = urlparse(self.path)
                path = parsed.path
                if path in ("/", "/index.html"):
                    self._send(
                        HTTPStatus.OK,
                        HTML.encode("utf-8"),
                        "text/html; charset=utf-8",
                    )
                    return
                if path == "/customer_display":
                    self._send(
                        HTTPStatus.OK,
                        CUSTOMER_DISPLAY_HTML.encode("utf-8"),
                        "text/html; charset=utf-8",
                    )
                    return
                if path == "/api/state":
                    self._send_json(HTTPStatus.OK, node._snapshot())
                    return
                if path == "/api/menu/scheduled":
                    qs = parse_qs(parsed.query)
                    date_str = (qs.get("date") or [""])[0].strip()
                    if not date_str:
                        date_str = datetime.date.today().isoformat()
                    try:
                        datetime.date.fromisoformat(date_str)
                    except ValueError:
                        self._send_json(HTTPStatus.BAD_REQUEST, {"error": "date must be YYYY-MM-DD"})
                        return
                    items = node.schedule_store.get_menu_for_date(date_str)
                    self._send_json(HTTPStatus.OK, {"date": date_str, "items": items})
                    return
                self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

            def do_POST(self) -> None:
                if not self._check_auth():
                    return
                path = urlparse(self.path).path
                data = self._read_json()
                if path == "/api/menu/init":
                    items = data.get("items", [])
                    # 空リスト([])は「本日のメニューをクリアする」という正当な操作
                    # として受け付ける(行を全部削除して送信した場合に、残数表示を
                    # 実際に空にできるようにするため)。リスト以外の不正な形式のみ
                    # 拒否する。
                    if not isinstance(items, list):
                        self._send_json(
                            HTTPStatus.BAD_REQUEST,
                            {"error": "items must be a list"},
                        )
                        return
                    node._publish_json(node._menu_init_pub, {"items": items})
                    self._send_json(HTTPStatus.OK, {"ok": True})
                    return
                if path == "/api/count":
                    menu = str(data.get("menu", "")).strip()
                    if not menu:
                        self._send_json(
                            HTTPStatus.BAD_REQUEST,
                            {"error": "menu is required"},
                        )
                        return
                    try:
                        delta = int(float(data.get("delta", 1)))
                    except (TypeError, ValueError, OverflowError):
                        self._send_json(
                            HTTPStatus.BAD_REQUEST,
                            {"error": "delta must be a number"},
                        )
                        return
                    payload = {
                        "menu": menu,
                        "delta": delta,
                        "source": str(data.get("source", "web_ui")),
                    }
                    node._publish_json(node._count_pub, payload)
                    self._send_json(HTTPStatus.OK, {"ok": True})
                    return
                if path == "/api/sold_total":
                    menu = str(data.get("menu", "")).strip()
                    if not menu:
                        self._send_json(
                            HTTPStatus.BAD_REQUEST,
                            {"error": "menu is required"},
                        )
                        return
                    try:
                        sold_total = int(float(data.get("sold_total", 0)))
                    except (TypeError, ValueError, OverflowError):
                        self._send_json(
                            HTTPStatus.BAD_REQUEST,
                            {"error": "sold_total must be a number"},
                        )
                        return
                    if sold_total < 0:
                        self._send_json(
                            HTTPStatus.BAD_REQUEST,
                            {"error": "sold_total must not be negative"},
                        )
                        return
                    payload = {
                        "menu": menu,
                        "sold_total": sold_total,
                        "source": str(data.get("source", "web_ui")),
                        # 2026-09-25追加: tablet_ocr_reader.pyがOCR連続失敗時に
                        # TrendPredictorの外挿予測値を送る場合はTrueになる
                        # (inventory_manager.py側でconsumption_logのestimated列
                        # に記録し、消費推移グラフで実測と区別する)。
                        "estimated": bool(data.get("estimated", False)),
                    }
                    node._publish_json(node._sold_total_pub, payload)
                    self._send_json(HTTPStatus.OK, {"ok": True})
                    return
                self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

        return Handler


def main() -> None:
    rclpy.init()
    node = WebUiNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

