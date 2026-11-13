"""Stage1(ESP32 + Windows PC 足回り単体試験)専用の監視・操作ダッシュボード。

このシステムで実際に動いているもの(wheel_odom・ESP32からの/wheel_ticks・
/estop_state)だけを表示する。LiDAR/カメラ/Nav2/音声/残数は
このStageには存在しないため、あえて出さない(要件どおり「全体監視HTMLとは
別」の、実装されている範囲だけを見せる簡易ダッシュボード)。
"""
from __future__ import annotations

import math
from typing import Any, Dict

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray

from .nav_layout import NK_FIELD_JS
from .secrets_loader import resolve_password
from .dashboard_common import (
    DASHBOARD_ODOM_TOPIC,
    PidGainsController,
    StepResponseResultsController,
    TeleopController,
    build_step_response_html,
    make_dashboard_handler,
    start_dashboard_server,
)

HTML = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>のこさへん - Stage1 足回り単体試験</title>
<style>
  :root { color-scheme: light; font-family: system-ui, -apple-system, "Segoe UI", sans-serif; background:#f4f6f5; color:#16211d; }
  html, body { height:100%; margin:0; }
  header { height:44px; box-sizing:border-box; padding:8px 18px; background:#124739; color:#fff; display:flex; justify-content:space-between; align-items:center; }
  /* グリッドだと一番背の高いセクション(ゲイン調整)に合わせて同じ行の
     高さが揃ってしまい、その行に空白ができるのに次のセクションは次の行へ
     追いやられてスクロールが必要になる問題があった。高さがバラバラな
     カードを隙間なく詰めるのに向いたCSS多段組(masonry風)に変更している。 */
  main { min-height:calc(100vh - 44px); box-sizing:border-box; margin:0; padding:14px; columns:360px; column-gap:14px; }
  section { background:#fff; border:1px solid #dce4df; border-radius:8px; padding:14px; break-inside:avoid; margin:0 0 14px; }
  h2 { margin:0 0 10px; font-size:16px; }
  .row { display:flex; gap:16px; flex-wrap:wrap; }
  .metric { background:#eef3f0; border-radius:8px; padding:10px 14px; min-width:120px; }
  .metric span { display:block; font-size:12px; color:#53635c; }
  .metric b { font-size:20px; }
  .estop-badge { padding:4px 12px; border-radius:999px; font-weight:700; font-size:13px; }
  .estop-ok { background:#e1f5ee; color:#0f6e56; }
  .estop-bad { background:#faece7; color:#993c1d; }
  .dpad { display:grid; grid-template-columns:repeat(3, 64px); grid-template-rows:repeat(3, 64px); gap:6px; justify-content:center; }
  .dpad button { font-size:22px; border:0; border-radius:8px; background:#176b56; color:#fff; touch-action:none; user-select:none; -webkit-user-select:none; -webkit-touch-callout:none; }
  .dpad button:active, .dpad button.pressed { background:#0c4a3a; }
  .stop-btn { width:100%; padding:16px; font-size:18px; font-weight:700; border:0; border-radius:8px; background:#a32d2d; color:#fff; margin-top:10px; }
  .stop-btn.engaged { background:#5c1717; }
  .hint { font-size:12px; color:#6b6a64; margin-top:8px; }
</style>
</head>
<body>
<header>
  <h1 style="margin:0;font-size:20px;">Stage1: 足回り単体試験</h1>
  <span id="estopBadge" class="estop-badge estop-ok">--</span>
</header>
<main>
  <section>
    <h2>状態</h2>
    <div class="row">
      <div class="metric"><span>左エンコーダ</span><b id="tickLeft">-</b></div>
      <div class="metric"><span>右エンコーダ</span><b id="tickRight">-</b></div>
      <div class="metric"><span>odom x</span><b id="odomX">-</b></div>
      <div class="metric"><span>odom y</span><b id="odomY">-</b></div>
      <div class="metric"><span>odom yaw(deg)</span><b id="odomYaw">-</b></div>
    </div>
  </section>

  <section>
    <h2>手動運転(WASD / タップ)</h2>
    <div class="dpad">
      <span></span><button id="btnF" data-fwd="1" data-turn="0">W</button><span></span>
      <button id="btnL" data-fwd="0" data-turn="1">A</button>
      <button id="btnStop2" data-fwd="0" data-turn="0">■</button>
      <button id="btnR" data-fwd="0" data-turn="-1">D</button>
      <span></span><button id="btnB" data-fwd="-1" data-turn="0">S</button><span></span>
    </div>
    <p class="hint">キーボードのW/A/S/Dでも操作できます(押している間だけ動く)。車輪を浮かせた状態で試すこと。</p>
    <button id="estopBtn" class="stop-btn">緊急停止(ソフト)</button>
    <p class="hint">これはソフトウェア側の停止です。GPIO34の物理配線が本来の緊急停止です。</p>
  </section>

  <section>
    <h2>手動運転の最高速度</h2>
    <div class="row">
      <label>直進(m/s) <input id="manualLinear" type="number" step="0.01" min="0.01" max="1.1" style="width:80px"></label>
      <label>旋回(rad/s) <input id="manualAngular" type="number" step="0.01" min="0.01" max="3" style="width:80px"></label>
      <button id="speedSaveBtn">保存</button>
    </div>
    <p class="hint">直進速度の上限は1.1m/s(ファーム側の速度制限)。これを超える値は1.1m/sに丸められます。</p>
    <p class="hint" id="speedSaveHint">&nbsp;</p>
  </section>

  <section>
    <h2>PIDゲイン調整(再書き込み不要・即反映)</h2>
    <p class="hint">ESP32ファームウェアが今使っている値。前進(FWD)/後退(BWD)で別のゲインを持てます(後退すると直進からずれる場合に個別調整してください)。変更すると即座に反映され、積分項もリセットされます。</p>
    <p class="hint" style="font-weight:700;margin-top:10px;">右車輪</p>
    <div class="row">
      <label>前進 KP <input id="gainRkpFwd" type="number" step="0.1" style="width:70px"></label>
      <label>前進 KI <input id="gainRkiFwd" type="number" step="0.1" style="width:70px"></label>
      <label>前進 KD <input id="gainRkdFwd" type="number" step="0.1" style="width:70px"></label>
    </div>
    <div class="row" style="margin-top:6px;">
      <label>後退 KP <input id="gainRkpBwd" type="number" step="0.1" style="width:70px"></label>
      <label>後退 KI <input id="gainRkiBwd" type="number" step="0.1" style="width:70px"></label>
      <label>後退 KD <input id="gainRkdBwd" type="number" step="0.1" style="width:70px"></label>
    </div>
    <p class="hint" style="font-weight:700;margin-top:14px;">左車輪</p>
    <div class="row">
      <label>前進 KP <input id="gainLkpFwd" type="number" step="0.1" style="width:70px"></label>
      <label>前進 KI <input id="gainLkiFwd" type="number" step="0.1" style="width:70px"></label>
      <label>前進 KD <input id="gainLkdFwd" type="number" step="0.1" style="width:70px"></label>
    </div>
    <div class="row" style="margin-top:6px;">
      <label>後退 KP <input id="gainLkpBwd" type="number" step="0.1" style="width:70px"></label>
      <label>後退 KI <input id="gainLkiBwd" type="number" step="0.1" style="width:70px"></label>
      <label>後退 KD <input id="gainLkdBwd" type="number" step="0.1" style="width:70px"></label>
    </div>
    <button id="gainSaveBtn" style="margin-top:10px;">ゲインを送信</button>
    <p class="hint" id="gainSaveHint">&nbsp;</p>
  </section>

  <section>
    <h2>距離/時間指定 走行テスト</h2>
    <p class="hint">必ず車輪を浮かせた状態、またはすぐ止められる状態で実行すること。実行中は他の操作と競合しないよう1つずつ実行されます。</p>
    <div class="row">
      <label>動作
        <select id="testMode">
          <option value="forward">前進</option>
          <option value="backward">後退</option>
          <option value="rotate_left">その場回転(左)</option>
          <option value="rotate_right">その場回転(右)</option>
        </select>
      </label>
      <label id="testDistanceLabel">距離(m) <input id="testDistance" type="number" step="0.1" value="1.0" style="width:70px"></label>
      <label id="testAngleLabel" style="display:none;">回転角度(度) <input id="testAngle" type="number" step="10" value="90" style="width:70px"></label>
      <label>時間(秒) <input id="testDuration" type="number" step="0.5" min="0.1" max="15" value="5.0" style="width:70px"></label>
      <button id="testRunBtn">実行</button>
    </div>
    <p class="hint" id="testRunHint">&nbsp;</p>
    <a href="/step_response" target="_blank" style="display:block;margin-top:10px;"><button type="button" style="width:100%;">ステップ応答テスト結果を見る</button></a>
  </section>
</main>
<script>
__NK_FIELD_JS__
  const state = { forward: 0, turn: 0, estopEngaged: false };

  function showError(message) {
    let banner = document.getElementById("errorBanner");
    if (!banner) {
      banner = document.createElement("div");
      banner.id = "errorBanner";
      banner.style.cssText = "background:#faece7;color:#993c1d;padding:6px 10px;font-size:12px;text-align:center;";
      document.body.insertBefore(banner, document.body.firstChild);
    }
    banner.textContent = message;
    banner.style.display = message ? "block" : "none";
  }

  async function postJson(path, payload, opts) {
    try {
      const res = await fetch(path, Object.assign({ method: "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify(payload || {}) }, opts || {}));
      if (!res.ok) {
        let msg = `操作エラー(${res.status})`;
        try { const data = await res.json(); if (data && data.message) msg = data.message; } catch (e2) { /* 本文がJSONでなくても既定メッセージのまま続行 */ }
        showError(msg);
        return false;
      }
      showError("");
      return true;
    } catch (e) {
      showError("通信エラー: サーバーに接続できません");
      return false;
    }
  }

  const teleopTabId = (() => {
    try { const a = new Uint8Array(8); crypto.getRandomValues(a); return Array.from(a, (b) => b.toString(16).padStart(2, "0")).join(""); }
    catch (e) { return (Math.random().toString(36).slice(2, 14) || "t"); }
  })();
  let teleopSeq = 0;
  let driveInFlightSince = 0;
  let driveInFlightSeq = 0;
  // 【2026-10-03修正】seq/tab_idを付与し、非停止の前回リクエストが未完了なら周期送信を見送る(停止は常に送る)。
  function sendDrive() {
    const stop = state.forward === 0 && state.turn === 0;
    if (!stop && driveInFlightSince && (Date.now() - driveInFlightSince) < 1000) return;
    const seq = ++teleopSeq;
    const p = postJson("/api/teleop", { forward: state.forward, turn: state.turn, seq, tab_id: teleopTabId }, { keepalive: true });
    if (stop) { driveInFlightSince = 0; return; }
    driveInFlightSince = Date.now(); driveInFlightSeq = seq;
    p.finally(() => { if (driveInFlightSeq === seq) driveInFlightSince = 0; });
  }

  const keyMap = { "w":[1,0], "s":[-1,0], "a":[0,1], "d":[0,-1] };
  const pressed = new Set();

  function recomputeFromKeys() {
    let f = 0, t = 0;
    for (const k of pressed) {
      const [kf, kt] = keyMap[k] || [0,0];
      f += kf; t += kt;
    }
    state.forward = Math.max(-1, Math.min(1, f));
    state.turn = Math.max(-1, Math.min(1, t));
  }

  // 【2026-10-02追加】入力欄(例: PIDゲイン欄)への文字入力でW/A/S/Dが走行にならないよう、
  // 入力欄フォーカス中・修飾キー併用・キーリピートは無視する。
  function isTypingTarget(e) {
    const t = e.target;
    if (!t) return false;
    const tag = t.tagName || "";
    return tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT" || !!t.isContentEditable;
  }
  // 押しっぱなし状態を全て破棄して停止を送る(Alt+Tab・別ウィンドウのクリック・
  // confirm()等でkeyupを受け取れない場合に走り続けるのを防ぐ)。
  function stopAll() {
    pressed.clear();
    activePointerId = null;
    state.forward = 0; state.turn = 0;
    document.querySelectorAll(".dpad button.pressed").forEach((b) => b.classList.remove("pressed"));
    sendDrive();
  }
  window.addEventListener("keydown", (e) => {
    if (e.repeat || e.ctrlKey || e.metaKey || e.altKey) return;
    if (isTypingTarget(e)) return;
    const k = e.key.toLowerCase();
    if (!(k in keyMap)) return;
    pressed.add(k);
    recomputeFromKeys();
    sendDrive(); // キー入力直後に即送信(最大100ms周期を待たずに反映する)
  });
  window.addEventListener("keyup", (e) => {
    const k = e.key.toLowerCase();
    if (!(k in keyMap)) return;
    pressed.delete(k);
    recomputeFromKeys();
    sendDrive(); // キーを離した瞬間も即送信(離してから最大100ms止まらない遅延を無くす)
  });

  // 【2026-10-02修正】Pointer Events+setPointerCaptureに統一。touchcancel/pointercancel/
  // 画面ロック/タブ非表示でも停止を送る(以前はtouchendしか見ておらず、キャンセルされると
  // 前進を送り続けた)。
  let activePointerId = null;
  document.querySelectorAll(".dpad button").forEach((btn) => {
    const fwd = Number(btn.dataset.fwd), turn = Number(btn.dataset.turn);
    btn.addEventListener("pointerdown", (ev) => {
      ev.preventDefault();
      if (activePointerId !== null && activePointerId !== ev.pointerId) { stopAll(); return; }
      activePointerId = ev.pointerId;
      try { btn.setPointerCapture(ev.pointerId); } catch (e) { /* 取れない場合はpointerleaveで停止する */ }
      state.forward = fwd; state.turn = turn; btn.classList.add("pressed"); sendDrive();
    });
    btn.addEventListener("pointerup", stopAll);
    btn.addEventListener("pointercancel", stopAll);
    btn.addEventListener("lostpointercapture", stopAll);
    btn.addEventListener("pointerleave", (ev) => {
      let captured = false;
      try { captured = btn.hasPointerCapture(ev.pointerId); } catch (e) { captured = false; }
      if (!captured && activePointerId === ev.pointerId) stopAll();
    });
    btn.addEventListener("touchcancel", stopAll);
    btn.addEventListener("contextmenu", (ev) => ev.preventDefault());
  });
  window.addEventListener("blur", stopAll);
  window.addEventListener("pagehide", stopAll);
  document.addEventListener("visibilitychange", () => { if (document.hidden) stopAll(); });
  // confirm()はkeyupを飲み込むことがあるため、表示前に必ず停止する。
  const rawConfirm = window.confirm.bind(window);
  window.confirm = (msg) => { stopAll(); return rawConfirm(msg); };
  // alert()もkeyupを飲み込むため、表示前に必ず停止する。
  const rawAlert = window.alert.bind(window);
  window.alert = (m) => { stopAll(); rawAlert(m); };

  setInterval(sendDrive, 100);

  const estopBtn = document.getElementById("estopBtn");
  // 【2026-10-02修正】ボタン表示はサーバー側の実際の状態(/api/stateのsoft_estop_engaged)に
  // 同期させる。以前は押した瞬間に表示だけ反転し、POST失敗(409/403/通信エラー)でも
  // 「停止中」と表示され、再読み込み・他端末の操作とも食い違っていた。
  let estopPending = false;
  let estopIgnoreSyncUntil = 0;
  function renderEstop(engaged) {
    state.estopEngaged = engaged;
    estopBtn.textContent = engaged ? "緊急停止を解除" : "緊急停止(ソフト)";
    estopBtn.classList.toggle("engaged", engaged);
  }
  function syncEstop(s) {
    if (estopPending || Date.now() < estopIgnoreSyncUntil) return;
    if (typeof s.soft_estop_engaged === "boolean") renderEstop(s.soft_estop_engaged);
  }
  function showEstopWarning(message) {
    let banner = document.getElementById("estopWarning");
    if (!banner) {
      banner = document.createElement("div");
      banner.id = "estopWarning";
      banner.style.cssText = "background:#a32d2d;color:#fff;padding:8px 10px;font-size:13px;font-weight:700;text-align:center;";
      document.body.insertBefore(banner, document.body.firstChild);
    }
    banner.textContent = message;
    banner.style.display = message ? "block" : "none";
  }
  estopBtn.addEventListener("click", async () => {
    const engaged = !state.estopEngaged;
    if (engaged) stopAll(); // 停止側へ倒す(alert/通信失敗でkeyupを失っても走り続けないように)
    if (estopPending) return; // 二重送信防止
    const prev = state.estopEngaged;
    estopPending = true;
    estopBtn.disabled = true;
    renderEstop(engaged);
    let ok = false;
    try {
      ok = await postJson("/api/soft_estop", { engaged });
    } finally {
      estopPending = false;
      estopBtn.disabled = false;
      estopIgnoreSyncUntil = Date.now() + 1500;
    }
    if (ok) { showEstopWarning(""); return; }
    renderEstop(prev);
    if (engaged) {
      showEstopWarning("緊急停止の送信に失敗しました。機体は止まっていない可能性があります。物理の非常停止を使ってください。");
      alert("緊急停止の送信に失敗しました。機体は止まっていない可能性があります。物理の非常停止を使ってください。");
    } else {
      showEstopWarning("緊急停止の解除に失敗しました(緊急停止中のままです)。");
    }
  });

  async function poll() {
    try {
      const res = await fetch("/api/state");
      const s = await res.json();
      syncEstop(s);
      document.getElementById("tickLeft").textContent = s.wheel_ticks ? s.wheel_ticks[0] : "-";
      document.getElementById("tickRight").textContent = s.wheel_ticks ? s.wheel_ticks[1] : "-";
      document.getElementById("odomX").textContent = s.odom ? s.odom.x.toFixed(3) : "-";
      document.getElementById("odomY").textContent = s.odom ? s.odom.y.toFixed(3) : "-";
      document.getElementById("odomYaw").textContent = s.odom ? (s.odom.yaw_deg.toFixed(1)) : "-";
      const badge = document.getElementById("estopBadge");
      const engaged = !!s.estop_state;
      badge.textContent = engaged ? "緊急停止中" : "正常";
      badge.className = "estop-badge " + (engaged ? "estop-bad" : "estop-ok");
    } catch (e) {
      document.getElementById("estopBadge").textContent = "接続断";
    }
  }
  setInterval(poll, 500);
  poll();

  async function loadSpeedSettings() {
    try {
      const res = await fetch("/api/speed_settings");
      const s = await res.json();
      // 入力中・未保存の編集(dirty)・保存確認中の欄は書き換えない(NKField)
      NKField.set("manualLinear", s.manual_linear_mps);
      NKField.set("manualAngular", s.manual_angular_rps);
      return s;
    } catch (e) { /* 未取得ならinput空のまま */ return null; }
  }
  const SPEED_KEY_ID = { manual_linear_mps: "manualLinear", manual_angular_rps: "manualAngular" };
  NKField.guard(Object.values(SPEED_KEY_ID), { clamp: true });  // 範囲はmin/max属性。範囲外は入力時に表示
  async function speedRead() { try { return await (await fetch("/api/speed_settings")).json(); } catch (e) { return null; } }
  document.getElementById("speedSaveBtn").addEventListener("click", async () => {
    // 【2026-10-02修正】空欄/数値でない欄は送らない(Number("")=0が最小値0.01m/sに丸められて
    // 保存されるのを防ぐ)。
    const req = {};
    for (const [key, id] of [["manual_linear_mps", "manualLinear"], ["manual_angular_rps", "manualAngular"]]) {
      const raw = document.getElementById(id).value.trim();
      if (raw === "" || !NKField.isDirty(id)) continue;  // 触った欄だけ送る
      const v = Number(raw);
      if (Number.isFinite(v)) req[key] = v;
    }
    const hint = document.getElementById("speedSaveHint");
    if (!Object.keys(req).length) { hint.textContent = "保存する値が入力されていません(値を変えてから保存してください)"; return; }
    const tx = NKField.begin("speed", Object.keys(req).map((k) => SPEED_KEY_ID[k]));  // 送信中は定期取得で入力欄を上書きしない
    const ok = await postJson("/api/speed_settings", req);
    if (!ok) { tx.fail(); hint.textContent = "保存できませんでした(入力した値は残しています)"; return; }
    // 保存後にサーバーが実際に適用した値を読み直して入力欄へ反映する
    // (上限を超える値はサーバー側で丸められるため、入力した値のまま残さない)。送った値と違う(丸めでもない)ときは警告して入力値を残す。
    const labels = { manual_linear_mps: "手動直進", manual_angular_rps: "手動旋回", auto_linear_mps: "自動移動直進", auto_angular_rps: "自動移動旋回" };
    const accept = (key) => (sent, v, srv) => {
      const lim = (srv && srv.limits) || {}, base = key.replace(/_(mps|rps)$/, "");
      const mx = lim[base + "_max"], mn = lim[base + "_min"] !== undefined ? lim[base + "_min"] : 0.01;
      return (typeof mx === "number" && sent > mx && Math.abs(v - mx) < 1e-6) || (sent < mn && Math.abs(v - mn) < 1e-6);
    };
    const vr = await tx.verify({
      items: Object.keys(req).map((k) => ({ el: SPEED_KEY_ID[k], sent: req[k], label: labels[k], read: (srv) => srv[k], accept: accept(k) })),
      load: speedRead, delays: [0, 800, 2000],
    });
    const s = vr.server;
    if (vr.state === "mismatch") { hint.textContent = "反映されていません: " + vr.rows.filter((x) => x.status === "mismatch").map((x) => `${x.label} 送信 ${x.sent} / サーバー ${x.server}`).join("、"); return; }
    if (vr.state === "unknown") { hint.textContent = "保存を送信しましたが、反映を確認できませんでした(サーバーから値を読めません)"; return; }
    const clamped = [];
    if (s) {
      for (const k of Object.keys(req)) {
        if (s[k] !== undefined && Math.abs(Number(req[k]) - Number(s[k])) > 1e-6) {
          clamped.push(`${labels[k]} ${req[k]}→${s[k]}`);
        }
      }
    }
    if (clamped.length) {
      hint.textContent = `上限/下限に丸めて保存しました: ${clamped.join("、")}(直進の上限は${s.max_linear_mps}m/s)`;
      setTimeout(() => { hint.textContent = " "; }, 8000);
    } else {
      hint.textContent = "保存しました";
      setTimeout(() => { hint.textContent = " "; }, 2000);
    }
  });
  loadSpeedSettings();

  const gainFields = [
    "rkp_fwd", "rki_fwd", "rkd_fwd", "rkp_bwd", "rki_bwd", "rkd_bwd",
    "lkp_fwd", "lki_fwd", "lkd_fwd", "lkp_bwd", "lki_bwd", "lkd_bwd",
  ];
  function gainInputId(field) {
    // "rkp_fwd" -> "gainRkpFwd"
    const camel = field.split("_").map(s => s[0].toUpperCase() + s.slice(1)).join("");
    return "gain" + camel;
  }
  const gainInputIds = gainFields.map(gainInputId);

  // 入力中・未保存の編集(dirty)がある欄は自動更新で上書きしない(NKField)。1つでも未保存の編集があれば、対応関係が
  // 分かりにくくなるので全体の更新を見送る(保存するか読み込み直すまで)。
  NKField.guard(gainInputIds, { tol: 1e-3 });  // ESP32はfloat32で返すので小さな誤差は同じ値とみなす
  async function gainsRead() { try { const r = await fetch("/api/pid_gains"); return r.ok ? await r.json() : null; } catch (e) { return null; } }
  async function loadGains() {
    if (NKField.dirtyOf(gainInputIds).length) return;
    const tk = NKField.ticket("gains");
    try {
      const res = await fetch("/api/pid_gains");
      if (!res.ok) return;
      const g = await res.json();
      if (!NKField.fresh(tk)) return;  // 送信前に始まった古い取得結果は捨てる
      for (const field of gainFields) {
        NKField.set(gainInputId(field), g[field]);
      }
    } catch (e) { /* ESP32未接続ならinput空のまま */ }
  }
  document.getElementById("gainSaveBtn").addEventListener("click", async () => {
    const hint = document.getElementById("gainSaveHint");
    const payload = {};
    for (const field of gainFields) {
      const raw = document.getElementById(gainInputId(field)).value.trim();
      const v = Number(raw);
      // 空欄を0(ゲイン0)として送らない。APIは12個すべてを要求するため、足りなければ送らない
      if (raw === "" || !Number.isFinite(v)) { hint.textContent = "空欄または数値でない欄があります(12個すべて入力してください)"; return; }
      payload[field] = v;
    }
    const tx = NKField.begin("gains", gainInputIds);
    const ok = await postJson("/api/pid_gains", payload);
    if (!ok) { tx.fail(); hint.textContent = "送信に失敗しました(入力した値は残しています)"; return; }
    // ESP32が返す現在のゲインを読み直して、送った値と一致したことを確かめる(floatの誤差は許容)
    const vr = await tx.verify({
      items: gainFields.map((f) => ({ el: gainInputId(f), sent: payload[f], label: f, read: (g) => g[f] })),
      load: gainsRead, delays: [0, 800, 2000, 3500],
    });
    hint.textContent = vr.state === "mismatch" ? "ESP32に反映されていません: " + vr.rows.filter((x) => x.status === "mismatch").map((x) => `${x.label} 送信 ${x.sent} / ESP32 ${x.server}`).join("、")
      : (vr.state === "unknown" ? "送信しましたが、ESP32の現在値を読めないため反映を確認できません" : "送信しました(積分項リセット済み・ESP32の値で確認済み)");
    setTimeout(() => { hint.textContent = " "; }, vr.state === "ok" || vr.state === "clamped" ? 2500 : 12000);
  });
  loadGains();
  NKField.poll(loadGains, 3000);

  const testModeSel = document.getElementById("testMode");
  testModeSel.addEventListener("change", () => {
    const rotate = testModeSel.value.startsWith("rotate");
    document.getElementById("testDistanceLabel").style.display = rotate ? "none" : "";
    document.getElementById("testAngleLabel").style.display = rotate ? "" : "none";
  });
  document.getElementById("testRunBtn").addEventListener("click", async () => {
    const mode = testModeSel.value;
    const duration_s = Number(document.getElementById("testDuration").value);
    const payload = { mode, duration_s };
    if (mode.startsWith("rotate")) {
      payload.rotate_deg = Number(document.getElementById("testAngle").value);
    } else {
      payload.distance_m = Number(document.getElementById("testDistance").value);
    }
    const ok = await postJson("/api/drive_test", payload);
    const hint = document.getElementById("testRunHint");
    hint.textContent = ok ? "実行中..." : "実行できませんでした(他のテスト実行中の可能性)";
    setTimeout(() => { hint.textContent = " "; }, Math.max(2500, duration_s * 1000));
  });
</script>
</body>
</html>
"""


class Stage1DashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("stage1_dashboard")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8090)
        self.declare_parameter("linear_speed_mps", 1.0)
        self.declare_parameter("angular_speed_rps", 1.5)
        # 操作盤(管理者用)のBasic認証。両方とも空にすれば従来通り認証無しになる。
        self.declare_parameter("auth_username", "TL")
        self.declare_parameter("auth_password", "")
        # scripts/step_response_test/step_response_test.pyが結果を書き込む
        # ディレクトリを読み取り専用で見に行く(2026-09-24追加、/step_response
        # が使う)。CLI側の既定出力先(--out)と同じパスにすること。
        self.declare_parameter("step_response_results_dir", "~/.nokosahen_step_response_tests")

        self._wheel_ticks = None
        self._odom: Dict[str, float] = {}
        self._estop_state = False

        self.create_subscription(Int32MultiArray, "/wheel_ticks", self._on_ticks, 20)
        self.create_subscription(Odometry, DASHBOARD_ODOM_TOPIC, self._on_odom, 10)
        self.create_subscription(Bool, "/estop_state", self._on_estop, 10)

        self.teleop = TeleopController(
            self,
            linear_speed=float(self.get_parameter("linear_speed_mps").value),
            angular_speed=float(self.get_parameter("angular_speed_rps").value),
        )
        self.pid_gains = PidGainsController(self, teleop=self.teleop)
        # 走行テスト中だけ購読する高頻度odomを、このダッシュボードのstate用odomにも
        # 流す(CLIが/api/stateから読むyaw/x/yを、間引き前の精度にするため)。
        self.pid_gains.add_odom_listener(self._on_odom)
        self.step_response_results = StepResponseResultsController(
            str(self.get_parameter("step_response_results_dir").value)
        )

        handler = make_dashboard_handler(
            html_getter=lambda: HTML.replace("__NK_FIELD_JS__", NK_FIELD_JS).encode("utf-8"),
            state_getter=self._state,
            teleop=self.teleop,
            map_goal=None,  # Stage1にはSLAM/Nav2が無いので地図クリック運転は無し
            pid_gains=self.pid_gains,
            step_response_results=self.step_response_results,
            step_response_html_getter=build_step_response_html,
            auth_username=str(self.get_parameter("auth_username").value),
            auth_password=resolve_password(
                self.get_logger(), "stage1_dashboard", "auth_password",
                str(self.get_parameter("auth_password").value),
                str(self.get_parameter("auth_username").value),
            ),
            # Stage1はESP32足回り単体のベンチ試験用ツールなので、完成版とは
            # 違い同時1台制限は適用しない(複数端末から見比べながら試験できる方が便利)。
            enforce_single_controller=False,
        )
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self._server = start_dashboard_server(self, host, port, handler)

    def destroy_node(self) -> bool:
        self._server.shutdown()
        self._server.server_close()
        return super().destroy_node()

    def _on_ticks(self, msg: Int32MultiArray) -> None:
        self._wheel_ticks = list(msg.data)

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw_deg": math.degrees(yaw),
        }

    def _on_estop(self, msg: Bool) -> None:
        self._estop_state = bool(msg.data)

    def _state(self) -> Dict[str, Any]:
        state = {
            "wheel_ticks": self._wheel_ticks,
            "odom": self._odom or None,
            "estop_state": self._estop_state,
        }
        state.update(self.teleop.status())
        return state


def main() -> None:
    rclpy.init()
    node = Stage1DashboardNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
