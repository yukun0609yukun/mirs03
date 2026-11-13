"""Stage2(Jetson Orin Nano + ESP32 + LiDAR + Webカメラ、残数カウントは無効)
専用の監視・操作ダッシュボード。

このStageで実際に動いているもの(crowd_monitor・person_target_detector・
behavior_director・nokosahen_voice・Nav2/SLAM)だけを表示する。
残数カウント(inventory_manager/camera_counter/web_ui)はこのStageでは
無効なので出さない(完成版は`full_dashboard_node.py`で別途用意する)。
"""
from __future__ import annotations

import math
import time
from typing import Any, Dict

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .assets import LOGO_DATA_URI
from .secrets_loader import resolve_password
from .dashboard_common import (
    DASHBOARD_ODOM_TOPIC,
    AutoSpeedController,
    BehaviorDirectorControlController,
    BoundaryController,
    CameraPreviewController,
    CrowdBackgroundController,
    FrontierExplorerController,
    KeepOutZoneController,
    MapGoalController,
    MapLifecycleController,
    MappingController,
    PidGainsController,
    RobotPoseController,
    RouteTrailController,
    ScanController,
    StepResponseResultsController,
    TeleopController,
    build_detection_log_html,
    build_map_editor_html,
    build_step_response_html,
    build_utterance_log_html,
    make_dashboard_handler,
    make_mobile_handler,
    start_dashboard_server,
)
from .detection_log import DetectionLog
from .json_tools import loads
from .map_view import MAP_VIEW_JS
from .nav_layout import NK_FIELD_JS
from .utterance_log import UtteranceLogReader

HTML = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>のこさへん - Stage2 実地試験(残数カウント無効)</title>
<style>
  :root { color-scheme: light; font-family: system-ui, -apple-system, "Segoe UI", sans-serif; background:#f4f6f5; color:#16211d; }
  html, body { height:100%; margin:0; overflow:hidden; }
  header { height:44px; box-sizing:border-box; padding:8px 18px; background:#124739; color:#fff; display:flex; justify-content:space-between; align-items:center; }
  main {
    height:calc(100vh - 44px); box-sizing:border-box; padding:10px; display:grid; gap:10px;
    grid-template-columns: 0.9fr 1fr 1fr 1fr;
    grid-template-rows: 1fr auto;
    grid-template-areas: "state map lidar camera" "drive map lidar camera";
  }
  @media (max-width:1100px) {
    main { grid-template-columns: 1fr; grid-template-rows: auto auto auto auto auto; grid-template-areas: "state" "map" "lidar" "camera" "drive"; overflow-y:auto; }
    html, body { overflow:auto; }
  }
  .s-state { grid-area: state; }
  .s-map { grid-area: map; }
  .s-lidar { grid-area: lidar; }
  .s-camera { grid-area: camera; }
  .s-drive { grid-area: drive; }
  #mapCanvas { flex:1 1 0; min-height:420px; }
  @media (max-width:640px) { #mapCanvas { min-height:60vh; } }
  section { background:#fff; border:1px solid #dce4df; border-radius:8px; padding:10px; min-height:0; display:flex; flex-direction:column; overflow:auto; }
  h2 { margin:0 0 8px; font-size:14px; flex:0 0 auto; }
  .row { display:flex; gap:10px; flex-wrap:wrap; flex:0 0 auto; }
  .metric { background:#eef3f0; border-radius:8px; padding:6px 10px; min-width:100px; }
  .metric span { display:block; font-size:11px; color:#53635c; }
  .metric b { font-size:16px; }
  .estop-badge { padding:4px 12px; border-radius:999px; font-weight:700; font-size:13px; }
  .estop-ok { background:#e1f5ee; color:#0f6e56; }
  .estop-bad { background:#faece7; color:#993c1d; }
  .estop-policy { background:#fff3cd; color:#7a5b00; }
  .header-right { display:flex; align-items:center; gap:10px; }
  #accountLabel { font-size:12px; opacity:0.85; }
  .logout-btn { padding:4px 10px; font-size:12px; border:1px solid rgba(255,255,255,0.5); border-radius:6px; background:transparent; color:#fff; cursor:pointer; }
  .logout-btn:hover { background:rgba(255,255,255,0.15); }
  .stop-btn { width:100%; padding:10px; font-size:14px; font-weight:700; border:0; border-radius:8px; background:#a32d2d; color:#fff; margin-top:8px; flex:0 0 auto; }
  .stop-btn.engaged { background:#5c1717; }
  .resume-btn { width:100%; padding:8px; font-size:13px; border:0; border-radius:8px; background:#4e5e57; color:#fff; margin-top:6px; flex:0 0 auto; }
  .map-save-row { display:flex; gap:6px; margin-top:8px; flex:0 0 auto; }
  .map-save-row input { flex:1 1 auto; min-width:0; padding:6px; border:1px solid #dce4df; border-radius:6px; font-size:12px; }
  .map-save-row button { padding:6px 10px; font-size:12px; border:0; border-radius:6px; background:#176b56; color:#fff; }
  canvas { width:100%; flex:1 1 auto; min-height:0; background:#0c1b16; border-radius:8px; cursor:crosshair; touch-action:none; }
  .hint { font-size:11px; color:#6b6a64; margin-top:6px; flex:0 0 auto; }
  #announce { font-size:13px; background:#fff6df; border-radius:8px; padding:6px 8px; min-height:1.2em; flex:0 0 auto; }
</style>
</head>
<body>
<header>
  <h1 style="margin:0;font-size:18px;">Stage2: 実地試験(残数カウント無効)</h1>
  <div class="header-right">
    <span id="estopBadge" class="estop-badge estop-ok">--</span>
    <span id="accountLabel"></span>
    <button id="logoutBtn" class="logout-btn">ログアウト</button>
  </div>
</header>
<main>
  <section class="s-state">
    <h2>状態</h2>
    <div class="row">
      <div class="metric"><span>混雑度</span><b id="crowdLevel">-</b></div>
      <div class="metric"><span>行動モード</span><b id="mode">-</b></div>
      <div class="metric"><span>発話中</span><b id="audioBusy">-</b></div>
      <div class="metric"><span>手動運転</span><b id="manualOverride">-</b></div>
      <div class="metric"><span>odom x,y</span><b id="odomXY">-</b></div>
    </div>
    <h2 style="margin-top:10px;">発話内容</h2>
    <p id="announce">-</p>
    <a href="/utterance_log" target="_blank"><button type="button" style="width:100%;">発話履歴を見る</button></a>
    <a href="/detection_log" target="_blank" style="display:block;margin-top:6px;"><button type="button" style="width:100%;">人物検知ログを見る</button></a>
    <a href="/step_response" target="_blank" style="display:block;margin-top:6px;"><button type="button" style="width:100%;">ステップ応答テスト結果を見る</button></a>
  </section>

  <section class="s-map">
    <h2>地図(クリックでそこへ自動走行)</h2>
    <canvas id="mapCanvas"></canvas>
    <p class="hint">地図が育つ(SLAMが進む)まで表示は空です。クリックした地点へNav2が経路計画して移動します。緑の線=実際に通った軌跡、青の破線=Nav2の計画経路。</p>
    <button id="resumeBtn" class="resume-btn">自律運転に戻す</button>
    <p class="hint">地図モード: <b id="mapModeLabel">-</b></p>
    <div class="map-save-row">
      <button id="mapStartBtn">地図作成を開始</button>
      <button id="mapStopBtn">停止</button>
      <button id="mapResetBtn">リセット</button>
    </div>
    <div class="map-save-row">
      <input id="mapSaveName" type="text" placeholder="保存名(英数字・_・-、空欄なら自動命名)" />
      <button id="mapSaveBtn">地図を保存</button>
    </div>
    <div class="map-save-row">
      <select id="mapLoadSelect"></select>
      <button id="mapLoadBtn">この地図を読み込む</button>
    </div>
    <p id="mapSaveHint" class="hint"></p>
    <div class="map-save-row">
      <button id="boundarySetBtn">走行範囲を設定(地図を2回クリック)</button>
      <button id="boundaryClearBtn">範囲解除</button>
    </div>
    <p id="boundaryHint" class="hint">黄色い点線が現在の走行可能範囲(自律動作と地図クリックの目的地に適用、WASD手動運転は対象外)。四隅の□をドラッグで調整できます。</p>
    <div class="map-save-row">
      <a href="/map_editor" target="_blank" style="flex:1 1 auto;"><button type="button" style="width:100%;">地図を大きく開いて範囲を編集</button></a>
    </div>
    <p class="hint">走行可能範囲(外周)に加えて、地図内の「立入禁止エリア」を複数指定できます(別タブで開きます)。</p>
    <div class="map-save-row">
      <button id="explorationStartBtn">自律探索を開始</button>
      <button id="explorationStopBtn">停止</button>
    </div>
    <p id="explorationHint" class="hint">地図作成中に自分で走り回って探索する(手動運転が入ったら即座にそちらを優先)。</p>
  </section>

  <section class="s-lidar">
    <h2>LiDARの見え方(自機中心・前方が上)</h2>
    <canvas id="lidarCanvas" width="360" height="360"></canvas>
    <p class="hint">緑の三角が自機、赤い点がLiDARで検出した障害物。円は1m刻みの目安。</p>
    <button id="calibrateBgBtn">混雑判定の背景をキャリブレーション</button>
    <p id="calibrateBgHint" class="hint">壁・什器等が近い設置場所での誤判定(無人でも「混雑」になる)対策。部屋を無人にしてから押してください。</p>
  </section>

  <section class="s-camera">
    <h2>正面カメラの見え方</h2>
    <img id="cameraView" alt="カメラ映像なし" style="width:100%;flex:1 1 auto;min-height:0;object-fit:contain;background:#0c1b16;border-radius:8px;" />
    <p class="hint">緑の枠が人物として認識している範囲(数字は確度)。数百ms毎の静止画更新のため、滑らかな動画ではありません。</p>
  </section>

  <section class="s-drive">
    <h2>緊急停止・最高速度設定</h2>
    <p class="hint">手動運転(D-pad)はスマホ操作用ページに移動しました(下部「スマホ操作」参照)。ここでは緊急停止と、手動/自動移動の最高速度の確認・変更ができます。</p>
    <button id="estopBtn" class="stop-btn">緊急停止(ソフト)</button>
    <h2 style="margin-top:14px;">最高速度設定</h2>
    <div class="row">
      <label>手動直進(m/s) <input id="manualLinear" type="number" step="0.01" min="0.01" max="1.1" style="width:70px"></label>
      <label>手動旋回(rad/s) <input id="manualAngular" type="number" step="0.01" min="0.01" max="3" style="width:70px"></label>
    </div>
    <div class="row">
      <label>自動移動直進(m/s) <input id="autoLinear" type="number" step="0.01" min="0.01" max="1.1" style="width:70px"></label>
      <label>自動移動旋回(rad/s) <input id="autoAngular" type="number" step="0.01" min="0.01" max="3" style="width:70px"></label>
    </div>
    <button id="speedSaveBtn">保存</button>
    <p class="hint">直進速度の上限は1.1m/s(ファーム側の速度制限)。これを超える値は1.1m/sに丸められます。</p>
    <p class="hint" id="speedSaveHint">&nbsp;</p>
    <h2 style="margin-top:14px;">スマホ操作</h2>
    <p class="hint">同じログイン(アカウント/パスワード共通)でスマホからD-padの手動運転ができます: <b id="mobileUrl">-</b></p>
  </section>
</main>
<script>
__NK_FIELD_JS__
</script>
<script>
  // 401(未ログイン/セッション切れ)を受け取ったら自動でログイン画面へ誘導する。
  (function () {
    const rawFetch = window.fetch.bind(window);
    window.fetch = async (...args) => {
      const res = await rawFetch(...args);
      if (res.status === 401 && location.pathname !== "/login") {
        location.href = "/login";
      }
      return res;
    };
  })();

  const state = { estopEngaged: false, forward: 0, turn: 0 };

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

  // 【2026-10-02追加】結果のメッセージを返すPOST。通信エラー・非JSON応答・HTTPエラーでも
  // 例外を投げず、必ず{ok, message}を返す(「〜しています...」の表示のまま止まるのを防ぐ)。
  async function postForMessage(path, body) {
    try {
      const res = await fetch(path, { method: "POST", headers: {"Content-Type":"application/json"}, body: body === undefined ? "{}" : body });
      let d = null;
      try { d = await res.json(); } catch (e2) { /* 本文がJSONでない */ }
      if (!res.ok) return { ok: false, message: "失敗: " + ((d && d.message) || `操作エラー(${res.status})`) };
      return { ok: !(d && d.ok === false), message: (d && d.message) || "" };
    } catch (e) {
      return { ok: false, message: "失敗: 通信エラー(サーバーに接続できません)" };
    }
  }

  // 【2026-10-02追加】読み取り専用(guest)アカウントでは運転コマンドを送らない
  // (送り続けても403が返るだけで、エラーバナーが出続けてしまうため)。
  let readOnlyAccount = false;
  let driveTimer = null;

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

  document.getElementById("mobileUrl").textContent = `http://${location.hostname}:8095`;

  // PC側は画面上にD-pad/WASDボタンは出さない(スマホ専用ページに移動済み)が、
  // 物理キーボードのW/A/S/Dキーには反応する(ボタンが無いだけでキー入力は使える)。
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
  const teleopTabId = (() => {
    try { const a = new Uint8Array(8); crypto.getRandomValues(a); return Array.from(a, (b) => b.toString(16).padStart(2, "0")).join(""); }
    catch (e) { return (Math.random().toString(36).slice(2, 14) || "t"); }
  })();
  let teleopSeq = 0;
  let driveInFlightSince = 0;
  let driveInFlightSeq = 0;
  // 【2026-10-03修正】seq(単調増加)+tab_id(ページ固有)を付け、サーバー側で古い前進が
  // 最後の停止を追い越すのを防ぐ。非停止の前回リクエストが未完了の間は周期送信を
  // 見送る(停止は常に送る。1秒で見送り解除=通信ハング時に操作不能のままにしない)。
  function sendDriveKeyboard() {
    if (readOnlyAccount) return;
    const stop = state.forward === 0 && state.turn === 0;
    if (!stop && driveInFlightSince && (Date.now() - driveInFlightSince) < 1000) return;
    const seq = ++teleopSeq;
    const p = postJson("/api/teleop", { forward: state.forward, turn: state.turn, seq, tab_id: teleopTabId }, { keepalive: true });
    if (stop) { driveInFlightSince = 0; return; }
    driveInFlightSince = Date.now(); driveInFlightSeq = seq;
    p.finally(() => { if (driveInFlightSeq === seq) driveInFlightSince = 0; });
  }
  // 【2026-10-02追加】入力欄(INPUT/TEXTAREA/SELECT)での文字入力(例: 「west」)で
  // 走行しないよう、これらにフォーカスがある間・修飾キー併用・キーリピートは無視する。
  function isTypingTarget(e) {
    const t = e.target;
    if (!t) return false;
    const tag = t.tagName || "";
    return tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT" || !!t.isContentEditable;
  }
  // 押しっぱなし状態を全て破棄して停止を送る。Alt+Tab・別ウィンドウのクリック・
  // confirm()ダイアログ等でkeyupを受け取れないと、pressedが残ったまま走り続けて
  // しまうため、フォーカス喪失・非表示・ダイアログ表示の前に必ず呼ぶ。
  function releaseAllKeys() {
    pressed.clear();
    recomputeFromKeys();
    sendDriveKeyboard();
  }
  window.addEventListener("keydown", (e) => {
    if (e.repeat || e.ctrlKey || e.metaKey || e.altKey) return;
    if (isTypingTarget(e)) return;
    const k = e.key.toLowerCase();
    if (!(k in keyMap)) return;
    pressed.add(k);
    recomputeFromKeys();
    sendDriveKeyboard(); // キー入力直後に即送信(最大100ms周期を待たずに反映する)
  });
  window.addEventListener("keyup", (e) => {
    const k = e.key.toLowerCase();
    if (!(k in keyMap)) return;
    pressed.delete(k);
    recomputeFromKeys();
    sendDriveKeyboard(); // キーを離した瞬間も即送信(離してから最大100ms止まらない遅延を無くす)
  });
  window.addEventListener("blur", releaseAllKeys);
  window.addEventListener("pagehide", releaseAllKeys);
  document.addEventListener("visibilitychange", () => { if (document.hidden) releaseAllKeys(); });
  // confirm()はkeyupを飲み込むことがあるため、表示前に必ず停止する。
  const rawConfirm = window.confirm.bind(window);
  window.confirm = (msg) => { releaseAllKeys(); return rawConfirm(msg); };
  // alert()もkeyupを飲み込むため、表示前に必ず停止する。
  const rawAlert = window.alert.bind(window);
  window.alert = (m) => { releaseAllKeys(); rawAlert(m); };
  driveTimer = setInterval(sendDriveKeyboard, 100);

  const estopBtn = document.getElementById("estopBtn");
  // 【2026-10-02修正】ボタン表示はサーバー側の実際の状態(/api/stateのsoft_estop_engaged)に
  // 同期させる。以前は押した瞬間に表示だけ反転しており、POSTが409(in_use)/403/通信エラーで
  // 失敗しても「停止中」と表示され、再読み込み・他端末での操作とも食い違っていた。
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
  async function setEstop(engaged) {
    if (readOnlyAccount) return;
    if (engaged) releaseAllKeys(); // 停止側へ倒す(alert/通信失敗でkeyupを失っても走り続けないように)
    if (estopPending) return; // 二重送信防止
    const prev = state.estopEngaged;
    estopPending = true;
    estopBtn.disabled = true;
    renderEstop(engaged); // 押した直後は即時に表示を変える(失敗したら下で戻す)
    let ok = false;
    try {
      ok = await postJson("/api/soft_estop", { engaged });
    } finally {
      estopPending = false;
      estopBtn.disabled = false;
      estopIgnoreSyncUntil = Date.now() + 1500; // 送信前に始まった古い/api/state応答で表示が戻るのを避ける
    }
    if (ok) { showEstopWarning(""); return; }
    renderEstop(prev);
    if (engaged) {
      showEstopWarning("緊急停止の送信に失敗しました。機体は止まっていない可能性があります。物理の非常停止を使ってください。");
      alert("緊急停止の送信に失敗しました。機体は止まっていない可能性があります。物理の非常停止を使ってください。");
    } else {
      showEstopWarning("緊急停止の解除に失敗しました(緊急停止中のままです)。");
    }
  }
  estopBtn.addEventListener("click", async () => {
    if (state.estopEngaged) {
      if (!confirm("本当に解除しますか？")) return;
      await setEstop(false);
    } else {
      await setEstop(true);
    }
  });
  // スペースキーでどこからでも緊急停止できるようにする(入力欄にフォーカスが
  // あるときは通常のスペース入力を優先し、発動させない)。解除は誤操作防止の
  // ため必ずボタン+確認ダイアログ経由のみとし、スペースキーでは解除しない。
  window.addEventListener("keydown", (e) => {
    if (e.code !== "Space") return;
    const tag = (e.target && e.target.tagName) || "";
    if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
    if (state.estopEngaged) return;
    e.preventDefault();
    setEstop(true);
  });
  document.getElementById("resumeBtn").addEventListener("click", () => postJson("/api/resume_autonomy", {}));

  // --- 地図モードの開始/停止/リセット/読み込み ---
  const mapModeLabel = document.getElementById("mapModeLabel");
  const mapLoadSelect = document.getElementById("mapLoadSelect");
  document.getElementById("mapStartBtn").addEventListener("click", async () => {
    mapSaveHint.textContent = "地図作成を開始しています...";
    const d = await postForMessage("/api/mapping/start", "{}");
    mapSaveHint.textContent = d.message;
  });
  document.getElementById("mapStopBtn").addEventListener("click", async () => {
    mapSaveHint.textContent = "停止しています...";
    const d = await postForMessage("/api/mapping/stop", "{}");
    mapSaveHint.textContent = d.message;
  });
  document.getElementById("mapResetBtn").addEventListener("click", async () => {
    if (!confirm("地図をリセットします(保存していない作成中の地図は失われます)。本当によろしいですか？")) return;
    mapSaveHint.textContent = "リセットしています...";
    const d = await postForMessage("/api/mapping/reset", "{}");
    mapSaveHint.textContent = d.message;
  });
  document.getElementById("mapLoadBtn").addEventListener("click", async () => {
    const name = mapLoadSelect.value;
    if (!name) { mapSaveHint.textContent = "読み込む地図を選択してください"; return; }
    mapSaveHint.textContent = "読み込んでいます...";
    const d = await postForMessage("/api/mapping/load", JSON.stringify({ name }));
    mapSaveHint.textContent = d.message;
  });
  async function refreshMapList() {
    try {
      const res = await fetch("/api/mapping/list");
      if (!res.ok) return;
      const d = await res.json();
      NKField.setOptions(mapLoadSelect, d.maps || []);  // 中身が同じ/操作中なら作り直さない
    } catch (e) { /* 無視 */ }
  }
  async function pollMapLifecycleStatus() {
    try {
      const res = await fetch("/api/mapping/lifecycle_status");
      if (!res.ok) return;
      const s = await res.json();
      const modeLabel = { idle: "停止中", mapping: "作成中(SLAM)", localization: "読み込み済み(localization)" }[s.mode] || s.mode;
      let label = s.loaded_map ? `${modeLabel}: ${s.loaded_map}` : modeLabel;
      // 【2026-10-02追加】プロセス稼働状態と、異常終了などのlast_errorも表示する。
      if (s.mode !== "idle") label += s.running ? "(プロセス稼働中)" : "(プロセス停止)";
      if (s.last_error) label += ` / エラー: ${s.last_error}`;
      mapModeLabel.textContent = label;
      mapModeLabel.style.color = s.last_error ? "#993c1d" : "";
    } catch (e) { /* 無視 */ }
  }
  NKField.guard(["mapSaveName", "mapLoadSelect"]);
  NKField.poll(refreshMapList, 5000);
  setInterval(pollMapLifecycleStatus, 2000);
  refreshMapList();
  pollMapLifecycleStatus();

  // --- 地図の保存(SLAMで作成中の地図をボタン1つで書き出す) ---
  const mapSaveBtn = document.getElementById("mapSaveBtn");
  const mapSaveHint = document.getElementById("mapSaveHint");
  mapSaveBtn.addEventListener("click", async () => {
    const name = document.getElementById("mapSaveName").value.trim();
    mapSaveBtn.disabled = true;
    mapSaveHint.textContent = "保存中...";
    try {
      const res = await fetch("/api/mapping/save", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name }),
      });
      const data = await res.json();
      mapSaveHint.textContent = data.message || (data.ok ? "保存を開始しました" : "保存に失敗しました");
    } catch (e) {
      mapSaveHint.textContent = "通信エラー: サーバーに接続できません";
    } finally {
      mapSaveBtn.disabled = false;
    }
  });
  let lastMapSaveTime = null;
  async function pollMappingStatus() {
    try {
      const res = await fetch("/api/mapping/status");
      if (!res.ok) return;
      const s = await res.json();
      // 【2026-10-02修正】last_result.timeが変わった時だけ表示を更新する
      // (3秒毎に上書きすると、他の操作メッセージが消えてしまうため)。
      if (s.last_result && s.last_result.time && s.last_result.time !== lastMapSaveTime) {
        lastMapSaveTime = s.last_result.time;
        const r = s.last_result;
        mapSaveHint.textContent = r.ok
          ? `保存済み: ${r.name}(${r.path}.yaml / .pgm)`
          : `保存失敗: ${r.message}`;
      }
    } catch (e) { /* 無視 */ }
  }
  setInterval(pollMappingStatus, 3000);

  // --- 地図表示とクリック運転 ---
  __MAP_VIEW_JS__

  // --- 走行可能範囲(矩形)の設定: 地図を2回クリックして対角の2点を指定する ---
  // 状態: {enabled, center_x, center_y, width, height, yaw[rad], yaw_deg, min_x, max_x, min_y, max_y}
  // (min/maxは回転矩形の軸平行外接矩形。描画・判定は回転矩形で行う)
  let currentBoundary = null;
  let boundaryPickMode = false;
  let boundaryFirstPoint = null;
  const boundaryHint = document.getElementById("boundaryHint");

  function drawBoundary() {
    drawBoundaryShape(currentBoundary);  // 回転矩形+四隅のハンドル(map_view.py)
  }

  async function fetchBoundary() {
    const tk = NKField.ticket("boundary");
    try {
      const res = await fetch("/api/boundary");
      if (!res.ok) return;
      const b = await res.json();
      if (!NKField.fresh(tk) || mapBoundaryDrag) return;  // 送信前に始まった古い結果・角のドラッグ中は反映しない
      currentBoundary = b;
      renderMap();
    } catch (e) { /* 無視 */ }
  }

  document.getElementById("boundarySetBtn").addEventListener("click", () => {
    boundaryPickMode = true;
    boundaryFirstPoint = null;
    boundaryHint.textContent = "地図上で範囲の1つ目の角をクリックしてください(表示の向きに沿った矩形になります)";
  });
  document.getElementById("boundaryClearBtn").addEventListener("click", async () => {
    boundaryPickMode = false;
    boundaryFirstPoint = null;
    const d = await postForMessage("/api/boundary/clear", "{}");
    boundaryHint.textContent = d.message;
    fetchBoundary();
  });

  canvas.addEventListener("click", (ev) => {
    if (!mapMeta || mapCssW < 2 || mapCssH < 2) return;
    // ドラッグ/ピンチ操作の直後のclickは目的地指定として扱わない
    if (Date.now() < mapClickSuppressUntil) return;
    const [px, py] = mapEventToPixel(ev);
    const [x, y] = pixelToWorld(px, py);

    if (boundaryPickMode) {
      if (!boundaryFirstPoint) {
        boundaryFirstPoint = [x, y];
        boundaryHint.textContent = "2つ目の角(対角)をクリックしてください";
      } else {
        // 表示の向きで水平・垂直に見える矩形(世界座標では表示回転ぶん回った矩形)
        const rect = boundaryRectFromViewCorners(boundaryFirstPoint, [x, y], mapView.rot || 0);
        boundaryPickMode = false;
        boundaryFirstPoint = null;
        if (rect.width < 0.2 || rect.height < 0.2) {
          boundaryHint.textContent = "範囲が小さすぎます(対角の2点を離してクリックしてください)";
          return;
        }
        (async () => {
          const d = await postForMessage("/api/boundary/set", JSON.stringify(rect));
          boundaryHint.textContent = d.message;
          fetchBoundary();
        })();
      }
      return;
    }
    postJson("/api/goal", { x, y });
  });

  setInterval(fetchMap, 2000);
  NKField.poll(fetchBoundary, 5000);
  fetchMap();
  fetchBoundary();

  // --- 自律探索マッピング(frontier_explorer)の開始/停止 ---
  const explorationHint = document.getElementById("explorationHint");
  document.getElementById("explorationStartBtn").addEventListener("click", async () => {
    const d = await postForMessage("/api/exploration/start", "{}");
    explorationHint.textContent = d.message;
  });
  document.getElementById("explorationStopBtn").addEventListener("click", async () => {
    const d = await postForMessage("/api/exploration/stop", "{}");
    explorationHint.textContent = d.message;
  });
  async function pollExplorationStatus() {
    try {
      const res = await fetch("/api/exploration/status");
      if (!res.ok) return;
      const s = await res.json();
      if (!s.status) return;
      const label = { idle: "未実行", exploring: "探索中", paused: "一時停止中", complete: "探索完了" }[s.status] || s.status;
      explorationHint.textContent = `状態: ${label}` + (s.reason ? `(${s.reason})` : "");
    } catch (e) { /* 無視 */ }
  }
  setInterval(pollExplorationStatus, 2000);
  pollExplorationStatus();

  // --- LiDARの見え方(自機中心のレーダー風表示) ---
  const lidarCanvas = document.getElementById("lidarCanvas");
  const lidarCtx = lidarCanvas.getContext("2d");

  function drawScan(s) {
    lidarCtx.clearRect(0, 0, lidarCanvas.width, lidarCanvas.height);
    const cx = lidarCanvas.width / 2, cy = lidarCanvas.height / 2;
    const maxRange = Math.min(s.range_max, 6.0);
    const scale = Math.min(cx, cy) / maxRange;

    lidarCtx.strokeStyle = "#234339";
    lidarCtx.lineWidth = 1;
    for (let r = 1; r <= maxRange; r += 1) {
      lidarCtx.beginPath();
      lidarCtx.arc(cx, cy, r * scale, 0, Math.PI * 2);
      lidarCtx.stroke();
    }

    lidarCtx.fillStyle = "#ff5a5a";
    let angle = s.angle_min;
    for (const r of s.ranges) {
      if (r !== null && r <= maxRange) {
        const px = cx - r * Math.sin(angle) * scale;
        const py = cy - r * Math.cos(angle) * scale;
        lidarCtx.fillRect(px - 1.5, py - 1.5, 3, 3);
      }
      angle += s.angle_increment;
    }

    // 自機(中心、前方=画面上向き)
    lidarCtx.fillStyle = "#3fd68c";
    lidarCtx.beginPath();
    lidarCtx.moveTo(cx, cy - 9);
    lidarCtx.lineTo(cx - 6, cy + 6);
    lidarCtx.lineTo(cx + 6, cy + 6);
    lidarCtx.closePath();
    lidarCtx.fill();
  }

  async function fetchScan() {
    try {
      const res = await fetch("/api/scan");
      if (!res.ok) return;
      drawScan(await res.json());
    } catch (e) { /* 無視、次のポーリングで再試行 */ }
  }
  setInterval(fetchScan, 500);
  fetchScan();

  // --- 混雑判定の背景キャリブレーション(壁・柱等の静的な近さを誤検出しないように) ---
  document.getElementById("calibrateBgBtn").addEventListener("click", async () => {
    if (!confirm("周囲に人がいない状態にしてから実行してください。今キャリブレーションしますか？")) return;
    const btn = document.getElementById("calibrateBgBtn");
    const hint = document.getElementById("calibrateBgHint");
    // 【2026-10-13変更】要求にrequest_idが付き、crowd_monitorが返す「受理(started)→完了(done)/失敗(failed)」を
    // 読み戻して表示する。以前はcrowd_monitorが止まっていても成功と表示していた。
    const tx = NKField.begin("crowdcal", [btn]);
    hint.textContent = "要求を送信中…";
    let d = null;
    try {
      const res = await fetch("/api/crowd_background/calibrate", { method: "POST", headers: {"Content-Type":"application/json"}, body: "{}" });
      d = await res.json().catch(() => null);
      if (!res.ok) d = { ok: false, message: "失敗: " + ((d && d.message) || `操作エラー(${res.status})`) };
    } catch (e) { d = { ok: false, message: "失敗: 通信エラー(サーバーに接続できません)" }; }
    if (!d || !d.ok || !d.request_id) { tx.fail(); hint.textContent = (d && d.message) || "失敗: 要求を送れませんでした"; return; }
    hint.textContent = "要求を送りました。crowd_monitorの応答を待っています…";
    const rid = d.request_id;
    let started = false, fin = null;
    for (let i = 0; i < 20 && !fin; i++) {   // 最大約20秒(受理は数秒以内、完了は次のLiDARスキャン)
      await new Promise((r) => setTimeout(r, 1000));
      let s = null;
      try { s = await (await fetch("/api/crowd_background/state")).json(); } catch (e) { continue; }
      if (!s || s.request_id !== rid) continue;
      if (s.state === "started") { started = true; hint.textContent = "crowd_monitorが受け付けました。次のLiDARスキャンを待っています…"; }
      else if (s.state === "done" || s.state === "failed") fin = s;
    }
    tx.done();
    if (fin && fin.state === "done") hint.textContent = "混雑判定の背景を記録しました(確認済み。LiDARの有効点 " + fin.background_frames + ")";
    else if (fin) hint.textContent = "背景の記録に失敗しました: " + (fin.error || "原因不明");
    else if (started) hint.textContent = "crowd_monitorは受け付けましたが、LiDARスキャンが来ないため完了を確認できません(LiDARは動いていますか?)";
    else hint.textContent = "crowd_monitorが応答しません(起動していますか?)。届いたか確認できません";
  });

  // --- 正面カメラの見え方(静止画ポーリング。滑らかな動画ではない) ---
  const cameraView = document.getElementById("cameraView");
  function fetchCamera() {
    // キャッシュ回避のため毎回クエリを変える。fetch+blobではなくimg.srcの
    // 直接更新にすることで、読み込み失敗時も前の画像が消えずに残る。
    clearTimeout(cameraTimer);
    cameraTimer = setTimeout(fetchCamera, 10000); // 応答が来ない場合の保険
    cameraView.src = "/api/camera?t=" + Date.now();
  }
  // 【2026-10-02修正】前の画像の読み込みが終わる前にsrcを差し替えると、遅い回線では
  // いつまでも1枚も表示完了しない(飢餓)。onload/onerrorを待ってから次を予約する。
  let cameraTimer = null;
  function scheduleCamera() { clearTimeout(cameraTimer); cameraTimer = setTimeout(fetchCamera, 400); }
  cameraView.addEventListener("load", scheduleCamera);
  cameraView.addEventListener("error", scheduleCamera);
  fetchCamera();

  // 安全装置(estop・立入禁止エリア等)で自律動作が止まっている時、何が原因かを
  // ヘッダーのバッジに分かりやすく表示する(2026-07-17、ユーザー要望:
  // 「なぜ動かないか」をAPIを叩かなくても画面で分かるようにしたい)。
  const HOLD_REASONS = {
    NEAR_PERSON_HOLD: "人物接近(50cm以内)で自律停止中",
    OUT_OF_BOUNDS_HOLD: "走行可能範囲外のため自律停止中",
    KEEPOUT_ZONE_HOLD: "立入禁止エリアのため自律停止中",
    CAMERA_OBSTACLE_HOLD: "カメラが障害物を検知、自律停止中",
    MAP_NOT_LOADED_HOLD: "地図が読み込まれていません。地図を読み込んでください(自律動作は停止中)",
    // 【2026-10-02追加】これらも自律が止まっている状態なのにバッジが「正常」になっていた。
    MAPPING_ACTIVE_HOLD: "地図作成/探索中のため自律停止中",
    SURVEY_ACTIVE_HOLD: "アンケート回答中のため停止中",
    WAIT_FOR_SENSOR: "センサー待ちのため自律停止中",
  };
  // 【2026-10-06追加】時間帯・状況ルール(方針)による待機/移動。安全装置のホールドではないので
  // 赤ではなく黄色のバッジで表示する(behavior_rules、docs/behavior_rules.md)。
  const POLICY_REASONS = {
    // 【2026-11-05】「向く」機能の通常動作(安全装置のホールドではない)。赤ではなく黄色で表示する。
    FACE_PERSON: "人の方を向いています(その場で旋回)",
    FACE_SWEEP: "人を探して見回しています(その場で旋回)",
    POLICY_STAY: "方針により待機中(その場で待機)",
    POLICY_GO_ENTRANCE: "方針により入口へ移動中",
    POLICY_AT_ENTRANCE: "方針により入口で待機中",
    POLICY_NO_SCAN: "方針により見回し停止中(待機)",
  };
  // behavior_directorからの指令が数秒来ていない(落ちている/通信断)場合は、
  // 古いmode/estop_activeを信用せず「無応答」と表示する。
  function isBehaviorStale(s) {
    return s.behavior_age_s === null || s.behavior_age_s === undefined || s.behavior_age_s > 5;
  }
  function updateStatusBadge(s) {
    const badge = document.getElementById("estopBadge");
    if (isBehaviorStale(s)) {
      badge.textContent = "behavior_director無応答";
      badge.className = "estop-badge estop-bad";
    } else if (s.estop_active) {
      badge.textContent = "緊急停止中";
      badge.className = "estop-badge estop-bad";
    } else if (HOLD_REASONS[s.mode]) {
      badge.textContent = HOLD_REASONS[s.mode];
      badge.className = "estop-badge estop-bad";
    } else if (POLICY_REASONS[s.mode]) {
      badge.textContent = POLICY_REASONS[s.mode];
      badge.className = "estop-badge estop-policy";
    } else {
      badge.textContent = "正常";
      badge.className = "estop-badge estop-ok";
    }
  }

  async function poll() {
    try {
      const res = await fetch("/api/state");
      const s = await res.json();
      syncEstop(s);
      document.getElementById("crowdLevel").textContent = s.crowd_level || "-";
      document.getElementById("mode").textContent = s.mode || "-";
      document.getElementById("audioBusy").textContent = s.audio_busy ? "はい" : "いいえ";
      document.getElementById("manualOverride").textContent = s.manual_override ? "はい" : "いいえ";
      document.getElementById("odomXY").textContent = s.odom ? `${s.odom.x.toFixed(2)}, ${s.odom.y.toFixed(2)}` : "-";
      document.getElementById("announce").textContent = s.announce_text || "";
      updateStatusBadge(s);
      if (s.robot_pose) { robotPose = s.robot_pose; drawRobot(); }
      routeTrail = s.trail || [];
      plannedPath = s.planned_path || [];
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
      NKField.set("autoLinear", s.auto_linear_mps);
      NKField.set("autoAngular", s.auto_angular_rps);
      return s;
    } catch (e) { /* 未取得ならinput空のまま */ return null; }
  }
  const SPEED_KEY_ID = { manual_linear_mps: "manualLinear", manual_angular_rps: "manualAngular", auto_linear_mps: "autoLinear", auto_angular_rps: "autoAngular" };
  NKField.guard(Object.values(SPEED_KEY_ID), { clamp: true });  // 範囲はmin/max属性。範囲外は入力時に表示
  async function speedRead() { try { return await (await fetch("/api/speed_settings")).json(); } catch (e) { return null; } }
  NKField.poll(loadSpeedSettings, 15000);  // 他の端末で変えた速度も反映する(編集中の欄は上書きしない)
  document.getElementById("speedSaveBtn").addEventListener("click", async () => {
    // 【2026-10-02修正】空欄/数値でない欄はリクエストに含めない(以前はNumber("")=0を送り、
    // 最小値0.01m/sに丸めて保存されていた)。
    const req = {};
    for (const [key, id] of [["manual_linear_mps", "manualLinear"], ["manual_angular_rps", "manualAngular"], ["auto_linear_mps", "autoLinear"], ["auto_angular_rps", "autoAngular"]]) {
      const raw = document.getElementById(id).value.trim();
      if (raw === "" || !NKField.isDirty(id)) continue;  // 触った欄だけ送る(触っていない欄は、他の端末で変えられていても上書きしない)
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

  // --- guest(閲覧専用)アカウント対応: 操作系UIを無効化して見た目上も
  // 「モックアップ・見るだけ」と分かるようにする ---
  async function checkAccount() {
    try {
      const res = await fetch("/api/whoami");
      const w = await res.json();
      document.getElementById("accountLabel").textContent = w.account ? `(${w.account})` : "";
      if (w.account === "guest") {
        // 閲覧専用: 運転コマンドの定期送信を止める(403が返り続けるため)。
        readOnlyAccount = true;
        if (driveTimer !== null) { clearInterval(driveTimer); driveTimer = null; }
        document.querySelectorAll("button, input").forEach((el) => { el.disabled = true; });
        const banner = document.createElement("div");
        banner.textContent = "閲覧専用(guestアカウント): ここでは操作できません";
        banner.style.cssText = "background:#fff6df;color:#7a5c00;padding:6px 10px;font-size:12px;text-align:center;";
        document.body.insertBefore(banner, document.body.firstChild);
      }
    } catch (e) { /* 取得失敗時は通常通り(操作可能のまま)にしておく */ }
  }

  // ログアウト: サーバー側セッションを破棄し、専用ログインページへ移動する。
  document.getElementById("logoutBtn").addEventListener("click", async () => {
    try {
      await fetch("/api/logout", { method: "POST" });
    } catch (e) { /* 失敗してもログイン画面へは移動する */ }
    location.href = "/login";
  });
  checkAccount();
</script>
</body>
</html>
""".replace("__NOKOSAHEN_LOGO__", LOGO_DATA_URI).replace("__MAP_VIEW_JS__", MAP_VIEW_JS).replace("__NK_FIELD_JS__", NK_FIELD_JS)

# スマホ(6.2インチ想定)から機体の移動制御だけを行うための最小限のページ。
# カメラ・LiDAR・地図・残数・決済カメラ等はここには置かない
# (それらはPC版の操作盤ダッシュボードを参照する)。ログインはPC版と共通。
MOBILE_HTML = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no" />
<title>のこさへん - スマホ操作</title>
<style>
  :root { color-scheme: light; font-family: system-ui, -apple-system, "Segoe UI", sans-serif; background:#f4f6f5; color:#16211d; }
  html, body { margin:0; }
  header { padding:10px 14px; background:#124739; color:#fff; display:flex; justify-content:space-between; align-items:center; }
  .logo-title { display:flex; align-items:center; gap:8px; }
  .logo-title img { height:28px; width:28px; border-radius:6px; }
  h1 { margin:0; font-size:16px; }
  main { padding:12px; display:flex; flex-direction:column; gap:12px; max-width:480px; margin:0 auto; }
  section { background:#fff; border:1px solid #dce4df; border-radius:10px; padding:14px; }
  h2 { margin:0 0 10px; font-size:14px; }
  .estop-badge { padding:4px 12px; border-radius:999px; font-weight:700; font-size:12px; }
  .estop-ok { background:#e1f5ee; color:#0f6e56; }
  .estop-bad { background:#faece7; color:#993c1d; }
  .estop-policy { background:#fff3cd; color:#7a5b00; }
  .header-right { display:flex; align-items:center; gap:8px; }
  #accountLabel { font-size:11px; opacity:0.85; }
  .logout-btn { padding:4px 8px; font-size:11px; border:1px solid rgba(255,255,255,0.5); border-radius:6px; background:transparent; color:#fff; cursor:pointer; }
  .dpad { display:grid; grid-template-columns:repeat(3, 84px); grid-template-rows:repeat(3, 84px); gap:8px; justify-content:center; margin:0 auto; }
  .dpad button { font-size:26px; border:0; border-radius:12px; background:#176b56; color:#fff; touch-action:none; user-select:none; -webkit-user-select:none; -webkit-touch-callout:none; }
  .dpad button:active, .dpad button.pressed { background:#0c4a3a; }
  .stop-btn { width:100%; padding:16px; font-size:17px; font-weight:700; border:0; border-radius:10px; background:#a32d2d; color:#fff; margin-top:14px; }
  .stop-btn.engaged { background:#5c1717; }
  .row { display:flex; gap:10px; flex-wrap:wrap; margin-bottom:8px; }
  .row label { flex:1 1 45%; font-size:12px; display:flex; flex-direction:column; gap:4px; }
  .row input { font-size:16px; padding:6px; border:1px solid #cfd8d3; border-radius:6px; }
  .save-btn { width:100%; padding:12px; font-size:15px; font-weight:700; border:0; border-radius:10px; background:#176b56; color:#fff; }
  .hint { font-size:11px; color:#6b6a64; margin-top:8px; }
</style>
</head>
<body>
<header>
  <div class="logo-title">
    <img src="__NOKOSAHEN_LOGO__" alt="のこさへん" />
    <h1>のこさへん: スマホ操作</h1>
  </div>
  <div class="header-right">
    <span id="estopBadge" class="estop-badge estop-ok">--</span>
    <span id="accountLabel"></span>
    <button id="logoutBtn" class="logout-btn">ログアウト</button>
  </div>
</header>
<main>
  <section>
    <h2>手動運転</h2>
    <div class="dpad">
      <span></span><button data-fwd="1" data-turn="0">▲</button><span></span>
      <button data-fwd="0" data-turn="1">◀</button>
      <button data-fwd="0" data-turn="0">■</button>
      <button data-fwd="0" data-turn="-1">▶</button>
      <span></span><button data-fwd="-1" data-turn="0">▼</button><span></span>
    </div>
    <p class="hint">ボタンを押している間だけ動きます。操作している間は自律行動が一時停止します。</p>
    <button id="estopBtn" class="stop-btn">緊急停止(ソフト)</button>
  </section>

  <section id="speedSection">
    <h2>最高速度設定</h2>
    <div class="row">
      <label>手動直進(m/s) <input id="manualLinear" type="number" step="0.01" min="0.01" max="1.1"></label>
      <label>手動旋回(rad/s) <input id="manualAngular" type="number" step="0.01" min="0.01" max="3"></label>
    </div>
    <div class="row">
      <label>自動移動直進(m/s) <input id="autoLinear" type="number" step="0.01" min="0.01" max="1.1"></label>
      <label>自動移動旋回(rad/s) <input id="autoAngular" type="number" step="0.01" min="0.01" max="3"></label>
    </div>
    <button id="speedSaveBtn" class="save-btn">保存</button>
    <p class="hint">直進速度の上限は1.1m/s(ファーム側の速度制限)。これを超える値は1.1m/sに丸められます。</p>
    <p class="hint" id="speedSaveHint">&nbsp;</p>
  </section>

  <section>
    <h2>待機中の左右振動</h2>
    <p class="hint">人がまだ見つかっていない間、センサーを使わず左右へ1秒ずつ交互に回転するだけの動きに切り替えます。</p>
    <div class="row">
      <button id="wiggleOnBtn" class="save-btn">ON</button>
      <button id="wiggleOffBtn" class="save-btn" style="background:#6b6a64;">OFF</button>
    </div>
    <p class="hint" id="wiggleHint">&nbsp;</p>
  </section>
</main>
<script>
__NK_FIELD_JS__
</script>
<script>
  // 401(未ログイン/セッション切れ)を受け取ったら自動でログイン画面へ誘導する。
  (function () {
    const rawFetch = window.fetch.bind(window);
    window.fetch = async (...args) => {
      const res = await rawFetch(...args);
      if (res.status === 401 && location.pathname !== "/login") {
        location.href = "/login";
      }
      return res;
    };
  })();

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

  // 【2026-10-02追加】読み取り専用(guest)アカウントでは運転コマンドを送らない
  // (送り続けても403が返るだけで、エラーバナーが出続けてしまうため)。
  let readOnlyAccount = false;
  let driveTimer = null;

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

  // 【2026-07-17追加】restricted(restricted_account)アカウントの運転コマンドがTL優先で
  // 破棄された場合、ここだけ専用のバナーで「TLが操作中です」と知らせる
  // (通常のエラーバナーとは別枠、TLの操作が終われば自動的に消える)。
  function showTlActiveBanner(active) {
    let banner = document.getElementById("tlActiveBanner");
    if (active) {
      if (!banner) {
        banner = document.createElement("div");
        banner.id = "tlActiveBanner";
        banner.style.cssText = "background:#fff6df;color:#7a5c00;padding:6px 10px;font-size:12px;text-align:center;";
        document.body.insertBefore(banner, document.body.firstChild);
      }
      banner.textContent = "TLが操作中です";
      banner.style.display = "block";
    } else if (banner) {
      banner.style.display = "none";
    }
  }

  const teleopTabId = (() => {
    try { const a = new Uint8Array(8); crypto.getRandomValues(a); return Array.from(a, (b) => b.toString(16).padStart(2, "0")).join(""); }
    catch (e) { return (Math.random().toString(36).slice(2, 14) || "t"); }
  })();
  let teleopSeq = 0;
  let driveInFlightSince = 0;
  let driveInFlightSeq = 0;
  async function sendDrive() {
    if (readOnlyAccount) return;
    // 【2026-10-03修正】seq/tab_idを付与し、非停止の前回リクエストが未完了なら周期送信を見送る(停止は常に送る)。
    const stop = state.forward === 0 && state.turn === 0;
    if (!stop && driveInFlightSince && (Date.now() - driveInFlightSince) < 1000) return;
    const seq = ++teleopSeq;
    if (stop) { driveInFlightSince = 0; } else { driveInFlightSince = Date.now(); driveInFlightSeq = seq; }
    try {
      const res = await fetch("/api/teleop", {
        method: "POST", headers: {"Content-Type":"application/json"},
        body: JSON.stringify({ forward: state.forward, turn: state.turn, seq, tab_id: teleopTabId }),
        keepalive: true, // pagehide時の停止送信がページ破棄で中断されないようにする
      });
      if (!res.ok) {
        showError(`操作エラー(${res.status})`);
        return;
      }
      const data = await res.json();
      showTlActiveBanner(data.reason === "tl_active");
      if (data.reason === "estop_engaged") {
        showError(data.message || "緊急停止中です。解除してから操作してください。");
      } else if (data.reason !== "tl_active") {
        showError("");
      }
    } catch (e) {
      showError("通信エラー: サーバーに接続できません");
    } finally {
      if (!stop && driveInFlightSeq === seq) driveInFlightSince = 0;
    }
  }

  // 【2026-10-02修正】Pointer Events+setPointerCaptureに統一し、タッチのキャンセル
  // (通知の割り込み・システムジェスチャ等でtouchendが来ない)・画面ロック・アプリ切替・
  // タブ非表示でも必ず停止を送る。以前はtouchend/mouseupしか見ておらず、キャンセルされると
  // state.forward=1のまま100ms毎に前進を送り続けた。
  let activePointerId = null;
  function stopDrive() {
    activePointerId = null;
    state.forward = 0; state.turn = 0;
    document.querySelectorAll(".dpad button.pressed").forEach((b) => b.classList.remove("pressed"));
    sendDrive();
  }
  document.querySelectorAll(".dpad button").forEach((btn) => {
    const fwd = Number(btn.dataset.fwd), turn = Number(btn.dataset.turn);
    btn.addEventListener("pointerdown", (ev) => {
      ev.preventDefault();
      if (readOnlyAccount) return;
      // 既に別の指で操作中に2本目が触れた場合は、安全側として一旦停止する。
      if (activePointerId !== null && activePointerId !== ev.pointerId) { stopDrive(); return; }
      activePointerId = ev.pointerId;
      try { btn.setPointerCapture(ev.pointerId); } catch (e) { /* 取れない場合はpointerleaveで停止する */ }
      state.forward = fwd; state.turn = turn; btn.classList.add("pressed"); sendDrive();
    });
    btn.addEventListener("pointerup", stopDrive);
    btn.addEventListener("pointercancel", stopDrive);
    btn.addEventListener("lostpointercapture", stopDrive);
    btn.addEventListener("pointerleave", (ev) => {
      let captured = false;
      try { captured = btn.hasPointerCapture(ev.pointerId); } catch (e) { captured = false; }
      if (!captured && activePointerId === ev.pointerId) stopDrive();
    });
    btn.addEventListener("touchcancel", stopDrive);
    btn.addEventListener("contextmenu", (ev) => ev.preventDefault());
  });
  window.addEventListener("blur", stopDrive);
  window.addEventListener("pagehide", stopDrive);
  document.addEventListener("visibilitychange", () => { if (document.hidden) stopDrive(); });
  driveTimer = setInterval(sendDrive, 100);
  // alert()もタッチ/キー入力のキャンセルを飲み込むため、表示前に必ず停止する。
  const rawAlert = window.alert.bind(window);
  window.alert = (m) => { stopDrive(); rawAlert(m); };

  const estopBtn = document.getElementById("estopBtn");
  // 【2026-10-02修正】ボタン表示はサーバー側の実際の状態(/api/stateのsoft_estop_engaged)に
  // 同期させる。以前は押した瞬間に表示だけ反転しており、POSTが409(in_use)/403/通信エラーで
  // 失敗しても「停止中」と表示され、再読み込み・他端末での操作とも食い違っていた。
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
  async function setEstop(engaged) {
    if (readOnlyAccount) return;
    if (engaged) stopDrive(); // 停止側へ倒す(alert/通信失敗でkeyupを失っても走り続けないように)
    if (estopPending) return; // 二重送信防止
    const prev = state.estopEngaged;
    estopPending = true;
    estopBtn.disabled = true;
    renderEstop(engaged); // 押した直後は即時に表示を変える(失敗したら下で戻す)
    let ok = false;
    try {
      ok = await postJson("/api/soft_estop", { engaged });
    } finally {
      estopPending = false;
      estopBtn.disabled = false;
      estopIgnoreSyncUntil = Date.now() + 1500; // 送信前に始まった古い/api/state応答で表示が戻るのを避ける
    }
    if (ok) { showEstopWarning(""); return; }
    renderEstop(prev);
    if (engaged) {
      showEstopWarning("緊急停止の送信に失敗しました。機体は止まっていない可能性があります。物理の非常停止を使ってください。");
      alert("緊急停止の送信に失敗しました。機体は止まっていない可能性があります。物理の非常停止を使ってください。");
    } else {
      showEstopWarning("緊急停止の解除に失敗しました(緊急停止中のままです)。");
    }
  }
  estopBtn.addEventListener("click", async () => {
    if (state.estopEngaged) {
      if (!confirm("本当に解除しますか？")) return;
      await setEstop(false);
    } else {
      await setEstop(true);
    }
  });
  // スマホでも物理/画面キーボードのスペースキーで発動できるようにしておく
  // (発動のみ。解除は誤操作防止のためボタン+確認ダイアログ経由のみ)。
  window.addEventListener("keydown", (e) => {
    if (e.code !== "Space") return;
    const tag = (e.target && e.target.tagName) || "";
    if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
    if (state.estopEngaged) return;
    e.preventDefault();
    setEstop(true);
  });

  const HOLD_REASONS = {
    NEAR_PERSON_HOLD: "人物接近(50cm以内)で自律停止中",
    OUT_OF_BOUNDS_HOLD: "走行可能範囲外のため自律停止中",
    KEEPOUT_ZONE_HOLD: "立入禁止エリアのため自律停止中",
    CAMERA_OBSTACLE_HOLD: "カメラが障害物を検知、自律停止中",
    MAP_NOT_LOADED_HOLD: "地図が読み込まれていません。地図を読み込んでください(自律動作は停止中)",
    // 【2026-10-02追加】これらも自律が止まっている状態なのにバッジが「正常」になっていた。
    MAPPING_ACTIVE_HOLD: "地図作成/探索中のため自律停止中",
    SURVEY_ACTIVE_HOLD: "アンケート回答中のため停止中",
    WAIT_FOR_SENSOR: "センサー待ちのため自律停止中",
  };
  // 【2026-10-06追加】時間帯・状況ルール(方針)による待機/移動。安全装置のホールドではないので
  // 赤ではなく黄色のバッジで表示する(behavior_rules、docs/behavior_rules.md)。
  const POLICY_REASONS = {
    // 【2026-11-05】「向く」機能の通常動作(安全装置のホールドではない)。赤ではなく黄色で表示する。
    FACE_PERSON: "人の方を向いています(その場で旋回)",
    FACE_SWEEP: "人を探して見回しています(その場で旋回)",
    POLICY_STAY: "方針により待機中(その場で待機)",
    POLICY_GO_ENTRANCE: "方針により入口へ移動中",
    POLICY_AT_ENTRANCE: "方針により入口で待機中",
    POLICY_NO_SCAN: "方針により見回し停止中(待機)",
  };
  // behavior_directorからの指令が数秒来ていない(落ちている/通信断)場合は、
  // 古いmode/estop_activeを信用せず「無応答」と表示する。
  function isBehaviorStale(s) {
    return s.behavior_age_s === null || s.behavior_age_s === undefined || s.behavior_age_s > 5;
  }
  async function poll() {
    try {
      const res = await fetch("/api/state");
      const s = await res.json();
      syncEstop(s);
      const badge = document.getElementById("estopBadge");
      if (isBehaviorStale(s)) {
        badge.textContent = "behavior_director無応答";
        badge.className = "estop-badge estop-bad";
      } else if (s.estop_active) {
        badge.textContent = "緊急停止中";
        badge.className = "estop-badge estop-bad";
      } else if (HOLD_REASONS[s.mode]) {
        badge.textContent = HOLD_REASONS[s.mode];
        badge.className = "estop-badge estop-bad";
      } else if (POLICY_REASONS[s.mode]) {
        badge.textContent = POLICY_REASONS[s.mode];
        badge.className = "estop-badge estop-policy";
      } else {
        badge.textContent = "正常";
        badge.className = "estop-badge estop-ok";
      }
      const wiggleHint = document.getElementById("wiggleHint");
      if (wiggleHint) {
        wiggleHint.textContent = `現在: ${s.idle_wiggle_enabled ? "ON" : "OFF"}`;
      }
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
      NKField.set("autoLinear", s.auto_linear_mps);
      NKField.set("autoAngular", s.auto_angular_rps);
      return s;
    } catch (e) { /* 未取得ならinput空のまま */ return null; }
  }
  const SPEED_KEY_ID = { manual_linear_mps: "manualLinear", manual_angular_rps: "manualAngular", auto_linear_mps: "autoLinear", auto_angular_rps: "autoAngular" };
  NKField.guard(Object.values(SPEED_KEY_ID), { clamp: true });  // 範囲はmin/max属性。範囲外は入力時に表示
  async function speedRead() { try { return await (await fetch("/api/speed_settings")).json(); } catch (e) { return null; } }
  NKField.poll(loadSpeedSettings, 15000);  // 他の端末で変えた速度も反映する(編集中の欄は上書きしない)
  document.getElementById("speedSaveBtn").addEventListener("click", async () => {
    // 【2026-10-02修正】空欄/数値でない欄はリクエストに含めない(以前はNumber("")=0を送り、
    // 最小値0.01m/sに丸めて保存されていた)。
    const req = {};
    for (const [key, id] of [["manual_linear_mps", "manualLinear"], ["manual_angular_rps", "manualAngular"], ["auto_linear_mps", "autoLinear"], ["auto_angular_rps", "autoAngular"]]) {
      const raw = document.getElementById(id).value.trim();
      if (raw === "" || !NKField.isDirty(id)) continue;  // 触った欄だけ送る(触っていない欄は、他の端末で変えられていても上書きしない)
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

  document.getElementById("wiggleOnBtn").addEventListener("click", () => postJson("/api/idle_wiggle", { enabled: true }));
  document.getElementById("wiggleOffBtn").addEventListener("click", () => postJson("/api/idle_wiggle", { enabled: false }));

  // --- guest(閲覧専用)アカウント対応: 操作系UIを無効化して見た目上も
  // 「モックアップ・見るだけ」と分かるようにする ---
  // --- restricted(機体操作+緊急停止のみ)アカウント対応: 最高速度設定の
  // セクションだけを無効化する(2026-07-17追加)。 ---
  async function checkAccount() {
    try {
      const res = await fetch("/api/whoami");
      const w = await res.json();
      document.getElementById("accountLabel").textContent = w.account ? `(${w.account})` : "";
      if (w.account === "guest") {
        // 閲覧専用: 運転コマンドの定期送信を止める(403が返り続けるため)。
        readOnlyAccount = true;
        if (driveTimer !== null) { clearInterval(driveTimer); driveTimer = null; }
        document.querySelectorAll("button, input").forEach((el) => { el.disabled = true; });
        const banner = document.createElement("div");
        banner.textContent = "閲覧専用(guestアカウント): ここでは操作できません";
        banner.style.cssText = "background:#fff6df;color:#7a5c00;padding:6px 10px;font-size:12px;text-align:center;";
        document.body.insertBefore(banner, document.body.firstChild);
      } else if (w.restricted) {
        // 消す(非表示)。disabledで残すだけだと項目が見えたままなので、
        // 2026-07-17のユーザー指示で完全に隠す形に変更した。
        const section = document.getElementById("speedSection");
        if (section) {
          section.style.display = "none";
        }
        const banner = document.createElement("div");
        banner.textContent = "このアカウントは機体操作・緊急停止のみ利用できます(速度設定は変更できません)";
        banner.style.cssText = "background:#fff6df;color:#7a5c00;padding:6px 10px;font-size:12px;text-align:center;";
        document.body.insertBefore(banner, document.body.firstChild);
      }
    } catch (e) { /* 取得失敗時は通常通り(操作可能のまま)にしておく */ }
  }
  checkAccount();

  // ログアウト: サーバー側セッションを破棄し、専用ログインページへ移動する。
  document.getElementById("logoutBtn").addEventListener("click", async () => {
    try {
      await fetch("/api/logout", { method: "POST" });
    } catch (e) { /* 失敗してもログイン画面へは移動する */ }
    location.href = "/login";
  });
</script>
</body>
</html>
""".replace("__NOKOSAHEN_LOGO__", LOGO_DATA_URI).replace("__NK_FIELD_JS__", NK_FIELD_JS)


class Stage2DashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("stage2_dashboard")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8090)
        # スマホ操作用ページ(移動制御のみ)のポート。ログインはPC版と共通。
        self.declare_parameter("mobile_port", 8095)
        self.declare_parameter("linear_speed_mps", 0.2)
        self.declare_parameter("angular_speed_rps", 0.8)
        # SLAMで作成中の地図を「地図を保存」ボタンで書き出す先。マシンローカルの
        # 保存先なので、他の~/.nokosahen_*設定と同じ考え方でホーム配下を既定にする。
        self.declare_parameter("maps_dir", "~/.nokosahen_maps")
        # 【2026-08-06追加】Jetson 2台分散構成向け。full_dashboard_node.pyと同じ
        # (詳細はそちらのコメント参照)。空文字なら従来通りローカル起動。
        self.declare_parameter("mapping_control_remote_host", "")
        # 操作盤(管理者用)のBasic認証。両方とも空にすれば従来通り認証無しになる。
        self.declare_parameter("auth_username", "TL")
        self.declare_parameter("auth_password", "")
        # 【2026-07-17追加】スマホ版限定の、機体操作+緊急停止だけできる
        # 制限付きアカウント(最高速度設定は変更不可)。空文字なら無効。
        self.declare_parameter("restricted_username", "")
        self.declare_parameter("restricted_password", "")
        # voice_bridge.py(Jetsonのホスト側、別プロセス)が書き込む発話履歴
        # SQLiteを読み取り専用で開く(2026-07-22追加、/utterance_logが使う)。
        # voice_bridge.py側の環境変数VOICE_UTTERANCE_LOG_DBと同じパスにすること。
        self.declare_parameter("utterance_log_db", "~/.nokosahen_utterance_log.db")
        # voice_bridgeが別マシンで動く場合の取得先URL(2026-10-02追加)。
        # 空ならutterance_log_dbのローカルファイルを読む。
        self.declare_parameter("utterance_log_url", "")
        # person_target_detector(同一プロセス内)が書き込む人物検知ログを
        # 読み取り専用で開く(2026-07-24追加、/detection_logが使う)。
        self.declare_parameter("detection_db_file", "~/.nokosahen_detection_log.db")
        # scripts/step_response_test/step_response_test.pyが結果を書き込む
        # ディレクトリを読み取り専用で見に行く(2026-09-24追加、/step_response
        # が使う)。CLI側の既定出力先(--out)と同じパスにすること。
        self.declare_parameter("step_response_results_dir", "~/.nokosahen_step_response_tests")

        self._crowd_state: Dict[str, Any] = {}
        self._behavior_command: Dict[str, Any] = {}
        # behavior_directorの指令を最後に受信した時刻(time.monotonic())。
        # 無応答(落ちている/通信断)をUIバッジに出すために使う(2026-10-02追加)。
        self._behavior_command_rx = 0.0
        self._announce_text = ""
        self._audio_busy = False
        self._odom: Dict[str, float] = {}

        self.create_subscription(String, "/nokosahen/crowd_state", self._on_crowd_state, 10)
        self.create_subscription(String, "/nokosahen/behavior_command", self._on_behavior_command, 10)
        self.create_subscription(String, "/nokosahen/announce_text", self._on_announce_text, 10)
        self.create_subscription(Bool, "/audio/busy", self._on_audio_busy, 10)
        self.create_subscription(Odometry, DASHBOARD_ODOM_TOPIC, self._on_odom, 10)

        self.teleop = TeleopController(
            self,
            linear_speed=float(self.get_parameter("linear_speed_mps").value),
            angular_speed=float(self.get_parameter("angular_speed_rps").value),
        )
        self.map_goal = MapGoalController(self)
        self.robot_pose = RobotPoseController(self)
        self.route_trail = RouteTrailController(self, self.robot_pose)
        self.scan = ScanController(self)
        self.camera = CameraPreviewController(self)
        self.auto_speed = AutoSpeedController(self)
        self.mapping = MappingController(
            self, str(self.get_parameter("maps_dir").value), robot_pose=self.robot_pose
        )
        self.map_lifecycle = MapLifecycleController(
            self,
            str(self.get_parameter("maps_dir").value),
            on_stopped=self._on_map_lifecycle_stopped,
            on_started=self._on_map_lifecycle_started,
            remote_ssh_host=str(self.get_parameter("mapping_control_remote_host").value),
        )
        self.boundary = BoundaryController(self, "~/.nokosahen_travel_boundary.json")
        self.keepout_zones = KeepOutZoneController(self, "~/.nokosahen_keepout_zones.json")
        self.crowd_background = CrowdBackgroundController(self)
        self.frontier_explorer = FrontierExplorerController(self)
        self.behavior_control = BehaviorDirectorControlController(self)
        # 【2026-09-24追加】走行距離のステップ応答テスト用。PidGainsController
        # 自体はStage1(ESP32単体試験)向けに作られたが、run_drive_test()は
        # Stage2でも安全に使える(manual_override経由でNav2/behavior_directorと
        # のcmd_vel競合を回避する、dashboard_common.py PidGainsControllerの
        # docstring参照)。ESP32 PIDゲイン編集UI自体はStage2には追加しない
        # (このStageの主目的である走行テスト機能だけを使う)。
        self.pid_gains = PidGainsController(self, teleop=self.teleop)
        # 走行テスト中だけ購読する高頻度odomを、このダッシュボードのstate用odomにも
        # 流す(CLIが/api/stateから読むyaw/x/yを、間引き前の精度にするため)。
        self.pid_gains.add_odom_listener(self._on_odom)
        self.step_response_results = StepResponseResultsController(
            str(self.get_parameter("step_response_results_dir").value)
        )
        self.utterance_log = UtteranceLogReader(
            str(self.get_parameter("utterance_log_db").value),
            remote_url=str(self.get_parameter("utterance_log_url").value),
        )
        self.detection_log = DetectionLog(
            str(self.get_parameter("detection_db_file").value)
        )

        handler = make_dashboard_handler(
            html_getter=lambda: HTML.encode("utf-8"),
            state_getter=self._state,
            teleop=self.teleop,
            map_goal=self.map_goal,
            scan=self.scan,
            camera=self.camera,
            auto_speed=self.auto_speed,
            mapping=self.mapping,
            map_lifecycle=self.map_lifecycle,
            boundary=self.boundary,
            keepout_zones=self.keepout_zones,
            map_editor_html_getter=build_map_editor_html,
            crowd_background=self.crowd_background,
            frontier_explorer=self.frontier_explorer,
            utterance_log=self.utterance_log,
            utterance_log_html_getter=build_utterance_log_html,
            detection_log=self.detection_log,
            detection_log_html_getter=build_detection_log_html,
            pid_gains=self.pid_gains,
            step_response_results=self.step_response_results,
            step_response_html_getter=build_step_response_html,
            auth_username=str(self.get_parameter("auth_username").value),
            auth_password=resolve_password(
                self.get_logger(), "stage2_dashboard", "auth_password",
                str(self.get_parameter("auth_password").value),
                str(self.get_parameter("auth_username").value),
            ),
            # Stage2は実地試験用ツールなので、完成版とは違い同時1台制限は
            # 適用しない(複数端末から同時に見たり操作したりできる方が開発効率が良い)。
            enforce_single_controller=False,
        )
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self._server = start_dashboard_server(self, host, port, handler)

        mobile_handler = make_mobile_handler(
            html_getter=lambda: MOBILE_HTML.encode("utf-8"),
            state_getter=self._state,
            teleop=self.teleop,
            auto_speed=self.auto_speed,
            auth_username=str(self.get_parameter("auth_username").value),
            auth_password=resolve_password(
                self.get_logger(), "stage2_dashboard", "auth_password",
                str(self.get_parameter("auth_password").value),
                str(self.get_parameter("auth_username").value),
            ),
            restricted_username=str(self.get_parameter("restricted_username").value),
            restricted_password=resolve_password(
                self.get_logger(), "stage2_dashboard", "restricted_password",
                str(self.get_parameter("restricted_password").value),
                str(self.get_parameter("restricted_username").value),
            ),
            enforce_single_controller=False,
            behavior_control=self.behavior_control,
        )
        mobile_port = int(self.get_parameter("mobile_port").value)
        self._mobile_server = start_dashboard_server(self, host, mobile_port, mobile_handler)

    def destroy_node(self) -> bool:
        self._server.shutdown()
        self._server.server_close()
        self._mobile_server.shutdown()
        self._mobile_server.server_close()
        return super().destroy_node()

    def _on_crowd_state(self, msg: String) -> None:
        data = loads(msg.data, {})
        if isinstance(data, dict):
            self._crowd_state = data

    def _on_behavior_command(self, msg: String) -> None:
        data = loads(msg.data, {})
        if isinstance(data, dict):
            self._behavior_command = data
            self._behavior_command_rx = time.monotonic()

    def _on_announce_text(self, msg: String) -> None:
        self._announce_text = msg.data

    def _on_audio_busy(self, msg: Bool) -> None:
        self._audio_busy = bool(msg.data)

    def _on_odom(self, msg: Odometry) -> None:
        # 【2026-09-24追加】yaw_degはstep_response_test.py CLIの回転(rotate_left/
        # rotate_right)ステップ応答テストが/api/stateから読む(stage1_dashboard_node
        # の_on_odom()と同じ計算方法)。
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw_deg": math.degrees(yaw),
        }

    def _on_map_lifecycle_stopped(self) -> None:
        # 【2026-09-15修正】以前はlambda: (self.map_goal.clear(),
        # self.route_trail.clear())にまとめていたが、tupleの評価は
        # 前者が例外を投げると後者が実行されないため、try/finallyで
        # 両方を確実に呼ぶようにする。
        try:
            self.map_goal.clear()
        finally:
            self.route_trail.clear()

    def _on_map_lifecycle_started(self) -> None:
        try:
            self.mapping.clear_camera_obstacle_points()
        finally:
            self.route_trail.clear()

    def _state(self) -> Dict[str, Any]:
        state = {
            "crowd_level": self._crowd_state.get("level", "unknown"),
            "person_target": self._crowd_state.get("person_target"),
            "mode": self._behavior_command.get("mode"),
            "estop_active": self._behavior_command.get("estop_active", False),
            "manual_override": self._behavior_command.get("manual_override", False),
            "idle_wiggle_enabled": self._behavior_command.get("idle_wiggle_enabled", False),
            # 最後にbehavior_directorの指令を受信してからの経過秒(未受信はNone)。
            "behavior_age_s": (
                time.monotonic() - self._behavior_command_rx
                if self._behavior_command_rx else None
            ),
            "announce_text": self._announce_text,
            "audio_busy": self._audio_busy,
            "odom": self._odom or None,
            "robot_pose": self.robot_pose.get_pose(),
        }
        state.update(self.teleop.status())
        state.update(self.route_trail.status())
        return state


def main() -> None:
    rclpy.init()
    node = Stage2DashboardNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
