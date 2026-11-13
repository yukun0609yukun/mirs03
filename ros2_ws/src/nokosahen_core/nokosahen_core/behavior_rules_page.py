"""時間帯・状況ルール(behavior_rules)の編集ページ(`/behavior_rules`)のHTML。

【2026-10-06追加】完成版ダッシュボード(full_dashboard_node)から配信される。
バックエンドは dashboard_common.BehaviorRulesController / behavior_rules_store.py。
ページ自体はROSに触れず、HTTP API(/api/behavior_rules*)だけを使う。

設計メモ:
- 楽観的UIは使わない。保存はPOSTして、成功したらサーバーから読み直して表示する。
- 全ての可変テキストは escapeHtml() を通してから innerHTML に入れる。
- 変更は「下書き」(未保存)としてブラウザ内に持ち、「保存して反映」で一括検証・保存する。
- guestアカウントは閲覧のみ(入力欄・ボタンを無効化。サーバー側もPOSTは403)。
- 緊急停止ドック(#estopDock)をメイン画面と同じく画面下端に固定表示する。
"""

from .nav_layout import NK_FIELD_JS
from .page_guide import PAGE_RULES, inject_page_guide

_BEHAVIOR_RULES_HTML = r"""<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>のこさへん - 時間帯・状況ルール</title>
<style>
  :root { color-scheme: light; font-family: system-ui, -apple-system, "Segoe UI", sans-serif; background:#f4f6f5; color:#16211d; }
  html, body { margin:0; }
  body { padding-bottom:84px; }
  header { box-sizing:border-box; min-height:44px; padding:6px 18px; background:#124739; color:#fff; display:flex; justify-content:space-between; align-items:center; gap:8px; flex-wrap:wrap; }
  header a { color:#cfe9df; text-decoration:none; font-size:13px; }
  header h1 { margin:0; font-size:18px; }
  main { max-width:1000px; margin:0 auto; padding:14px; display:flex; flex-direction:column; gap:14px; }
  section { background:#fff; border:1px solid #dce4df; border-radius:8px; padding:12px 14px; }
  h2 { margin:0 0 8px; font-size:15px; }
  h3 { margin:12px 0 6px; font-size:13px; color:#33463e; }
  .hint { font-size:13px; color:#53635c; margin:6px 0 0; }
  .row { display:flex; gap:8px; flex-wrap:wrap; align-items:center; }
  button { padding:8px 12px; font-size:13px; border:0; border-radius:6px; background:#176b56; color:#fff; cursor:pointer; }
  button.secondary { background:#5d6f67; }
  button.danger { background:#a32d2d; }
  button.small { padding:4px 9px; font-size:13px; }
  button:disabled { opacity:0.45; cursor:not-allowed; }
  input[type="text"], input[type="number"], select { padding:6px; font-size:13px; border:1px solid #cfd8d3; border-radius:6px; box-sizing:border-box; max-width:100%; }
  input:disabled, select:disabled { background:#f1f3f2; color:#555; }
  label { font-size:13px; color:#33463e; }
  .banner { padding:8px 12px; border-radius:6px; font-size:13px; }
  .banner.err { background:#faece7; color:#993c1d; }
  .banner.ok { background:#e1f5ee; color:#0f6e56; }
  .banner.warn { background:#fff3cd; color:#7a5b00; }
  .banner.info { background:#eef3f0; color:#33463e; }
  .banner ul { margin:4px 0 0 18px; padding:0; }
  .grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(200px,1fr)); gap:8px; }
  .metric { background:#eef3f0; border-radius:8px; padding:6px 10px; min-width:0; }
  .metric span { display:block; font-size:13px; color:#53635c; }
  .metric b { font-size:14px; word-break:break-all; }
  .pill { display:inline-block; padding:2px 9px; border-radius:999px; font-size:13px; font-weight:700; }
  .pill.ok { background:#e1f5ee; color:#0f6e56; }
  .pill.bad { background:#faece7; color:#993c1d; }
  .pill.wait { background:#fff3cd; color:#7a5b00; }
  .pill.off { background:#e4e8e6; color:#53635c; }
  .rule { border:1px solid #dce4df; border-radius:8px; margin-top:10px; background:#fbfcfb; }
  .rule.disabled .rule-head { opacity:0.75; }
  .rule.has-error { border-color:#d6735a; box-shadow:0 0 0 2px #faece7; }
  .rule-head { display:flex; gap:10px; align-items:center; padding:8px 10px; flex-wrap:wrap; }
  .rule-title { flex:1 1 220px; min-width:0; }
  .rule-title b { font-size:14px; }
  .rule-title small { display:block; color:#53635c; font-size:13px; word-break:break-all; }
  .rule-actions { display:flex; gap:6px; flex-wrap:wrap; }
  .switch { display:inline-flex; align-items:center; gap:6px; font-size:13px; cursor:pointer; }
  .switch input { width:18px; height:18px; }
  .rule-body { border-top:1px dashed #dce4df; padding:8px 12px 12px; }
  .field { display:flex; flex-direction:column; gap:3px; }
  .checks { display:flex; flex-wrap:wrap; gap:6px 12px; }
  .checks label { display:inline-flex; align-items:center; gap:4px; font-size:13px; color:#16211d; }
  .policy-list { display:grid; grid-template-columns:repeat(auto-fit,minmax(260px,1fr)); gap:4px 14px; }
  .policy-list label { display:flex; gap:6px; align-items:flex-start; font-size:13px; color:#16211d; }
  .policy-list small { color:#53635c; display:block; font-size:13px; }
  .tr-row { display:flex; align-items:center; gap:6px; margin-bottom:4px; }
  .rule-errors { margin:6px 0 0; padding:6px 10px; background:#faece7; color:#993c1d; border-radius:6px; font-size:13px; }
  table { width:100%; border-collapse:collapse; font-size:13px; }
  th, td { border-bottom:1px solid #e4ebe7; padding:6px 8px; text-align:left; vertical-align:top; }
  th { background:#f1f5f3; font-size:13px; }
  .chip { display:inline-block; margin:1px 4px 1px 0; padding:1px 8px; border-radius:999px; background:#eef3f0; font-size:13px; }
  .chip.maybe { background:#fff3cd; color:#7a5b00; }
  .pin-dot { display:inline-block; width:14px; height:14px; border-radius:50%; border:1px solid #888; vertical-align:middle; background:#ccc; }
  .pin-box select { min-width:240px; }
  .pin-note { margin-top:4px; font-size:13px; }
  .pin-note.warn { background:#fff3cd; color:#7a5b00; padding:4px 8px; border-radius:6px; }
  .pin-note.info { color:#53635c; }
  #estopDock { position:fixed; left:0; right:0; bottom:0; z-index:2000; box-sizing:border-box; padding:8px 12px; background:rgba(18,71,57,0.96); box-shadow:0 -2px 10px rgba(0,0,0,0.3); display:flex; gap:10px; align-items:center; }
  #estopDock .stop-btn { flex:1 1 auto; margin:0; padding:14px; font-size:18px; min-height:52px; border:2px solid #fff; font-weight:700; background:#a32d2d; color:#fff; border-radius:8px; }
  #estopDock .stop-btn.engaged { background:#5c1717; }
  #estopDock a { color:#cfe9df; font-size:13px; white-space:nowrap; }
  .dirty-bar { position:sticky; top:0; z-index:50; }
  @media (max-width:640px) { main { padding:8px; } .policy-list { grid-template-columns:1fr; } }
__PAGE_GUIDE_CSS__
</style>
</head>
<body>
<header>
  <h1>時間帯・状況ルール</h1>
  <div class="row">
    <span id="accountLabel" style="font-size:13px;opacity:0.85;"></span>
    <a href="/">&larr; 操作盤に戻る</a>
  </div>
</header>
<main>
__PAGE_GUIDE_HTML__
  <div id="globalMsg"></div>

  <section id="liveSection">
    <h2>いまの状態</h2>
    <div class="row" style="margin-bottom:8px;">
      <span id="liveBadge" class="pill wait">確認中...</span>
      <span id="liveMsg" class="hint" style="margin:0;"></span>
    </div>
    <div class="grid" id="liveGrid"></div>
    <div id="liveWarn"></div>
    <p class="hint">2秒ごとに更新します。このページを閉じると、ダッシュボードはこの状態の購読をやめます(負荷がかかりません)。</p>
  </section>

  <section>
    <h2>ルール</h2>
    <div class="banner info" style="margin-bottom:8px;">
      ルールは「この時間帯はこうする」「この状況ならこうする」を決める表です。安全装置(緊急停止・手動運転・人物接近停止・
      走行範囲外・立入禁止エリア・カメラ障害物)は常にルールより優先され、ルールで動きを増やすことはできません。
      ルールが無い/すべて無効なら、これまでどおりの動作です。
    </div>
    <div id="fileInfo" class="hint"></div>
    <div id="pinInfo" class="hint"></div>
    <div id="pinWarn"></div>
    <div id="dirtyBar" class="dirty-bar"></div>
    <div id="saveErrors"></div>
    <div class="row" style="margin:10px 0;">
      <button id="addBtn" type="button">ルールを追加</button>
      <button id="exampleBtn" type="button" class="secondary">例から読み込む</button>
      <button id="reloadFileBtn" type="button" class="secondary">保存済みの内容を読み直す</button>
      <button id="notifyBtn" type="button" class="secondary" title="ファイルを変えずに、behavior_directorへ再読み込みだけ通知します">director再読み込み通知</button>
    </div>
    <h3>全体設定</h3>
    <div class="row">
      <div class="field"><label for="minHold">最小保持時間(秒、0〜600)</label>
        <input id="minHold" type="number" min="0" max="600" step="1" style="width:120px;" /></div>
      <div class="field" style="flex:1 1 260px;"><label for="docDesc">説明(メモ)</label>
        <input id="docDesc" type="text" style="width:100%;" /></div>
    </div>
    <p class="hint">最小保持時間: ルールが切り替わった後、この秒数の間は次のルールへ切り替えません(混雑度のゆらぎで方針が往復しないため)。</p>
    <div id="rulesList"></div>
    <div class="row" style="margin-top:14px;">
      <button id="saveBtn" type="button">保存して反映</button>
      <button id="discardBtn" type="button" class="secondary">変更を破棄</button>
      <span id="saveStatus" class="hint" style="margin:0;"></span>
    </div>
    <p class="hint">保存するとサーバー側で検証し、1つでも不正があれば何も書き込みません。保存できたら直前のファイルを .bak に残し、behavior_directorへ即時に再読み込みを通知します。</p>
  </section>

  <section>
    <h2>次の24時間の予定</h2>
    <div class="row" style="margin-bottom:6px;">
      <button id="previewBtn" type="button" class="secondary">保存済みの内容で再計算</button>
      <span id="previewNow" class="hint" style="margin:0;"></span>
    </div>
    <div class="banner warn" id="previewNote" style="margin-bottom:8px;">
      この予定は「保存済みのルール」を、時刻・曜日・日付の条件だけで計算したものです。混雑度・人物・残数・無人時間などの
      状況に依存する条件は予測できないため、「条件次第」として別に表示します(実際に発動するかは当日の状況しだいです)。
      未保存の編集内容は反映されません。
    </div>
    <div id="previewBody">読み込み中...</div>
  </section>
</main>

<div id="estopDock">
  <button id="estopBtn" class="stop-btn" type="button">緊急停止(ソフト)</button>
  <a href="/">操作盤へ</a>
</div>

<script>
__NK_FIELD_JS__
__PAGE_GUIDE_JS__
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

  function escapeHtml(value) {
    return String(value === null || value === undefined ? "" : value).replace(/[&<>"']/g, (ch) => ({
      "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"
    }[ch]));
  }
  const $ = (id) => document.getElementById(id);

  const POLICY_LABELS = {
    normal: ["通常(現行動作)", "他の方針とは併用できません。下のパラメータだけ使いたいときに選びます"],
    stay: ["その場で待機", "接近・見回し・揺動・ホーム帰還をしません(発話は従来どおり)"],
    announce_only: ["待機して発話のみ", "stayと同じ動き"],
    approach_enabled: ["人物への接近を許可", "既定の動作"],
    approach_disabled: ["人物への接近を禁止", "混雑時の入口移動もせず、停止して発話のみ"],
    crowd_scan_enabled: ["見回しを許可", "既定の動作"],
    crowd_scan_disabled: ["見回しを禁止", "待機します"],
    face_enabled: ["向くを許可", "人がいれば、前進せずその場で旋回して人の多い方を向きます"],
    face_disabled: ["向かない", "その場で人の方を向く動作をしません"],
    idle_wiggle_on: ["待機時の左右揺動を強制ON", "ダッシュボードの設定より優先"],
    idle_wiggle_off: ["待機時の左右揺動を強制OFF", "ダッシュボードの設定より優先"],
    go_to_entrance: ["入口へ1回移動して待機", "ルールが有効になった時に1回だけ入口ゴールを送ります"],
    return_home: ["ホーム位置へ戻って待機", "現状ホームは入口と同じ座標です"],
    goto_pin: ["地図のピンへ移動して待機", "選んだピンの位置・向きへ1回だけ移動し、到着したら待機します。地図が読み込まれていて、そのピンがある時だけ動きます(無ければその場で待機)"],
    quiet: ["発話しない", "呼びかけ・案内の発話を出しません"],
    speed_scale: ["自律の速度を制限(倍率)", "直接走行(見回し・揺動など)の速度を倍率で下げます。上げることはできません"],
  };
  const WEEKDAY_LABELS = { mon: "月", tue: "火", wed: "水", thu: "木", fri: "金", sat: "土", sun: "日" };
  const CROWD_LABELS = { calm: "空いている(calm)", crowded: "混雑(crowded)", unknown: "不明(unknown)" };

  let meta = null;
  let readOnly = true;           // 取得できるまでは編集不可にしておく
  let draft = null;              // 編集中の文書(未保存の下書き)
  let serverDoc = null;          // 最後にサーバーから読んだ文書(dirty判定用)
  let serverSig = null;          // 競合検知用(最後に読んだファイルの指紋)
  let serverInfo = null;
  const openCards = new Set();   // 編集フォームを開いているルールのindex
  let ruleErrors = {};           // index -> [エラー文]
  let busy = false;
  let pinsInfo = null;           // /api/pins(読み込み中の地図のピン一覧)。取得できるまでnull
  let pinsSig = "";

  // ---------------------------------------------------------------- 通信
  async function api(path, opts) {
    const res = await fetch(path, opts);
    let data = null;
    try { data = await res.json(); } catch (e) { /* 本文がJSONでない */ }
    return { ok: res.ok, status: res.status, data: data || {} };
  }

  function showGlobal(kind, text) {
    $("globalMsg").innerHTML = text ? `<div class="banner ${kind}">${escapeHtml(text)}</div>` : "";
  }

  function errorsHtml(title, errors) {
    const items = (errors || []).map((e) => `<li>${escapeHtml(e)}</li>`).join("");
    return `<div class="banner err">${escapeHtml(title)}<ul>${items}</ul></div>`;
  }

  // ---------------------------------------------------------------- いまの状態(2秒ごと)
  function fmtTime(iso) {
    if (!iso) return "-";
    const s = String(iso);
    return s.length >= 16 ? `${s.slice(5, 10).replace("-", "/")} ${s.slice(11, 16)}` : s;
  }

  async function pollLive() {
    if (document.hidden) return;
    try {
      const r = await api("/api/behavior_rules/state");
      if (!r.ok) throw new Error("HTTP " + r.status);
      renderLive(r.data);
    } catch (e) {
      $("liveBadge").className = "pill bad";
      $("liveBadge").textContent = "ダッシュボードに接続できません";
      $("liveMsg").textContent = "";
    }
  }

  function renderLive(d) {
    const badge = $("liveBadge");
    const st = d.state;
    if (d.status === "ok" && st) {
      badge.className = "pill ok";
      badge.textContent = "behavior_director 応答中";
    } else if (d.status === "waiting") {
      badge.className = "pill wait";
      badge.textContent = "状態を待っています";
    } else {
      badge.className = "pill bad";
      badge.textContent = "behavior_director 無応答/未起動";
    }
    $("liveMsg").textContent = d.message || "";
    if (!st) { $("liveGrid").innerHTML = ""; $("liveWarn").innerHTML = ""; return; }
    const enabled = Number(st.enabled_rule_count || 0);
    const total = Number(st.rule_count || 0);
    const active = st.active_rule_id
      ? `${st.active_rule_id}(${st.active_rule_name || ""})`
      : "なし(現行動作)";
    const cells = [
      ["適用中ルール", active],
      ["方針", st.policy || "normal"],
      ...(st.effects && st.effects.go_to === "pin"
        ? [["行き先ピン", `${st.effects.go_to_pin_name || ""} (${st.effects.go_to_pin_id || "-"})`]] : []),
      ["適用開始", fmtTime(st.since)],
      ["次の切替の目安", st.next_change_at ? fmtTime(st.next_change_at) : "なし/48時間以内に無し"],
      ["有効なルール", `${enabled} / ${total} 件`],
      ["最小保持時間", `${st.min_hold_seconds ?? "-"} 秒`],
      ["directorが読むファイル", st.rules_file || "(未設定)"],
    ];
    $("liveGrid").innerHTML = cells.map(([k, v]) =>
      `<div class="metric"><span>${escapeHtml(k)}</span><b>${escapeHtml(v)}</b></div>`).join("");
    let warn = "";
    if (st.load_error) {
      warn += `<div class="banner err" style="margin-top:8px;">直近のルール読み込みに失敗しました(直前のルールを使い続けています): ${escapeHtml(st.load_error)}</div>`;
    }
    if (enabled === 0) {
      warn += `<div class="banner info" style="margin-top:8px;">有効なルールがありません。ロボットはこれまでどおりの動作です。</div>`;
    }
    if (d.path_mismatch) {
      warn += `<div class="banner warn" style="margin-top:8px;">behavior_directorが読んでいるファイル(${escapeHtml(st.rules_file)})と、このページが編集するファイル(${escapeHtml(d.dashboard_rules_file)})のパスが違います。別のマシン/ユーザーで動いている場合、ここで保存しても反映されません。設定(behavior_rules_file / rules_file)を確認してください。</div>`;
    }
    $("liveWarn").innerHTML = warn;
  }

  // ---------------------------------------------------------------- ルール文書の読み込み
  function clone(o) { return JSON.parse(JSON.stringify(o)); }

  // ---------------------------------------------------------------- 地図ピン(goto_pin用)
  const COLOR_RE = /^#[0-9a-fA-F]{6}$/;
  function pinList() { return pinsInfo && Array.isArray(pinsInfo.pins) ? pinsInfo.pins : []; }
  function loadedMap() { return pinsInfo && pinsInfo.loaded_map ? String(pinsInfo.loaded_map) : ""; }
  function pinColor(p) { return p && COLOR_RE.test(String(p.color || "")) ? p.color : "#cccccc"; }
  function ruleParams(r) { return (r.then && r.then.params) || {}; }
  function hasGotoPin(r) {
    const pol = r.then && r.then.policy;
    return Array.isArray(pol) ? pol.includes("goto_pin") : pol === "goto_pin";
  }

  async function loadPins() {
    try {
      const r = await api("/api/pins");
      if (!r.ok) throw new Error("HTTP " + r.status);
      pinsInfo = r.data;
    } catch (e) {
      pinsInfo = null;
    }
    const sig = JSON.stringify(pinsInfo ? [pinsInfo.loaded_map, pinsInfo.pins] : null);
    if (sig !== pinsSig) { pinsSig = sig; refreshPinUi(); }
  }

  // 行き先ピンの状態: {level: "ok"|"warn"|"info", text, pin}
  function pinStatus(r) {
    const params = ruleParams(r);
    const id = params.pin_id || "";
    if (!pinsInfo) return { level: "warn", text: "ピン一覧を取得できませんでした(ダッシュボードとの通信を確認してください)。" };
    const map = loadedMap();
    if (!map) {
      return { level: "warn", text: "地図が読み込まれていません。地図を読み込むとピンが選べます。実行時は、地図が読み込まれていてそのピンがあるときだけ動き、無ければその場で待機します。" };
    }
    if (!id) {
      return pinList().length
        ? { level: "info", text: "行き先のピンを選んでください。" }
        : { level: "warn", text: `地図「${map}」にはピンがありません。操作盤の地図画面でピンを立ててください。` };
    }
    if (params.pin_map && params.pin_map !== map) {
      return { level: "warn", text: `このピンは別の地図「${params.pin_map}」用です(いまの地図: 「${map}」)。実行時はこのピンへ行かず、その場で待機します。` };
    }
    const pin = pinList().find((p) => p.id === id);
    if (!pin) {
      return { level: "warn", text: `このピンは現在の地図「${map}」にありません(削除された、または別の地図)。実行時はこのピンへ行かず、その場で待機します。` };
    }
    return { level: "ok", text: `地図「${map}」のピン(${Number(pin.x).toFixed(2)}, ${Number(pin.y).toFixed(2)}) m、向き ${Math.round(pin.yaw_deg)}° へ移動します。`, pin };
  }

  function pinOptionsHtml(r) {
    const params = ruleParams(r);
    const selId = params.pin_id || "";
    let found = false;
    const opts = pinList().map((p) => {
      const sel = p.id === selId;
      if (sel) found = true;
      return `<option value="${escapeHtml(p.id)}" style="color:${escapeHtml(pinColor(p))}" ${sel ? "selected" : ""}>● ${escapeHtml(p.name)}</option>`;
    });
    const head = `<option value="" ${selId ? "" : "selected"}>(ピンを選んでください)</option>`;
    const missing = selId && !found
      ? `<option value="${escapeHtml(selId)}" selected>⚠ ${escapeHtml(params.pin_name || selId)}(現在の地図に無い)</option>` : "";
    return head + missing + opts.join("");
  }

  function pinBoxInner(r) {
    const st = pinStatus(r);
    const sel = pinList().find((p) => p.id === ruleParams(r).pin_id);
    const dis = readOnly ? "disabled" : "";
    return `<label>行き先のピン(読み込み中の地図: ${escapeHtml(loadedMap() || "なし")})</label>
      <div class="row"><span class="pin-dot" style="background:${escapeHtml(pinColor(sel))}"></span>
        <select data-f="pin" ${dis}>${pinOptionsHtml(r)}</select></div>
      <div class="pin-note ${st.level === "warn" ? "warn" : "info"}">${escapeHtml(st.text)}</div>`;
  }

  // ピン一覧や読み込み地図が変わったら、開いているフォームのピン欄と上の案内を作り直す(入力中の欄は触らない)
  function refreshPinUi() {
    renderPinInfo();
    if (!draft) return;
    document.querySelectorAll(".pin-box").forEach((box) => {
      if (box.contains(document.activeElement)) return;
      const body = box.closest(".rule-body");
      const r = body && draft.rules[Number(body.getAttribute("data-i"))];
      if (r) box.innerHTML = pinBoxInner(r);
    });
  }

  function renderPinInfo() {
    const el = $("pinInfo");
    if (!el) return;
    if (!pinsInfo) {
      el.textContent = "地図ピン: 取得できませんでした。";
    } else if (!loadedMap()) {
      el.textContent = "地図ピン: 地図が読み込まれていません(読み込むと、ルールの行き先にピンを選べます)。";
    } else {
      el.textContent = `地図ピン: 読み込み中の地図「${loadedMap()}」 / ピン${pinList().length}個`;
    }
    const w = [];
    if (serverDoc && Array.isArray(serverDoc.rules)) {
      serverDoc.rules.forEach((r, i) => {
        if (!hasGotoPin(r)) return;
        const st = pinStatus(r);
        if (st.level === "warn") w.push(`ルール「${r.name || r.id}」: ${st.text}`);
      });
    }
    $("pinWarn").innerHTML = w.length
      ? `<div class="banner warn" style="margin-top:6px;">保存済みのルールに、いま使えない行き先ピンがあります(保存はできますが、実行時は走行せずその場で待機します)。<ul>${w.map((t) => `<li>${escapeHtml(t)}</li>`).join("")}</ul></div>`
      : "";
  }

  function emptyDoc() {
    return { version: 1, description: "", min_hold_seconds: meta ? meta.default_min_hold_seconds : 10, rules: [] };
  }

  async function loadRules(keepMessage) {
    try {
      const r = await api("/api/behavior_rules");
      if (!r.ok) { showGlobal("err", `ルールを読み込めませんでした(HTTP ${r.status})`); return false; }
      const d = r.data;
      meta = d.meta;
      serverInfo = d;
      serverSig = d.sig;
      serverDoc = d.document && typeof d.document === "object" ? d.document : emptyDoc();
      if (!Array.isArray(serverDoc.rules)) serverDoc.rules = [];
      draft = clone(serverDoc);
      openCards.clear();
      ruleErrors = {};
      $("saveErrors").innerHTML = "";
      if (!keepMessage) showGlobal("", "");
      if (!d.valid) {
        $("saveErrors").innerHTML = errorsHtml("保存済みのファイルに問題があります(directorはこのファイルを拒否し、直前のルールを使い続けます)。修正して保存するか、例から読み込んでください。", d.errors);
      }
      renderAll();
      return true;
    } catch (e) {
      showGlobal("err", "ルールを読み込めませんでした: " + (e && e.message ? e.message : "通信エラー"));
      return false;
    }
  }

  // ---------------------------------------------------------------- 下書き → 保存用の文書
  function toNum(v) {
    const t = String(v).trim();
    if (t === "") return null;
    const n = Number(t);
    return Number.isFinite(n) ? n : t;  // 数値にできなければ文字のまま送り、サーバーに拒否させる
  }

  function isDirty() {
    return draft && serverDoc && JSON.stringify(draft) !== JSON.stringify(serverDoc);
  }

  // ---------------------------------------------------------------- 描画
  function renderAll() {
    $("minHold").value = draft.min_hold_seconds ?? "";
    $("docDesc").value = draft.description ?? "";
    for (const id of ["minHold", "docDesc"]) $(id).disabled = readOnly;
    renderFileInfo();
    renderRules();
    renderDirty();
    applyReadOnly();
    renderPinInfo();
  }

  function renderFileInfo() {
    if (!serverInfo) return;
    const bits = [`編集中のファイル: ${serverInfo.path}`];
    bits.push(serverInfo.exists ? "(保存済み)" : "(まだ作られていません。保存すると新規作成されます)");
    if (serverInfo.backup_exists) bits.push(`/ 直前の版: ${serverInfo.backup_path}`);
    $("fileInfo").textContent = bits.join(" ");
  }

  function renderDirty() {
    $("dirtyBar").innerHTML = isDirty()
      ? `<div class="banner warn">未保存の変更があります。「保存して反映」を押すまで、ロボットには反映されません。</div>`
      : "";
  }

  function applyReadOnly() {
    for (const id of ["addBtn", "exampleBtn", "saveBtn", "discardBtn", "notifyBtn"]) $(id).disabled = readOnly || busy;
    if (readOnly) {
      $("saveStatus").textContent = "guestアカウントは閲覧専用です(編集・保存はできません)。";
    }
  }

  function policyChips(r) {
    const pols = (r.then && Array.isArray(r.then.policy)) ? r.then.policy
      : (r.then && r.then.policy ? [r.then.policy] : []);
    return pols.map((p) => {
      let label = (POLICY_LABELS[p] || [p])[0];
      if (p === "goto_pin") label = `ピン「${ruleParams(r).pin_name || ruleParams(r).pin_id || "未選択"}」へ移動して待機`;
      return `<span class="chip">${escapeHtml(label)}</span>`;
    }).join("");
  }

  function whenSummary(r) {
    const w = r.when || {};
    const bits = [];
    if (Array.isArray(w.time_ranges) && w.time_ranges.length) bits.push("時間 " + w.time_ranges.join(", "));
    if (Array.isArray(w.weekdays) && w.weekdays.length) bits.push("曜日 " + w.weekdays.map((d) => WEEKDAY_LABELS[d] || d).join(""));
    if (Array.isArray(w.dates) && w.dates.length) bits.push("日付 " + w.dates.join(", "));
    if (Array.isArray(w.crowd_level) && w.crowd_level.length) bits.push("混雑度 " + w.crowd_level.join("/"));
    if (w.person_present === true) bits.push("人物あり");
    if (w.person_present === false) bits.push("人物なし");
    if (w.menu) bits.push(`メニュー「${w.menu}」`);
    if (w.remaining_ratio_below !== undefined) bits.push(`残数比<${w.remaining_ratio_below}`);
    if (w.remaining_ratio_above !== undefined) bits.push(`残数比>${w.remaining_ratio_above}`);
    if (w.idle_seconds_above !== undefined) bits.push(`人物不在${w.idle_seconds_above}秒超`);
    return bits.length ? bits.join(" かつ ") : "(常時)";
  }

  function renderRules() {
    const list = $("rulesList");
    const rules = draft.rules;
    if (!rules.length) {
      list.innerHTML = `<p class="hint">ルールはまだありません(現行動作のまま)。「ルールを追加」または「例から読み込む」で作れます。</p>`;
      return;
    }
    list.innerHTML = rules.map((r, i) => ruleCardHtml(r, i)).join("");
  }

  function ruleCardHtml(r, i) {
    const open = openCards.has(i);
    const errs = ruleErrors[i] || [];
    const enabled = r.enabled === true;
    const dis = readOnly ? "disabled" : "";
    return `<div class="rule ${enabled ? "" : "disabled"} ${errs.length ? "has-error" : ""}" data-i="${i}">
      <div class="rule-head">
        <label class="switch"><input type="checkbox" data-act="toggle" ${enabled ? "checked" : ""} ${dis} />${enabled ? "有効" : "無効"}</label>
        <div class="rule-title">
          <b data-sum="name">${escapeHtml(r.name || r.id || "(名前なし)")}</b>
          <small data-sum="meta">id: ${escapeHtml(r.id)} / 優先度 ${escapeHtml(r.priority ?? 0)} / ${escapeHtml(whenSummary(r))}</small>
          <div data-sum="chips">${policyChips(r)}</div>
        </div>
        <div class="rule-actions">
          <button type="button" class="small secondary" data-act="edit">${open ? "閉じる" : (readOnly ? "内容を見る" : "編集")}</button>
          <button type="button" class="small secondary" data-act="dup" ${dis}>複製</button>
          <button type="button" class="small danger" data-act="del" ${dis}>削除</button>
        </div>
      </div>
      ${errs.length ? `<div class="rule-errors">${errs.map((e) => `<div>${escapeHtml(e)}</div>`).join("")}</div>` : ""}
      ${open ? ruleFormHtml(r, i) : ""}
    </div>`;
  }

  function ruleFormHtml(r, i) {
    const w = r.when || {};
    const then = r.then || {};
    const pols = Array.isArray(then.policy) ? then.policy : (then.policy ? [then.policy] : []);
    const params = then.params || {};
    const dis = readOnly ? "disabled" : "";
    const trs = Array.isArray(w.time_ranges) ? w.time_ranges : [];
    const trHtml = trs.map((t, k) => {
      const idx = String(t).indexOf("-");
      const s = idx >= 0 ? String(t).slice(0, idx) : String(t);
      const e = idx >= 0 ? String(t).slice(idx + 1) : "";
      return `<div class="tr-row" data-k="${k}">
        <input type="text" data-f="tr-start" value="${escapeHtml(s)}" placeholder="11:30" maxlength="5" style="width:80px;" ${dis} />
        <span>〜</span>
        <input type="text" data-f="tr-end" value="${escapeHtml(e)}" placeholder="13:00" maxlength="5" style="width:80px;" ${dis} />
        <button type="button" class="small secondary" data-act="tr-del" ${dis}>削除</button></div>`;
    }).join("");
    const weekdays = (meta ? meta.weekdays : []).map((d) =>
      `<label><input type="checkbox" data-f="weekday" value="${escapeHtml(d)}" ${Array.isArray(w.weekdays) && w.weekdays.includes(d) ? "checked" : ""} ${dis} />${escapeHtml(WEEKDAY_LABELS[d] || d)}</label>`).join("");
    const crowds = (meta ? meta.crowd_levels : []).map((c) =>
      `<label><input type="checkbox" data-f="crowd" value="${escapeHtml(c)}" ${Array.isArray(w.crowd_level) && w.crowd_level.includes(c) ? "checked" : ""} ${dis} />${escapeHtml(CROWD_LABELS[c] || c)}</label>`).join("");
    const conflictOf = (p) => {
      const out = [];
      for (const [a, b] of (meta ? meta.conflicts : [])) { if (a === p) out.push(b); if (b === p) out.push(a); }
      return out;
    };
    const policyBoxes = (meta ? meta.policies : Object.keys(POLICY_LABELS)).map((p) => {
      const checked = pols.includes(p);
      const blocked = !checked && (conflictOf(p).some((c) => pols.includes(c)) || (p !== "normal" && pols.includes("normal")) || (p === "normal" && pols.length > 0));
      const lab = POLICY_LABELS[p] || [p, ""];
      return `<label><input type="checkbox" data-f="policy" value="${escapeHtml(p)}" ${checked ? "checked" : ""} ${(readOnly || blocked) ? "disabled" : ""} />
        <span>${escapeHtml(lab[0])} <code>${escapeHtml(p)}</code><small>${escapeHtml(lab[1])}</small></span></label>`;
    }).join("");
    const person = w.person_present === true ? "true" : (w.person_present === false ? "false" : "");
    const showScale = pols.includes("speed_scale");
    return `<div class="rule-body" data-i="${i}">
      <div class="grid">
        <div class="field"><label>名前</label><input type="text" data-f="name" value="${escapeHtml(r.name ?? "")}" ${dis} /></div>
        <div class="field"><label>id(英数字・_・-、64文字まで。一意)</label><input type="text" data-f="id" value="${escapeHtml(r.id ?? "")}" ${dis} /></div>
        <div class="field"><label>優先度(整数 -1000〜1000、大きいほど優先)</label><input type="number" step="1" min="-1000" max="1000" data-f="priority" value="${escapeHtml(r.priority ?? 0)}" ${dis} /></div>
        <div class="field" style="grid-column:1/-1;"><label>説明(メモ)</label><input type="text" data-f="description" value="${escapeHtml(r.description ?? "")}" ${dis} style="width:100%;" /></div>
      </div>
      <h3>いつ・どんな時に使うか(書いた条件がすべて成り立ったときだけ有効)</h3>
      <div class="field"><label>時間帯(日付またぎ可。例 22:00-06:00。終了だけ 24:00 可)</label>
        ${trHtml}
        <div><button type="button" class="small secondary" data-act="tr-add" ${dis}>時間帯を追加</button></div></div>
      <div class="field" style="margin-top:8px;"><label>曜日(未選択=毎日)</label><div class="checks">${weekdays}</div></div>
      <div class="field" style="margin-top:8px;"><label>日付(YYYY-MM-DD、カンマ区切り。空=指定なし)</label>
        <input type="text" data-f="dates" value="${escapeHtml(Array.isArray(w.dates) ? w.dates.join(", ") : "")}" placeholder="2026-10-24, 2026-10-25" ${dis} /></div>
      <h3>状況の条件(混雑・人物・残数・無人時間。次の24時間の予定では予測できません)</h3>
      <div class="field"><label>混雑度(未選択=問わない)</label><div class="checks">${crowds}</div></div>
      <div class="grid" style="margin-top:8px;">
        <div class="field"><label>人物</label><select data-f="person" ${dis}>
          <option value="" ${person === "" ? "selected" : ""}>問わない</option>
          <option value="true" ${person === "true" ? "selected" : ""}>いる</option>
          <option value="false" ${person === "false" ? "selected" : ""}>いない</option></select></div>
        <div class="field"><label>メニュー名(残数比の対象。空=最も少ないメニュー)</label><input type="text" data-f="menu" value="${escapeHtml(w.menu ?? "")}" ${dis} /></div>
        <div class="field"><label>残数比がこれ未満(0〜1)</label><input type="number" step="0.05" min="0" max="1" data-f="below" value="${escapeHtml(w.remaining_ratio_below ?? "")}" ${dis} /></div>
        <div class="field"><label>残数比がこれ超過(0〜1)</label><input type="number" step="0.05" min="0" max="1" data-f="above" value="${escapeHtml(w.remaining_ratio_above ?? "")}" ${dis} /></div>
        <div class="field"><label>人物が見えない時間がこの秒数を超えたら</label><input type="number" step="1" min="0" data-f="idle" value="${escapeHtml(w.idle_seconds_above ?? "")}" ${dis} /></div>
      </div>
      <h3>方針(1つ以上。矛盾する組み合わせは選べません)</h3>
      <div class="policy-list">${policyBoxes}</div>
      <div class="grid" style="margin-top:8px;">
        ${pols.includes("goto_pin") ? `<div class="field pin-box" style="grid-column:1/-1;">${pinBoxInner(r)}</div>` : ""}
        ${showScale ? `<div class="field"><label>speed_scale の倍率(${escapeHtml(meta ? meta.speed_scale_min : 0.3)}〜${escapeHtml(meta ? meta.speed_scale_max : 1)})</label>
          <input type="number" step="0.05" min="${escapeHtml(meta ? meta.speed_scale_min : 0.3)}" max="${escapeHtml(meta ? meta.speed_scale_max : 1)}" data-f="scale" value="${escapeHtml(params.scale ?? "")}" ${dis} /></div>` : ""}
        <div class="field"><label>発話依頼の最短間隔(秒、${escapeHtml(meta ? meta.announce_interval_min : 10)}〜${escapeHtml(meta ? meta.announce_interval_max : 600)}。空=既定)</label>
          <input type="number" step="1" min="${escapeHtml(meta ? meta.announce_interval_min : 10)}" max="${escapeHtml(meta ? meta.announce_interval_max : 600)}" data-f="interval" value="${escapeHtml(params.announce_min_interval_seconds ?? "")}" ${dis} /></div>
      </div>
    </div>`;
  }

  // ---------------------------------------------------------------- 下書きの編集(イベント委譲)
  function updateSummary(i) {
    const card = document.querySelector(`.rule[data-i="${i}"]`);
    if (!card) return;
    const r = draft.rules[i];
    card.querySelector('[data-sum="name"]').textContent = r.name || r.id || "(名前なし)";
    card.querySelector('[data-sum="meta"]').textContent = `id: ${r.id} / 優先度 ${r.priority ?? 0} / ${whenSummary(r)}`;
    card.querySelector('[data-sum="chips"]').innerHTML = policyChips(r);
    card.classList.toggle("disabled", r.enabled !== true);
  }

  function ensure(r, ...path) {
    let o = r;
    for (const k of path) { if (!o[k] || typeof o[k] !== "object") o[k] = {}; o = o[k]; }
    return o;
  }
  function setWhen(r, key, value) {
    const w = ensure(r, "when");
    const empty = value === null || value === undefined || value === "" || (Array.isArray(value) && value.length === 0);
    if (empty) delete w[key]; else w[key] = value;
  }
  function setParam(r, key, value) {
    const p = ensure(r, "then", "params");
    if (value === null || value === "") delete p[key]; else p[key] = value;
  }

  function collectTimeRanges(body) {
    const out = [];
    body.querySelectorAll(".tr-row").forEach((row) => {
      const s = row.querySelector('[data-f="tr-start"]').value.trim();
      const e = row.querySelector('[data-f="tr-end"]').value.trim();
      out.push(`${s}-${e}`);   // 空欄が混じっていても送り、サーバーの検証で理由を出す
    });
    return out;
  }

  function onFieldChange(ev) {
    if (readOnly) return;
    const el = ev.target;
    const f = el.getAttribute && el.getAttribute("data-f");
    const body = el.closest(".rule-body");
    if (!f || !body) return;
    const i = Number(body.getAttribute("data-i"));
    const r = draft.rules[i];
    if (!r) return;
    let structural = false;
    if (f === "name") r.name = el.value;
    else if (f === "id") r.id = el.value.trim();
    else if (f === "description") { if (el.value === "") delete r.description; else r.description = el.value; }
    else if (f === "priority") { const n = toNum(el.value); r.priority = n === null ? 0 : n; }
    else if (f === "tr-start" || f === "tr-end") setWhen(r, "time_ranges", collectTimeRanges(body));
    else if (f === "weekday") setWhen(r, "weekdays", Array.from(body.querySelectorAll('[data-f="weekday"]:checked')).map((c) => c.value));
    else if (f === "dates") setWhen(r, "dates", el.value.split(/[\s,、]+/).filter((x) => x));
    else if (f === "crowd") setWhen(r, "crowd_level", Array.from(body.querySelectorAll('[data-f="crowd"]:checked')).map((c) => c.value));
    else if (f === "person") setWhen(r, "person_present", el.value === "" ? null : el.value === "true");
    else if (f === "menu") setWhen(r, "menu", el.value.trim());
    else if (f === "below") setWhen(r, "remaining_ratio_below", toNum(el.value));
    else if (f === "above") setWhen(r, "remaining_ratio_above", toNum(el.value));
    else if (f === "idle") setWhen(r, "idle_seconds_above", toNum(el.value));
    else if (f === "scale") setParam(r, "scale", toNum(el.value));
    else if (f === "interval") setParam(r, "announce_min_interval_seconds", toNum(el.value));
    else if (f === "pin") {
      const id = el.value;
      if (!id) {
        setParam(r, "pin_id", null); setParam(r, "pin_name", null); setParam(r, "pin_map", null);
      } else {
        const pin = pinList().find((p) => p.id === id);
        setParam(r, "pin_id", id);
        if (pin) { setParam(r, "pin_name", pin.name); setParam(r, "pin_map", loadedMap() || null); }
      }
      const box = body.querySelector(".pin-box");
      if (box) box.innerHTML = pinBoxInner(r);
    }
    else if (f === "policy") {
      const chosen = Array.from(body.querySelectorAll('[data-f="policy"]:checked')).map((c) => c.value);
      ensure(r, "then").policy = chosen;
      if (!chosen.includes("speed_scale")) setParam(r, "scale", null);
      if (!chosen.includes("goto_pin")) { setParam(r, "pin_id", null); setParam(r, "pin_name", null); setParam(r, "pin_map", null); }
      structural = true;   // 併用不可の選択肢の無効化・倍率欄の出し分けのため再描画
    }
    if (!r.when) r.when = {};
    updateSummary(i);
    renderDirty();
    if (structural) renderRules();
  }

  function uniqueId(base) {
    const ids = new Set(draft.rules.map((r) => r.id));
    let n = 1, id = base;
    while (ids.has(id)) { n += 1; id = `${base}_${n}`; }
    return id.slice(0, 64);
  }

  function onRulesClick(ev) {
    const btn = ev.target.closest("[data-act]");
    if (!btn) return;
    const act = btn.getAttribute("data-act");
    const card = btn.closest(".rule");
    if (!card) return;
    const i = Number(card.getAttribute("data-i"));
    const r = draft.rules[i];
    if (act === "edit") {
      if (openCards.has(i)) openCards.delete(i); else openCards.add(i);
      renderRules(); return;
    }
    if (readOnly) return;
    if (act === "tr-add") {
      const w = ensure(r, "when");
      const trs = (w.time_ranges || []).slice();
      trs.push("-");   // 空の行(未入力のまま保存するとサーバーが理由を返す)
      w.time_ranges = trs;
      renderRules(); renderDirty(); return;
    }
    if (act === "tr-del") {
      const k = Number(btn.closest(".tr-row").getAttribute("data-k"));
      const trs = (r.when && r.when.time_ranges) ? r.when.time_ranges.slice() : [];
      trs.splice(k, 1);
      setWhen(r, "time_ranges", trs);
      renderRules(); renderDirty(); return;
    }
    if (act === "dup") {
      const c = clone(r);
      c.id = uniqueId((r.id || "rule") + "_copy");
      c.name = (r.name || r.id || "") + "(コピー)";
      c.enabled = false;
      draft.rules.splice(i + 1, 0, c);
      openCards.clear(); openCards.add(i + 1); ruleErrors = {};
      renderRules(); renderDirty(); return;
    }
    if (act === "del") {
      if (!confirm(`ルール「${r.name || r.id}」を削除しますか？(「保存して反映」を押すまで確定しません)`)) return;
      draft.rules.splice(i, 1);
      openCards.clear(); ruleErrors = {};
      renderRules(); renderDirty(); return;
    }
  }

  function onRulesToggle(ev) {
    const el = ev.target;
    if (el.getAttribute && el.getAttribute("data-act") === "toggle") {
      if (readOnly) { el.checked = !el.checked; return; }
      const i = Number(el.closest(".rule").getAttribute("data-i"));
      draft.rules[i].enabled = el.checked;
      el.parentElement.lastChild.textContent = el.checked ? "有効" : "無効";
      updateSummary(i);
      renderDirty();
    }
  }

  // ---------------------------------------------------------------- 保存など
  function setBusy(v) {
    busy = v; applyReadOnly();
    for (const id of ["rulesList", "minHold", "docDesc"]) { const el = $(id); if (el) el.inert = v; }  // 保存中は編集不可
  }
  // 保存した内容と読み直した内容の要点(件数・id・有効/無効・優先度・最小保持時間)が一致するか
  function docKey(d) {
    return JSON.stringify([d.min_hold_seconds, (d.rules || []).map((r) => [r.id, r.enabled === true, r.priority ?? 0])]);
  }
  // 保存後、behavior_directorが保存したルールを読み込んだか(ルール件数・有効件数)を数秒見て確かめる
  async function verifyDirector(sent) {
    const total = (sent.rules || []).length, on = (sent.rules || []).filter((r) => r.enabled === true).length;
    let last = null;
    for (const w of [800, 1200, 2000, 3000]) {
      await new Promise((r) => setTimeout(r, w));
      try {
        const r = await api("/api/behavior_rules/state");
        if (r.ok) { last = r.data; renderLive(r.data); }
      } catch (e) { /* 次の確認で再試行 */ }
      const st = last && last.state;
      if (last && last.status === "ok" && st && Number(st.rule_count) === total && Number(st.enabled_rule_count) === on) {
        $("saveStatus").textContent = "directorが保存したルールを読み込みました(" + on + " / " + total + " 件が有効)";
        return;
      }
    }
    const st = last && last.state;
    $("saveStatus").textContent = (last && last.status === "ok" && st)
      ? "directorはまだ保存したルールを読み込んでいません(director側 有効" + st.enabled_rule_count + " / " + st.rule_count + "件、保存した内容 有効" + on + " / " + total + "件)。続く場合はルールファイルのパス設定を確認してください"
      : "behavior_directorから状態が届かないため、反映を確認できません(起動したときに保存済みのルールを読み込みます)";
  }

  function mapRuleErrors(errors) {
    const map = {};
    (errors || []).forEach((e) => {
      const m = /^rules\[(\d+)\]/.exec(String(e));
      if (m) { (map[Number(m[1])] = map[Number(m[1])] || []).push(e); }
    });
    return map;
  }

  async function save() {
    if (readOnly || busy) return;
    $("saveStatus").textContent = "保存しています...";
    setBusy(true);
    try {
      const r = await api("/api/behavior_rules/save", {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ document: draft, expected_sig: serverSig }),
      });
      if (r.ok && r.data.ok) {
        $("saveStatus").textContent = "";
        // 楽観的に表示せず、サーバーに保存された内容を読み直して表示する(送った内容と違えば警告する)
        const sentKey = docKey(draft);
        NKField.bump("rules");
        const ok = await loadRules(true);
        const warns = Array.isArray(r.data.warnings) ? r.data.warnings : [];
        if (ok && serverDoc && docKey(serverDoc) !== sentKey) warns.push("保存した内容と読み直した内容が一致しません(サーバーで調整された可能性があります)。画面は保存済みの内容です。");
        showGlobal(warns.length ? "warn" : "ok", (r.data.message || "保存しました。") + (ok ? "" : "(画面の再読み込みに失敗しました。ページを再読み込みしてください)") + (warns.length ? " 注意: " + warns.join(" ") : ""));
        loadPreview();
        pollLive();
        if (ok && serverDoc) verifyDirector(serverDoc);
      } else {
        const errs = r.data.errors || [r.data.message || r.data.error || `保存に失敗しました(HTTP ${r.status})`];
        ruleErrors = mapRuleErrors(errs);
        const title = r.status === 409 ? "保存しませんでした(競合)" : (r.status === 403 ? "保存できません(閲覧専用アカウント)" : "保存しませんでした。次の点を直してください(何も書き込んでいません)");
        $("saveErrors").innerHTML = errorsHtml(title, errs);
        $("saveStatus").textContent = "";
        renderRules();
        $("saveErrors").scrollIntoView({ behavior: "smooth", block: "nearest" });
      }
    } catch (e) {
      $("saveErrors").innerHTML = errorsHtml("保存に失敗しました", ["通信エラー(サーバーに接続できません)。何も書き込まれていない可能性が高いです。読み直して確認してください。"]);
      $("saveStatus").textContent = "";
    } finally {
      setBusy(false);
    }
  }

  async function loadExample() {
    if (readOnly || busy) return;
    if (isDirty() && !confirm("未保存の変更は例の内容で置き換わります。よろしいですか？")) return;
    else if (!isDirty() && draft.rules.length && !confirm("今のルールの代わりに、例(すべて無効)を編集画面に読み込みます。保存するまで反映されません。よろしいですか？")) return;
    try {
      const r = await api("/api/behavior_rules/example");
      if (!r.ok || !r.data.ok) {
        showGlobal("err", "例を読み込めませんでした: " + ((r.data.errors || [])[0] || `HTTP ${r.status}`));
        return;
      }
      draft = clone(r.data.document);
      if (!Array.isArray(draft.rules)) draft.rules = [];
      openCards.clear(); ruleErrors = {};
      $("saveErrors").innerHTML = "";
      showGlobal("info", "例を編集画面に読み込みました(すべて無効です)。使うルールを有効にして「保存して反映」を押してください。");
      renderAll();
    } catch (e) {
      showGlobal("err", "例を読み込めませんでした(通信エラー)");
    }
  }

  async function notifyReload() {
    if (readOnly || busy) return;
    try {
      const r = await api("/api/behavior_rules/reload", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" });
      showGlobal(r.ok && r.data.ok ? "ok" : "err", r.data.message || `失敗(HTTP ${r.status})`);
    } catch (e) {
      showGlobal("err", "通知に失敗しました(通信エラー)");
    }
  }

  // ---------------------------------------------------------------- 次の24時間の予定
  function ruleBrief(x) {
    const params = x.params && Object.keys(x.params).length ? " " + JSON.stringify(x.params) : "";
    return `${escapeHtml(x.name || x.id)} <code>${escapeHtml(x.policy)}${escapeHtml(params)}</code>`;
  }

  async function loadPreview() {
    $("previewBody").textContent = "計算中...";
    try {
      const r = await api("/api/behavior_rules/preview");
      if (!r.ok || !r.data.ok) {
        $("previewBody").innerHTML = errorsHtml("予定を計算できません", r.data.errors || [`HTTP ${r.status}`]);
        return;
      }
      const d = r.data;
      $("previewNow").textContent = `基準時刻 ${fmtTime(d.now)} から${d.hours}時間 / 有効なルール ${d.enabled_rule_count} 件`;
      if (!d.enabled_rule_count) {
        $("previewBody").innerHTML = `<div class="banner info">有効なルールがありません。24時間ずっと現行動作です。</div>`;
        return;
      }
      const rows = d.segments.map((s) => {
        const sure = s.definite ? `<b>${ruleBrief(s.definite)}</b>` : `<span class="hint" style="margin:0;">ルールなし(現行動作)</span>`;
        const maybe = (s.possible || []).map((p) =>
          `<div><span class="chip maybe">条件次第</span>${ruleBrief(p)}<small style="color:#53635c;"> ← ${escapeHtml(p.when)}</small></div>`).join("");
        return `<tr><td style="white-space:nowrap;">${escapeHtml(fmtTime(s.start))} 〜 ${escapeHtml(fmtTime(s.end))}</td><td>${sure}</td><td>${maybe || "-"}</td></tr>`;
      }).join("");
      $("previewBody").innerHTML = `<table><thead><tr><th>時間帯</th><th>時刻だけで決まる方針</th><th>状況しだいで上書きされうるルール</th></tr></thead><tbody>${rows}</tbody></table>
        <p class="hint">最小保持時間(${escapeHtml(d.min_hold_seconds)}秒)は本番と同じ計算に含めています。</p>`;
    } catch (e) {
      $("previewBody").innerHTML = errorsHtml("予定を取得できませんでした", ["通信エラー(サーバーに接続できません)"]);
    }
  }

  // ---------------------------------------------------------------- 緊急停止ドック(メイン画面と同じ動作)
  let estopEngaged = false;
  let estopPending = false;
  function renderEstop(engaged) {
    estopEngaged = engaged;
    $("estopBtn").textContent = engaged ? "緊急停止を解除" : "緊急停止(ソフト)";
    $("estopBtn").classList.toggle("engaged", engaged);
  }
  async function pollEstop() {
    if (document.hidden || estopPending) return;
    try {
      const r = await api("/api/state");
      if (r.ok && typeof r.data.soft_estop_engaged === "boolean") renderEstop(r.data.soft_estop_engaged);
    } catch (e) { /* 次のポーリングで再試行 */ }
  }
  async function setEstop(engaged) {
    if (readOnly || estopPending) return;
    estopPending = true;
    $("estopBtn").disabled = true;
    const prev = estopEngaged;
    renderEstop(engaged);
    let ok = false;
    try {
      const r = await api("/api/soft_estop", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ engaged }) });
      ok = r.ok;
    } catch (e) { ok = false; }
    estopPending = false;
    $("estopBtn").disabled = readOnly;
    if (!ok) {
      renderEstop(prev);
      if (engaged) alert("緊急停止の送信に失敗しました。機体は止まっていない可能性があります。物理の非常停止を使ってください。");
      else showGlobal("err", "緊急停止の解除に失敗しました(緊急停止中のままです)。");
    }
  }
  $("estopBtn").addEventListener("click", async () => {
    if (estopEngaged) { if (!confirm("本当に解除しますか？")) return; await setEstop(false); }
    else await setEstop(true);
  });

  // ---------------------------------------------------------------- 起動
  NKField.guard(["minHold", "docDesc"]);   // 全体設定の入力欄(範囲外の値を入力時に表示。値は下書きが持つのでポーリングでは書き換わらない)
  NKField.watch("rulesList");              // ルール編集欄(動的に作る入力欄の範囲外表示・編集中の判定)
  $("rulesList").addEventListener("click", onRulesClick);
  $("rulesList").addEventListener("change", (ev) => { onRulesToggle(ev); onFieldChange(ev); });
  $("minHold").addEventListener("change", () => { if (readOnly) return; const n = toNum($("minHold").value); draft.min_hold_seconds = n === null ? 10 : n; renderDirty(); });
  $("docDesc").addEventListener("change", () => { if (readOnly) return; draft.description = $("docDesc").value; renderDirty(); });
  $("addBtn").addEventListener("click", () => {
    if (readOnly) return;
    draft.rules.push({ id: uniqueId("rule"), enabled: false, name: "新しいルール", priority: 0, when: {}, then: { policy: ["normal"], params: {} } });
    openCards.clear(); openCards.add(draft.rules.length - 1); ruleErrors = {};
    renderRules(); renderDirty();
  });
  $("exampleBtn").addEventListener("click", loadExample);
  $("saveBtn").addEventListener("click", save);
  $("discardBtn").addEventListener("click", async () => {
    if (isDirty() && !confirm("未保存の変更を破棄して、保存済みの内容に戻しますか？")) return;
    await loadRules();
  });
  $("reloadFileBtn").addEventListener("click", async () => {
    if (isDirty() && !confirm("未保存の変更は失われます。保存済みの内容を読み直しますか？")) return;
    await loadRules();
  });
  $("notifyBtn").addEventListener("click", notifyReload);
  $("previewBtn").addEventListener("click", loadPreview);
  window.addEventListener("beforeunload", (e) => { if (isDirty()) { e.preventDefault(); e.returnValue = ""; } });
  document.addEventListener("visibilitychange", () => { if (!document.hidden) { pollLive(); pollEstop(); } });

  (async function init() {
    try {
      const w = await api("/api/whoami");
      const account = (w.data && w.data.account) || "";
      $("accountLabel").textContent = account ? `ログイン: ${account}` : "";
      readOnly = !w.ok || account === "guest" || account === "";
    } catch (e) {
      readOnly = true;
    }
    if (readOnly) {
      showGlobal("info", "閲覧専用です(guestアカウント、または認証を確認できませんでした)。内容は見られますが、編集・保存はできません。");
      $("estopBtn").disabled = true;
    }
    await loadPins();
    await loadRules(true);
    loadPreview();
    pollLive();
    pollEstop();
    NKField.poll(loadPins, 5000);
    NKField.poll(pollLive, 2000);
    setInterval(pollEstop, 2000);
  })();
</script>
</body>
</html>
"""


def build_behavior_rules_html() -> bytes:
    """時間帯・状況ルールの編集ページ(`/behavior_rules`)。full_dashboard_nodeから配信される。"""
    html = inject_page_guide(_BEHAVIOR_RULES_HTML, PAGE_RULES)  # 【2026-10-23】3ページの説明バナー+機体の時計(足すだけ)
    return html.replace("__NK_FIELD_JS__", NK_FIELD_JS).encode("utf-8")
