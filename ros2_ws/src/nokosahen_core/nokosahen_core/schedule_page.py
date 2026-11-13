"""かんたん時間割(`/schedule`)のページのHTML。

【2026-10-23追加】完成版ダッシュボード(full_dashboard_node)から配信される。
「何時何分にどこ(地図のピン)へ行き、どの状態になるか」を表で並べ、「このスケジュールで動け」で実行する。
ページ自体はROSに触れず、HTTP API(下のAPI_*定数)だけを使う。behavior_tree_page.py と同じ流儀。
APIの実装はバックエンド側(別担当)。このページは契約どおりの形で呼ぶだけで、無いキーがあっても壊れない。

設計メモ:
- 変更は「下書き」(未保存)としてブラウザ内に持ち、「保存」でPOSTする。実行は保存済みのスケジュールで動く
  (未保存の変更があるときは、保存してから実行するか確認する)。
- 日本語のプレビュー(summary)は /api/schedule/preview の結果をそのまま表示する(ページ側で文章を作らない)。
- 「動ける範囲」: そのまま(keep) / 指定(set。座標の矩形。ダッシュボードの現在の範囲を取り込める) /
  解除してNav2で走行(release)。本番は範囲の角にコーンを置く想定(将来コーンから範囲を決める拡張の余地。今回は未実装)。
- 【2026-10-23追加】地図パネル(GET /api/map。範囲を地図上のドラッグで指定、ピンのクリックで行き先指定)。
  座標変換はmapWorldToScreen/mapScreenToWorldの1か所(PURE-BEGIN〜ENDの純粋関数はテストがnodeで数値検証する)。
- 全ての可変テキストは escapeHtml() を通してから innerHTML に入れる。
- guestアカウントは閲覧のみ(入力欄・ボタンを無効化。サーバー側もPOSTは403)。
"""

from .page_guide import PAGE_SCHEDULE, inject_page_guide

# UIが叩くAPIパス(テストでHTMLに含まれることを確認する)
API_SCHEDULE = "/api/schedule"
API_SCHEDULE_PREVIEW = "/api/schedule/preview"
API_SCHEDULE_RUN = "/api/schedule/run"
API_SCHEDULE_STOP = "/api/schedule/stop"
API_PINS = "/api/pins"
API_BOUNDARY = "/api/boundary"
API_MAP = "/api/map"

_SCHEDULE_HTML = r"""<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>のこさへん - かんたん時間割</title>
<style>
  :root { color-scheme: light; --bg:#f4f6f5; --fg:#16211d; --card:#fff; --line:#dce4df; --head:#124739; --accent:#176b56;
          --muted:#53635c; --soft:#eef3f0; --err-bg:#faece7; --err-fg:#993c1d; --ok-bg:#e1f5ee; --ok-fg:#0f6e56;
          --warn-bg:#fff3cd; --warn-fg:#7a5b00; --danger:#a32d2d;
          font-family: system-ui, -apple-system, "Segoe UI", sans-serif; background:var(--bg); color:var(--fg); }
  html, body { margin:0; }
  body { padding-bottom:84px; }
  header { box-sizing:border-box; min-height:44px; padding:6px 18px; background:var(--head); color:#fff; display:flex; justify-content:space-between; align-items:center; gap:8px; flex-wrap:wrap; }
  header a { color:#cfe9df; text-decoration:none; font-size:13px; }
  header h1 { margin:0; font-size:18px; }
  main { max-width:1000px; margin:0 auto; padding:14px; display:flex; flex-direction:column; gap:14px; }
  section { background:var(--card); border:1px solid var(--line); border-radius:8px; padding:12px 14px; }
  h2 { margin:0 0 8px; font-size:15px; }
  .hint { font-size:13px; color:var(--muted); margin:6px 0 0; }
  .row { display:flex; gap:8px; flex-wrap:wrap; align-items:center; }
  button { padding:8px 12px; font-size:13px; border:0; border-radius:6px; background:var(--accent); color:#fff; cursor:pointer; }
  button.secondary { background:#5d6f67; }
  button.danger { background:var(--danger); }
  button.small { padding:4px 9px; font-size:13px; }
  button.ghost { background:var(--soft); color:var(--fg); }
  button.big { padding:12px 22px; font-size:16px; font-weight:700; }
  button:disabled { opacity:0.45; cursor:not-allowed; }
  input[type="text"], input[type="number"], input[type="time"], select { padding:6px; font-size:13px; border:1px solid #cfd8d3; border-radius:6px; box-sizing:border-box; max-width:100%; background:#fff; color:var(--fg); }
  input:disabled, select:disabled { background:#f1f3f2; color:#555; }
  label { font-size:13px; color:#33463e; }
  .banner { padding:8px 12px; border-radius:6px; font-size:13px; margin-top:6px; }
  .banner.err { background:var(--err-bg); color:var(--err-fg); }
  .banner.ok { background:var(--ok-bg); color:var(--ok-fg); }
  .banner.warn { background:var(--warn-bg); color:var(--warn-fg); }
  .banner.info { background:var(--soft); color:#33463e; }
  .banner ul, .banner ol { margin:4px 0 0 18px; padding:0; }
  .pill { display:inline-block; padding:3px 11px; border-radius:999px; font-size:13px; font-weight:700; }
  .pill.ok { background:var(--ok-bg); color:var(--ok-fg); }
  .pill.bad { background:var(--err-bg); color:var(--err-fg); }
  .pill.wait { background:var(--warn-bg); color:var(--warn-fg); }
  .pill.off { background:#e4e8e6; color:var(--muted); }
  .grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(200px,1fr)); gap:8px; margin-top:8px; }
  .metric { background:var(--soft); border-radius:8px; padding:6px 10px; min-width:0; }
  .metric span { display:block; font-size:13px; color:var(--muted); }
  .metric b { font-size:14px; word-break:break-all; }
  /* 時間割の行 */
  .entry { border:1px solid var(--line); border-left:5px solid var(--accent); border-radius:8px; padding:8px 10px; margin:8px 0; background:#fbfcfb; }
  .entry.bad { border-left-color:var(--danger); background:var(--err-bg); }
  .entry .top { display:flex; gap:6px; flex-wrap:wrap; align-items:center; justify-content:space-between; margin-bottom:6px; }
  .entry .no { font-weight:700; color:var(--head); }
  .entry .acts { display:flex; gap:4px; flex-wrap:wrap; }
  .fields { display:grid; grid-template-columns:repeat(auto-fit,minmax(150px,1fr)); gap:8px 10px; }
  .field { display:flex; flex-direction:column; gap:3px; min-width:0; }
  .field.wide { grid-column:1 / -1; }
  .field select, .field input[type="text"], .field input[type="time"], .field input[type="number"] { width:100%; }
  .wds { display:flex; flex-wrap:wrap; gap:4px 10px; }
  .wds label { display:inline-flex; align-items:center; gap:3px; font-size:13px; color:var(--fg); }
  .rectbox { display:none; margin-top:6px; padding:6px 8px; background:var(--soft); border-radius:6px; }
  .entry[data-bmode="set"] .rectbox { display:block; }
  .rectbox .rg { display:grid; grid-template-columns:repeat(auto-fit,minmax(90px,1fr)); gap:6px; }
  .compact { font-size:13px; color:var(--muted); }
  #estopDock { position:fixed; left:0; right:0; bottom:0; z-index:2000; box-sizing:border-box; padding:8px 12px; background:rgba(18,71,57,0.96); box-shadow:0 -2px 10px rgba(0,0,0,0.3); display:flex; gap:10px; align-items:center; }
  #estopDock .stop-btn { flex:1 1 auto; margin:0; padding:14px; font-size:18px; min-height:52px; border:2px solid #fff; font-weight:700; background:var(--danger); color:#fff; border-radius:8px; }
  #estopDock .stop-btn.engaged { background:#5c1717; }
  #estopDock a { color:#cfe9df; font-size:13px; white-space:nowrap; }
  details summary { cursor:pointer; font-size:13px; color:#33463e; }
  details table { border-collapse:collapse; width:100%; font-size:13px; margin-top:6px; }
  details th, details td { border:1px solid var(--line); padding:4px 6px; text-align:left; vertical-align:top; }
  pre.raw { background:var(--bg); border-radius:6px; padding:8px; font-size:13px; overflow:auto; max-height:200px; }
  /* 地図パネル */
  #mapWrap { position:relative; border:1px solid var(--line); border-radius:6px; overflow:hidden; background:#cfd6d2; }
  #mapCv { display:block; touch-action:none; user-select:none; -webkit-user-select:none; cursor:grab; }
  #mapMsg { display:none; }
  #mapLegend { display:flex; flex-wrap:wrap; gap:4px 12px; margin-top:6px; font-size:13px; color:var(--muted); }
  #mapLegend .lgi { display:inline-flex; align-items:center; gap:4px; }
  #mapLegend i { display:inline-block; width:14px; height:10px; border-radius:2px; }
  #mapLegend i.dot { width:10px; border-radius:50%; }
  #mapLegend i.dash { background:none; border-top:3px dashed #ffb000; height:0; }
  .entry.mapediting { box-shadow:0 0 0 2px #1c7ed6; }
  @media (max-width:640px) { main { padding:8px; } header { padding:6px 10px; } .fields { grid-template-columns:1fr 1fr; } .field.wide { grid-column:1 / -1; } }
__PAGE_GUIDE_CSS__
</style>
</head>
<body>
<header>
  <h1>かんたん時間割</h1>
  <div class="row">
    <span id="accountLabel" style="font-size:13px;opacity:0.85;"></span>
    <a href="/behavior_tree">ビヘイビアツリー</a>
    <a href="/behavior_rules">時間帯・状況ルール</a>
    <a href="/">&larr; 操作盤に戻る</a>
  </div>
</header>
<main>
__PAGE_GUIDE_HTML__
  <div id="globalMsg"></div>

  <section id="runSection">
    <h2>実行</h2>
    <div class="row">
      <span id="runBadge" class="pill wait">確認中...</span>
      <button id="runBtn" class="big" type="button">このスケジュールで動け</button>
      <button id="stopBtn" class="danger big" type="button">停止</button>
    </div>
    <div id="stateAlerts" aria-live="polite"></div>
    <div class="grid">
      <div class="metric"><span>今の項目</span><b id="curItem">-</b></div>
      <div class="metric"><span>次の切替</span><b id="nextSwitch">-</b></div>
      <div class="metric"><span>頭部ディスプレイの画面</span><b id="dispState">-</b></div>
    </div>
    <div class="row" style="margin-top:8px;">
      <label><input type="checkbox" id="runOnStartup" /> ロボットを起動したとき、自動でこのスケジュールを実行する(既定はOFF=実行しない。起動後2分たってから、機体の時計が合っているときだけ始まります)</label>
    </div>
    <p class="hint">「保存」したスケジュールが実行されます。停止すると、時間帯・状況ルール(またはビヘイビアツリー)の動作に戻ります。
      安全層(緊急停止・人が近すぎる・走行範囲外など)はどの動作でも常に最優先です。</p>
    <details style="margin-top:6px;"><summary>機体から届いている状態(詳細)</summary><pre class="raw" id="stateRaw">-</pre></details>
  </section>

  <section>
    <h2>時間割の表</h2>
    <div class="banner info" style="margin-top:0;">
      行ごとに「<b>開始時刻</b>になったら、<b>行き先</b>へ行って、<b>状態</b>になる」を並べます。終了時刻を省略すると次の行の開始まで続きます。
      行き先は地図で作ったピンから選びます(<span id="pinInfo">ピンを確認中...</span>)。
      「動ける範囲」は、その時間に走ってよい範囲です。<b>本番は範囲の角にコーンを置く想定</b>です(将来、コーンから範囲を決められるようにする余地を残しています。今は座標の矩形で指定します)。
    </div>
    <div class="row" style="margin-top:10px;">
      <label>名前 <input type="text" id="schName" maxlength="60" style="width:200px;" /></label>
      <select id="exampleSel">
        <option value="peak">例: 昼のピークはピンで待機→夕方に範囲を解除して入口へ</option>
        <option value="quiet">例: 朝は静かに入口で待機→日中は見回し</option>
      </select>
      <button id="exampleBtn" class="ghost" type="button">例を読み込む</button>
      <button id="reloadBtn" class="ghost" type="button">保存済みを読み直す</button>
    </div>
    <div id="entries"></div>
    <div class="row">
      <button id="addBtn" class="secondary" type="button">+ 行を追加</button>
      <span id="dirtyBadge" class="pill off">変更なし</span>
    </div>
  </section>

  <section id="mapSection">
    <h2>地図で範囲・行き先を指定</h2>
    <div class="banner info" style="margin-top:0;">本番は範囲の角にコーンを置く想定。いまは地図上でドラッグするか、機体を角に動かして座標を確認してください。</div>
    <div id="mapEditInfo" class="compact" style="margin:6px 0;"></div>
    <div id="mapMsg" class="banner warn">地図がありません(地図作成または保存した地図の読み込み後に使えます)。いまは「動ける範囲」の数値入力で指定してください。</div>
    <div class="row" style="margin-bottom:6px;">
      <button id="mapZoomIn" type="button" class="small ghost" disabled>＋ 拡大</button>
      <button id="mapZoomOut" type="button" class="small ghost" disabled>－ 縮小</button>
      <button id="mapFitBtn" type="button" class="small ghost" disabled>全体表示</button>
      <button id="mapClearRectBtn" type="button" class="small ghost" disabled>この行の範囲を消す</button>
      <button id="mapDoneBtn" type="button" class="small secondary" disabled>編集を終える</button>
    </div>
    <div id="mapWrap" style="display:none;"><canvas id="mapCv"></canvas></div>
    <div id="mapLegend"></div>
    <p class="hint">操作: 編集中の行ではドラッグで範囲を描き、角の白い四角でサイズ変更、内側のドラッグで移動できます(数値入力とつながっています)。
      ピンをクリックすると編集中の行の行き先になります。ホイールで拡大縮小、中ボタン(またはスペース+ドラッグ)で移動。スマホは1本指でドラッグ、2本指で移動・拡大縮小です。座標は地図のmap座標(m)です。</p>
  </section>

  <section>
    <h2>日本語のプレビュー</h2>
    <p class="hint" style="margin-top:0;">いま編集中の内容を、サーバーが日本語に直した結果です(保存しなくても確認できます)。</p>
    <div id="previewBody"><div class="banner info">読み込み中...</div></div>
    <div class="row" style="margin-top:8px;">
      <button id="previewBtn" class="ghost" type="button">プレビューを更新</button>
      <button id="saveBtn" type="button">保存</button>
    </div>
    <div id="saveResult"></div>
  </section>

  <section>
    <h2>使い方・3ページの違い</h2>
    <details open><summary>どのページで何を決めるか</summary>
      <table>
        <tr><th>ページ</th><th>決めること</th><th>向いている人・場面</th></tr>
        <tr><td>かんたん時間割(ここ)</td><td>何時に・どこへ行って・どの状態になるか(表)。範囲の指定/解除も</td><td>まず使う。時刻と行き先が決まっているとき</td></tr>
        <tr><td>ビヘイビアツリー</td><td>混雑・人物・残数などの条件で枝分かれする動作(木)</td><td>上級者。条件分岐を細かく組みたいとき</td></tr>
        <tr><td>時間帯・状況ルール</td><td>時間帯や状況の条件に合う「方針」(待機・通常・接近…)</td><td>行き先は決めず、方針だけ条件で切り替えたいとき</td></tr>
      </table>
    </details>
  </section>
</main>

<div id="estopDock">
  <button id="estopBtn" class="stop-btn" type="button">緊急停止(ソフト)</button>
  <a href="/">操作盤へ</a>
</div>

<script>
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
  function clone(o) { return JSON.parse(JSON.stringify(o)); }

  const API_SCHEDULE = "__API_SCHEDULE__";
  const API_PREVIEW = "__API_SCHEDULE_PREVIEW__";
  const API_RUN = "__API_SCHEDULE_RUN__";
  const API_STOP = "__API_SCHEDULE_STOP__";
  const API_PINS = "__API_PINS__";
  const API_BOUNDARY = "__API_BOUNDARY__";
  const API_MAP_PATH = "__API_MAP__";

  const WD = [["mon", "月"], ["tue", "火"], ["wed", "水"], ["thu", "木"], ["fri", "金"], ["sat", "土"], ["sun", "日"]];
  const POLICIES = [["stay", "待機"], ["normal", "通常"], ["approach", "接近"], ["scan", "見回し"], ["face", "人の方を向く"], ["quiet", "静か(呼びかけ控えめ)"]];
  const BMODES = [["keep", "そのまま"], ["set", "指定する(矩形)"], ["release", "解除してNav2で走行"]];
  const DMODES = [["keep", "そのまま"], ["customer", "お客さん向け"], ["survey", "アンケート"], ["ocrtest", "OCRテスト"]];
  const DISPLAY_JA = { customer: "お客さん向け", survey: "アンケート", ocrtest: "OCRテスト" };
  const RECT_KEYS = [["x_min", "x最小"], ["y_min", "y最小"], ["x_max", "x最大"], ["y_max", "y最大"]];

  let readOnly = true;
  let draft = { name: "", run_on_startup: false, entries: [] };   // 編集中(未保存)。entriesは内部表現
  let savedSig = null;           // 保存済み(サーバー)の署名
  let serverSchedule = undefined;
  let pinsInfo = null;
  let running = false;
  let previewSeq = 0;
  let previewTimer = null;
  let busy = false;

  // ---------------------------------------------------------------- 通信
  async function api(path, opts) {
    const res = await fetch(path, opts);
    let data = null;
    try { data = await res.json(); } catch (e) { /* 本文がJSONでない */ }
    return { ok: res.ok, status: res.status, data: data || {} };
  }
  function postJson(path, body) {
    return api(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  }
  function showGlobal(kind, text) {
    $("globalMsg").innerHTML = text ? `<div class="banner ${kind}">${escapeHtml(text)}</div>` : "";
  }
  function listHtml(kind, title, items) {
    if (!items || !items.length) return "";
    return `<div class="banner ${kind}">${escapeHtml(title)}<ul>${items.map((e) => `<li>${escapeHtml(typeof e === "string" ? e : JSON.stringify(e))}</li>`).join("")}</ul></div>`;
  }
  function toNum(v) {
    if (v === null || v === undefined || String(v).trim() === "") return null;
    const n = Number(v);
    return isFinite(n) ? n : null;
  }

  // ---------------------------------------------------------------- ピン
  function pinList() {
    return pinsInfo && Array.isArray(pinsInfo.pins) ? pinsInfo.pins.filter((p) => p && p.id !== undefined) : [];
  }
  function pinName(id, fallback) {
    const p = pinList().find((q) => String(q.id) === String(id));
    return p ? (p.name || String(id)) : (fallback || "");
  }
  async function loadPins() {
    let sig = "";
    try {
      const r = await api(API_PINS);
      if (!r.ok) throw new Error("HTTP " + r.status);
      pinsInfo = r.data;
    } catch (e) { pinsInfo = null; }
    const newSig = JSON.stringify(pinsInfo ? [pinsInfo.loaded_map, pinsInfo.pins] : null);
    if (newSig !== loadPins.sig) {
      loadPins.sig = newSig;
      $("pinInfo").textContent = pinsInfo
        ? (pinList().length ? `地図「${pinsInfo.loaded_map || "?"}」のピン ${pinList().length} 個から選べます` : "ピンがありません。地図で先にピンを作ってください")
        : "ピン一覧を取得できませんでした";
      if (!isEditingEntries()) renderEntries();
      mapRefresh();
    }
  }
  loadPins.sig = null;

  // ---------------------------------------------------------------- 内部表現 <-> API
  let idSeq = 0;
  function newId() {
    const used = new Set(draft.entries.map((e) => e.id));
    do { idSeq += 1; } while (used.has("e" + idSeq));
    return "e" + idSeq;
  }
  function emptyEntry() {
    return { id: newId(), start: "09:00", end: "", weekdays: [], dest: "stay", destName: "", policy: "normal",
             announce: "", speed: "", bmode: "keep", rect: { x_min: "", y_min: "", x_max: "", y_max: "" }, display: "keep", note: "" };
  }
  function entryFromApi(e) {
    e = e || {};
    const d = e.destination || {};
    const st = e.state || {};
    const b = e.boundary || {};
    const r = b.rect || {};
    return {
      id: String(e.id || ""), start: e.start || "", end: e.end || "",
      weekdays: Array.isArray(e.weekdays) ? e.weekdays.slice() : [],
      dest: d.type === "pin" ? "pin:" + d.pin_id : (d.type === "entrance" ? "entrance" : "stay"),
      destName: d.pin_name || "",
      policy: st.policy || "normal",
      announce: st.announce_interval_seconds === undefined || st.announce_interval_seconds === null ? "" : String(st.announce_interval_seconds),
      speed: st.speed_scale === undefined || st.speed_scale === null ? "" : String(st.speed_scale),
      bmode: b.mode || "keep",
      rect: { x_min: r.x_min ?? "", y_min: r.y_min ?? "", x_max: r.x_max ?? "", y_max: r.y_max ?? "" },
      display: (e.display && typeof e.display === "object" && e.display.mode) || "keep",
      note: e.note || ""
    };
  }
  function entryToApi(en) {
    const o = { id: en.id, start: en.start };
    if (en.end) o.end = en.end;
    if (en.weekdays.length) o.weekdays = WD.map((w) => w[0]).filter((k) => en.weekdays.includes(k));
    if (en.dest.startsWith("pin:")) {
      const id = en.dest.slice(4);
      o.destination = { type: "pin", pin_id: id, pin_name: pinName(id, en.destName) };
    } else o.destination = { type: en.dest === "entrance" ? "entrance" : "stay" };
    const st = { policy: en.policy };
    const a = toNum(en.announce); if (a !== null) st.announce_interval_seconds = a;
    const s = toNum(en.speed); if (s !== null) st.speed_scale = s;
    o.state = st;
    const b = { mode: en.bmode };
    if (en.bmode === "set") {
      const rect = {};
      for (const [k] of RECT_KEYS) { const n = toNum(en.rect[k]); if (n !== null) rect[k] = n; }
      b.rect = rect;
    }
    o.boundary = b;
    if (en.display && en.display !== "keep") o.display = { mode: en.display };   // keep(そのまま)は省略=何も変えない
    o.note = en.note || "";
    return o;
  }
  function draftToApi() {
    return { version: 1, name: draft.name || "", run_on_startup: !!draft.run_on_startup, entries: draft.entries.map(entryToApi) };
  }
  function scheduleToDraft(s) {
    if (!s || typeof s !== "object") return { name: "マイ時間割", run_on_startup: false, entries: [] };
    const d = { name: s.name || "", run_on_startup: !!s.run_on_startup, entries: (Array.isArray(s.entries) ? s.entries : []).map(entryFromApi) };
    const used = new Set(d.entries.map((e) => e.id).filter((x) => x));
    let n = 0;
    d.entries.forEach((e) => { if (!e.id) { do { n += 1; } while (used.has("e" + n)); e.id = "e" + n; used.add(e.id); } });
    return d;
  }
  function sigOf(s) { return JSON.stringify(s); }
  function isDirty() { return savedSig !== null && sigOf(draftToApi()) !== savedSig; }
  function renderDirty() {
    const d = savedSig === null ? false : isDirty();
    $("dirtyBadge").textContent = savedSig === null ? "未保存(まだ保存されていません)" : (d ? "未保存の変更あり" : "変更なし(保存済み)");
    $("dirtyBadge").className = "pill " + ((d || savedSig === null) ? "wait" : "off");
  }

  // ---------------------------------------------------------------- 行の描画
  function opt(v, label, sel) { return `<option value="${escapeHtml(v)}"${sel ? " selected" : ""}>${escapeHtml(label)}</option>`; }
  function destOptions(en) {
    let h = opt("stay", "その場で待つ", en.dest === "stay") + opt("entrance", "入口", en.dest === "entrance");
    const pins = pinList();
    let found = false;
    for (const p of pins) {
      const v = "pin:" + p.id;
      if (v === en.dest) found = true;
      h += opt(v, "ピン: " + (p.name || p.id), v === en.dest);
    }
    if (en.dest.startsWith("pin:") && !found) h += opt(en.dest, "(見つかりません) " + (en.destName || en.dest.slice(4)), true);
    return h;
  }
  function entryHtml(en, i, n) {
    const wds = WD.map(([k, l]) => `<label><input type="checkbox" data-f="wd" data-k="${k}"${en.weekdays.includes(k) ? " checked" : ""} />${l}</label>`).join("");
    const rect = RECT_KEYS.map(([k, l]) => `<div class="field"><label>${l}(m)</label><input type="number" step="0.01" data-f="rect" data-k="${k}" value="${escapeHtml(en.rect[k])}" /></div>`).join("");
    return `<div class="entry" data-idx="${i}" data-bmode="${escapeHtml(en.bmode)}">
      <div class="top"><span class="no">${i + 1}行目</span>
        <span class="acts">
          <button type="button" class="small ghost" data-act="up"${i === 0 ? " disabled" : ""}>↑</button>
          <button type="button" class="small ghost" data-act="down"${i === n - 1 ? " disabled" : ""}>↓</button>
          <button type="button" class="small ghost" data-act="dup">複製</button>
          <button type="button" class="small danger" data-act="del">削除</button>
        </span></div>
      <div class="fields">
        <div class="field"><label>開始時刻</label><input type="time" data-f="start" value="${escapeHtml(en.start)}" /></div>
        <div class="field"><label>終了時刻(省略=次の行まで)</label><input type="time" data-f="end" value="${escapeHtml(en.end)}" /></div>
        <div class="field wide"><label>曜日(未選択=毎日)</label><div class="wds">${wds}</div></div>
        <div class="field"><label>行き先</label><select data-f="dest">${destOptions(en)}</select></div>
        <div class="field"><label>状態</label><select data-f="policy">${POLICIES.map(([v, l]) => opt(v, l, en.policy === v)).join("")}</select></div>
        <div class="field"><label>呼びかけ間隔(秒・10〜600、空=既定)</label><input type="number" min="10" max="600" step="1" data-f="announce" value="${escapeHtml(en.announce)}" /></div>
        <div class="field"><label>速度倍率(0.3〜1.0、空=既定)</label><input type="number" min="0.3" max="1" step="0.05" data-f="speed" value="${escapeHtml(en.speed)}" /></div>
        <div class="field wide"><label>動ける範囲</label>
          <select data-f="bmode">${BMODES.map(([v, l]) => opt(v, l, en.bmode === v)).join("")}</select>
          <div class="rectbox">
            <div class="rg">${rect}</div>
            <div class="row" style="margin-top:6px;"><button type="button" class="small ghost" data-act="grab">ダッシュボードの現在の範囲を取り込む</button>
              <button type="button" class="small" data-act="mapedit">地図で指定</button>
              <span class="compact">「地図で指定」で下の地図からドラッグして指定できます。操作盤の「走行範囲を設定」で作った範囲も取り込めます。斜めの範囲は外接矩形になります。本番は範囲の角にコーンを置く想定です。</span></div>
          </div>
          <span class="compact">「解除してNav2で走行」= その時刻に範囲の制限を外し、目的地までNav2(SLAM)で走ります。</span>
        </div>
        <div class="field wide"><label>頭部ディスプレイの画面</label>
          <select data-f="display">${DMODES.map(([v, l]) => opt(v, l, en.display === v)).join("")}</select>
          <span class="compact">「そのまま」=画面を変えません。この時間になったら指定の画面に切り替え、項目が終わると元の画面に戻ります。アンケートは回答中ロボットが止まり、回答中は切替を待ちます(回答が消えないように、終わってから切り替えます)。</span>
        </div>
        <div class="field wide"><label>メモ</label><input type="text" data-f="note" maxlength="120" value="${escapeHtml(en.note)}" /></div>
      </div></div>`;
  }
  function isEditingEntries() {
    const a = document.activeElement;
    return !!(a && $("entries").contains(a) && (a.tagName === "INPUT" || a.tagName === "SELECT"));
  }
  function renderEntries() {
    const n = draft.entries.length;
    $("entries").innerHTML = n
      ? draft.entries.map((en, i) => entryHtml(en, i, n)).join("")
      : `<div class="banner info">まだ行がありません。「+ 行を追加」か「例を読み込む」から始めてください。</div>`;
    applyReadOnly();
    mapRefresh();
  }
  function applyReadOnly() {
    document.querySelectorAll("#entries input, #entries select, #entries button").forEach((el) => { if (readOnly) el.disabled = true; });
    ["addBtn", "exampleBtn", "saveBtn", "runBtn", "stopBtn", "runOnStartup", "schName"].forEach((id) => { if (readOnly) $(id).disabled = true; });
  }
  function afterEdit(rerender) {
    if (rerender) renderEntries();
    renderDirty();
    schedulePreview();
    mapRefresh();
  }

  // ---------------------------------------------------------------- 行の編集(イベント委譲)
  function entryOf(el) {
    const card = el.closest(".entry");
    return card ? { card, idx: Number(card.dataset.idx), en: draft.entries[Number(card.dataset.idx)] } : null;
  }
  function onFieldInput(ev) {
    if (readOnly) return;
    const el = ev.target;
    const f = el.dataset && el.dataset.f;
    if (!f) return;
    const hit = entryOf(el);
    if (!hit || !hit.en) return;
    const en = hit.en;
    if (f === "wd") {
      const k = el.dataset.k;
      en.weekdays = en.weekdays.filter((x) => x !== k);
      if (el.checked) en.weekdays.push(k);
    } else if (f === "rect") en.rect[el.dataset.k] = el.value;
    else if (f === "dest") { en.dest = el.value; en.destName = el.value.startsWith("pin:") ? pinName(el.value.slice(4), en.destName) : ""; }
    else if (f === "bmode") { en.bmode = el.value; hit.card.dataset.bmode = el.value; }
    else if (f in en) en[f] = el.value;
    afterEdit(false);
  }
  async function grabBoundary(card, en) {
    const r = await api(API_BOUNDARY);
    const d = r.data || {};
    if (!r.ok || !d.enabled || ![d.min_x, d.max_x, d.min_y, d.max_y].every((v) => typeof v === "number")) {
      showGlobal("warn", "ダッシュボードに有効な走行範囲がありません。操作盤の「走行範囲を設定」で先に範囲を作るか、座標を直接入力してください。");
      return;
    }
    en.rect = { x_min: d.min_x, y_min: d.min_y, x_max: d.max_x, y_max: d.max_y };
    showGlobal("ok", "ダッシュボードの現在の範囲を取り込みました。");
    afterEdit(true);
  }
  function onEntryClick(ev) {
    if (readOnly) return;
    const b = ev.target.closest("button[data-act]");
    if (!b) return;
    const hit = entryOf(b);
    if (!hit) return;
    const { idx, en, card } = hit;
    const act = b.dataset.act;
    if (act === "up" && idx > 0) { [draft.entries[idx - 1], draft.entries[idx]] = [draft.entries[idx], draft.entries[idx - 1]]; afterEdit(true); }
    else if (act === "down" && idx < draft.entries.length - 1) { [draft.entries[idx + 1], draft.entries[idx]] = [draft.entries[idx], draft.entries[idx + 1]]; afterEdit(true); }
    else if (act === "dup") { const c = clone(en); c.id = newId(); draft.entries.splice(idx + 1, 0, c); afterEdit(true); }
    else if (act === "del") { if (confirm(`${idx + 1}行目を削除しますか？`)) { draft.entries.splice(idx, 1); afterEdit(true); } }
    else if (act === "grab") grabBoundary(card, en);
    else if (act === "mapedit") mapStartEdit(en);
  }
  $("entries").addEventListener("input", onFieldInput);
  $("entries").addEventListener("change", onFieldInput);
  $("entries").addEventListener("click", onEntryClick);
  $("addBtn").addEventListener("click", () => {
    if (readOnly) return;
    const last = draft.entries[draft.entries.length - 1];
    const e = emptyEntry();
    if (last && last.start) e.start = last.end || last.start;
    draft.entries.push(e);
    afterEdit(true);
  });
  $("schName").addEventListener("input", () => { if (readOnly) return; draft.name = $("schName").value; afterEdit(false); });
  $("runOnStartup").addEventListener("change", () => { if (readOnly) return; draft.run_on_startup = $("runOnStartup").checked; afterEdit(false); });

  // ---------------------------------------------------------------- 例
  function exampleDraft(kind) {
    const pins = pinList();
    const A = pins.length ? pins[0] : null;
    const mk = (o) => Object.assign(emptyEntry(), o);
    const pinDest = A ? "pin:" + A.id : "stay";
    const pinNm = A ? (A.name || String(A.id)) : "";
    if (kind === "quiet") {
      return { name: "朝は静か・日中は見回し", run_on_startup: false, entries: [
        mk({ id: "e1", start: "09:00", end: "11:00", dest: "entrance", policy: "quiet", display: "customer", note: "開店直後は静かに入口で待機(ラズパイはお客さん向け)" }),
        mk({ id: "e2", start: "11:00", end: "17:00", dest: "stay", policy: "scan", speed: "0.7", note: "日中は見回し" })
      ] };
    }
    return { name: "昼ピークと夕方の切替(例)", run_on_startup: false, entries: [
      mk({ id: "e1", start: "11:30", end: "13:30", dest: pinDest, destName: pinNm, policy: "stay", display: "customer", note: A ? "昼のピークはピン「" + pinNm + "」で待機" : "昼のピークは待機(ピンが無いのでその場)" }),
      mk({ id: "e2", start: "13:30", end: "17:00", dest: "stay", policy: "normal", speed: "0.7", note: "通常" }),
      mk({ id: "e3", start: "17:00", end: "", dest: "entrance", policy: "approach", bmode: "release", note: "夕方は範囲を解除して入口へ(Nav2で走行)" })
    ] };
  }
  $("exampleBtn").addEventListener("click", () => {
    if (readOnly) return;
    if (isDirty() && !confirm("未保存の変更は失われます。例を読み込みますか？(読み込んでも保存するまでロボットの動作は変わりません)")) return;
    draft = exampleDraft($("exampleSel").value);
    syncTop();
    showGlobal("info", "例を下書きに読み込みました。ピンは実際のピンに選び直してください。保存するまで動作は変わりません。");
    afterEdit(true);
  });

  // ---------------------------------------------------------------- プレビュー・保存
  function schedulePreview() {
    clearTimeout(previewTimer);
    previewTimer = setTimeout(runPreview, 600);
  }
  function renderResult(target, r, withSummaryTitle) {
    const d = r.data || {};
    let h = "";
    const errs = Array.isArray(d.errors) ? d.errors : [];
    const warns = Array.isArray(d.warnings) ? d.warnings : [];
    const sum = Array.isArray(d.summary) ? d.summary : [];
    if (errs.length) h += listHtml("err", "直す必要があります", errs);
    else if (!r.ok && !sum.length) h += `<div class="banner err">プレビューを取得できませんでした(HTTP ${escapeHtml(r.status)})。</div>`;
    if (warns.length) h += listHtml("warn", "注意", warns);
    if (sum.length) h += `<div class="banner ${errs.length ? "info" : "ok"}">${escapeHtml(withSummaryTitle)}<ol>${sum.map((s) => `<li>${escapeHtml(s)}</li>`).join("")}</ol></div>`;
    else if (!errs.length && r.ok) h += `<div class="banner info">表示できる内容がありません(行を追加してください)。</div>`;
    target.innerHTML = h;
  }
  async function runPreview() {
    const seq = ++previewSeq;
    let r;
    try { r = await postJson(API_PREVIEW, { schedule: draftToApi() }); } catch (e) { r = { ok: false, status: 0, data: {} }; }
    if (seq !== previewSeq) return;
    renderResult($("previewBody"), r, "このスケジュールだとこう動きます");
    markBadEntries(r.data && r.data.errors);
  }
  function markBadEntries(errs) {
    const cards = document.querySelectorAll("#entries .entry");
    cards.forEach((c, i) => {
      const id = draft.entries[i] ? draft.entries[i].id : "";
      const bad = Array.isArray(errs) && errs.some((e) => {
        const t = typeof e === "string" ? e : JSON.stringify(e);
        return (id && t.indexOf(id) >= 0) || t.indexOf(`${i + 1}行目`) >= 0 || t.indexOf(`entries[${i}]`) >= 0;
      });
      c.classList.toggle("bad", bad);
    });
  }
  async function save() {
    if (readOnly || busy) return false;
    busy = true; $("saveBtn").disabled = true;
    let ok = false;
    try {
      const r = await postJson(API_SCHEDULE, { schedule: draftToApi() });
      renderResult($("saveResult"), r, r.data && r.data.ok ? "保存しました" : "保存できませんでした");
      markBadEntries(r.data && r.data.errors);
      if (r.ok && r.data.ok) { savedSig = sigOf(draftToApi()); ok = true; showGlobal("ok", "保存しました。「このスケジュールで動け」で実行できます。"); }
      else showGlobal("err", "保存できませんでした。プレビューの赤い内容を直してください。");
    } catch (e) { showGlobal("err", "保存の通信に失敗しました。"); }
    busy = false; $("saveBtn").disabled = readOnly;
    renderDirty();
    return ok;
  }
  $("saveBtn").addEventListener("click", save);
  $("previewBtn").addEventListener("click", runPreview);

  // ---------------------------------------------------------------- 実行・停止・状態
  $("runBtn").addEventListener("click", async () => {
    if (readOnly || busy) return;
    if (isDirty() || savedSig === null) {
      if (!confirm("未保存の変更があります。保存してから実行しますか？(実行されるのは保存したスケジュールです)")) return;
      if (!(await save())) return;
    }
    busy = true; $("runBtn").disabled = true;
    try {
      const r = await postJson(API_RUN, {});
      showGlobal(r.ok && r.data.ok ? "ok" : "err", r.data.message || (r.ok && r.data.ok ? "実行を開始しました。" : "実行できませんでした。"));
    } catch (e) { showGlobal("err", "実行の通信に失敗しました。"); }
    busy = false; $("runBtn").disabled = readOnly;
    pollStatus();
  });
  $("stopBtn").addEventListener("click", async () => {
    if (readOnly) return;
    try {
      const r = await postJson(API_STOP, {});
      showGlobal(r.ok && r.data.ok ? "ok" : "err", r.data.message || (r.ok && r.data.ok ? "停止しました。" : "停止できませんでした。"));
    } catch (e) { showGlobal("err", "停止の通信に失敗しました。"); }
    pollStatus();
  });
  function pick(obj, keys) {
    if (!obj || typeof obj !== "object") return undefined;
    for (const k of keys) if (obj[k] !== undefined && obj[k] !== null && obj[k] !== "") return obj[k];
    return undefined;
  }
  function fmtVal(v) {
    if (v === undefined || v === null || v === "") return "-";
    if (typeof v === "number") return v > 1e9 ? new Date(v * 1000).toLocaleString("ja-JP") : String(v);
    if (typeof v === "string") return v;
    if (typeof v === "object") {
      const t = pick(v, ["label", "summary", "text", "time", "at", "iso", "start", "id"]);
      const base = t !== undefined && typeof t !== "object" ? fmtVal(t) : JSON.stringify(v);
      const extra = pick(v, ["note", "destination_name"]);
      return extra ? base + "(" + extra + ")" : base;
    }
    return String(v);
  }
  // 「頭部ディスプレイの画面: いま○○(保留中: 回答中のため)」。st.display は schedule_display.DisplayScheduleController.status()(無い/古い構成でも壊れない)
  function displayText(st) {
    const dp = st && st.display && typeof st.display === "object" ? st.display : null;
    if (!dp) {
      const rq = st && st.display_request;
      return rq && DISPLAY_JA[rq] ? "要求: " + DISPLAY_JA[rq] + "(反映状況は不明)" : "指定なし(そのまま)";
    }
    const cur = DISPLAY_JA[dp.current] ? "いま" + DISPLAY_JA[dp.current] : "いま不明";
    const want = DISPLAY_JA[dp.requested] || "元の画面";
    if (dp.state === "pending") return cur + "(保留中: " + (dp.pending_reason || "回答中のため") + "。終わったら" + want + "へ)";
    if (dp.state === "error") return cur + "(エラー: " + (dp.error || "ラズパイに届きません") + "。ロボットの走行には影響しません)";
    if (dp.state === "ok") return cur + "(このスケジュールの指定)";
    return "指定なし(そのまま)";
  }
  // 状態の注意表示(バックエンド schedule_controller.state の clock_suspect / status / safety_hold / boundary_override / armed / path_mismatch / startup_pending を読む)。
  // 重大(赤)から順に並べる。足すだけで、他の表示には触らない。
  function boundaryText(b) {
    if (!b || typeof b !== "object") return "";
    if (b.mode === "release" || b.enabled === false) return "解除";
    const g = (a, c) => (b[a] !== undefined ? b[a] : b[c]);
    const x0 = g("x_min", "min_x"), x1 = g("x_max", "max_x"), y0 = g("y_min", "min_y"), y1 = g("y_max", "max_y");
    const n = (v) => (typeof v === "number" ? (Math.round(v * 100) / 100) : "?");
    return "指定: x " + n(x0) + "〜" + n(x1) + " m、y " + n(y0) + "〜" + n(y1) + " m";
  }
  function stateAlerts(d, st) {
    const out = [];
    const clk = d && d.robot_clock && typeof d.robot_clock === "object" ? d.robot_clock : {};
    const noResp = st.status === "no_response";
    if (st.clock_suspect === true || clk.clock_suspect === true) {
      out.push(["err", "機体の時計が合っていないため、時刻の条件は成立しません(スケジュールは動きません)。"]);
    }
    if (noResp) {
      out.push(["err", "directorから応答がありません。" + (st.message ? "(" + st.message + ")" : "")]);
    } else if (st.status === "waiting") {
      out.push(["warn", "directorからの応答を待っています。" + (st.message ? "(" + st.message + ")" : "")]);
    }
    if (st.path_mismatch === true) {
      out.push(["err", "木のファイルのパスが一致していません(ダッシュボードとdirectorで、別の木のファイルを見ています。設定を確認してください)。"]);
    }
    if (st.safety_hold === true) {
      out.push(["warn", "安全のため停止中(緊急停止・範囲外など)。解除されるまで、スケジュールの行き先へは動きません。"]);
    }
    if (st.boundary_override) {
      out.push(["info", "動ける範囲を上書き中(" + (boundaryText(st.boundary_override) || "内容不明") + ")。"]);
    }
    if (st.startup_pending === true) {
      out.push(["info", "起動時の自動実行を待っています(起動後2分たち、機体の時計が合うと始まります)。"]);
    } else if (st.armed === true && st.running !== true && !noResp) {
      out.push(["warn", "実行の合図は出ていますが、まだ実行中になっていません(木のファイルが無効、または保存し直した直後の可能性があります)。"]);
    }
    return out;
  }
  function renderStateAlerts(d, st) {
    const box = $("stateAlerts");
    if (!box) return;
    box.textContent = "";
    for (const [kind, text] of stateAlerts(d, st)) {
      const div = document.createElement("div");
      div.className = "banner " + kind;
      div.textContent = text;
      box.appendChild(div);
    }
  }
  async function pollStatus() {
    if (document.hidden) return;
    try {
      const r = await api(API_SCHEDULE);
      if (!r.ok) throw new Error("HTTP " + r.status);
      const d = r.data;
      running = d.running === true;
      $("runBadge").textContent = running ? "実行中" : "停止中";
      $("runBadge").className = "pill " + (running ? "ok" : "off");
      const st = d.state && typeof d.state === "object" ? d.state : {};
      renderStateAlerts(d, st);
      const inner = st.state && typeof st.state === "object" ? st.state : st;   // state.state の中身はバックエンド次第
      // バックエンドの実際のキー(schedule_controller.state): active_entry_id / active_until / next_entry_id / next_entry_at。項目idは、保存済みスケジュールの行から時刻と行き先の名前にする。
      const entryLabel = (id) => { const es = (d.schedule && Array.isArray(d.schedule.entries)) ? d.schedule.entries : []; const e = es.find((x) => x && x.id === id); if (!e) return id || ""; const dest = e.destination ? (e.destination.pin_name || (e.destination.type === "entrance" ? "入口" : e.destination.type === "stay" ? "その場" : e.destination.pin_id || "")) : ""; return (e.start || "") + (e.end ? "〜" + e.end : "") + (dest ? " " + dest : ""); };
      const hhmm = (iso) => { if (!iso) return ""; const m = /T(\d\d:\d\d)/.exec(String(iso)); return m ? m[1] : String(iso); };
      const curByKey = st.active_entry_id ? entryLabel(st.active_entry_id) + (st.active_until ? "(" + hhmm(st.active_until) + "まで)" : "") : null;
      const nextByKey = st.next_entry_at ? hhmm(st.next_entry_at) + (st.next_entry_id ? " " + entryLabel(st.next_entry_id) : "") : null;
      $("curItem").textContent = running ? fmtVal(curByKey ?? pick(inner, ["current_label", "current_summary", "current_entry", "current_item", "current", "active_entry", "entry"]) ?? pick(st, ["current_label", "current_entry", "current"])) : "-";
      $("nextSwitch").textContent = running ? fmtVal(nextByKey ?? pick(inner, ["next_switch", "next_switch_at", "next_change", "next_entry", "next"]) ?? pick(st, ["next_switch", "next_switch_at", "next_change", "next"])) : "-";
      $("dispState").textContent = running ? displayText(st) : "-";
      $("stateRaw").textContent = JSON.stringify(d.state === undefined ? null : d.state, null, 2);
      const ss = d.schedule === undefined ? null : d.schedule;
      const sig = sigOf(ss);
      if (serverSchedule === undefined || (sig !== sigOf(serverSchedule) && !isDirty())) {
        serverSchedule = ss;
        loadFromServer(ss);
      }
    } catch (e) {
      $("runBadge").textContent = "状態を取得できません";
      $("runBadge").className = "pill bad";
    }
  }
  function syncTop() {
    $("schName").value = draft.name || "";
    $("runOnStartup").checked = !!draft.run_on_startup;
  }
  function loadFromServer(s) {
    draft = scheduleToDraft(s);
    savedSig = s ? sigOf(draftToApi()) : null;
    syncTop();
    renderEntries();
    renderDirty();
    runPreview();
  }
  $("reloadBtn").addEventListener("click", async () => {
    if (isDirty() && !confirm("未保存の変更を破棄して、保存済みを読み直しますか？")) return;
    serverSchedule = undefined;
    await pollStatus();
  });

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
      if (r.ok) mapSetPose(r.data.robot_pose);
    } catch (e) { /* 次のポーリングで再試行 */ }
  }
  async function setEstop(engaged) {
    if (readOnly || estopPending) return;
    estopPending = true;
    $("estopBtn").disabled = true;
    const prev = estopEngaged;
    renderEstop(engaged);
    let ok = false;
    try { ok = (await postJson("/api/soft_estop", { engaged })).ok; } catch (e) { ok = false; }
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

  // ---------------------------------------------------------------- 地図パネル(範囲・行き先を地図で指定)
  // 座標系: map座標(m, x右・y上)。地図は /api/map のグリッド(行優先、行0が下=origin側)で、
  // data_base64 の各バイトは「占有値+1」(0=未知、1=空き、101=壁)。既存の操作盤の描画(行0を下に描く)と同じ向きで描く。
  // 画面⇔map座標の変換は mapWorldToScreen / mapScreenToWorld の2つに集約している(v = {cx, cy, s(px/m), cw, ch})。
  // ==PURE-BEGIN==
  function mapWorldToScreen(v, x, y) { return [(x - v.cx) * v.s + v.cw / 2, (v.cy - y) * v.s + v.ch / 2]; }
  function mapScreenToWorld(v, sx, sy) { return [(sx - v.cw / 2) / v.s + v.cx, v.cy - (sy - v.ch / 2) / v.s]; }
  function mapFitView(meta, cw, ch) {
    const wm = Math.max(1e-6, meta.width * meta.resolution), hm = Math.max(1e-6, meta.height * meta.resolution);
    return { cx: meta.origin_x + wm / 2, cy: meta.origin_y + hm / 2, s: Math.max(1e-3, Math.min(cw / wm, ch / hm) * 0.96), cw: cw, ch: ch };
  }
  function mapZoomAt(v, sx, sy, factor, smin, smax) {
    const w = mapScreenToWorld(v, sx, sy);
    const s = Math.max(smin, Math.min(smax, v.s * factor));
    return { cx: w[0] - (sx - v.cw / 2) / s, cy: w[1] + (sy - v.ch / 2) / s, s: s, cw: v.cw, ch: v.ch };
  }
  function mapPanBy(v, dsx, dsy) { return { cx: v.cx - dsx / v.s, cy: v.cy + dsy / v.s, s: v.s, cw: v.cw, ch: v.ch }; }
  function mapRound2(n) { return Math.round(n * 100) / 100; }
  function mapNormRect(x0, y0, x1, y1) {
    return { x_min: Math.min(x0, x1), y_min: Math.min(y0, y1), x_max: Math.max(x0, x1), y_max: Math.max(y0, y1) };
  }
  // 矩形の角(screen上のnw=左上=(x_min,y_max))か内側(move)か外(null)。pad=角の当たり判定(px)
  function mapHit(v, r, sx, sy, pad) {
    const a = mapWorldToScreen(v, r.x_min, r.y_max), b = mapWorldToScreen(v, r.x_max, r.y_min);
    const corners = [["nw", a[0], a[1]], ["ne", b[0], a[1]], ["sw", a[0], b[1]], ["se", b[0], b[1]]];
    let best = null, bd = 1e18;
    for (const c of corners) {
      const d = Math.max(Math.abs(sx - c[1]), Math.abs(sy - c[2]));
      if (d <= pad && d < bd) { best = c[0]; bd = d; }
    }
    if (best) return best;
    return (sx >= a[0] && sx <= b[0] && sy >= a[1] && sy <= b[1]) ? "move" : null;
  }
  // 角ドラッグでのサイズ変更: 反対側の角を固定し、ドラッグ中の点(wx,wy)との矩形に正規化する(越えて反転しても崩れない)
  function mapResizeRect(r0, handle, wx, wy) {
    const ax = handle.indexOf("w") >= 0 ? r0.x_max : r0.x_min;
    const ay = handle.indexOf("n") >= 0 ? r0.y_min : r0.y_max;
    return mapNormRect(ax, ay, wx, wy);
  }
  function mapMoveRect(r0, dx, dy) {
    return { x_min: r0.x_min + dx, y_min: r0.y_min + dy, x_max: r0.x_max + dx, y_max: r0.y_max + dy };
  }
  function mapPinAt(v, pins, sx, sy, rad) {
    let best = null, bd = rad;
    for (const p of pins) {
      const s = mapWorldToScreen(v, p.x, p.y);
      const d = Math.hypot(sx - s[0], sy - s[1]);
      if (d <= bd) { best = p; bd = d; }
    }
    return best;
  }
  // 地図の1バイト(占有値+1)→グレー。0=未知(暗い灰)、1以降=空き(白)〜壁(黒)。既存の操作盤と同じ式
  function mapShadeOf(b) {
    const o = b - 1;
    if (o < 0) return 60;
    return Math.max(0, Math.min(255, 255 - Math.round(o * 2.3)));
  }
  // グリッドの添字i(行優先、行0=下)→描画用画像の添字(画像は上が行末)
  function mapImgIndexOf(i, width, height) {
    return ((height - 1 - Math.floor(i / width)) * width + (i % width));
  }
  // ==PURE-END==

  const API_MAP = API_MAP_PATH;
  const MAP_COLORS = ["#d9480f", "#1c7ed6", "#2f9e44", "#9c36b5", "#c2255c", "#0c8599", "#e8590c", "#5c940d"];
  const mapCv = $("mapCv");
  const mapCtx = mapCv.getContext("2d");
  let mapMeta = null;          // /api/mapの{width,height,resolution,origin_x,origin_y}
  let mapImg = null;           // 描画済みの地図(オフスクリーンcanvas)
  let mapSig = null;
  let mapView = { cx: 0, cy: 0, s: 50, cw: 300, ch: 300 };
  let mapFitS = 50;
  let mapUserMoved = false;
  let mapEditId = null;        // 地図で編集中の行のid
  let mapBoundary = null;      // /api/boundary(ダッシュボードの現在の範囲)
  let mapPose = null;          // 機体位置(/api/stateのrobot_pose。無ければ描かない)
  let mapSpace = false, mapHover = false;
  const mapPtrs = new Map();
  let mapDrag = null;
  let mapPinch = null;

  function mapEditEntry() { return mapEditId === null ? null : (draft.entries.find((e) => e.id === mapEditId) || null); }
  function mapEntryRect(en) {
    const n = RECT_KEYS.map(([k]) => toNum(en.rect[k]));
    if (n.some((x) => x === null)) return null;
    return mapNormRect(n[0], n[1], n[2], n[3]);
  }
  function mapPins() { return pinList().filter((p) => typeof p.x === "number" && typeof p.y === "number"); }

  function mapResize() {
    const w = $("mapWrap");
    const cw = Math.max(200, Math.floor(w.clientWidth || 300));
    const ch = Math.max(240, Math.min(560, Math.round(cw * 0.75), Math.round(window.innerHeight * 0.7)));
    const dpr = window.devicePixelRatio || 1;
    mapCv.style.width = cw + "px"; mapCv.style.height = ch + "px";
    mapCv.width = Math.round(cw * dpr); mapCv.height = Math.round(ch * dpr);
    mapView.cw = cw; mapView.ch = ch;
    if (mapMeta && !mapUserMoved) mapFit(); else mapDraw();
  }
  function mapFit() {
    if (!mapMeta) return;
    mapView = mapFitView(mapMeta, mapView.cw, mapView.ch);
    mapFitS = mapView.s;
    mapUserMoved = false;
    mapDraw();
  }
  function mapZoomBy(factor, sx, sy) {
    if (!mapMeta) return;
    mapView = mapZoomAt(mapView, sx === undefined ? mapView.cw / 2 : sx, sy === undefined ? mapView.ch / 2 : sy, factor, mapFitS * 0.3, mapFitS * 60);
    mapUserMoved = true;
    mapDraw();
  }

  // ---- 地図データの取得
  function mapSetNone() {
    const had = mapMeta !== null;
    mapMeta = null; mapImg = null; mapSig = null;
    $("mapWrap").style.display = "none";
    $("mapMsg").style.display = "block";
    ["mapZoomIn", "mapZoomOut", "mapFitBtn"].forEach((id) => { $(id).disabled = true; });
    if (had) mapDraw();
  }
  async function mapLoad() {
    if (document.hidden) return;
    try {
      const res = await fetch(API_MAP);
      if (!res.ok) { mapSetNone(); return; }
      const m = await res.json();
      if (!m || !m.width || !m.height || !m.resolution || typeof m.data_base64 !== "string") { mapSetNone(); return; }
      const sig = m.map_id !== undefined ? "id" + m.map_id : null;
      if (sig !== null && sig === mapSig && mapMeta) return;    // 同じ地図は作り直さない
      const bytes = Uint8Array.from(atob(m.data_base64), (c) => c.charCodeAt(0));
      const img = document.createElement("canvas");
      img.width = m.width; img.height = m.height;
      const ictx = img.getContext("2d");
      const id = ictx.createImageData(m.width, m.height);
      const n = m.width * m.height;
      for (let i = 0; i < n; i++) {
        const sh = mapShadeOf(bytes[i]);
        const k = mapImgIndexOf(i, m.width, m.height) * 4;
        id.data[k] = id.data[k + 1] = id.data[k + 2] = sh; id.data[k + 3] = 255;
      }
      ictx.putImageData(id, 0, 0);
      const first = mapMeta === null;
      mapImg = img; mapMeta = m; mapSig = sig;
      $("mapWrap").style.display = "block";
      $("mapMsg").style.display = "none";
      ["mapZoomIn", "mapZoomOut", "mapFitBtn"].forEach((id2) => { $(id2).disabled = false; });
      if (first) { mapResize(); mapFit(); } else mapDraw();
    } catch (e) { /* 次のポーリングで再試行 */ }
  }
  async function mapLoadBoundary() {
    if (document.hidden) return;
    try {
      const r = await api(API_BOUNDARY);
      mapBoundary = r.ok && r.data && r.data.enabled ? r.data : null;
    } catch (e) { mapBoundary = null; }
    mapRefresh();
  }
  function mapSetPose(p) {
    mapPose = p && typeof p.x === "number" && typeof p.y === "number" ? p : null;
    mapDraw();
  }

  // ---- 描画
  function mapBoundaryPts(b) {
    let cx, cy, w, h, yaw;
    if (b.width !== undefined && b.center_x !== undefined) { cx = b.center_x; cy = b.center_y; w = b.width; h = b.height; yaw = b.yaw || 0; }
    else { cx = (b.min_x + b.max_x) / 2; cy = (b.min_y + b.max_y) / 2; w = b.max_x - b.min_x; h = b.max_y - b.min_y; yaw = 0; }
    if (![cx, cy, w, h].every((x) => typeof x === "number" && isFinite(x))) return null;
    const c = Math.cos(yaw), s = Math.sin(yaw);
    return [[-w / 2, -h / 2], [w / 2, -h / 2], [w / 2, h / 2], [-w / 2, h / 2]].map(([lx, ly]) => mapWorldToScreen(mapView, cx + lx * c - ly * s, cy + lx * s + ly * c));
  }
  function mapDrawRect(r, color, edit) {
    const a = mapWorldToScreen(mapView, r.x_min, r.y_max), b = mapWorldToScreen(mapView, r.x_max, r.y_min);
    const x = a[0], y = a[1], w = b[0] - a[0], h = b[1] - a[1];
    mapCtx.save();
    mapCtx.globalAlpha = edit ? 0.28 : 0.13;
    mapCtx.fillStyle = color; mapCtx.fillRect(x, y, w, h);
    mapCtx.globalAlpha = edit ? 1 : 0.65;
    mapCtx.strokeStyle = color; mapCtx.lineWidth = edit ? 2.5 : 1.5;
    mapCtx.strokeRect(x, y, w, h);
    if (edit) {
      mapCtx.fillStyle = "#fff"; mapCtx.lineWidth = 2;
      for (const [px, py] of [[x, y], [x + w, y], [x, y + h], [x + w, y + h]]) { mapCtx.fillRect(px - 5, py - 5, 10, 10); mapCtx.strokeRect(px - 5, py - 5, 10, 10); }
      const label = `${(r.x_max - r.x_min).toFixed(2)} × ${(r.y_max - r.y_min).toFixed(2)} m`;
      mapCtx.font = "bold 12px system-ui, sans-serif";
      const tw = mapCtx.measureText(label).width;
      const ly = y > 22 ? y - 6 : y + h + 16;
      mapCtx.fillStyle = "rgba(255,255,255,0.88)"; mapCtx.fillRect(x, ly - 13, tw + 8, 17);
      mapCtx.fillStyle = color; mapCtx.fillText(label, x + 4, ly);
    }
    mapCtx.restore();
  }
  function mapDraw() {
    const dpr = window.devicePixelRatio || 1;
    mapCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
    mapCtx.clearRect(0, 0, mapView.cw, mapView.ch);
    if (!mapMeta || !mapImg) return;
    mapCtx.fillStyle = "#cfd6d2"; mapCtx.fillRect(0, 0, mapView.cw, mapView.ch);
    const tl = mapWorldToScreen(mapView, mapMeta.origin_x, mapMeta.origin_y + mapMeta.height * mapMeta.resolution);
    mapCtx.imageSmoothingEnabled = false;
    mapCtx.drawImage(mapImg, tl[0], tl[1], mapMeta.width * mapMeta.resolution * mapView.s, mapMeta.height * mapMeta.resolution * mapView.s);
    // ダッシュボードの現在の範囲(橙の破線)
    if (mapBoundary) {
      const pts = mapBoundaryPts(mapBoundary);
      if (pts) {
        mapCtx.save(); mapCtx.strokeStyle = "#ffb000"; mapCtx.lineWidth = 2.5; mapCtx.setLineDash([8, 5]); mapCtx.beginPath();
        pts.forEach((p, i) => { if (i === 0) mapCtx.moveTo(p[0], p[1]); else mapCtx.lineTo(p[0], p[1]); });
        mapCtx.closePath(); mapCtx.stroke(); mapCtx.restore();
      }
    }
    // 各行の範囲(行ごとの色。編集中の行は最後=最前面に濃く)
    draft.entries.forEach((en, i) => {
      if (en.bmode !== "set" || en.id === mapEditId) return;
      const r = mapEntryRect(en);
      if (r) mapDrawRect(r, MAP_COLORS[i % MAP_COLORS.length], false);
    });
    const ed = mapEditEntry();
    if (ed && ed.bmode === "set") {
      const r = mapEntryRect(ed);
      if (r) mapDrawRect(r, MAP_COLORS[draft.entries.indexOf(ed) % MAP_COLORS.length], true);
    }
    // ピン
    mapCtx.font = "12px system-ui, sans-serif";
    for (const p of mapPins()) {
      const s = mapWorldToScreen(mapView, p.x, p.y);
      const sel = !!(ed && ed.dest === "pin:" + p.id);
      mapCtx.beginPath(); mapCtx.arc(s[0], s[1], sel ? 9 : 7, 0, Math.PI * 2);
      mapCtx.fillStyle = sel ? "#e03131" : "#176b56"; mapCtx.fill();
      mapCtx.lineWidth = 2; mapCtx.strokeStyle = "#fff"; mapCtx.stroke();
      const nm = String(p.name || p.id);
      mapCtx.lineWidth = 3; mapCtx.strokeStyle = "rgba(255,255,255,0.9)"; mapCtx.strokeText(nm, s[0] + 11, s[1] + 4);
      mapCtx.fillStyle = "#111"; mapCtx.fillText(nm, s[0] + 11, s[1] + 4);
    }
    // 機体(位置が取れたときだけ)
    if (mapPose) {
      const s = mapWorldToScreen(mapView, mapPose.x, mapPose.y);
      mapCtx.fillStyle = "#ff5a3c"; mapCtx.beginPath(); mapCtx.arc(s[0], s[1], 7, 0, Math.PI * 2); mapCtx.fill();
      if (typeof mapPose.yaw === "number") {
        mapCtx.strokeStyle = "#ff5a3c"; mapCtx.lineWidth = 2; mapCtx.beginPath(); mapCtx.moveTo(s[0], s[1]);
        mapCtx.lineTo(s[0] + 18 * Math.cos(mapPose.yaw), s[1] - 18 * Math.sin(mapPose.yaw)); mapCtx.stroke();
      }
    }
  }
  function mapEntryLabel(en, i) { return `${i + 1}行目 ${en.start || "--:--"}${en.end ? "〜" + en.end : ""}`; }
  function mapRefresh() {
    const ed = mapEditEntry();
    if (mapEditId !== null && (!ed || ed.bmode !== "set")) mapEditId = null;
    document.querySelectorAll("#entries .entry").forEach((card) => {
      const en = draft.entries[Number(card.dataset.idx)];
      const on = !!en && en.id === mapEditId;
      card.classList.toggle("mapediting", on);
      const b = card.querySelector('button[data-act="mapedit"]');
      if (b) b.textContent = on ? "地図で編集中" : "地図で指定";
    });
    const cur = mapEditEntry();
    $("mapEditInfo").textContent = cur
      ? `編集中: ${mapEntryLabel(cur, draft.entries.indexOf(cur))}。地図をドラッグして範囲を描きます。ピンをクリックすると、この行の行き先になります。`
      : "編集する行がありません。「動ける範囲」を「指定する」にした行の「地図で指定」を押してください(地図は見るだけでも使えます)。";
    $("mapDoneBtn").disabled = !cur;
    $("mapClearRectBtn").disabled = !cur || readOnly;
    let lg = "";
    draft.entries.forEach((en, i) => {
      if (en.bmode === "set" && mapEntryRect(en)) lg += `<span class="lgi"><i style="background:${MAP_COLORS[i % MAP_COLORS.length]}"></i>${escapeHtml(mapEntryLabel(en, i))}${en.id === mapEditId ? "(編集中)" : ""}</span>`;
    });
    lg += `<span class="lgi"><i class="dash"></i>ダッシュボードの現在の範囲${mapBoundary ? "" : "(なし)"}</span>`;
    lg += `<span class="lgi"><i class="dot" style="background:#176b56"></i>ピン</span>`;
    if (mapPose) lg += `<span class="lgi"><i class="dot" style="background:#ff5a3c"></i>機体</span>`;
    $("mapLegend").innerHTML = lg;
    mapDraw();
  }
  function mapStartEdit(en) {
    if (readOnly) return;
    mapEditId = en.id;
    if (!mapMeta) showGlobal("warn", "地図がないため地図では指定できません。座標を数値で入力してください。");
    mapRefresh();
    $("mapSection").scrollIntoView({ behavior: "smooth", block: "nearest" });
  }
  // 矩形をその行の4値へ書き込み、入力欄(DOM)も同期する
  function mapWriteRect(en, r) {
    const blank = r === null;
    en.rect = blank ? { x_min: "", y_min: "", x_max: "", y_max: "" }
      : { x_min: mapRound2(r.x_min), y_min: mapRound2(r.y_min), x_max: mapRound2(r.x_max), y_max: mapRound2(r.y_max) };
    const card = document.querySelector(`#entries .entry[data-idx="${draft.entries.indexOf(en)}"]`);
    if (card) for (const [k] of RECT_KEYS) { const inp = card.querySelector(`input[data-f="rect"][data-k="${k}"]`); if (inp) inp.value = en.rect[k]; }
    renderDirty(); schedulePreview(); mapDraw();
  }

  // ---- 操作(マウス・タッチ共通のPointer Events)
  function mapPos(ev) { const r = mapCv.getBoundingClientRect(); return [ev.clientX - r.left, ev.clientY - r.top]; }
  function mapPinchStart() {
    const [a, b] = Array.from(mapPtrs.values());
    mapPinch = { c: [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2], d: Math.max(1, Math.hypot(a[0] - b[0], a[1] - b[1])) };
    mapDrag = null;
  }
  function mapPickPin(pin) {
    const en = mapEditEntry();
    if (!en || readOnly) { $("mapEditInfo").textContent = `ピン「${pin.name || pin.id}」。行き先にするには、先に行の「地図で指定」を押してください。`; return; }
    en.dest = "pin:" + pin.id; en.destName = pin.name || "";
    afterEdit(true);
    showGlobal("ok", `${draft.entries.indexOf(en) + 1}行目の行き先をピン「${pin.name || pin.id}」にしました。`);
  }
  mapCv.addEventListener("pointerdown", (ev) => {
    if (!mapMeta) return;
    try { mapCv.setPointerCapture(ev.pointerId); } catch (e) { /* 無視 */ }
    const p = mapPos(ev);
    mapPtrs.set(ev.pointerId, p);
    if (mapPtrs.size === 2) { mapPinchStart(); return; }
    if (mapPtrs.size > 2) return;
    if (ev.pointerType === "mouse" && (ev.button === 1 || ev.button === 2 || (ev.button === 0 && mapSpace))) { mapDrag = { kind: "pan", last: p }; ev.preventDefault(); return; }
    if (ev.pointerType === "mouse" && ev.button !== 0) return;
    const pin = mapPinAt(mapView, mapPins(), p[0], p[1], ev.pointerType === "touch" ? 18 : 12);
    if (pin) { mapDrag = { kind: "pin", pin: pin, start: p, last: p }; return; }
    const en = mapEditEntry();
    if (!en || readOnly) { mapDrag = { kind: "pan", last: p }; return; }
    const r = mapEntryRect(en);
    const hit = r ? mapHit(mapView, r, p[0], p[1], ev.pointerType === "touch" ? 18 : 10) : null;
    const w = mapScreenToWorld(mapView, p[0], p[1]);
    if (hit === "move") mapDrag = { kind: "move", r0: r, w0: w, en: en };
    else if (hit) mapDrag = { kind: "resize", r0: r, handle: hit, en: en };
    else mapDrag = { kind: "new", start: p, w0: w, started: false, en: en };
  });
  mapCv.addEventListener("pointermove", (ev) => {
    if (!mapMeta) return;
    const p = mapPos(ev);
    if (mapPtrs.has(ev.pointerId)) mapPtrs.set(ev.pointerId, p);
    if (mapPtrs.size === 2 && mapPinch) {
      const [a, b] = Array.from(mapPtrs.values());
      const c = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2], d = Math.max(1, Math.hypot(a[0] - b[0], a[1] - b[1]));
      mapView = mapPanBy(mapView, c[0] - mapPinch.c[0], c[1] - mapPinch.c[1]);
      mapView = mapZoomAt(mapView, c[0], c[1], d / mapPinch.d, mapFitS * 0.3, mapFitS * 60);
      mapPinch = { c: c, d: d }; mapUserMoved = true; mapDraw();
      return;
    }
    if (!mapDrag) {
      if (ev.pointerType === "mouse") {
        const en = mapEditEntry(), r = en && !readOnly ? mapEntryRect(en) : null;
        const h = r ? mapHit(mapView, r, p[0], p[1], 10) : null;
        mapCv.style.cursor = mapPinAt(mapView, mapPins(), p[0], p[1], 12) ? "pointer"
          : (h === "nw" || h === "se" ? "nwse-resize" : (h === "ne" || h === "sw" ? "nesw-resize" : (h === "move" ? "move" : (en && !readOnly ? "crosshair" : "grab"))));
      }
      return;
    }
    const d = mapDrag;
    if (d.kind === "pan") { mapView = mapPanBy(mapView, p[0] - d.last[0], p[1] - d.last[1]); d.last = p; mapUserMoved = true; mapDraw(); }
    else if (d.kind === "pin") {
      if (Math.hypot(p[0] - d.start[0], p[1] - d.start[1]) > 6) mapDrag = { kind: "pan", last: p };   // 動かしたらクリックではなくパン
    } else {
      const w = mapScreenToWorld(mapView, p[0], p[1]);
      if (d.kind === "move") mapWriteRect(d.en, mapMoveRect(d.r0, w[0] - d.w0[0], w[1] - d.w0[1]));
      else if (d.kind === "resize") mapWriteRect(d.en, mapResizeRect(d.r0, d.handle, w[0], w[1]));
      else if (d.kind === "new") {
        if (!d.started && Math.hypot(p[0] - d.start[0], p[1] - d.start[1]) < 6) return;
        d.started = true;
        mapWriteRect(d.en, mapNormRect(d.w0[0], d.w0[1], w[0], w[1]));
      }
    }
  });
  function mapPointerEnd(ev) {
    mapPtrs.delete(ev.pointerId);
    if (mapPtrs.size < 2) mapPinch = null;
    const d = mapDrag;
    if (mapPtrs.size === 0) mapDrag = null;
    if (!d) return;
    if (d.kind === "pin" && ev.type === "pointerup") mapPickPin(d.pin);
    else if (d.kind === "move" || d.kind === "resize" || d.kind === "new") mapRefresh();
  }
  mapCv.addEventListener("pointerup", mapPointerEnd);
  mapCv.addEventListener("pointercancel", mapPointerEnd);
  mapCv.addEventListener("pointerenter", () => { mapHover = true; });
  mapCv.addEventListener("pointerleave", () => { mapHover = false; });
  mapCv.addEventListener("contextmenu", (ev) => ev.preventDefault());
  mapCv.addEventListener("wheel", (ev) => {
    if (!mapMeta) return;
    ev.preventDefault();
    const p = mapPos(ev);
    mapZoomBy(Math.exp(-Math.max(-300, Math.min(300, ev.deltaY)) * 0.002), p[0], p[1]);
  }, { passive: false });
  document.addEventListener("keydown", (ev) => {
    if (ev.code !== "Space") return;
    const t = ev.target && ev.target.tagName;
    if (t === "INPUT" || t === "SELECT" || t === "TEXTAREA" || t === "BUTTON") return;
    mapSpace = true;
    if (mapHover) ev.preventDefault();
  });
  document.addEventListener("keyup", (ev) => { if (ev.code === "Space") mapSpace = false; });
  $("mapZoomIn").addEventListener("click", () => mapZoomBy(1.4));
  $("mapZoomOut").addEventListener("click", () => mapZoomBy(1 / 1.4));
  $("mapFitBtn").addEventListener("click", mapFit);
  $("mapDoneBtn").addEventListener("click", () => { mapEditId = null; mapRefresh(); });
  $("mapClearRectBtn").addEventListener("click", () => { const en = mapEditEntry(); if (en && !readOnly) { mapWriteRect(en, null); mapRefresh(); } });
  window.addEventListener("resize", mapResize);
  mapSetNone();
  mapResize();
  mapRefresh();
  mapLoad(); mapLoadBoundary();
  setInterval(mapLoad, 10000);
  setInterval(mapLoadBoundary, 5000);

  window.addEventListener("beforeunload", (e) => { if (isDirty()) { e.preventDefault(); e.returnValue = ""; } });
  document.addEventListener("visibilitychange", () => { if (!document.hidden) { pollStatus(); pollEstop(); } });

  (async function init() {
    try {
      const w = await api("/api/whoami");
      const account = (w.data && w.data.account) || "";
      $("accountLabel").textContent = account ? `ログイン: ${account}` : "";
      readOnly = !w.ok || account === "guest" || account === "";
    } catch (e) { readOnly = true; }
    if (readOnly) {
      showGlobal("info", "閲覧専用です(guestアカウント、または認証を確認できませんでした)。内容は見られますが、編集・保存・実行はできません。");
      $("estopBtn").disabled = true;
    }
    await loadPins();
    await pollStatus();
    if (savedSig === null && serverSchedule === undefined) { renderEntries(); renderDirty(); }
    pollEstop();
    setInterval(loadPins, 5000);
    setInterval(pollStatus, 2000);
    setInterval(pollEstop, 2000);
  })();
</script>
</body>
</html>
"""


def build_schedule_html() -> bytes:
    """かんたん時間割ページ(`/schedule`)。full_dashboard_nodeから配信される。"""
    html = inject_page_guide(_SCHEDULE_HTML, PAGE_SCHEDULE, big_clock=True)
    for key, val in (
        ("__API_SCHEDULE_PREVIEW__", API_SCHEDULE_PREVIEW),
        ("__API_SCHEDULE_RUN__", API_SCHEDULE_RUN),
        ("__API_SCHEDULE_STOP__", API_SCHEDULE_STOP),
        ("__API_SCHEDULE__", API_SCHEDULE),
        ("__API_PINS__", API_PINS),
        ("__API_BOUNDARY__", API_BOUNDARY),
        ("__API_MAP__", API_MAP),
    ):
        html = html.replace(key, val)
    return html.encode("utf-8")
