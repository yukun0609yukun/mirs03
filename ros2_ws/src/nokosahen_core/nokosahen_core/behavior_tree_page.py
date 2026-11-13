"""ビヘイビアツリー(behavior_tree)の編集ページ(`/behavior_tree`)のHTML。

【2026-10-09追加】完成版ダッシュボード(full_dashboard_node)から配信される。
バックエンドは behavior_tree_store.BehaviorTreeController / behavior_tree.py。
ページ自体はROSに触れず、HTTP API(/api/behavior_tree*)だけを使う。behavior_rules_page.py と同じ流儀。

設計メモ:
- ノード種別・パラメータ・選択肢は /api/behavior_tree の meta(behavior_tree.registry_meta())から動的に作る
  (ページ側に種別の一覧を持たない=サーバーの検証と食い違わない)。
- 楽観的UIは使わない。保存はPOSTして、成功したらサーバーから読み直して表示する。
- 全ての可変テキストは escapeHtml() を通してから innerHTML に入れる。
- 変更は「下書き」(未保存)としてブラウザ内に持ち、「保存して反映」で一括検証・保存する。
- 木のライブ状態(各ノードの成功/失敗/実行中)は、directorが読み込み中の木と保存済みの木が一致し、
  かつ下書きに未保存の変更が無い時だけ表示する(ノードのidは木の位置なので、食い違うと誤表示になるため)。
- 安全層(緊急停止〜地図作成中)は木の外のコード側で常に優先され、このページでは編集できない(表示のみ)。
- guestアカウントは閲覧のみ(入力欄・ボタンを無効化。サーバー側もPOSTは403)。
"""

from .nav_layout import NK_FIELD_JS
from .page_guide import PAGE_TREE, inject_page_guide

_BEHAVIOR_TREE_HTML = r"""<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>のこさへん - ビヘイビアツリー</title>
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
  button.small { padding:3px 8px; font-size:13px; }
  button.ghost { background:#eef3f0; color:#16211d; }
  button:disabled { opacity:0.45; cursor:not-allowed; }
  input[type="text"], input[type="number"], select, textarea { padding:6px; font-size:13px; border:1px solid #cfd8d3; border-radius:6px; box-sizing:border-box; max-width:100%; }
  input:disabled, select:disabled, textarea:disabled { background:#f1f3f2; color:#555; }
  label { font-size:13px; color:#33463e; }
  .banner { padding:8px 12px; border-radius:6px; font-size:13px; margin-top:6px; }
  .banner.err { background:#faece7; color:#993c1d; }
  .banner.ok { background:#e1f5ee; color:#0f6e56; }
  .banner.warn { background:#fff3cd; color:#7a5b00; }
  .banner.info { background:#eef3f0; color:#33463e; }
  .banner ul { margin:4px 0 0 18px; padding:0; }
  .grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(190px,1fr)); gap:8px; }
  .metric { background:#eef3f0; border-radius:8px; padding:6px 10px; min-width:0; }
  .metric span { display:block; font-size:13px; color:#53635c; }
  .metric b { font-size:14px; word-break:break-all; }
  .pill { display:inline-block; padding:2px 9px; border-radius:999px; font-size:13px; font-weight:700; }
  .pill.ok { background:#e1f5ee; color:#0f6e56; }
  .pill.bad { background:#faece7; color:#993c1d; }
  .pill.wait { background:#fff3cd; color:#7a5b00; }
  .pill.off { background:#e4e8e6; color:#53635c; }
  .tabs { display:flex; gap:4px; margin:10px 0 6px; border-bottom:1px solid #dce4df; }
  .tabs button { background:#eef3f0; color:#16211d; border-radius:6px 6px 0 0; }
  .tabs button.on { background:#176b56; color:#fff; }
  /* 安全層(固定) */
  .safety { border:2px solid #a32d2d; background:#fdf3f1; border-radius:8px; padding:8px 10px; margin-bottom:10px; }
  .safety h3 { margin:0 0 4px; color:#7a1f1f; }
  .safety .chips { display:flex; gap:6px; flex-wrap:wrap; }
  .schip { padding:2px 9px; border-radius:999px; font-size:13px; background:#f1dcd8; color:#7a1f1f; }
  .schip.hit { background:#a32d2d; color:#fff; font-weight:700; }
  /* ツリー */
  .tree { font-size:13px; }
  .node { display:flex; align-items:center; gap:6px; flex-wrap:wrap; padding:4px 6px; border-radius:6px; border:1px solid transparent; border-left:5px solid var(--kc, #999); background:#fbfcfb; margin:2px 0; }
  .node.sel { border-color:#176b56; box-shadow:0 0 0 2px #cfe9df; }
  .node.off { opacity:0.5; text-decoration:line-through; }
  .node.has-error { border-color:#d6735a; background:#faece7; }
  .node.st-SUCCESS { background:#e8f6ef; }
  .node.st-RUNNING { background:#fff3cd; }
  .node.st-FAILURE { background:#f3f4f3; color:#6b7a73; }
  .node .ic { width:22px; text-align:center; font-size:15px; }
  .node .title { font-weight:700; cursor:pointer; }
  .node .kind { font-size:13px; color:#fff; background:var(--kc, #999); border-radius:4px; padding:0 5px; }
  .node .summ { color:#33463e; font-size:13px; word-break:break-all; flex:1 1 160px; }
  .node .acts { display:flex; gap:3px; flex-wrap:wrap; margin-left:auto; }
  .node .badge { font-size:13px; border-radius:4px; padding:0 6px; font-weight:700; }
  .badge.S { background:#176b56; color:#fff; } .badge.R { background:#b7791f; color:#fff; } .badge.F { background:#9aa5a0; color:#fff; }
  .badge.out { background:#124739; color:#fff; }
  .children { margin-left:22px; border-left:1px dashed #b9c6bf; padding-left:8px; }
  .editor { margin:2px 0 6px 30px; padding:8px 10px; border:1px dashed #b9c6bf; border-radius:6px; background:#fff; display:flex; flex-direction:column; gap:8px; }
  .field { display:flex; flex-direction:column; gap:3px; }
  .checks { display:flex; flex-wrap:wrap; gap:6px 12px; }
  .checks label { display:inline-flex; align-items:center; gap:4px; font-size:13px; color:#16211d; }
  .nerr { margin:2px 0 0; padding:4px 8px; background:#faece7; color:#993c1d; border-radius:6px; font-size:13px; }
  .pin-note { margin-top:4px; font-size:13px; }
  .pin-note.warn { background:#fff3cd; color:#7a5b00; padding:4px 8px; border-radius:6px; }
  textarea#jsonBox { width:100%; min-height:380px; font-family:ui-monospace,Consolas,monospace; font-size:13px; }
  pre.diff { background:#f4f6f5; border-radius:6px; padding:8px; font-size:13px; overflow:auto; max-height:260px; margin:6px 0 0; }
  pre.diff .add { color:#0f6e56; } pre.diff .del { color:#993c1d; }
  #estopDock { position:fixed; left:0; right:0; bottom:0; z-index:2000; box-sizing:border-box; padding:8px 12px; background:rgba(18,71,57,0.96); box-shadow:0 -2px 10px rgba(0,0,0,0.3); display:flex; gap:10px; align-items:center; }
  #estopDock .stop-btn { flex:1 1 auto; margin:0; padding:14px; font-size:18px; min-height:52px; border:2px solid #fff; font-weight:700; background:#a32d2d; color:#fff; border-radius:8px; }
  #estopDock .stop-btn.engaged { background:#5c1717; }
  #estopDock a { color:#cfe9df; font-size:13px; white-space:nowrap; }
  .dirty-bar { position:sticky; top:0; z-index:50; }
  @media (max-width:640px) { main { padding:8px; } .children { margin-left:10px; } .editor { margin-left:8px; } }
__PAGE_GUIDE_CSS__
</style>
</head>
<body>
<header>
  <h1>ビヘイビアツリー</h1>
  <div class="row">
    <span id="accountLabel" style="font-size:13px;opacity:0.85;"></span>
    <a href="/behavior_rules">時間帯・状況ルール</a>
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
      <span id="syncBadge" class="pill off">木の反映: -</span>
      <span id="liveMsg" class="hint" style="margin:0;"></span>
    </div>
    <div class="grid" id="liveGrid"></div>
    <div id="liveWarn"></div>
    <p class="hint">1.5秒ごとに更新します。このページを閉じると、ダッシュボードはこの状態の購読をやめます(負荷がかかりません)。</p>
  </section>

  <section>
    <h2>木の編集</h2>
    <div class="banner info" style="margin-top:0;">
      木は「いつ・どんな時に・どう動くか」を上から順に判断する仕組みです。<b>条件</b>(時間帯・混雑・人物・残数など)で枝を選び、
      <b>動作</b>(待機・入口へ・ピンへ移動・呼びかけ間隔など)を出します。木が決めるのは「方針」だけで、
      <b>下の赤い安全層は木の外で常に最優先</b>です(緊急停止・手動運転・アンケート回答中・人が近すぎる・走行範囲外・立入禁止・カメラ障害物・地図作成中)。
      安全層に該当する間は、木の結果にかかわらず走行指令もゴールも出ません。木のファイルが無い/不正/無効の間は、これまでどおり(時間帯・状況ルール表のみ)の動作です。
    </div>
    <div id="fileInfo" class="hint"></div>
    <div id="pinInfo" class="hint"></div>
    <div id="pinWarn"></div>
    <div id="dirtyBar" class="dirty-bar"></div>
    <div id="saveErrors"></div>

    <div class="row" style="margin:10px 0;">
      <select id="addType"></select>
      <button id="addChildBtn" type="button" class="small">選択ノードの子に追加</button>
      <button id="addSibBtn" type="button" class="small ghost">選択ノードの下に追加</button>
      <button id="wrapBtn" type="button" class="small ghost" title="選択ノードを、制御/加工ノード(上のリストから選んだもの)で包みます">選択ノードを包む</button>
      <span style="flex:1"></span>
      <select id="exampleSel"></select>
      <button id="exampleBtn" type="button" class="small secondary">例から読み込む</button>
    </div>

    <div class="tabs">
      <button id="tabVisual" type="button" class="on">ビジュアル</button>
      <button id="tabJson" type="button">JSON(上級者向け)</button>
    </div>

    <div id="visualPane">
      <div class="safety" id="safetyBox"></div>
      <div id="treeBox" class="tree"></div>
    </div>
    <div id="jsonPane" style="display:none;">
      <textarea id="jsonBox" spellcheck="false"></textarea>
      <div class="row" style="margin-top:6px;">
        <button id="jsonApplyBtn" type="button" class="secondary">JSONを下書きに反映</button>
        <span class="hint" style="margin:0;">反映前に形式だけ確認します。内容の検証は「検証」または「保存して反映」で行います。</span>
      </div>
    </div>

    <h3>全体設定</h3>
    <div class="row">
      <div class="field"><label for="minHold">最小保持時間(秒、0〜600)</label>
        <input id="minHold" type="number" min="0" max="600" step="1" style="width:120px;" /></div>
      <div class="field"><label for="docEnabled">木を使う</label>
        <select id="docEnabled"><option value="true">使う</option><option value="false">使わない(従来のルール表のみ)</option></select></div>
      <div class="field" style="flex:1 1 260px;"><label for="docDesc">説明(メモ)</label>
        <input id="docDesc" type="text" style="width:100%;" /></div>
    </div>
    <p class="hint">最小保持時間: 木が選ぶ方針が切り替わった後、この秒数の間は次の方針へ切り替えません(混雑度のゆらぎで往復しないため)。</p>

    <div id="validateResult"></div>
    <div class="row" style="margin-top:14px;">
      <button id="validateBtn" type="button" class="secondary">検証</button>
      <button id="saveBtn" type="button">保存して反映</button>
      <button id="discardBtn" type="button" class="secondary">変更を破棄</button>
      <button id="resetBtn" type="button" class="danger" title="木のファイルを退避(.bak)して無くし、従来のルール表のみの動作に戻します">既定に戻す</button>
      <span id="saveStatus" class="hint" style="margin:0;"></span>
    </div>
    <p class="hint">保存するとサーバー側で検証し、1つでも不正があれば何も書き込みません。保存できたら直前のファイルを .bak に残し、behavior_directorが1秒以内に自動で読み込みます。</p>
  </section>

  <section>
    <h2>現在の動作と保存済みの木の比較</h2>
    <div class="row">
      <button id="compareBtn" type="button" class="secondary">比較する</button>
      <span class="hint" style="margin:0;">(1) directorが今使っている木と保存済みの木が一致しているか (2) いまの下書きと保存済みの木の違い</span>
    </div>
    <div id="compareBody" class="hint">「比較する」を押してください。</div>
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
  function clone(o) { return JSON.parse(JSON.stringify(o)); }

  const KIND_COLORS = { composite: "#2b6cb0", decorator: "#7b4bb0", condition: "#b7791f", action: "#176b56", leaf: "#4a5568" };
  const KIND_LABELS = { composite: "制御", decorator: "加工", condition: "条件", action: "動作", leaf: "ルール表" };
  const NUM_DEFAULTS = { value: 0.3, scale: 0.7, seconds: 60, interval_seconds: 30 };
  const COLOR_RE = /^#[0-9a-fA-F]{6}$/;

  let meta = null;
  let TYPES = {};
  let readOnly = true;
  let draft = null;              // 編集中の文書(未保存の下書き)
  let serverDoc = null;          // 最後にサーバーから読んだ文書(dirty判定用)
  let serverSig = null;          // 競合検知用
  let serverInfo = null;
  let selId = "0";               // 選択中のノードid(木の位置。"0"=ルート、"0.1"=ルートの2番目の子)
  const openEdit = new Set();    // 編集フォームを開いているノードid
  let nodeErrors = {};           // id -> [エラー文]
  let live = null;               // /api/behavior_tree/state の最新
  let examples = [];
  let pinsInfo = null;
  let pinsSig = "";
  let busy = false;
  let jsonEdited = false;        // JSON欄に、下書きへ未反映の編集がある

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
    return `<div class="banner ${kind}">${escapeHtml(title)}<ul>${items.map((e) => `<li>${escapeHtml(e)}</li>`).join("")}</ul></div>`;
  }

  // ---------------------------------------------------------------- ノード操作(idは木の位置)
  function idxOf(id) { return id.split(".").slice(1).map(Number); }
  function nodeAt(id, root) {
    let n = root || draft.root;
    for (const i of idxOf(id)) { if (!n || !n.children) return null; n = n.children[i]; }
    return n || null;
  }
  function parentId(id) { const p = id.split("."); return p.length > 1 ? p.slice(0, -1).join(".") : null; }
  function lastIdx(id) { const p = id.split("."); return Number(p[p.length - 1]); }
  function arityOf(n) { return (TYPES[n.type] || {}).children || "none"; }

  function defaultParams(typ) {
    const spec = TYPES[typ];
    const p = {};
    for (const s of spec.params) {
      if (s.type === "hidden") continue;
      if (s.default !== undefined) { if (s.required || s.default !== false && s.default !== true) p[s.key] = s.default; continue; }
      if (!s.required) continue;
      if (s.type === "number") {
        let v = NUM_DEFAULTS[s.key] !== undefined ? NUM_DEFAULTS[s.key] : (s.min !== undefined ? Math.max(s.min, 10) : 10);
        if (s.max !== undefined) v = Math.min(v, s.max);
        if (s.min !== undefined) v = Math.max(v, s.min);
        p[s.key] = v;
      } else if (s.type === "enum") p[s.key] = s.options[0][0];
      else if (s.type === "multi") p[s.key] = [s.options[0][0]];
      else if (s.type === "time_range") p[s.key] = "09:00-17:00";
      else if (s.type === "dates") p[s.key] = [new Date().toISOString().slice(0, 10)];
    }
    return p;
  }
  function newNode(typ) {
    const n = { type: typ };
    const p = defaultParams(typ);
    if (Object.keys(p).length) n.params = p;
    if (arityOf(n) !== "none") n.children = [];
    return n;
  }

  function addNode(mode) {
    if (readOnly) return;
    const typ = $("addType").value;
    if (!TYPES[typ]) return;
    const sel = nodeAt(selId);
    if (!sel) return;
    const node = newNode(typ);
    let newId;
    if (mode === "child") {
      const a = arityOf(sel);
      if (a === "none") { showGlobal("warn", `「${labelOf(sel)}」は子を持てません。ほかのノードを選ぶか「下に追加」を使ってください。`); return; }
      if (a === "one" && sel.children && sel.children.length >= 1) { showGlobal("warn", `「${labelOf(sel)}」の子は1つだけです。`); return; }
      sel.children = sel.children || [];
      sel.children.push(node);
      newId = `${selId}.${sel.children.length - 1}`;
    } else if (mode === "sibling") {
      const pid = parentId(selId);
      if (!pid) { showGlobal("warn", "ルートの下には追加できません。「選択ノードの子に追加」を使ってください。"); return; }
      const par = nodeAt(pid);
      if (arityOf(par) === "one") { showGlobal("warn", `「${labelOf(par)}」の子は1つだけです。`); return; }
      const at = lastIdx(selId) + 1;
      par.children.splice(at, 0, node);
      newId = `${pid}.${at}`;
    } else { // wrap
      const a = arityOf(node);
      if (a === "none") { showGlobal("warn", "包めるのは制御/加工ノードだけです(上のリストで制御・加工を選んでください)。"); return; }
      const pid = parentId(selId);
      node.children = [sel];
      if (pid) nodeAt(pid).children[lastIdx(selId)] = node; else draft.root = node;
      newId = selId;
    }
    showGlobal("", "");
    openEdit.clear();
    selId = newId;
    openEdit.add(newId);
    afterEdit();
  }

  function moveNode(id, delta) {
    const pid = parentId(id);
    if (!pid) return;
    const kids = nodeAt(pid).children;
    const i = lastIdx(id), j = i + delta;
    if (j < 0 || j >= kids.length) return;
    [kids[i], kids[j]] = [kids[j], kids[i]];
    openEdit.clear();
    selId = `${pid}.${j}`;
    afterEdit();
  }
  function deleteNode(id) {
    const pid = parentId(id);
    if (!pid) { showGlobal("warn", "ルートは削除できません(中身を空にしたい場合は子を削除してください)。"); return; }
    const n = nodeAt(id);
    const count = countNodes(n);
    if (count > 1 && !confirm(`「${labelOf(n)}」と、その下の${count - 1}個のノードを削除しますか？`)) return;
    nodeAt(pid).children.splice(lastIdx(id), 1);
    openEdit.clear();
    selId = pid;
    afterEdit();
  }
  function countNodes(n) { return 1 + (n.children || []).reduce((s, c) => s + countNodes(c), 0); }
  function toggleNode(id) {
    const n = nodeAt(id);
    if (!n) return;
    if (id === "0") { showGlobal("warn", "ルートノードは無効にできません。"); return; }
    if (n.enabled === false) delete n.enabled; else n.enabled = false;
    afterEdit();
  }

  function afterEdit() {
    renderTree();
    renderDirty();
    syncJsonBox();
  }

  // ---------------------------------------------------------------- 表示用の文
  function labelOf(n) { return n.name || (TYPES[n.type] ? TYPES[n.type].label : n.type); }
  function optLabel(s, v) {
    const o = (s.options || []).find((x) => x[0] === v);
    return o ? o[1] : String(v);
  }
  function summary(n) {
    const spec = TYPES[n.type];
    if (!spec) return "";
    const p = n.params || {};
    const parts = [];
    for (const s of spec.params) {
      if (s.type === "hidden" || p[s.key] === undefined) continue;
      let v = p[s.key];
      if (s.type === "enum") v = optLabel(s, v);
      else if (s.type === "multi") v = v.map((x) => optLabel(s, x)).join("・");
      else if (s.type === "bool") v = v ? "はい" : "いいえ";
      else if (s.type === "dates") v = v.join(", ");
      else if (s.type === "pin") { v = p.pin_name || v; if (pinsInfo && !pinList().some((q) => q.id === p.pin_id)) v += "(現在の地図に無い)"; }
      parts.push(`${s.label.replace(/[((].*$/, "")}: ${v}`);
    }
    return parts.join(" / ");
  }

  // ---------------------------------------------------------------- ツリー描画
  function renderTree() {
    const html = renderNode(draft.root, "0");
    $("treeBox").innerHTML = html;
    applyStatuses();
  }
  function renderNode(n, id) {
    const spec = TYPES[n.type] || { kind: "leaf", icon: "?", label: n.type, params: [], children: "none" };
    const color = KIND_COLORS[spec.kind] || "#999";
    const errs = nodeErrors[id];
    const cls = ["node"];
    if (id === selId) cls.push("sel");
    if (n.enabled === false) cls.push("off");
    if (errs) cls.push("has-error");
    const canEdit = !readOnly;
    const acts = canEdit ? `
      <span class="acts">
        <button class="small ghost" data-act="up" data-id="${id}" title="上へ">↑</button>
        <button class="small ghost" data-act="down" data-id="${id}" title="下へ">↓</button>
        <button class="small ghost" data-act="edit" data-id="${id}">${openEdit.has(id) ? "閉じる" : "編集"}</button>
        <button class="small ghost" data-act="toggle" data-id="${id}">${n.enabled === false ? "有効にする" : "無効にする"}</button>
        <button class="small danger" data-act="del" data-id="${id}">削除</button>
      </span>` : `<span class="acts"><button class="small ghost" data-act="edit" data-id="${id}">${openEdit.has(id) ? "閉じる" : "詳細"}</button></span>`;
    let out = `<div class="${cls.join(" ")}" data-node="${id}" style="--kc:${color}">
      <span class="ic">${escapeHtml(spec.icon)}</span>
      <span class="kind">${escapeHtml(KIND_LABELS[spec.kind] || spec.kind)}</span>
      <span class="title" data-act="select" data-id="${id}" title="${escapeHtml(spec.desc || "")}">${escapeHtml(labelOf(n))}</span>
      <span class="summ">${escapeHtml(n.name ? spec.label + (summary(n) ? " / " : "") : "")}${escapeHtml(summary(n))}</span>
      <span class="badge-slot" data-slot="${id}"></span>
      ${acts}
    </div>`;
    if (errs) out += errs.map((e) => `<div class="nerr">${escapeHtml(e)}</div>`).join("");
    if (openEdit.has(id)) out += renderEditor(n, id, spec);
    const kids = n.children || [];
    if (kids.length) out += `<div class="children">${kids.map((c, i) => renderNode(c, `${id}.${i}`)).join("")}</div>`;
    else if (spec.children === "many" || spec.children === "one") out += `<div class="children"><div class="hint">(子がありません。このノードを選んで「子に追加」)</div></div>`;
    return out;
  }

  function pinOptionsHtml(selId_) {
    let found = false;
    const opts = pinList().map((p) => {
      const sel = p.id === selId_;
      if (sel) found = true;
      return `<option value="${escapeHtml(p.id)}" style="color:${escapeHtml(pinColor(p))}" ${sel ? "selected" : ""}>● ${escapeHtml(p.name)}</option>`;
    });
    const head = `<option value="" ${selId_ ? "" : "selected"}>(ピンを選んでください)</option>`;
    const missing = selId_ && !found ? `<option value="${escapeHtml(selId_)}" selected>⚠ ${escapeHtml(selId_)}(現在の地図に無い)</option>` : "";
    return head + missing + opts.join("");
  }

  function renderEditor(n, id, spec) {
    const p = n.params || {};
    const dis = readOnly ? "disabled" : "";
    let fields = `<div class="field"><label>表示名(メモ。空なら種別名)</label>
      <input type="text" data-nf="name" data-id="${id}" value="${escapeHtml(n.name || "")}" maxlength="60" ${dis} /></div>
      <div class="hint" style="margin:0;">${escapeHtml(spec.desc || "")}</div>`;
    for (const s of spec.params) {
      if (s.type === "hidden") continue;
      const k = escapeHtml(s.key);
      const cur = p[s.key];
      const hint = s.hint ? `<div class="hint" style="margin:0;">${escapeHtml(s.hint)}</div>` : "";
      let ctl = "";
      if (s.type === "number") {
        ctl = `<input type="number" data-pk="${k}" data-id="${id}" value="${cur === undefined ? "" : escapeHtml(cur)}" ${s.min !== undefined ? `min="${s.min}"` : ""} ${s.max !== undefined ? `max="${s.max}"` : ""} step="any" style="width:140px;" ${dis} />`;
      } else if (s.type === "bool") {
        const v = cur === undefined ? (s.default === true) : cur;
        ctl = `<select data-pk="${k}" data-id="${id}" data-bool="1" ${dis}><option value="true" ${v ? "selected" : ""}>はい</option><option value="false" ${v ? "" : "selected"}>いいえ</option></select>`;
      } else if (s.type === "enum") {
        ctl = `<select data-pk="${k}" data-id="${id}" ${dis}>${(s.required ? "" : "") + s.options.map((o) => `<option value="${escapeHtml(o[0])}" ${(cur !== undefined ? cur : s.default) === o[0] ? "selected" : ""}>${escapeHtml(o[1])}</option>`).join("")}</select>`;
      } else if (s.type === "multi") {
        const cs = Array.isArray(cur) ? cur : [];
        ctl = `<div class="checks">${s.options.map((o) => `<label><input type="checkbox" data-pm="${k}" data-id="${id}" value="${escapeHtml(o[0])}" ${cs.includes(o[0]) ? "checked" : ""} ${dis} />${escapeHtml(o[1])}</label>`).join("")}</div>`;
      } else if (s.type === "dates") {
        ctl = `<input type="text" data-pk="${k}" data-dates="1" data-id="${id}" value="${escapeHtml((cur || []).join(", "))}" style="width:320px;" placeholder="2026-10-24, 2026-10-25" ${dis} />`;
      } else if (s.type === "pin") {
        ctl = `<select data-pk="${k}" data-pin="1" data-id="${id}" style="min-width:240px;" ${dis}>${pinOptionsHtml(cur || "")}</select>${pinNote(n)}`;
      } else {
        ctl = `<input type="text" data-pk="${k}" data-id="${id}" value="${cur === undefined ? "" : escapeHtml(cur)}" ${s.type === "time_range" ? 'placeholder="11:30-13:00" style="width:160px;"' : 'style="width:260px;"'} ${dis} />`;
      }
      fields += `<div class="field"><label>${escapeHtml(s.label)}${s.required ? " *" : ""}</label>${ctl}${hint}</div>`;
    }
    return `<div class="editor" data-editor="${id}">${fields}</div>`;
  }
  function pinNote(n) {
    const p = n.params || {};
    if (!p.pin_id) return `<div class="pin-note warn">行き先のピンを選んでください(地図エディタで作ったピンから選べます)。</div>`;
    if (!pinsInfo) return `<div class="pin-note warn">ピン一覧を取得できませんでした。</div>`;
    if (!loadedMap()) return `<div class="pin-note warn">地図が読み込まれていません。この状態では実行時にそのピンへ行かず、その場で待機します。</div>`;
    const q = pinList().find((x) => x.id === p.pin_id);
    if (!q) return `<div class="pin-note warn">このピンは現在の地図「${escapeHtml(loadedMap())}」にありません。実行時はそのピンへ行かず、その場で待機します。</div>`;
    if (p.pin_map && p.pin_map !== loadedMap()) return `<div class="pin-note warn">このピンは別の地図「${escapeHtml(p.pin_map)}」用です。</div>`;
    return `<div class="pin-note hint">地図「${escapeHtml(loadedMap())}」のピン(${Number(q.x).toFixed(2)}, ${Number(q.y).toFixed(2)}) m、向き ${Math.round(q.yaw_deg)}°</div>`;
  }

  // ---------------------------------------------------------------- ツリーのイベント
  $("treeBox").addEventListener("click", (ev) => {
    const el = ev.target.closest("[data-act]");
    if (!el) return;
    const id = el.getAttribute("data-id");
    const act = el.getAttribute("data-act");
    if (act === "select") { selId = id; renderTree(); return; }
    if (act === "edit") { if (openEdit.has(id)) openEdit.delete(id); else openEdit.add(id); selId = id; renderTree(); return; }
    if (readOnly) return;
    if (act === "up") moveNode(id, -1);
    else if (act === "down") moveNode(id, +1);
    else if (act === "del") deleteNode(id);
    else if (act === "toggle") toggleNode(id);
  });
  function toNum(v) { if (v === "" || v === null || v === undefined) return null; const n = Number(v); return Number.isFinite(n) ? n : null; }
  $("treeBox").addEventListener("change", (ev) => {
    if (readOnly) return;
    const t = ev.target;
    const id = t.getAttribute("data-id");
    if (!id) return;
    const n = nodeAt(id);
    if (!n) return;
    if (t.getAttribute("data-nf") === "name") {
      const v = t.value.trim();
      if (v) n.name = v; else delete n.name;
      afterEditKeepFocus(); return;
    }
    n.params = n.params || {};
    const pm = t.getAttribute("data-pm");
    if (pm) {
      const boxes = Array.from($("treeBox").querySelectorAll(`input[data-pm="${pm}"][data-id="${id}"]`));
      n.params[pm] = boxes.filter((b) => b.checked).map((b) => b.value);
      if (!n.params[pm].length) delete n.params[pm];
      afterEditKeepFocus(); return;
    }
    const pk = t.getAttribute("data-pk");
    if (!pk) return;
    if (t.getAttribute("data-pin")) {
      const pid = t.value;
      if (!pid) { delete n.params.pin_id; delete n.params.pin_name; delete n.params.pin_map; }
      else {
        const q = pinList().find((x) => x.id === pid);
        n.params.pin_id = pid;
        if (q) { n.params.pin_name = q.name; if (loadedMap()) n.params.pin_map = loadedMap(); }
      }
      renderTree(); renderDirty(); syncJsonBox(); return;
    }
    if (t.getAttribute("data-bool")) { n.params[pk] = t.value === "true"; }
    else if (t.getAttribute("data-dates")) {
      const ds = t.value.split(/[,\s]+/).filter(Boolean);
      if (ds.length) n.params[pk] = ds; else delete n.params[pk];
    } else if (t.type === "number") {
      const v = toNum(t.value);
      if (v === null) delete n.params[pk]; else n.params[pk] = v;
    } else if (t.tagName === "SELECT") { n.params[pk] = t.value; }
    else {
      const v = t.value.trim();
      if (v) n.params[pk] = v; else delete n.params[pk];
    }
    if (!Object.keys(n.params).length) delete n.params;
    afterEditKeepFocus();
  });
  // 入力欄を作り直さずに、見出し行の要約だけ更新する(入力中のフォーカスを保つ)
  function afterEditKeepFocus() {
    renderDirty(); syncJsonBox();
    const box = $("treeBox");
    const active = document.activeElement;
    const activeId = active && active.getAttribute ? active.getAttribute("data-id") : null;
    renderTree();
    if (activeId) { /* 変更確定後(change)なのでフォーカス喪失は問題ない */ }
  }

  // ---------------------------------------------------------------- 安全層(固定表示)
  function renderSafety() {
    const st = live && live.state ? live.state : null;
    const sf = st ? st.safety || {} : {};
    const layers = meta.safety_layers || [];
    $("safetyBox").innerHTML = `<h3>🔒 安全層(固定・編集不可。コード側で木より常に先に判定されます)</h3>
      <div class="chips">${layers.map((l, i) => `<span class="schip ${sf[l.key] ? "hit" : ""}" title="${escapeHtml(l.mode)}">${i + 1}. ${escapeHtml(l.label)}${sf[l.key] ? " ← 該当中" : ""}</span>`).join("")}</div>
      <div class="hint" style="margin:4px 0 0;">優先順: 上から順に評価し、最初に該当した分岐でロボットを止めます。どれにも該当しない時だけ、下の木が決めた方針に従います。
      木の動作ノードは「方針」を出すだけで、走行・ゴール送信は従来どおり走行可能範囲・立入禁止・ピンの範囲判定を通ります。</div>`;
  }

  // ---------------------------------------------------------------- ライブ状態
  function liveUsable() {
    return !!(live && live.status === "ok" && live.state && live.sync === "synced" && !isDirty() && live.state.user_active !== undefined);
  }
  function applyStatuses() {
    const usable = liveUsable();
    const sts = usable ? (live.state.statuses || {}) : {};
    const leaves = usable ? (live.state.active_leaves || []) : [];
    document.querySelectorAll("#treeBox .node").forEach((el) => {
      const id = el.getAttribute("data-node");
      el.classList.remove("st-SUCCESS", "st-RUNNING", "st-FAILURE");
      const s = sts[id];
      if (s) el.classList.add("st-" + s);
      const slot = el.querySelector(".badge-slot");
      if (slot) {
        const lab = { SUCCESS: ["S", "成功"], RUNNING: ["R", "実行中"], FAILURE: ["F", "失敗"] }[s];
        slot.innerHTML = (lab ? `<span class="badge ${lab[0]}">${lab[1]}</span>` : "") + (leaves.includes(id) ? ` <span class="badge out">▶ 出力中</span>` : "");
      }
    });
  }
  async function pollLive() {
    if (document.hidden) return;
    try {
      const r = await api("/api/behavior_tree/state");
      if (!r.ok) throw new Error("HTTP " + r.status);
      live = r.data;
      renderLive();
    } catch (e) {
      $("liveBadge").className = "pill bad";
      $("liveBadge").textContent = "ダッシュボードに接続できません";
      $("liveMsg").textContent = "";
    }
  }
  function renderLive() {
    const d = live;
    const st = d.state;
    const badge = $("liveBadge");
    if (d.status === "ok" && st) { badge.className = "pill ok"; badge.textContent = "behavior_director 応答中"; }
    else if (d.status === "waiting") { badge.className = "pill wait"; badge.textContent = "状態を待っています"; }
    else { badge.className = "pill bad"; badge.textContent = "behavior_director 無応答/未起動"; }
    $("liveMsg").textContent = d.message || "";
    const sb = $("syncBadge");
    if (d.sync === "synced") { sb.className = "pill ok"; sb.textContent = "木の反映: 保存済みと一致"; }
    else if (d.sync === "stale") { sb.className = "pill wait"; sb.textContent = "木の反映: directorはまだ別の木(1秒ほどで反映)"; }
    else { sb.className = "pill off"; sb.textContent = "木の反映: 不明"; }
    renderSafety();
    if (!st) { $("liveGrid").innerHTML = ""; $("liveWarn").innerHTML = ""; applyStatuses(); return; }
    const rows = [
      ["木の使用", st.user_active ? "木を使用中" : (st.enabled ? "従来動作(木のファイルなし/無効)" : "従来動作(enabled:false)")],
      ["現在の動作モード", st.mode || "-"],
      ["出している方針", st.policy || "normal"],
      ["動作したノード", (st.active_leaves || []).length ? st.active_leaves.join(", ") : "なし"],
      ["ルール表の有効ルール", st.active_rule_name ? `${st.active_rule_name} (${st.active_rule_id})` : "なし"],
      ["安全層", st.safety_hold ? "該当中(木の結果は無視)" : "該当なし"],
      ["ノード数", st.node_count],
    ];
    if (st.params && Object.keys(st.params).length) rows.push(["方針のパラメータ", JSON.stringify(st.params)]);
    $("liveGrid").innerHTML = rows.map(([k, v]) => `<div class="metric"><span>${escapeHtml(k)}</span><b>${escapeHtml(v)}</b></div>`).join("");
    let warn = "";
    if (st.load_error) warn += `<div class="banner err">director側の木の読み込みエラー(直前の木/従来動作を維持中): ${escapeHtml(st.load_error)}</div>`;
    if (d.path_mismatch) warn += `<div class="banner warn">ダッシュボードの木ファイル(${escapeHtml(d.dashboard_tree_file)})とdirectorのtree_file(${escapeHtml(st.tree_file)})が違います。保存しても反映されません。nokosahen.yamlの2つのパスを揃えてください。</div>`;
    if (!liveUsable() && d.status === "ok") warn += `<div class="hint">${isDirty() ? "未保存の変更があるため、各ノードの成功/失敗の表示は止めています。" : (d.sync === "stale" ? "directorが保存済みの木を読み込むまで、各ノードの表示は止めています。" : "")}</div>`;
    $("liveWarn").innerHTML = warn;
    applyStatuses();
  }

  // ---------------------------------------------------------------- ピン
  function pinList() { return pinsInfo && Array.isArray(pinsInfo.pins) ? pinsInfo.pins : []; }
  function loadedMap() { return pinsInfo && pinsInfo.loaded_map ? String(pinsInfo.loaded_map) : ""; }
  function pinColor(p) { return p && COLOR_RE.test(String(p.color || "")) ? p.color : "#cccccc"; }
  async function loadPins() {
    try {
      const r = await api("/api/pins");
      if (!r.ok) throw new Error("HTTP " + r.status);
      pinsInfo = r.data;
    } catch (e) { pinsInfo = null; }
    const sig = JSON.stringify(pinsInfo ? [pinsInfo.loaded_map, pinsInfo.pins] : null);
    if (sig !== pinsSig) {
      pinsSig = sig;
      $("pinInfo").textContent = pinsInfo
        ? (loadedMap() ? `ピン: 地図「${loadedMap()}」の ${pinList().length} 個から選べます` : "ピン: 地図が読み込まれていません(ピンへ移動する動作は、地図を読み込むまで待機になります)")
        : "ピン一覧を取得できませんでした";
      if (draft) NKField.defer("treeBox", renderTree);  // 編集中は後回し(編集が終わると自動で描画)
    }
  }

  // ---------------------------------------------------------------- 文書の読み込み・差分・JSON
  function isDirty() { return !!draft && (jsonEdited || JSON.stringify(normalize(draft)) !== JSON.stringify(normalize(serverDoc))); }
  function normalize(d) { return d ? JSON.parse(JSON.stringify(d)) : d; }
  function renderDirty() {
    const bar = $("dirtyBar");
    if (!draft) { bar.innerHTML = ""; return; }
    bar.innerHTML = isDirty() ? `<div class="banner warn">未保存の変更があります。「保存して反映」を押すまでロボットの動作は変わりません。</div>` : "";
    $("minHold").value = draft.min_hold_seconds === undefined ? 10 : draft.min_hold_seconds;
    $("docDesc").value = draft.description || "";
    $("docEnabled").value = draft.enabled === false ? "false" : "true";
    const dis = readOnly;
    ["minHold", "docDesc", "docEnabled", "addType", "addChildBtn", "addSibBtn", "wrapBtn", "exampleSel", "exampleBtn", "saveBtn", "discardBtn", "resetBtn", "jsonApplyBtn"].forEach((x) => { if ($(x)) $(x).disabled = dis; });
    $("jsonBox").readOnly = dis;
    $("validateBtn").disabled = false;
    if (live) applyStatuses();
  }
  function syncJsonBox() { if ($("jsonPane").style.display !== "none" && !jsonEdited) $("jsonBox").value = JSON.stringify(draft, null, 2); }

  function buildTypeSelect() {
    const groups = meta.kinds.map(([k, lab]) => {
      const items = meta.types.filter((t) => t.kind === k);
      return items.length ? `<optgroup label="${escapeHtml(lab)}">${items.map((t) => `<option value="${escapeHtml(t.type)}">${escapeHtml(t.icon)} ${escapeHtml(t.label)}</option>`).join("")}</optgroup>` : "";
    }).join("");
    $("addType").innerHTML = groups;
  }

  async function loadTree() {
    const r = await api("/api/behavior_tree");
    if (!r.ok) { showGlobal("err", "木を取得できませんでした(HTTP " + r.status + ")"); return; }
    const d = r.data;
    meta = d.meta;
    TYPES = {};
    meta.types.forEach((t) => { TYPES[t.type] = t; });
    examples = d.examples || [];
    buildTypeSelect();
    $("exampleSel").innerHTML = examples.map((e) => `<option value="${escapeHtml(e.id)}">${escapeHtml(e.title)}</option>`).join("");
    serverInfo = d;
    serverSig = d.sig;
    serverDoc = clone(d.document || d.default_document);
    draft = clone(serverDoc);
    jsonEdited = false;
    nodeErrors = {};
    if (!nodeAt(selId)) selId = "0";
    let info = d.exists ? `木ファイル: ${d.path}` : `木ファイル: ${d.path}(まだありません=従来のルール表のみの動作。下の既定の木を編集して保存すると作られます)`;
    if (d.exists && !d.valid) info += " ※このファイルは不正です。directorは直前の木/従来動作を維持します。";
    $("fileInfo").textContent = info;
    let msg = "";
    if (d.exists && !d.valid) msg += listHtml("err", "保存済みのファイルが不正です", d.errors);
    msg += listHtml("warn", "注意", (d.warnings || []).concat(d.pin_warnings || []));
    $("pinWarn").innerHTML = msg;
    $("saveErrors").innerHTML = "";
    $("validateResult").innerHTML = "";
    live = d.live || live;
    renderSafety();
    renderTree();
    renderDirty();
    syncJsonBox();
    if (live) renderLive();
  }

  function applyErrors(errors) {
    nodeErrors = {};
    (errors || []).forEach((e) => {
      const m = /ノード([0-9.]+)/.exec(e);
      if (m) (nodeErrors[m[1]] = nodeErrors[m[1]] || []).push(e);
    });
    Object.keys(nodeErrors).forEach((id) => { const p = id.split("."); for (let i = 1; i < p.length; i++) { /* 親は開かない */ } });
    renderTree();
  }

  function currentDocForSend() {
    if ($("jsonPane").style.display !== "none") {
      try { draft = JSON.parse($("jsonBox").value); } catch (e) { throw new Error("JSONとして読めません: " + e.message); }
      jsonEdited = false;
    }
    return draft;
  }

  async function validateDraft() {
    let doc;
    try { doc = currentDocForSend(); } catch (e) { $("validateResult").innerHTML = listHtml("err", "検証できません", [e.message]); return null; }
    const r = await postJson("/api/behavior_tree/validate", { document: doc });
    const d = r.data;
    if (r.status === 403) { $("validateResult").innerHTML = listHtml("err", "検証できません", ["guestアカウントは検証できません(閲覧専用)"]); return null; }
    if (!d.ok) {
      $("validateResult").innerHTML = listHtml("err", "検証NG(保存はできません)", d.errors);
      applyErrors(d.errors);
      return d;
    }
    nodeErrors = {};
    renderTree();
    $("validateResult").innerHTML = `<div class="banner ok">検証OK: ${escapeHtml(d.node_count)}ノード</div>`
      + listHtml("warn", "注意(保存はできます)", (d.warnings || []).concat(d.pin_warnings || []))
      + `<pre class="diff">${escapeHtml((d.outline || []).join("\n"))}</pre>`;
    return d;
  }

  async function save() {
    if (readOnly || busy) return;
    let doc;
    try { doc = currentDocForSend(); } catch (e) { $("saveErrors").innerHTML = listHtml("err", "保存できません", [e.message]); return; }
    busy = true; $("saveBtn").disabled = true; $("saveStatus").textContent = "保存中...";
    for (const x of ["treeBox", "minHold", "docDesc", "docEnabled", "jsonBox"]) { if ($(x)) $(x).inert = true; }  // 保存中は編集不可(読み直しで編集内容が消えるのを防ぐ)
    try {
      const r = await postJson("/api/behavior_tree", { document: doc, expected_sig: serverSig });
      const d = r.data;
      if (!r.ok) {
        const kind = d.kind === "conflict" ? "保存できませんでした(競合)" : "保存できませんでした(何も書き込んでいません)";
        $("saveErrors").innerHTML = listHtml("err", kind, d.errors || [d.message || "エラー"]);
        if (d.kind === "invalid") applyErrors(d.errors);
        $("saveStatus").textContent = "";
        return;
      }
      $("saveErrors").innerHTML = "";
      const sentLines = outlineLines(doc.root, 0, []).join("\n") + "|" + doc.min_hold_seconds + "|" + doc.enabled;
      NKField.bump("tree");
      await loadTree();
      const gotLines = serverDoc && serverDoc.root ? outlineLines(serverDoc.root, 0, []).join("\n") + "|" + serverDoc.min_hold_seconds + "|" + serverDoc.enabled : "";
      $("saveStatus").textContent = d.message || "保存しました";
      if (gotLines !== sentLines) showGlobal("warn", (d.message || "保存しました") + " 注意: 保存した木と読み直した木が一致しません(サーバーで調整された可能性があります)。画面は保存済みの内容です。");
      else showGlobal("ok", d.message || "保存しました");
      verifyTreeSync();
    } catch (e) {
      $("saveErrors").innerHTML = listHtml("err", "保存できませんでした", ["通信エラー(サーバーに接続できません)"]);
    } finally {
      busy = false; $("saveBtn").disabled = readOnly;
      for (const x of ["treeBox", "minHold", "docDesc", "docEnabled", "jsonBox"]) { if ($(x)) $(x).inert = false; }
    }
  }
  // 保存後、behavior_directorが保存済みの木を読み込んだか(synced)を数秒見て確かめる
  async function verifyTreeSync() {
    for (const w of [700, 1200, 2000, 3000]) {
      await new Promise((r) => setTimeout(r, w));
      await pollLive();
      if (live && live.sync === "synced") { $("saveStatus").textContent = "directorが保存した木を読み込みました"; return; }
    }
    $("saveStatus").textContent = (live && live.status === "ok")
      ? "directorはまだ別の木のままです(続く場合は木ファイルのパス設定を確認してください)"
      : "behavior_directorから状態が届かないため、反映を確認できません(起動したときに保存済みの木を読み込みます)";
  }

  async function resetDefault() {
    if (readOnly || busy) return;
    if (!confirm("木のファイルを退避(.bak)して無くし、従来のルール表のみの動作に戻します。よろしいですか？")) return;
    const r = await postJson("/api/behavior_tree/reset", { expected_sig: serverSig });
    if (!r.ok) { $("saveErrors").innerHTML = listHtml("err", "既定に戻せませんでした", r.data.errors || ["エラー"]); return; }
    await loadTree();
    showGlobal("ok", r.data.message || "既定に戻しました");
  }

  // ---------------------------------------------------------------- 比較
  function outlineLines(n, depth, out) {
    const spec = TYPES[n.type] || { label: n.type, icon: "?" };
    out.push("  ".repeat(depth) + `${spec.icon} ${labelOf(n)}${n.enabled === false ? " [無効]" : ""} <${n.type}>` + (n.params ? " " + JSON.stringify(n.params) : ""));
    (n.children || []).forEach((c) => outlineLines(c, depth + 1, out));
    return out;
  }
  function diffLines(a, b) {
    const sa = new Set(a), sb = new Set(b);
    const out = [];
    a.forEach((l) => { if (!sb.has(l)) out.push(`<span class="del">- ${escapeHtml(l)}</span>`); });
    b.forEach((l) => { if (!sa.has(l)) out.push(`<span class="add">+ ${escapeHtml(l)}</span>`); });
    return out;
  }
  async function compare() {
    const r = await api("/api/behavior_tree");
    if (!r.ok) { $("compareBody").innerHTML = listHtml("err", "比較できません", ["取得に失敗しました"]); return; }
    const saved = r.data.document || r.data.default_document;
    const lv = r.data.live || {};
    let html = "";
    if (lv.status === "ok" && lv.state) {
      html += lv.sync === "synced"
        ? `<div class="banner ok">directorが使っている木は、保存済みの木と一致しています(${escapeHtml(lv.state.node_count)}ノード)。</div>`
        : `<div class="banner warn">directorが使っている木は、保存済みの木と<b>一致していません</b>(directorの木: ${escapeHtml(lv.state.node_count)}ノード)。保存直後なら1秒ほどで反映されます。続く場合は木ファイルのパス設定(director/ダッシュボード)を確認してください。</div>`;
      html += `<div class="hint">director: ${lv.state.user_active ? "木を使用中" : "従来動作(ルール表のみ)"}、出している方針 ${escapeHtml(lv.state.policy)}</div>`;
    } else html += `<div class="banner warn">behavior_directorの状態が届いていないため、director側の木とは比較できません。</div>`;
    const savedLines = outlineLines(saved.root, 0, []);
    const draftLines = draft ? outlineLines(draft.root, 0, []) : savedLines;
    const dl = diffLines(savedLines, draftLines);
    html += dl.length ? `<h3>下書きと保存済みの違い</h3><pre class="diff">${dl.join("\n")}</pre>` : `<div class="hint">下書きは保存済みの木と同じです。</div>`;
    $("compareBody").innerHTML = html;
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

  // ---------------------------------------------------------------- イベント・起動
  NKField.guard(["minHold", "docDesc", "docEnabled", "addType", "exampleSel", "jsonBox"]);   // 全体設定・追加/例の選択・JSON欄(値は下書きが持つのでポーリングでは書き換わらない)
  NKField.watch("treeBox");   // 木のノード編集欄(動的に作る入力欄の範囲外表示・編集中は再描画を後回しにする)
  $("jsonBox").addEventListener("input", () => { if (!jsonEdited) { jsonEdited = true; renderDirty(); } });
  $("addChildBtn").addEventListener("click", () => addNode("child"));
  $("addSibBtn").addEventListener("click", () => addNode("sibling"));
  $("wrapBtn").addEventListener("click", () => addNode("wrap"));
  $("minHold").addEventListener("change", () => { if (readOnly) return; const n = toNum($("minHold").value); draft.min_hold_seconds = n === null ? 10 : n; renderDirty(); syncJsonBox(); });
  $("docDesc").addEventListener("change", () => { if (readOnly) return; draft.description = $("docDesc").value; renderDirty(); syncJsonBox(); });
  $("docEnabled").addEventListener("change", () => { if (readOnly) return; draft.enabled = $("docEnabled").value === "true"; renderDirty(); syncJsonBox(); });
  $("exampleBtn").addEventListener("click", () => {
    if (readOnly) return;
    const ex = examples.find((e) => e.id === $("exampleSel").value);
    if (!ex) return;
    if (isDirty() && !confirm("未保存の変更は失われます。例を読み込みますか？(読み込んでも保存するまでロボットの動作は変わりません)")) return;
    draft = clone(ex.document);
    nodeErrors = {}; openEdit.clear(); selId = "0";
    showGlobal("info", `例「${ex.title}」を下書きに読み込みました。${ex.description} ピンを使う例は、行き先のピンを実際のピンに選び直してください。`);
    afterEdit();
  });
  $("validateBtn").addEventListener("click", validateDraft);
  $("saveBtn").addEventListener("click", save);
  $("resetBtn").addEventListener("click", resetDefault);
  $("discardBtn").addEventListener("click", async () => {
    if (isDirty() && !confirm("未保存の変更を破棄して、保存済みの内容に戻しますか？")) return;
    await loadTree();
  });
  $("compareBtn").addEventListener("click", compare);
  function showTab(json) {
    $("visualPane").style.display = json ? "none" : "";
    $("jsonPane").style.display = json ? "" : "none";
    $("tabVisual").classList.toggle("on", !json);
    $("tabJson").classList.toggle("on", json);
    if (json) { $("jsonBox").value = JSON.stringify(draft, null, 2); jsonEdited = false; }
  }
  $("tabVisual").addEventListener("click", () => {
    if ($("jsonPane").style.display !== "none" && !readOnly) {
      try { draft = JSON.parse($("jsonBox").value); } catch (e) { $("validateResult").innerHTML = listHtml("err", "JSONとして読めません(ビジュアルへ戻る前に直してください)", [e.message]); return; }
      jsonEdited = false;
    }
    showTab(false); nodeErrors = {}; if (!nodeAt(selId)) selId = "0"; renderTree(); renderDirty();
  });
  $("tabJson").addEventListener("click", () => showTab(true));
  $("jsonApplyBtn").addEventListener("click", () => {
    try { draft = JSON.parse($("jsonBox").value); } catch (e) { $("validateResult").innerHTML = listHtml("err", "JSONとして読めません", [e.message]); return; }
    if (!draft || typeof draft !== "object" || !draft.root) { $("validateResult").innerHTML = listHtml("err", "反映できません", ["rootが必要です"]); return; }
    nodeErrors = {}; openEdit.clear(); selId = "0"; jsonEdited = false;
    $("validateResult").innerHTML = `<div class="banner info">下書きに反映しました。「検証」で内容を確認できます。</div>`;
    renderTree(); renderDirty();
  });
  window.addEventListener("beforeunload", (e) => { if (isDirty()) { e.preventDefault(); e.returnValue = ""; } });
  document.addEventListener("visibilitychange", () => { if (!document.hidden) { pollLive(); pollEstop(); } });

  (async function init() {
    try {
      const w = await api("/api/whoami");
      const account = (w.data && w.data.account) || "";
      $("accountLabel").textContent = account ? `ログイン: ${account}` : "";
      readOnly = !w.ok || account === "guest" || account === "";
    } catch (e) { readOnly = true; }
    if (readOnly) {
      showGlobal("info", "閲覧専用です(guestアカウント、または認証を確認できませんでした)。内容は見られますが、編集・保存はできません。");
      $("estopBtn").disabled = true;
    }
    await loadPins();
    await loadTree();
    pollLive();
    pollEstop();
    NKField.poll(loadPins, 5000);
    NKField.poll(pollLive, 1500);
    setInterval(pollEstop, 2000);
  })();
</script>
</body>
</html>
"""


def build_behavior_tree_html() -> bytes:
    """ビヘイビアツリーの編集ページ(`/behavior_tree`)。full_dashboard_nodeから配信される。"""
    html = inject_page_guide(_BEHAVIOR_TREE_HTML, PAGE_TREE)  # 【2026-10-23】3ページの説明バナー+機体の時計(足すだけ)
    return html.replace("__NK_FIELD_JS__", NK_FIELD_JS).encode("utf-8")
