"""状況アナウンス設定ページ(`/announce_settings`)のHTML。

【2026-10-13追加】docs/announcements.md。full_dashboard_nodeから配信される。
バックエンドは announce_events_proxy.py(/api/announce_events と /api/announce_events/test)。
【2026-10-30】末尾の「文化祭の出店紹介」カードは pr_intro.py(/api/pr_intro、/api/pr_intro/prepare、/api/pr_intro/test)。上のイベントとは
独立に保存し(編集中は定期更新で入力を上書きしない)、カードのHTMLは全てJSで作る。
ページはROSにも2台目にも直接触れず、ダッシュボードのHTTP APIだけを使う。

設計メモ:
- イベントのカードは、サーバーが返す meta(固定のイベント一覧)から一度だけ作る。値の反映は NKField.set(編集中は上書きしない)。
- 保存は「変更した欄だけ」を送り、NKField.begin -> tx.verify で1台目の設定と2台目の音量を読み戻して一致を確認する。
- 音量が2倍を超える保存は確認バー(実行する/やめる)を挟む(音量カードと同じ流儀)。
- 全ての可変テキストは escapeHtml() を通す。guestは閲覧のみ(入力欄・ボタン無効、サーバー側もPOSTは403)。
"""

from .nav_layout import NK_FIELD_JS

_ANNOUNCE_HTML = r"""<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>のこさへん - 状況アナウンス</title>
<style>
  :root { color-scheme: light; font-family: system-ui, -apple-system, "Segoe UI", sans-serif; background:#f4f6f5; color:#16211d; }
  html, body { margin:0; }
  body { padding-bottom:150px; }
  header { box-sizing:border-box; min-height:44px; padding:6px 18px; background:#124739; color:#fff; display:flex; justify-content:space-between; align-items:center; gap:8px; flex-wrap:wrap; }
  header a { color:#cfe9df; text-decoration:none; font-size:13px; }
  header h1 { margin:0; font-size:18px; }
  main { max-width:1000px; margin:0 auto; padding:14px; display:flex; flex-direction:column; gap:14px; }
  section.card { background:#fff; border:1px solid #dce4df; border-radius:8px; padding:12px 14px; }
  h2 { margin:0 0 8px; font-size:15px; }
  h2.sect { margin:6px 2px -4px; font-size:17px; color:#124739; padding-bottom:4px; border-bottom:2px solid #176b56; scroll-margin-top:8px; }
  h2.sect small { font-weight:400; font-size:13px; color:#53635c; margin-left:8px; }
  nav.jump { display:flex; gap:8px; flex-wrap:wrap; align-items:center; background:#fff; border:1px solid #dce4df; border-radius:8px; padding:10px 14px; }
  nav.jump a { padding:6px 12px; border-radius:6px; background:#e1f5ee; color:#0f6e56; text-decoration:none; font-size:14px; font-weight:700; }
  nav.jump .now { font-size:14px; color:#33463e; margin-left:auto; }
  .modes { display:grid; grid-template-columns:repeat(auto-fit,minmax(240px,1fr)); gap:10px; }
  label.mode { display:grid; grid-template-columns:22px 1fr; column-gap:8px; align-items:start; padding:10px 12px; border:2px solid #dce4df; border-radius:8px; background:#fbfcfb; cursor:pointer; font-size:14px; color:#16211d; }
  label.mode input { width:18px; height:18px; margin:2px 0 0; }
  label.mode b { font-size:15px; }
  label.mode small { grid-column:2; color:#53635c; font-size:13px; line-height:1.5; margin-top:2px; }
  label.mode:has(input:checked) { border-color:#176b56; background:#e1f5ee; }
  label.mode:has(input:focus-visible) { outline:3px solid #7fc4b0; }
  .hint { font-size:13px; color:#53635c; margin:6px 0 0; line-height:1.5; }
  .row { display:flex; gap:8px; flex-wrap:wrap; align-items:center; }
  button { padding:8px 12px; font-size:13px; border:0; border-radius:6px; background:#176b56; color:#fff; cursor:pointer; }
  button.secondary { background:#5d6f67; }
  button.small { padding:4px 9px; font-size:12px; }
  button:disabled { opacity:0.45; cursor:not-allowed; }
  input[type="number"], input[type="text"], select { padding:6px; font-size:14px; border:1px solid #cfd8d3; border-radius:6px; box-sizing:border-box; max-width:100%; font-family:inherit; }
  input:disabled, select:disabled { background:#f1f3f2; color:#555; }
  label { font-size:13px; color:#33463e; }
  .banner { padding:8px 12px; border-radius:6px; font-size:13px; line-height:1.5; }
  .banner.err { background:#faece7; color:#993c1d; }
  .banner.ok { background:#e1f5ee; color:#0f6e56; }
  .banner.warn { background:#fff3cd; color:#7a5b00; }
  .banner.info { background:#eef3f0; color:#33463e; }
  .banner ul { margin:4px 0 0 18px; padding:0; }
  .pill { display:inline-block; padding:2px 9px; border-radius:999px; font-size:12px; font-weight:700; }
  .pill.ok { background:#e1f5ee; color:#0f6e56; }
  .pill.bad { background:#faece7; color:#993c1d; }
  .pill.wait { background:#fff3cd; color:#7a5b00; }
  .pill.off { background:#e4e8e6; color:#53635c; }
  .fields { display:grid; grid-template-columns:repeat(auto-fit,minmax(200px,1fr)); gap:10px; margin-top:8px; }
  .field { display:flex; flex-direction:column; gap:3px; min-width:0; }
  .field input[type="text"], .field input[type="number"], .field select { width:100%; }
  .field.wide { grid-column:1 / -1; }
  .field small { color:#53635c; font-size:12px; line-height:1.4; }
  .vrow { display:flex; gap:8px; align-items:center; }
  .vrow input[type="range"] { flex:1 1 auto; min-width:0; }
  .vrow input[type="number"] { width:80px; flex:none; }
  .toggle { display:flex; gap:8px; align-items:flex-start; padding:6px 8px; border:1px solid #dce4df; border-radius:8px; background:#fbfcfb; font-size:13px; }
  .fields .toggle { align-self:end; align-items:center; min-height:38px; box-sizing:border-box; }
  .toggle input { width:18px; height:18px; margin-top:1px; flex:none; }
  .evhead { display:flex; justify-content:space-between; gap:8px; align-items:center; flex-wrap:wrap; }
  .evhead label.master { display:flex; gap:8px; align-items:center; font-size:15px; font-weight:700; color:#124739; }
  .evhead input[type="checkbox"] { width:20px; height:20px; }
  .evstat { font-size:13px; color:#53635c; margin-top:8px; line-height:1.6; }
  #saveBar { position:fixed; left:0; right:0; bottom:68px; z-index:1900; box-sizing:border-box; padding:8px 12px; background:#fff; border-top:1px solid #cfd8d3; box-shadow:0 -2px 8px rgba(0,0,0,0.12); display:flex; gap:10px; align-items:center; flex-wrap:wrap; }
  #saveBar .msg { flex:1 1 200px; font-size:13px; color:#53635c; }
  #confirmBar { display:none; width:100%; box-sizing:border-box; padding:8px 10px; background:#fff3cd; color:#7a5b00; border-radius:6px; font-size:13px; gap:8px; align-items:center; flex-wrap:wrap; }
  #confirmBar.show { display:flex; }
  #estopDock { position:fixed; left:0; right:0; bottom:0; z-index:2000; box-sizing:border-box; padding:8px 12px; background:rgba(18,71,57,0.96); box-shadow:0 -2px 10px rgba(0,0,0,0.3); display:flex; gap:10px; align-items:center; }
  #estopDock .stop-btn { flex:1 1 auto; margin:0; padding:14px; font-size:18px; min-height:52px; border:2px solid #fff; font-weight:700; background:#a32d2d; color:#fff; border-radius:8px; }
  #estopDock .stop-btn.engaged { background:#5c1717; }
  #estopDock a { color:#cfe9df; font-size:12px; white-space:nowrap; }
  @media (max-width:640px) { main { padding:8px; } }
</style>
</head>
<body>
<header>
  <h1>状況アナウンス</h1>
  <div class="row">
    <span id="accountLabel" style="font-size:12px;opacity:0.85;"></span>
    <a href="/">&larr; 操作盤に戻る</a>
  </div>
</header>
<main>
  <div id="globalMsg"></div>

  <nav class="jump" aria-label="このページの内容">
    <a href="#secEvents">状況アナウンス(移動中・探索中など)</a>
    <a href="#secPr">出店紹介(文化祭)</a>
    <span class="now" id="modeNow"></span>
  </nav>

  <h2 class="sect" id="secEvents">状況アナウンス<small>ロボットの状況に合わせて流す、短いお知らせ。変更は画面下の「保存して反映」</small></h2>
  <section class="card">
    <div class="evhead">
      <label class="master" for="aeMaster"><input id="aeMaster" type="checkbox" /> 状況アナウンス全体をONにする</label>
      <span id="bridgeBadge" class="pill wait">確認中...</span>
    </div>
    <p class="hint">「探索中です」のような短いお知らせを、ロボットの状況に合わせて声で流します。ロボットの動きには一切影響しません。
      一度合成した音声はキャッシュして使い回すので、同じ言葉は何度でもすぐに鳴ります。保存するまでは何も鳴りません(保存済みの設定が無い間は全体OFF)。</p>
    <div id="fileInfo" class="hint"></div>
    <div class="fields">
      <div class="field">
        <label for="aeVolNum">アナウンスの基準音量(倍率 0〜5)</label>
        <div class="vrow">
          <input id="aeVol" type="range" min="0" max="5" step="0.05" />
          <input id="aeVolNum" type="number" min="0" max="5" step="0.05" inputmode="decimal" />
        </div>
        <small>各イベントの音量(下)と掛け算で決まります(最大5倍で頭打ち)。2台目の発話・BGMの音量と同じ保存先です。2倍を超えると保存時に確認が出ます。</small>
      </div>
    </div>
  </section>

  <div id="events"></div>
  <h2 class="sect" id="secPr">出店紹介(文化祭)<small>クラスの店・ロボットの特徴を順番に流す。変更はこの下のカードの「保存して音声を作る」</small></h2>
  <div id="prIntro"></div>
</main>

<div id="saveBar">
  <div id="confirmBar" role="alert">
    <span id="confirmText"></span>
    <button id="confirmYes" type="button">実行する</button>
    <button id="confirmNo" type="button" class="secondary">やめる</button>
  </div>
  <button id="saveBtn" type="button">保存して反映</button>
  <button id="discardBtn" type="button" class="secondary">変更を破棄</button>
  <span class="msg" id="saveMsg">状況アナウンス: 変更はありません</span>
</div>
<div id="estopDock">
  <button id="estopBtn" class="stop-btn" type="button">緊急停止(ソフト)</button>
  <a href="/">操作盤へ</a>
</div>

<script>
__NK_FIELD_JS__
  (function () {
    const rawFetch = window.fetch.bind(window);
    window.fetch = async (...args) => {
      const res = await rawFetch(...args);
      if (res.status === 401 && location.pathname !== "/login") { location.href = "/login"; }
      return res;
    };
  })();

  function escapeHtml(value) {
    return String(value === null || value === undefined ? "" : value).replace(/[&<>"']/g, (ch) => ({
      "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"
    }[ch]));
  }
  const $ = (id) => document.getElementById(id);
  const CONFIRM_ABOVE = 2;
  // 欄の定義: key=サーバーの項目名, sfx=要素idの末尾, kind=型
  const FIELDS = [
    { key: "enabled", sfx: "enabled", kind: "check" },
    { key: "text", sfx: "text", kind: "text" },
    { key: "mode", sfx: "mode", kind: "select" },
    { key: "interval_sec", sfx: "interval", kind: "num" },
    { key: "volume", sfx: "volume", kind: "num", slider: true },
    { key: "cooldown_sec", sfx: "cooldown", kind: "num" },
    { key: "min_active_sec", sfx: "minact", kind: "num" },
    { key: "quiet_when_person_near", sfx: "quiet", kind: "check" },
    { key: "during_survey", sfx: "survey", kind: "check" },
    { key: "during_estop", sfx: "estop", kind: "check" },
  ];
  const fid = (eid, sfx) => "ev_" + eid + "_" + sfx;

  let readOnly = true;
  let busy = false;
  let meta = null;
  let loaded = null;   // 最新の /api/announce_events
  let built = false;

  async function api(path, opts) {
    const res = await fetch(path, opts);
    let data = {};
    try { data = await res.json(); } catch (e) { data = {}; }
    return { ok: res.ok, status: res.status, data };
  }
  function postJson(path, body) {
    return api(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  }
  function showGlobal(kind, text) {
    $("globalMsg").innerHTML = text ? `<div class="banner ${kind}">${escapeHtml(text)}</div>` : "";
  }
  function errorsHtml(title, errors) {
    const items = (errors || []).map((e) => `<li>${escapeHtml(e)}</li>`).join("");
    return `<div class="banner err"><b>${escapeHtml(title)}</b><ul>${items}</ul></div>`;
  }

  function buildCards() {
    const L = meta.limits;
    const html = meta.events.map((ev) => {
      const id = ev.id;
      return `<section class="card" id="card_${id}">
        <div class="evhead">
          <label class="master" for="${fid(id, "enabled")}"><input id="${fid(id, "enabled")}" type="checkbox" /> ${escapeHtml(ev.label)}</label>
          <span class="row"><span class="pill off" id="pill_${id}">-</span>
            <button type="button" class="secondary small" id="test_${id}">試しに鳴らす</button></span>
        </div>
        <p class="hint">${escapeHtml(ev.desc)}<br>きっかけ: ${escapeHtml(ev.trigger_desc)}${ev.urgent ? "(緊急用: 発話の待ちを無視して先に鳴ります)" : ""}</p>
        <div class="fields">
          <div class="field wide"><label for="${fid(id, "text")}">言う内容(${L.text_max}文字まで)</label>
            <input id="${fid(id, "text")}" type="text" maxlength="${L.text_max}" /></div>
          <div class="field"><label for="${fid(id, "mode")}">モード</label>
            <select id="${fid(id, "mode")}"><option value="loop">繰り返す(状況が続く間)</option><option value="once">始まった時に1回</option></select></div>
          <div class="field"><label for="${fid(id, "interval")}">間隔(秒、繰り返しのみ)</label>
            <input id="${fid(id, "interval")}" type="number" min="${L.interval[0]}" max="${L.interval[1]}" step="1" inputmode="decimal" /></div>
          <div class="field"><label for="${fid(id, "volumeNum")}">このイベントの音量(倍率)</label>
            <div class="vrow"><input id="${fid(id, "volume")}" type="range" min="0" max="${L.volume[1]}" step="0.05" />
            <input id="${fid(id, "volumeNum")}" type="number" min="0" max="${L.volume[1]}" step="0.05" inputmode="decimal" /></div></div>
          <div class="field"><label for="${fid(id, "cooldown")}">クールダウン(秒、1回のみ)</label>
            <input id="${fid(id, "cooldown")}" type="number" min="0" max="${L.cooldown[1]}" step="1" inputmode="decimal" /></div>
          <div class="field"><label for="${fid(id, "minact")}">続いてから鳴らす(秒)</label>
            <input id="${fid(id, "minact")}" type="number" min="0" max="${L.min_active[1]}" step="0.5" inputmode="decimal" /></div>
        </div>
        <div class="fields">
          <label class="toggle"><input id="${fid(id, "quiet")}" type="checkbox" /><span>人が近いときは出さない</span></label>
          <label class="toggle"><input id="${fid(id, "survey")}" type="checkbox" /><span>アンケート回答中でも出す</span></label>
          <label class="toggle"><input id="${fid(id, "estop")}" type="checkbox" /><span>緊急停止中でも出す</span></label>
        </div>
        <div class="evstat" id="stat_${id}">-</div>
      </section>`;
    }).join("");
    $("events").innerHTML = html;
    const guardIds = ["aeMaster", "aeVol", "aeVolNum"];
    NKField.link("aeVol", "aeVolNum");
    for (const ev of meta.events) {
      for (const f of FIELDS) guardIds.push(fid(ev.id, f.sfx));
      NKField.link(fid(ev.id, "volume"), fid(ev.id, "volumeNum"));
      guardIds.push(fid(ev.id, "volumeNum"));
      $("test_" + ev.id).addEventListener("click", () => testEvent(ev.id));
      $(fid(ev.id, "mode")).addEventListener("change", () => syncModeEnable(ev.id));
    }
    NKField.guard(guardIds);
    built = true;
  }
  function syncModeEnable(id) {
    const loop = $(fid(id, "mode")).value === "loop";
    $(fid(id, "interval")).disabled = readOnly || !loop;
    $(fid(id, "cooldown")).disabled = readOnly || loop;
  }
  function applyDisabled() {
    $("aeMaster").disabled = readOnly || busy;
    for (const x of ["aeVol", "aeVolNum"]) $(x).disabled = readOnly || busy;
    $("saveBtn").disabled = readOnly || busy;
    $("discardBtn").disabled = readOnly || busy;
    $("estopBtn").disabled = readOnly;
    if (!built) return;
    for (const ev of meta.events) {
      for (const f of FIELDS) $(fid(ev.id, f.sfx)).disabled = readOnly || busy;
      $(fid(ev.id, "volumeNum")).disabled = readOnly || busy;
      $("test_" + ev.id).disabled = readOnly || busy;
      syncModeEnable(ev.id);
    }
  }

  function fmtTime(ts) {
    if (!ts) return "まだ鳴っていません";
    const d = new Date(ts * 1000), ago = Math.max(0, Math.round(Date.now() / 1000 - ts));
    return d.toLocaleTimeString("ja-JP") + "(" + (ago < 90 ? ago + "秒前" : Math.round(ago / 60) + "分前") + ")";
  }
  function renderStatus(data) {
    const cfg = data.config, br = data.bridge;
    const b = $("bridgeBadge");
    if (br && br.ok) {
      const st = br.state;
      b.className = "pill " + (st.muted ? "wait" : "ok");
      b.textContent = st.muted ? "2台目: 音声が全体OFF(鳴りません)" : "2台目: 接続OK(キャッシュ " + st.cache.items + "件)";
    } else { b.className = "pill bad"; b.textContent = "2台目に接続できません"; }
    $("fileInfo").textContent = cfg.exists
      ? "設定ファイル: " + cfg.path + (cfg.valid ? "" : "(不正: " + (cfg.errors || []).join(" / ") + " 直前の有効な設定を使用中)")
      : "設定ファイルはまだありません(下は「おすすめの既定値」です。保存すると有効になります)";
    for (const ev of meta.events) {
      const c = cfg.events[ev.id];
      const pill = $("pill_" + ev.id);
      let stat = [];
      if (br && br.ok) {
        const st = br.state;
        const cached = st.cached && st.cached[c.text];
        stat.push("音声キャッシュ: " + (cached ? "済(すぐ鳴ります)" : "未(初回は合成に数秒かかります)"));
        const lp = st.loops && st.loops[ev.id];
        const once = st.once && st.once[ev.id];
        let last = lp && lp.last_played_ts ? lp.last_played_ts : (once ? once.last_played_ts : null);
        stat.push("最後に鳴った時刻: " + fmtTime(last));
        if (lp && lp.active) {
          stat.push("繰り返し再生中(次まで約" + lp.next_in_sec + "秒)" + (lp.text !== c.text ? " ※2台目はまだ古い言葉です(数秒で更新)" : ""));
        }
        if (lp && lp.last_error) stat.push("2台目のエラー: " + lp.last_error);
        pill.className = "pill " + (lp && lp.active ? "ok" : (c.enabled && cfg.enabled ? "wait" : "off"));
        pill.textContent = lp && lp.active ? "鳴動中" : (!cfg.enabled ? "全体OFF" : (c.enabled ? "待機中" : "OFF"));
      } else {
        stat.push("2台目の状態を取得できません");
        pill.className = "pill off"; pill.textContent = c.enabled ? "ON" : "OFF";
      }
      $("stat_" + ev.id).innerHTML = escapeHtml(stat.join(" / "));
    }
  }
  function applyConfig(data) {
    const cfg = data.config;
    NKField.set("aeMaster", cfg.enabled);
    const v = data.bridge && data.bridge.ok ? data.bridge.announce_volume : null;
    if (typeof v === "number") { NKField.set("aeVol", v); NKField.set("aeVolNum", v); }
    for (const ev of meta.events) {
      const c = cfg.events[ev.id];
      for (const f of FIELDS) {
        NKField.set(fid(ev.id, f.sfx), c[f.key]);
        if (f.slider) NKField.set(fid(ev.id, "volumeNum"), c[f.key]);
      }
      syncModeEnable(ev.id);
    }
  }
  async function loadAll() {
    const t = NKField.ticket("announce");
    const r = await api("/api/announce_events");
    if (!NKField.fresh(t)) return false;
    if (!r.ok || !r.data || !r.data.config) { showGlobal("err", "設定を読み込めません(HTTP " + r.status + ")"); return false; }
    loaded = r.data;
    if (!meta) meta = r.data.meta;
    if (!built) { buildCards(); applyDisabled(); }
    applyConfig(r.data);
    renderStatus(r.data);
    renderDirty();
    return true;
  }

  function allFieldEls() {
    const els = [$("aeMaster"), $("aeVol"), $("aeVolNum")];
    if (built) for (const ev of meta.events) { for (const f of FIELDS) els.push($(fid(ev.id, f.sfx))); els.push($(fid(ev.id, "volumeNum"))); }
    return els;
  }
  function isDirty() { return built && NKField.dirtyOf(allFieldEls()).length > 0; }
  function renderDirty() {
    $("saveMsg").textContent = isDirty() ? "状況アナウンス: 未保存の変更があります(出店紹介は別に保存します)" : "状況アナウンス: 変更はありません";
  }
  function parseVal(el, f) {
    if (f.kind === "check") return el.checked;
    if (f.kind === "num") return el.value.trim() === "" ? NaN : Number(el.value);
    return el.value;
  }
  // 変更した欄だけ集める。items=読み戻しで照合する対象
  function buildChanges() {
    const body = {}, items = [], errors = [];
    let maxVol = 0;
    if (NKField.isDirty("aeMaster")) {
      body.enabled = $("aeMaster").checked;
      items.push({ el: "aeMaster", sent: body.enabled, read: (s) => s.config.enabled, label: "全体ON/OFF" });
    }
    if (NKField.isDirty("aeVol") || NKField.isDirty("aeVolNum")) {
      const v = Number($("aeVolNum").value);
      if (!Number.isFinite(v) || v < 0 || v > 5) errors.push("基準音量は0〜5の範囲で指定してください");
      else { body.announce_volume = v; maxVol = Math.max(maxVol, v); items.push({ el: "aeVolNum", sent: v, read: (s) => s.bridge && s.bridge.ok ? s.bridge.announce_volume : undefined, label: "基準音量" }); }
    }
    if (built) for (const ev of meta.events) {
      for (const f of FIELDS) {
        const el = $(fid(ev.id, f.sfx));
        if (!NKField.isDirty(el) && !(f.slider && NKField.isDirty(fid(ev.id, "volumeNum")))) continue;
        const src = f.slider ? $(fid(ev.id, "volumeNum")) : el;
        const val = parseVal(src, f);
        if (f.kind === "num" && !Number.isFinite(val)) { errors.push(ev.label + ": 数値で入力してください"); continue; }
        if (f.kind === "text" && !String(val).trim()) { errors.push(ev.label + ": 言う内容が空です"); continue; }
        body.events = body.events || {};
        (body.events[ev.id] = body.events[ev.id] || {})[f.key] = f.kind === "text" ? String(val).trim() : val;
        if (f.key === "volume") maxVol = Math.max(maxVol, val);
        const key = f.key;
        items.push({ el: src, sent: f.kind === "text" ? String(val).trim() : val, read: (s) => s.config.events[ev.id][key], label: ev.label + "/" + key });
      }
    }
    return { body, items, errors, maxVol };
  }
  function askConfirm(text) {
    return new Promise((resolve) => {
      $("confirmText").textContent = text;
      $("confirmBar").classList.add("show");
      const done = (v) => { $("confirmBar").classList.remove("show"); $("confirmYes").onclick = null; $("confirmNo").onclick = null; resolve(v); };
      $("confirmYes").onclick = () => done(true);
      $("confirmNo").onclick = () => done(false);
    });
  }
  async function save() {
    if (readOnly || busy) return;
    const { body, items, errors, maxVol } = buildChanges();
    if (errors.length) { $("globalMsg").innerHTML = errorsHtml("保存できません", errors); return; }
    if (!items.length) { showGlobal("info", "変更はありません"); return; }
    if (maxVol > CONFIRM_ABOVE && !(await askConfirm("音量が" + CONFIRM_ABOVE + "倍を超えています(最大 " + maxVol + " 倍)。大きな音が出ます。実行しますか?"))) return;
    busy = true; applyDisabled();
    $("saveMsg").textContent = "保存中...";
    const tx = NKField.begin("announce", items.map((i) => i.el));
    try {
      const r = await postJson("/api/announce_events", body);
      if (r.ok && r.data.ok) {
        const res = await tx.verify({ items, load: async () => { const x = await api("/api/announce_events"); return x.ok && x.data && x.data.config ? x.data : null; }, delays: [0, 700, 1500, 3000] });
        if (res.state === "ok" || res.state === "clamped") showGlobal("ok", (r.data.message || "保存しました") + "(1台目の設定と2台目の音量を読み戻して一致を確認しました。ロボット側へは約1秒で反映されます)");
        else showGlobal("warn", "保存を送信しましたが、読み戻した値が一致しない項目があります。入力欄はそのままにしています(各欄の注記を確認してください)。");
        await loadAll();
      } else {
        tx.fail();
        const list = r.data.errors && r.data.errors.length ? r.data.errors : [r.data.message || r.data.error || ("保存できませんでした(HTTP " + r.status + ")")];
        $("globalMsg").innerHTML = errorsHtml("保存できませんでした", list);
      }
    } catch (e) {
      tx.fail();
      showGlobal("err", "保存に失敗しました(通信エラー)。保存されたかどうか分からないため、ページを再読み込みして確認してください。");
    } finally {
      busy = false; applyDisabled(); renderDirty();
    }
  }
  async function testEvent(id) {
    if (readOnly || busy) return;
    const text = $(fid(id, "text")).value.trim();
    const vol = Number($(fid(id, "volumeNum")).value);
    const r = await postJson("/api/announce_events/test", { id, text, volume: vol });
    if (r.ok && r.data.ok) showGlobal(r.data.played === false ? "warn" : "info", r.data.message || "鳴らします");
    else $("globalMsg").innerHTML = errorsHtml("試し再生できませんでした", r.data.errors && r.data.errors.length ? r.data.errors : [r.data.message || ("HTTP " + r.status)]);
  }
  $("saveBtn").addEventListener("click", save);
  $("discardBtn").addEventListener("click", async () => {
    if (isDirty() && !confirm("未保存の変更を破棄して、保存済みの内容に戻しますか？")) return;
    NKField.clean(allFieldEls());
    if (loaded) applyConfig(loaded);
    await loadAll();
  });
  document.addEventListener("input", () => renderDirty());

  // ---------------------------------------------------------------- 文化祭の出店紹介(2026-10-30、docs/announcements.md)
  // 上の「イベントのカード」とは独立した保存(POST /api/pr_intro)。文章を保存すると2台目が音声を1回だけ作って保存し、
  // 以降は保存済みの音声を順に繰り返すだけ(再生中は合成しない)。編集中(dirty)は定期更新で入力を上書きせず、状態表示だけ更新する。
  const PR = { data: null, form: null, sig: null, dirty: false, busy: false };
  const PR_STATE = {
    ready: ["ok", "音声: 作成済み"], queued: ["wait", "音声: 順番待ち"], synthesizing: ["wait", "音声: 作成中..."],
    failed: ["bad", "音声: 失敗"], missing: ["off", "音声: 未作成"],
  };
  const prClone = (x) => JSON.parse(JSON.stringify(x));
  function prLimits() { return PR.data.meta.limits; }
  function prBuild() {
    const L = prLimits(), f = PR.form;
    const parts = f.parts.map((p, i) => `<div class="card" style="margin:8px 0;background:#fbfcfb;" data-part="${i}">
        <div class="evhead">
          <label class="master" style="font-size:14px;"><input type="checkbox" data-f="enabled" data-i="${i}" ${p.enabled ? "checked" : ""} /> パート${i + 1}</label>
          <span class="row"><span class="pill off" id="pr_st_${i}">-</span>
            <button type="button" class="secondary small" data-act="up" data-i="${i}" title="上へ">▲</button>
            <button type="button" class="secondary small" data-act="down" data-i="${i}" title="下へ">▼</button>
            <button type="button" class="secondary small" data-act="test" data-i="${i}">試し再生</button>
            <button type="button" class="secondary small" data-act="del" data-i="${i}">削除</button></span>
        </div>
        <div class="fields">
          <div class="field"><label>見出し(この画面だけで使う・${L.title_max}文字まで)</label>
            <input type="text" maxlength="${L.title_max}" data-f="title" data-i="${i}" value="${escapeHtml(p.title)}" /></div>
          <div class="field"><label>このパートの音量(倍率)</label>
            <input type="number" min="0" max="${L.volume[1]}" step="0.05" inputmode="decimal" data-f="volume" data-i="${i}" value="${escapeHtml(p.volume)}" /></div>
          <div class="field wide"><label>読み上げる文章(${L.text_max}文字まで)</label>
            <textarea rows="4" maxlength="${L.text_max}" data-f="text" data-i="${i}" style="width:100%;box-sizing:border-box;font:inherit;padding:6px;border:1px solid #cfd8d3;border-radius:6px;">${escapeHtml(p.text)}</textarea>
            <small id="pr_cnt_${i}"></small></div>
        </div>
      </div>`).join("");
    $("prIntro").innerHTML = `<section class="card" id="prCard">
        <div class="evhead">
          <label class="master" for="pr_enabled"><input id="pr_enabled" type="checkbox" ${f.enabled ? "checked" : ""} /> 文化祭の出店紹介をONにする</label>
          <span id="pr_badge" class="pill off">-</span>
        </div>
        <p class="hint">パート1→パート2→…の順に流し、一巡したら間隔をあけて最初に戻ります。文章を保存すると、2台目がその時だけ音声を作って保存し(長い文章は数十秒〜数分)、
          あとは保存した音声を再生するだけです。移動中の注意・探索中・呼びかけ・アンケート回答中・緊急停止中などは流さず、終わって数秒後に再開します。
          <b>保存するまで鳴りません。既定はOFFです。</b></p>
        <div id="pr_warn"></div>
        <div class="fields">
          <div class="field wide"><label id="pr_mode_lbl">平常時に流す声(どちらか片方だけ。同時には流れません)</label>
            <div class="modes" role="radiogroup" aria-labelledby="pr_mode_lbl">
              <label class="mode"><input type="radio" name="pr_mode" value="menu" ${f.speech_mode !== "pr" ? "checked" : ""} /><b>メニュー紹介</b>
                <small>人に近づいて、おすすめのメニューを声かけします。出店紹介は流れません。</small></label>
              <label class="mode"><input type="radio" name="pr_mode" value="pr" ${f.speech_mode === "pr" ? "checked" : ""} /><b>出店紹介</b>
                <small>下のパートを順番に流します。上の「ONにする」も入れてください。メニュー紹介の声かけ(人に近づくときも)は止まります。</small></label>
            </div>
            <small>保存して約1秒で切り替わります。</small></div>
          <div class="field"><label for="pr_interval">一巡したあと、最初に戻るまでの間隔(秒、${L.interval[0]}〜${L.interval[1]})</label>
            <input id="pr_interval" type="number" min="${L.interval[0]}" max="${L.interval[1]}" step="1" inputmode="decimal" value="${escapeHtml(f.interval_sec)}" /></div>
          <div class="field"><label for="pr_gap">次のパートまでの間隔(秒、${L.gap[0]}〜${L.gap[1]})</label>
            <input id="pr_gap" type="number" min="${L.gap[0]}" max="${L.gap[1]}" step="1" inputmode="decimal" value="${escapeHtml(f.gap_sec)}" /></div>
          <div class="field"><label for="pr_volume">紹介全体の音量(倍率 0〜${L.volume[1]})</label>
            <input id="pr_volume" type="number" min="0" max="${L.volume[1]}" step="0.05" inputmode="decimal" value="${escapeHtml(f.volume)}" />
            <small>パートごとの音量・上の「アナウンスの基準音量」と掛け算で決まります(最大5倍で頭打ち)。</small></div>
          <label class="toggle"><input id="pr_quiet" type="checkbox" ${f.quiet_when_person_near ? "checked" : ""} /><span>人が近いときは流さない</span></label>
        </div>
        <div id="pr_parts">${parts}</div>
        <div class="row" style="margin-top:6px;">
          <button type="button" class="secondary small" id="pr_add">パートを追加</button>
          <span class="hint" id="pr_total"></span>
        </div>
        <div class="row" style="margin-top:10px;">
          <button type="button" id="pr_save">保存して音声を作る</button>
          <button type="button" class="secondary" id="pr_prepare">音声を作り直す(失敗したとき)</button>
          <button type="button" class="secondary" id="pr_discard">変更を破棄</button>
          <span class="hint" id="pr_msg"></span>
        </div>
        <div class="evstat" id="pr_stat">-</div>
      </section>`;
    prCounts();
    prApply();
    prStatus();
  }
  function prCollect() {
    if (!PR.form || !$("pr_enabled")) return;
    const f = PR.form;
    f.enabled = $("pr_enabled").checked;
    f.interval_sec = $("pr_interval").value.trim() === "" ? NaN : Number($("pr_interval").value);
    f.gap_sec = $("pr_gap").value.trim() === "" ? NaN : Number($("pr_gap").value);
    f.volume = $("pr_volume").value.trim() === "" ? NaN : Number($("pr_volume").value);
    f.quiet_when_person_near = $("pr_quiet").checked;
    { const m = $("prCard").querySelector('input[name="pr_mode"]:checked'); if (m) f.speech_mode = m.value; }
    $("pr_parts").querySelectorAll("[data-f]").forEach((el) => {
      const p = f.parts[Number(el.dataset.i)];
      if (!p) return;
      const k = el.dataset.f;
      p[k] = k === "enabled" ? el.checked : (k === "volume" ? (el.value.trim() === "" ? NaN : Number(el.value)) : el.value);
    });
  }
  function prCounts() {
    if (!PR.form || !$("pr_total")) return;
    const L = prLimits();
    let total = 0;
    PR.form.parts.forEach((p, i) => {
      const n = String(p.text || "").length;
      total += n;
      const el = $("pr_cnt_" + i);
      if (el) el.textContent = n + " / " + L.text_max + "文字" + (/^\s*[(（]仮[)）]/.test(p.text || "") ? "(「(仮)」で始まる仮の文です。本番の文章に差し替えてください)" : "");
    });
    $("pr_total").textContent = "全パートの合計 " + total + " / " + L.total_text_max + " 文字(パートは最大" + L.parts_max + "個)";
  }
  function prApply() {
    if (!$("pr_enabled")) return;
    if ($("modeNow") && PR.form) $("modeNow").textContent = "平常時に流す声: " + (PR.form.speech_mode === "pr" ? "出店紹介" : "メニュー紹介") + (PR.dirty ? "(未保存)" : "");
    const dis = readOnly || PR.busy;
    $("prCard").querySelectorAll("input,textarea,select,button").forEach((el) => { el.disabled = dis; });
    // 試し再生は、保存済みの文章・音声でのみ(編集中は無効)
    $("prCard").querySelectorAll('[data-act="test"]').forEach((el) => { el.disabled = dis || PR.dirty; });
    $("pr_add").disabled = dis || (PR.form.parts.length >= prLimits().parts_max);
    $("pr_discard").disabled = dis || !PR.dirty;
    $("pr_prepare").disabled = dis || PR.dirty;
    $("pr_msg").textContent = PR.dirty ? "未保存の変更があります" : "";
  }
  function prStatus() {
    const d = PR.data;
    if (!d || !$("pr_badge")) return;
    const br = d.bridge, badge = $("pr_badge");
    const loop = br && br.ok ? br.loop : null;
    if (!br || !br.ok) { badge.className = "pill bad"; badge.textContent = "2台目に接続できません"; }
    else if (loop && loop.active) { badge.className = "pill ok"; badge.textContent = "再生中(次はパート" + ((loop.next_part || 0) + 1) + ")"; }
    else if (d.config.enabled) { badge.className = "pill wait"; badge.textContent = "待機中(状況により停止)"; }
    else { badge.className = "pill off"; badge.textContent = "OFF"; }
    const warns = [];
    if (!d.exists) warns.push("設定ファイルはまだありません(下は仮の文の例です。保存すると有効になります)");
    if (!d.valid) warns.push("設定ファイルが不正です(直前の有効な設定を使用中): " + (d.errors || []).join(" / "));
    if (d.placeholder) warns.push("有効なパートに仮の文(「(仮)」で始まる文)が残っています。本番の文章に差し替えてから、ONにしてください。");
    $("pr_warn").innerHTML = warns.map((w) => `<div class="banner warn" style="margin-top:6px;">${escapeHtml(w)}</div>`).join("");
    const lines = [];
    const items = br && br.ok ? br.items : [];
    (PR.form ? PR.form.parts : d.config.parts).forEach((p, i) => {
      const el = $("pr_st_" + i);
      if (!el) return;
      if (PR.dirty) { el.className = "pill off"; el.textContent = "未保存"; return; }
      const it = items[i];
      if (!it) { el.className = "pill off"; el.textContent = "音声: 不明"; return; }
      const s = PR_STATE[it.state] || ["off", it.state];
      el.className = "pill " + s[0];
      el.textContent = s[1];
      el.title = it.error || "";
      if (it.state === "failed") lines.push("パート" + (i + 1) + "の音声作成に失敗: " + it.error);
    });
    if (br && br.ok) {
      const pr = br.prepare || {};
      if (pr.synthesizing || pr.queued) lines.push("2台目で音声を作成中です(待ち " + (pr.queued || 0) + " 件)。完了するまでそのパートは鳴りません。");
      if (loop) {
        lines.push("再生: " + (loop.active ? "動作中" : "停止中") + " / 一巡 " + loop.rounds + " 回 / 最後に鳴った時刻: " + fmtTime(loop.last_played_ts));
        if (loop.skipped_unprepared) lines.push("音声が未作成で飛ばした回数: " + loop.skipped_unprepared);
        if (loop.last_error) lines.push("2台目のエラー: " + loop.last_error);
      }
      lines.push("作成済みの音声は2台目に保存されています(再起動後も再利用。文章を変えたパートだけ作り直します)");
    } else if (br) {
      lines.push("2台目の状態を取得できません: " + (br.error || ""));
    }
    $("pr_stat").textContent = lines.join(" / ");
  }
  async function prLoad() {
    if (PR.busy) return;
    const r = await api("/api/pr_intro");
    if (!r.ok || !r.data || !r.data.config) { return; }
    PR.data = r.data;
    if (!PR.dirty && (PR.sig !== r.data.sig || !$("pr_enabled"))) {
      PR.sig = r.data.sig;
      PR.form = prClone(r.data.config);
      PR.form.parts = PR.form.parts.map((p) => ({ enabled: p.enabled, title: p.title, text: p.text, volume: p.volume }));
      prBuild();
      return;
    }
    prStatus();
  }
  function prFormBody() {
    const f = PR.form, L = prLimits(), errors = [];
    const chk = (v, lo, hi, name) => { if (!Number.isFinite(v) || v < lo || v > hi) errors.push(name + "は" + lo + "〜" + hi + "の範囲の数値にしてください"); };
    chk(f.interval_sec, L.interval[0], L.interval[1], "一巡後の間隔");
    chk(f.gap_sec, L.gap[0], L.gap[1], "パート間の間隔");
    chk(f.volume, 0, L.volume[1], "紹介全体の音量");
    let total = 0;
    f.parts.forEach((p, i) => {
      chk(p.volume, 0, L.volume[1], "パート" + (i + 1) + "の音量");
      const n = String(p.text || "").trim().length;
      total += n;
      if (n === 0) errors.push("パート" + (i + 1) + "の文章が空です(不要なら削除してください)");
      if (n > L.text_max) errors.push("パート" + (i + 1) + "の文章が" + L.text_max + "文字を超えています");
    });
    if (total > L.total_text_max) errors.push("全パートの合計が" + L.total_text_max + "文字を超えています");
    if (f.parts.length > L.parts_max) errors.push("パートは" + L.parts_max + "個までです");
    if (f.enabled && !f.parts.some((p) => p.enabled)) errors.push("ONにするには、有効なパートが1つ以上必要です");
    const body = {
      enabled: f.enabled, interval_sec: f.interval_sec, gap_sec: f.gap_sec, volume: f.volume, quiet_when_person_near: f.quiet_when_person_near, speech_mode: f.speech_mode,
      parts: f.parts.map((p) => ({ enabled: p.enabled, title: String(p.title || "").trim(), text: String(p.text || "").trim(), volume: p.volume })),
    };
    return { body, errors };
  }
  async function prSave() {
    if (readOnly || PR.busy) return;
    prCollect();
    const { body, errors } = prFormBody();
    if (errors.length) { $("globalMsg").innerHTML = errorsHtml("保存できません", errors); return; }
    const maxVol = Math.max(body.volume, ...body.parts.map((p) => p.volume));
    if (maxVol > CONFIRM_ABOVE && !(await askConfirm("音量が" + CONFIRM_ABOVE + "倍を超えています(最大 " + maxVol + " 倍)。大きな音が出ます。実行しますか?"))) return;
    PR.busy = true; prApply();
    try {
      const r = await postJson("/api/pr_intro", body);
      if (r.ok && r.data.ok) {
        PR.dirty = false; PR.sig = null;
        showGlobal("ok", r.data.message || "保存しました");
      } else {
        const list = r.data.errors && r.data.errors.length ? r.data.errors : [r.data.message || ("保存できませんでした(HTTP " + r.status + ")")];
        $("globalMsg").innerHTML = errorsHtml("保存できませんでした", list);
      }
    } catch (e) {
      showGlobal("err", "保存に失敗しました(通信エラー)。保存されたかどうか分からないため、ページを再読み込みして確認してください。");
    } finally {
      PR.busy = false;
    }
    await prLoad();
    prApply();
  }
  async function prAction(path, body, label) {
    if (readOnly || PR.busy) return;
    const r = await postJson(path, body);
    if (r.ok && r.data.ok) showGlobal(r.data.played === false ? "warn" : "info", r.data.message || label);
    else $("globalMsg").innerHTML = errorsHtml(label + "できませんでした", r.data.errors && r.data.errors.length ? r.data.errors : [r.data.message || ("HTTP " + r.status)]);
    await prLoad();
  }
  function prMarkDirty() { PR.dirty = true; prCounts(); prApply(); prStatus(); }
  function prInit() {
    const root = $("prIntro");
    root.addEventListener("input", (ev) => { if (!PR.form) return; prCollect(); prMarkDirty(); });
    root.addEventListener("change", (ev) => { if (!PR.form) return; prCollect(); prMarkDirty(); });
    root.addEventListener("click", async (ev) => {
      const b = ev.target.closest("button");
      if (!b || !PR.form) return;
      if (b.id === "pr_save") return prSave();
      if (b.id === "pr_prepare") return prAction("/api/pr_intro/prepare", { force: true }, "音声の作成を依頼");
      if (b.id === "pr_discard") {
        if (!confirm("未保存の変更を破棄して、保存済みの内容に戻しますか？")) return;
        PR.dirty = false; PR.sig = null; await prLoad(); return;
      }
      if (b.id === "pr_add") {
        prCollect();
        PR.form.parts.push({ enabled: true, title: "", text: "", volume: 1 });
        prBuild(); prMarkDirty(); return;
      }
      const act = b.dataset.act, i = Number(b.dataset.i);
      if (!act) return;
      prCollect();
      const parts = PR.form.parts;
      if (act === "test") { return prAction("/api/pr_intro/test", { index: i }, "試し再生"); }
      if (act === "del") {
        if (!confirm("パート" + (i + 1) + "を削除しますか？(保存するまで確定しません)")) return;
        parts.splice(i, 1);
      } else if (act === "up" && i > 0) { [parts[i - 1], parts[i]] = [parts[i], parts[i - 1]]; }
      else if (act === "down" && i < parts.length - 1) { [parts[i + 1], parts[i]] = [parts[i], parts[i + 1]]; }
      else return;
      prBuild(); prMarkDirty();
    });
  }
  prInit();
  window.addEventListener("beforeunload", (e) => { if (PR.dirty) { e.preventDefault(); e.returnValue = ""; } });

  // ---------------------------------------------------------------- 緊急停止ドック(メイン画面と同じ動作)
  let estopEngaged = false, estopPending = false;
  function renderEstop(engaged) {
    estopEngaged = engaged;
    $("estopBtn").textContent = engaged ? "緊急停止を解除" : "緊急停止(ソフト)";
    $("estopBtn").classList.toggle("engaged", engaged);
  }
  async function pollEstop() {
    if (estopPending) return;
    try { const r = await api("/api/state"); if (r.ok && typeof r.data.soft_estop_engaged === "boolean") renderEstop(r.data.soft_estop_engaged); } catch (e) { /* 次回 */ }
  }
  async function setEstop(engaged) {
    if (readOnly || estopPending) return;
    estopPending = true; $("estopBtn").disabled = true;
    const prev = estopEngaged; renderEstop(engaged);
    let ok = false;
    try { ok = (await postJson("/api/soft_estop", { engaged })).ok; } catch (e) { ok = false; }
    estopPending = false; $("estopBtn").disabled = readOnly;
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
  window.addEventListener("beforeunload", (e) => { if (isDirty()) { e.preventDefault(); e.returnValue = ""; } });

  (async function init() {
    try {
      const w = await api("/api/whoami");
      const account = (w.data && w.data.account) || "";
      $("accountLabel").textContent = account ? `ログイン: ${account}` : "";
      readOnly = !w.ok || account === "guest" || account === "";
    } catch (e) { readOnly = true; }
    if (readOnly) showGlobal("info", "閲覧専用です(guestアカウント、または認証を確認できませんでした)。内容は見られますが、編集・保存・試し再生はできません。");
    NKField.guard(["aeMaster", "aeVol", "aeVolNum"]);
    await loadAll();
    applyDisabled();
    await prLoad();
    NKField.poll(() => { if (!PR.busy) return prLoad(); }, 4000);
    pollEstop();
    NKField.poll(pollEstop, 2000);
    NKField.poll(() => { if (!busy) return loadAll(); }, 5000);
  })();
</script>
</body>
</html>
"""


def build_announce_settings_html() -> bytes:
    """状況アナウンス設定ページ(`/announce_settings`)。full_dashboard_nodeから配信される。"""
    return _ANNOUNCE_HTML.replace("__NK_FIELD_JS__", NK_FIELD_JS).encode("utf-8")
