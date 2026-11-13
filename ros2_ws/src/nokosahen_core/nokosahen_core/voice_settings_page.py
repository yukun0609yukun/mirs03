"""呼びかけ音声設定ページ(`/voice_settings`)のHTML。

【2026-10-06追加】完成版ダッシュボード(full_dashboard_node)から配信される。
バックエンドは voice_settings_controller.py / voice_settings_proxy.py / announce_settings.py。
ページ自体はROSにも2台目にも直接触れず、ダッシュボードのHTTP API(/api/voice_settings*)だけを使う。

設計メモ:
- 楽観的UIは使わない。保存はPOSTして、成功したらサーバーから読み直して表示する。
- 全ての可変テキスト(発話文・プロンプト・エラー・モデル名等)は escapeHtml() を通してから innerHTML に入れる。
- 3つのタブ(基本設定 / プロンプト / 実績)は1つの下書きを共有し、下部の「保存して反映」で一括保存する
  (変更した項目だけを送る)。
- 2台目に接続できなくても、1台目の発話間隔は編集・保存できる。
- guestアカウントは閲覧のみ(入力欄・ボタンを無効化。サーバー側もPOSTは403)。
- 緊急停止ドック(#estopDock)をメイン画面と同じく画面下端に固定表示する。
"""

from .nav_layout import NK_FIELD_JS

_VOICE_SETTINGS_HTML = r"""<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>のこさへん - 呼びかけ音声設定</title>
<style>
  :root { color-scheme: light; font-family: system-ui, -apple-system, "Segoe UI", sans-serif; background:#f4f6f5; color:#16211d;
          --c-wait:#9aa7a1; --c-llm:#2b6cb0; --c-tts:#d98324; --c-play:#2f9e6b; }
  html, body { margin:0; }
  body { padding-bottom:150px; }
  header { box-sizing:border-box; min-height:44px; padding:6px 18px; background:#124739; color:#fff; display:flex; justify-content:space-between; align-items:center; gap:8px; flex-wrap:wrap; }
  header a { color:#cfe9df; text-decoration:none; font-size:13px; }
  header h1 { margin:0; font-size:18px; }
  main { max-width:1000px; margin:0 auto; padding:14px; display:flex; flex-direction:column; gap:14px; }
  section.card { background:#fff; border:1px solid #dce4df; border-radius:8px; padding:12px 14px; }
  h2 { margin:0 0 8px; font-size:15px; }
  h3 { margin:12px 0 6px; font-size:13px; color:#33463e; }
  .hint { font-size:13px; color:#53635c; margin:6px 0 0; line-height:1.5; }
  .row { display:flex; gap:8px; flex-wrap:wrap; align-items:center; }
  button { padding:8px 12px; font-size:13px; border:0; border-radius:6px; background:#176b56; color:#fff; cursor:pointer; }
  button.secondary { background:#5d6f67; }
  button.small { padding:4px 9px; font-size:13px; }
  button:disabled { opacity:0.45; cursor:not-allowed; }
  input[type="number"], select, textarea { padding:6px; font-size:14px; border:1px solid #cfd8d3; border-radius:6px; box-sizing:border-box; max-width:100%; font-family:inherit; }
  input:disabled, select:disabled, textarea:disabled { background:#f1f3f2; color:#555; }
  textarea { width:100%; min-height:260px; line-height:1.55; resize:vertical; }
  label { font-size:13px; color:#33463e; }
  .banner { padding:8px 12px; border-radius:6px; font-size:13px; line-height:1.5; }
  .banner.err { background:#faece7; color:#993c1d; }
  .banner.ok { background:#e1f5ee; color:#0f6e56; }
  .banner.warn { background:#fff3cd; color:#7a5b00; }
  .banner.info { background:#eef3f0; color:#33463e; }
  .banner ul { margin:4px 0 0 18px; padding:0; }
  .pill { display:inline-block; padding:2px 9px; border-radius:999px; font-size:13px; font-weight:700; }
  .pill.ok { background:#e1f5ee; color:#0f6e56; }
  .pill.bad { background:#faece7; color:#993c1d; }
  .pill.wait { background:#fff3cd; color:#7a5b00; }
  .pill.off { background:#e4e8e6; color:#53635c; }
  .tabs { display:flex; gap:4px; flex-wrap:wrap; }
  .tabs button { background:#e4ebe7; color:#16211d; border-radius:8px 8px 0 0; padding:10px 16px; font-size:14px; }
  .tabs button[aria-selected="true"] { background:#fff; border:1px solid #dce4df; border-bottom-color:#fff; font-weight:700; color:#124739; }
  .tabpanel { display:none; flex-direction:column; gap:14px; }
  .tabpanel.active { display:flex; }
  .fields { display:grid; grid-template-columns:repeat(auto-fit,minmax(230px,1fr)); gap:12px; }
  .field { display:flex; flex-direction:column; gap:3px; min-width:0; }
  .field input[type="number"] { width:100%; }
  .field small { color:#53635c; font-size:13px; line-height:1.4; }
  .src { display:inline-block; padding:0 6px; border-radius:999px; background:#eef3f0; color:#53635c; font-size:13px; margin-left:4px; }
  .toggle { display:flex; gap:10px; align-items:flex-start; padding:8px 10px; border:1px solid #dce4df; border-radius:8px; background:#fbfcfb; }
  .toggle input { width:20px; height:20px; margin-top:2px; flex:none; }
  .toggle b { font-size:13px; }
  .toggle small { display:block; color:#53635c; font-size:13px; margin-top:2px; line-height:1.45; }
  .tablewrap { overflow-x:auto; }
  table { width:100%; border-collapse:collapse; font-size:13px; }
  th, td { border-bottom:1px solid #e4ebe7; padding:6px 8px; text-align:left; vertical-align:top; }
  th { background:#f1f5f3; font-size:13px; white-space:nowrap; }
  td.num, th.num { text-align:right; font-variant-numeric:tabular-nums; white-space:nowrap; }
  tr.sel td { background:#eaf4ef; }
  tr.clickable { cursor:pointer; }
  tr.clickable:hover td { background:#f4f9f6; }
  .chip { display:inline-block; white-space:nowrap; margin:1px 4px 1px 0; padding:1px 8px; border-radius:999px; background:#eef3f0; font-size:13px; }
  .chip.yes { background:#e1f5ee; color:#0f6e56; }
  .chip.no { background:#e4e8e6; color:#53635c; }
  .chip.fb { background:#fff3cd; color:#7a5b00; }
  .legend { display:flex; gap:12px; flex-wrap:wrap; font-size:13px; margin:6px 0; }
  .legend i { display:inline-block; width:12px; height:12px; border-radius:3px; margin-right:4px; vertical-align:-1px; }
  .timings-legend dt { font-weight:700; font-size:13px; margin-top:6px; }
  .timings-legend dd { margin:0 0 0 0; font-size:13px; color:#53635c; line-height:1.5; }
  pre.prompt { white-space:pre-wrap; word-break:break-all; background:#f6f8f7; border:1px solid #dce4df; border-radius:6px; padding:8px; font-size:13px; max-height:340px; overflow:auto; margin:6px 0 0; }
  .result { border:1px solid #cfe3d9; background:#f4fbf7; border-radius:8px; padding:10px 12px; }
  .result .said { font-size:15px; line-height:1.7; white-space:pre-wrap; word-break:break-all; margin:6px 0; }
  .counter { font-size:13px; color:#53635c; }
  .counter.over { color:#a32d2d; font-weight:700; }
  svg.chart { width:100%; height:auto; display:block; }
  svg.chart rect.bar { cursor:pointer; }
  #saveBar { position:fixed; left:0; right:0; bottom:68px; z-index:1900; box-sizing:border-box; padding:8px 12px; background:#fff; border-top:1px solid #cfd8d3; box-shadow:0 -2px 8px rgba(0,0,0,0.12); display:flex; gap:10px; align-items:center; flex-wrap:wrap; }
  #saveBar .msg { flex:1 1 200px; font-size:13px; color:#53635c; }
  #estopDock { position:fixed; left:0; right:0; bottom:0; z-index:2000; box-sizing:border-box; padding:8px 12px; background:rgba(18,71,57,0.96); box-shadow:0 -2px 10px rgba(0,0,0,0.3); display:flex; gap:10px; align-items:center; }
  #estopDock .stop-btn { flex:1 1 auto; margin:0; padding:14px; font-size:18px; min-height:52px; border:2px solid #fff; font-weight:700; background:#a32d2d; color:#fff; border-radius:8px; }
  #estopDock .stop-btn.engaged { background:#5c1717; }
  #estopDock a { color:#cfe9df; font-size:13px; white-space:nowrap; }
  @media (max-width:640px) { main { padding:8px; } .tabs button { padding:9px 10px; font-size:13px; } }
</style>
</head>
<body>
<header>
  <h1>呼びかけ音声設定</h1>
  <div class="row">
    <span id="accountLabel" style="font-size:13px;opacity:0.85;"></span>
    <a href="/">&larr; 操作盤に戻る</a>
  </div>
</header>
<main>
  <div id="globalMsg"></div>

  <div class="tabs" role="tablist">
    <button type="button" role="tab" id="tabBtn-basic" data-tab="basic" aria-selected="true">基本設定</button>
    <button type="button" role="tab" id="tabBtn-prompt" data-tab="prompt" aria-selected="false">プロンプト</button>
    <button type="button" role="tab" id="tabBtn-metrics" data-tab="metrics" aria-selected="false">実績(生成時間)</button>
  </div>

  <!-- ============================ 基本設定 ============================ -->
  <div class="tabpanel active" id="tab-basic">
    <section class="card" data-cap="voice.announce_interval">
      <h2>呼びかけの間隔(1台目・このロボット)</h2>
      <div class="fields">
        <div class="field">
          <label for="intervalInput">新しい呼びかけを始める最短間隔(秒)</label>
          <input id="intervalInput" type="number" inputmode="numeric" min="10" max="600" step="1" />
          <small id="intervalHint">-</small>
        </div>
      </div>
      <p class="hint">ロボットが新しい呼びかけ(発話)を始めてよい最短の間隔です。短くしすぎると、音声の生成と再生(20〜40秒かかります)が追いつかず、前の発話の続きを話し続けることになります。
        保存すると、behavior_directorが1秒以内に自動で読み込みます(再起動不要)。時間帯・状況ルールで「発話間隔の短縮」が有効な間は、そちらが優先されます。</p>
      <div id="intervalFileInfo" class="hint"></div>
    </section>

    <section class="card" data-cap="voice.announce_text">
      <div class="row" style="justify-content:space-between;">
        <h2 style="margin:0;">音声の作り方(2台目・音声サーバー)</h2>
        <span id="voiceBadge" class="pill wait">確認中...</span>
      </div>
      <div id="voiceConn"></div>
      <div id="voiceBody">
        <h3>文面に含める情報</h3>
        <div class="fields" style="grid-template-columns:repeat(auto-fit,minmax(300px,1fr));">
          <label class="toggle" for="incMenu">
            <input id="incMenu" type="checkbox" />
            <span><b>メニュー名を使う</b><small>ONにすると、今日のメニュー名をAIに渡して、おすすめの文面に入れさせます。OFFにするとメニュー名は文面の材料から外れます(「いらっしゃいませ」のような一般的な呼びかけになります)。</small></span>
          </label>
          <label class="toggle" for="incRem">
            <input id="incRem" type="checkbox" />
            <span><b>残りの数を考慮する</b><small>ONにすると、残数に余裕があるメニューをすすめる場面で、残数の情報もAIに渡します。残数カウントがまだ機能していない間(促進対象が無い間)は、ONでも渡す材料が無いため含まれません(実績の「残数」欄で確認できます)。</small></span>
          </label>
        </div>
        <h3>生成の細かい設定</h3>
        <div class="fields">
          <div class="field"><label for="fTemp">ばらつき(temperature)<span class="src" id="srcTemp"></span></label>
            <input id="fTemp" type="number" step="0.05" /><small id="hTemp"></small>
            <small>小さいほど毎回似た文面、大きいほど毎回違う文面になります。</small></div>
          <div class="field"><label for="fNum">最大の長さ(num_predict、トークン)<span class="src" id="srcNum"></span></label>
            <input id="fNum" type="number" step="1" /><small id="hNum"></small>
            <small>AIが書く文章の長さの上限(日本語は1文字が1〜2トークン程度)。短くすると生成が速くなりますが、途中で切れやすくなります(切れたときは最後の文末まで使い、文末が無ければ標準の文面になります)。</small></div>
          <div class="field"><label for="fMax">最大文字数(0で制限なし)<span class="src" id="srcMax"></span></label>
            <input id="fMax" type="number" step="1" /><small id="hMax"></small>
            <small>生成された文面がこれを超えたら、超えない最後の文末までに切り詰めます(文末が無ければ標準の文面)。</small></div>
          <div class="field"><label for="fTimeout">AIの待ち時間(秒)<span class="src" id="srcTimeout"></span></label>
            <input id="fTimeout" type="number" step="1" /><small id="hTimeout"></small>
            <small>この時間を超えても生成できなければ、標準の文面(ルールベース)を話します。</small></div>
          <div class="field"><label for="fRepeat">同じ依頼の無視時間(秒)<span class="src" id="srcRepeat"></span></label>
            <input id="fRepeat" type="number" step="1" /><small id="hRepeat"></small>
            <small>全く同じ内容の依頼がこの秒数以内にもう一度届いたら無視します(再送・ループ対策)。</small></div>
        </div>
        <p class="hint">使っているAIモデル: <b id="modelName">-</b>(モデルの変更はここからはできません)　/　設定ファイル: <code id="voiceFile">-</code></p>
      </div>
    </section>
  </div>

  <!-- ============================ プロンプト ============================ -->
  <div class="tabpanel" id="tab-prompt">
    <section class="card" data-cap="voice.announce_text">
      <h2>ロボットの人物設定(プロンプト)</h2>
      <div class="banner info">ここに書いた文章が、AIへ渡す文面の「設定」の部分になります。AIに渡す全文は、この文章に
        「指示」(その場の状況に応じた自動の文)と「メニュー一覧/残数状況」(上の基本設定で選んだ情報)が自動で付いたものです。
        保存すると、次の発話から反映されます。</div>
      <div id="promptUnavailable"></div>
      <textarea id="personaText" spellcheck="false" aria-label="人物設定のプロンプト"></textarea>
      <div class="row" style="justify-content:space-between;margin-top:6px;">
        <span class="counter" id="personaCount">0 / 4000文字</span>
        <span class="row">
          <button id="personaResetBtn" type="button" class="secondary small">標準に戻す</button>
        </span>
      </div>
      <div id="personaSource" class="hint"></div>
      <p class="hint">注意: 標準のプロンプトには「メニュー一覧から1つ選んで紹介する」という指示と「〇〇」の書き方の例が含まれています。
        「メニュー名を使う」をOFFにするときは、この文章からもメニューを選ぶ指示を外しておくと自然な文面になります。
        AIが「〇〇」のような伏せ字を出力した場合は、安全のため標準の文面に置き換わります。</p>
    </section>

    <section class="card" data-cap="voice.announce_text">
      <h2>この設定で試す(お試し生成)</h2>
      <p class="hint" style="margin-top:0;">いま画面にある内容(未保存でも)で文面を作ってみます。<b>話さず、発話履歴にも記録せず、設定も保存しません</b>。
        材料には、いま入力されているメニューの状態を使います(無いときは見本のメニュー)。AIの生成に数十秒かかることがあります。</p>
      <div class="row">
        <button id="previewBtn" type="button">現在のプロンプトで試す</button>
        <label style="display:inline-flex;gap:6px;align-items:center;"><input id="forceSample" type="checkbox" /> 見本のメニューで試す</label>
        <span id="previewStatus" class="hint" style="margin:0;"></span>
      </div>
      <div id="previewResult" style="margin-top:10px;"></div>
    </section>
  </div>

  <!-- ============================ 実績 ============================ -->
  <div class="tabpanel" id="tab-metrics">
    <section class="card">
      <div class="row" style="justify-content:space-between;">
        <h2 style="margin:0;">発話ごとの所要時間</h2>
        <div class="row">
          <label for="limitSel">集計する件数</label>
          <select id="limitSel" data-nk-keep><option value="50">直近50件</option><option value="200">直近200件</option></select>
          <button id="metricsReload" type="button" class="secondary small" data-nk-keep>更新</button>
        </div>
      </div>
      <div id="metricsMsg"></div>
      <div id="metricsStats" class="tablewrap">読み込み中...</div>
      <div id="metricsFlags" class="hint"></div>
    </section>

    <section class="card">
      <h2>最近の発話(棒の長さ=総所要時間、色=内訳)</h2>
      <div class="legend">
        <span><i style="background:var(--c-wait)"></i>待ち</span>
        <span><i style="background:var(--c-llm)"></i>AI生成</span>
        <span><i style="background:var(--c-tts)"></i>音声合成</span>
        <span><i style="background:var(--c-play)"></i>再生</span>
      </div>
      <div id="chartBox" class="tablewrap"></div>
      <p class="hint">棒または下の行をクリックすると、その発話の詳細(AIに送ったプロンプト)を表示します。左が古く、右が新しい発話です。</p>
    </section>

    <section class="card">
      <div id="rowsBox" class="tablewrap">-</div>
    </section>

    <section class="card" id="detailCard" style="display:none;">
      <h2>発話の詳細</h2>
      <div id="detailBox"></div>
    </section>

    <section class="card">
      <h2>各時間の意味</h2>
      <dl class="timings-legend">
        <dt>待ち</dt><dd>呼びかけの依頼を受け付けてから、順番が来て処理を始めるまでの時間。前の発話の処理が終わっていないと長くなります。</dd>
        <dt>AI生成</dt><dd>AI(Ollama)に文面を考えさせるのにかかった時間。AIを使わなかった発話(材料のメニューが無い等)は「-」です。</dd>
        <dt>音声合成</dt><dd>文面を音声データ(WAV)に変換するのにかかった時間(VOICEVOX)。文が長いほど長くなります。</dd>
        <dt>再生</dt><dd>スピーカーから実際に音が出ている時間。</dd>
        <dt>合計</dt><dd>依頼を受け付けてから、再生が終わるまでの全体の時間(待ち+AI生成+音声合成+再生+その他の処理)。</dd>
        <dt>メニュー名/残数</dt><dd>AIに送ったプロンプトに、実際にメニュー名・残数の情報が入っていたか。設定がONでも、材料が無ければ「なし」になります。</dd>
        <dt>標準の文面</dt><dd>AIの文面を使わず、ルールベースの標準の文面を話した発話。理由(時間切れ・エラー・空の応答・伏せ字が残った・長すぎて切れた・メニュー情報なし)を表示します。</dd>
      </dl>
    </section>
  </div>
</main>

<div id="saveBar">
  <button id="saveBtn" type="button" data-cap-any="voice.announce_interval voice.announce_text">保存して反映</button>
  <button id="discardBtn" type="button" class="secondary" data-nk-keep>変更を破棄</button>
  <span class="msg" id="saveMsg">変更はありません</span>
</div>
<div id="estopDock">
  <button id="estopBtn" class="stop-btn" type="button">緊急停止(ソフト)</button>
  <a href="/">操作盤へ</a>
</div>

<script>
__NK_FIELD_JS__
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

  const FALLBACK_LABELS = {
    timeout: "AIが時間切れ", error: "AIがエラー", empty: "AIの応答が空",
    placeholder: "「○○」の伏せ字が残った", length_cut: "長すぎて切れた", no_menu: "メニュー情報なし(生成せず)"
  };
  const SOURCE_LABELS = { file: "設定ファイル", env: "環境変数", default: "既定", builtin: "内蔵の標準", "persona_prompt.txt": "標準ファイル" };
  const VOICE_FIELDS = [
    { key: "temperature", el: "fTemp", src: "srcTemp", hint: "hTemp", unit: "" },
    { key: "num_predict", el: "fNum", src: "srcNum", hint: "hNum", unit: "" },
    { key: "max_chars", el: "fMax", src: "srcMax", hint: "hMax", unit: "" },
    { key: "ollama_timeout_sec", el: "fTimeout", src: "srcTimeout", hint: "hTimeout", unit: "秒" },
    { key: "min_repeat_sec", el: "fRepeat", src: "srcRepeat", hint: "hRepeat", unit: "秒" },
  ];

  let readOnly = true;
  let busy = false;
  let loaded = null;      // サーバーから読んだ最新の状態(/api/voice_settings)
  let personaReset = false;
  let metricsLimit = 50;
  let metricsRows = [];
  let selectedId = null;
  let lastPreview = null; // 直近のお試し生成の結果(試し発話に使う)

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
    return `<div class="banner err">${escapeHtml(title)}<ul>${items}</ul></div>`;
  }
  function fmtSec(ms) {
    if (ms === null || ms === undefined || !isFinite(ms)) return "-";
    return (ms / 1000).toFixed(1) + "秒";
  }
  function fmtTime(ts) {
    if (!ts) return "-";
    return String(ts).replace("T", " ").slice(5, 19);
  }
  function charCount(s) { return Array.from(String(s)).length; }
  function toNum(text) {
    const t = String(text).trim();
    if (t === "") return null;
    const n = Number(t);
    return Number.isFinite(n) ? n : null;
  }

  // ---------------------------------------------------------------- タブ
  function selectTab(name) {
    for (const b of document.querySelectorAll(".tabs button")) b.setAttribute("aria-selected", b.dataset.tab === name ? "true" : "false");
    for (const p of document.querySelectorAll(".tabpanel")) p.classList.toggle("active", p.id === "tab-" + name);
    if (name === "metrics") loadMetrics();
    try { history.replaceState(null, "", "#" + name); } catch (e) { /* 無くても動く */ }
  }
  for (const b of document.querySelectorAll(".tabs button")) b.addEventListener("click", () => selectTab(b.dataset.tab));

  // ---------------------------------------------------------------- 読み込みと表示
  function voiceOk() { return !!(loaded && loaded.voice && loaded.voice.ok); }

  function renderAll() {
    const a = loaded.announce || {};
    const lim = a.limits || { min: 10, max: 600 };
    $("intervalInput").min = lim.min; $("intervalInput").max = lim.max;
    $("intervalInput").value = a.announce_min_interval_seconds !== null && a.announce_min_interval_seconds !== undefined ? a.announce_min_interval_seconds : "";
    $("intervalInput").placeholder = "60";
    $("intervalHint").textContent = (a.announce_min_interval_seconds === null || a.announce_min_interval_seconds === undefined)
      ? `未設定(behavior_directorのパラメータ値が使われます。標準は60秒)。範囲 ${lim.min}〜${lim.max}秒`
      : `範囲 ${lim.min}〜${lim.max}秒`;
    let info = `設定ファイル(1台目): ${escapeHtml(a.path || "-")} / ${a.exists ? "あり" : "なし"}`;
    if (a.valid === false) info += `<div class="banner err" style="margin-top:4px;">${escapeHtml((a.errors || []).join(" / "))}(このファイルは無視され、directorは直前の値で動いています)</div>`;
    $("intervalFileInfo").innerHTML = info;

    const v = loaded.voice;
    const conn = $("voiceConn");
    if (!loaded.configured) {
      $("voiceBadge").className = "pill off"; $("voiceBadge").textContent = "未設定";
      conn.innerHTML = `<div class="banner info">2台目(音声サーバー)の中継先(voice_settings_url)が設定されていないため、音声の作り方の設定・実績は使えません。発話間隔だけ編集できます。</div>`;
    } else if (!voiceOk()) {
      $("voiceBadge").className = "pill bad"; $("voiceBadge").textContent = "接続できません";
      conn.innerHTML = `<div class="banner err">2台目に接続できません。音声サーバー(voice_bridge)が起動しているか、ネットワークを確認してください。発話間隔(1台目)は引き続き編集・保存できます。<div style="margin-top:4px;font-size:13px;">${escapeHtml(v && v.error ? v.error : "")}</div></div>`;
    } else {
      $("voiceBadge").className = "pill ok"; $("voiceBadge").textContent = "接続OK";
      conn.innerHTML = v.load_error ? `<div class="banner warn">${escapeHtml(v.load_error)}</div>` : "";
    }
    const enabled = voiceOk();
    $("voiceBody").style.opacity = enabled ? "1" : "0.5";
    if (enabled) {
      const s = v.settings, lm = v.limits || {}, src = v.sources || {}, df = v.defaults || {};
      $("incMenu").checked = !!s.include_menu_names;
      $("incRem").checked = !!s.include_remaining;
      for (const f of VOICE_FIELDS) {
        $(f.el).value = s[f.key];
        const l = lm[f.key] || {};
        $(f.el).min = l.min; $(f.el).max = l.max;
        $(f.src).textContent = SOURCE_LABELS[src[f.key]] || "";
        $(f.hint).textContent = (l.min === undefined ? "" : `範囲 ${l.min}〜${l.max}${f.unit} / `) + `標準 ${df[f.key]}${f.unit}`;
      }
      $("modelName").textContent = v.model || "-";
      $("voiceFile").textContent = v.settings_file || "-";
      $("personaText").value = s.persona_prompt;
      personaReset = false;
      $("personaSource").textContent = "現在のプロンプトの出どころ: " + (SOURCE_LABELS[src.persona_prompt] || "-");
      $("promptUnavailable").innerHTML = "";
    } else {
      $("promptUnavailable").innerHTML = `<div class="banner warn">2台目に接続できないため、プロンプトの読み込み・保存・お試しはできません。</div>`;
      $("personaText").value = "";
      $("personaSource").textContent = "";
    }
    applyDisabled();
    updatePersonaCount();
    renderDirty();
  }

  function applyDisabled() {
    const v = voiceOk();
    $("intervalInput").disabled = readOnly;
    for (const id of ["incMenu", "incRem", "fTemp", "fNum", "fMax", "fTimeout", "fRepeat", "personaText"]) $(id).disabled = readOnly || !v;
    $("personaResetBtn").disabled = readOnly || !v;
    $("previewBtn").disabled = readOnly || !v || busy;
    $("forceSample").disabled = readOnly || !v;
    $("saveBtn").disabled = readOnly || busy;
    $("discardBtn").disabled = readOnly || busy;
    $("estopBtn").disabled = readOnly;
  }

  async function loadAll(quiet) {
    try {
      const r = await api("/api/voice_settings");
      if (!r.ok || !r.data || r.data.ok === false) {
        showGlobal("err", `設定を読み込めませんでした(HTTP ${r.status})`);
        return false;
      }
      loaded = r.data;
      if (!quiet) showGlobal("", "");
      renderAll();
      return true;
    } catch (e) {
      showGlobal("err", "設定を読み込めませんでした(通信エラー)");
      return false;
    }
  }

  // ---------------------------------------------------------------- 下書き(変更の検出)
  function updatePersonaCount() {
    const n = charCount($("personaText").value.trim());
    const el = $("personaCount");
    el.textContent = `${n} / 4000文字`;
    el.classList.toggle("over", n > 4000 || n === 0);
  }

  // フォームの値から「変更した項目だけ」の送信内容を作る。入力不備はerrorsに入れる。
  function buildChanges() {
    const errors = [];
    const out = { announce: null, voice: {} };
    if (!loaded) return { errors, out, count: 0 };
    const a = loaded.announce || {};
    const iv = $("intervalInput").value.trim();
    if (iv === "") {
      if (a.announce_min_interval_seconds !== null && a.announce_min_interval_seconds !== undefined) errors.push("発話間隔を空にはできません(10〜600秒)");
    } else {
      const n = toNum(iv);
      if (n === null) errors.push("発話間隔は数値で入力してください");
      else if (n !== a.announce_min_interval_seconds) out.announce = { announce_min_interval_seconds: n };
    }
    if (voiceOk()) {
      const s = loaded.voice.settings;
      if ($("incMenu").checked !== !!s.include_menu_names) out.voice.include_menu_names = $("incMenu").checked;
      if ($("incRem").checked !== !!s.include_remaining) out.voice.include_remaining = $("incRem").checked;
      for (const f of VOICE_FIELDS) {
        const raw = $(f.el).value.trim();
        const n = toNum(raw);
        if (n === null) { errors.push(`${f.key}は数値で入力してください`); continue; }
        if (n !== s[f.key]) out.voice[f.key] = n;
      }
      const text = $("personaText").value;
      if (personaReset) { out.voice.persona_prompt = null; }
      else if (text.trim() !== String(s.persona_prompt).trim()) out.voice.persona_prompt = text;
    }
    const count = (out.announce ? 1 : 0) + Object.keys(out.voice).length;
    return { errors, out, count };
  }

  function renderDirty() {
    if (!loaded) return;
    const { errors, count } = buildChanges();
    if (errors.length) $("saveMsg").textContent = "入力を確認してください: " + errors[0];
    else $("saveMsg").textContent = count ? `未保存の変更があります(${count}項目)` : "変更はありません";
  }
  function isDirty() { const c = buildChanges(); return c.count > 0; }

  for (const id of ["intervalInput", "incMenu", "incRem", "fTemp", "fNum", "fMax", "fTimeout", "fRepeat"]) {
    $(id).addEventListener("input", renderDirty);
    $(id).addEventListener("change", renderDirty);
  }
  // 入力欄の登録(範囲はサーバーが返す上下限=min/max属性。範囲外は入力時に表示。このページの入力欄は読み込み・保存・破棄の時だけ書き換わる)
  NKField.guard(["intervalInput", "incMenu", "incRem", "fTemp", "fNum", "fMax", "fTimeout", "fRepeat", "personaText", "forceSample", "limitSel"]);
  $("personaText").addEventListener("input", () => { personaReset = false; updatePersonaCount(); renderDirty(); });
  $("personaResetBtn").addEventListener("click", () => {
    if (readOnly || !voiceOk()) return;
    if (!confirm("プロンプトを標準の文章に戻しますか？(「保存して反映」を押すまでは保存されません)")) return;
    $("personaText").value = loaded.voice.defaults.persona_prompt;
    personaReset = true;
    updatePersonaCount();
    renderDirty();
  });

  // ---------------------------------------------------------------- 保存(楽観更新なし。保存後に読み直す)
  // 送った変更(body)がサーバーの実際の値(d)に反映されているか。違う項目の説明を返す(空配列=すべて反映済み)
  function mismatchOf(body, d) {
    const out = [];
    if (body.announce) {
      const got = (d.announce || {}).announce_min_interval_seconds;
      if (Number(got) !== body.announce.announce_min_interval_seconds) out.push(`発話間隔 送信 ${body.announce.announce_min_interval_seconds} / サーバー ${got}`);
    }
    if (body.voice) {
      const s = d.voice && d.voice.settings;
      if (!s) out.push("音声の設定を2台目から読めません");
      else {
        for (const [k, v] of Object.entries(body.voice)) {
          if (k === "persona_prompt") { if (v !== null && String(s[k]).trim() !== String(v).trim()) out.push("プロンプト"); }
          else if (typeof v === "boolean") { if (!!s[k] !== v) out.push(`${k} 送信 ${v} / サーバー ${s[k]}`); }
          else if (!(Math.abs(Number(s[k]) - Number(v)) < 1e-9)) out.push(`${k} 送信 ${v} / サーバー ${s[k]}`);
        }
      }
    }
    return out;
  }
  async function save() {
    if (readOnly || busy) return;
    const { errors, out, count } = buildChanges();
    if (errors.length) { showGlobal("err", errors.join(" / ")); return; }
    if (!count) { showGlobal("info", "変更はありません"); return; }
    const body = {};
    if (out.announce) body.announce = out.announce;
    if (Object.keys(out.voice).length) body.voice = out.voice;
    busy = true; applyDisabled();
    $("saveMsg").textContent = "保存中...";
    for (const x of ["tab-basic", "tab-prompt"]) $(x).inert = true;  // 保存中は編集不可(読み直しで編集内容が消えるのを防ぐ)
    try {
      const r = await postJson("/api/voice_settings/save", body);
      if (r.ok && r.data.ok) {
        // 保存後の実際の値を読み直し、送った値と一致したことを確かめてから画面を更新する(不一致なら入力欄はそのままにして警告)
        let bad = [];
        for (const w of [0, 700, 1500]) {
          if (w) await new Promise((res) => setTimeout(res, w));
          const fr = await api("/api/voice_settings");
          bad = (fr.ok && fr.data && fr.data.ok !== false) ? mismatchOf(body, fr.data) : ["サーバーの値を読み直せません"];
          if (!bad.length) break;
        }
        if (bad.length) {
          showGlobal("warn", "保存を送信しましたが、反映されていません: " + bad.join("、") + "。入力欄は変更していません(もう一度「保存して反映」を押すか、ページを再読み込みして確認してください)");
        } else {
          const reloaded = await loadAll(true);
          showGlobal(reloaded ? "ok" : "warn", (r.data.message || "保存しました") + (reloaded ? "" : "(読み直しに失敗しました。ページを再読み込みして確認してください)"));
        }
      } else {
        const list = r.data.errors && r.data.errors.length ? r.data.errors : [r.data.message || r.data.error || `保存できませんでした(HTTP ${r.status})`];
        $("globalMsg").innerHTML = errorsHtml("保存できませんでした", list) + (r.data.message ? `<div class="hint">${escapeHtml(r.data.message)}</div>` : "");
        await loadAll(true); // 一部だけ保存されている場合も、実際の状態を表示し直す
        $("globalMsg").innerHTML = errorsHtml("保存できませんでした(画面は保存済みの実際の値に戻しました)", list) + (r.data.message ? `<div class="hint">${escapeHtml(r.data.message)}</div>` : "");
      }
    } catch (e) {
      showGlobal("err", "保存に失敗しました(通信エラー)。保存されたかどうか分からないため、ページを再読み込みして確認してください。");
    } finally {
      busy = false; applyDisabled(); renderDirty();
      for (const x of ["tab-basic", "tab-prompt"]) $(x).inert = false;
    }
  }
  $("saveBtn").addEventListener("click", save);
  $("discardBtn").addEventListener("click", async () => {
    if (isDirty() && !confirm("未保存の変更を破棄して、保存済みの内容に戻しますか？")) return;
    await loadAll();
  });

  // ---------------------------------------------------------------- お試し生成(ドライラン)と試し発話
  function currentVoiceDraft() {
    // お試しには、画面の値すべて(未保存分を含む)を送る。入力不備は送らずに知らせる。
    const out = { include_menu_names: $("incMenu").checked, include_remaining: $("incRem").checked };
    for (const f of VOICE_FIELDS) {
      const n = toNum($(f.el).value);
      if (n === null) return { error: `${f.key}は数値で入力してください` };
      out[f.key] = n;
    }
    out.persona_prompt = personaReset ? null : $("personaText").value;
    return { settings: out };
  }

  function previewHtml(d) {
    const flags = `<span class="chip ${d.used_menu_names ? "yes" : "no"}">メニュー名 ${d.used_menu_names ? "あり" : "なし"}</span>` +
      `<span class="chip ${d.used_remaining ? "yes" : "no"}">残数 ${d.used_remaining ? "あり" : "なし"}</span>` +
      (d.trimmed ? `<span class="chip">文末で切り詰め</span>` : "");
    const fb = d.fallback_reason
      ? `<div class="banner warn" style="margin:6px 0;">AIの文面は使われず、標準の文面になりました: ${escapeHtml(FALLBACK_LABELS[d.fallback_reason] || d.fallback_reason)}${d.error ? "(" + escapeHtml(d.error) + ")" : ""}</div>`
      : "";
    const src = d.context_source === "sample" ? "見本のメニュー" : "いまのメニュー状態";
    const speakBtn = `<button id="speakTryBtn" type="button" class="secondary small" data-cap="voice.test_speak" ${readOnly ? "disabled" : ""}>この文面を試しに話す</button>`;
    return `<div class="result">
      <div class="row"><b>生成された文面</b><span class="hint" style="margin:0;">AI生成 ${escapeHtml(fmtSec(d.llm_ms))} / ${escapeHtml(d.chars)}文字 / モデル ${escapeHtml(d.model || "-")}</span></div>
      <div class="said">${escapeHtml(d.text)}</div>
      ${fb}
      <div>${flags}</div>
      <div class="hint">材料: ${escapeHtml(src)}(${escapeHtml((d.context_menu || []).join("、"))} / フェーズ: ${escapeHtml(d.context_phase)})</div>
      <div class="row" style="margin-top:8px;">${speakBtn}<span id="speakStatus" class="hint" style="margin:0;"></span></div>
      <details style="margin-top:8px;"><summary style="cursor:pointer;font-size:13px;">AIに送ったプロンプト全文を見る</summary><pre class="prompt">${escapeHtml(d.prompt || "(AIを呼ばなかったためありません)")}</pre></details>
    </div>`;
  }

  async function runPreview() {
    if (readOnly || busy || !voiceOk()) return;
    const draft = currentVoiceDraft();
    if (draft.error) { $("previewResult").innerHTML = errorsHtml("試せません", [draft.error]); return; }
    busy = true; applyDisabled();
    $("previewStatus").textContent = "生成中...(数十秒かかることがあります)";
    $("previewResult").innerHTML = "";
    lastPreview = null;
    try {
      const r = await postJson("/api/voice_settings/preview", { settings: draft.settings, force_sample: $("forceSample").checked });
      if (r.ok && r.data.ok) {
        lastPreview = r.data;
        $("previewResult").innerHTML = previewHtml(r.data);
        const sb = $("speakTryBtn");
        if (sb) sb.addEventListener("click", speakTry);
        $("previewStatus").textContent = "";
      } else {
        const list = r.data.errors && r.data.errors.length ? r.data.errors : [r.data.error || `失敗しました(HTTP ${r.status})`];
        $("previewResult").innerHTML = errorsHtml("お試し生成できませんでした", list);
        $("previewStatus").textContent = "";
      }
    } catch (e) {
      $("previewResult").innerHTML = errorsHtml("お試し生成できませんでした", ["通信エラー(サーバーに接続できません)"]);
      $("previewStatus").textContent = "";
    } finally {
      busy = false; applyDisabled();
    }
  }
  $("previewBtn").addEventListener("click", runPreview);

  async function speakTry() {
    if (readOnly || !lastPreview || busy) return;
    const text = lastPreview.text;
    if (!confirm("実際にロボットのスピーカーから、次の文面を話します。よろしいですか？\n\n" + text)) return;
    const st = $("speakStatus");
    st.textContent = "送信中...";
    try {
      const r = await postJson("/api/voice_settings/speak", { text });
      st.textContent = r.ok && r.data.ok ? (r.data.message || "話す依頼を送りました") : ((r.data.errors && r.data.errors[0]) || r.data.error || `失敗しました(HTTP ${r.status})`);
    } catch (e) {
      st.textContent = "送信に失敗しました(通信エラー)";
    }
  }

  // ---------------------------------------------------------------- 実績
  function statRow(label, t) {
    return `<tr><th scope="row">${escapeHtml(label)}</th><td class="num">${escapeHtml(t.count)}</td><td class="num">${escapeHtml(fmtSec(t.mean))}</td><td class="num">${escapeHtml(fmtSec(t.median))}</td><td class="num">${escapeHtml(fmtSec(t.p95))}</td><td class="num">${escapeHtml(fmtSec(t.max))}</td></tr>`;
  }
  function renderStats(stats) {
    const tm = stats.timings;
    $("metricsStats").innerHTML = `<table><thead><tr><th>項目</th><th class="num">件数</th><th class="num">平均</th><th class="num">中央値</th><th class="num">95%点</th><th class="num">最大</th></tr></thead><tbody>` +
      statRow("待ち", tm.queue_wait_ms) + statRow("AI生成", tm.llm_ms) + statRow("音声合成", tm.tts_ms) + statRow("再生", tm.play_ms) + statRow("合計", tm.total_ms) + `</tbody></table>`;
    const reasons = Object.entries(stats.fallback_reasons || {}).map(([k, v]) => `${escapeHtml(FALLBACK_LABELS[k] || k)} ${escapeHtml(v)}件`).join("、");
    $("metricsFlags").innerHTML = `対象 ${escapeHtml(stats.rows)}件(うち計測あり ${escapeHtml(stats.measured)}件。古い発話は計測前のため含まれません) / ` +
      `メニュー名ありで生成 ${escapeHtml(stats.used_menu_names_count)}件 / 残数ありで生成 ${escapeHtml(stats.used_remaining_count)}件 / ` +
      `標準の文面 ${escapeHtml(stats.fallback_count)}件${reasons ? "(" + reasons + ")" : ""} / 再生失敗 ${escapeHtml(stats.speak_failed_count)}件。` +
      ` 95%点は「20回に19回はこの時間以内」の目安です。`;
  }

  function renderChart(rows) {
    const data = rows.filter((r) => r.total_ms !== null && r.total_ms !== undefined).slice(-50);
    if (!data.length) { $("chartBox").innerHTML = `<div class="hint">計測のある発話がまだありません。</div>`; return; }
    const parts = (r) => [
      ["待ち", r.queue_wait_ms || 0, "var(--c-wait)"], ["AI生成", r.llm_ms || 0, "var(--c-llm)"],
      ["音声合成", r.tts_ms || 0, "var(--c-tts)"], ["再生", r.play_ms || 0, "var(--c-play)"],
    ];
    const maxMs = Math.max(1000, ...data.map((r) => Math.max(r.total_ms, parts(r).reduce((a, p) => a + p[1], 0))));
    const bw = 14, gap = 3, left = 40, top = 8, h = 160, bottom = 18;
    const W = left + data.length * (bw + gap) + 8, H = top + h + bottom;
    let svg = `<svg class="chart" viewBox="0 0 ${W} ${H}" style="min-width:${Math.min(W, 640)}px" role="img" aria-label="発話ごとの所要時間の棒グラフ">`;
    for (let i = 0; i <= 4; i++) {
      const y = top + h - (h * i) / 4;
      svg += `<line x1="${left}" x2="${W}" y1="${y}" y2="${y}" stroke="#e4ebe7" stroke-width="1"/><text x="${left - 4}" y="${y + 3}" font-size="9" text-anchor="end" fill="#53635c">${Math.round(maxMs * i / 4 / 1000)}秒</text>`;
    }
    data.forEach((r, i) => {
      const x = left + i * (bw + gap);
      let y = top + h;
      const title = `${fmtTime(r.timestamp)} 合計${fmtSec(r.total_ms)}`;
      for (const [label, ms, color] of parts(r)) {
        const hh = (h * ms) / maxMs;
        if (hh <= 0) continue;
        y -= hh;
        svg += `<rect class="bar" data-id="${escapeHtml(r.id)}" x="${x}" y="${y}" width="${bw}" height="${hh}" fill="${color}"><title>${escapeHtml(title + " / " + label + " " + fmtSec(ms))}</title></rect>`;
      }
      if (r.id === selectedId) svg += `<rect x="${x - 1}" y="${top}" width="${bw + 2}" height="${h}" fill="none" stroke="#124739" stroke-width="1.5"/>`;
    });
    svg += `</svg>`;
    $("chartBox").innerHTML = svg;
  }

  function flagChip(v, onLabel) {
    if (v === null || v === undefined) return `<span class="chip no">-</span>`;
    return `<span class="chip ${v ? "yes" : "no"}">${v ? onLabel : "なし"}</span>`;
  }
  function renderRows(rows) {
    const list = rows.slice(-50).reverse();
    if (!list.length) { $("rowsBox").innerHTML = `<div class="hint">発話の記録がまだありません。</div>`; return; }
    const body = list.map((r) => {
      const fb = r.fallback_reason ? `<span class="chip fb">${escapeHtml(FALLBACK_LABELS[r.fallback_reason] || r.fallback_reason)}</span>` : "";
      const text = String(r.text || "");
      return `<tr class="clickable ${r.id === selectedId ? "sel" : ""}" data-id="${escapeHtml(r.id)}">
        <td style="white-space:nowrap;">${escapeHtml(fmtTime(r.timestamp))}</td>
        <td style="min-width:220px;">${escapeHtml(text.length > 60 ? text.slice(0, 60) + "…" : text)}</td>
        <td class="num">${escapeHtml(fmtSec(r.queue_wait_ms))}</td><td class="num">${escapeHtml(fmtSec(r.llm_ms))}</td>
        <td class="num">${escapeHtml(fmtSec(r.tts_ms))}</td><td class="num">${escapeHtml(fmtSec(r.play_ms))}</td>
        <td class="num"><b>${escapeHtml(fmtSec(r.total_ms))}</b></td>
        <td>${flagChip(r.used_menu_names, "あり")}</td><td>${flagChip(r.used_remaining, "あり")}</td>
        <td>${fb}</td><td>${escapeHtml(r.model || "-")}</td></tr>`;
    }).join("");
    $("rowsBox").innerHTML = `<table><thead><tr><th>時刻</th><th>文面</th><th class="num">待ち</th><th class="num">AI生成</th><th class="num">音声合成</th><th class="num">再生</th><th class="num">合計</th><th>メニュー名</th><th>残数</th><th>標準の文面</th><th>モデル</th></tr></thead><tbody>${body}</tbody></table>`;
  }

  async function loadMetrics() {
    if (!loaded || !loaded.configured) { $("metricsMsg").innerHTML = `<div class="banner info">2台目の中継先が未設定のため、実績は表示できません。</div>`; return; }
    $("metricsMsg").innerHTML = "";
    try {
      const r = await api(`/api/voice_settings/metrics?limit=${metricsLimit}`);
      if (!r.ok || !r.data.ok) {
        $("metricsMsg").innerHTML = `<div class="banner err">${escapeHtml(r.data.error && r.data.error.indexOf("2台目") === 0 ? r.data.error : "実績を取得できませんでした: " + (r.data.error || "HTTP " + r.status))}</div>`;
        return;
      }
      metricsRows = r.data.rows || [];
      renderStats(r.data.stats);
      renderChart(metricsRows);
      renderRows(metricsRows);
    } catch (e) {
      $("metricsMsg").innerHTML = `<div class="banner err">実績を取得できませんでした(通信エラー)</div>`;
    }
  }
  $("limitSel").addEventListener("change", () => { metricsLimit = Number($("limitSel").value) || 50; loadMetrics(); });
  $("metricsReload").addEventListener("click", loadMetrics);

  async function showDetail(id) {
    selectedId = id;
    renderChart(metricsRows); renderRows(metricsRows);
    $("detailCard").style.display = "";
    $("detailBox").textContent = "読み込み中...";
    try {
      const r = await api(`/api/voice_settings/metrics?id=${encodeURIComponent(id)}`);
      if (!r.ok || !r.data.ok) { $("detailBox").innerHTML = errorsHtml("詳細を取得できませんでした", [r.data.error || `HTTP ${r.status}`]); return; }
      const d = r.data.row;
      const fb = d.fallback_reason ? `<span class="chip fb">${escapeHtml(FALLBACK_LABELS[d.fallback_reason] || d.fallback_reason)}</span>` : `<span class="chip no">AIの文面を使用</span>`;
      $("detailBox").innerHTML = `
        <div class="said" style="font-size:15px;line-height:1.7;white-space:pre-wrap;word-break:break-all;">${escapeHtml(d.text)}</div>
        <div style="margin:6px 0;">${fb}${flagChip(d.used_menu_names, "メニュー名あり")}${flagChip(d.used_remaining, "残数あり")}${d.trimmed ? '<span class="chip">文末で切り詰め</span>' : ""}${d.speak_ok === false ? '<span class="chip fb">再生失敗</span>' : ""}</div>
        <div class="hint">時刻 ${escapeHtml(d.timestamp)} / 種類 ${escapeHtml(d.kind || "-")} / フェーズ ${escapeHtml(d.phase || "-")} / モデル ${escapeHtml(d.model || "-")} / temperature ${escapeHtml(d.temperature === null ? "-" : d.temperature)} / num_predict ${escapeHtml(d.num_predict === null ? "-" : d.num_predict)} / 依頼ID ${escapeHtml(d.request_id || "-")}</div>
        <div class="hint">待ち ${escapeHtml(fmtSec(d.queue_wait_ms))} + AI生成 ${escapeHtml(fmtSec(d.llm_ms))} + 音声合成 ${escapeHtml(fmtSec(d.tts_ms))} + 再生 ${escapeHtml(fmtSec(d.play_ms))} → 合計 ${escapeHtml(fmtSec(d.total_ms))}</div>
        ${d.error ? `<div class="banner warn" style="margin-top:6px;">${escapeHtml(d.error)}</div>` : ""}
        <h3>AIに送ったプロンプト(最大4000文字まで保存)</h3>
        <pre class="prompt">${escapeHtml(d.prompt || "(記録なし: AIを呼ばなかった発話、または計測前の発話です)")}</pre>`;
    } catch (e) {
      $("detailBox").innerHTML = errorsHtml("詳細を取得できませんでした", ["通信エラー"]);
    }
  }
  $("chartBox").addEventListener("click", (ev) => { const t = ev.target.closest("rect.bar"); if (t) showDetail(Number(t.dataset.id)); });
  $("rowsBox").addEventListener("click", (ev) => { const tr = ev.target.closest("tr.clickable"); if (tr) showDetail(Number(tr.dataset.id)); });

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
      const r = await postJson("/api/soft_estop", { engaged });
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
  window.addEventListener("beforeunload", (e) => { if (loaded && isDirty()) { e.preventDefault(); e.returnValue = ""; } });
  document.addEventListener("visibilitychange", () => { if (!document.hidden) pollEstop(); });

  (async function init() {
    try {
      const w = await api("/api/whoami");
      const account = (w.data && w.data.account) || "";
      $("accountLabel").textContent = account ? `ログイン: ${account}` : "";
      readOnly = !w.ok || account === "guest" || account === "";
    } catch (e) {
      readOnly = true;
    }
    if (readOnly) showGlobal("info", "閲覧専用です(guestアカウント、または認証を確認できませんでした)。内容は見られますが、編集・保存・お試しはできません。");
    await loadAll(readOnly);
    const hash = (location.hash || "").replace("#", "");
    if (["basic", "prompt", "metrics"].includes(hash)) selectTab(hash);
    pollEstop();
    setInterval(pollEstop, 2000);
    NKField.poll(() => { if ($("tab-metrics").classList.contains("active")) return loadMetrics(); }, 15000);
  })();
</script>
</body>
</html>
"""


def build_voice_settings_html() -> bytes:
    """呼びかけ音声設定ページ(`/voice_settings`)。full_dashboard_nodeから配信される。"""
    return _VOICE_SETTINGS_HTML.replace("__NK_FIELD_JS__", NK_FIELD_JS).encode("utf-8")
