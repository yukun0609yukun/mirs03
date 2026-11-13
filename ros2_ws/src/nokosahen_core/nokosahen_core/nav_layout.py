"""完成版ダッシュボード(PC版)の共通ナビ・レイアウト部品(CSS/HTML/JS文字列)。

【2026-10-10】配置を再設計(いまの状況・グループ見出し・カード状態バッジ)。
【2026-10-06】管理画面を「タブ(ダッシュボード/地図・走行/発話・音声/ログ・分析/設定)+
メニュードロワー」に再構成した。リンク集(LINK_GROUPS)をここに1か所だけ持ち、
タブ内のカード表示とドロワーの両方を同じデータから生成する(リンクの追加はここだけ)。
純Python(rclpy非依存)。HTML/JSはfull_dashboard_node.pyの文字列へ__NAV_*__で埋め込む。
"""
from __future__ import annotations

from html import escape
from typing import Dict, List, Sequence, Tuple

# (グループid, 見出し, [(href, ラベル, 説明, 補足属性)])
# 補足属性: "" または "ocr"(href末尾をこの画面のホスト名:8091へJSで差し替える)
LinkItem = Tuple[str, str, str, str]
LINK_GROUPS: List[Tuple[str, str, List[LinkItem]]] = [
    ("map", "地図・走行", [
        ("/map_editor", "地図エディタ", "地図を大きく開いて走行範囲・立入禁止を編集", ""),
    ]),
    ("speech", "発話・音声", [
        ("/utterance_log", "発話履歴", "ロボットが話した内容の記録", ""),
        ("/voice_settings", "呼びかけ音声設定", "発話の間隔・AIへの指示文・応答時間の記録など", ""),
        ("/announce_settings", "状況アナウンス", "「探索中です」などの声のお知らせ(内容・ON/OFF・音量・間隔)", ""),
    ]),
    ("logs", "ログ・分析", [
        ("/detection_log", "人物検知ログ", "人物を検知した記録", ""),
        ("/step_response", "ステップ応答", "ステップ応答テストの結果", ""),
        ("/consumption_graph", "消費推移グラフ", "メニューの消費の推移", ""),
        ("/recommendation_effectiveness", "推奨効果分析", "推奨発話の効果の分析", ""),
        ("/activity_report", "日報/週報", "活動のレポート", ""),
    ]),
    ("display", "表示・デモ", [
        ("/audience_display", "案内表示(観客向け)", "観客向けの案内画面を開く", ""),
    ]),
    ("rules", "動作ルール", [
        ("/schedule", "かんたん時間割", "何時にどこへ行くかを表で指定して実行(まずはこれ)", ""),
        ("/behavior_rules", "時間帯・状況ルール", "時間帯や状況ごとの動きを編集", ""),
        ("/behavior_tree", "ビヘイビアツリー", "いつ・どんな動作をするかを木で組む", ""),
    ]),
    ("ocr", "OCR", [
        ("#", "タブレットOCR(映像・読み取り)", "読み取り結果とカメラ映像の確認(設定タブの中。確認画面8091は1台目の内側だけ)", "ocr"),
    ]),
]

# タブ(id, ラベル)
TABS: List[Tuple[str, str]] = [
    ("dash", "ダッシュボード"),
    ("map", "地図・走行"),
    ("voice", "発話・音声"),
    ("logs", "ログ・分析"),
    ("settings", "設定"),
]


def _group_map() -> Dict[str, Tuple[str, List[LinkItem]]]:
    return {gid: (title, items) for gid, title, items in LINK_GROUPS}


def _link_html(item: LinkItem) -> str:
    href, label, desc, flag = item
    extra = ' data-ocr="1"' if flag == "ocr" else ""
    return (
        '<a class="linkcard" href="' + escape(href) + '" target="_blank" rel="noopener"' + extra + '>'
        '<b>' + escape(label) + '</b><span>' + escape(desc) + '</span></a>'
    )


def link_cards_html(group_ids: Sequence[str] = (), show_title: bool = True) -> str:
    """指定グループ(空なら全グループ)を「見出し+リンクカードのグリッド」で返す。
    show_title=Falseは、親のグループ見出しが既にある場所(各タブのカード内)用で小見出しを省く。"""
    gm = _group_map()
    ids = list(group_ids) or [g[0] for g in LINK_GROUPS]
    out = []
    for gid in ids:
        title, items = gm[gid]
        out.append(
            '<div class="linkgroup">' + ('<h3>' + escape(title) + '</h3>' if show_title else '') + '<div class="linkgrid">'
            + "".join(_link_html(i) for i in items) + '</div></div>'
        )
    return "".join(out)


def nav_header_html() -> str:
    tabs = "".join(
        '<a href="#tab-' + tid + '" role="tab" data-tab="' + tid + '">' + escape(label) + '</a>'
        for tid, label in TABS
    )
    return (
        '<header id="topbar">'
        '<div class="brand"><h1>のこさへん</h1><span class="brand-sub">管理画面</span></div>'
        '<nav id="tabs" role="tablist" aria-label="画面の切り替え">' + tabs + '</nav>'
        '<div class="header-right">'
        '<span id="softEstopChip" class="chip chip-bad" hidden>ソフト停止中</span>'
        '<span id="voiceOffChip" class="chip chip-warn" hidden title="音声の全体スイッチがOFF(発話・BGMとも停止中)">音声OFF</span>'
        '<span id="estopBadge" class="estop-badge estop-ok">--</span>'
        '<span id="accountLabel"></span>'
        '<a id="menuToggle" class="hbtn" href="#" role="button" aria-expanded="false" aria-controls="menuDrawer">&#9776; メニュー</a>'
        '<button id="logoutBtn" class="logout-btn">ログアウト</button>'
        '</div></header>'
    )


def nav_drawer_html() -> str:
    return (
        '<div id="menuScrim" hidden></div>'
        '<aside id="menuDrawer" hidden aria-label="管理メニュー">'
        '<div class="drawer-head"><b>管理メニュー</b>'
        '<a id="menuClose" class="hbtn dark" href="#" role="button">閉じる</a></div>'
        + link_cards_html() +
        '</aside>'
    )


NAV_CSS = r"""
  :root {
    color-scheme: light;
    font-family: system-ui, -apple-system, "Segoe UI", "Hiragino Sans", "Yu Gothic UI", sans-serif;
    --bg:#f2f5f3; --card:#fff; --line:#d9e2dd; --ink:#16211d; --mute:#566860;
    --brand:#124739; --brand2:#176b56; --brand-soft:#eaf3ef;
    --ok-bg:#e1f5ee; --ok:#0f6e56; --warn-bg:#fff3cd; --warn:#7a5b00; --bad-bg:#faece7; --bad:#993c1d;
    --stop:#a32d2d;
    --s1:4px; --s2:8px; --s3:12px; --s4:16px; --s5:24px; --r:10px;
    --hdr:56px; --dock:68px;
    background:var(--bg); color:var(--ink);
  }
  * { box-sizing:border-box; }
  html, body { margin:0; }
  body { background:var(--bg); font-size:14px; line-height:1.45; padding-bottom:var(--dock); overflow-x:hidden; }
  h1 { margin:0; font-size:18px; }
  h2 { margin:0 0 var(--s2); font-size:15px; flex:0 0 auto; }
  h3 { margin:0 0 var(--s2); font-size:13px; color:var(--mute); font-weight:700; }
  p { margin:0; }
  button, select, input { font:inherit; }
  button { font-size:13px; padding:6px 12px; min-height:34px; border:1px solid #bccbc3; border-radius:8px; background:#fff; color:var(--ink); cursor:pointer; }
  button:hover:not(:disabled) { background:var(--brand-soft); }
  button:disabled { opacity:.5; cursor:not-allowed; }
  input[type=number], input[type=text], select { padding:5px 8px; min-height:32px; border:1px solid #bccbc3; border-radius:6px; font-size:13px; background:#fff; color:var(--ink); }
  label { font-size:13px; color:var(--mute); }

  /* ---- 固定ヘッダー(タイトル+タブ+接続/停止状態) ---- */
  #topbar { position:sticky; top:0; z-index:1000; min-height:var(--hdr); padding:var(--s2) var(--s4); background:var(--brand); color:#fff;
    display:flex; align-items:center; gap:var(--s4); flex-wrap:wrap; }
  .brand { display:flex; align-items:baseline; gap:var(--s2); flex:0 0 auto; }
  .brand-sub { font-size:13px; opacity:.8; }
  #tabs { display:flex; gap:var(--s1); flex:1 1 auto; min-width:0; overflow-x:auto; scrollbar-width:none; }
  #tabs::-webkit-scrollbar { display:none; }
  #tabs a { color:#d6e7e0; text-decoration:none; padding:8px 14px; border-radius:8px; font-size:14px; font-weight:600; white-space:nowrap; }
  #tabs a:hover { background:rgba(255,255,255,.12); }
  #tabs a.active { background:#fff; color:var(--brand); }
  .header-right { display:flex; align-items:center; gap:var(--s2); flex:0 0 auto; }
  #accountLabel { font-size:13px; opacity:.85; }
  .hbtn { color:#fff; text-decoration:none; font-size:13px; padding:6px 12px; border:1px solid rgba(255,255,255,.5); border-radius:8px; white-space:nowrap; }
  .hbtn:hover { background:rgba(255,255,255,.15); }
  .hbtn.dark { color:var(--brand); border-color:#bccbc3; }
  .logout-btn { padding:6px 10px; min-height:0; font-size:13px; border:1px solid rgba(255,255,255,.5); border-radius:8px; background:transparent; color:#fff; }
  .logout-btn:hover:not(:disabled) { background:rgba(255,255,255,.15); }
  .estop-badge, .chip { display:inline-block; padding:3px 12px; border-radius:999px; font-weight:700; font-size:13px; }
  .estop-ok, .chip-ok { background:var(--ok-bg); color:var(--ok); }
  .estop-bad, .chip-bad { background:var(--bad-bg); color:var(--bad); }
  .estop-policy, .chip-warn { background:var(--warn-bg); color:var(--warn); }
  [hidden] { display:none !important; }

  /* ---- 緊急停止ドック: 常に画面下端に固定、全タブ・全幅で最前面 ---- */
  #estopDock { position:fixed; left:0; right:0; bottom:0; z-index:2000; padding:var(--s2) var(--s3); background:rgba(18,71,57,.96); box-shadow:0 -2px 10px rgba(0,0,0,.3); }
  #estopDock .stop-btn { margin:0; padding:14px; font-size:18px; min-height:52px; border:2px solid #fff; }
  .stop-btn { width:100%; padding:10px; font-size:14px; font-weight:700; border:0; border-radius:var(--r); background:var(--stop); color:#fff; flex:0 0 auto; }
  .stop-btn:hover:not(:disabled) { background:#8a2424; }
  .stop-btn.engaged { background:#5c1717; }

  /* ---- カード・ヒント・状態チップ ---- */
  .tabpane { display:none; padding:var(--s3) var(--s4); }
  .tabpane.active { display:block; }
  .card, section.card { background:var(--card); border:1px solid var(--line); border-radius:var(--r); padding:var(--s3) var(--s4); min-height:0; }
  .hint { font-size:13px; color:var(--mute); margin-top:var(--s2); }
  .row { display:flex; gap:var(--s2); flex-wrap:wrap; }
  .metrics { display:grid; grid-template-columns:repeat(auto-fill,minmax(120px,1fr)); gap:var(--s2); }
  .metric { background:#eef3f0; border-radius:8px; padding:var(--s2) var(--s3); border-left:4px solid transparent; min-width:0; }
  .metric span { display:block; font-size:13px; color:var(--mute); }
  .metric b { font-size:16px; word-break:break-all; }
  .metric[data-state=ok] { border-left-color:#2e9d7d; }
  .metric[data-state=warn] { border-left-color:#d9a400; background:var(--warn-bg); }
  .metric[data-state=bad] { border-left-color:var(--stop); background:var(--bad-bg); }
  #announce { font-size:14px; background:#fff6df; border-radius:8px; padding:var(--s2) var(--s3); min-height:2.4em; }
  table { width:100%; border-collapse:collapse; }
  th, td { border-bottom:1px solid #e4ebe7; padding:6px 8px; text-align:left; font-size:13px; }
  th { color:var(--mute); font-size:13px; }
  .promote { color:var(--bad); font-weight:700; }
  .tblwrap { overflow:auto; max-height:60vh; margin-top:var(--s2); }
  .btns { display:flex; gap:var(--s2); flex-wrap:wrap; margin-top:var(--s2); }
  .btns > * { flex:1 1 auto; }
  .btns input[type=text], .btns select { flex:2 1 160px; min-width:0; }
  .primary { background:var(--brand2); border-color:var(--brand2); color:#fff; }
  .primary:hover:not(:disabled) { background:#0f5a47; }
  .warnbtn { background:#fff3cd; border-color:#d9a400; color:var(--warn); }
  .resume-btn { width:100%; padding:10px; font-size:14px; font-weight:700; border:0; border-radius:var(--r); background:#4e5e57; color:#fff; }
  .resume-btn:hover:not(:disabled) { background:#3d4a44; }
  .we-btn { margin-top:var(--s2); }
  .we-btn.we-urgent { background:#b3261e; box-shadow:0 0 0 3px rgba(179,38,30,.35); font-size:16px; }
  .we-btn.we-urgent:hover:not(:disabled) { background:#8c1d18; }
  .map-note.wall-stuck-note { background:rgba(179,38,30,.95); font-weight:700; cursor:default; }
  .map-note.wall-stuck-note::after { content:none; }

  /* ---- 管理メニューのリンクカード ---- */
  .linkgroup + .linkgroup { margin-top:var(--s3); }
  .linkgrid { display:grid; grid-template-columns:repeat(auto-fill,minmax(190px,1fr)); gap:var(--s2); }
  .linkcard { display:flex; flex-direction:column; gap:2px; padding:var(--s2) var(--s3); border:1px solid var(--line); border-radius:8px; background:#fafcfb; color:var(--ink); text-decoration:none; }
  .linkcard:hover { background:var(--brand-soft); border-color:#9bb8ac; }
  .linkcard b { font-size:14px; color:var(--brand); }
  .linkcard span { font-size:13px; color:var(--mute); }
  #menuScrim { position:fixed; inset:0; z-index:1400; background:rgba(0,0,0,.35); }
  #menuDrawer { position:fixed; top:var(--hdr); right:0; bottom:var(--dock); width:min(380px,100vw); z-index:1500; overflow:auto; background:#fff; border-left:1px solid var(--line); box-shadow:-4px 0 14px rgba(0,0,0,.2); padding:var(--s3) var(--s4); }
  #menuDrawer .linkgrid { grid-template-columns:1fr; }
  .drawer-head { display:flex; justify-content:space-between; align-items:center; margin-bottom:var(--s3); }

  /* ---- ダッシュボードタブ ---- */
  #tab-dash.active { display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:var(--s3); align-items:start; }
  #tab-dash .s-summary, #tab-dash .s-links { grid-column:1 / -1; }
  details.s-links > summary { cursor:pointer; font-weight:700; font-size:14px; color:var(--brand); }
  details.s-links[open] > summary { margin-bottom:var(--s3); }
  .dash-col { display:flex; flex-direction:column; gap:var(--s3); min-width:0; }

  /* ---- 発話・音声 / ログ・分析 / 設定タブ ---- */
  #tab-voice.active, #tab-logs.active, #tab-settings.active { display:block; max-width:1500px; }
  /* グループ見出し+カードのグリッド(2026-10-10): 同じ役割のカードを1つの見出しの下にまとめる */
  .grp { margin-bottom:var(--s5); }
  .grp-head { margin:0 0 var(--s2); padding-left:var(--s3); border-left:4px solid var(--brand2); }
  h2.grp-title { margin:0; font-size:16px; color:var(--brand); }
  .grp-note { font-size:13px; color:var(--mute); margin:2px 0 0; }
  .grp-grid { display:grid; grid-template-columns:repeat(auto-fill,minmax(min(420px,100%),1fr)); gap:var(--s3); align-items:start; }
  .grp-grid > .card { min-width:0; }
  /* カード見出し行+状態バッジ */
  .card-head { display:flex; align-items:center; justify-content:space-between; gap:var(--s2); flex-wrap:wrap; margin-bottom:var(--s1); }
  .card-head h2, .card-head h3 { margin:0; }
  .card-desc { font-size:13px; color:var(--mute); margin:0 0 var(--s2); }
  .sbadge { display:inline-block; padding:2px 10px; border-radius:999px; font-size:13px; font-weight:700; background:#e6ebe8; color:var(--mute); }
  .sb-ok { background:var(--ok-bg); color:var(--ok); }
  .sb-warn { background:var(--warn-bg); color:var(--warn); }
  .sb-bad { background:var(--bad-bg); color:var(--bad); }
  .sb-mute { background:#e6ebe8; color:var(--mute); }
  .sumgrid { display:grid; grid-template-columns:repeat(auto-fit,minmax(150px,1fr)); gap:var(--s2); }
  .sumitem { display:flex; flex-direction:column; align-items:flex-start; gap:4px; padding:var(--s2) var(--s3); border:1px solid var(--line); border-radius:8px; background:#fafcfb; color:var(--ink); text-decoration:none; min-width:0; }
  .sumitem:hover { background:var(--brand-soft); border-color:#9bb8ac; }
  .sum-l { font-size:13px; color:var(--mute); }
  .sumitem .sbadge { white-space:normal; font-size:13px; }
  .metric-sub { display:block; font-style:normal; font-size:13px; color:var(--mute); }
  .subpanel { margin-top:var(--s3); padding:var(--s3); border:1px solid var(--line); border-radius:8px; background:#f7faf8; }
  .subpanel h3 { color:var(--ink); }
  .danger-zone { margin-top:var(--s3); padding-top:var(--s2); border-top:1px dashed var(--line); }

  /* ---- 設定タブの配置(2026-10-22): 目次 / 2カラム / 折りたためるカード / コンパクトな状態表 / 危険な操作 ---- */
  .grp { scroll-margin-top:calc(var(--hdr) + var(--s3)); }
  .set-toc { display:flex; flex-wrap:wrap; align-items:center; gap:var(--s2); margin:0 0 var(--s3); }
  .set-toc-l { font-size:13px; color:var(--mute); }
  .set-toc a { padding:4px 12px; border:1px solid var(--line); border-radius:999px; background:var(--card); color:var(--brand); font-size:13px; font-weight:700; text-decoration:none; }
  .set-toc a:hover { background:var(--brand-soft); border-color:#9bb8ac; }
  .set-cols { display:flex; flex-direction:column; }
  .set-col { min-width:0; }
  @media (min-width:1100px) {
    /* 【2026-11-02】設定タブは、2列に「高さが同じくらいになるよう自動で詰めて」並べる(以前は左右の列が別々で、
       片方が短いと下に大きな空きができた)。各グループは途中で切れないようにし、並びは文書の順(上→下、左→右)。 */
    .set-cols { display:block; column-count:2; column-gap:var(--s4); }
    .set-col { display:contents; }
    .set-cols .grp { break-inside:avoid; -webkit-column-break-inside:avoid; margin-bottom:var(--s4); }
    #tab-settings .grp-grid { grid-template-columns:minmax(0,1fr); }
    /* 「発話・音声」「ログ・分析」も同じく、グループを2列に自動で詰める(1つのカードだけの行で右が空かないように)。 */
    #tab-voice.active, #tab-logs.active { column-count:2; column-gap:var(--s4); }
    #tab-voice .grp, #tab-logs .grp { break-inside:avoid; -webkit-column-break-inside:avoid; }
    #tab-voice .grp-grid, #tab-logs .grp-grid { grid-template-columns:minmax(0,1fr); }
  }
  details.cf > summary { cursor:pointer; list-style:none; margin-bottom:0; }
  details.cf > summary::-webkit-details-marker { display:none; }
  details.cf > summary::before { content:"\25B8"; color:var(--brand); font-size:13px; }
  details.cf[open] > summary::before { content:"\25BE"; }
  details.cf[open] > summary { margin-bottom:var(--s2); }
  .cf-sum { display:flex; align-items:center; gap:var(--s2); flex-wrap:wrap; }
  .cf-sum h2, .cf-sum h3 { flex:1 1 auto; }
  .cf-note { flex:1 1 100%; display:none; font-size:13px; color:var(--mute); }
  details.cf:not([open]) .cf-note:not(:empty) { display:block; }
  details.cf-sub { margin-top:var(--s3); }
  details.cf-sub > summary h3 { color:var(--ink); }
  .kv { grid-template-columns:repeat(auto-fill,minmax(220px,1fr)); gap:2px var(--s3); }
  .kv .metric { display:flex; justify-content:space-between; align-items:baseline; gap:var(--s2); background:none; border-radius:0; border-left:0; border-bottom:1px solid var(--line); padding:4px 2px; }
  .kv .metric span { display:inline; flex:0 0 auto; }
  .kv .metric b { font-size:14px; text-align:right; }
  .set-danger { grid-column:1 / -1; padding:var(--s3); border:1px dashed var(--bad); border-radius:var(--r); background:var(--bad-bg); }
  .set-danger-title { margin:0 0 var(--s2); font-size:13px; font-weight:700; color:var(--bad); }

  /* ---- 地図・走行タブ(地図が主役。画面高にぴったり収める) ---- */
  /* 【2026-10-08】地図が主役: 地図の高さは「ツールの数」に左右されず一定(下の.map-stage)。足りない分はページを縦スクロールする。 */
  #tab-map.active { display:grid; gap:var(--s3); grid-template-columns:minmax(0,1fr) 460px; align-items:start; }  /* 2026-10-22: 400->460px(カメラ映像を大きく) */
  .s-map { display:flex; flex-direction:column; min-height:0; min-width:0; align-self:start; position:sticky; top:calc(var(--hdr) + var(--s3)); }
  .map-head { display:flex; justify-content:space-between; align-items:baseline; gap:var(--s3); flex-wrap:wrap; flex:0 0 auto; }
  .map-head h2 { margin:0 0 var(--s2); }
  .map-head h2 small { font-weight:400; font-size:13px; color:var(--mute); }
  /* 地図ツールバー(1本。ポップオーバーはツールバー基準で開く) */
  .map-toolbar { position:relative; z-index:30; display:flex; flex-wrap:wrap; gap:6px; align-items:center; margin:0 0 var(--s2); flex:0 0 auto; }
  .map-toolbar .tb-btn, .tb-pop button:not(.pin-swatch) { padding:4px 10px; min-height:34px; font-size:13px; }
  .tb-btn { white-space:nowrap; }
  .tb-btn[aria-expanded="true"] { background:var(--brand-soft); border-color:var(--brand2); }
  .tb-wrap { display:inline-block; }
  .tb-right { margin-left:auto; }
  .tb-icon { width:34px; padding:4px 0 !important; border-radius:50%; font-weight:700; }
  .tb-pop { position:absolute; left:0; top:calc(100% + 4px); z-index:40; width:min(420px,100%); max-height:60vh; overflow:auto; padding:10px 12px; background:#fff; border:1px solid #9bb8ac; border-radius:10px; box-shadow:0 8px 24px rgba(0,0,0,.28); display:flex; flex-direction:column; gap:8px; }
  .tb-row { display:flex; flex-wrap:wrap; gap:6px; align-items:center; }
  .tb-row input[type=text] { flex:1 1 150px; min-width:110px; }
  .tb-lbl { font-size:13px; color:var(--mute); }
  .tb-help p { font-size:13px; line-height:1.55; }
  .tb-help p + p { margin-top:6px; }
  details.color-dd { position:relative; }
  details.color-dd > summary { cursor:pointer; list-style:none; display:inline-flex; align-items:center; gap:6px; padding:4px 10px; min-height:32px; font-size:13px; border:1px solid #bccbc3; border-radius:8px; background:#fff; }
  details.color-dd > summary::-webkit-details-marker { display:none; }
  details.color-dd[open] > summary { background:var(--brand-soft); }
  details.color-dd > .pin-swatches-row { margin-top:6px; }
  /* 地図ステージ: 高さは常に確保(コントロールが増えても潰れない)。通知は下端に重ねる */
  .map-stage { position:relative; flex:0 0 auto; min-width:0; height:max(480px, calc(100vh - var(--hdr) - var(--dock) - 150px)); }
  .map-notes { position:absolute; left:8px; right:8px; bottom:8px; z-index:5; display:flex; flex-direction:column; justify-content:flex-end; gap:4px; max-height:60%; overflow:hidden; pointer-events:none; }
  .map-notes > * { pointer-events:auto; margin:0; }
  .map-note { align-self:flex-start; max-width:100%; padding:3px 12px; border-radius:999px; background:rgba(12,27,22,.9); color:#fff; font-size:13px; cursor:pointer; display:-webkit-box; -webkit-line-clamp:2; -webkit-box-orient:vertical; overflow:hidden; }
  .map-note::after { content:"  \00d7"; opacity:.7; }
  .map-note:empty { display:none; }
  .map-notes .loc-bar, .map-notes .loc-warn { margin:0; padding:4px 8px; gap:6px; flex-wrap:nowrap; font-size:13px; box-shadow:0 2px 8px rgba(0,0,0,.4); }
  .map-notes .loc-bar > span { flex:1 1 0; min-width:0; overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }
  .map-notes .loc-bar button { min-height:28px; padding:2px 10px; font-size:13px; flex:0 0 auto; white-space:nowrap; }
  .map-mode { font-size:13px; color:var(--mute); }
  .chip-mute { background:#e6ebe8; color:var(--mute); }
  .map-loc { display:flex; align-items:center; gap:var(--s2); flex-wrap:wrap; }
  .loc-detail { font-size:13px; color:var(--mute); }
  .loc-warn { margin:6px 0 0; padding:6px 10px; border-radius:8px; background:var(--warn-bg); color:var(--warn); font-size:13px; font-weight:700; flex:0 0 auto; }
  .loc-bar { display:flex; flex-wrap:wrap; gap:var(--s2); align-items:center; margin-top:6px; padding:8px 10px; border-radius:8px; background:#f3e8ff; border:1px solid #a21caf; font-size:13px; font-weight:600; flex:0 0 auto; }
  .loc-bar-warn { background:var(--warn-bg); border-color:#d9a400; color:var(--warn); }
  .loc-bar > span { flex:1 1 240px; }
  .loc-bar-info { background:#e8f1ff; border-color:#2b6fe0; color:#12346b; }
  button.danger { background:#a32d2d; border-color:#a32d2d; color:#fff; }
  button.danger:hover:not(:disabled) { background:#7d2121; }
  /* 地図ピン(2026-10-07): ツールバー・一覧・色見本 */
  .pin-swatches, .pin-swatches-row { display:inline-flex; flex-wrap:wrap; gap:4px; align-items:center; }
  .pin-swatch { width:22px; height:22px; min-height:22px; padding:0; border-radius:50%; border:2px solid #fff; box-shadow:0 0 0 1px #8a9a92; }
  .pin-swatch.auto { width:auto; border-radius:12px; padding:0 8px; font-size:13px; background:#fff; color:var(--ink); }
  .pin-swatch.on { box-shadow:0 0 0 2px #111; }
  /* ピン一覧(右カラムの独立カード): 普通の行は「色・名前・距離・行く」の1行、選択中/編集中だけ操作ボタンを展開する */
  .s-pins { padding:0; }
  details.pins-card > summary { cursor:pointer; list-style:none; padding:var(--s3) var(--s4); font-size:15px; font-weight:700; }
  details.pins-card > summary::-webkit-details-marker { display:none; }
  details.pins-card > summary::before { content:"\25B8  "; color:var(--brand); }
  details.pins-card[open] > summary::before { content:"\25BE  "; }
  details.pins-card > summary small { font-weight:400; font-size:13px; color:var(--mute); }
  .pins-body { padding:0 var(--s4) var(--s3); }
  .pin-list { display:flex; flex-direction:column; gap:4px; max-height:min(340px,42vh); overflow:auto; }
  .pin-row { display:flex; flex-wrap:wrap; align-items:center; gap:4px 8px; border:1px solid var(--line); border-radius:8px; padding:4px 8px; background:#fff; }
  .pin-row.sel { border-color:#16a34a; background:#e8f6ee; box-shadow:0 0 0 2px #16a34a; }
  .pin-row .pin-head { flex:1 1 120px; min-width:0; }
  .pin-row .pin-btns { margin-top:0; flex:0 0 auto; }
  .pin-row.sel .pin-btns, .pin-row.editing .pin-btns { flex:1 1 100%; margin-top:2px; }
  .pin-row:not(.sel):not(.editing) .pin-btns > button:not(.primary) { display:none; }
  .pin-head { display:flex; align-items:center; gap:8px; cursor:pointer; }
  .pin-dot { width:14px; height:14px; border-radius:50%; border:2px solid #fff; box-shadow:0 0 0 1px #8a9a92; flex:0 0 auto; }
  .pin-name { flex:1 1 auto; min-width:0; overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }
  .pin-dist { font-size:13px; color:var(--mute); flex:0 0 auto; }
  .pin-btns { display:flex; flex-wrap:wrap; gap:4px; margin-top:6px; align-items:center; }
  .pin-btns button { padding:4px 10px; min-height:32px; font-size:13px; }
  .pin-edit-input { flex:1 1 140px; min-width:100px; }
  .pin-confirm-text { flex:1 1 100%; font-size:13px; font-weight:600; }
  .pin-sel-bar { background:#eefbf3; border-color:#16a34a; }
  canvas { width:100%; background:#0c1b16; border-radius:8px; cursor:crosshair; touch-action:none; }
  #mapCanvas { display:block; position:absolute; inset:0; width:100%; height:100%; }
  .sidecol { display:flex; flex-direction:column; gap:var(--s3); min-height:0; min-width:0; }
  .sidecol > .card { flex:0 0 auto; }
  .speed-grid { display:grid; grid-template-columns:repeat(4,minmax(0,1fr)); gap:var(--s2); margin-top:var(--s2); }
  .speed-grid label { display:flex; flex-direction:column; gap:2px; }
  .speed-grid input { width:100%; }
  .s-drive .hint { margin-top:var(--s2); }
  /* 【2026-10-22】カメラが3台(正面+後ろ左右)になったので、2列のグリッドに: LiDARと正面は全幅、後ろの2台は半幅ずつ並べる */
  .sensor-row { display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:var(--s2); }
  .sensor-row figure { margin:0; min-width:0; grid-column:span 2; }
  .sensor-row figure.sensor-half { grid-column:span 1; }
  .sensor-row figcaption { font-size:13px; color:var(--mute); margin-top:4px; }
  #lidarCanvas { height:auto; aspect-ratio:1/1; cursor:default; }
  /* 【2026-10-22】カメラを上に大きく、LiDARは最後に小さめで中央 */
  .sensor-row figure.sensor-lidar { order:3; max-width:360px; margin-inline:auto; }
  /* カメラ映像をクリックで画面いっぱいに拡大(もう一度クリック/Escで戻す) */
  #cameraView, #cameraViewLeft, #cameraViewRight { cursor:zoom-in; }
  .cam-zoom { position:fixed !important; inset:4vh 4vw; width:92vw !important; height:92vh !important; aspect-ratio:auto !important; z-index:1000; cursor:zoom-out !important; box-shadow:0 0 0 100vmax rgba(0,0,0,.78); }
  #cameraView { display:block; width:100%; height:auto; aspect-ratio:16/9; object-fit:contain; background:#0c1b16; border-radius:8px; }
  #cameraViewLeft, #cameraViewRight { display:block; width:100%; height:auto; aspect-ratio:4/3; object-fit:contain; background:#0c1b16; border-radius:8px; }
  details.fold { border:1px solid var(--line); border-radius:8px; margin-top:var(--s2); background:#fafcfb; }
  details.fold:first-of-type { margin-top:0; }
  details.fold > summary { cursor:pointer; padding:var(--s2) var(--s3); font-weight:700; font-size:13px; color:var(--brand); list-style:none; }
  details.fold > summary::before { content:"\25B8  "; }
  details.fold[open] > summary::before { content:"\25BE  "; }
  details.fold > .fold-body { padding:0 var(--s3) var(--s3); }
  .linkbtn { display:block; text-align:center; text-decoration:none; padding:7px 12px; border:1px solid #bccbc3; border-radius:8px; background:#fff; color:var(--ink); font-size:13px; flex:1 1 auto; }
  .linkbtn:hover { background:var(--brand-soft); }

  @media (min-width:1600px) {
    .s-drive .speed-grid { grid-template-columns:1fr 1fr; }
    #tab-map.active { grid-template-columns:minmax(0,1fr) 340px 580px; }  /* 2026-10-22: センサー列を340->580px(カメラ映像を大きく) */
    #tab-map.active { grid-template-rows:auto auto 1fr; }  /* 余った高さは最下段(地図の操作)に回し、カード間に隙間を作らない */
    .sidecol { display:contents; }
    .s-map { grid-column:1; grid-row:1 / 4; }
    .s-pins { grid-column:2; grid-row:1; }
    .s-drive { grid-column:2; grid-row:2; }
    .s-sensors { grid-column:3; grid-row:1 / 4; }
    .s-folds { grid-column:2; grid-row:3; }
    .sensor-row { gap:var(--s3) var(--s2); }
  }
  @media (max-width:1199px) {
    #tab-map.active { display:flex; flex-direction:column; align-items:stretch; }
    .s-map { position:static; align-self:stretch; width:auto; }  /* 【2026-11-03】幅1199px以下(iPadの縦など)で地図のカードだけ細くなり右が空く問題 */
    .map-stage { height:max(480px, 60vh); }
  }
  @media (max-width:760px) {
    #topbar { padding:var(--s2) var(--s3); gap:var(--s2); }
    #tabs { order:3; flex:1 0 100%; }
    .header-right { flex:1 1 100%; flex-wrap:wrap; }
    #tab-dash.active { grid-template-columns:minmax(0,1fr); }
    .tabpane { padding:var(--s2) var(--s3); }
    #tabs a { padding:7px 10px; font-size:13px; }
    .brand-sub { display:none; }
    .speed-grid { grid-template-columns:1fr 1fr; }
    .sensor-row { flex-direction:column; }
    .map-stage { height:max(340px, 60vh); }
    .map-notes .loc-bar { flex-wrap:wrap; }
    .map-notes .loc-bar > span { flex:1 1 100%; white-space:normal; }
  }
"""

NK_FIELD_JS = r"""
  // 【2026-10-10】NKField: 入力欄と定期更新(ポーリング)の競合を防ぐ共通ヘルパー(docs/dashboard_field_sync.md)。
  // 守ること: (1)編集中(フォーカス・IME変換中・ドラッグ中・未送信の編集=dirty・送信確認中)の欄は、取得した値で上書きしない
  // (2)送信したら世代番号を進め、送信前に始まった古い取得結果は捨てる (3)送信後はサーバーの値を読み直し、送った値と一致したと
  // 確認できてからdirtyを解除する(不一致は「反映されていません(サーバー値 X)」を出して入力値を残す)
  // (4)範囲外の値は入力時に表示 (5)非表示タブではポーリングを止め、戻ったらすぐ取り直す。
  (function () {
    if (window.NKField) return;
    const S = new WeakMap();
    const RANGES = new Set();
    const BOUND = new Set();     // 登録済みの入力欄(テストが「全入力が登録済みか」を調べるのに使う)
    const WATCHED = [];          // watch済みの領域
    const GEN = Object.create(null);
    const POLLS = [];
    const ROOTS = new WeakMap();
    const TOL = 1e-6;
    const byId = (x) => (typeof x === "string" ? document.getElementById(x) : x);
    const isCheck = (el) => el.type === "checkbox" || el.type === "radio";
    const read = (el) => (isCheck(el) ? el.checked : el.value);
    function st(el) {
      let s = S.get(el);
      if (!s) { s = { dirty: false, focus: false, comp: false, drag: false, sending: false, pending: undefined, server: undefined, noteEl: null, bound: false, opts: {} }; S.set(el, s); }
      return s;
    }
    function same(el, a, b) {
      if (isCheck(el)) return !!a === !!b;
      if (el.type === "number" || el.type === "range" || (el.dataset && el.dataset.nkNum)) {
        const x = Number(a), y = Number(b);
        if (String(a).trim() !== "" && String(b).trim() !== "" && Number.isFinite(x) && Number.isFinite(y)) return Math.abs(x - y) <= (st(el).opts.tol || TOL);
      }
      return String(a) === String(b);
    }
    function ownBusy(el) {
      const s = st(el);
      return s.dirty || s.sending || s.comp || s.drag || s.focus || document.activeElement === el;
    }
    // スライダーと数値欄のように、同じ値を表す2つの欄は片方が編集中ならもう片方も上書きしない(link)
    function busy(el) {
      const s = st(el);
      return ownBusy(el) || (s.partner ? ownBusy(s.partner) : false);
    }
    function write(el, v) {
      if (isCheck(el)) { if (el.checked !== !!v) el.checked = !!v; }
      else if (el.value !== String(v)) el.value = String(v);
    }
    function note(el, text, kind) {
      const s = st(el);
      if (!text) { if (s.noteEl) { s.noteEl.remove(); s.noteEl = null; } return; }
      if (!s.noteEl) {
        const n = document.createElement("small");
        n.setAttribute("role", "status");
        el.insertAdjacentElement("afterend", n);
        s.noteEl = n;
      }
      s.noteEl.className = "nk-note nk-" + (kind || "info");
      s.noteEl.textContent = text;
    }
    function lim(el, k) {
      const o = st(el).opts[k];
      const raw = o !== undefined && o !== null ? o : el.getAttribute(k);
      if (raw === null || raw === undefined || raw === "") return null;
      const v = Number(raw);
      return Number.isFinite(v) ? v : null;
    }
    // 範囲外・数値でない入力を入力時に知らせる(サーバー検証と同じ範囲=min/max属性。opts.clamp=サーバーが丸める欄)
    function check(el, final) {
      if (el.type !== "number") return true;
      const s = st(el);
      let msg = "";
      const raw = String(el.value).trim();
      if (el.validity && el.validity.badInput) { if (final || document.activeElement !== el) msg = "数値で入力してください"; }   // 入力の途中(「-」だけ等)では出さない
      else if (raw !== "") {
        const v = Number(raw), lo = lim(el, "min"), hi = lim(el, "max");
        if (Number.isFinite(v) && ((lo !== null && v < lo - TOL) || (hi !== null && v > hi + TOL))) {
          const bound = (lo !== null && v < lo) ? lo : hi;
          msg = s.opts.rangeText || ("範囲は " + (lo !== null ? lo : "") + "〜" + (hi !== null ? hi : "") + " です"
            + (s.opts.clamp ? "(範囲外の値は " + bound + " に丸められます)" : "(範囲外の値は保存できません)"));
        }
      }
      el.classList.toggle("nk-invalid", !!msg);
      note(el, msg, msg ? "bad" : "");
      return !msg;
    }
    function put(el, v) {
      const s = st(el);
      s.server = v;
      write(el, v);
      s.dirty = false; s.pending = undefined;
      if (el.type === "number") el.classList.remove("nk-invalid");
      note(el, "", "");
    }
    function flush(el) {
      const s = st(el);
      if (s.pending !== undefined && !busy(el)) {
        const v = s.pending;
        put(el, v);
        if (s.partner) put(s.partner, v);
      }
      if (s.partner) { const p = st(s.partner); if (p.pending !== undefined && !busy(s.partner)) { const v = p.pending; put(s.partner, v); put(el, v); } }
    }
    function bind(el, o) {
      if (!el) return el;
      const s = st(el);
      if (o) Object.assign(s.opts, o);
      if (s.bound) return el;
      s.bound = true;
      BOUND.add(el);
      const onEdit = (ev) => {
        s.dirty = s.server === undefined ? true : !same(el, read(el), s.server);
        if (s.partner) st(s.partner).dirty = s.dirty;
        check(el, ev && ev.type === "change");
      };
      el.addEventListener("input", onEdit);
      el.addEventListener("change", onEdit);
      el.addEventListener("focus", () => { s.focus = true; });
      el.addEventListener("blur", () => { s.focus = false; flush(el); });
      el.addEventListener("compositionstart", () => { s.comp = true; });
      el.addEventListener("compositionend", () => { s.comp = false; flush(el); });
      if (el.type === "range") {
        RANGES.add(el);
        el.addEventListener("pointerdown", () => { s.drag = true; });
        el.addEventListener("touchstart", () => { s.drag = true; }, { passive: true });
      }
      return el;
    }
    ["pointerup", "pointercancel", "touchend", "touchcancel", "mouseup", "blur"].forEach((n) => {
      window.addEventListener(n, () => { RANGES.forEach((el) => { const s = st(el); if (s.drag) { s.drag = false; flush(el); } }); }, true);
    });
    function list(x) {
      if (x === undefined || x === null) return [];
      if (typeof x === "string") return [byId(x)].filter(Boolean);
      if (x instanceof Element) return [x];
      return Array.from(x).reduce((a, e) => a.concat(list(e)), []);
    }
    function guard(x, o) { return list(x).map((el) => bind(el, o)); }
    function guardAll(sel, o) { return guard(Array.from(document.querySelectorAll(sel)), o); }
    // サーバーの値を入力欄へ反映する。編集中・dirty・送信確認中なら書かず(保留して)falseを返す。force=trueなら必ず書く
    function set(x, value, o) {
      const el = byId(x);
      if (!el) return false;
      const s = st(el);
      bind(el);
      const v = isCheck(el) ? !!value : String(value);
      s.server = v;
      if (s.partner) st(s.partner).server = v;
      if (!(o && o.force) && busy(el)) { s.pending = v; return false; }
      put(el, v);
      if (s.partner) put(s.partner, v);
      return true;
    }
    function link(a, b) {
      a = byId(a); b = byId(b);
      if (!a || !b) return;
      bind(a); bind(b);
      st(a).partner = b; st(b).partner = a;
    }
    function isDirty(x) { const el = byId(x); return !!el && st(el).dirty; }
    function dirtyOf(xs) { return list(xs).filter((el) => st(el).dirty); }
    function clean(x) { list(x).forEach((el) => { const s = st(el); s.dirty = false; s.pending = undefined; note(el, "", ""); el.classList.remove("nk-invalid"); }); }
    function editing(root) {
      root = byId(root);
      const a = document.activeElement;
      if (!root) return false;
      const rs = ROOTS.get(root);
      if (rs && rs.comp) return true;
      return !!(a && a !== document.body && root.contains(a) && /^(INPUT|SELECT|TEXTAREA)$/.test(a.tagName));
    }
    // 入力欄のある領域(innerHTMLで作り直す一覧など)の再描画を、編集中は後回しにする。編集が終わったら自動で実行する
    function watch(root, o) {
      root = byId(root);
      if (!root) return null;
      let rs = ROOTS.get(root);
      if (rs) return rs;
      rs = { comp: false, later: null };
      ROOTS.set(root, rs);
      WATCHED.push(root);
      const isField = (t) => t && t.matches && t.matches("input,select,textarea");
      const idle = () => { setTimeout(() => { if (rs.later && !editing(root)) { const f = rs.later; rs.later = null; f(); } }, 0); };
      root.addEventListener("input", (ev) => { if (isField(ev.target)) check(ev.target); }, true);
      root.addEventListener("compositionstart", () => { rs.comp = true; }, true);
      root.addEventListener("compositionend", () => { rs.comp = false; idle(); }, true);
      root.addEventListener("focusout", idle, true);
      return rs;
    }
    function defer(root, fn) {
      root = byId(root);
      const rs = watch(root);
      if (rs && editing(root)) { rs.later = fn; return false; }
      fn();
      return true;
    }
    function bump(k) { GEN[k] = (GEN[k] || 0) + 1; }
    function ticket(k) { return { k: k, g: GEN[k] || 0 }; }
    function fresh(t) { return (GEN[t.k] || 0) === t.g; }
    function judge(it, srv) {
      let v;
      try { v = it.read(srv); } catch (e) { v = undefined; }
      if (v === undefined || v === null) return { el: it.el, label: it.label, sent: it.sent, server: undefined, status: "unknown" };
      if (same(it.el, it.sent, v)) return { el: it.el, label: it.label, sent: it.sent, server: v, status: "ok" };
      if (it.accept && it.accept(it.sent, v, srv)) return { el: it.el, label: it.label, sent: it.sent, server: v, status: "clamped" };
      const lo = lim(it.el, "min"), hi = lim(it.el, "max"), n = Number(it.sent);
      if (Number.isFinite(n) && ((lo !== null && n < lo && same(it.el, v, lo)) || (hi !== null && n > hi && same(it.el, v, hi)))) {
        return { el: it.el, label: it.label, sent: it.sent, server: v, status: "clamped" };
      }
      return { el: it.el, label: it.label, sent: it.sent, server: v, status: "mismatch" };
    }
    // 送信の開始。以後、finish(done/fail/verify)までは同じキーの古い取得結果を捨て、対象の欄を上書きしない
    function begin(key, els) {
      bump(key);
      const targets = list(els);
      targets.forEach((el) => { st(el).sending = true; });
      let finished = false;
      const tx = {
        key: key, els: targets,
        done: function () {
          if (finished) return;
          finished = true;
          targets.forEach((el) => { const s = st(el); s.sending = false; s.pending = undefined; });
          bump(key);
        },
        fail: function (msg) { tx.done(); if (msg) targets.forEach((el) => note(el, msg, "bad")); },
        // サーバーの値を読み直して、送った値と一致したか確かめる。一致した欄だけdirtyを解除して表示を更新する
        // o.items: [{el, sent, read(serverObj), label}] / o.load: async () => serverObj|null / o.delays: 再確認の待ち(ms)
        verify: async function (o) {
          const items = o.items.map((it) => ({ el: byId(it.el), sent: it.sent, read: it.read, label: it.label || (byId(it.el) || {}).id, accept: it.accept }));
          let srv = null, rows = items.map((it) => judge(it, {}));
          for (const d of (o.delays || [0, 700, 1500, 3000])) {
            if (d) await new Promise((r) => setTimeout(r, d));
            let cur = null;
            try { cur = await o.load(); } catch (e) { cur = null; }
            if (!cur) continue;
            srv = cur;
            rows = items.map((it) => judge(it, cur));
            if (rows.every((r) => r.status === "ok" || r.status === "clamped")) break;
          }
          for (const r of rows) {
            if (!r.el) continue;
            if (r.status === "ok") set(r.el, r.server, { force: true });
            else if (r.status === "clamped") { set(r.el, r.server, { force: true }); note(r.el, r.sent + " は範囲外のため " + r.server + " に丸められました", "warn"); }
            else if (r.status === "mismatch") note(r.el, "反映されていません(サーバー値 " + r.server + ")", "bad");
            else note(r.el, "反映を確認できませんでした(サーバーから値を読めません)", "warn");
          }
          tx.done();
          const state = rows.some((r) => r.status === "mismatch") ? "mismatch" : (rows.some((r) => r.status === "unknown") ? "unknown" : (rows.some((r) => r.status === "clamped") ? "clamped" : "ok"));
          return { state: state, rows: rows, server: srv };
        },
      };
      return tx;
    }
    // 【2026-10-13】プログラム終了(docs/system_shutdown.md): 全ての定期処理(setInterval/setTimeout)と通信を止める。
    // 停止後は管理画面に繋がらないので、接続エラーを大量に出さないため。以後のfetchは応答を返さない(エラーにもならない)。
    let FROZEN = false;
    function freeze() {
      if (FROZEN) return;
      FROZEN = true;
      const hi = setTimeout(function () {}, 0);
      for (let i = 0; i <= hi; i++) { clearInterval(i); clearTimeout(i); }
      window.fetch = function () { return new Promise(function () {}); };
    }
    // 定期取得。非表示の間は動かさず、戻ったらすぐ取り直す。前回が終わっていなければ重ねて動かさない
    function poll(fn, ms) {
      const p = { ms: ms, last: 0, running: false };
      p.run = async function () {
        if (FROZEN || document.hidden || p.running) return;
        p.running = true;
        try { await fn(); } catch (e) { /* 次回に再試行 */ } finally { p.running = false; p.last = Date.now(); }
      };
      p.timer = setInterval(p.run, ms);
      POLLS.push(p);
      return p;
    }
    document.addEventListener("visibilitychange", () => { if (!document.hidden) POLLS.forEach((p) => { if (Date.now() - p.last > p.ms / 2) p.run(); }); });
    // select等の選択肢の作り直し。開いている/編集中なら後回しにし、中身が同じなら何もしない
    function setOptions(x, values, labelOf) {
      const el = byId(x);
      if (!el) return false;
      const cur = Array.from(el.options).map((o) => o.value);
      const next = values.map(String);
      if (cur.length === next.length && cur.every((v, i) => v === next[i])) return false;
      const s = st(el);
      if (s.focus || document.activeElement === el) { s.pendingOptions = [values, labelOf]; return false; }
      const keep = el.value;
      el.innerHTML = "";
      next.forEach((v) => { const o = document.createElement("option"); o.value = v; o.textContent = labelOf ? labelOf(v) : v; el.appendChild(o); });
      if (next.includes(keep)) el.value = keep;
      return true;
    }
    function isEnter(ev) { return ev.key === "Enter" && !ev.isComposing && ev.keyCode !== 229; }
    const css = document.createElement("style");
    css.textContent = ".nk-note{display:block;flex:1 1 100%;font-size:13px;line-height:1.4;margin-top:2px}.nk-bad{color:#b3261e;font-weight:700}.nk-warn{color:#7a5b00;font-weight:700}.nk-ok{color:#0f6e56}.nk-info{color:#53635c}.nk-invalid{outline:2px solid #d6735a;outline-offset:1px}";
    (document.head || document.documentElement).appendChild(css);
    window.NKField = { guarded: () => Array.from(BOUND), watched: () => WATCHED.slice(), link: link, guard: guard, guardAll: guardAll, set: set, isDirty: isDirty, dirtyOf: dirtyOf, clean: clean, note: note, check: check, busy: (x) => busy(byId(x)),
      editing: editing, watch: watch, defer: defer, begin: begin, ticket: ticket, fresh: fresh, bump: bump, poll: poll, freeze: freeze, frozen: () => FROZEN, setOptions: setOptions, isEnter: isEnter, version: 1 };
  })();
  // 【2026-10-13】同時1端末制限(docs/open_items.md)で他の端末に操作権を取られたとき、通信が409(in_use)で
  // 失敗し続けて画面が止まったように見えるのを防ぐ。上部に案内を出し、通信が通ったら消す(fetchを包むだけ)。
  (function () {
    if (window.__nkInUseWatch || typeof window.fetch !== "function") return;
    window.__nkInUseWatch = true;
    const rawFetch = window.fetch.bind(window);
    let bar = null;
    function show() {
      if (bar || !document.body) return;
      bar = document.createElement("div");
      bar.setAttribute("role", "alert");
      bar.style.cssText = "position:fixed;top:0;left:0;right:0;z-index:1800;background:#993c1d;color:#fff;padding:10px 14px;font-size:14px;font-weight:700;text-align:center";
      bar.textContent = "他の端末がこのアカウントで操作中です。この画面からは操作できません(相手が止めて約8秒後に自動で戻ります)。";
      document.body.appendChild(bar);
    }
    function hide() { if (bar) { bar.remove(); bar = null; } }
    window.fetch = async (...args) => {
      const res = await rawFetch(...args);
      try {
        if (res.status === 409) {
          res.clone().json().then((d) => { if (d && d.error === "in_use") show(); }).catch(() => {});
        } else if (res.ok) {
          hide();
        }
      } catch (e) { /* 表示だけの補助。失敗しても通信には影響させない */ }
      return res;
    };
  })();
"""

NAV_JS = NK_FIELD_JS + r"""
  // 【2026-10-06】タブ・メニュードロワー・状態表示の色付け(安全系の処理には触れない)。
  (function () {
    const KEY = "nokosahen_full_tab";
    const tabs = Array.from(document.querySelectorAll("#tabs [data-tab]"));
    const panes = Array.from(document.querySelectorAll(".tabpane"));
    function show(name, save) {
      if (!panes.some((p) => p.dataset.pane === name)) name = "dash";
      tabs.forEach((t) => { const on = t.dataset.tab === name; t.classList.toggle("active", on); t.setAttribute("aria-selected", on ? "true" : "false"); });
      panes.forEach((p) => p.classList.toggle("active", p.dataset.pane === name));
      if (save) { try { localStorage.setItem(KEY, name); } catch (e) { /* 保存できなくても動く */ } }
      // 非表示だった地図キャンバスの大きさを取り直させる
      requestAnimationFrame(() => window.dispatchEvent(new Event("resize")));
    }
    tabs.forEach((t) => t.addEventListener("click", (ev) => { ev.preventDefault(); show(t.dataset.tab, true); closeDrawer(); }));
    let first = "dash";
    try { first = localStorage.getItem(KEY) || "dash"; } catch (e) { /* 既定のタブ */ }
    if (/^#tab-/.test(location.hash)) first = location.hash.slice(5);
    // ドロワー
    const drawer = document.getElementById("menuDrawer"), scrim = document.getElementById("menuScrim");
    const toggle = document.getElementById("menuToggle"), closeBtn = document.getElementById("menuClose");
    function closeDrawer() { drawer.hidden = true; scrim.hidden = true; toggle.setAttribute("aria-expanded", "false"); }
    function openDrawer() { drawer.hidden = false; scrim.hidden = false; toggle.setAttribute("aria-expanded", "true"); }
    toggle.addEventListener("click", (ev) => { ev.preventDefault(); if (drawer.hidden) openDrawer(); else closeDrawer(); });
    closeBtn.addEventListener("click", (ev) => { ev.preventDefault(); closeDrawer(); });
    scrim.addEventListener("click", closeDrawer);
    drawer.addEventListener("click", (ev) => { if (ev.target.closest && ev.target.closest("a.linkcard")) setTimeout(closeDrawer, 0); });
    window.addEventListener("keydown", (ev) => { if (ev.key === "Escape") closeDrawer(); });
    // 【2026-10-21】OCR確認画面(8091)は、1台目の内側だけで待ち受けるようにした。ここは、設定タブのOCRカード(映像・読み取り結果)へ移動する
    document.querySelectorAll("a[data-ocr]").forEach((a) => {
      a.href = "#ocrBlock";
      a.addEventListener("click", (ev) => { ev.preventDefault(); show("settings", true); const b = document.getElementById("ocrBlock"); if (b) b.scrollIntoView({ block: "start" }); });
    });
    show(first, false);
    // 「いまの状況」などの[data-goto]は、指定タブへ切り替える(タブ間の移動を1タップに)
    document.querySelectorAll("[data-goto]").forEach((a) => a.addEventListener("click", (ev) => { ev.preventDefault(); show(a.dataset.goto, true); window.scrollTo(0, 0); }));

    // 状態の色付け: 既存の更新処理が書き込むテキストを見て、親の.metricに色を付ける。
    function metricState(id, text) {
      const t = (text || "").trim();
      if (!t || t === "-") return "";
      if (id === "mode") return /HOLD$|^WAIT_FOR_SENSOR$|ESTOP/.test(t) ? "bad" : (/^POLICY_/.test(t) ? "warn" : "ok");
      if (id === "manualOverride") return t === "はい" ? "warn" : "ok";
      if (id === "activeRule") return /^なし/.test(t) ? "" : "ok";
      return "";
    }
    ["mode", "manualOverride", "activeRule"].forEach((id) => {
      const el = document.getElementById(id);
      if (!el || !el.parentElement) return;
      const apply = () => { const st = metricState(id, el.textContent); if (st) el.parentElement.dataset.state = st; else delete el.parentElement.dataset.state; };
      new MutationObserver(apply).observe(el, { childList: true, characterData: true, subtree: true });
      apply();
    });
    // ヘッダー: 接続断は赤、ソフト緊急停止中はチップ表示
    const badge = document.getElementById("estopBadge");
    new MutationObserver(() => {
      if (badge.textContent === "接続断" && !badge.classList.contains("estop-bad")) badge.className = "estop-badge estop-bad";
    }).observe(badge, { childList: true, characterData: true, subtree: true });
    const estopBtnEl = document.getElementById("estopBtn"), softChip = document.getElementById("softEstopChip");
    if (estopBtnEl && softChip) {
      const sync = () => { softChip.hidden = !estopBtnEl.classList.contains("engaged"); };
      new MutationObserver(sync).observe(estopBtnEl, { attributes: true, attributeFilter: ["class"] });
      sync();
    }
  })();

  // 【2026-10-10】状態バッジと「いまの状況」: 既存の更新処理が書いた文言(DOM)を読んで表示するだけ。API・ハンドラは変えない。
  (function () {
    const $ = (id) => document.getElementById(id);
    const T = (id) => { const e = $(id); return e ? (e.textContent || "").trim() : ""; };
    const off = (id) => { const e = $(id); return !e || e.style.display === "none"; };   // 未設定(JSが非表示にしたカード)
    function badge(id, cls, text, title) {
      const e = $(id); if (!e) return;
      const c = "sbadge sb-" + cls;
      if (e.className !== c) e.className = c;
      if (e.textContent !== text) e.textContent = text;
      if (title !== undefined && e.title !== title) e.title = title;
    }
    // 行動モードの内部名 -> 日本語(未知の名前はそのまま表示)
    const MODE_JA = {
      NEAR_PERSON_HOLD: "人物が近いため停止", MAP_NOT_LOADED_HOLD: "地図を読み込んでください", FACE_PERSON: "人の方を向いています", FACE_SWEEP: "人を探して見回しています", OUT_OF_BOUNDS_HOLD: "走行範囲外のため停止", KEEPOUT_ZONE_HOLD: "立入禁止エリアのため停止",
      CAMERA_OBSTACLE_HOLD: "障害物を検知して停止", MAPPING_ACTIVE_HOLD: "地図作成中のため停止", SURVEY_ACTIVE_HOLD: "アンケート回答中のため停止",
      WAIT_FOR_SENSOR: "センサー待ちで停止", ESTOP_HOLD: "緊急停止中", MANUAL_OVERRIDE: "手動運転中", APPROACH_PERSON: "人に近づいている",
      APPROACH_ARRIVED: "人のそばに到着", STOP_ANNOUNCE: "止まって呼びかけ中", POLICY_STAY: "方針により待機", POLICY_GO_ENTRANCE: "入口へ移動中(方針)",
      POLICY_AT_ENTRANCE: "入口で待機(方針)", POLICY_GO_PIN: "ピンへ移動中(方針)", POLICY_AT_PIN: "ピンで待機(方針)",
      POLICY_PIN_UNAVAILABLE: "方針のピンが使えず待機", POLICY_NO_SCAN: "見回し停止(方針)",
    };
    function tick() {
      const eb = $("estopBadge");
      if (eb) badge("sumStop", /estop-bad/.test(eb.className) ? "bad" : (/estop-policy/.test(eb.className) ? "warn" : (/estop-ok/.test(eb.className) ? "ok" : "mute")), T("estopBadge") || "-");
      const m = T("mode"), ja = MODE_JA[m] || "";
      const mj = $("modeJa"); if (mj && mj.textContent !== ja) mj.textContent = ja;
      badge("sumMode", (!m || m === "-") ? "mute" : (/HOLD$|^WAIT_FOR_SENSOR$|ESTOP/.test(m) ? "bad" : (/^POLICY_/.test(m) ? "warn" : "ok")), ja || m || "-", m);
      const lc = $("locChip");
      if (lc) badge("sumLoc", /chip-ok/.test(lc.className) ? "ok" : (/chip-warn/.test(lc.className) ? "warn" : (/chip-bad/.test(lc.className) ? "bad" : "mute")), T("locChip").replace(/^位置の確かさ:\s*/, "") || "-");
      // 音声の全体ON/OFF
      const vs = T("voiceMasterState");
      let v = ["mute", "未設定", "未設定"];
      if (!off("voiceMasterBlock")) v = /OFF/.test(vs) ? ["bad", "OFF(無音)", "停止中"] : (/ON/.test(vs) ? ["ok", "ON", "動作中"] : ["warn", "要確認", "要確認"]);
      badge("sumVoice", v[0], v[1]); badge("voiceMasterBadge", v[0], v[2]);
      const voiceOff = v[0] === "bad";
      // BGM(状態文は読み取りのみ)と音量カード
      const bs = T("bgmStatus");
      let g = ["mute", "未設定"];
      if (!off("bgmBlock")) g = /接続できません/.test(bs) ? ["warn", "要確認"] : (/全体OFF/.test(bs) ? ["warn", "音声OFFで停止"] : (/BGM: ON/.test(bs) ? (/再生中/.test(bs) ? ["ok", "再生中"] : ["ok", "ON(停止中)"]) : (/BGM: OFF/.test(bs) ? ["mute", "OFF"] : ["warn", "要確認"])));
      badge("sumBgm", g[0], g[1]);
      const vn = T("vlNow");
      if (off("voiceLevelsBlock")) badge("vlBadge", "mute", "未設定");
      else if (/^現在値/.test(vn)) badge("vlBadge", voiceOff ? "warn" : "ok", voiceOff ? "音声OFFで停止中" : "動作中");
      else badge("vlBadge", vn === "-" ? "mute" : "warn", vn === "-" ? "-" : "要確認");
      // アンケート(頭部ディスプレイ)
      const sm = T("surveyMode"), sa = T("surveyActive");
      let q = ["mute", "未設定"];
      if (!off("surveyBlock")) {
        const er = $("surveyErr");
        if (sm === "-" || sm === "") q = (er && !er.hidden) ? ["warn", "要確認"] : ["mute", "-"];
        else if (/停止中/.test(sa)) q = ["warn", "回答中(ロボット停止)"];
        else if (/アンケート表示中/.test(sm)) q = ["ok", "アンケート表示中"];
        else if (/OCRテスト/.test(sm)) q = ["warn", "OCRテスト中"];
        else if (/^顔/.test(sm)) q = ["ok", "顔"];
        else q = ["ok", "お客さん向け"];
      }
      badge("sumSurvey", q[0], q[1]); badge("surveyBadge", q[0], q[1]);
      const mw = $("ocrMockWhy");
      if (off("surveyBlock")) badge("ocrMockBadge", "mute", "未設定");
      else badge("ocrMockBadge", mw && !mw.hidden ? "mute" : "ok", mw && !mw.hidden ? "いまは使えません" : "使えます");
      // タブレットOCR
      const op = T("ocrProfile"), oe = $("ocrErr");
      let o = ["mute", "未設定"];
      if (!off("ocrBlock")) o = (op === "-" || op === "") ? ["mute", "-"] : (/届きません|不明/.test(op) || (oe && !oe.hidden) ? ["warn", "要確認"] : ["ok", "動作中: " + op]);
      badge("sumOcr", o[0], o[1]); badge("ocrBadge", o[0], o[1]);
      // 届かない/使えないカードは1行(バッジ+理由)に畳む。状態が変わった瞬間だけ開閉し、利用者が開き直した後は勝手に閉じない
      const foldSet = (id, noteId, down, note) => {
        const d = $(id); if (!d) return;
        const was = d.dataset.down === "1";
        if (down && !was) { d.dataset.down = "1"; d.open = false; } else if (!down && was) { d.dataset.down = "0"; d.open = true; }
        const n = $(noteId); if (n) n.textContent = down ? note : "";
      };
      const svErr = $("surveyErr"), svDown = !off("surveyBlock") && (sm === "-" || sm === "") && !!(svErr && !svErr.hidden);
      foldSet("surveyFold", "surveyNote", svDown, svDown ? svErr.textContent : "");
      foldSet("ocrMockBlock", "ocrMockNote", !off("surveyBlock") && !!(mw && !mw.hidden), "いまは使えません(頭部ディスプレイがOCRテストのときだけ操作できます)");
      const ocDown = !off("ocrBlock") && /届きません/.test(op);
      foldSet("ocrFold", "ocrNote", ocDown, ocDown && oe && !oe.hidden ? oe.textContent : "OCRに届きません");
      // 運転
      const mo = T("manualOverride");
      badge("driveBadge", mo === "はい" ? "warn" : (mo === "いいえ" ? "ok" : "mute"), mo === "はい" ? "手動運転中" : (mo === "いいえ" ? "自律運転" : "-"));
    }
    setInterval(tick, 1000);
    tick();
  })();
"""
