"""自律動作の設定ページ3つ(かんたん時間割 / ビヘイビアツリー / 時間帯・状況ルール)に共通の
「どのページを使うか」の説明バナーと「機体の時計」表示。

【2026-10-23追加】3ページの違いが分かりにくいという声への対応。ページ本体には触れず、
プレースホルダへ差し込むだけにする(既存のid・JSに影響させない)。
- 時計は GET /api/robot_clock を1秒ごとに取得する(取れなければ「時計を取得できません」)。
- JSは即時関数の中に閉じ込め、グローバルを増やさない。要素idは pg 始まり。
"""

# UIが叩くAPIパス(テストでHTMLに含まれることを確認する)
ROBOT_CLOCK_API = "/api/robot_clock"

PAGE_SCHEDULE = "schedule"
PAGE_TREE = "tree"
PAGE_RULES = "rules"

_PAGES = [
    (PAGE_SCHEDULE, "/schedule", "かんたん時間割",
     "かんたん。「何時にどこへ行って、どの状態になるか」を表で並べるだけ。まずはこれを使います。"),
    (PAGE_TREE, "/behavior_tree", "ビヘイビアツリー",
     "詳細。条件分岐(混雑・人物・残数など)を木で組む上級者向け。"),
    (PAGE_RULES, "/behavior_rules", "時間帯・状況ルール",
     "時間帯や状況の条件だけで「方針」(待機・通常・接近…)を選ぶ表。"),
]

_THIS_PAGE_NOTE = {
    PAGE_SCHEDULE: "このページ: 時刻ごとの行き先と状態を表で指定し、「このスケジュールで動け」で実行します。",
    PAGE_TREE: "このページ: 条件分岐を木で細かく組みます。時刻と行き先だけ決めたいなら「かんたん時間割」の方が簡単です。",
    PAGE_RULES: "このページ: 時間帯・混雑などの条件に合う方針を選ぶ表です。行き先(どのピンへ行くか)は決めません。",
}

PAGE_GUIDE_CSS = r"""
  .pg-guide { background:#fff; border:1px solid #dce4df; border-radius:8px; padding:10px 14px; font-size:13px; color:#16211d; }
  .pg-guide h2 { margin:0 0 6px; font-size:14px; }
  .pg-guide .pg-this { margin:0 0 8px; font-weight:700; color:#124739; }
  .pg-cards { display:grid; grid-template-columns:repeat(auto-fit,minmax(220px,1fr)); gap:8px; }
  .pg-card { display:block; text-decoration:none; color:#16211d; border:1px solid #dce4df; border-radius:8px; padding:8px 10px; background:#fbfcfb; }
  .pg-card b { display:block; font-size:13px; color:#124739; }
  .pg-card span { display:block; font-size:13px; color:#33463e; margin-top:2px; }
  .pg-card.on { border:2px solid #176b56; background:#e8f6ef; }
  .pg-clock { margin-top:10px; display:flex; flex-wrap:wrap; gap:4px 14px; align-items:baseline; background:#eef3f0; border-radius:8px; padding:6px 10px; }
  .pg-clock .pg-label { font-size:13px; color:#53635c; }
  .pg-clock .pg-time { font-size:18px; font-weight:700; font-variant-numeric:tabular-nums; }
  .pg-clock .pg-diff { font-size:13px; color:#53635c; }
  .pg-clock.big .pg-time { font-size:34px; line-height:1.1; }
  .pg-clock.big .pg-date { font-size:15px; font-weight:700; }
  .pg-clock.bad { background:#faece7; }
  .pg-clock.bad .pg-time { color:#993c1d; }
  .pg-warn { margin-top:6px; padding:6px 10px; border-radius:6px; background:#a32d2d; color:#fff; font-size:13px; font-weight:700; }
  .pg-warn.soft { background:#fff3cd; color:#7a5b00; font-weight:400; }
"""

PAGE_GUIDE_JS = r"""
  // ---- 機体の時計(page_guide.py)。GET /api/robot_clock を1秒ごとに取得する
  (function () {
    const timeEl = document.getElementById("pgClockTime");
    if (!timeEl) return;
    const dateEl = document.getElementById("pgClockDate");
    const diffEl = document.getElementById("pgClockDiff");
    const boxEl = document.getElementById("pgClock");
    const warnEl = document.getElementById("pgClockWarn");
    let busy = false;
    function fail(msg) {
      timeEl.textContent = "--:--:--";
      if (dateEl) dateEl.textContent = "";
      if (diffEl) diffEl.textContent = msg;
      boxEl.classList.remove("bad");
      warnEl.style.display = "none";
    }
    async function tick() {
      if (busy || document.hidden) return;
      busy = true;
      try {
        const res = await fetch("__ROBOT_CLOCK_API__", { cache: "no-store" });
        if (!res.ok) throw new Error("HTTP " + res.status);
        const d = await res.json();
        const iso = typeof d.iso === "string" ? d.iso : "";
        const m = iso.match(/^(\d{4}-\d{2}-\d{2})T(\d{2}:\d{2}:\d{2})/);
        if (!m) throw new Error("bad iso");
        timeEl.textContent = m[2];
        if (dateEl) dateEl.textContent = m[1] + (d.tz ? " (" + d.tz + ")" : "");
        const ep = Number(d.epoch);
        if (isFinite(ep)) {
          const diff = Math.round(ep - Date.now() / 1000);
          diffEl.textContent = Math.abs(diff) < 2
            ? "このPCの時刻とほぼ同じ"
            : "このPCの時刻より " + Math.abs(diff) + " 秒" + (diff > 0 ? "進んでいます" : "遅れています");
        } else diffEl.textContent = "";
        const suspect = d.clock_suspect === true;
        boxEl.classList.toggle("bad", suspect);
        warnEl.style.display = suspect ? "" : "none";
      } catch (e) {
        fail("時計を取得できません");
      }
      busy = false;
    }
    tick();
    setInterval(tick, 1000);
  })();
"""


def page_guide_html(current: str, big_clock: bool = False) -> str:
    """説明バナー+機体の時計のHTML(`<main>`の先頭に入れる)。currentは PAGE_* のどれか。"""
    cards = []
    for key, href, title, desc in _PAGES:
        on = " on" if key == current else ""
        cards.append(
            '<a class="pg-card%s" href="%s"><b>%s</b><span>%s</span></a>' % (on, href, title, desc)
        )
    note = _THIS_PAGE_NOTE.get(current, "")
    clock_cls = "pg-clock big" if big_clock else "pg-clock"
    return (
        '<section class="pg-guide" id="pgGuide">'
        "<h2>自律動作の設定は3ページあります。どれを使うか</h2>"
        '<p class="pg-this">%s</p>'
        '<div class="pg-cards">%s</div>'
        '<div class="%s" id="pgClock">'
        '<span class="pg-label">機体の時計</span>'
        '<span class="pg-time" id="pgClockTime">--:--:--</span>'
        '<span class="pg-date" id="pgClockDate"></span>'
        '<span class="pg-diff" id="pgClockDiff">取得中...</span>'
        "</div>"
        '<div class="pg-warn" id="pgClockWarn" style="display:none;">'
        "機体の時計が古い日付のままの可能性があります(電源投入直後や時刻の未同期)。"
        "時刻で切り替わる動作は、時計が正しくなるまで当てになりません。"
        "</div>"
        "</section>"
    ) % (note, "".join(cards), clock_cls)


def page_guide_js() -> str:
    return PAGE_GUIDE_JS.replace("__ROBOT_CLOCK_API__", ROBOT_CLOCK_API)


def inject_page_guide(html: str, current: str, big_clock: bool = False,
                      marker_html: str = "__PAGE_GUIDE_HTML__") -> str:
    """ページ文字列のプレースホルダ(__PAGE_GUIDE_CSS__/__PAGE_GUIDE_HTML__/__PAGE_GUIDE_JS__)を置換する。"""
    return (
        html.replace("__PAGE_GUIDE_CSS__", PAGE_GUIDE_CSS)
        .replace(marker_html, page_guide_html(current, big_clock))
        .replace("__PAGE_GUIDE_JS__", page_guide_js())
    )

