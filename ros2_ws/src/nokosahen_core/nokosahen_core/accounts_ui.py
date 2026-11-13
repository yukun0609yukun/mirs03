"""アカウント・権限まわりの画面部品(HTML/CSS/JSの文字列。純Python、ROS非依存)。

【2026-10-13追加】docs/accounts.md。

1. ROLE_UI_SNIPPET: 全てのHTMLページ(ログイン画面と観客向け表示を除く)の末尾へサーバー側で差し込む共通スクリプト(NKRole)。
   operatorでログインしているときだけ働く: 許可されていない操作の入力欄・ボタンを無効化(disabled)して
   理由を表示し(「この操作は管理者のみです」/「許可されていません」)、上部に「一時的な変更(再起動で元に戻ります)」を出す。
   adminには「一時的な上書きが N件あります」を出す。**画面の無効化は補助で、権限チェックの本体はサーバー側**
   (無効化を外して送っても403)。許可された操作の目印は `data-cap="権限名"`(祖先でも可、近い方が優先)。
   目印が無い入力欄・ボタンは(ナビゲーション等の KEEP を除き)全て無効 = デフォルト拒否。
2. ACCOUNTS_CARD_HTML / ACCOUNTS_CARD_JS: 「設定」タブの「アカウントと権限」カード(adminだけに表示)。
   アカウント一覧、operatorの許可権限チェックリスト(admin_only・未実装は選択不可、確認バー付き保存)、
   一時的な上書きの一覧と「全て破棄」。NKField(docs/dashboard_field_sync.md)に乗せ、編集中は更新で上書きしない。
"""
from __future__ import annotations

ROLE_UI_JS = r"""
(function () {
  if (window.__nkRole || location.pathname === "/login") return;
  window.__nkRole = true;
  var ADMIN_MSG = "この操作は管理者のみです";
  var NOT_GRANTED_MSG = "この操作は許可されていません(管理者に権限の付与を依頼してください)";
  // 無効化の対象外: ナビゲーション・ヘッダー・メニュー・緊急停止ボタン(止める方向は全員可。解除は下の捕捉で止める)・data-nk-keep
  var KEEP = "#topbar, #menuDrawer, #menuScrim, #estopBtn, [role=tab], [data-nk-keep]";
  var who = null;
  var banner = null;
  var scheduled = false;
  function capOf(el) {
    var h = el.closest("[data-cap]");
    return h ? (h.getAttribute("data-cap") || "").split(/\s+/).filter(Boolean) : [];
  }
  function lockInfo(el) {
    if (el.closest(KEEP)) return null;
    // data-cap-any: 列挙した権限のどれか1つでもあれば有効(保存ボタンのように複数の項目をまとめて保存する操作用)
    var any = (el.getAttribute("data-cap-any") || "").split(/\s+/).filter(Boolean);
    if (any.length) {
      var got = (who.permissions || []);
      return any.some(function (c) { return got.indexOf(c) >= 0; }) ? null : ADMIN_MSG;
    }
    var caps = capOf(el);
    if (!caps.length) return ADMIN_MSG;
    var have = who.permissions || [];
    var missing = caps.filter(function (c) { return have.indexOf(c) < 0; });
    if (!missing.length) return null;
    var locked = who.admin_only || [];
    return missing.some(function (c) { return locked.indexOf(c) >= 0; }) ? ADMIN_MSG : NOT_GRANTED_MSG;
  }
  function showBanner(text, kind) {
    if (!document.body) return;
    if (!banner) {
      banner = document.createElement("div");
      banner.id = "nkRoleBanner";
      banner.setAttribute("role", "status");
      banner.setAttribute("data-nk-keep", "");
      document.body.insertBefore(banner, document.body.firstChild);
    }
    banner.style.cssText = "padding:6px 12px;font-size:13px;text-align:center;" +
      (kind === "op" ? "background:#fff6df;color:#7a5c00;" : "background:#e8f0fe;color:#1a3d7c;");
    banner.textContent = text;
  }
  function hideBanner() { if (banner) { banner.remove(); banner = null; } }
  function render() {
    scheduled = false;
    if (!who) return;
    var n = who.ephemeral_count || 0;
    if (who.role === "admin") {
      if (n > 0) showBanner("operatorによる一時的な上書きが " + n + " 件あります(プログラムを再起動すると元に戻ります)。設定タブの「アカウントと権限」で確認・破棄できます。", "admin");
      else hideBanner();
      return;
    }
    if (who.role !== "operator") return;
    showBanner("操作アカウント(" + who.account + "): 許可された操作だけ使えます。ここでの変更は一時的です(プログラムを再起動すると元に戻ります)" +
      (n > 0 ? "。現在の一時的な変更: " + n + " 件" : "") + "。", "op");
    var cards = new Set();
    document.querySelectorAll("button, input, select, textarea").forEach(function (el) {
      if (el.tagName === "INPUT" && el.type === "hidden") return;
      var why = lockInfo(el);
      if (why) {
        if (!el.disabled || el.dataset.nkLocked !== "1") { el.disabled = true; el.dataset.nkLocked = "1"; }
        if (el.title !== why) el.title = why;
        var c = el.closest(".card");
        if (c) cards.add(c);
      } else if (el.dataset.nkLocked === "1") {
        el.disabled = false; delete el.dataset.nkLocked; el.removeAttribute("title");
      }
    });
    document.querySelectorAll(".card").forEach(function (card) {
      var ctrls = Array.prototype.filter.call(card.querySelectorAll("button, input, select, textarea"),
        function (e) { return !(e.tagName === "INPUT" && e.type === "hidden"); });
      var locked = ctrls.filter(function (e) { return e.dataset.nkLocked === "1"; });
      var note = card.querySelector(":scope > .nk-role-note");
      if (ctrls.length && locked.length === ctrls.length) {
        if (!note) {
          note = document.createElement("p");
          note.className = "nk-role-note";
          note.setAttribute("data-nk-keep", "");
          note.style.cssText = "margin:0 0 6px;font-size:13px;font-weight:700;color:#7a5c00;";
          note.textContent = "🔒 " + ADMIN_MSG + "(状況の閲覧のみ)";
          card.insertBefore(note, card.firstChild);
        }
      } else if (note) { note.remove(); }
    });
  }
  function schedule() { if (!scheduled) { scheduled = true; (window.requestAnimationFrame || setTimeout)(render); } }
  async function refresh() {
    try {
      var res = await fetch("/api/whoami");
      if (!res.ok) return;
      who = await res.json();
      schedule();
    } catch (e) { /* 取得できなければ何もしない(サーバー側の権限チェックが本体) */ }
  }
  // 【2026-11-06】手動運転(/api/teleop)は、operatorにもロールの既定で許可される(robot.manual_drive)。画面側で止めるのは、
  // whoamiの権限に robot.manual_drive が無いとき(念のため)だけ。画面は停止指令を100msごとに送り続けるので、そのときは
  // 通信せずに画面側で止める(停止指令=エラー表示なし、動かす指令=403の理由つき)。権限チェックの本体はサーバー側(/api/teleop)。
  if (typeof window.fetch === "function") {
    var rawFetch = window.fetch.bind(window);
    window.fetch = function (input, init) {
      try {
        var url = typeof input === "string" ? input : ((input && input.url) || "");
        if (who && who.role === "operator" && (who.permissions || []).indexOf("robot.manual_drive") < 0 && /\/api\/teleop(\?|$)/.test(url) && init && String(init.method || "GET").toUpperCase() === "POST") {
          var body = {};
          try { body = JSON.parse(init.body || "{}"); } catch (e) { body = {}; }
          var idle = !body.forward && !body.turn;
          return Promise.resolve(new Response(
            JSON.stringify(idle ? { ok: false, reason: "role_blocked" } : { error: "forbidden_capability", message: ADMIN_MSG, admin_only: true }),
            { status: idle ? 200 : 403, headers: { "Content-Type": "application/json" } }));
        }
      } catch (e) { /* 失敗したら通常の送信に任せる(サーバー側が403にする) */ }
      return rawFetch(input, init);
    };
  }
  // 緊急停止ボタン: かける(止める方向)は誰でも可。【2026-11-06】解除はoperatorもロール既定で可(robot.estop_release)。
  // 権限が無いときだけ、押す前に止めて理由を出す(画面の確認ダイアログは従来どおり)。サーバー側でも解除は権限を判定する。
  document.addEventListener("click", function (ev) {
    var b = ev.target && ev.target.closest ? ev.target.closest("#estopBtn") : null;
    if (b && who && who.role === "operator" && (who.permissions || []).indexOf("robot.estop_release") < 0 && b.classList.contains("engaged")) {
      ev.preventDefault(); ev.stopImmediatePropagation();
      showBanner("緊急停止の解除は管理者のみです。管理者に解除を依頼してください。", "op");
    }
  }, true);
  var obs = new MutationObserver(function () { if (who && who.role === "operator") schedule(); });
  function start() {
    obs.observe(document.body, { childList: true, subtree: true, attributes: true, attributeFilter: ["disabled"] });
    refresh();
    setInterval(function () { if (!document.hidden) refresh(); }, 15000);
  }
  if (document.body) start(); else document.addEventListener("DOMContentLoaded", start);
  window.NKRole = { refresh: refresh, who: function () { return who; } };
})();
"""

ROLE_UI_SNIPPET = ("<script>" + ROLE_UI_JS + "</script>").encode("utf-8")


def inject_role_ui(body: bytes, content_type: str, path: str = "") -> bytes:
    """HTMLページの末尾(最後の</body>の直前)へNKRoleを差し込む。ログイン画面・観客向け表示・HTML以外は何もしない。"""
    if not content_type.startswith("text/html") or path.startswith("/audience_display"):
        return body
    if b'id="loginForm"' in body:
        return body
    i = body.rfind(b"</body>")
    if i < 0:
        return body
    return body[:i] + ROLE_UI_SNIPPET + body[i:]


ACCOUNTS_CARD_HTML = r"""
    <div class="grp" id="grpAccounts" hidden>
      <div class="grp-head"><h2 class="grp-title">アカウントと権限</h2><p class="grp-note">弱いアカウント(operator)が使える操作を、1つずつ決めます(管理者のみ)。operatorの変更はプログラムを再起動すると元に戻ります。</p></div>
      <div class="grp-grid">
        <style>
          .acct-tbl{width:100%;border-collapse:collapse;font-size:13px;}.acct-tbl td:nth-child(3),.acct-tbl td:nth-child(2){white-space:nowrap;}.acct-tbl th,.acct-tbl td{border-bottom:1px solid var(--line);padding:6px 8px;text-align:left;}
          #permList details.fold{margin:6px 0;border:1px solid var(--line);border-radius:8px;}
          #permList details.fold > summary{cursor:pointer;list-style:none;padding:6px 10px;font-size:13px;font-weight:700;display:flex;align-items:center;gap:8px;}
          #permList details.fold > summary::-webkit-details-marker{display:none;}
          #permList details.fold > summary::before{content:"\25B8";color:var(--brand);}
          #permList details.fold[open] > summary::before{content:"\25BE";}
          #permList details.fold[open] > summary{border-bottom:1px solid var(--line);}
          .perm-count{margin-left:auto;font-size:13px;font-weight:400;color:var(--mute);}
          .perm-gnote{margin:0;padding:4px 10px 0;font-size:13px;color:var(--mute);}
          .perm-item{display:grid;grid-template-columns:auto minmax(0,1fr);gap:2px 10px;align-items:start;padding:6px 10px;border-top:1px solid var(--line);}
          .perm-item > input{margin-top:3px;}
          .perm-item:first-of-type{border-top:0;}
          .perm-item label{display:block;font-size:13px;color:inherit;cursor:pointer;min-width:0;}.perm-item small{display:block;color:var(--mute);font-size:13px;word-break:break-all;}
          .perm-item.locked{background:#f1f3f2;color:var(--mute);}.perm-item.locked label{cursor:not-allowed;}
          .perm-item.changed{background:var(--warn-bg);}
          .perm-risk{grid-column:2;display:block;min-width:0;padding:2px 8px;border-radius:6px;font-size:13px;font-weight:400;line-height:1.5;background:#eef2f0;color:var(--mute);white-space:normal;overflow-wrap:anywhere;}
          
        </style>
        <section class="card" id="acctListBlock">
          <div class="card-head"><h2>アカウント一覧</h2><span id="acctBadge" class="sbadge sb-mute">-</span></div>
          <p class="card-desc">名前・ロール・有効/無効・最後のログイン。パスワードとハッシュは表示しません。名前とパスワードは後で決めます(手順は docs/accounts.md)。</p>
          <div class="tblwrap"><table class="acct-tbl"><thead><tr><th>名前</th><th>ロール</th><th>状態</th><th>最後のログイン</th><th>備考</th></tr></thead><tbody id="acctRows"><tr><td colspan="5">-</td></tr></tbody></table></div>
          <p class="hint" id="acctProblems" hidden></p>
          <p class="hint" id="acctDenied" hidden></p>
        </section>
        <section class="card" id="permBlock">
          <div class="card-head"><h2>operatorに許可する操作</h2><span id="permBadge" class="sbadge sb-mute">-</span></div>
          <p class="card-desc">何もチェックしない=閲覧と緊急停止(かける)・目的地キャンセルだけ。チェックした操作だけが使えます。ロボットの根幹に関わる操作は選べません。</p>
          <div id="permList" role="group" aria-label="operatorに許可する操作"></div>
          <div class="loc-bar loc-bar-warn" id="permConfirm" hidden>
            <span id="permConfirmText"></span>
            <button id="permConfirmYes" type="button" class="primary">保存する</button>
            <button id="permConfirmNo" type="button">やめる</button>
          </div>
          <div class="btns"><button id="permSaveBtn" type="button" class="primary">変更を保存</button><button id="permRevertBtn" type="button">取り消す</button></div>
          <p class="hint" id="permHint">&nbsp;</p>
        </section>
        <section class="card" id="ephBlock">
          <details class="cf" id="ephFold">
          <summary class="card-head cf-sum"><h2>一時的な上書き(operatorの変更)</h2><span id="ephBadge" class="sbadge sb-mute">-</span></summary>
          <p class="card-desc">operatorが変えた設定は保存されず、プログラムの再起動で元に戻ります。管理者が同じ項目を保存すると、その項目の上書きは消えます。</p>
          <div class="tblwrap"><table class="acct-tbl"><thead><tr><th>項目</th><th>値</th><th>変更した人</th><th>時刻</th></tr></thead><tbody id="ephRows"><tr><td colspan="4">-</td></tr></tbody></table></div>
          <div class="loc-bar loc-bar-warn" id="ephConfirm" hidden>
            <span id="ephConfirmText">一時的な上書きを全て破棄して、保存されている値に戻します。</span>
            <button id="ephConfirmYes" type="button" class="primary">全て破棄する</button>
            <button id="ephConfirmNo" type="button">やめる</button>
          </div>
          <div class="btns"><button id="ephClearBtn" type="button">全て破棄</button></div>
          <p class="hint" id="ephHint">&nbsp;</p>
          </details>
        </section>
      </div>
    </div>
"""

ACCOUNTS_CARD_JS = r"""
  // --- 【2026-10-13追加】アカウントと権限(adminのみ表示。docs/accounts.md)。権限チェックの本体はサーバー側 ---
  (function () {
    const $ = (id) => document.getElementById(id);
    const grp = $("grpAccounts");
    if (!grp) return;
    const esc = (s) => String(s == null ? "" : s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
    const ROLE_LABEL = { admin: "管理者", operator: "operator(弱い)", viewer: "閲覧のみ", restricted: "限定(スマホ)" };
    let serverPerms = new Set();
    let draft = new Set();
    let caps = [];
    let sending = false;
    let lastEphCount = null;
    let userEdited = false;  // チェックを触ったら、サーバーの値で下書きを上書きしない(保存・取り消しで解除)
    const toc = $("tocAccounts");
    if (toc) {  // 目次の「アカウントと権限」は、カードが出ている(adminの)ときだけ表示
      const sync = () => { toc.hidden = grp.hidden; };
      new MutationObserver(sync).observe(grp, { attributes: true, attributeFilter: ["hidden"] });
      sync();
    }
    const permRoot = $("permList");
    NKField.watch(permRoot);
    const dirty = () => draft.size !== serverPerms.size || Array.from(draft).some((n) => !serverPerms.has(n));
    // 見出しごとに折りたためる。再描画(定期取得)しても、利用者が開閉した状態を保つ
    const foldOpen = { rec: true, hold: true, deferred: false, admin_only: false };
    permRoot.addEventListener("toggle", (ev) => {
      const t = ev.target;
      if (t && t.dataset && t.dataset.tier) foldOpen[t.dataset.tier] = t.open;
    }, true);
    function renderPerms() {
      const groups = [
        ["rec", "推奨(許可してよい操作)", "許可すると便利で、リスクが小さい操作です(変更は一時的 = 再起動で元に戻ります)。", (c) => c.tier === "grantable" && c.recommend === "許可"],
        ["hold", "保留(許可するか未決定)", "必要になったときに、リスクを見て決めます(変更は一時的 = 再起動で元に戻ります)。", (c) => c.tier === "grantable" && c.recommend !== "許可"],
        ["deferred", "許可できない(一時的な反映が未実装)", "", (c) => c.tier === "deferred"],
        ["admin_only", "管理者専用(ロボットの根幹・安全。コードで固定)", "", (c) => c.tier === "admin_only"],
      ];
      permRoot.innerHTML = groups.map(([tier, title, note, pick]) => {
        const items = caps.filter(pick);
        if (!items.length) return "";
        const can = tier === "rec" || tier === "hold";
        const rows = items.map((c) => {
          const on = can && draft.has(c.name);
          const changed = can && (draft.has(c.name) !== serverPerms.has(c.name));
          const id = "perm_" + esc(c.name.replace(/[^a-z0-9]/gi, "_"));
          const risk = c.risk ? '<span class="perm-risk">リスク: ' + esc(c.risk) + '</span>' : "";
          return '<div class="perm-item' + (can ? "" : " locked") + (changed ? " changed" : "") + '">' +
            '<input type="checkbox" id="' + id + '" data-perm="' + esc(c.name) + '"' + (on ? " checked" : "") + (can ? "" : " disabled") + '>' +
            '<label for="' + id + '">' + esc(c.label) + '<small>' + esc(c.name) + '</small></label>' + risk + '</div>';
        }).join("");
        const cnt = can ? "許可 " + items.filter((c) => draft.has(c.name)).length + " / " + items.length : items.length + " 件";
        return '<details class="fold" data-tier="' + tier + '"' + (foldOpen[tier] ? " open" : "") + '><summary>' + esc(title) +
          '<span class="perm-count">' + cnt + '</span></summary>' + (note ? '<p class="perm-gnote">' + esc(note) + '</p>' : "") + rows + '</details>';
      }).join("");
      $("permBadge").textContent = dirty() ? "未保存の変更あり" : ("許可 " + serverPerms.size + " 件");
      $("permBadge").className = "sbadge " + (dirty() ? "sb-warn" : "sb-mute");
    }
    permRoot.addEventListener("change", (ev) => {
      const t = ev.target;
      if (!t || !t.dataset || !t.dataset.perm) return;
      if (t.checked) draft.add(t.dataset.perm); else draft.delete(t.dataset.perm);
      userEdited = true;
      NKField.defer(permRoot, renderPerms);
      $("permBadge").textContent = dirty() ? "未保存の変更あり" : ("許可 " + serverPerms.size + " 件");
    });
    function renderAccounts(d) {
      $("acctBadge").textContent = d.accounts.length + " 件";
      $("acctRows").innerHTML = d.accounts.map((a) =>
        "<tr><td>" + esc(a.username) + "</td><td>" + esc(ROLE_LABEL[a.role] || a.role) + "</td><td>" + (a.enabled ? "有効" : "無効") +
        "</td><td>" + esc(a.last_login || "-") + "</td><td>" + esc(a.note || "") + "</td></tr>").join("");
      const pr = $("acctProblems");
      pr.hidden = !(d.problems && d.problems.length);
      pr.textContent = pr.hidden ? "" : "設定の注意: " + d.problems.join(" / ");
      const dn = $("acctDenied");
      dn.hidden = !(d.recent_denied && d.recent_denied.length);
      dn.textContent = dn.hidden ? "" : "最近の拒否(誰が・いつ・何を): " + d.recent_denied.slice(-5).map((r) => r.at + " " + r.account + " " + r.path).join(" / ");
    }
    function renderEph(d) {
      const e = d.ephemeral || { count: 0, items: [] };
      $("ephBadge").textContent = e.count + " 件";
      $("ephBadge").className = "sbadge " + (e.count ? "sb-warn" : "sb-mute");
      if (lastEphCount !== e.count) { $("ephFold").open = e.count > 0; lastEphCount = e.count; }  // 0件のときは1行に畳む(開き直した後は件数が変わるまで触らない)
      $("ephRows").innerHTML = e.items.length ? e.items.map((r) =>
        "<tr><td>" + esc(r.label) + "</td><td>" + esc(r.value) + "</td><td>" + esc(r.by) + "</td><td>" + esc(r.at) + "</td></tr>").join("")
        : '<tr><td colspan="4">一時的な上書きはありません</td></tr>';
      const extra = e.pending_restore ? "(前回の起動で戻せなかった項目が " + e.pending_restore + " 件あり、自動で再試行中)" : "";
      if (!$("ephHint").dataset.msg) $("ephHint").textContent = extra || " ";
    }
    async function load() {
      let d = null;
      try {
        const res = await fetch("/api/accounts");
        if (!res.ok) { grp.hidden = true; return; }
        d = await res.json();
      } catch (e) { return; }
      if (!d || !d.ok) { grp.hidden = true; return; }
      grp.hidden = false;
      caps = d.capabilities || [];
      serverPerms = new Set(d.operator_permissions || []);
      if (!userEdited && !sending) draft = new Set(serverPerms);
      renderAccounts(d);
      renderEph(d);
      NKField.defer(permRoot, renderPerms);
    }
    function confirmText() {
      const add = Array.from(draft).filter((n) => !serverPerms.has(n));
      const del = Array.from(serverPerms).filter((n) => !draft.has(n));
      const label = (n) => (caps.find((c) => c.name === n) || {}).label || n;
      return "operatorの権限を変更します。" + (add.length ? " 許可する: " + add.map(label).join("、") + "。" : "") +
        (del.length ? " 取り消す: " + del.map(label).join("、") + "。" : "") + " よろしいですか?";
    }
    $("permSaveBtn").addEventListener("click", () => {
      if (!dirty()) { $("permHint").textContent = "変更はありません。"; return; }
      $("permConfirmText").textContent = confirmText();
      $("permConfirm").hidden = false;
    });
    $("permConfirmNo").addEventListener("click", () => { $("permConfirm").hidden = true; });
    $("permRevertBtn").addEventListener("click", () => { draft = new Set(serverPerms); userEdited = false; $("permConfirm").hidden = true; NKField.defer(permRoot, renderPerms); $("permHint").textContent = " "; });
    $("permConfirmYes").addEventListener("click", async () => {
      $("permConfirm").hidden = true;
      sending = true;
      try {
        const res = await fetch("/api/accounts/permissions", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ operator_permissions: Array.from(draft).sort() }) });
        const d = await res.json().catch(() => ({}));
        $("permHint").textContent = res.ok ? "保存しました(すぐ有効になります)。" : ("保存できませんでした: " + (d.message || d.error || res.status));
        if (res.ok) { serverPerms = new Set(d.operator_permissions || []); draft = new Set(serverPerms); userEdited = false; }
      } catch (e) {
        $("permHint").textContent = "保存できませんでした(通信エラー)";
      } finally { sending = false; }
      await load();
    });
    $("ephClearBtn").addEventListener("click", () => { $("ephConfirm").hidden = false; });
    $("ephConfirmNo").addEventListener("click", () => { $("ephConfirm").hidden = true; });
    $("ephConfirmYes").addEventListener("click", async () => {
      $("ephConfirm").hidden = true;
      try {
        const res = await fetch("/api/ephemeral/clear", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" });
        const d = await res.json().catch(() => ({}));
        const ng = (d.restore_failed || []).length + (d.remote_reset_failed || []).length;
        $("ephHint").dataset.msg = "1";
        $("ephHint").textContent = res.ok ? (d.ok ? "全て破棄しました。" : "破棄しました。ただし " + ng + " 件は相手に届かず、自動で再試行します。") : "破棄できませんでした: " + (d.message || d.error || res.status);
        setTimeout(() => { delete $("ephHint").dataset.msg; }, 8000);
      } catch (e) {
        $("ephHint").textContent = "破棄できませんでした(通信エラー)";
      }
      await load();
    });
    load();
    NKField.poll(load, 10000);
  })();
"""
