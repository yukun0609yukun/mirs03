"""「設定」タブ > メンテナンス の「プログラム終了(全停止)」カード(HTML/JSの文字列。純Python、ROS非依存)。

【2026-10-13追加】docs/system_shutdown.md。権限チェックの本体はサーバー側(POST /api/system/shutdown は admin、または system.shutdown を許可されたoperator。
「未保存の地図を捨てて終了」だけadmin専用)。画面は権限の無いアカウントではカード全体を無効化して理由を出す(guestは自前で無効化)。
確認文字列「シャットダウン」を手で打つまで「実行」は押せない → 確認バー → 実行後は共通ヘルパー NKField.freeze() で
全ての定期取得・通信を止め(接続エラーを大量に出さない)、カウントダウンと「接続が切れます」を表示する。
"""
from __future__ import annotations

CONFIRM_TEXT = "シャットダウン"  # system_shutdown.CONFIRM_TEXT と同じ(テストで一致を確認)

SHUTDOWN_CARD_HTML = r"""
        <section class="card" id="shutdownCard" data-cap="system.shutdown" style="border:1px solid #e3b4a8;">
          <div class="card-head"><h2 style="color:#993c1d;">プログラム終了(全停止)</h2><span id="shBadge" class="sbadge sb-warn">権限が必要</span></div>
          <p class="card-desc">1台目のロボット関連プログラムを<b>全部止めます</b>。数秒後にこの管理画面にも入れなくなります。下の「OSも停止して電源を落とす」にチェックがあると(既定)、<b>ラズパイ・2台目・1台目のOSも止まります</b>(外すと、OSは動いたままです)。</p>
          <ul class="hint" style="margin:4px 0 8px 18px;padding:0;">
            <li><b>止まるもの</b>: ロボット制御(ROS一式・Nav2・LiDAR等)、この管理画面、タブレットOCR/Coral(選択時)、2台目の音声(選択時のみ)</li>
            <li><b>「OSも停止して電源を落とす」にチェックがあるとき</b>: ラズパイ・2台目・1台目のOSも止まり、電源を抜いてよい状態になります(Wi-Fi・SSHも使えなくなります)</li>
            <li><b>チェックを外したとき(OSは動いたまま)</b>: Jetsonの電源・Wi-Fi・SSH、ラズパイの画面、2台目(Ollama/VOICEVOX。音声だけ選択時に停止)は止まりません</li>
            <li><b>再起動の方法</b>: OSも停止したときは、<b>一式の電源を入れ直す</b>と自動で手動モードが起動します(登録済みの場合。docs/autostart.md)。OSが動いたままなら、1台目にSSHして <code>bash ~/mirs2603_nokosahen/scripts/start_all.sh manual</code></li>
            <li>止める前に、ロボットのモーターを必ず止めます(ソフト緊急停止)。経過は 1台目の <code>~/.nokosahen_shutdown.log</code> に残ります</li>
          </ul>
          <div class="btns" style="flex-direction:column;align-items:flex-start;gap:4px;">
            <label><input type="checkbox" id="shOcr" checked> OCR(タブレットOCR・Coral)も止める</label>
            <label><input type="checkbox" id="shVoice"> 2台目の音声(voice_bridge/BGM)も止める</label>
            <label><input type="checkbox" id="shPower" checked> OSも停止して電源を落とす(ラズパイ・2台目・1台目。再起動は、一式の電源を入れ直す)</label>
            <label id="shDiscardRow" hidden><input type="checkbox" id="shDiscard"> 未保存の地図を捨てて終了</label>
          </div>
          <p id="shMapWarn" class="hint" hidden style="color:#993c1d;font-weight:700;"></p>
          <p id="shNavWarn" class="hint" hidden style="color:#993c1d;font-weight:700;">いま走行中です(目的地へ移動中)。実行すると、その場で止まります。</p>
          <div class="btns">
            <label>確認: 「シャットダウン」と入力 <input type="text" id="shConfirm" autocomplete="off" placeholder="シャットダウン" style="width:10em;"></label>
            <button id="shRunBtn" type="button" disabled style="background:#b3402a;color:#fff;border-color:#8f2f1d;">プログラム終了を実行</button>
          </div>
          <div class="loc-bar loc-bar-warn" id="shBar" hidden>
            <span id="shBarText">本当に止めますか?</span>
            <button id="shYes" type="button" class="primary">止める</button>
            <button id="shNo" type="button">やめる</button>
          </div>
          <p id="shStatus" class="hint" role="status">&nbsp;</p>
          <p id="shLock" class="hint" hidden style="color:#7a5c00;font-weight:700;"></p>
        </section>
"""

SHUTDOWN_CARD_JS = r"""
  // --- 【2026-10-13追加】プログラム終了(全停止)カード。docs/system_shutdown.md。権限チェックの本体はサーバー側 ---
  (function () {
    const $ = (id) => document.getElementById(id);
    const card = $("shutdownCard");
    if (!card) return;
    const CONFIRM = "シャットダウン";
    NKField.guard(["shConfirm", "shOcr", "shVoice", "shPower", "shDiscard"]);
    let srv = null;       // サーバーの状態(未保存の地図・走行中・二重実行)
    let isAdmin = false;  // 実行を許可されたアカウントか(管理者、または system.shutdown を許可されたoperator)
    let canDiscard = false; // 未保存の地図を捨てて終了できるのは管理者だけ
    let sending = false;
    let finished = false; // 停止処理を開始した後(以後は何も送らない)
    function refresh() {
      const typed = $("shConfirm").value === CONFIRM;
      const needDiscard = !!(srv && srv.map_unsaved);
      $("shDiscardRow").hidden = !(needDiscard && canDiscard);
      if (!needDiscard) $("shDiscard").checked = false;  // 行が隠れたら、前回のチェックを持ち越さない(後でまた未保存になっても勝手に捨てる選択にならない)
      const blockedByMap = needDiscard && !(canDiscard && $("shDiscard").checked);
      const mw = $("shMapWarn");
      mw.hidden = !blockedByMap;
      mw.textContent = blockedByMap ? ((srv.map_reason || "地図が未保存です。先に保存してください") + (canDiscard
        ? "(捨てて終了するなら下の「未保存の地図を捨てて終了」にチェック)"
        : "(管理者に地図を保存してもらってから終了してください)")) : "";
      $("shNavWarn").hidden = !(srv && srv.nav_active);
      const blocked = !isAdmin || finished || sending || (srv && (srv.in_progress || srv.enabled === false || srv.script_ok === false));
      $("shRunBtn").disabled = !(typed && !blockedByMap && !blocked);
      if (srv && srv.enabled === false) $("shStatus").textContent = "この機能は無効です(設定 system_shutdown_enabled=false)。";
      else if (srv && srv.script_ok === false && isAdmin) $("shStatus").textContent = "停止スクリプトが見つかりません(設定 shutdown_script を確認してください)。";
    }
    function lockAll(reason) {
      isAdmin = false;
      card.querySelectorAll("input,button").forEach((e) => { e.disabled = true; });
      $("shLock").hidden = false;
      $("shLock").textContent = "🔒 " + reason;
      $("shBadge").textContent = "権限なし";
    }
    async function load() {
      if (finished) return;
      const t = NKField.ticket("shutdown");
      try {
        const res = await fetch("/api/system/shutdown/state");
        if (!res.ok) return;
        const d = await res.json();
        if (!NKField.fresh(t)) return;
        srv = d;
        refresh();
      } catch (e) { /* 次回に再試行 */ }
    }
    async function init() {
      try {
        const res = await fetch("/api/whoami");
        const who = res.ok ? await res.json() : null;
        const okRole = who && (who.role === "admin" || (who.role === "operator" && (who.permissions || []).indexOf("system.shutdown") >= 0));
        if (!okRole) { lockAll("この操作は許可されていません(管理者に権限の付与を依頼してください)"); return; }
        canDiscard = who.role === "admin";
        $("shBadge").textContent = who.role === "admin" ? "管理者" : "許可済み";
      } catch (e) { return; }
      isAdmin = true;
      card.querySelectorAll("input,button").forEach((e) => { if (e.dataset.nkLocked !== "1") e.disabled = e.id === "shRunBtn"; });
      await load();
      refresh();
      NKField.poll(load, 5000);
    }
    $("shConfirm").addEventListener("input", refresh);
    $("shDiscard").addEventListener("change", refresh);
    // 確認バーを出した時点の選択を控えておき、「止める」ではその値を送る(バーを見たあとにチェックを変えても、警告と送る内容がずれない)。
    // バーを出している間は、入力欄を操作できないようにする。
    let pending = null;
    const OPT_IDS = ["shConfirm", "shOcr", "shVoice", "shPower", "shDiscard"];
    function lockOptions(lock) { OPT_IDS.forEach((id) => { const e = $(id); if (e && e.dataset.nkLocked !== "1") e.disabled = lock; }); }
    $("shRunBtn").addEventListener("click", () => {
      if ($("shRunBtn").disabled) return;
      pending = { ocr: $("shOcr").checked, voice: $("shVoice").checked, power: $("shPower").checked,
                  discard: canDiscard && $("shDiscard").checked, confirm: $("shConfirm").value };
      const parts = ["本当に止めますか? ロボットのモーターを止め、プログラムを全部止めます。数秒後にこの画面は開けなくなります。"];
      if (srv && srv.nav_active) parts.push("【走行中です】その場で止まります。");
      if (pending.discard && srv && srv.map_unsaved) parts.push("【未保存の地図は捨てられます】");
      if (pending.voice) parts.push("2台目の音声も止めます。");
      if (pending.power) parts.push("【ラズパイ・2台目・1台目の電源が落ちます】再起動は、一式の電源を入れ直してください。");
      else parts.push("OSは動いたままです。再起動はSSHで行います。");
      $("shBarText").textContent = parts.join(" ");
      lockOptions(true);
      $("shBar").hidden = false;
    });
    $("shNo").addEventListener("click", () => { pending = null; $("shBar").hidden = true; lockOptions(false); $("shRunBtn").disabled = true; refresh(); });
    function banner(text) {
      let b = $("shutdownBanner");
      if (!b) {
        b = document.createElement("div");
        b.id = "shutdownBanner";
        b.setAttribute("role", "alert");
        b.style.cssText = "position:fixed;top:0;left:0;right:0;z-index:1900;background:#b3402a;color:#fff;padding:10px 12px;font-size:14px;font-weight:700;text-align:center;";
        document.body.appendChild(b);
      }
      b.textContent = text;
    }
    $("shYes").addEventListener("click", async () => {
      const opt = pending;
      if (!opt) return;  // バーを出した時点の選択が無ければ送らない
      pending = null;
      $("shBar").hidden = true;
      sending = true;
      const powered = opt.power;  // バーを出した時点の選択
      lockOptions(false);
      refresh();
      let res = null, d = {};
      try {
        res = await fetch("/api/system/shutdown", { method: "POST", headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ confirm: opt.confirm, include_ocr: opt.ocr, include_voice: opt.voice, include_poweroff: opt.power, discard_map: opt.discard }) });
        d = await res.json().catch(() => ({}));
      } catch (e) {
        // 応答が届く前に管理画面が止まった可能性がある(サーバーは受理済みで、停止処理の最中かもしれない)
        sending = false; refresh();
        $("shStatus").textContent = "通信エラー: 停止処理が始まった可能性があります。数秒後に画面が開けなければ、停止しています(経過は ~/.nokosahen_shutdown.log)。開けたままなら、止まっていないので、もう一度実行できます。";
        return;
      }
      sending = false;
      if (!res.ok || !d.ok) {
        $("shStatus").textContent = "止めませんでした: " + (d.message || d.error || res.status);
        refresh();
        return;
      }
      finished = true;
      NKField.freeze();  // 以後、定期取得と通信を全部止める(接続エラーを大量に出さない)
      card.querySelectorAll("input,button").forEach((e) => { e.disabled = true; });
      let left = Math.max(1, d.will_stop_in_sec || 3);
      const tick = () => {
        const msg = "停止処理を開始しました。数秒後にこの画面は開けなくなります。" + (left > 0 ? " 接続が切れます(あと約" + left + "秒)" : (powered ? " 接続が切れました。電源が落ちたら、一式の電源を入れ直すと再起動します。" : " 接続が切れました。再起動はSSHで行ってください。"));
        $("shStatus").textContent = msg;
        banner(msg);
        if (left > 0) { left -= 1; setTimeout(tick, 1000); }
      };
      tick();
    });
    refresh();
    init();
  })();
"""
