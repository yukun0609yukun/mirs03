"""呼びかけ音声設定ページ(/voice_settings)のサーバー側ロジック(純Python、ROS非依存)。

【2026-10-06追加】2台の設定をまとめて扱う。
- 1台目(このダッシュボードの機体): 発話間隔 `announce_min_interval_seconds`
    -> `announce_settings.AnnounceSettingsStore`(~/.nokosahen_announce_settings.json)。
       behavior_directorがmtimeで自動的に再読み込みする。
- 2台目(voice_bridge、Ollama+VOICEVOX): プロンプト・メニュー名/残数を使うか・temperature等と実績
    -> `voice_settings_proxy.VoiceSettingsProxy`経由(ブラウザは2台目を直接呼ばない)。
2台目に接続できなくても、1台目の発話間隔の編集・保存はできる。
ロボットを動かす機能は無い(設定の読み書きと、文面生成のお試し・試し発話だけ)。
"""

from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

from . import announce_settings as ann
from .voice_settings_proxy import SPEAK_TEXT_MAX, VoiceSettingsProxy, validate_voice_settings

# 現在のメニュー状態が無い(食数未入力など)ときにお試しで使う見本。実際のメニューではないことを画面に出す。
SAMPLE_MENU_STATE = {
    "phase": "induction",
    "promote_item": "Bセット",
    "items": [
        {"name": "Aセット", "initial": 20, "remaining": 12},
        {"name": "Bセット", "initial": 20, "remaining": 17},
        {"name": "カレー", "initial": 15, "remaining": 6},
    ],
}


def build_preview_context(menu_state: Any, force_sample: bool = False) -> Tuple[Dict[str, Any], str]:
    """お試し生成用の context(voice_bridgeの/speakに送るのと同じ形)を作る。(context, 出どころ)。

    出どころ: "live"(いまのメニュー状態) / "sample"(見本)。behavior_director._build_announce_contextと
    同じ規則: 食数が入力済みの品目名をメニュー一覧にし、induction(促進品目あり)のときだけ残数状況を付ける。
    """
    source = "live"
    state = menu_state if isinstance(menu_state, dict) else {}
    items = [it for it in (state.get("items") or []) if isinstance(it, dict) and str(it.get("name", "")).strip()]
    if force_sample or not items:
        state, source = SAMPLE_MENU_STATE, "sample"
        items = list(SAMPLE_MENU_STATE["items"])
    phase = state.get("phase", "initial")
    promote = state.get("promote_item")
    context: Dict[str, Any] = {
        "phase": phase,
        "menu_items": [{"name": str(it["name"]).strip(), "merit": str(it.get("merit", "") or "")} for it in items],
        "fallback_text": "本日のメニューをご確認ください。お好きなものをどうぞ。",
    }
    if phase == "induction" and promote:
        context["promote_item"] = promote
        context["remaining_status"] = [
            {"name": it.get("name"), "remaining": it.get("remaining"), "initial": it.get("initial")}
            for it in items
        ]
    return context, source


class VoiceSettingsController:
    def __init__(self, announce_store: ann.AnnounceSettingsStore, proxy: VoiceSettingsProxy) -> None:
        self.announce = announce_store
        self.proxy = proxy

    # --- 読み取り ---

    def payload(self) -> Dict[str, Any]:
        """ページの初期表示用。2台目に繋がらなくても1台目の情報は返す(voice.ok=false)。"""
        out: Dict[str, Any] = {
            "ok": True,
            "configured": self.proxy.configured,
            "announce": self.announce.read(),
            "voice": None,
        }
        if self.proxy.configured:
            res = self.proxy.get_settings()
            out["voice"] = (
                {"ok": True, **res["data"]} if res["ok"]
                else {"ok": False, "kind": res.get("kind"), "error": res["error"]}
            )
        return out

    def metrics(self, limit: int = 50, row_id: Optional[int] = None) -> Tuple[int, Dict[str, Any]]:
        res = self.proxy.get_metrics(limit, row_id)
        if res["ok"]:
            return 200, {"ok": True, "configured": True, **res["data"]}
        if not res.get("configured", True):
            return 200, {"ok": False, "configured": False, "error": res["error"]}
        return 200, {"ok": False, "configured": True, "kind": res.get("kind"), "error": res["error"]}

    # --- 書き込み ---

    def save(self, body: Any, persist: bool = True) -> Tuple[int, Dict[str, Any]]:
        """{"announce": {"announce_min_interval_seconds": N}, "voice": {...部分更新...}}。

        先に両方を検証し、1つでも不正なら何も書かない(400)。その後、1台目のファイルと2台目への送信を
        それぞれ行い、結果を個別に返す(2台目が落ちていても1台目の保存は有効)。
        persist=Falseは弱いアカウント(operator)の一時的な変更。2台目へはpersist:falseで送る(メモリのみ)。
        1台目の発話間隔はファイルへ書く(directorがファイルを読むため)が、直前の値の記録と起動時の復元は
        呼び出し側(ダッシュボード、ephemeral_overrides)が行う。docs/accounts.md
        """
        if not isinstance(body, dict) or not ("announce" in body or "voice" in body):
            return 400, {"ok": False, "errors": ["announce か voice のどちらかを指定してください"]}
        errors = []
        interval = None
        if "announce" in body:
            a = body["announce"]
            if not isinstance(a, dict) or ann.KEY not in a:
                errors.append(f"announceには{ann.KEY}を指定してください")
            else:
                interval, err = ann.parse_interval(a[ann.KEY])
                if err:
                    errors.append(err)
        voice_payload: Dict[str, Any] = {}
        if "voice" in body:
            if not self.proxy.configured:
                errors.append("音声設定の中継先(voice_settings_url)が未設定のため、2台目の設定は保存できません")
            voice_payload, verrs = validate_voice_settings(body["voice"])
            errors.extend(verrs)
            if not verrs and not voice_payload:
                errors.append("voiceに変更する項目がありません")
        if errors:
            return 400, {"ok": False, "errors": errors}

        result: Dict[str, Any] = {"ok": True}
        status = 200
        messages = []
        if interval is not None:
            ares = self.announce.save_interval(interval)
            result["announce"] = ares
            if ares["ok"]:
                messages.append("1台目: 発話間隔を保存しました(behavior_directorが1秒以内に自動で反映します)")
            else:
                result["ok"] = False
                status = 500
                messages.append("1台目: 発話間隔を保存できませんでした")
        if voice_payload:
            vres = self.proxy.save_settings(voice_payload, persist=persist)
            if vres["ok"]:
                result["voice"] = {"ok": True}
                messages.append("2台目: 音声設定を保存しました(次の発話から反映します)")
            else:
                result["voice"] = {"ok": False, "kind": vres.get("kind"), "errors": vres.get("errors", []), "error": vres["error"]}
                result["ok"] = False
                if status == 200:
                    status = 400 if vres.get("kind") == "http" else 502
                messages.append("2台目: " + vres["error"])
        result["message"] = " / ".join(messages)
        return status, result

    def preview(self, body: Any, menu_state: Any) -> Tuple[int, Dict[str, Any]]:
        """ドライラン(話さない・履歴に残さない)。bodyの"settings"は未保存の設定(部分でよい)。"""
        if not isinstance(body, dict):
            return 400, {"ok": False, "errors": ["不正なリクエストです"]}
        settings, errors = validate_voice_settings(body.get("settings") or {})
        if errors:
            return 400, {"ok": False, "errors": errors}
        if "force_sample" in body and not isinstance(body["force_sample"], bool):
            return 400, {"ok": False, "errors": ["force_sampleは true/false で指定してください"]}
        context, source = build_preview_context(menu_state, bool(body.get("force_sample")))
        res = self.proxy.preview(context, settings)
        if not res["ok"]:
            if not res.get("configured", True):
                return 200, {"ok": False, "configured": False, "error": res["error"]}
            return (400 if res.get("kind") == "http" else 502), {
                "ok": False, "kind": res.get("kind"), "error": res["error"], "errors": res.get("errors", [])}
        return 200, {**res["data"], "context_source": source,
                     "context_menu": [m["name"] for m in context["menu_items"]],
                     "context_phase": context["phase"]}

    def speak(self, body: Any) -> Tuple[int, Dict[str, Any]]:
        """生成済みの文面をそのまま話す(試し発話)。呼び出し側(ルート)がTL(書込み)権限を確認する。"""
        text = body.get("text") if isinstance(body, dict) else None
        if not isinstance(text, str) or not text.strip():
            return 400, {"ok": False, "errors": ["話す文面(text)が必要です"]}
        text = text.strip()
        if len(text) > SPEAK_TEXT_MAX:
            return 400, {"ok": False, "errors": [f"文面は{SPEAK_TEXT_MAX}文字以内にしてください"]}
        res = self.proxy.speak_text(text)
        if res["ok"]:
            return 200, {"ok": True, "message": "話す依頼をキューに入れました(合成と再生に数秒かかります)"}
        if not res.get("configured", True):
            return 200, {"ok": False, "configured": False, "error": res["error"]}
        return (400 if res.get("kind") == "http" else 502), {"ok": False, "kind": res.get("kind"), "error": res["error"]}
