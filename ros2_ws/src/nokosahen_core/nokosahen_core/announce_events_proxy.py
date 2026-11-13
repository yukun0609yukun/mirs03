"""状況アナウンス設定(`/announce_settings`、`/api/announce_events*`)のサーバー側ロジック(純Python、ROS非依存)。

【2026-10-13追加】docs/announcements.md。voice_levels_proxy.py / voice_master_proxy.pyと同じ流儀(標準ライブラリのurllibのみ)。
- イベント定義表(言う内容・ON/OFF・モード・間隔・音量など): 1台目のファイル `~/.nokosahen_announce_events.json`
  (`announce_events.AnnounceEventsStore`)。announce_nodeが1秒以内に自動で読み直す(再起動不要)。
- アナウンス全体の基準音量 `announce_volume`(0〜5): 2台目voice_bridgeの `/voice_levels`(発話・BGMの音量と同じ保存先)。
- 「試しに鳴らす」: 2台目の `POST /announce`(その場で1回。合成してキャッシュも温まる)。
- 状態(キャッシュ済みか・最後に鳴った時刻・ループが動いているか): 2台目の `GET /announce_state`。
ブラウザは2台目を直接呼ばず、ダッシュボードのログイン(認可)の内側から中継する。2台目に届かなくても1台目の設定は編集・保存できる。
ロボットを動かす機能は無い(設定の読み書きと試し再生だけ)。
"""

from __future__ import annotations

import json
import math
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Dict, Optional, Tuple

from . import announce_events as ae

TIMEOUT_S = 3.0
VOLUME_MAX = 5.0
NOT_CONFIGURED = "2台目(voice_bridge)の宛先が未設定です(voice_levels_urlが空です)"


class AnnounceBridgeProxy:
    def __init__(self, base_url: str = "", timeout_s: float = TIMEOUT_S) -> None:
        self.base_url = (base_url or "").strip().rstrip("/")
        self.timeout_s = float(timeout_s)

    @property
    def configured(self) -> bool:
        return bool(self.base_url)

    def _request(self, method: str, path: str, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        req = urllib.request.Request(
            f"{self.base_url}{path}",
            data=None if payload is None else json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={"Content-Type": "application/json"} if payload is not None else {},
            method=method,
        )
        with urllib.request.urlopen(req, timeout=self.timeout_s) as res:
            body = json.loads(res.read().decode("utf-8"))
        if not isinstance(body, dict):
            raise ValueError("応答の形式が想定と違います")
        return body

    def _call(self, method: str, path: str, payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        if not self.configured:
            return {"ok": False, "configured": False, "error": NOT_CONFIGURED}
        try:
            return {"ok": True, "configured": True, "data": self._request(method, path, payload)}
        except urllib.error.HTTPError as exc:
            detail = ""
            try:
                body = json.loads(exc.read().decode("utf-8"))
                detail = str(body.get("message") or body.get("error") or "")[:200] if isinstance(body, dict) else ""
            except Exception:  # noqa: BLE001
                detail = ""
            finally:
                exc.close()
            if exc.code == 404:
                return {"ok": False, "configured": True, "error": "2台目のvoice_bridgeが状況アナウンスに未対応です(更新して再起動してください)"}
            if exc.code == 403:
                return {"ok": False, "configured": True, "error": "2台目が1台目からの接続を許可していません(VOICE_BRIDGE_ALLOW)"}
            return {"ok": False, "configured": True, "error": detail or f"voice_bridgeがエラーを返しました(HTTP {exc.code})", "status": exc.code}
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "configured": True, "error": f"2台目に接続できません: {exc}"}

    def get_state(self, texts: Optional[list] = None) -> Dict[str, Any]:
        """texts: キャッシュ済みか知りたい文言(2台目の "cached" に入る)。"""
        q = "".join(("&" if i else "?") + "text=" + urllib.parse.quote(t, safe="") for i, t in enumerate((texts or [])[:16]))
        return self._call("GET", "/announce_state" + q)

    def play_once(self, event_id: str, text: str, volume: float) -> Dict[str, Any]:
        return self._call("POST", "/announce", {"id": event_id, "text": text, "volume": volume})

    def prepare(self, texts: list, start: bool = True, force: bool = False) -> Dict[str, Any]:
        """【2026-10-30】文化祭の出店紹介(pr_intro.py): 文章の音声を事前に合成して保存するよう依頼する(2台目の背景で1本ずつ。すぐ返る)。
        start=Falseは状態を見るだけ。応答の data["items"] は texts と同じ順の {"state": ready|queued|synthesizing|failed|missing, "error"}。"""
        return self._call("POST", "/announce_prepare", {"texts": list(texts), "start": bool(start), "force": bool(force)})

    def play_cached(self, event_id: str, text: str, volume: float) -> Dict[str, Any]:
        """作成済みの音声だけを1回鳴らす(合成しない。無ければ2台目が破棄する)。長い文章(400字)も可。"""
        return self._call("POST", "/announce", {"id": event_id, "text": text, "volume": volume, "cached_only": True})

    def set_volume(self, value: float) -> Dict[str, Any]:
        return self._call("POST", "/voice_levels", {"announce_volume": value})


def parse_volume(value: Any) -> Tuple[Optional[float], str]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None, "アナウンス音量は数値で指定してください"
    v = float(value)
    if not math.isfinite(v):
        return None, "アナウンス音量は有限の数値で指定してください"
    if v < 0.0 or v > VOLUME_MAX:
        return None, f"アナウンス音量は0〜{VOLUME_MAX:g}の範囲で指定してください"
    return v, ""


class AnnounceEventsController:
    def __init__(self, store: ae.AnnounceEventsStore, bridge: AnnounceBridgeProxy) -> None:
        self.store = store
        self.bridge = bridge

    def payload(self) -> Dict[str, Any]:
        """ページの初期表示・読み戻し用。2台目に繋がらなくても1台目の設定は返す(bridge.ok=false)。"""
        out: Dict[str, Any] = {"ok": True, "configured": self.bridge.configured, "config": self.store.read(),
                               "meta": ae.event_meta(), "bridge": None}
        res = self.bridge.get_state([e["text"] for e in out["config"]["events"].values()])
        out["bridge"] = (
            {"ok": True, "state": res["data"], "announce_volume": res["data"].get("announce_volume")}
            if res.get("ok") else {"ok": False, "error": res.get("error", "")}
        )
        return out

    def save(self, data: Any) -> Tuple[int, Dict[str, Any]]:
        """{"enabled"?, "events"?: {id: {変えた項目}}, "announce_volume"?} を検証して保存する。
        検証エラーは400で何も保存しない。(HTTPステータス, 応答dict)"""
        if not isinstance(data, dict):
            return 400, {"ok": False, "errors": ["不正なリクエストです"], "message": "不正なリクエストです"}
        body = dict(data)
        vol = None
        errors = []
        if "announce_volume" in body:
            vol, err = parse_volume(body.pop("announce_volume"))
            if err:
                errors.append(err)
        clean, derr = ae.validate_document(body)
        errors.extend(derr)
        if vol is None and not errors and not clean:
            errors.append("変更する項目がありません")
        if errors:
            return 400, {"ok": False, "errors": errors, "message": errors[0]}
        result: Dict[str, Any] = {"ok": True, "errors": [], "saved": {}}
        if clean:
            r = self.store.save(body)
            if not r.get("ok"):
                status = 400 if r.get("kind") == "invalid" else 500
                return status, {"ok": False, "errors": r.get("errors", []), "message": (r.get("errors") or ["保存できませんでした"])[0]}
            result["saved"]["events"] = True
            result["sig"] = r.get("sig")
        if vol is not None:
            res = self.bridge.set_volume(vol)
            if not res.get("ok"):
                result.update(ok=False, errors=[res.get("error", "")], message="アナウンス音量を2台目へ送れませんでした: " + res.get("error", ""))
                return 200, result
            warn = (res.get("data") or {}).get("warning")
            if warn:
                result["message"] = str(warn)
            result["saved"]["announce_volume"] = True
        result.setdefault("message", "保存しました")
        return 200, result

    def test(self, data: Any) -> Tuple[int, Dict[str, Any]]:
        """{"id", "text"?, "volume"?}。その場で1回鳴らす(textを省けば保存済みの言う内容)。(HTTPステータス, 応答dict)"""
        if not isinstance(data, dict):
            return 400, {"ok": False, "message": "不正なリクエストです", "errors": ["不正なリクエストです"]}
        eid = data.get("id")
        if not isinstance(eid, str) or eid not in ae.EVENT_DEFS:
            return 400, {"ok": False, "message": "未知のイベントです", "errors": ["未知のイベントです"]}
        for k in data:
            if k not in ("id", "text", "volume"):
                return 400, {"ok": False, "message": f"未知の項目です: {k}", "errors": [f"未知の項目です: {k}"]}
        cur = self.store.read()["events"][eid]
        fields = {"text": data.get("text", cur["text"]), "volume": data.get("volume", cur["volume"])}
        clean, errors = ae.validate_event_fields(eid, fields)
        if errors:
            return 400, {"ok": False, "errors": errors, "message": errors[0]}
        res = self.bridge.play_once(eid, clean["text"], float(clean["volume"]))
        if not res.get("ok"):
            msg = res.get("error", "")
            return 200, {"ok": False, "configured": res.get("configured", True), "message": msg, "errors": [msg]}
        body = res["data"]
        if body.get("muted"):
            return 200, {"ok": True, "played": False, "muted": True,
                         "message": "音声が全体OFFのため鳴りません(音声の全体スイッチをONにしてください)"}
        return 200, {"ok": True, "played": True, "message": "鳴らします(初めての文言は合成に数秒かかります)"}
