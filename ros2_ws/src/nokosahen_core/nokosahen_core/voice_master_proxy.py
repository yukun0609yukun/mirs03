"""voice_bridgeの音声「全体ON/OFF」API(GET/POST {voice_master_url}/voice_master)を、
ダッシュボード経由で中継する(bgm_proxy.pyと同じ作り、標準ライブラリのurllibのみ)。

【2026-10-09追加】docs/voice_master.md。マスタースイッチがOFFの間、voice_bridge(2台目Jetson、
http://198.51.100.3:8100)は発話(合成・再生)・Ollamaによる文面生成・BGMのすべてを止める。
voice_bridgeの/voice_masterはLAN内では認証なしで叩ける(/bgm等と同じ)ので、ブラウザから
直接呼ばせず、ダッシュボードのログイン(/api/voice_master)の内側に閉じ込める。

- `voice_master_url`が空ならこの機能は無効(configured=False、UI側は項目ごと隠す)。
- GET: voice_bridgeの状態をそのまま`status`として返す(3秒タイムアウト)。
- POST: `{"enabled": true/false}` だけを受け付ける(それ以外・文字列"false"等は400)。
"""

from __future__ import annotations

import json
import urllib.error
import urllib.request
from typing import Any, Dict, Optional, Tuple

MASTER_TIMEOUT_S = 3.0

NOT_CONFIGURED = "音声の全体ON/OFFは未設定です(voice_master_urlが空です)"


class VoiceMasterProxy:
    def __init__(self, base_url: str = "", timeout_s: float = MASTER_TIMEOUT_S) -> None:
        self.base_url = (base_url or "").strip().rstrip("/")
        self.timeout_s = float(timeout_s)

    @property
    def configured(self) -> bool:
        return bool(self.base_url)

    @staticmethod
    def validate(data: Any) -> Tuple[Optional[Dict[str, Any]], str]:
        """入力を検証してvoice_bridgeへ送るdictを返す。(payload, error)。enabledはboolのみ。"""
        if not isinstance(data, dict):
            return None, "不正なリクエストです"
        extra = set(data) - {"enabled"}
        if extra:
            return None, "指定できる項目は enabled のみです"
        if not isinstance(data.get("enabled"), bool):
            return None, "enabledは true/false で指定してください"
        return {"enabled": data["enabled"]}, ""

    def _request(self, payload: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        url = f"{self.base_url}/voice_master"
        if payload is None:
            req = urllib.request.Request(url, method="GET")
        else:
            req = urllib.request.Request(
                url,
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
        with urllib.request.urlopen(req, timeout=self.timeout_s) as res:
            body = json.loads(res.read().decode("utf-8"))
        if not isinstance(body, dict) or not isinstance(body.get("enabled"), bool):
            raise ValueError("応答の形式が想定と違います(voice_bridgeが古い可能性があります)")
        return body

    @staticmethod
    def _fail(text: str) -> Dict[str, Any]:
        # messageはブラウザ側postForMessageが表示に使う(errorと同じ文面)。
        return {"ok": False, "configured": True, "error": text, "message": text}

    def _call(self, payload: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        if not self.configured:
            return {"ok": False, "configured": False, "error": NOT_CONFIGURED, "message": NOT_CONFIGURED}
        try:
            return {"ok": True, "configured": True, "status": self._request(payload)}
        except urllib.error.HTTPError as exc:
            exc.close()
            if exc.code == 404:
                return self._fail("2台目のvoice_bridgeが全体ON/OFFに未対応です(voice_bridgeを更新して再起動してください)")
            return self._fail(f"voice_bridgeがエラーを返しました(HTTP {exc.code})")
        except Exception as exc:  # noqa: BLE001  (URLError/timeout/JSON不正など全て接続不可として扱う)
            return self._fail(f"2台目に接続できません: {exc}")

    def get_status(self) -> Dict[str, Any]:
        return self._call(None)

    def set(self, data: Any, persist: bool = True) -> Tuple[int, Dict[str, Any]]:
        """(HTTPステータス, 応答dict)を返す。検証エラーは400、転送失敗は200+ok:false。
        persist=Falseは弱いアカウント(operator)の一時的な変更: voice_bridgeにpersist:falseを付けて送り、保存ファイルは変えさせない(docs/accounts.md)。"""
        if not self.configured:
            return 200, {"ok": False, "configured": False, "error": NOT_CONFIGURED, "message": NOT_CONFIGURED}
        payload, err = self.validate(data)
        if payload is None:
            return 400, {"ok": False, "configured": True, "error": err, "message": err}
        if not persist:
            payload = {**payload, "persist": False}
        res = self._call(payload)
        if res.get("ok"):
            warn = (res.get("status") or {}).get("warning")
            if warn:
                res["message"] = str(warn)  # 保存失敗の警告をUIに出す(切替自体は反映済み)
        return 200, res
