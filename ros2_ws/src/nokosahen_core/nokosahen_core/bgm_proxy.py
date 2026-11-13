"""voice_bridgeのBGM API(GET/POST {bgm_url}/bgm)を、ダッシュボード経由で中継する。

voice_bridge(2台目Jetson、http://203.0.113.9:8100)の/bgmはLAN内では認証なしで
叩ける。ブラウザから直接呼ばせず、ダッシュボードのログイン(/api/bgm)の内側に
閉じ込めるための薄いプロキシ。標準ライブラリのみ(urllib)で実装している。

- `bgm_url`が空ならこの機能は無効(configured=False、UI側は項目ごと隠す)。
- GET: voice_bridgeの状態をそのまま`status`として返す(3秒タイムアウト)。
- POST: enabled(bool)/volume/duck_volume(数値、0..BGM_VOLUME_MAXに丸める)を検証して転送する。
"""

from __future__ import annotations

import json
import math
import urllib.error
import urllib.request
from typing import Any, Dict, Optional, Tuple

BGM_TIMEOUT_S = 3.0
# voice_bridgeのBGM音量の上限(voice_bridge.BGM_VOLUME_MAXと同じ値)。以前は1.0で、音量カード(最大5)と食い違っていた(2026-10-10)。
BGM_VOLUME_MAX = 5.0


class BgmProxy:
    def __init__(self, base_url: str = "", timeout_s: float = BGM_TIMEOUT_S) -> None:
        self.base_url = (base_url or "").strip().rstrip("/")
        self.timeout_s = float(timeout_s)

    @property
    def configured(self) -> bool:
        return bool(self.base_url)

    @staticmethod
    def _clamp01(value: float) -> float:
        # 名前は互換のため残す(中身は0〜BGM_VOLUME_MAX)
        return min(BGM_VOLUME_MAX, max(0.0, value))

    def validate(self, data: Any) -> Tuple[Optional[Dict[str, Any]], str]:
        """入力を検証してvoice_bridgeへ送るdictを返す。(payload, error)。"""
        if not isinstance(data, dict):
            return None, "不正なリクエストです"
        payload: Dict[str, Any] = {}
        if "enabled" in data:
            if not isinstance(data["enabled"], bool):
                return None, "enabledは true/false で指定してください"
            payload["enabled"] = data["enabled"]
        for key, label in (("volume", "通常音量"), ("duck_volume", "発話中の音量")):
            if key not in data:
                continue
            value = data[key]
            # boolはintのサブクラスなので明示的に弾く
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                return None, f"{label}は数値で指定してください"
            value = float(value)
            if not math.isfinite(value):
                return None, f"{label}は有限の数値で指定してください"
            payload[key] = self._clamp01(value)
        if not payload:
            return None, "enabled / volume / duck_volume のいずれかを指定してください"
        return payload, ""

    def _request(self, payload: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        url = f"{self.base_url}/bgm"
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
        if not isinstance(body, dict):
            raise ValueError("応答がJSONオブジェクトではありません")
        return body

    @staticmethod
    def _fail(text: str) -> Dict[str, Any]:
        # messageはブラウザ側postForMessageが表示に使う(errorと同じ文面)。
        return {"ok": False, "configured": True, "error": text, "message": text}

    def _call(self, payload: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        if not self.configured:
            return {"ok": False, "configured": False, "error": "BGM未設定(bgm_urlが空です)", "message": "BGM未設定(bgm_urlが空です)"}
        try:
            return {"ok": True, "configured": True, "status": self._request(payload)}
        except urllib.error.HTTPError as exc:
            exc.close()
            return self._fail(f"voice_bridgeがエラーを返しました(HTTP {exc.code})")
        except Exception as exc:  # noqa: BLE001  (URLError/timeout/JSON不正など全て接続不可として扱う)
            return self._fail(f"接続できません: {exc}")

    def get_status(self) -> Dict[str, Any]:
        return self._call(None)

    def set(self, data: Any, persist: bool = True) -> Tuple[int, Dict[str, Any]]:
        """(HTTPステータス, 応答dict)を返す。検証エラーは400、転送失敗は200+ok:false。
        persist=Falseは弱いアカウント(operator)の一時的な変更: voice_bridgeにpersist:falseを付けて送り、保存ファイルは変えさせない(docs/accounts.md)。"""
        if not self.configured:
            return 200, {"ok": False, "configured": False, "error": "BGM未設定(bgm_urlが空です)", "message": "BGM未設定(bgm_urlが空です)"}
        payload, err = self.validate(data)
        if payload is None:
            return 400, {"ok": False, "configured": True, "error": err, "message": err}
        if not persist:
            payload = {**payload, "persist": False}
        return 200, self._call(payload)
