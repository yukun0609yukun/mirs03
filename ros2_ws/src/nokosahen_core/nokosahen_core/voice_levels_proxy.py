"""voice_bridgeの音量API(GET/POST {voice_levels_url}/voice_levels)を、ダッシュボード経由で中継する
(voice_master_proxy.py / bgm_proxy.pyと同じ作り、標準ライブラリのurllibのみ)。

【2026-10-09追加】docs/voice_levels.md。発話音量(倍率)・BGM通常音量・BGM発話中(ダッキング)音量を
別々に設定する。voice_bridge(2台目Jetson、http://198.51.100.3:8100)の/voice_levelsはLAN内では認証なしで
叩けるので、ブラウザから直接呼ばせず、ダッシュボードのログイン(/api/voice_levels)の内側に閉じ込める。

- `voice_levels_url`が空ならこの機能は無効(configured=False、UI側は項目ごと隠す)。
- GET: voice_bridgeの状態をそのまま`status`として返す(3秒タイムアウト)。
- POST: speech_volume(0〜5) / bgm_volume(0〜5) / bgm_duck_volume(0〜bgm_volume)の部分更新(2026-10-09に上限を3/1から5へ)。
  値null=既定に戻す。丸めずに範囲外・型違い・未知の項目は400(voice_bridgeへは送らない)。
- 範囲は voice_bridge.py の LEVEL_MAX と同じ(この1か所の LEVEL_MAX_ALL がダッシュボード画面の上限にも使われる)にすること(test_voice_levels.pyが一致を確認する)。
"""

from __future__ import annotations

import json
import math
import urllib.error
import urllib.request
from typing import Any, Dict, Optional, Tuple

LEVELS_TIMEOUT_S = 3.0

# 音量の上限(倍率)。voice_bridge.py の SPEECH_VOLUME_MAX / BGM_VOLUME_MAX と同じ。画面のスライダー上限もこれから作る。
LEVEL_MAX_ALL = 5.0
# 画面で「実行する/やめる」の確認を出す境目(voice_bridge.LEVEL_CONFIRM_ABOVE と同じ)
LEVEL_CONFIRM_ABOVE = 2.0

# key: (表示名, 最大値) — voice_bridge.LEVEL_MAX / LEVEL_LABELS と同じ
LEVEL_LIMITS: Dict[str, Tuple[str, float]] = {
    "speech_volume": ("発話音量", LEVEL_MAX_ALL),
    "bgm_volume": ("BGM通常音量", LEVEL_MAX_ALL),
    "bgm_duck_volume": ("BGM発話中音量", LEVEL_MAX_ALL),
    # 【2026-10-13】状況アナウンスの基準音量(docs/announcements.md)。古いvoice_bridgeは持たないので応答での存在は必須にしない
    "announce_volume": ("アナウンス音量", LEVEL_MAX_ALL),
}
# 応答に必ず入っているはずの項目(announce_volumeは古いvoice_bridgeに無い)
REQUIRED_LEVEL_KEYS = ("speech_volume", "bgm_volume", "bgm_duck_volume")

NOT_CONFIGURED = "音量設定は未設定です(voice_levels_urlとbgm_urlが空です)"


class VoiceLevelsProxy:
    def __init__(self, base_url: str = "", timeout_s: float = LEVELS_TIMEOUT_S) -> None:
        self.base_url = (base_url or "").strip().rstrip("/")
        self.timeout_s = float(timeout_s)

    @property
    def configured(self) -> bool:
        return bool(self.base_url)

    @staticmethod
    def validate(data: Any) -> Tuple[Optional[Dict[str, Any]], str]:
        """入力を検証してvoice_bridgeへ送るdictを返す。(payload, error)。丸めない。"""
        if not isinstance(data, dict):
            return None, "不正なリクエストです"
        payload: Dict[str, Any] = {}
        for key, value in data.items():
            if key not in LEVEL_LIMITS:
                return None, f"未知の項目です: {key}"
            label, hi = LEVEL_LIMITS[key]
            if value is None:
                payload[key] = None
                continue
            # boolはintのサブクラスなので明示的に弾く
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                return None, f"{label}は数値で指定してください"
            v = float(value)
            if not math.isfinite(v):
                return None, f"{label}は有限の数値で指定してください"
            if v < 0.0 or v > hi:
                return None, f"{label}は0〜{hi:g}の範囲で指定してください"
            payload[key] = v
        if not payload:
            return None, "speech_volume / bgm_volume / bgm_duck_volume / announce_volume のいずれかを指定してください"
        duck, vol = payload.get("bgm_duck_volume"), payload.get("bgm_volume")
        if duck is not None and vol is not None and duck > vol:
            return None, "BGM発話中音量は通常音量以下にしてください"
        return payload, ""

    def _request(self, payload: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        url = f"{self.base_url}/voice_levels"
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
        if not isinstance(body, dict) or not all(
            isinstance(body.get(k), (int, float)) and not isinstance(body.get(k), bool) for k in REQUIRED_LEVEL_KEYS
        ):
            raise ValueError("応答の形式が想定と違います(voice_bridgeが古い可能性があります)")
        return body

    @staticmethod
    def _fail(text: str) -> Dict[str, Any]:
        # messageはブラウザ側が表示に使う(errorと同じ文面)。
        return {"ok": False, "configured": True, "error": text, "message": text}

    def _call(self, payload: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        if not self.configured:
            return {"ok": False, "configured": False, "error": NOT_CONFIGURED, "message": NOT_CONFIGURED}
        try:
            return {"ok": True, "configured": True, "status": self._request(payload)}
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
                return self._fail("2台目のvoice_bridgeが音量設定に未対応です(voice_bridgeを更新して再起動してください)")
            if exc.code == 400 and detail:
                return self._fail(detail)
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
                res["message"] = str(warn)  # 保存失敗の警告をUIに出す(反映自体は済み)
        return 200, res
