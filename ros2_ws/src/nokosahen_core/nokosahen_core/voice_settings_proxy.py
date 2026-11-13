"""voice_bridgeの音声設定・計測API(/voice_settings, /voice_metrics, /voice_preview, /speak)を、
ダッシュボード経由で中継する(bgm_proxy.pyと同じ作り、標準ライブラリのurllibのみ)。

【2026-10-06追加】呼びかけ音声設定ページ(/voice_settings、docs/voice_settings.md)用。
voice_bridge(2台目Jetson、http://203.0.113.9:8100)のこれらのAPIはLAN内では認証なしで叩ける
(/status /speak /bgm と同じ)。ブラウザから直接呼ばせず、ダッシュボードのログインの内側に閉じ込める。

- `voice_settings_url`が空ならこの機能は無効(configured=False、UI側は項目ごと隠す)。
- 接続できない/タイムアウトのときは `{"ok": False, "kind": "unreachable", "error": "2台目に接続できません: ..."}`。
  voice_bridgeがHTTPエラー(400等)を返したときは `kind: "http"` で、相手の日本語エラー一覧を`errors`に入れる。
- タイムアウト: 設定の読み書きは3秒、計測の取得は8秒(SQLiteの読み出し)、
  お試し生成(/voice_preview)はOllamaを1回呼ぶので長め(既定95秒)。
- 検証(validate)の値の範囲は voice_bridge.py の NUMERIC_LIMITS / PERSONA_MAX_CHARS と同じにすること
  (テスト`test_voice_settings.py`が両者の一致を確認する)。最終的な検証はvoice_bridge側でも行われる。
"""

from __future__ import annotations

import json
import math
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional, Tuple

SETTINGS_TIMEOUT_S = 3.0
METRICS_TIMEOUT_S = 8.0
PREVIEW_TIMEOUT_S = 95.0

PERSONA_MAX_CHARS = 4000
BOOL_KEYS = ("include_menu_names", "include_remaining")
# key: (最小, 最大, 整数か) — voice_bridge.NUMERIC_LIMITSと同じ
NUMERIC_LIMITS: Dict[str, Tuple[float, float, bool]] = {
    "temperature": (0.0, 2.0, False),
    "num_predict": (32, 2000, True),
    "max_chars": (0, 500, True),
    "ollama_timeout_sec": (5.0, 300.0, False),
    "min_repeat_sec": (0.0, 600.0, False),
}
SETTING_KEYS = BOOL_KEYS + tuple(NUMERIC_LIMITS) + ("persona_prompt",)
SPEAK_TEXT_MAX = 400


def validate_voice_settings(data: Any) -> Tuple[Dict[str, Any], List[str]]:
    """音声生成の設定(部分更新可)を検証する。(payload, エラー一覧[日本語])。丸めない。

    値がNone(null)の項目は「既定に戻す」としてそのまま通す。
    """
    if not isinstance(data, dict):
        return {}, ["音声設定はオブジェクトで指定してください"]
    payload: Dict[str, Any] = {}
    errors: List[str] = []
    for key, value in data.items():
        if key not in SETTING_KEYS:
            errors.append(f"未知の項目です: {key}")
            continue
        if value is None:
            payload[key] = None
            continue
        if key in BOOL_KEYS:
            if not isinstance(value, bool):
                errors.append(f"{key}は true/false で指定してください")
            else:
                payload[key] = value
        elif key == "persona_prompt":
            if not isinstance(value, str):
                errors.append("プロンプトは文字列で指定してください")
                continue
            text = value.strip()
            if not text:
                errors.append("プロンプトを空にはできません(標準に戻すときは「標準に戻す」を使ってください)")
            elif len(text) > PERSONA_MAX_CHARS:
                errors.append(f"プロンプトは{PERSONA_MAX_CHARS}文字以内にしてください(現在{len(text)}文字)")
            elif "\x00" in text:
                errors.append("プロンプトに使えない文字(NUL)が含まれています")
            else:
                payload[key] = text
        else:
            lo, hi, is_int = NUMERIC_LIMITS[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                errors.append(f"{key}は数値で指定してください")
                continue
            v = float(value)
            if not math.isfinite(v):
                errors.append(f"{key}は有限の数値で指定してください")
                continue
            if is_int:
                if v != int(v):
                    errors.append(f"{key}は整数で指定してください")
                    continue
                v = int(v)
            if v < lo or v > hi:
                errors.append(f"{key}は{lo:g}〜{hi:g}の範囲で指定してください(指定: {value})")
                continue
            payload[key] = v
    return payload, errors


class VoiceSettingsProxy:
    def __init__(
        self,
        base_url: str = "",
        timeout_s: float = SETTINGS_TIMEOUT_S,
        metrics_timeout_s: float = METRICS_TIMEOUT_S,
        preview_timeout_s: float = PREVIEW_TIMEOUT_S,
    ) -> None:
        self.base_url = (base_url or "").strip().rstrip("/")
        self.timeout_s = float(timeout_s)
        self.metrics_timeout_s = float(metrics_timeout_s)
        self.preview_timeout_s = float(preview_timeout_s)

    @property
    def configured(self) -> bool:
        return bool(self.base_url)

    # --- 低レベル ---

    def _request(self, path: str, payload: Optional[Dict[str, Any]], timeout: float) -> Dict[str, Any]:
        url = f"{self.base_url}{path}"
        if payload is None:
            req = urllib.request.Request(url, method="GET")
        else:
            # allow_nan=False: 万一NaN等が混ざっても不正なJSONを送らない(ここで例外=送らない)
            req = urllib.request.Request(
                url,
                data=json.dumps(payload, allow_nan=False).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
        with urllib.request.urlopen(req, timeout=timeout) as res:
            body = json.loads(res.read().decode("utf-8"))
        if not isinstance(body, dict):
            raise ValueError("応答がJSONオブジェクトではありません")
        return body

    @staticmethod
    def _fail(kind: str, text: str, errors: Optional[List[str]] = None) -> Dict[str, Any]:
        return {"ok": False, "configured": True, "kind": kind, "error": text,
                "errors": errors or [text], "message": text}

    def _call(self, path: str, payload: Optional[Dict[str, Any]], timeout: float) -> Dict[str, Any]:
        if not self.configured:
            msg = "音声設定未設定(voice_settings_urlが空です)"
            return {"ok": False, "configured": False, "kind": "not_configured",
                    "error": msg, "errors": [msg], "message": msg}
        try:
            return {"ok": True, "configured": True, "data": self._request(path, payload, timeout)}
        except urllib.error.HTTPError as exc:
            detail: List[str] = []
            try:
                body = json.loads(exc.read().decode("utf-8"))
                if isinstance(body, dict):
                    if isinstance(body.get("errors"), list):
                        detail = [str(e) for e in body["errors"]]
                    elif body.get("error"):
                        detail = [str(body["error"])]
            except Exception:  # noqa: BLE001
                pass
            finally:
                exc.close()
            text = f"voice_bridgeがエラーを返しました(HTTP {exc.code})"
            if detail:
                text += ": " + " / ".join(detail)
            return self._fail("http", text, detail or [text])
        except Exception as exc:  # noqa: BLE001  (URLError/timeout/JSON不正など全て接続不可として扱う)
            return self._fail("unreachable", f"2台目に接続できません: {exc}")

    # --- 公開API ---

    def get_settings(self) -> Dict[str, Any]:
        return self._call("/voice_settings", None, self.timeout_s)

    def save_settings(self, payload: Dict[str, Any], persist: bool = True) -> Dict[str, Any]:
        """検証済みのpayload(validate_voice_settingsの結果)をvoice_bridgeへ送る。
        persist=Falseは弱いアカウント(operator)の一時的な変更: voice_bridgeにpersist:falseを付けて送り、保存ファイルは変えさせない(docs/accounts.md)。"""
        if not persist:
            payload = {**payload, "persist": False}
        return self._call("/voice_settings", payload, self.timeout_s)

    def get_metrics(self, limit: int = 50, row_id: Optional[int] = None) -> Dict[str, Any]:
        if row_id is not None:
            return self._call(f"/voice_metrics?id={int(row_id)}", None, self.metrics_timeout_s)
        limit = max(1, min(int(limit), 500))
        return self._call(f"/voice_metrics?limit={limit}", None, self.metrics_timeout_s)

    def preview(self, context: Dict[str, Any], settings: Dict[str, Any]) -> Dict[str, Any]:
        """ドライラン。話さず・履歴に残さず、未保存の設定で文面を生成して返す。"""
        return self._call("/voice_preview", {"context": context, "settings": settings}, self.preview_timeout_s)

    def speak_text(self, text: str) -> Dict[str, Any]:
        """既に決まった文面をそのまま話す(AIによる言い換えなし)。既存の/speakを使う。"""
        return self._call("/speak", {"text": text, "generate": False}, self.timeout_s)
