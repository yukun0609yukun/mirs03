"""ラズパイ(腹部ディスプレイ)の管理用API(kiosk_admin.py、既定 http://198.51.100.1:8083)を、
ダッシュボード経由で中継する薄いプロキシ。標準ライブラリのみ(urllib)。

- `survey_admin_url`が空ならこの機能は無効(configured=False、UI側は項目ごと隠す)。
- GET状態: ラズパイの /state をそのまま`state`として返す。短いタイムアウト(2.5秒)で、
  複数の端末が5秒おきに読んでもラズパイへの問い合わせは約2秒に1回に抑える(結果をキャッシュ。
  失敗もキャッシュするので、ラズパイが落ちていても画面の更新が詰まらない)。
- モード切替: {"mode":"survey"|"customer"} だけを検証して転送する。キャッシュは切替直後に捨てる。
- 模擬POSの数字変更(2026-10-09追加): {"action":"randomize"|"reset"|"set","values":[7個]} を検証して
  ラズパイの POST /ocr_mock へ転送する(OCRテスト用。ocrtestモードでなければラズパイが409で拒否し、その理由を返す)。
  応答の state.ocr_mock.seq を、画面が GET の state.ocr_mock.ack_seq(模擬画面が適用した番号)と突き合わせて読み戻し確認する(2026-10-13)。
- CSV: 回答CSV(個人情報は持たない設計)を上限 MAX_CSV_BYTES で中継する。
呼び出し元の認可(ログイン・guestは書込み不可)はダッシュボード側(dashboard_common)で済ませる。
"""

from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request
from typing import Any, Callable, Dict, Optional, Tuple

SURVEY_TIMEOUT_S = 2.5
MODE_TIMEOUT_S = 12.0           # 切替はラズパイ側で switch_kiosk.sh を呼ぶので少し長め
MAX_CSV_BYTES = 2 * 1024 * 1024
CACHE_TTL_S = 2.0
MODES = ("survey", "customer", "ocrtest")
OCR_MOCK_ACTIONS = ("randomize", "reset", "set")
OCR_MOCK_ROWS = 7
OCR_MOCK_MAX = 9999


def validate_mode_request(data: Any) -> Tuple[Optional[str], str]:
    """POST /api/survey/mode のボディを検証する。(mode, error)。許すのは {"mode": survey|customer|ocrtest} のみ"""
    if not isinstance(data, dict) or set(data) != {"mode"}:
        return None, 'リクエストは {"mode": "survey" / "customer" / "ocrtest"} だけにしてください'
    mode = data["mode"]
    if not isinstance(mode, str) or mode not in MODES:
        return None, "modeは survey / customer / ocrtest のどれかを指定してください"
    return mode, ""


def validate_ocr_mock_request(data: Any) -> Tuple[Optional[Dict[str, Any]], str]:
    """POST /api/survey/ocr_mock のボディを検証する。(転送するdict, error)。
    許すのは {"action":"randomize"|"reset"} か {"action":"set","values":[7個の整数0〜9999]} のみ(kiosk_admin.pyと同じ規則)"""
    if not isinstance(data, dict) or "action" not in data:
        return None, 'リクエストは {"action": "randomize" / "reset" / "set"} の形にしてください'
    action = data["action"]
    if not isinstance(action, str) or action not in OCR_MOCK_ACTIONS:
        return None, "actionは randomize / reset / set のどれかを指定してください"
    if action != "set":
        if set(data) != {"action"}:
            return None, "randomize / reset には他の項目を付けられません"
        return {"action": action}, ""
    if set(data) != {"action", "values"}:
        return None, 'setは {"action":"set","values":[7個の数]} で指定してください'
    values = data["values"]
    if not isinstance(values, list) or len(values) != OCR_MOCK_ROWS:
        return None, f"valuesは{OCR_MOCK_ROWS}個の数にしてください"
    for v in values:
        if isinstance(v, bool) or not isinstance(v, int) or v < 0 or v > OCR_MOCK_MAX:
            return None, f"数字は0〜{OCR_MOCK_MAX}の整数にしてください"
    return {"action": "set", "values": list(values)}, ""


class SurveyAdminProxy:
    def __init__(self, base_url: str = "", timeout_s: float = SURVEY_TIMEOUT_S,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.base_url = (base_url or "").strip().rstrip("/")
        self.timeout_s = float(timeout_s)
        self._clock = clock
        self._lock = threading.Lock()
        self._cache: Optional[Tuple[float, Dict[str, Any]]] = None

    @property
    def configured(self) -> bool:
        return bool(self.base_url)

    @staticmethod
    def _fail(text: str) -> Dict[str, Any]:
        return {"ok": False, "configured": True, "error": text, "message": text}

    @staticmethod
    def _unconfigured() -> Dict[str, Any]:
        text = "アンケート管理未設定(survey_admin_urlが空です)"
        return {"ok": False, "configured": False, "error": text, "message": text}

    def _request(self, path: str, payload: Optional[Dict[str, Any]], timeout: float) -> Dict[str, Any]:
        url = self.base_url + path
        if payload is None:
            req = urllib.request.Request(url, method="GET")
        else:
            req = urllib.request.Request(
                url, data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(req, timeout=timeout) as res:
            body = json.loads(res.read(65536).decode("utf-8"))
        if not isinstance(body, dict):
            raise ValueError("応答がJSONオブジェクトではありません")
        return body

    def _call(self, path: str, payload: Optional[Dict[str, Any]], timeout: float) -> Dict[str, Any]:
        if not self.configured:
            return self._unconfigured()
        try:
            return {"ok": True, "configured": True, "state": self._request(path, payload, timeout)}
        except urllib.error.HTTPError as exc:
            exc.close()
            return self._fail(f"ラズパイがエラーを返しました(HTTP {exc.code})")
        except Exception as exc:  # noqa: BLE001  (URLError/timeout/JSON不正など全て届かないとして扱う)
            return self._fail(f"ラズパイに届きません: {exc}")

    def get_state(self, fresh: bool = False) -> Dict[str, Any]:
        """fresh=True はキャッシュを使わず問い合わせる(回答中かの確認など、古い値だと困る用途)。"""
        if not self.configured:
            return self._unconfigured()
        now = self._clock()
        with self._lock:
            if not fresh and self._cache is not None and now - self._cache[0] < CACHE_TTL_S:
                return self._cache[1]
        res = self._call("/state", None, self.timeout_s)  # ロックの外で呼ぶ(他のリクエストを待たせない)
        with self._lock:
            self._cache = (self._clock(), res)
        return res

    def set_mode(self, data: Any) -> Tuple[int, Dict[str, Any]]:
        """(HTTPステータス, 応答dict)。検証エラーは400、転送失敗は200+ok:false"""
        if not self.configured:
            return 200, self._unconfigured()
        mode, err = validate_mode_request(data)
        if mode is None:
            return 400, {"ok": False, "configured": True, "error": err, "message": err}
        res = self._call("/mode", {"mode": mode}, MODE_TIMEOUT_S)
        with self._lock:
            self._cache = None  # 切替直後の再取得で新しい状態を見せる
        return 200, res

    def set_ocr_mock(self, data: Any) -> Tuple[int, Dict[str, Any]]:
        """模擬POSの数字変更を転送する。(HTTPステータス, 応答dict)。検証エラーは400、
        ラズパイの409(OCRテストモードでない)は理由つきの ok:false(200)で返す"""
        if not self.configured:
            return 200, self._unconfigured()
        cmd, err = validate_ocr_mock_request(data)
        if cmd is None:
            return 400, {"ok": False, "configured": True, "error": err, "message": err}
        try:
            body = self._request("/ocr_mock", cmd, SURVEY_TIMEOUT_S + 2)
            with self._lock:
                self._cache = None  # 送った直後の読み戻し(ocr_mock.ack_seq)で古い状態を見せない
            return 200, {"ok": True, "configured": True, "state": body}
        except urllib.error.HTTPError as exc:
            msg = ""
            try:
                msg = str(json.loads(exc.read(4096).decode("utf-8")).get("message") or "")
            except Exception:  # noqa: BLE001
                pass
            code = exc.code
            exc.close()
            return 200, self._fail(msg or f"ラズパイがエラーを返しました(HTTP {code})")
        except Exception as exc:  # noqa: BLE001
            return 200, self._fail(f"ラズパイに届きません: {exc}")

    def get_csv(self) -> Tuple[int, bytes, str]:
        """(HTTPステータス, CSV本体 or 空, エラー文)。上限超過は502"""
        if not self.configured:
            return 404, b"", self._unconfigured()["message"]
        try:
            with urllib.request.urlopen(self.base_url + "/responses.csv", timeout=self.timeout_s + 2) as res:
                body = res.read(MAX_CSV_BYTES + 1)
        except urllib.error.HTTPError as exc:
            exc.close()
            return 502, b"", f"ラズパイがエラーを返しました(HTTP {exc.code})"
        except Exception as exc:  # noqa: BLE001
            return 502, b"", f"ラズパイに届きません: {exc}"
        if len(body) > MAX_CSV_BYTES:
            return 502, b"", "回答CSVが大きすぎます(2MB超)"
        return 200, body, ""
