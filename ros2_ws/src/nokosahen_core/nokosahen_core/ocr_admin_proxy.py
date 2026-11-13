"""タブレットOCR(tablet_ocr_reader.py、既定 http://127.0.0.1:8091)を、ダッシュボード経由で
状態表示・画質プロファイル切替するための薄いプロキシ(2026-10-09追加)。標準ライブラリのみ(urllib)。

- `ocr_admin_url`が空ならこの機能は無効(configured=False、UI側はカードごと隠す)。
- GET状態: OCRの /api/settings.json(プロファイル・実効解像度など)と /api/readings.json(7スロットの読み取り)
  を取り、画面表示用に要約して`state`として返す。短いタイムアウトで、結果は2秒キャッシュ(失敗もキャッシュ。
  OCRが落ちていても画面の更新が詰まらない)。画像(/api/preview.jpg等)は取らない
  (OCR側は画像を取得すると「閲覧中」とみなして負荷が上がるため)。
- 画質(2026-10-10追加): stateの`quality`にピント(OCRの/api/readings.jsonのfocus)と画質の自動調整の状態
  (/api/settings.jsonのautotune)の要約を含める。POST /api/ocr/autotune は {"action":"start"|"revert"|"cancel"}
  だけを検証してOCRの POST /api/autotune へ転送する(古いOCRで404/409でも例外にせず理由を返す)。
- プロファイル切替: {"camera_profile":"demo"|"production"} だけを検証して POST /api/settings へ転送する。
  カメラの再設定に数秒かかるので長めのタイムアウト。キャッシュは切替直後に捨てる。
呼び出し元の認可(ログイン・guestは書込み不可)はダッシュボード側(dashboard_common)で済ませる。
"""

from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request
from typing import Any, Callable, Dict, List, Optional, Tuple

OCR_TIMEOUT_S = 2.5
PROFILE_TIMEOUT_S = 20.0        # カメラの設定し直しに数秒かかる
CACHE_TTL_S = 2.0
PROFILES = ("demo", "production")
AUTOTUNE_ACTIONS = ("start", "revert", "cancel")
# OCR側のスロット名 -> 表示名(模擬POSの並びと同じ順)
SLOT_LABELS = (
    ("A", "Aランチ"), ("B", "Bランチ"), ("C", "Cランチ"), ("curry", "カレー"),
    ("noodles", "麺類"), ("alacarte_b", "アラカルトB"), ("alacarte_100", "アラカルト100円"),
)


def validate_profile_request(data: Any) -> Tuple[Optional[str], str]:
    """POST /api/ocr/profile のボディを検証する。(profile, error)。許すのは {"camera_profile": demo|production} のみ"""
    if not isinstance(data, dict) or set(data) != {"camera_profile"}:
        return None, 'リクエストは {"camera_profile": "demo" / "production"} だけにしてください'
    p = data["camera_profile"]
    if not isinstance(p, str) or p not in PROFILES:
        return None, "camera_profileは demo / production のどちらかを指定してください"
    return p, ""


def validate_autotune_request(data: Any) -> Tuple[Optional[str], str]:
    """POST /api/ocr/autotune のボディを検証する。(action, error)。許すのは {"action": start|revert|cancel} のみ"""
    if not isinstance(data, dict) or set(data) != {"action"}:
        return None, 'リクエストは {"action": "start" / "revert" / "cancel"} だけにしてください'
    a = data["action"]
    if not isinstance(a, str) or a not in AUTOTUNE_ACTIONS:
        return None, "actionは start / revert / cancel のどれかを指定してください"
    return a, ""


def summarize_quality(settings: Any, readings: Any) -> Optional[Dict[str, Any]]:
    """OCRのfocus(readings.json)とautotune(settings.json)を表示用に要約する。どちらも無ければNone(古いOCR)"""
    focus = readings.get("focus") if isinstance(readings, dict) else None
    at = settings.get("autotune") if isinstance(settings, dict) else None
    if not isinstance(focus, dict) and not isinstance(at, dict):
        return None
    out: Dict[str, Any] = {"focus": None, "autotune": None}
    if isinstance(focus, dict):
        out["focus"] = {
            "grade": focus.get("grade") if isinstance(focus.get("grade"), str) else None,
            "grade_label": focus.get("grade_label") if isinstance(focus.get("grade_label"), str) else None,
            "avg": _num(focus.get("avg")), "peak": _num(focus.get("peak")),
            "advice": focus.get("advice") if isinstance(focus.get("advice"), str) else None,
            "warnings": [w for w in focus.get("warnings", []) if isinstance(w, str)][:4]
            if isinstance(focus.get("warnings"), list) else [],
        }
    if isinstance(at, dict):
        prog = at.get("progress") if isinstance(at.get("progress"), dict) else {}
        out["autotune"] = {
            "state": at.get("state") if isinstance(at.get("state"), str) else None,
            "message": at.get("message") if isinstance(at.get("message"), str) else "",
            "done": _num(prog.get("done")), "total": _num(prog.get("total")),
            "has_saved": bool(at.get("saved")),
        }
    return out


def summarize_readings(data: Any) -> Optional[List[Dict[str, Any]]]:
    """OCRの /api/readings.json の "readings" を、固定順の表示用リストに要約する"""
    if not isinstance(data, dict) or not isinstance(data.get("readings"), dict):
        return None
    rd = data["readings"]
    out: List[Dict[str, Any]] = []
    known = {k for k, _ in SLOT_LABELS}
    for slot, label in list(SLOT_LABELS) + [(k, k) for k in rd if k not in known]:
        r = rd.get(slot)
        if not isinstance(r, dict):
            out.append({"slot": slot, "label": label, "confirmed": None, "raw": None, "sent": False,
                        "estimated": False, "seen": False})
            continue
        out.append({
            "slot": slot, "label": str(r.get("menu_name") or label),
            "confirmed": r.get("confirmed_value"), "raw": r.get("raw_value"),
            "sent": bool(r.get("sent")), "estimated": bool(r.get("estimated")), "seen": True,
        })
    return out


def _num(v: Any) -> Optional[float]:
    return v if isinstance(v, (int, float)) and not isinstance(v, bool) else None


class OcrAdminProxy:
    def __init__(self, base_url: str = "", timeout_s: float = OCR_TIMEOUT_S,
                 clock: Callable[[], float] = time.monotonic,
                 wall: Callable[[], float] = time.time) -> None:
        self.base_url = (base_url or "").strip().rstrip("/")
        self.timeout_s = float(timeout_s)
        self._clock = clock
        self._wall = wall
        self._lock = threading.Lock()
        self._cache: Optional[Tuple[float, Dict[str, Any]]] = None

    @property
    def configured(self) -> bool:
        return bool(self.base_url)

    # 【2026-10-21】カメラ映像の中継。OCRの確認画面(8091)は、1台目の内側(127.0.0.1)だけで待ち受けるようにしたため、
    # パソコンから映像を見るには、ダッシュボード(ログイン必須)が中継する。見ている間だけOCRが負荷を上げる(OCR側が「閲覧中」と
    # みなす)ので、画面側はチェックを入れたときだけ取りにくる。
    IMAGE_PATHS = {"preview": "/api/preview.jpg", "processed": "/api/processed.jpg"}
    MAX_IMAGE_BYTES = 2_000_000

    def get_image(self, kind: str) -> Tuple[int, bytes]:
        """OCRのJPEG(preview=ROI枠・読み取り値つきのカメラ映像 / processed=認識に渡している画像)を取る。(HTTPステータス, 本体)。
        取れない(未設定・OCRが落ちている・まだ映像がない)ときは、404か502と空の本体。JPEG以外・大きすぎる応答は502。"""
        path = self.IMAGE_PATHS.get(kind)
        if not self.configured or path is None:
            return 404, b""
        try:
            with urllib.request.urlopen(urllib.request.Request(self.base_url + path, method="GET"), timeout=self.timeout_s) as res:
                ctype = str(res.headers.get("Content-Type", "")).lower()
                data = res.read(self.MAX_IMAGE_BYTES + 1)
        except urllib.error.HTTPError as exc:
            exc.close()
            return (404 if exc.code == 404 else 502), b""
        except Exception:  # noqa: BLE001  (届かない・タイムアウト)
            return 502, b""
        if "image/jpeg" not in ctype or len(data) > self.MAX_IMAGE_BYTES or not data:
            return 502, b""
        return 200, data

    @staticmethod
    def _fail(text: str) -> Dict[str, Any]:
        return {"ok": False, "configured": True, "error": text, "message": text}

    @staticmethod
    def _unconfigured() -> Dict[str, Any]:
        text = "タブレットOCR管理未設定(ocr_admin_urlが空です)"
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
            body = json.loads(res.read(262144).decode("utf-8"))
        if not isinstance(body, dict):
            raise ValueError("応答がJSONオブジェクトではありません")
        return body

    def _fetch_state(self) -> Dict[str, Any]:
        try:
            st = self._request("/api/settings.json", None, self.timeout_s)
        except urllib.error.HTTPError as exc:
            exc.close()
            return self._fail(f"OCRがエラーを返しました(HTTP {exc.code})")
        except Exception as exc:  # noqa: BLE001  (届かない・JSON不正など)
            return self._fail(f"OCRに届きません: {exc}")
        prof = st.get("camera_profile")
        state: Dict[str, Any] = {
            "profile": prof if prof in PROFILES else None,
            "profiles": st.get("camera_profiles") if isinstance(st.get("camera_profiles"), dict) else {},
            "camera_actual": st.get("camera_actual") if isinstance(st.get("camera_actual"), dict) else None,
            "frame_rotate_deg": st.get("frame_rotate_deg"),
            "profile_supported": prof in PROFILES,
            "readings": None, "roi_lock": None, "updated_age_s": None, "readings_error": "",
            "quality": summarize_quality(st, None),
        }
        try:
            rd = self._request("/api/readings.json", None, self.timeout_s)
            state["readings"] = summarize_readings(rd)
            state["quality"] = summarize_quality(st, rd)
            lk = rd.get("roi_lock")
            state["roi_lock"] = lk if isinstance(lk, dict) else None
            upd = _num(rd.get("updated_at"))
            if upd:
                state["updated_age_s"] = max(0.0, round(self._wall() - upd, 1))
        except Exception as exc:  # noqa: BLE001  (設定は取れたので読み取りだけ欠けた状態で返す)
            state["readings_error"] = f"読み取り結果を取得できません: {exc}"
        return {"ok": True, "configured": True, "state": state}

    def get_state(self) -> Dict[str, Any]:
        if not self.configured:
            return self._unconfigured()
        now = self._clock()
        with self._lock:
            if self._cache is not None and now - self._cache[0] < CACHE_TTL_S:
                return self._cache[1]
        res = self._fetch_state()  # ロックの外で呼ぶ(他のリクエストを待たせない)
        with self._lock:
            self._cache = (self._clock(), res)
        return res

    def set_profile(self, data: Any) -> Tuple[int, Dict[str, Any]]:
        """(HTTPステータス, 応答dict)。検証エラーは400、転送失敗は200+ok:false"""
        if not self.configured:
            return 200, self._unconfigured()
        profile, err = validate_profile_request(data)
        if profile is None:
            return 400, {"ok": False, "configured": True, "error": err, "message": err}
        try:
            self._request("/api/settings", {"camera_profile": profile}, PROFILE_TIMEOUT_S)
            res: Dict[str, Any] = {"ok": True, "configured": True, "profile": profile}
        except urllib.error.HTTPError as exc:
            exc.close()
            res = self._fail(f"OCRが切替を受け付けませんでした(HTTP {exc.code})。OCR側が古い可能性があります")
        except Exception as exc:  # noqa: BLE001
            res = self._fail(f"OCRに届きません: {exc}")
        with self._lock:
            self._cache = None  # 切替直後の再取得で新しい状態を見せる
        return 200, res

    def set_autotune(self, data: Any) -> Tuple[int, Dict[str, Any]]:
        """画質の自動調整/元に戻す/中断をOCRの POST /api/autotune へ転送する(2026-10-10追加)。
        (HTTPステータス, 応答dict)。検証エラーは400、OCRが拒否(実行中など)・届かない場合は200+ok:false。"""
        if not self.configured:
            return 200, self._unconfigured()
        action, err = validate_autotune_request(data)
        if action is None:
            return 400, {"ok": False, "configured": True, "error": err, "message": err}
        try:
            self._request("/api/autotune", {"action": action}, self.timeout_s)
            res: Dict[str, Any] = {"ok": True, "configured": True, "action": action}
        except urllib.error.HTTPError as exc:
            reason = ""
            try:
                body = json.loads(exc.read(4096).decode("utf-8"))
                reason = str(body.get("error", "")) if isinstance(body, dict) else ""
            except Exception:  # noqa: BLE001
                reason = ""
            exc.close()
            res = self._fail(reason or f"OCRが受け付けませんでした(HTTP {exc.code})。OCR側が古い可能性があります")
        except Exception as exc:  # noqa: BLE001
            res = self._fail(f"OCRに届きません: {exc}")
        with self._lock:
            self._cache = None
        return 200, res
