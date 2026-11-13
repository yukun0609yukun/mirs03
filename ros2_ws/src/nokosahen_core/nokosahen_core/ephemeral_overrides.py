"""弱いアカウント(operator)の変更を「一時的(プログラム再起動で消える)」にするための薄い共通層(純Python、ROS非依存)。

【2026-10-13追加】docs/accounts.md。

考え方
- 書き込みの**実行者のロール**がadmin以外なら、設定を**永続化しない**。adminの書き込みは従来どおり永続化する。
- この層は「いま一時的に上書きされている項目」の台帳(項目名→値、設定時刻、設定者)を持つ。
  `effective(item, 永続値)` は「永続値 + 一時上書き」を返す。台帳は**メモリ上のみ**で、プロセス再起動で空になる。
- 実際の値の反映先は2種類あり、どちらも**再起動で元に戻す**:
  (a) voice_bridge(2台目)のメモリ上の上書き: `persist:false` を付けて送り、voice_bridge側は保存ファイルを更新せず
      メモリだけ変える。ダッシュボード起動時に `POST /ephemeral_reset` で一時上書きを全て破棄する
      (voice_bridgeが再起動した場合は、そもそも保存していないので元に戻る)。
  (b) ファイル/ラズパイ側に既に実値が書かれてしまう項目(発話間隔ファイル、頭部ディスプレイの表示モード等):
      最初の上書きの直前の値(baseline)を**ジャーナル**(`~/.nokosahen_ephemeral_journal.json`)に記録し、
      ダッシュボード起動時にbaselineへ戻す(戻せなければ次回の起動/定期リトライまでジャーナルを残す)。
- adminが同じ項目を保存したら、その項目の一時上書きは破棄され、adminの値(永続)になる。
"""
from __future__ import annotations

import datetime
import json
import os
import tempfile
import threading
import time
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple

DEFAULT_JOURNAL_FILE = "~/.nokosahen_ephemeral_journal.json"

# 復元の種類(ジャーナルに書く。キー→復元関数はdashboard_common側で登録する)
RESTORE_ANNOUNCE_INTERVAL = "announce_interval"
RESTORE_SURVEY_MODE = "survey_mode"
RESTORE_OCR_MOCK = "ocr_mock"

ITEM_LABELS: Dict[str, str] = {
    "voice_levels.speech_volume": "発話音量",
    "voice_levels.bgm_volume": "BGM通常音量",
    "voice_levels.bgm_duck_volume": "BGM発話中音量",
    "voice_levels.announce_volume": "状況アナウンスの音量",
    "voice_master.enabled": "音声の全体ON/OFF",
    "announce.min_interval": "呼びかけの発話間隔",
    "display.survey_mode": "頭部ディスプレイの表示モード",
    "display.ocr_mock": "模擬POSの数字",
}


def item_label(item: str) -> str:
    if item in ITEM_LABELS:
        return ITEM_LABELS[item]
    if item.startswith("voice_settings."):
        return "音声設定(" + item.split(".", 1)[1] + ")"
    return item


def _short(v: Any) -> Any:
    return v if not isinstance(v, str) else (v if len(v) <= 80 else v[:80] + "…")


def items_for(path: str, data: Any) -> Dict[str, Any]:
    """この書き込みAPIが変える「項目名 -> 設定値」(一時上書き台帳のキーと表示用の値)。変更対象が無いAPIは空。"""
    d = data if isinstance(data, dict) else {}
    out: Dict[str, Any] = {}
    if path == "/api/voice_levels":
        for k in ("speech_volume", "bgm_volume", "bgm_duck_volume", "announce_volume"):
            if k in d:
                out[f"voice_levels.{k}"] = d[k]
    elif path == "/api/bgm":
        if "volume" in d:
            out["voice_levels.bgm_volume"] = d["volume"]
        if "duck_volume" in d:
            out["voice_levels.bgm_duck_volume"] = d["duck_volume"]
    elif path == "/api/voice_master":
        out["voice_master.enabled"] = d.get("enabled")
    elif path == "/api/voice_settings/save":
        if isinstance(d.get("announce"), dict):
            out["announce.min_interval"] = d["announce"].get("announce_min_interval_seconds")
        if isinstance(d.get("voice"), dict):
            for k, v in d["voice"].items():
                out[f"voice_settings.{k}"] = _short(v)
    elif path == "/api/survey/mode":
        out["display.survey_mode"] = d.get("mode")
    elif path == "/api/survey/ocr_mock":
        out["display.ocr_mock"] = d.get("action")
    return out


# 外部(ファイル/ラズパイ)に実値が書かれる項目の復元の種類
JOURNALED_ITEMS: Dict[str, str] = {
    "announce.min_interval": RESTORE_ANNOUNCE_INTERVAL,
    "display.survey_mode": RESTORE_SURVEY_MODE,
    "display.ocr_mock": RESTORE_OCR_MOCK,
}


class EphemeralOverrides:
    """一時的な上書きの台帳(スレッドセーフ)+ジャーナル。"""

    def __init__(self, journal_path: Optional[str] = None, clock: Callable[[], float] = time.time) -> None:
        self._lock = threading.Lock()
        self._clock = clock
        self._items: Dict[str, Dict[str, Any]] = {}
        # ジャーナルを使わない(メモリのみ)ときは None。空文字は「使わない」として扱う
        self.journal_path = os.path.expanduser(journal_path) if journal_path else None
        # 起動時にジャーナルを読む(前回のプロセスが残した復元待ち)。pending: item -> {"restore","baseline"}
        self._pending: Dict[str, Dict[str, Any]] = self._read_journal()

    # --- ジャーナル ---
    def _read_journal(self) -> Dict[str, Dict[str, Any]]:
        if not self.journal_path:
            return {}
        try:
            with open(self.journal_path, "r", encoding="utf-8") as f:
                doc = json.loads(f.read())
            ent = doc.get("entries") if isinstance(doc, dict) else None
            if not isinstance(ent, dict):
                return {}
            return {str(k): {"restore": str(v.get("restore", "")), "baseline": v.get("baseline")}
                    for k, v in ent.items() if isinstance(v, dict) and v.get("restore")}
        except (OSError, ValueError):
            return {}

    def _write_journal_locked(self) -> None:
        if not self.journal_path:
            return
        entries = dict(self._pending)
        for item, e in self._items.items():
            if e.get("restore"):
                entries[item] = {"restore": e["restore"], "baseline": e.get("baseline")}
        if not entries:
            try:
                os.remove(self.journal_path)
            except OSError:
                pass
            return
        doc = {"version": 1, "entries": entries,
               "updated_at": datetime.datetime.now().isoformat(timespec="seconds")}
        directory = os.path.dirname(self.journal_path) or "."
        tmp_path = None
        try:
            os.makedirs(directory, exist_ok=True)
            fd, tmp_path = tempfile.mkstemp(prefix=".nokosahen_ephemeral_", suffix=".tmp", dir=directory)
            with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as f:
                f.write(json.dumps(doc, ensure_ascii=False, indent=2) + "\n")
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp_path, self.journal_path)
            tmp_path = None
        except OSError:
            pass  # ジャーナルを書けなくても台帳(メモリ)は有効。復元できないリスクはdocsに記載
        finally:
            if tmp_path:
                try:
                    os.remove(tmp_path)
                except OSError:
                    pass

    # --- 台帳 ---
    def has_item(self, item: str) -> bool:
        with self._lock:
            return item in self._items

    def record(self, item: str, value: Any, actor: str, restore: Optional[str] = None,
               baseline: Any = None) -> None:
        """一時上書きを記録する。restore付き(外部に実値が書かれる項目)は最初のbaselineをジャーナルに残す。"""
        with self._lock:
            prev = self._items.get(item)
            keep_baseline = prev.get("baseline") if prev else baseline
            if prev is None and item in self._pending:
                keep_baseline = self._pending[item].get("baseline")  # 前回の復元待ちのbaselineを引き継ぐ
            self._items[item] = {"value": value, "at": self._clock(), "by": actor,
                                 "restore": restore, "baseline": keep_baseline}
            self._pending.pop(item, None)
            if restore:
                self._write_journal_locked()

    def discard(self, items: Iterable[str]) -> int:
        """adminが保存した項目の一時上書きを破棄する(adminの値が永続の正)。破棄した件数を返す。"""
        n = 0
        with self._lock:
            changed = False
            for item in items:
                e = self._items.pop(item, None)
                if e is not None:
                    n += 1
                    changed = changed or bool(e.get("restore"))
                if self._pending.pop(item, None) is not None:
                    changed = True
            if changed:
                self._write_journal_locked()
        return n

    def effective(self, item: str, persisted: Any) -> Tuple[Any, bool]:
        """(有効な値, 一時上書きか)。永続値 + 一時上書き。"""
        with self._lock:
            e = self._items.get(item)
        return (e["value"], True) if e is not None else (persisted, False)

    def count(self) -> int:
        with self._lock:
            return len(self._items)

    def items(self) -> List[Dict[str, Any]]:
        with self._lock:
            rows = [
                {"item": k, "label": item_label(k), "value": v["value"], "by": v["by"],
                 "at": datetime.datetime.fromtimestamp(v["at"]).isoformat(timespec="seconds")}
                for k, v in self._items.items()
            ]
        return sorted(rows, key=lambda r: r["at"])

    def pending_restore_count(self) -> int:
        with self._lock:
            return len(self._pending)

    # --- 復元 ---
    def restore_pending(self, restorers: Dict[str, Callable[[Any], bool]]) -> Tuple[int, int]:
        """前回の起動で残った(baselineがある)項目を元に戻す。(成功数, 残り数)。失敗した項目はジャーナルに残す。"""
        with self._lock:
            todo = dict(self._pending)
        done = 0
        for item, e in todo.items():
            fn = restorers.get(e.get("restore", ""))
            ok = False
            if fn is not None:
                try:
                    ok = bool(fn(e.get("baseline")))
                except Exception:  # noqa: BLE001
                    ok = False
            if ok:
                with self._lock:
                    self._pending.pop(item, None)
                done += 1
        with self._lock:
            self._write_journal_locked()
            return done, len(self._pending)

    def clear_all(self, restorers: Optional[Dict[str, Callable[[Any], bool]]] = None) -> Dict[str, Any]:
        """管理者の「全て破棄」。台帳を空にし、外部に実値がある項目はbaselineへ戻す(戻せなければ復元待ちに残す)。"""
        with self._lock:
            items = dict(self._items)
            self._items.clear()
        failed: List[str] = []
        for item, e in items.items():
            kind = e.get("restore")
            if not kind:
                continue  # voice_bridge等のメモリ上書きは、呼び出し側が /ephemeral_reset を送る
            fn = (restorers or {}).get(kind)
            ok = False
            if fn is not None:
                try:
                    ok = bool(fn(e.get("baseline")))
                except Exception:  # noqa: BLE001
                    ok = False
            if not ok:
                failed.append(item)
                with self._lock:
                    self._pending[item] = {"restore": kind, "baseline": e.get("baseline")}
        with self._lock:
            self._write_journal_locked()
        return {"cleared": len(items), "restore_failed": failed}


# ---------------------------------------------------------------------------
# 復元・リセット(ダッシュボード起動時 / 管理者の「全て破棄」)
# ---------------------------------------------------------------------------
SURVEY_GUARD_ENV = "NOKOSAHEN_EPHEMERAL_SURVEY_GUARD"  # "0"/"false"/"off" で、回答中の確認を無効化(従来動作)


def survey_answering(survey_admin: Any) -> Optional[bool]:
    """ラズパイ(腹部ディスプレイ)でアンケート回答中か。True=回答中 / False=回答中でない / None=確認できない。
    kiosk_adminの/stateの active(回答中)または session_state=="answering" で判定する。"""
    getter = getattr(survey_admin, "get_state", None)
    if getter is None:
        return False  # 状態を取れない替え玉(旧テスト等)は確認なし
    try:
        try:
            res = getter(fresh=True)  # キャッシュ(2秒)を使わない
        except TypeError:
            res = getter()
        if not isinstance(res, dict) or not res.get("ok"):
            return None
        st = res.get("state")
        if not isinstance(st, dict):
            return None
        return bool(st.get("active")) or st.get("session_state") == "answering"
    except Exception:  # noqa: BLE001
        return None


def build_restorers(announce_store: Any = None, survey_admin: Any = None,
                    guard_survey_active: Optional[bool] = None) -> Dict[str, Callable[[Any], bool]]:
    """ジャーナルのbaselineへ戻す関数群。引数は実物でもテスト用の替え玉でもよい(ダックタイピング)。

    guard_survey_active(既定True、環境変数 NOKOSAHEN_EPHEMERAL_SURVEY_GUARD=0 でも無効化): 表示モードの復元は、
    アンケート回答中は行わない(モードを書き換えるとChromiumが閉じて回答が消えるため)。回答中、または状態が確認できない
    ときは「戻せなかった」(False)として返し、ジャーナルに残して次の再試行(5秒間隔・最大60回)や次回起動に回す。
    確認できないときも戻さない理由: 戻すのが遅れても害は小さい(再試行される)が、回答の消失は取り返せない。
    確認できない=ラズパイに届かない状況で、どのみち切替も届かない。"""
    if guard_survey_active is None:
        guard_survey_active = os.environ.get(SURVEY_GUARD_ENV, "1").strip().lower() not in ("0", "false", "off", "no")
    restorers: Dict[str, Callable[[Any], bool]] = {}
    if announce_store is not None:
        def _announce(baseline: Any) -> bool:
            if baseline is None:  # ファイルが無かった=ROSパラメータ値に戻す
                try:
                    os.remove(announce_store.path)
                except FileNotFoundError:
                    pass
                except OSError:
                    return False
                return True
            return bool(announce_store.save_interval(baseline).get("ok"))
        restorers[RESTORE_ANNOUNCE_INTERVAL] = _announce
    if survey_admin is not None and getattr(survey_admin, "configured", False):
        def _mode(baseline: Any) -> bool:
            if guard_survey_active and survey_answering(survey_admin) is not False:
                return False  # 回答中/確認不可: 復元しない(ジャーナルに残して再試行)
            _status, payload = survey_admin.set_mode({"mode": baseline if isinstance(baseline, str) and baseline else "customer"})
            return bool(payload.get("ok"))

        def _ocr_mock(_baseline: Any) -> bool:
            _status, payload = survey_admin.set_ocr_mock({"action": "reset"})
            if payload.get("ok"):
                return True
            # OCRテストモードでない等(ラズパイが理由を返した)なら戻すものが無い。届かない場合だけ失敗扱い
            return "届きません" not in str(payload.get("message", ""))
        restorers[RESTORE_SURVEY_MODE] = _mode
        restorers[RESTORE_OCR_MOCK] = _ocr_mock
    return restorers


def ephemeral_reset_post(base_url: str, timeout_s: float = 3.0) -> str:
    """voice_bridgeの POST /ephemeral_reset を呼ぶ。"ok" / "unsupported"(古いvoice_bridge=元々一時上書きが無い) / "error"。"""
    import urllib.error
    import urllib.request

    url = (base_url or "").strip().rstrip("/")
    if not url:
        return "ok"
    req = urllib.request.Request(url + "/ephemeral_reset", data=b"{}", method="POST",
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout_s) as res:
            res.read(65536)
        return "ok"
    except urllib.error.HTTPError as exc:
        code = exc.code
        exc.close()
        return "unsupported" if code in (404, 405) else "error"
    except Exception:  # noqa: BLE001
        return "error"


def reset_remote(base_urls: Iterable[str], done: Optional[set] = None) -> Tuple[List[str], List[str]]:
    """重複を除いた各voice_bridgeへ/ephemeral_resetを送る。(成功した一覧, 失敗した一覧)。doneの分は送らない。"""
    ok: List[str] = []
    ng: List[str] = []
    seen = set(done or ())
    for u in base_urls:
        u = (u or "").strip().rstrip("/")
        if not u or u in seen:
            continue
        seen.add(u)
        (ok if ephemeral_reset_post(u) in ("ok", "unsupported") else ng).append(u)
    return ok, ng


def start_startup_reset(
    eph: EphemeralOverrides,
    restorers: Dict[str, Callable[[Any], bool]],
    base_urls: Iterable[str],
    attempts: int = 60,
    interval_s: float = 5.0,
    sleep: Callable[[float], None] = time.sleep,
    log: Optional[Callable[[str], None]] = None,
) -> threading.Thread:
    """ダッシュボード起動時: 前回の一時上書きを元に戻し、voice_bridgeのメモリ上の一時上書きを破棄する。

    相手(2台目・ラズパイ)がまだ起動していない場合に備え、全部成功するまでinterval_s間隔でattempts回まで再試行する。
    """
    urls = [u for u in base_urls if u]

    def _run() -> None:
        done_urls: set = set()
        for i in range(max(1, attempts)):
            _restored, remaining = eph.restore_pending(restorers)
            ok, ng = reset_remote(urls, done_urls)
            done_urls.update(ok)
            if remaining == 0 and not ng:
                if log:
                    log("一時的な変更(operator)の起動時リセットが完了しました")
                return
            sleep(interval_s)
        if log:
            log("一時的な変更(operator)の起動時リセットを完了できませんでした(相手に届かない)。ジャーナルに残してあります")

    th = threading.Thread(target=_run, name="ephemeral-startup-reset", daemon=True)
    th.start()
    return th
