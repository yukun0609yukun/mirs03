"""状況アナウンス(「探索中です」などの声のお知らせ)のイベント定義表と判定ロジック(純Python、ROS非依存)。

【2026-10-13追加】docs/announcements.md。ロボットの動作には一切影響しない(cmd_vel・ゴールを出さない)。

- イベント定義表: `~/.nokosahen_announce_events.json`(1台目Jetson)。`announce_settings.py`と同じ保存方式
  (原子的置換+`.bak`、不正なファイルは全体を拒否して直前の有効な状態を維持)。
    {"version": 1, "enabled": true,
     "events": {"exploring": {"enabled": true, "text": "探索中です。ご注意ください", "mode": "loop",
                              "interval_sec": 8, "volume": 1.0, "cooldown_sec": 0, "min_active_sec": 2,
                              "quiet_when_person_near": false, "during_survey": false, "during_estop": false}, ...}}
  * ファイルが無い間は全体OFF(従来と完全に同じ=何も鳴らさない)。画面にはおすすめの既定値が出ていて、保存して初めて有効。
  * `enabled`(全体)がfalseなら、個別がONでも何も鳴らさない。
  * `mode`: "loop"=状況が続く間 interval_sec 秒間隔で繰り返す / "once"=始まった時に1回(cooldown_sec 秒は同じイベントを再発話しない)。
  * `min_active_sec`: 状況がこの秒数続いて初めて鳴らす(一瞬の変化で喋らない)。
  * 抑制条件: `quiet_when_person_near`(人が近いと出さない)、`during_survey`(アンケート回答中でも出す)、
    `during_estop`(緊急停止中でも出す)。既定は「アンケート中は出さない」「緊急停止中は出さない(緊急停止のお知らせ自体を除く)」。
- `AnnounceManager`: 状況フラグを受けて、定義表に従い「voice_bridgeへ送る操作(ops)」を決める。ROS・HTTPは持たない。
"""

from __future__ import annotations

import copy
import json
import math
import os
import shutil
import tempfile
import threading
from typing import Any, Dict, List, Optional, Tuple

DEFAULT_FILE = "~/.nokosahen_announce_events.json"
FILE_VERSION = 1

# voice_bridge/announce_engine.py と同じ範囲(test_announce_api.pyが一致を確認する)
TEXT_MAX = 80
INTERVAL_MIN = 2.0
INTERVAL_MAX = 300.0
VOLUME_MAX = 5.0
COOLDOWN_MAX = 3600.0
MIN_ACTIVE_MAX = 60.0
MODES = ("loop", "once")
KEEPALIVE_SEND_SEC = 5.0  # ループ中、voice_bridgeへ「生きている」信号(同じ内容の再送)を送る間隔(2台目の停止判定は15秒)

FIELD_ORDER = ("enabled", "text", "mode", "interval_sec", "volume", "cooldown_sec", "min_active_sec",
               "quiet_when_person_near", "during_survey", "during_estop")
BOOL_FIELDS = ("enabled", "quiet_when_person_near", "during_survey", "during_estop")

# ---- イベント表(コードの固定部分: id・表示名・説明・どの状況フラグで鳴るか・おすすめの既定値) ----
# trigger: 状況フラグ名(announce_node.pyが各トピックから作る。下のFLAGSに説明)。
# urgent: voice_bridgeで発話の待ちを無視して先に鳴らす(緊急停止のみ)。
EVENT_DEFS: Dict[str, Dict[str, Any]] = {
    "exploring": {
        "label": "自立探索中", "trigger": "exploring", "urgent": False,
        "desc": "自動探索(フロンティア探索)で動いている間、数秒おきに繰り返す。周りの人に動いていることを知らせる。",
        "defaults": {"enabled": True, "text": "探索中です。ご注意ください", "mode": "loop", "interval_sec": 8, "volume": 1.0,
                     "cooldown_sec": 0, "min_active_sec": 2, "quiet_when_person_near": False,
                     "during_survey": False, "during_estop": False},
    },
    "backing_up": {
        "label": "後退中", "trigger": "backing_up", "urgent": False,
        "desc": "後ろへ下がっている間(Nav2の回復動作など)。後ろの人に気づいてもらう安全のお知らせ。手動運転中は出さない。",
        "defaults": {"enabled": True, "text": "後ろに下がります", "mode": "loop", "interval_sec": 3, "volume": 1.0,
                     "cooldown_sec": 0, "min_active_sec": 0, "quiet_when_person_near": False,
                     "during_survey": False, "during_estop": False},
    },
    "blocked_waiting": {
        "label": "進めず待機中", "trigger": "blocked", "urgent": False,
        "desc": "人や障害物で進めずに止まっている(近すぎる人・カメラの障害物)状態が続いたとき1回。",
        "defaults": {"enabled": True, "text": "通れません。少々お待ちください", "mode": "once", "interval_sec": 8, "volume": 1.0,
                     "cooldown_sec": 90, "min_active_sec": 4, "quiet_when_person_near": False,
                     "during_survey": False, "during_estop": False},
    },
    "approaching_person": {
        "label": "人に近づくとき", "trigger": "approaching_person", "urgent": False,
        "desc": "呼びかけのために人へ近づき始めたとき1回。",
        "defaults": {"enabled": True, "text": "失礼します", "mode": "once", "interval_sec": 8, "volume": 1.0,
                     "cooldown_sec": 120, "min_active_sec": 0, "quiet_when_person_near": False,
                     "during_survey": False, "during_estop": False},
    },
    "emergency_stop": {
        "label": "緊急停止", "trigger": "estop", "urgent": True,
        "desc": "緊急停止(ハード/ソフト)がかかったとき1回。緊急停止中でもアンケート中でも出せる専用の経路(既定は出す)。",
        "defaults": {"enabled": True, "text": "緊急停止しました", "mode": "once", "interval_sec": 8, "volume": 1.5,
                     "cooldown_sec": 10, "min_active_sec": 0, "quiet_when_person_near": False,
                     "during_survey": True, "during_estop": True},
    },
    "manual_driving": {
        "label": "手動運転中", "trigger": "manual", "urgent": False,
        "desc": "ダッシュボードのWASD・地図クリックで手動運転している間。操作者は近くにいる前提なので既定はOFF。",
        "defaults": {"enabled": False, "text": "手動運転中です", "mode": "once", "interval_sec": 20, "volume": 1.0,
                     "cooldown_sec": 300, "min_active_sec": 1, "quiet_when_person_near": True,
                     "during_survey": False, "during_estop": False},
    },
    "mapping_started": {
        "label": "地図作成の開始", "trigger": "mapping", "urgent": False,
        "desc": "地図作成(SLAM)を始めたとき1回。スタッフ向けなのでお客さんが近いときは出さない。",
        "defaults": {"enabled": True, "text": "地図を作成します", "mode": "once", "interval_sec": 8, "volume": 1.0,
                     "cooldown_sec": 60, "min_active_sec": 0, "quiet_when_person_near": True,
                     "during_survey": False, "during_estop": False},
    },
    "exploration_complete": {
        "label": "探索の完了", "trigger": "exploration_complete", "urgent": False,
        "desc": "自動探索が行ける所を回り終えたとき1回。スタッフ向けなのでお客さんが近いときは出さない。",
        "defaults": {"enabled": True, "text": "探索が完了しました", "mode": "once", "interval_sec": 8, "volume": 1.0,
                     "cooldown_sec": 60, "min_active_sec": 0, "quiet_when_person_near": True,
                     "during_survey": False, "during_estop": False},
    },
    "leaving_boundary": {
        "label": "範囲外・立入禁止", "trigger": "out_of_bounds", "urgent": False,
        "desc": "走行可能範囲の外、または立入禁止エリアに入って自律走行が止まったとき1回。",
        "defaults": {"enabled": True, "text": "範囲の外に出たため止まります", "mode": "once", "interval_sec": 8, "volume": 1.0,
                     "cooldown_sec": 60, "min_active_sec": 2, "quiet_when_person_near": True,
                     "during_survey": False, "during_estop": False},
    },
    "moving_notice": {
        "label": "移動中の注意", "trigger": "moving_notice", "urgent": False,
        "desc": "車輪が実際に動いている間、20秒おきに繰り返す(動き始めにすぐ1回)。対象の動きは、時間割・ピンへの移動、ホーム・開始位置へ戻る、"
                "手動運転、自律探索・地図作成のみ(人への接近・見回し・待機の揺動・回復動作では出さない)。iPadの「移動中です。ご注意ください」画面と同じ判定。"
                "アンケート回答中・呼びかけ中・発話中・緊急停止中は出さない。",
        "defaults": {"enabled": True, "text": "ロボットが移動しています。ご注意ください", "mode": "loop", "interval_sec": 20, "volume": 1.0,
                     "cooldown_sec": 0, "min_active_sec": 0, "quiet_when_person_near": False,
                     "during_survey": False, "during_estop": False},
    },
}

# 状況フラグ(announce_node.pyが作る)の説明。画面にも出す。
FLAGS: Dict[str, str] = {
    "exploring": "自動探索が動いている(frontier_explorerのstatusがexploring)",
    "backing_up": "後退している(/cmd_velが後ろ向きに一定時間続く。手動運転中は除く)",
    "blocked": "近すぎる人/カメラの障害物で自律走行がホールド中(NEAR_PERSON_HOLD・CAMERA_OBSTACLE_HOLD)",
    "approaching_person": "人へ接近中(APPROACH_PERSON)",
    "estop": "ハード/ソフト緊急停止中",
    "manual": "手動運転中",
    "mapping": "地図作成(SLAM)中",
    "exploration_complete": "自動探索が完了した直後",
    "out_of_bounds": "走行可能範囲の外/立入禁止エリアで停止中",
    "moving_notice": "移動中の注意を出す条件(車輪が動いていて、時間割・ピン/ホームへ戻る/手動運転/探索・地図作成のどれか。moving_notice.pyが判定し/nokosahen/moving_noticeで届く)",
}
CONTEXT_KEYS = ("survey", "person_near", "estop")  # フラグとは別に、抑制の判定に使う状況

EVENT_IDS = tuple(EVENT_DEFS)

# 【2026-10-23, 動作のかぶり調査 B9】営業の呼びかけ(behavior_director→announce_context→voice_node の /speak)を出さない状況。
# announce_node の状況アナウンスが対象にする(または抑制の判断に使う)状況と同じ一覧: アンケート中(context["survey"])、
# 近接停止(/nokosahen/near_person_stop。アナウンス側では blocked=NEAR_PERSON_HOLD)、地図作成(flags["mapping"])、探索中(flags["exploring"])。
# ここに載せた名前で director が判定する(表の外に別の一覧を持たない)。
SALES_SUPPRESS_SITUATIONS = ("survey", "near_person_stop", "mapping", "exploring")


def sales_speech_blocked(situations: Dict[str, bool]) -> List[str]:
    """営業発話を止める状況の名前(SALES_SUPPRESS_SITUATIONS のうち立っているもの)。空なら出してよい。"""
    return [name for name in SALES_SUPPRESS_SITUATIONS if situations.get(name)]


def _reject_constant(name):
    raise ValueError(f"JSONに{name}(NaN/Infinity)は使えません")


def default_event(event_id: str) -> Dict[str, Any]:
    return copy.deepcopy(EVENT_DEFS[event_id]["defaults"])


def _num(value) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    v = float(value)
    return v if math.isfinite(v) else None


def _norm_num(v: float):
    return int(v) if v == int(v) else v


def validate_event_fields(event_id: str, fields: Any, base: Optional[Dict[str, Any]] = None) -> Tuple[Dict[str, Any], List[str]]:
    """1イベントの項目(部分でも可)を検証する。(正規化した項目dict, エラー一覧)。丸めずに範囲外・型違い・未知の項目は拒否。
    baseが与えられれば、その上に重ねた結果の整合性(textが空でないなど)も見る。"""
    label = EVENT_DEFS[event_id]["label"] if event_id in EVENT_DEFS else event_id
    if not isinstance(fields, dict):
        return {}, [f"{label}: 項目はJSONオブジェクトにしてください"]
    errors: List[str] = []
    out: Dict[str, Any] = {}
    for k, v in fields.items():
        if k not in FIELD_ORDER:
            errors.append(f"{label}: 未知の項目です: {k}")
            continue
        if k in BOOL_FIELDS:
            if not isinstance(v, bool):
                errors.append(f"{label}: {k}は true/false で指定してください")
            else:
                out[k] = v
        elif k == "text":
            if not isinstance(v, str):
                errors.append(f"{label}: 言う内容は文字列で指定してください")
                continue
            t = v.strip()
            if not t:
                errors.append(f"{label}: 言う内容が空です")
            elif len(t) > TEXT_MAX:
                errors.append(f"{label}: 言う内容は{TEXT_MAX}文字以内にしてください")
            elif any(ord(c) < 32 or ord(c) == 127 for c in t):
                errors.append(f"{label}: 言う内容に制御文字は使えません")
            else:
                out[k] = t
        elif k == "mode":
            if v not in MODES:
                errors.append(f"{label}: modeは loop か once です")
            else:
                out[k] = v
        else:
            lo, hi, name = {
                "interval_sec": (INTERVAL_MIN, INTERVAL_MAX, "間隔(秒)"),
                "volume": (0.0, VOLUME_MAX, "音量"),
                "cooldown_sec": (0.0, COOLDOWN_MAX, "クールダウン(秒)"),
                "min_active_sec": (0.0, MIN_ACTIVE_MAX, "続いてから鳴らすまで(秒)"),
            }[k]
            n = _num(v)
            if n is None:
                errors.append(f"{label}: {name}は有限の数値で指定してください")
            elif n < lo or n > hi:
                errors.append(f"{label}: {name}は{lo:g}〜{hi:g}の範囲で指定してください")
            else:
                out[k] = _norm_num(n)
    return out, errors


def validate_document(doc: Any) -> Tuple[Optional[Dict[str, Any]], List[str]]:
    """ファイル内容/保存リクエスト(dict)を検証する。(正規化したdict {"enabled"?, "events": {id: 項目}}, エラー一覧)。
    未知のキー・未知のイベントidはエラー(黙って無視しない)。"""
    if not isinstance(doc, dict):
        return None, ["設定はJSONオブジェクトにしてください"]
    errors: List[str] = []
    clean: Dict[str, Any] = {}
    for k in doc:
        if k not in ("version", "description", "enabled", "events"):
            errors.append(f"未知のキー: {k}")
    if "enabled" in doc:
        if not isinstance(doc["enabled"], bool):
            errors.append("全体のenabledは true/false で指定してください")
        else:
            clean["enabled"] = doc["enabled"]
    if "events" in doc:
        evs = doc["events"]
        if not isinstance(evs, dict):
            errors.append("eventsはJSONオブジェクトにしてください")
        else:
            clean_events: Dict[str, Any] = {}
            for eid, fields in evs.items():
                if eid not in EVENT_DEFS:
                    errors.append(f"未知のイベントです: {eid}")
                    continue
                f, e = validate_event_fields(eid, fields)
                errors.extend(e)
                clean_events[eid] = f
            clean["events"] = clean_events
    if errors:
        return None, errors
    return clean, []


def merged_config(clean: Optional[Dict[str, Any]], file_exists: bool) -> Dict[str, Any]:
    """検証済みの文書をおすすめの既定値に重ねた、全イベントがそろった設定を返す。
    ファイルが無い間は全体OFF(従来どおり何も鳴らさない)。"""
    events = {eid: default_event(eid) for eid in EVENT_DEFS}
    enabled = False
    if clean:
        enabled = bool(clean.get("enabled", False)) if file_exists else False
        for eid, f in (clean.get("events") or {}).items():
            events[eid].update(f)
    return {"enabled": enabled, "events": events}


def file_sig(path: str) -> str:
    try:
        st = os.stat(path)
    except OSError:
        return "missing"
    return f"{st.st_mtime_ns}-{st.st_size}"


def event_meta() -> Dict[str, Any]:
    """画面用の固定情報(表示名・説明・おすすめ既定値・範囲)。"""
    return {
        "events": [
            {"id": eid, "label": d["label"], "desc": d["desc"], "trigger": d["trigger"], "urgent": d["urgent"],
             "trigger_desc": FLAGS[d["trigger"]], "defaults": default_event(eid)}
            for eid, d in EVENT_DEFS.items()
        ],
        "limits": {"text_max": TEXT_MAX, "interval": [INTERVAL_MIN, INTERVAL_MAX], "volume": [0.0, VOLUME_MAX],
                   "cooldown": [0.0, COOLDOWN_MAX], "min_active": [0.0, MIN_ACTIVE_MAX]},
    }


class AnnounceEventsStore:
    """ダッシュボード側: 読み取りと検証つき保存(スレッドセーフ)。"""

    def __init__(self, path: str = DEFAULT_FILE) -> None:
        self.path = os.path.expanduser(path or DEFAULT_FILE)
        self._lock = threading.Lock()

    @property
    def backup_path(self) -> str:
        return self.path + ".bak"

    def read(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "path": self.path, "exists": False, "sig": file_sig(self.path), "valid": True, "errors": [],
            **merged_config(None, False),
        }
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                text = f.read()
        except FileNotFoundError:
            return out
        except (OSError, UnicodeDecodeError) as exc:
            out["exists"] = os.path.exists(self.path)
            out["valid"], out["errors"] = False, [f"ファイルを読めません: {exc}"]
            return out
        out["exists"] = True
        try:
            doc = json.loads(text, parse_constant=_reject_constant)
        except ValueError as exc:
            out["valid"], out["errors"] = False, [f"JSONとして読めません: {exc}"]
            return out
        clean, errors = validate_document(doc)
        if errors or clean is None:
            out["valid"], out["errors"] = False, errors
            return out
        out.update(merged_config(clean, True))
        return out

    def save(self, update: Any) -> Dict[str, Any]:
        """部分更新({"enabled"?, "events": {id: {変えた項目だけ}}})を検証して保存する。
        現在の有効な内容(無ければおすすめの既定値)に重ね、全イベントを書き出す。例外は投げない。"""
        clean, errors = validate_document(update)
        if errors or clean is None:
            return {"ok": False, "kind": "invalid", "errors": errors}
        if not clean:
            return {"ok": False, "kind": "invalid", "errors": ["変更する項目がありません"]}
        with self._lock:
            cur = self.read()
            if not cur["valid"]:
                # 壊れたファイルは上書きで直せるが、無言で捨てない(.bakに退避される)。現在値はおすすめ既定から始める
                cur = {**cur, **merged_config(None, False)}
            events = cur["events"]
            for eid, f in (clean.get("events") or {}).items():
                events[eid].update(f)
            # 重ねた結果の整合性(部分更新の項目が個別には正しくても、全体として成立するか)
            for eid, f in events.items():
                _, e = validate_event_fields(eid, f)
                if e:
                    return {"ok": False, "kind": "invalid", "errors": e}
            enabled = clean.get("enabled", cur["enabled"] if cur["exists"] else True)
            doc = {"version": FILE_VERSION, "enabled": enabled,
                   "events": {eid: {k: events[eid][k] for k in FIELD_ORDER} for eid in EVENT_DEFS}}
            text = json.dumps(doc, ensure_ascii=False, indent=2) + "\n"
            directory = os.path.dirname(self.path) or "."
            tmp_path = bak_tmp = None
            try:
                os.makedirs(directory, exist_ok=True)
                fd, tmp_path = tempfile.mkstemp(prefix=".nokosahen_announce_events_", suffix=".tmp", dir=directory)
                with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as f:
                    f.write(text)
                    f.flush()
                    os.fsync(f.fileno())
                backed_up = False
                if os.path.exists(self.path):
                    bak_tmp = self.backup_path + ".tmp"
                    shutil.copy2(self.path, bak_tmp)
                    os.replace(bak_tmp, self.backup_path)
                    bak_tmp = None
                    backed_up = True
                os.replace(tmp_path, self.path)
                tmp_path = None
            except OSError as exc:
                return {"ok": False, "kind": "io",
                        "errors": [f"ファイルを書き込めませんでした(元のファイルは変更されていません): {exc}"]}
            finally:
                for p in (tmp_path, bak_tmp):
                    if p:
                        try:
                            os.remove(p)
                        except OSError:
                            pass
        return {"ok": True, "kind": None, "errors": [], "backed_up": backed_up, "sig": file_sig(self.path)}


class AnnounceEventsReader:
    """announce_node側: 内容が変わったときだけ読み直す(1Hzで呼ぶ)。例外は投げない。
    不正なファイルは直前の有効な設定を維持する。ファイルが無ければ全体OFF。"""

    def __init__(self, path: str = DEFAULT_FILE) -> None:
        self.path = os.path.expanduser(path or DEFAULT_FILE)
        self._sig: Optional[str] = None
        self.config: Dict[str, Any] = merged_config(None, False)
        self.load_error = ""

    def poll(self) -> Tuple[Dict[str, Any], str, bool]:
        """(現在の設定, ログに出すメッセージ, 変化したか)。"""
        try:
            with open(self.path, "rb") as f:
                raw = f.read(262144)
            sig = "c:" + str(hash(raw))
        except FileNotFoundError:
            sig = "missing"
        except OSError as exc:
            sig = "err:" + str(exc)
        if sig == self._sig:
            return self.config, "", False
        self._sig = sig
        if sig == "missing":
            changed = self.config.get("enabled") or bool(self.load_error)
            self.config, self.load_error = merged_config(None, False), ""
            return self.config, ("状況アナウンスの設定ファイルが無いため、全体OFFです" if changed else ""), bool(changed)
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                doc = json.loads(f.read(), parse_constant=_reject_constant)
        except (OSError, ValueError) as exc:
            self.load_error = f"状況アナウンスの設定ファイルを読めません(直前の設定を維持): {exc}"
            return self.config, self.load_error, False
        clean, errors = validate_document(doc)
        if errors or clean is None:
            self.load_error = "状況アナウンスの設定ファイルが不正です(直前の設定を維持): " + " / ".join(errors)
            return self.config, self.load_error, False
        self.load_error = ""
        self.config = merged_config(clean, True)
        return self.config, "状況アナウンスの設定を読み込みました", True


class _EventState:
    def __init__(self) -> None:
        self.active_since: Optional[float] = None  # フラグがtrueになった時刻(falseで解除)
        self.fired = False  # 今回のフラグtrue区間で、onceを鳴らし済みか
        self.last_fired: Optional[float] = None
        self.loop_on = False
        self.last_sent = 0.0
        self.sent_sig: Optional[tuple] = None
        self.reason = ""  # 今鳴らさない理由(画面表示用)


class AnnounceManager:
    """状況フラグ→voice_bridgeへ送る操作。時刻は引数で受け取る(テスト容易、ROS非依存)。

    `update(flags, context, now)` が返す ops の要素:
      {"op": "loop", "id", "text", "interval_sec", "volume"}  # 開始・設定変更・「生きている」信号の再送(同じ形)
      {"op": "loop_stop", "id"}
      {"op": "once", "id", "text", "volume", "urgent"}
    flags: EVENT_DEFSのtriggerに使う状況フラグ(bool)。context: survey/person_near/estop(bool、抑制の判定用)。
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None, keepalive_send_sec: float = KEEPALIVE_SEND_SEC) -> None:
        self.config = config or merged_config(None, False)
        self.keepalive_send_sec = keepalive_send_sec
        self._st: Dict[str, _EventState] = {eid: _EventState() for eid in EVENT_DEFS}

    def set_config(self, config: Dict[str, Any]) -> None:
        self.config = config

    def update(self, flags: Dict[str, bool], context: Dict[str, bool], now: float) -> List[Dict[str, Any]]:
        ops: List[Dict[str, Any]] = []
        master = bool(self.config.get("enabled"))
        for eid, d in EVENT_DEFS.items():
            cfg = self.config["events"][eid]
            st = self._st[eid]
            raw = bool(flags.get(d["trigger"], False))
            if raw:
                if st.active_since is None:
                    st.active_since = now
            else:
                st.active_since = None
                st.fired = False
            reason = ""
            if not master:
                reason = "全体OFF"
            elif not cfg["enabled"]:
                reason = "OFF"
            elif not raw:
                reason = ""
            elif context.get("survey") and not cfg["during_survey"]:
                reason = "アンケート回答中"
            elif context.get("estop") and not cfg["during_estop"]:
                reason = "緊急停止中"
            elif context.get("person_near") and cfg["quiet_when_person_near"]:
                reason = "人が近い"
            elif now - (now if st.active_since is None else st.active_since) < float(cfg["min_active_sec"]):
                reason = "続くのを待機中"
            st.reason = reason
            active = raw and not reason
            if cfg["mode"] == "loop":
                if active:
                    sig = (cfg["text"], float(cfg["interval_sec"]), float(cfg["volume"]))
                    if (not st.loop_on) or sig != st.sent_sig or now - st.last_sent >= self.keepalive_send_sec:
                        ops.append({"op": "loop", "id": eid, "text": cfg["text"],
                                    "interval_sec": float(cfg["interval_sec"]), "volume": float(cfg["volume"])})
                        st.loop_on, st.sent_sig, st.last_sent = True, sig, now
                elif st.loop_on:
                    ops.append({"op": "loop_stop", "id": eid})
                    st.loop_on, st.sent_sig = False, None
            else:
                if st.loop_on:  # loopからonceへ設定が変わった
                    ops.append({"op": "loop_stop", "id": eid})
                    st.loop_on, st.sent_sig = False, None
                if active and not st.fired:
                    cool = float(cfg["cooldown_sec"])
                    if st.last_fired is not None and now - st.last_fired < cool:
                        st.reason = "クールダウン中"
                    else:
                        ops.append({"op": "once", "id": eid, "text": cfg["text"], "volume": float(cfg["volume"]),
                                    "urgent": bool(d["urgent"])})
                        st.fired, st.last_fired = True, now
        return ops

    def stop_all(self) -> List[Dict[str, Any]]:
        """動いているループをすべて止める操作(ノード終了時など)。"""
        ops = []
        for eid, st in self._st.items():
            if st.loop_on:
                ops.append({"op": "loop_stop", "id": eid})
                st.loop_on, st.sent_sig = False, None
        return ops

    def status(self) -> Dict[str, Any]:
        return {
            "enabled": bool(self.config.get("enabled")),
            "events": {eid: {"loop_on": st.loop_on, "flag_active": st.active_since is not None, "reason": st.reason,
                             "last_fired": st.last_fired}
                       for eid, st in self._st.items()},
        }


# ---- 状況フラグの組み立て(announce_node.pyが購読したトピックを渡す。ROS非依存・時刻は引数) ----
BLOCKED_MODES = frozenset({"NEAR_PERSON_HOLD", "CAMERA_OBSTACLE_HOLD"})
OUT_OF_BOUNDS_MODES = frozenset({"OUT_OF_BOUNDS_HOLD", "KEEPOUT_ZONE_HOLD"})
APPROACH_MODES = frozenset({"APPROACH_PERSON"})


class SituationBuilder:
    """既存トピックの値(behavior_command・survey_active・crowd_state・map_building_active・
    frontier_explorer/status・/cmd_vel)から、状況フラグ(FLAGS)と抑制用の状況(CONTEXT_KEYS)を作る。
    どの値も一定時間(stale_sec)届かなければ「無い」扱い(送り元が落ちたまま鳴りっぱなしにならない)。
    behavior_director.pyは編集せず、director が既に配信している /nokosahen/behavior_command を読むだけ。"""

    def __init__(self, stale_sec: float = 3.0, frontier_stale_sec: float = 10.0, back_speed_mps: float = 0.03,
                 back_hold_sec: float = 0.6, back_release_sec: float = 0.8, complete_hold_sec: float = 5.0) -> None:
        self.stale_sec = stale_sec
        self.frontier_stale_sec = frontier_stale_sec
        self.back_speed_mps = back_speed_mps
        self.back_hold_sec = back_hold_sec
        self.back_release_sec = back_release_sec
        self.complete_hold_sec = complete_hold_sec
        self._cmd: Dict[str, Any] = {}
        self._cmd_t: Optional[float] = None
        self._survey = False
        self._survey_t: Optional[float] = None
        self._person_target = False
        self._person_t: Optional[float] = None
        self._near_stop = False
        self._near_stop_t: Optional[float] = None
        self._map_building = False
        self._map_t: Optional[float] = None
        self._frontier: Dict[str, Any] = {}
        self._frontier_t: Optional[float] = None
        self._back_since: Optional[float] = None
        self._back_last: Optional[float] = None
        self._complete_until = 0.0
        self._prev_frontier_status = ""
        self._moving: Dict[str, Any] = {}
        self._moving_t: Optional[float] = None

    def on_behavior_command(self, d: Any, now: float) -> None:
        if isinstance(d, dict):
            self._cmd, self._cmd_t = d, now

    def on_survey_active(self, active: bool, now: float) -> None:
        self._survey, self._survey_t = bool(active), now

    def on_crowd_state(self, d: Any, now: float) -> None:
        if isinstance(d, dict):
            self._person_target, self._person_t = isinstance(d.get("person_target"), dict), now

    def on_near_person_stop(self, active: bool, now: float) -> None:
        self._near_stop, self._near_stop_t = bool(active), now

    def on_map_building_active(self, active: bool, now: float) -> None:
        self._map_building, self._map_t = bool(active), now

    def on_frontier_status(self, d: Any, now: float) -> None:
        if not isinstance(d, dict):
            return
        status = str(d.get("status", "")) if d.get("enabled") else ""
        if status == "complete" and self._prev_frontier_status in ("exploring", "paused"):
            self._complete_until = now + self.complete_hold_sec  # 探索中→完了の変化だけ(起動直後の古いcompleteでは鳴らさない)
        self._prev_frontier_status = status
        self._frontier, self._frontier_t = d, now

    def on_moving_notice(self, d: Any, now: float) -> None:
        """ダッシュボードが判定した「移動中の注意」(/nokosahen/moving_notice。moving_notice.py)。判定は1か所に集め、ここでは読むだけ。"""
        if isinstance(d, dict):
            self._moving, self._moving_t = d, now

    def on_cmd_vel(self, linear_x: float, now: float) -> None:
        if linear_x < -self.back_speed_mps:
            if self._back_since is None:
                self._back_since = now
            self._back_last = now
        elif linear_x >= -0.01 and self._back_last is not None and now - self._back_last > self.back_release_sec:
            self._back_since = None

    def _fresh(self, t: Optional[float], now: float, limit: Optional[float] = None) -> bool:
        return t is not None and now - t <= (self.stale_sec if limit is None else limit)

    def flags(self, now: float) -> Dict[str, bool]:
        cmd = self._cmd if self._fresh(self._cmd_t, now) else {}
        mode = str(cmd.get("mode", ""))
        manual = bool(cmd.get("manual_override", False))
        fr = self._frontier if self._fresh(self._frontier_t, now, self.frontier_stale_sec) else {}
        backing = (
            self._back_since is not None and self._back_last is not None
            and now - self._back_last <= self.back_release_sec
            and now - self._back_since >= self.back_hold_sec and not manual
        )
        return {
            "exploring": bool(fr.get("enabled")) and fr.get("status") == "exploring",
            "backing_up": backing,
            "blocked": mode in BLOCKED_MODES,
            "approaching_person": mode in APPROACH_MODES,
            "estop": bool(cmd.get("estop_active", False)),
            "manual": manual,
            "mapping": self._map_building and self._fresh(self._map_t, now),
            "exploration_complete": now < self._complete_until,
            "out_of_bounds": mode in OUT_OF_BOUNDS_MODES,
            # 届かない/古い(ダッシュボード停止・通信断)なら鳴らさない。bool以外(文字列など)は真と扱わない
            "moving_notice": self._moving.get("sound") is True and self._fresh(self._moving_t, now),
        }

    def context(self, now: float) -> Dict[str, bool]:
        cmd = self._cmd if self._fresh(self._cmd_t, now) else {}
        mode = str(cmd.get("mode", ""))
        return {
            "survey": self._survey and self._fresh(self._survey_t, now),
            "estop": bool(cmd.get("estop_active", False)),
            "person_near": (
                (self._person_target and self._fresh(self._person_t, now))
                or (self._near_stop and self._fresh(self._near_stop_t, now))
                or mode == "NEAR_PERSON_HOLD"
            ),
        }
