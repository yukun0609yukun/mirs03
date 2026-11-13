"""ルールファイル(behavior_rules)の「検証つき保存」と「次の24時間の予定プレビュー」(純Python、ROS非依存)。

ダッシュボードのルール編集ページ(/behavior_rules)が使う。ロボットを直接動かすことは一切ない。
検証は behavior_rules.py 自身の関数(parse_rules_text / _parse_rule)をそのまま使うので、
behavior_director が読むときの判定と食い違わない(ここで通ったものだけが保存される)。

保存の安全策:
- 1つでも不正があれば何も書かない(不正なファイルは絶対に作らない)。
- 一時ファイルに書いてfsyncし、os.replaceで置き換える(途中で落ちても元のファイルは壊れない)。
- 置き換える前に、直前のファイルを `<path>.bak` へコピーして残す。
- 編集画面を開いた後に別の誰かがファイルを書き換えていたら(sigが違う)、上書きせず競合として返す。
"""

from __future__ import annotations

import datetime
import json
import os
import shutil
import tempfile
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from . import behavior_rules as br

EXAMPLE_FILE_NAME = "behavior_rules.example.json"
MAX_BODY_BYTES = 512 * 1024


# --------------------------------------------------------------------------
# 検証
# --------------------------------------------------------------------------


def _reject_constant(name: str):
    raise ValueError(f"JSONに{name}(NaN/Infinity)は使えません")


def parse_json_strict(raw: bytes) -> Tuple[Optional[Any], Optional[str]]:
    """NaN/Infinityを含むJSONを拒否する厳格なパース。(値, エラー文)を返す。"""
    try:
        return json.loads(raw.decode("utf-8"), parse_constant=_reject_constant), None
    except UnicodeDecodeError:
        return None, "本文がUTF-8ではありません"
    except ValueError as exc:
        return None, f"JSONとして読めません: {exc}"


def validate_document(doc: Any) -> Tuple[List[str], Optional[br.RuleSet], Optional[str]]:
    """ルール文書(dict)を検証する。

    戻り値: (エラー一覧[日本語], RuleSet, 保存用のJSONテキスト)。エラーが1つでもあれば
    RuleSet/テキストはNone。ルールごとに独立して検証するので、複数のルールの誤りを
    まとめて返せる(behavior_rules.parse_rules_textは最初の1件で止まるため)。
    最後に、保存するテキスト全体を parse_rules_text で通して最終確認する(これが決定権を持つ)。
    """
    if not isinstance(doc, dict):
        return ["トップレベルはオブジェクト({\"rules\": [...]})にしてください"], None, None
    errors: List[str] = []

    rules = doc.get("rules", [])
    top = {k: v for k, v in doc.items() if k != "rules"}
    top["rules"] = []
    try:
        br.parse_rules_text(json.dumps(top, ensure_ascii=False, allow_nan=False))
    except br.RuleError as exc:
        errors.append(str(exc))
    except (ValueError, TypeError) as exc:
        errors.append(f"JSONにできない値が含まれています(NaN/Infinity等): {exc}")

    if not isinstance(rules, list):
        errors.append("rulesは配列にしてください")
    else:
        if len(rules) > br.MAX_RULES:
            errors.append(f"ルールが多すぎます(最大{br.MAX_RULES}件)")
        seen: Dict[str, int] = {}
        for i, raw in enumerate(rules):
            try:
                rule = br._parse_rule(raw, i)  # noqa: SLF001 (同一パッケージの検証関数をそのまま使う)
            except br.RuleError as exc:
                errors.append(str(exc))
                continue
            except Exception as exc:  # noqa: BLE001
                errors.append(f"rules[{i}]: 想定外の形式です: {exc}")
                continue
            if rule.id in seen:
                errors.append(f"idが重複しています: {rule.id}(rules[{seen[rule.id]}]とrules[{i}])")
            else:
                seen[rule.id] = i
    if errors:
        return errors, None, None

    try:
        text = json.dumps(doc, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    except (ValueError, TypeError) as exc:
        return [f"JSONにできない値が含まれています(NaN/Infinity等): {exc}"], None, None
    try:
        ruleset = br.parse_rules_text(text)
    except br.RuleError as exc:
        return [str(exc)], None, None
    return [], ruleset, text


# --------------------------------------------------------------------------
# ファイルの読み書き
# --------------------------------------------------------------------------


def file_sig(path: str) -> str:
    """競合検知用の指紋。ファイルが無ければ "missing"。"""
    try:
        st = os.stat(path)
    except OSError:
        return "missing"
    return f"{st.st_mtime_ns}-{st.st_size}"


class RulesFileStore:
    """ルールファイル1つの読み書き。スレッドセーフ(保存は直列化)。"""

    def __init__(self, path: str) -> None:
        self.path = os.path.expanduser(path or br.DEFAULT_RULES_FILE)
        self._lock = threading.Lock()

    @property
    def backup_path(self) -> str:
        return self.path + ".bak"

    def read(self) -> Dict[str, Any]:
        """現在のファイル内容(GET用)。壊れていても例外は投げず、理由を返す。"""
        out: Dict[str, Any] = {
            "path": self.path,
            "backup_path": self.backup_path,
            "backup_exists": os.path.exists(self.backup_path),
            "exists": False,
            "sig": file_sig(self.path),
            "document": None,
            "valid": True,
            "errors": [],
            "raw_text": None,
        }
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                text = f.read()
        except FileNotFoundError:
            return out
        except (OSError, UnicodeDecodeError) as exc:
            out["exists"] = os.path.exists(self.path)
            out["valid"] = False
            out["errors"] = [f"ファイルを読めません: {exc}"]
            return out
        out["exists"] = True
        try:
            doc = json.loads(text, parse_constant=_reject_constant)
        except ValueError as exc:
            out["valid"] = False
            out["errors"] = [f"JSONとして読めません: {exc}"]
            out["raw_text"] = text[:20000]
            return out
        errors, _rs, _text = validate_document(doc)
        out["document"] = doc if isinstance(doc, dict) else None
        out["valid"] = not errors
        out["errors"] = errors
        if not isinstance(doc, dict):
            out["raw_text"] = text[:20000]
        return out

    def load_ruleset(self) -> Tuple[Optional[br.RuleSet], List[str]]:
        """保存済みファイルをRuleSetにする。ファイル無しは空のRuleSet。不正ならNone+理由。"""
        state = self.read()
        if not state["exists"]:
            return br.RuleSet(), []
        if not state["valid"]:
            return None, list(state["errors"])
        errors, rs, _ = validate_document(state["document"])
        return rs, errors

    def save(self, doc: Any, expected_sig: Optional[str] = None) -> Dict[str, Any]:
        """検証→バックアップ→原子的に置き換え。結果を dict で返す(例外は投げない)。

        ok: 保存できたか / kind: "invalid"(検証NG) | "conflict" | "io"(書き込み失敗) | None
        """
        errors, ruleset, text = validate_document(doc)
        if errors:
            return {"ok": False, "kind": "invalid", "errors": errors}
        assert ruleset is not None and text is not None
        with self._lock:
            current = file_sig(self.path)
            if expected_sig is not None and expected_sig != current:
                return {
                    "ok": False,
                    "kind": "conflict",
                    "errors": [
                        "編集を始めた後にルールファイルが別の操作で書き換えられました。"
                        "上書きを避けるため保存しませんでした。ページを再読み込みして最新の内容から編集し直してください。"
                    ],
                    "sig": current,
                }
            directory = os.path.dirname(self.path) or "."
            tmp_path = None
            bak_tmp = None
            try:
                os.makedirs(directory, exist_ok=True)
                fd, tmp_path = tempfile.mkstemp(prefix=".nokosahen_rules_", suffix=".tmp", dir=directory)
                with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as f:
                    f.write(text)
                    f.flush()
                    os.fsync(f.fileno())
                backed_up = False
                if os.path.exists(self.path):
                    # 直前のファイルを残す(コピー→置換で、.bak自体も途中状態にならない)
                    bak_tmp = self.backup_path + ".tmp"
                    shutil.copy2(self.path, bak_tmp)
                    os.replace(bak_tmp, self.backup_path)
                    bak_tmp = None
                    backed_up = True
                os.replace(tmp_path, self.path)
                tmp_path = None
            except OSError as exc:
                return {
                    "ok": False,
                    "kind": "io",
                    "errors": [f"ファイルを書き込めませんでした(元のファイルは変更されていません): {exc}"],
                }
            finally:
                for p in (tmp_path, bak_tmp):
                    if p:
                        try:
                            os.remove(p)
                        except OSError:
                            pass
            enabled = sum(1 for r in ruleset.rules if r.enabled)
            return {
                "ok": True,
                "kind": None,
                "errors": [],
                "sig": file_sig(self.path),
                "backed_up": backed_up,
                "rule_count": len(ruleset.rules),
                "enabled_rule_count": enabled,
            }


# --------------------------------------------------------------------------
# 例ファイル
# --------------------------------------------------------------------------


def find_example_file(explicit: str = "") -> Optional[str]:
    """config/behavior_rules.example.json の場所を探す(明示指定 → ament share → リポジトリ内)。"""
    candidates: List[str] = []
    if explicit:
        candidates.append(os.path.expanduser(explicit))
    try:
        from ament_index_python.packages import get_package_share_directory  # type: ignore

        candidates.append(
            os.path.join(get_package_share_directory("nokosahen_bringup"), "config", EXAMPLE_FILE_NAME)
        )
    except Exception:  # noqa: BLE001 - ROS無し/未インストールでも動く
        pass
    here = Path(__file__).resolve()
    # <repo>/ros2_ws/src/nokosahen_core/nokosahen_core/ -> <repo>/ros2_ws/src/nokosahen_bringup/config/
    candidates.append(str(here.parents[2] / "nokosahen_bringup" / "config" / EXAMPLE_FILE_NAME))
    for c in candidates:
        if c and os.path.isfile(c):
            return c
    return None


def load_example(explicit: str = "") -> Tuple[Optional[dict], Optional[str], Optional[str]]:
    """(例の文書, 読んだパス, エラー文)。例ファイル自体も検証してから返す。"""
    path = find_example_file(explicit)
    if path is None:
        return None, None, "例ファイル(behavior_rules.example.json)が見つかりません"
    try:
        with open(path, "r", encoding="utf-8") as f:
            doc = json.loads(f.read(), parse_constant=_reject_constant)
    except (OSError, ValueError) as exc:
        return None, path, f"例ファイルを読めません: {exc}"
    errors, _rs, _t = validate_document(doc)
    if errors:
        return None, path, "例ファイルが検証に通りません: " + " / ".join(errors)
    return doc, path, None


# --------------------------------------------------------------------------
# 次の24時間の予定プレビュー
# --------------------------------------------------------------------------


def is_situational(rule: br.Rule) -> bool:
    """人数・人物・残数・無人時間など、時刻だけでは決まらない条件を持つか。"""
    return bool(
        rule.crowd_level
        or rule.person_present is not None
        or rule.remaining_ratio_below is not None
        or rule.remaining_ratio_above is not None
        or rule.menu is not None
        or rule.idle_seconds_above is not None
    )


def situation_text(rule: br.Rule) -> str:
    parts: List[str] = []
    if rule.crowd_level:
        parts.append("混雑度が " + "/".join(rule.crowd_level))
    if rule.person_present is not None:
        parts.append("人物あり" if rule.person_present else "人物なし")
    if rule.menu:
        parts.append(f"メニュー「{rule.menu}」")
    if rule.remaining_ratio_below is not None:
        parts.append(f"残数比<{rule.remaining_ratio_below:g}")
    if rule.remaining_ratio_above is not None:
        parts.append(f"残数比>{rule.remaining_ratio_above:g}")
    if rule.idle_seconds_above is not None:
        parts.append(f"人物不在{rule.idle_seconds_above:g}秒超")
    return " かつ ".join(parts)


class _PreviewEngine(br.RuleEngine):
    """プレビュー用。1分刻みで1440回評価するので、評価ごとの48時間先探索は省く。"""

    def _estimate_next_change(self, enabled, dt):  # noqa: D401
        return None


def _rule_brief(rule: br.Rule) -> Dict[str, Any]:
    return {
        "id": rule.id,
        "name": rule.name,
        "priority": rule.priority,
        "policy": "+".join(rule.policies),
        "params": dict(rule.params),
    }


def build_preview(
    ruleset: br.RuleSet, now: datetime.datetime, hours: int = 24
) -> Dict[str, Any]:
    """保存済みルールから、今から`hours`時間の予定(1分刻み→連続区間にまとめる)を計算する。

    - 「確定」: 時刻/曜日/日付の条件だけで決まるルール。behavior_rules.RuleEngine.evaluate を
      模擬時計で1分ずつ進めて求める(最小保持時間も本番と同じ)。
    - 「条件次第」: 人数・人物・残数・無人時間の条件を持つルールで、その時間帯に時刻条件が
      成立していて、確定ルールより優先されうるもの。これらが実際に発動するかは予測できない
      (混雑や在庫は未来が分からないため)。予定は「時刻だけ」の見通し。
    """
    hours = max(1, min(int(hours), 48))
    start = now.replace(second=0, microsecond=0)
    enabled = [r for r in ruleset.rules if r.enabled]
    ordered = sorted(enabled, key=lambda r: -r.priority)  # 安定ソート=同順位は記載順
    order_idx = {r.id: i for i, r in enumerate(ordered)}
    timed_rules = tuple(r for r in enabled if not is_situational(r))
    situational_rules = [r for r in ordered if is_situational(r)]

    sim = {"dt": start, "m": 0.0}
    engine = _PreviewEngine("", clock=lambda: sim["dt"], mono=lambda: sim["m"])
    engine.set_ruleset(
        br.RuleSet(rules=timed_rules, min_hold_seconds=ruleset.min_hold_seconds)
    )
    by_id = {r.id: r for r in enabled}

    segments: List[Dict[str, Any]] = []
    cur_key = None
    for i in range(hours * 60):
        t = start + datetime.timedelta(minutes=i)
        sim["dt"] = t
        sim["m"] = i * 60.0
        st = engine.evaluate(br.Context())
        definite_id = st.active_rule_id
        maybe = [
            r
            for r in situational_rules
            if r.time_part_matches(t)
            and (definite_id is None or order_idx[r.id] < order_idx[definite_id])
        ]
        key = (definite_id, tuple(r.id for r in maybe))
        if key != cur_key:
            if segments:
                segments[-1]["end"] = t.isoformat(timespec="minutes")
            definite = _rule_brief(by_id[definite_id]) if definite_id else None
            segments.append(
                {
                    "start": t.isoformat(timespec="minutes"),
                    "end": None,
                    "definite": definite,
                    "possible": [dict(_rule_brief(r), when=situation_text(r)) for r in maybe],
                }
            )
            cur_key = key
    horizon = start + datetime.timedelta(hours=hours)
    if segments:
        segments[-1]["end"] = horizon.isoformat(timespec="minutes")
    next_change = segments[0]["end"] if len(segments) > 1 else None
    return {
        "now": now.isoformat(timespec="minutes"),
        "horizon": horizon.isoformat(timespec="minutes"),
        "hours": hours,
        "enabled_rule_count": len(enabled),
        "situational_rule_count": len(situational_rules),
        "next_time_change_at": next_change,
        "min_hold_seconds": ruleset.min_hold_seconds,
        "segments": segments,
        "note": (
            "時刻・曜日・日付の条件だけから計算した予定です。混雑度・人物・残数・無人時間などの"
            "状況に依存する条件は予測できないため「条件次第」として別に表示します。"
        ),
    }
