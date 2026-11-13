"""実機の記録(person_candidates)を、本番の「向く先」の処理に通して再生する道具(ROS・実機は使わない)。2026-11-06。

本番の処理(behavior_director の _fresh_person_candidates → 体の枠除外 → 監視範囲 → FaceBearingSelector)と同じ順に通し、
時間順に「向く先(世界の方位[deg])」を出して指標を計算する。yaw は記録の値をそのまま使う(閉ループではない)。
記録の形式(1行1メッセージ): t[s], yaw[deg], lidar[{b,r,conf,rej}], camera[bearing deg], ...
使い方: python test/face_camera_replay.py            # ベースラインと新方式の指標
        python test/face_camera_replay.py sweep      # パラメータの感度分析
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nokosahen_core import crowd_bearing as cb  # noqa: E402

DATA = Path(__file__).resolve().parent / "data" / "face_rec_20261106_standing.jsonl"
JUMP_DEG = 20.0
HOLD_BAND_DEG = 10.0


def load(path: Path = DATA) -> List[dict]:
    return [json.loads(l) for l in Path(path).read_text(encoding="utf-8").splitlines() if l.strip()]


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def to_message(rec: dict) -> dict:
    """記録1行 → person_candidates の中身(director が受け取る形。方位はrad)。"""
    return {
        "lidar": [{"bearing": math.radians(l["b"]), "range": l["r"], "confirmed": bool(l["conf"]),
                   "rejected": bool(l["rej"])} for l in rec.get("lidar") or []],
        "camera": [{"bearing": math.radians(b)} for b in rec.get("camera") or []],
    }


def replay(records: List[dict], tracker=None, body_range_m: float = 0.36, ref_yaw_deg: Optional[float] = None,
           selector=None, tiers=None) -> List[dict]:
    """戻り値: 各メッセージの {t, yaw, world(向く先の世界方位deg or None), ncam(融合に渡したカメラ数)}。
    tracker: update(now, yaw_rad, camera_bearings_rad) -> [rad] を持つもの。None ならカメラ検出をそのまま使う(ベースライン)。"""
    sel = selector or cb.FaceBearingSelector()
    out = []
    last_world = None
    for rec in records:
        t, yaw = float(rec["t"]), math.radians(rec["yaw"])
        msg = to_message(rec)
        cams = msg["camera"]
        if tracker is not None:
            cams = [{"bearing": b} for b in tracker.update(t, yaw, [c["bearing"] for c in cams])]
        cands = list(cb.fuse_candidates(msg["lidar"], cams))
        if body_range_m > 0:
            cands = [c for c in cands if not cb.is_body_candidate(c, body_range_m)]
        if ref_yaw_deg is not None:
            ref = math.radians(ref_yaw_deg)
            cands = [c for c in cands if cb.in_monitor_range_sticky(c.get("bearing"), yaw, ref, last_world)]
        if tiers is not None:
            cands = tiers.update(t, yaw, cands)
        b = sel.update(t, cands, yaw=yaw) if cands else None
        world = None if b is None else wrap_deg(math.degrees(yaw + b))
        last_world = None if b is None else yaw + b
        legacy = bool(getattr(sel, "_tracks", None) == [] and cands)   # 距離が分かる人が居らず、重みの多数決に落ちた
        out.append({"t": t, "yaw": rec["yaw"], "world": world, "ncam": len(cams), "legacy": legacy})
    return out


def metrics(out: List[dict]) -> Dict[str, float]:
    """①切り替わり回数(連続する出力の20度超のジャンプ) ②同じ向き(±10度)を保った最長時間[s] ③Noneの割合 ④連続出力の差の平均/最大[deg]"""
    n = len(out)
    none = sum(1 for o in out if o["world"] is None)
    diffs = []
    prev = None
    for o in out:
        if o["world"] is None:
            continue
        if prev is not None:
            diffs.append(abs(wrap_deg(o["world"] - prev)))
        prev = o["world"]
    jumps = sum(1 for d in diffs if d > JUMP_DEG)
    longest = 0.0
    start = None
    for o in out:
        w = o["world"]
        if w is None:
            start = None
            continue
        if start is None or abs(wrap_deg(w - start[1])) > HOLD_BAND_DEG:
            start = (o["t"], w)
        longest = max(longest, o["t"] - start[0])
    return {"jumps": jumps, "longest_hold_s": round(longest, 2), "none_ratio": round(none / n, 4) if n else 0.0,
            "mean_diff_deg": round(sum(diffs) / len(diffs), 2) if diffs else 0.0,
            "max_diff_deg": round(max(diffs), 1) if diffs else 0.0}


FOV_HALF_DEG = 75.0


def synth_oscillation(seed: int, duration: float = 60.0, person_world: Optional[float] = 150.0, person_range: float = 1.7,
                      statics=((90.0, 0.8), (-110.0, 1.0), (-150.0, 1.1)), period: float = 9.5, amp_deg: float = 110.0,
                      center_deg: float = 150.0) -> List[dict]:
    """2026-11-06 10:40の実機観測の再現(合成)。ロボットが約9.5秒周期で±110度ゆっくり往復旋回する。遠い(1.7 m)カメラ確認済みの人が1人と、
    近い(0.8〜1.1 m)LiDARだけの静止物が複数(世界方位が離れている)。カメラの視野(±75度)に入っている静止物は「カメラに人と否定された」(rej)、
    人はカメラが検出した回(0.8秒周期、検出率0.88)だけ confirmed。LiDARは0.2秒周期、各候補10%見逃し、方位1度・距離3 cmのばらつき。
    person_world=None なら人は居ない(静止物だけ)。"""
    import random
    rng = random.Random(seed)
    recs, cam, conf, n = [], [], False, 0
    while n * MSG_PERIOD <= duration + 1e-9:
        t = round(n * MSG_PERIOD, 3)
        yaw = wrap_deg(center_deg + amp_deg * math.sin(2 * math.pi * t / period))
        rel = lambda w: wrap_deg(w - yaw)
        if n % int(round(OBS_PERIOD / MSG_PERIOD)) == 0:
            cam = []
            if person_world is not None and abs(rel(person_world)) <= FOV_HALF_DEG and rng.random() < P_DETECT:
                cam = [round(rel(person_world) + rng.gauss(0, SIGMA_DEG), 1)]
        lidar = []
        objs = [(w, r, False) for w, r in statics] + ([(person_world, person_range, True)] if person_world is not None else [])
        for w, r, is_person in objs:
            if rng.random() < 0.1:
                continue
            b = rel(w)
            infov = abs(b) <= FOV_HALF_DEG
            confirmed = bool(is_person and cam and any(abs(c - b) <= 15.0 for c in cam))
            lidar.append({"b": round(b + rng.gauss(0, 1.0), 1), "r": round(r + rng.gauss(0, 0.03), 2), "conf": confirmed,
                          "rej": bool(infov and not confirmed and not is_person)})
        recs.append({"t": t, "yaw": yaw, "lidar": lidar, "camera": list(cam)})
        n += 1
    return recs


def camera_only(records: List[dict]) -> List[dict]:
    """LiDARの距離を全部消した記録(カメラだけの候補しか無い、問題の状況の再現)。"""
    return [dict(r, lidar=[]) for r in records]


# --- 新しい対象への反応時間(合成。カメラだけの候補。検出の周期・欠測率・ばらつきは記録から: 周期0.8秒、検出率0.88、方位のばらつき3度) ---
OBS_PERIOD = 0.8
MSG_PERIOD = 0.2
P_DETECT = 0.88
SIGMA_DEG = 3.0


def synth_scene(seed: int, a_deg: Optional[float], b_deg: Optional[float], t_switch: float, t_end: float,
                yaw_deg: float = 0.0) -> List[dict]:
    """ロボット静止(yaw固定)。0〜t_switch は a にだけ、t_switch〜t_end は b にだけ人がいる(Noneならいない)。カメラだけ。
    メッセージは0.2秒ごと、カメラの検出は0.8秒ごとに更新され、間は同じ内容が繰り返し届く(実機の記録と同じ)。"""
    import random
    rng = random.Random(seed)
    recs, cam, n = [], [], 0
    while n * MSG_PERIOD <= t_end + 1e-9:
        t = round(n * MSG_PERIOD, 3)
        if n % int(round(OBS_PERIOD / MSG_PERIOD)) == 0:
            tgt = a_deg if t < t_switch else b_deg
            cam = [round(wrap_deg(tgt - yaw_deg + rng.gauss(0, SIGMA_DEG)), 1)] if (tgt is not None and rng.random() < P_DETECT) else []
        recs.append({"t": t, "yaw": yaw_deg, "lidar": [], "camera": list(cam)})
        n += 1
    return recs


def reaction_times(make_tracker, seeds=range(12)) -> Dict[str, float]:
    """appear: 誰もいない所から人が現れて、向く先が出るまでの時間。swap: A(-50度)の人が居なくなり同時にB(+60度)が現れて、
    向く先がBの15度以内になるまでの時間。どちらも 平均[s] と 最悪[s](届かなかった種は上限の20秒として数える)。"""
    res = {"appear": [], "swap": []}
    t0 = 8.0
    for sd in seeds:
        recs = synth_scene(sd, None, 60.0, t0, t0 + 20.0)
        out = replay(recs, tracker=make_tracker() if make_tracker else None)
        ts = [o["t"] - t0 for o in out if o["t"] >= t0 and o["world"] is not None]
        res["appear"].append(ts[0] if ts else 20.0)
        recs = synth_scene(sd, -50.0, 60.0, t0, t0 + 20.0)
        out = replay(recs, tracker=make_tracker() if make_tracker else None)
        ts = [o["t"] - t0 for o in out if o["t"] >= t0 and o["world"] is not None and abs(wrap_deg(o["world"] - 60.0)) <= 15.0]
        res["swap"].append(ts[0] if ts else 20.0)
    return {"appear_mean": round(sum(res["appear"]) / len(res["appear"]), 2), "appear_max": round(max(res["appear"]), 2),
            "swap_mean": round(sum(res["swap"]) / len(res["swap"]), 2), "swap_max": round(max(res["swap"]), 2)}


def evaluate(make_tracker, records=None) -> Dict[str, object]:
    recs = records or load()
    return {"full": metrics(replay(recs, tracker=make_tracker() if make_tracker else None)),
            "cam_only": metrics(replay(camera_only(recs), tracker=make_tracker() if make_tracker else None)),
            "reaction": reaction_times(make_tracker)}


def sweep():
    from nokosahen_core.camera_tracks import CameraTrackBank
    recs = load()
    rows = []
    base = evaluate(None, recs)
    print("baseline", base)
    for n in (1, 2, 3):
        for hold in (0.0, 1.0, 1.6, 2.0, 2.5, 3.0, 4.0):
            for match in (8.0, 12.0, 16.0):
                for alpha in (0.3, 0.5, 1.0):
                    mk = lambda: CameraTrackBank(match_deg=match, confirm_n=n, hold_sec=hold, smooth_alpha=alpha)
                    e = evaluate(mk, recs)
                    rows.append((n, hold, match, alpha, e))
    return base, rows


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "sweep":
        base, rows = sweep()
        print("n hold match alpha | full(jumps,hold,none,mean) | cam_only(jumps,hold,none,mean) | appear mean/max swap mean/max")
        for n, hold, match, alpha, e in rows:
            f, c, r = e["full"], e["cam_only"], e["reaction"]
            print(f"{n} {hold:3.1f} {match:4.0f} {alpha:3.1f} | {f['jumps']:2d} {f['longest_hold_s']:5.1f} {f['none_ratio']:.3f} {f['mean_diff_deg']:4.2f} | "
                  f"{c['jumps']:2d} {c['longest_hold_s']:5.1f} {c['none_ratio']:.3f} {c['mean_diff_deg']:4.2f} | "
                  f"{r['appear_mean']:4.2f}/{r['appear_max']:4.2f} {r['swap_mean']:4.2f}/{r['swap_max']:4.2f}")
    else:
        from nokosahen_core.camera_tracks import CameraTrackBank
        print("baseline", evaluate(None))
        print("new     ", evaluate(CameraTrackBank))
