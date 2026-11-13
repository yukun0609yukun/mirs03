"""状況アナウンスのノード(1台目Jetson)。docs/announcements.md

既存のトピック(director・survey_monitor・crowd_monitor・frontier_explorer・/cmd_vel)を読むだけで、
ロボットの動作には一切影響しない(cmd_vel・ゴールを出さない)。状況を`announce_events.SituationBuilder`で
フラグにし、`AnnounceManager`が定義表(~/.nokosahen_announce_events.json)に従って決めた操作を、
別スレッドでvoice_bridge(2台目)の `/announce_loop` `/announce` へ送る。ROSのコールバックはブロックしない。

- 通常の呼びかけ(announce_context→voice_node)とは別経路。探索中の発話抑止(MAPPING_ACTIVE_HOLD)の例外として動く。
- 設定ファイルが無い間は全体OFFで何も送らない(従来と同じ)。
- ループは約5秒ごとに同じ内容を再送(生きている信号)。このノード/1台目が落ちると、2台目が15秒で自動停止する。
- 音声の全体スイッチがOFFの間は送らない(voice_bridge/statusのvoice_enabledを数秒ごとに確認)。
- voice_bridgeに届かないときは静かに諦める(同じ失敗のログは60秒に1回まで)。
- `/nokosahen/announce_state`(String JSON)に、いまの状況フラグ・抑制の理由・送信状態を1Hzで出す(画面・調査用)。
- 【2026-10-30】文化祭の出店紹介(`pr_intro.py`、docs/announcements.md): 同じ状況フラグを`PrIntroManager`にも渡し、
  `~/.nokosahen_pr_intro.json`に従って、保存済みの音声を順に繰り返すプレイリスト(`/announce_loop`のparts)と、音声の事前作成の依頼
  (`/announce_prepare`)を2台目へ送る。紹介は状況アナウンス・呼びかけ・アンケート・緊急停止などが出る間は流さない(一番控えめ)。
  紹介の判定の例外は握り、状況アナウンスや走行に影響させない。
"""

from __future__ import annotations

import json
import queue
import threading
import time
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .announce_events import AnnounceEventsReader, AnnounceManager, SituationBuilder
from .pr_intro import PrIntroManager, PrIntroReader
from .json_tools import dumps, loads


class AnnounceSender:
    """voice_bridgeへ操作を送るスレッド(ROSに依存しない)。キューが詰まったら古いものから捨てる。"""

    LOG_EVERY_SEC = 60.0

    def __init__(self, base_url: str, timeout_s: float = 3.0, log=None, status_every_sec: float = 3.0) -> None:
        self.base_url = (base_url or "").strip().rstrip("/")
        self.timeout_s = float(timeout_s)
        self.status_every_sec = status_every_sec
        self._log = log or (lambda msg: None)
        self._q: "queue.Queue[Optional[Dict[str, Any]]]" = queue.Queue(maxsize=64)
        self._thread: Optional[threading.Thread] = None
        self.voice_enabled = True
        self.sent = 0
        self.failed = 0
        self.dropped = 0
        self.skipped_muted = 0
        self.last_error = ""
        self._last_log = 0.0
        self._last_status = 0.0

    @property
    def configured(self) -> bool:
        return bool(self.base_url)

    def start(self) -> None:
        if self.configured and self._thread is None:
            self._thread = threading.Thread(target=self._run, name="announce_sender", daemon=True)
            self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        try:
            self._q.put_nowait(None)
        except queue.Full:
            pass
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def submit(self, ops: List[Dict[str, Any]]) -> None:
        for op in ops:
            try:
                self._q.put_nowait(op)
            except queue.Full:
                try:
                    self._q.get_nowait()
                    self.dropped += 1
                    self._q.put_nowait(op)
                except (queue.Empty, queue.Full):
                    self.dropped += 1

    def _post(self, path: str, payload: Dict[str, Any]) -> None:
        req = urllib.request.Request(
            f"{self.base_url}{path}", data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(req, timeout=self.timeout_s) as res:
            body = json.loads(res.read().decode("utf-8") or "{}")
        if isinstance(body, dict) and body.get("muted") is True:
            self.voice_enabled = False  # 送信直前にOFFになった。次の状態確認で追従する

    def _refresh_status(self) -> None:
        try:
            with urllib.request.urlopen(f"{self.base_url}/status", timeout=self.timeout_s) as res:
                body = json.loads(res.read().decode("utf-8"))
            en = body.get("voice_enabled", True) if isinstance(body, dict) else True
            if isinstance(en, bool):
                self.voice_enabled = en
        except Exception:  # noqa: BLE001 - 届かないときは「ON」のまま。送信側の失敗として扱う
            pass

    def _note_failure(self, what: str, exc: BaseException) -> None:
        self.failed += 1
        self.last_error = f"{what}: {exc}"[:200]
        now = time.monotonic()
        if now - self._last_log >= self.LOG_EVERY_SEC:
            self._last_log = now
            self._log(f"状況アナウンスをvoice_bridgeへ送れません(静かに諦めます): {self.last_error}")

    def handle(self, op: Dict[str, Any]) -> None:
        """1件を送る(スレッド内から呼ぶ。テストからも直接呼べる)。"""
        now = time.monotonic()
        if now - self._last_status >= self.status_every_sec:
            self._last_status = now
            self._refresh_status()
        kind = op.get("op")
        if kind not in ("loop_stop", "playlist_stop", "prepare") and not self.voice_enabled:
            self.skipped_muted += 1  # マスターOFF中は送らない(停止は常に送る。音声の事前作成は鳴らさないので送る)
            return
        try:
            if kind == "loop":
                self._post("/announce_loop", {"id": op["id"], "text": op["text"], "interval_sec": op["interval_sec"],
                                              "volume": op["volume"], "enabled": True})
            elif kind == "loop_stop":
                self._post("/announce_loop", {"id": op["id"], "enabled": False})
            elif kind == "once":
                self._post("/announce", {"id": op["id"], "text": op["text"], "volume": op["volume"],
                                         "urgent": bool(op.get("urgent"))})
            elif kind == "playlist":  # 【2026-10-30】文化祭の出店紹介(pr_intro.py)。保存済みの音声を順に繰り返す(2台目は合成しない)
                self._post("/announce_loop", {"id": op["id"], "parts": op["parts"], "interval_sec": op["interval_sec"],
                                              "gap_sec": op["gap_sec"], "enabled": True})
            elif kind == "playlist_stop":
                self._post("/announce_loop", {"id": op["id"], "enabled": False})
            elif kind == "prepare":  # 音声の事前作成の依頼(作成済みなら2台目は何もしない)
                self._post("/announce_prepare", {"texts": op["texts"], "start": True})
            else:
                return
            self.sent += 1
            self.last_error = ""
        except urllib.error.HTTPError as exc:
            self._note_failure(f"HTTP {exc.code}", exc)
            exc.close()
        except Exception as exc:  # noqa: BLE001
            self._note_failure("接続", exc)

    def _run(self) -> None:
        while True:
            try:
                op = self._q.get(timeout=self.status_every_sec)
            except queue.Empty:
                self._refresh_status()  # 暇なときもOFF/ONの変化を追う
                self._last_status = time.monotonic()
                continue
            if op is None:
                return
            self.handle(op)

    def status(self) -> Dict[str, Any]:
        return {"configured": self.configured, "voice_enabled": self.voice_enabled, "sent": self.sent,
                "failed": self.failed, "dropped": self.dropped, "skipped_muted": self.skipped_muted,
                "last_error": self.last_error}


class AnnounceNode(Node):
    def __init__(self) -> None:
        super().__init__("announce_node")
        self.declare_parameter("voice_bridge_url", "http://198.51.100.3:8100")
        self.declare_parameter("announce_events_file", "~/.nokosahen_announce_events.json")
        # 【2026-10-30】文化祭の出店紹介の設定(ダッシュボードの full_dashboard の pr_intro_file と同じパスにすること)
        self.declare_parameter("pr_intro_file", "~/.nokosahen_pr_intro.json")
        self.declare_parameter("tick_hz", 4.0)
        self.declare_parameter("request_timeout_sec", 3.0)

        self._situation = SituationBuilder()
        self._reader = AnnounceEventsReader(str(self.get_parameter("announce_events_file").value))
        self._manager = AnnounceManager(self._reader.config)
        self._pr_reader = PrIntroReader(str(self.get_parameter("pr_intro_file").value))
        self._pr_manager = PrIntroManager(self._pr_reader.config)
        self._sender = AnnounceSender(
            str(self.get_parameter("voice_bridge_url").value),
            float(self.get_parameter("request_timeout_sec").value),
            log=lambda m: self.get_logger().warn(m),
        )
        self._sender.start()
        self._last_reader_poll = 0.0

        self.create_subscription(String, "/nokosahen/behavior_command", self._on_behavior_command, 10)
        self.create_subscription(Bool, "/nokosahen/survey_active", lambda m: self._situation.on_survey_active(m.data, self._now()), 5)
        self.create_subscription(String, "/nokosahen/crowd_state", self._on_crowd_state, 10)
        self.create_subscription(Bool, "/nokosahen/near_person_stop", lambda m: self._situation.on_near_person_stop(m.data, self._now()), 5)
        self.create_subscription(Bool, "/nokosahen/map_building_active", lambda m: self._situation.on_map_building_active(m.data, self._now()), 5)
        self.create_subscription(String, "/nokosahen/frontier_explorer/status", self._on_frontier_status, 5)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(String, "/nokosahen/moving_notice", self._on_moving_notice, 5)  # 約4Hz(ダッシュボードが判定)
        self._state_pub = self.create_publisher(String, "/nokosahen/announce_state", 5)
        hz = max(1.0, float(self.get_parameter("tick_hz").value))
        self.create_timer(1.0 / hz, self._tick)
        self.create_timer(1.0, self._publish_state)
        self.get_logger().info(f"announce_node -> {self._sender.base_url or '(voice_bridge_url未設定)'}")

    def _now(self) -> float:
        return time.monotonic()

    def _on_behavior_command(self, msg: String) -> None:
        self._situation.on_behavior_command(loads(msg.data, {}), self._now())

    def _on_crowd_state(self, msg: String) -> None:
        self._situation.on_crowd_state(loads(msg.data, {}), self._now())

    def _on_frontier_status(self, msg: String) -> None:
        self._situation.on_frontier_status(loads(msg.data, {}), self._now())

    def _on_moving_notice(self, msg: String) -> None:
        self._situation.on_moving_notice(loads(msg.data, {}), self._now())

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._situation.on_cmd_vel(float(msg.linear.x), self._now())

    def _tick(self) -> None:
        now = self._now()
        if now - self._last_reader_poll >= 1.0:
            self._last_reader_poll = now
            cfg, message, changed = self._reader.poll()
            if message:
                self.get_logger().info(message)
            if changed:
                self._manager.set_config(cfg)
            try:  # 文化祭の出店紹介の失敗が状況アナウンスや走行に影響しないよう、独立して扱う
                pcfg, pmsg, pchanged = self._pr_reader.poll()
                if pmsg:
                    self.get_logger().info(pmsg)
                if pchanged:
                    self._pr_manager.set_config(pcfg)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"文化祭の出店紹介の設定の読み込みで例外: {exc}")
        flags, context = self._situation.flags(now), self._situation.context(now)
        ops = self._manager.update(flags, context, now)
        try:
            ops = ops + self._pr_manager.update(flags, context, now)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"文化祭の出店紹介の判定で例外(状況アナウンスは続行): {exc}")
        if ops:
            self._sender.submit(ops)

    def _publish_state(self) -> None:
        now = self._now()
        msg = String()
        msg.data = dumps({
            "stamp": time.time(),
            "flags": self._situation.flags(now),
            "context": self._situation.context(now),
            "manager": self._manager.status(),
            "pr_intro": self._pr_manager.status(),
            "pr_intro_error": self._pr_reader.load_error,
            "sender": self._sender.status(),
            "config_error": self._reader.load_error,
        })
        self._state_pub.publish(msg)

    def shutdown(self) -> None:
        for op in self._manager.stop_all() + self._pr_manager.stop_all():
            self._sender.handle(op)  # 終了時は同期で止めを送る(届かなくても2台目が15秒で止める)
        self._sender.stop()


def main() -> None:
    rclpy.init()
    node = AnnounceNode()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
