from __future__ import annotations

import json
import time

import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

# 同じ内容の発話依頼を、送信に成功してからこの秒数以内にもう一度受け取ったら、重複として捨てる(2026-10-21)。
# directorの発話間隔(既定60秒)より十分短く、同じ内容でも次の周期の依頼は送る。
DEDUP_WINDOW_SEC = 5.0


class VoiceNode(Node):
    """`/nokosahen/announce_context` を購読し、ホスト側の `voice_bridge` に発話させる。

    このノード自体は音声合成も再生も行わない。VOICEVOX/Ollamaはコンテナの外
    (Jetsonの素のOS)に常駐する `voice_bridge` が担当し、ここは
    `--network host` 経由で `127.0.0.1` へHTTPで発話を依頼するだけの薄い橋渡し
    (`docs/setup_docker.md`参照)。

    `announce_context`にはメニュー一覧(+induction時は残数状況)と、AI側が
    使えない場合のルールベースのフォールバック文が入っている。generate=trueの
    ときは、実際に何を話すかの文面自体をvoice_bridge側でJetson内のAI
    (Ollama)に考えさせる(このノードでは中身を解釈しない、そのまま橋渡しする)。
    """

    def __init__(self) -> None:
        super().__init__("voice_node")
        self.declare_parameter("voice_bridge_url", "http://127.0.0.1:8100")
        # true: メニュー内容(+完成版では残数状況)からJetson内のAI(Ollama)に
        # 発話文面を考えさせる。false: ルールベースのフォールバック文を
        # そのまま話す(Ollama未検証/不調時の安全側の設定)。
        self.declare_parameter("generate", True)
        self.declare_parameter("request_timeout_sec", 3.0)
        self.declare_parameter("status_poll_hz", 1.0)

        self.voice_bridge_url = str(self.get_parameter("voice_bridge_url").value).rstrip("/")
        self.generate = bool(self.get_parameter("generate").value)
        self.timeout = float(self.get_parameter("request_timeout_sec").value)
        poll_hz = max(0.2, float(self.get_parameter("status_poll_hz").value))
        poll_period = 1.0 / poll_hz
        # rclpy.spin()はシングルスレッドなので、ポーリング周期より長いHTTP
        # タイムアウトのままvoice_bridgeが応答不能になると、_poll_status()が
        # 毎回タイムアウトいっぱいブロックし続け、他のコールバック(発話依頼等)
        # の処理が滞る。ステータス確認は軽量・頻繁な用途なので、周期の半分を
        # 上限にタイムアウトを短くしておく。
        self.status_poll_timeout = min(self.timeout, max(0.2, poll_period * 0.5))

        self._last_context_raw = ""
        self._last_context_time = float("-inf")  # 最後に送信に成功した単調時計の時刻
        # 【2026-10-09】voice_bridgeの音声マスタースイッチ(全体ON/OFF、docs/voice_master.md)。
        # OFFの間は発話依頼を送らない(/statusのvoice_enabledを_poll_statusで更新)。
        # 古いvoice_bridge(項目なし)・接続不可のときは「ON」のまま従来どおり送る。
        self._voice_enabled = True
        self._busy_pub = self.create_publisher(Bool, "/audio/busy", 10)

        self.create_subscription(
            String, "/nokosahen/announce_context", self._on_announce_context, 10
        )
        self.create_timer(1.0 / poll_hz, self._poll_status)

        self.get_logger().info(f"voice_node -> {self.voice_bridge_url}")

    def _on_announce_context(self, msg: String) -> None:
        raw = msg.data.strip()
        # 【2026-10-21】重複排除は、直前に「送信に成功した」同じ内容が、DEDUP_WINDOW_SEC秒以内にもう一度届いた場合だけ。
        # 以前は、内容が同じなら(directorが60秒ごとに出す依頼は、メニュー・残数が変わらなければ毎回同じ内容)
        # 2回目以降を永久に送らず、ロボットが黙っていた。さらに、送信の前に「送った」と記録していたので、
        # 2台目が止まっていた間・音声が全体OFFだった間の依頼も、二度と送られなかった(全体レビュー)。
        now = time.monotonic()
        if not raw or (raw == self._last_context_raw and (now - self._last_context_time) < DEDUP_WINDOW_SEC):
            return

        try:
            context = json.loads(raw)
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(f"announce_contextのJSON解析に失敗: {exc}")
            return
        if not isinstance(context, dict):
            return

        if not self._voice_enabled:
            # マスターOFF中は送らない(失敗でもない。ONに戻った後に古い案内をまとめて話さないよう、積み直しもしない)。
            # 送っていないので「送信済み」とは記録しない(ONに戻ったあとの同じ内容の依頼は、送る)。
            self.get_logger().debug("音声が全体OFFのため発話依頼を送りません")
            return

        try:
            res = requests.post(
                f"{self.voice_bridge_url}/speak",
                json={"context": context, "generate": self.generate},
                timeout=self.timeout,
            )
            # ネットワークレベルでは繋がっていても、voice_bridge側が4xx/5xxを
            # 返す(パラメータ不備・キュー満杯等)ケースを無言で見逃さないようにする。
            res.raise_for_status()
            # マスターOFFのvoice_bridgeは200+{"muted":true}を返す(送信直前にOFFになった場合)。
            # これは失敗ではないので警告しない(状態は次の_poll_statusで追従する)。
            self._last_context_raw = raw  # 送信に成功したときだけ記録する
            self._last_context_time = now
        except Exception as exc:  # noqa: BLE001 - voice_bridge停止中でもノードは落とさない
            self.get_logger().warn(f"voice_bridgeへの発話依頼に失敗: {exc}")

    def _poll_status(self) -> None:
        try:
            res = requests.get(f"{self.voice_bridge_url}/status", timeout=self.status_poll_timeout)
            res.raise_for_status()
            body = res.json()
            speaking = bool(body.get("speaking", False))
            enabled = body.get("voice_enabled", True)
            if isinstance(enabled, bool):
                if enabled != self._voice_enabled:
                    self.get_logger().info("音声の全体スイッチ: " + ("ON" if enabled else "OFF(発話依頼を止めます)"))
                self._voice_enabled = enabled
        except Exception:  # noqa: BLE001
            speaking = False

        msg = Bool()
        msg.data = speaking
        self._busy_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = VoiceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
