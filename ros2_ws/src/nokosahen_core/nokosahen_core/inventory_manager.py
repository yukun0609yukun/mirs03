from __future__ import annotations

import datetime
import os
from dataclasses import asdict, dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .consumption_log import ConsumptionLog
from .json_tools import dumps, loads


@dataclass
class MenuItem:
    name: str
    initial: int
    remaining: int
    sold: int = 0

    @property
    def ratio_remaining(self) -> float:
        if self.initial <= 0:
            return 0.0
        return max(0.0, min(1.0, self.remaining / self.initial))


class InventoryManager(Node):
    """Track menu inventory and select the menu that should be promoted."""

    def __init__(self) -> None:
        super().__init__("inventory_manager")
        self.declare_parameter("initial_phase_seconds", 120.0)
        self.declare_parameter("promote_gap_count", 5)
        self.declare_parameter("promote_gap_ratio", 0.15)
        # 昇格(induction)と降格(initialへ戻る)を同じ閾値で判定すると、残数差が
        # 閾値ちょうど付近で訂正操作(±1)のたびにphaseがチラつく。降格側は
        # 昇格側より緩い閾値(enter閾値×この比率)にして、いったん昇格したら
        # 大きく差が縮まるまで維持するヒステリシスを設ける。
        self.declare_parameter("promote_hysteresis_ratio", 0.5)
        self.declare_parameter("publish_hz", 1.0)
        # 入力・カウントの簡易監査ログ(何時に誰が何をしたか)。DBは導入せず、
        # 追記型のJSON Linesファイルで十分とする(授業規模の運用を想定)。
        # 空文字ならログ出力しない。
        self.declare_parameter("log_file", "nokosahen_events.jsonl")
        # カメラ/手動によるcount_eventの履歴(いつ・何が・何食減ったか)を、
        # ダッシュボードでグラフ表示できるようSQLiteへ記録する
        # (2026-07-22追加、ユーザー要望)。空文字なら記録しない。
        self.declare_parameter("consumption_db_file", "~/.nokosahen_consumption.db")

        self.initial_phase_seconds = float(
            self.get_parameter("initial_phase_seconds").value
        )
        self.promote_gap_count = int(self.get_parameter("promote_gap_count").value)
        self.promote_gap_ratio = float(self.get_parameter("promote_gap_ratio").value)
        self.promote_hysteresis_ratio = max(
            0.0, min(1.0, float(self.get_parameter("promote_hysteresis_ratio").value))
        )
        publish_hz = max(0.2, float(self.get_parameter("publish_hz").value))
        log_file = str(self.get_parameter("log_file").value).strip()
        self.log_file = os.path.expanduser(log_file) if log_file else ""
        self._log_write_failures = 0
        consumption_db_file = str(self.get_parameter("consumption_db_file").value).strip()
        self._consumption_log = (
            ConsumptionLog(consumption_db_file) if consumption_db_file else None
        )
        # ログ書き込みが連続でこの回数失敗したら、以後は無効化して
        # (ディスクフル等の恒久障害で)警告ログをスパムし続けないようにする。
        self._log_write_failure_limit = 10

        self._items: Dict[str, MenuItem] = {}
        self._started_at = self.get_clock().now()
        self._last_state = {}
        self._promoted_item: Optional[str] = None

        self._menu_init_sub = self.create_subscription(
            String, "/nokosahen/menu_init", self._on_menu_init, 10
        )
        self._count_event_sub = self.create_subscription(
            String, "/nokosahen/count_event", self._on_count_event, 50
        )
        # 【2026-08-07追加】明電舎側のタブレット(POSレジ集計)が出す「累計販売数」を
        # 手入力してもらい、残数 = 初期数 - 累計販売数 で算出する方式。
        # 通過人数カメラ方式(_on_count_event、checkout_camera_node/camera_counter)は
        # 精度に限界がある(7/24現地検証で誤差約2.2%、重複カウント・職員誤認識等)
        # ことが実機で判明したため、より正確な明電舎側の実売データを正とする
        # 運用に切り替える。カメラ方式のコードは温存し、削除しない
        # (将来的な自動化・他会場での代替手段として使える可能性があるため)。
        self._sold_total_sub = self.create_subscription(
            String, "/nokosahen/sold_total_event", self._on_sold_total_event, 20
        )
        self._state_pub = self.create_publisher(
            String, "/nokosahen/menu_state", 10
        )
        self._timer = self.create_timer(1.0 / publish_hz, self._publish_state)

    def _on_menu_init(self, msg: String) -> None:
        data = loads(msg.data, {})
        if not isinstance(data, dict):
            self.get_logger().warn("menu_init ignored: payload is not an object")
            return
        items = data.get("items", [])
        next_items: Dict[str, MenuItem] = {}

        if isinstance(items, dict):
            items = [{"name": name, "initial": amount} for name, amount in items.items()]
        if not isinstance(items, list):
            self.get_logger().warn("menu_init ignored: items is not a list")
            return

        for raw in items:
            if not isinstance(raw, dict):
                continue
            name = str(raw.get("name", "")).strip()
            if not name:
                continue
            # 【2026-10-21】null・文字列・無限大などが1項目でも混ざると、int()の例外(TypeError/OverflowError)がコールバックから
            # 漏れてノードが終了し、メモリ上の本日の残数がすべて消えていた。その項目だけを飛ばす。
            try:
                initial = max(0, int(raw.get("initial", raw.get("remaining", 0))))
                remaining = max(0, int(raw.get("remaining", initial)))
                sold = max(0, int(raw.get("sold", max(0, initial - remaining))))
            except (TypeError, ValueError, OverflowError):
                self.get_logger().warn(f"menu_init: item {name!r} skipped (initial/remaining/sold is not a finite number)")
                continue
            next_items[name] = MenuItem(
                name=name,
                initial=max(initial, remaining + sold),
                remaining=remaining,
                sold=sold,
            )

        # 【2026-08-06修正】以前は再登録のたびに無条件でリセットしていたため、
        # 入力ミスの訂正等でmenu_initを送り直すたびにinitial_phase_seconds
        # (既定120秒)が最初からやり直しになり、頻繁に修正すると永久に
        # induction phaseへ入れなかった(A/Bテスト・推奨効果分析が
        # promoted_menu=Noneしか記録できなくなる)。真に新規の初期化(前回まで
        # メニュー未登録)の場合だけ計測開始時刻をリセットする。
        # 【2026-08-07修正】以前はnext_itemsが空だと無条件で無視していたため、
        # 画面上でメニュー行を全部削除して送信しても本日の残数表示をクリア
        # できなかった。空での送信は「本日のメニューをクリアする」という
        # 意図の正当な操作として扱い、実際に空の状態を反映する。
        # self._itemsが既に空({})なら、次に空でない内容が来たときこの条件が
        # 自然にTrueになるため、クリア時にここで_started_atを触る必要は無い
        # (Noneにすると_elapsed_seconds()がNone減算でクラッシュするため不可)。
        if next_items and not self._items:
            self._started_at = self.get_clock().now()
        self._items = next_items
        self.get_logger().info(
            f"initialized menus: {', '.join(self._items.keys())}" if self._items
            else "menu cleared (empty items received)"
        )
        self._log_event(
            "menu_init",
            {"items": [asdict(item) for item in self._items.values()]},
        )
        self._publish_state()

    def _on_count_event(self, msg: String) -> None:
        data = loads(msg.data, {})
        if not isinstance(data, dict):
            self.get_logger().warn("count_event ignored: payload is not an object")
            return
        menu = str(data.get("menu", "")).strip()
        raw_delta = data.get("delta", data.get("count", 1))
        try:
            # float(x)を経由することで、JSON数値の1.5だけでなく文字列"1.5"の
            # ような入力も同じ経路で処理しつつ、"abc"のような非数値は
            # ここでまとめて弾く(未処理のValueErrorがコールバックから
            # 漏れてノードを不安定にするのを防ぐ)。
            delta = int(float(raw_delta))
        except (TypeError, ValueError, OverflowError):  # OverflowError: float("inf") / 1e999
            self.get_logger().warn(f"count_event ignored: delta is not a number: {raw_delta!r}")
            return

        if not menu:
            self.get_logger().warn("count_event ignored because menu is empty")
            return
        if menu not in self._items:
            self.get_logger().warn(f"count_event ignored for unknown menu: {menu}")
            return
        if delta == 0:
            return

        item = self._items[menu]
        if delta > 0:
            # 通常の消費カウント(+1など)。残数を超えては減らさない。
            consumed = min(delta, item.remaining)
            item.remaining -= consumed
            item.sold += consumed
            if consumed < delta:
                self.get_logger().warn(f"{menu} is already empty; clipped count delta")
        else:
            # マイナス操作(誤カウントの訂正・入力し直し)。
            # 売れた数(sold)以上は戻せない(remainingがinitialを超えないようにする)。
            restore = min(-delta, item.sold)
            item.remaining += restore
            item.sold -= restore
            if restore < -delta:
                self.get_logger().warn(f"{menu} has no more sold count to restore; clipped")

        source = str(data.get("source", "unknown"))
        self._log_event(
            "count_event",
            {
                "menu": menu,
                "delta": delta,
                "source": source,
                "remaining_after": item.remaining,
                "sold_after": item.sold,
            },
        )
        if self._consumption_log is not None:
            self._consumption_log.record(
                timestamp=datetime.datetime.now().isoformat(timespec="seconds"),
                menu=menu,
                delta=delta,
                source=source,
                remaining_after=item.remaining,
                sold_after=item.sold,
            )
        self._publish_state()

    def _on_sold_total_event(self, msg: String) -> None:
        """明電舎タブレット(POSレジ集計)の「累計販売数」をそのまま受け取り、
        残数 = 初期数 - 累計販売数 で上書きする。

        _on_count_event(delta積み上げ方式、カメラ通過カウント/手動+1-1用)とは
        独立した経路。こちらは毎回「今の累計」を絶対値で受け取るため、
        入力を送り直しても二重計上にならない(差分だけをconsumption_logへ
        記録するので、グラフ上の推移表示はdelta方式と同じ形で扱える)。
        """
        data = loads(msg.data, {})
        if not isinstance(data, dict):
            self.get_logger().warn("sold_total_event ignored: payload is not an object")
            return
        menu = str(data.get("menu", "")).strip()
        if not menu:
            self.get_logger().warn("sold_total_event ignored because menu is empty")
            return
        if menu not in self._items:
            self.get_logger().warn(f"sold_total_event ignored for unknown menu: {menu}")
            return
        try:
            sold_total = int(float(data.get("sold_total", 0)))
        except (TypeError, ValueError, OverflowError):
            self.get_logger().warn(
                f"sold_total_event ignored: sold_total is not a number: {data.get('sold_total')!r}"
            )
            return
        if sold_total < 0:
            self.get_logger().warn(f"sold_total_event ignored: negative sold_total: {sold_total}")
            return

        item = self._items[menu]
        old_sold = item.sold
        if sold_total == old_sold:
            return  # 変化なし。ログを毎回積まないよう何もしない。

        # タブレット側の累計は仕込み追加等でinitialを超えることもあり得るため、
        # sold自体はクランプせずそのまま反映する(remainingだけ0未満にしない)。
        item.sold = sold_total
        item.remaining = max(0, item.initial - sold_total)
        delta = sold_total - old_sold

        source = str(data.get("source", "unknown"))
        estimated = bool(data.get("estimated", False))
        self._log_event(
            "sold_total_event",
            {
                "menu": menu,
                "sold_total": sold_total,
                "source": source,
                "remaining_after": item.remaining,
                "sold_after": item.sold,
                "estimated": estimated,
            },
        )
        if self._consumption_log is not None:
            self._consumption_log.record(
                timestamp=datetime.datetime.now().isoformat(timespec="seconds"),
                menu=menu,
                delta=delta,
                source=source,
                remaining_after=item.remaining,
                sold_after=item.sold,
                estimated=estimated,
            )
        self._publish_state()

    def _log_event(self, kind: str, payload: dict) -> None:
        if not self.log_file:
            return
        entry = {
            "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
            "kind": kind,
            **payload,
        }
        try:
            with open(self.log_file, "a", encoding="utf-8") as f:
                f.write(dumps(entry) + "\n")
            self._log_write_failures = 0
        except (OSError, ValueError, UnicodeError) as exc:
            # UnicodeEncodeErrorはOSErrorのサブクラスではない(ValueErrorの
            # サブクラス)ため、ディスクフル等のOSErrorだけでなくここも拾う。
            self._log_write_failures += 1
            if self._log_write_failures >= self._log_write_failure_limit:
                self.get_logger().error(
                    f"イベントログの書き込みが{self._log_write_failures}回連続で失敗したため、"
                    f"監査ログを無効化します: {exc}"
                )
                self.log_file = ""
            else:
                self.get_logger().warn(f"イベントログの書き込みに失敗: {exc}")

    def _elapsed_seconds(self) -> float:
        return (self.get_clock().now() - self._started_at).nanoseconds / 1e9

    def _select_promote_item(self) -> Optional[str]:
        if len(self._items) < 2:
            self._promoted_item = None
            return None
        if self._elapsed_seconds() < self.initial_phase_seconds:
            self._promoted_item = None
            return None

        items: List[MenuItem] = list(self._items.values())
        max_remaining = max(item.remaining for item in items)
        min_remaining = min(item.remaining for item in items)
        max_ratio = max(item.ratio_remaining for item in items)
        min_ratio = min(item.ratio_remaining for item in items)

        gap_count = max_remaining - min_remaining
        gap_ratio = max_ratio - min_ratio

        # ヒステリシス: 既に昇格中なら、より緩い閾値(exit)を下回るまで維持する。
        # そうでなければ、通常の(enter)閾値を上回って初めて昇格させる。
        # これにより、閾値ちょうど付近での訂正操作(±1)によるphaseのチラつきを防ぐ。
        if self._promoted_item is not None:
            exit_gap_count = self.promote_gap_count * self.promote_hysteresis_ratio
            exit_gap_ratio = self.promote_gap_ratio * self.promote_hysteresis_ratio
            if gap_count < exit_gap_count and gap_ratio < exit_gap_ratio:
                self._promoted_item = None
                return None
        else:
            if gap_count < self.promote_gap_count and gap_ratio < self.promote_gap_ratio:
                return None

        candidates = [item for item in items if item.remaining == max_remaining]
        candidates.sort(key=lambda item: item.ratio_remaining, reverse=True)
        if not candidates or candidates[0].remaining <= 0:
            self._promoted_item = None
            return None

        self._promoted_item = candidates[0].name
        return self._promoted_item

    def _make_state(self) -> dict:
        promote_item = self._select_promote_item()
        phase = "induction" if promote_item else "initial"
        total_initial = sum(item.initial for item in self._items.values())
        total_remaining = sum(item.remaining for item in self._items.values())
        total_sold = sum(item.sold for item in self._items.values())

        return {
            "stamp": self.get_clock().now().nanoseconds / 1e9,
            "phase": phase,
            "promote_item": promote_item,
            "elapsed_seconds": self._elapsed_seconds(),
            "totals": {
                "initial": total_initial,
                "remaining": total_remaining,
                "sold": total_sold,
                "remaining_ratio": (total_remaining / total_initial)
                if total_initial
                else 0.0,
            },
            "items": [asdict(item) | {"ratio_remaining": item.ratio_remaining}
                      for item in self._items.values()],
        }

    def _publish_state(self) -> None:
        state = self._make_state()
        self._last_state = state
        msg = String()
        msg.data = dumps(state)
        self._state_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = InventoryManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

