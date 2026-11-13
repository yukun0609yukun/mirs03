"""マッピング中に機体自身が走り回って地図を作る、簡易フロンティア探索ノード。

occupancy grid(/map)上で「既知(自由)」と「未知」の境界(フロンティア)を
探し、一番近いフロンティアへ`/goal_pose`(nav2_goal_router経由でNav2へ橋渡し
される、既存の仕組みをそのまま使う)を送る。フロンティアが無くなれば
(=地図全体を見終えた)探索を終了する。

安全のため:
- ハード/ソフト緊急停止、手動運転(WASD)が始まったら即座に探索ゴールの
  送信を止める(手動運転を最優先する、というユーザー方針)。ゴールの
  キャンセル自体はnav2_goal_router側が担当するので、ここでは新しい
  ゴールを送らないことだけに専念する。
- ダッシュボードの「走行可能範囲」(travel_boundary)が設定されていれば、
  その範囲内のフロンティアだけを対象にする。
- 走行はNav2のNavigateToPose(nav2_goal_router経由)だけ。このノードは/cmd_velを出さず、独自の速度も持たない
  (速度は自律走行と同じnav_max_speed、回避・後退再計画・衝突監視も同じNav2設定。docs/exploration_and_approach.md)。
- 既定では無効(enabled=false)。ダッシュボードの「自律探索を開始」ボタン
  (/nokosahen/frontier_explorer/enabledへpublish)で有効化する。
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .json_tools import dumps, loads
from .map_orientation import point_in_rect, rect_from_state
from .motion_hold_logic import UserCancelHold
from .patrol_planner import grid_from_msg, snap_to_drivable

try:
    from tf2_ros import Buffer, TransformListener
except Exception:  # pragma: no cover - 環境依存
    Buffer = None  # type: ignore
    TransformListener = None  # type: ignore


class FrontierExplorer(Node):
    # __init__で設定する。クラス側の既定None=未設定(__init__を通さないテスト用インスタンス向け)
    _user_cancel: Optional[UserCancelHold] = None

    def __init__(self) -> None:
        super().__init__("frontier_explorer")
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("free_threshold", 40)
        # このセル数未満のフロンティア塊はノイズとして無視する。
        self.declare_parameter("min_frontier_cells", 6)
        # このゴールに時間がかかりすぎたら諦めて次のフロンティアへ切り替える。
        self.declare_parameter("goal_timeout_sec", 25.0)
        # 目的地にこれだけ近づいたら「到達した」とみなし、次のフロンティアへ。
        self.declare_parameter("goal_reached_tolerance_m", 0.4)
        # フロンティア(既知/未知の境界)がすぐそのまま壁際にあると、Nav2の
        # コストマップ膨張(inflation_radius、実機で0.45〜0.55m)の中に
        # ゴールが入ってしまい、経路計画が失敗し続ける(実機で確認した不具合)。
        # 障害物からこの距離以上離れたフロンティアだけを対象にする。
        self.declare_parameter("min_clearance_from_obstacle_m", 0.6)
        # 【2026-10-22追加】目的地を壁(占有セル)から「機体端で」この距離以上離れた点へ寄せる(patrol_planner)。
        # 壁に近すぎる目的地だと経路が作れず動けなくなるため。0以下で無効(従来どおり)。
        self.declare_parameter("goal_wall_margin_m", 0.30)
        self.declare_parameter("tick_hz", 1.0)
        self.declare_parameter("user_cancel_hold_sec", 10.0)
        # 【2026-07-17】ユーザー要望: person_target_detectorが追跡している人物の
        # map座標系位置(/nokosahen/tracked_person_world)から、この距離以上
        # 離れたフロンティアを優先する(人がいない方へ探索しに行く)。該当する
        # フロンティアが1つも無い場合は、探索が完全に止まらないよう従来通り
        # 全フロンティアから選ぶ(安全に倒すのではなく探索継続を優先)。
        self.declare_parameter("person_avoid_radius_m", 1.0)
        self.declare_parameter("person_avoid_timeout_sec", 2.0)
        # 【2026-09-30追加】ABORTED/タイムアウトしたフロンティアは、到達不能の
        # 可能性が高いのに毎回「最寄り」として選び直され同じ失敗を繰り返す
        # (無限ループ)ため、この半径・時間だけ候補から除外する。
        self.declare_parameter("blacklist_radius_m", 0.5)
        self.declare_parameter("blacklist_sec", 120.0)

        self.occupied_threshold = int(self.get_parameter("occupied_threshold").value)
        self.free_threshold = int(self.get_parameter("free_threshold").value)
        self.min_frontier_cells = int(self.get_parameter("min_frontier_cells").value)
        self.goal_timeout_sec = max(
            1.0, float(self.get_parameter("goal_timeout_sec").value)
        )
        self.goal_reached_tolerance_m = max(
            0.05, float(self.get_parameter("goal_reached_tolerance_m").value)
        )
        self.min_clearance_from_obstacle_m = max(
            0.0, float(self.get_parameter("min_clearance_from_obstacle_m").value)
        )
        self.goal_wall_margin_m = float(self.get_parameter("goal_wall_margin_m").value)
        self.person_avoid_radius_m = max(
            0.0, float(self.get_parameter("person_avoid_radius_m").value)
        )
        self.person_avoid_timeout_sec = max(
            0.1, float(self.get_parameter("person_avoid_timeout_sec").value)
        )
        self.blacklist_radius_m = max(0.0, float(self.get_parameter("blacklist_radius_m").value))
        self.blacklist_sec = max(0.0, float(self.get_parameter("blacklist_sec").value))
        # (x, y, 失効時刻[秒、self.get_clock()基準])
        self._blacklist: List[Tuple[float, float, float]] = []
        tick_hz = max(0.2, float(self.get_parameter("tick_hz").value))
        # 【2026-10-23追加】(B6)ダッシュボードの「中止」(/nokosahen/user_cancel)を受けたら、この秒数の間、
        # 新しい目的地を出さず一時停止する(以前は中止の約1秒後に次の目的地を出し直していた)。0以下で無効。
        # 専用トピックにした理由: 自分が無効化時に出す/nokosahen/cancel_goalと取り違えず、ユーザーの中止だけに反応するため。
        self.user_cancel_hold_sec = float(self.get_parameter("user_cancel_hold_sec").value)
        self._user_cancel = UserCancelHold(self.user_cancel_hold_sec)

        # 【2026-10-08】無効(既定)の間は /map と /tf を一切購読しない。実機で探索を無効にしていても
        # このノードがCPU 27%を使っていた(PythonのTransformListenerが高頻度の/tfを全件処理し、
        # 約42万セルの/mapも毎回Pythonオブジェクトへ変換していた)。有効化した時だけ購読を始め、
        # 無効化したら止める。有効な間の挙動は従来と同じ。
        self.tf_available = Buffer is not None
        self.tf_buffer = None
        self.tf_listener = None
        self._map_sub = None

        self._enabled = False
        self._latest_map: Optional[OccupancyGrid] = None
        self._hardware_estop = False
        self._soft_estop = False
        self._manual_override_raw = False
        self._last_manual_override_time = None
        self._boundary: dict = {"enabled": False}
        self._boundary_rect: Optional[dict] = None  # 回転矩形(enabled時のみ)
        self._keepout_zones: list = []
        self._current_goal: Optional[Tuple[float, float]] = None
        self._goal_sent_time = None
        self._status = "idle"  # "idle"(未有効) / "exploring" / "paused" / "complete"
        self._tracked_person: Optional[Tuple[float, float]] = None
        self._tracked_person_receive_time = None

        self.create_subscription(
            String, "/nokosahen/tracked_person_world", self._on_tracked_person, 5
        )
        self.create_subscription(Bool, "/estop_state", self._on_hardware_estop, 10)
        self.create_subscription(
            Bool, "/nokosahen/soft_estop", self._on_soft_estop, 10
        )
        self.create_subscription(
            Bool, "/nokosahen/manual_override", self._on_manual_override, 10
        )
        self.create_subscription(
            Bool, "/nokosahen/frontier_explorer/enabled", self._on_enabled, 5
        )
        self.create_subscription(Bool, "/nokosahen/user_cancel", self._on_user_cancel, 10)
        # 【2026-11-05】近接停止中は新しい目的地を出さず一時停止する(取り消しはルーター/directorが担当)
        self.create_subscription(Bool, "/nokosahen/near_person_stop", self._on_near_person_stop, 10)
        self.create_subscription(
            String, "/nokosahen/travel_boundary", self._on_boundary, 10
        )
        self.create_subscription(
            String, "/nokosahen/keepout_zones", self._on_keepout_zones, 10
        )
        # Nav2が経路計画不可等ですぐに失敗しても、25秒のタイムアウトを待たず
        # 即座に次のフロンティアへ切り替えられるようにする。
        self.create_subscription(
            String, "/nokosahen/nav2_goal_status", self._on_nav2_goal_status, 10
        )
        self._goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        # 【2026-10-07追加】探索を無効化(停止)したとき、すでにNav2へ送った目的地の走行を取り消すため。
        # nav2_goal_routerが購読していて、受信するとNav2のゴールをキャンセルしゼロ速度を出す。
        self._cancel_pub = self.create_publisher(Bool, "/nokosahen/cancel_goal", 10)
        self._status_pub = self.create_publisher(
            String, "/nokosahen/frontier_explorer/status", 5
        )
        self.create_timer(1.0 / tick_hz, self._tick)
        self.get_logger().info("frontier_explorer: 起動しました(既定は無効)")

    # --- 購読コールバック ---

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _start_sensing(self) -> None:
        """有効化時に/mapと/tfの購読を始める(冪等)。tfは毎回新しいBufferで始め、
        無効だった間の古い姿勢を使わない。"""
        if self._map_sub is None:
            self._map_sub = self.create_subscription(OccupancyGrid, "/map", self._on_map, 1)
        if self.tf_available and self.tf_listener is None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

    def _stop_sensing(self) -> None:
        """無効化時に/mapと/tfの購読を止め、保持している地図を捨てる(冪等)。"""
        if self._map_sub is not None:
            try:
                self.destroy_subscription(self._map_sub)
            except Exception:  # noqa: BLE001
                pass
            self._map_sub = None
        if self.tf_listener is not None:
            try:
                self.tf_listener.unregister()
            except Exception:  # noqa: BLE001
                pass
            self.tf_listener = None
            self.tf_buffer = None
        self._latest_map = None

    def _on_hardware_estop(self, msg: Bool) -> None:
        self._hardware_estop = bool(msg.data)

    def _on_soft_estop(self, msg: Bool) -> None:
        self._soft_estop = bool(msg.data)

    def _on_near_person_stop(self, msg: Bool) -> None:
        self._near_person_stop = bool(msg.data)
        self._near_person_stop_time = self.get_clock().now()

    @property
    def _near_person_blocking(self) -> bool:
        """近接停止中(trueを受信して1秒以内)ならTrue。未受信・falseはFalse。"""
        t = getattr(self, "_near_person_stop_time", None)
        if not getattr(self, "_near_person_stop", False) or t is None:
            return False
        return (self.get_clock().now() - t).nanoseconds / 1e9 <= 1.0

    def _on_user_cancel(self, msg: Bool) -> None:
        """ユーザー中止: 目的地を捨てて、user_cancel_hold_sec秒は新しい目的地を出さない。Nav2側の取り消しはルーターが行う。"""
        if not msg.data:
            return
        self._user_cancel.on_user_cancel(self.get_clock().now().nanoseconds / 1e9)
        self._current_goal = None
        self._goal_sent_time = None

    def _on_manual_override(self, msg: Bool) -> None:
        self._manual_override_raw = bool(msg.data)
        if self._manual_override_raw:
            self._last_manual_override_time = self.get_clock().now()

    def _on_boundary(self, msg: String) -> None:
        data = loads(msg.data, None)
        # 壊れたJSON/"enabled"の無いdictを「範囲解除」と取り違えない(2026-10-06)。
        if not isinstance(data, dict) or "enabled" not in data:
            return
        # enabled=trueなのに範囲キーが欠けていると、_within_boundaryが毎tick
        # KeyErrorで死に続ける(behavior_director.pyと同じ問題、2026-07-17
        # レビューで発見)。不正なpublishは無視して直前の状態を保つ。
        # 【2026-10-06】回転矩形対応: center_x/center_y/width/height/yaw(新形式)
        # または min/max(旧形式=yaw 0)のどちらでも読める。NaN等の不正値も無視する。
        rect = None
        if data.get("enabled"):
            rect = rect_from_state(data)
            if rect is None:
                self.get_logger().warn(
                    "travel_boundary: enabled=trueなのに範囲が不正/不足しています。無視します"
                )
                return
        self._boundary = data
        self._boundary_rect = rect

    def _on_keepout_zones(self, msg: String) -> None:
        data = loads(msg.data, [])
        if not isinstance(data, list):
            return
        valid = [
            z
            for z in data
            if isinstance(z, dict)
            and all(k in z for k in ("min_x", "max_x", "min_y", "max_y"))
        ]
        self._keepout_zones = valid

    def _on_tracked_person(self, msg: String) -> None:
        data = loads(msg.data, {})
        if not isinstance(data, dict):
            return
        if data.get("tracked") and "x" in data and "y" in data:
            try:
                self._tracked_person = (float(data["x"]), float(data["y"]))
            except (TypeError, ValueError):
                return
            self._tracked_person_receive_time = self.get_clock().now()
        else:
            self._tracked_person = None

    @property
    def _fresh_tracked_person(self) -> Optional[Tuple[float, float]]:
        if self._tracked_person is None or self._tracked_person_receive_time is None:
            return None
        elapsed = (
            self.get_clock().now() - self._tracked_person_receive_time
        ).nanoseconds / 1e9
        if elapsed > self.person_avoid_timeout_sec:
            return None
        return self._tracked_person

    def _on_nav2_goal_status(self, msg: String) -> None:
        # 【2026-08-06修正】以前はこのノード自身が送ったゴールかどうかを
        # 区別せず、他の送信元(behavior_director等)の完了通知でも
        # self._current_goalを無条件でクリアしていた。両者が同時に稼働して
        # いると、互いの完了通知で相手がまだ追いかけているゴールを誤って
        # クリアし合う競合が起きていた(nav2_goal_router.py側のコメントで
        # 既知の課題として記録されていた)。nav2_goal_routerがpublishする
        # target_x/target_yが自分のcurrent_goalと一致する場合だけクリアする。
        if self._current_goal is None:
            return
        data = loads(msg.data, {})
        if not isinstance(data, dict):
            return
        target_x = data.get("target_x")
        target_y = data.get("target_y")
        if target_x is None or target_y is None:
            return
        if (
            math.isclose(float(target_x), self._current_goal[0], abs_tol=1e-3)
            and math.isclose(float(target_y), self._current_goal[1], abs_tol=1e-3)
        ):
            # Nav2がABORTED(6)で失敗した、または受理されなかったゴールは、
            # 到達不能の可能性が高いのでしばらく候補から外す。
            # (CANCELED=5はestop/手動運転による取り消しなので対象外)
            event = data.get("event")
            status = data.get("status")
            if event == "rejected" or (event == "finished" and status == 6):
                self._blacklist_frontier(self._current_goal)
            self._current_goal = None
            self._goal_sent_time = None

    def _blacklist_frontier(self, xy: Optional[Tuple[float, float]]) -> None:
        if xy is None or self.blacklist_sec <= 0.0:
            return
        expiry = self.get_clock().now().nanoseconds / 1e9 + self.blacklist_sec
        self._blacklist.append((xy[0], xy[1], expiry))
        self.get_logger().warn(
            f"frontier_explorer: フロンティア({xy[0]:.2f}, {xy[1]:.2f})を"
            f"{self.blacklist_sec:.0f}秒間候補から除外します"
        )

    def _is_blacklisted(self, x: float, y: float, now_sec: float) -> bool:
        self._blacklist = [b for b in self._blacklist if b[2] > now_sec]
        return any(
            math.hypot(x - bx, y - by) <= self.blacklist_radius_m
            for bx, by, _ in self._blacklist
        )

    def _publish_cancel_goal(self) -> None:
        """Nav2へ送信済みの目的地を取り消す(nav2_goal_routerが/nokosahen/cancel_goalを購読)。
        取りこぼしに備えて短い間隔で2回送る(取り消しは何度送っても安全)。"""
        msg = Bool()
        msg.data = True
        try:
            self._cancel_pub.publish(msg)
            self._cancel_pub.publish(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cancel_goalの送信に失敗: {exc}")

    def _on_enabled(self, msg: Bool) -> None:
        self._enabled = bool(msg.data)
        if not self._enabled:
            self._current_goal = None
            self._goal_sent_time = None
            self._status = "idle"
            self.get_logger().info("frontier_explorer: 無効化しました")
            # 以前は内部状態を空にするだけで、Nav2へ送信済みの目的地は取り消されず、探索を止めても
            # ロボットが最後の目的地へ走り続けていた(実機で77秒間)。必ず取り消す。
            self._publish_cancel_goal()
            self._blacklist = []
            self._stop_sensing()
        else:
            self._start_sensing()
            self._status = "exploring"
            self.get_logger().info("frontier_explorer: 有効化しました。探索を開始します")
        # _tick()は「有効な間だけ」状態を配信する作りなので、無効化した瞬間は
        # ここで明示的に配信しないと、ダッシュボードが最後の(古い)状態を
        # 表示し続けてしまう(「停止したのに反映されない」ように見える)。
        self._publish_status()

    @property
    def _estop_active(self) -> bool:
        return self._hardware_estop or self._soft_estop

    @property
    def _manual_override(self) -> bool:
        # behavior_directorと同じ「一定時間再送されなければ自動失効」の考え方。
        if not self._manual_override_raw or self._last_manual_override_time is None:
            return False
        elapsed = (
            self.get_clock().now() - self._last_manual_override_time
        ).nanoseconds / 1e9
        return elapsed <= 1.0

    def _get_pose(self) -> Optional[Tuple[float, float]]:
        if not self.tf_available or self.tf_buffer is None:
            return None
        try:
            import rclpy.time

            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    # --- フロンティア探索 ---

    def _find_frontiers(
        self, grid: OccupancyGrid, robot_xy: Tuple[float, float]
    ) -> List[Tuple[float, float, int]]:
        """既知(自由)セルのうち、未知セルに隣接しているものを「フロンティア」
        として抽出し、隣接するフロンティアセル同士を1つの塊にまとめる。
        戻り値は各塊の(中心x, 中心y, セル数)のリスト。
        """
        width = grid.info.width
        height = grid.info.height
        data = np.array(grid.data, dtype=np.int16).reshape(height, width)

        free = (data >= 0) & (data <= self.free_threshold)
        unknown = data < 0
        occupied = data >= self.occupied_threshold

        # 4近傍のいずれかが未知である自由セルをフロンティア候補にする。
        frontier_mask = np.zeros_like(free)
        frontier_mask[:-1, :] |= free[:-1, :] & unknown[1:, :]
        frontier_mask[1:, :] |= free[1:, :] & unknown[:-1, :]
        frontier_mask[:, :-1] |= free[:, :-1] & unknown[:, 1:]
        frontier_mask[:, 1:] |= free[:, 1:] & unknown[:, :-1]

        # フロンティアが壁際すぎると、Nav2のコストマップ膨張(inflation_radius)
        # の範囲内にゴールが入ってしまい経路計画が失敗し続ける(実機で確認済み
        # の不具合)。占有セルをmin_clearance_from_obstacle_m分だけ膨張させ、
        # その範囲に入るフロンティアセルは候補から除外する。
        resolution = grid.info.resolution
        clearance_cells = (
            int(math.ceil(self.min_clearance_from_obstacle_m / resolution))
            if resolution > 0
            else 0
        )
        inflated_occupied = occupied.copy()
        for _ in range(clearance_cells):
            shifted = inflated_occupied.copy()
            shifted[:-1, :] |= inflated_occupied[1:, :]
            shifted[1:, :] |= inflated_occupied[:-1, :]
            shifted[:, :-1] |= inflated_occupied[:, 1:]
            shifted[:, 1:] |= inflated_occupied[:, :-1]
            inflated_occupied = shifted
        frontier_mask &= ~inflated_occupied

        visited = np.zeros_like(frontier_mask)
        clusters: List[Tuple[float, float, int]] = []
        rows, cols = np.where(frontier_mask)
        cell_set = set(zip(rows.tolist(), cols.tolist()))

        for start in list(cell_set):
            if visited[start]:
                continue
            # 単純なBFSで隣接フロンティアセルをまとめる(斜め隣接も1塊とみなす)。
            stack = [start]
            visited[start] = True
            members = []
            while stack:
                r, c = stack.pop()
                members.append((r, c))
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        nr, nc = r + dr, c + dc
                        if (nr, nc) in cell_set and not visited[nr, nc]:
                            visited[nr, nc] = True
                            stack.append((nr, nc))
            if len(members) < self.min_frontier_cells:
                continue
            mean_r = sum(m[0] for m in members) / len(members)
            mean_c = sum(m[1] for m in members) / len(members)
            world_x = grid.info.origin.position.x + (mean_c + 0.5) * grid.info.resolution
            world_y = grid.info.origin.position.y + (mean_r + 0.5) * grid.info.resolution
            clusters.append((world_x, world_y, len(members)))

        return clusters

    def _wall_safe_target(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        """目的地を壁から機体端でgoal_wall_margin_m以上離れた点へ寄せる(patrol_planner)。
        無効(0以下)・地図が取れない・寄せ先が無いときはNone(呼び出し側は従来どおりの点を使う)。
        範囲(travel_boundary)が有効なら、その内側へ寄せる。"""
        margin = getattr(self, "goal_wall_margin_m", 0.0)
        if margin <= 0.0 or self._latest_map is None:
            return None
        grid = grid_from_msg(self._latest_map)
        if grid is None:
            return None
        rect = self._boundary_rect if self._boundary.get("enabled") else None
        res = snap_to_drivable(grid, x, y, margin_m=margin, rect=rect)
        if res["status"] in ("ok", "snapped"):
            return float(res["x"]), float(res["y"])
        return None

    def _within_boundary(self, x: float, y: float) -> bool:
        b = self._boundary
        if not b.get("enabled"):
            return True
        rect = self._boundary_rect
        if rect is None:
            return False  # enabledなのに矩形が無い状態は安全側(候補を採用しない)
        return point_in_rect(x, y, rect)

    def _in_keepout_zone(self, x: float, y: float) -> bool:
        return any(
            z["min_x"] <= x <= z["max_x"] and z["min_y"] <= y <= z["max_y"]
            for z in self._keepout_zones
        )

    def _publish_status(self, extra: Optional[dict] = None) -> None:
        payload = {"status": self._status, "enabled": self._enabled}
        if extra:
            payload.update(extra)
        msg = String()
        msg.data = dumps(payload)
        self._status_pub.publish(msg)

    def _tick(self) -> None:
        if not self._enabled:
            # 【2026-07-21追加】単に起動直後の1回だけpublishする方式だと、DDSの
            # discovery(publisher/subscriber同士がお互いを見つけるまでの
            # 時間差)によってその1回が届かない可能性が残る(届かなければ、
            # 次にenabled状態が変化するまで二度と送られず、ダッシュボード側の
            # 古い表示が直らないまま)。tick_hz(既定1Hz程度)ごとに無効状態も
            # 送り続けることで、discoveryが完了し次第すぐに(遅くとも次の
            # tickまでに)正しい状態へ収束することを保証する。
            self._publish_status()
            return
        if self._estop_active:
            self._status = "paused"
            self._publish_status({"reason": "estop"})
            return
        if self._manual_override:
            # 手動運転(WASD)を最優先する方針。新しいゴールは送らない
            # (既存ゴールのキャンセルはnav2_goal_router側が担当する)。
            self._status = "paused"
            self._current_goal = None
            self._goal_sent_time = None
            self._publish_status({"reason": "manual_override"})
            return
        if self._near_person_blocking:
            # 【2026-11-05】近接停止中は新しい目的地を出さない。今の目的地は捨てる(走行の取り消しはルーターが担当)。
            self._status = "paused"
            self._current_goal = None
            self._goal_sent_time = None
            self._publish_status({"reason": "near_person_stop"})
            return
        if self._user_cancel is not None and self._user_cancel.pending and self._user_cancel.active(self.get_clock().now().nanoseconds / 1e9):
            self._status = "paused"
            self._current_goal = None
            self._goal_sent_time = None
            self._publish_status({"reason": "user_cancel"})
            return
        if self._latest_map is None:
            # 【2026-08-06修正】ここでstatusを配信せずにreturnしていたため、
            # 地図がまだ無い間はダッシュボードが直前の状態(古い"exploring"等)を
            # 表示し続け、「探索onにしたのに動かない・理由が分からない」症状の
            # 原因になっていた(実機で確認)。
            self._status = "paused"
            self._publish_status({"reason": "waiting_for_map"})
            return
        robot_xy = self._get_pose()
        if robot_xy is None:
            self._status = "paused"
            self._publish_status({"reason": "no_pose"})
            return

        now = self.get_clock().now()
        if self._current_goal is not None and self._goal_sent_time is not None:
            elapsed = (now - self._goal_sent_time).nanoseconds / 1e9
            dist = math.hypot(
                robot_xy[0] - self._current_goal[0], robot_xy[1] - self._current_goal[1]
            )
            if dist <= self.goal_reached_tolerance_m:
                self._current_goal = None
                self._goal_sent_time = None
            elif elapsed >= self.goal_timeout_sec:
                self.get_logger().warn(
                    "frontier_explorer: ゴールに時間がかかりすぎたため次のフロンティアへ切り替えます"
                )
                self._blacklist_frontier(self._current_goal)
                self._current_goal = None
                self._goal_sent_time = None
            else:
                self._status = "exploring"
                self._publish_status({"target": self._current_goal})
                return  # 現在のゴールへ向かっている途中なので何もしない

        clusters = self._find_frontiers(self._latest_map, robot_xy)
        clusters = [
            c
            for c in clusters
            if self._within_boundary(c[0], c[1]) and not self._in_keepout_zone(c[0], c[1])
        ]
        if not clusters:
            self._status = "complete"
            self._publish_status({"reason": "no_more_frontiers"})
            return

        # ABORTED/タイムアウト済みのフロンティア近傍を除外する。除外した結果
        # 候補が残らない場合は、到達可能なフロンティアはもう無い=完了扱い。
        now_sec = self.get_clock().now().nanoseconds / 1e9
        reachable = [c for c in clusters if not self._is_blacklisted(c[0], c[1], now_sec)]
        if not reachable:
            self._status = "complete"
            self._publish_status(
                {"reason": "only_blacklisted_frontiers", "blacklisted": len(self._blacklist)}
            )
            return
        clusters = reachable

        # 追跡中の人物から離れたフロンティアを優先する(人がいない方へ探索しに
        # 行く、2026-07-17)。該当するものが1つも無ければ探索を止めないよう
        # 全フロンティアにフォールバックする。
        tracked_person = self._fresh_tracked_person
        if tracked_person is not None:
            far_clusters = [
                c
                for c in clusters
                if math.hypot(c[0] - tracked_person[0], c[1] - tracked_person[1])
                >= self.person_avoid_radius_m
            ]
            if far_clusters:
                clusters = far_clusters

        clusters.sort(key=lambda c: math.hypot(c[0] - robot_xy[0], c[1] - robot_xy[1]))
        target_x, target_y, _ = clusters[0]

        goal = PoseStamped()
        goal.header.stamp = now.to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = target_x
        goal.pose.position.y = target_y
        # 【2026-10-10変更】目的地の向きは「進行方向(今の位置から目的地への方位)」にする。以前は常にyaw 0(map座標の+x向き)で、
        # Nav2が到着ごとにその向きへ余計に旋回していた(自律走行の地図クリックは進行方向で送られる)。位置の選び方は不変。
        yaw = math.atan2(target_y - robot_xy[1], target_x - robot_xy[0])
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self._goal_pub.publish(goal)
        self._current_goal = (target_x, target_y)
        self._goal_sent_time = now
        self._status = "exploring"
        self._publish_status({"target": self._current_goal, "frontier_count": len(clusters)})


def main() -> None:
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
