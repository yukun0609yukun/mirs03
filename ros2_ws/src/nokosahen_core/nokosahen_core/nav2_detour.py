"""nav2_goal_routerの「迂回の処理」(2026-10-25)。Nav2の目的地がBTの回復でも失敗して中止(status 6)になったとき、
後退せずに、経路をシミュレーション(path_sim)して越せると分かってから進む。流れ・パラメータ: docs/detour.md

状態の流れはDetourMachine(detour_machine.py、ROS不要)が決め、ここは「計画(/compute_path_to_pose)・シミュレーション・送信」を行うだけ。
速度指令は、これまでどおりNav2の目的地(安全層を通る)と、先回し旋回(Nav2のSpin、失敗時は/cmd_vel_navの代替の回頭)だけ。このファイルは/cmd_velへ直接出さない。

Nav2GoalRouterのミックスイン。__init__を通さない単体テストでも動くよう、状態の既定値はクラス変数で持つ。
"""
from __future__ import annotations

import math
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from std_msgs.msg import String

from . import path_sim
from .detour_machine import DetourMachine
from .json_tools import dumps
from .map_orientation import point_in_rect

ABORTED = 6


def yaw_to_quaternion_zw(yaw: float) -> Tuple[float, float]:
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def path_to_xy(poses) -> List[Tuple[float, float]]:
    out = []
    for p in poses:
        try:
            x, y = float(p.pose.position.x), float(p.pose.position.y)
        except (AttributeError, TypeError, ValueError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            out.append((x, y))
    return out


def end_heading(path: List[Tuple[float, float]], chord: float = 0.3) -> Optional[float]:
    """経路の終わりの進行方向(最後の点から、chord手前の点への逆向き)。"""
    if len(path) < 2:
        return None
    ex, ey = path[-1]
    for px, py in reversed(path[:-1]):
        if math.hypot(ex - px, ey - py) >= chord:
            return math.atan2(ey - py, ex - px)
    px, py = path[0]
    if math.hypot(ex - px, ey - py) < 1e-6:
        return None
    return math.atan2(ey - py, ex - px)


class DetourMixin:
    _detour = None                 # DetourMachine。Noneなら迂回の処理は無効
    _detour_enabled = False
    _detour_seq = 0                # 迂回の処理を始めたときの_goal_seq。変わったら(取り消し・新しい目的地)中止する
    _detour_goal_pose = None
    _detour_paths = None
    _detour_cands = None
    _detour_heading = None         # 次に送る目的地の最初の進行方向(map座標のrad)。旋回の要否に使う
    _compute_client = None
    _compute_action = None
    _active_goal_seq = None        # 直近に受理されたNav2の目的地の世代。取り消し・新しい目的地が来ていれば_goal_seqと違う
    _nav_pose = None               # 直近に_send_navigateへ渡した目的地
    _detour_max_points = 150
    _detour_lookahead = 4.0
    _detour_margin = 0.03
    _detour_offsets = (0.7, 1.0, 1.4)
    _detour_pre_rotate = math.radians(20.0)

    # ---- 初期化 ----
    def _detour_init(self) -> None:
        self.declare_parameter("detour_enabled", True)
        self.declare_parameter("detour_max_attempts", 2)
        self.declare_parameter("detour_settle_sec", 1.0)
        self.declare_parameter("detour_lookahead_m", 4.0)
        self.declare_parameter("detour_max_points", 150)
        self.declare_parameter("detour_margin_m", 0.03)
        self.declare_parameter("detour_offsets", [0.7, 1.0, 1.4])
        self.declare_parameter("detour_pre_rotate_deg", 20.0)
        self._detour_enabled = bool(self.get_parameter("detour_enabled").value)
        if not self._detour_enabled:
            self.get_logger().info("迂回の処理は無効です(detour_enabled=false)")
            return
        try:
            from nav2_msgs.action import ComputePathToPose  # type: ignore

            self._compute_action = ComputePathToPose
            self._compute_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"迂回の処理は使えません(ComputePathToPoseが無い): {exc}")
            self._detour_enabled = False
            return
        self._detour_max_points = max(20, int(self.get_parameter("detour_max_points").value))
        self._detour_lookahead = float(self.get_parameter("detour_lookahead_m").value)
        self._detour_margin = max(0.0, float(self.get_parameter("detour_margin_m").value))
        self._detour_offsets = tuple(float(v) for v in self.get_parameter("detour_offsets").value)
        self._detour_pre_rotate = math.radians(float(self.get_parameter("detour_pre_rotate_deg").value))
        self._detour = DetourMachine(
            max_attempts=int(self.get_parameter("detour_max_attempts").value),
            settle_sec=float(self.get_parameter("detour_settle_sec").value),
        )
        self.create_timer(0.1, self._detour_tick)

    # ---- ルーター本体から呼ばれる入口 ----
    def _detour_handle_result(self, status: int, goal_xy: Tuple[float, float]) -> bool:
        """Nav2の目的地の結果が届いたとき。迂回の処理が扱ったらTrue(ルーターは「finished」を出さない)。"""
        m = self._detour
        if m is None:
            return False
        if m.active:
            if not m.sending:
                return False
            if self._goal_seq != self._detour_seq:
                self._detour_apply(m.cancel("取り消し/新しい目的地"))
                return True
            self._detour_apply(m.goal_done(m.sid, int(status), time.monotonic()))
            return True
        if status != ABORTED or not self._detour_enabled:
            return False
        if self._detour_blocked_reason() is not None:
            return False
        pose = self._nav_pose
        if pose is None or not self._pose_matches(pose, goal_xy):
            return False
        frame = str(getattr(pose.header, "frame_id", "") or "")
        if frame not in ("map", ""):
            return False
        if self._active_goal_seq != self._goal_seq:
            return False   # この目的地を受理したあとに、取り消し・新しい目的地が来ている
        if self._compute_client is None or not self._compute_client.server_is_ready() or not self._client.server_is_ready():
            self.get_logger().warn("迂回の処理を省きます(Nav2のアクションサーバーが準備できていません)")
            return False
        self._detour_seq = self._goal_seq
        self._detour_goal_pose = pose
        self._detour_paths = {}
        self._detour_cands = []
        self._detour_heading = None
        self.get_logger().info(
            f"Nav2の目的地が回復でも中止になったため、迂回の処理を始めます(目的地 {goal_xy[0]:.2f}, {goal_xy[1]:.2f})。後退はしません"
        )
        self._detour_apply(m.start(goal_xy, time.monotonic()))
        return True

    @staticmethod
    def _pose_matches(pose, goal_xy) -> bool:
        return abs(pose.pose.position.x - goal_xy[0]) < 1e-6 and abs(pose.pose.position.y - goal_xy[1]) < 1e-6

    def _detour_send_error(self, reason: str) -> None:
        """迂回の処理が送った目的地が、拒否された/結果が取れなかった。後退せず諦める。"""
        m = self._detour
        if m is not None and m.sending:
            self._detour_apply(m.send_failed(m.sid, reason))

    # ---- 中止の条件 ----
    def _detour_blocked_reason(self) -> Optional[str]:
        """迂回の処理を始めない/続けない理由。無ければNone。緊急停止・アンケート・手動運転・調停ノードの取り消し(近接停止等)。"""
        if self._estop_active:
            return "estop"
        if self._survey_blocking:
            return "survey"
        if getattr(self, "_wasd_active_prev", False):
            return "manual"
        grant_time = getattr(self, "_grant_time", None)
        if grant_time is not None and time.monotonic() - grant_time <= 1.0:
            owner = self._grant_owner
            # 【2026-11-04】"near_person_face_turn" も近接停止の持ち主(前方一致)。見落とすと近接停止中も迂回が続く。
            if isinstance(owner, str) and (owner in ("estop", "manual") or owner.startswith("near_person")):
                return "near_person" if owner.startswith("near_person") else owner
        return None

    def _detour_tick(self) -> None:
        m = self._detour
        if m is None or not m.active:
            return
        reason = self._detour_blocked_reason()
        if reason is not None:
            # 緊急停止・アンケート・手動運転は、それぞれの処理が既にNav2の目的地を取り消している。近接停止は、走っている目的地も止める。
            if reason == "near_person" and m.sending:
                self._cancel_active_goal("近接停止のため迂回を中止")
            self.get_logger().warn(f"迂回の処理を中止します({reason})")
            self._detour_apply(m.cancel(reason))
            return
        if self._goal_seq != self._detour_seq:
            self._detour_apply(m.cancel("取り消し/新しい目的地"))
            return
        self._detour_apply(m.tick(time.monotonic()))

    # ---- アクションの実行 ----
    def _detour_apply(self, actions) -> None:
        for act in actions:
            kind = act[0]
            if kind == "plan":
                self._detour_plan(act[1], act[2], act[3])
            elif kind == "candidates":
                self._detour_make_candidates()
            elif kind == "check":
                self._detour_check(act[1], act[2])
            elif kind == "send":
                self._detour_send(act[1], act[2])
            elif kind == "fail":
                self._detour_finish_fail(str(act[1]))
            elif kind == "done":
                self.get_logger().info("迂回の処理で目的地に着きました")
                self._detour_finish(4)
            elif kind == "cancelled":
                self.get_logger().info(f"迂回の処理を取り消しました({act[1]})")
                self._detour_finish(5)

    def _detour_goal_xy(self) -> Optional[Tuple[float, float]]:
        return self._detour.goal_xy if self._detour is not None else None

    def _detour_finish(self, status: int) -> None:
        self._detour_paths = None
        self._detour_cands = None
        xy = self._detour_goal_xy()
        self._publish_goal_status("finished", status=status, target=xy)

    def _detour_finish_fail(self, reason: str) -> None:
        xy = self._detour_goal_xy()
        self.get_logger().warn(f"迂回の処理が失敗しました({reason})。後退せず、この目的地を諦めます")
        payload = {"event": "detour_failed", "status": ABORTED, "reason": reason}
        if xy is not None:
            payload["target_x"], payload["target_y"] = xy
        msg = String()
        msg.data = dumps(payload)
        self._goal_status_pub.publish(msg)
        self._detour_finish(ABORTED)   # 従来の消費側(探索の除外・wall_escape等)は「finished 6」を見るので、続けて従来どおりの中止も出す

    # ---- 位置・LiDAR ----
    def _robot_pose_map(self) -> Optional[Tuple[float, float, float]]:
        bp = self._base_pose()  # 2026-10-26: /tfの直接購読をやめ、pose_relayの /nokosahen/robot_pose から(nav2_goal_router._base_pose)
        if bp is None or bp[3] > 1.5:
            return None
        x, y, yaw, _age = bp
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return None
        return (x, y, yaw)

    def _detour_scan_map(self, pose, max_range: float) -> Optional[List[Tuple[float, float]]]:
        """最新の/scanをmap座標の点にして、間引く。古い/無いときはNone。迂回の処理の間だけ呼ぶ(通常走行ではCPUを使わない)。"""
        msg = self._scan_msg
        if msg is None or self._scan_xy_time is None or time.monotonic() - self._scan_xy_time > 1.0:
            return None
        px, py, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        pts = []
        lo = max(float(msg.range_min), 0.05)
        for i, r in enumerate(msg.ranges):
            if r is None or not math.isfinite(r) or r < lo or r > max_range:
                continue
            a = msg.angle_min + i * msg.angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            pts.append((px + c * lx - s * ly, py + s * lx + c * ly))
        return path_sim.decimate_points(pts, self._detour_max_points)

    # ---- 計画 ----
    def _detour_wp_pose(self, idx: int):
        wx, wy = self._detour_cands[idx]
        gx, gy = self._detour.goal_xy
        return self._detour_make_pose(wx, wy, math.atan2(gy - wy, gx - wx))

    def _detour_make_pose(self, x: float, y: float, yaw: float):
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = yaw_to_quaternion_zw(yaw)
        return p

    def _detour_plan(self, tag: str, target: str, idx: int) -> None:
        m = self._detour
        sid = m.sid
        try:
            goal = self._compute_action.Goal()
            if target == "goal":
                goal.goal = self._detour_goal_pose
            else:
                goal.goal = self._detour_wp_pose(idx)
            goal.planner_id = "GridBased"
            if tag == "wp_b":
                goal.start = self._detour_wp_pose(idx)   # 中継点から元の目的地へ
                goal.use_start = True
            else:
                goal.use_start = False   # 機体の今の位置から
            if not self._compute_client.server_is_ready():
                raise RuntimeError("compute_path_to_poseが準備できていません")
            fut = self._compute_client.send_goal_async(goal)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"迂回の経路計画を送れません({tag}): {exc}")
            self._detour_plan_done(sid, tag, None)
            return
        fut.add_done_callback(lambda f, sid=sid, tag=tag: self._on_detour_plan_response(f, sid, tag))

    def _on_detour_plan_response(self, future, sid: int, tag: str) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"迂回の経路計画の送信に失敗({tag}): {exc}")
            self._detour_plan_done(sid, tag, None)
            return
        if handle is None or not handle.accepted:
            self._detour_plan_done(sid, tag, None)
            return
        handle.get_result_async().add_done_callback(lambda f, sid=sid, tag=tag: self._on_detour_plan_result(f, sid, tag))

    def _on_detour_plan_result(self, future, sid: int, tag: str) -> None:
        path = None
        try:
            res = future.result()
            if int(res.status) == 4:
                xy = path_to_xy(res.result.path.poses)
                if len(xy) >= 2:
                    path = xy
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"迂回の経路計画の結果取得に失敗({tag}): {exc}")
        self._detour_plan_done(sid, tag, path)

    def _detour_plan_done(self, sid: int, tag: str, path) -> None:
        m = self._detour
        if m is None or sid != m.sid or not m.active or self._detour_paths is None:
            return
        if path is not None:
            self._detour_paths[tag] = path
        self._detour_apply(m.plan_result(sid, tag, path is not None, time.monotonic()))

    # ---- シミュレーション ----
    def _detour_make_candidates(self) -> None:
        m = self._detour
        pose = self._robot_pose_map()
        pts = self._detour_scan_map(pose, 3.5) if pose is not None else None
        cands: list = []
        if pose is not None and pts is not None:
            cands = path_sim.detour_waypoints(pts, pose, m.goal_xy, offsets=self._detour_offsets, margin=self._detour_margin)
            rect = getattr(self, "_boundary_rect", None)
            if rect is not None:
                cands = [p for p in cands if point_in_rect(p[0], p[1], rect)]   # 走行可能範囲の外の中継点は使わない
        else:
            self.get_logger().warn("迂回の中継点を作れません(機体の位置かLiDARが取れません)")
        self._detour_cands = cands
        self._detour_apply(m.candidates(m.sid, len(cands), time.monotonic()))

    def _detour_check(self, kind: str, idx: int) -> None:
        m = self._detour
        sid = m.sid
        pose = self._robot_pose_map()
        pts = self._detour_scan_map(pose, self._detour_lookahead + 1.0) if pose is not None else None
        if pose is None or pts is None:
            self._detour_apply(m.verdict(sid, False, time.monotonic(), "no_pose_or_scan"))
            return
        kw = dict(margin=self._detour_margin, lookahead_m=self._detour_lookahead)
        paths = self._detour_paths or {}
        if kind == "direct":
            path = paths.get("direct")
            d = path_sim.first_collision_on_path(path, pose, pts, 0.0, **kw) if path else 0.0
            self._detour_heading = path_sim.path_heading(path, pose) if path else None
            reason = "" if d is None else f"collision_at_{d:.2f}m"
        else:
            pa, pb = paths.get("wp_a"), paths.get("wp_b")
            if not pa or not pb:
                self._detour_apply(m.verdict(sid, False, time.monotonic(), "no_path"))
                return
            d = path_sim.first_collision_on_path(pa, pose, pts, 0.0, **kw)
            reason = "" if d is None else f"wp_leg1_collision_at_{d:.2f}m"
            if d is None:
                hb = end_heading(pa)
                start_b = (pb[0][0], pb[0][1], hb if hb is not None else pose[2])
                d = path_sim.first_collision_on_path(pb, start_b, pts, 0.0, **kw)
                reason = "" if d is None else f"wp_leg2_collision_at_{d:.2f}m"
            self._detour_heading = path_sim.path_heading(pa, pose)
        self._detour_apply(m.verdict(sid, d is None, time.monotonic(), reason))

    # ---- 送信 ----
    def _detour_send(self, target: str, idx: int) -> None:
        m = self._detour
        sid = m.sid
        if target == "goal":
            pose = self._detour_goal_pose
            heading = self._detour_heading if m.state == "send_direct" else None   # 中継点のあとの目的地は、旋回しない
        else:
            pose = self._detour_wp_pose(idx)
            heading = self._detour_heading
        delta = self._detour_rotation_delta(heading)
        if delta is not None:
            if not self._rotate_granted():
                self._detour_apply(m.send_failed(sid, "rotate_not_granted"))
                return
            self.get_logger().info(f"迂回: 経路の向きへ先にその場で旋回します(相対{math.degrees(delta):.0f}度)")
            self._detour_spin_then_navigate(pose, delta)
            return
        self._send_navigate(pose)

    def _detour_rotation_delta(self, heading: Optional[float]) -> Optional[float]:
        if heading is None or not math.isfinite(heading):
            return None
        pose = self._robot_pose_map()
        if pose is None:
            return None
        delta = math.atan2(math.sin(heading - pose[2]), math.cos(heading - pose[2]))
        return delta if abs(delta) >= self._detour_pre_rotate else None

    def _detour_spin_then_navigate(self, pose, delta: float) -> None:
        """先回し旋回(Nav2のSpin)。完了したら_on_spin_resultが目的地を送る。Spinが使えない/失敗なら、既存の代替の回頭(/cmd_vel_nav)。"""
        seq = self._goal_seq
        if self._spin_client is None or not self._spin_client.server_is_ready():
            self._begin_fallback_rotation(pose, seq)
            return
        self._rotating_hint_until = time.monotonic() + 1.0
        spin_goal = self._spin_action.Goal()
        spin_goal.target_yaw = float(delta)
        spin_goal.time_allowance = Duration(seconds=self._pre_rotate_allowance).to_msg()
        fut = self._spin_client.send_goal_async(spin_goal)
        fut.add_done_callback(lambda f, pose=pose, seq=seq: self._on_spin_response(f, pose, seq))
