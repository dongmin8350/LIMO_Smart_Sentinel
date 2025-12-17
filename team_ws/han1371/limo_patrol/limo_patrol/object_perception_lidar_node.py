#!/usr/bin/env python3
import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from nav_msgs.msg import OccupancyGrid

import tf2_ros
import requests


@dataclass
class Track:
    tid: int
    x: float      # ✅ map frame x
    y: float      # ✅ map frame y
    vx: float     # ✅ map frame vx
    vy: float     # ✅ map frame vy
    last_t: float
    seen_t: float
    missed: int


class ObjectPerceptionLidarNode(Node):
    """
    - /scan 전방 포인트 -> 클러스터링 -> centroid 추출
    - (옵션) 맵 필터: 맵 occupied(벽/기둥/구조물) 위의 점 제거
    - ✅ centroid를 map frame으로 변환해서 track 업데이트/속도 계산 (로봇 이동에 따른 오탐 감소)
    - moving(속도) 판단 시 정지 + 텔레그램 1회
    - clear 시 WAITING_RESUME(자동 재개 X)
    - /patrol/resume=True 수동 재개
    """

    def __init__(self):
        super().__init__('object_perception_lidar_node')

        # --------- Parameters ---------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('front_angle_deg', 30.0)
        self.declare_parameter('min_range_m', 0.12)
        self.declare_parameter('obstacle_range_m', 0.70)
        self.declare_parameter('stop_publish_hz', 10.0)

        # Map filter
        self.declare_parameter('use_map_filter', True)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('laser_frame', '')  # 비우면 scan.header.frame_id 사용
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('map_inflation_m', 0.15)
        self.declare_parameter('treat_unknown_as_occupied', True)

        # Clustering
        self.declare_parameter('cluster_dist_m', 0.18)
        self.declare_parameter('cluster_min_pts', 4)
        self.declare_parameter('cluster_min_width', 0.10)
        self.declare_parameter('cluster_max_width', 1.20)

        # Tracking / motion
        self.declare_parameter('track_match_dist_m', 0.45)
        self.declare_parameter('track_forget_sec', 1.0)
        self.declare_parameter('motion_speed_mps', 0.15)
        self.declare_parameter('motion_hold_sec', 0.4)

        # Stop/Resume hysteresis
        self.declare_parameter('clear_confirm_sec', 0.6)

        # Manual resume
        self.declare_parameter('resume_topic', '/patrol/resume')
        self.declare_parameter('require_cleared_before_resume', True)

        # Telegram
        self.declare_parameter('telegram_enable', True)
        self.declare_parameter('telegram_bot_token', '')
        self.declare_parameter('telegram_chat_id', '')
        self.declare_parameter('telegram_prefix', '[LIMO]')

        # --------- Load ---------
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value

        self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)
        self.min_range_m = float(self.get_parameter('min_range_m').value)
        self.obstacle_range_m = float(self.get_parameter('obstacle_range_m').value)
        self.stop_publish_hz = float(self.get_parameter('stop_publish_hz').value)

        self.use_map_filter = bool(self.get_parameter('use_map_filter').value)
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.laser_frame_param = self.get_parameter('laser_frame').get_parameter_value().string_value
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.map_inflation_m = float(self.get_parameter('map_inflation_m').value)
        self.treat_unknown_as_occupied = bool(self.get_parameter('treat_unknown_as_occupied').value)

        self.cluster_dist_m = float(self.get_parameter('cluster_dist_m').value)
        self.cluster_min_pts = int(self.get_parameter('cluster_min_pts').value)
        self.cluster_min_width = float(self.get_parameter('cluster_min_width').value)
        self.cluster_max_width = float(self.get_parameter('cluster_max_width').value)

        self.match_dist = float(self.get_parameter('track_match_dist_m').value)
        self.track_forget_sec = float(self.get_parameter('track_forget_sec').value)
        self.motion_speed = float(self.get_parameter('motion_speed_mps').value)
        self.motion_hold_sec = float(self.get_parameter('motion_hold_sec').value)

        self.clear_confirm_sec = float(self.get_parameter('clear_confirm_sec').value)

        self.resume_topic = self.get_parameter('resume_topic').get_parameter_value().string_value
        self.require_cleared_before_resume = bool(self.get_parameter('require_cleared_before_resume').value)

        self.telegram_enable = bool(self.get_parameter('telegram_enable').value)
        self.bot_token = self.get_parameter('telegram_bot_token').get_parameter_value().string_value
        self.chat_id = self.get_parameter('telegram_chat_id').get_parameter_value().string_value
        self.tg_prefix = self.get_parameter('telegram_prefix').get_parameter_value().string_value

        # --------- Pub/Sub ---------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.obstacle_pub = self.create_publisher(Bool, '/patrol/obstacle_detected', 10)
        self.event_pub = self.create_publisher(String, '/patrol/event', 10)

        # ✅ /scan QoS: BEST_EFFORT (ydlidar 호환)
        qos_scan = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos_scan)

        # ✅ 수동 재개
        self.resume_sub = self.create_subscription(Bool, self.resume_topic, self.on_resume_cmd, 10)

        # Map
        self._map_lock = threading.Lock()
        self._map: Optional[OccupancyGrid] = None

        # ✅ /map QoS: TRANSIENT_LOCAL (late join에도 맵 받기 쉬움)
        qos_map = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        if self.use_map_filter:
            self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, qos_map)

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=3.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --------- State ---------
        self._stop = False
        self._alert_sent = False
        self._clear_since: Optional[float] = None
        self._ready_to_resume = False

        self._tracks: Dict[int, Track] = {}
        self._next_tid = 1
        self._motion_since: Optional[float] = None

        self._last_warn: Dict[str, float] = {}

        period = 1.0 / max(self.stop_publish_hz, 1.0)
        self.stop_timer = self.create_timer(period, self.on_stop_timer)

        self.get_logger().info("[Lidar Dynamic Detector] ready (map-frame tracking, manual resume mode)")

    # --------- throttle logger ---------
    def _warn_throttle(self, key: str, sec: float, msg: str):
        now = time.time()
        last = self._last_warn.get(key, 0.0)
        if (now - last) >= sec:
            self._last_warn[key] = now
            self.get_logger().warn(msg)

    # ---------------- Manual resume ----------------
    def on_resume_cmd(self, msg: Bool):
        if not msg.data:
            return

        if not self._stop:
            self.get_logger().info("Resume cmd ignored: not stopped.")
            return

        if self.require_cleared_before_resume and (not self._ready_to_resume):
            self.get_logger().warn("Resume cmd ignored: not cleared/confirmed yet.")
            self._publish_event("RESUME_REQUESTED_BUT_NOT_CLEARED")
            return

        self._stop = False
        self._ready_to_resume = False
        self._clear_since = None
        self._alert_sent = False
        self._publish_obstacle(False)
        self._publish_event("RESUMED_BY_ADMIN")
        self.get_logger().info("RESUME: by admin command")

    # ---------------- Map ----------------
    def on_map(self, msg: OccupancyGrid):
        with self._map_lock:
            self._map = msg

    def _get_map(self) -> Optional[OccupancyGrid]:
        with self._map_lock:
            return self._map

    def _world_to_map(self, m: OccupancyGrid, x: float, y: float) -> Optional[Tuple[int, int]]:
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        mx = int((x - ox) / res)
        my = int((y - oy) / res)
        if mx < 0 or my < 0 or mx >= m.info.width or my >= m.info.height:
            return None
        return mx, my

    def _cell_value(self, m: OccupancyGrid, mx: int, my: int) -> int:
        idx = my * m.info.width + mx
        return int(m.data[idx])

    def _is_occupied_with_inflation(self, m: OccupancyGrid, x: float, y: float) -> bool:
        cell = self._world_to_map(m, x, y)
        if cell is None:
            return True  # 맵 밖은 occupied로 취급

        mx, my = cell
        r = max(0, int(self.map_inflation_m / max(m.info.resolution, 1e-6)))

        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                nx, ny = mx + dx, my + dy
                if nx < 0 or ny < 0 or nx >= m.info.width or ny >= m.info.height:
                    continue
                v = self._cell_value(m, nx, ny)
                if v < 0:
                    if self.treat_unknown_as_occupied:
                        return True
                    continue
                if v >= self.occupied_threshold:
                    return True
        return False

    # ---------------- TF helper ----------------
    def _lookup_laser_to_map(self, scan: LaserScan):
        laser_frame = self.laser_frame_param.strip() or scan.header.frame_id
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                laser_frame,
                rclpy.time.Time.from_msg(scan.header.stamp),
                timeout=Duration(seconds=0.1),
            )
            return tf
        except Exception:
            return None

    @staticmethod
    def _yaw_from_quat(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _laser_to_map_xy(self, tf, x_l: float, y_l: float) -> Tuple[float, float]:
        q = tf.transform.rotation
        yaw = self._yaw_from_quat(q)
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        cy, sy = math.cos(yaw), math.sin(yaw)
        x_m = tx + (cy * x_l - sy * y_l)
        y_m = ty + (sy * x_l + cy * y_l)
        return x_m, y_m

    # ---------------- Scan processing ----------------
    def on_scan(self, scan: LaserScan):
        now = time.time()

        # 1) front window indices
        front = math.radians(self.front_angle_deg)
        a_min = scan.angle_min
        inc = scan.angle_increment

        i0 = int(round(((-front) - a_min) / inc))
        i1 = int(round(((+front) - a_min) / inc))
        i0 = max(0, min(i0, len(scan.ranges) - 1))
        i1 = max(0, min(i1, len(scan.ranges) - 1))
        if i0 > i1:
            i0, i1 = i1, i0

        window = scan.ranges[i0:i1 + 1]
        if not window:
            self._step_clear(now)
            return

        # 2) points in laser frame
        pts_laser: List[Tuple[float, float]] = []
        angle = a_min + i0 * inc
        for r in window:
            if math.isfinite(r) and (self.min_range_m < r < min(scan.range_max, self.obstacle_range_m)):
                pts_laser.append((r * math.cos(angle), r * math.sin(angle)))
            angle += inc

        if not pts_laser:
            self._step_clear(now)
            self._tracks_prune(now)
            return

        # 3) map filter 준비
        m = None
        tf_l2m = None
        if self.use_map_filter:
            m = self._get_map()
            if m is None:
                self._warn_throttle("no_map", 2.0, "No /map yet. Skipping dynamic detection.")
                self._step_clear(now)
                return

            tf_l2m = self._lookup_laser_to_map(scan)
            if tf_l2m is None:
                self._warn_throttle("tf_fail", 2.0, "TF laser->map failed. Skipping dynamic detection.")
                self._step_clear(now)
                return

            # 4) map occupied 점 제거
            filtered: List[Tuple[float, float]] = []
            for (x_l, y_l) in pts_laser:
                x_m, y_m = self._laser_to_map_xy(tf_l2m, x_l, y_l)
                if not self._is_occupied_with_inflation(m, x_m, y_m):
                    filtered.append((x_l, y_l))
            pts_laser = filtered

            if len(pts_laser) < self.cluster_min_pts:
                self._step_clear(now)
                self._tracks_prune(now)
                return

        # 5) clustering in laser frame (거리/너비 계산은 laser에서 해도 OK)
        clusters = self._cluster_points(pts_laser)

        # 6) centroid 계산 + ✅ map frame centroid로 변환
        centroids_map: List[Tuple[float, float]] = []

        for c in clusters:
            if len(c) < self.cluster_min_pts:
                continue

            cx_l = sum(p[0] for p in c) / len(c)
            cy_l = sum(p[1] for p in c) / len(c)

            w = self._cluster_width(c)
            if w < self.cluster_min_width or w > self.cluster_max_width:
                continue

            if self.use_map_filter:
                # map frame centroid (안정적인 동적 판정용)
                cx_m, cy_m = self._laser_to_map_xy(tf_l2m, cx_l, cy_l)
                centroids_map.append((cx_m, cy_m))
            else:
                # fallback: map 없이 laser에서 트랙(오탐 가능)
                centroids_map.append((cx_l, cy_l))

        # 7) track update / motion check (✅ centroids_map 기준)
        moving = self._tracks_update_and_check_motion(centroids_map, now)

        if moving:
            self._step_stop(now)
        else:
            self._step_clear(now)

        self._tracks_prune(now)

    def _cluster_points(self, pts: List[Tuple[float, float]]) -> List[List[Tuple[float, float]]]:
        clusters: List[List[Tuple[float, float]]] = []
        cur: List[Tuple[float, float]] = [pts[0]]
        for i in range(1, len(pts)):
            x0, y0 = pts[i - 1]
            x1, y1 = pts[i]
            if math.hypot(x1 - x0, y1 - y0) <= self.cluster_dist_m:
                cur.append((x1, y1))
            else:
                clusters.append(cur)
                cur = [(x1, y1)]
        clusters.append(cur)
        return clusters

    def _cluster_width(self, c: List[Tuple[float, float]]) -> float:
        xs = [p[0] for p in c]
        ys = [p[1] for p in c]
        return math.hypot(max(xs) - min(xs), max(ys) - min(ys))

    def _tracks_update_and_check_motion(self, meas_xy: List[Tuple[float, float]], now: float) -> bool:
        assigned = set()
        for t in self._tracks.values():
            t.missed += 1

        for (mx, my) in meas_xy:
            best_id = None
            best_d = 1e9
            for tid, tr in self._tracks.items():
                d = math.hypot(mx - tr.x, my - tr.y)
                if d < best_d:
                    best_d = d
                    best_id = tid

            if best_id is not None and best_d <= self.match_dist and best_id not in assigned:
                tr = self._tracks[best_id]
                dt = max(now - tr.last_t, 1e-3)
                vx = (mx - tr.x) / dt
                vy = (my - tr.y) / dt
                alpha = 0.5
                tr.vx = alpha * vx + (1 - alpha) * tr.vx
                tr.vy = alpha * vy + (1 - alpha) * tr.vy
                tr.x, tr.y = mx, my
                tr.last_t = now
                tr.seen_t = now
                tr.missed = 0
                assigned.add(best_id)
            else:
                tid = self._next_tid
                self._next_tid += 1
                self._tracks[tid] = Track(
                    tid=tid,
                    x=mx, y=my,
                    vx=0.0, vy=0.0,
                    last_t=now, seen_t=now,
                    missed=0,
                )
                assigned.add(tid)

        # motion 판단
        any_fast = False
        for tr in self._tracks.values():
            if (now - tr.seen_t) > 0.3:
                continue
            speed = math.hypot(tr.vx, tr.vy)
            if speed >= self.motion_speed:
                any_fast = True
                break

        if any_fast:
            if self._motion_since is None:
                self._motion_since = now
            return (now - self._motion_since) >= self.motion_hold_sec
        else:
            self._motion_since = None
            return False

    def _tracks_prune(self, now: float):
        forget = []
        for tid, tr in self._tracks.items():
            if (now - tr.seen_t) > self.track_forget_sec:
                forget.append(tid)
        for tid in forget:
            del self._tracks[tid]

    # ---------------- Stop/Clear ----------------
    def _step_stop(self, now: float):
        self._clear_since = None
        self._ready_to_resume = False

        if self._stop:
            return

        self._stop = True
        self._publish_obstacle(True)
        self._publish_event("DYNAMIC_MOVING_OBJECT_DETECTED")

        if not self._alert_sent:
            self._alert_sent = True
            threading.Thread(
                target=self.send_telegram,
                args=(f"{self.tg_prefix} 야간 순찰 중 움직이는 객체 감지! 로봇 정지.",),
                daemon=True,
            ).start()

        self.get_logger().info("STOP: moving object detected")

    def _step_clear(self, now: float):
        if not self._stop:
            return

        if self._ready_to_resume:
            return

        if self._clear_since is None:
            self._clear_since = now
            return

        if (now - self._clear_since) >= self.clear_confirm_sec:
            self._ready_to_resume = True
            self._publish_event("DYNAMIC_MOVING_OBJECT_CLEARED_WAITING_RESUME")
            self.get_logger().info("CLEARED: waiting for admin resume")

    def on_stop_timer(self):
        if not self._stop:
            return
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.cmd_pub.publish(t)

    def _publish_obstacle(self, detected: bool):
        b = Bool()
        b.data = detected
        self.obstacle_pub.publish(b)

    def _publish_event(self, text: str):
        ev = String()
        ev.data = text
        self.event_pub.publish(ev)

    # ---------------- Telegram ----------------
    def send_telegram(self, text: str):
        if not self.telegram_enable:
            return
        if not self.bot_token or not self.chat_id:
            self.get_logger().warn("telegram_enable=True but token/chat_id empty.")
            return
        try:
            url = f"https://api.telegram.org/bot{self.bot_token}/sendMessage"
            payload = {"chat_id": self.chat_id, "text": text}
            requests.post(url, json=payload, timeout=2.5)
        except Exception as e:
            self.get_logger().warn(f"Telegram exception: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPerceptionLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
