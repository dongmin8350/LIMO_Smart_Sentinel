# limo_night_patrol/patrol_manager_node.py

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class PatrolManagerNode(Node):
    def __init__(self):
        super().__init__('patrol_manager_node')

        # 현재 모드 / 이벤트 상태
        self.current_mode = 'NIGHT'
        self.last_event_type = None
        self.event_pause_sec = 5.0   # 이벤트 후 몇 초 동안 순찰 일시정지할지
        self.event_active_until = 0.0

        # 모드 / 이벤트 구독
        self.sub_mode = self.create_subscription(
            String,
            'patrol_mode',
            self.mode_callback,
            10
        )
        self.sub_event = self.create_subscription(
            String,
            'patrol_event',
            self.event_callback,
            10
        )

        # Nav2 액션 클라이언트
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.declare_parameter('frame_id', 'map')
        self.frame_id = self.get_parameter('frame_id').value

        # 간단한 waypoint 리스트 (예시)
        self.waypoints = self.create_default_waypoints()
        self.current_index = 0
        self.current_goal_handle = None
        self.is_sending_goal = False

        self.timer = self.create_timer(2.0, self.timer_callback)

        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('PatrolManagerNode started')

    # ----------------------
    # Waypoint 설정
    # ----------------------
    def create_default_waypoints(self):
        # map 좌표계에서 (x, y, yaw) 리스트 – 실제 환경에 맞게 수정
        coords = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (1.0, 1.0, math.pi / 2),
            (0.0, 1.0, math.pi),
        ]
        waypoints = []
        for x, y, yaw in coords:
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            # yaw -> quaternion
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            waypoints.append(ps)
        return waypoints

    # ----------------------
    # 콜백: 모드 / 이벤트
    # ----------------------
    def mode_callback(self, msg: String):
        prev = self.current_mode
        self.current_mode = msg.data.strip().upper()
        if self.current_mode != prev:
            self.get_logger().info(f'Patrol mode: {prev} -> {self.current_mode}')

    def event_callback(self, msg: String):
        """밤 모드 이벤트 수신 시, 잠시 순찰 일시정지"""
        self.last_event_type = msg.data
        now = self.get_clock().now().nanoseconds * 1e-9
        self.event_active_until = now + self.event_pause_sec

        self.get_logger().info(f'Patrol event received: {msg.data}, pause patrol for {self.event_pause_sec} s')

        # 현재 goal이 있으면 취소 (로봇 잠시 멈추도록)
        if self.current_goal_handle is not None:
            self.get_logger().info('Canceling current navigation goal due to event')
            cancel_future = self.current_goal_handle.cancel_goal_async()

            # 결과만 로그용으로 받음
            def _cancel_done(fut):
                try:
                    cancel_result = fut.result()
                    self.get_logger().info(f'Cancel result: {cancel_result}')
                except Exception as e:
                    self.get_logger().warn(f'Cancel goal failed: {e}')

            cancel_future.add_done_callback(_cancel_done)

        # goal 상태 플래그 리셋
        self.is_sending_goal = False
        self.current_goal_handle = None

    # ----------------------
    # 타이머: 순찰 메인 루프
    # ----------------------
    def timer_callback(self):
        # 1) 모드가 NIGHT가 아니면 순찰 안 함
        if self.current_mode != 'NIGHT':
            return

        # 2) 이벤트 쿨다운 시간 동안은 순찰 일시정지
        now = self.get_clock().now().nanoseconds * 1e-9
        if now < self.event_active_until:
            # self.get_logger().info('Patrol paused due to recent event')
            return

        if self.is_sending_goal:
            # 이미 목표를 향해 가는 중
            return

        if not self.waypoints:
            return

        # 다음 waypoint로 이동
        goal_pose = self.waypoints[self.current_index]
        self.send_goal(goal_pose)

        # 다음 인덱스로
        self.current_index = (self.current_index + 1) % len(self.waypoints)

    # ----------------------
    # 액션 클라이언트 관련
    # ----------------------
    def send_goal(self, pose: PoseStamped):
        goal_msg = NavigateToPose.Goal()
        pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose = pose

        self.get_logger().info(
            f'Sending goal to ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )

        self.is_sending_goal = True
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.is_sending_goal = False
            self.current_goal_handle = None
            return

        self.get_logger().info('Goal accepted')
        self.current_goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'Goal result received. status={status}, result={result}')
        self.is_sending_goal = False

    def feedback_callback(self, feedback_msg):
        # 필요하면 현재 위치/거리 등에 대한 로그 추가 가능
        # feedback = feedback_msg.feedback
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PatrolManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
