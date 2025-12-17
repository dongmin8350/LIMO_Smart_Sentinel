# limo_night_patrol/dummy_navigator_node.py

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose


class DummyNavigatorNode(Node):
    def __init__(self):
        super().__init__('dummy_navigator_node')

        # /cmd_vel/nav2 로 속도 명령 퍼블리시
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel/nav2', 10)

        # navigate_to_pose 액션 서버 (Nav2 흉내)
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )

        self.get_logger().info('DummyNavigatorNode started (no real Nav2)')

    def execute_callback(self, goal_handle):
        """단순히 일정 시간 동안 앞으로 전진하는 더미 네비게이션"""
        goal = goal_handle.request
        x = goal.pose.pose.position.x
        y = goal.pose.pose.position.y

        self.get_logger().info(f'Received goal: ({x:.2f}, {y:.2f})')

        # 1) 간단하게 "앞으로 조금 가는" 동작 시뮬레이션
        period = 0.5          # 초
        total_time = 3.0      # 몇 초 동안 움직이는 척 할지
        steps = int(total_time / period)

        twist = Twist()
        twist.linear.x = 0.1   # 전진 속도
        twist.angular.z = 0.0

        for _ in range(steps):
            # goal 이 중간에 cancel 되었는지 체크
            if not goal_handle.is_active:
                self.get_logger().warn('Goal canceled while executing')
                break

            self.cmd_pub.publish(twist)
            time.sleep(period)

        # 2) 정지 명령
        stop_twist = Twist()
        self.cmd_pub.publish(stop_twist)

        # 3) goal 성공 처리
        if goal_handle.is_active:
            goal_handle.succeed()
            self.get_logger().info('Dummy navigation finished (SUCCEEDED)')
        else:
            self.get_logger().info('Dummy navigation ended (CANCELED/INACTIVE)')

        result = NavigateToPose.Result()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DummyNavigatorNode()
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
