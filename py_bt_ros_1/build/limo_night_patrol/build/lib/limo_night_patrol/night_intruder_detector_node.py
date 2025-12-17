# limo_night_patrol/night_intruder_detector_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class NightIntruderDetector(Node):
    def __init__(self):
        super().__init__('night_intruder_detector_node')

        self.declare_parameter('hold_time_sec', 2.0)            # 몇 초 이상 가까이 있으면 intruder
        self.declare_parameter('obstacle_topic', '/depth_obstacles')

        self.hold_time = self.get_parameter('hold_time_sec').get_parameter_value().double_value
        obstacle_topic = self.get_parameter('obstacle_topic').get_parameter_value().string_value

        self.sub = self.create_subscription(Bool, obstacle_topic, self.obstacle_cb, 10)
        self.pub = self.create_publisher(Bool, '/night_intruder', 10)

        self.last_obstacle_start = None
        self.intruder_active = False

        self.get_logger().info(
            f"NightIntruderDetectorNode started (hold_time={self.hold_time}s, obstacle_topic={obstacle_topic})"
        )

    def obstacle_cb(self, msg: Bool):
        now = self.get_clock().now().nanoseconds * 1e-9  # seconds
        if msg.data:
            # 장애물이 보이기 시작한 시간 기록
            if self.last_obstacle_start is None:
                self.last_obstacle_start = now

            duration = now - self.last_obstacle_start
            if not self.intruder_active and duration >= self.hold_time:
                # 가까운 물체가 일정 시간 이상 머무르면 intruder로 판단
                self.intruder_active = True
                self.get_logger().info(f"Intruder detected (duration={duration:.2f}s)")
                self.pub.publish(Bool(data=True))
        else:
            # 장애물이 사라지면 상태 리셋
            self.last_obstacle_start = None
            if self.intruder_active:
                self.intruder_active = False
                self.get_logger().info("Intruder cleared")
                self.pub.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = NightIntruderDetector()
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
