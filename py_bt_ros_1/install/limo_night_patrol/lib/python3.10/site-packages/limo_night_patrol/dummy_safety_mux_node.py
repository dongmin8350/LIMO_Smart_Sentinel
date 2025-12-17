# limo_night_patrol/dummy_safety_mux_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DummySafetyMuxNode(Node):
    def __init__(self):
        super().__init__('dummy_safety_mux_node')

        self.sub_nav = self.create_subscription(
            Twist,
            'cmd_vel/nav2',
            self.nav_cb,
            10
        )

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info('DummySafetyMuxNode started (just forwarding cmd_vel/nav2)')

    def nav_cb(self, msg: Twist):
        # 실제 safety mux라면 estop 등을 고려해야 하지만,
        # 여기선 단순히 포워딩만 한다.
        self.pub_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummySafetyMuxNode()
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
