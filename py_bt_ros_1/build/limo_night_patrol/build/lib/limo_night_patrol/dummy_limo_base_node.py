# limo_night_patrol/dummy_limo_base_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DummyLimoBaseNode(Node):
    def __init__(self):
        super().__init__('dummy_limo_base_node')

        self.sub_cmd = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_cb,
            10
        )

        self.get_logger().info('DummyLimoBaseNode started (logging cmd_vel only)')

    def cmd_cb(self, msg: Twist):
        self.get_logger().info(
            f'Received cmd_vel: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DummyLimoBaseNode()
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
