# limo_night_patrol/dummy_rgb_publisher_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class DummyRGBPublisherNode(Node):
    def __init__(self):
        super().__init__('dummy_rgb_publisher_node')
        self.pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.bridge = CvBridge()

        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        self.timer = self.create_timer(0.1, self.timer_cb)  # 10Hz

        self.get_logger().info('DummyRGBPublisherNode started (publishing black images)')

    def timer_cb(self):
        w = self.get_parameter('width').value
        h = self.get_parameter('height').value

        # 검은색 이미지 생성
        img = np.zeros((h, w, 3), dtype=np.uint8)

        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'dummy_camera'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DummyRGBPublisherNode()
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
