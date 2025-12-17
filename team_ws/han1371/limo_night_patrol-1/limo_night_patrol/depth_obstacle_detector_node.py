# limo_night_patrol/depth_obstacle_detector_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import numpy as np
from cv_bridge import CvBridge


class DepthObstacleDetector(Node):
    def __init__(self):
        super().__init__('depth_obstacle_detector_node')
        self.bridge = CvBridge()

        # 파라미터
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('obstacle_threshold_m', 0.8)  # m, 이 거리 이내면 장애물
        self.declare_parameter('roi_ratio', 0.3)             # 중앙 ROI 비율
        self.declare_parameter('min_pixels', 0)              # 장애물 판단에 필요한 최소 유효 픽셀 수

        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.obstacle_distance = self.get_parameter('obstacle_threshold_m').get_parameter_value().double_value
        self.roi_ratio = self.get_parameter('roi_ratio').get_parameter_value().double_value
        self.min_pixels = self.get_parameter('min_pixels').get_parameter_value().integer_value

        self.sub = self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.pub = self.create_publisher(Bool, '/depth_obstacles', 10)

        self.get_logger().info(
            f"DepthObstacleDetectorNode started, topic={depth_topic}, "
            f"threshold={self.obstacle_distance} m, "
            f"roi_ratio={self.roi_ratio}, min_pixels={self.min_pixels}"
        )

    def depth_callback(self, msg: Image):
        try:
            # 32FC1, depth in meters 가정
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().warn(f"Failed to convert depth image: {e}")
            return

        h, w = depth.shape
        roi_w = int(w * self.roi_ratio)
        roi_h = int(h * self.roi_ratio)
        x1 = w // 2 - roi_w // 2
        y1 = h // 2 - roi_h // 2
        x2 = x1 + roi_w
        y2 = y1 + roi_h

        roi = depth[y1:y2, x1:x2]

        # 유효한 depth 값만 추출 (0 < d < 10m 같은)
        valid = roi[(roi > 0.1) & (roi < 10.0)]
        if valid.size == 0 or valid.size < self.min_pixels:
            # 관측 없음 또는 유효 픽셀 부족 → 장애물 없음으로 처리
            self.pub.publish(Bool(data=False))
            return

        min_dist = float(np.min(valid))

        obstacle = min_dist < self.obstacle_distance
        self.pub.publish(Bool(data=obstacle))
        # 디버그가 필요하면 아래 로그 활성화
        # self.get_logger().info(f"min_dist={min_dist:.2f} m, obstacle={obstacle}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleDetector()
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
