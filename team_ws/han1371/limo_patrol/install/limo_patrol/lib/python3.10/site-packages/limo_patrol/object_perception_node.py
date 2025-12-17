#!/usr/bin/env python3
import time
import math
from collections import deque

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge

import numpy as np


class ObjectPerceptionNode(Node):
    """
    - illumination 파라미터 기반 day/night 모드 결정
    - /patrol/mode 를 1초마다 주기적으로 publish (디버깅용)
    """

    def __init__(self):
        super().__init__('object_perception_node')

        # ---------------- parameters ----------------
        self.declare_parameter('illumination', 50.0)               # DOUBLE
        self.declare_parameter('illumination_threshold', 100.0)   # DOUBLE

        self.declare_parameter('yolo_topic', '/yolov5/detections')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('max_detection_distance', 2.0)

        # ---------------- read parameters ----------------
        self.illumination = float(self.get_parameter('illumination').value)
        self.illumination_threshold = float(self.get_parameter('illumination_threshold').value)

        self.yolo_topic = self.get_parameter('yolo_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.max_detection_distance = float(self.get_parameter('max_detection_distance').value)

        # ---------------- publishers ----------------
        self.mode_pub = self.create_publisher(String, '/patrol/mode', 10)
        self.obstacle_pub = self.create_publisher(Bool, '/patrol/obstacle_detected', 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # ---------------- subscribers ----------------
        self.yolo_sub = self.create_subscription(
            Detection2DArray,
            self.yolo_topic,
            self.yolo_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )

        # ---------------- utils ----------------
        self.bridge = CvBridge()

        # ---------------- mode init ----------------
        self.mode = self.decide_mode()

        # 🔥 핵심: 1초마다 mode publish
        self.mode_timer = self.create_timer(1.0, self.publish_mode)

        self.get_logger().info(
            f'ObjectPerceptionNode started | '
            f'illumination={self.illumination}, '
            f'threshold={self.illumination_threshold}, '
            f'mode={self.mode}'
        )

    # =================================================
    # Mode logic
    # =================================================
    def decide_mode(self) -> str:
        """illumination 파라미터 기준으로 주/야간 결정"""
        self.illumination = float(self.get_parameter('illumination').value)
        self.illumination_threshold = float(self.get_parameter('illumination_threshold').value)

        # ✔ 일반적인 정의
        # 밝으면 day, 어두우면 night
        if self.illumination > self.illumination_threshold:
            return 'day'
        else:
            return 'night'

    def publish_mode(self):
        """주기적으로 모드 publish"""
        new_mode = self.decide_mode()

        if new_mode != self.mode:
            self.get_logger().warn(f'[MODE CHANGE] {self.mode} -> {new_mode}')
            self.mode = new_mode

        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)

    # =================================================
    # DAY: YOLO
    # =================================================
    def yolo_callback(self, msg: Detection2DArray):
        if self.mode != 'day':
            return

        detected = len(msg.detections) > 0
        self.obstacle_pub.publish(Bool(data=detected))

        if detected:
            self.get_logger().warn('[DAY] YOLO detected object')

    # =================================================
    # NIGHT: Depth
    # =================================================
    def depth_callback(self, msg: Image):
        if self.mode != 'night':
            return

        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth convert failed: {e}')
            return

        depth = depth_img.astype(np.float32)
        valid = depth[np.isfinite(depth)]
        valid = valid[valid > 0.0]

        if valid.size == 0:
            self.obstacle_pub.publish(Bool(data=False))
            return

        min_dist = float(np.min(valid))
        obstacle = min_dist <= self.max_detection_distance
        self.obstacle_pub.publish(Bool(data=obstacle))

        if obstacle:
            self.get_logger().warn(f'[NIGHT] Obstacle detected | min_dist={min_dist:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
