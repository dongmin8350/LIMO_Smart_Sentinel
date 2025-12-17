# limo_night_patrol/event_handler_night_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class EventHandlerNightNode(Node):
    def __init__(self):
        super().__init__('event_handler_night_node')

        # 현재 모드 (기본값 NIGHT로 시작)
        self.current_mode = 'NIGHT'

        # 모드 구독: /patrol_mode (ModeManagerNode가 퍼블리시)
        self.sub_mode = self.create_subscription(
            String,
            'patrol_mode',
            self.mode_callback,
            10
        )

        # 자동 센싱 노드(DepthObstacleDetector, NightIntruderDetector)가 퍼블리시하는 토픽
        self.sub_depth_obstacle = self.create_subscription(
            Bool,
            '/depth_obstacles',
            self.depth_obstacle_callback,
            10
        )
        self.sub_intruder = self.create_subscription(
            Bool,
            '/night_intruder',
            self.intruder_callback,
            10
        )

        # 패트롤 매니저로 보내는 이벤트
        self.pub_event = self.create_publisher(String, 'patrol_event', 10)

        # 카메라 캡처 서비스 클라이언트
        self.cli_capture = self.create_client(Trigger, 'camera/capture')
        while not self.cli_capture.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for camera/capture service...')

        # 쿨다운(너무 자주 캡처 방지)
        self.last_capture_time = self.get_clock().now()
        self.cooldown_sec = 10.0

        self.get_logger().info('EventHandlerNightNode started')

    # ----------------------
    # 콜백들
    # ----------------------
    def mode_callback(self, msg: String):
        """현재 순찰 모드(DAY / NIGHT) 업데이트"""
        prev = self.current_mode
        self.current_mode = msg.data.strip().upper()
        if self.current_mode != prev:
            self.get_logger().info(f'Patrol mode changed: {prev} -> {self.current_mode}')

    def depth_obstacle_callback(self, msg: Bool):
        # NIGHT 모드가 아니면 무시 (낮에는 별도 event_handler_day가 담당)
        if self.current_mode != 'NIGHT':
            return

        if msg.data:
            self.get_logger().info('Obstacle detected by depth. Trigger capture.')
            self.handle_event('NIGHT_OBSTACLE')

    def intruder_callback(self, msg: Bool):
        if self.current_mode != 'NIGHT':
            return

        if msg.data:
            self.get_logger().info('Night intruder detected. Trigger capture.')
            self.handle_event('NIGHT_INTRUDER')

    # ----------------------
    # 이벤트 공통 처리
    # ----------------------
    def handle_event(self, event_type: str):
        now = self.get_clock().now()
        dt = (now - self.last_capture_time).nanoseconds * 1e-9
        if dt < self.cooldown_sec:
            # 너무 잦은 이벤트는 억제
            self.get_logger().info(f'Skipping capture due to cooldown ({dt:.1f}s)')
            return

        self.last_capture_time = now

        # 1) 사진 촬영 서비스 호출
        req = Trigger.Request()
        future = self.cli_capture.call_async(req)

        def _done(fut):
            try:
                res = fut.result()
                if res.success:
                    self.get_logger().info(f'Capture OK: {res.message}')
                else:
                    self.get_logger().warn(f'Capture failed: {res.message}')
            except Exception as e:
                self.get_logger().error(f'Capture service call failed: {e}')

        future.add_done_callback(_done)

        # 2) 이벤트 토픽 퍼블리시 (PatrolManager가 이걸 보고 일시정지/재개 등 처리)
        msg = String()
        msg.data = event_type
        self.pub_event.publish(msg)
        self.get_logger().info(f'Published patrol_event: {event_type}')


def main(args=None):
    rclpy.init(args=args)
    node = EventHandlerNightNode()
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
