import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import Log  # ROS 로그 메시지 타입

class LogRelay(Node):
    def __init__(self):
        super().__init__('log_relay')

        # 1. 앱으로 알림을 쏘는 퍼블리셔
        self.app_pub = self.create_publisher(String, '/to_app/telegram', 10)

        # 2. 터미널 로그를 엿듣는 구독자 (도청장치)
        # '/rosout'에는 터미널에 출력되는 모든 로그가 모입니다.
        self.sub = self.create_subscription(Log, '/rosout', self.log_callback, 10)
        
        self.get_logger().info("=== 로그 중계기 가동: '감지' 메시지 대기 중 ===")

    def log_callback(self, msg):
        # msg.msg 안에 실제 로그 텍스트가 들어있습니다.
        log_text = msg.msg
        
        # [핵심] 아래 단어들이 포함된 로그만 앱으로 보냅니다.
        # 사용자님의 스크린샷에 나온 "사람 감지", "화재" 등을 넣었습니다.
        keywords = ["사람 감지", "person detected", "화재", "Fire", "등록되지 않은"]
        
        for word in keywords:
            if word in log_text:
                # 앱으로 전송!
                send_msg = String()
                # 보기 좋게 앞에 이모지 추가
                send_msg.data = f"🚨 알림: {log_text}"
                self.app_pub.publish(send_msg)
                
                # 로그창에도 표시
                # self.get_logger().info(f"앱으로 전송됨: {send_msg.data}")
                break

def main(args=None):
    rclpy.init(args=args)
    node = LogRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()