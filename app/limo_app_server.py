import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from limo_msgs.msg import LimoStatus 
import signal

class LimoAppServer(Node):
    def __init__(self):
        super().__init__('limo_app_server')

        # 1. 앱으로 정보 보내기 (배터리, 텔레그램 알림용)
        self.status_pub = self.create_publisher(String, '/to_app/status', 10)
        self.telegram_pub = self.create_publisher(String, '/to_app/telegram', 10)
        
        # [삭제됨] 시작/정지 서비스, 모드 퍼블리셔 등 복잡한 거 다 삭제

        # 2. 데이터 수신 (배터리 확인용)
        self.status_sub = self.create_subscription(LimoStatus, '/limo_status', self.battery_callback, 10)
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.battery_percentage = 0.0
        
        self.get_logger().info("=== LIMO 앱 서버 (기본 모니터링 모드) 가동됨 ===")

    def battery_callback(self, msg):
        vol = msg.battery_voltage
        if vol > 0:
            pct = (vol - 9.6) / (12.6 - 9.6) * 100
            self.battery_percentage = max(0.0, min(100.0, pct))

    def timer_callback(self):
        # 단순 상태 메시지 전송
        full_msg = String()
        full_msg.data = f"로봇 작동 중... 🤖 | 배터리: {int(self.battery_percentage)}%"
        self.status_pub.publish(full_msg)

    # 텔레그램 메시지 중계용 함수 (로그 중계기가 씀)
    def send_telegram_alert(self, text):
        pass # 사용 안 함

def main(args=None):
    rclpy.init(args=args)
    node = LimoAppServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()