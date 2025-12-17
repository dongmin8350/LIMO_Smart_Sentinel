#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import serial
import json
import time

class FireSensorReader(Node):
    def __init__(self):
        super().__init__("fire_sensor_reader")

        # 아두이노 USB 포트 (ls /dev/ttyACM* 로 확인)
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("시리얼 포트 /dev/ttyACM0 연결 성공")
            # 아두이노 초기화 대기 (연결 시 리셋되므로 버퍼 비우기)
            time.sleep(2.0)
            self.ser.reset_input_buffer()  # 초기 버퍼 비우기
            self.get_logger().info("시리얼 포트 초기화 완료 (2초 대기 후 버퍼 클리어)")
        except Exception as e:
            self.get_logger().error(f"시리얼 포트 연결 실패: {e}")
            raise

        # JSON 전체 데이터 발행
        self.pub = self.create_publisher(String, "fire_sensor", 10)
        # 온도 값만 별도로 발행
        self.temp_pub = self.create_publisher(Float32, "temperature", 10)
        self.timer = self.create_timer(0.1, self.read_sensor)  # 0.1초마다 읽기 (10Hz) - 더 빠른 폴링
        
        # 디버깅용 카운터
        self.read_count = 0
        self.publish_count = 0
        self.last_log_time = 0

    def read_sensor(self):
        try:
            # 시리얼 포트가 열려있는지 확인
            if not self.ser.is_open:
                self.get_logger().error("시리얼 포트가 닫혀있습니다")
                return
            
            # 데이터가 있는지 확인
            if self.ser.in_waiting > 0:
                try:
                    # readline() 대신 available한 모든 데이터 읽기 시도
                    raw_data = self.ser.read(self.ser.in_waiting)
                    line = raw_data.decode('utf-8', errors='ignore').strip()
                    self.read_count += 1
                    
                    # 빈 라인 무시
                    if not line:
                        return

                    # 모든 읽은 데이터 로그 출력 (디버깅용)
                    self.get_logger().info(f"[RAW] 읽은 데이터 ({self.read_count}): {line} (원본 bytes: {raw_data})")

                    # JSON 형식인지 체크
                    if not line.startswith("{"):
                        self.get_logger().warn(f"JSON 형식이 아님: {line}")
                        # JSON이 아니어도 원본 데이터는 발행
                        msg = String()
                        msg.data = line
                        self.pub.publish(msg)
                        self.publish_count += 1
                        return

                    # JSON 파싱해서 온도 값 추출
                    try:
                        data = json.loads(line)
                        temp = data.get("temp", 0)
                        flame = data.get("flame", 0)
                        
                        # 실시간 온도 값 로그 출력
                        self.get_logger().info(f"🌡️ 온도: {temp}°C | 🔥 화염: {'감지됨' if flame == 1 else '없음'}")
                        
                        # 온도 값만 별도 토픽으로 발행
                        temp_msg = Float32()
                        temp_msg.data = float(temp)
                        self.temp_pub.publish(temp_msg)
                        
                    except json.JSONDecodeError as e:
                        self.get_logger().warn(f"JSON 파싱 실패: {line} (오류: {e})")
                        # 파싱 실패해도 원본 데이터는 발행

                    # 원본 JSON 데이터 발행 (파싱 성공/실패 관계없이)
                    msg = String()
                    msg.data = line
                    self.pub.publish(msg)
                    self.publish_count += 1
                    self.get_logger().info(f"[PUBLISH] 토픽 발행 완료 ({self.publish_count}번째)")
                    
                except serial.SerialTimeoutException:
                    # 타임아웃은 정상적인 상황일 수 있음 (아두이노가 데이터를 보내지 않을 때)
                    pass
                except UnicodeDecodeError as e:
                    self.get_logger().warn(f"인코딩 오류: {e}")
                except Exception as e:
                    # 기타 에러는 한 번만 로그 출력 (너무 많이 출력되지 않도록)
                    if not hasattr(self, '_last_error') or self._last_error != str(e):
                        self.get_logger().warn(f"데이터 읽기 오류: {e}")
                        self._last_error = str(e)
            else:
                # 데이터가 없을 때 주기적으로 상태 로그 출력 (5초마다)
                current_time = time.time()
                if current_time - self.last_log_time > 5.0:
                    # 시리얼 포트 상태도 함께 출력
                    port_status = "열림" if self.ser.is_open else "닫힘"
                    self.get_logger().warn(f"[대기 중] 시리얼 포트에서 데이터 대기... (읽음: {self.read_count}, 발행: {self.publish_count}, 포트 상태: {port_status}, in_waiting: {self.ser.in_waiting})")
                    self.last_log_time = current_time
                        
        except serial.SerialException as e:
            self.get_logger().error(f"시리얼 포트 오류: {e}")
        except Exception as e:
            self.get_logger().error(f"예상치 못한 오류: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FireSensorReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
