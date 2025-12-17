#!/usr/bin/env python3
"""
시리얼 포트 테스트 스크립트
아두이노가 실제로 데이터를 보내는지 확인
"""
import serial
import time

port = '/dev/ttyACM0'
baudrate = 115200

print(f"시리얼 포트 {port} 연결 시도 (보드레이트: {baudrate})...")
try:
    ser = serial.Serial(port, baudrate, timeout=2)
    print(f"✓ 연결 성공!")
    print(f"포트 설정: {ser}")
    print(f"초기화 대기 중... (3초)")
    time.sleep(3)
    ser.reset_input_buffer()
    
    print("\n=== 데이터 읽기 시작 (10초간) ===")
    start_time = time.time()
    read_count = 0
    last_status_time = start_time
    
    while time.time() - start_time < 10:
        elapsed = int(time.time() - start_time)
        if ser.in_waiting > 0:
            try:
                # readline() 시도
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:  # 빈 라인이 아닌 경우만
                    read_count += 1
                    print(f"[{read_count}] ({elapsed}초) {line} (길이: {len(line)} bytes)")
            except Exception as e:
                print(f"읽기 오류: {e}")
        else:
            # 2초마다 상태 출력
            if time.time() - last_status_time >= 2.0:
                print(f"[{elapsed}초] 대기 중... (in_waiting: {ser.in_waiting} bytes)")
                last_status_time = time.time()
            time.sleep(0.1)
    
    print(f"\n=== 최종 결과 ===")
    print(f"총 읽은 라인 수: {read_count}")
    print(f"포트 상태: {'열림' if ser.is_open else '닫힘'}")
    print(f"마지막 in_waiting: {ser.in_waiting} bytes")
    
    if read_count == 0:
        print("\n⚠️  데이터가 전혀 오지 않았습니다!")
        print("\n🔍 추가 진단:")
        # 다른 보드레이트로도 시도해볼 수 있는지 안내
        print("다른 보드레이트로 시도해보세요:")
        print("  - 9600: python3 -c \"import serial; s=serial.Serial('/dev/ttyACM0', 9600, timeout=1); import time; time.sleep(2); print('읽은 데이터:', s.read(s.in_waiting).decode('utf-8', errors='ignore') if s.in_waiting > 0 else '없음'); s.close()\"")
        print("  - 57600: python3 -c \"import serial; s=serial.Serial('/dev/ttyACM0', 57600, timeout=1); import time; time.sleep(2); print('읽은 데이터:', s.read(s.in_waiting).decode('utf-8', errors='ignore') if s.in_waiting > 0 else '없음'); s.close()\"")
        print("\n확인 사항:")
        print("1. 아두이노가 전원이 켜져 있는지 확인 (LED 확인)")
        print("2. 아두이노 코드가 Serial.begin(115200)로 설정되어 있는지 확인")
        print("3. 아두이노 코드가 실제로 Serial.print() 또는 Serial.println()을 호출하는지 확인")
        print("4. 아두이노 IDE의 시리얼 모니터로 데이터가 보이는지 확인")
        print("5. USB 케이블이 제대로 연결되어 있는지 확인")
    else:
        print("✓ 데이터가 정상적으로 수신되고 있습니다!")
    
    ser.close()
    
except serial.SerialException as e:
    print(f"✗ 시리얼 포트 연결 실패: {e}")
    print("\n확인 사항:")
    print("1. 다른 프로세스가 포트를 사용 중인지 확인: lsof /dev/ttyACM0")
    print("2. 포트 권한 확인: ls -l /dev/ttyACM0")
    print("3. 사용자를 dialout 그룹에 추가: sudo usermod -a -G dialout $USER")
except Exception as e:
    print(f"✗ 오류 발생: {e}")

