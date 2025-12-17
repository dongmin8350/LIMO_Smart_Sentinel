# test_parallel.xml 실행 가이드

## 기능 설명

이 Behavior Tree는 다음 기능을 수행합니다:
1. **카메라 이미지 캡처 및 YOLO 검출**
2. **사람 감지 및 등록된 사람 구분**
   - 등록된 사람: 환영 메시지 전송
   - 등록되지 않은 사람: 경고 메시지 + 스크린샷
3. **4개 waypoint 순차 이동**

## 사전 요구사항

### 1. ROS2 환경 설정
```bash
source /opt/ros/humble/setup.bash
```

### 2. 필요한 패키지 설치
```bash
# ChromaDB 설치
pip3 install chromadb

# 기타 의존성 (이미 설치되어 있을 수 있음)
pip3 install -r requirements.txt
```

### 3. Nav2 실행 (필수)
`/navigate_to_pose` 액션 서버가 실행되어 있어야 합니다.

**옵션 A: 실제 Nav2 사용**
```bash
# 별도 터미널에서 Nav2 실행
ros2 launch nav2_bringup bringup_launch.py
```

**옵션 B: 더미 네비게이터 사용 (테스트용)**
```bash
# 별도 터미널에서 더미 네비게이터 실행
ros2 run limo_night_patrol dummy_navigator
```

### 4. 카메라 토픽 확인
카메라가 `/camera/color/image_raw` 토픽으로 이미지를 발행해야 합니다.

## 실행 방법

### 방법 1: 기본 실행 (config.yaml 사용)
```bash
cd ~/team_ws/psh030917/py_bt_ros_1
python3 main.py
```

### 방법 2: run.sh 스크립트 사용
```bash
cd ~/team_ws/psh030917/py_bt_ros_1
./run.sh
```

### 방법 3: 텔레그램 브리지 없이 실행
```bash
python3 main.py --no-telegram
```

## 사람 등록 방법

### 런타임 등록 (ROS 토픽 사용)

1. Behavior Tree 실행 중에 등록할 사람이 카메라 앞에 서있어야 함
2. 별도 터미널에서 등록 명령 실행:

```bash
# 노드 이름 확인 (로그에서 확인 가능, 예: RegisterPerson_1)
ros2 topic pub --once /register_person_RegisterPerson_1 std_msgs/String "data: '홍길동'"
```

### 등록 확인
등록된 사람 수를 확인하려면:
```bash
# Chroma 데이터베이스 위치
ls -la ./chroma_db_registered_*/
```

## 동작 확인

### 1. 로그 확인
실행 시 다음과 같은 로그가 출력됩니다:
```
[INFO] MoveToCoordinate: 초기화 완료 - 기본 좌표: ...
[INFO] IsDetectedSomething: 감지 대상 = "person", use_chroma = True
[INFO] IsRegisteredPerson: 초기화 완료
```

### 2. 토픽 확인
```bash
# 카메라 이미지 확인
ros2 topic echo /camera/color/image_raw --no-arr

# YOLO 검출 결과 확인
ros2 topic echo /yolo/image --no-arr

# 네비게이션 목표 확인
ros2 topic echo /navigate_to_pose/_action/feedback --no-arr
```

### 3. Behavior Tree 시각화
BT Visualizer가 자동으로 실행됩니다 (화면에 표시됨).

## 문제 해결

### 문제 1: "Waiting for navigate_to_pose action server..."
**해결**: Nav2 또는 더미 네비게이터가 실행 중인지 확인

### 문제 2: "ChromaDB가 설치되지 않았습니다"
**해결**: 
```bash
pip3 install chromadb
```

### 문제 3: "No camera image in blackboard"
**해결**: 카메라 토픽이 올바르게 발행되는지 확인
```bash
ros2 topic list | grep camera
ros2 topic echo /camera/color/image_raw --no-arr
```

### 문제 4: CUDA 메모리 오류
**해결**: GPU 메모리 확인 또는 CPU 모드 사용
```bash
# GPU 메모리 확인
nvidia-smi

# 다른 프로세스가 GPU를 사용 중이면 종료
```

## 설정 변경

### config.yaml 수정
```yaml
scenario: scenarios.test
agent:
  behavior_tree_xml: "test_parallel.xml"  # 사용할 XML 파일
bt_runner:
  bt_tick_rate: 30.0  # Behavior Tree 실행 주기 (Hz)
```

### XML 파라미터 조정
- `similarity_threshold`: 유사도 임계값 (0.7 ~ 0.8 권장)
- `cooldown_time`: 사람 감지 쿨다운 시간 (초)
- `confidence_threshold`: YOLO 신뢰도 임계값

## 종료 방법

- `Ctrl+C`로 종료
- 자동으로 텔레그램 브리지도 함께 종료됨

