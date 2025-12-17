# 빠른 시작 가이드

## 1단계: 필수 서비스 실행

### 터미널 1: Nav2 또는 더미 네비게이터
```bash
# 옵션 A: 실제 Nav2
ros2 launch nav2_bringup bringup_launch.py

# 옵션 B: 더미 네비게이터 (테스트용)
ros2 run limo_night_patrol dummy_navigator
```

### 터미널 2: 카메라 (필요한 경우)
```bash
# 카메라 노드 실행 (카메라가 자동으로 실행되지 않는 경우)
# 예: ros2 run your_camera_package camera_node
```

## 2단계: Behavior Tree 실행

### 터미널 3: Behavior Tree 실행
```bash
cd ~/team_ws/psh030917/py_bt_ros_1

# 방법 1: 실행 스크립트 사용 (추천)
./scenarios/test/run_test_parallel.sh

# 방법 2: 직접 실행
python3 main.py

# 방법 3: 텔레그램 없이 실행
python3 main.py --no-telegram
```

## 3단계: 사람 등록 (선택사항)

### 터미널 4: 사람 등록
```bash
# 등록할 사람이 카메라 앞에 서있어야 함
# 노드 이름은 실행 로그에서 확인 (예: RegisterPerson_1)

ros2 topic pub --once /register_person_RegisterPerson_1 std_msgs/String "data: '홍길동'"
```

## 동작 확인

### 로그 확인
- `[INFO] MoveToCoordinate: 초기화 완료` - 좌표 이동 노드 준비됨
- `[INFO] IsDetectedSomething: 감지 대상 = "person"` - 사람 감지 준비됨
- `[INFO] IsRegisteredPerson: 초기화 완료` - 등록된 사람 확인 준비됨

### 동작 흐름
1. 카메라 이미지 캡처 및 YOLO 검출
2. 사람 감지 시:
   - 등록된 사람 → "등록된 사용자입니다. 환영합니다!" 메시지
   - 등록되지 않은 사람 → "등록되지 않은 사람이 감지되었습니다!" + 스크린샷
3. 4개 waypoint 순차 이동

## 문제 발생 시

### "Waiting for navigate_to_pose action server..."
→ 터미널 1에서 Nav2 또는 더미 네비게이터 실행 확인

### "ChromaDB가 설치되지 않았습니다"
→ `pip3 install chromadb` 실행

### "No camera image in blackboard"
→ 카메라 토픽 확인: `ros2 topic list | grep camera`

## 종료
`Ctrl+C`로 종료하면 모든 프로세스가 정리됩니다.

