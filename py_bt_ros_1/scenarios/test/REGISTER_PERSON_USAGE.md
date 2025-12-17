# 사람 등록 전용 XML 사용 가이드

## 개요

`register_person.xml`은 사람을 등록하기 위한 전용 Behavior Tree입니다.
메인 Behavior Tree와 분리되어 있어 등록 작업만 간단하게 수행할 수 있습니다.

## 사용 방법

### 방법 1: config 파일 사용 (추천)

```bash
cd ~/team_ws/psh030917/py_bt_ros_1
python3 main.py --config scenarios/test/config_register.yaml
```

### 방법 2: 직접 config.yaml 수정

`config.yaml` 파일을 수정:
```yaml
agent:
  behavior_tree_xml: "register_person.xml"
```

그리고 실행:
```bash
python3 main.py
```

## 등록 절차

### 1단계: 등록 모드 실행
```bash
cd ~/team_ws/psh030917/py_bt_ros_1
python3 main.py --config scenarios/test/config_register.yaml
```

### 2단계: 등록할 사람이 카메라 앞에 서있기
- 카메라가 사람을 명확히 볼 수 있어야 함
- YOLO가 사람을 감지할 수 있는 위치

### 3단계: 등록 명령 실행 (별도 터미널)

#### 옵션 A: ROS 토픽으로 등록 (이름 지정)
```bash
# 노드 이름 확인 (로그에서 확인, 예: RegisterPerson_1)
ros2 topic pub --once /register_person_RegisterPerson_1 std_msgs/String "data: '홍길동'"
```

#### 옵션 B: blackboard에 이름 설정 (코드에서)
다른 노드에서 blackboard에 설정:
```python
blackboard['person_name'] = "홍길동"
```

### 4단계: 등록 확인
로그에서 확인:
```
[INFO] 등록 요청 수신: 홍길동
[INFO] 새 사람 등록 완료: 홍길동 (ID: 홍길동_1234567890)
[INFO] 현재 등록된 사람 수: 1
[INFO] SendTelegramMessage: 텔레그램 메시지 전송 - "사람이 등록되었습니다!"
```

## 동작 방식

1. **카메라 이미지 캡처**: `/camera/color/image_raw`에서 이미지 수신
2. **YOLO 검출**: 사람 감지
3. **중복 확인**: 이미 등록된 사람인지 확인
   - 등록된 사람이면: 등록하지 않고 통과
   - 등록되지 않은 사람이면: 등록 진행
4. **등록 실행**: Chroma 데이터베이스에 저장
5. **알림**: 텔레그램 메시지 전송 및 스크린샷 저장

## 특징

- **중복 방지**: 이미 등록된 사람은 다시 등록하지 않음
- **자동 감지**: 사람이 감지되면 자동으로 등록 프로세스 시작
- **ROS 토픽 지원**: 런타임에 이름을 지정하여 등록 가능
- **간단한 사용**: 메인 Behavior Tree와 분리되어 등록 작업만 수행

## 예시 시나리오

### 여러 사람 등록하기

```bash
# 터미널 1: 등록 모드 실행
cd ~/team_ws/psh030917/py_bt_ros_1
python3 main.py --config scenarios/test/config_register.yaml

# 터미널 2: 사람 등록
# 1. 홍길동 등록 (카메라 앞에 서있어야 함)
ros2 topic pub --once /register_person_RegisterPerson_1 std_msgs/String "data: '홍길동'"

# 2. 김철수 등록
ros2 topic pub --once /register_person_RegisterPerson_1 std_msgs/String "data: '김철수'"

# 3. 이영희 등록
ros2 topic pub --once /register_person_RegisterPerson_1 std_msgs/String "data: '이영희'"
```

## 등록된 사람 확인

등록된 사람 수를 확인하려면:
```bash
# Chroma 데이터베이스 위치
ls -la ~/team_ws/psh030917/py_bt_ros_1/scenarios/test/chroma_db_registered_*/
```

또는 실행 로그에서:
```
[INFO] 등록된 사람 컬렉션 초기화 완료: "registered_person_RegisterPerson_1" (등록된 사람 수: 3)
```

## 메인 Behavior Tree로 전환

등록이 완료되면 메인 Behavior Tree로 전환:

```bash
# config.yaml을 원래대로 되돌리거나
python3 main.py  # 기본 config.yaml 사용
```

또는:
```bash
# config.yaml 수정
agent:
  behavior_tree_xml: "test_parallel.xml"  # 원래 XML로 변경
```

## 팁

1. **여러 각도에서 등록**: 같은 사람을 여러 각도에서 등록하면 인식 정확도가 향상됩니다.
2. **명확한 조명**: 등록 시 충분한 조명이 있는 곳에서 진행하세요.
3. **명확한 이름 사용**: 등록 시 명확한 이름을 사용하세요 (예: "홍길동", "김철수")
4. **등록 후 테스트**: 등록 후 메인 Behavior Tree에서 등록된 사람이 제대로 인식되는지 확인하세요.

