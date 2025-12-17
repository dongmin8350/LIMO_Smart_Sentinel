# 사람 등록 방법 가이드

## 방법 1: ROS 토픽을 통한 등록 (가장 추천 ⭐)

### 단계별 가이드

#### 1단계: Behavior Tree 실행
```bash
cd ~/team_ws/psh030917/py_bt_ros_1
python3 main.py
```

#### 2단계: 등록할 사람이 카메라 앞에 서있기
- 카메라가 사람을 볼 수 있어야 함
- YOLO가 사람을 감지할 수 있는 위치에 있어야 함

#### 3단계: 노드 이름 확인
실행 로그에서 `RegisterPerson` 노드 이름을 확인:
```
[INFO] RegisterPerson: 초기화 완료 - target_class="person", 토픽: /register_person_RegisterPerson_1
```
→ 노드 이름이 `RegisterPerson_1`인 경우

#### 4단계: 등록 명령 실행 (별도 터미널)
```bash
# 노드 이름이 RegisterPerson_1인 경우
ros2 topic pub --once /register_person_RegisterPerson_1 std_msgs/String "data: '홍길동'"

# 다른 이름인 경우 (예: RegisterPerson_2)
ros2 topic pub --once /register_person_RegisterPerson_2 std_msgs/String "data: '홍길동'"
```

#### 5단계: 등록 확인
로그에서 다음 메시지 확인:
```
[INFO] 새 사람 등록 완료: 홍길동 (ID: 홍길동_1234567890)
[INFO] 현재 등록된 사람 수: 1
```

### 예시 시나리오

```bash
# 터미널 1: Behavior Tree 실행
cd ~/team_ws/psh030917/py_bt_ros_1
python3 main.py

# 터미널 2: 사람 등록
# 1. 홍길동 등록
ros2 topic pub --once /register_person_RegisterPerson_1 std_msgs/String "data: '홍길동'"

# 2. 김철수 등록
ros2 topic pub --once /register_person_RegisterPerson_1 std_msgs/String "data: '김철수'"

# 3. 이영희 등록
ros2 topic pub --once /register_person_RegisterPerson_1 std_msgs/String "data: '이영희'"
```

## 방법 2: XML에서 자동 등록

XML 파일에서 등록되지 않은 사람을 자동으로 등록하도록 설정:

```xml
<Sequence>
  <IsDetectedSomething target_class="person" use_chroma="true"/>
  
  <Fallback>
    <!-- 이미 등록된 사람이면 통과 -->
    <IsRegisteredPerson target_class="person"/>
    
    <!-- 등록되지 않은 사람이면 자동 등록 -->
    <Sequence>
      <RegisterPerson target_class="person" person_name="Unknown_Person"/>
      <SendTelegramMessage message="새로운 사람이 자동 등록되었습니다."/>
    </Sequence>
  </Fallback>
</Sequence>
```

**주의**: 이 방법은 모든 감지된 사람을 "Unknown_Person"으로 등록합니다.

## 방법 3: blackboard를 통한 동적 등록

다른 노드에서 blackboard에 `person_name`을 설정:

```python
# 다른 노드에서 실행
blackboard['person_name'] = "홍길동"
```

그리고 XML에서:
```xml
<RegisterPerson target_class="person"/>  <!-- person_name 파라미터 없음 -->
```

## 등록 확인 방법

### 1. 로그 확인
등록 성공 시 로그에 표시:
```
[INFO] 새 사람 등록 완료: 홍길동
[INFO] 현재 등록된 사람 수: 1
```

### 2. Chroma 데이터베이스 확인
```bash
cd ~/team_ws/psh030917/py_bt_ros_1/scenarios/test
ls -la chroma_db_registered_*/
```

### 3. 등록된 사람 수 확인
Behavior Tree 실행 로그에서:
```
[INFO] 등록된 사람 컬렉션 초기화 완료: "registered_person_RegisterPerson_1" (등록된 사람 수: 3)
```

## 등록된 사람 테스트

등록 후, 같은 사람이 다시 감지되면:
- `IsRegisteredPerson` 노드가 SUCCESS 반환
- "등록된 사용자입니다. 환영합니다!" 메시지 전송

## 문제 해결

### 문제 1: "등록할 사람 정보가 없습니다"
**원인**: 사람이 감지되지 않았거나, `IsDetectedSomething`이 실행되지 않음
**해결**: 
- 카메라 앞에 사람이 있는지 확인
- `IsDetectedSomething` 노드가 먼저 실행되어야 함

### 문제 2: "등록된 사람 컬렉션이 초기화되지 않았습니다"
**원인**: ChromaDB 초기화 실패
**해결**: 
- ChromaDB 설치 확인: `pip3 install chromadb`
- Behavior Tree 재시작

### 문제 3: 노드 이름을 모르겠습니다
**해결**: 
- 실행 로그에서 `RegisterPerson: 초기화 완료` 메시지 확인
- 또는 `ros2 topic list | grep register_person` 명령으로 확인

## 팁

1. **여러 각도에서 등록**: 같은 사람을 여러 각도에서 등록하면 인식 정확도가 향상됩니다.
2. **명확한 이름 사용**: 등록 시 명확한 이름을 사용하세요 (예: "홍길동", "김철수")
3. **유사도 임계값 조정**: 필요시 `similarity_threshold`를 조정하세요 (0.7 ~ 0.8 권장)

## 빠른 참조

```bash
# 1. Behavior Tree 실행
cd ~/team_ws/psh030917/py_bt_ros_1 && python3 main.py

# 2. 별도 터미널에서 등록 (노드 이름 확인 필요)
ros2 topic pub --once /register_person_RegisterPerson_1 std_msgs/String "data: '홍길동'"

# 3. 등록 확인
# 로그에서 "새 사람 등록 완료" 메시지 확인
```

