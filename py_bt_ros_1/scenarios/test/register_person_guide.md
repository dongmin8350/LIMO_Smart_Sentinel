# 사람 등록 방법 가이드

## 방법 1: XML에서 직접 등록 (가장 간단)

Behavior Tree XML에서 `RegisterPerson` 노드를 사용하여 감지된 사람을 등록합니다.

```xml
<Sequence>
  <!-- 사람 감지 -->
  <IsDetectedSomething target_class="person" 
                       confidence_threshold="0.5"
                       use_chroma="true"/>
  
  <!-- 등록되지 않은 사람이면 등록 -->
  <Fallback>
    <IsRegisteredPerson target_class="person" similarity_threshold="0.75"/>
    <RegisterPerson target_class="person" person_name="홍길동"/>
  </Fallback>
</Sequence>
```

## 방법 2: blackboard를 통한 동적 등록

다른 노드에서 blackboard에 `person_name`을 설정하면, `RegisterPerson` 노드가 이를 사용합니다.

```xml
<Sequence>
  <IsDetectedSomething target_class="person"/>
  <!-- person_name을 blackboard에 설정하는 다른 노드 필요 -->
  <RegisterPerson target_class="person"/>  <!-- person_name 파라미터 없음 -->
</Sequence>
```

## 방법 3: ROS 토픽을 통한 런타임 등록 (추천)

실시간으로 사람을 등록할 수 있습니다.

### 등록 명령:
```bash
# 터미널에서 실행
ros2 topic pub --once /register_person_{node_name} std_msgs/String "data: '홍길동'"
```

예를 들어, 노드 이름이 `RegisterPerson_1`이면:
```bash
ros2 topic pub --once /register_person_RegisterPerson_1 std_msgs/String "data: '홍길동'"
```

### 동작 방식:
1. 사람이 카메라 앞에 서있어야 함 (YOLO로 감지되어야 함)
2. 위 명령을 실행하면 현재 감지된 사람이 "홍길동"으로 등록됨
3. 다음에 같은 사람이 감지되면 `IsRegisteredPerson`이 SUCCESS 반환

## 방법 4: 별도 등록 스크립트 사용

Python 스크립트로 사람을 등록할 수 있습니다:

```python
# register_person.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def register_person(name, node_name="RegisterPerson_1"):
    rclpy.init()
    node = Node('register_person_client')
    pub = node.create_publisher(String, f'/register_person_{node_name}', 10)
    
    msg = String()
    msg.data = name
    pub.publish(msg)
    node.get_logger().info(f'등록 요청 전송: {name}')
    
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    name = sys.argv[1] if len(sys.argv) > 1 else "Unknown"
    register_person(name)
```

사용법:
```bash
python3 register_person.py 홍길동
```

## 실제 사용 예시

### 예시 1: 자동 등록 (등록되지 않은 사람 자동 등록)
```xml
<ReactiveSequence>
  <CaptureCameraImage camera_topic="/camera/color/image_raw"/>
  <DisplayYOLODetection/>
  <Sequence>
    <IsDetectedSomething target_class="person" use_chroma="true"/>
    <Fallback>
      <IsRegisteredPerson target_class="person"/>
      <Sequence>
        <RegisterPerson target_class="person" person_name="Unknown"/>
        <SendTelegramMessage message="새로운 사람이 자동 등록되었습니다."/>
      </Sequence>
    </Fallback>
  </Sequence>
</ReactiveSequence>
```

### 예시 2: 수동 등록 (특정 이름으로 등록)
```xml
<ReactiveSequence>
  <CaptureCameraImage camera_topic="/camera/color/image_raw"/>
  <DisplayYOLODetection/>
  <Sequence>
    <IsDetectedSomething target_class="person" use_chroma="true"/>
    <!-- ROS 토픽으로 등록 요청이 오면 등록 -->
    <RegisterPerson target_class="person"/>
  </Sequence>
</ReactiveSequence>
```

그리고 별도 터미널에서:
```bash
ros2 topic pub --once /register_person_RegisterPerson_1 std_msgs/String "data: '홍길동'"
```

### 예시 3: 등록된 사람과 등록되지 않은 사람 구분
```xml
<ReactiveSequence>
  <CaptureCameraImage camera_topic="/camera/color/image_raw"/>
  <DisplayYOLODetection/>
  <Sequence>
    <IsDetectedSomething target_class="person" use_chroma="true"/>
    
    <Fallback>
      <!-- 등록된 사람: 환영 -->
      <Sequence>
        <IsRegisteredPerson target_class="person" similarity_threshold="0.75"/>
        <SendTelegramMessage message="등록된 사용자입니다. 환영합니다!"/>
      </Sequence>
      
      <!-- 등록되지 않은 사람: 경고 -->
      <Sequence>
        <SendTelegramMessage message="등록되지 않은 사람이 감지되었습니다!"/>
        <CaptureScreen/>
      </Sequence>
    </Fallback>
  </Sequence>
</ReactiveSequence>
```

## 주의사항

1. **사람이 카메라 앞에 있어야 함**: `RegisterPerson` 노드가 실행될 때 `IsDetectedSomething`으로 사람이 감지되어 있어야 합니다.

2. **등록된 사람 데이터베이스 위치**: 
   - `./chroma_db_registered_{node_name}/` 디렉토리에 저장됩니다.
   - 재시작 후에도 유지됩니다.

3. **유사도 임계값**: 
   - `similarity_threshold`가 너무 높으면 같은 사람도 다른 사람으로 인식될 수 있습니다.
   - 너무 낮으면 다른 사람도 같은 사람으로 인식될 수 있습니다.
   - 권장값: 0.7 ~ 0.8

4. **여러 각도에서 등록**: 같은 사람을 여러 각도에서 등록하면 인식 정확도가 향상됩니다.

