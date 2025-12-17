시뮬 런치에서 사용하는 구성

dummy_rgb_pub	: 완전 검은색 이미지를 /camera/color/image_raw 로 퍼블리시 → ModeManager가 NIGHT 모드로 전환하도록 만듦

mode_manager : 이미지 밝기를 읽어 DAY/NIGHT 모드 자동 판단

depth_obstacle_detector :	실제 depth 대신 dummy depth를 사용 (기본은 장애물 없음)

night_intruder_detector	: depth 장애물이 일정 시간 지속되면 침입자로 판단

dummy_navigator	: Nav2 흉내내는 navigate_to_pose 액션 서버

dummy_safety_mux : /cmd_vel/nav2 → /cmd_vel 포워딩

dummy_limo_base	: 실제 모터 대신 cmd_vel 로그만 출력

camera_capture_server	: RGB 이미지 수신 후 Trigger 서비스로 캡처 수행

event_handler_night	: 장애물/침입자 발생 시 사진 촬영 + 이벤트 발행

patrol_manager : 네비게이션 목표를 waypoint 순서대로 전송

----------------------------------------------------------------------------------------------------------------------

시뮬레이션 테스트 실행 방법

시뮬 테스트는 dummy 센서·dummy 네비게이터 노드를 사용해 실제 하드웨어 없이 전체 플로우를 검증합니다.

실행
ros2 launch limo_night_patrol night_patrol_sim.launch.py

----------------------------------------------------------------------------------------------------------------------

실제 로봇 구동 (실센서 기반)

실제 LIMO 로봇에서 동작시키려면 아래 런치를 실행합니다:

ros2 launch limo_night_patrol night_patrol.launch.py

----------------------------------------------------------------------------------------------------------------------

시스템 전체 동작 흐름

1 모드 자동 감지 (ModeManagerNode)

입력: /camera/color/image_raw

밝기 평균 계산
mean < threshold → NIGHT
mean >= threshold → DAY

결과: /patrol_mode 퍼블리시



2 Depth 장애물 감지 (DepthObstacleDetector)

입력: /camera/depth/image_raw
중앙 ROI 영역에서 가장 가까운 depth 픽셀 탐색
min_depth < threshold 이면 장애물 감지 True
출력: /depth_obstacles (Bool)

센서: Orbbec Depth Camera



3 침입자 감지 (NightIntruderDetector)

입력: /depth_obstacles
장애물이 일정 시간(기본 3초) 이상 지속되면 침입자로 판단
출력: /night_intruder

논리:

depth_obstacles == True → 시간 누적
누적된 시간이 hold_time 이상이면 → intruder True
장애물이 사라지면 → intruder False



4 사진 촬영 + 이벤트 송출 (EventHandlerNight)

입력:
/patrol_mode
/depth_obstacles
/night_intruder

조건:
현재 모드가 NIGHT일 때만 동작

이벤트 발생 시:
camera/capture Trigger 서비스 호출 → 사진 저장
/patrol_event 발행
   "NIGHT_OBSTACLE"
   "NIGHT_INTRUDER"

또한 과도한 사진 촬영 방지를 위해 10초 쿨다운 적용.



5 야간 순찰 관리자 (PatrolManager)

입력:
/patrol_mode
/patrol_event

기본 동작:
모드가 NIGHT이면 waypoint 순찰
Nav2 Action Server (navigate_to_pose) 로 목표 전송

이벤트 처리:
이벤트 도착 → 현재 goal 취소
일정 시간(기본 5초) 정지 후 순찰 재개



6 시뮬 테스트 전용 노드들
DummyNavigator
   : Nav2 흉내내는 fake 액션 서버
   : 매 goal 마다 3초 동안 forward cmd_vel 출력 후 성공(SUCCEEDED)

DummySafetyMux  
   : /cmd_vel/nav2 → /cmd_vel 포워딩

DummyLimoBase
   :cmd_vel 수신만 하고 출력(log)

DummyRGBPublisher
   : 완전 검정색(RGB 0,0,0) 이미지 출력
      → ModeManager가 NIGHT 모드로 자동 변경됨

---------------------------------------------------------------------------------------------------------------------

시뮬 동작 예시 로그

ModeManager(auto) → NIGHT 모드 판단
PatrolManager → waypoint 0, 1, 2 ... 순찰
DummyNavigator → SUCCEEDED
DummyLimoBase → cmd_vel 출력
EventHandler → 이벤트 발생 시 캡처 서비스 호출
CameraCaptureServer → /tmp/night_captures/... 저장
