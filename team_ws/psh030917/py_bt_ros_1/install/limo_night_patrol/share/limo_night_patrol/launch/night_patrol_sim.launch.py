# launch/night_patrol_sim.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nodes = []

    # 1) 모드 매니저 (더미 RGB를 이용해서 DAY/NIGHT 자동 판단)
    nodes.append(
        Node(
            package='limo_night_patrol',
            executable='mode_manager',
            name='mode_manager_node',
            output='screen',
            parameters=[{
                # auto: /camera/color/image_raw 밝기 보고 DAY / NIGHT 결정
                'mode_source': 'auto',
                'brightness_topic': '/camera/color/image_raw',
                'brightness_threshold': 40.0,
                'initial_mode': 'NIGHT',
            }],
        )
    )

    # 2) 더미 RGB 퍼블리셔 (검은 화면을 /camera/color/image_raw 로 퍼블리시)
    nodes.append(
        Node(
            package='limo_night_patrol',
            executable='dummy_rgb_pub',
            name='dummy_rgb_publisher_node',
            output='screen',
        )
    )

    # 3) Depth 기반 장애물 감지 (시뮬에선 나중에 depth dummy 추가해서 테스트 가능)
    nodes.append(
        Node(
            package='limo_night_patrol',
            executable='depth_obstacle_detector',
            name='depth_obstacle_detector_node',
            output='screen',
            parameters=[{
                # 실제 로봇에서는 orbbec depth 토픽 (/camera/depth/image_raw) 사용
                'depth_topic': '/camera/depth/image_raw',
                'obstacle_threshold_m': 0.6,
                'min_pixels': 200,
            }],
        )
    )

    # 4) 침입자 감지: 일정 시간 이상 장애물이 있으면 night_intruder True 로 판단
    nodes.append(
        Node(
            package='limo_night_patrol',
            executable='night_intruder_detector',
            name='night_intruder_detector_node',
            output='screen',
            parameters=[{
                'obstacle_topic': '/depth_obstacles',
                'hold_time_sec': 3.0,   # 3초 이상 장애물이면 침입자
            }],
        )
    )

    # 5) 카메라 캡처 서버 (Trigger 서비스: camera/capture)
    nodes.append(
        Node(
            package='limo_night_patrol',
            executable='camera_capture_server',
            name='camera_capture_server_node',
            output='screen',
            parameters=[{
                'save_dir': '/tmp/night_captures',   # 시뮬에선 /tmp에 저장
                'image_topic': '/camera/color/image_raw',
            }],
        )
    )

    # 6) 야간 이벤트 핸들러 (장애물/침입자 → 사진 촬영 + patrol_event 발행)
    nodes.append(
        Node(
            package='limo_night_patrol',
            executable='event_handler_night',
            name='event_handler_night_node',
            output='screen',
        )
    )

    # 7) 패트롤 매니저 (Nav2 액션 클라이언트, 더미 네비게이터로 테스트)
    nodes.append(
        Node(
            package='limo_night_patrol',
            executable='patrol_manager',
            name='patrol_manager_node',
            output='screen',
            parameters=[{
                'frame_id': 'map',
            }],
        )
    )

    # 8) 더미 네비게이터 (navigate_to_pose 액션 서버를 흉내내는 노드)
    nodes.append(
        Node(
            package='limo_night_patrol',
            executable='dummy_navigator',
            name='dummy_navigator_node',
            output='screen',
        )
    )

    # 9) 더미 safety mux: /cmd_vel/nav2 → /cmd_vel로 단순 포워딩
    nodes.append(
        Node(
            package='limo_night_patrol',
            executable='dummy_safety_mux',
            name='dummy_safety_mux_node',
            output='screen',
        )
    )

    # 10) 더미 LIMO 베이스: /cmd_vel 구독해서 로그만 찍는 노드
    nodes.append(
        Node(
            package='limo_night_patrol',
            executable='dummy_limo_base',
            name='dummy_limo_base_node',
            output='screen',
        )
    )

    return LaunchDescription(nodes)
