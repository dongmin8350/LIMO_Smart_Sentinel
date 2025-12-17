# launch/night_patrol.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nodes = []

    # 1) 모드 매니저: 실제 Orbbec 카메라 밝기 기반으로 DAY / NIGHT 자동 결정
    nodes.append(
        Node(
            package='limo_night_patrol',
            executable='mode_manager',
            name='mode_manager_node',
            output='screen',
            parameters=[{
                'mode_source': 'auto',
                'brightness_topic': '/camera/color/image_raw',
                'brightness_threshold': 40.0,
                'initial_mode': 'NIGHT',
            }],
        )
    )

    # 2) Depth 기반 장애물 감지 (실제 Orbbec depth 이미지 사용)
    nodes.append(
        Node(
            package='limo_night_patrol',
            executable='depth_obstacle_detector',
            name='depth_obstacle_detector_node',
            output='screen',
            parameters=[{
                'depth_topic': '/camera/depth/image_raw',
                'obstacle_threshold_m': 0.6,
                'min_pixels': 200,
            }],
        )
    )

    # 3) 침입자 감지 노드: 일정 시간 이상 장애물이 있으면 침입자로 판단
    nodes.append(
        Node(
            package='limo_night_patrol',
            executable='night_intruder_detector',
            name='night_intruder_detector_node',
            output='screen',
            parameters=[{
                'obstacle_topic': '/depth_obstacles',
                'hold_time_sec': 3.0,
            }],
        )
    )

    # 4) 카메라 캡처 서버: 실제 환경에서는 프로젝트 폴더 안에 저장하는 걸 추천
    nodes.append(
        Node(
            package='limo_night_patrol',
            executable='camera_capture_server',
            name='camera_capture_server_node',
            output='screen',
            parameters=[{
                'save_dir': '/home/hanjuntae/wego_ws/src/LIMO_Smart_Sentinel/night_captures',
                'image_topic': '/camera/color/image_raw',
            }],
        )
    )

    # 5) 야간 이벤트 핸들러: 장애물/침입자 → 사진 촬영 + patrol_event 발행
    nodes.append(
        Node(
            package='limo_night_patrol',
            executable='event_handler_night',
            name='event_handler_night_node',
            output='screen',
        )
    )

    # 6) 패트롤 매니저: Nav2의 navigate_to_pose 액션 서버와 직접 통신
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

    # 실제 로봇에서는 dummy_* 노드는 필요 없음.
    # Nav2가 /cmd_vel/nav2 → safety mux → /cmd_vel → limo_base 까지 처리한다고 가정.

    return LaunchDescription(nodes)
