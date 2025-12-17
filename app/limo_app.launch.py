import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 1. Rosbridge (앱 통신 연결) - 포트 9091 사용
    # (포트 충돌 방지를 위해 9091로 변경됨)
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')
        ]),
        launch_arguments={'port': '9091'}.items()
    )

    # 2. Web Video Server (카메라 영상 송출)
    video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen'
    )

    # 3. Limo App Server (배터리, 위치, 신호등 서버)
    app_server_process = ExecuteProcess(
        cmd=['python3', '/home/wego/wego_ws/limo_app_server.py'],
        output='screen'
    )

    # 4. 로그 중계기 (Log Relay - 알림 메시지 전송용)
    log_relay_process = ExecuteProcess(
        cmd=['python3', '/home/wego/wego_ws/log_relay.py'],
        output='screen'
    )

    # [중요 수정] 5. 메인 로봇 (자동 실행 끔)
    # "뚜둑"거리는 충돌을 막기 위해 여기서 실행하지 않고 주석 처리했습니다.
    # 필요할 때 터미널에서 'python3 ~/wego_ws/main_robot.py'로 직접 실행하세요.
    
    # main_robot_process = ExecuteProcess(
    #     cmd=['python3', '/home/wego/wego_ws/main_robot.py'],
    #     output='screen'
    # )

    return LaunchDescription([
        rosbridge_launch,
        video_server_node,
        app_server_process,
        log_relay_process,
        # main_robot_process  
    ])