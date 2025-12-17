from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # --- package share dir ---
    pkg_dir = get_package_share_directory('limo_patrol')

    # --- params yaml for LiDAR perception (installed under share/limo_patrol/config/) ---
    lidar_params = os.path.join(pkg_dir, 'config', 'lidar_perception.yaml')

    # ---------------- Mode Manager (AUTO) ----------------
    # 카메라 밝기 평균으로 DAY/NIGHT 결정
    mode_manager = Node(
        package='limo_patrol',
        executable='mode_manager_node',
        name='mode_manager_node',
        output='screen',
        parameters=[
            {'mode_source': 'auto'},
            {'initial_mode': 'NIGHT'},
            {'brightness_topic': '/camera/color/image_raw'},  # 네 환경에 맞는 토픽
            {'brightness_threshold': 50.0},
        ],
    )

    # ---------------- LiDAR Object Perception ----------------
    # 맵 필터 + 동적(움직임) 객체 감지 -> 정지 + 이벤트(/patrol/event) 발행
    object_perception = Node(
        package='limo_patrol',
        executable='object_perception_lidar_node',
        name='object_perception_lidar_node',
        output='screen',
        parameters=[lidar_params],  # ✅ config yaml 로딩
    )

    return LaunchDescription([
        mode_manager,
        object_perception,
    ])
