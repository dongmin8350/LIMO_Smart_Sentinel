from setuptools import setup
import os
from glob import glob

package_name = 'limo_patrol'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),  # ✅ 전부 설치
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wego',
    maintainer_email='wego@example.com',
    description='LIMO patrol package: LiDAR obstacle detection + one-shot day/night mode manager',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_perception_lidar_node = limo_patrol.object_perception_lidar_node:main',
            'mode_manager_node = limo_patrol.mode_manager_node:main',
            # 필요하면 아래도 추가 가능
            # 'waypoint_patrol_node = limo_patrol.waypoint_patrol_node:main',
            # 'flame_sensor_node = limo_patrol.flame_sensor_node:main',
        ],
    },
)
