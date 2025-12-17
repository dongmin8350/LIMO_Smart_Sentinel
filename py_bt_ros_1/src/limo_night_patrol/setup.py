from glob import glob
import os
from setuptools import setup

package_name = 'limo_night_patrol'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/limo_night_patrol']),
    ('share/limo_night_patrol', ['package.xml']),
    ('share/limo_night_patrol/launch', glob(os.path.join('launch', '*.launch.py'))),
]
,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='LIMO night patrol (night mode) nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'mode_manager = limo_night_patrol.mode_manager_node:main',
        'depth_obstacle_detector = limo_night_patrol.depth_obstacle_detector_node:main',
        'night_intruder_detector = limo_night_patrol.night_intruder_detector_node:main',
        'event_handler_night = limo_night_patrol.event_handler_night_node:main',
        'camera_capture_server = limo_night_patrol.camera_capture_server_node:main',
        'patrol_manager = limo_night_patrol.patrol_manager_node:main',

        # --- 새로 추가 ---
        'dummy_navigator = limo_night_patrol.dummy_navigator_node:main',
        'dummy_safety_mux = limo_night_patrol.dummy_safety_mux_node:main',
        'dummy_limo_base = limo_night_patrol.dummy_limo_base_node:main',
        'dummy_rgb_pub = limo_night_patrol.dummy_rgb_publisher_node:main',
    ],
},
)
