```bash
## 로봇의 터미널 1
source /opt/ros/humble/setup.bash
cd wego_ws
source install/setup.bash
ros2 launch wego teleop_launch.py 
```

```bash
## 로봇의 터미널 2
source /opt/ros/humble/setup.bash
cd ~/app/
source install/setup.bash
ros2 launch limo_app_launch.py

```

```bash
## 로봇의 터미널 3
source /opt/ros/humble/setup.bash
cd wego_ws
source install/setup.bash
ros2 launch wego navigation_diff_launch.py 
```

```bash
## 로봇의 터미널 4
source /opt/ros/humble/setup.bash
cd ~/py_bt_ros1/
source install/setup.bash
python3 main.py

```
