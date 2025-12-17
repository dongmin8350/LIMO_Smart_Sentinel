# 🤖 LIMO Smart Sentinel (방범 순찰 로봇)

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue) ![Platform](https://img.shields.io/badge/Platform-LIMO-orange) ![License](https://img.shields.io/badge/License-Apache%202.0-green)

**AgileX LIMO 로봇을 활용한 자율주행 기반 실내 방범 및 순찰 시스템 프로젝트**입니다.
ROS 2 Humble 환경에서 SLAM, Navigation2, Computer Vision 기술을 융합하여 구현되었습니다.

---

## 🚀 주요 기능 (Key Features)

1.  **자율 주행 (Autonomous Navigation)**
    * LiDAR 기반 SLAM (SLAM Toolbox)을 이용한 정밀 지도 작성
    * Nav2 스택을 활용한 동적 장애물 회피 및 최적 경로 계획
    * 좁은 통로 및 가벽 환경에서의 안정적인 주행

2.  **스마트 순찰 (Smart Patrol)**
    * 지정된 보안 구역(Waypoints)을 순차적으로 이동하며 감시
    * Behavior Tree 기반의 유연한 임무 수행 로직 (순찰 $\leftrightarrow$ 추적 $\leftrightarrow$ 복귀)

3.  **침입자 감지 (Intruder Detection)**
    * **주간:** RGB 카메라(Orbbec Astra)를 활용한 객체 인식 및 색상 추적
    * **야간/암전:** LiDAR 센서 기반의 동적 장애물(움직이는 물체) 감지
    * **대응:** 침입자 발견 시 경고 로그 출력, 이미지 캡처 및 관제탑 전송

---

## 🛠️ 시스템 환경 (Environment)

* **Hardware:** AgileX LIMO (NVIDIA Jetson Nano / Orin Nano)
* **Sensors:** YDLIDAR X2/G4, Orbbec Astra Stereo Camera, IMU
* **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)
* **Middleware:** ROS 2 Humble Hawksbill
* **Languages:** Python 3.10, C++

---

## 📦 설치 가이드 (Installation Guide)

이 프로젝트는 **`wego_ws`** 워크스페이스를 기준으로 설정되어 있습니다.
아래 절차를 순서대로 따라 하면 오류 없이 환경을 구축할 수 있습니다.

### 1. 작업 공간 생성 및 필수 드라이버 복제
기존에 `src` 폴더가 있다면 백업 후 진행하는 것을 권장합니다.

```bash
# 워크스페이스 생성
mkdir -p ~/wego_ws/src
cd ~/wego_ws/src

# (1) LIMO 기본 구동 패키지
git clone -b humble https://github.com/agilexrobotics/limo_ros2.git

# (2) LiDAR 드라이버
git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git

# (3) 카메라 드라이버 (검증된 버전)
git clone -b v1.5.7 https://github.com/orbbec/OrbbecSDK_ROS2.git

# (4) LIMO Smart Sentinel 프로젝트 (본 레포지토리)
git clone https://github.com/dongmin8350/LIMO_Smart_Sentinel.git
````

### 2\. 의존성 설치 및 SLAM 툴박스 설정

빌드 시간을 단축하고 오류를 방지하기 위해 `slam_toolbox`는 패키지 관리자로 설치합니다.

```bash
cd ~/wego_ws

# 시스템 업데이트 및 SLAM Toolbox 설치
sudo apt update
sudo apt install ros-humble-slam-toolbox -y

# 의존성 자동 설치 (시뮬레이션 관련 키 제외)
rosdep update
rosdep install --from-paths src --ignore-src -r -y --skip-keys "libgazebo_ros rviz"
```

### 3\. 빌드 오류 사전 방지 (Critical Fixes)

`limo_car` 패키지 빌드 시 발생하는 누락된 폴더 에러를 방지하기 위해 빈 디렉토리를 미리 생성합니다.

```bash
mkdir -p ~/wego_ws/src/limo_ros2/limo_car/log
mkdir -p ~/wego_ws/src/limo_ros2/limo_car/src
mkdir -p ~/wego_ws/src/limo_ros2/limo_car/worlds
```

### 4\. 전체 빌드 (Build)

```bash
cd ~/wego_ws
# 이전 빌드 캐시 삭제 (Clean Build)
rm -rf build install log

# 심볼릭 링크 빌드
colcon build --symlink-install

# 환경 변수 적용
source ~/.bashrc
source install/setup.bash
```

### 5\. USB 권한 설정 (필수)

로봇 하드웨어(MCU)와 LiDAR 접근 권한을 설정합니다. **설정 후 USB를 재연결하거나 재부팅해야 합니다.**

```bash
cd ~/wego_ws/src/limo_ros2/limo_base/scripts
sudo bash create_udev_rules.sh
```

-----

## 🎮 실행 방법 (Usage)

설치가 완료되면 터미널을 열고 아래 명령어로 로봇을 구동합니다.

### 1\. 통합 구동 (추천)

모든 센서(Base, LiDAR, Camera)를 한 번에 실행합니다.

```bash
ros2 launch wego teleop_launch.py
```

### 2\. 개별 모듈 테스트

문제가 발생할 경우 각 모듈을 따로 실행하여 확인할 수 있습니다.

```bash
# 로봇 베이스 구동
ros2 launch limo_base limo_base.launch.py

# LiDAR 센서 구동
ros2 launch ydlidar_ros2_driver ydlidar.launch.py

# 카메라 센서 구동
ros2 launch orbbec_camera astra_stereo_u3.launch.py
```

### 3\. 지도 작성 (SLAM)

```bash
ros2 launch slam_toolbox online_async_launch.py
```

### 4\. 자율 주행 (Navigation)

```bash
ros2 launch nav2_bringup bringup_launch.py map:=/home/wego/wego_ws/src/LIMO_Smart_Sentinel/maps/my_map.yaml
```

-----

## 👥 팀원 (Contributors)

  * **팀장:** [장동민] - 시스템 통합, 내비게이션(Nav2), SLAM 지도 작성
  * **팀원:** [박성현] - 컴퓨터 비전(OpenCV)
  * **팀원:** [손민근] - 재난 대응 시스템
  * **팀원:** [한준태] - 야간 주행 시스템, 아키텍처 설계

<!-- end list -->

```
```