# 🤖 LIMO Smart Sentinel (방범 순찰 로봇)

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue) ![Platform](https://img.shields.io/badge/Platform-LIMO-orange) ![License](https://img.shields.io/badge/License-Apache%202.0-green)

**AgileX LIMO 로봇을 활용한 자율주행 기반 실내 방범 및 순찰 시스템 프로젝트**입니다.
ROS 2 Humble 환경에서 SLAM, Navigation2, Computer Vision 기술을 융합하여 구현되었습니다.

---

## 🚀 주요 기능 (Key Features)

1.  **자율 주행 (Autonomous Navigation)**
    * LiDAR 기반 SLAM (Cartographer/SLAM Toolbox)을 이용한 정밀 지도 작성
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

## 📦 설치 방법 (Installation)

이 레포지토리는 **원클릭 자동 설치(Auto-Install)**를 지원합니다.
복잡한 의존성 설치 과정 없이 아래 스크립트 하나로 개발 환경을 구축할 수 있습니다.

### 1. 레포지토리 복제 (Clone)
```bash
cd ~
git clone https://github.com/psh030917/LIMO_Smart_Sentinel.git
cd LIMO_Smart_Sentinel
````

### 2\. 자동 설치 스크립트 실행

```bash
# 첫번째 명령어 입력 후 
sudo bash ./install1.sh
# 두번째 명령어 입력
bash ./install2.sh
```

> **참고:** 스크립트가 실행되면 `~/wego_ws` 워크스페이스를 생성하고, `limo.repos`에 정의된 모든 패키지(Driver, SLAM, Vision)를 다운로드 및 빌드합니다. (약 5\~10분 소요)

-----

## 🎮 실행 방법 (Usage)

설치가 완료된 후, 터미널에서 아래 명령어로 로봇을 구동할 수 있습니다.

### 1\. 하드웨어 및 센서 구동

```bash
# (1) 로봇 베이스 구동
ros2 launch limo_base limo_base.launch.py

# (2) LiDAR 센서 구동
ros2 launch ydlidar_ros2_driver ydlidar_launch.py

# (3) 카메라 센서 구동
ros2 launch orbbec_camera astra.launch.py
```

### 2\. 지도 작성 (SLAM)

```bash
ros2 launch slam_toolbox online_async_launch.py
# 지도 작성 후 저장 명령어:
# ros2 run nav2_map_server map_saver_cli -f ~/wego_ws/src/LIMO_Smart_Sentinel/maps/my_map
```

### 3\. 자율 주행 (Navigation)

```bash
ros2 launch nav2_bringup bringup_launch.py map:=/home/wego/wego_ws/src/LIMO_Smart_Sentinel/maps/my_map.yaml
```

-----

## 📂 프로젝트 구조 (Structure)

```text
LIMO_Smart_Sentinel/
├── install.sh        # 원클릭 설치 스크립트 (Environment Setup)
├── limo.repos        # 의존성 패키지 리스트 (ROS 2 Drivers & Tools)
├── maps/             # SLAM으로 생성된 지도 파일 (.yaml, .pgm)
├── README.md         # 프로젝트 설명서
└── src/              # (추후 업데이트) 방범 로직 소스코드
```

-----

## 👥 팀원 (Contributors)

  * **팀장:** [이름 입력] - PM, 시스템 통합, 아키텍처 설계
  * **팀원:** [이름 입력] - 내비게이션(Nav2), SLAM 지도 작성
  * **팀원:** [이름 입력] - 컴퓨터 비전(OpenCV), 센서 융합
  * **팀원:** [이름 입력] - 임베디드 제어, 하드웨어 유지보수

-----

```
```
