#!/bin/bash
set -e  # 에러 발생 시 즉시 중단

# 환경변수 로드
source "$(dirname "$0")/.env"

echo "🚀 LIMO Smart Sentinel 설치를 시작합니다..."
echo "📍 워크스페이스 경로: $WEGO_WS_PATH"

# 빌드 의존성 추가
sudo apt update
sudo apt install -y build-essential cmake make git

# 1. 워크스페이스 준비
echo "📂 워크스페이스 생성 중..."
cd "$WEGO_WS_PATH"

# 기존 빌드 결과물 제거 (클린 설치)
echo "🧹 기존 빌드 결과물 및 소스 제거 중..."
rm -rf build install log src

mkdir -p src

# 2. 패키지 다운로드 (이미 있으면 업데이트)
echo "📥 패키지 다운로드 중..."
vcs import src < "$LIMO_SENTINEL_PATH/limo.repos" || true

# 3. [중요] LIMO 패키지 누락 폴더 복구 (에러 원인 제거)
echo "🔧 LIMO 패키지 빈 폴더 강제 생성 중..."
mkdir -p src/limo_ros2/limo_car/worlds
mkdir -p src/limo_ros2/limo_car/models
mkdir -p src/limo_ros2/limo_car/maps
mkdir -p src/limo_ros2/limo_car/launch
mkdir -p src/limo_ros2/limo_car/urdf
mkdir -p src/limo_ros2/limo_car/log ### 누락 되었음
mkdir -p src/limo_ros2/limo_car/paramsvcs
mkdir -p src/limo_ros2/limo_car/src
mkdir -p src/limo_ros2/limo_base/launch

# 4. 의존성 설치
echo "📦 필수 라이브러리 설치 중..."
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 5. [수정됨] YDLidar-SDK 수동 설치
# (이전 스크립트에서 mkdir build가 빠져서 에러가 났을 수 있음)
if [ ! -f "/usr/local/lib/libydlidar_sdk.so" ]; then
    echo "📡 YDLidar SDK가 없어 설치합니다..."
    cd ~
    # 기존에 실패한 폴더가 있다면 삭제 후 재시도
    rm -rf YDLidar-SDK
    git clone https://github.com/YDLIDAR/YDLidar-SDK.git
    cd YDLidar-SDK
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    colcon build --symlink-install
    cd "$WEGO_WS_PATH"
else
    echo "✅ YDLidar SDK가 이미 설치되어 있습니다."
fi
