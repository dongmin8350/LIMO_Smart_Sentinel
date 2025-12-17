# 6. 빌드 (가장 중요한 단계)
echo "🔨 전체 빌드 시작 (실패한 패키지도 다시 빌드)..."
# 환경변수 로드
source "$(dirname "$0")/.env"

cd "$WEGO_WS_PATH"

sudo chmod -R 777 .

colcon build --symlink-install

# 7. 환경 설정
echo "✅ 빌드 완료! 환경 변수를 설정합니다."
if ! grep -q "source $WEGO_WS_PATH/install/setup.bash" ~/.bashrc; then
    echo "source $WEGO_WS_PATH/install/setup.bash" >> ~/.bashrc
fi

echo "🎉 모든 설치가 완료되었습니다! 'source ~/.bashrc'를 입력하세요."