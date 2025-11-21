#!/bin/bash

# ROS2 Jazzy 설치 스크립트 (Ubuntu 24.04)

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║        ROS2 Jazzy 설치 스크립트               ║${NC}"
echo -e "${BLUE}║        Ubuntu 24.04 LTS (Noble)                ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════╝${NC}"
echo ""

# sudo 권한 확인
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}이 스크립트는 sudo 권한이 필요합니다.${NC}"
    echo "다음 명령으로 실행하세요:"
    echo "  sudo $0 $@"
    exit 1
fi

# ========================================
# STEP 1: Locale 설정
# ========================================
echo -e "${YELLOW}[1/7] Locale 설정 확인...${NC}"
locale | grep -q "UTF-8"
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ UTF-8 locale 확인됨${NC}"
else
    echo -e "${YELLOW}UTF-8 locale 설정 중...${NC}"
    apt update && apt install -y locales
    locale-gen en_US en_US.UTF-8
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    echo -e "${GREEN}✓ Locale 설정 완료${NC}"
fi
echo ""

# ========================================
# STEP 2: Universe 저장소 활성화
# ========================================
echo -e "${YELLOW}[2/7] Universe 저장소 활성화...${NC}"
apt install -y software-properties-common
add-apt-repository -y universe
echo -e "${GREEN}✓ Universe 저장소 활성화 완료${NC}"
echo ""

# ========================================
# STEP 3: ROS2 GPG 키 추가
# ========================================
echo -e "${YELLOW}[3/7] ROS2 GPG 키 추가...${NC}"
apt update && apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo -e "${GREEN}✓ GPG 키 추가 완료${NC}"
echo ""

# ========================================
# STEP 4: ROS2 저장소 추가
# ========================================
echo -e "${YELLOW}[4/7] ROS2 저장소 추가...${NC}"
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
echo -e "${GREEN}✓ ROS2 저장소 추가 완료${NC}"
echo ""

# ========================================
# STEP 5: 패키지 업데이트
# ========================================
echo -e "${YELLOW}[5/7] 패키지 목록 업데이트...${NC}"
apt update
apt upgrade -y
echo -e "${GREEN}✓ 패키지 업데이트 완료${NC}"
echo ""

# ========================================
# STEP 6: ROS2 Jazzy Desktop 설치
# ========================================
echo -e "${YELLOW}[6/7] ROS2 Jazzy Desktop 설치 중...${NC}"
echo -e "${BLUE}이 작업은 시간이 걸릴 수 있습니다 (약 5-10분)${NC}"
apt install -y ros-jazzy-desktop
echo -e "${GREEN}✓ ROS2 Jazzy Desktop 설치 완료${NC}"
echo ""

# ========================================
# STEP 7: 개발 도구 설치
# ========================================
echo -e "${YELLOW}[7/7] ROS2 개발 도구 설치...${NC}"
apt install -y \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool
echo -e "${GREEN}✓ 개발 도구 설치 완료${NC}"
echo ""

# ========================================
# rosdep 초기화
# ========================================
echo -e "${YELLOW}rosdep 초기화 중...${NC}"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
fi
echo -e "${GREEN}✓ rosdep 초기화 완료${NC}"
echo ""

# ========================================
# 설치 완료
# ========================================
echo -e "${GREEN}╔════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║          ROS2 Jazzy 설치 완료! ✓              ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}설치된 ROS2 버전:${NC}"
echo "  Distribution: ROS2 Jazzy Jalisco"
echo "  Ubuntu: 24.04 LTS (Noble)"
echo ""
echo -e "${YELLOW}다음 단계:${NC}"
echo ""
echo "1. 일반 사용자로 rosdep 업데이트:"
echo -e "   ${GREEN}rosdep update${NC}"
echo ""
echo "2. ROS2 환경 설정 (매번 터미널을 열 때마다 필요):"
echo -e "   ${GREEN}source /opt/ros/jazzy/setup.bash${NC}"
echo ""
echo "3. 자동 설정을 위해 bashrc에 추가:"
echo -e "   ${GREEN}echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc${NC}"
echo ""
echo "4. ROS2 설치 확인:"
echo -e "   ${GREEN}ros2 --help${NC}"
echo ""
echo -e "${BLUE}Ouster 라이다 드라이버 설치는 다음 명령으로:${NC}"
echo -e "   ${GREEN}sudo apt install ros-jazzy-ouster-ros${NC}"
echo ""
