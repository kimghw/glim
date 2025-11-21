#!/bin/bash

# Ouster 드라이버 실행 간편 스크립트

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SETUP_DIR="$(dirname "$SCRIPT_DIR")"

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 센서 설정
SENSOR_IP="192.168.10.10"

echo -e "${BLUE}=== Ouster 드라이버 관리 ===${NC}"
echo "센서 IP: $SENSOR_IP"
echo ""

source /opt/ros/jazzy/setup.bash

# Stopped 프로세스 정리
cleanup_stopped_processes() {
    STOPPED_PIDS=$(ps aux | grep -E "ouster_ros.*driver\.launch\.py|os_driver" | grep " T" | awk '{print $2}' || true)
    if [ ! -z "$STOPPED_PIDS" ]; then
        echo -e "${YELLOW}⚠ 일시 정지된(Stopped) 프로세스 발견${NC}"
        echo "PID: $STOPPED_PIDS"
        echo -e "${BLUE}→ 자동으로 종료합니다...${NC}"

        for pid in $STOPPED_PIDS; do
            kill -9 $pid 2>/dev/null
        done
        sleep 1
        echo -e "${GREEN}✓ 정리 완료${NC}"
        echo ""
    fi
}

# 정상 작동 중인 드라이버 확인
check_running_driver() {
    # 타임아웃 적용
    if timeout 3 ros2 node list 2>/dev/null | grep -q "os_driver\|os_sensor"; then
        echo -e "${GREEN}✓ Ouster 드라이버가 이미 실행 중입니다${NC}"
        echo ""
        echo "실행 중인 노드:"
        timeout 3 ros2 node list 2>/dev/null | grep -E "os_driver|os_sensor" || true
        echo ""

        # 터미널이 있는 경우에만 대화형으로 물어봄
        if [ -t 0 ]; then
            echo -e "${YELLOW}옵션:${NC}"
            echo "  1) 기존 드라이버 계속 사용"
            echo "  2) 기존 드라이버 종료 후 새로 시작"
            echo "  3) 취소"
            read -p "선택 [1-3]: " choice

            case $choice in
                1)
                    echo -e "${GREEN}기존 드라이버를 계속 사용합니다${NC}"
                    exit 0
                    ;;
                2)
                    echo -e "${YELLOW}기존 드라이버를 종료합니다...${NC}"
                    pkill -f "ros2 launch ouster_ros"
                    pkill -f "os_driver"
                    sleep 2
                    echo -e "${GREEN}✓ 종료 완료${NC}"
                    echo ""
                    ;;
                3)
                    echo "취소되었습니다"
                    exit 0
                    ;;
                *)
                    echo -e "${RED}잘못된 선택입니다${NC}"
                    exit 1
                    ;;
            esac
        else
            # 터미널이 없으면 기존 드라이버 유지
            echo -e "${GREEN}기존 드라이버를 계속 사용합니다 (자동 모드)${NC}"
            exit 0
        fi
    fi
}

# 1. 먼저 Stopped 프로세스 정리
cleanup_stopped_processes

# 2. 정상 작동 중인 드라이버 확인
check_running_driver

# 드라이버 시작
echo -e "${BLUE}Ouster 드라이버를 시작합니다...${NC}"
echo ""

# 센서 연결 확인
if ! ping -c 2 -W 2 $SENSOR_IP > /dev/null 2>&1; then
    echo -e "${RED}✗ 센서에 연결할 수 없습니다 ($SENSOR_IP)${NC}"
    echo "센서가 켜져 있고 네트워크가 연결되어 있는지 확인하세요."
    exit 1
fi

# 임시 파라미터 파일 생성 (record_ouster.sh와 동일한 방식)
TEMP_PARAMS="/tmp/ouster_driver_params_$$.yaml"
cat > "$TEMP_PARAMS" << EOF
ouster/os_driver:
  ros__parameters:
    sensor_hostname: '$SENSOR_IP'
    lidar_mode: '2048x10'
    timestamp_mode: 'TIME_FROM_INTERNAL_OSC'
    imu_port: 7503
    lidar_port: 7502
EOF

echo "파라미터 파일: $TEMP_PARAMS"
echo ""

# 드라이버 실행 (viz는 선택적)
ros2 launch ouster_ros driver.launch.py \
    params_file:="$TEMP_PARAMS" \
    viz:=True
