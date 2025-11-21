#!/bin/bash

# Ouster 라이다 데이터 rosbag 녹화 스크립트

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║     Ouster 라이다 ROS2 Bag 녹화 스크립트     ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════╝${NC}"
echo ""

# 도움말 출력
show_help() {
    echo "사용법:"
    echo "  $0 [bag_파일명] [녹화시간(초)] [옵션]"
    echo ""
    echo "예제:"
    echo "  $0                                    # 자동 파일명으로 무한 녹화"
    echo "  $0 my_data                           # my_data.db3로 무한 녹화"
    echo "  $0 my_data 60                        # my_data.db3로 60초 녹화"
    echo "  $0 my_data 60 --all                  # 모든 토픽 녹화 (60초)"
    echo ""
    echo "옵션:"
    echo "  --all           모든 Ouster 토픽 녹화 (기본: raw packets만)"
    echo "  --with-viz      RViz 함께 실행"
    echo "  -h, --help      도움말 표시"
    echo ""
}

# 파라미터 파싱
BAG_FILE=""
DURATION=""
RECORD_ALL=false
WITH_VIZ=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        --all)
            RECORD_ALL=true
            shift
            ;;
        --with-viz)
            WITH_VIZ=true
            shift
            ;;
        *)
            if [ -z "$BAG_FILE" ]; then
                BAG_FILE=$1
            elif [ -z "$DURATION" ]; then
                DURATION=$1
            fi
            shift
            ;;
    esac
done

# 기본값 설정
if [ -z "$BAG_FILE" ]; then
    BAG_FILE="ouster_$(date +%Y%m%d_%H%M%S)"
fi

# 센서 설정
SENSOR_IP="192.168.10.10"
OUSTER_NS="ouster"

echo -e "${GREEN}녹화 설정:${NC}"
echo "  센서 IP        : $SENSOR_IP"
echo "  Bag 파일       : ${BAG_FILE}.db3"
if [ -n "$DURATION" ]; then
    echo "  녹화 시간      : ${DURATION}초"
else
    echo "  녹화 시간      : 무제한 (Ctrl+C로 중지)"
fi
echo "  네임스페이스   : $OUSTER_NS"
if [ "$RECORD_ALL" = true ]; then
    echo "  녹화 모드      : 전체 토픽"
else
    echo "  녹화 모드      : Raw packets (권장)"
fi
echo ""

# ROS2 환경 설정
source /opt/ros/jazzy/setup.bash

# ========================================
# STEP 1: 센서 노드 확인 및 실행
# ========================================
echo -e "${YELLOW}[1/4] 센서 노드 확인 중...${NC}"

# Stopped 프로세스 자동 정리
cleanup_stopped_driver() {
    STOPPED_DRIVER=$(ps aux | grep -E "ouster_ros.*driver\.launch\.py" | grep " T" || true)
    if [ ! -z "$STOPPED_DRIVER" ]; then
        STOPPED_PID=$(echo "$STOPPED_DRIVER" | awk '{print $2}')
        echo -e "${YELLOW}⚠ 일시 정지된 Ouster 드라이버 발견 (PID: $STOPPED_PID)${NC}"
        echo -e "${BLUE}→ 자동으로 종료합니다...${NC}"

        kill -9 $STOPPED_PID 2>/dev/null
        pkill -f "os_driver" 2>/dev/null
        sleep 1
        echo -e "${GREEN}✓ 정리 완료${NC}"
        echo ""
    fi
}

# Stopped 프로세스 정리
cleanup_stopped_driver

# 실행 중인 드라이버 확인
set +e  # 임시로 에러 무시
NODE_LIST=$(timeout 3 ros2 node list 2>/dev/null)
NODE_CHECK_EXIT=$?
set -e  # 에러 체크 재활성화

if [ $NODE_CHECK_EXIT -eq 0 ] && echo "$NODE_LIST" | grep -q "os_driver\|os_sensor"; then
    echo -e "${GREEN}✓ Ouster 센서 노드가 이미 실행 중입니다${NC}"
    echo -e "${BLUE}기존 드라이버를 사용하여 녹화를 진행합니다${NC}"
    SENSOR_STARTED_BY_SCRIPT=false
    DRIVER_STATUS=0
else
    DRIVER_STATUS=1
fi

# 드라이버가 실행되지 않은 경우에만 새로 시작
if [ $DRIVER_STATUS -eq 1 ]; then
    echo -e "${YELLOW}⚠ Ouster 센서 노드가 실행되지 않았습니다${NC}"
    echo -e "${BLUE}센서 노드를 시작합니다...${NC}"
    echo ""

    # 센서 연결 확인
    if ! ping -c 2 -W 2 $SENSOR_IP > /dev/null 2>&1; then
        echo -e "${RED}✗ 센서에 연결할 수 없습니다 ($SENSOR_IP)${NC}"
        echo "센서가 켜져 있고 네트워크가 연결되어 있는지 확인하세요."
        exit 1
    fi

    # 임시 파라미터 파일 생성
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

    # 드라이버 시작 (driver.launch.py 사용하여 포인트클라우드 처리 보장)
    if [ "$WITH_VIZ" = true ]; then
        echo "실행 명령: ros2 launch ouster_ros driver.launch.py params_file:=$TEMP_PARAMS viz:=true (백그라운드)"
        nohup ros2 launch ouster_ros driver.launch.py params_file:=$TEMP_PARAMS viz:=true > /tmp/ouster_sensor.log 2>&1 &
    else
        echo "실행 명령: ros2 launch ouster_ros driver.launch.py params_file:=$TEMP_PARAMS viz:=false (백그라운드)"
        nohup ros2 launch ouster_ros driver.launch.py params_file:=$TEMP_PARAMS viz:=false > /tmp/ouster_sensor.log 2>&1 &
    fi
    SENSOR_PID=$!
    disown $SENSOR_PID 2>/dev/null || true

    echo -e "${BLUE}센서 초기화 대기 중...${NC}"
    sleep 8

    # 노드 시작 확인
    if ! ros2 node list 2>/dev/null | grep -q "os_driver\|os_sensor"; then
        echo -e "${RED}✗ 센서 노드 시작 실패${NC}"
        echo "로그 확인: cat /tmp/ouster_sensor.log"
        exit 1
    fi
    echo -e "${GREEN}✓ 센서 노드 시작 완료${NC}"

    # Lifecycle 노드 활성화
    echo -e "${BLUE}센서 노드 활성화 중...${NC}"

    # Lifecycle 상태 확인 (타임아웃 5초)
    LIFECYCLE_STATE=$(timeout 5 ros2 lifecycle get /ouster/os_driver 2>/dev/null || echo "timeout")

    if echo "$LIFECYCLE_STATE" | grep -q "inactive"; then
        echo "  → Inactive 상태 감지, 활성화 시도..."
        if timeout 5 ros2 lifecycle set /ouster/os_driver activate > /dev/null 2>&1; then
            sleep 2
            echo -e "${GREEN}✓ 센서 노드 활성화 완료${NC}"
        else
            echo -e "${YELLOW}⚠ Lifecycle 활성화 실패 (타임아웃)${NC}"
            echo -e "${YELLOW}  → 토픽 데이터 확인 단계로 진행합니다${NC}"
        fi
    elif echo "$LIFECYCLE_STATE" | grep -q "active"; then
        echo -e "${GREEN}✓ 이미 Active 상태${NC}"
    elif echo "$LIFECYCLE_STATE" | grep -q "timeout"; then
        echo -e "${YELLOW}⚠ Lifecycle 상태 확인 실패 (타임아웃)${NC}"
        echo -e "${YELLOW}  → 토픽 데이터 확인 단계로 진행합니다${NC}"
    else
        echo -e "${YELLOW}⚠ 알 수 없는 Lifecycle 상태: $LIFECYCLE_STATE${NC}"
        echo -e "${YELLOW}  → 토픽 데이터 확인 단계로 진행합니다${NC}"
    fi

    SENSOR_STARTED_BY_SCRIPT=true
fi
echo ""

# ========================================
# STEP 2: 토픽 데이터 발행 확인
# ========================================
echo -e "${YELLOW}[2/4] 토픽 데이터 확인 중...${NC}"

# 토픽이 발행될 때까지 대기 (최대 10초)
for i in {1..10}; do
    # 토픽 존재 확인
    if ros2 topic list 2>/dev/null | grep -q "/ouster/points"; then
        # 실제 데이터 발행 확인 (timeout으로 1개 메시지 받기)
        if timeout 2 ros2 topic echo /ouster/points --once > /dev/null 2>&1; then
            echo -e "${GREEN}✓ 포인트 클라우드 데이터 발행 확인됨${NC}"
            break
        fi
    fi

    if [ $i -eq 10 ]; then
        echo -e "${RED}✗ 토픽 데이터가 발행되지 않습니다${NC}"
        echo "센서 상태를 확인하세요: ros2 topic list"
        exit 1
    fi

    echo "  대기 중... ($i/10)"
    sleep 1
done
echo ""

# ========================================
# STEP 3: 녹화 토픽 설정
# ========================================
echo -e "${YELLOW}[3/4] 녹화 토픽 설정...${NC}"

if [ "$RECORD_ALL" = true ]; then
    # 모든 Ouster 토픽 녹화
    TOPICS="/ouster/points \
            /ouster/imu \
            /ouster/scan \
            /ouster/range_image \
            /ouster/reflec_image \
            /ouster/nearir_image \
            /ouster/imu_packets \
            /ouster/lidar_packets \
            /ouster/metadata \
            /tf \
            /tf_static"
    echo -e "${BLUE}전체 토픽 녹화 모드 (모든 데이터)${NC}"
else
    # 기본 토픽만 녹화 - 포인트클라우드와 IMU 데이터
    TOPICS="/ouster/points \
            /ouster/imu \
            /ouster/metadata \
            /tf_static"
    echo -e "${BLUE}기본 토픽 녹화 모드 (포인트클라우드 + IMU)${NC}"
fi
echo ""

# ========================================
# STEP 4: Rosbag 녹화 시작
# ========================================
echo -e "${YELLOW}[4/4] Rosbag 녹화 시작...${NC}"
echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║              녹화 시작!                        ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${YELLOW}녹화를 중지하려면 Ctrl+C를 누르세요.${NC}"
echo ""

# 저장 디렉토리 생성
SAVE_DIR="/home/kimghw/glim/rosbag_data"
mkdir -p "$SAVE_DIR"
cd "$SAVE_DIR"

# 녹화 명령 생성 - 전체 경로 포함
FULL_PATH="$SAVE_DIR/$BAG_FILE"
RECORD_CMD="ros2 bag record -o $FULL_PATH"

# 지속 시간 추가
if [ -n "$DURATION" ]; then
    RECORD_CMD="$RECORD_CMD --max-cache-size 0 -d $DURATION"
fi

# 토픽 추가
RECORD_CMD="$RECORD_CMD $TOPICS"

# RViz와 함께 실행하는 경우
if [ "$WITH_VIZ" = true ]; then
    echo -e "${BLUE}RViz를 함께 실행합니다...${NC}"
    rviz2 -d /opt/ros/jazzy/share/ouster_ros/config/viz.rviz &
    RVIZ_PID=$!
    sleep 2
fi

# 녹화 시작
echo -e "${BLUE}명령: $RECORD_CMD${NC}"
echo ""

# 트랩 설정 (Ctrl+C 처리)
cleanup() {
    echo ""
    echo -e "${YELLOW}녹화를 중지합니다...${NC}"

    if [ -n "$RVIZ_PID" ]; then
        kill $RVIZ_PID 2>/dev/null || true
    fi

    # 스크립트가 시작한 센서 노드만 종료
    if [ "$SENSOR_STARTED_BY_SCRIPT" = "true" ] && [ -n "$SENSOR_PID" ]; then
        echo -e "${YELLOW}센서 노드를 종료합니다...${NC}"
        kill $SENSOR_PID 2>/dev/null || true
    elif [ "$SENSOR_STARTED_BY_SCRIPT" = "false" ]; then
        echo -e "${BLUE}기존 드라이버를 유지합니다 (다른 작업에서 사용 가능)${NC}"
    fi

    echo ""
    echo -e "${GREEN}╔════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║            녹화 완료!                          ║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${BLUE}저장 위치: $SAVE_DIR${NC}"

    # 파일 정보 표시
    if [ -d "$SAVE_DIR/$BAG_FILE" ]; then
        echo ""
        echo "녹화된 파일:"
        ls -lh "$SAVE_DIR/$BAG_FILE"
        echo ""
        echo "Bag 정보:"
        ros2 bag info "$SAVE_DIR/$BAG_FILE"
    fi

    # 메타데이터 수집 (선택적)
    # 터미널이 있는 경우에만 대화형으로 물어봄
    if [ -t 0 ]; then
        echo ""
        echo -e "${YELLOW}메타데이터를 수집하시겠습니까? (y/N)${NC}"
        read -t 10 -n 1 -r COLLECT_METADATA || COLLECT_METADATA="n"
        echo ""

        if [[ $COLLECT_METADATA =~ ^[Yy]$ ]]; then
            echo -e "${BLUE}메타데이터 수집 중...${NC}"
            python3 /home/kimghw/glim/ouster_setup/scripts/metadata_manager.py "$SAVE_DIR/$BAG_FILE" --interactive
        else
            # 기본 메타데이터만 생성
            echo -e "${BLUE}기본 메타데이터 생성 중...${NC}"
            python3 /home/kimghw/glim/ouster_setup/scripts/metadata_manager.py "$SAVE_DIR/$BAG_FILE"
        fi
    else
        # 터미널이 없으면 자동으로 기본 메타데이터만 생성
        echo -e "${BLUE}기본 메타데이터 생성 중...${NC}"
        python3 /home/kimghw/glim/ouster_setup/scripts/metadata_manager.py "$SAVE_DIR/$BAG_FILE"
    fi

    echo ""
    echo -e "${YELLOW}재생 방법:${NC}"
    echo "  # 단순 재생:"
    echo "  ros2 bag play $SAVE_DIR/$BAG_FILE"
    echo ""
    echo "  # 반복 재생:"
    echo "  ros2 bag play $SAVE_DIR/$BAG_FILE --loop"
    echo ""
    echo "  # RViz2로 시각화:"
    echo "  rviz2  # 별도 터미널에서 실행 후 /ouster/points 토픽 추가"
    echo ""

    exit 0
}

trap cleanup SIGINT SIGTERM

# 녹화 실행
eval $RECORD_CMD

# 시간 제한이 있는 경우 자동 종료
if [ -n "$DURATION" ]; then
    cleanup
fi
