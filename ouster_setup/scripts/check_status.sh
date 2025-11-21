#!/bin/bash

# Ouster 및 ROS2 프로세스 상태 확인 스크립트

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color
BOLD='\033[1m'

echo -e "${BLUE}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║           Ouster & ROS2 프로세스 상태 확인                    ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""

# ROS2 환경 설정
source /opt/ros/jazzy/setup.bash 2>/dev/null

# 프로세스 상태 코드 설명
explain_state() {
    case $1 in
        "R") echo "Running (CPU 실행 중)" ;;
        "S") echo "Interruptible Sleep (정상 작동 - 이벤트 대기)" ;;
        "D") echo "Uninterruptible Sleep (I/O 처리 중)" ;;
        "T") echo "Stopped (일시 정지 - Ctrl+Z)" ;;
        "t") echo "Traced (디버거 추적 중)" ;;
        "Z") echo "Zombie (종료 대기 중)" ;;
        "X") echo "Dead (종료됨)" ;;
        "I") echo "Idle (유휴)" ;;
        "W") echo "Paging (메모리 스왑)" ;;
        "<") echo "High priority (높은 우선순위)" ;;
        "N") echo "Low priority (낮은 우선순위)" ;;
        "L") echo "Has locked pages (메모리 잠금)" ;;
        "s") echo "Session leader (세션 리더)" ;;
        "l") echo "Multi-threaded (멀티스레드)" ;;
        "+") echo "Foreground (포그라운드)" ;;
        *) echo "Unknown (알 수 없음)" ;;
    esac
}

# ========================================
# 1. ROS2 데몬 상태
# ========================================
echo -e "${CYAN}[1] ROS2 데몬 상태${NC}"
echo -e "${BOLD}─────────────────────────────────────────────────${NC}"

DAEMON_INFO=$(ps aux | grep -E "ros2-daemon" | grep -v grep)
if [ ! -z "$DAEMON_INFO" ]; then
    PID=$(echo "$DAEMON_INFO" | awk '{print $2}')
    CPU=$(echo "$DAEMON_INFO" | awk '{print $3}')
    MEM=$(echo "$DAEMON_INFO" | awk '{print $4}')
    STAT=$(echo "$DAEMON_INFO" | awk '{print $8}')
    START=$(echo "$DAEMON_INFO" | awk '{print $9}')
    TIME=$(echo "$DAEMON_INFO" | awk '{print $10}')

    echo -e "  ${GREEN}● ROS2 데몬이 실행 중입니다${NC}"
    echo -e "    PID:        $PID"
    echo -e "    상태:       $STAT ($(explain_state ${STAT:0:1}))"
    echo -e "    CPU 사용률:  ${CPU}%"
    echo -e "    메모리 사용: ${MEM}%"
    echo -e "    시작 시간:   $START"
    echo -e "    실행 시간:   $TIME"
else
    echo -e "  ${RED}○ ROS2 데몬이 실행되지 않았습니다${NC}"
fi
echo ""

# ========================================
# 2. Ouster 드라이버 프로세스
# ========================================
echo -e "${CYAN}[2] Ouster 드라이버 프로세스${NC}"
echo -e "${BOLD}─────────────────────────────────────────────────${NC}"

# Launch 프로세스 확인
LAUNCH_INFO=$(ps aux | grep -E "ros2.*launch.*ouster_ros.*driver\.launch" | grep -v grep)
if [ ! -z "$LAUNCH_INFO" ]; then
    echo -e "  ${GREEN}● Launch 프로세스${NC}"
    while IFS= read -r line; do
        PID=$(echo "$line" | awk '{print $2}')
        STAT=$(echo "$line" | awk '{print $8}')
        CPU=$(echo "$line" | awk '{print $3}')
        MEM=$(echo "$line" | awk '{print $4}')

        # 상태에 따른 색상
        if [[ "$STAT" == *"T"* ]]; then
            STATE_COLOR="${YELLOW}"
            STATE_ICON="⚠"
        else
            STATE_COLOR="${GREEN}"
            STATE_ICON="✓"
        fi

        echo -e "    ${STATE_ICON} PID: ${PID}"
        echo -e "      상태: ${STATE_COLOR}$STAT${NC} ($(explain_state ${STAT:0:1}))"
        echo -e "      CPU: ${CPU}% | MEM: ${MEM}%"
    done <<< "$LAUNCH_INFO"
else
    echo -e "  ${RED}○ Launch 프로세스가 없습니다${NC}"
fi

# os_driver 노드 프로세스 확인
DRIVER_INFO=$(ps aux | grep -E "os_driver" | grep -v grep)
if [ ! -z "$DRIVER_INFO" ]; then
    echo -e "  ${GREEN}● os_driver 노드${NC}"
    while IFS= read -r line; do
        PID=$(echo "$line" | awk '{print $2}')
        STAT=$(echo "$line" | awk '{print $8}')
        CPU=$(echo "$line" | awk '{print $3}')
        MEM=$(echo "$line" | awk '{print $4}')

        # 상태에 따른 색상
        if [[ "$STAT" == *"T"* ]]; then
            STATE_COLOR="${YELLOW}"
            STATE_ICON="⚠"
        else
            STATE_COLOR="${GREEN}"
            STATE_ICON="✓"
        fi

        echo -e "    ${STATE_ICON} PID: ${PID}"
        echo -e "      상태: ${STATE_COLOR}$STAT${NC} ($(explain_state ${STAT:0:1}))"
        echo -e "      CPU: ${CPU}% | MEM: ${MEM}%"
    done <<< "$DRIVER_INFO"
else
    echo -e "  ${RED}○ os_driver 노드가 없습니다${NC}"
fi
echo ""

# ========================================
# 3. 실행 스크립트 프로세스
# ========================================
echo -e "${CYAN}[3] 실행 스크립트${NC}"
echo -e "${BOLD}─────────────────────────────────────────────────${NC}"

# run_driver.sh 확인
RUN_DRIVER=$(ps aux | grep -E "run_driver\.sh" | grep -v grep)
if [ ! -z "$RUN_DRIVER" ]; then
    echo -e "  ${GREEN}● run_driver.sh${NC}"
    PID=$(echo "$RUN_DRIVER" | awk '{print $2}')
    STAT=$(echo "$RUN_DRIVER" | awk '{print $8}')

    if [[ "$STAT" == *"T"* ]]; then
        echo -e "    ${YELLOW}⚠ PID: $PID - 일시 정지됨 (Stopped)${NC}"
    else
        echo -e "    ${GREEN}✓ PID: $PID - 실행 중${NC}"
    fi
else
    echo -e "  ${GRAY}○ run_driver.sh가 실행되지 않음${NC}"
fi

# record_ouster.sh 확인
RECORD_SCRIPT=$(ps aux | grep -E "record_ouster\.sh" | grep -v grep)
if [ ! -z "$RECORD_SCRIPT" ]; then
    echo -e "  ${GREEN}● record_ouster.sh${NC}"
    PID=$(echo "$RECORD_SCRIPT" | awk '{print $2}')
    STAT=$(echo "$RECORD_SCRIPT" | awk '{print $8}')

    if [[ "$STAT" == *"T"* ]]; then
        echo -e "    ${YELLOW}⚠ PID: $PID - 일시 정지됨 (Stopped)${NC}"
    else
        echo -e "    ${GREEN}✓ PID: $PID - 실행 중${NC}"
    fi
else
    echo -e "  ${GRAY}○ record_ouster.sh가 실행되지 않음${NC}"
fi

# rosbag record 프로세스 확인
ROSBAG_RECORD=$(ps aux | grep -E "ros2 bag record" | grep -v grep)
if [ ! -z "$ROSBAG_RECORD" ]; then
    echo -e "  ${GREEN}● rosbag 녹화${NC}"
    while IFS= read -r line; do
        PID=$(echo "$line" | awk '{print $2}')
        STAT=$(echo "$line" | awk '{print $8}')

        if [[ "$STAT" == *"T"* ]]; then
            echo -e "    ${YELLOW}⚠ PID: $PID - 일시 정지됨 (Stopped)${NC}"
        else
            echo -e "    ${GREEN}✓ PID: $PID - 녹화 중${NC}"
        fi
    done <<< "$ROSBAG_RECORD"
else
    echo -e "  ${GRAY}○ rosbag 녹화가 실행되지 않음${NC}"
fi
echo ""

# ========================================
# 4. ROS2 노드 상태
# ========================================
echo -e "${CYAN}[4] ROS2 노드 목록${NC}"
echo -e "${BOLD}─────────────────────────────────────────────────${NC}"

NODES=$(ros2 node list 2>/dev/null)
if [ ! -z "$NODES" ]; then
    echo -e "  ${GREEN}활성 노드:${NC}"
    echo "$NODES" | sed 's/^/    /'

    # os_driver 노드가 있으면 상세 정보
    if echo "$NODES" | grep -q "os_driver"; then
        echo ""
        echo -e "  ${BLUE}os_driver 노드 정보:${NC}"

        # Lifecycle 상태 확인
        LIFECYCLE=$(timeout 2 ros2 lifecycle get /ouster/os_driver 2>/dev/null || echo "N/A")
        if [ "$LIFECYCLE" != "N/A" ]; then
            echo -e "    Lifecycle 상태: $LIFECYCLE"
        fi
    fi
else
    echo -e "  ${RED}활성 노드가 없습니다${NC}"
fi
echo ""

# ========================================
# 5. ROS2 토픽 상태
# ========================================
echo -e "${CYAN}[5] Ouster 토픽 상태${NC}"
echo -e "${BOLD}─────────────────────────────────────────────────${NC}"

TOPICS=$(ros2 topic list 2>/dev/null | grep "/ouster")
if [ ! -z "$TOPICS" ]; then
    TOPIC_COUNT=$(echo "$TOPICS" | wc -l)
    echo -e "  ${GREEN}발행 중인 Ouster 토픽: ${TOPIC_COUNT}개${NC}"
    echo ""

    # 주요 토픽만 Hz 체크 (전체 체크는 시간이 오래 걸림)
    MAIN_TOPICS="/ouster/points /ouster/imu /ouster/scan"

    echo -e "  ${CYAN}주요 토픽 데이터 속도:${NC}"
    for topic in $MAIN_TOPICS; do
        if echo "$TOPICS" | grep -q "^${topic}$"; then
            # 메시지가 있는지 빠르게 확인 (echo --once 사용)
            if timeout 2 ros2 topic echo "$topic" --once > /dev/null 2>&1; then
                # 데이터가 있으면 Hz 측정 (3초간)
                HZ=$(timeout 3 ros2 topic hz "$topic" 2>/dev/null | grep "average rate" | head -1 | awk '{print $3}')
                if [ ! -z "$HZ" ]; then
                    echo -e "    ${GREEN}✓${NC} $topic: ${GREEN}${HZ} Hz${NC}"
                else
                    echo -e "    ${GREEN}✓${NC} $topic: ${GREEN}데이터 발행 중${NC}"
                fi
            else
                echo -e "    ${YELLOW}○${NC} $topic: 데이터 없음"
            fi
        else
            echo -e "    ${GRAY}✗${NC} $topic: 토픽 없음"
        fi
    done

    echo ""
    echo -e "  ${CYAN}전체 토픽 목록:${NC}"
    echo "$TOPICS" | sed 's/^/    /'
else
    echo -e "  ${RED}Ouster 토픽이 없습니다${NC}"
fi
echo ""

# ========================================
# 6. 네트워크 연결 상태
# ========================================
echo -e "${CYAN}[6] Ouster 센서 연결 상태${NC}"
echo -e "${BOLD}─────────────────────────────────────────────────${NC}"

SENSOR_IP="192.168.10.10"
if ping -c 1 -W 1 $SENSOR_IP > /dev/null 2>&1; then
    echo -e "  ${GREEN}● 센서 연결됨 ($SENSOR_IP)${NC}"

    # UDP 포트는 netstat/ss로 확인 (nc는 TCP만 체크함)
    # 대신 실제 데이터 수신 여부로 판단
    echo -e "  ${CYAN}UDP 데이터 스트림 상태:${NC}"

    # 로컬에서 UDP 포트 리스닝 확인
    LIDAR_LISTEN=$(ss -ulpn 2>/dev/null | grep ":7502" || netstat -ulpn 2>/dev/null | grep ":7502")
    IMU_LISTEN=$(ss -ulpn 2>/dev/null | grep ":7503" || netstat -ulpn 2>/dev/null | grep ":7503")

    if [ ! -z "$LIDAR_LISTEN" ]; then
        echo -e "    ${GREEN}✓ LiDAR 데이터 포트 (7502/UDP) 수신 대기 중${NC}"
    else
        echo -e "    ${YELLOW}○ LiDAR 포트 (7502/UDP) 대기 안 함${NC}"
    fi

    if [ ! -z "$IMU_LISTEN" ]; then
        echo -e "    ${GREEN}✓ IMU 데이터 포트 (7503/UDP) 수신 대기 중${NC}"
    else
        echo -e "    ${YELLOW}○ IMU 포트 (7503/UDP) 대기 안 함${NC}"
    fi

    # 웹 API 접근 확인 (HTTP/TCP)
    if curl -s --connect-timeout 1 http://$SENSOR_IP/api/v1/sensor/metadata > /dev/null 2>&1; then
        echo -e "    ${GREEN}✓ 센서 API (HTTP) 응답 정상${NC}"
    else
        echo -e "    ${YELLOW}○ 센서 API 응답 없음${NC}"
    fi
else
    echo -e "  ${RED}○ 센서에 연결할 수 없음 ($SENSOR_IP)${NC}"
fi
echo ""

# ========================================
# 7. 모든 관련 프로세스 상세 정보
# ========================================
echo -e "${CYAN}[7] 전체 프로세스 목록 (PID, 상태, 명령)${NC}"
echo -e "${BOLD}─────────────────────────────────────────────────${NC}"

# 모든 관련 프로세스를 한번에 가져오기
ALL_PROCS=$(ps aux | grep -E "(ros2|ouster|rviz|os_driver)" | grep -v grep | grep -v "check_status")

if [ ! -z "$ALL_PROCS" ]; then
    # 헤더 출력
    printf "  ${BOLD}%-7s %-4s %-4s %-6s %-10s %s${NC}\n" "PID" "CPU%" "MEM%" "상태" "설명" "명령"
    echo "  ────────────────────────────────────────────────────────────────────"

    while IFS= read -r line; do
        PID=$(echo "$line" | awk '{print $2}')
        CPU=$(echo "$line" | awk '{print $3}')
        MEM=$(echo "$line" | awk '{print $4}')
        STAT=$(echo "$line" | awk '{print $8}')
        CMD=$(echo "$line" | awk '{for(i=11;i<=NF;i++) printf "%s ", $i; print ""}' | cut -c1-40)

        # 상태 설명 짧게
        STATE_SHORT=""
        if [[ "$STAT" == *"T"* ]]; then
            STATE_SHORT="${YELLOW}정지됨${NC}"
        elif [[ "$STAT" == *"R"* ]]; then
            STATE_SHORT="${GREEN}실행중${NC}"
        elif [[ "$STAT" == *"S"* ]]; then
            STATE_SHORT="${GREEN}정상${NC}"
        elif [[ "$STAT" == *"D"* ]]; then
            STATE_SHORT="${GREEN}I/O중${NC}"
        elif [[ "$STAT" == *"Z"* ]]; then
            STATE_SHORT="${RED}좀비${NC}"
        else
            STATE_SHORT="$STAT"
        fi

        # 프로세스 타입 판별
        PROC_TYPE=""
        if echo "$line" | grep -q "ros2-daemon"; then
            PROC_TYPE="데몬"
        elif echo "$line" | grep -q "driver\.launch"; then
            PROC_TYPE="Launch"
        elif echo "$line" | grep -q "os_driver"; then
            PROC_TYPE="드라이버"
        elif echo "$line" | grep -q "rviz"; then
            PROC_TYPE="RViz"
        elif echo "$line" | grep -q "ros2 bag record"; then
            PROC_TYPE="녹화"
        elif echo "$line" | grep -q "run_driver"; then
            PROC_TYPE="스크립트"
        elif echo "$line" | grep -q "record_ouster"; then
            PROC_TYPE="스크립트"
        else
            PROC_TYPE="기타"
        fi

        printf "  %-7s %-4s %-4s %-6b %-10s %s\n" "$PID" "$CPU" "$MEM" "$STATE_SHORT" "$PROC_TYPE" "$CMD..."
    done <<< "$ALL_PROCS"
else
    echo -e "  ${GRAY}관련 프로세스가 없습니다${NC}"
fi
echo ""

# ========================================
# 8. 권장 조치
# ========================================
echo -e "${CYAN}[8] 상태 요약 및 권장 조치${NC}"
echo -e "${BOLD}─────────────────────────────────────────────────${NC}"

# 전체 시스템 상태 평가
HAS_ISSUES=false

# 1. Stopped 프로세스 확인
STOPPED_PROCS=$(ps aux | grep -E "(ouster|ros2.*launch)" | grep " T" | wc -l)
if [ $STOPPED_PROCS -gt 0 ]; then
    HAS_ISSUES=true
    echo -e "  ${YELLOW}⚠ 일시 정지된 프로세스가 ${STOPPED_PROCS}개 있습니다${NC}"

    # Stopped 프로세스 PID 목록
    STOPPED_PIDS=$(ps aux | grep -E "(ouster|ros2.*launch)" | grep " T" | awk '{print $2}')
    echo -e "    정지된 PID: $STOPPED_PIDS"
    echo -e "    ${CYAN}조치:${NC}"
    echo -e "      • 재개: fg (포그라운드) 또는 bg (백그라운드)"
    echo -e "      • 종료: kill -9 $STOPPED_PIDS"
    echo ""
fi

# 2. 드라이버 실행 여부 확인
if ! ros2 node list 2>/dev/null | grep -q "os_driver"; then
    HAS_ISSUES=true
    echo -e "  ${RED}⚠ Ouster 드라이버가 실행되지 않았습니다${NC}"
    echo -e "    ${CYAN}조치: ./run_driver.sh${NC}"
    echo ""
fi

# 3. 센서 데이터 발행 확인 (토픽 리스트로 빠르게 확인)
OUSTER_TOPICS=$(ros2 topic list 2>/dev/null | grep "/ouster/points")
if [ -z "$OUSTER_TOPICS" ]; then
    HAS_ISSUES=true
    echo -e "  ${YELLOW}⚠ 센서 토픽이 발행되지 않습니다${NC}"
    echo -e "    ${CYAN}조치: 센서 연결 및 드라이버 재시작 확인${NC}"
    echo ""
fi

# 4. 센서 연결 확인
if ! ping -c 1 -W 1 192.168.10.10 > /dev/null 2>&1; then
    HAS_ISSUES=true
    echo -e "  ${RED}⚠ 센서에 연결할 수 없습니다${NC}"
    echo -e "    ${CYAN}조치: 센서 전원 및 네트워크 케이블 확인${NC}"
    echo ""
fi

# 최종 상태 메시지
if [ "$HAS_ISSUES" = false ]; then
    echo -e "  ${GREEN}✅ 모든 시스템이 정상 작동 중입니다!${NC}"
    echo ""
    echo -e "  ${CYAN}시스템 요약:${NC}"
    echo -e "    • ROS2 드라이버: 정상"
    echo -e "    • 센서 연결: 정상"
    echo -e "    • 데이터 스트림: 정상"
    echo -e "    • 토픽 발행: 정상"
fi

echo ""
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "업데이트: $(date '+%Y-%m-%d %H:%M:%S')"