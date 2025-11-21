#!/bin/bash

# Ouster rosbag 재생 스크립트
# 녹화된 bag 파일을 재생하고 시각화

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║     Ouster ROS2 Bag 재생 스크립트            ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════╝${NC}"
echo ""

# 도움말
show_help() {
    echo "사용법:"
    echo "  $0 [bag_파일_경로] [옵션]"
    echo ""
    echo "예제:"
    echo "  $0                                    # 대화형 모드 (파일 선택) + RViz"
    echo "  $0 /path/to/bag                      # 특정 파일 재생 + RViz"
    echo "  $0 /path/to/bag --no-viz             # RViz 없이 재생만"
    echo "  $0 /path/to/bag --loop                # 무한 반복 재생 + RViz"
    echo "  $0 /path/to/bag --rate 0.5           # 0.5배속 재생 + RViz"
    echo ""
    echo "옵션:"
    echo "  --loop          무한 반복 재생"
    echo "  --rate [배속]   재생 속도 조정 (예: 0.5, 1.0, 2.0)"
    echo "  --viz           RViz 자동 실행 (기본값: 활성화)"
    echo "  --no-viz        RViz 실행 안함"
    echo "  --clock         시뮬레이션 시간 발행 (/clock)"
    echo "  -h, --help      도움말 표시"
    echo ""
}

# 파라미터 파싱
BAG_FILE=""
USE_LOOP=false
USE_VIZ=true  # 기본값을 true로 변경 (RViz 자동 실행)
USE_CLOCK=true  # 기본값을 true로 변경 (시뮬레이션 시간 사용)
RATE="1.0"

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        --loop)
            USE_LOOP=true
            shift
            ;;
        --viz)
            USE_VIZ=true
            shift
            ;;
        --no-viz)
            USE_VIZ=false
            shift
            ;;
        --clock)
            USE_CLOCK=true
            shift
            ;;
        --rate)
            RATE=$2
            shift 2
            ;;
        *)
            if [ -z "$BAG_FILE" ]; then
                BAG_FILE=$1
            fi
            shift
            ;;
    esac
done

# ROS2 환경 설정
source /opt/ros/jazzy/setup.bash 2>/dev/null

# ========================================
# BAG 파일 선택
# ========================================
ROSBAG_DIR="/home/kimghw/glim/rosbag_data"

if [ -z "$BAG_FILE" ]; then
    echo -e "${CYAN}녹화된 bag 파일 목록:${NC}"
    echo "────────────────────────────────────────"

    # bag 파일 목록 표시
    cd "$ROSBAG_DIR" 2>/dev/null || cd .

    # 디렉토리 목록 가져오기
    BAGS=()
    INDEX=1

    for dir in */; do
        if [ -d "$dir" ]; then
            # 메타데이터 또는 mcap 파일 확인
            if ls "$dir"/*.mcap 1> /dev/null 2>&1 || ls "$dir"/*.db3 1> /dev/null 2>&1; then
                SIZE=$(du -sh "$dir" | cut -f1)
                MTIME=$(stat -c "%y" "$dir" | cut -d' ' -f1,2 | cut -d'.' -f1)
                echo -e "  ${GREEN}[$INDEX]${NC} ${dir%/} (${SIZE}) - ${MTIME}"
                BAGS+=("$dir")
                INDEX=$((INDEX + 1))
            fi
        fi
    done

    if [ ${#BAGS[@]} -eq 0 ]; then
        echo -e "${RED}녹화된 bag 파일이 없습니다.${NC}"
        exit 1
    fi

    echo ""
    echo -n -e "${YELLOW}재생할 파일 번호를 선택하세요: ${NC}"
    read -r SELECTION

    if [[ "$SELECTION" =~ ^[0-9]+$ ]] && [ "$SELECTION" -ge 1 ] && [ "$SELECTION" -le ${#BAGS[@]} ]; then
        BAG_FILE="$ROSBAG_DIR/${BAGS[$((SELECTION-1))]}"
    else
        echo -e "${RED}잘못된 선택입니다.${NC}"
        exit 1
    fi
fi

# BAG 파일 존재 확인
if [ ! -d "$BAG_FILE" ] && [ ! -f "$BAG_FILE" ]; then
    echo -e "${RED}오류: BAG 파일을 찾을 수 없습니다: $BAG_FILE${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}재생 설정:${NC}"
echo "  파일 경로 : $BAG_FILE"
echo "  재생 속도 : ${RATE}x"
echo "  반복 재생 : $([ "$USE_LOOP" = true ] && echo "✓ 활성화" || echo "○ 비활성화")"
echo "  시각화    : $([ "$USE_VIZ" = true ] && echo "✓ RViz 자동 실행" || echo "○ RViz 없음")"
echo "  Clock     : $([ "$USE_CLOCK" = true ] && echo "✓ 시뮬레이션 시간 사용" || echo "○ 시스템 시간 사용")"
echo ""

# ========================================
# 기존 프로세스 확인
# ========================================
echo -e "${YELLOW}기존 재생 프로세스 확인 중...${NC}"

# 기존 bag play 프로세스 종료
if pgrep -f "ros2 bag play" > /dev/null; then
    echo -e "${YELLOW}→ 기존 재생을 중지합니다...${NC}"
    pkill -f "ros2 bag play" 2>/dev/null || true
    sleep 1
fi

# ========================================
# BAG 정보 표시
# ========================================
echo -e "${CYAN}Bag 파일 정보:${NC}"
echo "────────────────────────────────────────"
ros2 bag info "$BAG_FILE" 2>/dev/null | head -15 || echo "정보를 가져올 수 없습니다."
echo ""

# ========================================
# RViz 실행 (옵션)
# ========================================
if [ "$USE_VIZ" = true ]; then
    echo -e "${YELLOW}RViz를 시작합니다...${NC}"

    # RViz 설정 파일 확인 - 우선순위: 1) 로컬 설정, 2) 시스템 설정
    RVIZ_CONFIG="/home/kimghw/glim/ouster_setup/config/ouster_replay.rviz"
    if [ ! -f "$RVIZ_CONFIG" ]; then
        RVIZ_CONFIG="/opt/ros/jazzy/share/ouster_ros/config/viz.rviz"
    fi
    if [ ! -f "$RVIZ_CONFIG" ]; then
        RVIZ_CONFIG=""
    fi

    if [ -n "$RVIZ_CONFIG" ]; then
        rviz2 -d "$RVIZ_CONFIG" > /dev/null 2>&1 &
    else
        rviz2 > /dev/null 2>&1 &
    fi

    RVIZ_PID=$!
    echo -e "${GREEN}✓ RViz 실행 (PID: $RVIZ_PID)${NC}"
    sleep 2
fi

# ========================================
# BAG 재생 시작
# ========================================
echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║              재생 시작!                        ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════╝${NC}"
echo ""

# 재생 명령 생성
PLAY_CMD="ros2 bag play '$BAG_FILE'"

# 옵션 추가
[ "$USE_LOOP" = true ] && PLAY_CMD="$PLAY_CMD --loop"
[ "$USE_CLOCK" = true ] && PLAY_CMD="$PLAY_CMD --clock"
PLAY_CMD="$PLAY_CMD --rate $RATE"

# 추가 옵션: 시작 지연 및 타임스탬프 오프셋
PLAY_CMD="$PLAY_CMD --delay 1.0 --start-offset 0"

echo -e "${CYAN}실행 명령: $PLAY_CMD${NC}"
echo ""
echo -e "${YELLOW}제어 방법:${NC}"
echo "  SPACE     : 일시정지/재개"
echo "  →         : 다음 메시지"
echo "  ↑         : 속도 10% 증가"
echo "  ↓         : 속도 10% 감소"
echo "  Ctrl+C    : 종료"
echo ""
echo -e "${GREEN}1초 후 재생이 시작됩니다...${NC}"
echo ""

# 트랩 설정 (Ctrl+C 처리)
cleanup() {
    echo ""
    echo -e "${YELLOW}재생을 중지합니다...${NC}"

    if [ -n "$RVIZ_PID" ] && [ "$USE_VIZ" = true ]; then
        kill $RVIZ_PID 2>/dev/null || true
        echo -e "${GREEN}✓ RViz 종료${NC}"
    fi

    echo ""
    echo -e "${GREEN}╔════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║            재생 완료!                          ║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════════════╝${NC}"
    echo ""
    exit 0
}

trap cleanup SIGINT SIGTERM

# 재생 실행
eval $PLAY_CMD

# 정상 종료
cleanup