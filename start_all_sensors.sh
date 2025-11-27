#!/bin/bash
# ============================================================
# 센서 통합 실행 스크립트 (tmux 사용)
# - Ouster 라이다 드라이버
# - Microstrain IMU 드라이버
# - 웹 서버
# - 토픽 모니터링 (추가됨)
# 모두 포그라운드에서 실행하며 로그 확인 가능
# ============================================================

SESSION_NAME="sensors"

# 색상 정의
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo "============================================================"
echo "  센서 통합 실행 스크립트"
echo "============================================================"
echo ""

# tmux가 설치되어 있는지 확인
if ! command -v tmux &> /dev/null; then
    echo -e "${RED}❌ tmux가 설치되어 있지 않습니다.${NC}"
    echo ""
    echo "설치 방법:"
    echo "  sudo apt install tmux"
    echo ""
    exit 1
fi

# 이미 세션이 존재하는지 확인
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    echo -e "${YELLOW}⚠️  '$SESSION_NAME' 세션이 이미 실행 중입니다.${NC}"
    echo ""
    echo "선택:"
    echo "  1) 기존 세션에 연결"
    echo "  2) 기존 세션 종료 후 새로 시작"
    echo "  3) 취소"
    echo ""
    read -p "선택 (1-3): " choice

    case $choice in
        1)
            echo -e "${GREEN}✓ 기존 세션에 연결합니다...${NC}"
            tmux attach -t $SESSION_NAME
            exit 0
            ;;
        2)
            echo -e "${YELLOW}⚠️  기존 세션을 종료합니다...${NC}"
            tmux kill-session -t $SESSION_NAME
            ;;
        3)
            echo "취소되었습니다."
            exit 0
            ;;
        *)
            echo -e "${RED}잘못된 선택입니다.${NC}"
            exit 1
            ;;
    esac
fi

echo -e "${GREEN}🚀 센서 시스템을 시작합니다...${NC}"
echo ""

# 경로 확인
GLIM_DIR="/home/kimghw/glim"
OUSTER_SCRIPT="$GLIM_DIR/ouster_setup/scripts/run_driver.sh"
IMU_SCRIPT="$GLIM_DIR/microstrain_setup/start_microstrain.sh"
WEB_SCRIPT="$GLIM_DIR/web/start_web.sh"

# 스크립트 존재 여부 확인
if [ ! -f "$OUSTER_SCRIPT" ]; then
    echo -e "${RED}❌ Ouster 스크립트를 찾을 수 없습니다: $OUSTER_SCRIPT${NC}"
    exit 1
fi

if [ ! -f "$IMU_SCRIPT" ]; then
    echo -e "${RED}❌ IMU 스크립트를 찾을 수 없습니다: $IMU_SCRIPT${NC}"
    exit 1
fi

if [ ! -f "$WEB_SCRIPT" ]; then
    echo -e "${RED}❌ 웹 서버 스크립트를 찾을 수 없습니다: $WEB_SCRIPT${NC}"
    exit 1
fi

# tmux 세션 생성 및 레이아웃 설정
echo "  [1/4] tmux 세션 생성..."

# 첫 번째 창: Ouster 드라이버
tmux new-session -d -s $SESSION_NAME -n "Ouster" "cd $GLIM_DIR/ouster_setup/scripts && bash -c './run_driver.sh; echo 종료됨. Enter를 눌러 세션을 유지하거나 Ctrl+C로 종료하세요.; read'"

echo "  [2/4] Ouster 드라이버 창 생성 완료"

# 두 번째 창: IMU 드라이버
tmux new-window -t $SESSION_NAME -n "IMU" "cd $GLIM_DIR/microstrain_setup && bash -c 'while true; do ./start_microstrain.sh; echo \"[$(date)] IMU 드라이버 종료. 5초 후 재시작...\"; sleep 5; done'"

echo "  [3/4] IMU 드라이버 창 생성 완료"

# 세 번째 창: 웹 서버
tmux new-window -t $SESSION_NAME -n "WebServer" "cd $GLIM_DIR/web && bash -c './start_web.sh; echo 종료됨. Enter를 눌러 세션을 유지하거나 Ctrl+C로 종료하세요.; read'"

echo "  [4/4] 웹 서버 창 생성 완료"

# 네 번째 창: 토픽 모니터링 (발행 여부 확인용)
# 대시보드가 구독 여부로만 상태를 판단하는 문제를 보완하기 위해, 실제 토픽 발행(Publication) 여부를 확인하는 창을 추가합니다.
# rostopic list 대신 주요 센서 토픽의 hz를 체크하여 실제 데이터 유입을 확인합니다.
tmux new-window -t $SESSION_NAME -n "Monitor" "bash -c 'echo \"ROS 토픽 발행 상태(Hz)를 모니터링합니다...\"; sleep 5; rostopic hz /ouster/points /imu/data'"
echo "  [+] 토픽 모니터링 창 생성 완료"

echo ""

# 첫 번째 창으로 이동
tmux select-window -t $SESSION_NAME:0

echo -e "${GREEN}✅ 모든 센서가 시작되었습니다!${NC}"
echo ""
echo "============================================================"
echo "  tmux 사용법"
echo "============================================================"
echo "  창 전환:"
echo "    Ctrl+B, 0    → Ouster 드라이버"
echo "    Ctrl+B, 1    → IMU 드라이버"
echo "    Ctrl+B, 2    → 웹 서버"
echo "    Ctrl+B, 3    → 토픽 모니터링 (Monitor)"
echo "    Ctrl+B, n    → 다음 창"
echo "    Ctrl+B, p    → 이전 창"
echo ""
echo "  세션 제어:"
echo "    Ctrl+B, d    → 세션 detach (백그라운드로)"
echo "    tmux attach -t sensors    → 다시 연결"
echo ""
echo "  종료:"
echo "    각 창에서 Ctrl+C    → 개별 프로세스 종료"
echo "    tmux kill-session -t sensors    → 전체 종료"
echo "============================================================"
echo ""
echo -e "${YELLOW}3초 후 tmux 세션에 연결합니다...${NC}"
sleep 3

# tmux 세션에 attach
tmux attach -t $SESSION_NAME
