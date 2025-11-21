#!/bin/bash

# Ouster 웹 대시보드 실행 스크립트

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
RESTART=0

if [[ "$1" == "--restart" ]]; then
    RESTART=1
fi

echo "================================================"
echo "  Ouster 라이다 웹 대시보드 시작"
echo "================================================"
echo ""

if [ $RESTART -eq 1 ]; then
    echo "옵션: 기존 프로세스를 종료하고 웹 서버를 재시작합니다 (--restart)"
    echo ""
fi

# 가상환경 설정
VENV_DIR="$SCRIPT_DIR/venv"

# 가상환경이 없으면 생성
if [ ! -d "$VENV_DIR" ]; then
    echo "[1/3] 가상환경 생성 중..."
    python3 -m venv "$VENV_DIR"
    if [ $? -eq 0 ]; then
        echo "✓ 가상환경 생성 완료"
    else
        echo "✗ 가상환경 생성 실패"
        exit 1
    fi
else
    echo "[1/3] 가상환경 확인 완료"
fi

# 가상환경 활성화
echo "[2/3] 가상환경 활성화 중..."
source "$VENV_DIR/bin/activate"

# 의존성 설치/확인
echo "[3/3] 의존성 확인 중..."
pip install -q flask flask-cors psutil numpy opencv-python pillow

if [ $? -eq 0 ]; then
    echo "✓ 의존성 확인 완료"
else
    echo "✗ 의존성 설치 실패"
    exit 1
fi

echo ""
echo "================================================"
echo "  기존 백그라운드 프로세스 정리"
echo "================================================"

kill_pattern() {
    local pattern="$1"
    local label="$2"
    local pids

    pids=$(pgrep -f "$pattern" | tr '\n' ' ' | sed 's/ *$//')
    if [ -n "$pids" ]; then
        echo "  ${label} 종료 중... (PID: $pids)"
        pkill -f -TERM "$pattern" 2>/dev/null
        sleep 0.5
        # 아직 남아있다면 강제 종료
        if pgrep -f "$pattern" > /dev/null 2>&1; then
            pkill -f -KILL "$pattern" 2>/dev/null
        fi
    fi
}

# 웹 서버 중복 실행 방지 (기본: 재사용)
existing_web_pids=$(pgrep -f "$SCRIPT_DIR/app.py" | xargs 2>/dev/null)
if [ -n "$existing_web_pids" ] && [ $RESTART -eq 0 ]; then
    echo "이미 웹 서버가 실행 중입니다. (PID: $existing_web_pids)"
    echo "다시 시작하려면 --restart 옵션을 사용하세요."
    exit 0
fi

# 백그라운드 프로세스 정리
kill_pattern "$SCRIPT_DIR/app.py" "웹 서버"
kill_pattern "capture_pointcloud.py" "캐پ처 스크립트"
kill_pattern "record_ouster.sh" "녹화 스크립트"
kill_pattern "ros2 bag record" "Rosbag 녹화"
kill_pattern "ros2 bag play" "Rosbag 재생"
kill_pattern "driver.launch.py" "Ouster 드라이버"
kill_pattern "os_driver" "드라이버 (os_driver)"
kill_pattern "ouster_ros" "Ouster ROS"

# ROS2 daemon 종료 (가능한 경우)
if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
fi

# 포트 5001 사용 중인 프로세스 종료
if lsof -i :5001 > /dev/null 2>&1; then
    echo "  포트 5001 사용 프로세스 종료 중..."
    lsof -ti :5001 | xargs -r kill -9 2>/dev/null
fi

sleep 1
echo "✓ 모든 백그라운드 프로세스 정리 완료"
echo ""
echo "================================================"
echo "  웹 서버 시작"
echo "================================================"
echo "  접속 주소: http://localhost:5001"
echo "  종료: Ctrl+C"
echo "================================================"
echo ""

cd "$SCRIPT_DIR"

# ROS2 환경 소싱 (캡처 스크립트에서 sensor_msgs 등을 import하기 위해 필요)
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

python3 app.py
