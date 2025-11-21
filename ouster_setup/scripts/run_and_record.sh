#!/bin/bash

# Ouster 드라이버 시작 및 녹화 스크립트

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SETUP_DIR="$(dirname "$SCRIPT_DIR")"

# 파라미터
DURATION=${1:-30}  # 기본 30초
FILENAME=${2:-"recording_$(date +%Y%m%d_%H%M%S)"}

echo "================================================"
echo "  Ouster 녹화 시작"
echo "================================================"
echo "  파일명: $FILENAME"
echo "  시간: ${DURATION}초"
echo "================================================"
echo ""

source /opt/ros/jazzy/setup.bash

# record.launch.xml 사용 (센서 + 녹화)
ros2 launch ouster_ros record.launch.xml \
    sensor_hostname:=192.168.10.10 \
    bag_file:=$FILENAME \
    lidar_mode:=2048x10 \
    viz:=false
