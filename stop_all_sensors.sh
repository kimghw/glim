#!/bin/bash

#############################################
# GLIM 센서 통합 정지 스크립트
#############################################

echo "========================================="
echo "GLIM Sensor System Shutdown"
echo "========================================="
echo ""

if ! tmux has-session -t glim_sensors 2>/dev/null; then
    echo "No 'glim_sensors' session found."
    echo "Nothing to stop."
    exit 0
fi

echo "Found 'glim_sensors' tmux session."
read -p "Kill the session and stop all sensors? (y/N): " -n 1 -r
echo

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborted."
    exit 0
fi

echo "Stopping all sensors..."

# tmux 세션 종료 (모든 프로세스 자동 종료)
tmux kill-session -t glim_sensors

# 추가 정리: orphan rosbag 프로세스
echo "Cleaning up any orphan rosbag processes..."
pkill -f "ros2 bag record" 2>/dev/null

# 상태 파일 정리
rm -f /tmp/rosbag_recorder_state.json

echo ""
echo "========================================="
echo "✓ All sensors stopped successfully!"
echo "========================================="
