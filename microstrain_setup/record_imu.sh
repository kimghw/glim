#!/bin/bash

echo "========================================="
echo "Microstrain IMU Data Recording"
echo "========================================="
echo ""

# ROS2 환경 소싱
source /opt/ros/jazzy/setup.bash
source /home/kimghw/microstrain_ws/install/setup.bash

# 드라이버 실행 여부 확인
if ! pgrep -f "microstrain_inertial_driver_node" > /dev/null; then
    echo "ERROR: Microstrain driver is not running."
    echo "Please start the driver first:"
    echo "  /home/kimghw/glim/microstrain_setup/start_microstrain.sh"
    exit 1
fi

# 저장 디렉토리 설정
SAVE_DIR="/home/kimghw/glim/rosbag_data"

# 디렉토리가 없으면 생성
if [ ! -d "$SAVE_DIR" ]; then
    mkdir -p "$SAVE_DIR"
    echo "Created directory: $SAVE_DIR"
fi

# 타임스탬프로 파일명 생성
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
OUTPUT_NAME="imu_${TIMESTAMP}"

echo "Recording IMU data..."
echo "Save location: $SAVE_DIR/$OUTPUT_NAME"
echo ""
echo "Recording topics:"
echo "  - /imu/data"
echo "  - /imu/data_raw"
echo "  - /ekf/status"
echo ""
echo "Press Ctrl+C to stop recording"
echo "========================================="
echo ""

# rosbag 레코딩 시작
cd "$SAVE_DIR"
ros2 bag record -o "$OUTPUT_NAME" /imu/data /imu/data_raw /ekf/status

echo ""
echo "========================================="
echo "Recording stopped."
echo "Data saved to: $SAVE_DIR/$OUTPUT_NAME"
echo "========================================="
