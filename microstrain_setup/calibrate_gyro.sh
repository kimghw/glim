#!/bin/bash

echo "========================================="
echo "Microstrain Gyro Calibration Monitor"
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

echo "Driver is running."
echo "IMPORTANT: Keep the IMU completely STILL during calibration!"
echo ""
echo "Monitoring gyro bias estimation status..."
echo "Press Ctrl+C to stop monitoring"
echo ""
echo "========================================="

# EKF 상태 토픽 모니터링
ros2 topic echo /ekf/status --field filter_state
