#!/bin/bash

echo "========================================="
echo "Microstrain IMU Driver Status Check"
echo "========================================="
echo ""

# ROS2 환경 소싱
source /opt/ros/jazzy/setup.bash
source /home/kimghw/microstrain_ws/install/setup.bash 2>/dev/null

# 1. 드라이버 프로세스 확인
echo "1. Driver Process Status:"
if pgrep -f "microstrain_inertial_driver_node" > /dev/null; then
    echo "   ✓ Driver is RUNNING"
    echo "   PID: $(pgrep -f microstrain_inertial_driver_node)"
else
    echo "   ✗ Driver is NOT RUNNING"
fi
echo ""

# 2. ROS2 노드 확인
echo "2. ROS2 Nodes:"
ros2 node list 2>/dev/null || echo "   No nodes found"
echo ""

# 3. IMU 토픽 확인
echo "3. IMU Topics:"
ros2 topic list 2>/dev/null | grep -E "(imu|ekf)" || echo "   No IMU topics found"
echo ""

# 4. 토픽 Hz 확인 (5초 샘플링)
echo "4. Topic Publishing Rate (sampling 5 seconds):"
if ros2 topic list 2>/dev/null | grep -q "/imu/data"; then
    timeout 5 ros2 topic hz /imu/data 2>/dev/null | head -n 2 || echo "   No data received"
else
    echo "   /imu/data topic not available"
fi
echo ""

# 5. 시리얼 포트 확인
echo "5. Serial Port Status:"
if [ -e /dev/ttyACM0 ]; then
    echo "   ✓ /dev/ttyACM0 exists"
    ls -l /dev/ttyACM0
else
    echo "   ✗ /dev/ttyACM0 not found"
fi
echo ""

echo "========================================="
