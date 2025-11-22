#!/bin/bash

# Microstrain IMU 드라이버 실행 스크립트

# 기존 Microstrain 드라이버 프로세스 종료
echo "Checking for existing Microstrain processes..."
pkill -9 -f "microstrain_inertial_driver_node"

# 포트 사용 중인 프로세스 종료
if [ -e /dev/ttyACM0 ]; then
    PORT_PID=$(lsof -t /dev/ttyACM0 2>/dev/null)
    if [ ! -z "$PORT_PID" ]; then
        echo "Killing process using /dev/ttyACM0: $PORT_PID"
        kill -9 $PORT_PID 2>/dev/null
        sleep 1
    fi
fi

# ROS2 환경 소싱
source /opt/ros/jazzy/setup.bash
source /home/kimghw/microstrain_ws/install/setup.bash

# Microstrain IMU 드라이버 실행
ros2 launch microstrain_inertial_driver microstrain_launch.py params_file:=/home/kimghw/glim/microstrain_setup/microstrain_config.yml
