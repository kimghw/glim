#!/bin/bash

# Microstrain IMU 드라이버 실행 스크립트

# 시리얼 포트 권한 설정 (필요시 비밀번호 입력)
sudo chmod 666 /dev/ttyACM0

# ROS2 환경 소싱
source /opt/ros/jazzy/setup.bash
source /home/kimghw/microstrain_ws/install/setup.bash

# Microstrain IMU 드라이버 실행
ros2 launch microstrain_inertial_driver microstrain_launch.py params_file:=/home/kimghw/glim/microstrain_setup/microstrain_config.yml
