#!/bin/bash

# Start EKF filter node for Microstrain IMU
source /opt/ros/jazzy/setup.bash

ros2 run robot_localization ekf_node --ros-args --params-file /home/kimghw/glim/microstrain_setup/ekf_config.yaml
