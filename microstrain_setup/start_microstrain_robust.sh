#!/bin/bash

# Robust Microstrain IMU driver starter with automatic reconnection

echo "=== Robust IMU Driver Starter ==="

while true; do
    echo "[$(date)] Checking IMU device..."

    # Wait for device to appear
    while [ ! -e /dev/ttyACM0 ]; do
        echo "[$(date)] Waiting for IMU device at /dev/ttyACM0..."
        sleep 2
    done

    echo "[$(date)] IMU device found at /dev/ttyACM0"

    # Kill any existing processes
    echo "[$(date)] Cleaning up old processes..."
    pkill -9 -f "microstrain_inertial_driver_node" 2>/dev/null

    # Kill any process using the port
    PORT_PID=$(lsof -t /dev/ttyACM0 2>/dev/null)
    if [ ! -z "$PORT_PID" ]; then
        echo "[$(date)] Killing process using port: $PORT_PID"
        kill -9 $PORT_PID 2>/dev/null
        sleep 1
    fi

    # Source ROS2 environment
    source /opt/ros/jazzy/setup.bash
    source /home/kimghw/microstrain_ws/install/setup.bash

    echo "[$(date)] Starting IMU driver..."

    # Start the driver with a timeout
    # If it hangs, it will be killed after 60 seconds of no output
    timeout --foreground 60 ros2 launch microstrain_inertial_driver microstrain_launch.py \
        params_file:=/home/kimghw/glim/microstrain_setup/microstrain_config.yml

    EXIT_CODE=$?

    if [ $EXIT_CODE -eq 124 ]; then
        echo "[$(date)] Driver timed out (no response for 60s)"
    else
        echo "[$(date)] Driver exited with code: $EXIT_CODE"
    fi

    # Force kill if still running
    pkill -9 -f "microstrain_inertial_driver_node" 2>/dev/null

    echo "[$(date)] Waiting 3 seconds before retry..."
    sleep 3
done