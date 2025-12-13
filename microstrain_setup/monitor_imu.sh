#!/bin/bash

# IMU 연결 모니터링 및 자동 재시작 스크립트

echo "IMU Connection Monitor Started"
echo "Monitoring /dev/ttyACM0..."

LAST_STATE="disconnected"

while true; do
    if [ -e /dev/ttyACM0 ]; then
        # Device exists
        if [ "$LAST_STATE" = "disconnected" ]; then
            echo "[$(date)] IMU device detected at /dev/ttyACM0"

            # Check if driver is running
            if ! pgrep -f "microstrain_inertial_driver_node" > /dev/null; then
                echo "[$(date)] Starting IMU driver..."

                # Kill any stuck processes
                pkill -9 -f "microstrain_inertial_driver_node" 2>/dev/null
                sleep 1

                # Find IMU tmux window and restart
                if tmux list-windows -t sensors 2>/dev/null | grep -q "IMU"; then
                    tmux send-keys -t sensors:IMU C-c 2>/dev/null
                    sleep 1
                    tmux send-keys -t sensors:IMU "./start_microstrain.sh" Enter
                    echo "[$(date)] IMU driver restart command sent"
                fi
            fi
            LAST_STATE="connected"
        fi
    else
        # Device not exists
        if [ "$LAST_STATE" = "connected" ]; then
            echo "[$(date)] IMU device disconnected"
            LAST_STATE="disconnected"
        fi
    fi

    sleep 2
done