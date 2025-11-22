#!/bin/bash

#############################################
# GLIM 센서 통합 시작 스크립트 (tmux)
#############################################

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 옵션 파싱
AUTO_CALIBRATE=false
if [[ "$1" == "--auto-calibrate" ]] || [[ "$1" == "-c" ]]; then
    AUTO_CALIBRATE=true
fi

echo "========================================="
echo "GLIM Sensor System Startup"
echo "========================================="
echo ""

# 기존 세션 종료 확인
if tmux has-session -t glim_sensors 2>/dev/null; then
    echo "Existing 'glim_sensors' tmux session found."
    read -p "Kill existing session and restart? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        tmux kill-session -t glim_sensors
        echo "Killed existing session."
    else
        echo "Aborted. Attach to existing session with:"
        echo "  tmux attach -t glim_sensors"
        exit 0
    fi
fi

echo "Creating tmux session 'glim_sensors'..."
echo ""

# 메인 tmux 세션 생성 (첫 번째 window: Microstrain IMU)
tmux new-session -d -s glim_sensors -n microstrain

# Window 0: Microstrain IMU
tmux send-keys -t glim_sensors:0 "cd $SCRIPT_DIR/microstrain_setup" C-m
tmux send-keys -t glim_sensors:0 "echo '=== Starting Microstrain IMU Driver ==='" C-m
tmux send-keys -t glim_sensors:0 "./start_microstrain.sh" C-m

echo "✓ Window 0: Microstrain IMU driver starting..."

# 드라이버 초기화 대기
echo "  Waiting for IMU driver to initialize (5 seconds)..."
sleep 5

# Window 1: Ouster LiDAR
tmux new-window -t glim_sensors:1 -n ouster
tmux send-keys -t glim_sensors:1 "cd $SCRIPT_DIR/ouster_setup" C-m
tmux send-keys -t glim_sensors:1 "echo '=== Starting Ouster LiDAR Driver ==='" C-m
tmux send-keys -t glim_sensors:1 "# Ouster driver command here (if available)" C-m
tmux send-keys -t glim_sensors:1 "# ./start_ouster.sh" C-m

echo "✓ Window 1: Ouster LiDAR driver (ready, not started)"

# Window 2: Web Server
tmux new-window -t glim_sensors:2 -n webserver
tmux send-keys -t glim_sensors:2 "cd $SCRIPT_DIR/web" C-m
tmux send-keys -t glim_sensors:2 "source venv/bin/activate" C-m
tmux send-keys -t glim_sensors:2 "echo '=== Starting Web Server ==='" C-m
tmux send-keys -t glim_sensors:2 "python3 app.py" C-m

echo "✓ Window 2: Web Server starting..."

# 웹서버 초기화 대기
echo "  Waiting for web server to initialize (3 seconds)..."
sleep 3

# Window 3: Gyro Bias Calibration
tmux new-window -t glim_sensors:3 -n gyro_calib
tmux send-keys -t glim_sensors:3 "cd $SCRIPT_DIR/microstrain_setup" C-m
tmux send-keys -t glim_sensors:3 "echo '=== IMU Gyro Bias Calibration ==='" C-m

if [ "$AUTO_CALIBRATE" = true ]; then
    # 자동 캘리브레이션
    tmux send-keys -t glim_sensors:3 "echo 'AUTO-CALIBRATION MODE: Starting in 2 seconds...'" C-m
    tmux send-keys -t glim_sensors:3 "echo 'IMPORTANT: Make sure IMU is completely STILL!'" C-m
    tmux send-keys -t glim_sensors:3 "sleep 2 && ./calibrate_gyro.sh" C-m
    echo "✓ Window 3: Gyro calibration auto-starting..."
else
    # 수동 캘리브레이션 (기본)
    tmux send-keys -t glim_sensors:3 "echo 'IMPORTANT: Keep the IMU completely STILL!'" C-m
    tmux send-keys -t glim_sensors:3 "echo 'Press Enter to start calibration, or Ctrl+C to skip'" C-m
    tmux send-keys -t glim_sensors:3 "read -p 'Start gyro bias calibration? ' && ./calibrate_gyro.sh"
    echo "✓ Window 3: Gyro calibration ready (press Enter to execute)"
fi

# Window 4: Control/Monitor
tmux new-window -t glim_sensors:4 -n control
tmux send-keys -t glim_sensors:4 "cd $SCRIPT_DIR" C-m
tmux send-keys -t glim_sensors:4 "echo '=== GLIM Sensor Control Panel ==='" C-m
tmux send-keys -t glim_sensors:4 "echo ''" C-m
tmux send-keys -t glim_sensors:4 "echo 'Quick Commands:'" C-m
tmux send-keys -t glim_sensors:4 "echo '  ros2 topic list            - List all topics'" C-m
tmux send-keys -t glim_sensors:4 "echo '  ros2 topic hz /imu/data    - Check IMU frequency'" C-m
tmux send-keys -t glim_sensors:4 "echo '  ros2 node list             - List all nodes'" C-m
tmux send-keys -t glim_sensors:4 "echo '  curl http://localhost:5001/health - Check web server'" C-m
tmux send-keys -t glim_sensors:4 "echo ''" C-m
tmux send-keys -t glim_sensors:4 "echo 'Tmux Navigation:'" C-m
tmux send-keys -t glim_sensors:4 "echo '  Ctrl+B, 0  - Microstrain IMU'" C-m
tmux send-keys -t glim_sensors:4 "echo '  Ctrl+B, 1  - Ouster LiDAR'" C-m
tmux send-keys -t glim_sensors:4 "echo '  Ctrl+B, 2  - Web Server'" C-m
tmux send-keys -t glim_sensors:4 "echo '  Ctrl+B, 3  - Gyro Calibration'" C-m
tmux send-keys -t glim_sensors:4 "echo '  Ctrl+B, 4  - Control Panel (this window)'" C-m
tmux send-keys -t glim_sensors:4 "echo '  Ctrl+B, D  - Detach from session'" C-m
tmux send-keys -t glim_sensors:4 "echo ''" C-m

echo "✓ Window 4: Control panel ready"

# 첫 번째 window로 이동 (Microstrain)
tmux select-window -t glim_sensors:0

echo ""
echo "========================================="
echo "✓ All sensors started successfully!"
echo "========================================="
echo ""
echo "Tmux session 'glim_sensors' is running with:"
echo "  Window 0: Microstrain IMU driver"
echo "  Window 1: Ouster LiDAR driver"
echo "  Window 2: Web Server (ROS2 node)"
if [ "$AUTO_CALIBRATE" = true ]; then
    echo "  Window 3: Gyro Calibration (auto-running)"
else
    echo "  Window 3: Gyro Calibration (manual, press Enter)"
fi
echo "  Window 4: Control Panel"
echo ""
echo "To attach to the session:"
echo "  tmux attach -t glim_sensors"
echo ""
echo "To detach (while inside):"
echo "  Ctrl+B, D"
echo ""
echo "To switch windows (while inside):"
echo "  Ctrl+B, 0-4"
echo ""
echo "To kill the session:"
echo "  tmux kill-session -t glim_sensors"
echo ""
echo "Usage:"
echo "  $0              - Manual gyro calibration (default)"
echo "  $0 --auto-calibrate  - Auto gyro calibration on startup"
echo "  $0 -c           - Short form of --auto-calibrate"
echo ""
echo "========================================="
