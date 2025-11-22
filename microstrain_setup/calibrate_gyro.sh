#!/bin/bash

echo "========================================="
echo "Microstrain Gyro Bias Calibration"
echo "========================================="
echo ""

# 드라이버 실행 여부 확인
if ! pgrep -f "microstrain_inertial_driver_node" > /dev/null; then
    echo "ERROR: Microstrain driver is not running."
    echo "Please start the driver first:"
    echo "  /home/kimghw/glim/microstrain_setup/start_microstrain.sh"
    exit 1
fi

echo "Driver is running."
echo ""
echo "IMPORTANT: Keep the IMU completely STILL during calibration!"
echo "Starting gyro bias calibration..."
echo "This may take a few seconds..."
echo ""

# 새로운 bash 세션에서 ROS2 환경을 초기화하고 서비스 호출
RESULT=$(bash -c '
source /opt/ros/jazzy/setup.bash
source /home/kimghw/microstrain_ws/install/setup.bash
timeout 30 ros2 service call /mip/three_dm/capture_gyro_bias microstrain_inertial_msgs/srv/Mip3dmCaptureGyroBias
' 2>&1)
EXIT_CODE=$?

echo ""
if [ $EXIT_CODE -eq 124 ]; then
    echo "ERROR: Service call timed out after 30 seconds."
    echo "The calibration may be taking longer than expected or the service is not responding."
    echo "========================================="
    exit 1
elif [ $EXIT_CODE -ne 0 ]; then
    echo "ERROR: Service call failed."
    echo "$RESULT"
    echo "========================================="
    exit 1
else
    echo "Calibration Result:"
    echo "$RESULT"
    echo ""
    if echo "$RESULT" | grep -q "bias=array"; then
        echo "✓ Gyro bias calibration completed successfully!"
        # Extract bias values
        BIAS=$(echo "$RESULT" | grep -oP "bias=array\(\[.*?\]" | grep -oP "\[.*\]")
        echo "Captured gyro bias values: $BIAS"
        echo ""
        echo "Saving gyro bias to non-volatile memory..."

        # Save settings to non-volatile memory
        SAVE_RESULT=$(bash -c '
source /opt/ros/jazzy/setup.bash
source /home/kimghw/microstrain_ws/install/setup.bash
timeout 5 ros2 service call /mip/three_dm/device_settings/save std_srvs/srv/Empty
' 2>&1)

        if [ $? -eq 0 ]; then
            echo "✓ Gyro bias saved to device memory!"
            echo "The bias will persist after device restart."
        else
            echo "⚠ Warning: Could not save settings to device."
            echo "The bias is applied but will be lost on restart."
        fi
    else
        echo "✗ Gyro bias calibration failed."
        echo "Check the error message above."
    fi
fi
echo "========================================="
