#!/bin/bash

# Display IMU data from rosbag file with interactive selection

echo "==================================="
echo "  IMU Data Display from ROS Bag"
echo "==================================="

# Default topic
DEFAULT_TOPIC="/microstrain/imu/data"

# Check if bag file is provided as argument
if [ "$#" -eq 0 ]; then
    # List available bag files
    echo -e "\nAvailable bag files:"
    echo "-----------------------------------"

    # Find all .bag files in current directory and common locations
    echo "Searching for bag files..."
    mapfile -t bag_files < <(find . ~/Downloads ~/lidarslamlecture/data ~/Kalibr/data -maxdepth 3 -name "*.bag" -type f 2>/dev/null | sort)

    if [ ${#bag_files[@]} -eq 0 ]; then
        echo "No bag files found!"
        echo "Searched in: current directory, ~/Downloads, ~/lidarslamlecture/data, ~/Kalibr/data"
        exit 1
    fi

    # Display files with numbers
    for i in "${!bag_files[@]}"; do
        size=$(du -h "${bag_files[$i]}" | cut -f1)
        echo "$((i+1)). ${bag_files[$i]} (${size})"
    done

    echo -e "\nEnter the number of the bag file to display:"
    read -r selection

    # Validate selection
    if ! [[ "$selection" =~ ^[0-9]+$ ]] || [ "$selection" -lt 1 ] || [ "$selection" -gt "${#bag_files[@]}" ]; then
        echo "Invalid selection!"
        exit 1
    fi

    BAG_FILE="${bag_files[$((selection-1))]}"
else
    BAG_FILE="$1"
fi

# Check if file exists
if [ ! -f "$BAG_FILE" ]; then
    echo "Error: Bag file '$BAG_FILE' not found!"
    exit 1
fi

echo -e "\nSelected: $BAG_FILE"

# Ask for custom topic or use default
echo -e "\nEnter IMU topic name (press Enter for default: $DEFAULT_TOPIC):"
read -r topic_input

if [ -z "$topic_input" ]; then
    TOPIC="$DEFAULT_TOPIC"
else
    TOPIC="$topic_input"
fi

echo -e "\nLoading IMU data from topic: $TOPIC"
echo "Please wait..."

# Check which script to use based on available modules and bag format
if python3 -c "import bagpy" 2>/dev/null; then
    # Use bagpy script (works with ROS 1 bags)
    echo "Using bagpy to read bag file..."
    python3 /home/kimghw/glim/display_imu_bagpy.py "$BAG_FILE" --topic "$TOPIC"
elif python3 -c "import rosbag" 2>/dev/null; then
    # Use ROS 1 script if rosbag module is available
    python3 /home/kimghw/glim/display_imu_from_bag.py "$BAG_FILE" --topic "$TOPIC"
else
    # Use ROS 2 script with ros2 bag commands
    echo "Using ROS 2 bag player (will capture data for 30 seconds)..."
    python3 /home/kimghw/glim/display_imu_ros2.py "$BAG_FILE" --topic "$TOPIC" --duration 30
fi

echo -e "\nVisualization complete!"