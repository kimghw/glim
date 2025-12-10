#!/bin/bash

# ANSI color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored text
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Check if rosbag_data directory exists
ROSBAG_DIR="/home/kimghw/glim/rosbag_data"
if [ ! -d "$ROSBAG_DIR" ]; then
    print_error "Rosbag directory not found: $ROSBAG_DIR"
    exit 1
fi

# Find all rosbag directories
print_info "Searching for rosbag files..."
rosbag_dirs=()
while IFS= read -r dir; do
    if [ -d "$dir" ]; then
        # Check if directory contains mcap or db3 files
        if ls "$dir"/*.mcap 2>/dev/null 1>&2 || ls "$dir"/*.db3 2>/dev/null 1>&2; then
            rosbag_dirs+=("$dir")
        fi
    fi
done < <(find "$ROSBAG_DIR" -maxdepth 1 -type d | sort -r)

# Check if any rosbags found
if [ ${#rosbag_dirs[@]} -eq 0 ]; then
    print_error "No rosbag files found in $ROSBAG_DIR"
    exit 1
fi

# Display menu
echo ""
echo "======================================"
echo "        ROS2 Bag Player"
echo "======================================"
echo ""
print_info "Available rosbag files:"
echo ""

for i in "${!rosbag_dirs[@]}"; do
    dir="${rosbag_dirs[$i]}"
    basename_dir=$(basename "$dir")

    # Get bag info
    if [ "$basename_dir" != "rosbag_data" ]; then
        # Get duration and size
        info=$(ros2 bag info "$dir" 2>/dev/null | head -n 5)
        size=$(echo "$info" | grep "Bag size:" | awk '{print $3, $4}')
        duration=$(echo "$info" | grep "Duration:" | awk '{print $2}' | cut -d's' -f1)

        if [ -n "$duration" ]; then
            # Convert duration to minutes and seconds
            duration_sec=${duration%.*}
            if [ -n "$duration_sec" ] && [ "$duration_sec" -gt 0 ]; then
                minutes=$((duration_sec / 60))
                seconds=$((duration_sec % 60))
                duration_str="${minutes}m ${seconds}s"
            else
                duration_str="N/A"
            fi
        else
            duration_str="N/A"
        fi

        echo "  $((i+1)). $basename_dir"
        echo "      Size: $size | Duration: $duration_str"
    fi
done

echo ""
echo "  0. Exit"
echo ""
echo "======================================"
echo ""

# Get user selection
read -p "Select rosbag number to play: " selection

# Validate selection
if [ "$selection" == "0" ]; then
    print_info "Exiting..."
    exit 0
fi

if ! [[ "$selection" =~ ^[0-9]+$ ]] || [ "$selection" -lt 1 ] || [ "$selection" -gt ${#rosbag_dirs[@]} ]; then
    print_error "Invalid selection"
    exit 1
fi

# Get selected rosbag
selected_bag="${rosbag_dirs[$((selection-1))]}"
print_success "Selected: $(basename "$selected_bag")"

# Display playback options
echo ""
echo "======================================"
echo "       Playback Options"
echo "======================================"
echo ""
echo "  1. Normal playback"
echo "  2. Loop playback"
echo "  3. Slow motion (0.5x)"
echo "  4. Fast forward (2x)"
echo "  5. Custom rate"
echo "  6. Play with rviz2"
echo ""
read -p "Select playback option [1-6]: " play_option

# Build play command
play_cmd="ros2 bag play \"$selected_bag\""

case $play_option in
    1)
        print_info "Playing at normal speed..."
        ;;
    2)
        print_info "Playing in loop mode..."
        play_cmd="$play_cmd -l"
        ;;
    3)
        print_info "Playing at 0.5x speed..."
        play_cmd="$play_cmd -r 0.5"
        ;;
    4)
        print_info "Playing at 2x speed..."
        play_cmd="$play_cmd -r 2.0"
        ;;
    5)
        read -p "Enter playback rate (e.g., 0.1 for 10% speed): " custom_rate
        if [[ "$custom_rate" =~ ^[0-9]*\.?[0-9]+$ ]]; then
            print_info "Playing at ${custom_rate}x speed..."
            play_cmd="$play_cmd -r $custom_rate"
        else
            print_error "Invalid rate. Using normal speed..."
        fi
        ;;
    6)
        print_info "Launching rviz2 in background..."
        # Check if rviz config exists
        RVIZ_CONFIG="/home/kimghw/glim/rosbag_player.rviz"
        if [ -f "$RVIZ_CONFIG" ]; then
            rviz2 -d "$RVIZ_CONFIG" &
        else
            rviz2 &
        fi
        RVIZ_PID=$!
        print_success "rviz2 launched (PID: $RVIZ_PID)"
        sleep 2
        print_info "Playing at normal speed..."
        ;;
    *)
        print_warning "Invalid option. Using normal playback..."
        ;;
esac

# Show bag information
echo ""
print_info "Bag information:"
ros2 bag info "$selected_bag" | head -n 10
echo ""

# Show controls
echo "======================================"
echo "        Playback Controls"
echo "======================================"
echo ""
echo "  SPACE  : Pause/Resume"
echo "  s      : Step through messages"
echo "  Ctrl+C : Stop playback"
echo ""
echo "======================================"
echo ""

# Execute play command
print_info "Starting playback..."
echo "Command: $play_cmd"
echo ""

# Trap Ctrl+C to clean up
trap cleanup INT

cleanup() {
    echo ""
    print_info "Stopping playback..."

    # Kill rviz2 if it was launched
    if [ -n "$RVIZ_PID" ] && kill -0 $RVIZ_PID 2>/dev/null; then
        print_info "Closing rviz2..."
        kill $RVIZ_PID 2>/dev/null
    fi

    print_success "Playback stopped"
    exit 0
}

# Run the play command
eval $play_cmd

# Clean up if normal exit
cleanup