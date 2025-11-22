#!/bin/bash

# Web server startup script for rosbag recording management

# Cleanup function
cleanup() {
    echo "Shutting down web server..."
    # Kill Flask server
    pkill -f "python.*web/app.py"
    # Kill any remaining rosbag recording processes
    pkill -f "ros2 bag record"
    # Clean up state file
    rm -f /tmp/rosbag_recorder_state.json
    echo "Cleanup complete"
    exit 0
}

# Trap SIGINT and SIGTERM
trap cleanup SIGINT SIGTERM

echo "Starting rosbag recording web server..."

# Kill existing server if running
pkill -f "python.*web/app.py"
# Kill any orphaned rosbag processes
pkill -f "ros2 bag record"
sleep 1

# Activate virtual environment and start Flask app
cd /home/kimghw/glim/web
source venv/bin/activate
python app.py &
FLASK_PID=$!

echo "Flask server started with PID: $FLASK_PID"
echo "Press Ctrl+C to stop the server"

# Wait for Flask process
wait $FLASK_PID
