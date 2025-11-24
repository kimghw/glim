#!/usr/bin/env python3
"""
Flask web server for managing ROS2 rosbag recording
"""

from flask import Flask, jsonify, request, render_template
import subprocess
import os
import signal
import json
import atexit
import yaml
import threading
from datetime import datetime
from pathlib import Path

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import rosidl_runtime_py.utilities

app = Flask(__name__)

# Load configuration from YAML file
CONFIG_FILE = "/home/kimghw/glim/web/config.yaml"
STATE_FILE = "/tmp/rosbag_recorder_state.json"

def load_config():
    """Load configuration from YAML file"""
    try:
        with open(CONFIG_FILE, 'r') as f:
            config = yaml.safe_load(f)
        return config
    except Exception as e:
        print(f"Error loading config file: {e}")
        # Return default config
        return {
            "rosbag_dir": "/home/kimghw/glim/rosbag_data",
            "topics": ["/imu/data", "/imu/data_raw", "/ekf/status"],
            "recording": {"max_bag_size": 0, "compression": "none"}
        }

# Load configuration
config = load_config()
ROSBAG_DIR = config.get("rosbag_dir", "/home/kimghw/glim/rosbag_data")
TOPICS = config.get("topics", ["/imu/data", "/imu/data_raw", "/ekf/status"])
RECORDING_OPTIONS = config.get("recording", {})

# ROS2 Topic Monitor Node
class TopicMonitor(Node):
    """ROS2 node for monitoring topics in real-time"""

    def __init__(self):
        super().__init__('web_topic_monitor')
        self.latest_messages = {}  # Store latest message for each topic
        self.message_timestamps = {}  # Store timestamp of last message
        self.topic_subscriptions = {}  # Store subscription objects
        self.get_logger().info('Topic Monitor Node initialized')

    def subscribe_to_topic(self, topic_name, msg_type):
        """Subscribe to a topic dynamically"""
        if topic_name in self.topic_subscriptions:
            self.get_logger().warn(f'Already subscribed to {topic_name}')
            return

        try:
            # Create subscription
            sub = self.create_subscription(
                msg_type,
                topic_name,
                lambda msg, topic=topic_name: self._topic_callback(topic, msg),
                10
            )
            self.topic_subscriptions[topic_name] = sub
            self.get_logger().info(f'Subscribed to {topic_name}')
        except Exception as e:
            self.get_logger().error(f'Failed to subscribe to {topic_name}: {e}')

    def _topic_callback(self, topic_name, msg):
        """Callback for all subscribed topics"""
        self.latest_messages[topic_name] = msg
        self.message_timestamps[topic_name] = self.get_clock().now()

    def get_latest_message(self, topic_name):
        """Get the latest message from a topic"""
        if topic_name not in self.latest_messages:
            return None

        msg = self.latest_messages[topic_name]
        timestamp = self.message_timestamps[topic_name]

        return {
            'message': self._msg_to_dict(msg),
            'timestamp': timestamp.nanoseconds / 1e9,
            'age_seconds': (self.get_clock().now() - timestamp).nanoseconds / 1e9
        }

    def _msg_to_dict(self, msg):
        """Convert ROS message to dictionary"""
        try:
            # Simple conversion - expand this as needed
            msg_dict = {}
            for field in msg.get_fields_and_field_types():
                value = getattr(msg, field)
                if hasattr(value, 'x') and hasattr(value, 'y') and hasattr(value, 'z'):
                    # Vector3 type
                    msg_dict[field] = {'x': value.x, 'y': value.y, 'z': value.z}
                elif hasattr(value, 'w') and hasattr(value, 'x'):
                    # Quaternion type
                    msg_dict[field] = {'x': value.x, 'y': value.y, 'z': value.z, 'w': value.w}
                elif isinstance(value, (int, float, str, bool)):
                    msg_dict[field] = value
                else:
                    msg_dict[field] = str(value)
            return msg_dict
        except Exception as e:
            return {'error': f'Failed to convert message: {e}', 'raw': str(msg)}

# Global ROS2 node and executor
ros_node = None
ros_executor = None
ros_thread = None

class RosbagRecorder:
    def __init__(self):
        self.state_file = STATE_FILE
        self.rosbag_dir = ROSBAG_DIR

    def get_state(self):
        """Get current recording state"""
        if not os.path.exists(self.state_file):
            return {"recording": False, "pid": None, "bag_name": None, "start_time": None}

        try:
            with open(self.state_file, 'r') as f:
                state = json.load(f)

            # Check if process is still running
            if state.get("pid"):
                try:
                    os.kill(state["pid"], 0)  # Check if process exists
                except OSError:
                    # Process doesn't exist, clean up state
                    state["recording"] = False
                    state["pid"] = None
                    self._save_state(state)

            return state
        except Exception as e:
            return {"recording": False, "pid": None, "bag_name": None, "start_time": None, "error": str(e)}

    def _save_state(self, state):
        """Save state to file"""
        with open(self.state_file, 'w') as f:
            json.dump(state, f)

    def start_recording(self, bag_name=None):
        """Start rosbag recording"""
        current_state = self.get_state()

        # Check if already recording
        if current_state.get("recording"):
            return {"success": False, "error": "Already recording", "state": current_state}

        # Create rosbag directory if it doesn't exist
        os.makedirs(self.rosbag_dir, exist_ok=True)

        # Generate bag name if not provided
        if not bag_name:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            bag_name = f"rosbag_{timestamp}"

        bag_path = os.path.join(self.rosbag_dir, bag_name)

        # Build ros2 bag record command
        cmd = [
            "bash", "-c",
            f"source /opt/ros/jazzy/setup.bash && source /home/kimghw/microstrain_ws/install/setup.bash && "
            f"ros2 bag record -o {bag_path} {' '.join(TOPICS)}"
        ]

        try:
            # Start recording process in background with new process group
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid  # Create new session to isolate from parent
            )

            # Save state with process group ID
            state = {
                "recording": True,
                "pid": process.pid,
                "pgid": os.getpgid(process.pid),  # Save process group ID
                "bag_name": bag_name,
                "bag_path": bag_path,
                "start_time": datetime.now().isoformat(),
                "topics": TOPICS
            }
            self._save_state(state)

            return {"success": True, "message": "Recording started", "state": state}

        except Exception as e:
            return {"success": False, "error": str(e)}

    def stop_recording(self):
        """Stop rosbag recording"""
        current_state = self.get_state()

        if not current_state.get("recording"):
            return {"success": False, "error": "Not currently recording", "state": current_state}

        pid = current_state.get("pid")
        if not pid:
            return {"success": False, "error": "No PID found in state"}

        try:
            # Send SIGINT to process group to gracefully stop recording
            os.killpg(os.getpgid(pid), signal.SIGINT)

            # Wait a bit for graceful shutdown
            import time
            time.sleep(2)

            # Check if process is still running
            try:
                os.kill(pid, 0)
                # Still running, force kill
                os.killpg(os.getpgid(pid), signal.SIGKILL)
            except OSError:
                # Process already terminated
                pass

            # Update state
            final_state = current_state.copy()
            final_state["recording"] = False
            final_state["stop_time"] = datetime.now().isoformat()
            self._save_state({"recording": False, "pid": None, "bag_name": None, "start_time": None})

            return {"success": True, "message": "Recording stopped", "state": final_state}

        except Exception as e:
            return {"success": False, "error": str(e)}

    def list_recordings(self):
        """List all recorded bags"""
        if not os.path.exists(self.rosbag_dir):
            return []

        bags = []
        for item in os.listdir(self.rosbag_dir):
            item_path = os.path.join(self.rosbag_dir, item)
            if os.path.isdir(item_path):
                # Get directory size and modification time
                size = sum(f.stat().st_size for f in Path(item_path).rglob('*') if f.is_file())
                mtime = os.path.getmtime(item_path)
                bags.append({
                    "name": item,
                    "path": item_path,
                    "size_bytes": size,
                    "size_mb": round(size / (1024 * 1024), 2),
                    "modified": datetime.fromtimestamp(mtime).isoformat()
                })

        return sorted(bags, key=lambda x: x["modified"], reverse=True)

# Create recorder instance
recorder = RosbagRecorder()

# Routes
@app.route('/')
def index():
    """Serve the web interface"""
    return render_template('index.html')

@app.route('/api/status', methods=['GET'])
def status():
    """Get current recording status"""
    state = recorder.get_state()
    return jsonify(state)

@app.route('/api/start', methods=['POST'])
def start():
    """Start recording"""
    data = request.get_json() or {}
    bag_name = data.get('bag_name')
    result = recorder.start_recording(bag_name)
    return jsonify(result), 200 if result.get("success") else 400

@app.route('/api/stop', methods=['POST'])
def stop():
    """Stop recording"""
    result = recorder.stop_recording()
    return jsonify(result), 200 if result.get("success") else 400

@app.route('/api/recordings', methods=['GET'])
def recordings():
    """List all recordings"""
    bags = recorder.list_recordings()
    return jsonify({"recordings": bags, "count": len(bags)})

@app.route('/health', methods=['GET'])
def health():
    """Health check"""
    return jsonify({"status": "ok", "service": "rosbag_recorder"})

@app.route('/api/config', methods=['GET'])
def get_config():
    """Get current configuration"""
    return jsonify({
        "rosbag_dir": ROSBAG_DIR,
        "topics": TOPICS,
        "recording_options": RECORDING_OPTIONS
    })

@app.route('/api/config/reload', methods=['POST'])
def reload_config():
    """Reload configuration from file"""
    global config, ROSBAG_DIR, TOPICS, RECORDING_OPTIONS

    # Check if recording is in progress
    state = recorder.get_state()
    if state.get("recording"):
        return jsonify({
            "success": False,
            "error": "Cannot reload config while recording is in progress"
        }), 400

    try:
        config = load_config()
        ROSBAG_DIR = config.get("rosbag_dir", "/home/kimghw/glim/rosbag_data")
        TOPICS = config.get("topics", ["/imu/data", "/imu/data_raw", "/ekf/status"])
        RECORDING_OPTIONS = config.get("recording", {})

        return jsonify({
            "success": True,
            "message": "Configuration reloaded",
            "config": {
                "rosbag_dir": ROSBAG_DIR,
                "topics": TOPICS,
                "recording_options": RECORDING_OPTIONS
            }
        })
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@app.route('/api/topics', methods=['GET'])
def get_topics():
    """Get current topic list"""
    return jsonify({
        "topics": TOPICS,
        "count": len(TOPICS)
    })

@app.route('/api/topics/add', methods=['POST'])
def add_topic():
    """Add topic to recording list"""
    global TOPICS

    # Check if recording is in progress
    state = recorder.get_state()
    if state.get("recording"):
        return jsonify({
            "success": False,
            "error": "Cannot modify topics while recording is in progress"
        }), 400

    data = request.get_json() or {}
    topic = data.get('topic')

    if not topic:
        return jsonify({
            "success": False,
            "error": "Topic name is required"
        }), 400

    if topic in TOPICS:
        return jsonify({
            "success": False,
            "error": f"Topic '{topic}' already exists"
        }), 400

    TOPICS.append(topic)

    return jsonify({
        "success": True,
        "message": f"Topic '{topic}' added",
        "topics": TOPICS
    })

@app.route('/api/topics/remove', methods=['POST'])
def remove_topic():
    """Remove topic from recording list"""
    global TOPICS

    # Check if recording is in progress
    state = recorder.get_state()
    if state.get("recording"):
        return jsonify({
            "success": False,
            "error": "Cannot modify topics while recording is in progress"
        }), 400

    data = request.get_json() or {}
    topic = data.get('topic')

    if not topic:
        return jsonify({
            "success": False,
            "error": "Topic name is required"
        }), 400

    if topic not in TOPICS:
        return jsonify({
            "success": False,
            "error": f"Topic '{topic}' not found"
        }), 400

    TOPICS.remove(topic)

    return jsonify({
        "success": True,
        "message": f"Topic '{topic}' removed",
        "topics": TOPICS
    })

@app.route('/api/topics/set', methods=['POST'])
def set_topics():
    """Set entire topic list"""
    global TOPICS

    # Check if recording is in progress
    state = recorder.get_state()
    if state.get("recording"):
        return jsonify({
            "success": False,
            "error": "Cannot modify topics while recording is in progress"
        }), 400

    data = request.get_json() or {}
    topics = data.get('topics')

    if not topics or not isinstance(topics, list):
        return jsonify({
            "success": False,
            "error": "Topics must be a list"
        }), 400

    TOPICS = topics

    return jsonify({
        "success": True,
        "message": f"Topics updated ({len(TOPICS)} topics)",
        "topics": TOPICS
    })

@app.route('/api/topics/echo', methods=['POST'])
def echo_topic():
    """Echo the latest message from a subscribed topic (direct subscription)"""
    global ros_node

    data = request.get_json() or {}
    topic = data.get('topic')

    if not topic:
        return jsonify({
            "success": False,
            "error": "Topic name is required"
        }), 400

    if ros_node is None:
        return jsonify({
            "success": False,
            "error": "ROS2 node not initialized"
        }), 500

    try:
        # Get latest message from node
        msg_data = ros_node.get_latest_message(topic)

        if msg_data is None:
            return jsonify({
                "success": False,
                "error": f"No data received on topic '{topic}'. Topic may not be subscribed or no messages published yet."
            }), 404

        return jsonify({
            "success": True,
            "topic": topic,
            "data": msg_data['message'],
            "timestamp": msg_data['timestamp'],
            "age_seconds": msg_data['age_seconds']
        })

    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@app.route('/api/topics/subscribe', methods=['POST'])
def subscribe_topic():
    """Subscribe to a topic for real-time monitoring"""
    global ros_node

    data = request.get_json() or {}
    topic = data.get('topic')
    msg_type_str = data.get('msg_type', 'sensor_msgs/msg/Imu')  # Default to IMU

    if not topic:
        return jsonify({
            "success": False,
            "error": "Topic name is required"
        }), 400

    if ros_node is None:
        return jsonify({
            "success": False,
            "error": "ROS2 node not initialized"
        }), 500

    try:
        # Parse message type
        if msg_type_str == 'sensor_msgs/msg/Imu':
            msg_type = Imu
        else:
            # Try to dynamically load the message type
            return jsonify({
                "success": False,
                "error": f"Message type '{msg_type_str}' not supported yet. Use 'sensor_msgs/msg/Imu'"
            }), 400

        ros_node.subscribe_to_topic(topic, msg_type)

        return jsonify({
            "success": True,
            "message": f"Subscribed to topic '{topic}'",
            "topic": topic,
            "msg_type": msg_type_str
        })

    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@app.route('/api/topics/list_available', methods=['GET'])
def list_available_topics():
    """List all available ROS2 topics"""
    try:
        cmd = [
            "bash", "-c",
            "source /opt/ros/jazzy/setup.bash && source /home/kimghw/microstrain_ws/install/setup.bash && "
            "timeout 2 ros2 topic list"
        ]

        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=3
        )

        if result.returncode == 0:
            topics = [t.strip() for t in result.stdout.strip().split('\n') if t.strip()]
            return jsonify({
                "success": True,
                "topics": topics,
                "count": len(topics)
            })
        else:
            return jsonify({
                "success": False,
                "error": "Failed to list topics"
            }), 500

    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@app.route('/api/drivers/status', methods=['GET'])
def driver_status():
    """Check status of IMU and Ouster drivers"""
    try:
        # Check Microstrain IMU driver
        imu_running = subprocess.run(
            ['pgrep', '-f', 'microstrain_inertial_driver_node'],
            capture_output=True
        ).returncode == 0

        # Check Ouster driver
        ouster_running = subprocess.run(
            ['pgrep', '-f', 'os_driver'],
            capture_output=True
        ).returncode == 0

        # Get list of available topics
        imu_topics_active = False
        ouster_topics_active = False

        try:
            result = subprocess.run(
                ['bash', '-c', 'source /opt/ros/jazzy/setup.bash && source /home/kimghw/microstrain_ws/install/setup.bash && timeout 2 ros2 topic list'],
                capture_output=True,
                text=True,
                timeout=3
            )

            if result.returncode == 0:
                available_topics = result.stdout.strip().split('\n')

                # Check if IMU topics exist
                imu_topics_active = '/imu/data' in available_topics or '/imu/data_raw' in available_topics

                # Check if Ouster topics exist
                ouster_topics_active = '/ouster/points' in available_topics or '/ouster/imu' in available_topics

        except subprocess.TimeoutExpired:
            # If timeout, assume topics are not available
            pass

        return jsonify({
            "success": True,
            "drivers": {
                "imu": {
                    "running": imu_running,
                    "publishing": imu_topics_active,
                    "status": "active" if (imu_running and imu_topics_active) else ("started" if imu_running else "stopped")
                },
                "ouster": {
                    "running": ouster_running,
                    "publishing": ouster_topics_active,
                    "status": "active" if (ouster_running and ouster_topics_active) else ("started" if ouster_running else "stopped")
                }
            }
        })

    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@app.route('/api/imu/calibrate_gyro', methods=['POST'])
def calibrate_gyro():
    """Calibrate IMU gyro bias

    IMPORTANT: IMU must be completely still during calibration!
    """
    try:
        # Check if IMU driver is running
        if not subprocess.run(['pgrep', '-f', 'microstrain_inertial_driver_node'],
                            capture_output=True).returncode == 0:
            return jsonify({
                "success": False,
                "error": "Microstrain IMU driver is not running"
            }), 400

        # Call gyro bias calibration service and wait for completion
        cmd = [
            "bash", "-c",
            "source /opt/ros/jazzy/setup.bash && source /home/kimghw/microstrain_ws/install/setup.bash && "
            "timeout 30 ros2 service call /mip/three_dm/capture_gyro_bias microstrain_inertial_msgs/srv/Mip3dmCaptureGyroBias"
        ]

        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        # Wait for completion and get output
        stdout, stderr = process.communicate()
        returncode = process.returncode

        if returncode == 0:
            # Parse bias values from output
            if "bias=array" in stdout:
                # Extract bias values
                import re
                bias_match = re.search(r'bias=array\(\[(.*?)\]', stdout)
                bias_values = None
                if bias_match:
                    bias_str = bias_match.group(1)
                    bias_values = [float(x.strip()) for x in bias_str.split(',')]

                # Save to non-volatile memory
                save_cmd = [
                    "bash", "-c",
                    "source /opt/ros/jazzy/setup.bash && source /home/kimghw/microstrain_ws/install/setup.bash && "
                    "timeout 5 ros2 service call /mip/three_dm/device_settings/save std_srvs/srv/Empty"
                ]

                save_process = subprocess.Popen(save_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                save_stdout, save_stderr = save_process.communicate()
                saved = save_process.returncode == 0

                return jsonify({
                    "success": True,
                    "message": "Gyro bias calibration completed successfully",
                    "bias": bias_values,
                    "saved_to_device": saved,
                    "raw_output": stdout
                })
            else:
                return jsonify({
                    "success": False,
                    "error": "Calibration failed - no bias values returned",
                    "raw_output": stdout
                }), 500

        elif returncode == 124:
            return jsonify({
                "success": False,
                "error": "Calibration timed out after 30 seconds"
            }), 408
        else:
            return jsonify({
                "success": False,
                "error": stderr or "Calibration service call failed",
                "raw_output": stdout
            }), 500

    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

def cleanup_on_exit():
    """Clean up any running recording processes when server exits"""
    global ros_executor, ros_thread

    print("Cleaning up recording processes...")
    state = recorder.get_state()
    if state.get("recording") and state.get("pid"):
        try:
            recorder.stop_recording()
            print(f"Stopped recording process PID: {state['pid']}")
        except Exception as e:
            print(f"Error during cleanup: {e}")

    # Shutdown ROS2
    print("Shutting down ROS2 node...")
    if ros_executor:
        ros_executor.shutdown()
    if ros_thread and ros_thread.is_alive():
        ros_thread.join(timeout=2)
    if rclpy.ok():
        rclpy.shutdown()

def cleanup_orphaned_processes():
    """Clean up any orphaned rosbag recording processes on startup"""
    print("Checking for orphaned recording processes...")

    # Check state file
    if os.path.exists(STATE_FILE):
        try:
            with open(STATE_FILE, 'r') as f:
                state = json.load(f)

            if state.get("recording") and state.get("pid"):
                pid = state["pid"]
                try:
                    # Check if process exists
                    os.kill(pid, 0)
                    # Process exists, kill it
                    print(f"Found orphaned recording process PID: {pid}, cleaning up...")
                    os.kill(pid, signal.SIGTERM)
                    import time
                    time.sleep(1)
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except:
                        pass
                except OSError:
                    # Process doesn't exist
                    pass

                # Reset state file
                with open(STATE_FILE, 'w') as f:
                    json.dump({"recording": False, "pid": None, "bag_name": None, "start_time": None}, f)
                print("Cleaned up state file")
        except Exception as e:
            print(f"Error during startup cleanup: {e}")

def init_ros2():
    """Initialize ROS2 node and executor"""
    global ros_node, ros_executor, ros_thread

    print("Initializing ROS2 node...")
    rclpy.init()

    ros_node = TopicMonitor()

    # Auto-subscribe to common topics
    ros_node.subscribe_to_topic('/imu/data', Imu)
    ros_node.subscribe_to_topic('/imu/data_raw', Imu)

    # Create executor and spin in separate thread
    ros_executor = MultiThreadedExecutor()
    ros_executor.add_node(ros_node)

    ros_thread = threading.Thread(target=ros_executor.spin, daemon=True)
    ros_thread.start()

    print("ROS2 node initialized and spinning in background")

if __name__ == '__main__':
    # Ensure rosbag directory exists
    os.makedirs(ROSBAG_DIR, exist_ok=True)

    # Clean up any orphaned processes from previous run
    cleanup_orphaned_processes()

    # Initialize ROS2 node
    init_ros2()

    # Register cleanup handler
    atexit.register(cleanup_on_exit)

    # Handle SIGTERM and SIGINT
    def signal_handler(sig, frame):
        print(f"\nReceived signal {sig}, cleaning up...")
        cleanup_on_exit()
        exit(0)

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    # Run Flask app
    try:
        app.run(host='0.0.0.0', port=5001, debug=False)
    finally:
        cleanup_on_exit()
