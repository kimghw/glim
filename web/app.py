#!/usr/bin/env python3
"""
Flask web server for managing ROS2 rosbag recording
"""

from flask import Flask, jsonify, request, render_template, Response
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
from sensor_msgs.msg import Imu, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import rosidl_runtime_py.utilities
from cv_bridge import CvBridge
import cv2
import base64
import numpy as np
import time
from collections import deque
import math

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
        self.cv_bridge = CvBridge()  # For converting ROS images to OpenCV

        # IMU path integration variables
        self.imu_path_enabled = False
        self.imu_path_plane = 'xy'  # Default plane
        self.imu_path_duration = 5.0  # seconds
        self.imu_data_buffer = deque(maxlen=500)  # Buffer for IMU data (at 100Hz = 5 seconds)
        self.imu_path = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # Current position (meters)
        self.imu_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # Current velocity (m/s)
        self.imu_path_history = deque(maxlen=500)  # Store path points
        self.last_imu_time = None
        self.imu_bias = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # Bias offsets
        self.imu_noise_std = {'x': 0.05, 'y': 0.05, 'z': 0.05}  # Default noise std
        self.imu_calibrating = False
        self.calibration_samples = []

        # Accumulation buffer for fixed-rate integration
        self.imu_acc_buffer = []  # Buffer to accumulate acceleration samples
        self.last_integration_time = None  # Last time we performed integration
        self.integration_interval = 0.1  # Integrate every 100ms (10Hz)

        self.get_logger().info('Topic Monitor Node initialized')

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to euler angles (roll, pitch, yaw) in degrees"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Convert to degrees
        roll_deg = roll * 180.0 / math.pi
        pitch_deg = pitch * 180.0 / math.pi
        yaw_deg = yaw * 180.0 / math.pi

        return roll_deg, pitch_deg, yaw_deg

    def subscribe_to_topic(self, topic_name, msg_type):
        """Subscribe to a topic dynamically"""
        if topic_name in self.topic_subscriptions:
            self.get_logger().warn(f'Already subscribed to {topic_name}')
            return

        try:
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

            # Use BEST_EFFORT for sensor topics (especially images and point clouds)
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                durability=DurabilityPolicy.VOLATILE
            )

            # Create subscription with BEST_EFFORT QoS
            sub = self.create_subscription(
                msg_type,
                topic_name,
                lambda msg, topic=topic_name: self._topic_callback(topic, msg),
                qos_profile
            )
            self.topic_subscriptions[topic_name] = sub
            self.get_logger().info(f'Subscribed to {topic_name} with BEST_EFFORT QoS')
        except Exception as e:
            self.get_logger().error(f'Failed to subscribe to {topic_name}: {e}')

    def _topic_callback(self, topic_name, msg):
        """Callback for all subscribed topics"""
        self.latest_messages[topic_name] = msg
        self.message_timestamps[topic_name] = self.get_clock().now()

        # Process IMU data for path integration
        if self.imu_path_enabled and topic_name == '/imu/data' and hasattr(msg, 'linear_acceleration'):
            self._process_imu_for_path(msg)

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
            # Check if this is an Image message
            if hasattr(msg, 'encoding') and hasattr(msg, 'data'):
                return self._image_to_dict(msg)

            # Simple conversion - expand this as needed
            msg_dict = {}
            for field in msg.get_fields_and_field_types():
                value = getattr(msg, field)
                # Check for Quaternion first (has w attribute)
                if hasattr(value, 'w') and hasattr(value, 'x') and hasattr(value, 'y') and hasattr(value, 'z'):
                    # Quaternion type
                    msg_dict[field] = {'x': value.x, 'y': value.y, 'z': value.z, 'w': value.w}
                    # Add euler angles for orientation field
                    if field == 'orientation':
                        roll, pitch, yaw = self.quaternion_to_euler(value.x, value.y, value.z, value.w)
                        msg_dict['euler_angles'] = {
                            'roll': round(roll, 2),
                            'pitch': round(pitch, 2),
                            'yaw': round(yaw, 2)
                        }
                elif hasattr(value, 'x') and hasattr(value, 'y') and hasattr(value, 'z'):
                    # Vector3 type (no w attribute)
                    msg_dict[field] = {'x': value.x, 'y': value.y, 'z': value.z}
                elif isinstance(value, (int, float, str, bool)):
                    msg_dict[field] = value
                else:
                    msg_dict[field] = str(value)
            return msg_dict
        except Exception as e:
            return {'error': f'Failed to convert message: {e}', 'raw': str(msg)}

    def _image_to_dict(self, img_msg):
        """Convert ROS Image message to dictionary with base64 encoded image"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')

            # Normalize image for visualization
            if img_msg.encoding in ['16UC1', '32FC1']:
                # For depth/range images, normalize to 8-bit
                cv_image_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
                cv_image_8bit = cv_image_normalized.astype(np.uint8)
            elif img_msg.encoding == 'mono8':
                cv_image_8bit = cv_image
            else:
                # Try to convert to grayscale
                if len(cv_image.shape) == 3:
                    cv_image_8bit = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                else:
                    cv_image_8bit = cv_image

            # Encode as JPEG
            _, buffer = cv2.imencode('.jpg', cv_image_8bit)
            img_base64 = base64.b64encode(buffer).decode('utf-8')

            return {
                'type': 'image',
                'encoding': img_msg.encoding,
                'height': img_msg.height,
                'width': img_msg.width,
                'image_data': img_base64
            }
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return {
                'type': 'image',
                'error': str(e),
                'encoding': img_msg.encoding,
                'height': img_msg.height,
                'width': img_msg.width
            }

    def _process_imu_for_path(self, imu_msg):
        """Process IMU data - accumulate in buffer for fixed-rate integration"""
        current_time = time.time()

        # Calibration mode - collect samples
        if self.imu_calibrating:
            if len(self.calibration_samples) < 100:  # Collect 100 samples (1 second at 100Hz)
                # Get orientation quaternion
                q = imu_msg.orientation
                q_w, q_x, q_y, q_z = q.w, q.x, q.y, q.z

                # Rotation matrix from quaternion
                rot_matrix = np.array([
                    [1 - 2*(q_y**2 + q_z**2), 2*(q_x*q_y - q_w*q_z), 2*(q_x*q_z + q_w*q_y)],
                    [2*(q_x*q_y + q_w*q_z), 1 - 2*(q_x**2 + q_z**2), 2*(q_y*q_z - q_w*q_x)],
                    [2*(q_x*q_z - q_w*q_y), 2*(q_y*q_z + q_w*q_x), 1 - 2*(q_x**2 + q_y**2)]
                ])

                # Rotate to world frame
                acc_raw = np.array([
                    imu_msg.linear_acceleration.x,
                    imu_msg.linear_acceleration.y,
                    imu_msg.linear_acceleration.z
                ])
                acc_world = rot_matrix @ acc_raw
                acc_world[2] -= 9.81  # Remove gravity

                self.calibration_samples.append(acc_world)

                # Calculate bias when enough samples collected (increased to 200 samples ~ 2 seconds)
                if len(self.calibration_samples) == 200:
                    samples = np.array(self.calibration_samples)
                    self.imu_bias['x'] = np.mean(samples[:, 0])
                    self.imu_bias['y'] = np.mean(samples[:, 1])
                    self.imu_bias['z'] = np.mean(samples[:, 2])

                    # Also store the standard deviation for adaptive thresholding
                    self.imu_noise_std = {
                        'x': np.std(samples[:, 0]),
                        'y': np.std(samples[:, 1]),
                        'z': np.std(samples[:, 2])
                    }

                    self.imu_calibrating = False
                    self.calibration_samples = []
                    self.get_logger().info(f"IMU Bias Calibrated: X={self.imu_bias['x']:.4f}, Y={self.imu_bias['y']:.4f}, Z={self.imu_bias['z']:.4f}")
                    self.get_logger().info(f"IMU Noise STD: X={self.imu_noise_std['x']:.4f}, Y={self.imu_noise_std['y']:.4f}, Z={self.imu_noise_std['z']:.4f}")
            return

        # Process acceleration and add to buffer
        q = imu_msg.orientation
        q_w, q_x, q_y, q_z = q.w, q.x, q.y, q.z

        # Rotation matrix from quaternion
        rot_matrix = np.array([
            [1 - 2*(q_y**2 + q_z**2), 2*(q_x*q_y - q_w*q_z), 2*(q_x*q_z + q_w*q_y)],
            [2*(q_x*q_y + q_w*q_z), 1 - 2*(q_x**2 + q_z**2), 2*(q_y*q_z - q_w*q_x)],
            [2*(q_x*q_z - q_w*q_y), 2*(q_y*q_z + q_w*q_x), 1 - 2*(q_x**2 + q_y**2)]
        ])

        # Get raw acceleration
        acc_raw = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ])

        # Rotate acceleration to world frame and remove gravity
        acc_world = rot_matrix @ acc_raw
        acc_world[2] -= 9.81

        # Remove bias
        acc_corrected = {
            'x': acc_world[0] - self.imu_bias['x'],
            'y': acc_world[1] - self.imu_bias['y'],
            'z': acc_world[2] - self.imu_bias['z'],
            'time': current_time
        }

        # Add to accumulation buffer
        self.imu_acc_buffer.append(acc_corrected)

        # Initialize integration timer if needed
        if self.last_integration_time is None:
            self.last_integration_time = current_time
            return

        # Check if it's time to integrate (every 0.1 seconds)
        if current_time - self.last_integration_time >= self.integration_interval:
            self._perform_integration(current_time)

    def _perform_integration(self, current_time):
        """Perform integration at fixed intervals using accumulated data"""
        if not self.imu_acc_buffer:
            return

        # Calculate average acceleration from buffer
        acc_x = np.mean([a['x'] for a in self.imu_acc_buffer])
        acc_y = np.mean([a['y'] for a in self.imu_acc_buffer])
        acc_z = np.mean([a['z'] for a in self.imu_acc_buffer])

        # Store buffer size before clearing
        buffer_size = len(self.imu_acc_buffer)

        # Clear buffer for next integration
        self.imu_acc_buffer = []

        # Calculate actual dt
        dt = current_time - self.last_integration_time
        self.last_integration_time = current_time

        # Log integration rate
        if not hasattr(self, 'integration_counter'):
            self.integration_counter = 0
        self.integration_counter += 1
        if self.integration_counter % 10 == 0:  # Log every 1 second (10 * 0.1s)
            self.get_logger().info(f"Integration Rate: {1.0/dt:.1f}Hz, Buffer averaged {buffer_size} samples")

        # Higher threshold to filter out all noise in stationary state
        # Based on observed noise levels: X~0.07, Y~0.3 m/s^2
        if hasattr(self, 'imu_noise_std'):
            threshold_x = max(0.3, 3 * self.imu_noise_std['x'])  # Increased to 0.3 m/s^2
            threshold_y = max(0.3, 3 * self.imu_noise_std['y'])
            threshold_z = max(0.3, 3 * self.imu_noise_std['z'])
        else:
            # Default threshold based on observed noise
            threshold_x = threshold_y = threshold_z = 0.3  # m/s^2

        if abs(acc_x) < threshold_x: acc_x = 0
        if abs(acc_y) < threshold_y: acc_y = 0
        if abs(acc_z) < threshold_z: acc_z = 0

        # Check for stationary condition (all accelerations are zero after thresholding)
        is_stationary = (acc_x == 0 and acc_y == 0 and acc_z == 0)

        # Integrate acceleration to get velocity
        self.imu_velocity['x'] += acc_x * dt
        self.imu_velocity['y'] += acc_y * dt
        self.imu_velocity['z'] += acc_z * dt

        # Apply stronger damping when stationary
        if is_stationary:
            # Aggressive damping when stationary
            damping = 0.95  # Stronger damping
            # Also apply zero-velocity update if really still
            speed = np.sqrt(self.imu_velocity['x']**2 +
                          self.imu_velocity['y']**2 +
                          self.imu_velocity['z']**2)
            if speed < 0.05:  # If moving very slowly when stationary
                self.imu_velocity['x'] *= 0.9  # Extra damping
                self.imu_velocity['y'] *= 0.9
                self.imu_velocity['z'] *= 0.9
        else:
            damping = 0.98  # Normal damping when moving

        self.imu_velocity['x'] *= damping
        self.imu_velocity['y'] *= damping
        self.imu_velocity['z'] *= damping

        # Zero out very small velocities - increased threshold
        velocity_threshold = 0.01  # m/s (increased from 0.001)
        if abs(self.imu_velocity['x']) < velocity_threshold: self.imu_velocity['x'] = 0
        if abs(self.imu_velocity['y']) < velocity_threshold: self.imu_velocity['y'] = 0
        if abs(self.imu_velocity['z']) < velocity_threshold: self.imu_velocity['z'] = 0

        # Integrate velocity to get position
        self.imu_path['x'] += self.imu_velocity['x'] * dt
        self.imu_path['y'] += self.imu_velocity['y'] * dt
        self.imu_path['z'] += self.imu_velocity['z'] * dt

        # Store path history
        self.imu_path_history.append({
            'x': self.imu_path['x'],
            'y': self.imu_path['y'],
            'z': self.imu_path['z'],
            'time': current_time
        })

        # Remove old data beyond duration
        cutoff_time = current_time - self.imu_path_duration
        while self.imu_path_history and self.imu_path_history[0]['time'] < cutoff_time:
            self.imu_path_history.popleft()

    def reset_imu_path(self):
        """Reset IMU path integration"""
        self.imu_path = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.imu_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.imu_path_history.clear()
        self.imu_data_buffer.clear()
        self.imu_acc_buffer = []
        self.last_integration_time = None
        if hasattr(self, 'integration_counter'):
            self.integration_counter = 0

    def calibrate_imu_bias(self):
        """Start IMU bias calibration"""
        self.imu_calibrating = True
        self.calibration_samples = []
        self.reset_imu_path()
        self.get_logger().info("Starting IMU bias calibration...")

    def get_imu_path_data(self):
        """Get current IMU path data for visualization"""
        if not self.imu_path_history:
            return None

        # Convert path history to list of points based on selected plane
        path_points = []
        for point in self.imu_path_history:
            if self.imu_path_plane == 'xy':
                path_points.append({'x': point['x'], 'y': point['y']})
            elif self.imu_path_plane == 'xz':
                path_points.append({'x': point['x'], 'y': point['z']})
            elif self.imu_path_plane == 'yz':
                path_points.append({'x': point['y'], 'y': point['z']})

        # Calculate update rate info (now fixed at 10Hz)
        update_rate = 10.0  # Fixed 10Hz integration rate

        return {
            'plane': self.imu_path_plane,
            'path': path_points,
            'current_position': self.imu_path,
            'current_velocity': self.imu_velocity,
            'duration': self.imu_path_duration,
            'enabled': self.imu_path_enabled,
            'update_rate_hz': update_rate,
            'buffer_size': len(self.imu_acc_buffer)
        }

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

@app.route('/imu_timeseries')
def imu_timeseries():
    """Serve the IMU time series visualization page"""
    return render_template('imu_timeseries.html')

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
        elif msg_type_str == 'sensor_msgs/msg/Image':
            msg_type = Image
        else:
            # Try to dynamically load the message type
            return jsonify({
                "success": False,
                "error": f"Message type '{msg_type_str}' not supported yet. Use 'sensor_msgs/msg/Imu' or 'sensor_msgs/msg/Image'"
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
    global ros_node

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

        # Check if topics are actually publishing (receiving messages)
        imu_topics_active = False
        ouster_topics_active = False
        imu_topic_list = []
        ouster_topic_list = []

        # Define timeout for considering a topic as active (seconds)
        TOPIC_ACTIVE_TIMEOUT = 3.0

        if ros_node:
            current_time = ros_node.get_clock().now()

            # Check IMU topics for recent messages
            imu_topics_to_check = ['/imu/data', '/imu/data_raw']
            for topic in imu_topics_to_check:
                if topic in ros_node.message_timestamps:
                    age = (current_time - ros_node.message_timestamps[topic]).nanoseconds / 1e9
                    if age < TOPIC_ACTIVE_TIMEOUT:
                        imu_topics_active = True
                        imu_topic_list.append(topic)

            # Check Ouster topics for recent messages
            ouster_topics_to_check = ['/ouster/nearir_image', '/ouster/range_image',
                                     '/ouster/reflec_image', '/ouster/signal_image']
            for topic in ouster_topics_to_check:
                if topic in ros_node.message_timestamps:
                    age = (current_time - ros_node.message_timestamps[topic]).nanoseconds / 1e9
                    if age < TOPIC_ACTIVE_TIMEOUT:
                        ouster_topics_active = True
                        ouster_topic_list.append(topic)

        # Also get list of available topics from ros2 topic list for completeness
        try:
            result = subprocess.run(
                ['bash', '-c', 'source /opt/ros/jazzy/setup.bash && source /home/kimghw/microstrain_ws/install/setup.bash && timeout 2 ros2 topic list'],
                capture_output=True,
                text=True,
                timeout=3
            )

            if result.returncode == 0:
                available_topics = result.stdout.strip().split('\n')

                # Add any additional IMU topics that exist
                all_imu_topics = [t for t in available_topics if 'imu' in t.lower() or 'ekf' in t or 'mip' in t or 'gnss' in t.lower()]
                for topic in all_imu_topics:
                    if topic not in imu_topic_list:
                        imu_topic_list.append(topic)

                # Add any additional Ouster topics that exist
                all_ouster_topics = [t for t in available_topics if 'ouster' in t.lower()]
                for topic in all_ouster_topics:
                    if topic not in ouster_topic_list:
                        ouster_topic_list.append(topic)

        except subprocess.TimeoutExpired:
            # If timeout, use only the data we have from ros_node
            pass

        return jsonify({
            "success": True,
            "drivers": {
                "imu": {
                    "running": imu_running,
                    "publishing": imu_topics_active,
                    "status": "active" if imu_topics_active else ("started" if imu_running else "stopped"),
                    "topics": sorted(imu_topic_list)
                },
                "ouster": {
                    "running": ouster_running,
                    "publishing": ouster_topics_active,
                    "status": "active" if ouster_topics_active else ("started" if ouster_running else "stopped"),
                    "topics": sorted(ouster_topic_list)
                }
            }
        })

    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@app.route('/api/imu/path/enable', methods=['POST'])
def enable_imu_path():
    """Enable IMU path tracking with bias calibration"""
    global ros_node

    if ros_node is None:
        return jsonify({
            "success": False,
            "error": "ROS2 node not initialized"
        }), 500

    data = request.get_json() or {}
    plane = data.get('plane', 'xy')
    duration = data.get('duration', 5.0)

    if plane not in ['xy', 'xz', 'yz']:
        return jsonify({
            "success": False,
            "error": "Invalid plane. Must be 'xy', 'xz', or 'yz'"
        }), 400

    ros_node.imu_path_plane = plane
    ros_node.imu_path_duration = duration

    # Start calibration first
    ros_node.calibrate_imu_bias()
    ros_node.imu_path_enabled = True

    return jsonify({
        "success": True,
        "message": f"IMU calibrating... Keep IMU still for 1 second",
        "plane": plane,
        "duration": duration,
        "calibrating": True
    })

@app.route('/api/imu/path/disable', methods=['POST'])
def disable_imu_path():
    """Disable IMU path tracking"""
    global ros_node

    if ros_node is None:
        return jsonify({
            "success": False,
            "error": "ROS2 node not initialized"
        }), 500

    ros_node.imu_path_enabled = False

    return jsonify({
        "success": True,
        "message": "IMU path tracking disabled"
    })

@app.route('/api/imu/path/reset', methods=['POST'])
def reset_imu_path():
    """Reset IMU path data"""
    global ros_node

    if ros_node is None:
        return jsonify({
            "success": False,
            "error": "ROS2 node not initialized"
        }), 500

    ros_node.reset_imu_path()

    return jsonify({
        "success": True,
        "message": "IMU path reset"
    })

@app.route('/api/imu/path/data', methods=['GET'])
def get_imu_path_data():
    """Get current IMU path data"""
    global ros_node

    if ros_node is None:
        return jsonify({
            "success": False,
            "error": "ROS2 node not initialized"
        }), 500

    path_data = ros_node.get_imu_path_data()

    if path_data is None:
        return jsonify({
            "success": False,
            "error": "No path data available"
        }), 404

    # Add calibration status
    path_data['calibrating'] = ros_node.imu_calibrating
    path_data['bias'] = dict(ros_node.imu_bias)

    return jsonify({
        "success": True,
        "data": path_data
    })

@app.route('/api/imu_stream')
def imu_stream():
    """Stream IMU data as Server-Sent Events"""
    global ros_node

    def generate():
        if ros_node is None:
            yield f"data: {json.dumps({'error': 'ROS2 node not initialized'})}\n\n"
            return

        while True:
            try:
                # Get latest IMU message
                msg_data = ros_node.get_latest_message('/imu/data')

                if msg_data is not None:
                    # Send the IMU data as JSON
                    imu_data = msg_data['message']
                    yield f"data: {json.dumps(imu_data)}\n\n"

                # Sleep briefly to control update rate (around 20Hz)
                time.sleep(0.05)

            except Exception as e:
                yield f"data: {json.dumps({'error': str(e)})}\n\n"
                break

    return Response(generate(), mimetype='text/event-stream')

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

    # Auto-subscribe to Ouster image topics
    ros_node.subscribe_to_topic('/ouster/nearir_image', Image)
    ros_node.subscribe_to_topic('/ouster/range_image', Image)
    ros_node.subscribe_to_topic('/ouster/reflec_image', Image)
    ros_node.subscribe_to_topic('/ouster/signal_image', Image)

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
