#!/usr/bin/env python3
"""
Display IMU data from ROS 2 bag file using ros2 bag play and topic echo
"""

import sys
import argparse
import subprocess
import signal
import time
import json
import math
import matplotlib.pyplot as plt
from collections import deque
import threading
import numpy as np

class IMUDisplayROS2:
    def __init__(self, bag_file, topic='/microstrain/imu/data', duration=30):
        self.bag_file = bag_file
        self.topic = topic
        self.duration = duration
        self.running = True

        # Data storage
        self.times = deque(maxlen=10000)
        self.roll = deque(maxlen=10000)
        self.pitch = deque(maxlen=10000)
        self.yaw = deque(maxlen=10000)
        self.gyro_x = deque(maxlen=10000)
        self.gyro_y = deque(maxlen=10000)
        self.gyro_z = deque(maxlen=10000)
        self.acc_x = deque(maxlen=10000)
        self.acc_y = deque(maxlen=10000)
        self.acc_z = deque(maxlen=10000)

        self.start_time = None
        self.data_count = 0

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (in degrees)"""
        # Roll
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def check_bag_info(self):
        """Check bag file information using ros2 bag info"""
        try:
            cmd = f"ros2 bag info {self.bag_file}"
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)

            if result.returncode != 0:
                print(f"Error reading bag file: {result.stderr}")
                return False

            print("\nBag File Information:")
            print("-" * 40)
            print(result.stdout)

            # Check if topic exists
            if self.topic not in result.stdout:
                print(f"\nWarning: Topic '{self.topic}' may not be in this bag file.")
                print("Available topics with 'imu' in name:")

                # Find IMU topics
                lines = result.stdout.split('\n')
                for line in lines:
                    if 'imu' in line.lower() and '|' in line:
                        print(f"  - {line.strip()}")

                print("\nContinue anyway? (y/n): ", end='')
                response = input().strip().lower()
                if response != 'y':
                    return False

            return True

        except Exception as e:
            print(f"Error checking bag info: {e}")
            return False

    def capture_data(self):
        """Capture IMU data by playing bag and echoing topic"""
        print(f"\nPlaying bag file for {self.duration} seconds...")
        print(f"Listening to topic: {self.topic}")
        print("Press Ctrl+C to stop early\n")

        try:
            # Start ros2 bag play in background
            play_cmd = f"ros2 bag play {self.bag_file} --rate 1.0"
            play_process = subprocess.Popen(play_cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            # Start ros2 topic echo to capture data
            echo_cmd = f"ros2 topic echo {self.topic} sensor_msgs/msg/Imu --once"

            start = time.time()
            while time.time() - start < self.duration and self.running:
                try:
                    result = subprocess.run(echo_cmd, shell=True, capture_output=True, text=True, timeout=1)

                    if result.returncode == 0:
                        self.parse_imu_message(result.stdout)
                        self.data_count += 1

                        if self.data_count % 10 == 0:
                            print(f"Captured {self.data_count} messages...", end='\r')

                except subprocess.TimeoutExpired:
                    continue
                except KeyboardInterrupt:
                    break

            # Stop bag playback
            play_process.terminate()
            play_process.wait(timeout=2)

            print(f"\nCaptured {self.data_count} IMU messages")

        except Exception as e:
            print(f"Error during data capture: {e}")

    def parse_imu_message(self, msg_text):
        """Parse IMU message from ros2 topic echo output"""
        try:
            lines = msg_text.split('\n')

            # Extract values from YAML-like format
            orientation = {'x': 0, 'y': 0, 'z': 0, 'w': 1}
            angular_velocity = {'x': 0, 'y': 0, 'z': 0}
            linear_acceleration = {'x': 0, 'y': 0, 'z': 0}

            section = None
            for line in lines:
                line = line.strip()

                if 'orientation:' in line:
                    section = 'orientation'
                elif 'angular_velocity:' in line:
                    section = 'angular_velocity'
                elif 'linear_acceleration:' in line:
                    section = 'linear_acceleration'
                elif section and ':' in line:
                    key, value = line.split(':', 1)
                    key = key.strip()
                    value = value.strip()

                    if key in ['x', 'y', 'z', 'w']:
                        try:
                            val = float(value)
                            if section == 'orientation':
                                orientation[key] = val
                            elif section == 'angular_velocity':
                                angular_velocity[key] = val
                            elif section == 'linear_acceleration':
                                linear_acceleration[key] = val
                        except ValueError:
                            pass

            # Store time
            if self.start_time is None:
                self.start_time = time.time()

            rel_time = time.time() - self.start_time
            self.times.append(rel_time)

            # Convert quaternion to Euler
            r, p, y = self.quaternion_to_euler(
                orientation['x'], orientation['y'],
                orientation['z'], orientation['w']
            )
            self.roll.append(r)
            self.pitch.append(p)
            self.yaw.append(y)

            # Store angular velocity
            self.gyro_x.append(angular_velocity['x'])
            self.gyro_y.append(angular_velocity['y'])
            self.gyro_z.append(angular_velocity['z'])

            # Store linear acceleration
            self.acc_x.append(linear_acceleration['x'])
            self.acc_y.append(linear_acceleration['y'])
            self.acc_z.append(linear_acceleration['z'])

        except Exception as e:
            # Silently skip parsing errors
            pass

    def display(self):
        """Display captured IMU data"""
        if len(self.times) == 0:
            print("No data to display!")
            return

        print(f"\nDisplaying {len(self.times)} data points...")

        # Create figure
        fig = plt.figure(figsize=(15, 10))
        fig.suptitle(f'IMU Data from: {self.bag_file}', fontsize=14, fontweight='bold')

        # Convert deques to lists for plotting
        times = list(self.times)

        # Orientation (Euler angles)
        ax1 = plt.subplot(3, 3, 1)
        ax1.plot(times, list(self.roll), 'r-', linewidth=0.8)
        ax1.set_title('Roll')
        ax1.set_ylabel('Degrees')
        ax1.grid(True, alpha=0.3)

        ax2 = plt.subplot(3, 3, 2)
        ax2.plot(times, list(self.pitch), 'g-', linewidth=0.8)
        ax2.set_title('Pitch')
        ax2.set_ylabel('Degrees')
        ax2.grid(True, alpha=0.3)

        ax3 = plt.subplot(3, 3, 3)
        ax3.plot(times, list(self.yaw), 'b-', linewidth=0.8)
        ax3.set_title('Yaw')
        ax3.set_ylabel('Degrees')
        ax3.grid(True, alpha=0.3)

        # Angular velocity
        ax4 = plt.subplot(3, 3, 4)
        ax4.plot(times, list(self.gyro_x), 'c-', linewidth=0.8)
        ax4.set_title('Gyro X')
        ax4.set_ylabel('rad/s')
        ax4.grid(True, alpha=0.3)

        ax5 = plt.subplot(3, 3, 5)
        ax5.plot(times, list(self.gyro_y), 'm-', linewidth=0.8)
        ax5.set_title('Gyro Y')
        ax5.set_ylabel('rad/s')
        ax5.grid(True, alpha=0.3)

        ax6 = plt.subplot(3, 3, 6)
        ax6.plot(times, list(self.gyro_z), 'y-', linewidth=0.8)
        ax6.set_title('Gyro Z')
        ax6.set_ylabel('rad/s')
        ax6.grid(True, alpha=0.3)

        # Linear acceleration
        ax7 = plt.subplot(3, 3, 7)
        ax7.plot(times, list(self.acc_x), 'orange', linewidth=0.8)
        ax7.set_title('Accel X')
        ax7.set_xlabel('Time (s)')
        ax7.set_ylabel('m/s²')
        ax7.grid(True, alpha=0.3)

        ax8 = plt.subplot(3, 3, 8)
        ax8.plot(times, list(self.acc_y), 'lime', linewidth=0.8)
        ax8.set_title('Accel Y')
        ax8.set_xlabel('Time (s)')
        ax8.set_ylabel('m/s²')
        ax8.grid(True, alpha=0.3)

        ax9 = plt.subplot(3, 3, 9)
        ax9.plot(times, list(self.acc_z), 'cyan', linewidth=0.8)
        ax9.set_title('Accel Z')
        ax9.set_xlabel('Time (s)')
        ax9.set_ylabel('m/s²')
        ax9.grid(True, alpha=0.3)

        # Statistics
        duration = max(times) if times else 0
        stats = f"Capture Duration: {duration:.1f}s | "
        stats += f"Messages: {len(times)} | "
        if duration > 0:
            stats += f"Rate: {len(times)/duration:.1f} Hz"

        fig.text(0.5, 0.02, stats, ha='center', fontsize=10,
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.tight_layout(rect=[0, 0.05, 1, 0.96])
        plt.show()

def main():
    parser = argparse.ArgumentParser(description='Display IMU data from ROS 2 bag')
    parser.add_argument('bag_file', help='Path to ROS 2 bag file')
    parser.add_argument('--topic', default='/microstrain/imu/data',
                       help='IMU topic (default: /microstrain/imu/data)')
    parser.add_argument('--duration', type=int, default=30,
                       help='Duration to capture data in seconds (default: 30)')

    args = parser.parse_args()

    # Create display
    display = IMUDisplayROS2(args.bag_file, args.topic, args.duration)

    # Check bag info
    if not display.check_bag_info():
        sys.exit(1)

    # Capture data
    display.capture_data()

    # Display visualization
    display.display()

if __name__ == '__main__':
    main()