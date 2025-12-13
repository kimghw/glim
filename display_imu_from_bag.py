#!/usr/bin/env python3
"""
Display IMU data from ROS bag file
Shows orientation, angular velocity, and linear acceleration in real-time graphs
"""

import sys
import argparse
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque
import rosbag
import math

class IMUDisplay:
    def __init__(self, bag_file, topic='/microstrain/imu/data'):
        self.bag_file = bag_file
        self.topic = topic

        # Data storage
        self.times = []
        self.roll = []
        self.pitch = []
        self.yaw = []
        self.gyro_x = []
        self.gyro_y = []
        self.gyro_z = []
        self.acc_x = []
        self.acc_y = []
        self.acc_z = []

        # Load data from bag
        self.load_bag()

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

    def load_bag(self):
        """Load IMU data from rosbag"""
        try:
            bag = rosbag.Bag(self.bag_file, 'r')

            # Check if topic exists
            info = bag.get_type_and_topic_info()
            if self.topic not in info.topics:
                print(f"\nError: Topic '{self.topic}' not found in bag!")
                print("\nAvailable IMU topics:")
                for topic in info.topics:
                    if 'imu' in topic.lower():
                        print(f"  - {topic}")
                sys.exit(1)

            msg_count = info.topics[self.topic].message_count
            print(f"\nLoading {msg_count} IMU messages from {self.topic}...")

            start_time = None
            count = 0

            for topic, msg, t in bag.read_messages(topics=[self.topic]):
                if start_time is None:
                    start_time = t.to_sec()

                # Time relative to start
                rel_time = t.to_sec() - start_time
                self.times.append(rel_time)

                # Orientation (convert quaternion to Euler)
                r, p, y = self.quaternion_to_euler(
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w
                )
                self.roll.append(r)
                self.pitch.append(p)
                self.yaw.append(y)

                # Angular velocity (rad/s)
                self.gyro_x.append(msg.angular_velocity.x)
                self.gyro_y.append(msg.angular_velocity.y)
                self.gyro_z.append(msg.angular_velocity.z)

                # Linear acceleration (m/s²)
                self.acc_x.append(msg.linear_acceleration.x)
                self.acc_y.append(msg.linear_acceleration.y)
                self.acc_z.append(msg.linear_acceleration.z)

                count += 1
                if count % 1000 == 0:
                    print(f"  Loaded {count}/{msg_count} messages...")

            bag.close()

            print(f"Successfully loaded {len(self.times)} data points")
            print(f"Duration: {self.times[-1]:.1f} seconds")
            print(f"Sample rate: {len(self.times)/self.times[-1]:.1f} Hz\n")

        except Exception as e:
            print(f"Error loading bag file: {e}")
            sys.exit(1)

    def display(self):
        """Display IMU data in matplotlib plots"""
        # Create figure with subplots
        fig = plt.figure(figsize=(15, 10))
        fig.suptitle(f'IMU Data from: {self.bag_file}', fontsize=14, fontweight='bold')

        # Orientation (Euler angles)
        ax1 = plt.subplot(3, 3, 1)
        ax1.plot(self.times, self.roll, 'r-', linewidth=0.8)
        ax1.set_title('Roll')
        ax1.set_ylabel('Degrees')
        ax1.grid(True, alpha=0.3)

        ax2 = plt.subplot(3, 3, 2)
        ax2.plot(self.times, self.pitch, 'g-', linewidth=0.8)
        ax2.set_title('Pitch')
        ax2.set_ylabel('Degrees')
        ax2.grid(True, alpha=0.3)

        ax3 = plt.subplot(3, 3, 3)
        ax3.plot(self.times, self.yaw, 'b-', linewidth=0.8)
        ax3.set_title('Yaw')
        ax3.set_ylabel('Degrees')
        ax3.grid(True, alpha=0.3)

        # Angular velocity
        ax4 = plt.subplot(3, 3, 4)
        ax4.plot(self.times, self.gyro_x, 'c-', linewidth=0.8)
        ax4.set_title('Gyro X')
        ax4.set_ylabel('rad/s')
        ax4.grid(True, alpha=0.3)

        ax5 = plt.subplot(3, 3, 5)
        ax5.plot(self.times, self.gyro_y, 'm-', linewidth=0.8)
        ax5.set_title('Gyro Y')
        ax5.set_ylabel('rad/s')
        ax5.grid(True, alpha=0.3)

        ax6 = plt.subplot(3, 3, 6)
        ax6.plot(self.times, self.gyro_z, 'y-', linewidth=0.8)
        ax6.set_title('Gyro Z')
        ax6.set_ylabel('rad/s')
        ax6.grid(True, alpha=0.3)

        # Linear acceleration
        ax7 = plt.subplot(3, 3, 7)
        ax7.plot(self.times, self.acc_x, 'orange', linewidth=0.8)
        ax7.set_title('Accel X')
        ax7.set_xlabel('Time (s)')
        ax7.set_ylabel('m/s²')
        ax7.grid(True, alpha=0.3)

        ax8 = plt.subplot(3, 3, 8)
        ax8.plot(self.times, self.acc_y, 'lime', linewidth=0.8)
        ax8.set_title('Accel Y')
        ax8.set_xlabel('Time (s)')
        ax8.set_ylabel('m/s²')
        ax8.grid(True, alpha=0.3)

        ax9 = plt.subplot(3, 3, 9)
        ax9.plot(self.times, self.acc_z, 'cyan', linewidth=0.8)
        ax9.set_title('Accel Z')
        ax9.set_xlabel('Time (s)')
        ax9.set_ylabel('m/s²')
        ax9.grid(True, alpha=0.3)

        # Add statistics
        stats_text = self.get_statistics()
        fig.text(0.5, 0.02, stats_text, ha='center', fontsize=10,
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.tight_layout(rect=[0, 0.05, 1, 0.96])
        plt.show()

    def get_statistics(self):
        """Calculate and return statistics string"""
        duration = self.times[-1] if self.times else 0

        # Calculate RMS for each sensor
        rms_gyro = np.sqrt(np.mean(np.array(self.gyro_x)**2 +
                                   np.array(self.gyro_y)**2 +
                                   np.array(self.gyro_z)**2))
        rms_acc = np.sqrt(np.mean(np.array(self.acc_x)**2 +
                                  np.array(self.acc_y)**2 +
                                  np.array(self.acc_z)**2))

        stats = f"Duration: {duration:.1f}s | "
        stats += f"Samples: {len(self.times)} | "
        stats += f"Rate: {len(self.times)/duration:.1f} Hz | "
        stats += f"RMS Gyro: {rms_gyro:.3f} rad/s | "
        stats += f"RMS Accel: {rms_acc:.2f} m/s²"

        return stats

def main():
    parser = argparse.ArgumentParser(description='Display IMU data from ROS bag')
    parser.add_argument('bag_file', help='Path to ROS bag file')
    parser.add_argument('--topic', default='/microstrain/imu/data',
                       help='IMU topic (default: /microstrain/imu/data)')

    args = parser.parse_args()

    # Create display
    display = IMUDisplay(args.bag_file, args.topic)

    # Show visualization
    display.display()

if __name__ == '__main__':
    main()