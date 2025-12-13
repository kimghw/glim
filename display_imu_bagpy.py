#!/usr/bin/env python3
"""
Display IMU data from ROS bag file using bagpy
Works with both ROS 1 and ROS 2 bag files
"""

import sys
import argparse
import matplotlib.pyplot as plt
import numpy as np
import math
from bagpy import bagreader
import pandas as pd

class IMUDisplayBagpy:
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

    def load_data(self):
        """Load IMU data from bag file using bagpy"""
        try:
            print(f"\nReading bag file: {self.bag_file}")
            print(f"Looking for topic: {self.topic}")

            # Create bag reader
            b = bagreader(self.bag_file)

            # Get topics
            print("\nAvailable topics:")
            for topic in b.topic_table['Topics']:
                print(f"  - {topic}")

            # Check if our topic exists
            if self.topic not in b.topic_table['Topics'].values:
                print(f"\nWarning: Topic '{self.topic}' not found!")

                # Look for IMU topics
                imu_topics = [t for t in b.topic_table['Topics'] if 'imu' in t.lower()]
                if imu_topics:
                    print(f"Found IMU topics: {imu_topics}")
                    print(f"Using first IMU topic: {imu_topics[0]}")
                    self.topic = imu_topics[0]
                else:
                    print("No IMU topics found in bag file!")
                    return False

            # Read IMU data
            print(f"\nReading IMU data from topic: {self.topic}")
            csv_file = b.message_by_topic(self.topic)
            print(f"CSV file created: {csv_file}")

            # Read CSV data
            df = pd.read_csv(csv_file)
            print(f"Loaded {len(df)} messages")

            # Extract data from dataframe
            if 'Time' in df.columns:
                start_time = df['Time'].iloc[0]
                self.times = (df['Time'] - start_time).tolist()
            else:
                self.times = list(range(len(df)))

            # Process orientation (quaternion to Euler)
            if all(col in df.columns for col in ['orientation.x', 'orientation.y', 'orientation.z', 'orientation.w']):
                for _, row in df.iterrows():
                    r, p, y = self.quaternion_to_euler(
                        row['orientation.x'],
                        row['orientation.y'],
                        row['orientation.z'],
                        row['orientation.w']
                    )
                    self.roll.append(r)
                    self.pitch.append(p)
                    self.yaw.append(y)
            else:
                print("Warning: Orientation data not found")
                self.roll = [0] * len(df)
                self.pitch = [0] * len(df)
                self.yaw = [0] * len(df)

            # Angular velocity
            if all(col in df.columns for col in ['angular_velocity.x', 'angular_velocity.y', 'angular_velocity.z']):
                self.gyro_x = df['angular_velocity.x'].tolist()
                self.gyro_y = df['angular_velocity.y'].tolist()
                self.gyro_z = df['angular_velocity.z'].tolist()
            else:
                print("Warning: Angular velocity data not found")
                self.gyro_x = [0] * len(df)
                self.gyro_y = [0] * len(df)
                self.gyro_z = [0] * len(df)

            # Linear acceleration
            if all(col in df.columns for col in ['linear_acceleration.x', 'linear_acceleration.y', 'linear_acceleration.z']):
                self.acc_x = df['linear_acceleration.x'].tolist()
                self.acc_y = df['linear_acceleration.y'].tolist()
                self.acc_z = df['linear_acceleration.z'].tolist()
            else:
                print("Warning: Linear acceleration data not found")
                self.acc_x = [0] * len(df)
                self.acc_y = [0] * len(df)
                self.acc_z = [0] * len(df)

            print(f"\nSuccessfully loaded {len(self.times)} data points")
            if self.times:
                print(f"Duration: {self.times[-1]:.1f} seconds")
                print(f"Sample rate: {len(self.times)/self.times[-1]:.1f} Hz")

            return True

        except Exception as e:
            print(f"Error loading bag file: {e}")
            import traceback
            traceback.print_exc()
            return False

    def display(self):
        """Display IMU data in matplotlib plots"""
        if len(self.times) == 0:
            print("No data to display!")
            return

        # Create figure with subplots
        fig = plt.figure(figsize=(15, 10))
        fig.suptitle(f'IMU Data from: {self.bag_file}', fontsize=14, fontweight='bold')

        # Orientation (Euler angles)
        ax1 = plt.subplot(3, 3, 1)
        ax1.plot(self.times, self.roll, 'r-', linewidth=0.8)
        ax1.set_title('Roll')
        ax1.set_ylabel('Degrees')
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim([min(self.roll)-5, max(self.roll)+5] if self.roll else [-180, 180])

        ax2 = plt.subplot(3, 3, 2)
        ax2.plot(self.times, self.pitch, 'g-', linewidth=0.8)
        ax2.set_title('Pitch')
        ax2.set_ylabel('Degrees')
        ax2.grid(True, alpha=0.3)
        ax2.set_ylim([min(self.pitch)-5, max(self.pitch)+5] if self.pitch else [-90, 90])

        ax3 = plt.subplot(3, 3, 3)
        ax3.plot(self.times, self.yaw, 'b-', linewidth=0.8)
        ax3.set_title('Yaw')
        ax3.set_ylabel('Degrees')
        ax3.grid(True, alpha=0.3)
        ax3.set_ylim([min(self.yaw)-5, max(self.yaw)+5] if self.yaw else [-180, 180])

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
        if not self.times:
            return "No data available"

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
        if duration > 0:
            stats += f"Rate: {len(self.times)/duration:.1f} Hz | "
        stats += f"RMS Gyro: {rms_gyro:.3f} rad/s | "
        stats += f"RMS Accel: {rms_acc:.2f} m/s²"

        return stats

def main():
    parser = argparse.ArgumentParser(description='Display IMU data from ROS bag using bagpy')
    parser.add_argument('bag_file', help='Path to ROS bag file')
    parser.add_argument('--topic', default='/microstrain/imu/data',
                       help='IMU topic (default: /microstrain/imu/data)')

    args = parser.parse_args()

    # Create display
    display = IMUDisplayBagpy(args.bag_file, args.topic)

    # Load data
    if display.load_data():
        # Show visualization
        display.display()
    else:
        sys.exit(1)

if __name__ == '__main__':
    main()