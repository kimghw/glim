#!/usr/bin/env python3
"""
Position tracking node using IMU data integration
Integrates acceleration to estimate velocity and position
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
import numpy as np
import json
import os

class PositionTracker(Node):
    def __init__(self):
        super().__init__('position_tracker')

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publishers for filter-like outputs
        self.pose_pub = self.create_publisher(PoseStamped, '/filter/pose', 10)
        self.velocity_pub = self.create_publisher(TwistStamped, '/filter/velocity', 10)
        self.accel_pub = self.create_publisher(AccelStamped, '/filter/accel', 10)
        self.odom_pub = self.create_publisher(Odometry, '/filter/odom', 10)
        self.heading_pub = self.create_publisher(Float64, '/filter/heading', 10)
        self.path_pub = self.create_publisher(Path, '/filter/path', 10)

        # State variables
        self.position = np.array([0.0, 0.0, 0.0])  # x, y, z in ENU
        self.velocity = np.array([0.0, 0.0, 0.0])  # vx, vy, vz in ENU
        self.last_time = None
        self.orientation = None

        # Path for visualization
        self.path = Path()
        self.path.header.frame_id = 'odom'

        # Save position data for web visualization
        self.position_history = []
        self.data_file = '/tmp/position_tracking.json'

        # Timer to save data periodically
        self.save_timer = self.create_timer(0.5, self.save_position_data)

        self.get_logger().info('Position tracker node started')
        self.get_logger().warn('Note: Position from IMU-only integration will drift over time')

    def imu_callback(self, msg):
        current_time = self.get_clock().now()

        # Initialize on first message
        if self.last_time is None:
            self.last_time = current_time
            self.orientation = msg.orientation
            return

        # Calculate dt
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0 or dt > 1.0:  # Skip if dt is invalid
            self.last_time = current_time
            return

        # Get orientation quaternion
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w

        # Convert quaternion to rotation matrix
        # https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
        rot_matrix = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
        ])

        # Transform linear acceleration from body frame to world frame (ENU)
        accel_body = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Rotate to world frame and remove gravity (assuming z is up in ENU)
        accel_world = rot_matrix @ accel_body
        accel_world[2] -= 9.81  # Remove gravity

        # Integrate acceleration to get velocity (simple integration)
        self.velocity += accel_world * dt

        # Integrate velocity to get position
        self.position += self.velocity * dt

        # Get heading (yaw) in radians from quaternion
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy**2 + qz**2)
        heading = np.arctan2(siny_cosp, cosy_cosp)

        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time.to_msg()
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.position.x = self.position[0]
        pose_msg.pose.position.y = self.position[1]
        pose_msg.pose.position.z = self.position[2]
        pose_msg.pose.orientation = msg.orientation
        self.pose_pub.publish(pose_msg)

        # Publish velocity
        vel_msg = TwistStamped()
        vel_msg.header.stamp = current_time.to_msg()
        vel_msg.header.frame_id = 'odom'
        vel_msg.twist.linear.x = self.velocity[0]
        vel_msg.twist.linear.y = self.velocity[1]
        vel_msg.twist.linear.z = self.velocity[2]
        vel_msg.twist.angular.x = msg.angular_velocity.x
        vel_msg.twist.angular.y = msg.angular_velocity.y
        vel_msg.twist.angular.z = msg.angular_velocity.z
        self.velocity_pub.publish(vel_msg)

        # Publish acceleration
        accel_msg = AccelStamped()
        accel_msg.header.stamp = current_time.to_msg()
        accel_msg.header.frame_id = 'odom'
        accel_msg.accel.linear.x = accel_world[0]
        accel_msg.accel.linear.y = accel_world[1]
        accel_msg.accel.linear.z = accel_world[2]
        self.accel_pub.publish(accel_msg)

        # Publish odometry (combined pose + velocity)
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose = pose_msg.pose
        odom_msg.twist.twist = vel_msg.twist
        self.odom_pub.publish(odom_msg)

        # Publish heading
        heading_msg = Float64()
        heading_msg.data = heading
        self.heading_pub.publish(heading_msg)

        # Add to path
        self.path.header.stamp = current_time.to_msg()
        self.path.poses.append(pose_msg)
        # Keep only last 1000 points
        if len(self.path.poses) > 1000:
            self.path.poses.pop(0)
        self.path_pub.publish(self.path)

        # Store for visualization
        self.position_history.append({
            'timestamp': current_time.nanoseconds / 1e9,
            'x': float(self.position[0]),
            'y': float(self.position[1]),
            'z': float(self.position[2]),
            'vx': float(self.velocity[0]),
            'vy': float(self.velocity[1]),
            'vz': float(self.velocity[2]),
            'heading': float(heading)
        })

        # Keep only last 10000 points
        if len(self.position_history) > 10000:
            self.position_history.pop(0)

        # Update time
        self.last_time = current_time

    def save_position_data(self):
        """Save position data to file for web visualization"""
        if len(self.position_history) > 0:
            try:
                with open(self.data_file, 'w') as f:
                    json.dump({
                        'position_history': self.position_history[-1000:],  # Last 1000 points
                        'current_position': {
                            'x': float(self.position[0]),
                            'y': float(self.position[1]),
                            'z': float(self.position[2])
                        },
                        'current_velocity': {
                            'x': float(self.velocity[0]),
                            'y': float(self.velocity[1]),
                            'z': float(self.velocity[2]),
                            'speed': float(np.linalg.norm(self.velocity))
                        }
                    }, f)
            except Exception as e:
                self.get_logger().error(f'Failed to save position data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PositionTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
