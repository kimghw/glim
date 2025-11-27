#!/usr/bin/env python3
import math
import sys

def quaternion_to_euler(x, y, z, w):
    """
    Convert quaternion to euler angles (roll, pitch, yaw)
    Returns angles in both radians and degrees
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def rad_to_deg(rad):
    return rad * 180.0 / math.pi

# Example with current IMU quaternion
if __name__ == "__main__":
    # Current IMU quaternion values
    x = 0.995705202746909
    y = -0.021352098868784683
    z = 0.0637251101309553
    w = 0.06367375782402383

    print("Quaternion (x, y, z, w):")
    print(f"  x: {x}")
    print(f"  y: {y}")
    print(f"  z: {z}")
    print(f"  w: {w}")
    print()

    roll, pitch, yaw = quaternion_to_euler(x, y, z, w)

    print("Euler Angles:")
    print(f"  Roll:  {roll:.4f} rad = {rad_to_deg(roll):.2f}°")
    print(f"  Pitch: {pitch:.4f} rad = {rad_to_deg(pitch):.2f}°")
    print(f"  Yaw:   {yaw:.4f} rad = {rad_to_deg(yaw):.2f}°")
    print()

    # Check quaternion normalization
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    print(f"Quaternion norm: {norm:.6f} (should be 1.0)")

    if abs(norm - 1.0) > 0.01:
        print("WARNING: Quaternion is not properly normalized!")