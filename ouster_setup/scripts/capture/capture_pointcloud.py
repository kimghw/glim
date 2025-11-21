#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
import numpy as np
import cv2
import base64
from io import BytesIO
from PIL import Image
import threading
import time
from pathlib import Path

OUTPUT_JSON = Path('/tmp/latest_capture.json')

class ImageCapture(Node):
    def __init__(self):
        super().__init__('image_capture')

        # QoS profile for best effort reliability (matches bag playback)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to nearir_image topic
        self.img_sub = self.create_subscription(
            RosImage,
            '/ouster/nearir_image',
            self.image_callback,
            qos_profile
        )

        self.latest_image = None
        self.capture_lock = threading.Lock()
        self.get_logger().info('Image capture node started')

    def image_callback(self, msg):
        """Process nearir_image and convert to displayable format"""
        try:
            # Convert ROS Image to numpy array
            # nearir_image is mono16 (16-bit grayscale)
            height = msg.height
            width = msg.width

            # Convert bytes to numpy array
            img_array = np.frombuffer(msg.data, dtype=np.uint16).reshape(height, width)

            # Normalize to 8-bit for display (0-255)
            img_normalized = cv2.normalize(img_array, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

            # Apply colormap for better visualization
            img_colored = cv2.applyColorMap(img_normalized, cv2.COLORMAP_JET)

            # Resize for better viewing (reduce size for web)
            display_height = 512
            display_width = int(width * display_height / height)
            img_resized = cv2.resize(img_colored, (display_width, display_height), interpolation=cv2.INTER_LINEAR)

            # Add label
            cv2.putText(img_resized, 'NEAR-IR IMAGE', (10, 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Store latest image
            with self.capture_lock:
                self.latest_image = img_resized

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def get_latest_image_base64(self):
        """Get latest captured image as base64"""
        with self.capture_lock:
            if self.latest_image is None:
                return None

            # Convert to PIL Image
            pil_img = Image.fromarray(cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2RGB))

            # Convert to base64 with JPEG compression
            buffer = BytesIO()
            pil_img.save(buffer, format='JPEG', quality=85)
            img_base64 = base64.b64encode(buffer.getvalue()).decode('utf-8')

            return img_base64

    def save_latest_image(self, filepath):
        """Save latest image to file"""
        with self.capture_lock:
            if self.latest_image is not None:
                cv2.imwrite(filepath, self.latest_image)
                return filepath
        return None


def capture_single_frame():
    """Capture a single frame and save it"""
    import sys

    # 기존 결과 삭제 (stale 파일 방지)
    try:
        if OUTPUT_JSON.exists():
            OUTPUT_JSON.unlink()
    except Exception:
        pass

    # Redirect stderr to suppress ROS2 warnings
    old_stderr = sys.stderr
    sys.stderr = open('/dev/null', 'w')

    try:
        rclpy.init()
        node = ImageCapture()

        # Spin for a few seconds to capture data
        start_time = time.time()
        captured = False

        while time.time() - start_time < 5:
            rclpy.spin_once(node, timeout_sec=0.1)

            # Check if we have image
            if node.latest_image is not None and not captured:
                # Save image
                timestamp = int(time.time())
                filepath = f'/tmp/nearir_capture_{timestamp}.png'
                saved_file = node.save_latest_image(filepath)

                if saved_file:
                    # Also get base64 for web
                        img_base64 = node.get_latest_image_base64()
                        if img_base64:
                            # Save base64 image to file for web access
                            import json
                            with open(OUTPUT_JSON, 'w') as f:
                                json.dump({'nearir': img_base64}, f)
                            captured = True
                            # Restore stderr before printing
                            sys.stderr = old_stderr
                            print(f"Image saved: {saved_file}")
                        print("Base64 image saved for web access")

                node.destroy_node()
                rclpy.shutdown()
                return saved_file

        # Restore stderr
        sys.stderr = old_stderr
        print("No image data received")
        node.destroy_node()
        rclpy.shutdown()
        return None

    finally:
        # Ensure stderr is restored
        if sys.stderr != old_stderr:
            sys.stderr = old_stderr


if __name__ == '__main__':
    result_path = capture_single_frame()
    # Return non-zero exit code when capture fails so callers can detect the error
    import sys
    sys.exit(0 if result_path else 1)
