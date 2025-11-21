#!/usr/bin/env python3
"""
간단한 포인트 클라우드 캡처 스크립트 (디버그용)
"""

import sys
import time
import base64
import numpy as np
from datetime import datetime

print("[DEBUG] 캡처 스크립트 시작", file=sys.stderr)

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    print("[DEBUG] ROS2 모듈 로드 성공", file=sys.stderr)
except ImportError as e:
    print(f"[ERROR] ROS2 모듈 로드 실패: {e}", file=sys.stderr)
    sys.exit(1)

try:
    import cv2
    from PIL import Image
    from io import BytesIO
    print("[DEBUG] 이미지 처리 모듈 로드 성공", file=sys.stderr)
except ImportError as e:
    print(f"[ERROR] 이미지 모듈 로드 실패: {e}", file=sys.stderr)
    sys.exit(1)


class SimpleCapture(Node):
    def __init__(self):
        super().__init__('simple_capture')

        # QoS 설정 (bag 재생과 호환)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            '/ouster/points',
            self.callback,
            qos
        )

        self.captured = False
        self.get_logger().info('캡처 노드 시작됨')
        print("[DEBUG] 토픽 구독 시작: /ouster/points", file=sys.stderr)

    def callback(self, msg):
        if self.captured:
            return

        print(f"[DEBUG] 포인트 클라우드 수신: {msg.width}x{msg.height} 포인트", file=sys.stderr)

        try:
            # 간단한 이미지 생성 (800x800 검은 배경에 색상 점)
            img = np.zeros((800, 800, 3), dtype=np.uint8)

            # 테스트용 패턴 그리기
            cv2.circle(img, (400, 400), 50, (0, 255, 0), -1)  # 중앙 녹색 원
            cv2.putText(img, f"Captured at {datetime.now().strftime('%H:%M:%S')}",
                       (200, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(img, f"Points: {msg.width}x{msg.height}",
                       (200, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            # 격자 그리기
            for i in range(0, 800, 80):
                cv2.line(img, (i, 0), (i, 800), (50, 50, 50), 1)
                cv2.line(img, (0, i), (800, i), (50, 50, 50), 1)

            # PNG 파일로 저장
            timestamp = int(time.time())
            filepath = f'/tmp/capture_{timestamp}.png'
            cv2.imwrite(filepath, img)
            print(f"[SUCCESS] 이미지 저장: {filepath}", file=sys.stderr)

            # Base64 인코딩
            pil_img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            buffer = BytesIO()
            pil_img.save(buffer, format='PNG')
            img_base64 = base64.b64encode(buffer.getvalue()).decode('utf-8')

            # Base64 파일 저장
            with open('/tmp/latest_capture.b64', 'w') as f:
                f.write(img_base64)
            print("[SUCCESS] Base64 파일 저장: /tmp/latest_capture.b64", file=sys.stderr)

            # 성공 메시지 출력 (웹 API용)
            print(f"Image saved to: {filepath}")
            print("Base64 image saved for web access")

            self.captured = True
            self.get_logger().info('캡처 완료')

        except Exception as e:
            print(f"[ERROR] 캡처 실패: {e}", file=sys.stderr)
            import traceback
            traceback.print_exc(file=sys.stderr)


def main():
    print("[DEBUG] main() 함수 시작", file=sys.stderr)

    try:
        rclpy.init()
        print("[DEBUG] ROS2 초기화 완료", file=sys.stderr)

        node = SimpleCapture()

        # 3초간 실행
        start = time.time()
        while time.time() - start < 3:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.captured:
                break

        if not node.captured:
            print("[WARNING] 타임아웃: 데이터를 받지 못함", file=sys.stderr)
            print("No point cloud data received")

        node.destroy_node()
        rclpy.shutdown()

        return 0 if node.captured else 1

    except Exception as e:
        print(f"[ERROR] 실행 오류: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc(file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())