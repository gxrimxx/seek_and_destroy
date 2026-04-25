import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        # Parameter: path to targets config file
        self.declare_parameter('targets_file', '')
        targets_file = self.get_parameter('targets_file').value

        if not targets_file or not os.path.exists(targets_file):
            self.get_logger().error(f'targets.yaml not found at: {targets_file}')
            self.get_logger().error('Pass it with: --ros-args -p targets_file:=/path/to/targets.yaml')
            return

        with open(targets_file, 'r') as f:
            config = yaml.safe_load(f)
        self.targets = config['targets']
        self.get_logger().info(f'Loaded {len(self.targets)} targets: {list(self.targets.keys())}')

        self.bridge = CvBridge()

        # Publishers
        self.result_pub = self.create_publisher(String, '/detection/result', 10)
        self.debug_pub = self.create_publisher(Image, '/detection/image_debug', 10)

        # Subscriber to camera
        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('Color detector ready. Watching for targets...')

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]
        total_pixels = h * w

        best_name = None
        best_conf = 0.0
        best_box = None

        for name, cfg in self.targets.items():
            lo = np.array(cfg['hsv_lower'], dtype=np.uint8)
            hi = np.array(cfg['hsv_upper'], dtype=np.uint8)
            mask = cv2.inRange(hsv, lo, hi)

            # Handle red hue wrap-around (red appears at BOTH ends of HSV)
            if 'hsv_lower2' in cfg:
                lo2 = np.array(cfg['hsv_lower2'], dtype=np.uint8)
                hi2 = np.array(cfg['hsv_upper2'], dtype=np.uint8)
                mask2 = cv2.inRange(hsv, lo2, hi2)
                mask = cv2.bitwise_or(mask, mask2)

            # Clean up noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area < cfg.get('min_area', 400):
                continue

            # Confidence = fraction of image the object fills (capped at 1.0)
            conf = min(area / (total_pixels * 0.5), 1.0)

            if conf > best_conf:
                best_conf = conf
                best_name = name
                best_box = cv2.boundingRect(largest)

        # Publish result
        if best_name and best_conf > 0.01:
            result_msg = String()
            result_msg.data = f'{best_name}:{best_conf:.3f}'
            self.result_pub.publish(result_msg)

        # Always publish debug image
        if best_box is not None:
            x, y, bw, bh = best_box
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            label = f'{best_name} ({best_conf:.2f})'
            cv2.putText(frame, label, (x, max(y - 8, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.debug_pub.publish(debug_msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    rclpy.spin(node)


if __name__ == '__main__':
    main()