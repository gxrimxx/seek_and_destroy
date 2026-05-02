#!/usr/bin/env python3
"""
color_detector.py  -  Project Seek & Destroy
Detects coloured targets and publishes their MAP-FRAME coordinates.

Two localization methods selectable via ROS parameter 'use_depth':
  Method A (use_depth: true)  → RGB-D back-projection via camera intrinsics + tf2
  Method B (use_depth: false) → LiDAR ray-casting via /scan bearing lookup

Published topic payload:   /detection/result  →  "name:confidence:map_x:map_y"
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os
import math

import message_filters

import tf2_ros
import tf2_geometry_msgs
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class ColorDetector(Node):

    def __init__(self):
        super().__init__('color_detector')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('targets_file', '')
        self.declare_parameter('use_depth',    True)
        # D435 horizontal FOV ≈ 69° = 1.2043 rad.  Adjust if your camera differs.
        self.declare_parameter('camera_hfov',  1.2043)

        targets_file      = self.get_parameter('targets_file').value
        self.use_depth    = self.get_parameter('use_depth').value
        self.camera_hfov  = self.get_parameter('camera_hfov').value

        if not targets_file or not os.path.exists(targets_file):
            self.get_logger().error(f'targets.yaml not found at: {targets_file}')
            return

        with open(targets_file, 'r') as f:
            config = yaml.safe_load(f)
        self.targets = config['targets']
        self.get_logger().info(
            f'Loaded {len(self.targets)} targets: {list(self.targets.keys())}'
        )

        self.bridge = CvBridge()

        # ── TF2 ─────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Camera intrinsics (populated from /camera_info) ──────────────────
        self._fx:        float = None
        self._fy:        float = None
        self._cx_intr:   float = None   # principal point X  (not pixel centroid)
        self._cy_intr:   float = None
        self._img_width: int   = 640

        # ── Latest LiDAR scan cache (Method B) ───────────────────────────────
        self._latest_scan: LaserScan = None

        # ── Publishers ───────────────────────────────────────────────────────
        self.result_pub = self.create_publisher(String, '/detection/result',     10)
        self.debug_pub  = self.create_publisher(Image,  '/detection/image_debug', 10)

        # ── Always subscribe to camera info ──────────────────────────────────
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._on_camera_info, 1
        )

        # ── Method-specific subscriptions ────────────────────────────────────
        if self.use_depth:
            self.get_logger().info('Localization: METHOD A — RGB-D depth camera')
            color_sub = message_filters.Subscriber(
                self, Image, '/camera/color/image_raw'
            )
            depth_sub = message_filters.Subscriber(
                self, Image, '/camera/depth/image_raw'
            )
            self._sync = message_filters.ApproximateTimeSynchronizer(
                [color_sub, depth_sub], queue_size=10, slop=0.05
            )
            self._sync.registerCallback(self._callback_rgbd)
        else:
            self.get_logger().info('Localization: METHOD B — LiDAR ray casting')
            self.create_subscription(
                Image, '/camera/color/image_raw', self._callback_color_only, 10
            )
            self.create_subscription(
                LaserScan, '/scan', self._on_scan, 10
            )

        self.get_logger().info('Color detector ready.')

    # ─────────────────────────────────────────────────────────────────────────
    # INFRASTRUCTURE CALLBACKS
    # ─────────────────────────────────────────────────────────────────────────

    def _on_camera_info(self, msg: CameraInfo):
        """Cache intrinsics once on first message."""
        if self._fx is None:
            self._fx       = msg.k[0]
            self._fy       = msg.k[4]
            self._cx_intr  = msg.k[2]
            self._cy_intr  = msg.k[5]
            self._img_width = msg.width
            self.get_logger().info(
                f'Intrinsics cached: fx={self._fx:.1f} fy={self._fy:.1f} '
                f'cx={self._cx_intr:.1f} cy={self._cy_intr:.1f}'
            )

    def _on_scan(self, msg: LaserScan):
        self._latest_scan = msg

    # ─────────────────────────────────────────────────────────────────────────
    # IMAGE ENTRY POINTS
    # ─────────────────────────────────────────────────────────────────────────

    def _callback_rgbd(self, color_msg: Image, depth_msg: Image):
        """Method A: synchronised colour + depth."""
        frame, det = self._detect_objects(color_msg)
        if det is None:
            return

        if det['name'] is None:
            self._publish_debug(frame, None, None, None, None)
            return

        try:
            depth_frame = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough'
            )
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')
            return

        cx_px, cy_px         = det['center']
        map_x, map_y         = self._depth_to_map(cx_px, cy_px, depth_frame,
                                                    color_msg.header)
        self._publish_result(det['name'], det['conf'], map_x, map_y)
        self._publish_debug(frame, det['box'], det['name'], det['conf'],
                             (map_x, map_y))

    def _callback_color_only(self, color_msg: Image):
        """Method B: colour only, bearing from LiDAR."""
        frame, det = self._detect_objects(color_msg)
        if det is None:
            return

        if det['name'] is None:
            self._publish_debug(frame, None, None, None, None)
            return

        cx_px              = det['center'][0]
        map_x, map_y       = self._lidar_raycast_to_map(cx_px)
        self._publish_result(det['name'], det['conf'], map_x, map_y)
        self._publish_debug(frame, det['box'], det['name'], det['conf'],
                             (map_x, map_y))

    # ─────────────────────────────────────────────────────────────────────────
    # CORE HSV DETECTION  (shared by both methods)
    # ─────────────────────────────────────────────────────────────────────────

    def _detect_objects(self, color_msg: Image):
        """
        Run HSV colour detection on a colour image.
        Returns (annotated_frame, result_dict) or (frame, None-filled dict) on error.
        result_dict: { name, conf, center: (cx,cy), box: (x,y,w,h) }
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}')
            return None, None

        hsv            = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w           = frame.shape[:2]
        total_pixels   = h * w

        best = {'name': None, 'conf': 0.0, 'center': None, 'box': None}

        for name, cfg in self.targets.items():
            lo   = np.array(cfg['hsv_lower'], dtype=np.uint8)
            hi   = np.array(cfg['hsv_upper'], dtype=np.uint8)
            mask = cv2.inRange(hsv, lo, hi)

            if 'hsv_lower2' in cfg:
                lo2  = np.array(cfg['hsv_lower2'], dtype=np.uint8)
                hi2  = np.array(cfg['hsv_upper2'], dtype=np.uint8)
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo2, hi2))

            kernel = np.ones((5, 5), np.uint8)
            mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,   kernel)
            mask   = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue

            largest = max(contours, key=cv2.contourArea)
            area    = cv2.contourArea(largest)
            if area < cfg.get('min_area', 400):
                continue

            conf = min(area / (total_pixels * 0.5), 1.0)
            if conf > best['conf']:
                box             = cv2.boundingRect(largest)
                x, y, bw, bh   = box
                best = {
                    'name':   name,
                    'conf':   conf,
                    'center': (x + bw // 2, y + bh // 2),
                    'box':    box,
                }

        return frame, best

    # ─────────────────────────────────────────────────────────────────────────
    # METHOD A — RGB-D BACK-PROJECTION
    # ─────────────────────────────────────────────────────────────────────────

    def _depth_to_map(self, cx_px, cy_px, depth_frame, header):
        """
        Convert pixel + depth → camera-frame 3-D point → map-frame (x, y).

        Handles both float32 metres (Gazebo) and uint16 millimetres (real D435).
        Returns (nan, nan) on any failure so the state machine can fall back
        to robot pose.
        """
        NAN = float('nan')

        if self._fx is None:
            return NAN, NAN

        try:
            # ── Read depth value ─────────────────────────────────────────────
            raw = float(depth_frame[int(cy_px), int(cx_px)])

            # Gazebo publishes float32 metres; real D435 uint16 millimetres
            d = raw / 1000.0 if raw > 100.0 else raw

            # If pixel depth is zero/invalid, try median over a 11×11 patch
            if not math.isfinite(d) or d <= 0.01 or d > 12.0:
                patch = depth_frame[
                    max(0, int(cy_px) - 5): int(cy_px) + 6,
                    max(0, int(cx_px) - 5): int(cx_px) + 6,
                ].astype(np.float32)
                patch[patch == 0] = np.nan
                raw_median = float(np.nanmedian(patch))
                d = raw_median / 1000.0 if raw_median > 100.0 else raw_median
                if not math.isfinite(d) or d <= 0.01:
                    return NAN, NAN

            # ── Back-project to camera frame ─────────────────────────────────
            X_cam = (cx_px - self._cx_intr) * d / self._fx
            Y_cam = (cy_px - self._cy_intr) * d / self._fy
            Z_cam = d

            # ── Transform camera_color_optical_frame → map ───────────────────
            pt              = PointStamped()
            pt.header       = header
            # RealSense optical frame — adjust to 'camera_link' if needed
            pt.header.frame_id = 'camera_color_optical_frame'
            pt.point.x      = X_cam
            pt.point.y      = Y_cam
            pt.point.z      = Z_cam

            pt_map = self.tf_buffer.transform(
                pt, 'map',
                timeout=rclpy.duration.Duration(seconds=0.15)
            )
            return pt_map.point.x, pt_map.point.y

        except Exception as e:
            self.get_logger().warn(f'[MethodA] RGB-D localization failed: {e}')
            return NAN, NAN

    # ─────────────────────────────────────────────────────────────────────────
    # METHOD B — LIDAR RAY-CASTING
    # ─────────────────────────────────────────────────────────────────────────

    def _lidar_raycast_to_map(self, cx_px: int):
        """
        Estimate target map coordinates from camera bearing + nearest LiDAR ray.

        Steps:
          1. Convert pixel offset → horizontal bearing relative to robot forward.
          2. Find the LiDAR range at that bearing (median over ±3 beams).
          3. Project range along world-frame bearing using robot TF pose.

        Returns (nan, nan) on any failure.
        """
        NAN = float('nan')

        if self._latest_scan is None:
            self.get_logger().warn('[MethodB] No LiDAR scan received yet.')
            return NAN, NAN

        scan = self._latest_scan

        # ── Step 1: pixel offset → bearing ──────────────────────────────────
        # Normalised offset: −0.5 (far left) … +0.5 (far right)
        dx_norm    = (cx_px - self._img_width / 2.0) / self._img_width
        # Camera convention: positive dx = rightward in image = clockwise
        bearing_cam = dx_norm * self.camera_hfov   # radians

        # LiDAR convention on ROSbot: 0 = forward, positive = CCW (left)
        # Negate to align with camera's CW-positive convention
        bearing_lidar = -bearing_cam

        # ── Step 2: LiDAR range lookup ───────────────────────────────────────
        if not (scan.angle_min <= bearing_lidar <= scan.angle_max):
            self.get_logger().warn(
                f'[MethodB] Bearing {math.degrees(bearing_lidar):.1f}° '
                f'outside scan range '
                f'[{math.degrees(scan.angle_min):.1f}°, '
                f'{math.degrees(scan.angle_max):.1f}°]'
            )
            return NAN, NAN

        idx     = int((bearing_lidar - scan.angle_min) / scan.angle_increment)
        idx     = max(0, min(idx, len(scan.ranges) - 1))

        # Median over ±3 neighbouring beams for robustness against noise
        window  = 3
        beams   = scan.ranges[max(0, idx - window): idx + window + 1]
        valid   = [r for r in beams
                   if math.isfinite(r)
                   and scan.range_min < r < scan.range_max]
        if not valid:
            self.get_logger().warn('[MethodB] All nearby LiDAR beams are invalid.')
            return NAN, NAN

        range_m = float(np.median(valid))

        # ── Step 3: project into map frame ───────────────────────────────────
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.15)
            )
            rx   = tf.transform.translation.x
            ry   = tf.transform.translation.y
            q    = tf.transform.rotation
            ryaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
        except Exception as e:
            self.get_logger().warn(f'[MethodB] TF lookup failed: {e}')
            return NAN, NAN

        world_angle = ryaw + bearing_lidar
        target_x    = rx + range_m * math.cos(world_angle)
        target_y    = ry + range_m * math.sin(world_angle)

        return target_x, target_y

    # ─────────────────────────────────────────────────────────────────────────
    # PUBLISH HELPERS
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_result(self, name: str, conf: float,
                         map_x: float, map_y: float):
        if name and conf > 0.01:
            msg      = String()
            msg.data = f'{name}:{conf:.4f}:{map_x:.4f}:{map_y:.4f}'
            self.result_pub.publish(msg)

    def _publish_debug(self, frame, box, name, conf, map_coords):
        if box is not None:
            x, y, bw, bh = box
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            label = f'{name} ({conf:.2f})'
            if map_coords:
                mx, my = map_coords
                if math.isfinite(mx) and math.isfinite(my):
                    label += f' [{mx:.2f},{my:.2f}]'
            cv2.putText(frame, label, (x, max(y - 8, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        try:
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            )
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    rclpy.spin(node)


if __name__ == '__main__':
    main()