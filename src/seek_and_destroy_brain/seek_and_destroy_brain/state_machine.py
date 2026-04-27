#!/usr/bin/env python3
"""
state_machine.py  -  Project Seek & Destroy
Architecture: explore_lite (frontier-based) + Nav2 go_home + color_detector

State flow:
  IDLE ──► EXPLORING ──► HOME ──► END ──► IDLE (or quit)
                  │
                  └──► (incidental finds logged but state unchanged)

FIXES applied:
  1. CONFIDENCE_THRESHOLD raised to 0.35 → marker placed closer to actual target
  2. Detection allowed during HOME state → robot logs targets on way back
  3. Markers overlaid on saved map PNG using OpenCV
"""

import os
import glob
import time
import threading
from typing import Tuple
import yaml

import rclpy
from rclpy.node import Node

# Message types
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker

# TF2
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# OpenCV for map overlay
import cv2
import numpy as np


# ──────────────────────────────────────────────────────────────────────────────
# Colour palette: substring of target name → (R, G, B) as floats 0–1
# ──────────────────────────────────────────────────────────────────────────────
COLOUR_MAP = {
    'red':    (1.0, 0.1, 0.1),
    'blue':   (0.1, 0.3, 1.0),
    'green':  (0.1, 0.9, 0.1),
    'yellow': (1.0, 0.9, 0.0),
    'orange': (1.0, 0.5, 0.0),
    'purple': (0.6, 0.1, 0.9),
    'white':  (1.0, 1.0, 1.0),
    'black':  (0.1, 0.1, 0.1),
}
DEFAULT_COLOUR = (0.8, 0.8, 0.8)   # grey fallback


def colour_for(name: str):
    """Return (r, g, b) based on colour keywords in the target name."""
    low = name.lower()
    for keyword, rgb in COLOUR_MAP.items():
        if keyword in low:
            return rgb
    return DEFAULT_COLOUR


def format_target(raw: str) -> str:
    """
    Normalise user input to match detector output format.
    e.g. '  Red Cylinder  ' → 'redcylinder'
    """
    return raw.strip().lower().replace(' ', '')


# ──────────────────────────────────────────────────────────────────────────────

class StateMachine(Node):

    # FIX 1: Raised from 0.10 → 0.35 so marker is placed much closer to target
    CONFIDENCE_THRESHOLD = 0.35

    def __init__(self):
        super().__init__('state_machine')

        # ── State ────────────────────────────────────────────────────────────
        self.state          = 'IDLE'
        self.target_name    = None
        self.found_location = None

        # Tracks incidental targets already marked so we don't spam markers.
        self._marked_targets: dict = {}

        # Marker ID counter
        self._marker_id = 0

        # ── TF2 ──────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Publishers ───────────────────────────────────────────────────────
        self.cmd_pub    = self.create_publisher(String,  '/mission/command',      10)
        self.resume_pub = self.create_publisher(Bool,    '/explore/resume',       10)
        self.marker_pub = self.create_publisher(Marker,  '/visualization_marker', 10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(String, '/detection/result', self.on_detection, 10)
        self.create_subscription(String, '/mission/status',   self.on_status,    10)

        # ── Banner ───────────────────────────────────────────────────────────
        print('\n' + '=' * 40)
        print('   PROJECT SEEK & DESTROY')
        print('   Frontier-based exploration (explore_lite)')
        print('   Robot ready at Home (0, 0)')
        print('=' * 40)

        self.explore_start_time = None
        self.create_timer(1.0, self._check_timeout)

        ui_thread = threading.Thread(target=self._run_ui, daemon=True)
        ui_thread.start()

        self.declare_parameter('targets_file', '')
        targets_file = self.get_parameter('targets_file').value

        self.valid_targets = []

        if not targets_file or not os.path.exists(targets_file):
            self.get_logger().error(f'targets.yaml not found at: {targets_file}')
        else:
            try:
                with open(targets_file, 'r') as f:
                    config = yaml.safe_load(f)
                    self.valid_targets = list(config['targets'].keys())
                    self.get_logger().info(
                        f'Loaded {len(self.valid_targets)} valid targets: {self.valid_targets}')
            except Exception as e:
                self.get_logger().error(f'Could not parse targets.yaml: {e}')

        self.declare_parameter('map_save_dir', '')
        self.maps_dir = self.get_parameter('map_save_dir').value

    # ─────────────────────────────────────────────────────────────────────────
    # CALLBACKS
    # ─────────────────────────────────────────────────────────────────────────

    def on_detection(self, msg: String):
        """
        Fired every frame the camera detector publishes a result.
        Format: "target_name:confidence"

        FIX 2: Now also processes detections during HOME state so the robot
        logs incidental targets on its way back to base.
        """
        # FIX 2: Allow 'HOME' in addition to 'EXPLORING'
        if self.state not in ('EXPLORING', 'HOME'):
            return

        parts = msg.data.split(':')
        if len(parts) != 2:
            return

        try:
            detected_name = parts[0].strip()
            confidence    = float(parts[1])
        except ValueError:
            return

        x, y = self._get_robot_pose()

        # ── Primary target found (only act on it during EXPLORING) ───────────
        if (detected_name == self.target_name
                and confidence >= self.CONFIDENCE_THRESHOLD
                and self.state == 'EXPLORING'):
            self.get_logger().info(
                f'PRIMARY TARGET "{detected_name}" detected '
                f'(conf={confidence:.3f}) at ({x:.2f}, {y:.2f})'
            )
            self._publish_marker(detected_name, x, y)
            self.found_location = (x, y)

            self._set_explore(False)
            self._publish_cmd('GO_HOME')
            self._transition('HOME')
            return

        # ── Incidental target (any state where detection is allowed) ─────────
        if confidence >= self.CONFIDENCE_THRESHOLD:
            if detected_name not in self._marked_targets:
                self._marked_targets[detected_name] = (x, y)
                self._publish_marker(detected_name, x, y)
                self.get_logger().info(
                    f'Incidental target "{detected_name}" noted '
                    f'(conf={confidence:.3f}) at ({x:.2f}, {y:.2f})'
                )
                state_label = 'on way home' if self.state == 'HOME' else 'while exploring'
                print(
                    f'\n  [INFO] Incidental find ({state_label}): '
                    f'"{detected_name}" at ({x:.2f}, {y:.2f})'
                )

    def on_status(self, msg: String):
        """Called by go_home node once Nav2 has reached its goal."""
        if msg.data == 'ARRIVED_HOME':
            if self.state == 'HOME':
                self._transition('SAVING_MAP')
            elif self.state == 'HOME_FAILED':
                self._transition('END_FAILED')

        elif msg.data == 'ARRIVED_TARGET':
            if self.state == 'NAVIGATING_TO_KNOWN':
                print(f'\n  [INFO] Reached known target! Now heading back to base.')
                self._publish_cmd('GO_HOME')
                self._transition('HOME')

        elif msg.data == 'MAP_SAVED':
            if self.state == 'SAVING_MAP':
                print('  ✅ Map successfully saved!')
                # FIX 3: Overlay markers on the saved map PNG
                self._overlay_markers_on_saved_map()
                self._transition('END')

    def _check_timeout(self):
        """If exploring takes longer than 5 minutes, go home."""
        if self.state == 'EXPLORING' and self.explore_start_time:
            elapsed = time.time() - self.explore_start_time
            if elapsed > 300.0:
                print(f'\n  [TIMEOUT] Explored for 5 minutes without finding "{self.target_name}".')
                self._set_explore(False)
                self._publish_cmd('GO_HOME')
                self._transition('HOME_FAILED')

    # ─────────────────────────────────────────────────────────────────────────
    # STATE MACHINE
    # ─────────────────────────────────────────────────────────────────────────

    def _transition(self, new_state: str):
        old = self.state
        self.state = new_state
        print(f'\n  [STATE] {old} → {new_state}')

        if new_state == 'EXPLORING':
            self.explore_start_time = time.time()
            self._set_explore(True)

        elif new_state == 'HOME':
            pass  # GO_HOME already published before calling _transition

        elif new_state == 'SAVING_MAP':
            self._request_map_save()

        elif new_state == 'END':
            loc_str = (
                f'({self.found_location[0]:.2f}, {self.found_location[1]:.2f})'
                if self.found_location else 'unknown'
            )
            print(f'\n  ✅ TARGET FOUND: {self.target_name}')
            print(f'     Location: {loc_str}')
            print('\n  🏠 Robot is back at Home.')

    # ─────────────────────────────────────────────────────────────────────────
    # HELPER – EXPLORATION CONTROL
    # ─────────────────────────────────────────────────────────────────────────

    def _set_explore(self, resume: bool):
        msg = Bool()
        msg.data = resume
        self.resume_pub.publish(msg)
        action = 'RESUMED' if resume else 'PAUSED'
        self.get_logger().info(f'explore_lite {action}')

    # ─────────────────────────────────────────────────────────────────────────
    # HELPER – MISSION COMMAND
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_cmd(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'Mission command: {cmd}')

    # ─────────────────────────────────────────────────────────────────────────
    # HELPER – TF2 POSE LOOKUP
    # ─────────────────────────────────────────────────────────────────────────

    def _get_robot_pose(self) -> Tuple[float, float]:
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            return x, y
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return 0.0, 0.0

    # ─────────────────────────────────────────────────────────────────────────
    # HELPER – RVIZ MARKER
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_marker(self, target_name: str, x: float, y: float):
        r, g, b = colour_for(target_name)

        marker = Marker()
        marker.header.frame_id    = 'map'
        marker.header.stamp       = self.get_clock().now().to_msg()
        marker.ns                 = 'seek_destroy'
        marker.id                 = self._marker_id
        marker.type               = Marker.SPHERE
        marker.action             = Marker.ADD
        marker.pose.position      = Point(x=x, y=y, z=0.3)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.9
        marker.lifetime = Duration(sec=0, nanosec=0)

        self.marker_pub.publish(marker)
        self._marker_id += 1

        self.get_logger().info(
            f'Marker #{marker.id} published for "{target_name}" '
            f'at ({x:.2f}, {y:.2f})'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # HELPER – MAP SAVE
    # ─────────────────────────────────────────────────────────────────────────

    def _request_map_save(self):
        print('  Asking base station to save map...')
        os.makedirs(self.maps_dir, exist_ok=True)
        existing = glob.glob(os.path.join(self.maps_dir, 'map*.yaml'))
        next_num = len(existing) + 1
        map_path = os.path.join(self.maps_dir, f'map{next_num}')
        self._publish_cmd(f'SAVE_MAP:{map_path}')

    # ─────────────────────────────────────────────────────────────────────────
    # FIX 3 – OVERLAY MARKERS ON SAVED MAP IMAGE
    # ─────────────────────────────────────────────────────────────────────────

    def _overlay_markers_on_saved_map(self):
        """
        After map is saved, draw colored circles on the PNG at each
        target location so the saved map shows where targets were found.
        """
        try:
            existing = glob.glob(os.path.join(self.maps_dir, 'map*.png'))
            if not existing:
                self.get_logger().warn('No saved map PNG found to overlay markers on.')
                return

            latest_png = sorted(existing)[-1]
            latest_yaml = latest_png.replace('.png', '.yaml')

            if not os.path.exists(latest_yaml):
                self.get_logger().warn(f'Map YAML not found: {latest_yaml}')
                return

            with open(latest_yaml, 'r') as f:
                map_meta = yaml.safe_load(f)

            resolution = map_meta.get('resolution', 0.05)
            origin     = map_meta.get('origin', [0.0, 0.0, 0.0])

            img = cv2.imread(latest_png)
            if img is None:
                self.get_logger().warn(f'Could not read map image: {latest_png}')
                return

            h, w = img.shape[:2]

            # Collect all marker positions
            all_markers = {}
            if self.found_location and self.target_name:
                all_markers[self.target_name] = self.found_location
            all_markers.update(self._marked_targets)

            for name, (mx, my) in all_markers.items():
                # Convert map coords → pixel coords
                px = int((mx - origin[0]) / resolution)
                py = int(h - (my - origin[1]) / resolution)

                if 0 <= px < w and 0 <= py < h:
                    r, g, b = colour_for(name)
                    color_bgr = (int(b * 255), int(g * 255), int(r * 255))
                    cv2.circle(img, (px, py), 10, color_bgr, -1)
                    cv2.circle(img, (px, py), 12, (0, 0, 0), 2)  # black outline
                    cv2.putText(img, name, (px + 14, py + 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_bgr, 1)

            cv2.imwrite(latest_png, img)
            print(f'  🗺️  Markers overlaid on saved map: {latest_png}')
            self.get_logger().info(f'Map with markers saved: {latest_png}')

        except Exception as e:
            self.get_logger().error(f'Failed to overlay markers on map: {e}')

    # ─────────────────────────────────────────────────────────────────────────
    # TERMINAL UI
    # ─────────────────────────────────────────────────────────────────────────

    def _run_ui(self):
        time.sleep(1.0)

        while rclpy.ok():

            if self.state == 'IDLE':
                print('\n' + '-' * 40)
                print('  Available targets: Red Cylinder, Blue Box, Green Cylinder')
                raw = input('  Which target are you looking for?\n  > ').strip()

                if not raw:
                    print('  ⚠️  Please type a target name.')
                    continue

                self.target_name = format_target(raw)

                if self.target_name in self._marked_targets:
                    x, y = self._marked_targets[self.target_name]
                    print(f'\n  🎯 TARGET ALREADY KNOWN! Driving to ({x:.2f}, {y:.2f})...')
                    self.found_location = (x, y)
                    self._publish_cmd(f'GO_TO:{x},{y}')
                    self._transition('NAVIGATING_TO_KNOWN')
                else:
                    print(f'\n  Target not found yet. Starting exploration for: "{self.target_name}"')
                    self.found_location = None
                    self._transition('EXPLORING')

            elif self.state in ('END', 'END_FAILED'):
                print('\n' + '-' * 40)
                if self.state == 'END_FAILED':
                    print('  ⚠️ MISSION FAILED: Target was not found.')

                answer = input(
                    '  Search for another target or quit?\n'
                    '  [s = search again  /  q = quit]\n  > '
                ).strip().lower()

                if answer == 'q':
                    print('\n  Goodbye! Mission complete. 🚀')
                    self._set_explore(False)
                    rclpy.shutdown()
                    break

                elif answer == 's':
                    self.target_name    = None
                    self.found_location = None
                    self.state          = 'IDLE'

                else:
                    print('  Type "s" to search again or "q" to quit.')

            else:
                time.sleep(0.5)


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()