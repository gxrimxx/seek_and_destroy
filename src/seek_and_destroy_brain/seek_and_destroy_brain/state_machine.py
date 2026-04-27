#!/usr/bin/env python3
"""
state_machine.py  -  Project Seek & Destroy
Architecture: explore_lite (frontier-based) + Nav2 go_home + color_detector

State flow:
  IDLE ──► EXPLORING ──► HOME ──► END ──► IDLE (or quit)
                  │
                  └──► (incidental finds logged but state unchanged)
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
    # Simply strip whitespace and convert to lowercase
    return raw.strip().lower().replace(' ', '')


# ──────────────────────────────────────────────────────────────────────────────

class StateMachine(Node):

    CONFIDENCE_THRESHOLD = 0.10   # minimum confidence to accept a primary target

    def __init__(self):
        super().__init__('state_machine')

        # ── State ────────────────────────────────────────────────────────────
        self.state          = 'IDLE'
        self.target_name    = None          # formatted with underscores
        self.found_location = None

        # Tracks incidental targets already marked so we don't spam markers.
        # key: target_name (str)  value: (x, y) tuple at time of first detection
        self._marked_targets: dict = {}

        # Marker ID counter (each published marker needs a unique int ID)
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
        self.create_timer(1.0, self._check_timeout) # Checks every 1 second

        # Run the terminal UI in a daemon thread so rclpy.spin() runs freely
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
                    self.get_logger().info(f'Loaded {len(self.valid_targets)} valid targets: {self.valid_targets}')
            except Exception as e:
                self.get_logger().error(f"Could not parse targets.yaml: {e}")
        
        self.declare_parameter('map_save_dir', '')
        self.maps_dir = self.get_parameter('map_save_dir').value

    # ─────────────────────────────────────────────────────────────────────────
    # CALLBACKS
    # ─────────────────────────────────────────────────────────────────────────

    def on_detection(self, msg: String):
        """
        Fired every frame the camera detector publishes a result.
        Format: "target_name:confidence"   (name already uses underscores)
        """
        if self.state not in ('EXPLORING',):
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

        # ── Primary target found ─────────────────────────────────────────────
        if detected_name == self.target_name and confidence >= self.CONFIDENCE_THRESHOLD:
            self.get_logger().info(
                f'PRIMARY TARGET "{detected_name}" detected '
                f'(conf={confidence:.3f}) at ({x:.2f}, {y:.2f})'
            )
            self._publish_marker(detected_name, x, y)
            self.found_location = (x, y)

            # Freeze explorer immediately
            self._set_explore(False)

            # Send robot home
            self._publish_cmd('GO_HOME')
            self._transition('HOME')
            return

        # ── Incidental target (not what user requested) ──────────────────────
        if confidence >= self.CONFIDENCE_THRESHOLD:
            if detected_name not in self._marked_targets:
                self._marked_targets[detected_name] = (x, y)
                self._publish_marker(detected_name, x, y)
                self.get_logger().info(
                    f'Incidental target "{detected_name}" noted '
                    f'(conf={confidence:.3f}) at ({x:.2f}, {y:.2f})'
                )
                print(
                    f'\n  [INFO] Incidental find: "{detected_name}" '
                    f'at ({x:.2f}, {y:.2f})  — continuing search...'
                )

    def on_status(self, msg: String):
        """Called by go_home node once Nav2 has reached (0, 0)."""
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
                print('  ✅ Map successfully saved by base station!')
                self._transition('END')  # Now we trigger the final UI prompt!
    
    def _check_timeout(self):
        """If exploring takes longer than 3 minutes (180 seconds), give up."""
        if self.state == 'EXPLORING' and self.explore_start_time:
            elapsed = time.time() - self.explore_start_time
            if elapsed > 600.0:  # 5 minutes
                print(f'\n  [TIMEOUT] Explored for 5 minutes but could not find "{self.target_name}".')
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
            # Clear per-mission state; keep _marked_targets so we don't
            # re-mark incidentals found in earlier missions this session.
            # self._marked_targets.clear()
            self.explore_start_time = time.time()  # <--- Start the clock!
            self._set_explore(True)

        elif new_state == 'HOME':
            # Robot is navigating home — nothing else to do here.
            # We already published GO_HOME before calling _transition.
            pass

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
            # self._save_map()
            # The UI thread will now detect state == 'END' and prompt the user.

    # ─────────────────────────────────────────────────────────────────────────
    # HELPER – EXPLORATION CONTROL
    # ─────────────────────────────────────────────────────────────────────────

    def _set_explore(self, resume: bool):
        """Publish True to start/resume explore_lite, False to pause it."""
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
        """
        Look up the current robot X, Y in the map frame.
        Returns (0.0, 0.0) and logs a warning if the transform is unavailable.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),              # latest available transform
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
        """
        Drop a coloured sphere marker on the map at (x, y).

        Colour is determined from keywords in the target name.
        Each call gets a unique marker ID so markers accumulate in RViz.
        """
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
        marker.color.a = 0.9          # nearly opaque

        # Keep the marker visible indefinitely (0 = forever)
        marker.lifetime = Duration(sec=0, nanosec=0)

        self.marker_pub.publish(marker)
        self._marker_id += 1

        self.get_logger().info(
            f'Marker #{marker.id} published for "{target_name}" '
            f'at ({x:.2f}, {y:.2f})  colour=({r:.1f},{g:.1f},{b:.1f})'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # HELPER – MAP SAVE
    # ─────────────────────────────────────────────────────────────────────────

    def _save_map(self):
        print('  Saving map...')

        map_dir = os.path.expanduser('~/seek_destroy_ws/src/saved_maps')
        os.makedirs(map_dir, exist_ok=True)

        existing = glob.glob(os.path.join(map_dir, 'map*.yaml'))
        next_num = len(existing) + 1
        map_name = f'map{next_num}'
        map_path = os.path.join(map_dir, map_name)

        cmd = f'ros2 run nav2_map_server map_saver_cli -f {map_path}'
        ret = os.system(cmd)

        if ret == 0:
            print(f'  ✅ Map saved as {map_name}  ({map_path}.yaml)')
        else:
            print(f'  ⚠️  map_saver_cli exited with code {ret}. '
                  'Check that nav2_map_server is installed and the map server is running.')
    
    def _request_map_save(self):
        print('  Asking base station to save map...')

        os.makedirs(self.maps_dir, exist_ok=True)
        existing = glob.glob(os.path.join(self.maps_dir, 'map*.yaml'))
        next_num = len(existing) + 1
        map_path = os.path.join(self.maps_dir, f'map{next_num}')
        
        # Send the command to Terminal 1
        self._publish_cmd(f'SAVE_MAP:{map_path}')

    # ─────────────────────────────────────────────────────────────────────────
    # TERMINAL UI  (runs in a background daemon thread)
    # ─────────────────────────────────────────────────────────────────────────

    def _run_ui(self):
        """Block on user input; all ROS work happens on the spin thread."""
        time.sleep(1.0)   # let the node finish initialising

        while rclpy.ok():

            # ── IDLE: ask for a target ────────────────────────────────────────
            if self.state == 'IDLE':
                print('\n' + '-' * 40)
                print('  Available targets: Red Cylinder, Blue Box, Green Cylinder')
                raw = input('  Which target are you looking for?\n  > ').strip()

                if not raw:
                    print('  ⚠️  Please type a target name.')
                    continue

                self.target_name    = format_target(raw)

                # Check memory first
                if self.target_name in self._marked_targets:
                    x, y = self._marked_targets[self.target_name]
                    print(f'\n  🎯 TARGET ALREADY KNOWN! Driving to ({x:.2f}, {y:.2f})...')
                    self.target_name = self.target_name
                    self.found_location = (x, y)
                    
                    # Instead of exploring, we publish a specific goal
                    # Publish the GO_TO command using the existing publisher
                    self._publish_cmd(f'GO_TO:{x},{y}') 
                    self._transition('NAVIGATING_TO_KNOWN')
                else:
                    print(f'\n  Target not found yet. Starting exploration for: "{self.target_name}"')
                    self.target_name = self.target_name
                    self.found_location = None
                    self._transition('EXPLORING')

            # ── END: map saved, offer another round ──────────────────────────
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
                    # Don't call _transition here — just reset so the IDLE
                    # branch above picks it up cleanly on the next iteration.

                else:
                    print('  Type "s" to search again or "q" to quit.')

            # ── Any other state: robot is busy, just wait ─────────────────────
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