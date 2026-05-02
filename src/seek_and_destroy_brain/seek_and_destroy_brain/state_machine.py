#!/usr/bin/env python3
"""
state_machine.py  -  Project Seek & Destroy
Joyce's full architecture + Garima's GUI integration (/mission/set_target, /mission/state)
"""




import os
import glob
import time
import math
import threading
from typing import Dict, List, Optional, Set, Tuple
import yaml




import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient




from std_msgs.msg       import String, Bool
from geometry_msgs.msg  import Point
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg       import OccupancyGrid
from nav2_msgs.action   import Spin




import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException




import cv2
import numpy as np




COLOUR_MAP = {
    'red':    (1.0, 0.1, 0.1),
    'blue':   (0.1, 0.3, 1.0),
    'green':  (0.1, 0.9, 0.1),
    'yellow': (1.0, 0.9, 0.0),
    'orange': (1.0, 0.5, 0.0),
    'purple': (0.6, 0.1, 0.9),
    'white':  (0.95, 0.95, 0.95),
    'black':  (0.1, 0.1, 0.1),
}
DEFAULT_COLOUR = (0.7, 0.7, 0.7)




LEGAL_TRANSITIONS: Dict[str, Set[str]] = {
    'IDLE':                 {'SWEEPING', 'NAVIGATING_TO_KNOWN'},
    'SWEEPING':             {'EXPLORING', 'HOMING'},
    'EXPLORING':            {'INVESTIGATING', 'ROAMING', 'HOMING'},
    'INVESTIGATING':        {'EXPLORING', 'HOMING'},
    'ROAMING':              {'HOMING', 'HOME_FAILED', 'INVESTIGATING'},  # ← add INVESTIGATING
    'NAVIGATING_TO_KNOWN':  {'HOMING', 'IDLE'},
    'HOMING':               {'SAVING_MAP'},
    'SAVING_MAP':           {'END'},
    'HOME_FAILED':          {'IDLE'},
    'END':                  {'IDLE'},
}








def colour_for(name: str) -> Tuple[float, float, float]:
    for kw, rgb in COLOUR_MAP.items():
        if kw in name.lower():
            return rgb
    return DEFAULT_COLOUR








def format_target(raw: str) -> str:
    return raw.strip().lower().replace(' ', '')








class StateMachine(Node):




    CONF_DECAY        = 0.80
    CONF_INVESTIGATE  = 0.30
    CONF_CONFIRM      = 0.10
    CONF_REJECT_FLOOR = 0.03




    ROAM_MAX_WAYPOINTS    = 25
    ROAM_STANDOFF_METRES  = 1.5
    FRONTIER_TIMEOUT_SECS = 15.0
    FRONTIER_MIN_EXPLORE  = 30.0




    def __init__(self):
        super().__init__('state_machine')




        self.declare_parameter('targets_file', '')
        self.declare_parameter('map_save_dir',
            os.path.expanduser('~/seek_destroy_ws/src/saved_maps'))
        targets_file  = self.get_parameter('targets_file').value
        self.maps_dir = self.get_parameter('map_save_dir').value




        self.valid_targets: List[str] = []
        if targets_file and os.path.exists(targets_file):
            try:
                with open(targets_file, 'r') as f:
                    cfg = yaml.safe_load(f)
                self.valid_targets = list(cfg['targets'].keys())
                self.get_logger().info(f'Valid targets: {self.valid_targets}')
            except Exception as e:
                self.get_logger().error(f'Cannot parse targets.yaml: {e}')




        self.state                    = 'IDLE'
        self.target_name              = None
        self.found_location           = None
        self._confirmed_targets: Dict[str, Tuple[float, float]]  = {}
        self._incidental_targets: Dict[str, Tuple[float, float]] = {}
        self._conf_accum: Dict[str, float] = {}
        self._invest_name             = None
        self._invest_pos              = None
        self._roam_waypoints: List[Tuple[float, float]] = []
        self._roam_index              = 0
        self._latest_map              = None
        self._last_frontier_active    = time.time()
        self._explore_start_time      = 0.0
        self._sweep_goal_handle       = None
        self._sweep_confirmed_target = None  # (name, x, y) recorded during sweep
        self._sweep_incidentals: Dict[str, Tuple[float, float]] = {}
        self._marker_id               = 0
        self._lock                    = threading.Lock()




        # GUI pending target
        self._gui_target_pending      = None
        self._gui_quit_pending        = False




        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._spin_client = ActionClient(self, Spin, '/spin')




        self.cmd_pub    = self.create_publisher(String, '/mission/command',      10)
        self.resume_pub = self.create_publisher(Bool,   '/explore/resume',       10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        # GUI: publish state so dashboard can read it
        self.state_pub  = self.create_publisher(String, '/mission/state',        10)




        self.create_subscription(String,        '/detection/result',   self.on_detection,    10)
        self.create_subscription(String,        '/mission/status',     self.on_status,       10)
        self.create_subscription(OccupancyGrid, '/map',                self._on_map,         1)
        self.create_subscription(MarkerArray,   '/explore/frontiers',  self._on_frontiers,   1)
        # GUI: accept target from dashboard
        self.create_subscription(String,        '/mission/set_target', self.on_set_target,   10)




        self._pause_timer = self.create_timer(1.0, self._send_initial_pause)
        self.create_timer(5.0,  self._check_frontier_exhaustion)
        self.create_timer(0.5,  self._publish_state)
        # Keeps republishing Bool(True) every 2 s while EXPLORING so explore_lite
        # never misses it regardless of subscription timing or QoS.
        self.create_timer(2.0, self._maintain_explore_resume)




        print('\n' + '=' * 40)
        print('   PROJECT SEEK & DESTROY')
        print('   IDLE→SWEEPING→EXPLORING→(INVESTIGATING)→HOMING→END')
        print('   GUI: publish target to /mission/set_target')
        print('=' * 40)




        threading.Thread(target=self._run_ui, daemon=True).start()




    # ── Add this new method: ──────────────────────────────────────────────────────
    def _maintain_explore_resume(self):
        with self._lock:
            if self.state == 'EXPLORING':
                msg      = Bool()
                msg.data = True
                self.resume_pub.publish(msg)




    # ── GUI integration ───────────────────────────────────────────────────────




    def on_set_target(self, msg: String):
        """
        Called from the ROS spin thread when the GUI publishes a target.
        Handles the full mission start directly — does NOT use _gui_target_pending
        because the UI thread is blocked at input() and would never check it.
        """
        data = msg.data.strip()
        self.get_logger().info(f'GUI set_target received: "{data}"')




        if data == 'QUIT':
            self._gui_quit_pending = True
            return




        if data == 'SEARCH_AGAIN':
            with self._lock:
                if self.state in ('END', 'HOME_FAILED'):
                    self.target_name    = None
                    self.found_location = None
                    self.state          = 'IDLE'
                    print('\n  [GUI] Ready for new target.')
            return




        # Only accept a new target when genuinely IDLE
        with self._lock:
            if self.state != 'IDLE':
                self.get_logger().warn(
                    f'GUI target ignored — state is {self.state}, not IDLE'
                )
                return




        fmt = format_target(data)




        if self.valid_targets and fmt not in self.valid_targets:
            self.get_logger().warn(
                f'GUI target "{fmt}" not in valid list: {self.valid_targets}'
            )
            return




        # Memory hit — already found this target before
        if fmt in self._confirmed_targets:
            cx, cy = self._confirmed_targets[fmt]
            print(f'\n  [GUI] Already found! Driving to ({cx:.2f},{cy:.2f})...')
            with self._lock:
                self.target_name    = fmt
                self.found_location = (cx, cy)
            self._publish_cmd(f'GO_TO:{cx:.4f},{cy:.4f}')
            self._transition('NAVIGATING_TO_KNOWN')
            return




        # Fresh mission
        with self._lock:
            self.target_name    = fmt
            self.found_location = None
            self._conf_accum.clear()




        print(f'\n  [GUI] Starting mission for: "{fmt}"')
        self._transition('SWEEPING')




    def _publish_state(self):
        msg      = String()
        msg.data = self.state
        self.state_pub.publish(msg)




    # ── Startup ───────────────────────────────────────────────────────────────




    def _send_initial_pause(self):
        self._set_explore(False)
        self.destroy_timer(self._pause_timer)




    # ── Callbacks ─────────────────────────────────────────────────────────────
    def on_detection(self, msg: String):
        with self._lock:
            current_state  = self.state
            current_target = self.target_name




        active_states = ('SWEEPING', 'EXPLORING', 'INVESTIGATING', 'ROAMING')
        if current_state not in active_states:
            return




        parts = msg.data.split(':')
        if len(parts) != 4:
            return




        try:
            name  = parts[0].strip()
            conf  = float(parts[1])
            map_x = float(parts[2])
            map_y = float(parts[3])
        except ValueError:
            return




        if not (math.isfinite(map_x) and math.isfinite(map_y)):
            map_x, map_y = self._get_robot_pose()




        # ── SWEEPING: record only, never move ────────────────────────────────────
        # The sweep must complete fully before any navigation begins.
        # Sightings are stored and processed in _sweep_result().
        if current_state == 'SWEEPING':
            if name == current_target and conf >= self.CONF_CONFIRM:
                # Primary target clearly visible — record best position seen
                if self._sweep_confirmed_target is None:
                    self._sweep_confirmed_target = (name, map_x, map_y)
                    print(
                        f'\n  [SWEEP] Primary target "{name}" spotted '
                        f'(conf={conf:.3f}) — will act after sweep completes.'
                    )
                else:
                    # Update to higher-confidence reading if available
                    prev_conf = self._conf_accum.get(name, 0.0)
                    if conf > prev_conf:
                        self._sweep_confirmed_target = (name, map_x, map_y)




            elif conf >= self.CONF_CONFIRM and name not in self._sweep_incidentals:
                self._sweep_incidentals[name] = (map_x, map_y)
                print(
                    f'\n  [SWEEP] Incidental "{name}" noted at '
                    f'({map_x:.2f},{map_y:.2f}) — will mark after sweep.'
                )




            # Update accumulator for post-sweep decisions but take no action
            prev = self._conf_accum.get(name, 0.0)
            self._conf_accum[name] = prev * self.CONF_DECAY + conf
            return  # ← critical: never fall through to movement logic during sweep




        # ── Update confidence integrator (non-sweep states) ──────────────────────
        prev = self._conf_accum.get(name, 0.0)
        self._conf_accum[name] = prev * self.CONF_DECAY + conf
        accumulated = self._conf_accum[name]




        # ── INVESTIGATING: confirm or reject ─────────────────────────────────────
        if current_state == 'INVESTIGATING':
            if name != self._invest_name:
                return
            if conf >= self.CONF_CONFIRM:
                self.get_logger().info(
                    f'[INVESTIGATE] CONFIRMED "{name}" conf={conf:.3f}'
                )
                self._on_target_confirmed(name, map_x, map_y)
            elif accumulated < self.CONF_REJECT_FLOOR:
                self.get_logger().info(f'[INVESTIGATE] REJECTED "{name}"')
                print(f'\n  [INVESTIGATE] "{name}" false positive — resuming exploration.')
                self._conf_accum[name] = 0.0
                self._publish_cmd('CANCEL')
                self._transition('EXPLORING')
            return




        # ── Primary target ────────────────────────────────────────────────────────
        if name == current_target:
            if conf >= self.CONF_CONFIRM:
                self._on_target_confirmed(name, map_x, map_y)
                return
            if (accumulated >= self.CONF_INVESTIGATE and current_state in ('EXPLORING', 'ROAMING')):
                rx, ry   = self._get_robot_pose()
                approach = self._approach_point(
                    rx, ry, map_x, map_y, self.ROAM_STANDOFF_METRES
                )
                self._invest_name = name
                self._invest_pos  = (map_x, map_y)
                print(
                    f'\n  [INVESTIGATE] Possible "{name}" '
                    f'(score={accumulated:.3f}) — driving closer...'
                )
                self._set_explore(False)
                self._publish_cmd('CANCEL')       # cancels active roam/explore goal
                self._publish_cmd(
                    f'INVESTIGATE:{approach[0]:.4f},{approach[1]:.4f}'
                )
                self._transition('INVESTIGATING')
            return




        # ── Incidental ────────────────────────────────────────────────────────────
        if conf >= self.CONF_CONFIRM and name not in self._incidental_targets:
            self._incidental_targets[name] = (map_x, map_y)
            self._publish_marker(name, map_x, map_y)
            print(
                f'\n  [INCIDENTAL] "{name}" at ({map_x:.2f},{map_y:.2f}) '
                '— not the target, continuing.'
            )
   
    def on_status(self, msg: String):
        data = msg.data
        with self._lock:
            s = self.state




        if data == 'ARRIVED_HOME' and s == 'HOMING':
            self._transition('SAVING_MAP')
        elif data == 'ARRIVED_TARGET' and s == 'NAVIGATING_TO_KNOWN':
            print('\n  [INFO] Arrived at known target.')
            self._publish_cmd('GO_HOME')
            self._transition('HOMING')
        elif data == 'ARRIVED_INVESTIGATE' and s == 'INVESTIGATING':
            print('\n  [INVESTIGATE] Arrived — monitoring confidence...')
        elif data == 'ARRIVED_ROAM' and s == 'ROAMING':
            self._advance_roam_waypoint()
        elif data == 'MAP_SAVED' and s == 'SAVING_MAP':
            self._overlay_markers_on_map()
            self._transition('END')
        elif data == 'MAP_SAVE_FAILED' and s == 'SAVING_MAP':
            print('\n  Warning: Map save failed.')
            self._transition('END')
        elif data.startswith('NAV_FAILED:'):
            self._handle_nav_failure(data.split(':')[1], s)




    def _on_map(self, msg: OccupancyGrid):
        self._latest_map = msg




    def _on_frontiers(self, msg: MarkerArray):
        if msg.markers:
            self._last_frontier_active = time.time()




    def _check_frontier_exhaustion(self):
        with self._lock:
            if self.state != 'EXPLORING':
                return
        if (time.time() - self._last_frontier_active > self.FRONTIER_TIMEOUT_SECS
                and time.time() - self._explore_start_time > self.FRONTIER_MIN_EXPLORE):
            print('\n  [EXPLORE] Frontiers exhausted — switching to roaming...')
            self._transition('ROAMING')




    # ── State machine ─────────────────────────────────────────────────────────




    def _transition(self, new_state: str):
        with self._lock:
            old = self.state
            if new_state not in LEGAL_TRANSITIONS.get(old, set()):
                self.get_logger().error(f'ILLEGAL: {old} → {new_state} dropped')
                return
            self.state = new_state




        print(f'\n  [STATE] {old} → {new_state}')




        if new_state == 'SWEEPING':
            self._sweep_confirmed_target = None
            self._sweep_incidentals.clear()
            self._execute_sweep()
        elif new_state == 'EXPLORING':
            self._conf_accum.clear()
            self._explore_start_time   = time.time()
            self._last_frontier_active = time.time()
            self._set_explore(True)
        elif new_state == 'ROAMING':
            self._set_explore(False)
            self._start_roaming()
        elif new_state == 'HOMING':
            self._set_explore(False)
        elif new_state == 'SAVING_MAP':
            self._request_map_save()
        elif new_state == 'HOME_FAILED':
            print(f'\n  Target "{self.target_name}" not found.')
        elif new_state == 'END':
            loc = self.found_location
            loc_s = f'({loc[0]:.2f}, {loc[1]:.2f})' if loc else 'N/A'
            print(f'\n  TARGET FOUND: {self.target_name} at {loc_s}')
            print('  Robot is home.')
        elif new_state == 'IDLE':
            self._set_explore(False)




    # ── Feature 4: 360 sweep ──────────────────────────────────────────────────




    def _execute_sweep(self):
        print('\n  Performing 360 initial sweep...')
        if not self._spin_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Spin server unavailable — skipping to EXPLORING.')
            self._transition('EXPLORING')
            return
        goal = Spin.Goal()
        goal.target_yaw = 6.2832
        fut = self._spin_client.send_goal_async(goal, feedback_callback=lambda fb: None)
        fut.add_done_callback(self._sweep_goal_response)




    def _sweep_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self._transition('EXPLORING')
            return
        self._sweep_goal_handle = handle
        handle.get_result_async().add_done_callback(self._sweep_result)
   
    def _sweep_result(self, future):
        self._sweep_goal_handle = None
        print('\n  ✅  360° sweep complete.')




        # Give Nav2 a moment to settle after the spin action
        time.sleep(1.0)  # ← add this


        # Clear Nav2 costmaps so explore_lite gets a clean start
        # self._publish_cmd('CLEAR_COSTMAPS')  # ← add this




        # Publish incidentals noted during the sweep
        for inc_name, (inc_x, inc_y) in self._sweep_incidentals.items():
            if inc_name not in self._incidental_targets:
                self._incidental_targets[inc_name] = (inc_x, inc_y)
                self._publish_marker(inc_name, inc_x, inc_y)
                print(
                    f'\n  [SWEEP→INCIDENTAL] Marked "{inc_name}" '
                    f'at ({inc_x:.2f},{inc_y:.2f})'
                )
        self._sweep_incidentals.clear()




        # If primary target was clearly seen during sweep, confirm it now
        if self._sweep_confirmed_target is not None:
            name, x, y = self._sweep_confirmed_target
            self._sweep_confirmed_target = None
            print(f'\n  [SWEEP] Primary target "{name}" was seen — confirming...')
            self._on_target_confirmed(name, x, y)
            return




        # Normal path: sweep done, nothing confirmed → begin frontier exploration
        with self._lock:
            if self.state != 'SWEEPING':
                return  # state changed externally (shouldn't happen, but guard it)
        self._transition('EXPLORING')


        # ── NEW: wait for Nav2 BT to fully reset before resuming explore ──
        threading.Thread(target=self._delayed_explore_start, daemon=True).start()


    def _delayed_explore_start(self):
        import time
        self.get_logger().info('[SWEEP] Waiting for Nav2 BT to reset...')
       
        # Give the spin action result callback time to propagate through Nav2 internals
        time.sleep(2.0)
       
        # Then poll: try sending a dummy cancel to see if the action server responds
        # If it responds (even with "no goal"), Nav2 is alive and idle
        deadline = time.time() + 30.0
        while time.time() < deadline:
            if not self._spin_client.server_is_ready():
                time.sleep(0.5)
                continue
            # Spin server is back and idle — Nav2 BT has reset
            break
       
        with self._lock:
            if self.state != 'SWEEPING':
                return


        print('\n  [SWEEP] Nav2 ready — starting exploration.')
        self._transition('EXPLORING')


    # ── Feature 2: roaming ────────────────────────────────────────────────────




    def _start_roaming(self):
        if self._latest_map is None:
            self._transition('HOME_FAILED')
            return
        self._roam_waypoints = self._extract_obstacle_approach_points(self._latest_map)
        if not self._roam_waypoints:
            self._transition('HOME_FAILED')
            return
        print(f'\n  {len(self._roam_waypoints)} roam waypoints generated.')
        self._roam_index = 0
        self._send_next_roam_waypoint()




    def _extract_obstacle_approach_points(self, occ_grid: OccupancyGrid):
        info = occ_grid.info
        data = np.array(occ_grid.data, dtype=np.int8).reshape(info.height, info.width)
        occupied = (data == 100).astype(np.uint8)
        free     = (data == 0).astype(np.uint8)
        cpm      = max(1, int(round(1.0 / info.resolution)))
        near_k   = np.ones((cpm * 4 + 1, cpm * 4 + 1), np.uint8)
        near_obs = cv2.dilate(occupied, near_k)
        close_k  = np.ones((cpm + 1, cpm + 1), np.uint8)
        too_close = cv2.dilate(occupied, close_k)
        approach_mask  = free & near_obs & ~too_close
        approach_cells = np.argwhere(approach_mask)
        if len(approach_cells) == 0:
            return []
        step    = max(1, len(approach_cells) // self.ROAM_MAX_WAYPOINTS)
        sampled = approach_cells[::step][:self.ROAM_MAX_WAYPOINTS]
        origin  = info.origin.position
        points  = [(origin.x + (int(col) + 0.5) * info.resolution,
                    origin.y + (int(row) + 0.5) * info.resolution)
                   for row, col in sampled]
        rx, ry  = self._get_robot_pose()
        points.sort(key=lambda p: (p[0]-rx)**2 + (p[1]-ry)**2)
        return points




    def _send_next_roam_waypoint(self):
        if self._roam_index >= len(self._roam_waypoints):
            self._transition('HOME_FAILED')
            return
        x, y = self._roam_waypoints[self._roam_index]
        print(f'\n  [ROAM] Waypoint {self._roam_index+1}/{len(self._roam_waypoints)}: ({x:.2f},{y:.2f})')
        self._publish_cmd(f'ROAM_TO:{x:.4f},{y:.4f}')




    def _advance_roam_waypoint(self):
        with self._lock:
            if self.state != 'ROAMING':
                return
        self._roam_index += 1
        self._send_next_roam_waypoint()




    def _on_target_confirmed(self, name: str, map_x: float, map_y: float):
        with self._lock:
            self.found_location               = (map_x, map_y)
            self._confirmed_targets[name]     = (map_x, map_y)
            if self.state == 'SWEEPING' and self._sweep_goal_handle:
                self._sweep_goal_handle.cancel_goal_async()
                self._sweep_goal_handle = None
        self._publish_marker(name, map_x, map_y)
        print(f'\n  TARGET CONFIRMED: "{name}" at ({map_x:.2f},{map_y:.2f})')
        self._set_explore(False)
        self._publish_cmd('GO_HOME')
        self._transition('HOMING')




    def _handle_nav_failure(self, goal_type: str, current_state: str):
        if goal_type == 'INVESTIGATE' and current_state == 'INVESTIGATING':
            self._conf_accum[self._invest_name or ''] = 0.0
            self._transition('EXPLORING')
        elif goal_type == 'ROAM' and current_state == 'ROAMING':
            self._advance_roam_waypoint()
        elif goal_type == 'INVESTIGATE' and current_state == 'ROAMING':
            # Roam was cancelled to investigate but nav failed immediately
            self._conf_accum[self._invest_name or ''] = 0.0
            self._transition('ROAMING')  # ← restart roaming




    # ── Map save & overlay ────────────────────────────────────────────────────




    def _request_map_save(self):
        os.makedirs(self.maps_dir, exist_ok=True)
        existing = glob.glob(os.path.join(self.maps_dir, 'map*.yaml'))
        path     = os.path.join(self.maps_dir, f'map{len(existing)+1}')
        self._publish_cmd(f'SAVE_MAP:{path}')




    def _overlay_markers_on_map(self):
        try:
            pngs = sorted(glob.glob(os.path.join(self.maps_dir, 'map*.png')))
            if not pngs:
                return
            png = pngs[-1]
            yml = png.replace('.png', '.yaml')
            if not os.path.exists(yml):
                return
            with open(yml) as f:
                meta = yaml.safe_load(f)
            res    = meta.get('resolution', 0.05)
            origin = meta.get('origin', [0.0, 0.0, 0.0])
            img    = cv2.imread(png)
            if img is None:
                return
            h, w = img.shape[:2]
            all_marks = {**self._incidental_targets}
            if self.found_location and self.target_name:
                all_marks[self.target_name] = self.found_location
            for tname, (mx, my) in all_marks.items():
                px = int((mx - origin[0]) / res)
                py = int(h - (my - origin[1]) / res)
                if 0 <= px < w and 0 <= py < h:
                    r, g, b = colour_for(tname)
                    bgr     = (int(b*255), int(g*255), int(r*255))
                    cv2.circle(img, (px, py), 10, bgr, -1)
                    cv2.circle(img, (px, py), 12, (0,0,0), 2)
                    cv2.putText(img, tname, (px+14, py+4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, bgr, 1)
            cv2.imwrite(png, img)
            print(f'\n  Map with markers saved: {png}')
        except Exception as e:
            self.get_logger().error(f'Map overlay failed: {e}')




    # ── Helpers ───────────────────────────────────────────────────────────────




    def _set_explore(self, resume: bool):
        msg = Bool(); msg.data = resume
        self.resume_pub.publish(msg)




    def _publish_cmd(self, cmd: str):
        msg = String(); msg.data = cmd
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'cmd → {cmd}')




    def _get_robot_pose(self) -> Tuple[float, float]:
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
            return tf.transform.translation.x, tf.transform.translation.y
        except:
            return 0.0, 0.0




    def _publish_marker(self, name: str, x: float, y: float):
        r, g, b = colour_for(name)
        mk = Marker()
        mk.header.frame_id    = 'map'
        mk.header.stamp       = self.get_clock().now().to_msg()
        mk.ns                 = 'seek_destroy'
        mk.id                 = self._marker_id
        mk.type               = Marker.SPHERE
        mk.action             = Marker.ADD
        mk.pose.position      = Point(x=x, y=y, z=0.3)
        mk.pose.orientation.w = 1.0
        mk.scale.x = mk.scale.y = mk.scale.z = 0.25
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = r, g, b, 0.9
        mk.lifetime           = Duration(sec=0, nanosec=0)
        self.marker_pub.publish(mk)
        self._marker_id += 1




    @staticmethod
    def _approach_point(rx, ry, tx, ty, standoff):
        dx, dy = tx-rx, ty-ry
        dist   = math.hypot(dx, dy)
        if dist < 0.01:
            return rx, ry
        ratio = max(0.0, dist-standoff) / dist
        return rx + dx*ratio, ry + dy*ratio




    # ── Terminal UI (fallback if GUI not running) ─────────────────────────────




    # def _run_ui(self):
    #     time.sleep(1.5)
    #     while rclpy.ok():
    #         if self._gui_quit_pending:
    #             self._set_explore(False)
    #             rclpy.shutdown()
    #             break




    #         with self._lock:
    #             s = self.state




    #         if s == 'IDLE':
    #             if self._gui_target_pending:
    #                 raw = self._gui_target_pending
    #                 self._gui_target_pending = None
    #             else:
    #                 print('\n' + '-'*40)
    #                 if self.valid_targets:
    #                     print(f'  Targets: {", ".join(self.valid_targets)}')
    #                 print('  (or use GUI buttons)')
    #                 try:
    #                     raw = input('  Target > ').strip()
    #                 except EOFError:
    #                     time.sleep(0.5)
    #                     continue




    #             if not raw:
    #                 time.sleep(0.2)
    #                 continue




    #             fmt = format_target(raw)




    #             if fmt in self._confirmed_targets:
    #                 cx, cy = self._confirmed_targets[fmt]
    #                 print(f'\n  Already found! Driving to ({cx:.2f},{cy:.2f})...')
    #                 with self._lock:
    #                     self.target_name    = fmt
    #                     self.found_location = (cx, cy)
    #                 self._publish_cmd(f'GO_TO:{cx:.4f},{cy:.4f}')
    #                 self._transition('NAVIGATING_TO_KNOWN')
    #                 continue




    #             with self._lock:
    #                 self.target_name    = fmt
    #                 self.found_location = None
    #                 self._conf_accum.clear()
    #             print(f'\n  Searching for: "{fmt}"')
    #             self._transition('SWEEPING')




    #         elif s in ('END', 'HOME_FAILED'):
    #             print('\n' + '-'*40)
    #             if s == 'HOME_FAILED':
    #                 print(f'  Target "{self.target_name}" not found.')
    #             try:
    #                 ans = input('  [s = search again  /  q = quit] > ').strip().lower()
    #             except EOFError:
    #                 time.sleep(0.5)
    #                 continue
    #             if ans == 'q':
    #                 self._set_explore(False)
    #                 rclpy.shutdown()
    #                 break
    #             elif ans == 's':
    #                 with self._lock:
    #                     self.target_name    = None
    #                     self.found_location = None
    #                     self.state          = 'IDLE'
    #         else:
    #             time.sleep(0.4)




    def _run_ui(self):
        """
        Terminal fallback — only handles xterm keyboard input.
        GUI targets are handled directly in on_set_target() on the ROS thread.
        """
        time.sleep(1.5)




        while rclpy.ok():
            if self._gui_quit_pending:
                self._set_explore(False)
                rclpy.shutdown()
                break




            with self._lock:
                s = self.state




            if s == 'IDLE':
                print('\n' + '-' * 58)
                if self.valid_targets:
                    print(f'  Targets: {", ".join(self.valid_targets)}')
                print('  (or use GUI buttons)')
                try:
                    raw = input('  Target > ').strip()
                except EOFError:
                    time.sleep(0.5)
                    continue




                # GUI may have started a mission while input() was blocking
                with self._lock:
                    if self.state != 'IDLE':
                        print('  [xterm] GUI already started a mission — ignoring.')
                        continue




                if not raw:
                    time.sleep(0.2)
                    continue




                fmt = format_target(raw)




                if self.valid_targets and fmt not in self.valid_targets:
                    print(f'  ⚠️  "{fmt}" not valid. Options: {", ".join(self.valid_targets)}')
                    continue




                if fmt in self._confirmed_targets:
                    cx, cy = self._confirmed_targets[fmt]
                    print(f'\n  Already found! Driving to ({cx:.2f},{cy:.2f})...')
                    with self._lock:
                        self.target_name    = fmt
                        self.found_location = (cx, cy)
                    self._publish_cmd(f'GO_TO:{cx:.4f},{cy:.4f}')
                    self._transition('NAVIGATING_TO_KNOWN')
                    continue




                with self._lock:
                    self.target_name    = fmt
                    self.found_location = None
                    self._conf_accum.clear()
                print(f'\n  Searching for: "{fmt}"')
                self._transition('SWEEPING')




            elif s in ('END', 'HOME_FAILED'):
                print('\n' + '-' * 58)
                if s == 'HOME_FAILED':
                    print(f'  Target "{self.target_name}" was not found.')
                try:
                    ans = input('  [s = search again   q = quit] > ').strip().lower()
                except EOFError:
                    time.sleep(0.5)
                    continue




                if ans == 'q':
                    self._set_explore(False)
                    rclpy.shutdown()
                    break
                elif ans == 's':
                    with self._lock:
                        self.target_name    = None
                        self.found_location = None
                        self.state          = 'IDLE'
                else:
                    print('  Type "s" or "q".')
            else:
                time.sleep(0.4)








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