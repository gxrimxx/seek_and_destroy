#!/usr/bin/env python3
"""
navigator.py  -  Project Seek & Destroy
Central navigation handler — accepts commands on /mission/command and
reports outcomes on /mission/status.

Commands accepted:
  GO_HOME           → navigate to (0, 0)     → ARRIVED_HOME
  GO_TO:x,y         → navigate to (x, y)     → ARRIVED_TARGET
  INVESTIGATE:x,y   → navigate to (x, y)     → ARRIVED_INVESTIGATE
  ROAM_TO:x,y       → navigate to (x, y)     → ARRIVED_ROAM
  CANCEL            → cancel current goal    → (silent)
  SAVE_MAP:path     → save map PNG + YAML    → MAP_SAVED | MAP_SAVE_FAILED
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
import math
import os


class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_subscription(String, '/mission/command', self.on_cmd, 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)

        # Type of the in-flight goal — drives which status message is published
        self._goal_type: str           = 'HOME'
        self._active_goal_handle       = None

        self.get_logger().info(
            'Navigator ready. Listening on /mission/command...'
        )

    # ─────────────────────────────────────────────────────────────────────────
    # COMMAND ROUTER
    # ─────────────────────────────────────────────────────────────────────────

    def on_cmd(self, msg: String):
        data = msg.data

        if data == 'GO_HOME':
            self._goal_type = 'HOME'
            self.get_logger().info('GO_HOME → navigating to (0, 0)')
            self._send_nav_goal(0.0, 0.0)

        elif data == 'CANCEL':
            self._cancel()

        elif data.startswith('GO_TO:'):
            x, y = self._parse_xy(data, 'GO_TO:')
            if x is not None:
                self._goal_type = 'TARGET'
                self.get_logger().info(f'GO_TO → ({x:.3f}, {y:.3f})')
                self._send_nav_goal(x, y)

        elif data.startswith('INVESTIGATE:'):
            x, y = self._parse_xy(data, 'INVESTIGATE:')
            if x is not None:
                self._goal_type = 'INVESTIGATE'
                self.get_logger().info(f'INVESTIGATE → ({x:.3f}, {y:.3f})')
                self._send_nav_goal(x, y)

        elif data.startswith('ROAM_TO:'):
            x, y = self._parse_xy(data, 'ROAM_TO:')
            if x is not None:
                self._goal_type = 'ROAM'
                self.get_logger().info(f'ROAM_TO → ({x:.3f}, {y:.3f})')
                self._send_nav_goal(x, y)

        elif data.startswith('SAVE_MAP:'):
            path = data[len('SAVE_MAP:'):]
            self._save_map(path)

    # ─────────────────────────────────────────────────────────────────────────
    # NAVIGATION
    # ─────────────────────────────────────────────────────────────────────────

    def _send_nav_goal(self, x: float, y: float, yaw: float = 0.0):
        goal                               = NavigateToPose.Goal()
        goal.pose                          = PoseStamped()
        goal.pose.header.frame_id          = 'map'
        goal.pose.header.stamp             = self.get_clock().now().to_msg()
        goal.pose.pose.position.x          = x
        goal.pose.pose.position.y          = y
        goal.pose.pose.position.z          = 0.0
        goal.pose.pose.orientation.z       = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w       = math.cos(yaw / 2.0)

        self.get_logger().info('Waiting for Nav2 action server...')
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _cancel(self):
        if self._active_goal_handle is not None:
            self.get_logger().info('Cancelling active navigation goal.')
            self._active_goal_handle.cancel_goal_async()
            self._active_goal_handle = None

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal.')
            self._publish_status(f'NAV_FAILED:{self._goal_type}')
            return
        self._active_goal_handle = handle
        self.get_logger().info('Goal accepted — robot is moving.')
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        self._active_goal_handle = None
        status = future.result().status

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f'Navigation did not succeed '
                f'(status={status}, goal_type={self._goal_type})'
            )
            self._publish_status(f'NAV_FAILED:{self._goal_type}')
            return

        status_map = {
            'HOME':        'ARRIVED_HOME',
            'TARGET':      'ARRIVED_TARGET',
            'INVESTIGATE': 'ARRIVED_INVESTIGATE',
            'ROAM':        'ARRIVED_ROAM',
        }
        reply = status_map.get(self._goal_type, 'ARRIVED_HOME')
        self.get_logger().info(f'Arrived — publishing: {reply}')
        self._publish_status(reply)

    # ─────────────────────────────────────────────────────────────────────────
    # MAP SAVING
    # ─────────────────────────────────────────────────────────────────────────

    def _save_map(self, map_path: str):
        self.get_logger().info(f'Saving map → {map_path}')
        os.makedirs(os.path.dirname(map_path), exist_ok=True)
        ret = os.system(
            f'ros2 run nav2_map_server map_saver_cli -f {map_path} --fmt png'
        )
        self._publish_status('MAP_SAVED' if ret == 0 else 'MAP_SAVE_FAILED')
        if ret != 0:
            self.get_logger().error(f'map_saver_cli failed (exit {ret})')

    # ─────────────────────────────────────────────────────────────────────────
    # UTILITIES
    # ─────────────────────────────────────────────────────────────────────────

    def _publish_status(self, text: str):
        msg      = String()
        msg.data = text
        self.status_pub.publish(msg)

    @staticmethod
    def _parse_xy(data: str, prefix: str):
        try:
            coords  = data[len(prefix):]
            xs, ys  = coords.split(',')
            return float(xs), float(ys)
        except Exception:
            return None, None


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()