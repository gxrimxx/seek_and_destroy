import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time

class WaypointExplorer(Node):
    def __init__(self):
        super().__init__('waypoint_explorer')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Define exploration waypoints as (x, y) coordinates in the map frame
        # Spread these around your Gazebo world — adjust values after seeing the map
        self.waypoints = [
            (1.0,  0.0),
            (1.0,  1.5),
            (0.0,  1.5),
            (-1.0, 1.5),
            (-1.0, 0.0),
            (-1.0,-1.5),
            (0.0, -1.5),
            (1.0, -1.5),
        ]
        self.current_index = 0
        self.exploring = True
        
        self.get_logger().info('WaypointExplorer started — waiting for Nav2...')
        self._client.wait_for_server()
        self.get_logger().info('Nav2 ready. Starting exploration.')
        self.send_next_waypoint()

    def send_next_waypoint(self):
        if not self.exploring:
            return
            
        if self.current_index >= len(self.waypoints):
            self.get_logger().info('All waypoints visited. Looping exploration.')
            self.current_index = 0  # Loop for continuous exploration
            
        x, y = self.waypoints[self.current_index]
        self.get_logger().info(f'Navigating to waypoint {self.current_index}: ({x}, {y})')
        
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0  # Face forward
        
        send_goal_future = self._client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Waypoint rejected by Nav2. Skipping.')
            self.current_index += 1
            self.send_next_waypoint()
            return
            
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.current_index += 1
        self.send_next_waypoint()

    def stop_exploration(self):
        self.exploring = False
        self.get_logger().info('Exploration stopped by target detector.')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointExplorer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()