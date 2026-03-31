import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
import subprocess

class AutoMapSaver(Node):
    def __init__(self):
        super().__init__('auto_map_saver')
        # Subscribe to the standard ROS 2 logging topic
        self.subscription = self.create_subscription(
            Log,
            '/rosout',
            self.rosout_callback,
            10
        )
        self.map_saved = False
        self.get_logger().info('Auto Map Saver active! Listening for explore_lite to finish...')

    def rosout_callback(self, msg):
        if self.map_saved:
            return
        
        # Check if the log is from explore_lite and contains the exact completion string
        if msg.name == 'explore_node' and 'Successfully returned to initial pose' in msg.msg:
            self.get_logger().info('Exploration complete detected! Triggering map saver...')
            self.save_map()

    def save_map(self):
        # The exact path where your maps folder is located
        map_path = '/home/user/seek_destroy_ws/src/seek_destroy/maps/world_map'
        
        try:
            # Run the terminal command via Python
            subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path], check=True)
            self.get_logger().info(f'SUCCESS: Map permanently saved to {map_path}.pgm and .yaml!')
            self.map_saved = True
            
            # Mission accomplished, safely kill this specific node
            raise SystemExit
            
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Failed to save map: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AutoMapSaver()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.get_logger('rclpy').info('Auto Map Saver shutting down gracefully.')
    finally:
        # Cleanup
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()