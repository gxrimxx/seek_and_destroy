import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math


class GoHome(Node):
    def __init__(self):
        super().__init__('go_home')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_subscription(String, '/mission/command', self.on_cmd, 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)

        self.get_logger().info('Go Home node ready. Listening for GO_HOME command...')

    def on_cmd(self, msg):
        if msg.data == 'GO_HOME':
            self.get_logger().info('GO_HOME received! Navigating to (0, 0)...')
            self.navigate_to(x=0.0, y=0.0, yaw=0.0)

    def navigate_to(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info('Waiting for Nav2 action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal to (0, 0)...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal REJECTED by Nav2!')
            return
        self.get_logger().info('Goal accepted. Robot is heading home...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info('Robot has ARRIVED HOME!')
        msg = String()
        msg.data = 'ARRIVED_HOME'
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoHome()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()