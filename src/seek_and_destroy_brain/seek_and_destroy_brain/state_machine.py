import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import threading
import time


class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')

        self.state = 'IDLE'
        self.target_name = None
        self.found_location = None

        # Publishers
        self.cmd_pub = self.create_publisher(String, '/mission/command', 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.create_subscription(String, '/detection/result', self.on_detection, 10)
        self.create_subscription(String, '/mission/status', self.on_status, 10)

        # Timer for exploration spin behavior
        self.explore_timer = None
        self.explore_direction = 1.0
        self.explore_step = 0

        print('\n' + '='*50)
        print('   PROJECT SEEK & DESTROY')
        print('   Robot is ready at Home (0, 0)')
        print('='*50)

        # Run UI in a background thread so ROS can spin at the same time
        ui_thread = threading.Thread(target=self.run_ui, daemon=True)
        ui_thread.start()

    # ─── CALLBACKS ───────────────────────────────────────────

    def on_detection(self, msg):
        """Called every time the camera detector finds a colored object."""
        if self.state != 'SEARCHING':
            return

        parts = msg.data.split(':')
        if len(parts) != 2:
            return

        detected_name = parts[0].strip()
        confidence = float(parts[1])

        self.get_logger().info(
            f'Detector sees: {detected_name} (confidence: {confidence:.2f})')

        # Cross-check: is this what the user asked for?
        if detected_name == self.target_name and confidence > 0.05:
            self.found_location = '(current robot position)'
            self.transition('FOUND')

    def on_status(self, msg):
        """Called when go_home node says we've arrived."""
        if msg.data == 'ARRIVED_HOME':
            if self.state == 'HOME':
                self.transition('END')

    # ─── STATE MACHINE ───────────────────────────────────────

    def transition(self, new_state):
        old = self.state
        self.state = new_state
        print(f'\n  [STATE] {old} → {new_state}')

        if new_state == 'EXPLORING':
            self.publish_cmd('START_EXPLORE')
            self.start_exploration()

        elif new_state == 'SEARCHING':
            # Keep moving but now check detections
            pass

        elif new_state == 'FOUND':
            self.stop_robot()
            print(f'\n  ✅ TARGET FOUND: {self.target_name}')
            print(f'     Location: {self.found_location}')
            print('  Returning to Home...')
            self.publish_cmd('GO_HOME')
            self.state = 'HOME'

        elif new_state == 'NOT_FOUND':
            self.stop_robot()
            print(f'\n  ❌ Target "{self.target_name}" was NOT found.')
            print('  Returning to Home...')
            self.publish_cmd('GO_HOME')
            self.state = 'HOME'

        elif new_state == 'HOME':
            self.stop_robot()

        elif new_state == 'END':
            print('\n  🏠 Robot is back at Home.')
            print('  Map has been (or can be) saved.')

    def publish_cmd(self, cmd):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'Published command: {cmd}')

    # ─── EXPLORATION (simple spin-and-drive) ─────────────────

    def start_exploration(self):
        """Simple exploration: drive forward, spin, repeat."""
        self.get_logger().info('Starting exploration behavior...')
        self.explore_step = 0
        # Switch to SEARCHING after 3 seconds so detector is active
        self.create_timer(3.0, self.start_searching_callback)
        # Start movement
        self.exploration_timer = self.create_timer(0.1, self.explore_tick)

    def start_searching_callback(self):
        """After a few seconds of exploring, also check for targets."""
        if self.state == 'EXPLORING':
            self.state = 'SEARCHING'
            print('\n  [STATE] EXPLORING → SEARCHING (camera now active)')

    def explore_tick(self):
        """Called every 0.1s — drives the robot in a search pattern."""
        if self.state not in ('EXPLORING', 'SEARCHING'):
            return

        self.explore_step += 1
        twist = Twist()

        # Pattern: drive forward for 3s, turn for 2s, repeat
        cycle = self.explore_step % 50  # 50 ticks = 5 seconds per cycle
        if cycle < 30:
            twist.linear.x = 0.2   # drive forward
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.6 * self.explore_direction  # turn

        # Flip direction every 5 cycles
        if self.explore_step % 250 == 0:
            self.explore_direction *= -1

        self.vel_pub.publish(twist)

        # Auto-timeout: if exploring for 90 seconds with no find, go NOT_FOUND
        if self.explore_step >= 900 and self.state == 'SEARCHING':
            self.transition('NOT_FOUND')

    def stop_robot(self):
        twist = Twist()
        self.vel_pub.publish(twist)

    # ─── TERMINAL UI ─────────────────────────────────────────

    def run_ui(self):
        """Runs in a thread — handles terminal input."""
        time.sleep(1.0)  # wait for node to fully start

        while rclpy.ok():
            if self.state == 'IDLE':
                print('\n' + '-'*50)
                self.target_name = input('  Which target are you looking for?\n  > ').strip()

                if not self.target_name:
                    print('  Please type a target name (e.g. Red Cylinder, Blue Box, Green Cylinder)')
                    continue

                print(f'\n  Searching for: "{self.target_name}"')
                print('  Available targets: Red Cylinder, Blue Box, Green Cylinder')
                self.transition('EXPLORING')

            elif self.state == 'END':
                print('\n' + '-'*50)
                answer = input('  Search for another target or quit? [s = search again / q = quit]\n  > ').strip().lower()

                if answer == 'q':
                    print('\n  Goodbye! Mission complete.')
                    self.stop_robot()
                    rclpy.shutdown()
                    break
                elif answer == 's':
                    self.target_name = None
                    self.found_location = None
                    self.explore_step = 0
                    self.transition('IDLE')
                    self.state = 'IDLE'
                else:
                    print('  Type "s" to search again or "q" to quit')

            else:
                time.sleep(0.5)  # wait while robot is doing stuff


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)


if __name__ == '__main__':
    main()