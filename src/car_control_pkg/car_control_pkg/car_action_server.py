import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from action_interface.action import NavGoal
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
from car_control_pkg.utils import parse_control_signal
from car_control_pkg.car_control_common import CarControlPublishers
import math
import time


class NavigationActionServer(Node):
    def __init__(self):
        super().__init__("navigation_action_server_node")
        self._action_server = ActionServer(
            self,
            NavGoal,
            "nav_action_server",
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.subscription = self.create_subscription(
            String, "car_control_signal", self.key_callback, 10
        )

        # Get publishers from common module
        self.rear_wheel_pub, self.front_wheel_pub = (
            CarControlPublishers.create_publishers(self)
        )

    def publish_control(self, action):
        # Use common method for publishing
        CarControlPublishers.publish_control(
            self, action, self.rear_wheel_pub, self.front_wheel_pub
        )

    # Add key_callback method if needed
    def key_callback(self, msg):
        pass

    def goal_callback(self, goal_request):
        pass

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        pass

        while rclpy.ok():
            pass


def main(args=None):
    rclpy.init(args=args)
    action_server = NavigationActionServer()
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
