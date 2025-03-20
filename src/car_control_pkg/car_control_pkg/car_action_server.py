import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from action_interface.action import NavGoal
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from car_control_pkg.utils import parse_control_signal
import math
import time


class NavigationActionServer(Node):
    def __init__(self):
        super().__init__("navigation_action_server_node")
        self._action_server = ActionServer(
            self,
            NavGoal,
            "my_action",
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # Publisher for movement commands
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Current position (in real system would come from odometry)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.get_logger().info("Navigation Action Server is ready")

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # Check if goal format is valid (expecting at least [x, y])
        if len(goal_request.goal) < 2:
            self.get_logger().error("Invalid goal format - need at least [x, y]")
            return GoalResponse.REJECT

        self.get_logger().info(f"Received goal: {goal_request.goal}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info("Executing navigation goal...")

        # Extract goal coordinates from goal array
        goal_array = goal_handle.request.goal
        target_x = goal_array[0]
        target_y = goal_array[1]
        target_theta = goal_array[2] if len(goal_array) > 2 else None

        self.get_logger().info(f"Navigating to: ({target_x}, {target_y})")

        # Create feedback and result messages
        feedback = NavGoal.Feedback()
        result = NavGoal.Result()

        # Navigation loop
        rate = self.create_rate(10)  # 10Hz

        while rclpy.ok():
            # Calculate distance to goal
            distance_to_goal = math.sqrt(
                (target_x - self.current_x) ** 2 + (target_y - self.current_y) ** 2
            )

            # Check if goal is reached
            if distance_to_goal < 0.1:  # Within 10cm
                self.get_logger().info("Goal reached!")
                break

            # Check if goal is cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                result.success = False
                result.message = "Navigation canceled"
                return result

            # Simple motion control toward goal
            cmd = Twist()

            # Determine direction to goal
            dx = target_x - self.current_x
            dy = target_y - self.current_y

            # Calculate desired heading
            desired_theta = math.atan2(dy, dx)

            # Proportional control for rotation
            cmd.angular.z = 0.5 * (desired_theta - self.current_theta)

            # Forward velocity (limited for safety)
            cmd.linear.x = min(0.2, 0.5 * distance_to_goal)

            # Publish command
            self.cmd_vel_publisher.publish(cmd)

            # Update position (simulated here - would be from sensors in real system)
            self.current_x += cmd.linear.x * math.cos(self.current_theta) * 0.1
            self.current_y += cmd.linear.x * math.sin(self.current_theta) * 0.1
            self.current_theta += cmd.angular.z * 0.1

            # Normalize theta
            self.current_theta = math.atan2(
                math.sin(self.current_theta), math.cos(self.current_theta)
            )

            # Publish feedback
            feedback.distance_to_goal = float(distance_to_goal)
            goal_handle.publish_feedback(feedback)

            self.get_logger().debug(f"Distance to goal: {distance_to_goal:.2f}")

            # Sleep to maintain rate
            rate.sleep()

        # Set final result
        result.success = True
        result.message = "Goal reached successfully"
        goal_handle.succeed()

        return result


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
