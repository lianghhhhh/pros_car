import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from action_interface.action import NavGoal
from car_control_pkg.car_control_common import BaseCarControlNode
from car_control_pkg.nav_processing import Nav2Processing
import math


class NavigationActionServer(BaseCarControlNode):
    def __init__(self):
        # Initialize base class with navigation subscribers enabled
        super().__init__("navigation_action_server_node", enable_nav_subscribers=True)
        # Create action server
        self._action_server = ActionServer(
            self,
            NavGoal,
            "nav_action_server",
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info("Navigation Action Server initialized")

    def goal_callback(self, goal_request):
        """
        Accept or reject a client request to begin an action.
        Now also validates that a valid navigation mode is provided.
        """
        self.get_logger().info(f"mode: {goal_request.mode}")

        # Validate mode (allowed modes: "Auto Nav", "Custom Mode", "Manual Nav")
        allowed_modes = ["Auto_Nav", "Custom_Mode", "Manual_Nav"]
        if goal_request.mode not in allowed_modes:
            self.get_logger().error(
                f"Invalid mode: {goal_request.mode}. Allowed modes: {allowed_modes}"
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept a client request to cancel an action."""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the navigation action."""
        self.get_logger().info("Executing navigation goal...")

        # Extract goal data and mode from the goal request
        mode = goal_handle.request.mode

        # Create result and feedback messages
        feedback = NavGoal.Feedback()
        result = NavGoal.Result()

        # Set control loop rate (10 Hz)
        rate = self.create_rate(10)

        # Main navigation loop
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled")
                self.publish_control("STOP")
                goal_handle.canceled()
                result.success = False
                result.message = "Navigation canceled"
                return result

            # Get current position and orientation using the base class helper method
            car_position, car_orientation = self.get_car_position_and_orientation()

            if not car_position:
                self.get_logger().warn("No position data available")
                rate.sleep()
                continue

            # Calculate Euclidean distance to the goal
            distance = 0.3

            # Check if the goal is reached (within 20cm of the goal)
            if distance < 0.2:
                self.get_logger().info("Goal reached!")
                self.publish_control("STOP")
                break

            # Update and publish feedback
            feedback.distance_to_goal = float(distance)
            goal_handle.publish_feedback(feedback)

            rate.sleep()

        # Navigation completed successfully
        result.success = True
        result.message = "Navigation completed successfully"
        goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = NavigationActionServer()
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        action_server.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
