import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from action_interface.action import NavGoal
from car_control_pkg.car_control_common import BaseCarControlNode
import time


class NavigationActionServer(BaseCarControlNode):
    def __init__(self):
        super().__init__("navigation_action_server_node", enable_nav_subscribers=True)

        # Create the action server
        self._action_server = ActionServer(
            self,
            NavGoal,
            "nav_action_server",
            execute_callback=self.execute_callback,
        )
        self.get_logger().info("Navigation Action Server initialized")

    def execute_callback(self, goal_handle):
        """Long-running navigation action callback."""
        self.get_logger().info("Executing goal...")

        # Extract goal data
        mode = goal_handle.request.mode
        self.get_logger().info(f"Navigation mode: {mode}")

        # Create feedback and result messages
        feedback_msg = NavGoal.Feedback()
        result = NavGoal.Result()

        # # Get initial car position
        car_position, car_orientation = self.get_car_position_and_orientation()

        # # Check if we have valid position data
        if not car_position:
            self.get_logger().error("Navigation failed: No position data")
            result.success = False
            result.message = "Cannot obtain car position data"
            goal_handle.abort()
            return result

        # Main navigation loop
        # rate = self.create_rate(1)  # 10Hz update rate

        for i in range(50):  # Just a simple loop for demonstration
            # Check if goal has been canceled
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled")
                goal_handle.canceled()
                result.success = False
                result.message = "Navigation canceled"
                return result

            # Calculate current distance to goal
            # In a real implementation, this would use actual position and goal data
            distance = 5.0 - (i * 0.1)  # Simulated distance that decreases

            # Update feedback
            feedback_msg.distance_to_goal = float(distance)

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"Published feedback: distance = {distance:.2f}m")

            # Check if we've reached the goal
            if distance < 0.2:
                break

            # Sleep to control the loop rate
            # time.sleep(0.1)

        # Set result
        result.success = True
        result.message = "Navigation completed successfully"
        goal_handle.succeed()
        print("success")
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = NavigationActionServer()

    # Use a multi-threaded executor, no spin_once() calls in the execute_callback
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        action_server.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
