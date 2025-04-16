import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from action_interface.action import ArmGoal


class ArmActionServer(Node):
    def __init__(self, arm_commute_node, arm_auto_controller):
        super().__init__("arm_action_server_node")
        self._action_server = ActionServer(
            self,
            ArmGoal,
            "arm_action_server",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.arm_commute_node = arm_commute_node
        self.arm_auto_controller = arm_auto_controller
        self.get_logger().info("Arm Action Server initialized")

    def goal_callback(self, goal_request):
        self.get_logger().info("Received arm auto request")
        requested_mode = goal_request.mode
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Enter the cancel callback")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Navigation action callback"""
        result = ArmGoal.Result()
        mode = goal_handle.request.mode
        self.get_logger().info(f"Executing navigation in mode: {mode}")
        if mode == "wave":
            arm_auto_method = self.arm_auto_controller.arm_wave
        else:
            pass
        rate = self.create_rate(10)
        while rclpy.ok():
            rate.sleep()
            arm_auto_result = arm_auto_method()
            if isinstance(arm_auto_result, ArmGoal.Result):
                if arm_auto_result.success:
                    self.get_logger().info(
                        f"Arm auto completed: {arm_auto_result.message}"
                    )
                    goal_handle.succeed()
                else:
                    self.get_logger().error(
                        f"Navigation failed: {arm_auto_result.message}"
                    )
                    goal_handle.abort()
                # Exit the loop and return the result once a final state is reached.
                result = arm_auto_result
                break

            # Publish feedback if navigation is ongoing
            feedback_msg = ArmGoal.Feedback()
            feedback_msg.distance_to_goal = float(0.0)
            goal_handle.publish_feedback(feedback_msg)

        return result
