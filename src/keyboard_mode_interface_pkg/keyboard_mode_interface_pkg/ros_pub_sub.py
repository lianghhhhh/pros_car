import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from action_interface.action import NavGoal
from keyboard_mode_interface_pkg.action_server_handler import (
    handle_action_result,
)
import traceback


class ROS2Manager(Node):
    def __init__(self):
        super().__init__("ros2_manager")

        # Publisher（發送指令給其他節點）
        self.car_control_publisher = self.create_publisher(
            String, "car_control_signal", 10
        )

        self.arm_control_publisher = self.create_publisher(
            String, "arm_control_signal", 10
        )

        self.test_publisher = self.create_publisher(String, "test", 10)

        # Subscriber（接收來自其他節點的訊息）
        self.subscription = self.create_subscription(
            String, "menu_feedback", self.listener_callback, 10
        )

        # Create action client
        self.nav_client = ActionClient(self, NavGoal, "nav_action_server")
        self.current_goal_handle = None

    def send_navigation_goal(self, mode):
        """Send a navigation goal"""
        goal_msg = NavGoal.Goal()
        goal_msg.mode = mode

        # Clear any previous goal handles
        self.current_goal_handle = None

        # Wait for server with timeout
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Navigation server not available!")
            return False

        # Store the future temporarily (not the goal handle yet)
        self.current_goal_handle = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.current_goal_handle.add_done_callback(self.goal_response_callback)

        return True

    def cancel_navigation_goal(self):
        """Cancel the current navigation goal if one exists"""
        if not hasattr(self, "current_goal_handle") or self.current_goal_handle is None:
            self.get_logger().warn("No active navigation goal to cancel")
            return False

        # Request cancellation
        self.get_logger().info("Requesting navigation goal cancellation...")
        try:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_result_callback)
            return True
        except Exception as e:
            self.get_logger().error(f"Error cancelling navigation goal: {e}")
            return False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            self.current_goal_handle = None  # Clear the future
            return

        self.get_logger().info("Goal accepted :)")
        self.current_goal_handle = goal_handle

        self._get_result_future = self.current_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def cancel_result_callback(self, future):
        try:
            cancel_response = future.result()  # This is a CancelGoal_Response object
            # For example, check return_code or other fields
            self.get_logger().info(f"Cancel response: {cancel_response}")
        except Exception as e:
            self.get_logger().error(f"Error in cancel_result_callback: {e}")

    def get_result_callback(self, future):
        """Process the action result, including abort signals."""
        try:
            # Get the goal handle result
            goal_result = future.result()

            # Get the status code
            status = goal_result.status

            # Get the actual result data
            result = goal_result.result

            handle_action_result(
                node=self,
                status=status,
                result=result,
            )

        except Exception as e:
            # Robust exception handling

            self.get_logger().error(f"Error in get_result_callback: {e}")
            self.get_logger().error(traceback.format_exc())

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(
        #     "Received feedback: {0}".format(feedback.distance_to_goal)
        # )

    def publish_car_signal(self, command):
        """發布選單的指令"""
        msg = String()
        msg.data = command
        self.car_control_publisher.publish(msg)

    def publish_arm_signal(self, command):
        msg = String()
        msg.data = command
        self.arm_control_publisher.publish(msg)

    def listener_callback(self, msg):
        """監聽來自其他節點的回應"""
        self.get_logger().info(f"收到回應: {msg.data}")
