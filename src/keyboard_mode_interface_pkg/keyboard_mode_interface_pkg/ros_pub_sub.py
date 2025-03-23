import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient
from action_interface.action import NavGoal


class ROS2Manager(Node):
    def __init__(self):
        super().__init__("ros2_manager")

        # Publisher（發送指令給其他節點）
        self.car_control_publisher = self.create_publisher(
            String, "car_control_signal", 10
        )

        # Subscriber（接收來自其他節點的訊息）
        self.subscription = self.create_subscription(
            String, "menu_feedback", self.listener_callback, 10
        )

        # Create action client
        self.nav_client = ActionClient(self, NavGoal, "nav_action_server")
        self.current_goal_handle = None

    def send_navigation_goal(self, mode="Manual_Nav"):
        """Send a goal to the navigation action server"""
        # Wait for action server
        self.nav_client.wait_for_server()

        # Create the goal
        goal_msg = NavGoal.Goal()
        goal_msg.mode = mode

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        # Add callback for when the future completes
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """Process feedback during goal execution"""
        # Extract the distance from the feedback message
        distance = feedback_msg.feedback.distance_to_goal

        # Log the feedback - this is what you'll see in the console
        self.get_logger().info(f"Distance to goal: {distance:.2f}m")

    def goal_response_callback(self, future):
        """Callback upon receiving the goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Navigation goal rejected")
            return
        self.get_logger().info("Navigation goal accepted")
        self.current_goal_handle = goal_handle
        # Wait for the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Callback to handle the result of the action."""
        result = future.result().result
        self.get_logger().info(f"Navigation result: {result.message}")

    def cancel_navigation(self):
        """Cancel the active navigation goal"""
        if self.current_goal_handle:
            self.nav_client.cancel_goal_async(self.current_goal_handle)
            self.get_logger().info("Navigation goal cancellation requested.")

    def publish_car_signal(self, command):
        """發布選單的指令"""
        msg = String()
        msg.data = command
        self.car_control_publisher.publish(msg)

    def listener_callback(self, msg):
        """監聽來自其他節點的回應"""
        self.get_logger().info(f"收到回應: {msg.data}")
