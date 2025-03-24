import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_interface.action import NavGoal


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__("fibonacci_action_client")
        self.nav_client = ActionClient(self, NavGoal, "nav_action_server")

    def send_goal(self, mode):
        goal_msg = NavGoal.Goal()
        goal_msg.mode = mode
        self.nav_client.wait_for_server()
        self.current_goal_handle = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.current_goal_handle.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            "Result: success={0}, message='{1}'".format(result.success, result.message)
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            "Received feedback: {0}".format(feedback.distance_to_goal)
        )


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(mode="test")

    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
