import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from action_interface.action import NavGoal
import traceback
from keyboard_mode_interface_pkg.action_server_handler import (
    handle_action_result,
)


class CarActionClient:
    def __init__(self, node):
        """
        Initializes the CarActionClient.

        Args:
            node: The rclpy.node.Node instance to use for communication.
        """
        self._node = node  # Store the node instance
        self._logger = node.get_logger()  # Get logger from the node

        self.nav_client = ActionClient(self._node, NavGoal, "nav_action_server")
        self.current_goal_handle = None
        self._get_result_future = None  # Initialize this attribute

        self._logger.info("CarActionClient initialized.")

    def send_navigation_goal(self, mode):
        """Send a navigation goal"""
        goal_msg = NavGoal.Goal()
        goal_msg.mode = mode

        # Ensure previous goal processing is cleared before sending a new one
        if self.current_goal_handle is not None:
            self._logger.warn(
                "A previous goal might still be active. Cancelling it implicitly is not recommended."
            )
            self.current_goal_handle = None  # Reset handle reference
            if (
                self._get_result_future is not None
                and not self._get_result_future.done()
            ):
                self._get_result_future.cancel()  # Attempt to cancel the future if not done
                self._logger.info("Cancelled previous pending result future.")
            self._get_result_future = None

        # Wait for server with timeout
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self._logger.error("Navigation server not available!")
            return False

        self._logger.info(f"Sending navigation goal with mode: {mode}")
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

        return True

    def cancel_navigation_goal(self):
        """Cancel the current navigation goal if one exists"""
        if self.current_goal_handle is None:
            self._logger.warn("No active navigation goal handle to cancel.")
            return False

        if not self.current_goal_handle.is_active:
            self._logger.warn("Goal is no longer active, cannot cancel.")
            return False

        self._logger.info("Requesting navigation goal cancellation...")
        try:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_result_callback)
            return True
        except Exception as e:
            self._logger.error(f"Error cancelling navigation goal: {e}")
            return False

    def goal_response_callback(self, future):
        """Handles the response after sending a goal."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._logger.info("Goal rejected :(")
                self.current_goal_handle = None
                return

            self._logger.info("Goal accepted :)")
            self.current_goal_handle = goal_handle

            self._get_result_future = self.current_goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)

        except Exception as e:
            self._logger.error(f"Error in goal response callback: {e}")
            self.current_goal_handle = None

    def cancel_result_callback(self, future):
        """Handles the response after requesting a cancellation."""
        try:
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
                self._logger.info(
                    f"Cancellation request acknowledged for goal ID: {cancel_response.goals_canceling[0].goal_id.uuid.hex()}"
                )
            else:
                self._logger.warn(
                    "Cancellation request failed or goal already completed."
                )
        except Exception as e:
            self._logger.error(f"Error processing cancel response: {e}")

    def get_result_callback(self, future):
        """Process the action result, including abort signals."""
        try:
            goal_result = future.result()
            status = goal_result.status
            result = goal_result.result

            handle_action_result(
                node=self._node,
                status=status,
                result=result,
            )

        except Exception as e:
            self._logger.error(f"Error in get_result_callback: {e}")
            self._logger.error(traceback.format_exc())
        finally:
            self.current_goal_handle = None
            self._get_result_future = None

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self._logger.info(
        #     "Received feedback: {0}".format(feedback.distance_to_goal)
        # )
