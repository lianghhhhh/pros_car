import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from action_interface.action import NavGoal
from car_control_pkg.car_control_common import BaseCarControlNode
from car_control_pkg.nav2_utils import (
    get_yaw_from_quaternion,
    get_direction_vector,
    get_angle_to_target,
    calculate_angle_point,
    quaternion_to_euler,
    calculate_angle_to_target,
    round_to_decimal_places,
    cal_distance,
    calculate_diff_angle,
)
import time
import asyncio


class NavigationActionServer(BaseCarControlNode):
    def __init__(self):
        super().__init__("navigation_action_server_node", enable_nav_subscribers=True)

        # Create the action server
        self._action_server = ActionServer(
            self,
            NavGoal,
            "nav_action_server",
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info("Navigation Action Server initialized")
        self.index = 0

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received request to cancel goal!")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Async navigation action callback"""
        self.get_logger().info("Executing navigation goal...")

        # Extract goal data
        mode = goal_handle.request.mode
        self.get_logger().info(f"Navigation mode: {mode}")

        # Create feedback and result messages
        feedback_msg = NavGoal.Feedback()
        result = NavGoal.Result()

        # Get initial car position
        car_position, car_orientation = self.get_car_position_and_orientation()

        # Check if we have valid position data
        if not car_position:
            self.get_logger().error("Navigation failed: No position data")
            result.success = False
            result.message = "Cannot obtain car position data"
            goal_handle.abort()
            return result

        # Get path points with orientation
        path_points = self.get_path_points(dynamic=True, include_orientation=True)

        if not path_points:
            self.get_logger().error("Navigation failed: No path points available")
            result.success = False
            result.message = "No path available for navigation"
            goal_handle.abort()
            return result

        self.get_logger().info(f"Starting navigation with {len(path_points)} waypoints")

        # Set up control rate (10Hz)
        # rate = self.create_rate(10)  # 10 Hz control loop

        # Navigation parameters
        waypoint_threshold = 0.5  # Distance in meters to consider waypoint reached
        current_waypoint_index = 0
        rate = self.create_rate(10)
        action_key = None
        # Main navigation loop
        while rclpy.ok():
            # Check if goal was canceled
            print(goal_handle.is_cancel_requested)
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Navigation goal canceled")
                self.publish_control("STOP")
                result.success = False
                result.message = "Navigation canceled"
                goal_handle.canceled()
                break
            rate.sleep()
            # await asyncio.sleep(0.1)

            # Get current car position
            car_position, car_orientation = self.get_car_position_and_orientation()
            car_position = car_position[0:3]
            car_orientation = car_orientation[2:4]
            if not car_position:
                self.get_logger().warn("Lost position data during navigation")
                self.publish_control("STOP")
                result.success = False
                result.message = "Lost position data during navigation"
                goal_handle.abort()
                return result

            target_x, target_y = self.get_next_target_point(
                car_position, path_points=path_points
            )
            final_pos = path_points[-1]["position"][0:2]
            target_distance = cal_distance(car_position, final_pos)
            if target_x is None or target_distance < 0.5:
                self.get_logger().info(
                    f"Goal reached! Distance: {target_distance:.2f}m"
                )
                break  # EXIT THE LOOP HERE
            diff_angle = calculate_diff_angle(
                car_position, car_orientation, target_x, target_y
            )
            if diff_angle < 20 and diff_angle > -20:
                action_key = "FORWARD"
            elif diff_angle < -20 and diff_angle > -180:
                action_key = "CLOCKWISE_ROTATION"
            elif diff_angle > 20 and diff_angle < 180:
                action_key = "COUNTERCLOCKWISE_ROTATION"
            self.publish_control(action_key)
            # Update feedback
            feedback_msg.distance_to_goal = float(target_distance)
            goal_handle.publish_feedback(feedback_msg)
            self.publish_control(action_key)
            # await asyncio.sleep(0.1)
            # rate.sleep()
        # Navigation completed successfully
        self.publish_control("STOP")
        result.success = True
        result.message = "Navigation completed successfully"
        goal_handle.succeed()

        return result

    def get_next_target_point(
        self, car_position, path_points, min_required_distance=0.5
    ):
        """
        選擇距離車輛 min_required_distance 以上最短路徑然後返回 target_x, target_y
        """
        if path_points is None:
            print("Error: global_plan_msg is None or poses is missing!")
            return None, None
        while self.index < len(path_points) - 1:
            target_x = path_points[self.index]["position"][0]
            target_y = path_points[self.index]["position"][1]
            distance_to_target = cal_distance(car_position, (target_x, target_y))

            if distance_to_target < min_required_distance:
                self.index += 1
            else:
                # self.ros_communicator.publish_selected_target_marker(
                #     x=target_x, y=target_y
                # )
                return target_x, target_y

        return None, None


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    action_server = NavigationActionServer()
    executor.add_node(action_server)
    # Use a multi-threaded executor, no spin_once() calls in the execute_callback
    try:
        executor.spin()
    except KeyboardInterrupt:
        action_server.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
