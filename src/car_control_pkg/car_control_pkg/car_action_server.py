import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
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
import threading


class NavigationActionServer(Node):
    def __init__(self, car_control_node):
        super().__init__("navigation_action_server_node")
        self._action_server = ActionServer(
            self,
            NavGoal,
            "nav_action_server",
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
        )
        self.car_control_node = car_control_node
        self.get_logger().info("Navigation Action Server initialized")
        self.index = 0
        self._navigation_timer = None
        self._finished_event = threading.Event()
        self._result = None
        self._goal_handle = None
        self._path_points = None

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Enter the cancel callback")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing navigation goal...")
        self._goal_handle = goal_handle

        # 建立回傳結果的結構
        result = NavGoal.Result()

        # 初始檢查：取得位置與路徑
        car_position, car_orientation = (
            self.car_control_node.get_car_position_and_orientation()
        )
        if not car_position:
            self.get_logger().error("Navigation failed: No position data")
            result.success = False
            result.message = "Cannot obtain car position data"
            goal_handle.abort()
            return result

        self._path_points = self.car_control_node.get_path_points(
            dynamic=True, include_orientation=True
        )
        if not self._path_points:
            self.get_logger().error("Navigation failed: No path points available")
            result.success = False
            result.message = "No path available for navigation"
            goal_handle.abort()
            return result

        self.get_logger().info(
            f"Starting navigation with {len(self._path_points)} waypoints"
        )

        # 清除完成旗標，並建立 Timer，每 0.1 秒執行一次 navigation_step
        self._finished_event.clear()
        self._navigation_timer = self.create_timer(0.1, self.navigation_step)

        # 等待 Timer 透過設定 _finished_event 通知導航結束
        while not self._finished_event.is_set():
            time.sleep(0.05)

        # 回傳 Timer 執行完畢後設定的結果
        return self._result

    def navigation_step(self):
        goal_handle = self._goal_handle
        if goal_handle is None:
            return

        # 先檢查是否收到取消請求
        if goal_handle.is_cancel_requested:
            self.get_logger().info("Navigation goal canceled")
            self.car_control_node.publish_control("STOP")
            self._result = NavGoal.Result()
            self._result.success = False
            self._result.message = "Navigation canceled"
            goal_handle.canceled()
            self._navigation_timer.cancel()
            self._finished_event.set()
            return

        # 取得目前位置資訊
        car_position, car_orientation = (
            self.car_control_node.get_car_position_and_orientation()
        )
        if not car_position:
            self.get_logger().warn("Lost position data during navigation")
            self.car_control_node.publish_control("STOP")
            self._result = NavGoal.Result()
            self._result.success = False
            self._result.message = "Lost position data during navigation"
            goal_handle.abort()
            self._navigation_timer.cancel()
            self._finished_event.set()
            return

        # 取前三個座標，並取部分方向資料
        car_position = car_position[0:3]
        car_orientation = car_orientation[2:4]

        # 計算下一個目標點
        target_x, target_y = self.get_next_target_point(
            car_position, path_points=self._path_points
        )
        final_pos = self._path_points[-1]["position"][0:2]
        target_distance = cal_distance(car_position, final_pos)
        if target_x is None or target_distance < 0.5:
            self.get_logger().info(f"Goal reached! Distance: {target_distance:.2f}m")
            self.car_control_node.publish_control("STOP")
            self._result = NavGoal.Result()
            self._result.success = True
            self._result.message = "Navigation completed successfully"
            goal_handle.succeed()
            self._navigation_timer.cancel()
            self._finished_event.set()
            return

        # 計算角度差，決定控制指令
        diff_angle = calculate_diff_angle(
            car_position, car_orientation, target_x, target_y
        )
        if -20 < diff_angle < 20:
            action_key = "FORWARD"
        elif diff_angle < -20 and diff_angle > -180:
            action_key = "CLOCKWISE_ROTATION"
        elif diff_angle > 20 and diff_angle < 180:
            action_key = "COUNTERCLOCKWISE_ROTATION"
        else:
            action_key = "STOP"

        self.car_control_node.publish_control(action_key)

        # 發送回饋訊息
        feedback_msg = NavGoal.Feedback()
        feedback_msg.distance_to_goal = float(target_distance)
        goal_handle.publish_feedback(feedback_msg)

    def get_next_target_point(
        self, car_position, path_points, min_required_distance=0.5
    ):
        if path_points is None:
            self.get_logger().error(
                "Error: global_plan_msg is None or poses is missing!"
            )
            return None, None
        while self.index < len(path_points) - 1:
            target_x = path_points[self.index]["position"][0]
            target_y = path_points[self.index]["position"][1]
            distance_to_target = cal_distance(car_position, (target_x, target_y))
            if distance_to_target < min_required_distance:
                self.index += 1
            else:
                return target_x, target_y
        return None, None
