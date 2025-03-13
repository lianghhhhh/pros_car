import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from custome_interfaces.action import MyCustomAction  # 自訂的 Action
from std_msgs.msg import String
from car_control_pkg.utils import parse_control_signal
import math


class ManualNavNode(Node):
    def __init__(self):
        super().__init__("manual_nav_node")
        # 訂閱 car_control_signal，格式如 "Manual Nav:<command>"
        self.subscription = self.create_subscription(
            String, "car_control_signal", self.key_callback, 10
        )
        # 建立 Action Server 用於 Manual Nav 任務
        self.action_server = ActionServer(
            self,
            MyCustomAction,
            "manual_nav_action",
            self.goal_callback,
            self.cancel_callback,
            self.execute_callback,
        )
        self.current_goal_handle = None
        self.current_mode = None  # 當前應該為 "Manual Nav"
        self.last_signal = None

    def key_callback(self, msg: String):
        mode, key = parse_control_signal(msg.data)
        if len(parts) >= 2 and parts[0] == "Manual Nav":
            self.last_signal = (mode, command)
            # 設定模式
            if self.current_mode is None:
                self.current_mode = mode
                self.get_logger().info(f"Manual Nav 模式設定為: {self.current_mode}")
            self.get_logger().info(f"Manual Nav 收到指令: {parts}")
            # 如果收到 "q"，取消當前 Action
            if command == "q" and self.current_goal_handle is not None:
                self.get_logger().info("Manual Nav: 收到 'q' 指令，取消導航任務")
                self.current_goal_handle.abort()
        else:
            # 若訊號不屬於 Manual Nav，則忽略
            pass

    def goal_callback(self, goal_request):
        self.get_logger().info("Manual Nav: 收到新的 Action 目標，接受目標")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Manual Nav: 收到取消請求")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Manual Nav: 開始執行 Action")
        self.current_goal_handle = goal_handle
        feedback_msg = MyCustomAction.Feedback()
        result = MyCustomAction.Result()
        goal_values = goal_handle.request.goal
        self.get_logger().info(f"Manual Nav: 目標 {goal_values}")

        # 等待模式設定為 Manual Nav
        while self.current_mode != "Manual Nav":
            await rclpy.sleep(0.1)

        i = 1
        while True:
            # 檢查是否收到 "q"
            if self.last_signal:
                _, command = self.last_signal
                if command == "q":
                    self.get_logger().info("Manual Nav: 收到 'q'，取消導航任務")
                    goal_handle.abort()
                    result.success = False
                    result.message = "Action aborted due to 'q' command"
                    self.current_goal_handle = None
                    self.current_mode = None
                    self.last_signal = None
                    return result
                self.last_signal = None

            # 模擬導航回饋，例如距離逐步減少
            feedback_msg.distance_to_goal = 50.0 - i * 2.0
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(
                f"Manual Nav: 迴圈 {i}，回饋距離 {feedback_msg.distance_to_goal}"
            )
            i += 1
            await rclpy.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = ManualNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt，節點關閉中...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
