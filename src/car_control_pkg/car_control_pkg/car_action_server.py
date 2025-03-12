import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ModeManager(Node):
    def __init__(self):
        super().__init__("mode_manager")
        # 訂閱 car_control_signal topic，預期格式例如 "Manual Control:go" 或 "Manual Nav:go"
        self.subscription = self.create_subscription(
            String, "car_control_signal", self.key_callback, 10
        )
        # 建立 timer 定期檢查模式，這裡設定 0.5 秒執行一次
        self.timer = self.create_timer(0.5, self.mode_loop)
        # 儲存最新接收到的訊號，格式為 [mode_name, command]
        self.last_signal = None
        # 當前模式（例如 "Manual Control" 或 "Manual Nav"）
        self.current_mode = None

    def key_callback(self, msg: String):
        # 將收到的字串以冒號分隔並去除前後空白
        car_control_signal = [s.strip() for s in msg.data.split(":")]
        if len(car_control_signal) >= 2:
            self.last_signal = car_control_signal
            self.get_logger().info(f"收到訊號: {car_control_signal}")
        else:
            self.get_logger().warning("收到格式錯誤的訊號")

    def mode_loop(self):
        """
        每次 timer 觸發時：
          - 檢查是否有新的訊號，若有則根據訊號決定切換模式或退出模式
          - 若沒有新的訊號，則持續執行目前模式的動作
        """
        if self.last_signal:
            mode_name, command = self.last_signal[0], self.last_signal[1].lower()
            # 如果指令為 "q"，則停止目前模式
            if command == "q":
                self.get_logger().info("收到 'q' 指令，停止目前模式")
                self.stop_mode()
                self.last_signal = None
                return
            else:
                # 如果模式變更，則切換到新模式
                if self.current_mode != mode_name:
                    self.current_mode = mode_name
                    self.get_logger().info(f"切換到模式: {self.current_mode}")
            # 清除 last_signal，避免重複處理
            self.last_signal = None

        # 根據目前模式執行對應動作
        if self.current_mode == "Manual Control":
            self.manual_control_action()
        elif self.current_mode == "Manual Nav":
            self.manual_nav_action()
        else:
            self.get_logger().info("尚未收到有效模式指令或模式已停止")

    def manual_control_action(self):
        """
        Manual Control 模式：使用鍵盤操控
        此處僅為模板，你可以在這裡加入鍵盤讀取與控制邏輯
        """
        self.get_logger().info("Manual Control 運作中...（請實作鍵盤操控邏輯）")

    def manual_nav_action(self):
        """
        Manual Nav 模式：訂閱其他 package 的 service 取得資料後作決定
        此處僅為模板，你可以在這裡補上呼叫 service 的邏輯
        """
        self.get_logger().info("Manual Nav 運作中...（請實作呼叫 service 的邏輯）")
        # 範例：
        # self.call_nav_service()

    def stop_mode(self):
        """
        當收到 'q' 指令時呼叫此方法停止目前模式，
        可在此處加入額外的停止後處理邏輯（例如清除狀態、復位等）。
        """
        if self.current_mode:
            self.get_logger().info(f"停止模式: {self.current_mode}")
            self.current_mode = None
        else:
            self.get_logger().info("目前無模式執行中")


def main(args=None):
    rclpy.init(args=args)
    mode_manager = ModeManager()
    try:
        rclpy.spin(mode_manager)
    except KeyboardInterrupt:
        mode_manager.get_logger().info("Keyboard Interrupt，節點關閉中...")
    finally:
        mode_manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
