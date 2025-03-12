import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ModeManager(Node):
    def __init__(self):
        super().__init__("mode_manager")
        # 訂閱來自 car_control_signal 主題的訊號
        self.subscription = self.create_subscription(
            String, "car_control_signal", self.key_callback, 10
        )
        # 建立一個 timer，每秒觸發一次 mode_loop
        self.timer = self.create_timer(1.0, self.mode_loop)
        # 儲存最後接收到的訊號 (格式為 [mode_name, command])
        self.last_signal = None
        # 目前模式名稱
        self.current_mode = None

    def key_callback(self, msg: String):
        # 解析收到的字串，假設格式為 "模式名稱:指令"
        car_control_signal = [s.strip() for s in msg.data.split(":")]
        if len(car_control_signal) >= 2:
            self.last_signal = car_control_signal
        else:
            self.get_logger().warning("收到格式錯誤的訊號")

    def mode_loop(self):
        """
        每次 timer 觸發時：
         - 檢查是否有新的訊號，若有則根據訊號決定切換模式或退出模式
         - 若沒有新的訊號，則繼續使用目前模式
        """
        if self.last_signal:
            mode_name, command = self.last_signal[0], self.last_signal[1].lower()

            # 若指令為 "q"，則退出目前模式
            if command == "q":
                if self.current_mode:
                    self.get_logger().info(
                        f"收到中斷訊號 'q'，結束模式 {self.current_mode}"
                    )
                    self.current_mode = None
                else:
                    self.get_logger().info("收到中斷訊號 'q'，但目前沒有模式在執行")
                # 清除 last_signal 避免重複處理
                self.last_signal = None
                return
            else:
                # 如果模式改變，則切換到新的模式
                if self.current_mode != mode_name:
                    self.current_mode = mode_name
                    self.get_logger().info(f"切換到模式: {self.current_mode}")

        # 根據目前模式執行對應動作
        if self.current_mode:
            if self.current_mode == "Manual Control":
                self.get_logger().info("模式 Manual Control 運作中...")
            elif self.current_mode == "Manual Nav":
                self.get_logger().info("模式 Manual Nav 運作中...")
            else:
                self.get_logger().info("未知模式..." + self.current_mode)
        else:
            self.get_logger().info("尚未收到模式指令或模式已結束")


def main(args=None):
    rclpy.init(args=args)
    mode_manager = ModeManager()

    try:
        rclpy.spin(mode_manager)
    except KeyboardInterrupt:
        mode_manager.get_logger().info("Keyboard Interrupt，準備關閉...")
    finally:
        mode_manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
