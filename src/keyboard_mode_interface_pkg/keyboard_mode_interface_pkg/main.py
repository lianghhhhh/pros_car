import urwid
import rclpy
from keyboard_mode_interface_pkg.ros_pub_sub import ROS2Manager


class MenuApp:
    def __init__(self, ros_manager):
        self.ros_manager = ros_manager

        # === 定義選單結構 ===
        self.menu_items = {
            "Control Vehicle": None,
            "Manual Arm Control": {
                "0": None,
                "1": None,
                "2": None,
                "3": None,
                "4": None,
            },
            "Manual Crane Control": {
                "Lift": None,
                "Lower": None,
                "Rotate Left": None,
                "Rotate Right": None,
            },
            "Auto Navigation": {"Start": None, "Stop": None},
            "Automatic Arm Mode": None,
            "Exit": None,
        }

        # === 初始化主選單 ===
        self.menu_stack = []  # 用於儲存 (menu dict, menu title)
        self.current_menu = self.menu_items
        self.current_title = "Main Menu"

        # 顯示標題和底部訊息的元件
        self.header_text = urwid.Text(self.current_title, align="center")
        self.footer_text = urwid.Text("", align="center")

        # 建立主 ListBox
        self.menu = urwid.ListBox(urwid.SimpleFocusListWalker(self.create_menu()))

        # 組合成 Frame
        self.main_frame = urwid.Frame(
            self.menu, header=self.header_text, footer=self.footer_text
        )

    def create_menu(self):
        """根據 self.current_menu 建立動態選單按鈕"""
        menu_widgets = []
        for item in self.current_menu:
            button = urwid.Button(f"< {item}")
            urwid.connect_signal(button, "click", self.menu_selected, item)
            menu_widgets.append(urwid.AttrMap(button, None, focus_map="reversed"))
        return menu_widgets

    def menu_selected(self, button, choice):
        """
        點擊選單選項的回呼：
        - 若對應值是 dict，就進入子選單
        - 若對應值是 None(或其他非 dict)，表示執行指令
        """
        if isinstance(self.current_menu[choice], dict):
            # --- 進入子選單 ---
            self.menu_stack.append((self.current_menu, self.current_title))
            self.current_menu = self.current_menu[choice]
            self.current_title = choice
            self.header_text.set_text(self.current_title)
            self.menu.body = urwid.SimpleFocusListWalker(self.create_menu())

        else:
            # --- 執行指令 ---
            if choice == "Exit":
                raise urwid.ExitMainLoop()  # 直接結束
            else:
                # 先發送 ROS 指令
                self.ros_manager.publish_command(choice)

                # 將「目前的狀態」壓到 stack，表示我們要「進入」一個結果畫面
                # 如此一來，按下 'q' 時會回到這個子選單（而不是又回到更上層）
                self.menu_stack.append((self.current_menu, self.current_title))

                # 重新定義一個「結果畫面」(視為新的子選單)
                self.current_menu = {}
                self.current_title = f"Command Result: {choice}"
                self.header_text.set_text(self.current_title)

                # 直接在畫面上顯示「指令已執行」
                self.menu.body = urwid.SimpleFocusListWalker(
                    [
                        urwid.Text(f"已執行指令: {choice}", align="center"),
                        urwid.Divider(),
                        urwid.Text("按 'q' 回到上一頁", align="center"),
                    ]
                )
                self.footer_text.set_text(f"Command published: {choice}")

    def handle_unhandled_input(self, key):
        """處理未攔截的按鍵，特別是 'q' 用於返回上一層"""
        if key == "q":
            if self.menu_stack:
                # 從堆疊彈回上一層
                prev_menu, prev_title = self.menu_stack.pop()
                self.current_menu = prev_menu
                self.current_title = prev_title
                self.header_text.set_text(self.current_title)
                self.menu.body = urwid.SimpleFocusListWalker(self.create_menu())
            else:
                # 如果沒有上一層可回，就結束程式（或者自行決定行為）
                raise urwid.ExitMainLoop()
        else:
            # 顯示按下了什麼按鍵
            self.footer_text.set_text(f"Pressed key: {key}")

    def run(self):
        """啟動 UI 迴圈"""
        palette = [
            ("reversed", "standout", ""),
            ("header", "white,bold", "dark blue"),
        ]
        loop = urwid.MainLoop(
            widget=self.main_frame,
            palette=palette,
            unhandled_input=self.handle_unhandled_input,
        )
        loop.screen.set_terminal_properties(colors=256)
        loop.run()


def main():
    rclpy.init()
    ros_manager = ROS2Manager()
    app = MenuApp(ros_manager)
    try:
        app.run()
    finally:
        ros_manager.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
