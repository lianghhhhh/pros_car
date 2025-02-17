import urwid
import rclpy
from keyboard_mode_interface_pkg.ros_pub_sub import ROS2Manager


class MenuApp:
    def __init__(self, ros_manager):
        self.ros_manager = ros_manager  # ROS 2 管理器

        # **定義選單結構**
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

        # **初始化主選單**
        self.menu_stack = []  # 儲存選單歷史
        self.current_menu = self.menu_items

        self.menu = urwid.ListBox(urwid.SimpleFocusListWalker(self.create_menu()))
        self.main_frame = urwid.Frame(self.menu)

    def create_menu(self):
        """根據 `self.current_menu` 建立動態選單"""
        menu_widgets = []
        for item in self.current_menu:
            button = urwid.Button(f"< {item}")
            urwid.connect_signal(button, "click", self.menu_selected, item)
            menu_widgets.append(urwid.AttrMap(button, None, focus_map="reversed"))

        # **如果不在主選單，則加上返回 `Back` 按鈕**
        if self.menu_stack:
            back_button = urwid.Button("< Back")
            urwid.connect_signal(back_button, "click", self.go_back)
            menu_widgets.append(urwid.AttrMap(back_button, None, focus_map="reversed"))

        return menu_widgets

    def menu_selected(self, button, choice):
        """當使用者選擇選單選項時的行為"""
        if isinstance(self.current_menu[choice], dict):
            # **進入子選單**
            self.menu_stack.append(self.current_menu)  # 儲存當前選單
            self.current_menu = self.current_menu[choice]  # 切換到子選單
        else:
            # **發送 ROS 2 指令**
            self.ros_manager.publish_command(choice)

        # **更新 UI**
        self.menu.body = urwid.SimpleFocusListWalker(self.create_menu())

    def go_back(self, button):
        """返回上一層選單"""
        if self.menu_stack:
            self.current_menu = self.menu_stack.pop()  # 回到上一層

        # **更新 UI**
        self.menu.body = urwid.SimpleFocusListWalker(self.create_menu())

    def run(self):
        """啟動 UI 介面"""
        urwid.MainLoop(self.main_frame, palette=[("reversed", "standout", "")]).run()


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
