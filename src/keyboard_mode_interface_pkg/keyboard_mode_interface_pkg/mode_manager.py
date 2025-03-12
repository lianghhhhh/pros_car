class ModeManager:
    def __init__(self, ros_manager):
        self.ros_manager = ros_manager

    def update_mode(self, pressed_key_info):
        if "Control Vehicle" in pressed_key_info:
            print("tdgsaiyud")
        return pressed_key_info
