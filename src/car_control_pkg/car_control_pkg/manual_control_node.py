from std_msgs.msg import String
from car_control_pkg.utils import parse_control_signal
from car_control_pkg.base_car_control_node import BaseCarControlNode

class ManualControlNode(BaseCarControlNode):
    def __init__(self):
        # Initialize the base class with the node name
        super().__init__("manual_control_node")

        self.subscription = self.create_subscription(String, "car_control_signal", self.key_callback, 10)

    # Common methods for all car control nodes
    def key_callback(self, msg):
        """Parse control signal and delegate to handle_command"""
        mode, command = parse_control_signal(msg.data)  # Parse signal
        if mode is None or command is None:
            return

        # Call the handle_command method that derived classes implement
        self.handle_command(mode, command)

    def handle_command(self, mode, command):
        # Only handle Manual Control mode commands
        if mode == "Manual_Control":
            self.key_control(command)

    def key_control(self, key):
        # Implement key mapping logic
        if key == "w":
            self.publish_control("FORWARD")
        elif key == "s":
            self.publish_control("BACKWARD")
        elif key == "a":
            self.publish_control("LEFT_FRONT")
        elif key == "d":
            self.publish_control("RIGHT_FRONT")
        elif key == "e":
            self.publish_control("COUNTERCLOCKWISE_ROTATION")
        elif key == "r":
            self.publish_control("CLOCKWISE_ROTATION")
        elif key == "z" or key == "q":
            self.publish_control("STOP")
