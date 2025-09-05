# mode_manager.py

## Overview
The ModeManager class serves as the central command interpreter and dispatcher for the robotic system. It processes user input from the terminal UI and translates it into appropriate ROS2 commands for vehicle control, arm operations, and other system functions.

## Key Methods

### update_mode(pressed_key_info)
Main method that processes user input and dispatches appropriate commands based on the menu context and key pressed.

**Parameters:**
- `pressed_key_info` - String in format `{big_heading}:{sub_heading}:{key}`

**Processing Logic:**
- **Control Vehicle**: Handles navigation goals and manual control signals
- **Manual Arm Control**: Publishes direct arm joint control commands  
- **Automatic Arm Mode**: Sends high-level arm operation goals
- **Manual Crane Control**: Placeholder (not implemented)

## Usage
```python
# Initialize mode manager
ros_manager = ROS2Manager()
mode_manager = ModeManager(ros_manager)

# Process user input from UI
pressed_key_info = "Control Vehicle:Manual_Nav:w"
mode_manager.update_mode(pressed_key_info)

# Navigation with arm coordination
pressed_key_info = "Automatic Arm Mode:catch:Enter"
mode_manager.update_mode(pressed_key_info)
```

## Key Features
- **Command Interpretation**: Translates UI input into ROS2 commands
- **Multi-Modal Control**: Handles both real-time and goal-based robot control
- **Vehicle-Arm Coordination**: Automatically coordinates navigation with arm operations
- **Context-Aware Processing**: Different behavior based on menu context
- **Cancellation Support**: 'q' key cancels active operations across all modes
- **Input Filtering**: Ignores unwanted key events like "down" navigation