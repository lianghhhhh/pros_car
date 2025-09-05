# ros2_manager.py

## Overview
The ROS2Manager class serves as the central ROS2 communication hub for the robotic system. It extends the ROS2 Node class and manages both publishers for real-time control signals and action clients for complex, goal-oriented operations.

## Key Methods

### publish_car_signal(command)
Publishes car control commands to the car control signal topic.
- **Parameters:** `command` - String command to send (e.g., "Manual_Control:w")
- **Topic:** `"car_control_signal"`

### publish_arm_signal(command)
Publishes arm control commands to the arm control signal topic.
- **Parameters:** `command` - String command to send (e.g., "ManualArmControl:1")
- **Topic:** `"arm_control_signal"`

### car_action_client
Provides access to the CarActionClient for navigation operations.
- **Available Methods:** `send_navigation_goal()`, `cancel_navigation_goal()`, `is_navigation_successful()`

### arm_action_client
Provides access to the ArmActionClient for arm operations.
- **Available Methods:** `send_arm_mode()`, `cancel_arm()`

## Usage
```python
# Initialize ROS2 manager
ros_manager = ROS2Manager()

# Manual control via publishers
ros_manager.publish_car_signal("Manual_Control:w")
ros_manager.publish_arm_signal("ManualArmControl:1")

# Autonomous operations via action clients
ros_manager.car_action_client.send_navigation_goal("Auto_Nav")
ros_manager.arm_action_client.send_arm_mode("catch")

# Coordinated operations
ros_manager.car_action_client.send_navigation_goal("Manual_Nav")
ros_manager.arm_action_client.send_arm_mode("catch")
```

## Key Features
- **Dual Communication Modes**: Publishers for real-time control, action clients for goal-based operations
- **Centralized Management**: Single point for all ROS2 communication
- **Multi-threaded Support**: Designed for concurrent UI and ROS2 operations
- **Action Client Integration**: Built-in car and arm action clients
- **Topic-based Publishing**: Immediate control signal transmission
- `cancel_arm()` - Cancel current arm operation

## Communication Architecture

### Publishers (Real-time Control)
Used for immediate, continuous control commands:
```
ROS2Manager → car_control_signal → Vehicle Control Node
ROS2Manager → arm_control_signal → Arm Control Node
```

### Action Clients (Goal-oriented Operations)
Used for complex operations with feedback and status:
```
ROS2Manager → nav_action_server → Navigation System
ROS2Manager → arm_action_server → Arm Control System
```

## Usage Patterns

### Manual Control
```python
# Direct control via publishers
ros_manager.publish_car_signal("Manual_Control:w")
ros_manager.publish_arm_signal("ManualArmControl:1")
```

### Autonomous Operations
```python
# Goal-based control via action clients
ros_manager.car_action_client.send_navigation_goal("Auto_Nav")
ros_manager.arm_action_client.send_arm_mode("catch")
```

### Coordinated Operations
```python
# Combined navigation and arm operations
ros_manager.car_action_client.send_navigation_goal("Manual_Nav")
ros_manager.arm_action_client.send_arm_mode("catch")
```

## Integration
The ROS2Manager is typically:
1. Created in the main application (main.py)
2. Passed to ModeManager for command dispatching
3. Run in a separate thread for ROS2 spinning
4. Accessed through the terminal UI system

**Example Integration:**
```python
# From main.py
rclpy.init()
ros_manager = ROS2Manager()
mode_manager = ModeManager(ros_manager)

# Start ROS spinning in separate thread
ros_spin_thread = threading.Thread(target=lambda: rclpy.spin(ros_manager))
ros_spin_thread.start()
```

## Topic Configuration

| Publisher | Topic Name | Message Type | Purpose |
|-----------|------------|--------------|---------|
| car_control_publisher | car_control_signal | String | Real-time vehicle control |
| arm_control_publisher | arm_control_signal | String | Real-time arm control |

## Action Server Connections

| Client | Server Name | Action Type | Purpose |
|--------|-------------|-------------|---------|
| car_action_client | nav_action_server | NavGoal | Navigation operations |
| arm_action_client | arm_action_server | ArmGoal | Arm operations |

## Notes
- Centralizes all ROS2 communication for the application
- Provides both immediate control (publishers) and complex operations (action clients)
- Designed for multi-threaded operation with UI systems
- Extensible design allows for easy addition of new communication channels