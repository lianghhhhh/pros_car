# arm_action_client.py

## Overview
Specialized action client for controlling robotic arm operations. Inherits from `BaseActionClient` and provides arm-specific functionality for sending arm control goals and managing arm actions.

## Key Methods

### send_arm_mode(mode)
Sends an arm mode goal to the action server.
- **Parameters:** `mode` - The arm operation mode to execute
- **Returns:** `bool` - True if goal was sent successfully

### cancel_arm()
Cancels the current arm goal if one exists.
- **Returns:** `bool` - True if cancellation was requested

### _create_goal_msg(mode)
Creates an arm goal message with the specified mode.
- **Parameters:** `mode` - The arm operation mode (string)
- **Returns:** `ArmGoal.Goal()` - Configured goal message

## Usage
```python
# Initialize the arm action client
arm_client = ArmActionClient(node)

# Send arm commands
arm_client.send_arm_mode("catch")
arm_client.send_arm_mode("wave")
arm_client.send_arm_mode("init_pose")

# Cancel current operation
arm_client.cancel_arm()

# Check server status
if arm_client.is_server_ready():
    arm_client.send_arm_mode("init_pose")
```

## Key Features
- **High-Level Control**: Supports complex arm operations like "catch", "wave", "init_pose"
- **Directional Movement**: Supports basic movements (up, down, left, right, forward, backward)
- **Action Integration**: Seamlessly integrates with navigation operations
- **Cancellation Support**: Can interrupt ongoing arm operations
- **Server Status Checking**: Verifies arm server availability before sending goals