# car_action_client.py

## Overview
Specialized action client for controlling vehicle navigation operations. Inherits from `BaseActionClient` and provides car-specific functionality for sending navigation goals, managing navigation actions, and coordinating with subsequent operations.

## Key Methods

### send_navigation_goal(mode)
Sends a navigation goal with the specified mode to the action server.
- **Parameters:** `mode` - The navigation mode to execute
- **Returns:** `bool` - True if goal was sent successfully

### cancel_navigation_goal()
Cancels the current navigation goal if one exists.
- **Returns:** `bool` - True if cancellation was requested

### is_navigation_successful()
Checks if the last navigation goal completed successfully.
- **Returns:** `bool` - True if the last goal status was SUCCESS

### start_next_action(next_action=None)
Executes specified follow-up actions after navigation completion.
- **Parameters:** `next_action` - Function or lambda to execute after navigation completion

### _create_goal_msg(mode)
Creates a navigation goal message with the specified mode.
- **Parameters:** `mode` - The navigation mode (string)
- **Returns:** `NavGoal.Goal()` - Configured goal message

## Usage
```python
# Initialize the car action client
car_client = CarActionClient(node)

# Basic navigation
car_client.send_navigation_goal("Auto_Nav")

# Navigation with follow-up action
def after_navigation():
    print("Navigation completed, starting next task...")

car_client.send_navigation_goal("Manual_Nav")
car_client.start_next_action(after_navigation)

# Status checking
if car_client.is_navigation_successful():
    print("Navigation completed successfully")
```

## Key Features
- **Multiple Navigation Modes**: Supports Manual_Nav, Auto_Nav, and Customize_Nav
- **Action Chaining**: Built-in support for executing follow-up actions after navigation
- **Status Monitoring**: Can check navigation success status
- **Flexible Integration**: Works with arm operations for coordinated robotics tasks
- **Error Handling**: Comprehensive error logging for next action execution