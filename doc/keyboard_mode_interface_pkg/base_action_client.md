# base_action_client.py

## Overview
Base class for ROS2 action clients that provides common functionality for sending goals, handling responses, and managing action server communication. This class is designed to be inherited by specific action client implementations.

## Key Methods

### send_goal(mode)
Sends a goal to the action server.
- **Parameters:** `mode` - The mode parameter for the goal
- **Returns:** `bool` - True if goal was sent successfully

### cancel_goal()
Cancels the currently active goal.
- **Returns:** `bool` - True if cancellation was requested

### is_server_ready()
- **Returns:** `bool` - True if the action server is ready to accept goals

### has_active_goal()
- **Returns:** `bool` - True if there's currently an active goal

### get_goal_status()
- **Returns:** Current goal status (GoalStatus constant)

### _create_goal_msg(mode) [Abstract]
Must be implemented by derived classes to create goal messages.

## Usage
```python
class MyActionClient(BaseActionClient):
    def __init__(self, node):
        super().__init__(
            node=node,
            action_type=MyAction,
            server_name="my_action_server",
            client_name="MyActionClient"
        )
    
    def _create_goal_msg(self, mode):
        goal_msg = MyAction.Goal()
        goal_msg.mode = mode
        return goal_msg
```

## Key Features
- **Asynchronous Operations**: Non-blocking goal sending with callback handling
- **Action Chaining**: Built-in support for triggering subsequent actions
- **State Management**: Tracks goal handles and status
- **Error Handling**: Comprehensive exception handling and logging
- **Template Pattern**: Base implementation with customizable goal creation