# action_server_handler.py

## Overview
This module provides a generic handler for ROS2 action server results. It centralizes the logging and processing of action goal status updates across different action clients.

## Key Methods

### handle_action_result(node, status, result)
Generic navigation result handler that processes action server responses.

**Parameters:**
- `node` - The ROS node instance (for logging)
- `status` - GoalStatus integer value
- `result` - The actual result from the action server

**Behavior:**
- Logs the goal completion status with human-readable status name
- Provides specific logging for SUCCESS, ABORTED, and CANCELED states

## Usage
```python
handle_action_result(
    node=self._node,
    status=goal_result.status,
    result=goal_result.result
)
```

## Key Features
- **Consistent Logging**: Standardized status reporting across all action clients
- **Human-Readable Status**: Converts GoalStatus constants to readable strings
- **Extensible Design**: Placeholder for emergency stops and system resets
- **Centralized Processing**: Single point for action result handling