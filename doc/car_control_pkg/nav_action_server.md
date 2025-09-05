# nav_action_server.py

## Overview
This module implements a ROS2 Action Server for navigation control, providing a standardized interface for executing navigation tasks. It coordinates between client requests and the navigation controller to handle autonomous navigation with proper goal management and cancellation support.

## Key Methods

### `__init__(self, car_control_node)`
- **Parameters**: `car_control_node` - Reference to the main car control node
- **Functionality**: 
  - Initializes ROS2 action server with NavGoal action type
  - Creates navigation controller instance
  - Sets up callback handlers for goals and cancellation

### `goal_callback(self, goal_request)`
- **Parameters**: `goal_request` - Incoming navigation goal request
- **Returns**: `GoalResponse.ACCEPT` or `GoalResponse.REJECT`
- **Functionality**: 
  - Validates goal requests based on mode
  - Checks prerequisites for "Manual_Nav" mode
  - Provides early rejection for invalid requests

### `cancel_callback(self, goal_handle)`
- **Parameters**: `goal_handle` - Handle to the goal being cancelled
- **Returns**: `CancelResponse.ACCEPT`
- **Functionality**: 
  - Immediately stops car movement
  - Accepts all cancellation requests
  - Provides clean termination of navigation

### `execute_callback(self, goal_handle)`
- **Parameters**: `goal_handle` - Handle containing goal information and state
- **Returns**: `NavGoal.Result` - Navigation execution result
- **Functionality**: 
  - Main navigation execution loop
  - Handles mode selection and method delegation
  - Manages goal state transitions and feedback

### `_select_car_auto_method(self, mode)`
- **Parameters**: `mode` - Navigation mode string
- **Returns**: Method reference for the specified navigation mode
- **Functionality**: Maps mode strings to navigation controller methods

## Usage
```python
# Typically instantiated in main.py
from car_control_pkg.nav_action_server import NavigationActionServer

action_server = NavigationActionServer(car_control_node)

# Used with ROS2 action clients
# Example client command:
# ros2 action send_goal /nav_action_server action_interface/NavGoal "{mode: 'Manual_Nav'}"
```

## Key Features

### Action Server Interface
- **Action Type**: Uses `NavGoal` action from action_interface package
- **Server Name**: `nav_action_server`
- **Standard Compliance**: Follows ROS2 action server patterns

### Goal Management
- **Goal Validation**: Checks prerequisites before accepting goals
- **Mode Support**: Handles multiple navigation modes
- **Early Rejection**: Rejects invalid goals before execution starts

### Navigation Modes
- **Manual_Nav**: Path-following navigation using predefined waypoints
- **Customize_Nav**: Object-tracking navigation with real-time adjustments
- **Extensible**: Easy to add new navigation modes

### Execution Control
- **Rate-Limited Loop**: Runs at 10Hz for consistent performance
- **Cancellation Support**: Responsive to client cancellation requests
- **State Management**: Proper goal state transitions (succeed/abort/cancel)

### Feedback System
- **Real-Time Feedback**: Publishes navigation progress to clients
- **Distance Reporting**: Currently reports distance to goal (placeholder)
- **Status Updates**: Keeps clients informed of navigation progress

### Error Handling
- **Prerequisite Checking**: Validates required data before navigation
- **Graceful Failure**: Provides meaningful error messages
- **Safe Termination**: Ensures car stops on errors or cancellation

### Integration Points
- **Navigation Controller**: Delegates actual navigation logic
- **Car Control Node**: Accesses car state and control methods
- **ROS2 Ecosystem**: Standard action interface for client integration

### Execution Flow
1. **Goal Reception**: Client submits navigation goal request
2. **Validation**: Check mode and prerequisites
3. **Acceptance/Rejection**: Respond to goal request
4. **Execution Loop**: Continuous navigation with feedback
5. **State Transition**: Success, failure, or cancellation handling
6. **Cleanup**: Stop movement and clean up resources

### Threading Considerations
- **Multi-Threaded Safe**: Designed for multi-threaded executor
- **No Blocking**: Avoids blocking calls in execute_callback
- **Rate Control**: Uses ROS2 rate for consistent timing