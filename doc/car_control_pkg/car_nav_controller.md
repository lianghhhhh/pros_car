# car_nav_controller.py

## Overview
This module implements navigation control logic for an autonomous robotic car. It provides two main navigation modes: manual path-following navigation and customize object-tracking navigation, with comprehensive error handling and prerequisite checking.

## Key Methods

### `__init__(self, car_control_node)`
- Initializes the navigation controller with reference to car control node
- Sets up navigation end flag for state management

### `check_prerequisites(self)`
- **Returns**: Either error result or tuple of (car_position, car_orientation, path_points, goal_pose)
- **Functionality**: Validates all required data for navigation execution
- **Error Handling**: Returns detailed error messages for missing prerequisites

### `customize_nav(self)`
- **Functionality**: Object-tracking navigation mode
- **Features**:
  - Tracks objects (specifically "ball") using coordinate data
  - Stops when object distance < 0.3 units
  - Rotates slowly when no object detected
  - Falls back to manual navigation when no coordinates available

### `manual_nav(self)`
- **Functionality**: Path-following navigation mode
- **Features**:
  - Follows predefined path points to reach goal
  - Stops when within 0.5 units of goal
  - Calculates angles and chooses appropriate actions
  - Handles path point progression automatically

### `choose_action_y_offset(self, y_offset, object_depth)`
- **Parameters**: 
  - `y_offset`: Lateral offset of detected object
  - `object_depth`: Distance to object
- **Returns**: Action string for object tracking
- **Logic**: Adjusts movement based on object position and distance

### `choose_action(self, diff_angle)`
- **Parameters**: `diff_angle`: Angle difference to target
- **Returns**: Action string for path following
- **Logic**: Selects forward, clockwise, or counterclockwise rotation based on angle

### `get_next_target_point(self, car_position, path_points, min_required_distance=0.5)`
- **Parameters**:
  - `car_position`: Current car coordinates
  - `path_points`: List of path waypoints
  - `min_required_distance`: Minimum distance threshold
- **Returns**: Tuple of target coordinates and orientation
- **Functionality**: Finds next suitable waypoint along path

## Usage
```python
from car_control_pkg.car_nav_controller import NavigationController

# Initialize with car control node
nav_controller = NavigationController(car_control_node)

# Reset path index
nav_controller.reset_index()

# Execute navigation modes
result = nav_controller.manual_nav()
result = nav_controller.customize_nav()
```

## Key Features

### Dual Navigation Modes
- **Manual Navigation**: Follows predefined path points to goal
- **Customize Navigation**: Tracks and approaches detected objects

### Intelligent Object Tracking
- **Distance-Based Speed Control**: Adjusts movement limits based on object depth
- **Center Alignment**: Keeps object centered using y-offset corrections
- **Safety Stopping**: Stops when object is within 0.3 units

### Robust Path Following
- **Dynamic Waypoint Selection**: Skips nearby waypoints for smoother navigation
- **Angle-Based Control**: Uses angular differences for precise steering
- **Goal Proximity Detection**: Automatically stops near destination

### Error Recovery
- **Prerequisite Validation**: Checks all required data before navigation
- **Fallback Behavior**: Switches between modes based on available data
- **State Management**: Uses navigation end flag for flow control

### Advanced Path Management
- **Index Tracking**: Maintains current position along path
- **Distance Filtering**: Ensures minimum distance between waypoints
- **Final Point Handling**: Always provides valid target even if criteria not met

### Flexible Action Selection
- **Object Tracking Actions**: `FORWARD_SLOW`, rotation for alignment
- **Path Following Actions**: `FORWARD`, `CLOCKWISE_ROTATION`, `COUNTERCLOCKWISE_ROTATION`
- **Safety Actions**: `STOP` for completion and obstacle avoidance