# nav_control_node.py

## Overview
This module implements the main navigation control node that manages all navigation-related data and communication. It extends BaseCarControlNode to provide comprehensive navigation functionality including AMCL pose tracking, goal management, path planning, object detection integration, and velocity command processing.

## Key Methods

### `__init__(self, enable_nav_subscribers=False)`
- **Parameters**: `enable_nav_subscribers` - Boolean to enable navigation topic subscriptions
- **Functionality**: 
  - Initializes base car control functionality
  - Creates publishers for clearing navigation plans and goals
  - Optionally creates navigation data subscribers
  - Initializes data storage for navigation state

### `clear_goal_pose(self)`
- **Functionality**: 
  - Publishes empty PoseStamped message to clear goal
  - Resets internal goal pose storage
  - Provides clean goal cancellation mechanism

### `clear_plan(self)`
- **Functionality**: 
  - Publishes empty Path message to clear navigation plan
  - Resets internal plan storage
  - Clears current path planning data

### `get_car_position_and_orientation(self)`
- **Returns**: Tuple of (position, orientation) or (None, None)
- **Functionality**: 
  - Retrieves current car pose from AMCL data
  - Provides position and orientation information for navigation
  - Handles cases where pose data is unavailable

### `get_goal_pose(self)`
- **Returns**: Goal position object or None
- **Functionality**: 
  - Retrieves current navigation goal position
  - Handles missing or malformed goal data
  - Provides safe access to goal information

### `get_path_points(self, include_orientation=True)`
- **Parameters**: `include_orientation` - Whether to include orientation data
- **Returns**: List of path points with position and optionally orientation
- **Functionality**: 
  - Extracts waypoints from global navigation plan
  - Converts ROS messages to Python data structures
  - Supports both position-only and full pose data

### `get_latest_object_coordinates(self, label=None)`
- **Parameters**: `label` - Specific object label to retrieve (optional)
- **Returns**: Dictionary of object coordinates or specific object data
- **Functionality**: 
  - Provides access to YOLO object detection results
  - Supports both single object and all objects retrieval
  - Returns 3D coordinates in FLU frame

### `cmd_vel_callback(self, msg)`
- **Parameters**: `msg` - Twist message with linear and angular velocity
- **Functionality**: 
  - Converts cmd_vel to differential drive wheel speeds
  - Applies speed limits and safety constraints
  - Calculates left and right wheel velocities

## Usage
```python
# Initialize navigation control node
nav_node = NavControlNode(enable_nav_subscribers=True)

# Get car position
position, orientation = nav_node.get_car_position_and_orientation()

# Get path points
path_points = nav_node.get_path_points(include_orientation=True)

# Get object coordinates
ball_coords = nav_node.get_latest_object_coordinates("ball")
all_objects = nav_node.get_latest_object_coordinates()

# Clear navigation data
nav_node.clear_plan()
nav_node.clear_goal_pose()
```

## Key Features

### Comprehensive Data Management
- **AMCL Pose**: Real-time localization data storage
- **Goal Pose**: Current navigation target management
- **Global Plan**: Path planning data from navigation stack
- **Camera Depth**: Depth sensor data integration
- **Object Detection**: YOLO-based object coordinate tracking
- **Velocity Commands**: Cmd_vel to wheel speed conversion

### Topic Subscriptions
- **`/amcl_pose`**: Localization updates from AMCL
- **`/goal_pose`**: Navigation goal from planning system
- **`/received_global_plan`**: Global path from Nav2 planner
- **`/cmd_vel`**: Velocity commands from navigation
- **`/camera/x_multi_depth_values`**: Depth camera data
- **`/yolo/object/offset`**: Object detection coordinates

### Topic Publishers
- **`/plan`**: For clearing navigation plans
- **`/goal_pose`**: For clearing navigation goals
- **Wheel Control**: Inherited from BaseCarControlNode

### Object Detection Integration
- **JSON Parsing**: Processes YOLO detection results
- **3D Coordinates**: Stores object positions in FLU frame
- **Label Mapping**: Maps object labels to coordinate data
- **Error Handling**: Robust parsing with validation

### Velocity Processing
- **Differential Drive**: Converts twist to wheel speeds
- **Speed Limiting**: Applies maximum and minimum speed constraints
- **Wheel Distance**: Configurable wheel separation parameter
- **Safety Bounds**: Prevents excessive wheel speeds

### Navigation State Management
- **Data Persistence**: Maintains latest navigation data
- **Clean Shutdown**: Provides methods to clear navigation state
- **Thread Safety**: Designed for multi-threaded ROS2 execution
- **Error Recovery**: Handles missing or malformed data gracefully

### Flexible Initialization
- **Optional Subscribers**: Can disable navigation subscriptions for testing
- **Modular Design**: Supports different operational modes
- **Base Class Integration**: Inherits core car control functionality

### Coordinate System Support
- **FLU Frame**: Forward-Left-Up coordinate system for objects
- **Map Frame**: Global coordinate system for navigation
- **Base Link**: Robot-centric coordinate transformations

### Data Access Methods
- **Position Queries**: Easy access to car and goal positions
- **Path Information**: Structured path point data
- **Object Tracking**: Real-time object coordinate access
- **Velocity Monitoring**: Current velocity command tracking