# base_car_control_node.py

## Overview
This module provides a base class for car control nodes in a ROS2 environment. It implements common functionality for publishing control commands to both rear and front wheel topics, serving as a foundation for specialized car control implementations.

## Key Methods

### `__init__(self, node_name)`
- Initializes the ROS2 node with the specified name
- Creates publishers for rear and front wheel control topics
- Sets up communication channels for car control commands

### `publish_control(self, action)`
- **Parameters**: 
  - `action`: Either a string (action key) or list (direct velocity values)
- **Functionality**: 
  - Converts string actions to velocity arrays using action mappings
  - Publishes control commands to appropriate wheel topics
  - Handles both unified and split wheel control scenarios

## Usage
```python
from car_control_pkg.base_car_control_node import BaseCarControlNode

class MyCarController(BaseCarControlNode):
    def __init__(self):
        super().__init__("my_car_controller")
        
    def move_forward(self):
        self.publish_control("FORWARD")
        
    def custom_move(self):
        # Direct velocity control [front_left, front_right, rear_left, rear_right]
        self.publish_control([5.0, 5.0, 5.0, 5.0])
```

## Key Features

### Flexible Control Input
- **String Actions**: Accepts predefined action strings (e.g., "FORWARD", "STOP")
- **Direct Velocity**: Accepts velocity arrays for precise control
- **Automatic Conversion**: Converts 2-element arrays to 4-element wheel arrays

### Publisher Management
- **Dual Publisher Support**: Handles both rear and front wheel topics
- **Fallback Mode**: Uses only rear publisher if front publisher unavailable
- **Topic Names**: 
  - `car_C_rear_wheel` - Rear wheel control
  - `car_C_front_wheel` - Front wheel control

### Message Format
- Uses `Float32MultiArray` messages for velocity data
- **Split Mode**: Front wheels [0:2], Rear wheels [2:4]
- **Unified Mode**: All wheels [0:4] to rear topic only

### Debug Logging
- Provides detailed logging of published control data
- Differentiates between split and unified control modes
- Helps with system debugging and monitoring

### Inheritance Design
- Designed as a base class for specialized car controllers
- Provides common functionality while allowing customization
- Promotes code reuse and consistent interfaces across car control nodes