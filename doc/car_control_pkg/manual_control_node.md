# manual_control_node.py

## Overview
This module implements a manual control node that allows keyboard-based control of the robotic car. It extends the BaseCarControlNode to provide real-time manual control capabilities through ROS2 topic subscriptions and keyboard mapping.

## Key Methods

### `__init__(self)`
- Initializes the manual control node with name "manual_control_node"
- Sets up subscription to "car_control_signal" topic
- Inherits publisher functionality from BaseCarControlNode

### `key_callback(self, msg)`
- **Parameters**: `msg` - String message containing control signal
- **Functionality**: 
  - Parses incoming control signals using utility function
  - Delegates to command handler based on parsed mode and command
  - Provides error handling for invalid signals

### `handle_command(self, mode, command)`
- **Parameters**: 
  - `mode` - Control mode (filters for "Manual_Control")
  - `command` - Specific command to execute
- **Functionality**: Filters commands and delegates to key control handler

### `key_control(self, key)`
- **Parameters**: `key` - Single character key press
- **Functionality**: Maps keyboard inputs to car movement actions
- **Key Mapping**: Translates keys to predefined action strings

## Usage
```python
# Node runs automatically when instantiated in main.py
# Controlled via published messages to "car_control_signal" topic

# Example message format: "Manual_Control:w"
# This would trigger forward movement

# Direct usage (not typical):
from car_control_pkg.manual_control_node import ManualControlNode
node = ManualControlNode()
```

## Key Features

### Keyboard Control Mapping
- **w**: `FORWARD` - Move forward
- **s**: `BACKWARD` - Move backward  
- **a**: `LEFT_FRONT` - Move diagonally forward-left
- **d**: `RIGHT_FRONT` - Move diagonally forward-right
- **e**: `COUNTERCLOCKWISE_ROTATION` - Rotate left in place
- **r**: `CLOCKWISE_ROTATION` - Rotate right in place
- **z** or **q**: `STOP` - Stop all movement

### Mode Filtering
- **Selective Processing**: Only responds to "Manual_Control" mode commands
- **Mode Isolation**: Ignores commands from other control modes
- **Clean Separation**: Maintains clear boundaries between control systems

### Real-Time Control
- **Topic Subscription**: Listens to "car_control_signal" for commands
- **Immediate Response**: Processes commands as soon as they arrive
- **Low Latency**: Direct mapping from key to action without complex processing

### Integration Design
- **Base Class Extension**: Inherits publishing capabilities from BaseCarControlNode
- **Utility Integration**: Uses shared parsing utilities for consistent signal handling
- **Publisher Reuse**: Leverages existing wheel control publishers

### Error Handling
- **Signal Validation**: Checks for valid mode and command parsing
- **Graceful Degradation**: Ignores invalid or malformed signals
- **No Error Propagation**: Invalid inputs don't crash the system

### Communication Protocol
- **Topic Interface**: Uses standard ROS2 String messages
- **Signal Format**: Expects "mode:command" format strings
- **Standard Integration**: Compatible with external control interfaces

### Extensibility
- **Modular Design**: Easy to add new key mappings
- **Override Capability**: Methods can be overridden for customization
- **Base Functionality**: Can be extended for more complex manual control scenarios