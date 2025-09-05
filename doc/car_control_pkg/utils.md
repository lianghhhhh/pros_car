# utils.py

## Overview
This utility module provides essential helper functions for the car control system. It includes action mapping retrieval and control signal parsing functionality, serving as a bridge between configuration data and runtime control operations.

## Key Methods

### `get_action_mapping(action_name)`
- **Parameters**: `action_name` - String identifier for the desired action
- **Returns**: List of velocity values [front_left, front_right, rear_left, rear_right]
- **Functionality**: 
  - Retrieves velocity configuration for specified action from ACTION_MAPPINGS
  - Provides interface to action configuration without direct import
  - Returns wheel velocity array for four-wheel control system

### `parse_control_signal(signal_str)`
- **Parameters**: `signal_str` - Control signal string in "mode:command" format
- **Returns**: Tuple of (mode, keyboard_command) or (None, None) for invalid input
- **Functionality**: 
  - Parses incoming control signals from ROS topics
  - Extracts mode and command components from formatted strings
  - Converts command to lowercase for consistent processing
  - Handles malformed input gracefully

## Usage
```python
from car_control_pkg.utils import get_action_mapping, parse_control_signal

# Get action mapping for movement
forward_velocities = get_action_mapping("FORWARD")
# Returns: [10.0, 10.0, 10.0, 10.0]

stop_velocities = get_action_mapping("STOP")
# Returns: [0.0, 0.0, 0.0, 0.0]

# Parse control signals
mode, command = parse_control_signal("Manual_Control:w")
# Returns: ("Manual_Control", "w")

mode, command = parse_control_signal("Navigation:start")
# Returns: ("Navigation", "start")

# Handle invalid input
mode, command = parse_control_signal("invalid_format")
# Returns: (None, None)
```

## Key Features

### Action Mapping Interface
- **Configuration Access**: Provides clean interface to ACTION_MAPPINGS dictionary
- **Centralized Control**: Single point of access for velocity configurations
- **Type Safety**: Returns consistent velocity array format
- **Error Handling**: Raises KeyError for invalid action names

### Signal Parsing
- **Protocol Support**: Implements "mode:command" communication protocol
- **Case Normalization**: Converts commands to lowercase automatically
- **Whitespace Handling**: Strips whitespace from parsed components
- **Robust Parsing**: Handles various input formats gracefully

### Integration Design
- **Modular Architecture**: Separates utility functions from main logic
- **Import Optimization**: Reduces direct dependencies on configuration files
- **Reusable Components**: Functions can be used across multiple modules
- **Clean Interfaces**: Simple function signatures for easy integration

### Error Handling
- **Graceful Degradation**: Returns None values for invalid input
- **No Exceptions**: Parse function doesn't raise exceptions for bad input
- **Validation Support**: Enables calling code to check for valid parsing

### Communication Protocol
- **Standardized Format**: Enforces consistent signal format across system
- **Mode Separation**: Cleanly separates control modes from specific commands
- **Extensible Design**: Easy to add new modes and commands
- **Topic Compatible**: Designed for ROS2 String message topics

### Data Processing
- **String Manipulation**: Handles common string processing tasks
- **Format Validation**: Ensures minimum format requirements are met
- **Component Extraction**: Separates compound data into usable parts
- **Preprocessing**: Prepares data for consumption by other modules

### Performance Characteristics
- **Lightweight Functions**: Minimal computational overhead
- **No Side Effects**: Pure functions without global state changes
- **Fast Execution**: Optimized for frequent calls
- **Memory Efficient**: No unnecessary data copying or storage

### Usage Patterns
- **Callback Integration**: Designed for use in ROS callback functions
- **Real-time Processing**: Suitable for real-time control applications
- **Batch Processing**: Can process multiple signals efficiently
- **Error Recovery**: Supports systems that need to handle bad input

### System Integration
- **ROS2 Compatible**: Designed for ROS2 message processing
- **Topic Bridge**: Connects topic data to internal control systems
- **Protocol Implementation**: Implements communication protocol specification
- **Interoperability**: Works with different control node implementations