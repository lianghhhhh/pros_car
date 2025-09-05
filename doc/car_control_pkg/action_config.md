# action_config.py

## Overview
This configuration file defines velocity parameters and action mappings for a four-wheel robotic car control system. It provides predefined movement patterns with different speed settings for various navigation scenarios.

## Key Methods
This file contains only configuration data - no methods are defined.

## Usage
- Import the `ACTION_MAPPINGS` dictionary and velocity constants in other modules
- Used primarily in `ros_receive_and_data_processing/AI_node.py`
- Provides standardized action commands for car movement control

## Key Features

### Velocity Parameters
- **vel**: 10.0 - Normal forward/backward speed
- **vel_slow**: 3.0 - Slow movement speed
- **rotate_vel**: 10.0 - Normal rotation speed
- **rotate_vel_slow**: 2.0 - Slow rotation speed
- **rotate_vel_median**: 5.0 - Medium rotation speed

### Action Mappings
The `ACTION_MAPPINGS` dictionary defines wheel velocities for each wheel (front-left, front-right, rear-left, rear-right):

#### Linear Movement
- **FORWARD**: All wheels move forward at normal speed
- **FORWARD_SLOW**: All wheels move forward at slow speed
- **BACKWARD**: All wheels move backward at normal speed
- **BACKWARD_SLOW**: All wheels move backward at slow speed

#### Directional Movement
- **LEFT_FRONT**: Move diagonally forward-left
- **RIGHT_FRONT**: Move diagonally forward-right
- **LEFT_SHIFT**: Strafe left (lateral movement)
- **RIGHT_SHIFT**: Strafe right (lateral movement)

#### Rotational Movement
- **COUNTERCLOCKWISE_ROTATION**: Rotate left in place (normal speed)
- **COUNTERCLOCKWISE_ROTATION_SLOW**: Rotate left in place (slow speed)
- **COUNTERCLOCKWISE_ROTATION_MEDIAN**: Rotate left in place (medium speed)
- **CLOCKWISE_ROTATION**: Rotate right in place (normal speed)
- **CLOCKWISE_ROTATION_SLOW**: Rotate right in place (slow speed)
- **CLOCKWISE_ROTATION_MEDIAN**: Rotate right in place (medium speed)

#### Control
- **STOP**: Stop all wheel movement

### Wheel Configuration
The system uses a four-wheel configuration with the following arrangement:
- Index 0: Front-left wheel
- Index 1: Front-right wheel  
- Index 2: Rear-left wheel
- Index 3: Rear-right wheel