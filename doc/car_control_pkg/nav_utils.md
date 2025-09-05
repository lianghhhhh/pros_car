# nav2_utils.py

## Overview
This utility module provides essential mathematical functions for navigation calculations including coordinate transformations, angle computations, distance calculations, and quaternion operations. It serves as a foundation for navigation algorithms and path planning operations.

## Key Methods

### `get_yaw_from_quaternion(z, w)`
- **Parameters**: 
  - `z` - Z component of quaternion
  - `w` - W component of quaternion  
- **Returns**: Yaw angle in degrees
- **Functionality**: Converts quaternion Z and W components to yaw rotation angle

### `get_direction_vector(current_position, target_position)`
- **Parameters**: 
  - `current_position` - Current coordinates [x, y]
  - `target_position` - Target coordinates [x, y]
- **Returns**: Direction vector as numpy array
- **Functionality**: Calculates vector pointing from current position to target

### `get_angle_to_target(car_yaw, direction_vector)`
- **Parameters**: 
  - `car_yaw` - Current car heading in degrees
  - `direction_vector` - Direction vector to target
- **Returns**: Angle to target in degrees (0-360 range)
- **Functionality**: Computes angle from car heading to target direction

### `calculate_angle_point(car_quaternion_1, car_quaternion_2, car_pos, target_pos)`
- **Parameters**: 
  - `car_quaternion_1`, `car_quaternion_2` - Car orientation quaternion components
  - `car_pos` - Current car position
  - `target_pos` - Target position
- **Returns**: Angle difference in degrees (-180 to 180)
- **Functionality**: Calculates signed angle difference between car heading and target direction

### `quaternion_to_euler(z, w)`
- **Parameters**: 
  - `z` - Z component of quaternion
  - `w` - W component of quaternion
- **Returns**: Yaw angle in radians
- **Functionality**: Converts quaternion to Euler yaw angle using atan2

### `calculate_angle_to_target(vehicle_pos, target_pos, vehicle_orientation)`
- **Parameters**: 
  - `vehicle_pos` - Vehicle position [x, y]
  - `target_pos` - Target position [x, y]
  - `vehicle_orientation` - Vehicle quaternion orientation
- **Returns**: Angle difference in degrees
- **Functionality**: Comprehensive angle calculation with proper angle normalization

### `round_to_decimal_places(data_list, decimal_places=3)`
- **Parameters**: 
  - `data_list` - List of numeric values
  - `decimal_places` - Number of decimal places (default: 3)
- **Returns**: List of rounded values
- **Functionality**: Rounds all values in list to specified decimal places

### `cal_distance(car_pos, target_pos)`
- **Parameters**: 
  - `car_pos` - Car position coordinates
  - `target_pos` - Target position coordinates
- **Returns**: Euclidean distance as float
- **Functionality**: Calculates straight-line distance between two points

### `calculate_diff_angle(car_position, car_orientation, target_point)`
- **Parameters**: 
  - `car_position` - Current car position
  - `car_orientation` - Car orientation quaternion
  - `target_point` - Target point coordinates
- **Returns**: Angle difference in degrees
- **Functionality**: Wrapper function for angle calculation using calculate_angle_point

## Usage
```python
from car_control_pkg.nav2_utils import *

# Calculate distance between points
distance = cal_distance([0, 0], [3, 4])  # Returns 5.0

# Get yaw from quaternion
yaw = get_yaw_from_quaternion(0.707, 0.707)  # Returns 90.0 degrees

# Calculate angle to target
car_pos = [0, 0]
target_pos = [1, 1]
car_orientation = [0, 0.707, 0, 0.707]  # Facing north
angle = calculate_angle_to_target(car_pos, target_pos, car_orientation)

# Round coordinates
coords = [1.23456, 2.34567, 3.45678]
rounded = round_to_decimal_places(coords, 2)  # [1.23, 2.35, 3.46]
```

## Key Features

### Quaternion Operations
- **Yaw Extraction**: Multiple methods for extracting yaw from quaternions
- **Euler Conversion**: Converts quaternion to Euler angles
- **Robust Calculations**: Handles edge cases in quaternion math

### Angle Calculations
- **Signed Angles**: Provides directional angle differences (-180 to 180)
- **Target Bearing**: Calculates bearing from current position to target
- **Angle Normalization**: Ensures angles stay within valid ranges
- **Multiple Algorithms**: Different approaches for various use cases

### Distance Metrics
- **Euclidean Distance**: Standard straight-line distance calculation
- **2D Navigation**: Optimized for ground-based robot navigation
- **Efficient Computation**: Uses math.hypot for numerical stability

### Vector Mathematics
- **Direction Vectors**: Calculates direction from one point to another
- **Numpy Integration**: Uses numpy arrays for efficient vector operations
- **2D and 3D Support**: Handles both 2D navigation and 3D positioning

### Utility Functions
- **Data Formatting**: Rounds floating-point data for display/logging
- **List Processing**: Batch operations on coordinate lists
- **Precision Control**: Configurable decimal place rounding

### Navigation-Specific Calculations
- **Heading Corrections**: Calculates required heading changes
- **Path Planning Support**: Provides primitives for path algorithms
- **Real-time Processing**: Optimized for frequent navigation updates

### Mathematical Robustness
- **Angle Wrapping**: Proper handling of angle discontinuities at ±180°
- **Numerical Stability**: Uses stable mathematical functions
- **Edge Case Handling**: Manages special cases like zero distances

### Coordinate System Support
- **Multiple Frames**: Works with different coordinate systems
- **Consistent Conventions**: Uses standard navigation conventions
- **Transform Ready**: Compatible with ROS coordinate transforms

### Performance Optimization
- **Efficient Algorithms**: Uses optimized mathematical functions
- **Minimal Dependencies**: Relies only on standard math libraries
- **Vectorized Operations**: Supports batch processing where applicable