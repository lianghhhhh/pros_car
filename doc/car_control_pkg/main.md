# main.py

## Overview
This is the main entry point for the car control system. It initializes and coordinates multiple ROS2 nodes using a multi-threaded executor, enabling concurrent operation of navigation control, manual control, and action server functionality.

## Key Methods

### `main(args=None)`
- **Parameters**: `args` - Command line arguments for ROS2 initialization
- **Functionality**: 
  - Initializes ROS2 system
  - Creates and configures all required nodes
  - Sets up multi-threaded execution environment
  - Handles graceful shutdown on interruption

## Usage
```bash
# Run with ROS2 launch system
ros2 run car_control_pkg car_control_node

```

## Key Features

### Node Initialization
- **NavControlNode**: Primary navigation control with subscriber enablement
- **ManualControlNode**: Handles manual control commands
- **NavigationActionServer**: Provides action-based navigation interface

### Multi-Threaded Architecture
- **Concurrent Execution**: All nodes run simultaneously without blocking
- **Thread Safety**: Uses ROS2's MultiThreadedExecutor for proper synchronization
- **Performance**: Eliminates need for explicit `spin_once()` calls in callbacks

### Robust Lifecycle Management
- **Initialization**: Proper ROS2 system setup with argument passing
- **Execution**: Continuous operation until interruption
- **Cleanup**: Graceful shutdown of all nodes and ROS2 system

### Error Handling
- **Keyboard Interrupt**: Catches Ctrl+C for clean shutdown
- **Resource Cleanup**: Ensures proper node destruction and ROS2 shutdown
- **Logging**: Provides shutdown confirmation messages

### System Integration
- **Node Communication**: Enables inter-node communication through ROS2 topics and actions
- **Shared Resources**: Allows car_control_node sharing with action server
- **Modular Design**: Each node handles specific functionality independently

### Execution Flow
1. **ROS2 Initialization**: Sets up ROS2 runtime environment
2. **Node Creation**: Instantiates all required control nodes
3. **Executor Setup**: Configures multi-threaded execution environment
4. **Node Registration**: Adds all nodes to executor
5. **Continuous Operation**: Runs until interruption
6. **Graceful Shutdown**: Cleans up resources and exits

### Entry Point Configuration
- **Standard Python Entry**: Can be run directly as Python script
- **ROS2 Integration**: Compatible with ROS2 package management
- **Command Line Support**: Accepts ROS2 command line arguments