# main.py

## Overview
The main application module that implements a terminal-based user interface for controlling a robotic system. It provides a hierarchical menu system for vehicle control, arm operations, and crane control, integrated with ROS2 for real-time robot communication.

## Key Methods

### MenuApp Class

#### menu_selected(button, choice)
Handles menu item selection and navigation.
- **Parameters:** `button` - The clicked button widget, `choice` - The selected menu item name
- **Behavior:** Navigates to submenus or executes commands based on menu structure

#### handle_unhandled_input(key)
Processes keyboard input not handled by the menu system.
- **Parameters:** `key` - The pressed key
- **Key Functions:** 'q' for navigation back/exit, other keys processed through ModeManager

#### get_big_heading()
Determines the current main menu category.
- **Returns:** String representing the primary menu category

#### get_sub_heading(big_heading)
Determines the current submenu context.
- **Parameters:** `big_heading` - The primary menu category
- **Returns:** Current submenu context or "None"

#### run()
Starts the main UI event loop with urwid configuration.

### main()
Application entry point that sets up ROS2 and UI systems.
- Initializes ROS2 and creates manager instances
- Starts ROS2 spinning in daemon thread
- Runs UI in main thread with proper cleanup

## Usage
```bash
# Start the application
ros2 run keyboard_mode_interface_pkg keyboard_control_node
```

## Key Features
- **Hierarchical Menu System**: Multi-level navigation with breadcrumb support
- **Real-time Control**: Direct keyboard input processing for robot control
- **Multi-threaded Architecture**: UI in main thread, ROS2 in background thread
- **Context-aware Commands**: Menu context determines command interpretation
- **Visual Feedback**: Dynamic header updates and key press display
- **Clean Navigation**: 'q' key for consistent back/exit functionality