# diff_vesc_can_ros2_control - Independent VESC CAN Robot Control Package

**DiffBot VESC CAN** (*Differential Mobile Robot with VESC CAN Control*) is a complete ros2_control implementation for differential drive robots using VESC motor controllers over CAN bus. This package provides a self-contained, independent implementation with no external dependencies.

## Features

- ✅ **VESC CAN Integration**: Direct interface with VESC motor controllers via CAN bus
- ✅ **Complete Independence**: No external dependencies - fully self-contained package  
- ✅ **Differential Drive Control**: Proper implementation of diff drive kinematics
- ✅ **Hardware Interface**: Real VESC hardware integration with fallback to mock hardware
- ✅ **RViz Visualization**: Real-time robot visualization with independent URDF and materials
- ✅ **Odometry Publishing**: Accurate pose estimation and velocity tracking
- ✅ **TwistStamped Commands**: Modern ROS2 message interface with timestamp support
- ✅ **Joint State Broadcasting**: Real-time wheel position and velocity feedback
- ✅ **Production Ready**: Clean, tested implementation for real robot deployment

## Architecture

### Hardware Interface
- **DiffBotSystemHardware**: Implements `hardware_interface::SystemInterface` for VESC CAN
- **State Interfaces**: Position and velocity for each wheel joint
- **Command Interfaces**: Velocity commands sent directly to VESC controllers
- **CAN Communication**: Direct interface with VESC motor controllers
- **Lifecycle Management**: Proper activation/deactivation with VESC initialization

### Controllers
- **diff_drive_controller**: Converts Twist commands to wheel velocities
- **joint_state_broadcaster**: Publishes joint states for visualization

### Topics
- `/cmd_vel` (geometry_msgs/TwistStamped): Robot velocity commands
- `/diffbot_base_controller/odom` (nav_msgs/Odometry): Robot pose and twist
- `/joint_states` (sensor_msgs/JointState): Wheel positions and velocities
- `/tf` and `/tf_static`: Transform tree for visualization

## Quick Start

## Quick Start

### 1. Build the Package
```bash
cd /home/robot/robot_ws
colcon build --packages-select diff_vesc_can_ros2_control
source install/setup.bash
```

### 2. Launch Robot Visualization
```bash
ros2 launch diff_vesc_can_ros2_control view_robot.launch.py
```

### 3. Launch Complete Robot System
```bash
ros2 launch diff_vesc_can_ros2_control diffbot.launch.py
```

### 4. Control the Robot
```bash
# Send velocity commands (linear.x = 0.2 m/s, angular.z = 0.1 rad/s)
# ************************ WORKS *********************************
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: now, frame_id: base_link}, \
    twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, \
           angular: {x: 0.0, y: 0.0, z: 0.1}}}" --rate 1

# Stop the robot
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: now, frame_id: base_link}, \
    twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, \
           angular: {x: 0.0, y: 0.0, z: 0.0}}}" --once
```

## Package Structure

```
diff_vesc_can_ros2_control/
├── bringup/
│   ├── config/
│   │   └── diffbot_controllers.yaml     # Controller configuration
│   └── launch/
│       └── diffbot.launch.py           # Main system launch
├── description/
│   ├── launch/
│   │   └── view_robot.launch.py        # Robot visualization
│   ├── ros2_control/
│   │   └── diffbot.ros2_control.xacro  # Hardware interface config
│   └── urdf/
│       ├── diffbot.urdf.xacro          # Robot description
│       ├── diffbot_description.urdf.xacro
│       └── diffbot.materials.xacro     # Visual materials
├── hardware/
│   ├── include/diff_vesc_can_ros2_control/
│   │   └── diffbot_system.hpp          # Hardware interface header
│   └── diffbot_system.cpp              # VESC hardware implementation
├── rviz/
│   ├── diffbot.rviz                    # RViz configuration
│   └── diffbot_view.rviz              # Visualization config
└── README.md                           # This file
```

This will start:
- Controller manager with VESC CAN interface
- Hardware interface (real VESC or mock)
- Differential drive controller
- Joint state broadcaster
- RViz visualization
- Robot state publisher

### 3. Control the Robot

**Forward Movement:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, 
    twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" -r 10
```

**Rotation:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, 
    twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}" -r 10
```

**Combined Movement (Arc):**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, 
    twist: {linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}}" -r 10
```

### 4. Monitor Robot State

**Joint States:**
```bash
ros2 topic echo /joint_states
```

**Odometry:**
```bash
ros2 topic echo /diffbot_base_controller/odom
```

**Controller Status:**
```bash
ros2 control list_controllers
```

## Integration with VESC Hardware

This package serves as a reference for implementing real hardware interfaces. In our workspace, we have:

- `diff_vesc_can_ros2_pkg_cpp`: Production VESC CAN communication
- `modular_diffbot_control`: Real robot control implementation

### Key Learnings for VESC Integration

1. **Hardware Interface Pattern**: Study `hardware/diffbot_system.cpp` for proper lifecycle management
2. **Command Processing**: See how `write()` method handles velocity commands
3. **State Publishing**: Understand how `read()` method updates joint states
4. **Controller Configuration**: Review the YAML configuration patterns
5. **Launch File Structure**: Analyze the launch file organization

## Code Structure

```
ros2_control_diffbot_original/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package dependencies
├── README.md                   # This file
├── hardware/
│   ├── diffbot_system.cpp      # Hardware interface implementation
│   └── include/
│       └── ros2_control_diffbot_original/
│           └── diffbot_system.hpp
├── bringup/
│   ├── launch/
│   │   ├── diffbot.launch.py   # Main launch file
│   │   └── view_robot.launch.py
│   └── config/
│       └── diffbot_controllers.yaml
├── description/
│   └── urdf/
│       ├── diffbot_description.urdf.xacro
│       └── diffbot.ros2_control.xacro
└── ros2_control_diffbot_original.xml  # Plugin declaration
```

## Educational Value

This package demonstrates:

1. **ros2_control Framework**: Complete implementation from hardware interface to visualization
2. **Differential Drive Kinematics**: Mathematical model to wheel commands
3. **ROS2 Best Practices**: Proper package structure, lifecycle management, and communication patterns
4. **Real-time Control**: High-frequency control loops and state updates
5. **Simulation to Reality**: Bridge between simulated and real hardware

## Monitoring and Debugging

### Check System Status
```bash
# List all nodes
ros2 node list

# Check controller status
ros2 control list_controllers

# Monitor topics
ros2 topic list
ros2 topic hz /joint_states
ros2 topic hz /diffbot_base_controller/odom
```

### Performance Metrics
- **Control Frequency**: 100Hz (configurable)
- **Joint State Publishing**: 100Hz
- **Odometry Publishing**: 100Hz
- **Command Processing**: Real-time response

## Related Packages in Workspace

- **diff_vesc_can_ros2_pkg_cpp**: Real VESC hardware interface with CAN communication
- **modular_diffbot_control**: Production robot control system
- **ros2_control_demo_description**: Robot URDF descriptions

## Next Steps

1. **Study the Code**: Understand the hardware interface implementation
2. **Modify Parameters**: Experiment with wheel radius, base width, etc.
3. **Extend Functionality**: Add sensors, implement custom controllers
4. **Hardware Integration**: Use this as a template for real VESC implementation

## Documentation References

- [ros2_control Documentation](https://control.ros.org/)
- [diff_drive_controller Documentation](https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
- [Writing a Hardware Interface](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_interface.html)

---

**Status**: ✅ Fully functional and tested  
