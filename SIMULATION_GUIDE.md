# Simulation Guide

## 🎯 Overview

This guide covers simulation options for the DiffBot VESC CAN system, including Gazebo physics simulation and RViz visualization.

## 🚀 Quick Start

### Option 1: RViz Visualization (Recommended)
```bash
# Launch with RViz for visualization
ros2 launch diff_vesc_can_ros2_control diffbot_gazebo_rviz.launch.py
```

**Features**:
- ✅ Complete robot model visualization
- ✅ Real-time movement display
- ✅ Odometry tracking
- ✅ Joint state monitoring
- ✅ Transform tree visualization

### Option 2: Pure Gazebo Simulation
```bash
# Launch with Gazebo physics simulation
ros2 launch diff_vesc_can_ros2_control diffbot_gazebo_pure.launch.py
```

**Features**:
- ✅ Physics-based simulation
- ✅ Visual robot movement
- ✅ Collision detection
- ✅ Realistic dynamics

## 📊 Comparison: RViz vs Gazebo

| Feature | RViz | Gazebo |
|---------|------|--------|
| **Visualization** | ✅ Complete model | ⚠️ Basic model |
| **Physics** | ❌ None | ✅ Full physics |
| **Performance** | ✅ Fast | ⚠️ Resource intensive |
| **Real-time** | ✅ Excellent | ✅ Good |
| **Setup** | ✅ Simple | ⚠️ Complex |

## 🔧 Configuration

### RViz Configuration
**File**: `rviz/diffbot.rviz`

**Key Displays**:
- **RobotModel**: Shows complete robot with wheels
- **TF**: Coordinate frame visualization
- **Odometry**: Robot movement tracking
- **Grid**: Reference grid

### Gazebo Configuration
**File**: `description/urdf/diffbot_gazebo.urdf.xacro`

**Key Features**:
- **Differential Drive Plugin**: Enables wheel movement
- **Materials**: Proper colors and textures
- **Physics**: Realistic mass and inertia
- **Collision**: Proper collision geometry

## 🎮 Control Commands

### Basic Movement
```bash
# Forward movement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --once

# Circle movement
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" --once
```

### Continuous Movement
```bash
# Continuous forward movement (10 Hz)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --rate 10
```

## 📈 Monitoring

### Topic Monitoring
```bash
# Monitor joint states
ros2 topic echo /joint_states

# Monitor odometry
ros2 topic echo /diffbot_base_controller/odom

# Check topic frequency
ros2 topic hz /joint_states
ros2 topic hz /diffbot_base_controller/odom
```

### System Status
```bash
# List all nodes
ros2 node list

# Check controller status
ros2 control list_controllers

# View system logs
ros2 log show
```

## 🔍 Troubleshooting

### RViz Issues
**Problem**: Robot not visible
```bash
# Check robot description
ros2 topic echo /robot_description

# Verify TF tree
ros2 run tf2_tools view_frames
```

**Problem**: No movement in RViz
```bash
# Check joint states
ros2 topic echo /joint_states

# Verify controller status
ros2 control list_controllers
```

### Gazebo Issues
**Problem**: Gray box instead of robot
- **Solution**: Use RViz for visualization
- **Alternative**: Implement proper Gazebo plugins

**Problem**: Robot not moving in Gazebo
```bash
# Check if Gazebo plugin is loaded
ros2 topic list | grep gazebo

# Verify CAN messages are being sent
ros2 topic echo /cmd_vel
```

## 🎯 Best Practices

### For Development
1. **Use RViz for visualization** - faster and more reliable
2. **Test control logic** - commands work the same in simulation and reality
3. **Monitor topics** - verify data flow is correct
4. **Use continuous commands** - test smooth movement

### For Testing
1. **Start with low velocities** - 0.1-0.3 m/s
2. **Test all movement types** - forward, backward, turning
3. **Verify odometry** - check position tracking
4. **Test safety features** - command timeouts, limits

## 🔗 Integration

### Navigation Stack
```bash
# Launch with navigation (future enhancement)
ros2 launch nav2_bringup bringup_launch.py
```

### SLAM
```bash
# Launch with SLAM (future enhancement)
ros2 launch slam_toolbox online_async_launch.py
```

## 📚 Advanced Features

### Custom Worlds
Create custom Gazebo worlds in `worlds/` directory:
```xml
<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="custom_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <!-- Add custom obstacles here -->
  </world>
</sdf>
```

### Custom Robot Models
Modify `description/urdf/diffbot_description.urdf.xacro` to:
- Add sensors (LIDAR, camera, IMU)
- Change robot dimensions
- Add additional links/joints

---

**Status**: ✅ Fully Functional | **Last Updated**: 2025-07-26
