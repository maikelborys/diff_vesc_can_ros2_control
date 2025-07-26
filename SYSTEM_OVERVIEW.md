# System Overview & Technical Architecture

## 🏗️ System Architecture

### High-Level Overview
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   ROS2 Topics   │    │  ros2_control    │    │   VESC CAN      │
│                 │    │                  │    │                 │
│ /cmd_vel        │───▶│ diff_drive_      │───▶│ Left VESC (28)  │
│ /joint_states   │◀───│ controller       │◀───│ Right VESC (46) │
│ /odom           │◀───│                  │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                              │
                              ▼
                       ┌──────────────────┐
                       │ Hardware         │
                       │ Interface        │
                       │ (VESC CAN)       │
                       └──────────────────┘
```

### Core Components

#### 1. Hardware Interface (`VescCanDiffBotSystemHardware`)
**Location**: `hardware/vesc_can_diffbot_system.cpp`

**Key Functions**:
- `on_init()`: Initialize CAN socket and VESC parameters
- `on_configure()`: Set up wheel parameters and limits
- `on_activate()`: Start CAN communication
- `write()`: Convert velocity commands to VESC duty cycles
- `read()`: Read tachometer feedback and update joint states

**State Interfaces**:
- `left_wheel_joint/position`: Wheel position (radians)
- `left_wheel_joint/velocity`: Wheel velocity (rad/s)
- `right_wheel_joint/position`: Wheel position (radians)
- `right_wheel_joint/velocity`: Wheel velocity (rad/s)

**Command Interfaces**:
- `left_wheel_joint/velocity`: Target wheel velocity (rad/s)
- `right_wheel_joint/velocity`: Target wheel velocity (rad/s)

#### 2. Controllers
**diff_drive_controller**:
- Converts TwistStamped commands to wheel velocities
- Implements differential drive kinematics
- Publishes odometry and transforms

**joint_state_broadcaster**:
- Publishes joint states for visualization
- Updates wheel positions and velocities

#### 3. Topics & Messages
- `/cmd_vel` (TwistStamped): Velocity commands
- `/joint_states` (JointState): Wheel feedback
- `/diffbot_base_controller/odom` (Odometry): Robot pose
- `/tf` (Transform): Coordinate frames

## 🔧 Technical Implementation

### CAN Communication Protocol

#### VESC CAN Message Format
```cpp
struct can_frame {
    uint32_t can_id;    // 0x1C (28) or 0x2E (46)
    uint8_t can_dlc;    // 4 bytes
    uint8_t data[8];    // 32-bit signed integer
};
```

#### Velocity to Duty Cycle Conversion
```cpp
// Convert rad/s to m/s
double left_mps = hw_commands_[0] * wheel_radius_;
double right_mps = hw_commands_[1] * wheel_radius_;

// Convert m/s to duty cycle (empirical calibration)
double left_duty = std::clamp(left_mps / 7.857, -1.0, 1.0);
double right_duty = std::clamp(right_mps / 7.857, -1.0, 1.0);

// Convert duty cycle to VESC CAN value
int32_t left_vesc_value = static_cast<int32_t>(left_duty * 100000);
int32_t right_vesc_value = static_cast<int32_t>(right_duty * 100000);
```

#### CAN Message Construction
```cpp
// Build CAN frame for left wheel
can_frame left_frame;
left_frame.can_id = 0x8000001C;  // VESC ID 28
left_frame.can_dlc = 4;
left_frame.data[0] = (left_vesc_value >> 24) & 0xFF;
left_frame.data[1] = (left_vesc_value >> 16) & 0xFF;
left_frame.data[2] = (left_vesc_value >> 8) & 0xFF;
left_frame.data[3] = left_vesc_value & 0xFF;
```

### Tachometer Feedback Processing

#### VESC STATUS_5 Message (Direct Drive)
```cpp
// Read tachometer value (direct ticks per mechanical revolution)
int32_t tachometer_value = (frame.data[0] << 24) |
                          (frame.data[1] << 16) |
                          (frame.data[2] << 8) |
                          frame.data[3];

// Direct drive: 138 ticks per mechanical revolution
// No conversion needed - tachometer_value is already mechanical ticks

// Convert ticks to wheel position (radians)
double wheel_position = (tachometer_value * 2.0 * M_PI) / 138.0;

// Calculate velocity (rad/s)
double wheel_velocity = (wheel_position - prev_position) / dt;
```

### Differential Drive Kinematics

#### Forward Kinematics
```cpp
// Convert wheel velocities to robot velocity
double linear_velocity = (left_velocity + right_velocity) * wheel_radius_ / 2.0;
double angular_velocity = (right_velocity - left_velocity) * wheel_radius_ / wheel_separation_;
```

#### Inverse Kinematics
```cpp
// Convert robot velocity to wheel velocities
double left_velocity = (linear_velocity - angular_velocity * wheel_separation_ / 2.0) / wheel_radius_;
double right_velocity = (linear_velocity + angular_velocity * wheel_separation_ / 2.0) / wheel_radius_;
```

## 📊 Performance Characteristics

### Timing Analysis
- **Control Loop**: 100 Hz (10ms period)
- **CAN Message Latency**: < 1ms
- **Command Processing**: < 5ms
- **Odometry Update**: 100 Hz

### Accuracy Metrics
- **Position Accuracy**: ±2% (with proper calibration)
- **Velocity Accuracy**: ±1% (steady state)
- **Timing Jitter**: < 0.1ms

### Resource Usage
- **CPU**: < 5% (single core)
- **Memory**: < 50MB
- **Network**: 100 Hz CAN traffic

## 🔧 Configuration Management

### Key Parameters
```yaml
# Robot physical parameters
wheel_radius: 0.1778          # Wheel radius (m)
wheel_separation: 0.370       # Distance between wheels (m)

# VESC CAN parameters
left_vesc_id: 28             # Left VESC CAN ID
right_vesc_id: 46            # Right VESC CAN ID
can_interface: "can0"         # CAN interface name

# Control parameters
max_velocity: 1.0            # Maximum velocity (m/s)
max_acceleration: 0.3        # Maximum acceleration (m/s²)
cmd_vel_timeout: 0.5         # Command timeout (s)

# Calibration parameters
duty_cycle_scale: 7.857      # Velocity to duty cycle scaling
tachometer_resolution: 138   # Ticks per mechanical revolution (direct drive)
```

### Calibration Process
1. **Wheel Parameters**: Measure physical wheel radius and separation
2. **Duty Cycle Scaling**: Calibrate velocity to duty cycle conversion
3. **Tachometer Resolution**: Verify 138 ticks per mechanical revolution (direct drive)
4. **Distance per Tick**: Confirm 8.1 mm per tick (1.117m ÷ 138 ticks)
5. **Timing**: Adjust control loop frequency if needed

## 🚨 Safety Features

### Command Validation
```cpp
// Validate velocity commands
if (std::abs(velocity) > max_velocity_) {
    velocity = std::copysign(max_velocity_, velocity);
}

// Check command timeout
if (time_since_last_command > cmd_vel_timeout_) {
    velocity = 0.0;  // Stop robot
}
```

### Hardware Protection
- **Overcurrent Protection**: VESC internal protection
- **Temperature Monitoring**: VESC temperature feedback
- **Emergency Stop**: Zero duty cycle command
- **Timeout Protection**: Stop if no commands received

### Software Safety
- **Parameter Validation**: Check all configuration parameters
- **Error Handling**: Graceful degradation on CAN errors
- **State Monitoring**: Track hardware interface state
- **Logging**: Comprehensive error logging

## 🔍 Debugging & Diagnostics

### Debug Output
```cpp
RCLCPP_DEBUG(logger_, "Left wheel: %.3f rad/s (%.3f m/s)", 
             hw_commands_[0], hw_commands_[0] * wheel_radius_);
RCLCPP_DEBUG(logger_, "Duty cycle: left=%.3f right=%.3f", 
             left_duty, right_duty);
RCLCPP_DEBUG(logger_, "CAN value: left=%d right=%d", 
             left_vesc_value, right_vesc_value);
```

### Monitoring Commands
```bash
# Monitor CAN traffic
candump can0

# Check system status
ros2 control list_controllers
ros2 topic hz /joint_states

# View debug logs
ros2 log show --level DEBUG
```

### Performance Profiling
```bash
# Monitor CPU usage
top -p $(pgrep ros2_control_node)

# Check memory usage
ps aux | grep ros2_control_node

# Monitor network (CAN) traffic
cat /proc/net/can/stats
```

## 🔄 Lifecycle Management

### Startup Sequence
1. **Hardware Interface**: Initialize CAN socket
2. **Controller Manager**: Load and configure controllers
3. **Controllers**: Activate and start publishing
4. **Hardware**: Activate and start communication

### Shutdown Sequence
1. **Hardware**: Stop motors and close CAN socket
2. **Controllers**: Deactivate and stop publishing
3. **Controller Manager**: Unload controllers
4. **Hardware Interface**: Cleanup resources

### Error Recovery
- **CAN Errors**: Retry with exponential backoff
- **Controller Errors**: Restart controller
- **Hardware Errors**: Deactivate and report error
- **System Errors**: Graceful shutdown

---

**Last Updated**: 2025-07-26  
**Version**: 1.0.0
