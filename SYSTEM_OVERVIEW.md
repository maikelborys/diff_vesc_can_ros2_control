# System Overview & Technical Architecture

## 🎯 **NEW OBJECTIVE: Modular RL-Ready Robot System**

### **Goal**: Create a stable, modular differential drive robot system that enables:
1. **Seamless Simulation ↔ Hardware Transition**
2. **Reinforcement Learning Training**
3. **Semantic Mapping Implementation**
4. **Step-by-Step Development & Testing**

### **Hardware Setup**:
- **Robot**: Differential drive with 2 VESC motors (CAN)
- **Computer**: ASUS TUF15 with RTX4070
- **Sensors**: Intel RealSense D455 (working with rtabmap)
- **Simulation**: Isaac Sim + Gazebo compatibility

## 🏗️ **Modular System Architecture**

### **High-Level Overview**
```
┌─────────────────────────────────────────────────────────────┐
│                    MODULAR ROBOT SYSTEM                     │
├─────────────────────────────────────────────────────────────┤
│  Application Layer                                          │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │   RL Agent  │ │  Navigation │ │  Semantic   │           │
│  │             │ │   Stack     │ │  Mapping    │           │
│  └─────────────┘ └─────────────┘ └─────────────┘           │
├─────────────────────────────────────────────────────────────┤
│  Control Layer                                              │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │  ros2_control│ │  Trajectory │ │  Safety     │           │
│  │   Framework │ │  Planning   │ │  Monitor    │           │
│  └─────────────┘ └─────────────┘ └─────────────┘           │
├─────────────────────────────────────────────────────────────┤
│  Hardware Abstraction Layer                                 │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │   Real      │ │  Simulation │ │   Mock      │           │
│  │  Hardware   │ │  Interface  │ │  Interface  │           │
│  └─────────────┘ └─────────────┘ └─────────────┘           │
├─────────────────────────────────────────────────────────────┤
│  Physical Layer                                             │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐           │
│  │   VESC      │ │   Isaac     │ │   Gazebo    │           │
│  │   Motors    │ │    Sim      │ │  Physics    │           │
│  └─────────────┘ └─────────────┘ └─────────────┘           │
└─────────────────────────────────────────────────────────────┘
```

### **Current System Architecture** (Phase 1 - Completed)
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

## 🔧 **Technical Implementation**

### **CAN Communication Protocol**

#### VESC CAN Message Format
```
CAN ID: 0x1C (Left VESC ID 28) or 0x2E (Right VESC ID 46)
DLC: 4 bytes
Data: 32-bit signed integer (big-endian)
Range: -100000 to +100000 (representing -100% to +100% duty cycle)
```

#### Velocity to Duty Cycle Conversion
```
duty_cycle = velocity_mps / 7.857
```
**Example**: 1.0 m/s → duty_cycle = 0.127 (12.7%) → CAN value = 12700

### **Tachometer Feedback Processing (Direct Drive)**

#### VESC STATUS_5 Message (Direct Drive)
```
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

### **Differential Drive Kinematics**

#### Forward Kinematics
```
// Convert wheel velocities to robot velocity
double linear_velocity = (left_velocity + right_velocity) * wheel_radius_ / 2.0;
double angular_velocity = (right_velocity - left_velocity) * wheel_radius_ / wheel_separation_;
```

#### Inverse Kinematics
```
// Convert robot velocity to wheel velocities
double left_velocity = (linear_velocity - angular_velocity * wheel_separation_ / 2.0) / wheel_radius_;
double right_velocity = (linear_velocity + angular_velocity * wheel_separation_ / 2.0) / wheel_radius_;
```

## 📊 **Performance Characteristics**

### **Timing Analysis**
- **Control Loop**: 100 Hz (10ms period)
- **CAN Message Latency**: < 1ms
- **Command Processing**: < 5ms
- **Odometry Update**: 100 Hz

### **Accuracy Metrics**
- **Position Accuracy**: ±2% (with proper calibration)
- **Velocity Accuracy**: ±1% (steady state)
- **Timing Jitter**: < 0.1ms

### **Resource Usage**
- **CPU**: < 5% (single core)
- **Memory**: < 50MB
- **Network**: 100 Hz CAN traffic

## 🔧 **Configuration Management**

### **Key Parameters**
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

### **Calibration Process**
1. **Wheel Parameters**: Measure physical wheel radius and separation
2. **Duty Cycle Scaling**: Calibrate velocity to duty cycle conversion
3. **Tachometer Resolution**: Verify 138 ticks per mechanical revolution (direct drive)
4. **Distance per Tick**: Confirm 8.1 mm per tick (1.117m ÷ 138 ticks)
5. **Timing**: Adjust control loop frequency if needed

## 🚨 **Safety Features**

### **Command Validation**
- **Velocity Limits**: Hardware and software limits
- **Command Timeout**: Motors stop if no command received
- **Emergency Stop**: Zero duty cycle command
- **Parameter Validation**: Check all configuration parameters

### **Hardware Protection**
- **Overcurrent Protection**: VESC internal protection
- **Temperature Monitoring**: VESC temperature feedback
- **Timeout Protection**: Stop if no commands received

### **Software Safety**
- **Error Handling**: Graceful degradation on CAN errors
- **State Monitoring**: Track hardware interface state
- **Logging**: Comprehensive error logging

## 🔍 **Debugging & Diagnostics**

### **Debug Output**
- **CAN Communication**: Monitor message sending/receiving
- **Velocity Conversion**: Track command processing
- **Performance Metrics**: Latency and throughput monitoring

### **Monitoring Commands**
```bash
# Monitor CAN traffic
candump can0

# Check system status
ros2 control list_controllers
ros2 topic hz /joint_states

# View debug logs
ros2 log show --level DEBUG
```

### **Performance Profiling**
```bash
# Monitor CPU usage
top -p $(pgrep ros2_control_node)

# Check memory usage
ps aux | grep ros2_control_node

# Monitor network (CAN) traffic
cat /proc/net/can/stats
```

## 🔄 **Lifecycle Management**

### **Startup Sequence**
1. **Hardware Interface**: Initialize CAN socket
2. **Controller Manager**: Load and configure controllers
3. **Controllers**: Activate and start publishing
4. **Hardware**: Activate and start communication

### **Shutdown Sequence**
1. **Hardware**: Stop motors and close CAN socket
2. **Controllers**: Deactivate and stop publishing
3. **Controller Manager**: Unload controllers
4. **Hardware Interface**: Cleanup resources

### **Error Recovery**
- **CAN Errors**: Retry with exponential backoff
- **Controller Errors**: Restart controller
- **Hardware Errors**: Deactivate and report error
- **System Errors**: Graceful shutdown

## 🎯 **Future Development (Phase 2+)**

### **Modular Architecture Goals**
1. **Unified URDF System**: Single robot description for all environments
2. **Parameter Management**: Centralized configuration system
3. **Launch Modularization**: Flexible launch system
4. **Testing Framework**: Comprehensive testing infrastructure

### **Simulation Parity Goals**
1. **Isaac Sim Integration**: NVIDIA Isaac Sim compatibility
2. **Physics Calibration**: Identical behavior between simulation and hardware
3. **Sensor Simulation**: Realistic D455 simulation
4. **Behavior Validation**: Automated testing framework

### **RL Training Infrastructure Goals**
1. **Environment Setup**: Gym-compatible RL environment
2. **Training Pipeline**: Complete training and deployment system
3. **Model Deployment**: Real-time inference on hardware
4. **Safety Integration**: RL agents with safety constraints

### **Semantic Mapping Goals**
1. **Semantic Segmentation**: Real-time object detection
2. **Mapping Algorithms**: Semantic SLAM integration
3. **Visualization Tools**: 3D semantic maps
4. **Performance Optimization**: Real-time processing

## 🏗️ **Core Components**

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

---

**Last Updated**: 2025-07-26  
**Version**: 2.0.0 - Modular RL-Ready System
