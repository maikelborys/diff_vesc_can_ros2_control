# DiffBot Simulation Integration Guide

## Goal
Simulate your robot in Gazebo with the same control interface, parameters, and behaviour as your real hardware. This enables you to test navigation, control, and perception in a virtual environment before deploying to the real robot.

---

## Step-by-Step Plan

### 1. Prepare the Robot Description (URDF/Xacro)
- Ensure your URDF/xacro describes all links, joints, inertias, and collision/visual geometry.
- Wheel radius and separation must match your real robot and controller config.
- Add the Gazebo ROS2 control plugin:
  ```xml
  <gazebo>
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so"/>
  </gazebo>
  ```
- (Optional) Add simulated sensors (LIDAR, IMU, etc.) as plugins.

### 2. Create a Gazebo Launch File
- Create a launch file (e.g. `diffbot_gazebo.launch.py`) that:
  - Launches Gazebo (with or without GUI)
  - Spawns your robot using the URDF/xacro
  - Loads the same controller config as hardware (`diffbot_controllers.yaml`)
  - Example snippet:
    ```python
    from launch import LaunchDescription
    from launch_ros.actions import Node
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    import os

    def generate_launch_description():
        return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': Command(['xacro ', PathJoinSubstitution([...])])}]
            ),
            # Controller manager, spawners, etc.
        ])
    ```

### 3. Set Simulation Parameters
- In your controller YAML:
  - `use_sim_time: true`
  - `open_loop: true` (unless you want simulated encoders)
- Make sure all parameters (wheel radius, separation, joint names) match your real robot.

### 4. Launch the Simulation
```bash
ros2 launch diff_vesc_can_ros2_control diffbot_gazebo.launch.py
```

### 5. Control and Visualize
- Send `/cmd_vel` commands as with hardware.
- Use RViz or Gazebo GUI to visualize robot movement.
- Monitor `/odom`, `/joint_states`, and controller status.

### 6. (Optional) Add Navigation Stack
- Launch `nav2_bringup` or your navigation stack for full autonomy testing.

---

## Behaviour Explanation
- **Controller behaviour:**
  - In simulation, `diff_drive_controller` receives `/cmd_vel` and publishes wheel commands to the simulated robot.
  - With `open_loop: true`, odometry is computed from commanded velocities (not simulated encoders).
  - With `open_loop: false` and simulated encoders, odometry is computed from joint positions.
- **Robot behaviour:**
  - The robot in Gazebo will move according to the same physical parameters as your real robot.
  - All velocity/acceleration limits, timeouts, and frame IDs are respected.
  - You can test navigation, obstacle avoidance, and sensor integration exactly as on hardware.
- **Switching between simulation and hardware:**
  - Only change `use_sim_time` and `open_loop` in your YAML.
  - All other parameters and launch commands remain the same.

---

## Best Practices
- Always keep URDF/xacro, controller config, and hardware in sync.
- Test all navigation and control logic in simulation before deploying to real robot.
- Use RViz and Gazebo for visualization and debugging.
- Document any changes to simulation or hardware setup.

---

For more details, see ROS2 and Gazebo documentation, or ask for a template launch file or URDF snippet.
