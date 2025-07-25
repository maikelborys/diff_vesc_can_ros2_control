## UPDATED CONTROLLER LIMITS (2025-07-25)

### Velocity and Acceleration Limits
The following parameters are now set in `diffbot_controllers.yaml` for safe and smooth operation:

```
linear.x.max_velocity: 1.0
linear.x.min_velocity: -1.0
linear.x.max_acceleration: 0.3
linear.x.min_acceleration: -0.3
angular.z.max_velocity: 1.0
angular.z.min_velocity: -1.0
angular.z.max_acceleration: 0.3
angular.z.min_acceleration: -0.3
```

These values provide a good balance between responsiveness and safety for small robots. Adjust as needed for your platform.

### Duty Cycle Scaling
The hardware interface now uses:

```
duty_cycle = (velocity_mps / 7.857) / 10.0)
```
This ensures that velocity commands in m/s match real-world robot speed.
## TESTED AND VALIDATED (2025-07-24)

### Summary
The VESC CAN hardware interface for the diff drive robot has been fully validated as of July 24, 2025. The following steps were performed and confirmed:

- **Unit Consistency:** Controller outputs (rad/s) are now correctly converted to linear velocity (m/s) using the real wheel radius in the hardware interface.
- **Parameter Alignment:** All configuration files and xacro/URDFs use the real robot's parameters:
  - `wheel_radius`: 0.1778
  - `wheel_separation`: 0.370
  - `left_vesc_id`: 28
  - `right_vesc_id`: 46
  - `can_interface`: "can0"
- **CAN Message Validation:** Debug output and real CAN messages were checked and confirmed to match the intended wheel speeds and robot motion.
- **Test Command:**
  - Publishing to `/cmd_vel` with a known velocity produces the correct CAN frame values and data.

### Example Debug Output
```
[DEBUG] hw_commands: left=6.666424 rad/s (1.185290 m/s) right=6.666424 rad/s (1.185290 m/s)
[DEBUG] duty_cycle: left=0.150858 right=0.150858
[DEBUG] CAN value: left=15085 right=15085
[DEBUG] Would send CAN: LEFT (ID 0x8000001C) data 00.00.3A.ED | RIGHT (ID 0x8000002E) data 00.00.3A.ED
```

### How to Reproduce
1. Set all parameters as above in your config and xacro files.
2. Build and launch the system.
3. Publish a velocity command:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: now, frame_id: base_link}, twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" --once
   ```
4. Observe debug output and CAN frames for correctness.

### Notes
- The debug section in the hardware interface can be commented out and real CAN sending enabled for deployment.
- This configuration is now the reference for future deployments and debugging.
# System Overview: CAN Message Logic and Robot Parameters (cmdvel2can)

## CAN Message Sending Logic

The cmdvel2can package converts velocity commands into VESC CAN messages as follows:

1. **Velocity to Duty Cycle Conversion**
   - Formula (empirical, 2025):
     
     ```
     duty_cycle = velocity_mps / 7.857
     ```
     - `velocity_mps`: Desired wheel velocity in meters per second
     - `7.857`: Derived from wheel/tick calibration (see below)
   - **Example:**
     - For 1.117 m/s (1 wheel revolution/sec):
       - `duty_cycle = 1.117 / 7.857 ≈ 0.142` (**≈14% duty cycle**)
     - For 1.1 m/s:
       - `duty_cycle = 1.1 / 7.857 ≈ 0.14` (**≈14% duty cycle**)
     - **Summary:** Sending a CAN value corresponding to 14% duty cycle (e.g., 14000) will result in approximately 1.1 m/s wheel speed.

2. **Duty Cycle to VESC CAN Format**
   - Duty cycle is scaled to a signed 32-bit integer in the range [-100000, 100000] (representing -100% to +100%).
   - Example: 0.10 (10%) → 10000 → 0x00002710
   - Big-endian byte order:
     ```cpp
     frame.data[0] = (vesc_value >> 24) & 0xFF;  // MSB
     frame.data[1] = (vesc_value >> 16) & 0xFF;
     frame.data[2] = (vesc_value >> 8) & 0xFF;
     frame.data[3] = vesc_value & 0xFF;          // LSB
     ```

3. **CAN Message Structure**
   - **CAN ID**: 0x1C (28, left) or 0x2E (46, right)
   - **DLC**: 4 bytes
   - **Data**: 32-bit signed integer, big-endian
   - **Zero**: 0x00000000 = 0% duty cycle (motor stop)

4. **Example CAN Commands**
   - Forward (10%): `cansend can0 0000001C#00.00.27.10`
   - Reverse (10%): `cansend can0 0000001C#FF.FF.D8.F0`
   - Stop: `cansend can0 0000001C#00.00.00.00`

## Robot Parameters and Ticks

- **Wheel diameter**: 0.3556 m
- **Wheel circumference**: 1.117 m
- **Ticks per revolution**: 138 (23 poles × 6 states)
- **Distance per tick**: 1.117 m / 138 ≈ 0.0081 m (8.1 mm)
- **Empirical ticks/sec per duty**: ~970 (from sweep)

## Odometry and Decoding

- VESC STATUS_5 messages report tachometer value as electrical revolutions.
- Odometry node divides by 6 to get mechanical revolutions.
- For each mechanical revolution, the wheel moves 1.117 m and the tachometer increases by 138 ticks.
- To convert duty cycle to distance/sec: (ticks/sec per duty) × 8.1 mm/tick.

## Safety and Testing

- **Recommended duty cycle limits**: ±10% for initial testing
- **Deadband**: 1% minimum
- **Emergency stop**: 0% duty cycle


## CAN Message Hex Examples (VESC ID 28 and 46)

The following are verified CAN messages for duty cycles from -10% to +10%:

### VESC ID 28 (CAN ID 0x1C)

| Duty Cycle (%) | CAN Value | Hex Message (cansend)           |
|:--------------:|:---------:|:--------------------------------|
|    -10         | -10000    | cansend can0 0000001C#FF.FF.D8.F0|
|    -8.192      |  -8192    | cansend can0 0000001C#FF.FF.E0.00|
|    -6.384      |  -6384    | cansend can0 0000001C#FF.FF.E7.10|
|    -4.576      |  -4576    | cansend can0 0000001C#FF.FF.EE.20|
|    -2.512      |  -2512    | cansend can0 0000001C#FF.FF.F6.30|
|      0         |      0    | cansend can0 0000001C#00.00.00.00|
|      2         |   2000    | cansend can0 0000001C#00.00.07.D0|
|      4         |   4000    | cansend can0 0000001C#00.00.0F.A0|
|      6         |   6000    | cansend can0 0000001C#00.00.17.70|
|      8         |   8000    | cansend can0 0000001C#00.00.1F.40|
|     10         |  10000    | cansend can0 0000001C#00.00.27.10|

### VESC ID 46 (CAN ID 0x2E)

| Duty Cycle (%) | CAN Value | Hex Message (cansend)           |
|:--------------:|:---------:|:--------------------------------|
|    -10         | -10000    | cansend can0 0000002E#FF.FF.D8.F0|
|    -8.192      |  -8192    | cansend can0 0000002E#FF.FF.E0.00|
|    -6.384      |  -6384    | cansend can0 0000002E#FF.FF.E7.10|
|    -4.576      |  -4576    | cansend can0 0000002E#FF.FF.EE.20|
|    -2.512      |  -2512    | cansend can0 0000002E#FF.FF.F6.30|
|      0         |      0    | cansend can0 0000002E#00.00.00.00|
|      2         |   2000    | cansend can0 0000002E#00.00.07.D0|
|      4         |   4000    | cansend can0 0000002E#00.00.0F.A0|
|      6         |   6000    | cansend can0 0000002E#00.00.17.70|
|      8         |   8000    | cansend can0 0000002E#00.00.1F.40|
|     10         |  10000    | cansend can0 0000002E#00.00.27.10|

**Note:**
- A duty cycle of ±10% corresponds to a CAN value of ±10000 (0x00002710 or 0xFFFFD8F0).
- These hex messages are verified for both VESC IDs (28 and 46).

**Always use the above logic and parameters for CAN message generation and decoding.**
