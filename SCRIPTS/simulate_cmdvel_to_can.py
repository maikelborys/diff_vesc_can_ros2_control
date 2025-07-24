import struct

def velocity_to_duty_cycle(velocity_mps):
    # Empirical calibration: duty_cycle = velocity_mps / 7.857
    return max(min(velocity_mps / 7.857, 1.0), -1.0)

def duty_cycle_to_bytes(duty_cycle):
    # VESC expects int32_t, duty_cycle * 100000, big-endian
    val = int(duty_cycle * 100000)
    return [(val >> 24) & 0xFF, (val >> 16) & 0xFF, (val >> 8) & 0xFF, val & 0xFF]

def simulate_cmdvel_to_can(linear_x, angular_z, wheel_separation=0.37):
    # Differential drive kinematics
    left_vel = linear_x - (angular_z * wheel_separation / 2.0)
    right_vel = linear_x + (angular_z * wheel_separation / 2.0)
    left_duty = velocity_to_duty_cycle(left_vel)
    right_duty = velocity_to_duty_cycle(right_vel)
    left_bytes = duty_cycle_to_bytes(left_duty)
    right_bytes = duty_cycle_to_bytes(right_duty)
    print(f"Input: linear.x={linear_x:.3f} angular.z={angular_z:.3f}")
    print(f"Left wheel velocity: {left_vel:.4f} m/s, duty: {left_duty:.4f}, CAN bytes: {left_bytes}")
    print(f"Right wheel velocity: {right_vel:.4f} m/s, duty: {right_duty:.4f}, CAN bytes: {right_bytes}")

# Example usage:
simulate_cmdvel_to_can(0.05, 0.0)   # Forward slow
simulate_cmdvel_to_can(0.0, 0.5)    # Rotate in place
simulate_cmdvel_to_can(0.2, 0.0)    # Forward faster
simulate_cmdvel_to_can(0.0, 0.0)    # Stop