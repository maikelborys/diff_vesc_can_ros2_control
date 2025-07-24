# System Overview and Hardware Interface Development Guide

This document is the single source of truth for the current state, architecture, and development roadmap of the `diff_vesc_can_ros2_control` project. It is designed to be updated as the project evolves, providing a clear view of what is implemented, what is in progress, and what is planned.

---

## 1. Understand Existing Structure

- The package already has two hardware interfaces:
  - `DiffBotSystemHardware` (likely for simulation or generic diffbot)
  - `VESCDiffBotSystemHardware` (intended for real VESC CAN hardware, but the .cpp is empty)
- Both inherit from `hardware_interface::SystemInterface` and are designed for use with `ros2_control`.

---

## 2. What Your Custom Hardware Interface Must Do

- **Send CAN commands** to each wheel (left/right) using the VESC protocol.
- **Read odometry** and **real wheel positions** from the VESC tachometer (CAN feedback).
- **Integrate with ros2_control**: expose state and command interfaces for each wheel.
- **Be modular**: allow switching between simulation and real hardware.

---

## 3. Implementation Plan

### A. Create a New Hardware Interface Class
- Name suggestion: `VESCRealDiffBotSystemHardware` (or similar).
- Place in `hardware/include/diff_vesc_can_ros2_control/vesc_real_diffbot_system.hpp` and `hardware/vesc_real_diffbot_system.cpp`.

### B. Inherit from SystemInterface
- Implement all required methods:
  - `on_init`, `on_configure`, `on_activate`, `on_deactivate`
  - `export_state_interfaces`, `export_command_interfaces`
  - `read`, `write`

### C. CAN Communication Layer
- Use or adapt code from your working VESC CAN bridge (e.g., from `cmdvel2can`).
- On `write()`: Convert velocity commands to VESC CAN duty cycle and send to each wheel.
- On `read()`: Read CAN feedback (tachometer) and update wheel position/velocity.

### D. State and Command Interfaces
- For each wheel joint:
  - **State**: position (from tachometer), velocity (from CAN feedback)
  - **Command**: velocity (to be converted to duty cycle and sent via CAN)

### E. Parameterization
- Read parameters for:
  - CAN interface name
  - VESC IDs for left/right
  - Wheel radius, wheel separation
  - Any scaling factors (duty/velocity, tacho ticks/rev, etc.)

### F. Lifecycle Management
- Ensure safe startup/shutdown (e.g., stop motors on deactivate).
- Handle CAN errors gracefully.

### G. Testing and Debugging
- Add debug output for CAN messages sent/received.
- Test with both simulation and real hardware.

---

## 4. How to Integrate

- Register your new hardware interface as a plugin in the package XML.
- Update the `ros2_control` .xacro and YAML configs to use your new interface.
- Test with the `diff_drive_controller` and joint state broadcaster.

---

## 5. Reference Existing Code

- Use `diffbot_system.cpp` and `vesc_diffbot_system.hpp` as templates for structure.
- Use your proven CAN message builder and VESC protocol code for the actual CAN communication.

---

## 6. Documentation

- Document your new interface in the README and SYSTEM_OVERVIEW.md.
- Add usage instructions and troubleshooting tips.

---

## 7. Summary Table

| Step | Action |
|------|--------|
| 1 | Create new hardware interface class (header + cpp) |
| 2 | Implement SystemInterface methods |
| 3 | Integrate CAN send/receive for VESC |
| 4 | Expose state/command interfaces for wheels |
| 5 | Parameterize all hardware details |
| 6 | Register as plugin, update configs |
| 7 | Test with real hardware and simulation |
| 8 | Document everything |

---

**This approach will give you a robust, production-ready hardware interface for your DIFFBOT with VESC CAN, fully integrated with ros2_control and ready for both real and simulated operation.**

_Last updated: 2025-07-24_
