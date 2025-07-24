
#include "diff_vesc_can_ros2_control/vesc_can_diffbot_system.hpp"
#include <limits>
#include <vector>
#include <string>

namespace diff_vesc_can_ros2_control {

VescCanDiffBotSystemHardware::VescCanDiffBotSystemHardware() {}
VescCanDiffBotSystemHardware::~VescCanDiffBotSystemHardware() {}

hardware_interface::CallbackReturn VescCanDiffBotSystemHardware::on_init(const hardware_interface::HardwareInfo & info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Parse parameters
  can_interface_ = info_.hardware_parameters["can_interface"];
  left_vesc_id_ = std::stoi(info_.hardware_parameters["left_vesc_id"]);
  right_vesc_id_ = std::stoi(info_.hardware_parameters["right_vesc_id"]);
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
  wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);

  // Resize joint vectors
  size_t n_joints = info_.joints.size();
  hw_commands_.resize(n_joints, 0.0);
  hw_positions_.resize(n_joints, 0.0);
  hw_velocities_.resize(n_joints, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescCanDiffBotSystemHardware::on_configure(const rclcpp_lifecycle::State & previous_state) {
  // No-op for now
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VescCanDiffBotSystemHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "position", &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "velocity", &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VescCanDiffBotSystemHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, "velocity", &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn VescCanDiffBotSystemHardware::on_activate(const rclcpp_lifecycle::State & previous_state) {
  // Reset state
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescCanDiffBotSystemHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
  // Optionally stop motors here
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type VescCanDiffBotSystemHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
  // Dummy: just increment position by velocity * dt
  double dt = period.seconds();
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    hw_positions_[i] += hw_velocities_[i] * dt;
    // For now, just mirror command as velocity
    hw_velocities_[i] = hw_commands_[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VescCanDiffBotSystemHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period) {
  // No-op: would send CAN commands here
  return hardware_interface::return_type::OK;
}


} // namespace diff_vesc_can_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diff_vesc_can_ros2_control::VescCanDiffBotSystemHardware, hardware_interface::SystemInterface)
