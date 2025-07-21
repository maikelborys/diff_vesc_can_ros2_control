/**
 * @file vesc_diffbot_system.hpp
 * @brief Complete ros2_control implementation for VESC-based differential drive robots
 * 
 * This hardware interface combines the proven ros2_control demo structure with
 * real VESC CAN communication for production-ready robot control.
 */

#ifndef DIFF_VESC_CAN_ROS2_CONTROL__VESC_DIFFBOT_SYSTEM_HPP_
#define DIFF_VESC_CAN_ROS2_CONTROL__VESC_DIFFBOT_SYSTEM_HPP_

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <memory>
#include <string>
#include <vector>

namespace diff_vesc_can_ros2_control
{

/**
 * @brief Complete ros2_control implementation for VESC differential drive robots
 * 
 * This class provides a complete ros2_control SystemInterface that:
 * - Uses real VESC hardware via CAN communication
 * - Implements proper differential drive kinematics
 * - Provides robust lifecycle management
 * - Offers both simulation and real hardware modes
 */
class VESCDiffBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  VESCDiffBotSystemHardware();
  ~VESCDiffBotSystemHardware();

  // SystemInterface methods
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Hardware configuration
  bool use_fake_hardware_;
  std::string can_interface_;
  
  // Joint data storage
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  
  // Fake hardware simulation parameters
  double hw_start_sec_;
  double hw_stop_sec_;
  
  // Robot parameters
  double wheel_radius_;
  double wheel_separation_;
  
  // Logger and clock for fake hardware mode
  std::shared_ptr<rclcpp::Logger> logger_;
  std::shared_ptr<rclcpp::Clock> clock_;
};

}  // namespace diff_vesc_can_ros2_control

#endif  // DIFF_VESC_CAN_ROS2_CONTROL__VESC_DIFFBOT_SYSTEM_HPP_
