#ifndef DIFF_VESC_CAN_ROS2_CONTROL__VESC_CAN_DIFFBOT_SYSTEM_HPP_
#define DIFF_VESC_CAN_ROS2_CONTROL__VESC_CAN_DIFFBOT_SYSTEM_HPP_

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

#include <thread>
#include <atomic>
#include <optional>

namespace diff_vesc_can_ros2_control
{

class VescCanDiffBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  VescCanDiffBotSystemHardware();
  ~VescCanDiffBotSystemHardware();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Hardware configuration
  std::string can_interface_;
  int left_vesc_id_;
  int right_vesc_id_;
  double wheel_radius_;
  double wheel_separation_;

  // Joint data storage
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // Logger and clock
  std::shared_ptr<rclcpp::Logger> logger_;
  std::shared_ptr<rclcpp::Clock> clock_;

  // CAN socket
  int can_sock_ = -1;

  // CAN read thread and odometry state
  std::thread can_read_thread_;
  std::atomic<bool> can_read_running_{false};
  std::optional<int32_t> left_tach_initial_;
  std::optional<int32_t> right_tach_initial_;
  int32_t left_tach_current_ = 0;
  int32_t right_tach_current_ = 0;
  double wheel_circumference_ = 0.0;
  double distance_per_pulse_raw_ = 0.0;
};

}  // namespace diff_vesc_can_ros2_control

#endif  // DIFF_VESC_CAN_ROS2_CONTROL__VESC_CAN_DIFFBOT_SYSTEM_HPP_
