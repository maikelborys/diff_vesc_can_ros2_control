
#include "diff_vesc_can_ros2_control/vesc_can_diffbot_system.hpp"

#include <limits>
#include <vector>
#include <string>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>

namespace diff_vesc_can_ros2_control {

#ifndef CAN_EFF_FLAG
#define CAN_EFF_FLAG 0x80000000U
#endif

VescCanDiffBotSystemHardware::VescCanDiffBotSystemHardware() : can_sock_(-1) {}
VescCanDiffBotSystemHardware::~VescCanDiffBotSystemHardware() {
  // Stop CAN read thread if running
  can_read_running_ = false;
  if (can_read_thread_.joinable()) can_read_thread_.join();
  if (can_sock_ >= 0) close(can_sock_);
}

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

  // Open CAN socket (SocketCAN, modeled after can_bridge_node.cpp)
  can_sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_sock_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("vesc_can_diffbot_system"), "Failed to create CAN socket: %s", strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }
  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ-1);
  ifr.ifr_name[IFNAMSIZ-1] = '\0';
  if (ioctl(can_sock_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("vesc_can_diffbot_system"), "Failed to get interface index for %s: %s", can_interface_.c_str(), strerror(errno));
    close(can_sock_); can_sock_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }
  struct sockaddr_can addr = {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(can_sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("vesc_can_diffbot_system"), "Failed to bind CAN socket: %s", strerror(errno));
    close(can_sock_); can_sock_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("vesc_can_diffbot_system"), "CAN interface %s initialized successfully", can_interface_.c_str());
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
  // Calculate wheel circumference and distance per pulse
  wheel_circumference_ = M_PI * (2.0 * wheel_radius_); // wheel_diameter = 2 * radius
  double ticks_per_mechanical_revolution = 138.0;
  distance_per_pulse_raw_ = wheel_circumference_ / ticks_per_mechanical_revolution;
  // Start CAN read thread for STATUS_5 parsing
  can_read_running_ = true;
  can_read_thread_ = std::thread([this]() {
    while (can_read_running_) {
      struct can_frame frame;
      ssize_t nbytes = ::read(can_sock_, &frame, sizeof(frame));
      if (nbytes == sizeof(frame)) {
        uint32_t actual_id = frame.can_id & 0x1FFFFFFF;
        if (actual_id == (0x1B00 + left_vesc_id_)) {
          int32_t tach_raw = (frame.data[2] << 8) | frame.data[3];
          if (tach_raw > 32767) tach_raw -= 65536;
          if (!left_tach_initial_.has_value()) left_tach_initial_ = tach_raw;
          left_tach_current_ = tach_raw;
        } else if (actual_id == (0x1B00 + right_vesc_id_)) {
          int32_t tach_raw = (frame.data[2] << 8) | frame.data[3];
          if (tach_raw > 32767) tach_raw -= 65536;
          if (!right_tach_initial_.has_value()) right_tach_initial_ = tach_raw;
          right_tach_current_ = tach_raw;
        }
      }
    }
  });
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescCanDiffBotSystemHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state) {
  // Stop CAN read thread
  can_read_running_ = false;
  if (can_read_thread_.joinable()) can_read_thread_.join();
  // Optionally stop motors here
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type VescCanDiffBotSystemHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
  // Dummy: just increment position by velocity * dt
  // Instead of integrating velocity, use real tachometer readings for position
  // This requires CAN STATUS_5 parsing in write() or a dedicated CAN read thread
  // For demonstration, update positions if initial values are set
  if (left_tach_initial_.has_value() && right_tach_initial_.has_value()) {
    int32_t left_tach_diff = left_tach_current_ - left_tach_initial_.value();
    int32_t right_tach_diff = right_tach_current_ - right_tach_initial_.value();
    hw_positions_[0] = left_tach_diff * distance_per_pulse_raw_;
    hw_positions_[1] = right_tach_diff * distance_per_pulse_raw_;
  }
  // Velocity can still be mirrored from command for now
  for (size_t i = 0; i < hw_velocities_.size(); ++i) {
    hw_velocities_[i] = hw_commands_[i];
  }
  return hardware_interface::return_type::OK;
  // --- CAN STATUS_5 parsing for real odometry ---
  // This is a demonstration: you need to receive CAN frames and parse STATUS_5
  // Example frame parsing:
  // struct can_frame frame;
  // ssize_t nbytes = ::read(can_sock_, &frame, sizeof(frame));
  // if (nbytes == sizeof(frame)) {
  //   uint32_t actual_id = frame.can_id & 0x1FFFFFFF;
  //   if (actual_id == (0x1B00 + left_vesc_id_)) {
  //     int32_t tach_raw = (frame.data[2] << 8) | frame.data[3];
  //     if (tach_raw > 32767) tach_raw -= 65536;
  //     if (!left_tach_initial_.has_value()) left_tach_initial_ = tach_raw;
  //     left_tach_current_ = tach_raw;
  //   } else if (actual_id == (0x1B00 + right_vesc_id_)) {
  //     int32_t tach_raw = (frame.data[2] << 8) | frame.data[3];
  //     if (tach_raw > 32767) tach_raw -= 65536;
  //     if (!right_tach_initial_.has_value()) right_tach_initial_ = tach_raw;
  //     right_tach_current_ = tach_raw;
  //   }
  // }
}

hardware_interface::return_type VescCanDiffBotSystemHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period) {
  // Assume joint 0 = left, joint 1 = right
  if (can_sock_ < 0) return hardware_interface::return_type::ERROR;
  if (hw_commands_.size() < 2) return hardware_interface::return_type::ERROR;


  // Convert incoming commands from rad/s to m/s using wheel_radius
  double left_mps = hw_commands_[0] * wheel_radius_;
  double right_mps = hw_commands_[1] * wheel_radius_;
  RCLCPP_INFO(rclcpp::get_logger("vesc_can_diffbot_system"),
    "[DEBUG] hw_commands: left=%.6f rad/s (%.6f m/s) right=%.6f rad/s (%.6f m/s)",
    hw_commands_[0], left_mps, hw_commands_[1], right_mps);

  // Empirical calibration: duty_cycle = (velocity_mps / 7.857) * 10.0 ITS APPROXIMATE ************************
double left_duty = std::clamp(left_mps / 7.857, -1.0, 1.0);
double right_duty = std::clamp(right_mps / 7.857, -1.0, 1.0);
  RCLCPP_INFO(rclcpp::get_logger("vesc_can_diffbot_system"),
    "[DEBUG] duty_cycle: left=%.6f right=%.6f", left_duty, right_duty);

  int32_t left_val = static_cast<int32_t>(left_duty * 100000.0);
  int32_t right_val = static_cast<int32_t>(right_duty * 100000.0);
  RCLCPP_INFO(rclcpp::get_logger("vesc_can_diffbot_system"),
    "[DEBUG] CAN value: left=%d right=%d", left_val, right_val);

  struct can_frame left_frame = {};
  left_frame.can_id = static_cast<uint32_t>(left_vesc_id_) | CAN_EFF_FLAG;
  left_frame.can_dlc = 4;
  left_frame.data[0] = (left_val >> 24) & 0xFF;
  left_frame.data[1] = (left_val >> 16) & 0xFF;
  left_frame.data[2] = (left_val >> 8) & 0xFF;
  left_frame.data[3] = left_val & 0xFF;

  struct can_frame right_frame = {};
  right_frame.can_id = static_cast<uint32_t>(right_vesc_id_) | CAN_EFF_FLAG;
  right_frame.can_dlc = 4;
  right_frame.data[0] = (right_val >> 24) & 0xFF;
  right_frame.data[1] = (right_val >> 16) & 0xFF;
  right_frame.data[2] = (right_val >> 8) & 0xFF;
  right_frame.data[3] = right_val & 0xFF;

  // --- DEBUG SECTION: Commented out for production ---
  /*
  char left_hex[16], right_hex[16];
  snprintf(left_hex, sizeof(left_hex), "%02X.%02X.%02X.%02X",
           left_frame.data[0], left_frame.data[1], left_frame.data[2], left_frame.data[3]);
  snprintf(right_hex, sizeof(right_hex), "%02X.%02X.%02X.%02X",
           right_frame.data[0], right_frame.data[1], right_frame.data[2], right_frame.data[3]);
  RCLCPP_INFO(rclcpp::get_logger("vesc_can_diffbot_system"),
    "[DEBUG] Would send CAN: LEFT (ID 0x%X) data %s | RIGHT (ID 0x%X) data %s",
    left_frame.can_id, left_hex, right_frame.can_id, right_hex);
  */

  // --- PRODUCTION: Send real CAN messages ---
  ssize_t n1 = ::write(can_sock_, &left_frame, sizeof(left_frame));
  ssize_t n2 = ::write(can_sock_, &right_frame, sizeof(right_frame));
  if (n1 != sizeof(left_frame) || n2 != sizeof(right_frame)) {
    RCLCPP_ERROR(rclcpp::get_logger("vesc_can_diffbot_system"), "Failed to send CAN frames: %s", strerror(errno));
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}


} // namespace diff_vesc_can_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diff_vesc_can_ros2_control::VescCanDiffBotSystemHardware, hardware_interface::SystemInterface)
