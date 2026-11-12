#include "diff_vesc_can_ros2_control/vesc_can_diffbot_system.hpp"

#include <limits>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <sys/socket.h>
#include <sys/time.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

namespace diff_vesc_can_ros2_control {

#ifndef CAN_EFF_FLAG
#define CAN_EFF_FLAG 0x80000000U
#endif

VescCanDiffBotSystemHardware::VescCanDiffBotSystemHardware() : can_sock_(-1) {}

VescCanDiffBotSystemHardware::~VescCanDiffBotSystemHardware() {
  send_stop_commands();
  can_read_running_ = false;
  if (can_read_thread_.joinable()) {
    can_read_thread_.join();
  }
  if (can_sock_ >= 0) {
    close(can_sock_);
    can_sock_ = -1;
  }
}

hardware_interface::CallbackReturn VescCanDiffBotSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  // Parse required parameters
  can_interface_ = info_.hardware_parameters["can_interface"];
  left_vesc_id_ = std::stoi(info_.hardware_parameters["left_vesc_id"]);
  right_vesc_id_ = std::stoi(info_.hardware_parameters["right_vesc_id"]);
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
  wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);

  // Parse optional parameters
  if (info_.hardware_parameters.find("max_linear_speed_mps") != info_.hardware_parameters.end()) {
    max_linear_speed_mps_ = std::stod(info_.hardware_parameters.at("max_linear_speed_mps"));
  }
  if (info_.hardware_parameters.find("ticks_per_mechanical_revolution") != info_.hardware_parameters.end()) {
    ticks_per_mechanical_revolution_ = std::stod(info_.hardware_parameters.at("ticks_per_mechanical_revolution"));
  }
  if (info_.hardware_parameters.find("motor_pole_pairs") != info_.hardware_parameters.end()) {
    motor_pole_pairs_ = std::stoi(info_.hardware_parameters.at("motor_pole_pairs"));
    if (motor_pole_pairs_ <= 0) {
      motor_pole_pairs_ = 15;
    }
  }

  // Resize joint vectors
  size_t n_joints = info_.joints.size();
  hw_commands_.resize(n_joints, 0.0);
  hw_positions_.resize(n_joints, 0.0);
  hw_velocities_.resize(n_joints, 0.0);

  // Open CAN socket (SocketCAN)
  can_sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_sock_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("vesc_can_diffbot_system"), "Failed to create CAN socket: %s", strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Set receive timeout so that read thread does not block indefinitely (BUG-12)
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100000;  // 100 ms timeout
  if (setsockopt(can_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    RCLCPP_WARN(rclcpp::get_logger("vesc_can_diffbot_system"), "Failed to set SO_RCVTIMEO on CAN socket: %s", strerror(errno));
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (ioctl(can_sock_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("vesc_can_diffbot_system"), "Failed to get interface index for %s: %s", can_interface_.c_str(), strerror(errno));
    close(can_sock_);
    can_sock_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  struct sockaddr_can addr = {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(can_sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("vesc_can_diffbot_system"), "Failed to bind CAN socket: %s", strerror(errno));
    close(can_sock_);
    can_sock_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("vesc_can_diffbot_system"),
    "CAN interface %s initialized (max_speed=%.2f m/s, ticks_rev=%.1f, pole_pairs=%d)",
    can_interface_.c_str(), max_linear_speed_mps_, ticks_per_mechanical_revolution_, motor_pole_pairs_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescCanDiffBotSystemHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
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

hardware_interface::CallbackReturn VescCanDiffBotSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  // Reset state
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }

  // Reset atomic odometry tracking states (BUG-01)
  left_tach_initial_set_.store(false);
  right_tach_initial_set_.store(false);
  left_tach_current_.store(0);
  right_tach_current_.store(0);
  left_tach_initial_.store(0);
  right_tach_initial_.store(0);
  left_erpm_received_.store(false);
  right_erpm_received_.store(false);
  left_erpm_current_.store(0);
  right_erpm_current_.store(0);

  // Calculate wheel circumference and distance per pulse
  wheel_circumference_ = M_PI * (2.0 * wheel_radius_);
  distance_per_pulse_raw_ = wheel_circumference_ / ticks_per_mechanical_revolution_;

  // Start CAN read thread for STATUS_5 parsing
  can_read_running_.store(true);
  can_read_thread_ = std::thread([this]() {
    while (can_read_running_.load()) {
      struct can_frame frame;
      ssize_t nbytes = ::read(can_sock_, &frame, sizeof(frame));
      if (nbytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
          continue;  // Timeout elapsed or signal, loop and check can_read_running_
        }
        if (can_read_running_.load()) {
          RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("vesc_can_diffbot_system"), *clock_, 5000,
            "CAN read error: %s", strerror(errno));
        }
        continue;
      }

      if (nbytes == sizeof(frame)) {
        uint32_t actual_id = frame.can_id & 0x1FFFFFFF;
        // VESC CAN_PACKET_STATUS (0x09) is 0x0900 + vesc_id -> contains eRPM in bytes 0..3
        if (actual_id == static_cast<uint32_t>(0x0900 + left_vesc_id_)) {
          int32_t erpm_raw = (static_cast<int32_t>(frame.data[0]) << 24) |
                             (static_cast<int32_t>(frame.data[1]) << 16) |
                             (static_cast<int32_t>(frame.data[2]) << 8)  |
                             (static_cast<int32_t>(frame.data[3]));
          left_erpm_current_.store(erpm_raw);
          left_erpm_received_.store(true);
        } else if (actual_id == static_cast<uint32_t>(0x0900 + right_vesc_id_)) {
          int32_t erpm_raw = (static_cast<int32_t>(frame.data[0]) << 24) |
                             (static_cast<int32_t>(frame.data[1]) << 16) |
                             (static_cast<int32_t>(frame.data[2]) << 8)  |
                             (static_cast<int32_t>(frame.data[3]));
          right_erpm_current_.store(erpm_raw);
          right_erpm_received_.store(true);
        }
        // VESC CAN_PACKET_STATUS_5 is 0x1B00 + vesc_id
        else if (actual_id == static_cast<uint32_t>(0x1B00 + left_vesc_id_)) {
          // Parse full 32-bit big-endian tachometer reading (BUG-02)
          int32_t tach_raw = (static_cast<int32_t>(frame.data[0]) << 24) |
                             (static_cast<int32_t>(frame.data[1]) << 16) |
                             (static_cast<int32_t>(frame.data[2]) << 8)  |
                             (static_cast<int32_t>(frame.data[3]));

          bool expected = false;
          if (left_tach_initial_set_.compare_exchange_strong(expected, true)) {
            left_tach_initial_.store(tach_raw);
          }
          left_tach_current_.store(tach_raw);
        } else if (actual_id == static_cast<uint32_t>(0x1B00 + right_vesc_id_)) {
          // Parse full 32-bit big-endian tachometer reading (BUG-02)
          int32_t tach_raw = (static_cast<int32_t>(frame.data[0]) << 24) |
                             (static_cast<int32_t>(frame.data[1]) << 16) |
                             (static_cast<int32_t>(frame.data[2]) << 8)  |
                             (static_cast<int32_t>(frame.data[3]));

          bool expected = false;
          if (right_tach_initial_set_.compare_exchange_strong(expected, true)) {
            right_tach_initial_.store(tach_raw);
          }
          right_tach_current_.store(tach_raw);
        }
      }
    }
  });

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescCanDiffBotSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  // Explicitly stop motors on deactivate (BUG-13)
  send_stop_commands();

  // Stop CAN read thread gracefully (BUG-12)
  can_read_running_.store(false);
  if (can_read_thread_.joinable()) {
    can_read_thread_.join();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

void VescCanDiffBotSystemHardware::send_stop_commands() {
  if (can_sock_ < 0) {
    return;
  }
  // Send 0 duty cycle frames to both left and right VESCs
  struct can_frame left_frame = {};
  left_frame.can_id = static_cast<uint32_t>(left_vesc_id_) | CAN_EFF_FLAG;
  left_frame.can_dlc = 4;

  struct can_frame right_frame = {};
  right_frame.can_id = static_cast<uint32_t>(right_vesc_id_) | CAN_EFF_FLAG;
  right_frame.can_dlc = 4;

  ::write(can_sock_, &left_frame, sizeof(left_frame));
  ::write(can_sock_, &right_frame, sizeof(right_frame));
}

hardware_interface::return_type VescCanDiffBotSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // Thread-safe atomic read of positions (BUG-01)
  if (left_tach_initial_set_.load() && right_tach_initial_set_.load()) {
    int32_t left_tach_diff = left_tach_current_.load() - left_tach_initial_.load();
    int32_t right_tach_diff = right_tach_current_.load() - right_tach_initial_.load();
    hw_positions_[0] = static_cast<double>(left_tach_diff) * distance_per_pulse_raw_;
    hw_positions_[1] = static_cast<double>(right_tach_diff) * distance_per_pulse_raw_;
  }

  // Real angular velocity feedback in rad/s computed from VESC STATUS_1 eRPM:
  // omega [rad/s] = (erpm / pole_pairs) * (2 * pi / 60)
  if (left_erpm_received_.load() && right_erpm_received_.load()) {
    const double erpm_to_rad_s = M_PI / (static_cast<double>(motor_pole_pairs_) * 30.0);
    hw_velocities_[0] = static_cast<double>(left_erpm_current_.load()) * erpm_to_rad_s;
    hw_velocities_[1] = static_cast<double>(right_erpm_current_.load()) * erpm_to_rad_s;
  } else {
    // Fallback during initial boot before first STATUS_1 frame is received
    for (size_t i = 0; i < hw_velocities_.size(); ++i) {
      hw_velocities_[i] = hw_commands_[i];
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VescCanDiffBotSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (can_sock_ < 0) {
    return hardware_interface::return_type::ERROR;
  }
  if (hw_commands_.size() < 2) {
    return hardware_interface::return_type::ERROR;
  }

  // Convert incoming commands from rad/s to m/s using wheel_radius
  double left_mps = hw_commands_[0] * wheel_radius_;
  double right_mps = hw_commands_[1] * wheel_radius_;

  // Duty cycle normalized using configurable max_linear_speed_mps (BUG-03)
  double left_duty = std::clamp(left_mps / max_linear_speed_mps_, -1.0, 1.0);
  double right_duty = std::clamp(right_mps / max_linear_speed_mps_, -1.0, 1.0);

  int32_t left_val = static_cast<int32_t>(left_duty * 100000.0);
  int32_t right_val = static_cast<int32_t>(right_duty * 100000.0);

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

  ssize_t n1 = ::write(can_sock_, &left_frame, sizeof(left_frame));
  ssize_t n2 = ::write(can_sock_, &right_frame, sizeof(right_frame));

  if (n1 != sizeof(left_frame) || n2 != sizeof(right_frame)) {
    RCLCPP_ERROR(rclcpp::get_logger("vesc_can_diffbot_system"), "Failed to send CAN frames: %s", strerror(errno));
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace diff_vesc_can_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diff_vesc_can_ros2_control::VescCanDiffBotSystemHardware, hardware_interface::SystemInterface)
