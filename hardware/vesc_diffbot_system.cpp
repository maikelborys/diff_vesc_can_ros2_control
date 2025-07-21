/**
 * @file vesc_diffbot_system.cpp
 * @brief Implementation of VESC-based differential drive robot hardware interface
 */

#include "diff_vesc_can_ros2_control/vesc_diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diff_vesc_can_ros2_control
{

VESCDiffBotSystemHardware::VESCDiffBotSystemHardware()
: use_fake_hardware_(true),
  can_interface_("can0"),
  hw_start_sec_(2.0),
  hw_stop_sec_(3.0),
  wheel_radius_(0.05),
  wheel_separation_(0.4)
{
}

VESCDiffBotSystemHardware::~VESCDiffBotSystemHardware()
{
  if (vesc_interface_) {
    vesc_interface_.reset();
  }
}

hardware_interface::CallbackReturn VESCDiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize logger and clock for fake hardware mode
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.VESCDiffBot"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // Read configuration parameters
  if (info_.hardware_parameters.find("use_fake_hardware") != info_.hardware_parameters.end())
  {
    use_fake_hardware_ = info_.hardware_parameters.at("use_fake_hardware") == "true";
  }

  if (info_.hardware_parameters.find("can_interface") != info_.hardware_parameters.end())
  {
    can_interface_ = info_.hardware_parameters.at("can_interface");
  }

  if (info_.hardware_parameters.find("wheel_radius") != info_.hardware_parameters.end())
  {
    wheel_radius_ = hardware_interface::stod(info_.hardware_parameters.at("wheel_radius"));
  }

  if (info_.hardware_parameters.find("wheel_separation") != info_.hardware_parameters.end())
  {
    wheel_separation_ = hardware_interface::stod(info_.hardware_parameters.at("wheel_separation"));
  }

  // Read fake hardware parameters (for simulation mode)
  if (info_.hardware_parameters.find("hw_start_duration_sec") != info_.hardware_parameters.end())
  {
    hw_start_sec_ = hardware_interface::stod(info_.hardware_parameters.at("hw_start_duration_sec"));
  }

  if (info_.hardware_parameters.find("hw_stop_duration_sec") != info_.hardware_parameters.end())
  {
    hw_stop_sec_ = hardware_interface::stod(info_.hardware_parameters.at("hw_stop_duration_sec"));
  }

  // Initialize joint data storage
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Validate joint configuration
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBot has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        *logger_, "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        *logger_, "Joint '%s' has '%s' command interface. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        *logger_, "Joint '%s' has %zu state interfaces. 2 expected.", 
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        *logger_, "Joint '%s' has '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        *logger_, "Joint '%s' has '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(*logger_, "VESCDiffBotSystemHardware initialized successfully");
  RCLCPP_INFO(*logger_, "Mode: %s", use_fake_hardware_ ? "Simulation" : "Real Hardware");
  if (!use_fake_hardware_) {
    RCLCPP_INFO(*logger_, "CAN Interface: %s", can_interface_.c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VESCDiffBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("VESCDiffBotSystemHardware"), "Configuring ...please wait...");

  if (!use_fake_hardware_) {
    // TODO: Initialize VESC interface when available
    RCLCPP_INFO(rclcpp::get_logger("VESCDiffBotSystemHardware"), 
                "Real VESC hardware mode configured for interface: %s", can_interface_.c_str());
    RCLCPP_WARN(rclcpp::get_logger("VESCDiffBotSystemHardware"), 
                "VESC hardware interface not yet implemented - using simulation mode");
    use_fake_hardware_ = true;  // Fallback to simulation for now
  }

  RCLCPP_INFO(rclcpp::get_logger("VESCDiffBotSystemHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VESCDiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VESCDiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn VESCDiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(*logger_, "Activating VESCDiffBotSystemHardware...");

  if (!use_fake_hardware_)
  {
    // Activate VESC interface
    if (vesc_interface_ && 
        vesc_interface_->on_activate(rclcpp_lifecycle::State()) != hardware_interface::CallbackReturn::SUCCESS)
    {
      RCLCPP_ERROR(*logger_, "Failed to activate VESC hardware interface");
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(*logger_, "VESC hardware interface activated");
  }
  else
  {
    // Fake hardware activation sequence
    for (auto i = 0; i < hw_start_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(*logger_, "%.1f seconds left...", hw_start_sec_ - i);
    }
  }

  // Set default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(*logger_, "VESCDiffBotSystemHardware activated successfully!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VESCDiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(*logger_, "Deactivating VESCDiffBotSystemHardware...");

  if (!use_fake_hardware_)
  {
    // Deactivate VESC interface
    if (vesc_interface_)
    {
      vesc_interface_->on_deactivate(rclcpp_lifecycle::State());
    }
    RCLCPP_INFO(*logger_, "VESC hardware interface deactivated");
  }
  else
  {
    // Fake hardware deactivation sequence
    for (auto i = 0; i < hw_stop_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(*logger_, "%.1f seconds left...", hw_stop_sec_ - i);
    }
  }

  RCLCPP_INFO(*logger_, "VESCDiffBotSystemHardware deactivated successfully!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type VESCDiffBotSystemHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!use_fake_hardware_)
  {
    // TODO: Read from real VESC hardware
    // For now, use simulation logic until VESC interface is integrated
    for (std::size_t i = 0; i < hw_velocities_.size(); i++)
    {
      // Simply integrate velocity to get position
      hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];
    }
    
    RCLCPP_DEBUG(*logger_, "Would read from real VESC hardware");
  }
  else
  {
    // Simulate DiffBot wheels movement as a first-order system
    for (std::size_t i = 0; i < hw_velocities_.size(); i++)
    {
      // Simply integrate velocity to get position
      hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];
    }

    // Log state periodically (throttled)
    std::stringstream ss;
    ss << "Reading states:";
    for (std::size_t i = 0; i < hw_velocities_.size(); i++)
    {
      ss << std::fixed << std::setprecision(2) << std::endl
         << "\t position " << hw_positions_[i] 
         << " and velocity " << hw_velocities_[i] 
         << " for '" << info_.joints[i].name.c_str() << "'!";
    }
    RCLCPP_INFO_THROTTLE(*logger_, *clock_, 500, "%s", ss.str().c_str());
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VESCDiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (use_fake_hardware_) {
    // Update fake hardware velocities from commands
    for (std::size_t i = 0; i < hw_commands_.size(); i++)
    {
      hw_velocities_[i] = hw_commands_[i];
    }
    
    // Log commands periodically (throttled)
    std::stringstream ss;
    ss << "Writing commands:";
    for (std::size_t i = 0; i < hw_commands_.size(); i++)
    {
      ss << std::fixed << std::setprecision(2) << std::endl
         << "\t velocity " << hw_commands_[i] 
         << " for '" << info_.joints[i].name.c_str() << "'!";
    }
    RCLCPP_INFO_THROTTLE(*logger_, *clock_, 500, "%s", ss.str().c_str());
  } else {
    // TODO: Real VESC hardware implementation
    // For now, just apply commands to velocities for simulation
    for (std::size_t i = 0; i < hw_commands_.size(); i++)
    {
      hw_velocities_[i] = hw_commands_[i];
    }
    
    RCLCPP_DEBUG(*logger_, 
                 "Would write to VESC: left_wheel_cmd=%.3f, right_wheel_cmd=%.3f", 
                 hw_commands_[0], hw_commands_[1]);
  }
  
  return hardware_interface::return_type::OK;
}

}  // namespace diff_vesc_can_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diff_vesc_can_ros2_control::VESCDiffBotSystemHardware, hardware_interface::SystemInterface)
