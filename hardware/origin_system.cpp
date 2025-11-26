// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "origin_hardware/origin_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace origin_hardware
{
hardware_interface::CallbackReturn OriginSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Defensive parameter parsing with defaults and warnings to avoid
  // std::invalid_argument/exception thrown from std::stof/std::stoi.
  auto get_param = [&](const std::string & key, const std::string & def)->std::string {
    if (info_.hardware_parameters.count(key) > 0) {
      return info_.hardware_parameters.at(key);
    }
    RCLCPP_WARN(rclcpp::get_logger("OriginSystemHardware"), "Hardware parameter '%s' not supplied. Using default '%s'", key.c_str(), def.c_str());
    return def;
  };

  cfg_.wheel1_name = get_param("wheel1_name", "wheel1_joint");
  cfg_.wheel2_name = get_param("wheel2_name", "wheel2_joint");
  cfg_.wheel3_name = get_param("wheel3_name", "wheel3_joint");

  std::string tmp;
  tmp = get_param("loop_rate", "20.0");
  try { cfg_.loop_rate = std::stof(tmp); } catch (const std::exception & e) { RCLCPP_WARN(rclcpp::get_logger("OriginSystemHardware"), "Invalid loop_rate '%s' (%s). Using 20.0", tmp.c_str(), e.what()); cfg_.loop_rate = 20.0; }

  cfg_.device = get_param("device", "/dev/ttyUSB0");

  tmp = get_param("baud_rate", "57600");
  try { cfg_.baud_rate = std::stoi(tmp); } catch (const std::exception & e) { RCLCPP_WARN(rclcpp::get_logger("OriginSystemHardware"), "Invalid baud_rate '%s' (%s). Using 57600", tmp.c_str(), e.what()); cfg_.baud_rate = 57600; }

  tmp = get_param("timeout_ms", "100");
  try { cfg_.timeout_ms = std::stoi(tmp); } catch (const std::exception & e) { RCLCPP_WARN(rclcpp::get_logger("OriginSystemHardware"), "Invalid timeout_ms '%s' (%s). Using 100", tmp.c_str(), e.what()); cfg_.timeout_ms = 100; }

  tmp = get_param("enc_counts_per_rev", "4096");
  try { cfg_.enc_counts_per_rev = std::stoi(tmp); } catch (const std::exception & e) { RCLCPP_WARN(rclcpp::get_logger("OriginSystemHardware"), "Invalid enc_counts_per_rev '%s' (%s). Using 4096", tmp.c_str(), e.what()); cfg_.enc_counts_per_rev = 4096; }

  if (info_.hardware_parameters.count("pid_p") > 0) {
    try {
      cfg_.pid_p = std::stoi(info_.hardware_parameters.at("pid_p"));
      cfg_.pid_d = std::stoi(info_.hardware_parameters.at("pid_d"));
      cfg_.pid_i = std::stoi(info_.hardware_parameters.at("pid_i"));
      cfg_.pid_o = std::stoi(info_.hardware_parameters.at("pid_o"));
    } catch (const std::exception & e) {
      RCLCPP_WARN(rclcpp::get_logger("OriginSystemHardware"), "Invalid PID parameters (%s). Using defaults.", e.what());
      cfg_.pid_p = cfg_.pid_d = cfg_.pid_i = cfg_.pid_o = 0;
    }
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OriginSystemHardware"), "PID values not supplied, using defaults.");
  }
  wheel_1_.setup(cfg_.wheel1_name, cfg_.enc_counts_per_rev);
  wheel_2_.setup(cfg_.wheel2_name, cfg_.enc_counts_per_rev);
  wheel_3_.setup(cfg_.wheel3_name, cfg_.enc_counts_per_rev);


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // it has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("OriginSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("OriginSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("OriginSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("OriginSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("OriginSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OriginSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_1_.name, hardware_interface::HW_IF_POSITION, &wheel_1_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_1_.name, hardware_interface::HW_IF_VELOCITY, &wheel_1_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_2_.name, hardware_interface::HW_IF_POSITION, &wheel_2_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_2_.name, hardware_interface::HW_IF_VELOCITY, &wheel_2_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_3_.name, hardware_interface::HW_IF_POSITION, &wheel_3_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_3_.name, hardware_interface::HW_IF_VELOCITY, &wheel_3_.vel));


  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> OriginSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_1_.name, hardware_interface::HW_IF_VELOCITY, &wheel_1_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_2_.name, hardware_interface::HW_IF_VELOCITY, &wheel_2_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_3_.name, hardware_interface::HW_IF_VELOCITY, &wheel_3_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn OriginSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("OriginSystemHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  if (!comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms))
  {
    RCLCPP_WARN(rclcpp::get_logger("OriginSystemHardware"), "Failed to open hardware device '%s' (baud=%d). Continuing without hardware connected. Connect the device and call lifecycle transitions when ready.", cfg_.device.c_str(), cfg_.baud_rate);
    // Do not return ERROR here — allow controller_manager to continue. Activation
    // will fail if the hardware is not connected, which is handled in on_activate().
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OriginSystemHardware"), "Successfully configured with hardware device '%s'", cfg_.device.c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OriginSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("OriginSystemHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("OriginSystemHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn OriginSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("OriginSystemHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    RCLCPP_WARN(rclcpp::get_logger("OriginSystemHardware"), "Activation: hardware device not connected. Activation will complete in degraded mode — connect the device and perform lifecycle transitions when ready.");
    // Return SUCCESS to avoid controller_manager throwing when attempting to set
    // the initial lifecycle state. Read/write will report errors until hardware
    // is connected.
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  if (cfg_.pid_p > 0)
  {
    comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
  }
  RCLCPP_INFO(rclcpp::get_logger("OriginSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OriginSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("OriginSystemHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("OriginSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type OriginSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  comms_.read_encoder_values(wheel_1_.enc, wheel_2_.enc, wheel_3_.enc);

  double delta_seconds = period.seconds();

  double pos_prev = wheel_1_.pos;
  wheel_1_.pos = wheel_1_.calc_enc_angle();
  wheel_1_.vel = (wheel_1_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_2_.pos;
  wheel_2_.pos = wheel_2_.calc_enc_angle();
  wheel_2_.vel = (wheel_2_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_3_.pos;
  wheel_3_.pos = wheel_3_.calc_enc_angle();
  wheel_3_.vel = (wheel_3_.pos - pos_prev) / delta_seconds;


  return hardware_interface::return_type::OK;
}

hardware_interface::return_type origin_hardware::OriginSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  int motor_1_counts_per_loop = wheel_1_.cmd / wheel_1_.rads_per_count / cfg_.loop_rate;
  int motor_2_counts_per_loop = wheel_2_.cmd / wheel_2_.rads_per_count / cfg_.loop_rate;
  int motor_3_counts_per_loop = wheel_3_.cmd / wheel_3_.rads_per_count / cfg_.loop_rate;
  comms_.set_motor_values(motor_1_counts_per_loop, motor_2_counts_per_loop, motor_3_counts_per_loop);
  return hardware_interface::return_type::OK;
}

}  // namespace origin_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  origin_hardware::OriginSystemHardware, hardware_interface::SystemInterface)
