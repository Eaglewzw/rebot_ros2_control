// Copyright 2026 reBot ros2_control contributors
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

#include "rebot_hardware/rebot_system.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rebot_hardware
{

namespace
{

constexpr size_t kMaxRxFramesPerCycle = 64;

/// Fetch a parameter from a map, returning `fallback` when absent.
double param_or(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  double fallback)
{
  const auto it = params.find(key);
  return it == params.end() ? fallback : std::stod(it->second);
}

}  // namespace

ReBotSystemHardware::~ReBotSystemHardware()
{
  // Safety net for unclean shutdown (e.g. Ctrl+C without lifecycle
  // transitions): make sure no motor is left holding torque.
  disable_all_motors();
}

hardware_interface::CallbackReturn ReBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    const auto & hw_params = info_.hardware_parameters;
    serial_port_ = hw_params.count("serial_port") ? hw_params.at("serial_port") : "/dev/ttyACM0";
    baud_rate_ = static_cast<int>(
      param_or(hw_params, "baud_rate", SerialCanBridge::kDefaultBaudRate));
    comm_error_threshold_ =
      static_cast<unsigned int>(param_or(hw_params, "comm_error_threshold", 10.0));
    activate_timeout_ms_ = static_cast<int>(param_or(hw_params, "activate_timeout_ms", 100.0));
    enable_retries_ = static_cast<int>(param_or(hw_params, "enable_retries", 5.0));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger_, "Invalid hardware parameter: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const size_t n = info_.joints.size();
  joints_.resize(n);
  hw_commands_position_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocity_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_commands_kp_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_commands_kd_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_commands_effort_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_states_position_.assign(n, 0.0);
  hw_states_velocity_.assign(n, 0.0);
  hw_states_effort_.assign(n, 0.0);
  missed_replies_.assign(n, 0);

  for (size_t i = 0; i < n; ++i) {
    const auto & joint = info_.joints[i];
    JointConfig & cfg = joints_[i];

    // Interface sanity checks, following ros2_control_demos example_1 style.
    // Full Damiao MIT tuple, in this fixed order.
    const std::array<const char *, 5> expected_cmd = {
      hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY, HW_IF_KP, HW_IF_KD,
      hardware_interface::HW_IF_EFFORT};
    bool cmd_ok = joint.command_interfaces.size() == expected_cmd.size();
    for (size_t c = 0; cmd_ok && c < expected_cmd.size(); ++c) {
      cmd_ok = joint.command_interfaces[c].name == expected_cmd[c];
    }
    if (!cmd_ok) {
      RCLCPP_FATAL(
        logger_,
        "Joint '%s' must have exactly the command interfaces "
        "position/velocity/kp/kd/effort (in this order).",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 3 ||
      joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
      joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY ||
      joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(
        logger_, "Joint '%s' must have position/velocity/effort state interfaces.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    try {
      const auto & p = joint.parameters;
      cfg.motor_id = static_cast<uint32_t>(std::stoul(p.at("motor_id"), nullptr, 0));
      // Damiao convention: MasterID (feedback id) = CAN id + 0x10.
      cfg.feedback_id = p.count("feedback_id") ?
        static_cast<uint32_t>(std::stoul(p.at("feedback_id"), nullptr, 0)) :
        cfg.motor_id + 0x10;

      const std::string model = p.at("motor_model");
      const auto limits = damiao::limits_for_model(model);
      if (!limits) {
        RCLCPP_FATAL(
          logger_, "Joint '%s': unknown motor_model '%s'.", joint.name.c_str(), model.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      cfg.limits = *limits;

      cfg.kp = std::stod(p.at("kp"));
      cfg.kd = std::stod(p.at("kd"));
      if (cfg.kp < damiao::kKpMin || cfg.kp > damiao::kKpMax || cfg.kd < damiao::kKdMin ||
        cfg.kd > damiao::kKdMax)
      {
        RCLCPP_FATAL(
          logger_, "Joint '%s': kp/kd (%.1f/%.2f) outside [0,500]/[0,5].", joint.name.c_str(),
          cfg.kp, cfg.kd);
        return hardware_interface::CallbackReturn::ERROR;
      }

      cfg.reduction = param_or(p, "mechanical_reduction", 1.0);
      cfg.offset = param_or(p, "position_offset", 0.0);
      if (cfg.reduction == 0.0) {
        RCLCPP_FATAL(logger_, "Joint '%s': mechanical_reduction must not be 0.",
          joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // Software position limits: explicit params win, otherwise use the
      // min/max of the position command interface from the URDF.
      double min_pos = -cfg.limits.p_max;
      double max_pos = cfg.limits.p_max;
      if (!joint.command_interfaces[0].min.empty()) {
        min_pos = std::stod(joint.command_interfaces[0].min);
      }
      if (!joint.command_interfaces[0].max.empty()) {
        max_pos = std::stod(joint.command_interfaces[0].max);
      }
      cfg.min_position = param_or(p, "min_position", min_pos);
      cfg.max_position = param_or(p, "max_position", max_pos);
      if (cfg.min_position >= cfg.max_position) {
        RCLCPP_FATAL(
          logger_, "Joint '%s': min_position (%.3f) >= max_position (%.3f).", joint.name.c_str(),
          cfg.min_position, cfg.max_position);
        return hardware_interface::CallbackReturn::ERROR;
      }
    } catch (const std::exception & e) {
      RCLCPP_FATAL(
        logger_,
        "Joint '%s': missing or invalid parameter (%s). Required params: motor_id, motor_model, "
        "kp, kd.",
        joint.name.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  bridge_ = std::make_unique<SerialCanBridge>();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ReBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string error;
  if (!bridge_->open(serial_port_, baud_rate_, error)) {
    RCLCPP_ERROR(logger_, "Failed to open serial CAN bridge: %s", error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    logger_, "Opened Damiao USB-CAN bridge on '%s' @ %d baud.", serial_port_.c_str(), baud_rate_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ReBotSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  disable_all_motors();
  bridge_->close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ReBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  damiao::CanFrame rx[kMaxRxFramesPerCycle];

  // Drop any stale bytes from previous sessions.
  bridge_->receive(rx, kMaxRxFramesPerCycle);

  // Per motor: solicit feedback with a zero-gain MIT frame (kp=kd=tau=0
  // produces no torque even on an enabled motor), initialise the command to
  // the current position, then enable. This guarantees command == state at
  // the moment torque appears, preventing power-on jumps.
  for (size_t i = 0; i < joints_.size(); ++i) {
    const JointConfig & cfg = joints_[i];
    bool got_state = false;

    for (int attempt = 0; attempt < enable_retries_ && !got_state; ++attempt) {
      const auto probe =
        damiao::make_mit_frame(cfg.motor_id, cfg.limits, 0.0, 0.0, 0.0, 0.0, 0.0);
      if (!bridge_->send(probe)) {
        RCLCPP_ERROR(
          logger_, "Joint '%s': failed to write to serial bridge.", info_.joints[i].name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      const size_t got = bridge_->receive_for(rx, kMaxRxFramesPerCycle, activate_timeout_ms_);
      for (size_t f = 0; f < got; ++f) {
        if (apply_feedback(rx[f]) == static_cast<int>(i)) {got_state = true;}
      }
    }
    if (!got_state) {
      RCLCPP_ERROR(
        logger_,
        "Joint '%s' (motor 0x%02X): no feedback after %d attempts. Check power, CAN wiring and "
        "motor id.",
        info_.joints[i].name.c_str(), cfg.motor_id, enable_retries_);
      disable_all_motors();
      return hardware_interface::CallbackReturn::ERROR;
    }

    hw_commands_position_[i] = hw_states_position_[i];
    hw_commands_velocity_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_commands_kp_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_commands_kd_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_commands_effort_[i] = std::numeric_limits<double>::quiet_NaN();

    // Set MIT control mode BEFORE enabling. The motor will ignore MIT frames
    // unless RID_CTRL_MODE (register 10) is explicitly set to MODE_MIT (1).
    if (!bridge_->send(
        damiao::make_parameter_write_frame(cfg.motor_id, damiao::kRidCtrlMode, damiao::kModeMit)))
    {
      RCLCPP_ERROR(
        logger_, "Joint '%s': failed to write MIT control mode.", info_.joints[i].name.c_str());
      disable_all_motors();
      return hardware_interface::CallbackReturn::ERROR;
    }
    bridge_->receive_for(rx, kMaxRxFramesPerCycle, activate_timeout_ms_);

    // Configure CAN communication timeout (RID_TIMEOUT = 9, 500 ms). If the
    // motor receives no valid CAN frame for this period (e.g. USB cable
    // unplugged, serial bridge crash), it auto-disables as a safety fallback.
    if (!bridge_->send(
        damiao::make_parameter_write_frame(cfg.motor_id, damiao::kRidTimeout, damiao::kDefaultCanTimeout)))
    {
      RCLCPP_ERROR(
        logger_, "Joint '%s': failed to set CAN timeout.", info_.joints[i].name.c_str());
      disable_all_motors();
      return hardware_interface::CallbackReturn::ERROR;
    }
    bridge_->receive_for(rx, kMaxRxFramesPerCycle, activate_timeout_ms_);

    if (!bridge_->send(damiao::make_enable_frame(cfg.motor_id))) {
      disable_all_motors();
      return hardware_interface::CallbackReturn::ERROR;
    }
    bridge_->receive_for(rx, kMaxRxFramesPerCycle, activate_timeout_ms_);

    RCLCPP_INFO(
      logger_, "Joint '%s' (motor 0x%02X) enabled at %.3f.", info_.joints[i].name.c_str(),
      cfg.motor_id, hw_states_position_[i]);
  }

  std::fill(missed_replies_.begin(), missed_replies_.end(), 0u);
  error_log_counter_ = 0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ReBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  disable_all_motors();
  RCLCPP_INFO(logger_, "All motors disabled.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ReBotSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  disable_all_motors();
  bridge_->close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ReBotSystemHardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(logger_, "Hardware error: disabling all motors and closing the bridge.");
  disable_all_motors();
  bridge_->close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ReBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ReBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]);
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]);
    command_interfaces.emplace_back(info_.joints[i].name, HW_IF_KP, &hw_commands_kp_[i]);
    command_interfaces.emplace_back(info_.joints[i].name, HW_IF_KD, &hw_commands_kd_[i]);
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_effort_[i]);
  }
  return command_interfaces;
}

int ReBotSystemHardware::apply_feedback(const damiao::CanFrame & frame)
{
  for (size_t i = 0; i < joints_.size(); ++i) {
    const JointConfig & cfg = joints_[i];
    // Feedback frames arrive with the motor's MasterID; the slave id is
    // repeated in the low nibble of the first payload byte.
    if (frame.id != cfg.feedback_id && (frame.data[0] & 0x0F) != cfg.motor_id) {continue;}
    const auto fb = damiao::parse_feedback(frame, cfg.limits);
    hw_states_position_[i] = (fb.position - cfg.offset) / cfg.reduction;
    hw_states_velocity_[i] = fb.velocity / cfg.reduction;
    hw_states_effort_[i] = fb.torque * cfg.reduction;
    missed_replies_[i] = 0;
    if (damiao::is_fault(fb.error)) {
      if (error_log_counter_++ % 100 == 0) {
        RCLCPP_ERROR(
          logger_, "Joint '%s' (motor 0x%02X) fault 0x%X.", info_.joints[i].name.c_str(),
          cfg.motor_id, fb.error);
      }
      return -2;
    }
    return static_cast<int>(i);
  }
  return -1;
}

hardware_interface::return_type ReBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  damiao::CanFrame rx[kMaxRxFramesPerCycle];
  const size_t got = bridge_->receive(rx, kMaxRxFramesPerCycle);
  bool fault = false;
  for (size_t f = 0; f < got; ++f) {
    if (apply_feedback(rx[f]) == -2) {fault = true;}
  }
  if (fault) {
    return hardware_interface::return_type::ERROR;
  }
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (missed_replies_[i] > comm_error_threshold_) {
      if (error_log_counter_++ % 100 == 0) {
        RCLCPP_ERROR(
          logger_, "Joint '%s' (motor 0x%02X): %u cycles without feedback.",
          info_.joints[i].name.c_str(), joints_[i].motor_id, missed_replies_[i]);
      }
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ReBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < joints_.size(); ++i) {
    const JointConfig & cfg = joints_[i];
    // NaN commands fall back to safe defaults (see class docs).
    double pos_cmd = hw_commands_position_[i];
    if (std::isnan(pos_cmd)) {pos_cmd = hw_states_position_[i];}
    // Software limits: clamp in joint space before conversion.
    pos_cmd = std::clamp(pos_cmd, cfg.min_position, cfg.max_position);

    const double vel_cmd = std::isnan(hw_commands_velocity_[i]) ? 0.0 : hw_commands_velocity_[i];
    const double kp = std::isnan(hw_commands_kp_[i]) ?
      cfg.kp : std::clamp(hw_commands_kp_[i], damiao::kKpMin, damiao::kKpMax);
    const double kd = std::isnan(hw_commands_kd_[i]) ?
      cfg.kd : std::clamp(hw_commands_kd_[i], damiao::kKdMin, damiao::kKdMax);
    const double tau_cmd = std::isnan(hw_commands_effort_[i]) ? 0.0 : hw_commands_effort_[i];

    // Joint space -> motor space (virtual work: tau_joint = tau_motor * reduction).
    const double motor_pos = pos_cmd * cfg.reduction + cfg.offset;
    const double motor_vel = vel_cmd * cfg.reduction;
    const double motor_tau = tau_cmd / cfg.reduction;

    const auto frame =
      damiao::make_mit_frame(cfg.motor_id, cfg.limits, motor_pos, motor_vel, kp, kd, motor_tau);
    if (!bridge_->send(frame)) {
      if (error_log_counter_++ % 100 == 0) {
        RCLCPP_ERROR(
          logger_, "Joint '%s': serial write failed.", info_.joints[i].name.c_str());
      }
      return hardware_interface::return_type::ERROR;
    }
    ++missed_replies_[i];
  }
  return hardware_interface::return_type::OK;
}

void ReBotSystemHardware::disable_all_motors()
{
  if (!bridge_ || !bridge_->is_open()) {return;}
  damiao::CanFrame rx[kMaxRxFramesPerCycle];
  for (const auto & cfg : joints_) {
    bridge_->send(damiao::make_disable_frame(cfg.motor_id));
    bridge_->receive_for(rx, kMaxRxFramesPerCycle, 5);
  }
}

}  // namespace rebot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rebot_hardware::ReBotSystemHardware, hardware_interface::SystemInterface)
