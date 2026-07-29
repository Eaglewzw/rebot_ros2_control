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

#include "rebot_controllers/teleop_stream_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace rebot_controllers
{

controller_interface::CallbackReturn TeleopStreamController::on_init()
{
  try {
    param_listener_ = std::make_shared<teleop_stream_controller::ParamListener>(get_node());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
TeleopStreamController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
    mit_command_interface_names(params_.joints)};
}

controller_interface::InterfaceConfiguration
TeleopStreamController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
    pos_vel_state_interface_names(params_.joints)};
}

controller_interface::CallbackReturn TeleopStreamController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  const size_t n = params_.joints.size();
  if (n == 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (params_.kp.size() != n || params_.kd.size() != n || params_.max_velocity.size() != n ||
    params_.max_acceleration.size() != n)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "'kp', 'kd', 'max_velocity' and 'max_acceleration' must have one entry per joint");
    return controller_interface::CallbackReturn::ERROR;
  }

  limiters_.assign(n, RateLimiter{});
  smoothed_.assign(n, 0.0);
  target_.assign(n, 0.0);

  command_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands", rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::Float64MultiArray & msg) {
      if (msg.data.size() != params_.joints.size()) {return;}
      auto target = std::make_shared<StampedTarget>();
      target->positions = msg.data;
      target->stamp = get_node()->now();
      target_buffer_.writeFromNonRT(target);
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TeleopStreamController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!command_handles_.assign(command_interfaces_, params_.joints) ||
    !state_handles_.assign(state_interfaces_, params_.joints))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to order command/state interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    smoothed_[i] = state_handles_.position[i].get().get_value();
    target_[i] = smoothed_[i];
    limiters_[i].reset();
  }
  timed_out_ = false;
  target_buffer_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TeleopStreamController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  command_handles_.write_safe_defaults();
  command_handles_.release();
  state_handles_.release();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TeleopStreamController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }
  const size_t n = params_.joints.size();
  const double dt = period.seconds();

  const auto latest = *target_buffer_.readFromRT();
  const bool fresh =
    latest && (time - latest->stamp).seconds() <= params_.command_timeout;

  if (fresh) {
    if (timed_out_) {
      RCLCPP_INFO_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Command stream resumed");
    }
    timed_out_ = false;
    for (size_t i = 0; i < n; ++i) {target_[i] = latest->positions[i];}
  } else if (!timed_out_) {
    // Stream stalled: latch onto the current smoothed position with raised
    // damping until commands resume.
    timed_out_ = true;
    for (size_t i = 0; i < n; ++i) {
      target_[i] = smoothed_[i];
      limiters_[i].reset();
    }
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Command stream timed out; holding position with raised damping");
  }

  for (size_t i = 0; i < n; ++i) {
    smoothed_[i] = limiters_[i].step(
      smoothed_[i], target_[i], params_.max_velocity[i], params_.max_acceleration[i], dt);

    const double kd = timed_out_ ?
      std::min(params_.kd[i] * params_.timeout_kd_scale, 5.0) : params_.kd[i];
    command_handles_.position[i].get().set_value(smoothed_[i]);
    command_handles_.velocity[i].get().set_value(limiters_[i].velocity);
    command_handles_.kp[i].get().set_value(params_.kp[i]);
    command_handles_.kd[i].get().set_value(kd);
    command_handles_.effort[i].get().set_value(0.0);
  }
  return controller_interface::return_type::OK;
}

}  // namespace rebot_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  rebot_controllers::TeleopStreamController, controller_interface::ControllerInterface)
