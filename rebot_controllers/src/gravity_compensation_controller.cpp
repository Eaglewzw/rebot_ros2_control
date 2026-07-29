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

#include "rebot_controllers/gravity_compensation_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace rebot_controllers
{

controller_interface::CallbackReturn GravityCompensationController::on_init()
{
  try {
    param_listener_ =
      std::make_shared<gravity_compensation_controller::ParamListener>(get_node());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
GravityCompensationController::command_interface_configuration() const
{
  // Claims the full tuple. position/velocity are functionally irrelevant
  // (kp = 0) and are fed with the measured state; claiming them anyway keeps
  // mock_components/GenericSystem switchable (its command-mode check
  // requires a position/velocity/acceleration interface per joint) and
  // makes the mutual exclusion with other arm controllers explicit.
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
    mit_command_interface_names(params_.joints)};
}

controller_interface::InterfaceConfiguration
GravityCompensationController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
    pos_vel_state_interface_names(params_.joints)};
}

controller_interface::CallbackReturn GravityCompensationController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (params_.rated_torques.size() != params_.joints.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "'rated_torques' must have one entry per joint (%zu != %zu)",
      params_.rated_torques.size(), params_.joints.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  std::string urdf = params_.robot_description;
  if (urdf.empty()) {
    urdf = fetch_robot_description(params_.robot_description_timeout);
  }
  if (urdf.empty()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "No URDF: set the 'robot_description' parameter or publish /robot_description");
    return controller_interface::CallbackReturn::ERROR;
  }
  std::string error;
  if (!gravity_.init(urdf, params_.base_link, params_.tip_link, params_.joints, error)) {
    RCLCPP_ERROR(get_node()->get_logger(), "KDL gravity init failed: %s", error.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  positions_.assign(params_.joints.size(), 0.0);
  gravity_torques_.assign(params_.joints.size(), 0.0);

  torque_pub_ = get_node()->create_publisher<TorqueMsg>(
    "~/gravity_torques", rclcpp::SystemDefaultsQoS());
  rt_torque_pub_ = std::make_unique<realtime_tools::RealtimePublisher<TorqueMsg>>(torque_pub_);
  rt_torque_pub_->msg_.data.assign(params_.joints.size(), 0.0);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!command_handles_.assign(command_interfaces_, params_.joints) ||
    !state_handles_.assign(state_interfaces_, params_.joints))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to order command/state interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  command_handles_.write_safe_defaults();
  command_handles_.release();
  state_handles_.release();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GravityCompensationController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }

  const size_t n = params_.joints.size();
  for (size_t i = 0; i < n; ++i) {
    positions_[i] = state_handles_.position[i].get().get_value();
  }
  gravity_.compute(positions_, gravity_torques_);

  for (size_t i = 0; i < n; ++i) {
    const double limit = params_.torque_limit_ratio * params_.rated_torques[i];
    const double tau =
      std::clamp(params_.gravity_scale * gravity_torques_[i], -limit, limit);
    command_handles_.position[i].get().set_value(positions_[i]);
    command_handles_.velocity[i].get().set_value(0.0);
    command_handles_.kp[i].get().set_value(0.0);
    command_handles_.kd[i].get().set_value(params_.kd_damping);
    command_handles_.effort[i].get().set_value(tau);
    gravity_torques_[i] = tau;
  }

  if (rt_torque_pub_ && rt_torque_pub_->trylock()) {
    for (size_t i = 0; i < n; ++i) {rt_torque_pub_->msg_.data[i] = gravity_torques_[i];}
    rt_torque_pub_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

}  // namespace rebot_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  rebot_controllers::GravityCompensationController, controller_interface::ControllerInterface)
