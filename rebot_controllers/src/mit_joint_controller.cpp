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

#include "rebot_controllers/mit_joint_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace rebot_controllers
{

controller_interface::CallbackReturn MitJointController::on_init()
{
  try {
    param_listener_ = std::make_shared<mit_joint_controller::ParamListener>(get_node());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MitJointController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
    mit_command_interface_names(params_.joints)};
}

controller_interface::InterfaceConfiguration
MitJointController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
    pos_vel_state_interface_names(params_.joints)};
}

controller_interface::CallbackReturn MitJointController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  command_sub_ = get_node()->create_subscription<rebot_msgs::msg::MitJointCommand>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const rebot_msgs::msg::MitJointCommand & msg) {command_callback(msg);});

  return controller_interface::CallbackReturn::SUCCESS;
}

void MitJointController::command_callback(const rebot_msgs::msg::MitJointCommand & msg)
{
  // Non-RT thread: validate, reorder to controller joint order, push to the
  // realtime buffer. Unknown joints are ignored; missing joints/fields stay
  // NaN ("not set" -> hardware defaults).
  const size_t n = params_.joints.size();
  auto cmd = std::make_shared<Command>();
  cmd->position.assign(n, kNaN);
  cmd->velocity.assign(n, kNaN);
  cmd->kp.assign(n, kNaN);
  cmd->kd.assign(n, kNaN);
  cmd->effort.assign(n, kNaN);
  cmd->stamp = get_node()->now();

  auto field = [](const std::vector<double> & src, size_t j) {
      return j < src.size() ? src[j] : kNaN;
    };

  for (size_t j = 0; j < msg.joint_names.size(); ++j) {
    const auto it = std::find(params_.joints.begin(), params_.joints.end(), msg.joint_names[j]);
    if (it == params_.joints.end()) {continue;}
    const size_t i = static_cast<size_t>(std::distance(params_.joints.begin(), it));
    cmd->position[i] = field(msg.position, j);
    cmd->velocity[i] = field(msg.velocity, j);
    cmd->kp[i] = field(msg.kp, j);
    cmd->kd[i] = field(msg.kd, j);
    cmd->effort[i] = field(msg.effort, j);
  }
  command_buffer_.writeFromNonRT(cmd);
}

controller_interface::CallbackReturn MitJointController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!command_handles_.assign(command_interfaces_, params_.joints) ||
    !state_handles_.assign(state_interfaces_, params_.joints))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to order command/state interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }
  // No stale commands from a previous activation; hardware defaults hold
  // the current position until the first message arrives.
  command_buffer_.reset();
  command_handles_.write_safe_defaults();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MitJointController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  command_handles_.write_safe_defaults();
  command_handles_.release();
  state_handles_.release();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MitJointController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }

  const auto cmd = *command_buffer_.readFromRT();
  if (!cmd) {
    return controller_interface::return_type::OK;  // hardware defaults hold
  }
  if (params_.command_timeout > 0.0 &&
    (time - cmd->stamp).seconds() > params_.command_timeout)
  {
    command_handles_.write_safe_defaults();
    return controller_interface::return_type::OK;
  }

  for (size_t i = 0; i < params_.joints.size(); ++i) {
    command_handles_.position[i].get().set_value(cmd->position[i]);
    command_handles_.velocity[i].get().set_value(cmd->velocity[i]);
    command_handles_.kp[i].get().set_value(cmd->kp[i]);
    command_handles_.kd[i].get().set_value(cmd->kd[i]);
    command_handles_.effort[i].get().set_value(cmd->effort[i]);
  }
  return controller_interface::return_type::OK;
}

}  // namespace rebot_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  rebot_controllers::MitJointController, controller_interface::ControllerInterface)
