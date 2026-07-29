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

#include "rebot_controllers/joint_impedance_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace rebot_controllers
{

controller_interface::CallbackReturn JointImpedanceController::on_init()
{
  try {
    param_listener_ = std::make_shared<joint_impedance_controller::ParamListener>(get_node());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const
{
  // Full tuple: onboard mode writes q_ref into the position interface.
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
    mit_command_interface_names(params_.joints)};
}

controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
    pos_vel_state_interface_names(params_.joints)};
}

controller_interface::CallbackReturn JointImpedanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  const size_t n = params_.joints.size();
  if (n == 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (params_.stiffness.size() != n || params_.damping.size() != n) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "'stiffness' and 'damping' must have one entry per joint");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (params_.mode != "onboard" && params_.mode != "software") {
    RCLCPP_ERROR(get_node()->get_logger(), "'mode' must be 'onboard' or 'software'");
    return controller_interface::CallbackReturn::ERROR;
  }
  software_mode_ = params_.mode == "software";
  if (!software_mode_) {
    for (size_t i = 0; i < n; ++i) {
      if (params_.stiffness[i] > 500.0 || params_.damping[i] > 5.0) {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "onboard mode: stiffness <= 500 and damping <= 5 required (MIT encoding range); "
          "use mode 'software' for higher gains");
        return controller_interface::CallbackReturn::ERROR;
      }
    }
  }
  if (params_.rated_torques.size() != n) {
    RCLCPP_ERROR(get_node()->get_logger(), "'rated_torques' must have one entry per joint");
    return controller_interface::CallbackReturn::ERROR;
  }

  use_gravity_ff_ = params_.use_gravity_ff;
  if (use_gravity_ff_) {
    std::string urdf = params_.robot_description;
    if (urdf.empty()) {urdf = fetch_robot_description(params_.robot_description_timeout);}
    std::string error;
    if (urdf.empty() ||
      !gravity_.init(urdf, params_.base_link, params_.tip_link, params_.joints, error))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Gravity feed-forward unavailable (%s); disabling it.",
        urdf.empty() ? "no URDF" : error.c_str());
      use_gravity_ff_ = false;
    }
  }

  q_.assign(n, 0.0);
  qd_.assign(n, 0.0);
  g_tau_.assign(n, 0.0);

  reference_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/reference", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray & msg) {
      if (msg.data.size() != params_.joints.size()) {return;}
      reference_buffer_.writeFromNonRT(std::make_shared<std::vector<double>>(msg.data));
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface>
JointImpedanceController::on_export_reference_interfaces()
{
  const size_t n = params_.joints.size();
  reference_interfaces_.assign(n, kNaN);
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    interfaces.emplace_back(
      get_node()->get_name(),
      params_.joints[i] + "/" + hardware_interface::HW_IF_POSITION,
      &reference_interfaces_[i]);
  }
  return interfaces;
}

controller_interface::CallbackReturn JointImpedanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!command_handles_.assign(command_interfaces_, params_.joints) ||
    !state_handles_.assign(state_interfaces_, params_.joints))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to order command/state interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }
  // Reference = current position at activation (no jump).
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    reference_interfaces_[i] = state_handles_.position[i].get().get_value();
  }
  reference_buffer_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  command_handles_.write_safe_defaults();
  command_handles_.release();
  state_handles_.release();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointImpedanceController::update_reference_from_subscribers()
{
  // Not chained: take the latest topic reference (if any).
  const auto ref = *reference_buffer_.readFromRT();
  if (ref) {
    for (size_t i = 0; i < reference_interfaces_.size(); ++i) {
      reference_interfaces_[i] = (*ref)[i];
    }
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type JointImpedanceController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }
  const size_t n = params_.joints.size();

  for (size_t i = 0; i < n; ++i) {
    q_[i] = state_handles_.position[i].get().get_value();
    qd_[i] = state_handles_.velocity[i].get().get_value();
  }
  if (use_gravity_ff_) {gravity_.compute(q_, g_tau_);}

  for (size_t i = 0; i < n; ++i) {
    double q_ref = reference_interfaces_[i];
    if (std::isnan(q_ref)) {q_ref = q_[i];}
    const double limit = params_.torque_limit_ratio * params_.rated_torques[i];
    const double g = use_gravity_ff_ ? g_tau_[i] : 0.0;

    if (software_mode_) {
      // tau = K (q_ref - q) + D (0 - qd) + g(q), closed at the update rate.
      const double tau = std::clamp(
        params_.stiffness[i] * (q_ref - q_[i]) - params_.damping[i] * qd_[i] + g,
        -limit, limit);
      command_handles_.position[i].get().set_value(q_[i]);
      command_handles_.velocity[i].get().set_value(0.0);
      command_handles_.kp[i].get().set_value(0.0);
      command_handles_.kd[i].get().set_value(0.0);
      command_handles_.effort[i].get().set_value(tau);
    } else {
      // Onboard: the motor's MIT loop closes the impedance at current-loop
      // rate; software only supplies the gravity feed-forward.
      command_handles_.position[i].get().set_value(q_ref);
      command_handles_.velocity[i].get().set_value(0.0);
      command_handles_.kp[i].get().set_value(params_.stiffness[i]);
      command_handles_.kd[i].get().set_value(params_.damping[i]);
      command_handles_.effort[i].get().set_value(std::clamp(g, -limit, limit));
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace rebot_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  rebot_controllers::JointImpedanceController, controller_interface::ChainableControllerInterface)
