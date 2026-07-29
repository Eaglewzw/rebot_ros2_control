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

#include "rebot_controllers/mit_trajectory_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

namespace rebot_controllers
{

controller_interface::CallbackReturn MitTrajectoryController::on_init()
{
  try {
    param_listener_ = std::make_shared<mit_trajectory_controller::ParamListener>(get_node());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MitTrajectoryController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
    mit_command_interface_names(params_.joints)};
}

controller_interface::InterfaceConfiguration
MitTrajectoryController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
    pos_vel_state_interface_names(params_.joints)};
}

controller_interface::CallbackReturn MitTrajectoryController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  const size_t n = params_.joints.size();
  if (n == 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (params_.kp.size() != n || params_.kd.size() != n) {
    RCLCPP_ERROR(get_node()->get_logger(), "'kp' and 'kd' must have one entry per joint");
    return controller_interface::CallbackReturn::ERROR;
  }

  use_gravity_ff_ = params_.use_gravity_ff;
  if (use_gravity_ff_) {
    if (params_.rated_torques.size() != n) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "'rated_torques' must have one entry per joint when use_gravity_ff is true");
      return controller_interface::CallbackReturn::ERROR;
    }
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

  q_ref_.assign(n, 0.0);
  qd_ref_.assign(n, 0.0);
  q_.assign(n, 0.0);
  g_tau_.assign(n, 0.0);
  hold_position_.assign(n, 0.0);
  start_position_.assign(n, 0.0);

  action_server_ = rclcpp_action::create_server<FollowJTraj>(
    get_node(), std::string(get_node()->get_name()) + "/follow_joint_trajectory",
    [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowJTraj::Goal> goal) {
      return goal_callback(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandle> goal_handle) {return cancel_callback(goal_handle);},
    [this](std::shared_ptr<GoalHandle> goal_handle) {accepted_callback(goal_handle);});

  return controller_interface::CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse MitTrajectoryController::goal_callback(
  const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const FollowJTraj::Goal> goal)
{
  if (get_node()->get_current_state().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_WARN(get_node()->get_logger(), "Rejecting goal: controller is not active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  const auto & traj = goal->trajectory;
  if (traj.points.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "Rejecting goal: empty trajectory");
    return rclcpp_action::GoalResponse::REJECT;
  }
  // Every controlled joint must be present; partial goals are not accepted.
  for (const auto & joint : params_.joints) {
    if (std::find(traj.joint_names.begin(), traj.joint_names.end(), joint) ==
      traj.joint_names.end())
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "Rejecting goal: joint '%s' missing", joint.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  double last_t = -1.0;
  for (const auto & point : traj.points) {
    const double t =
      rclcpp::Duration(point.time_from_start).seconds();
    if (t <= last_t || point.positions.size() != traj.joint_names.size()) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Rejecting goal: non-monotonic time_from_start or wrong positions size");
      return rclcpp_action::GoalResponse::REJECT;
    }
    last_t = t;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MitTrajectoryController::cancel_callback(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  if (rt_active_goal_ && rt_active_goal_->gh_ == goal_handle) {
    // Replace the active trajectory by a hold (picked up in update()).
    auto hold = std::make_shared<ActiveTrajectory>();
    trajectory_buffer_.writeFromNonRT(hold);
    auto result = std::make_shared<FollowJTraj::Result>();
    rt_active_goal_->setCanceled(result);
    rt_active_goal_.reset();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MitTrajectoryController::accepted_callback(std::shared_ptr<GoalHandle> goal_handle)
{
  // Non-RT: reorder the trajectory into controller joint order.
  const auto goal = goal_handle->get_goal();
  const auto & traj = goal->trajectory;
  const size_t n = params_.joints.size();

  std::vector<size_t> index_map(n, 0);
  for (size_t i = 0; i < n; ++i) {
    const auto it = std::find(traj.joint_names.begin(), traj.joint_names.end(),
      params_.joints[i]);
    index_map[i] = static_cast<size_t>(std::distance(traj.joint_names.begin(), it));
  }

  std::vector<TrajectoryPoint> points;
  points.reserve(traj.points.size());
  for (const auto & src : traj.points) {
    TrajectoryPoint dst;
    dst.time_from_start = rclcpp::Duration(src.time_from_start).seconds();
    dst.positions.resize(n);
    const bool has_vel = src.velocities.size() == traj.joint_names.size();
    if (has_vel) {dst.velocities.resize(n);}
    for (size_t i = 0; i < n; ++i) {
      dst.positions[i] = src.positions[index_map[i]];
      if (has_vel) {dst.velocities[i] = src.velocities[index_map[i]];}
    }
    points.push_back(std::move(dst));
  }

  // Preempt any previous goal.
  if (rt_active_goal_) {
    auto result = std::make_shared<FollowJTraj::Result>();
    result->error_code = FollowJTraj::Result::INVALID_GOAL;
    result->error_string = "Preempted by a new goal";
    rt_active_goal_->setAborted(result);
  }

  auto active = std::make_shared<ActiveTrajectory>();
  active->trajectory = Trajectory(std::move(points), n);
  active->goal_handle = std::make_shared<RealtimeGoalHandle>(goal_handle);
  // Preallocate feedback so update() only copies into existing storage.
  auto & fb = active->goal_handle->preallocated_feedback_;
  fb->joint_names = params_.joints;
  fb->desired.positions.resize(n);
  fb->desired.velocities.resize(n);
  fb->actual.positions.resize(n);
  // Mark the realtime handle as executing — without this every later
  // setSucceeded/setAborted call is silently ignored.
  active->goal_handle->execute();
  rt_active_goal_ = active->goal_handle;
  trajectory_buffer_.writeFromNonRT(active);

  // Timer that shovels feedback/result from the RT thread to the action
  // client (official JTC pattern).
  goal_handle_timer_ = get_node()->create_wall_timer(
    std::chrono::milliseconds(50), [this]() {
      if (rt_active_goal_) {rt_active_goal_->runNonRealtime();}
    });
}

controller_interface::CallbackReturn MitTrajectoryController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!command_handles_.assign(command_interfaces_, params_.joints) ||
    !state_handles_.assign(state_interfaces_, params_.joints))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to order command/state interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }
  // Hold the current position until a goal arrives (no power-on jump).
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    hold_position_[i] = state_handles_.position[i].get().get_value();
  }
  holding_ = true;
  trajectory_buffer_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MitTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (rt_active_goal_) {
    auto result = std::make_shared<FollowJTraj::Result>();
    result->error_code = FollowJTraj::Result::INVALID_GOAL;
    result->error_string = "Controller deactivated";
    rt_active_goal_->setAborted(result);
    rt_active_goal_.reset();
  }
  goal_handle_timer_.reset();
  command_handles_.write_safe_defaults();
  command_handles_.release();
  state_handles_.release();
  return controller_interface::CallbackReturn::SUCCESS;
}

void MitTrajectoryController::hold_current_position()
{
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    hold_position_[i] = state_handles_.position[i].get().get_value();
  }
  holding_ = true;
}

controller_interface::return_type MitTrajectoryController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (param_listener_->is_old(params_)) {
    // Only scalar/gain fields are hot-swappable; joints are fixed after
    // configure. get_params() copies, which is acceptable at 100 Hz.
    params_ = param_listener_->get_params();
  }
  const size_t n = params_.joints.size();

  for (size_t i = 0; i < n; ++i) {
    q_[i] = state_handles_.position[i].get().get_value();
  }

  auto active_ptr = *trajectory_buffer_.readFromRT();
  const bool has_traj = active_ptr && !active_ptr->trajectory.empty();

  if (has_traj) {
    auto & active = *active_ptr;
    if (!active.started) {
      active.start_time = time;
      active.started = true;
      sample_hint_ = 0;
      holding_ = false;
      // Reference for the ramp-in below (JTC semantics: a first point with
      // time_from_start > 0 is reached by interpolating from the current
      // position, never by jumping).
      for (size_t i = 0; i < n; ++i) {start_position_[i] = q_[i];}
    }
    const double t = (time - active.start_time).seconds();
    const auto & first = active.trajectory.points().front();
    if (t < first.time_from_start) {
      const double s = t / first.time_from_start;
      for (size_t i = 0; i < n; ++i) {
        const double delta = first.positions[i] - start_position_[i];
        q_ref_[i] = start_position_[i] + s * delta;
        qd_ref_[i] = delta / first.time_from_start;
      }
    } else {
      active.trajectory.sample(t, q_ref_, qd_ref_, sample_hint_);
    }

    const bool past_end = t >= active.trajectory.duration();
    auto & goal = active.goal_handle;

    // Path tolerance (during execution only). Note: goal-handle result
    // methods are RT-safe (deferred to the non-RT timer); the trajectory
    // buffer is never written from this thread — `holding_` overrides the
    // sampled reference instead.
    if (!holding_ && !past_end && params_.path_tolerance.size() == n) {
      for (size_t i = 0; i < n; ++i) {
        const double tol = params_.path_tolerance[i];
        if (tol > 0.0 && std::abs(q_[i] - q_ref_[i]) > tol) {
          if (goal) {
            auto result = std::make_shared<FollowJTraj::Result>();
            result->error_code = FollowJTraj::Result::PATH_TOLERANCE_VIOLATED;
            goal->setAborted(result);
            active.goal_handle.reset();
          }
          hold_current_position();
          break;
        }
      }
    }

    if (!holding_ && past_end) {
      // Goal tolerance / goal time tolerance.
      bool within_goal = true;
      if (params_.goal_tolerance.size() == n) {
        for (size_t i = 0; i < n; ++i) {
          const double tol = params_.goal_tolerance[i];
          if (tol > 0.0 && std::abs(q_[i] - q_ref_[i]) > tol) {within_goal = false;}
        }
      }
      if (within_goal) {
        if (goal) {
          auto result = std::make_shared<FollowJTraj::Result>();
          result->error_code = FollowJTraj::Result::SUCCESSFUL;
          goal->setSucceeded(result);
          active.goal_handle.reset();
        }
        for (size_t i = 0; i < n; ++i) {hold_position_[i] = q_ref_[i];}
        holding_ = true;
      } else if (
        params_.goal_time_tolerance > 0.0 &&
        t > active.trajectory.duration() + params_.goal_time_tolerance)
      {
        if (goal) {
          auto result = std::make_shared<FollowJTraj::Result>();
          result->error_code = FollowJTraj::Result::GOAL_TOLERANCE_VIOLATED;
          goal->setAborted(result);
          active.goal_handle.reset();
        }
        hold_current_position();
      }
    }

    // Feedback (published by the non-RT timer; storage preallocated).
    if (goal) {
      auto & fb = goal->preallocated_feedback_;
      fb->header.stamp = time;
      std::copy(q_ref_.begin(), q_ref_.end(), fb->desired.positions.begin());
      std::copy(qd_ref_.begin(), qd_ref_.end(), fb->desired.velocities.begin());
      std::copy(q_.begin(), q_.end(), fb->actual.positions.begin());
      goal->setFeedback(fb);
    }
  }

  if (holding_ || !has_traj) {
    for (size_t i = 0; i < n; ++i) {
      q_ref_[i] = hold_position_[i];
      qd_ref_[i] = 0.0;
    }
  }

  if (use_gravity_ff_) {
    gravity_.compute(q_, g_tau_);
  }

  for (size_t i = 0; i < n; ++i) {
    double tau = 0.0;
    if (use_gravity_ff_) {
      const double limit = params_.torque_limit_ratio * params_.rated_torques[i];
      tau = std::clamp(g_tau_[i], -limit, limit);
    }
    command_handles_.position[i].get().set_value(q_ref_[i]);
    command_handles_.velocity[i].get().set_value(qd_ref_[i]);
    command_handles_.kp[i].get().set_value(params_.kp[i]);
    command_handles_.kd[i].get().set_value(params_.kd[i]);
    command_handles_.effort[i].get().set_value(tau);
  }
  return controller_interface::return_type::OK;
}

}  // namespace rebot_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  rebot_controllers::MitTrajectoryController, controller_interface::ControllerInterface)
