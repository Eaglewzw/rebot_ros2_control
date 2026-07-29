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

#ifndef REBOT_CONTROLLERS__MIT_INTERFACE_HELPERS_HPP_
#define REBOT_CONTROLLERS__MIT_INTERFACE_HELPERS_HPP_

/// \file mit_interface_helpers.hpp
/// \brief Shared handling of the per-joint Damiao MIT command tuple
///        (position / velocity / kp / kd / effort) exported by
///        rebot_hardware, plus the NaN "not set" sentinel convention.

#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace rebot_controllers
{

constexpr char HW_IF_KP[] = "kp";
constexpr char HW_IF_KD[] = "kd";

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// Command-interface name list for the MIT tuple, in claim order. With
/// `with_position_velocity` false only kp/kd/effort are claimed (pure
/// torque-mode controllers such as gravity compensation).
inline std::vector<std::string> mit_command_interface_names(
  const std::vector<std::string> & joints, bool with_position_velocity = true)
{
  std::vector<std::string> names;
  names.reserve(joints.size() * (with_position_velocity ? 5 : 3));
  for (const auto & joint : joints) {
    if (with_position_velocity) {
      names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
      names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    names.push_back(joint + "/" + HW_IF_KP);
    names.push_back(joint + "/" + HW_IF_KD);
    names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return names;
}

/// Per-type ordered references into a controller's command_interfaces_.
struct MitCommandHandles
{
  using CmdRef = std::reference_wrapper<hardware_interface::LoanedCommandInterface>;

  std::vector<CmdRef> position;
  std::vector<CmdRef> velocity;
  std::vector<CmdRef> kp;
  std::vector<CmdRef> kd;
  std::vector<CmdRef> effort;

  bool assign(
    std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces,
    const std::vector<std::string> & joints, bool with_position_velocity = true)
  {
    release();
    bool ok = true;
    if (with_position_velocity) {
      ok = controller_interface::get_ordered_interfaces(
        command_interfaces, joints, hardware_interface::HW_IF_POSITION, position) &&
        controller_interface::get_ordered_interfaces(
        command_interfaces, joints, hardware_interface::HW_IF_VELOCITY, velocity);
    }
    return ok &&
           controller_interface::get_ordered_interfaces(
      command_interfaces, joints, HW_IF_KP, kp) &&
           controller_interface::get_ordered_interfaces(
      command_interfaces, joints, HW_IF_KD, kd) &&
           controller_interface::get_ordered_interfaces(
      command_interfaces, joints, hardware_interface::HW_IF_EFFORT, effort);
  }

  void release()
  {
    position.clear();
    velocity.clear();
    kp.clear();
    kd.clear();
    effort.clear();
  }

  /// Write the NaN sentinel everywhere: the hardware falls back to
  /// "hold position with URDF-default gains, zero feed-forward". Called on
  /// deactivation so controller switching is torque-jump free.
  void write_safe_defaults()
  {
    for (auto & handle : position) {handle.get().set_value(kNaN);}
    for (auto & handle : velocity) {handle.get().set_value(kNaN);}
    for (auto & handle : kp) {handle.get().set_value(kNaN);}
    for (auto & handle : kd) {handle.get().set_value(kNaN);}
    for (auto & handle : effort) {handle.get().set_value(kNaN);}
  }
};

/// Ordered position/velocity state handles.
struct JointStateHandles
{
  using StateRef = std::reference_wrapper<hardware_interface::LoanedStateInterface>;

  std::vector<StateRef> position;
  std::vector<StateRef> velocity;

  bool assign(
    std::vector<hardware_interface::LoanedStateInterface> & state_interfaces,
    const std::vector<std::string> & joints)
  {
    position.clear();
    velocity.clear();
    return controller_interface::get_ordered_interfaces(
      state_interfaces, joints, hardware_interface::HW_IF_POSITION, position) &&
           controller_interface::get_ordered_interfaces(
      state_interfaces, joints, hardware_interface::HW_IF_VELOCITY, velocity);
  }

  void release()
  {
    position.clear();
    velocity.clear();
  }
};

/// State-interface name list (position + velocity).
inline std::vector<std::string> pos_vel_state_interface_names(
  const std::vector<std::string> & joints)
{
  std::vector<std::string> names;
  names.reserve(joints.size() * 2);
  for (const auto & joint : joints) {
    names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return names;
}

}  // namespace rebot_controllers

#endif  // REBOT_CONTROLLERS__MIT_INTERFACE_HELPERS_HPP_
