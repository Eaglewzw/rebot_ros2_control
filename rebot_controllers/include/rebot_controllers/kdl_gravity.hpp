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

#ifndef REBOT_CONTROLLERS__KDL_GRAVITY_HPP_
#define REBOT_CONTROLLERS__KDL_GRAVITY_HPP_

/// \file kdl_gravity.hpp
/// \brief Gravity torque g(q) from the URDF via KDL ChainDynParam.
///
/// The environment provides no C++ Pinocchio, so KDL (kdl_parser +
/// ChainDynParam::JntToGravity) is used instead; the module is isolated so a
/// Pinocchio backend can be swapped in without touching the controllers.

#include <memory>
#include <string>
#include <vector>

#include "kdl/chaindynparam.hpp"
#include "kdl/chain.hpp"
#include "kdl/jntarray.hpp"

namespace rebot_controllers
{

/// Gravity torque calculator, realtime-safe after init().
class KdlGravity
{
public:
  /// Build the KDL chain base_link -> tip_link from a URDF string and map
  /// the given ros2_control joint order onto the chain joints. Joints that
  /// are not part of the chain (e.g. the gripper) get zero gravity torque.
  /// Returns false and fills `error` on failure.
  bool init(
    const std::string & urdf, const std::string & base_link, const std::string & tip_link,
    const std::vector<std::string> & joint_names, std::string & error);

  bool initialized() const {return dyn_ != nullptr;}

  /// g(q) for the mapped joints. `positions` / `torques_out` are in
  /// ros2_control joint order and have size `joint_names.size()` from
  /// init(). No allocation, no locking.
  void compute(const std::vector<double> & positions, std::vector<double> & torques_out);

private:
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_;
  KDL::JntArray q_;
  KDL::JntArray g_;
  /// Per ros2_control joint: index into the chain's joint array, or -1.
  std::vector<int> map_;
};

/// Fetch the URDF from the /robot_description topic (transient_local, as
/// published by robot_state_publisher) using a temporary node with its own
/// executor, so it is safe to call from a controller's on_configure().
/// Returns an empty string on timeout.
std::string fetch_robot_description(double timeout_sec);

}  // namespace rebot_controllers

#endif  // REBOT_CONTROLLERS__KDL_GRAVITY_HPP_
