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

#include "rebot_controllers/kdl_gravity.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace rebot_controllers
{

bool KdlGravity::init(
  const std::string & urdf, const std::string & base_link, const std::string & tip_link,
  const std::vector<std::string> & joint_names, std::string & error)
{
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf, tree)) {
    error = "cannot parse URDF into a KDL tree";
    return false;
  }
  if (!tree.getChain(base_link, tip_link, chain_)) {
    error = "no kinematic chain from '" + base_link + "' to '" + tip_link + "'";
    return false;
  }

  // Chain joint names in q-index order (fixed joints carry no q).
  std::vector<std::string> chain_joints;
  for (unsigned int s = 0; s < chain_.getNrOfSegments(); ++s) {
    const KDL::Joint & joint = chain_.getSegment(s).getJoint();
    if (joint.getType() != KDL::Joint::None) {chain_joints.push_back(joint.getName());}
  }

  // Explicit ros2_control order -> chain q-index mapping (never assume the
  // orders coincide).
  map_.assign(joint_names.size(), -1);
  size_t mapped = 0;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    for (size_t c = 0; c < chain_joints.size(); ++c) {
      if (chain_joints[c] == joint_names[i]) {
        map_[i] = static_cast<int>(c);
        ++mapped;
        break;
      }
    }
  }
  if (mapped != chain_joints.size()) {
    error = "chain has " + std::to_string(chain_joints.size()) +
      " joints but only " + std::to_string(mapped) + " were matched by name";
    return false;
  }

  q_.resize(chain_.getNrOfJoints());
  g_.resize(chain_.getNrOfJoints());
  KDL::SetToZero(q_);
  KDL::SetToZero(g_);
  dyn_ = std::make_unique<KDL::ChainDynParam>(chain_, KDL::Vector(0.0, 0.0, -9.81));
  return true;
}

void KdlGravity::compute(const std::vector<double> & positions, std::vector<double> & torques_out)
{
  for (size_t i = 0; i < map_.size(); ++i) {
    if (map_[i] >= 0) {q_(map_[i]) = positions[i];}
  }
  dyn_->JntToGravity(q_, g_);
  for (size_t i = 0; i < map_.size(); ++i) {
    torques_out[i] = map_[i] >= 0 ? g_(map_[i]) : 0.0;
  }
}

std::string fetch_robot_description(double timeout_sec)
{
  // Temporary node + own executor: safe inside a controller's on_configure()
  // (the controller_manager executor is busy running that very callback).
  auto node = std::make_shared<rclcpp::Node>(
    "rebot_controllers_robot_description_client",
    rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(false));
  std::string urdf;
  auto sub = node->create_subscription<std_msgs::msg::String>(
    "/robot_description", rclcpp::QoS(1).transient_local(),
    [&urdf](const std_msgs::msg::String & msg) {urdf = msg.data;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
  while (urdf.empty() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(50));
  }
  return urdf;
}

}  // namespace rebot_controllers
