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

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "rebot_controllers/kdl_gravity.hpp"

namespace
{

// Single-link pendulum: revolute joint about +Y at the origin, point mass
// m = 2 kg at l = 0.5 m along +X when q = 0.
// Analytic gravity torque about Y: tau = m * g * l * cos(q).
constexpr char kPendulumUrdf[] = R"(<?xml version="1.0"?>
<robot name="pendulum">
  <link name="base_link"/>
  <link name="arm">
    <inertial>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
  </link>
  <joint name="pivot" type="revolute">
    <parent link="base_link"/>
    <child link="arm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="10"/>
  </joint>
</robot>)";

constexpr double kG = 9.81;

}  // namespace

TEST(KdlGravity, PendulumMatchesAnalyticTorque)
{
  rebot_controllers::KdlGravity gravity;
  std::string error;
  ASSERT_TRUE(gravity.init(kPendulumUrdf, "base_link", "arm", {"pivot"}, error)) << error;

  std::vector<double> q(1), tau(1);
  for (double angle : {0.0, 0.3, 1.0, M_PI / 2.0, -0.7}) {
    q[0] = angle;
    gravity.compute(q, tau);
    // JntToGravity returns G(q) of  tau = M qdd + C + G : the torque the
    // joint must apply to hold the pose. With the +y axis convention a
    // positive joint torque pushes the mass down, so the holding torque is
    // -m*g*l*cos(q).
    EXPECT_NEAR(tau[0], -2.0 * kG * 0.5 * std::cos(angle), 1e-9) << "q=" << angle;
  }
}

TEST(KdlGravity, UnmappedJointGetsZeroTorque)
{
  rebot_controllers::KdlGravity gravity;
  std::string error;
  // "gripper" is not part of the chain -> mapped to -1 -> zero torque.
  ASSERT_TRUE(gravity.init(kPendulumUrdf, "base_link", "arm", {"pivot", "gripper"}, error))
    << error;

  std::vector<double> q = {0.0, 0.123};
  std::vector<double> tau(2, 99.0);
  gravity.compute(q, tau);
  EXPECT_NEAR(tau[0], -2.0 * kG * 0.5, 1e-9);
  EXPECT_DOUBLE_EQ(tau[1], 0.0);
}

TEST(KdlGravity, FailsOnMissingChainJoint)
{
  rebot_controllers::KdlGravity gravity;
  std::string error;
  // Chain joint 'pivot' is not covered by the given joint names.
  EXPECT_FALSE(gravity.init(kPendulumUrdf, "base_link", "arm", {"other"}, error));
  EXPECT_FALSE(error.empty());
}

TEST(KdlGravity, FailsOnBadLinks)
{
  rebot_controllers::KdlGravity gravity;
  std::string error;
  EXPECT_FALSE(gravity.init(kPendulumUrdf, "base_link", "nope", {"pivot"}, error));
  EXPECT_FALSE(gravity.init("not a urdf", "base_link", "arm", {"pivot"}, error));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
