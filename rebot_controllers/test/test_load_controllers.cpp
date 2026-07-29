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

// Load test in the style of the official ros2_controllers
// test_load_* suites: every plugin must be loadable by a ControllerManager.

#include <gtest/gtest.h>

#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

namespace
{

// Minimal robot on mock hardware. The binary-installed
// ros2_control_test_assets URDFs reference test hardware plugins that are
// not registered in the Humble release, so mock_components is used instead.
constexpr char kMinimalUrdf[] = R"(<?xml version="1.0"?>
<robot name="minimal">
  <link name="base_link"/>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <ros2_control name="MockSystem" type="system">
    <hardware><plugin>mock_components/GenericSystem</plugin></hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>)";

}  // namespace

class LoadControllersTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    cm_ = std::make_shared<controller_manager::ControllerManager>(
      std::make_unique<hardware_interface::ResourceManager>(kMinimalUrdf),
      executor_, "test_controller_manager");
  }

  std::shared_ptr<rclcpp::Executor> executor_;
  std::shared_ptr<controller_manager::ControllerManager> cm_;
};

TEST_F(LoadControllersTest, LoadMitJointController)
{
  ASSERT_NE(
    cm_->load_controller("test_mit_joint_controller", "rebot_controllers/MitJointController"),
    nullptr);
}

TEST_F(LoadControllersTest, LoadGravityCompensationController)
{
  ASSERT_NE(
    cm_->load_controller(
      "test_gravity_compensation_controller",
      "rebot_controllers/GravityCompensationController"),
    nullptr);
}

TEST_F(LoadControllersTest, LoadMitTrajectoryController)
{
  ASSERT_NE(
    cm_->load_controller(
      "test_mit_trajectory_controller", "rebot_controllers/MitTrajectoryController"),
    nullptr);
}

TEST_F(LoadControllersTest, LoadJointImpedanceController)
{
  ASSERT_NE(
    cm_->load_controller(
      "test_joint_impedance_controller", "rebot_controllers/JointImpedanceController"),
    nullptr);
}

TEST_F(LoadControllersTest, LoadTeleopStreamController)
{
  ASSERT_NE(
    cm_->load_controller(
      "test_teleop_stream_controller", "rebot_controllers/TeleopStreamController"),
    nullptr);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
