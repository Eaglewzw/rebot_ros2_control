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

#include <vector>

#include "rebot_controllers/trajectory.hpp"

using rebot_controllers::Trajectory;
using rebot_controllers::TrajectoryPoint;

namespace
{

Trajectory make_linear()
{
  std::vector<TrajectoryPoint> points(2);
  points[0].time_from_start = 0.0;
  points[0].positions = {0.0, 1.0};
  points[1].time_from_start = 2.0;
  points[1].positions = {2.0, -1.0};
  return Trajectory(std::move(points), 2);
}

Trajectory make_cubic()
{
  // Single segment 0..1 s from q=0,v=0 to q=1,v=0 (classic smooth step).
  std::vector<TrajectoryPoint> points(2);
  points[0].time_from_start = 0.0;
  points[0].positions = {0.0};
  points[0].velocities = {0.0};
  points[1].time_from_start = 1.0;
  points[1].positions = {1.0};
  points[1].velocities = {0.0};
  return Trajectory(std::move(points), 1);
}

}  // namespace

TEST(Trajectory, LinearInterpolationMidpointAndEndpoints)
{
  const auto traj = make_linear();
  std::vector<double> q(2), qd(2);
  size_t hint = 0;

  traj.sample(0.0, q, qd, hint);
  EXPECT_DOUBLE_EQ(q[0], 0.0);
  EXPECT_DOUBLE_EQ(q[1], 1.0);

  traj.sample(1.0, q, qd, hint);
  EXPECT_DOUBLE_EQ(q[0], 1.0);
  EXPECT_DOUBLE_EQ(q[1], 0.0);
  EXPECT_DOUBLE_EQ(qd[0], 1.0);
  EXPECT_DOUBLE_EQ(qd[1], -1.0);

  // Past the end: clamps to the final point with zero velocity.
  traj.sample(5.0, q, qd, hint);
  EXPECT_DOUBLE_EQ(q[0], 2.0);
  EXPECT_DOUBLE_EQ(qd[0], 0.0);
}

TEST(Trajectory, CubicHermiteSmoothStep)
{
  const auto traj = make_cubic();
  std::vector<double> q(1), qd(1);
  size_t hint = 0;

  // Midpoint of the 0->1 smooth step: q = 0.5, velocity maximal = 1.5.
  traj.sample(0.5, q, qd, hint);
  EXPECT_NEAR(q[0], 0.5, 1e-12);
  EXPECT_NEAR(qd[0], 1.5, 1e-12);

  // Quarter point of 3s^2 - 2s^3.
  traj.sample(0.25, q, qd, hint);
  EXPECT_NEAR(q[0], 3 * 0.0625 - 2 * 0.015625, 1e-12);

  // Velocity approaches zero at both ends.
  traj.sample(0.999, q, qd, hint);
  EXPECT_NEAR(qd[0], 0.0, 1e-2);
}

TEST(Trajectory, HintAllowsBackwardSeek)
{
  std::vector<TrajectoryPoint> points(4);
  for (int i = 0; i < 4; ++i) {
    points[i].time_from_start = i;
    points[i].positions = {static_cast<double>(i)};
  }
  const Trajectory traj(std::move(points), 1);
  std::vector<double> q(1), qd(1);
  size_t hint = 0;

  traj.sample(2.5, q, qd, hint);
  EXPECT_NEAR(q[0], 2.5, 1e-12);
  EXPECT_EQ(hint, 2u);
  // Re-sample earlier with a stale hint.
  traj.sample(0.5, q, qd, hint);
  EXPECT_NEAR(q[0], 0.5, 1e-12);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
