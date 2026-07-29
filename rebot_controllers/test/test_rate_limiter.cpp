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

#include "rebot_controllers/rate_limiter.hpp"

using rebot_controllers::RateLimiter;

TEST(RateLimiter, RespectsVelocityAndAccelerationLimits)
{
  RateLimiter limiter;
  const double v_max = 1.0, a_max = 2.0, dt = 0.01;
  double pos = 0.0, prev_v = 0.0;
  for (int i = 0; i < 1000; ++i) {
    const double prev = pos;
    pos = limiter.step(pos, 5.0, v_max, a_max, dt);
    const double v = (pos - prev) / dt;
    EXPECT_LE(std::abs(v), v_max + 1e-9);
    EXPECT_LE(std::abs(v - prev_v), a_max * dt + 1e-9);
    prev_v = v;
  }
}

TEST(RateLimiter, ConvergesWithoutOvershoot)
{
  RateLimiter limiter;
  const double target = 0.5;
  double pos = 0.0;
  double max_pos = 0.0;
  for (int i = 0; i < 5000; ++i) {
    pos = limiter.step(pos, target, 2.0, 4.0, 0.005);
    max_pos = std::max(max_pos, pos);
  }
  EXPECT_NEAR(pos, target, 1e-6);
  EXPECT_LE(max_pos, target + 0.02);  // no significant overshoot
}

TEST(RateLimiter, ConvergesFromAbove)
{
  RateLimiter limiter;
  double pos = 1.0;
  for (int i = 0; i < 5000; ++i) {
    pos = limiter.step(pos, -1.0, 1.0, 2.0, 0.005);
  }
  EXPECT_NEAR(pos, -1.0, 1e-6);
}

TEST(RateLimiter, HoldsAtTarget)
{
  RateLimiter limiter;
  double pos = 0.3;
  for (int i = 0; i < 10; ++i) {
    pos = limiter.step(pos, 0.3, 1.0, 1.0, 0.01);
  }
  EXPECT_DOUBLE_EQ(pos, 0.3);
  EXPECT_DOUBLE_EQ(limiter.velocity, 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
