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

#ifndef REBOT_CONTROLLERS__RATE_LIMITER_HPP_
#define REBOT_CONTROLLERS__RATE_LIMITER_HPP_

#include <algorithm>
#include <cmath>

namespace rebot_controllers
{

/// Velocity- and acceleration-bounded online smoothing towards a target
/// (per joint). Second-order limiter: the commanded velocity approaches the
/// deceleration-safe profile sqrt(2*a_max*|err|) so the position converges
/// without overshoot. Header-only, allocation-free, unit-testable.
struct RateLimiter
{
  double velocity{0.0};

  void reset(double v = 0.0) {velocity = v;}

  /// Advance `position` one step of `dt` towards `target`.
  /// Returns the new position; `velocity` is updated in place. The output
  /// is exactly the integral of `velocity`, so the velocity and
  /// acceleration bounds hold for the emitted position stream as well.
  double step(double position, double target, double v_max, double a_max, double dt)
  {
    const double err = target - position;
    // Fastest speed from which we can still stop at the target.
    const double v_stop = std::sqrt(2.0 * a_max * std::abs(err));
    // err/dt lands exactly on the target during the final approach.
    double v_des = std::clamp(err / dt, -v_stop, v_stop);
    v_des = std::clamp(v_des, -v_max, v_max);
    velocity += std::clamp(v_des - velocity, -a_max * dt, a_max * dt);
    return position + velocity * dt;
  }
};

}  // namespace rebot_controllers

#endif  // REBOT_CONTROLLERS__RATE_LIMITER_HPP_
