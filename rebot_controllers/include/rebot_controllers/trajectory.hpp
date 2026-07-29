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

#ifndef REBOT_CONTROLLERS__TRAJECTORY_HPP_
#define REBOT_CONTROLLERS__TRAJECTORY_HPP_

/// \file trajectory.hpp
/// \brief Preprocessed joint trajectory with realtime-safe sampling.
///
/// Interpolation follows the official joint_trajectory_controller semantics:
/// cubic Hermite between waypoints when velocities are given, linear
/// otherwise. Preprocessing (joint reordering, validation) happens in the
/// non-realtime action callback; sample() is allocation-free.

#include <vector>

namespace rebot_controllers
{

struct TrajectoryPoint
{
  double time_from_start{0.0};
  std::vector<double> positions;
  std::vector<double> velocities;  ///< empty when not provided
};

class Trajectory
{
public:
  Trajectory() = default;
  Trajectory(std::vector<TrajectoryPoint> points, size_t dof)
  : points_(std::move(points)), dof_(dof) {}

  bool empty() const {return points_.empty();}
  size_t dof() const {return dof_;}
  double duration() const {return points_.empty() ? 0.0 : points_.back().time_from_start;}
  const std::vector<TrajectoryPoint> & points() const {return points_;}

  /// Sample positions/velocities at `t` seconds from start into the given
  /// buffers (size dof). Clamps to the endpoints outside the time range.
  /// `hint` caches the last segment index for O(1) forward playback.
  void sample(
    double t, std::vector<double> & positions, std::vector<double> & velocities,
    size_t & hint) const
  {
    if (points_.empty()) {return;}
    if (t <= points_.front().time_from_start) {
      copy_point(points_.front(), positions, velocities, true);
      return;
    }
    if (t >= points_.back().time_from_start) {
      copy_point(points_.back(), positions, velocities, true);
      return;
    }
    if (hint >= points_.size() - 1) {hint = 0;}
    while (points_[hint + 1].time_from_start < t) {++hint;}
    while (hint > 0 && points_[hint].time_from_start > t) {--hint;}

    const TrajectoryPoint & p0 = points_[hint];
    const TrajectoryPoint & p1 = points_[hint + 1];
    const double dt = p1.time_from_start - p0.time_from_start;
    const double s = (t - p0.time_from_start) / dt;
    const bool cubic = !p0.velocities.empty() && !p1.velocities.empty();

    for (size_t i = 0; i < dof_; ++i) {
      const double q0 = p0.positions[i];
      const double q1 = p1.positions[i];
      if (cubic) {
        // Cubic Hermite basis.
        const double v0 = p0.velocities[i];
        const double v1 = p1.velocities[i];
        const double s2 = s * s;
        const double s3 = s2 * s;
        positions[i] = (2 * s3 - 3 * s2 + 1) * q0 + (s3 - 2 * s2 + s) * dt * v0 +
          (-2 * s3 + 3 * s2) * q1 + (s3 - s2) * dt * v1;
        velocities[i] = ((6 * s2 - 6 * s) * q0 + (3 * s2 - 4 * s + 1) * dt * v0 +
          (-6 * s2 + 6 * s) * q1 + (3 * s2 - 2 * s) * dt * v1) / dt;
      } else {
        positions[i] = q0 + s * (q1 - q0);
        velocities[i] = (q1 - q0) / dt;
      }
    }
  }

private:
  static void copy_point(
    const TrajectoryPoint & p, std::vector<double> & positions, std::vector<double> & velocities,
    bool zero_velocity_at_end)
  {
    for (size_t i = 0; i < p.positions.size(); ++i) {
      positions[i] = p.positions[i];
      velocities[i] =
        (!zero_velocity_at_end && !p.velocities.empty()) ? p.velocities[i] : 0.0;
    }
  }

  std::vector<TrajectoryPoint> points_;
  size_t dof_{0};
};

}  // namespace rebot_controllers

#endif  // REBOT_CONTROLLERS__TRAJECTORY_HPP_
