# Copyright 2026 reBot ros2_control contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Joint-space teleop mapping (mode ``joint``, no IK).

Per the task spec (4.2):
  handle roll   -> joint6, 1:1 incremental position (rate-limited)
  handle pitch  -> joint4, 1:1 incremental position (rate-limited)
  handle yaw    -> joint1, velocity (deadband +-8 deg, saturation +-45 deg)
  stick fwd/back-> joint2, velocity
  stick left/rt -> joint5, velocity
  shoulder held -> joint3, +velocity;  stick press held -> joint3, -velocity
Soft-limit margin (default 5 deg): commands toward a nearby limit decay
linearly to zero; the opposite direction stays free.

Pure Python — unit-tested in test/test_joint_mapper.py.
"""

import math

from rebot_teleop_joy.attitude_gate import wrap_angle


class Gear:
    """Speed gear (low/mid/high) shared by incremental scale and velocities."""

    def __init__(self, incremental_scales=(0.5, 1.0, 1.5),
                 velocity_scales=(0.3, 0.6, 1.0)):
        self.incremental_scales = tuple(incremental_scales)
        self.velocity_scales = tuple(velocity_scales)
        self.index = 0  # default: low

    def cycle(self):
        self.index = (self.index + 1) % len(self.incremental_scales)

    @property
    def incremental(self):
        return self.incremental_scales[self.index]

    @property
    def velocity(self):
        return self.velocity_scales[self.index]


def limit_margin_factor(position, direction, lower, upper, margin):
    """Linear decay 1 -> 0 of motion toward a nearby soft limit; motion away
    from the limit is unaffected."""
    if margin <= 0.0 or direction == 0.0:
        return 1.0
    if direction > 0.0:
        distance = upper - position
    else:
        distance = position - lower
    if distance >= margin:
        return 1.0
    return max(0.0, distance / margin)


class JointMapper:
    """Integrates the six joint targets from one anchored input stream."""

    JOINT_COUNT = 6

    def __init__(self, limits_lower, limits_upper, max_velocities,
                 roll_scale=1.0, pitch_scale=1.0,
                 yaw_deadband=math.radians(8.0), yaw_saturation=math.radians(45.0),
                 yaw_max_velocity=0.8, limit_margin=math.radians(5.0), gear=None):
        assert len(limits_lower) == self.JOINT_COUNT
        self.lower = list(limits_lower)
        self.upper = list(limits_upper)
        self.max_vel = list(max_velocities)      # rad/s, per joint (hard cap)
        self.roll_scale = float(roll_scale)
        self.pitch_scale = float(pitch_scale)
        self.yaw_deadband = float(yaw_deadband)
        self.yaw_saturation = float(yaw_saturation)
        self.yaw_max_velocity = float(yaw_max_velocity)
        self.limit_margin = float(limit_margin)
        self.gear = gear or Gear()
        self.targets = [0.0] * self.JOINT_COUNT
        self.anchor_attitude = (0.0, 0.0, 0.0)
        self.anchor_j4 = 0.0
        self.anchor_j6 = 0.0

    def sync(self, joint_positions):
        """Adopt the measured joint positions (activation / hold states)."""
        self.targets = [
            min(max(q, lo), up) for q, lo, up in zip(joint_positions, self.lower, self.upper)]

    def anchor(self, attitude, joint_positions):
        """Deadman press / re-anchor: attitude reference and the incremental
        joints' base angles are captured at this instant."""
        self.anchor_attitude = tuple(attitude)
        self.sync(joint_positions)
        self.anchor_j4 = self.targets[3]
        self.anchor_j6 = self.targets[5]

    def _clamp_step(self, joint_index, desired, dt):
        """Rate-limit a target step and clamp it into the soft limits."""
        current = self.targets[joint_index]
        step = desired - current
        max_step = self.max_vel[joint_index] * dt
        step = min(max(step, -max_step), max_step)
        factor = limit_margin_factor(
            current, step, self.lower[joint_index], self.upper[joint_index], self.limit_margin)
        new = current + step * factor
        return min(max(new, self.lower[joint_index]), self.upper[joint_index])

    def _velocity_joint(self, joint_index, velocity, dt):
        factor = limit_margin_factor(
            self.targets[joint_index], velocity, self.lower[joint_index],
            self.upper[joint_index], self.limit_margin)
        desired = self.targets[joint_index] + velocity * factor * dt
        self.targets[joint_index] = self._clamp_step(joint_index, desired, dt)

    def yaw_velocity(self, yaw_delta):
        """Deadband + saturation mapping of the yaw offset to j1 velocity."""
        magnitude = abs(yaw_delta)
        if magnitude < self.yaw_deadband:
            return 0.0
        span = self.yaw_saturation - self.yaw_deadband
        normalized = min(1.0, (magnitude - self.yaw_deadband) / max(1e-9, span))
        return math.copysign(normalized, yaw_delta) * self.yaw_max_velocity * self.gear.velocity

    def step(self, attitude, stick_horizontal, stick_vertical, shoulder_held,
             stick_pressed, dt):
        """One ENGAGED cycle. ``attitude`` = filtered (roll, pitch, yaw).
        Returns the six joint targets."""
        droll = wrap_angle(attitude[0] - self.anchor_attitude[0])
        dpitch = wrap_angle(attitude[1] - self.anchor_attitude[1])
        dyaw = wrap_angle(attitude[2] - self.anchor_attitude[2])

        # Incremental 1:1 (scaled, geared) position mapping, rate-limited.
        j6_desired = self.anchor_j6 + droll * self.roll_scale * self.gear.incremental
        j4_desired = self.anchor_j4 + dpitch * self.pitch_scale * self.gear.incremental
        self.targets[5] = self._clamp_step(5, j6_desired, dt)
        self.targets[3] = self._clamp_step(3, j4_desired, dt)

        # Velocity joints.
        self._velocity_joint(0, self.yaw_velocity(dyaw), dt)
        self._velocity_joint(
            1, stick_vertical * self.max_vel[1] * self.gear.velocity, dt)
        self._velocity_joint(
            4, stick_horizontal * self.max_vel[4] * self.gear.velocity, dt)
        j3_direction = (1.0 if shoulder_held else 0.0) - (1.0 if stick_pressed else 0.0)
        self._velocity_joint(2, j3_direction * self.max_vel[2] * self.gear.velocity, dt)

        return list(self.targets)
