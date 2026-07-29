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

"""Cartesian teleop mapping (mode ``cartesian``).

Per the task spec (4.1):
  handle roll/pitch/yaw -> incremental end-effector orientation
                           (relative to the anchor, scale parameterized)
  stick                 -> horizontal XY translation velocity,
                           heading-relative: stick-forward moves along the
                           horizontal projection of the handle's current yaw
                           (pitch/roll do NOT leak into the direction)
  stick press held      -> constant downward velocity
  shoulder held         -> constant upward velocity

Output is a target *pose stream* (not twist): the pose target is composed
deterministically from the anchor pose plus the attitude delta, which keeps
orientation increments exact under filtering (design decision documented in
the README).

Pure Python (PyKDL rotations only) — unit-tested in test/test_cartesian_mapper.py.
"""

import math

import numpy as np
import PyKDL

from rebot_teleop_joy.attitude_gate import wrap_angle


class CartesianMapper:

    def __init__(self, xy_speed=0.10, z_speed=0.06, rotation_scale=1.0,
                 gear=None, workspace_min=None, workspace_max=None):
        self.xy_speed = float(xy_speed)      # m/s at full stick, gear=1
        self.z_speed = float(z_speed)        # m/s, gear=1
        self.rotation_scale = float(rotation_scale)
        self.gear = gear
        self.workspace_min = np.array(workspace_min if workspace_min is not None
                                      else [-0.6, -0.6, 0.02])
        self.workspace_max = np.array(workspace_max if workspace_max is not None
                                      else [0.6, 0.6, 0.8])
        self.anchor_attitude = (0.0, 0.0, 0.0)
        self.anchor_position = np.zeros(3)
        self.anchor_rotation = PyKDL.Rotation.Identity()
        self.target_position = np.zeros(3)
        self.target_rotation = PyKDL.Rotation.Identity()

    def _gear_scale(self):
        return self.gear.velocity if self.gear is not None else 1.0

    def _rot_gear_scale(self):
        return self.gear.incremental if self.gear is not None else 1.0

    def anchor(self, attitude, ee_position, ee_rotation):
        """Deadman press / re-anchor: record the handle attitude and the
        current end-effector pose as the tracking reference."""
        self.anchor_attitude = tuple(attitude)
        self.anchor_position = np.asarray(ee_position, dtype=float).copy()
        self.anchor_rotation = PyKDL.Rotation(ee_rotation)
        self.target_position = self.anchor_position.copy()
        self.target_rotation = PyKDL.Rotation(ee_rotation)

    def reanchor_attitude(self, attitude):
        """X click while engaged: re-reference the handle attitude only; the
        end-effector target pose stays where it is."""
        self.anchor_attitude = tuple(attitude)
        self.anchor_position = self.target_position.copy()
        self.anchor_rotation = PyKDL.Rotation(self.target_rotation)

    def heading_relative_xy(self, stick_horizontal, stick_vertical, yaw):
        """World-frame XY velocity: stick-forward follows the handle's yaw
        heading (horizontal projection), ignoring pitch/roll."""
        forward = np.array([math.cos(yaw), math.sin(yaw)])
        right = np.array([math.sin(yaw), -math.cos(yaw)])
        speed = self.xy_speed * self._gear_scale()
        xy = (forward * stick_vertical + right * stick_horizontal) * speed
        return xy

    def step(self, attitude, stick_horizontal, stick_vertical, shoulder_held,
             stick_pressed, dt):
        """One ENGAGED cycle -> (position ndarray[3], PyKDL.Rotation)."""
        droll = wrap_angle(attitude[0] - self.anchor_attitude[0]) * self.rotation_scale
        dpitch = wrap_angle(attitude[1] - self.anchor_attitude[1]) * self.rotation_scale
        dyaw = wrap_angle(attitude[2] - self.anchor_attitude[2]) * self.rotation_scale
        scale = self._rot_gear_scale()
        delta = PyKDL.Rotation.RPY(droll * scale, dpitch * scale, dyaw * scale)
        # Attitude delta applied in the world frame on top of the anchor pose.
        self.target_rotation = delta * self.anchor_rotation

        xy = self.heading_relative_xy(stick_horizontal, stick_vertical, attitude[2])
        z = 0.0
        if shoulder_held:
            z += self.z_speed * self._gear_scale()
        if stick_pressed:
            z -= self.z_speed * self._gear_scale()
        self.target_position = self.target_position + np.array([xy[0], xy[1], z]) * dt
        self.target_position = np.clip(
            self.target_position, self.workspace_min, self.workspace_max)
        return self.target_position.copy(), self.target_rotation
