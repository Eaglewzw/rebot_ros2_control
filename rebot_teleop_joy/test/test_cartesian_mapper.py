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

import math

import numpy as np
import PyKDL

from rebot_teleop_joy.cartesian_mapper import CartesianMapper


DT = 0.02


def make_mapper(**kwargs):
    mapper = CartesianMapper(**kwargs)
    mapper.anchor((0.0, 0.0, 0.0), [0.3, 0.0, 0.4], PyKDL.Rotation.Identity())
    return mapper


def test_attitude_delta_maps_to_ee_rotation():
    mapper = make_mapper()
    _, rotation = mapper.step((0.2, 0.0, 0.0), 0.0, 0.0, False, False, DT)
    roll, pitch, yaw = rotation.GetRPY()
    assert abs(roll - 0.2) < 1e-9
    assert abs(pitch) < 1e-9 and abs(yaw) < 1e-9


def test_heading_relative_translation():
    mapper = make_mapper(xy_speed=1.0)
    # Handle yawed 90 deg left: stick-forward must move along world +Y.
    xy = mapper.heading_relative_xy(0.0, 1.0, math.pi / 2.0)
    assert abs(xy[0]) < 1e-9
    assert abs(xy[1] - 1.0) < 1e-9
    # Stick-right at the same yaw moves along world +X.
    xy = mapper.heading_relative_xy(1.0, 0.0, math.pi / 2.0)
    assert abs(xy[0] - 1.0) < 1e-9
    assert abs(xy[1]) < 1e-9


def test_pitch_roll_do_not_leak_into_translation():
    mapper = make_mapper(xy_speed=1.0)
    # heading uses yaw only: same result regardless of roll/pitch (they are
    # simply not part of the projection).
    flat = mapper.heading_relative_xy(0.0, 1.0, 0.3)
    assert abs(np.linalg.norm(flat) - 1.0) < 1e-9


def test_up_down_velocities():
    mapper = make_mapper(z_speed=0.1)
    position, _ = mapper.step((0.0, 0.0, 0.0), 0.0, 0.0, True, False, 1.0)
    assert abs(position[2] - 0.5) < 1e-9      # up 0.1 m in 1 s from 0.4
    position, _ = mapper.step((0.0, 0.0, 0.0), 0.0, 0.0, False, True, 1.0)
    assert abs(position[2] - 0.4) < 1e-9      # back down


def test_reanchor_attitude_keeps_target_pose():
    mapper = make_mapper()
    mapper.step((0.3, 0.1, 0.2), 0.5, 0.5, False, False, 1.0)
    position_before = mapper.target_position.copy()
    rpy_before = mapper.target_rotation.GetRPY()
    # In-hand regrip at a completely different handle attitude.
    mapper.reanchor_attitude((1.0, -0.5, 2.0))
    position, rotation = mapper.step((1.0, -0.5, 2.0), 0.0, 0.0, False, False, DT)
    assert np.allclose(position, position_before)
    assert np.allclose(rotation.GetRPY(), rpy_before, atol=1e-9)


def test_workspace_clamp():
    mapper = make_mapper(xy_speed=10.0, workspace_min=[-0.4, -0.4, 0.1],
                         workspace_max=[0.4, 0.4, 0.6])
    for _ in range(200):
        position, _ = mapper.step((0.0, 0.0, 0.0), 1.0, 1.0, True, False, 0.1)
    assert position[0] <= 0.4 + 1e-9
    assert position[1] <= 0.4 + 1e-9
    assert position[2] <= 0.6 + 1e-9
