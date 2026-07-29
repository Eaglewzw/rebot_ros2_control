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

from rebot_teleop_joy.kinematics import DifferentialIk, chain_from_urdf


# 2-dof planar arm in the XZ plane: two unit links, joints about +Y.
PLANAR_URDF = """<?xml version="1.0"?>
<robot name="planar2">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="tool"/>
  <joint name="q1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.0" upper="3.0" effort="1" velocity="1"/>
  </joint>
  <joint name="q2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.0" upper="3.0" effort="1" velocity="1"/>
  </joint>
  <joint name="tool_joint" type="fixed">
    <parent link="link2"/><child link="tool"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
  </joint>
</robot>"""


def test_chain_construction():
    chain, names, lower, upper = chain_from_urdf(PLANAR_URDF, 'base_link', 'tool')
    assert names == ['q1', 'q2']
    assert lower == [-3.0, -3.0]
    assert upper == [3.0, 3.0]
    assert chain.getNrOfJoints() == 2


def test_fk_known_poses():
    ik = DifferentialIk(PLANAR_URDF, 'base_link', 'tool')
    # Straight along +X.
    position, _ = ik.fk([0.0, 0.0])
    assert np.allclose(position, [2.0, 0.0, 0.0], atol=1e-9)
    # First joint +90 deg about +Y folds the arm to -Z.
    position, _ = ik.fk([math.pi / 2.0, 0.0])
    assert np.allclose(position, [0.0, 0.0, -2.0], atol=1e-9)


def test_differential_ik_converges_to_reachable_target():
    ik = DifferentialIk(PLANAR_URDF, 'base_link', 'tool', max_joint_step=0.2)
    # Consistent target: the exact pose of a reachable configuration (the
    # 2-dof arm cannot satisfy independent position + orientation targets).
    q_goal = [0.5, 0.7]
    target_pos, target_rot = ik.fk(q_goal)
    q = [0.1, 0.1]
    for _ in range(400):
        q, error = ik.step(q, target_pos, target_rot, 0.02)
    position, _ = ik.fk(q)
    assert np.linalg.norm(position - target_pos) < 0.01
    assert error < 0.02


def test_joint_limits_respected():
    ik = DifferentialIk(PLANAR_URDF, 'base_link', 'tool')
    q = [2.9, 2.9]
    target_pos = np.array([-2.0, 0.0, 0.0])   # would require exceeding limits
    _, rot = ik.fk(q)
    for _ in range(300):
        q, _ = ik.step(q, target_pos, rot, 0.02)
    assert all(-3.0 - 1e-9 <= value <= 3.0 + 1e-9 for value in q)


def test_singularity_slows_down():
    ik = DifferentialIk(PLANAR_URDF, 'base_link', 'tool', singular_threshold=0.5)
    # Fully stretched arm: at the workspace boundary, radially singular.
    q = [0.0, 0.0]
    target_pos = np.array([2.5, 0.0, 0.0])    # unreachable, straight out
    _, rot = ik.fk(q)
    ik.step(q, target_pos, rot, 0.02)
    assert ik.singularity_scale < 1.0
