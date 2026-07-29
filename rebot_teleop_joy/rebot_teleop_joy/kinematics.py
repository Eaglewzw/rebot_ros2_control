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

"""Minimal servo kinematics: FK + damped-least-squares differential IK.

Stands in for the module-A servo pipeline with the same behaviour contract:
pose target in, joint-limit-clamped joint step out, singularity slow-down.
Built on PyKDL + urdf_parser_py (kdl_parser_py is not shipped for Humble
binaries, so the chain is assembled manually from the URDF).

Pure Python — unit-tested in test/test_kinematics.py.
"""

import math

import numpy as np
import PyKDL
from urdf_parser_py import urdf as urdf_model


def _kdl_frame(origin):
    xyz = origin.xyz if origin and origin.xyz else [0.0, 0.0, 0.0]
    rpy = origin.rpy if origin and origin.rpy else [0.0, 0.0, 0.0]
    return PyKDL.Frame(
        PyKDL.Rotation.RPY(*rpy), PyKDL.Vector(*xyz))


def chain_from_urdf(urdf_string, base_link, tip_link):
    """Build a PyKDL chain and return (chain, joint_names, lower, upper)."""
    robot = urdf_model.Robot.from_xml_string(urdf_string)
    chain_links = robot.get_chain(base_link, tip_link, joints=True, links=False)
    chain = PyKDL.Chain()
    joint_names, lower, upper = [], [], []
    for joint_name in chain_links:
        joint = robot.joint_map[joint_name]
        frame = _kdl_frame(joint.origin)
        if joint.type in ('revolute', 'continuous'):
            axis = PyKDL.Vector(*joint.axis)
            # Joint rotation about an axis anchored at the joint origin.
            kdl_joint = PyKDL.Joint(
                joint.name, frame.p, frame.M * axis, PyKDL.Joint.RotAxis)
            chain.addSegment(PyKDL.Segment(joint.child, kdl_joint, frame))
            joint_names.append(joint.name)
            if joint.type == 'continuous' or joint.limit is None:
                lower.append(-math.pi)
                upper.append(math.pi)
            else:
                lower.append(float(joint.limit.lower))
                upper.append(float(joint.limit.upper))
        else:
            chain.addSegment(
                PyKDL.Segment(joint.child, PyKDL.Joint(joint.name), frame))
    return chain, joint_names, lower, upper


class DifferentialIk:
    """Damped-least-squares velocity IK with singularity slow-down."""

    def __init__(self, urdf_string, base_link, tip_link,
                 damping=0.05, max_joint_step=0.1,
                 singular_threshold=0.02, position_gain=4.0, rotation_gain=4.0):
        self.chain, self.joint_names, self.lower, self.upper = chain_from_urdf(
            urdf_string, base_link, tip_link)
        self.n = len(self.joint_names)
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)
        self.jac_solver = PyKDL.ChainJntToJacSolver(self.chain)
        self.damping = float(damping)
        self.max_joint_step = float(max_joint_step)
        self.singular_threshold = float(singular_threshold)
        self.position_gain = float(position_gain)
        self.rotation_gain = float(rotation_gain)
        self.singularity_scale = 1.0  # diagnostic: 1 = fine, ->0 near singular

    def _jnt_array(self, q):
        array = PyKDL.JntArray(self.n)
        for i, value in enumerate(q):
            array[i] = value
        return array

    def fk(self, q):
        """Forward kinematics -> (position ndarray[3], rotation PyKDL.Rotation)."""
        frame = PyKDL.Frame()
        self.fk_solver.JntToCart(self._jnt_array(q), frame)
        position = np.array([frame.p[0], frame.p[1], frame.p[2]])
        return position, frame.M

    def jacobian(self, q):
        jac = PyKDL.Jacobian(self.n)
        self.jac_solver.JntToJac(self._jnt_array(q), jac)
        matrix = np.zeros((6, self.n))
        for row in range(6):
            for col in range(self.n):
                matrix[row, col] = jac[row, col]
        return matrix

    @staticmethod
    def pose_error(current_pos, current_rot, target_pos, target_rot):
        """6-vector [v; w] driving current toward target."""
        linear = np.asarray(target_pos) - np.asarray(current_pos)
        rot_err = target_rot * current_rot.Inverse()
        angle_axis = rot_err.GetRot()  # PyKDL.Vector = axis * angle
        angular = np.array([angle_axis[0], angle_axis[1], angle_axis[2]])
        return np.concatenate([linear, angular])

    def step(self, q, target_pos, target_rot, dt):
        """One servo step toward the target pose. Returns (q_new, error_norm)."""
        current_pos, current_rot = self.fk(q)
        error = self.pose_error(current_pos, current_rot, target_pos, target_rot)
        twist = np.concatenate([
            error[:3] * self.position_gain, error[3:] * self.rotation_gain])

        jac = self.jacobian(q)
        # Damped least squares: qdot = J^T (J J^T + lambda^2 I)^-1 v
        jjt = jac @ jac.T
        # Singularity metric: smallest singular value of J. Taken from an SVD
        # of J rather than the eigenvalues of J J^T, which are structurally
        # zero whenever the chain has fewer than six joints.
        sigma_min = float(np.min(np.linalg.svd(jac, compute_uv=False)))
        if sigma_min < self.singular_threshold:
            # Slow down near singularities (and damp harder).
            self.singularity_scale = max(0.1, sigma_min / self.singular_threshold)
            damping = self.damping * 4.0
        else:
            self.singularity_scale = 1.0
            damping = self.damping
        qdot = jac.T @ np.linalg.solve(jjt + (damping ** 2) * np.eye(6), twist)
        qdot *= self.singularity_scale

        step = np.clip(qdot * dt, -self.max_joint_step, self.max_joint_step)
        q_new = np.clip(np.asarray(q) + step, self.lower, self.upper)
        return q_new.tolist(), float(np.linalg.norm(error))
