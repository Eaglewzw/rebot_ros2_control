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

"""Servo kinematics: FK + weighted damped-least-squares differential IK.

This is the fallback backend of the servo pipeline; the QP backend lives in
placo_backend.py.  Both expose the same small contract, so
`ik_solver` can switch between them without touching the node.

Tuning follows the reference implementation in Eaglewzw/JoyReBot
(joyrebot_teleop): position and orientation are weighted **asymmetrically**
(~300:1).  On a non-redundant 6-DoF arm an unreachable target forces a
trade-off, and for teleoperation the end-effector position is what the
operator is actually aiming with — orientation must yield first.  Joint
limits are pulled inward by a safety margin and the step is bounded by a real
joint speed limit rather than a bare per-cycle step clip.

Built on PyKDL + urdf_parser_py (kdl_parser_py is not shipped for Humble
binaries, so the chain is assembled manually from the URDF).
"""

from dataclasses import dataclass
import math

import numpy as np
import PyKDL
from urdf_parser_py import urdf as urdf_model


@dataclass(frozen=True)
class IKDiagnostics:
    """Outcome of one IK update, mirroring the JoyReBot diagnostics."""

    position_error: float = float('inf')
    orientation_error: float = float('inf')
    target_reached: bool = False
    at_joint_limit: bool = False
    velocity_limited: bool = False
    singularity_scale: float = 1.0
    failure: str = ''


def _kdl_frame(origin):
    xyz = origin.xyz if origin and origin.xyz else [0.0, 0.0, 0.0]
    rpy = origin.rpy if origin and origin.rpy else [0.0, 0.0, 0.0]
    return PyKDL.Frame(PyKDL.Rotation.RPY(*rpy), PyKDL.Vector(*xyz))


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
            chain.addSegment(PyKDL.Segment(joint.child, PyKDL.Joint(joint.name), frame))
    return chain, joint_names, lower, upper


def pose_error(current_pos, current_rot, target_pos, target_rot):
    """6-vector [v; w] driving the current pose toward the target."""
    linear = np.asarray(target_pos) - np.asarray(current_pos)
    angle_axis = (target_rot * current_rot.Inverse()).GetRot()
    angular = np.array([angle_axis[0], angle_axis[1], angle_axis[2]])
    return np.concatenate([linear, angular])


class DifferentialIk:
    """Weighted DLS velocity IK with singularity slow-down.

    Contract shared with PlacoChain (the IkBackend interface):
      attributes  n, joint_names, lower, upper   (limits include the margin)
      fk(q)    -> (position ndarray[3], PyKDL.Rotation)
      step(q, target_pos, target_rot, dt) -> (q_new list, IKDiagnostics)
    """

    def __init__(self, urdf_string, base_link, tip_link,
                 damping=0.05, max_joint_speed=0.7, joint_margin=0.02,
                 position_weight=100.0, orientation_weight=0.35, gain=4.0,
                 singular_threshold=0.02,
                 position_tolerance=0.004, orientation_tolerance=0.015):
        self.chain, self.joint_names, raw_lower, raw_upper = chain_from_urdf(
            urdf_string, base_link, tip_link)
        self.n = len(self.joint_names)
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)
        self.jac_solver = PyKDL.ChainJntToJacSolver(self.chain)

        # Safety margin: pull the hard limits inward so the servo never rides
        # exactly on a joint stop.
        self.joint_margin = float(joint_margin)
        if not np.isfinite(self.joint_margin) or self.joint_margin < 0.0:
            raise ValueError('joint_margin must be finite and non-negative')
        self.lower = np.asarray(raw_lower) + self.joint_margin
        self.upper = np.asarray(raw_upper) - self.joint_margin
        if np.any(self.lower >= self.upper):
            raise ValueError('joint_margin leaves one or more joints without a valid range')

        self.damping = float(damping)
        self.max_joint_speed = float(max_joint_speed)
        self.gain = float(gain)
        self.singular_threshold = float(singular_threshold)
        self.position_tolerance = float(position_tolerance)
        self.orientation_tolerance = float(orientation_tolerance)

        if not np.isfinite(self.damping) or self.damping <= 0.0:
            raise ValueError('damping must be finite and positive')
        if not np.isfinite(self.max_joint_speed) or self.max_joint_speed <= 0.0:
            raise ValueError('max_joint_speed must be finite and positive')
        if not np.isfinite(self.gain) or self.gain <= 0.0:
            raise ValueError('gain must be finite and positive')
        if self.singular_threshold <= 0.0:
            raise ValueError('singular_threshold must be positive')
        if self.position_tolerance <= 0.0 or self.orientation_tolerance <= 0.0:
            raise ValueError('IK tolerances must be positive')
        if (not np.isfinite(position_weight)
                or not np.isfinite(orientation_weight)
                or position_weight <= 0.0 or orientation_weight < 0.0):
            raise ValueError(
                'position_weight must be finite and > 0; '
                'orientation_weight must be finite and >= 0')
        # Normalized against the position weight so the damping stays on a
        # consistent scale; only the ratio carries meaning.
        ratio = float(orientation_weight) / float(position_weight)
        self.task_weights = np.array([1.0, 1.0, 1.0, ratio, ratio, ratio])

        self.singularity_scale = 1.0
        self.last_diagnostics = IKDiagnostics()

    def _jnt_array(self, q):
        array = PyKDL.JntArray(self.n)
        for i, value in enumerate(q):
            array[i] = value
        return array

    def fk(self, q):
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
        return pose_error(current_pos, current_rot, target_pos, target_rot)

    def step(self, q, target_pos, target_rot, dt):
        """One servo period toward the target pose."""
        dt = float(dt)
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError('dt must be finite and positive')
        q = np.asarray(q, dtype=float)
        if q.shape != (self.n,):
            raise ValueError(f'expected {self.n} joint values, got shape {q.shape}')
        if not np.all(np.isfinite(q)):
            safe_q = np.clip(
                np.nan_to_num(q, nan=0.0, posinf=0.0, neginf=0.0),
                self.lower, self.upper)
            self.last_diagnostics = IKDiagnostics(failure='non-finite seed')
            return safe_q.tolist(), self.last_diagnostics
        q = np.clip(q, self.lower, self.upper)
        current_pos, current_rot = self.fk(q)
        error = pose_error(current_pos, current_rot, target_pos, target_rot)
        if not np.all(np.isfinite(error)):
            self.last_diagnostics = IKDiagnostics(failure='invalid target pose')
            return q.tolist(), self.last_diagnostics

        jac = self.jacobian(q)
        # Singularity metric: smallest singular value of J. Taken from an SVD
        # of J rather than the eigenvalues of J J^T, which are structurally
        # zero whenever the chain has fewer than six joints.
        sigma_min = float(np.min(np.linalg.svd(jac, compute_uv=False)))
        if sigma_min < self.singular_threshold:
            self.singularity_scale = max(0.1, sigma_min / self.singular_threshold)
            damping = self.damping * 4.0
        else:
            self.singularity_scale = 1.0
            damping = self.damping

        # Weighted damped least squares (left form, valid for any joint count):
        #   qdot = (J^T W J + lambda^2 I)^-1 J^T W (gain * e)
        # W de-prioritizes orientation so an unreachable target sacrifices
        # orientation before position.
        weights = np.diag(self.task_weights)
        jtw = jac.T @ weights
        lhs = jtw @ jac + (damping ** 2) * np.eye(self.n)
        try:
            qdot = np.linalg.solve(lhs, jtw @ (self.gain * error))
        except np.linalg.LinAlgError as solve_error:
            self.last_diagnostics = IKDiagnostics(
                singularity_scale=self.singularity_scale,
                failure=f'DLS solve failed: {solve_error}')
            return q.tolist(), self.last_diagnostics
        qdot *= self.singularity_scale

        max_step = self.max_joint_speed * dt
        raw_step = qdot * dt
        step = np.clip(raw_step, -max_step, max_step)
        q_new = np.clip(q + step, self.lower, self.upper)

        new_pos, new_rot = self.fk(q_new)
        new_error = pose_error(new_pos, new_rot, target_pos, target_rot)
        position_error = float(np.linalg.norm(new_error[:3]))
        orientation_error = float(np.linalg.norm(new_error[3:]))
        self.last_diagnostics = IKDiagnostics(
            position_error=position_error,
            orientation_error=orientation_error,
            target_reached=(position_error <= self.position_tolerance
                            and orientation_error <= self.orientation_tolerance),
            at_joint_limit=bool(np.any(
                np.minimum(q_new - self.lower, self.upper - q_new) <= 1e-5)),
            velocity_limited=bool(np.any(np.abs(raw_step) > max_step * (1.0 - 1e-4))),
            singularity_scale=self.singularity_scale)
        return q_new.tolist(), self.last_diagnostics


def make_ik_solver(kind, urdf_string, base_link, tip_link, logger=None, **kwargs):
    """Build the requested backend; fall back to DLS when PlaCo is missing.

    Returns (backend, actual_kind).
    """
    def report(message):
        if logger is not None:
            logger(message)

    common = {
        'max_joint_speed', 'joint_margin', 'position_weight',
        'orientation_weight', 'position_tolerance', 'orientation_tolerance',
    }
    dls_only = {'damping', 'gain', 'singular_threshold'}
    placo_only = {'solve_iterations'}
    unknown = set(kwargs) - common - dls_only - placo_only
    if unknown:
        raise TypeError(f'unexpected IK options: {sorted(unknown)}')
    placo_kwargs = {
        key: value for key, value in kwargs.items()
        if key in common or key in placo_only
    }
    dls_kwargs = {
        key: value for key, value in kwargs.items()
        if key in common or key in dls_only
    }

    if kind == 'placo':
        try:
            from rebot_teleop_joy.placo_backend import PlacoChain
            return PlacoChain(
                urdf_string, base_link, tip_link, **placo_kwargs), 'placo'
        except ImportError as error:
            report(f"PlaCo backend unavailable ({error}); falling back to 'dls'. "
                   'Install it with: pip install placo')
        except Exception as error:  # noqa: BLE001 - model/QP setup problems
            report(f"PlaCo backend failed to initialize ({error}); falling back to 'dls'.")
    elif kind != 'dls':
        raise ValueError(f"unknown ik_solver '{kind}' (expected 'placo' or 'dls')")
    return DifferentialIk(
        urdf_string, base_link, tip_link, **dls_kwargs), 'dls'
