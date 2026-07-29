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

"""Constrained, incremental inverse kinematics backed by PlaCo.

PlaCo is deliberately imported only when this backend is selected.  A URDF
received through ``/robot_description`` is materialized in a private temporary
directory because :class:`placo.RobotWrapper` accepts a path, not XML text.
"""

import os
import tempfile

import numpy as np
import PyKDL

from rebot_teleop_joy.kinematics import (
    IKDiagnostics,
    chain_from_urdf,
    pose_error,
)


def _rotation_matrix(rotation):
    return np.array([
        [rotation[row, column] for column in range(3)]
        for row in range(3)
    ], dtype=float)


def _kdl_rotation(matrix):
    return PyKDL.Rotation(*np.asarray(matrix, dtype=float).reshape(-1).tolist())


class PlacoChain:
    """QP IK backend with joint-position and joint-velocity constraints."""

    def __init__(self, urdf_string, base_link, tip_link,
                 max_joint_speed=0.7, joint_margin=0.02,
                 position_weight=100.0, orientation_weight=0.35,
                 position_tolerance=0.004, orientation_tolerance=0.015,
                 solve_iterations=1):
        try:
            import placo
        except ImportError as error:  # pragma: no cover - optional dependency
            raise ImportError(
                "ik_solver='placo' requires the optional 'placo' package") from error

        self.max_joint_speed = float(max_joint_speed)
        joint_margin = float(joint_margin)
        position_weight = float(position_weight)
        orientation_weight = float(orientation_weight)
        self.position_tolerance = float(position_tolerance)
        self.orientation_tolerance = float(orientation_tolerance)
        self.solve_iterations = int(solve_iterations)
        self.tip_link = str(tip_link)
        self.singularity_scale = 1.0  # Contract shared with DifferentialIk.

        if not np.isfinite(self.max_joint_speed) or self.max_joint_speed <= 0.0:
            raise ValueError('max_joint_speed must be finite and positive')
        if self.solve_iterations < 1:
            raise ValueError('solve_iterations must be at least one')
        if joint_margin < 0.0 or not np.isfinite(joint_margin):
            raise ValueError('joint_margin must be finite and non-negative')
        if (position_weight <= 0.0 or orientation_weight < 0.0
                or not np.isfinite(position_weight)
                or not np.isfinite(orientation_weight)):
            raise ValueError(
                'position_weight must be finite and > 0; '
                'orientation_weight must be finite and >= 0')
        if self.position_tolerance <= 0.0 or self.orientation_tolerance <= 0.0:
            raise ValueError('IK tolerances must be positive')

        _, self.joint_names, raw_lower, raw_upper = chain_from_urdf(
            urdf_string, base_link, tip_link)
        self.n = len(self.joint_names)
        self.joint_margin = float(joint_margin)
        self.lower = np.asarray(raw_lower, dtype=float) + self.joint_margin
        self.upper = np.asarray(raw_upper, dtype=float) - self.joint_margin
        if np.any(self.lower >= self.upper):
            raise ValueError(
                'joint_margin leaves one or more joints without a valid range')

        # RobotWrapper consumes the model during construction, nevertheless
        # retain the directory for the backend lifetime in case a PlaCo version
        # resolves model resources lazily.
        self._urdf_directory = tempfile.TemporaryDirectory(prefix='rebot_placo_')
        urdf_path = os.path.join(self._urdf_directory.name, 'robot.urdf')
        with open(urdf_path, 'w', encoding='utf-8') as urdf_file:
            urdf_file.write(urdf_string)

        self.robot = placo.RobotWrapper(urdf_path)
        self.solver = placo.KinematicsSolver(self.robot)

        state_q = np.asarray(self.robot.state.q)
        if state_q.size >= self.n + 7:
            # RobotWrapper models the root as a free flyer.  Start it at the
            # identity pose, then remove those DoFs from the QP.
            if np.linalg.norm(state_q[3:7]) < 1e-12:
                self.robot.state.q[:7] = np.array(
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
            self.solver.mask_fbase(True)

        for name, lower, upper in zip(
                self.joint_names, self.lower, self.upper):
            self.robot.set_joint_limits(name, float(lower), float(upper))
            self.robot.set_velocity_limit(name, self.max_joint_speed)
        self.solver.enable_joint_limits(True)
        self.solver.enable_velocity_limits(True)

        self.robot.update_kinematics()
        self.frame_task = self.solver.add_frame_task(
            self.tip_link, self.robot.get_T_world_frame(self.tip_link))
        self.frame_task.configure(
            'ee_pose', 'soft', float(position_weight), float(orientation_weight))
        self.last_diagnostics = IKDiagnostics()

    def _validate_q(self, q):
        values = np.asarray(q, dtype=float)
        if values.shape != (self.n,):
            raise ValueError(
                f'expected {self.n} joint values, got shape {values.shape}')
        return values

    def _set_q(self, q):
        for name, value in zip(self.joint_names, self._validate_q(q)):
            self.robot.set_joint(name, float(value))
        self.robot.update_kinematics()

    def _get_q(self):
        return np.asarray([
            self.robot.get_joint(name) for name in self.joint_names
        ], dtype=float)

    def _tip_transform(self):
        return np.asarray(
            self.robot.get_T_world_frame(self.tip_link), dtype=float)

    def fk(self, q):
        self._set_q(q)
        transform = self._tip_transform()
        return transform[:3, 3].copy(), _kdl_rotation(transform[:3, :3])

    def _fail(self, safe_seed, reason):
        self._set_q(safe_seed)
        self.last_diagnostics = IKDiagnostics(failure=str(reason))
        return safe_seed.tolist(), self.last_diagnostics

    def step(self, q, target_pos, target_rot, dt):
        """Advance the QP by one controller period toward the target pose."""
        dt = float(dt)
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError('dt must be finite and positive')

        seed = self._validate_q(q)
        if not np.all(np.isfinite(seed)):
            safe_seed = np.clip(
                np.nan_to_num(seed, nan=0.0, posinf=0.0, neginf=0.0),
                self.lower, self.upper)
            return self._fail(safe_seed, 'non-finite seed')
        safe_seed = np.clip(seed, self.lower, self.upper)

        target = np.eye(4)
        target[:3, 3] = np.asarray(target_pos, dtype=float)
        target[:3, :3] = _rotation_matrix(target_rot)
        if not np.all(np.isfinite(target)):
            return self._fail(safe_seed, 'invalid target pose')

        self._set_q(safe_seed)
        self.frame_task.T_world_frame = target
        self.solver.dt = dt / self.solve_iterations
        try:
            for _ in range(self.solve_iterations):
                self.solver.solve(True)
                self.robot.update_kinematics()
        except RuntimeError as error:
            return self._fail(safe_seed, f'QP solve failed: {error}')

        result = self._get_q()
        if not np.all(np.isfinite(result)):
            return self._fail(safe_seed, 'QP returned non-finite joints')

        # Trust boundary: independently verify the native solver result before
        # allowing it to reach a ROS command topic.
        epsilon = 1e-7
        if (np.any(result < self.lower - epsilon)
                or np.any(result > self.upper + epsilon)):
            return self._fail(safe_seed, 'QP violated a joint limit')
        max_step = self.max_joint_speed * dt
        delta = result - safe_seed
        if np.any(np.abs(delta) > max_step + epsilon):
            return self._fail(safe_seed, 'QP violated the velocity limit')

        result = np.clip(result, self.lower, self.upper)
        self._set_q(result)
        position, rotation = self.fk(result)
        error = pose_error(position, rotation, target_pos, target_rot)
        position_error = float(np.linalg.norm(error[:3]))
        orientation_error = float(np.linalg.norm(error[3:]))
        self.last_diagnostics = IKDiagnostics(
            position_error=position_error,
            orientation_error=orientation_error,
            target_reached=(position_error <= self.position_tolerance
                            and orientation_error <= self.orientation_tolerance),
            at_joint_limit=bool(np.any(
                np.minimum(result - self.lower, self.upper - result) <= 1e-5)),
            velocity_limited=bool(np.any(
                np.abs(delta) >= max_step * (1.0 - 1e-4))))
        return result.tolist(), self.last_diagnostics
