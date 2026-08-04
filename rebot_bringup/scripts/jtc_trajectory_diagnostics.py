#!/usr/bin/env python3
# Copyright 2026 Eaglewzw
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

"""Record standard JTC action feedback and joint states for tuning trials."""

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
import sys

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


JOINTS = [f"joint{i}" for i in range(1, 7)]


@dataclass
class Sample:
    elapsed: float
    desired_position: list
    desired_velocity: list
    actual_position: list
    actual_velocity: list
    effort: list


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else math.nan


def peak(values):
    return max((abs(value) for value in values), default=math.nan)


def by_joint(names, values):
    return dict(zip(names, values)) if len(names) == len(values) else {}


class JtcDiagnostics(Node):
    """Synchronize JTC desired feedback with hardware joint-state feedback."""

    def __init__(self, controller):
        super().__init__("jtc_trajectory_diagnostics")
        self.reference = None
        self.capture_start = None
        self.last_feedback_time = None
        self.last_feedback_elapsed = None
        self.samples = []
        feedback_topic = f"/{controller}/follow_joint_trajectory/_action/feedback"
        self.create_subscription(
            FollowJointTrajectory.Impl.FeedbackMessage, feedback_topic, self._feedback_cb, 10)
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_cb,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f"Waiting for trajectory feedback on {feedback_topic}")

    def _feedback_cb(self, msg):
        feedback = msg.feedback
        desired_position = by_joint(feedback.joint_names, feedback.desired.positions)
        desired_velocity = by_joint(feedback.joint_names, feedback.desired.velocities)
        actual_position = by_joint(feedback.joint_names, feedback.actual.positions)
        if not all(joint in desired_position for joint in JOINTS):
            return
        self.reference = (
            [desired_position[joint] for joint in JOINTS],
            [desired_velocity.get(joint, math.nan) for joint in JOINTS],
            [actual_position.get(joint, math.nan) for joint in JOINTS],
        )
        now = self.get_clock().now()
        self.last_feedback_time = now
        if self.capture_start is None:
            self.capture_start = now
            self.get_logger().info("Trajectory detected; recording started")
        self.last_feedback_elapsed = (now - self.capture_start).nanoseconds * 1e-9

    def _joint_state_cb(self, msg):
        if self.reference is None or self.capture_start is None:
            return
        position = by_joint(msg.name, msg.position)
        velocity = by_joint(msg.name, msg.velocity)
        effort = by_joint(msg.name, msg.effort)
        if not all(joint in position and math.isfinite(position[joint]) for joint in JOINTS):
            return
        now = self.get_clock().now()
        self.samples.append(Sample(
            elapsed=(now - self.capture_start).nanoseconds * 1e-9,
            desired_position=list(self.reference[0]),
            desired_velocity=list(self.reference[1]),
            actual_position=[position[joint] for joint in JOINTS],
            actual_velocity=[velocity.get(joint, math.nan) for joint in JOINTS],
            effort=[effort.get(joint, math.nan) for joint in JOINTS],
        ))


def write_csv(path, samples):
    output = Path(path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    header = ["time_s"]
    for joint in JOINTS:
        header += [
            f"{joint}_desired_position", f"{joint}_actual_position", f"{joint}_position_error",
            f"{joint}_desired_velocity", f"{joint}_actual_velocity", f"{joint}_effort",
        ]
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for sample in samples:
            row = [f"{sample.elapsed:.9f}"]
            for index in range(len(JOINTS)):
                row += [
                    f"{sample.desired_position[index]:.9f}",
                    f"{sample.actual_position[index]:.9f}",
                    f"{sample.desired_position[index] - sample.actual_position[index]:.9f}",
                    f"{sample.desired_velocity[index]:.9f}",
                    f"{sample.actual_velocity[index]:.9f}",
                    f"{sample.effort[index]:.9f}",
                ]

            writer.writerow(row)
    return output


def print_summary(node, samples, mock):
    active = samples
    node.get_logger().info(f"Recorded {len(active)} synchronized samples")
    if len(active) > 1:
        intervals = [b.elapsed - a.elapsed for a, b in zip(active, active[1:])]
        node.get_logger().info(
            f"Sample period: mean={sum(intervals) / len(intervals):.4f}s "
            f"min={min(intervals):.4f}s max={max(intervals):.4f}s")
    node.get_logger().info(
        "joint   max_error  rms_error  final_error  desired_v_peak  actual_v_peak")
    for index, joint in enumerate(JOINTS):
        errors = [
            sample.desired_position[index] - sample.actual_position[index]
            for sample in active
        ]
        desired_velocity = [sample.desired_velocity[index] for sample in active]
        actual_velocity = [sample.actual_velocity[index] for sample in active]
        node.get_logger().info(
            f"{joint:<7} {peak(errors):.6f}   {rms(errors):.6f}   {errors[-1]:+.6f}    "
            f"{peak(desired_velocity):.6f}        {peak(actual_velocity):.6f}")
    if mock:
        node.get_logger().warn(
            "Mock dynamics are disabled: actual-velocity and smoothness metrics "
            "are not physical results.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", default="joint_trajectory_controller")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--start-timeout", type=float, default=60.0)
    parser.add_argument("--feedback-timeout", type=float, default=0.5)
    parser.add_argument("--mock", action="store_true", help="Mark velocity metrics as nonphysical")
    parser.add_argument("--trial", default="unnamed")
    parser.add_argument("--output")
    args, ros_args = parser.parse_known_args()
    if min(args.duration, args.start_timeout, args.feedback_timeout) <= 0.0:
        parser.error("duration and timeouts must be positive")
    if args.output is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = f"/tmp/jtc_trial_{args.trial}_{stamp}.csv"

    rclpy.init(args=ros_args)
    node = JtcDiagnostics(args.controller)
    result = 1
    try:
        waiting_since = node.get_clock().now()
        while rclpy.ok() and node.capture_start is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            waited = (node.get_clock().now() - waiting_since).nanoseconds * 1e-9
            if waited >= args.start_timeout:
                raise RuntimeError("No JTC trajectory feedback received before timeout")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = node.get_clock().now()
            elapsed = (now - node.capture_start).nanoseconds * 1e-9
            silence = (now - node.last_feedback_time).nanoseconds * 1e-9
            if silence >= args.feedback_timeout:
                break
            if elapsed >= args.duration:
                raise RuntimeError("Maximum duration reached while feedback remained active")
        if not node.samples:
            raise RuntimeError("No synchronized desired/actual samples were recorded")
        output = write_csv(args.output, node.samples)
        print_summary(node, node.samples, args.mock)
        node.get_logger().info(f"Trial '{args.trial}' CSV written to {output}")
        result = 0
    except (KeyboardInterrupt, RuntimeError, ValueError) as exc:
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
