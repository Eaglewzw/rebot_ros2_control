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

"""
Record raw MIT trajectory data and calculate per-joint safety metrics.

The recorder is passive: it sends no motion command and writes only its CSV.
Hardware-only values come from `/dynamic_joint_states`; unavailable values are
kept as NaN rather than being fabricated.
"""

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
import sys

from action_msgs.msg import GoalStatusArray
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import DynamicJointState
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

JOINTS = [f"joint{i}" for i in range(1, 7)]
STATUS_NAMES = {
    0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING",
    4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED",
}


@dataclass
class Sample:
    """One synchronized desired, actual, and read-only diagnostics sample."""

    elapsed: float
    desired_position: list
    desired_velocity: list
    actual_position: list
    actual_velocity: list
    effort: list
    mos_temperature: list
    rotor_temperature: list
    fault_code: list
    missed_replies: list
    action_status: int


def finite(values):
    """Return the finite members of a metric series."""
    return [value for value in values if math.isfinite(value)]


def rms(values):
    """Return root mean square, or NaN if no finite values exist."""
    values = finite(values)
    if not values:
        return math.nan
    return math.sqrt(sum(value * value for value in values) / len(values))


def percentile(values, fraction):
    """Return a linearly interpolated percentile, or NaN for no values."""
    values = sorted(finite(values))
    if not values:
        return math.nan
    index = (len(values) - 1) * fraction
    low, high = math.floor(index), math.ceil(index)
    if low == high:
        return values[low]
    return values[low] + (values[high] - values[low]) * (index - low)


def peak(values):
    """Return the finite absolute peak, or NaN if no finite values exist."""
    return max((abs(value) for value in finite(values)), default=math.nan)


def derivative(times, values):
    """Estimate first derivative with adjacent timestamped samples."""
    result = []
    pairs = zip(times, times[1:], values, values[1:])
    for left_time, right_time, left, right in pairs:
        dt = right_time - left_time
        valid = dt > 0.0 and math.isfinite(left) and math.isfinite(right)
        result.append((right - left) / dt if valid else math.nan)
    return result


def detrended_noise(times, values):
    """Return drift rate, RMS residual, and residual peak-to-peak motion."""
    if len(times) < 2 or not all(math.isfinite(value) for value in values):
        return math.nan, math.nan, math.nan
    mean_t = sum(times) / len(times)
    mean_v = sum(values) / len(values)
    denominator = sum((time - mean_t) ** 2 for time in times)
    numerator = sum(
        (time - mean_t) * (value - mean_v)
        for time, value in zip(times, values)
    )
    slope = 0.0 if denominator == 0.0 else numerator / denominator
    residuals = [
        value - (mean_v + slope * (time - mean_t))
        for time, value in zip(times, values)
    ]
    return slope, rms(residuals), max(residuals) - min(residuals)


def final_stationary_window(samples, end_elapsed, settle_window):
    """Find active samples and its final constant-reference interval."""
    active = [sample for sample in samples if sample.elapsed <= end_elapsed]
    if not active:
        raise ValueError("no samples in active action window")
    target = active[-1].desired_position
    suffix = []
    for sample in reversed(active):
        changed = any(
            abs(actual - expected) > 1e-5
            for actual, expected in zip(sample.desired_position, target)
        )
        if changed:
            break
        suffix.append(sample)
    if not suffix:
        raise ValueError("final stationary reference was not observed")
    start = max(suffix[-1].elapsed, end_elapsed - settle_window)
    return active, [sample for sample in active if sample.elapsed >= start]


def settling_time(samples, joint, tolerance):
    """Return first time whose remaining samples all satisfy tolerance."""
    errors = [
        abs(sample.desired_position[joint] - sample.actual_position[joint])
        for sample in samples
    ]
    for index in range(len(samples)):
        if all(error <= tolerance for error in errors[index:]):
            return samples[index].elapsed
    return math.nan


def overshoot(samples, joint):
    """Return overshoot beyond the final target in the commanded direction."""
    start = samples[0].desired_position[joint]
    target = samples[-1].desired_position[joint]
    delta = target - start
    if abs(delta) < 1e-6:
        return 0.0
    values = [sample.actual_position[joint] for sample in samples]
    return max(0.0, max(values) - target) if delta > 0.0 else max(0.0, target - min(values))


def summarize(samples, settle_window, action_end_elapsed):
    """Return per-joint trial metrics and sampling-period quantiles."""
    active, settled = final_stationary_window(samples, action_end_elapsed, settle_window)
    times = [sample.elapsed for sample in active]
    intervals = [right - left for left, right in zip(times, times[1:])]
    result = []
    for index, joint in enumerate(JOINTS):
        errors = [
            sample.desired_position[index] - sample.actual_position[index]
            for sample in active
        ]
        settled_actual = [sample.actual_position[index] for sample in settled]
        settled_times = [sample.elapsed for sample in settled]
        endpoint_count = max(1, len(settled) // 4)
        endpoint_values = settled_actual[-endpoint_count:]
        target = settled[-1].desired_position[index]
        velocity = [sample.actual_velocity[index] for sample in active]
        acceleration = derivative(times, velocity)
        jerk = derivative(times[1:], acceleration)
        drift, jitter_rms, jitter_peak_to_peak = detrended_noise(
            settled_times, settled_actual)
        result.append({
            "joint": joint,
            "tracking_rms": rms(errors),
            "tracking_p95": percentile([abs(error) for error in errors], .95),
            "tracking_max": peak(errors),
            "endpoint_error": target - sum(endpoint_values) / len(endpoint_values),
            "overshoot": overshoot(active, index),
            "settling_time": settling_time(active, index, .01),
            "jitter_rms": jitter_rms,
            "jitter_peak_to_peak": jitter_peak_to_peak,
            "drift_rate": drift,
            "velocity_peak": peak(velocity),
            "acceleration_peak": peak(acceleration),
            "jerk_peak": peak(jerk),
            "effort_peak": peak([sample.effort[index] for sample in active]),
            "mos_temperature_peak": max(
                finite([sample.mos_temperature[index] for sample in active]),
                default=math.nan),
            "rotor_temperature_peak": max(
                finite([sample.rotor_temperature[index] for sample in active]),
                default=math.nan),
            "fault_code_peak": max(
                finite([sample.fault_code[index] for sample in active]),
                default=math.nan),
            "missed_replies_peak": max(
                finite([sample.missed_replies[index] for sample in active]),
                default=math.nan),
        })
    sampling = {
        "samples": len(active),
        "dt_p50": percentile(intervals, .50),
        "dt_p95": percentile(intervals, .95),
        "dt_p99": percentile(intervals, .99),
    }
    return result, sampling


def by_joint(names, values):
    """Return a joint-name-to-value map only for a complete ROS array."""
    return dict(zip(names, values)) if len(names) == len(values) else {}


class MitTrajectoryDiagnostics(Node):
    """Synchronize action feedback with joint and read-only hardware state."""

    def __init__(self, controller):
        super().__init__("mit_trajectory_diagnostics")
        self.reference = None
        self.capture_start = None
        self.last_feedback_time = None
        self.last_feedback_elapsed = None
        self.samples = []
        self.action_status = 0
        self.dynamic = {joint: {} for joint in JOINTS}
        prefix = f"/{controller}/follow_joint_trajectory/_action"
        self.create_subscription(
            FollowJointTrajectory.Impl.FeedbackMessage,
            prefix + "/feedback", self._feedback_cb, 10)
        self.create_subscription(
            GoalStatusArray, prefix + "/status", self._status_cb, 10)
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            DynamicJointState, "/dynamic_joint_states", self._dynamic_cb,
            qos_profile_sensor_data)
        self.get_logger().info(f"Waiting for trajectory feedback on {prefix}/feedback")

    def _feedback_cb(self, msg):
        feedback = msg.feedback
        desired_position = by_joint(
            feedback.joint_names, feedback.desired.positions)
        desired_velocity = by_joint(
            feedback.joint_names, feedback.desired.velocities)
        if not all(joint in desired_position for joint in JOINTS):
            return
        self.reference = (
            [desired_position[joint] for joint in JOINTS],
            [desired_velocity.get(joint, math.nan) for joint in JOINTS],
        )
        now = self.get_clock().now()
        self.last_feedback_time = now
        if self.capture_start is None:
            self.capture_start = now
            self.get_logger().info("Trajectory detected; recording started")
        self.last_feedback_elapsed = (
            now - self.capture_start).nanoseconds * 1e-9

    def _status_cb(self, msg):
        if msg.status_list:
            self.action_status = msg.status_list[-1].status

    def _dynamic_cb(self, msg):
        for joint, values in zip(msg.joint_names, msg.interface_values):
            if joint in self.dynamic:
                self.dynamic[joint] = dict(
                    zip(values.interface_names, values.values))

    def _joint_state_cb(self, msg):
        if self.reference is None or self.capture_start is None:
            return
        position = by_joint(msg.name, msg.position)
        velocity = by_joint(msg.name, msg.velocity)
        effort = by_joint(msg.name, msg.effort)
        valid = all(
            joint in position and math.isfinite(position[joint])
            for joint in JOINTS)
        if not valid:
            return

        def diagnostic(joint, interface):
            return self.dynamic[joint].get(interface, math.nan)

        now = self.get_clock().now()
        self.samples.append(Sample(
            elapsed=(now - self.capture_start).nanoseconds * 1e-9,
            desired_position=list(self.reference[0]),
            desired_velocity=list(self.reference[1]),
            actual_position=[position[joint] for joint in JOINTS],
            actual_velocity=[velocity.get(joint, math.nan) for joint in JOINTS],
            effort=[effort.get(joint, math.nan) for joint in JOINTS],
            mos_temperature=[
                diagnostic(joint, "mos_temperature") for joint in JOINTS],
            rotor_temperature=[
                diagnostic(joint, "rotor_temperature") for joint in JOINTS],
            fault_code=[diagnostic(joint, "fault_code") for joint in JOINTS],
            missed_replies=[
                diagnostic(joint, "missed_replies") for joint in JOINTS],
            action_status=self.action_status,
        ))


def write_csv(path, samples):
    """Write raw data with hardware diagnostic columns to a CSV."""
    output = Path(path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    header = ["time_s", "action_status", "action_status_name"]
    fields = (
        "desired_position", "actual_position", "position_error",
        "desired_velocity", "actual_velocity", "effort", "mos_temperature",
        "rotor_temperature", "fault_code", "missed_replies",
    )
    for joint in JOINTS:
        header.extend(f"{joint}_{field}" for field in fields)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for sample in samples:
            row = [
                f"{sample.elapsed:.9f}", sample.action_status,
                STATUS_NAMES.get(sample.action_status, "INVALID"),
            ]
            for index in range(len(JOINTS)):
                row.extend([
                    f"{sample.desired_position[index]:.9f}",
                    f"{sample.actual_position[index]:.9f}",
                    f"{sample.desired_position[index] - sample.actual_position[index]:.9f}",
                    f"{sample.desired_velocity[index]:.9f}",
                    f"{sample.actual_velocity[index]:.9f}",
                    f"{sample.effort[index]:.9f}",
                    f"{sample.mos_temperature[index]:.9f}",
                    f"{sample.rotor_temperature[index]:.9f}",
                    f"{sample.fault_code[index]:.9f}",
                    f"{sample.missed_replies[index]:.9f}",
                ])
            writer.writerow(row)
    return output


def print_summary(node, metrics, sampling, action_status):
    """Print a compact metric table suitable for the trial record."""
    status = STATUS_NAMES.get(action_status, "INVALID")
    node.get_logger().info(f"Action status at capture end: {status} ({action_status})")
    node.get_logger().info(
        "Sampling n={samples} dt P50/P95/P99={dt_p50:.4f}/{dt_p95:.4f}/"
        "{dt_p99:.4f} s".format(**sampling))
    node.get_logger().info(
        "joint rms p95 max endpoint overshoot settle jitter_rms jitter_pp "
        "v_peak a_peak jerk_peak effort mos rotor fault missed")
    for item in metrics:
        node.get_logger().info(
            "{joint:<7} {tracking_rms:.6f} {tracking_p95:.6f} "
            "{tracking_max:.6f} {endpoint_error:+.6f} {overshoot:.6f} "
            "{settling_time:.3f} {jitter_rms:.6f} "
            "{jitter_peak_to_peak:.6f} {velocity_peak:.6f} "
            "{acceleration_peak:.6f} {jerk_peak:.6f} {effort_peak:.6f} "
            "{mos_temperature_peak:.1f} {rotor_temperature_peak:.1f} "
            "{fault_code_peak:.0f} {missed_replies_peak:.0f}".format(**item))


def main():
    """Run the passive recorder until trajectory feedback stops."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", default="mit_trajectory_controller")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--start-timeout", type=float, default=60.0)
    parser.add_argument("--settle-window", type=float, default=2.0)
    parser.add_argument("--feedback-timeout", type=float, default=0.5)
    parser.add_argument(
        "--ready-file",
        help="Touch this unique file after subscriptions are ready for the action.")
    parser.add_argument("--output")
    args, ros_args = parser.parse_known_args()
    durations = (
        args.duration, args.start_timeout, args.settle_window,
        args.feedback_timeout)
    if min(durations) <= 0.0 or args.settle_window > args.duration:
        parser.error("durations must be positive; settle-window must not exceed duration")
    if args.output is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = f"/tmp/rebot_mit_tuning/{stamp}/trial.csv"
    rclpy.init(args=ros_args)
    node = MitTrajectoryDiagnostics(args.controller)
    result = 1
    try:
        if args.ready_file:
            ready_path = Path(args.ready_file).expanduser().resolve()
            ready_path.parent.mkdir(parents=True, exist_ok=True)
            ready_path.touch(exist_ok=False)
            node.get_logger().info(f"Recorder ready; wrote handshake file {ready_path}")
        waiting_since = node.get_clock().now()
        while rclpy.ok() and node.capture_start is None:
            rclpy.spin_once(node, timeout_sec=.1)
            waited = (node.get_clock().now() - waiting_since).nanoseconds * 1e-9
            if waited >= args.start_timeout:
                raise RuntimeError("No MIT trajectory feedback received before timeout")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
            now = node.get_clock().now()
            elapsed = (now - node.capture_start).nanoseconds * 1e-9
            silence = (now - node.last_feedback_time).nanoseconds * 1e-9
            if silence >= args.feedback_timeout:
                break
            if elapsed >= args.duration:
                raise RuntimeError(
                    "Maximum duration reached while trajectory feedback remained active")
        if not node.samples:
            raise RuntimeError("No synchronized desired/actual samples were recorded")
        output = write_csv(args.output, node.samples)
        metrics, sampling = summarize(
            node.samples, args.settle_window, node.last_feedback_elapsed)
        print_summary(node, metrics, sampling, node.action_status)
        node.get_logger().info(f"Raw samples written to {output}")
        result = 0
    except (KeyboardInterrupt, RuntimeError, ValueError) as exc:
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
