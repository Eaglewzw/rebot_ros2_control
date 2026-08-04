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

"""Send one conservative relative joint step to the MIT trajectory controller."""

import argparse
import math
from pathlib import Path
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS = [f"joint{i}" for i in range(1, 7)]
LIMITS = {
    "joint1": (-2.8, 2.8),
    "joint2": (-3.14, 0.0),
    "joint3": (-3.14, 0.0),
    "joint4": (-1.87, 1.57),
    "joint5": (-1.57, 1.57),
    "joint6": (-3.14, 3.14),
}


def duration(seconds):
    """Convert floating-point seconds to a builtin_interfaces Duration."""
    from builtin_interfaces.msg import Duration

    whole = int(seconds)
    return Duration(sec=whole, nanosec=int((seconds - whole) * 1e9))


class MitTuningStep(Node):
    """Read the current state and send a bounded, single-joint action goal."""

    def __init__(self, joint, delta, move_duration, cancel_after=None, ready_file=None):
        super().__init__("mit_tuning_step")
        self.joint = joint
        self.delta = delta
        self.move_duration = move_duration
        self.cancel_after = cancel_after
        self.ready_file = ready_file
        self.positions = None
        self.goal_handle = None
        self.create_subscription(JointState, "/joint_states", self._state_cb, 10)
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/mit_trajectory_controller/follow_joint_trajectory",
        )

    def _state_cb(self, msg):
        by_name = dict(zip(msg.name, msg.position))
        if all(name in by_name and math.isfinite(by_name[name]) for name in JOINTS):
            self.positions = [by_name[name] for name in JOINTS]

    def run(self):
        if self.ready_file:
            ready_path = Path(self.ready_file).expanduser().resolve()
            deadline = time.monotonic() + 10.0
            while not ready_path.exists() and time.monotonic() < deadline:
                time.sleep(0.05)
            if not ready_path.exists():
                raise RuntimeError(f"Recorder handshake file was not created: {ready_path}")
            self.get_logger().info(f"Recorder handshake confirmed: {ready_path}")
        self.get_logger().info("Waiting for a complete /joint_states sample...")
        deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=5.0)
        while rclpy.ok() and self.positions is None and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.positions is None:
            raise RuntimeError("No complete joint state received within 5 seconds")

        target = list(self.positions)
        index = JOINTS.index(self.joint)
        requested = target[index] + self.delta
        lower, upper = LIMITS[self.joint]
        if requested < lower or requested > upper:
            raise RuntimeError(
                f"Target {requested:.4f} rad is outside {self.joint} limits "
                f"[{lower:.4f}, {upper:.4f}]"
            )
        target[index] = requested

        if not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("MIT trajectory action server is unavailable")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        start = JointTrajectoryPoint()
        start.positions = list(self.positions)
        start.velocities = [0.0] * len(JOINTS)
        start.time_from_start = duration(0.5)
        finish = JointTrajectoryPoint()
        finish.positions = target
        finish.velocities = [0.0] * len(JOINTS)
        finish.time_from_start = duration(self.move_duration)
        goal.trajectory.points = [start, finish]

        self.get_logger().info(
            f"Sending {self.joint}: {self.positions[index]:.4f} -> "
            f"{target[index]:.4f} rad over {self.move_duration:.1f} s"
        )
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        self.goal_handle = send_future.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            raise RuntimeError("MIT trajectory goal was rejected")

        result_future = self.goal_handle.get_result_async()
        if self.cancel_after is not None:
            cancel_deadline = self.get_clock().now() + rclpy.duration.Duration(
                seconds=self.cancel_after)
            while (
                rclpy.ok() and not result_future.done()
                and self.get_clock().now() < cancel_deadline
            ):
                rclpy.spin_once(self, timeout_sec=0.05)
            if not result_future.done():
                self.get_logger().warn(
                    "Canceling active MIT trajectory for the requested safety check")
                self.cancel()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError("No action result received")
        result = wrapped.result
        actual = self.positions[index]
        error = target[index] - actual
        self.get_logger().info(
            f"Action finished: status={wrapped.status}, error_code={result.error_code}, "
            f"error='{result.error_string}'"
        )
        self.get_logger().info(
            f"Final {self.joint}: target={target[index]:.4f}, actual={actual:.4f}, "
            f"position_error={error:+.4f} rad"
        )
        return 0 if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL else 1

    def cancel(self):
        """Best-effort cancellation used when the operator presses Ctrl-C."""
        if self.goal_handle is None or not self.goal_handle.accepted:
            return
        future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("joint", choices=JOINTS)
    parser.add_argument(
        "delta",
        type=float,
        help="Relative step in radians. Default max is 0.1 rad; use --max-delta to override.",
    )
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument(
        "--max-delta", type=float, default=0.1,
        help="Maximum allowed absolute delta (default 0.1 rad).")
    parser.add_argument(
        "--cancel-after", type=float,
        help="Cancel after this many seconds; for cancel/hold verification only.")
    parser.add_argument(
        "--ready-file",
        help="Wait for this recorder handshake file before sending the action.")
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.delta) or abs(args.delta) > args.max_delta:
        parser.error(f"delta must be finite and within [{-args.max_delta}, {args.max_delta}] rad")
    if not math.isfinite(args.duration) or args.duration < 2.0:
        parser.error("duration must be at least 2.0 seconds")
    if args.cancel_after is not None:
        if not math.isfinite(args.cancel_after) or not 0.0 < args.cancel_after < args.duration:
            parser.error("cancel-after must be finite, positive, and shorter than duration")

    rclpy.init(args=ros_args)
    node = MitTuningStep(
        args.joint, args.delta, args.duration, args.cancel_after, args.ready_file)
    exit_code = 1
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted; canceling the active MIT trajectory")
        node.cancel()
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
