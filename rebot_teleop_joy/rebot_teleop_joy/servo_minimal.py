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

"""Minimal servo pipeline (drop-in until the module-A pipeline lands).

Interface (module-A compatible):
  in : /servo/joint_command  std_msgs/Float64MultiArray  joint-space channel
  in : /servo/pose_target    geometry_msgs/PoseStamped   cartesian channel
  out: /teleop_stream_controller/commands  Float64MultiArray (6 arm joints)
  out: /servo/ee_pose        PoseStamped  current end-effector pose (FK)
  out: ~/status              String       singularity / limit warnings

The joint channel is a validated passthrough (clamped into the soft limits).
The cartesian channel runs damped-least-squares differential IK with
singularity slow-down (see kinematics.py); final velocity/acceleration
smoothing is provided by the downstream TeleopStreamController.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

import PyKDL

from rebot_teleop_joy.kinematics import DifferentialIk

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


class ServoMinimalNode(Node):

    def __init__(self):
        super().__init__('servo_minimal')
        rate = self.declare_parameter('rate', 50.0).value
        self.base_link = self.declare_parameter('base_link', 'base_link').value
        self.tip_link = self.declare_parameter('tip_link', 'link6').value
        self.pose_timeout = self.declare_parameter('pose_timeout', 0.3).value

        self.ik = None
        self.urdf = ''
        transient = QoSProfile(depth=1)
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, '/robot_description', self._robot_description, transient)

        self.command_pub = self.create_publisher(
            Float64MultiArray, '/teleop_stream_controller/commands', 10)
        self.ee_pose_pub = self.create_publisher(PoseStamped, '/servo/ee_pose', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.create_subscription(
            Float64MultiArray, '/servo/joint_command', self._joint_command, 10)
        self.create_subscription(
            PoseStamped, '/servo/pose_target', self._pose_target, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_states, 10)

        self.measured = None            # measured arm joint positions
        self.q = None                   # IK integration state
        self.target = None              # (position ndarray, PyKDL.Rotation)
        self.target_time = None
        self.last_status = ''

        self.create_timer(1.0 / float(rate), self._tick)

    def _robot_description(self, msg):
        if self.ik is not None:
            return
        try:
            self.ik = DifferentialIk(msg.data, self.base_link, self.tip_link)
            self.get_logger().info(
                f'Servo chain ready: {self.base_link} -> {self.tip_link} '
                f'({self.ik.n} joints)')
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Cannot build the servo chain: {error}')

    def _joint_states(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(j in positions for j in ARM_JOINTS):
            self.measured = [positions[j] for j in ARM_JOINTS]

    def _joint_command(self, msg):
        """Joint channel: clamp into soft limits and forward."""
        if len(msg.data) != len(ARM_JOINTS):
            return
        if self.ik is not None:
            clamped = [
                min(max(v, lo), up)
                for v, lo, up in zip(msg.data, self.ik.lower, self.ik.upper)]
        else:
            clamped = list(msg.data)
        out = Float64MultiArray()
        out.data = [float(v) for v in clamped]
        self.command_pub.publish(out)
        # A joint-channel command supersedes any cartesian target.
        self.target = None

    def _pose_target(self, msg):
        import numpy as np
        position = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q = msg.pose.orientation
        rotation = PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w)
        fresh_stream = self.target is None
        self.target = (position, rotation)
        self.target_time = self.get_clock().now()
        if fresh_stream and self.measured is not None:
            # Seed the IK state from the measured joints at stream start.
            self.q = list(self.measured)

    def _status(self, text):
        if text and text != self.last_status:
            self.get_logger().warning(text)
        self.last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _tick(self):
        if self.ik is None or self.measured is None:
            return

        # Publish the current end-effector pose for the teleop anchor.
        position, rotation = self.ik.fk(self.measured)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.base_link
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            float(position[0]), float(position[1]), float(position[2]))
        qx, qy, qz, qw = rotation.GetQuaternion()
        pose.pose.orientation.x, pose.pose.orientation.y = float(qx), float(qy)
        pose.pose.orientation.z, pose.pose.orientation.w = float(qz), float(qw)
        self.ee_pose_pub.publish(pose)

        if self.target is None:
            return
        age = (self.get_clock().now() - self.target_time).nanoseconds * 1e-9
        if age > self.pose_timeout:
            # Stale pose stream: stop stepping; the stream controller holds.
            self.target = None
            self._status('')
            return

        if self.q is None:
            self.q = list(self.measured)
        target_pos, target_rot = self.target
        self.q, error = self.ik.step(self.q, target_pos, target_rot, 0.02)

        out = Float64MultiArray()
        out.data = [float(v) for v in self.q]
        self.command_pub.publish(out)

        if self.ik.singularity_scale < 1.0:
            self._status(
                f'SINGULARITY: slowing down (scale {self.ik.singularity_scale:.2f})')
        elif error > 0.15:
            self._status(f'TRACKING: pose error {error:.2f} (limits or reach)')
        else:
            self._status('')


def main(args=None):
    rclpy.init(args=args)
    node = ServoMinimalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
