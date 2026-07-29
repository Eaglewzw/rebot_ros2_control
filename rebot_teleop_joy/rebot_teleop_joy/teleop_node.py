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

"""Single Joy-Con teleoperation node for the reBot Arm B601-DM.

Mode is fixed at launch (``teleop_mode``: ``joint`` or ``cartesian``) and the
deadman-clutch semantics are implemented by TeleopStateMachine.  Motion is
only ever published while ENGAGED or HOMING; in every other state nothing is
published and the downstream TeleopStreamController's command timeout holds
the arm (freeze-by-silence safety chain).

Outputs (module-A compatible servo interface, served by servo_minimal):
  joint mode:     std_msgs/Float64MultiArray -> /servo/joint_command
  cartesian mode: geometry_msgs/PoseStamped  -> /servo/pose_target
  always:         std_msgs/String            -> ~/status
"""

import math

import numpy as np
import PyKDL
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from rebot_teleop_joy.attitude_gate import AttitudeGate
from rebot_teleop_joy.cartesian_mapper import CartesianMapper
from rebot_teleop_joy.joint_mapper import Gear, JointMapper
from rebot_teleop_joy.joycon_session import JoyconSession
from rebot_teleop_joy.teleop_state import State, TeleopStateMachine

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
# Soft limits from the official DM model (see rebot_b601_dm.urdf).
LIMITS_LOWER = [-2.8, -3.14, -3.14, -1.87, -1.57, -3.14]
LIMITS_UPPER = [2.8, 0.0, 0.0, 1.57, 1.57, 3.14]
MAX_VELOCITIES = [1.5, 1.5, 1.5, 2.0, 2.0, 2.0]

GRIPPER_OPEN = 0.05
GRIPPER_CLOSED = 0.0


class TeleopJoyNode(Node):

    def __init__(self):
        super().__init__('rebot_teleop_joy')
        self.mode = self.declare_parameter('teleop_mode', 'joint').value
        if self.mode not in ('joint', 'cartesian'):
            raise ValueError("teleop_mode must be 'joint' or 'cartesian'")
        rate = self.declare_parameter('rate', 50.0).value
        input_timeout = self.declare_parameter('input_timeout', 0.5).value
        cutoff_hz = self.declare_parameter('attitude_cutoff_hz', 10.0).value
        max_rate = self.declare_parameter('attitude_max_rate', 25.0).value
        spike_limit = self.declare_parameter('attitude_spike_limit', 5).value
        stick_center = self.declare_parameter('stick_center', 2048.0).value
        stick_half_range = self.declare_parameter('stick_half_range', 1400.0).value
        stick_h_sign = self.declare_parameter('stick_horizontal_sign', 1.0).value
        stick_v_sign = self.declare_parameter('stick_vertical_sign', 1.0).value

        gear = Gear(
            tuple(self.declare_parameter(
                'gear_incremental_scales', [0.5, 1.0, 1.5]).value),
            tuple(self.declare_parameter(
                'gear_velocity_scales', [0.3, 0.6, 1.0]).value))
        self.gear = gear

        self.joint_mapper = JointMapper(
            LIMITS_LOWER, LIMITS_UPPER, MAX_VELOCITIES,
            roll_scale=self.declare_parameter('roll_scale', 1.0).value,
            pitch_scale=self.declare_parameter('pitch_scale', 1.0).value,
            yaw_deadband=math.radians(
                self.declare_parameter('yaw_deadband_deg', 8.0).value),
            yaw_saturation=math.radians(
                self.declare_parameter('yaw_saturation_deg', 45.0).value),
            yaw_max_velocity=self.declare_parameter('yaw_max_velocity', 0.8).value,
            limit_margin=math.radians(
                self.declare_parameter('limit_margin_deg', 5.0).value),
            gear=gear)
        self.cartesian_mapper = CartesianMapper(
            xy_speed=self.declare_parameter('xy_speed', 0.10).value,
            z_speed=self.declare_parameter('z_speed', 0.06).value,
            rotation_scale=self.declare_parameter('rotation_scale', 1.0).value,
            gear=gear)

        self.session = JoyconSession(
            input_timeout=input_timeout, stick_center=stick_center,
            stick_half_range=stick_half_range, stick_horizontal_sign=stick_h_sign,
            stick_vertical_sign=stick_v_sign,
            info=lambda m: self.get_logger().info(m),
            warning=lambda m: self.get_logger().warning(m))
        self.gate = AttitudeGate(
            cutoff_hz=cutoff_hz, max_rate=max_rate, spike_limit=int(spike_limit))
        self.machine = TeleopStateMachine()

        self.joint_pub = self.create_publisher(
            Float64MultiArray, '/servo/joint_command', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/servo/pose_target', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd')
        self.create_subscription(JointState, '/joint_states', self._joint_states, 10)
        self.create_subscription(
            PoseStamped, '/servo/ee_pose', self._ee_pose, 10)

        self.joint_positions = None       # dict name -> position
        self.home_positions = None        # captured at startup
        self.ee_position = None
        self.ee_rotation = None
        self.gripper_open = False
        self.homing_done = False
        self.last_status = ''
        self.last_rescan = self.get_clock().now()
        self.previous_time = None

        self.timer = self.create_timer(1.0 / float(rate), self._tick)
        self.get_logger().info(
            f"Teleop mode '{self.mode}'. Connect a Joy-Con (right first); "
            'keep it FLAT on the desk for ~2 s while the IMU calibrates.')

    # ------------------------------------------------------------------
    def _joint_states(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if not all(j in positions for j in ARM_JOINTS):
            return
        self.joint_positions = positions
        if self.home_positions is None:
            self.home_positions = [positions[j] for j in ARM_JOINTS]
            self.get_logger().info(
                'Home pose recorded: '
                + ', '.join(f'{q:.2f}' for q in self.home_positions))

    def _ee_pose(self, msg):
        self.ee_position = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q = msg.pose.orientation
        self.ee_rotation = PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w)

    def _arm_positions(self):
        if self.joint_positions is None:
            return None
        return [self.joint_positions[j] for j in ARM_JOINTS]

    def _status(self, text):
        if text != self.last_status:
            self.last_status = text
            self.get_logger().info(text)
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _toggle_gripper(self):
        self.gripper_open = not self.gripper_open
        goal = GripperCommand.Goal()
        goal.command.position = GRIPPER_OPEN if self.gripper_open else GRIPPER_CLOSED
        goal.command.max_effort = 5.0
        if self.gripper_client.server_is_ready():
            self.gripper_client.send_goal_async(goal)
        else:
            self.get_logger().warning('Gripper action server not ready')

    def _publish_joint_targets(self, targets):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in targets]
        self.joint_pub.publish(msg)

    def _publish_pose_target(self, position, rotation):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
            float(position[0]), float(position[1]), float(position[2]))
        qx, qy, qz, qw = rotation.GetQuaternion()
        msg.pose.orientation.x, msg.pose.orientation.y = float(qx), float(qy)
        msg.pose.orientation.z, msg.pose.orientation.w = float(qz), float(qw)
        self.pose_pub.publish(msg)

    # ------------------------------------------------------------------
    def _tick(self):
        now = self.get_clock().now()
        dt = 0.0
        if self.previous_time is not None:
            dt = (now - self.previous_time).nanoseconds * 1e-9
        self.previous_time = now

        if not self.session.connected:
            if (now - self.last_rescan).nanoseconds * 1e-9 > 1.0:
                self.last_rescan = now
                self.session.rescan()

        sample = self.session.poll()
        attitude = (0.0, 0.0, 0.0)
        fault = False
        if sample.fresh:
            attitude = self.gate.update(sample.roll, sample.pitch, sample.yaw, dt)
            fault = self.gate.fault

        result = self.machine.step(
            connected=sample.connected, calibrating=sample.calibrating,
            buttons=sample.buttons, attitude_fault=fault,
            static=self.gate.is_static(), home_done=self.homing_done)
        state = result.state

        if state == State.DISCONNECTED:
            self.gate.reset()
            self._status('DISCONNECTED: no Joy-Con; arm holding (stream timeout)')
            return
        if state == State.CALIBRATING:
            self._status('CALIBRATING: keep the Joy-Con flat and still (~2 s)...')
            return

        arm = self._arm_positions()
        if arm is None:
            self._status('WAITING: no /joint_states yet')
            return

        if result.start_calibration:
            self._status('RECALIBRATING: keep the Joy-Con flat and still (~2 s)...')
            self.session.recalibrate()
            self.gate.reset()
            return

        if result.anchor_full:
            self.joint_mapper.anchor(attitude, arm)
            if self.mode == 'cartesian':
                if self.ee_position is not None:
                    self.cartesian_mapper.anchor(
                        attitude, self.ee_position, self.ee_rotation)
                else:
                    self._status('WARN: no end-effector pose yet; cannot anchor')
        elif result.anchor_attitude:
            # X click while engaged: in-hand regrip, arm target unchanged.
            self.joint_mapper.anchor(attitude, arm)
            if self.mode == 'cartesian':
                self.cartesian_mapper.reanchor_attitude(attitude)

        if state == State.HOMING:
            if self.home_positions is None:
                self.homing_done = True
                return
            self._publish_joint_targets(self.home_positions)
            error = max(abs(a - h) for a, h in zip(arm, self.home_positions))
            self.homing_done = error < 0.03
            self._status('HOMING: returning to the startup pose '
                         '(press any button to interrupt)')
            return
        self.homing_done = False

        if state == State.IDLE:
            self._status(f'IDLE [{sample.side}] gear={self.gear.index + 1}: '
                         'hold ZR/ZL to engage')
        elif state == State.ENGAGED:
            self._status(f'ENGAGED [{self.mode}] gear={self.gear.index + 1}')

        # Clicks valid in IDLE and ENGAGED (reported by the state machine).
        if result.gripper_click:
            self._toggle_gripper()
        if result.gear_click:
            self.gear.cycle()

        if state != State.ENGAGED:
            return

        if self.mode == 'joint':
            targets = self.joint_mapper.step(
                attitude, sample.stick_horizontal, sample.stick_vertical,
                sample.buttons.get('shoulder', False),
                sample.buttons.get('stick_press', False), dt)
            self._publish_joint_targets(targets)
        else:
            if self.ee_position is None:
                self._status('WARN: waiting for /servo/ee_pose')
                return
            position, rotation = self.cartesian_mapper.step(
                attitude, sample.stick_horizontal, sample.stick_vertical,
                sample.buttons.get('shoulder', False),
                sample.buttons.get('stick_press', False), dt)
            self._publish_pose_target(position, rotation)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.session.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
