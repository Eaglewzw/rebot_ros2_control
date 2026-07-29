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

"""State-transition tests for the deadman-clutch teleop state machine."""

from rebot_teleop_joy.teleop_state import State, TeleopStateMachine


def released():
    return {key: False for key in (
        'deadman', 'shoulder', 'stick_press', 'gripper', 'reanchor', 'gear',
        'recalib', 'home')}


def step(machine, **overrides):
    buttons = released()
    buttons.update(overrides.pop('buttons', {}))
    defaults = dict(connected=True, calibrating=False, attitude_fault=False,
                    static=False, home_done=False)
    defaults.update(overrides)
    return machine.step(buttons=buttons, **defaults)


def make_idle():
    machine = TeleopStateMachine()
    step(machine, calibrating=True)   # DISCONNECTED -> CALIBRATING
    step(machine, calibrating=True)
    step(machine)                     # calibration done -> IDLE
    assert machine.state == State.IDLE
    return machine


def test_connection_forces_calibration():
    machine = TeleopStateMachine()
    assert machine.state == State.DISCONNECTED
    result = step(machine, calibrating=True)
    assert result.state == State.CALIBRATING
    # Motion is never allowed while calibrating, deadman or not.
    result = step(machine, calibrating=True, buttons={'deadman': True})
    assert result.state == State.CALIBRATING
    assert not machine.motion_allowed()


def test_deadman_engage_anchors_and_release_freezes():
    machine = make_idle()
    result = step(machine, buttons={'deadman': True})
    assert result.state == State.ENGAGED
    assert result.anchor_full
    assert machine.motion_allowed()
    # Holding: no re-anchor.
    result = step(machine, buttons={'deadman': True})
    assert not result.anchor_full and not result.anchor_attitude
    # Release: freeze.
    result = step(machine)
    assert result.state == State.IDLE
    assert not machine.motion_allowed()
    # Re-press: fresh anchor (clutch semantics).
    result = step(machine, buttons={'deadman': True})
    assert result.state == State.ENGAGED
    assert result.anchor_full


def test_reanchor_click_while_engaged():
    machine = make_idle()
    step(machine, buttons={'deadman': True})
    result = step(machine, buttons={'deadman': True, 'reanchor': True})
    assert result.state == State.ENGAGED
    assert result.anchor_attitude and not result.anchor_full


def test_attitude_fault_force_releases():
    machine = make_idle()
    step(machine, buttons={'deadman': True})
    result = step(machine, buttons={'deadman': True}, attitude_fault=True)
    assert result.state == State.IDLE
    # Still held: does NOT re-engage on level (needs a fresh edge).
    result = step(machine, buttons={'deadman': True})
    assert result.state == State.IDLE
    # Release then press again re-engages.
    step(machine)
    result = step(machine, buttons={'deadman': True})
    assert result.state == State.ENGAGED


def test_disconnect_from_any_state_requires_recalibration():
    machine = make_idle()
    step(machine, buttons={'deadman': True})
    result = step(machine, connected=False)
    assert result.state == State.DISCONNECTED
    # Reconnection goes through CALIBRATING again.
    result = step(machine, calibrating=True)
    assert result.state == State.CALIBRATING
    result = step(machine)
    assert result.state == State.IDLE


def test_recalibration_only_when_idle_and_static():
    machine = make_idle()
    result = step(machine, buttons={'recalib': True}, static=False)
    assert result.state == State.IDLE and not result.start_calibration
    step(machine)  # release
    result = step(machine, buttons={'recalib': True}, static=True)
    assert result.state == State.CALIBRATING
    assert result.start_calibration


def test_homing_interrupted_by_any_key():
    machine = make_idle()
    result = step(machine, buttons={'home': True})
    assert result.state == State.HOMING
    assert result.start_home
    step(machine)  # home released, still homing
    assert machine.state == State.HOMING
    result = step(machine, buttons={'gear': True})
    assert result.state == State.IDLE  # interrupted, hold position


def test_homing_completes():
    machine = make_idle()
    step(machine, buttons={'home': True})
    step(machine)
    result = step(machine, home_done=True)
    assert result.state == State.IDLE


def test_clicks_only_when_calibrated():
    machine = TeleopStateMachine()
    result = step(machine, calibrating=True, buttons={'gripper': True})
    assert not result.gripper_click
    machine = make_idle()
    result = step(machine, buttons={'gripper': True, 'gear': True})
    assert result.gripper_click and result.gear_click
    # Held, not clicked again.
    result = step(machine, buttons={'gripper': True, 'gear': True})
    assert not result.gripper_click and not result.gear_click
