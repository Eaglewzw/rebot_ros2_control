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

"""Deadman-clutch teleop state machine (mode-independent).

States
  DISCONNECTED  no Joy-Con (or dropped): zero command, hold, rescan
  CALIBRATING   IMU rest calibration running: motion commands forbidden
  IDLE          ready, deadman released: hold position
  ENGAGED       deadman held: anchored incremental tracking active
  HOMING        smooth return to the startup pose; any key interrupts

Transitions (see docs/controllers_design.md and the task spec):
  * connect            -> CALIBRATING (calibration is mandatory after every
                          (re)connection)
  * calibration done   -> IDLE
  * deadman press      -> ENGAGED, emits anchor=True (caller records the
                          attitude + arm reference at this instant)
  * deadman release    -> IDLE (commands zeroed, arm holds)
  * reanchor click     -> stays ENGAGED, emits anchor=True (in-hand regrip)
  * attitude fault     -> force release: ENGAGED -> IDLE
  * disconnect         -> DISCONNECTED from any state; reconnection requires
                          calibration AND a fresh deadman press
  * home click (IDLE)  -> HOMING; any button press in HOMING -> IDLE (hold)
  * recalib click      -> CALIBRATING, only from IDLE and only when static

Pure Python state machine — unit-tested in test/test_teleop_state.py.
"""

from enum import Enum


class State(Enum):
    DISCONNECTED = 'disconnected'
    CALIBRATING = 'calibrating'
    IDLE = 'idle'
    ENGAGED = 'engaged'
    HOMING = 'homing'


class StepResult:
    """What the mapper must do this cycle."""

    __slots__ = (
        'state', 'anchor_full', 'anchor_attitude', 'start_home',
        'start_calibration', 'gripper_click', 'gear_click')

    def __init__(self, state):
        self.state = state
        self.anchor_full = False      # deadman press: anchor attitude + arm pose
        self.anchor_attitude = False  # reanchor click: attitude reference only
        self.start_home = False
        self.start_calibration = False
        self.gripper_click = False
        self.gear_click = False


class TeleopStateMachine:

    def __init__(self):
        self.state = State.DISCONNECTED
        self._prev_buttons = {}

    def _clicked(self, buttons, key):
        """Rising edge of a button."""
        now = bool(buttons.get(key, False))
        before = bool(self._prev_buttons.get(key, False))
        return now and not before

    @staticmethod
    def _any_pressed(buttons):
        return any(bool(value) for value in buttons.values())

    def step(self, connected, calibrating, buttons, attitude_fault=False, static=False,
             home_done=False):
        """Advance one cycle. ``buttons`` is the semantic dict from
        JoyconSession. Returns a StepResult."""
        result = StepResult(self.state)

        if not connected:
            self.state = State.DISCONNECTED
            self._prev_buttons = {}
            result.state = self.state
            return result

        if self.state == State.DISCONNECTED:
            # Fresh (re)connection: calibration is mandatory.
            self.state = State.CALIBRATING
        elif self.state == State.CALIBRATING:
            if not calibrating:
                self.state = State.IDLE
        elif self.state == State.IDLE:
            if self._clicked(buttons, 'deadman') or (
                    buttons.get('deadman', False) and not self._prev_buttons):
                # Deadman pressed (edge, or held during state entry).
                self.state = State.ENGAGED
                result.anchor_full = True
            elif self._clicked(buttons, 'home'):
                self.state = State.HOMING
                result.start_home = True
            elif self._clicked(buttons, 'recalib') and static:
                self.state = State.CALIBRATING
                result.start_calibration = True
        elif self.state == State.ENGAGED:
            if attitude_fault:
                # Spike overrun: force-release the deadman semantics.
                self.state = State.IDLE
            elif not buttons.get('deadman', False):
                self.state = State.IDLE
            elif self._clicked(buttons, 'reanchor'):
                result.anchor_attitude = True
        elif self.state == State.HOMING:
            if self._any_pressed(buttons):
                # Any key interrupts homing and holds the current position.
                self.state = State.IDLE
            elif home_done:
                self.state = State.IDLE

        # Clicks valid while calibrated (IDLE and ENGAGED).
        if self.state in (State.IDLE, State.ENGAGED):
            result.gripper_click = self._clicked(buttons, 'gripper')
            result.gear_click = self._clicked(buttons, 'gear')

        self._prev_buttons = dict(buttons)
        result.state = self.state
        return result

    def motion_allowed(self):
        return self.state == State.ENGAGED

    def gripper_allowed(self):
        """Gripper toggle is allowed whenever calibrated and connected."""
        return self.state in (State.IDLE, State.ENGAGED)
