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

"""ROS-independent Joy-Con acquisition (adapted from Eaglewzw/JoyReBot).

Wraps the vendored ``joyconrobotics`` hidapi driver: right Joy-Con first,
left as fallback, staleness detection, normalized semantic buttons shared by
both teleop modes.  The semantic names map physical-position-equivalent keys
between the two sides (yaml-overridable):

  deadman   ZR / ZL      (deadman + clutch, see design doc)
  shoulder  R  / L       (cartesian: up;   joint: joint3 +)
  stick_press            (cartesian: down; joint: joint3 -)
  gripper   A / Right    (toggle open-close)
  reanchor  X / Up       (re-anchor attitude reference while engaged)
  gear      B / Down     (speed gear cycle low/mid/high)
  recalib   Plus / Minus (IMU recalibration when idle & static)
  home      Home / Capture (smooth return to startup pose)
"""

from dataclasses import dataclass, field
import time


DEFAULT_SIDE_BINDINGS = {
    'right': {
        'deadman': 'get_button_zr',
        'shoulder': 'get_button_r',
        'stick_press': 'get_button_r_stick',
        'gripper': 'get_button_a',
        'reanchor': 'get_button_x',
        'gear': 'get_button_b',
        'recalib': 'get_button_plus',
        'home': 'get_button_home',
        'stick': ('get_stick_right_horizontal', 'get_stick_right_vertical'),
    },
    'left': {
        'deadman': 'get_button_zl',
        'shoulder': 'get_button_l',
        'stick_press': 'get_button_l_stick',
        'gripper': 'get_button_right',
        'reanchor': 'get_button_up',
        'gear': 'get_button_down',
        'recalib': 'get_button_minus',
        'home': 'get_button_capture',
        'stick': ('get_stick_left_horizontal', 'get_stick_left_vertical'),
    },
}
BUTTON_KEYS = (
    'deadman', 'shoulder', 'stick_press', 'gripper', 'reanchor', 'gear', 'recalib', 'home')


def report_is_ready(report):
    """True once the driver received a real input report (not the zeroed
    startup buffer, whose zero stick counts decode as full deflection)."""
    return bool(report) and bool(report[0])


def normalize_axis(raw, center, half_range):
    """Normalize a 12-bit Joy-Con stick count into [-1, 1]."""
    value = (float(raw) - float(center)) / max(1.0, float(half_range))
    return max(-1.0, min(1.0, value))


@dataclass(frozen=True)
class JoySample:
    """One normalized, side-agnostic input frame."""

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    stick_horizontal: float = 0.0
    stick_vertical: float = 0.0
    buttons: dict = field(default_factory=dict)
    fresh: bool = False
    connected: bool = False
    calibrating: bool = False
    side: str = ''
    connection_generation: int = 0


class JoyconSession:
    """Owns one Joy-Con and yields validated, normalized samples."""

    def __init__(self, input_timeout=0.5, stick_center=2048.0, stick_half_range=1400.0,
                 stick_horizontal_sign=1.0, stick_vertical_sign=1.0,
                 side_bindings=None, clock=time.monotonic, controller_factory=None,
                 info=lambda message: None, warning=lambda message: None):
        self.input_timeout = float(input_timeout)
        self.stick_center = float(stick_center)
        self.stick_half_range = float(stick_half_range)
        self.stick_horizontal_sign = float(stick_horizontal_sign)
        self.stick_vertical_sign = float(stick_vertical_sign)
        self.side_bindings = side_bindings or DEFAULT_SIDE_BINDINGS
        self.clock = clock
        if controller_factory is None:
            from rebot_teleop_joy.vendor.joyconrobotics import JoyconRobotics
            controller_factory = JoyconRobotics
        self.controller_factory = controller_factory
        self.info = info
        self.warning = warning
        self.controller = None
        self.side = ''
        self.last_report = None
        self.last_input_time = None
        self.connection_generation = 0

    @property
    def connected(self):
        return self.controller is not None

    def _sample(self, **values):
        return JoySample(
            connected=self.connected, side=self.side,
            connection_generation=self.connection_generation, **values)

    def connect(self, announce_failure=False):
        """Right Joy-Con first, left as fallback. Connecting triggers the
        vendor driver's 2-second horizontal rest calibration."""
        if self.connected:
            return True
        for side in ('right', 'left'):
            try:
                self.controller = self.controller_factory(
                    side, without_rest_init=False, all_button_return=True)
                self.side = side
                self.last_report = None
                self.last_input_time = None
                self.connection_generation += 1
                self.info(f'{side.capitalize()} Joy-Con connected')
                return True
            except Exception as error:  # noqa: BLE001 - any HID error means "not there"
                self.controller = None
                if announce_failure:
                    self.warning(f'{side.capitalize()} Joy-Con unavailable: {error}')
        if announce_failure:
            self.warning('No Joy-Con found; holding still and rescanning.')
        return False

    def rescan(self):
        return self.connect()

    def recalibrate(self):
        """Re-run the vendor gyro rest calibration (caller must ensure the
        controller is static and no motion command is active)."""
        if not self.connected:
            return False
        try:
            self.controller.reset_joycon()  # vendor API: 2 s rest calibration
            return True
        except Exception as error:  # noqa: BLE001
            self.warning(f'Recalibration failed: {error}')
            return False

    def is_calibrating(self):
        if not self.connected:
            return False
        try:
            return bool(self.controller.gyro.is_calibrating)
        except Exception:  # noqa: BLE001
            return False

    def _drop(self):
        if self.controller is not None:
            try:
                self.controller.running = False
                self.controller.disconnnect()  # vendor spelling
            except Exception:  # noqa: BLE001
                pass
        was_connected = self.connected
        self.controller = None
        self.side = ''
        self.last_report = None
        self.last_input_time = None
        if was_connected:
            self.connection_generation += 1

    def poll(self):
        """Read one sample; unready or stale reports yield no usable input."""
        if not self.connected:
            return self._sample()
        bindings = self.side_bindings[self.side]
        joycon = self.controller.joycon
        try:
            report = bytes(joycon._input_report)
            calibrating = self.is_calibrating()
            posture = self.controller.get_control()[0]
            raw_sticks = tuple(getattr(joycon, name)() for name in bindings['stick'])
            buttons = {key: bool(getattr(joycon, bindings[key])()) for key in BUTTON_KEYS}
        except Exception as error:  # noqa: BLE001
            self.warning(f'Joy-Con read failed ({error}); dropping the connection')
            self._drop()
            return self._sample()
        if not report_is_ready(report):
            return self._sample(calibrating=calibrating)
        now = self.clock()
        if report != self.last_report:
            self.last_report = report
            self.last_input_time = now
        if now - self.last_input_time > self.input_timeout:
            # Data timeout: report the connection as lost so the state
            # machine freezes and demands recalibration on recovery.
            self.warning('Joy-Con data timeout; dropping the connection')
            self._drop()
            return self._sample()
        horizontal = normalize_axis(
            raw_sticks[0], self.stick_center, self.stick_half_range) * self.stick_horizontal_sign
        vertical = normalize_axis(
            raw_sticks[1], self.stick_center, self.stick_half_range) * self.stick_vertical_sign
        roll, pitch, yaw = posture[3:6]
        return self._sample(
            roll=float(roll), pitch=float(pitch), yaw=float(yaw),
            stick_horizontal=float(horizontal), stick_vertical=float(vertical),
            buttons=buttons, fresh=not calibrating, calibrating=calibrating)

    def close(self):
        self._drop()
