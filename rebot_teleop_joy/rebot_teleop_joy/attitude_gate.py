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

"""Attitude post-processing between the driver and the teleop mapper.

The vendored Mahony estimator already provides drift-free roll/pitch and a
gyro-integrated yaw.  This gate adds the task-level requirements on top:

* configurable low-pass (bandwidth limit against hand tremor),
* spike rejection: frames whose angular rate exceeds ``max_rate`` are
  dropped and the previous output is frozen for one cycle; ``spike_limit``
  consecutive spikes report a fault (the mapper then force-releases the
  deadman state),
* static detection (for gating recalibration requests).

Pure Python, no ROS — unit-tested in test/test_attitude_gate.py.
"""

import math


def wrap_angle(angle):
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class AttitudeGate:

    def __init__(self, cutoff_hz=10.0, max_rate=25.0, spike_limit=5,
                 static_rate_threshold=0.05, static_time=0.5):
        self.cutoff_hz = float(cutoff_hz)
        self.max_rate = float(max_rate)          # rad/s, per axis
        self.spike_limit = int(spike_limit)
        self.static_rate_threshold = float(static_rate_threshold)  # rad/s
        self.static_time = float(static_time)
        self.reset()

    def reset(self):
        self.filtered = None
        self.previous_raw = None
        self.spike_count = 0
        self.static_elapsed = 0.0

    @property
    def fault(self):
        """True after ``spike_limit`` consecutive over-rate frames."""
        return self.spike_count >= self.spike_limit

    def is_static(self):
        return self.static_elapsed >= self.static_time

    def update(self, roll, pitch, yaw, dt):
        """Feed one raw attitude sample; returns (roll, pitch, yaw) filtered,
        or the frozen previous output when the frame was rejected."""
        raw = (roll, pitch, yaw)
        if dt <= 0.0:
            return self.filtered if self.filtered is not None else raw
        if self.previous_raw is None:
            self.previous_raw = raw
            self.filtered = raw
            return self.filtered

        rates = [abs(wrap_angle(raw[i] - self.previous_raw[i])) / dt for i in range(3)]
        peak = max(rates)
        self.previous_raw = raw

        # Static detection runs on raw rates regardless of spikes.
        if peak < self.static_rate_threshold:
            self.static_elapsed += dt
        else:
            self.static_elapsed = 0.0

        if peak > self.max_rate:
            # Spike: freeze one cycle.
            self.spike_count += 1
            return self.filtered

        self.spike_count = 0
        # First-order low-pass, alpha from the cutoff frequency.
        alpha = 1.0 if self.cutoff_hz <= 0.0 else (
            (2.0 * math.pi * self.cutoff_hz * dt) / (2.0 * math.pi * self.cutoff_hz * dt + 1.0))
        self.filtered = tuple(
            self.filtered[i] + alpha * wrap_angle(raw[i] - self.filtered[i]) for i in range(3))
        return self.filtered
