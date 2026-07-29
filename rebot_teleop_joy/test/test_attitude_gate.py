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

import math

from rebot_teleop_joy.attitude_gate import AttitudeGate, wrap_angle


DT = 0.01


def test_wrap_angle():
    assert abs(wrap_angle(math.pi + 0.1) - (-math.pi + 0.1)) < 1e-9
    assert abs(wrap_angle(-math.pi - 0.1) - (math.pi - 0.1)) < 1e-9
    assert wrap_angle(0.5) == 0.5


def test_low_pass_converges_and_smooths():
    gate = AttitudeGate(cutoff_hz=5.0, max_rate=100.0)
    gate.update(0.0, 0.0, 0.0, DT)
    out = None
    for _ in range(300):
        out = gate.update(0.3, -0.2, 0.1, DT)
    assert abs(out[0] - 0.3) < 1e-3
    assert abs(out[1] + 0.2) < 1e-3
    assert abs(out[2] - 0.1) < 1e-3
    # A step is smoothed: the first filtered response stays well below the step.
    gate2 = AttitudeGate(cutoff_hz=5.0, max_rate=100.0)
    gate2.update(0.0, 0.0, 0.0, DT)
    first = gate2.update(1.0, 0.0, 0.0, DT)
    assert first[0] < 0.5


def test_spike_freezes_one_cycle_and_faults_after_limit():
    gate = AttitudeGate(cutoff_hz=10.0, max_rate=5.0, spike_limit=3)
    gate.update(0.0, 0.0, 0.0, DT)
    steady = gate.update(0.001, 0.0, 0.0, DT)
    # Jump of 1 rad in 10 ms = 100 rad/s >> max_rate: output frozen.
    frozen = gate.update(1.0, 0.0, 0.0, DT)
    assert frozen == steady
    assert not gate.fault
    # Two more spikes reach the limit -> fault.
    gate.update(-1.0, 0.0, 0.0, DT)
    gate.update(1.0, 0.0, 0.0, DT)
    assert gate.fault
    # A clean frame clears the fault.
    gate.update(1.001, 0.0, 0.0, DT)
    assert not gate.fault


def test_static_detection():
    gate = AttitudeGate(static_rate_threshold=0.05, static_time=0.5)
    gate.update(0.0, 0.0, 0.0, DT)
    for _ in range(60):
        gate.update(0.0, 0.0, 0.0, DT)
    assert gate.is_static()
    # Motion resets the static timer.
    gate.update(0.5, 0.0, 0.0, DT)
    assert not gate.is_static()
