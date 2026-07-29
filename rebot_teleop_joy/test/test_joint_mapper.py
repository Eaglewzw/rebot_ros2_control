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

from rebot_teleop_joy.joint_mapper import Gear, JointMapper, limit_margin_factor


LOWER = [-2.8, -3.14, -3.14, -1.87, -1.57, -3.14]
UPPER = [2.8, 0.0, 0.0, 1.57, 1.57, 3.14]
MAXV = [1.5, 1.5, 1.5, 2.0, 2.0, 2.0]
DT = 0.02


def full_gear():
    gear = Gear(incremental_scales=(1.0,), velocity_scales=(1.0,))
    return gear


def make_mapper(**kwargs):
    mapper = JointMapper(LOWER, UPPER, MAXV, gear=full_gear(), **kwargs)
    mapper.anchor((0.0, 0.0, 0.0), [0.0, -1.0, -1.0, 0.0, 0.0, 0.0])
    return mapper


def test_roll_pitch_incremental_mapping():
    mapper = make_mapper()
    # A held attitude offset converges 1:1 (rate limiter permitting).
    targets = None
    for _ in range(50):
        targets = mapper.step((0.1, -0.2, 0.0), 0.0, 0.0, False, False, DT)
    assert abs(targets[5] - 0.1) < 1e-9      # joint6 <- roll
    assert abs(targets[3] + 0.2) < 1e-9      # joint4 <- pitch


def test_incremental_mapping_is_rate_limited():
    mapper = make_mapper()
    # A huge instantaneous wrist flick must not exceed max_vel * dt.
    targets = mapper.step((2.0, 0.0, 0.0), 0.0, 0.0, False, False, DT)
    assert targets[5] <= MAXV[5] * DT + 1e-9


def test_yaw_deadband_and_saturation():
    mapper = make_mapper()
    assert mapper.yaw_velocity(math.radians(5.0)) == 0.0          # inside deadband
    v_mid = mapper.yaw_velocity(math.radians(30.0))
    v_sat = mapper.yaw_velocity(math.radians(60.0))
    assert 0.0 < v_mid < v_sat
    assert abs(v_sat - mapper.yaw_max_velocity) < 1e-9            # saturated
    assert mapper.yaw_velocity(-math.radians(60.0)) == -v_sat     # symmetric


def test_velocity_joints_follow_sticks_and_buttons():
    mapper = make_mapper()
    before = list(mapper.targets)
    mapper.step((0.0, 0.0, 0.0), 0.5, -0.8, True, False, DT)
    after = list(mapper.targets)
    assert after[1] < before[1]        # stick back -> joint2 negative
    assert after[4] > before[4]        # stick right -> joint5 positive
    assert after[2] > before[2]        # shoulder -> joint3 positive
    mapper.step((0.0, 0.0, 0.0), 0.0, 0.0, False, True, DT)
    assert mapper.targets[2] < after[2]  # stick press -> joint3 negative


def test_limit_margin_decay():
    margin = math.radians(5.0)
    # Far from the limit: no decay.
    assert limit_margin_factor(0.0, 1.0, -1.0, 1.0, margin) == 1.0
    # At half margin toward the limit: half speed.
    position = 1.0 - margin / 2.0
    assert abs(limit_margin_factor(position, 1.0, -1.0, 1.0, margin) - 0.5) < 1e-9
    # At the limit: zero toward it, free away from it.
    assert limit_margin_factor(1.0, 1.0, -1.0, 1.0, margin) == 0.0
    assert limit_margin_factor(1.0, -1.0, -1.0, 1.0, margin) == 1.0


def test_targets_never_leave_soft_limits():
    mapper = make_mapper()
    for _ in range(2000):
        mapper.step((6.0, -6.0, 3.0), 1.0, 1.0, True, False, DT)
    for target, lo, up in zip(mapper.targets, LOWER, UPPER):
        assert lo - 1e-9 <= target <= up + 1e-9


def test_anchor_resyncs_incremental_joints():
    mapper = make_mapper()
    mapper.step((0.5, 0.3, 0.0), 0.0, 0.0, False, False, 1.0)
    # Re-anchor at new attitude: no motion until the attitude changes again.
    mapper.anchor((0.5, 0.3, 0.0), list(mapper.targets))
    before = list(mapper.targets)
    targets = mapper.step((0.5, 0.3, 0.0), 0.0, 0.0, False, False, DT)
    assert targets == before


def test_gear_scales_increment():
    gear = Gear(incremental_scales=(0.5, 1.0), velocity_scales=(0.5, 1.0))
    mapper = JointMapper(LOWER, UPPER, MAXV, gear=gear)
    mapper.anchor((0.0, 0.0, 0.0), [0.0, -1.0, -1.0, 0.0, 0.0, 0.0])
    targets_low = mapper.step((0.02, 0.0, 0.0), 0.0, 0.0, False, False, DT)
    assert abs(targets_low[5] - 0.01) < 1e-9  # 0.5x gear
    gear.cycle()
    mapper.anchor((0.0, 0.0, 0.0), [0.0, -1.0, -1.0, 0.0, 0.0, 0.0])
    targets_high = mapper.step((0.02, 0.0, 0.0), 0.0, 0.0, False, False, DT)
    assert abs(targets_high[5] - 0.02) < 1e-9  # 1.0x gear
