// Copyright 2026 reBot ros2_control contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef REBOT_HARDWARE__DAMIAO_MOTOR_DRIVER_HPP_
#define REBOT_HARDWARE__DAMIAO_MOTOR_DRIVER_HPP_

/// \file damiao_motor_driver.hpp
/// \brief Damiao (达妙) motor CAN protocol: MIT-mode frame encoding/decoding.
///
/// Pure protocol layer, independent of ROS, unit-testable.
/// Sources (see docs/damiao_protocol_notes.md for details):
///  - DM-J4310-2EC V1.1 official manual (frame layouts, kp/kd ranges)
///  - Damiao official SDK DM_CAN.py (limit table, 0xFC/0xFD/0xFE frames)

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace rebot_hardware
{
namespace damiao
{

/// A raw classic CAN frame (standard ID, fixed 8-byte payload for Damiao).
struct CanFrame
{
  uint32_t id{0};
  std::array<uint8_t, 8> data{};
};

/// Per-model fixed-point mapping ranges. MUST match the motor firmware
/// (registers PMAX/VMAX/TMAX), otherwise position/velocity/torque values
/// are scaled wrongly. Do not mix models.
struct MotorLimits
{
  double p_max;  ///< position range: [-p_max, p_max] rad
  double v_max;  ///< velocity range: [-v_max, v_max] rad/s
  double t_max;  ///< torque   range: [-t_max, t_max] N*m
};

/// kp/kd ranges are identical for all Damiao models (official manual).
constexpr double kKpMin = 0.0;
constexpr double kKpMax = 500.0;
constexpr double kKdMin = 0.0;
constexpr double kKdMax = 5.0;

/// Look up factory-default limits by model name.
/// Accepted spellings: "4310", "DM4310", "DM-J4310", "4340P", "DM-J4340P", ...
std::optional<MotorLimits> limits_for_model(const std::string & model);

/// Linear float -> unsigned fixed-point mapping (clamps x to [x_min, x_max]).
uint16_t float_to_uint(double x, double x_min, double x_max, unsigned int bits);

/// Inverse mapping of float_to_uint().
double uint_to_float(uint16_t u, double x_min, double x_max, unsigned int bits);

/// Damiao register IDs (see official manual / motorbridge damiao_registers.h).
constexpr uint8_t kRidCtrlMode = 10;
constexpr uint8_t kRidTimeout = 9;
constexpr uint8_t kModeMit = 1;  ///< MIT control mode value for RID_CTRL_MODE
/// CAN communication timeout: 500 ms in 50 µs units. If the motor receives
/// no valid CAN frame for this period it auto-disables (safety fallback for
/// serial-bridge disconnection, severed cable, etc.).
constexpr uint32_t kDefaultCanTimeout = 500'000 / 50;  // = 10000

/// Management frames (data = FF FF FF FF FF FF FF FC/FD/FE, frame id = CAN id).
CanFrame make_enable_frame(uint32_t motor_id);
CanFrame make_disable_frame(uint32_t motor_id);
CanFrame make_set_zero_frame(uint32_t motor_id);

/// Parameter write frame: writes a 32-bit value to a motor register.
/// CAN id = 0x7FF (broadcast parameter access).
/// Payload: [motor_id, 0x00, 0x55, reg_id, value_32LE]
CanFrame make_parameter_write_frame(
  uint32_t motor_id, uint8_t register_id, uint32_t value);

/// MIT-mode control frame (frame id = CAN id).
/// Layout: p[15:8] p[7:0] v[11:4] v[3:0]|kp[11:8] kp[7:0] kd[11:4]
///         kd[3:0]|t[11:8] t[7:0]
CanFrame make_mit_frame(
  uint32_t motor_id, const MotorLimits & limits, double position, double velocity, double kp,
  double kd, double torque);

/// Position-velocity mode frame (frame id = 0x100 + CAN id). Reserved for
/// future use; the framework drives the motors in MIT mode.
CanFrame make_pos_vel_frame(uint32_t motor_id, float position, float velocity);

/// Decoded feedback frame (identical layout in every control mode).
struct Feedback
{
  uint8_t motor_id{0};  ///< low 4 bits of the motor CAN id (D0 low nibble)
  uint8_t error{0};     ///< fault code (D0 high nibble), 0/1 = no fault
  double position{0.0};
  double velocity{0.0};
  double torque{0.0};
  double t_mos{0.0};    ///< driver MOS temperature [deg C]
  double t_rotor{0.0};  ///< motor coil temperature [deg C]
};

/// Fault codes reported in Feedback::error (official manual).
enum class FaultCode : uint8_t
{
  kOvervoltage = 0x8,
  kUndervoltage = 0x9,
  kOvercurrent = 0xA,
  kMosOvertemperature = 0xB,
  kCoilOvertemperature = 0xC,
  kCommunicationLoss = 0xD,
  kOverload = 0xE,
};

/// True when the code is one of the documented fault values (>= 0x8).
bool is_fault(uint8_t error_code);

/// Decode a feedback frame using the limits of the motor that sent it.
Feedback parse_feedback(const CanFrame & frame, const MotorLimits & limits);

}  // namespace damiao
}  // namespace rebot_hardware

#endif  // REBOT_HARDWARE__DAMIAO_MOTOR_DRIVER_HPP_
