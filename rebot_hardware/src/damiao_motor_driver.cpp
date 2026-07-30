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

#include "rebot_hardware/damiao_motor_driver.hpp"

#include <algorithm>
#include <cstring>

namespace rebot_hardware
{
namespace damiao
{

namespace
{

/// Strip vendor prefixes so "DM-J4310", "DM4310" and "4310" all match.
std::string normalize_model(const std::string & model)
{
  std::string m = model;
  std::transform(m.begin(), m.end(), m.begin(), [](unsigned char c) { return std::toupper(c); });
  for (const char * prefix : {"DM-J", "DM-", "DMJ", "DM"}) {
    if (m.rfind(prefix, 0) == 0) {
      m = m.substr(std::strlen(prefix));
      break;
    }
  }
  return m;
}

CanFrame make_management_frame(uint32_t motor_id, uint8_t command)
{
  CanFrame frame;
  frame.id = motor_id;
  frame.data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command};
  return frame;
}

}  // namespace

std::optional<MotorLimits> limits_for_model(const std::string & model)
{
  // Factory defaults, source: Damiao SDK DM_CAN.py Limit_Param and
  // motorbridge DAMIAO_MODEL_LIMITS (P_MAX, V_MAX, T_MAX).
  const std::string m = normalize_model(model);
  if (m == "4310") {return MotorLimits{12.5, 30.0, 10.0};}
  if (m == "4310P" || m == "4310_48V") {return MotorLimits{12.5, 50.0, 10.0};}
  if (m == "4340") {return MotorLimits{12.5, 8.0, 28.0};}
  if (m == "4340P" || m == "4340_48V") {return MotorLimits{12.5, 10.0, 28.0};}
  if (m == "6006") {return MotorLimits{12.5, 45.0, 20.0};}
  if (m == "8006") {return MotorLimits{12.5, 45.0, 40.0};}
  if (m == "8009") {return MotorLimits{12.5, 45.0, 54.0};}
  return std::nullopt;
}

uint16_t float_to_uint(double x, double x_min, double x_max, unsigned int bits)
{
  const double clamped = std::clamp(x, x_min, x_max);
  const double span = x_max - x_min;
  const double normalized = (clamped - x_min) / span;
  return static_cast<uint16_t>(normalized * static_cast<double>((1u << bits) - 1));
}

double uint_to_float(uint16_t u, double x_min, double x_max, unsigned int bits)
{
  const double span = x_max - x_min;
  const double normalized = static_cast<double>(u) / static_cast<double>((1u << bits) - 1);
  return normalized * span + x_min;
}

CanFrame make_enable_frame(uint32_t motor_id) {return make_management_frame(motor_id, 0xFC);}

CanFrame make_disable_frame(uint32_t motor_id) {return make_management_frame(motor_id, 0xFD);}

CanFrame make_set_zero_frame(uint32_t motor_id) {return make_management_frame(motor_id, 0xFE);}

CanFrame make_parameter_write_frame(uint32_t motor_id, uint8_t register_id, uint32_t value)
{
  CanFrame frame;
  frame.id = 0x7FF;  // broadcast parameter access
  frame.data[0] = static_cast<uint8_t>(motor_id & 0xFF);
  frame.data[1] = 0x00;
  frame.data[2] = 0x55;  // opcode: write
  frame.data[3] = register_id;
  frame.data[4] = static_cast<uint8_t>(value & 0xFF);
  frame.data[5] = static_cast<uint8_t>((value >> 8) & 0xFF);
  frame.data[6] = static_cast<uint8_t>((value >> 16) & 0xFF);
  frame.data[7] = static_cast<uint8_t>((value >> 24) & 0xFF);
  return frame;
}

CanFrame make_mit_frame(
  uint32_t motor_id, const MotorLimits & limits, double position, double velocity, double kp,
  double kd, double torque)
{
  const uint16_t p = float_to_uint(position, -limits.p_max, limits.p_max, 16);
  const uint16_t v = float_to_uint(velocity, -limits.v_max, limits.v_max, 12);
  const uint16_t kp_u = float_to_uint(kp, kKpMin, kKpMax, 12);
  const uint16_t kd_u = float_to_uint(kd, kKdMin, kKdMax, 12);
  const uint16_t t = float_to_uint(torque, -limits.t_max, limits.t_max, 12);

  CanFrame frame;
  frame.id = motor_id;
  frame.data[0] = static_cast<uint8_t>(p >> 8);
  frame.data[1] = static_cast<uint8_t>(p & 0xFF);
  frame.data[2] = static_cast<uint8_t>(v >> 4);
  frame.data[3] = static_cast<uint8_t>(((v & 0xF) << 4) | ((kp_u >> 8) & 0xF));
  frame.data[4] = static_cast<uint8_t>(kp_u & 0xFF);
  frame.data[5] = static_cast<uint8_t>(kd_u >> 4);
  frame.data[6] = static_cast<uint8_t>(((kd_u & 0xF) << 4) | ((t >> 8) & 0xF));
  frame.data[7] = static_cast<uint8_t>(t & 0xFF);
  return frame;
}

CanFrame make_pos_vel_frame(uint32_t motor_id, float position, float velocity)
{
  CanFrame frame;
  frame.id = 0x100 + motor_id;
  std::memcpy(frame.data.data(), &position, sizeof(float));
  std::memcpy(frame.data.data() + 4, &velocity, sizeof(float));
  return frame;
}

bool is_fault(uint8_t error_code) {return error_code >= 0x8;}

Feedback parse_feedback(const CanFrame & frame, const MotorLimits & limits)
{
  Feedback fb;
  fb.motor_id = frame.data[0] & 0x0F;
  fb.error = frame.data[0] >> 4;
  const auto p = static_cast<uint16_t>((frame.data[1] << 8) | frame.data[2]);
  const auto v = static_cast<uint16_t>((frame.data[3] << 4) | (frame.data[4] >> 4));
  const auto t = static_cast<uint16_t>(((frame.data[4] & 0xF) << 8) | frame.data[5]);
  fb.position = uint_to_float(p, -limits.p_max, limits.p_max, 16);
  fb.velocity = uint_to_float(v, -limits.v_max, limits.v_max, 12);
  fb.torque = uint_to_float(t, -limits.t_max, limits.t_max, 12);
  fb.t_mos = static_cast<double>(frame.data[6]);
  fb.t_rotor = static_cast<double>(frame.data[7]);
  return fb;
}

}  // namespace damiao
}  // namespace rebot_hardware
