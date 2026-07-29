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

#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "rebot_hardware/damiao_motor_driver.hpp"
#include "rebot_hardware/serial_can_bridge.hpp"

namespace damiao = rebot_hardware::damiao;

constexpr damiao::MotorLimits kDm4310{12.5, 30.0, 10.0};
constexpr damiao::MotorLimits kDm4340p{12.5, 10.0, 28.0};

TEST(LimitsForModel, KnownModelsAndSpellings)
{
  auto l = damiao::limits_for_model("4310");
  ASSERT_TRUE(l.has_value());
  EXPECT_DOUBLE_EQ(l->p_max, 12.5);
  EXPECT_DOUBLE_EQ(l->v_max, 30.0);
  EXPECT_DOUBLE_EQ(l->t_max, 10.0);

  for (const char * spelling : {"4340P", "DM4340P", "DM-J4340P", "dm-j4340p"}) {
    auto l2 = damiao::limits_for_model(spelling);
    ASSERT_TRUE(l2.has_value()) << spelling;
    EXPECT_DOUBLE_EQ(l2->v_max, 10.0) << spelling;
    EXPECT_DOUBLE_EQ(l2->t_max, 28.0) << spelling;
  }

  EXPECT_FALSE(damiao::limits_for_model("9999").has_value());
  EXPECT_FALSE(damiao::limits_for_model("").has_value());
}

TEST(FloatUintMapping, EndpointsAndMidpoint)
{
  // Endpoints map to 0 and 2^bits - 1.
  EXPECT_EQ(damiao::float_to_uint(-12.5, -12.5, 12.5, 16), 0);
  EXPECT_EQ(damiao::float_to_uint(12.5, -12.5, 12.5, 16), 65535);
  EXPECT_EQ(damiao::float_to_uint(-30.0, -30.0, 30.0, 12), 0);
  EXPECT_EQ(damiao::float_to_uint(30.0, -30.0, 30.0, 12), 4095);
  // Zero of a symmetric range maps just below the middle (truncation),
  // matching the official SDK.
  EXPECT_EQ(damiao::float_to_uint(0.0, -12.5, 12.5, 16), 32767);
  EXPECT_EQ(damiao::float_to_uint(0.0, -30.0, 30.0, 12), 2047);
}

TEST(FloatUintMapping, ClampsOutOfRange)
{
  EXPECT_EQ(damiao::float_to_uint(100.0, -12.5, 12.5, 16), 65535);
  EXPECT_EQ(damiao::float_to_uint(-100.0, -12.5, 12.5, 16), 0);
  EXPECT_EQ(damiao::float_to_uint(600.0, 0.0, 500.0, 12), 4095);
  EXPECT_EQ(damiao::float_to_uint(-1.0, 0.0, 500.0, 12), 0);
}

TEST(FloatUintMapping, RoundTripWithinQuantizationError)
{
  // 16-bit position over [-12.5, 12.5]: LSB = 25 / 65535 ~ 3.8e-4 rad.
  for (double x : {-12.5, -3.14159, -0.5, 0.0, 0.5, 1.0, 3.14159, 12.5}) {
    const uint16_t u = damiao::float_to_uint(x, -12.5, 12.5, 16);
    EXPECT_NEAR(damiao::uint_to_float(u, -12.5, 12.5, 16), x, 25.0 / 65535.0);
  }
  // 12-bit torque over [-28, 28]: LSB = 56 / 4095 ~ 0.014 N*m.
  for (double x : {-28.0, -1.0, 0.0, 5.0, 28.0}) {
    const uint16_t u = damiao::float_to_uint(x, -28.0, 28.0, 12);
    EXPECT_NEAR(damiao::uint_to_float(u, -28.0, 28.0, 12), x, 56.0 / 4095.0);
  }
}

TEST(ManagementFrames, EnableDisableSetZero)
{
  const auto enable = damiao::make_enable_frame(0x01);
  EXPECT_EQ(enable.id, 0x01u);
  const std::array<uint8_t, 8> expected_enable{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  EXPECT_EQ(enable.data, expected_enable);

  const auto disable = damiao::make_disable_frame(0x07);
  EXPECT_EQ(disable.id, 0x07u);
  EXPECT_EQ(disable.data[7], 0xFD);

  const auto zero = damiao::make_set_zero_frame(0x03);
  EXPECT_EQ(zero.id, 0x03u);
  EXPECT_EQ(zero.data[7], 0xFE);
}

TEST(MitFrame, ZeroCommandKnownBytes)
{
  // pos=0 -> 32767 (0x7FFF), vel=0 -> 2047 (0x7FF), kp=0 -> 0, kd=0 -> 0,
  // tau=0 -> 2047 (0x7FF). Reference: official SDK controlMIT() packing.
  const auto f = damiao::make_mit_frame(0x02, kDm4310, 0.0, 0.0, 0.0, 0.0, 0.0);
  EXPECT_EQ(f.id, 0x02u);
  const std::array<uint8_t, 8> expected{0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF};
  EXPECT_EQ(f.data, expected);
}

TEST(MitFrame, FullScaleKnownBytes)
{
  // All values at the positive end of their ranges -> all-ones fields.
  const auto f = damiao::make_mit_frame(0x01, kDm4340p, 12.5, 10.0, 500.0, 5.0, 28.0);
  const std::array<uint8_t, 8> expected{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  EXPECT_EQ(f.data, expected);

  const auto g = damiao::make_mit_frame(0x01, kDm4340p, -12.5, -10.0, 0.0, 0.0, -28.0);
  const std::array<uint8_t, 8> expected_min{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_EQ(g.data, expected_min);
}

TEST(MitFrame, EncodeParseRoundTrip)
{
  // Encode a command, rebuild the equivalent feedback frame, and check the
  // decoded values agree with the command within quantization error.
  const double pos = 1.234, vel = -2.5, tau = 3.3;
  const auto cmd = damiao::make_mit_frame(0x05, kDm4310, pos, vel, 100.0, 2.0, tau);

  const auto p = static_cast<uint16_t>((cmd.data[0] << 8) | cmd.data[1]);
  const auto v = static_cast<uint16_t>((cmd.data[2] << 4) | (cmd.data[3] >> 4));

  damiao::CanFrame fb_frame;
  fb_frame.id = 0x15;
  fb_frame.data[0] = 0x05;  // motor id 5, no error
  fb_frame.data[1] = static_cast<uint8_t>(p >> 8);
  fb_frame.data[2] = static_cast<uint8_t>(p & 0xFF);
  fb_frame.data[3] = static_cast<uint8_t>(v >> 4);
  const uint16_t t = damiao::float_to_uint(tau, -kDm4310.t_max, kDm4310.t_max, 12);
  fb_frame.data[4] = static_cast<uint8_t>(((v & 0xF) << 4) | ((t >> 8) & 0xF));
  fb_frame.data[5] = static_cast<uint8_t>(t & 0xFF);
  fb_frame.data[6] = 40;
  fb_frame.data[7] = 35;

  const auto fb = damiao::parse_feedback(fb_frame, kDm4310);
  EXPECT_EQ(fb.motor_id, 0x05);
  EXPECT_EQ(fb.error, 0x0);
  EXPECT_FALSE(damiao::is_fault(fb.error));
  EXPECT_NEAR(fb.position, pos, 25.0 / 65535.0);
  EXPECT_NEAR(fb.velocity, vel, 60.0 / 4095.0);
  EXPECT_NEAR(fb.torque, tau, 20.0 / 4095.0);
  EXPECT_DOUBLE_EQ(fb.t_mos, 40.0);
  EXPECT_DOUBLE_EQ(fb.t_rotor, 35.0);
}

TEST(ParseFeedback, FaultCodes)
{
  damiao::CanFrame frame;
  frame.data[0] = 0xD1;  // motor 1, error 0xD = communication loss
  const auto fb = damiao::parse_feedback(frame, kDm4310);
  EXPECT_EQ(fb.motor_id, 0x01);
  EXPECT_EQ(fb.error, 0xD);
  EXPECT_TRUE(damiao::is_fault(fb.error));
  EXPECT_EQ(
    static_cast<damiao::FaultCode>(fb.error), damiao::FaultCode::kCommunicationLoss);
  // Enable-state code (0x1) is not a fault.
  EXPECT_FALSE(damiao::is_fault(0x1));
}

TEST(PosVelFrame, IdOffsetAndLittleEndianFloats)
{
  const auto f = damiao::make_pos_vel_frame(0x04, 1.5f, -0.25f);
  EXPECT_EQ(f.id, 0x104u);
  float pos = 0.0f, vel = 0.0f;
  std::memcpy(&pos, f.data.data(), 4);
  std::memcpy(&vel, f.data.data() + 4, 4);
  EXPECT_FLOAT_EQ(pos, 1.5f);
  EXPECT_FLOAT_EQ(vel, -0.25f);
}

// ---------------------------------------------------------------------------
// Serial bridge wire format
// ---------------------------------------------------------------------------

TEST(SerialWireFormat, TxFrameLayout)
{
  const auto cmd = damiao::make_enable_frame(0x0102);
  uint8_t wire[rebot_hardware::SerialCanBridge::kTxFrameSize];
  rebot_hardware::SerialCanBridge::encode_tx_frame(cmd, wire);

  EXPECT_EQ(wire[0], 0x55);
  EXPECT_EQ(wire[1], 0xAA);
  EXPECT_EQ(wire[2], 0x1E);  // frame length 30
  EXPECT_EQ(wire[13], 0x02);  // CAN id low byte
  EXPECT_EQ(wire[14], 0x01);  // CAN id high byte
  EXPECT_EQ(wire[18], 0x08);  // DLC
  EXPECT_EQ(0, std::memcmp(wire + 21, cmd.data.data(), 8));
  EXPECT_EQ(wire[29], 0x00);
}

namespace
{

std::vector<uint8_t> make_rx_packet(uint32_t can_id, const std::array<uint8_t, 8> & data)
{
  std::vector<uint8_t> pkt(16, 0);
  pkt[0] = 0xAA;
  pkt[1] = 0x11;
  pkt[3] = static_cast<uint8_t>(can_id & 0xFF);
  pkt[4] = static_cast<uint8_t>((can_id >> 8) & 0xFF);
  pkt[5] = static_cast<uint8_t>((can_id >> 16) & 0xFF);
  pkt[6] = static_cast<uint8_t>((can_id >> 24) & 0xFF);
  std::memcpy(pkt.data() + 7, data.data(), 8);
  pkt[15] = 0x55;
  return pkt;
}

}  // namespace

TEST(FrameParser, SingleAndMultipleFrames)
{
  rebot_hardware::FrameParser parser;
  damiao::CanFrame out[8];

  const std::array<uint8_t, 8> payload{0x01, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 40, 35};
  auto pkt = make_rx_packet(0x11, payload);
  EXPECT_EQ(parser.push(pkt.data(), pkt.size(), out, 8), 1u);
  EXPECT_EQ(out[0].id, 0x11u);
  EXPECT_EQ(out[0].data, payload);

  // Two frames back-to-back.
  std::vector<uint8_t> two = make_rx_packet(0x12, payload);
  auto second = make_rx_packet(0x13, payload);
  two.insert(two.end(), second.begin(), second.end());
  EXPECT_EQ(parser.push(two.data(), two.size(), out, 8), 2u);
  EXPECT_EQ(out[0].id, 0x12u);
  EXPECT_EQ(out[1].id, 0x13u);
}

TEST(FrameParser, PartialFrameAcrossPushes)
{
  rebot_hardware::FrameParser parser;
  damiao::CanFrame out[4];

  const std::array<uint8_t, 8> payload{0x02, 0, 0, 0, 0, 0, 0, 0};
  auto pkt = make_rx_packet(0x12, payload);
  // First half only: no frame yet.
  EXPECT_EQ(parser.push(pkt.data(), 9, out, 4), 0u);
  // Second half completes the frame.
  EXPECT_EQ(parser.push(pkt.data() + 9, pkt.size() - 9, out, 4), 1u);
  EXPECT_EQ(out[0].id, 0x12u);
}

TEST(FrameParser, ResynchronizesAfterGarbage)
{
  rebot_hardware::FrameParser parser;
  damiao::CanFrame out[4];

  const std::array<uint8_t, 8> payload{0x03, 1, 2, 3, 4, 5, 6, 7};
  std::vector<uint8_t> stream{0xDE, 0xAD, 0xBE, 0xEF, 0x55, 0xAA};  // noise
  auto pkt = make_rx_packet(0x13, payload);
  stream.insert(stream.end(), pkt.begin(), pkt.end());
  EXPECT_EQ(parser.push(stream.data(), stream.size(), out, 4), 1u);
  EXPECT_EQ(out[0].id, 0x13u);
  EXPECT_EQ(out[0].data, payload);
}

TEST(FrameParser, IgnoresNonFeedbackCommands)
{
  rebot_hardware::FrameParser parser;
  damiao::CanFrame out[4];

  const std::array<uint8_t, 8> payload{0x01, 0, 0, 0, 0, 0, 0, 0};
  auto pkt = make_rx_packet(0x11, payload);
  pkt[1] = 0x22;  // not a feedback command
  EXPECT_EQ(parser.push(pkt.data(), pkt.size(), out, 4), 0u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
