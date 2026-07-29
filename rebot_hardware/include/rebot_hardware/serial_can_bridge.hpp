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

#ifndef REBOT_HARDWARE__SERIAL_CAN_BRIDGE_HPP_
#define REBOT_HARDWARE__SERIAL_CAN_BRIDGE_HPP_

/// \file serial_can_bridge.hpp
/// \brief Damiao USB-CAN serial bridge (/dev/ttyACM*) transport.
///
/// Wire format (source: Damiao SDK DM_CAN.py, see
/// docs/damiao_protocol_notes.md section 4):
///  - host -> bridge: fixed 30-byte frame, CAN id at [13..14] (LE),
///    DLC=8 at [18], CAN payload at [21..28]
///  - bridge -> host: fixed 16-byte frame, 0xAA header / 0x55 tail,
///    command byte at [1] (0x11 = motor feedback), CAN id at [3..6] (LE),
///    CAN payload at [7..14]
///
/// The parser is exposed separately (FrameParser) so it can be unit-tested
/// without hardware. All hot-path methods are allocation-free.

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

#include "rebot_hardware/damiao_motor_driver.hpp"

namespace rebot_hardware
{

/// Incremental parser for the 16-byte bridge->host frames.
class FrameParser
{
public:
  static constexpr size_t kRxFrameSize = 16;
  static constexpr uint8_t kRxHeader = 0xAA;
  static constexpr uint8_t kRxTail = 0x55;
  static constexpr uint8_t kCmdFeedback = 0x11;

  /// Append raw bytes; complete feedback frames are written to `out`
  /// (up to `max_frames`). Returns the number of frames produced.
  /// Unconsumed trailing bytes are kept for the next call.
  size_t push(const uint8_t * bytes, size_t len, damiao::CanFrame * out, size_t max_frames);

  void reset() {buffer_len_ = 0;}

private:
  std::array<uint8_t, 4096> buffer_{};
  size_t buffer_len_{0};
};

/// Thread-safe, non-blocking serial port wrapper for the Damiao USB-CAN
/// bridge board.
class SerialCanBridge
{
public:
  static constexpr size_t kTxFrameSize = 30;
  static constexpr int kDefaultBaudRate = 921600;

  SerialCanBridge() = default;
  ~SerialCanBridge();
  SerialCanBridge(const SerialCanBridge &) = delete;
  SerialCanBridge & operator=(const SerialCanBridge &) = delete;

  /// Encode a CAN frame into the 30-byte host->bridge wire format.
  static void encode_tx_frame(const damiao::CanFrame & frame, uint8_t (&out)[kTxFrameSize]);

  /// Open and configure the port (raw mode, non-blocking).
  /// On failure returns false and fills `error`.
  bool open(const std::string & device, int baud_rate, std::string & error);
  void close();
  bool is_open() const {return fd_ >= 0;}

  /// Non-blocking send of one CAN frame. Returns false on write failure.
  bool send(const damiao::CanFrame & frame);

  /// Drain the OS receive buffer and parse complete feedback frames into
  /// `out` (up to `max_frames`). Non-blocking; returns frames produced.
  size_t receive(damiao::CanFrame * out, size_t max_frames);

  /// Blocking variant with timeout, for use outside the real-time loop
  /// (activation / deactivation). Returns frames produced (0 on timeout).
  size_t receive_for(damiao::CanFrame * out, size_t max_frames, int timeout_ms);

private:
  int fd_{-1};
  FrameParser parser_;
  std::array<uint8_t, 2048> read_chunk_{};
  std::mutex mutex_;
};

}  // namespace rebot_hardware

#endif  // REBOT_HARDWARE__SERIAL_CAN_BRIDGE_HPP_
