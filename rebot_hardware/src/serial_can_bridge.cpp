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

#include "rebot_hardware/serial_can_bridge.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace rebot_hardware
{

size_t FrameParser::push(
  const uint8_t * bytes, size_t len, damiao::CanFrame * out, size_t max_frames)
{
  // Append (drop oldest data if the buffer would overflow — the caller
  // drains every control cycle so this only happens after long stalls).
  if (len > buffer_.size()) {
    bytes += len - buffer_.size();
    len = buffer_.size();
    buffer_len_ = 0;
  }
  if (buffer_len_ + len > buffer_.size()) {
    const size_t excess = buffer_len_ + len - buffer_.size();
    std::memmove(buffer_.data(), buffer_.data() + excess, buffer_len_ - excess);
    buffer_len_ -= excess;
  }
  std::memcpy(buffer_.data() + buffer_len_, bytes, len);
  buffer_len_ += len;

  // Scan for 0xAA ... 0x55 delimited 16-byte frames (same sync strategy as
  // the official SDK: header + tail at fixed distance).
  size_t produced = 0;
  size_t i = 0;
  size_t consumed = 0;
  while (i + kRxFrameSize <= buffer_len_ && produced < max_frames) {
    if (buffer_[i] == kRxHeader && buffer_[i + kRxFrameSize - 1] == kRxTail) {
      const uint8_t * p = buffer_.data() + i;
      if (p[1] == kCmdFeedback) {
        damiao::CanFrame & frame = out[produced++];
        frame.id = static_cast<uint32_t>(p[3]) | (static_cast<uint32_t>(p[4]) << 8) |
          (static_cast<uint32_t>(p[5]) << 16) | (static_cast<uint32_t>(p[6]) << 24);
        std::memcpy(frame.data.data(), p + 7, 8);
      }
      i += kRxFrameSize;
      consumed = i;
    } else {
      ++i;
    }
  }
  // Keep the unconsumed remainder (possible partial frame) for next time.
  if (consumed > 0) {
    std::memmove(buffer_.data(), buffer_.data() + consumed, buffer_len_ - consumed);
    buffer_len_ -= consumed;
  }
  return produced;
}

SerialCanBridge::~SerialCanBridge() {close();}

void SerialCanBridge::encode_tx_frame(
  const damiao::CanFrame & frame, uint8_t (&out)[kTxFrameSize])
{
  // Fixed 30-byte frame observed in the official SDK (DM_CAN.py
  // send_data_frame): only the CAN id [13..14] and payload [21..28] vary.
  static constexpr uint8_t kTemplate[kTxFrameSize] = {
    0x55, 0xAA, 0x1E, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  std::memcpy(out, kTemplate, kTxFrameSize);
  out[13] = static_cast<uint8_t>(frame.id & 0xFF);
  out[14] = static_cast<uint8_t>((frame.id >> 8) & 0xFF);
  std::memcpy(out + 21, frame.data.data(), 8);
}

bool SerialCanBridge::open(const std::string & device, int baud_rate, std::string & error)
{
  close();
  const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    error = "cannot open '" + device + "': " + std::strerror(errno);
    return false;
  }

  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    error = "tcgetattr failed on '" + device + "': " + std::strerror(errno);
    ::close(fd);
    return false;
  }
  cfmakeraw(&tty);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  speed_t speed = B921600;
  switch (baud_rate) {
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
    case 460800: speed = B460800; break;
    case 921600: speed = B921600; break;
    case 1000000: speed = B1000000; break;
    default:
      error = "unsupported baud rate " + std::to_string(baud_rate);
      ::close(fd);
      return false;
  }
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    error = "tcsetattr failed on '" + device + "': " + std::strerror(errno);
    ::close(fd);
    return false;
  }
  tcflush(fd, TCIOFLUSH);

  std::lock_guard<std::mutex> lock(mutex_);
  fd_ = fd;
  parser_.reset();
  return true;
}

void SerialCanBridge::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialCanBridge::send(const damiao::CanFrame & frame)
{
  uint8_t wire[kTxFrameSize];
  encode_tx_frame(frame, wire);

  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) {return false;}
  size_t written = 0;
  int spins = 0;
  while (written < kTxFrameSize) {
    const ssize_t n = ::write(fd_, wire + written, kTxFrameSize - written);
    if (n > 0) {
      written += static_cast<size_t>(n);
    } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      // Output buffer full: bounded retry keeps write() real-time safe.
      if (++spins > 100) {return false;}
    } else if (n < 0 && errno == EINTR) {
      continue;
    } else {
      return false;
    }
  }
  return true;
}

size_t SerialCanBridge::receive(damiao::CanFrame * out, size_t max_frames)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) {return 0;}
  size_t produced = 0;
  while (produced < max_frames) {
    const ssize_t n = ::read(fd_, read_chunk_.data(), read_chunk_.size());
    if (n > 0) {
      produced +=
        parser_.push(read_chunk_.data(), static_cast<size_t>(n), out + produced,
        max_frames - produced);
      if (static_cast<size_t>(n) < read_chunk_.size()) {break;}
    } else {
      break;  // EAGAIN (no data) or error: nothing more to drain now
    }
  }
  return produced;
}

size_t SerialCanBridge::receive_for(damiao::CanFrame * out, size_t max_frames, int timeout_ms)
{
  const size_t immediate = receive(out, max_frames);
  if (immediate > 0) {return immediate;}

  pollfd pfd{};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0) {return 0;}
    pfd.fd = fd_;
  }
  pfd.events = POLLIN;
  if (::poll(&pfd, 1, timeout_ms) <= 0) {return 0;}
  return receive(out, max_frames);
}

}  // namespace rebot_hardware
