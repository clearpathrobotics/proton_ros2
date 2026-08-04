/*
 * Copyright 2026 Rockwell Automation Technologies, Inc., All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Tom Wallis (thomas.wallis@rockwellautomation.com)
 */

#include <array>
#include <iterator>

#include <protoncpp/transport/core_serial.hpp>

#include "proton_ros2_node/serial_transport.hpp"

namespace proton_ros2_node
{

namespace serial = proton::transport::serial;

proton_status_e SerialTransport::encode_and_send(const std::vector<uint8_t> & buf)
{
  std::vector<uint8_t> send_buf;
  send_buf.resize(buf.size() + serial::FRAME_OVERHEAD);

  uint16_t crc;
  proton_status_e status = serial::fill_crc16(buf.data(), buf.size(), &crc);

  if (status == PROTON_OK) {
    send_buf[serial::FRAME_HEADER_OVERHEAD] = static_cast<uint8_t>(crc & 0xFF);
    send_buf[serial::FRAME_HEADER_OVERHEAD + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    std::copy(buf.begin(), buf.end(), send_buf.begin() + serial::FRAME_OVERHEAD);

    send(send_buf.data(), send_buf.size());
  }

  return status;
}

void SerialTransport::handle_bytes(const uint8_t * buf, const size_t len)
{
  using namespace proton::transport;

  for (size_t i = 0; i < len; i++) {
    rx_buf_.push_back(buf[i]);
  }

  const size_t buf_len = rx_buf_.size();

  if (decode_state_ == DecodeState::GetHeader) {
    if (buf_len <= serial::FRAME_HEADER_OVERHEAD) {
      return;
    }

    bool header_found = false;
    for (size_t i = 0; i <= buf_len - serial::FRAME_HEADER_OVERHEAD; i++) {
      if (serial::get_framed_payload_length(
          {rx_buf_.data() + i, buf_len - i}, payload_len_
        ) == PROTON_OK)
      {
        // header found, discard bytes before header so decode index starts at byte 0
        if (i > 0) {
          rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + i);
        }
        decode_state_ = DecodeState::AccumulateLength;
        header_found = true;
        break;
      }
    }

    if (!header_found) {
      rx_buf_.clear();
    }
  }

  if (decode_state_ == DecodeState::AccumulateLength) {
    const size_t required_len = serial::FRAME_OVERHEAD + payload_len_;
    if (rx_buf_.size() >= required_len) {
      decode_state_ = DecodeState::Verify;
    }
  }

  if (decode_state_ == DecodeState::Verify) {
    constexpr size_t crc_offset = serial::FRAME_HEADER_OVERHEAD;
    constexpr size_t payload_offset = serial::FRAME_OVERHEAD;

    // Check CRC
    // All data is written little-endian on the wire
    const uint16_t read_crc = static_cast<uint16_t>(rx_buf_[crc_offset]) |
      static_cast<uint16_t>(rx_buf_[crc_offset + 1]) << 8;
    proton_status_e verify_status = serial::check_framed_payload(&rx_buf_[payload_offset],
        payload_len_, read_crc);

    if (verify_status == PROTON_OK) {
      message_ready({&rx_buf_[payload_offset], payload_len_});
    } else {
      RCLCPP_ERROR(logger_, "Error checking serial payload: %s. CRC read 0x%04x",
      proton_status_to_string(verify_status), read_crc);
      // Bad CRC, drop one byte and resync to see if another message is present
      rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + 1);
    }

    payload_len_ = 0;
    decode_state_ = DecodeState::GetHeader;
  }
}

}  // namespace proton_ros2_node
