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

void SerialTransport::encode_and_send(const std::vector<uint8_t> & buf)
{
  std::vector<uint8_t> send_buf;
  send_buf.resize(len + sizeof(serial::FRAME_OVERHEAD));

  std::array<uint8_t, serial::FRAME_CRC_OVERHEAD> crc;
  serial::fill_crc16(buf.data(), buf.size(), crc.data());
  serial::fill_frame_header(send_buf.data(), buf.size());

  std::copy(crc.begin(), crc.end(), std::next(send_buf.begin(), serial::FRAME_HEADER_OVERHEAD));
  std::copy(buf.begin(), buf.end(), std::next(send_buf.begin(), serial::FRAME_OVERHEAD));

  send(send_buf.data(), send_buf.size());
}

}  // namespace proton_ros2_node
