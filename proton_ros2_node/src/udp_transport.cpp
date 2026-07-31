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
#include <string>

#include <protoncpp/transport/core_udp4.hpp>

#include "proton_ros2_node/udp_transport.hpp"

namespace proton_ros2_node
{

namespace udp4 = proton::transport::udp4;

void UdpTransport::encode_and_send(const std::vector<uint8_t> & buf)
{
  udp4::Header header;
  udp4::fill_header(header, peer_node_id_);

  std::vector<uint8_t> send_buf;
  send_buf.resize(buf.size() + sizeof(header));
  std::memcpy(send_buf.data(), &header, sizeof(header));
  std::copy(buf.begin(), buf.end(), std::next(send_buf.begin(), sizeof(header)));

  send(send_buf.data(), send_buf.size());
}

}  // namespace proton_ros2_node
