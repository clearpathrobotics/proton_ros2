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

#ifndef PROTON_ROS2_NODE_UDP_TRANSPORT_HPP
#define PROTON_ROS2_NODE_UDP_TRANSPORT_HPP

#include <cstdint>
#include <string>

#include <protoncpp/transport/core_udp4.hpp>

#include <serial_hardware/drivers/byte_udp_driver.hpp>

#include "proton_ros2_node/base_transport.hpp"

namespace proton_ros2_node
{

/**
 * @class UdpTransport for UDP-based communications over proton using OTTO Motors' serial_hardware
 */
class UdpTransport : public BaseTransport
{
public:
  explicit UdpTransport(
    uint32_t node_id, uint32_t endpoint_id,
    const std::string & host_ip, const std::string & remote_ip,
    const size_t host_port, const size_t remote_port)
  : BaseTransport(node_id, endpoint_id)
  {
    driver_ = std::make_unique<serial_hardware::drivers::ByteUdpDriver>(host_ip, remote_ip,
        host_port, remote_port);
  }

  virtual ~UdpTransport() = default;

  void encode_and_send(const std::vector<uint8_t> & buf) override;

protected:
  void handle_bytes(const uint8_t * buf, const size_t len) override;
};

}  // namespace proton_ros2_node

#endif // PROTON_ROS2_NODE_UDP_TRANSPORT_HPP
