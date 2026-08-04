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

#ifndef PROTON_ROS2_NODE_SERIAL_TRANSPORT_HPP
#define PROTON_ROS2_NODE_SERIAL_TRANSPORT_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "proton_ros2_node/base_transport.hpp"

#include <serial_hardware/drivers/serial_driver.hpp>

namespace proton_ros2_node
{
/**
 * @class SerialTransport for serial-based communications over proton using OTTO Motors' serial_hardware
 */
class SerialTransport : public BaseTransport
{
public:
  explicit SerialTransport(
    uint32_t node_id, uint32_t endpoint_id,
    const std::string & port, const size_t baud, const size_t buf_size,
    const size_t recovery_timer_interval_ms, const size_t recovery_error_threshold)
  : BaseTransport(node_id, endpoint_id)
  {
    driver_ = std::make_unique<serial_hardware::drivers::SerialDriver>(port, baud, buf_size,
        recovery_timer_interval_ms, recovery_error_threshold);
  }

  virtual ~SerialTransport() = default;

  void encode_and_send(const std::vector<uint8_t> & buf) override;

protected:
  void handle_bytes(const uint8_t * buf, const size_t len) override;

private:
  // Accumulates bytes across driver callbacks for serial framing.
  std::vector<uint8_t> rx_buf_;
};

}  // namespace proton_ros2_node

#endif  // PROTON_ROS2_NODE_SERIAL_TRANSPORT_HPP
