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

#ifndef PROTON_ROS2_NODE_BASE_TRANSPORT_HPP
#define PROTON_ROS2_NODE_BASE_TRANSPORT_HPP

#include <cstdint>
#include <functional>
#include <memory>

#include <serial_hardware/drivers/base_driver.hpp>

namespace proton_ros2_node
{

/**
 * @class BaseTransport: base class for proton transport
 */
class BaseTransport
{
public:
  virtual ~BaseTransport() = default;

  // TODO (inherited from HAL) UDP requires reconnection logic.
  // Will come from upstream serial_hardware at a later date.
  virtual void connect()
  {
    driver_->connect();
  }

  virtual void disconnect()
  {
    driver_->disconnect();
  }

  virtual void send(const uint8_t * buf, const size_t len)
  {
    driver_->send(buf, len);
  }

  virtual void set_receive_callback(std::function<void(const uint8_t * buf, const size_t len)> & fn)
  {
    // TODO fix this in HAL -_-
    driver_->setRecieveCallback(fn);  // cspell:disable-line
  }

protected:
  std::unique_ptr<serial_hardware::drivers::BaseDriver> driver_;
};

}  // namespace proton_ros2_node

#endif  // PROTON_ROS2_NODE_BASE_TRANSPORT_HPP
