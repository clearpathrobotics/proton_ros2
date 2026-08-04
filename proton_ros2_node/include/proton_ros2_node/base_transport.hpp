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
#include <span>
#include <vector>

#include <proton/common.h>

#include <proton_ros2/node.hpp>

#include <serial_hardware/drivers/base_driver.hpp>

namespace proton_ros2_node
{

/**
 * @class BaseTransport: base class for proton transport
 */
class BaseTransport
{
public:
  using MessageCallback = std::function<void(std::span<const uint8_t>)>;

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

  virtual void encode_and_send(const std::vector<uint8_t> & buf) = 0;

  // Called by owner when a complete decoded payload is ready. Registered
  // callback is stored as a member so it outlives the driver, which holds a
  // reference-typed callback of its own.
  void set_message_callback(MessageCallback fn)
  {
    message_cb_ = std::move(fn);
    if (!driver_cb_registered_) {
      driver_recv_cb_ = [this](const uint8_t * buf, const size_t len) {
          this->handle_bytes(buf, len);
        };
      // TODO fix spelling mistake in HAL -_-
      driver_->setRecieveCallback(driver_recv_cb_);  // cspell:disable-line
      driver_cb_registered_ = true;
    }
  }

  uint32_t node_id() const
  {
    return peer_node_id_;
  }

  uint32_t endpoint_id() const
  {
    return peer_endpoint_id_;
  }

protected:
  explicit BaseTransport(uint32_t peer_node_id, uint32_t peer_endpoint_id)
  : peer_node_id_(peer_node_id)
    , peer_endpoint_id_(peer_endpoint_id)
  {}

  /**
   * @brief Receive bytes from transport drivers and alert via
   * message_ready when a complete payload is received.
   */
  virtual void handle_bytes(const uint8_t * buf, const size_t len) = 0;

  void message_ready(std::span<const uint8_t> payload)
  {
    if (message_cb_) {
      message_cb_(payload);
    }
  }

  virtual void send(const uint8_t * buf, const size_t len)
  {
    driver_->send(buf, len);
  }

  uint32_t peer_node_id_;
  uint32_t peer_endpoint_id_;

  std::unique_ptr<serial_hardware::drivers::BaseDriver> driver_;

private:
  MessageCallback message_cb_;
  std::function<void(const uint8_t * buf, const size_t len)> driver_recv_cb_;
  bool driver_cb_registered_ = false;
};

}  // namespace proton_ros2_node

#endif  // PROTON_ROS2_NODE_BASE_TRANSPORT_HPP
