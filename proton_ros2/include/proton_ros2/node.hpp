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

#ifndef PROTON_ROS2_NODE_HPP
#define PROTON_ROS2_NODE_HPP

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <protoncpp/node_builder/generator.hpp>
#include <proton/common.h>

#include <proton_ros2_interfaces/adaptor_interface.hpp>

#include "proton_ros2/plugin_loader.hpp"

#include "rclcpp/rclcpp.hpp"

namespace proton_ros2
{

/**
 * @struct DataForPeers
 *
 * Data for peers to receive
 */
struct DataForPeers
{
  std::vector<proton_endpoint_t> peers;
  std::vector<uint8_t> data;
};

/**
 * @class ProtonRos2Node
 *
 * Central class for proton ROS 2 node.
 * Handles subscriptions to topics that will be converted into proton signals,
 * and decoding received bundles into topics
 */
class ProtonRos2Node : public rclcpp::Node
{
public:
  ProtonRos2Node();
  virtual ~ProtonRos2Node() = default;

  /**
   * @brief Receive bytes from transport for processing. Should already be
   * decoded from proton transport
   */
  void recv_bytes(const uint8_t * buf, std::size_t len);
  void recv_bytes(const std::vector<uint8_t> & buf)
  {
    recv_bytes(buf.data(), buf.size());
  }

#if __cplusplus >= 202002L
  /**
   * @brief span-based access for C++20 and newer
   */
  void recv_bytes(std::span<const uint8_t> buf)
  {
    recv_bytes(buf.data(), buf.size());
  }
#endif  // __cplusplus >= 202002L

  /**
   * @brief Method that must be called periodically to get updated data to write
   * Returns a vector of data-peer pairings to encode and send according to peer options.
   */
  std::vector<DataForPeers> spin_once(const rclcpp::Time & time);

  const proton::node_builder::Config & get_config() const
  {
    return proton_config_;
  }

  std::string get_name() const
  {
    return target_name_;
  }

private:
  std::string target_name_;
  PluginLoader plugin_loader_;
  proton::node_builder::Config proton_config_;

  // publishers_ must be declared such that they die AFTER proton_node_. This prevents dangling
  // raw pointers captured in bundle-update callbacks (which are deliberately leaked by proton's
  // BundleAccess::set_callback and could be invoked during proton_node_ teardown).
  std::vector<std::shared_ptr<proton_ros2_interfaces::GenericPublisher>> publishers_;
  std::vector<std::unique_ptr<proton_ros2_interfaces::GenericSubscription>> subscribers_;

  proton::node_builder::GeneratedNode proton_node_;
};

}  // namespace proton_ros2

#endif  // PROTON_ROS2_NODE_HPP
