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
#include <map>
#include <span>
#include <string>
#include <vector>

#include <protoncpp/node_builder/generator.hpp>
#include <proton/common.h>

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
  void process_bytes(const uint8_t * buf, std::size_t len);
  void process_bytes(const std::vector<uint8_t> & buf)
  {
    process_bytes(buf.data(), buf.size());
  }

#if __cplusplus >= 202002L
  /**
   * @brief span-based access for C++20 and newer
   */
  void process_bytes(std::span<const uint8_t> buf)
  {
    process_bytes(buf.data(), buf.size());
  }
#endif  // __cplusplus >= 202002L

  /**
   * @brief Method that must be called periodically to get updated data to write
   * Returns a vector of data-peer pairings to encode and send according to peer options.
   */
  std::vector<DataForPeers> spin_once(const rclcpp::Time & time);

private:
  proton::node_builder::GeneratedNode proton_node_;
  std::map<std::string, uint32_t> bundle_id_map_;
  std::map<std::string, uint32_t> signal_id_map_;
};

}  // namespace proton_ros2

#endif  // PROTON_ROS2_NODE_HPP
