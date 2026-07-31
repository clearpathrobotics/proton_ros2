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
#include <optional>
#include <utility>

#include "proton_ros2_node/transport_factory.hpp"
#include "proton_ros2_node/serial_transport.hpp"
#include "proton_ros2_node/udp_transport.hpp"

namespace proton_ros2_node
{

std::unique_ptr<BaseTransport> transport_factory(
  rclcpp::Logger logger, const Config & config,
  const std::string & node_name, uint32_t endpoint_id)
{
  // Determine if there is a connection for this node and endpoint
  std::optional<std::string> peer_name = std::nullopt;
  std::optional<uint32_t> peer_endpoint_id = std::nullopt;
  for (const auto & conn : config.connections) {
    if (conn.first.node == node_name && conn.first.id == endpoint_id) {
      peer_name = conn.second.node;
      peer_endpoint_id = conn.second.id;
      break;
    } else if (conn.second.node == node_name && conn.second.id == endpoint_id) {
      peer_name = conn.first.node;
      peer_endpoint_id = conn.first.id;
      break;
    }
  }

  if (peer_name.has_value() && peer_endpoint_id.has_value()) {
    RCLCPP_INFO(logger, "Connecting %s:%d with %s:%d",
      node_name.c_str(), endpoint_id, peer_name->c_str(), *peer_endpoint_id
    );
  } else {
    throw std::runtime_error("No connection found for '" + node_name + "' endpoint " +
        std::to_string(endpoint_id));
  }

  // Check if these node/endpoint pairings exist
  std::array<std::pair<std::string, uint32_t>, 2> node_endpoints = {{
    {node_name, endpoint_id},
    {*peer_name, *peer_endpoint_id}
  }};

  for (const auto & [name, id] : node_endpoints) {
    if (!config.nodes.contains(name)) {
      throw std::runtime_error("Node name '" + name + "' does not exist in config");
    }

    const auto & node_config = config.nodes.at(name);

    if (!node_config.endpoints.contains(id)) {
      throw std::runtime_error("Endpoint ID " + std::to_string(id) + " does not exist within '" +
          node_name + "'");
    }
  }

  const auto & host_endpoint = config.nodes.at(node_name).endpoints.at(endpoint_id);
  const auto & peer_endpoint = config.nodes.at(*peer_name).endpoints.at(*peer_endpoint_id);

  if (host_endpoint.type != peer_endpoint.type) {
    throw std::runtime_error(
      "Mismatched endpoint types for '" + node_name + "':" + std::to_string(endpoint_id) +
      " and '" + *peer_name + "':" + std::to_string(*peer_endpoint_id) + ". " +
      host_endpoint.type + " != " + peer_endpoint.type);
  }

  if (host_endpoint.type == "udp4") {
    return std::make_unique<UdpTransport>(host_endpoint.ip, peer_endpoint.ip, host_endpoint.port,
        peer_endpoint.port);
  } else if (host_endpoint.type == "serial") {
    // TODO (twallis) these parameters aren't part of the proton config, may need to add them. Baud at the very least
    return std::make_unique<SerialTransport>(host_endpoint.device, 115200, 1024, 1000, 10);
  } else {
    throw std::runtime_error("Unknown transport type '" + host_endpoint.type + "'");
  }

  return nullptr;
}

}  // namespace proton_ros2_node
