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

#include "proton_ros2/config.hpp"
#include "proton_ros2/node.hpp"
#include "proton_ros2/utils.hpp"

#include <protoncpp/node_builder/config.hpp>
#include <protoncpp/node_access.hpp>

namespace proton_ros2
{

ProtonRos2Node::ProtonRos2Node()
: rclcpp::Node("proton_ros2")
{
  this->declare_parameter("config_file", rclcpp::PARAMETER_STRING);
  this->declare_parameter("target", rclcpp::PARAMETER_STRING);

  const auto config_file = get_parameter("config_file").as_string();
  const auto target = get_parameter("target").as_string();

  // Proton node builder will throw exceptions from errors in the config,
  // so allow the process to fail early.
  proton_node_ = node_from_config(config_file, target);
}

void ProtonRos2Node::recv_bytes(const uint8_t * buf, std::size_t len)
{
  proton_status_e status = proton::NodeAccess(proton_node_.node()).receive(buf, len);

  if (status != PROTON_OK) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Proton reception error: %s",
      error_to_string(status).c_str()
    );
  }
}

std::vector<DataForPeers> ProtonRos2Node::spin_once(const rclcpp::Time & time)
{
  proton::NodeAccess proton_node(proton_node_.node());
  const auto num_peers = proton_node.num_peers();
  const auto bundle_count = proton_node_.registry()->bundle_count;
  const uint64_t time_ms = time.seconds() * 1000 + time.nanoseconds() / 1000000;

  std::vector<DataForPeers> to_send;

  for (auto i = 0; i < bundle_count; i++) {
    std::vector<uint8_t> buf(1024);
    std::vector<proton_endpoint_t> peers(num_peers);
    std::size_t out_len = 0;
    std::size_t num_selected_peers = 0;

    proton_status_e status = proton_node.update(time_ms, buf, out_len, peers, num_selected_peers);
    if (status != PROTON_OK) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Could not encode proton message: %s",
        error_to_string(status).c_str()
      );
      // Investigate returning a variant rather than a vector, so that an error may be returned properly
      break;
    } else if (out_len != 0) {
      peers.resize(num_selected_peers);
      buf.resize(out_len);
      to_send.push_back({peers, buf});
    } else {
      // No more data to send
      break;
    }
  }

  return to_send;
}

}  // namespace proton_ros2
