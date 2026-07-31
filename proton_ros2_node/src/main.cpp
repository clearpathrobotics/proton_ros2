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
 * @author Roni Kreinin (roni.kreinin@rockwellautomation.com)
 */

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <ranges>
#include <vector>

#include <proton_ros2/node.hpp>

#include "proton_ros2_node/transport_factory.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  auto proton_node = std::make_shared<proton_ros2::ProtonRos2Node>();

  const auto proton_config = proton_node->get_config();
  const auto target_name = proton_node->get_name();

  const auto endpoint_config = proton_config.nodes.at(target_name).endpoints;

  std::vector<std::unique_ptr<proton_ros2_node::BaseTransport>> transports;
  for (const auto & ep_id : std::views::keys(endpoint_config)) {
    try {
      std::unique_ptr<proton_ros2_node::BaseTransport> transport =
        proton_ros2_node::transport_factory(proton_node->get_logger(), proton_config, target_name,
        ep_id);
      if (transport != nullptr) {
        transport->set_receive_callback([transport, proton_node](const uint8_t * buf, size_t len){
          if (transport->receive_and_decode(buf, len) == PROTON_OK) {
            // Serial needs to hold on to a vector, so it's going to be a problem to pass the buf/len combo
            proton_node->recv_bytes(buf, len);
          }
        });
        transports.push_back(std::move(transport));
      }
    } catch (std::exception & e) {
      RCLCPP_ERROR(proton_node->get_logger(), "Error constructing transports: %s", e.what());
    }
  }

  auto spin_timer = proton_node->create_wall_timer(
    std::chrono::milliseconds(500),
    [proton_node]() {
      std::vector<proton_ros2::DataForPeers> data_for_peers =
      proton_node->spin_once(proton_node->now());
      if (!data_for_peers.empty()) {
        RCLCPP_INFO(proton_node->get_logger(), "data for peer received. send to %ld peers",
        data_for_peers.size());
      }

      for (const auto & peer : data_for_peers.peers) {
        for (auto & transport : transports) {
          if (peer.node_id == transport->node_id() && peer.endpoint_id == transport->endpoint_id()) {
            transport->encode_and_send(data_for_peers.data);
          }
        }
      }
    }
  );
  (void)spin_timer;

  executor.add_node(proton_node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
