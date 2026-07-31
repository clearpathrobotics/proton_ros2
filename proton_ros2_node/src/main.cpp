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

namespace proton_ros2_node
{

// Owns the node and its transports. Ownership flows one direction (App -> node,
// App -> transports) so callbacks can use non-owning raw pointers without
// creating shared_ptr cycles.
class App
{
public:
  App()
  : node_(std::make_shared<proton_ros2::ProtonRos2Node>())
  {
  }

  std::shared_ptr<proton_ros2::ProtonRos2Node> node() const {return node_;}

  void build_transports()
  {
    const auto & proton_config = node_->get_config();
    const auto target_name = node_->get_name();
    const auto & endpoint_config = proton_config.nodes.at(target_name).endpoints;

    for (const auto & ep_id : std::views::keys(endpoint_config)) {
      try {
        auto transport = transport_factory(
          node_->get_logger(), proton_config, target_name, ep_id);
        if (transport == nullptr) {
          RCLCPP_ERROR(node_->get_logger(),
              "transport_factory returned nullptr for %s:%d. Skipping...", target_name.c_str(),
              ep_id);
          continue;
        }
        auto * transport_raw = transport.get();
        auto * node_raw = node_.get();
        transport->set_receive_callback(
          [transport_raw, node_raw](const uint8_t * buf, size_t len) {
            if (transport_raw->receive_and_decode(buf, len) == PROTON_OK) {
              node_raw->recv_bytes(buf, len);
            }
          });
        transports_.push_back(std::move(transport));
      } catch (std::exception & e) {
        RCLCPP_ERROR(node_->get_logger(), "Error constructing transports: %s", e.what());
      }
    }
  }

  void start_spin_timer(std::chrono::milliseconds period)
  {
    // Capture raw pointers only; the timer is owned by the node which is owned
    // by this App, so lifetime is guaranteed and no shared_ptr cycle is formed.
    auto * self = this;
    auto * node_raw = node_.get();
    spin_timer_ = node_->create_wall_timer(
      period,
      [self, node_raw]() {
        const auto data_for_peers = node_raw->spin_once(node_raw->now());
        if (!data_for_peers.empty()) {
          RCLCPP_INFO(
            node_raw->get_logger(), "data for peer received. send to %ld batches",
            data_for_peers.size());
        }
        for (const auto & dfp : data_for_peers) {
          for (const auto & peer : dfp.peers) {
            for (auto & transport : self->transports_) {
              if (peer.node_id == transport->node_id() &&
              peer.endpoint_id == transport->endpoint_id())
              {
                transport->encode_and_send(dfp.data);
              }
            }
          }
        }
      });
  }

private:
  std::shared_ptr<proton_ros2::ProtonRos2Node> node_;
  std::vector<std::unique_ptr<BaseTransport>> transports_;
  rclcpp::TimerBase::SharedPtr spin_timer_;
};

}  // namespace proton_ros2_node

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  proton_ros2_node::App app;
  app.build_transports();
  app.start_spin_timer(std::chrono::milliseconds(500));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(app.node());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
