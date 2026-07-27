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

#include <proton_ros2/node.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;

  auto node = std::make_shared<proton_ros2::ProtonRos2Node>();
  auto spin_timer = node->create_wall_timer(
    std::chrono::milliseconds(500),
    [node]() {
      std::vector<proton_ros2::DataForPeers> data_for_peers = node->spin_once(node->now());
      if (!data_for_peers.empty()) {
        RCLCPP_INFO(node->get_logger(), "data for peer received. send to %ld peers",
        data_for_peers.size());
      }
    }
  );
  (void)spin_timer;

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
