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

#include <protoncpp/node_builder/config.hpp>

namespace proton_ros2
{

ProtonRos2Node::ProtonRos2Node() : rclcpp::Node("proton_ros2")
{
  this->declare_parameter("config_file", rclcpp::PARAMETER_STRING);
  this->declare_parameter("target", rclcpp::PARAMETER_STRING);

  const auto config_file = get_parameter("config_file").as_string();
  const auto target = get_parameter("target").as_string();

  // Proton node builder will throw exceptions from errors in the config,
  // so allow the process to fail early.
  proton_node_ = node_from_config(config_file, target);
}

}  // namespace proton_ros2
