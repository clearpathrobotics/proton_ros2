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

#include <string>
#include <unordered_map>
#include <vector>

#include <protoncpp/node_builder/config.hpp>
#include <protoncpp/node_builder/generator.hpp>

#include "rclcpp/rclcpp.hpp"

namespace proton_ros2
{

/**
 * @class ProtonRos2Node
 *
 * Central class for proton ROS 2 node
 */
class ProtonRos2Node : public rclcpp::Node
{
public:
  ProtonRos2Node();
};

}  // namespace proton_ros2

#endif  // PROTON_ROS2_NODE_HPP
