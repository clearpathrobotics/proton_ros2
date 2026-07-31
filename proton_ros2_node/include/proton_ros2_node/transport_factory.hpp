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

#ifndef PROTON_ROS2_NODE_TRANSPORT_FACTORY_HPP
#define PROTON_ROS2_NODE_TRANSPORT_FACTORY_HPP

#include <cstdint>
#include <memory>
#include <string>

#include <protoncpp/node_builder/config.hpp>

#include "proton_ros2_node/base_transport.hpp"

#include "rclcpp/rclcpp.hpp"

namespace proton_ros2_node
{

using Config = proton::node_builder::Config;

/**
 * @brief get a transport for a peer from a given host and its corresponding endpoint ID
 *
 * IE: If there is a connection between node A, endpoint 0, and node B, endpoint 1, this factory
 *     will return that connection if given "node A", 0.
 *
 * @return pointer to transport, already configured
 *
 * @throws std::runtime error if the endpoints are not present, or if they are malformed.
 */
std::unique_ptr<BaseTransport> transport_factory(rclcpp::Logger logger, const Config & config, const std::string & node_name, uint32_t endpoint_id);

}  // namespace proton_ros2_node

#endif  // PROTON_ROS2_NODE_TRANSPORT_FACTORY_HPP
