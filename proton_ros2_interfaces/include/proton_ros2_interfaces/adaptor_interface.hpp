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

#ifndef PROTON_ROS2_INTERFACES_ADAPTOR_INTERFACE_HPP
#define PROTON_ROS2_INTERFACES_ADAPTOR_INTERFACE_HPP

#include <memory>
#include <string>

#include <proton_ros2_interfaces/generic_pubsub.hpp>

#include <protoncpp/node_builder/generator.hpp>
#include <proton/registry.h>

#include "rclcpp/rclcpp.hpp"

namespace proton_ros2_interfaces
{

class MessageAdaptorIface {
public:
  virtual ~MessageAdaptorIface() = default;

  /**
   * @brief Unique binding name
   */
  virtual std::string getBindingName() const = 0;

  /**
   * @brief Fully qualified ROS 2 message type (ex: "geometry_msgs/msg/TwistStamped")
   */
  virtual std::string getMessageType() const = 0;

  /**
   * @brief Create publisher: reads from signal registry, serializes to ROS message
   */
  virtual std::unique_ptr<GenericPublisher> createPublisher(
    rclcpp::Node * node, const std::string & topic,
    const rclcpp::QoS & qos, proton_registry_t * registry, SerializeFn serialize) const = 0;

  /**
   * @brief Create subscription: deserializes ROS message, writes to signal registry, optionally triggers bundles
   */
  virtual std::unique_ptr<GenericSubscription> createSubscription(
    rclcpp::Node * node, const std::string & topic, const rclcpp::QoS & qos,
    protoncpp::node_builder::GeneratedNode & proton_node,
    DeserializeAndConvertFn convert) const = 0;
};

}  // namespace proton_ros2_interfaces

#endif  // PROTON_ROS2_INTERFACES_ADAPTOR_INTERFACE_HPP
