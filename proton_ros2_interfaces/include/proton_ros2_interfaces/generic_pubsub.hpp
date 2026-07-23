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

#ifndef PROTON_ROS2_INTERFACES_GENERIC_PUBSUB_HPP
#define PROTON_ROS2_INTERFACES_GENERIC_PUBSUB_HPP

#include <algorithm>
#include <functional>
#include <string>

#include <protoncpp/node_builder/generator.hpp>
#include <protoncpp/registry_lock.hpp>
#include <proton/registry.h>

#include "rclcpp/rclcpp.hpp"

namespace proton_ros2_interfaces
{

/// Serialization function: registry → serialized ROS message
using SerializeFn = std::function<rclcpp::SerializedMessage(proton_registry_t *)>;

/// Deserialization + conversion function: serialized → registry signals
using DeserializeAndConvertFn = std::function<void(const rclcpp::SerializedMessage &,
    proton::node_builder::GeneratedNode &)>;

class GenericPublisher {
public:
  GenericPublisher(
    rclcpp::Node * node, const std::string & topic,
    const std::string & msg_type, const rclcpp::QoS & qos,
    proton_registry_t * registry, const std::vector<std::string> & trigger_bundles,
    SerializeFn serialize)
  : registry_(registry), trigger_bundles_(trigger_bundles), serialize_(std::move(serialize))
  {
    pub_ = node->create_generic_publisher(topic, msg_type, qos);
  }

  void publish(const std::string & bundle_name)
  {
    if (std::find(trigger_bundles_.begin(), trigger_bundles_.end(), bundle_name) != trigger_bundles_.end()) {
      pub_->publish(serialize_(registry_));
    }
  }

private:
  rclcpp::GenericPublisher::SharedPtr pub_;
  proton_registry_t * registry_;
  std::vector<std::string> trigger_bundles_;
  SerializeFn serialize_;
};

class GenericSubscription {
public:
  GenericSubscription(
    rclcpp::Node * node, const std::string & topic,
    const std::string & msg_type, const rclcpp::QoS & qos,
    proton::node_builder::GeneratedNode & proton_node,
    const std::vector<std::string> & target_bundles,
    DeserializeAndConvertFn convert) : target_bundles_(target_bundles)
  {
    sub_ = node->create_generic_subscription(topic, msg_type, qos,
        [&proton_node, convert = std::move(convert)]
        (std::shared_ptr<rclcpp::SerializedMessage> msg) {
          proton::ScopedLock lock(proton_node.registry());
          convert(*msg, proton_node);
      });
  }

private:
  rclcpp::GenericSubscription::SharedPtr sub_;
  std::vector<std::string> target_bundles_;
};

}  // namespace proton_ros2_interfaces

#endif  // PROTON_ROS2_INTERFACES_GENERIC_PUBSUB_HPP
