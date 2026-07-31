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

#include <proton/common.h>
#include <protoncpp/node_builder/config.hpp>
#include <protoncpp/node_access.hpp>

namespace proton_ros2
{

ProtonRos2Node::ProtonRos2Node()
: rclcpp::Node("proton_ros2")
  , plugin_loader_(get_logger())
{
  declare_parameter("proton_config_file", rclcpp::PARAMETER_STRING);
  declare_parameter("binding_config_file", rclcpp::PARAMETER_STRING);
  declare_parameter("target", rclcpp::PARAMETER_STRING);

  const auto proton_config_file = get_parameter("proton_config_file").as_string();
  const auto binding_config_file = get_parameter("binding_config_file").as_string();
  const auto target = get_parameter("target").as_string();

  // Proton node builder will throw exceptions from errors in the config,
  // so allow the process to fail early.
  try {
    proton_config_ = get_filtered_proton_config(proton_config_file, target);
    proton_node_ = proton::node_builder::GeneratedNode(proton_config_, target);
  } catch (proton::node_builder::NodeBuilderException & e) {
    RCLCPP_FATAL(
      get_logger(),
      "Proton configuration error: %s", e.what()
    );
    throw;
  } catch (std::exception & e) {
    RCLCPP_FATAL(
      get_logger(),
      "proton_ros2 encountered an error: %s", e.what()
    );
    throw;
  }

  // Get names and ID's of bundles in our config
  const auto bundle_name_to_id = get_bundles(proton_config_file, target);

  // Load runtime config for message bindings
  const auto binding_config = parse_binding_config(get_logger(), binding_config_file);

  plugin_loader_.load_plugins(binding_config.adaptor_packages);

  // Validate that adaptor bindings exist
  for (const auto & pub : binding_config.publishers) {
    if (!plugin_loader_.has_binding(pub.binding)) {
      throw std::runtime_error(
        "Publisher on '" + pub.topic + "' references unknown binding: '" + pub.binding + "'"
      );
    }
  }
  for (const auto & sub : binding_config.subscribers) {
    if (!plugin_loader_.has_binding(sub.binding)) {
      throw std::runtime_error(
        "Subscriber on '" + sub.topic + "' references unknown binding: '" + sub.binding + "'"
      );
    }
  }

  for (const auto & pub : binding_config.publishers) {
    auto adaptor = plugin_loader_.get_by_binding(pub.binding);

    RCLCPP_INFO(
      get_logger(),
      "Creating publisher on topic %s with binding %s (bundle=%s)",
      pub.topic.c_str(), pub.binding.c_str(), pub.bundle.c_str()
    );

    auto publisher = adaptor->create_publisher(
      this, pub.topic, pub.qos.profile, proton_node_.registry(),
      pub.bundle
    );

    publishers_.push_back(std::move(publisher));
  }

  // Aggregate publishers by bundle name and shared ptr for callback insertion
  std::unordered_map<
    std::string,
    std::shared_ptr<proton_ros2_interfaces::GenericPublisher>
  > bundle_to_publishers;
  for (size_t i = 0; i < binding_config.publishers.size(); ++i) {
    const auto bundle_name = binding_config.publishers[i].bundle;

    if (bundle_to_publishers.contains(bundle_name)) {
      RCLCPP_WARN(get_logger(),
      "Bundle '%s' has already been found, there is a 1:1 binding between bundles and publishers",
        bundle_name.c_str()
      );
      continue;
    }
    bundle_to_publishers[bundle_name] = publishers_[i];
  }

  proton::NodeAccess node_access(proton_node_.node());
  for (const auto & [bundle_name, pub_ptr] : bundle_to_publishers) {
    if (!bundle_name_to_id.contains(bundle_name)) {
      RCLCPP_WARN(
        get_logger(),
        "Publisher references unknown bundle '%s' - no callback will be registered",
        bundle_name.c_str()
      );
      continue;
    }

    // Weak ptr here will survive post-teardown of the proton_node, which means
    // the intentionally-leaked callbacks will not call a nullptr.
    std::weak_ptr<proton_ros2_interfaces::GenericPublisher> weak_pub = pub_ptr;
    node_access.on_bundle_update(
      bundle_name_to_id.at(bundle_name),
      [weak_pub](uint32_t, const uint32_t *, size_t) {
        if (auto p = weak_pub.lock()) {
          p->publish();
        }
      }
    );
  }

  for (const auto & sub : binding_config.subscribers) {
    auto adaptor = plugin_loader_.get_by_binding(sub.binding);

    RCLCPP_INFO(
      get_logger(),
      "Creating subscriber on topic %s with binding %s (bundle=%s)",
      sub.topic.c_str(), sub.binding.c_str(), sub.bundle.c_str()
    );

    auto subscriber = adaptor->create_subscription(
      this, sub.topic, sub.qos.profile, proton_node_,
      sub.bundle
    );

    subscribers_.push_back(std::move(subscriber));
  }
}

void ProtonRos2Node::recv_bytes(const uint8_t * buf, std::size_t len)
{
  proton_status_e status = proton::NodeAccess(proton_node_.node()).receive(buf, len);

  if (status != PROTON_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "Proton reception error: %s",
      proton_status_to_string(status)
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
        get_logger(),
        "Could not encode proton message: %s",
        proton_status_to_string(status)
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
