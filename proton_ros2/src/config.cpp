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

#include <stdexcept>

#include "proton_ros2/config.hpp"

#include <protoncpp/node_builder/config.hpp>

namespace proton_ros2
{

static QosConfig parse_qos(const proton::node_builder::ConfigNode & qos_node)
{
  std::string profile_str = qos_node["profile"].is_defined() ? qos_node["profile"].as_string() : "";
  std::string history_str = qos_node["history"].is_defined() ? qos_node["history"].as_string() : "";
  std::string durability_str =
    qos_node["durability"].is_defined() ? qos_node["durability"].as_string() : "";
  std::string reliability_str =
    qos_node["reliability"].is_defined() ? qos_node["reliability"].as_string() : "";
  size_t depth = qos_node["depth"].is_defined() ? qos_node["depth"].as_uint32() : 0;

  qos::QoSProfile profile = qos::parse_qos_profile(profile_str).value_or(qos::QoSProfile::Default);
  qos::QoSHistory history =
    qos::parse_qos_history(history_str).value_or(qos::QoSHistory::SystemDefault);
  qos::QoSReliability reliability =
    qos::parse_qos_reliability(reliability_str).value_or(qos::QoSReliability::SystemDefault);
  qos::QoSDurability durability =
    qos::parse_qos_durability(durability_str).value_or(qos::QoSDurability::SystemDefault);

  QosConfig qos {
    .profile = get_qos_profile(profile),
    .history = get_qos_history(history),
    .depth = depth,
    .reliability = get_qos_reliability(reliability),
    .durability = get_qos_durability(durability),
  };

  return qos;
}

proton::node_builder::GeneratedNode node_from_config(
  const std::string & config_path,
  const std::string & target_name)
{
  using namespace proton::node_builder;

  const auto config_tree = ConfigTree::from_yaml_file(config_path);
  const auto proton_config = Config(config_tree);
  const auto filtered_config = filter_for_target(proton_config, target_name);

  return GeneratedNode(filtered_config, target_name);
}

ProtonRos2Config parse_binding_config(rclcpp::Logger logger, const std::string & config_path)
{
  ProtonRos2Config runtime_config;

  RCLCPP_INFO(logger, "Creating binding config from %s", config_path.c_str());

  const auto config_tree = proton::node_builder::ConfigTree::from_yaml_file(config_path);

  RCLCPP_INFO(logger, "Config tree created. Pub size: %zu, sub size: %zu",
    config_tree["publishers"].size(), config_tree["subscribers"].size()
  );

  const auto publishers_node = config_tree["publishers"];
  if (publishers_node.is_sequence()) {
    for (const auto & pub : publishers_node) {
      std::vector<std::string> trigger_bundles;
      if (pub["trigger_bundles"].is_sequence()) {
        for (const auto & bundle : pub["trigger_bundles"]) {
          trigger_bundles.push_back(bundle.as_string());
        }
      } else {
        throw std::runtime_error("Config 'trigger_bundles' is not a list");
      }

      TopicConfig pub_config {
        .topic = pub["topic"].as_string(),
        .binding = pub["binding"].as_string(),
        .bundles = trigger_bundles,
        .qos = parse_qos(pub["qos"]),
      };

      runtime_config.publishers.push_back(pub_config);
    }
  } else {
    throw std::runtime_error("Config 'publishers' is not a list");
  }

  const auto subscribers_node = config_tree["subscribers"];
  if (subscribers_node.is_sequence()) {
    for (const auto & sub : subscribers_node) {
      std::vector<std::string> target_bundles;
      if (sub["target_bundles"].is_sequence()) {
        for (const auto & bundle : sub["target_bundles"]) {
          target_bundles.push_back(bundle.as_string());
        }
      } else {
        throw std::runtime_error("Config 'target_bundles' is not a list");
      }

      TopicConfig sub_config {
        .topic = sub["topic"].as_string(),
        .binding = sub["binding"].as_string(),
        .bundles = target_bundles,
        .qos = parse_qos(sub["qos"]),
      };

      runtime_config.subscribers.push_back(sub_config);
    }
  } else {
    throw std::runtime_error("Config 'subscribers' is not a list");
  }

  const auto adaptors_node = config_tree["adaptor_packages"];
  if (adaptors_node.is_sequence()) {
    for (const auto & pkg : adaptors_node) {
      runtime_config.adaptor_packages.push_back(pkg.as_string());
    }
  } else {
    throw std::runtime_error("config 'adaptor_packages' is not a list");
  }

  return runtime_config;
}

}  // namespace proton_ros2
