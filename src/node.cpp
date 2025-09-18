/**
 * Software License Agreement (proprietary)
 *
 * @copyright Copyright (c) 2025 Clearpath Robotics, Inc., All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, is not permitted without the express permission of Clearpath
 * Robotics.
 *
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#include "proton_ros2/node.hpp"
#include "yaml-cpp/yaml.h"
#include <chrono>
#include <iostream>

namespace YAML {

template<>
struct convert<proton::ros2::TopicConfig> {
  static bool decode(const Node& node, proton::ros2::TopicConfig& rhs) {
    if(!node.IsDefined() || node.IsNull()) {
      return false;
    }

    rhs.topic = node[proton::ros2::keys::TOPIC].as<std::string>();
    rhs.message = node[proton::ros2::keys::MESSAGE].as<std::string>();
    rhs.bundle = node[proton::ros2::keys::BUNDLE].as<std::string>();

    auto qos_profile_node = node[proton::ros2::keys::QOS_PROFILE];
    if (qos_profile_node.IsDefined() && !qos_profile_node.IsNull())
    {
      rhs.qos = qos_profile_node.as<std::string>();
    }
    else
    {
      rhs.qos = proton::ros2::qos::SYSTEM_DEFAULT;
    }

    return true;
  }
};

template<>
struct convert<proton::ros2::ROS2Config> {
  static bool decode(const Node& node, proton::ros2::ROS2Config& rhs) {
    if(!node.IsDefined() || node.IsNull()) {
      return false;
    }

    for (auto topic: node[proton::ros2::keys::TOPICS])
    {
      rhs.topics.push_back(topic.as<proton::ros2::TopicConfig>());
    }

    return true;
  }
};

}


using namespace proton::ros2;

Node::Node() : rclcpp::Node("proton_ros2") {
  declare_parameter(
      "config_file",
      "/home/rkreinin/proto_ws/src/proton_ros2/examples/a300/a300.yaml");
  declare_parameter("target", "pc");

  config_file_ = get_parameter("config_file").as_string();
  target_ = get_parameter("target").as_string();

  // Create Proton Node
  proton_node_ = proton::Node(config_file_, target_);

  // Parse ROS 2 config
  YAML::Node yaml_node = proton_node_.getConfig().getYamlNode();
  ros2_config_ = yaml_node[proton::ros2::keys::ROS2].as<ROS2Config>();

  // Create publishers and subscribers
  for (auto config : ros2_config_.topics) {
    auto& handle = proton_node_.getBundle(config.bundle);
    std::cout << config.qos << std::endl;

    // Proton node consumes this bundle, so the ROS node should publish it.
    if (handle.getConsumer() == proton_node_.getTarget()) {
      auto pub = Factory::createTypedPublisher(config.message, this, config.topic, proton::ros2::qos::profiles.at(config.qos));
      publishers_.emplace(config.bundle, pub);
      proton_node_.registerCallback(
          config.bundle, std::bind(&Node::protonCallback, this, std::placeholders::_1));

      RCLCPP_INFO(get_logger(), "Created publisher %s",
        rclcpp::expand_topic_or_service_name(
          pub->getTopic(),
          get_name(),
          get_namespace()).c_str());
    }
    // Proton node produces this bundle, so the ROS node should subscribe to it.
    else if (handle.getProducer() == proton_node_.getTarget()) {
      auto sub = Factory::createTypedSubscriber(
                  config.message, this, config.topic,
                  proton::ros2::qos::profiles.at(config.qos), handle,
                  std::bind(&Node::rosCallback, this, std::placeholders::_1));
      subscribers_.emplace(config.bundle, sub);
      RCLCPP_INFO(get_logger(), "Created subscriber %s",
        rclcpp::expand_topic_or_service_name(
          sub->getTopic(),
          get_name(),
          get_namespace()).c_str());
    }
  }

  proton_timer_ =
      create_wall_timer(std::chrono::milliseconds(1),
                        [this]() -> void { proton_node_.spinOnce(); });
}

/**
 * @brief Callback for proton bundle received from a proton peer.
 * Publish directly to ROS 2.
 * @param bundle
 */
void Node::protonCallback(proton::BundleHandle &bundle) {
  publishers_.at(bundle.getName())->publish(bundle);
}

/**
 * @brief Callback for proton bundle received from ROS 2.
 * Send directly to proton peer.
 *
 * @param bundle
 */
void Node::rosCallback(proton::BundleHandle &bundle) {
  proton_node_.sendBundle(bundle);
}
