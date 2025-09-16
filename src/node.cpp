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
  parseRos2Configs();

  // Create publishers and subscribers
  for (auto config : ros2_configs_) {
    auto& handle = proton_node_.getBundle(config.bundle);

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
 * @brief Parse ros2 section of config file.
 * Read in topics, message types, qos profiles, and matching bundle
 *
 */
void Node::parseRos2Configs()
{
  YAML::Node yaml_node = proton_node_.getConfig().getYamlNode();

  auto ros2_config = yaml_node[proton::ros2::keys::ROS2];
  for (auto topic_config: ros2_config[proton::ros2::keys::TOPICS])
  {
    auto topic = topic_config[proton::ros2::keys::TOPIC].as<std::string>();
    auto message = topic_config[proton::ros2::keys::MESSAGE].as<std::string>();
    auto bundle = topic_config[proton::ros2::keys::BUNDLE].as<std::string>();

    std::string qos;
    try {
      qos = topic_config[proton::ros2::keys::QOS][proton::ros2::keys::PROFILE].as<std::string>();
    }
    catch (const YAML::TypedBadConversion<std::string> &e) {
      qos = proton::ros2::qos::SYSTEM_DEFAULT;
    }

    ros2_configs_.push_back({topic, message, qos, bundle});
  }
}

/**
 * @brief Callback for proton bundle received from a proton peer.
 * Publish directly to ROS 2.
 * @param bundle
 */
void Node::protonCallback(proton::BundleHandle &bundle) {
  //bundle.printBundle();
  publishers_.at(bundle.getName())->publish(bundle);
}

/**
 * @brief Callback for proton bundle received from ROS 2.
 * Send directly to proton peer.
 *
 * @param bundle
 */
void Node::rosCallback(proton::BundleHandle &bundle) {
  RCLCPP_INFO(get_logger(), "ROS 2 Callback");
  bundle.printBundle();
  proton_node_.sendBundle(bundle);
}
