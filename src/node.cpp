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

  proton_node_ = proton::Node(config_file_, target_);

  parseRos2Configs();

  for (auto config : ros2_configs_) {
    auto& handle = proton_node_.getBundle(config.bundle);

    // Proton node consumes this bundle, so the ROS node should publish it.
    if (handle.getConsumer() == proton_node_.getTarget()) {
      publishers_.emplace(config.bundle, createTypedPublisher(config.message, this, config.topic, proton::ros2::qos::profiles.at(config.qos)));
      proton_node_.registerCallback(
          config.bundle, std::bind(&Node::protonCallback, this, std::placeholders::_1));
      if (config.topic.at(0) != '/')
      {
        std::cout << "Created publisher: " << get_namespace() << "/" << config.topic << std::endl;
      }
      else
      {
        std::cout << "Created publisher: " << config.topic << std::endl;
      }
    }
    // Proton node produces this bundle, so the ROS node should subscribe to it.
    else if (handle.getProducer() == proton_node_.getTarget()) {
      subscribers_.emplace(
          config.bundle,
          createTypedSubscriber(
              config.message, this, config.topic,
              proton::ros2::qos::profiles.at(config.qos), handle,
              std::bind(&Node::rosCallback, this, std::placeholders::_1)));
      if (config.topic.at(0) != '/')
      {
        std::cout << "Created subscriber: " << get_namespace() << "/" << config.topic << std::endl;
      }
      else
      {
        std::cout << "Created subscriber: " << config.topic << std::endl;
      }
    }
  }

  proton_timer_ =
      create_wall_timer(std::chrono::milliseconds(1),
                        [this]() -> void { proton_node_.spinOnce(); });
}

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
  bundle.printBundle();
  proton_node_.sendBundle(bundle);
}
