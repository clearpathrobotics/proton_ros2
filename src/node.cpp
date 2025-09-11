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
#include "proton_ros2/conversions/factory.hpp"
#include "yaml-cpp/yaml.h"
#include <chrono>
#include <iostream>

using namespace proton::ros2;

Node::Node() : rclcpp::Node("proton_ros2") {
  declare_parameter(
      "config_file",
      "/home/rkreinin/proto_ws/src/proton_ros2/examples/j100/j100.yaml");
  declare_parameter("target", "pc");

  config_file_ = get_parameter("config_file").as_string();
  target_ = get_parameter("target").as_string();

  proton_node_ = proton::Node(config_file_, target_);

  YAML::Node yaml_node = proton_node_.getConfig().getYamlNode();

  for (auto bundle : yaml_node[proton::keys::BUNDLES]) {
    try {
      auto name = bundle[proton::keys::NAME].as<std::string>();
      auto message =
          bundle[proton::ros2::keys::ROS2][proton::ros2::keys::MESSAGE]
              .as<std::string>();
      auto topic = bundle[proton::ros2::keys::ROS2][proton::ros2::keys::TOPIC]
                       .as<std::string>();

      auto& handle = proton_node_.getBundle(name);

      // Proton node consumes this bundle, so the ROS node should publish it.
      if (handle.getConsumer() == proton_node_.getTarget()) {
        publishers_.emplace(name, createTypedPublisher(message,this, topic, rclcpp::SensorDataQoS()));
        proton_node_.registerCallback(
            name, std::bind(&Node::protonCallback, this, std::placeholders::_1));
        std::cout << "Created " << name << " publisher with ros message " << message << " on topic " << topic << std::endl;
      }
      // Proton node produces this bundle, so the ROS node should subscribe to it.
      else if (handle.getProducer() == proton_node_.getTarget()) {
        subscribers_.emplace(
            name,
            createTypedSubscriber(
                message, this, topic,
                rclcpp::SensorDataQoS(), proton_node_.getBundle(name),
                std::bind(&Node::rosCallback, this, std::placeholders::_1)));
        std::cout << "Created " << name << " subscriber with ros message " << message << " on topic " << topic << std::endl;
      }

    } catch (const YAML::TypedBadConversion<std::string> &e) {
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
