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
struct convert<proton::ros2::QosConfig> {
  static bool decode(const Node& node, proton::ros2::QosConfig& rhs) {
    if(!node.IsDefined() || node.IsNull()) {
      rhs.profile = proton::ros2::qos::profiles::SERVICES;
      return true;
    }

    // String representing a standard QoS profile
    if (node.IsScalar())
    {
      rhs.profile = node.as<std::string>();
    }
    // Map representing a custome QoS profile
    else if (node.IsMap())
    {
      // History
      if (node[proton::ros2::keys::HISTORY])
      {
        rhs.history = node[proton::ros2::keys::HISTORY].as<std::string>();
      }
      else
      {
        rhs.history = proton::ros2::qos::history::SYSTEM_DEFAULT;
      }

      // Depth
      if (node[proton::ros2::keys::DEPTH])
      {
        rhs.depth = node[proton::ros2::keys::DEPTH].as<size_t>();
      }
      else
      {
        rhs.depth = 10U;
      }

      // Reliability
      if (node[proton::ros2::keys::RELIABILITY])
      {
        rhs.reliability = node[proton::ros2::keys::RELIABILITY].as<std::string>();
      }
      else
      {
        rhs.reliability = proton::ros2::qos::reliability::SYSTEM_DEFAULT;
      }

      // Durability
      if (node[proton::ros2::keys::DURABILITY])
      {
        rhs.durability = node[proton::ros2::keys::DURABILITY].as<std::string>();
      }
      else
      {
        rhs.durability = proton::ros2::qos::durability::SYSTEM_DEFAULT;
      }
    }

    return true;
  }
};

template<>
struct convert<proton::ros2::TopicConfig> {
  static bool decode(const Node& node, proton::ros2::TopicConfig& rhs) {
    if(!node.IsDefined() || node.IsNull()) {
      return false;
    }

    rhs.topic = node[proton::ros2::keys::TOPIC].as<std::string>();
    rhs.message = node[proton::ros2::keys::MESSAGE].as<std::string>();
    rhs.bundle = node[proton::ros2::keys::BUNDLE].as<std::string>();
    rhs.qos = node[proton::ros2::keys::QOS].as<proton::ros2::QosConfig>();

    return true;
  }
};

template<>
struct convert<proton::ros2::ServiceConfig> {
  static bool decode(const Node& node, proton::ros2::ServiceConfig& rhs) {
    if(!node.IsDefined() || node.IsNull()) {
      return false;
    }

    rhs.topic = node[proton::ros2::keys::TOPIC].as<std::string>();
    rhs.service = node[proton::ros2::keys::SERVICE].as<std::string>();
    rhs.request = node[proton::ros2::keys::REQUEST].as<std::string>();
    rhs.qos = node[proton::ros2::keys::QOS].as<proton::ros2::QosConfig>();

    auto response_node = node[proton::ros2::keys::RESPONSE];
    if (response_node.IsDefined() && !response_node.IsNull())
    {
      rhs.response = response_node.as<std::string>();
    }
    else
    {
      rhs.response = "";
    }

    auto timeout_node = node[proton::ros2::keys::TIMEOUT];
    if (timeout_node.IsDefined() && !timeout_node.IsNull())
    {
      rhs.timeout = timeout_node.as<uint32_t>();
    }
    else
    {
      rhs.timeout = proton::ros2::Node::DEFAULT_TIMEOUT_MS;
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

    for (auto service: node[proton::ros2::keys::SERVICES])
    {
      rhs.services.push_back(service.as<proton::ros2::ServiceConfig>());
    }

    return true;
  }
};

}

using namespace proton::ros2;

Node::Node() : rclcpp::Node("proton_ros2"), updater_(this) {
  declare_parameter(
      "config_file",
      "/home/rkreinin/proto_ws/src/proton_ros2/examples/j100/j100.yaml");
  declare_parameter("target", "pc");

  config_file_ = get_parameter("config_file").as_string();
  target_ = get_parameter("target").as_string();

  // Create Proton Node
  proton_node_ = std::make_unique<proton::Node>(config_file_, target_, true, false);

  // Parse ROS 2 config
  YAML::Node yaml_node = proton_node_->getConfig().getYamlNode();
  ros2_config_ = yaml_node[proton::ros2::keys::ROS2].as<ROS2Config>();

  // Create publishers and subscribers
  for (auto config : ros2_config_.topics) {
    auto& handle = proton_node_->getBundle(config.bundle);
    auto consumers = handle.getConsumers();
    auto producers = handle.getProducers();

    bool handle_consumer = std::find(consumers.begin(), consumers.end(), proton_node_->getName()) != consumers.end();
    bool handle_producer = std::find(producers.begin(), producers.end(), proton_node_->getName()) != producers.end();

    // Proton node consumes this bundle, so the ROS node should publish it.
    if (handle_consumer) {
      auto pub = Factory::createTypedPublisher(config.message, this, config.topic, getQoS(config.qos));
      publishers_.emplace(config.bundle, pub);
      proton_node_->registerCallback(
          config.bundle, std::bind(&Node::protonCallback, this, std::placeholders::_1));

      RCLCPP_INFO(get_logger(), "Created publisher %s",
        rclcpp::expand_topic_or_service_name(
          pub->getTopic(),
          get_name(),
          get_namespace()).c_str());
    }
    // Proton node produces this bundle, so the ROS node should subscribe to it.
    else if (handle_producer) {
      auto sub = Factory::createTypedSubscriber(
                  config.message, this, config.topic,
                  getQoS(config.qos), handle,
                  std::bind(&Node::rosCallback, this, std::placeholders::_1));
      subscribers_.emplace(config.bundle, sub);
      RCLCPP_INFO(get_logger(), "Created subscriber %s",
        rclcpp::expand_topic_or_service_name(
          sub->getTopic(),
          get_name(),
          get_namespace()).c_str());
    }
  }

  // Create services
  for (auto config : ros2_config_.services) {
    auto& request_handle = proton_node_->getBundle(config.request);
    auto request_consumers = request_handle.getConsumers();
    auto request_producers = request_handle.getProducers();
    bool request_consumer = std::find(request_consumers.begin(), request_consumers.end(), proton_node_->getName()) != request_consumers.end();
    bool request_producer = std::find(request_producers.begin(), request_producers.end(), proton_node_->getName()) != request_producers.end();

    if (config.response != "")
    {
      auto &response_handle = proton_node_->getBundle(config.response);
      auto response_consumers = response_handle.getConsumers();
      auto response_producers = response_handle.getProducers();
      bool response_consumer = std::find(response_consumers.begin(), response_consumers.end(), proton_node_->getName()) != response_consumers.end();
      bool response_producer = std::find(response_consumers.begin(), response_consumers.end(), proton_node_->getName()) != response_producers.end();

      if (request_producer && response_consumer)
      {
        auto srv = Factory::createTypedService(
          config.service,
          this,
          config.topic,
          getQoS(config.qos),
          config.timeout,
          request_handle,
          response_handle,
          std::bind(&Node::rosCallback, this, std::placeholders::_1));
        // Register the response callback
        proton_node_->registerCallback(config.response, std::bind(&IService::responseCallback, srv, std::placeholders::_1));
        services_.emplace(config.request, srv);

        RCLCPP_INFO(get_logger(), "Created service %s",
        rclcpp::expand_topic_or_service_name(
          srv->getServiceName(),
          get_name(),
          get_namespace()).c_str());
      }
      else if (request_consumer && response_producer)
      {
        auto client = Factory::createTypedClient(
          config.service,
          this,
          config.topic,
          getQoS(config.qos),
          config.timeout,
          response_handle,
          std::bind(&Node::rosCallback, this, std::placeholders::_1)
        );

        // Add request bundle to clients list
        clients_.emplace(config.request, client);

        // Register request bundle to proton callbacks
        proton_node_->registerCallback(
          config.request, std::bind(&Node::protonCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Created client %s",
          rclcpp::expand_topic_or_service_name(
            client->getServiceName(),
            get_name(),
            get_namespace()).c_str());
      }
      else
      {
        throw std::runtime_error("Invalid request and response bundle for service " + config.service);
      }
    }
    else {
      if (request_producer)
      {
        auto srv = Factory::createTypedService(
          config.service,
          this,
          config.topic,
          getQoS(config.qos),
          config.timeout,
          request_handle,
          std::bind(&Node::rosCallback, this, std::placeholders::_1));

        services_.emplace(config.request, srv);

        RCLCPP_INFO(get_logger(), "Created service %s",
        rclcpp::expand_topic_or_service_name(
          srv->getServiceName(),
          get_name(),
          get_namespace()).c_str());
      }
      else if (request_consumer)
      {
        auto client = Factory::createTypedClient(
          config.service,
          this,
          config.topic,
          getQoS(config.qos),
          config.timeout
        );

        // Add request bundle to clients list
        clients_.emplace(config.request, client);

        // Register request bundle to proton callbacks
        proton_node_->registerCallback(
          config.request, std::bind(&Node::protonCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Created client %s",
        rclcpp::expand_topic_or_service_name(
          client->getServiceName(),
          get_name(),
          get_namespace()).c_str());
      }
      else
      {
        throw std::runtime_error("Invalid request bundle for service " + config.service);
      }
    }
  }

  // Setup diagnostics
  updater_.setHardwareID(target_);

  // Proton node diagnostics
  updater_.add("Proton Statistics", this, &Node::nodeDiagnostic);

  // Bundle diagnostics
  for (auto &[name, handle]: proton_node_->getBundleMap())
  {
    updater_.add(name, std::bind(&Node::bundleDiagnostic, this, std::placeholders::_1, handle.getName()));
  }

  // Heartbeat diagnostics
  for (auto& [name, node]: proton_node_->getConfig().getNodes())
  {
    if (node.name != target_ && node.heartbeat.enabled)
    {
      updater_.add(node.name, std::bind(&Node::heartbeatDiagnostic, this, std::placeholders::_1, node.name));
    }
  }

  proton_node_->activate();
  proton_node_->startStatsThread();

  proton_thread_ = std::thread(std::bind(&proton::Node::spin, proton_node_.get()));
}

/**
 * @brief Callback for proton bundle received from a proton peer.
 * Publish directly to ROS 2.
 * @param bundle
 */
void Node::protonCallback(proton::BundleHandle &bundle) {
  if (publishers_.find(bundle.getName()) != publishers_.end())
  {
    publishers_.at(bundle.getName())->publish(bundle);
  }
  else if (clients_.find(bundle.getName()) != clients_.end())
  {
    clients_.at(bundle.getName())->sendRequest(bundle);
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Invalid bundle in protonCallback %s", bundle.getName().c_str());
  }
}

/**
 * @brief Callback for proton bundle received from ROS 2.
 * Send directly to proton peer.
 *
 * @param bundle
 */
void Node::rosCallback(proton::BundleHandle &bundle) {
  proton_node_->sendBundle(bundle);
}

/**
 * @brief Get QoS profile from config
 *
 * @param config
 * @return rclcpp::QoS
 */
rclcpp::QoS Node::getQoS(QosConfig config)
{
  if (config.profile != "")
  {
    return proton::ros2::qos::profiles::profiles.at(config.profile);
  }
  else
  {
    rmw_qos_profile_t profile = {
      proton::ros2::qos::history::policies.at(config.history),
      config.depth,
      proton::ros2::qos::reliability::policies.at(config.reliability),
      proton::ros2::qos::durability::policies.at(config.durability),
      RMW_QOS_DEADLINE_DEFAULT,
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
      RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
      false
    };

    return rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(profile), profile);
  }
}

void Node::nodeDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Proton Node statistics");
  stat.add("Config", proton_node_->getConfig().getName());
  stat.add("Node", target_);
  for (auto& [name, connection] : proton_node_->getConnections())
  {
    stat.add(name + " Rx", connection.getRxKbps());
    stat.add(name + " Tx", connection.getTxKbps());
  }
}

void Node::bundleDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string& bundle_name)
{
  auto & handle = proton_node_->getBundle(bundle_name);
  auto consumers = handle.getConsumers();
  auto producers = handle.getProducers();
  std::string producers_string = "[";
  std::string consumers_string = "[";

  stat.add("Bundle", handle.getName());

  std::size_t i = 0;
  for (auto& p: producers)
  {
    if (i++ < producers.size() - 1)
    {
      producers_string += p + ", ";
    }
    else
    {
      producers_string += p + "]";
    }
  }
  stat.add("Producers", producers_string);

  i = 0;
  for (auto& c: consumers)
  {
    if (i++ < consumers.size() - 1)
    {
      consumers_string += c + ", ";
    }
    else
    {
      consumers_string += c + "]";
    }
  }
  stat.add("Consumers", consumers_string);

  if (std::find(consumers.begin(), consumers.end(), proton_node_->getName()) != consumers.end())
  {
    stat.add("Frequency", handle.getRxps());
  }
  else if (std::find(producers.begin(), producers.end(), proton_node_->getName()) != producers.end())
  {
    stat.add("Frequency", handle.getTxps());
  }

  stat.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, handle.getName() + " statistics");
}

void Node::heartbeatDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string& producer)
{
  auto& handle = proton_node_->getHeartbeat(producer);
  uint32_t hz = handle.getRxps();

  proton::NodeConfig node_config = proton_node_->getConfig().getNodes().at(producer);

  if (hz != (1000 / node_config.heartbeat.period))
  {
    stat.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Heartbeat does not match period");
  }
  else
  {
    stat.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Heartbeat OK");
  }

  stat.add("Frequency", handle.getRxps());
}
