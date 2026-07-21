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
 * @author Roni Kreinin (roni.kreinin@rockwellautomation.com)
 */

#ifndef INC_PROTON_ROS2_NODE_HPP_
#define INC_PROTON_ROS2_NODE_HPP_

#include <map>

#include "protoncpp/proton.hpp"
#include "rclcpp/rclcpp.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "proton_ros2/typed.hpp"
#include "proton_ros2/factory.hpp"

namespace proton::ros2 {

namespace keys {
  static const char *const ROS2 = "ros2";
  static const char *const TOPICS = "topics";
  static const char *const SERVICES = "services";
  static const char *const TOPIC = "topic";
  static const char *const MESSAGE = "message";
  static const char *const SERVICE = "service";
  static const char *const BUNDLE = "bundle";
  static const char *const REQUEST = "request";
  static const char *const RESPONSE = "response";
  static const char *const TIMEOUT = "timeout";
  static const char *const QOS = "qos";
  static const char *const PROFILE = "profile";
  static const char *const HISTORY = "history";
  static const char *const DEPTH = "depth";
  static const char *const RELIABILITY = "reliability";
  static const char *const DURABILITY = "durability";
}

namespace qos {
  namespace profiles {
    inline static const std::string DEFAULT = "default";
    inline static const std::string SYSTEM_DEFAULTS = "system_defaults";
    inline static const std::string SENSOR_DATA = "sensor_data";
    inline static const std::string SERVICES = "services";
    inline static const std::string ROSOUT = "rosout";

    const std::map<std::string, rclcpp::QoS> profiles = {
      {DEFAULT, rclcpp::QoS(10)},
      {SYSTEM_DEFAULTS, rclcpp::SystemDefaultsQoS()},
      {SENSOR_DATA, rclcpp::SensorDataQoS()},
      {SERVICES, rclcpp::ServicesQoS()},
      {ROSOUT, rclcpp::RosoutQoS()},
    };
  }

  namespace history {
    inline static const std::string SYSTEM_DEFAULT = "system_default";
    inline static const std::string KEEP_LAST = "keep_last";
    inline static const std::string KEEP_ALL = "keep_all";

    const std::map<std::string, rmw_qos_history_policy_t> policies = {
      {SYSTEM_DEFAULT, RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT},
      {KEEP_LAST, RMW_QOS_POLICY_HISTORY_KEEP_LAST},
      {KEEP_ALL, RMW_QOS_POLICY_HISTORY_KEEP_ALL},
    };
  }

  namespace reliability {
    inline static const std::string SYSTEM_DEFAULT = "system_default";
    inline static const std::string RELIABLE = "reliable";
    inline static const std::string BEST_EFFORT = "best_effort";

    const std::map<std::string, rmw_qos_reliability_policy_t> policies = {
      {SYSTEM_DEFAULT, RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT},
      {RELIABLE, RMW_QOS_POLICY_RELIABILITY_RELIABLE},
      {BEST_EFFORT, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT},
    };
  }

  namespace durability {
    inline static const std::string SYSTEM_DEFAULT = "system_default";
    inline static const std::string TRANSIENT_LOCAL = "transient_local";
    inline static const std::string VOLATILE = "volatile";

    const std::map<std::string, rmw_qos_durability_policy_t> policies = {
      {SYSTEM_DEFAULT, RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT},
      {TRANSIENT_LOCAL, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL},
      {VOLATILE, RMW_QOS_POLICY_DURABILITY_VOLATILE},
    };
  }
}

typedef struct {
  std::string profile;
  std::string history;
  size_t depth;
  std::string reliability;
  std::string durability;
} QosConfig;

struct TopicConfig {
  std::string topic;
  std::string message;
  std::string bundle;
  QosConfig qos;
};

struct ServiceConfig {
  std::string topic;
  std::string service;
  QosConfig qos;
  std::string request;
  std::string response;
  uint32_t timeout;
};

struct ROS2Config {
  std::vector<TopicConfig> topics;
  std::vector<ServiceConfig> services;
};

class Node : public rclcpp::Node
{
public:
  // Default services timeout in milliseconds
  static constexpr uint32_t DEFAULT_TIMEOUT_MS = 1000;

  Node();

private:
  rclcpp::QoS getQoS(QosConfig config);
  void protonCallback(proton::BundleHandle& bundle);
  void rosCallback(proton::BundleHandle& bundle);
  proton::BundleHandle& serviceCallback(proton::BundleHandle & request);
  void nodeDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void bundleDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string& bundle_name);
  void heartbeatDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string& producer);

  std::string config_file_;
  std::string target_;
  std::unique_ptr<proton::Node> proton_node_;
  ROS2Config ros2_config_;

  rclcpp::TimerBase::SharedPtr proton_timer_;
  std::thread proton_thread_;

  std::map<std::string, std::shared_ptr<IPublisher>> publishers_;
  std::map<std::string, std::shared_ptr<ISubscriber>> subscribers_;
  std::map<std::string, std::shared_ptr<IService>> services_;
  std::map<std::string, std::shared_ptr<IClient>> clients_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_;
};

}



#endif  // INC_PROTON_ROS2_NODE_HPP_