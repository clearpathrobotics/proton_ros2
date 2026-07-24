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

#ifndef PROTON_ROS2_CONFIG_HPP
#define PROTON_ROS2_CONFIG_HPP

#include <algorithm>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace proton_ros2
{
namespace qos
{

enum class QoSProfile {
  Default,
  SystemDefaults,
  SensorData,
  Services,
  Rosout,
};

inline std::optional<QoSProfile> parseQosProfile(std::string_view s)
{
  static const std::unordered_map<std::string_view, QoSProfile> lookup = {
    {"default", QoSProfile::Default},
    {"system_defaults", QoSProfile::SystemDefaults},
    {"sensor_data", QoSProfile::SensorData},
    {"services", QoSProfile::Services},
    {"rosout", QoSProfile::Rosout},
  };

  auto it = lookup.find(s);
  if (it != lookup.end()) {
    return it->second;
  }

  return std::nullopt;
}

inline rclcpp::QoS getQosProfile(QoSProfile profile)
{
  switch (profile) {
    case QoSProfile::Default:
      return rclcpp::QoS(10);
    case QoSProfile::SystemDefaults:
      return rclcpp::SystemDefaultsQoS();
    case QoSProfile::SensorData:
      return rclcpp::SensorDataQoS();
    case QoSProfile::Services:
      return rclcpp::ServicesQoS();
    case QoSProfile::Rosout:
      return rclcpp::RosoutQoS();
  }
}


enum class QoSHistory {
  SystemDefault,
  KeepLast,
  KeepAll,
};

inline std::optional<QoSHistory> parseQosHistory(std::string_view s)
{
  static const std::unordered_map<std::string_view, QoSHistory> lookup = {
    {"system_default", QoSHistory::SystemDefault},
    {"keep_last", QoSHistory::KeepLast},
    {"keep_all", QoSHistory::KeepAll},
  };

  auto it = lookup.find(s);
  if (it != lookup.end()) {
    return it->second;
  }
  return std::nullopt;
}

inline rmw_qos_history_policy_t getQosHistory(QoSHistory history)
{
  switch (history) {
    case QoSHistory::SystemDefault:
      return RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
    case QoSHistory::KeepLast:
      return RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    case QoSHistory::KeepAll:
      return RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  }
}

enum class QoSReliability {
  SystemDefault,
  Reliable,
  BestEffort,
};

inline std::optional<QoSReliability> parseQosReliability(std::string_view s)
{
  static const std::unordered_map<std::string_view, QoSReliability> lookup = {
    {"system_default", QoSReliability::SystemDefault},
    {"reliable", QoSReliability::Reliable},
    {"best_effort", QoSReliability::BestEffort},
  };

  auto it = lookup.find(s);
  if (it != lookup.end()) {
    return it->second;
  }
  return std::nullopt;
}

inline rmw_qos_reliability_policy_t getQosReliability(QoSReliability reliability)
{
  switch (reliability) {
    case QoSReliability::SystemDefault:
      return RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    case QoSReliability::Reliable:
      return RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    case QoSReliability::BestEffort:
      return RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  }
}

enum class QoSDurability {
  SystemDefault,
  TransientLocal,
  Volatile,
};

inline std::optional<QoSDurability> parseQosDurability(std::string_view s)
{
  static const std::unordered_map<std::string_view, QoSDurability> lookup = {
    {"system_default", QoSDurability::SystemDefault},
    {"transient_local", QoSDurability::TransientLocal},
    {"volatile", QoSDurability::Volatile},
  };

  auto it = lookup.find(s);
  if (it != lookup.end()) {
    return it->second;
  }
  return std::nullopt;
}

inline rmw_qos_durability_policy_t getQosDurability(QoSDurability durability)
{
  switch (durability) {
    case QoSDurability::SystemDefault:
      return RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    case QoSDurability::TransientLocal:
      return RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    case QoSDurability::Volatile:
      return RMW_QOS_POLICY_DURABILITY_VOLATILE;
  }
}
}  // namespace qos

typedef struct
{
  std::string profile;
  std::string history;
  size_t depth;
  std::string reliability;
  std::string durability;
} QosConfig;

struct TopicConfig
{
  std::string topic;
  std::string message;
  std::string bundle;
  QosConfig qos;
};

struct Config
{
  std::vector<TopicConfig> topics;
};

}  // namespace proton_ros2

#endif  // PROTON_ROS2_CONFIG_HPP
