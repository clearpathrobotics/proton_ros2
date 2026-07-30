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

#ifndef PROTON_ROS2_PLUGIN_LOADER_HPP
#define PROTON_ROS2_PLUGIN_LOADER_HPP

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <proton_ros2_interfaces/adaptor_interface.hpp>

#include "rclcpp/rclcpp.hpp"

namespace proton_ros2
{

/**
 * @class PluginLoader
 *
 * Identify and load proton/ROS 2 message bridge plugins
 */
class PluginLoader
{
public:
  using AdaptorPtr = std::shared_ptr<proton_ros2_interfaces::MessageAdaptorIface>;

  explicit PluginLoader(rclcpp::Logger logger);

  virtual ~PluginLoader() = default;

  void load_plugins(const std::vector<std::string> & packages);

  AdaptorPtr get_by_binding(const std::string & binding_name) const;

  std::vector<std::string> bindings() const;

  bool has_binding(const std::string & binding_name) const;

private:
  void register_instance(const std::string & class_id, AdaptorPtr adaptor);

  rclcpp::Logger logger_;

  pluginlib::ClassLoader<proton_ros2_interfaces::MessageAdaptorIface> loader_;
  std::unordered_map<std::string, AdaptorPtr> binding_to_adaptor_;
};

}  // namespace proton_ros2

#endif  // PROTON_ROS2_PLUGIN_LOADER_HPP
