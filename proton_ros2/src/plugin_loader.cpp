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

#include <ranges>
#include <stdexcept>

#include <proton_ros2/plugin_loader.hpp>

namespace proton_ros2
{

PluginLoader::PluginLoader(rclcpp::Logger logger)
: logger_(logger)
  , loader_("proton_ros2_interfaces", "proton_ros2_interfaces::MessageAdaptorIface")
{
}

void PluginLoader::load_plugins(const std::vector<std::string> & packages)
{
  const auto declared_classes = loader_.getDeclaredClasses();

  RCLCPP_INFO(
    logger_,
    "Discovered %zu adaptor classes, filtering from %zu packages",
    declared_classes.size(), packages.size()
  );

  for (const auto & class_id : declared_classes) {
    const auto owner_package = loader_.getClassPackage(class_id);

    if (std::find(packages.begin(), packages.end(), owner_package) != packages.end()) {
      try {
        auto instance = loader_.createSharedInstance(class_id);
        register_instance(class_id, std::move(instance));
      } catch (const pluginlib::PluginlibException & e) {
        RCLCPP_ERROR(
          logger_, "Failed to load '%s' from '%s': %s",
          class_id.c_str(), owner_package.c_str(), e.what()
        );
        throw;
      }
    }
  }

  RCLCPP_INFO(logger_, "Loaded adaptors");
}

void PluginLoader::register_instance(const std::string & class_id, AdaptorPtr adaptor)
{
  const auto binding = adaptor->get_binding_name();

  if (binding.empty()) {
    throw std::runtime_error("Adaptor '" + class_id + "' has empty binding name");
  }

  if (binding_to_adaptor_.contains(binding)) {
    throw std::runtime_error("Duplicate binding '" + binding + "' from class '" + class_id + "'");
  }

  binding_to_adaptor_[binding] = std::move(adaptor);

  RCLCPP_INFO(
    logger_,
    "Registered binding '%s', class '%s', msg_type '%s'",
    binding.c_str(), class_id.c_str(), binding_to_adaptor_[binding]->get_message_type().c_str()
  );
}

PluginLoader::AdaptorPtr PluginLoader::get_by_binding(const std::string & binding_name) const
{
  const auto it = binding_to_adaptor_.find(binding_name);

  if (it == binding_to_adaptor_.end()) {
    RCLCPP_ERROR(logger_, "Could not find binding '%s'", binding_name.c_str());
    return nullptr;
  }
  return it->second;
}

std::vector<std::string> PluginLoader::bindings() const
{
  const auto keys_view = std::views::keys(binding_to_adaptor_);
  std::vector<std::string> bindings{keys_view.begin(), keys_view.end()};

  return bindings;
}

bool PluginLoader::has_binding(const std::string & binding_name) const
{
  return binding_to_adaptor_.contains(binding_name);
}

}  // namespace proton_ros2
