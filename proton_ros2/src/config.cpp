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

#include "proton_ros2/config.hpp"

#include <protoncpp/node_builder/config.hpp>

namespace proton_ros2
{

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

}  // namespace proton_ros2
