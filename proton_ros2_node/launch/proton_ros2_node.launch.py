# Copyright 2026 Rockwell Automation Technologies, Inc., All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (roni.kreinin@rockwellautomation.com)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arg_namespace = DeclareLaunchArgument(
      'namespace',
      default_value='',
      description='Namespace to launch the proton_ros2 node under.',
    )

    arg_proton_config_file = DeclareLaunchArgument(
      'proton_config_file',
      description='Path to the proton message YAML config file.',
    )

    arg_binding_config_file = DeclareLaunchArgument(
      'binding_config_file',
      description='Path to the proton_ros2 binding YAML config file.',
    )

    arg_target = DeclareLaunchArgument(
      'target',
      description='Target node name defined in the proton_ros2 config.',
    )

    proton_ros2_node = Node(
        name='proton_ros2_node',
        executable='proton_ros2_node',
        package='proton_ros2_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
          {'target': LaunchConfiguration('target')},
          {'proton_config_file': LaunchConfiguration('proton_config_file')},
          {'binding_config_file': LaunchConfiguration('binding_config_file')},
        ],
        remappings=[
          ('/diagnostics', 'diagnostics')
        ],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(arg_namespace)
    ld.add_action(arg_proton_config_file)
    ld.add_action(arg_binding_config_file)
    ld.add_action(arg_target)
    ld.add_action(proton_ros2_node)

    return ld
