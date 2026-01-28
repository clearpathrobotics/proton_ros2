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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import  Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_proton_ros2 = FindPackageShare('proton_ros2')

    arg_namespace = DeclareLaunchArgument(
      'namespace',
      default_value='')

    arg_config_file = DeclareLaunchArgument(
      'config_file',
      default_value=PathJoinSubstitution([pkg_proton_ros2, 'examples/a300/a300.yaml'])
    )

    arg_target = DeclareLaunchArgument(
      'target',
      default_value='pc'
    )

    node_a300_proton_ros2 = Node(
        name='proton_ros2',
        executable='proton_ros2_node',
        package='proton_ros2',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
          {'target': LaunchConfiguration('target')},
          {'config_file': LaunchConfiguration('config_file')},
        ],
        remappings=[
          ('/diagnostics', 'diagnostics')
        ],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(arg_namespace)
    ld.add_action(arg_config_file)
    ld.add_action(arg_target)
    ld.add_action(node_a300_proton_ros2)
    return ld
